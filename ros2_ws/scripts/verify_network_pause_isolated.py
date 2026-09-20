#!/usr/bin/env python3
"""Actual ROS controller on localhost domain 86, synthetic inputs, NO hardware/gateway.

No SSH, model, /cmd_vel publisher or physical movement. Run domain-86 tests sequentially.
"""
import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import time
import uuid


PROFILES = {.05: .12, .1: .2, .2: .3}
ACCELERATION_MPS2 = .025
MAX_TICK_GAP_SEC = .2


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--max-linear-mps', type=float, choices=tuple(PROFILES), default=.05)
    parser.add_argument('--output-root', type=Path, default=(
        Path(__file__).resolve().parents[2] / 'outputs/campus_api'))
    return parser.parse_args(argv)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def match_publication(publications, index, speed, angular):
    """Associate ordered reliable ROS output with the passively observed call."""
    require(index < len(publications), 'Received a desired command without an observed publication')
    publication = publications[index]
    require(publication['v'] == speed and publication['w'] == angular,
            'ROS desired command does not match observed publication order/value')
    return publication['control_steady']


def ramp_summary(samples, max_linear_mps, started):
    """Check actual ROS outputs against their controller's exact steady clock.

    ROS header stamps are assigned later in _publish and subscriber receipt
    clocks include callback/transport jitter. Neither is the clock that controls
    acceleration. Passive tracing retains the actual unmodified core.tick
    argument and associates every ROS message with its publication in order.
    """
    require(max_linear_mps in PROFILES, 'Unsupported verification profile')
    require(len(samples) >= 5, 'Insufficient desired-command samples')
    for row in samples:
        require(all(math.isfinite(row[key]) for key in ('v', 'w', 't', 'stamp_ns')),
                'Nonfinite desired command')
        require(0. <= row['v'] <= max_linear_mps and abs(row['w']) <= .1,
                'Desired command exceeded its speed profile')
    active = [row for row in samples if row['v'] > 0.]
    require(active, 'Ramp never produced nonzero desired command')
    require(active[0]['v'] <= ACCELERATION_MPS2 * MAX_TICK_GAP_SEC + 1e-9,
            'Ramp did not restart from zero')
    require(all(isinstance(row.get('control_steady'), (int, float))
                and math.isfinite(row['control_steady']) for row in active),
            'Nonzero ROS output is missing its controller steady timestamp')
    for previous, current in zip(active, active[1:]):
        require(current['stamp_ns'] > previous['stamp_ns'], 'Desired command stamps did not increase')
        elapsed = current['control_steady'] - previous['control_steady']
        require(0. < elapsed <= MAX_TICK_GAP_SEC,
                'Nonzero control tick gap exceeded the unchanged timer limit')
        require(current['v'] - previous['v'] <= ACCELERATION_MPS2 * elapsed + 1e-12,
                'Desired command exceeded the unchanged acceleration limit')
    at_cap = [row for row in active if abs(row['v'] - max_linear_mps) <= 1e-9]
    require(len(at_cap) >= 5, 'Selected speed cap was not sustained for five samples')
    first_at_cap_sec = at_cap[0]['t'] - started
    nominal_ramp_sec = max_linear_mps / ACCELERATION_MPS2
    require(nominal_ramp_sec - .3 <= first_at_cap_sec <= nominal_ramp_sec + .5,
            'Observed ramp arrival is inconsistent with 0.025 m/s^2 acceleration')
    return dict(max_linear_mps=max_linear_mps, nominal_ramp_sec=nominal_ramp_sec,
                first_nonzero_v=active[0]['v'], first_at_cap_sec=first_at_cap_sec,
                at_cap_output_count=len(at_cap), peak_v=max(row['v'] for row in active))


def main(argv=None):
    args = parse_args(argv)
    from verify_fast_control_isolated import isolate_environment
    isolate_environment()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PointStamped, TwistStamped
    from sensor_msgs.msg import CompressedImage
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    from std_srvs.srv import Trigger
    from nomad_interfaces.srv import ArmFor
    from nomad_navigation.robot_control_node import RobotControlNode, TOPICS, SERVICES

    output = args.output_root / (
        'pause_isolated__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = dict(domain=86, localhost_only=True, hardware=False, gateway=False,
        cmd_vel_publisher_created=False, physics_simulation=False, model_loaded=False,
        max_linear_mps=args.max_linear_mps, max_measured_linear_mps=PROFILES[args.max_linear_mps],
        network_pause_resume=True, linear_acceleration_mps2=ACCELERATION_MPS2,
        max_angular_rps=.1, max_tick_gap_sec=MAX_TICK_GAP_SEC, checks=[], checkpoints=[], status='running')
    context = Context()
    executor = source = control = None
    rows, tick_rows, publications = [], [], []

    def checkpoint(label):
        state = dict(label=label, monotonic=time.monotonic(), desired_row_count=len(rows))
        if control is not None:
            state.update(control.core.snapshot(state['monotonic']))
            state.update(last_waypoint_rejection=control.core.gate.last_rejection,
                         odom_rejection=control.core.odom_rejection,
                         odom=control.core.odom,
                         waypoint=control.core.gate.waypoint,
                         resume_after_stamp_ns=control.core.resume_after_stamp)
        if rows:
            state['last_desired'] = rows[-1]
        report['checkpoints'].append(state)

    try:
        rclpy.init(args=[], context=context)
        require(context.get_domain_id() == 86, 'Refusing context outside localhost domain 86')
        executor = SingleThreadedExecutor(context=context)
        source = Node('network_pause_test_source', context=context, use_global_arguments=False)
        executor.add_node(source)
        for _ in range(50):
            executor.spin_once(timeout_sec=.02)
        for topic in ('/cmd_vel', TOPICS['desired'], TOPICS['waypoint'], TOPICS['odom']):
            require(source.count_publishers(topic) == 0, 'Existing isolated test publisher')
        control = RobotControlNode(context=context, use_global_arguments=False,
            parameter_overrides=[Parameter('network_pause_resume', value=True),
                Parameter('max_linear_mps', value=args.max_linear_mps),
                Parameter('max_measured_linear_mps', value=PROFILES[args.max_linear_mps])])
        executor.add_node(control)
        require(control.describe_parameter('network_pause_resume').read_only,
                'Network-pause option must remain startup-only')
        require(control.core.limits.max_linear_mps == args.max_linear_mps
                and control.core.limits.max_measured_linear_mps == PROFILES[args.max_linear_mps],
                'Controller did not apply selected speed profile')
        require(control.core.limits.linear_acceleration_mps2 == ACCELERATION_MPS2
                and control.core.limits.max_angular_rps == .1
                and control.core.limits.max_tick_gap_sec == MAX_TICK_GAP_SEC,
                'Speed profile relaxed acceleration, turning, or timer-gap protection')
        # Observe, but never substitute, production tick arguments/results and
        # publication values. The core uses monotonic time for acceleration;
        # ROS header stamps are sampled later and are not interchangeable.
        original_tick = control.core.tick
        original_publish = control._publish

        def observe_tick(now, steady):
            result = original_tick(now, steady)
            tick_rows.append(dict(ros_ns=now, control_steady=steady,
                                  v=result[0], w=result[1], reason=result[2]))
            return result

        def observe_publish(speed, angular):
            tick = None
            if speed != 0. or angular != 0.:
                require(tick_rows and tick_rows[-1]['v'] == speed and tick_rows[-1]['w'] == angular,
                        'Nonzero publication did not correspond to the real controller tick')
                tick = tick_rows[-1]['control_steady']
            publications.append(dict(v=speed, w=angular, control_steady=tick))
            return original_publish(speed, angular)

        control.core.tick = observe_tick
        control._publish = observe_publish
        camera = source.create_publisher(CompressedImage, TOPICS['camera'], 10)
        odom = source.create_publisher(Odometry, TOPICS['odom'], 10)
        waypoint = source.create_publisher(PointStamped, TOPICS['waypoint'], 10)
        model = source.create_publisher(String, TOPICS['model'], QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        estop = source.create_publisher(Bool, TOPICS['estop'], 10)
        def record_desired(message):
            speed, angular = message.twist.linear.x, message.twist.angular.z
            steady = match_publication(publications, len(rows), speed, angular)
            rows.append(dict(t=time.monotonic(), v=speed, w=angular, control_steady=steady,
                stamp_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec))

        source.create_subscription(TwistStamped, TOPICS['desired'], record_desired, 100)
        arm = source.create_client(ArmFor, SERVICES['arm_for'])
        stop = source.create_client(Trigger, SERVICES['disarm'])
        streams = dict(camera=True, waypoint=True, model='running', session=str(uuid.uuid4()), odom_v=0.)
        sent = dict(sensor=0., waypoint=0., model=0.)

        def publish_waypoint():
            p = PointStamped()
            p.header.stamp, p.header.frame_id = source.get_clock().now().to_msg(), 'base_link'
            p.point.x, p.point.y = .2, 0.  # Straight target must permit all three speed caps.
            waypoint.publish(p)
            sent['waypoint'] = time.monotonic()

        def pump(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                now = time.monotonic()
                stamp = source.get_clock().now().to_msg()
                if now-sent['sensor'] >= .035:
                    if streams['camera']:
                        m = CompressedImage()
                        m.header.stamp, m.header.frame_id = stamp, 'camera_color_optical_frame'
                        m.format, m.data = 'jpeg', b'\xff\xd8\xff\xd9'
                        camera.publish(m)
                    o = Odometry()
                    o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
                    o.twist.twist.linear.x = streams['odom_v']
                    odom.publish(o)  # Explicit synthetic measurement, not physical dynamics.
                    sent['sensor'] = now
                if streams['waypoint'] and now-sent['waypoint'] >= .25:
                    publish_waypoint()
                if now-sent['model'] >= .1:
                    model.publish(String(data=json.dumps(dict(state=streams['model'],
                        session_id=streams['session'], network_pause_resume=True, fatal=False))))
                    sent['model'] = now
                executor.spin_once(timeout_sec=.002)

        def call(client, request):
            require(client.service_is_ready(), 'Isolated control service is not ready')
            future = client.call_async(request)
            deadline = time.monotonic()+2.
            while not future.done() and time.monotonic() < deadline:
                pump(.025)
            require(future.done() and future.result() is not None and future.result().success,
                    'Isolated control service failed')

        def arm_continuous():
            request = ArmFor.Request()
            request.mode, request.duration_sec = 1, 0.
            call(arm, request)

        def assert_zero_since(mark):
            sampled = [row for row in rows if row['t'] > mark]
            require(len(sampled) >= 5 and all(row['v'] == row['w'] == 0. for row in sampled),
                    'Expected sustained zero desired commands')

        pump(1.5)
        index = len(rows)
        started = time.monotonic()
        arm_continuous()
        pump(args.max_linear_mps / ACCELERATION_MPS2 + 1.)
        report['initial_ramp'] = ramp_summary(rows[index:], args.max_linear_mps, started)
        checkpoint('initial_ramp_complete')
        require(control.core.armed, 'Full-speed continuous ramp unexpectedly disarmed')
        report['checks'].append('explicit_continuous_service_ramps_to_selected_cap')
        streams.update(model='network_wait', waypoint=False)
        sent['model'] = 0.
        pause_started = time.monotonic()
        index = len(rows)
        pump(.2)
        zero = next((row for row in rows[index:] if row['v'] == row['w'] == 0.), None)
        require(zero is not None and zero['t'] - pause_started <= .2,
                'Network wait did not immediately produce zero within 200 ms observation bound')
        report['network_wait_to_zero_sec'] = zero['t'] - pause_started
        mark = time.monotonic()
        pump(2.)
        checkpoint('network_wait_held_zero')
        assert_zero_since(mark)
        require(control.core.armed and control.core.motion_paused, 'Network wait lost pause intent')
        report['checks'].append('network_wait_continues_zero_commands_without_disarming_intent')
        # Deliver exactly one and two fresh predictions: neither may resume.
        # After the third, a nonstationary (but below-fault-limit) odometer must
        # still prevent resume. This is a guard test, not a chassis simulation.
        streams.update(model='running', waypoint=False, odom_v=0.)
        sent['model'] = 0.
        pump(.1)
        checkpoint('model_running_before_first_fresh_prediction')
        recovery_counts = []
        for expected in (1, 2, 3):
            if expected == 3:
                streams['odom_v'] = .02
                pump(.1)  # Deliver moving odometry before the third fresh prediction.
            mark = time.monotonic()
            publish_waypoint()
            pump(.15)
            checkpoint(f'fresh_prediction_{expected}')
            require(control.core.armed and control.core.motion_paused,
                    'Controller disarmed or resumed before three predictions and stationary odometry')
            require(control.core.resume_candidates == expected,
                    'Fresh post-pause candidate count mismatch')
            require(all(row['v'] == row['w'] == 0. for row in rows if row['t'] > mark),
                    'Prediction or moving odometry allowed premature resume')
            recovery_counts.append(control.core.resume_candidates)
        report['fresh_prediction_counts_before_stationary'] = recovery_counts
        streams.update(waypoint=True, odom_v=0.)
        index = len(rows)
        started = time.monotonic()
        pump(args.max_linear_mps / ACCELERATION_MPS2 + 1.)
        require(control.core.armed and not control.core.motion_paused and rows[-1]['v'] > 0.,
                'Fresh post-pause predictions and stationary odometry did not resume')
        report['recovery_ramp'] = ramp_summary(rows[index:], args.max_linear_mps, started)
        checkpoint('recovery_ramp_complete')
        report['checks'].append('three_fresh_predictions_plus_stationary_resume_from_zero_to_selected_cap')
        streams.update(camera=False, waypoint=False)
        pump(1.2)
        mark = time.monotonic()
        pump(.4)
        assert_zero_since(mark)
        require(control.core.armed and control.core.motion_paused, 'Source timeout did not retain pause intent')
        streams.update(camera=True, waypoint=True)
        pump(1.2)
        require(not control.core.motion_paused and rows[-1]['v'] > 0., 'Fresh source did not resume')
        report['checks'].append('local_source_timeout_also_pauses_then_recovers')
        streams.update(model='network_wait', waypoint=False)
        pump(.3)
        call(stop, Trigger.Request())
        streams.update(model='running', waypoint=True)
        mark = time.monotonic()
        pump(1.2)
        assert_zero_since(mark)
        require(not control.core.armed, 'Manual stop restored arm intent')
        report['checks'].append('manual_disarm_cancels_resume_intent')
        arm_continuous()
        streams.update(model='network_wait', waypoint=False)
        pump(.3)
        estop.publish(Bool(data=True))
        pump(.2)
        estop.publish(Bool(data=False))
        streams.update(model='running', waypoint=True)
        mark = time.monotonic()
        pump(1.2)
        assert_zero_since(mark)
        require(not control.core.armed, 'Estop release restored arm intent')
        report['checks'].append('estop_release_does_not_restore_resume_intent')
        arm_continuous()
        streams['session'] = str(uuid.uuid4())
        pump(.2)
        mark = time.monotonic()
        pump(.5)
        assert_zero_since(mark)
        require(not control.core.armed, 'Changed model session restored arm intent')
        report['checks'].append('changed_model_session_cannot_inherit_motion_intent')
        require(source.count_publishers('/cmd_vel') == 0, 'Forbidden /cmd_vel publisher in isolated test')
        report['status'] = 'passed'
    except Exception as error:
        checkpoint('failure')
        report.update(status='failed', error=repr(error))
        raise
    finally:
        if control is not None:
            control.prepare_shutdown()
            control.destroy_node()
        if source is not None:
            source.destroy_node()
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if context.ok():
            context.shutdown()
        report['desired_rows'] = rows
        report['controller_ticks'] = tick_rows
        report['observed_publications'] = publications
        (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(dict(status=report['status'], report=str(output/'report.json'))), flush=True)


if __name__ == '__main__':
    main()
