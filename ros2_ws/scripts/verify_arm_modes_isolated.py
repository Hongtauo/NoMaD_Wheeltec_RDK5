#!/usr/bin/env python3
"""Exercise actual timed/continuous arm services on localhost-only ROS domain 86.

Synthetic sensor streams drive the real RobotControlNode. No gateway, /cmd_vel
publisher, hardware driver, SSH, model, or robot connection is created. Nonzero
desired/preview messages stay in this isolated domain. This does not establish
physical safety, obstacle avoidance, or a robot firmware stop guarantee.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time
import uuid


def isolate_environment():
    """Call before any ROS import: never inherit the live robot DDS profile."""
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'ROS_DISCOVERY_SERVER', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
                'FASTDDS_ENVIRONMENT_FILE', 'ROS_STATIC_PEERS',
                'ROS_AUTOMATIC_DISCOVERY_RANGE', 'CYCLONEDDS_URI'):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID='86', ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', SKIP_DEFAULT_XML='1')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output-root', type=Path, default=(
        Path(__file__).resolve().parents[2] / 'outputs/ros2_robot_arm_modes_tests'))
    args = parser.parse_args(argv)
    isolate_environment()

    import rclpy
    from geometry_msgs.msg import PointStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from nomad_interfaces.srv import ArmFor
    from nomad_navigation.robot_control_node import RobotControlNode, TOPICS, SERVICES
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    output = args.output_root / ('domain86__'
                                + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {
        'status': 'running', 'domain': 86, 'localhost_only': True,
        'hardware_test': False, 'gateway_created': False,
        'cmd_vel_publisher_created': False, 'checks': [], 'service_calls': [],
        'timed_runs': [], 'fault_runs': [],
        'created_utc': datetime.now(timezone.utc).isoformat(),
    }
    outputs, states = [], []
    context = executor = source = control = None
    result = 1

    def require(condition, message):
        if not condition:
            raise RuntimeError(message)

    try:
        context = Context()
        rclpy.init(args=[], context=context)
        require(context.get_domain_id() == 86, 'Refusing a context outside isolated domain 86')
        require(ArmFor.Request.TIMED == 0 and ArmFor.Request.CONTINUOUS == 1,
                'Unexpected ArmFor service mode constants')
        source = Node('robot_arm_modes_isolated_test_source', context=context,
                      use_global_arguments=False)
        # No startup override: verify the legacy Trigger interface still defaults
        # to a bounded three-second lease alongside the new per-request modes.
        control = RobotControlNode(context=context, use_global_arguments=False)
        require(control.get_parameter('arm_lease_sec').value == 3.0,
                'Legacy default arm_lease_sec is no longer three seconds')
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(source)
        executor.add_node(control)
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        camera = source.create_publisher(CompressedImage, TOPICS['camera'], reliable)
        waypoint = source.create_publisher(PointStamped, TOPICS['waypoint'], reliable)
        odom = source.create_publisher(Odometry, TOPICS['odom'], reliable)
        model = source.create_publisher(String, TOPICS['model'], retained)
        estop = source.create_publisher(Bool, TOPICS['estop'], reliable)
        arm_for = source.create_client(ArmFor, SERVICES['arm_for'])
        arm = source.create_client(Trigger, SERVICES['arm'])
        disarm = source.create_client(Trigger, SERVICES['disarm'])

        def on_desired(message):
            outputs.append({
                'mono': time.monotonic(),
                'stamp_ns': message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                'frame': message.header.frame_id, 'v': message.twist.linear.x,
                'w': message.twist.angular.z,
                'other': [message.twist.linear.y, message.twist.linear.z,
                          message.twist.angular.x, message.twist.angular.y],
            })

        def on_state(message):
            states.append({'mono': time.monotonic(), **json.loads(message.data)})

        source.create_subscription(TwistStamped, TOPICS['desired'], on_desired, reliable)
        source.create_subscription(String, TOPICS['status'], on_state, retained)
        streams = {'camera': True, 'waypoint': True, 'odom': True,
                   'model': 'running', 'odom_v': 0.0, 'odom_w': 0.0}
        last_sent = {'camera': 0.0, 'waypoint': 0.0, 'odom': 0.0, 'model': 0.0}

        def pump(seconds, until=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                now = time.monotonic()
                stamp = source.get_clock().now().to_msg()
                if streams['camera'] and now - last_sent['camera'] >= .033:
                    message = CompressedImage()
                    message.header.stamp = stamp
                    message.header.frame_id = 'camera_color_optical_frame'
                    message.format = 'jpeg'
                    # Envelope-only data is sufficient for the controller's
                    # header check. It is NOT a valid image/inference test.
                    message.data = b'\xff\xd8\xff\xd9'
                    camera.publish(message)
                    last_sent['camera'] = now
                if streams['waypoint'] and now - last_sent['waypoint'] >= .25:
                    message = PointStamped()
                    message.header.stamp = stamp
                    message.header.frame_id = 'base_link'
                    message.point.x, message.point.y = .2, .03
                    waypoint.publish(message)
                    last_sent['waypoint'] = now
                if streams['odom'] and now - last_sent['odom'] >= .04:
                    message = Odometry()
                    message.header.stamp = stamp
                    message.header.frame_id = 'odom'
                    message.child_frame_id = 'base_footprint'
                    message.twist.twist.linear.x = streams['odom_v']
                    message.twist.twist.angular.z = streams['odom_w']
                    odom.publish(message)
                    last_sent['odom'] = now
                if now - last_sent['model'] >= .2:
                    model.publish(String(data=json.dumps({'state': streams['model']})))
                    last_sent['model'] = now
                executor.spin_once(timeout_sec=.005)
                if until is not None and until():
                    return True
            return until is None

        def call(client, request=None, label=None):
            require(client.service_is_ready(), f'Service not ready: {client.srv_name}')
            request = request if request is not None else Trigger.Request()
            row = {'service': client.srv_name, 'label': label,
                   'started_monotonic': time.monotonic()}
            if isinstance(request, ArmFor.Request):
                # JSON encodes nonfinite request cases as strings, not the
                # non-standard NaN/Infinity tokens, without changing the wire.
                row['request'] = {
                    'mode': request.mode,
                    'duration_sec': (request.duration_sec if math.isfinite(request.duration_sec)
                                     else repr(request.duration_sec)),
                }
            else:
                row['request'] = {}
            report['service_calls'].append(row)
            future = client.call_async(request)
            require(pump(2.0, until=future.done), f'Service timed out: {client.srv_name}')
            response = future.result()
            require(response is not None, 'Service produced no response')
            row.update(completed_monotonic=time.monotonic(), success=response.success,
                       message=response.message)
            return response

        def request_mode(mode, seconds, label=None):
            request = ArmFor.Request()
            request.mode, request.duration_sec = mode, float(seconds)
            return call(arm_for, request, label)

        def zero_tail(index, message):
            require(outputs[index:] and all(row['v'] == 0.0 and row['w'] == 0.0
                                           for row in outputs[index:]), message)

        def begin(mode, seconds, label):
            pump(.35)
            index = len(outputs)
            response = request_mode(mode, seconds, label)
            require(response.success, 'Arm refused on fresh stationary inputs: ' + response.message)
            require(pump(.5, until=lambda: any(row['v'] > 0.0 for row in outputs[index:])),
                    'Successful arm produced no isolated nonzero desired command')
            return index

        def ensure_stays_disarmed(label):
            # Drain in-flight desired/status messages before checking the tail.
            pump(.08)
            index = len(outputs)
            pump(.35)
            zero_tail(index, label + ': fresh inputs auto-resumed nonzero output')
            require(not control.core.armed and states and not states[-1]['armed'],
                    label + ': controller unexpectedly armed')

        def await_reason(reason, state_index, seconds=1.25):
            require(pump(seconds, until=lambda: any(row['reason'] == reason
                                                   for row in states[state_index:])),
                    'Expected stop reason not observed: ' + reason)
            state = next(row for row in states[state_index:] if row['reason'] == reason)
            require(not state['armed'] and state['linear_x'] == 0.0 and state['angular_z'] == 0.0,
                    reason + ': stop status failed to confirm disarmed zero command')
            return state

        require(pump(4.0, until=lambda: bool(states) and all(client.service_is_ready()
                     for client in (arm_for, arm, disarm))
                     and control.core.gate.waypoint is not None
                     and control.core.gate.camera is not None and control.core.odom is not None),
                'DDS discovery or synthetic input reception failed')
        pump(.2)
        require(not control.core.armed, 'Startup must be disarmed')
        zero_tail(0, 'Controller emitted nonzero before explicit arm')
        require(source.count_publishers('/cmd_vel') == 0, 'Isolated domain contains a /cmd_vel publisher')
        report['checks'].append('startup_disarmed_zero_and_no_cmd_vel_publisher')

        invalid_requests = [
            (0, 0.0), (0, -1.0), (0, 10.001), (0, math.nan),
            (0, math.inf), (0, -math.inf),
            (1, .1), (1, -1.0), (1, math.nan), (1, math.inf), (1, -math.inf),
            (2, 0.0), (255, 1.0),
        ]
        for mode, seconds in invalid_requests:
            response = request_mode(mode, seconds, 'invalid_request')
            require(not response.success and not control.core.armed,
                    f'Invalid arm request accepted: mode={mode}, seconds={seconds!r}')
        ensure_stays_disarmed('invalid_requests')
        report['checks'].append('wire_service_rejects_invalid_modes_durations_nonfinite_and_timed_zero')

        for seconds in (1.0, 2.5):
            index = begin(ArmFor.Request.TIMED, seconds, f'timed_{seconds:g}_seconds')
            deadline = control.core.arm_deadline
            require(deadline is not None, 'Timed request created no monotonic deadline')
            armed_call = report['service_calls'][-1]
            lease_start = deadline - seconds
            require(armed_call['started_monotonic'] <= lease_start <= armed_call['completed_monotonic'],
                    'Actual timed deadline does not match the service-request duration')
            for mode, duration in ((0, 10.0), (1, 0.0)):
                response = request_mode(mode, duration, 'reject_renew_or_switch_while_timed')
                require(not response.success and control.core.arm_deadline == deadline,
                        'A timed lease was extended or switched while armed')
            require(not call(arm, label='reject_legacy_renew_while_timed').success,
                    'Legacy arm renewed an active timed lease')
            require(control.core.arm_deadline == deadline, 'Rejected legacy arm changed deadline')
            state_index = len(states)
            stopped = await_reason('arm_lease_expired', state_index,
                                   max(0.0, deadline - time.monotonic()) + .5)
            require(deadline <= stopped['mono'] <= deadline + .15,
                    'Timed expiry was early or over 0.15 seconds late')
            active = [row for row in outputs[index:] if row['v'] > 0.0]
            require(active and active[-1]['mono'] >= deadline - .15,
                    'Timed test stopped before the requested lease ended')
            report['timed_runs'].append({
                'requested_sec': seconds, 'lease_start_monotonic': lease_start,
                'deadline_monotonic': deadline, 'stop_observed_monotonic': stopped['mono'],
                'expiry_observation_delay_sec': stopped['mono'] - deadline,
                'elapsed_until_stop_observed_sec': stopped['mono'] - lease_start,
                'nonzero_count': len(active), 'first_nonzero_monotonic': active[0]['mono'],
                'last_nonzero_monotonic': active[-1]['mono'],
            })
            ensure_stays_disarmed('timed_lease_expiry')
        report['checks'].extend(['per_request_one_and_two_point_five_second_deadlines',
                                 'timed_lease_cannot_renew_or_switch_mode',
                                 'timed_expiry_zero_without_auto_rearm'])

        index = begin(ArmFor.Request.CONTINUOUS, 0.0, 'continuous_over_ten_seconds')
        continuous_start = report['service_calls'][-1]['completed_monotonic']
        require(control.core.arm_deadline is None, 'Continuous mode acquired a timed deadline')
        for mode, duration in ((0, 1.0), (1, 0.0)):
            require(not request_mode(mode, duration, 'reject_renew_or_switch_while_continuous').success,
                    'Mode switch or renewal succeeded while continuously armed')
            require(control.core.armed and control.core.arm_deadline is None,
                    'Rejected request changed active continuous mode')
        require(not call(arm, label='reject_legacy_arm_while_continuous').success,
                'Legacy arm changed continuous mode')
        pump(max(0.0, continuous_start + 10.4 - time.monotonic()))
        continuous_end = time.monotonic()
        require(control.core.armed and control.core.arm_deadline is None and states[-1]['armed'],
                'Fresh continuous mode failed to remain armed past ten seconds')
        recent = [row for row in outputs[index:] if row['mono'] >= continuous_end - 1.0]
        require(len(recent) >= 15 and all(row['v'] > 0.0 for row in recent),
                'Continuous mode stopped producing nonzero commands after ten seconds')
        require(states[-1]['arm_deadline_monotonic'] is None,
                'Published continuous status incorrectly advertises a deadline')
        require(states[-1]['active_mode'] == 'continuous'
                and states[-1]['run_until_disarmed'] is True
                and states[-1]['requested_duration_sec'] == 0.0
                and states[-1]['lease_remaining_sec'] is None,
                'Published status does not unambiguously identify continuous operation')
        response = call(disarm, label='manual_stop_after_continuous_run')
        require(response.success, 'Manual disarm failed in continuous mode')
        report['continuous_run'] = {
            'verified_active_after_sec': continuous_end - continuous_start,
            'deadline_monotonic': None, 'nonzero_count': sum(row['v'] > 0 for row in outputs[index:]),
            'last_second_nonzero_count': len(recent), 'manual_disarm_success': response.success,
        }
        ensure_stays_disarmed('continuous_manual_disarm')
        report['checks'].extend(['continuous_fresh_inputs_remain_active_beyond_ten_seconds',
                                 'continuous_cannot_renew_or_switch_to_timed',
                                 'continuous_manual_disarm_zero_without_auto_rearm'])

        begin(1, 0.0, 'continuous_waypoint_loss')
        streams['waypoint'] = False
        # Drain a waypoint that may already be in the executor queue, then use
        # its actual receipt time instead of the synthetic publisher send time.
        pump(.05)
        last_waypoint_receipt = control.core.gate.waypoint[3]
        state_index = len(states)
        stopped = await_reason('waypoint_timeout', state_index)
        delay = stopped['mono'] - last_waypoint_receipt
        require(.6 <= delay <= .75, 'Waypoint stop exceeded receipt timeout plus tick/observation tolerance')
        require(not control.core.armed, 'Waypoint loss left continuous mode armed')
        report['fault_runs'].append({
            'reason': 'waypoint_timeout', 'configured_receipt_timeout_sec': .6,
            'last_waypoint_receipt_monotonic': last_waypoint_receipt,
            'stop_observed_monotonic': stopped['mono'], 'observed_receipt_to_stop_sec': delay,
            'allowed_tick_and_observation_tolerance_sec': .15,
        })
        streams['waypoint'] = True
        last_sent['waypoint'] = 0.0
        ensure_stays_disarmed('continuous_waypoint_recovery')
        report['checks'].append('continuous_waypoint_timeout_zero_without_auto_recovery')

        begin(1, 0.0, 'continuous_software_estop')
        state_index = len(states)
        estop.publish(Bool(data=True))
        stopped = await_reason('emergency_stop', state_index, .3)
        require(not request_mode(1, 0.0, 'reject_arm_during_estop').success,
                'Continuous arm accepted while software estop was active')
        require(call(disarm, label='disarm_during_estop').success,
                'Disarm did not succeed during active estop')
        estop.publish(Bool(data=False))
        ensure_stays_disarmed('continuous_estop_release')
        report['fault_runs'].append({'reason': 'emergency_stop', 'stop_observed_monotonic': stopped['mono']})
        report['checks'].append('continuous_estop_zero_refuses_arm_and_release_never_rearms')

        begin(1, 0.0, 'continuous_odom_speed_fault')
        state_index = len(states)
        streams['odom_v'], last_sent['odom'] = .13, 0.0
        stopped = await_reason('odom_speed_limit', state_index, .3)
        require(not request_mode(1, 0.0, 'reject_arm_during_odom_fault').success,
                'Continuous arm accepted with abnormal measured odometry speed')
        require(call(disarm, label='disarm_during_odom_fault').success,
                'Disarm did not succeed during odometry fault')
        streams['odom_v'] = 0.0
        ensure_stays_disarmed('continuous_odom_recovery')
        report['fault_runs'].append({'reason': 'odom_speed_limit', 'stop_observed_monotonic': stopped['mono']})
        report['checks'].append('continuous_odom_speed_fault_zero_without_auto_recovery')

        begin(1, 0.0, 'continuous_odom_loss')
        streams['odom'] = False
        state_index = len(states)
        stopped = await_reason('odom_timeout', state_index, .8)
        streams['odom'], last_sent['odom'] = True, 0.0
        ensure_stays_disarmed('continuous_odom_stream_recovery')
        report['fault_runs'].append({'reason': 'odom_timeout', 'stop_observed_monotonic': stopped['mono']})
        report['checks'].append('continuous_odom_timeout_zero_without_auto_recovery')

        begin(1, 0.0, 'continuous_camera_loss')
        streams['camera'] = False
        state_index = len(states)
        # The gate names camera freshness loss camera_timeout in the shared core.
        stopped = await_reason('camera_timeout', state_index, 1.3)
        streams['camera'], last_sent['camera'] = True, 0.0
        ensure_stays_disarmed('continuous_camera_recovery')
        report['fault_runs'].append({'reason': 'camera_timeout', 'stop_observed_monotonic': stopped['mono']})
        report['checks'].append('continuous_camera_timeout_zero_without_auto_recovery')

        begin(1, 0.0, 'continuous_control_timer_stall')
        state_index = len(states)
        # Deliberately pause this isolated single-threaded executor. No robot or
        # gateway exists here; this tests the controller's *resumption* guard,
        # not stopping during a real executor stall (the gateway handles that).
        time.sleep(.26)
        stopped = await_reason('control_timer_gap', state_index, .3)
        ensure_stays_disarmed('continuous_control_timer_recovery')
        report['fault_runs'].append({'reason': 'control_timer_gap', 'stop_observed_monotonic': stopped['mono']})
        report['checks'].append('continuous_timer_resumption_fault_requires_explicit_rearm')

        pump(.35)
        legacy_index = len(outputs)
        response = call(arm, label='legacy_default_three_second_arm')
        require(response.success, 'Legacy Trigger arm refused fresh stationary inputs')
        legacy_deadline = control.core.arm_deadline
        legacy_call = report['service_calls'][-1]
        require(legacy_deadline is not None
                and legacy_call['started_monotonic'] <= legacy_deadline - 3.0
                <= legacy_call['completed_monotonic'],
                'Legacy Trigger arm no longer creates the default three-second deadline')
        state_index = len(states)
        stopped = await_reason('arm_lease_expired', state_index, 3.5)
        require(any(row['v'] > 0 for row in outputs[legacy_index:]),
                'Legacy bounded arm produced no nonzero commands')
        require(legacy_deadline <= stopped['mono'] <= legacy_deadline + .15,
                'Legacy default lease did not expire at three seconds')
        report['legacy_run'] = {'default_duration_sec': 3.0, 'deadline_monotonic': legacy_deadline,
                                'stop_observed_monotonic': stopped['mono']}
        ensure_stays_disarmed('legacy_expiry')
        report['checks'].append('legacy_trigger_default_three_second_lease_remains_bounded')

        begin(1, 0.0, 'continuous_shutdown')
        control.prepare_shutdown()
        index = len(outputs)
        pump(.25)
        zero_tail(index, 'Shutdown zero absent or followed by a nonzero command')
        require(not control.core.armed and control._timer.is_canceled(),
                'Shutdown failed to disarm or cancel the control timer')
        require(source.count_publishers('/cmd_vel') == 0, 'Test unexpectedly created a /cmd_vel publisher')
        report['checks'].append('continuous_shutdown_final_zero_and_cancelled_timer')

        for row in outputs:
            require(row['frame'] == 'base_link' and row['stamp_ns'] > 0, 'Invalid desired command header')
            require(all(math.isfinite(value) for value in (row['v'], row['w'])), 'Nonfinite desired command')
            require(0.0 <= row['v'] <= .050001 and abs(row['w']) <= .100001,
                    'Timed or continuous mode exceeded reviewed speed limits')
            require(abs(row['w']) <= row['v'] + 1e-6, 'Ackermann minimum-turning-radius bound exceeded')
            require(all(value == 0.0 for value in row['other']), 'Nonplanar desired command')
        for previous, row in zip(outputs, outputs[1:]):
            dt = (row['stamp_ns'] - previous['stamp_ns']) / 1e9
            require(row['v'] - previous['v'] <= .025 * max(dt, 0.0) + 1e-4,
                    'Linear acceleration exceeded 0.025 m/s^2')
        report['command_summary'] = {
            'desired_message_count': len(outputs), 'max_linear_mps': max(row['v'] for row in outputs),
            'max_abs_angular_rps': max(abs(row['w']) for row in outputs),
            'motion_output_topic': TOPICS['desired'], 'direct_cmd_vel_publisher_count': 0,
        }
        report['checks'].append('all_modes_retain_speed_acceleration_and_ackermann_curvature_limits')
        report['status'], result = 'passed', 0
    except BaseException as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
        print(report['error'], flush=True)
    finally:
        cleanup_errors = []
        if control is not None:
            try:
                control.prepare_shutdown()
            except Exception as error:
                cleanup_errors.append(f'final zero: {error!r}')
        for name, action in (
                ('executor', lambda: executor.shutdown(timeout_sec=2.0) if executor is not None else None),
                ('control node', lambda: control.destroy_node() if control is not None else None),
                ('source node', lambda: source.destroy_node() if source is not None else None),
                ('context', lambda: context.shutdown() if context is not None and context.ok() else None)):
            try:
                action()
            except Exception as error:
                cleanup_errors.append(f'{name}: {error!r}')
        if cleanup_errors:
            report.update(status='failed', cleanup_errors=cleanup_errors)
            result = 1
        report.update(outputs=outputs, states=states, finished_utc=datetime.now(timezone.utc).isoformat())
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n',
                                          encoding='utf-8')
        print(json.dumps({'status': report['status'], 'report': str(output / 'report.json')}), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
