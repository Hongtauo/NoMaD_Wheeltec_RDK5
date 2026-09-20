#!/usr/bin/env python3
"""Real ROS callbacks, synthetic data; domain 87 localhost-only, no hardware.

No gateway/base driver, physical SSH, model, or /cmd_vel publisher is created.
Nonzero desired commands are confined to this isolated software-test domain.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import uuid

from verify_operator_guard_isolated import isolate_environment


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--speed', type=float, choices=(.05, .1, .2), default=.05)
    parser.add_argument('--verify-window-expiry', action='store_true')
    parser.add_argument('--output-root', type=Path, default=Path(__file__).resolve().parents[2]
                        / 'outputs/ros2_operator_pause_resume_tests')
    args = parser.parse_args(argv)
    isolate_environment()
    os.environ.update(ROS_DOMAIN_ID='87', NOMAD_OPERATOR_PAUSE_RESUME='1')

    import rclpy
    from geometry_msgs.msg import PointStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from nomad_navigation.operator_guard import OPERATOR_LINK_TOPIC
    from nomad_navigation.robot_control_node import RobotControlNode, TOPICS

    output = args.output_root / ('domain87__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                 + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = dict(status='running', domain=87, localhost_only=True, hardware_test=False,
                  gateway_created=False, cmd_vel_publisher_created=False,
                  max_linear_mps=args.speed, checks=[])
    context = source = control = executor = None
    outputs, states = [], []
    result = 1

    def require(condition, message):
        if not condition:
            raise RuntimeError(message)

    try:
        context = Context()
        rclpy.init(args=[], context=context)
        require(context.get_domain_id() == 87, 'Refusing non-isolated ROS domain')
        source = Node('operator_resume_isolated_source', context=context, use_global_arguments=False)
        control = RobotControlNode(context=context, use_global_arguments=False, parameter_overrides=[
            Parameter('network_pause_resume', value=True),
            Parameter('max_linear_mps', value=args.speed),
            Parameter('max_measured_linear_mps', value={.05: .12, .1: .2, .2: .3}[args.speed])])
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(source)
        executor.add_node(control)
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pubs = dict(camera=source.create_publisher(CompressedImage, TOPICS['camera'], reliable),
                    waypoint=source.create_publisher(PointStamped, TOPICS['waypoint'], reliable),
                    odom=source.create_publisher(Odometry, TOPICS['odom'], reliable),
                    model=source.create_publisher(String, TOPICS['model'], retained),
                    operator=source.create_publisher(String, OPERATOR_LINK_TOPIC, reliable))
        source.create_subscription(TwistStamped, TOPICS['desired'],
            lambda msg: outputs.append(dict(mono=time.monotonic(), v=msg.twist.linear.x,
                                             w=msg.twist.angular.z)), reliable)
        source.create_subscription(String, TOPICS['status'],
            lambda msg: states.append(dict(mono=time.monotonic(), **json.loads(msg.data))), retained)
        streams = dict(heartbeat=True, waypoint=True, generation=1, sequence=0)
        sent = {key: 0. for key in ('camera', 'waypoint', 'odom', 'model', 'heartbeat')}
        counts = {key: 0 for key in sent}
        model_session = str(uuid.uuid4())

        def link(action='heartbeat', **changes):
            streams['sequence'] += 1
            payload = dict(schema=1, session_id=os.environ['NOMAD_OPERATOR_GUARD_SESSION'],
                           generation=streams['generation'], sequence=streams['sequence'],
                           connected=action != 'pause', action=action,
                           deadline_steady_ns=time.monotonic_ns() + 2_000_000_000)
            if action == 'arm':
                payload.update(arm_nonce=str(uuid.uuid4()), mode=1, duration_sec=0.)
            payload.update(changes)
            pubs['operator'].publish(String(data=json.dumps(payload)))
            sent['heartbeat'] = time.monotonic()
            counts['heartbeat'] += 1
            return payload

        def pump(seconds, until=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                now = time.monotonic()
                stamp = source.get_clock().now().to_msg()
                if now - sent['camera'] >= .035:
                    image = CompressedImage()
                    image.header.stamp, image.header.frame_id = stamp, 'camera_color_optical_frame'
                    image.format, image.data = 'jpeg', b'\xff\xd8\xff\xd9'
                    pubs['camera'].publish(image)
                    sent['camera'], counts['camera'] = now, counts['camera'] + 1
                if streams['waypoint'] and now - sent['waypoint'] >= .1:
                    point = PointStamped()
                    point.header.stamp, point.header.frame_id = stamp, 'base_link'
                    point.point.x, point.point.y = .5, .03
                    pubs['waypoint'].publish(point)
                    sent['waypoint'], counts['waypoint'] = now, counts['waypoint'] + 1
                if now - sent['odom'] >= .035:
                    reading = Odometry()
                    reading.header.stamp, reading.header.frame_id = stamp, 'odom'
                    reading.child_frame_id = 'base_footprint'
                    pubs['odom'].publish(reading)
                    sent['odom'], counts['odom'] = now, counts['odom'] + 1
                if now - sent['model'] >= .2:
                    pubs['model'].publish(String(data=json.dumps(dict(state='running',
                        session_id=model_session, network_pause_resume=True, fatal=False))))
                    sent['model'], counts['model'] = now, counts['model'] + 1
                if streams['heartbeat'] and now - sent['heartbeat'] >= .2:
                    link()
                executor.spin_once(timeout_sec=.002)
                if until is not None and until():
                    return True
            return until is None

        def zero_after(index, message):
            require(outputs[index:] and all(row['v'] == row['w'] == 0. for row in outputs[index:]), message)

        def new_generation():
            streams['generation'] += 1
            streams['heartbeat'] = True
            link()
            require(pump(.5, lambda: control.operator_guard.connected
                         and control.operator_guard.generation == streams['generation']),
                    'New generation did not connect')

        def arm(mode=1):
            pump(.15)
            request = link('arm', mode=mode, duration_sec=0. if mode == 1 else 3.)
            require(pump(.5, lambda: control.operator_guard.last_arm_result is not None
                         and control.operator_guard.last_arm_result['arm_nonce'] == request['arm_nonce']),
                    'No arm acknowledgement')
            require(control.operator_guard.last_arm_result['success'],
                    'Explicit arm failed: ' + control.operator_guard.last_arm_result['message'])
            require(pump(.5, lambda: control.core.last_linear > 0.), 'No isolated desired motion')

        require(pump(4., lambda: control.operator_guard.connected and control.core.gate.waypoint is not None
                     and control.core.gate.camera is not None and control.core.odom is not None),
                'Synthetic input/discovery readiness failed')
        pump(.1)
        require(source.count_publishers('/cmd_vel') == 0, 'Forbidden /cmd_vel publisher')
        require(not control.core.armed, 'Heartbeat/fresh data created arm intent')
        zero_after(0, 'Nonzero before explicit arm')
        report['checks'].append('startup_never_arms')
        arm()

        streams['heartbeat'] = False
        deadline = control.operator_guard.deadline_steady_ns
        require(pump(2.4, lambda: control.core.operator_paused and not control.operator_guard.connected),
                'Lease expiry did not pause the isolated controller')
        require(control.core.armed and control.core.last_linear == 0., 'Continuous intent not safely paused')
        report['expiry_observation_delay_sec'] = (time.monotonic_ns() - deadline) / 1e9
        pump(.1)
        index = len(outputs)
        pump(.2)
        zero_after(index, 'Nonzero while operator absent with fresh navigation')
        report['checks'].append('lease_expiry_zero_with_continuous_intent_retained')

        streams['waypoint'] = False
        new_generation()
        index = len(outputs)
        pump(.3)
        zero_after(index, 'Heartbeat alone resumed motion')
        require(control.core.resume_candidates == 0, 'Queued old predictions counted on recovery')
        streams['waypoint'] = True
        index = len(states)
        require(pump(1., lambda: control.core.last_linear > 0.), 'Fresh predictions did not resume retained run')
        require(control.core.resume_candidates == 3 and not control.core.operator_paused,
                'Recovery bypassed fresh three predictions')
        require(all(row['linear_x'] == 0. for row in states[index:]
                    if row['operator_paused'] or row['resume_candidates'] < 3), 'Early nonzero during recovery')
        report['checks'].append('new_generation_requires_fresh_three_and_ramps_from_zero_without_arm_rpc')

        streams['heartbeat'] = False
        link('pause')
        require(pump(.3, lambda: control.core.operator_paused), 'Explicit pause did not stop')
        link('disarm', connected=False)
        require(pump(.3, lambda: not control.core.armed), 'Manual stop did not erase intent')
        new_generation()
        pump(.1)
        index = len(outputs)
        pump(.5)
        zero_after(index, 'Manual stop auto-resumed')
        report['checks'].append('manual_stop_erases_retained_intent')

        arm(mode=0)
        streams['heartbeat'] = False
        link('pause')
        require(pump(.3, lambda: not control.core.armed), 'Timed run retained intent through loss')
        new_generation()
        pump(.1)
        index = len(outputs)
        pump(.4)
        zero_after(index, 'Timed run auto-resumed')
        report['checks'].append('timed_run_is_not_resumed')

        if args.verify_window_expiry:
            arm()
            streams['heartbeat'] = False
            link('pause')
            require(pump(.3, lambda: control.core.operator_paused), 'Window test did not pause')
            expiry = control.core.operator_resume_deadline
            require(pump(30.5, lambda: not control.core.armed), '30-second window did not cancel intent')
            require(time.monotonic() >= expiry, 'Run unexpectedly disarmed before recovery window')
            require(control.core.reason == 'operator_resume_window_expired', 'Unexpected hard stop reason')
            new_generation()
            pump(.1)
            index = len(outputs)
            pump(.5)
            zero_after(index, 'Expired window auto-resumed')
            report['checks'].append('real_monotonic_30_second_window_expires_without_auto_rearm')

        require(all(0. <= row['v'] <= args.speed + 1e-9 and abs(row['w']) <= .100001 for row in outputs),
                'Command exceeds configured limits')
        require(source.count_publishers('/cmd_vel') == 0, 'Forbidden /cmd_vel publisher appeared')
        report.update(status='passed', counts=counts)
        result = 0
    except BaseException as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
    finally:
        if control is not None:
            control.prepare_shutdown()
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if control is not None:
            control.destroy_node()
        if source is not None:
            source.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        report.update(outputs=outputs, states=states, finished_utc=datetime.now(timezone.utc).isoformat())
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(dict(status=report['status'], error=report.get('error'),
                              report=str(output / 'report.json'))), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
