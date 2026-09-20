#!/usr/bin/env python3
"""Verify reconnectable operator authorization on localhost-only ROS domain 86.

Synthetic camera/odometry/model inputs remain fresh while only operator packets
are interrupted. No gateway, /cmd_vel publisher, base driver, model process, SSH
or physical robot is created. Nonzero desired commands exist ONLY in this test
domain. This is software integration evidence, not hardware stopping evidence.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import uuid


def isolate_environment():
    # Must run before importing anything that imports ROS.
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'ROS_DISCOVERY_SERVER', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
                'FASTDDS_ENVIRONMENT_FILE', 'ROS_STATIC_PEERS'):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID='86', ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', SKIP_DEFAULT_XML='1',
                      NOMAD_OPERATOR_GUARD_SESSION=str(uuid.uuid4()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output-root', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'outputs/ros2_operator_guard_tests')
    parser.add_argument('--speed', type=float, choices=(.05, .1, .2), default=.05,
                        help='Configured speed limit, not a physical motion command')
    args = parser.parse_args(argv)
    isolate_environment()

    import rclpy
    from geometry_msgs.msg import PointStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from nomad_interfaces.srv import ArmFor
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from nomad_navigation.operator_guard import OPERATOR_LINK_TOPIC
    from nomad_navigation.robot_control_node import RobotControlNode, TOPICS, SERVICES

    output = args.output_root / ('domain86__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                 + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = dict(status='running', domain=86, localhost_only=True, hardware_test=False,
                  gateway_created=False, cmd_vel_publisher_created=False,
                  max_linear_mps=args.speed, checks=[],
                  created_utc=datetime.now(timezone.utc).isoformat())
    context = executor = source = control = None
    outputs, states = [], []
    result = 1

    def require(condition, text):
        if not condition:
            raise RuntimeError(text)

    try:
        context = Context()
        rclpy.init(args=[], context=context)
        require(context.get_domain_id() == 86, 'Refusing non-isolated ROS domain')
        source = Node('operator_guard_isolated_source', context=context, use_global_arguments=False)
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
        camera = source.create_publisher(CompressedImage, TOPICS['camera'], reliable)
        waypoint = source.create_publisher(PointStamped, TOPICS['waypoint'], reliable)
        odom = source.create_publisher(Odometry, TOPICS['odom'], reliable)
        model = source.create_publisher(String, TOPICS['model'], retained)
        operator = source.create_publisher(String, OPERATOR_LINK_TOPIC, reliable)
        legacy_arm = source.create_client(ArmFor, SERVICES['arm_for'])

        def on_desired(message):
            outputs.append(dict(mono_ns=time.monotonic_ns(), v=message.twist.linear.x,
                                w=message.twist.angular.z))

        def on_state(message):
            states.append(dict(mono_ns=time.monotonic_ns(), **json.loads(message.data)))

        source.create_subscription(TwistStamped, TOPICS['desired'], on_desired, reliable)
        source.create_subscription(String, TOPICS['status'], on_state, retained)
        streams = dict(heartbeat=True, generation=1, sequence=0)
        last_sent = dict(camera=0., waypoint=0., odom=0., model=0., heartbeat=0.)
        counts = dict(camera=0, waypoint=0, odom=0, model=0, heartbeat=0)
        guard_session = os.environ['NOMAD_OPERATOR_GUARD_SESSION']
        model_session = str(uuid.uuid4())

        def send_link(action='heartbeat', **changes):
            streams['sequence'] += 1
            payload = dict(schema=1, session_id=guard_session, generation=streams['generation'],
                           sequence=streams['sequence'], connected=True,
                           deadline_steady_ns=time.monotonic_ns() + 2_000_000_000, action=action)
            if action == 'arm':
                payload.update(arm_nonce=str(uuid.uuid4()), mode=1, duration_sec=0)
            payload.update(changes)
            operator.publish(String(data=json.dumps(payload)))
            last_sent['heartbeat'] = time.monotonic()
            counts['heartbeat'] += 1
            return payload

        def pump(seconds, until=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                now = time.monotonic()
                stamp = source.get_clock().now().to_msg()
                if now - last_sent['camera'] >= .033:
                    image = CompressedImage()
                    image.header.stamp = stamp
                    image.header.frame_id = 'camera_color_optical_frame'
                    image.format = 'jpeg'
                    # This validates only controller envelopes, NOT model pixels.
                    image.data = b'\xff\xd8\xff\xd9'
                    camera.publish(image)
                    last_sent['camera'] = now
                    counts['camera'] += 1
                if now - last_sent['waypoint'] >= .1:
                    point = PointStamped()
                    point.header.stamp = stamp
                    point.header.frame_id = 'base_link'
                    point.point.x, point.point.y = .5, .03
                    waypoint.publish(point)
                    last_sent['waypoint'] = now
                    counts['waypoint'] += 1
                if now - last_sent['odom'] >= .04:
                    reading = Odometry()
                    reading.header.stamp = stamp
                    reading.header.frame_id = 'odom'
                    reading.child_frame_id = 'base_footprint'
                    odom.publish(reading)
                    last_sent['odom'] = now
                    counts['odom'] += 1
                if now - last_sent['model'] >= .2:
                    model.publish(String(data=json.dumps(dict(state='running', session_id=model_session,
                                                               network_pause_resume=True, fatal=False))))
                    last_sent['model'] = now
                    counts['model'] += 1
                if streams['heartbeat'] and now - last_sent['heartbeat'] >= .2:
                    send_link()
                executor.spin_once(timeout_sec=.003)
                if until is not None and until():
                    return True
            return until is None

        def zero_tail(index, text):
            require(outputs[index:] and all(row['v'] == row['w'] == 0. for row in outputs[index:]), text)

        def new_generation():
            streams['generation'] += 1
            streams['heartbeat'] = True
            send_link()
            require(pump(.5, until=lambda: control.operator_guard.connected
                         and control.operator_guard.generation == streams['generation']),
                    'Fresh generation did not establish operator link')
            pump(.15)

        def fresh_arm():
            pump(.15)
            index = len(outputs)
            payload = send_link('arm')
            require(pump(.5, until=lambda: control.operator_guard.last_arm_result is not None
                         and control.operator_guard.last_arm_result['arm_nonce'] == payload['arm_nonce']),
                    'Nonce-bound arm did not produce acknowledgement')
            ack = control.operator_guard.last_arm_result
            require(ack['success'], 'Fresh explicit arm failed: ' + ack['message'])
            require(pump(.5, until=lambda: any(row['v'] > 0. for row in outputs[index:])),
                    'Fresh explicit arm did not produce isolated nonzero desired command')
            return payload

        require(pump(4., until=lambda: bool(states) and legacy_arm.service_is_ready()
                     and control.operator_guard.connected and control.core.odom is not None
                     and control.core.gate.camera is not None and control.core.gate.waypoint is not None),
                'Discovery or synthetic input readiness failed')
        pump(.2)
        require(not control.core.armed, 'Fresh inputs alone armed the controller')
        zero_tail(0, 'Nonzero before first explicit arm')
        require(source.count_publishers('/cmd_vel') == 0, 'Forbidden /cmd_vel publisher in test domain')
        report['checks'].append('fresh_inputs_and_heartbeat_do_not_arm_no_cmd_vel_publisher')

        first_arm = fresh_arm()
        pump(.25)
        streams['heartbeat'] = False
        lease_deadline_ns = control.operator_guard.deadline_steady_ns
        state_index = len(states)
        sent_before_loss = dict(counts)
        require(pump(2.4, until=lambda: any(row['reason'] == 'operator_link_expired'
                                          for row in states[state_index:])),
                'Missing operator heartbeat did not expire independently of fresh model data')
        expired = next(row for row in states[state_index:] if row['reason'] == 'operator_link_expired')
        delay = (expired['mono_ns'] - lease_deadline_ns) / 1e9
        require(0 <= delay <= .2, 'Lease expiry was not observed within 0.2 seconds of deadline')
        require(not expired['armed'] and not expired['motion_paused']
                and expired['linear_x'] == expired['angular_z'] == 0.,
                'Operator expiry retained motion intent or produced nonzero')
        require(all(counts[key] > sent_before_loss[key] for key in ('camera', 'waypoint', 'odom', 'model')),
                'Synthetic navigation streams stopped during operator interruption')
        report['expiry_observation_delay_sec'] = delay
        report['checks'].append('operator_expiry_disarms_with_fresh_navigation_no_network_auto_resume')
        index = len(outputs)
        pump(.2)
        zero_tail(index, 'Motion continued after operator expiry')

        new_generation()
        index = len(outputs)
        pump(.5)
        zero_tail(index, 'Recovered operator heartbeat auto-armed robot')
        require(not control.core.armed, 'Recovered link retained old arm intent')
        report['checks'].append('new_generation_reconnect_remains_disarmed_and_zero')

        # The old ROS service can arrive arbitrarily late; guard mode refuses it.
        request = ArmFor.Request()
        request.mode, request.duration_sec = 1, 0.
        future = legacy_arm.call_async(request)
        require(pump(.7, until=future.done), 'Legacy delayed-arm service did not return')
        require(future.result() is not None and not future.result().success
                and not control.core.armed, 'Legacy arm bypassed the guarded nonce/generation path')
        report['checks'].append('delayed_direct_arm_for_service_cannot_bypass_guard')

        streams['heartbeat'] = False
        send_link('arm', generation=first_arm['generation'], arm_nonce=str(uuid.uuid4()))
        require(pump(.3, until=lambda: control.operator_guard.requires_new_generation),
                'Old-generation delayed arm was not rejected')
        require(not control.core.armed, 'Old-generation delayed arm regained motion')
        new_generation()
        streams['heartbeat'] = False
        send_link('arm', arm_nonce=first_arm['arm_nonce'])
        require(pump(.3, until=lambda: control.operator_guard.reason == 'operator_link_replayed_arm'),
                'Old arm nonce was not rejected after reconnection')
        require(not control.core.armed, 'Repeated arm nonce regained motion')
        report['checks'].append('old_generation_and_old_confirmation_nonce_rejected')

        new_generation()
        index = len(outputs)
        pump(.2)
        zero_tail(index, 'Recovery from rejected replay caused motion')
        fresh_arm()
        report['checks'].append('new_explicit_nonce_after_reconnection_can_arm_with_fresh_inputs')

        streams['heartbeat'] = False
        disconnected_ns = time.monotonic_ns()
        send_link(connected=False, deadline_steady_ns=0)
        require(pump(.2, until=lambda: not control.core.armed
                     and control.operator_guard.reason == 'operator_link_lost'),
                'Explicit disconnect did not cancel authorization promptly')
        report['disconnect_observation_delay_sec'] = (time.monotonic_ns() - disconnected_ns) / 1e9
        pump(.04)
        index = len(outputs)
        pump(.2)
        zero_tail(index, 'Explicit disconnect retained nonzero commands')
        require(not control.core.motion_paused, 'Disconnect retained automatic resume intent')
        new_generation()
        index = len(outputs)
        pump(.3)
        zero_tail(index, 'Explicit disconnect recovery auto-armed robot')
        report['checks'].append('explicit_disconnect_zero_and_reconnect_requires_new_run')
        require(all(0. <= row['v'] <= args.speed + 1e-6 and abs(row['w']) <= .100001
                    for row in outputs), 'Desired command exceeded configured limits')
        require(source.count_publishers('/cmd_vel') == 0, 'A /cmd_vel publisher appeared during test')
        report['synthetic_input_counts'] = counts
        report['status'] = 'passed'
        result = 0
    except BaseException as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
        print(report['error'], flush=True)
    finally:
        cleanup_errors = []
        if control is not None:
            try:
                control.prepare_shutdown()
            except Exception as error:
                cleanup_errors.append(repr(error))
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if control is not None:
            control.destroy_node()
        if source is not None:
            source.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        if cleanup_errors:
            report.update(status='failed', cleanup_errors=cleanup_errors)
            result = 1
        report.update(outputs=outputs, states=states, finished_utc=datetime.now(timezone.utc).isoformat())
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(dict(status=report['status'], report=str(output / 'report.json'))), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
