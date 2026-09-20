#!/usr/bin/env python3
"""Test RobotControlNode with synthetic data on localhost-only ROS domain 86.

NO gateway, /cmd_vel publisher, base driver, model, SSH, or robot connection is
created. Small nonzero desired/preview commands exist only in this local test
domain. This is software integration evidence, not a hardware safety test.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time
import uuid


def lease_seconds(value):
    """Validate the test override before importing ROS or creating artifacts."""
    try:
        seconds = float(value)
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError('lease seconds must be a number within (0, 10]') from error
    if not math.isfinite(seconds) or not 0. < seconds <= 10.:
        raise argparse.ArgumentTypeError('lease seconds must be finite and within (0, 10]')
    return seconds


def isolate_environment():
    # Must execute before importing ROS or a module that imports ROS.
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'ROS_DISCOVERY_SERVER', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
                'FASTDDS_ENVIRONMENT_FILE', 'ROS_STATIC_PEERS'):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID='86', ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', SKIP_DEFAULT_XML='1')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output-root', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'outputs/ros2_robot_control_tests')
    parser.add_argument('--lease-seconds', type=lease_seconds, default=1.5,
                        help='Startup arm_lease_sec override in (0, 10]; default 1.5 for quick regression')
    args = parser.parse_args(argv)
    isolate_environment()

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rcl_interfaces.srv import SetParameters
    from geometry_msgs.msg import PointStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from nomad_navigation.robot_control_node import RobotControlNode, TOPICS, SERVICES

    output = args.output_root / ('domain86__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                 + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'domain': 86, 'localhost_only': True, 'hardware_test': False,
              'gateway_created': False, 'checks': [], 'test_lease_sec': args.lease_seconds,
              'created_utc': datetime.now(timezone.utc).isoformat()}
    context = executor = node = control = None
    outputs, states = [], []
    result = 1

    def require(condition, message):
        if not condition:
            raise RuntimeError(message)

    try:
        context = Context()
        rclpy.init(args=[], context=context)
        require(context.get_domain_id() == 86, 'Refusing a context outside isolated domain 86')
        # Construct sequentially, before the valid control node. Its constructor
        # destroys itself on validation failure, so identities never overlap.
        invalid_control = None
        invalid_startup_error = None
        try:
            invalid_control = RobotControlNode(context=context, use_global_arguments=False,
                                               parameter_overrides=[Parameter('arm_lease_sec', value=10.001)])
        except ValueError as error:
            invalid_startup_error = str(error)
        finally:
            if invalid_control is not None:
                invalid_control.prepare_shutdown()
                invalid_control.destroy_node()
        require(invalid_startup_error is not None and 'arm_lease_sec' in invalid_startup_error,
                'A startup arm_lease_sec above 10 seconds was not rejected by the actual ROS node')
        report['invalid_startup_parameter'] = {'arm_lease_sec': 10.001, 'error': invalid_startup_error}
        report['checks'].append('actual_ros_node_rejects_startup_lease_above_ten_seconds')
        node = Node('robot_control_isolated_test_source', context=context, use_global_arguments=False)
        control = RobotControlNode(context=context, use_global_arguments=False,
                                   parameter_overrides=[Parameter('arm_lease_sec', value=args.lease_seconds)])
        require(control.get_parameter('arm_lease_sec').value == args.lease_seconds
                and control.core.limits.arm_lease_sec == args.lease_seconds,
                'Actual ROS parameter/core limits differ from the requested startup lease')
        require(control.describe_parameter('arm_lease_sec').read_only,
                'arm_lease_sec must remain read-only after startup')
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(control)
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        camera = node.create_publisher(CompressedImage, TOPICS['camera'], reliable)
        waypoint = node.create_publisher(PointStamped, TOPICS['waypoint'], reliable)
        odom = node.create_publisher(Odometry, TOPICS['odom'], reliable)
        model = node.create_publisher(String, TOPICS['model'], retained)
        estop = node.create_publisher(Bool, TOPICS['estop'], reliable)
        arm_client = node.create_client(Trigger, SERVICES['arm'])
        disarm_client = node.create_client(Trigger, SERVICES['disarm'])
        parameter_client = node.create_client(SetParameters, '/nomad/robot_control/set_parameters')

        def on_desired(message):
            outputs.append({'mono': time.monotonic(),
                            'stamp_ns': message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                            'frame': message.header.frame_id, 'v': message.twist.linear.x,
                            'w': message.twist.angular.z,
                            'other': [message.twist.linear.y, message.twist.linear.z,
                                      message.twist.angular.x, message.twist.angular.y]})

        def on_state(message):
            states.append({'mono': time.monotonic(), **json.loads(message.data)})

        node.create_subscription(TwistStamped, TOPICS['desired'], on_desired, reliable)
        node.create_subscription(String, TOPICS['status'], on_state, retained)
        streams = {'camera': True, 'waypoint': True, 'odom': True, 'model': 'running',
                   'odom_v': 0.0, 'odom_w': 0.0}
        last_sent = {'camera': 0., 'waypoint': 0., 'odom': 0., 'model': 0.}

        def pump(seconds, until=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                now = time.monotonic()
                stamp = node.get_clock().now().to_msg()
                if streams['camera'] and now - last_sent['camera'] >= .033:
                    image = CompressedImage()
                    image.header.stamp = stamp
                    image.header.frame_id = 'camera_color_optical_frame'
                    image.format = 'jpeg'
                    # Deliberate JPEG envelope only: the local controller does
                    # no pixel decoding. This is NOT valid model-image evidence.
                    image.data = b'\xff\xd8\xff\xd9'
                    camera.publish(image)
                    last_sent['camera'] = now
                if streams['waypoint'] and now - last_sent['waypoint'] >= .25:
                    point = PointStamped()
                    point.header.stamp = stamp
                    point.header.frame_id = 'base_link'
                    point.point.x, point.point.y = .2, .03
                    waypoint.publish(point)
                    last_sent['waypoint'] = now
                if streams['odom'] and now - last_sent['odom'] >= .04:
                    reading = Odometry()
                    reading.header.stamp = stamp
                    reading.header.frame_id = 'odom'
                    reading.child_frame_id = 'base_footprint'
                    reading.twist.twist.linear.x = streams['odom_v']
                    reading.twist.twist.angular.z = streams['odom_w']
                    odom.publish(reading)
                    last_sent['odom'] = now
                if now - last_sent['model'] >= .2:
                    model.publish(String(data=json.dumps({'state': streams['model']})))
                    last_sent['model'] = now
                executor.spin_once(timeout_sec=.005)
                if until is not None and until():
                    return True
            return until is None

        def call(client):
            require(client.service_is_ready(), 'Control service not ready')
            future = client.call_async(Trigger.Request())
            require(pump(2., until=future.done), 'Control service timed out')
            require(future.result() is not None, 'Control service produced no response')
            return future.result()

        def arm_and_wait():
            pump(.35)
            index = len(outputs)
            response = call(arm_client)
            require(response.success, 'Arm refused on fresh stationary data: ' + response.message)
            require(pump(.5, until=lambda: any(row['v'] > 0. for row in outputs[index:])),
                    'Fresh arm never produced a nonzero isolated desired command')
            return index

        def require_zero_tail(index, description):
            require(outputs[index:] and all(row['v'] == 0. and row['w'] == 0. for row in outputs[index:]),
                    description)

        require(pump(4., until=lambda: bool(states) and arm_client.service_is_ready()
                     and disarm_client.service_is_ready() and parameter_client.service_is_ready()
                     and control.core.gate.waypoint is not None
                     and control.core.odom is not None and control.core.gate.camera is not None),
                'Controller failed discovery or did not receive all synthetic inputs')
        pump(.2)
        require(outputs and not control.core.armed, 'Startup must remain disarmed')
        require_zero_tail(0, 'Controller emitted nonzero before explicit arm')
        require(node.count_publishers('/cmd_vel') == 0, 'This isolated test must never create /cmd_vel publisher')
        report['checks'].append('startup_disarmed_fresh_inputs_zero_no_cmd_vel_publisher')

        attempted_lease = 3. if args.lease_seconds == 10. else 10.
        request = SetParameters.Request()
        request.parameters = [Parameter('arm_lease_sec', value=attempted_lease).to_parameter_msg()]
        future = parameter_client.call_async(request)
        require(pump(2., until=future.done), 'Readonly parameter service test timed out')
        response = future.result()
        require(response is not None and len(response.results) == 1 and not response.results[0].successful,
                'Runtime parameter service accepted a change to the read-only arm lease')
        require(control.get_parameter('arm_lease_sec').value == args.lease_seconds
                and control.core.limits.arm_lease_sec == args.lease_seconds,
                'Failed runtime parameter update nevertheless changed the active lease')
        require(states[-1]['limits']['arm_lease_sec'] == args.lease_seconds,
                'Published ROS status does not confirm the requested startup lease')
        report['active_parameter'] = {
            'arm_lease_sec': control.get_parameter('arm_lease_sec').value,
            'core_arm_lease_sec': control.core.limits.arm_lease_sec,
            'published_arm_lease_sec': states[-1]['limits']['arm_lease_sec'],
            'read_only': True, 'attempted_runtime_value': attempted_lease,
            'runtime_update_successful': response.results[0].successful,
            'runtime_update_reason': response.results[0].reason,
        }
        report['checks'].extend(['actual_ros_startup_parameter_matches_requested_lease',
                                 'runtime_readonly_parameter_service_refuses_lease_change'])

        index = arm_and_wait()
        deadline = control.core.arm_deadline
        lease_start = deadline - args.lease_seconds
        response = call(arm_client)
        require(not response.success and control.core.arm_deadline == deadline,
                'Repeated arm extended the active short lease')
        state_index = len(states)
        require(pump(max(0., deadline - time.monotonic()) + .5,
                     until=lambda: any(row['reason'] == 'arm_lease_expired' for row in states[state_index:])),
                'Configured lease did not expire under continuously fresh inputs')
        require(not control.core.armed and control.core.arm_deadline is None, 'Expired lease remained armed')
        expired_state = next(row for row in states[state_index:] if row['reason'] == 'arm_lease_expired')
        require(deadline <= expired_state['mono'] <= deadline + .25,
                'Lease expiry was early or arrived more than 0.25 seconds after its monotonic deadline')
        require(expired_state['linear_x'] == 0. and expired_state['angular_z'] == 0.,
                'Lease expiry status did not confirm zero desired command')
        segment = outputs[index:]
        for current in segment:
            require(current['frame'] == 'base_link' and current['stamp_ns'] > 0, 'Bad desired command header')
            require(all(math.isfinite(value) for value in (current['v'], current['w'])), 'Nonfinite command')
            require(0. <= current['v'] <= .050001 and abs(current['w']) <= .100001,
                    'Command exceeded hard velocity bounds')
            require(abs(current['w']) <= current['v'] + 1e-6, 'Command exceeded the 1 m turning-radius bound')
            require(all(value == 0. for value in current['other']), 'Nonplanar command components')
        for previous, current in zip(segment, segment[1:]):
            dt = (current['stamp_ns'] - previous['stamp_ns']) / 1e9
            require(current['v'] - previous['v'] <= .025 * max(dt, 0.) + 1e-4,
                    'Linear acceleration exceeded 0.025 m/s^2')
        report['checks'].extend(['fresh_arm_nonzero_only_within_short_lease', 'rearm_does_not_extend_lease',
                                 'linear_acceleration_and_ackermann_curvature_bounds', 'lease_expiry_zero'])
        nonzero_segment = [row for row in segment if row['v'] > 0.]
        require(nonzero_segment[-1]['mono'] >= deadline - .15,
                'Fresh input test stopped generating nonzero commands before the final lease ticks')
        report['configured_lease_evidence'] = {
            'requested_seconds': args.lease_seconds, 'start_monotonic': lease_start,
            'deadline_monotonic': deadline, 'expiry_observed_monotonic': expired_state['mono'],
            'elapsed_until_expiry_observed_seconds': expired_state['mono'] - lease_start,
            'expiry_observation_delay_seconds': expired_state['mono'] - deadline,
            'first_nonzero_monotonic': nonzero_segment[0]['mono'],
            'last_nonzero_monotonic': nonzero_segment[-1]['mono'],
            'observed_nonzero_span_seconds': nonzero_segment[-1]['mono'] - nonzero_segment[0]['mono'],
            'nonzero_command_count': len(nonzero_segment),
            'expiry_command': {'linear_x': expired_state['linear_x'], 'angular_z': expired_state['angular_z']},
        }
        index = len(outputs)
        pump(.3)
        require_zero_tail(index, 'Fresh inputs auto-rearmed an expired lease')
        report['checks'].append('lease_expiry_does_not_auto_recover')

        arm_and_wait()
        streams['waypoint'] = False
        state_index = len(states)
        require(pump(1., until=lambda: any(row['reason'] == 'waypoint_timeout' for row in states[state_index:])),
                'Missing waypoint did not cancel arm at the 0.6 second receipt timeout')
        require(not control.core.armed, 'Waypoint timeout left controller armed')
        streams['waypoint'] = True
        last_sent['waypoint'] = 0.
        pump(.1)
        index = len(outputs)
        pump(.4)
        require_zero_tail(index, 'Fresh waypoints auto-rearmed after timeout')
        report['checks'].append('waypoint_receipt_timeout_cancels_arm_without_auto_recovery')

        arm_and_wait()
        state_index = len(states)
        estop.publish(Bool(data=True))
        require(pump(.3, until=lambda: any(row['reason'] == 'emergency_stop' for row in states[state_index:])),
                'Software emergency stop failed to cancel arm')
        index = len(outputs)
        pump(.15)
        require_zero_tail(index, 'Nonzero command while estop was active')
        require(not call(arm_client).success, 'Arm succeeded while estop was active')
        estop.publish(Bool(data=False))
        pump(.4)
        require(not control.core.armed, 'Estop release auto-rearmed controller')
        report['checks'].append('estop_immediate_zero_refuses_arm_and_release_does_not_rearm')

        arm_and_wait()
        state_index = len(states)
        streams['odom_v'] = .13
        last_sent['odom'] = 0.
        require(pump(.3, until=lambda: any(row['reason'] == 'odom_speed_limit' for row in states[state_index:])),
                'Abnormal measured odometry speed did not cancel arm')
        require(not control.core.armed, 'Odometry anomaly left controller armed')
        require(call(disarm_client).success, 'Disarm failed under an active odometry fault')
        streams['odom_v'] = 0.
        pump(.35)
        index = len(outputs)
        pump(.2)
        require_zero_tail(index, 'Odometry recovery auto-rearmed controller')
        report['checks'].append('odom_speed_anomaly_immediate_zero_disarm_always_succeeds')

        arm_and_wait()
        require(call(disarm_client).success, 'Explicit disarm failed')
        pump(.05)
        index = len(outputs)
        pump(.2)
        require_zero_tail(index, 'Explicit disarm did not maintain zero')
        report['checks'].append('explicit_disarm_zero_without_new_arm')

        # Exercise the same final-zero implementation used by main's signal
        # finally block. Continue executing fresh inputs: a cancelled timer may
        # never generate nonzero again after shutdown disarm.
        arm_and_wait()
        control.prepare_shutdown()
        index = len(outputs)
        pump(.25)
        require_zero_tail(index, 'Shutdown zero was absent or followed by nonzero')
        require(not control.core.armed and control._timer.is_canceled(), 'Shutdown failed to cancel timer/arm')
        report['checks'].append('shutdown_cancels_timer_before_final_zero_no_nonzero_regression')
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
                cleanup_errors.append(f'final zero: {error!r}')
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if control is not None:
            control.destroy_node()
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        if cleanup_errors:
            report.update(status='failed', cleanup_errors=cleanup_errors)
            result = 1
        report['outputs'] = outputs
        report['states'] = states
        report['finished_utc'] = datetime.now(timezone.utc).isoformat()
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({'status': report['status'], 'report': str(output / 'report.json')}), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
