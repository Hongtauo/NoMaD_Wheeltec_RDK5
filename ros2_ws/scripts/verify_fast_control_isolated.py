#!/usr/bin/env python3
"""Verify the explicit 0.2 m/s ROS controller/gateway chain on domain 86 only.

The real controller and a separate real gateway exchange ROS messages on
localhost. Synthetic odometry follows the observed isolated /cmd_vel output;
this is NOT a physical dynamics, camera-decoding, model, obstacle, or hardware
test. No robot connection, hardware driver, SSH, or live DDS profile is used.
Run sequentially, without another domain-86 test. The script never resets a
gateway fault automatically; a fresh test owns its own gateway process.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def isolate_environment():
    # Must precede every ROS import and subprocess creation.
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
        Path(__file__).resolve().parents[2] / 'outputs/ros2_robot_speed_tests'))
    args = parser.parse_args(argv)
    isolate_environment()

    import rclpy
    from geometry_msgs.msg import PointStamped, Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from nomad_interfaces.srv import ArmFor
    from nomad_navigation.robot_control_node import RobotControlNode, TOPICS, SERVICES
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    output = args.output_root / ('domain86__'
                                + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {
        'status': 'running', 'domain': 86, 'localhost_only': True,
        'hardware_test': False, 'physics_simulation': False, 'model_loaded': False,
        'gateway_created': True, 'cmd_vel_isolated_domain_only': True,
        'max_linear_mps': .2, 'max_measured_linear_mps': .3,
        'linear_acceleration_mps2': .025, 'max_angular_rps': .1,
        'checks': [], 'service_calls': [],
        'created_utc': datetime.now(timezone.utc).isoformat(),
    }
    desired_rows, cmd_rows, control_states, gateway_states = [], [], [], []
    context = executor = source = control = child = gateway_log = None
    previous_signals = {}
    result = 1

    def require(condition, message):
        if not condition:
            raise RuntimeError(message)

    def on_signal(signum, frame):
        raise KeyboardInterrupt(f'Interrupted by signal {signum}')

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_signals[signum] = signal.signal(signum, on_signal)
        context = Context()
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        require(context.get_domain_id() == 86, 'Refusing context outside isolated domain 86')
        source = Node('fast_control_isolated_test_source', context=context,
                      use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(source)
        # Give the graph time to expose an already-running test before creating
        # any publisher which could conflict with it.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        for topic in ('/cmd_vel', TOPICS['desired'], TOPICS['waypoint'], TOPICS['odom']):
            require(source.count_publishers(topic) == 0,
                    f'Existing domain-86 publisher on {topic}; run tests sequentially')

        control = RobotControlNode(
            context=context, use_global_arguments=False,
            parameter_overrides=[Parameter('max_linear_mps', value=.2),
                                 Parameter('max_measured_linear_mps', value=.3)])
        executor.add_node(control)
        require(control.core.limits.max_linear_mps == .2
                and control.core.limits.max_measured_linear_mps == .3,
                'Controller did not apply the explicit 0.2/0.3 startup profile')
        require(control.core.limits.linear_acceleration_mps2 == .025
                and control.core.limits.max_angular_rps == .1
                and control.core.limits.min_turning_radius_m == 1.0,
                'Faster profile relaxed an unrelated motion protection')
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        camera = source.create_publisher(CompressedImage, TOPICS['camera'], reliable)
        waypoint = source.create_publisher(PointStamped, TOPICS['waypoint'], reliable)
        odom = source.create_publisher(Odometry, TOPICS['odom'], reliable)
        model = source.create_publisher(String, TOPICS['model'], retained)
        arm_for = source.create_client(ArmFor, SERVICES['arm_for'])
        disarm = source.create_client(Trigger, SERVICES['disarm'])

        def twist_row(twist):
            return {'mono': time.monotonic(), 'v': twist.linear.x, 'w': twist.angular.z,
                    'other': [twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y]}

        def on_desired(message):
            row = twist_row(message.twist)
            row.update(stamp_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                       frame=message.header.frame_id)
            desired_rows.append(row)

        def on_cmd(message):
            cmd_rows.append(twist_row(message))

        def on_control_state(message):
            control_states.append({'mono': time.monotonic(), **json.loads(message.data)})

        def on_gateway_state(message):
            gateway_states.append({'mono': time.monotonic(), **json.loads(message.data)})

        source.create_subscription(TwistStamped, TOPICS['desired'], on_desired, reliable)
        source.create_subscription(Twist, '/cmd_vel', on_cmd, reliable)
        source.create_subscription(String, TOPICS['status'], on_control_state, retained)
        source.create_subscription(String, '/nomad/robot_gateway/status', on_gateway_state, reliable)
        streams = {'camera': True, 'waypoint': True, 'odom': True}
        last_sent = {key: 0.0 for key in ('camera', 'waypoint', 'odom', 'model')}

        def pump(seconds, until=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if child is not None:
                    require(child.poll() is None, 'Gateway exited before verification completed')
                now = time.monotonic()
                stamp = source.get_clock().now().to_msg()
                if streams['camera'] and now - last_sent['camera'] >= .033:
                    message = CompressedImage()
                    message.header.stamp = stamp
                    message.header.frame_id = 'camera_color_optical_frame'
                    message.format = 'jpeg'
                    message.data = b'\xff\xd8\xff\xd9'  # Header guard only, not a decoded image.
                    camera.publish(message)
                    last_sent['camera'] = now
                if streams['waypoint'] and now - last_sent['waypoint'] >= .25:
                    message = PointStamped()
                    message.header.stamp = stamp
                    message.header.frame_id = 'base_link'
                    message.point.x, message.point.y = .2, .1
                    waypoint.publish(message)
                    last_sent['waypoint'] = now
                if streams['odom'] and now - last_sent['odom'] >= .04:
                    message = Odometry()
                    message.header.stamp = stamp
                    message.header.frame_id = 'odom'
                    message.child_frame_id = 'base_footprint'
                    # Synthetic instantaneous feedback, no hardware or dynamics.
                    if cmd_rows:
                        message.twist.twist.linear.x = cmd_rows[-1]['v']
                        message.twist.twist.angular.z = cmd_rows[-1]['w']
                    odom.publish(message)
                    last_sent['odom'] = now
                if now - last_sent['model'] >= .2:
                    model.publish(String(data=json.dumps({'state': 'running'})))
                    last_sent['model'] = now
                executor.spin_once(timeout_sec=.005)
                if until is not None and until():
                    return True
            return until is None

        def call(client, request, label):
            require(client.service_is_ready(), f'Service not ready: {client.srv_name}')
            row = {'label': label, 'service': client.srv_name,
                   'started_monotonic': time.monotonic()}
            if isinstance(request, ArmFor.Request):
                row['request'] = {'mode': request.mode, 'duration_sec': request.duration_sec}
            report['service_calls'].append(row)
            future = client.call_async(request)
            require(pump(2., until=future.done), f'Service timed out: {client.srv_name}')
            response = future.result()
            require(response is not None, 'Service returned no response')
            row.update(completed_monotonic=time.monotonic(), success=response.success,
                       message=response.message)
            require(response.success, f'{label}: service refused: {response.message}')
            return row

        def begin(label, require_forward=True):
            pump(.4)
            require(not control.core.armed, 'Previous phase did not leave controller disarmed')
            request = ArmFor.Request()
            request.mode, request.duration_sec = ArmFor.Request.CONTINUOUS, 0.0
            index = len(cmd_rows)
            row = call(arm_for, request, label)
            if require_forward:
                require(pump(.7, until=lambda: any(item['v'] > 0. for item in cmd_rows[index:])),
                        label + ': controller command did not reach the real gateway output')
            return index, row

        def zero_tail(rows, index, label):
            tail = rows[index:]
            require(tail and all(row['v'] == 0. and row['w'] == 0. for row in tail), label)

        def require_stays_stopped(label):
            pump(.2)  # Drain zero/status transitions already in flight.
            desired_index, cmd_index = len(desired_rows), len(cmd_rows)
            pump(.5)
            zero_tail(desired_rows, desired_index, label + ': desired command auto-resumed')
            zero_tail(cmd_rows, cmd_index, label + ': gateway output auto-resumed')
            require(not control.core.armed and control_states and not control_states[-1]['armed'],
                    label + ': controller unexpectedly re-armed')

        gateway = (Path(__file__).resolve().parents[1]
                   / 'src/nomad_navigation/nomad_navigation/robot_gateway.py')
        gateway_log = (output / 'gateway.log').open('w', encoding='utf-8')
        child = subprocess.Popen([sys.executable, '-u', str(gateway), '--max-linear-mps', '0.2'],
                                 env=dict(os.environ), stdout=gateway_log,
                                 stderr=subprocess.STDOUT, text=True, start_new_session=True)
        require(pump(5., until=lambda: bool(gateway_states) and bool(control_states)
                     and bool(cmd_rows) and arm_for.service_is_ready() and disarm.service_is_ready()
                     and control.core.gate.waypoint is not None
                     and control.core.gate.camera is not None and control.core.odom is not None),
                'ROS discovery/startup did not complete')
        pump(.3)
        require(source.count_publishers('/cmd_vel') == 1
                and source.count_publishers(TOPICS['desired']) == 1,
                'Unexpected command publisher count in isolated test')
        require(gateway_states[-1].get('max_linear_mps') == .2
                and gateway_states[-1]['fault'] is None, 'Gateway profile mismatch or startup fault')
        zero_tail(cmd_rows, 0, 'Gateway emitted nonzero before explicit arm')
        zero_tail(desired_rows, 0, 'Controller emitted nonzero before explicit arm')
        report['checks'].append('explicit_fast_profiles_start_disarmed_and_zero')

        index, arm_call = begin('continuous_fast_profile_nine_seconds')
        start = arm_call['completed_monotonic']
        pump(max(0., start + 9.0 - time.monotonic()))
        end = time.monotonic()
        active = [row for row in cmd_rows[index:] if row['v'] > 0.]
        at_cap = [row for row in active if abs(row['v'] - .2) <= 1e-9]
        require(end - start >= 8.5 and control.core.armed and control.core.arm_deadline is None,
                'Continuous run did not remain armed for at least 8.5 seconds')
        require(at_cap and len(at_cap) >= 5, 'Actual gateway /cmd_vel never sustained the 0.2 m/s cap')
        require(7.7 <= at_cap[0]['mono'] - arm_call['started_monotonic'] <= 8.5,
                'Observed 0.2 m/s arrival inconsistent with the unchanged 0.025 m/s^2 ramp')
        require(all(row['v'] > 0. for row in cmd_rows[index:]
                    if row['mono'] >= end - .5), 'Continuous output unexpectedly stopped')
        report['continuous_run'] = {
            'verified_active_sec': end - start, 'observed_nonzero_count': len(active),
            'first_at_cap_after_request_sec': at_cap[0]['mono'] - arm_call['started_monotonic'],
            'at_cap_output_count': len(at_cap), 'peak_v': max(row['v'] for row in active),
            'peak_abs_w': max(abs(row['w']) for row in active),
        }
        report['checks'].append('real_controller_to_gateway_ramps_to_point_two_in_continuous_mode')

        stop_index = len(cmd_rows)
        stop_call = call(disarm, Trigger.Request(), 'manual_stop_from_point_two')
        require(pump(.4, until=lambda: any(row['v'] == 0. and row['w'] == 0.
                                          for row in cmd_rows[stop_index:])),
                'Manual disarm failed to reach gateway zero output')
        first_zero = next(row for row in cmd_rows[stop_index:] if row['v'] == 0. and row['w'] == 0.)
        report['manual_stop_observed_sec'] = first_zero['mono'] - stop_call['started_monotonic']
        require(report['manual_stop_observed_sec'] <= .4, 'Manual stop observation exceeded 400 ms')
        require_stays_stopped('manual_disarm')
        report['checks'].append('manual_stop_zero_without_automatic_rearm')

        begin('continuous_waypoint_stream_loss')
        streams['waypoint'] = False
        pump(.05)
        last_receipt = control.core.gate.waypoint[3]
        state_index, cmd_index = len(control_states), len(cmd_rows)
        require(pump(1.2, until=lambda: any(row['reason'] == 'waypoint_timeout'
                                          for row in control_states[state_index:])),
                'Waypoint stream loss did not disarm controller')
        require(pump(.4, until=lambda: any(row['v'] == 0. and row['w'] == 0.
                                          for row in cmd_rows[cmd_index:])),
                'Waypoint stream loss did not reach gateway zero output')
        first_zero = next(row for row in cmd_rows[cmd_index:] if row['v'] == 0. and row['w'] == 0.)
        delay = first_zero['mono'] - last_receipt
        require(.6 <= delay <= .85, 'Waypoint timeout output exceeded configured TTL plus observation margin')
        report['waypoint_receipt_to_zero_sec'] = delay
        streams['waypoint'] = True
        last_sent['waypoint'] = 0.
        require_stays_stopped('waypoint_stream_recovery')
        require(gateway_states[-1]['fault'] is None,
                'Normal controller safety-zero incorrectly tripped independent gateway')
        report['checks'].append('waypoint_loss_zero_and_recovery_does_not_rearm')

        begin('independent_gateway_command_timeout')
        pump(.4)
        require(cmd_rows[-1]['v'] > 0., 'Watchdog phase did not begin with actual nonzero gateway output')
        require(desired_rows and desired_rows[-1]['v'] > 0., 'No real controller desired command to interrupt')
        last_desired = desired_rows[-1]
        state_index, cmd_index = len(gateway_states), len(cmd_rows)
        # Simulate a controller/executor stall, without publishing an orderly
        # shutdown zero. The separate gateway process must keep its own clock.
        executor.remove_node(control)
        stall_at = time.monotonic()
        require(pump(.6, until=lambda: any(row['fault'] == 'command_timeout'
                                         for row in gateway_states[state_index:])),
                'Independent gateway did not latch its desired-command timeout')
        require(pump(.2, until=lambda: any(row['v'] == 0. and row['w'] == 0.
                                          for row in cmd_rows[cmd_index:])),
                'Independent gateway did not output zero after controller stall')
        first_zero = next(row for row in cmd_rows[cmd_index:] if row['v'] == 0. and row['w'] == 0.)
        delay = first_zero['mono'] - last_desired['mono']
        require(0. <= delay <= .4, 'Independent gateway stop exceeded 400 ms local observation margin')
        report['gateway_watchdog'] = {
            'last_real_controller_desired_monotonic': last_desired['mono'],
            'controller_executor_removed_monotonic': stall_at,
            'zero_observed_monotonic': first_zero['mono'],
            'last_observed_desired_to_zero_sec': delay,
            'configured_source_and_receipt_timeout_sec': .25,
        }
        executor.add_node(control)
        require_stays_stopped('controller_executor_recovery')
        require(gateway_states[-1]['fault'] == 'command_timeout',
                'Fresh zero commands automatically cleared gateway timeout latch')
        # An explicit controller arm is not a gateway reset. Prove the latter
        # still refuses fresh nonzero desired commands after the process stall.
        desired_index, cmd_index = len(desired_rows), len(cmd_rows)
        begin('explicit_rearm_does_not_reset_gateway_fault', require_forward=False)
        pump(.4)
        require(any(row['v'] > 0. for row in desired_rows[desired_index:]),
                'Latch test had no fresh nonzero real-controller desired commands')
        zero_tail(cmd_rows, cmd_index, 'New controller arm bypassed gateway timeout latch')
        require(gateway_states[-1]['fault'] == 'command_timeout',
                'Gateway timeout fault was unexpectedly cleared')
        call(disarm, Trigger.Request(), 'final_controller_disarm')
        require_stays_stopped('final_disarm')
        report['checks'].extend(['independent_gateway_timeout_zero_within_400ms',
                                 'controller_stall_recovery_does_not_auto_rearm',
                                 'fresh_zero_or_explicit_controller_arm_does_not_reset_gateway_latch'])

        for name, rows in (('desired', desired_rows), ('cmd_vel', cmd_rows)):
            require(rows and all(
                all(math.isfinite(value) for value in (row['v'], row['w'], *row['other']))
                and 0. <= row['v'] <= .200001 and abs(row['w']) <= .100001
                and abs(row['w']) <= row['v'] + 1e-6
                and all(value == 0. for value in row['other']) for row in rows),
                name + ' exceeded finite, planar, forward, speed, angular or curvature bounds')
        require(all(row['frame'] == 'base_link' for row in desired_rows),
                'Controller desired frame changed')
        report['checks'].append('all_observed_commands_keep_reviewed_speed_angular_curvature_planar_bounds')
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
                # No sensor production here; just let the owned source observe
                # the shutdown zero while the independent gateway still lives.
                deadline = time.monotonic() + .15
                while executor is not None and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
            except Exception as error:
                cleanup_errors.append('controller shutdown: ' + repr(error))
        if child is not None:
            try:
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=3.)
                        cleanup_errors.append('gateway required forced kill')
                report['gateway_returncode'] = child.returncode
                if child.returncode != 0:
                    cleanup_errors.append(f'gateway return code {child.returncode}')
            except Exception as error:
                cleanup_errors.append('gateway cleanup: ' + repr(error))
        if gateway_log is not None:
            gateway_log.close()
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if control is not None:
            control.destroy_node()
        if source is not None:
            source.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)
        if cleanup_errors:
            report.update(status='failed', cleanup_errors=cleanup_errors)
            result = 1
        report.update(desired_commands=desired_rows, cmd_vel_outputs=cmd_rows,
                      control_states=control_states, gateway_states=gateway_states,
                      finished_utc=datetime.now(timezone.utc).isoformat())
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n',
                                          encoding='utf-8')
        print(json.dumps({'status': report['status'], 'report': str(output / 'report.json')}), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
