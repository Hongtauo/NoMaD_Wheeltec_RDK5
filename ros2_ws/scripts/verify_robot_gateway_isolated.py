#!/usr/bin/env python3
"""Exercise the robot gateway on loopback-only ROS domain 86, never domain 0.

This test sends small NONZERO test messages only inside the isolated local test
domain. It must not be used as evidence of a motor-controller hardware watchdog.
No robot, camera, SSH, base driver, or model is started or contacted. The gateway
is the only subprocess created and is always stopped during cleanup.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


DOMAIN = '86'


def isolate_environment():
    # Do this before any rclpy import/init, including the child gateway's init.
    # A inherited robot profile must never allocate its fixed network ports.
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'ROS_DISCOVERY_SERVER', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
                'FASTDDS_ENVIRONMENT_FILE', 'ROS_STATIC_PEERS'):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', SKIP_DEFAULT_XML='1')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output-root', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'outputs/ros2_robot_gateway_tests')
    args = parser.parse_args(argv)
    isolate_environment()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist, TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    output = args.output_root / ('domain86__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                 + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'domain': DOMAIN, 'localhost_only': True,
              'hardware_test': False, 'checks': [], 'created_utc': datetime.now(timezone.utc).isoformat()}
    received, states = [], []
    child = context = node = executor = None
    last_send = 0.
    last_sent_mono = None
    result = 1

    def require(condition, message):
        if not condition:
            raise RuntimeError(message)

    try:
        context = Context()
        rclpy.init(args=[], context=context)
        require(context.get_domain_id() == int(DOMAIN), 'ROS context is not isolated domain 86')
        node = Node('gateway_isolated_test', context=context, use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        desired = node.create_publisher(TwistStamped, '/nomad/robot_control/desired_cmd', qos)
        reset_client = node.create_client(Trigger, '/nomad/robot_gateway/reset')

        def on_output(msg):
            received.append({'mono': time.monotonic(), 'v': msg.linear.x, 'w': msg.angular.z,
                             'other': [msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y]})

        def on_status(msg):
            states.append({'mono': time.monotonic(), **json.loads(msg.data)})

        node.create_subscription(Twist, '/cmd_vel', on_output, qos)
        node.create_subscription(String, '/nomad/robot_gateway/status', on_status, qos)

        def pump(seconds, command=None, until=None, child_should_live=True):
            nonlocal last_send, last_sent_mono
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if child_should_live:
                    require(child.poll() is None, 'Gateway process exited before test phase completed')
                now = time.monotonic()
                if command is not None and now - last_send >= .05:
                    msg = TwistStamped()
                    msg.header.frame_id = 'base_link'
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.twist.linear.x, msg.twist.angular.z = command
                    desired.publish(msg)
                    last_send = last_sent_mono = now
                executor.spin_once(timeout_sec=.01)
                if until is not None and until():
                    return True
            return until is None

        def fresh_zero_reset():
            pump(.20, (0., 0.))
            require(reset_client.service_is_ready(), 'Reset service is not available')
            future = reset_client.call_async(Trigger.Request())
            require(pump(2., (0., 0.), until=future.done), 'Reset service timed out')
            require(future.result() is not None and future.result().success, 'Fresh-zero reset was refused')

        gateway = Path(__file__).resolve().parents[1] / 'src/nomad_navigation/nomad_navigation/robot_gateway.py'
        child = subprocess.Popen([sys.executable, '-u', str(gateway)], env=dict(os.environ),
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        require(pump(5., (0., 0.), until=lambda: bool(states) and reset_client.service_is_ready()),
                'Gateway did not become visible in isolated domain 86')
        require(received and all(row['v'] == 0. and row['w'] == 0. for row in received),
                'Gateway emitted nonzero output before a nonzero command')
        report['checks'].append('startup_zero_only')
        fresh_zero_reset()

        index = len(received)
        require(pump(2., (.05, .025), until=lambda: any(row['v'] > 0 for row in received[index:])),
                'Fresh command was not forwarded')
        pump(.30, (.05, .025))
        require(all(0 <= row['v'] <= .050001 and abs(row['w']) <= .100001
                    and all(value == 0. for value in row['other']) for row in received),
                'Output exceeded the reviewed low-speed/planar bounds')
        report['checks'].append('fresh_low_speed_command_forwarded')

        index = len(received)
        silence_at = last_sent_mono
        require(pump(.45, until=lambda: any(row['v'] == 0. and row['w'] == 0.
                                            for row in received[index:])),
                'Gateway failed to publish zero after command silence')
        stopped_at = next(row['mono'] for row in received[index:] if row['v'] == 0. and row['w'] == 0.)
        report['silence_to_zero_sec'] = stopped_at - silence_at
        require(report['silence_to_zero_sec'] <= .40, 'Command timeout stop exceeded 400 ms local test margin')
        pump(.15)
        require(any(row['state'] == 'command_timeout' for row in states), 'Timeout was not reported')
        report['checks'].append('silence_ttl_zero_within_400ms')

        index = len(received)
        pump(.35, (.05, .025))
        require(received[index:] and all(row['v'] == 0. and row['w'] == 0. for row in received[index:]),
                'Nonzero fresh commands bypassed the timeout latch')
        report['checks'].append('timeout_latched_until_explicit_zero_reset')
        fresh_zero_reset()
        index = len(received)
        require(pump(2., (.05, .025), until=lambda: any(row['v'] > 0 for row in received[index:])),
                'Gateway did not resume after explicit fresh-zero reset')

        # A second publisher need not send anything: graph conflicts alone must stop.
        for topic, message_type, expected in (
            ('/nomad/robot_control/desired_cmd', TwistStamped, 'multiple_command_publishers'),
            ('/cmd_vel', Twist, 'multiple_velocity_publishers'),
        ):
            extra = node.create_publisher(message_type, topic, qos)
            state_index = len(states)
            require(pump(3., (.05, .025), until=lambda: any(row['state'] == expected for row in states[state_index:])),
                    f'Publisher conflict did not latch: {topic}')
            index = len(received)
            pump(.20, (.05, .025))
            require(received[index:] and all(row['v'] == 0. and row['w'] == 0. for row in received[index:]),
                    f'Gateway did not remain zero under {expected}')
            node.destroy_publisher(extra)
            require(pump(3., (0., 0.), until=lambda: node.count_publishers(topic) == 1),
                    'Extra publisher was not removed from the graph')
            fresh_zero_reset()
            index = len(received)
            require(pump(2., (.05, .025), until=lambda: any(row['v'] > 0 for row in received[index:])),
                    'Gateway did not resume after conflict removal and explicit zero reset')
            report['checks'].append(expected + '_stops_and_latches')

        # Keep desired messages arriving during graceful shutdown. Cancelled timers
        # and a latched shutdown must prevent a final zero -> nonzero regression.
        index = len(received)
        child.send_signal(signal.SIGTERM)
        require(pump(3., (.05, .025), until=lambda: child.poll() is not None, child_should_live=False),
                'Gateway did not exit after SIGTERM')
        pump(.20, child_should_live=False)
        tail = received[index:]
        zero_index = next((i for i, row in enumerate(tail) if row['v'] == 0. and row['w'] == 0.), None)
        require(zero_index is not None, 'Graceful shutdown emitted no observed zero')
        require(all(row['v'] == 0. and row['w'] == 0. for row in tail[zero_index:]),
                'Gateway published nonzero again after its shutdown zero')
        require(child.returncode == 0, f'Gateway exited abnormally: {child.returncode}')
        report['checks'].append('sigterm_zero_without_nonzero_regression')
        report['status'] = 'passed'
        result = 0
    except BaseException as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
        print(report['error'], flush=True)
    finally:
        if child is not None:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=5.)
                    report['forced_child_kill'] = True
            stdout = child.communicate(timeout=2.)[0]
            (output / 'gateway.log').write_text(stdout, encoding='utf-8')
            report['gateway_returncode'] = child.returncode
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        report['outputs'] = received
        report['states'] = states
        report['finished_utc'] = datetime.now(timezone.utc).isoformat()
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({'status': report['status'], 'report': str(output / 'report.json')}), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
