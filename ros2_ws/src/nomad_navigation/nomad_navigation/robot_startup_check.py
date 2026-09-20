"""Read-only, fail-closed startup check before any hardware driver starts.

This process never creates a publisher or service client. It verifies the two
independent software control processes are alive, disarmed, correctly capped,
and publishing only zero commands. It is not a substitute for the physical
emergency stop or a motor-controller watchdog.
"""

import argparse
import json
import math
import time


# Command ceiling and independent measured linear-speed fault limit. The new
# intermediate profile does not change the existing profiles or angular limits.
SPEED_PROFILES = {'0.05': (0.05, 0.12), '0.1': (0.1, 0.2), '0.2': (0.2, 0.3)}
TOPICS = {
    'control': '/nomad/robot_control/status',
    'gateway': '/nomad/robot_gateway/status',
    'desired': '/nomad/robot_control/desired_cmd',
    'output': '/cmd_vel',
}


def numeric_equal(value, expected):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and abs(value - expected) < 1e-9)


def command_values(message):
    twist = getattr(message, 'twist', message)
    return (twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z)


def endpoint_names(endpoints):
    return [(endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name)
            for endpoint in endpoints]


class StartupObservation:
    """Pure observation accumulator. Unsafe observations latch failure."""

    freshness_sec = 0.5
    stable_sec = 1.0

    def __init__(self, speed_profile, operator_session_id=None, *, operator_pause_resume=False):
        if speed_profile not in SPEED_PROFILES:
            raise ValueError('Use the reviewed 0.05, 0.1 or 0.2 m/s startup profile')
        if type(operator_pause_resume) is not bool or (operator_pause_resume and operator_session_id is None):
            raise ValueError('Operator pause/resume startup requires an explicit boolean and a guarded session')
        self.operator_pause_resume = operator_pause_resume
        linear, measured = SPEED_PROFILES[speed_profile]
        self.expected_limits = {
            'max_linear_mps': linear, 'max_measured_linear_mps': measured,
            'max_angular_rps': 0.1, 'linear_acceleration_mps2': 0.025,
            'min_turning_radius_m': 1.0,
        }
        self.received = {}
        self.sample_counts = {key: 0 for key in TOPICS}
        self.failure = None
        self.stable_since = None
        self.operator_session_id = operator_session_id
        if operator_session_id is not None:
            from .operator_guard import canonical_uuid
            canonical_uuid(operator_session_id)

    def fail(self, reason):
        self.failure = self.failure or reason

    def accept_status(self, source, state, mono):
        if source not in ('control', 'gateway'):
            raise ValueError('Unknown startup status source')
        if not isinstance(state, dict):
            self.fail(f'{source}: invalid status object')
            return
        if source == 'control':
            if state.get('operator_pause_resume', False) is not self.operator_pause_resume:
                self.fail('controller operator pause/resume policy does not match explicit startup policy')
                return
            if self.operator_session_id is not None:
                link = state.get('operator_link', {})
                if (not isinstance(link, dict) or link.get('enabled') is not True
                        or link.get('session_id') != self.operator_session_id):
                    self.fail('controller lacks the required operator-session authorization guard')
                    return
                if self.operator_pause_resume and (link.get('pause_resume') is not True
                        or state.get('network_pause_resume') is not True
                        or state.get('operator_paused') is not False):
                    self.fail('controller lacks the required idle operator pause/resume gates')
                    return
            if state.get('armed') is not False:
                self.fail('controller must explicitly report armed=false')
                return
            if state.get('active_mode') is not None:
                self.fail('controller has an active run mode at startup')
                return
            limits = state.get('limits')
            if not isinstance(limits, dict) or any(
                    not numeric_equal(limits.get(key), value)
                    for key, value in self.expected_limits.items()):
                self.fail('controller startup limits do not match the selected profile')
                return
            values = (state.get('linear_x'), state.get('angular_z'))
        else:
            if 'fault' not in state or state['fault'] is not None:
                self.fail('gateway reports a fault or lacks its fault field')
                return
            if not numeric_equal(state.get('max_linear_mps'),
                                 self.expected_limits['max_linear_mps']):
                self.fail('gateway speed cap does not match the selected profile')
                return
            values = (state.get('v'), state.get('w'))
            if not all(numeric_equal(value, 0.0) for value in values):
                self.fail('gateway reports a nonzero or malformed startup command')
                return
            if state.get('state') != 'stopped':
                # A fresh zero desired command must have reached the gateway;
                # waiting_command is not sufficient to start the chassis.
                self.stable_since = None
                return
        if not all(numeric_equal(value, 0.0) for value in values):
            self.fail(f'{source} reports a nonzero or malformed startup command')
            return
        self.received[source] = mono
        self.sample_counts[source] += 1

    def accept_command(self, source, values, mono):
        if source not in ('desired', 'output'):
            raise ValueError('Unknown startup command source')
        if len(values) != 6 or not all(numeric_equal(value, 0.0) for value in values):
            self.fail(f'{source}: nonzero or malformed startup command')
            return
        self.received[source] = mono
        self.sample_counts[source] += 1

    def evaluate(self, mono, output_publishers, desired_publishers):
        if self.failure:
            return False, self.failure
        for actual, expected, label in (
                (output_publishers, '/nomad/robot_gateway', '/cmd_vel'),
                (desired_publishers, '/nomad/robot_control', 'desired_cmd')):
            if len(actual) > 1 or (actual and actual != [expected]):
                self.fail(f'{label} publisher conflict: {actual!r}')
                return False, self.failure
            if actual != [expected]:
                self.stable_since = None
                return False, f'waiting for the sole expected {label} publisher'
        if any(key not in self.received or
               not 0 <= mono - self.received[key] <= self.freshness_sec
               for key in TOPICS):
            self.stable_since = None
            return False, 'waiting for fresh zero commands and both status streams'
        if self.stable_since is None:
            self.stable_since = mono
        if mono - self.stable_since < self.stable_sec:
            return False, 'observing stable disarmed zero-output state'
        return True, 'startup zero-output, publisher identities, and speed caps verified'


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-linear-mps', choices=SPEED_PROFILES, default='0.2')
    parser.add_argument('--timeout-sec', type=float, default=15.0)
    options = parser.parse_args(args)
    if not math.isfinite(options.timeout_sec) or not 2 <= options.timeout_sec <= 30:
        parser.error('--timeout-sec must be within [2, 30]')
    # No test bypass or publisher is supplied by this command. Pure-state
    # tests exercise StartupObservation without touching a ROS domain.
    from nomad_navigation.robot_session_guard import require_robot_session
    require_robot_session()
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    context = Context()
    node = None
    executor = None
    import os
    from nomad_navigation.operator_guard import GUARD_ENV, operator_pause_resume_from_env
    observation = StartupObservation(options.max_linear_mps, os.environ.get(GUARD_ENV),
        operator_pause_resume=operator_pause_resume_from_env(os.environ))

    def on_status(source, message):
        try:
            state = json.loads(message.data)
        except (ValueError, TypeError):
            observation.fail(f'{source}: status is not valid JSON')
            return
        observation.accept_status(source, state, time.monotonic())

    try:
        rclpy.init(args=[], context=context)
        node = Node('robot_startup_check', namespace='/nomad', context=context,
                    use_global_arguments=False)
        # A private Context must have its own executor. rclpy.spin_once without
        # one uses the uninitialized global Context and fails on ROS Humble.
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        subscriptions = []
        for source in ('control', 'gateway'):
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=(DurabilityPolicy.TRANSIENT_LOCAL if source == 'control'
                                         else DurabilityPolicy.VOLATILE))
            subscriptions.append(node.create_subscription(
                String, TOPICS[source], lambda msg, key=source: on_status(key, msg), qos))
        for source, msg_type in (('desired', TwistStamped), ('output', Twist)):
            subscriptions.append(node.create_subscription(
                msg_type, TOPICS[source],
                lambda msg, key=source: observation.accept_command(
                    key, command_values(msg), time.monotonic()),
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)))
        deadline = time.monotonic() + options.timeout_sec
        reason = 'no startup samples'
        while context.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            passed, reason = observation.evaluate(
                time.monotonic(),
                endpoint_names(node.get_publishers_info_by_topic(TOPICS['output'])),
                endpoint_names(node.get_publishers_info_by_topic(TOPICS['desired'])))
            if observation.failure:
                raise RuntimeError(observation.failure)
            if passed:
                node.get_logger().info(reason)
                return 0
        raise RuntimeError(f'Startup verification timed out: {reason}')
    finally:
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=2.)
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                if context.ok():
                    context.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
