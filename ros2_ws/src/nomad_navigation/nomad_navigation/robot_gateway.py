"""Robot-local, independent velocity watchdog. Never run this on the PC.

This is an extra software layer, NOT a substitute for the physical emergency
stop or a verified motor-controller/firmware watchdog. A whole robot OS freeze
or failure of this process still requires those hardware protections.
"""

import argparse
import json
import math
import os
import platform
import signal
import subprocess
import time


REVIEWED_LINEAR_CAPS = (0.05, 0.1, 0.2)


class CommandWatchdog:
    timeout = 0.25
    source_timeout = 0.25
    future_tolerance = 0.2

    def __init__(self, max_linear_mps=0.05):
        if (isinstance(max_linear_mps, bool)
                or not isinstance(max_linear_mps, (int, float))
                or max_linear_mps not in REVIEWED_LINEAR_CAPS
                or not math.isfinite(max_linear_mps)):
            raise ValueError('max_linear_mps must be one of the reviewed startup caps: 0.05, 0.1, 0.2')
        self.max_linear_mps = float(max_linear_mps)
        self.command = None
        self.highwater = 0
        self.last_ros = None
        self.fault = None
        self.moving = False

    def accept(self, values, stamp, frame, now, mono):
        if (len(values) != 6 or not all(math.isfinite(v) for v in values)
                or abs(values[0]) > self.max_linear_mps + 1e-6 or values[0] < 0
                or abs(values[5]) > .100001
                or abs(values[5]) > values[0] + 1e-6
                or any(abs(values[i]) > 1e-9 for i in (1, 2, 3, 4))):
            self.fault = self.fault or 'invalid_command'
        elif frame != 'base_link':
            self.fault = self.fault or 'wrong_frame'
        elif stamp <= self.highwater or stamp <= 0:
            self.fault = self.fault or 'replayed_command'
        elif not -self.future_tolerance * 1e9 <= now - stamp <= self.source_timeout * 1e9:
            self.fault = self.fault or 'stale_command'
        else:
            self.highwater = stamp
            self.command = (values[0], values[5], stamp, mono)
            return True
        self.command = None
        return False

    def decide(self, now, mono):
        if self.last_ros is not None and now < self.last_ros:
            self.fault = 'clock_rollback'
        self.last_ros = now
        if self.command is None:
            reason = self.fault or 'waiting_command'
        elif (mono - self.command[3] > self.timeout or
              not -self.future_tolerance * 1e9 <= now - self.command[2] <= self.source_timeout * 1e9):
            if self.moving:
                self.fault = self.fault or 'command_timeout'
            reason = self.fault or 'command_timeout'
        else:
            reason = self.fault
        if reason:
            self.moving = False
            return 0., 0., reason
        v, w = self.command[:2]
        self.moving = bool(v or w)
        return v, w, 'forwarding' if self.moving else 'stopped'

    def reset(self, now, mono):
        if self.fault in ('clock_rollback', 'shutdown'):
            return False
        command = self.command
        if (command is None or command[0] != 0 or command[1] != 0
                or mono - command[3] > self.timeout
                or not -self.future_tolerance * 1e9 <= now - command[2] <= self.source_timeout * 1e9):
            return False
        self.fault = None
        return True


def parse_args(args=None):
    parser = argparse.ArgumentParser(description='Robot-local independent velocity watchdog',
                                     allow_abbrev=False)
    parser.add_argument('--max-linear-mps', type=float, choices=REVIEWED_LINEAR_CAPS,
                        default=0.05, help='Reviewed startup speed cap; default retains the 0.05 m/s profile')
    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)
    # Domain 86 is reserved for isolated localhost integration tests. Domain 0
    # actuation may run only on the reviewed ARM robot, never Docker/Windows.
    domain = os.environ.get('ROS_DOMAIN_ID')
    if domain == '0':
        if os.environ.get('NOMAD_TRANSPORT') == 'api':
            from .robot_session_guard import require_robot_session
            require_robot_session()  # Identity and fresh clock token, not a mutable DHCP IP.
        else:
            addresses = subprocess.check_output(['hostname', '-I'], text=True).split()
            if platform.machine() != 'aarch64' or '192.168.0.100' not in addresses:
                raise RuntimeError('Domain 0 gateway is restricted to the reviewed ARM robot at 192.168.0.100')
    elif domain != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use robot domain 0, or isolated domain 86 with ROS_LOCALHOST_ONLY=1')
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist, TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class Gateway(Node):
        def __init__(self):
            super().__init__('robot_gateway', namespace='/nomad', use_global_arguments=False)
            self.guard = CommandWatchdog(max_linear_mps=options.max_linear_mps)
            self.last_reason = None
            self.ticks = 0
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             lifespan=Duration(seconds=.2))
            self.output = self.create_publisher(Twist, '/cmd_vel', qos)
            self.status = self.create_publisher(String, '/nomad/robot_gateway/status', 1)
            self.input = self.create_subscription(TwistStamped, '/nomad/robot_control/desired_cmd', self.receive, qos)
            self.reset_service = self.create_service(Trigger, '/nomad/robot_gateway/reset', self.reset)
            self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def receive(self, msg):
            t = msg.twist
            self.guard.accept((t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z),
                              msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                              msg.header.frame_id, self.get_clock().now().nanoseconds, time.monotonic())

        def reset(self, request, response):
            response.success = self.guard.reset(self.get_clock().now().nanoseconds, time.monotonic())
            response.message = 'reset' if response.success else 'need fresh zero command; clock fault requires restart'
            return response

        def stop(self):
            self.output.publish(Twist())

        def tick(self):
            # Check each tick: an additional velocity source is a conflict, not
            # something this process should compete with by overwriting commands.
            if self.count_publishers('/cmd_vel') != 1:
                self.guard.fault = self.guard.fault or 'multiple_velocity_publishers'
            if self.count_publishers('/nomad/robot_control/desired_cmd') > 1:
                self.guard.fault = self.guard.fault or 'multiple_command_publishers'
            v, w, reason = self.guard.decide(self.get_clock().now().nanoseconds, time.monotonic())
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            self.output.publish(msg)
            self.ticks += 1
            if self.ticks % 2 == 0:
                status = String()
                status.data = json.dumps({'state': reason, 'v': v, 'w': w, 'fault': self.guard.fault,
                                          'max_linear_mps': self.guard.max_linear_mps})
                self.status.publish(status)
            if reason != self.last_reason:
                self.get_logger().info(f'gateway={reason} v={v:.3f} w={w:.3f}')
                self.last_reason = reason

    # The fixed robot endpoints cannot be remapped through ROS arguments.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Gateway()
    stopped = False

    def stop_signal(signum, frame):
        nonlocal stopped
        stopped = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop_signal)
    try:
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.timer.cancel()
        node.guard.fault = 'shutdown'
        for _ in range(5):
            node.stop()
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
