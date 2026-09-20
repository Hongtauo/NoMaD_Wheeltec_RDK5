"""Preview-only ROS 2 velocity controller with input freshness and stop gates."""

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time
import uuid

from geometry_msgs.msg import PointStamped, Twist
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from nomad_navigation.controller_core import ControlLimits, SafetyGate


class ControllerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('waypoint_controller', **kwargs)
        self._publisher = None
        self._csv_file = None
        self._events = None
        self._closed = False
        self._io_fault = False
        self._state = None
        self.run_dir = None
        self.command_count = 0
        self.nonzero_count = 0
        try:
            defaults = {
                'dry_run': True, 'preview_topic': '/cmd_vel_preview',
                'waypoint_topic': '/waypoint_stamped', 'image_topic': '/camera/image_raw',
                'model_status_topic': '/nomad/status', 'status_topic': '/controller/status',
                'estop_topic': '/nomad/emergency_stop', 'reached_goal_topic': '/topoplan/reached_goal',
                'reset_estop_service': '/nomad/reset_emergency_stop',
                'control_hz': 20.0, 'output_root': '',
                **ControlLimits().__dict__,
            }
            for key, value in defaults.items():
                self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
            self.params = {key: self.get_parameter(key).value for key in defaults}
            p = self.params
            if not p['dry_run']:
                raise ValueError('Only dry_run=true is implemented; live robot output is intentionally disabled')
            # Resolve remapping before creating any Twist publisher.
            resolved = self.resolve_topic_name(p['preview_topic'])
            if resolved.rsplit('/', 1)[-1] != 'cmd_vel_preview':
                raise ValueError('Dry-run output must resolve to a cmd_vel_preview topic; motion-topic remapping refused')
            if not math.isfinite(p['control_hz']) or not 1 <= p['control_hz'] <= 100:
                raise ValueError('control_hz must be between 1 and 100')
            if not p['output_root'].strip():
                raise ValueError('output_root is required for per-run controller logs')
            self.limits = ControlLimits(**{key: p[key] for key in ControlLimits.__dataclass_fields__})
            self.gate = SafetyGate(self.limits)
            root = Path(p['output_root']).expanduser().resolve()
            root.mkdir(parents=True, exist_ok=True)
            candidate = root / ('controller__dry-run__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                + '__' + uuid.uuid4().hex[:8])
            candidate.mkdir(exist_ok=False)
            self.run_dir = candidate
            self._csv_file = (candidate / 'cmd_vel.csv').open('w', newline='', encoding='utf-8')
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(['time_ns', 'monotonic_sec', 'source_stamp_ns', 'linear_x', 'angular_z', 'reason'])
            self._events = (candidate / 'events.jsonl').open('w', encoding='utf-8')
            self._publisher = self.create_publisher(Twist, p['preview_topic'], QoSProfile(
                depth=1, lifespan=Duration(seconds=0.2)))
            self._status_pub = self.create_publisher(String, p['status_topic'], QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(PointStamped, p['waypoint_topic'], self._on_waypoint, 1)
            self.create_subscription(Image, p['image_topic'], self._on_camera,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(String, p['model_status_topic'], self._on_model,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Bool, p['estop_topic'], self._on_estop, 1)
            self.create_subscription(Bool, p['reached_goal_topic'], self._on_reached_goal, 1)
            self.create_service(Trigger, p['reset_estop_service'], self._reset_estop)
            # The watchdog must keep ticking even when /clock is paused.
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._timer = self.create_timer(1 / p['control_hz'], self._tick, clock=self._steady_clock)
            self._publish(0.0, 0.0, 'startup')
            self.get_logger().info(f'Preview controller ready; output={self._publisher.topic_name}; logs={candidate}')
        except Exception:
            self.destroy_node()
            raise

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    def _on_waypoint(self, msg):
        accepted = self.gate.accept_waypoint(msg.point.x, msg.point.y, msg.point.z, self._stamp(msg),
                                             msg.header.frame_id, self.get_clock().now().nanoseconds, time.monotonic())
        if not accepted:
            self._tick()

    def _on_camera(self, msg):
        channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding, 0)
        if (msg.width <= 0 or msg.height <= 0 or msg.step <= 0
                or not channels or msg.step < msg.width * channels
                or len(msg.data) != msg.height * msg.step):
            self.gate.camera = None
            self.gate.reject('invalid_camera')
            self._tick()
            return
        if not self.gate.accept_camera(self._stamp(msg), msg.header.frame_id,
                                       self.get_clock().now().nanoseconds, time.monotonic()):
            self.gate.reject('invalid_camera')
            self._tick()

    def _on_model(self, msg):
        try:
            state = json.loads(msg.data)['state']
            if not isinstance(state, str):
                raise ValueError('Invalid model state')
        except (ValueError, KeyError, TypeError):
            state = 'unknown'
        self.gate.set_model_state(state)
        if state != 'running':
            self._tick()

    def _on_estop(self, msg):
        self.gate.set_estop(msg.data)
        if msg.data:
            self._tick()

    def _on_reached_goal(self, msg):
        self.gate.reached_goal = msg.data
        if msg.data:
            self.gate.reject('reached_goal')
            self._tick()

    def _reset_estop(self, request, response):
        response.success = self.gate.reset_estop()
        response.message = ('Software stop reset; waiting for a new waypoint' if response.success
                            else 'Emergency-stop input is still true; reset refused')
        self._tick()
        return response

    def _tick(self):
        if self._io_fault:
            self._publish(0.0, 0.0, 'logging_fault')
            return
        v, w, reason = self.gate.decide(self.get_clock().now().nanoseconds, time.monotonic())
        self._publish(v, w, reason)

    def _publish(self, v, w, reason):
        if self._publisher is None:
            return
        source = self.gate.waypoint[2] if self.gate.waypoint else 0
        if not self._io_fault:
            try:
                self._csv.writerow([self.get_clock().now().nanoseconds, time.monotonic(), source, v, w, reason])
                self._csv_file.flush()
            except (OSError, ValueError) as error:
                self._io_fault = True
                self.get_logger().error(f'Logging failed; forcing zero preview: {error}')
        if self._io_fault:
            v, w, reason = 0.0, 0.0, 'logging_fault'
        command = Twist()
        command.linear.x, command.angular.z = float(v), float(w)
        self._publisher.publish(command)
        self.command_count += 1
        self.nonzero_count += int(v != 0 or w != 0)
        if reason != self._state:
            self._state = reason
            state = {'state': reason, 'dry_run': True, 'run_dir': str(self.run_dir),
                     'output_topic': self._publisher.topic_name, 'time_ns': self.get_clock().now().nanoseconds}
            self._status_pub.publish(String(data=json.dumps(state)))
            self.get_logger().info(f'preview_state={reason}; v={v:.3f}, w={w:.3f}')
            try:
                self._events.write(json.dumps(state) + '\n')
                self._events.flush()
            except (OSError, ValueError) as error:
                self._io_fault = True
                self.get_logger().error(f'Event logging failed: {error}')
                self._publisher.publish(Twist())
        self._write_manifest()

    def _write_manifest(self):
        if self.run_dir is None or self._io_fault:
            return
        try:
            manifest = {'parameters': self.params, 'resolved_output_topic': self._publisher.topic_name,
                        'state': self._state, 'command_count': self.command_count,
                        'nonzero_count': self.nonzero_count, 'dry_run_only': True}
            (self.run_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
        except OSError as error:
            self._io_fault = True
            self.get_logger().error(f'Manifest logging failed: {error}')
            self._publisher.publish(Twist())

    def prepare_shutdown(self):
        if self._publisher is not None and self.context.ok():
            self._publish(0.0, 0.0, 'shutdown')
            # Bounded DDS acknowledgement wait; never a substitute for a robot watchdog.
            self._publisher.wait_for_all_acked(Duration(seconds=0.3))

    def destroy_node(self):
        if not self._closed:
            self._closed = True
            for handle in (self._csv_file, self._events):
                if handle is not None:
                    handle.close()
        return super().destroy_node()


def main(args=None):
    # Keep ROS alive until the final zero is published on SIGINT/SIGTERM.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopped = False
    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True
    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    result = 0
    try:
        node = ControllerNode()
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as error:
        rclpy.logging.get_logger('waypoint_controller').error(f'{type(error).__name__}: {error}')
        result = 1
    finally:
        try:
            if node is not None:
                result = 1 if node._io_fault else result
                node.prepare_shutdown()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
