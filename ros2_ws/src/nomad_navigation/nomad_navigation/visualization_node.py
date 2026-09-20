"""Stationary local-frame decision display, never a robot simulator or controller."""
import json
import math
import time

from geometry_msgs.msg import Point, PointStamped, PoseArray, Twist
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def decode_layout(value):
    samples, steps, index = (value[k] for k in ('samples', 'steps', 'waypoint_index'))
    if any(type(n) is not int for n in (samples, steps, index)):
        raise ValueError('Layout dimensions must be integers')
    if not 1 <= samples <= 64 or not 1 <= steps <= 256 or not 0 <= index < steps:
        raise ValueError('Invalid trajectory layout')
    scale = value['scale_applied']
    if not math.isfinite(scale) or scale <= 0 or not value['frame_id']:
        raise ValueError('Invalid trajectory frame or nominal scale')
    return samples, steps, index, value['frame_id']


def paired_paths(poses, waypoint, layout):
    samples, steps, index, frame = decode_layout(layout)
    if (poses.header != waypoint.header or poses.header.frame_id != frame
            or len(poses.poses) != samples * steps):
        raise ValueError('Predictions must have matching headers and layout')
    points = [pose.position for pose in poses.poses]
    points_to_check = points + [waypoint.point]
    if any(not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or abs(p.z) > 1e-6
           for p in points_to_check):
        raise ValueError('Invalid planar prediction')
    selected = points[index]
    if math.hypot(selected.x-waypoint.point.x, selected.y-waypoint.point.y) > 1e-5:
        raise ValueError('Waypoint does not match selected trajectory position')
    return [points[i*steps:(i+1)*steps] for i in range(samples)]


class VisualizationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('nomad_visualization', **kwargs)
        defaults = {'frame_id': 'base_link', 'trajectory_topic': '/nomad/trajectory_poses',
                    'waypoint_topic': '/waypoint_stamped', 'velocity_topic': '/cmd_vel_preview',
                    'model_status_topic': '/nomad/status', 'controller_status_topic': '/controller/status',
                    'marker_topic': '/nomad/visualization', 'refresh_hz': 10.0,
                    'prediction_timeout_sec': 1.0, 'velocity_timeout_sec': 0.6,
                    'future_tolerance_sec': 0.2, 'arrow_horizon_sec': 2.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key in ('refresh_hz', 'prediction_timeout_sec', 'velocity_timeout_sec', 'arrow_horizon_sec'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if (not self.p['frame_id'].strip() or self.p['frame_id'].startswith('/')
                or not math.isfinite(self.p['future_tolerance_sec']) or self.p['future_tolerance_sec'] < 0):
            raise ValueError('Invalid frame or future tolerance')
        self.layout = None
        self.model_state = 'unknown'
        self.controller_state = 'unknown'
        self.predictions = {}
        self.waypoints = {}
        self.pair = None
        self.velocity = None
        self.last_ros = None
        self.publisher = self.create_publisher(MarkerArray, self.p['marker_topic'], 1)
        self.create_subscription(PoseArray, self.p['trajectory_topic'], self._poses, 1)
        self.create_subscription(PointStamped, self.p['waypoint_topic'], self._waypoint, 1)
        self.create_subscription(Twist, self.p['velocity_topic'], self._velocity, 1)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, self.p['model_status_topic'], self._model, status_qos)
        self.create_subscription(String, self.p['controller_status_topic'], self._controller, status_qos)
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1/self.p['refresh_hz'], self._tick, clock=self._steady)
        self.get_logger().info('Local RViz display ready; stationary origin, no odometry or motion publisher')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def _fresh_stamp(self, stamp):
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        return stamp > 0 and -self.p['future_tolerance_sec'] <= age <= self.p['prediction_timeout_sec']

    def _clear_predictions(self):
        self.predictions.clear()
        self.waypoints.clear()
        self.pair = None

    def _model(self, msg):
        try:
            value = json.loads(msg.data)
            layout = value['trajectory_layout']
            decode_layout(layout)
            if layout['frame_id'] != self.p['frame_id']:
                raise ValueError('Frame mismatch')
            if layout != self.layout:
                self._clear_predictions()
            self.layout, self.model_state = layout, str(value['state'])
        except (ValueError, KeyError, TypeError):
            self.layout, self.model_state = None, 'invalid_metadata'
        if self.model_state != 'running':
            self._clear_predictions()

    def _controller(self, msg):
        try:
            self.controller_state = str(json.loads(msg.data)['state'])
        except (ValueError, KeyError, TypeError):
            self.controller_state = 'unknown'

    def _cache(self, cache, msg):
        stamp = self.stamp(msg)
        if msg.header.frame_id != self.p['frame_id'] or not self._fresh_stamp(stamp):
            self._clear_predictions()
            return
        cache[stamp] = (msg, time.monotonic())
        while len(cache) > 4:
            del cache[min(cache)]

    def _poses(self, msg):
        self._cache(self.predictions, msg)

    def _waypoint(self, msg):
        self._cache(self.waypoints, msg)

    def _velocity(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-6 for v in values[1:5]):
            self.velocity = None
            return
        self.velocity = (msg.linear.x, msg.angular.z, time.monotonic())

    def _marker(self, kind, namespace, ident, color):
        marker = Marker()
        marker.header.frame_id = self.p['frame_id']
        # Frame-local display at latest transform, not a global trajectory estimate.
        marker.ns, marker.id, marker.type = namespace, ident, kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = Duration(seconds=0.5).to_msg()
        return marker

    def render(self):
        now, mono = self.get_clock().now().nanoseconds, time.monotonic()
        if self.last_ros is not None and now < self.last_ros:
            self._clear_predictions()
            self.velocity = None
        self.last_ros = now
        if self.layout and self.model_state == 'running':
            common = self.predictions.keys() & self.waypoints.keys()
            if common:
                stamp = max(common)
                poses, arrived = self.predictions[stamp]
                waypoint, wp_arrived = self.waypoints[stamp]
                if self._fresh_stamp(stamp) and mono-min(arrived, wp_arrived) <= self.p['prediction_timeout_sec']:
                    try:
                        paths = paired_paths(poses, waypoint, self.layout)
                        self.pair = (paths, waypoint, stamp, min(arrived, wp_arrived))
                    except ValueError:
                        self.pair = None
        if self.pair and (not self._fresh_stamp(self.pair[2])
                          or mono-self.pair[3] > self.p['prediction_timeout_sec']):
            self._clear_predictions()
        # DELETEALL guarantees that fewer candidates or stale input leaves no old paths.
        markers = [Marker(action=Marker.DELETEALL)]
        robot = self._marker(Marker.CUBE, 'robot_symbol', 0, (0.65, 0.70, 0.78, 1.0))
        robot.scale.x, robot.scale.y, robot.scale.z = 0.12, 0.08, 0.025
        robot.pose.position.z = 0.0125
        markers.append(robot)
        heading = self._marker(Marker.ARROW, 'robot_symbol', 1, (0.95, 0.95, 0.95, 1.0))
        heading.scale.x, heading.scale.y, heading.scale.z = 0.008, 0.018, 0.025
        heading.points = [Point(z=0.035), Point(x=0.10, z=0.035)]
        markers.append(heading)
        if self.pair:
            paths, waypoint, _, _ = self.pair
            for index, path in enumerate(paths):
                line = self._marker(Marker.LINE_STRIP, 'candidate_paths', index, (0.20, 0.70, 1.0, 0.65))
                line.scale.x = 0.004
                line.points = [Point()] + list(path)
                markers.append(line)
            chosen = self._marker(Marker.LINE_STRIP, 'selected_path', 0, (0.25, 1.0, 0.45, 1.0))
            chosen.scale.x = 0.009
            chosen.points = [Point()] + list(paths[0])
            markers.append(chosen)
            point = self._marker(Marker.SPHERE, 'selected_waypoint', 0, (1.0, 0.80, 0.12, 1.0))
            point.pose.position = Point(x=waypoint.point.x, y=waypoint.point.y, z=0.015)
            point.scale.x = point.scale.y = point.scale.z = 0.028
            markers.append(point)
        fresh_velocity = self.velocity and mono-self.velocity[2] <= self.p['velocity_timeout_sec']
        v, w = self.velocity[:2] if fresh_velocity else (0.0, 0.0)
        if fresh_velocity and abs(v) > 1e-6:
            arrow = self._marker(Marker.ARROW, 'velocity', 0, (1.0, 0.40, 0.12, 1.0))
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.009, 0.025, 0.04
            arrow.points = [Point(z=0.05), Point(x=v*self.p['arrow_horizon_sec'], z=0.05)]
            markers.append(arrow)
        if fresh_velocity and abs(w) > 1e-6:
            arc = self._marker(Marker.LINE_STRIP, 'angular_velocity', 0, (1.0, 0.40, 0.12, 1.0))
            arc.scale.x = 0.006
            angle = max(-math.pi, min(math.pi, w*self.p['arrow_horizon_sec']))
            arc.points = [Point(x=0.10*math.cos(angle*i/20), y=0.10*math.sin(angle*i/20), z=0.05)
                          for i in range(21)]
            markers.append(arc)
        label = self._marker(Marker.TEXT_VIEW_FACING, 'status', 0, (0.95, 0.95, 0.95, 1.0))
        label.pose.position.x, label.pose.position.z = -0.10, 0.04
        label.scale.z = 0.026
        label.text = (f'LOCAL PREVIEW ONLY | {"fresh prediction" if self.pair else "NO FRESH PREDICTION"}\n'
                      f'v={v:.3f} m/s  w={w:.3f} rad/s  {"" if fresh_velocity else "[velocity stale]"}\n'
                      f'model: {self.model_state} | controller: {self.controller_state}')
        markers.append(label)
        return MarkerArray(markers=markers)

    def _tick(self):
        self.publisher.publish(self.render())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualizationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
