"""Robot-local NoMaD HTTPS client; no arm calls and no velocity publisher.

Defaults to shadow outputs. --control-output only selects reviewed model topics;
the independent local controller/gateway still require explicit operator arming.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

from nomad_navigation.campus_client_core import (
    CampusClientCore, HTTPSInference, OneRequestWorker, REQUEST_DEADLINE_SEC,
)
from nomad_navigation.campus_upload_transport import CompactUploadTransport


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--server-url', required=True)
    result.add_argument('--ca-file', required=True)
    result.add_argument('--token-file', required=True)
    result.add_argument('--seconds', type=float, default=0.0)
    result.add_argument('--output-root', required=True)
    result.add_argument('--control-output', action='store_true')
    result.add_argument('--network-pause-resume', action='store_true',
                        help='Keep transient network/camera waits alive; never publish an expired result')
    return result


def require_local_ros():
    for name, expected in (('ROS_DOMAIN_ID', '0'), ('ROS_LOCALHOST_ONLY', '1')):
        if os.environ.get(name, expected) != expected:
            raise ValueError(name + '=' + expected + ' is mandatory for the campus client')
        os.environ[name] = expected


def require_control_session(control_output):
    if control_output:
        from nomad_navigation.robot_session_guard import require_robot_session
        return require_robot_session()
    return None


def create_node(args, transport):
    # Import ROS only at the executable boundary, never in protocol/core tests.
    from geometry_msgs.msg import PointStamped, Pose, PoseArray
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    class CampusAPINode(Node):
        def __init__(self):
            super().__init__('campus_api_client', namespace='/nomad')
            self.core = CampusClientCore(latch_faults=args.control_output,
                network_pause_resume=getattr(args, 'network_pause_resume', False))
            self.worker = OneRequestWorker(transport)
            self.control_output = args.control_output
            self.prefix = '/nomad/robot_preview' if args.control_output else '/nomad/campus_shadow'
            self.topics = {key: self.prefix + '/' + key for key in
                           ('waypoint_stamped', 'status', 'trajectory_poses')}
            self.started = time.monotonic()
            self.last_graph = 0.0
            self.events = []
            self.previous_state = None
            self.telemetry = {}
            self.closed = False
            self.timer = None
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Campus client requires the real robot clock')
            if self.get_fully_qualified_name() != '/nomad/campus_api_client':
                raise ValueError('Client identity remapping is forbidden')
            for topic in self.topics.values():
                if self.resolve_topic_name(topic) != topic:
                    raise ValueError('Output remapping is forbidden')
                if self.get_publishers_info_by_topic(topic):
                    raise ValueError('Output already has a publisher: ' + topic)
            prediction_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        lifespan=Duration(seconds=.6))
            self.point_pub = self.create_publisher(PointStamped, self.topics['waypoint_stamped'], prediction_qos)
            self.pose_pub = self.create_publisher(PoseArray, self.topics['trajectory_poses'], prediction_qos)
            self.status_pub = self.create_publisher(String, self.topics['status'], QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
            sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed',
                                     self.on_camera, sensor_qos)
            status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/nomad/robot_control/status',
                                     lambda msg: self.on_status('controller_status', msg), status_qos)
            self.create_subscription(String, '/nomad/robot_gateway/status',
                                     lambda msg: self.on_status('gateway_status', msg), 1)
            self.create_subscription(Odometry, '/odom', self.on_odom, sensor_qos)
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(.05, self.tick, clock=self.steady)
            self.publish_status()

        def now(self):
            return self.get_clock().now().nanoseconds, time.monotonic()

        @staticmethod
        def stamp(message):
            return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

        def on_camera(self, message):
            now, steady = self.now()
            self.core.add_camera(bytes(message.data), self.stamp(message),
                                 message.header.frame_id, now, steady)
            if self.core.state != 'running':
                self.publish_status()

        def on_status(self, key, message):
            if len(message.data) > 32768:
                self.telemetry.pop(key, None)
                return
            try:
                from nomad_navigation.campus_protocol import decode_json
                value = decode_json(message.data.encode('utf-8'))
                if not isinstance(value, dict):
                    raise ValueError('Status must be an object')
                self.telemetry[key] = value
            except (ValueError, TypeError):
                self.telemetry.pop(key, None)

        def on_odom(self, message):
            linear, angular = message.twist.twist.linear.x, message.twist.twist.angular.z
            if not math.isfinite(linear) or not math.isfinite(angular):
                self.telemetry.pop('odom', None)
                return
            stamp = self.stamp(message)
            if stamp <= 0:
                self.telemetry.pop('odom', None)
                return
            self.telemetry['odom'] = dict(stamp_ns=stamp, frame_id=message.header.frame_id,
                                         child_frame_id=message.child_frame_id,
                                         linear_x=float(linear), angular_z=float(angular))

        def check_graph(self):
            for topic in self.topics.values():
                publishers = self.get_publishers_info_by_topic(topic)
                if len(publishers) > 1 or any(
                        item.node_name != 'campus_api_client' or item.node_namespace != '/nomad'
                        for item in publishers):
                    self.core.fail('competing_model_publisher', fatal=True)
                    self.worker.transport.cancel()
                    return False
            return True

        def publish_status(self):
            snapshot = self.core.snapshot()
            snapshot.update(control_output=self.control_output, transport='https',
                            node='/nomad/campus_api_client', direct_cmd_vel_publisher=False,
                            hardware_stop_guaranteed=False, obstacle_protection=False)
            self.status_pub.publish(String(data=json.dumps(snapshot, allow_nan=False)))
            if self.core.state != self.previous_state:
                event = dict(elapsed_sec=round(time.monotonic()-self.started, 6),
                             state=self.core.state, sequence=self.core.sequence)
                self.events.append(event)
                if len(self.events) > 1024:
                    del self.events[:len(self.events)-1024]
                self.previous_state = self.core.state
                self.get_logger().info('Campus API: ' + self.core.state)

        def publish_prediction(self, response):
            if not self.check_graph():
                return
            point = PointStamped()
            stamp = response['observation_stamp_ns']
            point.header.stamp.sec, point.header.stamp.nanosec = divmod(stamp, 10**9)
            point.header.frame_id = 'base_link'
            point.point.x, point.point.y = map(float, response['waypoint'])
            poses = PoseArray()
            poses.header = point.header
            for trajectory in response['trajectories']:
                for x, y in trajectory:
                    pose = Pose()
                    pose.position.x, pose.position.y = float(x), float(y)
                    pose.orientation.w = 1.0
                    poses.poses.append(pose)
            self.pose_pub.publish(poses)
            self.point_pub.publish(point)

        def tick(self):
            now, steady = self.now()
            self.core.watchdog(now, steady)
            # Do not hide a brief watchdog fault behind a result that happens
            # to finish in the same timer callback: the controller must disarm.
            if self.core.state != 'running':
                self.publish_status()
            if steady-self.last_graph >= .25:
                self.last_graph = steady
                self.check_graph()
            if self.core.pending is not None and steady-self.core.pending['started'] >= REQUEST_DEADLINE_SEC:
                self.worker.transport.cancel()
            item = self.worker.take()
            if item is not None:
                _, response, error, completed_at = item
                result = self.core.complete(response, error, completed_at, now, steady)
                if result is not None:
                    self.publish_prediction(result)
            # Always publish failure before trying to recover, so a controller
            # receiving fresh data after reconnection still needs explicit arm.
            self.publish_status()
            if not self.worker.busy:
                request = self.core.request(now, steady, dict(self.telemetry))
                if request is not None and not self.worker.submit(request, steady):
                    self.core.fail('worker_unavailable', fatal=True)

        def close(self):
            if self.closed:
                return True
            self.closed = True
            if self.timer is not None:
                self.timer.cancel()
            self.core.fail('stopped', clear=True)
            self.publish_status()
            try:
                self.status_pub.wait_for_all_acked(Duration(seconds=.1))
            finally:
                return self.worker.close()

    return CampusAPINode()


def main(argv=None):
    args = parser().parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 0:
        raise ValueError('--seconds must be finite and nonnegative; zero means no session limit')
    require_local_ros()
    # Shadow preview needs no motion authority. Formal model outputs require
    # the same enrolled-robot and recent clock/session checks as local control.
    require_control_session(args.control_output)
    transport = CompactUploadTransport(HTTPSInference(args.server_url, args.ca_file, args.token_file))
    transport.warmup()  # Before the ROS subscriptions and all request deadlines.
    output = Path(args.output_root).resolve() / (
        'campus_client__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    stopping = False
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    result = 0
    report = dict(control_output=args.control_output, server_url=args.server_url,
                  auto_arm=False, direct_cmd_vel_publisher=False)
    try:
        node = create_node(args, transport)
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.core.fatal:
                raise RuntimeError('Client failed closed: ' + node.core.state)
            if args.seconds and time.monotonic()-node.started >= args.seconds:
                break
    except Exception as error:
        result = 1
        # Do not include raw exception text: it may contain URLs or credentials.
        report['failure_type'] = type(error).__name__
    finally:
        if node is not None:
            report.update(node.core.snapshot())
            report['events'] = node.events
            try:
                report['worker_stopped'] = node.close()
                if not report['worker_stopped']:
                    result = 1
            except Exception as error:
                report['cleanup_failure_type'] = type(error).__name__
                result = 1
                transport.cancel()
            node.destroy_node()
        else:
            transport.cancel()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        report['upload_transport'] = transport.snapshot()
        report['exit_code'] = result
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(exit_code=result, report=str(output/'report.json'))), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
