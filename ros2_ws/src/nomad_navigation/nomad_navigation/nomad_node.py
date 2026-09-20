"""Goal-free NoMaD ROS 2 inference. No Twist publisher or robot control here."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import signal
import time
import uuid

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseArray
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, String
import yaml

from nomad_navigation.nomad_core import NoMaDBackend, ObservationBuffer, select_waypoint


class NoMaDNode(Node):
    def __init__(self, backend_factory=NoMaDBackend, **kwargs):
        super().__init__('nomad_inference', **kwargs)
        self.backend = None
        self._events = None
        self._rows = None
        self.run_dir = None
        self._manifest = {}
        self._state = None
        self._faulted = False
        self._closed = False
        self.published_count = 0
        self._attempt = 0
        self._last_processed = None
        try:
            defaults = {
                'model_config': '', 'checkpoint': '', 'action_stats_path': '', 'output_root': '',
                'run_name': 'nomad-goalfree', 'device': 'cuda', 'warmup': True,
                'image_topic': '/camera/image_raw', 'image_transport': 'raw',
                'waypoint_topic': '/waypoint',
                'sampled_actions_topic': '/sampled_actions',
                'stamped_waypoint_topic': '/waypoint_stamped', 'status_topic': '/nomad/status',
                'trajectory_poses_topic': '/nomad/trajectory_poses',
                'output_frame_id': 'base_link', 'observation_hz': 4.0, 'inference_hz': 4.0,
                'num_samples': 8, 'waypoint_index': 2, 'max_speed_mps': 0.2,
                'image_timeout_sec': 1.0, 'max_frame_gap_sec': 0.75,
                'future_tolerance_sec': 0.2, 'seed': 0,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            self.params = {key: self.get_parameter(key).value for key in defaults}
            p = self.params
            if p['image_transport'] not in ('raw', 'compressed'):
                raise ValueError('image_transport must be raw or compressed')
            for key in ('model_config', 'checkpoint', 'action_stats_path', 'output_root'):
                if not p[key].strip():
                    raise ValueError(f'{key} is required')
                p[key] = str(Path(p[key]).expanduser().resolve())
            for key in ('observation_hz', 'inference_hz', 'max_speed_mps',
                        'image_timeout_sec', 'max_frame_gap_sec'):
                if not math.isfinite(p[key]) or p[key] <= 0:
                    raise ValueError(f'{key} must be finite and positive')
            if not math.isfinite(p['future_tolerance_sec']) or p['future_tolerance_sec'] < 0:
                raise ValueError('future_tolerance_sec must be finite and nonnegative')
            if not 0 < p['num_samples'] <= 64 or p['waypoint_index'] < 0 or p['seed'] < 0:
                raise ValueError('Invalid sample count, waypoint index or seed')
            if p['inference_hz'] > p['observation_hz']:
                raise ValueError('inference_hz must not exceed observation_hz')
            if not 1 / p['observation_hz'] < p['max_frame_gap_sec'] <= p['image_timeout_sec']:
                raise ValueError('Require observation period < max_frame_gap_sec <= image_timeout_sec')
            topics = [p[key] for key in defaults if key.endswith('_topic')]
            if any(not topic.strip() for topic in topics) or len(set(topics)) != len(topics):
                raise ValueError('Input and output topic names must be nonempty and distinct')
            if not p['output_frame_id'].strip() or p['output_frame_id'].startswith('/'):
                raise ValueError('output_frame_id must be nonempty and have no leading slash')
            root = Path(p['output_root'])
            root.mkdir(parents=True, exist_ok=True)
            prefix = re.sub(r'[^A-Za-z0-9._-]+', '-', p['run_name']).strip('-._') or 'nomad'
            candidate = root / (prefix + '__nomad-goalfree__' +
                                datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') +
                                '__' + uuid.uuid4().hex[:8])
            candidate.mkdir(exist_ok=False)
            self.run_dir = candidate
            (self.run_dir / 'windows').mkdir()
            self._events = (self.run_dir / 'events.jsonl').open('w', encoding='utf-8')
            self._rows = (self.run_dir / 'inference.jsonl').open('w', encoding='utf-8')
            self._manifest = {'status': 'loading', 'parameters': p, 'published_count': 0,
                              'created_utc': datetime.now(timezone.utc).isoformat()}
            self._write_manifest()
            self.get_logger().info(f'Loading NoMaD; output={self.run_dir}')
            self.backend = backend_factory(p)
            config = self.backend.config
            if p['waypoint_index'] >= config['len_traj_pred']:
                raise ValueError('waypoint_index is outside the configured trajectory')
            self._scale = p['max_speed_mps'] / p['observation_hz'] if config['normalize'] else 1.0
            self.buffer = ObservationBuffer(self.backend.context_frames, p['observation_hz'],
                                            p['image_timeout_sec'], p['max_frame_gap_sec'],
                                            p['future_tolerance_sec'])
            self._bridge = CvBridge()
            command_qos = QoSProfile(depth=1, lifespan=Duration(seconds=p['image_timeout_sec']))
            self._waypoint_pub = self.create_publisher(Float32MultiArray, p['waypoint_topic'], command_qos)
            self._actions_pub = self.create_publisher(Float32MultiArray, p['sampled_actions_topic'], command_qos)
            self._stamped_pub = self.create_publisher(PointStamped, p['stamped_waypoint_topic'], command_qos)
            self._poses_pub = self.create_publisher(PoseArray, p['trajectory_poses_topic'], command_qos)
            self._status_pub = self.create_publisher(String, p['status_topic'], QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            image_type = Image if p['image_transport'] == 'raw' else CompressedImage
            self.create_subscription(image_type, p['image_topic'], self._on_image, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self._timer = self.create_timer(1 / p['inference_hz'], self._on_tick)
            self._manifest.update(model=config, context_frames=self.backend.context_frames,
                                  waypoint_scale=self._scale, warmup_ms=self.backend.warmup_ms,
                                  runtime=getattr(self.backend, 'runtime_info', {'backend': 'injected-test'}))
            (self.run_dir / 'model_config.yaml').write_text(yaml.safe_dump(config), encoding='utf-8')
            stats_path = Path(p['action_stats_path'])
            if stats_path.is_file():
                (self.run_dir / 'action_stats.yaml').write_text(stats_path.read_text(encoding='utf-8'), encoding='utf-8')
            self._set_state('waiting_for_images')
            self.get_logger().info(
                f'NoMaD ready; context={self.backend.context_frames}; scale={self._scale:g}; '
                'goal-free; no velocity publisher')
        except Exception as error:
            self._faulted = True
            if self.run_dir is not None:
                self._manifest.update(status='failed', error=repr(error))
                self._write_manifest()
            self.destroy_node()
            raise

    def _write_manifest(self):
        (self.run_dir / 'manifest.json').write_text(json.dumps(self._manifest, indent=2) + '\n', encoding='utf-8')

    def _event(self, event, **fields):
        self._events.write(json.dumps({'event': event, 'time_ns': self.get_clock().now().nanoseconds, **fields}) + '\n')
        self._events.flush()

    def _set_state(self, state):
        if self._state == state:
            return
        self._state = state
        self._event('state', state=state)
        self._manifest.update(status=state, published_count=self.published_count)
        self._write_manifest()
        self._status_pub.publish(String(data=json.dumps({
            'state': state, 'run_dir': str(self.run_dir), 'published_count': self.published_count,
            'trajectory_layout': {'samples': self.params['num_samples'],
                                  'steps': self.backend.config['len_traj_pred'],
                                  'waypoint_index': self.params['waypoint_index'],
                                  'frame_id': self.params['output_frame_id'],
                                  'scale_applied': self._scale}})))
        self.get_logger().info(f'state={state}')

    def _on_image(self, message):
        if self._faulted:
            return
        try:
            if self.params['image_transport'] == 'raw':
                if message.height <= 0 or message.width <= 0 or len(message.data) != message.step * message.height:
                    raise ValueError('Malformed Image dimensions/data length')
                rgb = self._bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            else:
                if not message.data:
                    raise ValueError('Empty CompressedImage data')
                rgb = self._bridge.compressed_imgmsg_to_cv2(message, desired_encoding='rgb8')
            if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError('Expected uint8 RGB image')
            # Both transports retain the camera's original timestamp and frame.
            # Decoding is not a new observation and must never refresh stale data.
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            _, reason = self.buffer.add(rgb, stamp, message.header.frame_id,
                                        self.get_clock().now().nanoseconds, time.monotonic())
            if reason:
                self._event('context_reset', reason=reason)
                self._set_state('waiting_for_images')
        except Exception as error:
            self.buffer.clear()
            self._event('image_rejected', error=repr(error))
            self.get_logger().warning(f'Image rejected: {error}')
            self._set_state('waiting_for_images')

    def _on_tick(self):
        if self._faulted:
            return
        now = self.get_clock().now().nanoseconds
        if not self.buffer.fresh(now, time.monotonic()):
            if self.buffer.frames:
                self.buffer.clear()
                self._set_state('stale_input')
            return
        if not self.buffer.ready:
            return
        stamp = self.buffer.frames[-1][0]
        if stamp == self._last_processed:
            return
        self._last_processed = stamp
        snapshot = list(self.buffer.frames)
        try:
            actions, elapsed_ms = self.backend.infer([frame[1] for frame in snapshot], self._attempt)
            self._attempt += 1
            waypoint = select_waypoint(actions, self.params['waypoint_index'], self._scale,
                                       self.params['num_samples'], self.backend.config['len_traj_pred'])
            # Synchronous inference may outlast the input timeout. Never publish
            # its result just because the image was fresh when inference began.
            if not self.buffer.fresh(self.get_clock().now().nanoseconds, time.monotonic()):
                self.buffer.clear()
                self._event('result_discarded', reason='input_expired_during_inference', inference_ms=elapsed_ms)
                self._set_state('stale_input')
                return
            index = self.published_count
            archive = f'windows/attempt_{self._attempt - 1:06d}.npz'
            np.savez_compressed(self.run_dir / archive,
                                trajectories=actions, waypoint=waypoint,
                                observation_stamps_ns=np.asarray([frame[0] for frame in snapshot], dtype=np.int64),
                                current_rgb=snapshot[-1][1])
            if not self.buffer.fresh(self.get_clock().now().nanoseconds, time.monotonic()):
                self.buffer.clear()
                self._event('result_discarded', reason='input_expired_during_recording', archive=archive)
                self._set_state('stale_input')
                return
            raw_message = Float32MultiArray(data=np.concatenate(([0.0], actions.flatten())).astype(np.float32).tolist())
            point = PointStamped()
            point.header.stamp.sec, point.header.stamp.nanosec = divmod(stamp, 10**9)
            point.header.frame_id = self.params['output_frame_id']
            point.point.x, point.point.y = map(float, waypoint)
            # Visualization-only stamped positions: sample-major, already in the
            # same nominal metric scale as waypoint. Orientations are placeholders.
            poses = PoseArray(header=point.header)
            for x, y in (actions * self._scale).reshape(-1, 2):
                pose = Pose()
                pose.position.x, pose.position.y = float(x), float(y)
                pose.orientation.w = 1.0
                poses.poses.append(pose)
            self._actions_pub.publish(raw_message)
            self._stamped_pub.publish(point)
            self._poses_pub.publish(poses)
            self._waypoint_pub.publish(Float32MultiArray(data=waypoint.tolist()))
            self.published_count += 1
            row = {'index': index, 'archive': archive, 'observation_stamp_ns': stamp,
                   'published_ns': self.get_clock().now().nanoseconds,
                   'inference_ms': elapsed_ms, 'waypoint': waypoint.tolist(),
                   'waypoint_scale': self._scale, 'seed': self.params['seed'] + self._attempt - 1}
            self._rows.write(json.dumps(row) + '\n')
            self._rows.flush()
            self._set_state('running')
            self.get_logger().info(f'waypoint[{index}]={waypoint.tolist()}; inference={elapsed_ms:.1f} ms')
        except Exception as error:
            self._faulted = True
            self.buffer.clear()
            self._timer.cancel()
            self._event('inference_error', error=repr(error))
            self._set_state('faulted')
            self.get_logger().error(f'Inference disabled until restart: {type(error).__name__}: {error}')

    def destroy_node(self):
        if not self._closed:
            self._closed = True
            if self.run_dir is not None:
                self._manifest.update(status='failed' if self._faulted else 'stopped',
                                      published_count=self.published_count)
                self._write_manifest()
            for handle in (self._rows, self._events):
                if handle is not None:
                    handle.close()
            self.backend = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = NoMaDNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        rclpy.logging.get_logger('nomad_inference').error(f'{type(error).__name__}: {error}')
        result = 1
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                result = 1 if node._faulted else result
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
