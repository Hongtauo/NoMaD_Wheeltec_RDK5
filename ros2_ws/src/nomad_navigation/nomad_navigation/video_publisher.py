"""Replay a local constant-frame-rate video as sensor-data ROS Image messages."""

import math
from pathlib import Path
import signal

import cv2
from cv_bridge import CvBridge
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FrameSchedule:
    """Sample source frames on a video-time grid; end_sec is exclusive.自定义的 Python 类，用来计算“第几次采样，应该取原视频的哪一帧”"""

    def __init__(self, source_fps, frame_count, publish_fps, start_sec=0.0, end_sec=-1.0):
        values = (source_fps, publish_fps, start_sec, end_sec)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('FPS and time parameters must be finite')
        if source_fps <= 0 or frame_count <= 0:
            raise ValueError('Video must report a positive FPS and frame count')
        if publish_fps <= 0 or publish_fps > source_fps:
            raise ValueError('publish_fps must be positive and no greater than source FPS')
        duration = frame_count / source_fps
        if start_sec < 0 or start_sec >= duration:
            raise ValueError('start_sec must be within the video')
        if end_sec != -1.0 and (end_sec <= start_sec or end_sec > duration):
            raise ValueError('end_sec must be -1 or greater than start_sec and within the video')
        self.first = math.ceil(start_sec * source_fps - 1e-9)
        self.stop = frame_count if end_sec == -1.0 else math.ceil(end_sec * source_fps - 1e-9)
        if self.first >= self.stop:
            raise ValueError('Selected interval contains no video frames')
        self.ratio = source_fps / publish_fps

    def frame_at(self, sample_index):
        # self.ratio就是实际视频的帧率/目标帧率的比值，通过加上一个极小的数1e-9然后向下取整，通过相对first帧的偏移计算帧号，返回帧号index
        index = self.first + math.floor(sample_index * self.ratio + 1e-9)
        return index if index < self.stop else None


class VideoPublisher(Node):
    """Publish BGR8 images only; never publishes waypoints or motion commands."""

    def __init__(self, **kwargs):
        super().__init__('video_publisher', **kwargs)
        self._capture = None
        self.finished = False
        self.published_count = 0
        self.loop_count = 0
        try:
            # 默认参数列表
            defaults = {
                'video_path': '',
                'image_topic': '/camera/image_raw',
                'frame_id': 'camera_optical_frame',
                'publish_fps': 4.0,
                'loop': False,
                'start_sec': 0.0,
                'end_sec': -1.0,
                'resize_width': 320,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            params = {name: self.get_parameter(name).value for name in defaults}
            if not params['video_path']:
                raise ValueError('video_path is required; pass a local video filename')
            path = Path(params['video_path']).expanduser().resolve()
            if not path.is_file():
                raise ValueError(f'Video file does not exist: {path}')
            if not params['frame_id'].strip() or params['frame_id'].startswith('/'):
                raise ValueError('frame_id must be nonempty and must not start with /')
            if not params['image_topic'].strip():
                raise ValueError('image_topic must not be empty')
            if params['resize_width'] < 0:
                raise ValueError('resize_width must be nonnegative; 0 keeps original resolution')
            self._capture = cv2.VideoCapture(str(path))
            if not self._capture.isOpened():
                raise ValueError(f'Cannot open video: {path}')
            source_fps = self._capture.get(cv2.CAP_PROP_FPS)
            frame_count = int(self._capture.get(cv2.CAP_PROP_FRAME_COUNT))
            # 创建抽帧计算器，传入
            self._schedule = FrameSchedule(
                source_fps, # 原始视频的帧率
                frame_count, # 原始视频的总帧数
                params['publish_fps'], # 目标发布帧率
                params['start_sec'], # 从视频第几秒开始
                params['end_sec']) # 视频的第几秒结束
            self._params = params
            self._sample_index = 0
            self._next_source_frame = 0
            self._bridge = CvBridge()
            # 创建信息发布者，发布类型是Image，话题是"/camera/image_raw"，通信策略（QoS）是高频传感器数据qos_profile_sensor_data，规定消息传输的可靠性、缓存等行为，适用于传感器数据
            self._publisher = self.create_publisher(
                Image, params['image_topic'], qos_profile_sensor_data)

            """
                创建定时器：默认 publish_fps=4，即每 0.25 秒触发一次 _publish_next。
                回调根据当前采样序号和原视频帧率计算原视频中的绝对帧号，
                读取该帧并发布，实现按设定频率抽帧。
            """
            self._timer = self.create_timer(1.0 / params['publish_fps'], self._publish_next)
            self.get_logger().info(
                f'Video ready: {path}; source={source_fps:g} FPS/{frame_count} frames; '
                f'publish={params["publish_fps"]:g} Hz; topic={self._publisher.topic_name}; '
                f'loop={params["loop"]}; QoS=best_effort/volatile/depth5; encoding=bgr8')
        except Exception:
            self.destroy_node()
            raise

    # 读取帧图像
    def _read_frame(self, index):
        # Decode forward rather than seeking every sampled frame. Seek only on
        # initial offset or loop restart, so long videos remain inexpensive.
        if index < self._next_source_frame or (self._next_source_frame == 0 and index > 0):
            if not self._capture.set(cv2.CAP_PROP_POS_FRAMES, index):
                raise RuntimeError(f'Video seek failed at frame {index}')
            self._next_source_frame = index
        while self._next_source_frame <= index:
            if not self._capture.grab():
                raise RuntimeError(f'Unexpected decode failure at frame {self._next_source_frame}')
            self._next_source_frame += 1
        ok, frame = self._capture.retrieve()
        if not ok or frame is None:
            raise RuntimeError(f'Video frame retrieval failed at frame {index}')
        return frame

    def _publish_next(self):
        # index是加上视频起始帧self.first的视频种的绝对帧号，绝对帧号是通过采样帧算出来的
        index = self._schedule.frame_at(self._sample_index)
        if index is None:
            if not self._params['loop']:
                self.finished = True
                self._timer.cancel()
                self._capture.release()
                self.get_logger().info(f'Playback complete; published={self.published_count}')
                return
            self.loop_count += 1
            self._sample_index = 0
            index = self._schedule.frame_at(0)
            self.get_logger().info(
                f'Loop restart={self.loop_count}; video content jumps to selected start')
        # 读取绝对帧号所在帧
        frame = self._read_frame(index)
        width = self._params['resize_width']
        if width and width != frame.shape[1]:
            height = max(1, round(frame.shape[0] * width / frame.shape[1]))
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        # 将图像通过_bridge转换为图像消息message
        message = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # Replay acts as a live camera: stamp simulated acquisition now, not
        # the original recording's timestamp. No CameraInfo/TF is fabricated.
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._params['frame_id']
        # 调用发布函数发布消息
        self._publisher.publish(message)
        # 采样帧自增1
        self._sample_index += 1
        self.published_count += 1
        if self.published_count == 1 or self.published_count % max(1, round(self._params['publish_fps'] * 5)) == 0:
            self.get_logger().info(
                f'published={self.published_count}; source_frame={index}; '
                f'image={message.width}x{message.height}; loops={self.loop_count}')

    def destroy_node(self):
        if self._capture is not None:
            self._capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = VideoPublisher()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        rclpy.logging.get_logger('video_publisher').error(str(error))
        result = 1
    finally:
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
