"""Video sampling, validation, Image transport and end-of-file tests."""

import time

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from nomad_navigation.video_publisher import FrameSchedule, VideoPublisher


def test_sample_on_video_time_grid():
    schedule = FrameSchedule(30.0, 300, 4.0, 1.0, 2.0)
    assert [schedule.frame_at(i) for i in range(5)] == [30, 37, 45, 52, None]


@pytest.mark.parametrize('kwargs', [
    {'publish_fps': 0.0}, {'publish_fps': 31.0}, {'publish_fps': float('nan')},
    {'start_sec': -1.0}, {'start_sec': 10.0}, {'end_sec': 0.0},
    {'end_sec': 11.0}, {'source_fps': 0.0}, {'frame_count': 0},
])
def test_invalid_schedule(kwargs):
    params = dict(source_fps=30.0, frame_count=300, publish_fps=4.0)
    params.update(kwargs)
    with pytest.raises(ValueError):
        FrameSchedule(**params)


@pytest.fixture
def video_path(tmp_path):
    path = tmp_path / 'synthetic.avi'
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 10.0, (64, 48))
    assert writer.isOpened()
    for index in range(20):
        frame = np.full((48, 64, 3), (index * 10, 40, 180), dtype=np.uint8)
        writer.write(frame)
    writer.release()
    return path


@pytest.fixture
def ros_context():
    context = Context()
    rclpy.init(context=context)
    yield context
    context.shutdown()


def test_missing_video_fails_cleanly(ros_context, tmp_path):
    with pytest.raises(ValueError, match='does not exist'):
        VideoPublisher(context=ros_context, parameter_overrides=[
            Parameter('video_path', value=str(tmp_path / 'missing.mp4'))])


@pytest.mark.parametrize('loop', [False, True])
def test_images_range_loop_and_release(video_path, ros_context, loop):
    node = VideoPublisher(context=ros_context, parameter_overrides=[
        Parameter('video_path', value=str(video_path)),
        Parameter('publish_fps', value=5.0),
        Parameter('start_sec', value=0.2),
        Parameter('end_sec', value=1.0),
        Parameter('resize_width', value=32),
        Parameter('loop', value=loop),
    ])
    node._timer.cancel()  # Drive frame callbacks deterministically in this test.
    receiver = Node('video_unit_receiver', context=ros_context)
    received = []
    receiver.create_subscription(Image, '/camera/image_raw', received.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(receiver)
    executor.add_node(node)
    bridge = CvBridge()
    try:
        assert node._publisher.qos_profile.depth == 5
        deadline = time.monotonic() + 5
        while node._publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert node._publisher.get_subscription_count() == 1
        count = 6 if loop else 4
        for i in range(count):
            node._publish_next()
            deadline = time.monotonic() + 3
            while len(received) <= i and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            assert len(received) == i + 1
        assert node.published_count == count
        for message in received:
            assert (message.width, message.height, message.encoding) == (32, 24, 'bgr8')
            assert message.header.frame_id == 'camera_optical_frame'
            assert message.header.stamp.sec > 0
            assert len(message.data) == message.step * message.height
        if loop:
            assert node.loop_count == 1
            assert not node.finished
            np.testing.assert_array_equal(
                bridge.imgmsg_to_cv2(received[0]), bridge.imgmsg_to_cv2(received[4]))
        else:
            node._publish_next()
            assert node.finished and not node._capture.isOpened()
            values = [int(bridge.imgmsg_to_cv2(msg)[0, 0, 0]) for msg in received]
            np.testing.assert_allclose(values, [20, 40, 60, 80], atol=3)
        stamps = [msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec for msg in received]
        assert all(b > a for a, b in zip(stamps, stamps[1:]))
    finally:
        executor.shutdown()
        receiver.destroy_node()
        node.destroy_node()
    assert not node._capture.isOpened()
