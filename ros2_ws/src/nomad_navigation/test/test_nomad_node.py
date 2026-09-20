"""Safety/transport unit tests use an injected backend, not a fake GPU claim."""

import json
import time

from cv_bridge import CvBridge
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image

from nomad_navigation.nomad_core import ObservationBuffer, select_waypoint
from nomad_navigation.nomad_node import NoMaDNode


def test_buffer_spacing_and_discontinuity():
    buffer = ObservationBuffer(4, 4.0, 1.0, 0.75)
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    for i in range(4):
        stamp = 10**10 + i * 250_000_000
        assert buffer.add(rgb, stamp, 'camera', stamp, i * 0.25)[0]
    assert buffer.ready
    assert not buffer.add(rgb, stamp + 10_000_000, 'camera', stamp + 10_000_000, 0.76)[0]
    assert buffer.ready
    accepted, reason = buffer.add(rgb, stamp + 1_000_000_000, 'camera', stamp + 1_000_000_000, 1.75)
    assert accepted and reason == 'observation_discontinuity' and len(buffer.frames) == 1


@pytest.mark.parametrize('stamp, frame_id', [(0, 'camera'), (1, 'camera'), (20_000_000_000, 'camera'), (10_000_000_000, '')])
def test_invalid_image_time_or_frame(stamp, frame_id):
    buffer = ObservationBuffer(4, 4.0, 1.0, 0.75)
    assert not buffer.add(np.zeros((8, 8, 3), dtype=np.uint8), stamp, frame_id, 10**10, 0)[0]
    assert not buffer.frames


def test_clock_rollback_and_camera_change_reset_context():
    buffer = ObservationBuffer(4, 4.0, 1.0, 0.75)
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    buffer.add(rgb, 10**10, 'front', 10**10, 0)
    _, reason = buffer.add(rgb, 10**10 - 1, 'front', 10**10, 0.1)
    assert reason and len(buffer.frames) == 1
    _, reason = buffer.add(rgb, 10**10 + 250_000_000, 'rear', 10**10 + 250_000_000, 0.25)
    assert reason and len(buffer.frames) == 1
    assert not buffer.fresh(10**10 + 250_000_000, 2.0)  # Even with a frozen ROS clock.


def test_waypoint_selection_and_nominal_scale():
    actions = np.arange(128, dtype=np.float32).reshape(8, 8, 2)
    np.testing.assert_allclose(select_waypoint(actions, 2, 0.2 / 4, 8, 8), [0.2, 0.25])


@pytest.mark.parametrize('actions', [np.full((8, 8, 2), np.nan), np.zeros((1, 8, 2))])
def test_bad_trajectories_rejected(actions):
    with pytest.raises(ValueError):
        select_waypoint(actions, 2, 0.05, 8, 8)


class FakeBackend:
    config = {'context_size': 3, 'image_size': [96, 96], 'len_traj_pred': 8, 'normalize': True}
    context_frames = 4
    warmup_ms = 0.0

    def __init__(self):
        self.calls = 0
        self.error = None
        self.hook = None
        self.frames = None

    def infer(self, frames, sequence):
        self.calls += 1
        self.frames = frames
        if self.error:
            raise self.error
        if self.hook:
            self.hook()
        return np.ones((8, 8, 2), dtype=np.float32), 1.0


@pytest.fixture
def node_and_backend(tmp_path, request):
    context = Context()
    rclpy.init(context=context)
    backend = FakeBackend()
    overrides = [Parameter(key, value=str(tmp_path / key))
                 for key in ('model_config', 'checkpoint', 'action_stats_path', 'output_root')]
    if hasattr(request, 'param'):
        overrides.append(Parameter('image_transport', value=request.param))
    node = NoMaDNode(context=context, backend_factory=lambda params: backend, parameter_overrides=overrides)
    node._timer.cancel()
    yield node, backend
    node.destroy_node()
    context.shutdown()


def send_context(node, count=4):
    bridge = CvBridge()
    now = node.get_clock().now().nanoseconds
    messages = []
    for i in range(count):
        bgr = np.full((8, 8, 3), (10, 20, 30), dtype=np.uint8)
        if node.params['image_transport'] == 'compressed':
            message = bridge.cv2_to_compressed_imgmsg(bgr, dst_format='jpg')
        else:
            message = bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        message.header.frame_id = 'camera_optical_frame'
        stamp = now - (count - 1 - i) * 250_000_000
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        node._on_image(message)
        messages.append(message)
    return messages


def test_wait_context_rgb_conversion_and_no_repeat(node_and_backend):
    node, backend = node_and_backend
    assert node.params['image_transport'] == 'raw'
    assert any(subscription.msg_type is Image for subscription in node.subscriptions)
    send_context(node, 3)
    node._on_tick()
    assert backend.calls == node.published_count == 0
    node.buffer.clear()
    send_context(node)
    node._on_tick()
    assert backend.calls == node.published_count == 1
    np.testing.assert_array_equal(backend.frames[0][0, 0], [30, 20, 10])
    node._on_tick()
    assert backend.calls == 1
    row = json.loads((node.run_dir / 'inference.jsonl').read_text().strip())
    with np.load(node.run_dir / row['archive']) as result:
        np.testing.assert_allclose(result['waypoint'], [0.05, 0.05])
        assert result['trajectories'].shape == (8, 8, 2)
    node.buffer.last_arrival = time.monotonic() - 2
    node._on_tick()
    assert node._state == 'stale_input' and not node.buffer.frames
    assert node.published_count == 1


@pytest.mark.parametrize('node_and_backend', ['raw', 'compressed'], indirect=True)
def test_expiry_during_inference_discards_result(node_and_backend):
    node, backend = node_and_backend
    send_context(node)
    backend.hook = lambda: setattr(node.buffer, 'last_arrival', time.monotonic() - 2)
    node._on_tick()
    assert backend.calls == 1 and node.published_count == 0
    assert node._state == 'stale_input'


def test_inference_exception_latches_fault(node_and_backend):
    node, backend = node_and_backend
    backend.error = RuntimeError('Injected CUDA out of memory')
    send_context(node)
    node._on_tick()
    assert node._faulted and node._state == 'faulted'
    send_context(node)
    node._on_tick()
    assert node.published_count == 0 and backend.calls == 1


def test_malformed_message_clears_history(node_and_backend):
    node, backend = node_and_backend
    send_context(node)
    node._on_image(Image(height=10, width=10, step=30, encoding='rgb8', data=[1]))
    node._on_tick()
    assert not node.buffer.frames and backend.calls == node.published_count == 0


def test_stamped_visualization_positions_match_waypoint(node_and_backend, monkeypatch):
    node, _ = node_and_backend
    poses, points = [], []
    monkeypatch.setattr(node._poses_pub, 'publish', poses.append)
    monkeypatch.setattr(node._stamped_pub, 'publish', points.append)
    send_context(node)
    node._on_tick()
    assert len(poses) == len(points) == 1
    assert poses[0].header == points[0].header
    assert len(poses[0].poses) == 64
    assert poses[0].poses[2].position == points[0].point
    assert poses[0].poses[2].position.x == pytest.approx(0.05)


@pytest.mark.parametrize('node_and_backend', ['compressed'], indirect=True)
def test_compressed_jpeg_rgb_and_original_header(node_and_backend, monkeypatch):
    node, backend = node_and_backend
    assert any(subscription.msg_type is CompressedImage for subscription in node.subscriptions)
    assert node.describe_parameter('image_transport').read_only
    points = []
    monkeypatch.setattr(node._stamped_pub, 'publish', points.append)
    messages = send_context(node)
    stamps = [message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
              for message in messages]
    assert [frame[0] for frame in node.buffer.frames] == stamps
    assert all(frame[2] == 'camera_optical_frame' for frame in node.buffer.frames)
    node._on_tick()
    assert backend.calls == node.published_count == 1
    assert backend.frames[0].dtype == np.uint8
    np.testing.assert_allclose(backend.frames[0][0, 0], [30, 20, 10], atol=3)
    assert points[0].header.stamp == messages[-1].header.stamp
    # Output coordinates remain robot-relative; input frame_id is retained in
    # the observation buffer for camera-change detection, not copied to a pose.
    assert points[0].header.frame_id == node.params['output_frame_id']
    row = json.loads((node.run_dir / 'inference.jsonl').read_text().strip())
    assert row['observation_stamp_ns'] == stamps[-1]
    with np.load(node.run_dir / row['archive']) as result:
        np.testing.assert_array_equal(result['observation_stamps_ns'], stamps)


@pytest.mark.parametrize('node_and_backend', ['compressed'], indirect=True)
@pytest.mark.parametrize('age_seconds', [2, -1])
def test_compressed_stale_or_future_image_is_not_restamped(node_and_backend, age_seconds):
    node, backend = node_and_backend
    messages = send_context(node)
    message = messages[-1]
    stamp = node.get_clock().now().nanoseconds - age_seconds * 10**9
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
    node._on_image(message)
    node._on_tick()
    assert not node.buffer.frames
    assert backend.calls == node.published_count == 0
    assert node._state == 'waiting_for_images'
    events = [json.loads(line) for line in (node.run_dir / 'events.jsonl').read_text().splitlines()]
    assert any(event.get('reason') == 'invalid_or_stale_stamp' for event in events)


@pytest.mark.parametrize('node_and_backend', ['compressed'], indirect=True)
@pytest.mark.parametrize('data', [b'', b'not a jpeg image\x00\xff'])
def test_malformed_compressed_image_clears_history(node_and_backend, data):
    node, backend = node_and_backend
    messages = send_context(node)
    message = CompressedImage(header=messages[-1].header, format='jpeg', data=data)
    node._on_image(message)
    node._on_tick()
    assert not node.buffer.frames
    assert backend.calls == node.published_count == 0
    assert not node._faulted and node._state == 'waiting_for_images'
    events = [json.loads(line) for line in (node.run_dir / 'events.jsonl').read_text().splitlines()]
    assert events[-1]['event'] == 'image_rejected'


@pytest.mark.parametrize('node_and_backend', ['compressed'], indirect=True)
def test_compressed_camera_change_and_timeout_keep_guards(node_and_backend):
    node, backend = node_and_backend
    message = send_context(node)[-1]
    message.header.frame_id = 'different_camera_optical_frame'
    # Same timestamp and a different source cannot complete an old context.
    node._on_image(message)
    assert len(node.buffer.frames) == 1
    assert node.buffer.frames[0][2] == message.header.frame_id
    node._on_tick()
    assert backend.calls == node.published_count == 0
    node.buffer.clear()
    send_context(node)
    node.buffer.last_arrival = time.monotonic() - 2
    node._on_tick()
    assert not node.buffer.frames and node._state == 'stale_input'
    assert backend.calls == node.published_count == 0


@pytest.mark.parametrize('transport', ['', 'jpeg', 'compressedDepth', 'RAW'])
def test_invalid_image_transport_fails_before_model_load(tmp_path, transport):
    context = Context()
    rclpy.init(context=context)
    called = []
    overrides = [Parameter(key, value=str(tmp_path / key))
                 for key in ('model_config', 'checkpoint', 'action_stats_path', 'output_root')]
    overrides.append(Parameter('image_transport', value=transport))
    try:
        with pytest.raises(ValueError, match='image_transport must be raw or compressed'):
            NoMaDNode(context=context, backend_factory=lambda params: called.append(params),
                      parameter_overrides=overrides)
        assert not called
        assert not (tmp_path / 'output_root').exists()
    finally:
        context.shutdown()
