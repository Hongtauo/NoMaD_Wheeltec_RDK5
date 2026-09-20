"""Control-law, failure gates, dry-run routing and steady watchdog tests."""

import csv
import math
import time

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from nomad_navigation.controller_core import ControlLimits, SafetyGate, waypoint_velocity
from nomad_navigation.controller_node import ControllerNode


@pytest.mark.parametrize('x,y,v,w,reason', [
    (0.1, 0.0, 0.2, 0.0, 'tracking'),
    (0.03, 0.0, 0.12, 0.0, 'tracking'),
    (0.1, 0.1, 0.2, 0.4, 'tracking'),
    (0.1, -0.1, 0.2, -0.4, 'tracking'),
    (0.0, 0.1, 0.0, 0.4, 'tracking'),
    (0.0, -0.1, 0.0, -0.4, 'tracking'),
    (0.0, 0.0, 0.0, 0.0, 'within_tolerance'),
    (-0.1, 0.1, 0.0, 0.0, 'rear_waypoint'),
    (math.nan, 0.1, 0.0, 0.0, 'invalid_waypoint'),
    (10.0, 0.1, 0.0, 0.0, 'waypoint_out_of_range'),
])
def test_velocity(x, y, v, w, reason):
    actual_v, actual_w, actual_reason = waypoint_velocity(x, y, ControlLimits())
    assert (actual_v, actual_w) == pytest.approx((v, w))
    assert actual_reason == reason


@pytest.mark.parametrize('value', [0.0, -1.0, math.nan, math.inf])
def test_invalid_limits(value):
    with pytest.raises(ValueError):
        ControlLimits(max_linear_mps=value)


def ready_gate():
    gate = SafetyGate(ControlLimits())
    gate.set_model_state('running')
    gate.accept_camera(10**10, 'camera_optical_frame', 10**10, 0.0)
    assert gate.accept_waypoint(0.1, 0.0, 0.0, 10**10, 'base_link', 10**10, 0.0)
    return gate


@pytest.mark.parametrize('kind', ['receipt', 'source', 'camera', 'clock', 'model'])
def test_independent_stop_gates(kind):
    gate = ready_gate()
    now, monotonic = 10**10, 0.0
    if kind == 'receipt':
        now += 700_000_000
        monotonic = 0.7
        gate.accept_camera(now, 'camera_optical_frame', now, monotonic)
        expected = 'waypoint_timeout'
    elif kind == 'source':
        gate.waypoint = (0.1, 0.0, 9_100_000_000, 0.0)
        now += 200_000_000
        monotonic = 0.2
        expected = 'source_image_timeout'
    elif kind == 'camera':
        now += 1_100_000_000
        monotonic = 1.1
        gate.accept_waypoint(0.1, 0.0, 0.0, now, 'base_link', now, monotonic)
        expected = 'camera_timeout'
    elif kind == 'clock':
        gate.decide(now, monotonic)
        now -= 1
        expected = 'clock_rollback'
    else:
        gate.set_model_state('faulted')
        expected = 'model_not_running'
    assert gate.decide(now, monotonic) == (0.0, 0.0, expected)


def test_frozen_ros_clock_still_expires():
    gate = ready_gate()
    assert gate.decide(10**10, 1.1) == (0.0, 0.0, 'camera_timeout')


@pytest.mark.parametrize('stamp,frame,z', [(0, 'base_link', 0.0), (7*10**9, 'base_link', 0.0),
    (12*10**9, 'base_link', 0.0), (10**10+1, 'map', 0.0), (10**10+1, 'base_link', 1.0),
    (10**10, 'base_link', 0.0)])
def test_invalid_or_replayed_waypoints(stamp, frame, z):
    gate = ready_gate()
    assert not gate.accept_waypoint(0.1, 0.0, z, stamp, frame, 10**10, 0.1)
    assert gate.waypoint is None
    assert gate.decide(10**10, 0.1)[:2] == (0.0, 0.0)


def test_estop_latches_and_reset_requires_new_waypoint():
    gate = ready_gate()
    gate.set_estop(True)
    assert not gate.reset_estop()
    gate.set_estop(False)
    assert gate.decide(10**10, 0.1) == (0.0, 0.0, 'emergency_stop')
    assert gate.reset_estop() and gate.waypoint is None
    assert gate.decide(10**10, 0.1)[:2] == (0.0, 0.0)


@pytest.fixture
def context():
    context = Context()
    rclpy.init(context=context)
    yield context
    context.shutdown()


@pytest.fixture
def controller(context, tmp_path):
    node = ControllerNode(context=context, parameter_overrides=[Parameter('output_root', value=str(tmp_path))])
    node._timer.cancel()
    yield node
    node.destroy_node()


def test_dry_run_cannot_be_disabled(context, tmp_path):
    with pytest.raises(ValueError, match='Only dry_run'):
        ControllerNode(context=context, parameter_overrides=[
            Parameter('dry_run', value=False), Parameter('output_root', value=str(tmp_path))])


def test_motion_topic_remap_refused(context, tmp_path):
    with pytest.raises(ValueError, match='remapping refused'):
        ControllerNode(context=context, cli_args=['--ros-args', '-r', '/cmd_vel_preview:=/cmd_vel'],
                       parameter_overrides=[Parameter('output_root', value=str(tmp_path))])


def test_preview_logging_and_final_zero(controller):
    node = controller
    node._on_model(String(data='{"state":"running"}'))
    now = node.get_clock().now().nanoseconds
    node.gate.accept_camera(now, 'camera_optical_frame', now, time.monotonic())
    msg = PointStamped()
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(now, 10**9)
    msg.point.x = 0.1
    node._on_waypoint(msg)
    node._tick()
    assert node.nonzero_count == 1
    node._on_estop(Bool(data=True))
    assert node._state == 'emergency_stop'
    response = node._reset_estop(Trigger.Request(), Trigger.Response())
    assert not response.success
    node._on_estop(Bool(data=False))
    assert node._reset_estop(Trigger.Request(), Trigger.Response()).success
    node.prepare_shutdown()
    rows = list(csv.DictReader((node.run_dir / 'cmd_vel.csv').open()))
    assert rows[0]['reason'] == 'startup'
    assert rows[-1]['reason'] == 'shutdown'
    assert float(rows[-1]['linear_x']) == float(rows[-1]['angular_z']) == 0.0
    assert node._publisher.topic_name == '/cmd_vel_preview'


def test_camera_row_too_short_rejected(controller):
    controller.gate = ready_gate()
    controller._on_camera(Image(width=8, height=8, step=1, encoding='bgr8', data=[0] * 8))
    assert controller.gate.camera is None and controller.gate.waypoint is None
    assert controller.nonzero_count == 0


def test_steady_watchdog_runs_without_sim_clock(context, tmp_path):
    node = ControllerNode(context=context, parameter_overrides=[
        Parameter('output_root', value=str(tmp_path)), Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        until = time.monotonic() + 1.0
        while node.command_count < 3 and time.monotonic() < until:
            executor.spin_once(timeout_sec=0.05)
        assert node.command_count >= 3 and node.nonzero_count == 0
    finally:
        executor.shutdown()
        node.destroy_node()
