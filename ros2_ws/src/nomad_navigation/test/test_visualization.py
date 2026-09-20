"""Timestamp pairing, layout, nominal scale and stale-marker tests."""
import copy
import json
import math
import time

from geometry_msgs.msg import PointStamped, Pose, PoseArray, Twist
import pytest
import rclpy
from rclpy.context import Context
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from nomad_navigation.visualization_node import VisualizationNode, decode_layout, paired_paths


LAYOUT = {'samples': 2, 'steps': 3, 'waypoint_index': 2, 'frame_id': 'base_link', 'scale_applied': 0.05}


def prediction(stamp):
    poses, waypoint = PoseArray(), PointStamped()
    poses.header.frame_id = 'base_link'
    poses.header.stamp.sec, poses.header.stamp.nanosec = divmod(stamp, 10**9)
    waypoint.header = copy.deepcopy(poses.header)
    for i in range(6):
        pose = Pose()
        pose.position.x = 0.05 * (i+1)
        pose.orientation.w = 1.0
        poses.poses.append(pose)
    waypoint.point = copy.deepcopy(poses.poses[2].position)
    return poses, waypoint


@pytest.mark.parametrize('field,value', [('samples', 0), ('samples', True), ('steps', 0),
    ('waypoint_index', 3), ('scale_applied', math.nan), ('frame_id', '')])
def test_invalid_layout(field, value):
    with pytest.raises(ValueError):
        decode_layout({**LAYOUT, field: value})


def test_no_double_scaling():
    poses, waypoint = prediction(10**10)
    paths = paired_paths(poses, waypoint, LAYOUT)
    assert len(paths) == 2 and len(paths[0]) == 3
    assert paths[0][2].x == pytest.approx(0.15)


@pytest.mark.parametrize('kind', ['stamp', 'frame', 'size', 'nan', 'waypoint'])
def test_mismatched_predictions_rejected(kind):
    poses, wp = prediction(10**10)
    if kind == 'stamp':
        wp.header.stamp.nanosec += 1
    elif kind == 'frame':
        wp.header.frame_id = 'map'
    elif kind == 'size':
        poses.poses.pop()
    elif kind == 'nan':
        poses.poses[0].position.x = math.nan
    else:
        wp.point.y = 0.5
    with pytest.raises(ValueError):
        paired_paths(poses, wp, LAYOUT)


@pytest.fixture
def node():
    context = Context()
    rclpy.init(context=context)
    node = VisualizationNode(context=context)
    node.timer.cancel()
    node._model(String(data=json.dumps({'state': 'running', 'trajectory_layout': LAYOUT})))
    yield node
    node.destroy_node()
    context.shutdown()


def test_pair_in_any_arrival_order_and_no_message_mutation(node):
    poses, wp = prediction(node.get_clock().now().nanoseconds)
    node._waypoint(wp)
    assert not any(m.ns == 'candidate_paths' for m in node.render().markers)
    node._poses(poses)
    for _ in range(2):
        markers = node.render().markers
        assert len([m for m in markers if m.ns == 'candidate_paths']) == 2
        assert wp.point.z == 0.0
        assert markers[0].action == Marker.DELETEALL


def test_stale_predictions_and_velocity_disappear(node):
    poses, wp = prediction(node.get_clock().now().nanoseconds)
    node._poses(poses)
    node._waypoint(wp)
    node.render()
    paths, waypoint, stamp, _ = node.pair
    node.predictions.clear()
    node.waypoints.clear()
    node.pair = (paths, waypoint, stamp, time.monotonic()-2)
    node.velocity = (0.2, 0.4, time.monotonic()-2)
    markers = node.render().markers
    assert not any(m.ns in ('candidate_paths', 'selected_waypoint', 'velocity') for m in markers)


def test_model_fault_removes_predictions(node):
    poses, wp = prediction(node.get_clock().now().nanoseconds)
    node._poses(poses)
    node._waypoint(wp)
    node.render()
    node._model(String(data=json.dumps({'state': 'faulted', 'trajectory_layout': LAYOUT})))
    assert node.pair is None


def test_velocity_arrow_and_arc(node):
    msg = Twist()
    msg.linear.x, msg.angular.z = 0.2, -0.4
    node._velocity(msg)
    markers = node.render().markers
    arrow = next(m for m in markers if m.ns == 'velocity')
    assert arrow.points[-1].x == pytest.approx(0.4)
    arc = next(m for m in markers if m.ns == 'angular_velocity')
    assert arc.points[-1].y < 0


def test_clock_rollback_clears_cache(node):
    poses, wp = prediction(node.get_clock().now().nanoseconds)
    node._poses(poses)
    node._waypoint(wp)
    node.last_ros = node.get_clock().now().nanoseconds+10**9
    node.render()
    assert not node.predictions and not node.waypoints and node.pair is None
