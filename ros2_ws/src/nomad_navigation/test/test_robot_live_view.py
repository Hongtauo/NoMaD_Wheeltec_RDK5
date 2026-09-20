"""Live RViz config and node behavior, without camera, robot or motion outputs."""

import copy
import importlib.util
import json
from pathlib import Path
import time

from geometry_msgs.msg import PointStamped, Pose, PoseArray
import pytest
import rclpy
from rclpy.context import Context
from std_msgs.msg import String
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE.parents[1] / 'scripts' / 'robot_live_view.py'
SPEC = importlib.util.spec_from_file_location('robot_live_view_ros', SCRIPT)
view = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(view)
LAYOUT = {'samples': 1, 'steps': 3, 'waypoint_index': 2, 'frame_id': 'base_link', 'scale_applied': .05}


@pytest.fixture
def live_node():
    context = Context()
    rclpy.init(args=[], context=context)
    node = view.make_visualizer(context)
    node.timer.cancel()
    yield node
    node.destroy_node()
    context.shutdown()


def test_rviz_live_topics_qos_and_no_fake_global_frame():
    config = yaml.safe_load((PACKAGE / 'rviz' / 'nomad_robot_live.rviz').read_text())['Visualization Manager']
    displays = {item['Class']: item for item in config['Displays']}
    camera = displays['rviz_default_plugins/Image']
    assert camera['Topic']['Value'] == '/camera/color/image_raw/compressed'
    assert camera['Topic']['Reliability Policy'] == 'Best Effort'
    assert 'Transport Hint' not in camera  # Humble infers transport from topic suffix.
    assert displays['rviz_default_plugins/MarkerArray']['Topic']['Value'] == view.MARKER_TOPIC
    assert config['Global Options']['Fixed Frame'] == 'base_link'
    assert 'rviz_default_plugins/Odometry' not in displays
    assert 'rviz_default_plugins/Path' not in displays


def test_factory_shares_context_and_only_publishes_markers(live_node):
    assert live_node.get_namespace() == '/nomad/robot_preview'
    assert live_node.p['controller_status_topic'] == '/nomad/robot_control/status'
    assert live_node.p['marker_topic'] == view.MARKER_TOPIC
    publishers = list(live_node.publishers)
    assert {publisher.topic_name for publisher in publishers} <= {
        view.MARKER_TOPIC, '/rosout', '/parameter_events'}
    assert '/cmd_vel' not in {publisher.topic_name for publisher in publishers}


def test_live_prediction_pairing_expiration_and_label(live_node):
    live_node._model(String(data=json.dumps({'state': 'running', 'trajectory_layout': LAYOUT})))
    poses, waypoint = PoseArray(), PointStamped()
    poses.header.frame_id = 'base_link'
    poses.header.stamp = live_node.get_clock().now().to_msg()
    for x in (.05, .10, .15):
        pose = Pose()
        pose.position.x = x
        pose.orientation.w = 1.0
        poses.poses.append(pose)
    waypoint.header = copy.deepcopy(poses.header)
    waypoint.point = copy.deepcopy(poses.poses[2].position)
    live_node._poses(poses)
    live_node._waypoint(waypoint)
    markers = live_node.render().markers
    assert any(marker.ns == 'candidate_paths' for marker in markers)
    status = next(marker for marker in markers if marker.ns == 'status')
    assert 'LIVE | NO ODOMETRY' in status.text
    assert 'not measured motion' in status.text
    paths, point, stamp, _ = live_node.pair
    live_node.predictions.clear()
    live_node.waypoints.clear()
    live_node.pair = paths, point, stamp, time.monotonic() - 2
    markers = live_node.render().markers
    assert not any(marker.ns == 'candidate_paths' for marker in markers)
    assert 'NO FRESH PREDICTION' in next(marker for marker in markers if marker.ns == 'status').text


def test_live_factory_rejects_missing_context():
    with pytest.raises(ValueError):
        view.make_visualizer(None)


def reported_control(node, mode='timed', remaining=5.0):
    return {'time_ns': node.get_clock().now().nanoseconds, 'state': 'tracking' if mode else 'operator_disarmed',
            'armed': mode is not None, 'active_mode': mode,
            'requested_duration_sec': 5.0 if mode == 'timed' else 0.0 if mode else None,
            'lease_remaining_sec': remaining if mode == 'timed' else None if mode else 0.,
            'run_until_disarmed': mode == 'continuous'}


def rendered_status(node):
    return next(marker.text for marker in node.render().markers if marker.ns == 'status')


@pytest.mark.parametrize('mode,label', [('timed', 'run: TIMED'), ('continuous', 'run: CONTINUOUS'),
                                        (None, 'run: DISARMED')])
def test_live_control_callback_preserves_parent_state_and_shows_mode(live_node, mode, label):
    status = reported_control(live_node, mode)
    live_node._controller(String(data=json.dumps(status)))
    assert live_node.controller_state == status['state']
    text = rendered_status(live_node)
    assert label in text
    assert 'not measured motion' in text


def test_live_control_expired_receipt_cannot_keep_old_armed_label(live_node):
    live_node._controller(String(data=json.dumps(reported_control(live_node, 'continuous'))))
    assert 'run: CONTINUOUS' in rendered_status(live_node)
    live_node._control_received = time.monotonic() - 1.0
    text = rendered_status(live_node)
    assert 'run: UNKNOWN [control status stale]' in text
    assert 'control: unknown' in text
    assert 'CONTINUOUS' not in text
    assert 'DISARMED' not in text


def test_live_control_expired_source_is_unknown_even_after_recent_receipt(live_node):
    status = reported_control(live_node, 'continuous')
    status['time_ns'] -= 2_000_000_000
    live_node._controller(String(data=json.dumps(status)))
    assert 'run: UNKNOWN' in rendered_status(live_node)


def test_live_control_replayed_status_cannot_refresh_arming_display(live_node):
    message = String(data=json.dumps(reported_control(live_node, 'continuous')))
    live_node._controller(message)
    assert 'run: CONTINUOUS' in rendered_status(live_node)
    live_node._controller(message)
    assert 'run: UNKNOWN' in rendered_status(live_node)


@pytest.mark.parametrize('clock', ['ros', 'steady'])
def test_live_control_clock_rollback_clears_armed_mode(live_node, clock):
    live_node._controller(String(data=json.dumps(reported_control(live_node, 'continuous'))))
    assert 'run: CONTINUOUS' in rendered_status(live_node)
    if clock == 'ros':
        live_node.last_ros = live_node.get_clock().now().nanoseconds + 1_000_000_000
    else:
        live_node._live_last_monotonic = time.monotonic() + 1.0
    text = rendered_status(live_node)
    assert 'run: UNKNOWN' in text
    assert 'CONTINUOUS' not in text
    assert 'control: unknown' in text


@pytest.mark.parametrize('payload', ['invalid json', 'null', '[]', '{"armed":true}',
                                    '{"state":"tracking","armed":true,"active_mode":"continuous"}'])
def test_live_invalid_control_payload_replaces_old_armed_display(live_node, payload):
    live_node._controller(String(data=json.dumps(reported_control(live_node, 'continuous'))))
    live_node._controller(String(data=payload))
    assert 'run: UNKNOWN' in rendered_status(live_node)
