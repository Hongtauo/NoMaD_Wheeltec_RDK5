"""Unit checks for startup parameters, heartbeat and safe node behavior."""

import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from nomad_navigation.smoke_node import SmokeNode, validate_heartbeat_period


@pytest.fixture
def ros_context():
    context = Context()
    rclpy.init(context=context)
    yield context
    context.shutdown()


@pytest.mark.parametrize('period', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_period(period):
    with pytest.raises(ValueError, match='greater than zero'):
        validate_heartbeat_period(period)


def test_valid_period():
    assert validate_heartbeat_period(0.25) == 0.25


def test_node_defaults_and_heartbeat(ros_context):
    node = SmokeNode(context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    try:
        assert node.get_name() == 'nomad_smoke'
        assert node.get_parameter('heartbeat_period_sec').value == 2.0
        deadline = time.monotonic() + 4.0
        while node._heartbeat_count == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert node._heartbeat_count >= 1
        topics = node.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace())
        assert all(name in ('/rosout', '/parameter_events') for name, _ in topics)
    finally:
        executor.shutdown()
        node.destroy_node()


def test_startup_overrides_and_readonly_parameters(ros_context):
    node = SmokeNode(
        context=ros_context,
        parameter_overrides=[
            Parameter('heartbeat_period_sec', value=0.25),
            Parameter('greeting', value='test startup'),
        ],
    )
    try:
        assert node.get_parameter('heartbeat_period_sec').value == 0.25
        assert node.get_parameter('greeting').value == 'test startup'
        result = node.set_parameters([Parameter('heartbeat_period_sec', value=1.0)])
        assert not result[0].successful
    finally:
        node.destroy_node()
