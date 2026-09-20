"""No robot, ROS graph, network, service calls or motion in these tests."""

import json
from types import MethodType, SimpleNamespace

import pytest

from nomad_navigation.operator_guard import OperatorGuard


SESSION = '32f4523c-e069-41ee-b13c-d22098653c33'
NONCE = '03c8a9b7-78ee-4d60-bd25-6f0cda78e0c1'
NOW = 10_000_000_000


def message(sequence=1, generation=1, action='heartbeat', **changes):
    result = dict(schema=1, session_id=SESSION, generation=generation, sequence=sequence,
                  connected=True, deadline_steady_ns=NOW + 2_000_000_000, action=action)
    if action == 'arm':
        result.update(arm_nonce=NONCE, mode=1, duration_sec=0)
    result.update(changes)
    return result


@pytest.mark.parametrize('value', ['', None, 3, 'not-a-uuid', SESSION.upper(),
                                  SESSION.replace('-', ''), '{' + SESSION + '}'])
def test_invalid_configured_session_never_disables_guard(value):
    with pytest.raises(ValueError):
        OperatorGuard(value)


def test_initial_heartbeat_establishes_generation_but_always_disarms():
    guard = OperatorGuard(SESSION)
    assert not guard.check(NOW).accepted
    decision = guard.accept(message(), NOW)
    assert decision.accepted and decision.disarm and decision.arm_request is None
    status = guard.snapshot(NOW)
    assert status['connected'] and status['remaining_sec'] == 2
    assert status['generation'] == 1 and status['sequence'] == 1
    assert status['last_arm_result'] is None


def test_only_later_explicit_nonce_message_can_request_arm():
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    decision = guard.accept(message(2, action='arm'), NOW)
    assert decision.accepted and not decision.disarm
    assert decision.arm_request == dict(arm_nonce=NONCE, generation=1, mode=1, duration_sec=0)
    guard.record_arm_result(decision.arm_request, True, 'armed')
    assert guard.snapshot(NOW)['last_arm_result'] == dict(
        arm_nonce=NONCE, generation=1, success=True, message='armed')


@pytest.mark.parametrize('bad', [
    {'schema': True}, {'schema': 2}, {'extra': 'unexpected'}, {'generation': True},
    {'generation': 0}, {'sequence': True}, {'sequence': 0}, {'sequence': 2**63},
    {'connected': 1}, {'deadline_steady_ns': True}, {'deadline_steady_ns': -1},
    {'deadline_steady_ns': 2**63}, {'session_id': '61a68c65-bd10-4dc3-b25c-a275a2aed103'},
    {'action': 'unknown'}, {'action': []}, {'session_id': SESSION.upper()},
])
def test_any_invalid_message_revokes_prior_authorization_generation(bad):
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    decision = guard.accept(dict(message(2), **bad), NOW)
    assert not decision.accepted and decision.disarm
    assert not guard.connected and guard.requires_new_generation
    assert not guard.accept(message(3), NOW).accepted
    assert guard.accept(message(4, generation=2), NOW).accepted


@pytest.mark.parametrize('text', ['[]', '{}', 'null', '"x"', '{', '[]' * 2048,
                                '{"schema":1,"schema":1}', '{"x": NaN}',
                                '{"x": Infinity}', '\ud800'])
def test_bad_json_never_leaves_guard_connected(text):
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    decision = guard.accept_json(text, NOW)
    assert decision.disarm and not decision.accepted and not guard.connected


@pytest.mark.parametrize('deadline', [NOW, NOW - 1, NOW + 2_050_000_001])
def test_stale_or_extended_lease_cannot_authorize_motion(deadline):
    guard = OperatorGuard(SESSION)
    assert not guard.accept(message(deadline_steady_ns=deadline), NOW).accepted


def test_transport_allowance_is_bounded():
    guard = OperatorGuard(SESSION)
    assert guard.accept(message(deadline_steady_ns=NOW + 2_050_000_000), NOW).accepted


@pytest.mark.parametrize('sequence,generation', [(1, 1), (0, 1), (2, 0)])
def test_replayed_sequence_or_old_generation_cancels_motion(sequence, generation):
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    decision = guard.accept(message(sequence, generation), NOW)
    assert decision.disarm and not decision.accepted
    assert not guard.check(NOW).accepted


def test_sequence_is_monotone_even_across_generations():
    guard = OperatorGuard(SESSION)
    guard.accept(message(10), NOW)
    assert not guard.accept(message(1, generation=2), NOW).accepted
    assert guard.accept(message(11, generation=2), NOW).accepted


def test_expiry_requires_new_generation_and_heartbeat_not_queued_arm():
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    guard.accept(message(2, action='arm'), NOW)
    expiry = NOW + 2_000_000_000
    assert guard.check(expiry).reason == 'operator_link_expired'
    assert not guard.accept(message(3, deadline_steady_ns=expiry + 2_000_000_000), expiry).accepted
    assert not guard.accept(message(4, generation=2, action='arm',
                                    deadline_steady_ns=expiry + 2_000_000_000), expiry).accepted
    # The rejected arm generation itself cannot later be revived.
    assert not guard.accept(message(5, generation=2,
                                    deadline_steady_ns=expiry + 2_000_000_000), expiry).accepted
    recovered = guard.accept(message(6, generation=3,
                                     deadline_steady_ns=expiry + 2_000_000_000), expiry)
    assert recovered.accepted and recovered.disarm and recovered.arm_request is None
    assert guard.last_arm_result is None


def test_queued_same_generation_heartbeat_checks_expiry_before_refresh():
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    later = NOW + 2_000_000_001
    decision = guard.accept(message(2, deadline_steady_ns=later + 2_000_000_000), later)
    assert not decision.accepted and decision.disarm


def test_disconnected_heartbeat_and_manual_stop_both_revoke_generation():
    for changes in ({'connected': False, 'deadline_steady_ns': 0}, {'action': 'disarm'}):
        guard = OperatorGuard(SESSION)
        guard.accept(message(), NOW)
        assert guard.accept(message(2, **changes), NOW).disarm
        assert not guard.accept(message(3), NOW).accepted
        assert guard.accept(message(4, generation=2), NOW).accepted


def test_duplicate_nonce_never_rearms_even_after_later_generation():
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    guard.accept(message(2, action='arm'), NOW)
    guard.accept(message(3, generation=2), NOW)
    duplicate = guard.accept(message(4, generation=2, action='arm'), NOW)
    assert not duplicate.accepted and duplicate.disarm
    assert duplicate.reason == 'operator_link_replayed_arm'


@pytest.mark.parametrize('changes', [dict(mode=True), dict(mode=2), dict(duration_sec=True),
    dict(duration_sec=float('inf')), dict(duration_sec=float('nan')), dict(duration_sec=1),
    dict(mode=0, duration_sec=0), dict(mode=0, duration_sec=10.001), dict(arm_nonce='RUN')])
def test_invalid_arm_cannot_pass_boundary(changes):
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    decision = guard.accept(message(2, action='arm', **changes), NOW)
    assert not decision.accepted and decision.disarm and decision.arm_request is None


def test_valid_timed_arm_is_not_silently_continuous():
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    decision = guard.accept(message(2, action='arm', mode=0, duration_sec=3), NOW)
    assert decision.arm_request['mode'] == 0
    assert decision.arm_request['duration_sec'] == 3


def test_nonce_capacity_does_not_evict_replay_protection(monkeypatch):
    from nomad_navigation import operator_guard as module
    monkeypatch.setattr(module, 'MAX_ARM_NONCES', 1)
    guard = OperatorGuard(SESSION)
    guard.accept(message(), NOW)
    guard.accept(message(2, action='arm'), NOW)
    decision = guard.accept(message(3, action='arm',
                                    arm_nonce='433d17d7-8790-476f-aa16-2338cfdc786d'), NOW)
    assert decision.reason == 'operator_link_arm_nonce_capacity'
    assert len(guard._arm_nonces) == 1


class FakeCore:
    """Only the controller's authorization side effects, never actual ROS I/O."""
    def __init__(self):
        self.armed = False
        self.motion_paused = False
        self.reason = 'disarmed'
        self.arm_calls = []

    def disarm(self, reason):
        self.armed = False
        self.motion_paused = False
        self.reason = reason

    def arm_for(self, now, steady, duration, mode):
        self.arm_calls.append((duration, mode))
        self.armed = True
        return True, 'test_armed'

    def arm(self, now, steady):
        return self.arm_for(now, steady, 3, 0)

    def tick(self, now, steady):
        return (.2, .1, 'test_tracking') if self.armed else (0, 0, self.reason)


def node_harness(monkeypatch):
    pytest.importorskip('rclpy')
    from nomad_navigation import robot_control_node as module
    clock = [NOW]
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda: clock[0])
    node = SimpleNamespace(operator_guard=OperatorGuard(SESSION), core=FakeCore(), commands=[])
    node._now = lambda: (clock[0], clock[0] / 1e9)
    node._publish = lambda v, w: node.commands.append((v, w))
    for name in ('_operator_ready', '_apply_operator_decision', '_reject_unguarded_arm', '_on_operator_link',
                 '_tick', '_arm', '_arm_for', '_disarm'):
        setattr(node, name, MethodType(getattr(module.RobotControlNode, name), node))
    return node, clock


def deliver(node, payload):
    node._on_operator_link(SimpleNamespace(data=json.dumps(payload)))


def test_node_expired_operator_cannot_network_pause_and_auto_resume(monkeypatch):
    node, clock = node_harness(monkeypatch)
    deliver(node, message())
    deliver(node, message(2, action='arm'))
    assert node.core.armed and len(node.core.arm_calls) == 1
    node.core.motion_paused = True  # Previously authorized network auto-resume.
    clock[0] += 2_000_000_000
    node._tick()
    assert not node.core.armed and not node.core.motion_paused
    assert node.commands[-1] == (0.0, 0.0)
    deliver(node, message(3, generation=2, deadline_steady_ns=clock[0] + 2_000_000_000))
    node._tick()
    assert not node.core.armed and len(node.core.arm_calls) == 1
    assert node.commands[-1] == (0.0, 0.0)
    deliver(node, message(4, generation=2, action='arm',
                         arm_nonce='de735034-f812-4d94-95e4-82166b33f5ce',
                         deadline_steady_ns=clock[0] + 2_000_000_000))
    assert node.core.armed and len(node.core.arm_calls) == 2


@pytest.mark.parametrize('method', ['_arm', '_arm_for'])
def test_node_legacy_arm_services_cannot_bypass_guard(monkeypatch, method):
    node, clock = node_harness(monkeypatch)
    deliver(node, message())
    node.core.armed = True
    result = getattr(node, method)(SimpleNamespace(duration_sec=0, mode=1), SimpleNamespace())
    assert not result.success and not node.core.armed
    assert not node.core.arm_calls and node.commands[-1] == (0.0, 0.0)


def test_node_direct_disarm_invalidates_queued_arm_generation(monkeypatch):
    node, clock = node_harness(monkeypatch)
    deliver(node, message())
    result = node._disarm(None, SimpleNamespace())
    assert result.success
    deliver(node, message(2, action='arm'))
    assert not node.core.armed and not node.core.arm_calls


def test_node_changed_generation_cancels_previous_run(monkeypatch):
    node, clock = node_harness(monkeypatch)
    deliver(node, message())
    deliver(node, message(2, action='arm'))
    deliver(node, message(3, generation=2))
    assert not node.core.armed and node.commands[-1] == (0.0, 0.0)
    assert node.operator_guard.last_arm_result is None


@pytest.mark.parametrize('method', ['_arm', '_arm_for'])
def test_node_legacy_unguarded_paths_remain_compatible(monkeypatch, method):
    node, clock = node_harness(monkeypatch)
    node.operator_guard = None
    result = getattr(node, method)(SimpleNamespace(duration_sec=0, mode=1), SimpleNamespace())
    assert result.success and node.core.armed and len(node.core.arm_calls) == 1
    node._tick()
    assert node.commands[-1] == (.2, .1)


def test_node_publisher_has_final_zero_output_guard(monkeypatch):
    node, clock = node_harness(monkeypatch)
    from nomad_navigation import robot_control_node as module
    deliver(node, message())
    node.core.armed = True
    node.core.limits = SimpleNamespace()
    node.core.snapshot = lambda steady: dict(armed=node.core.armed, reason=node.core.reason)
    desired, preview, status = [], [], []
    node._desired_pub = SimpleNamespace(publish=desired.append)
    node._preview_pub = SimpleNamespace(publish=preview.append)
    node._status_pub = SimpleNamespace(publish=status.append)
    clock[0] += 2_000_000_000
    module.RobotControlNode._publish(node, .2, .1)
    assert desired[0].twist.linear.x == desired[0].twist.angular.z == 0
    assert preview[0].linear.x == preview[0].angular.z == 0
    report = json.loads(status[0].data)
    assert not report['armed'] and not report['operator_link']['connected']
    assert report['reason'] == 'operator_link_expired'


def test_invalid_env_rejects_node_construction_without_fallback(monkeypatch):
    pytest.importorskip('rclpy')
    from nomad_navigation import robot_control_node as module
    # Only exercise the constructor's initial env parsing. No ROS context,
    # publishers, subscriptions or node graph is ever created by this test.
    monkeypatch.setattr(module.Node, '__init__', lambda self, *args, **kwargs: None)
    monkeypatch.setattr(module.Node, 'destroy_node', lambda self: True)
    monkeypatch.setenv('NOMAD_OPERATOR_GUARD_SESSION', '')
    with pytest.raises(ValueError, match='canonical UUID'):
        module.RobotControlNode()


def test_rejected_core_arm_is_recorded_but_nonce_is_never_reused(monkeypatch):
    node, clock = node_harness(monkeypatch)
    deliver(node, message())
    node.core.arm_for = lambda *args, **kwargs: (False, 'waiting_fresh_prediction')
    deliver(node, message(2, action='arm'))
    assert node.operator_guard.last_arm_result == dict(
        arm_nonce=NONCE, generation=1, success=False, message='waiting_fresh_prediction')
    deliver(node, message(3, action='arm'))
    assert not node.core.armed
    assert node.operator_guard.reason == 'operator_link_replayed_arm'
