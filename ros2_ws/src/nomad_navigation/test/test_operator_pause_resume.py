"""Bounded opt-in operator recovery; pure state machines, no hardware or SSH."""

import json
from types import MethodType, SimpleNamespace

import pytest

from nomad_navigation.operator_guard import (
    GUARD_ENV, OPERATOR_PAUSE_RESUME_ENV, OperatorGuard, operator_pause_resume_from_env,
)
from nomad_navigation.robot_control_core import RobotControlCore, RobotControlLimits


SESSION = '32f4523c-e069-41ee-b13c-d22098653c33'
NONCE = '03c8a9b7-78ee-4d60-bd25-6f0cda78e0c1'
NOW = 10_000_000_000


def payload(sequence=1, generation=1, action='heartbeat', at=10., **changes):
    message = dict(schema=1, session_id=SESSION, sequence=sequence, generation=generation,
                   action=action, connected=action != 'pause',
                   deadline_steady_ns=round((at + 2.) * 1e9))
    if action == 'arm':
        message.update(arm_nonce=NONCE, mode=1, duration_sec=0.)
    message.update(changes)
    return message


def make_core(speed=.05, enabled=True):
    return RobotControlCore(RobotControlLimits(max_linear_mps=speed,
        max_measured_linear_mps={.05: .12, .1: .2, .2: .3}[speed]),
        network_pause_resume=True, operator_pause_resume=enabled)


def fresh(core, at, *, waypoint=True, stamp=None, camera=True, odom=True, speed=0., turn=0.):
    now = round(at * 1e9)
    core.set_model_state('running')
    if camera:
        assert core.accept_camera(now, 'camera_color_optical_frame', now, at)
    if waypoint:
        assert core.accept_waypoint(.5, .03, 0., now if stamp is None else stamp,
                                    'base_link', now, at)
    if odom:
        assert core.accept_odom((speed, 0., 0.), (0., 0., turn), now,
                                'odom', 'base_footprint', now, at)
    return now


def armed_core(speed=.05, mode=1):
    core = make_core(speed)
    core.operator_link_recovered(NOW, 10.)
    assert core.arm_for(fresh(core, 10.), 10., 0. if mode == 1 else 3., mode)[0]
    assert core.tick(fresh(core, 10.05), 10.05)[0] > 0.
    return core


def pause(core, at=10.1):
    core.operator_link_pause('operator_link_paused', round(at * 1e9), at)


def recover(core, at=10.15):
    core.operator_link_recovered(round(at * 1e9), at)


@pytest.mark.parametrize('env', [
    {OPERATOR_PAUSE_RESUME_ENV: '1'},
    {OPERATOR_PAUSE_RESUME_ENV: '0', GUARD_ENV: SESSION},
    {OPERATOR_PAUSE_RESUME_ENV: '', GUARD_ENV: SESSION},
    {OPERATOR_PAUSE_RESUME_ENV: 'true', GUARD_ENV: SESSION},
    {OPERATOR_PAUSE_RESUME_ENV: '1', GUARD_ENV: ''},
])
def test_invalid_opt_in_cannot_silently_fall_back(env):
    with pytest.raises(ValueError):
        operator_pause_resume_from_env(env)


def test_opt_in_requires_guard_network_mode_and_explicit_boolean():
    assert not operator_pause_resume_from_env({})
    assert operator_pause_resume_from_env({OPERATOR_PAUSE_RESUME_ENV: '1', GUARD_ENV: SESSION})
    for value in ('1', 1, None):
        with pytest.raises(ValueError):
            OperatorGuard(SESSION, pause_resume=value)
        with pytest.raises(ValueError):
            RobotControlCore(network_pause_resume=True, operator_pause_resume=value)
    with pytest.raises(ValueError):
        RobotControlCore(operator_pause_resume=True)


@pytest.mark.parametrize('speed', [.05, .1, .2])
def test_three_profiles_pause_then_three_new_predictions_resume_from_zero(speed):
    core = armed_core(speed)
    pause(core)
    assert core.armed and core.operator_paused and core.motion_paused
    assert core.last_linear == 0.
    for at in (10.1, 10.15, 10.2):
        assert core.tick(fresh(core, at), at)[:2] == (0., 0.)
    assert core.resume_candidates == 0
    recover(core, 10.21)
    deadline = core.operator_resume_deadline
    assert core.resume_after_stamp == 10_210_000_000
    for index, at in enumerate((10.25, 10.3, 10.35), 1):
        result = core.tick(fresh(core, at), at)
        assert core.resume_candidates == index
        if index < 3:
            assert result[:2] == (0., 0.) and core.operator_paused
            assert core.operator_resume_deadline == deadline
        else:
            assert 0. < result[0] <= .025 * .05 + 1e-12
            assert abs(result[1]) <= result[0] + 1e-12
            assert core.armed and not core.operator_paused
            assert core.operator_resume_deadline is None


def test_queued_pre_reconnect_predictions_cannot_count():
    core = armed_core()
    pause(core)
    assert core.tick(fresh(core, 10.1), 10.1)[0] == 0.
    recover(core, 10.15)
    for at, stamp in ((10.16, 10_110_000_000), (10.17, 10_120_000_000),
                      (10.18, 10_150_000_000)):
        assert core.tick(fresh(core, at, stamp=stamp), at)[0] == 0.
        assert core.resume_candidates == 0 and core.armed
    for at in (10.2, 10.25, 10.3):
        result = core.tick(fresh(core, at), at)
    assert result[0] > 0.


def test_repeated_prediction_stamp_is_hard_not_an_extra_candidate():
    core = armed_core()
    pause(core)
    recover(core)
    core.tick(fresh(core, 10.2), 10.2)
    assert core.resume_candidates == 1
    assert not core.accept_waypoint(.5, .03, 0., 10_200_000_000,
                                     'base_link', 10_210_000_000, 10.21)
    assert not core.armed and not core.operator_paused
    recover(core, 10.22)
    assert core.tick(fresh(core, 10.25), 10.25)[0] == 0.


def test_stationary_and_all_freshness_gates_remain_required():
    core = armed_core()
    pause(core)
    recover(core)
    for at in (10.2, 10.25, 10.3):
        assert core.tick(fresh(core, at, speed=.02), at)[0] == 0.
    assert core.resume_candidates == 3 and core.operator_paused
    assert core.tick(fresh(core, 10.35), 10.35)[0] > 0.


@pytest.mark.parametrize('flaps', [False, True])
def test_thirty_second_window_expires_and_neither_flaps_nor_packets_extend(flaps):
    core = armed_core()
    pause(core)
    deadline = core.operator_resume_deadline
    assert deadline == 40.1
    for step in range(602):
        at = 10.1 + step * .05
        if flaps and step % 10 == 3:
            recover(core, at)
        elif step % 10 == 4 or not flaps:
            pause(core, at)
        result = core.tick(fresh(core, at, waypoint=False), at)
        assert result[:2] == (0., 0.)
        if at < deadline:
            assert core.armed and core.operator_resume_deadline == deadline
        else:
            assert not core.armed and not core.operator_paused
            assert core.operator_resume_deadline is None
    recover(core, 40.21)
    for at in (40.25, 40.3, 40.35):
        assert core.tick(fresh(core, at), at)[0] == 0.
    assert not core.armed


def test_recovery_window_does_not_reset_while_waiting_for_three_after_reconnect():
    core = armed_core()
    pause(core)
    deadline = core.operator_resume_deadline
    for index, at in enumerate((10.15, 10.2, 10.25, 10.3, 10.35, 10.4)):
        if index % 2 == 0:
            recover(core, at)
        else:
            pause(core, at)
        assert core.tick(fresh(core, at), at)[0] == 0.
        assert core.operator_resume_deadline == deadline
    recover(core, 10.45)
    for at in (10.5, 10.55, 10.6):
        result = core.tick(fresh(core, at), at)
    assert result[0] > 0.
    pause(core, 10.65)
    assert core.operator_resume_deadline == 40.65  # A genuinely resumed run's next pause.


@pytest.mark.parametrize('fault', ['manual', 'estop', 'model', 'camera', 'odom', 'clock', 'timer', 'shutdown'])
def test_hard_fault_while_offline_erases_intent_and_cannot_revive(fault):
    core = armed_core()
    pause(core)
    core.tick(fresh(core, 10.1), 10.1)
    if fault in ('manual', 'shutdown'):
        core.disarm('operator_disarmed' if fault == 'manual' else 'shutdown')
    elif fault == 'estop':
        core.set_estop(True)
        core.set_estop(False)
    elif fault == 'model':
        core.set_model_state('faulted')
    elif fault == 'camera':
        core.reject_camera()
    elif fault == 'odom':
        core.accept_odom((.121, 0., 0.), (0., 0., 0.), 10_150_000_000,
                         'odom', 'base_footprint', 10_150_000_000, 10.15)
    elif fault == 'clock':
        core.tick(10_000_000_000, 10.15)
    elif fault == 'timer':
        core.tick(fresh(core, 10.4), 10.4)
    assert not core.armed and not core.operator_paused
    recover(core, 10.5)
    if fault == 'clock':
        assert core.tick(10_550_000_000, 10.55)[0] == 0.
        assert core.gate.clock_fault
    else:
        for at in (10.55, 10.6, 10.65):
            assert core.tick(fresh(core, at), at)[0] == 0.
    assert not core.armed


def test_odom_timeout_is_not_hidden_behind_disconnected_guard_or_model_wait():
    core = armed_core()
    pause(core)
    for step in range(1, 15):
        at = 10.05 + step * .05
        fresh(core, at, waypoint=False, odom=False)
        core.set_model_state('network_wait')
        assert core.tick(round(at * 1e9), at)[0] == 0.
    assert not core.armed and not core.operator_paused


def test_timed_mode_loss_disarms_and_startup_heartbeats_never_create_intent():
    core = armed_core(mode=0)
    pause(core)
    assert not core.armed and not core.operator_paused
    for core in (core, make_core()):
        recover(core, 10.15)
        for at in (10.2, 10.25, 10.3):
            assert core.tick(fresh(core, at), at)[0] == 0.
        assert not core.armed


def test_default_pause_request_still_disarms():
    guard = OperatorGuard(SESSION)
    guard.accept(payload(), NOW)
    result = guard.accept(payload(2, action='pause'), NOW)
    assert result.disarm and not result.pause
    core = make_core(enabled=False)
    assert core.arm_for(fresh(core, 10.), 10., 0., 1)[0]
    pause(core)
    assert not core.armed


@pytest.mark.parametrize('trigger', ['expiry', 'pause'])
def test_opted_in_guard_reports_soft_pause_and_requires_new_generation(trigger):
    guard = OperatorGuard(SESSION, pause_resume=True)
    first = guard.accept(payload(), NOW)
    assert first.pause and first.generation_changed and not first.disarm
    now = NOW + 2_000_000_000 if trigger == 'expiry' else NOW
    result = (guard.check(now) if trigger == 'expiry'
              else guard.accept(payload(2, action='pause'), now))
    assert result.pause and not result.disarm and not result.accepted
    assert guard.check(now).pause
    assert guard.snapshot(now)['pause_resume']
    result = guard.accept(payload(3, generation=2, at=now / 1e9), now)
    assert result.accepted and result.pause and result.generation_changed
    assert not result.disarm and result.arm_request is None
    assert not guard.accept(payload(4, generation=2, at=now / 1e9), now).pause


def test_exact_original_expired_heartbeat_is_only_drained_never_renewed():
    guard = OperatorGuard(SESSION, pause_resume=True)
    guard.accept(payload(), NOW)
    core = armed_core()
    expired = NOW + 2_000_000_000
    decision = guard.accept(payload(2), expired)
    assert decision.pause and not decision.accepted and not decision.disarm
    assert not guard.connected and guard.requires_new_generation
    assert guard.deadline_steady_ns == 0 and guard.sequence == 2
    # Keep the core ticking with fresh input before the real lease expiration.
    for step in range(2, 41):
        at = 10. + step * .05
        core.tick(fresh(core, at), at)
    core.operator_link_pause(decision.reason, expired, 12.)
    deadline = core.operator_resume_deadline
    for step in range(1, 21):
        at = 12. + step * .05
        decision = guard.accept(payload(step + 2), round(at * 1e9))
        assert decision.pause and not decision.accepted
        core.operator_link_pause(decision.reason, round(at * 1e9), at)
        assert core.tick(fresh(core, at), at)[0] == 0.
        assert core.operator_resume_deadline == deadline
    assert guard.accept(payload(24, generation=2, at=13.), 13_000_000_000).generation_changed


@pytest.mark.parametrize('changes', [
    {'deadline_steady_ns': NOW + 2_000_000_001},
    {'deadline_steady_ns': NOW + 1_999_999_999},
    {'deadline_steady_ns': NOW + 3_000_000_000},
    {'sequence': 1}, {'generation': 0}, {'generation': 2},
    {'action': 'arm', 'arm_nonce': NONCE, 'mode': 1, 'duration_sec': 0.},
    {'connected': False}, {'extra': True},
])
def test_expired_heartbeat_exception_does_not_accept_changed_or_replayed_messages(changes):
    guard = OperatorGuard(SESSION, pause_resume=True)
    guard.accept(payload(), NOW)
    guard.check(NOW + 2_000_000_000)
    message = payload(2)
    message.update(changes)
    result = guard.accept(message, NOW + 2_000_000_000)
    assert result.disarm and not result.pause and not result.accepted


def test_hard_revoke_cannot_be_softened_by_original_expired_heartbeat():
    guard = OperatorGuard(SESSION, pause_resume=True)
    guard.accept(payload(), NOW)
    guard.revoke('operator_disarmed')
    result = guard.accept(payload(2), NOW + 2_000_000_000)
    assert result.disarm and not result.pause


@pytest.mark.parametrize('changes', [
    {'action': 'disarm'}, {'connected': False}, {'sequence': 1}, {'generation': 0},
    {'deadline_steady_ns': NOW}, {'action': 'pause', 'connected': True},
    {'action': 'pause', 'connected': False, 'deadline_steady_ns': NOW + 3_000_000_000},
    {'action': 'pause', 'connected': False, 'extra': 1}, {'session_id': 'wrong'},
])
def test_malformed_replay_and_manual_disarm_are_hard_even_with_opt_in(changes):
    guard = OperatorGuard(SESSION, pause_resume=True)
    guard.accept(payload(), NOW)
    message = payload(2)
    message.update(changes)
    decision = guard.accept(message, NOW)
    assert decision.disarm and not decision.pause
    assert guard.check(NOW).disarm and not guard.check(NOW).pause


def test_node_callbacks_apply_generation_then_post_reconnect_gate(monkeypatch):
    pytest.importorskip('rclpy')
    from nomad_navigation import robot_control_node as module
    clock = [NOW]
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda: clock[0])
    node = SimpleNamespace(operator_guard=OperatorGuard(SESSION, pause_resume=True),
                           core=make_core(), commands=[])
    node._now = lambda: (clock[0], clock[0] / 1e9)
    node._publish = lambda v, w: node.commands.append((v, w))
    for name in ('_apply_operator_decision', '_operator_ready', '_on_operator_link', '_tick'):
        setattr(node, name, MethodType(getattr(module.RobotControlNode, name), node))
    def send(message):
        node._on_operator_link(SimpleNamespace(data=json.dumps(message)))
    send(payload())
    fresh(node.core, 10.)
    send(payload(2, action='arm'))
    assert node.core.armed
    # No explicit pause packet: the first NEW generation must still insert pause.
    clock[0] += 50_000_000
    fresh(node.core, 10.05)
    node._tick()
    assert node.commands[-1][0] > 0.
    send(payload(3, generation=2, at=10.05))
    assert node.core.operator_paused and node.commands[-1] == (0., 0.)
    for index in range(1, 4):
        clock[0] = NOW + 50_000_000 + index * 50_000_000
        fresh(node.core, clock[0] / 1e9)
        node._tick()
        assert (node.commands[-1][0] > 0.) == (index == 3)
    # Expiry while fresh camera/model/odom keep arriving still forces zero.
    for step in range(1, 41):
        clock[0] = NOW + 200_000_000 + step * 50_000_000
        fresh(node.core, clock[0] / 1e9)
        node._tick()
    assert node.core.armed and node.core.operator_paused
    assert node.commands[-1] == (0., 0.)
    # Independent control timer remains active behind the paused operator gate.
    clock[0] += 300_000_000
    fresh(node.core, clock[0] / 1e9)
    node._tick()
    assert not node.core.armed and not node.core.operator_paused
    assert node.commands[-1] == (0., 0.)
