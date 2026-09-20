"""No hardware: transient pause retains intent, never stale motion authority."""
import pytest
from nomad_navigation.robot_control_core import RobotControlCore, RobotControlLimits
from nomad_navigation.campus_client_core import CampusClientCore, TRANSIENT_PAUSE_REASONS


@pytest.fixture(params=[.05, .1, .2])
def cap(request):
    return request.param


def sensors(core, t, *, waypoint=False, speed=0.):
    ns = round(t * 1e9)
    assert core.accept_camera(ns, 'camera_color_optical_frame', ns, t)
    assert core.accept_odom((speed, 0., 0.), (0., 0., 0.), ns, 'odom', 'base_footprint', ns, t)
    if waypoint:
        assert core.accept_waypoint(.2, .02, 0., ns, 'base_link', ns, t)
    return ns


def armed(*, opt=True, mode=1, cap=.05):
    measured = {.05: .12, .1: .2, .2: .3}[cap]
    core = RobotControlCore(RobotControlLimits(max_linear_mps=cap, max_measured_linear_mps=measured),
                            network_pause_resume=opt)
    core.set_model_state('running')
    now = sensors(core, 10., waypoint=True)
    assert core.arm_for(now, 10., 0. if mode == 1 else 3., mode)[0]
    assert core.tick(sensors(core, 10.05), 10.05)[0] > 0.
    return core


def recover(core, start=10.10, speed=0.):
    commands = []
    core.set_model_state('running')
    for step in range(15):
        t = round(start + step * .05, 8)
        now = sensors(core, t, waypoint=step in (0, 6, 12), speed=speed)
        commands.append(core.tick(now, t))
    return commands


def test_opt_in_pauses_then_three_new_predictions_resume_with_acceleration(cap):
    core = armed(cap=cap)
    core.set_model_state('network_wait')
    assert core.armed and core.motion_paused and core.last_linear == 0.
    commands = recover(core)
    assert all(command[:2] == (0., 0.) for command in commands[:12])
    assert 0 < commands[12][0] <= .025 * .05 + 1e-10
    assert core.armed and not core.motion_paused
    assert core.snapshot(10.9)['network_pause_resume'] is True


@pytest.mark.parametrize('opt,mode', [(False, 1), (True, 0)])
def test_default_and_timed_mode_do_not_resume(opt, mode, cap):
    core = armed(opt=opt, mode=mode, cap=cap)
    core.set_model_state('network_wait')
    assert not core.armed
    assert all(row[:2] == (0., 0.) for row in recover(core))


@pytest.mark.parametrize('stop', ['manual', 'estop', 'clock', 'invalid_waypoint', 'model_fatal'])
def test_hard_stops_cancel_paused_intent(stop, cap):
    core = armed(cap=cap)
    core.set_model_state('network_wait')
    if stop == 'manual':
        core.disarm('operator_disarmed')
    elif stop == 'estop':
        core.set_estop(True)
        core.set_estop(False)
    elif stop == 'clock':
        core.tick(9_000_000_000, 10.06)
    elif stop == 'invalid_waypoint':
        core.accept_waypoint(float('nan'), 0., 0., 10_060_000_000, 'base_link', 10_060_000_000, 10.06)
    else:
        core.set_model_state('identity_or_protocol_error')
    assert not core.armed
    if stop != 'clock':
        assert all(row[:2] == (0., 0.) for row in recover(core))


def test_moving_robot_cannot_resume_until_stationary(cap):
    core = armed(cap=cap)
    core.set_model_state('network_wait')
    assert all(row[:2] == (0., 0.) for row in recover(core, speed=.02))
    assert core.motion_paused
    assert core.tick(sensors(core, 10.85, waypoint=True), 10.85)[0] > 0.


def test_missing_odom_is_hard_even_while_network_waits(cap):
    core = armed(cap=cap)
    core.set_model_state('network_wait')
    for step in range(1, 14):
        t = 10.05 + step * .05
        assert core.tick(round(t * 1e9), t)[:2] == (0., 0.)
    assert not core.armed


def test_local_waypoint_expiry_pauses_without_model_fault_message(cap):
    core = armed(cap=cap)
    for step in range(2, 16):
        t = 10. + step * .05
        result = core.tick(sensors(core, t), t)
    assert result[:2] == (0., 0.)
    assert core.motion_paused and core.armed
    commands = recover(core, start=10.8)
    assert commands[-1][0] > 0.


def test_pause_does_not_reuse_pre_pause_camera_or_waypoint(cap):
    core = armed(cap=cap)
    core.set_model_state('network_wait')
    barrier = core.resume_after_stamp
    core.set_model_state('running')
    for step in range(1, 8):
        t = 10.05 + step * .05
        result = core.tick(sensors(core, t), t)
        assert result[:2] == (0., 0.)
    assert core.resume_candidates == 0 and barrier > 0


def test_running_status_before_waypoint_keeps_zero_pause_intent(cap):
    core = armed(cap=cap)
    core.set_model_state('network_wait')
    core.set_model_state('running')
    for step in range(1, 26):
        t = round(10.05 + step*.05, 8)
        assert core.tick(sensors(core, t), t)[:2] == (0., 0.)
        assert core.armed and core.motion_paused and core.resume_candidates == 0
    assert recover(core, start=round(t+.05, 8))[-1][0] > 0.


def test_timer_gap_still_disarms_during_pause(cap):
    core = armed(cap=cap)
    core.set_model_state('network_wait')
    assert core.tick(sensors(core, 10.4), 10.4)[:2] == (0., 0.)
    assert not core.armed


def test_opt_in_requires_explicit_supported_profile_and_bool():
    with pytest.raises(ValueError):
        RobotControlCore(RobotControlLimits(max_linear_mps=.15, max_measured_linear_mps=.3), network_pause_resume=True)
    with pytest.raises(ValueError):
        RobotControlCore(network_pause_resume='true')


def test_each_profile_reaches_its_cap_then_zeroes_and_reramps(cap):
    from nomad_navigation.robot_gateway import CommandWatchdog
    core = armed(cap=cap)
    gateway = CommandWatchdog(cap)
    previous = core.last_linear
    steps = round(cap/.025/.05)+4
    for step in range(2, steps):
        t = round(10. + step*.05, 8)
        now = sensors(core, t, waypoint=True)
        v, w, _ = core.tick(now, t)
        assert 0 <= v <= cap and abs(w) <= min(v, .1)+1e-10
        assert v-previous <= .025*.05+1e-10
        assert gateway.accept((v, 0., 0., 0., 0., w), now, 'base_link', now, t)
        assert gateway.decide(now, t)[:2] == (v, w)
        previous = v
    assert previous == cap
    core.set_model_state('network_wait')
    for step in range(steps, steps+25):
        t = round(10.+step*.05, 8)
        now = sensors(core, t)
        v, w, _ = core.tick(now, t)
        assert (v, w) == (0., 0.) and core.armed
        assert gateway.accept((v, 0., 0., 0., 0., w), now, 'base_link', now, t)
        assert gateway.decide(now, t)[:2] == (0., 0.) and gateway.fault is None
    commands = recover(core, start=round(t+.05, 8))
    assert all(row[:2] == (0., 0.) for row in commands[:12])
    assert 0 < commands[12][0] <= .025*.05+1e-10


def test_speed_increase_does_not_relax_time_or_angular_guards(cap):
    core = armed(cap=cap)
    limits = core.limits
    assert (limits.source_timeout_sec, limits.camera_timeout_sec, limits.waypoint_timeout_sec,
            limits.odom_timeout_sec, limits.max_tick_gap_sec) == (1., 1., .6, .5, .2)
    assert (limits.max_angular_rps, limits.max_measured_angular_rps, limits.min_turning_radius_m,
            limits.linear_acceleration_mps2) == (.1, .3, 1., .025)
    core.set_model_state('network_wait')
    assert not core.accept_odom((0., 0., 0.), (0., 0., .301),
        10_100_000_000, 'odom', 'base_footprint', 10_100_000_000, 10.1)
    assert not core.armed


@pytest.mark.parametrize('reason', sorted(TRANSIENT_PAUSE_REASONS))
def test_client_transient_stays_alive_but_clears_prediction(reason):
    core = CampusClientCore(latch_faults=True, network_pause_resume=True)
    core.last_result_stamp, core.last_result_at = 10_000_000_000, 10.
    core.fail(reason)
    assert core.state == 'network_wait' and not core.fatal
    assert core.last_result_stamp is None and core.last_result_at is None
    assert core.pause_reason == reason


@pytest.mark.parametrize('reason', ['invalid_camera', 'camera_replay', 'clock_rollback',
                                   'competing_model_publisher', 'identity_or_protocol_error'])
def test_client_hard_fault_still_latches(reason):
    core = CampusClientCore(latch_faults=True, network_pause_resume=True)
    core.fail(reason)
    assert core.fatal
    core.fail('network_error')
    assert core.fatal and core.state == reason


def test_explicit_fatal_cannot_be_relabeled_as_recoverable():
    core = CampusClientCore(latch_faults=True, network_pause_resume=True)
    core.fail('network_error', fatal=True)
    assert core.fatal and core.state == 'network_error'
