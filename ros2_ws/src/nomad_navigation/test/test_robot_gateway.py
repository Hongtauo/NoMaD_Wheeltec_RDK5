import pytest

from nomad_navigation.robot_gateway import CommandWatchdog, parse_args


def accept(g, v=.05, w=.025, stamp=10_000_000_000, now=10_000_000_000, mono=1.):
    return g.accept((v, 0., 0., 0., 0., w), stamp, 'base_link', now, mono)


def test_initially_stopped():
    assert CommandWatchdog().decide(10**10, 1.) == (0., 0., 'waiting_command')


@pytest.mark.parametrize('cap', [.05, .1, .2])
def test_reviewed_profiles_accept_cap_and_reject_overspeed(cap):
    g = CommandWatchdog(max_linear_mps=cap)
    assert g.max_linear_mps == cap
    assert accept(g, v=cap, w=min(cap, .1))
    assert g.decide(10**10, 1.)[:2] == (cap, min(cap, .1))
    assert not accept(g, v=cap + .00001, w=0., stamp=11*10**9, now=11*10**9, mono=2.)
    assert g.decide(11*10**9, 2.) == (0., 0., 'invalid_command')


def test_default_profile_does_not_silently_raise_speed():
    assert CommandWatchdog().max_linear_mps == .05
    assert parse_args([]).max_linear_mps == .05
    assert not accept(CommandWatchdog(), v=.1, w=0.)
    for cap in (.05, .1, .2):
        assert parse_args(['--max-linear-mps', str(cap)]).max_linear_mps == cap


@pytest.mark.parametrize('cap', [0., -.05, .15, .20001, float('inf'), float('nan'),
                                 True, None, '0.2', 10**400])
def test_invalid_profile_never_constructs_watchdog(cap):
    with pytest.raises(ValueError):
        CommandWatchdog(max_linear_mps=cap)


@pytest.mark.parametrize('args', [
    ['--max-linear-mps', '.15'], ['--max-linear-mps', '.20001'],
    ['--max-linear-mps', 'nan'], ['--max-linear-mps', 'inf'],
    ['--ros-args', '-r', '/cmd_vel:=/unsafe'], ['--unknown'], ['--max-linear', '.2'],
])
def test_unknown_cli_arguments_and_unreviewed_profiles_are_rejected(args):
    with pytest.raises(SystemExit):
        parse_args(args)


@pytest.mark.parametrize('v,w', [(.2, .10001), (.03, .03001), (0., .01), (-.01, 0.)])
def test_faster_profile_keeps_angular_curvature_and_forward_only_guards(v, w):
    g = CommandWatchdog(max_linear_mps=.2)
    assert not accept(g, v=v, w=w)
    assert g.decide(10**10, 1.) == (0., 0., 'invalid_command')


@pytest.mark.parametrize('cap', [.05, .1, .2])
def test_all_profiles_keep_nonplanar_wrong_frame_and_stale_guards(cap):
    for axis in (1, 2, 3, 4):
        g = CommandWatchdog(max_linear_mps=cap)
        values = [cap, 0., 0., 0., 0., .025]
        values[axis] = .001
        assert not g.accept(values, 10**10, 'base_link', 10**10, 1.)
        assert g.fault == 'invalid_command'
    g = CommandWatchdog(max_linear_mps=cap)
    assert not g.accept((cap, 0., 0., 0., 0., .025), 10**10, 'odom', 10**10, 1.)
    assert g.fault == 'wrong_frame'
    g = CommandWatchdog(max_linear_mps=cap)
    assert not accept(g, v=cap, now=10**10 + 250_000_001)
    assert g.fault == 'stale_command'


@pytest.mark.parametrize('cap', [.05, .1, .2])
def test_profile_timeout_and_reset_remain_fail_closed(cap):
    g = CommandWatchdog(max_linear_mps=cap)
    assert (g.timeout, g.source_timeout, g.future_tolerance) == (.25, .25, .2)
    assert accept(g, v=cap)
    assert g.decide(10**10, 1.)[0] == cap
    assert g.decide(10**10 + 300_000_000, 1.3) == (0., 0., 'command_timeout')
    assert accept(g, v=cap, stamp=11*10**9, now=11*10**9, mono=2.)
    assert g.decide(11*10**9, 2.)[:2] == (0., 0.)
    assert not g.reset(11*10**9, 2.)
    assert accept(g, v=0., w=0., stamp=12*10**9, now=12*10**9, mono=3.)
    assert g.reset(12*10**9, 3.)
    assert g.decide(12*10**9, 3.) == (0., 0., 'stopped')


def test_timeout_latches_and_requires_fresh_zero_reset():
    g = CommandWatchdog()
    assert accept(g)
    assert g.decide(10**10, 1.)[:2] == (.05, .025)
    assert g.decide(10**10 + 300_000_000, 1.3)[2] == 'command_timeout'
    assert accept(g, stamp=11*10**9, now=11*10**9, mono=2.)
    assert g.decide(11*10**9, 2.)[:2] == (0., 0.)
    assert not g.reset(11*10**9, 2.)
    assert accept(g, v=0., w=0., stamp=12*10**9, now=12*10**9, mono=3.)
    assert g.reset(12*10**9, 3.)


@pytest.mark.parametrize('v,w', [(-.01, 0.), (.051, 0.), (0., .01), (.05, .06), (float('nan'), 0.), (.01, float('inf'))])
def test_reject_unsafe_commands(v, w):
    g = CommandWatchdog()
    assert not accept(g, v=v, w=w)
    assert g.decide(10**10, 1.) == (0., 0., 'invalid_command')


def test_replay_and_future_rejected():
    g = CommandWatchdog()
    assert accept(g)
    assert not accept(g)
    assert g.fault == 'replayed_command'
    g = CommandWatchdog()
    assert not accept(g, stamp=11*10**9)
    assert g.fault == 'stale_command'


def test_clock_rollback_permanent():
    g = CommandWatchdog()
    accept(g)
    g.decide(10**10, 1.)
    assert g.decide(9*10**9, 1.1) == (0., 0., 'clock_rollback')
    assert not accept(g, v=float('nan'))
    assert g.fault == 'clock_rollback'
    assert accept(g, v=0., w=0., stamp=11*10**9, now=11*10**9, mono=2.)
    assert not g.reset(11*10**9, 2.)
