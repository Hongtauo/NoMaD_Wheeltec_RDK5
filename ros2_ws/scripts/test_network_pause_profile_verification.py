"""Pure checks for the isolated ROS verification harness; no ROS or hardware."""
import copy

import pytest

from verify_network_pause_isolated import PROFILES, match_publication, parse_args, ramp_summary


def ramp(speed):
    # Synthetic *recorded command samples*, not a robot or controller simulation.
    return [dict(t=10. + step * .05, stamp_ns=round(step * .05 * 1e9),
                 control_steady=10. + step * .05,
                 v=min(step * .05 * .025, speed), w=0.)
            for step in range(1, round((speed / .025 + 1.) / .05))]


@pytest.mark.parametrize('speed,measured', [(.05, .12), (.1, .2), (.2, .3)])
def test_all_three_profiles_select_explicit_caps(speed, measured):
    args = parse_args(['--max-linear-mps', str(speed)])
    assert args.max_linear_mps == speed
    assert PROFILES[speed] == measured


def test_default_is_lowest_profile():
    assert parse_args([]).max_linear_mps == .05


@pytest.mark.parametrize('value', ['0', '.15', '.3', 'nan', 'inf', '-.05'])
def test_unsupported_profiles_fail_before_ros_imports(value):
    with pytest.raises(SystemExit):
        parse_args(['--max-linear-mps', value])


@pytest.mark.parametrize('speed', [.05, .1, .2])
def test_verifier_requires_and_accepts_full_ramp_each_profile(speed):
    result = ramp_summary(ramp(speed), speed, 10.)
    assert result['max_linear_mps'] == result['peak_v'] == speed
    assert result['first_nonzero_v'] == pytest.approx(.00125)
    assert result['first_at_cap_sec'] == pytest.approx(speed / .025)
    assert result['at_cap_output_count'] >= 5


def test_nonzero_is_not_enough_without_reaching_requested_cap():
    with pytest.raises(RuntimeError, match='speed cap'):
        ramp_summary(ramp(.05), .2, 10.)


def test_resume_cannot_jump_back_to_previous_speed():
    samples = ramp(.2)
    samples[0]['v'] = .1
    with pytest.raises(RuntimeError, match='restart from zero'):
        ramp_summary(samples, .2, 10.)


def test_selected_speed_never_relaxes_acceleration():
    samples = ramp(.2)
    samples[10]['v'] += .001
    with pytest.raises(RuntimeError, match='acceleration limit'):
        ramp_summary(samples, .2, 10.)


def test_ros_stamp_jitter_does_not_change_controller_acceleration_clock():
    samples = ramp(.2)
    # The production publisher samples ROS time after tick; a 1 ms variation
    # must not be mistaken for acceleration measured by the earlier steady clock.
    samples[10]['stamp_ns'] -= 1_000_000
    assert ramp_summary(samples, .2, 10.)['peak_v'] == .2


def test_actual_tick_gap_is_not_hidden_by_publication_stamps():
    samples = ramp(.2)
    samples[10]['control_steady'] += .201
    with pytest.raises(RuntimeError, match='timer limit'):
        ramp_summary(samples, .2, 10.)


def test_missing_monotonic_correspondence_never_falls_back_to_ros_clock():
    samples = ramp(.2)
    samples[10]['control_steady'] = None
    with pytest.raises(RuntimeError, match='steady timestamp'):
        ramp_summary(samples, .2, 10.)


def test_ros_output_matches_passive_publication_trace_exactly():
    publications = [dict(v=0., w=0., control_steady=None),
                    dict(v=.001, w=0., control_steady=4.)]
    assert match_publication(publications, 0, 0., 0.) is None
    assert match_publication(publications, 1, .001, 0.) == 4.


@pytest.mark.parametrize('index,speed', [(0, .1), (1, .002), (2, .001)])
def test_unmatched_ros_output_fails(index, speed):
    publications = [dict(v=0., w=0., control_steady=None),
                    dict(v=.001, w=0., control_steady=4.)]
    with pytest.raises(RuntimeError):
        match_publication(publications, index, speed, 0.)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -.01, .3])
def test_invalid_desired_speed_fails_verification(bad):
    samples = ramp(.2)
    samples[10]['v'] = bad
    with pytest.raises(RuntimeError):
        ramp_summary(samples, .2, 10.)


def test_repeated_command_timestamp_fails():
    samples = ramp(.2)
    samples[10]['stamp_ns'] = samples[9]['stamp_ns']
    with pytest.raises(RuntimeError, match='stamps did not increase'):
        ramp_summary(samples, .2, 10.)


def test_at_cap_too_late_fails_without_changing_core_timer_limit():
    samples = copy.deepcopy(ramp(.2))
    for row in samples:
        row['t'] += 1.
    with pytest.raises(RuntimeError, match='ramp arrival'):
        ramp_summary(samples, .2, 10.)


def test_one_cap_sample_is_not_sustained_operation():
    samples = ramp(.2)[:160]
    with pytest.raises(RuntimeError, match='five samples'):
        ramp_summary(samples, .2, 10.)
