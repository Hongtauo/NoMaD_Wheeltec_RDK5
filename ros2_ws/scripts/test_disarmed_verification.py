"""No-network regression checks for the bounded, never-arm observer."""
import copy
from unittest.mock import Mock, patch

import pytest
import robot_console


def ready():
    return dict(running=True, ros=dict(
        control=dict(fresh=True, value=dict(armed=False, active_mode=None,
            linear_x=0., angular_z=0., model_state='running')),
        gateway=dict(fresh=True, value=dict(v=0., w=0., fault=None))))


def test_observer_never_arms_or_changes_state():
    rpc = Mock()
    rpc.call.return_value = ready()
    evidence = {}
    with patch.object(robot_console.time, 'monotonic', side_effect=[0., 1., 12.]), \
            patch.object(robot_console.time, 'sleep'), patch('builtins.print'):
        robot_console.verify_disarmed(rpc, 10, evidence)
    assert evidence['status'] == 'passed'
    assert evidence['arm_requested'] is False
    assert len(evidence['samples']) == 2
    assert all(call.args == ('status',) for call in rpc.call.call_args_list)


def test_opted_in_observer_accepts_only_zero_transient_wait_after_running():
    snapshots = [ready(), ready(), ready()]
    snapshots[1]['ros']['control']['value'].update(model_state='network_wait', network_pause_resume=True)
    rpc = Mock()
    rpc.call.side_effect = snapshots
    evidence = {}
    with patch.object(robot_console.time, 'monotonic', side_effect=[0., 1., 5., 12.]), \
            patch.object(robot_console.time, 'sleep'), patch('builtins.print'):
        robot_console.verify_disarmed(rpc, 10, evidence, allow_network_wait=True)
    assert evidence['status'] == 'passed'
    assert all(call.args == ('status',) for call in rpc.call.call_args_list)


@pytest.mark.parametrize('change', ['armed', 'motion', 'gateway_motion', 'gateway_fault', 'stale', 'model_stopped'])
def test_fault_never_retries_or_rearms(change):
    first = ready()
    second = copy.deepcopy(first)
    cv = second['ros']['control']['value']
    gv = second['ros']['gateway']['value']
    if change == 'armed':
        cv['armed'] = True
    elif change == 'motion':
        cv['linear_x'] = .01
    elif change == 'gateway_motion':
        gv['w'] = .01
    elif change == 'gateway_fault':
        gv['fault'] = 'watchdog'
    elif change == 'stale':
        second['ros']['control']['fresh'] = False
    else:
        cv['model_state'] = 'stale_result'
    rpc = Mock()
    rpc.call.side_effect = [first, second]
    with patch.object(robot_console.time, 'monotonic', side_effect=[0., 1., 2.]), \
            patch.object(robot_console.time, 'sleep'), patch('builtins.print'):
        with pytest.raises(RuntimeError):
            robot_console.verify_disarmed(rpc, 10, {})
    assert all(call.args == ('status',) for call in rpc.call.call_args_list)


def test_startup_wait_is_bounded():
    rpc = Mock()
    rpc.call.return_value = dict(running=True, ros={})
    with patch.object(robot_console.time, 'monotonic', side_effect=[0., 31.]), \
            patch.object(robot_console.time, 'sleep'):
        with pytest.raises(RuntimeError, match='30 seconds'):
            robot_console.verify_disarmed(rpc, 10, {})


@pytest.mark.parametrize('bad_value', [{}, None, [], {'v': 0., 'w': 0.},
    {'v': False, 'w': 0., 'fault': None}, {'v': float('nan'), 'w': 0., 'fault': None},
    {'v': 0., 'w': float('inf'), 'fault': None}])
def test_missing_or_non_numeric_gateway_is_not_zero_evidence(bad_value):
    snapshot = ready()
    snapshot['ros']['gateway']['value'] = bad_value
    rpc = Mock()
    rpc.call.return_value = snapshot
    with patch.object(robot_console.time, 'monotonic', side_effect=[0., 1.]), \
            patch.object(robot_console.time, 'sleep'):
        with pytest.raises(RuntimeError, match='gateway'):
            robot_console.verify_disarmed(rpc, 10, {})


@pytest.mark.parametrize('bad_value', [{}, None, [],
    {'armed': False, 'linear_x': 0., 'angular_z': 0., 'model_state': 'running'},
    {'armed': False, 'active_mode': None, 'linear_x': False, 'angular_z': 0., 'model_state': 'running'}])
def test_incomplete_or_non_numeric_control_is_not_zero_evidence(bad_value):
    snapshot = ready()
    snapshot['ros']['control']['value'] = bad_value
    rpc = Mock()
    rpc.call.return_value = snapshot
    with patch.object(robot_console.time, 'monotonic', side_effect=[0., 1.]), \
            patch.object(robot_console.time, 'sleep'):
        with pytest.raises(RuntimeError, match='controller'):
            robot_console.verify_disarmed(rpc, 10, {})
