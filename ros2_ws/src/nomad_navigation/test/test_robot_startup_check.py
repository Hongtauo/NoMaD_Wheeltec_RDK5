"""No ROS initialization or hardware I/O: only the startup decision logic."""

import math
import sys
from types import ModuleType, SimpleNamespace

import pytest

from nomad_navigation.robot_startup_check import (
    StartupObservation, command_values, endpoint_names, numeric_equal)


OUTPUT = ['/nomad/robot_gateway']
DESIRED = ['/nomad/robot_control']


def status_pair(check):
    return (
        dict(armed=False, active_mode=None, limits=dict(check.expected_limits),
             linear_x=0.0, angular_z=0.0),
        dict(fault=None, state='stopped', v=0.0, w=0.0,
             max_linear_mps=check.expected_limits['max_linear_mps']),
    )


def test_operator_pause_startup_requires_matching_guard_and_all_policies():
    session = '6b475048-4489-4f2c-9fd1-9a1c544b1d5f'
    check = StartupObservation('0.05',session,operator_pause_resume=True)
    control, _ = status_pair(check)
    control.update(operator_pause_resume=True,network_pause_resume=True,operator_paused=False,
        operator_link={'enabled':True,'session_id':session,'pause_resume':True})
    check.accept_status('control',control,10.)
    assert check.failure is None
    for key in ('operator_pause_resume','network_pause_resume','operator_paused','operator_link'):
        other = StartupObservation('0.05',session,operator_pause_resume=True)
        missing = dict(control)
        missing.pop(key)
        other.accept_status('control',missing,10.)
        assert other.failure


def test_operator_pause_startup_refuses_unrequested_policy_or_missing_session():
    with pytest.raises(ValueError):
        StartupObservation('0.05',operator_pause_resume=True)
    check = StartupObservation('0.05')
    control, _ = status_pair(check)
    control['operator_pause_resume'] = True
    check.accept_status('control',control,10.)
    assert check.failure


def refresh(check, mono):
    control, gateway = status_pair(check)
    check.accept_status('control', control, mono)
    check.accept_status('gateway', gateway, mono)
    check.accept_command('desired', (0.,) * 6, mono)
    check.accept_command('output', (0.,) * 6, mono)


@pytest.mark.parametrize('profile', ['0.05', '0.1', '0.2'])
def test_requires_continuous_fresh_zero_observation(profile):
    check = StartupObservation(profile)
    assert not check.evaluate(10., OUTPUT, DESIRED)[0]
    refresh(check, 10.)
    assert not check.evaluate(10., OUTPUT, DESIRED)[0]
    refresh(check, 10.5)
    assert not check.evaluate(10.5, OUTPUT, DESIRED)[0]
    refresh(check, 11.)
    assert check.evaluate(11., OUTPUT, DESIRED)[0]


def test_stale_sample_restarts_observation_window():
    check = StartupObservation('0.2')
    refresh(check, 10.)
    check.evaluate(10., OUTPUT, DESIRED)
    assert not check.evaluate(10.6, OUTPUT, DESIRED)[0]
    refresh(check, 11.)
    assert not check.evaluate(11., OUTPUT, DESIRED)[0]
    refresh(check, 11.5)
    check.evaluate(11.5, OUTPUT, DESIRED)
    refresh(check, 12.)
    assert check.evaluate(12., OUTPUT, DESIRED)[0]


def test_three_profiles_match_gateway_caps_and_preserve_other_limits():
    from nomad_navigation.robot_startup_check import SPEED_PROFILES
    from nomad_navigation.robot_gateway import REVIEWED_LINEAR_CAPS
    assert SPEED_PROFILES == {'0.05': (.05, .12), '0.1': (.1, .2), '0.2': (.2, .3)}
    assert tuple(value[0] for value in SPEED_PROFILES.values()) == REVIEWED_LINEAR_CAPS
    for profile in SPEED_PROFILES:
        limits = StartupObservation(profile).expected_limits
        assert limits['max_angular_rps'] == .1
        assert limits['linear_acceleration_mps2'] == .025
        assert limits['min_turning_radius_m'] == 1.


@pytest.mark.parametrize('source', ['output', 'desired'])
@pytest.mark.parametrize('value', [0.01, -0.01, math.nan, math.inf, True, '0'])
def test_nonzero_or_invalid_commands_latch(source, value):
    check = StartupObservation('0.2')
    check.accept_command(source, (value, 0., 0., 0., 0., 0.), 10.)
    refresh(check, 11.)
    assert check.failure
    assert not check.evaluate(11., OUTPUT, DESIRED)[0]


@pytest.mark.parametrize('source,changes', [
    ('control', {'armed': True}),
    ('control', {'armed': 0}),
    ('control', {'active_mode': 'continuous'}),
    ('control', {'linear_x': 0.2}),
    ('control', {'angular_z': math.nan}),
    ('control', {'limits': {'max_linear_mps': 0.2}}),
    ('gateway', {'fault': 'clock_rollback'}),
    ('gateway', {'max_linear_mps': 0.05}),
    ('gateway', {'v': 0.2, 'state': 'forwarding'}),
    ('gateway', {'w': math.nan}),
])
def test_status_mismatch_latches(source, changes):
    check = StartupObservation('0.2')
    control, gateway = status_pair(check)
    state = control if source == 'control' else gateway
    state.update(changes)
    check.accept_status(source, state, 10.)
    refresh(check, 11.)
    assert check.failure
    assert not check.evaluate(11., OUTPUT, DESIRED)[0]


def test_gateway_must_have_received_zero_not_merely_started():
    check = StartupObservation('0.2')
    control, gateway = status_pair(check)
    gateway['state'] = 'waiting_command'
    check.accept_status('control', control, 10.)
    check.accept_status('gateway', gateway, 10.)
    assert check.failure is None
    assert not check.evaluate(10., OUTPUT, DESIRED)[0]


@pytest.mark.parametrize('output,desired', [
    (OUTPUT * 2, DESIRED), (OUTPUT, DESIRED * 2),
    (['/teleop'], DESIRED), (OUTPUT, ['/nomad/other_control']),
])
def test_unexpected_publishers_are_latched_not_waited_out(output, desired):
    check = StartupObservation('0.2')
    refresh(check, 10.)
    assert not check.evaluate(10., output, desired)[0]
    assert check.failure
    refresh(check, 11.)
    assert not check.evaluate(11., OUTPUT, DESIRED)[0]


def test_missing_expected_publishers_can_be_discovered_later():
    check = StartupObservation('0.2')
    refresh(check, 10.)
    assert not check.evaluate(10., [], DESIRED)[0]
    assert check.failure is None


def test_message_and_identity_adapters():
    xyz = SimpleNamespace(x=0., y=0., z=0.)
    twist = SimpleNamespace(linear=xyz, angular=xyz)
    assert command_values(twist) == (0.,) * 6
    assert command_values(SimpleNamespace(twist=twist)) == (0.,) * 6
    assert endpoint_names([SimpleNamespace(node_namespace='/nomad',
                                          node_name='robot_gateway')]) == OUTPUT
    assert not numeric_equal(True, 1.)


def test_unreviewed_profile_rejected():
    with pytest.raises(ValueError):
        StartupObservation('0.3')


@pytest.mark.parametrize('failure', [None, 'subscription', 'spin'])
def test_private_context_owns_executor_and_all_setup_paths_cleanup(monkeypatch, failure):
    """Old global rclpy.spin_once path fails this regression without ROS I/O."""
    from nomad_navigation import robot_session_guard, robot_startup_check
    events = []
    instances = []

    class Context:
        def __init__(self):
            self.initialized = False
            instances.append(self)

        def ok(self):
            return self.initialized

        def shutdown(self):
            events.append('context.shutdown')
            self.initialized = False

    class Node:
        def __init__(self, *args, context, **kwargs):
            assert context is instances[0] and context.initialized
            self.context = context

        def create_subscription(self, *_args):
            if failure == 'subscription':
                raise RuntimeError('subscription failed')
            return object()

        def get_publishers_info_by_topic(self, _topic):
            return []

        def get_logger(self):
            return SimpleNamespace(info=lambda _message: None)

        def destroy_node(self):
            events.append('node.destroy')

    class Executor:
        def __init__(self, *, context):
            assert context is instances[0] and context.initialized
            self.context = context
            events.append('executor.created')

        def add_node(self, node):
            assert node.context is self.context
            events.append('executor.add_node')

        def spin_once(self, *, timeout_sec):
            assert timeout_sec == .05
            events.append('executor.spin')
            if failure == 'spin':
                raise RuntimeError('spin failed')

        def shutdown(self, *, timeout_sec):
            assert timeout_sec == 2.
            events.append('executor.shutdown')

    def init(*, args, context):
        assert args == [] and context is instances[0]
        context.initialized = True

    def forbidden_global_spin(*_args, **_kwargs):
        raise AssertionError('Do not use the global executor with a private Context')

    modules = {
        'rclpy': {'init': init, 'spin_once': forbidden_global_spin},
        'rclpy.context': {'Context': Context},
        'rclpy.node': {'Node': Node},
        'rclpy.executors': {'SingleThreadedExecutor': Executor},
        'rclpy.qos': {
            'QoSProfile': lambda **kwargs: SimpleNamespace(**kwargs),
            'DurabilityPolicy': SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=0),
            'ReliabilityPolicy': SimpleNamespace(RELIABLE=1)},
        'geometry_msgs.msg': {'Twist': object, 'TwistStamped': object},
        'std_msgs.msg': {'String': object},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(robot_session_guard, 'require_robot_session', lambda: {})
    monkeypatch.setattr(StartupObservation, 'evaluate', lambda *_args: (True, 'verified'))
    if failure:
        with pytest.raises(RuntimeError, match=failure):
            robot_startup_check.main([])
    else:
        assert robot_startup_check.main([]) == 0
    assert events[:2] == ['executor.created', 'executor.add_node']
    assert events[-3:] == ['executor.shutdown', 'node.destroy', 'context.shutdown']
    assert len(instances) == 1 and not instances[0].ok()
