"""Error diagnostics expose bounded allowlisted data, never peer/secret text."""

import json

import pytest

from operator_errors import safe_initialization_error, rejection_message
from robot_clock import RobotClockError


@pytest.mark.parametrize('raw,code', [
    ('Another operator is running, or the lock is unsafe', 'clock_lock_busy_or_unsafe'),
    ("Cannot parse the robot's process inventory", 'clock_process_inventory_invalid'),
    ('Enrolled campus robot identity/environment validation failed', 'clock_identity_invalid'),
    ('Required robot command returned an error', 'clock_command_failed'),
    ('A clock service is transitioning; retry after it settles: ntp.service', 'clock_service_transitioning'),
    ('Stop existing ROS/hardware/control processes before clock changes; PIDs: 21557,21558',
     'clock_ros_processes_active'),
])
def test_known_clock_failure_has_actionable_fixed_reason(raw, code):
    payload = safe_initialization_error(RobotClockError(raw))
    assert payload['error_code'] == code
    rendered = rejection_message('prepare', payload)
    assert code in rendered and 'no request was retried' in rendered
    if 'PIDs:' in raw:
        assert payload['error_details']['pids'] == [21557, 21558]
        assert 'PIDs: 21557,21558' in rendered


@pytest.mark.parametrize('error', [
    RuntimeError('never-print-secret'),
    RobotClockError('never-print-secret'),
    RobotClockError('Cannot determine clock service state: never-print-secret'),
    RobotClockError('Stop existing ROS/hardware/control processes before clock changes; PIDs: 2,never-print-secret'),
    type('never-print-secret', (RuntimeError,), {})('never-print-secret'),
])
def test_unknown_or_malicious_exception_is_generic(error):
    payload = safe_initialization_error(error)
    assert payload['error_code'] == 'initialization_failed'
    assert 'never-print-secret' not in json.dumps(payload)
    assert 'never-print-secret' not in rejection_message('prepare', payload)


@pytest.mark.parametrize('pids', [[True], [0], [-1], [2**31], ['never-print-secret'], list(range(1, 34))])
def test_peer_details_are_validated_again(pids):
    rendered = rejection_message('prepare', {'error_code': 'clock_ros_processes_active',
        'error_details': {'pids': pids}, 'error': 'never-print-secret'})
    assert 'PIDs:' not in rendered and 'never-print-secret' not in rendered


def test_peer_unknown_code_and_message_are_not_reflected():
    rendered = rejection_message('prepare', {'error_code': 'never-print-secret',
        'error': 'never-print-secret', 'error_details': {'pids': 'never-print-secret'}})
    assert rendered == 'Robot rejected prepare; no request was retried'


def test_rollback_warning_is_fixed_safe_field():
    payload = safe_initialization_error(RobotClockError(
        'Another operator is running, or the lock is unsafe; clock service restoration needs attention'))
    assert payload['error_code'] == 'clock_lock_busy_or_unsafe'
    assert payload['error_details'] == {'clock_restore_needs_attention': True}
    assert 'clock service restoration needs attention' in rejection_message('prepare', payload)
