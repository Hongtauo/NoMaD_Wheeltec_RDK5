"""Allowlisted operator diagnostics: never reflect arbitrary exception text."""

import re


INITIALIZATION_OPERATIONS = frozenset(('prepare', 'sample', 'set_time', 'verify', 'plan', 'start'))
ERROR_MESSAGES = {
    'initialization_failed': 'Robot initialization failed; inspect the private session failure record',
    'clock_identity_invalid': 'Robot identity or campus environment validation failed',
    'clock_lock_busy_or_unsafe': 'Another operator owns the robot lock, or its permissions are unsafe',
    'clock_lock_unavailable': 'Cannot safely open the robot operator lock',
    'clock_ros_processes_active': 'Stop existing ROS/hardware/control processes before clock preparation',
    'clock_process_inventory_invalid': 'Cannot parse the robot process inventory',
    'clock_command_failed': 'A required robot clock command failed',
    'clock_command_timeout': 'A required robot clock command failed or timed out',
    'clock_service_state_invalid': 'Cannot determine the state of a robot clock service',
    'clock_service_transitioning': 'A robot clock service is still transitioning; retry after it settles',
    'clock_daemon_unmanaged': 'An unmanaged clock daemon is running',
    'clock_daemon_ownership_unknown': 'Cannot establish clock daemon ownership',
    'clock_source_still_active': 'A clock service or daemon remains active',
    'clock_session_phase_invalid': 'Clock operation is not valid in the current session phase',
    'clock_bound_invalid': 'PC/robot clock error bound must be finite and at most 0.1 seconds',
    'clock_timezone_changed': 'Robot timezone changed during clock preparation',
    'clock_boot_id_invalid': 'Cannot validate the robot boot ID',
    'clock_prepare_failed': 'Could not prepare the robot clock session',
}
_EXACT_CLOCK_ERRORS = {
    'Enrolled campus robot identity/environment validation failed': 'clock_identity_invalid',
    'Another operator is running, or the lock is unsafe': 'clock_lock_busy_or_unsafe',
    'Cannot open the robot operator lock': 'clock_lock_unavailable',
    'The operator lock must be a regular file owned by the operator': 'clock_lock_busy_or_unsafe',
    "Cannot parse the robot's process inventory": 'clock_process_inventory_invalid',
    'Required robot command returned an error': 'clock_command_failed',
    'Required robot command failed or timed out': 'clock_command_timeout',
    'Cannot establish clock daemon ownership': 'clock_daemon_ownership_unknown',
    'A clock service is still active or was restarted': 'clock_source_still_active',
    'A clock daemon remains active outside the stopped services': 'clock_source_still_active',
    'Clock operation is not valid in the current session phase': 'clock_session_phase_invalid',
    'Clock preparation can run only once per session': 'clock_session_phase_invalid',
    'PC/robot clock error bound must be finite and at most 0.1 seconds': 'clock_bound_invalid',
    'Robot timezone changed during clock preparation': 'clock_timezone_changed',
    'Cannot validate the robot boot ID': 'clock_boot_id_invalid',
    'Could not prepare robot clock session': 'clock_prepare_failed',
}
_SAFE_TYPES = frozenset(('RobotClockError', 'RuntimeError', 'ValueError', 'OSError',
                        'TimeoutError', 'PermissionError', 'FileNotFoundError'))
_RESTORE_SUFFIX = '; clock service restoration needs attention'
_UNIT = r'(?:ntp|ntpsec|chrony|systemd-timesyncd)\.service'


def _pid_list(value):
    if (not isinstance(value, list) or not 1 <= len(value) <= 32
            or any(type(pid) is not int or not 0 < pid < 2**31 for pid in value)):
        return None
    return list(value)


def safe_initialization_error(error):
    """Convert known clock errors to fixed codes; unknown text is discarded."""
    from robot_clock import RobotClockError
    result = {'error_code': 'initialization_failed', 'error_details': {},
              'error_type': type(error).__name__ if type(error).__name__ in _SAFE_TYPES else 'Exception'}
    if not isinstance(error, RobotClockError):
        return result
    message = str(error)
    if message.endswith(_RESTORE_SUFFIX):
        message = message[:-len(_RESTORE_SUFFIX)]
        result['error_details']['clock_restore_needs_attention'] = True
    if message in _EXACT_CLOCK_ERRORS:
        result['error_code'] = _EXACT_CLOCK_ERRORS[message]
        return result
    for prefix, code in (
            ('Stop existing ROS/hardware/control processes before clock changes; PIDs: ',
             'clock_ros_processes_active'),
            ('An unmanaged clock daemon is running; PID: ', 'clock_daemon_unmanaged')):
        if message.startswith(prefix):
            numbers = message[len(prefix):]
            if len(numbers) <= 351 and re.fullmatch(r'[0-9]+(?:,[0-9]+){0,31}', numbers):
                pids = _pid_list([int(item) for item in numbers.split(',')])
                if pids is not None:
                    result['error_code'] = code
                    result['error_details']['pids'] = pids
            return result
    for prefix, code in (
            ('Cannot determine clock service state: ', 'clock_service_state_invalid'),
            ('Cannot determine clock service PID: ', 'clock_service_state_invalid'),
            ('A clock service is transitioning; retry after it settles: ', 'clock_service_transitioning')):
        if re.fullmatch(re.escape(prefix) + _UNIT, message):
            result['error_code'] = code
            return result
    return result


def rejection_message(operation, payload):
    """Render only trusted local text and validated numeric details on the PC."""
    operation = operation if operation in INITIALIZATION_OPERATIONS else 'operation'
    code = payload.get('error_code')
    if not isinstance(code, str) or code not in ERROR_MESSAGES:
        return f'Robot rejected {operation}; no request was retried'
    message = f'Robot rejected {operation} [{code}]: {ERROR_MESSAGES[code]}'
    details = payload.get('error_details')
    if isinstance(details, dict):
        pids = _pid_list(details.get('pids'))
        if code in ('clock_ros_processes_active', 'clock_daemon_unmanaged') and pids is not None:
            message += '; PIDs: ' + ','.join(str(pid) for pid in pids)
        if details.get('clock_restore_needs_attention') is True:
            message += '; clock service restoration needs attention'
    return message + '; no request was retried'
