"""Pure session-token tests; neither system clock nor robot is modified."""

import json
import math
import os
import uuid

import pytest

from nomad_navigation.robot_session_guard import (
    read_session_token, validate_target, validate_token_data)


def token():
    return dict(schema=1, verified=True, boot_id='test-boot', monotonic_sec=100.,
                pc_clock_bound_sec=0.02, uid=1000, session_id=str(uuid.uuid4()))


def validate(data, mono=101.):
    return validate_token_data(data, uid=1000, boot_id='test-boot', mono=mono)


def test_recent_bounded_token_accepted():
    data = token()
    assert validate(data) is data
    assert validate(data, mono=160.) is data


@pytest.mark.parametrize('changes', [
    {'schema': True}, {'schema': 2}, {'verified': 1}, {'verified': False},
    {'boot_id': 'previous-boot'}, {'uid': 0}, {'uid': 1000.},
    {'session_id': 'invalid'}, {'session_id': None},
    {'monotonic_sec': math.nan}, {'monotonic_sec': True},
    {'monotonic_sec': -1.}, {'monotonic_sec': 102.},
    {'pc_clock_bound_sec': math.inf}, {'pc_clock_bound_sec': -0.01},
    {'pc_clock_bound_sec': 0.100001}, {'pc_clock_bound_sec': '0.02'},
])
def test_invalid_token_rejected(changes):
    data = token()
    data.update(changes)
    with pytest.raises(ValueError):
        validate(data)


def test_expired_token_rejected():
    with pytest.raises(ValueError):
        validate(token(), mono=160.001)


def test_missing_token_fields_rejected():
    for key in token():
        data = token()
        del data[key]
        with pytest.raises(ValueError):
            validate(data)


def test_target_restrictions():
    env = dict(ROS_DOMAIN_ID='0', ROS_LOCALHOST_ONLY='0',
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    validate_target(env, 'aarch64', ['192.168.0.100', '127.0.0.1'])
    for changed, machine, addresses in (
            ({**env, 'ROS_DOMAIN_ID': '86'}, 'aarch64', ['192.168.0.100']),
            ({**env, 'ROS_LOCALHOST_ONLY': '1'}, 'aarch64', ['192.168.0.100']),
            ({**env, 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}, 'aarch64', ['192.168.0.100']),
            (env, 'x86_64', ['192.168.0.100']), (env, 'aarch64', ['192.168.0.180'])):
        with pytest.raises(ValueError):
            validate_target(changed, machine, addresses)


@pytest.mark.skipif(os.name != 'posix', reason='Robot owner/mode/symlink semantics are POSIX')
def test_token_file_permissions_owner_and_symlinks(tmp_path):
    path = tmp_path / 'session.json'
    data = token()
    data['uid'] = os.getuid()
    path.write_text(json.dumps(data))
    path.chmod(0o600)
    assert read_session_token(str(path), boot_id='test-boot', mono=101.) == data
    path.chmod(0o644)
    with pytest.raises(ValueError):
        read_session_token(str(path), boot_id='test-boot', mono=101.)
    path.chmod(0o600)
    linked = tmp_path / 'linked.json'
    linked.symlink_to(path)
    with pytest.raises(OSError):
        read_session_token(str(linked), boot_id='test-boot', mono=101.)
    with pytest.raises(ValueError):
        read_session_token(str(path), uid=os.getuid() + 1, boot_id='test-boot', mono=101.)
    fifo = tmp_path / 'not-a-token'
    os.mkfifo(fifo, 0o600)
    with pytest.raises(ValueError, match='regular file'):
        read_session_token(str(fifo), boot_id='test-boot', mono=101.)


def test_absolute_token_path_required():
    for path in (None, '', 'relative.json'):
        with pytest.raises(ValueError):
            read_session_token(path, uid=1000, boot_id='test-boot', mono=101.)
