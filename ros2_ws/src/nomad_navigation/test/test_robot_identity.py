"""No network, robot, clock changes or subprocess effects in enrollment tests."""

import json
import os
from pathlib import Path

import pytest

from nomad_navigation import robot_identity as identity
from nomad_navigation import robot_session_guard as guard


def enrolled():
    return dict(schema=1, machine_id='a' * 32, permanent_wifi_mac='18:ce:df:79:7d:bb',
                hostname='wheeltec', wifi_interface='wlan0')


def environment():
    return dict(NOMAD_TRANSPORT='api', ROS_DOMAIN_ID='0', ROS_LOCALHOST_ONLY='1',
                RMW_IMPLEMENTATION='rmw_fastrtps_cpp')


@pytest.mark.parametrize('changes', [{}, {'extra': 1}, {'schema': True}, {'schema': 2},
                                   {'machine_id': '0' * 32}, {'machine_id': '../bad'},
                                   {'permanent_wifi_mac': '00:00:00:00:00:00'},
                                   {'wifi_interface': '../wlan0'}, {'hostname': '$(whoami)'}])
def test_enrollment_schema_is_explicit(changes):
    data = enrolled()
    if not changes:
        assert identity.validate_identity_data(data) == data
    else:
        with pytest.raises(ValueError):
            identity.validate_identity_data(dict(data, **changes))


def test_api_environment_is_local_dds_not_mutable_campus_ip():
    env = environment()
    guard.validate_target(env, 'aarch64', [])
    guard.validate_target(env, 'arm64', ['10.201.207.37'])
    for key, value in [('ROS_LOCALHOST_ONLY', '0'), ('ROS_DOMAIN_ID', '86'),
                       ('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')]:
        with pytest.raises(ValueError):
            identity.validate_api_environment(dict(env, **{key: value}), 'aarch64')
    for key in identity.DDS_ENVIRONMENT:
        with pytest.raises(ValueError):
            identity.validate_api_environment(dict(env, **{key: '/old/profile'}), 'aarch64')
    with pytest.raises(ValueError):
        identity.validate_api_environment(env, 'x86_64')


@pytest.mark.parametrize('key,value', [('machine_id', 'b' * 32), ('hostname', 'other-robot'),
                                     ('permanent_wifi_mac', '18:ce:df:79:7d:bc')])
def test_identity_requires_all_three_machine_properties(monkeypatch, key, value):
    data = enrolled()
    actual = dict(data, **{key: value})
    monkeypatch.setattr(identity.platform, 'machine', lambda: 'aarch64')
    monkeypatch.setattr(identity.os, 'getuid', lambda: 1000, raising=False)
    monkeypatch.setattr(identity, 'read_private_json', lambda path: data)
    monkeypatch.setattr(identity.Path, 'read_text', lambda path: actual['machine_id'])
    monkeypatch.setattr(identity.socket, 'gethostname', lambda: actual['hostname'])
    monkeypatch.setattr(identity, 'permanent_wifi_mac', lambda iface: actual['permanent_wifi_mac'])
    with pytest.raises(ValueError, match='does not match'):
        identity.require_robot_identity()


def test_correct_identity_is_accepted_without_network(monkeypatch):
    data = enrolled()
    monkeypatch.setattr(identity.platform, 'machine', lambda: 'aarch64')
    monkeypatch.setattr(identity.os, 'getuid', lambda: 1000, raising=False)
    monkeypatch.setattr(identity, 'read_private_json', lambda path: data)
    monkeypatch.setattr(identity.Path, 'read_text', lambda path: data['machine_id'])
    monkeypatch.setattr(identity.socket, 'gethostname', lambda: data['hostname'])
    monkeypatch.setattr(identity, 'permanent_wifi_mac', lambda iface: data['permanent_wifi_mac'])
    assert identity.require_robot_identity() == data


@pytest.mark.parametrize('transport,changed', [('dds', False), ('api', True)])
def test_api_clock_token_cannot_be_from_other_identity_or_transport(monkeypatch, transport, changed):
    monkeypatch.setattr(guard.os, 'environ', environment())
    monkeypatch.setattr(guard.platform, 'machine', lambda: 'aarch64')
    token = dict(transport=transport, robot_identity=enrolled())
    if changed:
        token['robot_identity'] = dict(enrolled(), machine_id='b' * 32)
    monkeypatch.setattr(guard, 'read_session_token', lambda path: token)
    monkeypatch.setattr(identity, 'require_robot_identity', enrolled)
    with pytest.raises(ValueError, match='Clock token'):
        guard.require_robot_session()


def test_api_clock_token_matches_enrollment_and_skips_ip_command(monkeypatch):
    monkeypatch.setattr(guard.os, 'environ', environment())
    monkeypatch.setattr(guard.platform, 'machine', lambda: 'aarch64')
    token = dict(transport='api', robot_identity=enrolled())
    monkeypatch.setattr(guard, 'read_session_token', lambda path: token)
    monkeypatch.setattr(identity, 'require_robot_identity', enrolled)
    def never(*args, **kwargs):
        raise AssertionError('API identity cannot use hostname -I')
    monkeypatch.setattr(guard.subprocess, 'check_output', never)
    assert guard.require_robot_session() == token


@pytest.mark.skipif(os.name != 'posix', reason='Linux file permissions')
def test_private_config_permissions_fifo_and_symlink(tmp_path):
    path = tmp_path / 'identity.json'
    path.write_text(json.dumps(enrolled()))
    path.chmod(0o600)
    assert identity.read_private_json(path) == enrolled()
    with pytest.raises(ValueError, match='owned'):
        identity.read_private_json(path, uid=os.getuid() + 1)
    path.chmod(0o644)
    with pytest.raises(ValueError, match='0600'):
        identity.read_private_json(path)
    path.chmod(0o600)
    link = tmp_path / 'link'
    link.symlink_to(path)
    with pytest.raises(OSError):
        identity.read_private_json(link)
    fifo = tmp_path / 'fifo'
    os.mkfifo(fifo, 0o600)
    with pytest.raises(ValueError, match='regular'):
        identity.read_private_json(fifo)


@pytest.mark.skipif(os.name != 'posix', reason='Linux file permissions')
def test_api_config_https_and_private_credentials(monkeypatch, tmp_path):
    monkeypatch.setattr(identity, 'CAMPUS_ROOT', tmp_path)
    token, ca = tmp_path / 'token', tmp_path / 'server.crt'
    token.write_text('test-only-secret')
    token.chmod(0o600)
    ca.write_text('test-only-certificate')
    ca.chmod(0o644)
    data = dict(server_url='https://10.201.91.120:8443', ca_file=str(ca), token_file=str(token))
    monkeypatch.setattr(identity, 'read_private_json', lambda path: data)
    assert identity.read_campus_client_config() == data
    for url in ('http://10.201.91.120:8443', 'https://user:pass@host:8443',
                'https://host:8443/query?secret=x', 'https://host'):
        data['server_url'] = url
        with pytest.raises(ValueError):
            identity.read_campus_client_config()
    data['server_url'] = 'https://10.201.91.120:8443'
    token.chmod(0o644)
    with pytest.raises(ValueError, match='unsafe'):
        identity.read_campus_client_config()
