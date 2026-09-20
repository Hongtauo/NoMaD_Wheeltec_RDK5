"""Explicit robot enrollment for campus DHCP deployments; never changes the OS.

The enrollment is owner-only configuration, not authentication against code
already running as wheeltec. SSH host-key verification authenticates the remote
machine; these checks prevent accidentally starting actuation on another host.
"""

import ipaddress
import json
import os
from pathlib import Path
import platform
import re
import socket
import stat
import subprocess
from urllib.parse import urlsplit


CAMPUS_ROOT = Path('/home/wheeltec/nomad_live_control/campus')
IDENTITY_PATH = CAMPUS_ROOT / 'identity.json'
CLIENT_CONFIG_PATH = CAMPUS_ROOT / 'client.json'
DDS_ENVIRONMENT = ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                   'FASTDDS_ENVIRONMENT_FILE',
                   'ROS_DISCOVERY_SERVER', 'SKIP_DEFAULT_XML', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
                   'ROS_STATIC_PEERS', 'ROS_AUTOMATIC_DISCOVERY_RANGE')


def read_private_json(path, *, uid=None):
    """Read the checked descriptor, not a potentially substituted path/symlink."""
    if not isinstance(path, (str, Path)) or not Path(path).is_absolute():
        raise ValueError('Robot configuration requires an absolute file path')
    uid = os.getuid() if uid is None else uid
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, 'r', encoding='utf-8') as stream:
        metadata = os.fstat(stream.fileno())
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError('Robot configuration must be a regular file')
        if metadata.st_uid != uid or stat.S_IMODE(metadata.st_mode) != 0o600:
            raise ValueError('Robot configuration must be owned by the current user with mode 0600')
        if metadata.st_size > 4096:
            raise ValueError('Robot configuration is unexpectedly large')
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError('Robot configuration must be a JSON object')
    return data


def validate_identity_data(data):
    fields = {'schema', 'machine_id', 'permanent_wifi_mac', 'hostname', 'wifi_interface'}
    if not isinstance(data, dict) or set(data) != fields:
        raise ValueError('Robot enrollment fields are incomplete or unknown')
    if type(data['schema']) is not int or data['schema'] != 1:
        raise ValueError('Unsupported robot enrollment schema')
    for key, pattern in (
            ('machine_id', r'[0-9a-f]{32}'),
            ('permanent_wifi_mac', r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}'),
            ('hostname', r'[A-Za-z0-9][A-Za-z0-9._-]{0,252}'),
            ('wifi_interface', r'[A-Za-z0-9][A-Za-z0-9_.-]{0,14}')):
        if not isinstance(data[key], str) or not re.fullmatch(pattern, data[key]):
            raise ValueError('Invalid enrolled robot ' + key)
    if data['machine_id'] == '0' * 32 or data['permanent_wifi_mac'] in (
            '00:00:00:00:00:00', 'ff:ff:ff:ff:ff:ff'):
        raise ValueError('Empty robot hardware identity is forbidden')
    return data


def permanent_wifi_mac(interface):
    base = Path('/sys/class/net') / interface
    if not (base / 'wireless').is_dir():
        raise ValueError('Enrolled interface is not a local Wi-Fi interface')
    # Linux marks a driver-assigned permanent address with NET_ADDR_PERM=0.
    # Never mistake a randomized current address for a permanent identity.
    if (base / 'addr_assign_type').read_text().strip() == '0':
        return (base / 'address').read_text().strip().lower()
    result = subprocess.run(['ethtool', '-P', interface], capture_output=True,
                            text=True, timeout=5, check=True)
    matched = re.fullmatch(r'Permanent address: ((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})',
                           result.stdout.strip())
    if matched is None:
        raise ValueError('Cannot establish the permanent Wi-Fi MAC address')
    return matched.group(1).lower()


def require_robot_identity(path=IDENTITY_PATH):
    if platform.machine().lower() not in ('aarch64', 'arm64') or os.getuid() == 0:
        raise ValueError('Campus robot startup requires the non-root ARM64 robot user')
    expected = validate_identity_data(read_private_json(path))
    actual = dict(expected,
                  machine_id=Path('/etc/machine-id').read_text().strip().lower(),
                  hostname=socket.gethostname(),
                  permanent_wifi_mac=permanent_wifi_mac(expected['wifi_interface']))
    if actual != expected:
        raise ValueError('Machine ID, permanent Wi-Fi MAC or hostname does not match robot enrollment')
    return expected


def validate_api_environment(environment, machine):
    if (environment.get('NOMAD_TRANSPORT') != 'api'
            or environment.get('ROS_DOMAIN_ID') != '0'
            or environment.get('ROS_LOCALHOST_ONLY') != '1'
            or environment.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp'
            or machine.lower() not in ('aarch64', 'arm64')):
        raise ValueError('Campus mode requires ARM64, domain 0, localhost-only Fast DDS and NOMAD_TRANSPORT=api')
    if any(environment.get(key) for key in DDS_ENVIRONMENT):
        raise ValueError('Campus mode must not retain cross-network DDS configuration')


def read_campus_client_config(path=CLIENT_CONFIG_PATH):
    data = read_private_json(path)
    if set(data) != {'server_url', 'ca_file', 'token_file'}:
        raise ValueError('Campus client configuration needs server_url, ca_file and token_file only')
    if any(not isinstance(value, str) or not value or any(c in value for c in '\r\n\x00')
           for value in data.values()):
        raise ValueError('Invalid campus client configuration value')
    url = urlsplit(data['server_url'])
    if (url.scheme != 'https' or not url.hostname or url.username is not None
            or url.password is not None or url.query or url.fragment or url.path not in ('', '/')
            or url.port is None or not 1 <= url.port <= 65535):
        raise ValueError('Campus server must be an explicit HTTPS origin with a port')
    for key in ('ca_file', 'token_file'):
        candidate = Path(data[key])
        if not candidate.is_absolute() or candidate.parent != CAMPUS_ROOT:
            raise ValueError('Campus credentials must be direct files in the enrolled campus directory')
        descriptor = os.open(candidate, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(descriptor, 'rb') as stream:
            metadata = os.fstat(stream.fileno())
            if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != os.getuid()
                    or metadata.st_size == 0 or metadata.st_mode & 0o022
                    or (key == 'token_file' and stat.S_IMODE(metadata.st_mode) != 0o600)):
                raise ValueError('Campus credential has unsafe owner, permissions or content size')
    return data


def validate_robot_host(value):
    """IPv4-only SSH target; no options, shell syntax, DNS surprises or userinfo."""
    address = ipaddress.IPv4Address(value)
    if (address.is_unspecified or address.is_loopback or address.is_multicast
            or address.is_link_local or int(address) == 0xffffffff):
        raise ValueError('Robot SSH target must be a routable unicast IPv4 address')
    return str(address)
