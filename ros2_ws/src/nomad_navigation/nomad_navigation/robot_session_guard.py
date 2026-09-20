"""Validate the operator's recent pre-start clock check, without changing time.

The owner-only token prevents accidental bare launch and stale-session reuse;
it is not authentication against code already running as the robot user.
"""

import json
import math
import os
from pathlib import Path
import platform
import stat
import subprocess
import time
import uuid


TOKEN_ENV = 'NOMAD_SESSION_TOKEN_PATH'
TOKEN_MAX_AGE_SEC = 60.0


def validate_token_data(data, *, uid, boot_id, mono):
    if not isinstance(data, dict):
        raise ValueError('Session token must be a JSON object')
    if type(data.get('schema')) is not int or data['schema'] != 1:
        raise ValueError('Unsupported session token schema')
    if data.get('verified') is not True:
        raise ValueError('Session clock verification is missing')
    if type(data.get('uid')) is not int or data['uid'] != uid:
        raise ValueError('Session token belongs to a different user')
    if data.get('boot_id') != boot_id:
        raise ValueError('Session token belongs to a different robot boot')
    try:
        session_id = data['session_id']
        if not isinstance(session_id, str):
            raise ValueError()
        uuid.UUID(session_id)
    except (ValueError, TypeError, AttributeError, KeyError) as error:
        raise ValueError('Session token needs a valid session UUID') from error
    for key in ('monotonic_sec', 'pc_clock_bound_sec'):
        value = data.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            raise ValueError(f'Invalid session token {key}')
    if not 0 <= mono - data['monotonic_sec'] <= TOKEN_MAX_AGE_SEC:
        raise ValueError('Session clock token expired; run the operator startup command again')
    if not 0 <= data['pc_clock_bound_sec'] <= 0.1:
        raise ValueError('Verified PC/robot clock bound must be at most 0.1 seconds')
    return data


def read_session_token(path, *, uid=None, boot_id=None, mono=None):
    if not path or not os.path.isabs(path):
        raise ValueError(f'{TOKEN_ENV} must identify an absolute, owner-only token file')
    uid = os.getuid() if uid is None else uid
    boot_id = (Path('/proc/sys/kernel/random/boot_id').read_text().strip()
               if boot_id is None else boot_id)
    mono = time.monotonic() if mono is None else mono
    # O_NOFOLLOW and validation of the open descriptor avoid following a token
    # symlink or validating a different inode from the one that is read.
    # NONBLOCK also prevents an accidentally supplied FIFO from hanging before
    # its non-regular inode can be rejected below.
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, 'r', encoding='utf-8') as stream:
        metadata = os.fstat(stream.fileno())
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError('Session token must be a regular file')
        if metadata.st_uid != uid or stat.S_IMODE(metadata.st_mode) != 0o600:
            raise ValueError('Session token must be owned by the current user with mode 0600')
        if metadata.st_size > 4096:
            raise ValueError('Session token is unexpectedly large')
        data = json.load(stream)
    return validate_token_data(data, uid=uid, boot_id=boot_id, mono=mono)


def validate_target(environment, machine, addresses):
    if environment.get('NOMAD_TRANSPORT') == 'api':
        from .robot_identity import validate_api_environment
        validate_api_environment(environment, machine)
        return
    if environment.get('NOMAD_TRANSPORT', 'dds') != 'dds':
        raise ValueError('Unknown robot transport')
    if environment.get('ROS_DOMAIN_ID') != '0' or environment.get('ROS_LOCALHOST_ONLY') != '0':
        raise ValueError('Robot startup requires ROS_DOMAIN_ID=0 and ROS_LOCALHOST_ONLY=0')
    if machine != 'aarch64' or '192.168.0.100' not in addresses:
        raise ValueError('Robot startup is restricted to the reviewed ARM robot at 192.168.0.100')
    if environment.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp':
        raise ValueError('Use the reviewed rmw_fastrtps_cpp robot environment')


def require_robot_session():
    api = os.environ.get('NOMAD_TRANSPORT') == 'api'
    addresses = [] if api else subprocess.check_output(['hostname', '-I'], text=True).split()
    validate_target(os.environ, platform.machine(), addresses)
    token = read_session_token(os.environ.get(TOKEN_ENV))
    if api:
        from .robot_identity import require_robot_identity
        if token.get('transport') != 'api' or token.get('robot_identity') != require_robot_identity():
            raise ValueError('Clock token does not match the enrolled campus robot')
    elif token.get('transport', 'dds') != 'dds':
        raise ValueError('Clock token belongs to another transport')
    return token
