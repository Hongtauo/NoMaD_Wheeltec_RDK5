#!/usr/bin/env python3
"""Robot-local SSH operator session. JSON protocol; never arms at startup.

An authenticated Windows console supplies UTC only while the robot is idle.
Each arm requires a separate explicit request. Legacy SSH sessions stop on loss.
The optional campus supervisor owns reconnect lifecycle separately from a short,
controller-enforced motion lease; reconnect never sends arm or resynchronizes clocks.
An explicit optional pause policy lets the controller retain continuous intent,
not motion authority, across a short link outage.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
import uuid

from robot_clock import RobotClockSession

ROBOT_ROOT = Path('/home/wheeltec/nomad_live_control')
HEARTBEAT_TIMEOUT = 4.0
PROCESS_SCAN_INTERVAL = .25


def validate_arm(request):
    if request.get('confirmed') is not True:
        raise ValueError('Each arm requires an on-site operator confirmation')
    mode, seconds = request.get('mode'), request.get('duration_sec')
    if type(mode) is not int or mode not in (0, 1):
        raise ValueError('mode must be 0 (timed) or 1 (continuous)')
    if (type(seconds) not in (float, int) or not math.isfinite(seconds)
            or (mode == 0 and not 0 < seconds <= 10)
            or (mode == 1 and seconds != 0)):
        raise ValueError('Timed duration must be (0,10]; continuous duration must be 0')
    return mode, float(seconds)


class RosOperator:
    """Only services and control/gateway state, never a velocity publisher."""
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from nomad_interfaces.srv import ArmFor
        from std_srvs.srv import Trigger
        from std_msgs.msg import String
        self.rclpy, self.ArmFor, self.Trigger = rclpy, ArmFor, Trigger
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('robot_operator', namespace='/nomad', context=self.context,
                         use_global_arguments=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.states = {}
        self.link_publisher = None
        self.link_session = None
        self.link_sequence = 0
        self.node.create_subscription(String, '/nomad/robot_control/status',
                                      lambda msg: self.receive('control', msg),
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String, '/nomad/robot_gateway/status',
                                      lambda msg: self.receive('gateway', msg), 1)
        self.arm_client = self.node.create_client(ArmFor, '/nomad/robot_control/arm_for')
        self.disarm_client = self.node.create_client(Trigger, '/nomad/robot_control/disarm')

    def receive(self, name, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Invalid status')
            self.states[name] = (value, time.monotonic())
        except (ValueError, TypeError):
            self.states.pop(name, None)

    def spin(self, seconds=.0):
        end = time.monotonic() + seconds
        while True:
            self.executor.spin_once(timeout_sec=.01)
            if time.monotonic() >= end:
                break

    def snapshot(self):
        now = time.monotonic()
        return {name: {'received_age_sec': now - stamp, 'fresh': now - stamp <= .5,
                       'value': value} for name, (value, stamp) in self.states.items()}

    def call(self, client, request, seconds=2.):
        if not client.service_is_ready():
            raise RuntimeError('ROS service is not ready; no request sent')
        future = client.call_async(request)
        end = time.monotonic() + seconds
        while not future.done() and time.monotonic() < end:
            self.spin()
        if not future.done() or future.result() is None:
            # Never retry an uncertain arm: the first request may have arrived.
            raise RuntimeError('Service reply timed out; do not retry arm automatically')
        response = future.result()
        return {'success': response.success, 'message': response.message}

    def arm(self, request):
        mode, seconds = validate_arm(request)
        state = self.snapshot()
        if not all(state.get(name, {}).get('fresh') for name in ('control', 'gateway')):
            raise RuntimeError('Control/gateway status is not fresh')
        if state['gateway']['value'].get('fault') is not None:
            raise RuntimeError('Gateway has a fault; investigate before rearming')
        call = self.ArmFor.Request()
        call.mode, call.duration_sec = mode, seconds
        return self.call(self.arm_client, call)

    def disarm(self):
        return self.call(self.disarm_client, self.Trigger.Request())

    def configure_link_guard(self, session_id):
        """One local publisher for ordered, epoch-bound authorization messages."""
        from rclpy.duration import Duration
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        if self.link_publisher is not None or str(uuid.UUID(session_id)) != session_id:
            raise ValueError('Invalid/repeated operator guard setup')
        self.link_session = session_id
        self.link_publisher = self.node.create_publisher(String,
            '/nomad/robot_control/operator_link', QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE, lifespan=Duration(seconds=.2)))

    def publish_link(self, generation, deadline_steady_ns, connected, *, action='heartbeat',
                     arm_nonce=None, mode=None, duration_sec=None):
        from std_msgs.msg import String
        if self.link_publisher is None:
            raise RuntimeError('Operator guard was not configured')
        self.link_sequence += 1
        message = dict(schema=1, session_id=self.link_session, generation=generation,
                       sequence=self.link_sequence, connected=connected,
                       deadline_steady_ns=deadline_steady_ns, action=action)
        if action == 'arm':
            message.update(arm_nonce=arm_nonce, mode=mode, duration_sec=duration_sec)
        self.link_publisher.publish(String(data=json.dumps(message, allow_nan=False)))

    def arm_guarded(self, request, generation, deadline_steady_ns, arm_nonce):
        """One atomic arm message, never an unscoped/queued ArmFor service call."""
        mode, seconds = validate_arm(request)
        if str(uuid.UUID(arm_nonce)) != arm_nonce:
            raise ValueError('Invalid one-shot arm identity')
        state = self.snapshot()
        if not all(state.get(name, {}).get('fresh') for name in ('control', 'gateway')):
            raise RuntimeError('Control/gateway status is not fresh')
        if state['gateway']['value'].get('fault') is not None:
            raise RuntimeError('Gateway has a fault; investigate before rearming')
        link = state['control']['value'].get('operator_link', {})
        if link.get('generation') != generation or link.get('connected') is not True:
            raise RuntimeError('Controller has not acknowledged this operator generation')
        if deadline_steady_ns - time.monotonic_ns() < 250_000_000:
            raise RuntimeError('Refresh operator heartbeat before a new RUN')
        self.publish_link(generation, deadline_steady_ns, True, action='arm',
                          arm_nonce=arm_nonce, mode=mode, duration_sec=seconds)
        until = min(time.monotonic() + 1., deadline_steady_ns / 1e9)
        while time.monotonic() < until:
            self.spin()
            control = self.snapshot().get('control', {})
            value = control.get('value', {})
            current_link = value.get('operator_link', {})
            result = current_link.get('last_arm_result')
            if (control.get('fresh') is True and isinstance(result, dict)
                    and result.get('arm_nonce') == arm_nonce
                    and result.get('generation') == generation
                    and type(result.get('success')) is bool
                    and isinstance(result.get('message'), str)):
                if result['success'] and (current_link.get('connected') is not True
                        or current_link.get('generation') != generation
                        or value.get('armed') is not True
                        or time.monotonic_ns() >= deadline_steady_ns):
                    raise RuntimeError('Arm acknowledgement no longer has current motion authority')
                return {'success': result['success'], 'message': result['message']}
        raise RuntimeError('Guarded arm reply uncertain; authorization must be revoked, not retried')

    def close(self):
        self.executor.shutdown(timeout_sec=2.)
        self.node.destroy_node()
        self.context.shutdown()


class OperatorSession:
    def __init__(self, transport='dds'):
        if transport not in ('dds', 'api'):
            raise ValueError('Unknown robot transport')
        self.transport = transport
        self.clock = None
        self.token = None
        self.token_path = None
        self.output = None
        self.launcher = None
        self.launch_birth = None
        self.owned_identities = {}
        self.ownership_errors = []
        self.process_cleanup_errors = []
        self.last_process_scan = 0.
        self.launch_log = None
        self.ros = None
        self.running = False
        self.closed = False
        self.close_result = None
        self.last_contact = time.monotonic()
        self.reconnect_session_id = None
        self.operator_generation = 0
        self.operator_deadline_ns = 0
        self.operator_connected = False
        self.operator_pause_resume = False
        self.operator_link_action = 'heartbeat'
        self.last_link_publish = 0.

    def enable_reconnect(self, session_id):
        if (self.transport != 'api' or self.running or self.ros is not None
                or self.reconnect_session_id is not None
                or str(uuid.UUID(session_id)) != session_id):
            raise ValueError('Reconnect guard requires a fresh campus operator session')
        self.reconnect_session_id = session_id

    def operator_link_update(self, generation, deadline_steady_ns, connected):
        if (self.reconnect_session_id is None or type(generation) is not int or generation < 1
                or generation < self.operator_generation or type(deadline_steady_ns) is not int
                or deadline_steady_ns <= 0 or type(connected) is not bool
                or (connected and deadline_steady_ns - time.monotonic_ns() > 2_050_000_000)):
            raise ValueError('Invalid operator authorization lease')
        self.operator_generation = generation
        self.operator_deadline_ns = deadline_steady_ns
        self.operator_connected = connected
        self.operator_link_action = 'heartbeat'
        if self.ros and self.ros.link_publisher is not None:
            self.ros.publish_link(generation, deadline_steady_ns, connected)
            self.last_link_publish = time.monotonic()

    def operator_link_disarm(self, reason):
        """Revoke the current generation; the supervisor alone creates the next."""
        self.operator_connected = False
        self.operator_link_action = 'disarm'
        self.operator_deadline_ns = time.monotonic_ns()
        if self.ros and self.ros.link_publisher is not None and self.operator_generation > 0:
            self.ros.publish_link(self.operator_generation, self.operator_deadline_ns,
                                  False, action='disarm')
            self.last_link_publish = time.monotonic()
        return {'success': True, 'message': 'Motion authority revoked; a new RUN is required'}

    def operator_link_pause(self, reason):
        """Stop motion without renewing the lease or creating any arm intent.

        Only the opted-in running controller can retain already-authorized
        continuous intent, under its independent bounded recovery window.
        """
        if (not self.operator_pause_resume or not self.running or self.closed
                or self.reconnect_session_id is None):
            return self.operator_link_disarm(reason)
        self.operator_connected = False
        self.operator_link_action = 'pause'
        if self.ros and self.ros.link_publisher is not None and self.operator_generation > 0:
            self.ros.publish_link(self.operator_generation, self.operator_deadline_ns,
                                  False, action='pause')
            self.last_link_publish = time.monotonic()
        return {'success': True, 'message': 'Motion paused; controller owns bounded recovery eligibility'}

    def arm_guarded(self, request, generation, deadline_steady_ns, arm_nonce):
        if (not self.running or self.ros is None or self.reconnect_session_id is None
                or not self.operator_connected or generation != self.operator_generation
                or deadline_steady_ns != self.operator_deadline_ns
                or time.monotonic_ns() >= deadline_steady_ns):
            raise RuntimeError('A current operator connection and fresh RUN are required')
        try:
            return self.ros.arm_guarded(request, generation, deadline_steady_ns, arm_nonce)
        except Exception:
            self.operator_link_disarm('uncertain_arm')
            raise

    def dispatch(self, request):
        self.last_contact = time.monotonic()
        op = request.get('op')
        if op == 'ping':
            return {'time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
                    'running': self.running}
        if op == 'prepare':
            if self.clock is not None:
                raise RuntimeError('A clock session is already prepared')
            password = request.pop('sudo_password', None)
            if not isinstance(password, str) or not password:
                raise ValueError('sudo password is required; it is never logged or stored on disk')
            self.clock = (RobotClockSession(password, transport='api') if self.transport == 'api'
                          else RobotClockSession(password))
            return self.clock.prepare()
        if op == 'sample':
            self.require_clock()
            return self.clock.sample()
        if op == 'set_time':
            self.require_clock()
            if self.running or self.token is not None:
                raise RuntimeError('Clock changes are forbidden after verification or startup')
            return self.clock.set_time(request.get('epoch_ns'))
        if op == 'verify':
            self.require_clock()
            self.token = self.clock.verify(request.get('bound_sec'))
            return self.token
        if op == 'plan':
            return {'launch_file': str(ROBOT_ROOT / 'robot_bringup.launch.py'),
                    'max_linear_mps': request.get('max_linear_mps', .05 if self.transport == 'api' else .2),
                    'automatic_arm': False, 'hardware_started': False}
        if op == 'start':
            return self.start(request.get('max_linear_mps', .05 if self.transport == 'api' else .2),
                              network_pause_resume=request.get('network_pause_resume', False),
                              operator_pause_resume=request.get('operator_pause_resume', False))
        if op == 'status':
            return {'running': self.running, 'output': str(self.output) if self.output else None,
                    'operator_pause_resume': self.operator_pause_resume,
                    'ros': self.ros.snapshot() if self.ros else {}}
        if op == 'arm':
            if not self.running or self.ros is None:
                raise RuntimeError('The robot session is not running')
            if self.reconnect_session_id is not None:
                raise RuntimeError('Guarded sessions require a generation-bound one-shot RUN')
            return self.ros.arm(request)
        if op == 'disarm':
            return self.ros.disarm() if self.ros else {'success': True, 'message': 'No owned control process'}
        if op == 'stop':
            return self.close()
        raise ValueError('Unknown operator command')

    def require_clock(self):
        if self.clock is None:
            raise RuntimeError('Prepare the idle robot before synchronizing time')

    def start(self, cap, *, network_pause_resume=False, operator_pause_resume=False):
        if type(cap) not in (float, int) or cap not in (.05, .1, .2):
            raise ValueError('Supported speed caps: 0.05, 0.1 or 0.2 m/s')
        if type(network_pause_resume) is not bool or (network_pause_resume and
                self.transport != 'api'):
            raise ValueError('Network pause/resume requires the campus API transport')
        if (type(operator_pause_resume) is not bool or (operator_pause_resume and
                (self.transport != 'api' or not network_pause_resume
                 or self.reconnect_session_id is None))):
            raise ValueError('Operator pause/resume requires supervised campus API and network pause/resume')
        if self.running or self.launcher is not None or self.token is None:
            raise RuntimeError('Start requires a fresh verified clock session, exactly once')
        if self.transport == 'api':
            from nomad_navigation.robot_identity import (read_campus_client_config,
                                                        require_robot_identity,
                                                        validate_api_environment)
            import platform
            validate_api_environment(os.environ, platform.machine())
            if (self.token.get('transport') != 'api'
                    or self.token.get('robot_identity') != require_robot_identity()):
                raise RuntimeError('Campus clock token does not match the enrolled robot')
            read_campus_client_config()  # Refuse unsafe/missing API credentials before ROS startup.
        self.require_pidfd()  # Fail before ROS/hardware startup if cleanup cannot pin identities.
        self.output = ROBOT_ROOT / 'runs' / (
            'session__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            + '__' + uuid.uuid4().hex[:8])
        self.output.mkdir(parents=True, exist_ok=False)
        self.token_path = self.output / 'clock_token.json'
        with os.fdopen(os.open(self.token_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w') as file:
            json.dump(self.token, file, allow_nan=False)
        env = dict(os.environ, NOMAD_SESSION_TOKEN_PATH=str(self.token_path),
                   NOMAD_OPERATOR_SESSION_ID=self.token['session_id'],
                   ROS_LOG_DIR=str(self.output / 'ros_logs'))
        if self.reconnect_session_id is not None:
            env['NOMAD_OPERATOR_GUARD_SESSION'] = self.reconnect_session_id
        # Never inherit an opt-in policy from the shell or another session.
        env.pop('NOMAD_OPERATOR_PAUSE_RESUME', None)
        if operator_pause_resume:
            env['NOMAD_OPERATOR_PAUSE_RESUME'] = '1'
        self.ros = RosOperator()
        if self.reconnect_session_id is not None:
            self.ros.configure_link_guard(self.reconnect_session_id)
        self.launch_log = (self.output / 'bringup.log').open('w', encoding='utf-8')
        self.launcher = subprocess.Popen(
            ['ros2', 'launch', str(ROBOT_ROOT / 'robot_bringup.launch.py'),
             f'max_linear_mps:={cap}',
             'network_pause_resume:=' + str(network_pause_resume).lower()], env=env, stdout=self.launch_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        self.launch_birth = int(Path(f'/proc/{self.launcher.pid}/stat').read_text().rsplit(')', 1)[1].split()[19])
        self.owned_identities[self.launcher.pid] = str(self.launch_birth)
        self.operator_pause_resume = operator_pause_resume
        self.running = True
        return {'running': True, 'max_linear_mps': cap, 'output': str(self.output),
                'network_pause_resume': network_pause_resume,
                'operator_pause_resume': operator_pause_resume,
                'automatic_arm': False, 'message': 'Starting disarmed; inspect status before explicit arm'}

    def tick(self):
        if self.closed:
            return  # A successful stop must not spin the already-destroyed ROS context.
        if self.ros:
            self.ros.spin()
        if self.running:
            now = time.monotonic()
            if now - self.last_process_scan >= PROCESS_SCAN_INTERVAL:
                self.owned_processes()  # Remember children before wrappers can exit/reparent.
                self.last_process_scan = now
                if self.ownership_errors:
                    raise RuntimeError('; '.join(self.ownership_errors))
            if self.launcher.poll() is not None:
                raise RuntimeError('Required robot launch exited; see bringup.log')
            if self.reconnect_session_id is not None:
                # Republishing preserves the original robot-monotonic expiry;
                # a running Python process can never renew PC authority itself.
                if self.operator_generation > 0 and now - self.last_link_publish >= .1:
                    options = ({'action': self.operator_link_action}
                               if self.operator_link_action != 'heartbeat' else {})
                    self.ros.publish_link(self.operator_generation, self.operator_deadline_ns,
                                          self.operator_connected, **options)
                    self.last_link_publish = now
            elif time.monotonic() - self.last_contact > HEARTBEAT_TIMEOUT:
                raise RuntimeError('Operator console heartbeat lost; disarming and stopping owned session')

    def close(self):
        if self.closed:
            return self.close_result
        errors = []
        if self.ros:
            try:
                if self.reconnect_session_id is not None:
                    self.operator_link_disarm('session_close')
                self.ros.disarm()
                self.ros.spin(.5)
            except Exception as error:
                errors.append('disarm: ' + str(error))
        try:
            launcher_stopped = self.stop_owned_processes()
        except Exception as error:
            launcher_stopped = False
            errors.append('Owned process cleanup: ' + str(error))
        if not launcher_stopped:
            errors.append('Owned ROS processes may remain; use physical estop and inspect processes')
        errors.extend(self.process_cleanup_errors)
        self.running = False
        if self.ros:
            try:
                self.ros.close()
            except Exception as error:
                errors.append('ROS cleanup: ' + str(error))
        if self.launch_log:
            try:
                self.launch_log.close()
            except Exception as error:
                errors.append('Launch log cleanup: ' + str(error))
        if self.token_path:
            try:
                self.token_path.unlink(missing_ok=True)  # Only this session's ephemeral credential.
            except Exception as error:
                errors.append('Session token cleanup: ' + str(error))
        if self.clock:
            try:
                previous = self.clock.restoration_result()
                if (self.clock.phase == 'restored' and isinstance(previous, dict)
                        and previous.get('success') is True and previous.get('pending_services') == []):
                    # prepare() may have refused existing ROS and already rolled back.
                    # Do not misreport that completed rollback as a cleanup failure.
                    clock_result = previous
                else:
                    if not launcher_stopped:
                        raise RuntimeError('Time-service restoration deferred while launch remains active')
                    self.clock._check_no_ros_processes(self.clock._processes())
                    clock_result = self.clock.restore()
                if clock_result.get('success') is not True:
                    errors.append('Clock services were not fully restored: ' + json.dumps(clock_result))
            except Exception as error:
                clock_result = {'error': str(error)}
                errors.append('clock restore: ' + str(error))
        else:
            clock_result = None
        self.closed = True
        result = {'stopped': launcher_stopped, 'errors': errors, 'clock_restore': clock_result,
                  'hardware_stop_guaranteed': False}
        self.close_result = result
        if self.output:
            try:
                (self.output / 'session_result.json').write_text(
                    json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
            except Exception as error:
                errors.append('Session report: ' + str(error))
        return result

    @staticmethod
    def require_pidfd():
        if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
            raise RuntimeError('Linux pidfd APIs are required before robot startup')
        descriptor = os.pidfd_open(os.getpid())
        try:
            signal.pidfd_send_signal(descriptor, 0)  # Permission probe; sends no signal.
        finally:
            os.close(descriptor)

    @staticmethod
    def process_metadata(path):
        raw = (path / 'stat').read_text()
        name, tail = raw.split('(', 1)[1].rsplit(')', 1)
        fields = tail.split()
        return {'pid': int(path.name), 'uid': path.stat().st_uid, 'name': name,
                'state': fields[0], 'ppid': int(fields[1]), 'pgid': int(fields[2]),
                'sid': int(fields[3]), 'birth': fields[19]}

    def independent_ssh_session(self, row, snapshot):
        """Exclude an unreadable login process only using a proven external chain.

        Linux sshd drops dumpability, so same-UID /proc/environ can be unreadable.
        A new login descends from the already-running root sshd daemon, not our
        newly-created Linux session. A name alone or a different SID is not proof.
        """
        visited, chain = set(), []
        while row['pid'] not in visited:
            visited.add(row['pid'])
            chain.append(row)
            if (self.owned_identities.get(row['pid']) == row['birth']
                    or row['sid'] == self.launcher.pid or row['pgid'] == self.launcher.pid):
                return False
            if (row['uid'] == 0 and row['name'] in ('sshd', 'sshd-session')
                    and int(row['birth']) < self.launch_birth):
                # The /proc snapshot is not atomic. Do not exclude a new login
                # using a root sshd identity which vanished during enumeration.
                return all(self.verified_relation(child, parent, 'parent')
                           for child, parent in zip(chain, chain[1:]))
            row = snapshot.get(row['ppid'])
            if row is None or row['pid'] == 1:
                return False  # Reparented unknown processes remain unverifiable.
        return False

    def verified_relation(self, target, source, relation):
        """Revalidate both endpoints before promoting snapshot-derived ownership.

        A snapshot can contain an old parent/anchor and a later, unrelated child
        after PID/SID reuse. Pinning only the child's identity when signalling
        does not repair that false ownership decision. Re-read source and target
        identities around the relationship check; unstable evidence grants no
        ownership (the exact random environment marker remains a fallback).
        """
        try:
            target_before = self.process_metadata(Path(f'/proc/{target["pid"]}'))
            source_before = self.process_metadata(Path(f'/proc/{source["pid"]}'))
            target_after = self.process_metadata(Path(f'/proc/{target["pid"]}'))
            source_after = self.process_metadata(Path(f'/proc/{source["pid"]}'))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            return False
        for expected, before, after in ((target, target_before, target_after),
                                        (source, source_before, source_after)):
            if any(before[key] != expected[key] or after[key] != expected[key]
                   for key in ('pid', 'uid', 'birth')):
                return False
            if any(before[key] != after[key] for key in ('ppid', 'pgid', 'sid')):
                return False
        if relation == 'parent':
            return target_before['ppid'] == source_before['pid']
        if relation == 'session':
            return target_before['sid'] == source_before['sid'] == self.launcher.pid
        raise ValueError('Unknown process relationship')

    def owned_processes(self):
        """Snapshot owned identities; retain unknowns without skipping safe cleanup."""
        if self.launcher is None:
            return {}
        marker = ('NOMAD_OPERATOR_SESSION_ID=' + self.token['session_id']).encode()
        snapshot, issues = {}, []
        for path in Path('/proc').iterdir():
            if not path.name.isdigit() or int(path.name) == os.getpid():
                continue
            try:
                snapshot[int(path.name)] = self.process_metadata(path)
            except (FileNotFoundError, ProcessLookupError):
                continue
            except PermissionError:
                issues.append(f'Cannot inspect process metadata for PID {path.name}')

        live_known = {pid for pid, row in snapshot.items()
                      if row['uid'] == os.getuid() and self.owned_identities.get(pid) == row['birth']}
        # A start_new_session launcher cannot share its SID with unrelated SSH
        # logins. Require a still-identical known member to guard numeric SID reuse.
        session_anchors = [snapshot[pid] for pid in live_known
                           if snapshot[pid]['sid'] == self.launcher.pid]
        for pid, row in snapshot.items():
            if (pid not in live_known and row['uid'] == os.getuid()
                    and row['sid'] == self.launcher.pid and int(row['birth']) >= self.launch_birth):
                if any(self.verified_relation(row, anchor, 'session') for anchor in session_anchors):
                    live_known.add(pid)
        # Children which create new sessions are still ours when ancestry is seen.
        while True:
            descendants = {pid for pid, row in snapshot.items() if pid not in live_known
                           and row['uid'] == os.getuid() and row['ppid'] in live_known
                           and int(row['birth']) >= self.launch_birth
                           and self.verified_relation(row, snapshot[row['ppid']], 'parent')}
            if descendants <= live_known:
                break
            live_known.update(descendants)
        for pid, row in snapshot.items():
            if row['uid'] != os.getuid() or int(row['birth']) < self.launch_birth:
                continue
            if pid not in live_known:
                try:
                    if marker in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0'):
                        current = self.process_metadata(Path(f'/proc/{pid}'))
                        if all(current[key] == row[key] for key in ('pid', 'uid', 'birth')):
                            live_known.add(pid)
                except (FileNotFoundError, ProcessLookupError):
                    continue
                except PermissionError:
                    if row['state'] != 'Z' and not self.independent_ssh_session(row, snapshot):
                        issues.append(f'Cannot verify potential operator process PID {pid}')
            if pid in live_known:
                self.owned_identities[pid] = row['birth']
        self.ownership_errors = issues
        return {pid: snapshot[pid]['birth'] for pid in live_known if snapshot[pid]['state'] != 'Z'}

    def signal_owned(self, pid, birth, sig):
        """Signal a pinned Linux process, never a recycled numeric PID."""
        descriptor = None
        try:
            if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
                raise RuntimeError('Linux pidfd APIs are required for race-safe process cleanup')
            descriptor = os.pidfd_open(pid)
            row = self.process_metadata(Path(f'/proc/{pid}'))
            if row['birth'] == birth and row['uid'] == os.getuid() and row['state'] != 'Z':
                signal.pidfd_send_signal(descriptor, sig)
        except (FileNotFoundError, ProcessLookupError):
            pass
        finally:
            if descriptor is not None:
                os.close(descriptor)

    def stop_owned_processes(self):
        if self.launcher is None:
            return True
        self.process_cleanup_errors = []
        sent = set()
        for sig, delay in ((signal.SIGINT, 20.), (signal.SIGTERM, 5.)):
            until = time.monotonic() + delay
            while time.monotonic() < until:
                self.launcher.poll()
                owned = self.owned_processes()
                if not owned and self.launcher.poll() is not None:
                    self.process_cleanup_errors.extend(self.ownership_errors)
                    return not self.ownership_errors and not self.process_cleanup_errors
                # Let ROS launch perform its ordered stop first. If it exits
                # before native grandchildren, signal those exact orphans now.
                targets = {self.launcher.pid: owned[self.launcher.pid]} if (
                    sig == signal.SIGINT and self.launcher.pid in owned) else owned
                for pid, birth in targets.items():
                    key = (pid, birth, sig)
                    if key in sent:
                        continue
                    try:
                        self.signal_owned(pid, birth, sig)
                        sent.add(key)
                    except Exception as error:
                        self.process_cleanup_errors.append(f'Cannot signal owned PID {pid}: {error}')
                        sent.add(key)
                time.sleep(.1)
        self.process_cleanup_errors.extend(self.ownership_errors)
        return False  # Do not pretend a launch parent exit proved hardware stopped.


def read_requests(stream, inbox):
    try:
        for line in stream:
            if len(line) > 8192:
                raise ValueError('Oversized operator request')
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError('Operator requests must be objects')
            inbox.put(request)
    except Exception:
        inbox.put({'op': '_protocol_error'})
    finally:
        inbox.put(None)


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--transport', choices=('dds', 'api'), default='dds')
    args = parser.parse_args()
    if os.environ.get('NOMAD_TRANSPORT', 'dds') != args.transport:
        parser.error('Selected transport does not match the explicitly sourced robot environment')
    inbox = queue.Queue()
    session = OperatorSession(transport=args.transport)
    stop_signal = False

    def request_stop(*_args):
        nonlocal stop_signal
        stop_signal = True

    def reply(value):
        print(json.dumps(value, allow_nan=False), flush=True)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    threading.Thread(target=read_requests, args=(sys.stdin, inbox), daemon=True).start()
    reply({'event': 'ready', 'protocol': 1, 'automatic_arm': False})
    result = 0
    try:
        while not stop_signal and not session.closed:
            try:
                request = inbox.get(timeout=.05)
            except queue.Empty:
                session.tick()
                continue
            if request is None:
                break
            identifier = request.get('id')
            try:
                value = session.dispatch(request)
                reply({'id': identifier, 'ok': True, 'result': value})
            except Exception as error:
                # Deliberately never echo a request (prepare carries a secret).
                reply({'id': identifier, 'ok': False, 'error': str(error)})
                if request.get('op') in ('prepare', 'set_time', 'verify', 'start', '_protocol_error'):
                    result = 1
                    break
            session.tick()
    except Exception as error:
        result = 1
        reply({'event': 'fault', 'error': str(error)})
    finally:
        cleanup = session.close()
        reply({'event': 'closed', 'result': cleanup})
        if cleanup.get('errors'):
            result = 1
    return result


if __name__ == '__main__':
    raise SystemExit(main())
