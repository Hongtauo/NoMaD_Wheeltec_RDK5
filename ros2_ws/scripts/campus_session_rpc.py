#!/usr/bin/env python3
"""Reconnect only an existing campus session; never replay startup or motion.

The session capability lives in this process's memory and the encrypted SSH
stdin stream. It must not be copied into argv, environment, files or reports.
"""
from datetime import datetime, timezone
import json
import queue
import secrets
import subprocess
import sys
import threading
import time
import uuid

from operator_errors import rejection_message


class ConnectionLost(RuntimeError):
    """Transport confirmation was lost; the operation must not be replayed."""


class ProtocolFault(ConnectionLost):
    """Invalid authenticated protocol state must cancel resumable motion intent."""


class RemoteRejected(RuntimeError):
    """An authenticated peer explicitly rejected this operation."""


class ArmCancelled(RuntimeError):
    """A motion confirmation has expired or crossed a connection boundary."""


class LineTransport:
    """Multiplex replies so a ROS service call cannot block operator heartbeats."""

    def __init__(self, command, hello, *, popen_options=None, timeout=15.):
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=None, text=True, encoding='utf-8',
            bufsize=1, **(popen_options or {}))
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.pending = {}
        self.sequence = 0
        self.failure = None
        self.protocol_error = None
        self.closed_result = None
        self.closed = threading.Event()
        self.reader_done = threading.Event()
        self.ready_queue = queue.Queue(maxsize=1)
        threading.Thread(target=self._read, daemon=True).start()
        try:
            # Do not save the handshake or include it in errors/diagnostics.
            self.process.stdin.write(json.dumps(hello, allow_nan=False) + '\n')
            self.process.stdin.flush()
            ready = self.ready_queue.get(timeout=timeout)
            if isinstance(ready, BaseException):
                raise ready
            if not isinstance(ready, dict) or ready.get('event') != 'ready' or ready.get('protocol') != 1:
                raise ProtocolFault('Invalid campus session handshake')
            self.ready = ready
        except BaseException:
            self.detach()
            raise

    def _fail(self, error):
        with self.lock:
            if self.failure is None:
                self.failure = error
            pending = list(self.pending.values())
        for inbox in pending:
            try:
                inbox.put_nowait(error)
            except queue.Full:
                pass
        try:
            self.ready_queue.put_nowait(error)
        except queue.Full:
            pass

    def _read(self):
        ready_seen = False
        try:
            while True:
                line = self.process.stdout.readline(65537)
                if not line:
                    break
                if len(line.encode('utf-8')) > 65536 or not line.endswith('\n'):
                    raise ValueError('Session response exceeds its line bound')
                message = json.loads(line)
                if not isinstance(message, dict):
                    raise ValueError('Session response must be an object')
                event = message.get('event')
                if event == 'ready':
                    if ready_seen:
                        raise ValueError('Duplicate session handshake')
                    ready_seen = True
                    self.ready_queue.put_nowait(message)
                elif event == 'closed':
                    from robot_console import validate_cleanup_result
                    if self.closed.is_set():
                        raise ValueError('Duplicate terminal session response')
                    self.closed_result = validate_cleanup_result(message.get('result'))
                    self.closed.set()
                    self._fail(ConnectionLost('Remote session closed; request not retried'))
                elif event == 'fault':
                    self.protocol_error = 'remote_fault'
                    self._fail(ProtocolFault('Remote session fault'))
                else:
                    identifier = message.get('id')
                    if type(identifier) is not int:
                        raise ValueError('Invalid session response identifier')
                    with self.lock:
                        inbox = self.pending.get(identifier)
                    if inbox is not None:
                        inbox.put_nowait(message)
        except Exception as error:
            self.protocol_error = type(error).__name__
            self._fail(ProtocolFault(f'SSH protocol {type(error).__name__}'))
        finally:
            self._fail(ConnectionLost('SSH EOF; request confirmation lost'))
            self.reader_done.set()

    def call(self, op, timeout=30., **fields):
        deadline = time.monotonic() + timeout
        inbox = queue.Queue(maxsize=1)
        identifier = None
        try:
            if not self.write_lock.acquire(timeout=max(0., deadline-time.monotonic())):
                raise ConnectionLost('SSH write lock timeout; no request sent')
            try:
                # Allocate and write in the same short critical section: the
                # peer rejects non-increasing ids, including concurrent callers.
                with self.lock:
                    if self.failure is not None:
                        raise self.failure
                    if self.closed.is_set():
                        raise ConnectionLost('Remote session already closed')
                    if len(self.pending) >= 16:
                        raise ConnectionLost('Too many outstanding operator requests')
                    self.sequence += 1
                    identifier = self.sequence
                    self.pending[identifier] = inbox
                self.process.stdin.write(json.dumps({'id': identifier, 'op': op, **fields},
                                                    allow_nan=False) + '\n')
                self.process.stdin.flush()
            except (OSError, ValueError) as error:
                raise ConnectionLost(f'SSH write {type(error).__name__}') from error
            finally:
                self.write_lock.release()
            try:
                message = inbox.get(timeout=max(.001, deadline-time.monotonic()))
            except queue.Empty as error:
                raise ConnectionLost(f'{op} response timeout; result uncertain, not retried') from error
            if isinstance(message, BaseException):
                raise message
            if type(message.get('ok')) is not bool:
                raise ProtocolFault('Invalid RPC confirmation')
            if not message['ok']:
                # Do not echo arbitrary peer output (prepare carries sudo input).
                # Fatal startup failures are immediately followed by `closed`.
                # A received rejection remains useful evidence after cleanup;
                # unlike a success, it can never authorize a motion operation.
                raise RemoteRejected(rejection_message(op, message))
            if self.closed.is_set():
                raise ConnectionLost('Remote session closed before request confirmation')
            if 'result' not in message:
                raise ProtocolFault('Missing RPC result')
            return message['result']
        finally:
            with self.lock:
                self.pending.pop(identifier, None)

    def detach(self):
        """Close only the local SSH attachment, not the persistent robot session."""
        self._fail(ConnectionLost('SSH attachment detached'))
        try:
            if self.process.poll() is None:
                self.process.terminate()
        except OSError:
            pass
        try:
            self.process.stdin.close()
        except (OSError, ValueError):
            pass
        # Reap only this local transport. Keep cleanup off the heartbeat/close
        # deadline and retain the Popen object until its child has really exited.
        process, reader_done = self.process, self.reader_done
        def reap():
            try:
                process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                    process.wait(timeout=1.)
                except (OSError, subprocess.TimeoutExpired):
                    return
            if reader_done.wait(1.):
                try:
                    process.stdout.close()
                except (OSError, ValueError, AttributeError):
                    pass
        threading.Thread(target=reap, daemon=True).start()


def _transport_factory(*, robot_host, autofill, session_id, secret, create, timeout):
    from robot_console import ssh_command
    from robot_ssh_autofill import auto_ssh_options
    command = ssh_command('api', robot_host)
    command[-1] = ('source /home/wheeltec/nomad_live_control/campus/setup.bash >/dev/null && '
        'exec python3 -u /home/wheeltec/nomad_live_control/robot_session_server.py '
        '--proxy --session-id ' + session_id + (' --create' if create else ''))
    options = {}
    if autofill:
        command = [command[0], '-o', 'PreferredAuthentications=password',
            '-o', 'PasswordAuthentication=yes', '-o', 'PubkeyAuthentication=no',
            '-o', 'KbdInteractiveAuthentication=no', '-o', 'BatchMode=no',
            '-o', 'NumberOfPasswordPrompts=1', *command[1:]]
        options = auto_ssh_options()
    elif not create:
        # A background password prompt must never compete with a RUN input.
        # Key/agent authentication may reconnect; password-only manual mode
        # remains safely disconnected until this operator session is exited.
        command = [command[0], '-o', 'BatchMode=yes', *command[1:]]
    return LineTransport(command, {'secret': secret}, popen_options=options, timeout=timeout)


class ReconnectingRpc:
    """Existing-session reconnect with single-use, generation-bound RUN tickets."""
    reconnectable = True
    heartbeat_interval = .5
    heartbeat_timeout = 1.2

    def __init__(self, transport='api', robot_host='192.168.0.100', *, autofill=False,
                 operator_pause_resume=False,
                 _factory=None, _start_thread=True):
        if transport != 'api':
            raise ValueError('Reconnectable sessions require campus API transport')
        if type(operator_pause_resume) is not bool:
            raise ValueError('Operator pause/resume requires an explicit boolean')
        self.robot_host = robot_host
        self.autofill = autofill
        self.operator_pause_resume = operator_pause_resume
        self.session_id = str(uuid.uuid4())
        self._secret = secrets.token_hex(32)
        self._factory = _factory or _transport_factory
        self._lock = threading.RLock()
        self._lease_lock = threading.Lock()
        self._connect_lock = threading.Lock()
        self._transport = None
        self._connected = False
        self._epoch = 0
        self._generation = None
        self._challenge = None
        self._runtime = False
        self._pending_disarm = False
        self._revocation_serial = 0
        self._arm_in_flight = False
        self._closing = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self.fault = None
        self.diagnostics = []
        self.closed_result = None
        self._attach(create=True, timeout=90.)
        if _start_thread:
            self._thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self._thread.start()

    def _record(self, event, error=None, **fields):
        item = {'time_utc': datetime.now(timezone.utc).isoformat(), 'event': event,
                'connection_epoch': self._epoch, 'generation': self._generation, **fields}
        if error is not None:
            # Preserve category, phase and timestamp, never arbitrary remote
            # text that could reflect a sudo password or session capability.
            item.update(error_type=type(error).__name__)
        with self._lock:
            self.diagnostics.append(item)
            del self.diagnostics[:-128]

    @staticmethod
    def _link(value, session_id, generation=None):
        if (not isinstance(value, dict) or value.get('session_id') != session_id
                or type(value.get('generation')) is not int or value['generation'] < 1
                or (generation is not None and value['generation'] != generation)
                or not isinstance(value.get('challenge'), str) or len(value['challenge']) < 16
                or type(value.get('challenge_deadline_steady_ns')) is not int):
            raise ProtocolFault('Invalid authenticated session link state')
        return value

    def _attach(self, *, create, timeout):
        with self._connect_lock:
            try:
                channel = self._factory(robot_host=self.robot_host, autofill=self.autofill,
                    session_id=self.session_id, secret=self._secret, create=create, timeout=timeout)
            except ProtocolFault:
                with self._lock:
                    self._queue_disarm_locked()
                raise
            try:
                ready = self._link(channel.ready, self.session_id)
                if self._generation is not None and ready['generation'] <= self._generation:
                    raise ProtocolFault('New attachment did not advance its connection generation')
                # A pending manual stop/uncertain arm MUST reach the robot
                # before the first ping can renew a resumable motion lease.
                with self._lease_lock:
                    result = self._synchronize_channel(channel,
                        timeout=min(timeout, self.heartbeat_timeout), link=ready)
                with self._lock:
                    if self._closing.is_set():
                        raise ConnectionLost('Console is closing')
                    self._transport = channel
                    self._connected = True
                    self._epoch += 1
                    self._generation = result['generation']
                    self._challenge = result['challenge']
                self._record('attached', created=create)
            except BaseException as error:
                if isinstance(error, ProtocolFault) or getattr(channel, 'protocol_error', None):
                    with self._lock:
                        self._queue_disarm_locked()
                channel.detach()
                raise

    @property
    def connected(self):
        with self._lock:
            return self._connected and not self._closing.is_set()

    def connection_status(self):
        with self._lock:
            return {'operator_connected': self._connected,
                    'session_id': self.session_id, 'connection_epoch': self._epoch,
                    'generation': self._generation, 'automatic_rearm': False,
                    'operator_pause_resume': self.operator_pause_resume,
                    'automatic_continuation_window_sec': 30 if self.operator_pause_resume else 0,
                    'pending_disarm': self._pending_disarm,
                    'recent_events': list(self.diagnostics[-5:])}

    def _lost(self, channel, error, *, arm_uncertain=False):
        with self._lock:
            # A heartbeat may already have detached this channel while its arm
            # reply was in flight. Retain the pending revoke in that race too.
            if (arm_uncertain or self._arm_in_flight or isinstance(error, ProtocolFault)
                    or getattr(channel, 'protocol_error', None)):
                self._queue_disarm_locked()
                self._wake.set()
            if channel is not self._transport:
                return
            self._transport = None
            self._connected = False
            self._epoch += 1  # Invalidate an input prompt even before reconnect.
            if (channel.closed.is_set() and channel.closed_result is not None
                    and not getattr(channel, 'protocol_error', None)):
                self.closed_result = channel.closed_result
                self.fault = 'Remote operator session ended; no automatic session recreation'
            if not self._runtime:
                self.fault = f'Operator connection lost during startup: {type(error).__name__}'
        self._record('link_lost', error, arm_confirmation_uncertain=arm_uncertain)
        channel.detach()
        self._wake.set()
        if not self._closing.is_set():
            recovery = ('30 秒内恢复且原连续授权/安全检查仍有效时可能自动继续；s/q 取消。'
                        if self.operator_pause_resume and not self._pending_disarm
                        else '恢复后须重新选择挡位并输入 RUN。')
            print('\n操作台连接中断：车端停车；相机/推理继续等待，正在重连同一会话。' + recovery
                  + f' [{type(error).__name__}]',
                  file=sys.stderr, flush=True)

    def _heartbeat_once(self):
        # Serialize only short challenge operations, not status/start/service
        # calls. A deliberate disarm consumes its generation and obtains a new
        # unarmed generation without throwing away the SSH connection.
        with self._lease_lock:
            return self._heartbeat_locked()

    def _queue_disarm_locked(self):
        self._pending_disarm = True
        self._revocation_serial += 1

    def _synchronize_channel(self, channel, timeout=1., link=None):
        # Runs under _lease_lock. Stop intent can still be recorded immediately
        # under _lock while SSH is in flight; never erase a newer stop request.
        for _ in range(3):
            with self._lock:
                if self._closing.is_set():
                    raise ConnectionLost('Console is closing; no lease renewed')
                serial, pending = self._revocation_serial, self._pending_disarm
            if pending:
                stopped = channel.call('disarm', timeout=timeout)
                if not isinstance(stopped, dict) or stopped.get('success') is not True:
                    raise ConnectionLost('Robot did not confirm the pending stop')
                link = None
            if link is None:
                link = self._link(channel.call('link_sync', timeout=timeout), self.session_id)
            with self._lock:
                if self._closing.is_set():
                    raise ConnectionLost('Console is closing; no lease renewed')
                if serial != self._revocation_serial:
                    link = None
                    continue
            result = channel.call('ping', timeout=timeout,
                                  generation=link['generation'], challenge=link['challenge'])
            result = self._link(result, self.session_id, link['generation'])
            if result.get('connected') is not True:
                raise ProtocolFault('Robot has not accepted the operator lease')
            with self._lock:
                if self._closing.is_set():
                    raise ConnectionLost('Console closed during link confirmation')
                if serial != self._revocation_serial:
                    link = None
                    continue
                if pending:
                    self._pending_disarm = False
                return result
        raise ConnectionLost('Stop intent changed during recovery; leave link unconfirmed')

    def _heartbeat_locked(self):
        with self._lock:
            channel, generation, challenge = self._transport, self._generation, self._challenge
        if channel is None:
            return False
        try:
            if channel.failure is not None:
                raise channel.failure
            resynchronized = False
            try:
                result = self._synchronize_channel(channel, timeout=self.heartbeat_timeout,
                    link={'generation': generation, 'challenge': challenge})
                resynchronized = result['generation'] != generation
            except RemoteRejected:
                # An expired generation may be recovered on the same live SSH
                # attachment, but the old human RUN confirmation is discarded.
                result = self._synchronize_channel(channel)
                resynchronized = True
            if result.get('connected') is not True:
                raise ProtocolFault('Robot operator lease has been revoked')
            with self._lock:
                if self._transport is channel:
                    self._challenge = result['challenge']
                    self._generation = result['generation']
                    if resynchronized:
                        self._epoch += 1
            if resynchronized:
                self._record('authority_resynchronized',
                             operator_pause_resume=self.operator_pause_resume)
            return True
        except Exception as error:
            self._lost(channel, error)
            return False

    def _reconnect_once(self):
        with self._lock:
            if not self._runtime or self._connected or self._closing.is_set() or self.fault:
                return False
        try:
            self._attach(create=False, timeout=10.)
            if self.operator_pause_resume:
                print('\n操作台已恢复连接；仅未取消且未过期的连续任务，在新鲜预测/停稳检查通过后可自动继续。'
                      '硬故障、手停或过期仍需新 RUN；旧解锁提示已作废。', flush=True)
            else:
                print('\n操作台已恢复连接；车辆保持未解锁。旧 RUN 确认已作废，请重新选择 t/c 后确认。', flush=True)
            return True
        except Exception as error:
            self._record('reconnect_failed', error)
            return False

    def _heartbeat_loop(self):
        while not self._closing.is_set():
            self._wake.wait(self.heartbeat_interval)
            self._wake.clear()
            if self._closing.is_set():
                break
            if not self._heartbeat_once():
                self._reconnect_once()

    def start_heartbeat(self):
        # The heartbeat already protects startup; only successful start enables
        # reconnect. Calling this method cannot by itself make startup replayable.
        if not self._runtime:
            raise RuntimeError('Cannot enable runtime reconnect before start succeeds')

    def call(self, op, timeout=30., **fields):
        arm_started = False
        with self._lock:
            if op == 'disarm':
                self._queue_disarm_locked()
            channel = self._transport
            epoch, generation = self._epoch, self._generation
            if self._closing.is_set() or channel is None or not self._connected:
                raise ConnectionLost('操作台尚未恢复连接；未发送请求，请等待后重新操作')
            if op == 'start':
                if (fields.get('operator_pause_resume', False) is not self.operator_pause_resume
                        or (self.operator_pause_resume and fields.get('network_pause_resume') is not True)):
                    raise ValueError('Start must match the explicitly configured operator/network recovery policy')
            if op == 'arm':
                ticket_epoch = fields.pop('_connection_epoch', None)
                if (ticket_epoch != epoch or fields.get('generation') != generation
                        or not isinstance(fields.get('ticket'), str)):
                    raise ArmCancelled('旧 RUN 确认已失效，未发送解锁；请重新选择 t/c')
                if self._pending_disarm or self._arm_in_flight:
                    raise ArmCancelled('停止请求或解锁请求尚未完成，未发送新的解锁')
                self._arm_in_flight = arm_started = True
        try:
            if op == 'disarm':
                with self._lease_lock:
                    with self._lock:
                        if channel is not self._transport:
                            raise ConnectionLost('Attachment changed before disarm')
                        self._epoch += 1
                    link = self._synchronize_channel(channel, timeout=min(timeout, 1.))
                    with self._lock:
                        if channel is not self._transport:
                            raise ConnectionLost('Attachment changed before disarm confirmation')
                        self._generation = link['generation']
                        self._challenge = link['challenge']
                    self._record('disarmed_requires_new_run')
                    return {'success': True, 'message': 'Disarmed; a new explicit RUN is required'}
            result = channel.call(op, timeout=min(timeout, 1.2) if op == 'arm' else timeout,
                                  **fields)
            with self._lock:
                if channel is not self._transport or epoch != self._epoch:
                    raise ConnectionLost('Connection changed before request confirmation')
                if op == 'start':
                    if (not isinstance(result, dict)
                            or result.get('operator_pause_resume', False) is not self.operator_pause_resume):
                        raise ProtocolFault('Robot did not confirm the requested operator recovery policy')
                    self._runtime = True
                elif op == 'arm_prepare':
                    if (not isinstance(result, dict) or result.get('generation') != generation
                            or not isinstance(result.get('ticket'), str)):
                        raise ProtocolFault('Invalid single-use arm ticket')
                    result = {**result, '_connection_epoch': epoch}
            return result
        except RemoteRejected:
            # This is a confirmed refusal, never a reason to replay an operation.
            raise
        except Exception as error:
            self._lost(channel, error, arm_uncertain=(op == 'arm'))
            if op == 'arm':
                raise ArmCancelled('解锁回执不确定，本次确认已撤销；不重试。请确认车辆停止并等待重连') from error
            raise
        finally:
            if arm_started:
                with self._lock:
                    self._arm_in_flight = False

    def close(self, timeout=40.):
        """Explicit exit stops the owned group; reconnect never creates a group."""
        if self.closed_result is not None:
            return self.closed_result
        self._closing.set()
        self._wake.set()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                channel = self._transport
                self._transport = None
                self._connected = False
                self._epoch += 1
            if channel is None:
                remaining = deadline-time.monotonic()
                if not self._connect_lock.acquire(timeout=max(0., remaining)):
                    break
                try:
                    channel = self._factory(robot_host=self.robot_host, autofill=self.autofill,
                        session_id=self.session_id, secret=self._secret, create=False,
                        timeout=max(.01, min(10., deadline-time.monotonic())))
                except Exception as error:
                    self._record('close_reconnect_failed', error)
                finally:
                    self._connect_lock.release()
            if channel is not None:
                try:
                    if not channel.closed.is_set():
                        channel.call('stop', timeout=max(.001, deadline-time.monotonic()))
                except Exception as error:
                    self._record('close_stop_reply', error)
                channel.reader_done.wait(max(0., deadline-time.monotonic()))
                if (channel.closed.is_set() and channel.closed_result is not None
                        and not getattr(channel, 'protocol_error', None)):
                    self.closed_result = channel.closed_result
                    channel.detach()
                    return self.closed_result
                channel.detach()
            if time.monotonic() < deadline:
                self._wake.wait(min(.5, deadline-time.monotonic()))
                self._wake.clear()
        raise RuntimeError('Remote cleanup uncertain: 无法确认远端清理。车端短期运动授权最长2秒到期归零；'
                           '会话最多保留120秒后尝试清理。请现场确认车辆停止及校时服务，必要时使用物理急停。')
