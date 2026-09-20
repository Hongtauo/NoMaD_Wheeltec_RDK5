#!/usr/bin/env python3
"""Robot-local reconnectable campus supervisor and credential-free SSH proxy.

Only --serve owns OperatorSession. Losing an authenticated proxy revokes motion
authority but does not terminate camera/model processes. Optional bounded pause
retains controller-owned continuous intent; no arm request is replayed on attach,
ping, reconnect, or startup. The secret travels over stdin/socket only.
"""
import argparse
import hmac
import json
import os
from pathlib import Path
import platform
import re
import secrets
import select
import signal
import socket
import stat
import struct
import subprocess
import sys
import threading
import time
import uuid

from operator_errors import safe_initialization_error


SESSION_ROOT = Path('/home/wheeltec/nomad_live_control/.sessions')
MAX_LINE = 8192
MAX_OUTPUT = 65536
CHALLENGE_NS = 2_000_000_000
ARM_TICKET_NS = 60_000_000_000
STARTUP_IDLE_NS = 30_000_000_000
RECONNECT_NS = 120_000_000_000
TRANSIENT_GUARD_REASONS = frozenset(('operator_link_expired', 'operator_link_paused'))
BACKGROUND_PENDING = object()
STOP_PENDING = object()


class ClientProtocolError(ValueError):
    """Malformed proxy input loses only that connection, not the ROS session."""


def canonical_session(value):
    if not isinstance(value, str) or str(uuid.UUID(value)) != value:
        raise ValueError('Session ID must be a canonical UUID')
    return value


def secret_value(value):
    if not isinstance(value, str) or re.fullmatch(r'[0-9a-f]{64}', value) is None:
        raise ValueError('Expected a private 64-hex session credential')
    return value


def decode_line(raw):
    if not isinstance(raw, bytes) or not raw.endswith(b'\n') or len(raw) > MAX_LINE:
        raise ValueError('Invalid or oversized protocol line')
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('Duplicate protocol key')
            result[key] = value
        return result
    result = json.loads(raw.decode('utf-8'), object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite protocol number')))
    if not isinstance(result, dict):
        raise ValueError('Protocol line must contain an object')
    return result


def encode_line(value):
    raw = (json.dumps(value, allow_nan=False, separators=(',', ':')) + '\n').encode('utf-8')
    if len(raw) > MAX_OUTPUT:
        raise ValueError('Protocol output is too large')
    return raw


def read_secret(stream):
    value = decode_line(stream.readline(MAX_LINE + 1))
    if set(value) != {'secret'}:
        raise ValueError('Expected only the private session credential')
    return secret_value(value['secret'])


class SessionSupervisor:
    """Testable monotonic authority state machine; no socket/ROS imports here."""
    def __init__(self, session, session_id, *, clock=time.monotonic_ns):
        self.session = session
        self.session_id = canonical_session(session_id)
        self.clock = clock
        self.generation = 0
        self.attached = False
        self.connected = False
        self.deadline = 0
        self.challenge = None
        self.challenge_deadline = 0
        self.ticket = None
        self.last_id = 0
        self.detached_at = self.clock()
        self.last_request_at = self.detached_at
        self.closed_result = None
        self.session_busy = False
        self.pending_hard_revoke = None
        self.stop_requested = False
        self.session.enable_reconnect(self.session_id)

    def _challenge(self):
        self.challenge = secrets.token_hex(32)
        self.challenge_deadline = self.clock() + CHALLENGE_NS

    def link_state(self):
        return dict(session_id=self.session_id, generation=self.generation,
            connected=self.connected, deadline_steady_ns=self.deadline,
            challenge=self.challenge, challenge_deadline_steady_ns=self.challenge_deadline,
            challenge_ttl_sec=CHALLENGE_NS/1e9)

    def _revoke(self, reason, *, transient=False):
        # The control guard burns a disconnected generation permanently. Revoke
        # OLD authority first; never publish False for the next generation.
        self.connected = False
        self.deadline = 0
        self.ticket = None
        self.challenge = None
        self.challenge_deadline = 0
        self.detached_at = self.clock()
        try:
            if self.session_busy:
                # ROS/startup state must not be touched concurrently with the
                # sole background task. Preserve the hard cancellation even if
                # SSH reconnects before that task completes.
                self.pending_hard_revoke = reason
            else:
                pause = (transient and not self.stop_requested and self.session.running
                         and getattr(self.session, 'operator_pause_resume', False) is True)
                if pause:
                    self.session.operator_link_pause(reason)
                else:
                    self.session.operator_link_disarm(reason)
        finally:
            self.generation += 1
            if self.attached:
                self._challenge()

    def attach(self):
        if self.closed_result is not None:
            raise RuntimeError('This session is closed; create a new session ID')
        if self.attached:
            raise RuntimeError('Another operator connection is active')
        self.attached = True
        self.last_id = 0
        self.last_request_at = self.clock()
        self._revoke('operator_attached', transient=True)
        return dict(event='ready', protocol=1, **self.link_state())

    def detach(self, *, transient=True, reason='operator_connection_lost'):
        if not self.attached:
            return
        self.attached = False
        self._revoke(reason, transient=transient)

    def maintain(self):
        if self.closed_result is not None:
            return
        now = self.clock()
        if self.connected and now >= self.deadline:
            self._revoke('operator_authority_expired', transient=True)
        # Existing hardware/process failures still fail the complete session.
        if not self.session_busy:
            self.session.tick()
            ros = getattr(self.session, 'ros', None)
            if ros is not None and self.connected:
                control = ros.snapshot().get('control', {})
                guard = control.get('value', {}).get('operator_link', {})
                generation = guard.get('generation')
                if (control.get('fresh') is True and guard.get('requires_new_generation') is True
                        and type(generation) is int and generation >= self.generation):
                    self._revoke('controller_authority_revoked',
                                 transient=guard.get('reason') in TRANSIENT_GUARD_REASONS)
                    if self.generation <= generation:
                        self.generation = generation+1
                        self._challenge()
        running = bool(self.session.running)
        if running:
            if not self.connected and now-self.detached_at >= RECONNECT_NS:
                self.stop_requested = True
        elif ((not self.attached and now-self.detached_at >= STARTUP_IDLE_NS)
              or (self.attached and now-self.last_request_at >= STARTUP_IDLE_NS)):
            self.stop_requested = True
        if self.stop_requested and not self.session_busy:
            self.close()

    def _require_link(self, generation=None):
        if (not self.attached or not self.connected or self.clock() >= self.deadline
                or (generation is not None and generation != self.generation)):
            raise RuntimeError('Operator authority is unavailable; wait for authenticated link recovery before this request')

    def dispatch(self, request, *, defer_session=None):
        self.maintain()
        if self.closed_result is not None:
            raise RuntimeError('Session has closed')
        if not self.attached:
            raise RuntimeError('No authenticated operator connection')
        identifier = request.get('id')
        if type(identifier) is not int or not self.last_id < identifier <= 2**53-1:
            raise ClientProtocolError('Request IDs must increase; requests are never replayed')
        self.last_id = identifier
        self.last_request_at = self.clock()
        op = request.get('op')
        if op == 'link_sync':
            if self.challenge is None or self.clock() >= self.challenge_deadline:
                self._challenge()
            return self.link_state()
        if op == 'ping':
            supplied = request.get('challenge')
            if (type(request.get('generation')) is not int
                    or request['generation'] != self.generation
                    or not isinstance(supplied, str) or re.fullmatch(r'[0-9a-f]{64}', supplied) is None
                    or self.challenge is None
                    or not hmac.compare_digest(supplied, self.challenge)
                    or self.clock() >= self.challenge_deadline):
                raise RuntimeError('Expired or mismatched operator challenge; synchronize the link again')
            # Anchor expiry to issuance, not receipt of delayed/buffered ping.
            deadline = self.challenge_deadline
            self.challenge = None
            if not self.session_busy:
                self.session.operator_link_update(self.generation, deadline, True)
            self.deadline, self.connected = deadline, True
            self._challenge()
            return self.link_state()
        if op == 'arm_prepare':
            if self.session_busy:
                raise RuntimeError('Initialization is still running; no RUN is permitted')
            self._require_link()
            if not self.session.running:
                raise RuntimeError('Start the disarmed robot session first')
            self.ticket = dict(ticket=str(uuid.uuid4()), generation=self.generation,
                               expires_steady_ns=self.clock() + ARM_TICKET_NS)
            return dict(self.ticket)
        if op == 'arm':
            ticket, self.ticket = self.ticket, None  # Every attempt consumes it.
            if self.session_busy:
                raise RuntimeError('Initialization is still running; no RUN is permitted')
            self._require_link(request.get('generation'))
            if (type(request.get('generation')) is not int or ticket is None
                    or ticket['generation'] != self.generation
                    or request.get('ticket') != ticket['ticket']
                    or self.clock() >= ticket['expires_steady_ns']):
                raise RuntimeError('RUN ticket is expired or from an old connection; confirm a new RUN')
            try:
                result = self.session.arm_guarded(request, self.generation,
                                                  self.deadline, ticket['ticket'])
            except Exception:
                self._revoke('uncertain_arm_result')
                raise
            # A blocking service must not turn a stale success into authority.
            self.maintain()
            if not self.connected or ticket['generation'] != self.generation:
                # A stale success is uncertain even under optional pause mode.
                # It must never leave resumable intent after the caller is told
                # that its arm was not confirmed.
                self._revoke('uncertain_arm_result')
                raise RuntimeError('Operator authority expired during arm; result is not permission to move')
            return result
        if op == 'disarm':
            self._revoke('operator_disarmed')
            return dict(success=True, message='Disarmed; synchronize link and confirm a new RUN')
        if op == 'stop':
            if self.session_busy:
                self.stop_requested = True
                self._revoke('operator_stop_during_initialization')
                return STOP_PENDING
            return self.close()
        if op == 'status':
            value = self.session.dispatch(request)
            return dict(value, operator_link=self.link_state())
        if op not in ('prepare', 'sample', 'set_time', 'verify', 'plan', 'start'):
            raise ClientProtocolError('Unknown operator command')
        if self.session_busy:
            raise RuntimeError('One initialization operation is already in progress')
        if defer_session is not None:
            self.session_busy = True
            return defer_session(request)
        return self.session.dispatch(request)

    def finish_background(self):
        """Called only after the sole startup task has stopped touching session."""
        self.session_busy = False
        if self.pending_hard_revoke is not None:
            self.session.operator_link_disarm(self.pending_hard_revoke)
            self.pending_hard_revoke = None
        if self.connected and self.clock() < self.deadline and not self.stop_requested:
            self.session.operator_link_update(self.generation, self.deadline, True)
        else:
            self.session.operator_link_disarm('initialization_link_unavailable')

    def close(self):
        if self.session_busy:
            raise RuntimeError('Cannot restore clocks while initialization is running')
        if self.closed_result is None:
            revoke_error = None
            try:
                self._revoke('operator_session_stopping')
            except Exception as error:
                revoke_error = 'Motion revoke failed: ' + type(error).__name__
            try:
                self.closed_result = self.session.close()
            except Exception as error:
                self.closed_result = dict(stopped=False,
                    errors=['Session cleanup failed: ' + type(error).__name__],
                    clock_restore=None, hardware_stop_guaranteed=False)
            if revoke_error:
                self.closed_result.setdefault('errors', []).append(revoke_error)
            self.attached = False
            self.challenge = None
        return self.closed_result


def _private_directory(path, *, create=False, exclusive=False):
    if create:
        path.mkdir(mode=0o700, exist_ok=not exclusive)
    info = path.lstat()
    if (path.resolve(strict=True) != path.absolute() or not stat.S_ISDIR(info.st_mode)
            or info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700):
        raise RuntimeError('Session directory must be owner-only and not a symlink')
    return path


def session_directory(session_id, *, create=False, root=SESSION_ROOT):
    canonical_session(session_id)
    root = _private_directory(Path(root), create=create)
    directory = _private_directory(root/session_id, create=create, exclusive=create)
    if len(os.fsencode(str(directory/'control.sock'))) >= 108:
        raise RuntimeError('Session socket path is too long')
    return directory


def _owned_socket(path):
    info = path.lstat()
    if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o600:
        raise RuntimeError('Expected an owner-only session socket')


def _write_private(path, value):
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
    fd = os.open(str(path), flags, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8') as stream:
        json.dump(value, stream, allow_nan=False)
        stream.write('\n')


def _peer_owned(connection):
    if not hasattr(socket, 'SO_PEERCRED'):
        raise RuntimeError('Linux peer credentials are mandatory')
    _, uid, _ = struct.unpack('3i', connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
    return uid == os.getuid()


def serve(session_id, secret, *, root=SESSION_ROOT, session_factory=None,
          clock=time.monotonic_ns):
    secret_value(secret)
    directory = session_directory(session_id, root=root)
    # O_EXCL is the single-start fence. Finished/crashed session IDs cannot be
    # silently resurrected even if their old Unix socket has disappeared.
    _write_private(directory/'started.json', dict(session_id=session_id, pid=os.getpid()))
    if session_factory is None:
        from robot_operator import OperatorSession
        session_factory = lambda: OperatorSession(transport='api')
    supervisor = SessionSupervisor(session_factory(), session_id, clock=clock)
    path = directory/'control.sock'
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    connection = None
    authenticated = False
    incoming, outgoing = bytearray(), bytearray()
    accepted_at = 0.
    stop = False
    result = 1
    background = None
    background_result = None
    background_connection = None
    stop_reply_id = None
    socket_identity = None

    def request_stop(*_args):
        nonlocal stop
        stop = True
    previous = ({sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
                if threading.current_thread() is threading.main_thread() else {})

    def disconnect(*, transient=True, reason='operator_connection_lost'):
        nonlocal connection, authenticated, incoming, outgoing
        if authenticated:
            supervisor.detach(transient=transient, reason=reason)
        if connection is not None:
            connection.close()
        connection, authenticated = None, False
        incoming, outgoing = bytearray(), bytearray()

    def queue(value):
        raw = encode_line(value)
        if len(outgoing)+len(raw) > MAX_OUTPUT:
            raise RuntimeError('Operator output backlog exceeded its bound')
        outgoing.extend(raw)

    def defer(request):
        nonlocal background, background_result, background_connection
        background_connection = connection
        identifier, operation = request['id'], request['op']
        def work():
            nonlocal background_result
            try:
                value = supervisor.session.dispatch(request)
                background_result = dict(id=identifier, ok=True, result=value, operation=operation)
            except Exception as error:
                # Even an unexpected lower-level error must not reflect the
                # sudo password, request fields or arbitrary command output.
                safe_error = safe_initialization_error(error)
                record = dict(schema=1, session_id=session_id, operation=operation,
                              request_id=identifier, recorded_time_ns=time.time_ns(), **safe_error)
                try:
                    _write_private(directory/f'failure__{identifier}.json', record)
                except OSError:
                    # Failure evidence must not prevent the original fail-closed
                    # cleanup. Never log the exception text or request here.
                    pass
                background_result = dict(id=identifier, ok=False,
                    error='Initialization failed', operation=operation, **safe_error)
        background = threading.Thread(target=work, name='campus-startup-operation', daemon=False)
        try:
            background.start()
        except Exception:
            supervisor.session_busy = False
            background = None
            raise
        return BACKGROUND_PENDING

    try:
        listener.bind(str(path))
        os.chmod(path, 0o600)
        socket_info = path.lstat()
        socket_identity = (socket_info.st_dev, socket_info.st_ino)
        listener.listen(2)
        listener.setblocking(False)
        while not stop and supervisor.closed_result is None:
            if background is not None and not background.is_alive():
                background.join()
                supervisor.finish_background()
                value, background_result, background = background_result, None, None
                if connection is background_connection and authenticated:
                    queue({key: item for key, item in value.items() if key != 'operation'} |
                          {'operator_link': supervisor.link_state()})
                if not value['ok'] and value['operation'] in ('prepare', 'set_time', 'verify', 'start'):
                    stop = True
            supervisor.maintain()
            readable = [listener] + ([connection] if connection is not None else [])
            writable = [connection] if connection is not None and outgoing else []
            ready, writes, _ = select.select(readable, writable, [], .05)
            if listener in ready:
                candidate, _ = listener.accept()
                if connection is not None or not _peer_owned(candidate):
                    candidate.close()  # Never steal an existing authenticated link.
                else:
                    connection = candidate
                    connection.setblocking(False)
                    accepted_at = time.monotonic()
            if connection is not None and not authenticated and time.monotonic()-accepted_at > 2.:
                disconnect()
            if connection is not None and connection in writes:
                try:
                    count = connection.send(outgoing)
                    del outgoing[:count]
                except (BlockingIOError, InterruptedError):
                    pass
                except (BrokenPipeError, ConnectionResetError):
                    disconnect()
            if connection is not None and connection in ready:
                try:
                    chunk = connection.recv(4096)
                except (BlockingIOError, InterruptedError):
                    chunk = None
                except ConnectionResetError:
                    chunk = b''
                if chunk == b'':
                    disconnect()
                elif chunk:
                    incoming.extend(chunk)
                    while connection is not None and b'\n' in incoming:
                        line, _, rest = incoming.partition(b'\n')
                        incoming = bytearray(rest)
                        try:
                            request = decode_line(bytes(line)+b'\n')
                            if not authenticated:
                                supplied = request.get('secret')
                                if (set(request) != {'secret'} or secret_value(supplied) != supplied
                                        or not hmac.compare_digest(supplied, secret)):
                                    raise ValueError('Authentication rejected')
                                queue(supervisor.attach())
                                authenticated = True
                            else:
                                identifier = request.get('id')
                                try:
                                    value = supervisor.dispatch(request, defer_session=defer)
                                    if value is STOP_PENDING:
                                        stop_reply_id = identifier
                                    elif value is not BACKGROUND_PENDING:
                                        queue(dict(id=identifier, ok=True, result=value,
                                                   operator_link=supervisor.link_state()))
                                except ClientProtocolError:
                                    disconnect(transient=False, reason='operator_protocol_error')
                                except Exception as error:
                                    # Never echo request values: prepare contains a sudo secret.
                                    queue(dict(id=identifier, ok=False, error=str(error),
                                               operator_link=supervisor.link_state()))
                        except (ValueError, UnicodeError, RecursionError):
                            disconnect(transient=False, reason='operator_protocol_error')
                    if len(incoming) > MAX_LINE:
                        disconnect(transient=False, reason='operator_protocol_error')
        result = 0
    finally:
        # Never race NTP restore/cleanup against prepare/set_time/start. The
        # motion gate expires independently while a bounded subprocess finishes.
        finalization_errors = []
        if background is not None:
            background.join()
            try:
                supervisor.finish_background()
            except Exception as error:
                # ROS may already have failed while applying the latest lease.
                # The worker is joined: cleanup must still stop owned children
                # before restoring time services, even if this publish fails.
                finalization_errors.append('Background finalization failed: ' + type(error).__name__)
        cleanup = supervisor.close()
        cleanup.setdefault('errors', []).extend(finalization_errors)
        if connection is not None and authenticated:
            try:
                if stop_reply_id is not None:
                    outgoing.extend(encode_line(dict(id=stop_reply_id, ok=True, result=cleanup,
                                                     operator_link=supervisor.link_state())))
                outgoing.extend(encode_line(dict(event='closed', result=cleanup)))
                connection.setblocking(True)
                connection.settimeout(1.)
                connection.sendall(outgoing)
            except OSError:
                pass
        if connection is not None:
            connection.close()
        listener.close()
        try:
            _owned_socket(path)
            socket_info = path.lstat()
            if socket_identity == (socket_info.st_dev, socket_info.st_ino):
                path.unlink()
        except FileNotFoundError:
            pass
        except RuntimeError:
            cleanup.setdefault('errors', []).append('Session socket ownership changed; path not removed')
        _write_private(directory/'closed.json', dict(session_id=session_id, result=cleanup))
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return result if not cleanup.get('errors') else 1


def _start_server(session_id, secret, directory):
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
    fd = os.open(str(directory/'supervisor.log'), flags, 0o600)
    with os.fdopen(fd, 'wb') as log:
        child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
            '--serve', '--session-id', session_id], stdin=subprocess.PIPE,
            stdout=log, stderr=log, start_new_session=True, close_fds=True)
    try:
        child.stdin.write(encode_line({'secret': secret}))
        child.stdin.close()
    except Exception:
        child.terminate()
        raise
    return child


def proxy(session_id, secret, *, create=False, root=SESSION_ROOT):
    secret_value(secret)
    directory = session_directory(session_id, create=create, root=root)
    child = _start_server(session_id, secret, directory) if create else None
    path = directory/'control.sock'
    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    deadline = time.monotonic()+10.
    while True:
        try:
            _owned_socket(path)
            connection.connect(str(path))
            if not _peer_owned(connection):
                raise RuntimeError('Supervisor belongs to a different user')
            break
        except (FileNotFoundError, ConnectionRefusedError):
            if (directory/'closed.json').exists() or time.monotonic() >= deadline or (child is not None and child.poll() is not None):
                connection.close()
                raise RuntimeError('Session supervisor unavailable; do not recreate this session ID')
            time.sleep(.05)
    try:
        connection.sendall(encode_line({'secret': secret}))
        connection.setblocking(False)
        input_fd, output_fd = sys.stdin.fileno(), sys.stdout.fileno()
        buffered = bytearray()
        while True:
            reads, writes, _ = select.select([input_fd, connection],
                [connection] if buffered else [], [], .1)
            if input_fd in reads:
                raw = os.read(input_fd, 4096)
                if not raw:
                    return 0  # Closing this socket is detach, not session stop.
                buffered.extend(raw)
                if len(buffered) > 2*MAX_LINE:
                    raise RuntimeError('Proxy input backlog exceeded its bound')
            if connection in writes:
                sent = connection.send(buffered)
                del buffered[:sent]
            if connection in reads:
                raw = connection.recv(4096)
                if not raw:
                    return 0
                while raw:
                    count = os.write(output_fd, raw)
                    raw = raw[count:]
    finally:
        connection.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--proxy', action='store_true')
    mode.add_argument('--serve', action='store_true')
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--create', action='store_true')
    args = parser.parse_args(argv)
    if args.create and not args.proxy:
        parser.error('--create is valid only for the proxy')
    try:
        session_id = canonical_session(args.session_id)
        from nomad_navigation.robot_identity import validate_api_environment, require_robot_identity
        validate_api_environment(os.environ, platform.machine())
        require_robot_identity()
        secret = read_secret(sys.stdin.buffer)
        if args.serve:
            return serve(session_id, secret)
        return proxy(session_id, secret, create=args.create)
    except Exception as error:
        # Exception messages can originate in arbitrary JSON/password input.
        print('Campus session stopped: ' + type(error).__name__, file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
