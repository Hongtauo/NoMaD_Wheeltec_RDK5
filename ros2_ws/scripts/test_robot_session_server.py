"""Fake-session authority tests and Linux Unix sockets; no ROS/SSH/hardware."""
import io
import json
import os
from pathlib import Path
import socket
import stat
import tempfile
import threading
import time
from types import SimpleNamespace
import uuid

import pytest

import robot_session_server as server


SESSION = '343df852-d527-422c-a80f-909e0d97d2f0'
SECRET = 'a' * 64


class FakeClock:
    def __init__(self):
        self.now = 10_000_000_000
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            return self.now

    def advance(self, seconds):
        with self.lock:
            self.now += int(seconds*1e9)


class FakeSession:
    def __init__(self):
        self.running = False
        self.operator_pause_resume = False
        self.ros = None
        self.events = []
        self.close_calls = 0
        self.arm_calls = []
        self.start_calls = 0
        self.arm_error = None
        self.tick_error = None
        self.prepare_started = threading.Event()
        self.prepare_release = threading.Event()
        self.block_prepare = False
        self.prepare_active = False
        self.prepare_error = None
        self.ticked_during_prepare = False

    def enable_reconnect(self, session_id):
        self.events.append(('enable', session_id))

    def operator_link_update(self, generation, deadline, connected):
        self.events.append(('update', generation, deadline, connected))

    def operator_link_disarm(self, reason):
        self.events.append(('revoke', reason))

    def operator_link_pause(self, reason):
        self.events.append(('pause', reason))

    def arm_guarded(self, request, generation, deadline, nonce):
        self.arm_calls.append((dict(request), generation, deadline, nonce))
        if self.arm_error:
            raise self.arm_error
        return {'success': True, 'message': 'fake arm acknowledgement, no hardware'}

    def tick(self):
        if self.prepare_active:
            self.ticked_during_prepare = True
        if self.tick_error:
            raise self.tick_error

    def dispatch(self, request):
        operation = request['op']
        if operation == 'prepare':
            self.prepare_active = True
            self.prepare_started.set()
            self.events.append(('prepare_begin',))
            try:
                if self.block_prepare and not self.prepare_release.wait(5.):
                    raise RuntimeError('Fake prepare release timed out')
                if self.prepare_error:
                    raise self.prepare_error
                return {'prepared': True}
            finally:
                self.prepare_active = False
                self.events.append(('prepare_end',))
        if operation == 'start':
            self.start_calls += 1
            self.operator_pause_resume = request.get('operator_pause_resume', False)
            self.running = True
            return {'running': True, 'automatic_arm': False}
        if operation == 'status':
            return {'running': self.running, 'ros': {}, 'output': None}
        return {'operation': operation}

    def close(self):
        assert not self.prepare_active, 'NTP/session cleanup raced initialization'
        self.close_calls += 1
        self.events.append(('close_restore_clock',))
        self.running = False
        return {'stopped': True, 'errors': [], 'clock_restore': {'success': True},
                'hardware_stop_guaranteed': False}


def make_supervisor(running=False):
    session, clock = FakeSession(), FakeClock()
    session.running = running
    supervisor = server.SessionSupervisor(session, SESSION, clock=clock)
    supervisor.attach()
    return supervisor, session, clock


def call(supervisor, operation, **fields):
    return supervisor.dispatch({'id': supervisor.last_id+1, 'op': operation, **fields})


def ping(supervisor):
    link = supervisor.link_state()
    return call(supervisor, 'ping', generation=link['generation'], challenge=link['challenge'])


def arm(supervisor, ticket):
    return call(supervisor, 'arm', ticket=ticket['ticket'], generation=ticket['generation'],
                mode=1, duration_sec=0., confirmed=True)


def test_attach_has_new_generation_but_never_motion_authority_or_arm():
    supervisor, session, _ = make_supervisor()
    assert supervisor.generation == 1 and supervisor.connected is False
    assert not session.arm_calls
    assert not [row for row in session.events if row[0] == 'update']
    with pytest.raises(RuntimeError, match='Another operator'):
        supervisor.attach()
    assert supervisor.link_state()['session_id'] == SESSION


def test_challenge_single_use_and_expiry_anchored_to_issuance():
    supervisor, session, clock = make_supervisor()
    old = supervisor.link_state()
    clock.advance(.75)
    result = ping(supervisor)
    assert result['connected'] is True
    assert result['deadline_steady_ns'] == old['challenge_deadline_steady_ns']
    assert result['deadline_steady_ns']-clock() == 1_250_000_000
    assert result['challenge'] != old['challenge']
    with pytest.raises(RuntimeError, match='challenge'):
        call(supervisor, 'ping', challenge=old['challenge'], generation=old['generation'])
    assert len([row for row in session.events if row[0] == 'update']) == 1


def test_two_second_loss_revokes_and_old_buffered_ping_cannot_recover():
    supervisor, session, clock = make_supervisor(running=True)
    old = ping(supervisor)
    ticket = call(supervisor, 'arm_prepare')
    clock.advance(2)
    with pytest.raises(RuntimeError, match='challenge'):
        call(supervisor, 'ping', generation=old['generation'], challenge=old['challenge'])
    assert supervisor.generation > old['generation']
    assert supervisor.connected is False and supervisor.ticket is None
    assert session.running and session.close_calls == 0
    ping(supervisor)
    with pytest.raises(RuntimeError):
        arm(supervisor, ticket)
    assert not session.arm_calls


def test_detach_and_reattach_keep_same_session_and_require_new_ticket():
    supervisor, session, _ = make_supervisor(running=True)
    ping(supervisor)
    ticket = call(supervisor, 'arm_prepare')
    before = supervisor.generation
    supervisor.detach()
    assert session.running and session.close_calls == 0 and supervisor.ticket is None
    supervisor.attach()
    assert supervisor.generation > before
    ping(supervisor)
    with pytest.raises(RuntimeError):
        arm(supervisor, ticket)
    fresh = call(supervisor, 'arm_prepare')
    assert arm(supervisor, fresh)['success']
    assert len(session.arm_calls) == 1 and session.start_calls == 0


def test_ticket_is_consumed_once_and_manual_stop_clears_future_arm():
    supervisor, session, _ = make_supervisor(running=True)
    ping(supervisor)
    ticket = call(supervisor, 'arm_prepare')
    assert arm(supervisor, ticket)['success']
    with pytest.raises(RuntimeError, match='ticket'):
        arm(supervisor, ticket)
    ticket = call(supervisor, 'arm_prepare')
    call(supervisor, 'disarm')
    ping(supervisor)
    with pytest.raises(RuntimeError):
        arm(supervisor, ticket)
    assert len(session.arm_calls) == 1


def test_uncertain_arm_burns_generation_without_retry_or_full_shutdown():
    supervisor, session, _ = make_supervisor(running=True)
    ping(supervisor)
    old = supervisor.generation
    ticket = call(supervisor, 'arm_prepare')
    session.arm_error = RuntimeError('fake missing reply')
    with pytest.raises(RuntimeError, match='fake missing'):
        arm(supervisor, ticket)
    assert supervisor.generation == old+1 and not supervisor.connected
    assert supervisor.ticket is None and session.close_calls == 0 and len(session.arm_calls) == 1


def test_ticket_lifetime_is_bounded_even_with_live_authority():
    supervisor, session, clock = make_supervisor(running=True)
    ping(supervisor)
    ticket = call(supervisor, 'arm_prepare')
    for _ in range(121):
        clock.advance(.5)
        ping(supervisor)
    with pytest.raises(RuntimeError, match='ticket'):
        arm(supervisor, ticket)
    assert not session.arm_calls


def test_opt_in_detach_reattach_and_ping_only_pause_never_replay_arm_or_start():
    supervisor, session, _ = make_supervisor(running=True)
    session.operator_pause_resume = True
    ping(supervisor)
    assert arm(supervisor, call(supervisor, 'arm_prepare'))['success']
    previous_generation = supervisor.generation
    session.events.clear()
    supervisor.detach()
    supervisor.attach()
    ping(supervisor)
    assert supervisor.generation > previous_generation and supervisor.connected
    assert ('pause', 'operator_connection_lost') in session.events
    assert ('pause', 'operator_attached') in session.events
    assert not [row for row in session.events if row[0] == 'revoke']
    assert len(session.arm_calls) == 1 and session.start_calls == 0 and session.close_calls == 0


def test_opt_in_authority_expiry_pauses_then_accepts_new_challenge_without_arm():
    supervisor, session, clock = make_supervisor(running=True)
    session.operator_pause_resume = True
    ping(supervisor)
    session.events.clear()
    clock.advance(2.)
    supervisor.maintain()
    assert ('pause', 'operator_authority_expired') in session.events
    assert not supervisor.connected
    ping(supervisor)
    assert supervisor.connected and not session.arm_calls and session.close_calls == 0


@pytest.mark.parametrize('reason,kind', [
    ('operator_link_expired', 'pause'), ('operator_link_paused', 'pause'),
    ('operator_link_invalid', 'revoke'), ('operator_link_replay', 'revoke'),
    ('operator_disarmed', 'revoke'), (None, 'revoke')])
def test_opt_in_only_exact_transient_controller_reasons_preserve_intent(reason, kind):
    supervisor, session, _ = make_supervisor(running=True)
    session.operator_pause_resume = True
    ping(supervisor)
    generation = supervisor.generation
    session.ros = SimpleNamespace(snapshot=lambda: {'control': {'fresh': True, 'value': {
        'operator_link': {'generation': generation, 'requires_new_generation': True,
                          'reason': reason}}}})
    session.events.clear()
    supervisor.maintain()
    assert (kind, 'controller_authority_revoked') in session.events
    assert not supervisor.connected


@pytest.mark.parametrize('operation', ['disarm', 'stop', 'protocol', 'uncertain'])
def test_opt_in_manual_stop_protocol_and_uncertain_arm_are_hard(operation):
    supervisor, session, _ = make_supervisor(running=True)
    session.operator_pause_resume = True
    ping(supervisor)
    session.events.clear()
    if operation == 'protocol':
        supervisor.detach(transient=False, reason='operator_protocol_error')
    elif operation == 'uncertain':
        ticket = call(supervisor, 'arm_prepare')
        session.arm_error = RuntimeError('fake missing reply')
        with pytest.raises(RuntimeError, match='fake missing'):
            arm(supervisor, ticket)
    else:
        call(supervisor, operation)
    assert [row for row in session.events if row[0] == 'revoke']
    assert not [row for row in session.events if row[0] == 'pause']


def test_opt_in_stale_successful_arm_ack_hard_cancels_after_transient_pause():
    supervisor, session, clock = make_supervisor(running=True)
    session.operator_pause_resume = True
    ping(supervisor)
    ticket = call(supervisor, 'arm_prepare')
    session.events.clear()
    original = session.arm_guarded
    def delayed_ack(*args):
        result = original(*args)
        clock.advance(2.)
        return result
    session.arm_guarded = delayed_ack
    with pytest.raises(RuntimeError, match='expired during arm'):
        arm(supervisor, ticket)
    assert ('pause', 'operator_authority_expired') in session.events
    assert session.events[-1] == ('revoke', 'uncertain_arm_result')
    assert not supervisor.connected and len(session.arm_calls) == 1


def test_opt_in_does_not_pause_initialization_or_skip_stop_cleanup():
    supervisor, session, _ = make_supervisor()
    session.operator_pause_resume = True
    session.events.clear()
    supervisor.detach()
    assert session.events[-1][0] == 'revoke'
    supervisor.attach()
    supervisor.session_busy = True
    session.running = True
    supervisor.detach()
    supervisor.finish_background()
    assert session.events[-1] == ('revoke', 'initialization_link_unavailable')
    assert not [row for row in session.events if row[0] == 'pause']


def test_busy_link_loss_stays_hard_even_when_reconnected_before_worker_finishes():
    supervisor, session, _ = make_supervisor(running=True)
    session.operator_pause_resume = True
    ping(supervisor)
    supervisor.session_busy = True
    session.events.clear()
    supervisor.detach()
    supervisor.attach()
    ping(supervisor)
    assert supervisor.connected and supervisor.pending_hard_revoke is not None
    assert session.events == []  # No concurrent touch of the startup-owned ROS object.
    supervisor.finish_background()
    assert [row[0] for row in session.events] == ['revoke', 'update']
    assert session.events[1][1] == supervisor.generation
    assert supervisor.pending_hard_revoke is None
    assert not session.arm_calls


@pytest.mark.parametrize('identifier', [0, -1, True, 1.2, '1', 2**53, None])
def test_invalid_request_identity_is_connection_protocol_error(identifier):
    supervisor, session, _ = make_supervisor()
    with pytest.raises(server.ClientProtocolError):
        supervisor.dispatch({'id': identifier, 'op': 'start'})
    assert not session.start_calls and not session.close_calls


def test_completed_request_id_cannot_be_replayed():
    supervisor, session, _ = make_supervisor()
    call(supervisor, 'status')
    with pytest.raises(server.ClientProtocolError):
        supervisor.dispatch({'id': 1, 'op': 'start'})
    assert session.start_calls == 0


def test_guard_burned_current_generation_triggers_fresh_challenge():
    supervisor, session, _ = make_supervisor(running=True)
    ping(supervisor)
    before = supervisor.generation
    session.ros = SimpleNamespace(snapshot=lambda: {'control': {'fresh': True, 'value': {
        'operator_link': {'generation': before, 'requires_new_generation': True}}}})
    supervisor.maintain()
    assert supervisor.generation == before+1 and not supervisor.connected
    ping(supervisor)
    supervisor.maintain()
    assert supervisor.connected and supervisor.generation == before+1


def test_running_disconnect_retention_is_120_seconds_not_a_motion_lease():
    supervisor, session, clock = make_supervisor(running=True)
    ping(supervisor)
    supervisor.detach()
    clock.advance(119.999)
    supervisor.maintain()
    assert session.running and not supervisor.connected and not session.close_calls
    clock.advance(.001)
    supervisor.maintain()
    assert supervisor.closed_result['stopped'] and session.close_calls == 1
    with pytest.raises(RuntimeError, match='closed'):
        supervisor.attach()
    supervisor.close()
    assert session.close_calls == 1


def test_startup_disconnect_has_30_second_cleanup_and_stop_needs_no_motion_lease():
    supervisor, session, clock = make_supervisor()
    supervisor.detach()
    clock.advance(30)
    supervisor.maintain()
    assert session.close_calls == 1
    supervisor, session, _ = make_supervisor(running=True)
    assert call(supervisor, 'stop')['stopped'] and session.close_calls == 1


def test_background_startup_defers_session_touch_and_stop_waits():
    supervisor, session, _ = make_supervisor()
    ping(supervisor)
    result = supervisor.dispatch({'id': supervisor.last_id+1, 'op': 'prepare'},
                                 defer_session=lambda request: server.BACKGROUND_PENDING)
    assert result is server.BACKGROUND_PENDING and supervisor.session_busy
    previous_updates = len([row for row in session.events if row[0] == 'update'])
    ping(supervisor)
    assert len([row for row in session.events if row[0] == 'update']) == previous_updates
    with pytest.raises(RuntimeError, match='initialization'):
        supervisor.close()
    assert call(supervisor, 'stop') is server.STOP_PENDING
    assert session.close_calls == 0
    supervisor.finish_background()
    supervisor.maintain()
    assert session.close_calls == 1


@pytest.mark.parametrize('raw', [b'{}\n', b'{"secret":"wrong"}\n',
    b'{"secret":"' + b'a'*64 + b'","secret":"' + b'a'*64 + b'"}\n',
    b'{"secret":"' + b'a'*64 + b'","other":1}\n', b'x'*8193 + b'\n'])
def test_secret_handshake_rejects_bad_input_without_reflecting_it(raw):
    with pytest.raises(ValueError) as caught:
        server.read_secret(io.BytesIO(raw))
    assert SECRET not in str(caught.value)


def test_secret_has_exact_lowercase_hex_shape_and_never_enters_ready():
    assert server.read_secret(io.BytesIO(server.encode_line({'secret': SECRET}))) == SECRET
    supervisor, _, _ = make_supervisor()
    assert SECRET not in json.dumps(supervisor.link_state())
    for value in ('B'*64, 'é'*64, 'a'*63, None):
        with pytest.raises(ValueError):
            server.secret_value(value)


LINUX = hasattr(socket, 'SO_PEERCRED') and hasattr(os, 'getuid')


def until(predicate, seconds=2.):
    deadline = time.monotonic()+seconds
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('Timed out waiting for fake-session test condition')
        time.sleep(.01)


class UnixClient:
    def __init__(self, path, secret=SECRET):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(2.)
        self.socket.connect(str(path))
        self.socket.sendall(server.encode_line({'secret': secret}))
        self.reader = self.socket.makefile('rb')
        self.next_id = 0
        self.ready = self.receive()

    def receive(self):
        raw = self.reader.readline(server.MAX_OUTPUT+1)
        return json.loads(raw) if raw else None

    def send(self, operation, **fields):
        self.next_id += 1
        self.socket.sendall(server.encode_line({'id': self.next_id, 'op': operation, **fields}))
        return self.next_id

    def request(self, operation, **fields):
        identifier = self.send(operation, **fields)
        reply = self.receive()
        assert reply['id'] == identifier, reply
        return reply

    def ping(self, link=None):
        link = self.ready if link is None else link
        result = self.request('ping', generation=link['generation'], challenge=link['challenge'])
        assert result['ok'], result
        return result['result']

    def close(self):
        self.reader.close()
        self.socket.close()


@pytest.fixture
def unix_runtime():
    if not LINUX:
        pytest.skip('Actual owner-checked Unix sockets require Linux')
    with tempfile.TemporaryDirectory(prefix='rs-') as temporary:
        root = Path(temporary)/'s'
        session_id = str(uuid.uuid4())
        directory = server.session_directory(session_id, create=True, root=root)
        fake, clock = FakeSession(), FakeClock()
        result = {}
        def run():
            try:
                result['returncode'] = server.serve(session_id, SECRET, root=root,
                    session_factory=lambda: fake, clock=clock)
            except Exception as error:
                result['exception'] = error
        worker = threading.Thread(target=run, daemon=True)
        worker.start()
        until(lambda: (directory/'control.sock').exists() or not worker.is_alive())
        assert worker.is_alive(), result
        value = SimpleNamespace(root=root, directory=directory, path=directory/'control.sock',
            session_id=session_id, fake=fake, clock=clock, worker=worker, result=result)
        try:
            yield value
        finally:
            fake.prepare_release.set()
            if worker.is_alive():
                clock.advance(3.)
                time.sleep(.1)
                clock.advance(121.)
                worker.join(3.)
            assert not worker.is_alive(), 'Fake supervisor did not shut down'
            assert 'exception' not in result, result


def test_unix_eof_preserves_processes_reconnect_does_not_replay_start_or_arm(unix_runtime):
    run = unix_runtime
    first = UnixClient(run.path)
    first.ping()
    started = first.request('start')
    assert started['ok'] and run.fake.start_calls == 1
    ticket = first.request('arm_prepare')['result']
    before = ticket['generation']
    first.close()
    until(lambda: any(row == ('revoke', 'operator_connection_lost') for row in run.fake.events))
    assert run.fake.running and run.fake.close_calls == 0
    second = UnixClient(run.path)
    assert second.ready['generation'] > before
    second.ping()
    rejected = second.request('arm', ticket=ticket['ticket'], generation=ticket['generation'],
                              mode=1, duration_sec=0., confirmed=True)
    assert not rejected['ok'] and run.fake.arm_calls == []
    assert run.fake.start_calls == 1
    assert second.request('stop')['result']['stopped']
    assert second.receive()['event'] == 'closed'
    second.close()
    run.worker.join(2.)
    assert run.fake.close_calls == 1


def test_unix_prepare_can_block_while_ping_flows_and_stop_waits_for_cleanup(unix_runtime):
    run = unix_runtime
    run.fake.block_prepare = True
    client = UnixClient(run.path)
    link = client.ping()
    prepare_id = client.send('prepare', sudo_password='not-a-real-password')
    assert run.fake.prepare_started.wait(1.)
    for _ in range(5):
        run.clock.advance(.5)
        link = client.ping(link)
    assert not run.fake.ticked_during_prepare
    stop_id = client.send('stop')
    time.sleep(.1)
    assert run.fake.close_calls == 0 and run.worker.is_alive()
    run.fake.prepare_release.set()
    replies = [client.receive(), client.receive(), client.receive()]
    assert any(row.get('id') == prepare_id and row['ok'] for row in replies)
    assert any(row.get('id') == stop_id and row['result']['stopped'] for row in replies)
    assert replies[-1]['event'] == 'closed'
    client.close()
    run.worker.join(2.)
    names = [row[0] for row in run.fake.events]
    assert names.index('prepare_end') < names.index('close_restore_clock')
    for path in run.directory.iterdir():
        if path.is_file():
            assert 'not-a-real-password' not in path.read_text()
            assert SECRET not in path.read_text()


def test_unix_background_link_publish_failure_still_closes_and_records_cleanup(unix_runtime):
    run = unix_runtime
    run.fake.block_prepare = True
    client = UnixClient(run.path)
    client.ping()
    client.send('prepare', sudo_password='never-record-this-test-password')
    assert run.fake.prepare_started.wait(1.)

    def failed_publish(*_args):
        raise RuntimeError('Injected ROS publisher failure')

    run.fake.operator_link_update = failed_publish
    run.fake.prepare_release.set()
    terminal = client.receive()
    assert terminal['event'] == 'closed'
    assert terminal['result']['stopped'] is True
    assert 'Background finalization failed: RuntimeError' in terminal['result']['errors']
    client.close()
    run.worker.join(2.)
    assert not run.worker.is_alive() and run.fake.close_calls == 1
    # The original main-loop exception still terminates the daemon, but only
    # after cleanup and the durable terminal result. The fixture checks any
    # other, unanticipated exception in its own teardown.
    assert isinstance(run.result.pop('exception'), RuntimeError)
    assert not run.path.exists()
    recorded = (run.directory/'closed.json').read_text()
    assert 'Background finalization failed: RuntimeError' in recorded
    assert 'never-record-this-test-password' not in recorded and SECRET not in recorded


@pytest.mark.parametrize('known', [True, False])
def test_unix_initialization_failure_is_safe_durable_and_still_closes(unix_runtime, known):
    from robot_clock import RobotClockError
    run = unix_runtime
    run.fake.prepare_error = (RobotClockError(
        'Stop existing ROS/hardware/control processes before clock changes; PIDs: 123,456')
        if known else RuntimeError('never-record-this-test-password ' + SECRET))
    client = UnixClient(run.path)
    client.ping()
    identifier = client.send('prepare', sudo_password='never-record-this-test-password')
    reply = client.receive()
    assert reply['id'] == identifier and reply['ok'] is False
    assert reply['error_code'] == ('clock_ros_processes_active' if known else 'initialization_failed')
    if known:
        assert reply['error_details']['pids'] == [123, 456]
    terminal = client.receive()
    assert terminal['event'] == 'closed' and terminal['result']['stopped'] is True
    client.close()
    run.worker.join(2.)
    assert not run.worker.is_alive() and run.fake.close_calls == 1
    record_path = run.directory/f'failure__{identifier}.json'
    recorded = json.loads(record_path.read_text())
    assert recorded['error_code'] == reply['error_code'] and recorded['operation'] == 'prepare'
    assert stat.S_IMODE(record_path.stat().st_mode) == 0o600
    for path in run.directory.iterdir():
        if path.is_file():
            value = path.read_text()
            assert 'never-record-this-test-password' not in value and SECRET not in value


def test_unix_two_seconds_burns_motion_authority_but_120_seconds_closes(unix_runtime):
    run = unix_runtime
    client = UnixClient(run.path)
    old = client.ping()
    assert client.request('start')['ok']
    run.clock.advance(2.)
    until(lambda: ('revoke', 'operator_authority_expired') in run.fake.events)
    assert run.fake.close_calls == 0 and run.fake.running
    stale = client.request('ping', generation=old['generation'], challenge=old['challenge'])
    assert not stale['ok']
    assert stale['operator_link']['generation'] > old['generation']
    run.clock.advance(120.)
    assert client.receive()['event'] == 'closed'
    client.close()
    run.worker.join(2.)
    assert run.fake.close_calls == 1


def test_unix_opt_in_eof_reconnect_retains_only_controller_intent_without_arm_replay(unix_runtime):
    run = unix_runtime
    first = UnixClient(run.path)
    first.ping()
    assert first.request('start', operator_pause_resume=True)['ok']
    ticket = first.request('arm_prepare')['result']
    assert first.request('arm', ticket=ticket['ticket'], generation=ticket['generation'],
                         mode=1, duration_sec=0., confirmed=True)['result']['success']
    run.fake.events.clear()
    first.close()
    until(lambda: ('pause', 'operator_connection_lost') in run.fake.events)
    second = UnixClient(run.path)
    second.ping()
    assert len(run.fake.arm_calls) == 1 and run.fake.start_calls == 1
    assert not [row for row in run.fake.events if row[0] == 'revoke']
    assert second.request('stop')['result']['stopped']
    assert second.receive()['event'] == 'closed'
    second.close()


@pytest.mark.parametrize('raw', [b'{"id":true,"op":"status"}\n', b'not-json\n', b'x'*8193])
def test_unix_opt_in_malformed_protocol_hard_disarms_not_transient_pause(unix_runtime, raw):
    run = unix_runtime
    client = UnixClient(run.path)
    client.ping()
    assert client.request('start', operator_pause_resume=True)['ok']
    run.fake.events.clear()
    client.socket.sendall(raw)
    assert client.receive() is None
    client.close()
    assert ('revoke', 'operator_protocol_error') in run.fake.events
    assert not [row for row in run.fake.events if row[0] == 'pause']
    another = UnixClient(run.path)
    assert another.request('stop')['ok']
    another.receive()
    another.close()


def test_unix_bad_auth_and_invalid_id_detach_without_ending_session(unix_runtime):
    run = unix_runtime
    bad = UnixClient(run.path, 'b'*64)
    assert bad.ready is None and run.fake.close_calls == 0
    bad.close()
    client = UnixClient(run.path)
    client.ping()
    assert client.request('start')['ok']
    client.socket.sendall(server.encode_line({'id': True, 'op': 'start'}))
    assert client.receive() is None
    client.close()
    assert run.fake.start_calls == 1 and run.fake.close_calls == 0
    another = UnixClient(run.path)
    assert another.request('stop')['ok']
    another.receive()
    another.close()


def test_unix_single_connection_cannot_be_taken_over(unix_runtime):
    run = unix_runtime
    first = UnixClient(run.path)
    first.ping()
    second = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    second.settimeout(1.)
    second.connect(str(run.path))
    assert second.recv(1) == b''
    second.close()
    assert first.request('status')['ok']
    first.request('stop')
    first.receive()
    first.close()


def test_unix_private_permissions_no_session_resurrection_and_secret_not_on_disk(unix_runtime):
    run = unix_runtime
    assert stat.S_IMODE(run.directory.stat().st_mode) == 0o700
    assert stat.S_IMODE(run.path.stat().st_mode) == 0o600
    assert stat.S_IMODE((run.directory/'started.json').stat().st_mode) == 0o600
    client = UnixClient(run.path)
    client.request('stop')
    client.receive()
    client.close()
    run.worker.join(2.)
    assert not run.path.exists()
    with pytest.raises(FileExistsError):
        server.session_directory(run.session_id, create=True, root=run.root)
    with pytest.raises(FileExistsError):
        server.serve(run.session_id, SECRET, root=run.root, session_factory=FakeSession)
    for path in run.directory.iterdir():
        assert SECRET not in path.read_text()
        assert stat.S_IMODE(path.stat().st_mode) == 0o600


@pytest.mark.skipif(not LINUX, reason='Unix filesystem ownership checks')
def test_private_paths_refuse_symlink_wide_permissions_and_wrong_owner(monkeypatch):
    with tempfile.TemporaryDirectory(prefix='rp-') as temporary:
        root = Path(temporary)/'root'
        root.mkdir(mode=0o700)
        link = Path(temporary)/'link'
        link.symlink_to(root, target_is_directory=True)
        with pytest.raises(RuntimeError, match='owner-only'):
            server.session_directory(SESSION, create=True, root=link)
        root.chmod(0o755)
        with pytest.raises(RuntimeError, match='owner-only'):
            server.session_directory(SESSION, create=True, root=root)
        root.chmod(0o700)
        uid = os.getuid()
        monkeypatch.setattr(server.os, 'getuid', lambda: uid+1)
        with pytest.raises(RuntimeError, match='owner-only'):
            server.session_directory(SESSION, create=True, root=root)


@pytest.mark.skipif(not LINUX, reason='SO_PEERCRED')
def test_peer_credentials_require_same_uid(monkeypatch):
    left, right = socket.socketpair()
    try:
        assert server._peer_owned(left)
        uid = os.getuid()
        monkeypatch.setattr(server.os, 'getuid', lambda: uid+1)
        assert not server._peer_owned(left)
    finally:
        left.close()
        right.close()


def test_daemon_secret_is_stdin_only_and_child_detaches(monkeypatch, tmp_path):
    class Input:
        def __init__(self):
            self.raw = None
            self.closed = False
        def write(self, raw):
            self.raw = raw
        def close(self):
            self.closed = True
    child = SimpleNamespace(stdin=Input())
    calls = []
    def popen(argv, **kwargs):
        calls.append((argv, kwargs))
        return child
    monkeypatch.setattr(server.subprocess, 'Popen', popen)
    assert server._start_server(SESSION, SECRET, tmp_path) is child
    argv, options = calls[0]
    assert SECRET not in json.dumps(argv)
    assert 'env' not in options and options['start_new_session'] is True and options['close_fds'] is True
    assert json.loads(child.stdin.raw)['secret'] == SECRET and child.stdin.closed
    assert SECRET not in (tmp_path/'supervisor.log').read_text()
