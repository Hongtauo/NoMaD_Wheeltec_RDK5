"""Fake streams and channels only; never SSH, clock changes, ROS or motion."""
import contextlib
import io
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

import campus_session_rpc as session
import robot_console as console
from test_robot_console_protocol import FakeProcess, cleanup_result


class FakeChannel:
    def __init__(self, session_id, generation):
        self.ready = self.link(session_id, generation, False)
        self.requests = []
        self.failure = None
        self.closed = threading.Event()
        self.reader_done = threading.Event()
        self.closed_result = None
        self.detached = False
        self.callback = None

    @staticmethod
    def link(session_id, generation, connected=True):
        return {'event': 'ready', 'protocol': 1, 'session_id': session_id,
                'generation': generation, 'connected': connected,
                'challenge': 'a' * 64, 'challenge_ttl_sec': 2.,
                'challenge_deadline_steady_ns': time.monotonic_ns()+2_000_000_000}

    def call(self, op, timeout=30., **fields):
        self.requests.append((op, fields))
        if self.callback is not None:
            value = self.callback(op, fields)
            if value is not None:
                return value
        if op in ('ping', 'link_sync'):
            return self.link(self.ready['session_id'], self.ready['generation'])
        if op == 'arm_prepare':
            return {'ticket': 'ticket', 'generation': self.ready['generation'],
                    'expires_steady_ns': time.monotonic_ns()+60_000_000_000}
        if op == 'stop':
            self.closed_result = cleanup_result()
            self.closed.set()
            self.reader_done.set()
            return self.closed_result
        if op == 'start':
            return {'success': True, 'operator_pause_resume': fields.get('operator_pause_resume', False)}
        return {'success': True}

    def detach(self):
        self.detached = True
        self.failure = session.ConnectionLost('detached fake attachment')
        self.reader_done.set()


class FakeFactory:
    def __init__(self):
        self.channels = []
        self.options = []
        self.error = None

    def __call__(self, **options):
        self.options.append(options)
        if self.error:
            raise self.error
        channel = FakeChannel(options['session_id'], len(self.channels)+1)
        self.channels.append(channel)
        return channel


class ReconnectTests(unittest.TestCase):
    def make_rpc(self, runtime=True, operator_pause_resume=False):
        factory = FakeFactory()
        rpc = session.ReconnectingRpc(_factory=factory, _start_thread=False,
                                      operator_pause_resume=operator_pause_resume)
        if runtime:
            options = {'network_pause_resume': True, 'operator_pause_resume': True} if operator_pause_resume else {}
            rpc.call('start', max_linear_mps=.05, **options)
        return rpc, factory

    def lose(self, rpc):
        with contextlib.redirect_stderr(io.StringIO()):
            rpc._lost(rpc._transport, TimeoutError('original ping timed out'))

    def reconnect(self, rpc):
        with contextlib.redirect_stdout(io.StringIO()):
            return rpc._reconnect_once()

    def test_reconnect_uses_same_identity_and_never_replays_start_or_motion(self):
        rpc, factory = self.make_rpc()
        original = factory.channels[0]
        self.lose(rpc)
        self.assertFalse(rpc.connected)
        self.assertIsNone(rpc.fault)
        self.assertTrue(self.reconnect(rpc))
        self.assertTrue(original.detached)
        self.assertEqual([entry['create'] for entry in factory.options], [True, False])
        self.assertEqual(len({entry['session_id'] for entry in factory.options}), 1)
        self.assertEqual(len({entry['secret'] for entry in factory.options}), 1)
        self.assertEqual([op for op, _ in factory.channels[1].requests], ['ping'])
        self.assertFalse(rpc.connection_status()['automatic_rearm'])

    def test_startup_failure_is_terminal_and_never_replayed(self):
        rpc, factory = self.make_rpc(runtime=False)
        self.lose(rpc)
        self.assertIn('during startup', rpc.fault)
        self.assertFalse(self.reconnect(rpc))
        self.assertEqual(len(factory.channels), 1)

    def test_cross_connection_prompt_cannot_arm_even_after_recovery(self):
        rpc, factory = self.make_rpc()
        ticket = rpc.call('arm_prepare')
        self.lose(rpc)
        self.assertTrue(self.reconnect(rpc))
        with self.assertRaises(session.ArmCancelled):
            rpc.call('arm', mode=1, duration_sec=0, confirmed=True,
                     ticket=ticket['ticket'], generation=ticket['generation'],
                     _connection_epoch=ticket['_connection_epoch'])
        self.assertNotIn('arm', [op for channel in factory.channels for op, _ in channel.requests])

    def test_new_explicit_confirmation_after_reconnect_can_arm_once(self):
        rpc, factory = self.make_rpc()
        self.lose(rpc)
        self.reconnect(rpc)
        ticket = rpc.call('arm_prepare')
        self.assertTrue(rpc.call('arm', mode=1, duration_sec=0., confirmed=True,
            ticket=ticket['ticket'], generation=ticket['generation'],
            _connection_epoch=ticket['_connection_epoch'])['success'])
        self.assertEqual([op for op, _ in factory.channels[1].requests], ['ping', 'arm_prepare', 'arm'])
        self.assertNotIn('_connection_epoch', factory.channels[1].requests[-1][1])

    def test_uncertain_arm_detaches_and_reconnect_disarms_without_retry(self):
        rpc, factory = self.make_rpc()
        ticket = rpc.call('arm_prepare')
        def timeout(op, fields):
            if op == 'arm':
                raise session.ConnectionLost('arm acknowledgement timeout')
        factory.channels[0].callback = timeout
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(session.ArmCancelled):
            rpc.call('arm', mode=1, duration_sec=0., confirmed=True,
                ticket=ticket['ticket'], generation=ticket['generation'],
                _connection_epoch=ticket['_connection_epoch'])
        self.assertFalse(rpc.connected)
        self.assertTrue(rpc.connection_status()['pending_disarm'])
        self.assertTrue(self.reconnect(rpc))
        self.assertEqual([op for op, _ in factory.channels[1].requests],
                         ['disarm', 'link_sync', 'ping'])
        self.assertEqual(sum(op == 'arm' for channel in factory.channels for op, _ in channel.requests), 1)

    def test_delayed_success_after_link_loss_is_not_accepted(self):
        rpc, factory = self.make_rpc()
        ticket = rpc.call('arm_prepare')
        def delayed_success(op, fields):
            if op == 'arm':
                self.lose(rpc)
                return {'success': True}
        factory.channels[0].callback = delayed_success
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(session.ArmCancelled):
            rpc.call('arm', mode=1, duration_sec=0., confirmed=True,
                ticket=ticket['ticket'], generation=ticket['generation'],
                _connection_epoch=ticket['_connection_epoch'])
        self.assertFalse(rpc.connected)
        self.assertTrue(rpc.connection_status()['pending_disarm'])

    def test_terminal_cleanup_is_retained_and_never_recreates_remote_session(self):
        rpc, factory = self.make_rpc()
        channel = factory.channels[0]
        channel.closed_result = cleanup_result()
        channel.closed.set()
        channel.reader_done.set()
        channel.failure = session.ConnectionLost('closed by supervisor')
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertFalse(rpc._heartbeat_once())
        self.assertFalse(self.reconnect(rpc))
        self.assertEqual(rpc.close(timeout=.1), cleanup_result())
        self.assertEqual(len(factory.channels), 1)

    def test_stop_while_disconnected_is_queued_as_revocation_only(self):
        rpc, factory = self.make_rpc()
        self.lose(rpc)
        with self.assertRaises(session.ConnectionLost):
            rpc.call('disarm')
        self.reconnect(rpc)
        self.assertEqual([op for op, _ in factory.channels[1].requests],
                         ['disarm', 'link_sync', 'ping'])

    def test_optional_recovery_never_sends_an_automatic_arm(self):
        rpc, factory = self.make_rpc(operator_pause_resume=True)
        self.lose(rpc)
        self.assertTrue(self.reconnect(rpc))
        self.assertEqual([op for op,_ in factory.channels[1].requests], ['ping'])
        self.assertTrue(rpc.connection_status()['operator_pause_resume'])
        self.assertFalse(rpc.connection_status()['automatic_rearm'])
        self.assertEqual(rpc.connection_status()['automatic_continuation_window_sec'],30)

    def test_optional_pending_stop_precedes_first_renewing_ping(self):
        rpc, factory = self.make_rpc(operator_pause_resume=True)
        self.lose(rpc)
        with self.assertRaises(session.ConnectionLost):
            rpc.call('disarm')
        self.assertTrue(self.reconnect(rpc))
        self.assertEqual([op for op,_ in factory.channels[1].requests], ['disarm','link_sync','ping'])
        self.assertFalse(rpc.connection_status()['pending_disarm'])

    def test_explicit_remote_fault_cancels_before_reconnecting_ping(self):
        rpc, factory = self.make_rpc(operator_pause_resume=True)
        channel = factory.channels[0]
        channel.failure = session.ProtocolFault('Remote session fault')
        channel.protocol_error = 'remote_fault'
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertFalse(rpc._heartbeat_once())
        self.assertTrue(rpc.connection_status()['pending_disarm'])
        self.assertTrue(self.reconnect(rpc))
        self.assertEqual([op for op, _ in factory.channels[1].requests], ['disarm', 'link_sync', 'ping'])

    def test_arm_in_flight_link_loss_cancels_before_reconnect_even_before_arm_thread_returns(self):
        rpc, factory = self.make_rpc(operator_pause_resume=True)
        ticket = rpc.call('arm_prepare')
        def lost_before_ack(op, fields):
            if op == 'arm':
                self.lose(rpc)
                self.assertTrue(rpc.connection_status()['pending_disarm'])
                self.assertTrue(self.reconnect(rpc))
                return {'success':True}
        factory.channels[0].callback = lost_before_ack
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(session.ArmCancelled):
            rpc.call('arm',mode=1,duration_sec=0.,confirmed=True,ticket=ticket['ticket'],
                generation=ticket['generation'],_connection_epoch=ticket['_connection_epoch'])
        self.assertEqual([op for op,_ in factory.channels[1].requests], ['disarm','link_sync','ping'])
        self.assertTrue(rpc._pending_disarm)
        self.assertTrue(rpc._heartbeat_once())
        self.assertEqual([op for op,_ in factory.channels[1].requests][-3:], ['disarm','link_sync','ping'])

    def test_new_stop_during_recovery_is_not_erased_by_previous_disarm_ack(self):
        rpc, factory = self.make_rpc(operator_pause_resume=True)
        self.lose(rpc)
        with self.assertRaises(session.ConnectionLost): rpc.call('disarm')
        original = rpc._factory
        def interrupted_factory(**options):
            channel = original(**options)
            changed = [False]
            def callback(op, fields):
                if op == 'link_sync' and not changed[0]:
                    changed[0] = True
                    with self.assertRaises(session.ConnectionLost): rpc.call('disarm')
            channel.callback = callback
            return channel
        rpc._factory = interrupted_factory
        self.assertTrue(self.reconnect(rpc))
        self.assertEqual([op for op,_ in factory.channels[1].requests],
            ['disarm','link_sync','disarm','link_sync','ping'])

    def test_closing_during_handshake_does_not_renew_a_lease(self):
        rpc, factory = self.make_rpc(operator_pause_resume=True)
        self.lose(rpc)
        original = rpc._factory
        def closing_factory(**options):
            channel = original(**options)
            rpc._closing.set()
            return channel
        rpc._factory = closing_factory
        self.assertFalse(self.reconnect(rpc))
        self.assertEqual(factory.channels[1].requests,[])
        self.assertTrue(factory.channels[1].detached)

    def test_policy_mismatch_or_missing_network_policy_cannot_start(self):
        rpc, factory = self.make_rpc(runtime=False,operator_pause_resume=True)
        with self.assertRaises(ValueError): rpc.call('start',max_linear_mps=.05)
        self.assertEqual([op for op,_ in factory.channels[0].requests],['ping'])
        factory.channels[0].callback = lambda op,fields: ({'operator_pause_resume':False} if op=='start' else None)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(session.ConnectionLost):
            rpc.call('start',max_linear_mps=.05,operator_pause_resume=True,network_pause_resume=True)
        self.assertFalse(rpc._runtime)
        self.assertFalse(self.reconnect(rpc))

    def test_protocol_failure_cancels_before_reconnect_lease(self):
        rpc, factory = self.make_rpc(operator_pause_resume=True)
        factory.channels[0].protocol_error = 'ValueError'
        self.lose(rpc)
        self.assertTrue(rpc._pending_disarm)
        self.assertTrue(self.reconnect(rpc))
        self.assertEqual([op for op,_ in factory.channels[1].requests],['disarm','link_sync','ping'])

    def test_invalid_reconnect_identity_cancels_before_later_valid_attach(self):
        rpc, factory = self.make_rpc(operator_pause_resume=True)
        self.lose(rpc)
        original = rpc._factory
        def invalid_factory(**options):
            channel = original(**options)
            channel.ready['session_id'] = 'wrong-session'
            return channel
        rpc._factory = invalid_factory
        self.assertFalse(self.reconnect(rpc))
        self.assertTrue(rpc._pending_disarm)
        rpc._factory = original
        self.assertTrue(self.reconnect(rpc))
        self.assertEqual([op for op,_ in factory.channels[-1].requests],['disarm','link_sync','ping'])

    def test_close_offline_reconnects_existing_session_and_only_stops(self):
        rpc, factory = self.make_rpc()
        self.lose(rpc)
        self.assertTrue(rpc.close(timeout=.2)['stopped'])
        self.assertEqual([op for op, _ in factory.channels[1].requests], ['stop'])
        self.assertFalse(factory.options[-1]['create'])
        self.assertEqual(rpc.close(timeout=.2), cleanup_result())
        self.assertEqual(len(factory.channels), 2)

    def test_unreachable_close_has_bounded_uncertain_result_and_no_create(self):
        rpc, factory = self.make_rpc()
        self.lose(rpc)
        factory.error = ConnectionError('no route')
        started = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, '120秒'):
            rpc.close(timeout=.02)
        self.assertLess(time.monotonic()-started, .3)
        self.assertTrue(all(not entry['create'] for entry in factory.options[1:]))

    def test_failure_diagnostics_retain_class_time_and_redact_secret(self):
        rpc, factory = self.make_rpc()
        with contextlib.redirect_stderr(io.StringIO()) as output:
            rpc._lost(rpc._transport, TimeoutError('timeout ' + rpc._secret))
        evidence = json.dumps(rpc.connection_status())
        self.assertIn('TimeoutError', evidence)
        self.assertIn('time_utc', evidence)
        self.assertNotIn(rpc._secret, evidence)
        self.assertNotIn(rpc._secret, output.getvalue())

    def test_menu_recovery_during_run_prompt_cancels_old_run_and_stays_open(self):
        rpc, factory = self.make_rpc()
        answers = iter(['c', 'RUN', 'i', 'q'])
        def answer(prompt):
            value = next(answers)
            if value == 'RUN':
                self.lose(rpc)
                self.reconnect(rpc)
            return value
        with mock.patch('builtins.input', side_effect=answer), mock.patch('builtins.print'):
            console.operator_menu(rpc, .05)
        self.assertNotIn('arm', [op for channel in factory.channels for op, _ in channel.requests])
        self.assertIn('status', [op for op, _ in factory.channels[1].requests])

    def test_menu_disconnected_cannot_prepare_or_arm_and_q_returns_for_cleanup(self):
        rpc, factory = self.make_rpc()
        self.lose(rpc)
        with mock.patch('builtins.input', side_effect=['c', 'i', 's', 'q']), mock.patch('builtins.print'):
            console.operator_menu(rpc, .05)
        self.assertEqual([op for op, _ in factory.channels[0].requests], ['ping', 'start'])
        self.assertTrue(rpc.connection_status()['pending_disarm'])


class MultiplexTests(unittest.TestCase):
    def make_channel(self):
        process = FakeProcess()
        self.addCleanup(process.exit)
        with mock.patch.object(session.subprocess, 'Popen', return_value=process):
            channel = session.LineTransport(['fake-ssh'], {'secret': 'not-a-real-secret'}, timeout=.3)
        self.addCleanup(channel.detach)
        return channel, process

    def test_concurrent_request_ids_are_in_wire_order(self):
        channel, process = self.make_channel()
        def reply(request):
            if 'id' in request:
                process.send({'id': request['id'], 'ok': True, 'result': {}})
        process.on_request = reply
        barrier = threading.Barrier(12)
        errors = []
        def invoke():
            try:
                barrier.wait(timeout=1.)
                channel.call('status', timeout=1.)
            except Exception as error:
                errors.append(error)
        threads = [threading.Thread(target=invoke) for _ in range(12)]
        for worker in threads:
            worker.start()
        for worker in threads:
            worker.join(2.)
        self.assertEqual(errors, [])
        self.assertEqual([request['id'] for request in process.requests if 'id' in request],
                         list(range(1, 13)))

    def test_oversize_reply_rejected_without_echoing_payload(self):
        channel, process = self.make_channel()
        process.stdout.lines.put(json.dumps({'private': 'x'*65536})+'\n')
        self.assertTrue(channel.reader_done.wait(.5))
        self.assertEqual(channel.protocol_error, 'ValueError')
        self.assertNotIn('xxx', str(channel.failure))

    def test_explicit_fault_is_not_a_transient_transport_loss(self):
        channel, process = self.make_channel()
        process.exit({'event': 'fault', 'error': 'private payload'})
        self.assertTrue(channel.reader_done.wait(.5))
        self.assertIsInstance(channel.failure, session.ProtocolFault)
        self.assertEqual(channel.protocol_error, 'remote_fault')
        self.assertNotIn('private', str(channel.failure))

    def test_arbitrary_rejection_body_is_not_echoed(self):
        channel, process = self.make_channel()
        process.on_request = lambda request: process.send(
            {'id': request['id'], 'ok': False, 'error': 'private password and capability'})
        with self.assertRaises(session.RemoteRejected) as raised:
            channel.call('prepare', sudo_password='private', timeout=.2)
        self.assertNotIn('private', str(raised.exception))

    def test_safe_rejection_survives_immediate_terminal_cleanup(self):
        channel, process = self.make_channel()
        def reply(request):
            process.send({'id': request['id'], 'ok': False, 'error': 'private',
                'error_code': 'clock_ros_processes_active', 'error_details': {'pids': [123]}})
            process.send({'event': 'closed', 'result': cleanup_result()})
            self.assertTrue(channel.closed.wait(.5))
        process.on_request = reply
        with self.assertRaises(session.RemoteRejected) as raised:
            channel.call('prepare', sudo_password='private', timeout=1.)
        self.assertIn('clock_ros_processes_active', str(raised.exception))
        self.assertIn('PIDs: 123', str(raised.exception))
        self.assertNotIn('private', str(raised.exception))

    def test_duplicate_terminal_result_marks_protocol_invalid(self):
        channel, process = self.make_channel()
        closed = {'event': 'closed', 'result': cleanup_result()}
        process.exit(closed, closed)
        self.assertTrue(channel.reader_done.wait(.5))
        self.assertEqual(channel.protocol_error, 'ValueError')

    def test_heartbeat_does_not_wait_for_a_long_service_reply(self):
        process = FakeProcess()
        self.addCleanup(process.exit)
        status_received = threading.Event()
        def respond(request):
            if request.get('op') == 'status':
                status_received.set()
            elif request.get('op') == 'ping':
                process.send({'id': request['id'], 'ok': True, 'result': {'alive': True}})
        process.on_request = respond
        with mock.patch.object(session.subprocess, 'Popen', return_value=process):
            channel = session.LineTransport(['fake-ssh'], {'secret': 'not-a-real-secret'}, timeout=.3)
        outcomes = []
        def request_status():
            outcomes.append(channel.call('status', timeout=1.))
        worker = threading.Thread(target=request_status)
        worker.start()
        self.assertTrue(status_received.wait(.3))
        self.assertEqual(channel.call('ping', timeout=.1), {'alive': True})
        status = next(row for row in process.requests if row.get('op') == 'status')
        process.send({'id': status['id'], 'ok': True, 'result': {'running': True}})
        worker.join(.3)
        self.assertEqual(outcomes, [{'running': True}])
        channel.detach()

    def test_factory_keeps_capability_out_of_argv_and_environment(self):
        secret = 'b' * 64
        with mock.patch.object(session, 'LineTransport') as transport, \
                mock.patch.object(console, 'auto_ssh_options'):
            session._transport_factory(robot_host='10.201.207.37', autofill=False,
                session_id='12345678-1234-1234-1234-123456789abc', secret=secret,
                create=True, timeout=1.)
        command, hello = transport.call_args.args
        self.assertNotIn(secret, str(command))
        self.assertNotIn(secret, str(transport.call_args.kwargs))
        self.assertEqual(hello, {'secret': secret})
        self.assertIn('HostKeyAlias=192.168.0.100', command)
        self.assertIn('StrictHostKeyChecking=yes', command)
        self.assertIn('--create', command[-1])


class StubOperatorSession:
    """No ROS/hardware imports: a real protocol server owns only these fields."""
    def __init__(self):
        self.running = False
        self.armed = False
        self.starts = 0
        self.arms = 0
        self.closed = False
        self.ros = None
        self.operator_pause_resume = False
        self.pause_reasons = []
        self.arm_ack_started = None
        self.release_arm_ack = None
        self.prepare_started = None
        self.release_prepare = None

    def enable_reconnect(self, session_id):
        self.session_id = session_id

    def operator_link_disarm(self, reason):
        self.armed = False

    def operator_link_pause(self, reason):
        self.pause_reasons.append(reason)

    def operator_link_update(self, generation, deadline, connected):
        pass

    def arm_guarded(self, request, generation, deadline, ticket):
        self.armed = True
        self.arms += 1
        if self.arm_ack_started is not None:
            self.arm_ack_started.set()
            self.release_arm_ack.wait(1.)
        return {'success': True, 'message': 'stub_only_no_robot'}

    def tick(self):
        pass

    def dispatch(self, request):
        op = request['op']
        if op == 'prepare' and self.prepare_started is not None:
            self.prepare_started.set()
            self.release_prepare.wait(4.)
        if op == 'start':
            self.running = True
            self.operator_pause_resume = request.get('operator_pause_resume', False)
            self.starts += 1
        return {'running': self.running, 'armed': self.armed,
                'operator_pause_resume': self.operator_pause_resume,
                'starts': self.starts, 'stub_only_no_hardware': True}

    def close(self):
        self.running = self.armed = False
        self.closed = True
        return cleanup_result()


@unittest.skipUnless(sys.platform == 'linux' and hasattr(socket, 'SO_PEERCRED'),
                     'Real local supervisor/proxy protocol requires Linux Unix sockets')
class LocalSupervisorProtocolTests(unittest.TestCase):
    def make_rpc(self, *, heartbeat=False, operator_pause_resume=False):
        import robot_session_server as server
        temporary = tempfile.TemporaryDirectory(prefix='nomad-rpc-')
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        os.chmod(root, 0o700)
        stub = StubOperatorSession()
        offset = [0]
        server_threads, creations = [], []
        scripts = str(Path(__file__).resolve().parent)
        def factory(**options):
            creations.append(options['create'])
            if options['create']:
                server.session_directory(options['session_id'], create=True, root=root)
                worker = threading.Thread(target=server.serve,
                    args=(options['session_id'], options['secret']),
                    kwargs={'root': root, 'session_factory': lambda: stub,
                            'clock': lambda: time.monotonic_ns()+offset[0]}, daemon=True)
                worker.start()
                server_threads.append(worker)
            # Exercise the actual proxy and bounded stdio transport, with the
            # capability on stdin only. This explicitly bypasses production
            # identity checks because the supervisor owns a pure test stub.
            code = ('import sys; sys.path.insert(0,sys.argv[1]); '
                    'import robot_session_server as s; '
                    'secret=s.read_secret(sys.stdin.buffer); '
                    'raise SystemExit(s.proxy(sys.argv[2],secret,root=s.Path(sys.argv[3])))')
            return session.LineTransport([sys.executable, '-u', '-c', code, scripts,
                options['session_id'], str(root)], {'secret': options['secret']},
                timeout=min(options['timeout'], 3.))
        rpc = session.ReconnectingRpc(_factory=factory, _start_thread=heartbeat,
                                      operator_pause_resume=operator_pause_resume)
        def finish():
            if not stub.closed:
                try:
                    rpc.close(timeout=3.)
                except Exception:
                    pass
            for worker in server_threads:
                worker.join(3.)
        self.addCleanup(finish)
        return rpc, stub, offset, creations

    def arm(self, rpc):
        ticket = rpc.call('arm_prepare', timeout=1.)
        result = rpc.call('arm', timeout=1., mode=1, duration_sec=0., confirmed=True,
            ticket=ticket['ticket'], generation=ticket['generation'],
            _connection_epoch=ticket['_connection_epoch'])
        self.assertTrue(result['success'])
        return ticket

    def test_long_prepare_keeps_half_second_heartbeats_without_authority_expiry(self):
        rpc, stub, offset, creations = self.make_rpc(heartbeat=True)
        stub.prepare_started = threading.Event()
        stub.release_prepare = threading.Event()
        outcomes, failures = [], []
        def prepare():
            try:
                outcomes.append(rpc.call('prepare', timeout=4., sudo_password='test-only-no-sudo'))
            except Exception as error:
                failures.append(error)
        worker = threading.Thread(target=prepare)
        worker.start()
        self.assertTrue(stub.prepare_started.wait(1.))
        before = rpc.connection_status()
        initial_sequence = rpc._transport.sequence
        # Hold the real server startup worker longer than the 2 s authority TTL.
        # The supervisor event loop must still respond to each 0.5 s heartbeat.
        self.assertFalse(stub.release_prepare.wait(2.3))
        self.assertGreaterEqual(rpc._transport.sequence-initial_sequence, 4)
        self.assertEqual(rpc.connection_status()['generation'], before['generation'])
        self.assertEqual(rpc.connection_status()['connection_epoch'], before['connection_epoch'])
        self.assertTrue(rpc.connected)
        self.assertEqual(creations, [True])
        stub.release_prepare.set()
        worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(len(outcomes), 1)
        self.assertFalse(stub.armed)
        self.assertEqual(stub.starts, 0)
        self.assertTrue(rpc.close(timeout=3.)['stopped'])

    def test_real_proxy_detach_resume_requires_new_run_and_never_restarts(self):
        rpc, stub, offset, creations = self.make_rpc()
        rpc.call('prepare', sudo_password='test-only-no-sudo', timeout=2.)
        rpc.call('start', timeout=2., max_linear_mps=.05)
        self.arm(rpc)
        before = rpc.connection_status()
        rpc.call('disarm', timeout=2.)
        self.assertFalse(stub.armed)
        self.assertGreater(rpc.connection_status()['generation'], before['generation'])
        self.assertTrue(rpc._heartbeat_once())
        old_ticket = rpc.call('arm_prepare', timeout=1.)
        self.arm(rpc)
        with contextlib.redirect_stderr(io.StringIO()):
            rpc._lost(rpc._transport, ConnectionResetError('test detach'))
        deadline = time.monotonic()+3.
        while stub.armed and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertFalse(stub.armed)
        with contextlib.redirect_stdout(io.StringIO()):
            while not rpc._reconnect_once() and time.monotonic() < deadline:
                time.sleep(.02)
        self.assertTrue(rpc.connected)
        status = rpc.call('status', timeout=1.)
        self.assertTrue(status['running'])
        self.assertFalse(status['armed'])
        self.assertEqual(status['starts'], 1)
        with self.assertRaises(session.ArmCancelled):
            rpc.call('arm', mode=1, duration_sec=0., confirmed=True,
                ticket=old_ticket['ticket'], generation=old_ticket['generation'],
                _connection_epoch=old_ticket['_connection_epoch'])
        self.arm(rpc)
        self.assertTrue(rpc.close(timeout=3.)['stopped'])
        self.assertTrue(stub.closed)
        self.assertEqual(creations.count(True), 1)
        self.assertEqual(stub.starts, 1)

    def test_opted_in_real_proxy_retains_only_existing_intent_and_manual_stop_cancels(self):
        # This stub models task intent only, never a controller or wheel output.
        rpc, stub, offset, creations = self.make_rpc(operator_pause_resume=True)
        rpc.call('start', timeout=2., max_linear_mps=.05,
                 network_pause_resume=True, operator_pause_resume=True)
        self.arm(rpc)
        with contextlib.redirect_stderr(io.StringIO()):
            rpc._lost(rpc._transport, ConnectionResetError('test transient disconnect'))
        deadline = time.monotonic() + 3.
        while not stub.pause_reasons and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(stub.pause_reasons)
        self.assertTrue(stub.armed)
        with contextlib.redirect_stdout(io.StringIO()):
            while not rpc._reconnect_once() and time.monotonic() < deadline:
                time.sleep(.02)
        self.assertTrue(rpc.connected)
        self.assertTrue(stub.armed)
        self.assertEqual(stub.arms, 1)
        self.assertEqual(stub.starts, 1)
        # Revocation sent while offline must precede any resumed authority.
        with contextlib.redirect_stderr(io.StringIO()):
            rpc._lost(rpc._transport, ConnectionResetError('test second disconnect'))
        with self.assertRaises(session.ConnectionLost):
            rpc.call('disarm')
        deadline = time.monotonic() + 3.
        with contextlib.redirect_stdout(io.StringIO()):
            while not rpc._reconnect_once() and time.monotonic() < deadline:
                time.sleep(.02)
        self.assertTrue(rpc.connected)
        self.assertFalse(stub.armed)
        self.assertEqual(stub.arms, 1)
        self.assertEqual(creations.count(True), 1)
        self.assertTrue(rpc.close(timeout=3.)['stopped'])

    def test_same_ssh_expired_generation_invalidates_prompt_without_auto_arm(self):
        rpc, stub, offset, creations = self.make_rpc()
        rpc.call('start', timeout=2., max_linear_mps=.05)
        ticket = rpc.call('arm_prepare', timeout=1.)
        before = rpc.connection_status()
        offset[0] += 2_100_000_000
        self.assertTrue(rpc._heartbeat_once())
        self.assertGreater(rpc.connection_status()['generation'], before['generation'])
        self.assertGreater(rpc.connection_status()['connection_epoch'], before['connection_epoch'])
        with self.assertRaises(session.ArmCancelled):
            rpc.call('arm', mode=1, duration_sec=0., confirmed=True,
                ticket=ticket['ticket'], generation=ticket['generation'],
                _connection_epoch=ticket['_connection_epoch'])
        self.assertFalse(stub.armed)
        self.assertEqual(stub.arms, 0)
        self.assertEqual(creations, [True])
        self.arm(rpc)
        self.assertTrue(rpc.close(timeout=3.)['stopped'])

    def test_lost_arm_ack_never_replays_arm_and_reconnect_confirms_disarm(self):
        rpc, stub, offset, creations = self.make_rpc()
        rpc.call('start', timeout=2., max_linear_mps=.05)
        ticket = rpc.call('arm_prepare', timeout=1.)
        stub.arm_ack_started = threading.Event()
        stub.release_arm_ack = threading.Event()
        failures = []
        def request_arm():
            try:
                rpc.call('arm', timeout=1., mode=1, duration_sec=0., confirmed=True,
                    ticket=ticket['ticket'], generation=ticket['generation'],
                    _connection_epoch=ticket['_connection_epoch'])
            except Exception as error:
                failures.append(error)
        with contextlib.redirect_stderr(io.StringIO()):
            worker = threading.Thread(target=request_arm)
            worker.start()
            self.assertTrue(stub.arm_ack_started.wait(1.))
            rpc._lost(rpc._transport, ConnectionResetError('drop acknowledgement transport'))
            stub.release_arm_ack.set()
            worker.join(2.)
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], session.ArmCancelled)
        deadline = time.monotonic()+3.
        with contextlib.redirect_stdout(io.StringIO()):
            while not rpc._reconnect_once() and time.monotonic() < deadline:
                time.sleep(.02)
        self.assertTrue(rpc.connected)
        self.assertFalse(stub.armed)
        self.assertEqual(stub.arms, 1)
        self.assertEqual(stub.starts, 1)
        self.assertTrue(rpc._heartbeat_once())
        self.assertTrue(rpc.close(timeout=3.)['stopped'])


if __name__ == '__main__':
    unittest.main()
