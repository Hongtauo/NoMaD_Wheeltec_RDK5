"""Mocked SSH streams only: terminal cleanup must survive errors and EOF races."""
import json
import queue
import subprocess
import threading
import time
import unittest
from unittest import mock

import robot_console as console


def cleanup_result(**changes):
    result = {
        'stopped': True, 'errors': [], 'hardware_stop_guaranteed': False,
        'clock_restore': {
            'success': True, 'phase': 'restored', 'session_id': 'test-session',
            'restored_services': [], 'pending_services': [], 'errors': [],
        },
    }
    result.update(changes)
    return result


class QueueStream:
    def __init__(self):
        self.lines = queue.Queue()

    def __iter__(self):
        while True:
            line = self.lines.get(timeout=2.)
            if line is None:
                return
            yield line

    def readline(self, _size=-1):
        line = self.lines.get(timeout=2.)
        return '' if line is None else line


class FakeInput:
    def __init__(self, process):
        self.process = process
        self.pending = ''
        self.closed = False

    def write(self, line):
        if self.closed or self.process.exited.is_set():
            raise BrokenPipeError('fake SSH already closed')
        self.pending += line

    def flush(self):
        if self.pending:
            request = json.loads(self.pending)
            self.pending = ''
            self.process.requests.append(request)
            self.process.on_request(request)

    def close(self):
        self.closed = True
        if not self.process.exited.is_set():
            self.process.exit()


class FakeProcess:
    """A process-shaped stream fixture; it never starts a system process."""
    def __init__(self):
        self.stdout = QueueStream()
        self.stdin = FakeInput(self)
        self.exited = threading.Event()
        self.requests = []
        self.terminations = 0
        self.on_request = lambda _request: None
        self.send({'event': 'ready', 'protocol': 1, 'automatic_arm': False})

    def send(self, message):
        self.stdout.lines.put(json.dumps(message) + '\n')

    def exit(self, *messages):
        if self.exited.is_set():
            return
        for message in messages:
            self.send(message)
        self.stdout.lines.put(None)
        self.exited.set()

    def poll(self):
        return 1 if self.exited.is_set() else None

    def wait(self, timeout):
        if not self.exited.wait(timeout):
            raise subprocess.TimeoutExpired('fake-ssh', timeout)
        return 1

    def terminate(self):
        self.terminations += 1
        self.exit()


class ConsoleProtocolTests(unittest.TestCase):
    def make_rpc(self):
        process = FakeProcess()
        with mock.patch.object(console.subprocess, 'Popen', return_value=process):
            rpc = console.Rpc()
        self.addCleanup(process.exit)
        return rpc, process

    def test_prepare_error_then_closed_then_eof_keeps_original_error_and_cleanup(self):
        rpc, process = self.make_rpc()
        expected = cleanup_result()

        def reject(request):
            process.exit({'id': request['id'], 'ok': False, 'error': 'Existing ROS; PIDs: 1,2'},
                         {'event': 'closed', 'result': expected})

        process.on_request = reject
        with self.assertRaisesRegex(RuntimeError, 'Existing ROS; PIDs: 1,2'):
            rpc.call('prepare', sudo_password='dummy-secret')
        self.assertEqual(rpc.close(timeout=.5), expected)
        self.assertEqual([row['op'] for row in process.requests], ['prepare'])

    def test_close_after_ssh_exit_consumes_buffered_terminal_event(self):
        rpc, process = self.make_rpc()
        expected = cleanup_result()
        process.exit({'event': 'closed', 'result': expected})
        self.assertIsNotNone(process.poll())
        self.assertEqual(rpc.close(timeout=.5), expected)
        self.assertEqual(process.requests, [])

    def test_closed_racing_stop_needs_no_stop_id_reply(self):
        rpc, process = self.make_rpc()
        expected = cleanup_result()
        process.on_request = lambda _request: process.exit({'event': 'closed', 'result': expected})
        self.assertEqual(rpc.close(timeout=.5), expected)
        self.assertEqual([row['op'] for row in process.requests], ['stop'])
        self.assertEqual(rpc.close(timeout=.5), expected)
        self.assertEqual(len(process.requests), 1)

    def test_terminal_result_not_stop_ack_is_authoritative(self):
        rpc, process = self.make_rpc()
        expected = cleanup_result(errors=['NTP restore failed'], clock_restore={'error': 'ntp.service'})

        def finish(request):
            process.exit({'id': request['id'], 'ok': True, 'result': cleanup_result()},
                         {'event': 'closed', 'result': expected})

        process.on_request = finish
        self.assertEqual(rpc.close(timeout=.5), expected)

    def test_no_closed_after_already_exited_is_uncertain(self):
        rpc, process = self.make_rpc()
        process.exit()
        with self.assertRaisesRegex(RuntimeError, 'cleanup uncertain'):
            rpc.close(timeout=.5)
        self.assertEqual(process.requests, [])

    def test_stop_ack_and_eof_without_closed_is_uncertain(self):
        rpc, process = self.make_rpc()
        process.on_request = lambda request: process.exit(
            {'id': request['id'], 'ok': True, 'result': cleanup_result()})
        with self.assertRaisesRegex(RuntimeError, 'cleanup uncertain'):
            rpc.close(timeout=.5)
        self.assertEqual([row['op'] for row in process.requests], ['stop'])

    def test_failed_closed_preserves_all_error_details(self):
        rpc, process = self.make_rpc()
        expected = cleanup_result(stopped=False, errors=['Child remains', 'NTP restoration deferred'],
                                  clock_restore={'error': 'ROS remains'})
        process.exit({'event': 'closed', 'result': expected})
        self.assertEqual(rpc.close(timeout=.5), expected)

    def test_invalid_terminal_results_are_never_success(self):
        malformed = [None, [], {}, cleanup_result(stopped='true'),
                     cleanup_result(hardware_stop_guaranteed=True), cleanup_result(clock_restore=[]),
                     cleanup_result(clock_restore={'success': 'true'}),
                     cleanup_result(clock_restore={'error': 'failure'}),
                     cleanup_result(clock_restore={'success': False, 'errors': [], 'pending_services': []})]
        for value in malformed:
            with self.subTest(value=value):
                rpc, process = self.make_rpc()
                process.exit({'event': 'closed', 'result': value})
                with self.assertRaisesRegex(RuntimeError, 'cleanup uncertain'):
                    rpc.close(timeout=.5)

    def test_invalid_json_and_nonobject_response_are_uncertain(self):
        for line in ('not-json\n', '[]\n'):
            with self.subTest(line=line):
                rpc, process = self.make_rpc()
                process.stdout.lines.put(line)
                process.exit()
                with self.assertRaisesRegex(RuntimeError, 'cleanup uncertain'):
                    rpc.close(timeout=.5)

    def test_duplicate_closed_is_invalid_protocol(self):
        rpc, process = self.make_rpc()
        message = {'event': 'closed', 'result': cleanup_result()}
        process.exit(message, message)
        with self.assertRaisesRegex(RuntimeError, 'cleanup uncertain'):
            rpc.close(timeout=.5)

    def test_closed_and_old_reply_never_confirm_arm(self):
        rpc, process = self.make_rpc()
        expected = cleanup_result()

        def finish(request):
            process.exit({'id': request['id'] - 1, 'ok': True, 'result': {'success': True}},
                         {'event': 'closed', 'result': expected})

        process.on_request = finish
        with self.assertRaisesRegex(RuntimeError, 'closed before request confirmation'):
            rpc.call('arm', mode=1, duration_sec=0, confirmed=True)
        self.assertEqual(rpc.close(timeout=.5), expected)
        self.assertEqual([row['op'] for row in process.requests], ['arm'])

    def test_boolean_request_id_cannot_confirm_arm(self):
        rpc, process = self.make_rpc()
        process.on_request = lambda _request: process.exit(
            {'id': True, 'ok': True, 'result': {'success': True}},
            {'event': 'closed', 'result': cleanup_result()})
        with self.assertRaisesRegex(RuntimeError, 'closed before request confirmation'):
            rpc.call('arm', mode=1, duration_sec=0, confirmed=True)
        rpc.close(timeout=.5)

    def test_cached_closed_prevents_new_arm_request(self):
        rpc, process = self.make_rpc()
        process.exit({'event': 'closed', 'result': cleanup_result()})
        self.assertTrue(rpc.reader_done.wait(.5))
        with self.assertRaisesRegex(RuntimeError, 'already closed'):
            rpc.call('arm', mode=1, duration_sec=0, confirmed=True)
        self.assertEqual(process.requests, [])
        rpc.close(timeout=.5)

    def test_call_lock_wait_is_bounded_without_sending_request(self):
        rpc, process = self.make_rpc()
        rpc.lock.acquire()
        started = time.monotonic()
        try:
            with self.assertRaisesRegex(RuntimeError, 'lock timed out'):
                rpc.call('arm', timeout=.02)
        finally:
            rpc.lock.release()
        self.assertLess(time.monotonic() - started, .5)
        self.assertEqual(process.requests, [])
        process.exit({'event': 'closed', 'result': cleanup_result()})
        rpc.close(timeout=.5)

    def test_deliberate_close_prevents_late_heartbeat_request(self):
        rpc, process = self.make_rpc()
        rpc.closing.set()
        with self.assertRaisesRegex(RuntimeError, 'closing; no request sent'):
            rpc.call('ping', timeout=.02)
        self.assertEqual(process.requests, [])
        process.exit({'event': 'closed', 'result': cleanup_result()})
        rpc.close(timeout=.5)

    def test_inflight_heartbeat_and_close_are_serialized_without_retry(self):
        rpc, process = self.make_rpc()
        ping_sent = threading.Event()
        values, failures = [], []

        def receive(request):
            if request['op'] == 'ping':
                ping_sent.set()
            else:
                process.exit({'event': 'closed', 'result': cleanup_result()})

        def execute(operation):
            try:
                values.append(operation())
            except Exception as error:
                failures.append(error)

        process.on_request = receive
        ping = threading.Thread(target=execute, args=(lambda: rpc.call('ping', timeout=.5),))
        ping.start()
        self.assertTrue(ping_sent.wait(.5))
        closer = threading.Thread(target=execute, args=(lambda: rpc.close(timeout=.5),))
        closer.start()
        self.assertTrue(rpc.closing.wait(.5))
        process.send({'id': process.requests[0]['id'], 'ok': True, 'result': {'running': False}})
        ping.join(.5)
        closer.join(.5)
        self.assertFalse(ping.is_alive())
        self.assertFalse(closer.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual([row['op'] for row in process.requests], ['ping', 'stop'])
        self.assertIn(cleanup_result(), values)

    def test_unresponsive_stop_is_bounded_and_not_retried(self):
        rpc, process = self.make_rpc()
        started = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'cleanup uncertain'):
            rpc.close(timeout=.02)
        self.assertLess(time.monotonic() - started, .5)
        self.assertEqual([row['op'] for row in process.requests], ['stop'])
        with self.assertRaisesRegex(RuntimeError, 'cleanup uncertain'):
            rpc.close(timeout=.02)
        self.assertEqual(len(process.requests), 1)


if __name__ == '__main__':
    unittest.main()
