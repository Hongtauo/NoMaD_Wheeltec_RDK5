"""Operator protocol/lifecycle tests; no SSH, sudo, Docker or robot hardware."""
import io
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import time
import unittest
from unittest import mock

import robot_console as console
import robot_operator as operator


class OperatorTests(unittest.TestCase):
    def test_explicit_modes_and_confirmation(self):
        for mode, duration in ((0, 1), (0, 10.), (1, 0.)):
            self.assertEqual(operator.validate_arm({'mode': mode, 'duration_sec': duration, 'confirmed': True}),
                             (mode, float(duration)))
        for request in ({}, {'mode': 0, 'duration_sec': 5},
                        {'mode': 1, 'duration_sec': 0, 'confirmed': 'true'}):
            with self.assertRaises(ValueError):
                operator.validate_arm(request)
        for mode, duration in ((0, 0), (0, 10.001), (0, float('nan')), (1, .1), (2, 0), (True, 0)):
            with self.subTest(mode=mode, duration=duration), self.assertRaises(ValueError):
                operator.validate_arm({'mode': mode, 'duration_sec': duration, 'confirmed': True})

    def test_prepare_secret_consumed_not_returned(self):
        session = operator.OperatorSession()
        clock = mock.Mock()
        clock.prepare.return_value = {'success': True}
        request = {'op': 'prepare', 'sudo_password': 'not-a-real-password'}
        with mock.patch.object(operator, 'RobotClockSession', return_value=clock):
            self.assertEqual(session.dispatch(request), {'success': True})
        self.assertNotIn('sudo_password', request)
        self.assertFalse(session.running)
        self.assertIsNone(session.launcher)

    def test_start_requires_verified_token_no_process_on_refusal(self):
        session = operator.OperatorSession()
        with mock.patch.object(operator.subprocess, 'Popen') as popen:
            for cap in (.2, .05, .3, float('nan'), True):
                with self.assertRaises((RuntimeError, ValueError)):
                    session.start(cap)
            popen.assert_not_called()

    def test_missing_pidfd_refuses_before_any_ros_or_hardware_start(self):
        session = operator.OperatorSession()
        session.token = {'verified': True}
        with mock.patch.object(session, 'require_pidfd', side_effect=RuntimeError('pidfd unavailable')), \
                mock.patch.object(operator, 'RosOperator') as ros, \
                mock.patch.object(operator.subprocess, 'Popen') as popen:
            with self.assertRaisesRegex(RuntimeError, 'pidfd'):
                session.start(.2)
        ros.assert_not_called()
        popen.assert_not_called()
        self.assertIsNone(session.output)

    def test_live_process_scan_is_throttled_but_cleanup_scan_is_not(self):
        session = operator.OperatorSession()
        session.running, session.launcher = True, mock.Mock()
        session.launcher.poll.return_value = None
        session.last_contact, session.last_process_scan = 1., 1.
        with mock.patch.object(session, 'owned_processes', return_value={}) as scan, \
                mock.patch.object(operator.time, 'monotonic', return_value=1.1):
            session.tick()
            scan.assert_not_called()
        with mock.patch.object(session, 'owned_processes', return_value={}) as scan, \
                mock.patch.object(operator.time, 'monotonic', return_value=1.3):
            session.tick()
            scan.assert_called_once()
        session.launcher.poll.return_value = 0
        with mock.patch.object(session, 'owned_processes', return_value={}) as scan:
            self.assertTrue(session.stop_owned_processes())
            scan.assert_called_once()

    def test_no_arm_before_running(self):
        session = operator.OperatorSession()
        with self.assertRaises(RuntimeError):
            session.dispatch({'op': 'arm', 'mode': 1, 'duration_sec': 0, 'confirmed': True})

    def test_verified_clock_is_not_reset(self):
        session = operator.OperatorSession()
        session.clock, session.token = mock.Mock(), {'verified': True}
        with self.assertRaises(RuntimeError):
            session.dispatch({'op': 'set_time', 'epoch_ns': 1789000000000000000})
        session.clock.set_time.assert_not_called()

    def test_heartbeat_timeout_and_child_failure(self):
        session = operator.OperatorSession()
        session.running, session.launcher = True, mock.Mock()
        session.launcher.poll.return_value = None
        session.last_contact = 1.
        with mock.patch.object(operator.time, 'monotonic', return_value=6.), \
                mock.patch.object(session, 'owned_processes', return_value={}):
            with self.assertRaisesRegex(RuntimeError, 'heartbeat'):
                session.tick()
        session.launcher.poll.return_value = 1
        with mock.patch.object(session, 'owned_processes', return_value={}):
            with self.assertRaisesRegex(RuntimeError, 'launch exited'):
                session.tick()

    def test_cleanup_disarms_before_stopping_then_restores_clock(self):
        session, order = operator.OperatorSession(), []
        session.ros, session.clock = mock.Mock(), mock.Mock()
        session.ros.disarm.side_effect = lambda: order.append('disarm')
        session.clock.restore.side_effect = lambda: order.append('restore') or {'success': True}
        with mock.patch.object(session, 'stop_owned_processes', side_effect=lambda: order.append('stop') or True):
            result = session.close()
        self.assertEqual(order, ['disarm', 'stop', 'restore'])
        self.assertEqual(result['errors'], [])
        self.assertIs(session.close(), result)

    def test_clock_restore_failure_propagates(self):
        session = operator.OperatorSession()
        session.clock = mock.Mock()
        session.clock.restore.return_value = {'success': False, 'pending_services': ['ntp.service']}
        result = session.close()
        self.assertTrue(result['errors'])
        self.assertIn('ntp.service', result['errors'][0])

    def test_remaining_ros_prevents_ntp_restore(self):
        session = operator.OperatorSession()
        session.clock = mock.Mock()
        session.clock._check_no_ros_processes.side_effect = RuntimeError('ROS remains')
        result = session.close()
        session.clock.restore.assert_not_called()
        self.assertTrue(result['errors'])

    def test_unstopped_children_prevent_clock_restore_even_if_parent_died(self):
        session = operator.OperatorSession()
        session.clock, session.launcher = mock.Mock(), mock.Mock()
        session.launcher.poll.return_value = 0
        with mock.patch.object(session, 'stop_owned_processes', return_value=False):
            result = session.close()
        session.clock.restore.assert_not_called()
        self.assertFalse(result['stopped'])
        self.assertTrue(result['errors'])

    def test_orphaned_tagged_child_is_signalled_without_parent(self):
        session = operator.OperatorSession()
        session.launcher = mock.Mock(pid=100)
        session.launcher.poll.return_value = 1
        with mock.patch.object(session, 'owned_processes', side_effect=[{200: '123'}, {}]), \
                mock.patch.object(session, 'signal_owned') as kill:
            self.assertTrue(session.stop_owned_processes())
        kill.assert_called_once_with(200, '123', operator.signal.SIGINT)

    def test_completed_prepare_rollback_not_rechecked_against_existing_ros(self):
        session = operator.OperatorSession()
        session.clock = mock.Mock(phase='restored')
        restored = {'success': True, 'pending_services': [], 'restored_services': []}
        session.clock.restoration_result.return_value = restored
        session.clock._check_no_ros_processes.side_effect = RuntimeError('Pre-existing ROS')
        result = session.close()
        self.assertEqual(result['errors'], [])
        self.assertEqual(result['clock_restore'], restored)
        session.clock._check_no_ros_processes.assert_not_called()
        session.clock.restore.assert_not_called()

    def test_failed_rollback_still_requires_safe_restoration(self):
        session = operator.OperatorSession()
        session.clock = mock.Mock(phase='restored')
        session.clock.restoration_result.return_value = {'success': False, 'pending_services': ['ntp.service']}
        session.clock._check_no_ros_processes.side_effect = RuntimeError('Existing ROS')
        result = session.close()
        self.assertTrue(result['errors'])
        session.clock._check_no_ros_processes.assert_called_once()

    def test_log_and_token_failures_do_not_skip_clock_restore(self):
        session = operator.OperatorSession()
        session.clock, session.launch_log, session.token_path = mock.Mock(), mock.Mock(), mock.Mock()
        session.clock.restore.return_value = {'success': True}
        session.launch_log.close.side_effect = OSError('disk close failed')
        session.token_path.unlink.side_effect = PermissionError('token failed')
        result = session.close()
        self.assertTrue(result['stopped'])
        self.assertEqual(len(result['errors']), 2)
        session.clock.restore.assert_called_once()

    def test_report_write_failure_is_returned_after_cleanup(self):
        session = operator.OperatorSession()
        session.output = mock.MagicMock()
        session.output.__truediv__.return_value.write_text.side_effect = OSError('full')
        result = session.close()
        self.assertTrue(result['stopped'])
        self.assertIn('Session report', result['errors'][0])

    def test_post_stop_tick_does_not_spin_destroyed_ros_context(self):
        session = operator.OperatorSession()
        session.ros = mock.Mock()
        session.close()
        session.ros.spin.reset_mock()
        session.tick()
        session.ros.spin.assert_not_called()

    def test_protocol_eof_and_malformed_requests(self):
        for value in ('[]\n', 'invalid\n', 'x' * 9000):
            inbox = queue.Queue()
            operator.read_requests(io.StringIO(value), inbox)
            self.assertEqual(inbox.get()['op'], '_protocol_error')
            self.assertIsNone(inbox.get())
        inbox = queue.Queue()
        operator.read_requests(io.StringIO('{"id":1,"op":"ping"}\n'), inbox)
        self.assertEqual(inbox.get()['op'], 'ping')
        self.assertIsNone(inbox.get())


class ProcessOwnershipTests(unittest.TestCase):
    @staticmethod
    def row(pid, ppid=1, sid=100, pgid=100, birth='123', uid=1000, name='python', state='S'):
        return dict(pid=pid, ppid=ppid, sid=sid, pgid=pgid, birth=birth, uid=uid, name=name, state=state)

    def session(self):
        session = operator.OperatorSession()
        session.launcher = mock.Mock(pid=100)
        session.launcher.poll.return_value = 0
        session.launch_birth = 100
        session.token = {'session_id': 'test-random-session'}
        session.owned_identities = {100: '100'}
        return session

    def scan(self, session, rows, environments, metadata_reads=None):
        def read_environment(path):
            value = environments.get(int(path.parent.name), b'')
            if isinstance(value, Exception):
                raise value
            return value
        paths = [Path('/proc') / str(pid) for pid in rows]
        def metadata(path):
            pid = int(path.name)
            sequence = (metadata_reads or {}).get(pid)
            if sequence:
                return sequence.pop(0) if len(sequence) > 1 else sequence[0]
            return rows[pid]
        with mock.patch.object(operator.Path, 'iterdir', return_value=paths), \
                mock.patch.object(session, 'process_metadata', side_effect=metadata), \
                mock.patch.object(operator.Path, 'read_bytes', autospec=True, side_effect=read_environment), \
                mock.patch.object(operator.os, 'getuid', create=True, return_value=1000):
            return session.owned_processes()

    def test_new_nondumpable_ssh_login_does_not_block_orphan_discovery(self):
        session = self.session()
        rows = {200: self.row(200),
                300: self.row(300, ppid=51, sid=300, pgid=300, name='sshd'),
                51: self.row(51, ppid=50, sid=50, pgid=50, uid=0, birth='125', name='sshd'),
                50: self.row(50, sid=50, pgid=50, uid=0, birth='50', name='sshd')}
        result = self.scan(session, rows, {200: b'NOMAD_OPERATOR_SESSION_ID=test-random-session\0',
                                           300: PermissionError()})
        self.assertEqual(result, {200: '123'})
        self.assertEqual(session.ownership_errors, [])

    def test_unknown_denied_process_kept_uncertain_but_owned_children_returned(self):
        session = self.session()
        rows = {200: self.row(200), 300: self.row(300, sid=300, pgid=300, name='sshd')}
        result = self.scan(session, rows, {200: b'NOMAD_OPERATOR_SESSION_ID=test-random-session\0',
                                           300: PermissionError()})
        self.assertEqual(result, {200: '123'})
        self.assertIn('300', session.ownership_errors[0])

    def test_tracked_child_with_unreadable_environment_remains_owned(self):
        session = self.session()
        session.owned_identities[200] = '123'
        result = self.scan(session, {200: self.row(200, sid=200, pgid=200)}, {200: PermissionError()})
        self.assertEqual(result, {200: '123'})
        self.assertEqual(session.ownership_errors, [])

    def test_known_session_and_setsid_descendant_are_tracked(self):
        session = self.session()
        rows = {100: self.row(100, birth='100'), 200: self.row(200, ppid=100),
                201: self.row(201, ppid=200, sid=201, pgid=201)}
        result = self.scan(session, rows, {200: PermissionError(), 201: PermissionError()})
        self.assertEqual(result, {100: '100', 200: '123', 201: '123'})
        self.assertEqual(session.ownership_errors, [])

    def test_reused_pid_and_sid_do_not_acquire_ownership(self):
        session = self.session()
        result = self.scan(session, {100: self.row(100, birth='200'), 200: self.row(200, birth='201')}, {})
        self.assertEqual(result, {})

    def test_mixed_snapshot_reused_parent_does_not_own_new_child(self):
        session = self.session()
        parent = self.row(200, birth='110', sid=200, pgid=200)
        session.owned_identities[200] = '110'
        child = self.row(300, ppid=200, birth='201', sid=300, pgid=300)
        replacement = self.row(200, birth='200', sid=200, pgid=200)
        result = self.scan(session, {200: parent, 300: child}, {},
                           {200: [parent, replacement]})
        self.assertNotIn(300, result)
        self.assertNotIn(300, session.owned_identities)
        self.assertEqual(session.ownership_errors, [])
        with mock.patch.object(session, 'owned_processes', side_effect=[result, {}]), \
                mock.patch.object(session, 'process_metadata', return_value=replacement), \
                mock.patch.object(operator.os, 'getuid', create=True, return_value=1000), \
                mock.patch.object(operator.os, 'pidfd_open', create=True, return_value=7) as pin, \
                mock.patch.object(operator.os, 'close'), \
                mock.patch.object(operator.signal, 'pidfd_send_signal', create=True) as send:
            self.assertTrue(session.stop_owned_processes())
        pin.assert_called_once_with(200)  # No attempt to pin or signal unrelated child 300.
        send.assert_not_called()  # Reused parent identity is rejected at signal time too.

    def test_mixed_snapshot_reused_session_anchor_does_not_own_new_member(self):
        session = self.session()
        anchor = self.row(100, birth='100')
        child = self.row(300, ppid=100, birth='201')
        replacement = self.row(100, birth='200')
        result = self.scan(session, {100: anchor, 300: child}, {},
                           {100: [anchor, replacement]})
        self.assertNotIn(300, result)
        self.assertNotIn(300, session.owned_identities)

    def test_changed_target_during_relation_check_is_not_promoted(self):
        session = self.session()
        anchor = self.row(100, birth='100')
        child = self.row(300, ppid=100)
        replacement = self.row(300, ppid=100, birth='999')
        result = self.scan(session, {100: anchor, 300: child}, {},
                           {300: [child, child, replacement]})
        self.assertNotIn(300, result)

    def test_changed_target_during_marker_read_is_not_promoted(self):
        session = self.session()
        child = self.row(300, sid=300, pgid=300)
        replacement = self.row(300, sid=300, pgid=300, birth='999')
        result = self.scan(session, {300: child},
                           {300: b'NOMAD_OPERATOR_SESSION_ID=test-random-session\0'},
                           {300: [child, replacement]})
        self.assertEqual(result, {})

    def test_unstable_parent_still_allows_exact_stable_marker_ownership(self):
        session = self.session()
        parent = self.row(200, birth='110', sid=200, pgid=200)
        session.owned_identities[200] = '110'
        child = self.row(300, ppid=200, birth='201', sid=300, pgid=300)
        replacement = self.row(200, birth='200', sid=200, pgid=200)
        result = self.scan(session, {200: parent, 300: child},
                           {300: b'NOMAD_OPERATOR_SESSION_ID=test-random-session\0'},
                           {200: [parent, replacement]})
        self.assertEqual(result[300], '201')

    def test_unstable_ssh_ancestor_does_not_exclude_unreadable_process(self):
        session = self.session()
        daemon = self.row(50, uid=0, name='sshd', birth='50', sid=50, pgid=50)
        replacement = self.row(50, uid=0, name='sshd', birth='200', sid=50, pgid=50)
        candidate = self.row(300, ppid=50, sid=300, pgid=300, birth='201', name='sshd')
        self.scan(session, {50: daemon, 300: candidate}, {300: PermissionError()},
                  {50: [daemon, replacement]})
        self.assertIn('300', session.ownership_errors[0])

    def test_process_named_sshd_is_not_sufficient_exclusion(self):
        session = self.session()
        rows = {200: self.row(200, ppid=50, sid=200, pgid=200),
                50: self.row(50, sid=50, pgid=50, birth='50', name='sshd')}
        self.scan(session, rows, {200: PermissionError()})
        self.assertTrue(session.ownership_errors)

    def test_pidfd_rechecks_identity_before_signal(self):
        session = self.session()
        with mock.patch.object(operator.os, 'pidfd_open', create=True, return_value=7), \
                mock.patch.object(operator.os, 'getuid', create=True, return_value=1000), \
                mock.patch.object(operator.os, 'close') as close, \
                mock.patch.object(operator.signal, 'pidfd_send_signal', create=True) as send, \
                mock.patch.object(session, 'process_metadata', return_value=self.row(200, birth='999')):
            session.signal_owned(200, '123', signal.SIGINT)
        send.assert_not_called()
        close.assert_called_once_with(7)

    def test_pidfd_probe_closes_handle_even_if_permission_check_fails(self):
        with mock.patch.object(operator.os, 'pidfd_open', create=True, return_value=7), \
                mock.patch.object(operator.os, 'close') as close, \
                mock.patch.object(operator.signal, 'pidfd_send_signal', create=True,
                                  side_effect=PermissionError('denied')):
            with self.assertRaises(PermissionError):
                operator.OperatorSession.require_pidfd()
        close.assert_called_once_with(7)

    def test_unknown_process_does_not_prevent_stopping_verified_process(self):
        session = self.session()
        session.ownership_errors = ['Cannot verify PID 300']
        with mock.patch.object(session, 'owned_processes', side_effect=[{200: '123'}, {}]), \
                mock.patch.object(session, 'signal_owned') as send:
            self.assertFalse(session.stop_owned_processes())
        send.assert_called_once_with(200, '123', signal.SIGINT)
        self.assertEqual(session.process_cleanup_errors, ['Cannot verify PID 300'])

    @unittest.skipUnless(sys.platform == 'linux' and hasattr(os, 'pidfd_open'), 'Linux pidfd integration')
    def test_real_launcher_exit_leaves_no_tagged_native_orphan(self):
        session = self.session()
        session.token = {'session_id': 'isolated-cleanup-' + str(os.getpid()) + '-' + str(time.monotonic_ns())}
        code = ('import signal,subprocess,sys,time; '
                'signal.signal(signal.SIGINT, lambda *a: sys.exit(0)); '
                'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"],start_new_session=True); '
                'print(p.pid,flush=True); time.sleep(60)')
        session.launcher = subprocess.Popen([sys.executable, '-u', '-c', code], start_new_session=True,
                                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                            env=dict(os.environ, NOMAD_OPERATOR_SESSION_ID=session.token['session_id']),
                                            text=True)
        child = int(session.launcher.stdout.readline())
        session.launch_birth = int(session.process_metadata(Path(f'/proc/{session.launcher.pid}'))['birth'])
        session.owned_identities = {session.launcher.pid: str(session.launch_birth)}
        child_birth = session.process_metadata(Path(f'/proc/{child}'))['birth']
        try:
            self.assertTrue(session.stop_owned_processes(), session.process_cleanup_errors)
            self.assertEqual(session.owned_processes(), {})
        finally:
            # Only this test's exact recorded identities, never a process-name kill.
            session.signal_owned(child, child_birth, signal.SIGTERM)
            session.signal_owned(session.launcher.pid, str(session.launch_birth), signal.SIGTERM)
            session.launcher.wait(timeout=5)
            session.launcher.stdout.close()


class ConsoleTests(unittest.TestCase):
    def test_invalid_handshake_closes_ssh(self):
        process = mock.Mock()
        process.stdout = io.StringIO('{"event":"wrong"}\n')
        with mock.patch.object(console.subprocess, 'Popen', return_value=process):
            with self.assertRaises(RuntimeError):
                console.Rpc()
        process.stdin.close.assert_called_once()
        process.wait.assert_called_once_with(timeout=5.)

    def test_clock_step_rejected(self):
        with mock.patch.object(console.time, 'time_ns', side_effect=[10**12, 10**12 + 100000000]), \
                mock.patch.object(console.time, 'monotonic_ns', side_effect=[10**9, 10**9 + 1000000]):
            with self.assertRaisesRegex(RuntimeError, 'clock stepped'):
                console.timed_sample(lambda: 10**12)

    def test_sample_offset_and_rtt_bound(self):
        with mock.patch.object(console.time, 'time_ns', side_effect=[10**12, 10**12 + 10000000]), \
                mock.patch.object(console.time, 'monotonic_ns', side_effect=[10**9, 10**9 + 10000000]):
            result = console.timed_sample(lambda: 10**12 + 5000000)
        self.assertEqual(result['offset_sec'], 0)
        self.assertAlmostEqual(result['bound_sec'], .006)

    def test_sync_verifies_only_after_bound_passed(self):
        rpc = mock.Mock()
        row = {'offset_sec': .001, 'rtt_sec': .002, 'uncertainty_sec': .002, 'bound_sec': .003}
        rpc.call.side_effect = [{'success': True}, {'verified': True, 'pc_clock_bound_sec': .003}]
        with mock.patch.object(console, 'timed_sample', return_value=row):
            result = console.synchronize(rpc)
        self.assertTrue(result['verified']['verified'])
        self.assertEqual([call.args[0] for call in rpc.call.call_args_list], ['set_time', 'verify'])

    def test_sync_never_starts_or_arms_and_refuses_poor_bound(self):
        rpc = mock.Mock()
        with mock.patch.object(console, 'timed_sample', return_value={'uncertainty_sec': .2, 'rtt_sec': .4}):
            with self.assertRaises(RuntimeError):
                console.synchronize(rpc)
        rpc.call.assert_not_called()

    def test_menu_requires_exact_run_and_never_repeats_arm(self):
        rpc = mock.Mock(fault=None)
        rpc.call.return_value = {'success': True}
        with mock.patch('builtins.input', side_effect=['c', 'no', 'q']), mock.patch('builtins.print'):
            console.operator_menu(rpc, .2)
        rpc.call.assert_not_called()
        with mock.patch('builtins.input', side_effect=['t', '5', 'RUN', 's', 'q']), mock.patch('builtins.print'):
            console.operator_menu(rpc, .2)
        self.assertEqual(rpc.call.call_args_list, [mock.call('arm', mode=0, duration_sec=5., confirmed=True),
                                                 mock.call('disarm')])


if __name__ == '__main__':
    unittest.main()
