"""Camera runner tests; race regressions are entirely mocked and host independent.

The optional Linux tests launch only harmless local sleepers, never ROS/hardware.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import robot_clock
import run_campus_shadow as shadow


class ShadowOwnershipTests(unittest.TestCase):
    def test_stopped_launcher_still_runs_verified_descendant_cleanup(self):
        group = shadow.OwnedProcess()
        group.owner.launcher = mock.Mock()
        group.owner.launcher.poll.return_value = 0
        group.owner.launch_birth = 100
        with mock.patch.object(group.owner, 'stop_owned_processes', return_value=True) as cleanup:
            group.close()
        cleanup.assert_called_once()

    def test_uncertain_cleanup_is_not_success_and_does_not_signal_group(self):
        group = shadow.OwnedProcess()
        group.owner.launcher = mock.Mock()
        group.owner.launch_birth = 100
        group.owner.process_cleanup_errors = ['Cannot verify potential operator process PID 123']
        with mock.patch.object(group.owner, 'stop_owned_processes', return_value=False), \
                mock.patch.object(shadow.os, 'killpg', create=True) as killpg:
            with self.assertRaisesRegex(RuntimeError, 'cleanup is unconfirmed'):
                group.close()
        killpg.assert_not_called()

    def test_missing_birth_identity_does_not_guess_pid_ownership(self):
        group = shadow.OwnedProcess()
        group.owner.launcher = mock.Mock()
        with mock.patch.object(group.owner, 'stop_owned_processes') as cleanup:
            with self.assertRaisesRegex(RuntimeError, 'Cannot verify'):
                group.close()
        cleanup.assert_not_called()

    def test_scan_errors_fail_closed(self):
        group = shadow.OwnedProcess()
        group.owner.launcher = mock.Mock()
        group.owner.launch_birth = 100
        group.owner.ownership_errors = ['unverifiable process']
        with mock.patch.object(group.owner, 'owned_processes'):
            with self.assertRaisesRegex(RuntimeError, 'unverifiable'):
                group.scan()

    def test_no_process_started_if_pidfd_not_supported(self):
        group = shadow.OwnedProcess()
        with mock.patch.object(group.owner, 'require_pidfd', side_effect=RuntimeError('pidfd missing')), \
                mock.patch.object(shadow.subprocess, 'Popen') as popen:
            with self.assertRaisesRegex(RuntimeError, 'pidfd'):
                group.start(['not-run'], None)
        popen.assert_not_called()
        group.close()

    def test_each_child_has_unique_marker_and_dedicated_session(self):
        first, second = shadow.OwnedProcess(), shadow.OwnedProcess()
        self.assertNotEqual(first.owner.token, second.owner.token)
        process = mock.Mock(pid=123)
        with mock.patch.object(first.owner, 'require_pidfd'), \
                mock.patch.object(first.owner, 'process_metadata', return_value={'uid': 1000, 'birth': '200'}), \
                mock.patch.object(shadow.os, 'getuid', return_value=1000, create=True), \
                mock.patch.object(first, 'scan'), \
                mock.patch.object(shadow.subprocess, 'Popen', return_value=process) as popen:
            first.start(['test-only'], None)
        kwargs = popen.call_args.kwargs
        self.assertIs(kwargs['start_new_session'], True)
        self.assertEqual(kwargs['env']['NOMAD_OPERATOR_SESSION_ID'], first.owner.token['session_id'])
        self.assertEqual(first.owner.owned_identities, {123: '200'})

    @unittest.skipUnless(sys.platform == 'linux' and hasattr(os, 'pidfd_open'), 'Linux pidfd test')
    def test_real_reparented_setsid_child_is_stopped_after_launcher_exit(self):
        group = shadow.OwnedProcess()
        command = ('import subprocess,sys; '
                   'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"], '
                   'start_new_session=True); print(p.pid,flush=True)')
        child = birth = None
        try:
            group.start([sys.executable, '-u', '-c', command], subprocess.PIPE)
            child = int(group.process.stdout.readline())
            birth = group.owner.process_metadata(Path(f'/proc/{child}'))['birth']
            group.process.wait(timeout=5)
            self.assertEqual(group.process.poll(), 0)
            group.close()
            self.assertEqual(group.owner.owned_processes(), {})
        finally:
            if child is not None and birth is not None:
                group.owner.signal_owned(child, birth, signal.SIGTERM)
            if group.process is not None:
                if group.owner.launch_birth is not None:
                    group.owner.signal_owned(group.process.pid, str(group.owner.launch_birth), signal.SIGTERM)
                group.process.wait(timeout=5)
                group.process.stdout.close()

    @unittest.skipUnless(os.name == 'posix', 'Linux lock semantics')
    def test_lock_excludes_concurrent_operator_and_releases(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(robot_clock, 'LOCK_PATH', Path(directory) / 'operator.lock'):
                first, second = shadow.IdleLock(), shadow.IdleLock()
                try:
                    first.acquire()
                    with self.assertRaises(BlockingIOError):
                        second.acquire()
                    first.close()
                    second.acquire()
                    second.close()
                finally:
                    first.close()
                    second.close()

    @unittest.skipUnless(os.name == 'posix', 'Linux file semantics')
    def test_lock_rejects_symlink_and_fifo(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / 'real'
            target.write_text('')
            target.chmod(0o600)
            link = root / 'link'
            link.symlink_to(target)
            with mock.patch.object(robot_clock, 'LOCK_PATH', link):
                with self.assertRaises(OSError):
                    shadow.IdleLock().acquire()
            fifo = root / 'fifo'
            os.mkfifo(fifo, 0o600)
            with mock.patch.object(robot_clock, 'LOCK_PATH', fifo):
                with self.assertRaisesRegex(RuntimeError, 'unsafe'):
                    shadow.IdleLock().acquire()


class ShadowExitRaceTests(unittest.TestCase):
    def setUp(self):
        self.camera, self.client = self.group(100), self.group(200)

    @staticmethod
    def group(pid):
        group = shadow.OwnedProcess()
        group.owner.launcher = mock.Mock(pid=pid)
        group.owner.launcher.poll.return_value = None
        group.owner.launch_birth = 123
        group.owner.owned_identities[pid] = '123'
        group.scan = mock.Mock()
        return group

    def failing_camera_scan(self, errors=None, *, repeats=False):
        errors = errors or ['Cannot verify potential operator process PID 200']
        self.camera.owner.ownership_errors = errors
        failure = RuntimeError('; '.join(errors))
        self.camera.scan.side_effect = failure if repeats else [failure, None]

    def finish_client_on_wait(self, code=0):
        def wait(timeout):
            self.assertEqual(timeout, .05)
            self.client.process.poll.return_value = code
            return code
        self.client.process.wait.side_effect = wait

    def test_natural_client_exit_is_polled_before_any_runtime_scan(self):
        self.client.process.poll.return_value = 0
        self.assertTrue(shadow.observe_children(self.camera, self.client, scan_due=True))
        self.camera.scan.assert_not_called()
        self.client.scan.assert_not_called()
        # Observing exit never replaces the mandatory final orphan cleanup.
        with mock.patch.object(self.client.owner, 'stop_owned_processes', return_value=True) as cleanup:
            self.client.close()
        cleanup.assert_called_once()

    def test_normal_poll_order_precedes_both_full_scans(self):
        order = []
        self.camera.process.poll.side_effect = lambda: order.append('camera_poll')
        self.client.process.poll.side_effect = lambda: order.append('client_poll')
        self.camera.scan.side_effect = lambda: order.append('camera_scan')
        self.client.scan.side_effect = lambda: order.append('client_scan')
        self.assertFalse(shadow.observe_children(self.camera, self.client, scan_due=True))
        self.assertEqual(order, ['camera_poll', 'client_poll', 'camera_scan', 'client_scan',
                                 'camera_poll', 'client_poll'])

    def test_exit_during_foreign_snapshot_requires_reap_and_two_strict_rescans(self):
        self.failing_camera_scan()
        self.finish_client_on_wait()
        self.assertTrue(shadow.observe_children(self.camera, self.client, scan_due=True))
        self.client.process.wait.assert_called_once_with(timeout=.05)
        self.assertEqual(self.camera.scan.call_count, 2)
        self.client.scan.assert_called_once()

    def test_still_live_unreadable_child_is_not_excused(self):
        self.failing_camera_scan()
        self.client.process.wait.side_effect = subprocess.TimeoutExpired('test-child', .05)
        with self.assertRaisesRegex(RuntimeError, 'Cannot verify'):
            shadow.observe_children(self.camera, self.client, scan_due=True)
        self.assertEqual(self.camera.scan.call_count, 1)
        self.client.scan.assert_not_called()

    def test_unknown_process_is_never_retried_or_excused(self):
        self.failing_camera_scan(['Cannot verify potential operator process PID 999'])
        with self.assertRaisesRegex(RuntimeError, 'PID 999'):
            shadow.observe_children(self.camera, self.client, scan_due=True)
        self.client.process.wait.assert_not_called()
        self.camera.process.wait.assert_not_called()

    def test_mixed_known_and_unknown_errors_remain_fatal(self):
        self.failing_camera_scan(['Cannot verify potential operator process PID 200',
                                  'Cannot verify potential operator process PID 999'])
        with self.assertRaisesRegex(RuntimeError, 'PID 999'):
            shadow.observe_children(self.camera, self.client, scan_due=True)
        self.client.process.wait.assert_not_called()

    def test_direct_child_birth_mismatch_is_not_excused(self):
        self.failing_camera_scan()
        self.client.owner.owned_identities[200] = '999'
        with self.assertRaisesRegex(RuntimeError, 'Cannot verify'):
            shadow.observe_children(self.camera, self.client, scan_due=True)
        self.client.process.wait.assert_not_called()

    def test_reaped_child_does_not_excuse_persistent_scan_error(self):
        self.failing_camera_scan(repeats=True)
        self.finish_client_on_wait()
        with self.assertRaisesRegex(RuntimeError, 'Cannot verify'):
            shadow.observe_children(self.camera, self.client, scan_due=True)
        self.assertEqual(self.camera.scan.call_count, 2)

    def test_nonzero_client_exit_is_still_failure(self):
        self.failing_camera_scan()
        self.finish_client_on_wait(code=1)
        with self.assertRaisesRegex(RuntimeError, 'Shadow client failed'):
            shadow.observe_children(self.camera, self.client, scan_due=True)

    def test_camera_exit_is_still_failure_even_when_client_completed(self):
        self.camera.process.poll.return_value = 0
        self.client.process.poll.return_value = 0
        with self.assertRaisesRegex(RuntimeError, 'Camera process exited'):
            shadow.observe_children(self.camera, self.client, scan_due=True)

    def test_throttled_iteration_still_polls_both_children(self):
        self.assertFalse(shadow.observe_children(self.camera, self.client, scan_due=False))
        self.camera.process.poll.assert_called_once()
        self.client.process.poll.assert_called_once()
        self.camera.scan.assert_not_called()
        self.client.scan.assert_not_called()


if __name__ == '__main__':
    unittest.main()
