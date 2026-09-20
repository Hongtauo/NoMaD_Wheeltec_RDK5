"""Saved-credential console integration; no credentials, SSH or hardware access."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import robot_console as console
from test_robot_console_protocol import FakeProcess, cleanup_result


class PasswordConsoleTests(unittest.TestCase):
    def test_autofill_keeps_host_key_checks_and_secret_out_of_command(self):
        process = FakeProcess()
        self.addCleanup(process.exit)
        environment = {'SSH_ASKPASS': 'test-helper.exe', 'SSH_ASKPASS_REQUIRE': 'force'}
        with mock.patch.object(console, 'auto_ssh_options', return_value={'env': environment}), \
                mock.patch.object(console.subprocess, 'Popen', return_value=process) as popen:
            rpc = console.Rpc('api', '10.201.207.37', autofill=True)
        command = popen.call_args.args[0]
        for setting in ('StrictHostKeyChecking=yes', 'UpdateHostKeys=no',
                        'HostKeyAlias=192.168.0.100', 'NumberOfPasswordPrompts=1',
                        'PreferredAuthentications=password', 'KbdInteractiveAuthentication=no'):
            self.assertIn(setting, command)
        self.assertEqual(command[-2], 'wheeltec@10.201.207.37')
        self.assertEqual(popen.call_args.kwargs['env'], environment)
        self.assertNotIn('password=', str(command).lower())
        process.exit({'event': 'closed', 'result': cleanup_result()})
        self.assertTrue(rpc.close(timeout=.5)['stopped'])

    def test_manual_rpc_never_creates_autofill_helper(self):
        process = FakeProcess()
        self.addCleanup(process.exit)
        with mock.patch.object(console, 'auto_ssh_options') as options, \
                mock.patch.object(console.subprocess, 'Popen', return_value=process) as popen:
            rpc = console.Rpc('api', '10.201.207.37')
        options.assert_not_called()
        self.assertNotIn('env', popen.call_args.kwargs)
        process.exit({'event': 'closed', 'result': cleanup_result()})
        rpc.close(timeout=.5)

    def run_console(self, *, manual=False, saved='test-only-secret', auth_error=False):
        with tempfile.TemporaryDirectory() as temporary:
            fake_file = str(Path(temporary) / 'ros2_ws/scripts/robot_console.py')
            rpc = mock.Mock()
            rpc.close.return_value = cleanup_result()
            rpc.call.return_value = {'prepared': True}
            output = io.StringIO()
            arguments = ['--transport', 'api', '--robot-host', '10.201.207.37', '--sync-only']
            if manual:
                arguments.append('--manual-password')
            with mock.patch.object(console, '__file__', fake_file), \
                    mock.patch.object(console, 'read_password', return_value=saved) as read, \
                    mock.patch.object(console.getpass, 'getpass', return_value='manual-test-secret') as prompt, \
                    mock.patch.object(console, 'Rpc', return_value=rpc,
                        side_effect=RuntimeError('SSH authentication failed') if auth_error else None) as factory, \
                    mock.patch.object(console, 'synchronize', return_value={
                        'verified': {'pc_clock_bound_sec': .01}}), \
                    mock.patch.object(console, 'operator_menu') as menu, \
                    contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                code = console.main(arguments)
            reports = list(Path(temporary).glob('outputs/ros2_robot_operator/*/report.json'))
            self.assertEqual(len(reports), 1)
            text = reports[0].read_text(encoding='utf-8')
            for secret in ('test-only-secret', 'manual-test-secret'):
                self.assertNotIn(secret, text)
                self.assertNotIn(secret, output.getvalue())
                self.assertNotIn(secret, repr(factory.call_args))
            menu.assert_not_called()
            self.assertNotIn('arm', [call.args[0] for call in rpc.call.call_args_list])
            return code, json.loads(text), read, prompt, factory, rpc

    def test_saved_password_fills_ssh_and_sudo_without_changing_motion_confirmation(self):
        code, report, read, prompt, factory, rpc = self.run_console()
        self.assertEqual(code, 0)
        read.assert_called_once_with()
        prompt.assert_not_called()
        factory.assert_called_once_with(transport='api', robot_host='10.201.207.37', autofill=True)
        rpc.call.assert_called_once_with('prepare', sudo_password='test-only-secret')
        self.assertTrue(report['password_autofill'])
        self.assertFalse(report['automatic_arm'])

    def test_manual_override_never_reads_saved_credential(self):
        code, report, read, prompt, factory, rpc = self.run_console(manual=True)
        self.assertEqual(code, 0)
        read.assert_not_called()
        prompt.assert_called_once()
        self.assertFalse(factory.call_args.kwargs['autofill'])
        self.assertFalse(report['password_autofill'])
        rpc.call.assert_called_once_with('prepare', sudo_password='manual-test-secret')

    def test_missing_credential_falls_back_to_interactive_without_saving(self):
        code, report, read, prompt, factory, rpc = self.run_console(saved=None)
        self.assertEqual(code, 0)
        prompt.assert_called_once()
        self.assertFalse(factory.call_args.kwargs['autofill'])

    def test_authentication_failure_never_prepares_clock_or_starts_robot(self):
        code, report, read, prompt, factory, rpc = self.run_console(auth_error=True)
        self.assertEqual(code, 1)
        self.assertEqual(report['status'], 'failed')
        rpc.call.assert_not_called()
        rpc.close.assert_not_called()
        prompt.assert_not_called()


if __name__ == '__main__':
    unittest.main()
