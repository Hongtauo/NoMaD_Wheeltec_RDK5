"""Opt-in policy wiring only; never connect to a robot or launch its hardware."""
import contextlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

import robot_console as console
from test_robot_console_protocol import cleanup_result


class OperatorPauseResumeEntrypointTests(unittest.TestCase):
    def test_invalid_combinations_fail_before_credentials_or_connection(self):
        for arguments in (
            ['--operator-pause-resume'],
            ['--transport', 'api', '--operator-pause-resume'],
            ['--transport', 'api', '--network-pause-resume',
             '--operator-pause-resume', '--sync-only'],
        ):
            with self.subTest(arguments=arguments), \
                    mock.patch.object(console, 'read_password') as password, \
                    mock.patch.object(console, 'ReconnectingRpc') as rpc, \
                    mock.patch.object(console, 'ssh_command') as command, \
                    contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as stopped:
                console.main(arguments)
            self.assertEqual(stopped.exception.code, 2)
            password.assert_not_called()
            rpc.assert_not_called()
            command.assert_not_called()

    def test_policy_reaches_client_start_and_report_for_all_three_profiles(self):
        for speed in ('0.05', '0.1', '0.2'):
            with self.subTest(speed=speed), tempfile.TemporaryDirectory() as temporary:
                rpc = mock.Mock()
                rpc.reconnectable = True
                rpc.connection_status.return_value = {'connected': True}
                rpc.diagnostics = []
                rpc.close.return_value = cleanup_result()
                rpc.call.return_value = {'operator_pause_resume': True}
                with mock.patch.object(console, '__file__',
                        str(Path(temporary) / 'ros2_ws/scripts/robot_console.py')), \
                        mock.patch.object(console, 'read_password', return_value='dummy-test'), \
                        mock.patch.object(console, 'ReconnectingRpc', return_value=rpc) as factory, \
                        mock.patch.object(console, 'synchronize', return_value={
                            'verified': {'pc_clock_bound_sec': .01}}), \
                        mock.patch.object(console, 'operator_menu') as menu, \
                        contextlib.redirect_stdout(io.StringIO()):
                    code = console.main(['--transport', 'api', '--robot-host', '10.201.207.37',
                        '--max-linear-mps', speed, '--network-pause-resume', '--operator-pause-resume'])
                self.assertEqual(code, 0)
                factory.assert_called_once_with(transport='api', robot_host='10.201.207.37',
                                               autofill=True, operator_pause_resume=True)
                rpc.call.assert_any_call('start', max_linear_mps=float(speed),
                                         network_pause_resume=True, operator_pause_resume=True)
                self.assertNotIn('arm', [call.args[0] for call in rpc.call.call_args_list])
                menu.assert_called_once_with(rpc, float(speed), True)
                report = json.loads(next(Path(temporary).glob('outputs/ros2_robot_operator/*/report.json'))
                                    .read_text(encoding='utf-8'))
                self.assertTrue(report['operator_pause_resume'])
                self.assertFalse(report['automatic_arm'])

    @unittest.skipUnless(shutil.which('pwsh') or shutil.which('powershell'), 'Windows PowerShell required')
    def test_powershell_forwards_opt_in_and_rejects_invalid_combinations_without_python(self):
        wrapper = Path(__file__).resolve().parents[2] / 'docker/ros2/start_robot_campus.ps1'
        path = str(wrapper).replace("'", "''")
        program = """
$ErrorActionPreference='Stop'
function global:python { $global:forwarded=@($args); $global:LASTEXITCODE=0 }
function global:ssh { throw 'SSH must not run in this test' }
& 'WRAPPER' -RobotHost 10.201.207.37 -MaxLinearMps 0.1 -NetworkPauseResume -OperatorPauseResume
$valid=@($global:forwarded)
$rejections=0
$global:forwarded=$null
try { & 'WRAPPER' -RobotHost 10.201.207.37 -OperatorPauseResume } catch { $rejections++ }
try { & 'WRAPPER' -RobotHost 10.201.207.37 -NetworkPauseResume -OperatorPauseResume -SyncOnly } catch { $rejections++ }
@{valid=$valid; rejections=$rejections; invalidInvokedPython=($null -ne $global:forwarded)} | ConvertTo-Json -Compress
""".replace('WRAPPER', path)
        result = subprocess.run([shutil.which('pwsh') or shutil.which('powershell'),
            '-NoProfile', '-NonInteractive', '-Command', program], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertIn('--operator-pause-resume', data['valid'])
        self.assertIn('--network-pause-resume', data['valid'])
        self.assertEqual(data['valid'].count('--operator-pause-resume'), 1)
        self.assertEqual(data['rejections'], 2)
        self.assertFalse(data['invalidInvokedPython'])


if __name__ == '__main__':
    unittest.main()
