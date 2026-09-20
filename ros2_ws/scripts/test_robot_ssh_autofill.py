"""Askpass build/configuration tests: no credentials, network, or real compiler."""

import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

import robot_ssh_autofill as autofill


class AskpassTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / 'RobotSshAskpass.cs'
        self.source.write_text('// test fixture without any credentials', encoding='utf-8')
        self.compiler = self.root / 'csc.exe'
        self.compiler.touch()
        self.digest = hashlib.sha256(self.source.read_bytes()).hexdigest()
        self.output = (self.root / 'SocialEmbodiedAgent' / 'ssh-askpass'
                       / self.digest / 'RobotSshAskpass.exe')
        for patcher in (
                mock.patch.object(autofill.sys, 'platform', 'win32'),
                mock.patch.object(autofill, 'SOURCE', self.source),
                mock.patch.object(autofill, '_compiler_candidates', return_value=[self.compiler]),
                mock.patch.dict(os.environ, {'LOCALAPPDATA': str(self.root)})):
            patcher.start()
            self.addCleanup(patcher.stop)

    def fake_compile(self, command, **kwargs):
        Path(next(item[5:] for item in command if item.startswith('/out:'))).write_bytes(b'fake PE')
        return subprocess.CompletedProcess(command, 0, b'', b'')

    def test_build_is_source_versioned_hidden_and_atomic(self):
        with mock.patch.object(autofill.subprocess, 'run', side_effect=self.fake_compile) as run:
            self.assertEqual(autofill.ensure_askpass(), self.output)
        self.assertEqual(self.output.read_bytes(), b'fake PE')
        command = run.call_args.args[0]
        self.assertEqual(command[0], str(self.compiler))
        self.assertEqual(command[-1], str(self.source))
        self.assertNotEqual(next(value[5:] for value in command if value.startswith('/out:')),
                            str(self.output))
        self.assertNotIn('shell', run.call_args.kwargs)
        self.assertEqual(run.call_args.kwargs['creationflags'], 0x08000000)
        self.assertEqual(run.call_args.kwargs['timeout'], 30)
        self.assertEqual(run.call_args.kwargs['stdin'], subprocess.DEVNULL)
        self.assertEqual(run.call_args.kwargs['stdout'], subprocess.PIPE)
        self.assertEqual(run.call_args.kwargs['stderr'], subprocess.PIPE)
        self.assertEqual(list(self.output.parent.glob('*.exe')), [self.output])

    def test_cached_helper_needs_no_compiler(self):
        self.output.parent.mkdir(parents=True)
        self.output.write_bytes(b'cached')
        with mock.patch.object(autofill.subprocess, 'run') as run:
            self.assertEqual(autofill.ensure_askpass(), self.output)
        run.assert_not_called()

    def test_source_change_gets_another_cache(self):
        self.output.parent.mkdir(parents=True)
        self.output.write_bytes(b'old')
        self.source.write_text('// a different implementation', encoding='utf-8')
        with mock.patch.object(autofill.subprocess, 'run', side_effect=self.fake_compile):
            updated = autofill.ensure_askpass()
        self.assertNotEqual(updated, self.output)
        self.assertEqual(self.output.read_bytes(), b'old')

    def test_no_compiler_fails_before_subprocess(self):
        self.compiler.unlink()
        with mock.patch.object(autofill.subprocess, 'run') as run:
            with self.assertRaisesRegex(RuntimeError, 'Could not prepare'):
                autofill.ensure_askpass()
        run.assert_not_called()

    def test_compiler_failure_is_generic_and_cleans_partial_output(self):
        def fail(command, **kwargs):
            self.fake_compile(command, **kwargs)
            return subprocess.CompletedProcess(command, 1, b'private diagnostic', b'other detail')
        with mock.patch.object(autofill.subprocess, 'run', side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, '^' + autofill._BUILD_ERROR + '$'):
                autofill.ensure_askpass()
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.output.parent.glob('*.exe')), [])

    def test_success_without_output_fails(self):
        with mock.patch.object(autofill.subprocess, 'run',
                               return_value=subprocess.CompletedProcess([], 0, b'', b'')):
            with self.assertRaises(RuntimeError):
                autofill.ensure_askpass()

    def test_subprocess_timeout_does_not_disclose_arguments(self):
        with mock.patch.object(autofill.subprocess, 'run',
                               side_effect=subprocess.TimeoutExpired(['private command'], 30)):
            with self.assertRaisesRegex(RuntimeError, '^' + autofill._BUILD_ERROR + '$'):
                autofill.ensure_askpass()

    def test_other_os_rejected_before_source_or_process(self):
        with mock.patch.object(autofill.sys, 'platform', 'linux'), \
                mock.patch.object(autofill.subprocess, 'run') as run:
            with self.assertRaisesRegex(RuntimeError, 'requires Windows'):
                autofill.ensure_askpass()
        run.assert_not_called()

    def test_missing_or_relative_local_appdata_rejected(self):
        for value in ('', 'relative-cache'):
            with self.subTest(value=value), mock.patch.dict(os.environ, {'LOCALAPPDATA': value}):
                with self.assertRaises(RuntimeError):
                    autofill.ensure_askpass()

    def test_child_environment_only_without_secrets_or_parent_changes(self):
        values = {'SSH_ASKPASS': 'old helper', 'SSH_ASKPASS_REQUIRE': 'never',
                  'SSH_ASKPASS_PROMPT': 'confirm', 'NOMAD_ROBOT_ASKPASS': '0',
                  'UNRELATED_VALUE': 'preserved'}
        with mock.patch.dict(os.environ, values), \
                mock.patch.object(autofill, 'ensure_askpass', return_value=self.output):
            before = os.environ.copy()
            options = autofill.auto_ssh_options()
            self.assertEqual(os.environ, before)
        self.assertEqual(set(options), {'env'})
        environment = options['env']
        self.assertEqual(environment['SSH_ASKPASS'], str(self.output))
        self.assertEqual(environment['SSH_ASKPASS_REQUIRE'], 'force')
        self.assertEqual(environment['NOMAD_ROBOT_ASKPASS'], '1')
        self.assertNotIn('SSH_ASKPASS_PROMPT', environment)
        self.assertEqual(environment['UNRELATED_VALUE'], 'preserved')


class NativeHelperContractTests(unittest.TestCase):
    def test_native_helper_keeps_narrow_credential_and_invocation_guards(self):
        source = autofill.SOURCE.read_text(encoding='utf-8')
        for guard in ('args.Length == 1', 'args[0].Trim() == Prompt',
                      'Console.IsOutputRedirected', 'GetFileType(GetStdHandle(-11)) == FileTypePipe',
                      'GetEnvironmentVariable("SSH_ASKPASS_REQUIRE") == "force"',
                      'GetEnvironmentVariable("NOMAD_ROBOT_ASKPASS") == "1"',
                      'String.IsNullOrEmpty(Environment.GetEnvironmentVariable("SSH_ASKPASS_PROMPT"))',
                      'SocialEmbodiedAgent/robot/wheeltec@192.168.0.100',
                      'wheeltec@192.168.0.100\'s password:', 'new UTF8Encoding(false, true)',
                      'new UnicodeEncoding(false, false, true)', 'CredFree(allocation)',
                      'Array.Clear(password, 0, password.Length)',
                      'Marshal.WriteByte(credential.CredentialBlob, index, 0)'):
            with self.subTest(guard=guard):
                self.assertIn(guard, source)
        self.assertNotIn('Console.Write', source)
        self.assertNotIn('Console.Error', source)


if __name__ == '__main__':
    unittest.main()
