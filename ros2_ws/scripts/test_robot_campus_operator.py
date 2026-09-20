"""Campus operator tests: no network, sudo, SSH or physical robot."""

import unittest
from unittest import mock

import robot_console as console
import robot_operator as operator
import robot_clock as clock
from nomad_navigation import robot_identity as identity


class CampusOperatorTests(unittest.TestCase):
    def test_missing_cap_retains_lowest_campus_profile_without_autoarm(self):
        for transport, default_cap in [('api', .05), ('dds', .2)]:
            with self.subTest(transport=transport):
                session = operator.OperatorSession(transport=transport)
                plan = session.dispatch(dict(op='plan'))
                self.assertEqual(plan['max_linear_mps'], default_cap)
                self.assertFalse(plan['automatic_arm'])
                self.assertFalse(plan['hardware_started'])
                with mock.patch.object(session, 'start') as start:
                    session.dispatch(dict(op='start'))
                start.assert_called_once_with(default_cap, network_pause_resume=False, operator_pause_resume=False)

    def test_pause_resume_request_preserves_explicit_option(self):
        for cap in (.05, .1, .2):
            with self.subTest(cap=cap):
                session = operator.OperatorSession(transport='api')
                with mock.patch.object(session, 'start') as start:
                    session.dispatch(dict(op='start', max_linear_mps=cap, network_pause_resume=True))
                start.assert_called_once_with(cap, network_pause_resume=True, operator_pause_resume=False)

    def test_pause_resume_rejects_other_transport_or_non_boolean_before_start(self):
        for transport, cap, enabled in [('dds', .05, True), ('dds', .1, True),
                ('dds', .2, True), ('api', .05, 'true'), ('api', .1, 1), ('api', .2, None)]:
            session = operator.OperatorSession(transport=transport)
            with self.assertRaises(ValueError):
                session.start(cap, network_pause_resume=enabled)
            self.assertIsNone(session.launcher)

    def test_all_profiles_accept_pause_option_but_still_require_verified_clock(self):
        for cap in (.05, .1, .2):
            for enabled in (False, True):
                with self.subTest(cap=cap, enabled=enabled):
                    session = operator.OperatorSession(transport='api')
                    with mock.patch.object(operator.subprocess, 'Popen') as popen, \
                            mock.patch.object(operator, 'RosOperator') as ros:
                        with self.assertRaisesRegex(RuntimeError, 'verified clock'):
                            session.start(cap, network_pause_resume=enabled)
                    popen.assert_not_called()
                    ros.assert_not_called()
                    self.assertIsNone(session.launcher)

    def test_unsupported_caps_are_rejected_before_any_process(self):
        for cap in (0., .06, .15, .21, .3, True, '0.1', float('nan'), float('inf')):
            with self.subTest(cap=cap):
                session = operator.OperatorSession(transport='api')
                with mock.patch.object(operator.subprocess, 'Popen') as popen:
                    with self.assertRaises(ValueError):
                        session.start(cap, network_pause_resume=True)
                popen.assert_not_called()

    def test_console_accepts_all_three_profiles_with_pause_option_without_io(self):
        for cap in ('0.05', '0.1', '0.2'):
            with self.subTest(cap=cap), \
                    mock.patch.object(console, 'ssh_command', side_effect=RuntimeError('stop-before-io')) as ssh:
                with self.assertRaisesRegex(RuntimeError, 'stop-before-io'):
                    console.main(['--transport', 'api', '--robot-host', '10.201.207.37',
                                  '--max-linear-mps', cap, '--network-pause-resume'])
                ssh.assert_called_once_with('api', '10.201.207.37')

    def test_console_rejects_invalid_profiles_or_dds_pause_before_io(self):
        for arguments in (['--transport', 'api', '--max-linear-mps', '0.15'],
                          ['--transport', 'dds', '--network-pause-resume']):
            with self.subTest(arguments=arguments), mock.patch.object(console, 'ssh_command') as ssh:
                with self.assertRaises(SystemExit):
                    console.main(arguments)
                ssh.assert_not_called()

    def test_ssh_reuses_enrolled_key_and_never_auto_accepts(self):
        command = console.ssh_command('api', '10.201.207.37')
        self.assertIn('StrictHostKeyChecking=yes', command)
        self.assertIn('HostKeyAlias=192.168.0.100', command)
        self.assertIn('UpdateHostKeys=no', command)
        self.assertEqual(command[-2], 'wheeltec@10.201.207.37')
        self.assertIn('campus/setup.bash', command[-1])
        self.assertTrue(command[-1].endswith('--transport api'))
        self.assertNotIn('accept-new', str(command))

    def test_legacy_ssh_is_unchanged_and_cannot_silently_use_campus_ip(self):
        command = console.ssh_command()
        self.assertEqual(command[-2], 'wheeltec@192.168.0.100')
        self.assertNotIn('campus/', command[-1])
        with self.assertRaises(ValueError):
            console.ssh_command('dds', '10.201.207.37')

    def test_host_argument_cannot_inject_ssh_options_or_shell(self):
        for target in ('-oProxyCommand=x', '1.2.3.4;whoami', 'user@1.2.3.4',
                       '127.0.0.1', '0.0.0.0', '255.255.255.255', '224.0.0.1'):
            with self.subTest(target=target), self.assertRaises(ValueError):
                console.ssh_command('api', target)

    def test_prepare_selects_api_identity_clock(self):
        session = operator.OperatorSession(transport='api')
        with mock.patch.object(operator, 'RobotClockSession') as factory:
            session.dispatch(dict(op='prepare', sudo_password='test-secret'))
        factory.assert_called_once_with('test-secret', transport='api')
        self.assertFalse(session.running)

    def test_api_clock_uses_enrollment_and_local_only_environment(self):
        session = clock.RobotClockSession('test-secret', transport='api')
        with mock.patch.object(identity, 'validate_api_environment') as environment, \
                mock.patch.object(identity, 'require_robot_identity', return_value={'machine_id': 'test'}) as enrollment, \
                mock.patch.object(session, '_run') as run:
            session._check_identity()
        environment.assert_called_once()
        enrollment.assert_called_once()
        run.assert_not_called()
        self.assertEqual(session.robot_identity, {'machine_id': 'test'})

    def test_api_identity_failure_cannot_fall_back_to_hotspot_ip(self):
        session = clock.RobotClockSession('test-secret', transport='api')
        with mock.patch.object(identity, 'validate_api_environment'), \
                mock.patch.object(identity, 'require_robot_identity', side_effect=ValueError('mismatch')), \
                mock.patch.object(session, '_run') as run:
            with self.assertRaisesRegex(clock.RobotClockError, 'identity'):
                session._check_identity()
        run.assert_not_called()

    def test_api_refuses_mismatched_token_before_any_process(self):
        session = operator.OperatorSession(transport='api')
        session.token = {'verified': True, 'transport': 'dds'}
        with mock.patch.object(identity, 'validate_api_environment'), \
                mock.patch.object(identity, 'require_robot_identity', return_value={}), \
                mock.patch.object(operator.subprocess, 'Popen') as popen, \
                mock.patch.object(operator, 'RosOperator') as ros:
            with self.assertRaisesRegex(RuntimeError, 'clock token'):
                session.start(.05)
        popen.assert_not_called()
        ros.assert_not_called()


if __name__ == '__main__':
    unittest.main()
