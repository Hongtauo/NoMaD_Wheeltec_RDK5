"""Guard bridge lifecycle tests; no ROS initialization or robot/network I/O."""
import os
from pathlib import Path
import tempfile
import time
import unittest
import uuid
from unittest import mock

import robot_operator as operator


class ReconnectBridgeTests(unittest.TestCase):
    def session(self):
        session = operator.OperatorSession(transport='api')
        session.enable_reconnect(str(uuid.uuid4()))
        return session

    def test_only_fresh_api_session_can_enable_guard(self):
        with self.assertRaises(ValueError):
            operator.OperatorSession().enable_reconnect(str(uuid.uuid4()))
        session = self.session()
        with self.assertRaises(ValueError):
            session.enable_reconnect(str(uuid.uuid4()))

    def test_non_guarded_arm_cannot_bypass_supervisor_ticket(self):
        session = self.session()
        session.running = True
        session.ros = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'one-shot'):
            session.dispatch(dict(op='arm', mode=1, duration_sec=0., confirmed=True))
        session.ros.arm.assert_not_called()

    def test_link_expiry_is_original_and_never_extended_by_local_tick(self):
        session = self.session()
        session.running, session.launcher, session.ros = True, mock.Mock(), mock.Mock()
        session.launcher.poll.return_value = None
        session.last_contact = 0.
        with mock.patch.object(operator.time, 'monotonic_ns', return_value=10_000_000_000):
            session.operator_link_update(1, 11_000_000_000, True)
        session.ros.publish_link.reset_mock()
        session.last_link_publish = 0.
        with mock.patch.object(operator.time, 'monotonic', return_value=100.), \
                mock.patch.object(session, 'owned_processes', return_value={}):
            session.tick()  # Even long after expiry: do not issue a new 2-second lease.
        session.ros.publish_link.assert_called_once_with(1, 11_000_000_000, True)
        self.assertTrue(session.running)  # Supervisor, not legacy 4s path, owns cleanup.

    def test_disconnect_revokes_without_stopping_or_restoring_clock(self):
        session = self.session()
        session.ros, session.clock = mock.Mock(), mock.Mock()
        session.running = True
        now = time.monotonic_ns()
        session.operator_link_update(3, now + 1_000_000_000, True)
        with mock.patch.object(session, 'stop_owned_processes') as stop:
            session.operator_link_disarm('transport_eof')
        self.assertFalse(session.operator_connected)
        self.assertTrue(session.running)
        stop.assert_not_called()
        session.clock.restore.assert_not_called()
        call = session.ros.publish_link.call_args
        self.assertEqual(call.args[0], 3)
        self.assertIs(call.args[2], False)
        self.assertEqual(call.kwargs['action'], 'disarm')

    def test_arm_timeout_revokes_and_does_not_retry(self):
        session = self.session()
        session.running, session.ros = True, mock.Mock()
        deadline = time.monotonic_ns() + 1_500_000_000
        session.operator_link_update(4, deadline, True)
        session.ros.arm_guarded.side_effect = RuntimeError('reply uncertain')
        with self.assertRaisesRegex(RuntimeError, 'uncertain'):
            session.arm_guarded(dict(mode=1, duration_sec=0., confirmed=True),
                                4, deadline, str(uuid.uuid4()))
        session.ros.arm_guarded.assert_called_once()
        self.assertFalse(session.operator_connected)
        self.assertEqual(session.ros.publish_link.call_args.kwargs['action'], 'disarm')

    def test_opt_in_pause_repeats_original_deadline_without_shutdown_or_arm(self):
        session = self.session()
        session.running, session.operator_pause_resume = True, True
        session.ros, session.launcher, session.clock = mock.Mock(), mock.Mock(), mock.Mock()
        session.launcher.poll.return_value = None
        original_deadline = time.monotonic_ns() + 1_000_000_000
        session.operator_link_update(5, original_deadline, True)
        session.operator_link_pause('transport_eof')
        self.assertEqual(session.operator_deadline_ns, original_deadline)
        self.assertFalse(session.operator_connected)
        self.assertTrue(session.running)
        session.ros.publish_link.assert_called_with(5, original_deadline, False, action='pause')
        session.last_link_publish = 0.
        with mock.patch.object(operator.time, 'monotonic', return_value=100.), \
                mock.patch.object(session, 'owned_processes', return_value={}):
            session.tick()
        session.ros.publish_link.assert_called_with(5, original_deadline, False, action='pause')
        session.clock.restore.assert_not_called()
        session.ros.arm_guarded.assert_not_called()
        session.operator_link_disarm('manual_stop')
        self.assertEqual(session.operator_link_action, 'disarm')
        session.operator_link_update(6, time.monotonic_ns() + 1_000_000_000, True)
        self.assertEqual(session.operator_link_action, 'heartbeat')
        session.ros.arm_guarded.assert_not_called()

    def test_pause_without_opt_in_or_before_running_is_hard_disarm(self):
        for enabled, running in ((False, True), (True, False), (False, False)):
            session = self.session()
            session.ros = mock.Mock()
            session.operator_pause_resume, session.running = enabled, running
            session.operator_link_update(2, time.monotonic_ns() + 1_000_000_000, True)
            session.operator_link_pause('transport_eof')
            self.assertEqual(session.operator_link_action, 'disarm')
            self.assertEqual(session.ros.publish_link.call_args.kwargs['action'], 'disarm')

    def test_start_dispatch_propagates_explicit_pause_policy(self):
        session = self.session()
        with mock.patch.object(session, 'start') as start:
            session.dispatch(dict(op='start', max_linear_mps=.1,
                                  network_pause_resume=True, operator_pause_resume=True))
        start.assert_called_once_with(.1, network_pause_resume=True, operator_pause_resume=True)

    def test_operator_pause_requires_boolean_supervision_and_network_pause(self):
        for transport, supervised, network, enabled in (
                ('api', True, True, 1), ('api', True, True, 'true'),
                ('api', True, True, None), ('api', True, False, True),
                ('api', False, True, True), ('dds', False, False, True)):
            session = operator.OperatorSession(transport)
            if supervised:
                session.enable_reconnect(str(uuid.uuid4()))
            with self.subTest(transport=transport, supervised=supervised,
                              network=network, enabled=enabled), \
                    mock.patch.object(operator.subprocess, 'Popen') as popen, \
                    self.assertRaises(ValueError):
                session.start(.05, network_pause_resume=network, operator_pause_resume=enabled)
            popen.assert_not_called()

    def test_start_policy_environment_is_explicit_and_result_matches(self):
        from nomad_navigation import robot_identity as identity
        for enabled in (False, True):
            session = self.session()
            session.token = {'transport': 'api', 'robot_identity': {'test': True},
                             'session_id': str(uuid.uuid4())}
            launcher = mock.Mock(pid=12345)
            with self.subTest(enabled=enabled), tempfile.TemporaryDirectory() as directory, \
                    mock.patch.object(operator, 'ROBOT_ROOT', Path(directory)), \
                    mock.patch.object(identity, 'validate_api_environment'), \
                    mock.patch.object(identity, 'require_robot_identity', return_value={'test': True}), \
                    mock.patch.object(identity, 'read_campus_client_config'), \
                    mock.patch.object(session, 'require_pidfd'), \
                    mock.patch.object(operator, 'RosOperator'), \
                    mock.patch.dict(os.environ, {'NOMAD_OPERATOR_PAUSE_RESUME': 'inherited-unsafe'}), \
                    mock.patch.object(operator.subprocess, 'Popen', return_value=launcher) as popen, \
                    mock.patch.object(Path, 'read_text', return_value='1 (test) ' + ' '.join(['0']*19+['42'])):
                try:
                    result = session.start(.05, network_pause_resume=True,
                                           operator_pause_resume=enabled)
                    env = popen.call_args.kwargs['env']
                    self.assertEqual(env.get('NOMAD_OPERATOR_PAUSE_RESUME'), '1' if enabled else None)
                    self.assertEqual(env['NOMAD_OPERATOR_GUARD_SESSION'], session.reconnect_session_id)
                    self.assertIs(result['operator_pause_resume'], enabled)
                    self.assertIs(session.operator_pause_resume, enabled)
                    self.assertFalse(result['automatic_arm'])
                finally:
                    if session.launch_log:
                        session.launch_log.close()

    def test_wrong_generation_and_expired_lease_do_not_send_arm(self):
        session = self.session()
        session.running, session.ros = True, mock.Mock()
        deadline = time.monotonic_ns() + 1_000_000_000
        session.operator_link_update(4, deadline, True)
        for generation, until in ((3, deadline), (4, deadline + 1), (5, deadline)):
            with self.assertRaises(RuntimeError):
                session.arm_guarded({}, generation, until, str(uuid.uuid4()))
        with mock.patch.object(operator.time, 'monotonic_ns', return_value=deadline):
            with self.assertRaises(RuntimeError):
                session.arm_guarded({}, 4, deadline, str(uuid.uuid4()))
        session.ros.arm_guarded.assert_not_called()

    def test_startup_refuses_old_controller_without_guard_when_supervised(self):
        from nomad_navigation.robot_startup_check import StartupObservation
        identity = str(uuid.uuid4())
        observation = StartupObservation('0.05', identity)
        observation.accept_status('control', {'armed': False}, time.monotonic())
        self.assertIn('authorization guard', observation.failure)


if __name__ == '__main__':
    unittest.main()
