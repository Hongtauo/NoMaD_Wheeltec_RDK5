"""Pure runner/launcher checks: no ROS, Docker, GPU or physical robot needed."""

import contextlib
import io
from pathlib import Path
import unittest
from unittest import mock

import run_robot_live as live


ROOT = Path(__file__).resolve().parents[2]


class LiveRunnerTests(unittest.TestCase):
    def test_default_and_bounds(self):
        self.assertEqual(live.parse_args([]).seconds, 600)
        self.assertEqual(live.parse_args(['--seconds', '0']).seconds, 0)
        for value in ('nan', 'inf', '-1', '9', '3601'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                live.parse_args(['--seconds', value])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            live.parse_args(['--ros-args', '-r', '/waypoint:=/cmd_vel'])

    def test_continuous_session_has_no_deadline(self):
        self.assertTrue(live.session_active(1000000, 0))
        self.assertTrue(live.session_active(9, 10))
        self.assertFalse(live.session_active(10, 10))
        # Old single-participant runner stays explicitly bounded.
        with self.assertRaises(Exception):
            live.core.duration('0')

    def test_recording_space_stops_without_deleting_results(self):
        with mock.patch.object(live.shutil, 'disk_usage', return_value=mock.Mock(free=2 * 1024 ** 3)):
            live.require_recording_space(ROOT)
        with mock.patch.object(live.shutil, 'disk_usage', return_value=mock.Mock(free=2 * 1024 ** 3 - 1)):
            with self.assertRaisesRegex(RuntimeError, 'less than 2 GiB'):
                live.require_recording_space(ROOT)

    def test_only_known_robot_control_consumers(self):
        known = [('robot_control', '/nomad')]
        live.validate_existing_subscribers(live.core.OUTPUTS['stamped_waypoint_topic'], known)
        live.validate_existing_subscribers(live.core.OUTPUTS['status_topic'], known)
        live.validate_existing_subscribers(live.core.OUTPUTS['trajectory_poses_topic'], [])
        for topic, identities in [
            (live.core.OUTPUTS['waypoint_topic'], known),
            (live.core.OUTPUTS['stamped_waypoint_topic'], [('other', '/nomad')]),
            (live.core.OUTPUTS['status_topic'], [('robot_control', '/')]),
        ]:
            with self.subTest(topic=topic, identities=identities), self.assertRaises(RuntimeError):
                live.validate_existing_subscribers(topic, identities)

    def test_rviz_uses_separate_reviewed_profile(self):
        env = live.rviz_environment(ROOT, ROOT / 'outputs/example', {'ROS_DISCOVERY_SERVER': 'wrong:1'})
        self.assertTrue(env['FASTRTPS_DEFAULT_PROFILES_FILE'].endswith('robot_rviz_udp.xml'))
        self.assertEqual(env['FASTDDS_DEFAULT_PROFILES_FILE'], env['FASTRTPS_DEFAULT_PROFILES_FILE'])
        self.assertEqual(env['ROS_DOMAIN_ID'], '0')
        self.assertEqual(env['ROS_LOCALHOST_ONLY'], '0')
        self.assertNotIn('ROS_DISCOVERY_SERVER', env)
        self.assertEqual(live.validate_rviz_profile(ROOT / 'docker/ros2/robot_rviz_udp.xml')['participant_id'], 11)

    def test_live_environment_requires_new_profile_not_old_preview_profile(self):
        new_profile = ROOT / 'docker/ros2/robot_live_udp.xml'
        old_profile = ROOT / 'docker/ros2/fastdds_extlab/robot_wifi_only.xml'
        env = {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0',
               'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
               'FASTRTPS_DEFAULT_PROFILES_FILE': str(new_profile)}
        result = live.validate_environment(ROOT, env)
        self.assertEqual(result['locators']['announced_addresses'], ['127.0.0.1', '192.168.0.180'])
        for key, value in (('FASTRTPS_DEFAULT_PROFILES_FILE', str(old_profile)),
                           ('FASTDDS_DEFAULT_PROFILES_FILE', str(old_profile)),
                           ('ROS_DOMAIN_ID', '86'), ('ROS_LOCALHOST_ONLY', '1'),
                           ('ROS_DISCOVERY_SERVER', 'wrong:1'), ('SKIP_DEFAULT_XML', '1')):
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                live.validate_environment(ROOT, {**env, key: value})
        # Do not relax the earlier single-participant inference validator.
        old_env = {**env, 'FASTRTPS_DEFAULT_PROFILES_FILE': str(old_profile)}
        live.core.validate_environment(ROOT, old_env)
        with self.assertRaises(RuntimeError):
            live.core.validate_environment(ROOT, env)

    def test_only_owned_rviz_is_signalled(self):
        process = mock.Mock(pid=12345)
        process.poll.return_value = None
        with mock.patch.object(live.os, 'killpg', create=True) as kill, \
                mock.patch.object(live.signal, 'SIGKILL', 9, create=True):
            live.stop_rviz(process)
        kill.assert_called_once_with(12345, live.signal.SIGINT)
        process.wait.assert_called_once_with(timeout=5.0)

    def test_exited_rviz_is_not_signalled(self):
        process = mock.Mock()
        process.poll.return_value = 0
        with mock.patch.object(live.os, 'killpg', create=True) as kill:
            live.stop_rviz(process)
        kill.assert_not_called()

    def test_launcher_is_dedicated_and_not_arming(self):
        text = (ROOT / 'docker/ros2/run_robot_live.ps1').read_text(encoding='utf-8')
        self.assertIn("$containerName = 'nomad-robot-runtime'", text)
        self.assertIn("'--network', 'host', '--gpus', 'all', '--shm-size=8g'", text)
        self.assertIn('docker exec -it', text)
        self.assertNotIn('docker rm', text)
        self.assertNotIn('docker stop', text)
        self.assertNotIn('ros2 service call', text)
        self.assertIn("Join-Path $PSScriptRoot 'robot_live_udp.xml'", text)
        self.assertIn('FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/SocialEmbodiedAgent/docker/ros2/robot_live_udp.xml', text)
        self.assertNotIn('fastdds_extlab/robot_wifi_only.xml', text)
        self.assertIn("Where-Object { $_ -ne '127.0.0.1' }", text)


if __name__ == '__main__':
    unittest.main()
