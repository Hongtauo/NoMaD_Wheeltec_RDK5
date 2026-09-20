"""Standard-library configuration checks; no ROS, Docker or robot required."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).with_name('robot_live_view.py')
SPEC = importlib.util.spec_from_file_location('robot_live_view', SCRIPT)
view = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(view)
ROOT = SCRIPT.resolve().parents[2]
PROFILE = ROOT / 'docker/ros2/robot_rviz_udp.xml'
LIVE_PROFILE = ROOT / 'docker/ros2/robot_live_udp.xml'


class LiveViewGuards(unittest.TestCase):
    def control_status(self, mode='timed', remaining=5.0):
        return {'time_ns': 10_000_000_000, 'armed': mode is not None, 'active_mode': mode,
                'requested_duration_sec': 5.0 if mode == 'timed' else 0.0 if mode else None,
                'lease_remaining_sec': remaining if mode == 'timed' else None if mode else 0.,
                'run_until_disarmed': mode == 'continuous'}

    def test_private_topic_mapping(self):
        params = view.VISUALIZATION_PARAMETERS
        self.assertEqual(params['frame_id'], 'base_link')
        self.assertEqual(params['waypoint_topic'], '/nomad/robot_preview/waypoint_stamped')
        self.assertEqual(params['trajectory_topic'], '/nomad/robot_preview/trajectory_poses')
        self.assertEqual(params['model_status_topic'], '/nomad/robot_preview/status')
        self.assertEqual(params['controller_status_topic'], '/nomad/robot_control/status')
        self.assertEqual(params['velocity_topic'], '/nomad/robot_control/cmd_vel_preview')
        self.assertEqual(params['marker_topic'], '/nomad/robot_preview/visualization')
        self.assertNotIn('/cmd_vel', params.values())

    def test_live_label_does_not_claim_measured_motion(self):
        text = view.live_status_text(True, (.05, .02), True, 'running', 'armed')
        self.assertIn('LIVE | NO ODOMETRY', text)
        self.assertIn('command v=0.050', text)
        self.assertIn('not measured motion', text)
        self.assertNotIn('LOCAL PREVIEW ONLY', text)

    def test_stale_velocity_is_not_shown_as_current(self):
        text = view.live_status_text(False, (2., 3.), False, 'faulted', 'stopped')
        self.assertIn('NO FRESH PREDICTION', text)
        self.assertIn('command v=0.000 m/s  w=0.000 rad/s [command stale]', text)
        self.assertIn('control: stopped', text)

    def test_mode_label_is_included_without_claiming_measured_motion(self):
        text = view.live_status_text(True, (.05, .02), True, 'running', 'tracking', 'CONTINUOUS')
        self.assertIn('run: CONTINUOUS', text)
        self.assertIn('not measured motion', text)

    def test_timed_status_deducts_source_and_receipt_age_not_remote_deadline(self):
        status = self.control_status()
        status['arm_deadline_monotonic'] = 9999999.0
        self.assertEqual(view.control_mode_text(status, 100.0, 10_200_000_000, 100.3),
                         'TIMED 4.7 s remaining')
        self.assertEqual(view.control_mode_text(status, 100.0, 10_400_000_000, 100.1),
                         'TIMED 4.6 s remaining')

    def test_continuous_and_disarmed_are_reported_only_for_fresh_status(self):
        for mode, expected in (('continuous', 'CONTINUOUS'), (None, 'DISARMED')):
            status = self.control_status(mode)
            self.assertEqual(view.control_mode_text(status, 100., 10_000_000_000, 100.), expected)
            self.assertTrue(view.control_mode_text(status, 100., 10_000_000_000, 100.61).startswith('UNKNOWN'))
            self.assertTrue(view.control_mode_text(status, 100., 10_610_000_000, 100.).startswith('UNKNOWN'))

    def test_mode_is_unknown_on_future_source_or_receipt_clock_rollback(self):
        status = self.control_status('continuous')
        for now, mono in ((9_700_000_000, 100.0), (10_000_000_000, 99.9)):
            self.assertTrue(view.control_mode_text(status, 100., now, mono).startswith('UNKNOWN'))

    def test_timed_deadline_expiry_is_not_claimed_as_still_armed_or_stopped(self):
        status = self.control_status(remaining=.1)
        text = view.control_mode_text(status, 100., 10_200_000_000, 100.2)
        self.assertEqual(text, 'UNKNOWN [timed status expired]')
        self.assertNotIn('DISARMED', text)

    def test_malformed_or_inconsistent_status_cannot_claim_armed_mode(self):
        valid = self.control_status()
        variants = [None, [], {}, {**valid, 'armed': 'true'}, {**valid, 'armed': 1},
                    {**valid, 'active_mode': 'unknown'}, {**valid, 'time_ns': 10.0},
                    {**valid, 'time_ns': 10**400},
                    {**valid, 'requested_duration_sec': math.inf},
                    {**valid, 'requested_duration_sec': 10.01},
                    {**valid, 'lease_remaining_sec': math.nan},
                    {**valid, 'lease_remaining_sec': 6.0},
                    {**valid, 'run_until_disarmed': True},
                    {**self.control_status('continuous'), 'requested_duration_sec': 1.0},
                    {**self.control_status('continuous'), 'lease_remaining_sec': 1.0},
                    {**self.control_status('continuous'), 'run_until_disarmed': False},
                    {**self.control_status(None), 'lease_remaining_sec': 5.0},
                    {**self.control_status(None), 'run_until_disarmed': True}]
        for status in variants:
            with self.subTest(status=status):
                self.assertTrue(view.control_mode_text(status, 100., 10_000_000_000, 100.).startswith('UNKNOWN'))

    def test_separate_rviz_ports_and_explicit_loopback_wifi_announcement(self):
        result = view.validate_rviz_profile(PROFILE)
        self.assertEqual(result['participant_id'], 11)
        self.assertEqual((result['metatraffic_port'], result['data_port']), (7432, 7433))
        self.assertEqual(result['announced_addresses'], ['127.0.0.1', '192.168.0.180'])
        self.assertEqual(result['initial_peers'], ['127.0.0.1:7430', '192.168.0.100:standard-discovery'])

    def test_live_inference_ports_and_reciprocal_loopback_peer(self):
        result = view.validate_live_profile(LIVE_PROFILE)
        self.assertEqual(result['participant_id'], 10)
        self.assertEqual((result['metatraffic_port'], result['data_port']), (7430, 7431))
        self.assertEqual(result['announced_addresses'], ['127.0.0.1', '192.168.0.180'])
        self.assertEqual(result['initial_peers'], ['127.0.0.1:7432', '192.168.0.100:standard-discovery'])
        with self.assertRaises(ValueError):
            view.validate_live_profile(PROFILE)
        with self.assertRaises(ValueError):
            view.validate_rviz_profile(LIVE_PROFILE)

    def test_loopback_required_in_both_announced_lists(self):
        for profile, validate, data_port, meta_port in (
            (PROFILE, view.validate_rviz_profile, 7433, 7432),
            (LIVE_PROFILE, view.validate_live_profile, 7431, 7430),
        ):
            original = profile.read_text(encoding='utf-8')
            for port in (data_port, meta_port):
                entry = f'<locator><udpv4><address>127.0.0.1</address><port>{port}</port></udpv4></locator>'
                variant = original.replace(entry, '', 1)
                self.assertNotEqual(variant, original)
                with self.subTest(profile=profile.name, port=port), tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / 'profile.xml'
                    path.write_text(variant, encoding='utf-8')
                    with self.assertRaises(ValueError):
                        validate(path)

    def test_reject_unreviewed_locator_variants(self):
        original = PROFILE.read_text(encoding='utf-8')
        variants = [original.replace('192.168.0.180', '192.168.65.3'),
                    original.replace('192.168.0.180', '0.0.0.0'),
                    original.replace('<port>7433</port>', '<port>7431</port>'),
                    original.replace('<participantID>11</participantID>', '<participantID>10</participantID>'),
                    original.replace('<type>UDPv4</type>', '<type>UDPv4</type><interfaceWhiteList/>'),
                    original.replace('<builtin>', '<default_external_unicast_locators/><builtin>'),
                    original.replace('<useBuiltinTransports>false', '<useBuiltinTransports>true'),
                    original.replace('<port>7430</port>', '<port>7432</port>')]
        for index, variant in enumerate(variants):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'profile.xml'
                path.write_text(variant, encoding='utf-8')
                with self.assertRaises(ValueError):
                    view.validate_rviz_profile(path)

    def test_live_profile_rejects_automatic_internal_or_external_locators(self):
        original = LIVE_PROFILE.read_text(encoding='utf-8')
        variants = [original.replace('192.168.0.180', '192.168.65.3'),
                    original.replace('127.0.0.1', '0.0.0.0'),
                    original.replace('<port>7431</port>', '<port>7433</port>'),
                    original.replace('<builtin>', '<default_external_unicast_locators/><builtin>'),
                    original.replace('<type>UDPv4</type>', '<type>UDPv4</type><interfaceWhiteList/>')]
        for index, variant in enumerate(variants):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'profile.xml'
                path.write_text(variant, encoding='utf-8')
                with self.assertRaises(ValueError):
                    view.validate_live_profile(path)

    def test_factory_does_not_initialize_ros_or_publish_motion(self):
        source = SCRIPT.read_text(encoding='utf-8')
        self.assertNotIn('rclpy.init(', source)
        self.assertNotIn('create_publisher(', source)
        self.assertNotIn('TransformBroadcaster', source)
        self.assertIn('use_global_arguments=False', source)


if __name__ == '__main__':
    unittest.main()
