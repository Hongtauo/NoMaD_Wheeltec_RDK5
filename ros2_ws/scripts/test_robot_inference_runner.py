"""Pure standard-library guard/evidence tests; no ROS, Docker or robot required."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).with_name('run_robot_inference.py')
SPEC = importlib.util.spec_from_file_location('robot_runner', SCRIPT)
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)
ROOT = SCRIPT.resolve().parents[2]
PROFILE = ROOT / 'docker/ros2/fastdds_extlab/robot_wifi_only.xml'


class RunnerGuards(unittest.TestCase):
    def test_duration_bounds_and_remaps(self):
        for value in ('nan', 'inf', '9', '3601'):
            with self.assertRaises(Exception):
                runner.duration(value)
        self.assertEqual(runner.duration('10'), 10)
        self.assertEqual(runner.duration('3600'), 3600)
        with self.assertRaises(SystemExit):
            runner.parse_args(['--ros-args', '-r', '/waypoint:=/cmd_vel'])

    def test_reviewed_profile(self):
        self.assertEqual(runner.validate_profile(PROFILE)[0], {'address': '192.168.0.180', 'port': 7431})

    def test_reject_unsafe_profile_variants(self):
        original = PROFILE.read_text(encoding='utf-8')
        variants = [original.replace('192.168.0.180', '192.168.65.3'),
                    original.replace('<port>7431</port>', '<port>7432</port>'),
                    original.replace('<participantID>10</participantID>', '<participantID>11</participantID>'),
                    original.replace('<type>UDPv4</type>', '<type>UDPv4</type><interfaceWhiteList/>'),
                    original.replace('<builtin>', '<default_external_unicast_locators/><builtin>'),
                    original.replace('<useBuiltinTransports>false', '<useBuiltinTransports>true')]
        for index, variant in enumerate(variants):
            with self.subTest(index=index), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'profile.xml'
                path.write_text(variant, encoding='utf-8')
                with self.assertRaises(RuntimeError):
                    runner.validate_profile(path)

    def test_environment_and_conflict(self):
        env = {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0',
               'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'FASTRTPS_DEFAULT_PROFILES_FILE': str(PROFILE)}
        runner.validate_environment(ROOT, env)
        for key, value in [('ROS_DOMAIN_ID', '1'), ('ROS_LOCALHOST_ONLY', '1'),
                           ('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
                           ('ROS_DISCOVERY_SERVER', '192.168.0.100:11811'),
                           ('FASTDDS_DEFAULT_PROFILES_FILE', '/wrong.xml')]:
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                runner.validate_environment(ROOT, {**env, key: value})

    def test_port_failure_closes_all_sockets(self):
        allocated = []

        class FakeSocket:
            def __init__(self, *args):
                self.closed = False
                allocated.append(self)

            def bind(self, address):
                if address[1] == 7431:
                    raise OSError('address in use')

            def close(self):
                self.closed = True

        with self.assertRaisesRegex(RuntimeError, '7431 is occupied'):
            runner.require_free_ports(socket_factory=FakeSocket)
        self.assertEqual(len(allocated), 2)
        self.assertTrue(all(sock.closed for sock in allocated))

    def test_finite_metrics(self):
        self.assertEqual(runner.distribution([])['count'], 0)
        stats = runner.distribution([1, 2, 3, 4, 5])
        self.assertEqual(stats['mean'], 3)
        self.assertEqual(stats['p95'], 5)

    def test_output_liveness_startup_grace(self):
        runner.require_output_liveness(now=104.99, ready_at=100.0, last_point_at=None)
        runner.require_output_liveness(now=105.0, ready_at=100.0, last_point_at=None)

    def test_output_liveness_never_produced(self):
        with self.assertRaisesRegex(RuntimeError, 'No actual DDS preview waypoint'):
            runner.require_output_liveness(now=105.001, ready_at=100.0, last_point_at=None)

    def test_output_liveness_stalled(self):
        with self.assertRaisesRegex(RuntimeError, 'output stalled'):
            runner.require_output_liveness(now=110.0, ready_at=100.0, last_point_at=107.999)

    def test_output_liveness_recent_and_exact_boundary(self):
        runner.require_output_liveness(now=110.0, ready_at=100.0, last_point_at=109.8)
        runner.require_output_liveness(now=110.0, ready_at=100.0, last_point_at=108.0)

    def test_actual_output_allowlist(self):
        graph = [(topic, ['std_msgs/msg/String']) for topic in runner.OUTPUTS.values()]
        runner.validate_publisher_graph(graph)
        with self.assertRaises(RuntimeError):
            runner.validate_publisher_graph(graph + [('/cmd_vel', ['geometry_msgs/msg/Twist'])])
        with self.assertRaises(RuntimeError):
            runner.validate_publisher_graph([(topic, ['geometry_msgs/msg/Twist']) for topic, _ in graph])
        with self.assertRaises(RuntimeError):
            runner.validate_publisher_graph(graph[:-1])

    def test_actual_dds_points_match_saved_rows(self):
        points = [{'stamp_ns': 123, 'frame_id': 'base_link', 'xyz': [1.0, 2.0, 0.0]}]
        rows = [{'observation_stamp_ns': 123, 'waypoint': [1.0, 2.0]}]
        runner.match_points_to_rows(points, rows)
        for change in ({'stamp_ns': 124}, {'frame_id': 'map'}, {'xyz': [math.nan, 2, 0]},
                       {'xyz': [2, 2, 0]}):
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                runner.match_points_to_rows([{**points[0], **change}], rows)
        with self.assertRaises(RuntimeError):
            runner.match_points_to_rows([], rows)
        with self.assertRaises(RuntimeError):
            runner.match_points_to_rows(points * 2, rows * 2)

    def test_parameters_are_private_and_keep_freshness_guards(self):
        params = runner.model_parameters(ROOT, ROOT / 'outputs/example')
        self.assertTrue(all(params[key].startswith('/nomad/robot_preview/') for key in runner.OUTPUTS))
        self.assertEqual(params['image_transport'], 'compressed')
        self.assertEqual(params['image_timeout_sec'], 1.0)
        self.assertEqual(params['future_tolerance_sec'], .2)
        self.assertEqual(params['device'], 'cuda')
        self.assertNotIn('Twist', json.dumps(params))


if __name__ == '__main__':
    unittest.main()
