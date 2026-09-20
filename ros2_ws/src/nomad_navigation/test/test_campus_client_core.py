"""Hardware-free standard-library tests for the campus robot client."""
import copy
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import MagicMock, patch

from nomad_navigation.campus_client_core import (
    CampusClientCore, FatalClientError, HTTPResponseError, HTTPSInference, OneRequestWorker,
    METRIC_WINDOW_LIMIT, REQUEST_DEADLINE_SEC, metric_summary, read_token, server_address,
)
from nomad_navigation.campus_api_client import parser, require_local_ros, require_control_session
from nomad_navigation.robot_control_core import RobotControlCore


JPEG = b'\xff\xd8tiny-test-envelope\xff\xd9'


def response(request):
    paths = [[[.1*(step+1), .01] for step in range(8)] for _ in range(8)]
    return dict(version=1, session_id=request['session_id'], sequence=request['sequence'],
                observation_stamp_ns=request['frames'][-1]['stamp_ns'], frame_id='base_link',
                state='running', waypoint=paths[0][2][:], trajectories=paths,
                inference_ms=150.0, waypoint_scale=.05)


class CampusCoreTests(unittest.TestCase):
    def setUp(self):
        self.core = CampusClientCore()

    def camera(self, at, stamp=None, **kwargs):
        return self.core.add_camera(kwargs.get('jpeg', JPEG),
                                    round((at if stamp is None else stamp)*1e9),
                                    kwargs.get('frame', 'camera_color_optical_frame'),
                                    round(at*1e9), at)

    def ready(self):
        for at in (10., 10.25, 10.5, 10.75):
            self.assertTrue(self.camera(at))
        return self.core.request(10_750_000_000, 10.75)

    def test_exact_four_frame_context_and_original_stamps(self):
        request = self.ready()
        self.assertEqual([f['stamp_ns'] for f in request['frames']],
                         [10_000_000_000, 10_250_000_000, 10_500_000_000, 10_750_000_000])
        self.assertEqual(request['observation_now_ns'], 10_750_000_000)
        result = self.core.complete(response(request), None, 10.9, 10_900_000_000, 10.9)
        self.assertEqual(result['observation_stamp_ns'], 10_750_000_000)
        self.assertEqual(self.core.state, 'running')

    def test_three_frames_not_ready(self):
        for at in (10., 10.25, 10.5):
            self.camera(at)
        self.assertIsNone(self.core.request(10_500_000_000, 10.5))

    def test_request_aligned_context_uses_new_raw_frame_without_reusing_anchor(self):
        request = self.ready()
        self.assertTrue(self.camera(10.8))
        self.core.complete(response(request), None, 10.9, 10_900_000_000, 10.9)
        recent_request = self.core.request(10_950_000_000, 10.95)
        self.assertEqual(recent_request['frames'][-1]['stamp_ns'], 10_800_000_000)
        self.core.complete(response(recent_request), None, 10.99, 10_990_000_000, 10.99)
        self.assertIsNone(self.core.request(10_995_000_000, 10.995))
        self.assertTrue(self.camera(11.0))
        next_request = self.core.request(11_000_000_000, 11.)
        self.assertEqual(next_request['sequence'], 3)
        self.assertEqual(next_request['frames'][0]['stamp_ns'], 10_250_000_000)

    def test_single_inflight_no_backlog(self):
        self.ready()
        for at in (11., 11.25, 11.5):
            self.camera(at)
        self.assertIsNone(self.core.request(11_500_000_000, 11.5))

    def test_old_result_after_camera_discontinuity_is_discarded(self):
        request = self.ready()
        self.assertFalse(self.camera(10.8, stamp=10.5))
        self.assertIsNone(self.core.complete(response(request), None, 10.9, 10_900_000_000, 10.9))
        self.assertEqual(self.core.state, 'context_invalidated')

    def test_wrong_camera_frame_and_invalid_jpeg_clear_context(self):
        for kwargs in ({'frame': 'wrong'}, {'jpeg': b'invalid'}, {'jpeg': JPEG * 100000}):
            with self.subTest(kwargs=list(kwargs)):
                self.core = CampusClientCore()
                self.ready()
                self.assertFalse(self.camera(10.8, **kwargs))
                self.assertEqual(len(self.core.frames), 0)

    def test_camera_future_and_stale_rejected(self):
        for stamp in (8.999, 10.21):
            self.core = CampusClientCore()
            self.assertFalse(self.camera(10., stamp=stamp))

    def test_camera_gap_resets_context(self):
        self.ready()
        self.camera(11.6)
        self.assertEqual(len(self.core.frames), 1)
        self.assertNotEqual(self.core.state, 'running')

    def test_sensor_arrival_watchdog(self):
        self.ready()
        self.core.watchdog(11_501_000_000, 11.501)
        self.assertEqual(self.core.state, 'camera_gap')
        self.assertEqual(len(self.core.frames), 0)

    def test_waypoint_watchdog_independent_of_inflight_worker(self):
        request = self.ready()
        self.core.complete(response(request), None, 10.9, 10_900_000_000, 10.9)
        self.camera(11.0)
        self.core.request(11_000_000_000, 11.)
        self.camera(11.25)
        self.camera(11.5)
        self.core.watchdog(11_501_000_000, 11.501)
        self.assertEqual(self.core.state, 'waypoint_timeout')

    def test_result_source_stamp_expiry(self):
        for at in (10., 10.25, 10.5, 10.75):
            self.camera(at)
        request = self.core.request(11_200_000_000, 11.2)
        self.camera(11.3)
        self.assertIsNone(self.core.complete(response(request), None, 11.8, 11_800_000_000, 11.8))
        self.assertEqual(self.core.state, 'stale_result')

    def test_total_deadline_and_delayed_executor_reject(self):
        for completed, observed in ((11.55, 11.55), (10.9, 11.55)):
            self.core = CampusClientCore()
            request = self.ready()
            self.camera(11.)
            self.camera(11.25)
            self.camera(11.5)
            self.assertIsNone(self.core.complete(response(request), None, completed,
                                                  round(observed*1e9), observed))
            self.assertEqual(self.core.state, 'request_timeout')

    def test_clock_rollback_latched_until_restart(self):
        request = self.ready()
        self.core.watchdog(10_700_000_000, 10.8)
        self.assertEqual(self.core.state, 'clock_rollback')
        self.assertTrue(self.core.fatal)
        self.assertFalse(self.camera(11.))
        self.assertIsNone(self.core.complete(response(request), None, 11., 11_000_000_000, 11.))

    def test_invalid_clock_values_fail_closed(self):
        for now, steady in ((0, 10), (10, math.nan), (10, -1)):
            core = CampusClientCore()
            self.assertFalse(core.clock(now, steady))
            self.assertTrue(core.fatal)

    def test_identity_sequence_stamp_frame_scale_fail_fatal(self):
        mutations = (('session_id', '00000000-0000-0000-0000-000000000001'),
                     ('sequence', 2), ('observation_stamp_ns', 10_900_000_000),
                     ('frame_id', 'odom'), ('waypoint_scale', 1.0), ('state', 'ready'))
        for field, value in mutations:
            self.core = CampusClientCore()
            request = self.ready()
            output = response(request)
            output[field] = value
            self.assertIsNone(self.core.complete(output, None, 10.9, 10_900_000_000, 10.9))
            self.assertTrue(self.core.fatal, field)

    def test_malformed_path_and_waypoint_rejected(self):
        for alteration in ('nan', 'short', 'mismatch'):
            self.core = CampusClientCore()
            request = self.ready()
            output = response(request)
            if alteration == 'nan':
                output['trajectories'][1][0][0] = math.nan
            elif alteration == 'short':
                output['trajectories'].pop()
            else:
                output['waypoint'][0] += 1
            self.assertIsNone(self.core.complete(output, None, 10.9, 10_900_000_000, 10.9))
            self.assertTrue(self.core.fatal)

    def test_network_error_recovers_predictions_but_never_arm(self):
        self.ready()
        self.core.complete(None, TimeoutError(), 10.9, 10_900_000_000, 10.9)
        self.assertEqual(self.core.state, 'network_error')
        self.assertFalse(self.core.fatal)
        self.camera(11.)
        request = self.core.request(11_000_000_000, 11.)
        self.assertIsNotNone(self.core.complete(response(request), None, 11.15, 11_150_000_000, 11.15))
        self.assertFalse(self.core.snapshot()['auto_arm'])

    def test_fatal_authentication_does_not_retry(self):
        self.ready()
        self.core.complete(None, FatalClientError('auth'), 10.9, 10_900_000_000, 10.9)
        self.assertTrue(self.core.fatal)
        self.assertIsNone(self.core.request(11_000_000_000, 11.))

    def test_reconnect_running_status_cannot_rearm_existing_controller(self):
        controller = RobotControlCore()
        now, steady = 10_000_000_000, 10.
        controller.set_model_state('running')
        controller.accept_camera(now, 'camera_color_optical_frame', now, steady)
        controller.accept_waypoint(.2, .01, 0., now, 'base_link', now, steady)
        controller.accept_odom((0., 0., 0.), (0., 0., 0.), now,
                               'odom', 'base_footprint', now, steady)
        self.assertTrue(controller.arm(now, steady)[0])
        self.assertTrue(controller.armed)
        self.core.fail('network_error')
        controller.set_model_state(self.core.snapshot()['state'])
        self.assertFalse(controller.armed)
        request = self.ready()
        self.core.complete(response(request), None, 10.9, 10_900_000_000, 10.9)
        controller.set_model_state(self.core.snapshot()['state'])
        self.assertFalse(controller.armed)

    def test_telemetry_is_optional_not_a_motion_command(self):
        self.ready()
        self.core.pending = None
        self.camera(11.)
        request = self.core.request(11_000_000_000, 11., {'controller_status': {'armed': False}})
        self.assertFalse(request['telemetry']['controller_status']['armed'])
        self.assertNotIn('command', request)


class ClientConfigurationTests(unittest.TestCase):
    def test_only_numeric_https_and_no_credentials_or_proxy(self):
        self.assertEqual(server_address('https://10.201.91.120:8443'), ('10.201.91.120', 8443))
        self.assertEqual(server_address('https://[::1]:8443'), ('::1', 8443))
        for url in ('http://10.0.0.1', 'https://example.org', 'https://u:p@10.0.0.1',
                    'https://10.0.0.1/anything', 'https://10.0.0.1/?x=1', 'https://0.0.0.0'):
            with self.assertRaises(ValueError):
                server_address(url)

    def test_default_shadow_cli(self):
        args = parser().parse_args(['--server-url', 'https://10.0.0.1:8443', '--ca-file', 'ca',
                                    '--token-file', 'token', '--output-root', 'out'])
        self.assertFalse(args.control_output)
        self.assertEqual(args.seconds, 0)

    def test_local_ros_environment_guard(self):
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'}):
            require_local_ros()
        for updates in ({'ROS_DOMAIN_ID': '3'}, {'ROS_LOCALHOST_ONLY': '0'}):
            with patch.dict(os.environ, updates):
                with self.assertRaises(ValueError):
                    require_local_ros()

    def test_control_output_requires_existing_robot_session_once(self):
        with patch('nomad_navigation.robot_session_guard.require_robot_session') as guard:
            self.assertIsNone(require_control_session(False))
            guard.assert_not_called()
            require_control_session(True)
            guard.assert_called_once_with()

    def test_secret_file_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'token'
            target.write_text('a' * 43, encoding='ascii')
            target.chmod(0o600)
            self.assertEqual(read_token(target), 'a'*43)
            target.write_text('bad\r\nAuthorization: exposed', encoding='ascii')
            with self.assertRaises(ValueError):
                read_token(target)


class CampusMetricsTests(unittest.TestCase):
    setUp = CampusCoreTests.setUp
    camera = CampusCoreTests.camera
    ready = CampusCoreTests.ready

    def test_accepted_metric_values_have_explicit_millisecond_units(self):
        request = self.ready()
        self.core.complete(response(request), None, 10.9, 10_950_000_000, 10.95)
        snapshot = self.core.snapshot()
        expected = dict(request_elapsed_ms=200., worker_elapsed_ms=150.,
                        source_age_ms=200., observation_age_at_request_ms=0., inference_ms=150.)
        self.assertEqual(snapshot['accepted_results'], 1)
        self.assertEqual(snapshot['rejected_results'], 0)
        for name, value in expected.items():
            self.assertEqual(snapshot['accepted_metrics'][name]['count'], 1)
            for statistic in ('min', 'mean', 'p95', 'max'):
                self.assertAlmostEqual(snapshot['accepted_metrics'][name][statistic], value)
        json.dumps(snapshot, allow_nan=False)

    def test_no_accepted_result_reports_null_not_nonfinite_metrics(self):
        snapshot = self.core.snapshot()
        for values in snapshot['accepted_metrics'].values():
            self.assertEqual(values, dict(count=0, min=None, mean=None, p95=None, max=None))
        json.dumps(snapshot, allow_nan=False)

    def test_summary_nearest_rank_p95_and_finite_filtering(self):
        values = list(range(1, 101)) + [math.nan, math.inf, -math.inf, True]
        self.assertEqual(metric_summary(values), dict(count=100, min=1., mean=50.5, p95=95., max=100.))
        self.assertTrue(math.isfinite(metric_summary([1e308, 1e308])['mean']))

    def test_metric_windows_are_bounded_and_lifetime_acceptance_separate(self):
        pending = dict(started=10., stamp_ns=10_000_000_000, observation_age_at_request_ms=0.)
        for index in range(METRIC_WINDOW_LIMIT+17):
            self.core._record_accepted(pending, {'inference_ms': float(index % 300)}, 10.1,
                                       10_200_000_000, 10.2)
        for values in self.core.accepted_metrics.values():
            self.assertEqual(len(values), METRIC_WINDOW_LIMIT)
        snapshot = self.core.snapshot()
        self.assertEqual(snapshot['accepted_metrics_window_limit'], METRIC_WINDOW_LIMIT)
        self.assertEqual(snapshot['accepted_results'], 0)  # Audit helper cannot affect the state machine.
        self.assertEqual(snapshot['accepted_metrics']['inference_ms']['count'], METRIC_WINDOW_LIMIT)

    def test_statistics_are_cached_until_next_acceptance_and_not_externally_mutable(self):
        request = self.ready()
        self.core.complete(response(request), None, 10.9, 10_900_000_000, 10.9)
        first = self.core.snapshot()
        first['accepted_metrics']['inference_ms']['count'] = -1
        with patch('nomad_navigation.campus_client_core.metric_summary', side_effect=AssertionError('re-sort')):
            self.assertEqual(self.core.snapshot()['accepted_metrics']['inference_ms']['count'], 1)

    def test_timeout_error_audited_without_changing_original_network_error_state(self):
        self.ready()
        self.core.complete(None, TimeoutError(), 10.9, 10_900_000_000, 10.9)
        self.assertEqual(self.core.state, 'network_error')
        snapshot = self.core.snapshot()
        self.assertEqual(snapshot['rejection_counts'], {'request_timeout': 1})
        self.assertEqual(snapshot['rejected_results'], 1)
        self.assertEqual(snapshot['fault_events'], {'network_error': 1})

    def test_completed_results_count_once_independent_of_repeated_watchdog(self):
        request = self.ready()
        self.camera(11.)
        self.camera(11.25)
        self.camera(11.5)
        self.core.watchdog(11_550_000_000, 11.55)
        self.core.watchdog(11_560_000_000, 11.56)
        self.assertEqual(self.core.snapshot()['fault_events']['request_timeout'], 1)
        self.assertEqual(self.core.snapshot()['rejected_results'], 0)
        self.core.complete(response(request), None, 11.56, 11_560_000_000, 11.56)
        self.assertEqual(self.core.snapshot()['rejection_counts'], {'request_timeout': 1})

    def test_network_and_protocol_errors_have_separate_rejection_reasons(self):
        for error, expected in ((ConnectionError(), 'network_error'),
                                (FatalClientError(), 'identity_or_protocol_error')):
            self.core = CampusClientCore()
            self.ready()
            self.core.complete(None, error, 10.9, 10_900_000_000, 10.9)
            self.assertEqual(self.core.snapshot()['rejection_counts'], {expected: 1})
            self.assertEqual(self.core.snapshot()['accepted_metrics']['inference_ms']['count'], 0)

    def test_http_status_rejections_are_not_called_wifi_disconnects(self):
        for status in (408, 429, 500, 503, 504):
            for formal in (False, True):
                self.core = CampusClientCore(latch_faults=formal)
                self.ready()
                error = HTTPResponseError(status)
                self.assertIsInstance(error, ConnectionError)
                self.core.complete(None, error, 10.9, 10_900_000_000, 10.9)
                snapshot = self.core.snapshot()
                self.assertEqual(snapshot['rejection_counts'], {'http_%d' % status: 1})
                self.assertEqual(snapshot['last_http_status'], status)
                self.assertEqual(snapshot['last_error_type'], 'HTTPResponseError')
                self.assertEqual(snapshot['state'], 'network_error')
                self.assertEqual(snapshot['fatal'], formal)
                self.assertEqual(snapshot['accepted_results'], 0)
                self.assertEqual(snapshot['fault_events'], {'network_error': 1})

    def test_plain_connection_failures_do_not_invent_an_http_status(self):
        self.ready()
        self.core.complete(None, ConnectionError(), 10.9, 10_900_000_000, 10.9)
        snapshot = self.core.snapshot()
        self.assertEqual(snapshot['rejection_counts'], {'network_error': 1})
        self.assertIsNone(snapshot['last_http_status'])

    def test_http_error_retains_only_bounded_numeric_status(self):
        self.assertEqual(str(HTTPResponseError(504)), 'Inference service returned HTTP 504')
        for invalid in (None, True, '504', 99, 600):
            with self.assertRaises(ValueError):
                HTTPResponseError(invalid)

    def test_context_invalidated_and_stale_rejections_audited(self):
        request = self.ready()
        self.camera(10.8, stamp=10.5)
        self.core.complete(response(request), None, 10.9, 10_900_000_000, 10.9)
        self.assertEqual(self.core.snapshot()['rejection_counts'], {'context_invalidated': 1})
        self.core = CampusClientCore()
        for at in (10., 10.25, 10.5, 10.75):
            self.camera(at)
        request = self.core.request(11_200_000_000, 11.2)
        self.camera(11.3)
        self.core.complete(response(request), None, 11.8, 11_800_000_000, 11.8)
        self.assertEqual(self.core.snapshot()['rejection_counts'], {'stale_result': 1})

    def test_camera_age_tracks_all_valid_history_callbacks(self):
        self.camera(10., stamp=9.9)
        self.camera(10.05, stamp=9.95)  # Retained even though <225 ms after the previous frame.
        self.camera(10.25, stamp=10.1)  # Eligible history; request selection decides actual context.
        self.camera(10.3, stamp=10.2)
        self.camera(10.4, stamp=10.7)  # Invalid future stamp, excluded.
        self.camera(10.5, jpeg=b'bad')  # Invalid JPEG, excluded.
        snapshot = self.core.snapshot()
        self.assertEqual(snapshot['valid_camera_samples'], 4)
        self.assertEqual(snapshot['camera_callback_age_ms'],
                         dict(count=4, min=100., mean=112.5, p95=150., max=150.))
        self.assertEqual(snapshot['camera_age_sampling'], 'all_timestamp_valid_history_callbacks')

    def test_request_age_and_transport_age_explain_total_source_age(self):
        for at in (10., 10.25, 10.5, 10.75):
            self.camera(at, stamp=at-.1)
        request = self.core.request(10_800_000_000, 10.8)
        self.core.complete(response(request), None, 10.95, 11_000_000_000, 11.)
        metrics = self.core.snapshot()['accepted_metrics']
        self.assertAlmostEqual(metrics['observation_age_at_request_ms']['mean'], 150.)
        self.assertAlmostEqual(metrics['request_elapsed_ms']['mean'], 200.)
        self.assertAlmostEqual(metrics['source_age_ms']['mean'], 350.)


class FormalFaultLatchTests(unittest.TestCase):
    camera = CampusCoreTests.camera
    ready = CampusCoreTests.ready

    def setUp(self):
        self.core = CampusClientCore(latch_faults=True)

    def test_startup_context_and_normal_stop_are_not_faults(self):
        self.assertEqual(self.core.state, 'waiting_context')
        self.assertFalse(self.core.fatal)
        self.core.fail('waiting_context')
        self.assertFalse(self.core.fatal)
        self.ready()
        self.assertFalse(self.core.fatal)
        self.core.fail('stopped', clear=True)
        self.assertFalse(self.core.fatal)
        self.assertEqual(self.core.state, 'stopped')

    def test_any_real_fault_latches_until_process_restart(self):
        for reason in ('invalid_camera', 'camera_replay', 'camera_discontinuity', 'camera_gap',
                       'request_timeout', 'stale_result', 'waypoint_timeout', 'network_error',
                       'context_invalidated', 'competing_model_publisher', 'worker_unavailable'):
            with self.subTest(reason=reason):
                self.core = CampusClientCore(latch_faults=True)
                request = self.ready()
                self.core.fail(reason)
                self.assertTrue(self.core.fatal)
                self.assertEqual(self.core.state, reason)
                self.assertIsNone(self.core.complete(response(request), None, 10.9,
                                                      10_900_000_000, 10.9))
                self.assertFalse(self.camera(11.))
                self.assertIsNone(self.core.request(11_000_000_000, 11.))
                self.core.fail('stopped', clear=True)
                self.assertEqual(self.core.state, reason)
                self.assertTrue(self.core.fatal)
                self.assertEqual(self.core.accepted_results, 0)

    def test_discontinuity_does_not_overwrite_latched_fault_with_waiting_context(self):
        self.ready()
        self.assertFalse(self.camera(11.6))
        self.assertTrue(self.core.fatal)
        self.assertEqual(self.core.state, 'camera_discontinuity')
        self.assertEqual(len(self.core.frames), 0)

    def test_same_tick_stale_and_already_ready_response_cannot_return_to_running(self):
        first = self.ready()
        self.camera(11.)
        self.camera(11.25)
        self.core.complete(response(first), None, 11.29, 11_300_000_000, 11.3)
        second = self.core.request(11_300_000_000, 11.3)
        self.camera(11.5)
        self.camera(11.75)
        # Response is complete before the previous source expires, but was
        # not published/accepted before the watchdog observes that expiry.
        self.core.watchdog(11_760_000_000, 11.76)
        self.assertEqual(self.core.state, 'stale_result')
        self.assertTrue(self.core.fatal)
        self.assertIsNone(self.core.complete(response(second), None, 11.749,
                                              11_760_000_000, 11.76))
        self.assertEqual(self.core.state, 'stale_result')
        self.assertEqual(self.core.accepted_results, 1)
        self.assertEqual(self.core.snapshot()['rejection_counts'], {'stale_result': 1})

    def test_network_error_fails_formal_session_but_shadow_can_continue(self):
        for formal in (False, True):
            self.core = CampusClientCore(latch_faults=formal)
            self.ready()
            self.core.complete(None, ConnectionError(), 10.9, 10_900_000_000, 10.9)
            self.assertEqual(self.core.fatal, formal)
            self.assertEqual(self.core.state, 'network_error')

    def test_latch_configuration_must_not_accept_truthy_strings(self):
        for value in ('False', 0, 1, None):
            with self.assertRaises(ValueError):
                CampusClientCore(latch_faults=value)

    def test_cli_fatal_fault_exits_nonzero_and_runs_cleanup(self):
        from nomad_navigation.campus_api_client import main
        fake_node = types.SimpleNamespace(core=CampusClientCore(latch_faults=True), events=[],
                                          started=time.monotonic(), close=MagicMock(return_value=True),
                                          destroy_node=MagicMock())
        fake_ros = types.SimpleNamespace(init=MagicMock(), ok=MagicMock(return_value=True),
                                        shutdown=MagicMock(),
                                        spin_once=lambda *a, **kw: fake_node.core.fail('network_error'))
        fake_signals = types.SimpleNamespace(SignalHandlerOptions=types.SimpleNamespace(NO=0))
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict('sys.modules', {'rclpy': fake_ros, 'rclpy.signals': fake_signals}), \
                    patch('nomad_navigation.campus_api_client.require_local_ros'), \
                    patch('nomad_navigation.campus_api_client.require_control_session') as session, \
                    patch('nomad_navigation.campus_api_client.HTTPSInference') as transport_factory, \
                    patch('nomad_navigation.campus_api_client.CompactUploadTransport', side_effect=lambda value: value), \
                    patch('nomad_navigation.campus_api_client.create_node', return_value=fake_node), \
                    patch('builtins.print'):
                transport_factory.return_value.snapshot.return_value = {}
                code = main(['--server-url', 'https://127.0.0.1:8443', '--ca-file', 'ca',
                             '--token-file', 'token', '--output-root', directory, '--control-output'])
            self.assertEqual(code, 1)
            session.assert_called_once_with(True)
            fake_node.close.assert_called_once_with()
            fake_node.destroy_node.assert_called_once_with()
            report = json.loads(next(Path(directory).glob('campus_client__*/report.json')).read_text())
            self.assertTrue(report['fatal'])
            self.assertTrue(report['latch_faults'])
            self.assertEqual(report['state'], 'network_error')
            self.assertEqual(report['exit_code'], 1)
            self.assertFalse(report['auto_arm'])


class WorkerTests(unittest.TestCase):
    def test_single_inflight_and_bounded_shutdown(self):
        released = threading.Event()
        class Transport:
            def infer(self, request, started=None):
                released.wait(5)
                return {'sequence': request['sequence']}
            def cancel(self):
                released.set()
        worker = OneRequestWorker(Transport())
        self.assertTrue(worker.submit({'sequence': 1}, time.monotonic()))
        self.assertFalse(worker.submit({'sequence': 2}, time.monotonic()))
        started = time.monotonic()
        self.assertTrue(worker.close())
        self.assertLess(time.monotonic()-started, 1.)
        self.assertFalse(worker.submit({'sequence': 3}, time.monotonic()))
        self.assertEqual(worker.take()[1], {'sequence': 1})

    def test_worker_transports_exceptions_without_blocking_caller(self):
        class Transport:
            def infer(self, request, started=None):
                raise TimeoutError('deadline')
            def cancel(self):
                pass
        worker = OneRequestWorker(Transport())
        self.assertTrue(worker.submit({'sequence': 1}, time.monotonic()))
        worker._thread.join(timeout=.2)
        self.assertIsInstance(worker.take()[2], TimeoutError)
        self.assertIsNone(worker.take())


if __name__ == '__main__':
    unittest.main()
