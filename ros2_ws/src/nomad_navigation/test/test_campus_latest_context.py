"""Request-aligned JPEG context tests; no ROS, camera, model or robot needed."""
import json
import unittest
from unittest.mock import patch

from nomad_navigation.campus_client_core import (
    CampusClientCore, FAULT_TIMING_LIMIT, HISTORY_MAX_BYTES, HISTORY_MAX_FRAMES,
    HISTORY_WINDOW_NS, REQUEST_DEADLINE_SEC, SENSOR_GAP_NS, SOURCE_TIMEOUT_NS,
    WAYPOINT_TIMEOUT_SEC,
)
from nomad_navigation.campus_protocol import validate_raw_request as validate_request


JPEG = b'\xff\xd8example-image-envelope\xff\xd9'
BASE = 10_000_000_000


def add(core, stamp, now=None, jpeg=JPEG):
    now = stamp if now is None else now
    return core.add_camera(jpeg, stamp, 'camera_color_optical_frame', now, now/1e9)


def make_response(request):
    paths = [[[.1*(step+1), .01] for step in range(8)] for _ in range(8)]
    return dict(version=1, session_id=request['session_id'], sequence=request['sequence'],
                observation_stamp_ns=request['frames'][-1]['stamp_ns'], frame_id='base_link',
                state='running', waypoint=paths[0][2][:], trajectories=paths,
                inference_ms=150., waypoint_scale=.05)


def ready(core):
    for offset in (0, 250_000_000, 500_000_000, 750_000_000):
        add(core, BASE+offset)
    return core.request(BASE+750_000_000, 10.75)


class LatestContextTests(unittest.TestCase):
    def test_30hz_history_anchors_at_latest_frame_not_old_4hz_phase(self):
        core = CampusClientCore()
        stamps = [BASE + round(i*1e9/30) for i in range(35)]
        for stamp in stamps:
            self.assertTrue(add(core, stamp, stamp+15_000_000))
        now = BASE+1_165_000_000
        request = core.request(now, now/1e9)
        validate_request(request)
        selected = [frame['stamp_ns'] for frame in request['frames']]
        self.assertEqual(selected[-1], stamps[-1])
        self.assertLessEqual(now-selected[-1], 50_000_000)
        for previous, current in zip(selected, selected[1:]):
            self.assertGreaterEqual(current-previous, 225_000_000)
            self.assertLessEqual(current-previous, 267_000_000)
        self.assertEqual(core.pending['context_stamps_ns'], tuple(selected))
        # The old arrival-phase sampler would still be anchored at raw #28.
        self.assertGreater(now-stamps[28], 200_000_000)

    def test_exact_250ms_context_remains_unchanged(self):
        core = CampusClientCore()
        request = ready(core)
        self.assertEqual([frame['stamp_ns'] for frame in request['frames']],
                         [BASE, BASE+250_000_000, BASE+500_000_000, BASE+750_000_000])

    def test_selection_prefers_250ms_over_minimum_225ms_gap(self):
        core = CampusClientCore()
        for offset in (0, 250, 275, 500, 525, 750):
            add(core, BASE+offset*1_000_000)
        selected = core.select_context(BASE+750_000_000)
        self.assertEqual([frame[0] for frame in selected],
                         [BASE, BASE+250_000_000, BASE+500_000_000, BASE+750_000_000])

    def test_bounded_search_avoids_nearest_predecessor_dead_end(self):
        core = CampusClientCore()
        for offset in (200, 950, 1700, 1750, 2000):
            add(core, BASE+offset*1_000_000)
        request = core.request(BASE+2_000_000_000, 12.)
        validate_request(request)
        self.assertEqual([frame['stamp_ns']-BASE for frame in request['frames']],
                         [200_000_000, 950_000_000, 1_700_000_000, 2_000_000_000])

    def test_four_close_frames_cannot_substitute_for_model_context(self):
        core = CampusClientCore()
        for offset in (0, 30, 60, 90):
            add(core, BASE+offset*1_000_000)
        self.assertIsNone(core.request(BASE+90_000_000, 10.09))
        self.assertEqual(core.sequence, 0)

    def test_protocol_maximum_context_span_is_preserved(self):
        core = CampusClientCore()
        for offset in (0, 750, 1500, 2250):
            add(core, BASE+offset*1_000_000)
        request = core.request(BASE+2_250_000_000, 12.25)
        validate_request(request)
        self.assertEqual(request['frames'][-1]['stamp_ns']-request['frames'][0]['stamp_ns'],
                         HISTORY_WINDOW_NS)

    def test_future_frame_within_existing_tolerance_is_never_selected_early(self):
        core = CampusClientCore()
        for offset in (0, 250, 500, 750):
            add(core, BASE+offset*1_000_000)
        self.assertTrue(add(core, BASE+900_000_000, BASE+800_000_000))
        selected = core.select_context(BASE+800_000_000)
        self.assertEqual(selected[-1][0], BASE+750_000_000)
        request = core.request(BASE+800_000_000, 10.8)
        self.assertTrue(all(frame['stamp_ns'] <= request['observation_now_ns'] for frame in request['frames']))

    def test_new_anchor_is_used_when_future_frame_becomes_nonfuture(self):
        core = CampusClientCore()
        for offset in (0, 250, 500, 750):
            add(core, BASE+offset*1_000_000)
        add(core, BASE+900_000_000, BASE+800_000_000)
        selected = core.select_context(BASE+900_000_000)
        self.assertEqual(selected[-1][0], BASE+900_000_000)

    def test_no_anchor_is_reused_without_new_sensor_stamp(self):
        core = CampusClientCore()
        request = ready(core)
        core.complete(make_response(request), None, 10.9, BASE+900_000_000, 10.9)
        self.assertIsNone(core.request(BASE+950_000_000, 10.95))

    def test_sub_225ms_callback_is_retained_and_can_anchor_the_next_request(self):
        core = CampusClientCore()
        request = ready(core)
        self.assertTrue(add(core, BASE+800_000_000))
        core.complete(make_response(request), None, 10.9, BASE+900_000_000, 10.9)
        request2 = core.request(BASE+950_000_000, 10.95)
        self.assertEqual(request2['frames'][-1]['stamp_ns'], BASE+800_000_000)
        self.assertEqual(request2['sequence'], 2)
        validate_request(request2)

    def test_pending_is_frozen_while_new_history_arrives(self):
        core = CampusClientCore()
        request = ready(core)
        frozen = dict(core.pending)
        for offset in (800, 850, 900, 950, 1000):
            add(core, BASE+offset*1_000_000)
        self.assertIsNone(core.request(BASE+1_000_000_000, 11.))
        self.assertEqual(core.pending, frozen)
        self.assertEqual(request['frames'][-1]['stamp_ns'], BASE+750_000_000)
        result = core.complete(make_response(request), None, 11.05, BASE+1_050_000_000, 11.05)
        self.assertEqual(result['observation_stamp_ns'], BASE+750_000_000)
        next_request = core.request(BASE+1_050_000_000, 11.05)
        self.assertEqual(next_request['frames'][-1]['stamp_ns'], BASE+1_000_000_000)

    def test_old_response_cannot_match_new_pending_context(self):
        core = CampusClientCore()
        request = ready(core)
        core.complete(make_response(request), None, 10.9, BASE+900_000_000, 10.9)
        add(core, BASE+1_000_000_000)
        core.request(BASE+1_000_000_000, 11.)
        self.assertIsNone(core.complete(make_response(request), None, 11.1, BASE+1_100_000_000, 11.1))
        self.assertTrue(core.fatal)
        self.assertEqual(core.state, 'identity_or_protocol_error')


class HistoryBoundsTests(unittest.TestCase):
    def test_count_limit_holds_even_with_high_rate_burst(self):
        core = CampusClientCore()
        for index in range(HISTORY_MAX_FRAMES+40):
            add(core, BASE+index*1_000_000)
            self.assertLessEqual(len(core.frames), HISTORY_MAX_FRAMES)
            self.assertEqual(core.history_bytes, sum(len(frame[1]) for frame in core.frames))
        self.assertEqual(len(core.frames), HISTORY_MAX_FRAMES)

    def test_byte_limit_is_independent_of_count_limit(self):
        core = CampusClientCore()
        with patch('nomad_navigation.campus_client_core.HISTORY_MAX_BYTES', 3*len(JPEG)):
            for index in range(10):
                add(core, BASE+index*30_000_000)
                self.assertLessEqual(core.history_bytes, 3*len(JPEG))
            self.assertEqual(len(core.frames), 3)
            self.assertEqual(core.history_bytes, 3*len(JPEG))

    def test_time_span_is_bounded_for_slow_camera(self):
        core = CampusClientCore()
        for index in range(30):
            add(core, BASE+index*250_000_000)
            self.assertLessEqual(core.frames[-1][0]-core.frames[0][0], HISTORY_WINDOW_NS)
        self.assertLess(len(core.frames), HISTORY_MAX_FRAMES)
        self.assertLessEqual(core.history_bytes, HISTORY_MAX_BYTES)

    def test_fault_clear_resets_both_frames_and_byte_accounting(self):
        core = CampusClientCore()
        ready(core)
        self.assertGreater(core.history_bytes, 0)
        core.fail('camera_gap', clear=True)
        self.assertEqual(core.history_bytes, 0)
        self.assertFalse(core.frames)
        add(core, BASE+1_000_000_000)
        self.assertEqual(core.history_bytes, len(JPEG))

    def test_callback_age_metric_now_covers_all_valid_retained_frames(self):
        core = CampusClientCore()
        for offset in (0, 33, 66, 100):
            add(core, BASE+offset*1_000_000, BASE+(offset+15)*1_000_000)
        snapshot = core.snapshot()
        self.assertEqual(snapshot['valid_camera_samples'], 4)
        self.assertEqual(snapshot['camera_callback_age_ms']['count'], 4)
        self.assertEqual(snapshot['camera_callback_age_ms']['mean'], 15.)
        self.assertEqual(snapshot['camera_age_sampling'], 'all_timestamp_valid_history_callbacks')


class UnchangedSafetyAndFaultTimingTests(unittest.TestCase):
    def test_all_reviewed_deadlines_unchanged(self):
        self.assertEqual(REQUEST_DEADLINE_SEC, .8)
        self.assertEqual(SOURCE_TIMEOUT_NS, 1_000_000_000)
        self.assertEqual(WAYPOINT_TIMEOUT_SEC, .6)
        self.assertEqual(SENSOR_GAP_NS, 750_000_000)

    def test_watchdog_stale_captures_old_prediction_and_pending_request(self):
        core = CampusClientCore(latch_faults=True)
        first = ready(core)
        add(core, BASE+1_000_000_000)
        add(core, BASE+1_250_000_000)
        core.complete(make_response(first), None, 11.29, BASE+1_300_000_000, 11.3)
        second = core.request(BASE+1_300_000_000, 11.3)
        add(core, BASE+1_500_000_000)
        add(core, BASE+1_750_000_000)
        core.watchdog(BASE+1_760_000_000, 11.76)
        timing = core.snapshot()['fault_timing'][-1]
        self.assertEqual(timing['origin'], 'watchdog_previous_prediction')
        self.assertEqual(timing['reason'], 'stale_result')
        self.assertEqual(timing['pending_sequence'], second['sequence'])
        self.assertEqual(timing['last_result_stamp_ns'], first['frames'][-1]['stamp_ns'])
        self.assertAlmostEqual(timing['previous_source_age_ms'], 1010.)
        self.assertTrue(core.fatal)
        self.assertIsNone(core.complete(make_response(second), None, 11.749,
                                        BASE+1_760_000_000, 11.76))

    def test_incoming_stale_is_distinct_from_old_prediction_watchdog(self):
        core = CampusClientCore()
        for offset in (0, 250, 500, 750):
            add(core, BASE+offset*1_000_000)
        request = core.request(BASE+1_200_000_000, 11.2)
        add(core, BASE+1_300_000_000)
        self.assertIsNone(core.complete(make_response(request), None, 11.75,
                                        BASE+1_800_000_000, 11.8))
        timing = core.snapshot()['fault_timing'][-1]
        self.assertEqual(timing['origin'], 'incoming_result')
        self.assertEqual(timing['reason'], 'stale_result')
        self.assertAlmostEqual(timing['incoming_source_age_ms'], 1050.)
        self.assertAlmostEqual(timing['completed_result_wait_ms'], 50.)
        self.assertIsNone(timing['last_result_stamp_ns'])

    def test_camera_gap_replay_and_clock_faults_still_clear_and_latch(self):
        for kind in ('gap', 'replay', 'clock'):
            core = CampusClientCore(latch_faults=True)
            ready(core)
            if kind == 'gap':
                add(core, BASE+1_600_000_000)
            elif kind == 'replay':
                add(core, BASE+500_000_000, BASE+800_000_000)
            else:
                core.watchdog(BASE+700_000_000, 10.8)
            self.assertTrue(core.fatal)
            self.assertEqual(core.history_bytes, 0)
            self.assertFalse(core.frames)
            self.assertIsNone(core.request(BASE+2_000_000_000, 12.))

    def test_fault_timing_metadata_is_bounded_and_json_finite(self):
        core = CampusClientCore()
        core.clock(BASE, 10.)
        for index in range(FAULT_TIMING_LIMIT+20):
            core.fail('network_error' if index % 2 else 'stale_result')
        snapshot = core.snapshot()
        self.assertEqual(len(snapshot['fault_timing']), FAULT_TIMING_LIMIT)
        self.assertEqual(snapshot['fault_timing_total'], FAULT_TIMING_LIMIT+20)
        self.assertEqual(snapshot['fault_timing_limit'], FAULT_TIMING_LIMIT)
        raw = json.dumps(snapshot, allow_nan=False)
        self.assertNotIn('jpeg_b64', raw)
        self.assertNotIn('Authorization', raw)
        snapshot['fault_timing'][0]['reason'] = 'mutated'
        self.assertNotEqual(core.snapshot()['fault_timing'][0]['reason'], 'mutated')


if __name__ == '__main__':
    unittest.main()
