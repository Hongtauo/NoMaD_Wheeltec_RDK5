"""Pure hardware-free control tests; runnable with Python's standard unittest."""

from dataclasses import replace
import json
import math
import unittest

from nomad_navigation.robot_control_core import (
    ARM_MODE_CONTINUOUS, ARM_MODE_TIMED, RobotControlCore,
    RobotControlLimits, ackermann_velocity,
)


class RobotControlTests(unittest.TestCase):
    def setUp(self):
        self.core = RobotControlCore()
        self.base_ns = 10_000_000_000

    def fresh(self, at=10.0, x=.2, y=.03, speed=0.0, turn=0.0, waypoint=True, camera=True, odom=True):
        now = round(at * 1e9)
        self.core.set_model_state('running')
        if camera:
            self.assertTrue(self.core.accept_camera(now, 'camera_color_optical_frame', now, at))
        if waypoint:
            self.assertTrue(self.core.accept_waypoint(x, y, 0.0, now, 'base_link', now, at))
        if odom:
            self.assertTrue(self.core.accept_odom((speed, 0.0, 0.0), (0.0, 0.0, turn),
                                                  now, 'odom', 'base_footprint', now, at))
        return now

    def arm(self, at=10.0):
        now = self.fresh(at)
        self.assertTrue(self.core.arm(now, at)[0])

    def test_default_is_disarmed_even_with_fresh_inputs(self):
        now = self.fresh()
        self.assertEqual(self.core.tick(now, 10.0)[:2], (0.0, 0.0))
        self.assertFalse(self.core.armed)
        self.assertIsNone(self.core.arm_deadline)
        self.assertIsNone(self.core.active_mode)
        self.assertIsNone(self.core.requested_duration_sec)

    def test_arm_requires_every_fresh_input_and_running_model(self):
        self.assertFalse(self.core.arm(self.base_ns, 10.0)[0])
        self.fresh(waypoint=False)
        self.assertFalse(self.core.arm(self.base_ns, 10.0)[0])
        self.core.accept_waypoint(.2, 0.0, 0.0, self.base_ns, 'base_link', self.base_ns, 10.0)
        self.core.set_model_state('faulted')
        self.assertFalse(self.core.arm(self.base_ns, 10.0)[0])

    def test_arm_refuses_moving_robot(self):
        for speed, turn in ((.011, 0.0), (0.0, .031)):
            with self.subTest(speed=speed, turn=turn):
                self.core = RobotControlCore()
                now = self.fresh(speed=speed, turn=turn)
                self.assertEqual(self.core.arm(now, 10.0), (False, 'robot_not_stationary'))

    def test_arm_cannot_extend_live_lease(self):
        self.arm()
        deadline = self.core.arm_deadline
        self.assertFalse(self.core.arm(self.base_ns + 100_000_000, 10.1)[0])
        self.assertEqual(self.core.arm_deadline, deadline)
        self.assertEqual(deadline, 13.0)

    def test_acceleration_and_curvature_apply_to_actual_speed(self):
        self.arm()
        previous = 0.0
        for step in range(1, 41):
            at = 10.0 + step * .05
            now = self.fresh(at)
            speed, turn, reason = self.core.tick(now, at)
            self.assertEqual(reason, 'tracking')
            self.assertGreaterEqual(speed, 0.0)
            self.assertLessEqual(speed, .05)
            self.assertLessEqual(speed - previous, .025 * .05 + 1e-12)
            self.assertLessEqual(abs(turn), speed / 1.0 + 1e-12)
            self.assertLessEqual(abs(turn), .1)
            previous = speed
        self.assertAlmostEqual(previous, .05)

    def test_lease_expires_and_fresh_inputs_cannot_rearm(self):
        self.arm()
        for step in range(1, 61):
            at = 10.0 + step * .05
            now = self.fresh(at)
            command = self.core.tick(now, at)
        self.assertEqual(command[:2], (0.0, 0.0))
        self.assertEqual(self.core.reason, 'arm_lease_expired')
        self.assertFalse(self.core.armed)
        now = self.fresh(13.05)
        self.assertEqual(self.core.tick(now, 13.05)[:2], (0.0, 0.0))

    def test_explicit_disarm_always_succeeds_and_stops_immediately(self):
        self.arm()
        self.assertGreater(self.core.tick(self.base_ns + 50_000_000, 10.05)[0], 0.0)
        self.assertTrue(self.core.disarm('operator_disarmed'))
        self.assertEqual(self.core.last_linear, 0.0)
        self.assertEqual(self.core.tick(self.base_ns + 60_000_000, 10.06)[:2], (0.0, 0.0))
        self.core.gate.clock_fault = True
        self.assertTrue(self.core.disarm())

    def test_estop_release_requires_fresh_waypoint_and_explicit_arm(self):
        self.arm()
        self.core.set_estop(True)
        self.assertFalse(self.core.armed)
        self.assertEqual(self.core.last_linear, 0.0)
        self.assertFalse(self.core.arm(self.base_ns, 10.0)[0])
        self.core.set_estop(False)
        self.assertFalse(self.core.armed)
        self.assertIsNone(self.core.gate.waypoint)
        self.assertFalse(self.core.arm(self.base_ns, 10.0)[0])
        now = self.fresh(10.05)
        self.assertEqual(self.core.tick(now, 10.05)[:2], (0.0, 0.0))
        self.assertTrue(self.core.arm(now, 10.05)[0])

    def test_model_fault_cancels_arm_without_automatic_recovery(self):
        self.arm()
        self.core.set_model_state('faulted')
        self.assertFalse(self.core.armed)
        now = self.fresh(10.05)
        self.assertEqual(self.core.tick(now, 10.05)[:2], (0.0, 0.0))

    def test_waypoint_receipt_ttl_cancels_arm(self):
        self.arm()
        for step in range(1, 14):
            at = 10.0 + step * .05
            now = self.fresh(at, waypoint=False)
            command = self.core.tick(now, at)
        self.assertEqual(command[:2], (0.0, 0.0))
        self.assertEqual(self.core.reason, 'waypoint_timeout')
        self.assertFalse(self.core.armed)

    def test_camera_timeout_cancels_arm(self):
        self.arm()
        for step in range(1, 23):
            at = 10.0 + step * .05
            now = self.fresh(at, camera=False)
            command = self.core.tick(now, at)
        self.assertEqual(command[:2], (0.0, 0.0))
        self.assertEqual(self.core.reason, 'camera_timeout')
        self.assertFalse(self.core.armed)

    def test_odom_timeout_cancels_arm(self):
        self.arm()
        for step in range(1, 12):
            at = 10.0 + step * .05
            now = self.fresh(at, odom=False)
            command = self.core.tick(now, at)
        self.assertEqual(command[:2], (0.0, 0.0))
        self.assertEqual(self.core.reason, 'odom_timeout')
        self.assertFalse(self.core.armed)

    def test_odom_speed_anomaly_immediately_disarms(self):
        for linear, angular in (((.121, 0., 0.), (0., 0., 0.)),
                                ((0., .121, 0.), (0., 0., 0.)),
                                ((0., 0., 0.), (0., 0., .301)),
                                ((math.nan, 0., 0.), (0., 0., 0.))):
            with self.subTest(linear=linear, angular=angular):
                self.core = RobotControlCore()
                self.arm()
                self.assertFalse(self.core.accept_odom(linear, angular, self.base_ns + 1,
                                                       'odom', 'base_footprint', self.base_ns + 1, 10.01))
                self.assertFalse(self.core.armed)
                self.assertEqual(self.core.last_linear, 0.0)

    def test_odom_replay_wrong_frames_stale_and_future_are_rejected(self):
        for stamp, frame, child in ((self.base_ns, 'odom', 'base_footprint'),
                                    (self.base_ns + 1, 'map', 'base_footprint'),
                                    (self.base_ns + 1, 'odom', 'wrong_base'),
                                    (self.base_ns - 1_000_000_000, 'odom', 'base_footprint'),
                                    (self.base_ns + 1_000_000_000, 'odom', 'base_footprint')):
            with self.subTest(stamp=stamp, frame=frame, child=child):
                self.core = RobotControlCore()
                self.arm()
                self.assertFalse(self.core.accept_odom((0., 0., 0.), (0., 0., 0.), stamp,
                                                       frame, child, self.base_ns + 1, 10.01))
                self.assertFalse(self.core.armed)

    def test_waypoint_replay_nonfinite_frame_source_age_cancel_arm(self):
        for x, stamp, frame in ((.2, self.base_ns, 'base_link'),
                                (math.inf, self.base_ns + 1, 'base_link'),
                                (.2, self.base_ns + 1, 'map'),
                                (.2, self.base_ns - 2_000_000_000, 'base_link'),
                                (.2, self.base_ns + 1_000_000_000, 'base_link')):
            with self.subTest(x=x, stamp=stamp, frame=frame):
                self.core = RobotControlCore()
                self.arm()
                self.assertFalse(self.core.accept_waypoint(x, 0., 0., stamp, frame,
                                                           self.base_ns + 1, 10.01))
                self.assertFalse(self.core.armed)

    def test_camera_replay_stale_and_wrong_frame_cancel_arm(self):
        for stamp, frame in ((self.base_ns, 'camera_color_optical_frame'),
                              (self.base_ns + 1, 'other_camera'),
                              (self.base_ns - 2_000_000_000, 'camera_color_optical_frame')):
            with self.subTest(stamp=stamp, frame=frame):
                self.core = RobotControlCore()
                self.arm()
                self.assertFalse(self.core.accept_camera(stamp, frame, self.base_ns + 1, 10.01))
                self.assertFalse(self.core.armed)
                self.assertIsNone(self.core.gate.camera)

    def test_ros_clock_rollback_is_permanent_even_after_explicit_arm(self):
        self.arm()
        command = self.core.tick(self.base_ns - 1, 10.05)
        self.assertEqual(command, (0.0, 0.0, 'clock_rollback'))
        self.assertTrue(self.core.gate.clock_fault)
        self.assertFalse(self.core.arm(self.base_ns + 1, 10.06)[0])
        self.assertFalse(self.core.accept_camera(self.base_ns + 1, 'camera_color_optical_frame',
                                                self.base_ns + 1, 10.06))
        self.assertTrue(self.core.disarm())
        self.assertFalse(self.core.arm(self.base_ns + 2, 10.07)[0])

    def test_monotonic_expiry_operates_with_frozen_ros_clock(self):
        self.arm()
        for step in range(1, 12):
            command = self.core.tick(self.base_ns, 10.0 + step * .05)
        self.assertEqual(command[:2], (0.0, 0.0))
        self.assertEqual(self.core.reason, 'odom_timeout')

    def test_timer_stall_cancels_arm_even_when_inputs_are_fresh(self):
        self.arm()
        now = self.fresh(10.3)
        self.assertEqual(self.core.tick(now, 10.3), (0.0, 0.0, 'control_timer_gap'))
        self.assertFalse(self.core.armed)

    def test_side_rear_and_cone_targets_zero_and_disarm(self):
        for x, y in ((-.2, 0.), (0., .3), (.1, .2), (.01, 0.)):
            with self.subTest(x=x, y=y):
                self.core = RobotControlCore()
                self.arm()
                self.core.accept_waypoint(x, y, 0., self.base_ns + 1, 'base_link', self.base_ns + 1, 10.01)
                self.assertFalse(self.core.armed)
                self.assertEqual(self.core.last_linear, 0.0)

    def test_pure_pursuit_yaw_formula_and_sign(self):
        limits = RobotControlLimits()
        for y in (-.1, .1):
            speed, turn, reason = ackermann_velocity(1., y, limits)
            self.assertEqual(reason, 'tracking')
            self.assertAlmostEqual(turn, speed * 2 * y / (1 + y*y))
        for x, y in ((0., 1.), (-1., 0.), (.1, 1.)):
            self.assertEqual(ackermann_velocity(x, y, limits)[:2], (0.0, 0.0))

    def test_hard_parameter_bounds_cannot_be_relaxed(self):
        invalid = {
            'max_linear_mps': .20001, 'max_angular_rps': .101,
            'min_turning_radius_m': .99, 'linear_acceleration_mps2': .026,
            'arm_lease_sec': 10.01, 'waypoint_timeout_sec': .61,
            'source_timeout_sec': 1.01, 'camera_timeout_sec': 1.01,
            'odom_timeout_sec': .51, 'future_tolerance_sec': .21,
            'max_measured_linear_mps': .30001, 'max_measured_angular_rps': .301,
            'stationary_linear_mps': .011, 'stationary_angular_rps': .031,
            'max_tick_gap_sec': .201, 'max_target_heading_deg': 46.,
            'max_waypoint_distance_m': 5.01, 'goal_tolerance_m': .019,
            'waypoint_frame': 'odom', 'camera_frame': '/camera', 'odom_frame': '',
        }
        for key, value in invalid.items():
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    replace(RobotControlLimits(), **{key: value})
        for bad in (math.inf, math.nan, -1., 0.):
            with self.assertRaises(ValueError):
                RobotControlLimits(max_linear_mps=bad)

    def test_faster_profile_is_explicit_and_keeps_other_protections(self):
        old = RobotControlLimits()
        self.assertEqual(old.max_linear_mps, .05)
        self.assertEqual(old.max_measured_linear_mps, .12)
        new = replace(old, max_linear_mps=.2, max_measured_linear_mps=.3)
        self.assertEqual(new.max_linear_mps, .2)
        self.assertEqual(new.max_measured_linear_mps, .3)
        for key in old.__dataclass_fields__:
            if key not in ('max_linear_mps', 'max_measured_linear_mps'):
                self.assertEqual(getattr(old, key), getattr(new, key), key)
        for speed, measured in ((.2, .12), (.2, .19999), (.05, .04999)):
            with self.subTest(speed=speed, measured=measured), self.assertRaises(ValueError):
                RobotControlLimits(max_linear_mps=speed, max_measured_linear_mps=measured)
        RobotControlLimits(max_linear_mps=.2, max_measured_linear_mps=.2)

    def test_faster_profile_preserves_acceleration_angular_cap_and_curvature(self):
        self.core = RobotControlCore(RobotControlLimits(
            max_linear_mps=.2, max_measured_linear_mps=.3))
        now = self.fresh()
        self.assertTrue(self.core.arm_for(now, 10., 0., ARM_MODE_CONTINUOUS)[0])
        previous = 0.
        for step in range(1, 202):
            at = 10. + step * .05
            now = self.fresh(at, y=.1, speed=previous)
            speed, turn, reason = self.core.tick(now, at)
            self.assertEqual(reason, 'tracking')
            self.assertGreaterEqual(speed, 0.)
            self.assertLessEqual(speed, .2)
            self.assertLessEqual(speed - previous, .025 * .05 + 1e-12)
            self.assertLessEqual(abs(turn), speed + 1e-12)
            self.assertLessEqual(abs(turn), .1)
            if step < 160:
                self.assertLess(speed, .2)
            previous = speed
        self.assertAlmostEqual(previous, .2)
        self.assertAlmostEqual(turn, .1)
        self.core.disarm('operator_disarmed')
        self.assertEqual(self.core.tick(self.fresh(20.15), 20.15)[:2], (0., 0.))

    def test_faster_profile_measured_speed_is_fault_threshold_not_a_command(self):
        limits = RobotControlLimits(max_linear_mps=.2, max_measured_linear_mps=.3)
        for speed in (.2, .3):
            with self.subTest(speed=speed):
                self.core = RobotControlCore(limits)
                now = self.fresh(speed=speed)
                self.assertEqual(self.core.arm(now, 10.), (False, 'robot_not_stationary'))
        for linear in ((.30001, 0., 0.), (0., .30001, 0.), (.22, .22, 0.)):
            with self.subTest(linear=linear):
                self.core = RobotControlCore(limits)
                self.arm()
                self.assertFalse(self.core.accept_odom(
                    linear, (0., 0., 0.), self.base_ns + 1,
                    'odom', 'base_footprint', self.base_ns + 1, 10.01))
                self.assertEqual(self.core.reason, 'odom_speed_limit')
                self.assertFalse(self.core.armed)
                self.assertEqual(self.core.last_linear, 0.)
                self.assertEqual(self.core.tick(self.fresh(10.05), 10.05)[:2], (0., 0.))

    def test_configurable_lease_preserves_default_and_rejects_invalid_values(self):
        self.assertEqual(RobotControlLimits().arm_lease_sec, 3.0)
        for seconds in (1.5, 3.0, 5.0, 10.0):
            self.assertEqual(RobotControlLimits(arm_lease_sec=seconds).arm_lease_sec, seconds)
        for seconds in (0., -1., 10.0001, math.nan, math.inf):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                RobotControlLimits(arm_lease_sec=seconds)

    def test_ten_second_lease_stops_at_deadline_without_rearming(self):
        self.core = RobotControlCore(RobotControlLimits(arm_lease_sec=10.0))
        self.arm()
        self.assertEqual(self.core.arm_deadline, 20.0)
        for step in range(1, 202):
            at = 10.0 + step * .05
            now = self.fresh(at)
            command = self.core.tick(now, at)
            if step < 200:
                self.assertGreater(command[0], 0.)
                self.assertLessEqual(command[0], .05)
                self.assertLessEqual(abs(command[1]), command[0] + 1e-12)
                if step == 120:
                    self.assertFalse(self.core.arm(now, at)[0])
                    self.assertEqual(self.core.arm_deadline, 20.0)
            else:
                self.assertEqual(command[:2], (0., 0.))
                self.assertFalse(self.core.armed)
                self.assertEqual(self.core.reason, 'arm_lease_expired')

    def test_ten_second_lease_keeps_original_waypoint_timeout(self):
        self.core = RobotControlCore(RobotControlLimits(arm_lease_sec=10.0))
        self.arm()
        for step in range(1, 14):
            at = 10.0 + step * .05
            now = self.fresh(at, waypoint=False)
            command = self.core.tick(now, at)
        self.assertEqual(command, (0., 0., 'waypoint_timeout'))
        self.assertFalse(self.core.armed)
        now = self.fresh(10.7)
        self.assertEqual(self.core.tick(now, 10.7)[:2], (0., 0.))

    def test_arm_for_timed_uses_each_request_not_startup_default(self):
        for duration in (.05, 1.25, 3.0, 5.0, 10.0):
            with self.subTest(duration=duration):
                self.core = RobotControlCore(RobotControlLimits(arm_lease_sec=3.0))
                now = self.fresh()
                self.assertTrue(self.core.arm_for(now, 10.0, duration)[0])
                self.assertEqual(self.core.active_mode, 'timed')
                self.assertEqual(self.core.requested_duration_sec, duration)
                self.assertEqual(self.core.arm_deadline, 10.0 + duration)
                for step in range(1, round(duration / .05) + 1):
                    at = 10.0 + step * .05
                    command = self.core.tick(self.fresh(at), at)
                self.assertEqual(command, (0., 0., 'arm_lease_expired'))
                self.assertFalse(self.core.armed)

    def test_continuous_runs_beyond_ten_seconds_with_unchanged_limits(self):
        now = self.fresh()
        self.assertTrue(self.core.arm_for(now, 10.0, 0.0, ARM_MODE_CONTINUOUS)[0])
        self.assertEqual(self.core.active_mode, 'continuous')
        self.assertEqual(self.core.requested_duration_sec, 0.0)
        self.assertIsNone(self.core.arm_deadline)
        previous_speed = 0.0
        for step in range(1, 601):
            at = 10.0 + step * .05
            speed, turn, reason = self.core.tick(self.fresh(at), at)
            self.assertEqual(reason, 'tracking')
            self.assertGreater(speed, 0.0)
            self.assertLessEqual(speed, .05)
            self.assertLessEqual(speed - previous_speed, .025 * .05 + 1e-12)
            self.assertLessEqual(abs(turn), speed / 1.0 + 1e-12)
            self.assertLessEqual(abs(turn), .10)
            self.assertTrue(self.core.armed)
            previous_speed = speed
        state = self.core.snapshot(40.0)
        self.assertTrue(state['run_until_disarmed'])
        self.assertIsNone(state['lease_remaining_sec'])
        json.dumps(state, allow_nan=False)
        self.core.disarm('operator_disarmed')
        self.assertEqual(self.core.tick(self.fresh(40.05), 40.05)[:2], (0., 0.))
        state = self.core.snapshot(40.05)
        self.assertIsNone(state['active_mode'])
        self.assertIsNone(state['requested_duration_sec'])
        self.assertIsNone(state['arm_deadline_monotonic'])
        self.assertFalse(state['run_until_disarmed'])
        self.assertEqual(state['lease_remaining_sec'], 0.0)

    def test_bad_arm_for_requests_never_arm(self):
        invalid = [(mode, 3.0) for mode in (-1, 2, 255, '1', 1.0, True, None)]
        invalid += [(ARM_MODE_TIMED, duration) for duration in
                    (0., -1., 10.00001, math.inf, -math.inf, math.nan, '3', None, True, 10**400)]
        invalid += [(ARM_MODE_CONTINUOUS, duration) for duration in
                    (.001, 1., 3., 10., -1., math.inf, math.nan, None, True)]
        for mode, duration in invalid:
            with self.subTest(mode=mode, duration=duration):
                self.core = RobotControlCore()
                now = self.fresh()
                self.assertFalse(self.core.arm_for(now, 10.0, duration, mode)[0])
                self.assertFalse(self.core.armed)
                self.assertIsNone(self.core.active_mode)
                self.assertIsNone(self.core.arm_deadline)
                self.assertIsNone(self.core.requested_duration_sec)
                self.assertEqual(self.core.tick(now, 10.0)[:2], (0., 0.))

    def test_active_modes_cannot_be_switched_renewed_or_replaced(self):
        for mode, duration in ((ARM_MODE_TIMED, 3.0), (ARM_MODE_CONTINUOUS, 0.0)):
            with self.subTest(mode=mode):
                self.core = RobotControlCore()
                now = self.fresh()
                self.assertTrue(self.core.arm_for(now, 10.0, duration, mode)[0])
                original = (self.core.active_mode, self.core.arm_deadline, self.core.requested_duration_sec)
                for new_mode, new_duration in ((0, 10.), (1, 0.), (2, 0.), (0, math.nan)):
                    response = self.core.arm_for(now, 10.0, new_duration, new_mode)
                    self.assertFalse(response[0])
                    self.assertTrue(self.core.armed)
                    self.assertEqual((self.core.active_mode, self.core.arm_deadline,
                                      self.core.requested_duration_sec), original)
                self.assertFalse(self.core.arm(now, 10.0)[0])
                self.assertEqual((self.core.active_mode, self.core.arm_deadline,
                                  self.core.requested_duration_sec), original)

    def test_mode_change_requires_disarm_stationary_and_explicit_request(self):
        for old_mode, new_mode in ((0, 1), (1, 0)):
            with self.subTest(old_mode=old_mode):
                self.core = RobotControlCore()
                now = self.fresh()
                self.assertTrue(self.core.arm_for(now, 10., 3. if old_mode == 0 else 0., old_mode)[0])
                self.core.disarm()
                now = self.fresh(10.05, speed=.011)
                self.assertEqual(self.core.arm_for(now, 10.05, 3. if new_mode == 0 else 0., new_mode),
                                 (False, 'robot_not_stationary'))
                now = self.fresh(10.10)
                self.assertEqual(self.core.tick(now, 10.10)[:2], (0., 0.))
                self.assertTrue(self.core.arm_for(now, 10.10, 3. if new_mode == 0 else 0., new_mode)[0])
                self.assertEqual(self.core.active_mode, 'timed' if new_mode == 0 else 'continuous')

    def test_arm_for_both_modes_require_all_inputs_and_running_model(self):
        for mode, duration in ((0, 3.), (1, 0.)):
            for missing in ('camera', 'waypoint', 'odom', 'model'):
                with self.subTest(mode=mode, missing=missing):
                    self.core = RobotControlCore()
                    now = self.fresh(camera=missing != 'camera', waypoint=missing != 'waypoint',
                                     odom=missing != 'odom')
                    if missing == 'model':
                        self.core.set_model_state('faulted')
                    self.assertFalse(self.core.arm_for(now, 10., duration, mode)[0])
                    self.assertFalse(self.core.armed)

    def test_faults_cancel_both_modes_without_automatic_recovery(self):
        later_ns = self.base_ns + 100_000_000
        faults = {
            'camera_payload': lambda: self.core.reject_camera(),
            'camera_replay': lambda: self.core.accept_camera(
                self.base_ns, 'camera_color_optical_frame', later_ns, 10.1),
            'camera_frame': lambda: self.core.accept_camera(later_ns, 'wrong', later_ns, 10.1),
            'camera_source_age': lambda: self.core.accept_camera(
                1, 'camera_color_optical_frame', later_ns, 10.1),
            'camera_future': lambda: self.core.accept_camera(
                later_ns + 1_000_000_000, 'camera_color_optical_frame', later_ns, 10.1),
            'model': lambda: self.core.set_model_state('faulted'),
            'waypoint_replay': lambda: self.core.accept_waypoint(
                .2, 0., 0., self.base_ns, 'base_link', later_ns, 10.1),
            'waypoint_nonfinite': lambda: self.core.accept_waypoint(
                math.nan, 0., 0., later_ns, 'base_link', later_ns, 10.1),
            'waypoint_frame': lambda: self.core.accept_waypoint(
                .2, 0., 0., later_ns, 'wrong', later_ns, 10.1),
            'waypoint_source_age': lambda: self.core.accept_waypoint(
                .2, 0., 0., 1, 'base_link', later_ns, 10.1),
            'waypoint_future': lambda: self.core.accept_waypoint(
                .2, 0., 0., later_ns + 1_000_000_000, 'base_link', later_ns, 10.1),
            'waypoint_forward_cone': lambda: self.core.accept_waypoint(
                .1, .2, 0., later_ns, 'base_link', later_ns, 10.1),
            'waypoint_rear': lambda: self.core.accept_waypoint(
                -.2, 0., 0., later_ns, 'base_link', later_ns, 10.1),
            'waypoint_goal': lambda: self.core.accept_waypoint(
                .01, 0., 0., later_ns, 'base_link', later_ns, 10.1),
            'waypoint_range': lambda: self.core.accept_waypoint(
                6., 0., 0., later_ns, 'base_link', later_ns, 10.1),
            'odom_speed': lambda: self.core.accept_odom(
                (.121, 0., 0.), (0., 0., 0.), later_ns, 'odom', 'base_footprint', later_ns, 10.1),
            'odom_turn': lambda: self.core.accept_odom(
                (0., 0., 0.), (0., 0., .301), later_ns, 'odom', 'base_footprint', later_ns, 10.1),
            'odom_nonfinite': lambda: self.core.accept_odom(
                (math.nan, 0., 0.), (0., 0., 0.), later_ns, 'odom', 'base_footprint', later_ns, 10.1),
            'odom_replay': lambda: self.core.accept_odom(
                (0., 0., 0.), (0., 0., 0.), self.base_ns, 'odom', 'base_footprint', later_ns, 10.1),
            'odom_frame': lambda: self.core.accept_odom(
                (0., 0., 0.), (0., 0., 0.), later_ns, 'map', 'base_footprint', later_ns, 10.1),
            'odom_child_frame': lambda: self.core.accept_odom(
                (0., 0., 0.), (0., 0., 0.), later_ns, 'odom', 'wrong', later_ns, 10.1),
            'odom_source_age': lambda: self.core.accept_odom(
                (0., 0., 0.), (0., 0., 0.), 1, 'odom', 'base_footprint', later_ns, 10.1),
            'odom_future': lambda: self.core.accept_odom(
                (0., 0., 0.), (0., 0., 0.), later_ns + 1_000_000_000,
                'odom', 'base_footprint', later_ns, 10.1),
            'estop': lambda: self.core.set_estop(True),
        }
        for mode, duration in ((0, 3.), (1, 0.)):
            for name, fault in faults.items():
                with self.subTest(mode=mode, fault=name):
                    self.core = RobotControlCore()
                    self.assertTrue(self.core.arm_for(self.fresh(), 10., duration, mode)[0])
                    self.assertGreater(self.core.tick(self.base_ns + 50_000_000, 10.05)[0], 0.)
                    fault()
                    self.assertFalse(self.core.armed)
                    self.assertIsNone(self.core.active_mode)
                    self.assertIsNone(self.core.arm_deadline)
                    self.assertIsNone(self.core.requested_duration_sec)
                    self.assertEqual(self.core.last_linear, 0.)
                    if name == 'estop':
                        self.assertFalse(self.core.arm_for(later_ns, 10.1, duration, mode)[0])
                        self.core.set_estop(False)
                        self.assertIsNone(self.core.gate.waypoint)
                    now = self.fresh(10.15)
                    self.assertEqual(self.core.tick(now, 10.15)[:2], (0., 0.))
                    self.assertFalse(self.core.armed)

    def test_timeouts_and_timer_stall_cancel_both_modes(self):
        for mode, duration in ((0, 3.), (1, 0.)):
            for missing, steps, reason in (('camera', 23, 'camera_timeout'),
                                           ('waypoint', 14, 'waypoint_timeout'),
                                           ('odom', 12, 'odom_timeout'),
                                           ('timer', 1, 'control_timer_gap')):
                with self.subTest(mode=mode, missing=missing):
                    self.core = RobotControlCore()
                    self.assertTrue(self.core.arm_for(self.fresh(), 10., duration, mode)[0])
                    for step in range(1, steps + 1):
                        at = 10.3 if missing == 'timer' else 10.0 + step * .05
                        now = self.fresh(at, camera=missing != 'camera',
                                         waypoint=missing != 'waypoint', odom=missing != 'odom')
                        command = self.core.tick(now, at)
                    self.assertEqual(command, (0., 0., reason))
                    self.assertFalse(self.core.armed)
                    self.assertIsNone(self.core.active_mode)
                    self.assertIsNone(self.core.arm_deadline)
                    now = self.fresh(at + .05)
                    self.assertEqual(self.core.tick(now, at + .05)[:2], (0., 0.))

    def test_clock_faults_remain_latched_in_both_modes(self):
        for mode, duration in ((0, 3.), (1, 0.)):
            for kind in ('ros', 'steady'):
                with self.subTest(mode=mode, clock=kind):
                    self.core = RobotControlCore()
                    self.assertTrue(self.core.arm_for(self.fresh(), 10., duration, mode)[0])
                    now = self.base_ns - 1 if kind == 'ros' else self.base_ns
                    steady = 10.05 if kind == 'ros' else 9.99
                    self.assertEqual(self.core.tick(now, steady), (0., 0., 'clock_rollback'))
                    self.assertFalse(self.core.armed)
                    self.assertIsNone(self.core.active_mode)
                    self.assertIsNone(self.core.arm_deadline)
                    self.core.disarm()
                    self.assertFalse(self.core.arm_for(self.base_ns + 1, 10.1, duration, mode)[0])
                    self.assertTrue(self.core.gate.clock_fault)

    def test_legacy_arm_never_inherits_continuous_mode(self):
        now = self.fresh()
        self.assertTrue(self.core.arm_for(now, 10., 0., ARM_MODE_CONTINUOUS)[0])
        self.core.disarm()
        now = self.fresh(10.05)
        self.assertEqual(self.core.arm(now, 10.05), (True, 'armed_for_one_short_lease'))
        self.assertEqual(self.core.active_mode, 'timed')
        self.assertEqual(self.core.requested_duration_sec, 3.)
        self.assertEqual(self.core.arm_deadline, 13.05)


if __name__ == '__main__':
    unittest.main()
