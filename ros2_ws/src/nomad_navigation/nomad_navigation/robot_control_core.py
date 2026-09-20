"""Fail-closed Ackermann control with explicit arming; no hardware operations."""

from dataclasses import dataclass
import math

from nomad_navigation.controller_core import ControlLimits, SafetyGate


MAX_ARM_LEASE_SEC = 10.0
MAX_OPERATOR_PAUSE_SEC = 30.0
ARM_MODE_TIMED = 0
ARM_MODE_CONTINUOUS = 1


@dataclass(frozen=True)
class RobotControlLimits:
    max_linear_mps: float = 0.05
    max_angular_rps: float = 0.10
    min_turning_radius_m: float = 1.0
    linear_acceleration_mps2: float = 0.025
    arm_lease_sec: float = 3.0
    waypoint_timeout_sec: float = 0.6
    source_timeout_sec: float = 1.0
    camera_timeout_sec: float = 1.0
    odom_timeout_sec: float = 0.5
    future_tolerance_sec: float = 0.2
    max_measured_linear_mps: float = 0.12
    max_measured_angular_rps: float = 0.3
    stationary_linear_mps: float = 0.01
    stationary_angular_rps: float = 0.03
    max_tick_gap_sec: float = 0.2
    goal_tolerance_m: float = 0.02
    max_waypoint_distance_m: float = 5.0
    max_target_heading_deg: float = 45.0
    waypoint_frame: str = 'base_link'
    camera_frame: str = 'camera_color_optical_frame'
    odom_frame: str = 'odom'
    odom_child_frame: str = 'base_footprint'

    def __post_init__(self):
        # Startup parameters may select a bounded lease (default 3 s, <=10 s).
        # The explicit 0.1/0.2 m/s startup profiles may raise the command ceiling;
        # defaults remain the first-test 0.05 m/s profile. Curvature, angular
        # speed, acceleration and freshness protections are unchanged.
        ceilings = {
            'max_linear_mps': .2, 'max_angular_rps': .10,
            'linear_acceleration_mps2': .025, 'arm_lease_sec': MAX_ARM_LEASE_SEC,
            'waypoint_timeout_sec': .6, 'source_timeout_sec': 1.0,
            'camera_timeout_sec': 1.0, 'odom_timeout_sec': .5,
            'max_measured_linear_mps': .3, 'max_measured_angular_rps': .3,
            'stationary_linear_mps': .01, 'stationary_angular_rps': .03,
            'max_tick_gap_sec': .2, 'max_waypoint_distance_m': 5.0,
            'max_target_heading_deg': 45.0,
        }
        for name, ceiling in ceilings.items():
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 < value <= ceiling:
                raise ValueError(f'{name} must be finite and within (0, {ceiling}]')
        if not math.isfinite(self.min_turning_radius_m) or self.min_turning_radius_m < 1.0:
            raise ValueError('min_turning_radius_m must be finite and at least 1.0')
        if not math.isfinite(self.future_tolerance_sec) or not 0 <= self.future_tolerance_sec <= .2:
            raise ValueError('future_tolerance_sec must be within [0, 0.2]')
        if not math.isfinite(self.goal_tolerance_m) or not .02 <= self.goal_tolerance_m <= 1.0:
            raise ValueError('goal_tolerance_m must be within [0.02, 1.0]')
        if self.max_tick_gap_sec < .05:
            raise ValueError('max_tick_gap_sec must allow the 20 Hz control period')
        if self.max_measured_linear_mps < self.max_linear_mps:
            raise ValueError('Measured linear-speed fault limit must not be below command speed limit')
        if (self.stationary_linear_mps > self.max_measured_linear_mps
                or self.stationary_angular_rps > self.max_measured_angular_rps):
            raise ValueError('Stationary limits must not exceed measured-speed limits')
        if self.waypoint_frame != 'base_link':
            raise ValueError('The reviewed waypoint/output frame must be base_link')
        for name in ('camera_frame', 'odom_frame', 'odom_child_frame'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip() or value.startswith('/') or value != value.strip():
                raise ValueError(f'{name} must be a nonempty frame without a leading slash')

    def gate_limits(self):
        return ControlLimits(
            observation_hz=4.0, max_linear_mps=self.max_linear_mps,
            max_angular_rps=self.max_angular_rps,
            waypoint_timeout_sec=self.waypoint_timeout_sec,
            source_timeout_sec=self.source_timeout_sec,
            camera_timeout_sec=self.camera_timeout_sec,
            future_tolerance_sec=self.future_tolerance_sec,
            goal_tolerance_m=self.goal_tolerance_m,
            max_waypoint_distance_m=self.max_waypoint_distance_m,
            waypoint_frame=self.waypoint_frame, camera_frame=self.camera_frame)


def ackermann_velocity(x, y, limits, linear_cap=None):
    """Pure pursuit, positive forward cone only; never rotate in place/reverse."""
    if not all(math.isfinite(value) for value in (x, y)):
        return 0.0, 0.0, 'invalid_waypoint'
    distance = math.hypot(x, y)
    if distance > limits.max_waypoint_distance_m:
        return 0.0, 0.0, 'waypoint_out_of_range'
    if distance <= limits.goal_tolerance_m:
        return 0.0, 0.0, 'within_tolerance'
    if x <= 0:
        return 0.0, 0.0, 'rear_or_side_waypoint'
    if abs(math.atan2(y, x)) > math.radians(limits.max_target_heading_deg):
        return 0.0, 0.0, 'outside_forward_cone'
    speed = min(x * 4.0, limits.max_linear_mps)
    if linear_cap is not None:
        if not math.isfinite(linear_cap) or linear_cap < 0:
            return 0.0, 0.0, 'invalid_linear_cap'
        speed = min(speed, linear_cap)
    curvature = 2.0 * y / (x * x + y * y)
    curvature = max(-1.0 / limits.min_turning_radius_m,
                    min(curvature, 1.0 / limits.min_turning_radius_m))
    angular = max(-limits.max_angular_rps, min(speed * curvature, limits.max_angular_rps))
    return speed, angular, 'tracking'


class RobotControlCore:
    """All mutable motion decisions live here, independent of ROS callbacks.

    Defaults latch stops. Explicit three-profile network_pause_resume retains only
    continuous intent through enumerated transient waits while commanding zero.
    Hard faults/manual stops erase intent; fresh data never re-arms a stopped run.
    Clock faults remain latched until restart. All freshness limits stay intact.
    """

    def __init__(self, limits=None, *, network_pause_resume=False, operator_pause_resume=False):
        self.limits = limits or RobotControlLimits()
        if type(network_pause_resume) is not bool or (network_pause_resume
                and self.limits.max_linear_mps not in (.05, .1, .2)):
            raise ValueError('Network pause/resume requires an explicit boolean and speed profile 0.05, 0.1 or 0.2 m/s')
        self.network_pause_resume = network_pause_resume
        if type(operator_pause_resume) is not bool or (operator_pause_resume and not network_pause_resume):
            raise ValueError('Operator pause/resume requires network pause/resume and an explicit boolean')
        self.operator_pause_resume = operator_pause_resume
        self.operator_link_connected = not operator_pause_resume
        self.operator_paused = False
        self.operator_resume_deadline = None
        self.motion_paused = False
        self.resume_candidates = 0
        self.resume_after_stamp = 0
        self.last_resume_arrival = None
        self.gate = SafetyGate(self.limits.gate_limits())
        self.armed = False
        self.arm_deadline = None
        self.active_mode = None
        self.requested_duration_sec = None
        self.reason = 'disarmed'
        self.odom = None
        self.odom_rejection = 'waiting_odom'
        self.highest_odom_stamp = 0
        self.highest_camera_stamp = 0
        self.last_tick = None
        self.last_linear = 0.0

    def disarm(self, reason='disarmed'):
        self.armed = False
        self.arm_deadline = None
        self.active_mode = None
        self.requested_duration_sec = None
        self.last_linear = 0.0
        self.reason = reason
        self.motion_paused = False
        self.resume_candidates = 0
        self.last_resume_arrival = None
        self.operator_paused = False
        self.operator_resume_deadline = None
        return True

    def operator_link_pause(self, reason, now, monotonic):
        """Retain only a previously confirmed continuous run, while forcing zero.

        The first loss starts one bounded deadline. Repeated pauses, a fresh
        heartbeat, or incomplete recovery never grant another 30-second window.
        """
        self.operator_link_connected = False
        if not self._clock(now, monotonic):
            return
        if not self.operator_pause_resume:
            self.disarm(reason)
            return
        if not self._can_pause():
            if self.armed:  # Timed runs cannot retain intent through link loss.
                self.disarm(reason)
            # Repeated paused-link ticks must not hide the original hard fault
            # or expired recovery window on an already-disarmed controller.
            return
        if self.operator_resume_deadline is None:
            self.operator_resume_deadline = monotonic + MAX_OPERATOR_PAUSE_SEC
        if monotonic >= self.operator_resume_deadline:
            self.disarm('operator_resume_window_expired')
            return
        self.operator_paused = True
        self._pause(reason)
        self.reason = 'paused_operator:' + reason

    def operator_link_recovered(self, now, monotonic):
        """A NEW valid generation arrived; this never creates an arm intent."""
        self.operator_link_connected = True
        if not self._clock(now, monotonic):
            return
        if not self.operator_paused:
            return
        if self.operator_resume_deadline is None or monotonic >= self.operator_resume_deadline:
            self.disarm('operator_resume_window_expired')
            return
        # Reject queued pre-reconnect predictions even when their source age is
        # within the ordinary freshness bound. ROS time rollback still latches.
        self.resume_after_stamp = max(now, self.highest_camera_stamp,
                                      self.gate.highest_waypoint_stamp)
        self.resume_candidates = 0
        self.last_resume_arrival = None
        self.gate.reject('waiting_waypoint')
        self.motion_paused = True
        self.last_linear = 0.0
        self.reason = 'paused_operator:waiting_fresh_predictions_and_stationary'

    def _can_pause(self):
        return self.network_pause_resume and self.armed and self.active_mode == 'continuous'

    def _pause(self, reason, *, restart=False):
        if not self._can_pause():
            self.disarm(reason)
            return
        if (not self.motion_paused or restart or (self.resume_candidates and reason in
                ('camera_timeout', 'source_image_timeout', 'waypoint_timeout'))):
            self.resume_after_stamp = max(self.resume_after_stamp, self.highest_camera_stamp,
                                          self.gate.highest_waypoint_stamp)
            self.resume_candidates = 0
            self.last_resume_arrival = None
            self.gate.reject('waiting_waypoint')
        self.motion_paused = True
        self.last_linear = 0.0
        self.reason = 'paused_network:' + reason

    def _clock(self, now, monotonic):
        if (not math.isfinite(monotonic) or monotonic < 0 or now <= 0
                or (self.gate.last_ros_time is not None and now < self.gate.last_ros_time)
                or (self.last_tick is not None and monotonic < self.last_tick)):
            self.gate.clock_fault = True
            self.gate.reject('clock_rollback')
        self.gate.last_ros_time = now
        if self.gate.clock_fault:
            self.disarm('clock_rollback')
            return False
        return True

    def accept_waypoint(self, x, y, z, stamp, frame, now, monotonic):
        if not self._clock(now, monotonic):
            return False
        accepted = self.gate.accept_waypoint(x, y, z, stamp, frame, now, monotonic)
        if not accepted:
            if (self.gate.last_rejection == 'stale_or_future_waypoint'
                    and now - stamp > self.limits.source_timeout_sec * 1e9 and self._can_pause()):
                self._pause('source_image_timeout')
            else:
                self.disarm(self.gate.last_rejection)
        elif ackermann_velocity(x, y, self.limits)[2] != 'tracking':
            self.disarm(ackermann_velocity(x, y, self.limits)[2])
        elif (self.motion_paused and stamp > self.resume_after_stamp
                and (not self.operator_pause_resume or self.operator_link_connected)):
            if (self.last_resume_arrival is not None
                    and monotonic - self.last_resume_arrival > self.limits.waypoint_timeout_sec):
                self.resume_candidates = 0
            self.resume_candidates = min(3, self.resume_candidates + 1)
            self.last_resume_arrival = monotonic
        return accepted

    def reject_camera(self, reason='invalid_camera'):
        self.gate.camera = None
        self.gate.reject(reason)
        self.disarm(reason)

    def accept_camera(self, stamp, frame, now, monotonic):
        if not self._clock(now, monotonic):
            return False
        if stamp <= self.highest_camera_stamp or not self.gate.accept_camera(stamp, frame, now, monotonic):
            self.reject_camera()
            return False
        self.highest_camera_stamp = stamp
        return True

    def set_model_state(self, state):
        previous = self.gate.model_state
        self.gate.set_model_state(state)
        if (state == 'running' and previous != 'running' and self.motion_paused
                and self.gate.waypoint is None):
            # Model status and waypoint arrive on separate topics. A recovered
            # status must not leave the previous nonrunning rejection masquerading
            # as a hard fault while we still await post-pause predictions.
            self.gate.reject('waiting_waypoint')
        if state != 'running':
            if state in ('network_wait', 'waiting_context') and self._can_pause():
                self._pause(state, restart=previous == 'running')
            else:
                self.disarm('model_not_running')

    def set_estop(self, active):
        self.gate.set_estop(active)
        if active:
            self.disarm('emergency_stop')
        elif self.gate.estop_latched:
            # Releasing software stop only clears its latch and old waypoint.
            # A new fresh waypoint and a NEW explicit arm request are required.
            self.gate.reset_estop()
            self.disarm('waiting_waypoint_after_estop_release')

    def accept_odom(self, linear, angular, stamp, frame, child_frame, now, monotonic):
        if not self._clock(now, monotonic):
            return False
        reason = None
        if (len(linear) != 3 or len(angular) != 3
                or not all(math.isfinite(value) for value in (*linear, *angular))):
            reason = 'invalid_odom'
        elif frame != self.limits.odom_frame or child_frame != self.limits.odom_child_frame:
            reason = 'wrong_odom_frame'
        elif not self.gate._valid_time(stamp, now, self.limits.odom_timeout_sec):
            reason = 'stale_or_future_odom'
        elif stamp <= self.highest_odom_stamp:
            reason = 'replayed_odom'
        else:
            speed, turn = math.hypot(*linear), math.hypot(*angular)
            if speed > self.limits.max_measured_linear_mps or turn > self.limits.max_measured_angular_rps:
                reason = 'odom_speed_limit'
        if reason:
            self.odom = None
            self.odom_rejection = reason
            self.disarm(reason)
            return False
        self.highest_odom_stamp = stamp
        self.odom = (stamp, monotonic, speed, turn)
        self.odom_rejection = None
        return True

    def _ready_reason(self, now, monotonic, stationary=False):
        _, _, reason = self.gate.decide(now, monotonic)
        if reason != 'tracking':
            return reason
        odom_reason = self._odom_reason(now, monotonic, stationary)
        if odom_reason:
            return odom_reason
        return ackermann_velocity(self.gate.waypoint[0], self.gate.waypoint[1], self.limits)[2]

    def _odom_reason(self, now, monotonic, stationary=False):
        if self.odom is None:
            return self.odom_rejection or 'waiting_odom'
        stamp, arrival, speed, turn = self.odom
        if (not self.gate._valid_time(stamp, now, self.limits.odom_timeout_sec)
                or monotonic - arrival > self.limits.odom_timeout_sec):
            return 'odom_timeout'
        if speed > self.limits.max_measured_linear_mps or turn > self.limits.max_measured_angular_rps:
            return 'odom_speed_limit'
        if stationary and (speed > self.limits.stationary_linear_mps or turn > self.limits.stationary_angular_rps):
            return 'robot_not_stationary'
        return None

    def arm(self, now, monotonic):
        """Compatibility entry point: always use the bounded startup default."""
        success, reason = self.arm_for(now, monotonic, self.limits.arm_lease_sec)
        return success, 'armed_for_one_short_lease' if success else reason

    def arm_for(self, now, monotonic, duration_sec, mode=ARM_MODE_TIMED):
        """Start one explicitly requested run; active runs cannot be renewed.

        TIMED accepts a finite duration in (0, 10] seconds. CONTINUOUS requires
        duration 0, so accidentally omitting a timed duration cannot enable it.
        After any stop, the robot must again be stationary with valid fresh
        inputs before a new explicit arm request can succeed.
        """
        if not self._clock(now, monotonic):
            return False, self.reason
        if self.operator_pause_resume and not self.operator_link_connected:
            return False, 'operator_link_not_connected'
        if self.armed:
            # Do not change the mode or extend a live lease through repeated calls.
            return False, 'already_armed_lease_not_extended'
        if (isinstance(mode, bool) or not isinstance(mode, int)
                or mode not in (ARM_MODE_TIMED, ARM_MODE_CONTINUOUS)):
            return False, 'invalid_arm_mode'
        try:
            valid_duration = (not isinstance(duration_sec, bool)
                              and isinstance(duration_sec, (int, float))
                              and math.isfinite(duration_sec))
        except (OverflowError, TypeError, ValueError):
            valid_duration = False
        if not valid_duration:
            return False, 'invalid_arm_duration'
        if mode == ARM_MODE_TIMED and not 0 < duration_sec <= MAX_ARM_LEASE_SEC:
            return False, 'timed_duration_must_be_within_0_and_10_sec'
        if mode == ARM_MODE_CONTINUOUS and duration_sec != 0:
            return False, 'continuous_duration_must_be_zero'
        reason = self._ready_reason(now, monotonic, stationary=True)
        if reason != 'tracking':
            self.disarm(reason)
            return False, reason
        self.armed = True
        self.active_mode = 'timed' if mode == ARM_MODE_TIMED else 'continuous'
        self.requested_duration_sec = float(duration_sec)
        self.arm_deadline = monotonic + duration_sec if mode == ARM_MODE_TIMED else None
        self.last_tick = monotonic
        self.last_linear = 0.0
        self.reason = 'armed'
        if mode == ARM_MODE_CONTINUOUS:
            return True, 'armed_continuous_until_disarm_or_safety_stop'
        return True, f'armed_for_{duration_sec:g}_sec'

    def tick(self, now, monotonic):
        if not self._clock(now, monotonic):
            return 0.0, 0.0, self.reason
        elapsed = 0.0 if self.last_tick is None else monotonic - self.last_tick
        self.last_tick = monotonic
        if self.armed and elapsed > self.limits.max_tick_gap_sec:
            self.disarm('control_timer_gap')
        if self.armed and self.arm_deadline is not None and monotonic >= self.arm_deadline:
            self.disarm('arm_lease_expired')
        if (self.armed and self.operator_resume_deadline is not None
                and monotonic >= self.operator_resume_deadline):
            self.disarm('operator_resume_window_expired')
        # A missing/invalid odometer must never be hidden behind a network wait.
        if self.armed and self._odom_reason(now, monotonic):
            self.disarm(self._odom_reason(now, monotonic))
        reason = self._ready_reason(now, monotonic)
        if reason != 'tracking':
            transient = reason in ('camera_timeout', 'source_image_timeout', 'waypoint_timeout')
            if reason == 'model_not_running':
                transient = self.gate.model_state in ('network_wait', 'waiting_context')
            if self.motion_paused and reason in ('waiting_camera', 'waiting_waypoint'):
                transient = True
            if transient and self._can_pause():
                self._pause(reason)
                return 0.0, 0.0, self.reason
            self.disarm(reason)
        if not self.armed:
            return 0.0, 0.0, self.reason
        if self.operator_pause_resume and not self.operator_link_connected:
            self.last_linear = 0.0
            self.reason = 'paused_operator:waiting_connection'
            return 0.0, 0.0, self.reason
        if self.motion_paused:
            # A new run is NOT armed here. Only the already-authorized continuous
            # intent survives a transient wait; manual stop and hard faults erase it.
            if (self.resume_candidates < 3 or self.gate.waypoint[2] <= self.resume_after_stamp
                    or self._odom_reason(now, monotonic, stationary=True)):
                self.last_linear = 0.0
                self.reason = 'paused_network:waiting_fresh_predictions_and_stationary'
                return 0.0, 0.0, self.reason
            self.motion_paused = False
            self.operator_paused = False
            self.operator_resume_deadline = None
            self.last_linear = 0.0
        x, y = self.gate.waypoint[:2]
        cap = self.last_linear + self.limits.linear_acceleration_mps2 * elapsed
        speed, angular, reason = ackermann_velocity(x, y, self.limits, linear_cap=cap)
        if reason != 'tracking':
            self.disarm(reason)
            return 0.0, 0.0, self.reason
        self.last_linear = speed
        self.reason = 'tracking'
        return speed, angular, self.reason

    def snapshot(self, monotonic):
        remaining_sec = 0.0
        if self.armed:
            remaining_sec = (max(0.0, self.arm_deadline - monotonic)
                             if self.arm_deadline is not None else None)
        return {
            'armed': self.armed, 'reason': self.reason,
            'network_pause_resume': self.network_pause_resume,
            'operator_pause_resume': self.operator_pause_resume,
            'operator_paused': self.operator_paused,
            'operator_link_connected': self.operator_link_connected,
            'operator_resume_remaining_sec': (max(0.0, self.operator_resume_deadline - monotonic)
                if self.operator_resume_deadline is not None else 0.0),
            'motion_paused': self.motion_paused, 'resume_candidates': self.resume_candidates,
            'active_mode': self.active_mode,
            'requested_duration_sec': self.requested_duration_sec,
            'run_until_disarmed': self.armed and self.active_mode == 'continuous',
            'arm_deadline_monotonic': self.arm_deadline,
            'lease_remaining_sec': remaining_sec,
            'clock_fault': self.gate.clock_fault,
            'estop_input': self.gate.estop_input, 'estop_latched': self.gate.estop_latched,
            'model_state': self.gate.model_state,
            'source_waypoint_stamp_ns': self.gate.waypoint[2] if self.gate.waypoint else None,
        }
