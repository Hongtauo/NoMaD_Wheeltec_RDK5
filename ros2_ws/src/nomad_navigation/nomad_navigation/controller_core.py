"""Pure waypoint-to-velocity logic and fail-closed preview safety gates."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ControlLimits:
    observation_hz: float = 4.0
    max_linear_mps: float = 0.2
    max_angular_rps: float = 0.4
    waypoint_timeout_sec: float = 0.6
    source_timeout_sec: float = 1.0
    camera_timeout_sec: float = 1.0
    future_tolerance_sec: float = 0.2
    goal_tolerance_m: float = 0.02
    max_waypoint_distance_m: float = 5.0
    waypoint_frame: str = 'base_link'
    camera_frame: str = 'camera_optical_frame'

    def __post_init__(self):
        for key in ('observation_hz', 'max_linear_mps', 'max_angular_rps',
                    'waypoint_timeout_sec', 'source_timeout_sec', 'camera_timeout_sec',
                    'max_waypoint_distance_m'):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('future_tolerance_sec', 'goal_tolerance_m'):
            value = getattr(self, key)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        for frame in (self.waypoint_frame, self.camera_frame):
            if not frame.strip() or frame.startswith('/'):
                raise ValueError('Expected frame names without leading slash')


def waypoint_velocity(x, y, limits):
    """Match the author's forward-plane mapping; reject rear targets safely."""
    if not math.isfinite(x) or not math.isfinite(y):
        return 0.0, 0.0, 'invalid_waypoint'
    if math.hypot(x, y) > limits.max_waypoint_distance_m:
        return 0.0, 0.0, 'waypoint_out_of_range'
    if math.hypot(x, y) <= limits.goal_tolerance_m:
        return 0.0, 0.0, 'within_tolerance'
    if x < 0:
        return 0.0, 0.0, 'rear_waypoint'
    if abs(x) < 1e-8:
        velocity = 0.0
        angular = math.copysign(math.pi / 2 * limits.observation_hz, y)
    else:
        velocity = x * limits.observation_hz
        angular = math.atan2(y, x) * limits.observation_hz
    return (min(max(velocity, 0.0), limits.max_linear_mps),
            min(max(angular, -limits.max_angular_rps), limits.max_angular_rps), 'tracking')


class SafetyGate:
    def __init__(self, limits):
        self.limits = limits
        self.waypoint = None
        self.camera = None
        self.highest_waypoint_stamp = 0
        self.model_state = 'unknown'
        self.estop_input = False
        self.estop_latched = False
        self.reached_goal = False
        self.last_rejection = 'waiting_waypoint'
        self.last_ros_time = None
        self.clock_fault = False

    def reject(self, reason):
        self.waypoint = None
        self.last_rejection = reason

    def _valid_time(self, stamp, now, timeout):
        return (stamp > 0 and -self.limits.future_tolerance_sec * 1e9
                <= now - stamp <= timeout * 1e9)

    def accept_waypoint(self, x, y, z, stamp, frame, now, monotonic):
        if (not all(math.isfinite(v) for v in (x, y, z)) or abs(z) > 1e-6
                or math.hypot(x, y) > self.limits.max_waypoint_distance_m):
            self.reject('invalid_waypoint')
        elif frame != self.limits.waypoint_frame:
            self.reject('wrong_waypoint_frame')
        elif not self._valid_time(stamp, now, self.limits.source_timeout_sec):
            self.reject('stale_or_future_waypoint')
        elif stamp <= self.highest_waypoint_stamp:
            self.reject('replayed_waypoint')
        else:
            self.highest_waypoint_stamp = stamp
            self.waypoint = (x, y, stamp, monotonic)
            self.last_rejection = None
            return True
        return False

    def accept_camera(self, stamp, frame, now, monotonic):
        if frame != self.limits.camera_frame or not self._valid_time(stamp, now, self.limits.camera_timeout_sec):
            self.camera = None
            return False
        if self.camera and stamp <= self.camera[0]:
            self.camera = None
            return False
        self.camera = (stamp, monotonic)
        return True

    def set_model_state(self, state):
        self.model_state = state
        if state != 'running':
            self.reject('model_not_running')

    def set_estop(self, active):
        self.estop_input = bool(active)
        if active:
            self.estop_latched = True
            self.reject('emergency_stop')

    def reset_estop(self):
        if self.estop_input:
            return False
        self.estop_latched = False
        self.reject('waiting_waypoint_after_reset')
        return True

    def decide(self, now, monotonic):
        if self.last_ros_time is not None and now < self.last_ros_time:
            self.clock_fault = True
            self.reject('clock_rollback')
        self.last_ros_time = now
        reason = None
        if self.clock_fault:
            reason = 'clock_rollback'
        elif self.estop_latched:
            reason = 'emergency_stop'
        elif self.reached_goal:
            reason = 'reached_goal'
        elif self.model_state != 'running':
            reason = 'model_not_running'
        elif self.camera is None:
            reason = 'waiting_camera'
        elif (not self._valid_time(self.camera[0], now, self.limits.camera_timeout_sec)
              or monotonic - self.camera[1] > self.limits.camera_timeout_sec):
            reason = 'camera_timeout'
        elif self.waypoint is None:
            reason = self.last_rejection or 'waiting_waypoint'
        elif not self._valid_time(self.waypoint[2], now, self.limits.source_timeout_sec):
            reason = 'source_image_timeout'
        elif monotonic - self.waypoint[3] > self.limits.waypoint_timeout_sec:
            reason = 'waypoint_timeout'
        if reason:
            if reason in ('camera_timeout', 'source_image_timeout', 'waypoint_timeout'):
                self.reject(reason)
            return 0.0, 0.0, reason
        return waypoint_velocity(self.waypoint[0], self.waypoint[1], self.limits)
