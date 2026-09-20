"""Robot-local 20 Hz desired-command producer, never a /cmd_vel publisher.

The independent robot-local gateway is mandatory: it must expire desired_cmd
when this process, its executor, the PC link, or the Python runtime stalls.
"""

import json
import os
import signal
import time

from geometry_msgs.msg import PointStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from nomad_interfaces.srv import ArmFor
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from nomad_navigation.robot_control_core import RobotControlCore, RobotControlLimits
from nomad_navigation.operator_guard import (
    GUARD_ENV, OPERATOR_LINK_TOPIC, OperatorGuard, operator_pause_resume_from_env,
)


TOPICS = {
    'waypoint': '/nomad/robot_preview/waypoint_stamped',
    'model': '/nomad/robot_preview/status',
    'camera': '/camera/color/image_raw/compressed',
    'odom': '/odom',
    'estop': '/nomad/robot_control/set_estop',
    'desired': '/nomad/robot_control/desired_cmd',
    'preview': '/nomad/robot_control/cmd_vel_preview',
    'status': '/nomad/robot_control/status',
}
SERVICES = {'arm': '/nomad/robot_control/arm',
            'arm_for': '/nomad/robot_control/arm_for',
            'disarm': '/nomad/robot_control/disarm'}


class RobotControlNode(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_control', namespace='/nomad', **kwargs)
        self._desired_pub = self._preview_pub = self._status_pub = None
        self.core = None
        self._timer = None
        self._closed = False
        self._model_session = None
        self.operator_guard = None
        try:
            operator_pause_resume = operator_pause_resume_from_env(os.environ)
            if GUARD_ENV in os.environ:
                # An explicitly configured but invalid/empty session is fatal;
                # never silently fall back to the legacy unguarded arm services.
                self.operator_guard = OperatorGuard(os.environ[GUARD_ENV],
                                                     pause_resume=operator_pause_resume)
            if self.get_fully_qualified_name() != '/nomad/robot_control':
                raise ValueError('Robot control node identity remapping is forbidden')
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Robot control requires real ROS system time, not use_sim_time')
            for topic in TOPICS.values():
                if self.resolve_topic_name(topic) != topic:
                    raise ValueError(f'Robot control topic remapping is forbidden: {topic}')
            if (self.operator_guard is not None
                    and self.resolve_topic_name(OPERATOR_LINK_TOPIC) != OPERATOR_LINK_TOPIC):
                raise ValueError('Operator-link topic remapping is forbidden')
            for service in SERVICES.values():
                if self.resolve_service_name(service) != service:
                    raise ValueError(f'Robot control service remapping is forbidden: {service}')
            for key, value in RobotControlLimits().__dict__.items():
                self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
            limits = RobotControlLimits(**{key: self.get_parameter(key).value
                                           for key in RobotControlLimits.__dataclass_fields__})
            self.declare_parameter('network_pause_resume', False, ParameterDescriptor(read_only=True))
            self.core = RobotControlCore(limits,
                network_pause_resume=self.get_parameter('network_pause_resume').value,
                operator_pause_resume=operator_pause_resume)
            command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE,
                                     lifespan=Duration(seconds=.2))
            self._desired_pub = self.create_publisher(TwistStamped, TOPICS['desired'], command_qos)
            self._preview_pub = self.create_publisher(Twist, TOPICS['preview'], command_qos)
            self._status_pub = self.create_publisher(String, TOPICS['status'], QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(PointStamped, TOPICS['waypoint'], self._on_waypoint, 1)
            self.create_subscription(String, TOPICS['model'], self._on_model,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CompressedImage, TOPICS['camera'], self._on_camera, sensor_qos)
            self.create_subscription(Odometry, TOPICS['odom'], self._on_odom, sensor_qos)
            self.create_subscription(Bool, TOPICS['estop'], self._on_estop, 1)
            if self.operator_guard is not None:
                self.create_subscription(String, OPERATOR_LINK_TOPIC, self._on_operator_link,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE))
            self.create_service(Trigger, SERVICES['arm'], self._arm)
            self.create_service(ArmFor, SERVICES['arm_for'], self._arm_for)
            self.create_service(Trigger, SERVICES['disarm'], self._disarm)
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._timer = self.create_timer(.05, self._tick, clock=self._steady_clock)
            self._publish(0.0, 0.0)
        except Exception:
            self.destroy_node()
            raise

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    def _now(self):
        return self.get_clock().now().nanoseconds, time.monotonic()

    def _on_waypoint(self, message):
        now, steady = self._now()
        was_armed = self.core.armed
        accepted = self.core.accept_waypoint(message.point.x, message.point.y, message.point.z,
                                             self._stamp(message), message.header.frame_id, now, steady)
        if not accepted or (was_armed and not self.core.armed):
            self._publish(0.0, 0.0)

    def _on_camera(self, message):
        # No JPEG decoding or disk I/O in the robot watchdog process. The model
        # independently decodes the full payload; here check a JPEG envelope and
        # the original local sensor timestamp/frame without rewriting either.
        data = message.data
        accepted = False
        if len(data) < 4 or bytes(data[:2]) != b'\xff\xd8' or bytes(data[-2:]) != b'\xff\xd9':
            self.core.reject_camera()
        else:
            now, steady = self._now()
            accepted = self.core.accept_camera(self._stamp(message), message.header.frame_id, now, steady)
        if not accepted:
            self._publish(0.0, 0.0)

    def _on_model(self, message):
        try:
            payload = json.loads(message.data)
            state = payload['state']
            if not isinstance(state, str):
                raise ValueError('Model state is not a string')
            if self.core.network_pause_resume:
                session = payload.get('session_id')
                if (payload.get('network_pause_resume') is not True
                        or payload.get('fatal') is not False
                        or not isinstance(session, str) or len(session) != 36):
                    raise ValueError('Expected the opted-in, nonfatal campus model session')
                if self._model_session is None:
                    self._model_session = session
                if session != self._model_session:
                    raise ValueError('A restarted model cannot inherit continuous motion intent')
        except (ValueError, KeyError, TypeError):
            state = 'unknown'
        self.core.set_model_state(state)
        if not self.core.armed or self.core.motion_paused:
            self._publish(0.0, 0.0)

    def _on_odom(self, message):
        now, steady = self._now()
        twist = message.twist.twist
        accepted = self.core.accept_odom((twist.linear.x, twist.linear.y, twist.linear.z),
                                         (twist.angular.x, twist.angular.y, twist.angular.z),
                                         self._stamp(message), message.header.frame_id, message.child_frame_id,
                                         now, steady)
        if not accepted:
            self._publish(0.0, 0.0)

    def _on_estop(self, message):
        self.core.set_estop(message.data)
        if not self.core.armed:
            self._publish(0.0, 0.0)

    def _arm(self, request, response):
        if self.operator_guard is not None:
            return self._reject_unguarded_arm(response)
        now, steady = self._now()
        response.success, response.message = self.core.arm(now, steady)
        if not self.core.armed:
            self._publish(0.0, 0.0)
        return response

    def _disarm(self, request, response):
        if self.operator_guard is not None:
            self.operator_guard.revoke('operator_disarmed')
        self.core.disarm('operator_disarmed')
        # The logical disarm is unconditional, even when input is invalid or
        # ROS time has rolled back. Publishing failure cannot reject disarm.
        response.success = True
        response.message = 'Disarmed; a new explicit arm is required'
        try:
            self._publish(0.0, 0.0)
        except Exception:
            pass  # The independent gateway expires its own input lease.
        return response

    def _arm_for(self, request, response):
        if self.operator_guard is not None:
            return self._reject_unguarded_arm(response)
        now, steady = self._now()
        response.success, response.message = self.core.arm_for(
            now, steady, request.duration_sec, mode=request.mode)
        if not self.core.armed:
            self._publish(0.0, 0.0)
        return response

    def _tick(self):
        operator_ready = self._operator_ready()
        now, steady = self._now()
        speed, angular, _ = self.core.tick(now, steady)
        if not operator_ready:
            speed, angular = 0.0, 0.0
        self._publish(speed, angular)

    def _operator_ready(self):
        if self.operator_guard is None:
            return True
        decision = self.operator_guard.check(time.monotonic_ns())
        self._apply_operator_decision(decision)
        return decision.accepted

    def _apply_operator_decision(self, decision):
        if decision.disarm:
            self.core.disarm(decision.reason)
            self.core.operator_link_connected = decision.accepted
        elif decision.pause:
            now, steady = self._now()
            self.core.operator_link_pause(decision.reason, now, steady)
        if (decision.accepted and decision.generation_changed
                and self.operator_guard.pause_resume):
            now, steady = self._now()
            self.core.operator_link_recovered(now, steady)

    def _reject_unguarded_arm(self, response):
        # A delayed legacy ROS service request must never regain motion after
        # reconnection. Guarded arms are atomic generation+nonce link messages.
        self.core.disarm('operator_link_explicit_confirmation_required')
        response.success = False
        response.message = 'operator_link_use_confirmed_supervisor_arm'
        self._publish(0.0, 0.0)
        return response

    def _on_operator_link(self, message):
        decision = self.operator_guard.accept_json(message.data, time.monotonic_ns())
        self._apply_operator_decision(decision)
        if decision.arm_request is not None:
            request = decision.arm_request
            if self._operator_ready():
                now, steady = self._now()
                success, reason = self.core.arm_for(now, steady, request['duration_sec'],
                                                    mode=request['mode'])
            else:
                success, reason = False, self.operator_guard.reason
            self.operator_guard.record_arm_result(request, success, reason)
            # Emit the result immediately, always starting the newly armed run
            # at zero; the ordinary 20 Hz controller applies all motion gates.
            self._publish(0.0, 0.0)
        elif decision.disarm or decision.pause or not self.core.armed:
            self._publish(0.0, 0.0)

    def _publish(self, speed, angular):
        if self._desired_pub is None:
            return
        if not self._operator_ready() or getattr(self.core, 'operator_paused', False):
            speed, angular = 0.0, 0.0
        now, steady = self._now()
        command = TwistStamped()
        command.header.stamp.sec, command.header.stamp.nanosec = divmod(now, 10**9)
        command.header.frame_id = 'base_link'
        command.twist.linear.x, command.twist.angular.z = float(speed), float(angular)
        self._desired_pub.publish(command)
        self._preview_pub.publish(command.twist)
        state = self.core.snapshot(steady)
        state['operator_link'] = (self.operator_guard.snapshot(time.monotonic_ns())
                                  if self.operator_guard is not None else {'enabled': False})
        state.update(state=state['reason'], time_ns=now, node='/nomad/robot_control',
                     linear_x=float(speed), angular_z=float(angular),
                     robot_local=True, direct_cmd_vel_publisher=False, obstacle_protection=False,
                     mode='supervised_cleared_area_only', hardware_stop_guaranteed=False,
                     limits=self.core.limits.__dict__)
        self._status_pub.publish(String(data=json.dumps(state)))

    def prepare_shutdown(self):
        if self._timer is not None:
            self._timer.cancel()
        if self.core is not None:
            self.core.disarm('shutdown')
        if self._desired_pub is not None and self.context.ok():
            self._publish(0.0, 0.0)
            self._desired_pub.wait_for_all_acked(Duration(seconds=.1))

    def destroy_node(self):
        if self._closed:
            return True
        self._closed = True
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopped = False

    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True

    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    result = 0
    failure = None
    try:
        node = RobotControlNode()
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=.05)
    except Exception as error:
        result = 1
        failure = error
    finally:
        try:
            if node is not None:
                node.prepare_shutdown()
        except Exception as error:
            result = 1
            failure = failure or error
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    # No per-tick logs/file I/O; even error printing happens after the stop path.
    if failure is not None:
        print(f'Robot control stopped: {type(failure).__name__}: {failure}', flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
