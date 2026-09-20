"""Read-only local decision markers for a live robot-camera run.

Call ``make_visualizer(context)`` with the inference process's existing context,
then add the returned node to its executor. This module never initializes ROS,
loads a controller, publishes a velocity, or broadcasts a fabricated transform.
RViz itself uses participant 11; this node shares inference's participant 10.

Humble RViz's ImageTransportDisplay derives the compressed transport from the
topic suffix, so nomad_robot_live.rviz directly uses the /compressed topic.
It needs the compressed_image_transport plugin, not another raw-image relay.
"""

import json
import math
import time
import xml.etree.ElementTree as ET


NAMESPACE = '/nomad/robot_preview'
IMAGE_TOPIC = '/camera/color/image_raw/compressed'
MARKER_TOPIC = NAMESPACE + '/visualization'
VISUALIZATION_PARAMETERS = {
    'frame_id': 'base_link',
    'trajectory_topic': NAMESPACE + '/trajectory_poses',
    'waypoint_topic': NAMESPACE + '/waypoint_stamped',
    'model_status_topic': NAMESPACE + '/status',
    'controller_status_topic': '/nomad/robot_control/status',
    'velocity_topic': '/nomad/robot_control/cmd_vel_preview',
    'marker_topic': MARKER_TOPIC,
}
XML_NS = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}


def _finite_number(value):
    try:
        return (not isinstance(value, bool) and isinstance(value, (int, float))
                and math.isfinite(value))
    except OverflowError:
        return False


def control_mode_text(status, received_monotonic, now_ns, monotonic,
                      timeout_sec=.6, future_tolerance_sec=.2):
    """Describe a fresh reported mode, never infer arming from old status.

    The publisher's monotonic deadline is on another computer and must not be
    compared with this process's clock. Only its reported remaining duration is
    used, reduced by source age and local time since receipt.
    """
    if not isinstance(status, dict):
        return 'UNKNOWN [no control status]'
    stamp = status.get('time_ns')
    if (type(stamp) is not int or stamp <= 0 or type(now_ns) is not int or now_ns <= 0
            or not _finite_number(received_monotonic) or not _finite_number(monotonic)
            or received_monotonic < 0 or monotonic < 0):
        return 'UNKNOWN [invalid control status]'
    try:
        source_age = (now_ns - stamp) / 1e9
    except OverflowError:
        return 'UNKNOWN [invalid control status]'
    receipt_age = monotonic - received_monotonic
    if (not -future_tolerance_sec <= source_age <= timeout_sec
            or not 0 <= receipt_age <= timeout_sec):
        return 'UNKNOWN [control status stale]'
    armed = status.get('armed')
    mode = status.get('active_mode')
    duration = status.get('requested_duration_sec')
    remaining = status.get('lease_remaining_sec')
    if (armed is False and mode is None and duration is None
            and _finite_number(remaining) and remaining == 0
            and status.get('run_until_disarmed') is False):
        return 'DISARMED'
    if armed is not True:
        return 'UNKNOWN [invalid control status]'
    if (mode == 'continuous' and _finite_number(duration) and duration == 0
            and remaining is None and status.get('run_until_disarmed') is True):
        return 'CONTINUOUS'
    if (mode == 'timed' and _finite_number(duration) and 0 < duration <= 10
            and _finite_number(remaining) and 0 < remaining <= duration
            and status.get('run_until_disarmed') is False):
        remaining -= max(0.0, source_age, receipt_age)
        if remaining > 0:
            return f'TIMED {remaining:.1f} s remaining'
        return 'UNKNOWN [timed status expired]'
    return 'UNKNOWN [invalid control status]'


def live_status_text(has_prediction, velocity, fresh_velocity, model_state, controller_state,
                     mode_label='UNKNOWN [no control status]'):
    """Describe commands, never claim that an unobserved robot actually moved."""
    v, w = velocity if fresh_velocity else (0.0, 0.0)
    prediction = 'fresh prediction' if has_prediction else 'NO FRESH PREDICTION'
    stale = '' if fresh_velocity else ' [command stale]'
    return (f'LIVE | NO ODOMETRY | {prediction}\n'
            f'command v={v:.3f} m/s  w={w:.3f} rad/s{stale}\n'
            f'model: {model_state} | control: {controller_state}\n'
            f'run: {mode_label}\n'
            'Local predictions; command is not measured motion')


def make_visualizer(context):
    """Return a marker-only node, without creating another DDS participant."""
    from rclpy.parameter import Parameter
    from nomad_navigation.visualization_node import VisualizationNode

    class LiveVisualizationNode(VisualizationNode):
        def __init__(self, **kwargs):
            self._control_status = None
            self._control_received = None
            self._control_highest_stamp = 0
            self._live_last_monotonic = None
            super().__init__(**kwargs)

        def _controller(self, msg):
            super()._controller(msg)
            self._control_status = None
            self._control_received = None
            try:
                value = json.loads(msg.data)
                now, mono = self.get_clock().now().nanoseconds, time.monotonic()
                label = control_mode_text(value, mono, now, mono,
                                          self.p['velocity_timeout_sec'],
                                          self.p['future_tolerance_sec'])
                if label.startswith('UNKNOWN') or value['time_ns'] <= self._control_highest_stamp:
                    return
                self._control_highest_stamp = value['time_ns']
                self._control_status, self._control_received = value, mono
            except (ValueError, KeyError, TypeError, OverflowError):
                pass

        def render(self):
            now, mono = self.get_clock().now().nanoseconds, time.monotonic()
            if ((self.last_ros is not None and now < self.last_ros)
                    or (self._live_last_monotonic is not None and mono < self._live_last_monotonic)):
                self._control_status = None
                self._control_received = None
            self._live_last_monotonic = mono
            # Reuse pairing, expiration, finite-value and ROS rollback guards.
            result = super().render()
            fresh = bool(self.velocity and
                         time.monotonic() - self.velocity[2] <= self.p['velocity_timeout_sec'])
            velocity = self.velocity[:2] if fresh else (0.0, 0.0)
            mode_label = control_mode_text(self._control_status, self._control_received, now, mono,
                                           self.p['velocity_timeout_sec'],
                                           self.p['future_tolerance_sec'])
            state = 'unknown' if mode_label.startswith('UNKNOWN') else self.controller_state
            for marker in result.markers:
                if marker.ns == 'status' and marker.id == 0:
                    marker.text = live_status_text(bool(self.pair), velocity, fresh,
                                                  self.model_state, state, mode_label)
            return result

    if context is None:
        raise ValueError('Use the existing inference ROS context; do not create a second participant')
    return LiveVisualizationNode(
        context=context, namespace=NAMESPACE, use_global_arguments=False,
        parameter_overrides=[Parameter(name, value=value)
                             for name, value in VISUALIZATION_PARAMETERS.items()])


def _validate_pc_live_profile(path, participant_id, meta_port, data_port, peer_port):
    """Review the exact local-loopback + robot-facing Wi-Fi locator sets."""
    root = ET.parse(path).getroot()
    tags = {item.tag.rsplit('}', 1)[-1].lower() for item in root.iter()}
    if any('whitelist' in tag or ('external' in tag and 'locator' in tag) for tag in tags):
        raise ValueError('Live PC profiles must not use an interface whitelist or External Locators')
    participants = root.findall('f:participant', XML_NS)
    if len(participants) != 1 or participants[0].get('is_default_profile') != 'true':
        raise ValueError('Expected exactly one default live PC participant profile')
    rtps = participants[0].find('f:rtps', XML_NS)

    def text_at(parent, query):
        element = parent.find(query, XML_NS) if parent is not None else None
        return element.text.strip() if element is not None and element.text else None

    if (text_at(rtps, 'f:participantID') != str(participant_id)
            or text_at(rtps, 'f:useBuiltinTransports') != 'false'):
        raise ValueError(f'Expected participant {participant_id} and disabled builtin transports')
    transports = root.findall('f:transport_descriptors/f:transport_descriptor', XML_NS)
    used_transports = rtps.findall('f:userTransports/f:transport_id', XML_NS)
    if (len(transports) != 1 or text_at(transports[0], 'f:type') != 'UDPv4'
            or len(used_transports) != 1
            or used_transports[0].text != text_at(transports[0], 'f:transport_id')):
        raise ValueError('Expected the single reviewed UDPv4 transport')
    expected_lists = {
        'f:defaultUnicastLocatorList': [('127.0.0.1', str(data_port)), ('192.168.0.180', str(data_port))],
        'f:builtin/f:metatrafficUnicastLocatorList': [('127.0.0.1', str(meta_port)), ('192.168.0.180', str(meta_port))],
        'f:builtin/f:initialPeersList': [('127.0.0.1', str(peer_port)), ('192.168.0.100', None)],
    }
    for query, expected in expected_lists.items():
        locators = rtps.findall(query + '/f:locator', XML_NS)
        actual = []
        for locator in locators:
            if len(locator) != 1 or not locator[0].tag.endswith('}udpv4'):
                raise ValueError('Only UDPv4 live PC locators are allowed')
            actual.append((text_at(locator, 'f:udpv4/f:address'), text_at(locator, 'f:udpv4/f:port')))
        if actual != expected:
            raise ValueError(f'Unexpected live PC locator list {query}: {actual}')
    if (len(root.findall('.//f:locator', XML_NS)) != 6
            or len(root.findall('.//f:address', XML_NS)) != 6):
        raise ValueError('Unexpected extra live PC addresses or locators')
    return {'participant_id': participant_id, 'metatraffic_port': meta_port, 'data_port': data_port,
            'announced_addresses': ['127.0.0.1', '192.168.0.180'],
            'initial_peers': [f'127.0.0.1:{peer_port}', '192.168.0.100:standard-discovery']}


def validate_rviz_profile(path):
    """Require the reviewed local+Wi-Fi RViz profile, never inference ports."""
    return _validate_pc_live_profile(path, 11, 7432, 7433, 7430)


def validate_live_profile(path):
    """Require the new joint-live inference profile; old preview stays separate."""
    return _validate_pc_live_profile(path, 10, 7430, 7431, 7432)
