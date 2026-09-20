#!/usr/bin/env python3
"""Bounded, read-only robot-camera -> CUDA NoMaD preview.

One process/context/executor deliberately shares one Fast DDS participant.  No
controller or velocity publisher is constructed.  ROS imports are lazy so the
configuration and evidence checks can be tested without ROS or a robot.
"""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import socket
import statistics
import time
import uuid
import xml.etree.ElementTree as ET


NAMESPACE = '/nomad/robot_preview'
IMAGE_TOPIC = '/camera/color/image_raw/compressed'
OUTPUTS = {
    'waypoint_topic': NAMESPACE + '/waypoint',
    'stamped_waypoint_topic': NAMESPACE + '/waypoint_stamped',
    'sampled_actions_topic': NAMESPACE + '/sampled_actions',
    'trajectory_poses_topic': NAMESPACE + '/trajectory_poses',
    'status_topic': NAMESPACE + '/status',
}
INFRASTRUCTURE_TOPICS = {'/rosout', '/parameter_events'}
XML_NS = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
OUTPUT_STARTUP_GRACE_SEC = 5.0
OUTPUT_SILENCE_TIMEOUT_SEC = 2.0


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def duration(value):
    parsed = float(value)
    if not math.isfinite(parsed) or not 10 <= parsed <= 3600:
        raise argparse.ArgumentTypeError('--seconds must be finite and within 10..3600')
    return parsed


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--seconds', type=duration, default=60.0)
    return parser.parse_args(argv)


def validate_profile(path):
    """Accept only the reviewed Wi-Fi-only, wildcard-bind UDP profile."""
    tree = ET.parse(path)
    root = tree.getroot()
    tags = {element.tag.rsplit('}', 1)[-1].lower() for element in root.iter()}
    require(not any('whitelist' in tag or ('external' in tag and 'locator' in tag) for tag in tags),
            'Do not add interface whitelists or External Locators to the Wi-Fi-only profile')
    participants = root.findall('f:participant', XML_NS)
    require(len(participants) == 1 and participants[0].get('is_default_profile') == 'true',
            'Expected exactly one default participant profile')
    rtps = participants[0].find('f:rtps', XML_NS)
    require(rtps is not None, 'Missing RTPS profile')

    def text_at(parent, query):
        element = parent.find(query, XML_NS)
        return element.text.strip() if element is not None and element.text else None

    require(text_at(rtps, 'f:participantID') == '10', 'Expected participant ID 10')
    require(text_at(rtps, 'f:useBuiltinTransports') == 'false', 'Builtin transports must be disabled')
    transports = root.findall('f:transport_descriptors/f:transport_descriptor', XML_NS)
    require(len(transports) == 1 and text_at(transports[0], 'f:type') == 'UDPv4',
            'Expected exactly one UDPv4 transport')
    transport_id = text_at(transports[0], 'f:transport_id')
    user_transports = rtps.findall('f:userTransports/f:transport_id', XML_NS)
    require(len(user_transports) == 1 and user_transports[0].text.strip() == transport_id,
            'Participant must use the reviewed UDP transport')
    checked = []
    for query, expected_ip, expected_port in (
        ('f:defaultUnicastLocatorList', '192.168.0.180', '7431'),
        ('f:builtin/f:metatrafficUnicastLocatorList', '192.168.0.180', '7430'),
        ('f:builtin/f:initialPeersList', '192.168.0.100', None),
    ):
        locators = rtps.findall(query + '/f:locator', XML_NS)
        require(len(locators) == 1, f'Expected exactly one locator in {query}')
        locator = locators[0]
        require(len(locator) == 1 and locator[0].tag.endswith('}udpv4'), 'Only UDPv4 locators are allowed')
        address = text_at(locator, 'f:udpv4/f:address')
        port = text_at(locator, 'f:udpv4/f:port')
        require(address == expected_ip and port == expected_port,
                f'Unexpected locator in {query}: {address}:{port}')
        checked.append({'address': address, 'port': int(port) if port else None})
    require(len(root.findall('.//f:locator', XML_NS)) == 3,
            'Unexpected additional locators; refusing unreviewed network configuration')
    require(len(root.findall('.//f:address', XML_NS)) == 3,
            'Unexpected additional transport/interface addresses')
    return checked


def validate_environment(root, env=None):
    env = os.environ if env is None else env
    for key, value in {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0',
                       'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}.items():
        require(env.get(key) == value, f'{key} must be {value!r}')
    expected = (root / 'docker/ros2/fastdds_extlab/robot_wifi_only.xml').resolve()
    actual = env.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')
    require(actual and Path(actual).resolve() == expected,
            f'FASTRTPS_DEFAULT_PROFILES_FILE must be {expected}')
    alternate = env.get('FASTDDS_DEFAULT_PROFILES_FILE')
    require(not alternate or Path(alternate).resolve() == expected,
            'Conflicting FASTDDS_DEFAULT_PROFILES_FILE')
    require(env.get('ROS_DISCOVERY_SERVER', '') == '', 'ROS_DISCOVERY_SERVER must be unset')
    require(env.get('SKIP_DEFAULT_XML', '0') != '1', 'SKIP_DEFAULT_XML must not disable the profile')
    return {'profile': str(expected), 'locators': validate_profile(expected)}


def require_free_ports(ports=(7430, 7431), socket_factory=socket.socket):
    """Do not enable socket reuse: a running RViz/probe must fail this check."""
    sockets = []
    try:
        for port in ports:
            sock = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
            sockets.append(sock)
            try:
                sock.bind(('0.0.0.0', port))
            except OSError as error:
                raise RuntimeError(f'UDP {port} is occupied; stop the other fixed-port RViz/probe first') from error
    finally:
        for sock in sockets:
            sock.close()


def distribution(values):
    if not values:
        return {'count': 0, 'mean': None, 'p95': None, 'max': None}
    ordered = sorted(values)
    return {'count': len(values), 'mean': statistics.fmean(values),
            'p95': ordered[max(0, math.ceil(len(ordered) * .95) - 1)], 'max': ordered[-1]}


def require_output_liveness(now, ready_at, last_point_at):
    """Fail a live-camera run that never produces points or later stops doing so."""
    if now - ready_at <= OUTPUT_STARTUP_GRACE_SEC:
        return
    require(last_point_at is not None,
            'No actual DDS preview waypoint after the 5 second model startup grace')
    require(now - last_point_at <= OUTPUT_SILENCE_TIMEOUT_SEC,
            'Actual DDS preview waypoint output stalled for more than 2 seconds')


def validate_publisher_graph(publishers):
    allowed = set(OUTPUTS.values()) | INFRASTRUCTURE_TOPICS
    require(all(topic in allowed for topic, _ in publishers),
            f'Model advertised an unexpected output: {publishers}')
    require(not any('Twist' in type_name for _, types in publishers for type_name in types),
            'A velocity publisher is forbidden in robot preview')
    actual = {topic for topic, _ in publishers}
    require(set(OUTPUTS.values()) <= actual, 'Missing expected private model publishers')
    return [{'topic': topic, 'types': types} for topic, types in publishers]


def match_points_to_rows(points, rows):
    require(bool(points) and bool(rows), 'No actual DDS points or saved inference rows')
    by_stamp = {row['observation_stamp_ns']: row for row in rows}
    require(len(by_stamp) == len(rows), 'Duplicate inference observation timestamps')
    require(len({point['stamp_ns'] for point in points}) == len(points), 'Duplicate DDS point timestamps')
    require(len(points) == len(rows), f'DDS point count {len(points)} differs from saved rows {len(rows)}')
    for point in points:
        row = by_stamp.get(point['stamp_ns'])
        require(row is not None, 'DDS point timestamp absent from inference.jsonl')
        require(point['frame_id'] == 'base_link', 'DDS point is not in base_link')
        require(all(math.isfinite(value) for value in point['xyz']), 'DDS point is nonfinite')
        require(math.isclose(point['xyz'][0], row['waypoint'][0], abs_tol=1e-6)
                and math.isclose(point['xyz'][1], row['waypoint'][1], abs_tol=1e-6)
                and point['xyz'][2] == 0.0, 'DDS point differs from saved inference waypoint')


def model_parameters(root, output):
    return {
        'model_config': str(root / 'visualnav-transformer/train/config/nomad.yaml'),
        'checkpoint': str(root / 'visualnav-transformer/deployment/model_weights/nomad.pth'),
        'action_stats_path': str(root / 'visualnav-transformer/train/vint_train/data/data_config.yaml'),
        'output_root': str(output / 'model'), 'run_name': 'robot-camera',
        'device': 'cuda', 'warmup': True, 'image_transport': 'compressed',
        'image_topic': IMAGE_TOPIC, 'output_frame_id': 'base_link',
        'observation_hz': 4.0, 'inference_hz': 4.0, 'num_samples': 8,
        'waypoint_index': 2, 'max_speed_mps': 0.2, 'image_timeout_sec': 1.0,
        'max_frame_gap_sec': 0.75, 'future_tolerance_sec': 0.2,
        **OUTPUTS,
    }


def make_audit(context):
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PointStamped, PoseArray
    import numpy as np
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Float32MultiArray

    class Audit(Node):
        def __init__(self):
            super().__init__('robot_preview_audit', namespace=NAMESPACE, context=context,
                             use_global_arguments=False)
            self.bridge = CvBridge()
            self.failure = None
            self.image_count = 0
            self.image_stamps = set()
            self.image_ages = []
            self.image_arrivals = []
            self.headers = deque(maxlen=24)
            self.first_headers = []
            self.points, self.actions, self.poses = [], [], []
            self.steps = None
            self.phase = 'preflight'
            self.create_subscription(CompressedImage, IMAGE_TOPIC, self.on_image,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        def on_image(self, message):
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            evidence = {'stamp_sec': message.header.stamp.sec,
                        'stamp_nanosec': message.header.stamp.nanosec,
                        'stamp_ns': stamp, 'frame_id': message.header.frame_id,
                        'age_sec': age, 'bytes': len(message.data), 'phase': self.phase}
            self.headers.append(evidence)
            if len(self.first_headers) < 6:
                self.first_headers.append(evidence)
            try:
                require(stamp > 0 and message.header.frame_id and -.2 <= age <= 1.0,
                        f'Camera header is invalid/stale/future: {evidence}')
                require(bool(message.data), 'Camera sent an empty compressed image')
                rgb = self.bridge.compressed_imgmsg_to_cv2(message, desired_encoding='rgb8')
                require(rgb.dtype == np.uint8 and rgb.ndim == 3 and rgb.shape[2] == 3
                        and rgb.shape[0] > 0 and rgb.shape[1] > 0, 'Camera JPEG is not valid RGB')
                self.image_stamps.add(stamp)
                self.image_count += 1
                self.image_ages.append(age)
                self.image_arrivals.append(time.monotonic())
            except Exception as error:
                self.failure = f'Input validation failed: {error}'

        def subscribe_outputs(self):
            self.create_subscription(PointStamped, OUTPUTS['stamped_waypoint_topic'], self.on_point, 100)
            self.create_subscription(Float32MultiArray, OUTPUTS['sampled_actions_topic'], self.on_actions, 100)
            self.create_subscription(PoseArray, OUTPUTS['trajectory_poses_topic'], self.on_poses, 100)

        def on_point(self, message):
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            xyz = [message.point.x, message.point.y, message.point.z]
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            self.points.append({'stamp_ns': stamp, 'frame_id': message.header.frame_id,
                                'xyz': xyz, 'age_sec': age, 'arrival_monotonic': time.monotonic()})
            if not (stamp > 0 and message.header.frame_id == 'base_link'
                    and all(math.isfinite(value) for value in xyz) and -.2 <= age <= 1.0):
                self.failure = 'Received invalid, nonfinite, stale or future DDS waypoint'

        def on_actions(self, message):
            values = list(message.data)
            self.actions.append({'length': len(values), 'finite': all(math.isfinite(v) for v in values)})
            if (not values or not self.actions[-1]['finite'] or values[0] != 0.0
                    or (self.steps is not None and len(values) != 1 + 8 * self.steps * 2)):
                self.failure = 'Received malformed DDS sampled actions'

        def on_poses(self, message):
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            finite = all(math.isfinite(value) for pose in message.poses for value in (
                pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                pose.orientation.y, pose.orientation.z, pose.orientation.w))
            self.poses.append({'stamp_ns': stamp, 'frame_id': message.header.frame_id,
                               'length': len(message.poses), 'finite': finite})
            if (stamp <= 0 or message.header.frame_id != 'base_link' or not finite
                    or (self.steps is not None and len(message.poses) != 8 * self.steps)):
                self.failure = 'Received malformed DDS trajectory poses'

    return Audit()


def main(argv=None):
    args = parse_args(argv)
    root = args.project_root.resolve()
    output = root / 'outputs/ros2_robot_inference' / (
        'robot-camera__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'created_utc': datetime.now(timezone.utc).isoformat(),
              'output': str(output), 'requested_model_ready_seconds': args.seconds,
              'mode': 'preview_only_no_controller_no_velocity', 'checks': [],
              'timestamp_policy': 'Original camera headers; never re-stamped',
              'continuous_preview_output': {
                  'startup_grace_sec': OUTPUT_STARTUP_GRACE_SEC,
                  'max_point_silence_sec': OUTPUT_SILENCE_TIMEOUT_SEC,
                  'last_point_age_at_end_sec': None,
                  'end_reference': 'bounded inference end, or failure before completion'},
              'counts': {}}
    print(f'Robot inference report: {output}', flush=True)
    context = executor = audit = model = None
    ready_at = ended_at = None
    ready_image_count = 0
    result = 1
    try:
        report['network'] = validate_environment(root)
        report['checks'].append('reviewed_wifi_only_profile_and_environment')
        require_free_ports()
        report['checks'].append('fixed_udp_ports_free_before_context_init')
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from nomad_navigation.nomad_node import NoMaDNode

        context = Context()
        rclpy.init(args=[], context=context)
        executor = SingleThreadedExecutor(context=context)
        audit = make_audit(context)
        executor.add_node(audit)

        def check_health():
            require(context.ok(), 'ROS context shut down before validation completed')
            require(audit.failure is None, audit.failure or '')
            if model is not None:
                require(not model._faulted, 'NoMaD reported a model fault')
                now = time.monotonic()
                # The model retains its own freshness guard. A bounded real
                # camera acceptance must also not pass on old early-run points
                # after the camera disconnects for the rest of the run.
                if ready_at is not None and now - ready_at > 1.0:
                    require(bool(audit.image_arrivals) and now - audit.image_arrivals[-1] <= 1.0,
                            'Robot camera stream silent for more than the 1 second safety timeout')
                if ready_at is not None:
                    require_output_liveness(now, ready_at,
                                            audit.points[-1]['arrival_monotonic'] if audit.points else None)

        preflight_at = time.monotonic()
        while time.monotonic() - preflight_at < 15.0:
            executor.spin_once(timeout_sec=.05)
            check_health()
            if len(audit.image_stamps) >= 3 and time.monotonic() - preflight_at >= 1.0:
                break
        require(len(audit.image_stamps) >= 3, 'No three distinct fresh robot camera frames within 15 seconds')
        for topic in OUTPUTS.values():
            require(audit.count_publishers(topic) == 0 and audit.count_subscribers(topic) == 0,
                    f'Private preview topic is already occupied: {topic}')
        report['preflight'] = {'seconds': time.monotonic() - preflight_at,
                               'fresh_distinct_frames': len(audit.image_stamps)}
        report['checks'].extend(['fresh_camera_images_before_model_load', 'private_output_topics_unoccupied'])
        audit.subscribe_outputs()
        parameters = model_parameters(root, output)
        model = NoMaDNode(context=context, namespace=NAMESPACE, use_global_arguments=False,
                          parameter_overrides=[Parameter(name, value=value) for name, value in parameters.items()])
        executor.add_node(model)
        ready_at = time.monotonic()
        ready_image_count = audit.image_count
        # Model loading is synchronous; do not misreport its pause as a camera
        # transport gap. These metrics measure callbacks only after readiness.
        audit.image_ages.clear()
        audit.image_arrivals.clear()
        audit.phase = 'model_ready'
        audit.steps = model.backend.config['len_traj_pred']
        report['node_run'] = str(model.run_dir)
        report['runtime'] = model.backend.runtime_info
        require(str(report['runtime'].get('device', '')).startswith('cuda')
                and report['runtime'].get('cuda_runtime') and report['runtime'].get('gpu'),
                'Model is not actually running on a CUDA device')
        report['checks'].append('actual_model_cuda_runtime')
        graph_deadline = time.monotonic() + 2.0
        publishers = []
        while time.monotonic() < graph_deadline:
            publishers = audit.get_publisher_names_and_types_by_node(model.get_name(), model.get_namespace())
            if set(OUTPUTS.values()) <= {topic for topic, _ in publishers}:
                break
            executor.spin_once(timeout_sec=.05)
            check_health()
        report['model_publishers'] = validate_publisher_graph(publishers)
        report['checks'].append('actual_model_publisher_allowlist_no_twist')
        print('CUDA model ready; timing begins. Private preview topics only; no motion commands.', flush=True)
        next_progress = ready_at + 10.0
        while time.monotonic() - ready_at < args.seconds:
            executor.spin_once(timeout_sec=.05)
            check_health()
            if time.monotonic() >= next_progress:
                print(json.dumps({'elapsed_sec': round(time.monotonic() - ready_at, 2),
                                  'input_frames': audit.image_count, 'dds_points': len(audit.points),
                                  'model_state': model._state, 'model_published': model.published_count}), flush=True)
                next_progress += 10.0
        ended_at = time.monotonic()
        model._timer.cancel()
        # Let this same executor deliver already published DDS messages, without
        # extending the inference run or making an extra ROS participant.
        drain_until = time.monotonic() + .5
        while time.monotonic() < drain_until:
            executor.spin_once(timeout_sec=.02)
            check_health()
        rows = [json.loads(line) for line in (model.run_dir / 'inference.jsonl').read_text().splitlines() if line]
        match_points_to_rows(audit.points, rows)
        require(len(audit.points) == model.published_count, 'Model/DDS publication counts differ')
        require(len(audit.actions) == len(rows) and len(audit.poses) == len(rows),
                'Not all DDS action/pose messages were received')
        require([pose['stamp_ns'] for pose in audit.poses] == [point['stamp_ns'] for point in audit.points],
                'DDS trajectory and waypoint timestamps differ')
        report['inference_ms'] = distribution([row['inference_ms'] for row in rows])
        report['checks'].extend(['actual_dds_waypoints_match_saved_inference',
                                 'actual_dds_actions_and_poses_valid', 'original_stamps_base_link_finite',
                                 'continuous_preview_output'])
        # Inspect once more, catching output changes after discovery settles.
        report['model_publishers'] = validate_publisher_graph(
            audit.get_publisher_names_and_types_by_node(model.get_name(), model.get_namespace()))
        report['status'] = 'passed'
        result = 0
    except KeyboardInterrupt:
        report.update(status='interrupted', error='Interrupted before bounded acceptance completed')
    except Exception as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
        print(report['error'], flush=True)
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            acceptance_end = ended_at if ended_at is not None else time.monotonic()
            if ready_at is not None:
                report['actual_model_ready_seconds'] = acceptance_end - ready_at
            if audit is not None:
                points_before_end = [point for point in audit.points
                                     if point['arrival_monotonic'] <= acceptance_end]
                if points_before_end:
                    report['continuous_preview_output']['last_point_age_at_end_sec'] = (
                        acceptance_end - points_before_end[-1]['arrival_monotonic'])
                report['counts'].update(input_frames=audit.image_count, dds_points=len(audit.points),
                                         dds_actions=len(audit.actions), dds_poses=len(audit.poses))
                report['counts']['input_frames_after_model_ready'] = audit.image_count - ready_image_count if ready_at else 0
                report['input_metric_scope'] = ('audit callbacks after model ready (not sensor hardware FPS)'
                                                if ready_at else 'preflight audit callbacks')
                report['input_age_sec'] = distribution(audit.image_ages)
                report['input_gap_sec'] = distribution([b - a for a, b in zip(
                    audit.image_arrivals, audit.image_arrivals[1:])])
                report['point_age_sec'] = distribution([point['age_sec'] for point in audit.points])
                report['point_gap_sec'] = distribution([b['arrival_monotonic'] - a['arrival_monotonic']
                                                        for a, b in zip(audit.points, audit.points[1:])])
                report['input_header_evidence'] = {'first': audit.first_headers, 'last': list(audit.headers)}
            cleanup_errors = []
            if model is not None:
                report['counts']['model_published'] = model.published_count
                try:
                    model.destroy_node()
                except Exception as error:
                    cleanup_errors.append(f'model: {error!r}')
            if audit is not None:
                try:
                    audit.destroy_node()
                except Exception as error:
                    cleanup_errors.append(f'audit: {error!r}')
            if executor is not None:
                try:
                    executor.shutdown(timeout_sec=2.0)
                except Exception as error:
                    cleanup_errors.append(f'executor: {error!r}')
            if context is not None and context.ok():
                try:
                    context.shutdown()
                except Exception as error:
                    cleanup_errors.append(f'context: {error!r}')
            if audit is not None:
                try:
                    (output / 'dds_points.json').write_text(json.dumps(audit.points, indent=2) + '\n', encoding='utf-8')
                except Exception as error:
                    cleanup_errors.append(f'DDS evidence write: {error!r}')
            if model is not None and model.run_dir is not None:
                try:
                    rows = [json.loads(line) for line in (model.run_dir / 'inference.jsonl').read_text().splitlines() if line]
                    report['inference_ms'] = distribution([row['inference_ms'] for row in rows])
                    report['model_final_manifest_status'] = json.loads(
                        (model.run_dir / 'manifest.json').read_text())['status']
                except Exception as error:
                    cleanup_errors.append(f'model evidence read: {error!r}')
            if cleanup_errors:
                report.update(status='failed', cleanup_errors=cleanup_errors)
                result = 1
            report['finished_utc'] = datetime.now(timezone.utc).isoformat()
            (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        finally:
            signal.signal(signal.SIGINT, previous)
    print(json.dumps({'status': report['status'], 'report': str(output / 'report.json')}), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
