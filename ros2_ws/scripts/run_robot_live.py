#!/usr/bin/env python3
"""Live CUDA inference plus RViz; never controls or arms the robot.

Inference, evidence auditing and read-only markers share one ROS context and
participant 10. RViz alone uses the reviewed participant 11 profile. A robot-side
controller, if separately started, must remain independently supervised.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import uuid

import run_robot_inference as core
from robot_live_view import MARKER_TOPIC, make_visualizer, validate_live_profile, validate_rviz_profile


class StopRequested(Exception):
    pass


def live_duration(value):
    """Zero explicitly removes only the PC session deadline, not health checks."""
    if float(value) == 0.0:
        return 0.0
    return core.duration(value)


def session_active(elapsed, seconds):
    return seconds == 0.0 or elapsed < seconds


def require_recording_space(path):
    # Never delete old results. End inference (and hence guarded motion) before
    # unbounded recording exhausts storage; a fresh manual arm is then required.
    core.require(shutil.disk_usage(path).free >= 2 * 1024 ** 3,
                 'Recording volume has less than 2 GiB free; stopping live inference')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--seconds', type=live_duration, default=600.0,
                        help='10..3600 seconds, or 0 for no session deadline (all guards retained)')
    return parser.parse_args(argv)


def validate_existing_subscribers(topic, identities):
    allowed_topics = {core.OUTPUTS['stamped_waypoint_topic'], core.OUTPUTS['status_topic']}
    for name, namespace in identities:
        core.require(topic in allowed_topics and name == 'robot_control' and namespace == '/nomad',
                     f'Unexpected existing subscriber on {topic}: {namespace}/{name}')


def validate_environment(root, env=None):
    """Joint live mode alone accepts the reviewed loopback + Wi-Fi profile.

    The original run_robot_inference validator and Wi-Fi-only profile are not
    changed or relaxed: they still describe the earlier single-process preview.
    """
    env = os.environ if env is None else env
    for key, value in {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0',
                       'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}.items():
        core.require(env.get(key) == value, f'{key} must be {value!r}')
    expected = (root / 'docker/ros2/robot_live_udp.xml').resolve()
    actual = env.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')
    core.require(actual and Path(actual).resolve() == expected,
                 f'Live inference FASTRTPS_DEFAULT_PROFILES_FILE must be {expected}')
    alternate = env.get('FASTDDS_DEFAULT_PROFILES_FILE')
    core.require(not alternate or Path(alternate).resolve() == expected,
                 'Conflicting FASTDDS_DEFAULT_PROFILES_FILE for live inference')
    core.require(env.get('ROS_DISCOVERY_SERVER', '') == '', 'ROS_DISCOVERY_SERVER must be unset')
    core.require(env.get('SKIP_DEFAULT_XML', '0') != '1', 'SKIP_DEFAULT_XML must not disable the profile')
    return {'profile': str(expected), 'locators': validate_live_profile(expected)}


def rviz_environment(root, output, inherited=None):
    env = dict(os.environ if inherited is None else inherited)
    profile = str(root / 'docker/ros2/robot_rviz_udp.xml')
    env.update(ROS_DOMAIN_ID='0', ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               FASTRTPS_DEFAULT_PROFILES_FILE=profile, FASTDDS_DEFAULT_PROFILES_FILE=profile,
               ROS_LOG_DIR=str(output / 'rviz_ros_logs'))
    env.pop('ROS_DISCOVERY_SERVER', None)
    return env


def stop_rviz(process):
    """Only signal the process group created by this run, never another RViz."""
    if process is None or process.poll() is not None:
        return
    for requested_signal, seconds in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(process.pid, requested_signal)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=seconds)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError('Owned RViz process did not terminate')


def main(argv=None):
    args = parse_args(argv)
    root = args.project_root.resolve()
    output = root / 'outputs/ros2_robot_live' / (
        'run__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    manifest = {'status': 'starting', 'created_utc': datetime.now(timezone.utc).isoformat(),
                'output': str(output), 'model_ready_seconds_requested': args.seconds,
                'session_mode': 'continuous' if args.seconds == 0 else 'timed',
                'pc_mode': 'inference_and_markers_only_no_twist_no_automatic_arm',
                'original_camera_stamps_preserved': True, 'checks': [],
                'thresholds': {'image_timeout_sec': 1.0, 'point_stamp_timeout_sec': 1.0,
                               'future_tolerance_sec': .2, 'output_startup_grace_sec': 5.0,
                               'output_silence_timeout_sec': 2.0}}
    log = (output / 'runner.log').open('w', encoding='utf-8')
    context = executor = audit = model = visualizer = rviz = rviz_log = None
    ready_at = ended_at = None
    ready_image_count = 0
    requested_signal = None
    result = 1
    previous_handlers = {}

    def emit(value):
        text = value if isinstance(value, str) else json.dumps(value)
        print(text, flush=True)
        log.write(text + '\n')
        log.flush()

    def write_manifest():
        (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')

    def handle_signal(number, _frame):
        nonlocal requested_signal
        requested_signal = number
        raise StopRequested(signal.Signals(number).name)

    try:
        write_manifest()
        require_recording_space(output)
        emit(f'Live robot run: {output}; no PC velocity publisher, no automatic arming.')
        for number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[number] = signal.signal(number, handle_signal)
        manifest['inference_network'] = validate_environment(root)
        manifest['rviz_network'] = validate_rviz_profile(root / 'docker/ros2/robot_rviz_udp.xml')
        core.require_free_ports((7430, 7431, 7432, 7433))
        core.require(bool(os.environ.get('DISPLAY')), 'DISPLAY is required for the live RViz window')
        manifest['checks'].append('two_reviewed_profiles_and_four_free_ports')
        os.environ['ROS_LOG_DIR'] = str(output / 'model_ros_logs')
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from nomad_navigation.nomad_node import NoMaDNode

        context = Context()
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        executor = SingleThreadedExecutor(context=context)
        audit = core.make_audit(context)
        executor.add_node(audit)

        def check_health():
            if requested_signal is not None:
                raise StopRequested(signal.Signals(requested_signal).name)
            core.require(context.ok(), 'ROS context stopped unexpectedly')
            core.require(audit.failure is None, audit.failure or '')
            if rviz is not None and rviz.poll() is not None:
                if rviz.returncode == 0:
                    raise StopRequested('RViz window closed')
                raise RuntimeError(f'RViz exited unexpectedly with code {rviz.returncode}; see rviz.log')
            if model is not None:
                core.require(not model._faulted, 'NoMaD model fault; stopping the live pipeline')
            if ready_at is not None:
                now = time.monotonic()
                if now - ready_at > 1.0:
                    core.require(bool(audit.image_arrivals) and now - audit.image_arrivals[-1] <= 1.0,
                                 'Camera stream silent beyond the unchanged 1 second timeout')
                core.require_output_liveness(now, ready_at,
                                             audit.points[-1]['arrival_monotonic'] if audit.points else None)

        preflight = time.monotonic()
        while time.monotonic() - preflight < 15.0:
            executor.spin_once(timeout_sec=.05)
            check_health()
            if len(audit.image_stamps) >= 3 and time.monotonic() - preflight >= 1.0:
                break
        core.require(len(audit.image_stamps) >= 3, 'No three distinct fresh camera frames within 15 seconds')
        manifest['preflight_fresh_images'] = len(audit.image_stamps)
        manifest['existing_subscribers'] = {}
        for topic in core.OUTPUTS.values():
            core.require(audit.count_publishers(topic) == 0, f'Another model already publishes {topic}')
            identities = [(info.node_name, info.node_namespace)
                          for info in audit.get_subscriptions_info_by_topic(topic)]
            validate_existing_subscribers(topic, identities)
            manifest['existing_subscribers'][topic] = identities
        manifest['checks'].append('fresh_camera_and_no_competing_model_or_unknown_consumer')
        audit.subscribe_outputs()
        visualizer = make_visualizer(context)
        executor.add_node(visualizer)
        rviz_log = (output / 'rviz.log').open('w', encoding='utf-8')
        rviz = subprocess.Popen(
            ['rviz2', '-d', str(root / 'ros2_ws/src/nomad_navigation/rviz/nomad_robot_live.rviz')],
            env=rviz_environment(root, output), stdout=rviz_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        manifest['rviz_pid'] = rviz.pid
        model = NoMaDNode(context=context, namespace=core.NAMESPACE, use_global_arguments=False,
                          parameter_overrides=[Parameter(name, value=value) for name, value
                                               in core.model_parameters(root, output).items()])
        check_health()
        executor.add_node(model)
        ready_at = time.monotonic()
        ready_image_count = audit.image_count
        audit.image_ages.clear()
        audit.image_arrivals.clear()
        audit.phase = 'model_ready'
        audit.steps = model.backend.config['len_traj_pred']
        manifest.update(status='running', node_run=str(model.run_dir), runtime=model.backend.runtime_info)
        core.require(str(manifest['runtime'].get('device', '')).startswith('cuda')
                     and manifest['runtime'].get('cuda_runtime') and manifest['runtime'].get('gpu'),
                     'The model is not actually using CUDA')
        manifest['checks'].append('actual_cuda_model')
        write_manifest()
        emit('CUDA ready. RViz shows local predictions, not odometry. Robot arming is separate and never automatic.')
        next_progress = ready_at + 10.0
        graph_checked = False
        while session_active(time.monotonic() - ready_at, args.seconds):
            executor.spin_once(timeout_sec=.05)
            check_health()
            now = time.monotonic()
            if not graph_checked and now - ready_at >= 1.0:
                manifest['model_publishers'] = core.validate_publisher_graph(
                    audit.get_publisher_names_and_types_by_node(model.get_name(), model.get_namespace()))
                viewer_publishers = audit.get_publisher_names_and_types_by_node(
                    visualizer.get_name(), visualizer.get_namespace())
                core.require(all(topic in {MARKER_TOPIC} | core.INFRASTRUCTURE_TOPICS
                                 and all('Twist' not in type_name for type_name in types)
                                 for topic, types in viewer_publishers), 'Visualizer publisher allowlist violation')
                core.require(MARKER_TOPIC in {topic for topic, _ in viewer_publishers},
                             'Missing marker publisher')
                manifest['visualizer_publishers'] = viewer_publishers
                graph_checked = True
            if now >= next_progress:
                require_recording_space(output)
                emit({'elapsed_sec': round(now - ready_at, 2), 'dds_points': len(audit.points),
                      'input_frames': audit.image_count - ready_image_count, 'model_state': model._state,
                      'rviz_running': rviz.poll() is None})
                next_progress += 10.0
        ended_at = time.monotonic()
        model._timer.cancel()
        drain_until = time.monotonic() + .5
        while time.monotonic() < drain_until:
            executor.spin_once(timeout_sec=.02)
            check_health()
        rows = [json.loads(line) for line in (model.run_dir / 'inference.jsonl').read_text().splitlines() if line]
        core.match_points_to_rows(audit.points, rows)
        core.require(len(audit.points) == model.published_count == len(audit.actions) == len(audit.poses),
                     'Live DDS point/action/pose counts do not match model publications')
        core.require([pose['stamp_ns'] for pose in audit.poses] == [point['stamp_ns'] for point in audit.points],
                     'Live pose timestamps do not match waypoint timestamps')
        core.require(graph_checked, 'Publisher graph acceptance did not complete')
        manifest['checks'].extend(['pc_publisher_allowlists_no_twist', 'continuous_preview_output',
                                    'actual_dds_outputs_match_saved_inference', 'rviz_alive_through_bounded_run'])
        manifest['status'] = 'passed'
        result = 0
    except StopRequested as error:
        manifest.update(status='stopped', stop_reason=str(error))
        emit(f'Stopping live pipeline: {error}')
        result = 0
    except Exception as error:
        manifest.update(status='failed', error=f'{type(error).__name__}: {error}')
        emit(manifest['error'])
    finally:
        for number in previous_handlers:
            signal.signal(number, signal.SIG_IGN)
        cleanup_errors = []
        end = ended_at if ended_at is not None else time.monotonic()
        try:
            if ready_at is not None:
                manifest['model_ready_seconds_actual'] = end - ready_at
            # Stop model output first. RViz shutdown may require several seconds.
            if model is not None:
                manifest['model_published'] = model.published_count
                try:
                    model._timer.cancel()
                    model.destroy_node()
                except Exception as error:
                    cleanup_errors.append(f'model: {error!r}')
            for name, node in (('visualizer', visualizer), ('audit', audit)):
                if node is not None:
                    try:
                        node.destroy_node()
                    except Exception as error:
                        cleanup_errors.append(f'{name}: {error!r}')
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
            try:
                stop_rviz(rviz)
            except Exception as error:
                cleanup_errors.append(f'RViz cleanup: {error!r}')
            if rviz_log is not None:
                rviz_log.close()
            if rviz is not None:
                manifest['rviz_exit_code'] = rviz.poll()
            if audit is not None:
                manifest['counts'] = {'input_frames_total': audit.image_count,
                                      'input_frames_after_ready': audit.image_count - ready_image_count if ready_at else 0,
                                      'dds_points': len(audit.points), 'dds_actions': len(audit.actions),
                                      'dds_poses': len(audit.poses)}
                manifest['input_metric_scope'] = 'audit callback timing; not sensor hardware FPS'
                manifest['input_age_sec'] = core.distribution(audit.image_ages)
                manifest['point_age_sec'] = core.distribution([point['age_sec'] for point in audit.points])
                manifest['point_gap_sec'] = core.distribution([b['arrival_monotonic'] - a['arrival_monotonic']
                                                              for a, b in zip(audit.points, audit.points[1:])])
                before_end = [point for point in audit.points if point['arrival_monotonic'] <= end]
                manifest['last_point_age_at_end_sec'] = end - before_end[-1]['arrival_monotonic'] if before_end else None
                manifest['input_header_evidence'] = {'first': audit.first_headers, 'last': list(audit.headers)}
                (output / 'dds_points.json').write_text(json.dumps(audit.points, indent=2) + '\n', encoding='utf-8')
            if model is not None and model.run_dir is not None:
                rows = [json.loads(line) for line in (model.run_dir / 'inference.jsonl').read_text().splitlines() if line]
                manifest['inference_ms'] = core.distribution([row['inference_ms'] for row in rows])
                manifest['model_final_status'] = json.loads((model.run_dir / 'manifest.json').read_text())['status']
            if cleanup_errors:
                manifest.update(status='failed', cleanup_errors=cleanup_errors)
                result = 1
            manifest['finished_utc'] = datetime.now(timezone.utc).isoformat()
            write_manifest()
            emit({'status': manifest['status'], 'manifest': str(output / 'manifest.json')})
        finally:
            log.close()
            for number, handler in previous_handlers.items():
                signal.signal(number, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
