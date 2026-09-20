#!/usr/bin/env python3
"""Actual GPU + ROS video integration, including interruption/recovery checks."""

from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
import yaml


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            raise RuntimeError('Owned test process failed to stop normally')


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Use a dedicated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    root = Path(__file__).resolve().parents[2]
    output = root / 'outputs' / 'ros2_nomad_checks' / (
        '3-r__image-to-waypoint__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'output': str(output), 'checks': []}
    print(f'Integration report: {output}', flush=True)
    processes, handles = [], []
    waypoints, candidates, stamped, statuses, images = [], [], [], [], []
    rclpy.init()
    probe = rclpy.create_node('nomad_pipeline_probe_' + str(os.getpid()))
    probe.create_subscription(Float32MultiArray, '/waypoint',
                              lambda msg: waypoints.append(list(msg.data)), 100)
    probe.create_subscription(Float32MultiArray, '/sampled_actions',
                              lambda msg: candidates.append(list(msg.data)), 100)
    probe.create_subscription(PointStamped, '/waypoint_stamped', stamped.append, 100)
    probe.create_subscription(String, '/nomad/status', lambda msg: statuses.append(json.loads(msg.data)),
                              QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    probe.create_subscription(Image, '/camera/image_raw', lambda msg: images.append(
        msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec), qos_profile_sensor_data)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=0.05)

    def wait_for(predicate, seconds=30):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            spin(0.1)
            if statuses and statuses[-1]['state'] == 'faulted':
                raise RuntimeError('Real NoMaD node reported an inference fault')
            if predicate():
                return
        raise TimeoutError('Integration condition timed out')

    def launch(args, log_name):
        log = (output / log_name).open('w', encoding='utf-8')
        handles.append(log)
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                   env={**os.environ, 'ROS_LOG_DIR': str(output / 'ros_logs'),
                                        'PYTHONUNBUFFERED': '1'})
        processes.append(process)
        return process

    def start_video(label, end=12.0):
        return launch(['ros2', 'run', 'nomad_navigation', 'video_publisher', '--ros-args',
                       '-p', 'video_path:=' + str(root / 'offline_data' / '3-r.mp4'),
                       '-p', f'end_sec:={end}', '-p', 'loop:=false'], label + '.log')

    try:
        spin(0.3)
        assert probe.count_publishers('/waypoint') == 0, 'Dedicated test domain already has waypoints'
        assert probe.count_publishers('/camera/image_raw') == 0, 'Dedicated test domain already has images'
        config = {'/**': {'ros__parameters': {'run_name': '3-r-integration'}}}
        params = output / 'nomad_params.yaml'
        params.write_text(yaml.safe_dump(config), encoding='utf-8')
        model = launch(['ros2', 'launch', 'nomad_navigation', 'nomad.launch.py',
                        'project_root:=' + str(root), 'params_file:=' + str(params)], 'nomad.log')
        wait_for(lambda: bool(statuses) or model.poll() is not None, 90)
        assert model.poll() is None and statuses[-1]['state'] == 'waiting_for_images'
        run_dir = Path(statuses[-1]['run_dir'])
        report['node_run'] = str(run_dir)
        spin(1.0)
        assert not waypoints, 'Published waypoint before receiving images'
        print('GPU model ready; checking video -> waypoint.', flush=True)
        video = start_video('video_before_interruption')
        wait_for(lambda: len(waypoints) >= 16 or video.poll() is not None, 25)
        assert len(waypoints) >= 16, 'Too few outputs from real model'
        names = probe.get_publisher_names_and_types_by_node('nomad_inference', '/')
        allowed = {'/waypoint', '/sampled_actions', '/waypoint_stamped', '/nomad/status',
                   '/rosout', '/parameter_events'}
        assert all(name in allowed for name, _ in names), names
        assert not any('geometry_msgs/msg/Twist' in types for _, types in names)
        stop(video)
        assert video.returncode == 0
        wait_for(lambda: statuses[-1]['state'] == 'stale_input', 5)
        count = len(waypoints)
        spin(1.3)
        assert len(waypoints) == count, 'Outputs continued after camera timeout'
        report['checks'].append('camera_interruption_stops_waypoints')
        print('Interruption check passed; injecting stale and malformed images.', flush=True)
        publisher = probe.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        wait_for(lambda: publisher.get_subscription_count() >= 1)
        bridge = CvBridge()
        for _ in range(4):
            msg = bridge.cv2_to_imgmsg(np.zeros((180, 320, 3), dtype=np.uint8), encoding='bgr8')
            old = probe.get_clock().now().nanoseconds - 3_000_000_000
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(old, 10**9)
            msg.header.frame_id = 'camera_optical_frame'
            publisher.publish(msg)
            spin(0.26)
        publisher.publish(Image(width=10, height=10, step=30, encoding='bgr8', data=[1]))
        spin(0.5)
        assert len(waypoints) == count, 'Rejected images produced waypoints'
        probe.destroy_publisher(publisher)
        report['checks'].append('stale_and_malformed_images_rejected')
        resume_time = probe.get_clock().now().nanoseconds
        video = start_video('video_after_interruption', 6.0)
        wait_for(lambda: len(waypoints) >= count + 8 or video.poll() is not None, 20)
        assert len(waypoints) >= count + 8
        stop(video)
        wait_for(lambda: statuses[-1]['state'] == 'stale_input', 5)
        spin(0.5)
        report['checks'].append('fresh_context_rebuilt_and_inference_resumed')
        stop(model)
        assert model.returncode == 0
        spin(0.5)
        assert probe.count_publishers('/waypoint') == 0
        rows = [json.loads(line) for line in (run_dir / 'inference.jsonl').read_text().splitlines()]
        manifest = json.loads((run_dir / 'manifest.json').read_text())
        assert manifest['runtime']['device'].startswith('cuda')
        assert len(rows) == len(waypoints) == len(candidates) == len(stamped)
        assert manifest['published_count'] == len(rows) and manifest['status'] == 'stopped'
        for i, row in enumerate(rows):
            with np.load(run_dir / row['archive']) as data:
                actions = data['trajectories']
                assert actions.shape == (8, 8, 2) and np.isfinite(actions).all()
                np.testing.assert_allclose(actions[0, 2] * 0.05, waypoints[i], atol=1e-7)
                np.testing.assert_allclose(candidates[i], np.concatenate(([0.0], actions.flatten())), atol=1e-7)
                assert len(candidates[i]) == 129
                np.testing.assert_allclose([stamped[i].point.x, stamped[i].point.y], waypoints[i], atol=1e-7)
                assert stamped[i].header.frame_id == 'base_link'
                assert stamped[i].header.stamp.sec * 10**9 + stamped[i].header.stamp.nanosec == row['observation_stamp_ns']
                assert 0 <= row['published_ns'] - row['observation_stamp_ns'] < 1_000_000_000
                if i >= count:
                    assert data['observation_stamps_ns'].min() > resume_time
        report.update(waypoint_count=len(rows), runtime=manifest['runtime'],
                      mean_inference_ms=statistics.mean(row['inference_ms'] for row in rows),
                      median_inference_ms=statistics.median(row['inference_ms'] for row in rows))
        report['checks'] += ['actual_cuda_inference', 'waypoint_and_candidate_payloads_match_archives',
                             'no_twist_publishers', 'clean_shutdown']

        # Compare preprocessing and action decoding against repository code,
        # without importing any ROS 1 exploration node.
        import torch
        from PIL import Image as PILImage
        from nomad_navigation.nomad_core import NoMaDBackend, preprocess_rgb
        from vint_train.training.train_utils import get_action
        path = root / 'visualnav-transformer' / 'deployment' / 'src' / 'utils.py'
        spec = importlib.util.spec_from_file_location('official_nomad_utils', path)
        reference = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(reference)
        with np.load(run_dir / rows[0]['archive']) as data:
            frame = data['current_rgb']
        actual = preprocess_rgb([frame] * 4, [96, 96])
        expected = reference.transform_images([PILImage.fromarray(frame)] * 4, [96, 96], center_crop=False)
        assert torch.equal(actual, expected)
        decoder = NoMaDBackend.__new__(NoMaDBackend)
        stats = yaml.safe_load((run_dir / 'action_stats.yaml').read_text())['action_stats']
        decoder.minimum = np.asarray(stats['min'], dtype=np.float64)
        decoder.maximum = np.asarray(stats['max'], dtype=np.float64)
        noise = torch.randn((8, 8, 2), generator=torch.Generator().manual_seed(7))
        np.testing.assert_array_equal(decoder.decode(noise), get_action(noise).numpy())
        report['checks'].append('official_preprocessing_and_action_decoding_exact_match')

        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        selected = np.unique(np.linspace(0, len(rows) - 1, 3, dtype=int))
        figure, axes = plt.subplots(len(selected), 2, figsize=(10, 3.5 * len(selected)), squeeze=False)
        for position, index in enumerate(selected):
            with np.load(run_dir / rows[index]['archive']) as data:
                axes[position, 0].imshow(data['current_rgb'])
                axes[position, 0].set_title(f'ROS image / output {index}')
                axes[position, 0].axis('off')
                axis = axes[position, 1]
                for action in data['trajectories'] * 0.05:
                    axis.plot(np.r_[0, action[:, 1]], np.r_[0, action[:, 0]], '.-', alpha=0.6)
                x, y = data['waypoint']
                axis.scatter([y], [x], c='red', marker='x', s=100, label='/waypoint')
                axis.set(xlabel='lateral y (nominal m)', ylabel='forward x (nominal m)')
                axis.set_aspect('equal', adjustable='datalim')
                axis.grid(True)
                axis.legend()
        figure.tight_layout()
        figure.savefig(output / 'summary.png', dpi=140)
        plt.close(figure)
        report['status'] = 'complete'
    except Exception as error:
        report.update(status='failed', error=repr(error))
        raise
    finally:
        for process in reversed(processes):
            stop(process)
        for handle in handles:
            handle.close()
        probe.destroy_node()
        rclpy.shutdown()
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
