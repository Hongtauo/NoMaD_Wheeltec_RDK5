#!/usr/bin/env python3
"""Verify the real video demo's stamped prediction/MarkerArray pipeline and EOF cleanup."""
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import PointStamped, PoseArray, Twist
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Use a dedicated local ROS domain')
    root = Path(__file__).resolve().parents[2]
    output = root / 'outputs/rviz_checks' / ('video-decisions__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    print(f'RViz check: {output}', flush=True)
    report = {'status': 'running', 'output': str(output), 'checks': []}
    rclpy.init()
    node = rclpy.create_node('rviz_check_' + str(os.getpid()))
    images, poses, waypoints, markers, velocities, statuses = [], {}, {}, [], [], []
    def stamp(msg):
        return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    node.create_subscription(Image, '/camera/image_raw', lambda msg: images.append((time.monotonic(), stamp(msg))), qos_profile_sensor_data)
    node.create_subscription(PoseArray, '/nomad/trajectory_poses', lambda msg: poses.update({stamp(msg): msg}), 10)
    node.create_subscription(PointStamped, '/waypoint_stamped', lambda msg: waypoints.update({stamp(msg): msg}), 10)
    node.create_subscription(MarkerArray, '/nomad/visualization', lambda msg: markers.append((time.monotonic(), msg)), 10)
    node.create_subscription(Twist, '/cmd_vel_preview', lambda msg: velocities.append((time.monotonic(), msg.linear.x, msg.angular.z)), 10)
    node.create_subscription(String, '/nomad/status', lambda msg: statuses.append((time.monotonic(), json.loads(msg.data))),
                              QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
    def wait(predicate, timeout=15):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            spin(0.05)
            if predicate():
                return
        raise TimeoutError('RViz integration condition timed out')
    process = None
    handle = (output / 'launch.log').open('w', encoding='utf-8')
    try:
        spin(0.3)
        for topic in ('/camera/image_raw', '/waypoint_stamped', '/cmd_vel_preview', '/nomad/visualization', '/cmd_vel'):
            assert node.count_publishers(topic) == 0, f'Domain is not isolated: {topic}'
        process = subprocess.Popen(['ros2', 'launch', 'nomad_navigation', 'rviz_demo.launch.py',
            'project_root:=' + str(root), 'use_rviz:=false', 'end_sec:=30.0'],
            start_new_session=True, stdout=handle, stderr=subprocess.STDOUT,
            env={**os.environ, 'PYTHONUNBUFFERED': '1', 'ROS_LOG_DIR': str(output/'ros_logs')})
        wait(lambda: len(poses) >= 8 and any(m.ns == 'selected_waypoint' and m.action == Marker.ADD
                                           for _, message in markers for m in message.markers), 90)
        print('LIVE: real video, matching trajectories/waypoint and RViz markers are publishing.', flush=True)
        report['model_run'] = statuses[-1][1]['run_dir']
        report['checks'].append('combined_launch_real_cuda_to_markers')
        ready = next(t for t, state in statuses if state['state'] == 'waiting_for_images')
        assert images[0][0] > ready
        report['checks'].append('video_starts_after_model_ready')
        # Observe the complete 30-second clip and its stale-output transition.
        wait(lambda: statuses[-1][1]['state'] == 'stale_input', 45)
        spin(1.0)
        latest = markers[-1][1]
        assert latest.markers[0].action == Marker.DELETEALL
        assert not any(m.ns in ('candidate_paths', 'selected_path', 'selected_waypoint', 'velocity', 'angular_velocity')
                       and m.action == Marker.ADD for m in latest.markers)
        assert all(v == w == 0 for _, v, w in velocities[-5:])
        report['checks'].append('video_eof_removes_stale_predictions_and_zeros_preview')
        common = poses.keys() & waypoints.keys()
        assert len(common) >= 8
        for key in common:
            msg, waypoint = poses[key], waypoints[key]
            assert msg.header == waypoint.header and len(msg.poses) == 64
            assert msg.poses[2].position == waypoint.point
        report['checks'].append('exact_source_stamp_and_selected_position_pairing')
        model_run = Path(report['model_run'])
        rows = [json.loads(line) for line in (model_run/'inference.jsonl').read_text().splitlines()]
        checked = 0
        for row in rows:
            key = row['observation_stamp_ns']
            if key not in poses:
                continue
            actual = np.array([[p.position.x, p.position.y] for p in poses[key].poses]).reshape(8, 8, 2)
            with np.load(model_run/row['archive']) as archive:
                np.testing.assert_allclose(actual, archive['trajectories']*row['waypoint_scale'], rtol=1e-6, atol=1e-7)
            checked += 1
        assert checked >= 8
        report['checks'].append('visual_trajectory_scale_matches_real_model_archives')
        for _, message in markers:
            for marker in message.markers:
                if marker.action == Marker.ADD:
                    assert marker.header.frame_id == 'base_link'
                    assert marker.lifetime.sec > 0 or marker.lifetime.nanosec > 0
        report['checks'].append('local_frame_and_finite_marker_lifetime')
        assert node.count_publishers('/cmd_vel') == 0
        report['checks'].append('no_real_motion_publisher')
        os.killpg(process.pid, signal.SIGINT)
        wait(lambda: process.poll() is not None, 15)
        assert process.returncode == 0
        spin(0.3)
        for topic in ('/camera/image_raw', '/waypoint_stamped', '/cmd_vel_preview', '/nomad/visualization'):
            assert node.count_publishers(topic) == 0
        report['checks'].append('combined_launch_shutdown_cleans_owned_publishers')
        report.update(status='complete', image_messages=len(images), trajectory_messages=len(poses),
                      paired_predictions=len(common), archived_predictions_checked=checked,
                      marker_arrays=len(markers), preview_messages=len(velocities))
        with (output/'preview.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['monotonic_sec', 'linear_x', 'angular_z'])
            writer.writerows(velocities)
    except Exception as error:
        report.update(status='failed', error=repr(error))
        raise
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        handle.close()
        node.destroy_node()
        rclpy.shutdown()
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
