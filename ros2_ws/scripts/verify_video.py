#!/usr/bin/env python3
"""Receive real video Image messages and save independent content/rate checks."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
import yaml


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            raise RuntimeError('Video test process did not stop cleanly')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('video_path', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Set ROS_LOCALHOST_ONLY=1 and a dedicated ROS_DOMAIN_ID')
    workspace = Path(__file__).resolve().parents[1]
    output = workspace.parent / 'outputs' / 'ros2_video_checks' / (
        '3-r__video-input__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'video': str(args.video_path.resolve()),
              'output': str(output), 'checks': []}
    rclpy.init()
    probe = rclpy.create_node('video_check_probe_' + str(os.getpid()))
    bridge = CvBridge()

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)

    def exercise(label, start_sec, end_sec, loop):
        topic = '/replay/image_raw' if loop else '/camera/image_raw'
        received = []
        references = {}
        # Reference images are decoded independently from the original video.
        cap = cv2.VideoCapture(str(args.video_path))
        fps = cap.get(cv2.CAP_PROP_FPS)
        i = 0
        while start_sec + i / 4.0 < end_sec:
            index = round(start_sec * fps) + int(i * fps / 4.0)
            assert cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, frame = cap.read()
            assert ok
            frame = cv2.resize(frame, (320, 180), interpolation=cv2.INTER_AREA)
            references[hashlib.sha256(frame.tobytes()).hexdigest()] = index
            i += 1
        cap.release()

        def on_image(msg):
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            assert (msg.width, msg.height, msg.encoding) == (320, 180, 'bgr8')
            assert msg.header.frame_id == 'camera_optical_frame'
            assert len(msg.data) == msg.height * msg.step
            digest = hashlib.sha256(frame.tobytes()).hexdigest()
            assert digest in references, 'Received pixels differ from sampled source video'
            if not received:
                assert cv2.imwrite(str(output / (label + '_received.png')), frame)
            received.append({
                'received_monotonic': time.monotonic(),
                'stamp_ns': msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                'source_frame': references[digest], 'sha256': digest,
            })

        subscription = probe.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        params = {'/**': {'ros__parameters': {
            'publish_fps': 4.0, 'start_sec': start_sec, 'end_sec': end_sec,
            'loop': loop, 'resize_width': 320, 'image_topic': topic,
            'frame_id': 'camera_optical_frame',
        }}}
        config = output / (label + '.yaml')
        config.write_text(yaml.safe_dump(params), encoding='utf-8')
        if loop:
            command = ['ros2', 'run', 'nomad_navigation', 'video_publisher', '--ros-args',
                       '--params-file', str(config), '-p', 'video_path:=' + str(args.video_path)]
        else:
            command = ['ros2', 'launch', 'nomad_navigation', 'video.launch.py',
                       'video_path:=' + str(args.video_path), 'params_file:=' + str(config),
                       'namespace:=video_check']
        processes = []
        handles = []
        environment = {**os.environ, 'ROS_LOG_DIR': str(output / 'ros_logs'), 'PYTHONUNBUFFERED': '1'}

        def launch(command, name):
            handle = (output / name).open('w', encoding='utf-8')
            handles.append(handle)
            process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=environment)
            processes.append(process)
            return process

        try:
            assert probe.count_publishers(topic) == 0, 'Test domain already has an image publisher'
            process = launch(command, label + '.log')
            deadline = time.monotonic() + 25
            while not received and time.monotonic() < deadline:
                spin_for(0.1)
                if process.poll() is not None:
                    raise RuntimeError('Video process exited before the first image')
            assert received, 'No Image messages received'
            endpoints = probe.get_publishers_info_by_topic(topic)
            assert len(endpoints) == 1
            qos = endpoints[0].qos_profile
            assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
            assert qos.durability == DurabilityPolicy.VOLATILE
            # DDS discovery does not expose queue depth consistently across
            # RMW implementations. The publisher's local depth is unit-tested.
            publishers = probe.get_publisher_names_and_types_by_node(
                endpoints[0].node_name, endpoints[0].node_namespace)
            assert all(name in (topic, '/rosout', '/parameter_events') for name, _ in publishers)
            if not loop:
                launch(['ros2', 'topic', 'hz', topic, '--window', '10'], 'topic_hz.log')
                echo = launch(['ros2', 'topic', 'echo', topic, 'sensor_msgs/msg/Image',
                               '--field', 'header', '--once', '--qos-reliability', 'best_effort',
                               '--no-daemon'], 'topic_header.log')
            while time.monotonic() < deadline:
                spin_for(0.1)
                if (loop and len(received) >= 12) or (not loop and process.poll() is not None):
                    break
            if loop:
                assert len(received) >= 12
                stop(process)
            else:
                assert process.poll() == 0, 'Finite playback did not terminate normally'
                assert echo.poll() == 0, 'Header echo did not complete'
                assert len(received) >= 18, f'Only received {len(received)} of 24 published frames'
            assert process.returncode == 0
            spin_for(0.4)
            count_after_stop = len(received)
            spin_for(0.6)
            assert len(received) == count_after_stop, 'Images continued after process exit'
            assert probe.count_publishers(topic) == 0, 'Publisher remains after exit'
            intervals = [b['received_monotonic'] - a['received_monotonic']
                         for a, b in zip(received, received[1:])]
            rate = 1.0 / statistics.median(intervals)
            assert 3.4 <= rate <= 4.6, rate
            assert all(b['stamp_ns'] > a['stamp_ns'] > 0 for a, b in zip(received, received[1:]))
            wraps = sum(b['source_frame'] < a['source_frame'] for a, b in zip(received, received[1:]))
            assert wraps >= 2 if loop else wraps == 0
            report['checks'].append({
                'case': label, 'topic': topic, 'received': len(received),
                'median_receive_hz': rate, 'loop_wraps': wraps,
                'pixels_match_source': True, 'qos_sensor_data': True,
                'discovered_depth': qos.depth,
                'no_motion_publishers': True, 'clean_stop': True,
            })
        finally:
            for child in reversed(processes):
                stop(child)
            for handle in handles:
                handle.close()
            probe.destroy_subscription(subscription)
            (output / (label + '_messages.json')).write_text(json.dumps(received, indent=2), encoding='utf-8')
        text = (output / (label + '.log')).read_text(encoding='utf-8')
        assert 'Traceback' not in text and '[ERROR]' not in text, text
        if not loop:
            assert 'Playback complete; published=24' in text, text
            assert 'average rate:' in (output / 'topic_hz.log').read_text(encoding='utf-8')
            assert 'camera_optical_frame' in (output / 'topic_header.log').read_text(encoding='utf-8')

    try:
        exercise('finite_launch', 2.0, 8.0, False)
        exercise('loop_run', 0.0, 1.0, True)
        report['status'] = 'complete'
    except Exception as error:
        report.update(status='failed', error=repr(error))
        raise
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
