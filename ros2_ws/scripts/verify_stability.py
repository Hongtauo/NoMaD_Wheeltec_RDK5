#!/usr/bin/env python3
"""Bounded real-CUDA preview soak; finite replays separated by verified stale/zero gaps.

No robot output. This verifies the ROS pipeline, not RViz GUI rendering or robot safety.
"""
import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import psutil
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration-sec', type=float, default=1800)
    args = parser.parse_args()
    if not math.isfinite(args.duration_sec) or args.duration_sec < 60:
        raise ValueError('Duration must be at least 60 seconds')
    root = Path(__file__).resolve().parents[2]
    # Keep diagnostics and tested nodes on an isolated local UDP-only domain.
    os.environ.update(ROS_DOMAIN_ID='183', ROS_LOCALHOST_ONLY='0',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(root/'docker/ros2/rviz_udp.xml'))
    out = root/'outputs/stability_checks'/('nomad-preview__'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir(parents=True, exist_ok=False)
    report = dict(status='starting', output=str(out), requested_sec=args.duration_sec,
                  domain=183, scope='real CUDA ROS preview; finite replay with stale gaps; no GUI or robot',
                  runner_pid=os.getpid(), checks=[], rounds=1, completed_replays=0)
    def save():
        temporary = out/'report.tmp'
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        temporary.replace(out/'report.json')
    save()
    print(str(out), flush=True)
    rclpy.init()
    node = rclpy.create_node('nomad_stability_probe')
    counts, last, intervals = Counter(), {}, {}
    states = {'model': 'unknown', 'controller': 'unknown'}
    errors, processes, handles = [], [], []
    velocities = deque(maxlen=10)
    active_paths = False
    prev_ros = None
    rollback_count = 0
    stage = 'startup'
    epoch = time.monotonic()
    replay_started = epoch
    log = (out/'telemetry.jsonl').open('w')
    events = (out/'events.jsonl').open('w')
    def event(kind, **fields):
        events.write(json.dumps(dict(elapsed_sec=time.monotonic()-epoch, kind=kind, **fields))+'\n')
        events.flush()
    def received(topic):
        now = time.monotonic()
        if topic in last:
            intervals.setdefault(topic, deque(maxlen=200)).append(now-last[topic])
        last[topic] = now
        counts[topic] += 1
    def state(which, msg):
        value = json.loads(msg.data)
        states[which] = value['state']
        event(which, state=value['state'])
        if which == 'model' and value.get('run_dir'):
            report['model_run'] = value['run_dir']
        if value['state'] == 'faulted' or 'clock' in value['state']:
            errors.append(f'{which}: {value["state"]}')
    def waypoint(msg):
        received('waypoint')
        if not all(math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z)):
            errors.append('Nonfinite waypoint')
    def velocity(msg):
        received('preview')
        v, w = msg.linear.x, msg.angular.z
        velocities.append((time.monotonic(), v, w))
        if not (math.isfinite(v) and math.isfinite(w) and -1e-6 <= v <= .200001 and abs(w) <= .400001):
            errors.append(f'Invalid preview limits: {v}, {w}')
    def marker(msg):
        nonlocal active_paths
        received('markers')
        active_paths = any(m.action == Marker.ADD and m.ns in
            ('candidate_paths', 'selected_path', 'selected_waypoint', 'velocity', 'angular_velocity') for m in msg.markers)
    node.create_subscription(Image, '/camera/image_raw', lambda m: received('image'), qos_profile_sensor_data)
    node.create_subscription(PointStamped, '/waypoint_stamped', waypoint, 10)
    node.create_subscription(Twist, '/cmd_vel_preview', velocity, 10)
    node.create_subscription(MarkerArray, '/nomad/visualization', marker, 10)
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/nomad/status', lambda m: state('model', m), qos)
    node.create_subscription(String, '/controller/status', lambda m: state('controller', m), qos)
    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
    def start(command, name):
        handle = (out/name).open('w')
        handles.append(handle)
        p = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True,
            env={**os.environ, 'PYTHONUNBUFFERED': '1', 'ROS_LOG_DIR': str(out/'ros_logs')})
        processes.append(p)
        return p
    stack = replay = None
    samples = []
    trackers = {}
    try:
        spin(2)
        for topic in ('/camera/image_raw', '/waypoint_stamped', '/cmd_vel_preview', '/nomad/visualization', '/cmd_vel'):
            if node.count_publishers(topic):
                raise RuntimeError('Test domain busy: '+topic)
        if psutil.disk_usage(root).free < 5*1024**3:
            raise RuntimeError('Need at least 5 GiB free for per-inference archives')
        stack = start(['ros2', 'launch', 'nomad_navigation', 'rviz_demo.launch.py',
            'project_root:='+str(root), 'use_rviz:=false'], 'combined_launch.log')
        report['launch_pid'] = stack.pid
        deadline = time.monotonic()+90
        while counts['waypoint'] < 8:
            spin(.1)
            if errors or stack.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Startup failed: '+str(errors))
        epoch = time.monotonic()
        replay_started = epoch
        report.update(status='running', started_utc=datetime.now(timezone.utc).isoformat())
        stage, last_sample, gap_since = 'playing', epoch-10, None
        base_counts = counts.copy()
        save()
        event('measurement_start')
        while time.monotonic()-epoch < args.duration_sec:
            spin(.1)
            now = time.monotonic()
            ros = node.get_clock().now().nanoseconds
            if prev_ros is not None and ros < prev_ros:
                rollback_count += 1
                errors.append('ROS clock rollback')
            prev_ros = ros
            if stack.poll() is not None:
                errors.append('Combined launch exited unexpectedly')
            if replay is not None and replay.poll() not in (None, 0):
                errors.append('Video replay failed')
            if errors:
                raise RuntimeError('; '.join(errors))
            if now-last.get('preview', epoch) > 2 or now-last.get('markers', epoch) > 2:
                raise RuntimeError('Preview/marker heartbeat stalled for >2 seconds')
            if stage == 'playing':
                if now-last.get('image', epoch) < .75 and now-replay_started > 5 and now-last.get('waypoint', epoch) > 3:
                    raise RuntimeError('Inference stalled while camera is active')
                if (last.get('image', 0) >= replay_started and states['model'] == 'stale_input'
                        and now-last.get('image', epoch) > 1):
                    if not velocities or any(v or w for _, v, w in velocities) or active_paths:
                        if now-last['image'] > 4:
                            raise RuntimeError('EOF did not clear predictions and zero preview')
                    elif node.count_publishers('/camera/image_raw') == 0:
                        stage, gap_since = 'gap', now
                        report['completed_replays'] += 1
                        event('eof_zero_and_clear', round=report['rounds'])
                if now-replay_started > 80:
                    raise RuntimeError('Replay did not finish within 80 seconds')
            elif now-gap_since >= 2:
                replay = start(['ros2', 'launch', 'nomad_navigation', 'video.launch.py',
                    'video_path:='+str(root/'offline_data/3-r.mp4')], f'video_{report["rounds"]+1:03d}.log')
                report['rounds'] += 1
                stage, replay_started = 'playing', now
                event('replay_start', round=report['rounds'])
            if now-last_sample >= 10:
                if node.count_publishers('/cmd_vel'):
                    raise RuntimeError('Unexpected real motion publisher')
                owners = [psutil.Process(stack.pid)]
                if replay is not None and replay.poll() is None:
                    owners.append(psutil.Process(replay.pid))
                pids = {p.pid: p for owner in owners for p in [owner]+owner.children(recursive=True)}
                rss, cpu = 0, 0.0
                for pid, proc in pids.items():
                    try:
                        tracked = trackers.setdefault(pid, proc)
                        rss += tracked.memory_info().rss
                        cpu += tracked.cpu_percent()
                    except psutil.Error:
                        pass
                trackers = {pid: proc for pid, proc in trackers.items() if pid in pids}
                gpu = subprocess.check_output(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used,memory.total',
                    '--format=csv,noheader,nounits'], text=True, timeout=5).strip()
                sample = dict(elapsed_sec=now-epoch, stage=stage, states=dict(states), counts=dict(counts),
                    process_rss_mib=rss/1024**2, process_cpu_percent=cpu, gpu_csv=gpu,
                    system_memory_percent=psutil.virtual_memory().percent,
                    free_disk_gib=psutil.disk_usage(root).free/1024**3,
                    recent_hz={key: len(values)/sum(values) for key, values in intervals.items() if values and sum(values)>0})
                samples.append(sample)
                log.write(json.dumps(sample)+'\n'); log.flush()
                report.update(elapsed_sec=now-epoch, counts=dict(counts), latest=sample, clock_rollbacks=rollback_count)
                save()
                print(f'{now-epoch:.0f}/{args.duration_sec:.0f}s rounds={report["rounds"]} waypoint={counts["waypoint"]} rss={rss/1024**2:.0f}MiB', flush=True)
                last_sample = now
        report.update(elapsed_sec=time.monotonic()-epoch, counts=dict(counts),
            measured_counts=dict(counts-base_counts), clock_rollbacks=rollback_count,
            status='validating', max_process_rss_mib=max(s['process_rss_mib'] for s in samples))
        if counts['waypoint']-base_counts['waypoint'] < args.duration_sec*2:
            raise RuntimeError('Average prediction throughput below 2 Hz including replay gaps')
        report['checks'] = ['requested_monotonic_duration_reached', 'core_launch_alive',
            'real_predictions_continued', 'preview_limits_finite', 'marker_preview_heartbeats',
            'no_clock_rollback', 'no_real_motion_publisher', 'replay_eof_zero_and_clear']
    except BaseException as error:
        report.update(status='failed', error=repr(error), elapsed_sec=time.monotonic()-epoch)
    finally:
        cleanup = []
        for p in reversed(processes):
            if p.poll() is None:
                # Signal launch only: launch propagates to children without double SIGINT.
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(timeout=5)
            cleanup.append({'pid': p.pid, 'returncode': p.returncode})
        spin(1)
        remaining = {t: node.count_publishers(t) for t in
            ('/camera/image_raw', '/waypoint_stamped', '/cmd_vel_preview', '/nomad/visualization')}
        report['cleanup'] = cleanup
        report['remaining_publishers'] = remaining
        if report['status'] == 'validating':
            if any(remaining.values()) or any(p['returncode'] != 0 for p in cleanup):
                report.update(status='failed', error='Unclean owned process shutdown')
            else:
                report['checks'].append('owned_processes_clean_shutdown')
                report['status'] = 'complete'
        report['finished_utc'] = datetime.now(timezone.utc).isoformat()
        save()
        for handle in handles:
            handle.close()
        log.close(); events.close()
        node.destroy_node(); rclpy.shutdown()
        print(json.dumps(report, indent=2), flush=True)
    return 0 if report['status'] == 'complete' else 1


if __name__ == '__main__':
    raise SystemExit(main())
