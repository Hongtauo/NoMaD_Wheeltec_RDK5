#!/usr/bin/env python3
"""Preview safety scenarios followed by a real GPU NoMaD pipeline check."""

import csv
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import PointStamped, Twist
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.clock import Clock, ClockType
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steady-ros-clock', action='store_true',
                        help='Publish a test-only monotonic /clock; does not validate the host wall clock')
    options = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Set a dedicated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    root = Path(__file__).resolve().parents[2]
    output = root / 'outputs' / 'controller_checks' / (
        'preview-safety__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'output': str(output), 'checks': [],
              'clock_mode': 'test_monotonic_clock' if options.steady_ros_clock else 'system_ros_clock'}
    print(f'Controller check: {output}', flush=True)
    processes, files, commands, states, model_states, waypoints = [], [], [], [], [], []
    rclpy.init()
    probe = rclpy.create_node('controller_check_' + str(os.getpid()))
    clock_start = time.monotonic_ns()

    def source_now():
        return (10**9 + time.monotonic_ns() - clock_start if options.steady_ros_clock
                else probe.get_clock().now().nanoseconds)

    if options.steady_ros_clock:
        assert probe.count_publishers('/clock') == 0
        clock_pub = probe.create_publisher(ClockMessage, '/clock', 1)
        def publish_clock():
            message = ClockMessage()
            message.clock.sec, message.clock.nanosec = divmod(source_now(), 10**9)
            clock_pub.publish(message)
        clock_timer = probe.create_timer(0.01, publish_clock, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(msg):
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        assert all(math.isfinite(v) for v in values)
        assert 0 <= msg.linear.x <= 0.2 + 1e-9 and abs(msg.angular.z) <= 0.4 + 1e-9
        assert values[1:5] == [0.0] * 4
        commands.append((time.monotonic(), msg.linear.x, msg.angular.z))

    probe.create_subscription(Twist, '/cmd_vel_preview', receive, 100)
    probe.create_subscription(String, '/controller/status', lambda msg: states.append(json.loads(msg.data)),
                              QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    probe.create_subscription(String, '/nomad/status', lambda msg: model_states.append(json.loads(msg.data)),
                              QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    probe.create_subscription(PointStamped, '/waypoint_stamped', waypoints.append, 100)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.02)

    def wait(predicate, seconds=5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin(0.05)
            if predicate():
                return
        raise TimeoutError(f'Condition failed; controller state={states[-1] if states else None}')

    def launch(args, name):
        if options.steady_ros_clock and args[:2] == ['ros2', 'run']:
            args += ['-p', 'use_sim_time:=true']
        handle = (output / name).open('w', encoding='utf-8')
        files.append(handle)
        process = subprocess.Popen(args, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True,
                                   env={**os.environ, 'PYTHONUNBUFFERED': '1', 'ROS_LOG_DIR': str(output / 'ros_logs')})
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            end = time.monotonic() + 10
            while process.poll() is None and time.monotonic() < end:
                spin(0.02)  # Keep receiving the controller's final zero.
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
                raise RuntimeError('Owned process failed graceful shutdown')
        assert process.returncode == 0

    def moving():
        wait(lambda: commands and commands[-1][1] > 0 and states[-1]['state'] == 'tracking')

    def zero(reason=None):
        wait(lambda: len(commands) >= 3 and all(v == w == 0 for _, v, w in commands[-3:])
             and (reason is None or states[-1]['state'] == reason))

    try:
        spin(0.2)
        for topic in ('/cmd_vel', '/cmd_vel_preview', '/waypoint_stamped', '/camera/image_raw'):
            assert probe.count_publishers(topic) == 0, f'Test domain is not isolated: {topic}'
        controller = launch(['ros2', 'run', 'nomad_navigation', 'waypoint_controller', '--ros-args',
                             '-p', 'output_root:=' + str(root / 'outputs' / 'controller_runs')], 'controller.log')
        wait(lambda: bool(states) or controller.poll() is not None, 15)
        assert controller.poll() is None
        report['controller_run'] = states[-1]['run_dir']
        zero()
        report['checks'].append('startup_zero')
        cam_pub = probe.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        wp_pub = probe.create_publisher(PointStamped, '/waypoint_stamped', 1)
        status_pub = probe.create_publisher(String, '/nomad/status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        stop_pub = probe.create_publisher(Bool, '/nomad/emergency_stop', 1)
        goal_pub = probe.create_publisher(Bool, '/topoplan/reached_goal', 1)
        mode = {'camera': True, 'waypoint': True, 'kind': 'valid', 'last_stamp': 0}

        def emit():
            now = source_now()
            if mode['camera']:
                image = Image(width=8, height=8, step=24, encoding='bgr8', data=[0] * 192)
                image.header.frame_id = 'camera_optical_frame'
                image.header.stamp.sec, image.header.stamp.nanosec = divmod(now, 10**9)
                cam_pub.publish(image)
            if mode['waypoint']:
                point = PointStamped()
                point.header.frame_id = 'map' if mode['kind'] == 'frame' else 'base_link'
                stamp = now
                if mode['kind'] == 'old':
                    stamp -= 2_000_000_000
                if mode['kind'] == 'future':
                    stamp += 2_000_000_000
                if mode['kind'] == 'replay':
                    stamp = mode['last_stamp']
                else:
                    mode['last_stamp'] = stamp
                point.header.stamp.sec, point.header.stamp.nanosec = divmod(stamp, 10**9)
                point.point.x = math.nan if mode['kind'] == 'nan' else 0.1
                point.point.y = 0.1
                wp_pub.publish(point)

        timer = probe.create_timer(0.25, emit)
        status_pub.publish(String(data='{"state":"running"}'))
        moving()
        report['checks'].append('valid_waypoint_produces_bounded_preview')
        mode['waypoint'] = False
        zero('waypoint_timeout')
        report['checks'].append('waypoint_receipt_timeout_zero')
        mode['waypoint'] = True
        moving()
        mode['camera'] = False
        zero('camera_timeout')
        report['checks'].append('camera_timeout_with_fresh_waypoints_zero')
        mode['camera'] = True
        moving()
        for kind in ('frame', 'old', 'future', 'nan', 'replay'):
            mode['kind'] = kind
            zero()
            report['checks'].append(kind + '_waypoint_zero')
            mode['kind'] = 'valid'
            moving()
        status_pub.publish(String(data='{"state":"faulted"}'))
        zero('model_not_running')
        report['checks'].append('model_fault_state_zero')
        status_pub.publish(String(data='{"state":"running"}'))
        moving()
        client = probe.create_client(Trigger, '/nomad/reset_emergency_stop')
        wait(client.service_is_ready)
        stop_pub.publish(Bool(data=True))
        zero('emergency_stop')
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert not future.result().success
        stop_pub.publish(Bool(data=False))
        spin(0.4)
        zero('emergency_stop')
        mode['waypoint'] = False
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success
        spin(0.3)
        zero()
        mode['waypoint'] = True
        moving()
        report['checks'].append('software_estop_latch_and_explicit_reset')
        goal_pub.publish(Bool(data=True))
        zero('reached_goal')
        goal_pub.publish(Bool(data=False))
        moving()
        report['checks'].append('reached_goal_zero')
        probe.destroy_timer(timer)
        status_pub.publish(String(data='{"state":"waiting_for_images"}'))
        zero('model_not_running')
        for publisher in (cam_pub, wp_pub, status_pub, stop_pub, goal_pub):
            probe.destroy_publisher(publisher)
        probe.destroy_client(client)
        spin(0.3)
        print('Synthetic safety scenarios passed. Starting real NoMaD GPU pipeline.', flush=True)
        model_states.clear()
        model_args = ['ros2', 'launch', 'nomad_navigation', 'nomad.launch.py',
                      'project_root:=' + str(root)]
        if options.steady_ros_clock:
            import yaml
            config = yaml.safe_load((root / 'ros2_ws/src/nomad_navigation/config/nomad.yaml').read_text())
            config['/**']['ros__parameters']['use_sim_time'] = True
            clock_config = output / 'nomad_test_clock.yaml'
            clock_config.write_text(yaml.safe_dump(config), encoding='utf-8')
            model_args.append('params_file:=' + str(clock_config))
        model = launch(model_args, 'nomad.log')
        wait(lambda: bool(model_states) and 'run_dir' in model_states[-1], 90)
        assert model.poll() is None and model_states[-1]['state'] == 'waiting_for_images'
        report['model_run'] = model_states[-1]['run_dir']
        real_start = time.monotonic()
        base_waypoints = len(waypoints)
        video = launch(['ros2', 'run', 'nomad_navigation', 'video_publisher', '--ros-args',
                        '-p', 'video_path:=' + str(root / 'offline_data' / '3-r.mp4'),
                        '-p', 'end_sec:=20.0'], 'video.log')
        wait(lambda: len(waypoints) >= base_waypoints + 12, 25)
        moving()
        report['checks'].append('real_video_cuda_nomad_to_velocity_preview')
        stop(model)  # Camera continues, so waypoint silence must independently stop preview.
        zero()
        spin(0.5)
        assert all(v == w == 0 for _, v, w in commands[-5:])
        report['checks'].append('real_inference_process_exit_zero_with_camera_alive')
        real_end = time.monotonic()
        real_waypoint_count = len(waypoints) - base_waypoints
        assert video.poll() is None
        # Restore only synthetic waypoints for the final exit-while-moving check.
        status_pub = probe.create_publisher(String, '/nomad/status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        wp_pub = probe.create_publisher(PointStamped, '/waypoint_stamped', 1)
        mode.update(camera=False, waypoint=True, kind='valid')
        timer = probe.create_timer(0.25, emit)
        status_pub.publish(String(data='{"state":"running"}'))
        moving()
        stop(controller)
        spin(0.1)
        assert commands[-1][1:] == (0.0, 0.0), 'Final observed preview was not zero'
        report['checks'].append('sigint_while_moving_delivers_final_zero')
        probe.destroy_timer(timer)
        probe.destroy_publisher(status_pub)
        probe.destroy_publisher(wp_pub)
        stop(video)
        assert probe.count_publishers('/cmd_vel') == 0
        assert probe.count_publishers('/cmd_vel_preview') == 0
        report['checks'].append('no_real_cmd_vel_publisher')
        run = Path(report['controller_run'])
        rows = list(csv.DictReader((run / 'cmd_vel.csv').open()))
        assert rows[0]['reason'] == 'startup' and rows[-1]['reason'] == 'shutdown'
        assert float(rows[0]['linear_x']) == float(rows[-1]['linear_x']) == 0
        report.update(status='complete', received_preview_messages=len(commands),
                      real_waypoints_received=real_waypoint_count,
                      observed_max_linear=max(v for _, v, _ in commands),
                      observed_max_abs_angular=max(abs(w) for _, _, w in commands))
        with (output / 'received_preview.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['monotonic_sec', 'linear_x', 'angular_z'])
            writer.writerows(commands)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        real = [item for item in commands if real_start <= item[0] <= real_end]
        fig, axes = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
        for axis, index, label in ((axes[0], 1, 'linear.x (m/s)'), (axes[1], 2, 'angular.z (rad/s)')):
            axis.step([t - real_start for t, _, _ in real], [row[index] for row in real], where='post')
            axis.set_ylabel(label)
            axis.grid(True)
        axes[1].set_xlabel('seconds from real video start')
        fig.suptitle('Real NoMaD pipeline / preview only / model exit returns to zero')
        fig.tight_layout()
        fig.savefig(output / 'preview.png', dpi=140)
        plt.close(fig)
    except Exception as error:
        report.update(status='failed', error=repr(error))
        raise
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for handle in files:
            handle.close()
        probe.destroy_node()
        rclpy.shutdown()
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
