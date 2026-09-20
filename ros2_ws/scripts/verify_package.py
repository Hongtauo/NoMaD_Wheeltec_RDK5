#!/usr/bin/env python3
"""Verify the installed package with isolated log-only run/launch smoke tests."""

import json
import os
from pathlib import Path
import signal
import subprocess
import time
from datetime import datetime, timezone

import rclpy
from ament_index_python.packages import get_package_share_directory


def main():
    workspace = Path(__file__).resolve().parents[1]
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output = workspace.parent / 'outputs' / 'ros2_package_checks' / ('nomad_navigation__' + stamp)
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'output': str(output), 'checks': []}
    probe = None

    def command(args):
        result = subprocess.run(args, capture_output=True, text=True, timeout=25)
        if result.returncode:
            raise RuntimeError(f'{args}: {result.stdout}\n{result.stderr}')
        return result.stdout.strip()

    def wait_until(predicate, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
            if predicate():
                return
        raise TimeoutError('ROS graph or heartbeat did not reach the expected state')

    def exists(target):
        return any(
            (namespace.rstrip('/') + '/' + name) == target
            for name, namespace in probe.get_node_names_and_namespaces())

    def exercise(label, args, target, expected_period):
        log_path = output / (label + '.log')
        if exists(target):
            raise RuntimeError(f'{target} already exists in the test domain; refusing duplicate test')
        with log_path.open('w', encoding='utf-8') as log:
            process = subprocess.Popen(
                args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                env={**os.environ, 'ROS_LOG_DIR': str(output / 'ros_logs')},
            )
            try:
                wait_until(lambda: exists(target))
                wait_until(lambda: 'heartbeat=' in log_path.read_text(encoding='utf-8'))
                parameter = command(['ros2', 'param', 'get', target, 'heartbeat_period_sec'])
                assert parameter.endswith(str(expected_period)), parameter
                node_names = command(['ros2', 'node', 'list', '--no-daemon'])
                assert target in node_names.splitlines(), node_names
                name = target.rsplit('/', 1)[1]
                namespace = target.rsplit('/', 1)[0] or '/'
                publishers = probe.get_publisher_names_and_types_by_node(name, namespace)
                assert all(topic in ('/rosout', '/parameter_events') for topic, _ in publishers), publishers
            finally:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                    raise RuntimeError(f'{label} did not stop on SIGINT')
        assert process.returncode in (0, 130, -signal.SIGINT), process.returncode
        wait_until(lambda: not exists(target))
        log_text = log_path.read_text(encoding='utf-8')
        assert 'Traceback' not in log_text, log_text
        report['checks'].append({
            'mode': label, 'node': target, 'parameter': parameter,
            'returncode': process.returncode, 'clean_shutdown': True,
            'no_motion_publishers': True, 'log': log_path.name,
        })

    try:
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Set ROS_LOCALHOST_ONLY=1 to isolate this smoke test')
        if not os.environ.get('ROS_DOMAIN_ID'):
            raise RuntimeError('Set a dedicated ROS_DOMAIN_ID for this smoke test')
        report['ros_domain_id'] = os.environ['ROS_DOMAIN_ID']
        report['prefix'] = command(['ros2', 'pkg', 'prefix', 'nomad_navigation'])
        report['executables'] = command(['ros2', 'pkg', 'executables', 'nomad_navigation'])
        assert 'nomad_navigation smoke_node' in report['executables']
        share = Path(get_package_share_directory('nomad_navigation'))
        for resource in ('package.xml', 'config/smoke.yaml', 'launch/smoke.launch.py'):
            assert (share / resource).is_file(), resource
        report['installed_resources'] = True
        rclpy.init()
        probe = rclpy.create_node('nomad_package_probe_' + str(os.getpid()))
        exercise('ros2_run', [
            'ros2', 'run', 'nomad_navigation', 'smoke_node', '--ros-args',
            '-p', 'heartbeat_period_sec:=0.25',
        ], '/nomad_smoke', 0.25)
        exercise('ros2_launch', [
            'ros2', 'launch', 'nomad_navigation', 'smoke.launch.py', 'namespace:=package_check',
        ], '/package_check/nomad_smoke', 2.0)
        report['status'] = 'complete'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        raise
    finally:
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
