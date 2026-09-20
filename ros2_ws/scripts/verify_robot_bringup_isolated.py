#!/usr/bin/env python3
"""Real ROS launch/zero-check regression; every hardware command is replaced.

Always localhost-only domain 86, including child shim entry points. No arm,
clock change, camera/base driver, SSH, GPU, or production bypass is introduced.
Run inside the sourced ROS workspace. Results are never hardware acceptance.
"""

import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


SCRIPT = Path(__file__).resolve()
PROJECT = SCRIPT.parents[2]


def isolate_environment():
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'ROS_DISCOVERY_SERVER', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
                'FASTDDS_ENVIRONMENT_FILE', 'ROS_STATIC_PEERS',
                'NOMAD_SESSION_TOKEN_PATH', 'NOMAD_OPERATOR_SESSION_ID'):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID='86', ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', SKIP_DEFAULT_XML='1')


def checker_shim():
    # The only mocked production function is machine/token authorization.
    # Actual subscriptions, graph identities, zero checks and timing run intact.
    from nomad_navigation import robot_session_guard, robot_startup_check
    robot_session_guard.require_robot_session = lambda: {'session_id': 'isolated-test-only'}
    return robot_startup_check.main(['--max-linear-mps', '0.2', '--timeout-sec', '20'])


def hardware_stub(seconds):
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    end = time.monotonic() + seconds
    while not stopped and time.monotonic() < end:
        time.sleep(.05)
    return 0


def process_identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (FileNotFoundError, ProcessLookupError):
        return None


def cleanup_children(identities):
    def living():
        return {pid: birth for pid, birth in identities.items()
                if birth is not None and process_identity(pid) == birth}

    initially_alive = sorted(living())
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for pid in living():
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        end = time.monotonic() + 2.
        while living() and time.monotonic() < end:
            time.sleep(.05)
        if not living():
            break
    return initially_alive, sorted(living())


def run_case(case, destination):
    # No ROS/launch import occurs until after environment isolation in main.
    from launch import LaunchDescription, LaunchService
    from launch.actions import (EmitEvent, ExecuteProcess, OpaqueFunction,
                                RegisterEventHandler, TimerAction)
    from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
    from launch.events import Shutdown
    from nomad_navigation import robot_session_guard

    source = PROJECT / 'ros2_ws/src/nomad_navigation/launch/robot_bringup.launch.py'
    spec = importlib.util.spec_from_file_location('_isolated_robot_bringup', source)
    production = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(production)
    # Hardware is not installed/needed in the test container. Resolved paths
    # are test-only sentinels and are replaced below before any execution.
    production._native_executable = lambda package, executable: f'/test/lib/{package}/{executable}'
    expected = {
        'nomad_controller': ['python3', '-m', 'nomad_navigation.robot_control_node',
                             '--ros-args', '-p', 'max_linear_mps:=0.2',
                             '-p', 'max_measured_linear_mps:=0.3'],
        'nomad_gateway': ['python3', '-m', 'nomad_navigation.robot_gateway',
                         '--max-linear-mps', '0.2'],
        'nomad_startup_check': ['python3', '-m', 'nomad_navigation.robot_startup_check',
                               '--max-linear-mps', '0.2', '--timeout-sec', '20'],
        'wheeltec_camera': ['ros2', 'launch', 'turn_on_wheeltec_robot', 'wheeltec_camera.launch.py'],
        'wheeltec_base': ['/test/lib/turn_on_wheeltec_robot/wheeltec_robot_node',
                          '--ros-args', '-p', 'car_mode:=V550_akm'],
        'base_link_transform': ['/test/lib/tf2_ros/static_transform_publisher',
                                '--x', '0', '--y', '0', '--z', '0.01826',
                                '--roll', '0', '--pitch', '0', '--yaw', '0',
                                '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
    }
    hardware = {'wheeltec_camera', 'wheeltec_base', 'base_link_transform'}
    events, identities, action_names, substitutions = [], {}, {}, {}
    started = time.monotonic()

    def factory(*args, **kwargs):
        if args or kwargs.get('name') not in expected:
            raise RuntimeError('Refusing an unknown production process in isolated test')
        name = kwargs['name']
        if kwargs.get('cmd') != expected[name] or name in substitutions:
            raise RuntimeError(f'Refusing changed/duplicate command: {name}')
        if name in hardware:
            duration = '1.0' if case == 'required_exit' and name == 'wheeltec_base' else '30.0'
            command = [sys.executable, str(SCRIPT), '--hardware-stub', duration]
        elif name == 'nomad_startup_check':
            command = [sys.executable, str(SCRIPT), '--checker-shim']
        else:
            command = [sys.executable, *kwargs['cmd'][1:]]
        substitutions[name] = {'original': kwargs['cmd'], 'executed': command,
                               'hardware_stub': name in hardware}
        action = ExecuteProcess(**dict(kwargs, cmd=command))
        action_names[id(action)] = name
        return action

    def on_start(event, context):
        name = action_names.get(id(event.action), 'UNKNOWN')
        identities[event.pid] = process_identity(event.pid)
        events.append({'event': 'start', 'name': name, 'pid': event.pid,
                       'seconds': time.monotonic() - started})
        if name == 'UNKNOWN':
            return [EmitEvent(event=Shutdown(reason='Unknown process started; failing isolated test'))]
        return []

    def on_exit(event, context):
        events.append({'event': 'exit', 'name': action_names.get(id(event.action), 'UNKNOWN'),
                       'returncode': event.returncode, 'seconds': time.monotonic() - started})
        return []

    def on_shutdown(event, context):
        events.append({'event': 'shutdown', 'reason': str(event.reason),
                       'seconds': time.monotonic() - started})
        return []

    def deadline(context):
        events.append({'event': 'timer', 'seconds': time.monotonic() - started})
        return [EmitEvent(event=Shutdown(reason='Isolated acceptance timer elapsed'))]

    production.ExecuteProcess = factory
    original_guard = robot_session_guard.require_robot_session
    robot_session_guard.require_robot_session = lambda: {'session_id': 'isolated-test-only'}
    report = {'case': case, 'hardware_test': False, 'stubbed_hardware': True,
              'actual_ros_checker': True, 'domain': 86, 'localhost_only': True,
              'arm_requested': False, 'events': events, 'processes': substitutions}
    service = LaunchService(argv=[])
    description = LaunchDescription([
        RegisterEventHandler(OnProcessStart(on_start=on_start)),
        RegisterEventHandler(OnProcessExit(on_exit=on_exit)),
        RegisterEventHandler(OnShutdown(on_shutdown=on_shutdown)),
        TimerAction(period=5.0 if case == 'timed_shutdown' else 12.0,
                    actions=[OpaqueFunction(function=deadline)]),
        *production.generate_launch_description().entities,
    ])
    try:
        service.include_launch_description(description)
        report['launch_returncode'] = service.run()
        assert set(substitutions) == set(expected), 'The production command set changed'
        checker_exits = [event for event in events if event['event'] == 'exit'
                         and event['name'] == 'nomad_startup_check']
        assert len(checker_exits) == 1 and checker_exits[0]['returncode'] == 0, \
            'Actual ROS startup checker did not succeed'
        hardware_starts = [event for event in events if event['event'] == 'start'
                           and event['name'] in hardware]
        assert {event['name'] for event in hardware_starts} == hardware, 'Missing hardware stub starts'
        assert all(event['seconds'] >= checker_exits[0]['seconds'] for event in hardware_starts), \
            'A hardware stub started before the checker succeeded'
        starts = [event['name'] for event in events if event['event'] == 'start']
        exits = [event['name'] for event in events if event['event'] == 'exit']
        assert sorted(starts) == sorted(exits), 'Started processes did not all emit exit events'
        if case == 'required_exit':
            assert not any(event['event'] == 'timer' for event in events), \
                'Required process exit failed to shut down the session before the fallback timer'
            base_exit = next(event for event in events if event['event'] == 'exit'
                             and event['name'] == 'wheeltec_base')
            assert base_exit['returncode'] == 0, 'Fault-injection stub did not exit as planned'
            assert any(event['event'] == 'shutdown' and 'Required robot process exited' in event['reason']
                       for event in events), 'Required-process shutdown handler was not observed'
        else:
            assert any(event['event'] == 'timer' for event in events), 'Timed shutdown was not observed'
        report['passed'] = True
    except BaseException as error:
        report.update(passed=False, error=f'{type(error).__name__}: {error}')
    finally:
        robot_session_guard.require_robot_session = original_guard
        initially_alive, remaining = cleanup_children(identities)
        report['children_alive_after_launch'] = initially_alive
        report['children_remaining_after_cleanup'] = remaining
        if initially_alive or remaining:
            report.update(passed=False, cleanup_error='Launch did not cleanly reap all started children')
        report['elapsed_sec'] = time.monotonic() - started
        destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return 0 if report.get('passed') else 1


def main(argv=None):
    isolate_environment()
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output-root', type=Path,
                        default=PROJECT / 'outputs/ros2_robot_bringup_tests')
    parser.add_argument('--case', choices=('timed_shutdown', 'required_exit'))
    parser.add_argument('--case-report', type=Path)
    parser.add_argument('--checker-shim', action='store_true')
    parser.add_argument('--hardware-stub', type=float)
    args = parser.parse_args(argv)
    if args.checker_shim:
        return checker_shim()
    if args.hardware_stub is not None:
        if args.hardware_stub not in (1., 30.):
            parser.error('Only the bounded acceptance stub durations are supported')
        return hardware_stub(args.hardware_stub)
    if args.case:
        if args.case_report is None:
            parser.error('--case requires --case-report')
        return run_case(args.case, args.case_report)
    output = args.output_root / ('domain86__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                 + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    summary = {'hardware_test': False, 'stubbed_hardware': True, 'actual_ros_checker': True,
               'domain': 86, 'localhost_only': True, 'arm_requested': False, 'cases': []}
    for case in ('timed_shutdown', 'required_exit'):
        destination = output / (case + '.json')
        with (output / (case + '.log')).open('w', encoding='utf-8') as log:
            result = subprocess.run([sys.executable, str(SCRIPT), '--case', case,
                                     '--case-report', str(destination)],
                                    stdout=log, stderr=subprocess.STDOUT, timeout=45.)
        evidence = json.loads(destination.read_text()) if destination.exists() else {
            'passed': False, 'error': 'Worker exited without a report'}
        summary['cases'].append({'case': case, 'returncode': result.returncode,
                                 'report': str(destination), 'passed': evidence.get('passed', False)})
        if result.returncode or not evidence.get('passed'):
            break  # Do not start another case while cleanup is uncertain.
    summary['passed'] = len(summary['cases']) == 2 and all(case['passed'] for case in summary['cases'])
    (output / 'report.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(summary, output=str(output)), indent=2), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
