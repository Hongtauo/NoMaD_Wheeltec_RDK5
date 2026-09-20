"""Start independent robot processes, still DISARMED, after clock/zero checks.

Run via the operator session tool, not directly. A fresh clock token is required.
Any required process exit shuts down the entire launch; nothing respawns or
automatically arms, clears a fault, or retries a hardware driver.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def _native_executable(package, executable):
    """Resolve ROS libexec directly so signals reach the real node, not ros2 run."""
    from ament_index_python.packages import get_package_prefix
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(f'Required robot executable is unavailable: {package}/{executable}')
    return str(path)


def _required_exit(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(
        reason=f'Required robot process exited (code {event.returncode}); stopping the session'))]


def _configure(context):
    # This occurs before any child process is created. The clock is never
    # changed by launch, especially not after the control processes start.
    from nomad_navigation.robot_session_guard import require_robot_session
    from nomad_navigation.robot_startup_check import SPEED_PROFILES
    session = require_robot_session()
    api_config = None
    if os.environ.get('NOMAD_TRANSPORT') == 'api':
        from nomad_navigation.robot_identity import read_campus_client_config
        api_config = read_campus_client_config()
    profile = LaunchConfiguration('max_linear_mps').perform(context)
    if profile not in SPEED_PROFILES:
        raise ValueError('Use the reviewed max_linear_mps profile 0.05, 0.1 or 0.2')
    _, measured_limit = SPEED_PROFILES[profile]
    pause_option = LaunchConfiguration('network_pause_resume').perform(context)
    if pause_option not in ('true', 'false'):
        raise ValueError('network_pause_resume must be true or false')
    pause_resume = pause_option == 'true'
    if pause_resume and api_config is None:
        raise ValueError('Network pause/resume is only supported for campus API transport')
    # Validate before constructing any child. The ros2 run wrapper can exit
    # without forwarding signals to its native child in a noninteractive SSH
    # session; supervise the actual executables instead.
    base_executable = _native_executable('turn_on_wheeltec_robot', 'wheeltec_robot_node')
    tf_executable = _native_executable('tf2_ros', 'static_transform_publisher')

    def process(name, command):
        # Give signal handlers sufficient time to emit final zero commands.
        # SIGKILL is the final bound, not the normal shutdown mechanism.
        return ExecuteProcess(cmd=command, name=name, output='screen',
                              sigterm_timeout='8', sigkill_timeout='5')

    controller = process('nomad_controller', [
        'python3', '-m', 'nomad_navigation.robot_control_node', '--ros-args',
        '-p', f'max_linear_mps:={profile}',
        '-p', f'max_measured_linear_mps:={measured_limit}',
        '-p', f'network_pause_resume:={pause_option}'])
    gateway = process('nomad_gateway', [
        'python3', '-m', 'nomad_navigation.robot_gateway',
        '--max-linear-mps', profile])
    checker = process('nomad_startup_check', [
        'python3', '-m', 'nomad_navigation.robot_startup_check',
        '--max-linear-mps', profile, '--timeout-sec', '20'])
    camera = process('wheeltec_camera', [
        'ros2', 'launch', 'turn_on_wheeltec_robot', 'wheeltec_camera.launch.py'])
    base = process('wheeltec_base', [
        base_executable,
        '--ros-args', '-p', 'car_mode:=V550_akm'])
    transform = process('base_link_transform', [
        tf_executable,
        '--x', '0', '--y', '0', '--z', '0.01826',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'])
    api_client = None
    if api_config is not None:
        api_client = process('nomad_campus_api_client', [
            'python3', '-m', 'nomad_navigation.campus_api_client',
            '--server-url', api_config['server_url'], '--ca-file', api_config['ca_file'],
            '--token-file', api_config['token_file'],
            '--output-root', '/home/wheeltec/nomad_live_control/runs/campus',
            '--control-output'] + (['--network-pause-resume'] if pause_resume else []))

    def checked(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason='Read-only startup check failed; hardware drivers were not started'))]
        return [LogInfo(msg='Zero-output checks passed; starting hardware while DISARMED.'),
                base, camera, transform] + ([api_client] if api_client is not None else [])

    # Register all exit handlers before starting even the first process.
    required = [controller, gateway, base, camera, transform]
    if api_client is not None:
        required.append(api_client)
    actions = [RegisterEventHandler(OnProcessExit(target_action=child, on_exit=_required_exit))
               for child in required]
    actions.append(RegisterEventHandler(OnProcessExit(target_action=checker, on_exit=checked)))
    actions.extend([
        LogInfo(msg=(f'Clock session {session["session_id"]} verified; '
                     f'starting DISARMED, speed cap {profile} m/s.')),
        controller, gateway, checker,
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_linear_mps', default_value='0.2',
                              choices=['0.05', '0.1', '0.2'],
                              description='Explicit startup speed profile; never an arm command'),
        DeclareLaunchArgument('network_pause_resume', default_value='false', choices=['false', 'true'],
                              description='Continuous low-speed API mode: zero on timeout, resume only on fresh data'),
        OpaqueFunction(function=_configure),
    ])
