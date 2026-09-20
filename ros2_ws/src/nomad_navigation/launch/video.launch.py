"""Launch video input; the local video path is required and never hard-coded."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    node = Node(
        package='nomad_navigation', executable='video_publisher',
        name='video_publisher', namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('params_file'),
            {'video_path': ParameterValue(LaunchConfiguration('video_path'), value_type=str)},
        ],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('video_path', description='Required local video file path'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([
                FindPackageShare('nomad_navigation'), 'config', 'video.yaml'])),
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='Video input process exited'))],
        )),
        node,
    ])
