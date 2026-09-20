"""Launch the log-only smoke node with an optional namespace and parameter file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('nomad_navigation'), 'config', 'smoke.yaml']),
            description='Path to the smoke node YAML parameters.',
        ),
        DeclareLaunchArgument('namespace', default_value=''),
        Node(
            package='nomad_navigation',
            executable='smoke_node',
            name='nomad_smoke',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True,
        ),
    ])
