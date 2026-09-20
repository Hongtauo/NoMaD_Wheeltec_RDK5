"""Preview-only controller, independent of model/video process lifecycle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root', description='Project root for persistent preview logs'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('nomad_navigation'), 'config', 'controller.yaml'])),
        DeclareLaunchArgument('output_root', default_value=PathJoinSubstitution([
            LaunchConfiguration('project_root'), 'outputs', 'controller_runs'])),
        Node(package='nomad_navigation', executable='waypoint_controller', name='waypoint_controller',
             output='screen', parameters=[LaunchConfiguration('params_file'), {
                 'output_root': ParameterValue(LaunchConfiguration('output_root'), value_type=str),
             }]),
    ])
