"""Launch inference only. Start the video/camera after the readiness log."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = LaunchConfiguration('project_root')
    arguments = [
        DeclareLaunchArgument('project_root', description='Mounted project root on this platform'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('nomad_navigation'), 'config', 'nomad.yaml'])),
        DeclareLaunchArgument('namespace', default_value=''),
    ]
    relative_paths = {
        'model_config': ['visualnav-transformer', 'train', 'config', 'nomad.yaml'],
        'checkpoint': ['visualnav-transformer', 'deployment', 'model_weights', 'nomad.pth'],
        'action_stats_path': ['visualnav-transformer', 'train', 'vint_train', 'data', 'data_config.yaml'],
        'output_root': ['outputs', 'ros2_runs'],
    }
    for name, parts in relative_paths.items():
        arguments.append(DeclareLaunchArgument(name, default_value=PathJoinSubstitution([root, *parts])))
    node = Node(
        package='nomad_navigation', executable='nomad_node', name='nomad_inference',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            key: ParameterValue(LaunchConfiguration(key), value_type=str) for key in relative_paths
        }],
    )
    return LaunchDescription(arguments + [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason='NoMaD process exited'))])), node])
