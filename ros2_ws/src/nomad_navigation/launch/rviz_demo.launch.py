"""Local open-loop video decision viewer. No live robot output or odometry."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root, share = LaunchConfiguration('project_root'), FindPackageShare('nomad_navigation')
    def config(name):
        return PathJoinSubstitution([share, 'config', name + '.yaml'])
    def path(*parts):
        return PathJoinSubstitution([root, *parts])
    def string(value):
        return ParameterValue(value, value_type=str)
    model = Node(package='nomad_navigation', executable='nomad_node', output='screen', parameters=[
        config('nomad'), {'model_config': string(path('visualnav-transformer', 'train', 'config', 'nomad.yaml')),
                         'checkpoint': string(path('visualnav-transformer', 'deployment', 'model_weights', 'nomad.pth')),
                         'action_stats_path': string(path('visualnav-transformer', 'train', 'vint_train', 'data', 'data_config.yaml')),
                         'output_root': string(path('outputs', 'ros2_runs')),
                         'run_name': 'rviz-video'}])
    controller = Node(package='nomad_navigation', executable='waypoint_controller', output='screen', parameters=[
        config('controller'), {'output_root': string(path('outputs', 'controller_runs')), 'dry_run': True}])
    visualizer = Node(package='nomad_navigation', executable='nomad_visualization', output='screen',
                      parameters=[config('visualization')])
    # Explicitly fictional stationary preview frame, not an odometry estimate.
    frame = Node(package='tf2_ros', executable='static_transform_publisher', name='nomad_preview_frame',
                 arguments=['--frame-id', 'nomad_preview', '--child-frame-id', 'base_link'], output='screen')
    barrier = Node(package='nomad_navigation', executable='wait_for_nomad', output='screen')
    video = Node(package='nomad_navigation', executable='video_publisher', output='screen', parameters=[
        config('video'), {'video_path': string(LaunchConfiguration('video_path')), 'loop': False,
                          'end_sec': ParameterValue(LaunchConfiguration('end_sec'), value_type=float)}])
    rviz = Node(package='rviz2', executable='rviz2', name='nomad_rviz', output='screen',
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                arguments=['-d', PathJoinSubstitution([share, 'rviz', 'nomad_preview.rviz'])])
    def model_ready(event, context):
        if event.returncode == 0:
            return [TimerAction(period=1.0, actions=[video])]
        return [EmitEvent(event=Shutdown(reason='Model readiness failed'))]
    def video_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Video failed'))]
        # Keep RViz open after EOF; freshness guards remove stale paths and zero preview.
        return []
    actions = [
        DeclareLaunchArgument('project_root', description='Mounted project root'),
        DeclareLaunchArgument('video_path', default_value=path('offline_data', '3-r.mp4')),
        DeclareLaunchArgument('end_sec', default_value='-1.0'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='false when using external RViz viewer'),
        RegisterEventHandler(OnProcessExit(target_action=barrier, on_exit=model_ready)),
        RegisterEventHandler(OnProcessExit(target_action=video, on_exit=video_exit)),
    ]
    for process in (model, controller, visualizer, frame, rviz):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A required demo process exited'))])))
    return LaunchDescription(actions + [frame, visualizer, controller, model, barrier, rviz])
