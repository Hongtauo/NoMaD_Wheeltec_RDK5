# Windows Docker Desktop/WSLg viewer. Existing inference containers are unchanged.
[CmdletBinding()]
param([ValidateRange(0,232)][int]$DomainId = 180)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$viewerName = 'nomad-rviz-viewer'
$existingId = docker ps -aq --filter "name=^/$viewerName`$"
if ($LASTEXITCODE -ne 0) { throw 'Docker is unavailable.' }
if ($existingId) { throw "Container $viewerName already exists. Inspect/close that viewer before starting another." }
$running = docker inspect --format '{{.State.Running}}' ros2-dev
if ($LASTEXITCODE -ne 0 -or $running -ne 'true') { throw 'ros2-dev must be running.' }
$arguments = @(
    'run', '--rm', '-d', '--pull', 'never', '--name', $viewerName,
    '--label', 'org.socialembodiedagent.role=rviz-preview',
    '--network', 'container:ros2-dev',
    '--mount', "type=bind,source=$projectRoot,target=/workspace/SocialEmbodiedAgent,readonly",
    '--mount', 'type=bind,source=/mnt/host/wslg/.X11-unix,target=/tmp/.X11-unix,readonly',
    '-e', 'DISPLAY=:0', '-e', 'QT_X11_NO_MITSHM=1', '-e', 'LIBGL_ALWAYS_SOFTWARE=1',
    # Humble LOCALHOST_ONLY=1 adds SHM even with custom XML; XML restricts UDP to loopback instead.
    '-e', 'TZ=Asia/Shanghai', '-e', "ROS_DOMAIN_ID=$DomainId", '-e', 'ROS_LOCALHOST_ONLY=0',
    '-e', 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
    '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/SocialEmbodiedAgent/docker/ros2/rviz_udp.xml',
    'osrf/ros:humble-desktop-full-jammy', 'bash', '-lc',
    'source /opt/ros/humble/setup.bash && exec rviz2 -d /workspace/SocialEmbodiedAgent/ros2_ws/src/nomad_navigation/rviz/nomad_preview.rviz'
)
& docker @arguments
if ($LASTEXITCODE -ne 0) { throw 'RViz viewer could not start. Check WSLg mount availability.' }
Write-Output "RViz viewer requested on domain $DomainId; inspect with: docker logs $viewerName"
Write-Output 'Closing RViz removes this temporary GUI container; project files are mounted read-only.'
