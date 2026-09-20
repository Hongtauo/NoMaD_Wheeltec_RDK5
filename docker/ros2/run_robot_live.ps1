# Single PC live inference + RViz. This script never arms or starts robot control.
[CmdletBinding()]
param([ValidateScript({ $_ -eq 0 -or ($_ -ge 10 -and $_ -le 3600) })][int]$Seconds = 600)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Split-Path (Split-Path $PSScriptRoot -Parent) -Parent))
$containerName = 'nomad-robot-runtime'
$imageName = 'socialembodiedagent/nomad-ros2-runtime:robot-preview-20260909'
$roleLabel = 'robot-live-runtime'

[xml]$profile = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'robot_live_udp.xml') -Raw
$announcedAddresses = @($profile.profiles.participant.rtps.defaultUnicastLocatorList.locator | ForEach-Object { [string]$_.udpv4.address })
$wifiAddresses = @($announcedAddresses | Where-Object { $_ -ne '127.0.0.1' })
if ($announcedAddresses.Count -ne 2 -or '127.0.0.1' -notin $announcedAddresses -or $wifiAddresses.Count -ne 1) {
    throw 'Live DDS requires exactly the reviewed loopback and Wi-Fi data locators.'
}
$wifiAddress = $wifiAddresses[0]
$localAddresses = @(Get-NetIPAddress -AddressFamily IPv4 | Select-Object -ExpandProperty IPAddress)
if ($wifiAddress -notin $localAddresses) {
    throw "The reviewed DDS profile announces $wifiAddress, which is not assigned to Windows. Reconnect the robot Wi-Fi first."
}
$existing = & docker ps -aq --filter "name=^/$containerName`$"
if ($LASTEXITCODE -ne 0) { throw 'Docker is unavailable.' }
if (-not $existing) {
    & docker image inspect $imageName --format '{{.Id}}'
    if ($LASTEXITCODE -ne 0) { throw "Required prepared image is missing: $imageName. No image is pulled automatically." }
    $createArgs = @(
        'create', '--init', '--pull', 'never', '--name', $containerName,
        '--label', "org.socialembodiedagent.role=$roleLabel", '--network', 'host', '--gpus', 'all', '--shm-size=8g',
        '--mount', "type=bind,source=$projectRoot,target=/workspace/SocialEmbodiedAgent",
        '--mount', 'type=bind,source=/mnt/host/wslg/.X11-unix,target=/tmp/.X11-unix,readonly',
        '-w', '/workspace/SocialEmbodiedAgent', '-e', 'DISPLAY=:0', '-e', 'QT_X11_NO_MITSHM=1',
        '-e', 'LIBGL_ALWAYS_SOFTWARE=1', '-e', 'TZ=Asia/Shanghai', $imageName, 'sleep', 'infinity'
    )
    & docker @createArgs
    if ($LASTEXITCODE -ne 0) { throw 'Could not create the dedicated GPU/WSLg runtime container.' }
}
$metadataText = & docker inspect $containerName
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect the dedicated runtime container.' }
$metadata = @($metadataText | ConvertFrom-Json)[0]
if ($metadata.Config.Labels.'org.socialembodiedagent.role' -ne $roleLabel -or $metadata.Config.Image -ne $imageName) {
    throw 'A different container already uses the reserved runtime name. It was not modified.'
}
if ($metadata.HostConfig.NetworkMode -ne 'host' -or $metadata.HostConfig.ShmSize -lt 8589934592) {
    throw 'The runtime requires host networking and at least 8 GiB shared memory.'
}
if (-not (@($metadata.HostConfig.DeviceRequests | Where-Object { $_.Count -eq -1 }).Count)) {
    throw 'The runtime was not created with --gpus all.'
}
$projectMount = @($metadata.Mounts | Where-Object Destination -EQ '/workspace/SocialEmbodiedAgent')
if ($projectMount.Count -ne 1 -or -not $projectMount[0].RW) { throw 'Missing read/write project mount.' }
$mountSource = [IO.Path]::GetFullPath($projectMount[0].Source.Replace('/', '\')).TrimEnd('\')
if ($mountSource -ne $projectRoot.TrimEnd('\')) { throw 'Runtime points to a different project. It was not modified.' }
$displayMount = @($metadata.Mounts | Where-Object Destination -EQ '/tmp/.X11-unix')
if ($displayMount.Count -ne 1 -or $displayMount[0].RW -or $displayMount[0].Source -ne '/mnt/host/wslg/.X11-unix') {
    throw 'The reviewed read-only WSLg display socket mount is missing.'
}
foreach ($requiredEnv in @('DISPLAY=:0', 'QT_X11_NO_MITSHM=1', 'LIBGL_ALWAYS_SOFTWARE=1')) {
    if ($requiredEnv -notin $metadata.Config.Env) { throw "Missing required display setting: $requiredEnv" }
}
if (-not $metadata.State.Running) {
    & docker start $containerName
    if ($LASTEXITCODE -ne 0) { throw 'Could not start the dedicated runtime.' }
}
# Refuse to rebuild while another fixed-port live/test/RViz process is active.
$portCheck = 'import sys; sys.path.insert(0,"/workspace/SocialEmbodiedAgent/ros2_ws/scripts"); import run_robot_inference as r; r.require_free_ports((7430,7431,7432,7433))'
& docker exec $containerName python3 -c $portCheck
if ($LASTEXITCODE -ne 0) { throw 'Ports 7430..7433 are occupied. Close the existing dedicated preview first; nothing was stopped.' }
$build = 'set -eo pipefail; source /opt/ros/humble/setup.bash; mkdir -p /opt/nomad_robot_ws; cd /opt/nomad_robot_ws; colcon build --base-paths /workspace/SocialEmbodiedAgent/ros2_ws/src --packages-up-to nomad_navigation; source install/local_setup.bash; python3 -c "import nomad_navigation.nomad_node as n; print(n.__file__)"'
& docker exec $containerName bash -lc $build
if ($LASTEXITCODE -ne 0) { throw 'Build failed; no inference or RViz was launched.' }
Write-Output 'Starting PC inference + RViz. Robot control is separate and is NEVER armed by this command.'
Write-Output 'Use Ctrl+C or close RViz to stop this live session; the dedicated container is retained.'
$run = 'set -eo pipefail; source /opt/ros/humble/setup.bash; source /opt/nomad_robot_ws/install/local_setup.bash; exec python3 /workspace/SocialEmbodiedAgent/ros2_ws/scripts/run_robot_live.py --project-root /workspace/SocialEmbodiedAgent --seconds ' + $Seconds
# An interactive PTY forwards Ctrl+C to the Python runner, which also handles
# SIGTERM and stops its owned RViz process. Do not replace this with detached exec.
& docker exec -it -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp `
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/SocialEmbodiedAgent/docker/ros2/robot_live_udp.xml `
    -e PYTHONUNBUFFERED=1 $containerName bash -lc $run
if ($LASTEXITCODE -ne 0) { throw 'Live pipeline failed closed. Inspect the newest outputs/ros2_robot_live/run__* manifest and logs.' }
Write-Output 'Live session ended. No PC inference or owned RViz process remains; the runtime container is retained.'
