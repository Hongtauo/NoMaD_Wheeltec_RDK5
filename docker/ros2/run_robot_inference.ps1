# Bounded, inference-only robot preview. Does not create a controller or camera publisher.
[CmdletBinding()]
param(
    [ValidateRange(10,3600)][int]$Seconds = 60,
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_.-]*$')][string]$ContainerName = 'nomad-robot-inference'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$profilePath = Join-Path $PSScriptRoot 'fastdds_extlab/robot_wifi_only.xml'
[xml]$profile = Get-Content -LiteralPath $profilePath -Raw
$wifiAddress = [string]$profile.profiles.participant.rtps.defaultUnicastLocatorList.locator.udpv4.address
$localAddresses = @(Get-NetIPAddress -AddressFamily IPv4 | Select-Object -ExpandProperty IPAddress)
if ($wifiAddress -notin $localAddresses) {
    throw "The profile announces $wifiAddress, which is not assigned to Windows. Recheck the robot Wi-Fi connection and XML."
}
$metadataText = & docker inspect $ContainerName
if ($LASTEXITCODE -ne 0) {
    throw "Container $ContainerName is missing. Follow ros2_ws/docs/robot_inference.md to create the dedicated GPU container."
}
$metadata = @($metadataText | ConvertFrom-Json)[0]
if ($metadata.HostConfig.NetworkMode -ne 'host') { throw 'Robot preview requires a host-network container, not bridge.' }
if (-not $metadata.HostConfig.DeviceRequests) { throw 'The container was not created with GPU access.' }
$projectMount = @($metadata.Mounts | Where-Object Destination -EQ '/workspace/SocialEmbodiedAgent')
if ($projectMount.Count -ne 1 -or -not $projectMount[0].RW) {
    throw 'The project must be mounted read/write at /workspace/SocialEmbodiedAgent to retain unique run outputs.'
}
$mountSource = [IO.Path]::GetFullPath($projectMount[0].Source.Replace('/', '\')).TrimEnd('\')
if ($mountSource -ne [IO.Path]::GetFullPath($projectRoot).TrimEnd('\')) {
    throw 'The container is mounted to a different project directory.'
}
if (-not $metadata.State.Running) {
    & docker start $ContainerName
    if ($LASTEXITCODE -ne 0) { throw 'Could not start the dedicated robot inference container.' }
}

Write-Output 'Building the current source into the dedicated container workspace (not the original ros2-dev install).'
$build = 'set -eo pipefail; source /opt/ros/humble/setup.bash; mkdir -p /opt/nomad_robot_ws; cd /opt/nomad_robot_ws; colcon build --base-paths /workspace/SocialEmbodiedAgent/ros2_ws/src --packages-up-to nomad_navigation; source install/local_setup.bash; python3 -c "import nomad_navigation.nomad_node as n; print(n.__file__)"'
& docker exec $ContainerName bash -lc $build
if ($LASTEXITCODE -ne 0) { throw 'Build failed; inference was not started.' }

# Use one process/Context for inference and its auditor. Do not start RViz, echo,
# camera_check or a ROS daemon with the same fixed participant/ports concurrently.
$run = 'set -eo pipefail; source /opt/ros/humble/setup.bash; source /opt/nomad_robot_ws/install/local_setup.bash; exec python3 /workspace/SocialEmbodiedAgent/ros2_ws/scripts/run_robot_inference.py --project-root /workspace/SocialEmbodiedAgent --seconds ' + $Seconds + ' 2>&1'
& docker exec -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp `
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/SocialEmbodiedAgent/docker/ros2/fastdds_extlab/robot_wifi_only.xml `
    -e PYTHONUNBUFFERED=1 $ContainerName bash -lc $run
if ($LASTEXITCODE -ne 0) { throw 'Robot inference preview did not pass. Inspect the new report under outputs/ros2_robot_inference.' }
Write-Output 'Bounded preview finished. The container is left running; no inference process is left running.'
