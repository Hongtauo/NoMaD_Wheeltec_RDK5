# One operator window: Windows UTC -> robot startup -> explicit arm/stop menu.
[CmdletBinding()]
param(
    [switch]$SyncOnly,
    [switch]$ManualPassword,
    [ValidateSet('0.05','0.2')][string]$MaxLinearMps = '0.2'
)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Split-Path (Split-Path $PSScriptRoot -Parent) -Parent))
if (-not (Get-Command python -ErrorAction SilentlyContinue) -or -not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    throw 'Windows Python 3 and OpenSSH are required; no additional Python packages are needed.'
}
if ('192.168.0.180' -notin @(Get-NetIPAddress -AddressFamily IPv4 | Select-Object -ExpandProperty IPAddress)) {
    throw 'Reconnect the robot Wi-Fi; reviewed PC address 192.168.0.180 is missing.'
}
$metadataText = & docker inspect nomad-robot-runtime
if ($LASTEXITCODE -ne 0) { throw 'Prepared nomad-robot-runtime container is missing. Restore the dedicated runtime first.' }
$metadata = @($metadataText | ConvertFrom-Json)[0]
if ($metadata.Config.Labels.'org.socialembodiedagent.role' -ne 'robot-live-runtime') {
    throw 'Unexpected container identity; nothing was modified.'
}
if (-not $metadata.State.Running) {
    & docker start nomad-robot-runtime
    if ($LASTEXITCODE -ne 0) { throw 'Could not start the idle runtime container.' }
}
# Clock stepping is forbidden while a model/RViz participant is active.
$check = 'import sys; sys.path.insert(0,"/workspace/SocialEmbodiedAgent/ros2_ws/scripts"); import run_robot_inference as r; r.require_free_ports((7430,7431,7432,7433))'
& docker exec nomad-robot-runtime python3 -c $check
if ($LASTEXITCODE -ne 0) { throw 'A PC preview is active. Stop it and confirm robot stationary before clock synchronization.' }
$scriptArgs = @((Join-Path $projectRoot 'ros2_ws/scripts/robot_console.py'), '--max-linear-mps', $MaxLinearMps)
if ($SyncOnly) { $scriptArgs += '--sync-only' }
if ($ManualPassword) { $scriptArgs += '--manual-password' }
& python @scriptArgs
if ($LASTEXITCODE -ne 0) { throw 'Robot operator session failed; inspect its report and the robot before restarting.' }
