# Campus operator: Windows UTC -> enrolled robot startup -> explicit arm menu.
# Does not start Docker, change networking, add SSH host keys or arm automatically.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$RobotHost,
    [switch]$SyncOnly,
    [switch]$NetworkPauseResume,
    [switch]$OperatorPauseResume,
    [switch]$ManualPassword,
    [ValidateSet('0.05','0.1','0.2')][string]$MaxLinearMps = '0.05'
)
$ErrorActionPreference = 'Stop'
if ($OperatorPauseResume -and (-not $NetworkPauseResume -or $SyncOnly)) {
    throw 'OperatorPauseResume requires NetworkPauseResume and a normal campus session (not SyncOnly).'
}
$projectRoot = [IO.Path]::GetFullPath((Split-Path (Split-Path $PSScriptRoot -Parent) -Parent))
if (-not (Get-Command python -ErrorAction SilentlyContinue) -or -not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    throw 'Windows Python 3 and OpenSSH are required; no extra Python packages are needed.'
}
$robotAddress = $null
if (-not [Net.IPAddress]::TryParse($RobotHost, [ref]$robotAddress) -or
    $robotAddress.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
    $robotAddress.ToString() -ne $RobotHost) {
    throw 'RobotHost must be a canonical IPv4 address, for example 10.201.207.37.'
}
$scriptArgs = @((Join-Path $projectRoot 'ros2_ws/scripts/robot_console.py'),
    '--transport', 'api', '--robot-host', $RobotHost, '--max-linear-mps', $MaxLinearMps)
if ($SyncOnly) { $scriptArgs += '--sync-only' }
if ($NetworkPauseResume) { $scriptArgs += '--network-pause-resume' }
if ($OperatorPauseResume) { $scriptArgs += '--operator-pause-resume' }
if ($ManualPassword) { $scriptArgs += '--manual-password' }
& python @scriptArgs
if ($LASTEXITCODE -ne 0) { throw 'Campus robot operator failed closed; inspect its report and the robot before restarting.' }
