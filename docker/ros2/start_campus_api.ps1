# Starts inference only. Never starts/arms a robot and never stops another workload.
[CmdletBinding()]
param(
    [string]$ServerIP = '10.201.91.120',
    [ValidateRange(1024,65535)][int]$Port = 8443,
    [ValidateSet('cuda','cpu')][string]$Device = 'cuda',
    [string]$CredentialsDirectory = (Join-Path $env:LOCALAPPDATA 'SocialEmbodiedAgent\campus-api'),
    [switch]$WithRViz
)
$ErrorActionPreference = 'Stop'
$address = [Net.IPAddress]::Parse($ServerIP)
if ($address.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork) { throw 'IPv4 is required.' }
if ($ServerIP -notin @(Get-NetIPAddress -AddressFamily IPv4 | Select-Object -ExpandProperty IPAddress)) {
    throw 'The server IP is not currently assigned to this Windows computer.'
}
$projectRoot = [IO.Path]::GetFullPath((Split-Path (Split-Path $PSScriptRoot -Parent) -Parent))
. (Join-Path $PSScriptRoot 'campus_preview_volume.ps1')
$imageName = 'socialembodiedagent/nomad-ros2-runtime:robot-preview-20260909'
$containerName = 'nomad-campus-api'
& docker image inspect $imageName --format '{{.Id}}'
if ($LASTEXITCODE -ne 0) { throw 'Dedicated runtime image is missing.' }
$existingNames = @(& docker ps -a --format '{{.Names}}')
if ($LASTEXITCODE -ne 0) { throw 'Docker is unavailable.' }
if ($containerName -in $existingNames) {
    throw 'nomad-campus-api already exists. Inspect docker logs / inspect; no container was replaced or restarted.'
}
$CredentialsDirectory = [IO.Path]::GetFullPath($CredentialsDirectory)
$credentialBase = [IO.Path]::GetFullPath((Join-Path $env:LOCALAPPDATA 'SocialEmbodiedAgent')) + [IO.Path]::DirectorySeparatorChar
if (-not $CredentialsDirectory.StartsWith($credentialBase, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'CredentialsDirectory must be a dedicated subdirectory of LOCALAPPDATA\SocialEmbodiedAgent.'
}
New-Item -ItemType Directory -Path $CredentialsDirectory -Force | Out-Null
if ((Get-Item -LiteralPath $CredentialsDirectory).Attributes -band [IO.FileAttributes]::ReparsePoint) {
    throw 'Credential directory must not be a junction/symlink.'
}
$userSID = [Security.Principal.WindowsIdentity]::GetCurrent().User.Value
& icacls.exe $CredentialsDirectory /inheritance:r /grant:r "*$($userSID):(OI)(CI)F" '*S-1-5-18:(OI)(CI)F' | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Could not restrict credential-directory permissions.' }
$credentialItems = @((Get-Item -LiteralPath $CredentialsDirectory)) + @(Get-ChildItem -LiteralPath $CredentialsDirectory -Force)
foreach ($credentialItem in $credentialItems) {
    if ($credentialItem.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Credential symlinks are forbidden.' }
    foreach ($accessRule in (Get-Acl -LiteralPath $credentialItem.FullName).Access) {
        $accessSID = $accessRule.IdentityReference.Translate([Security.Principal.SecurityIdentifier]).Value
        if ($accessRule.AccessControlType -eq 'Allow' -and $accessSID -notin @($userSID,'S-1-5-18')) {
            throw 'Credential path retains an explicit permission for another identity. Review its ACL manually.'
        }
    }
}
& docker run --rm --pull never --network none `
    --mount "type=bind,src=$projectRoot,dst=/workspace/SocialEmbodiedAgent,readonly" `
    --mount "type=bind,src=$CredentialsDirectory,dst=/credentials" `
    --entrypoint python3 $imageName /workspace/SocialEmbodiedAgent/ros2_ws/scripts/provision_campus_api.py `
    --credentials-dir /credentials --server-ip $ServerIP
if ($LASTEXITCODE -ne 0) { throw 'TLS provisioning failed; no service started.' }
if ($Device -eq 'cuda') {
    $freeLines = @(& docker run --rm --pull never --network none --gpus all --entrypoint nvidia-smi $imageName --query-gpu=memory.free --format=csv,noheader,nounits)
    if ($LASTEXITCODE -ne 0 -or $freeLines.Count -ne 1) { throw 'Cannot verify free GPU memory for this single-GPU deployment.' }
    $freeMiB = 0
    if (-not [int]::TryParse($freeLines[0].Trim(), [ref]$freeMiB) -or $freeMiB -lt 2048) {
        throw "Less than 2048 MiB GPU memory is free ($freeMiB MiB). Existing workloads were NOT stopped."
    }
}
$previewVolume = Get-CampusPreviewVolume -CreateIfMissing
$arguments = @('run','-d','--pull','never','--init','--name',$containerName,
    '--label','org.socialembodiedagent.role=campus-inference-api',
    '--network','host','--shm-size=8g',
    '--mount',"type=bind,src=$projectRoot,dst=/workspace/SocialEmbodiedAgent",
    '--mount',"type=volume,src=$previewVolume,dst=/workspace/SocialEmbodiedAgent/outputs/campus_api/preview,volume-nocopy",
    '--mount',"type=bind,src=$CredentialsDirectory,dst=/credentials,readonly",
    '-w','/workspace/SocialEmbodiedAgent',
    '-e','PYTHONPATH=/workspace/SocialEmbodiedAgent/ros2_ws/src/nomad_navigation:/workspace/SocialEmbodiedAgent/visualnav-transformer/train:/workspace/SocialEmbodiedAgent/diffusion_policy',
    '-e','OMP_NUM_THREADS=4','-e','MKL_NUM_THREADS=4',
    '-e','ROS_DOMAIN_ID=87','-e','ROS_LOCALHOST_ONLY=1','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp')
if ($Device -eq 'cuda') { $arguments += @('--gpus','all') }
if ($Device -eq 'cpu') { $arguments += @('--cpus','4') }
$arguments += @($imageName,'python3','/workspace/SocialEmbodiedAgent/ros2_ws/scripts/run_campus_api.py',
    '--project-root','/workspace/SocialEmbodiedAgent','--bind','0.0.0.0','--port',"$Port",
    '--cert','/credentials/server.crt','--key','/credentials/server.key','--token-file','/credentials/token',
    '--device',$Device,'--output-root','/workspace/SocialEmbodiedAgent/outputs/campus_api')
& docker @arguments
if ($LASTEXITCODE -ne 0) { throw 'Container creation failed. Inspect Docker output.' }
Write-Host "Starting https://$($ServerIP):$Port ; loading/warmup is not readiness."
Write-Host 'Check: docker logs nomad-campus-api'
Write-Host 'Stop inference only: docker stop -t 10 nomad-campus-api'
Write-Host 'No firewall was disabled, no robot process was started, and no motion was armed.'
Write-Host 'Temporary latest-only preview: dedicated 4 MiB tmpfs volume (image/prediction, no Windows image file/history/credentials).'
if ($WithRViz) {
    # Viewer startup/WSLg failure is never a reason to stop/recreate inference.
    try { & (Join-Path $PSScriptRoot 'start_campus_rviz.ps1') }
    catch { Write-Warning ('API remains running; independent RViz did not start: ' + $_.Exception.Message) }
}
