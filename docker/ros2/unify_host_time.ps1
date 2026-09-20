# Run only when explicitly requested: host Windows NTP -> Hyper-V -> WSL/containers.
# Does not reboot WSL/Docker, modify NTP peers, or change ROS controller code.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = [Security.Principal.WindowsPrincipal]::new($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run this script in an elevated PowerShell.'
}
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$runName = 'unify-time__' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmss.fffffffZ')
$runPath = Join-Path $projectRoot ('outputs/time_sync_checks/' + $runName)
New-Item -ItemType Directory -Path $runPath | Out-Null
Start-Transcript -LiteralPath (Join-Path $runPath 'apply.log') | Out-Null
$status = [ordered]@{ status='started'; output=$runPath; changes=@(); error=$null }
try {
    if ((Get-CimInstance Win32_ComputerSystem).PartOfDomain) {
        throw 'Domain-joined computer: preserve domain time policy and review manually.'
    }
    $service = Get-Service W32Time
    $parameters = Get-ItemProperty 'HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\Parameters'
    $enabled = & wsl.exe -d Ubuntu-22.04 -u root -- systemctl is-enabled systemd-timesyncd
    $active = & wsl.exe -d Ubuntu-22.04 -u root -- systemctl is-active systemd-timesyncd
    $implicit = & wsl.exe -d Ubuntu-22.04 -u root -- cat /sys/module/hv_utils/parameters/timesync_implicit
    if ($LASTEXITCODE -ne 0 -or $implicit.Trim() -ne 'Y') {
        throw 'Expected Hyper-V implicit synchronization is not enabled; no changes made.'
    }
    [ordered]@{
        windows_service_status=[string]$service.Status
        windows_service_start_type=[string]$service.StartType
        windows_ntp_type=$parameters.Type
        windows_ntp_server=$parameters.NtpServer
        ubuntu_timesyncd_enabled=[string]$enabled
        ubuntu_timesyncd_active=[string]$active
        hyperv_implicit=[string]$implicit
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runPath 'before.json') -Encoding UTF8

    & wsl.exe -d Ubuntu-22.04 -u root -- systemctl disable --now systemd-timesyncd.service
    if ($LASTEXITCODE -ne 0) { throw 'Failed to stop/disable Ubuntu independent NTP.' }
    $status.changes += 'Ubuntu systemd-timesyncd stopped and disabled; Hyper-V implicit sync retained'
    Set-Service -Name W32Time -StartupType Automatic
    $status.changes += 'Windows W32Time startup set to Automatic'
    Start-Service -Name W32Time
    $status.changes += 'Windows W32Time started; existing NTP peers unchanged'
    & w32tm.exe /resync /rediscover
    if ($LASTEXITCODE -ne 0) { throw 'Windows resync failed; inspect the log before any further changes.' }
    & w32tm.exe /query /status /verbose
    & w32tm.exe /query /configuration
    $status.status = 'applied_pending_measurement'
} catch {
    $status.status = 'incomplete'
    $status.error = $_.Exception.Message
    Write-Warning $status.error
} finally {
    $status | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $runPath 'apply_result.json') -Encoding UTF8
    Stop-Transcript | Out-Null
}
if ($status.status -eq 'incomplete') { exit 1 }
