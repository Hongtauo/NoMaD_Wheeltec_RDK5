# Read-only cross-layer clock samples. Subprocess latency is reported as uncertainty.
[CmdletBinding()]
param([ValidateRange(1,30)][int]$Samples = 8)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$runPath = Join-Path $projectRoot ('outputs/time_sync_checks/offsets__' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmss.fffffffZ'))
New-Item -ItemType Directory -Path $runPath | Out-Null
$rows = foreach ($sample in 1..$Samples) {
    foreach ($target in @('Ubuntu-22.04','ros2-dev')) {
        $beforeMs = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
        $watch = [Diagnostics.Stopwatch]::StartNew()
        if ($target -eq 'Ubuntu-22.04') {
            $remote = & wsl.exe -d Ubuntu-22.04 -- date '+%s%N'
        } else {
            $remote = & docker.exe exec ros2-dev date '+%s%N'
        }
        $exitCode = $LASTEXITCODE
        $watch.Stop()
        $afterMs = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
        if ($exitCode -ne 0) { throw "Clock query failed: $target" }
        $remoteMs = [decimal]([string]$remote).Trim() / 1000000
        [pscustomobject]@{
            sample=$sample; target=$target; windows_before_ms=$beforeMs; windows_after_ms=$afterMs
            remote_unix_ms=$remoteMs
            estimated_remote_minus_windows_ms=$remoteMs-([decimal]$beforeMs+$afterMs)/2
            uncertainty_ms=([decimal]$afterMs-$beforeMs)/2+1
            command_elapsed_ms=$watch.Elapsed.TotalMilliseconds
            windows_step_discrepancy_ms=$afterMs-$beforeMs-$watch.Elapsed.TotalMilliseconds
        }
    }
}
$rows | Export-Csv -LiteralPath (Join-Path $runPath 'samples.csv') -NoTypeInformation -Encoding UTF8
$rows | Format-Table target,estimated_remote_minus_windows_ms,uncertainty_ms,windows_step_discrepancy_ms
Write-Output $runPath
