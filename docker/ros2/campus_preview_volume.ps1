# Shared temporary image channel. Dot-source only; this never starts containers.
function Get-CampusPreviewVolume {
    [CmdletBinding()]
    param([switch]$CreateIfMissing)
    $volumeName = 'socialembodiedagent-campus-preview'
    $roleKey = 'org.socialembodiedagent.role'
    $roleValue = 'campus-preview-tmpfs'
    $names = @(& docker volume ls --format '{{.Name}}')
    if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect Docker preview volumes.' }
    if ($volumeName -cnotin $names) {
        if (-not $CreateIfMissing) {
            throw 'The dedicated preview volume is missing. Start the updated headless API first.'
        }
        $created = @(& docker volume create --driver local --label "$roleKey=$roleValue" `
            --opt type=tmpfs --opt device=tmpfs --opt 'o=size=4m,mode=0700' $volumeName)
        if ($LASTEXITCODE -ne 0) { throw 'Cannot create the dedicated tmpfs preview volume.' }
    }
    # Recheck even after create: another invocation may have created that name.
    $description = @(& docker volume inspect $volumeName)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect the dedicated preview volume.' }
    try { $volumes = @(($description -join "`n") | ConvertFrom-Json) }
    catch { throw 'Docker returned invalid preview-volume metadata.' }
    if ($volumes.Count -ne 1) { throw 'Preview-volume metadata is ambiguous.' }
    $volume = $volumes[0]
    if ($volume.Name -cne $volumeName -or $volume.Driver -cne 'local' -or
        $volume.Scope -cne 'local' -or $volume.Labels.$roleKey -cne $roleValue -or
        $null -eq $volume.Options -or @($volume.Options.PSObject.Properties).Count -ne 3 -or
        $volume.Options.type -cne 'tmpfs' -or $volume.Options.device -cne 'tmpfs' -or
        $volume.Options.o -cne 'size=4m,mode=0700') {
        throw 'Preview volume conflicts with the required labelled 4 MiB/0700 local tmpfs. Nothing was changed; inspect it manually.'
    }
    return $volumeName
}
