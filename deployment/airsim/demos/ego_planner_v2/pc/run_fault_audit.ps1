param([Parameter(Mandatory=$true)][string[]]$RunId)
$ErrorActionPreference='Stop'
$running = & docker ps --format '{{.Names}}'
if ($LASTEXITCODE -ne 0) { throw 'Cannot check active missions' }
if ($running | Where-Object { $_ -match '^ego-(mission|compute)-' }) { throw 'Finish flight and stop test containers before offline auditing' }
foreach ($i in @(0,1)) {
    $state = & docker exec -w /var/lib/flight_board "ego-migration-ground-drone$i-1" /opt/px4-gazebo/bin/px4-commander --instance $i status
    if ($LASTEXITCODE -ne 0 -or ($state -join "`n") -notmatch '\bDisarmed\b') { throw 'Both vehicles must be disarmed' }
}
$projectRoot=(Resolve-Path (Join-Path $PSScriptRoot '../../../../..')).Path
$success=$true
foreach ($id in $RunId) {
    if ($id -notmatch '^[A-Za-z0-9_-]+$') { throw 'Invalid run ID' }
    $out=Join-Path $projectRoot "log/ego_planner_v2/$id"
    if (!(Test-Path -LiteralPath "$out/result.json")) { throw 'Missing result' }
    & docker run --rm --memory 512m --cpus 1 --network none --mount "type=bind,src=$PSScriptRoot,dst=/input,readonly" --mount "type=bind,src=$out,dst=/evidence" --entrypoint python3 ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9 /input/audit_fault_flight.py /evidence
    if ($LASTEXITCODE -ne 0) { $success=$false }
}
if (!$success) { exit 1 }
