param(
    [ValidateSet('basic','ego')][string]$Stage = 'basic',
    [string]$RunId = ('flight-' + (Get-Date -Format 'yyyyMMdd-HHmmss')),
    [string]$BasicRunId = 'basic-flight-05'
)
$ErrorActionPreference = 'Stop'
foreach ($value in @($RunId,$BasicRunId)) {
    if ($value -notmatch '^[A-Za-z0-9_-]+$') { throw 'Run IDs must contain only letters, numbers, _ or -' }
}
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../../../..')).Path
$out = Join-Path $projectRoot "log/ego_planner_v2/$RunId"
$baseline = Join-Path $projectRoot "log/ego_planner_v2/$BasicRunId"
if (Test-Path -LiteralPath $out) { throw "Evidence already exists: $out" }
if ($Stage -eq 'ego') {
    $basic = Get-Content -Raw (Join-Path $baseline 'result.json') | ConvertFrom-Json
    if (!$basic.passed -or !$basic.final_disarmed -or $basic.recovery.Count) { throw 'A successful native basic flight is required' }
}
$status = & docker exec -w /var/lib/flight_board ego-migration-ground-drone0-1 /opt/px4-gazebo/bin/px4-commander status
if ($LASTEXITCODE -ne 0 -or ($status -join "`n") -notmatch '\bDisarmed\b') { throw 'Drone0 is not confirmed disarmed' }
& docker exec -w /var/lib/flight_board ego-migration-ground-drone0-1 /opt/px4-gazebo/bin/px4-commander mode auto:loiter
if ($LASTEXITCODE -ne 0) { throw 'Cannot restore ground Hold mode' }
New-Item -ItemType Directory -Path $out | Out-Null
$argsList = @('run','--init','--name',"ego-mission-$RunId",'--network','ego-migration-ground_default',
    '-e','ROS_DOMAIN_ID=93','-e','ROS_LOCALHOST_ONLY=0',
    '--mount',"type=bind,src=$env:EGO_PC_WORK,dst=/work,readonly",
    '--mount',"type=bind,src=$PSScriptRoot,dst=/input,readonly",
    '--mount',"type=bind,src=$out,dst=/evidence")
$source = 'source /opt/ros/humble/setup.bash; source /opt/px4_ws/install/setup.bash; source /work/install/humble/local_setup.bash; source /root/aerostack2_ws/install/setup.bash; source /work/ground_overlay/install/local_setup.bash; source /work/ego_runtime/install/local_setup.bash; '
$script = 'flight_mission.py'
if ($Stage -eq 'ego') {
    & docker compose --project-directory $PSScriptRoot --profile ego-flight up -d --no-deps ego
    if ($LASTEXITCODE -ne 0) { throw 'EGO capability startup failed' }
    $argsList += @('--mount',"type=bind,src=$baseline,dst=/baseline,readonly")
    $script = 'ego_flight_mission.py'
}
$argsList += @('--entrypoint','bash','ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9',
    '-c',($source + 'python3 /input/' + $script))
& docker @argsList
$flightExit = $LASTEXITCODE
Set-Content -LiteralPath (Join-Path $out 'exit_code.txt') -Value $flightExit
if (Test-Path -LiteralPath (Join-Path $out 'result.json')) {
    $report = Get-Content -Raw (Join-Path $out 'result.json') | ConvertFrom-Json
    if ($report.final_disarmed) {
        if ($Stage -eq 'ego') {
            & docker compose --project-directory $PSScriptRoot --profile ego-flight stop ego
        }
        & docker exec -w /var/lib/flight_board ego-migration-ground-drone0-1 /opt/px4-gazebo/bin/px4-commander mode auto:loiter
    }
}
Write-Output "Evidence: $out"
exit $flightExit
