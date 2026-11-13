param(
    [ValidateSet('basic1','dual','crossing')][string]$Stage = 'basic1',
    [Parameter(Mandatory=$true)][string]$RunId,
    [string]$BasicRunId = 'drone1-basic-01',
    [ValidateSet('east','west')][string]$PreferredDirection = 'east'
)
$ErrorActionPreference = 'Stop'
$fleetStage = $Stage -in @('dual','crossing')
$configName = if ($Stage -eq 'crossing') { 'ego_crossing.yaml' } else { 'ego_dual.yaml' }
$missionCommand = if ($Stage -eq 'crossing') { 'python3 /input/crossing_flight_mission.py' } else { "python3 /input/dual_flight_mission.py --stage $Stage --preferred-direction $PreferredDirection" }
foreach ($value in @($RunId,$BasicRunId)) {
    if ($value -notmatch '^[A-Za-z0-9_-]+$') { throw 'Invalid run ID' }
}
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../../../..')).Path
$out = Join-Path $projectRoot "log/ego_planner_v2/$RunId"
$baseline = Join-Path $projectRoot "log/ego_planner_v2/$BasicRunId"
if (Test-Path -LiteralPath $out) { throw "Evidence exists: $out" }
if ($fleetStage) {
    $basic = Get-Content -Raw (Join-Path $baseline 'result.json') | ConvertFrom-Json
    if (!$basic.passed -or !$basic.both_disarmed -or $basic.recovery.Count) { throw 'Drone1 basic prerequisite failed' }
}
foreach ($i in @(0,1)) {
    $status = & docker exec -w /var/lib/flight_board "ego-migration-ground-drone$i-1" /opt/px4-gazebo/bin/px4-commander --instance $i status
    if ($LASTEXITCODE -ne 0 -or ($status -join "`n") -notmatch '\bDisarmed\b') { throw "Drone$i not confirmed disarmed" }
    & docker exec -w /var/lib/flight_board "ego-migration-ground-drone$i-1" /opt/px4-gazebo/bin/px4-commander --instance $i mode auto:loiter
    if ($LASTEXITCODE -ne 0) { throw 'Cannot restore ground Hold' }
}
New-Item -ItemType Directory -Path $out | Out-Null
$image = 'ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9'
$compute = "ego-compute-$RunId"
$client = "ego-mission-$RunId"
$envArgs = @('--network','ego-migration-ground_default','-e','ROS_DOMAIN_ID=93','-e','ROS_LOCALHOST_ONLY=0')
$source = 'source /opt/ros/humble/setup.bash; source /opt/px4_ws/install/setup.bash; source /work/install/humble/local_setup.bash; source /root/aerostack2_ws/install/setup.bash; source /work/ground_overlay/install/local_setup.bash; source /work/ego_runtime/install/local_setup.bash; '
Get-ChildItem -LiteralPath $PSScriptRoot -File | Where-Object Name -NE '.env' | Get-FileHash -Algorithm SHA256 | ConvertTo-Json -Depth 4 | Set-Content (Join-Path $out 'source_manifest.json')
$snapshot = Join-Path $out 'source_snapshot'
New-Item -ItemType Directory -Path $snapshot | Out-Null
Get-ChildItem -LiteralPath $PSScriptRoot -File | Where-Object Name -NE '.env' | Copy-Item -Destination $snapshot
if ($fleetStage) {
    & docker run -d --init --name $compute @envArgs -e "EGO_CONFIG_FILE=/input/deployment/airsim/demos/ego_planner_v2/pc/$configName" --mount "type=bind,src=$env:EGO_PC_WORK,dst=/work,readonly" --mount "type=bind,src=$projectRoot,dst=/input,readonly" --entrypoint bash $image /input/deployment/airsim/demos/ego_planner_v2/pc/start_ego_runtime.sh
    if ($LASTEXITCODE -ne 0) { throw 'EGO startup failed' }
}
$argsList = @('run','--init','--name',$client) + $envArgs + @(
    '--mount',"type=bind,src=$env:EGO_PC_WORK,dst=/work,readonly",
    '--mount',"type=bind,src=$PSScriptRoot,dst=/input,readonly",
    '--mount',"type=bind,src=$out,dst=/evidence")
if ($fleetStage) { $argsList += @('--mount',"type=bind,src=$baseline,dst=/baseline,readonly") }
$argsList += @('--entrypoint','bash',$image,'-c',($source + $missionCommand))
& docker @argsList
$flightExit = $LASTEXITCODE
Set-Content (Join-Path $out 'exit_code.txt') $flightExit
& docker logs $client 2>&1 | Set-Content (Join-Path $out 'mission.log')
& docker inspect $client | Set-Content (Join-Path $out 'mission-inspect.json')
$started = & docker inspect --format '{{.State.StartedAt}}' $client
& docker logs --since $started ego-migration-ground-as2-1 2>&1 | Set-Content (Join-Path $out 'as2-runtime.log')
if (Test-Path -LiteralPath (Join-Path $out 'result.json')) {
    $report = Get-Content -Raw (Join-Path $out 'result.json') | ConvertFrom-Json
    if ($report.both_disarmed) {
        if ($fleetStage) {
            & docker stop -s SIGINT -t 15 $compute
            & docker logs $compute 2>&1 | Set-Content (Join-Path $out 'ego-runtime.log')
            & docker inspect $compute | Set-Content (Join-Path $out 'compute-inspect.json')
        }
        foreach ($i in @(0,1)) {
            & docker exec -w /var/lib/flight_board "ego-migration-ground-drone$i-1" /opt/px4-gazebo/bin/px4-commander --instance $i mode auto:loiter
            & docker exec -w /var/lib/flight_board "ego-migration-ground-drone$i-1" /opt/px4-gazebo/bin/px4-commander --instance $i status | Set-Content (Join-Path $out "commander-drone$i.txt")
        }
    }
}
Write-Output "Evidence: $out"
exit $flightExit
