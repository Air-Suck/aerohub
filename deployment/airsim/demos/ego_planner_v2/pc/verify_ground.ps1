param(
    [ValidateSet('raw','as2')][string]$Stage = 'as2',
    [string]$RunId = ('ground-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
)
$ErrorActionPreference = 'Stop'
if ($RunId -notmatch '^[A-Za-z0-9_-]+$') { throw 'RunId must contain only letters, numbers, _ or -' }
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../../../..')).Path
$evidencePath = Join-Path $projectRoot "log/ego_planner_v2/$RunId"
if (Test-Path -LiteralPath $evidencePath) { throw "Evidence directory already exists: $evidencePath" }
New-Item -ItemType Directory -Path $evidencePath | Out-Null
$image = 'ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9'
$checkCommand = 'source /opt/ros/humble/setup.bash; source /opt/px4_ws/install/setup.bash; python3 /input/probe_ground.py'
if ($Stage -eq 'as2') {
    $checkCommand = 'source /opt/ros/humble/setup.bash; source /opt/px4_ws/install/setup.bash; source /root/aerostack2_ws/install/setup.bash; source /work/ground_overlay/install/local_setup.bash; python3 /input/probe_as2_ground.py'
}
& docker run --rm --network ego-migration-ground_default `
    -e ROS_DOMAIN_ID=93 -e ROS_LOCALHOST_ONLY=0 `
    --mount "type=bind,src=$env:EGO_PC_WORK,dst=/work,readonly" `
    --mount "type=bind,src=$PSScriptRoot,dst=/input,readonly" `
    --mount "type=bind,src=$evidencePath,dst=/evidence" `
    --entrypoint bash $image -c $checkCommand
$probeExit = $LASTEXITCODE
Set-Content -LiteralPath (Join-Path $evidencePath 'exit_code.txt') -Value $probeExit
Write-Output "Evidence: $evidencePath"
exit $probeExit
