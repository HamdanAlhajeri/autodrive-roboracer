param(
    [ValidateRange(1, 86400)][int]$Seconds = 120,
    [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$Label = 'run',
    [ValidateRange(1, 300)][int]$WaitForOdomSeconds = 30
)

$ErrorActionPreference = 'Stop'
$compose = @('compose', '-f', "$PSScriptRoot\docker-compose.avlite.yml",
    '-f', "$PSScriptRoot\docker-compose.windows.yml")
$bridge = & docker @compose ps -q --status running bridge
if ($LASTEXITCODE -ne 0 -or -not $bridge) {
    throw 'Start the simulator with .\run-windows.ps1 and connect it before recording.'
}

$startedUtc = [DateTime]::UtcNow.ToString('o')
$runName = (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '-' + $Label
$recordDir = Join-Path $PSScriptRoot "log\recordings\$runName"
$configDir = Join-Path $recordDir 'config'
New-Item -ItemType Directory -Path $configDir -Force | Out-Null
Get-ChildItem -LiteralPath "$PSScriptRoot\config" -Filter '*.yaml' |
    Copy-Item -Destination $configDir
$commit = & git -C $PSScriptRoot rev-parse HEAD
$dirty = [bool](& git -C $PSScriptRoot status --porcelain)
@{
    started_utc = $startedUtc
    requested_seconds = $Seconds
    wait_for_odometry_seconds = $WaitForOdomSeconds
    bridge_container_id = $bridge
    label = $Label
    git_commit = $commit
    working_tree_modified = $dirty
    config_note = 'Snapshot of files on disk; restart controllers after edits before recording.'
} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $recordDir 'run.json') -Encoding UTF8
& docker @compose logs --no-color --tail 100 avlite actuator bridge |
    Out-File -LiteralPath (Join-Path $recordDir 'controller-startup.log') -Encoding UTF8

Write-Host "Waiting for simulator data, then recording $Seconds seconds into $recordDir"
Write-Host 'Reproduce the behavior in the simulator while this command runs.'
Write-Host 'Finish controller/bridge restarts before recording; a restart needs a new recording.'
$recordExitCode = 0
try {
    $recordCommand = "source /opt/ros/humble/setup.bash && python -m avlite_autodrive.record --seconds $Seconds --wait-for-odom $WaitForOdomSeconds --output /records/telemetry.jsonl && python -m avlite_autodrive.plot_recording /records/telemetry.jsonl --title $runName"
    & docker @compose run --rm --no-deps -v "${recordDir}:/records" `
        --entrypoint /bin/bash avlite -lc $recordCommand
    $recordExitCode = $LASTEXITCODE
} finally {
    & docker @compose logs --no-color --timestamps --since $startedUtc avlite actuator bridge |
        Out-File -LiteralPath (Join-Path $recordDir 'controllers.log') -Encoding UTF8
}
$currentBridge = & docker @compose ps -q --status running bridge
if ($LASTEXITCODE -ne 0 -or $currentBridge -ne $bridge) {
    throw "The simulator bridge stopped or was replaced during recording. Finish starting the simulator and controllers, then rerun this command. Data and logs: $recordDir"
}
if ($recordExitCode -ne 0) {
    throw "Recording or plotting failed; see the connection or plotting message above. Finish starting the simulator and controllers before retrying. Available data and logs: $recordDir"
}
Write-Host "Graph: $recordDir\telemetry.png"
Write-Host "Spreadsheet data: $recordDir\telemetry.csv"
Write-Host 'Recording finished. The simulator and controller are still running.'
