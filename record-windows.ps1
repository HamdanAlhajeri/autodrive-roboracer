param(
    [ValidateRange(1, 86400)][int]$Seconds = 120,
    [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$Label = 'run',
    [ValidateRange(1, 300)][int]$WaitForOdomSeconds = 30,
    [switch]$StopAfterLap,
    [string]$OutputDirectory,
    [ValidatePattern('^[a-zA-Z0-9][a-zA-Z0-9_.-]+$')][string]$RecorderName,
    [ValidateRange(1, 10000)][int]$Laps = 1,
    [switch]$StopOnIncident,
    [switch]$NoTrackMap
)

$ErrorActionPreference = 'Stop'
$lapTarget = if ($PSBoundParameters.ContainsKey('Laps')) { $Laps } elseif ($StopAfterLap) { 1 } else { $null }
$compose = @('compose', '-f', "$PSScriptRoot\docker-compose.avlite.yml",
    '-f', "$PSScriptRoot\docker-compose.windows.yml")

function Invoke-RecorderDocker {
    # Windows PowerShell jobs treat native stderr as error records, even for
    # successful Docker progress messages. Print them and check the exit code.
    $savedPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & docker @args 2>&1 | ForEach-Object { $_.ToString() }
        $script:recorderDockerExitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $savedPreference
    }
}

$bridge = Invoke-RecorderDocker @compose ps -q --status running bridge
if ($recorderDockerExitCode -ne 0 -or -not $bridge) {
    throw 'Start the simulator with .\run-windows.ps1 and connect it before recording.'
}

$startedUtc = [DateTime]::UtcNow.ToString('o')
$runName = (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '-' + $Label
$recordDir = if ($OutputDirectory) {
    [System.IO.Path]::GetFullPath($OutputDirectory)
} else {
    Join-Path $PSScriptRoot "log\recordings\$runName"
}
if (Test-Path -LiteralPath $recordDir) {
    throw "Recording directory already exists; choose a new directory: $recordDir"
}
$configDir = Join-Path $recordDir 'config'
New-Item -ItemType Directory -Path $configDir -Force | Out-Null
Get-ChildItem -LiteralPath "$PSScriptRoot\config" -Filter '*.yaml' |
    Copy-Item -Destination $configDir
$commit = & git -C $PSScriptRoot rev-parse HEAD
$dirty = [bool](& git -C $PSScriptRoot status --porcelain)
@{
    started_utc = $startedUtc
    requested_seconds = $Seconds
    stop_after_lap = [bool]$StopAfterLap
    requested_laps = $lapTarget
    stop_on_incident = [bool]$StopOnIncident
    track_map = -not [bool]$NoTrackMap
    wait_for_odometry_seconds = $WaitForOdomSeconds
    bridge_container_id = $bridge
    label = $Label
    git_commit = $commit
    working_tree_modified = $dirty
    config_note = 'Snapshot of files on disk; restart controllers after edits before recording.'
} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $recordDir 'run.json') -Encoding UTF8
Invoke-RecorderDocker @compose logs --no-color --tail 100 avlite actuator bridge |
    Out-File -LiteralPath (Join-Path $recordDir 'controller-startup.log') -Encoding UTF8

Write-Host "Waiting for simulator data, then recording up to $Seconds seconds into $recordDir"
if ($null -ne $lapTarget) {
    Write-Host "Recording ends one second after $lapTarget lap-counter increases, or at the time limit."
    Write-Host 'Start at the lap beginning before recording; starting mid-lap captures only the remainder.'
}
if ($StopOnIncident) {
    Write-Host 'Recording also ends when a collision or reset is detected.'
}
Write-Host 'Reproduce the behavior in the simulator while this command runs.'
Write-Host 'Finish simulator, bridge and actuator restarts before recording. AVLite can start after odometry arrives.'
Write-Host 'Recording does not stop the car. Restart recording if the bridge is replaced.'
$recordExitCode = 0
$lapOption = if ($null -ne $lapTarget) { " --laps $lapTarget" } else { '' }
$incidentOption = if ($StopOnIncident) { ' --stop-on-incident' } else { '' }
$trackOption = if ($NoTrackMap) { '' } else { ' --track-map' }
$containerOptions = if ($RecorderName) { @('--name', $RecorderName) } else { @() }
try {
    $recordCommand = "source /opt/ros/humble/setup.bash && python -m avlite_autodrive.record --seconds $Seconds --wait-for-odom $WaitForOdomSeconds$lapOption$incidentOption$trackOption --output /records/telemetry.jsonl && python -m avlite_autodrive.plot_recording /records/telemetry.jsonl --title $runName --lap-report"
    Invoke-RecorderDocker @compose run --rm --no-deps @containerOptions -v "${recordDir}:/records" `
        --entrypoint /bin/bash avlite -lc $recordCommand
    $recordExitCode = $recorderDockerExitCode
} finally {
    Invoke-RecorderDocker @compose logs --no-color --timestamps --since $startedUtc avlite actuator bridge |
        Out-File -LiteralPath (Join-Path $recordDir 'controllers.log') -Encoding UTF8
}
$currentBridge = Invoke-RecorderDocker @compose ps -q --status running bridge
if ($recorderDockerExitCode -ne 0 -or $currentBridge -ne $bridge) {
    throw "The simulator bridge stopped or was replaced during recording. Finish starting the simulator and controllers, then rerun this command. Data and logs: $recordDir"
}
if ($recordExitCode -ne 0) {
    throw "Recording or plotting failed; see the connection or plotting message above. Finish starting the simulator and controllers before retrying. Available data and logs: $recordDir"
}
if ($null -ne $lapTarget -or $StopOnIncident) {
    $summary = Get-Content -LiteralPath (Join-Path $recordDir 'telemetry.summary.json') -Raw |
        ConvertFrom-Json
    if ($summary.incident_detected) {
        Write-Warning 'A collision or reset was detected. This recording does not pass clean-lap screening.'
    }
    if ($null -ne $lapTarget) {
        if ($null -eq $summary.completed_laps -or $summary.completed_laps -lt $lapTarget) {
            Write-Warning "The requested $lapTarget laps were not confirmed (completed: $($summary.completed_laps); recording limit: $Seconds seconds). The graphs show the recorded interval."
        } elseif ($summary.clean_run) {
            Write-Host "Clean-lap screening passed: $($summary.completed_laps) consecutive laps recorded."
        } elseif (-not $summary.incident_detected) {
            Write-Warning 'The lap counter reached the target, but the recorder could not confirm a clean run. Check telemetry.summary.json.'
        }
    }
    if ($null -ne $summary.first_lap_elapsed_s) {
        Write-Host ('First finish crossing: {0:F2} s from recording start (includes waiting/startup).' -f $summary.first_lap_elapsed_s)
    }
    if ($summary.lap_times_s.Count -gt 0) {
        $lapTimes = ($summary.lap_times_s | ForEach-Object { '{0:F2}' -f $_ }) -join ', '
        Write-Host "Full laps between finish crossings: $lapTimes s"
    }
}
Write-Host "Lap report: $recordDir\telemetry.lap.png"
if (Test-Path -LiteralPath (Join-Path $recordDir 'telemetry.track.json')) {
    Write-Host "Track outline data: $recordDir\telemetry.track.json"
}
Write-Host "Debugging graphs: $recordDir\telemetry.png"
if (Test-Path -LiteralPath (Join-Path $recordDir 'telemetry.control.png')) {
    Write-Host "Corner-entry diagnostics: $recordDir\telemetry.control.png"
}
Write-Host "Spreadsheet data: $recordDir\telemetry.csv"
Write-Host 'Recording finished. Stop AVLite separately when you want to stop driving.'
