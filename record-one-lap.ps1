<#
.SYNOPSIS
Record consecutive simulator laps and open the trajectory/speed graph.
.DESCRIPTION
Requires the simulator, bridge and actuator to be running. Stops AVLite, asks
you to reset the car, reloads actuator settings, records before starting AVLite,
then stops AVLite when the requested laps finish, an incident occurs, or the
time limit is reached. The simulator, bridge and actuator stay running.
Observed LiDAR surfaces are captured for the path graph unless -NoTrackMap is set.
.EXAMPLE
.\record-one-lap.ps1
.EXAMPLE
.\record-one-lap.ps1 -MaxSeconds 300 -Label controller-test -NoOpen
.EXAMPLE
.\record-one-lap.ps1 -Laps 3 -Label screening-2p5
#>
param(
    [ValidateRange(1, 86400)][int]$MaxSeconds = 600,
    [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$Label = 'one-lap',
    [ValidateRange(1, 300)][int]$WaitForOdomSeconds = 30,
    [switch]$NoOpen,
    [ValidateRange(1, 10000)][int]$Laps = 1,
    [switch]$NoTrackMap,
    [switch]$ResetSimulator,
    [ValidateRange(1.0, 2.5)][double]$ResponseSpeedMps,
    [ValidateRange(3, 20)][int]$ResponseTrials = 3
)

$ErrorActionPreference = 'Stop'
$responseMode = $PSBoundParameters.ContainsKey('ResponseSpeedMps')
if ($responseMode) { $Laps = $ResponseTrials + 1 }
if ($Laps -gt 1 -and -not $PSBoundParameters.ContainsKey('Label')) {
    $Label = "$Laps-laps"
}
$compose = @('compose', '-f', "$PSScriptRoot\docker-compose.avlite.yml",
    '-f', "$PSScriptRoot\docker-compose.windows.yml")

function Invoke-LapCompose {
    & docker @compose @args
    if ($LASTEXITCODE -ne 0) { throw "Docker Compose failed: $args" }
}

function Get-ReadySample([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path)) { return $null }
    $line = Get-Content -LiteralPath $Path -Tail 1
    if (-not $line) { return $null }
    try {
        $sample = $line | ConvertFrom-Json
        $sampleTime = [DateTimeOffset]::Parse($sample.timestamp_utc)
    } catch {
        return $null # The recorder may still be writing the last JSON line.
    }
    $age = ([DateTimeOffset]::UtcNow - $sampleTime).TotalSeconds
    if ($age -lt -2 -or $age -gt 2) { return $null }
    foreach ($field in @('x', 'y', 'speed', 'lap_count', 'collision_count',
            'throttle_command', 'steering_command')) {
        if ($null -eq $sample.$field) { return $null }
    }
    foreach ($field in @('odom_age_s', 'lap_count_age_s', 'collision_count_age_s',
            'throttle_command_age_s', 'steering_command_age_s')) {
        if ($null -eq $sample.$field -or $sample.$field -gt 0.5) { return $null }
    }
    # Wait for the car to settle after stopping/resetting before starting the lap.
    if ([Math]::Abs($sample.speed) -gt 0.1) { return $null }
    return $sample
}

$bridge = Invoke-LapCompose ps -q --status running bridge
$actuator = Invoke-LapCompose ps -q --status running actuator
$avlite = Invoke-LapCompose ps -a -q avlite
if (-not $bridge -or -not $actuator -or -not $avlite) {
    throw 'Run .\run-windows.ps1 first and connect the simulator, then retry this script.'
}

$runName = (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '-' + $Label
$recordDir = Join-Path $PSScriptRoot "log\recordings\$runName"
$recorderName = 'autodrive-lap-' + [Guid]::NewGuid().ToString('N')
$responseName = 'autodrive-response-' + [Guid]::NewGuid().ToString('N')
$recording = Join-Path $recordDir 'telemetry.jsonl'
$summaryPath = Join-Path $recordDir 'telemetry.summary.json'
$graph = Join-Path $recordDir 'telemetry.lap.png'
$job = $null
$controllerStopped = $false
$drivingStarted = $false

function Stop-LapController {
    if ($responseMode) {
        & docker stop --time 5 $responseName | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'Could not stop response controller' }
    } else {
        Invoke-LapCompose stop avlite
    }
    if ([Math]::Abs($sample.throttle_command) -gt 1 -or
        [Math]::Abs($sample.steering_command) -gt 1) { return $null }
}

try {
    Invoke-LapCompose stop avlite
    $controllerStopped = $true
    Invoke-LapCompose restart actuator
    Write-Host 'AVLite stopped. Reset the car to the starting position in the simulator.'
    Write-Host "Actuator settings reloaded. Recording will stop after $Laps laps or at the first collision/reset."
    Write-Host 'Keep Connection and Autonomous selected. Leave the simulator open.'
    if ($ResetSimulator) {
        Invoke-LapCompose exec -T bridge /bin/bash -lc 'source /opt/ros/humble/setup.bash && python3 -m avlite_autodrive.simulator_reset'
    } else {
        [void](Read-Host 'Press Enter after resetting; recording and driving will then start automatically')
    }

    $options = @{
        Seconds = $MaxSeconds
        Label = $Label
        WaitForOdomSeconds = $WaitForOdomSeconds
        Laps = $Laps
        StopOnIncident = $true
        NoTrackMap = [bool]$NoTrackMap
        ResponseData = [bool]$responseMode
        OutputDirectory = $recordDir
        RecorderName = $recorderName
    }
    $job = Start-Job -ArgumentList $PSScriptRoot, $options -ScriptBlock {
        param($projectRoot, $recordOptions)
        $ErrorActionPreference = 'Stop'
        & (Join-Path $projectRoot 'record-windows.ps1') @recordOptions
    }
    $readyDeadline = [DateTime]::UtcNow.AddSeconds($WaitForOdomSeconds + 30)
    $finishDeadline = $readyDeadline.AddSeconds($MaxSeconds + 60)
    while ($job.State -in @('NotStarted', 'Running')) {
        Receive-Job -Job $job -ErrorAction Stop
        if (Test-Path -LiteralPath $summaryPath) {
            if (-not $controllerStopped) {
                Stop-LapController
                $controllerStopped = $true
                Write-Host 'Capture ended; AVLite stopped. Finishing the graphs...'
            }
        } elseif (-not $drivingStarted) {
            $sample = Get-ReadySample $recording
            if ($sample) {
                $currentBridge = Invoke-LapCompose ps -q --status running bridge
                if ($currentBridge -ne $bridge) {
                    throw 'The bridge changed during setup. Rerun this script after startup finishes.'
                }
                # Docker can take time to respond; do not start on an expired
                # sample or after a short capture finished during that check.
                if ((Test-Path -LiteralPath $summaryPath) -or $job.State -ne 'Running' -or
                    -not (Get-ReadySample $recording)) {
                    continue
                }
                # Treat even a partially successful start as requiring cleanup.
                $controllerStopped = $false
                if ($responseMode) {
                    $speedText = $ResponseSpeedMps.ToString([Globalization.CultureInfo]::InvariantCulture)
                    $command = "source /opt/ros/humble/setup.bash && exec python -m avlite_autodrive.runner --config /config/avlite.yaml --response-speed-mps $speedText --response-trials $ResponseTrials"
                    Invoke-LapCompose run --no-deps -d --name $responseName `
                        --entrypoint /bin/bash avlite -lc $command
                } else {
                    Invoke-LapCompose start avlite
                }
                $drivingStarted = $true
                Write-Host "Recording is ready. AVLite is driving; capture ends after $Laps laps or an incident."
            } elseif ([DateTime]::UtcNow -gt $readyDeadline) {
                throw 'No fresh stationary odometry and lap counters arrived. Check the simulator connection.'
            }
        }
        if ($responseMode -and $drivingStarted -and -not $controllerStopped) {
            $running = & docker inspect --format '{{.State.Running}}' $responseName
            if ($LASTEXITCODE -ne 0 -or $running -ne 'true') {
                throw "Response controller exited. Check response-controller.log in $recordDir"
            }
        }
        if ([DateTime]::UtcNow -gt $finishDeadline) {
            throw "Recording did not finish in time. Available data: $recordDir"
        }
        Start-Sleep -Milliseconds 200
    }
    Receive-Job -Job $job -ErrorAction Stop
    if ($job.State -ne 'Completed') {
        throw "Recording job ended with state $($job.State). Available data: $recordDir"
    }
    if (-not $drivingStarted) {
        throw "Recording ended before driving could start. Check the connection or increase -MaxSeconds. Data: $recordDir"
    }
} finally {
    if (-not $controllerStopped) {
        try { Stop-LapController }
        catch { Write-Warning "Could not stop AVLite: $_" }
    }
    if ($job) {
        # Stop-Job alone can leave a Docker one-off container alive. This name is
        # unique to this invocation; never stop another recorder or the bridge.
        if ($job.State -in @('NotStarted', 'Running')) {
            try { & docker stop --time 5 $recorderName 2>$null | Out-Null }
            catch { Write-Verbose "Recorder was not running yet: $_" }
            Stop-Job -Job $job -ErrorAction SilentlyContinue
        }
        try { & docker rm --force $recorderName 2>$null | Out-Null }
        catch { Write-Verbose "Recorder container already removed: $_" }
        Remove-Job -Job $job -Force -ErrorAction SilentlyContinue
    }
    if ($responseMode -and (Test-Path -LiteralPath $recordDir)) {
        $savedPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            & docker logs $responseName 2>&1 |
                Out-File -LiteralPath (Join-Path $recordDir 'response-controller.log') -Encoding UTF8
        } finally { $ErrorActionPreference = $savedPreference }
        try { & docker rm --force $responseName 2>$null | Out-Null }
        catch { Write-Verbose "Response controller already removed: $_" }
        if (Test-Path -LiteralPath $summaryPath) {
            & (Join-Path $PSScriptRoot 'measure-response.ps1') -RecordingDirectory $recordDir -NoOpen
        }
    }
}

if (-not (Test-Path -LiteralPath $graph)) {
    throw "No lap graph was produced. Check the recording logs in $recordDir"
}
Write-Host "Finished. AVLite is stopped; the simulator remains open. Graph: $graph"
if (-not $NoOpen) { Invoke-Item -LiteralPath $graph }
