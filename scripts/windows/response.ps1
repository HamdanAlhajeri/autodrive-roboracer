<#
.SYNOPSIS
Measure low-speed simulator acceleration and throttle-off response, or replot a recording.
.DESCRIPTION
Without RecordingDirectory, delegates to the controlled-lap workflow to collect
response trials. With a directory, analyzes saved data in an offline container.
CoastOnly estimates throttle-off deceleration from ordinary telemetry instead
of dedicated trials. Analysis writes reports; it does not update driving limits.
.EXAMPLE
.\avlite.ps1 response -TargetSpeedMps 1.5 -Trials 3
.EXAMPLE
.\avlite.ps1 response -RecordingDirectory .\log\recordings\YOUR-RUN
#>
param(
    [ValidateRange(1.0, 20.0)][double]$TargetSpeedMps = 1.5,
    [ValidateRange(3, 20)][int]$Trials = 3,
    [ValidateRange(1, 86400)][int]$MaxSeconds = 600,
    [string]$RecordingDirectory,
    [switch]$ResetSimulator,
    [switch]$NoOpen,
    [switch]$CoastOnly
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
if ($CoastOnly -and -not $RecordingDirectory) {
    throw '-CoastOnly requires -RecordingDirectory with saved telemetry.jsonl.'
}
if ($RecordingDirectory) {
    $directory = (Resolve-Path -LiteralPath $RecordingDirectory).Path
    if ($CoastOnly) {
        if (-not (Test-Path -LiteralPath (Join-Path $directory 'telemetry.jsonl'))) {
            throw 'Select a recording containing telemetry.jsonl.'
        }
        & docker run --rm --network none -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 `
            -v "${directory}:/records" `
            -v "${ProjectRoot}/src/avlite_autodrive:/opt/integration:ro" `
            --entrypoint /opt/avlite-venv/bin/python autodrive-roboracer-avlite:local `
            -m avlite_autodrive.braking_analysis /records/telemetry.jsonl `
            --output /records/braking-response.json
        if ($LASTEXITCODE -ne 0) { throw 'Braking analysis failed; see the error above.' }
        Write-Host "Report: $directory\braking-response.json. Configuration has not been changed."
        return
    }
    & docker run --rm --network none -e PYTHONDONTWRITEBYTECODE=1 `
        -v "${ProjectRoot}/src/avlite_autodrive:/opt/integration:ro" `
        -v "${directory}:/records" --entrypoint python autodrive-roboracer-avlite:local `
        -m avlite_autodrive.response_analysis /records
    if ($LASTEXITCODE -ne 0) { throw "Response analysis failed. Data retained in $directory" }
    $report = Get-Content -LiteralPath (Join-Path $directory 'response-report.json') -Raw |
        ConvertFrom-Json
    Write-Host "Accepted response trials: $($report.accepted_trials)/$($report.required_trials)"
    if (-not $report.qualified) {
        Write-Warning 'Response did not qualify. Keep braking_calibrated false; inspect response-report.json.'
    }
    if (-not $NoOpen) { Invoke-Item -LiteralPath (Join-Path $directory 'response-report.png') }
} else {
    $label = 'response-' + $TargetSpeedMps.ToString([Globalization.CultureInfo]::InvariantCulture).Replace('.', 'p')
    & (Join-Path $PSScriptRoot 'laps.ps1') -ResponseSpeedMps $TargetSpeedMps `
        -ResponseTrials $Trials -MaxSeconds $MaxSeconds -Label $label -NoOpen:$NoOpen `
        -ResetSimulator:$ResetSimulator
}
