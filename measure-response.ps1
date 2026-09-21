<#
.SYNOPSIS
Measure low-speed simulator acceleration and throttle-off response, or replot a recording.
.EXAMPLE
.\measure-response.ps1 -TargetSpeedMps 1.5 -Trials 3
.EXAMPLE
.\measure-response.ps1 -RecordingDirectory .\log\recordings\YOUR-RUN
#>
param(
    [ValidateRange(1.0, 2.5)][double]$TargetSpeedMps = 1.5,
    [ValidateRange(3, 20)][int]$Trials = 3,
    [ValidateRange(1, 86400)][int]$MaxSeconds = 600,
    [string]$RecordingDirectory,
    [switch]$ResetSimulator,
    [switch]$NoOpen
)
$ErrorActionPreference = 'Stop'
if ($RecordingDirectory) {
    $directory = (Resolve-Path -LiteralPath $RecordingDirectory).Path
    & docker run --rm --network none -e PYTHONDONTWRITEBYTECODE=1 `
        -v "${PSScriptRoot}/src/avlite_autodrive:/opt/integration:ro" `
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
    & (Join-Path $PSScriptRoot 'record-one-lap.ps1') -ResponseSpeedMps $TargetSpeedMps `
        -ResponseTrials $Trials -MaxSeconds $MaxSeconds -Label $label -NoOpen:$NoOpen `
        -ResetSimulator:$ResetSimulator
}
