<#
.SYNOPSIS
Collect guided straight throttle-off trials, or analyze an existing recording.
.DESCRIPTION
Uses planned steering on the configured map, with a temporary 1.0-2.5 m/s
ceiling. The simulator must already be connected. Prompts for a reset, records
before driving, and stops the test controller on completion or failure.
Three trials use four laps to allow an extra opportunity to reach the straight.
Neither this script nor its analyzer changes saved driving settings.
.EXAMPLE
.\measure-response.ps1 -TargetSpeedMps 1.5 -Trials 3
.EXAMPLE
.\measure-response.ps1 -RecordingDirectory .\log\recordings\YOUR-RUN
#>
[CmdletBinding(DefaultParameterSetName = 'Run')]
param(
    [Parameter(ParameterSetName = 'Run')]
    [ValidateRange(1.0, 2.5)][double]$TargetSpeedMps = 1.5,
    [Parameter(ParameterSetName = 'Run')]
    [ValidateRange(3, 20)][int]$Trials = 3,
    [Parameter(ParameterSetName = 'Run')]
    [ValidateRange(30, 86400)][int]$MaxSeconds = 600,
    [Parameter(Mandatory = $true, ParameterSetName = 'Analyze')]
    [string]$RecordingDirectory,
    [switch]$NoOpen
)

$ErrorActionPreference = 'Stop'
if ($PSCmdlet.ParameterSetName -eq 'Run') {
    $speedLabel = $TargetSpeedMps.ToString('0.0#', [Globalization.CultureInfo]::InvariantCulture).Replace('.', 'p')
    $label = "response-$speedLabel"
    $recording = Join-Path $PSScriptRoot ('log\recordings\' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '-' + $label)
    Write-Host "Response test: $Trials coast attempts at $TargetSpeedMps m/s; $($Trials + 1) laps maximum."
    Write-Host 'Keep the simulator on the practice layout matching the configured map.'
    Write-Host 'The saved speed/throttle settings stay unchanged; the test uses a temporary speed ceiling.'
    & (Join-Path $PSScriptRoot 'record-one-lap.ps1') -Laps ($Trials + 1) -Label $label `
        -MaxSeconds $MaxSeconds -OutputDirectory $recording -ResponseSpeedMps $TargetSpeedMps `
        -ResponseTrials $Trials -NoOpen
} else {
    $recording = (Resolve-Path -LiteralPath $RecordingDirectory).Path
}
if (-not (Test-Path -LiteralPath (Join-Path $recording 'telemetry.jsonl'))) {
    throw 'Select a recording containing telemetry.jsonl.'
}
& docker run --rm --network none -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 `
    -v "${recording}:/records" `
    -v "${PSScriptRoot}/src/avlite_autodrive:/opt/integration:ro" `
    --entrypoint /opt/avlite-venv/bin/python autodrive-roboracer-avlite:local `
    -m avlite_autodrive.response_analysis /records
if ($LASTEXITCODE -ne 0) { throw "Response analysis failed. Telemetry remains in $recording" }
$report = Join-Path $recording 'response-report.json'
$graph = Join-Path $recording 'response-report.png'
$result = Get-Content -LiteralPath $report -Raw | ConvertFrom-Json
Write-Host "Report: $report"
Write-Host "Graph: $graph"
Write-Host $result.next_step
if (-not $NoOpen) { Invoke-Item -LiteralPath $graph }
