param([Parameter(Mandatory = $true)][string]$RecordingDirectory)

$ErrorActionPreference = 'Stop'
$recording = (Resolve-Path -LiteralPath $RecordingDirectory).Path
if (-not (Test-Path -LiteralPath (Join-Path $recording 'telemetry.jsonl'))) {
    throw 'Select a recording containing telemetry.jsonl.'
}
& docker run --rm --network none -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 `
    -v "${recording}:/records" `
    -v "${PSScriptRoot}/src/avlite_autodrive:/opt/integration:ro" `
    --entrypoint /opt/avlite-venv/bin/python autodrive-roboracer-avlite:local `
    -m avlite_autodrive.braking_analysis /records/telemetry.jsonl `
    --output /records/braking-response.json
if ($LASTEXITCODE -ne 0) { throw 'Braking analysis failed; see the error above.' }
Write-Host "Report: $recording\braking-response.json. Configuration has not been changed."
