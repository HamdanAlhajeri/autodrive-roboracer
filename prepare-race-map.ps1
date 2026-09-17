param(
    [Parameter(Mandatory = $true)][string]$RecordingDirectory,
    [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$Name = 'practice'
)

$ErrorActionPreference = 'Stop'
$recording = (Resolve-Path -LiteralPath $RecordingDirectory).Path
if (-not (Test-Path -LiteralPath (Join-Path $recording 'telemetry.jsonl')) -or
    -not (Test-Path -LiteralPath (Join-Path $recording 'telemetry.track.json'))) {
    throw 'Select a recording with telemetry.jsonl and telemetry.track.json.'
}
$maps = Join-Path $PSScriptRoot 'config\maps'
New-Item -ItemType Directory -Path $maps -Force | Out-Null
if (Test-Path -LiteralPath (Join-Path $maps "$Name.json")) {
    throw 'That map already exists. Choose a new -Name to preserve its provenance.'
}
# Isolated offline container: this command never starts the car or ROS stack.
& docker run --rm --network none -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 `
    -v "${recording}:/records:ro" -v "${maps}:/maps" `
    -v "${PSScriptRoot}/config:/config:ro" `
    -v "${PSScriptRoot}/src/avlite_autodrive:/opt/integration:ro" `
    --entrypoint /opt/avlite-venv/bin/python autodrive-roboracer-avlite:local `
    -m avlite_autodrive.race_map /records/telemetry.jsonl `
    --output "/maps/$Name.json" --config /config/avlite.yaml
if ($LASTEXITCODE -ne 0) {
    throw 'Map or plan validation failed. Inspect the error and map overlay; planned driving is not ready.'
}
Write-Host "Map and validated plan saved under $maps. Inspect $Name.png before commissioning."
Write-Host "Set planning.map_path to maps/$Name.json and driving_mode to planned in config/avlite.yaml."
