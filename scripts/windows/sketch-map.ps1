<#
.SYNOPSIS
Prepare the sketch map, commissioning profile and preview without starting driving.
.DESCRIPTION
Uses the supplied track metadata to generate config/maps/sketch.json, its plan
and overlay, and a separate avlite.sketch.yaml profile. The current AVLite
configuration supplies the base settings. Selecting that profile and building
the matching Unity scene are separate steps.
#>
param()
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
& docker run --rm --network none --memory 1g --cpus 2 `
    -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 `
    -v "${ProjectRoot}/config:/config" `
    -v "${ProjectRoot}/assets/tracks/sketch_track:/track:ro" `
    -v "${ProjectRoot}/src/avlite_autodrive:/opt/integration:ro" `
    --entrypoint /opt/avlite-venv/bin/python autodrive-roboracer-avlite:local `
    -m avlite_autodrive.sketch_track --metadata /track/track_metadata.json `
    --config /config/avlite.yaml --output /config/maps/sketch.json `
    --profile-output /config/avlite.sketch.yaml
if ($LASTEXITCODE -ne 0) { throw 'Sketch track preparation failed; do not activate its profile.' }
Write-Host 'Offline map and plan validated. Preview: config\maps\sketch.png'
Write-Host 'Unity build and driving validation are still required. See docs\sketch-track.md.'
