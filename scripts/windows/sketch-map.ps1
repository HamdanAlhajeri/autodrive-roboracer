<#
.SYNOPSIS
Prepare a custom track's map, commissioning profile and preview without starting driving.
.DESCRIPTION
Uses the selected track's metadata to generate config/maps/<track>.json, its plan
and overlay, and a separate avlite.<track>.yaml profile. Track is sketch (default)
or iros2026. The current AVLite configuration supplies the base settings.
Selecting that profile and building the matching Unity scene are separate steps.
#>
param(
    [ValidateSet('sketch', 'iros2026')]
    [string]$Track = 'sketch'
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$custom = Get-CustomTrack $Track
$assets = $custom.Assets -replace '\\', '/'
& docker run --rm --network none --memory 1g --cpus 2 `
    -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 `
    -v "${ProjectRoot}/config:/config" `
    -v "${ProjectRoot}/${assets}:/track:ro" `
    -v "${ProjectRoot}/src/avlite_autodrive:/opt/integration:ro" `
    --entrypoint /opt/avlite-venv/bin/python autodrive-roboracer-avlite:local `
    -m avlite_autodrive.sketch_track --metadata /track/track_metadata.json `
    --config /config/avlite.yaml --output "/config/maps/$($custom.Map).json" `
    --profile-output "/config/$($custom.Profile)"
if ($LASTEXITCODE -ne 0) { throw "$($custom.Label) track preparation failed; do not activate its profile." }
Write-Host "Offline map and plan validated. Preview: config\maps\$($custom.Map).png"
Write-Host "Unity build and driving validation are still required. See $($custom.Doc)."
