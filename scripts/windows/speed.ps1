<#
.SYNOPSIS
Show the configured map's planned speed range without starting the car.
.DESCRIPTION
Loads the shared settings and validates the race plan in an offline container.
TargetSpeedMps is a comparison threshold for the report, not a new speed setting.
Configuration is mounted read-only, so checking a target does not change tuning.
#>
param([ValidateRange(0.01, 1000)][double]$TargetSpeedMps = 10.0)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$target = $TargetSpeedMps.ToString([Globalization.CultureInfo]::InvariantCulture)
& docker run --rm --network none -e PYTHONDONTWRITEBYTECODE=1 `
    -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 `
    -v "${ProjectRoot}/config:/config:ro" `
    -v "${ProjectRoot}/src/avlite_autodrive:/opt/integration:ro" `
    --entrypoint /opt/avlite-venv/bin/python autodrive-roboracer-avlite:local `
    -m avlite_autodrive.speed_check --target-mps $target
if ($LASTEXITCODE -ne 0) { throw 'Speed/plan validation failed. Inspect the error above.' }
