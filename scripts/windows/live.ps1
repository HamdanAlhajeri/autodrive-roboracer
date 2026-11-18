<#
.SYNOPSIS
Open the live pose view: the estimated pose next to the simulator's true pose.
.DESCRIPTION
The AVLite service serves the page while visualization.pose_view is true in
config/avlite.yaml. This command only checks that the page answers and opens it;
it never starts or changes driving. Start the stack first with avlite.ps1 start.
Port must match visualization.port in config/avlite.yaml.
#>
param(
    [ValidateRange(1024, 65535)][int]$Port = 8765,
    [switch]$NoOpen
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$url = "http://localhost:$Port"
try {
    Invoke-WebRequest -Uri "$url/geometry" -UseBasicParsing -TimeoutSec 3 | Out-Null
} catch {
    throw "The pose view is not answering at $url. Start the stack with .\avlite.ps1 start, check that visualization.pose_view is true in config\avlite.yaml, and run .\avlite.ps1 restart after editing it."
}
Write-Host "Pose view: $url"
if (-not $NoOpen) { Start-Process $url }
