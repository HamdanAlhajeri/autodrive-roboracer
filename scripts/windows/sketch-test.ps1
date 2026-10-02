<#
.SYNOPSIS
Check the built sketch simulator's spawn, LiDAR, lap triggers and reset behavior.
.DESCRIPTION
Starts a separate simulator process with the smoke-test flag and waits for its
JSON report. The Unity test moves the car through triggers to check the scene;
it does not test autonomous driving. Logs and a screenshot stay under log for
inspection. Visible shows the test window; otherwise it runs in batch mode.
#>
param(
    [string]$SimulatorPath,
    [switch]$Visible
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
if (-not $SimulatorPath) { $SimulatorPath = Join-Path $ProjectRoot 'log\windows\sketch\AutoDRIVE Simulator.exe' }
$SimulatorPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($SimulatorPath)
if (-not (Test-Path -LiteralPath $SimulatorPath -PathType Leaf)) {
    throw 'Build the sketch simulator with .\avlite.ps1 sketch-build first.'
}
$testDirectory = Join-Path $ProjectRoot ('log\unity-build\smoke-' + (Get-Date -Format yyyyMMdd-HHmmss))
New-Item -ItemType Directory -Path $testDirectory -Force | Out-Null
$report = Join-Path $testDirectory 'report.json'
$playerLog = Join-Path $testDirectory 'player.log'
# This unused port keeps the test separate from the normal AVLite bridge.
$arguments = @('-screen-fullscreen', '0', '-screen-width', '960',
    '-screen-height', '540', '-ip', '127.0.0.1', '-port', '4568',
    '-sketch-smoke-test', ('"' + $report + '"'), '-logFile', ('"' + $playerLog + '"'))
$windowStyle = 'Hidden'
if ($Visible) { $windowStyle = 'Normal' } else { $arguments = @('-batchmode') + $arguments }
$player = Start-Process -FilePath $SimulatorPath -ArgumentList $arguments `
    -WorkingDirectory (Split-Path -Parent $SimulatorPath) -WindowStyle $windowStyle -PassThru
Write-Host "Testing sketch simulator. Results: $testDirectory"
if (-not $player.WaitForExit(180000)) {
    $player.Kill()
    throw "Sketch smoke test timed out. See $playerLog"
}
if (-not (Test-Path -LiteralPath $report)) {
    throw "Sketch test exited without a report (exit $($player.ExitCode)). See $playerLog"
}
$result = Get-Content -LiteralPath $report -Raw | ConvertFrom-Json
$result | Format-List
if ($player.ExitCode -ne 0 -or -not $result.passed) {
    throw "Sketch smoke test failed. See $report and $playerLog"
}
Write-Host 'Scene checks passed. Autonomous clean-lap commissioning is a separate test.'
