<#
.SYNOPSIS
Run the Windows command and recording-workflow tests using mocked services.
.DESCRIPTION
Each suite gets a fresh PowerShell process so its fake Docker and simulator
commands cannot leak into the next suite. A nonzero exit stops this runner and
reports the failed suite. No running Docker engine or simulator is required.
#>
param()
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')

# Each suite needs a fresh process because workflow tests mock global commands.
foreach ($suite in @('test-windows-cli.ps1', 'test-response-workflow.ps1', 'test-measure-response.ps1')) {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $ProjectRoot "tests\$suite")
    if ($LASTEXITCODE -ne 0) { throw "Windows workflow suite failed: $suite" }
}
