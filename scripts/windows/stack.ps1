<#
.SYNOPSIS
Manage the Docker services and selected native Windows simulator.
.DESCRIPTION
Start builds the services and opens the simulator; stop shuts down driving
before the bridge. Restart reloads controller and actuator settings while
leaving the simulator open. Status and logs only inspect the existing services.
An explicit SimulatorPath takes priority over the saved simulator selection.
Without either, the launcher uses the original practice simulator.
#>
param(
    [ValidateSet('start', 'stop', 'restart', 'status', 'logs')][string]$Action = 'start',
    [string]$SimulatorPath,
    [switch]$Follow,
    [ValidateRange(1, 100000)][int]$Tail = 100
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
if ($Action -eq 'status') { Invoke-AvliteCompose ps -a; return }
if ($Action -eq 'logs') {
    $logOptions = @('--tail', $Tail)
    if ($Follow) { $logOptions += '--follow' }
    Invoke-AvliteCompose logs @logOptions avlite actuator bridge
    return
}
if ($Action -eq 'restart') {
    # Stop the source of driving commands before resetting the actuator's state.
    Invoke-AvliteCompose stop avlite
    Invoke-AvliteCompose restart actuator
    Invoke-AvliteCompose start avlite
    Write-Host 'Settings reloaded. Driving resumes with fresh sensors and Autonomous selected.'
    return
}
$runtime = Join-Path $ProjectRoot 'log\windows'
# Resolve the same executable for start and stop, including an optional local
# selection file. This avoids closing a different simulator build by mistake.
$selectionPath = Join-Path $ProjectRoot 'config\windows-simulator.json'
if ([string]::IsNullOrWhiteSpace($SimulatorPath) -and (Test-Path -LiteralPath $selectionPath)) {
    $selection = Get-Content -LiteralPath $selectionPath -Raw | ConvertFrom-Json
    if ([string]::IsNullOrWhiteSpace($selection.executable)) {
        throw 'config\windows-simulator.json must specify an executable path.'
    }
    $SimulatorPath = Join-Path $ProjectRoot $selection.executable
}
$customSimulator = -not [string]::IsNullOrWhiteSpace($SimulatorPath)
if ($customSimulator) {
    $simulatorPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($SimulatorPath)
} else {
    $simulatorPath = Join-Path $runtime 'practice\autodrive_simulator\AutoDRIVE Simulator.exe'
}

if ($Action -eq 'stop') {
    Invoke-AvliteCompose stop avlite
    Start-Sleep -Seconds 1
    Invoke-AvliteCompose stop actuator bridge
    Get-Process -Name 'AutoDRIVE Simulator' -ErrorAction SilentlyContinue |
        Where-Object { $_.Path -eq $simulatorPath } |
        ForEach-Object { $_.CloseMainWindow() | Out-Null }
    Write-Host 'AVLite stopped; the simulator was asked to close.'
    return
}

& docker info --format '{{.OSType}}'
if ($LASTEXITCODE -ne 0) { throw 'Start Docker Desktop with Linux containers, then try again.' }
New-Item -ItemType Directory -Path $runtime -Force | Out-Null

if (-not (Test-Path -LiteralPath $simulatorPath)) {
    if ($customSimulator) {
        throw "Custom simulator not found: $simulatorPath. Build the Unity scene first."
    }
    $archive = Join-Path $runtime 'autodrive_simulator_practice_windows.zip'
    $url = 'https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-icra/autodrive_simulator_practice_windows.zip'
    & curl.exe --fail --location --retry 3 --output $archive $url
    if ($LASTEXITCODE -ne 0) { throw 'The Windows simulator download failed.' }
    Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $runtime 'practice') -Force
    if (-not (Test-Path -LiteralPath $simulatorPath -PathType Leaf)) {
        throw 'The downloaded archive did not contain the expected practice simulator executable.'
    }
    # The extracted runtime is all the launcher needs. Remove the download only
    # after extraction succeeds, avoiding a second large copy on every checkout.
    Remove-Item -LiteralPath $archive -Force
}

$otherSimulator = Get-Process -Name 'AutoDRIVE Simulator' -ErrorAction SilentlyContinue |
    Where-Object { $_.Path -and $_.Path -ne $simulatorPath } | Select-Object -First 1
if ($otherSimulator) {
    throw 'Another AutoDRIVE build is open. Close it before starting this track.'
}

Invoke-AvliteCompose up -d --build bridge actuator avlite
$simulator = Get-Process -Name 'AutoDRIVE Simulator' -ErrorAction SilentlyContinue |
    Where-Object { $_.Path -eq $simulatorPath } | Select-Object -First 1
if (-not $simulator) {
    $simulatorLog = Join-Path $runtime 'simulator.log'
    $simulator = Start-Process -FilePath $simulatorPath `
        -WorkingDirectory (Split-Path -Parent $simulatorPath) -WindowStyle Normal `
        -ArgumentList '-screen-fullscreen 0 -screen-width 1280 -screen-height 720 -ip 127.0.0.1 -port 4567 -logFile', ('"' + $simulatorLog + '"') `
        -PassThru
    $simulator.Id | Set-Content -LiteralPath (Join-Path $runtime 'simulator.pid')
}
Write-Host 'Simulator running. Use Connection and Autonomous if they are not already selected.'
if ($customSimulator) {
    Write-Host ('Stop with: .\avlite.ps1 stop -SimulatorPath "' + $simulatorPath + '"')
} else {
    Write-Host 'Stop with: .\avlite.ps1 stop'
}
