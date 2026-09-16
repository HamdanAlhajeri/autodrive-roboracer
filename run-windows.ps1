param([switch]$Stop)

$ErrorActionPreference = 'Stop'
$compose = @('compose', '-f', "$PSScriptRoot\docker-compose.avlite.yml",
    '-f', "$PSScriptRoot\docker-compose.windows.yml")
$runtime = Join-Path $PSScriptRoot 'log\windows'
$simulatorPath = Join-Path $runtime 'practice\autodrive_simulator\AutoDRIVE Simulator.exe'

function Invoke-Compose {
    & docker @compose @args
    if ($LASTEXITCODE -ne 0) { throw 'Docker Compose failed. Check that Docker Desktop is running.' }
}

if ($Stop) {
    Invoke-Compose stop avlite
    Start-Sleep -Seconds 1
    Invoke-Compose stop actuator bridge
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
    $archive = Join-Path $runtime 'autodrive_simulator_practice_windows.zip'
    $url = 'https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-icra/autodrive_simulator_practice_windows.zip'
    & curl.exe --fail --location --retry 3 --output $archive $url
    if ($LASTEXITCODE -ne 0) { throw 'The Windows simulator download failed.' }
    Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $runtime 'practice') -Force
}

Invoke-Compose up -d --build bridge actuator avlite
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
Write-Host 'Stop with: .\run-windows.ps1 -Stop'
