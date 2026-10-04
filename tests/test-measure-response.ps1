# Isolated workflow checks. All Docker calls are mocked; no simulator is driven.
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$testRoot = Join-Path $projectRoot ('log/script-checks/response-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testRoot -Force | Out-Null
$stub = @'
param($Seconds, $Label, $WaitForOdomSeconds, $OutputDirectory, $RecorderName, $Laps, $StopOnIncident, $NoTrackMap, $ResponseData)
$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$fixtureRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$scenario = Get-Content -LiteralPath (Join-Path $fixtureRoot 'scenario')
$row = @{ timestamp_utc = [DateTimeOffset]::UtcNow.ToString('o'); elapsed_s = 0
    x = 0; y = 0; speed = 0; odom_age_s = 0; lap_count = 0; lap_count_age_s = 0
    collision_count = 0; collision_count_age_s = 0
    throttle_command = 0; steering_command = 0
    throttle_command_age_s = 0; steering_command_age_s = 0 }
if ($scenario -eq 'missing-baseline') { $row.Remove('lap_count') }
$row | ConvertTo-Json -Compress | Set-Content -LiteralPath (Join-Path $OutputDirectory 'telemetry.jsonl')
if ($scenario -eq 'missing-baseline') { Start-Sleep -Milliseconds 500; throw 'Expected missing baseline' }
$deadline = [DateTime]::UtcNow.AddSeconds(10)
while (-not (Test-Path -LiteralPath (Join-Path $fixtureRoot 'started'))) {
    if ([DateTime]::UtcNow -gt $deadline) { throw 'Mock controller never started' }
    Start-Sleep -Milliseconds 50
}
if ($scenario -eq 'worker-failure') { throw 'Expected worker failure' }
if ($scenario -eq 'controller-failure') { Start-Sleep -Seconds 5 }
'{"clean_run":true}' | Set-Content -LiteralPath (Join-Path $OutputDirectory 'telemetry.summary.json')
while (-not (Test-Path -LiteralPath (Join-Path $fixtureRoot 'stopped-after-start'))) {
    if ([DateTime]::UtcNow -gt $deadline) { throw 'Controller not stopped before plotting' }
    Start-Sleep -Milliseconds 50
}
'MOCK GRAPH ONLY' | Set-Content -LiteralPath (Join-Path $OutputDirectory 'telemetry.lap.png')
'Mock recorder completed'
'@

function global:docker {
    $command = $args -join ' '
    Add-Content -LiteralPath (Join-Path $global:responseTestRoot 'calls') -Value $command
    $global:LASTEXITCODE = 0
    if ($command -match ' ps ') { 'container-id' }
    elseif ($command -match ' start avlite$| run --no-deps -d .*--response-speed') {
        if ($global:responseScenario -eq 'start-failure') { $global:LASTEXITCODE = 1; return }
        'started' | Set-Content -LiteralPath (Join-Path $global:responseTestRoot 'started')
    } elseif ($command -match ' stop avlite$|^stop --time 5 autodrive-response-') {
        if (Test-Path -LiteralPath (Join-Path $global:responseTestRoot 'started')) {
            'stopped' | Set-Content -LiteralPath (Join-Path $global:responseTestRoot 'stopped-after-start')
        }
    } elseif ($command -match '^inspect ') {
        if ($global:responseScenario -eq 'controller-failure') { 'false' } else { 'true' }
    }
}
function global:Read-Host { '' }

try {
    foreach ($scenario in @('normal-success', 'response-success', 'worker-failure', 'start-failure', 'controller-failure', 'missing-baseline')) {
        $global:responseScenario = $scenario
        $global:responseTestRoot = Join-Path $testRoot $scenario
        New-Item -ItemType Directory -Path $global:responseTestRoot | Out-Null
        $fixtureScripts = Join-Path $global:responseTestRoot 'scripts/windows'
        New-Item -ItemType Directory -Path $fixtureScripts -Force | Out-Null
        foreach ($script in @('laps.ps1', 'common.ps1')) {
            Copy-Item -LiteralPath (Join-Path $projectRoot "scripts/windows/$script") -Destination $fixtureScripts
        }
        $stub | Set-Content -LiteralPath (Join-Path $fixtureScripts 'record.ps1')
        'param($RecordingDirectory, [switch]$NoOpen)' |
            Set-Content -LiteralPath (Join-Path $fixtureScripts 'response.ps1')
        $scenario | Set-Content -LiteralPath (Join-Path $global:responseTestRoot 'scenario')
        $options = @{ MaxSeconds = 10; WaitForOdomSeconds = 1; Laps = 4; NoOpen = $true }
        if ($scenario -ne 'normal-success') { $options.ResponseSpeedMps = 1.5; $options.ResponseTrials = 3 }
        $failed = $false
        try { & (Join-Path $fixtureScripts 'laps.ps1') @options }
        catch { $failed = $true; "Expected failure in ${scenario}: $_" }
        if (($scenario -in @('normal-success', 'response-success')) -eq $failed) { throw "Wrong result: $scenario" }
        $calls = @(Get-Content -LiteralPath (Join-Path $global:responseTestRoot 'calls'))
        if ($scenario -eq 'response-success') {
            if (-not ($calls | Where-Object { $_ -match '--response-speed-mps 1.5 --response-trials 3' })) { throw 'Missing runtime arguments' }
            if ($calls | Where-Object { $_ -match ' start avlite$' }) { throw 'Started normal controller beside experiment' }
        }
        if ($scenario -eq 'missing-baseline' -and ($calls | Where-Object { $_ -match ' run --no-deps -d ' })) { throw 'Started without baseline' }
        if (-not ($calls | Where-Object { $_ -match '^rm --force autodrive-lap-' })) { throw 'Recorder cleanup missing' }
        if ($scenario -ne 'normal-success' -and -not ($calls | Where-Object { $_ -match '^rm --force autodrive-response-' })) { throw 'Test controller cleanup missing' }
        if (@(Get-Job).Count -ne 0) { throw 'Leaked background job' }
        "PASS: $scenario"
    }
} finally {
    Remove-Item Function:\docker -ErrorAction SilentlyContinue
    Remove-Item Function:\Read-Host -ErrorAction SilentlyContinue
}
