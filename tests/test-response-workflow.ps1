# Run in a fresh PowerShell process. Docker and jobs are mocked; no car is started.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$testRoot = Join-Path ([IO.Path]::GetTempPath()) ('response-workflow-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testRoot | Out-Null
$testScripts = Join-Path $testRoot 'scripts\windows'
New-Item -ItemType Directory -Path $testScripts -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $root 'avlite.ps1') -Destination $testRoot
foreach ($name in @('common.ps1', 'laps.ps1', 'response.ps1')) {
    Copy-Item -LiteralPath (Join-Path $root "scripts\windows\$name") -Destination $testScripts
}

function global:Read-Host { return '' }
function global:Start-Sleep { }
function global:Invoke-Item { }
function global:Start-Job {
    param($ArgumentList, $ScriptBlock)
    $global:options = $ArgumentList[1]
    $global:job = [pscustomobject]@{ State = 'Running' }
    New-Item -ItemType Directory -Path $global:options.OutputDirectory | Out-Null
    @{
        timestamp_utc = [DateTimeOffset]::UtcNow.ToString('o')
        x = 0; y = 0; speed = 0; lap_count = 0; collision_count = 0
        odom_age_s = 0; lap_count_age_s = 0; collision_count_age_s = 0
        throttle_command = 0; steering_command = 0
        throttle_command_age_s = 0; steering_command_age_s = 0
    } | ConvertTo-Json -Compress |
        Set-Content -LiteralPath (Join-Path $global:options.OutputDirectory 'telemetry.jsonl')
    Set-Content -LiteralPath (Join-Path $global:options.OutputDirectory 'telemetry.lap.png') -Value 'mock'
    return $global:job
}
function global:Receive-Job {
    param($Job, $ErrorAction)
    if ($global:started) {
        if ($global:scenario -eq 'recorder-failure') { throw 'simulated recorder failure' }
        $Job.State = 'Completed'
        '{}' | Set-Content -LiteralPath (Join-Path $global:options.OutputDirectory 'telemetry.summary.json')
    }
}
function global:Stop-Job { param($Job, $ErrorAction) $Job.State = 'Stopped' }
function global:Remove-Job { param($Job, [switch]$Force, $ErrorAction) }
function global:docker {
    $global:LASTEXITCODE = 0
    $call = $args -join ' '
    $global:calls.Add($call)
    if ($args[0] -eq 'compose') {
        if ($args -contains 'ps') { return 'stable-container' }
        if ($args -contains 'start') { $global:started = $true }
        if ($args -contains 'run') {
            $global:started = $true
            if ($global:scenario -eq 'partial-start') { $global:LASTEXITCODE = 1 }
            return 'response-container'
        }
    } elseif ($args[0] -eq 'inspect') {
        if ($global:scenario -eq 'controller-exit') { return 'false' }
        return 'true'
    } elseif ($args[0] -eq 'run') {
        '{"qualified":false,"accepted_trials":0,"required_trials":3}' |
            Set-Content -LiteralPath (Join-Path $global:options.OutputDirectory 'response-report.json')
    }
}

try {
    foreach ($global:scenario in @('normal-laps', 'success', 'auto-reset', 'partial-start', 'recorder-failure', 'controller-exit')) {
        $global:calls = [Collections.Generic.List[string]]::new()
        $global:started = $false
        $failed = $false
        try {
            if ($global:scenario -eq 'normal-laps') {
                & (Join-Path $testRoot 'avlite.ps1') laps -Laps 3 -Label team-test -NoOpen
            } else {
                & (Join-Path $testRoot 'avlite.ps1') response -TargetSpeedMps 1.5 -Trials 3 `
                    -NoOpen -ResetSimulator:($global:scenario -eq 'auto-reset')
            }
        } catch {
            $failed = $true
            Write-Output "Expected failure path: $_"
        }
        if ($failed -ne ($global:scenario -notin @('normal-laps', 'success', 'auto-reset'))) { throw "Wrong result: $global:scenario" }
        if ($global:scenario -eq 'normal-laps') {
            if ($global:options.Laps -ne 3 -or $global:options.ResponseData -or
                $global:options.Label -ne 'team-test' -or -not $global:options.StopOnIncident) {
                throw 'Normal lap capture options were not forwarded'
            }
            if (@($global:calls | Where-Object { $_ -match ' start avlite$' }).Count -ne 1 -or
                @($global:calls | Where-Object { $_ -match ' stop avlite$' }).Count -ne 2) {
                throw 'Normal lap controller was not started and stopped as expected'
            }
            Write-Output 'PASS: normal-laps'
            continue
        }
        if ($global:scenario -eq 'auto-reset' -and
            -not ($global:calls | Where-Object { $_ -match 'avlite_autodrive.simulator_reset' })) {
            throw 'Automatic simulator reset was not invoked'
        }
        if (-not ($global:calls | Where-Object { $_ -match '^stop --time 5 autodrive-response-' })) {
            throw "Response controller was not stopped: $global:scenario"
        }
        if (-not ($global:calls | Where-Object { $_ -match '^rm --force autodrive-response-' })) {
            throw "Response controller was not removed: $global:scenario"
        }
        if ($global:calls | Where-Object { $_ -match ' start avlite$' }) {
            throw 'Normal AVLite was started alongside the response controller'
        }
        if ($global:options.Laps -ne 4 -or -not $global:options.ResponseData) {
            throw 'Response capture/lap limit was not passed to recorder'
        }
        Write-Output "PASS: $global:scenario"
    }
} finally {
    # Only this test's resolved GUID directory may be removed.
    $resolved = [IO.Path]::GetFullPath($testRoot)
    $tempRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\') + '\'
    if ($resolved.StartsWith($tempRoot, [StringComparison]::OrdinalIgnoreCase) -and
        (Split-Path -Leaf $resolved) -match '^response-workflow-[0-9a-f]{32}$') {
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
}
