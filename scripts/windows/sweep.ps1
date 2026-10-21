<#
.SYNOPSIS
Run the speed-tuning test matrix and summarize every run in one table.
.DESCRIPTION
Stages run in order: A speed-loop gain, B cornering grip, C braking, D acceleration.
Each test edits config/avlite.yaml, validates the plan offline, then records laps
after a simulator reset. Stages B-D escalate only while tests pass. The fastest
passing value (lowest rolling lap time) carries into later stages. A failed test is
retried once by default, because a simulator stall can also end a run.
The original avlite.yaml is restored afterwards, even when a run fails. -DryRun
performs only the offline plan checks and never drives. Results are saved to
log/sweeps/<run>/results.csv and results.md.
.EXAMPLE
.\avlite.ps1 sweep -DryRun
.EXAMPLE
.\avlite.ps1 sweep -Stages B,C -Laps 3
#>
param(
    [ValidateSet('A', 'B', 'C', 'D')][string[]]$Stages = @('A', 'B', 'C', 'D'),
    [ValidateRange(2, 20)][int]$Laps = 3,
    [ValidateRange(0.01, 1.0)][double]$MaxPathDeviationM = 0.10,
    [ValidateRange(0, 3)][int]$Retries = 1,
    [switch]$ManualReset,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$culture = [Globalization.CultureInfo]::InvariantCulture

# Each test sets 'section.key' values in avlite.yaml. Acceleration and braking also
# move the matching controller limit, because the planner rejects a mismatch.
$matrix = [ordered]@{
    A = @{ Name = 'speed-loop gain'; Escalate = $false; Tests = @(
        @{ Id = 'A-valpha2'; Set = [ordered]@{ 'c30_control.c35_valpha' = 2.0 } },
        @{ Id = 'A-valpha3'; Set = [ordered]@{ 'c30_control.c35_valpha' = 3.0 } },
        @{ Id = 'A-valpha4'; Set = [ordered]@{ 'c30_control.c35_valpha' = 4.0 } }) }
    B = @{ Name = 'cornering grip'; Escalate = $true; Tests = @(
        foreach ($lat in 3.5, 4.0, 4.5, 5.0) {
            @{ Id = 'B-lat' + ($lat.ToString('0.0', $culture) -replace '\.', 'p')
               Set = [ordered]@{ 'planning.lateral_acceleration_mps2' = $lat } }
        }) }
    C = @{ Name = 'braking'; Escalate = $true; Tests = @(
        @{ Id = 'C-brk3p5'; Set = [ordered]@{ 'planning.braking_deceleration_mps2' = 3.5
                                              'c30_control.c32_ego_min_acceleration' = -3.5 } }) }
    D = @{ Name = 'acceleration'; Escalate = $true; Tests = @(
        foreach ($acc in 2.5, 3.0) {
            @{ Id = 'D-acc' + ($acc.ToString('0.0', $culture) -replace '\.', 'p')
               Set = [ordered]@{ 'planning.acceleration_mps2' = $acc
                                 'c30_control.c32_ego_max_acceleration' = $acc } }
        }) }
}

function Set-YamlValue([string]$Text, [string]$Name, [double]$Value) {
    <#
    .SYNOPSIS
    Replace one value inside one top-level YAML section, keeping comments intact.
    .DESCRIPTION
    Name is 'section.key'. The section ends at the next line starting in column 0,
    so equally named keys in other sections are untouched. Exactly one match is
    required; anything else throws instead of guessing.
    #>
    $section, $key = $Name.Split('.', 2)
    $header = [regex]::Match($Text, "(?m)^$([regex]::Escape($section)):[^\r\n]*\r?\n")
    if (-not $header.Success) { throw "Section '$section' not found in avlite.yaml" }
    $start = $header.Index + $header.Length
    $next = [regex]::Match($Text.Substring($start), '(?m)^\S')
    $end = if ($next.Success) { $start + $next.Index } else { $Text.Length }
    $body = $Text.Substring($start, $end - $start)
    $pattern = "(?m)^([ \t]+$([regex]::Escape($key)):[ \t]*)[^\s#]+"
    if ([regex]::Matches($body, $pattern).Count -ne 1) { throw "Expected one '$Name' in avlite.yaml" }
    $formatted = $Value.ToString('0.0##', $culture)
    $body = [regex]::Replace($body, $pattern, { param($m) $m.Groups[1].Value + $formatted })
    return $Text.Substring(0, $start) + $body + $Text.Substring($end)
}

function ConvertTo-Number($Value) {
    <# .SYNOPSIS Parse an invariant-culture number; return $null for blanks or text. #>
    $number = 0.0
    if ([double]::TryParse([string]$Value, [Globalization.NumberStyles]::Float, $culture, [ref]$number)) {
        return $number
    }
    return $null
}

function Get-RunMetrics([string]$Directory) {
    <#
    .SYNOPSIS
    Summarize one lap recording: counters, average/rolling speed, peak and path error.
    .DESCRIPTION
    Average speed is total distance over total time, including the standing start.
    Rolling speed covers only laps after the first lap-counter increase.
    #>
    $summary = Get-Content -LiteralPath (Join-Path $Directory 'telemetry.summary.json') -Raw | ConvertFrom-Json
    $points = New-Object System.Collections.Generic.List[object]
    $deviation = New-Object System.Collections.Generic.List[double]
    foreach ($row in Import-Csv -LiteralPath (Join-Path $Directory 'telemetry.csv')) {
        $t, $d, $lap = (ConvertTo-Number $row.elapsed_s), (ConvertTo-Number $row.distance_m), (ConvertTo-Number $row.lap_count)
        if ($null -ne $t -and $null -ne $d -and $null -ne $lap) { $points.Add(@($t, $d, $lap)) }
        $speed, $offset = (ConvertTo-Number $row.speed), (ConvertTo-Number $row.path_deviation_m)
        if ($null -ne $speed -and $speed -gt 0.5 -and $null -ne $offset) { $deviation.Add([Math]::Abs($offset)) }
    }
    $marks = @(for ($i = 1; $i -lt $points.Count; $i++) { if ($points[$i][2] -gt $points[$i - 1][2]) { $i } })
    $rollingLap = $null; $rollingSpeed = $null
    if ($marks.Count -ge 2) {
        $a, $b = $points[$marks[0]], $points[$marks[-1]]
        $count = $b[2] - $a[2]
        $rollingLap = ($b[0] - $a[0]) / $count
        $rollingSpeed = ($b[1] - $a[1]) / ($b[0] - $a[0])
    }
    $p95 = $null
    if ($deviation.Count) {
        $sorted = @($deviation | Sort-Object)
        $p95 = $sorted[[Math]::Min($sorted.Count - 1, [int][Math]::Floor(0.95 * $sorted.Count))]
    }
    [pscustomobject]@{
        Laps = [int]$summary.completed_laps
        Collisions = [int]$summary.final.collision_count
        Resets = [int]$summary.resets
        Incident = [bool]$summary.incident_detected
        AvgMps = if ($summary.elapsed_s) { $summary.distance_m / $summary.elapsed_s } else { $null }
        RollingMps = $rollingSpeed
        RollingLapS = $rollingLap
        PeakMps = $summary.max_speed_mps
        DevP95M = $p95
    }
}

function Format-Value($Value, [string]$Format) {
    if ($null -eq $Value) { return '-' }
    return ([double]$Value).ToString($Format, $culture)
}

$configPath = Join-Path $ProjectRoot 'config\avlite.yaml'
$original = [IO.File]::ReadAllText($configPath)
$utf8 = New-Object System.Text.UTF8Encoding $false
$sweepDir = Join-Path $ProjectRoot ('log\sweeps\' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $sweepDir -Force | Out-Null
[IO.File]::WriteAllText((Join-Path $sweepDir 'avlite.yaml.orig'), $original, $utf8)
if (-not $DryRun) {
    if (-not (Invoke-AvliteCompose ps -q --status running bridge)) {
        throw 'Run .\avlite.ps1 start first and connect the simulator, then retry this script.'
    }
}

$carried = [ordered]@{}   # Best passing settings so far, applied on top of avlite.yaml.
$bestLap = $null          # Rolling lap time of the carried settings, once measured.
$results = New-Object System.Collections.Generic.List[object]
try {
    foreach ($stageKey in $matrix.Keys) {
        if ($Stages -notcontains $stageKey) { continue }
        $stage = $matrix[$stageKey]
        Write-Host "`n=== Stage ${stageKey}: $($stage.Name) ===" -ForegroundColor Cyan
        $stageBest = $null
        foreach ($test in $stage.Tests) {
            $settings = [ordered]@{}
            foreach ($k in $carried.Keys) { $settings[$k] = $carried[$k] }
            foreach ($k in $test.Set.Keys) { $settings[$k] = $test.Set[$k] }
            $text = $original
            foreach ($k in $settings.Keys) { $text = Set-YamlValue $text $k $settings[$k] }
            [IO.File]::WriteAllText($configPath, $text, $utf8)
            $label = ($test.Set.GetEnumerator() | ForEach-Object {
                $_.Key.Split('.')[1] + '=' + $_.Value.ToString($culture) }) -join ' '
            $row = [ordered]@{ Stage = $stageKey; Test = $test.Id; Change = $label; PlanMaxMps = $null
                Result = $null; Laps = $null; Collisions = $null; Resets = $null; AvgMps = $null
                RollingMps = $null; RollingLapS = $null; PeakMps = $null; DevP95M = $null
                Note = ''; Recording = '' }
            Write-Host "`n--- $($test.Id): $label" -ForegroundColor Yellow
            try {
                $check = & (Join-Path $PSScriptRoot 'speed.ps1') | Out-String
                $planRange = [regex]::Match($check, 'Validated map profile: ([\d.]+) to ([\d.]+) m/s')
                if ($planRange.Success) { $row.PlanMaxMps = [double]::Parse($planRange.Groups[2].Value, $culture) }
            } catch {
                $row.Result = 'REJECTED'; $row.Note = "Offline plan check failed: $_"
                $results.Add([pscustomobject]$row)
                Write-Warning $row.Note
                if ($stage.Escalate) { break } else { continue }
            }
            if ($DryRun) {
                $row.Result = 'PLANNED'
                $results.Add([pscustomobject]$row)
                Write-Host ("Planned peak {0} m/s" -f (Format-Value $row.PlanMaxMps '0.00'))
                if ($stage.Escalate) { $stageBest = @{ Set = $test.Set; Lap = $null } }
                continue
            }
            for ($attempt = 0; $attempt -le $Retries; $attempt++) {
                $runDir = Join-Path $sweepDir ($test.Id + $(if ($attempt) { "-retry$attempt" } else { '' }))
                $lapOptions = @{ Laps = $Laps; Label = $test.Id; NoTrackMap = $true; NoOpen = $true
                                 OutputDirectory = $runDir; ResetSimulator = -not $ManualReset }
                try {
                    & (Join-Path $PSScriptRoot 'laps.ps1') @lapOptions | Out-Host
                } catch {
                    $row.Result = 'ERROR'; $row.Note = "Recording failed: $_"; $row.Recording = $runDir
                    $results.Add([pscustomobject]$row)
                    throw "Sweep stopped at $($test.Id). $($row.Note)"
                }
                $metrics = Get-RunMetrics $runDir
                foreach ($p in $metrics.PSObject.Properties) {
                    if (-not $row.Contains($p.Name)) { continue }
                    $row[$p.Name] = if ($p.Value -is [double] -or $p.Value -is [decimal]) {
                        [Math]::Round([double]$p.Value, 3) } else { $p.Value }
                }
                $row.Recording = $runDir
                $problems = @()
                if ($metrics.Laps -lt $Laps) { $problems += "$($metrics.Laps)/$Laps laps" }
                if ($metrics.Collisions) { $problems += "$($metrics.Collisions) collision(s)" }
                if ($metrics.Resets) { $problems += "$($metrics.Resets) reset(s)" }
                if ($null -ne $metrics.DevP95M -and $metrics.DevP95M -gt $MaxPathDeviationM) {
                    $problems += ('path deviation p95 {0} m' -f (Format-Value $metrics.DevP95M '0.000'))
                }
                $row.Result = if ($problems.Count) { 'FAIL' } else { 'PASS' }
                $row.Note = ($problems -join '; ')
                if ($attempt) { $row.Note = ("retry $attempt. " + $row.Note).Trim() }
                Write-Host ("{0}: rolling {1} m/s, lap {2} s, peak {3} m/s {4}" -f $row.Result,
                    (Format-Value $row.RollingMps '0.000'), (Format-Value $row.RollingLapS '0.00'),
                    (Format-Value $row.PeakMps '0.00'), $row.Note)
                if ($row.Result -eq 'PASS') { break }
            }
            $results.Add([pscustomobject]$row)
            if ($row.Result -ne 'PASS') {
                if ($stage.Escalate) { Write-Host 'Stopping this stage at the first failure.'; break }
                continue
            }
            if ($null -eq $stageBest -or ($null -ne $row.RollingLapS -and
                    ($null -eq $stageBest.Lap -or $row.RollingLapS -lt $stageBest.Lap))) {
                $stageBest = @{ Set = $test.Set; Lap = $row.RollingLapS }
            }
        }
        # Carry a stage's winner only when it beats the settings already carried.
        if ($stageBest -and ($null -eq $bestLap -or $null -eq $stageBest.Lap -or $stageBest.Lap -lt $bestLap)) {
            foreach ($k in $stageBest.Set.Keys) { $carried[$k] = $stageBest.Set[$k] }
            if ($null -ne $stageBest.Lap) { $bestLap = $stageBest.Lap }
        }
    }
} finally {
    [IO.File]::WriteAllText($configPath, $original, $utf8)
    Write-Host "`nRestored config\avlite.yaml to its original contents."
    if ($results.Count) {
        $results | Export-Csv -LiteralPath (Join-Path $sweepDir 'results.csv') -NoTypeInformation -Encoding UTF8
        $lines = @('# Speed sweep results', '',
            "Laps per test: $Laps. Pass: all laps, no collision/reset, path deviation p95 <= $MaxPathDeviationM m.", '',
            '| Test | Change | Plan peak | Result | Avg speed | Rolling speed | Rolling lap | Peak | Dev p95 | Note |',
            '|---|---|---|---|---|---|---|---|---|---|')
        foreach ($r in $results) {
            $lines += ('| {0} | {1} | {2} | {3} | {4} | {5} | {6} | {7} | {8} | {9} |' -f $r.Test, $r.Change,
                (Format-Value $r.PlanMaxMps '0.00'), $r.Result, (Format-Value $r.AvgMps '0.000'),
                (Format-Value $r.RollingMps '0.000'), (Format-Value $r.RollingLapS '0.00'),
                (Format-Value $r.PeakMps '0.00'), (Format-Value $r.DevP95M '0.000'), $r.Note)
        }
        $lines += '', '## Best settings found', ''
        if ($carried.Count) { $lines += @($carried.GetEnumerator() | ForEach-Object { "- ``$($_.Key)``: $($_.Value)" }) }
        else { $lines += '- No change beat the original configuration.' }
        [IO.File]::WriteAllLines((Join-Path $sweepDir 'results.md'), $lines, $utf8)
        $results | Format-Table Test, Result, PlanMaxMps, AvgMps, RollingMps, RollingLapS, PeakMps, DevP95M, Note -AutoSize | Out-Host
        Write-Host "Results: $(Join-Path $sweepDir 'results.md')"
        if ($carried.Count) {
            Write-Host 'Best settings (not applied; edit config\avlite.yaml to keep them):'
            $carried.GetEnumerator() | ForEach-Object { Write-Host "  $($_.Key) = $($_.Value)" }
        }
    }
}
