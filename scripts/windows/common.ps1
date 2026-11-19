# Shared repository paths and command registry. Dot-source after a script's param block.
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$compose = @('compose', '-f', (Join-Path $ProjectRoot 'docker-compose.avlite.yml'),
    '-f', (Join-Path $ProjectRoot 'docker-compose.windows.yml'))

function Invoke-AvliteCompose {
    <#
    .SYNOPSIS
    Run Docker Compose with this repository's shared and Windows configuration.
    .DESCRIPTION
    Extra arguments are forwarded unchanged to Docker, and its output is returned
    to the caller. A nonzero exit code becomes a PowerShell error so a failed
    container operation does not let the rest of the workflow continue.
    #>
    & docker @compose @args
    if ($LASTEXITCODE -ne 0) { throw "Docker Compose failed: $args. Check Docker Desktop and the output above." }
}

function Get-CustomTrack([string]$Track) {
    <#
    .SYNOPSIS
    Return the asset, map, profile and simulator paths of a custom Unity track.
    .DESCRIPTION
    sketch is the traced hand-drawn circuit; iros2026 replicates the official RoboRacer
    IROS 2026 map. sketch-map, sketch-build and sketch-test share this table so one
    -Track value selects matching files. Paths are relative to the repository root.
    #>
    $tracks = @{
        sketch = @{ Label = 'Sketch'; Assets = 'assets\tracks\sketch_track'; Mesh = 'sketch_track'
            Preview = 'sketch_track_preview.png'; Map = 'sketch'; Profile = 'avlite.sketch.yaml'
            UnityFolder = 'SketchTrack'; Player = 'log\windows\sketch\AutoDRIVE Simulator.exe'
            Doc = 'docs\sketch-track.md' }
        iros2026 = @{ Label = 'IROS 2026'; Assets = 'assets\tracks\iros2026'; Mesh = 'iros2026_track'
            Preview = 'iros2026_track_preview.png'; Map = 'iros2026'; Profile = 'avlite.iros2026.yaml'
            UnityFolder = 'IROS2026Track'; Player = 'log\windows\iros2026\AutoDRIVE Simulator.exe'
            Doc = 'docs\iros2026-track.md' }
    }
    if (-not $tracks.ContainsKey($Track)) { throw "Unknown custom track: $Track" }
    $tracks[$Track]
}

function Get-AvliteCommands {
    <#
    .SYNOPSIS
    Return the command names, implementation scripts and short help descriptions.
    .DESCRIPTION
    Preset contains arguments fixed by the command, such as Action=start. The
    root launcher uses this ordered table for both dispatch and help, so adding
    a command here keeps those two views consistent.
    #>
    [ordered]@{
        start = @{ Script = 'stack.ps1'; Preset = @{ Action = 'start' }; Description = 'Build/start containers and open the selected simulator' }
        stop = @{ Script = 'stack.ps1'; Preset = @{ Action = 'stop' }; Description = 'Stop driving, containers and the selected simulator' }
        restart = @{ Script = 'stack.ps1'; Preset = @{ Action = 'restart' }; Description = 'Reload AVLite and actuator settings; resumes driving' }
        status = @{ Script = 'stack.ps1'; Preset = @{ Action = 'status' }; Description = 'Show container status' }
        logs = @{ Script = 'stack.ps1'; Preset = @{ Action = 'logs' }; Description = 'Read controller and bridge logs (-Follow to stream)' }
        laps = @{ Script = 'laps.ps1'; Preset = @{}; Description = 'Record controlled laps; stop on completion, incident or timeout' }
        record = @{ Script = 'record.ps1'; Preset = @{}; Description = 'Capture telemetry without starting or stopping driving' }
        response = @{ Script = 'response.ps1'; Preset = @{}; Description = 'Measure response, replot a response run, or analyze ordinary coasting' }
        speed = @{ Script = 'speed.ps1'; Preset = @{}; Description = 'Inspect configured speed limits and plan without driving' }
        sweep = @{ Script = 'sweep.ps1'; Preset = @{}; Description = 'Run the speed-tuning test matrix and tabulate every run' }
        map = @{ Script = 'map.ps1'; Preset = @{}; Description = 'Build a race map from a clean recording' }
        'sketch-map' = @{ Script = 'sketch-map.ps1'; Preset = @{}; Description = 'Convert a custom track (-Track sketch|iros2026) to a validated map and profile' }
        'sketch-build' = @{ Script = 'sketch-build.ps1'; Preset = @{}; Description = 'Create/build a custom track scene (-Track) in an existing Unity source checkout' }
        'sketch-test' = @{ Script = 'sketch-test.ps1'; Preset = @{}; Description = 'Run native custom-track scene checks (-Track); no autonomous laps' }
        test = @{ Script = 'test.ps1'; Preset = @{}; Description = 'Run Windows command/workflow tests with Docker mocked' }
        live = @{ Script = 'live.ps1'; Preset = @{}; Description = 'Open the live view of estimated vs true car position' }
        car = @{ Script = 'car.ps1'; Preset = @{}; Description = 'Jetson car over SSH: status, map-start, race-start or stop' }
    }
}
