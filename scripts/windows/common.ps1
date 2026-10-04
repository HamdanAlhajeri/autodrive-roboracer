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
        map = @{ Script = 'map.ps1'; Preset = @{}; Description = 'Build a race map from a clean recording' }
        'sketch-map' = @{ Script = 'sketch-map.ps1'; Preset = @{}; Description = 'Convert supplied sketch assets to a validated map and profile' }
        'sketch-build' = @{ Script = 'sketch-build.ps1'; Preset = @{}; Description = 'Create/build the sketch track in an existing Unity source checkout' }
        'sketch-test' = @{ Script = 'sketch-test.ps1'; Preset = @{}; Description = 'Run native sketch scene checks; no autonomous laps' }
        test = @{ Script = 'test.ps1'; Preset = @{}; Description = 'Run Windows command/workflow tests with Docker mocked' }
    }
}
