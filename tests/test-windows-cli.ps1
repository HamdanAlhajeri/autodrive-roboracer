# Run in a fresh PowerShell process. No Docker daemon, Unity or simulator is used.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$testRoot = Join-Path ([IO.Path]::GetTempPath()) ('avlite-cli-' + [Guid]::NewGuid().ToString('N'))
$fixture = Join-Path $testRoot 'checkout with spaces'
$outside = Join-Path $testRoot 'another working directory'
$global:calls = [Collections.Generic.List[object]]::new()
$global:player = $null
$global:launch = $null
$global:dockerFailure = $false
$global:downloadFixture = $null
$global:downloadCount = 0

function Assert([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw $Message }
}
function Expect-Failure([scriptblock]$Operation, [string]$Message) {
    $failed = $false
    try { & $Operation | Out-Null } catch { $failed = $true }
    Assert $failed $Message
}
function global:Start-Sleep { }
function global:Get-Process { param($Name, $ErrorAction) return $global:player }
function global:Start-Process {
    param($FilePath, $WorkingDirectory, $WindowStyle, $ArgumentList, [switch]$PassThru)
    $global:launch = @($FilePath, $WorkingDirectory, $WindowStyle, $ArgumentList)
    $global:player = [pscustomobject]@{ Id = 1234; Path = $FilePath; Closed = $false }
    $global:player | Add-Member -MemberType ScriptMethod -Name CloseMainWindow -Value { $this.Closed = $true }
    return $global:player
}
function global:git { return 'test-fixture' }
function global:curl.exe {
    # Exercise real ZIP extraction with a local fixture, without network access.
    $destination = $args[[Array]::IndexOf($args, '--output') + 1]
    Copy-Item -LiteralPath $global:downloadFixture -Destination $destination
    $global:downloadCount++
    $global:LASTEXITCODE = 0
}
function global:docker {
    $global:calls.Add(@($args))
    $global:LASTEXITCODE = if ($global:dockerFailure) { 1 } else { 0 }
    if ($global:dockerFailure) { return }
    if ($args[0] -eq 'info') { return 'linux' }
    if ($args[0] -eq 'compose' -and $args -contains 'ps') { return 'fixture-bridge' }
}

try {
    foreach ($directory in @($fixture, $outside, (Join-Path $fixture 'scripts\windows'),
            (Join-Path $fixture 'config\maps'), (Join-Path $fixture 'log\windows\sketch'))) {
        New-Item -ItemType Directory -Path $directory -Force | Out-Null
    }
    Copy-Item -LiteralPath (Join-Path $root 'avlite.ps1'), (Join-Path $root 'docker-compose.avlite.yml'),
        (Join-Path $root 'docker-compose.windows.yml') -Destination $fixture
    Get-ChildItem -LiteralPath (Join-Path $root 'scripts\windows') -Filter '*.ps1' |
        Copy-Item -Destination (Join-Path $fixture 'scripts\windows')
    '{"executable":"log/windows/sketch/AutoDRIVE Simulator.exe"}' |
        Set-Content -LiteralPath (Join-Path $fixture 'config\windows-simulator.json')
    'speed_mps: 2.5' | Set-Content -LiteralPath (Join-Path $fixture 'config\driving.yaml')
    $playerPath = Join-Path $fixture 'log\windows\sketch\AutoDRIVE Simulator.exe'
    '' | Set-Content -LiteralPath $playerPath
    $cli = Join-Path $fixture 'avlite.ps1'
    Push-Location -LiteralPath $outside
    try {
        # Help must not run a command or prompt for mandatory map parameters.
        & $cli help | Out-Null
        foreach ($command in @('start', 'stop', 'restart', 'status', 'logs', 'laps', 'record',
                'response', 'speed', 'map', 'sketch-map', 'sketch-build', 'sketch-test', 'test')) {
            & $cli $command -Help | Out-Null
        }
        Assert ($global:calls.Count -eq 0) 'Help invoked Docker'
        Expect-Failure { & $cli laps -Laps 0 } 'Lap range validation was lost'
        Expect-Failure { & $cli laps -Label 'invalid;label' } 'Label validation was lost'
        Expect-Failure { & $cli speed -UnknownOption 1 } 'Unknown options were accepted'
        Expect-Failure { & $cli response -TargetSpeedMps 10 } 'Response-controller speed validation was lost'
        Assert ($global:calls.Count -eq 0) 'Invalid options caused side effects'
        Write-Output 'PASS: command help and parameter validation'

        & $cli start | Out-Null
        Assert ($global:launch[0] -eq $playerPath) 'Simulator selection was resolved from the caller directory'
        Assert ($global:launch[2] -eq 'Normal') 'Interactive simulator was not opened visibly'
        Assert (Test-Path -LiteralPath (Join-Path $fixture 'log\windows\simulator.pid')) 'PID written outside the repository'
        $global:calls.Clear()
        & $cli restart | Out-Null
        $operations = @($global:calls | ForEach-Object { ($_ | Select-Object -Skip 5) -join ' ' })
        Assert (($operations -join '|') -eq 'stop avlite|restart actuator|start avlite') 'Incorrect reload order'
        & $cli logs -Follow -Tail 17 | Out-Null
        $logCall = $global:calls[$global:calls.Count - 1]
        Assert ($logCall -contains '--follow' -and $logCall -contains 17) 'Log options were not forwarded'
        & $cli stop | Out-Null
        Assert $global:player.Closed 'Selected simulator was not asked to close'
        foreach ($call in $global:calls) {
            Assert ($call[1] -eq '-f' -and $call[2] -eq (Join-Path $fixture 'docker-compose.avlite.yml')) 'Compose base path drifted'
            Assert ($call[4] -eq (Join-Path $fixture 'docker-compose.windows.yml')) 'Windows Compose path drifted'
        }
        $global:dockerFailure = $true
        Expect-Failure { & $cli status } 'Docker errors were hidden'
        $global:dockerFailure = $false
        Write-Output 'PASS: simulator selection, reload order, Compose paths and errors'

        # A successful install keeps the extracted runtime, not a duplicate ZIP.
        Remove-Item -LiteralPath (Join-Path $fixture 'config\windows-simulator.json')
        $global:player = $null
        $downloadSource = Join-Path $testRoot 'download source\autodrive_simulator'
        New-Item -ItemType Directory -Path $downloadSource -Force | Out-Null
        'fixture executable' | Set-Content -LiteralPath (Join-Path $downloadSource 'AutoDRIVE Simulator.exe')
        $global:downloadFixture = Join-Path $testRoot 'practice.zip'
        Compress-Archive -LiteralPath $downloadSource -DestinationPath $global:downloadFixture
        $practiceExe = Join-Path $fixture 'log\windows\practice\autodrive_simulator\AutoDRIVE Simulator.exe'
        $downloadZip = Join-Path $fixture 'log\windows\autodrive_simulator_practice_windows.zip'
        & $cli start | Out-Null
        Assert ((Get-Content -LiteralPath $practiceExe -Raw).Trim() -eq 'fixture executable') 'Downloaded runtime was not extracted intact'
        Assert (-not (Test-Path -LiteralPath $downloadZip)) 'Successful download left a duplicate archive'
        Assert ($global:launch[0] -eq $practiceExe) 'Default launcher did not select the extracted practice runtime'
        & $cli start | Out-Null
        Assert ($global:downloadCount -eq 1) 'Existing practice runtime was downloaded again'

        # An unexpected archive must stop startup and remain available to inspect.
        Remove-Item -LiteralPath $practiceExe
        $global:player = $null
        $unexpectedFile = Join-Path $testRoot 'unexpected.txt'
        'Missing simulator executable' | Set-Content -LiteralPath $unexpectedFile
        $global:downloadFixture = Join-Path $testRoot 'unexpected.zip'
        Compress-Archive -LiteralPath $unexpectedFile -DestinationPath $global:downloadFixture
        $global:calls.Clear()
        Expect-Failure { & $cli start } 'Unexpected archive contents did not stop startup'
        Assert (Test-Path -LiteralPath $downloadZip) 'Failed installation removed its download'
        Assert (@($global:calls | Where-Object { $_ -contains 'up' }).Count -eq 0) 'Failed extraction started the driving services'
        Write-Output 'PASS: practice download cleanup, runtime reuse and failed-install preservation'

        $global:calls.Clear()
        & $cli speed -TargetSpeedMps 7.5
        $speedCall = $global:calls[0]
        Assert ($speedCall -contains '7.5') 'Numeric speed option was lost'
        Assert ($speedCall -contains "${fixture}/config:/config:ro") 'Config mount is not rooted in the checkout'
        Assert ($speedCall -contains "${fixture}/src/avlite_autodrive:/opt/integration:ro") 'Source mount is not rooted in the checkout'

        & $cli record -Seconds 1 -Label cli-test -OutputDirectory '.\captured run'
        $capture = Join-Path $outside 'captured run'
        Assert (Test-Path -LiteralPath (Join-Path $capture 'run.json')) 'Explicit recording output is not relative to the caller'
        Assert (Test-Path -LiteralPath (Join-Path $capture 'config\driving.yaml')) 'Recording config snapshot is missing'
        $recordCall = $global:calls | Where-Object { $_ -contains "${capture}:/records" } | Select-Object -First 1
        Assert ($null -ne $recordCall) 'Recording mount with spaces was split or lost'
        '' | Set-Content -LiteralPath (Join-Path $capture 'telemetry.jsonl')
        '{}' | Set-Content -LiteralPath (Join-Path $capture 'telemetry.track.json')
        & $cli response -RecordingDirectory '.\captured run' -CoastOnly
        $analysisCall = $global:calls[$global:calls.Count - 1]
        Assert ($analysisCall -contains 'avlite_autodrive.braking_analysis') 'Coasting analysis was not dispatched'
        Expect-Failure { & $cli response -CoastOnly } 'CoastOnly accepted missing recording'
        & $cli map -RecordingDirectory '.\captured run' -Name team-track
        $mapCall = $global:calls[$global:calls.Count - 1]
        Assert ($mapCall -contains '/maps/team-track.json') 'Map name was not forwarded'
        '{}' | Set-Content -LiteralPath (Join-Path $fixture 'config\maps\team-track.json')
        Expect-Failure { & $cli map -RecordingDirectory '.\captured run' -Name team-track } 'Existing maps can be overwritten'
        & $cli sketch-map
        Assert ($global:calls[$global:calls.Count - 1] -contains "${fixture}/assets/tracks/sketch_track:/track:ro") 'Sketch assets mount drifted'
        Write-Output 'PASS: offline tools, recording paths with spaces and overwrite protection'
    } finally { Pop-Location }
} finally {
    $resolved = [IO.Path]::GetFullPath($testRoot)
    $tempRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\') + '\'
    if ($resolved.StartsWith($tempRoot, [StringComparison]::OrdinalIgnoreCase) -and
        (Split-Path -Leaf $resolved) -match '^avlite-cli-[0-9a-f]{32}$') {
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
}
