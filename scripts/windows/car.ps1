<#
.SYNOPSIS
Send status, map-start, race-start or stop to the Jetson supervisor over SSH.
.DESCRIPTION
Runs the supervisor CLI on the Jetson through key-authenticated SSH; no control port
is exposed. map-start and race-start keep the connection open and send a heartbeat
line every 0.2 s. The Jetson stops the car when heartbeats stop, so closing this
window, losing Wi-Fi or a laptop crash all end the authorization. Ctrl+C sends stop
immediately. stop is always accepted and may be repeated.
Session is the value shown by status; pass it so a delayed command cannot act on a
later session. SSH loss is not a physical stop: keep the car's emergency stop ready.
#>
param(
    [Parameter(Mandatory)][ValidateSet('status', 'map-start', 'race-start', 'stop')]
    [string]$Action,
    [Parameter(Mandatory)][ValidatePattern('^[A-Za-z0-9._@-]+$')][string]$JetsonHost,
    [ValidatePattern('^[A-Za-z0-9-]*$')][string]$Session = '',
    [ValidatePattern('^[A-Za-z0-9_./~-]+$')]
    [string]$RemoteCommand = '~/autodrive-roboracer/scripts/jetson/roboracer'
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$sshOptions = @('-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
    '-o', 'ServerAliveInterval=1', '-o', 'ServerAliveCountMax=2')

if ($Action -in @('status', 'stop')) {
    & ssh @sshOptions $JetsonHost $RemoteCommand $Action
    if ($LASTEXITCODE -ne 0) { throw "Jetson $Action failed (exit $LASTEXITCODE). Check SSH access and that the supervisor is running." }
    return
}

$remote = @($RemoteCommand, $Action, '--heartbeat-stdin')
if ($Session) { $remote += @('--session', $Session) }
$start = [Diagnostics.ProcessStartInfo]::new('ssh')
$start.Arguments = (@($sshOptions) + @('-T', $JetsonHost) + $remote) -join ' '
$start.UseShellExecute = $false
$start.RedirectStandardInput = $true
$process = [Diagnostics.Process]::Start($start)
try {
    while (-not $process.HasExited) {
        $process.StandardInput.WriteLine('hb')
        $process.StandardInput.Flush()
        Start-Sleep -Milliseconds 200
    }
} finally {
    if (-not $process.HasExited) {
        # Closing stdin makes the Jetson client send stop; the explicit stop covers a
        # half-open connection where that end-of-input never arrives.
        try { $process.StandardInput.Close() } catch { }
        & ssh @sshOptions $JetsonHost $RemoteCommand 'stop'
        $process.WaitForExit(3000) | Out-Null
    }
}
if ($process.ExitCode -notin @(0, 130)) {
    throw "$Action ended with exit code $($process.ExitCode); run status to see why."
}
