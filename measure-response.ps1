<#
.SYNOPSIS
Collect guided straight throttle-off trials, or analyze an existing recording.
.DESCRIPTION
Uses planned steering on the configured map, with a temporary 1.0-2.5 m/s
ceiling. The simulator must already be connected. Prompts for a reset, records
before driving, and stops the test controller on completion or failure.
Three trials use four laps to allow an extra opportunity to reach the straight.
Neither this script nor its analyzer changes saved driving settings.
.EXAMPLE
.\measure-response.ps1 -TargetSpeedMps 1.5 -Trials 3
.EXAMPLE
.\measure-response.ps1 -RecordingDirectory .\log\recordings\YOUR-RUN
#>
[CmdletBinding(DefaultParameterSetName = 'Run')]
param(
    [Parameter(ParameterSetName = 'Run')]
    [ValidateRange(1.0, 2.5)][double]$TargetSpeedMps = 1.5,
    [Parameter(ParameterSetName = 'Run')]
    [ValidateRange(3, 20)][int]$Trials = 3,
    [Parameter(ParameterSetName = 'Run')]
    [ValidateRange(30, 86400)][int]$MaxSeconds = 600,
    [Parameter(Mandatory = $true, ParameterSetName = 'Analyze')]
    [string]$RecordingDirectory,
    [switch]$NoOpen
)

$ErrorActionPreference = 'Stop'
# Compatibility entry point for commands saved before the Windows CLI migration.
& (Join-Path $PSScriptRoot 'scripts/windows/response.ps1') @PSBoundParameters
