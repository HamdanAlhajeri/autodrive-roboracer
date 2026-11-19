<#
.SYNOPSIS
Create and build a custom track scene in an existing AutoDRIVE Unity source checkout.
.DESCRIPTION
Copies this repository's track tools and assets into ProjectPath, then invokes
Unity in batch mode. Track is sketch (default) or iros2026. SceneOnly updates
the scene; BuildOnly builds an existing scene. With neither switch, both stages
run. Logs and the Windows player are saved under log; the active AVLite track
profile is not selected by this script.
#>
param(
    [ValidateSet('sketch', 'iros2026')]
    [string]$Track = 'sketch',
    [string]$ProjectPath = (Join-Path $env:USERPROFILE 'AutoDRIVE-Unity'),
    [string]$UnityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.52f1\Editor\Unity.exe',
    [switch]$SceneOnly,
    [switch]$BuildOnly
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
if ($SceneOnly -and $BuildOnly) { throw 'Choose either SceneOnly or BuildOnly.' }
$custom = Get-CustomTrack $Track
$ProjectPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($ProjectPath)
if (-not (Test-Path -LiteralPath $UnityPath -PathType Leaf)) {
    throw "Unity Editor not found: $UnityPath"
}
if (-not (Test-Path -LiteralPath (Join-Path $ProjectPath 'Assets\Scenes\RoboRacer - Sim Racing.unity'))) {
    throw 'ProjectPath must be the AutoDRIVE-Simulator source checkout with its racing scene and dependencies.'
}
$versionFile = Join-Path $ProjectPath 'ProjectSettings\ProjectVersion.txt'
if (-not (Select-String -LiteralPath $versionFile -Pattern '^m_EditorVersion: 2022\.3\.52f1$' -Quiet)) {
    throw 'This importer was prepared for AutoDRIVE with Unity 2022.3.52f1.'
}
$editorTools = Join-Path $ProjectPath 'Assets\Editor\AutoDRIVEIntegration'
$runtimeTools = Join-Path $ProjectPath 'Assets\AutoDRIVEIntegration'
$trackAssets = Join-Path $ProjectPath ('Assets\Tracks\' + $custom.UnityFolder)
foreach ($directory in @($editorTools, $runtimeTools, $trackAssets)) {
    New-Item -ItemType Directory -Path $directory -Force | Out-Null
}
Copy-Item -LiteralPath "$ProjectRoot\tools\unity\Editor\SketchTrackBuilder.cs" -Destination $editorTools
Copy-Item -LiteralPath "$ProjectRoot\tools\unity\Runtime\SketchTrackSmokeCheck.cs" -Destination $runtimeTools
foreach ($trackFile in @("$($custom.Mesh).obj", "$($custom.Mesh).mtl", 'track_metadata.json', $custom.Preview)) {
    Copy-Item -LiteralPath (Join-Path (Join-Path $ProjectRoot $custom.Assets) $trackFile) -Destination $trackAssets
}
if (-not (Test-Path -LiteralPath "$ProjectRoot\config\maps\$($custom.Map).plan.json")) {
    throw "Run .\avlite.ps1 sketch-map -Track $Track to generate and validate the AVLite plan first."
}
$buildLogs = Join-Path $ProjectRoot 'log\unity-build'
New-Item -ItemType Directory -Path $buildLogs -Force | Out-Null
$playerPath = Join-Path $ProjectRoot $custom.Player

function Invoke-UnityStage([string]$Method, [string]$LogName) {
    <#
    .SYNOPSIS
    Run one SketchTrackBuilder method and wait for Unity Editor to finish.
    .DESCRIPTION
    Method selects the C# entry point, and LogName selects a file under the build
    log directory. Repository and output paths are passed explicitly so Unity
    reads the intended track. A failed process prints the log tail and raises an
    error instead of reporting an incomplete scene or player as successful.
    #>
    $stageLog = Join-Path $buildLogs $LogName
    $unityArguments = @('-batchmode', '-nographics', '-quit', '-projectPath', ('"' + $ProjectPath + '"'),
        '-executeMethod', ('AutoDriveTrackTools.SketchTrackBuilder.' + $Method),
        '-sketchSource', ('"' + $ProjectRoot + '"'), '-trackId', $Track,
        '-sketchBuild', ('"' + $playerPath + '"'), '-logFile', ('"' + $stageLog + '"'))
    Write-Host "Unity $Method running. Log: $stageLog"
    $unityProcess = Start-Process -FilePath $UnityPath -ArgumentList $unityArguments `
        -WorkingDirectory $ProjectPath -WindowStyle Hidden -PassThru
    # Wait for the Editor itself; compiler-server descendants can remain alive.
    $unityProcess.WaitForExit()
    if ($unityProcess.ExitCode -ne 0) {
        if (Test-Path -LiteralPath $stageLog) { Get-Content -LiteralPath $stageLog -Tail 45 }
        throw "Unity $Method failed (exit $($unityProcess.ExitCode)). See $stageLog"
    }
}

if ($SceneOnly) { Invoke-UnityStage 'CreateScene' 'create-scene.log' }
elseif ($BuildOnly) { Invoke-UnityStage 'BuildPlayer' 'build-player.log' }
else { Invoke-UnityStage 'CreateAndBuild' 'build-track.log' }
Write-Host "$($custom.Label) scene: Assets/Scenes/RoboRacer - $($custom.Label) Track.unity"
if (-not $SceneOnly) { Write-Host "$($custom.Label) simulator: $playerPath" }
Write-Host "Verify the simulator, then activate the matching profile using $($custom.Doc)."
