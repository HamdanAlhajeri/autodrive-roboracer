<#
.SYNOPSIS
Save timed screenshots of the AutoDRIVE simulator window for documentation.
.DESCRIPTION
Captures the simulator window every -IntervalSeconds into <Out>\<Label>-NNN.png until the
file <Out>\<Label>.stop appears or -MaxSeconds passes. It only reads the screen: it sends
no input to the simulator. The window must be visible (not minimized or covered).
.EXAMPLE
powershell -NoProfile -File tools\sim-localization\snap.ps1 -Out log\sim-localization\run\photos -Label avlite
#>
param(
    [Parameter(Mandatory)][string]$Out,
    [Parameter(Mandatory)][string]$Label,
    [double]$IntervalSeconds = 2.0,
    [int]$MaxSeconds = 600
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class SnapWin {
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
}
'@
[void][SnapWin]::SetProcessDPIAware()
New-Item -ItemType Directory -Path $Out -Force | Out-Null
$Out = (Resolve-Path -LiteralPath $Out).Path
$stop = Join-Path $Out "$Label.stop"
Remove-Item -LiteralPath $stop -Force -ErrorAction SilentlyContinue

$process = Get-Process -Name 'AutoDRIVE Simulator' -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $process -or $process.MainWindowHandle -eq [IntPtr]::Zero) { throw 'Simulator window not found.' }
$handle = $process.MainWindowHandle
if ([SnapWin]::IsIconic($handle)) { [void][SnapWin]::ShowWindow($handle, 9) }

$started = Get-Date
$index = 0
while (-not (Test-Path -LiteralPath $stop) -and ((Get-Date) - $started).TotalSeconds -lt $MaxSeconds) {
    $rect = New-Object SnapWin+RECT
    if ([SnapWin]::GetWindowRect($handle, [ref]$rect)) {
        $width, $height = ($rect.Right - $rect.Left), ($rect.Bottom - $rect.Top)
        if ($width -gt 100 -and $height -gt 100) {
            $bitmap = New-Object System.Drawing.Bitmap $width, $height
            $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
            # PrintWindow asks the window itself for its pixels (flag 2 includes DirectX
            # content), so another window on top cannot appear in the picture.
            $hdc = $graphics.GetHdc()
            $ok = [SnapWin]::PrintWindow($handle, $hdc, 2)
            $graphics.ReleaseHdc($hdc)
            if (-not $ok) { throw 'PrintWindow failed; the simulator window cannot be captured.' }
            $name = '{0}-{1:D3}.png' -f $Label, $index
            $bitmap.Save((Join-Path $Out $name), [System.Drawing.Imaging.ImageFormat]::Png)
            $graphics.Dispose(); $bitmap.Dispose()
            $index++
        }
    }
    Start-Sleep -Milliseconds ([int]($IntervalSeconds * 1000))
}
Write-Output "Saved $index screenshots to $Out"
