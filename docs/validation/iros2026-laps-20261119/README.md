# IROS 2026 track: first simulator laps

19 November 2026. AutoDRIVE simulator built from the IROS 2026 scene, AVLite planned controller on its own LiDAR
localization (ICP, `reference: map_hits`, wheel-odometry prediction), run on the merged `main` (`f7afb1a`) plus the
Unity builder retry described below.

## Setup

| Item | Value |
| --- | --- |
| Simulator | Unity 2022.3.52f1, Windows 64-bit player built from `RoboRacer - IROS 2026 Track.unity` |
| Localization | AVLite `LidarLocalization` (ICP) from the track spawn `(-20.3274, -3.0653, -1.3095)`, 14,076 reference map points |
| Plan | 199.4 m, estimated 80.2 s, planned speed 1.69 to 2.50 m/s (a_lat 3.0, a_accel 1.0, a_brake 1.5 m/s^2) |
| Limits | `speed_mps: 2.5`, `max_throttle: 0.2` (first-lap limits; braking not calibrated on this scene) |
| Command | `.\avlite.ps1 laps -Laps 3 -Label iros2026-commissioning -ResetSimulator -NoOpen` |

## Scene check (before driving)

`.\avlite.ps1 sketch-test -Track iros2026 -Visible` passed: 4 grounded wheels, spawn error 0.6 mm, 1070 finite LiDAR
hits, two consecutive lap triggers, reset error 0.01 mm. See `scene-smoke-test.json` and `scene-render.png`.

## Result: three consecutive clean laps

| Measure | Value |
| --- | --- |
| Laps completed | 3 (clean-lap screening passed) |
| Collisions / resets | 0 / 0 |
| Lap times | 107.13 s, 105.60 s (first lap 113.07 s including start-up); last lap 105.34 s |
| Mean lap time (laps 2 and 3) | 106.4 s (std 0.77 s) |
| Distance | 599 m |
| Speed | mean 1.84 m/s, median 1.93 m/s, max 2.50 m/s (never above the 2.5 m/s ceiling) |
| Controller path deviation | mean 0.001 m, 95th percentile 0.024 m, max 0.129 m |
| Plan valid | 3,159 of 3,159 samples where reported |
| Speed limited for an invalid pose (`speed_limit_reason` 3) | 0 samples |

Figures: `laps.png` (trajectory and speed), `planned-driving.png`, `control.png`.

## Findings

1. **The car completes the IROS 2026 track on ICP localization with no collisions.** The trajectory stays on the
   racing line (`laps.png`).
2. **ICP lost the track five times in three laps** (`localization-recoveries.txt`, alignment 0.32 to 0.45). Each time
   the recovery search found a fit 1.4 to 1.6 m from the estimate and the car stopped briefly (speed drops to 0 at
   about 62, 165, 217, 272 and 323 s). It recovered every time and none caused a collision, but 1.4 to 1.6 m is
   larger than the track is wide in places. This is the main open problem for this track; the offline check
   predicted the worst stretch is the long final straight.
3. **Laps take 106 s against a planned 80 s.** Mean speed is 1.84 m/s against a 2.48 m/s mean target. The throttle
   ceiling (`max_throttle: 0.2`) and the recovery stops explain most of the gap. Do not raise speed until the
   recoveries are fixed.
4. **A measured `localization_error_m` is not in this recording.** The live pose view showed 0.22 m with a worst
   case of 0.28 m during a lap before the recording, but the recorded telemetry has no localization-error column, so
   no figure is claimed here. Adding it to the telemetry is needed before reporting it in the SDP.
5. The lap graph title reads "practice track" because the plotting script hard-codes it; the data is the IROS 2026
   track.

## Build notes (for repeating this)

- Needs Unity 2022.3.52f1 and the AutoDRIVE source checkout. The sparse checkout helper
  `tools/unity/fetch_dependencies.py` did not fetch `Assets/Rendering Settings/HDRP Profile.asset` or `Assets/XR/`,
  and the build failed until both were checked out.
- The OpenXR package fails the first build of a session ("settings found in project but not yet loaded"). The builder
  now preloads `Assets/XR` and builds once more in the same session.
- In a non-interactive PowerShell 5.1 host the launcher stops at the first Docker progress message. Run the commands
  from a normal terminal, or from `cmd /c "powershell -File avlite.ps1 ... > log 2>&1"`.
