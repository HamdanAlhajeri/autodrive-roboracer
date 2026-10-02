# Windows commands

Run from the repository root in PowerShell. Docker Desktop must be running with
Linux containers for container commands. One entry point replaces the old root
scripts:

```powershell
.\avlite.ps1 help
.\avlite.ps1 laps -Help
```

The implementation lives in `scripts/windows/`. Command options and validation
come from those scripts; the launcher does not duplicate defaults. Paths to the
repository are resolved from the script, so it also works when called by absolute
path from another directory. Relative paths you supply are relative to your
current PowerShell directory.

## Start, stop and reload

```powershell
.\avlite.ps1 start
.\avlite.ps1 status
.\avlite.ps1 logs -Tail 100
.\avlite.ps1 logs -Follow
.\avlite.ps1 restart
.\avlite.ps1 stop
```

`start` builds/starts bridge, actuator and AVLite, then opens the executable in
`config/windows-simulator.json`. `-SimulatorPath` overrides that selection.
Without either selection it downloads the stock practice build. The AVLite map
and selected simulator must match; changing an executable does not change the map.
For the custom build, follow [sketch setup](sketch-track.md).

Select **Connection** (`127.0.0.1:4567`) and **Autonomous** in the simulator.
`restart` stops AVLite, reloads the actuator, and starts AVLite with current YAML
settings; driving resumes when sensors are ready. `stop` stops AVLite first,
then actuator/bridge, and asks the selected simulator to close. Use the same
`-SimulatorPath` when stopping a build started with an explicit override.

## Record and inspect

```powershell
# Reset when prompted. Starts capture before driving, then stops AVLite.
.\avlite.ps1 laps -Laps 3 -Label controller-test
.\avlite.ps1 laps -Laps 10 -MaxSeconds 600 -NoOpen

# Passive capture: observes an already running car; does not stop it.
.\avlite.ps1 record -Seconds 120 -Label corner-test

# Offline plan inspection; TargetSpeedMps is a comparison, not a settings edit.
.\avlite.ps1 speed -TargetSpeedMps 10
```

`laps` defaults to one lap and a 600-second limit. It stops on the requested lap
count, a collision/reset, a failure or the time limit. Optional `-ResetSimulator`
requests a simulator reset instead of prompting; `-NoTrackMap` skips LiDAR outline
capture. Reports, telemetry, settings and logs go to `log/recordings/<run>/`.
See [telemetry and lap timing](avlite-setup.md#lap-timing) for graph definitions.

## Response and map preparation

```powershell
.\avlite.ps1 response -TargetSpeedMps 2.5 -Trials 3
.\avlite.ps1 response -RecordingDirectory .\log\recordings\YOUR-RUN
.\avlite.ps1 response -RecordingDirectory .\log\recordings\YOUR-RUN -CoastOnly
.\avlite.ps1 map -RecordingDirectory .\log\recordings\YOUR-RUN -Name practice-v2
```

The guided response controller supports 1.0-2.5 m/s. `-Trials 3` records four
laps, with at most one coast attempt per circuit. Reports do not change braking
calibration or shared speed settings. `-CoastOnly` analyzes ordinary recorded
coast intervals, replacing the separate braking-analysis script. It requires
saved telemetry and never starts driving. See [response measurements](response-measurements.md).

`map` requires a clean complete recording with LiDAR outline data and refuses to
overwrite an existing map name. See [map preparation](planned-driving.md).

## Track development and tests

```powershell
.\avlite.ps1 sketch-map
.\avlite.ps1 sketch-build
.\avlite.ps1 sketch-test -Visible
.\avlite.ps1 test
```

`sketch-build` accepts `-ProjectPath`, `-UnityPath`, and either `-SceneOnly` or
`-BuildOnly`. It needs an existing AutoDRIVE Unity source project and the exact
editor version described in [sketch setup](sketch-track.md). `sketch-test` accepts
`-SimulatorPath` and checks the native scene; it does not validate autonomous laps.
`test` runs command routing and response workflow tests with Docker mocked. See
[CONTRIBUTING](../CONTRIBUTING.md) for Python/ROS testing.

## Migration from the old commands

The old root scripts have been removed. Update local shortcuts using this table.
Options retain their existing names unless shown otherwise.

| Previous command | Replacement |
| --- | --- |
| `run-windows.ps1` | `avlite.ps1 start` |
| `run-windows.ps1 -Stop` | `avlite.ps1 stop` |
| `record-one-lap.ps1` | `avlite.ps1 laps` |
| `record-windows.ps1` | `avlite.ps1 record` |
| `measure-response.ps1` | `avlite.ps1 response` |
| `analyze-braking.ps1 -RecordingDirectory <run>` | `avlite.ps1 response -RecordingDirectory <run> -CoastOnly` |
| `check-speed.ps1` | `avlite.ps1 speed` |
| `prepare-race-map.ps1` | `avlite.ps1 map` |
| `prepare-sketch-track.ps1` | `avlite.ps1 sketch-map` |
| `build-sketch-track.ps1` | `avlite.ps1 sketch-build` |
| `test-sketch-track.ps1` | `avlite.ps1 sketch-test` |
| `run_tests.sh` (legacy Linux controller only) | `bash scripts/linux/test-legacy.sh` |

Restart instructions formerly in `Restart.md` are included above. Track assets
are kept once under `assets/tracks/sketch_track/`; the redundant ZIP is not needed
for cloning, building or running the project.
