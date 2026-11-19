# AGENT.md: start here

Read this file first. It explains what the repository is, what every part does and how to work in it, so you do not
have to read the code to find your way around. Then read `HANDOVER.md` (the real car on the Jetson) and
`docs/validation/localization-at-speed-20261118/PLAN.md` (the current simulator work and its status).

## Working rules from the owner

- Do ONE plan step, write a short document for it (what was done, issues and how they were fixed, screenshots, all
  sources), then stop and wait for the owner's approval before the next step.
- Do not commit or push unless asked. No AI attribution lines in commits or pull requests (no "Co-Authored-By",
  no "Generated with"). Branch names must not contain "claude".
- Give frequent status updates during long runs. When a trial runs, keep reading its log and say what is going on;
  never leave the owner in the dark and never claim something is running without checking.
- Never kill all `msedge.exe` or other processes you did not start. Do not leave background `find /` running.
- In Windows PowerShell 5.1 never redirect docker stderr with `2>&1`.
- Be honest about unproven results. A single 5-minute simulator run is too noisy to compare settings (identical
  settings gave 13 and 4 respawns).

## What this project is

A race stack for an AutoDRIVE RoboRacer (a small F1TENTH-style car), in two halves that share the same planner and
controllers:

1. **Simulator half** (`src/avlite_autodrive`): runs on a Windows laptop. Docker containers run ROS 2 plus AVLite
   (Abu Dhabi's AV-Lab planning framework, pinned to revision `1653f592...`) and talk to the AutoDRIVE Unity
   simulator over `127.0.0.1:4567`. Used to develop and measure.
2. **Real-car half** (`src/avlite_roboracer`): runs on a Jetson Orin NX in the car (Hokuyo UST LiDAR, VESC through
   the F1TENTH stack, ROS 2 Jazzy per the docs). A supervisor maps one slow lap with SLAM Toolbox, builds a race
   line, localizes with ICP, then races on command. It has never run on hardware.

The planner pipeline (`race_planning.py`, `race_map.py`, `plugin/planned_controller.py`) is the owner's own work. It
wraps AVLite's upstream `GlobalRacePlanner` and Pure Pursuit, and it is the same code in both halves.

## Repository map

```
AGENT.md, HANDOVER.md        you are here; the Jetson runbook
README.md, CONTRIBUTING.md   overview; where changes belong
avlite.ps1                   the one Windows entry point (see "Commands")
config/                      all settings (see "Config files")
assets/tracks/               custom track meshes and metadata (sketch, IROS 2026)
docker/, docker-compose*.yml the pinned AVLite image (Dockerfile.avlite) and services; docker/legacy is old
scripts/windows/             the PowerShell implementations behind avlite.ps1
scripts/jetson/roboracer     small Jetson launcher script
src/avlite_autodrive/        simulator half (ROS package)
src/avlite_roboracer/        real-car half (ROS package)
src/my_team_racer/           an early, standalone Follow-the-Gap example node; not used by the stack
tools/                       helper scripts (car, Jetson, simulator measurement, Unity)
tests/                       PowerShell tests for the Windows commands
docs/                        guides, plans, and validation write-ups (see "Docs")
log/                         recordings and run output; git-ignored
local-reports/               local-only Word reports; git-ignored
```

### `src/avlite_autodrive/avlite_autodrive/` (simulator half)

| Module | What it does |
| --- | --- |
| `runner.py` | Entry point. Wall-clock paced launcher of AVLite's synchronous stack; wires localization, controller, diagnostics. |
| `simulator_bridge.py`, `plugin/bridge.py` | Start the stock AutoDRIVE API (guarded against incomplete packets); ROS transport for the simulator. No vehicle model runs here. |
| `adapter.py`, `actuation.py` | Independent actuator publisher (AVLite never owns the actuator topics); pure command conversion. |
| `sensors.py`, `ros_utils.py` | LaserScan to AVLite's sensor-frame cloud; message validation shared by the processes. |
| `race_map.py`, `race_planning.py` | Closed-track geometry and RaceMap preparation; adapter around AVLite `GlobalRacePlanner` (speed profile with corner braking). |
| `plugin/planned_controller.py` | Pure Pursuit with periodic racing and a LiDAR stopping envelope. Raises safety stops (`speed_limit_reason`; 3 = invalid pose). |
| `plugin/controller.py` | Follow-the-Gap gap selection (used for mapping laps). |
| `plugin/response_controller.py`, `response_test.py`, `response_analysis.py`, `braking_analysis.py` | Throttle-off response and braking measurements. |
| `avlite_localization.py` | AVLite's own LiDAR ICP as the pose source, with wheel-odometry prediction, gap handling, lost-pose recovery. |
| `slam_localization.py` | SLAM Toolbox localization as an alternative pose source (never reached READY here). |
| `sim_odometry.py` | Wheel odometry for the simulator without ground truth; `ClockRate` corrects for the simulator running slower than wall clock. |
| `pose_view.py`, `pose_view.html`, `plugin/pose_view.py` | Live browser view (http://localhost:8765) of estimated vs true pose. |
| `track_map.py`, `sketch_track.py` | Partial outline from LiDAR; converts Unity track metadata into an offline AVLite map and commissioning profile. |
| `record.py`, `lap_tracking.py`, `plot_recording.py`, `diagnostics.py`, `telemetry_health.py` | Telemetry capture, lap/finish counting, CSV+PNG export, allow-listed diagnostics, publication checks. |
| `simulator_reset.py`, `speed_check.py`, `configuration.py` | Reset and confirm a fresh start; offline speed feasibility; resolve shared values from `driving.yaml`. |

### `src/avlite_roboracer/avlite_roboracer/` (real-car half; most modules are ROS-free so they test anywhere)

| Module | What it does |
| --- | --- |
| `supervisor.py` | State machine: IDLE, MAPPING, FINALIZING, READY, RACING, STOPPED. No ROS, time passed in. |
| `runtime.py` | The ROS 2 node running the supervisor: map one lap, finalize, wait for `race-start`, race. |
| `cli.py`, `protocol.py` | Operator commands (`status`, `map-start`, `race-start`, `stop`) over a local Unix socket. Motion is authorized only while the CLI keeps sending heartbeats. |
| `actuator_node.py`, `command.py` | Independent hardware actuator: command ownership, expiry, conversion to `/drive`. Dead-man button must be held. |
| `profile.py`, `profile_edit.py`, `doctor.py` | Load/validate `hardware.yaml`; write measured values safely from the command line; list what is still unmeasured. |
| `commissioning.py`, `readiness.py` | Latency, braking and stopping distance from stop tests; gates that must pass before autonomous motion. |
| `slam.py`, `map_builder.py`, `occupancy.py`, `lap_detector.py` | Run SLAM Toolbox; build a RaceMap from the saved map and the driven route; occupancy queries; detect mapping-lap completion. |
| `icp_pose.py`, `localization.py`, `racing.py` | ICP localization for the real car; localization health for the READY gate; bind the racing profile to measured limits. |
| `recorder.py`, `records.py`, `recording_check.py`, `quality.py`, `report.py` | Record/export/replay sensor sessions; checks on timestamps, transforms, odometry; localization-quality report. |
| `geometry.py` | Planar poses and a time-indexed transform buffer. |

## Config files (`config/`)

| File | Purpose |
| --- | --- |
| `avlite.yaml` | Main simulator config: driving mode, map, `localization:` block (`source: avlite` = ICP, `ground_truth`, `slam`), recovery settings, pose view. |
| `driving.yaml` | Shared speed and throttle ceilings, loaded by `avlite.yaml` and `actuator.yaml`. |
| `actuator.yaml` | Actuator limits and timeout. |
| `avlite.sketch.yaml`, `maps/sketch*.json`, `maps/practice.json` | The custom sketch track profile and maps; the stock practice track map. |
| `avlite.iros2026.yaml`, `maps/iros2026*.json` | The IROS 2026 replica's ICP profile and map (see below). |
| `roboracer/hardware.yaml` | The car's hardware profile. Mostly `null` until measured: this is what blocks autonomous motion. |
| `roboracer/race.yaml` | Mapping laps, planning limits, localization source, racing laps. |
| `roboracer/sim_profile.yaml`, `slam_toolbox.yaml` | Same schema as `hardware.yaml` for the simulator; SLAM Toolbox settings. |

`config/windows-simulator.json` (a local file, not in git; create it only when using a custom build) selects which
simulator build is started. The map in `avlite.yaml` must match that build.

## Commands

Windows, from the repo root in PowerShell (`.\avlite.ps1 help` lists all; details in `docs/commands.md`):
`start`, `stop`, `restart`, `status`, `logs`, `laps -Laps N -Label x` (record laps and plot), `record`, `live`
(pose view), `speed` (planned speed range, no driving), `sweep`, `response`, `map`, `sketch-map`, `sketch-build`,
`sketch-test`, `car` (talk to the Jetson over SSH), `test`.

Unit tests (laptop has no pytest; they run in the pinned Docker image, about 40 s):

```bash
bash tools/jazzy-sim/unit.sh                          # both packages
bash tools/jazzy-sim/unit.sh test/test_foo.py         # one file; paths are relative to each package dir
```

Last full result: 167 roboracer and 395 autodrive tests pass.

Jetson (run on the car; `HANDOVER.md` has the full runbook): `bash tools/car-day.sh` (read-only status and next step),
`tools/jetson-setup.sh` (one-time install), `tools/jetson-run.sh` (commissioning and validation), `tools/car-run.sh`
(map slowly for a few laps, then race), `python -m avlite_roboracer.profile_edit` (fill in the profile).

Simulator measurement (`tools/sim-localization/`): `measure_pose.py` records truth, estimate, odometry and commands;
`run_trials.sh` runs independent trials with a proper respawn (`respawn.py`); `analyze_pose_measure.py`,
`summarize_trials.py`, `rt_summary.py` summarize; `gap_test.sh`, `gap_blind.py`, `launch_lead.py`, `reset_test.py`
are the Step 2 and 3 experiments. `tools/jazzy-sim/` is a no-car test harness for the Jetson stack. `tools/unity/`
builds custom tracks in a Unity checkout.

## Docs

- `docs/commands.md`, `docs/avlite-setup.md`, `docs/planned-driving.md`: running and tuning the simulator.
- `docs/jetson.md`, `docs/jetson-quickstart.md`, `docs/plans/`: the car (design, short version, checklists).
- `docs/sketch-track.md`, `docs/iros2026-track.md`: building the custom Unity tracks.
- `docs/validation/`: every measurement write-up, newest in `localization-at-speed-20261118/`.
- `docs/research/localization-resources.txt`, `docs/history/`: background.

## Goal and plan status (details: `docs/validation/localization-at-speed-20261118/`)

Goal: the simulated car crashes less with AVLite's own LiDAR localization (ICP), at current speeds, and keeps working
at higher speeds later.

| Step | What | Status | Write-up |
| --- | --- | --- | --- |
| 0 | Measure delay and localization error | Done | `step-0-measurement.md` |
| 1 | Fix the speed the controller sees (simulator clock) | Done. Speed ratio about 0.5 to 0.98 near real time; crashes did not drop | `step-1-speed-source.md` |
| 2 | Reset and recovery: respawn, no re-seed on a sensor gap, recovery search | Done. Blind travel of 3 m or more is still lost | `step-2-reset-and-recovery.md` |
| 3 | Launch error: limit the estimate's lead over wheel odometry | Done. The lead is real (wheel slip) but limiting it made crashes about 5x worse; left off | `step-3-launch-error.md` |
| 4 | Re-measure at real time; latency, braking, accel-limit options | Done. No gain from those options | `step-4-real-time-remeasure.md` |

Open and needing an owner decision: the planner margin at the tight corner and the permanent "invalid pose" stop.

## Key findings

- With true pose the car drives 38 clean laps in 5 minutes. With ICP it does 21 to 30 laps with 3 to 9 first wall
  contacts (simulator at 0.99 of real time).
- 8 of 9 first contacts are at 10.1 to 12.6 m along the lap (entry to the bottom corner); one at 1.0 m.
- Most respawns are "invalid pose" stalls (`speed_limit_reason` 3) with an accurate estimate. The car cannot restart
  from that state.
- Launch wheel slip: wheels read 3.4 to 4.7 m/s while the car moves 0.4 to 1.4 m/s, so the estimate leads by
  0.3 to 1.2 m. An estimate ahead of the car is the safe direction; limiting it makes it lag and crashes rise.
- A sensor gap no longer re-seeds the localizer; a recovery search runs after loss (3 m radius). Time-limiting the
  search made it worse (1 of 6 recovered against 3 of 4), so it is unlimited.
- Estimate lag is about 10 ms, so latency compensation is not the first fix.
- Real-car readiness: software ready to try; car not ready (hardware profile 0 of 23 set, stationary tests not done).
  Thresholds in `localization:` are simulator starting values, not measurements.

## IROS 2026 track

A simulator copy of the RoboRacer IROS 2026 track, traced from the organisers' map: `assets/tracks/iros2026`,
`config/maps/iros2026.json`, `config/avlite.iros2026.yaml`, and `-Track iros2026` for `sketch-map`, `sketch-build`,
`sketch-test`. Guide: `docs/iros2026-track.md`. The profile uses ICP (`source: avlite`, `reference: map_hits`): the
converter writes this track's spawn into `start_pose_world` and adds synthetic `observed_hits` along the pipe surfaces.
Offline, ICP held a full simulated lap (`docs/validation/iros2026-track-20261119/`). It has NOT been built in Unity or
driven; that needs Unity 2022.3.52f1 and a Unity source checkout.

## Gotchas

- Line endings: shell scripts must be LF (`.gitattributes` enforces it); Python and YAML files are often CRLF. Edit
  with tools that keep the file's line endings. `profile_edit` already does.
- Git Bash on Windows needs `MSYS_NO_PATHCONV=1` before `docker run -v` paths. `python3` is not on the laptop PATH.
- The Bash tool collapses backslashes in heredocs; use the Write tool for large files. Bash reads scripts
  incrementally, so do not edit a script while it runs.
- After the reset command the simulator car can stay frozen (wheels spin, body does not move). `respawn.py` and
  `run_trials.sh` handle it: stop AVLite, respawn, wait for "Sensors ready", restart odometry, start AVLite.
  If it still freezes, close the simulator, then `.\avlite.ps1 stop` and `.\avlite.ps1 start`, and select Connection
  and Autonomous.
- Simulator speed varies (0.5x to 0.99x real time). Check the real-time factor in the recording before comparing runs.
- The university network blocks SSH to the Jetson; see `HANDOVER.md` for the router and tmux advice. The Hokuyo uses
  the Jetson's wired port at 192.168.0.15/24.
- Unverified on hardware: `--start-stack` in `car-run.sh`, SLAM Toolbox node names on the installed version, VESC
  speed-0 braking, Follow-the-Gap with the real LiDAR mount, the `/drive` mux, and all localization thresholds.

## Sources

Opened directly: the paper abstract and repository README for the first two links. Everything else was read only as
search-result summaries and should be checked before it is quoted.

Localization, racing stacks and benchmarks:
- [Robustness Evaluation of Localization Techniques for Autonomous Racing (SynPF vs Cartographer, up to 7.6 m/s)](https://arxiv.org/abs/2401.07658)
- [ForzaETH particle filter (SynPF): code and tuning notes](https://github.com/ForzaETH/particle_filter)
- [Unifying F1TENTH Autonomous Racing: Survey, Methods and Benchmarks](https://arxiv.org/abs/2402.18558)
- [CDDT: Fast Approximate 2D Ray Casting for Accelerated Localization](https://www.arxiv-vanity.com/papers/1705.01167/)
- [Fast and Modular Autonomy Software for Autonomous Racing Vehicles](https://arxiv.org/pdf/2408.15425)

Delay compensation and control:
- [Stellenbosch University write-up: simulation to reality, pure pursuit, MPC and MPCC on the F1TENTH platform](https://www.su.ac.za/faculties/engineering/departments/electrical-electronic-engineering/news/simulation-reality-assessing-efficacy-pure-pursuit-mpc-and-mpcc-f1tenth-platform)
- [A critical evaluation of Pure Pursuit, MPC and MPCC (the underlying paper)](https://www.maratto.africa/research/a-critical-evaluation-of-pure-pursuit-mpc-and-cf1725da31)
- [TUM autonomous motorsport software for the Indy Autonomous Challenge](https://arxiv.org/pdf/2205.15979)
- [Patent: vehicle controller robust to time delays (pose extrapolation)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10800427)
- [Control strategies for autonomous vehicles (pure pursuit lookahead)](https://arxiv.org/pdf/2011.08729)
- [F1TENTH pure-pursuit lookahead paper, 2026 (found by search; the summary I read may describe a neighbouring result)](https://www.arxiv.org/pdf/2602.18386)

Scan matching, deskewing and fusion:
- [DAMM-LOAM: degeneracy-aware multi-metric LiDAR odometry and mapping](https://arxiv.org/html/2510.13287)
- [Kilometer-scale autonomous navigation in subarctic forests (deskewing with wheel odometry and IMU)](https://arxiv.org/pdf/2111.13981)
- [Direct LiDAR-Inertial Odometry: continuous-time motion correction](https://arxiv.org/pdf/2203.03749)
- [A comprehensive evaluation of LiDAR odometry techniques](https://arxiv.org/pdf/2507.16000)
- [Sensor fusion and robot localization using ROS 2 Jazzy (robot_localization)](https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/)

Localization-aware speed:
- [Kibalov and Shipitko: speed control under ego-pose uncertainty](https://arxiv.org/abs/2003.00675)

AVLite (the framework this builds on):
- [AV-Lab/avlite at the pinned revision](https://github.com/AV-Lab/avlite/tree/1653f592b2eb5289a14c0d1041e50fdea855322a) (task strategy, executer and app layers)

Project files used: `HANDOVER.md`, `docs/validation/overnight-20261118/README.md`,
`docs/validation/sim-localization-20261117/README.md`, `config/avlite.yaml`,
`src/avlite_autodrive/avlite_autodrive/sim_odometry.py`, `avlite_localization.py`, `runner.py`.
