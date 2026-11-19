# Handover: run the car (map slowly, then race)

Read this top to bottom before touching anything. It is written for an agent working on the
Jetson with a person at the car. Last updated 19 Nov 2026.

## 1. The mission

The owner wants one thing: **the car laps the track slowly by itself to map it, then races it.**
The software for that exists and is tested in the simulator and with a fake supervisor. **It has never run
on the real car.** Your job is to get it to the first real run safely and report what happened.

```bash
bash tools/car-run.sh --start-stack --laps 3 --race-seconds 60     # the whole thing
```

That command is the goal. Everything below is what has to be true before it will run (it refuses otherwise).

## 2. Rules (from the owner, do not break them)

- **A person must be at the car** with the emergency stop in hand and holding the joystick **dead-man button**
  during every motion step. The car moves only while that button is held and `car-run.sh` is running.
  Never try to get around that. You may not "test" motion yourself with raw ROS commands: motion goes only
  through the supervisor (`car-run.sh`, or `scripts/jetson/roboracer map-start|race-start`).
- **First autonomous speeds stay at or below 1.0 m/s.** `profile_edit` refuses more than 1.5 m/s without
  `--allow-fast`; do not pass it without measured stopping evidence and the owner's say-so.
- **Never copy simulator numbers into `config/roboracer/hardware.yaml`.** Simulator gains, limits and speeds do not
  transfer (the sim drives at 6 m/s, this car starts at 1).
- **Do not commit or push unless the owner asks.** No Claude or AI attribution in commits or pull requests.
- **Do not edit scripts or source while a run is going** (bash reads scripts as it runs).
- Report honestly: say what you measured, what you assumed, and what you did not check.

## 3. Where to start: one command

```bash
cd ~/autodrive-roboracer
bash tools/car-day.sh
```

It is read-only. It checks the install, the F1TENTH topics (if the bringup is running) and the hardware profile,
lists every missing measurement with how to take it, and says what to do next. Run it any time; re-run it after
each step. Work down this document in order and stop when `car-day.sh` shows no `[FAIL]` and
`0 of 23` has become `23 of 23`.

## 4. Setup, in order

| # | Do | Command | Who |
| --- | --- | --- | --- |
| 1 | Get the code on the Jetson | `git clone <repo> ~/autodrive-roboracer` (or `git pull`) | agent |
| 2 | Install the runtime (SLAM Toolbox, the pinned AVLite venv, build both packages) | `bash tools/jetson-setup.sh`, then add the `source` lines it prints to `~/.bashrc` | agent |
| 3 | Build the F1TENTH drivers (Jazzy has no apt package) | the commands in `docs/jetson.md` section "F1TENTH car" (verified once on 10 Nov, not re-run since) | agent |
| 4 | Network and ports | Hokuyo needs the wired port at `192.168.0.15/24` (sensor `192.168.0.10`); VESC port in `f1tenth_stack/config/vesc.yaml` | agent / person |
| 5 | Start the F1TENTH bringup | `ros2 launch f1tenth_stack bringup_launch.py` (leave it running, or let `car-run.sh --start-stack` do it) | agent |
| 6 | Confirm the interface names | `bash tools/jetson-run.sh inspect`; names must match `config/roboracer/hardware.yaml` (`/scan`, `/odom`, `/drive`, `/teleop`, frames `odom` `base_link` `laser`) | agent |

Known facts about the car (from `docs/jetson.md`, 3 and 10 Nov): Orin NX, Ubuntu 24.04, ROS 2 Jazzy, Python 3.12,
Hokuyo UST via `urg_node2`, VESC via `vesc_driver`. `jetson-setup.sh` also supports Humble on JetPack 6.

## 5. Fill in the hardware profile (this is what unlocks motion)

`config/roboracer/hardware.yaml` starts with every measurement `null`. The supervisor refuses to move until all 23
are set. **Write values only with the editor**, never by hand:

```bash
python3 -m avlite_roboracer.profile_edit set vehicle.wheelbase_m=0.33 steering.positive_left=true
```

(Run from the repo root with `PYTHONPATH=src/avlite_roboracer`, or from a shell where `~/roboracer_ws` is sourced.)
It checks the key, the type and the whole profile (limit ordering, positive numbers, geometry order) before
writing, changes only that line, keeps comments, and saves `hardware.yaml.bak`. A rejected value changes nothing.
Check progress with `python3 -m avlite_roboracer.doctor --profile config/roboracer/hardware.yaml`.

### 5.1 Values the agent can get by itself

| Keys | Command | Needs |
| --- | --- | --- |
| `mounts.laser.x_m`, `y_m`, `yaw_rad` | `python3 -m avlite_roboracer.profile_edit laser-from-tf` | bringup running (reads the real `base_link -> laser` transform) |
| `deadman.button` | `python3 -m avlite_roboracer.profile_edit deadman-from-joy` | joystick on; a person holds the button when it asks |
| `localization.max_position_stddev_m` 0.15, `max_yaw_stddev_rad` 0.08, `min_scan_alignment` 0.7, `max_motion_mismatch_m` 0.15, `max_motion_mismatch_rad` 0.10 | `python3 -m avlite_roboracer.profile_edit defaults-localization` | nothing. **These are starting acceptance limits that passed in the simulator, not measurements.** Tighten them later from `status` and the localization numbers. |
| `measurements.command_latency_s`, `braking_deceleration_mps2`, `max_acceleration_mps2`, `stop_test_speed_mps`, `stopping_distance_m` | see 5.2 | a person driving slow stop tests |

### 5.2 Stop tests (latency, braking, acceleration, stopping distance)

A person drives the car with the joystick, wheels on the floor, in clear space, at low speed:

1. Start the recorder: `bash tools/jetson-run.sh record` (records `/scan`, `/odom`, `/tf`, and the joystick and
   `/drive` commands; it is stopped with Ctrl+C).
2. The person does **5 to 8 quick pushes from rest to about 0.5 m/s, then lets go completely**, then the same to
   about **1.0 m/s**, each with a clear stop in between. The analysis looks for a command that goes from about 0 to
   0.3 m/s or more (a start) and back to about 0 (a stop).
3. Stop the recording, then `bash tools/jetson-run.sh commission`. It writes `stops.json` and prints the worst
   case values (latency, deceleration, stopping distance, the fastest stop speed, and the slowest acceleration).
4. Apply them: `python3 -m avlite_roboracer.profile_edit commissioning log/jetson/<run>/stops.json`
   (it only writes values that were actually measured; any key it could not measure stays `null`: do more stops).

### 5.3 Values only a person with a tape measure and the car can give

Ask the person; do not guess. Then write them with `profile_edit set ...`.

| Key | How |
| --- | --- |
| `vehicle.wheelbase_m` | front axle to rear axle centre-to-centre |
| `vehicle.width_m` | widest point including wheels |
| `vehicle.length_m` | bumper to bumper |
| `vehicle.rear_axle_to_front_m` | rear axle to front bumper (must satisfy wheelbase <= this <= length) |
| `steering.max_angle_rad` | wheels off the ground, steer to the largest angle without binding; the profile also needs the sign |
| `steering.positive_left` | wheels off the ground, command a small **positive** steering angle: `true` if the wheels point left |
| `motor.max_command_speed_mps` | hard clamp sent to the VESC: **1.0** to start |
| `limits.mapping_speed_mps` | slow mapping laps: **0.5 to 0.8** to start (must be <= `racing_speed_mps`) |
| `limits.racing_speed_mps` | **1.0** to start (must be <= `measurements.stop_test_speed_mps`, so stop tests at 1.0 m/s come first) |

When `doctor` reports `23 of 23` and `commissioned`, the profile is done.

### 5.4 Stationary tests (no motion allowed: wheels off the ground or the car held)

From `docs/jetson-quickstart.md` step 6, with the stack running (`ros2 launch f1tenth_stack bringup_launch.py` and
`bash tools/jetson-run.sh launch`): steering direction matches `positive_left`; actuator bounds hold; a manual
joystick command overrides autonomy; killing a process stops the car; losing the connection stops the car; speed 0
brakes rather than coasts. **A person does and watches these.** Do not skip them.

## 6. The run

```bash
bash tools/car-run.sh --start-stack --laps 3 --race-seconds 60
```

| Option | Meaning |
| --- | --- |
| `--start-stack` | also starts the F1TENTH bringup and the supervisor (logs in the run folder), stops them at the end |
| `--laps N` | how many slow mapping laps (edits `mapping.laps` in `config/roboracer/race.yaml`); default 3 |
| `--race-seconds N` | race for N seconds, then stop; default 60 |
| `--map-only` | map and validate, do not race |
| `--race-only` | race on an already READY map |
| `--remap` | map again even if a map is READY |
| `--yes` | skip the "type GO" prompts. **Only when a person is at the car with the emergency stop and dead-man button.** Without `--yes` and without a terminal it refuses to move the car. |
| `--no-preflight` | skip `car-day.sh` (do not) |

What happens, with the person holding the dead-man button:

1. **Preflight** (`car-day.sh`): install, bringup topics, profile commissioned. Fails here mean nothing was sent to the car.
2. **Mapping**: the car drives the slow laps with Follow the Gap while SLAM Toolbox builds the map. It stops by itself
   when the lap is confirmed (finish line seen again, or scan similarity). Travel limit and timeouts apply.
3. **Finalizing**: builds the race map, the racing line (AVLite `GlobalRacePlanner`) and checks localization against the
   thresholds. Reaches `READY` or `STOPPED` with reasons. The car does not move during this.
4. **Racing** for the set time: planned controller on the racing line, localized by AVLite's own ICP against the
   saved map (wheel-odometry prediction), then `stop`.

Exit codes: `0` done, `1` failed or stopped, `2` setup problem (nothing sent to the car), `130` interrupted.
Everything is logged to `log/car-run/<timestamp>/run.log` (plus `bringup.log`, `supervisor.log`, `preflight.log`).

Without the script (the laptop route): `.\avlite.ps1 car -Action map-start|race-start|status|stop -JetsonHost racer@jetson.local`.
Same supervisor, heartbeats from the laptop instead of the Jetson terminal.

## 7. When it stops (reasons and what to do)

The supervisor's `status` (`bash scripts/jetson/roboracer status`, or `--json`) lists `reasons`. Common ones:

| Reason (as printed) | Meaning and what to do |
| --- | --- |
| `hardware profile not commissioned: ...` | a measurement is missing: section 5 |
| `dead-man button released or joystick silent` | hold the button; check `deadman.button` and that `/joy` publishes |
| `manual override engaged` | a joystick command on `/teleop`; let go, start again |
| `LiDAR scans stale or missing` / `odometry stale or missing` | check the Hokuyo network and `vesc_to_odom`; `ros2 topic hz /scan /odom` |
| `actuator process unavailable` / `actuator fault: ...` | the actuator node is not running or faulted; see `supervisor.log` |
| `a SLAM process is already running; stop it first` | a leftover `slam_toolbox`; stop it, then retry |
| `expected exactly one localization source ...` | two localizers; one just stopped can linger a few seconds (fixed to ignore ours for 15 s); wait and retry |
| `mapping lap not confirmed within the travel limit` | the lap was not recognized: check the car actually completed a lap, the speed is not too low, and that LiDAR is not blocked; see `lap_detector.py` |
| `mapping pose stale (...)` / `stale LiDAR or odometry` | a sensor or CPU stall; check load, USB and network |
| `lost localization: ...` | the pose left its limits during mapping |
| `car moved during finalization` | something pushed the car; start again |
| finalize failure reasons (`finalize_failed`) | the map, plan or localization did not validate: read them, look at the map in `session_dir`, and see `docs/jetson.md` for each check |
| `localization pose stale` / `car is outside the validated corridor` (racing) | ICP lost the pose or the car left the corridor; the planner stops on purpose (see section 9) |

Every stop leaves the session folder (`status` shows `session_dir`) with the traces: `slam_trace.csv` (pose while mapping)
and `icp_trace.csv` (pose while racing). Keep them: they are the data the owner and the next agent need.

## 8. What to report back after a run

- Did it reach READY? How many laps did it map, and how long did mapping and finalizing take?
- The `status` block at READY (localization numbers, `icp_alignment`) and at the end of the race.
- Any stop and its reason, and what the person saw.
- The path of `log/car-run/<ts>/` and the `session_dir`.
- What you changed in the profile (every `profile_edit` line is printed; keep them) and what is still unverified.

Do **not** raise speeds after one good run. Raise `limits.*` only with a new stop test at the higher speed (the profile
rejects racing speeds above the measured stop-test speed), and only if the owner agrees.

## 9. What the simulator work found (so you do not redo it)

The planned-driving pipeline is the owner's own (map validation, racing line via AVLite's `GlobalRacePlanner`, Pure Pursuit
with safety stops, in `src/avlite_autodrive/`). The simulator results are in `AGENT.md` and
`docs/validation/localization-at-speed-20261118/` (steps 0 to 4, with a report in `local-reports/`). In short:

- **ICP localization works in the simulator**: about 0.14 m typical error at 150 s of racing with no drift, 0.35 m with
  10% bad odometry and a noisy LiDAR. SLAM Toolbox as the *localizer* never became READY in the harness (alignment 0.55),
  so the car uses ICP (`localization.source: icp` in `race.yaml`). Do not spend time on SLAM localization first.
- **SLAM during mapping** can slide up to 2.8 m on a long straight at `distance_variance_penalty: 0.5` (the kept default);
  lower values fix that with accurate odometry and break with bad odometry. Re-test 0.2 on the car after measuring the
  odometry error (`config/roboracer/slam_toolbox.yaml` has the numbers).
- **The true pose beats ICP in the simulator** (38 clean laps in 5 min against 21 to 30). ICP's 0.15 to 0.4 m error is
  enough to hit a wall at the tightest corner or to trip the planner's "invalid pose" stop (footprint outside the
  validated corridor), which then cannot restart. On the real car at 1 m/s the margins are far larger, but this is the
  main thing to watch: if the car stops with `car is outside the validated corridor`, check `icp_trace.csv`.
- **Things that made it worse, left off**: limiting the odometry's launch acceleration (estimate lagged, 5x crashes);
  weaker assumed braking and a 50 or 100 ms pose lead showed no gain. Options exist in `config/avlite.yaml` (simulator only).
- **Simulator-only fixes** (not on the car): the odometry clock-rate fix, the re-seed-on-sensor-gap fix and the lost-pose
  recovery in the simulator runner. The car has its own ICP wrapper with a guard and a recovery search (`icp_pose.py`).
- Open item for the owner: a slower or wider margin at the tight corner, or a recoverable "invalid pose" stop. Safety logic:
  do not change it without the owner's decision.

## 10. Where things are

| Need | Look at |
| --- | --- |
| The run script | `tools/car-run.sh` (tests: `src/avlite_roboracer/test/test_car_run.py`, fake supervisor) |
| Status of the car and what is missing | `tools/car-day.sh`, `python3 -m avlite_roboracer.doctor` |
| Write profile values | `python3 -m avlite_roboracer.profile_edit` (tests: `test_profile_edit.py`) |
| Hardware profile | `config/roboracer/hardware.yaml`, `profile.py` |
| Race and mapping tuning | `config/roboracer/race.yaml` (`mapping.laps`, planner limits, ICP settings) |
| SLAM Toolbox settings | `config/roboracer/slam_toolbox.yaml` |
| Supervisor state machine | `supervisor.py` (IDLE, MAPPING, FINALIZING, READY, RACING, STOPPED), `runtime.py` |
| Lap detection | `lap_detector.py` |
| ICP pose for the car | `icp_pose.py` wrapping `avlite_autodrive/avlite_localization.py` |
| Map, race map, planning | `map_builder.py`, `avlite_autodrive/race_planning.py`, `race_map.py` |
| Docs | `docs/jetson-quickstart.md`, `docs/jetson.md` |
| Simulator tests (no car) | `bash tools/jazzy-sim/unit.sh`; full flow `bash tools/jazzy-sim/run_humble.sh` (Docker, laptop) |

## 11. Gotchas

- Many files are CRLF on the owner's Windows laptop. Shell scripts must be LF (`.gitattributes` enforces it); a CRLF
  script fails with `$'\r': command not found`.
- `python3` is on the Jetson but not on the laptop (use `python` there).
- Do not run two localizers or two SLAM nodes at once.
- Docker or VM stalls are a laptop problem; the Jetson runs natively.
- `tools/*.sh` are not executable in git: call them with `bash tools/x.sh`.

## 12. Unverified (assume nothing)

From the quickstart and my own reading, not yet checked on the car: SLAM Toolbox service/topic/parameter names on the
installed version; `ros2 bag record --use-sim-time` support; the VESC's response to speed 0 (brake or coast); Follow the
Gap with the real LiDAR mount; whether `/drive` reaches the VESC without a held button (depends on `joy_teleop.yaml`
and the mux); and everything in `car-run.sh --start-stack` that starts real processes (the logic is tested against a
fake supervisor only).

## 13. State of the repository

Branch `main` on GitHub has the work through 18 Nov (commit `2fa1673`). The following is **not committed yet**:
the Step 1 to 4 simulator work (localization recovery, options, tools, docs), and today's additions: `tools/car-run.sh`,
`profile_edit.py`, acceleration in the commissioning analysis, the `car-day.sh` Jazzy fix, `.gitattributes`, and this file.
Ask the owner before committing; no AI attribution in commit messages.
