# Jetson quickstart

Steps to get the localization and mapping stack running on the car for the first time.
Run `tools/jetson-run.sh` at each stage — it logs everything under `log/jetson/`.

## Prerequisites

- Key-based SSH access from the laptop to the Jetson (`racer@jetson.local` or similar)
- ROS 2 and vehicle workspace sourced in the Jetson shell
- This repository cloned to `$HOME/autodrive-roboracer` on the Jetson

## Where am I?

`bash tools/car-day.sh` is read-only. It checks the install and the F1TENTH bringup, lists every
profile measurement still missing with how to take it, and says what to do next. Run it any time.

## The short version

Once the hardware profile is complete (`python3 -m avlite_roboracer.doctor` says `23 of 23`) and the stationary tests in
step 6 pass, steps 7 and 8 below are one command, run with a person at the car holding the dead-man button:

```bash
bash tools/car-run.sh --start-stack --laps 3 --race-seconds 60
```

`HANDOVER.md` is the full runbook for an agent: how each profile value is measured and written
(`python3 -m avlite_roboracer.profile_edit`), what to do when the car stops, and what to report back.

## Checklist

Work through these in order. **Nothing autonomous moves until step 3 is complete.**

- [ ] **Step 1 — Inspect**  
  `bash tools/jetson-run.sh inspect`  
  Reads hardware/OS/ROS info. Copy the reported frame and topic names into `config/roboracer/hardware.yaml`. Leave all measurement fields `null`.

- [ ] **Step 2 — Record a manual lap and verify sensors**  
  Drive a slow teleop lap, then:  
  `bash tools/jetson-run.sh record`  
  `bash tools/jetson-run.sh check`  
  Resolve every `fail` in the report before continuing.

- [ ] **Step 3 — Commission (unblocks autonomous motion)**  
  Record stop tests at low speed (wheels clear or on the track), then:  
  `bash tools/jetson-run.sh commission`  
  Review the suggested values and fill them into `config/roboracer/hardware.yaml` under `measurements:`, `limits:`, `vehicle:`, and `mounts.laser`. Verify steering direction and that speed-0 brakes.

- [ ] **Step 4 — Validate mapping + localization offline**  
  Record a second manual lap, then:  
  `bash tools/jetson-run.sh map`  
  `bash tools/jetson-run.sh localize`  
  Check that the localization report passes every threshold. Use `--reference poses.csv` if an external reference exists.

- [ ] **Step 5 — Install the Jetson runtime**  
  `bash tools/jetson-setup.sh`  
  Installs SLAM Toolbox, the pinned AVLite venv and this stack into `~/roboracer_ws`. Add the `source` lines it prints to `~/.bashrc`. The desktop Docker image is not the deployment image.

- [ ] **Step 6 — Stationary hardware tests (motors disabled)**  
  `ros2 launch f1tenth_stack bringup_launch.py` in one terminal, then  
  `bash tools/jetson-run.sh launch` (preflight checks `/scan`, `/odom`, the `/drive` mux and transforms first)  
  Then from the laptop confirm: steering direction, actuator bounds, manual override, process failure, connection loss. None may cause movement.

- [ ] **Step 7 — First autonomous mapping lap**  
  From the laptop:  
  ```powershell
  .\avlite.ps1 car -Action map-start -JetsonHost racer@jetson.local -Session <id>
  ```  
  Verify the car stops near start, map is accepted, state reaches `READY`.

- [ ] **Step 8 — Track acceptance**  
  ```powershell
  .\avlite.ps1 car -Action race-start -JetsonHost racer@jetson.local -Session <id>
  ```  
  Several laps, then `stop`. Record scan alignment, localization error and stop response. Raise speed only after thresholds are established.

## Unverified assumptions (watch for these in steps 4–7)

- SLAM Toolbox service, topic and parameter names on the installed version
- `ros2 bag record --use-sim-time` support in the installed rosbag2
- Motor driver response to a speed-0 stop command
- AVLite Follow the Gap with the real LiDAR mount

## Logs

All runs write timestamped logs to `log/jetson/<step>-<timestamp>/`.  
Each directory contains:
- `run.log` — full stdout/stderr of every command
- `report/` — localization and sensor check reports (steps 2 and 4)
- `records.jsonl` — exported bag data (steps 2 and 4)

## Reference

Full details and state-machine description: [jetson.md](jetson.md)  
Hardware profile: `config/roboracer/hardware.yaml`  
SLAM Toolbox settings: `config/roboracer/slam_toolbox.yaml`
