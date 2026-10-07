# Jetson quickstart

Steps to get the localization and mapping stack running on the car for the first time.
Run `tools/jetson-run.sh` at each stage — it logs everything under `log/jetson/`.

## Prerequisites

- Key-based SSH access from the laptop to the Jetson (`racer@jetson.local` or similar)
- ROS 2 and vehicle workspace sourced in the Jetson shell
- This repository cloned to `$HOME/autodrive-roboracer` on the Jetson

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
  Install ROS 2, SLAM Toolbox and the pinned AVLite revision for the JetPack version found in step 1. The desktop Docker image is not the deployment image.

- [ ] **Step 6 — Stationary hardware tests (motors disabled)**  
  `bash tools/jetson-run.sh launch`  
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

---

## jetson-run.sh

Save this as `tools/jetson-run.sh` on the Jetson and run with `bash tools/jetson-run.sh <step>`.

```bash
#!/usr/bin/env bash
# Jetson commissioning and validation runner.
# Usage: bash tools/jetson-run.sh <step>
#
# Steps: inspect | record | check | commission | map | localize | launch
#
# Logs everything under log/jetson/<step>-<timestamp>/.
# Safe to run multiple times; each run gets its own directory.
# Does not install software, push code, or send vehicle commands.
set -euo pipefail

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="$REPO/config/roboracer/hardware.yaml"
PYTHONPATH_EXTRA="$REPO/src/avlite_roboracer:$REPO/src/avlite_autodrive"
export PYTHONPATH="${PYTHONPATH_EXTRA}${PYTHONPATH:+:$PYTHONPATH}"

STEP="${1:-}"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
LOG_BASE="$REPO/log/jetson/${STEP}-${TIMESTAMP}"
LOG_FILE="$LOG_BASE/run.log"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
usage() {
    printf 'Usage: bash tools/jetson-run.sh <step>\n'
    printf 'Steps:\n'
    printf '  inspect     Hardware/OS/ROS report (safe, read-only)\n'
    printf '  record      Record a manual lap (requires teleop running)\n'
    printf '  check       Check most-recent recorded lap\n'
    printf '  commission  Analyze stop tests from most-recent recording\n'
    printf '  map         Replay most-recent recording through SLAM to build a map\n'
    printf '  localize    Replay second-most-recent recording and localize against the map\n'
    printf '  launch      Start the ROS stack (does not send drive commands)\n'
    exit 1
}

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG_FILE"; }

run() {
    log "$ $*"
    "$@" 2>&1 | tee -a "$LOG_FILE"
    log "exit $?"
}

latest_recording() {
    local base="$REPO/log/jetson"
    local match
    match="$(find "$base" -maxdepth 1 -name 'record-*' -type d | sort | tail -1)"
    if [[ -z "$match" ]]; then
        log "ERROR: no recording found under $base. Run the 'record' step first."
        exit 1
    fi
    printf '%s\n' "$match"
}

second_latest_recording() {
    local base="$REPO/log/jetson"
    local match
    match="$(find "$base" -maxdepth 1 -name 'record-*' -type d | sort | tail -2 | head -1)"
    if [[ -z "$match" ]]; then
        log "ERROR: need at least two recordings for localization. Run 'record' twice."
        exit 1
    fi
    printf '%s\n' "$match"
}

latest_map() {
    local base="$REPO/log/jetson"
    local replay
    replay="$(find "$base" -maxdepth 2 -name 'replay-mapping-*' -type d | sort | tail -1)"
    if [[ -z "$replay" ]]; then
        log "ERROR: no mapping replay found. Run the 'map' step first."
        exit 1
    fi
    printf '%s\n' "$replay"
}

# ---------------------------------------------------------------------------
# Guards
# ---------------------------------------------------------------------------
if [[ -z "$STEP" ]]; then usage; fi
if ! command -v python3 >/dev/null 2>&1; then
    printf 'ERROR: python3 not found. Source the ROS and vehicle workspace first.\n'
    exit 1
fi

mkdir -p "$LOG_BASE"
log "=== jetson-run.sh: step=$STEP repo=$REPO ==="
log "Log directory: $LOG_BASE"

# ---------------------------------------------------------------------------
# Steps
# ---------------------------------------------------------------------------
case "$STEP" in

inspect)
    log "--- Hardware and ROS inspection ---"
    run bash "$REPO/tools/inspect-jetson.sh"
    log "--- Done. Copy frame/topic names into config/roboracer/hardware.yaml ---"
    ;;

record)
    log "--- Recording a manual lap ---"
    log "Drive the car manually. Ctrl+C to stop recording."
    run python3 -m avlite_roboracer.recorder record \
        --profile "$PROFILE" \
        --output "$LOG_BASE"
    log "--- Exporting to records.jsonl ---"
    run python3 -m avlite_roboracer.recorder export \
        "$LOG_BASE" \
        --profile "$PROFILE"
    log "--- Recording complete: $LOG_BASE ---"
    ;;

check)
    REC="$(latest_recording)"
    log "--- Checking recording: $REC ---"
    REPORT_DIR="$LOG_BASE/report"
    mkdir -p "$REPORT_DIR"
    run python3 -m avlite_roboracer.report \
        "$REC/records.jsonl" \
        --profile "$PROFILE" \
        --output "$REPORT_DIR"
    log "--- Report written to $REPORT_DIR ---"
    log "Resolve every FAIL before proceeding to commissioning."
    ;;

commission)
    REC="$(latest_recording)"
    log "--- Analyzing stop tests from: $REC ---"
    STOPS_OUT="$LOG_BASE/stops.json"
    run python3 -m avlite_roboracer.commissioning \
        "$REC/records.jsonl" \
        --output "$STOPS_OUT"
    log "--- Commissioning results written to $STOPS_OUT ---"
    log "Review the values, then fill them into config/roboracer/hardware.yaml"
    log "under measurements:, limits:, vehicle:, mounts.laser, and steering."
    ;;

map)
    REC="$(latest_recording)"
    log "--- Replaying through SLAM Toolbox to build map: $REC ---"
    run python3 -m avlite_roboracer.recorder replay \
        "$REC" \
        --profile "$PROFILE" \
        --mode mapping
    MAP_REPLAY="$(find "$REC" -maxdepth 1 -name 'replay-mapping-*' -type d | sort | tail -1)"
    log "--- Map saved to: $MAP_REPLAY ---"
    log "Verify map.yaml exists and SLAM Toolbox closed the loop cleanly."
    ;;

localize)
    REC2="$(second_latest_recording)"
    MAP_REPLAY="$(latest_map)"
    POSEGRAPH="$(find "$MAP_REPLAY" -name 'posegraph*' | head -1)"
    if [[ -z "$POSEGRAPH" ]]; then
        log "ERROR: no posegraph file found in $MAP_REPLAY. Rerun the 'map' step."
        exit 1
    fi
    log "--- Localizing recording: $REC2 ---"
    log "    Against map: $MAP_REPLAY"
    log "    Pose graph:  $POSEGRAPH"
    run python3 -m avlite_roboracer.recorder replay \
        "$REC2" \
        --profile "$PROFILE" \
        --mode localization \
        --pose-graph "$POSEGRAPH"
    LOC_REPLAY="$(find "$REC2" -maxdepth 1 -name 'replay-localization-*' -type d | sort | tail -1)"
    run python3 -m avlite_roboracer.recorder export \
        "$LOC_REPLAY" \
        --profile "$PROFILE"
    REPORT_DIR="$LOG_BASE/report"
    mkdir -p "$REPORT_DIR"
    run python3 -m avlite_roboracer.report \
        "$LOC_REPLAY/records.jsonl" \
        --profile "$PROFILE" \
        --require-map-frame \
        --map "$(dirname "$MAP_REPLAY")/map.yaml" \
        --output "$REPORT_DIR"
    log "--- Localization report written to $REPORT_DIR ---"
    log "All thresholds must pass before autonomous mapping."
    ;;

launch)
    log "--- Starting ROS stack (no drive commands will be sent) ---"
    log "Stop with Ctrl+C. Operator commands come from the laptop via avlite.ps1."
    run ros2 launch avlite_roboracer roboracer.launch.py \
        "config_dir:=$REPO/config/roboracer"
    ;;

*)
    printf 'Unknown step: %s\n\n' "$STEP"
    usage
    ;;
esac

log "=== jetson-run.sh done: step=$STEP log=$LOG_BASE ==="
```
