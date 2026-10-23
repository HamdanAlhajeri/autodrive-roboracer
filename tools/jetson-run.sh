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
    printf '  launch      Check F1TENTH bringup, then start the ROS stack (no drive commands)\n'
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

preflight() {
    # Read-only: confirm the F1TENTH drivers feed this stack before it starts.
    local names scan odom drive odom_frame base_frame laser_frame failed=0
    names="$(python3 -c "from avlite_roboracer.profile import HardwareProfile as H
p = H.load('$PROFILE')
print(p.topic('scan'), p.topic('odom'), p.topic('drive_output'),
      p.frame('odom'), p.frame('base_link'), p.frame('laser'))")"
    read -r scan odom drive odom_frame base_frame laser_frame <<< "$names"
    for topic in "$scan" "$odom"; do
        if timeout 5 ros2 topic echo --once "$topic" >/dev/null 2>&1; then
            log "ok: $topic is publishing"
        else
            log "FAIL: no message on $topic. Is f1tenth_stack bringup running?"; failed=1
        fi
    done
    # Capture output before matching: with pipefail, a timed-out producer piped into
    # grep would fail the test even when the text was found.
    local out
    out="$(ros2 topic info "$drive" 2>/dev/null || true)"
    if grep -Eq 'Subscription count: [1-9]' <<< "$out"; then
        log "ok: $drive has a subscriber"
    else
        log "FAIL: nothing subscribes to $drive. Start ackermann_mux (f1tenth_stack bringup)."; failed=1
    fi
    for pair in "$odom_frame $base_frame" "$base_frame $laser_frame"; do
        # shellcheck disable=SC2086
        out="$(timeout 4 ros2 run tf2_ros tf2_echo $pair 2>/dev/null || true)"
        if grep -q 'Translation' <<< "$out"; then
            log "ok: transform ${pair/ /->} resolves"
        else
            log "FAIL: transform ${pair/ /->} is missing. Check vesc_to_odom and the static laser TF."; failed=1
        fi
    done
    if (( failed )); then
        log "Preflight failed; the car stack was not started."
        exit 1
    fi
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
    log "--- Preflight: F1TENTH sensors, drive mux and transforms ---"
    preflight
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
