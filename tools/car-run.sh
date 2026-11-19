#!/usr/bin/env bash
# Map the track slowly for a few laps, then race it. Run ON THE JETSON with an operator at the car.
#
#   bash tools/car-run.sh --start-stack                  # asks before each motion step
#   bash tools/car-run.sh --start-stack --yes --laps 3   # an agent running it: the operator is at the car
#   bash tools/car-run.sh --map-only                     # map and validate, do not race
#   bash tools/car-run.sh --race-only --race-seconds 60  # the map is already READY
#
# What it does, in order (each step stops the whole run if it fails):
#   1. preflight: tools/car-day.sh (install, bringup topics, hardware profile commissioned)
#   2. optionally starts the F1TENTH bringup and the supervisor (--start-stack)
#   3. waits until the operator holds the joystick dead-man button
#   4. MAPPING: the car drives --laps slow laps by itself (Follow the Gap + SLAM Toolbox) and stops
#   5. FINALIZING: builds and checks the map, plan and localization, then reaches READY or STOPPED
#   6. RACING for --race-seconds (default 60), then stops
# The car only moves while the dead-man button is held and this script keeps running. Ctrl+C stops it.
# SSH loss is not a physical stop: keep the emergency stop in hand.
#
# Exit codes: 0 done, 1 failed or stopped, 2 setup problem (nothing was sent to the car), 130 interrupted.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLI="${ROBORACER_CLI:-$REPO/scripts/jetson/roboracer}"
CAR_DAY="${CAR_DAY:-$REPO/tools/car-day.sh}"
RACE_YAML="${ROBORACER_RACE_YAML:-$REPO/config/roboracer/race.yaml}"

LAPS="" RACE_SECONDS=60 MAP_TIMEOUT=900 FINALIZE_TIMEOUT=300 DEADMAN_WAIT="${CAR_RUN_DEADMAN_WAIT:-60}"
YES=0 START_STACK=0 MAP_ONLY=0 RACE_ONLY=0 REMAP=0 PREFLIGHT=1 SESSION=""

usage() { sed -n '2,/^set -uo/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//'; }
while [[ $# -gt 0 ]]; do
    case "$1" in
        --laps) LAPS="${2:?--laps needs a number}"; shift 2 ;;
        --race-seconds) RACE_SECONDS="${2:?}"; shift 2 ;;
        --map-timeout) MAP_TIMEOUT="${2:?}"; shift 2 ;;
        --finalize-timeout) FINALIZE_TIMEOUT="${2:?}"; shift 2 ;;
        --session) SESSION="${2:?}"; shift 2 ;;
        --yes) YES=1; shift ;;
        --start-stack) START_STACK=1; shift ;;
        --map-only) MAP_ONLY=1; shift ;;
        --race-only) RACE_ONLY=1; shift ;;
        --remap) REMAP=1; shift ;;
        --no-preflight) PREFLIGHT=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 2 ;;
    esac
done
[[ "$MAP_ONLY" == 1 && "$RACE_ONLY" == 1 ]] && { echo "--map-only and --race-only cannot be combined"; exit 2; }
for n in "$RACE_SECONDS" "$MAP_TIMEOUT" "$FINALIZE_TIMEOUT"; do
    [[ "$n" =~ ^[0-9]+$ && "$n" -gt 0 ]] || { echo "Times must be whole seconds above 0 (got '$n')"; exit 2; }
done
[[ -z "$LAPS" || "$LAPS" =~ ^[1-9][0-9]?$ ]] || { echo "--laps must be a whole number from 1 to 99"; exit 2; }

LOGDIR="${CAR_RUN_LOG:-$REPO/log/car-run/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$LOGDIR"
exec > >(tee -a "$LOGDIR/run.log") 2>&1

STACK_PIDS=()
say()  { printf '\n== %s\n' "$*"; }
note() { printf '   %s\n' "$*"; }
die()  { local code="$1"; shift; printf '\nSTOPPED: %s\n' "$*"; printf 'Log: %s\n' "$LOGDIR/run.log"; exit "$code"; }

cleanup() {
    local code=$?
    trap - EXIT INT TERM
    if [[ -n "${SENT_MOTION:-}" ]]; then bash "$CLI" stop >/dev/null 2>&1; fi
    for pid in "${STACK_PIDS[@]:-}"; do
        [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null
    done
    exit "$code"
}
trap cleanup EXIT
trap 'echo; echo "Interrupted: stopping the car"; exit 130' INT TERM

# jget 'actuator.deadman_held'  -> value from the supervisor's status JSON (empty if absent)
jget() {
    bash "$CLI" status --json 2>/dev/null | python3 -c '
import json, sys
try:
    value = json.load(sys.stdin)
    for key in sys.argv[1].split("."):
        value = value.get(key) if isinstance(value, dict) else None
    print("" if value is None else (json.dumps(value) if isinstance(value, (list, dict, bool)) else value))
except Exception:
    pass' "$1"
}
status_reachable() { bash "$CLI" status --json >/dev/null 2>&1; }

confirm() {
    [[ "$YES" == 1 ]] && { note "(--yes) $1"; return 0; }
    if [[ ! -t 0 ]]; then
        die 2 "this step moves the car and needs a person's confirmation. Run it in a terminal, or pass --yes if an operator is at the car with the emergency stop and the dead-man button."
    fi
    printf '\n%s\nType GO to continue, anything else to cancel: ' "$1"
    local answer; read -r answer
    [[ "$answer" == "GO" ]] || die 1 "cancelled; nothing was sent to the car."
}

source_ros() {
    set +u
    local f
    for f in /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash \
             "$HOME/f1tenth_ws/install/setup.bash" "$HOME/roboracer_ws/install/setup.bash"; do
        # shellcheck disable=SC1090
        [[ -f "$f" ]] && source "$f"
    done
    set -u
}

start_stack() {
    source_ros
    command -v ros2 >/dev/null 2>&1 || die 2 "ros2 is not on PATH; run tools/jetson-setup.sh and source ROS."
    say "Starting the F1TENTH bringup and the supervisor (logs in $LOGDIR)"
    setsid ros2 launch f1tenth_stack bringup_launch.py >"$LOGDIR/bringup.log" 2>&1 &
    STACK_PIDS+=($!)
    local i
    for i in $(seq 1 60); do
        local topics; topics="$(timeout 5 ros2 topic list 2>/dev/null)"
        grep -qx /scan <<<"$topics" && grep -qx /odom <<<"$topics" && break
        sleep 1
    done
    grep -qx /scan <<<"${topics:-}" || die 2 "the bringup did not publish /scan and /odom within 60 s. See $LOGDIR/bringup.log"
    note "bringup is publishing /scan and /odom"
    setsid bash "$REPO/tools/jetson-run.sh" launch >"$LOGDIR/supervisor.log" 2>&1 &
    STACK_PIDS+=($!)
    for i in $(seq 1 90); do status_reachable && break; sleep 1; done
    status_reachable || die 2 "the supervisor did not answer within 90 s. See $LOGDIR/supervisor.log"
    note "supervisor is up"
}

print_status() { bash "$CLI" status 2>&1 | sed 's/^/   /'; }

set_laps() {
    [[ -z "$LAPS" ]] && return 0
    [[ -f "$RACE_YAML" ]] || die 2 "$RACE_YAML not found"
    if ! grep -qE '^mapping:' "$RACE_YAML"; then die 2 "no mapping: block in $RACE_YAML"; fi
    sed -i -E '/^mapping:/,/^[^ #]/ s/^(  laps:)[[:space:]]*[0-9]+/\1 '"$LAPS"'/' "$RACE_YAML"
    grep -qE "^  laps: $LAPS\b" "$RACE_YAML" || die 2 "could not set mapping.laps in $RACE_YAML"
    note "mapping.laps = $LAPS in $RACE_YAML (the supervisor reads it when it starts)"
    [[ "$START_STACK" == 1 ]] || note "restart the supervisor for this to apply if it is already running"
}

# ---------------------------------------------------------------- 1. preflight
say "1/6 Preflight"
if [[ "$PREFLIGHT" == 1 ]]; then
    if bash "$CAR_DAY" >"$LOGDIR/preflight.log" 2>&1; then
        note "car-day.sh: everything is in place"
    else
        sed 's/^/   /' "$LOGDIR/preflight.log" | tail -45
        die 2 "preflight failed (see above). Fix the [FAIL] lines and the missing profile values, then run again. Nothing was sent to the car."
    fi
else
    note "preflight skipped (--no-preflight)"
fi
set_laps

# ---------------------------------------------------------------- 2. stack
say "2/6 Supervisor"
if ! status_reachable; then
    if [[ "$START_STACK" == 1 ]]; then
        start_stack
    else
        cat <<EOF
   The supervisor is not running. In two other terminals on the Jetson:
     ros2 launch f1tenth_stack bringup_launch.py
     bash tools/jetson-run.sh launch
   or run this script again with --start-stack.
EOF
        die 2 "no supervisor to talk to. Nothing was sent to the car."
    fi
fi
print_status
STATE="$(jget state)"

# ---------------------------------------------------------------- 3. dead-man
wait_for_deadman() {
    local i
    [[ "$(jget actuator.deadman_held)" == "true" ]] && return 0
    note "Hold the joystick dead-man button now (release it at any time to stop the car)."
    for i in $(seq 1 "$DEADMAN_WAIT"); do
        [[ "$(jget actuator.deadman_held)" == "true" ]] && { note "dead-man held"; return 0; }
        sleep 1
    done
    die 1 "the dead-man button was not held within ${DEADMAN_WAIT} s. Check the button index in hardware.yaml (deadman.button) and that /joy publishes."
}

# ---------------------------------------------------------------- 4/5. mapping
if [[ "$RACE_ONLY" != 1 ]]; then
    if [[ "$STATE" == READY && "$REMAP" != 1 && "$MAP_ONLY" != 1 ]]; then
        note "a validated map is already READY: skipping mapping (use --remap to map again)"
    else
        [[ "$STATE" == READY || "$STATE" == RACING ]] && { bash "$CLI" stop >/dev/null 2>&1; sleep 1; STATE="$(jget state)"; }
        say "3/6 Dead-man button"
        confirm "The car will now drive ${LAPS:-the configured number of} SLOW laps by itself while it maps. Someone must stand at the car with the emergency stop and hold the dead-man button the whole time. The track must be clear."
        wait_for_deadman
        say "4/6 Mapping (slow laps, then the car stops by itself)"
        SENT_MOTION=1
        timeout --signal=INT "$MAP_TIMEOUT" bash "$CLI" map-start ${SESSION:+--session "$SESSION"}
        rc=$?
        if [[ $rc -eq 124 ]]; then die 1 "mapping did not finish within ${MAP_TIMEOUT} s; stopped. Check the car's speed limit (limits.mapping_speed_mps) and that it can complete a lap."; fi
        if [[ $rc -ne 0 && $rc -ne 130 ]]; then
            print_status; die 1 "mapping was refused or ended with exit code $rc (see above)."
        fi
        say "5/6 Finalizing (map, plan and localization are built and checked)"
        deadline=$((SECONDS + FINALIZE_TIMEOUT))
        while :; do
            STATE="$(jget state)"
            [[ "$STATE" == READY || "$STATE" == STOPPED ]] && break
            (( SECONDS > deadline )) && die 1 "finalizing took more than ${FINALIZE_TIMEOUT} s (state $STATE, step $(jget finalize_step))."
            sleep 2
        done
        SENT_MOTION=""
        print_status
        if [[ "$STATE" != READY ]]; then
            die 1 "mapping did not reach READY. The reason is listed above (also in $LOGDIR/run.log); see HANDOVER.md, section 'When it stops'."
        fi
        note "map validated: the car is READY to race"
    fi
fi
[[ "$MAP_ONLY" == 1 ]] && { say "Done: map is READY (--map-only)"; note "session: $(jget session_dir)"; exit 0; }

# ---------------------------------------------------------------- 6. race
STATE="$(jget state)"
[[ "$STATE" == READY ]] || die 1 "not READY (state $STATE): nothing to race. Map first (run without --race-only)."
say "6/6 Racing for ${RACE_SECONDS} s"
confirm "The car will now RACE the mapped track for ${RACE_SECONDS} s at limits.racing_speed_mps. Stay at the car with the emergency stop; hold the dead-man button; keep the track clear."
wait_for_deadman
SENT_MOTION=1
timeout --signal=INT "$RACE_SECONDS" bash "$CLI" race-start ${SESSION:+--session "$SESSION"}
rc=$?
STATE_AFTER="$(jget state)"; REASONS_AFTER="$(jget reasons)"
bash "$CLI" stop >/dev/null 2>&1
SENT_MOTION=""
case $rc in
    124|130) note "race time reached: car stopped" ;;
    0)
        if [[ "$STATE_AFTER" == STOPPED ]]; then
            print_status; die 1 "the supervisor stopped the race before the time was up: ${REASONS_AFTER:-no reason given}"
        fi
        note "the supervisor ended the race before the time was up" ;;
    *) print_status; die 1 "racing was refused or ended with exit code $rc (see above)." ;;
esac
say "Finished"
print_status
note "session folder: $(jget session_dir)"
note "run log: $LOGDIR/run.log"
exit 0
