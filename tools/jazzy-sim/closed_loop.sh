#!/usr/bin/env bash
# Full autonomous flow on the simulated car: launch -> map-start -> READY -> race-start -> stop.
set -eo pipefail
set -m
source /opt/ros/jazzy/setup.bash
source /opt/avlite-venv/bin/activate
source ~/roboracer_ws/install/setup.bash
set -u
REPO=~/Desktop/autodrive-roboracer
# Use the working-tree sources mounted at /repo-src, if any.
[[ -d /repo-src ]] && cp -r /repo-src/. "$REPO/src/"
OUT=/work/closed
rm -rf "$OUT"; mkdir -p "$OUT/cfg"
export ROS_DOMAIN_ID=43
cp "$REPO"/config/roboracer/*.yaml "$OUT/cfg/"
PYTHONPATH="$REPO/src/avlite_roboracer/test:${PYTHONPATH:-}" python - "$OUT/cfg/hardware.yaml" <<'EOF'
import sys, yaml
from roboracer_fixtures import commissioned_data
open(sys.argv[1], "w").write(yaml.safe_dump(commissioned_data(), sort_keys=False))
EOF
CLI="python -m avlite_roboracer.cli"

python /work/fake_car.py --mode drive --joy-button 5 --truth "$OUT/truth.csv" > "$OUT/car.log" 2>&1 &
CAR=$!
ros2 launch avlite_roboracer roboracer.launch.py "config_dir:=$OUT/cfg" > "$OUT/stack.log" 2>&1 &
STACK=$!
cleanup() { kill -INT $STACK $CAR 2>/dev/null || true; sleep 3; kill -9 $STACK $CAR 2>/dev/null || true; }
trap cleanup EXIT

wait_state() {  # wait_state "READY|STOPPED" timeout_s
    local want=$1 limit=$2 t=0 state
    while (( t < limit )); do
        state=$($CLI status --json 2>/dev/null | python -c 'import json,sys; print(json.load(sys.stdin)["state"])' || echo none)
        $CLI status --json >> "$OUT/status.jsonl" 2>/dev/null || true
        if [[ "$state" =~ ^($want)$ ]]; then echo "$state"; return 0; fi
        sleep 2; t=$((t + 2))
    done
    echo "timeout($state)"; return 1
}

echo "=== waiting for IDLE"; wait_state IDLE 60
$CLI status
echo "=== race-start before mapping (must be rejected)"
$CLI race-start && echo "UNEXPECTED: accepted" || echo "rejected as expected"

echo "=== map-start"
$CLI map-start > "$OUT/map-start.log" 2>&1 &
MAP=$!
wait_state "READY|STOPPED" 400 || true
wait $MAP || true
cat "$OUT/map-start.log"
$CLI status

if [[ "$($CLI status --json | python -c 'import json,sys; print(json.load(sys.stdin)["state"])')" == READY ]]; then
    echo "=== race-start"
    $CLI race-start > "$OUT/race-start.log" 2>&1 &
    RACE=$!
    sleep 90
    $CLI status
    echo "=== stop"
    $CLI stop
    wait $RACE || true
    cat "$OUT/race-start.log"
    sleep 2
    $CLI status
fi
echo "=== done"
