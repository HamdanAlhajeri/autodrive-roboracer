#!/usr/bin/env bash
# Offline mapping/localization pipeline on the simulated car, as docs/jetson.md step 4.
set -eo pipefail
set -m  # background jobs must receive SIGINT like a terminal Ctrl+C
source /opt/ros/jazzy/setup.bash
source /opt/avlite-venv/bin/activate
source ~/roboracer_ws/install/setup.bash
set -u
REPO=~/Desktop/autodrive-roboracer
# Use the working-tree sources mounted at /repo-src, if any.
[[ -d /repo-src ]] && cp -r /repo-src/. "$REPO/src/"
OUT=/work/out
rm -rf "$OUT"; mkdir -p "$OUT"
export ROS_DOMAIN_ID=42
cd "$REPO"

# Test profile: the commissioned values used by the unit tests, never real measurements.
P="$OUT/test-profile.yaml"
PYTHONPATH="$REPO/src/avlite_roboracer/test:${PYTHONPATH:-}" python - "$P" <<'EOF'
import sys, yaml
from roboracer_fixtures import commissioned_data
open(sys.argv[1], "w").write(yaml.safe_dump(commissioned_data(), sort_keys=False))
EOF

record_run() {
    local name=$1 seed=$2
    python -m avlite_roboracer.recorder record --profile "$P" --output "$OUT/$name" &
    local rec=$!
    sleep 3
    python /work/fake_car.py --mode scripted --laps 1.25 --truth "$OUT/$name-truth.csv" --seed "$seed"
    sleep 1
    kill -INT "$rec"; wait "$rec" || true
    python -m avlite_roboracer.recorder export "$OUT/$name" --profile "$P"
    python -m avlite_roboracer.report "$OUT/$name/records.jsonl" --profile "$P" \
        --output "$OUT/$name/report" || echo "REPORT EXIT $? for $name"
}

echo "=== record manual-1"; record_run manual-1 1
echo "=== record manual-2"; record_run manual-2 2

echo "=== replay mapping"
python -m avlite_roboracer.recorder replay "$OUT/manual-1" --profile "$P" --mode mapping
MAPDIR=$(ls -d "$OUT"/manual-1/replay-mapping-* | tail -1)
ls -la "$MAPDIR"

echo "=== replay localization"
python -m avlite_roboracer.recorder replay "$OUT/manual-2" --profile "$P" --mode localization \
    --pose-graph "$MAPDIR/posegraph"
LOCDIR=$(ls -d "$OUT"/manual-2/replay-localization-* | tail -1)
python -m avlite_roboracer.recorder export "$LOCDIR" --profile "$P"
python -m avlite_roboracer.report "$LOCDIR/records.jsonl" --profile "$P" --require-map-frame \
    --map "$MAPDIR/map.yaml" --reference "$OUT/manual-2-truth.csv" \
    --output "$OUT/localization-report" || echo "REPORT EXIT $? for localization"
echo "=== done"
