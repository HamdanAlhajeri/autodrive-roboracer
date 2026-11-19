#!/usr/bin/env bash
# Cut the sensors for GAP seconds in the middle of a lap and record what the localizer does after.
# The bridge container is paused, which stops /odom and /lidar like a USB or network dropout.
# Usage (stack driving, Autonomous selected):  bash tools/sim-localization/gap_test.sh LABEL [TRIALS=5] [GAP=1.5]
set -uo pipefail
LABEL="${1:?label}"; TRIALS="${2:-5}"; GAP="${3:-1.5}"
AV=autodrive-roboracer-avlite-1; BR=autodrive-roboracer-bridge-1
OUT="log/step2/$LABEL"; mkdir -p "$OUT"; export MSYS_NO_PATHCONV=1
docker cp tools/sim-localization/respawn.py "$BR:/tmp/respawn.py"
docker cp tools/sim-localization/diag_log.py "$AV:/tmp/diag_log.py"
for i in $(seq 1 "$TRIALS"); do
    docker exec "$BR" bash -c 'source /opt/ros/humble/setup.bash; python3 /tmp/respawn.py' >/dev/null
    sleep 6   # driving a lap, well away from the spawn point
    docker exec -d "$AV" bash -c "source /opt/ros/humble/setup.bash; python3 /tmp/diag_log.py /tmp/gap.jsonl 14"
    sleep 2
    docker pause "$BR" >/dev/null; sleep "$GAP"; docker unpause "$BR" >/dev/null
    echo "[$(date +%H:%M:%S)] trial $i: sensors cut for ${GAP} s"
    sleep 13
    docker cp "$AV:/tmp/gap.jsonl" "$OUT/trial-$i.jsonl"
done
python tools/sim-localization/gap_summary.py "$OUT" "$GAP"
