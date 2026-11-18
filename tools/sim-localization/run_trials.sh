#!/usr/bin/env bash
# Independent trials for comparing two settings fairly. Each trial respawns the car, restarts the
# odometry and the AVLite runner (so the localizer is seeded at the spawn pose, which sidesteps the
# lost-after-reset problem), then records for SECONDS with measure_pose.py (no automatic reset).
#
# Usage, from Git Bash at the repository root (the stack must be running, Autonomous selected):
#   bash tools/sim-localization/run_trials.sh LABEL [TRIALS=6] [SECONDS=40] [SIM_ODOMETRY_ARGS]
# Example: bash tools/sim-localization/run_trials.sh before 6 40 "--clock-rate 1"
# Results: log/step-trials/LABEL/trial-N.jsonl; summarize with summarize_trials.py.
set -euo pipefail
LABEL="${1:?label}"
TRIALS="${2:-6}"
SECONDS_PER="${3:-40}"
ODOM_ARGS="${4:-}"
AV=autodrive-roboracer-avlite-1
OD=autodrive-roboracer-odometry-1
OUT="log/step-trials/$LABEL"
mkdir -p "$OUT"
export MSYS_NO_PATHCONV=1
COMPOSE=(docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml)
log() { echo "[$(date +%H:%M:%S)] $*"; }

docker cp tools/sim-localization/measure_pose.py "$AV:/tmp/measure_pose.py"
log "odometry args: '${ODOM_ARGS}'"
SIM_ODOMETRY_ARGS="$ODOM_ARGS" "${COMPOSE[@]}" up -d --no-deps --force-recreate odometry >/dev/null 2>&1
sleep 5

for i in $(seq 1 "$TRIALS"); do
    log "trial $i/$TRIALS: respawn"
    docker exec "$AV" bash -c 'source /opt/ros/humble/setup.bash
        ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool "{data: true}" >/dev/null
        sleep 1
        ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool "{data: false}" >/dev/null'
    sleep 3
    docker restart "$OD" >/dev/null
    docker restart "$AV" >/dev/null
    sleep 14   # planner start-up, sensors ready, localizer seeded at the spawn pose
    log "trial $i/$TRIALS: recording ${SECONDS_PER} s"
    docker exec "$AV" bash -c "source /opt/ros/humble/setup.bash; /opt/avlite-venv/bin/python /tmp/measure_pose.py /tmp/trial.jsonl --seconds $SECONDS_PER" | tail -1
    docker cp "$AV:/tmp/trial.jsonl" "$OUT/trial-$i.jsonl"
done
log "done: $OUT"
