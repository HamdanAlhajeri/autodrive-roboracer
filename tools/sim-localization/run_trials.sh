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
BR=autodrive-roboracer-bridge-1
OUT="log/step-trials/$LABEL"
mkdir -p "$OUT"
export MSYS_NO_PATHCONV=1
COMPOSE=(docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml)
log() { echo "[$(date +%H:%M:%S)] $*"; }

docker cp tools/sim-localization/measure_pose.py "$AV:/tmp/measure_pose.py"
docker cp tools/sim-localization/respawn.py "$BR:/tmp/respawn.py"
log "odometry args: '${ODOM_ARGS}'"
SIM_ODOMETRY_ARGS="$ODOM_ARGS" "${COMPOSE[@]}" up -d --no-deps --force-recreate odometry >/dev/null 2>&1
sleep 5

for i in $(seq 1 "$TRIALS"); do
    log "trial $i/$TRIALS: respawn"
    # Stop AVLite first: left running it drives the car away before the odometry restarts, and the
    # odometry origin (and the localizer's spawn pose) would then be wrong.
    docker stop "$AV" >/dev/null
    docker exec "$BR" bash -c 'source /opt/ros/humble/setup.bash; python3 /tmp/respawn.py'
    sleep 2
    docker restart "$OD" >/dev/null
    sleep 4   # odometry takes its origin from the car at rest on the spawn point
    ready_before=$(docker logs "$AV" 2>&1 | grep -c "Sensors ready" || true)
    docker start "$AV" >/dev/null
    # The car starts driving as soon as the sensors are ready, so start recording right then.
    for _ in $(seq 1 120); do
        [ "$(docker logs "$AV" 2>&1 | grep -c "Sensors ready" || true)" -gt "$ready_before" ] && break
        sleep 0.5
    done
    log "trial $i/$TRIALS: recording ${SECONDS_PER} s"
    docker exec "$AV" bash -c "source /opt/ros/humble/setup.bash; /opt/avlite-venv/bin/python /tmp/measure_pose.py /tmp/trial.jsonl --seconds $SECONDS_PER --require-motion-s 25" | tail -1
    docker exec "$AV" test -s /tmp/trial.jsonl || true
    docker cp "$AV:/tmp/trial.jsonl" "$OUT/trial-$i.jsonl" 2>/dev/null || log "trial $i: no data"
done
log "done: $OUT"
