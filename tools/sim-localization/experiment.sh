#!/usr/bin/env bash
# Simulator mapping + SLAM localization experiment, entirely in Docker (headless AutoDRIVE).
#
#   bash tools/sim-localization/experiment.sh [speed_mps] [laps]
#
# 1. Start the headless simulator, bridge, actuator and wheel odometry.
# 2. Mapping run: drive planned laps on ground truth at a slow speed while recording the
#    SLAM inputs (sim_profile topics) and, separately, ground truth for calibration.
# 3. Build the SLAM map offline (recorder replay --mode mapping).
# 4. Localization run: reset, then drive the same planned algorithm on SLAM poses.
# 5. Baseline run: the same on ground truth. Then summarize both.
# MAP_FROM=log/sim-localization/<stamp> skips steps 2-3 and reuses that run's SLAM map.
# Results land in log/sim-localization/<stamp>/ (summary.json, summary.md).
set -euo pipefail
SPEED="${1:-2.5}"
LAPS="${2:-3}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="log/sim-localization/$STAMP"
mkdir -p "$OUT"
export MSYS_NO_PATHCONV=1
win() { if command -v cygpath >/dev/null; then cygpath -wa "$1"; else realpath "$1"; fi; }
W_REPO="$(win "$REPO")"; W_OUT="$(win "$OUT")"
IMAGE=autodrive-roboracer-avlite:local
# SIM=windows drives the visible native Windows simulator (.\avlite.ps1 stack, then select
# Connection and Autonomous in its window); ROS containers share the bridge's network.
# Otherwise the headless Linux simulator runs in Docker on the host network.
export COMPOSE_PROJECT_NAME=autodrive-roboracer  # same containers from any worktree
if [[ "${SIM:-docker}" == windows ]]; then
    COMPOSE=(docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml)
    SERVICES=(bridge actuator odometry)
    NETWORK=container:autodrive-roboracer-bridge-1
else
    COMPOSE=(docker compose -f docker-compose.avlite.yml)
    SERVICES=(simulator bridge actuator odometry)
    NETWORK=host
fi
log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$OUT/experiment.log"; }

# Run a command in the AVLite image on the stack's ROS network (domain 0, like the stack).
ros() {
    local name="$1"; shift
    local detach=() run="$*"
    # A named container runs one command: exec it, so it (not bash as PID 1, which ignores
    # SIGINT) receives stop()'s Ctrl+C and shuts down cleanly (bags close, SLAM stops).
    [[ "$name" != "-" ]] && detach=(-d --name "$name") && run="exec $*"
    docker run --rm "${detach[@]}" --network "$NETWORK" --ipc host -e ROS_DOMAIN_ID=0 \
        -e PYTHONPATH=/opt/integration:/opt/roboracer \
        -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
        -v "$W_REPO\\config:/config:ro" -v "$W_REPO\\src\\avlite_autodrive:/opt/integration:ro" \
        -v "$W_REPO\\src\\avlite_roboracer:/opt/roboracer:ro" -v "$W_OUT:/out" \
        --entrypoint bash "$IMAGE" -c "source /opt/ros/humble/setup.bash && $run"
}
# Ctrl+C each container, then give it 20 s to exit before docker stops it outright.
stop() { for c in "$@"; do docker kill -s INT "$c" >/dev/null 2>&1 || true; done
         for c in "$@"; do timeout 20 docker wait "$c" >/dev/null 2>&1              || docker stop -t 2 "$c" >/dev/null 2>&1 || true; done; }
cleanup() { stop rr-avlite rr-rec rr-truth rr-diag rr-laps; }
trap cleanup EXIT
# Helpers left by an interrupted run would block their names.
docker rm -f rr-avlite rr-rec rr-truth rr-diag rr-laps >/dev/null 2>&1 || true

topic_value() {  # topic_value TOPIC FIELD -> one number or empty (never a ROS warning)
    ros - "timeout 5 ros2 topic echo --once --field $2 $1 2>/dev/null" | tr -d '\r' \
        | grep -m1 -E '^-?[0-9]+([.][0-9]+)?$' || true
}

reset_sim() {
    log "Resetting the simulator"
    # The bridge holds the last flag: true re-spawns the car every frame until false.
    ros - "ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool '{data: true}' >/dev/null
           sleep 1; ros2 topic pub --once /autodrive/reset_command std_msgs/msg/Bool '{data: false}' >/dev/null"
    sleep 2
    "${COMPOSE[@]}" restart odometry >/dev/null  # odometry restarts at the spawn pose
    sleep 3
}

# write_config NAME SOURCE [localization yaml lines...]
write_config() {
    local name="$1" source="$2" extra="${3:-}"
    ros - "python3 - <<'PY'
import yaml
cfg = yaml.safe_load(open('/config/avlite.yaml'))
# The planner ceiling and control cruise speed both come from the shared speed_mps.
driving = yaml.safe_load(open('/config/driving.yaml'))
driving['speed_mps'] = float($SPEED)
yaml.safe_dump(driving, open('/out/driving.yaml', 'w'))
cfg['shared_settings'] = '/out/driving.yaml'
cfg['driving_mode'] = 'planned'
cfg['planning']['map_path'] = '/config/maps/practice.json'
loc = {'source': '$source', 'profile': '/config/roboracer/sim_profile.yaml',
       'slam_params': '/config/roboracer/slam_toolbox.yaml', 'log_dir': '/out/slam'}
loc.update(yaml.safe_load('''$extra''') or {})
cfg['localization'] = loc
yaml.safe_dump(cfg, open('/out/$name.yaml', 'w'), sort_keys=False)
PY"
}

# drive_laps CONFIG LABEL: run the planned runner until LAPS more laps or a collision.
drive_laps() {
    local config="$1" label="$2"
    local start_lap start_col lap col deadline
    start_lap="$(topic_value /autodrive/roboracer_1/lap_count data)"; start_lap="${start_lap:-0}"
    start_col="$(topic_value /autodrive/roboracer_1/collision_count data)"; start_col="${start_col:-0}"
    log "$label: driving $LAPS laps at up to $SPEED m/s (lap counter $start_lap)"
    ros rr-diag "ros2 topic echo --field data /avlite/controller_diagnostics std_msgs/msg/String > /out/$label-diagnostics.txt" >/dev/null
    ros rr-laps "ros2 topic echo --field data /autodrive/roboracer_1/last_lap_time std_msgs/msg/Float32 > /out/$label-lap-times.txt" >/dev/null
    ros rr-avlite "python -m avlite_autodrive.runner --config /out/$config.yaml > /out/$label-runner.log 2>&1" >/dev/null
    deadline=$((SECONDS + 120 + 60 * LAPS))
    while (( SECONDS < deadline )); do
        sleep 5
        lap="$(topic_value /autodrive/roboracer_1/lap_count data)"; lap="${lap:-$start_lap}"
        col="$(topic_value /autodrive/roboracer_1/collision_count data)"; col="${col:-$start_col}"
        if (( col > start_col )); then log "$label: COLLISION after $((lap - start_lap)) laps"; break; fi
        if (( lap - start_lap >= LAPS )); then log "$label: $LAPS laps done"; break; fi
        docker inspect rr-avlite >/dev/null 2>&1 || { log "$label: runner exited"; break; }
    done
    (( SECONDS < deadline )) || log "$label: TIMEOUT after $((lap - start_lap)) laps"
    echo "{\"laps\": $((lap - start_lap)), \"collisions\": $((col - start_col))}" > "$OUT/$label-counters.json"
    stop rr-avlite rr-diag rr-laps
}

log "Experiment $STAMP: speed $SPEED m/s, $LAPS laps per run"
log "Starting ${SERVICES[*]} (SIM=${SIM:-docker})"
"${COMPOSE[@]}" up -d "${SERVICES[@]}" >/dev/null
for _ in $(seq 1 30); do
    [[ -n "$(topic_value /autodrive/roboracer_1/lap_count data)" ]] && break
    sleep 5
done
[[ -n "$(topic_value /autodrive/roboracer_1/lap_count data)" ]] || { log "Simulator data never arrived"; exit 1; }
reset_sim

# --- AVLite's own localization (LOCALIZER=avlite): no mapping run or SLAM map needed ---------
if [[ "${LOCALIZER:-slam}" == avlite ]]; then
    reset_sim
    sleep 3  # the restarted odometry logs the spawn pose once it has an encoder sample
    ORIGIN="$("${COMPOSE[@]}" logs odometry 2>/dev/null \
        | grep -o 'map_origin_world for a mapping run): \[[^]]*\]' | tail -1 | sed 's/.*: //')"
    [[ -n "$ORIGIN" ]] || { log "Spawn pose not logged by odometry"; exit 1; }
    echo "$ORIGIN" > "$OUT/map_origin_world.txt"
    log "AVLite LidarLocalization from spawn pose $ORIGIN"
    write_config avlite avlite "start_pose_world: $ORIGIN"
    drive_laps avlite avlite
else

# --- Mapping run (skipped with MAP_FROM=<earlier experiment dir>) ---------------------------
if [[ -n "${MAP_FROM:-}" ]]; then
    SOURCE_MAP="$(ls -d "$MAP_FROM"/mapping/replay-mapping-* | tail -1)"
    MAPDIR="$OUT/mapping/$(basename "$SOURCE_MAP")"
    mkdir -p "$MAPDIR"
    cp "$SOURCE_MAP"/map.yaml "$SOURCE_MAP"/map.pgm "$SOURCE_MAP"/posegraph.* "$MAPDIR"/
    cp "$MAP_FROM/map_origin_world.txt" "$OUT/"
    ORIGIN="$(cat "$OUT/map_origin_world.txt")"
    log "Reusing SLAM map $SOURCE_MAP (origin in world ${ORIGIN:-unknown})"
else
write_config mapping ground_truth
ros rr-rec "python -m avlite_roboracer.recorder record --profile /config/roboracer/sim_profile.yaml --output /out/mapping --purpose 'sim mapping run'" >/dev/null
ros rr-truth "ros2 bag record -o /out/truth /autodrive/roboracer_1/odom /autodrive/roboracer_1/wheel_odom" >/dev/null
sleep 4
drive_laps mapping mapping
stop rr-rec rr-truth
"${COMPOSE[@]}" logs odometry 2>/dev/null | grep -o 'map_origin_world for a mapping run): \[[^]]*\]' | tail -1 \
    | sed 's/.*: //' > "$OUT/map_origin_world.txt" || true
ORIGIN="$(cat "$OUT/map_origin_world.txt")"
log "Mapping odometry origin in world: ${ORIGIN:-unknown}"
ros - "python -m avlite_roboracer.recorder export /out/mapping --profile /config/roboracer/sim_profile.yaml > /out/mapping-export.log 2>&1;
       python -m avlite_roboracer.report /out/mapping/records.jsonl --profile /config/roboracer/sim_profile.yaml --output /out/mapping/report > /out/mapping-report.log 2>&1 || true"
log "Building the SLAM map from the mapping run"
ros - "python -m avlite_roboracer.recorder replay /out/mapping --profile /config/roboracer/sim_profile.yaml --mode mapping --slam-params /config/roboracer/slam_toolbox.yaml > /out/replay-mapping.log 2>&1"
MAPDIR="$(ls -d "$OUT"/mapping/replay-mapping-* | tail -1)"
fi
[[ -f "$MAPDIR/map.yaml" ]] || { log "Mapping replay produced no map"; exit 1; }
MAPDIR_C="/out/mapping/$(basename "$MAPDIR")"
log "SLAM map: $MAPDIR"

# --- Localization run ----------------------------------------------------------------------
reset_sim
write_config slam slam "pose_graph: $MAPDIR_C/posegraph
map_yaml: $MAPDIR_C/map.yaml
start_pose: [0.0, 0.0, 0.0]
map_origin_world: ${ORIGIN:-null}"
drive_laps slam slam
fi  # LOCALIZER

# --- Ground-truth baseline (skipped with SKIP_BASELINE=1) -------------------------------------
if [[ -z "${SKIP_BASELINE:-}" ]]; then
    reset_sim
    write_config baseline ground_truth
    drive_laps baseline baseline
fi

log "Summarizing"
docker run --rm -v "$W_OUT:/out" -v "$W_REPO\\tools\\sim-localization:/tools:ro" --entrypoint python3 "$IMAGE" /tools/summarize.py /out
log "Done: $OUT/summary.md"
