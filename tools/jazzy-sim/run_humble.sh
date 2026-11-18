#!/usr/bin/env bash
# Run the car's full flow (map-start -> READY -> race-start -> stop) against the simulated F1TENTH
# car, on the working tree, in the pinned Humble image. No simulator window or hardware needed.
# Usage, from Git Bash at the repository root:  bash tools/jazzy-sim/run_humble.sh [run-name]
# Results land in tools/jazzy-sim/<run-name>/ (default closed-run; git-ignored). Takes ~8 minutes.
# Optional env: SLAM_OVERRIDES='{"param": value}', RACE_OVERRIDES='{"localization": {...}}' (merged
# into race.yaml for this run), RACE_SECONDS (default 90), CAR_ARGS (fake-car stress options,
# e.g. "--odom-scale 1.1 --yaw-bias 0.02 --scan-noise 0.03 --dropout 0.2").
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:-closed-run}
cat > "$ROOT/tools/jazzy-sim/.run_inside.sh" <<'INNER'
set -e
mkdir -p /tmp/ws /tmp/repo_rw && cd /tmp/ws
cp -r /repo/src /repo/config /tmp/repo_rw/ && cp -r /repo/src /tmp/src
source /opt/ros/humble/setup.bash
source /opt/avlite-venv/bin/activate
python -m colcon build --base-paths /tmp/src --packages-select avlite_autodrive avlite_roboracer 2>&1 | tail -3
export ROS_SETUP=/opt/ros/humble/setup.bash WS_SETUP=/tmp/ws/install/setup.bash REPO=/tmp/repo_rw
export PATH=/opt/avlite-venv/bin:$PATH OUT=/work/$RUN_NAME
if [[ -n "${RACE_OVERRIDES:-}" ]]; then
python - <<'PY'
import json, os, yaml
path = "/tmp/repo_rw/config/roboracer/race.yaml"
data = yaml.safe_load(open(path))
for key, value in json.loads(os.environ["RACE_OVERRIDES"]).items():
    if isinstance(value, dict):
        data.setdefault(key, {}).update(value)
    else:
        data[key] = value
open(path, "w").write(yaml.safe_dump(data, sort_keys=False))
PY
fi
bash /work/closed_loop.sh
INNER
MSYS_NO_PATHCONV=1 docker run --rm -e RUN_NAME="$NAME" -e SLAM_OVERRIDES -e RACE_OVERRIDES -e RACE_SECONDS -e CAR_ARGS \
    -v "$(cygpath -wa "$ROOT"):/repo:ro" -v "$(cygpath -wa "$ROOT/tools/jazzy-sim"):/work" \
    --entrypoint /bin/bash autodrive-roboracer-avlite:local /work/.run_inside.sh
