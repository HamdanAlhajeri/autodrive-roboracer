#!/usr/bin/env bash
# Run the roboracer and autodrive unit tests in the pinned Humble image (no simulator needed).
# Usage, from Git Bash at the repository root:  bash tools/jazzy-sim/unit.sh [pytest args or test paths]
# Default: both packages. Takes about a minute.
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
ARGS=${*:-}
MSYS_NO_PATHCONV=1 docker run --rm -v "$(cygpath -wa "$ROOT"):/repo:ro" --entrypoint bash \
    autodrive-roboracer-avlite:local -c '
source /opt/ros/humble/setup.bash
mkdir -p /tmp/r && for d in src config assets docs scripts tests tools; do [ -e /repo/$d ] && cp -r /repo/$d /tmp/r/; done; cd /tmp/r
rc=0
for pkg in avlite_roboracer avlite_autodrive; do
  (cd /tmp/r/src/$pkg && PYTHONPATH=/tmp/r/src/avlite_roboracer:/tmp/r/src/avlite_autodrive:/tmp/r/src/avlite_roboracer/test \
     python -m pytest -q -p no:cacheprovider -x '"${ARGS:-test}"' 2>&1 | tail -15) || rc=1
done
exit $rc'
