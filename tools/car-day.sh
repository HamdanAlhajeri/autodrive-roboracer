#!/usr/bin/env bash
# One command that says where the car stands and what to do next. Read-only: it starts nothing,
# installs nothing and sends no commands.
#   bash tools/car-day.sh            (on the Jetson, after tools/jetson-setup.sh)
# Checks the install, the F1TENTH bringup (if running), then lists the unmeasured profile values.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$REPO/src/avlite_roboracer:$REPO/src/avlite_autodrive${PYTHONPATH:+:$PYTHONPATH}"
bad=0
ok()   { printf '  [ ok ] %s\n' "$*"; }
fail() { printf '  [FAIL] %s\n' "$*"; bad=1; }
note() { printf '  [ -- ] %s\n' "$*"; }

echo "== Install"
[[ "$(uname -m)" == aarch64 ]] && ok "arm64 (Jetson)" || note "not arm64 ($(uname -m)): fine on a laptop, but this is not the car"
if   [[ -f /opt/ros/jazzy/setup.bash  ]]; then ok "ROS 2 Jazzy present"
elif [[ -f /opt/ros/humble/setup.bash ]]; then ok "ROS 2 Humble present"
else fail "no ROS 2 (Humble or Jazzy) under /opt/ros: install it for this JetPack first"; fi
if [[ -x /opt/avlite-venv/bin/python ]] && /opt/avlite-venv/bin/python -c "import avlite, scipy, shapely" 2>/dev/null; then
    ok "AVLite venv (/opt/avlite-venv) imports avlite, scipy, shapely"
else
    fail "AVLite venv missing or incomplete: run  bash tools/jetson-setup.sh"
fi
if command -v ros2 >/dev/null 2>&1; then
    ros2 pkg executables avlite_roboracer 2>/dev/null | grep -q roboracer_supervisor \
        && ok "avlite_roboracer built and sourced" \
        || fail "avlite_roboracer not found by ros2: source ~/roboracer_ws/install/setup.bash"
    ros2 pkg prefix slam_toolbox >/dev/null 2>&1 && ok "slam_toolbox installed" || fail "slam_toolbox missing: run tools/jetson-setup.sh"
else
    fail "ros2 not on PATH: source /opt/ros/<distro>/setup.bash (jazzy or humble)"
fi

echo "== F1TENTH bringup (only if it is running)"
if command -v ros2 >/dev/null 2>&1; then
    topics="$(timeout 8 ros2 topic list 2>/dev/null)"
    if [[ -z "$topics" ]]; then
        note "no ROS topics visible: start  ros2 launch f1tenth_stack bringup_launch.py  to check the car"
    else
        for t in /scan /odom /drive /joy; do
            grep -qx "$t" <<<"$topics" && ok "topic $t exists" || fail "topic $t missing (check the F1TENTH bringup)"
        done
    fi
fi

echo "== Hardware profile"
"$(command -v python3 || command -v python)" -m avlite_roboracer.doctor --profile "$REPO/config/roboracer/hardware.yaml" || bad=1

echo
echo "== Next step"
if [[ $bad -ne 0 ]]; then
    echo "Fix the [FAIL] lines and the missing measurements above, then run this again."
    echo "Order of work and how each measurement is taken: docs/jetson-quickstart.md"
else
    echo "Everything is in place. Continue at quickstart step 6 (stationary tests), then step 7."
fi
exit $bad
