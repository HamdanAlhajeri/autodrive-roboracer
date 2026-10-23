#!/usr/bin/env bash
# One-time runtime install for the Jetson car (Orin, JetPack 6, ROS 2 Humble).
# Usage: bash tools/jetson-setup.sh [--workspace DIR] [--venv DIR] [--skip-apt]
#
# Installs SLAM Toolbox and Python dependencies, creates the AVLite venv with the same
# pins as docker/Dockerfile.avlite, and builds avlite_autodrive + avlite_roboracer into
# a separate colcon workspace. Safe to rerun. It does not modify the F1TENTH workspace,
# start any node, or send vehicle commands.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$HOME/roboracer_ws"
VENV="/opt/avlite-venv"
SKIP_APT=0
# Keep these in step with docker/Dockerfile.avlite.
AVLITE_REVISION="1653f592b2eb5289a14c0d1041e50fdea855322a"
PIP_PINS=(pip==25.3 setuptools==79.0.1 wheel==0.45.1 packaging==25.0)
APT_PACKAGES=(ros-humble-slam-toolbox ros-humble-ackermann-msgs python3-venv python3-scipy
              python3-shapely python3-yaml python3-colcon-common-extensions)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --workspace) WORKSPACE="$2"; shift 2 ;;
        --venv) VENV="$2"; shift 2 ;;
        --skip-apt) SKIP_APT=1; shift ;;
        *) printf 'Usage: bash tools/jetson-setup.sh [--workspace DIR] [--venv DIR] [--skip-apt]\n'; exit 1 ;;
    esac
done

step() { printf '\n== %s\n' "$*"; }
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
# ROS setup scripts reference unset variables, so source them with -u relaxed.
source_relaxed() { set +u; source "$1"; set -u; }

step 'Checking the platform'
[[ "$(uname -m)" == "aarch64" ]] || printf 'WARNING: %s is not aarch64; this targets the Jetson.\n' "$(uname -m)"
[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble not found. Install ROS 2 Humble (JetPack 6 / Ubuntu 22.04) first.'
python3 -c 'import sys; sys.exit(sys.version_info[:2] != (3, 10))' \
    || fail "ROS 2 Humble expects Python 3.10; found $(python3 --version)."
source_relaxed /opt/ros/humble/setup.bash
missing=()
for pkg in urg_node2 vesc_driver ackermann_mux; do
    ros2 pkg prefix "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
done
if (( ${#missing[@]} )); then
    printf 'WARNING: F1TENTH packages not found in this shell: %s\n' "${missing[*]}"
    printf '         Source the F1TENTH workspace before launching the car stack.\n'
fi

if (( ! SKIP_APT )); then
    step 'Installing ROS and Python packages (sudo)'
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends "${APT_PACKAGES[@]}"
fi

step "Creating the AVLite venv at $VENV"
if [[ ! -x "$VENV/bin/python" ]]; then
    if [[ ! -w "$(dirname "$VENV")" ]]; then
        sudo install -d -o "$(id -u)" -g "$(id -g)" "$VENV"
    fi
    # System site packages keep rclpy and the ROS message packages importable.
    python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/pip" install --upgrade "${PIP_PINS[@]}"
"$VENV/bin/pip" install --no-build-isolation --constraint "$REPO/docker/avlite-constraints.txt" \
    "avlite @ git+https://github.com/AV-Lab/avlite.git@${AVLITE_REVISION}"
"$VENV/bin/pip" freeze > "$VENV/avlite-dependencies.txt"

step "Building avlite_autodrive and avlite_roboracer in $WORKSPACE"
mkdir -p "$WORKSPACE/src"
for pkg in avlite_autodrive avlite_roboracer; do
    # Symlinks let `git pull` in the repository update the workspace sources.
    ln -sfn "$REPO/src/$pkg" "$WORKSPACE/src/$pkg"
done
source_relaxed "$VENV/bin/activate"
(cd "$WORKSPACE" && python -m colcon build --symlink-install \
    --packages-select avlite_autodrive avlite_roboracer)
source_relaxed "$WORKSPACE/install/setup.bash"

step 'Verifying the install'
python -c 'import avlite, rclpy, tf2_ros, ackermann_msgs.msg, avlite_roboracer, avlite_autodrive' \
    || fail 'The venv cannot import AVLite and the ROS packages together.'
ros2 pkg executables avlite_roboracer

step 'Done. Add these lines to ~/.bashrc (F1TENTH workspace before this one):'
printf 'source /opt/ros/humble/setup.bash\n'
printf 'source <your F1TENTH workspace>/install/setup.bash\n'
printf 'source %s/bin/activate\n' "$VENV"
printf 'source %s/install/setup.bash\n' "$WORKSPACE"
printf '\nNext: bash tools/jetson-run.sh inspect\n'
