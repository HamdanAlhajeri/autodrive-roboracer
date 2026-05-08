#!/usr/bin/env bash
# Entrypoint script baked into a custom image submission.
# Auto-builds and launches the racer node when the container starts.
# Organizers can still open extra bash sessions without triggering this code
# because it is invoked explicitly as the entrypoint, NOT via ~/.bashrc.

set -e

WS=/home/autodrive_devkit

echo "[entrypoint] Building my_team_racer..."
cd "$WS"
colcon build --packages-select my_team_racer --symlink-install

# shellcheck source=/dev/null
source "$WS/install/setup.bash"

echo "[entrypoint] Launching autodrive_roboracer bringup + racer node..."
ros2 launch autodrive_roboracer bringup_graphics.launch.py &

sleep 3  # wait for autodrive bridge to come up

ros2 launch my_team_racer racer.launch.py
