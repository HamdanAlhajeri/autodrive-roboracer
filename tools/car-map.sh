#!/usr/bin/env bash
# MAPPING ONLY. Run ON THE JETSON, with a person at the car holding the emergency stop.
#
#   bash tools/car-map.sh [--laps N]
#
# The car drives its slow autonomous mapping lap(s), stops, then the map is built and checked.
# It ends with the car READY (a validated map) or STOPPED with the reason printed. It never races.
# It asks you to type GO before the car moves, and needs the joystick dead-man button held.
#
# The F1TENTH bringup and the supervisor must already be running and stay running, so the
# READY state is still there for tools/car-race.sh:
#   ros2 launch f1tenth_stack bringup_launch.py
#   bash tools/jetson-run.sh launch
exec bash "$(dirname "${BASH_SOURCE[0]}")/car-run.sh" --map-only "$@"
