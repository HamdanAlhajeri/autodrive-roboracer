#!/usr/bin/env bash
# RACING ONLY. Run ON THE JETSON, with a person at the car holding the emergency stop.
#
#   bash tools/car-race.sh [--race-seconds N]      (default 60)
#
# Races the map that tools/car-map.sh left READY in the running supervisor, for N seconds, then
# stops the car. It refuses to start unless the supervisor is READY, and it never maps. It asks
# you to type GO before the car moves, and needs the joystick dead-man button held.
exec bash "$(dirname "${BASH_SOURCE[0]}")/car-run.sh" --race-only "$@"
