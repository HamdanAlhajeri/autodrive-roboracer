#!/usr/bin/env bash
# Run the test suite inside the devkit container (no display/GPU needed).
#
# Usage from repo root:
#   ./run_tests.sh
set -e

docker compose \
  -f docker-compose.yml \
  -f docker-compose.dev.yml \
  run --rm test
