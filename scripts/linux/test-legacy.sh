#!/usr/bin/env bash
# Run the test suite inside the devkit container (no display/GPU needed).
#
# Usage: bash scripts/linux/test-legacy.sh
set -e

# Resolve paths from this script, so calling it outside the repository still
# selects the legacy Compose files and the correct source directory.
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
docker compose \
  --project-directory "$project_root" \
  -f "$project_root/docker/legacy/compose.yml" \
  -f "$project_root/docker/legacy/compose.dev.yml" \
  run --rm test
