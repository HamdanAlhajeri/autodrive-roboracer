# Contributing

## Set up a checkout

1. Clone this repository and read the [README](README.md).
2. On Windows, use PowerShell 5.1 or later and Docker Desktop with Linux containers.
   AVLite, Python dependencies and ROS run inside Docker; no host ROS install is needed.
3. The default setup uses the stock practice simulator and `maps/practice.json`.
   The launcher downloads that simulator if needed; Unity Editor is not required.
   For the optional custom track, follow [sketch setup](docs/sketch-track.md) and
   match `config/avlite.yaml` with the `config/windows-simulator.json` override.
   Native builds and Unity source checkouts are not stored in Git.
4. Run `.\avlite.ps1 start`, connect the simulator, then record changes with
   `.\avlite.ps1 laps -Laps 3 -Label your-change`.

## Where changes belong

| Work | Location |
| --- | --- |
| AVLite plugins, control, planning and telemetry | `src/avlite_autodrive/avlite_autodrive/` |
| Python/ROS regression tests | `src/avlite_autodrive/test/` |
| Shared speed/throttle, active control and map profiles | `config/` |
| Windows workflows | `scripts/windows/`; register commands in `common.ps1` |
| PowerShell command/workflow tests | `tests/` |
| Unity scene builder and native checks | `tools/unity/` |
| Reusable track source assets | `assets/tracks/` |
| Instructions, plans and curated evidence | `docs/` |
| Original standalone controller | `src/my_team_racer/`; launch via `docker/legacy/` |

Keep upstream AVLite pinned in `docker/Dockerfile.avlite` and implement project
adaptations in our package. New Windows operations should be commands on
`avlite.ps1`, rather than additional root scripts. Parameter declarations in the
implementation are the source of CLI types, validation and defaults.

## Check a change

For Windows workflow changes, run the same suite used by CI:

```powershell
.\avlite.ps1 test
```

For planner/config changes, also run `.\avlite.ps1 speed` to validate the active
map. For Python and ROS changes, use the pinned runtime and an isolated ROS domain:

```powershell
docker compose -f docker-compose.avlite.yml build actuator avlite
docker compose -f docker-compose.avlite.yml run --rm --no-deps `
  -e ROS_DOMAIN_ID=73 -e RUN_ROS_TESTS=1 --entrypoint /bin/bash avlite `
  -c 'source /opt/ros/humble/setup.bash && python -m pytest -q -p no:cacheprovider test'
```

Use a domain unused by a live car or another test session. These synthetic tests
do not replace recorded simulator laps. Linux and package-build commands are in
[setup](docs/avlite-setup.md#repeatable-tests). The legacy standalone controller
has a separate runner: `bash scripts/linux/test-legacy.sh`.

## Share reviewable work

Use a branch per focused change and open a pull request describing the resulting
behavior and checks performed. Include a reproducible command and relevant
graphs when changing driving behavior. Keep `config/driving.yaml` as the shared
source of speed and throttle; distinguish requested speed from achieved speed.

Keep full recording folders locally under `log/recordings/`. Publish only selected
graphs/reports in `docs/validation/`, with run IDs, track, settings and limitations.
Generated simulators, Unity projects, downloads, raw telemetry and caches stay
out of Git. Do not commit machine-specific absolute paths or credentials.
