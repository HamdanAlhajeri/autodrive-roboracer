# Simulator localization: status and to-do (hand-over, 17 Nov 2026)
> Older notes. Current state and next steps: HANDOVER.md at the repository root. The branch is sim-slam-localization.

Goal: drive the planned racing algorithm in the AutoDRIVE simulator on a pose that does not
come from simulator ground truth, then carry the same approach to the real car.

Branch: `claude/wizardly-fermat-48c8c6`. The main checkout still has an uncommitted copy of
the first localization commit (`e428cb4`, imported from another session). Discard that copy
before merging this branch, or the untracked files will block the merge.

## How to run (Windows, visible simulator)

1. `.\avlite.ps1 stack`, then click **Connection** and **Autonomous** in the simulator window.
2. From Git Bash in the repository root:
   - SLAM Toolbox, full: `SIM=windows bash tools/sim-localization/experiment.sh 2.5 3`
   - SLAM on an existing map: add `MAP_FROM=log/sim-localization/20261117-172125`
   - AVLite's own ICP: `SIM=windows LOCALIZER=avlite SKIP_BASELINE=1 bash tools/sim-localization/experiment.sh 2.5 3`
3. Results land in `log/sim-localization/<stamp>/` (`summary.md`, `*-runner.log`,
   `*-diagnostics.txt`). While it withholds commands, the bridge logs `Withholding commands: <reason>`.
4. Offline, with no simulator (replays recorded scans against ground truth):
   `python3 tools/sim-localization/offline_avlite.py log/sim-localization/20261117-172125 --seconds 60`
   inside `autodrive-roboracer-avlite:local`, with `PYTHONPATH=src/avlite_autodrive`.

The SLAM map from the mapping run is in
`log/sim-localization/20261117-172125/mapping/replay-mapping-20261117-133410/`.
`log/` is git-ignored, so this map exists only on this laptop.

## Results so far

| Run | Outcome |
| --- | --- |
| Mapping on ground truth (20261117-172125) | Laps clean; SLAM map built (452x378 cells, 5 cm) |
| Ground-truth baseline (20261117-175733) | 3 laps, 0 collisions |
| SLAM Toolbox localization (20261117-182317) | Car never completed a lap. Parked at the start, error was 1.7-5.3 cm (median 4.4) and yaw median 0.004 rad, but health flapped (see 1). |
| AVLite `LidarLocalization` (20261117-183009) | Drove, diverged by 8.7 m, collided after ~28 s (see 2) |

## Fixed this session (all committed)

- SLAM deadlock: localization only matched scans after 0.2 m of travel, so a parked car never
  got a pose. The sim now uses zero travel thresholds.
- SLAM too slow: it ran at about 2 Hz with poses 2 s late. The sim now uses a cheaper
  correlation search, a 3 m loop-closure search and 5 Hz matching (`DEFAULT_SLAM_OVERRIDES`).
- `map -> odom` stamps trail odometry, and the sim stamps scans 1-6 ms ahead of the encoders.
  tf2 never extrapolates, so health checks failed. Fixed in `slam_localization.map_pose`/`lookup`.
- `experiment.sh`:
  - releases the reset flag (a held `true` re-spawns the car every frame);
  - `exec`s helper commands so Ctrl+C reaches them;
  - reads lap counts as numbers only;
  - adds `SIM=windows`, `MAP_FROM`, `SKIP_BASELINE` and `LOCALIZER=avlite`.
- `map_builder` CLI: builds a race map from a SLAM mapping replay.

## To do (in order)

1. **SLAM scan alignment flaps at the 0.70 threshold** (it reads 0.72-0.75 parked at the
   start, with only ~4 cm pose error).
   - Find out why ~25% of LiDAR points are more than 10 cm from mapped walls. Range is not
     the cause: the LiDAR maxes at 10 m.
   - Suspects: map wall thickness or blur from 43 overlapping laps; the 5 cm map against the
     10 cm tolerance; and `scan_alignment` using the profile's laser mount
     (`sim_profile.yaml`, 0.2733 m).
   - Overlay one scan on `map.pgm` at the SLAM pose to see it.
   - Then lower `localization.min_scan_alignment` in `config/roboracer/sim_profile.yaml`, or
     fix the cause. Re-run with `MAP_FROM`.
2. **AVLite `LidarLocalization`: give it a real map.** Its reference map is only the first
   scan, so it gets pulled back toward the start (offline replay shows it with and without
   the new wheel-odometry prediction, `odometry_pose` in `avlite_localization.py`). Next:
   - Use the SLAM map's occupied cells as the ICP reference: `map.pgm` to points, then to the
     world frame with `map_origin_world`.
   - Replace the brute-force nearest-neighbour search with `scipy.spatial.cKDTree` to keep a
     step under 20 ms. It gives the same matches, just faster.
   - Wire `odometry_pose` from the bridge: store the wheel-odometry pose in
     `on_wheel_odom`, then pass it to `AutoDRIVELidarLocalization` in `runner.py`. It is
     not wired yet; only the offline tool uses it.
   - Validate offline first with `offline_avlite.py`, then in the simulator.
3. Once either source does 3 clean laps: run the full experiment (mapping, localization,
   baseline) and record lap times and error (median/p95/max) in `docs/`.
4. Race on a SLAM-built race map instead of the ground-truth `practice.json`:
   `python -m avlite_roboracer.map_builder REPLAY_DIR --profile config/avlite.yaml --name practice-slam`,
   then set `planning.frame_id: map` and drop `map_origin_world`.
5. Real car: the car's `config/roboracer/slam_toolbox.yaml` has the same 0.2 m travel
   thresholds. Check whether finalization's "localization did not become stable" wait can
   stall while the car is parked (same deadlock as fixed in the sim).
6. Full test suite: it stalls in `test_response_planned.py` and `test_adapter_clock.py` under
   CPU load, and was not verified this session. The localization, odometry, bridge and
   map-builder tests pass (27 + 129).
