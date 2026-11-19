# Step 2: reset and recovery

Status: **code done, unit-tested (147 and 383 tests pass) and measured in the simulator. One of the three
plan items was a real bug, one was not reproducible, one is added. A long sensor cut (the car travelling 3 m
or more blind) is still not recovered.** Not committed.

## Summary

| Plan item | What I found | What I did |
| --- | --- | --- |
| Odometry stays in step with a respawn | **Not reproducible.** In 10 respawns with AVLite left running the localizer was never lost (median error 0.04 m after 1 s, 0.26 m after 8 s). Step 0's "lost in 6 of 6" was measured with the simulator at about half real time. | No code change. Baseline kept as a regression check (`tools/sim-localization/reset_test.py`). |
| No re-seed on a short sensor gap | **Real bug.** `runner.py` re-seeded the localizer at the spawn pose whenever sensors went missing for more than 0.5 s, even though the car had not moved. After a 1.4 s cut, 3 of 5 trials were still 1.2 to 5.8 m wrong 5 s later. | The runner now re-seeds only on a real respawn (`after_interruption`); a sensor gap keeps the estimate. |
| Recovery search in the simulator runner | The car runtime had one (`IcpPose.recover`), the simulator runner did not. | Added the same lost-pose check and grid search to the simulator localizer (`recovery_radius_m: 3.0` in `config/avlite.yaml`; 0 turns it off). |
| (found on the way) | After a gap the wheel odometry's first update is a large, wrong step (6 m while the car moved 1 m). The existing guard cannot catch that, because it only undoes an ICP step that fits worse than the odometry prediction. | The first update after a gap compares the old pose and the odometry-moved pose on the current scan and keeps the better fit; a poor fit starts the search at once. |

## Measurements

### Respawn (10 respawns, AVLite left running)

| Seconds after respawn | Before the change, median / max error | After, median / max error |
| --- | --- | --- |
| 1 | 0.04 / 0.09 m | 0.05 / 0.07 m |
| 3 | 0.18 / 0.39 m | 0.30 / 0.54 m |
| 8 | 0.26 / 0.79 m | 0.24 / 0.76 m |

No regression (worst error in any window 1.36 m before, 1.12 m after). Data: `step2/respawn-before.json`,
`respawn-after.json`.

### Sensor gap (the bridge container is paused for about 1.5 s mid-lap, so `/odom` and `/lidar` both stop)

Recovered means the localization error is under 0.8 m five seconds after sensors return. The car keeps
driving while the sensors are cut, so I split trials by how far it travelled blind (`gap_blind.py`):

| Version | Blind under 3 m | Blind 3 m or more |
| --- | --- | --- |
| Before (re-seed at the spawn pose) | 2 of 5 | no trial (the car was standing) |
| After, search of the whole 3 m radius | **3 of 4** | **0 of 4** (7.6 to 9.3 m blind) |
| After, search limited to 0.3 s | 1 of 6 | 0 of 2 |

Data: `step2/gap-before`, `step2/ab-unlimited`, `step2/ab-limited`, summary `step2/gap-blind-summary.txt`.

**What this shows.** The re-seed fix and the first-update check turn a gap with little blind travel from a
coin flip into a mostly recovered case. They do not help when the car drives 3 m or more with no sensor data:
the true position is then outside the search radius, and the estimate stays wrong (the car stops because its
plan is invalid). With 4 + 6 + 8 trials the split is indicative only.

## Issues met and how they were fixed

1. **My first gap test measured the wrong thing.** In the "before" run the car was already standing when
   sensors were cut, so it showed the re-seed bug but not blind travel. The "after" run then looked worse
   (5 to 7 m errors) because the car had travelled blind. I looked at the data (`blind travel` per trial)
   instead of trusting the table, which led to the split above.
2. **The wheel-odometry jump after a gap** (6 m error with 1 m of blind travel) was found by reading one
   trial's time series: the estimate jumped 6 m in one step the moment sensors returned. Fixed as above.
3. **A time limit on the search made recovery worse.** A search takes about 1 s and holds the control loop,
   so I limited it to 0.3 s, nearest candidates first. That recovered 1 of 6 trials against 3 of 4
   unlimited. I reverted the default to no limit (`recovery_budget_s: 0`) and left the option in place.
4. **Log lines printed the alignment after the search** instead of before it. Fixed.
5. About 20 minutes went to reading and re-running before the cause of (1) was clear; the owner asked if
   everything was fine and I said that the latest run was worse, which it was.

## Not fixed

- **Blind travel of 3 m or more.** Needs a search along the planned race line (about 280 poses) rather
  than a grid around the old estimate. Not done; it would also help after any long dropout.
- **The search holds the control loop for about 1 s** (1.0 to 2.7 s seen). It only runs when the pose is
  lost, but at several m/s that is metres of driving on stale commands. Moving it to a thread is possible.
- A pause of the bridge container is my stand-in for a sensor dropout; a real USB or Wi-Fi dropout may differ.
- One PC, one track, 4 to 10 trials per cell.

## What changed

`avlite_localization.py` (lost-pose check, `recover` with nearest-first order and optional time budget,
`resume`, `after_interruption`, first-update-after-gap check, settings and validation),
`runner.py` (re-seed only on a respawn; `localization_recoveries` in the diagnostics),
`config/avlite.yaml` (`recovery_radius_m: 3.0`), 7 new unit tests in `test_avlite_localization.py`.
Tools: `reset_test.py`, `gap_test.sh`, `gap_summary.py`, `gap_blind.py`, `diag_log.py`.

## Sources

Project files only: `src/avlite_autodrive/avlite_autodrive/runner.py`, `avlite_localization.py`,
`plugin/bridge.py`, `src/avlite_roboracer/avlite_roboracer/icp_pose.py` (the car's recovery search this
reuses), `step-0-measurement.md`, `step-1-speed-source.md`.
