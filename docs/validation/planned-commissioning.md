# First planned-mode commissioning screen

Historical run `20260917-182353-462-planned-commissioning-2p5`,
17 September 2026, 18:23:59–18:24:47 Dubai time. Native practice simulator,
ground-truth localization and pinned AVLite GlobalRacePlanner →
ReferencePathPlanner → Pure Pursuit.

**Evidence availability:** the 18 September audit found the original run,
its reactive comparison and `log/planner-validation/commissioning-metrics.json`
absent locally. This report preserves the findings. The available September 18
planned recording is a different run.

| Measurement | Result |
| --- | --- |
| Clean laps; collisions / resets | 3; 0 / 0 |
| Shared / effective ceiling; throttle cap | 20 / 2.5 m/s; 0.2 |
| Peak measured speed | 2.5022 m/s |
| Rolling lap times | 12.608682 / 12.617444 s |
| Rolling mean / population standard deviation | 12.613063 / 0.004381 s |
| First-motion-to-finish time | 13.603302 s |
| Absolute path error, p95 / maximum | 0.01273 / 0.05237 m |
| Controller work, p95 / maximum | 22.30 / 45.31 ms |
| Controller interval, p95 / maximum | 50.36 / 51.28 ms |
| Sampled controller overruns while moving | 0 |
| Peak LiDAR / odometry age | 0.2763 / 0.2791 s |
| Peak throttle command | 0.10070 |

Statistics use moving samples with controller diagnostics no older than 0.5 s.
Observed limiting reasons were commissioning (246 samples) and corner/braking
profile (137); no sampled obstacle/invalid-plan limit appeared. Shorter events
between samples could be missed.

The historical reactive run `20260917-164344-474-3-laps` averaged 14.184055 s
with a 3.7425 m/s peak. The planned mean was 11.1% lower, but the controllers and
profiles differed. This is an initial comparison, not final repeatability evidence.

The screen also passed 239 AVLite/ROS tests, followed by 33 targeted
planner/braking checks including eight added after the full suite. Lint,
PowerShell parsing and whitespace checks passed. These counts describe that
milestone, not the current test suite.

Braking remained uncalibrated; the 1.5 m/s² assumption was provisional.
Higher-speed acceptance and three fresh ten-lap runs were not completed.
For current results and remaining work, use the [checklist](../checklist-2026-09-21.md).
