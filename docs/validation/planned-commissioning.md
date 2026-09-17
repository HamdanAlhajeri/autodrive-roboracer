# First planned-mode commissioning screen

Run: `20260917-182353-462-planned-commissioning-2p5`, 17 September 2026,
18:23:59–18:24:47 Dubai time. Native practice simulator, ground-truth localization,
pinned AVLite GlobalRacePlanner -> ReferencePathPlanner -> Pure Pursuit.

| Measurement | Result |
| --- | --- |
| Confirmed consecutive clean laps | 3 |
| Collisions / resets | 0 / 0 |
| Effective commissioning ceiling | 2.5 m/s |
| Shared requested ceiling / throttle cap | 20 m/s / 0.2 |
| Peak measured speed | 2.5022 m/s |
| Full rolling laps | 12.608681521 / 12.617444327 s |
| Rolling mean / standard deviation | 12.613062924 / 0.004381403 s |
| First lap from first observed motion | 13.603302422 s |
| Absolute path error, p95 / maximum | 0.01273 / 0.05237 m |
| Controller work, p95 / maximum | 22.30 / 45.31 ms |
| Controller interval, p95 / maximum | 50.36 / 51.28 ms |
| Sampled controller overruns while moving | 0 |
| Peak observed LiDAR / odometry age | 0.2763 / 0.2791 s |
| Peak throttle command | 0.10070 |

Tracking/timing statistics use moving samples with controller diagnostics no
older than 0.5 s. The sampled limiting reasons were commissioning ceiling (246)
and planned corner/braking profile (137); no obstacle or invalid-plan limit was
reported in those samples. Telemetry is sampled, not proof that no shorter event
occurred between samples.

The latest reactive run (`20260917-164344-474-3-laps`) had a rolling mean of
14.184054907 s and peak measured speed 3.7425 m/s. This planned screen's mean was
about 11.1% lower while staying within the 2.5 m/s commissioning ceiling. This is
an initial comparison of different controllers/profiles, not a controlled final
repeatability claim.

The recording directory under `log/recordings/` contains the actual map, plan,
resolved settings, JSONL, CSV, ordinary graphs, and `telemetry.planned.png`.
Additional moving-sample metrics are in
`log/planner-validation/commissioning-metrics.json`. The map used in the live
run is preserved in `telemetry.racemap.json`; later provenance additions to the
bundled map do not replace that snapshot.

Validation also passed the full 239-test AVLite/ROS suite, followed by targeted
planner/braking checks (33 tests, including the eight tests added after the full
suite). Python lint, PowerShell parsing, and `git diff --check` passed. The bundled
map passes final steering, acceleration/braking, full capsule, and raw-obstacle
checks. The AVLite revision was not changed.

No available recording supplied qualifying straight coast intervals for the
braking analyzer. `braking_calibrated` remains false; the 1.5 m/s² assumption is
provisional. Higher-speed screening and three fresh ten-lap runs remain pending.
AVLite was stopped after the screen and the default Follow the Gap mode restored.
