# Execution checklist

Updated 20 September 2026, including the 19 September response test.
Acceptance is defined in the [milestone plan](plan-2026-09-21.md);
the [real-car plan](real-car-plugin-plan.md) extends the work to Jetson testing.

**Current state:** planned mode has passed three-lap screening at an effective
2.5 m/s ceiling. Response tooling is implemented, but braking calibration,
measured 5 m/s, final repeatability and estimated localization remain open.

## Next actions

1. Improve response capture and short-interval qualification: the first test
   recorded three brief coast attempts but zero accepted fits.
2. Collect the missing Jetson facts with `bash tools/inspect-jetson.sh` and
   establish a reachable laptop connection.
3. Re-test response after the measurement changes, then review conservative
   acceleration/braking limits. Keep `braking_calibrated: false` until evidence qualifies.
4. Before enabling calibration, replace the shared 20 m/s ceiling with 3.0 m/s.
   Screen higher speeds in 0.5 m/s steps, then repeat the best unchanged profile.

## A. Simulator driving

### Implemented and demonstrated

- [x] AVLite WorldBridge, SyncExecuter and independent actuator integration.
- [x] Clean 0.5 m/s baseline with a 0.02 throttle cap:
  [validation](avlite-validation.md).
- [x] Telemetry/graphs for pose, speed demands, steering/throttle, preview,
  clearance, saturation, freshness, timing and lap/collision/reset counters.
- [x] Save configurations, logs, JSONL and CSV; provide guided Windows recording.
- [x] Rolling lap statistics, three-/ten-lap capture and finish-feedback checks.
- [x] Speed-dependent steering lookahead, independent gap preview and shorter fallback.
- [x] Curvature/clearance speed caps using explicitly provisional response assumptions.
- [x] Immediate forward-throttle removal and integral reset during deceleration requests.
- [x] Steady-clock actuator scheduling; synthetic and live clock-change checks.
- [x] Initial reactive configured-speed sweep from 2.5 to 5.0 m/s, three clean
  laps at each setting. Measured speed stayed below 5 m/s.
- [x] Built-in GlobalRacePlanner → ReferencePathPlanner → Pure Pursuit integration.
- [x] Reference-assisted RaceMap generation, cyclic profile/full vehicle-envelope
  validation, curved-path obstacle checks, resets and runtime plan recording.
- [x] Planned-mode three-lap screening at an effective 2.5 m/s ceiling.
- [x] Guided response test and analysis reports; first live test completed.
- [x] Automated controller, actuator, planner, recorder and ROS integration coverage.

The response implementation passed 270 AVLite/ROS tests, followed by 29 targeted
checks after the incomplete-summary guard, plus six PowerShell workflow scenarios.
These are development checks, not physical response measurements.

### Still required

- [ ] Capture qualifying acceleration/deceleration measurements, delays and
  slowdown distances at representative speeds; replace provisional assumptions.
- [ ] Tune controller acceleration and actuator response together using those measurements.
- [ ] Quantify steering onset, saturation, corner cutting and inside clearance
  on both bends; validate preview/lookahead beyond the initial screen.
- [ ] Recheck watchdog, sensor-loss, stale-command and competing-publisher behavior
  after control changes.
- [ ] Resolve controller/sensor interruptions and complete the timing items below.
- [ ] Produce comparable per-profile summaries: lap variation, straight/corner
  speed, incidents, saturation, sensor interruptions and controller timing.
- [ ] Repeat unchanged profiles to establish the fastest reliable setting.
- [ ] Screen calibrated planned speeds and demonstrate measured speed toward
  5 m/s on suitable straights without losing clean cornering.

### Final acceptance

Use the same final profile, zero collisions/resets and no manual intervention.

- [ ] Fresh run 1: ten consecutive clean laps.
- [ ] Fresh run 2: ten consecutive clean laps.
- [ ] Fresh run 3: ten consecutive clean laps.
- [ ] Publish the profile, comparable metrics, graphs and remaining limitations.

## B. Mapping and localization

- [ ] Optional raw LiDAR/IMU/encoder/transform/control/reference-pose capture,
  with source/receive times and documented alignment.
- [ ] Occupancy grid with ray-traced free/occupied cells, unknown space and map metadata.
- [ ] AVLite ICP wrapper with saved-map input, one initial pose and encoder/IMU prediction.
- [ ] Explicit rejection of invalid, stale or poorly matched scans; no ground-truth fallback.
- [ ] ROS-independent algorithms connected through AVLite strategy interfaces.
- [ ] Offline map-building, replay and localization-evaluation commands.
- [ ] Tests for known transforms, invalid/missing sensors, initialization and replay.
- [ ] Evaluation on a separate recording: map, estimated/reference paths,
  pose errors, failure counts and processing times.
- [ ] Document measured accuracy and limits before using estimated pose for driving.

The graph's ground-truth LiDAR outline and the planner's corridor map are already
available. Neither fulfills these occupancy-grid/localization checks.

## C. Real-car preparation

- [x] Document plugin architecture and provide the read-only Jetson inspection script.
- [ ] Identify Jetson/JetPack/ROS versions, sensors, actuator interface, frames and stop controls.
- [ ] Establish laptop connectivity and implement recording/replay plus hardware profiles.
- [ ] Implement the mapping/localization modules and evaluate on separate manual recordings.
- [ ] Implement remote start/stop/status/logs with local authorization expiry.
- [ ] Test physical stop, command expiry, process/network loss and reconnect behavior.
- [ ] Measure hardware response and commission low-speed autonomous motion.

These components are proposed; hardware details have not yet been supplied.
See the [implementation order](real-car-plugin-plan.md#implementation-and-acceptance).

## Experiment ledger

Means use complete intervals between observed finish crossings; three crossings
normally yield two rolling laps. See [timing definitions](avlite-setup.md#lap-timing).
Response laps include deliberate coasting and are excluded from racing rankings.

Raw recordings live under ignored `log/recordings/`. **†** means the original
folder was already absent during the 18 September audit. Its numbers are retained
historical findings, not reproducible local evidence. The audit found no duplicate
recordings and deleted none. [Published comparisons](../README.md#recording-references)
remain available.

| Run | Configured → measured peak (m/s) | Rolling mean (s) | Result / interpretation |
| --- | --- | --- | --- |
| [Initial validation](avlite-validation.md) | 0.5 → 0.502 | — | Clean baseline; throttle cap 0.02 |
| `20260916-184246-822-one-lap` | 5.0 → 4.749 | — | Failed: 2 collisions, 2 resets; 16.70 s capture is not a clean lap time |
| `20260916-224134-887-corner-v1-2p5` | 2.5 → 2.486 | 21.189 | 3 clean; inspect late turn selection |
| `20260916-230923-188-corner-v1-2p5` | — | — | Rejected: stale lap/collision counters; no completed laps |
| `20260916-231003-523-corner-v1-2p5` | 2.5 → 2.486 | 21.530 | 3 clean; published “before preview” |
| `20260916-233141-867-corner-v1-2p5` | 2.5 → 2.486 | 15.333 | 3 clean; actual profile `preview-v2` despite label |
| `20260917-075757-575-preview-v2-3p0` | 3.0 → 2.976 | — | Failed on lap 2 during actuator update interruption |
| `20260917-100851-863-preview-v2-3p0` | 3.0 → 2.975 | 15.534 | 3 clean after steady-clock fix |
| `20260917-113015-651-3-laps` † | 3.0 → 2.964 | 14.286 | 3 clean; source of bundled practice map |
| `20260917-113630-368-3-laps` † | 3.5 → 3.374 | 14.358 | 3 clean; higher peak did not improve mean |
| `20260917-114736-267-3-laps` † | 4.0 → 3.683 | 14.316 | 3 clean; rolling laps 14.577 / 14.054 |
| `20260917-114924-098-3-laps` † | 4.5 → 3.757 | 14.483 | 3 clean; slower than same-setting repeat |
| `20260917-120411-594-3-laps` † | 4.5 → 3.728 | 14.172 | 3 clean; fastest historical reactive mean, profile must be recovered/reproduced |
| `20260917-120619-901-3-laps` † | 5.0 → 3.754 | 14.389 | 3 clean; timing disturbance, no actual 5 m/s |
| `20260917-164344-474-3-laps` † | — → 3.743 | 14.184 | Historical reactive comparison for first planned screen |
| `20260917-182353-462-planned-commissioning-2p5` † | 20, capped 2.5 → 2.502 | 12.613 | 3 clean; [historical report](validation/planned-commissioning.md) |
| `20260918-001910-306-preview-v2-3p0-planned` | 20, capped 2.5 → 2.504 | 13.754 | 3 clean; separate available planned run |
| `20260919-101719-674-response-1p5` | 1.5 → 1.522 | Excluded | 4 clean; 3 coast attempts, 0 qualified fits |

All listed higher-speed screens used a 0.2 throttle cap. The six historical
3.0–5.0 m/s sweep runs from `113015` through `120619` used revision `9d042e6`
and identical AVLite/actuator profiles apart from the shared ceiling.
Different versions/profiles must not be treated as a controlled speed-only comparison.

The best 4.5 m/s repeat beat 5.0 by 0.217 s (1.53%), but its earlier repeat was
slower. Two rolling laps per run do not establish a reliable ranking. In the
4.5/5.0 pair, throttle peaked near 0.151/0.152, below the 0.2 cap; targets were
below their ceiling for about 92%/96% of rolling time. Investigate response and
corner limits before increasing throttle.

The September 18 planned run had path error p95 1.8 cm, maximum 3.72 cm, and no
sampled controller overruns or steering saturation. Peak LiDAR/odometry ages
were 0.339/0.288 s; track sensor delivery separately from controller compute time.
See [response findings](response-measurements.md#latest-result) for the latest run.

<a id="30-ms-test-actuator-update-interruption"></a>

## Timing follow-up

- [x] Replace the actuator's ROS/system-clock timer with steady-clock scheduling.
- [x] Verify continued updates under paused/backward clocks and a live 3.0 m/s repeat.
- [ ] Add independent command-age expiry at the simulator bridge.
- [ ] Include stale actuator publication in recorder health/screening decisions.
- [ ] Investigate controller and sensor delivery delays without relaxing timeouts.

The failed `075757` run had a 2.114 s backward UTC adjustment; at collision the
last actuator command was 1.822 s old despite continuing controller requests.
Both publication and watchdog had depended on the paused callback. The fixed
`100851` run saw another 2.348 s rollback but sampled command ages stayed below
55.1 ms. The source of the clock corrections remains unknown.

In historical `120619`, controller work reached 368 ms and command age 364 ms
near the bottom bend; later odometry/LiDAR ages reached 264/218 ms. No large
clock step was observed in that pair, so recurrence of the earlier timer failure
is not established. Its raw folder and detailed comparison exports are unavailable.

The earlier `231003` preview run also had a roughly 0.32 s sensor gap, distinct
from late steering. The [README](../README.md#tests-and-improvements) preserves
the before/after images and explains both implemented fixes.
