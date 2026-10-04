# Execution checklist

Updated 22 September 2026, including high-speed software checks and clearance optimization.
Acceptance is defined in the [milestone plan](milestone-2026-09-21.md);
the [real-car plan](real-car-plugin.md) extends the work to Jetson testing.

**Practice-track result, 22 September:** the final 2.0 m/s² acceleration / 3.0 m/s² braking profile
completed three clean laps with a 12 m/s shared ceiling, reaching 4.5134 m/s.
Its planned profile peaks at 5.698 m/s. Actual 10+ m/s on
this practice track, higher-speed response, repeatability and estimated
localization remain open.

## Next actions

**Speed work recorded on 22 September:** the shared ceiling was 12.0 m/s and throttle cap 0.60;
the next profile uses 2.0 m/s² acceleration and 3.0 m/s² braking. Both ROS adapters handle
normal high-speed odometry displacements, and indexed obstacle checks preserve
the exact swept clearance calculation. The latest 2.5 m/s response run completed
four clean laps and qualified all three coast fits, suggesting 3.76 m/s² braking.
See [evidence and commands](../validation/high-speed-20260922.md).

1. Obtain qualified response measurements at representative speeds before changing braking.
2. Improve target tracking using the saved diagnostics and repeat the updated profile.
3. Use measured vehicle limits to assess whether this track supports 10+ m/s;
   repeat the best unchanged profile and retain incomplete acceptance items.

Mapping, localization and hardware implementation are deferred from today's work.

## A. Simulator driving

### Implemented and demonstrated

- [x] AVLite WorldBridge, SyncExecuter and independent actuator integration.
- [x] Clean 0.5 m/s baseline with a 0.02 throttle cap:
  [validation](../validation/avlite-baseline.md).
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
- [x] Restore the guided-test implementation missing from this checkout, with
  source-rate odometry capture, short-interval fit uncertainty and timing graphs.
- [x] Temporary low-speed response profile, one coast per circuit, interruption
  abort and controller cleanup on success, partial startup and runtime failure.
- [x] Automated controller, actuator, planner, recorder and ROS integration coverage.
- [x] Software support above 10 m/s: shared settings, actuator demand and a
  synthetic large-track planner/controller test, including the finish seam.
- [x] Speed-aware odometry continuity preserves high-speed updates and still
  resets on unexplained displacement or sensor interruption.
- [x] Indexed LiDAR sweep matches full pairwise clearance, with an offline
  benchmark and curved-path/end-cap regression checks.
- [x] Live three-lap clearance-optimization check: fresh moving samples had
  8.16 ms controller-step p95 and no sampled overruns; 4.1409 m/s peak.
- [x] Qualified three coast trials during a clean four-lap 2.5 m/s response run.
- [x] Final 2.0 / 3.0 m/s² profile: three clean laps, 4.5134 m/s peak,
  rolling times 10.9304 / 11.1278 s. Actual 10+ m/s remains unachieved.

The September 19 live result is historical: its raw folder is absent here.
Today's verification is recorded in [response measurements](../response-measurements.md).
Automated checks do not establish physical response or lap performance.

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

### Supplied sketch track (24 September)

- [x] Extract the supplied mesh assets; convert Unity XZ boundaries into AVLite world XY.
- [x] Validate a 48.29 m commissioning line, including physical barrier and vehicle clearance.
- [x] Add a separate 2.5 m/s commissioning profile and custom executable launcher option.
- [x] Pass 63 geometry/planner/configuration tests and preserve the practice plan.
- [x] Install Unity Editor 2022.3.52f1 and the AutoDRIVE source project.
- [x] Add reproducible scene/build scripts with literal mesh coordinates and ordered checkpoints.
- [x] Build and open the native Windows sketch scene; select its matching map and commissioning settings.
- [x] Verify scale, spawn alignment, four-wheel contact, LiDAR, consecutive lap-counter cycles and reset placement.
- [ ] Verify barrier collision/respawn behavior during driving.
- [ ] Commission the new scene, measure its response and record clean laps.

See [sketch track setup](../sketch-track.md). Offline asset conversion does not
complete the independent mapping/localization work above.

## C. Real-car preparation

- [x] Document plugin architecture.
- [ ] Provide the documented Jetson inspection script (absent in this checkout; deferred).
- [ ] Identify Jetson/JetPack/ROS versions, sensors, actuator interface, frames and stop controls.
- [ ] Establish laptop connectivity and implement recording/replay plus hardware profiles.
- [ ] Implement the mapping/localization modules and evaluate on separate manual recordings.
- [ ] Implement remote start/stop/status/logs with local authorization expiry.
- [ ] Test physical stop, command expiry, process/network loss and reconnect behavior.
- [ ] Measure hardware response and commission low-speed autonomous motion.

These components are proposed; hardware details have not yet been supplied.
See the [implementation order](real-car-plugin.md#implementation-and-acceptance).

## Experiment ledger

Means use complete intervals between observed finish crossings; three crossings
normally yield two rolling laps. See [timing definitions](../avlite-setup.md#lap-timing).
Response laps include deliberate coasting and are excluded from racing rankings.

Raw recordings live under ignored `log/recordings/`. **†** means the original
folder was already absent during the 18 September audit. Its numbers are retained
historical findings, not reproducible local evidence. The audit found no duplicate
recordings and deleted none. [Published comparisons](../validation/controller-history.md#recording-references)
remain available.

| Run | Configured → measured peak (m/s) | Rolling mean (s) | Result / interpretation |
| --- | --- | --- | --- |
| [Initial validation](../validation/avlite-baseline.md) | 0.5 → 0.502 | — | Clean baseline; throttle cap 0.02 |
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
| `20260917-182353-462-planned-commissioning-2p5` † | 20, capped 2.5 → 2.502 | 12.613 | 3 clean; [historical report](../validation/planned-commissioning.md) |
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
See [response findings](../response-measurements.md#latest-result) for the latest run.

<a id="30-ms-test-actuator-update-interruption"></a>

## Timing follow-up

- [x] Replace the actuator's ROS/system-clock timer with steady-clock scheduling.
- [x] Verify continued updates under paused/backward clocks and a live 3.0 m/s repeat.
- [x] Add independent monotonic command-age expiry at the simulator bridge's final output.
- [x] Include stale/missing/invalid actuator publication in recorder health/screening decisions.
- [x] Add source-rate odometry timing diagnostics for response investigations.
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
from late steering. The [controller history](../validation/controller-history.md) preserves
the before/after images and explains both implemented fixes.
