# September 21 execution checklist

Updated: 16 September 2026. Scope and acceptance come from the
[September 21 plan](plan-2026-09-21.md).

## Objective and working rules

Minimize clean lap time on the practice track, with repeatable driving toward
5 m/s on suitable straights and automatic slowing before corners. Rank profiles
by measured clean lap times and reliability, rather than their configured speed.
The 0.5 m/s clean-lap baseline is established; repeating that demonstration is
not the next milestone. Retain it as a regression reference when needed.

- Driving is the immediate priority. Develop mapping/localization offline in
  parallel; live driving continues to use simulator ground-truth localization.
- Keep our algorithms in AVLite plugins with correct sensor frames and units.
- Keep watchdogs and sensor validity checks intact. Do not raise timeouts to
  conceal interruptions or count collision/reset runs as successful laps.
- Mark an item complete only with its implementation or recorded evidence.
  Passing unit tests alone does not establish driving performance.
- Update this checklist and the run ledger after each experiment. Save the
  profile, code revision/changes, measurements, result and next decision.

## Established work

- [x] AVLite controls the simulator through the bridge and independent actuator.
- [x] Low-speed clean-lap baseline established at 0.5 m/s and 0.02 throttle cap.
  Evidence: [validation report](avlite-validation.md) and user confirmation.
- [x] Lightweight telemetry captures measured pose/speed, steering, throttle,
  requested acceleration, odometry/command ages, and lap/collision/reset counts.
- [x] Save configuration snapshots, controller logs, JSONL, CSV and graphs.
- [x] Provide the Windows one-lap workflow with automatic recording, controller
  start/stop and trajectory/speed graph: `record-one-lap.ps1`.
- [x] Capture a 5 m/s failure case and locate both corner failures in telemetry.

Latest failure evidence: local run `20260916-184246-822-one-lap` under
`log/recordings/` (ignored by Git). Configured speed 5 m/s, throttle cap 0.2;
maximum measured speed 4.7487 m/s; two collisions and two observed resets/pose
jumps. Collisions occurred at approximately 6.39 s and 12.60 s, in the bottom
and top bends. Immediately beforehand, speed was still above 4.2 m/s despite
deceleration requests and steering near its limit. This supports investigating
late corner slowdown and actual braking response first.

The 16.70 s recording duration is not a valid clean lap time. The first simulator
lap timer also includes startup waiting. The counter reaching one is not a pass.

## A. Fast, repeatable driving

### 1. Finish the measurements needed for tuning

- [x] Record controller target velocity, actuator speed demand, effective
  lookahead, selected-path curvature and obstacle clearance, with timestamps.
- [x] Record steering/throttle/acceleration saturation, LiDAR age, and controller
  loop duration/overruns alongside existing odometry and command freshness.
- [x] Add `telemetry.control.png` with target vs actual speed, requested vs
  measured acceleration, throttle, clearance/lookahead, saturation, timing and
  collision/reset markers.
- [ ] Measure acceleration and deceleration response at representative speeds:
  commanded acceleration, throttle, measured speed, delay and stopping/slowdown
  distance. Establish what the simulator actually does when throttle falls.
- [ ] Use the new diagnostics and recorded positions to compare both failing
  corners across live runs and calibrate the braking assumptions.

### 2. Improve corner entry and steering

- [x] Implement bounded speed-dependent lookahead with a shorter-lookahead
  fallback when a long straight target corridor cannot fit a tight bend.
- [ ] Tune lookahead on the practice track and check for corner cutting.
- [x] Separate corner preview from the short steering/fallback distance so
  braking does not collapse the requested view ahead; add preview/bearing diagnostics.
- [x] Complete the first `preview-v2` screen with three clean laps at 2.5 m/s;
  compare the recorded path and rolling lap times with `corner-v1`.
- [ ] Quantify changes in steering onset, saturation and inside-corner clearance
  before accepting the profile for higher-speed screening.
- [x] Add speed caps informed by target curvature, observed corridor clearance,
  reaction delay and an assumed braking response.
- [ ] Replace the initial grip/braking assumptions with measured values and tune
  corner entry while preserving speed on available straights.
- [x] Make the actuator remove forward throttle promptly for deceleration
  requests and clear the accumulated speed error during braking.
- [ ] Tune AVLite's acceleration output and the actuator loop together; verify
  that deceleration promptly reduces throttle and produces the needed slowdown.
- [x] Add automated coverage for straight corridors, tight bends, blocked paths,
  changing speeds, lookahead bounds, deceleration requests and actuator recovery.
- [ ] Recheck watchdog, stale-command, sensor-loss and competing-publisher
  behavior after controller/actuator changes.

### 3. Measure progress and screen profiles

- [x] Extend recording/evaluation to distinguish individual laps and support
  three-lap screening and ten consecutive laps without stopping between laps.
- [x] Define consistent lap timing: report standing-start time separately from
  consecutive start/finish crossings; exclude setup waiting and the recorder's
  one-second feedback tail. Preserve the simulator timer as a separate field.
- [x] Require fresh lap/collision counters and the complete finish feedback
  window; stop screening at a collision/reset and reject counter discontinuities.
- [x] Add lap-count/timing tests and check PowerShell argument forwarding and
  controller cleanup with mocked workflows.
- [x] Summarize observed lap times and their mean/variation in JSON.
- [ ] Summarize lap times and their variation, straight/corner speed, collisions,
  resets, sensor interruptions, saturation and timing overruns for each profile.
- [ ] Screen targets below in 0.5 m/s steps, requiring three clean laps with an
  unchanged profile before advancing. Record failed attempts as well as passes.

| Target on suitable straights | Three clean laps | Evidence / status |
| --- | --- | --- |
| 2.5 m/s | [x] | `corner-v1` and `preview-v2` each passed a three-clean-lap screen; latest preview run `20260916-233141-867-corner-v1-2p5` |
| 3.0 m/s | [ ] | Pending |
| 3.5 m/s | [ ] | Pending |
| 4.0 m/s | [ ] | Pending |
| 4.5 m/s | [ ] | Pending |
| 5.0 m/s | [ ] | Existing fixed-lookahead profile failed; see recorded evidence above |

- [ ] Keep the fastest passing profile and its evidence. Explicitly record any
  remaining failure at 5 m/s; do not present a higher speed setting as success.

### 4. Final driving acceptance

Use the same final profile for three fresh runs. Each must achieve ten consecutive
clean laps with zero collisions, resets or manual intervention during measurement.

- [ ] Fresh run 1: ten consecutive clean laps.
- [ ] Fresh run 2: ten consecutive clean laps.
- [ ] Fresh run 3: ten consecutive clean laps.
- [ ] Publish configurations, comparable lap times/variation, actual straight-line
  speeds, sensor interruptions, graphs and known limitations.

## B. Offline mapping and localization

- [ ] Add optional raw LiDAR, IMU, encoder, transform, control and reference-pose
  recording with source/receive timestamps, frame metadata and a documented
  time-alignment method. Keep lightweight recording available.
- [ ] Build a reference 2D occupancy grid from LiDAR and ground-truth poses:
  ray-trace free/occupied cells, retain unknown space, save resolution and frames.
- [ ] Implement our initial 2D ICP scan-to-map localizer, initialized by one
  supplied pose and predicted using encoders and IMU yaw rate. Do not use
  ground-truth position or orientation for subsequent localization updates.
- [ ] Reject/report poor scan matches and missing inputs explicitly; never
  silently substitute ground-truth localization.
- [ ] Keep algorithm cores independent of ROS and connect them through AVLite's
  mapping and localization strategy interfaces.
- [ ] Provide repeatable offline map-building and localization-evaluation commands.
- [ ] Test known scan transforms, invalid/missing sensors, coordinate frames and replay.
- [ ] Evaluate against ground truth on a separate recording and export the map,
  estimated/reference trajectories, pose-error graphs, failure counts and timings.
- [ ] Document measured accuracy and limitations of the reference-assisted prototype.

## Schedule and scope

| Date | Work to complete |
| --- | --- |
| Sep 16-17 | Failure analysis, missing diagnostics, raw sensor capture |
| Sep 18-19 | Steering/speed/braking improvements, speed screening, reference map |
| Sep 20 | Final-profile repeatability tests and separate-recording localization evaluation |
| Sep 21 | Repeat demonstrations and package profiles, graphs, maps and findings |

The practice track is required. Importing a new `.obj` track is stretch work only
if it does not delay driving acceptance. Real hardware deployment, opponents,
loop closure, autonomous SLAM and live estimated localization remain later work.

## Experiment ledger

Record each candidate here; raw logs stay in the run folder. A changed profile
starts a new screening result rather than inheriting another profile's clean laps.

| Profile / run | Actual result | Decision |
| --- | --- | --- |
| Historical 0.5 m/s, cap 0.02 | Clean laps documented; peak 0.5017 m/s in saved final run | Established reference |
| `20260916-184246-822-one-lap`, 5 m/s, cap 0.2 | Peak 4.7487 m/s; 2 collisions; 2 resets/pose jumps | Failed; investigate corner anticipation and deceleration |
| `20260916-224134-887-corner-v1-2p5`, 2.5 m/s, cap 0.2 | Three clean laps; zero collisions/resets; rolling laps 20.914 / 21.464 s; peak 2.4859 m/s | Preserve passing profile; inspect late steering with a track overlay before further tuning |
| `20260916-230923-188-corner-v1-2p5` | Recording rejected after lap/collision telemetry was stale for over 0.5 s; no completed laps | Retain as an interrupted attempt; do not relax the freshness requirement |
| `20260916-231003-523-corner-v1-2p5`, track overlay enabled | Three clean laps; zero collisions/resets; rolling laps 20.604 / 22.457 s; peak 2.4859 m/s | Late turn selection repeats at the bottom bend on all three laps; separate preview from steering distance before increasing speed |
| `20260916-233141-867-corner-v1-2p5`, actually `preview-v2` | Three clean laps; zero collisions/resets; rolling laps 15.069 / 15.597 s; mean 15.333 s; peak 2.4859 m/s | First screen passed; mean rolling lap time 28.8% lower than `231003`; preserve this profile and assess detailed corner behavior before increasing speed |

The candidate requests at least 1.5 m of gap preview, with the existing shorter
fallback, while retaining steering lookahead gain 0.4 s bounded to 0.6–1.8 m; initial lateral
acceleration 3.0 m/s², braking deceleration 1.5 m/s², reaction time 0.25 s and
clearance margin 0.15 m. Those grip/braking values are uncalibrated. Clearance is
checked along straight heading/target corridors; this is not a full curved
footprint check or mapped corner preview. See the
[candidate guide](avlite-setup.md#corner-entry-screening-candidate).

### Latest turn-in evidence and next experiment

The `231003` recording now includes the observed LiDAR outline: 626 scans,
10,127 deduplicated points, and a maximum accepted scan/pose stamp difference
of 1.35 ms. It is an approximate surface map, not an exact track model or a
measurement of the full vehicle's clearance. This visualization does not
complete raw-sensor capture, occupancy-grid mapping or localization milestones.

| First bottom bend | Speed | Steering command | Steering feedback | Corridor clearance | Lookahead |
| --- | --- | --- | --- | --- | --- |
| 7.613 s | 1.092 m/s | 0 degrees | 0 degrees | 0.707 m | 0.600 m |
| 7.713 s | 0.601 m/s | -30 degrees | -1.1 degrees | 0.604 m | 0.600 m |
| 7.922 s | 0.652 m/s | -30 degrees | -30 degrees | 0.499 m | 0.600 m |

The straight-to-sharp-turn transition repeats near world y = -7.5 m at each
bottom-bend entry. The command reaches the steering limit before feedback
catches up; controller turn selection is late, with additional actuator delay.
Lookahead is at its 0.6 m floor for 51.9% of moving samples, and the first bottom
turn spends about 1.33 s at maximum steering. These are sampled observations,
not exact actuator response measurements.

The sharp speed dip near 49 s also coincides with a separate sensor feedback
gap: reported LiDAR/odometry ages reach approximately 0.32 s. No controller
overrun was recorded (maximum loop interval 50.73 ms, work time 6.26 ms).
Investigate that interruption separately rather than treating it as steering
alone or increasing the watchdog timeout.

The tested update uses a longer gap-search preview independent of the short
steering target, retaining the existing tight-bend fallback and braking limits.
The 1.5 m preferred preview passed its first three-clean-lap screen at 2.5 m/s;
broader validation remains open. A gain-only increase from 0.4 to 0.6 may change
the fast approach, but both settings still produce the same 0.6 m floor at
1 m/s and below. Do not raise the fallback minimum merely to extend preview.

The controller, active configuration and diagnostic plots now include this
experiment. Removing only `racing.gap_preview_min_m` restores the previous
coupled behavior. Automated preparation checks passed: 212 tests, including
ROS integration, diagnostic recording/plotting, mirrored early-turn cases,
straight driving, blocked paths, shorter fallback and reset handling; Python
lint and whitespace checks also passed. These checks do not establish live
lap performance or clearance through a moving turn.

For another run with an explicit preview profile label, connect the simulator and run:

```powershell
.\record-one-lap.ps1 -Laps 3 -Label preview-v2-2p5
```

Reset when prompted and keep the complete recording folder, including
`telemetry.lap.png` and `telemetry.control.png`. The `233141` run passed this
screen despite reusing the old `corner-v1` folder label: its saved configuration
and live diagnostics show the 1.5 m preview was active. Curated graphs and
summaries are linked in the [README comparison](../README.md#tests-and-improvements).
Detailed steering-onset, saturation and clearance comparison, higher-speed
screening and three fresh ten-lap acceptance runs remain pending.
