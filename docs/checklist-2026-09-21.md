# September 21 execution checklist

Updated: 17 September 2026, including the 18:24 planned-mode screen (Dubai time).
Scope and acceptance come from the
[September 21 plan](plan-2026-09-21.md).

## Built-in planner integration (17 September)

Setup and operation: [planned driving](planned-driving.md).

- [x] Integrate pinned AVLite GlobalRacePlanner -> ReferencePathPlanner -> Pure Pursuit.
- [x] Build a reference-assisted RaceMap and validate the full small-car envelope,
  steering feasibility, and cyclic acceleration/braking profile before driving.
- [x] Add periodic lookahead, reset handling, curved-path obstacle stopping, and
  explicit handling of no-return LiDAR beams. Keep Follow the Gap as the default.
- [x] Record the active map/plan/settings and planned-versus-actual speed/path graphs.
- [x] Pass 239 automated tests, including the planned executor and latched ROS
  recording integration; add six braking-analysis and two obstacle-validation
  regression tests, all passing. See [commissioning evidence](validation/planned-commissioning.md).
- [x] Complete the initial 2.5 m/s planned-mode three-lap screen:
  `20260917-182353-462-planned-commissioning-2p5`, three clean laps, zero collisions/resets,
  peak measured 2.5022 m/s; rolling laps 12.609 / 12.617 s (mean 12.613 s).
  Moving/fresh-diagnostic samples: path error p95 1.27 cm (max 5.24 cm), controller
  work p95 22.30 ms (max 45.31 ms), zero sampled controller overruns. Sensor ages
  still peak near 0.28 s; track that separately from controller scheduling.
- [ ] Measure straight throttle-off response before enabling speeds above the
  commissioning ceiling. Do not treat the provisional 1.5 m/s² value as measured.
- [ ] Screen calibrated speed increments toward measured 5 m/s and complete
  three fresh runs of ten clean laps. Mapping/localization acceptance is unchanged.

This initial planned run's rolling mean is about 11.1% lower than the latest
reactive run (`20260917-164344-474-3-laps`, 14.184 s), even with a 2.5 m/s
commissioning cap. This is a first comparison, not the final repeatability result.
The available recordings contain no qualifying straight coast intervals under
the braking analyzer's freshness, steering, and duration criteria. Braking
remains uncalibrated; higher-speed commissioning and final acceptance remain open.

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

Historical 5 m/s failure evidence: local run `20260916-184246-822-one-lap` under
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
- [x] Compare the 3.0/3.5 and latest 4.5/5.0 m/s recordings using saved profiles,
  actual speed, rolling lap times, corner behavior, throttle and timing diagnostics.
- [ ] Complete a consistent summary of lap times/variation, straight/corner speed, collisions,
  resets, sensor interruptions, saturation and timing overruns for each profile.
- [x] Complete the initial configured-speed sweep from 2.5 to 5.0 m/s in 0.5 m/s
  steps, with three clean laps at each setting. Retain failed attempts as well as passes.

The checkmarks below mean three clean laps with that **configured ceiling**,
not that the car reached or maintained that speed. All listed runs used a 0.2
throttle cap. Means use the two complete intervals between three finish crossings,
excluding the standing-start lap, setup waiting and capture tail.

| Configured ceiling | Three clean laps | Measured peak | Rolling mean | Evidence / status |
| --- | --- | --- | --- | --- |
| 2.5 m/s | [x] | 2.486 m/s | 15.333 s | `20260916-233141-867-corner-v1-2p5`: saved profile is `preview-v2` despite the older label |
| 3.0 m/s | [x] | 2.964 m/s | 14.286 s | `20260917-113015-651-3-laps`; earlier clock-related failure and successful repair retained below |
| 3.5 m/s | [x] | 3.374 m/s | 14.358 s | `20260917-113630-368-3-laps`: higher peak, similar lap time; more time lost in the bottom bend |
| 4.0 m/s | [x] | 3.683 m/s | 14.316 s | `20260917-114736-267-3-laps`: rolling laps 14.577 / 14.054 s; repeatability remains open |
| 4.5 m/s | [x] | 3.728 m/s | 14.172 s | `20260917-120411-594-3-laps`: fastest observed clean-run mean; earlier same-setting run averaged 14.483 s |
| 5.0 m/s | [x] | 3.754 m/s | 14.389 s | `20260917-120619-901-3-laps`: clean screen, but controller/sensor delays require investigation; actual 5 m/s not reached |

- [x] Identify and retain the fastest observed clean-run mean and its saved profile:
  4.5 m/s in `20260917-120411-594-3-laps`, with configuration and telemetry in that run folder.
- [ ] Repeat unchanged profiles to establish the fastest reliable setting. The latest
  pair favors 4.5 m/s by 0.217 s (1.53%), but earlier 4.5 m/s results and the timing
  disturbance in the 5.0 run prevent a firm ranking from this comparison alone.
- [ ] Investigate the 5.0 run's 368 ms controller step / 364 ms command age near
  the bottom bend and the later 264 ms odometry / 218 ms LiDAR ages. Determine the
  cause and repeat the run; preserve the watchdog thresholds.
- [ ] Demonstrate measured speed approaching 5 m/s on suitable straights while
  preserving clean cornering. Passing the configured 5.0 m/s screen does not complete this item.

Keep `max_throttle: 0.2` for the next controlled comparisons: recorded driving
commands peaked at 0.151 and 0.152 in the latest 4.5/5.0 runs, with no upper-cap
saturation. The speed target falls with available clearance before the bottom
bend. Investigate timing and corner-entry/acceleration behavior before increasing
the speed ceiling or throttle cap further; calibrate braking before changing its assumptions.

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
| `20260917-075757-575-preview-v2-3p0`, 3.0 m/s, cap 0.2 | One completed lap, one collision, zero resets; first driving lap 16.056 s; no complete rolling lap; peak 2.976 m/s | Failed screen; actuator output stopped for at least 1.82 s across a backward clock adjustment; repair timing before repeating unchanged settings |
| `20260917-100851-863-preview-v2-3p0`, steady-clock actuator | Three clean laps; zero collisions/resets; rolling laps 15.375 / 15.693 s; mean 15.534 s; peak 2.9753 m/s | Passed 3.0 m/s screen; command age stayed below 56 ms across a 2.348 s clock rollback; mean lap time remains 1.3% above the 2.5 m/s baseline |
| `20260917-113015-651-3-laps`, 3.0 m/s, cap 0.2 | Three clean laps; zero collisions/resets; rolling laps 14.289 / 14.282 s; mean 14.286 s; peak 2.9644 m/s | Baseline for the following 3.5 m/s comparison |
| `20260917-113630-368-3-laps`, 3.5 m/s, cap 0.2 | Three clean laps; zero collisions/resets; rolling laps 14.346 / 14.370 s; mean 14.358 s; peak 3.3740 m/s | Passed initial screen; approximately 0.19 s/lap gained on the right middle section offset by 0.25 s/lap lost in the bottom bend; one run does not establish a ranking |
| `20260917-114736-267-3-laps`, 4.0 m/s, cap 0.2 | Three clean laps; zero collisions/resets; rolling laps 14.577 / 14.054 s; mean 14.316 s; peak 3.6825 m/s | Passed initial screen; repeat to assess the 0.523 s spread between rolling laps |
| `20260917-114924-098-3-laps`, 4.5 m/s, cap 0.2 | Three clean laps; zero collisions/resets; rolling laps 14.561 / 14.406 s; mean 14.483 s; peak 3.7565 m/s | Passed initial screen; retain alongside the faster repeat rather than judging the setting from its best run alone |
| `20260917-120411-594-3-laps`, 4.5 m/s, cap 0.2 | Three clean laps; zero collisions/resets; rolling laps 14.168 / 14.176 s; mean 14.172 s; peak 3.7282 m/s | Fastest observed clean-run mean; use saved settings as the next comparison baseline; standing-start command delays remain recorded |
| `20260917-120619-901-3-laps`, 5.0 m/s, cap 0.2 | Three clean laps; zero collisions/resets; rolling laps 14.538 / 14.240 s; mean 14.389 s; peak 3.7538 m/s | Passed initial screen, but did not reach actual 5 m/s; investigate controller/sensor delays before further speed increases |

As of this update, the working `config/driving.yaml` requests 5.0 m/s with a 0.2
throttle cap. The fastest observed clean-run mean is the saved 4.5 m/s profile in
`120411`, not the highest configured ceiling. The six recent runs above use Git
revision `9d042e6`; their saved AVLite and actuator profiles are identical, with
only the shared speed ceiling varied. The 2.5 and 3.0 m/s profiles remain historical
baselines; the earlier 3.0 m/s actuator-clock repair is documented below.

The candidate requests at least 1.5 m of gap preview, with the existing shorter
fallback, while retaining steering lookahead gain 0.4 s bounded to 0.6–1.8 m; initial lateral
acceleration 3.0 m/s², braking deceleration 1.5 m/s², reaction time 0.25 s and
clearance margin 0.15 m. Those grip/braking values are uncalibrated. Clearance is
checked along straight heading/target corridors; this is not a full curved
footprint check or mapped corner preview. See the
[candidate guide](avlite-setup.md#corner-entry-screening-candidate).

### September 17 speed comparisons and timing follow-up

The 3.0-to-3.5 comparison raised peak speed by 0.410 m/s without improving rolling
lap time (14.286 to 14.358 s). The bottom-bend minimum sampled rolling speed fell
from 1.370 to 0.942 m/s. The small 0.073 s lap-mean difference is insufficient to
establish a reliable ranking from one run at each setting.

The latest 4.5-to-5.0 comparison raised peak speed by only 0.026 m/s. The speed
target was below the configured ceiling for approximately 92% and 96% of rolling
time respectively; clearance limits prompted deceleration before the bottom bend.
The 5.0 run's top-bend minimum rolling speed was 0.870 m/s versus 1.332 m/s at 4.5.
These are measured observations, not a calibrated explanation of cornering response.

At recording time 27.40 s in `120619`, near world (0.02, -6.52) m in the bottom
bend, a controller step took 368 ms and the actuator's last-command age reached
364 ms. The actuator's own interval was about 50 ms at that sample. Later,
odometry age reached 264 ms and controller-reported LiDAR age reached 218 ms.
The cause is unresolved. No large wall-clock step was observed in this pair,
so the recording does not establish recurrence of the earlier clock-sensitive
actuator timer failure. All sampled input ages remained below 0.5 s, but that
does not establish consistently prompt delivery for higher-speed driving.

The 4.5 run also recorded command delays up to 252 ms during its standing-start
lap; its two rolling laps had controller work below 25 ms and command age below
94 ms. The 5.0 rolling laps differed by 298 ms versus 8 ms in the latest 4.5 run.
Two rolling intervals per run are insufficient to complete repeatability acceptance.

Detailed comparisons and graphs are available locally under
`log/comparisons/20260917-3p0-vs-3p5/` and
`log/comparisons/20260917-4p5-vs-5p0/` (ignored by Git). The screening results and
limitations are retained in this checklist so progress is visible without those files.

### Earlier corner-preview evidence (2.5 m/s)

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

The actuator timer correction passed the checks below, and the unchanged 3.0 m/s
retest subsequently passed. Later configured-speed screens are recorded above.
For future repeats, reset when prompted and keep the complete recording folder, including
`telemetry.lap.png` and `telemetry.control.png`. The earlier `233141` run passed
the 2.5 m/s screen despite reusing the old `corner-v1` folder label: its saved configuration
and live diagnostics show the 1.5 m preview was active. Curated graphs and
summaries are linked in the [README comparison](../README.md#tests-and-improvements).
Detailed steering-onset and clearance validation, braking calibration, resolution
of the later controller/sensor delays, and three fresh ten-lap acceptance runs remain pending.

### 3.0 m/s test: actuator update interruption

The `20260917-075757-575-preview-v2-3p0` recording is a failed screen, with
valid lap/collision counters. It completes one lap before colliding at the upper
bend on lap two. The first-motion-to-finish time is 16.056 s, versus 15.607 s
for the passing 2.5 m/s recording; there is no completed rolling lap to compare
with that profile's 15.333 s rolling mean. The simulator's saved last-lap field
of 118.985 s does not match the measured driving interval and is not used.

The failure has a distinct actuator-timing signature:

| Recording elapsed time | Observation |
| --- | --- |
| 30.785–30.886 s | Recorded UTC steps backward by approximately 2.114 s in total; actuator publication stops |
| 30.990 s | AVLite requests -1.5 m/s² braking, but throttle feedback remains 0.094 |
| 31.199 s | Age of the last actuator command passes 0.5 s while sensor/controller updates continue |
| 32.236 s | AVLite asks for -30 degrees steering; physical steering remains about -5.25 degrees |
| 32.511 s | Collision counter rises; last actuator command is 1.822 s old |

During the interruption, speed stays near 2.332 m/s until impact. No controller
overrun is recorded: maximum compute time is 7.62 ms and maximum loop interval
53.36 ms. A similar 1.806 s backward clock adjustment at startup accompanies a
1.856 s actuator-loop gap and a `control timing discontinuity` log entry.

The likely mechanism is the default ROS/system-clock timer in
[adapter.py](../src/avlite_autodrive/avlite_autodrive/adapter.py). Both actuator
publishing and its watchdog run inside that timer callback. Monotonic age checks
inside the callback cannot run while the timer itself is paused. The stock
simulator bridge continues sending the last actuator values. This strongly
supports a clock-sensitive scheduling failure; the recording does not identify
the Windows/WSL source of the clock correction.

- [x] Schedule actuator publication and watchdog checks on a steady clock.
- [ ] Add an independent command-age stop guard at the simulator bridge.
- [ ] Include stale actuator publication in recording health and screening checks.
- [x] Verify continuing updates and stopping behavior under backward clock changes.
- [x] Repeat the unchanged 3.0 m/s / 1.5 m preview profile after the timing fix:
  `20260917-100851-863-preview-v2-3p0` passed three clean laps.

Driving settings were unchanged between the failed and passing 3.0 m/s runs.
The first result establishes an actuator-output interruption; the repeat
provides the first successful three-lap screen at this speed.

Timer correction validation: 30 actuator/integration tests passed, including
the permanent paused/backward-clock regression. An isolated
test of the actual adapter kept updates running with ROS time paused and moved
backward by two seconds (maximum callback interval 55.1 ms); the stale-input
watchdog sent zero throttle and steering after 0.516 s. A comparison ROS-clock
timer stopped firing under the same stimulus. No host clocks or live simulator
services were changed. The independent bridge timeout remains open.

The live repeat experienced another recorded UTC rollback of 2.348 s around
27.4–27.6 s, yet maximum sampled outgoing throttle/steering command ages stayed
below 55.1 ms and actuator loop intervals below 55.2 ms. No controller overruns
or moving steering-saturated samples were observed. This supports the scheduling
fix under an actual clock disturbance; it does not identify or eliminate the
source of the clock corrections. The outline mapper rejected 205 nonmonotonic
scan stamps and 205 odometry stamps, with one internal pairing-history reset;
the vehicle reset count remained zero.

The rolling mean is 15.534 s versus the earlier 2.5 m/s run's 15.333 s, so this
is a reliability improvement, not evidence of faster laps. Curated graphs and
summaries are in the [README comparison](../README.md#reliable-actuator-updates--17-september-2026).
The three fresh ten-lap acceptance runs remain pending.
