# Vehicle response measurements

Measure response before increasing braking assumptions or lifting planned mode's
commissioning cap. Results apply to the tested vehicle, speed range and surface;
simulator calibration does not calibrate the real car.

## Latest result

**Update, 21 September:** `20260921-155118-140-response-2` completed four clean
laps and three qualified coast fits, including displacement/time consistency.
Its conservative suggestion is 3.26 m/s². The prepared 3.0 m/s screening profile
uses 2.0 m/s² braking and 1.5 m/s² acceleration; the subsequent 2.5 m/s response
attempt failed on stale feedback. Higher-speed validation remains pending.
See [current evidence and configuration](validation/speed-fix-20260921.md).

### Earlier response attempt

Run `20260919-101719-674-response-1p5`, 19 September 2026:

| Measurement | Result |
| --- | --- |
| Clean laps; collisions / resets | 4; 0 / 0 |
| Requested / peak measured speed | 1.5 / 1.5222 m/s |
| Peak throttle / cap | 0.0615 / 0.2; no sampled upper saturation |
| Coast attempts / qualifying fits | 3 / 0 |
| Suggested braking deceleration | None; calibration remains pending |

The deliberate throttle cuts began near 22.11, 41.73 and 60.96 s.
Observed coast phases lasted about 0.105, 0.200 and 0.200 s. Only 2, 2 and 1
samples respectively satisfied the speed/zero-throttle conditions, before the
remaining qualification checks. The car slowed too quickly for the current
four-sample, 0.3-second acceptance rule at approximately 10 Hz.

**Implemented 21 September:** `telemetry.response.jsonl` captures each incoming
odometry update, including its header stamp, monotonic receive time and sequence.
The normal 10 Hz telemetry remains available for lap graphs. Qualification now
supports brief coast intervals and checks receive timing, duplicate stamps,
freshness, steering, independent trials and fit uncertainty. New live runs are
still required; the earlier result is not promoted to a braking calibration.

The September 19 result above is retained from the project log; its raw folder
is absent in this checkout. The previously documented guided-test files were
also absent and have now been restored as part of this implementation.

The clean laps confirm that the guided test ran; they do not validate a braking
value. Their deliberate coast periods also make them unsuitable for lap-time ranking.

## 21 September implementation checks

- Full isolated AVLite/ROS suite: **297 passed**. After adding explicit truncated
  capture rejection, the response-analysis suite passed **21 targeted tests**.
- Lint passed. Four Windows workflow scenarios passed: completion, partial
  controller startup, recorder failure and unexpected controller exit.
- Checked the final expiry hook against the installed AutoDRIVE bridge callbacks
  with captured socket output, including stale commands and incomplete startup data.
- Real planned-controller tests covered three synthetic circuits, coast limits,
  sensor-loss aborts and the temporary configuration saved with a ROS recording.

These initial tooling checks did not establish live response or lap performance.
The later live response results and the prepared screening profile are described
in the latest-result update above; 5 m/s and ten-lap acceptance remain pending.

## Guided test

The implemented workflow requires the practice map and simulator ground-truth
pose. With Docker and the native simulator connected:

```powershell
.\measure-response.ps1 -TargetSpeedMps 1.5 -Trials 3
```

Reset when prompted and leave Connection/Autonomous selected. The script uses
one temporary planned controller, retains steering/obstacle checks and removes
forward throttle on an eligible straight. Each coast lasts at most 0.8 s and
ends early if conditions fail or speed reaches 0.6 m/s.

There is at most one attempt per circuit; three requested attempts use four
laps. The test stops on lap/time limits, incidents or errors and cleans up its
controller. Saved YAML files and the actuator throttle cap are unchanged.
`telemetry.plan.json` records the effective temporary profile; `config/` keeps
the original disk files.

Supported options: `-TargetSpeedMps` 1.0–2.5, `-Trials` 3–20,
`-MaxSeconds` (default 600), `-NoOpen`. After improving and validating capture,
review 1.5, 2.0 and 2.5 m/s stages separately before higher-speed screening.
The bridge now replaces throttle and steering with zero if either actuator
publication is older than 0.5 s, using a monotonic clock at the final socket
emission. Reset, reconnect, invalid commands and incomplete sensor packets clear
the stored commands. This protects a running bridge against actuator stalls;
a stopped bridge cannot send a new stop command.

## Reports and qualification

Each run saves normal telemetry, `telemetry.response.jsonl`, `response-controller.log`,
`response-report.json` and `response-report.png`. The report covers attempts,
accepted fits, sampled delays, interval distance and descriptive powered
acceleration. The graph also shows odometry receive intervals; the JSON reports
median, p95, maximum interval and gaps longer than 150 ms.

Current acceptance requires:

| Check | Requirement |
| --- | --- |
| Samples / duration | At least 4 unique odometry updates over at least 0.075 s |
| Receive timing | Median interval at most 0.04 s; no fit gap above 0.06 s |
| Speed / drop | Above 0.5 m/s; decrease at least 0.15 m/s |
| Throttle command and feedback | At most 0.005 |
| Normalized steering magnitude | At most 0.05 |
| Required feedback and response-diagnostic ages | At most 0.15 s |
| Linear-fit RMS error | At most the smaller of 0.1 m/s and 10% of the speed drop |
| Fit uncertainty | Conservative lower slope bound positive and at least half the fitted slope |
| Displacement/time consistency | Position travel and integrated speed differ by no more than the larger of 2 cm or 20% |
| Run status | Complete, clean finish summary with fresh actuator publications |
| Independent accepted attempts | At least 3, or the requested count if larger |

Only one qualifying fragment counts per attempt. Fit speed against time:

$$
v(t)\approx c-bt,\qquad
b_{\mathrm{suggested}}=0.8\min_j\left(b_j-4.303\,\mathrm{SE}(b_j)\right).
$$

$b_j>0$ is fitted deceleration in m/s² for accepted attempt $j$.
The standard error uses a 0.005 m/s residual-noise floor; 4.303 is the two-sided
95% Student-t critical value for the smallest accepted fit (four samples).
These are conservative screening choices, not a measured sensor-noise model or
a guarantee of braking at untested speeds. The suggestion exists only after all
run/trial criteria pass; use no more than the smallest accepted suggestion across
tested speed stages. Duplicate/reordered stamps invalidate the run.

Monotonic receive times are used for fits. The bridge's odometry header stamps
are bridge timestamps, not simulator physics timestamps. Sampled delays include
delivery and diagnostic latency; interval distance is not a full stopping distance.
If source delivery is too slow or bursty, the test remains unqualified. Increasing
the recorder rate does not increase the simulator or bridge's actual delivery rate.

Regenerate an existing report without starting the car or editing settings:

```powershell
.\measure-response.ps1 -RecordingDirectory .\log\recordings\YOUR-RUN
```

The older `analyze-braking.ps1` command analyzes ordinary coast intervals.
Neither analyzer changes settings automatically.

After updating the bridge source, restart it before recording and restart the
actuator after the bridge is back up:

```powershell
docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml stop avlite actuator
docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml restart bridge
docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml start actuator
```

Reconnect the simulator, then run the guided test. It starts the temporary
controller itself and leaves the normal AVLite service stopped afterwards.

## Why the car stays below 5 m/s

In the available September 18 planned run, shared speed was 20 m/s but
`braking_calibrated: false` capped the target at 2.5 m/s. Measured peak was
2.5036 m/s; throttle peaked near 0.1007 against a 0.2 cap. That run demonstrates
commissioning/profile limits, not a simulator maximum at 5 m/s.

At higher demand, feedforward alone consumes the throttle allowance at:

$$
v_{\mathrm{ff\ cap}}=\frac{u_{\max}}{k_{\mathrm{ff}}}
=\frac{0.2}{0.04}=5\ \mathrm{m/s}.
$$

This is a property of the controller equation, not a physical top-speed estimate.
After calibration, inspect target vs measured speed, upper-throttle saturation,
straight length and corner/braking limits before increasing throttle.

Before enabling calibrated braking, replace the shared 20 m/s setting with
**3.0 m/s**, review braking against controller acceleration limits and apply the
accepted value manually. Screen in 0.5 m/s steps with three clean laps per step.
Speeds above 5 m/s follow the existing [acceptance milestone](plan-2026-09-21.md).
