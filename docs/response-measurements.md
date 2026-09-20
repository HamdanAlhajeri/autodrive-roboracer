# Vehicle response measurements

Measure response before increasing braking assumptions or lifting planned mode's
commissioning cap. Results apply to the tested vehicle, speed range and surface;
simulator calibration does not calibrate the real car.

## Latest result

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

**Next change:** capture response at incoming odometry rate with timestamps, then
design and validate qualification for these short intervals. Increasing sample
rate alone does not solve the 0.3-second duration requirement. Keep freshness,
steering, independent-trial and fit-quality checks; do not substitute an
unqualified slope for calibration. Re-test after those changes.

The clean laps confirm that the guided test ran; they do not validate a braking
value. Their deliberate coast periods also make them unsuitable for lap-time ranking.

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
The independent bridge command-expiry guard remains unimplemented.

## Reports and qualification

Each run saves normal telemetry plus `response-controller.log`,
`response-report.json` and `response-report.png`. The report covers attempts,
accepted fits, sampled delays, interval distance and active speed limits.

Current acceptance requires:

| Check | Requirement |
| --- | --- |
| Samples / duration | At least 4 over at least 0.3 s |
| Speed / drop | Above 0.5 m/s; decrease at least 0.15 m/s |
| Throttle command and feedback | At most 0.005 |
| Normalized steering magnitude | At most 0.05 |
| Required feedback and response-diagnostic ages | At most 0.15 s |
| Linear-fit RMS error | At most 0.1 m/s |
| Run status | Complete, clean finish summary |
| Independent accepted attempts | At least 3, or the requested count if larger |

Only one qualifying fragment counts per attempt. Fit speed against time:

$$
v(t)\approx c-bt,\qquad
b_{\mathrm{suggested}}=0.8\min_j b_j.
$$

$b_j>0$ is fitted deceleration in m/s² for accepted attempt $j$.
The suggestion exists only after all run/trial criteria pass; use no more than
the smallest accepted suggestion across tested speed stages. Short sampled
delays have 10 Hz resolution, and interval distance is not a full stopping distance.

Regenerate an existing report without starting the car or editing settings:

```powershell
.\measure-response.ps1 -RecordingDirectory .\log\recordings\YOUR-RUN
```

The older `analyze-braking.ps1` command analyzes ordinary coast intervals.
Neither analyzer changes settings automatically.

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
