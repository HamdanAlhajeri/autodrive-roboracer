# Planned driving

Set `driving_mode` in [config/avlite.yaml](../config/avlite.yaml) to `planned`
or `follow_the_gap`. The current working profile selects `planned`; the runner's
fallback when the setting is omitted is Follow the Gap.

Planned mode uses pinned AVLite GlobalRacePlanner → ReferencePathPlanner →
Pure Pursuit, with small-car and periodic-path adapters. Its corridor map is
reference-assisted and localization still uses simulator ground truth.

## Run the practice map

1. For commissioning, set `speed_mps: 2.5` and `max_throttle: 0.2` in
   [driving.yaml](../config/driving.yaml).
2. Select `driving_mode: planned`, `planning.map_path: maps/practice.json`
   and `braking_calibrated: false`.
3. Start/connect the Windows simulator, then run:

```powershell
.\record-one-lap.ps1 -Laps 3 -Label planned-2p5
```

Reset when prompted. The wrapper records before driving and stops AVLite after
the requested laps, an incident or timeout. See [recording](avlite-setup.md#recording-a-lap).

The shared ceiling currently on disk is 20 m/s. While uncalibrated, the
additional commissioning limit gives:

$$
v_{\mathrm{ceiling}}=
\begin{cases}
\min(v_{\mathrm{shared}},2.5), & \text{braking uncalibrated},\\
v_{\mathrm{shared}}, & \text{braking calibrated}.
\end{cases}
$$

Thus a 20 m/s request still yields a 2.5 m/s planned ceiling. Before enabling
calibration, replace 20 with the next controlled ceiling, 3.0 m/s.
Calibration is [still pending](response-measurements.md#latest-result).

## Prepare a map

The bundled `config/maps/practice.json` came from clean run
`20260917-113015-651-3-laps` in the unchanged simulator world frame.
That raw recording is unavailable locally; the generated map remains committed.
Use it only with that practice-track layout.

For another map, record at least two finish crossings at a repeatable low speed,
without collisions/resets and with LiDAR outline capture enabled:

```powershell
.\record-one-lap.ps1 -Laps 3 -Label mapping -NoOpen
.\prepare-race-map.ps1 -RecordingDirectory .\log\recordings\YOUR-RUN -Name practice-v2
```

The offline command writes `config/maps/practice-v2.json`, `.plan.json` and
`.png`. It sends no driving commands and refuses an existing name. Inspect the
overlay, select `planning.map_path: maps/practice-v2.json` and restart AVLite.

The converter takes a full lap between crossings, orders observed left/right
boundaries using the driven path, and fits cross-sections at 0.1 m spacing.
It rejects stale/discontinuous poses, missing boundaries, gaps over 0.5 m,
crossings and inadequate width. This version assumes a single corridor with
walls observed within 2.5 m on each side; junctions and opponents are unsupported.

Before driving, the plan is checked for cyclic speed/steering feasibility and
clearance. A rear-to-front-axle capsule uses wheelbase 0.324 m, radius 0.24 m
and tracking allowance 0.05 m. Validation checks the whole capsule at 0.025 m
intervals and checks the route against original LiDAR hits. Missing maps or
invalid plans abort startup.

## Speed and obstacle limits

The global profile uses curvature $\kappa$, lateral limit $a_{\mathrm{lat}}$,
acceleration $a_{\mathrm{acc}}$, deceleration $b$ and segment length $\Delta s$:

$$
v_{\mathrm{corner}}\leq\sqrt{\frac{a_{\mathrm{lat}}}{|\kappa|}},\qquad
v_{i+1}^2\leq v_i^2+2a_{\mathrm{acc}}\Delta s,\qquad
v_i^2\leq v_{i+1}^2+2b\Delta s.
$$

The speed ceiling handles straight segments. Longitudinal constraints also
apply across the lap's closing segment. Initial values are
$a_{\mathrm{acc}}=1.0$, $a_{\mathrm{lat}}=3.0$, $b=1.5$ m/s²; these are assumptions.

The controller previews the profile and checks LiDAR clearance along the curved
route and initial steering arc. For usable distance $d$ after the 0.15 m margin:

$$
v\tau+\frac{v^2}{2b}\leq d
\quad\Longrightarrow\quad
v\leq\sqrt{(b\tau)^2+2bd}-b\tau,\qquad \tau=0.25\ \mathrm{s}.
$$

This constant-deceleration model needs measured response. It cannot guarantee
stopping for an obstacle appearing beyond visibility. No-return beams are
excluded from obstacle hits; corrupt beams remain near obstacles.

Stale/missing sensors, invalid poses and reverse route headings request
deceleration. Resets/interruptions clear progress and PID state; lookahead wraps
around the lap. Negative acceleration removes forward throttle through the
actuator; it does not directly command a known brake force.

Follow the [response procedure](response-measurements.md) before setting
`braking_calibrated: true`. Once qualified, review the conservative value against
controller limits and screen speeds in 0.5 m/s steps, requiring three clean laps
at each step. Keep the 0.2 throttle cap initially and inspect actual saturation.

## Recording and rollback

Every planned recording captures the published runtime plan, including recordings
started after the controller:

| Artifact | Contents |
| --- | --- |
| `telemetry.plan.json` | Active path, speed profile, map/hash and resolved settings |
| `telemetry.racemap.json` / `telemetry.planning-config.json` | Separate map/settings copies |
| `telemetry.planned.png` | Planned/driven paths, speed vs lap distance, error and limiting reason |

Diagnostics include planned speed, progress, path deviation, obstacle speed
limit, plan validity and limiting reason:

| Code | Active limit |
| --- | --- |
| 0 | Speed ceiling |
| 1 | Corner/braking profile |
| 2 | Obstacle |
| 3 | Invalid pose |
| 4 | Missing/stale scan |
| 5 | Commissioning ceiling |
| 6 | Invalid plan |

Unknown obstacle distance is null. For rollback, stop AVLite, select
`follow_the_gap` and appropriate shared limits, then restart both controllers.
Do not run a second controller alongside the first.

The [first commissioning screen](validation/planned-commissioning.md) passed
three clean laps. Final acceptance remains three fresh ten-lap runs with the
same profile; see the [checklist](checklist-2026-09-21.md).
