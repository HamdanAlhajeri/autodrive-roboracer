# How localization is measured in the simulator runs

This note defines the localization figures used in the simulator validation notes
(`docs/validation/iros2026-*`): alignment, "localization lost" and localization error. It is
written so the definitions can be quoted in the SDP report.

Source code: `src/avlite_autodrive/avlite_autodrive/avlite_localization.py` (alignment, lost
detection, recovery), `src/avlite_autodrive/avlite_autodrive/runner.py` (published diagnostics) and
`tools/sim-localization/measure_pose.py` (recording).

## 1. Alignment (per LiDAR scan)

AVLite localizes the car by matching each LiDAR scan to a map of the track walls (ICP scan
matching against `reference: map_hits`). After each match it scores how well the scan fits:

1. Every LiDAR return that hit something is transformed into map coordinates using the current
   pose estimate (x, y, heading).
2. For each point, the distance to the nearest map wall point is found (k-d tree lookup).
3. The alignment is the share of points that lie close to a wall:

   alignment = (number of scan points within 0.10 m of a map wall) / (total scan points)

Alignment is between 0 and 1. A value of 0.93 means 93 % of the scan lands on a wall at the
estimated pose. The 0.10 m tolerance is `alignment_tolerance_m`.

## 2. "Localization lost" (recovery count)

| Setting | Value | Meaning |
| --- | --- | --- |
| `lost_alignment` | 0.5 | a scan below this alignment is a "low" scan |
| `lost_scans` | 5 | low scans in a row before the pose is treated as lost |
| `recovery_radius_m` | 5.0 m in these runs (earlier runs 3.0; 0 turns recovery off) | search radius around the estimate |
| `recovery_accept` | 0.6 | alignment a recovered pose must reach |
| `recovery_interval_s` | 3.0 s | at most one search in this time |

Procedure, on every scan:

1. If alignment is at least 0.5, the low-scan counter resets to 0.
2. Otherwise the low-scan counter increases by 1.
3. When it reaches 5, and at least 3 s have passed since the last search, the localizer is
   considered lost:
   - the attempt counter `recovery_tries` increases by 1;
   - candidate poses within 5 m of the estimate are scored with the same alignment formula;
   - if the best candidate scores at least 0.6, the localizer moves to it, the counter
     `recoveries` increases by 1 and the low-scan counter resets;
   - if no candidate reaches 0.6, the estimate is kept and `recoveries` does not change.
4. Each search is logged as `Localization lost (alignment …): searched 5.0 m …`.

AVLite publishes `recoveries` as `localization_recoveries` on `/avlite/controller_diagnostics`
(about 20 Hz). The counter starts at 0 when AVLite starts, and AVLite is restarted for every trial.

**Figure used in the tables:**

localization lost (per trial) = maximum of `localization_recoveries` over the trial's recording

Batch tables add the per-trial values.

**Limitations of this figure:**

- It counts **successful** recoveries only. A search that finds nothing increases
  `recovery_tries`, which is not recorded, so a car that stays lost through several failed
  searches may add little or nothing. The figure is a lower bound on how often the localizer
  was lost.
- It counts **events, not duration**. A loss of 0.3 s and one of 30 s each count once.
- **Respawns add to it.** With `--auto-reset`, a car that stops (usually after a crash) is
  moved back to the start; the localizer then loses track and usually recovers, so crashes and
  losses rise together.

## 3. Localization error (against simulator truth)

The simulator publishes the car's true pose. On every controller diagnostic (about 20 Hz) the
runner publishes the AVLite estimate and the true position, and the error is:

localization error = sqrt((x_estimate − x_true)² + (y_estimate − y_true)²)   (metres)

Its components along x and y and the absolute heading error are published as well
(`localization_x_error_m`, `localization_y_error_m`, `localization_yaw_error_rad`). The truth
pose is used only for this diagnostic; it never feeds the localizer or the controller.

**Figures used in the tables:** mean, median, 95th percentile (p95) and maximum of the
localization error over all diagnostic samples of a trial. The median shows typical accuracy;
p95 and maximum show how bad the worst moments were. After a respawn the error can briefly be
very large (the car moved, the estimate did not), which dominates the mean and maximum; the
median is the most robust figure.

This is the most reliable localization measure, but it exists only in the simulator: the real
car has no true pose to compare against, so on the car only alignment and the recovery counters
are available.

## 4. Recording and summary

`tools/sim-localization/measure_pose.py` records every 240 s trial to a JSON-lines file: simulator
truth, wheel odometry, IMU, controller commands, controller diagnostics (including
`localization_error_m` and `localization_recoveries`), lap and collision counters and, if asked,
raw LiDAR scans. The summary scripts compute, per trial:

| Figure | Calculation |
| --- | --- |
| Average speed | distance driven (sum of true-position steps, ignoring respawn jumps above 2 m) / recording time |
| Top speed | maximum true speed |
| Laps, lap time | increments of the simulator lap counter and the time between them |
| Collisions | increments of the simulator collision counter |
| Respawns | automatic resets by `--auto-reset` (car stopped for 3 s) |
| Localization lost | maximum of `localization_recoveries` (section 2) |
| Localization error | mean, median, p95, max of `localization_error_m` (section 3) |

## 5. Possible improvements to the metric

- Record `recovery_tries` as well, to count failed searches.
- Report time lost: seconds per trial with localization error above 0.5 m, or alignment below 0.5.
- Report figures excluding the few seconds after each respawn, so crashes are not counted twice.
