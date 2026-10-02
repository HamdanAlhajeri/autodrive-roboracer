# Above 10 m/s on the practice track — 22 September 2026

The requested outcome is actual speed above 10 m/s while racing the existing
practice track. It has not been demonstrated. Raising a ceiling alone does not
produce that result with the current map and vehicle-response limits.

## Implemented

- Shared ceiling: 12.0 m/s for planner, controller and actuator.
- Throttle cap: 0.60. The existing feedforward term requests 0.48 at 12 m/s;
  the old 0.25 cap would clip that demand. This is control headroom, not a
  measured throttle-to-speed calibration.
- Speed-aware odometry continuity in both ROS adapters: a 1.2 m displacement
  over 0.1 s at 12 m/s no longer causes a false simulator reset. Displacement
  beyond the speed/time allowance and gaps above the watchdog still reset state.
- Runtime logs show the validated profile's minimum and maximum, separately
  from the shared ceiling. `avlite.ps1 speed` reports the same map constraints
  offline and checks that planner and actuator ceilings agree.
- Curved-path obstacle checks use a spatial index to discard distant LiDAR
  hit/vehicle-pose pairs before the exact capsule calculation. The swept route,
  reaction arc, vehicle radius, sampling margin and stopping distance are
  unchanged. Regression checks compare against the full pairwise calculation
  and explicitly exercise both axle end caps.

After a qualified 2.5 m/s response run, the next profile uses 2.0 m/s²
acceleration and 3.0 m/s² braking, within the actuator's existing acceleration
command limits. Braking is below both the prior 3.26 and new 3.76 m/s²
low-speed suggestions. The lateral limit stays at 3.0 m/s². Obstacle stopping,
vehicle clearance and watchdogs remain active. These low-speed measurements
do not establish braking at 10 m/s.

## Offline result

Using the installed pinned AVLite and committed practice map:

| Shared ceiling | Acceleration / braking (m/s²) | Planned minimum | Planned maximum |
| --- | --- | --- | --- |
| 3.0 m/s | 1.5 / 2.0 | 1.772 m/s | 3.000 m/s |
| 12.0 m/s | 1.5 / 2.0 | 1.772 m/s | 5.043 m/s |
| 12.0 m/s | 2.0 / 3.0 | 1.772 m/s | 5.698 m/s |

The planned lap is 27.776 m long. At the configured acceleration, reaching
10 m/s from rest requires 25.00 m. At the configured braking and 0.25 s reaction
time, stopping from 10 m/s requires 19.17 m. These are illustrative constant
acceleration calculations; the planner also accounts for individual corners
and maintains acceleration/braking constraints across the finish line.

Consequently, the current profile never commands 10 m/s on this track. Achieving
the requested speed needs measured acceleration/braking and grip that support
a faster plan. Inventing stronger limits would only hide this constraint.

## Verification and live evidence

The initial changes passed 82 sensor/configuration/actuator/planner/ROS tests.
After the obstacle-check optimization and additional callback regressions,
35 planner/ROS tests passed; lint passed. A synthetic large-radius map verifies
that AVLite produces targets above 10 m/s, including across the finish line.
That synthetic result is not evidence of 10 m/s on the practice track.

An isolated 20-repeat benchmark with 1,080 hits and 600 swept poses returned
the same clearance with both implementations. Median calculation time fell
from 66.89 ms to 5.96 ms (p95: 70.14 to 6.79 ms). These are synthetic timings
on this host; live timings are evaluated separately.

`20260922-111843-398-4-laps` completed four clean laps at the temporary 2.5 m/s
response setting, reaching 2.5098 m/s. All three coast fits were rejected for
position/time inconsistency. No higher braking limit was derived from that run.

`20260922-112357-203-ceiling-12` completed three clean laps with the 12 m/s
shared ceiling before the clearance optimization. Measured peak speed was
4.5892 m/s; wall-clock rolling laps were 25.1875 and 24.5407 s. Controller
overruns were observed, motivating the obstacle-check optimization.

Docker interrupted the first optimization verification and stopped the bridge
and actuator after the completed recording. The isolated tests were rerun
successfully after recovery; the native simulator and services were restarted.

The repeat `20260922-134734-473-ceiling-12-fast-clearance` completed three
clean laps with a 4.1409 m/s peak and rolling laps of 11.0930 / 11.1712 s.
For fresh moving telemetry samples, median controller step time fell from
76.67 to 4.59 ms, p95 from 92.90 to 8.16 ms, and sampled overruns from 100%
to 0%. The repeat's loop p95 was 50.51 ms; path-error p95 was 0.0177 m.
The simulator was restarted in a 960×540 window, and its timing recovered,
so the wall-clock lap-time change is not an isolated algorithm comparison.

`20260922-134928-707-4-laps` then completed four clean response laps and
qualified all three coast fits. Conservative braking suggestion: 3.7577 m/s²;
position/integrated-distance ratios: 0.995, 1.035 and 0.995. Sampled coast-to-zero
feedback delays were 49, 47 and 70 ms. This qualifies the 2.5 m/s test stage,
not a 10 m/s braking claim.

The final screen `20260922-135252-011-ceiling-12-a2-b3` used 2.0 m/s²
acceleration and 3.0 m/s² braking. It passed three clean laps with zero
collisions/resets, reaching **4.5134 m/s**. Rolling laps were **10.9304 and
11.1278 s**, mean **11.0291 s**. AVLite was stopped automatically afterward.
Fresh moving samples had controller-step p95 **9.07 ms**, loop p95 **50.34 ms**,
zero sampled overruns, path-error p95 **0.0149 m**, and peak throttle **0.1836**.
The 0.60 throttle ceiling was not the binding limit during this run.
This is one three-lap screen, not final repeatability acceptance or 10 m/s racing.

Remaining work is representative higher-speed response/grip measurement and
controller target tracking. The current map profile's 5.698 m/s maximum itself
precludes a 10 m/s target. The qualified low-speed result does not justify
arbitrarily multiplying the planner's acceleration, cornering or braking limits.

## Commands and rollback

```powershell
# Offline only; no simulator connection or driving commands.
.\avlite.ps1 speed

# Record the current profile, stopping after three laps or an incident.
.\avlite.ps1 laps -Laps 3 -Label ceiling-12 -NoTrackMap
```

The recording wrapper reloads both processes. For the previous experimental
profile, restore `speed_mps: 3.0` and `max_throttle: 0.25` in
`config/driving.yaml`, planner/controller acceleration to 1.5 m/s² and
braking to 2.0 m/s² in `config/avlite.yaml`, then restart both processes or use
the recording wrapper.
For uncalibrated commissioning, also set `braking_calibrated: false`.
