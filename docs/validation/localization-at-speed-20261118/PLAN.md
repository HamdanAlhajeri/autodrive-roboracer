# Localization at speed: plan and status

The single place to see every step, its status and where its write-up is. Update this file when a
step starts or finishes. Working rule agreed with the owner: **do one step, write it up (what was done,
issues, how they were fixed, screenshots, sources), then stop and wait for approval before the next.**
Nothing is committed unless the owner asks; no Claude attribution in commits.

Goal: the car crashes less with AVLite's own LiDAR localization (ICP), at the current speeds,
and keeps working at higher speeds later.

| Step | What | Status | Write-up |
| --- | --- | --- | --- |
| 0 | Measure delay and localization error | **Done** (18 Nov) | [step-0-measurement.md](step-0-measurement.md) |
| 1 | Fix the speed the controller sees; get the simulator back to real time; re-run the ICP vs ground-truth comparison | **Done** (18 Nov): fix correct (speed ratio 0.96 to 0.98), no crash reduction on a near-real-time sim | [step-1-speed-source.md](step-1-speed-source.md) |
| 2 | Reset and recovery: odometry stays in step with a respawn, no re-seed on a short sensor gap, recovery search in the simulator runner | **Done** (18 Nov): re-seed bug fixed, recovery added; blind travel of 3 m or more still lost | [step-2-reset-and-recovery.md](step-2-reset-and-recovery.md) |
| 3 | Launch error: limit how far the estimate may lead wheel odometry while accelerating from a stop; use scan alignment to decide trust | **Done** (18 Nov): the lead is real (wheel slip) but limiting it made crashes about five times worse; left off | [step-3-launch-error.md](step-3-launch-error.md) |
| 4 | Re-measure at real time, then the earlier ideas as the data justify: latency compensation, alignment-based speed limit, planner margins from measured error, better matcher | **Done** (18 Nov): true pose still far better at real time; no setting helped; failures located (one corner, permanent invalid-pose stop) | [step-4-real-time-remeasure.md](step-4-real-time-remeasure.md) |

## What step 0 found (the reason for this order)

- Ground-truth pose: 19 clean laps in 5 minutes. ICP pose: 7 wall hits in 93 s of driving, 1 lap.
- At 6 of 7 crashes the estimate was accurate (1.5 to 15 cm). The car was doing 6.3 to 7.7 m/s where
  the plan wanted 3.0 to 3.7, because the controller reads speed from wheel odometry and that reads
  about half the true speed (the simulator runs at about 0.46 to 0.50 of real time).
- After a launch the estimate runs 1.4 m ahead of the car (up to 3.8 m) for a few seconds.
- After a reset the localizer started lost in 6 of 6 cases.
- Estimate lag is about 10 ms: latency compensation is not the first fix.

## Tools

- `tools/sim-localization/measure_pose.py`: records truth, estimate, odometry, commands, counters (runs in the avlite container).
- `tools/sim-localization/analyze_pose_measure.py`: crashes, speed against plan, real-time factor, error, delays, charts.
- Recordings go in `log/step*-measure/` (git-ignored). Screenshots and `results.json` go next to each step's write-up.
- Windows PowerShell 5.1: never redirect docker stderr with `2>&1`.
- Do not kill all `msedge.exe`; stop only a process you started. Do not leave background `find /` running.

## Sources

Listed in full at the end of [step-0-measurement.md](step-0-measurement.md); each later write-up adds its own.
