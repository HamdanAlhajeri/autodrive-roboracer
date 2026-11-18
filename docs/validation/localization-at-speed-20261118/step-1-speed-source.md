# Step 1: fix the speed the controller sees

Status: **code fix done and checked live and in unit tests; the planned before/after trial comparison
could not be completed** (see "What did not work"). Not committed as a validated result: treat the
crash-rate improvement as unproven until the re-run described at the end.

## Summary

Step 0 found that the car crashes with ICP localization mostly because the controller thinks it is
going about half as fast as it is. The wheel-odometry node turns encoder angles into speed by dividing
distance by the difference of ROS message stamps (wall clock), but the simulator runs at about 0.5x
real time, so each simulated second takes about two wall seconds. Speed and yaw rate came out about
half of the true values, and the controller kept pushing throttle.

Fix: `sim_odometry.py` now measures the simulator clock rate and works in simulated time.

## What was done

- New class `ClockRate` in `src/avlite_autodrive/avlite_autodrive/sim_odometry.py`. Over 0.5 s windows
  it computes `distance moved / (stamp time x mean reported speed)` from the truth topic, skips slow or
  implausible windows (below 1.5 m/s, ratios outside 0.2 to 1.5, resets), and smooths the result.
  Only this one number is taken from the truth topic, never a pose or a speed, so the "no ground truth
  in the localizer" property is kept.
- `WheelOdometry.time_scale` multiplies the stamp difference, so speed and gyro integration use
  simulated seconds. Default 1.0 (no change on a real car or a real-time simulator).
- New option `--clock-rate auto|N` (default `auto`; `1` turns it off). Logged: "Simulator clock rate X
  simulated s per wall s".
- 5 new unit tests in `src/avlite_autodrive/test/test_sim_odometry.py` (speed on a slow clock, gyro in
  sim time, clock measurement, following a change, ignoring teleports and bad samples): 11 tests pass.
- Tools for the comparison: `tools/sim-localization/run_trials.sh` (independent trials) and
  `summarize_trials.py`.

## Results that are solid

- Unit tests: 11 pass.
- Live check during the session (not saved as a file): ratio of wheel-odometry speed to true speed
  went from about 0.49 to about 0.97, and the odometry measured a clock rate of about 0.56.
  Step 0 had the same 0.46 to 0.50 factor measured independently.

## What did not work (issues and what happened)

1. **The trial method freezes the car.** `run_trials.sh` respawns the car with the reset command, then
   restarts odometry and AVLite. In the final runs the car stayed at the spawn point in most trials
   (peak speed 0 in 5 of 6 "before" and 2 of 3 "after" trials). Earlier in the session the same thing
   happened outside the script: after many resets the wheels spun and the front wheels steered but the
   body did not move, even with throttle sent straight to the simulator. Restarting containers or
   reset did not help; closing the simulator and a full `avlite.ps1 stop` then `start` did. So the
   reset command, or repeated resets, seems to wedge the simulator physics. Cause not found.
2. **Docker ran out of memory.** The laptop has 7.5 GB of RAM and about 0.5 GB was free. Docker's
   engine crashed ("out of memory allocating heap arena map", later "runtime.newosproc"), which cut
   the "after" runs to 3 complete trials and left Docker stopped. Python also hit
   "Insufficient system resources" while importing. OneDrive (about 0.6 GB), Windows memory
   compression (1.3 GB) and the simulator compete with Docker. This is a likely contributor to the
   slow simulator, but it was not isolated.
3. **My first summary was wrong to trust.** The first "before/after" run showed zero driving in all
   trials; I found the car frozen, restarted the whole stack, and re-ran. The re-run still has the
   frozen trials in item 1, so the numbers in `step1/trials-invalid-car-frozen.json` are kept only to
   show what happened and are not a result. The comparison chart was deleted because it would have
   suggested a measurement that was not made.

## Not fixed in this step

- Why the simulator runs at about half real time (memory pressure suspected; not proven).
- The freeze after resets.
- A before/after crash comparison with the new odometry.

## How to finish the check (next agent or owner)

1. Free memory first: pause OneDrive, close browsers, keep Docker's memory limit modest.
2. Start from a clean start (`.\avlite.ps1 stop`, `.\avlite.ps1 start`, select Connection and Autonomous).
3. Do not respawn with the reset command between trials. Instead run one long recording per setting
   (`--clock-rate 1`, then `--clock-rate auto`) of 5 minutes each with
   `tools/sim-localization/measure_pose.py` and compare crashes and speed against the plan with
   `analyze_pose_measure.py`, as in Step 0.
4. Success criterion: speed seen by the controller within about 10 percent of true speed (met live),
   and fewer wall hits than Step 0's 7 in 93 s of driving.

## Sources

Step 1 relied on project files and the Step 0 measurements, not new outside research. Project files:
`src/avlite_autodrive/avlite_autodrive/sim_odometry.py`, `src/avlite_autodrive/test/test_sim_odometry.py`,
`tools/sim-localization/*`, `config/avlite.yaml`. The measurement method and the external sources for the
overall plan are in the "Sources" list at the end of [step-0-measurement.md](step-0-measurement.md).
