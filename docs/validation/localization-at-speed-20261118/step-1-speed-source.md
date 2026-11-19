# Step 1: fix the speed the controller sees

Status: **code fix done, unit-tested, and the before/after comparison now completed on a second PC
(see "Re-run on a 31 GB PC, 18 Nov afternoon"). The fix is correct but does not reduce crashes there,
because that simulator already ran near real time. The first PC's problems are kept below for the record.**

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

## Re-run on a 31 GB PC (18 Nov afternoon)

Setup: 31 GB RAM, Docker VM 15 GB (about 14 GB free the whole time). Simulator clock rate measured by
the odometry node: **0.92 simulated s per wall s** (the first PC: about 0.5). Five 60 s trials per
setting from the spawn point: `--clock-rate 1` (before) and `--clock-rate auto` (after). Data:
`step1/trials-before-after.json`, chart `step1/trials-before-after.png`.

| | before (clock 1) | after (auto) |
| --- | --- | --- |
| Trials with a wall hit | 4 of 5 (one stalled, see below) | 5 of 5 |
| Laps in total | 17 | 20 |
| Driving time (speed above 0.5 m/s) | 161 s | 189 s |
| Wheel-odometry speed / true speed | 0.96 | 0.98 to 0.99 |
| Seconds over the plan by more than 1 m/s | 1 to 2 % | 0 to 4 % |

**What this shows.** The fix does what it was written to do: the speed the controller sees moves from
0.96 to 0.98-0.99 of the true speed. But on this PC the simulator already ran at 0.92x, so there was
little to fix, and the crash numbers are statistically the same (5 trials each; 17 vs 20 laps is
inside the trial-to-trial spread, 2 to 6 laps). The car is also no longer driving about twice the
planned speed (Step 0's cause), yet it still hits walls 10 to 35 s in. So the speed-source error was
only the dominant crash cause when the simulator was slow; at near real time something else crashes
the car. The estimate is 0.2 to 0.9 m off the truth 5 s after the start (launch error, Step 3's topic).
I did not look at where these crashes happen; that is Step 4's re-measure.

**One stalled trial.** Before, trial 3 stopped after 3 s at (-3.3, -6.0) with no collision counted. The
diagnostics show the plan went invalid while the localization error was 0.01 m, so AVLite itself
braked. This is a different failure, not a crash, and I did not investigate it.

### Issues met in the re-run and how they were fixed

1. **Reset with `ros2 topic pub --once` is unreliable** (in the scripted trials it was dropped, or
   printed "failed to initialize wait set"). Replaced by `tools/sim-localization/respawn.py`, which
   holds the reset flag for 1.5 s and prints the car's position. It put the car at the spawn point
   (0.80, 3.16) every time.
2. **Fixed `sleep 14` after restarting AVLite.** The car starts driving as soon as the sensors are
   ready, so it had already crashed or finished by the time recording began. The script now waits for
   "Sensors ready" in the log and starts recording then.
3. **AVLite kept driving the car away after the respawn**, before the odometry restarted. The
   odometry origin and the localizer's start pose were then wrong: the estimate ended 4.2 m off and
   AVLite braked with "plan invalid". The script now stops AVLite, respawns, restarts odometry with
   the car at rest, then starts AVLite.
4. **Frozen-car check** (`measure_pose.py --require-motion-s 25`, exit code 3) aborted four earlier
   batches, which is how issues 1 to 3 were caught; no fake results were recorded. It also logs the
   Docker VM's free memory at the start and end of each recording (about 14.2 GB both times).
5. The reset did not freeze the car on this PC: 15 respawns, none wedged. The first PC's freeze after
   repeated resets was not reproduced, and its cause is still unknown.

### Limits of this evidence

Five trials per setting on one track, one PC. It cannot show a small improvement or a small harm. The
speed-ratio improvement (0.96 to 0.98) is measured well; the crash comparison is not conclusive.

## Sources

Step 1 relied on project files and the Step 0 measurements, not new outside research. Project files:
`src/avlite_autodrive/avlite_autodrive/sim_odometry.py`, `src/avlite_autodrive/test/test_sim_odometry.py`,
`tools/sim-localization/*`, `config/avlite.yaml`. The measurement method and the external sources for the
overall plan are in the "Sources" list at the end of [step-0-measurement.md](step-0-measurement.md).
