# AGENT.md: notes for the next agent

Read this first, then `HANDOVER.md` (car and platform) and
`docs/validation/localization-at-speed-20261118/PLAN.md` (the plan and its status).

## Working rules from the owner

- Do ONE plan step, write a short document for it (what was done, issues and how they were fixed,
  screenshots, all sources), then stop and wait for the owner's approval before the next step.
- Do not commit or push unless asked. No AI attribution lines in commits or pull requests.
- Never kill all `msedge.exe` or other processes you did not start. Do not leave background
  `find /` running. In Windows PowerShell 5.1 never redirect docker stderr with `2>&1`.
- Be honest about unproven results: Step 1's before/after comparison is NOT done (see below).

## Goal

The simulated car crashes less with AVLite's own LiDAR localization (ICP), at current speeds, and keeps
working at higher speeds later. The pose-view plugin (`PoseComparisonView`, http://localhost:8765 via
`.\avlite.ps1 live`) shows estimated vs true pose in the browser.

## Plan and status (details: `docs/validation/localization-at-speed-20261118/`)

| Step | What | Status | Write-up |
| --- | --- | --- | --- |
| 0 | Measure delay and localization error | Done | `step-0-measurement.md` |
| 1 | Fix the speed the controller sees (simulator clock) and re-run the ICP vs ground-truth comparison | Code done and unit-tested; live speed ratio 0.49 to 0.97; before/after crash comparison NOT completed | `step-1-speed-source.md` |
| 2 | Reset and recovery: odometry in step with a respawn, no re-seed on a short sensor gap, recovery search in the simulator runner | Not started (awaiting owner approval) | |
| 3 | Launch error: limit the estimate's lead over wheel odometry while accelerating; use scan alignment for trust | Not started | |
| 4 | Re-measure at real time; then latency compensation, alignment-based speed limit, planner margins, better matcher as the data justify | Not started | |

## Key findings so far

- Ground-truth pose: 19 clean laps in 5 min. ICP pose: 7 wall hits in 93 s, 1 lap.
- At 6 of 7 crashes the estimate was accurate (1.5 to 15 cm); the car was doing 6.3 to 7.7 m/s where the
  plan wanted 3.0 to 3.7, because wheel-odometry speed read about half the true speed. Cause: the
  simulator runs at about 0.46 to 0.55x real time while ROS stamps are wall clock. Fixed in
  `sim_odometry.py` (`ClockRate`, `--clock-rate auto`).
- After a launch the estimate runs about 1.4 m ahead of the car (up to 3.8 m) for a few seconds.
- After a reset the localizer started lost in 6 of 6 cases.
- Estimate lag is about 10 ms, so latency compensation is not the first fix.

## Known environment problems (Step 1 hit these)

- The laptop has 7.5 GB RAM; with OneDrive, the simulator and Docker the free memory dropped to about
  0.5 GB and Docker's engine crashed mid-run. Free memory (pause OneDrive) before long runs.
- After the reset command (`/autodrive/reset_command`), the simulator car sometimes stays frozen: wheels
  spin, body does not move, even with throttle sent directly. Fix: close the simulator, then
  `.\avlite.ps1 stop` and `.\avlite.ps1 start`, select Connection and Autonomous. So do not respawn
  between trials; use one long recording per setting.
- Real-car readiness: the speed fix is simulator-only; real encoder, gyro and LiDAR behaviour is
  unmeasured. See the discussion in `step-1-speed-source.md`.

## Tools

- `tools/sim-localization/measure_pose.py` (records truth, estimate, odometry, commands; runs in the
  avlite container), `analyze_pose_measure.py` (crashes, speed vs plan, real-time factor, error, delays,
  charts), `run_trials.sh` and `summarize_trials.py` (independent trials; see the freeze issue above).
- Recordings go in `log/` (git-ignored); screenshots and results next to each step's write-up.

## Sources

(Copied from `step-0-measurement.md`; each later step's write-up adds its own. The first two links in
that list were opened directly; the rest were read from search summaries.)

I opened the first two links directly (the paper abstract and the repository README). Everything else
was read only as search-result summaries and should be checked before it is quoted.

Localization, racing stacks and benchmarks:
- [Robustness Evaluation of Localization Techniques for Autonomous Racing (SynPF vs Cartographer, up to 7.6 m/s)](https://arxiv.org/abs/2401.07658)
- [ForzaETH particle filter (SynPF): code and tuning notes](https://github.com/ForzaETH/particle_filter)
- [Unifying F1TENTH Autonomous Racing: Survey, Methods and Benchmarks](https://arxiv.org/abs/2402.18558)
- [CDDT: Fast Approximate 2D Ray Casting for Accelerated Localization](https://www.arxiv-vanity.com/papers/1705.01167/)
- [Fast and Modular Autonomy Software for Autonomous Racing Vehicles](https://arxiv.org/pdf/2408.15425)

Delay compensation and control:
- [Stellenbosch University write-up: simulation to reality, pure pursuit, MPC and MPCC on the F1TENTH platform](https://www.su.ac.za/faculties/engineering/departments/electrical-electronic-engineering/news/simulation-reality-assessing-efficacy-pure-pursuit-mpc-and-mpcc-f1tenth-platform)
- [A critical evaluation of Pure Pursuit, MPC and MPCC (the underlying paper)](https://www.maratto.africa/research/a-critical-evaluation-of-pure-pursuit-mpc-and-cf1725da31)
- [TUM autonomous motorsport software for the Indy Autonomous Challenge](https://arxiv.org/pdf/2205.15979)
- [Patent: vehicle controller robust to time delays (pose extrapolation)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10800427)
- [Control strategies for autonomous vehicles (pure pursuit lookahead)](https://arxiv.org/pdf/2011.08729)
- [F1TENTH pure-pursuit lookahead paper, 2026 (found by search; the summary I read may describe a neighbouring result)](https://www.arxiv.org/pdf/2602.18386)

Scan matching, deskewing and fusion:
- [DAMM-LOAM: degeneracy-aware multi-metric LiDAR odometry and mapping](https://arxiv.org/html/2510.13287)
- [Kilometer-scale autonomous navigation in subarctic forests (deskewing with wheel odometry and IMU)](https://arxiv.org/pdf/2111.13981)
- [Direct LiDAR-Inertial Odometry: continuous-time motion correction](https://arxiv.org/pdf/2203.03749)
- [A comprehensive evaluation of LiDAR odometry techniques](https://arxiv.org/pdf/2507.16000)
- [Sensor fusion and robot localization using ROS 2 Jazzy (robot_localization)](https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/)

Localization-aware speed:
- [Kibalov and Shipitko: speed control under ego-pose uncertainty](https://arxiv.org/abs/2003.00675)

AVLite (the framework the plugin and this analysis build on):
- [AV-Lab/avlite at the pinned revision](https://github.com/AV-Lab/avlite/tree/1653f592b2eb5289a14c0d1041e50fdea855322a) (task strategy, executer and app layers)

Project files used: `HANDOVER.md`, `docs/validation/overnight-20261118/README.md`,
`docs/validation/sim-localization-20261117/README.md`, `config/avlite.yaml`,
`src/avlite_autodrive/avlite_autodrive/sim_odometry.py`, `avlite_localization.py`, `runner.py`.
