# Localization run `avlite`

**Result: DRIVEN CLEAN** - laps 3, collisions 0, 856 control steps over 43.0 s.

## Numbers

| Measure | Median | p95 | Max |
| --- | --- | --- | --- |
| Position error (m) | 0.105 | 0.391 | 0.573 |
| Yaw error (rad) | 0.019 | 0.090 | 0.192 |
| Scan alignment (share of hits on the map) | 1.000 | 1.000 | 1.000 |
| Controller step (ms) | 2.618 | 4.905 | 16.469 |

Controller overruns: 0 of 856 steps.

## What happened

- **Error jitter, not drift.** 37 short excursions above 0.3 m (peak 0.57 m, longest 0.9 s), and the error came back every time. They cluster near lap positions 0, 3, 6, 15, 18, 21, 24 m. The car stayed on its line, so this is noise, but it caps how fast it can safely go.

## Timeline (every 2 s)

| t (s) | Lap pos (m) | Speed target | Error (m) | Yaw err (rad) | Alignment | Path dev (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 0.8 | 2.5 | 0.00 | 0.000 | 1.00 | -0.01 |
| 2 | 4.1 | 2.5 | 0.23 | 0.013 | 0.99 | -0.01 |
| 4 | 8.4 | 2.5 | 0.14 | 0.028 | 1.00 | 0.00 |
| 6 | 12.6 | 1.5 | 0.03 | 0.081 | 1.00 | 0.05 |
| 8 | 16.8 | 2.5 | 0.32 | 0.037 | 0.97 | -0.02 |
| 10 | 21.2 | 2.5 | 0.29 | 0.080 | 0.97 | -0.04 |
| 12 | 25.9 | 2.5 | 0.01 | 0.005 | 1.00 | 0.02 |
| 14 | 2.1 | 2.5 | 0.33 | 0.025 | 0.95 | -0.05 |
| 16 | 6.6 | 2.5 | 0.05 | 0.008 | 1.00 | -0.01 |
| 18 | 11.3 | 2.5 | 0.04 | 0.031 | 1.00 | 0.02 |
| 20 | 15.7 | 2.5 | 0.21 | 0.069 | 1.00 | 0.02 |
| 22 | 20.3 | 2.5 | 0.12 | 0.029 | 1.00 | -0.01 |
| 24 | 24.8 | 2.5 | 0.02 | 0.028 | 1.00 | 0.01 |
| 26 | 0.6 | 2.5 | 0.22 | 0.002 | 1.00 | 0.05 |
| 28 | 5.4 | 2.5 | 0.10 | 0.011 | 1.00 | -0.01 |
| 30 | 10.0 | 2.5 | 0.00 | 0.008 | 1.00 | -0.00 |
| 32 | 13.9 | 2.5 | 0.06 | 0.003 | 1.00 | 0.04 |
| 34 | 18.0 | 2.5 | 0.06 | 0.004 | 1.00 | -0.01 |
| 36 | 21.9 | 2.5 | 0.41 | 0.155 | 0.90 | -0.05 |
| 38 | 26.7 | 2.5 | 0.00 | 0.012 | 1.00 | 0.00 |
| 40 | 3.1 | 2.5 | 0.30 | 0.034 | 0.95 | -0.02 |
| 42 | 7.7 | 2.5 | 0.08 | 0.021 | 1.00 | -0.00 |

![Path and error](./avlite-analysis.png)
