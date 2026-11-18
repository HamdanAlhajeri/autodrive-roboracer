# Localization run `slam`

**Result: FAILED: collision** - laps 0 of 3, collisions 1, 161 control steps over 8.1 s.

## Numbers

| Measure | Median | p95 | Max |
| --- | --- | --- | --- |
| Position error (m) | 1.077 | 1.137 | 1.201 |
| Yaw error (rad) | 0.176 | 0.242 | 0.271 |
| Controller step (ms) | 2.251 | 4.937 | 9.334 |

Controller overruns: 0 of 161 steps.

## What happened

- **Pose was lost.** Error passed 0.3 m at t=2.4 s, 4.5 m along the lap, on a corner (planned speed 2.5 m/s, actual target 2.5 m/s).
- **Collision.** Run ended at t=8.1 s with position error 1.14 m, 7.0 m along the lap, speed target 0.0 m/s, path deviation -0.22 m.
- **Commands withheld.** The bridge stopped the car: scan does not align with the map (x2), no map -> base_link transform (x1), localization stable for only 0.4s (x1).

## Timeline (every 2 s)

| t (s) | Lap pos (m) | Speed target | Error (m) | Yaw err (rad) | Alignment | Path dev (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 1.0 | 2.5 | 0.14 | 0.021 | - | 0.07 |
| 2 | 3.8 | 2.5 | 0.20 | 0.037 | - | 0.01 |
| 4 | 6.5 | 1.0 | 1.14 | 0.238 | - | -0.04 |
| 6 | 7.1 | 0.0 | 1.11 | 0.176 | - | -0.23 |
| 8 | 7.0 | 0.0 | 1.14 | 0.188 | - | -0.22 |

## Simulator photos (11 saved in `photos/`)

![Simulator screenshots](./slam-photos.png)
