# Handoff: Rewrite racer_node.py from Scratch — Geometric Pure Pursuit

## Task

Delete the entire body of `racer_node.py` and rewrite it from scratch.
Do NOT carry over any logic from the previous version. Start clean.

---

## References (read these before writing any code)

1. **Medium article** — Pure Pursuit section only (ignore Stanley and MPC):
   https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081

2. **RoboRacer Learn** — Module D, Lecture 10 only:
   https://roboracer.ai/learn

---

## Algorithm to implement: Geometric Pure Pursuit

### Core geometry (from the video lecture)

The vehicle sits at the origin facing forward (x-axis). The goal point in the
vehicle's local frame is `(gx, gy)`. A circle is inscribed that passes through
both the vehicle and the goal point, with its center constrained to the y-axis
(perpendicular to the vehicle heading). Two simultaneous equations:

```
r = |gy| + d          (circle centre is d past the goal point along y)
d² + gx² = r²         (Pythagorean right triangle)
```

Substituting and solving:

```
(r - |gy|)² + gx² = r²
r² - 2r|gy| + gy² + gx² = r²
gx² + gy² = 2r|gy|        (note: gx² + gy² = L², the lookahead distance squared)
r = L² / (2|gy|)
curvature κ = 1/r = 2|gy| / L²
steering angle δ = arctan(κ * L_wheelbase)
```

`d` is an intermediate variable only — it is never used in the final code.

### What `gx` and `gy` are in this simulator

There is no pre-built reference path. The vehicle uses LiDAR to estimate the
track centerline in real time:

- Parse the 270° LaserScan into cartesian points (car frame: x forward, y left)
- Separate forward-facing left wall points and right wall points
- Estimate `gy` (lateral offset of centerline from vehicle) from wall distances
- Set `gx = LOOKAHEAD_DIST` (project straight ahead)
- `L² = gx² + gy²`

### Steering formula

```python
L_sq = gx**2 + gy**2
if abs(gy) < 1e-6:
    steering = 0.0
else:
    curvature = (2.0 * abs(gy)) / L_sq
    delta = math.atan(curvature * WHEELBASE)   # bicycle model steering angle
    steering = math.copysign(delta, gy)
    steering = float(np.clip(steering / MAX_STEER_RAD, -1.0, 1.0))  # normalise to [-1,1]
```

---

## File structure to implement

### Constants (tunable at top of file)

```python
LOOKAHEAD_DIST   = 0.8    # metres — lookahead distance L
WHEELBASE        = 0.32   # metres — RoboRacer wheelbase (approx)
MAX_STEER_RAD    = 0.4    # radians — max physical steering angle for normalisation
MAX_THROTTLE     = 0.4    # top speed on straights [0, 1]
MIN_THROTTLE     = 0.1    # minimum speed in corners
THROTTLE_DECAY   = 3.0    # corner braking aggressiveness
WALL_CLIP_DIST   = 4.0    # clip LiDAR beyond this distance (metres)
EMA_ALPHA        = 0.3    # smoothing factor for gy estimate (0=no update, 1=no filter)
```

### ROS 2 topics (unchanged from original)

| Topic | Direction | Type |
|---|---|---|
| `/autodrive/roboracer_1/lidar` | Subscribe | `sensor_msgs/LaserScan` |
| `/autodrive/roboracer_1/throttle_command` | Publish | `std_msgs/Float32` |
| `/autodrive/roboracer_1/steering_command` | Publish | `std_msgs/Float32` |

### Class structure

```
RacerNode(Node)
├── __init__()                        — subscriptions, publishers, state
├── _lidar_cb(msg)                    — entry point each scan
├── _estimate_gy(ranges, angles)      — returns smoothed lateral offset gy
├── _pure_pursuit_steer(gx, gy)       — returns normalised steering [-1,1]
└── _speed_from_steer(steering)       — returns throttle [MIN, MAX]
```

### `_estimate_gy` logic

```python
def _estimate_gy(self, ranges, angles):
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)

    fwd = xs > 0.05
    left_pts  = ys[fwd & (ys >= 0)]
    right_pts = ys[fwd & (ys <  0)]

    left_dist  = float(np.median(left_pts))   if len(left_pts)  > 0 else 1.0
    right_dist = float(np.median(-right_pts)) if len(right_pts) > 0 else 1.0

    # centerline y-offset in vehicle frame (positive = centerline is to the left)
    gy_raw = (left_dist - right_dist) / 2.0

    # EMA smoothing
    self._gy_filtered = (1.0 - EMA_ALPHA) * self._gy_filtered + EMA_ALPHA * gy_raw
    return self._gy_filtered
```

Note: `gy` here is the **signed lateral offset to the centerline**, not a
normalised error ratio. This maps directly to the `gy` in the geometric
derivation above.

---

## What NOT to do

- Do NOT use `atan2(ly, lx)` for steering — that is a heading angle, not curvature
- Do NOT use a PD controller or derivative terms — the geometry handles correction naturally
- Do NOT carry over the old `center_error` ratio approach — use raw metric `gy` in metres
- Do NOT add gap-follow or algorithm switching — pure pursuit only
- Do NOT add hardcoded correction jumps like `±0.4`

---

## Tuning notes for after first run

| Symptom | Adjustment |
|---|---|
| Understeer / drifts wide in corners | Decrease `LOOKAHEAD_DIST` |
| Oversteer / oscillates | Increase `LOOKAHEAD_DIST` |
| Slow to re-centre on straights | Increase `EMA_ALPHA` |
| Twitchy / noisy steering | Decrease `EMA_ALPHA` |
| Spins out at corners | Decrease `MAX_THROTTLE` or increase `THROTTLE_DECAY` |