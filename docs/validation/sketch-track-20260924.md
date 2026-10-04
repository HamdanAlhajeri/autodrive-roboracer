# Sketch simulator integration — 24 September 2026

Source project: `C:\Users\Xxthe\AutoDRIVE-Unity`, AutoDRIVE-Simulator revision
`75b7df5216524a3c6b3150d698139190c6c44f8f`, Unity Editor **2022.3.52f1**.
The generated scene is `Assets/Scenes/RoboRacer - Sketch Track.unity`.
The desktop player uses Mono, Direct3D 11 and HDRP forward rendering, with
automatic XR startup disabled.

## Scene checks

The generated scene passed these checks in Unity:

- Three meshes, read using the original OBJ's literal coordinates.
- Static non-convex road/barrier colliders; road raycasts every 0.25 m along the plan.
- Sixteen checkpoint gates intersecting both physical barriers.
- A 48.29113 m route with the rear axle at Unity `(1.285887, 0.060000, -9.925652)`
  and vehicle yaw `86.957466` degrees.
- LiDAR head height `0.156 m`, inside the adapted `0.3048 m` barriers.
- Map SHA-256 `c4068362337e14f124d3f0f85c6c400b1deda416ebb4cb3a57ada3d3f1df8662`.

The original barriers were only 0.1524 m tall. Their scene transforms scale Y
by 2 so the stock LiDAR can detect them; their horizontal footprint and the map
are preserved. Checkpoint raycasts select track barriers explicitly, avoiding
the vehicle colliders at the spawn gate.

The scene report is saved under `log/unity-build/scene-validation.json` and in
the source project as `sketch-scene-validation.json`.

## Native validation

The Windows player built successfully. The final visible standalone check passed
(`log/unity-build/smoke-20260924-180446/report.json`):

| Check | Result |
| --- | --- |
| Spawn displacement after settling | 0.000613454 m |
| Grounded wheels | 4 |
| Finite LiDAR hits | 1,065 |
| Consecutive lap-counter sequences | 1, then 2 |
| Reset position error | 0.000008702278 m |

The full-track camera view was inspected in the saved `report.png`. The reset
test explicitly waits for `ResetManager.Update` to consume its flag; counting
only physics ticks can finish too early during a slow rendered frame.

The normal launcher opened the selected sketch executable. AVLite regenerated
and validated its 48.29 m plan at **1.822–2.500 m/s**. The actuator confirmed
**2.500 m/s** and **0.200 maximum throttle**. The native simulator starts in manual
mode; Connection and Autonomous must be selected before driving.

Practice settings are backed up under `log/track-backup-20260924-180547`.
Autonomous laps have not been tested. The runtime check uses controlled
checkpoint crossings; it does not establish clean-lap driving performance or
collision/respawn behavior under motion.
