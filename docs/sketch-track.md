# Sketch track

**Current selection, 30 September 2026:** the project uses the original
practice track. The sketch assets and commissioning profile are retained
for later use. The unused native sketch build was removed to save disk space;
run `.\avlite.ps1 sketch-build` before selecting this track again.
The last active sketch settings were saved locally; their backup
directory is recorded in `log/unity-build/sketch-backup.txt`. Use the activation
steps below to select this track again.

The original track meshes and boundary metadata are kept in
[`assets/tracks/sketch_track`](../assets/tracks/sketch_track). All eight files
were verified identical to the supplied ZIP before the redundant archive was
removed. The extracted files are sufficient for a checkout.
These assets are not a runnable simulator. The installed
practice executable cannot load this OBJ as a replacement scene.

**Status on 24 September 2026:** the AVLite map and a 2.5 m/s commissioning plan
pass offline validation. Unity Editor 2022.3.52f1 and the AutoDRIVE source project
are installed, and the custom Windows simulator is built and selected.
Visible runtime checks passed for wheel contact, LiDAR, consecutive lap-counter
cycles and reset placement. No autonomous lap has been driven on this track.
At that validation date, the active profile used the sketch map,
`speed_mps: 2.5` and `max_throttle: 0.2`. AVLite and the actuator confirmed those
limits at startup. Read `config/` and run `.\avlite.ps1 speed` for current settings.

Run `.\avlite.ps1 start`, then select **Connection** and **Autonomous** in the
simulator. The practice settings from this switch are preserved in
`log/track-backup-20260924-180547`; its path is also saved in
`log/unity-build/practice-backup.txt`. See the
[native validation record](validation/sketch-track-20260924.md).

## Prepared files

| File | Purpose |
| --- | --- |
| `assets/tracks/sketch_track/sketch_track.obj` and `.mtl` | Original road and two barrier meshes |
| `config/maps/sketch.json` | AVLite boundaries and initial reference route in world metres |
| `config/avlite.sketch.yaml` | Commissioning profile, copied to the active `avlite.yaml` during activation |
| `config/windows-simulator.json` | Selected native executable used by the normal launcher |
| `config/maps/sketch.validation.json` | Offline checks, planned speed range and proposed spawn pose |
| `config/maps/sketch.png` | Generated map and racing-line preview |
| `config/maps/sketch.plan.json` | Generated plan with resolved settings; regenerated before driving |
| `avlite.ps1 sketch-map` | Repeat conversion and validation without starting the car |
| `avlite.ps1 sketch-build` | Create the Unity scene and build its Windows player |
| `avlite.ps1 sketch-test` | Run isolated native scene checks and save a screenshot/report |
| `tools/unity/Editor/SketchTrackBuilder.cs` | Mesh import, spawn, checkpoints, scene validation and build |
| `tools/unity/Runtime/SketchTrackSmokeCheck.cs` | Optional standalone checks for ground contact, LiDAR, lap triggers and reset |

The original ZIP SHA-256 is
`05d5a9b58dfb7d14a9ff344c6072fcb44deb64261c8e69046a0ec5c8830db527`.
The map records its source metadata hash. The plan and preview are ignored
generated files; recreate them with Docker Desktop running:

```powershell
.\avlite.ps1 sketch-map
```

The model occupies approximately **30 m by 16.54 m** between barrier centreline
extents. The 30 m value is a dimension of the model, not its lap length. The
validated commissioning line is approximately **48.29 m** long and requests
**1.82–2.50 m/s**. These are planned values, not measured driving performance.

## Install and build the simulator

1. Install [Unity Hub](https://unity.com/download), sign in and complete its
   licence setup. In Hub, install **Unity Editor 2022.3.52f1** with Windows build
   support. That is the version recorded in the AutoDRIVE source project's
   [ProjectVersion.txt](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/ProjectSettings/ProjectVersion.txt).
2. Get the simulator source into a separate directory, outside this repository.
   This installation uses source revision `75b7df5216524a3c6b3150d698139190c6c44f8f`.
   A sparse checkout fetches the RoboRacer scene and its dependencies:

   ```powershell
   git clone --depth 1 --single-branch --branch AutoDRIVE-Simulator --filter=blob:none --no-checkout https://github.com/Tinker-Twins/AutoDRIVE.git C:\Users\Xxthe\AutoDRIVE-Unity
   git -C C:\Users\Xxthe\AutoDRIVE-Unity sparse-checkout init --no-cone
   python tools\unity\fetch_dependencies.py C:\Users\Xxthe\AutoDRIVE-Unity
   ```

   The dependency helper retains the source GUID metadata and reports any
   referenced archived assets requiring extraction. The selected RoboRacer scene
   does not need the unrelated archived vehicles and terrain. See also the
   project's [source installation instructions](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator#install-from-source).
3. Keep Unity Hub open with an active licence, close any Editor using that
   project, and run from this repository:

   ```powershell
   .\avlite.ps1 sketch-map
   .\avlite.ps1 sketch-build
   ```

   `-ProjectPath` and `-UnityPath` override the local defaults. `-SceneOnly`
   creates and validates the scene without compiling a player. `-BuildOnly`
   rebuilds an existing generated scene. Logs are under
   `log/unity-build`. The generated scene is
   `Assets/Scenes/RoboRacer - Sketch Track.unity` in the separate Unity project;
   the Windows player is `log/windows/sketch/AutoDRIVE Simulator.exe`.
   Windows Mono, Direct3D 11 and HDRP forward rendering are used, so an IL2CPP/
   Visual Studio installation is not required. The desktop build disables
   automatic XR startup and strips the unused deferred rendering path. Build
   automation uses `-nographics`; the simulator itself still renders normally.
   If a build reports `No valid Unity Editor license found` after restarting
   Windows, reopen Hub before retrying. This machine's Store-installed Hub
   provides the active licence through its licensing service.
4. Run `.\avlite.ps1 sketch-test -Visible` before selecting its AVLite profile.
   This opens the test window and captures the rendered track. Omitting `-Visible`
   runs the same physics checks in batch mode, whose screenshot is black.
   The optional
   `-sketch-smoke-test <absolute-report.json>` player argument checks wheel
   contact, finite LiDAR returns, two consecutive lap-trigger sequences and
   reset placement, then exits. The wrapper uses port 4568, separate from the
   AVLite bridge on 4567, and saves results under `log/unity-build/smoke-*`.
   It moves the vehicle between checkpoints to test the counters; this does
   **not** establish autonomous driving performance.

The builder reads the OBJ's literal Unity coordinates, avoiding importer axis
conversion. It preserves the vehicle, sensors, communications and reset machinery,
disables the stock environment, and creates static non-convex mesh colliders at
identity transform with the road at Y = 0. Road and barriers retain the source
floor's physics material. Both barriers use the name expected by the collision
counter and layer 0 for the source LiDAR raycasts.

The source vehicle's LiDAR is **0.156 m** above ground, slightly above the ZIP's
0.1524 m tubes. The scene stretches the barriers vertically by 2 to a height of
**0.3048 m** so the existing scanner sees them. Their horizontal outline, road
width, original asset files and AVLite map are unchanged. The collider scales
with the visible barrier; the vehicle and sensor mounting are retained.

The new rear-axle spawn comes from the validated route, with the vehicle root
offset preserved. Sixteen ordered checkpoint gates and a finish gate are placed
across the physical road. Their saved respawn transforms describe the vehicle
root, as required by AutoDRIVE's lap timer. The builder checks ground beneath the
route every 0.25 m, intersects both walls for each gate, and verifies that the
LiDAR head is within the adapted 0.3048 m barrier height. Its report is
`sketch-scene-validation.json` in the Unity project.

AutoDRIVE's [custom racetrack instructions](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Racetracks#custom-racetracks)
also describe importing geometry and adding colliders. Adding a Docker file or
changing AVLite's JSON does not perform these Unity scene changes.

## Activate only after the matching scene is built

Stop the practice simulator and controllers, then back up the current settings:

```powershell
.\avlite.ps1 stop
$backup = Join-Path 'log' ('track-backup-' + (Get-Date -Format yyyyMMdd-HHmmss))
New-Item -ItemType Directory -Path $backup | Out-Null
Copy-Item config\avlite.yaml $backup
Copy-Item config\driving.yaml $backup
if (Test-Path config\windows-simulator.json) { Copy-Item config\windows-simulator.json $backup }
Copy-Item config\avlite.sketch.yaml config\avlite.yaml
```

Keep the backup until the track switch is complete. Set the shared
`config/driving.yaml` values to:

```yaml
speed_mps: 2.5
max_throttle: 0.2
```

Remember the selected executable, then use the usual launcher:

```powershell
@{ track = 'sketch'; executable = 'log/windows/sketch/AutoDRIVE Simulator.exe' } |
    ConvertTo-Json | Set-Content config\windows-simulator.json
.\avlite.ps1 start
```

Select Connection and Autonomous once the car has been reset on the new track.
The optional `-SimulatorPath` argument overrides the saved selection. Without
either a saved selection or an explicit path, the launcher opens the practice
build. Changing the executable selection does not change AVLite's map; always
select the corresponding profile as above.
The usual recording command works after the new scene's lap/collision/reset
telemetry has been verified:

```powershell
.\avlite.ps1 laps -Laps 3 -Label sketch-commissioning
```

The new profile sets acceleration to 1 m/s², provisional braking to 1.5 m/s²,
and lateral acceleration to 3 m/s². `braking_calibrated: false` retains the
2.5 m/s commissioning cap. Measure the new scene's response before raising
speeds; the practice scene's tyre/road response is not automatically transferable.
The shared speed and throttle values continue to come from `driving.yaml`.

To roll back, close the custom build and restore the saved profiles:

```powershell
.\avlite.ps1 stop -SimulatorPath '.\log\windows\sketch\AutoDRIVE Simulator.exe'
# Set $backup to the actual backup directory printed/saved during activation.
$backup = (Get-Content log\unity-build\practice-backup.txt -Raw).Trim()
Copy-Item (Join-Path $backup 'avlite.yaml') config\avlite.yaml
Copy-Item (Join-Path $backup 'driving.yaml') config\driving.yaml
if (Test-Path (Join-Path $backup 'windows-simulator.json')) {
    Copy-Item (Join-Path $backup 'windows-simulator.json') config\windows-simulator.json
} else {
    Remove-Item -LiteralPath config\windows-simulator.json
}
.\avlite.ps1 start
```

## Coordinate conversion and planner adaptation

The supplied metadata uses Unity `(X,Z)` with Y up. AutoDRIVE's
[`GPS.cs`](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Scripts/GPS.cs)
reports world `(x,y,z) = (Unity Z, -Unity X, Unity Y)`. The converter therefore
uses `(z,-x)` for the AVLite map. Its counterclockwise world route places the
inner island on the left. Keep the vehicle's parent frame unrotated as well:
the upstream IMU reports a local rotation.

The road boundaries exclude the full **0.0762 m barrier radius** and an extra
1 cm sampling allowance. Boundary loops are aligned and sampled at at most
0.1 m intervals. The exported corridor is checked against the original physical
road, not merely against its sampled widths.

The default boundary-midpoint initialization makes the pinned GlobalRacePlanner
fold its route near this track's concave inner bend. A new optional `ReferencePath`
provides an initial route 1.2 m outside the inner road boundary, smoothed over
0.3 m. `planning.optimization_step_limit_m: 0.25` limits each optimization step
while retaining the original boundary constraints and velocity solver. These
options do not apply to the practice profile. They produce a feasible initial
commissioning line; they do not claim the fastest possible line on this track.

Validation rejects reversed, incomplete or crossing reference routes and plans.
It checks steering, lateral acceleration and acceleration/braking across the
lap boundary, then sweeps the **0.324 m axle segment with a 0.29 m radius** every
0.025 m against both the exported corridor and the original barrier geometry.
The native checks confirm wheel contact, finite LiDAR returns, consecutive
lap-counter sequences and reset placement. Barrier collision/respawn behavior
under motion, response calibration and repeated autonomous clean laps remain
commissioning work.

Verification on 22 September: **63 tests passed** across the sketch-track,
race-planning and configuration suites. Python lint and PowerShell parser
checks passed. The practice profile still validates at 1.772–5.698 m/s over
27.78 m. No driving commands were sent during this track preparation.
