# IROS 2026 track

A replica of the RoboRacer IROS 2026 race track, built from the organisers' published
map. Asset details, rule checks and limitations are in
[`assets/tracks/iros2026`](../assets/tracks/iros2026/README.md).

**Status, 18 November 2026:** the mesh, AVLite map and a 2.5 m/s commissioning plan
pass offline validation. The Unity scene has not been built and no lap has been driven.

| | |
| --- | --- |
| Size | 44.3 x 23.8 m including pipes |
| Road width | 1.66 m minimum, 2.20 m median |
| Pipes | 0.3048 m tall in Unity (0.1524 m tubes stretched 2x) |
| Commissioning line | about 199 m, planned 1.69 to 2.50 m/s |

## Prepared files

| File | Purpose |
| --- | --- |
| `assets/tracks/iros2026/*` | OBJ/MTL, metadata, preview and generator |
| `config/maps/iros2026.json` | AVLite boundaries and seed route in world metres |
| `config/maps/iros2026.validation.json` | Offline checks, planned speed range and rear-axle spawn pose |
| `config/avlite.iros2026.yaml` | Commissioning profile, copied to `avlite.yaml` when activating |
| `config/maps/iros2026.png`, `.plan.json` | Generated preview and plan (ignored by Git; recreate with `sketch-map`) |

## Build and check the simulator

Unity setup is the same as for the sketch track: follow steps 1 and 2 of
[sketch setup](sketch-track.md#install-and-build-the-simulator). Then:

```powershell
.\avlite.ps1 sketch-map -Track iros2026
.\avlite.ps1 sketch-build -Track iros2026
.\avlite.ps1 sketch-test -Track iros2026 -Visible
```

`sketch-build` writes the scene `Assets/Scenes/RoboRacer - IROS 2026 Track.unity` and the
player `log/windows/iros2026/AutoDRIVE Simulator.exe`. It places 16 checkpoint gates and
a finish gate along the plan, and checks the road under the route every 0.25 m. The
default `-Track` is still `sketch`, so existing commands behave as before.

## Activate

Stop the simulator and controllers and back up the current settings, as in
[sketch activation](sketch-track.md#activate-only-after-the-matching-scene-is-built).
Then select the IROS 2026 profile and build:

```powershell
Copy-Item config\avlite.iros2026.yaml config\avlite.yaml
@{ track = 'iros2026'; executable = 'log/windows/iros2026/AutoDRIVE Simulator.exe' } |
    ConvertTo-Json | Set-Content config\windows-simulator.json
.\avlite.ps1 start
```

Set `config/driving.yaml` to `speed_mps: 2.5` and `max_throttle: 0.2` for the first
laps. The profile keeps `braking_calibrated: false`, so the plan stays capped at
2.5 m/s until braking is measured on this scene. Record with:

```powershell
.\avlite.ps1 laps -Laps 3 -Label iros2026-commissioning
```

Roll back the same way as the sketch track.

## How the map was made

`build_track.py` traces the free road on the official map, removes the 30 ft floor-grid
dots, and smooths the pixel steps. Pipe centres sit 0.07 m outside the road edge, the
middle of the drawn line.

The sketch converter (`avlite_autodrive.sketch_track`) needed three options for this
layout. They are set in the track metadata; the sketch track does not use them and its
output is unchanged.

- `map_boundary_pairing: monotone_nearest` pairs the inner and outer walls by shortest
  ordered matching. Equal-fraction pairing fails here because one wall is much longer
  than the other through hairpins and shared separators.
- `map_reference_seed: smoothed_midline` seeds the optimizer with the paired midline.
  A fixed offset from the island would cross the thin separators.
- `map_spacing_m: 0.05` samples the map every 5 cm so chords do not cut the rounded
  pipe corners.

Validation sweeps the 0.324 m axle with a 0.29 m envelope every 0.025 m against the
original pipe geometry, as for the sketch track.

## Limitations

- The published drawing is slightly larger than the written 43 x 23 m limit. The
  replica keeps the official scale.
- Floor friction, windows, lighting and gaps between pipes are not modelled.
- The race direction is not stated in the rules. The map drives counterclockwise.
- The 0.07 m pipe offset leaves the pipes up to 6 mm inside the drawn road edge.
