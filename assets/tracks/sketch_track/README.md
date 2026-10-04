# Sketch Track for AutoDRIVE

A 3D circuit traced from your supplied hand-drawn sketch. The road is the area
between the outer loop and inner island. The small opening in the outer stroke
at the top was closed, the short pen overshoot was removed, and the outlines
were lightly smoothed. The varying widths and bends were preserved.

## Files

- `sketch_track.obj` — geometry with three separately named objects.
- `sketch_track.mtl` — asphalt and red barrier materials; keep beside the OBJ.
- `sketch_track_preview.png` — top and perspective views of the mesh.
- `track_metadata.json` — dimensions, boundary coordinates, and mesh checks.
- `track_boundaries.json` — traced reference contours used by the generator.
- `build_track.py` — rebuild or resize the track without retracing the image.

## Dimensions and coordinates

The source sketch contains no dimensions. This model assumes a **30 m** outer
boundary length and measures approximately **16.54 m** across. The tube barriers
extend another 76.2 mm outside the boundary on each side.

- Units: **metres** (OBJ does not enforce units; check the Unity import scale).
- Up axis: **Y**. The road surface is at **Y = 0**.
- Road slab: **100 mm** thick, extending down to Y = -0.10 m.
- Barriers: circular cross-section, **152.4 mm** diameter, 16 sides.
- The top of the sketch points toward **-Z**; its right side is **+X**.
- The track is centred about the origin in X and Z.
- Approximate minimum boundary-to-boundary distance: 2.69 m; subtracting
  the two barrier radii leaves approximately 2.54 m. This is a sampled
  geometric measurement, not a certified minimum driving width.

The three objects are `Track_Surface`, `Outer_Barrier`, and `Inner_Barrier`.
All are closed, consistently oriented triangle meshes. Total: 19,278 vertices,
38,556 triangles. Plain materials are included; no external textures are needed.

## Import into AutoDRIVE

1. Open the **AutoDRIVE-Simulator source project** in Unity. Start from a copy
   of an existing F1TENTH/RoboRacer scene to retain its vehicle and sensors.
2. Copy `sketch_track.obj` and `sketch_track.mtl` together into a new folder
   within that project's `Assets` directory.
3. Drag the imported model into the scene. Check its dimensions and Y-up
   orientation in Unity. The intended model size is roughly 16.7 × 0.2524 ×
   30.15 m including the barriers and road thickness.
4. Disable/remove the old track barriers and any overlapping road surfaces.
   Preserve the vehicle and the simulator's other required scene objects.
5. Add a **Mesh Collider** to each of the three imported mesh objects, using
   that object's mesh. Leave **Convex disabled**: convexifying these shapes
   would incorrectly fill the inner island or bridge the bends.
6. Keep these environmental objects stationary, without a Rigidbody. Use
   the existing AutoDRIVE scene's appropriate layers and physics materials;
   imported material colours do not configure tyre friction.
7. If your render pipeline does not import the MTL appearance correctly,
   assign compatible asphalt and barrier materials in the Mesh Renderers.
   If the importer combines the object hierarchy, use its model settings
   or a 3D editor to preserve/separate the three named objects.
8. Place the vehicle above the road, with its wheels resting on Y = 0 when
   settled. Confirm the visual scale, ground contact, collisions, and LiDAR
   barrier returns before connecting an autonomous controller.

This is a **track asset**, not a Unity scene or trained driving agent. Spawn
configuration, checkpoint triggers, finish-line detection, rewards, and episode
resets must be added in your AutoDRIVE training scene. Do not assume importing
the OBJ configures those features.

## Resize or regenerate

Requires Python 3, NumPy, and SciPy. Matplotlib is needed for `--preview` only.
Run from this extracted folder:

```bash
python -m pip install numpy scipy matplotlib
python build_track.py --length 30 --output rebuilt --preview
```

For a 20 m version, change `--length 30` to `--length 20`. Barrier diameter
and slab thickness remain at physical defaults unless set explicitly:

```bash
python build_track.py --length 20 --barrier-diameter 0.1524 --floor-thickness 0.1 --output track_20m
```

Regenerating is preferable to scaling the whole object if you want barrier
diameter and floor thickness to remain fixed.

## Validation and limitations

The generator checks finite vertices, valid indices, nondegenerate triangles,
closed manifold edges, consistent winding, positive signed volumes, simple
boundary loops, and an inner island contained by the outer loop. It also checks
that triangulation respects both boundaries and covers the road area exactly.

Geometry and the exported OBJ were checked locally and a preview was rendered.
**The model has not been imported or run in Unity/AutoDRIVE here.** Real-world
dimensions and driving difficulty cannot be recovered from the sketch alone.

## Reference documentation

- [AutoDRIVE custom racetracks](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Racetracks#custom-racetracks)
- [AutoDRIVE source setup](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator#install-from-source)
- [Unity model formats](https://docs.unity3d.com/2022.3/Documentation/Manual/3D-formats.html)
