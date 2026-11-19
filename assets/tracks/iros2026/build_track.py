#!/usr/bin/env python3
"""Build a Y-up OBJ replica of the RoboRacer IROS 2026 track from its official map.

The official map is the 0.05 m/pixel occupancy image published for the race
(https://iros2026-race.roboracer.ai/race_resources.html). Its two drawn lines are the
pipe delimiters; the free area between them is the road. This script traces that road,
removes the 30 ft floor-grid reference dots drawn on the image, and writes a road slab
plus two tube barriers in the same layout as ../sketch_track, so the existing AVLite
converter and Unity builder can use it.

Requires Python 3, NumPy, SciPy, Shapely, scikit-image, Pillow and PyYAML. Matplotlib is
needed only for --preview.

Example:
    python build_track.py --map IROS2026.png --output . --preview
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.ndimage import gaussian_filter1d
from shapely.geometry import Polygon
from skimage import measure

# Reuse the mesh helpers already validated for the sketch track.
_SPEC = importlib.util.spec_from_file_location(
    "sketch_build_track", Path(__file__).resolve().parents[1] / "sketch_track" / "build_track.py")
mesh = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(mesh)

NAME = "iros2026_track"
SOURCE_URL = "https://iros2026-race.roboracer.ai/assets/maps/IROS2026.png"
SOURCE_SHA256 = {
    "IROS2026.png": "44ba81a754211f59fb3c0b6985e80736dffd7fd5bb8a4d2196eed97b3cf4c73b",
    "IROS2026.yaml": "483ff1d6d1bcb2a71aaf11f47608dae54131e2b5d5b0e29e419c9d8fc8803194",
}
# Competition rules for IROS 2026 (iros2026-race.roboracer.ai/competition_rules.html).
SPEC = {
    "event": "RoboRacer IROS 2026, David L. Lawrence Convention Center, Pittsburgh",
    "surface": "bare exhibition-hall floor, smooth concrete, not carpeted",
    "delimiters": "pipes: yellow/black stripes, orange/black stripes or plain black",
    "minimum_delimiter_height_m": 0.30,
    "minimum_width_m": 1.0,
    "target_width_m": 1.5,
    "maximum_track_extent_m": [43.0, 23.0],
}
GRID_DOT_RADIUS_PX = 7.5
LINE_HALF_WIDTH_PX = 1.6


def load_map(png, yaml_path):
    """Return the occupied-cell mask, resolution and origin of a ROS map_server map."""
    info = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8"))
    image = np.asarray(Image.open(png).convert("L"), dtype=float)
    occupancy = image / 255.0 if info.get("negate", 0) else (255.0 - image) / 255.0
    return occupancy > float(info["occupied_thresh"]), float(info["resolution"]), info["origin"][:2]


def _cluster(values, tolerance=4.0):
    """Group nearly equal 1-D coordinates and return each group's mean."""
    values = np.sort(values)
    groups = [[values[0]]]
    for value in values[1:]:
        if value - groups[-1][-1] <= tolerance:
            groups[-1].append(value)
        else:
            groups.append([value])
    return np.array([np.mean(g) for g in groups])


def remove_grid_dots(occupied):
    """Erase the regular floor-grid reference dots without breaking the drawn lines.

    Isolated dots locate the grid. At every grid point the dot is cleared; where a line
    entered the dot it is redrawn: a grazing line continues straight, a line passing
    through joins its entry and exit, and a junction joins its branches at the centre.
    """
    labels, count = ndimage.label(occupied, structure=np.ones((3, 3)))
    sizes = ndimage.sum(occupied, labels, range(1, count + 1))
    dots = [i + 1 for i, size in enumerate(sizes) if 60 <= size <= 300]
    if len(dots) < 4:
        raise ValueError("Could not find the floor-grid reference dots")
    centres = np.array(ndimage.center_of_mass(occupied, labels, dots))
    rows, cols = _cluster(centres[:, 0]), _cluster(centres[:, 1])
    rr, cc = np.mgrid[:occupied.shape[0], :occupied.shape[1]]
    clean = occupied.copy()
    redrawn = 0
    for r0 in rows:
        for c0 in cols:
            d2 = (rr - r0)**2 + (cc - c0)**2
            disk = d2 <= GRID_DOT_RADIUS_PX**2
            ring = (d2 > GRID_DOT_RADIUS_PX**2) & (d2 <= (GRID_DOT_RADIUS_PX + 2.5)**2) & occupied
            clean[disk] = False
            ends, n = ndimage.label(ring, structure=np.ones((3, 3)))
            if n == 1:
                points = np.column_stack(np.where(ends == 1)).astype(float)
                middle = points.mean(axis=0)
                direction = np.linalg.svd(points - middle)[2][0]
                segments = [(middle - 20 * direction, middle + 20 * direction)]
            elif n == 2:
                a, b = (np.array(ndimage.center_of_mass(ring, ends, k)) for k in (1, 2))
                segments = [(a, b)]
            else:
                segments = [(np.array(ndimage.center_of_mass(ring, ends, k)), np.array([r0, c0]))
                            for k in range(1, n + 1)]
            for a, b in segments:
                v = b - a
                t = np.clip(((rr - a[0]) * v[0] + (cc - a[1]) * v[1]) / (v @ v), 0, 1)
                near = np.hypot(rr - (a[0] + t * v[0]), cc - (a[1] + t * v[1])) <= LINE_HALF_WIDTH_PX
                clean |= disk & near
            redrawn += bool(n)
    return clean, dict(grid_rows_px=rows.round(1).tolist(), grid_cols_px=cols.round(1).tolist(),
                       dots_cleared=int(len(rows) * len(cols)), dots_touching_lines=redrawn)


def road_mask(occupied):
    """Return the drivable road: the largest enclosed free region, which must be one loop."""
    free, count = ndimage.label(~occupied)
    h, w = occupied.shape
    border = set(np.unique(np.r_[free[0], free[-1], free[:, 0], free[:, -1]]))
    sizes = ndimage.sum(~occupied, free, range(1, count + 1))
    candidates = [i + 1 for i in np.argsort(sizes)[::-1] if i + 1 not in border]
    road = free == candidates[0]
    holes, n_holes = ndimage.label(ndimage.binary_fill_holes(road) & ~road, structure=np.ones((3, 3)))
    if measure.euler_number(road, connectivity=2) != 0 or n_holes != 1:
        raise ValueError("The road is not a single closed loop around one island")
    return road, holes > 0


def contour_metres(mask, resolution, origin):
    """Trace a filled region's outline and convert pixel centres to map-frame metres."""
    contours = measure.find_contours(np.pad(mask, 1).astype(float), 0.5)
    rc = max(contours, key=len)[:-1] - 1
    h = mask.shape[0]
    return np.column_stack((origin[0] + (rc[:, 1] + 0.5) * resolution,
                            origin[1] + (h - rc[:, 0] - 0.5) * resolution))


def smooth_loop(points, spacing=0.02, sigma=0.075):
    """Resample a closed outline evenly and remove the pixel staircase."""
    closed = np.vstack([points, points[:1]])
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    t = np.arange(0, distance[-1], spacing)
    even = np.column_stack([np.interp(t, distance, closed[:, i]) for i in range(2)])
    return gaussian_filter1d(even, sigma / spacing, axis=0, mode="wrap")


def single_polygon(geometry, label):
    if geometry.geom_type != "Polygon" or geometry.is_empty or len(geometry.interiors):
        raise ValueError(f"{label} is not a single simple polygon")
    return Polygon(geometry.exterior)


def map_to_unity(points, centre):
    """Map-frame (x, y) to centred Unity (X, Z); inverse of world = (Unity Z, -Unity X)."""
    world = np.asarray(points) - centre
    return np.column_stack((-world[:, 1], world[:, 0]))


def width_profile(outer, inner, step=0.25):
    """Edge-to-edge road width sampled along both road edges."""
    from shapely.geometry import Point
    widths = []
    for edge, other in ((outer, inner), (inner, outer)):
        ring = edge.exterior
        for s in np.arange(0, ring.length, step):
            widths.append(other.exterior.distance(Point(ring.interpolate(s).coords[0])))
    return np.array(widths)


def write_obj(path, meshes, stripe_m):
    """Write named meshes; barriers get alternating stripe materials along their length."""
    with path.open("w") as out:
        out.write("# RoboRacer IROS 2026 track replica. Units: metres. Y up. Road top Y=0.\n")
        out.write("# Unity X = -(map y - centre y), Unity Z = map x - centre x. See track_metadata.json.\n")
        out.write(f"# Three closed meshes: road slab and two tube barriers.\nmtllib {path.stem}.mtl\n")
        voff = 0
        for m in meshes:
            v, f = m["vertices"], m["faces"]
            out.write(f"\no {m['name']}\n")
            for q in v:
                out.write("v %.7f %.7f %.7f\n" % tuple(q))
            if m["name"] == "Track_Surface":
                groups = [("Concrete", np.arange(len(f)))]
            else:
                ring = (f.min(axis=1) // m["sides"]).astype(int)
                stripe = (m["arclength"][ring] // stripe_m).astype(int) % 2
                groups = [(m["material"], np.where(stripe == 0)[0]), ("Pipe_Black", np.where(stripe == 1)[0])]
            for material, ids in groups:
                out.write(f"usemtl {material}\n")
                for face in f[ids]:
                    out.write("f %d %d %d\n" % tuple(face + voff + 1))
            voff += len(v)
    path.with_suffix(".mtl").write_text("""# Plain diffuse materials; no textures. Unity sets physics.
newmtl Concrete
Kd 0.62 0.62 0.60
Ks 0.10 0.10 0.10
Ns 20
illum 2

newmtl Pipe_Yellow
Kd 0.95 0.78 0.08
Ks 0.15 0.15 0.15
Ns 30
illum 2

newmtl Pipe_Orange
Kd 0.96 0.42 0.06
Ks 0.15 0.15 0.15
Ns 30
illum 2

newmtl Pipe_Black
Kd 0.04 0.04 0.04
Ks 0.15 0.15 0.15
Ns 30
illum 2
""")


def render(output, occupied, resolution, origin, edges, centrelines, metadata):
    """Save a top view of the replica beside the official map with the traced pipes overlaid."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as Patch
    h, w = occupied.shape
    extent = [origin[0], origin[0] + w * resolution, origin[1], origin[1] + h * resolution]
    fig, axes = plt.subplots(2, 1, figsize=(12, 13.5), facecolor="#f4f3ef")
    fig.suptitle("RoboRacer IROS 2026 track replica", x=0.06, ha="left", fontsize=20, weight="bold",
                 color="#1e2936")
    ax = axes[0]
    ax.imshow(np.where(occupied, 0.0, 1.0), cmap="gray", extent=extent, vmin=0, vmax=1, alpha=0.55)
    for line, colour in zip(centrelines, ("#e0a000", "#e8590c")):
        loop = np.vstack([line, line[:1]])
        ax.plot(loop[:, 0], loop[:, 1], color=colour, lw=1.4)
    ax.set_title("Official map (grey) with traced pipe centrelines; grid dots removed", loc="left",
                 color="#536170", fontsize=11)
    ax.set_xlabel("official map x (m)")
    ax.set_ylabel("official map y (m)")
    ax.set_aspect("equal")
    ax = axes[1]
    ax.set_facecolor("#f4f3ef")
    outer, inner = edges
    ax.add_patch(Patch(outer, closed=True, facecolor="#9b9b97", edgecolor="none"))
    ax.add_patch(Patch(inner, closed=True, facecolor="#f4f3ef", edgecolor="none"))
    radius = metadata["barrier_diameter_m"] / 2
    for line, colour in zip(centrelines, ("#f2c40f", "#f46a0f")):
        loop = np.vstack([line, line[:1]])
        s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(loop, axis=0), axis=1))]
        for k in range(len(loop) - 1):
            stripe = colour if int(s[k] // metadata["stripe_length_m"]) % 2 == 0 else "#111111"
            ax.plot(loop[k:k + 2, 0], loop[k:k + 2, 1], color=stripe, lw=2.2, solid_capstyle="butt")
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("official map x (m)")
    ax.set_ylabel("official map y (m)")
    ax.set_title(
        f"Replica: {metadata['extent_m'][0]:.1f} x {metadata['extent_m'][1]:.1f} m, "
        f"width {metadata['road_width_m']['min']:.2f}-{metadata['road_width_m']['max']:.2f} m "
        f"(median {metadata['road_width_m']['median']:.2f} m), "
        f"pipes {metadata['barrier_height_in_unity_m']:.3f} m tall (radius {radius:.4f} m)",
        loc="left", color="#536170", fontsize=11)
    fig.text(0.06, 0.015, "Generated from the published map. Not yet imported or driven in Unity/AutoDRIVE.",
             fontsize=9, color="#6c7680")
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    fig.savefig(output / f"{NAME}_preview.png", dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", type=Path, required=True, help="Official IROS2026.png (YAML beside it).")
    parser.add_argument("--yaml", type=Path, help="Map YAML; defaults to the PNG name with .yaml.")
    parser.add_argument("--barrier-diameter", type=float, default=0.1524,
                        help="Tube diameter in metres; Unity stretches it 2x vertically (default 0.1524).")
    parser.add_argument("--centreline-offset", type=float, default=0.07,
                        help="Pipe centre distance outside the traced road edge (default 0.07 m).")
    parser.add_argument("--floor-thickness", type=float, default=0.10)
    parser.add_argument("--spacing", type=float, default=0.10, help="Boundary sampling in metres.")
    parser.add_argument("--sides", type=int, default=16)
    parser.add_argument("--stripe-length", type=float, default=0.5)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--allow-different-source", action="store_true",
                        help="Accept a map whose SHA-256 differs from the published file.")
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    yaml_path = args.yaml or args.map.with_suffix(".yaml")
    digests = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.map, yaml_path)}
    if digests != SOURCE_SHA256 and not args.allow_different_source:
        parser.error(f"Map files differ from the published IROS 2026 map: {digests}")
    if min(args.barrier_diameter, args.floor_thickness, args.spacing, args.centreline_offset) <= 0:
        parser.error("Dimensions must be positive.")

    occupied, resolution, origin = load_map(args.map, yaml_path)
    occupied, dots = remove_grid_dots(occupied)
    road, island = road_mask(occupied)
    outer_edge = Polygon(smooth_loop(contour_metres(road | island, resolution, origin)))
    inner_edge = Polygon(smooth_loop(contour_metres(island, resolution, origin)))
    if not (outer_edge.is_valid and inner_edge.is_valid and outer_edge.contains(inner_edge)):
        raise ValueError("Traced road edges are not simple and nested")
    outer_line = single_polygon(outer_edge.buffer(args.centreline_offset), "Outer pipe centreline")
    inner_line = single_polygon(inner_edge.buffer(-args.centreline_offset), "Inner pipe centreline")
    # Shrinking the pipe centrelines back must reproduce the traced road; a thin separator
    # that merged would otherwise let the map join two neighbouring corridors.
    road_polygon = outer_edge.difference(inner_edge)
    rebuilt = outer_line.buffer(-args.centreline_offset).difference(inner_line.buffer(args.centreline_offset))
    if rebuilt.symmetric_difference(road_polygon).area > 0.01:
        raise ValueError("Pipe centreline offset changes the road; reduce --centreline-offset")

    lo, hi = np.array(outer_line.bounds[:2]), np.array(outer_line.bounds[2:])
    centre = (lo + hi) / 2
    as_xz = {k: mesh.resample(map_to_unity(np.asarray(g.exterior.coords)[:-1], centre), args.spacing)
             for k, g in (("outer_edge", outer_edge), ("inner_edge", inner_edge),
                          ("outer_line", outer_line), ("inner_line", inner_line))}
    # The map has ruler-straight sections; exactly collinear edge samples would give the
    # Delaunay road slab zero-area triangles. A fixed micrometre jitter avoids that.
    jitter = np.random.default_rng(2026)
    for key in ("outer_edge", "inner_edge"):
        as_xz[key] = as_xz[key] + jitter.normal(scale=1e-6, size=as_xz[key].shape)
    for loop in as_xz.values():
        mesh.check_simple(loop)
    if not mesh.contains(as_xz["inner_line"], as_xz["outer_line"]).all():
        raise ValueError("Inner pipe leaves the outer pipe loop")
    points, faces, _, road_area = mesh.triangulate_ring(as_xz["outer_edge"], as_xz["inner_edge"])
    floor = mesh.solid_floor(points, faces, args.floor_thickness)
    floor["material"] = "Concrete"
    radius = args.barrier_diameter / 2
    tubes = []
    for key, name, material in (("outer_line", "Outer_Barrier", "Pipe_Yellow"),
                                ("inner_line", "Inner_Barrier", "Pipe_Orange")):
        tube = mesh.tube(as_xz[key], radius, args.sides, name)
        loop = as_xz[key]
        tube.update(material=material, sides=args.sides, arclength=np.r_[0, np.cumsum(
            np.linalg.norm(np.diff(loop, axis=0), axis=1))])
        tubes.append(tube)
    meshes = [floor] + tubes
    checks = {m["name"]: mesh.validate(m) for m in meshes}

    widths = width_profile(outer_edge, inner_edge)
    clear = widths - 2 * max(0.0, radius - args.centreline_offset)
    extent = np.ptp(np.asarray(outer_line.exterior.coords), axis=0) + args.barrier_diameter
    metadata = {
        "name": "RoboRacer IROS 2026", "source_id": "iros2026_official_map",
        "units": "metres", "up_axis": "Y", "road_top_y": 0,
        "barrier_diameter_m": args.barrier_diameter,
        "barrier_vertical_scale_in_unity": 2,
        "barrier_height_in_unity_m": round(2 * args.barrier_diameter, 4),
        "pipe_centreline_offset_from_road_edge_m": args.centreline_offset,
        "floor_thickness_m": args.floor_thickness, "stripe_length_m": args.stripe_length,
        "extent_m": [round(float(e), 3) for e in extent],
        "road_area_m2": round(float(road_polygon.area), 2),
        "road_width_m": {"min": round(float(clear.min()), 3), "median": round(float(np.median(clear)), 3),
                         "max": round(float(clear.max()), 3),
                         "share_below_1_5_m": round(float(np.mean(clear < 1.5)), 4)},
        "centreline_length_m": {"outer": round(outer_line.length, 2), "inner": round(inner_line.length, 2)},
        "frames": {
            "official_map_frame_offset_m": [round(float(c), 4) for c in centre],
            "note": "official map (x, y) = AVLite world (x, y) + offset; "
                    "Unity (X, Z) = (-world y, world x).",
        },
        "source": {"url": SOURCE_URL, "resolution_m": resolution, "origin_m": list(origin),
                   "sha256": digests, "grid_dots": dots},
        "specification": SPEC,
        "spec_checks": {
            "minimum_width_ge_1_m": bool(clear.min() >= SPEC["minimum_width_m"]),
            "most_sections_ge_1_5_m": bool(np.mean(clear >= SPEC["target_width_m"]) >= 0.5),
            "pipe_height_ge_0_30_m": bool(2 * args.barrier_diameter >= SPEC["minimum_delimiter_height_m"]),
            "extent_within_43_x_23_m": bool(extent[0] <= 43 and extent[1] <= 23),
        },
        # AVLite conversion options for this snaking layout (see avlite_autodrive.sketch_track).
        "map_boundary_pairing": "monotone_nearest", "map_reference_seed": "smoothed_midline",
        # Pipe corners have ~9 cm inset radii; 5 cm samples keep map chords off the pipes.
        "map_spacing_m": 0.05,
        "approximate": True, "unity_runtime_tested": False, "objects": checks,
        "track_boundaries": {"outer_xz": as_xz["outer_line"].round(6).tolist(),
                             "inner_xz": as_xz["inner_line"].round(6).tolist()},
    }
    args.output.mkdir(parents=True, exist_ok=True)
    write_obj(args.output / f"{NAME}.obj", meshes, args.stripe_length)
    (args.output / "track_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    report = {"file": f"{NAME}.obj", "objects": checks, "spec_checks": metadata["spec_checks"],
              "unity_runtime_tested": False}
    (args.output / "validation_report.json").write_text(json.dumps(report, indent=2) + "\n")
    if args.preview:
        edges = [np.asarray(g.exterior.coords) for g in (outer_edge, inner_edge)]
        lines = [np.asarray(g.exterior.coords)[:-1] for g in (outer_line, inner_line)]
        render(args.output, occupied, resolution, origin, edges, lines, metadata)
    print(json.dumps({k: v for k, v in metadata.items() if k != "track_boundaries"}, indent=2))


if __name__ == "__main__":
    main()
