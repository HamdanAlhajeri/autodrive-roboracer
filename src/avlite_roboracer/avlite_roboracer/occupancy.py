"""Occupancy grids saved by SLAM Toolbox / map_saver, and queries on them (no ROS).

Cells use the nav_msgs/OccupancyGrid convention: -1 unknown, 0 free, 100 occupied; row 0
is at the origin's y, so images (top row first) are flipped on load and save.
"""

import math
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


def read_pgm(path):
    """Read a binary (P5) or ASCII (P2) 8-bit PGM image into a uint8 array."""
    data = Path(path).read_bytes()
    tokens, index = [], 0
    while len(tokens) < 4:
        while data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b"#":
            while data[index:index + 1] not in (b"\n", b""):
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    magic, width, height, maximum = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maximum > 255 or magic not in (b"P5", b"P2"):
        raise ValueError(f"{path}: only 8-bit P5/P2 PGM images are supported")
    if magic == b"P5":
        pixels = np.frombuffer(data[index + 1:index + 1 + width * height], dtype=np.uint8)
    else:
        pixels = np.array(data[index:].split(), dtype=np.uint8)[:width * height]
    if pixels.size != width * height:
        raise ValueError(f"{path}: truncated image")
    return pixels.reshape(height, width)


def write_pgm(path, pixels):
    height, width = pixels.shape
    header = f"P5\n{width} {height}\n255\n".encode()
    Path(path).write_bytes(header + pixels.astype(np.uint8).tobytes())


class OccupancyGrid:
    def __init__(self, cells, resolution, origin, frame_id="map"):
        """cells is a rows-by-columns int array; origin is the (x, y) of cell (0, 0)'s corner."""
        cells = np.asarray(cells, dtype=np.int16)
        if cells.ndim != 2 or cells.size == 0:
            raise ValueError("occupancy grid must be a non-empty 2-D array")
        if not set(np.unique(cells)).issubset({UNKNOWN, FREE, OCCUPIED}):
            cells = np.where(cells < 0, UNKNOWN, np.where(cells >= 65, OCCUPIED,
                                                          np.where(cells <= 25, FREE, UNKNOWN)))
        if not (math.isfinite(resolution) and resolution > 0):
            raise ValueError("resolution must be positive")
        self.cells = cells.astype(np.int8)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.frame_id = frame_id
        self._distance = None

    @classmethod
    def from_message_data(cls, width, height, resolution, origin, data, frame_id="map"):
        """Build from OccupancyGrid message fields (row-major data starting at the origin)."""
        return cls(np.asarray(data, dtype=np.int16).reshape(height, width), resolution, origin,
                   frame_id)

    @classmethod
    def load(cls, yaml_path, frame_id="map"):
        """Load a map_saver YAML description and its image, applying its thresholds."""
        yaml_path = Path(yaml_path)
        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        origin = meta.get("origin", [0, 0, 0])
        if len(origin) > 2 and abs(origin[2]) > 1e-9:
            raise ValueError("rotated map origins are not supported")
        pixels = read_pgm(yaml_path.parent / meta["image"]).astype(float)
        occupancy = pixels / 255.0 if meta.get("negate", 0) else (255.0 - pixels) / 255.0
        cells = np.full(pixels.shape, UNKNOWN, dtype=np.int16)
        cells[occupancy > meta.get("occupied_thresh", 0.65)] = OCCUPIED
        cells[occupancy < meta.get("free_thresh", 0.25)] = FREE
        if meta.get("mode", "trinary") == "trinary":
            # map_saver writes unknown as 205, which a 0.25 free threshold would read as
            # free. Unobserved space must never become drivable track.
            cells[pixels == 205] = UNKNOWN
        return cls(cells[::-1], meta["resolution"], origin[:2], frame_id)

    def save(self, yaml_path):
        """Write a trinary map_saver-compatible YAML and PGM pair."""
        yaml_path = Path(yaml_path)
        image = yaml_path.with_suffix(".pgm")
        pixels = np.full(self.cells.shape, 205, dtype=np.uint8)
        pixels[self.cells == FREE] = 254
        pixels[self.cells == OCCUPIED] = 0
        write_pgm(image, pixels[::-1])
        yaml_path.write_text(yaml.safe_dump({
            "image": image.name, "mode": "trinary", "resolution": self.resolution,
            "origin": [self.origin[0], self.origin[1], 0.0], "negate": 0,
            "occupied_thresh": 0.65, "free_thresh": 0.196,
        }, sort_keys=False), encoding="utf-8")

    @property
    def shape(self):
        return self.cells.shape

    def to_cell(self, points):
        """Return integer (row, col) arrays for world points."""
        points = np.atleast_2d(np.asarray(points, dtype=float))
        col = np.floor((points[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((points[:, 1] - self.origin[1]) / self.resolution).astype(int)
        return row, col

    def inside(self, row, col):
        return (row >= 0) & (row < self.shape[0]) & (col >= 0) & (col < self.shape[1])

    def values(self, points):
        """Cell values at world points; points outside the grid read as UNKNOWN."""
        row, col = self.to_cell(points)
        ok = self.inside(row, col)
        result = np.full(len(row), UNKNOWN, dtype=np.int16)
        result[ok] = self.cells[row[ok], col[ok]]
        return result

    def distance_to_occupied(self, points):
        """Distance in metres from each point's cell to the nearest occupied cell.

        Points outside the grid return infinity. With no occupied cells every distance is
        infinite.
        """
        if self._distance is None:
            occupied = self.cells == OCCUPIED
            self._distance = (np.full(self.shape, np.inf) if not occupied.any() else
                              distance_transform_edt(~occupied) * self.resolution)
        row, col = self.to_cell(points)
        ok = self.inside(row, col)
        result = np.full(len(row), np.inf)
        result[ok] = self._distance[row[ok], col[ok]]
        return result

    def occupied_points(self):
        """World x/y centres of every occupied cell."""
        row, col = np.nonzero(self.cells == OCCUPIED)
        return np.column_stack(((col + 0.5) * self.resolution + self.origin[0],
                                (row + 0.5) * self.resolution + self.origin[1]))

    def cast(self, origin, direction, max_range):
        """March from origin along unit direction until something other than free space.

        Return (distance_m, outcome) where outcome is 'occupied', 'unknown', 'outside' or
        'max_range'. Unknown cells stop the ray: an unobserved region is never treated as a
        wall or as open track.
        """
        step = self.resolution / 2
        distances = np.arange(0.0, max_range + step, step)
        points = np.asarray(origin, dtype=float) + distances[:, None] * np.asarray(direction)
        row, col = self.to_cell(points)
        ok = self.inside(row, col)
        values = np.full(len(points), UNKNOWN, dtype=np.int16)
        values[ok] = self.cells[row[ok], col[ok]]
        stops = np.flatnonzero((values != FREE) | ~ok)
        if not len(stops):
            return float(max_range), "max_range"
        i = stops[0]
        if not ok[i]:
            return float(distances[i]), "outside"
        return float(distances[i]), "occupied" if values[i] == OCCUPIED else "unknown"
