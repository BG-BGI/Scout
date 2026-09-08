"""BIM alignment math, occupancy-grid rasterisation, and overlay rendering.

All coordinate transforms implement the SE2+scale map→sheet model stored in
site.json's per-map `bim.alignment` block:
    sheet = scale * R(theta) * map + t
where map is in robot map-frame metres and sheet is in OpenSpace sheet units
(typically image pixels).  The inverse (sheet→map) is also provided for
projecting capture positions back into the map frame.
"""

from __future__ import annotations

import io
import json
import math
import os

import numpy as np
from PIL import Image, ImageDraw


def solve_alignment(
    p1_map: tuple[float, float],
    p1_sheet: tuple[float, float],
    p2_map: tuple[float, float],
    p2_sheet: tuple[float, float],
) -> dict:
    """Solve SE2+scale from two landmark correspondences.

    Returns {"tx", "ty", "theta", "scale"} where scale = sheet_units/map_metre.
    Raises ValueError if the two map points are closer than 1 mm (degenerate).
    """
    dm = (p2_map[0] - p1_map[0], p2_map[1] - p1_map[1])
    ds = (p2_sheet[0] - p1_sheet[0], p2_sheet[1] - p1_sheet[1])
    d_map = math.hypot(*dm)
    if d_map < 1e-3:
        raise ValueError("map landmarks are too close together (< 1 mm)")
    scale = math.hypot(*ds) / d_map
    theta = math.atan2(ds[1], ds[0]) - math.atan2(dm[1], dm[0])
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    rot_x = scale * (p1_map[0] * cos_t - p1_map[1] * sin_t)
    rot_y = scale * (p1_map[0] * sin_t + p1_map[1] * cos_t)
    return {
        "tx": p1_sheet[0] - rot_x,
        "ty": p1_sheet[1] - rot_y,
        "theta": theta,
        "scale": scale,
    }


def map_to_sheet(x_m: float, y_m: float, alignment: dict) -> tuple[float, float]:
    """Map frame (metres) → sheet frame (sheet units)."""
    s, theta = alignment["scale"], alignment["theta"]
    tx, ty = alignment["tx"], alignment["ty"]
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (
        s * (x_m * cos_t - y_m * sin_t) + tx,
        s * (x_m * sin_t + y_m * cos_t) + ty,
    )


def sheet_to_map(sx: float, sy: float, alignment: dict) -> tuple[float, float]:
    """Sheet frame (sheet units) → map frame (metres)."""
    s, theta = alignment["scale"], alignment["theta"]
    tx, ty = alignment["tx"], alignment["ty"]
    dx, dy = (sx - tx) / s, (sy - ty) / s
    cos_t, sin_t = math.cos(-theta), math.sin(-theta)
    return dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t


def load_pgm(yaml_path: str) -> tuple[np.ndarray, tuple[float, float], float]:
    """Read a nav2 occupancy map.  Returns (array, (origin_x, origin_y), resolution).

    Array convention: row 0 = map-south (low y); matches nav2 at-launch orientation.
    Values: 0=occupied, 205=unknown, 254=free  (nav2 negate:0 convention).
    """
    import yaml as _yaml  # pyyaml is already in the scout-skills image

    with open(yaml_path) as f:
        meta = _yaml.safe_load(f)
    pgm_path = meta["image"]
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    with open(pgm_path, "rb") as f:
        magic = f.readline().strip()
        assert magic == b"P5", f"expected binary PGM (P5), got {magic!r}"
        dims_line = b""
        while True:
            line = f.readline()
            if not line.startswith(b"#"):
                dims_line = line
                break
        w, h = map(int, dims_line.split())
        f.readline()  # max-value line (255)
        data = f.read()

    arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
    arr = np.flipud(arr)  # row 0 = map-south (y↑ convention)
    origin = meta["origin"]
    return arr, (float(origin[0]), float(origin[1])), float(meta["resolution"])


def _wall_mask(arr: np.ndarray) -> np.ndarray:
    """Bool mask of occupied wall cells (nav2 negate:0 convention)."""
    return arr < 50


def rasterize_rooms(
    room_polys_map: list[list[tuple[float, float]]],
    origin: tuple[float, float],
    resolution: float,
    shape: tuple[int, int],
) -> np.ndarray:
    """Rasterise ACC room boundary polygons into a bool array sized to the SLAM pgm.

    room_polys_map: list of polygons, each a list of (map_x, map_y) vertex pairs.
    Returns True where a room boundary wall cell exists.
    """
    ox, oy = origin
    h, w = shape
    img = Image.new("L", (w, h), 0)
    draw = ImageDraw.Draw(img)
    for poly in room_polys_map:
        pixels = [
            (int((mx - ox) / resolution), h - 1 - int((my - oy) / resolution))
            for mx, my in poly
        ]
        if len(pixels) >= 2:
            draw.line(pixels + [pixels[0]], fill=255, width=2)
    return np.array(img) > 0


def compute_deviations(
    slam_arr: np.ndarray,
    bim_arr: np.ndarray,
    origin: tuple[float, float],
    resolution: float,
    threshold_m: float = 0.15,
) -> list[dict]:
    """Return BIM wall cells whose nearest SLAM wall exceeds threshold_m.

    Each entry: {map_x, map_y, dist_m}.  Uses chunked vectorised numpy to
    avoid O(N·M) memory; practical limit ~10 000 BIM wall cells.
    """
    slam_wall = _wall_mask(slam_arr)
    bim_pts = np.argwhere(bim_arr)
    slam_pts = np.argwhere(slam_wall)
    if len(bim_pts) == 0 or len(slam_pts) == 0:
        return []

    ox, oy = origin
    h = slam_arr.shape[0]
    chunk = 400
    result = []
    for i in range(0, len(bim_pts), chunk):
        chunk_pts = bim_pts[i : i + chunk].astype(np.float32)
        slam_f = slam_pts.astype(np.float32)
        diffs = chunk_pts[:, None, :] - slam_f[None, :, :]
        min_dist_cells = np.sqrt((diffs**2).sum(axis=2)).min(axis=1)
        min_dist_m = min_dist_cells * resolution
        for (row, col), dist_m in zip(bim_pts[i : i + chunk], min_dist_m):
            if dist_m > threshold_m:
                map_x = ox + int(col) * resolution
                map_y = oy + int(row) * resolution
                result.append({"map_x": map_x, "map_y": map_y, "dist_m": float(dist_m)})
    return result


def overlay(
    slam_yaml_path: str,
    sheet_png_bytes: bytes,
    alignment: dict,
) -> bytes:
    """Return PNG: SLAM occupancy grid composited over the OpenSpace sheet image.

    The SLAM grid is rendered in the sheet's pixel space using the alignment
    transform, so both layers are geographically registered.  Output dimensions
    match the SLAM pgm.
    """
    arr, origin, resolution = load_pgm(slam_yaml_path)
    h, w = arr.shape  # row 0 = map-south

    sheet_img = Image.open(io.BytesIO(sheet_png_bytes)).convert("RGBA")
    sw, sh = sheet_img.size
    sheet_arr = np.array(sheet_img)  # (sh, sw, 4)

    col_idx = np.arange(w, dtype=np.float32)
    row_idx = np.arange(h, dtype=np.float32)
    col_grid, row_grid = np.meshgrid(col_idx, row_idx)

    # Map-frame coords for each SLAM pixel (row 0 = map-south = low y)
    mx = origin[0] + col_grid * resolution
    my = origin[1] + row_grid * resolution

    # Sheet coords via alignment transform
    s, theta = alignment["scale"], alignment["theta"]
    tx, ty = alignment["tx"], alignment["ty"]
    cos_t, sin_t = float(np.cos(theta)), float(np.sin(theta))
    sx = s * (mx * cos_t - my * sin_t) + tx
    sy = s * (mx * sin_t + my * cos_t) + ty

    # Sheet image uses image convention (row 0 = image-top, y↓).
    # If sheet y-axis is assumed to match map y-axis (y↑), flip:
    px = np.clip(sx.astype(np.int32), 0, sw - 1)
    py = np.clip((sh - 1 - sy).astype(np.int32), 0, sh - 1)
    sheet_sampled = sheet_arr[py, px]  # (h, w, 4)

    # SLAM layer RGBA: walls dark-opaque, free semi-transparent, unknown clear
    slam_rgba = np.zeros((h, w, 4), dtype=np.uint8)
    arr_flip = np.flipud(arr)  # flip so row 0 = image-top for the RGBA render
    free_mask = arr_flip > 200
    occ_mask = arr_flip < 50
    slam_rgba[free_mask] = [210, 230, 255, 120]
    slam_rgba[occ_mask] = [30, 30, 90, 230]

    base = Image.fromarray(sheet_sampled, "RGBA")
    top = Image.fromarray(slam_rgba, "RGBA")
    out = Image.alpha_composite(base, top)
    buf = io.BytesIO()
    out.convert("RGB").save(buf, format="PNG")
    return buf.getvalue()
