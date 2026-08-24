"""Negative-obstacle (cliff / down-stair) detection math — pure numpy, no ROS.

The D455 sits level at ~0.205 m, so base_link z=0 IS the floor plane and any
return meaningfully below it is ground past a drop (stair treads, a lower
landing). The camera's lowest ray meets the floor only ~0.51 m ahead of
base_link, and once the robot is closer than that the ledge leaves the FOV
entirely — which is why detections are remembered in the odom frame
(CliffMemory) instead of trusted to live sensing: the latch is what protects
the final approach and the look-away-then-return case.

Two deliberate scope limits (ADR-0024):
- Only below-floor RETURNS are used. Missing-returns ("the floor should be
  here and is not") is not inferable at x4 decimation with the spatial filter
  on — holes are routine on dark/glancing floors and would false-stop
  constantly. A true void with zero returns (glass-edged mezzanine) is
  therefore NOT detected.
- The drop threshold scales with range (drop_base + drop_slope*r) to absorb
  the D455's ~2% depth error and small chassis pitch without a bigger fixed
  margin killing sensitivity up close.
"""

import numpy as np


def parse_xyz(data, point_step, x_offset=0, y_offset=4, z_offset=8):
    """Nx3 float32 XYZ out of a PointCloud2 data buffer; non-finite rows
    dropped. Offsets come from the message's fields (xyz32 layouts only)."""
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    n = len(buf) // point_step
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    raw = buf[:n * point_step].reshape(n, point_step)
    cols = [raw[:, off:off + 4].copy().view(np.float32).ravel()
            for off in (x_offset, y_offset, z_offset)]
    xyz = np.column_stack(cols)
    return xyz[np.isfinite(xyz).all(axis=1)]


def find_cliff_cells(xyz_cam, rot, trans, *, min_range, max_range, drop_base,
                     drop_slope, max_drop, cell_size, min_points):
    """Cliff-lip cell centers (Nx2 float32, base frame) from a camera-frame
    cloud and the camera->base transform (rot 3x3, trans 3, node_util
    lookup_matrix shapes).

    Pipeline: transform to base -> gate to below-floor returns within
    [min_range, max_range] -> project each point's ray back up to the z=0
    floor plane (the mark must land on the LIP of the drop, walkable ground,
    not on the tread below it) -> bin into cell_size cells -> keep cells with
    >= min_points votes."""
    if len(xyz_cam) == 0:
        return np.empty((0, 2), dtype=np.float32)
    p = xyz_cam @ rot.T.astype(np.float32) + trans.astype(np.float32)
    r = np.hypot(p[:, 0], p[:, 1])
    drop = drop_base + drop_slope * r
    sel = ((r >= min_range) & (r <= max_range)
           & (p[:, 2] < -drop) & (p[:, 2] > -max_drop))
    p = p[sel]
    if len(p) == 0:
        return np.empty((0, 2), dtype=np.float32)
    # Ray from the camera origin c to point p crosses the floor plane z=0 at
    # parameter s = c_z / (c_z - p_z) (p_z < 0 < c_z, so 0 < s < 1).
    c = trans.astype(np.float32)
    s = c[2] / (c[2] - p[:, 2])
    edge = c[:2] + s[:, None] * (p[:, :2] - c[:2])
    idx = np.floor(edge / cell_size).astype(np.int64)
    uniq, counts = np.unique(idx, axis=0, return_counts=True)
    keep = uniq[counts >= min_points]
    return ((keep.astype(np.float32) + 0.5) * cell_size)


class CliffMemory:
    """Remembered cliff cells, keyed on a fixed odom-frame grid.

    Odom (not map) so the memory needs no localization and never jumps with a
    map->odom correction; drift over a single approach is cm-scale, and the
    TTL bounds how long drift can accumulate under a mark."""

    def __init__(self, cell_size, ttl_s, max_cells):
        self._cell = float(cell_size)
        self._ttl = float(ttl_s)
        self._max = int(max_cells)
        self._cells = {}          # (i, j) -> last_seen (monotonic seconds)

    def add(self, xy_odom, now):
        """Record cell hits (Nx2 odom XY) at time `now`, then prune expired
        cells and cap the store by dropping the oldest."""
        for x, y in np.asarray(xy_odom, dtype=np.float64).reshape(-1, 2):
            self._cells[(int(np.floor(x / self._cell)),
                         int(np.floor(y / self._cell)))] = now
        expired = [k for k, t in self._cells.items() if now - t > self._ttl]
        for k in expired:
            del self._cells[k]
        if len(self._cells) > self._max:
            for k, _ in sorted(self._cells.items(),
                               key=lambda kv: kv[1])[:len(self._cells) - self._max]:
                del self._cells[k]

    def cells(self):
        """Remembered cell centers, Nx2 float32 odom XY."""
        if not self._cells:
            return np.empty((0, 2), dtype=np.float32)
        idx = np.array(list(self._cells.keys()), dtype=np.float32)
        return (idx + 0.5) * self._cell


def stop_gate(xy_base, stop_x, half_width):
    """True when any cell (Nx2 base-frame XY) sits in the forward corridor
    [0, stop_x] x [-half_width, half_width]."""
    xy = np.asarray(xy_base, dtype=np.float32).reshape(-1, 2)
    if len(xy) == 0:
        return False
    return bool(((xy[:, 0] >= 0.0) & (xy[:, 0] <= stop_x)
                 & (np.abs(xy[:, 1]) <= half_width)).any())
