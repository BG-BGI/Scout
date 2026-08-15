"""Serpentine coverage-route planning over an occupancy grid.

Extracted from patrol_capture so the scanline clip, obstacle inflation, and
stripe splitting are testable without a running node. Inputs are plain arrays
and tuples; outputs are lists of {x, y, yaw} dicts with builtin floats (numpy
scalars blow up yaml.safe_dump when the route is saved — regression-tested).
"""

import math

import numpy as np

OCCUPIED = 50  # nav2 lethal convention (matches robot_profile occupied_threshold)


def scanline(poly, wy):
    """Sorted [(xa, xb), ...] spans where the horizontal line y=wy is inside poly."""
    xs = []
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 <= wy < y2) or (y2 <= wy < y1):
            xs.append(x1 + (wy - y1) * (x2 - x1) / (y2 - y1))
    xs.sort()
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]


def inflate(blocked, cells):
    """Grow a boolean obstacle mask by `cells` 4-connected dilations."""
    for _ in range(cells):
        d = blocked.copy()
        d[1:, :] |= blocked[:-1, :]
        d[:-1, :] |= blocked[1:, :]
        d[:, 1:] |= blocked[:, :-1]
        d[:, :-1] |= blocked[:, 1:]
        blocked = d
    return blocked


def plan_coverage(grid, origin_xy, resolution, poly, *,
                  spacing, inflation, min_run_m=0.45):
    """Serpentine stripes over free/unknown cells inside `poly`.

    grid: 2D int array (height, width), OccupancyGrid convention (>=OCCUPIED is
    lethal; -1 unknown counts as coverable). origin_xy: world coord of cell
    (0,0). Returns [{x, y, yaw}, ...] with builtin floats.
    """
    ox, oy = origin_xy
    res = resolution
    h, w = grid.shape
    blocked = grid >= OCCUPIED
    blocked = inflate(blocked, max(1, int(round(inflation / res))))

    def cell_x(wx):
        return int((wx - ox) / res)

    def cell_y(wy):
        return int((wy - oy) / res)

    min_run = max(2, int(round(min_run_m / res)))   # skip slivers < robot length
    y0 = min(p[1] for p in poly)
    y1 = max(p[1] for p in poly)
    route = []
    flip = False
    wy = y0 + spacing / 2.0
    while wy < y1:
        iy = cell_y(wy)
        if 0 <= iy < h:
            segs = []
            for xa, xb in scanline(poly, wy):
                ca = max(0, cell_x(xa))
                cb = min(w - 1, cell_x(xb))
                if cb - ca < min_run:
                    continue
                open_row = ~blocked[iy, ca:cb + 1]
                idx = np.flatnonzero(np.diff(np.concatenate(
                    ([0], open_row.view(np.int8), [0]))))
                segs.extend((ca + idx[i], ca + idx[i + 1] - 1)
                            for i in range(0, len(idx), 2)
                            if idx[i + 1] - idx[i] >= min_run)
            if flip:
                segs = [(b, a) for a, b in reversed(segs)]
            for a, b in segs:
                # plain floats: numpy scalars blow up yaml.safe_dump on save.
                wxa = float(ox + (a + 0.5) * res)
                wxb = float(ox + (b + 0.5) * res)
                yaw = 0.0 if wxb >= wxa else math.pi
                route.append({'x': round(wxa, 3), 'y': round(float(wy), 3),
                              'yaw': round(yaw, 3)})
                route.append({'x': round(wxb, 3), 'y': round(float(wy), 3),
                              'yaw': round(yaw, 3)})
        flip = not flip
        wy += spacing
    return route
