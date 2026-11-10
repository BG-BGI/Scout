"""Wall geometry -> nav2 map_server trinary occupancy grid (numpy only).

Source frame is y-up. `transform` is the triple (s, R, t) that calib.similarity and
calib.map_frame return: map_metres = s * R @ p + t. Widths are in source units and are
scaled by s. The grid is returned in PGM row order (row 0 is the top row, max y).

Walls are occupied, enclosed floor is free, everything outside the building is unknown.
Outside is told from inside on a sealed copy of the wall mask (doorways bridged by a
directional closing); the returned grid keeps the doorways open.
"""

import math
from dataclasses import dataclass

import numpy as np

OCCUPIED = 0
FREE = 254
UNKNOWN = 205
OCCUPIED_THRESH = 0.65
# nav2 reads (255 - 205) / 255 = 0.196 as free under free_thresh 0.25, so unknown needs <= 0.196.
FREE_THRESH = 0.196
MAX_CELLS = 4000 * 4000
CLOSE_M = 1.2
CLOSE_ORIENTATIONS = 16
MIN_HALF_CELLS = math.sqrt(0.5)
MAX_MASK_SUPERSAMPLE = 4
ORIGIN_DECIMALS = 6
EPS = 1e-9
PGM_MAXVAL = 255
DEFAULT_IMAGE = "map.pgm"


class PlanRasterError(ValueError):
    pass


class GridTooLargeError(PlanRasterError):
    pass


@dataclass(frozen=True, eq=False)
class Polyline:
    """Stroked wall centreline; `width` in source units (0 = hairline, still one cell)."""

    points: np.ndarray
    width: float = 0.0


@dataclass(frozen=True, eq=False)
class Fill:
    """Filled wall polygon; several rings are filled together with the even-odd rule."""

    rings: tuple


@dataclass(frozen=True, eq=False)
class Mask:
    """Pre-rasterized wall pixels (a rendered raster page). `bitmap` is bool with row 0 at the
    top; (`left`, `top`) is the source-frame position of its top-left corner (y-up) and
    (`pixel_w`, `pixel_h`) the source size of one pixel."""

    bitmap: np.ndarray
    left: float
    top: float
    pixel_w: float
    pixel_h: float


@dataclass(frozen=True, eq=False)
class Geometry:
    polylines: tuple = ()
    fills: tuple = ()
    masks: tuple = ()


def rasterize(geometry, transform, resolution, *, close_m=CLOSE_M):
    res = _positive(resolution, "resolution")
    close_m = _positive(close_m, "close_m")
    s, rot, trans = _check_transform(transform)
    lines, fills, masks = _to_world(geometry, s, rot, trans)
    ox, oy, h, w = _frame(lines, fills, masks, res, close_m)
    walls = _wall_mask(lines, fills, masks, np.array([ox, oy]), res, (h, w))
    if not walls.any():
        raise PlanRasterError("geometry produced no occupied cells")
    exterior = _exterior(walls, res, close_m)
    grid = np.full((h, w), UNKNOWN, dtype=np.uint8)
    grid[~exterior] = FREE
    grid[walls] = OCCUPIED
    return np.ascontiguousarray(grid[::-1]), _meta(res, ox, oy)


def encode_pgm(grid):
    h, w = grid.shape
    return f"P5\n{w} {h}\n{PGM_MAXVAL}\n".encode() + np.ascontiguousarray(grid).tobytes()


def yaml_text(meta):
    ox, oy, yaw = meta["origin"]
    return (
        f"image: {meta['image']}\n"
        f"mode: {meta['mode']}\n"
        f"resolution: {meta['resolution']!r}\n"
        f"origin: [{ox!r}, {oy!r}, {yaw!r}]\n"
        f"negate: {meta['negate']}\n"
        f"occupied_thresh: {meta['occupied_thresh']!r}\n"
        f"free_thresh: {meta['free_thresh']!r}\n"
    )


def _meta(res, ox, oy):
    return {
        "image": DEFAULT_IMAGE,
        "mode": "trinary",
        "resolution": res,
        "origin": [ox, oy, 0.0],
        "negate": 0,
        "occupied_thresh": OCCUPIED_THRESH,
        "free_thresh": FREE_THRESH,
    }


def _positive(value, what):
    try:
        v = float(value)
    except (TypeError, ValueError) as e:
        raise PlanRasterError(f"{what} must be a number") from e
    if not math.isfinite(v) or v <= 0:
        raise PlanRasterError(f"{what} must be finite and > 0")
    return v


def _check_transform(transform):
    try:
        s, rot, trans = transform
        rot = np.asarray(rot, dtype=float)
        trans = np.asarray(trans, dtype=float)
        s = float(s)
    except (TypeError, ValueError) as e:
        raise PlanRasterError("transform must be (scale, R 2x2, t 2)") from e
    if rot.shape != (2, 2) or trans.shape != (2,):
        raise PlanRasterError("transform must be (scale, R 2x2, t 2)")
    if not (math.isfinite(s) and np.isfinite(rot).all() and np.isfinite(trans).all()):
        raise PlanRasterError("transform must be finite")
    if s <= 0:
        raise PlanRasterError("transform scale must be > 0")
    return s, rot, trans


def _as_points(raw, what, minimum):
    try:
        a = np.asarray(raw, dtype=float)
    except (TypeError, ValueError) as e:
        raise PlanRasterError(f"{what}: points must be numeric") from e
    if a.ndim != 2 or a.shape[1] != 2 or len(a) < minimum:
        raise PlanRasterError(f"{what}: need an (N, 2) array with N >= {minimum}")
    if not np.isfinite(a).all():
        raise PlanRasterError(f"{what}: points must be finite")
    return a


def _to_world(geometry, s, rot, trans):
    def xf(pts):
        return s * (pts @ rot.T) + trans

    lines = []
    for pl in geometry.polylines:
        width = float(pl.width)
        if not math.isfinite(width) or width < 0:
            raise PlanRasterError("polyline width must be finite and >= 0")
        lines.append((xf(_as_points(pl.points, "polyline", 1)), width * s))
    fills = []
    for fill in geometry.fills:
        rings = [xf(_as_points(r, "fill ring", 3)) for r in fill.rings]
        if not rings:
            raise PlanRasterError("fill has no rings")
        fills.append(rings)
    masks = [m for m in (_mask_points(mask, xf, s) for mask in geometry.masks) if m is not None]
    if not lines and not fills and not masks:
        raise PlanRasterError("no geometry to rasterize")
    return lines, fills, masks


def _mask_points(mask, xf, s):
    """(centres in map metres, pixel edge in metres) of a mask's set pixels, None if empty."""
    bitmap = np.asarray(mask.bitmap)
    sizes = (mask.left, mask.top, mask.pixel_w, mask.pixel_h)
    if bitmap.ndim != 2 or not all(math.isfinite(v) for v in sizes):
        raise PlanRasterError("mask needs a 2-D bitmap and finite placement")
    if mask.pixel_w <= 0 or mask.pixel_h <= 0:
        raise PlanRasterError("mask pixel size must be > 0")
    rows, cols = np.nonzero(bitmap)
    if len(rows) == 0:
        return None
    x = mask.left + (cols + 0.5) * mask.pixel_w
    y = mask.top - (rows + 0.5) * mask.pixel_h
    src = np.stack([x, y], axis=1)
    return xf(src), s * max(mask.pixel_w, mask.pixel_h)


def _frame(lines, fills, masks, res, close_m):
    lo = np.full(2, np.inf)
    hi = np.full(2, -np.inf)
    for centres, edge in masks:
        lo = np.minimum(lo, centres.min(axis=0) - edge)
        hi = np.maximum(hi, centres.max(axis=0) + edge)
    for pts, width in lines:
        lo = np.minimum(lo, pts.min(axis=0) - width / 2)
        hi = np.maximum(hi, pts.max(axis=0) + width / 2)
    for rings in fills:
        for ring in rings:
            lo = np.minimum(lo, ring.min(axis=0))
            hi = np.maximum(hi, ring.max(axis=0))
    margin = close_m + 2 * res
    ox = round(math.floor((lo[0] - margin) / res + EPS) * res, ORIGIN_DECIMALS)
    oy = round(math.floor((lo[1] - margin) / res + EPS) * res, ORIGIN_DECIMALS)
    w = math.ceil((hi[0] + margin - ox) / res)
    h = math.ceil((hi[1] + margin - oy) / res)
    if w * h > MAX_CELLS:
        raise GridTooLargeError(
            f"grid would be {w} x {h} = {w * h} cells at {res} m/cell; cap is {MAX_CELLS} "
            "(check the scale, or use a coarser resolution)"
        )
    return ox, oy, int(h), int(w)


def _wall_mask(lines, fills, masks, origin, res, shape):
    h, w = shape
    diff = np.zeros((h, w + 1), dtype=np.int32)
    filled = np.zeros(shape, dtype=bool)
    for centres, edge in masks:
        _stamp_mask(filled, (centres - origin) / res, edge / res)
    for pts, width in lines:
        _stroke(diff, (pts - origin) / res, width / (2 * res))
    for rings in fills:
        _fill(filled, diff, [(r - origin) / res for r in rings])
    return (np.cumsum(diff, axis=1)[:, :w] > 0) | filled


def _stamp_mask(filled, cells, edge):
    """Mark the cell under each mask pixel; coarse pixels are supersampled so none are skipped."""
    k = max(1, math.ceil(edge / (0.5 + EPS)))
    if k > MAX_MASK_SUPERSAMPLE:
        raise PlanRasterError("mask pixels are too coarse for this resolution; render finer")
    h, w = filled.shape
    offsets = (np.arange(k) + 0.5) / k - 0.5
    for dx in offsets:
        for dy in offsets:
            c = np.floor(cells + np.array([dx, dy]) * edge).astype(np.int64)
            ok = (c[:, 0] >= 0) & (c[:, 0] < w) & (c[:, 1] >= 0) & (c[:, 1] < h)
            filled[c[ok, 1], c[ok, 0]] = True


def _dedupe(pts):
    if len(pts) < 2:
        return pts
    step = np.hypot(*np.diff(pts, axis=0).T)
    return pts[np.concatenate(([True], step > EPS))]


def _stroke(diff, pts, hw):
    """Butt-capped rectangles per segment plus round joins; hairlines widen to one cell."""
    thin = hw < MIN_HALF_CELLS
    half = MIN_HALF_CELLS if thin else hw
    ext = MIN_HALF_CELLS if thin else 0.0
    pts = _dedupe(pts)
    if len(pts) == 1:
        _add_span(diff, *_disc_span(pts[0], half))
        return
    for a, b in zip(pts[:-1], pts[1:], strict=True):
        _add_span(diff, *_rect_span(a, b, half, ext))
    if not thin:
        closed = np.allclose(pts[0], pts[-1])
        for p in pts[:-1] if closed else pts[1:-1]:
            _add_span(diff, *_disc_span(p, half))


def _rect_span(a, b, half, ext):
    u = (b - a) / math.hypot(*(b - a))
    n = np.array([-u[1], u[0]])
    a2, b2 = a - u * ext, b + u * ext
    return _convex_span(np.array([a2 + n * half, b2 + n * half, b2 - n * half, a2 - n * half]))


def _disc_span(c, r):
    r0 = math.ceil(c[1] - r - 0.5 - EPS)
    r1 = math.floor(c[1] + r - 0.5 + EPS)
    y = np.arange(r0, r1 + 1) + 0.5
    sq = r * r - (y - c[1]) ** 2
    half = np.sqrt(np.where(sq >= 0, sq, np.nan))
    return r0, c[0] - half, c[0] + half


def _convex_span(poly):
    r0 = math.ceil(poly[:, 1].min() - 0.5 - EPS)
    r1 = math.floor(poly[:, 1].max() - 0.5 + EPS)
    y = np.arange(r0, r1 + 1) + 0.5
    p, q = poly, np.roll(poly, -1, axis=0)
    dy = (q[:, 1] - p[:, 1])[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (y[None, :] - p[:, 1][:, None]) / dy
    valid = (dy != 0) & (t >= -EPS) & (t <= 1 + EPS)
    x = p[:, 0][:, None] + t * (q[:, 0] - p[:, 0])[:, None]
    return r0, np.where(valid, x, np.inf).min(axis=0), np.where(valid, x, -np.inf).max(axis=0)


def _add_span(diff, r0, xmin, xmax):
    """One interval per row (unique rows), accumulated as +1/-1 for a later cumsum."""
    h, w1 = diff.shape
    w = w1 - 1
    rows = np.arange(r0, r0 + len(xmin))
    ok = np.isfinite(xmin) & np.isfinite(xmax) & (rows >= 0) & (rows < h)
    rows, xmin, xmax = rows[ok], xmin[ok], xmax[ok]
    start = np.clip(np.ceil(xmin - 0.5 - EPS), 0, w).astype(np.int64)
    end = np.clip(np.floor(xmax - 0.5 + EPS) + 1, 0, w).astype(np.int64)
    keep = end > start
    diff[rows[keep], start[keep]] += 1
    diff[rows[keep], end[keep]] -= 1


def _fill(filled, diff, rings):
    if len(rings) == 1 and _thickness(rings[0]) < 1.0:
        _stroke(diff, np.vstack([rings[0], rings[0][:1]]), 0.0)
    p = np.concatenate(rings)
    q = np.concatenate([np.roll(r, -1, axis=0) for r in rings])
    h, w = filled.shape
    r0 = max(0, math.ceil(p[:, 1].min() - 0.5))
    r1 = min(h - 1, math.floor(p[:, 1].max() - 0.5))
    c0 = max(0, math.floor(p[:, 0].min() - 0.5))
    c1 = min(w - 1, math.ceil(p[:, 0].max() - 0.5))
    if r1 < r0 or c1 < c0:
        return
    rows, cols = _crossings(p, q)
    keep = (rows >= r0) & (rows <= r1)
    flat = (rows[keep] - r0) * (c1 - c0 + 2) + np.clip(cols[keep], c0, c1 + 1) - c0
    par = (np.bincount(flat, minlength=(r1 - r0 + 1) * (c1 - c0 + 2)) & 1).astype(np.uint8)
    par = par.reshape(r1 - r0 + 1, c1 - c0 + 2)
    inside = (np.cumsum(par, axis=1, dtype=np.uint8) & 1).astype(bool)
    filled[r0 : r1 + 1, c0 : c1 + 1] |= inside[:, : c1 - c0 + 1]


def _crossings(p, q):
    """(row, first-centre-column) of every edge crossing of a row-centre scanline."""
    ylo = np.minimum(p[:, 1], q[:, 1])
    yhi = np.maximum(p[:, 1], q[:, 1])
    first = np.ceil(ylo - 0.5).astype(np.int64)
    count = np.maximum(np.ceil(yhi - 0.5).astype(np.int64) - first, 0)
    edge = np.repeat(np.arange(len(p)), count)
    offset = np.arange(count.sum()) - (np.cumsum(count) - count)[edge]
    rows = first[edge] + offset
    t = (rows + 0.5 - p[edge, 1]) / (q[edge, 1] - p[edge, 1])
    x = p[edge, 0] + t * (q[edge, 0] - p[edge, 0])
    return rows, np.ceil(x - 0.5).astype(np.int64)


def _thickness(ring):
    x, y = ring[:, 0], ring[:, 1]
    xn, yn = np.roll(x, -1), np.roll(y, -1)
    area = abs(float(np.sum(x * yn - xn * y))) / 2
    perimeter = float(np.hypot(xn - x, yn - y).sum())
    return 2 * area / perimeter if perimeter > 0 else 0.0


def _exterior(walls, res, close_m):
    sealed = walls | _close_lines(walls, math.ceil(close_m / res) + 1)
    border = np.zeros_like(walls)
    border[0, :] = border[-1, :] = True
    border[:, 0] = border[:, -1] = True
    return _reachable(~sealed, border)


def _close_lines(walls, length):
    """Union of 1-D closings along several directions: bridges doorways, not open faces."""
    closed = np.zeros_like(walls)
    for k in range(CLOSE_ORIENTATIONS):
        offsets = _line_offsets(math.pi * k / CLOSE_ORIENTATIONS, length)
        closed |= _erode(_dilate(walls, offsets), offsets)
    return closed


def _line_offsets(angle, length):
    i = np.arange(length)
    pairs = np.stack([np.rint(i * math.sin(angle)), np.rint(i * math.cos(angle))], axis=1)
    return [(int(dy), int(dx)) for dy, dx in np.unique(pairs, axis=0)]


def _dilate(mask, offsets):
    out = np.zeros_like(mask)
    for dy, dx in offsets:
        _shift_or(out, mask, dy, dx)
    return out


def _erode(mask, offsets):
    return ~_dilate(~mask, [(-dy, -dx) for dy, dx in offsets])


def _shift_or(out, src, dy, dx):
    h, w = src.shape
    y0, y1 = max(dy, 0), h + min(dy, 0)
    x0, x1 = max(dx, 0), w + min(dx, 0)
    if y0 < y1 and x0 < x1:
        out[y0:y1, x0:x1] |= src[y0 - dy : y1 - dy, x0 - dx : x1 - dx]


def _reachable(passable, seeds):
    """4-connected flood fill by alternating row-run and column-run spreading."""
    reach = seeds & passable
    while True:
        before = int(reach.sum())
        reach = _spread(reach, passable)
        reach = _spread(reach.T, passable.T).T
        if int(reach.sum()) == before:
            return reach


def _spread(reach, passable):
    prev = np.zeros_like(passable)
    prev[:, 1:] = passable[:, :-1]
    run = np.cumsum((passable & ~prev).ravel(), dtype=np.int64).reshape(passable.shape)
    hit = np.zeros(int(run.max()) + 1, dtype=bool)
    hit[run[reach & passable]] = True
    return passable & hit[run]
