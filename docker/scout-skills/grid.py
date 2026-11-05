"""Structural grid: construction-drawing grid lines laid over a Scout map.

Pure Python (stdlib only, no ROS, no FastAPI). Storage is always meters in the
SLAM map frame; `units` ('ft'|'m') is display only.

Grid frame: local +u = origin.yaw direction, local +v = u rotated +90 deg.
  numbers  ("1","2",...) are lines of constant u (they run along v);
           offsets_m are positions along u.
  letters  ("A","B",...) are lines of constant v (they run along u);
           offsets_m are positions along v.
Bay "1B" = the cell between number lines 1..2 and letter lines B..C
(bay_anchor "lower": a bay is named by its LOWER-offset bounding lines). The
last line of an axis has no bay after it.

Two kinds of grid, told apart by `mode`:
  "structural" (default)  typed in from the drawings (define/align). Never
                          touched by auto-sizing.
  "auto"                  the standard grid: square cells of `cell_m` (30 ft by
                          default), yaw aligned to the walls and FROZEN at
                          creation, origin snapped to a whole cell and frozen.
                          Lines are APPENDED at either end as the map grows
                          (extend_to); labels never change. Each axis carries
                          `index_start` (integer index of its first line); line
                          index k sits at offset k*cell_m.

Auto label scheme (total over integers, anchored at the frozen origin):
  numbers  index i >= 0 -> "1","2","3",...   (i+1)
           index i <  0 -> ".1",".2",...     (".k" = k lines before line 1)
  letters  index j >= 0 -> A..Z, AA, AB, ... (bijective base 26)
           index j <  0 -> ".A",".B",...     (".A" = the line just below A)
A leading dot marks "before the origin". Positive labels never contain a dot,
digit labels never collide with letter labels, "-" is avoided because the
label parser treats it as a separator. Bay ".1.A" lies between lines ".1" and 1.

Both are checked by validate() (auto labels/offsets must follow the scheme).

docker/fleet-status/gridstore.py duplicates validate() (separate image);
scout/test/test_grid_rest.py keeps the two in agreement.
"""
import json
import math
import os
import re

FT_M = 0.3048
VERSION = 1
MAX_LINES = 200
MAX_ABS = 1.0e6
MODES = ("structural", "auto")
MIN_CELL_M = 1.0
MAX_CELL_M = 1000.0
_LABEL_RE = re.compile(r"^[A-Za-z0-9.]{1,8}$")
_SEP_RE = re.compile(r"[-/_\s]+")
_MAP_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
SESSION = "_session"


class GridError(ValueError):
    pass


# --- validation --------------------------------------------------------------

def _num(v, what):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise GridError(f"{what} must be a number")
    v = float(v)
    if not math.isfinite(v) or abs(v) > MAX_ABS:
        raise GridError(f"{what} must be finite and |.| <= {MAX_ABS:g}")
    return v


def _axis(ax, name):
    if not isinstance(ax, dict):
        raise GridError(f"{name} must be an object")
    labels, offs = ax.get("labels"), ax.get("offsets_m")
    if not isinstance(labels, list) or not isinstance(offs, list):
        raise GridError(f"{name}.labels and {name}.offsets_m must be lists")
    if len(labels) != len(offs):
        raise GridError(f"{name}: labels and offsets_m differ in length")
    if not 2 <= len(labels) <= MAX_LINES:
        raise GridError(f"{name}: need 2..{MAX_LINES} lines")
    out_l = []
    for lb in labels:
        if not isinstance(lb, str) or not _LABEL_RE.match(lb.strip()):
            raise GridError(f"{name}: bad label {lb!r} (1-8 chars of A-Z a-z 0-9 .)")
        out_l.append(lb.strip())
    if len({s.upper() for s in out_l}) != len(out_l):
        raise GridError(f"{name}: labels must be unique (case-insensitive)")
    out_o = [_num(o, f"{name}.offsets_m") for o in offs]
    if any(b <= a for a, b in zip(out_o, out_o[1:], strict=False)):
        raise GridError(f"{name}: offsets_m must be strictly increasing")
    out = {"labels": out_l, "offsets_m": out_o}
    idx = ax.get("index_start")
    if idx is not None:
        if isinstance(idx, bool) or not isinstance(idx, int) or abs(idx) > MAX_LINES * 1000:
            raise GridError(f"{name}.index_start must be an integer")
        out["index_start"] = idx
    return out


def validate(grid):
    """Strict check; returns the normalized grid (floats, stripped labels)."""
    if not isinstance(grid, dict):
        raise GridError("grid must be an object")
    if grid.get("version", VERSION) != VERSION:
        raise GridError(f"unsupported grid version {grid.get('version')!r}")
    units = grid.get("units", "ft")
    if units not in ("ft", "m"):
        raise GridError("units must be 'ft' or 'm'")
    o = grid.get("origin")
    if not isinstance(o, dict):
        raise GridError("origin must be an object")
    origin = {"x": _num(o.get("x"), "origin.x"), "y": _num(o.get("y"), "origin.y"),
              "yaw_rad": _num(o.get("yaw_rad"), "origin.yaw_rad")}
    if abs(origin["yaw_rad"]) > 2 * math.pi + 1e-6:
        raise GridError("origin.yaw_rad must be within +-2pi")
    nums = _axis(grid.get("numbers"), "numbers")
    lets = _axis(grid.get("letters"), "letters")
    both = {s.upper() for s in nums["labels"]} & {s.upper() for s in lets["labels"]}
    if both:
        raise GridError(f"labels shared by numbers and letters: {sorted(both)}")
    if grid.get("bay_anchor", "lower") != "lower":
        raise GridError("bay_anchor must be 'lower'")
    mode = grid.get("mode", "structural")
    if mode not in MODES:
        raise GridError("mode must be 'structural' or 'auto'")
    out = {"version": VERSION, "mode": mode, "units": units, "origin": origin,
           "numbers": nums, "letters": lets, "bay_anchor": "lower"}
    if mode == "auto":
        cell = _num(grid.get("cell_m"), "cell_m")
        if not MIN_CELL_M <= cell <= MAX_CELL_M:
            raise GridError(f"cell_m must be within {MIN_CELL_M}..{MAX_CELL_M} m")
        for name, ax, lab in (("numbers", nums, number_label),
                              ("letters", lets, letter_label)):
            i0 = ax.get("index_start")
            if i0 is None:
                raise GridError(f"auto grid: {name}.index_start required")
            for k, (lb, off) in enumerate(zip(ax["labels"], ax["offsets_m"],
                                              strict=True)):
                if lb != lab(i0 + k) or abs(off - (i0 + k) * cell) > 1e-6:
                    raise GridError(
                        f"auto grid: {name} line {k} must be {lab(i0 + k)!r} at "
                        f"{(i0 + k) * cell:.6g} m (label stability scheme)")
        out["cell_m"] = cell
    return out


# --- construction ------------------------------------------------------------

def to_m(value, units):
    return value * (FT_M if units == "ft" else 1.0)


def from_m(value_m, units):
    return value_m / (FT_M if units == "ft" else 1.0)


def axis_offsets(n, spacing, units="ft"):
    """Offsets (m) for n lines. spacing: one number (uniform) or a list of n-1
    gaps, in `units`."""
    if units not in ("ft", "m"):
        raise GridError("units must be 'ft' or 'm'")
    if isinstance(spacing, (int, float)) and not isinstance(spacing, bool):
        gaps = [spacing] * (n - 1)
    elif isinstance(spacing, (list, tuple)):
        gaps = list(spacing)
        if len(gaps) != n - 1:
            raise GridError(f"need {n - 1} spacings for {n} lines, got {len(gaps)}")
    else:
        raise GridError("spacing must be a number or a list of numbers")
    offs = [0.0]
    for g in gaps:
        g = _num(g, "spacing")
        if g <= 0:
            raise GridError("spacing must be > 0")
        offs.append(offs[-1] + to_m(g, units))
    return offs


def define(numbers, letters, number_spacing, letter_spacing, units="ft",
           origin=None):
    """Grid from label lists + uniform or per-gap spacing (in `units`).
    origin: {x,y,yaw_rad} or None (-> 0,0,0, i.e. unaligned)."""
    numbers, letters = list(numbers), list(letters)
    if origin is None:
        origin = {"x": 0.0, "y": 0.0, "yaw_rad": 0.0}
    return validate({
        "version": VERSION, "units": units, "origin": origin,
        "numbers": {"labels": numbers,
                    "offsets_m": axis_offsets(len(numbers), number_spacing, units)},
        "letters": {"labels": letters,
                    "offsets_m": axis_offsets(len(letters), letter_spacing, units)},
        "bay_anchor": "lower",
    })


def _axis_index(ax, label):
    want = str(label).strip().upper()
    for i, lb in enumerate(ax["labels"]):
        if lb.upper() == want:
            return i
    return None


def intersection_uv(grid, number, letter):
    i = _axis_index(grid["numbers"], number)
    j = _axis_index(grid["letters"], letter)
    if i is None:
        raise GridError(f"no number line {number!r}")
    if j is None:
        raise GridError(f"no letter line {letter!r}")
    return grid["numbers"]["offsets_m"][i], grid["letters"]["offsets_m"][j]


def align(grid, ref1, ref2):
    """Set origin/yaw from two map-frame points with known grid coordinates.
    ref = {"x","y","number":"1","letter":"A"} (an intersection). Returns
    (grid, residual_m): residual = |measured distance - distance implied by the
    configured spacing|, so a bad click shows up. Two points fix the pose."""
    g = validate(grid)
    pts = []
    for r in (ref1, ref2):
        if not isinstance(r, dict):
            raise GridError("ref must be an object {x,y,number,letter}")
        u, v = intersection_uv(g, r.get("number"), r.get("letter"))
        pts.append((_num(r.get("x"), "ref.x"), _num(r.get("y"), "ref.y"), u, v))
    (x1, y1, u1, v1), (x2, y2, u2, v2) = pts
    dx, dy, du, dv = x2 - x1, y2 - y1, u2 - u1, v2 - v1
    dg, dm = math.hypot(du, dv), math.hypot(dx, dy)
    if dg < 1e-9:
        raise GridError("references are the same grid intersection")
    if dm < 1e-9:
        raise GridError("reference map points coincide")
    yaw = math.atan2(dy, dx) - math.atan2(dv, du)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    c, s = math.cos(yaw), math.sin(yaw)
    ox = x1 - (c * u1 - s * v1)
    oy = y1 - (s * u1 + c * v1)
    out = dict(g)
    out["origin"] = {"x": ox, "y": oy, "yaw_rad": yaw}
    return validate(out), abs(dm - dg)


# --- frames ------------------------------------------------------------------

def from_grid_frame(grid, u, v):
    o = grid["origin"]
    c, s = math.cos(o["yaw_rad"]), math.sin(o["yaw_rad"])
    return o["x"] + c * u - s * v, o["y"] + s * u + c * v


def to_grid_frame(grid, x, y):
    o = grid["origin"]
    c, s = math.cos(o["yaw_rad"]), math.sin(o["yaw_rad"])
    dx, dy = x - o["x"], y - o["y"]
    return c * dx + s * dy, -s * dx + c * dy


# --- labels ------------------------------------------------------------------

def _pairs(grid, text):
    """All (number_idx, letter_idx) readings of text, either order."""
    t = str(text).strip().upper()
    if not t:
        raise GridError("empty label")
    nums = {lb.upper(): i for i, lb in enumerate(grid["numbers"]["labels"])}
    lets = {lb.upper(): j for j, lb in enumerate(grid["letters"]["labels"])}
    cands = []
    parts = [p for p in _SEP_RE.split(t) if p]
    if len(parts) == 2:
        splits = [(parts[0], parts[1])]
    elif len(parts) == 1:
        splits = [(t[:k], t[k:]) for k in range(1, len(t))]
    else:
        splits = []
    for a, b in splits:
        if a in nums and b in lets:
            cands.append((nums[a], lets[b]))
        if a in lets and b in nums:
            cands.append((nums[b], lets[a]))
    return sorted(set(cands))


def parse_label(grid, text):
    """'1B','b1','1-B','B-1','B/1' (any case) -> (number_idx, letter_idx)."""
    c = _pairs(grid, text)
    if not c:
        raise GridError(
            f"cannot read {text!r} as number+letter; numbers "
            f"{grid['numbers']['labels']}, letters {grid['letters']['labels']}")
    if len(c) > 1:
        raise GridError(f"{text!r} is ambiguous; use a separator like '1-B'")
    return c[0]


def bay_label(grid, i, j):
    return grid["numbers"]["labels"][i] + grid["letters"]["labels"][j]


# --- resolve / locate --------------------------------------------------------

def _bounds(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return {"min_x": min(xs), "min_y": min(ys), "max_x": max(xs), "max_y": max(ys)}


def resolve(grid, label, kind="bay"):
    """Bay or intersection -> geometry in the map frame. Corners are CCW:
    (u0,v0),(u1,v0),(u1,v1),(u0,v1). Intersection: single point, empty polygon."""
    g = validate(grid)
    if kind not in ("bay", "intersection"):
        raise GridError("kind must be 'bay' or 'intersection'")
    i, j = parse_label(g, label)
    no, lo = g["numbers"]["offsets_m"], g["letters"]["offsets_m"]
    units = g["units"]
    if kind == "intersection":
        u, v = no[i], lo[j]
        x, y = from_grid_frame(g, u, v)
        return {"label": bay_label(g, i, j), "kind": kind, "center": {"x": x, "y": y},
                "corners": [], "polygon": [], "bounds": _bounds([(x, y)]),
                "u_range": [u, u], "v_range": [v, v], "size_m": (0.0, 0.0),
                "size_in_units": (0.0, 0.0), "units": units, "u": u, "v": v}
    if i + 1 >= len(no):
        raise GridError(
            f"number line {g['numbers']['labels'][i]!r} is the last line: no bay "
            f"after it (bays use the lower line: last bay is "
            f"{g['numbers']['labels'][-2]}..)")
    if j + 1 >= len(lo):
        raise GridError(
            f"letter line {g['letters']['labels'][j]!r} is the last line: no bay "
            f"after it (last bay letter is {g['letters']['labels'][-2]!r})")
    u0, u1, v0, v1 = no[i], no[i + 1], lo[j], lo[j + 1]
    corners = [from_grid_frame(g, u, v) for u, v in
               ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]
    cx, cy = from_grid_frame(g, (u0 + u1) / 2, (v0 + v1) / 2)
    pts = [{"x": x, "y": y} for x, y in corners]
    du, dv = u1 - u0, v1 - v0
    return {"label": bay_label(g, i, j), "kind": kind, "center": {"x": cx, "y": cy},
            "corners": pts, "polygon": pts, "bounds": _bounds(corners),
            "u_range": [u0, u1], "v_range": [v0, v1], "size_m": (du, dv),
            "size_in_units": (from_m(du, units), from_m(dv, units)), "units": units}


def _slot(offs, p):
    """Index k with offs[k] <= p < offs[k+1] (last line counts into the last
    bay); None if outside."""
    if p < offs[0] or p > offs[-1]:
        return None
    for k in range(len(offs) - 1):
        if p < offs[k + 1]:
            return k
    return len(offs) - 2


def _nearest(offs, p):
    return min(range(len(offs)), key=lambda k: abs(offs[k] - p))


def locate(grid, x, y):
    """Map point -> {bay (label|None), nearest_intersection, u, v, inside}."""
    g = validate(grid)
    u, v = to_grid_frame(g, x, y)
    no, lo = g["numbers"]["offsets_m"], g["letters"]["offsets_m"]
    i, j = _slot(no, u), _slot(lo, v)
    inside = i is not None and j is not None
    ni, nj = _nearest(no, u), _nearest(lo, v)
    nx, ny = from_grid_frame(g, no[ni], lo[nj])
    return {
        "bay": bay_label(g, i, j) if inside else None,
        "inside": inside,
        "u": u, "v": v,
        "nearest_intersection": {
            "label": bay_label(g, ni, nj),
            "number": g["numbers"]["labels"][ni], "letter": g["letters"]["labels"][nj],
            "point": {"x": nx, "y": ny}, "distance_m": math.hypot(x - nx, y - ny)},
    }


def lines(grid):
    """Drawable segments in the map frame: one per number line (constant u,
    spanning the letter extent) and per letter line."""
    g = validate(grid)
    no, lo = g["numbers"]["offsets_m"], g["letters"]["offsets_m"]
    out = []
    for lb, u in zip(g["numbers"]["labels"], no, strict=True):
        a, b = from_grid_frame(g, u, lo[0]), from_grid_frame(g, u, lo[-1])
        out.append({"kind": "number", "label": lb, "offset_m": u,
                    "p1": {"x": a[0], "y": a[1]}, "p2": {"x": b[0], "y": b[1]}})
    for lb, v in zip(g["letters"]["labels"], lo, strict=True):
        a, b = from_grid_frame(g, no[0], v), from_grid_frame(g, no[-1], v)
        out.append({"kind": "letter", "label": lb, "offset_m": v,
                    "p1": {"x": a[0], "y": a[1]}, "p2": {"x": b[0], "y": b[1]}})
    return out


# --- automatic standard grid -------------------------------------------------
# BEGIN AUTO-GRID BLOCK (duplicated verbatim in docker/fleet-status/gridstore.py;
# scout/test/test_grid_auto.py keeps the two in agreement)

AUTO_CELL_FT = 30.0
_EPS = 1e-6
OCC_MIN = 65                 # occupancy >= this counts as a wall cell
YAW_MIN_CELLS = 200          # fewer occupied cells -> yaw 0
YAW_MIN_STRENGTH = 0.3       # circular resultant of the 4-theta histogram


def _bij(n):
    """1->A ... 26->Z, 27->AA (bijective base 26)."""
    s = ""
    while n > 0:
        n, r = divmod(n - 1, 26)
        s = chr(65 + r) + s
    return s


def number_label(i):
    """Number-line label for integer index i (see module docstring)."""
    return str(i + 1) if i >= 0 else "." + str(-i)


def letter_label(j):
    """Letter-line label for integer index j (see module docstring)."""
    return _bij(j + 1) if j >= 0 else "." + _bij(-j)


def _auto_axis(i0, i1, cell_m, labeler):
    idx = range(i0, i1 + 1)
    if len(idx) > MAX_LINES:
        raise GridError(f"extent needs {len(idx)} lines (max {MAX_LINES}); use a larger cell")
    return {"labels": [labeler(i) for i in idx],
            "offsets_m": [i * cell_m for i in idx], "index_start": i0}


def _norm_extent(e):
    if not isinstance(e, dict):
        raise GridError("extent must be an object {min_x,min_y,max_x,max_y}")
    out = {k: _num(e.get(k), f"extent.{k}") for k in ("min_x", "min_y", "max_x", "max_y")}
    if out["max_x"] <= out["min_x"] or out["max_y"] <= out["min_y"]:
        raise GridError("extent must have positive width and height")
    return out


def _corners(e):
    return [(e["min_x"], e["min_y"]), (e["max_x"], e["min_y"]),
            (e["max_x"], e["max_y"]), (e["min_x"], e["max_y"])]


def _index_range(lo, hi, cell_m):
    i0 = math.floor(lo / cell_m + _EPS)
    i1 = math.ceil(hi / cell_m - _EPS)
    return i0, max(i1, i0 + 1)


def auto_new(extent, yaw_rad=0.0, cell_ft=AUTO_CELL_FT):
    """New auto grid covering extent. Origin = lower-left corner of the extent
    in the yaw-rotated frame (anchored at map (0,0)) snapped DOWN to a whole
    cell; it never moves afterwards."""
    e = _norm_extent(extent)
    cell_m = to_m(_num(cell_ft, "cell_ft"), "ft")
    if not MIN_CELL_M <= cell_m <= MAX_CELL_M:
        raise GridError(f"cell must be within {MIN_CELL_M / FT_M:.1f}..{MAX_CELL_M / FT_M:.0f} ft")
    yaw = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
    c, s = math.cos(yaw), math.sin(yaw)
    us = [c * x + s * y for x, y in _corners(e)]
    vs = [-s * x + c * y for x, y in _corners(e)]
    u0 = math.floor(min(us) / cell_m + _EPS) * cell_m
    v0 = math.floor(min(vs) / cell_m + _EPS) * cell_m
    _, ni = _index_range(min(us) - u0, max(us) - u0, cell_m)
    _, nj = _index_range(min(vs) - v0, max(vs) - v0, cell_m)
    return validate({
        "version": VERSION, "mode": "auto", "units": "ft", "cell_m": cell_m,
        "origin": {"x": c * u0 - s * v0, "y": s * u0 + c * v0, "yaw_rad": yaw},
        "numbers": _auto_axis(0, ni, cell_m, number_label),
        "letters": _auto_axis(0, nj, cell_m, letter_label),
        "bay_anchor": "lower"})


def extend_to(grid, extent):
    """Auto grid grown (never shrunk) to cover extent: lines are APPENDED at
    either end, existing lines keep label and offset. Returns the validated
    grid (equal to the input when nothing grew)."""
    g = validate(grid)
    if g["mode"] != "auto":
        raise GridError("only auto grids can be extended (structural is never auto-sized)")
    e = _norm_extent(extent)
    cell = g["cell_m"]
    uv = [to_grid_frame(g, x, y) for x, y in _corners(e)]
    i0, i1 = _index_range(min(p[0] for p in uv), max(p[0] for p in uv), cell)
    j0, j1 = _index_range(min(p[1] for p in uv), max(p[1] for p in uv), cell)
    ni, nj = g["numbers"], g["letters"]
    a0, a1 = ni["index_start"], ni["index_start"] + len(ni["labels"]) - 1
    b0, b1 = nj["index_start"], nj["index_start"] + len(nj["labels"]) - 1
    a0, a1 = min(a0, i0), max(a1, i1)
    b0, b1 = min(b0, j0), max(b1, j1)
    out = dict(g)
    out["numbers"] = _auto_axis(a0, a1, cell, number_label)
    out["letters"] = _auto_axis(b0, b1, cell, letter_label)
    return validate(out)


# --- occupancy: extent, wall yaw, saved-map files ---------------------------
# occ = {"width","height","resolution","origin_x","origin_y","data"}; data is
# row-major, row 0 = BOTTOM (nav_msgs/OccupancyGrid), -1 unknown, 0..100.

def extent_from_occupancy(occ, margin=1):
    """Bounding box (map frame, m) of the KNOWN cells plus `margin` cells, or
    None when nothing is known."""
    w, h, res = occ["width"], occ["height"], occ["resolution"]
    data = occ["data"]
    c0, c1, r0, r1 = w, -1, h, -1
    for r in range(h):
        row = data[r * w:(r + 1) * w]
        first = next((c for c, v in enumerate(row) if v != -1), None)
        if first is None:
            continue
        last = w - 1 - next(c for c, v in enumerate(reversed(row)) if v != -1)
        c0, c1 = min(c0, first), max(c1, last)
        r0, r1 = min(r0, r), max(r1, r)
    if c1 < 0:
        return None
    ox, oy = occ["origin_x"], occ["origin_y"]
    return {"min_x": ox + (c0 - margin) * res, "max_x": ox + (c1 + 1 + margin) * res,
            "min_y": oy + (r0 - margin) * res, "max_y": oy + (r1 + 1 + margin) * res}


def estimate_yaw(occ, occ_min=OCC_MIN, min_cells=YAW_MIN_CELLS,
                 min_strength=YAW_MIN_STRENGTH):
    """Dominant wall direction mod 90 deg -> (yaw_rad in (-pi/4, pi/4], info).
    For each wall cell the local principal axis of the wall cells within ~0.6 m
    gives a line orientation phi (bias-free on raster stair-steps, unlike a
    pixel gradient); weight = coherence (0 at corners/blobs, 1 on a straight
    wall). phi is histogrammed as 4*phi (circular mean over the four-fold
    symmetry). Falls back to 0 when there are too few wall cells or the
    resultant length is weak."""
    w, h, data = occ["width"], occ["height"], occ["data"]
    r = max(3, round(0.6 / occ["resolution"]))
    pw = w + 2 * r
    img = bytearray(pw * (h + 2 * r))
    wall = []
    for k, v in enumerate(data):
        if v >= occ_min:
            y, x = divmod(k, w)
            p = (y + r) * pw + x + r
            img[p] = 1
            wall.append(p)
    info = {"occupied": len(wall), "strength": 0.0, "source": "fallback"}
    if len(wall) < min_cells:
        return 0.0, info
    offs = [(dx, dy, dy * pw + dx) for dy in range(-r, r + 1)
            for dx in range(-r, r + 1) if dx * dx + dy * dy <= r * r]
    step = max(1, len(wall) // 6000)
    sc = ss = sw = 0.0
    for p in wall[::step]:
        n = sx = sy = sxx = syy = sxy = 0
        for dx, dy, o in offs:
            if img[p + o]:
                n += 1
                sx += dx
                sy += dy
                sxx += dx * dx
                syy += dy * dy
                sxy += dx * dy
        if n < 6:
            continue
        cxx, cyy, cxy = sxx / n - (sx / n) ** 2, syy / n - (sy / n) ** 2, sxy / n - sx * sy / n / n
        tr = cxx + cyy
        if tr <= 1e-9:
            continue
        d = math.hypot(cxx - cyy, 2 * cxy)
        coh = d / tr
        a = 2.0 * math.atan2(2 * cxy, cxx - cyy)   # 4 * phi, phi = principal axis angle
        sc += coh * math.cos(a)
        ss += coh * math.sin(a)
        sw += coh
    if sw <= 0:
        return 0.0, info
    strength = math.hypot(sc, ss) / sw
    info["strength"] = strength
    if strength < min_strength:
        return 0.0, info
    yaw = math.atan2(ss, sc) / 4.0
    if yaw <= -math.pi / 4 + 1e-12:
        yaw = math.pi / 4
    info["source"] = "walls"
    return yaw, info


def read_map_yaml(path):
    """The few nav2 map_server yaml keys we need (no yaml dependency)."""
    kv = {}
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if ":" in line:
                k, v = line.split(":", 1)
                kv[k.strip()] = v.strip()
    try:
        res = float(kv["resolution"])
        o = [float(t) for t in kv["origin"].strip("[]").split(",")]
        if res <= 0 or len(o) < 2:
            raise ValueError
    except (KeyError, ValueError) as e:
        raise GridError(f"{path}: need resolution and origin [x, y, yaw]") from e
    return {"image": kv.get("image", "").strip("'\""), "resolution": res,
            "origin": o, "negate": int(float(kv.get("negate", 0))),
            "occupied_thresh": float(kv.get("occupied_thresh", 0.65)),
            "free_thresh": float(kv.get("free_thresh", 0.25))}


def read_pgm(path):
    """Binary PGM (P5, maxval <= 255) -> (width, height, bytes) top row first."""
    with open(path, "rb") as f:
        raw = f.read()
    toks, pos = [], 0
    while len(toks) < 4:
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":
            while pos < len(raw) and raw[pos:pos + 1] != b"\n":
                pos += 1
            continue
        st = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        if st == pos:
            raise GridError(f"{path}: truncated pgm header")
        toks.append(raw[st:pos])
    if toks[0] != b"P5":
        raise GridError(f"{path}: not a binary PGM (P5)")
    w, h, mx = (int(t) for t in toks[1:])
    if mx > 255 or w <= 0 or h <= 0 or w * h > 64_000_000:
        raise GridError(f"{path}: unsupported pgm size/maxval")
    body = raw[pos + 1:pos + 1 + w * h]
    if len(body) != w * h:
        raise GridError(f"{path}: pgm pixel data truncated")
    return w, h, body


def load_saved_map(yaml_path):
    """Saved nav2 map (yaml + pgm) -> occupancy dict (row 0 = bottom). Pixels
    are classified like map_server (negate / occupied_thresh) except that the
    ~205 'unknown' gray save_map writes is kept UNKNOWN (nav2's free_thresh
    would read it as free) and so does not count toward the extent."""
    meta = read_map_yaml(yaml_path)
    img = meta["image"]
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(yaml_path), img)
    w, h, px = read_pgm(img)
    lut = []
    for v in range(256):
        vn = 255 - v if meta["negate"] else v
        p = (255 - vn) / 255.0
        if p > meta["occupied_thresh"]:
            lut.append(100)
        elif 199 <= vn <= 211:
            lut.append(-1)
        elif p < meta["free_thresh"]:
            lut.append(0)
        else:
            lut.append(-1)
    rows = [bytes(px[r * w:(r + 1) * w]) for r in range(h)]
    data = []
    for r in reversed(rows):
        data.extend(lut[b] for b in r)
    return {"width": w, "height": h, "resolution": meta["resolution"],
            "origin_x": meta["origin"][0], "origin_y": meta["origin"][1], "data": data}


def auto_from_occupancy(occ=None, extent=None, existing=None,
                        cell_ft=AUTO_CELL_FT, align="walls",
                        regenerate=False, force=False):
    """Create or extend the auto grid. -> (grid, action, info).
    action: created | extended | unchanged | kept_structural | empty |
    regenerated. A structural grid is returned untouched. Yaw is estimated ONLY
    at creation (align 'walls' needs occ; without it, or align 'map', yaw is 0).
    regenerate=True builds a NEW auto grid at cell_ft, replacing an auto grid
    (info has old_cell_m/new_cell_m); a structural grid needs force=True."""
    if regenerate:
        if existing is not None and existing.get("mode") == "structural" and not force:
            raise GridError("structural grid not regenerated (it comes from drawings); "
                            "pass force=true to replace it")
        if align not in ("walls", "map"):
            raise GridError("align must be 'walls' or 'map'")
        if extent is None and occ is not None:
            extent = extent_from_occupancy(occ)
        if extent is None:
            return existing, "empty", {}
        yaw, info = 0.0, {"source": "map"}
        if align == "walls" and occ is not None:
            yaw, info = estimate_yaw(occ)
        new = auto_new(extent, yaw, cell_ft)
        info = dict(info)
        info["old_cell_m"] = existing.get("cell_m") if existing else None
        info["new_cell_m"] = new["cell_m"]
        return new, "regenerated", info
    if existing is not None and existing.get("mode") == "structural":
        return existing, "kept_structural", {}
    if align not in ("walls", "map"):
        raise GridError("align must be 'walls' or 'map'")
    if extent is None and occ is not None:
        extent = extent_from_occupancy(occ)
    if extent is None:
        return existing, "empty", {}
    if existing is not None:
        new = extend_to(existing, extent)
        return new, ("unchanged" if new == existing else "extended"), {}
    yaw, info = 0.0, {"source": "map"}
    if align == "walls" and occ is not None:
        yaw, info = estimate_yaw(occ)
    return auto_new(extent, yaw, cell_ft), "created", info


def apply_auto_files(yaml_path, grid_file, cell_ft=AUTO_CELL_FT, align="walls",
                     dry_run=False, refit=False):
    """Offline: auto grid for one saved map. Never touches a map that already
    has a grid unless refit=True, and then only an auto one (extend)."""
    name = os.path.basename(yaml_path)[:-len(".yaml")]
    res = {"map": name, "grid_file": os.path.basename(grid_file)}
    existing = None
    if os.path.exists(grid_file):
        try:
            with open(grid_file) as f:
                existing = validate(json.load(f))
        except (OSError, ValueError):
            return res | {"action": "skipped_unreadable_grid"}
        if not refit or existing["mode"] != "auto":
            return res | {"action": "skipped_has_grid", "mode": existing["mode"]}
    try:
        occ = load_saved_map(yaml_path)
        grid, action, info = auto_from_occupancy(occ, None, existing, cell_ft, align)
    except (OSError, ValueError) as e:
        return res | {"action": "error", "error": str(e)}
    res |= {"action": action, "mode": "auto"}
    if info:
        res |= {"yaw_deg": round(math.degrees(grid["origin"]["yaw_rad"]), 2),
                "yaw_source": info.get("source"),
                "wall_strength": round(info.get("strength", 0.0), 3)}
    if grid is not None and action in ("created", "extended"):
        res["lines"] = [len(grid["numbers"]["labels"]), len(grid["letters"]["labels"])]
        if not dry_run:
            tmp = grid_file + ".tmp"
            with open(tmp, "w") as f:
                json.dump(grid, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, grid_file)
    return res


# END AUTO-GRID BLOCK

# --- storage -----------------------------------------------------------------

def valid_map_name(name):
    return isinstance(name, str) and bool(_MAP_NAME_RE.match(name))


def grid_path(maps_dir, map_name):
    name = map_name if valid_map_name(map_name) else SESSION
    return os.path.join(maps_dir, f"{name}.grid.json")


def load(path):
    """Validated grid or None (missing/corrupt files read as no grid)."""
    try:
        with open(path) as f:
            return validate(json.load(f))
    except (OSError, ValueError):
        return None


def save(path, grid):
    g = validate(grid)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(g, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    return g
