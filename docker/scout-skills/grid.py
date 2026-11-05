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
    return {"labels": out_l, "offsets_m": out_o}


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
    return {"version": VERSION, "units": units, "origin": origin,
            "numbers": nums, "letters": lets, "bay_anchor": "lower"}


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
