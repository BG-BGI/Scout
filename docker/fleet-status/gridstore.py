"""Per-map structural grid store for the webui (GET/POST/DELETE /api/grid).

Stdlib only so it unit-tests standalone. validate() and the
auto-grid block (labels, extend_to, wall-yaw estimator, saved-map reader) are
copies of docker/scout-skills/grid.py (separate images cannot share files);
scout/test/test_grid_rest.py + test_grid_auto.py assert the two agree.
Auto grids (mode "auto") are the standard grid sized to the map; structural
grids (mode "structural") come from the drawings and are never auto-touched.
Files live beside the map:
<maps>/<active_map>.grid.json, or _session.grid.json when no map is active.
"""
import base64
import json
import math
import os
import re

VERSION = 1
MAX_LINES = 200
SESSION = "_session"
FT_M = 0.3048
MAX_ABS = 1.0e6
MODES = ("structural", "auto")
MIN_CELL_M = 1.0
MAX_CELL_M = 1000.0
_LABEL_RE = re.compile(r"^[A-Za-z0-9.]{1,8}$")
_MAP_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


class GridError(ValueError):
    pass


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
            raise GridError(f"{name}: bad label {lb!r}")
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
    if not isinstance(grid, dict):
        raise GridError("grid must be an object")
    if grid.get("version", VERSION) != VERSION:
        raise GridError("unsupported grid version")
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
    if ({s.upper() for s in nums["labels"]} & {s.upper() for s in lets["labels"]}):
        raise GridError("labels shared by numbers and letters")
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


def to_m(value, units):
    return value * (FT_M if units == "ft" else 1.0)


def to_grid_frame(grid, x, y):
    o = grid["origin"]
    c, s = math.cos(o["yaw_rad"]), math.sin(o["yaw_rad"])
    dx, dy = x - o["x"], y - o["y"]
    return c * dx + s * dy, -s * dx + c * dy


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
                        cell_ft=AUTO_CELL_FT, align="walls"):
    """Create or extend the auto grid. -> (grid, action, info).
    action: created | extended | unchanged | kept_structural | empty.
    A structural grid is returned untouched. Yaw is estimated ONLY at creation
    (align 'walls' needs occ; without it, or align 'map', yaw is 0)."""
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

def active_map_name(site_json):
    try:
        with open(site_json) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return None
        return data.get("active_map") or data.get("default_map") or None
    except (OSError, json.JSONDecodeError):
        return None


def grid_path(maps_dir, site_json, name=None):
    name = name or active_map_name(site_json)
    if not (isinstance(name, str) and _MAP_NAME_RE.match(name)):
        name = SESSION
    return os.path.join(maps_dir, f"{name}.grid.json")


def get_grid(maps_dir, site_json):
    path = grid_path(maps_dir, site_json)
    try:
        with open(path) as f:
            g = validate(json.load(f))
            return 200, {"grid": g, "mode": g["mode"],
                         "map": os.path.basename(path)[:-len(".grid.json")]}
    except FileNotFoundError:
        return 200, {"grid": None}
    except (OSError, ValueError) as e:
        return 200, {"grid": None, "error": f"unreadable grid file: {e}"}


def put_grid(maps_dir, site_json, body):
    try:
        g = validate(body)
    except GridError as e:
        return 400, {"error": str(e)}
    path = grid_path(maps_dir, site_json)
    os.makedirs(maps_dir, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(g, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    return 200, {"grid": g}


def delete_grid(maps_dir, site_json):
    try:
        os.remove(grid_path(maps_dir, site_json))
    except FileNotFoundError:
        pass
    return 200, {"grid": None}


def adopt_session(maps_dir, name):
    """After Save Map under `name`: carry _session.grid.json over to
    <name>.grid.json (never clobbers an existing grid). Best-effort."""
    if not (isinstance(name, str) and _MAP_NAME_RE.match(name)) or name == SESSION:
        return 400, {"error": "bad map name"}
    src = os.path.join(maps_dir, f"{SESSION}.grid.json")
    dst = os.path.join(maps_dir, f"{name}.grid.json")
    if not os.path.exists(src):
        return 200, {"adopted": False, "reason": "no session grid"}
    if os.path.exists(dst):
        return 200, {"adopted": False, "reason": "map already has a grid"}
    os.replace(src, dst)
    return 200, {"adopted": True}


# --- auto-grid REST ----------------------------------------------------------

_MAX_CELLS = 16_000_000


def occ_from_body(o):
    """Occupancy summary posted by the web UI -> occ dict. data is a list of
    ints (-1..100) or data_b64 = base64 of int8 bytes."""
    if not isinstance(o, dict):
        raise GridError("occupancy must be an object")
    w, h = o.get("width"), o.get("height")
    if (isinstance(w, bool) or isinstance(h, bool)
            or not isinstance(w, int) or not isinstance(h, int)
            or w <= 0 or h <= 0 or w * h > _MAX_CELLS):
        raise GridError("occupancy width/height must be positive integers")
    res = _num(o.get("resolution"), "occupancy.resolution")
    if res <= 0:
        raise GridError("occupancy.resolution must be > 0")
    if "data_b64" in o:
        try:
            raw = base64.b64decode(o["data_b64"], validate=True)
        except (ValueError, TypeError) as e:
            raise GridError("occupancy.data_b64 is not base64") from e
        data = [v - 256 if v > 127 else v for v in raw]
    else:
        data = o.get("data")
        if not isinstance(data, list) or not all(
                isinstance(v, int) and not isinstance(v, bool) for v in data):
            raise GridError("occupancy.data must be a list of ints")
    if len(data) != w * h:
        raise GridError("occupancy.data length != width*height")
    org = o.get("origin") or {}
    return {"width": w, "height": h, "resolution": res, "data": data,
            "origin_x": _num(org.get("x", o.get("origin_x")), "occupancy.origin.x"),
            "origin_y": _num(org.get("y", o.get("origin_y")), "occupancy.origin.y")}


def _read_grid_file(path):
    with open(path) as f:
        return validate(json.load(f))


def _write_grid_file(path, g):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(g, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _cell_ft(body):
    v = body.get("cell_ft", AUTO_CELL_FT)
    return AUTO_CELL_FT if v is None else v


def auto_grid(maps_dir, site_json, body):
    """POST /api/grid/auto {cell_ft?, align?, extent?, occupancy?}: create the
    active map's auto grid, or extend it if mode auto already exists. Source of
    the map: posted occupancy > posted extent > the saved map's yaml/pgm.
    A structural grid is never touched (action kept_structural)."""
    body = body if isinstance(body, dict) else {}
    path = grid_path(maps_dir, site_json)
    name = os.path.basename(path)[:-len(".grid.json")]
    try:
        try:
            existing = _read_grid_file(path)
        except FileNotFoundError:
            existing = None
        except (OSError, ValueError) as e:
            return 409, {"error": f"existing grid file unreadable, not overwriting: {e}"}
        occ = extent = None
        if body.get("occupancy") is not None:
            occ = occ_from_body(body["occupancy"])
        if body.get("extent") is not None:
            extent = _norm_extent(body["extent"])
        if occ is None and extent is None and name != SESSION:
            yml = os.path.join(maps_dir, name + ".yaml")
            if os.path.exists(yml):
                occ = load_saved_map(yml)
        if occ is None and extent is None:
            return 200, {"grid": existing, "action": "empty",
                         "reason": "no extent, occupancy or saved map to size from"}
        # extent-only create with a saved map around: wall-align from the file
        if existing is None and occ is None and name != SESSION:
            yml = os.path.join(maps_dir, name + ".yaml")
            if os.path.exists(yml):
                occ = load_saved_map(yml)
        g, action, info = auto_from_occupancy(
            occ, extent, existing, _cell_ft(body),
            body.get("align", "walls"))
    except GridError as e:
        return 400, {"error": str(e)}
    except OSError as e:
        return 500, {"error": str(e)}
    if action in ("created", "extended"):
        _write_grid_file(path, g)
    out = {"grid": g, "action": action, "mode": g["mode"] if g else None, "map": name}
    if info:
        out["yaw_source"] = info.get("source")
        out["wall_strength"] = round(info.get("strength", 0.0), 3)
        out["occupied_cells"] = info.get("occupied")
    return 200, out


def auto_grid_all(sites_dir, body=None):
    """POST /api/grid/auto/all {cell_ft?, align?, dry_run?, refit?}: auto grid
    for every saved map (maps/<name>.yaml + pgm) of every site that has no grid
    yet. Offline; reads the saved files only."""
    body = body if isinstance(body, dict) else {}
    results = []
    try:
        sites = sorted(os.listdir(sites_dir))
    except OSError:
        sites = []
    for site in sites:
        maps = os.path.join(sites_dir, site, "maps")
        if not os.path.isdir(maps):
            continue
        for fn in sorted(os.listdir(maps)):
            nm = fn[:-len(".yaml")]
            if not fn.endswith(".yaml") or not _MAP_NAME_RE.match(nm) or nm == SESSION:
                continue
            r = apply_auto_files(
                os.path.join(maps, fn), os.path.join(maps, nm + ".grid.json"),
                _cell_ft(body), body.get("align", "walls"),
                bool(body.get("dry_run")), bool(body.get("refit")))
            results.append({"site": site} | r)
    return 200, {"dry_run": bool(body.get("dry_run")), "results": results}
