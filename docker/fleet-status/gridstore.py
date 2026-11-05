"""Per-map structural grid store for the webui (GET/POST/DELETE /api/grid).

Stdlib only so it unit-tests standalone. validate() is a hand copy of
docker/scout-skills/grid.py validate() (separate images cannot share files);
scout/test/test_grid_rest.py asserts the two agree. Files live beside the map:
<maps>/<active_map>.grid.json, or _session.grid.json when no map is active.
"""
import json
import math
import os
import re

VERSION = 1
MAX_LINES = 200
MAX_ABS = 1.0e6
SESSION = "_session"
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
    return {"labels": out_l, "offsets_m": out_o}


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
    return {"version": VERSION, "units": units, "origin": origin,
            "numbers": nums, "letters": lets, "bay_anchor": "lower"}


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
            return 200, {"grid": validate(json.load(f)),
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
