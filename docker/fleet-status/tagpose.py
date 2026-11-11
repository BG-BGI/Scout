"""Operator-set AprilTag survey pose (webui map "Set tag position").

Stdlib only, no docker imports, so it unit-tests standalone. Writes the same
two stores scout-skills owns (tags.db row + waypoints.json standoff entry),
with the same schemas/formulas as docker/scout-skills/tags.py and server.py
(_scan_tags): map_yaw = heading of the standoff pose that FACES the tag; the
standoff waypoint sits STANDOFF_M behind the tag along that heading.
"""
import json
import logging
import math
import os
import sqlite3
import time

log = logging.getLogger("tagpose")

STANDOFF_M = 0.5  # keep equal to scout-skills tags.STANDOFF_M
MAX_COORD_M = 1000.0
WAYPOINTS_VERSION = 2


class PoseError(ValueError):
    pass


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def _num(body: dict, key: str) -> float:
    v = body.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise PoseError(f"{key} must be a number")
    v = float(v)
    if not math.isfinite(v):
        raise PoseError(f"{key} must be finite")
    return v


def validate(body: dict) -> tuple[float, float, float]:
    if not isinstance(body, dict):
        raise PoseError("body must be a JSON object")
    x, y, yaw = _num(body, "x"), _num(body, "y"), _num(body, "yaw")
    if abs(x) >= MAX_COORD_M or abs(y) >= MAX_COORD_M:
        raise PoseError(f"|x|,|y| must be < {MAX_COORD_M:.0f} m")
    if abs(yaw) > math.pi + 1e-6:
        raise PoseError("yaw must be in [-pi, pi] radians")
    return x, y, normalize_yaw(yaw)


def standoff(x: float, y: float, yaw: float) -> dict:
    return {
        "x": round(x - STANDOFF_M * math.cos(yaw), 3),
        "y": round(y - STANDOFF_M * math.sin(yaw), 3),
        "yaw": round(yaw, 3),
    }


def active_map_name(site_json: str) -> str | None:
    try:
        with open(site_json) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return None
        return data.get("active_map") or data.get("default_map") or None
    except (OSError, json.JSONDecodeError):
        return None


def _store_waypoints(path: str, store: dict) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(store, f, indent=2, sort_keys=True)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _load_waypoints(path: str) -> dict:
    try:
        with open(path) as f:
            data = json.load(f)
    except FileNotFoundError:
        data = {}
    if data.get("version") == WAYPOINTS_VERSION:
        data.setdefault("waypoints", {})
        data.setdefault("routes", {})
        return data
    return {
        "version": WAYPOINTS_VERSION,
        "waypoints": {k: v for k, v in data.items()
                      if isinstance(v, dict) and "x" in v},
        "routes": {},
    }


def set_pose(name: str, body: dict, db_path: str, waypoints_path: str,
             site_json: str) -> tuple[int, dict]:
    """-> (http_code, payload)."""
    try:
        x, y, yaw = validate(body)
    except PoseError as e:
        return 400, {"error": str(e)}
    if not os.path.exists(db_path):
        return 404, {"error": "no tags.db for the active site"}
    active = active_map_name(site_json)
    try:
        db = sqlite3.connect(db_path, timeout=3)
        db.row_factory = sqlite3.Row
        try:
            cols = {r["name"] for r in db.execute("PRAGMA table_info(tags)")}
            row = db.execute("SELECT * FROM tags WHERE name=?", (name,)).fetchone()
            if row is None:
                return 404, {"error": f"no tag named {name!r}"}
            old = dict(row)
            sets, args = ["map_x=?", "map_y=?", "map_yaw=?"], [x, y, yaw]
            if "map_name" in cols:
                sets.append("map_name=?")
                args.append(active)
            db.execute(f"UPDATE tags SET {', '.join(sets)} WHERE name=?",
                       (*args, name))
            db.commit()
            new = dict(db.execute("SELECT * FROM tags WHERE name=?",
                                  (name,)).fetchone())
        finally:
            db.close()
    except sqlite3.Error as e:
        return 500, {"error": f"tags.db: {e}"}

    wp = standoff(x, y, yaw) | {
        "saved": time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime()),
        "source": "manual",
    }
    if active:
        wp["map"] = active
    try:
        store = _load_waypoints(waypoints_path)
        store["waypoints"][name] = wp
        _store_waypoints(waypoints_path, store)
    except (OSError, json.JSONDecodeError) as e:
        return 500, {"error": f"tag pose saved but waypoints.json failed: {e}",
                     "tag": new}
    log.info("tag %s pose %s -> (%.3f, %.3f, %.3f) map=%s", name,
             (old.get("map_x"), old.get("map_y"), old.get("map_yaw")),
             x, y, yaw, active)
    return 200, {"ok": True, "tag": new, "waypoint": wp}
