"""POI registry — operator-marked map points with NO physical marker.

Third tag family beside AprilTags (tags.py, fiducials) and RFID/NFC/UHF reads
(companion recorders). A POI is just a named, categorized map-frame pose
stamped with the map it was marked on; nothing detects it.

Stdlib only. Two byte-identical copies exist (docker/scout-skills/poi.py and
docker/fleet-status/poi.py — separate images, same pattern as tags/tagpose);
scout/test/test_poi.py asserts they match. Edit both.

Store: <site>/maps/pois.db (sqlite). Opened per call so a site switch applies
live. Schema is created on demand.
"""

import json
import math
import os
import sqlite3
import time

DB_PATH = os.environ.get("POIS_DB", "/maps/pois.db")
SITE_JSON = os.environ.get("SITE_JSON", "/sites/active/site.json")
MAX_COORD_M = 1000.0
MAX_NAME = 64
MAX_NOTES = 1000


class PoiError(ValueError):
    pass


def active_map_name(site_json: str | None = None) -> str | None:
    try:
        with open(site_json or SITE_JSON) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return None
        return data.get("active_map") or data.get("default_map") or None
    except (OSError, json.JSONDecodeError):
        return None


def _connect(db_path: str | None = None) -> sqlite3.Connection:
    db = sqlite3.connect(db_path or DB_PATH, timeout=3)
    db.row_factory = sqlite3.Row
    db.execute(
        """CREATE TABLE IF NOT EXISTS pois(
             name     TEXT NOT NULL PRIMARY KEY,
             category TEXT NOT NULL DEFAULT '',
             notes    TEXT NOT NULL DEFAULT '',
             map_x    REAL NOT NULL,
             map_y    REAL NOT NULL,
             map_yaw  REAL NOT NULL DEFAULT 0,
             map_name TEXT,
             created  TEXT NOT NULL,
             updated  TEXT NOT NULL)"""
    )
    return db


def _now() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime())


def _num(v, key: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise PoiError(f"{key} must be a number")
    v = float(v)
    if not math.isfinite(v):
        raise PoiError(f"{key} must be finite")
    return v


def validate(name, x, y, yaw=0.0, category="", notes="") -> dict:
    if not isinstance(name, str) or not name.strip():
        raise PoiError("name required")
    name = name.strip()
    if len(name) > MAX_NAME:
        raise PoiError(f"name > {MAX_NAME} chars")
    x, y, yaw = _num(x, "x"), _num(y, "y"), _num(yaw, "yaw")
    if abs(x) >= MAX_COORD_M or abs(y) >= MAX_COORD_M:
        raise PoiError(f"|x|,|y| must be < {MAX_COORD_M:.0f} m")
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    if not isinstance(category, str) or not isinstance(notes, str):
        raise PoiError("category/notes must be strings")
    if len(notes) > MAX_NOTES:
        raise PoiError(f"notes > {MAX_NOTES} chars")
    return {"name": name, "x": round(x, 3), "y": round(y, 3),
            "yaw": round(yaw, 3), "category": category.strip(),
            "notes": notes.strip()}


def all_pois(db_path: str | None = None) -> list[dict]:
    """Every POI, all maps (rows carry map_name; filter client-side)."""
    with _connect(db_path) as db:
        return [dict(r) for r in db.execute("SELECT * FROM pois ORDER BY name")]


def get(name: str, db_path: str | None = None) -> dict | None:
    with _connect(db_path) as db:
        r = db.execute("SELECT * FROM pois WHERE name=?", (name,)).fetchone()
        return dict(r) if r else None


def upsert(name, x, y, yaw=0.0, category="", notes="", map_name=None,
           db_path: str | None = None, site_json: str | None = None) -> dict:
    """Create or move/replace a POI. map_name defaults to the active map."""
    v = validate(name, x, y, yaw, category, notes)
    mn = map_name if map_name is not None else active_map_name(site_json)
    now = _now()
    with _connect(db_path) as db:
        db.execute(
            """INSERT INTO pois(name,category,notes,map_x,map_y,map_yaw,
                                map_name,created,updated)
               VALUES(?,?,?,?,?,?,?,?,?)
               ON CONFLICT(name) DO UPDATE SET
                 category=excluded.category, notes=excluded.notes,
                 map_x=excluded.map_x, map_y=excluded.map_y,
                 map_yaw=excluded.map_yaw, map_name=excluded.map_name,
                 updated=excluded.updated""",
            (v["name"], v["category"], v["notes"], v["x"], v["y"], v["yaw"],
             mn, now, now),
        )
        return dict(db.execute("SELECT * FROM pois WHERE name=?",
                               (v["name"],)).fetchone())


def update_fields(name: str, fields: dict, db_path: str | None = None,
                  site_json: str | None = None) -> dict | None:
    """Partial edit (category/notes/x/y/yaw/rename via `new_name`). None = no
    such POI. Moving (x/y/yaw) re-stamps map_name to the active map."""
    cur = get(name, db_path)
    if cur is None:
        return None
    moved = any(k in fields for k in ("x", "y", "yaw"))
    v = validate(
        fields.get("new_name", name),
        fields.get("x", cur["map_x"]),
        fields.get("y", cur["map_y"]),
        fields.get("yaw", cur["map_yaw"]),
        fields.get("category", cur["category"]),
        fields.get("notes", cur["notes"]),
    )
    mn = (active_map_name(site_json) or cur["map_name"]) if moved else cur["map_name"]
    with _connect(db_path) as db:
        if v["name"] != name and db.execute(
                "SELECT 1 FROM pois WHERE name=?", (v["name"],)).fetchone():
            raise PoiError(f"a POI named {v['name']!r} already exists")
        db.execute(
            """UPDATE pois SET name=?, category=?, notes=?, map_x=?, map_y=?,
                   map_yaw=?, map_name=?, updated=? WHERE name=?""",
            (v["name"], v["category"], v["notes"], v["x"], v["y"], v["yaw"],
             mn, _now(), name),
        )
        return dict(db.execute("SELECT * FROM pois WHERE name=?",
                               (v["name"],)).fetchone())


def delete(name: str, db_path: str | None = None) -> bool:
    with _connect(db_path) as db:
        return db.execute("DELETE FROM pois WHERE name=?", (name,)).rowcount > 0
