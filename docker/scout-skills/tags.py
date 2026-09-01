"""AprilTag registry + map geometry. Detection itself runs in apriltag_ros
(official wrapper, `apriltag` node in robot.launch.py) which publishes
/detections and a TF frame per tag — this module only owns MEANING: the
sqlite registry (name/role/size, "doghouse" = home) and the standoff math
that turns a tag's TF frame into a waypoint.

Registry lives at /maps/tags.db (the ./maps bind mount — same persistence
story as waypoints.json). Detection coverage (family, sizes) is configured
in scout/config/apriltag.yaml and needs a robot-service restart to change;
registering a tag here is instant but only names what the node can already
see.
"""

import json
import math
import os
import sqlite3
import time

import numpy as np

DB_PATH = os.environ.get("TAGS_DB", "/maps/tags.db")
SITE_JSON = os.environ.get("SITE_JSON", "/sites/active/site.json")
STANDOFF_M = 0.5


def active_map_name() -> str | None:
    """Active map of the active site, or None. Read per call (same live-switch
    story as the tags.db reopen): tolerates site.json v1 (default_map) and v2
    (active_map, ADR-0029), and any read error."""
    try:
        with open(SITE_JSON) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return None
        return data.get("active_map") or data.get("default_map") or None
    except (OSError, json.JSONDecodeError):
        return None


def norm_family(fam: str) -> str:
    """'tagStandard52h13' / 'Standard52h13' / '36h11' → comparable form."""
    f = fam.lower()
    return f[3:] if f.startswith("tag") else f


# --- registry -----------------------------------------------------------------

def _connect() -> sqlite3.Connection:
    db = sqlite3.connect(DB_PATH)
    db.row_factory = sqlite3.Row
    db.execute(
        """CREATE TABLE IF NOT EXISTS tags(
             family  TEXT    NOT NULL,
             tag_id  INTEGER NOT NULL,
             name    TEXT    NOT NULL UNIQUE,
             role    TEXT    NOT NULL DEFAULT '',
             size_m  REAL    NOT NULL DEFAULT 0.16,
             map_x   REAL, map_y REAL, map_yaw REAL,
             last_seen TEXT,
             PRIMARY KEY(family, tag_id))"""
    )
    # v2 migration (ADR-0029): the map a survey was made on. NULL = legacy row
    # = assume the active map. One surveyed pose per tag ID (the PK), so each
    # floor's transit tag must be a distinct physical tag.
    cols = {r["name"] for r in db.execute("PRAGMA table_info(tags)")}
    if "map_name" not in cols:
        db.execute("ALTER TABLE tags ADD COLUMN map_name TEXT")
    return db


def all_tags() -> list[dict]:
    with _connect() as db:
        return [dict(r) for r in db.execute("SELECT * FROM tags ORDER BY name")]


def lookup(family: str, tag_id: int) -> dict | None:
    nf = norm_family(family)
    for t in all_tags():
        if t["tag_id"] == tag_id and norm_family(t["family"]) == nf:
            return t
    return None


def upsert(name: str, tag_id: int, family: str, role: str, size_m: float) -> dict:
    with _connect() as db:
        db.execute(
            """INSERT INTO tags(family, tag_id, name, role, size_m)
               VALUES(?,?,?,?,?)
               ON CONFLICT(family, tag_id) DO UPDATE
                 SET name=excluded.name, role=excluded.role,
                     size_m=excluded.size_m""",
            (family, tag_id, name, role, size_m),
        )
        return dict(
            db.execute("SELECT * FROM tags WHERE name=?", (name,)).fetchone()
        )


def delete(name: str) -> bool:
    with _connect() as db:
        return db.execute("DELETE FROM tags WHERE name=?", (name,)).rowcount > 0


def record_sighting(
    family: str, tag_id: int, map_pose: tuple | None, map_name: str | None = None
) -> None:
    stamp = time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime())
    with _connect() as db:
        if map_pose is not None:
            # map_name is stamped only alongside a surveyed pose — a pose-less
            # glimpse from the wrong floor must not re-home the tag.
            db.execute(
                """UPDATE tags SET last_seen=?, map_x=?, map_y=?, map_yaw=?,
                       map_name=?
                   WHERE family=? AND tag_id=?""",
                (stamp, *map_pose, map_name, family, tag_id),
            )
        else:
            db.execute(
                "UPDATE tags SET last_seen=? WHERE family=? AND tag_id=?",
                (stamp, family, tag_id),
            )


# --- geometry -----------------------------------------------------------------

def map_geometry(tree, tag_frame: str, robot_xy: tuple | None) -> dict:
    """Tag's map position + a floor-level standoff pose STANDOFF_M in front
    of its face, from the TF frame apriltag_ros publishes. The face normal is
    the tag frame's z-axis, disambiguated toward the robot (conventions
    differ on which way z points; the robot is by definition on the visible
    side). Returns {} when the TF chain is incomplete."""
    origin = tree.to_ancestor(np.zeros(3), tag_frame, "map")
    z_tip = tree.to_ancestor(np.array([0.0, 0.0, 1.0]), tag_frame, "map")
    if origin is None or z_tip is None:
        return {}
    normal = np.asarray(z_tip[:2]) - np.asarray(origin[:2])  # floor projection
    n = np.linalg.norm(normal)
    out = {"position_map": [round(float(c), 3) for c in origin[:2]]}
    if n < 0.2:
        return out  # tag lying flat — no meaningful approach direction
    normal /= n
    if robot_xy is not None:
        to_robot = np.array([robot_xy[0] - origin[0], robot_xy[1] - origin[1]])
        if float(normal @ to_robot) < 0:
            normal = -normal
    standoff = np.asarray(origin[:2]) + normal * STANDOFF_M
    yaw = math.atan2(origin[1] - standoff[1], origin[0] - standoff[0])
    out["standoff"] = {
        "x": round(float(standoff[0]), 3),
        "y": round(float(standoff[1]), 3),
        "yaw": round(yaw, 3),
    }
    return out
