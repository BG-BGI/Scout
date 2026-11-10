"""AprilTag registry + map geometry. Detection itself runs in apriltag_ros
(official wrapper, `apriltag` node in robot.launch.py) which publishes
/detections and a TF frame per tag — this module only owns MEANING: the
sqlite registry (name/role/size, "doghouse" = home) and the standoff math
that turns a tag's TF frame into a waypoint.

Registry lives at /maps/tags.db (the ./maps bind mount — same persistence
story as waypoints.json). Detection coverage (family, sizes) is configured
in companion/config/apriltag.yaml (companion apriltag service, ADR-0034)
and needs that service restarted to change;
registering a tag here is instant but only names what the node can already
see.
"""

import json
import math
import os
import sqlite3
import time
from contextlib import contextmanager

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
        active = data.get("active_map") or data.get("default_map") or None
        return active if isinstance(active, str) else None
    except (OSError, json.JSONDecodeError):
        return None


def norm_family(fam: str) -> str:
    """'tagStandard52h13' / 'Standard52h13' / '36h11' → comparable form."""
    f = fam.lower()
    return f[3:] if f.startswith("tag") else f


# --- registry -----------------------------------------------------------------
#
# Identity is per PHYSICAL tag (`tags`: name UNIQUE, role, size_m, last_seen);
# the surveyed pose is per (family, tag_id, map_name) in `tag_poses`, because a
# floor can hold a scan map AND a drawing map and each needs its own survey.
# A tag_poses.map_name of '' (LEGACY) is a pre-U11 row whose map was never
# stamped: it meant "the active map" and still does, but only while the tag has
# no named pose anywhere (see active_pose).
#
# The legacy pose columns on `tags` (map_x, map_y, map_yaw, map_name) stay in
# place after the migration and are never read or written again: an older image
# rolled back onto this file still finds them.
#
# ⚠ SCHEMA_VERSION / SCHEMA_STATEMENTS / active_pose are DUPLICATED in
# docker/fleet-status/tagpose.py (separate images share a schema, not code,
# ADR-0011) and mirrored in scout/core/tag_homes.py; scout/test/test_tags_db.py
# fails when the copies drift.

SCHEMA_VERSION = 3
LEGACY = ""
BUSY_S = 10.0   # sqlite busy wait: lets the loser of the migration race wait it out

SCHEMA_STATEMENTS = (
    """CREATE TABLE IF NOT EXISTS tags(
         family  TEXT    NOT NULL,
         tag_id  INTEGER NOT NULL,
         name    TEXT    NOT NULL UNIQUE,
         role    TEXT    NOT NULL DEFAULT '',
         size_m  REAL    NOT NULL DEFAULT 0.16,
         map_x   REAL, map_y REAL, map_yaw REAL,
         last_seen TEXT,
         map_name TEXT,
         PRIMARY KEY(family, tag_id))""",
    """CREATE TABLE IF NOT EXISTS tag_poses(
         family  TEXT    NOT NULL,
         tag_id  INTEGER NOT NULL,
         map_name TEXT   NOT NULL,
         map_x   REAL    NOT NULL,
         map_y   REAL    NOT NULL,
         map_yaw REAL    NOT NULL,
         surveyed_at TEXT,
         PRIMARY KEY(family, tag_id, map_name))""",
    """INSERT OR IGNORE INTO tag_poses(
         family, tag_id, map_name, map_x, map_y, map_yaw, surveyed_at)
       SELECT family, tag_id, COALESCE(map_name, ''), map_x, map_y, map_yaw, NULL
       FROM tags
       WHERE typeof(map_x) IN ('real', 'integer')
         AND typeof(map_y) IN ('real', 'integer')
         AND typeof(map_yaw) IN ('real', 'integer')""",
    """CREATE TRIGGER IF NOT EXISTS tags_delete_poses AFTER DELETE ON tags
       BEGIN
         DELETE FROM tag_poses WHERE family = old.family AND tag_id = old.tag_id;
       END""",
)


def migrate(db: sqlite3.Connection) -> None:
    """Bring tags.db to SCHEMA_VERSION. Idempotent and safe with several
    processes opening the file at once: one BEGIN IMMEDIATE transaction, the
    version re-checked inside the lock, so a loser of the race waits (BUSY_S)
    and then finds it already done. `db` must be in autocommit mode
    (isolation_level=None) so the explicit BEGIN is the only transaction.
    Legacy poses are COPIED (NULL map_name -> LEGACY), never moved."""
    if db.execute("PRAGMA user_version").fetchone()[0] >= SCHEMA_VERSION:
        return
    db.execute("BEGIN IMMEDIATE")
    try:
        if db.execute("PRAGMA user_version").fetchone()[0] < SCHEMA_VERSION:
            db.execute(SCHEMA_STATEMENTS[0])
            # pre-ADR-0029 files lack the column the fresh CREATE already has
            cols = {r[1] for r in db.execute("PRAGMA table_info(tags)")}
            if "map_name" not in cols:
                db.execute("ALTER TABLE tags ADD COLUMN map_name TEXT")
            for stmt in SCHEMA_STATEMENTS[1:]:
                db.execute(stmt)
            db.execute(f"PRAGMA user_version = {SCHEMA_VERSION}")
        db.execute("COMMIT")
    except BaseException:
        if db.in_transaction:
            db.execute("ROLLBACK")
        raise


def active_pose(poses: dict, active: str | None):
    """The pose of `poses` ({map_name: pose}) that applies to the active map:
    (map_name | None, pose), or None. A named row for the active map wins. A
    LEGACY row (map never recorded) applies to the active map only while the tag
    has no named pose at all — once it is surveyed by name anywhere, an
    unassigned pose is ambiguous and never stands in for another map's survey.
    No active map: only a LEGACY row, same rule."""
    if active is not None and active in poses:
        return active, poses[active]
    if LEGACY in poses and all(m == LEGACY for m in poses):
        return None, poses[LEGACY]
    return None


def _connect() -> sqlite3.Connection:
    """Open + migrate. Autocommit mode: callers use _read / _write."""
    db = sqlite3.connect(DB_PATH, timeout=BUSY_S, isolation_level=None)
    db.row_factory = sqlite3.Row
    try:
        migrate(db)
    except BaseException:
        db.close()
        raise
    return db


@contextmanager
def _read():
    db = _connect()
    try:
        yield db
    finally:
        db.close()


@contextmanager
def _write():
    """One BEGIN IMMEDIATE transaction: commit on success, roll back on error."""
    db = _connect()
    try:
        db.execute("BEGIN IMMEDIATE")
        yield db
        db.execute("COMMIT")
    except BaseException:
        if db.in_transaction:
            db.execute("ROLLBACK")
        raise
    finally:
        db.close()


def _poses_by_tag(db: sqlite3.Connection) -> dict:
    out: dict = {}
    for r in db.execute(
        "SELECT family, tag_id, map_name, map_x, map_y, map_yaw, surveyed_at"
        " FROM tag_poses"
    ):
        out.setdefault((r["family"], r["tag_id"]), {})[r["map_name"]] = {
            "x": r["map_x"], "y": r["map_y"], "yaw": r["map_yaw"],
            "surveyed_at": r["surveyed_at"],
        }
    return out


def _shape(row: sqlite3.Row, poses: dict, active: str | None) -> dict:
    """A tags row for callers: the legacy keys, with map_x/map_y/map_yaw/map_name
    now the pose for the ACTIVE map (None where unsurveyed there — never another
    map's pose), plus `poses` {map: {x, y, yaw, surveyed_at}} for the named
    maps. Bare LEGACY poses surface only as the active pose, map_name None."""
    d = dict(row)
    chosen = active_pose(poses, active)
    d["map_x"] = d["map_y"] = d["map_yaw"] = d["map_name"] = None
    if chosen is not None:
        name, pose = chosen
        d["map_x"], d["map_y"], d["map_yaw"] = pose["x"], pose["y"], pose["yaw"]
        d["map_name"] = name
    d["poses"] = {m: p for m, p in poses.items() if m != LEGACY}
    return d


def all_tags(active: str | None = None) -> list[dict]:
    """Every tag, active-map pose + `poses`. `active` defaults to the active map."""
    active = active if active is not None else active_map_name()
    with _read() as db:
        poses = _poses_by_tag(db)
        return [
            _shape(r, poses.get((r["family"], r["tag_id"]), {}), active)
            for r in db.execute("SELECT * FROM tags ORDER BY name")
        ]


def lookup(family: str, tag_id: int, active: str | None = None) -> dict | None:
    nf = norm_family(family)
    for t in all_tags(active):
        if t["tag_id"] == tag_id and norm_family(t["family"]) == nf:
            return t
    return None


def upsert(name: str, tag_id: int, family: str, role: str, size_m: float) -> dict:
    with _write() as db:
        db.execute(
            """INSERT INTO tags(family, tag_id, name, role, size_m)
               VALUES(?,?,?,?,?)
               ON CONFLICT(family, tag_id) DO UPDATE
                 SET name=excluded.name, role=excluded.role,
                     size_m=excluded.size_m""",
            (family, tag_id, name, role, size_m),
        )
    return next(t for t in all_tags() if t["name"] == name)


def clear_pose(
    name: str, map_name: str | None = None, all_maps: bool = False
) -> int | None:
    """Forget a tag's surveyed pose so the next survey starts afresh. Default:
    the pose that applies to the ACTIVE map (and only that); `map_name` names
    another map explicitly; `all_maps` forgets every pose. Returns how many
    poses were removed, or None when there is no such tag. Surveyed poses are
    otherwise immutable from sightings (the registry is the truth
    tag_relocalizer corrects amcl against)."""
    active = map_name if map_name is not None else active_map_name()
    with _write() as db:
        row = db.execute(
            "SELECT family, tag_id FROM tags WHERE name=?", (name,)
        ).fetchone()
        if row is None:
            return None
        key = (row["family"], row["tag_id"])
        if all_maps:
            targets = list(_poses_by_tag(db).get(key, {}))
        elif map_name is not None:
            targets = [map_name]
        else:
            chosen = active_pose(_poses_by_tag(db).get(key, {}), active)
            targets = [] if chosen is None else [
                chosen[0] if chosen[0] is not None else LEGACY]
        return sum(
            db.execute(
                "DELETE FROM tag_poses WHERE family=? AND tag_id=? AND map_name=?",
                (*key, m),
            ).rowcount
            for m in targets
        )


def delete(name: str) -> bool:
    """Remove the tag and ALL its poses (the trigger removes them even when an
    older image does the delete)."""
    with _write() as db:
        row = db.execute(
            "SELECT family, tag_id FROM tags WHERE name=?", (name,)
        ).fetchone()
        if row is None:
            return False
        db.execute("DELETE FROM tags WHERE name=?", (name,))
        db.execute(
            "DELETE FROM tag_poses WHERE family=? AND tag_id=?",
            (row["family"], row["tag_id"]),
        )
        return True


def record_sighting(
    family: str, tag_id: int, map_pose: tuple | None, map_name: str | None = None
) -> bool:
    """Stamp last_seen and, given a solved pose AND the map it was seen on,
    record that pose for THAT map only — never touching another map's survey,
    and never replacing an existing one (surveyed poses are immutable from
    sightings; re-survey = clear_pose first). No map_name means the map is
    ambiguous: the pose is dropped, not stored. Returns True when a pose was
    written."""
    stamp = time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime())
    nf = norm_family(family)
    with _write() as db:
        row = next(
            (r for r in db.execute(
                "SELECT family, tag_id FROM tags WHERE tag_id=?", (tag_id,))
             if norm_family(r["family"]) == nf),
            None,
        )
        if row is None:
            return False
        db.execute(
            "UPDATE tags SET last_seen=? WHERE family=? AND tag_id=?",
            (stamp, row["family"], row["tag_id"]),
        )
        if map_pose is None or not map_name:
            return False
        return db.execute(
            """INSERT OR IGNORE INTO tag_poses(
                 family, tag_id, map_name, map_x, map_y, map_yaw, surveyed_at)
               VALUES(?,?,?,?,?,?,?)""",
            (row["family"], row["tag_id"], map_name, *map_pose, stamp),
        ).rowcount > 0


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
