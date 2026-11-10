"""Operator-set AprilTag survey pose (webui map "Set tag position") and the
tag listing behind /api/apriltags.

Stdlib only, no docker imports, so it unit-tests standalone. Writes the same
two stores scout-skills owns (tags.db + waypoints.json standoff entry), with the
same schemas/formulas as docker/scout-skills/tags.py and server.py
(_scan_tags): map_yaw = heading of the standoff pose that FACES the tag; the
standoff waypoint sits STANDOFF_M behind the tag along that heading.

A tag's pose is per MAP (`tag_poses`, one row per family+tag_id+map_name): a
floor can hold a scan map and a drawing map, each surveyed separately. Setting a
pose writes the ACTIVE map's row only. ⚠ SCHEMA_VERSION / SCHEMA_STATEMENTS /
migrate / active_pose are DUPLICATED from docker/scout-skills/tags.py (separate
images share a schema, not code, ADR-0011); scout/test/test_tags_db.py fails
when the copies drift.
"""
import json
import logging
import math
import os
import sqlite3
import time

log = logging.getLogger("tagpose")

STANDOFF_M = 0.5  # keep equal to scout-skills tags.STANDOFF_M
BUSY_S = 3.0      # sqlite busy wait (the migration/write race with scout-skills)
MAX_COORD_M = 1000.0
WAYPOINTS_VERSION = 2


class PoseError(ValueError):
    pass


# --- tags.db schema: parity-tested copy of docker/scout-skills/tags.py ---------

SCHEMA_VERSION = 3
LEGACY = ""

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
    """Bring tags.db to SCHEMA_VERSION: idempotent, one BEGIN IMMEDIATE
    transaction with the version re-checked inside the lock (safe against
    scout-skills migrating at the same moment). `db` must be in autocommit mode
    (isolation_level=None). Legacy poses are copied, never moved."""
    if db.execute("PRAGMA user_version").fetchone()[0] >= SCHEMA_VERSION:
        return
    db.execute("BEGIN IMMEDIATE")
    try:
        if db.execute("PRAGMA user_version").fetchone()[0] < SCHEMA_VERSION:
            db.execute(SCHEMA_STATEMENTS[0])
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
    """Pose of `poses` ({map_name: pose}) that applies to the active map:
    (map_name | None, pose) or None. Named row for the active map wins; a LEGACY
    row (map never recorded) applies only while the tag has no named pose at
    all — never another map's survey. Same rule as scout-skills tags.active_pose."""
    if active is not None and active in poses:
        return active, poses[active]
    if LEGACY in poses and all(m == LEGACY for m in poses):
        return None, poses[LEGACY]
    return None


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _read_poses(db: sqlite3.Connection, rows: list) -> dict:
    """{(family, tag_id): {map_name: {x, y, yaw, surveyed_at}}}. From tag_poses
    when the file is migrated; otherwise (a read-only open cannot migrate) from
    the legacy columns, a NULL map_name becoming LEGACY — the same view the
    migration would produce."""
    out: dict = {}
    has_table = db.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name='tag_poses'"
    ).fetchone()
    if has_table:
        for r in db.execute(
            "SELECT family, tag_id, map_name, map_x, map_y, map_yaw, surveyed_at"
            " FROM tag_poses"
        ):
            out.setdefault((r["family"], r["tag_id"]), {})[r["map_name"]] = {
                "x": r["map_x"], "y": r["map_y"], "yaw": r["map_yaw"],
                "surveyed_at": r["surveyed_at"]}
        return out
    for r in rows:
        if all(_is_num(r.get(k)) for k in ("map_x", "map_y", "map_yaw")):
            out.setdefault((r["family"], r["tag_id"]), {})[
                r.get("map_name") or LEGACY] = {
                "x": r["map_x"], "y": r["map_y"], "yaw": r["map_yaw"],
                "surveyed_at": None}
    return out


def _shape(row: dict, poses: dict, active: str | None) -> dict:
    """Same shape as scout-skills tags._shape: legacy keys, with
    map_x/map_y/map_yaw/map_name the ACTIVE map's pose (None when unsurveyed
    there) plus `poses` {map: {x, y, yaw, surveyed_at}} for the named maps."""
    d = dict(row)
    chosen = active_pose(poses, active)
    d["map_x"] = d["map_y"] = d["map_yaw"] = d["map_name"] = None
    if chosen is not None:
        name, pose = chosen
        d["map_x"], d["map_y"], d["map_yaw"] = pose["x"], pose["y"], pose["yaw"]
        d["map_name"] = name
    d["poses"] = {m: p for m, p in poses.items() if m != LEGACY}
    return d


def read_tags(db: sqlite3.Connection, active: str | None) -> list[dict]:
    db.row_factory = sqlite3.Row
    rows = [dict(r) for r in db.execute("SELECT * FROM tags ORDER BY name")]
    poses = _read_poses(db, rows)
    return [_shape(r, poses.get((r["family"], r["tag_id"]), {}), active)
            for r in rows]


def list_tags(db_path: str, active: str | None) -> list[dict]:
    """Registry for /api/apriltags: read-only (a mid-write read cannot corrupt
    anything), works on migrated and un-migrated files. Raises sqlite3.Error."""
    db = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=BUSY_S)
    try:
        return read_tags(db, active)
    finally:
        db.close()


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
        active = data.get("active_map") or data.get("default_map") or None
        return active if isinstance(active, str) else None
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


def _survey(db_path: str, name: str, active: str, x: float, y: float,
            yaw: float):
    """Write the ACTIVE map's pose row for tag `name`, other maps untouched.
    -> (tag dict, previous active pose or None), or None when no such tag."""
    db = sqlite3.connect(db_path, timeout=BUSY_S, isolation_level=None)
    db.row_factory = sqlite3.Row
    try:
        migrate(db)
        db.execute("BEGIN IMMEDIATE")
        try:
            row = db.execute("SELECT * FROM tags WHERE name=?", (name,)).fetchone()
            if row is None:
                return None
            before = _read_poses(db, [dict(row)]).get(
                (row["family"], row["tag_id"]), {})
            db.execute(
                """INSERT INTO tag_poses(
                     family, tag_id, map_name, map_x, map_y, map_yaw, surveyed_at)
                   VALUES(?,?,?,?,?,?,?)
                   ON CONFLICT(family, tag_id, map_name) DO UPDATE SET
                     map_x=excluded.map_x, map_y=excluded.map_y,
                     map_yaw=excluded.map_yaw, surveyed_at=excluded.surveyed_at""",
                (row["family"], row["tag_id"], active, x, y, yaw,
                 time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime())))
            db.execute("COMMIT")
        finally:
            if db.in_transaction:
                db.execute("ROLLBACK")
        new = next(t for t in read_tags(db, active) if t["name"] == name)
        return new, active_pose(before, active)
    finally:
        db.close()


def set_pose(name: str, body: dict, db_path: str, waypoints_path: str,
             site_json: str) -> tuple[int, dict]:
    """-> (http_code, payload). Surveys `name` on the ACTIVE map only; the
    standoff waypoint named after the tag (waypoints.json is keyed by name and
    carries ONE map) is rewritten for that map, and the response says what it
    replaced — including which map the replaced waypoint belonged to."""
    try:
        x, y, yaw = validate(body)
    except PoseError as e:
        return 400, {"error": str(e)}
    if not os.path.exists(db_path):
        return 404, {"error": "no tags.db for the active site"}
    active = active_map_name(site_json)
    if active is None:
        return 409, {"error": "no active map — cannot tell which map this tag "
                              "pose belongs to; activate a map first"}
    try:
        result = _survey(db_path, name, active, x, y, yaw)
    except sqlite3.Error as e:
        return 500, {"error": f"tags.db: {e}"}
    if result is None:
        return 404, {"error": f"no tag named {name!r}"}
    new, old_pose = result

    wp = standoff(x, y, yaw) | {
        "saved": time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime()),
        "source": "manual",
        "map": active,
    }
    try:
        store = _load_waypoints(waypoints_path)
        prev = store["waypoints"].get(name)
        replaced = None if prev is None else {
            k: prev.get(k) for k in ("map", "x", "y", "yaw", "source", "saved")}
        store["waypoints"][name] = wp
        _store_waypoints(waypoints_path, store)
    except (OSError, json.JSONDecodeError) as e:
        return 500, {"error": f"tag pose saved but waypoints.json failed: {e}",
                     "tag": new}
    log.info("tag %s pose on map %s %s -> (%.3f, %.3f, %.3f)%s", name, active,
             None if old_pose is None else
             (old_pose[1]["x"], old_pose[1]["y"], old_pose[1]["yaw"]),
             x, y, yaw,
             "" if replaced is None else
             f" (replaced waypoint of map {replaced['map']!r})")
    return 200, {"ok": True, "tag": new, "waypoint": wp,
                 "replaced_waypoint": replaced}
