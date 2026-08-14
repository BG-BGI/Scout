"""AprilTag detection + sqlite tag registry.

Registry lives at /maps/tags.db (the ./maps bind mount — same persistence
story as waypoints.json): one row per physical tag, keyed (family, tag_id),
with a human name ("doghouse"), a role ("home"), the printed black-square
edge length, and the last place it was seen in the map frame.

Detection is dt-apriltags (bundled C lib, aarch64 wheel — pupil-apriltags
ships none). Pose trick: the detector takes ONE tag_size for a frame, but
pose_t is linear in tag size, so we detect with tag_size=1.0 and scale each
detection by its registered size afterwards.

Tag-frame convention safety: rather than trusting which way the library
points the tag's +z, the face normal is taken as whichever of ±(R @ z) points
back toward the camera (negative z in the optical frame). The standoff point
sits STANDOFF_M in front of the face along that normal.
"""

import math
import os
import sqlite3
import time

import numpy as np

DB_PATH = os.environ.get("TAGS_DB", "/maps/tags.db")
DEFAULT_FAMILY = "tag36h11"
STANDOFF_M = 0.5

# ⚠ dt-apriltags traps, both measured 2026-08-14:
#  - A multi-family Detector ("tag36h11 tagStandard52h13") SILENTLY detects
#    nothing — frames that decode fine single-family return zero.
#  - Destroying Detector instances corrupts malloc ("mismatching
#    next->prev_size"). So: one persistent Detector per family, created once,
#    NEVER dropped, and detection loops families sequentially.
_detectors: dict[str, object] = {}


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
    return db


def all_tags() -> list[dict]:
    with _connect() as db:
        return [dict(r) for r in db.execute("SELECT * FROM tags ORDER BY name")]


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


def record_sighting(family: str, tag_id: int, map_pose: tuple | None) -> None:
    stamp = time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime())
    with _connect() as db:
        if map_pose is not None:
            db.execute(
                """UPDATE tags SET last_seen=?, map_x=?, map_y=?, map_yaw=?
                   WHERE family=? AND tag_id=?""",
                (stamp, *map_pose, family, tag_id),
            )
        else:
            db.execute(
                "UPDATE tags SET last_seen=? WHERE family=? AND tag_id=?",
                (stamp, family, tag_id),
            )


# --- detection ------------------------------------------------------------------

def _get_detector(family: str):
    if family not in _detectors:
        from dt_apriltags import Detector  # deferred: loads the C lib

        _detectors[family] = Detector(families=family, nthreads=2)
    return _detectors[family]


def registered_families() -> frozenset:
    """Families of registered tags; the default only while the DB is empty
    (so scanning does something before the first registration)."""
    fams = {t["family"] for t in all_tags()}
    return frozenset(fams or {DEFAULT_FAMILY})


def detect(gray: np.ndarray, camera_params: tuple | None) -> list[dict]:
    """Detections with unit-size pose (scale pose_t by the real tag size).
    camera_params = (fx, fy, cx, cy) or None for detection without pose."""
    raw = []
    for family in sorted(registered_families()):
        raw.extend(
            _get_detector(family).detect(
                gray,
                estimate_tag_pose=camera_params is not None,
                camera_params=camera_params,
                tag_size=1.0,
            )
        )
    out = []
    for r in raw:
        fam = r.tag_family.decode() if isinstance(r.tag_family, bytes) else str(r.tag_family)
        d = {
            "family": fam,
            "tag_id": int(r.tag_id),
            "center_px": [round(float(c), 1) for c in r.center],
            "corners_px": [[float(x), float(y)] for x, y in r.corners],
            "decision_margin": round(float(r.decision_margin), 1),
        }
        if camera_params is not None and r.pose_t is not None:
            d["pose_t_unit"] = np.asarray(r.pose_t).flatten()
            d["pose_R"] = np.asarray(r.pose_R)
        out.append(d)
    return out


def map_geometry(det: dict, size_m: float, tree, cam_frame: str) -> dict:
    """distance + map-frame tag position and standoff pose for one detection.
    Returns {} when pose or TF is unavailable."""
    if "pose_t_unit" not in det:
        return {}
    t_cam = det["pose_t_unit"] * size_m
    out = {"distance_m": round(float(np.linalg.norm(t_cam)), 2)}
    normal = det["pose_R"] @ np.array([0.0, 0.0, 1.0])
    # Face normal points back toward the camera (−z in the optical frame),
    # whichever way the library's tag frame is handed.
    if normal[2] > 0:
        normal = -normal
    standoff_cam = t_cam + normal * STANDOFF_M
    tag_map = tree.to_ancestor(t_cam, cam_frame, "map")
    standoff_map = tree.to_ancestor(standoff_cam, cam_frame, "map")
    if tag_map is None or standoff_map is None:
        return out
    yaw = math.atan2(tag_map[1] - standoff_map[1], tag_map[0] - standoff_map[0])
    out["position_map"] = [round(float(c), 3) for c in tag_map[:2]]
    out["standoff"] = {
        "x": round(float(standoff_map[0]), 3),
        "y": round(float(standoff_map[1]), 3),
        "yaw": round(yaw, 3),
    }
    return out
