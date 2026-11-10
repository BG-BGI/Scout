"""Which map a sighted AprilTag's survey belongs to — the pure rule behind
tag_relocalizer's seeding and floor transit (U11). Stdlib only.

A physical tag is surveyed PER MAP (tags.db `tag_poses`: one row per family +
tag_id + map_name), because a floor can hold a scan map and a drawing map and
each needs its own pose. A row whose map was never recorded (LEGACY, '') is a
pre-U11 survey that meant "the active map"; it still does, but only while the
tag has no named pose anywhere.

choose_home picks what a sighting means:
  active   a pose applies to the active map -> relocalize there, never transit
  transit  no pose on the active map, exactly one other map to go to
  refused  surveyed only on maps that cannot be chosen between -> no transit
  none     no usable pose (or no active map to compare against)

`active_pose` is duplicated in docker/scout-skills/tags.py and
docker/fleet-status/tagpose.py (separate images); test_tags_db.py pins the
copies together.
"""

import math

from scout.core import sites

LEGACY = ''


def active_pose(poses, active):
    """Pose of `poses` ({map_name: pose}) that applies to the active map:
    (map_name | None, pose) or None. A named row for the active map wins; a
    LEGACY row applies only while the tag has no named pose at all — an
    unassigned pose never stands in for another map's survey."""
    if active is not None and active in poses:
        return active, poses[active]
    if LEGACY in poses and all(m == LEGACY for m in poses):
        return None, poses[LEGACY]
    return None


def choose_home(poses, active_map, maps):
    """-> (kind, home, pose, candidates) for one tag. `poses` {map: pose},
    `maps` the site's raw name -> entry mapping (for each map's source).
    `home` is the map the pose belongs to (None for a LEGACY row)."""
    chosen = active_pose(poses, active_map)
    if chosen is not None:
        return 'active', chosen[0], chosen[1], []
    if active_map is None:
        return 'none', None, None, []
    named = sorted(m for m in poses if m != LEGACY)
    if not named:
        return 'none', None, None, []
    source = sites.map_source(maps, active_map)
    same = [m for m in named if sites.map_source(maps, m) == source]
    if len(same) == 1:
        return 'transit', same[0], poses[same[0]], named
    if not same and len(named) == 1:
        # One candidate on the other source: hand it to the transit path, whose
        # cross-source check refuses with its own message.
        return 'transit', named[0], poses[named[0]], named
    return 'refused', None, None, named


def _usable(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def registry_poses(db, family, tag_id):
    """{map_name: {'x', 'y', 'yaw'}} for one registry tag, read through an open
    (typically read-only) connection. Uses tag_poses when the file is migrated,
    otherwise the legacy columns on `tags` (NULL map_name -> LEGACY): this
    reader never writes, so it cannot migrate. Rows with non-numeric or
    non-finite coordinates are skipped. Raises sqlite3.Error on a damaged file."""
    has_table = db.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name='tag_poses'"
    ).fetchone()
    if has_table:
        rows = db.execute(
            'SELECT map_name, map_x, map_y, map_yaw FROM tag_poses '
            'WHERE family=? AND tag_id=?', (family, tag_id)).fetchall()
        raw = [(r[0], r[1], r[2], r[3]) for r in rows]
    else:
        cols = {r[1] for r in db.execute('PRAGMA table_info(tags)')}
        name_col = 'map_name' if 'map_name' in cols else 'NULL'
        rows = db.execute(
            f'SELECT {name_col}, map_x, map_y, map_yaw FROM tags '
            'WHERE family=? AND tag_id=?', (family, tag_id)).fetchall()
        raw = [(r[0] or LEGACY, r[1], r[2], r[3]) for r in rows]
    return {
        name: {'x': x, 'y': y, 'yaw': yaw}
        for name, x, y, yaw in raw
        if isinstance(name, str) and _usable(x) and _usable(y) and _usable(yaw)
    }

