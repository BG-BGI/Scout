"""Pure lidar-measurement helpers for the scan_measure tool (no I/O, so they
unit-test off the robot). server.py does the rosbridge read; this module
projects scans into the map frame with the transform at each scan's own stamp,
segments the returns into clusters, matches clusters to target hints and
derives spacing + repeatability across scans."""

import math
import statistics

from geometry import planar_yaw

# Fallback base_link->laser (CLAUDE.md, measured 2026-07-30: translation
# (0.073, 0, 0.241), yaw +-pi — the scanner is mounted backwards). Used only
# when /tf_static was not replayed by rosbridge; the result is flagged.
LASER_FALLBACK = (0.073, 0.0, math.pi)

SEGMENT_GAP_M = 0.12  # Euclidean jump that splits two clusters
SEGMENT_INDEX_GAP = 3  # beams skipped (nulls) that split two clusters
MIN_SEGMENT_POINTS = 4
WIDE_CLUSTER_M = 0.8  # wider than any steel post: probably merged with a wall
SPREAD_FLAG_M = 0.02
STATIONARY_V = 0.02  # m/s and rad/s
STATIONARY_DRIFT_M = 0.02
STALE_TF_S = 0.25


def stamp_s(header: dict) -> float:
    s = header.get("stamp", {})
    return s.get("sec", 0) + s.get("nanosec", 0) * 1e-9


def index_tf(tf_msgs: list[dict]) -> dict[tuple[str, str], list[tuple]]:
    """/tf (+ /tf_static) messages -> {(parent, child): [(t, x, y, yaw)]} sorted
    by stamp."""
    out: dict[tuple[str, str], list[tuple]] = {}
    for m in tf_msgs:
        for t in m["transforms"]:
            tr, q = t["transform"]["translation"], t["transform"]["rotation"]
            key = (t["header"]["frame_id"], t["child_frame_id"].lstrip("/"))
            out.setdefault(key, []).append(
                (stamp_s(t["header"]), tr["x"], tr["y"], planar_yaw(q["z"], q["w"]))
            )
    for series in out.values():
        series.sort()
    return out


def nearest(series: list[tuple], t: float) -> tuple[tuple, float]:
    """Entry closest in time to t, and the |dt| to it."""
    best = min(series, key=lambda e: abs(e[0] - t))
    return best, abs(best[0] - t)


def compose(a: tuple, b: tuple) -> tuple:
    """(x, y, yaw) o (x, y, yaw)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    return (
        ax + bx * math.cos(ayaw) - by * math.sin(ayaw),
        ay + bx * math.sin(ayaw) + by * math.cos(ayaw),
        ayaw + byaw,
    )


def map_laser_at(tfs: dict, t: float, laser_frame: str = "laser"):
    """map->laser at stamp t from map->odom o odom->base_link o base_link->laser.
    Returns ((x, y, yaw), max_dt_s, used_fallback) or None if map->odom or
    odom->base_link has never been seen."""
    mo = tfs.get(("map", "odom"))
    ob = tfs.get(("odom", "base_link"))
    if not mo or not ob:
        return None
    (_, *mo_p), dt1 = nearest(mo, t)
    (_, *ob_p), dt2 = nearest(ob, t)
    bl = tfs.get(("base_link", laser_frame))
    fallback = bl is None
    bl_p = LASER_FALLBACK if fallback else bl[-1][1:]
    pose = compose(compose(tuple(mo_p), tuple(ob_p)), tuple(bl_p))
    return pose, max(dt1, dt2), fallback


def scan_points(scan: dict, laser_pose: tuple, max_range: float) -> list[tuple]:
    """LaserScan -> [(beam_idx, x, y, range)] in the map frame; drops nulls,
    < range_min and > max_range."""
    lx, ly, lyaw = laser_pose
    lo = max(scan.get("range_min", 0.0), 0.0)
    hi = min(scan.get("range_max", max_range), max_range)
    a0, da = scan["angle_min"], scan["angle_increment"]
    pts = []
    for i, r in enumerate(scan["ranges"]):
        if r is None or not isinstance(r, (int, float)) or not math.isfinite(r):
            continue
        if r < lo or r > hi:
            continue
        a = lyaw + a0 + i * da
        pts.append((i, lx + r * math.cos(a), ly + r * math.sin(a), r))
    return pts


def segment(points: list[tuple], gap: float = SEGMENT_GAP_M) -> list[list[tuple]]:
    """Split on beam-index gaps and Euclidean jumps; drop tiny segments."""
    segs: list[list[tuple]] = []
    cur: list[tuple] = []
    for p in points:
        if cur:
            q = cur[-1]
            if p[0] - q[0] > SEGMENT_INDEX_GAP or math.hypot(p[1] - q[1], p[2] - q[2]) > gap:
                segs.append(cur)
                cur = []
        cur.append(p)
    if cur:
        segs.append(cur)
    return [s for s in segs if len(s) >= MIN_SEGMENT_POINTS]


def characterize(seg: list[tuple], base_yaw_offset: float = 0.0, da: float = 0.0,
                 a0: float = 0.0, laser_yaw_in_base: float = 0.0) -> dict:
    n = len(seg)
    cx = sum(p[1] for p in seg) / n
    cy = sum(p[2] for p in seg) / n
    face = min(seg, key=lambda p: p[3])
    chord = math.hypot(seg[-1][1] - seg[0][1], seg[-1][2] - seg[0][2])
    b0 = math.degrees(laser_yaw_in_base + a0 + seg[0][0] * da)
    b1 = math.degrees(laser_yaw_in_base + a0 + seg[-1][0] * da)
    wrap = lambda d: (d + 180.0) % 360.0 - 180.0  # noqa: E731
    return {
        "centroid": {"x": round(cx, 3), "y": round(cy, 3)},
        "face": {"x": round(face[1], 3), "y": round(face[2], 3)},
        "chord_width_m": round(chord, 3),
        "points": n,
        "range_m": round(face[3], 3),
        "bearing_deg": [round(wrap(b0)), round(wrap(b1))],
    }


def match(segs: list[list[tuple]], hint: tuple[float, float], radius: float):
    """Segment whose centroid is nearest the hint within radius, else None.
    Never substitutes the hint."""
    best, best_d = None, radius
    for s in segs:
        cx = sum(p[1] for p in s) / len(s)
        cy = sum(p[2] for p in s) / len(s)
        d = math.hypot(cx - hint[0], cy - hint[1])
        if d <= best_d:
            best, best_d = s, d
    return best


def face_to_face(a: list[tuple], b: list[tuple]) -> float:
    """Minimum point-to-point distance between two clusters."""
    return min(math.hypot(p[1] - q[1], p[2] - q[2]) for p in a for q in b)


def centroid_dist(a: list[tuple], b: list[tuple]) -> float:
    ax = sum(p[1] for p in a) / len(a)
    ay = sum(p[2] for p in a) / len(a)
    bx = sum(p[1] for p in b) / len(b)
    by = sum(p[2] for p in b) / len(b)
    return math.hypot(ax - bx, ay - by)


def stats(vals: list[float]) -> dict:
    return {
        "mean": round(statistics.fmean(vals), 3),
        "spread": round(max(vals) - min(vals), 3),
        "stdev": round(statistics.pstdev(vals), 4),
        "per_scan": [round(v, 3) for v in vals],
    }


def ft_in(m: float) -> str:
    inches = round(m / 0.0254)
    return f"{inches // 12}'-{inches % 12}\""
