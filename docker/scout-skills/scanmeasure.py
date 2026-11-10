"""Pure lidar-measurement helpers for the scan_measure and wall_spans tools (no
I/O, so they unit-test off the robot). server.py does the rosbridge read; this
module projects scans into the map frame with the transform at each scan's own
stamp, segments the returns into clusters, matches clusters to target hints and
derives spacing + repeatability across scans. The wall-spans half (bottom of
the file) works in base_link and needs no map or localization."""

import math
import statistics
from typing import NamedTuple

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


# --- wall spans (base_link frame; no map, no localization) -------------------

WALL_MIN_LENGTH_M = 0.6  # default; clutter (posts, chair legs, boxes) is shorter
WALL_MIN_POINTS = 10
WALL_MAX_RMS_M = 0.03  # straight-line residual; arcs and leg rows fail it
WALL_SPLIT_TOL_M = 0.05  # split-and-merge: chord deviation that makes a corner
WALL_INLIER_TOL_M = 0.03  # one trim-and-refit pass against the fitted line
FRAGMENT_MIN_POINTS = MIN_SEGMENT_POINTS
# Fragments of one plane (grazing splits, the array wrap) re-join when they agree
# this well and the hole between them is this small.
WALL_MERGE_ANGLE_RAD = math.radians(2.0)
WALL_MERGE_OFFSET_M = 0.05
WALL_MERGE_GAP_M = 0.3
WALL_MATCH_ANGLE_RAD = math.radians(3.0)  # same wall across scans
WALL_MATCH_DIST_M = 0.10
WALL_MATCH_OVERLAP = 0.5  # of the shorter wall; keeps collinear walls apart
PAIR_MAX_PARALLEL_DEG = 3.0
PAIR_NEAR_MISS_DEG = 15.0  # opposite walls closer than this get a flag, not a pair
ORTHO_TOL_DEG = 15.0
STATIONARY_AGREE_M = 0.04  # 75th-pct |dr| first-vs-last scan; untuned vs real noise
AGREE_MIN_BEAMS = 50
POINTS_CAP = 1800


class Fit(NamedTuple):
    """Wall line in the working frame: unit normal (nx, ny) from the frame
    origin toward the wall, perpendicular distance d from that origin, point
    centroid (cx, cy), extent [t0, t1] along u = (-ny, nx) measured from the
    foot of the perpendicular, point count n and RMS residual."""

    nx: float
    ny: float
    d: float
    cx: float
    cy: float
    t0: float
    t1: float
    n: int
    rms: float

    @property
    def length(self) -> float:
        return self.t1 - self.t0


def base_laser(tfs: dict, laser_frame: str = "laser") -> tuple[tuple, bool]:
    """base_link->laser (x, y, yaw) from /tf_static, and whether LASER_FALLBACK
    had to stand in for it (static TF not replayed by rosbridge)."""
    series = tfs.get(("base_link", laser_frame.lstrip("/")))
    if series is None:
        return LASER_FALLBACK, True
    return tuple(series[-1][1:]), False


def _valid_range(scan: dict, r) -> bool:
    return (
        isinstance(r, (int, float))
        and math.isfinite(r)
        and scan.get("range_min", 0.0) <= r <= scan.get("range_max", math.inf)
    )


def scan_agreement(scans: list[dict]) -> float | None:
    """75th-percentile |dr| (m) between the first and last scan, matching beams
    by angle (rplidar array lengths drift between revolutions). Near the range
    noise when parked. Not the median: a slide along the long walls leaves most
    beams unchanged. None when there is nothing to compare."""
    if len(scans) < 2:
        return None
    a, b = scans[0], scans[-1]
    diffs = []
    for i, ra in enumerate(a["ranges"]):
        if not _valid_range(a, ra):
            continue
        j = round((a["angle_min"] + i * a["angle_increment"] - b["angle_min"])
                  / b["angle_increment"])
        if 0 <= j < len(b["ranges"]) and _valid_range(b, b["ranges"][j]):
            diffs.append(abs(ra - b["ranges"][j]))
    if len(diffs) < AGREE_MIN_BEAMS:
        return None
    return statistics.quantiles(diffs, n=4)[2]


def fit_line(pts: list[tuple], origin: tuple[float, float]) -> Fit:
    """Total-least-squares line through [(idx, x, y, r)]. The normal is
    oriented away from `origin` (the laser), so every wall faces the sensor."""
    n = len(pts)
    cx = sum(p[1] for p in pts) / n
    cy = sum(p[2] for p in pts) / n
    sxx = sum((p[1] - cx) ** 2 for p in pts)
    syy = sum((p[2] - cy) ** 2 for p in pts)
    sxy = sum((p[1] - cx) * (p[2] - cy) for p in pts)
    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    nx, ny = -math.sin(theta), math.cos(theta)
    if nx * (cx - origin[0]) + ny * (cy - origin[1]) < 0:
        nx, ny = -nx, -ny
    ux, uy = -ny, nx
    rms = math.sqrt(sum((nx * (p[1] - cx) + ny * (p[2] - cy)) ** 2 for p in pts) / n)
    ts = [ux * p[1] + uy * p[2] for p in pts]
    return Fit(nx, ny, nx * cx + ny * cy, cx, cy, min(ts), max(ts), n, rms)


def _off_line(f: Fit, x: float, y: float) -> float:
    return abs(f.nx * x + f.ny * y - f.d)


def _fit_trimmed(pts: list[tuple], origin: tuple[float, float]) -> tuple[Fit, list[tuple]]:
    fit = fit_line(pts, origin)
    keep = [p for p in pts if _off_line(fit, p[1], p[2]) <= WALL_INLIER_TOL_M]
    if FRAGMENT_MIN_POINTS <= len(keep) < len(pts):
        return fit_line(keep, origin), keep
    return fit, pts


def _chord_deviation(seg: list[tuple]) -> tuple[float, int]:
    """Largest perpendicular distance of any point from the first-last chord,
    and which point."""
    x0, y0, x1, y1 = seg[0][1], seg[0][2], seg[-1][1], seg[-1][2]
    dx, dy = x1 - x0, y1 - y0
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        return 0.0, 0
    devs = [abs(dy * (p[1] - x0) - dx * (p[2] - y0)) / norm for p in seg]
    k = max(range(len(seg)), key=devs.__getitem__)
    return devs[k], k


def split_chain(chain: list[tuple], tol: float = WALL_SPLIT_TOL_M) -> list[list[tuple]]:
    """Split-and-merge, split half: cut a contiguous chain at its worst chord
    deviation until every piece is straight within `tol`."""
    out, stack = [], [chain]
    while stack:
        cur = stack.pop()
        if len(cur) < FRAGMENT_MIN_POINTS:
            continue
        dev, k = _chord_deviation(cur)
        if dev > tol and len(cur) >= 2 * FRAGMENT_MIN_POINTS + 1:
            stack.extend((cur[:k], cur[k + 1:]))
        else:
            out.append(cur)
    return out


def _same_plane(a: Fit, b: Fit) -> bool:
    if a.nx * b.nx + a.ny * b.ny < math.cos(WALL_MERGE_ANGLE_RAD):
        return False
    return (
        _off_line(a, b.cx, b.cy) <= WALL_MERGE_OFFSET_M
        and _off_line(b, a.cx, a.cy) <= WALL_MERGE_OFFSET_M
    )


def merge_fragments(
    frags: list[tuple[Fit, list[tuple]]], origin: tuple[float, float]
) -> list[tuple[Fit, list[tuple]]]:
    """Split-and-merge, merge half: re-join fragments of one plane whose gap is
    at most WALL_MERGE_GAP_M (grazing-angle splits, the 360-degree array wrap).
    A real hole wider than that (a door) keeps two walls."""
    frags = list(frags)
    i = 0
    while i < len(frags):
        j = i + 1
        while j < len(frags):
            fa, fb = frags[i][0], frags[j][0]
            gap = max(fa.t0, fb.t0) - min(fa.t1, fb.t1)
            if _same_plane(fa, fb) and gap <= WALL_MERGE_GAP_M:
                fit, keep = _fit_trimmed(frags[i][1] + frags[j][1], origin)
                if fit.rms <= WALL_MAX_RMS_M:
                    frags[i] = (fit, keep)
                    del frags[j]
                    j = i + 1
                    continue
            j += 1
        i += 1
    return frags


def scan_walls(
    points: list[tuple], origin: tuple[float, float], min_length_m: float = WALL_MIN_LENGTH_M
) -> list[Fit]:
    """Straight wall segments in one scan's [(idx, x, y, r)] points. Clutter is
    rejected on length, point count and RMS residual."""
    frags = []
    for chain in segment(points):
        for piece in split_chain(chain):
            frags.append(_fit_trimmed(piece, origin))
    return [
        f
        for f, _ in merge_fragments(frags, origin)
        if f.length >= min_length_m and f.n >= WALL_MIN_POINTS and f.rms <= WALL_MAX_RMS_M
    ]


def _same_wall(a: Fit, b: Fit) -> bool:
    if a.nx * b.nx + a.ny * b.ny < math.cos(WALL_MATCH_ANGLE_RAD):
        return False
    if abs(a.d - b.d) > WALL_MATCH_DIST_M:
        return False
    overlap = min(a.t1, b.t1) - max(a.t0, b.t0)
    return overlap >= WALL_MATCH_OVERLAP * min(a.length, b.length)


def group_walls(per_scan: list[list[Fit]]) -> list[list[tuple[int, Fit]]]:
    """Walls seen in several scans -> [[(scan_idx, Fit), ...]], one group per
    physical wall. A scan contributes at most one wall to a group."""
    groups: list[list[tuple[int, Fit]]] = []
    for si, walls in enumerate(per_scan):
        taken: set[int] = set()
        for w in walls:
            hit = next(
                (gi for gi, g in enumerate(groups) if gi not in taken and _same_wall(g[-1][1], w)),
                None,
            )
            if hit is None:
                groups.append([(si, w)])
                hit = len(groups) - 1
            else:
                groups[hit].append((si, w))
            taken.add(hit)
    return groups


def _mean_fit(fits: list[Fit]) -> Fit:
    k = len(fits)
    mx, my = sum(f.nx for f in fits), sum(f.ny for f in fits)
    norm = math.hypot(mx, my)
    return Fit(
        mx / norm, my / norm,
        sum(f.d for f in fits) / k, sum(f.cx for f in fits) / k, sum(f.cy for f in fits) / k,
        sum(f.t0 for f in fits) / k, sum(f.t1 for f in fits) / k,
        round(sum(f.n for f in fits) / k), sum(f.rms for f in fits) / k,
    )


def _bearing(nx: float, ny: float) -> float:
    """Direction of the wall normal in (-pi, pi]; rounded value never leaves it."""
    b = math.atan2(ny, nx)
    if b <= -math.pi + 1e-9:
        b = math.pi
    return min(round(b, 4), math.pi)


def _axis(mx: float, my: float) -> float:
    """Span direction mod pi, in [0, pi)."""
    a = round(math.atan2(my, mx) % math.pi, 4)
    return 0.0 if a >= math.pi else a


def _wall_dict(wid: str, fit: Fit, spread: float) -> dict:
    ux, uy = -fit.ny, fit.nx
    ends = [[round(fit.d * fit.nx + t * ux, 3), round(fit.d * fit.ny + t * uy, 3)]
            for t in (fit.t0, fit.t1)]
    return {
        "id": wid,
        "bearing_rad": _bearing(fit.nx, fit.ny),
        "distance_m": round(fit.d, 3),
        "spread_m": round(spread, 3),
        "length_m": round(fit.length, 3),
        "points": fit.n,
        "rms_m": round(fit.rms, 4),
        "p0": ends[0],
        "p1": ends[1],
    }


def _pair_geometry(
    a: Fit, b: Fit, origin: tuple[float, float]
) -> tuple[float, float, float, float] | None:
    """(span_m, mx, my, parallel_err_deg) for two walls facing each other with
    `origin` between them, else None. m is the mean normal, pointing b -> a."""
    mx, my = a.nx - b.nx, a.ny - b.ny
    norm = math.hypot(mx, my)
    if norm < 1e-6:
        return None
    mx, my = mx / norm, my / norm
    sa = a.d * (mx * a.nx + my * a.ny)
    sb = b.d * (mx * b.nx + my * b.ny)
    if not sb < mx * origin[0] + my * origin[1] < sa:
        return None
    err = math.degrees(math.acos(max(-1.0, min(1.0, -(a.nx * b.nx + a.ny * b.ny)))))
    return sa - sb, mx, my, err


def _pair_dict(ida: str, idb: str, geo: tuple, spans: list[float]) -> dict:
    return {
        "a": ida,
        "b": idb,
        "span_m": round(statistics.fmean(spans), 3),
        "spread_m": round(max(spans) - min(spans), 3),
        "axis_rad": _axis(geo[1], geo[2]),
        "parallel_err_deg": round(geo[3], 2),
    }


def build_pairs(
    walls: list[tuple[str, Fit, dict[int, Fit]]],
    origin: tuple[float, float],
    max_parallel_deg: float = PAIR_MAX_PARALLEL_DEG,
) -> tuple[list[dict], list[str]]:
    """Pair near-parallel walls with the origin between them. Span per scan from
    that scan's own lines, then mean + spread. Near misses are flagged, never
    forced into a pair."""
    pairs: list[tuple[float, float, dict]] = []
    flags: list[str] = []
    for ia, (ida, fa, ma) in enumerate(walls):
        for idb, fb, mb in walls[ia + 1:]:
            geo = _pair_geometry(fa, fb, origin)
            if geo is None or geo[3] > PAIR_NEAR_MISS_DEG:
                continue
            if geo[3] > max_parallel_deg:
                flags.append(
                    f"{ida}/{idb}: {geo[3]:.1f} deg from parallel (limit "
                    f"{max_parallel_deg:g}) - not paired"
                )
                continue
            per = [_pair_geometry(ma[si], mb[si], origin) for si in sorted(ma.keys() & mb.keys())]
            spans = [g[0] for g in per if g] or [geo[0]]
            pairs.append((-min(fa.length, fb.length), max(spans) - min(spans),
                          _pair_dict(ida, idb, geo, spans)))
    pairs.sort(key=lambda p: p[:2])
    return [p[2] for p in pairs], flags


def _has_orthogonal_pair(pairs: list[dict]) -> bool:
    tol = math.radians(ORTHO_TOL_DEG)
    for i, p in enumerate(pairs):
        for q in pairs[i + 1:]:
            d = abs(p["axis_rad"] - q["axis_rad"]) % math.pi
            if abs(min(d, math.pi - d) - math.pi / 2) <= tol:
                return True
    return False


def wall_spans(
    per_scan_points: list[list[tuple]],
    origin: tuple[float, float],
    min_length_m: float = WALL_MIN_LENGTH_M,
    max_parallel_deg: float = PAIR_MAX_PARALLEL_DEG,
) -> dict:
    """base_link points of each scan -> {"walls", "pairs", "flags"}: straight
    walls fused across scans (mean + spread), and spans between facing walls.
    `origin` is the laser position in base_link."""
    per_scan = [scan_walls(pts, origin, min_length_m) for pts in per_scan_points]
    need = len(per_scan) // 2 + 1
    groups = group_walls(per_scan)
    kept = [g for g in groups if len(g) >= need]
    flags: list[str] = []
    if len(kept) < len(groups):
        flags.append(
            f"{len(groups) - len(kept)} wall candidate(s) seen in fewer than "
            f"{need}/{len(per_scan)} scans dropped"
        )
    fused = [
        (_mean_fit([f for _, f in g]), dict(g), max(f.d for _, f in g) - min(f.d for _, f in g))
        for g in kept
    ]
    fused.sort(key=lambda e: -e[0].length)
    named = [(f"w{i}", fit, members) for i, (fit, members, _) in enumerate(fused)]
    walls = [_wall_dict(wid, fit, fused[i][2]) for i, (wid, fit, _) in enumerate(named)]
    pairs, pair_flags = build_pairs(named, origin, max_parallel_deg)
    flags += pair_flags
    if not walls:
        flags.append(f"no straight wall >= {min_length_m:g} m found")
    paired = {w for p in pairs for w in (p["a"], p["b"])}
    flags += [
        f"{w['id']}: no facing wall seen - no span for this axis"
        for w in walls if w["id"] not in paired
    ]
    flags += [
        f"{p['a']}-{p['b']}: span spread {p['spread_m'] * 100:.1f} cm > "
        f"{SPREAD_FLAG_M * 100:.0f} cm"
        for p in pairs if p["spread_m"] > SPREAD_FLAG_M
    ]
    if not _has_orthogonal_pair(pairs):
        flags.append(
            f"no two pairs roughly orthogonal (+-{ORTHO_TOL_DEG:g} deg) - "
            "calibration needs two non-parallel spans"
        )
    return {"walls": walls, "pairs": pairs, "flags": flags}


def points_xy(points: list[tuple], cap: int = POINTS_CAP) -> list[list[float]]:
    """Decimated [[x, y]] (3 decimals) for a UI overlay, at most `cap` points."""
    step = max(1, math.ceil(len(points) / cap))
    return [[round(p[1], 3), round(p[2], 3)] for p in points[::step]]
