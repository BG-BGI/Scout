"""docker/scout-skills/scanmeasure.py: TF-at-stamp projection, segmentation,
target matching and pair spacing on synthetic scans."""

import importlib.util
import math
import sys
from pathlib import Path

SKILLS = Path(__file__).resolve().parents[2] / 'docker' / 'scout-skills'


def load():
    sys.path.insert(0, str(SKILLS))
    try:
        spec = importlib.util.spec_from_file_location('scanmeasure_t', SKILLS / 'scanmeasure.py')
        mod = importlib.util.module_from_spec(spec)
        sys.modules['scanmeasure_t'] = mod
        spec.loader.exec_module(mod)
        return mod
    finally:
        sys.path.remove(str(SKILLS))


sm = load()


def tfmsg(parent, child, x, y, yaw, t=10.0):
    return {'transforms': [{
        'header': {'frame_id': parent, 'stamp': {'sec': int(t), 'nanosec': int((t % 1) * 1e9)}},
        'child_frame_id': child,
        'transform': {
            'translation': {'x': x, 'y': y, 'z': 0.0},
            'rotation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)},
        },
    }]}


def make_scan(posts, laser=(0.0, 0.0, 0.0), n=720, rmax=10.0):
    """Scan from `laser` pose (map frame) hitting circular posts (x, y, radius)."""
    da = 2 * math.pi / n
    ranges = []
    for i in range(n):
        a = laser[2] - math.pi + i * da
        d, c, s = None, math.cos(a), math.sin(a)
        for px, py, r in posts:
            fx, fy = px - laser[0], py - laser[1]
            proj = fx * c + fy * s
            perp2 = fx * fx + fy * fy - proj * proj
            if proj > 0 and perp2 <= r * r:
                hit = proj - math.sqrt(r * r - perp2)
                d = hit if d is None else min(d, hit)
        ranges.append(d)
    return {'header': {'stamp': {'sec': 10, 'nanosec': 0}, 'frame_id': 'laser'},
            'angle_min': -math.pi, 'angle_increment': da, 'range_min': 0.15,
            'range_max': rmax, 'ranges': ranges}


def pose_of_laser(scan, laser_pose):
    return sm.scan_points(scan, laser_pose, 6.0)


def test_ft_in():
    assert sm.ft_in(3.048) == "10'-0\""
    assert sm.ft_in(2.64) == "8'-8\""


def test_map_laser_composes_tf_chain_at_stamp():
    tfs = sm.index_tf([
        tfmsg('map', 'odom', 1.0, 0.0, 0.0),
        tfmsg('odom', 'base_link', 0.5, 0.0, math.pi / 2),
        tfmsg('base_link', 'laser', 0.1, 0.0, 0.0),
    ])
    (x, y, yaw), dt, fb = sm.map_laser_at(tfs, 10.0)
    assert (round(x, 3), round(y, 3), round(yaw, 3)) == (1.5, 0.1, round(math.pi / 2, 3))
    assert dt == 0 and not fb


def test_map_laser_fallback_when_static_missing_and_none_when_chain_missing():
    tfs = sm.index_tf([tfmsg('map', 'odom', 0, 0, 0), tfmsg('odom', 'base_link', 0, 0, 0)])
    pose, _, fb = sm.map_laser_at(tfs, 10.0)
    assert fb and round(pose[2], 3) == round(math.pi, 3)
    assert sm.map_laser_at(sm.index_tf([tfmsg('map', 'odom', 0, 0, 0)]), 10.0) is None


def test_tf_picks_nearest_stamp():
    tfs = sm.index_tf([
        tfmsg('map', 'odom', 0, 0, 0, t=9.0), tfmsg('map', 'odom', 5, 0, 0, t=10.1),
        tfmsg('odom', 'base_link', 0, 0, 0), tfmsg('base_link', 'laser', 0, 0, 0),
    ])
    (x, _, _), dt, _ = sm.map_laser_at(tfs, 10.0)
    assert x == 5 and round(dt, 3) == 0.1


def test_segments_and_pair_spacing():
    posts = [(1.0, 1.0, 0.08), (-1.4, 1.0, 0.08)]   # centres 2.4 m apart
    laser = (0.0, 0.0, 0.0)
    scan = make_scan(posts, laser)
    segs = sm.segment(pose_of_laser(scan, laser))
    a = sm.match(segs, (1.05, 1.05), 0.5)
    b = sm.match(segs, (-1.4, 1.0), 0.5)
    assert a and b and a is not b
    f2f = sm.face_to_face(a, b)
    assert abs(f2f - (2.4 - 0.16)) < 0.03
    assert abs(sm.centroid_dist(a, b) - 2.4) < 0.1


def test_missing_target_not_backfilled():
    laser = (0.0, 0.0, 0.0)
    segs = sm.segment(pose_of_laser(make_scan([(1.0, 1.0, 0.08)], laser), laser))
    assert sm.match(segs, (-3.0, -3.0), 0.8) is None


def test_nulls_and_range_limits_dropped():
    scan = {'angle_min': 0.0, 'angle_increment': 0.1, 'range_min': 0.15, 'range_max': 10,
            'ranges': [None, 0.05, 2.0, 7.0, float('inf')]}
    pts = sm.scan_points(scan, (0, 0, 0), 6.0)
    assert [p[0] for p in pts] == [2]


def test_sparse_segments_dropped_and_stats():
    pts = [(i, i * 0.01, 1.0, 1.0) for i in range(3)]
    assert sm.segment(pts) == []
    s = sm.stats([2.34, 2.34, 2.33])
    assert s['spread'] == 0.01 and s['mean'] == 2.337


def test_characterize_reports_face_and_width():
    laser = (0.0, 0.0, 0.0)
    segs = sm.segment(pose_of_laser(make_scan([(1.0, 0.0, 0.1)], laser), laser))
    c = sm.characterize(segs[0], da=2 * math.pi / 720, a0=-math.pi)
    assert abs(c['range_m'] - 0.9) < 0.02 and 0.1 < c['chord_width_m'] < 0.21


# --- wall spans (base_link frame, synthetic ray-cast rooms) -------------------

ORIGIN = sm.LASER_FALLBACK[:2]
YAW20 = math.radians(20)


def rect(cx, cy, w, h):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [((x0, y1), (x1, y1)), ((x1, y1), (x1, y0)), ((x1, y0), (x0, y0)), ((x0, y0), (x0, y1))]


def cast(origin, ang, segs, circles):
    ox, oy = origin
    c, s = math.cos(ang), math.sin(ang)
    hits = []
    for (x1, y1), (x2, y2) in segs:
        ex, ey = x2 - x1, y2 - y1
        den = c * ey - s * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
        u = ((x1 - ox) * s - (y1 - oy) * c) / den
        if t > 0 and 0 <= u <= 1:
            hits.append(t)
    for px, py, r in circles:
        fx, fy = px - ox, py - oy
        proj = fx * c + fy * s
        perp2 = fx * fx + fy * fy - proj * proj
        if proj > 0 and perp2 <= r * r:
            hits.append(proj - math.sqrt(r * r - perp2))
    return min(hits) if hits else None


def room_scan(robot, segs, circles=(), n=1800, rmax=12.0, sigma=0.0, rng=None):
    """Scan taken from the 180-degree-mounted laser of a robot at `robot`
    (x, y, yaw in the world), against wall segments and circular clutter."""
    laser = sm.compose(robot, sm.LASER_FALLBACK)
    da = 2 * math.pi / n
    ranges = []
    for i in range(n):
        r = cast(laser[:2], laser[2] - math.pi + i * da, segs, circles)
        if r is None or r > rmax:
            ranges.append(None)
        else:
            ranges.append(r + (rng.gauss(0, sigma) if sigma else 0.0))
    return {'header': {'stamp': {'sec': 10, 'nanosec': 0}, 'frame_id': 'laser'},
            'angle_min': -math.pi, 'angle_increment': da, 'range_min': 0.15,
            'range_max': rmax, 'ranges': ranges}


def spans_of(scans, **kw):
    pts = [sm.scan_points(s, sm.LASER_FALLBACK, 10.0) for s in scans]
    return sm.wall_spans(pts, ORIGIN, **kw)


def wall_near(res, bearing):
    return min(res['walls'],
               key=lambda w: abs(math.remainder(w['bearing_rad'] - bearing, 2 * math.pi)))


def axis_gap(res):
    a, b = (p['axis_rad'] for p in res['pairs'][:2])
    d = abs(a - b) % math.pi
    return min(d, math.pi - d)


def test_room_off_centre_and_yawed_gives_both_spans():
    scan = room_scan((0.4, -0.3, YAW20), rect(0, 0, 4.0, 3.0))
    res = spans_of([scan] * 3)
    assert len(res['walls']) == 4 and len(res['pairs']) == 2
    assert sorted(round(p['span_m'], 2) for p in res['pairs']) == [3.0, 4.0]
    assert all(abs(p['span_m'] - round(p['span_m'])) < 0.01 for p in res['pairs'])
    assert abs(axis_gap(res) - math.pi / 2) < math.radians(2)
    assert all(p['parallel_err_deg'] < 0.5 and p['spread_m'] == 0 for p in res['pairs'])
    assert not [f for f in res['flags'] if 'orthogonal' in f]


def test_180_mount_front_wall_is_at_bearing_zero_with_base_link_distance():
    scan = room_scan((0.0, 0.0, 0.0), rect(0.5, 0, 5.0, 3.0))
    res = spans_of([scan])
    front = wall_near(res, 0.0)
    assert abs(front['bearing_rad']) < 0.01 and abs(front['distance_m'] - 3.0) < 0.01
    back = wall_near(res, math.pi)
    assert abs(abs(back['bearing_rad']) - math.pi) < 0.01 and abs(back['distance_m'] - 2.0) < 0.01
    assert abs(front['p0'][0] - 3.0) < 0.01 and abs(front['p1'][0] - 3.0) < 0.01


def test_180_mount_yaw_handedness():
    scan = room_scan((0.0, 0.0, YAW20), rect(0, 0, 6.0, 4.0))
    res = spans_of([scan])
    front = wall_near(res, -YAW20)
    assert abs(front['bearing_rad'] + YAW20) < 0.01 and abs(front['distance_m'] - 3.0) < 0.01


def test_bearing_range_is_half_open_at_pi():
    scan = room_scan((0.0, 0.0, 0.0), rect(0, 0, 6.0, 4.0))
    for w in spans_of([scan])['walls']:
        assert -math.pi < w['bearing_rad'] <= math.pi


def test_door_gap_keeps_span():
    full = rect(0, 0, 4.0, 3.0)
    gapped = [((-2.0, 1.5), (-0.5, 1.5)), ((0.4, 1.5), (2.0, 1.5))] + full[1:]
    res = spans_of([room_scan((0.2, -0.2, YAW20), gapped)] * 3)
    ref = spans_of([room_scan((0.2, -0.2, YAW20), full)] * 3)
    assert 4 <= len(res['walls']) <= 5
    spans = sorted(p['span_m'] for p in res['pairs'])
    assert all(abs(s - 3.0) < 0.01 or abs(s - 4.0) < 0.01 for s in spans)
    assert ({round(p['span_m'], 1) for p in res['pairs']}
            == {round(p['span_m'], 1) for p in ref['pairs']})
    assert any(abs(p['span_m'] - 3.0) < 0.01 for p in res['pairs'])


def test_clutter_alone_gives_no_wall_and_no_pair():
    chair_legs = [(1.0 + 0.4 * i, 1.0 + 0.4 * j, 0.025) for i in range(2) for j in range(2)]
    post = [(-1.0, 1.2, 0.1)]
    box = rect(1.5, -1.0, 0.3, 0.3)
    res = spans_of([room_scan((0.0, 0.0, 0.3), box, post + chair_legs)] * 3)
    assert res['walls'] == [] and res['pairs'] == []
    assert any('no straight wall' in f for f in res['flags'])


def test_clutter_in_a_room_adds_no_wall():
    chair_legs = [(0.8 + 0.4 * i, 0.9 + 0.4 * j, 0.025) for i in range(2) for j in range(2)]
    segs = rect(0, 0, 4.0, 3.0) + rect(-0.8, -0.6, 0.3, 0.3)
    res = spans_of([room_scan((0.0, 0.2, 0.1), segs, [(-1.2, 0.6, 0.1)] + chair_legs)] * 3)
    assert len(res['walls']) == 4 and len(res['pairs']) == 2
    assert sorted(round(p['span_m'], 2) for p in res['pairs']) == [3.0, 4.0]


def test_single_wall_on_an_axis_is_flagged_not_paired():
    segs = [((-2.0, 1.5), (2.0, 1.5)), ((2.0, -1.5), (-2.0, -1.5)), ((2.0, 1.5), (2.0, -1.5))]
    res = spans_of([room_scan((0.0, 0.0, 0.1), segs)] * 3)
    assert len(res['walls']) == 3 and len(res['pairs']) == 1
    assert abs(res['pairs'][0]['span_m'] - 3.0) < 0.01
    lone = wall_near(res, 0.0 - 0.1)
    assert any(f.startswith(lone['id'] + ':') and 'no facing wall' in f for f in res['flags'])
    assert any('orthogonal' in f for f in res['flags'])


def test_skewed_walls_are_not_paired():
    a, b = (-2.0, 1.5), (2.0, 1.5)
    c, d = (2.0, -1.5 + 4.0 * math.tan(math.radians(5))), (-2.0, -1.5)
    segs = [(a, b), (b, c), (c, d), (d, a)]
    res = spans_of([room_scan((0.0, 0.0, 0.0), segs)] * 3)
    assert len(res['pairs']) == 1 and abs(res['pairs'][0]['span_m'] - 4.0) < 0.01
    assert any('not paired' in f and '5.0 deg' in f for f in res['flags'])


def test_noise_three_scans_span_error_and_spread():
    import random
    rng = random.Random(1234)
    segs = rect(0, 0, 4.0, 3.0)
    scans = [room_scan((0.4, -0.3, YAW20), segs, sigma=0.01, rng=rng) for _ in range(3)]
    res = spans_of(scans)
    assert len(res['pairs']) == 2
    for p in res['pairs']:
        assert min(abs(p['span_m'] - 3.0), abs(p['span_m'] - 4.0)) < 0.015
        assert 0 < p['spread_m'] < 0.03
    assert all(w['rms_m'] < 0.02 and w['spread_m'] < 0.03 for w in res['walls'])


def test_wall_seen_in_a_minority_of_scans_is_dropped():
    segs = rect(0, 0, 4.0, 3.0)
    extra = segs + [((-1.0, 0.6), (1.0, 0.6))]
    scans = [room_scan((0.0, 0.0, 0.0), extra), room_scan((0.0, 0.0, 0.0), segs),
             room_scan((0.0, 0.0, 0.0), segs)]
    res = spans_of(scans)
    assert len(res['walls']) == 4
    assert any('dropped' in f for f in res['flags'])


def test_walls_sorted_by_length_and_ids_follow():
    res = spans_of([room_scan((0.0, 0.0, 0.0), rect(0, 0, 4.0, 3.0))])
    lengths = [w['length_m'] for w in res['walls']]
    assert lengths == sorted(lengths, reverse=True)
    assert [w['id'] for w in res['walls']] == ['w0', 'w1', 'w2', 'w3']


def test_min_length_parameter():
    scan = room_scan((0.0, 0.0, 0.0), rect(0, 0, 4.0, 3.0))
    assert len(spans_of([scan], min_length_m=3.5)['walls']) == 2


def test_parallel_walls_on_one_side_are_not_a_pair():
    segs = [((1.0, -1.0), (1.0, 1.0)), ((2.5, -5.0), (2.5, 5.0))]
    res = spans_of([room_scan((0.0, 0.0, 0.0), segs)])
    assert len(res['walls']) == 3 and res['pairs'] == []


def test_base_laser_from_static_tf_and_fallback():
    tfs = sm.index_tf([tfmsg('base_link', 'laser', 0.073, 0.0, -math.pi)])
    (x, y, yaw), fb = sm.base_laser(tfs, 'laser')
    assert (x, y) == (0.073, 0.0) and abs(abs(yaw) - math.pi) < 1e-9 and not fb
    pose, fb = sm.base_laser({}, 'laser')
    assert pose == sm.LASER_FALLBACK and fb


def test_scan_agreement_parked_vs_moved():
    segs = rect(0, 0, 4.0, 3.0)
    a = room_scan((0.0, 0.0, 0.0), segs)
    assert sm.scan_agreement([a]) is None
    assert sm.scan_agreement([a, room_scan((0.0, 0.0, 0.0), segs)]) < 0.001
    assert sm.scan_agreement([a, room_scan((0.25, 0.0, 0.0), segs)]) > sm.STATIONARY_AGREE_M


def test_scan_agreement_matches_beams_by_angle():
    segs = rect(0, 0, 4.0, 3.0)
    a = room_scan((0.0, 0.0, 0.0), segs, n=1800)
    b = room_scan((0.0, 0.0, 0.0), segs, n=1797)
    assert sm.scan_agreement([a, b]) < 0.02


def test_points_xy_decimates_and_rounds():
    pts = [(i, i * 0.0012345, 1.0, 1.0) for i in range(5000)]
    out = sm.points_xy(pts)
    assert len(out) <= 1800 and out[0] == [0.0, 1.0] and out[1] == [0.004, 1.0]


def test_pair_requires_the_origin_between_the_walls():
    def fit(nx, ny, d):
        return sm.Fit(nx, ny, d, nx * d, ny * d, -2.0, 2.0, 50, 0.005)

    walls = [('w0', fit(1, 0, 2.0), {0: fit(1, 0, 2.0)}),
             ('w1', fit(-1, 0, 1.0), {0: fit(-1, 0, 1.0)})]
    pairs, _ = sm.build_pairs(walls, (0.0, 0.0))
    assert len(pairs) == 1 and pairs[0]['span_m'] == 3.0 and pairs[0]['axis_rad'] == 0.0
    assert sm.build_pairs(walls, (3.0, 0.0)) == ([], [])
