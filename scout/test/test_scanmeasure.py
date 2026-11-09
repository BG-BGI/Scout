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
