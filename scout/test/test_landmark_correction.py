"""scout.core.landmark_correction — median window, thresholds, cooldown."""

import math

from scout.core.landmark_correction import LandmarkCorrector, circular_median

AMCL = (0.0, 0.0, 0.0)


def _feed(c, n, solve, amcl=AMCL, v=0.0, w=0.0, dist=0.6, t=0.0):
    out = None
    for i in range(n):
        out = c.add(solve, amcl, v, w, dist, t + i * 0.1)
    return out


def test_consistent_window_triggers_once_then_cooldown():
    c = LandmarkCorrector()
    assert _feed(c, 4, (1.0, 0.5, 0.2)) is None  # window not full
    r = c.add((1.0, 0.5, 0.2), AMCL, 0, 0, 0.6, 1.0)
    dx, dy, dyaw, dist, pose = r
    assert (dx, dy) == (1.0, 0.5) and abs(dyaw - 0.2) < 1e-9
    assert dist == 0.6
    assert pose[:2] == (1.0, 0.5) and abs(pose[2] - 0.2) < 1e-9
    assert c.count == 1
    # still wrong amcl, inside cooldown: suppressed
    assert _feed(c, 5, (1.0, 0.5, 0.2), t=2.0) is None
    # after cooldown: fires again
    assert c.add((1.0, 0.5, 0.2), AMCL, 0, 0, 0.6, 10.0) is not None
    assert c.count == 2


def test_noisy_window_does_not_trigger():
    c = LandmarkCorrector()
    out = None
    for i, x in enumerate([1.0, 1.2, 0.9, 1.3, 1.0]):
        out = c.add((x, 0.0, 0.0), AMCL, 0, 0, 0.6, i)
    assert out is None


def test_noisy_yaw_does_not_trigger():
    c = LandmarkCorrector()
    out = None
    for i, y in enumerate([0.3, 0.4, 0.3, 0.5, 0.3]):  # ~11 deg spread
        out = c.add((0.0, 0.0, y), AMCL, 0, 0, 0.6, i)
    assert out is None


def test_small_error_does_not_trigger():
    c = LandmarkCorrector()
    assert _feed(c, 5, (0.05, 0.05, math.radians(1.0))) is None


def test_yaw_only_error_triggers():
    c = LandmarkCorrector()
    r = _feed(c, 5, (0.0, 0.0, math.radians(5.0)))
    assert r is not None and abs(math.degrees(r[2]) - 5.0) < 1e-6


def test_yaw_wrap_at_pi():
    pts = [math.radians(d) for d in (179.0, -179.0, 180.0, -180.0, 179.5)]
    assert abs(abs(circular_median(pts)) - math.pi) < math.radians(1.0)
    c = LandmarkCorrector()
    out = None
    for i, a in enumerate(pts):
        out = c.add((0.0, 0.0, a), (0.0, 0.0, math.radians(-179.0)),
                    0, 0, 0.6, i)
    assert out is None  # ~1 deg apart across the wrap: no correction
    c2 = LandmarkCorrector()
    out = None
    for i, a in enumerate(pts):
        out = c2.add((0.0, 0.0, a), (0.0, 0.0, math.radians(170.0)),
                     0, 0, 0.6, i)
    assert out is not None and abs(math.degrees(out[2]) - 10.0) < 1.0


def test_fast_robot_suppresses_and_clears_window():
    c = LandmarkCorrector()
    _feed(c, 4, (1.0, 0.0, 0.0))
    assert c.add((1.0, 0.0, 0.0), AMCL, 0.3, 0.0, 0.6, 1.0) is None
    assert _feed(c, 4, (1.0, 0.0, 0.0), t=2.0) is None  # window restarted
    assert c.add((1.0, 0.0, 0.0), AMCL, 0.0, 0.5, 0.6, 3.0) is None  # fast yaw
    assert _feed(c, 5, (1.0, 0.0, 0.0), t=4.0) is not None


def test_distance_gate():
    c = LandmarkCorrector()
    assert _feed(c, 6, (1.0, 0.0, 0.0), dist=2.0) is None
    assert _feed(c, 5, (1.0, 0.0, 0.0), dist=1.4) is not None
