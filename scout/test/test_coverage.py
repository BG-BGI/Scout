import numpy as np

from scout.core import coverage as c


def test_scanline_rectangle_and_concave():
    rect = [(0, 0), (4, 0), (4, 4), (0, 4)]
    assert c.scanline(rect, 2.0) == [(0.0, 4.0)]
    # A C-shape gives two spans on a line crossing both arms.
    cshape = [(0, 0), (4, 0), (4, 1), (1, 1), (1, 3), (4, 3), (4, 4), (0, 4)]
    spans = c.scanline(cshape, 2.0)
    assert len(spans) == 1  # y=2 crosses the open mouth once (0..1)
    assert spans[0] == (0.0, 1.0)


def test_inflate_grows_and_zero_is_noop():
    b = np.zeros((5, 5), bool)
    b[2, 2] = True
    g = c.inflate(b, 1)
    assert g[1, 2] and g[3, 2] and g[2, 1] and g[2, 3]
    assert not g[0, 2]
    assert not c.inflate(b, 0)[1, 2]  # 0 dilations = no-op


def test_plan_coverage_returns_builtin_floats():
    grid = -np.ones((40, 40), dtype=np.int16)  # all unknown = coverable
    poly = [(0.1, 0.1), (1.8, 0.1), (1.8, 1.8), (0.1, 1.8)]
    route = c.plan_coverage(grid, (0.0, 0.0), 0.05, poly, spacing=0.5, inflation=0.0)
    assert route
    for wp in route:
        for v in (wp['x'], wp['y'], wp['yaw']):
            assert type(v) is float  # NOT a numpy scalar (yaml.safe_dump regression)


def test_plan_coverage_obstacle_splits_stripe():
    grid = -np.ones((40, 60), dtype=np.int16)
    grid[:, 28:32] = 100  # vertical wall at x ~ 1.4..1.6 m
    poly = [(0.1, 0.1), (2.8, 0.1), (2.8, 1.8), (0.1, 1.8)]
    route = c.plan_coverage(grid, (0.0, 0.0), 0.05, poly, spacing=0.5, inflation=0.0)
    assert any(wp['x'] < 1.3 for wp in route)   # coverage left of the wall
    assert any(wp['x'] > 1.7 for wp in route)   # and right of it
