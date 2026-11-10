"""Wall geometry -> nav2 grid (docker/floorplan-importer/planraster.py). numpy only."""
import collections
import importlib.util
import math
import pathlib
import sys

import numpy as np
import pytest

_DOCKER = pathlib.Path(__file__).resolve().parents[2] / 'docker'


def _load(rel, name):
    spec = importlib.util.spec_from_file_location(name, _DOCKER / rel)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


pr = _load('floorplan-importer/planraster.py', 'fp_planraster')
gs = _load('fleet-status/gridstore.py', 'fp_gridstore')

RES = 0.05
IDENT = (1.0, np.eye(2), np.zeros(2))


def rect(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]], dtype=float)


def rot(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def room(width=0.2, gap=None):
    """10 x 6 m room; `gap` = (x0, x1) removes that span from the bottom wall."""
    if gap is None:
        return pr.Geometry(polylines=(pr.Polyline(rect(0, 0, 10, 6), width),))
    lower_left = np.array([[0.0, 0.0], [gap[0], 0.0]])
    rest = np.array([[gap[1], 0.0], [10, 0], [10, 6], [0, 6], [0, 0]])
    return pr.Geometry(polylines=(pr.Polyline(lower_left, width), pr.Polyline(rest, width)))


class View:
    def __init__(self, grid, meta):
        self.grid = grid
        self.res = meta['resolution']
        self.ox, self.oy = meta['origin'][:2]
        self.h = grid.shape[0]

    def rc(self, x, y):
        return (self.h - 1 - int(math.floor((y - self.oy) / self.res)),
                int(math.floor((x - self.ox) / self.res)))

    def at(self, x, y):
        return int(self.grid[self.rc(x, y)])

    def centres(self, value):
        rows, cols = np.nonzero(self.grid == value)
        return (self.ox + (cols + 0.5) * self.res,
                self.oy + (self.h - 1 - rows + 0.5) * self.res)


def runs(line, value):
    out, n = [], 0
    for v in line:
        if v == value:
            n += 1
        elif n:
            out.append(n)
            n = 0
    if n:
        out.append(n)
    return out


def reach(passable, start):
    seen = np.zeros_like(passable)
    seen[start] = True
    todo = collections.deque([start])
    h, w = passable.shape
    while todo:
        r, c = todo.popleft()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < h and 0 <= cc < w and passable[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                todo.append((rr, cc))
    return seen


def components(mask):
    left, n = mask.copy(), 0
    while left.any():
        r, c = np.argwhere(left)[0]
        left &= ~reach(left, (r, c))
        n += 1
    return n


def free_count(grid):
    return int((grid == pr.FREE).sum())


# --- happy path: ring ---------------------------------------------------------------

def test_rectangle_ring_is_four_cells_thick_with_free_inside_unknown_outside():
    grid, meta = pr.rasterize(room(), IDENT, RES)
    v = View(grid, meta)
    assert grid.dtype == np.uint8
    assert set(np.unique(grid)) == {pr.OCCUPIED, pr.FREE, pr.UNKNOWN}
    assert runs(grid[v.rc(5, 3)[0]], pr.OCCUPIED) == [4, 4]
    assert runs(grid[:, v.rc(5, 3)[1]], pr.OCCUPIED) == [4, 4]
    assert v.at(5, 3) == pr.FREE
    assert v.at(0.3, 0.3) == pr.FREE
    for x, y in ((-0.5, 3), (10.5, 3), (5, -0.5), (5, 6.5), (-1, -1), (11, 7)):
        assert v.at(x, y) == pr.UNKNOWN, (x, y)
    assert free_count(grid) == round(9.8 / RES) * round(5.8 / RES)
    assert abs(int((grid == pr.OCCUPIED).sum()) - 2560) <= 20


def test_free_cells_all_lie_inside_the_wall_ring():
    grid, meta = pr.rasterize(room(), IDENT, RES)
    xs, ys = View(grid, meta).centres(pr.FREE)
    assert xs.min() > 0.1 and xs.max() < 9.9 and ys.min() > 0.1 and ys.max() < 5.9


def test_coarser_resolution_scales_wall_thickness():
    grid, meta = pr.rasterize(room(), IDENT, 0.1)
    v = View(grid, meta)
    assert runs(grid[v.rc(5, 3)[0]], pr.OCCUPIED) == [2, 2]
    assert meta['resolution'] == 0.1


def test_transform_scales_translates_and_the_yaml_origin_follows():
    pts = rect(100, 200, 1100, 800)
    geo = pr.Geometry(polylines=(pr.Polyline(pts, 20),))
    grid, meta = pr.rasterize(geo, (0.01, np.eye(2), np.array([1.0, 1.0])), RES)
    v = View(grid, meta)
    xs, ys = v.centres(pr.OCCUPIED)
    assert xs.min() == pytest.approx(1.9, abs=RES) and xs.max() == pytest.approx(12.1, abs=RES)
    assert ys.min() == pytest.approx(2.9, abs=RES) and ys.max() == pytest.approx(9.1, abs=RES)
    assert v.at(7, 6) == pr.FREE
    ox, oy, yaw = meta['origin']
    assert ox < 1.9 < ox + 2 and oy < 2.9 < oy + 2 and yaw == 0.0
    assert ox / RES == pytest.approx(round(ox / RES), abs=1e-4)


def test_polyline_corner_join_has_no_gap():
    geo = pr.Geometry(polylines=(pr.Polyline(np.array([[0, 0], [3, 1.3], [5, -1]]), 0.2),))
    grid, _ = pr.rasterize(geo, IDENT, RES)
    assert components(grid == pr.OCCUPIED) == 1


# --- no dropouts -------------------------------------------------------------------

def test_hairline_walls_cover_every_cell_they_touch():
    rng = np.random.default_rng(1234)
    for _ in range(12):
        a, b = rng.uniform(0, 6, size=(2, 2))
        width = float(rng.choice([0.0, 0.002, 0.01, 0.03]))
        grid, meta = pr.rasterize(
            pr.Geometry(polylines=(pr.Polyline(np.array([a, b]), width),)), IDENT, RES)
        v = View(grid, meta)
        for t in np.linspace(0, 1, 400):
            x, y = a + t * (b - a)
            assert v.at(x, y) == pr.OCCUPIED, (a, b, width, t)
        assert components(grid == pr.OCCUPIED) == 1


def test_wall_thinner_than_a_cell_keeps_every_column():
    geo = pr.Geometry(polylines=(pr.Polyline(np.array([[0, 0.025], [5, 0.025]]), 0.004),))
    grid, meta = pr.rasterize(geo, IDENT, RES)
    v = View(grid, meta)
    r0 = v.rc(0, 0.025)[0]
    for x in np.arange(0.025, 5, RES):
        assert grid[r0, v.rc(x, 0)[1]] == pr.OCCUPIED
    assert runs(grid[:, v.rc(2.5, 0)[1]], pr.OCCUPIED) == [1]


@pytest.mark.parametrize('points', [
    [[1.0, 1.0], [1.01, 1.0]], [[1.0, 1.0]], [[1.0, 1.0], [1.0, 1.0]],
])
def test_sub_cell_segment_or_point_still_marks_a_cell(points):
    grid, _ = pr.rasterize(pr.Geometry(polylines=(pr.Polyline(np.array(points), 0.0),)),
                           IDENT, RES)
    assert int((grid == pr.OCCUPIED).sum()) >= 1


# --- rotation -----------------------------------------------------------------------

def test_rotated_rectangle_is_a_closed_ring():
    geo = pr.Geometry(polylines=(pr.Polyline(rect(-5, -3, 5, 3), 0.2),))
    grid, meta = pr.rasterize(geo, (1.0, rot(10), np.zeros(2)), RES)
    v = View(grid, meta)
    outside = reach(grid != pr.OCCUPIED, (0, 0))
    assert not outside[v.rc(0, 0)]
    assert v.at(0, 0) == pr.FREE
    assert v.at(-6.2, 0) == pr.UNKNOWN and v.at(0, -4.6) == pr.UNKNOWN
    assert components(grid == pr.OCCUPIED) == 1
    assert free_count(grid) == pytest.approx(9.8 * 5.8 / RES**2, rel=0.02)


# --- review fix 1: doorways ----------------------------------------------------------

def test_door_gap_stays_free_while_interior_is_free_and_exterior_unknown():
    grid, meta = pr.rasterize(room(gap=(4.0, 4.9)), IDENT, RES)
    v = View(grid, meta)
    assert reach(grid != pr.OCCUPIED, (0, 0))[v.rc(5, 3)], 'the raw wall mask leaks'
    assert v.at(5, 3) == pr.FREE
    assert v.at(4.45, 0.15) == pr.FREE
    for x in np.arange(4.025, 4.9, RES):
        for y in (-0.075, -0.025, 0.025, 0.075):
            assert v.at(x, y) == pr.FREE, (x, y)
    assert v.at(3.95, 0.0) == pr.OCCUPIED and v.at(4.95, 0.0) == pr.OCCUPIED
    for x, y in ((4.45, -0.15), (4.45, -0.5), (4.45, -1.0), (-1, -1), (11, 7)):
        assert v.at(x, y) == pr.UNKNOWN, (x, y)
    assert free_count(grid) == pytest.approx(22736 + 72, abs=4)


def test_gap_cells_are_not_occupied_in_the_returned_layer():
    grid, meta = pr.rasterize(room(gap=(4.0, 4.9)), IDENT, RES)
    v = View(grid, meta)
    assert not (grid[v.rc(4.45, 0.075)[0] : v.rc(4.45, -0.075)[0] + 1,
                     v.rc(4.05, 0)[1] : v.rc(4.85, 0)[1] + 1] == pr.OCCUPIED).any()


@pytest.mark.parametrize('degrees', [10, 30, 45])
def test_door_gap_in_a_rotated_building(degrees):
    transform = (1.0, rot(degrees), np.zeros(2))
    grid, meta = pr.rasterize(room(gap=(4.0, 4.9)), transform, RES)
    v = View(grid, meta)

    def world(x, y):
        return transform[1] @ np.array([x, y])

    assert v.at(*world(5, 3)) == pr.FREE
    assert v.at(*world(4.45, 0.0)) == pr.FREE
    assert v.at(*world(4.45, -0.5)) == pr.UNKNOWN
    assert v.at(*world(4.45, -1.0)) == pr.UNKNOWN
    assert free_count(grid) == pytest.approx(22808, rel=0.03)


def test_gap_wider_than_the_closing_length_leaks_and_close_m_widens_it():
    wide = room(gap=(4.0, 6.0))
    grid, meta = pr.rasterize(wide, IDENT, RES)
    assert View(grid, meta).at(5, 3) == pr.UNKNOWN
    grid, meta = pr.rasterize(wide, IDENT, RES, close_m=2.4)
    assert View(grid, meta).at(5, 3) == pr.FREE


def test_walls_flush_to_an_open_face_leave_no_free_skirt_outside():
    grid, meta = pr.rasterize(room(gap=(4.0, 4.9)), (1.0, rot(20), np.zeros(2)), RES)
    xs, ys = View(grid, meta).centres(pr.FREE)
    local = np.linalg.inv(rot(20)) @ np.vstack([xs, ys])
    outside = (local[0] < -0.0) | (local[0] > 10.0) | (local[1] < -0.1) | (local[1] > 6.0)
    assert outside.sum() <= 12


# --- review fix 2: cell cap ----------------------------------------------------------

def test_huge_extent_is_refused_before_allocating():
    geo = pr.Geometry(polylines=(pr.Polyline(np.array([[0, 0], [250, 250]]), 0.2),))
    with pytest.raises(pr.GridTooLargeError, match='cap'):
        pr.rasterize(geo, IDENT, RES)


def test_cap_is_the_named_constant_and_boundary_is_inclusive(monkeypatch):
    assert pr.MAX_CELLS == 4000 * 4000
    grid, _ = pr.rasterize(room(), IDENT, RES)
    cells = grid.shape[0] * grid.shape[1]
    monkeypatch.setattr(pr, 'MAX_CELLS', cells)
    pr.rasterize(room(), IDENT, RES)
    monkeypatch.setattr(pr, 'MAX_CELLS', cells - 1)
    with pytest.raises(pr.GridTooLargeError):
        pr.rasterize(room(), IDENT, RES)


def test_too_large_error_is_a_value_error():
    assert issubclass(pr.GridTooLargeError, pr.PlanRasterError)
    assert issubclass(pr.PlanRasterError, ValueError)


# --- filled polygons -----------------------------------------------------------------

def test_filled_polygon_is_occupied_on_its_own_footprint():
    geo = pr.Geometry(fills=(pr.Fill(rings=(rect(2, 2, 3, 3)[:-1],)),))
    grid, meta = pr.rasterize(geo, IDENT, RES)
    assert int((grid == pr.OCCUPIED).sum()) == 400
    assert View(grid, meta).at(2.5, 2.5) == pr.OCCUPIED


def test_two_rings_fill_even_odd_so_the_inner_ring_is_a_hole():
    geo = pr.Geometry(fills=(pr.Fill(rings=(rect(0, 0, 4, 4)[:-1], rect(1, 1, 3, 3)[:-1])),))
    grid, meta = pr.rasterize(geo, IDENT, RES)
    v = View(grid, meta)
    assert int((grid == pr.OCCUPIED).sum()) == 4800
    assert v.at(0.5, 2) == pr.OCCUPIED
    assert v.at(2, 2) == pr.FREE
    assert v.at(5, 5) == pr.UNKNOWN


def test_filled_polygon_thinner_than_a_cell_is_not_dropped():
    geo = pr.Geometry(fills=(pr.Fill(rings=(rect(0, 0.021, 3, 0.029)[:-1],)),))
    grid, meta = pr.rasterize(geo, IDENT, RES)
    v = View(grid, meta)
    for x in np.arange(0.025, 3, RES):
        assert v.at(x, 0.025) == pr.OCCUPIED


def test_fill_and_stroke_combine():
    geo = pr.Geometry(polylines=room().polylines, fills=(pr.Fill(rings=(rect(4, 2, 5, 3)[:-1],)),))
    grid, meta = pr.rasterize(geo, IDENT, RES)
    assert View(grid, meta).at(4.5, 2.5) == pr.OCCUPIED
    assert View(grid, meta).at(7, 3) == pr.FREE


# --- output format ---------------------------------------------------------------------

def test_pgm_and_yaml_load_like_a_saved_map(tmp_path):
    grid, meta = pr.rasterize(room(gap=(4.0, 4.9)), IDENT, RES)
    meta['image'] = 'plan.pgm'
    (tmp_path / 'plan.pgm').write_bytes(pr.encode_pgm(grid))
    (tmp_path / 'plan.yaml').write_text(pr.yaml_text(meta))
    w, h, body = gs.read_pgm(str(tmp_path / 'plan.pgm'))
    assert (w, h) == (grid.shape[1], grid.shape[0])
    assert body == grid.tobytes()
    parsed = gs.read_map_yaml(str(tmp_path / 'plan.yaml'))
    assert parsed['image'] == 'plan.pgm'
    assert parsed['resolution'] == RES
    assert parsed['origin'] == [meta['origin'][0], meta['origin'][1], 0.0]
    assert (parsed['negate'], parsed['occupied_thresh'], parsed['free_thresh']) == (
        0, pr.OCCUPIED_THRESH, pr.FREE_THRESH)
    occ = gs.load_saved_map(str(tmp_path / 'plan.yaml'))
    data = np.array(occ['data']).reshape(h, w)
    top_first = np.flipud(data)
    assert ((top_first == 100) == (grid == pr.OCCUPIED)).all()
    assert ((top_first == 0) == (grid == pr.FREE)).all()
    assert ((top_first == -1) == (grid == pr.UNKNOWN)).all()
    assert (occ['origin_x'], occ['origin_y']) == (meta['origin'][0], meta['origin'][1])


def test_yaml_has_the_keys_nav2_map_server_needs():
    _, meta = pr.rasterize(room(), IDENT, RES)
    text = pr.yaml_text(meta)
    for line in ('image: map.pgm', 'mode: trinary', 'resolution: 0.05', 'negate: 0',
                 'occupied_thresh: 0.65', 'free_thresh: 0.196'):
        assert line in text.splitlines()
    assert text.splitlines()[3].startswith('origin: [') and text.splitlines()[3].endswith(', 0.0]')


def test_pgm_header_is_p5_and_row_zero_is_the_top():
    grid = np.array([[pr.OCCUPIED, pr.FREE, pr.UNKNOWN], [pr.FREE, pr.FREE, pr.FREE]],
                    dtype=np.uint8)
    pgm = pr.encode_pgm(grid)
    assert pgm.startswith(b'P5\n3 2\n255\n')
    assert pgm[len(b'P5\n3 2\n255\n'):] == bytes([0, 254, 205, 254, 254, 254])


def test_each_pixel_class_reads_back_as_itself_under_nav2_trinary_thresholds():
    _, meta = pr.rasterize(room(), IDENT, RES)

    def nav2(pixel):
        p = (255 - pixel) / 255.0
        if p > meta['occupied_thresh']:
            return 'occupied'
        return 'free' if p < meta['free_thresh'] else 'unknown'

    assert nav2(pr.OCCUPIED) == 'occupied'
    assert nav2(pr.FREE) == 'free'
    assert nav2(pr.UNKNOWN) == 'unknown'


def test_same_input_gives_identical_bytes():
    a, ma = pr.rasterize(room(gap=(4.0, 4.9)), (1.0, rot(17), np.array([2.0, 1.0])), RES)
    b, mb = pr.rasterize(room(gap=(4.0, 4.9)), (1.0, rot(17), np.array([2.0, 1.0])), RES)
    assert pr.encode_pgm(a) == pr.encode_pgm(b) and ma == mb


# --- errors ----------------------------------------------------------------------------

@pytest.mark.parametrize('bad', [0, -0.05, float('nan'), float('inf'), 'x', None])
def test_bad_resolution_is_refused(bad):
    with pytest.raises(pr.PlanRasterError):
        pr.rasterize(room(), IDENT, bad)


def test_empty_geometry_is_refused():
    with pytest.raises(pr.PlanRasterError, match='no geometry'):
        pr.rasterize(pr.Geometry(), IDENT, RES)


@pytest.mark.parametrize('transform', [
    (0.0, np.eye(2), np.zeros(2)),
    (-1.0, np.eye(2), np.zeros(2)),
    (float('nan'), np.eye(2), np.zeros(2)),
    (1.0, np.eye(3), np.zeros(2)),
    (1.0, np.eye(2), np.zeros(3)),
    (1.0, np.eye(2)),
])
def test_bad_transform_is_refused(transform):
    with pytest.raises(pr.PlanRasterError):
        pr.rasterize(room(), transform, RES)


@pytest.mark.parametrize('polyline', [
    pr.Polyline(np.array([[0, 0], [float('nan'), 1]]), 0.1),
    pr.Polyline(np.array([[0, 0], [1, float('inf')]]), 0.1),
    pr.Polyline(np.array([0.0, 1.0, 2.0]), 0.1),
    pr.Polyline(np.zeros((0, 2)), 0.1),
    pr.Polyline(np.array([[0, 0], [1, 1]]), -0.1),
    pr.Polyline(np.array([[0, 0], [1, 1]]), float('nan')),
])
def test_bad_polyline_is_refused(polyline):
    with pytest.raises(pr.PlanRasterError):
        pr.rasterize(pr.Geometry(polylines=(polyline,)), IDENT, RES)


def test_degenerate_fill_is_refused():
    with pytest.raises(pr.PlanRasterError):
        pr.rasterize(pr.Geometry(fills=(pr.Fill(rings=(np.array([[0, 0], [1, 1]]),)),)),
                     IDENT, RES)
    with pytest.raises(pr.PlanRasterError):
        pr.rasterize(pr.Geometry(fills=(pr.Fill(rings=()),)), IDENT, RES)


def test_non_positive_closing_length_is_refused():
    with pytest.raises(pr.PlanRasterError):
        pr.rasterize(room(), IDENT, RES, close_m=0)
