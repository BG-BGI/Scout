"""IFC storeys -> plan geometry (docker/floorplan-importer/ifcsource.py). Needs ifcopenshell."""
import importlib.util
import math
import pathlib
import sys

import numpy as np
import pytest

ifcopenshell = pytest.importorskip('ifcopenshell')
pytest.importorskip('ifcopenshell.geom')
pytest.importorskip('ifcopenshell.util.element')
shapely = pytest.importorskip('shapely')

_REPO = pathlib.Path(__file__).resolve().parents[2]
FIXTURES = pathlib.Path(__file__).resolve().parent / 'fixtures'
FIXTURE = FIXTURES / 'two_storey.ifc'


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


_IMPORTER = _REPO / 'docker' / 'floorplan-importer'
pr = _load(_IMPORTER / 'planraster.py', 'planraster')
calib = _load(_IMPORTER / 'calib.py', 'fp_calib')
ifs = _load(_IMPORTER / 'ifcsource.py', 'fp_ifcsource')
gs = _load(_REPO / 'docker' / 'fleet-status' / 'gridstore.py', 'fp_gridstore')
gen = _load(FIXTURES / 'make_ifc_fixture.py', 'fp_make_ifc_fixture')

RES = 0.05
TALL_WALL_XY = (1.5, 1.5)
DOORWAY_YS = tuple(np.arange(1.65, 2.5, RES))


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

    def row(self, y):
        return self.grid[self.rc(0, y)[0]]

    def col(self, x):
        return self.grid[:, self.rc(x, 0)[1]]


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


def connected(passable, a, b):
    seen = np.zeros_like(passable)
    seen[a] = True
    todo = [a]
    while todo:
        r, c = todo.pop()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if passable[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                todo.append((rr, cc))
    return bool(seen[b])


def raster(sl, resolution=RES):
    return pr.rasterize(sl.geometry, calib.map_frame(1.0, 0.0, (0.0, 0.0)), resolution)


def view(sl):
    return View(*raster(sl))


def area(sl):
    return sum(shapely.Polygon(f.rings[0], f.rings[1:]).area for f in sl.geometry.fills)


def covers(sl, x, y):
    point = shapely.Point(x, y)
    return any(shapely.Polygon(f.rings[0], f.rings[1:]).contains(point) for f in sl.geometry.fills)


def ring_key(sl):
    return [tuple(map(tuple, ring.round(9))) for f in sl.geometry.fills for ring in f.rings]


def reopen(f):
    return ifs.open_model(f.to_string().encode())


def fixture_file():
    return ifcopenshell.file.from_string(FIXTURE.read_text())


def floors(m):
    return [(s.floor, s.label, round(s.elevation_m, 6)) for s in ifs.list_storeys(m)]


@pytest.fixture(scope='module')
def model():
    return ifs.open_model(FIXTURE)


@pytest.fixture(scope='module')
def levels(model):
    return {s.label: s for s in ifs.list_storeys(model)}


@pytest.fixture(scope='module')
def l1(model, levels):
    return ifs.slice_storey(model, levels['Level 1'])


@pytest.fixture(scope='module')
def l2(model, levels):
    return ifs.slice_storey(model, levels['Level 2'])


# --- storeys (AE5) ---------------------------------------------------------------------

def test_three_levels_listed_in_metres_with_floor_numbers(model):
    assert floors(model) == [(1, 'Level 1', 0.0), (2, 'Level 2', 3.0), (3, 'Level 3', 6.0)]
    ids = [s.id for s in ifs.list_storeys(model)]
    assert len(set(ids)) == 3 and all(isinstance(i, int) for i in ids)
    assert all(s.global_id for s in ifs.list_storeys(model))


def test_storeys_are_ordered_by_elevation_not_file_order():
    f = fixture_file()
    level1 = next(s for s in f.by_type('IfcBuildingStorey') if s.Name == 'Level 1')
    level1.ObjectPlacement.RelativePlacement.Location.Coordinates = (0.0, 0.0, 9.0)
    assert floors(reopen(f)) == [(1, 'Level 2', 3.0), (2, 'Level 3', 6.0), (3, 'Level 1', 9.0)]


def test_blank_names_fall_back_to_long_name_then_a_numbered_label():
    f = fixture_file()
    by_name = {s.Name: s for s in f.by_type('IfcBuildingStorey')}
    by_name['Level 2'].Name = ''
    by_name['Level 3'].Name = None
    by_name['Level 3'].LongName = 'Roof'
    got = ifs.list_storeys(reopen(f))
    assert [s.label for s in got] == ['Level 1', 'Storey 2', 'Roof']
    assert [s.name for s in got] == ['Level 1', '', '']


def test_two_selected_levels_give_two_maps_with_floors_1_and_2(model):
    picked = ifs.list_storeys(model)[:2]
    assert [s.floor for s in picked] == [1, 2]
    grids = [raster(ifs.slice_storey(model, s))[0] for s in picked]
    assert len(grids) == 2 and all(int((g == pr.OCCUPIED).sum()) > 0 for g in grids)


def test_storey_can_be_selected_by_object_or_by_id(model, levels):
    by_obj = ifs.slice_storey(model, levels['Level 2'])
    by_id = ifs.slice_storey(model, levels['Level 2'].id)
    assert ring_key(by_obj) == ring_key(by_id) and by_obj.storey == by_id.storey


# --- units: scale applied once -----------------------------------------------------------

@pytest.mark.parametrize('unit,scale', [('mm', 0.001), ('ft', 0.3048)])
def test_non_metre_models_read_like_the_metre_model(unit, scale, l1, l2):
    m = reopen(gen.build(unit))
    assert m.unit_scale == pytest.approx(scale)
    assert floors(m) == [(1, 'Level 1', 0.0), (2, 'Level 2', 3.0), (3, 'Level 3', 6.0)]
    for ref, storey in zip((l1, l2), ifs.list_storeys(m)[:2], strict=True):
        got = ifs.slice_storey(m, storey)
        assert got.z == pytest.approx(ref.z, abs=1e-6)
        assert got.bounds == pytest.approx(ref.bounds, abs=1e-5)
        assert area(got) == pytest.approx(area(ref), abs=1e-4)


def test_millimetre_walls_are_neither_1000x_big_nor_1000x_small():
    m = reopen(gen.build('mm'))
    sl = ifs.slice_storey(m, ifs.list_storeys(m)[0])
    assert sl.bounds == pytest.approx((-0.1, -0.1, 6.1, 4.1), abs=1e-6)


def test_elevation_falls_back_to_the_attribute_and_is_scaled_once():
    f = ifcopenshell.file.from_string(gen.build('mm').to_string())
    level2 = next(s for s in f.by_type('IfcBuildingStorey') if s.Name == 'Level 2')
    assert level2.Elevation == 3000.0
    level2.ObjectPlacement = None
    assert floors(reopen(f))[1] == (2, 'Level 2', 3.0)


def test_elevation_and_geometry_follow_the_building_placement(l2):
    f = fixture_file()
    f.by_type('IfcBuilding')[0].ObjectPlacement.RelativePlacement.Location.Coordinates = (
        0.0, 0.0, 100.0)
    m = reopen(f)
    assert [round(s.elevation_m, 6) for s in ifs.list_storeys(m)] == [100.0, 103.0, 106.0]
    got = ifs.slice_storey(m, ifs.list_storeys(m)[1])
    assert got.z == pytest.approx(103.241)
    assert got.bounds == pytest.approx(l2.bounds, abs=1e-6)


# --- slicing: walls ------------------------------------------------------------------------

def test_wall_of_0_2_m_is_four_cells_thick_interior_free_exterior_unknown(l1):
    v = view(l1)
    assert runs(v.row(3.2), pr.OCCUPIED) == [4, 4, 4]
    assert runs(v.col(4.5), pr.OCCUPIED) == [4, 4]
    assert v.at(1.0, 3.0) == pr.FREE and v.at(4.5, 2.0) == pr.FREE
    for x, y in ((-0.6, 2.0), (6.6, 2.0), (3.0, -0.6), (3.0, 4.6), (-1, -1)):
        assert v.at(x, y) == pr.UNKNOWN, (x, y)
    xs = np.nonzero(v.grid == pr.FREE)[1] * v.res + v.ox
    assert xs.min() > 0.0 and xs.max() < 6.0


def test_geometry_is_metric_y_up_fills_with_holes_in_the_ifc_world_frame(l1):
    assert l1.bounds == pytest.approx((-0.1, -0.1, 6.1, 4.1), abs=1e-6)
    assert l1.z == pytest.approx(0.241)
    assert l1.geometry.polylines == ()
    assert all(isinstance(f, pr.Fill) for f in l1.geometry.fills)
    ring_counts = sorted(len(f.rings) for f in l1.geometry.fills)
    assert ring_counts == [1, 2]
    assert area(l1) == pytest.approx(6.2 * 4.2 - 5.8 * 3.8 + 0.2 * 1.0 + 0.2 * 3.8 - 0.2 * 0.9,
                                     abs=1e-4)


def test_storey_with_nothing_at_the_slice_height_is_refused_by_name(model, levels):
    with pytest.raises(ifs.EmptySliceError, match='Level 3') as info:
        ifs.slice_storey(model, levels['Level 3'])
    assert info.value.storey == levels['Level 3']
    assert info.value.z == pytest.approx(6.241)
    assert isinstance(info.value, ifs.IfcSourceError) and isinstance(info.value, ValueError)


def test_changing_the_slice_height_reslices(model, levels, l1):
    high = ifs.slice_storey(model, levels['Level 1'], slice_height=4.5)
    assert high.z == pytest.approx(4.5)
    assert high.bounds == pytest.approx((-0.1, -0.1, 4.1, 3.1), abs=1e-6)
    assert high.bounds != l1.bounds


def test_a_slice_above_everything_is_empty_and_a_lower_one_recovers(model, levels):
    with pytest.raises(ifs.EmptySliceError, match='Level 2'):
        ifs.slice_storey(model, levels['Level 2'], slice_height=3.5)
    assert ifs.slice_storey(model, levels['Level 2'], slice_height=2.0).bounds[2] > 4.0


def test_wall_crossing_two_storeys_shows_on_both_slices(model, l1, l2):
    tall = next(w for w in model.file.by_type('IfcWall') if w.Name == 'Tall wall')
    assert ifcopenshell.util.element.get_container(tall).Name == 'Level 1'
    assert view(l1).at(*TALL_WALL_XY) == pr.OCCUPIED
    assert view(l2).at(*TALL_WALL_XY) == pr.OCCUPIED


def test_wall_that_ends_below_the_plane_is_absent_from_the_upper_slice(l1, l2):
    assert view(l1).at(3.0, 3.0) == pr.OCCUPIED
    assert view(l2).at(3.0, 2.8) == pr.FREE


# --- doors, windows ----------------------------------------------------------------------

def test_doorway_stays_free_and_both_rooms_are_free(l1):
    v = view(l1)
    assert v.at(3.0, 3.0) == pr.OCCUPIED
    for y in DOORWAY_YS:
        for x in (2.925, 2.975, 3.025, 3.075):
            assert v.at(x, y) == pr.FREE, (x, y)
    assert v.at(2.2, 2.0) == pr.FREE and v.at(3.8, 2.0) == pr.FREE
    free = v.grid == pr.FREE
    assert connected(free, v.rc(1.0, 3.0), v.rc(4.5, 1.0))


def test_door_leaf_is_solid_at_lidar_height_so_the_default_leaves_doors_out(model, levels, l1):
    with_doors = ifs.slice_storey(model, levels['Level 1'],
                                  include=ifs.DEFAULT_INCLUDE + ('IfcDoor',))
    assert 'IfcDoor' not in ifs.DEFAULT_INCLUDE
    assert covers(with_doors, 2.96, 2.0)
    assert not covers(l1, 2.96, 2.0)
    assert with_doors.faces > l1.faces


def test_opening_subtraction_is_applied_so_the_partition_has_a_gap(model):
    part = next(w for w in model.file.by_type('IfcWall') if w.Name == 'L1 Partition')
    settings = ifcopenshell.geom.settings()
    settings.set('use-world-coords', True)
    voided = ifcopenshell.geom.create_shape(settings, part).geometry.faces
    settings.set('disable-opening-subtractions', True)
    solid = ifcopenshell.geom.create_shape(settings, part).geometry.faces
    assert len(voided) > len(solid)


def test_window_with_a_high_sill_leaves_the_wall_solid_at_lidar_height(l2):
    v = view(l2)
    assert runs(v.col(2.0), pr.OCCUPIED) == [4, 4]
    assert v.at(2.0, 3.0) == pr.OCCUPIED
    assert covers(l2, 2.0, 3.0)


def test_window_appears_only_when_the_plane_crosses_it(model, levels):
    glass = (2.0, 2.893)
    mid = ifs.slice_storey(model, levels['Level 2'], slice_height=1.2)
    assert not covers(mid, 2.0, 3.0)
    assert covers(mid, *glass)
    without = ifs.slice_storey(model, levels['Level 2'], slice_height=1.2, include=('IfcWall',))
    assert not covers(without, 2.0, 3.0) and not covers(without, *glass)
    assert without.faces < mid.faces


def test_exclude_wins_over_include_and_an_empty_include_is_an_empty_slice(model, levels):
    with pytest.raises(ifs.EmptySliceError):
        ifs.slice_storey(model, levels['Level 1'], include=('IfcWall',), exclude=('IfcWall',))
    with pytest.raises(ifs.EmptySliceError):
        ifs.slice_storey(model, levels['Level 1'], include=())


# --- section maths (private, synthetic meshes) ---------------------------------------------

def box(x0, y0, z0, x1, y1, z1, inward=False):
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], dtype=float)
    t = np.array([[0, 3, 2], [0, 2, 1], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                  [3, 7, 6], [3, 6, 2], [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5]])
    return v, t[:, [0, 2, 1]] if inward else t


def merged(*boxes):
    verts = np.vstack([b[0] for b in boxes])
    offsets = np.cumsum([0] + [len(b[0]) for b in boxes[:-1]])
    return verts, np.vstack([b[1] + o for b, o in zip(boxes, offsets, strict=True)])


def section_area(mesh, z):
    polys = ifs._element_polygons(*mesh, z)
    return shapely.unary_union(polys).area if polys else 0.0


def test_section_of_a_box_is_its_footprint():
    assert section_area(box(0, 0, 0, 2, 3, 1), 0.5) == pytest.approx(6.0)


def test_section_misses_a_box_outside_the_plane():
    assert section_area(box(0, 0, 0, 2, 3, 1), 1.5) == 0.0


def test_overlapping_shells_in_one_element_union_rather_than_cancel():
    mesh = merged(box(0, 0, 0, 2, 2, 1), box(1, 1, 0, 3, 3, 1))
    assert section_area(mesh, 0.5) == pytest.approx(4 + 4 - 1)


def test_inward_facing_inner_shell_is_a_hole():
    mesh = merged(box(0, 0, 0, 4, 4, 1), box(1, 1, 0, 3, 3, 1, inward=True))
    assert section_area(mesh, 0.5) == pytest.approx(16 - 4)


def test_solid_is_present_from_its_bottom_face_up_to_but_excluding_its_top():
    assert section_area(box(0, 0, 0, 2, 3, 1), 0.0) == pytest.approx(6.0)
    assert section_area(box(0, 0, 0, 2, 3, 1), 1.0) == 0.0


def test_slanted_diagonals_do_not_open_the_loop():
    v, t = box(0.1234567, 0.7654321, 0.0, 5.333333, 2.718281, 3.1415926)
    for z in (0.241, 1.0, 3.0):
        assert section_area((v, t), z) == pytest.approx(
            (5.333333 - 0.1234567) * (2.718281 - 0.7654321))


def test_pieces_that_meet_within_float_noise_merge_into_one_polygon():
    left = shapely.box(0, 0, 2, 1)
    right = shapely.box(2 + 1e-12, 0, 4, 1)
    assert len(ifs._union([left, right])) == 1


# --- errors --------------------------------------------------------------------------------

GOOD = FIXTURE.read_bytes()


@pytest.mark.parametrize('data', [
    b'',
    b'hello world',
    b'\x00\x01\x02 not an ifc',
    b'PK\x03\x04' + b'\x00' * 40,
    GOOD[:len(GOOD) // 2],
    GOOD.replace(b"FILE_SCHEMA(('IFC4'))", b"FILE_SCHEMA(('IFC9'))"),
    b'ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n',
], ids=['empty', 'text', 'binary', 'zip', 'truncated', 'bad-schema', 'no-schema'])
def test_unparsable_bytes_raise_a_typed_parse_error(data):
    with pytest.raises(ifs.IfcParseError):
        ifs.open_model(data)


def test_truncated_file_is_named_as_truncated():
    with pytest.raises(ifs.IfcParseError, match='truncated'):
        ifs.open_model(GOOD[:len(GOOD) // 2])


def test_bytearray_and_memoryview_are_accepted():
    for wrap in (bytearray, memoryview):
        assert len(ifs.list_storeys(ifs.open_model(wrap(GOOD)))) == 3


def test_paths_str_or_pathlike_open_and_bad_paths_are_parse_errors(tmp_path):
    assert len(ifs.list_storeys(ifs.open_model(str(FIXTURE)))) == 3
    assert len(ifs.list_storeys(ifs.open_model(FIXTURE))) == 3
    with pytest.raises(ifs.IfcParseError):
        ifs.open_model(tmp_path / 'missing.ifc')
    junk = tmp_path / 'junk.ifc'
    junk.write_bytes(b'not ifc')
    with pytest.raises(ifs.IfcParseError):
        ifs.open_model(junk)
    cut = tmp_path / 'cut.ifc'
    cut.write_bytes(GOOD[:len(GOOD) // 2])
    with pytest.raises(ifs.IfcParseError, match='truncated'):
        ifs.open_model(cut)


@pytest.mark.parametrize('bad', [42, None, 1.5])
def test_wrong_source_type_is_refused(bad):
    with pytest.raises(ifs.IfcSourceError):
        ifs.open_model(bad)


def test_size_cap_is_the_named_constant_and_the_boundary_is_inclusive(tmp_path):
    assert ifs.MAX_IFC_BYTES == 256 * 1024 * 1024
    assert len(ifs.list_storeys(ifs.open_model(GOOD, max_bytes=len(GOOD)))) == 3
    with pytest.raises(ifs.FileTooLargeError) as info:
        ifs.open_model(GOOD, max_bytes=len(GOOD) - 1)
    assert (info.value.count, info.value.limit) == (len(GOOD), len(GOOD) - 1)
    assert isinstance(info.value, ifs.IfcTooLargeError)
    with pytest.raises(ifs.FileTooLargeError):
        ifs.open_model(FIXTURE, max_bytes=len(GOOD) - 1)


@pytest.mark.parametrize('bad', [0, -1, 1.5, float('nan'), 'x', None])
def test_bad_size_cap_is_refused(bad):
    with pytest.raises(ifs.IfcSourceError):
        ifs.open_model(GOOD, max_bytes=bad)


def test_model_without_storeys_is_refused():
    with pytest.raises(ifs.NoStoreysError):
        reopen(gen.Builder('m').f)


def test_face_budget_failure_carries_the_count(model, levels, l1):
    with pytest.raises(ifs.FaceBudgetError) as info:
        ifs.slice_storey(model, levels['Level 1'], face_budget=10)
    assert info.value.limit == 10 and info.value.count > 10
    assert str(info.value.count) in str(info.value)
    assert isinstance(info.value, ifs.IfcTooLargeError)


def test_face_budget_boundary_is_inclusive_and_reports_the_total(model, levels, l1):
    assert l1.faces > 0
    ifs.slice_storey(model, levels['Level 1'], face_budget=l1.faces)
    with pytest.raises(ifs.FaceBudgetError) as info:
        ifs.slice_storey(model, levels['Level 1'], face_budget=l1.faces - 1)
    assert info.value.count == l1.faces and info.value.limit == l1.faces - 1


def test_face_budget_default_is_the_named_constant():
    assert ifs.DEFAULT_FACE_BUDGET == 3_000_000


def test_unknown_storey_is_a_typed_error(model):
    with pytest.raises(ifs.UnknownStoreyError) as info:
        ifs.slice_storey(model, 999999)
    assert info.value.storey_id == 999999


@pytest.mark.parametrize('bad', [0, -0.1, float('nan'), float('inf'), 'x', None])
def test_bad_slice_height_is_refused(model, levels, bad):
    with pytest.raises(ifs.IfcSourceError):
        ifs.slice_storey(model, levels['Level 1'], slice_height=bad)


@pytest.mark.parametrize('bad', [0, -5, 2.5, float('nan'), None])
def test_bad_face_budget_is_refused(model, levels, bad):
    with pytest.raises(ifs.IfcSourceError):
        ifs.slice_storey(model, levels['Level 1'], face_budget=bad)


def test_unknown_class_names_are_refused(model, levels):
    with pytest.raises(ifs.IfcSourceError, match='IfcNope'):
        ifs.slice_storey(model, levels['Level 1'], include=('IfcNope',))
    with pytest.raises(ifs.IfcSourceError, match='IfcNope'):
        ifs.slice_storey(model, levels['Level 1'], exclude=('IfcNope',))


def test_every_error_is_an_ifc_source_error_and_a_value_error():
    for cls in (ifs.IfcParseError, ifs.NoStoreysError, ifs.UnknownStoreyError,
                ifs.EmptySliceError, ifs.IfcTooLargeError, ifs.FileTooLargeError,
                ifs.FaceBudgetError):
        assert issubclass(cls, ifs.IfcSourceError) and issubclass(cls, ValueError)
    assert issubclass(ifs.FileTooLargeError, ifs.IfcTooLargeError)
    assert issubclass(ifs.FaceBudgetError, ifs.IfcTooLargeError)


# --- budgets: threads, determinism ---------------------------------------------------------

@pytest.mark.parametrize('cpus,expected', [(64, 4), (4, 4), (2, 2), (1, 1), (None, 1)])
def test_iterator_threads_is_min_of_four_and_cpu(monkeypatch, cpus, expected):
    monkeypatch.setattr(ifs.os, 'cpu_count', lambda: cpus)
    assert ifs.MAX_THREADS == 4
    assert ifs.iterator_threads() == expected


def test_requested_threads_are_clamped_and_validated():
    assert ifs.iterator_threads(2) == 2
    assert ifs.iterator_threads(99) == 4
    with pytest.raises(ifs.IfcSourceError):
        ifs.iterator_threads(0)


def test_thread_count_does_not_change_the_result(model, levels):
    one = ifs.slice_storey(model, levels['Level 1'], threads=1)
    many = ifs.slice_storey(model, levels['Level 1'], threads=4)
    assert ring_key(one) == ring_key(many)
    assert one.bounds == many.bounds and one.faces == many.faces
    assert pr.encode_pgm(raster(one)[0]) == pr.encode_pgm(raster(many)[0])


# --- fixture ------------------------------------------------------------------------------

def test_committed_fixture_is_small():
    assert len(GOOD) < 32 * 1024


def test_committed_fixture_matches_its_generator(model, l1, l2):
    fresh = reopen(gen.build('m'))
    assert floors(fresh) == floors(model)
    for ref, storey in zip((l1, l2), ifs.list_storeys(fresh)[:2], strict=True):
        got = ifs.slice_storey(fresh, storey)
        assert got.bounds == pytest.approx(ref.bounds, abs=1e-9)
        assert area(got) == pytest.approx(area(ref), abs=1e-9)


# --- integration: ifcsource -> planraster -> files -> gridstore ----------------------------

def test_slices_rasterize_to_pgm_and_yaml_that_gridstore_reads_back(model, tmp_path):
    for storey in ifs.list_storeys(model)[:2]:
        sl = ifs.slice_storey(model, storey)
        grid, meta = raster(sl)
        stem = 'floor%d' % storey.floor
        meta['image'] = stem + '.pgm'
        (tmp_path / (stem + '.pgm')).write_bytes(pr.encode_pgm(grid))
        (tmp_path / (stem + '.yaml')).write_text(pr.yaml_text(meta))
        w, h, body = gs.read_pgm(str(tmp_path / (stem + '.pgm')))
        assert (w, h) == (grid.shape[1], grid.shape[0]) and body == grid.tobytes()
        parsed = gs.read_map_yaml(str(tmp_path / (stem + '.yaml')))
        assert parsed['resolution'] == RES and parsed['image'] == stem + '.pgm'
        occ = gs.load_saved_map(str(tmp_path / (stem + '.yaml')))
        top_first = np.flipud(np.array(occ['data']).reshape(h, w))
        assert ((top_first == 100) == (grid == pr.OCCUPIED)).all()
        assert ((top_first == 0) == (grid == pr.FREE)).all()
        assert ((top_first == -1) == (grid == pr.UNKNOWN)).all()
        assert (occ['origin_x'], occ['origin_y']) == tuple(meta['origin'][:2])


def test_rotation_and_origin_go_through_calib_map_frame(l1):
    x0, y0, _, _ = l1.bounds
    grid, meta = pr.rasterize(l1.geometry, calib.map_frame(1.0, math.radians(30), (x0, y0)), RES)
    assert int((grid == pr.OCCUPIED).sum()) > 0
    assert grid.dtype == np.uint8 and (grid == pr.FREE).sum() > 1000


def test_same_slice_twice_gives_identical_bytes(model, levels):
    a = raster(ifs.slice_storey(model, levels['Level 1']))
    b = raster(ifs.slice_storey(model, levels['Level 1']))
    assert pr.encode_pgm(a[0]) == pr.encode_pgm(b[0]) and a[1] == b[1]
