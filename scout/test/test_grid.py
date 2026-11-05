"""Structural grid math (docker/scout-skills/grid.py)."""
import importlib.util
import itertools
import math
import pathlib

import pytest

_P = pathlib.Path(__file__).resolve().parents[2] / 'docker/scout-skills/grid.py'
_spec = importlib.util.spec_from_file_location('scout_grid', _P)
g = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g)

YAWS = [0.0, 0.5, math.pi / 2, -2.3, math.pi]


def _grid(yaw=0.0, units='ft', ox=3.0, oy=-2.0):
    return g.define(['1', '2', '3', '10'], ['A', 'B', 'C', 'AA'],
                    [12, 20, 7.5], [10, 30, 5], units=units,
                    origin={'x': ox, 'y': oy, 'yaw_rad': yaw})


def test_define_uniform_and_list_ft():
    gr = g.define(['1', '2', '3'], ['A', 'B'], 12, 20)
    assert gr['numbers']['offsets_m'] == pytest.approx([0, 12 * .3048, 24 * .3048])
    assert gr['letters']['offsets_m'] == pytest.approx([0, 20 * .3048])
    gm = g.define(['1', '2'], ['A', 'B'], [4.0], 2.5, units='m')
    assert gm['numbers']['offsets_m'] == [0.0, 4.0]


def test_define_bad_spacing():
    with pytest.raises(g.GridError):
        g.define(['1', '2', '3'], ['A', 'B'], [12], 5)
    with pytest.raises(g.GridError):
        g.define(['1', '2'], ['A', 'B'], 0, 5)
    with pytest.raises(g.GridError):
        g.define(['1', '2'], ['A', 'B'], 'x', 5)


@pytest.mark.parametrize('yaw', YAWS)
@pytest.mark.parametrize('units', ['ft', 'm'])
def test_roundtrip_every_bay(yaw, units):
    gr = _grid(yaw, units)
    for i, j in itertools.product(range(3), range(3)):
        lab = g.bay_label(gr, i, j)
        r = g.resolve(gr, lab)
        loc = g.locate(gr, r['center']['x'], r['center']['y'])
        assert loc['bay'] == lab and loc['inside']
        for c in r['corners']:  # corners are on the boundary, centre-ward nudge inside
            nx = c['x'] + (r['center']['x'] - c['x']) * 1e-6
            ny = c['y'] + (r['center']['y'] - c['y']) * 1e-6
            assert g.locate(gr, nx, ny)['bay'] == lab


def test_resolve_geometry_and_size():
    gr = _grid(0.7)
    r = g.resolve(gr, '1B')
    assert r['size_in_units'] == pytest.approx((12, 30))
    assert r['size_m'] == pytest.approx((12 * .3048, 30 * .3048))
    assert len(r['corners']) == 4
    # CCW => positive shoelace area
    p = r['corners']
    a = sum(p[k]['x'] * p[(k + 1) % 4]['y'] - p[(k + 1) % 4]['x'] * p[k]['y']
            for k in range(4)) / 2
    assert a == pytest.approx(r['size_m'][0] * r['size_m'][1])
    b = r['bounds']
    assert b['min_x'] < b['max_x'] and b['min_y'] < b['max_y']


def test_intersection_and_locate_nearest():
    gr = _grid(1.1)
    r = g.resolve(gr, '2/C', 'intersection')
    assert r['polygon'] == []
    loc = g.locate(gr, r['center']['x'] + 0.01, r['center']['y'])
    assert loc['nearest_intersection']['label'] == '2C'
    assert loc['nearest_intersection']['distance_m'] == pytest.approx(0.01)
    out = g.locate(gr, 500, 500)
    assert out['bay'] is None and not out['inside']


def test_locate_on_last_line_is_last_bay():
    gr = _grid(0.0)
    x, y = g.from_grid_frame(gr, gr['numbers']['offsets_m'][-1],
                             gr['letters']['offsets_m'][1] + 0.1)
    assert g.locate(gr, x, y)['bay'] == '3B'


@pytest.mark.parametrize('text', ['1B', 'b1', '1-B', 'B-1', 'B/1', ' 1 b ', 'b_1'])
def test_label_forms(text):
    assert g.parse_label(_grid(), text) == (0, 1)


def test_multichar_labels_and_ambiguity():
    gr = _grid()
    assert g.parse_label(gr, '10aa') == (3, 3)
    assert g.parse_label(gr, 'AA-10') == (3, 3)
    amb = g.define(['1', '11'], ['A', '1A'], 5, 5)
    with pytest.raises(g.GridError, match='ambiguous'):
        g.parse_label(amb, '11A')
    assert g.parse_label(amb, '11-A') == (1, 0)
    with pytest.raises(g.GridError):
        g.parse_label(gr, '1Z')
    with pytest.raises(g.GridError):
        g.parse_label(gr, '')


def test_last_line_has_no_bay():
    gr = _grid()
    with pytest.raises(g.GridError, match='last line'):
        g.resolve(gr, '10A')
    with pytest.raises(g.GridError, match='last line'):
        g.resolve(gr, '1AA')
    g.resolve(gr, '10AA', 'intersection')  # intersections are fine


def test_unknown_kind():
    with pytest.raises(g.GridError):
        g.resolve(_grid(), '1B', 'room')


@pytest.mark.parametrize('yaw', YAWS)
def test_frame_roundtrip(yaw):
    gr = _grid(yaw)
    x, y = g.from_grid_frame(gr, 4.2, -1.3)
    u, v = g.to_grid_frame(gr, x, y)
    assert (u, v) == pytest.approx((4.2, -1.3), abs=1e-12)


@pytest.mark.parametrize('yaw', YAWS)
def test_align_recovers_pose(yaw):
    truth = _grid(yaw, ox=-7.3, oy=11.9)
    p1 = g.resolve(truth, '1A', 'intersection')['center']
    p2 = g.resolve(truth, '3-C', 'intersection')['center']
    base = g.define(truth['numbers']['labels'], truth['letters']['labels'],
                    [12, 20, 7.5], [10, 30, 5])
    got, res = g.align(base, dict(p1, number='1', letter='A'),
                       dict(p2, number='3', letter='C'))
    o = got['origin']
    assert o['x'] == pytest.approx(-7.3, abs=1e-9)
    assert o['y'] == pytest.approx(11.9, abs=1e-9)
    assert math.cos(o['yaw_rad'] - yaw) == pytest.approx(1, abs=1e-9)
    assert res == pytest.approx(0, abs=1e-9)


def test_align_residual_flags_bad_click():
    base = _grid()
    p1 = {'x': 0, 'y': 0, 'number': '1', 'letter': 'A'}
    d = math.hypot(32 * .3048, 40 * .3048)
    p2 = {'x': d + 1.5, 'y': 0, 'number': '3', 'letter': 'C'}
    _, res = g.align(base, p1, p2)
    assert res == pytest.approx(1.5)


def test_align_errors():
    base = _grid()
    a = {'x': 0, 'y': 0, 'number': '1', 'letter': 'A'}
    with pytest.raises(g.GridError):
        g.align(base, a, dict(a, x=5))                    # same intersection
    with pytest.raises(g.GridError):
        g.align(base, a, {'x': 0, 'y': 0, 'number': '2', 'letter': 'A'})  # same point
    with pytest.raises(g.GridError):
        g.align(base, a, {'x': 1, 'y': 0, 'number': '9', 'letter': 'A'})


def test_lines():
    gr = _grid(0.3)
    ls = g.lines(gr)
    assert len(ls) == 8
    n1 = next(s for s in ls if s['kind'] == 'number' and s['label'] == '2')
    d = math.hypot(n1['p2']['x'] - n1['p1']['x'], n1['p2']['y'] - n1['p1']['y'])
    assert d == pytest.approx(45 * .3048)


def _ok(**kw):
    b = {'version': 1, 'units': 'ft', 'origin': {'x': 0, 'y': 0, 'yaw_rad': 0},
         'numbers': {'labels': ['1', '2'], 'offsets_m': [0, 3]},
         'letters': {'labels': ['A', 'B'], 'offsets_m': [0, 3]}}
    b.update(kw)
    return b


BAD = [
    ('not obj', [1]),
    ('units', _ok(units='yd')),
    ('version', _ok(version=2)),
    ('origin nan', _ok(origin={'x': float('nan'), 'y': 0, 'yaw_rad': 0})),
    ('origin bool', _ok(origin={'x': True, 'y': 0, 'yaw_rad': 0})),
    ('yaw big', _ok(origin={'x': 0, 'y': 0, 'yaw_rad': 10})),
    ('one line', _ok(numbers={'labels': ['1'], 'offsets_m': [0]})),
    ('len mismatch', _ok(numbers={'labels': ['1', '2'], 'offsets_m': [0]})),
    ('non increasing', _ok(numbers={'labels': ['1', '2'], 'offsets_m': [0, 0]})),
    ('inf', _ok(numbers={'labels': ['1', '2'], 'offsets_m': [0, float('inf')]})),
    ('dup label', _ok(letters={'labels': ['A', 'a'], 'offsets_m': [0, 3]})),
    ('shared label', _ok(letters={'labels': ['1', 'B'], 'offsets_m': [0, 3]})),
    ('bad label', _ok(letters={'labels': ['A B', 'C'], 'offsets_m': [0, 3]})),
    ('str offset', _ok(letters={'labels': ['A', 'B'], 'offsets_m': [0, '3']})),
    ('anchor', _ok(bay_anchor='upper')),
    ('missing axis', {'origin': {'x': 0, 'y': 0, 'yaw_rad': 0}}),
]


@pytest.mark.parametrize('name,body', BAD, ids=[b[0] for b in BAD])
def test_validate_rejects(name, body):
    with pytest.raises(g.GridError):
        g.validate(body)


def test_validate_accepts_and_normalizes():
    out = g.validate(_ok(numbers={'labels': [' 1 ', '2'], 'offsets_m': [0, 3]}))
    assert out['numbers']['labels'] == ['1', '2']
    assert out['bay_anchor'] == 'lower'
    assert g.validate(out) == out


def test_storage_roundtrip(tmp_path):
    assert g.grid_path(str(tmp_path), 'floor1').endswith('floor1.grid.json')
    assert g.grid_path(str(tmp_path), '../x').endswith('_session.grid.json')
    assert g.grid_path(str(tmp_path), None).endswith('_session.grid.json')
    p = g.grid_path(str(tmp_path), 'floor1')
    assert g.load(p) is None
    gr = _grid(0.2)
    g.save(p, gr)
    assert g.load(p) == gr
    (tmp_path / 'floor1.grid.json').write_text('{bad')
    assert g.load(p) is None
