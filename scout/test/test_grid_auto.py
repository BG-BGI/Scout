"""Automatic standard grid: label stability, wall-yaw estimator, extent, saved
maps, REST (docker/fleet-status/gridstore.py) and agreement between the two
copies of the code (docker/scout-skills/grid.py)."""
import base64
import importlib.util
import json
import math
import pathlib
import random

import pytest

_R = pathlib.Path(__file__).resolve().parents[2]


def _load(rel, name):
    spec = importlib.util.spec_from_file_location(name, _R / rel)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


sk = _load('docker/scout-skills/grid.py', 'skills_grid_auto')
gs = _load('docker/fleet-status/gridstore.py', 'fleet_gridstore_auto')
cli = _load('scripts/auto_grid_all_maps.py', 'auto_grid_cli')
MODS = [sk, gs]
CELL = 30 * 0.3048


def ext(x0, y0, x1, y1):
    return {'min_x': x0, 'min_y': y0, 'max_x': x1, 'max_y': y1}


def synth(angle_deg, w=400, h=400, res=0.05, noise=0.002, seed=1):
    """12 x 8 m room (+ partition walls) rotated by angle_deg, origin centred."""
    rnd = random.Random(seed)
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    data = [-1] * (w * h)
    for r in range(h):
        for q in range(w):
            x, y = (q - w / 2) * res, (r - h / 2) * res
            u, v = c * x + s * y, -s * x + c * y
            if abs(u) > 6.2 or abs(v) > 4.2:
                continue
            val = 0
            if (abs(u) > 5.95 or abs(v) > 3.95) and abs(u) <= 6.05 and abs(v) <= 4.05:
                val = 100
            if abs(u) < 0.05 and v < 1.5:
                val = 100
            if abs(v - 0.5) < 0.05 and u > 0:
                val = 100
            if abs(u) > 6.05 or abs(v) > 4.05:
                val = -1
            data[r * w + q] = val
    for k in range(len(data)):
        if rnd.random() < noise:
            data[k] = rnd.choice([0, 100])
    return {'width': w, 'height': h, 'resolution': res, 'data': data,
            'origin_x': -w * res / 2, 'origin_y': -h * res / 2}


# --- labels ------------------------------------------------------------------

@pytest.mark.parametrize('mod', MODS)
def test_label_scheme(mod):
    assert [mod.number_label(i) for i in (0, 1, 2, 9, -1, -2)] == \
        ['1', '2', '3', '10', '.1', '.2']
    assert [mod.letter_label(j) for j in (0, 1, 25, 26, 27, 701, 702, -1, -2, -26, -27)] == \
        ['A', 'B', 'Z', 'AA', 'AB', 'ZZ', 'AAA', '.A', '.B', '.Z', '.AA']
    nums = [mod.number_label(i) for i in range(-300, 300)]
    lets = [mod.letter_label(j) for j in range(-300, 300)]
    assert len({x.upper() for x in nums}) == 600
    assert len({x.upper() for x in lets}) == 600
    assert not {x.upper() for x in nums} & {x.upper() for x in lets}


def test_negative_labels_parse():
    g = sk.auto_new(ext(0, 0, 20, 20), 0.0)
    g = sk.extend_to(g, ext(-20, -20, 40, 40))
    assert sk.parse_label(g, '.1.A') == (g['numbers']['labels'].index('.1'),
                                          g['letters']['labels'].index('.A'))
    assert sk.parse_label(g, '1.A')[1] == g['letters']['labels'].index('.A')
    assert sk.parse_label(g, '1B') == (g['numbers']['labels'].index('1'),
                                       g['letters']['labels'].index('B'))
    r = sk.resolve(g, '1B')
    assert r['size_m'] == pytest.approx((CELL, CELL))


# --- creation + stability ----------------------------------------------------

def test_auto_new_snaps_and_covers():
    e = ext(1.0, -3.0, 30.0, 12.0)
    g = sk.auto_new(e, 0.0)
    assert g['mode'] == 'auto' and g['cell_m'] == pytest.approx(9.144)
    assert g['origin']['x'] == pytest.approx(0.0)
    assert g['origin']['y'] == pytest.approx(-9.144)   # floor(-3/9.144)*9.144
    assert g['numbers']['labels'][0] == '1' and g['letters']['labels'][0] == 'A'
    assert g['numbers']['offsets_m'][-1] >= 30.0 - 0 - 1e-9
    assert g['letters']['offsets_m'][-1] + g['origin']['y'] >= 12.0
    sk.validate(g)


def _snapshot(g):
    def one(ax):
        return dict(zip(ax['labels'], ax['offsets_m'], strict=True))
    return one(g['numbers']), one(g['letters'])


@pytest.mark.parametrize('mod', MODS)
@pytest.mark.parametrize('yaw', [0.0, 0.3, -0.7])
def test_growth_never_relabels(mod, yaw):
    g0 = mod.auto_new(ext(2, 3, 20, 14), yaw)
    n0, l0 = _snapshot(g0)
    g = g0
    steps = [ext(-5, 3, 20, 14),        # grow left
             ext(-5, -30, 20, 14),      # grow down
             ext(-5, -30, 80, 14),      # grow right
             ext(-5, -30, 80, 90),      # grow up
             ext(-100, -100, 200, 200),  # all four
             ext(0, 0, 1, 1)]           # shrink request: no-op
    for e in steps:
        prev = _snapshot(g)
        g = mod.extend_to(g, e)
        now = _snapshot(g)
        for a, b in zip(prev, now, strict=True):
            for lb, off in a.items():     # every old line: same label, same offset
                assert b[lb] == pytest.approx(off)
        # covers the extent
        for x, y in [(e['min_x'], e['min_y']), (e['max_x'], e['max_y']),
                     (e['min_x'], e['max_y']), (e['max_x'], e['min_y'])]:
            u, v = mod.to_grid_frame(g, x, y)
            assert g['numbers']['offsets_m'][0] <= u + 1e-6
            assert u <= g['numbers']['offsets_m'][-1] + 1e-6
            assert g['letters']['offsets_m'][0] <= v + 1e-6
            assert v <= g['letters']['offsets_m'][-1] + 1e-6
    n1, l1 = _snapshot(g)
    assert {k: n1[k] for k in n0} == pytest.approx(n0)
    assert {k: l1[k] for k in l0} == pytest.approx(l0)
    # growth appended at both ends with the negative scheme
    assert '.1' in n1 and '.A' in l1
    # step-by-step growth == one-shot growth (path independent)
    one = mod.extend_to(mod.extend_to(g0, ext(-100, -100, 200, 200)), ext(-5, -30, 80, 90))
    assert one == g


def test_extend_idempotent_and_structural_refused():
    g = sk.auto_new(ext(0, 0, 20, 20), 0.2)
    assert sk.extend_to(g, ext(1, 1, 19, 19)) == g
    st = sk.define(['1', '2'], ['A', 'B'], 12, 12)
    with pytest.raises(sk.GridError):
        sk.extend_to(st, ext(0, 0, 5, 5))
    out, action, _ = sk.auto_from_occupancy(None, ext(0, 0, 9, 9), st)
    assert action == 'kept_structural' and out == st


def test_cell_size_and_line_cap():
    with pytest.raises(sk.GridError):
        sk.auto_new(ext(0, 0, 5, 5), 0.0, cell_ft=0.5)
    with pytest.raises(sk.GridError):
        sk.auto_new(ext(0, 0, 5000, 5), 0.0, cell_ft=5)    # >200 lines
    g = sk.auto_new(ext(0, 0, 5, 5), 0.0, cell_ft=12)
    assert g['cell_m'] == pytest.approx(12 * 0.3048)


def _ag(case):
    def run(mod):
        try:
            return mod.validate(case)
        except mod.GridError:
            return 'rejected'
    return run


def _tamper():
    g = sk.auto_new(ext(0, 0, 20, 20), 0.1)
    import copy
    out = [g]
    a = copy.deepcopy(g)
    a['numbers']['labels'][1] = '7'
    b = copy.deepcopy(g)
    b['letters']['offsets_m'][1] += 0.01
    c = copy.deepcopy(g)
    del c['cell_m']
    d = copy.deepcopy(g)
    d['mode'] = 'weird'
    e = copy.deepcopy(g)
    e['numbers']['index_start'] = 1.5
    f = copy.deepcopy(g)
    del f['numbers']['index_start']
    h = sk.extend_to(g, ext(-30, -30, 60, 60))
    k = copy.deepcopy(g)
    k['cell_m'] = 0.2
    return out + [a, b, c, d, e, f, h, k]


@pytest.mark.parametrize('case', _tamper())
def test_auto_validators_agree(case):
    assert _ag(case)(sk) == _ag(case)(gs)


def test_auto_validator_rejects_tampering():
    for case in _tamper()[1:7] + _tamper()[8:]:
        assert _ag(case)(sk) == 'rejected'
    assert _ag(_tamper()[0])(sk) != 'rejected'
    assert _ag(_tamper()[7])(sk) != 'rejected'


def test_structural_default_mode_and_roundtrip():
    g = sk.define(['1', '2'], ['A', 'B'], 12, 12)
    assert g['mode'] == 'structural' and 'cell_m' not in g
    assert sk.validate({k: v for k, v in g.items() if k != 'mode'}) == g


def test_construction_agreement():
    e = ext(-4.2, 1.1, 33.3, 25.0)
    for yaw in (0.0, 0.4, -0.9):
        assert sk.auto_new(e, yaw) == gs.auto_new(e, yaw)
    g = sk.auto_new(e, 0.4)
    e2 = ext(-90, -50, 7, 3)
    assert sk.extend_to(g, e2) == gs.extend_to(g, e2)


# --- wall-yaw estimator ------------------------------------------------------

@pytest.mark.parametrize('angle', [0.0, 3.3, -7.0, 22.5, -30.0, 44.0, 12.4, -44.0])
def test_yaw_synthetic_rotated(angle):
    occ = synth(angle)
    for mod in MODS:
        yaw, info = mod.estimate_yaw(occ)
        assert info['source'] == 'walls'
        d = math.degrees(yaw) - angle
        d = (d + 45) % 90 - 45
        assert abs(d) < 0.5, (angle, math.degrees(yaw))
        assert -math.pi / 4 < yaw <= math.pi / 4 + 1e-9


def test_yaw_heavy_noise_still_ok():
    occ = synth(17.0, noise=0.01, seed=5)
    yaw, info = sk.estimate_yaw(occ)
    assert info['source'] == 'walls' and abs(math.degrees(yaw) - 17) < 0.5


def test_yaw_fallbacks():
    empty = {'width': 50, 'height': 50, 'resolution': 0.05, 'data': [-1] * 2500,
             'origin_x': 0, 'origin_y': 0}
    assert sk.estimate_yaw(empty)[0] == 0.0
    few = dict(empty, data=[100 if k < 40 else 0 for k in range(2500)])
    yaw, info = sk.estimate_yaw(few)
    assert yaw == 0.0 and info['source'] == 'fallback'
    rnd = random.Random(3)
    noisy = dict(empty, width=200, height=200,
                 data=[100 if rnd.random() < 0.05 else 0 for _ in range(40000)])
    yaw, info = sk.estimate_yaw(noisy)
    assert yaw == 0.0 and info['source'] == 'fallback' and info['strength'] < 0.3


def test_yaw_agreement_between_copies():
    occ = synth(-12.0, noise=0.004, seed=9)
    assert sk.estimate_yaw(occ) == gs.estimate_yaw(occ)


# --- extent + saved maps -----------------------------------------------------

def test_extent_from_occupancy():
    w, h = 10, 8
    data = [-1] * (w * h)
    data[2 * w + 3] = 0      # row 2 (from bottom), col 3
    data[5 * w + 7] = 100
    occ = {'width': w, 'height': h, 'resolution': 0.5, 'origin_x': 10.0,
           'origin_y': -4.0, 'data': data}
    e = sk.extent_from_occupancy(occ)
    assert e == {'min_x': 10 + 2 * .5, 'max_x': 10 + 9 * .5,
                 'min_y': -4 + 1 * .5, 'max_y': -4 + 7 * .5}
    assert sk.extent_from_occupancy(dict(occ, data=[-1] * 80)) is None
    assert gs.extent_from_occupancy(occ) == e


def write_map(d, name, occ_rows_top_first, res=0.05, origin=(-1.0, -2.0, 0.0), negate=0,
              hdr_comment=True):
    h, w = len(occ_rows_top_first), len(occ_rows_top_first[0])
    body = bytes(v for row in occ_rows_top_first for v in row)
    head = b'P5\n' + (b'# created by test\n' if hdr_comment else b'') + \
        f'{w} {h}\n255\n'.encode()
    (d / f'{name}.pgm').write_bytes(head + body)
    (d / f'{name}.yaml').write_text(
        f'image: {name}.pgm\nmode: trinary\nresolution: {res}\n'
        f'origin: [{origin[0]}, {origin[1]}, {origin[2]}]\nnegate: {negate}\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')


def _room_rows(w=300, h=200, unknown_pad=40):
    rows = [[205] * w for _ in range(h)]
    for r in range(unknown_pad, h - unknown_pad):
        for c in range(unknown_pad, w - unknown_pad):
            edge = (r in (unknown_pad, h - unknown_pad - 1)
                    or c in (unknown_pad, w - unknown_pad - 1))
            rows[r][c] = 0 if edge else 254
    return rows


def test_load_saved_map_and_extent(tmp_path):
    write_map(tmp_path, 'm', _room_rows())
    occ = sk.load_saved_map(str(tmp_path / 'm.yaml'))
    assert (occ['width'], occ['height']) == (300, 200)
    assert occ['data'].count(100) == 2 * 220 + 2 * 120 - 4
    assert occ['data'][0] == -1                      # 205 stays unknown
    e = sk.extent_from_occupancy(occ)
    # known cols 40..259, rows (top) 40..159 -> bottom-first rows 40..159; margin 1 cell
    assert e['min_x'] == pytest.approx(-1.0 + 39 * 0.05)
    assert e['max_x'] == pytest.approx(-1.0 + 261 * 0.05)
    assert e['min_y'] == pytest.approx(-2.0 + 39 * 0.05)
    assert e['max_y'] == pytest.approx(-2.0 + 161 * 0.05)
    assert gs.load_saved_map(str(tmp_path / 'm.yaml')) == occ


def test_load_saved_map_negate_and_flip(tmp_path):
    rows = [[205] * 4 for _ in range(3)]
    rows[0][1] = 0           # TOP row: occupied (negate=0)
    rows[2][2] = 254         # bottom row: free
    write_map(tmp_path, 'n0', rows)
    d = sk.load_saved_map(str(tmp_path / 'n0.yaml'))['data']
    assert d[2 * 4 + 1] == 100 and d[0 * 4 + 2] == 0     # row 0 = bottom
    inv = [[255 - v for v in r] for r in rows]
    write_map(tmp_path, 'n1', inv, negate=1)
    assert sk.load_saved_map(str(tmp_path / 'n1.yaml'))['data'] == d


def test_bad_files(tmp_path):
    (tmp_path / 'x.yaml').write_text('image: x.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
    (tmp_path / 'x.pgm').write_bytes(b'P2\n1 1\n255\n0')
    with pytest.raises(sk.GridError):
        sk.load_saved_map(str(tmp_path / 'x.yaml'))
    (tmp_path / 'y.yaml').write_text('image: y.pgm\n')
    with pytest.raises(sk.GridError):
        sk.read_map_yaml(str(tmp_path / 'y.yaml'))


# --- offline apply (CLI / apply-all) ------------------------------------------

def _site(tmp_path):
    maps = tmp_path / 'sites' / 'hq' / 'maps'
    maps.mkdir(parents=True)
    write_map(maps, 'floor1', _room_rows())
    write_map(maps, 'floor2', _room_rows(w=200, h=160, unknown_pad=20))
    (maps / '_session.yaml').write_text('x')
    return tmp_path / 'sites', maps


def test_apply_all_idempotent_and_dry_run(tmp_path):
    sites, maps = _site(tmp_path)
    (maps / 'floor2.grid.json').write_text(json.dumps(
        sk.define(['1', '2'], ['A', 'B'], 12, 12)))
    dry = cli.run(str(sites), dry_run=True)
    assert {r['map']: r['action'] for r in dry} == {'floor1': 'created',
                                                    'floor2': 'skipped_has_grid'}
    assert not (maps / 'floor1.grid.json').exists()
    first = cli.run(str(sites))
    assert first[0]['action'] == 'created' and (maps / 'floor1.grid.json').exists()
    snap = (maps / 'floor1.grid.json').read_text()
    again = cli.run(str(sites))
    assert [r['action'] for r in again] == ['skipped_has_grid', 'skipped_has_grid']
    assert (maps / 'floor1.grid.json').read_text() == snap
    st = json.loads((maps / 'floor2.grid.json').read_text())
    assert st['mode'] == 'structural'
    # refit: auto grid with an unchanged map is a no-op; structural untouched
    assert [r['action'] for r in cli.run(str(sites), refit=True)] == \
        ['unchanged', 'skipped_has_grid']
    assert cli.main(['--sites-dir', str(sites), '--dry-run']) == 0


def test_apply_all_gridstore_matches_cli(tmp_path):
    sites, maps = _site(tmp_path)
    code, out = gs.auto_grid_all(str(sites), {'dry_run': True})
    assert code == 200 and not (maps / 'floor1.grid.json').exists()
    assert {r['map'] for r in out['results']} == {'floor1', 'floor2'}
    gs.auto_grid_all(str(sites), {})
    via_rest = json.loads((maps / 'floor1.grid.json').read_text())
    (maps / 'floor1.grid.json').unlink()
    cli.run(str(sites))
    assert json.loads((maps / 'floor1.grid.json').read_text()) == via_rest
    assert via_rest['mode'] == 'auto'


def test_apply_files_corrupt_grid_not_overwritten(tmp_path):
    sites, maps = _site(tmp_path)
    (maps / 'floor1.grid.json').write_text('{nope')
    r = cli.run(str(sites))
    assert r[0]['action'] == 'skipped_unreadable_grid'
    assert (maps / 'floor1.grid.json').read_text() == '{nope'


# --- REST ---------------------------------------------------------------------

@pytest.fixture
def env(tmp_path):
    maps = tmp_path / 'maps'
    maps.mkdir()
    site = tmp_path / 'site.json'
    site.write_text(json.dumps({'active_map': 'floor1'}))
    return str(maps), str(site)


def _body(occ):
    raw = bytes((v + 256) % 256 for v in occ['data'])
    return {'width': occ['width'], 'height': occ['height'],
            'resolution': occ['resolution'],
            'origin': {'x': occ['origin_x'], 'y': occ['origin_y']},
            'data_b64': base64.b64encode(raw).decode()}


def test_rest_create_extend_structural(env):
    maps, site = env
    occ = synth(10.0)
    code, out = gs.auto_grid(maps, site, {'occupancy': _body(occ)})
    assert code == 200 and out['action'] == 'created' and out['mode'] == 'auto'
    assert out['yaw_source'] == 'walls'
    assert abs(math.degrees(out['grid']['origin']['yaw_rad']) - 10) < 0.5
    assert gs.get_grid(maps, site)[1]['mode'] == 'auto'
    g1 = out['grid']
    # same occupancy again: unchanged, file identical
    assert gs.auto_grid(maps, site, {'occupancy': _body(occ)})[1]['action'] == 'unchanged'
    # grow by extent (yaw stays frozen, labels stay)
    code, out = gs.auto_grid(maps, site, {'extent': ext(-60, -50, 90, 70)})
    assert out['action'] == 'extended'
    g2 = out['grid']
    assert g2['origin'] == g1['origin'] and g2['cell_m'] == g1['cell_m']
    n1, l1 = _snapshot(g1)
    n2, l2 = _snapshot(g2)
    assert all(n2[k] == pytest.approx(v) for k, v in n1.items())
    assert all(l2[k] == pytest.approx(v) for k, v in l1.items())
    # structural replaces auto, then auto refuses
    st = {'version': 1, 'mode': 'structural', 'units': 'ft',
          'origin': {'x': 0, 'y': 0, 'yaw_rad': 0},
          'numbers': {'labels': ['1', '2'], 'offsets_m': [0, 3.6]},
          'letters': {'labels': ['A', 'B'], 'offsets_m': [0, 3.6]}}
    assert gs.put_grid(maps, site, st)[0] == 200
    code, out = gs.auto_grid(maps, site, {'occupancy': _body(occ)})
    assert code == 200 and out['action'] == 'kept_structural'
    assert gs.get_grid(maps, site)[1]['grid']['mode'] == 'structural'
    assert gs.get_grid(maps, site)[1]['grid']['numbers']['labels'] == ['1', '2']


def test_rest_saved_map_fallback_and_errors(env):
    maps, site = env
    code, out = gs.auto_grid(maps, site, {})
    assert out['action'] == 'empty'                       # nothing to size from
    write_map(pathlib.Path(maps), 'floor1', _room_rows())
    code, out = gs.auto_grid(maps, site, {})
    assert code == 200 and out['action'] == 'created'
    assert gs.auto_grid(maps, site, {'align': 'bogus'})[0] == 400
    pathlib.Path(maps, 'floor1.grid.json').unlink()
    assert gs.auto_grid(maps, site, {'align': 'bogus'})[0] == 400
    assert gs.auto_grid(maps, site, {'extent': {'min_x': 1}})[0] == 400
    assert gs.auto_grid(maps, site, {'occupancy': {'width': 2, 'height': 2,
                                                   'resolution': 1, 'data': [0],
                                                   'origin': {'x': 0, 'y': 0}}})[0] == 400
    pathlib.Path(maps, 'floor1.grid.json').write_text('{bad')
    assert gs.auto_grid(maps, site, {'extent': ext(0, 0, 9, 9)})[0] == 409


def test_rest_matches_skills_module(env):
    maps, site = env
    occ = synth(-21.0, seed=4)
    _, out = gs.auto_grid(maps, site, {'occupancy': _body(occ), 'cell_ft': 20})
    g, action, _ = sk.auto_from_occupancy(occ, None, None, 20, 'walls')
    assert action == 'created' and g == out['grid']


def test_session_grid_extent_only(tmp_path):
    maps = str(tmp_path / 'maps')
    site = str(tmp_path / 'none.json')           # no site.json -> _session
    code, out = gs.auto_grid(maps, site, {'extent': ext(0, 0, 12, 7)})
    assert code == 200 and out['action'] == 'created'
    assert out['map'] == '_session'
    assert (tmp_path / 'maps' / '_session.grid.json').exists()
    assert out['grid']['origin']['yaw_rad'] == 0.0
