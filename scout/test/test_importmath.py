"""webui/importmath.js (page / map / preview / canvas maths, request bodies, importer address)
under Node, cross-checked against docker/floorplan-importer/calib.map_frame computed in Python.

The maths must agree with the importer exactly: a preview that lands in the wrong place
on the page looks like a bad calibration. Skips when `node` is absent (Pi container)."""

import importlib.util
import json
import math
import pathlib
import random
import shutil
import subprocess
import sys

import numpy as np
import pytest

from scout.core import sites

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
IMPORTMATH = REPO_ROOT / 'webui' / 'importmath.js'

_spec = importlib.util.spec_from_file_location(
    'fp_calib_for_importmath', REPO_ROOT / 'docker' / 'floorplan-importer' / 'calib.py'
)
cal = importlib.util.module_from_spec(_spec)
sys.modules['fp_calib_for_importmath'] = cal
_spec.loader.exec_module(cal)

RUNNER = """
const IM = require(process.argv[1]);
const call = JSON.parse(process.argv[2]);
console.log(JSON.stringify(IM[call.fn](...call.args)));
"""


def _call(fn, *args):
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    proc = subprocess.run(
        [node, '-e', RUNNER, str(IMPORTMATH), json.dumps({'fn': fn, 'args': list(args)})],
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return json.loads(proc.stdout)


def _close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# --- names / taps ---


@pytest.mark.parametrize(
    'name,ok',
    [
        ('plan', True),
        ('a', True),
        ('0abc', True),
        ('a-b_c', True),
        ('x' * 32, True),
        ('x' * 33, False),
        ('_x', False),
        ('-x', False),
        ('Plan', False),
        ('a b', False),
        ('plan\n', False),
        ('', False),
        (None, False),
        (7, False),
        ('a.b', False),
    ],
)
def test_valid_name(name, ok):
    assert _call('validName', name) is ok


def test_name_regex_is_the_shared_contract():
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed')
    src = subprocess.run(
        [node, '-e', f"console.log(require('{IMPORTMATH}').NAME_RE.source)"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    assert src == sites.SITE_NAME_RE.pattern


@pytest.mark.parametrize(
    'label,expect',
    [
        ('Level 1', 'level_1'),
        ('L2 - Offices!', 'l2_offices'),
        ('  ', 'floor'),
        (None, 'floor'),
        ('Ünïcode', 'n_code'),
        ('a' * 40, 'a' * 32),
        ('-- x --', 'x'),
    ],
)
def test_suggest_name_is_always_valid_or_floor(label, expect):
    got = _call('suggestName', label)
    assert got == expect and _call('validName', got) is True


def test_taps_closer_than_20_px_are_refused():
    assert _call('tooClose', {'x': 0, 'y': 0}, {'x': 19.9, 'y': 0}) is True
    assert _call('tooClose', {'x': 0, 'y': 0}, {'x': 20, 'y': 0}) is False
    assert _call('tooClose', {'x': 5, 'y': 5}, {'x': 5, 'y': 5}) is True
    assert _call('tooClose', {'x': 0, 'y': 0}, {'x': 14, 'y': 14}) is True  # 19.8 px
    assert _call('tooClose', {'x': 0, 'y': 0}, {'x': 14.2, 'y': 14.2}) is False


# --- PNG pixel <-> page ---

PNG = {'scale_x': 2.0, 'scale_y': 4.0, 'height': 100.0, 'width': 80.0}


def test_pixel_page_mapping_matches_the_contract():
    # page_x = px / scale_x, page_y = height - py / scale_y
    assert _call('pxToPage', PNG, 50, 40) == {'x': 25.0, 'y': 90.0}
    assert _call('pageToPx', PNG, 25, 90) == {'x': 50.0, 'y': 40.0}
    top_left = _call('pxToPage', PNG, 0, 0)
    assert top_left == {'x': 0.0, 'y': 100.0}  # py counts from the TOP


def test_pixel_page_round_trip_with_unequal_scales():
    for px, py in [(0, 0), (160, 400), (33.3, 77.7)]:
        p = _call('pxToPage', PNG, px, py)
        back = _call('pageToPx', PNG, p['x'], p['y'])
        assert _close(back['x'], px) and _close(back['y'], py)


# --- page <-> map, against calib.map_frame ---


def _py_map(m_per_unit, rot_deg, origin, p):
    s, rot, t = cal.map_frame(m_per_unit, math.radians(rot_deg), origin)
    return s * (rot @ np.array(p)) + t


def _py_page(m_per_unit, rot_deg, origin, m):
    s, rot, t = cal.map_frame(m_per_unit, math.radians(rot_deg), origin)
    return np.linalg.solve(s * rot, np.array(m) - t)


CASES = [
    (0.0123, 0.0, [0.0, 0.0]),
    (0.0123, 90.0, [100.0, 50.0]),
    (0.5, 37.5, [-3.0, 8.0]),
    (0.004, -120.0, [1200.5, 800.25]),
    (1.0, 180.0, [10.0, 10.0]),
    (0.02, 359.0, [5.0, -7.0]),
]


@pytest.mark.parametrize('m,rot,origin', CASES)
def test_page_to_map_matches_calib_map_frame(m, rot, origin):
    rng = random.Random(7)
    frame = _call('mapFrame', m, rot, origin)
    for _ in range(8):
        p = [rng.uniform(-500, 2500), rng.uniform(-500, 1800)]
        want = _py_map(m, rot, origin, p)
        got = _call('pageToMap', frame, {'x': p[0], 'y': p[1]})
        assert _close(got['x'], want[0], 1e-7) and _close(got['y'], want[1], 1e-7)
        back = _call('mapToPage', frame, got)
        assert _close(back['x'], p[0], 1e-6) and _close(back['y'], p[1], 1e-6)


@pytest.mark.parametrize('m,rot,origin', CASES)
def test_the_origin_is_map_zero_and_plus_x_points_rotation_ccw(m, rot, origin):
    frame = _call('mapFrame', m, rot, origin)
    zero = _call('pageToMap', frame, {'x': origin[0], 'y': origin[1]})
    assert _close(zero['x'], 0) and _close(zero['y'], 0)
    th = math.radians(rot)
    along = {'x': origin[0] + math.cos(th), 'y': origin[1] + math.sin(th)}
    on_x = _call('pageToMap', frame, along)
    assert _close(on_x['x'], m, 1e-9) and _close(on_x['y'], 0, 1e-9)


# --- preview placement on the page ---


def _calib(
    m=0.0123, rot=23.0, origin=(310.0, 220.0), pv_origin=(-4.0, -3.0), res=0.05, w=240, h=180
):
    return {
        'm_per_unit': m,
        'rotation_deg': rot,
        'origin': list(origin),
        'preview': {
            'origin': [pv_origin[0], pv_origin[1], 0.0],
            'resolution': res,
            'width': w,
            'height': h,
        },
    }


def _py_preview_page(c, u, v):
    pv = c['preview']
    m = [
        pv['origin'][0] + u * pv['resolution'],
        pv['origin'][1] + (pv['height'] - v) * pv['resolution'],
    ]
    return _py_page(c['m_per_unit'], c['rotation_deg'], c['origin'], m)


@pytest.mark.parametrize('u,v', [(0, 0), (240, 0), (0, 180), (240, 180), (120.5, 33.25)])
def test_preview_pixel_lands_where_calib_says(u, v):
    c = _calib()
    want = _py_preview_page(c, u, v)
    got = _call('previewPixelToPage', c, u, v)
    assert _close(got['x'], want[0], 1e-7) and _close(got['y'], want[1], 1e-7)


def test_preview_row_zero_is_the_top_of_the_grid():
    c = _calib(rot=0.0)
    top = _call('previewPixelToPage', c, 0, 0)
    bottom = _call('previewPixelToPage', c, 0, 180)
    assert top['y'] > bottom['y']  # page y is up, preview v is down


def test_a_page_point_round_trips_through_the_preview_matrix():
    c = _calib()
    png = {'scale_x': 1.5, 'scale_y': 1.5, 'height': 1728.0, 'width': 2592.0}
    page_pt = {'x': 400.0, 'y': 300.0}
    m = _py_map(c['m_per_unit'], c['rotation_deg'], c['origin'], [page_pt['x'], page_pt['y']])
    pv = c['preview']
    u = (m[0] - pv['origin'][0]) / pv['resolution']
    v = pv['height'] - (m[1] - pv['origin'][1]) / pv['resolution']
    mat = _call('previewToPagePxMatrix', c, png)
    px = _call('applyMatrix', mat, {'x': u, 'y': v})
    want = _call('pageToPx', png, page_pt['x'], page_pt['y'])
    assert _close(px['x'], want['x'], 1e-6) and _close(px['y'], want['y'], 1e-6)


def test_preview_matrix_handles_an_image_of_another_size():
    c = _calib()
    png = {'scale_x': 1.0, 'scale_y': 1.0, 'height': 800.0, 'width': 800.0}
    full = _call('previewToPagePxMatrix', c, png)
    half = _call('previewToPagePxMatrix', c, png, 120, 90)  # image is half the cells
    a = _call('applyMatrix', full, {'x': 100, 'y': 60})
    b = _call('applyMatrix', half, {'x': 50, 'y': 30})
    assert _close(a['x'], b['x'], 1e-9) and _close(a['y'], b['y'], 1e-9)


def test_canvas_matrices_agree_with_the_view_transform():
    view = {'k': 0.37, 'cx': 410.0, 'cy': 260.0}
    size = {'w': 320, 'h': 280}
    png = {'scale_x': 1.5, 'scale_y': 1.5, 'height': 1728.0, 'width': 2592.0}
    for px, py in [(0, 0), (1000, 400), (3888, 2592)]:
        page = _call('pxToPage', png, px, py)
        want = _call('pageToCanvas', view, size, page)
        got = _call(
            'applyMatrix', _call('pagePxToCanvasMatrix', png, view, size), {'x': px, 'y': py}
        )
        assert _close(got['x'], want['x'], 1e-6) and _close(got['y'], want['y'], 1e-6)
    c = _calib()
    for u, v in [(0, 0), (240, 180), (77, 120)]:
        page = _call('previewPixelToPage', c, u, v)
        want = _call('pageToCanvas', view, size, page)
        got = _call(
            'applyMatrix', _call('previewToCanvasMatrix', c, png, view, size), {'x': u, 'y': v}
        )
        assert _close(got['x'], want['x'], 1e-5) and _close(got['y'], want['y'], 1e-5)


def test_canvas_page_round_trip():
    view = {'k': 0.8, 'cx': 100.0, 'cy': 50.0}
    size = {'w': 300, 'h': 200}
    c = _call('pageToCanvas', view, size, {'x': 120, 'y': 40})
    back = _call('canvasToPage', view, size, c['x'], c['y'])
    assert _close(back['x'], 120) and _close(back['y'], 40)
    # the page point at the view centre is the canvas centre; y is flipped
    assert _call('pageToCanvas', view, size, {'x': 100, 'y': 50}) == {'x': 150.0, 'y': 100.0}
    assert _call('pageToCanvas', view, size, {'x': 100, 'y': 60})['y'] < 100


# --- IFC origin from a preview tap ---


def test_ifc_tap_becomes_the_new_map_zero():
    slice_ = {
        'origin': [10.0, 20.0],
        'rotation_deg': 30.0,
        'preview': {'origin': [-1.0, -2.0, 0.0], 'resolution': 0.05, 'width': 100, 'height': 80},
    }
    u, v = 40.0, 25.0
    world = _call('ifcOriginFromTap', slice_, u, v)
    m = [-1.0 + u * 0.05, -2.0 + (80 - v) * 0.05]
    th = math.radians(30.0)
    want = [
        10.0 + math.cos(th) * m[0] - math.sin(th) * m[1],
        20.0 + math.sin(th) * m[0] + math.cos(th) * m[1],
    ]
    assert _close(world[0], want[0]) and _close(world[1], want[1])
    # slicing again with that origin makes the tapped point map (0, 0)
    frame = _call('mapFrame', 1.0, 30.0, world)
    zero = _call('pageToMap', frame, {'x': want[0], 'y': want[1]})
    assert _close(zero['x'], 0) and _close(zero['y'], 0)


def test_ifc_map_frame_matches_calib_with_unit_scale():
    s, rot, t = cal.map_frame(1.0, math.radians(45.0), [3.0, 4.0])
    want = s * (rot @ np.array([5.0, 9.0])) + t
    got = _call('pageToMap', _call('mapFrame', 1.0, 45.0, [3.0, 4.0]), {'x': 5.0, 'y': 9.0})
    assert _close(got['x'], want[0]) and _close(got['y'], want[1])


def test_preview_pixel_map_round_trip():
    pv = {'origin': [-1.0, -2.0, 0.0], 'resolution': 0.05, 'width': 100, 'height': 80}
    m = _call('previewPixelToMap', pv, 12.5, 30.0)
    back = _call('mapToPreviewPixel', pv, m)
    assert _close(back['x'], 12.5) and _close(back['y'], 30.0)


def test_fit_contain():
    fit = _call('fitContain', 200, 100, 300, 300)
    assert fit == {'k': 1.5, 'ox': 0.0, 'oy': 75.0}
    px = _call('canvasToImagePx', fit, 150, 150)
    assert px == {'x': 100.0, 'y': 50.0}


# --- wall_spans: top pair per axis ---


def _pair(a, b, span, axis, spread=0.01):
    return {
        'a': a,
        'b': b,
        'span_m': span,
        'axis_rad': axis,
        'spread_m': spread,
        'parallel_err_deg': 0.5,
    }


def test_top_pair_per_axis_keeps_the_first_of_each_axis():
    pairs = [
        _pair('w0', 'w1', 3.20, 0.01),
        _pair('w2', 'w3', 3.19, 0.02),  # doorway duplicate
        _pair('w4', 'w5', 5.10, math.pi / 2),
        _pair('w6', 'w7', 3.21, math.pi + 0.005),
        _pair('w8', 'w9', 5.09, -math.pi / 2 + 0.01),
    ]
    top = _call('topPairPerAxis', {'pairs': pairs})
    assert [(p['a'], p['span_m']) for p in top] == [('w0', 3.20), ('w4', 5.10)]


def test_axes_are_grouped_modulo_pi_and_within_15_degrees():
    near = _call('topPairPerAxis', {'pairs': [_pair('a', 'b', 2, 0.0), _pair('c', 'd', 2, 0.2)]})
    assert len(near) == 1  # 11.5 deg apart: one axis
    apart = _call('topPairPerAxis', {'pairs': [_pair('a', 'b', 2, 0.0), _pair('c', 'd', 2, 0.4)]})
    assert len(apart) == 2  # 22.9 deg apart: two axes
    wrap = _call(
        'topPairPerAxis', {'pairs': [_pair('a', 'b', 2, 0.05), _pair('c', 'd', 2, math.pi - 0.05)]}
    )
    assert len(wrap) == 1  # 0.05 and pi - 0.05 are 0.1 rad apart


def test_top_pair_per_axis_skips_unusable_pairs_and_empty_input():
    bad = [
        {'span_m': None, 'axis_rad': 0},
        {'span_m': 2.0},
        {'span_m': -1, 'axis_rad': 0},
        {'span_m': 0, 'axis_rad': 0},
        None,
    ]
    assert _call('topPairPerAxis', {'pairs': bad}) == []
    assert _call('topPairPerAxis', {}) == []
    assert _call('topPairPerAxis', None) == []


def test_axis_diff_is_undirected():
    assert _close(_call('axisDiff', 0.0, math.pi), 0.0)
    assert _close(_call('axisDiff', 0.1, math.pi - 0.1), 0.2)
    assert _close(_call('axisDiff', 0.0, math.pi / 2), math.pi / 2)


# --- request bodies ---

P1, P2 = {'x': 10.0, 'y': 20.0}, {'x': 110.0, 'y': 20.0}


def test_calibrate_body_per_method_is_flat():
    assert _call('calibrateBody', {'page': 0, 'method': 'proposal'}) == {
        'body': {'page': 0, 'method': 'proposal'}
    }
    assert _call(
        'calibrateBody',
        {
            'page': 2,
            'method': 'two_point',
            'params': {'p1': P1, 'p2': P2, 'metres': 3.5},
            'rotation_deg': 12,
            'origin': [5, 6],
        },
    ) == {
        'body': {
            'page': 2,
            'method': 'two_point',
            'p1': [10.0, 20.0],
            'p2': [110.0, 20.0],
            'metres': 3.5,
            'rotation_deg': 12,
            'origin': [5, 6],
        }
    }
    assert _call('calibrateBody', {'page': 0, 'method': 'typed', 'params': {'scale': ' 1:100 '}})[
        'body'
    ] == {'page': 0, 'method': 'typed', 'scale': '1:100'}
    assert _call(
        'calibrateBody',
        {'page': 0, 'method': 'typed', 'params': {'scale': '', 'ratio': 48, 'paper_in': [36, 24]}},
    )['body'] == {'page': 0, 'method': 'typed', 'ratio': 48, 'paper_in': [36, 24]}
    spans = [
        {'p1': P1, 'p2': P2, 'metres': 3.2},
        {'p1': P2, 'p2': {'x': 110, 'y': 90}, 'metres': 5.1},
    ]
    body = _call('calibrateBody', {'page': 0, 'method': 'robot', 'params': {'spans': spans}})[
        'body'
    ]
    assert body['spans'] == [
        {'p1': [10.0, 20.0], 'p2': [110.0, 20.0], 'metres': 3.2},
        {'p1': [110.0, 20.0], 'p2': [110.0, 90.0], 'metres': 5.1},
    ]
    assert 'params' not in body


def test_calibrate_body_optional_fields_only_when_set():
    body = _call('calibrateBody', {'page': 0, 'method': 'proposal', 'rotation_deg': 0})['body']
    assert body == {'page': 0, 'method': 'proposal', 'rotation_deg': 0}
    assert (
        'force_raster'
        not in _call('calibrateBody', {'page': 0, 'method': 'proposal', 'force_raster': False})[
            'body'
        ]
    )
    full = _call(
        'calibrateBody',
        {
            'page': 1,
            'method': 'proposal',
            'force_raster': True,
            'resolution': 0.1,
            'close_m': 0.9,
            'origin': {'x': 1, 'y': 2},
        },
    )['body']
    assert full == {
        'page': 1,
        'method': 'proposal',
        'force_raster': True,
        'resolution': 0.1,
        'close_m': 0.9,
        'origin': [1, 2],
    }


@pytest.mark.parametrize(
    'opts',
    [
        {'page': -1, 'method': 'proposal'},
        {'page': 1.5, 'method': 'proposal'},
        {'page': None, 'method': 'proposal'},
        {'page': 0, 'method': 'magic'},
        {'page': 0, 'method': 'two_point', 'params': {'p1': P1, 'metres': 2}},
        {'page': 0, 'method': 'two_point', 'params': {'p1': P1, 'p2': P2, 'metres': 0}},
        {'page': 0, 'method': 'two_point', 'params': {'p1': P1, 'p2': P2, 'metres': None}},
        {'page': 0, 'method': 'typed', 'params': {}},
        {'page': 0, 'method': 'typed', 'params': {'scale': '1:100', 'ratio': 100}},
        {'page': 0, 'method': 'typed', 'params': {'ratio': 0}},
        {'page': 0, 'method': 'typed', 'params': {'ratio': 50, 'paper_in': [36]}},
        {'page': 0, 'method': 'typed', 'params': {'ratio': 50, 'paper_in': [36, 0]}},
        {'page': 0, 'method': 'robot', 'params': {'spans': []}},
        {'page': 0, 'method': 'robot', 'params': {'spans': [{'p1': P1, 'p2': P2, 'metres': 2}]}},
        {
            'page': 0,
            'method': 'robot',
            'params': {'spans': [{'p1': P1, 'p2': P2, 'metres': 2}, {'p1': P1, 'metres': 3}]},
        },
        {'page': 0, 'method': 'proposal', 'rotation_deg': 'x'},
    ],
)
def test_calibrate_body_errors(opts):
    out = _call('calibrateBody', opts)
    assert 'error' in out and 'body' not in out and out['error']


def test_slice_body_sends_only_what_is_set():
    assert _call('sliceBody', {}) == {'body': {}}
    assert _call('sliceBody', {'slice_height': 1.1, 'rotation_deg': 10, 'origin': [1, 2]}) == {
        'body': {'slice_height': 1.1, 'rotation_deg': 10, 'origin': [1, 2]}
    }
    assert 'error' in _call('sliceBody', {'slice_height': 0})
    assert 'error' in _call('sliceBody', {'rotation_deg': 'x'})


def test_install_body_has_no_force_and_validates():
    assert _call('installBody', {'name': 'plan'}) == {'body': {'name': 'plan'}}
    assert _call('installBody', {'name': 'plan', 'label': ' Lobby ', 'floor': '3'}) == {
        'body': {'name': 'plan', 'label': 'Lobby', 'floor': 3}
    }
    assert _call('installBody', {'name': 'plan', 'label': '  ', 'floor': ''}) == {
        'body': {'name': 'plan'}
    }
    assert _call('installBody', {'name': 'plan', 'floor': -1})['body']['floor'] == -1
    for bad in (
        {'name': 'Bad'},
        {'name': ''},
        {'name': 'plan\n'},
        {'name': 'plan', 'floor': '1.5'},
        {'name': 'plan', 'floor': 'x'},
        {'name': 'plan', 'map_start_pose': [1, 2]},
    ):
        assert 'error' in _call('installBody', bad)
    for ok_body in ({'name': 'plan', 'label': 'x', 'floor': 1, 'map_start_pose': [0, 0, 0]},):
        assert 'force' not in _call('installBody', {**ok_body, 'force': True})['body']


def test_no_force_anywhere_in_the_wizard_sources():
    for name in ('importmath.js', 'importflow.js', 'importwizard.js'):
        text = (REPO_ROOT / 'webui' / name).read_text().lower()
        assert 'force:' not in text.replace('force_raster', '').replace('forceraster', '')
    html = (REPO_ROOT / 'webui' / 'index.html').read_text().lower()
    start = html.index('id="import-panel"')
    assert 'force' not in html[start : html.index('</section>', start)].replace(
        'read as image', ''
    ).replace('imp-force-raster', '')


# --- importer address, retry, robot URL ---


@pytest.mark.parametrize(
    'text,expect',
    [
        ('http://box:9004', 'http://box:9004/api'),
        ('box:9004', 'http://box:9004/api'),
        ('http://box:9004/', 'http://box:9004/api'),
        ('http://box:9004/api', 'http://box:9004/api'),
        ('http://box:9004/api/', 'http://box:9004/api'),
        ('HTTPS://x.example', 'HTTPS://x.example/api'),
        ('  192.168.1.5:9004  ', 'http://192.168.1.5:9004/api'),
        ('', None),
        (None, None),
        ('http://', None),
        ('http://a b:9004', None),
        ('http://u:p@host/', None),
        ('ftp://x', None),
        ('file:///etc/passwd', None),
        ('javascript://x', None),
    ],
)
def test_normalize_base(text, expect):
    assert _call('normalizeBase', text) == expect


def test_resolve_base_prefers_the_remembered_address_then_the_companion():
    comp = {'configured': True, 'host': '10.0.0.7'}
    assert _call('resolveBase', 'laptop:9004', comp) == 'http://laptop:9004/api'
    assert _call('resolveBase', None, comp) == 'http://10.0.0.7:9004/api'
    assert _call('resolveBase', '', comp) == 'http://10.0.0.7:9004/api'
    assert _call('resolveBase', 'http://', comp) == 'http://10.0.0.7:9004/api'
    assert _call('resolveBase', None, {'configured': False, 'host': 'x'}) is None
    assert _call('resolveBase', None, {'configured': True}) is None
    assert _call('resolveBase', None, None) is None


@pytest.mark.parametrize(
    'header,ms',
    [
        ('3', 3000),
        ('0.2', 1000),
        ('0', 2000),
        ('-4', 2000),
        ('abc', 2000),
        (None, 2000),
        ('120', 10000),
        ('2.5', 2500),
    ],
)
def test_retry_delay_is_bounded(header, ms):
    assert _call('retryDelayMs', header) == ms


def test_wall_spans_url_has_no_query_string():
    url = _call('wallSpansUrl', '10.1.57.104')
    assert url == 'http://10.1.57.104:9001/api/wall_spans'
    assert '?' not in url and '&' not in url


def test_describe_error_passes_server_text_through():
    assert (
        _call('describeError', 409, {'error': "name 'x' is a scanned map"})
        == "name 'x' is a scanned map"
    )
    assert _call(
        'describeError', 502, {'error': 'fleet_status unreachable', 'upstream': 'http://pi:9003'}
    ) == ('fleet_status unreachable (upstream: http://pi:9003)')
    assert _call('describeError', 413, {'error': 'too many faces', 'count': 9, 'limit': 5}) == (
        'too many faces (9 > 5)'
    )
    assert _call('describeError', 500, None) == 'HTTP 500'
    assert _call('describeError', 422, {'error': ''}) == 'HTTP 422'


# --- close_m (door gap to close) -------------------------------------------------------------


@pytest.mark.parametrize(
    'text,expect',
    [
        ('', {}), ('   ', {}), (None, {}), ('1.2', {'value': 1.2}), ('0.1', {'value': 0.1}),
        ('5', {'value': 5}), (' 2.5 ', {'value': 2.5}),
    ],
)
def test_parse_close_m_blank_means_the_server_default(text, expect):
    assert _call('parseCloseM', text) == expect


@pytest.mark.parametrize('text', ['0', '-1', '5.01', '100', 'abc', '1,2', 'NaN', 'Infinity'])
def test_parse_close_m_rejects_out_of_range_and_junk(text):
    out = _call('parseCloseM', text)
    assert out == {'error': 'Door gap must be more than 0 and at most 5 m'}


def test_close_m_is_forwarded_in_calibrate_and_slice_bodies_only_when_set():
    assert _call('calibrateBody', {'page': 0, 'method': 'proposal', 'close_m': 2.5})['body'][
        'close_m'
    ] == 2.5
    assert 'close_m' not in _call('calibrateBody', {'page': 0, 'method': 'proposal'})['body']
    nulled = _call('calibrateBody', {'page': 0, 'method': 'proposal', 'close_m': None})
    assert 'close_m' not in nulled['body']
    assert _call('sliceBody', {'close_m': 5}) == {'body': {'close_m': 5}}
    assert _call('sliceBody', {}) == {'body': {}}
    assert _call('sliceBody', {'close_m': None}) == {'body': {}}


@pytest.mark.parametrize('bad', [0, -0.5, 5.5, 'x', True])
def test_close_m_out_of_range_is_refused_by_both_body_builders(bad):
    assert 'error' in _call('calibrateBody', {'page': 0, 'method': 'proposal', 'close_m': bad})
    assert 'error' in _call('sliceBody', {'close_m': bad})
