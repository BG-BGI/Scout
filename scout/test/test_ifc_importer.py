"""IFC jobs on the importer HTTP server (docker/floorplan-importer/server.py + ifcjob.py).

Real server over loopback, the committed two_storey.ifc fixture (Level 1 z=0 with 0.2 m walls,
Level 2 z=3 with a window, Level 3 z=6 empty), a stub fleet_status, and the REAL fleet-status
handler for the install integration. Skipped when ifcopenshell, shapely or PyMuPDF is absent.
"""
import base64
import hashlib
import json
import math
import pathlib

import numpy as np
import pytest
from importer_harness import (
    ROOT,
    Upstream,
    closed_port_url,
    load_server,
    pgm_grid,
    real_fleet,
    start_app,
    stop_app,
)

pytest.importorskip('ifcopenshell')
pytest.importorskip('ifcopenshell.geom')
pytest.importorskip('shapely')
pymupdf = pytest.importorskip('pymupdf')

srv = load_server('fp_importer_server_ifc')
pr = srv.planraster
FIXTURE = pathlib.Path(__file__).resolve().parent / 'fixtures' / 'two_storey.ifc'
IFC = FIXTURE.read_bytes()
RES = 0.05
CONTRACT_STOREY_KEYS = {'id', 'global_id', 'name', 'label', 'elevation_m', 'floor', 'slice',
                        'last_install'}
CONTRACT_SLICE_KEYS = {'storey', 'z', 'slice_height', 'rotation_deg', 'origin', 'resolution',
                       'close_m', 'bounds', 'faces', 'preview', 'warnings'}
CONTRACT_JOB_KEYS = {'id', 'kind', 'filename', 'sha256', 'bytes', 'expires_in', 'pages',
                     'storeys'}


@pytest.fixture
def make_app():
    started = []

    def make(fleet_url=None, **kwargs):
        app = start_app(srv, fleet_url, **kwargs)
        started.append(app)
        return app

    yield make
    for app in started:
        stop_app(srv, app)


@pytest.fixture
def app(make_app):
    return make_app('http://127.0.0.1:9')


@pytest.fixture
def upstream():
    made = []

    def make(**kwargs):
        made.append(Upstream(**kwargs))
        return made[-1]

    yield make
    for stub in made:
        stub.close()


def new_ifc(app, data=IFC, filename='tower.ifc'):
    status, out, _ = app.upload(data, filename)
    assert status == 201, out
    return out


def level_ids(out):
    return {s['label']: s['id'] for s in out['storeys']}


def slice_path(job, sid, tail='slice'):
    return f'/api/import/{job}/storey/{sid}/{tail}'


def do_slice(app, job, sid, **body):
    return app.jcall('POST', slice_path(job, sid), body)


def preview(app, job, sid):
    status, raw, _ = app.call('GET', slice_path(job, sid, 'preview.pgm'))
    assert status == 200
    return raw


def runs(line, value=pr.OCCUPIED):
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


def row_at(grid, origin, y, res=RES):
    return grid[grid.shape[0] - 1 - int(math.floor((y - origin[1]) / res))]


def tiny_pdf():
    doc = pymupdf.open()
    page = doc.new_page(width=600, height=400)
    page.draw_line((10, 10), (500, 10), width=4)
    return doc.tobytes()


# --- upload and listing ----------------------------------------------------------------------

def test_upload_lists_the_storeys_in_metres_with_floor_numbers(app):
    out = new_ifc(app)
    assert out['kind'] == 'ifc' and out['pages'] == [] and out['filename'] == 'tower.ifc'
    assert out['sha256'] == hashlib.sha256(IFC).hexdigest() and out['bytes'] == len(IFC)
    assert set(out) == {'id', 'kind', 'filename', 'sha256', 'bytes', 'pages', 'storeys'}
    storeys = out['storeys']
    assert [(s['floor'], s['label'], s['elevation_m']) for s in storeys] == [
        (1, 'Level 1', 0.0), (2, 'Level 2', 3.0), (3, 'Level 3', 6.0)]
    for s in storeys:
        assert set(s) == CONTRACT_STOREY_KEYS
        assert isinstance(s['id'], int) and s['global_id'] and s['name'] == s['label']
        assert s['slice'] is None and s['last_install'] is None
    assert len({s['id'] for s in storeys}) == 3


def test_get_job_state_has_exactly_the_contract_keys(app):
    out = new_ifc(app)
    state = app.state(out['id'])
    assert set(state) == CONTRACT_JOB_KEYS and state['kind'] == 'ifc' and state['pages'] == []
    assert state['storeys'] == out['storeys'] and 0 < state['expires_in'] <= srv.JOB_TTL_S
    assert (state['sha256'], state['bytes']) == (out['sha256'], out['bytes'])


def test_filename_defaults_and_is_sanitised(app):
    assert new_ifc(app, filename=None)['filename'] == 'upload.ifc'
    assert new_ifc(app, filename='../../etc/x.ifc')['filename'] == 'x.ifc'


def test_the_raw_bytes_are_not_kept_after_the_parse(app):
    job = app.importer.jobs[new_ifc(app)['id']]
    held = [v for v in vars(job).values() if isinstance(v, (bytes, bytearray, memoryview))]
    assert held == [] and job.model is not None and job.n_bytes == len(IFC)
    assert all(not isinstance(v, (bytes, bytearray)) for v in vars(job.storeys[
        next(iter(job.storeys))]).values())


def test_pdf_jobs_gain_an_empty_storeys_list(app):
    status, out, _ = app.upload(tiny_pdf())
    assert status == 201 and out['storeys'] == [] and out['kind'] == 'pdf'
    assert app.state(out['id'])['storeys'] == []


def test_health_reports_ifc_support(app):
    assert app.jcall('GET', '/api/health')[1]['ifc'] is True


# --- slicing (AE5: no calibration step) ----------------------------------------------------------

def test_slicing_both_levels_and_installing_them_never_calibrates(make_app, upstream, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('calibrate must not run for IFC')

    monkeypatch.setattr(srv.Importer, 'calibrate', forbidden)
    stub = upstream()
    app = make_app(stub.url)
    out = new_ifc(app)
    ids = level_ids(out)
    for label, floor in (('Level 1', 1), ('Level 2', 2)):
        status, sl, _ = do_slice(app, out['id'], ids[label])
        assert status == 200 and set(sl) == CONTRACT_SLICE_KEYS, sl
        assert sl['storey'] == {'id': ids[label], 'label': label,
                                'elevation_m': (floor - 1) * 3.0, 'floor': floor}
    for label, name in (('Level 1', 'l1'), ('Level 2', 'l2')):
        status, body, _ = app.jcall('POST', slice_path(out['id'], ids[label], 'install'),
                                    {'name': name})
        assert status == 200 and body['ok'] is True
    assert len(stub.requests) == 2
    first, second = stub.payload(0), stub.payload(1)
    for payload, name, floor, label in ((first, 'l1', 1, 'Level 1'), (second, 'l2', 2, 'Level 2')):
        assert payload['name'] == name and payload['source'] == 'drawing'
        assert payload['floor'] == floor and payload['label'] == label
        assert set(payload) == {'name', 'yaml', 'pgm_b64', 'sidecar', 'source', 'label', 'floor'}
        assert 'free_thresh: 0.196' in payload['yaml'].splitlines()
    assert base64.b64decode(first['pgm_b64']) == preview(app, out['id'], ids['Level 1'])
    assert base64.b64decode(second['pgm_b64']) == preview(app, out['id'], ids['Level 2'])
    assert first['pgm_b64'] != second['pgm_b64']
    assert app.call('POST', f"/api/import/{out['id']}/calibrate",
                    json.dumps({'method': 'proposal'}), {'Content-Type': 'application/json'}
                    )[0] == 409


def test_a_wall_of_0_2_m_is_a_four_cell_band_through_http(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    status, sl, _ = do_slice(app, out['id'], sid)
    assert status == 200
    assert sl['bounds'] == pytest.approx([-0.1, -0.1, 6.1, 4.1], abs=1e-6)
    assert sl['origin'] == pytest.approx([-0.1, -0.1], abs=1e-6)
    assert (sl['resolution'], sl['close_m'], sl['rotation_deg']) == (0.05, 1.2, 0.0)
    assert sl['slice_height'] == pytest.approx(0.241) and sl['z'] == pytest.approx(0.241)
    assert sl['faces'] > 0 and sl['warnings'] == []
    grid = pgm_grid(preview(app, out['id'], sid))
    assert runs(row_at(grid, sl['preview']['origin'], 3.5)) == [4, 4, 4]
    prev = sl['preview']
    assert prev['occupied_extent_m']['width'] == pytest.approx(6.2, abs=0.06)
    assert prev['occupied_extent_m']['height'] == pytest.approx(4.2, abs=0.06)
    assert prev['cells']['free'] > 8000 and prev['interior_fraction'] > 0.8
    assert (prev['width'], prev['height']) == (grid.shape[1], grid.shape[0])


def test_the_doorway_in_the_partition_stays_free_and_the_room_is_sealed(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    sl = do_slice(app, out['id'], sid)[1]
    grid = pgm_grid(preview(app, out['id'], sid))
    origin = sl['preview']['origin']

    def at(x, y):
        """x, y in IFC metres; the default origin makes IFC (-0.1, -0.1) the map's (0, 0)."""
        mx, my = x + 0.1, y + 0.1
        return int(row_at(grid, origin, my)[int(math.floor((mx - origin[0]) / RES))])

    assert at(1.025, 3.025) == pr.FREE and at(5.025, 3.025) == pr.FREE
    assert at(3.025, 2.025) == pr.FREE and at(3.025, 3.525) == pr.OCCUPIED
    assert at(-0.575, 2.025) == pr.UNKNOWN


def test_state_shows_the_slice_after_slicing(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    sl = do_slice(app, out['id'], sid, slice_height=0.5)[1]
    storey = {s['id']: s for s in app.state(out['id'])['storeys']}[sid]
    assert storey['slice'] == sl and storey['slice']['slice_height'] == 0.5
    assert all(s['slice'] is None for s in app.state(out['id'])['storeys'] if s['id'] != sid)


def test_a_new_slice_height_re_slices_and_replaces_the_grid(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    first = do_slice(app, out['id'], sid)[1]
    pgm_first = preview(app, out['id'], sid)
    second = do_slice(app, out['id'], sid, slice_height=4.0)[1]
    pgm_second = preview(app, out['id'], sid)
    assert second['z'] == pytest.approx(4.0) and second['slice_height'] == 4.0
    assert pgm_first != pgm_second
    assert second['preview']['cells']['occupied'] < first['preview']['cells']['occupied']
    assert second['bounds'] == pytest.approx([-0.1, -0.1, 4.1, 3.1], abs=0.05)
    assert app.state(out['id'])['storeys'][0]['slice'] == second
    window = level_ids(out)['Level 2']
    low = do_slice(app, out['id'], window)[1]['preview']['cells']['occupied']
    high = do_slice(app, out['id'], window, slice_height=1.0)[1]
    assert high['storey']['floor'] == 2 and high['z'] == pytest.approx(4.0)
    assert high['preview']['cells']['occupied'] != low


def test_rotation_origin_resolution_and_close_m_are_applied_and_echoed(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    flat = do_slice(app, out['id'], sid)[1]['preview']['occupied_extent_m']
    turned = do_slice(app, out['id'], sid, rotation_deg=90)[1]
    assert turned['rotation_deg'] == 90
    swapped = turned['preview']['occupied_extent_m']['width']
    assert swapped == pytest.approx(flat['height'], abs=0.1)
    coarse = do_slice(app, out['id'], sid, origin=[0, 0], resolution=0.1, close_m=2.0)[1]
    assert coarse['origin'] == [0.0, 0.0] and coarse['resolution'] == 0.1
    assert coarse['close_m'] == 2.0 and coarse['preview']['resolution'] == 0.1
    grid = pgm_grid(preview(app, out['id'], sid))
    assert runs(row_at(grid, coarse['preview']['origin'], 3.5, 0.1)) == [2, 2, 2]


def test_a_failed_reslice_keeps_the_previous_grid(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 2']
    good = do_slice(app, out['id'], sid)[1]
    before = preview(app, out['id'], sid)
    status, err, _ = do_slice(app, out['id'], sid, slice_height=4.9)
    assert status == 422 and 'Level 2' in err['error']
    assert preview(app, out['id'], sid) == before
    assert app.state(out['id'])['storeys'][1]['slice'] == good


# --- errors -------------------------------------------------------------------------------------

def test_an_empty_storey_is_422_naming_it(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 3']
    status, err, _ = do_slice(app, out['id'], sid)
    assert status == 422 and 'Level 3' in err['error'] and 'nothing solid' in err['error']
    assert app.call('GET', slice_path(out['id'], sid, 'preview.pgm'))[0] == 409
    assert app.state(out['id'])['storeys'][2]['slice'] is None


@pytest.mark.parametrize('blob', [
    IFC[:len(IFC) // 2],
    b"ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n#1=NONSENSE(;\nENDSEC;\n"
    b"END-ISO-10303-21;\n",
    b'ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n',
    b'ISO-10303-21;\n',
])
def test_an_unparsable_or_storeyless_ifc_is_422(app, blob):
    status, err, _ = app.upload(blob)
    assert status == 422 and err['error'], blob
    assert app.jcall('GET', '/api/health')[1]['jobs'] == 0


def test_face_budget_exceeded_is_413_with_the_count(make_app):
    app = make_app(None, ifc_face_budget=10)
    out = new_ifc(app)
    status, err, _ = do_slice(app, out['id'], level_ids(out)['Level 1'])
    assert status == 413 and err['limit'] == 10 and err['count'] > 10 and err['error']
    assert app.state(out['id'])['storeys'][0]['slice'] is None


def test_an_ifc_over_the_cap_is_413_with_count_and_limit(make_app):
    app = make_app(None, ifc_max_bytes=1000)
    status, err, _ = app.upload(IFC)
    assert status == 413 and err['limit'] == 1000 and err['count'] == len(IFC)
    assert app.jcall('GET', '/api/health')[1]['jobs'] == 0


def test_an_oversize_ifc_is_refused_after_the_sniff_without_reading_the_rest(make_app):
    app = make_app(None, ifc_max_bytes=100_000)
    status, err = app.head_only(srv.pdfsource.MAX_PDF_BYTES, IFC[:srv.HEADER_SNIFF].ljust(
        srv.HEADER_SNIFF, b' '))
    assert status == 413 and err['limit'] == 100_000
    assert err['count'] == srv.pdfsource.MAX_PDF_BYTES


def test_the_default_cap_fits_the_documented_budget():
    """The arithmetic in the comment above MAX_IFC_BYTES, with the compose limit of 2g."""
    assert srv.MAX_IFC_BYTES == 80 * 1024 * 1024 and srv.IFC_THREADS == 1
    assert srv.IFC_PEAK_FACTOR == 15 and srv.MEMORY_HEADROOM == 0.9
    mem, base, pdf_job, grids = 2048, 100, 192, 128
    slice_peak = base + 2 * pdf_job + grids + srv.IFC_PEAK_FACTOR * srv.MAX_IFC_BYTES / 2**20
    assert slice_peak <= srv.MEMORY_HEADROOM * mem


def test_unknown_storey_is_404_before_anything_else(app):
    out = new_ifc(app)
    job = out['id']
    for method, tail in (('POST', 'slice'), ('GET', 'preview.png'), ('GET', 'preview.pgm'),
                         ('POST', 'install')):
        status, raw, _ = app.call(method, slice_path(job, 999999, tail), b'{}' if method == 'POST'
                                  else None, {'Content-Type': 'application/json'})
        assert status == 404 and 'storey' in json.loads(raw)['error'], tail
    assert app.call('POST', f'/api/import/{job}/storey/abc/slice', b'{}')[0] == 404


def test_cross_kind_routes_are_409_naming_the_other_routes(app):
    ifc = new_ifc(app)['id']
    pdf = app.upload(tiny_pdf())[1]['id']
    json_h = {'Content-Type': 'application/json'}
    for method, tail, body in (('GET', 'page/0', None), ('GET', 'page/0.png', None),
                               ('POST', 'calibrate', b'{"method": "proposal"}'),
                               ('GET', 'preview.png', None), ('GET', 'preview.pgm', None),
                               ('POST', 'install', b'{"name": "x"}')):
        status, raw, _ = app.call(method, f'/api/import/{ifc}/{tail}', body, json_h)
        assert status == 409 and '/storey/' in json.loads(raw)['error'], tail
    sid = level_ids(app.state(ifc) | {'storeys': app.state(ifc)['storeys']})['Level 1']
    for method, tail, body in (('POST', f'storey/{sid}/slice', b'{}'),
                               ('GET', f'storey/{sid}/preview.png', None),
                               ('GET', f'storey/{sid}/preview.pgm', None),
                               ('POST', f'storey/{sid}/install', b'{"name": "x"}')):
        status, raw, _ = app.call(method, f'/api/import/{pdf}/{tail}', body, json_h)
        assert status == 409 and '/calibrate' in json.loads(raw)['error'], tail
    assert app.call('GET', f'/api/import/{ifc}')[0] == 200
    assert app.call('GET', f'/api/import/{pdf}')[0] == 200
    assert app.call('DELETE', f'/api/import/{ifc}')[0] == 200
    assert app.call('DELETE', f'/api/import/{pdf}')[0] == 200


@pytest.mark.parametrize('body', [
    {'surprise': 1}, {'slice_height': 0}, {'slice_height': 5.1}, {'slice_height': 'x'},
    {'slice_height': True}, {'slice_height': float('nan')}, {'rotation_deg': 'x'},
    {'rotation_deg': 99999}, {'origin': [1]}, {'origin': 'x'}, {'origin': [1, 'a']},
    {'resolution': 0}, {'resolution': 5}, {'close_m': 0}, {'close_m': 99},
    {'calibrate': 1}, {'method': 'proposal'}, {'wall_options': {}},
])
def test_bad_slice_bodies_are_400(app, body):
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    status, err, _ = app.call('POST', slice_path(out['id'], sid), json.dumps(body, allow_nan=True),
                              {'Content-Type': 'application/json'})
    assert status == 400 and json.loads(err)['error'], body
    assert app.state(out['id'])['storeys'][0]['slice'] is None


def test_slice_body_must_be_a_json_object(app):
    out = new_ifc(app)
    path = slice_path(out['id'], level_ids(out)['Level 1'])
    for blob in (b'[1]', b'"x"', b'{oops'):
        assert app.call('POST', path, blob, {'Content-Type': 'application/json'})[0] == 400
    assert app.call('POST', path, b'', {'Content-Type': 'application/json'})[0] == 200


@pytest.mark.parametrize('body', [
    {}, {'name': ''}, {'name': 7}, {'name': 'x', 'force': True}, {'name': 'x', 'url': 'http://e'},
    {'name': 'x', 'yaml': 'z'}, {'name': 'x', 'label': 3}, {'name': 'x', 'floor': 'one'},
    {'name': 'x', 'floor': True}, {'name': 'x', 'map_start_pose': [1, 2]},
    {'name': 'x', 'fleet_status_url': 'http://evil'},
])
def test_bad_install_bodies_are_400_and_send_nothing(make_app, upstream, body):
    stub = upstream()
    app = make_app(stub.url)
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    do_slice(app, out['id'], sid)
    status, _, _ = app.jcall('POST', slice_path(out['id'], sid, 'install'), body)
    assert status == 400 and not stub.requests, body


def test_install_before_slicing_is_409_and_unconfigured_is_503(make_app, upstream):
    stub = upstream()
    app = make_app(stub.url)
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    status, err, _ = app.jcall('POST', slice_path(out['id'], sid, 'install'), {'name': 'x'})
    assert status == 409 and 'slice' in err['error'] and not stub.requests
    bare = make_app(None)
    out2 = new_ifc(bare)
    sid2 = level_ids(out2)['Level 1']
    do_slice(bare, out2['id'], sid2)
    status, err, _ = bare.jcall('POST', slice_path(out2['id'], sid2, 'install'), {'name': 'x'})
    assert status == 503 and 'FLEET_STATUS_URL' in err['error']


# --- install -------------------------------------------------------------------------------------

def test_install_defaults_label_and_floor_and_accepts_overrides(make_app, upstream):
    stub = upstream()
    app = make_app(stub.url)
    out = new_ifc(app)
    sid = level_ids(out)['Level 2']
    do_slice(app, out['id'], sid)
    path = slice_path(out['id'], sid, 'install')
    assert app.jcall('POST', path, {'name': 'up'})[0] == 200
    assert (stub.payload()['label'], stub.payload()['floor']) == ('Level 2', 2)
    body = {'name': 'up', 'label': 'Mezzanine', 'floor': 7, 'map_start_pose': [1, 2, 0.5]}
    assert app.jcall('POST', path, body)[0] == 200
    sent = stub.payload()
    assert (sent['label'], sent['floor']) == ('Mezzanine', 7)
    assert sent['map_start_pose'] == [1.0, 2.0, 0.5]
    assert app.jcall('POST', path, {'name': 'up', 'floor': None})[0] == 200
    assert stub.payload()['floor'] is None and stub.payload()['label'] == 'Level 2'


def test_the_sidecar_follows_the_frozen_shape(make_app, upstream):
    stub = upstream()
    app = make_app(stub.url)
    out = new_ifc(app, filename='tower.ifc')
    ids = level_ids(out)
    sid = ids['Level 2']
    sl = do_slice(app, out['id'], sid, slice_height=0.5, rotation_deg=10, origin=[0.0, 0.0],
                  resolution=0.1, close_m=1.5)[1]
    assert app.jcall('POST', slice_path(out['id'], sid, 'install'), {'name': 'up'})[0] == 200
    side = stub.payload()['sidecar']
    assert set(side) == {'importer_version', 'kind', 'source_file', 'storey', 'slice',
                         'created_utc'}
    assert side['kind'] == 'ifc' and side['importer_version'] == 'dev'
    assert side['source_file'] == {'name': 'tower.ifc', 'sha256': out['sha256'],
                                   'bytes': len(IFC)}
    storey = {s['id']: s for s in out['storeys']}[sid]
    assert side['storey'] == {'id': sid, 'global_id': storey['global_id'], 'name': 'Level 2',
                              'label': 'Level 2', 'elevation_m': 3.0}
    assert side['slice'] == {'slice_height': 0.5, 'z': pytest.approx(3.5), 'rotation_deg': 10,
                             'origin': [0.0, 0.0], 'resolution': 0.1, 'close_m': 1.5,
                             'faces': sl['faces']}
    assert side['created_utc'].endswith('+00:00')
    assert len(json.dumps(side)) < 4096
    yaml_lines = stub.payload()['yaml'].splitlines()
    assert 'resolution: 0.1' in yaml_lines


def test_install_results_are_recorded_per_storey_and_relayed(make_app, upstream):
    stub = upstream(status=409, body=b'{"error": "map name taken by a scan map"}')
    app = make_app(stub.url)
    out = new_ifc(app)
    ids = level_ids(out)
    for label in ('Level 1', 'Level 2'):
        do_slice(app, out['id'], ids[label])
    status, body, _ = app.jcall('POST', slice_path(out['id'], ids['Level 1'], 'install'),
                                {'name': 'x'})
    assert status == 409 and body == {'error': 'map name taken by a scan map'}
    storeys = {s['label']: s for s in app.state(out['id'])['storeys']}
    assert storeys['Level 1']['last_install']['status'] == 409
    assert storeys['Level 1']['last_install']['response'] == body
    assert storeys['Level 2']['last_install'] is None


def test_an_unreachable_fleet_status_is_502_per_storey_and_the_job_survives(make_app):
    url = closed_port_url()
    app = make_app(url)
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    do_slice(app, out['id'], sid)
    status, err, _ = app.jcall('POST', slice_path(out['id'], sid, 'install'), {'name': 'x'})
    assert status == 502 and err['upstream'] == url + '/api/maps/install'
    state = app.state(out['id'])['storeys'][0]
    assert state['last_install']['status'] == 502 and state['slice'] is not None
    assert preview(app, out['id'], sid)


def test_both_levels_install_into_the_real_fleet_status(make_app, tmp_path):
    with real_fleet(tmp_path) as fleet:
        app = make_app(fleet.url)
        out = new_ifc(app)
        ids = level_ids(out)
        for label, name in (('Level 1', 'ground'), ('Level 2', 'upper')):
            do_slice(app, out['id'], ids[label])
            status, body, _ = app.jcall('POST', slice_path(out['id'], ids[label], 'install'),
                                        {'name': name})
            assert status == 200 and body['replaced'] is False and body['source'] == 'drawing'
            assert (fleet.maps / f'{name}.pgm').read_bytes() == preview(app, out['id'], ids[label])
        sidecar = json.loads((fleet.maps / 'upper.drawing.json').read_text())
        assert sidecar['kind'] == 'ifc' and sidecar['storey']['label'] == 'Level 2'
        site = json.loads(fleet.site_json.read_text())
        assert site['active_map'] == 'floor1'
        assert site['maps']['ground']['floor'] == 1 and site['maps']['upper']['floor'] == 2
        assert site['maps']['ground']['source'] == site['maps']['upper']['source'] == 'drawing'
        assert site['maps']['upper']['label'] == 'Level 2'
        again = app.jcall('POST', slice_path(out['id'], ids['Level 2'], 'install'),
                          {'name': 'upper'})
        assert again[0] == 200 and again[1]['replaced'] is True


# --- limits, retention, threads ------------------------------------------------------------------

def test_a_second_heavy_operation_gets_429_for_slices_and_ifc_uploads(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    with app.importer.heavy:
        status, raw, resp = app.call('POST', slice_path(out['id'], sid), b'{}',
                                     {'Content-Type': 'application/json'})
        assert status == 429 and json.loads(raw) == {'error': 'busy'}
        assert int(resp.getheader('Retry-After')) >= 1
        assert app.upload(IFC)[0] == 429
        assert app.call('GET', f"/api/import/{out['id']}")[0] == 200
    assert do_slice(app, out['id'], sid)[0] == 200


def test_a_job_mid_operation_answers_429_on_the_storey_routes(app):
    out = new_ifc(app)
    sid = level_ids(out)['Level 1']
    do_slice(app, out['id'], sid)
    bodies = {'slice': b'{}', 'install': b'{"name": "x"}'}
    with app.importer.jobs[out['id']].lock:
        for method, tail in (('POST', 'slice'), ('POST', 'install'), ('GET', 'preview.pgm'),
                             ('GET', 'preview.png')):
            status, _, _ = app.call(method, slice_path(out['id'], sid, tail), bodies.get(tail),
                                    {'Content-Type': 'application/json'})
            assert status == 429, tail
        assert app.call('DELETE', f"/api/import/{out['id']}")[0] == 429


def test_only_one_ifc_model_is_open_and_the_old_one_is_released(app):
    first = new_ifc(app)['id']
    old = app.importer.jobs[first]
    second = new_ifc(app)['id']
    assert app.call('GET', f'/api/import/{first}')[0] == 404
    assert old.model is None and old.storeys == {}
    assert app.call('GET', f'/api/import/{second}')[0] == 200
    assert app.jcall('GET', '/api/health')[1]['jobs'] == 1


def test_an_ifc_upload_leaves_pdf_jobs_alone_when_there_is_room(app):
    pdf = app.upload(tiny_pdf())[1]['id']
    ifc = new_ifc(app)['id']
    assert app.call('GET', f'/api/import/{pdf}')[0] == 200
    assert app.call('GET', f'/api/import/{ifc}')[0] == 200


def test_an_ifc_upload_evicts_the_oldest_idle_job_before_parsing_when_full(make_app):
    app = make_app(None, max_jobs=2)
    first, second = (app.upload(tiny_pdf())[1]['id'] for _ in range(2))
    ifc = new_ifc(app)['id']
    assert app.call('GET', f'/api/import/{first}')[0] == 404
    assert app.call('GET', f'/api/import/{second}')[0] == 200
    assert app.call('GET', f'/api/import/{ifc}')[0] == 200


def test_a_bad_ifc_still_evicts_the_open_model_because_memory_has_no_room_for_two(app):
    good = new_ifc(app)['id']
    assert app.upload(b'ISO-10303-21;\nGARBAGE\n')[0] == 422
    assert app.call('GET', f'/api/import/{good}')[0] == 404


def test_a_busy_ifc_model_blocks_a_second_ifc_upload_with_429(app):
    first = new_ifc(app)['id']
    with app.importer.jobs[first].lock:
        status, err, resp = app.upload(IFC)
        assert status == 429 and resp.getheader('Retry-After') and 'error' in err
    assert app.call('GET', f'/api/import/{first}')[0] == 200
    assert app.jcall('GET', '/api/health')[1]['jobs'] == 1


def test_every_slot_busy_refuses_a_pdf_upload_even_though_ifc_is_idle_elsewhere(make_app):
    app = make_app(None, max_jobs=1)
    first = new_ifc(app)['id']
    with app.importer.jobs[first].lock:
        assert app.upload(tiny_pdf())[0] == 429


def test_delete_and_ttl_release_the_model(make_app):
    now = [0.0]
    app = make_app(None, clock=lambda: now[0], ttl=100)
    first = new_ifc(app)['id']
    job = app.importer.jobs[first]
    now[0] += 101
    assert app.call('GET', f'/api/import/{first}')[0] == 404
    assert job.model is None
    second = new_ifc(app)['id']
    held = app.importer.jobs[second]
    assert app.call('DELETE', f'/api/import/{second}')[0] == 200
    assert held.model is None and app.call('GET', f'/api/import/{second}')[0] == 404


def test_cached_grids_are_trimmed_to_the_budget_oldest_first(make_app, upstream):
    stub = upstream()
    app = make_app(stub.url, ifc_grid_budget=1)
    out = new_ifc(app)
    ids = level_ids(out)
    job = out['id']
    do_slice(app, job, ids['Level 1'])
    assert app.jcall('POST', slice_path(job, ids['Level 1'], 'install'), {'name': 'a'})[0] == 200
    second = do_slice(app, job, ids['Level 2'])[1]
    assert any('Level 1' in w and 'dropped' in w for w in second['warnings'])
    storeys = {s['label']: s for s in app.state(job)['storeys']}
    assert storeys['Level 1']['slice'] is None and storeys['Level 2']['slice'] == second
    assert storeys['Level 1']['last_install']['status'] == 200
    assert app.call('GET', slice_path(job, ids['Level 1'], 'preview.pgm'))[0] == 409
    assert app.call('GET', slice_path(job, ids['Level 2'], 'preview.pgm'))[0] == 200
    assert app.importer.ifc_grid_budget == 1 and srv.IFC_GRID_BUDGET == 128 * 1024 * 1024


def test_grids_within_the_budget_are_all_kept(app):
    out = new_ifc(app)
    ids = level_ids(out)
    for label in ('Level 1', 'Level 2'):
        assert do_slice(app, out['id'], ids[label])[1]['warnings'] == []
    assert [s['slice'] is not None for s in app.state(out['id'])['storeys']] == [True, True, False]


def test_slices_use_the_configured_thread_count_and_face_budget(make_app, monkeypatch):
    calls = []
    real = srv.ifcjob.ifcsource.slice_storey

    def spy(model, storey, **kwargs):
        calls.append(kwargs)
        return real(model, storey, **kwargs)

    monkeypatch.setattr(srv.ifcjob.ifcsource, 'slice_storey', spy)
    plain = make_app(None)
    out = new_ifc(plain)
    do_slice(plain, out['id'], level_ids(out)['Level 1'])
    assert calls[-1]['threads'] == 1 and 'face_budget' not in calls[-1]
    tuned = make_app(None, ifc_threads=2, ifc_face_budget=123456)
    out = new_ifc(tuned)
    do_slice(tuned, out['id'], level_ids(out)['Level 1'])
    assert calls[-1]['threads'] == 2 and calls[-1]['face_budget'] == 123456
    assert calls[-1]['slice_height'] == pytest.approx(0.241)


def test_environment_knobs_and_their_fallbacks(capsys):
    imp = srv.importer_from_env({'IMPORTER_MAX_IFC_BYTES': '1000', 'IMPORTER_IFC_THREADS': '2',
                                 'IMPORTER_IFC_FACE_BUDGET': '77'})
    assert (imp.ifc_max_bytes, imp.ifc_threads, imp.ifc_face_budget) == (1000, 2, 77)
    imp = srv.importer_from_env({'IMPORTER_MAX_IFC_BYTES': 'lots', 'IMPORTER_IFC_THREADS': '0'})
    assert (imp.ifc_max_bytes, imp.ifc_threads, imp.ifc_face_budget) == (
        srv.MAX_IFC_BYTES, srv.IFC_THREADS, None)
    err = capsys.readouterr().err
    assert 'IMPORTER_MAX_IFC_BYTES' in err and 'IMPORTER_IFC_THREADS' in err
    default = srv.importer_from_env({})
    assert (default.ifc_max_bytes, default.ifc_threads) == (srv.MAX_IFC_BYTES, 1)


def test_ifc_import_degrades_cleanly_when_the_ifc_modules_are_missing(app, monkeypatch):
    monkeypatch.setattr(srv, 'ifcjob', None)
    status, err, _ = app.upload(IFC)
    assert status == 415 and 'not available' in err['error']
    assert app.jcall('GET', '/api/health')[1]['ifc'] is False


# --- deployment files ----------------------------------------------------------------------------

def test_requirements_pin_the_ifc_stack_and_the_no_deps_extras():
    lines = {line.split('==')[0].split('>=')[0].strip(): line.strip() for line in
             (ROOT / 'docker' / 'floorplan-importer' / 'requirements.txt').read_text().splitlines()
             if line.strip() and not line.startswith('#')}
    assert lines['ifcopenshell'] == 'ifcopenshell==0.9.0.post1'
    assert lines['shapely'] == 'shapely==2.2.0'
    assert {'lark', 'typing_extensions', 'numpy', 'pymupdf'} <= set(lines)
    assert 'trimesh' not in lines


def test_ci_runs_the_ifc_tests_in_the_importer_job():
    yaml = pytest.importorskip('yaml')
    ci = yaml.safe_load((ROOT / '.github' / 'workflows' / 'ci.yml').read_text())['jobs']
    steps = ' '.join(str(s.get('run', '')) for s in ci['importer-tests']['steps'])
    for name in ('test_ifcsource.py', 'test_ifc_importer.py', 'test_floorplan_importer.py'):
        assert name in steps
    assert '--no-deps -r docker/floorplan-importer/requirements.txt' in steps


def test_the_dockerfile_documents_the_ifc_dependencies():
    text = (ROOT / 'docker' / 'floorplan-importer' / 'Dockerfile').read_text()
    assert 'lark' in text and 'typing_extensions' in text and 'ifcopenshell' in text
    assert np.__name__ == 'numpy'


# --- streaming to a temp file, memory gate -------------------------------------------------------

def test_the_ifc_is_streamed_to_a_private_temp_file_parsed_by_path_and_removed(make_app,
                                                                               monkeypatch):
    seen = {}
    real = srv.ifcjob.ifcsource.open_model

    def spy(source, **kwargs):
        seen['source'] = source
        seen['exists'] = pathlib.Path(source).is_file()
        seen['head'] = pathlib.Path(source).read_bytes()[:12]
        seen['mode'] = pathlib.Path(source).parent.stat().st_mode & 0o777
        return real(source, **kwargs)

    monkeypatch.setattr(srv.ifcjob.ifcsource, 'open_model', spy)
    app = make_app(None)
    out = new_ifc(app)
    assert isinstance(seen['source'], str) and seen['exists'] and seen['head'] == b'ISO-10303-21'
    assert seen['mode'] == 0o700
    assert not pathlib.Path(seen['source']).exists()
    assert not pathlib.Path(seen['source']).parent.exists()
    assert out['sha256'] == hashlib.sha256(IFC).hexdigest() and out['bytes'] == len(IFC)


def test_a_body_larger_than_one_chunk_is_hashed_and_sized_correctly(make_app, monkeypatch):
    monkeypatch.setattr(srv, 'UPLOAD_CHUNK', 1000)
    app = make_app(None)
    out = new_ifc(app)
    assert len(IFC) > 5000 and out['sha256'] == hashlib.sha256(IFC).hexdigest()
    assert len(out['storeys']) == 3


def test_a_failed_parse_and_a_short_body_leave_no_temp_files(make_app, monkeypatch):
    made = []
    real = srv.tempfile.TemporaryDirectory

    def spy(*args, **kwargs):
        d = real(*args, **kwargs)
        made.append(d.name)
        return d

    monkeypatch.setattr(srv.tempfile, 'TemporaryDirectory', spy)
    app = make_app(None)
    assert app.upload(IFC[:len(IFC) // 2])[0] == 422
    status, err = app.head_only(len(IFC), IFC[:srv.HEADER_SNIFF] + b'x' * 100, close_write=True)
    assert status == 400 and made
    assert not any(pathlib.Path(d).exists() for d in made)


def test_health_reports_the_resident_size(app):
    out = app.jcall('GET', '/api/health')[1]
    assert 'rss_mb' in out and (out['rss_mb'] is None or out['rss_mb'] > 10)
    assert srv.rss_mb() is None or srv.rss_mb() > 10


class Recycle:
    def __init__(self):
        self.calls = 0

    def __call__(self):
        self.calls += 1


def test_the_memory_gate_refuses_recycles_and_drops_jobs(make_app):
    recycle = Recycle()
    app = make_app(None, memory_limit_mb=1000, rss_reader=lambda: 950.0, recycle=recycle)
    pdf = app.upload(tiny_pdf())[1]['id']
    held = app.importer.jobs[pdf]
    status, err, resp = app.upload(IFC)
    assert status == 503 and resp.getheader('Retry-After') == '15'
    assert err['rss_mb'] == 950.0 and err['limit_mb'] == 1000
    assert err['needed_mb'] == pytest.approx(950 + 15 * len(IFC) / 2**20, abs=0.1)
    assert 'restarting' in err['error']
    assert recycle.calls == 1 and app.jcall('GET', '/api/health')[1]['jobs'] == 0
    assert app.call('GET', f'/api/import/{pdf}')[0] == 404 and held.pdf.doc.is_closed


def test_the_memory_gate_does_not_kill_a_busy_import(make_app):
    recycle = Recycle()
    app = make_app(None, memory_limit_mb=1000, rss_reader=lambda: 950.0, recycle=recycle)
    pdf = app.upload(tiny_pdf())[1]['id']
    with app.importer.jobs[pdf].lock:
        status, err, resp = app.upload(IFC)
    assert status == 429 and resp.getheader('Retry-After') == '15' and err['limit_mb'] == 1000
    assert recycle.calls == 0 and app.call('GET', f'/api/import/{pdf}')[0] == 200


def test_the_memory_gate_lets_fitting_uploads_through_and_ignores_pdfs(make_app):
    recycle = Recycle()
    low = make_app(None, memory_limit_mb=2048, rss_reader=lambda: 150.0, recycle=recycle)
    assert low.upload(IFC)[0] == 201 and low.upload(tiny_pdf())[0] == 201
    full = make_app(None, memory_limit_mb=1000, rss_reader=lambda: 990.0, recycle=recycle)
    assert full.upload(tiny_pdf())[0] == 201
    off = make_app(None, memory_limit_mb=None, rss_reader=lambda: 99999.0, recycle=recycle)
    assert off.upload(IFC)[0] == 201
    blind = make_app(None, memory_limit_mb=1000, rss_reader=lambda: None, recycle=recycle)
    assert blind.upload(IFC)[0] == 201
    assert recycle.calls == 0


def test_the_default_recycle_exits_the_process_after_a_short_delay(monkeypatch):
    timers = []

    class FakeTimer:
        def __init__(self, delay, fn, args=()):
            self.delay, self.fn, self.args = delay, fn, args
            self.daemon = self.started = False
            timers.append(self)

        def start(self):
            self.started = True

    monkeypatch.setattr(srv.threading, 'Timer', FakeTimer)
    srv._schedule_exit()
    assert len(timers) == 1 and timers[0].started and timers[0].daemon
    assert timers[0].fn is srv.os._exit and timers[0].args == (srv.RECYCLE_EXIT_CODE,)
    assert timers[0].delay == srv.RECYCLE_DELAY_S and srv.RECYCLE_EXIT_CODE != 0


def test_the_cgroup_limit_reader(tmp_path):
    v2, v1, missing = tmp_path / 'max', tmp_path / 'v1', tmp_path / 'nope'
    v2.write_text('max\n')
    v1.write_text(str(2 * 1024**3) + '\n')
    assert srv.cgroup_limit_mb((str(missing), str(v2), str(v1))) == 2048
    v2.write_text(str(1024**3))
    assert srv.cgroup_limit_mb((str(v2), str(v1))) == 1024
    v1.write_text(str(2**62))
    assert srv.cgroup_limit_mb((str(missing), str(v1))) is None
    assert srv.cgroup_limit_mb(()) is None


def test_memory_limit_comes_from_the_environment_first():
    imp = srv.importer_from_env({'IMPORTER_MEMORY_LIMIT_MB': '1536'})
    assert imp.memory_limit_mb == 1536
