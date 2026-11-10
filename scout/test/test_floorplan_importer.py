"""Floorplan importer HTTP service (docker/floorplan-importer/server.py).

The real server runs in-process on a loopback port 0 over synthetic PDFs built with PyMuPDF.
Upstream failures use tiny stub servers; the integration tests run the REAL fleet-status
handler (SITE_SCAFFOLD=pi, temp sites dir) and install through the importer. The A-101 sample
test reads $SCOUT_FLOORPLAN_SAMPLES (default <repo>/docs/floorplans) and skips when absent.
"""
import base64
import http.client
import importlib
import importlib.util
import json
import math
import os
import pathlib
import socket
import sys
import threading
import types
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pytest

pymupdf = pytest.importorskip('pymupdf')

_ROOT = pathlib.Path(__file__).resolve().parents[2]
_DIR = _ROOT / 'docker' / 'floorplan-importer'
_FLEET = _ROOT / 'docker' / 'fleet-status'


def _load_server():
    sys.path.insert(0, str(_DIR))
    try:
        importlib.import_module('pdfsource')
        spec = importlib.util.spec_from_file_location('fp_importer_server', _DIR / 'server.py')
        module = importlib.util.module_from_spec(spec)
        sys.modules['fp_importer_server'] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(_DIR))


srv = _load_server()
ps = sys.modules['pdfsource']
calib = sys.modules['calib']
pr = sys.modules['planraster']

W, H = 2592, 1728
M_PT = 48 * 0.0254 / 72
NOTE = 'SCALE: 1/4" = 1\'-0"'
ROOM = (600.0, 500.0, 600.0 + 10 / M_PT, 500.0 + 6 / M_PT)
GAP = (800.0, 800.0 + 0.9 / M_PT)
ROOM_Y_UP = (H - ROOM[3], H - ROOM[1])
FOOTPRINT_M = (10.1, 6.1)
JSON = {'Content-Type': 'application/json'}


# --- synthetic sheets -------------------------------------------------------------------------

def sheet_bytes(note=NOTE, walls=True, door=True):
    doc = pymupdf.open()
    page = doc.new_page(width=W, height=H)
    page.draw_rect(pymupdf.Rect(36, 36, W - 36, H - 36), color=(0, 0, 0), width=2.2)
    page.draw_line((2300, 40), (2300, H - 40), color=(0, 0, 0), width=0.6)
    page.draw_line((ROOM[0], 440), (ROOM[2], 440), color=(0, 0, 0), width=0.4)
    page.insert_text((900, 700), 'ROOM 101', fontsize=20)
    if note:
        page.insert_text((2320, 1650), note, fontsize=12)
    if walls:
        x0, y0, x1, y1 = ROOM
        pts = [(x0, y1), (x1, y1), (x1, y0), (x0, y0), (x0, y1)]
        if door:
            pts = [(GAP[1], y1), (x1, y1), (x1, y0), (x0, y0), (x0, y1), (GAP[0], y1)]
        page.draw_polyline(pts, color=(0, 0, 0), width=6.0, closePath=False)
    return doc.tobytes()


def image_only_bytes(dpi=100):
    src = pymupdf.open(stream=sheet_bytes(), filetype='pdf')
    pix = src[0].get_pixmap(dpi=dpi, colorspace=pymupdf.csGRAY, alpha=False)
    out = pymupdf.open()
    page = out.new_page(width=W, height=H)
    page.insert_image(page.rect, pixmap=pix)
    return out.tobytes()


def pgm_grid(raw):
    head, _, rest = raw.partition(b'\n255\n')
    _, dims = head.split(b'\n', 1)
    w, h = (int(v) for v in dims.split())
    return np.frombuffer(rest, dtype=np.uint8).reshape(h, w)


def occupied_extent(raw, res=0.05):
    grid = pgm_grid(raw)
    rows, cols = np.nonzero(grid == pr.OCCUPIED)
    return (cols.max() - cols.min() + 1) * res, (rows.max() - rows.min() + 1) * res


# --- HTTP plumbing -------------------------------------------------------------------------------

class App:
    def __init__(self, importer, server, thread):
        self.importer, self.server, self.thread = importer, server, thread
        self.port = server.server_address[1]

    def call(self, method, path, body=None, headers=None, timeout=20):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=timeout)
        try:
            if isinstance(body, (dict, list)):
                body = json.dumps(body)
                headers = {**JSON, **(headers or {})}
            conn.request(method, path, body=body, headers=headers or {})
            resp = conn.getresponse()
            return resp.status, resp.read(), resp
        finally:
            conn.close()

    def jcall(self, method, path, body=None, headers=None):
        status, raw, resp = self.call(method, path, body, headers)
        return status, json.loads(raw), resp

    def upload(self, data, filename=None):
        path = '/api/import' + (f'?filename={urllib.parse.quote(filename)}' if filename else '')
        return self.jcall('POST', path, data, {'Content-Type': 'application/octet-stream'})

    def new_job(self, data=None):
        status, out, _ = self.upload(sheet_bytes() if data is None else data)
        assert status == 201, out
        return out['id']

    def calibrate(self, job, **body):
        return self.jcall('POST', f'/api/import/{job}/calibrate', body)

    def job(self, job):
        return self.jcall('GET', f'/api/import/{job}')[1]

    def preview(self, job):
        status, raw, _ = self.call('GET', f'/api/import/{job}/preview.pgm')
        assert status == 200
        return raw


@pytest.fixture
def make_app():
    started = []

    def make(fleet_url=None, cors_origin='*', **kwargs):
        importer = srv.Importer(fleet_url, **kwargs)
        server = srv.ImporterServer(('127.0.0.1', 0), importer, cors_origin)
        thread = threading.Thread(target=server.serve_forever,
                                  kwargs={'poll_interval': 0.01}, daemon=True)
        thread.start()
        started.append((server, thread, importer))
        return App(importer, server, thread)

    yield make
    for server, thread, importer in started:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
        for job in list(importer.jobs.values()):
            ps.close_pdf(job.pdf)


@pytest.fixture
def app(make_app):
    return make_app('http://127.0.0.1:9')


class Upstream:
    """A stub fleet_status: records POSTs, answers a canned status/body after `delay`."""

    def __init__(self, status=200, body=b'{"ok": true, "replaced": false}', delay=0.0,
                 content_type='application/json', headers=None):
        self.status, self.body, self.delay = status, body, delay
        self.content_type, self.extra, self.requests = content_type, headers or {}, []
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                raw = self.rfile.read(int(self.headers.get('Content-Length', 0)))
                outer.requests.append({'path': self.path, 'headers': dict(self.headers),
                                       'raw': raw})
                if outer.delay:
                    threading.Event().wait(outer.delay)
                try:
                    self.send_response(outer.status)
                    self.send_header('Content-Type', outer.content_type)
                    self.send_header('Content-Length', str(len(outer.body)))
                    for k, v in outer.extra.items():
                        self.send_header(k, v)
                    self.end_headers()
                    self.wfile.write(outer.body)
                except OSError:
                    pass

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), H)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs={'poll_interval': 0.01}, daemon=True)
        self.thread.start()
        self.url = f'http://127.0.0.1:{self.server.server_address[1]}'

    def payload(self, i=-1):
        return json.loads(self.requests[i]['raw'])

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)


@pytest.fixture
def upstream():
    made = []

    def make(**kwargs):
        stub = Upstream(**kwargs)
        made.append(stub)
        return stub

    yield make
    for stub in made:
        stub.close()


def closed_port_url():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return f'http://127.0.0.1:{port}'


# --- health, CORS ------------------------------------------------------------------------------

def test_health_reports_configuration_and_load(app):
    status, out, resp = app.jcall('GET', '/api/health')
    assert status == 200 and out == {
        'ok': True, 'version': 'dev', 'fleet_status_url': 'http://127.0.0.1:9',
        'jobs': 0, 'busy': False}
    app.new_job()
    assert app.jcall('GET', '/api/health')[1]['jobs'] == 1
    with app.importer.heavy:
        assert app.jcall('GET', '/api/health')[1]['busy'] is True


def test_every_response_carries_cors_even_errors(app):
    for method, path in (('GET', '/api/health'), ('GET', '/api/nope'),
                         ('GET', '/api/import/' + 'a' * 22)):
        status, _, resp = app.call(method, path)
        assert resp.getheader('Access-Control-Allow-Origin') == '*', (path, status)
    assert resp.getheader('Cache-Control') == 'no-store'


def test_preflight_for_a_raw_post_with_content_type(app):
    status, raw, resp = app.call('OPTIONS', '/api/import', headers={
        'Origin': 'http://ui.example', 'Access-Control-Request-Method': 'POST',
        'Access-Control-Request-Headers': 'content-type'})
    assert status == 204 and raw == b''
    assert resp.getheader('Access-Control-Allow-Origin') == '*'
    assert {'GET', 'POST', 'DELETE', 'OPTIONS'} <= set(
        resp.getheader('Access-Control-Allow-Methods').replace(' ', '').split(','))
    assert 'Content-Type' in resp.getheader('Access-Control-Allow-Headers')


def test_cors_origin_comes_from_configuration(make_app):
    app = make_app(None, cors_origin='https://ui.example')
    _, _, resp = app.call('GET', '/api/health')
    assert resp.getheader('Access-Control-Allow-Origin') == 'https://ui.example'
    assert resp.getheader('Vary') == 'Origin'
    _, _, resp = app.call('OPTIONS', '/api/import')
    assert resp.getheader('Access-Control-Allow-Origin') == 'https://ui.example'


# --- upload --------------------------------------------------------------------------------------

def test_upload_lists_pages_with_a_scale_proposal(app):
    data = sheet_bytes()
    status, out, _ = app.upload(data, 'plan A-101.pdf')
    assert status == 201 and out['kind'] == 'pdf' and out['filename'] == 'plan A-101.pdf'
    import hashlib
    assert out['sha256'] == hashlib.sha256(data).hexdigest() and out['bytes'] == len(data)
    page = out['pages'][0]
    assert (page['index'], page['width'], page['height'], page['has_vectors']) == (0, W, H, True)
    prop = page['scale']['proposal']
    assert prop['m_per_unit'] == pytest.approx(M_PT, rel=1e-12) and prop['confidence'] == 'high'
    assert prop['m_per_point'] == prop['m_per_unit']
    evidence = ' | '.join(prop['evidence'])
    assert 'ARCH D' in evidence and '1/4"' in evidence
    assert page['scale']['calibration_required'] is False and page['scale']['raster'] is False
    state = app.job(out['id'])
    assert state['pages'] == out['pages'] and state['preview'] is None
    assert state['calibration'] is None and state['last_install'] is None
    assert 0 < state['expires_in'] <= srv.JOB_TTL_S


@pytest.mark.parametrize('given, expected', [
    ('../../etc/passwd', 'passwd'), ('C:\\x\\plan.pdf', 'plan.pdf'), ('a\x00b\n.pdf', 'a_b_.pdf'),
    ('', 'upload.pdf'), ('..', 'upload.pdf'), ('....', 'upload.pdf'), (None, 'upload.pdf'),
    ('x' * 500 + '.pdf', 'x' * srv.MAX_FILENAME), ('pl\u00e4n.pdf', 'pl_n.pdf'),
])
def test_filename_is_reduced_to_a_display_name(given, expected):
    assert srv.safe_filename(given) == expected


def test_filename_query_is_sanitised_end_to_end(app):
    status, out, _ = app.upload(sheet_bytes(), '../../etc/passwd')
    assert status == 201 and out['filename'] == 'passwd'


def test_non_pdf_and_empty_uploads_are_415(app):
    for blob in (b'hello world', b'', b'<html></html>', b'\x00' * 50):
        status, out, _ = app.upload(blob)
        assert status == 415 and 'error' in out, blob


def test_ifc_header_is_415_with_a_clear_message(app):
    ifc = b"ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('ViewDefinition'),'2;1');\nENDSEC;\n"
    status, out, _ = app.upload(ifc)
    assert status == 415 and out == {'error': 'IFC import is not available yet'}
    assert app.jcall('GET', '/api/health')[1]['jobs'] == 0


def test_pdf_failures_map_to_their_status(app):
    locked = pymupdf.open()
    locked.new_page()
    encrypted = locked.tobytes(encryption=pymupdf.PDF_ENCRYPT_AES_256, owner_pw='o', user_pw='u')
    cases = [(b'%PDF-1.7\ngarbage garbage', 422, 'corrupt_pdf'), (encrypted, 422, 'encrypted_pdf')]
    for blob, want, code in cases:
        status, out, _ = app.upload(blob)
        assert status == want and out['code'] == code, out


def test_oversize_declared_length_is_413_without_reading_the_body(app, monkeypatch):
    monkeypatch.setattr(ps, 'MAX_PDF_BYTES', 1000)
    conn = http.client.HTTPConnection('127.0.0.1', app.port, timeout=5)
    conn.putrequest('POST', '/api/import')
    conn.putheader('Content-Length', '5000')
    conn.endheaders()
    resp = conn.getresponse()
    assert resp.status == 413 and 'max 1000' in json.loads(resp.read())['error']
    conn.close()


def test_missing_and_malformed_content_length(app):
    conn = http.client.HTTPConnection('127.0.0.1', app.port, timeout=5)
    conn.putrequest('POST', '/api/import')
    conn.endheaders()
    assert conn.getresponse().status == 411
    conn.close()
    conn = http.client.HTTPConnection('127.0.0.1', app.port, timeout=5)
    conn.putrequest('POST', '/api/import')
    conn.putheader('Content-Length', 'abc')
    conn.endheaders()
    assert conn.getresponse().status == 400
    conn.close()


def test_short_body_is_400(app):
    conn = http.client.HTTPConnection('127.0.0.1', app.port, timeout=5)
    conn.putrequest('POST', '/api/import')
    conn.putheader('Content-Length', '500')
    conn.endheaders()
    conn.send(b'%PDF-1.4 short')
    conn.sock.shutdown(socket.SHUT_WR)
    resp = conn.getresponse()
    assert resp.status == 400 and 'shorter' in json.loads(resp.read())['error']
    conn.close()


# --- routing -------------------------------------------------------------------------------------

def test_unknown_paths_methods_and_jobs(app):
    assert app.jcall('GET', '/api/nope')[0] == 404
    assert app.jcall('GET', '/api/import/short')[0] == 404
    assert app.jcall('GET', '/api/import/' + 'A' * 22)[0] == 404
    assert app.jcall('GET', '/api/import')[0] == 405
    assert app.jcall('POST', '/api/health')[0] == 405
    job = app.new_job()
    for path in (f'/api/import/{job}/calibrate', f'/api/import/{job}/install'):
        assert app.jcall('GET', path)[0] == 405


def test_unsupported_methods_and_garbage_requests_answer_json(app):
    for method in ('PUT', 'PATCH', 'HEAD'):
        status, raw, resp = app.call(method, '/api/import')
        assert status in (501, 405) and resp.getheader('Content-Type') == 'application/json'
        assert method == 'HEAD' or 'error' in json.loads(raw)
    with socket.create_connection(('127.0.0.1', app.port), timeout=5) as sock:
        sock.sendall(b'NOT A REQUEST\r\n\r\n')
        reply = b''
        while chunk := sock.recv(4096):
            reply += chunk
    assert 'error' in json.loads(reply[reply.index(b'{'):])


def test_every_error_is_json_never_a_traceback(app, monkeypatch):
    def boom(self, *args, **kwargs):
        raise RuntimeError('secret internal detail')

    monkeypatch.setattr(srv.Importer, 'job_json', boom)
    job = app.new_job()
    status, raw, _ = app.call('GET', f'/api/import/{job}')
    assert status == 500 and json.loads(raw) == {'error': 'internal error'}
    assert b'secret' not in raw and b'Traceback' not in raw


# --- pages and PNG -------------------------------------------------------------------------------

def test_page_json_and_png_agree_on_the_mapping(app):
    job = app.new_job()
    status, out, _ = app.jcall('GET', f'/api/import/{job}/page/0')
    assert status == 200 and out['index'] == 0 and out['page']['width'] == W
    png = out['png']
    assert png['url'] == f'/api/import/{job}/page/0.png'
    assert (png['width'], png['height']) == (W, H)
    assert png['width_px'] == pytest.approx(W * png['scale_x'], abs=1)
    status, raw, resp = app.call('GET', png['url'])
    assert status == 200 and raw.startswith(b'\x89PNG')
    assert resp.getheader('Content-Type') == 'image/png'
    assert float(resp.getheader('X-Scale-X')) == png['scale_x']
    assert float(resp.getheader('X-Scale-Y')) == png['scale_y']
    assert float(resp.getheader('X-Page-Width')) == W
    assert float(resp.getheader('X-Page-Height')) == H
    pix = pymupdf.Pixmap(raw)
    assert (pix.width, pix.height) == (png['width_px'], png['height_px'])


def test_px_per_pt_changes_the_scale_and_is_validated(app):
    job = app.new_job()
    big = app.jcall('GET', f'/api/import/{job}/page/0?px_per_pt=2')[1]['png']
    small = app.jcall('GET', f'/api/import/{job}/page/0?px_per_pt=0.5')[1]['png']
    assert big['scale_x'] == pytest.approx(2.0, abs=0.01)
    assert small['scale_x'] == pytest.approx(0.5, abs=0.01)
    for bad in ('abc', '0', '-1', '99', 'nan'):
        status, out, _ = app.jcall('GET', f'/api/import/{job}/page/0?px_per_pt={bad}')
        assert status == 400, bad
        assert app.call('GET', f'/api/import/{job}/page/0.png?px_per_pt={bad}')[0] == 400


def test_page_out_of_range_is_404(app):
    job = app.new_job()
    for path in (f'/api/import/{job}/page/1', f'/api/import/{job}/page/1.png',
                 f'/api/import/{job}/page/99'):
        assert app.call('GET', path)[0] == 404, path


def test_multi_page_pdf_computes_scale_lazily_per_page(app, monkeypatch):
    monkeypatch.setattr(srv, 'EAGER_SCALE_PAGES', 1)
    doc = pymupdf.open(stream=sheet_bytes(), filetype='pdf')
    doc.insert_pdf(pymupdf.open(stream=sheet_bytes(note=None), filetype='pdf'))
    status, out, _ = app.upload(doc.tobytes())
    assert status == 201 and [p['scale'] for p in out['pages']] == [None, None]
    job = out['id']
    page1 = app.jcall('GET', f'/api/import/{job}/page/1')[1]
    assert page1['scale']['calibration_required'] is True
    assert app.job(job)['pages'][1]['scale']['proposal']['m_per_unit'] is None
    assert app.job(job)['pages'][0]['scale'] is None


# --- calibrate: methods --------------------------------------------------------------------------

def test_proposal_calibration_builds_a_preview_matching_the_footprint(app):
    job = app.new_job()
    status, out, _ = app.calibrate(job, method='proposal')
    assert status == 200 and out['method'] == 'proposal' and out['page'] == 0
    assert out['m_per_unit'] == pytest.approx(M_PT, rel=1e-12)
    assert out['extraction']['kind'] == 'vector'
    assert out['extraction']['diagnostics']['mode'] == 'strokes'
    assert out['rotation_deg'] == 0 and out['resolution'] == 0.05 and out['close_m'] == pr.CLOSE_M
    assert out['origin'] == pytest.approx([ROOM[0], ROOM_Y_UP[0]], abs=0.5)
    prev = out['preview']
    ext = prev['occupied_extent_m']
    assert ext['width'] == pytest.approx(FOOTPRINT_M[0], rel=0.05)
    assert ext['height'] == pytest.approx(FOOTPRINT_M[1], rel=0.05)
    assert prev['cells']['free'] > 20000 and prev['cells']['occupied'] > 1000
    assert out['warnings'] == []
    pgm = app.preview(job)
    w, h = occupied_extent(pgm)
    assert w == pytest.approx(FOOTPRINT_M[0], rel=0.05)
    assert h == pytest.approx(FOOTPRINT_M[1], rel=0.05)
    assert pgm_grid(pgm).shape == (prev['height'], prev['width'])
    state = app.job(job)
    assert state['selected_page'] == 0 and state['preview'] == prev
    assert state['calibration']['method'] == 'proposal'


def test_door_gap_is_sealed_in_the_preview(app):
    job = app.new_job()
    out = app.calibrate(job, method='proposal')[1]
    grid = pgm_grid(app.preview(job))
    res, (ox, oy) = 0.05, out['preview']['origin'][:2]
    gap_x = (GAP[0] - ROOM[0] + (GAP[1] - GAP[0]) / 2) * M_PT

    def at(x, y):
        return int(grid[grid.shape[0] - 1 - int(math.floor((y - oy) / res)),
                        int(math.floor((x - ox) / res))])

    assert at(5.0, 3.0) == pr.FREE and at(gap_x, 0.0) == pr.FREE and at(gap_x, -0.6) == pr.UNKNOWN


def test_preview_png_colours_the_three_classes(app):
    job = app.new_job()
    app.calibrate(job, method='proposal')
    status, raw, resp = app.call('GET', f'/api/import/{job}/preview.png')
    assert status == 200 and raw.startswith(b'\x89PNG')
    assert resp.getheader('Content-Type') == 'image/png'
    pix = pymupdf.Pixmap(raw)
    img = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width)
    assert set(np.unique(img)) == {20, 170, 255}
    grid = pgm_grid(app.preview(job))
    assert ((img == 20) == (grid == pr.OCCUPIED)).all()
    assert ((img == 255) == (grid == pr.FREE)).all()


def test_two_point_calibration(app):
    job = app.new_job()
    status, out, _ = app.calibrate(job, method='two_point', p1=[600, 900], p2=[1100, 900],
                                   metres=500 * M_PT)
    assert status == 200 and out['m_per_unit'] == pytest.approx(M_PT, abs=1e-9)
    assert out['detail']['metres'] == pytest.approx(500 * M_PT)
    assert out['preview']['occupied_extent_m']['width'] == pytest.approx(10.1, rel=0.05)
    nested = app.calibrate(job, method='two_point',
                           params={'p1': [0, 0], 'p2': [0, 100], 'metres': 1.69333})[1]
    assert nested['m_per_unit'] == pytest.approx(0.0169333)


@pytest.mark.parametrize('kwargs, ratio', [
    ({'scale': '1/4" = 1\'-0"'}, 48.0), ({'scale': '1:100'}, 100.0),
    ({'scale': 'SCALE: 1/8"=1\''}, 96.0), ({'ratio': 48}, 48.0), ({'ratio': 250.5}, 250.5)])
def test_typed_calibration_uses_the_pages_userunit(app, kwargs, ratio):
    job = app.new_job()
    status, out, _ = app.calibrate(job, method='typed', **kwargs)
    assert status == 200
    assert out['m_per_unit'] == pytest.approx(ratio * 0.0254 / 72, rel=1e-9)
    assert out['detail']['ratio'] == pytest.approx(ratio)


def test_typed_scale_on_a_userunit_page(app):
    doc = pymupdf.open(stream=sheet_bytes(), filetype='pdf')
    doc.xref_set_key(doc[0].xref, 'UserUnit', '2')
    job = app.new_job(doc.tobytes())
    out = app.calibrate(job, method='typed', ratio=48)[1]
    assert out['m_per_unit'] == pytest.approx(2 * M_PT, rel=1e-9)


def test_typed_paper_size_rescales_a_reduced_print(app):
    job = app.new_job()
    out = app.calibrate(job, method='typed', scale='1/4" = 1\'-0"', paper_in=[72, 48])[1]
    assert out['m_per_unit'] == pytest.approx(2 * M_PT, rel=1e-9)
    assert any('paper_in' in w for w in out['warnings'])
    same = app.calibrate(job, method='typed', ratio=48, paper_in=[48, 72])[1]
    assert same['m_per_unit'] == pytest.approx(2 * M_PT, rel=1e-9)
    status, out, _ = app.calibrate(job, method='typed', ratio=48, paper_in=[30, 30])
    assert status == 400 and 'proportions' in out['error']


def span(p1, p2, metres):
    return {'p1': p1, 'p2': p2, 'metres': metres}


def test_robot_calibration_two_orthogonal_spans(app):
    job = app.new_job()
    spans = [span([600, 900], [1190.55, 900], 10.0 * 1.004),
             span([600, 900], [600, 1254.33], 6.0 * 0.998)]
    status, out, _ = app.calibrate(job, method='robot', spans=spans)
    assert status == 200 and out['warnings'] == []
    assert out['m_per_unit'] == pytest.approx(M_PT, rel=0.005)
    d = out['detail']
    assert d['consistent'] is True and d['spread'] < 0.01 and len(d['per_span']) == 2
    assert d['mean'] == pytest.approx(out['m_per_unit'])


def test_robot_three_percent_mismatch_warns_but_applies_the_mean(app):
    job = app.new_job()
    spans = [span([600, 900], [1190.55, 900], 10.0), span([600, 900], [600, 1254.33], 6.18)]
    status, out, _ = app.calibrate(job, method='robot', spans=spans)
    assert status == 200 and out['detail']['consistent'] is False
    assert out['detail']['spread'] == pytest.approx(0.03, abs=0.002)
    assert any('disagree' in w for w in out['warnings'])
    assert out['m_per_unit'] == pytest.approx(out['detail']['mean'])
    assert app.preview(job)


@pytest.mark.parametrize('spans, text', [
    ([span([0, 0], [100, 0], 2.0)], '2'),
    ([], '2'),
    ([span([0, 0], [100, 0], 2.0), span([0, 50], [200, 50], 4.0)], 'non-parallel'),
    ([span([0, 0], [100, 0], 2.0), span([0, 0], [-100, 5], 2.0)], 'non-parallel'),
    ([span([0, 0], [100, 0], 2.0), span([0, 0], [96.6, 25.9], 2.0)], 'non-parallel'),
    ([span([0, 0], [100, 0], 2.0), span([5, 5], [5, 5], 2.0)], 'coincide'),
    ([span([0, 0], [100, 0], 2.0), span([0, 0], [0, 100], -1.0)], 'metres'),
    ([span([0, 0], [100, 0], 2.0), {'p1': [0, 0], 'p2': [0, 1]}], 'metres'),
    ([span([0, 0], [100, 0], 2.0), {'p1': [0, 0], 'p2': [0, 1], 'metres': 1, 'x': 1}], 'unknown'),
    ('abc', '2'),
])
def test_robot_refusals(app, spans, text):
    job = app.new_job()
    status, out, _ = app.calibrate(job, method='robot', spans=spans)
    assert status == 400 and text in out['error']


def test_robot_accepts_spans_just_beyond_the_parallel_limit(app):
    job = app.new_job()
    spans = [span([0, 0], [1000, 0], 16.9), span([0, 0], [906.3, 422.6], 16.9)]
    status, out, _ = app.calibrate(job, method='robot', spans=spans)
    assert status == 200 and out['m_per_unit'] == pytest.approx(0.0169, rel=1e-3)


def test_three_spans_need_only_one_non_parallel_pair(app):
    job = app.new_job()
    spans = [span([0, 0], [1000, 0], 16.9), span([0, 5], [800, 5], 13.5),
             span([0, 0], [0, 500], 8.45)]
    assert app.calibrate(job, method='robot', spans=spans)[0] == 200


# --- calibrate: map frame, options, validation ---------------------------------------------------

def test_rotation_turns_the_map_and_origin_is_echoed(app):
    job = app.new_job()
    flat = app.calibrate(job, method='proposal')[1]['preview']['occupied_extent_m']
    rotated = app.calibrate(job, method='proposal', rotation_deg=90)[1]
    turned = rotated['preview']['occupied_extent_m']
    assert turned['width'] == pytest.approx(flat['height'], abs=0.15)
    assert turned['height'] == pytest.approx(flat['width'], abs=0.15)
    out = app.calibrate(job, method='proposal', origin=[0, 0], resolution=0.1, close_m=2.0)[1]
    assert out['origin'] == [0.0, 0.0] and out['resolution'] == 0.1 and out['close_m'] == 2.0
    assert out['preview']['resolution'] == 0.1
    assert app.job(job)['calibration']['origin'] == [0.0, 0.0]


def test_the_default_origin_is_the_geometry_lower_left(app):
    job = app.new_job()
    out = app.calibrate(job, method='proposal')[1]
    prev_origin = out['preview']['origin']
    assert out['origin'] == pytest.approx([ROOM[0], ROOM_Y_UP[0]], abs=0.5)
    assert prev_origin[2] == 0.0 and prev_origin[0] < 0 and prev_origin[1] < 0


def test_close_m_is_passed_through_to_the_rasterizer(app):
    job = app.new_job(sheet_bytes(door=False))
    base = app.calibrate(job, method='proposal')[1]
    assert base['preview']['cells']['free'] > 20000
    wide = pymupdf.open(stream=sheet_bytes(), filetype='pdf')
    x0, y0, x1, y1 = ROOM
    wide_page = wide.new_page(width=W, height=H)
    wide_page.insert_text((2320, 1650), NOTE, fontsize=12)
    wide_page.draw_polyline([(x0 + 2.0 / M_PT, y1), (x1, y1), (x1, y0), (x0, y0), (x0, y1),
                             (x0, y1)], width=6.0, closePath=False)
    open_room = app.new_job(wide.tobytes())
    leaky = app.calibrate(open_room, method='proposal', page=1)[1]
    assert leaky['preview']['interior_fraction'] < 0.1
    assert any('close_m' in w for w in leaky['warnings'])
    sealed = app.calibrate(open_room, method='proposal', page=1, close_m=2.6)[1]
    assert sealed['preview']['interior_fraction'] > 0.8 and sealed['warnings'] == []
    assert base['preview']['interior_fraction'] > 0.8 and base['warnings'] == []


def test_wall_options_reach_the_extractor(app):
    job = app.new_job()
    out = app.calibrate(job, method='proposal', wall_options={'min_stroke_width': 0.1,
                                                              'wall_layers': ['wall']})[1]
    assert out['extraction']['diagnostics']['width_threshold'] == 0.1
    assert app.calibrate(job, method='proposal',
                         wall_options={'mode': 'fills'})[0] == 422


@pytest.mark.parametrize('body', [
    {}, {'method': 'bogus'}, {'method': 'proposal', 'surprise': 1},
    {'method': 'proposal', 'p1': [0, 0]}, {'method': 'proposal', 'url': 'http://evil'},
    {'method': 'two_point'}, {'method': 'two_point', 'p1': [0, 0], 'p2': [1, 1]},
    {'method': 'two_point', 'p1': [0], 'p2': [1, 1], 'metres': 1},
    {'method': 'two_point', 'p1': [0, 0], 'p2': [0, 0], 'metres': 1},
    {'method': 'two_point', 'p1': [0, 0], 'p2': [1, 1], 'metres': 0},
    {'method': 'two_point', 'p1': [0, 0], 'p2': [1, 1], 'metres': 'x'},
    {'method': 'two_point', 'p1': [0, 0], 'p2': [1, 1], 'metres': 1, 'params': {'bogus': 1}},
    {'method': 'two_point', 'p1': [0, 0], 'p2': [1, 1], 'metres': True},
    {'method': 'typed'}, {'method': 'typed', 'scale': '1:100', 'ratio': 100},
    {'method': 'typed', 'scale': 'nonsense'}, {'method': 'typed', 'scale': '1:100 and 1:50'},
    {'method': 'typed', 'ratio': -5}, {'method': 'typed', 'ratio': 0},
    {'method': 'typed', 'ratio': 48, 'paper_in': [1]}, {'method': 'typed', 'scale': 5},
    {'method': 'proposal', 'resolution': 0}, {'method': 'proposal', 'resolution': 5},
    {'method': 'proposal', 'resolution': 'x'}, {'method': 'proposal', 'close_m': 0},
    {'method': 'proposal', 'close_m': 99}, {'method': 'proposal', 'rotation_deg': 'x'},
    {'method': 'proposal', 'rotation_deg': 99999}, {'method': 'proposal', 'origin': [1]},
    {'method': 'proposal', 'origin': 'x'}, {'method': 'proposal', 'force_raster': 'yes'},
    {'method': 'proposal', 'page': 'a'}, {'method': 'proposal', 'page': True},
    {'method': 'proposal', 'wall_options': 'x'}, {'method': 'proposal', 'wall_options': {'x': 1}},
    {'method': 'proposal', 'wall_options': {'mode': 'bogus'}},
    {'method': 'proposal', 'wall_options': {'curve_segments': 0}},
    {'method': 'proposal', 'wall_options': {'region': [0, 0, 0, 5]}},
    {'method': 'proposal', 'raster_options': 3},
    {'method': 'proposal', 'raster_options': {'x': 1}},
])
def test_bad_calibration_bodies_are_400(app, body):
    job = app.new_job()
    status, out, _ = app.calibrate(job, **body)
    assert status == 400 and out['error'], (body, out)
    assert app.job(job)['preview'] is None


def test_bad_raster_options_are_400_on_a_raster_page(app):
    job = app.new_job(image_only_bytes())
    status, out, _ = app.calibrate(job, method='two_point', p1=[0, 0], p2=[0, 100], metres=1.7,
                                   raster_options={'dark_threshold': 0})
    assert status == 400


def test_page_out_of_range_in_calibrate_is_404(app):
    job = app.new_job()
    assert app.calibrate(job, method='proposal', page=1)[0] == 404
    assert app.calibrate(job, method='proposal', page=-1)[0] == 404


def test_multi_page_calibrate_needs_a_page(app):
    doc = pymupdf.open(stream=sheet_bytes(), filetype='pdf')
    doc.insert_pdf(pymupdf.open(stream=sheet_bytes(), filetype='pdf'))
    job = app.new_job(doc.tobytes())
    status, out, _ = app.calibrate(job, method='proposal')
    assert status == 400 and 'page' in out['error']
    assert app.calibrate(job, method='proposal', page=1)[0] == 200
    assert app.job(job)['selected_page'] == 1


def test_proposal_method_needs_a_proposal(app):
    job = app.new_job(sheet_bytes(note=None))
    status, out, _ = app.calibrate(job, method='proposal')
    assert status == 400 and 'no scale proposal' in out['error']
    assert app.calibrate(job, method='two_point', p1=[0, 0], p2=[100, 0], metres=1.69)[0] == 200


def test_a_medium_confidence_proposal_warns(app):
    job = app.new_job(sheet_bytes(note=NOTE + ' SCALE: 1/8" = 1\'-0"'))
    out = app.calibrate(job, method='proposal')[1]
    assert any('confidence is low' in w for w in out['warnings'])


def test_vector_page_without_walls_is_422_with_a_retry_hint(app):
    job = app.new_job(sheet_bytes(walls=False))
    status, out, _ = app.calibrate(job, method='proposal')
    assert status == 422 and 'force_raster' in out['error'] and 'wall_options' in out['error']
    forced = app.calibrate(job, method='proposal', force_raster=True,
                           raster_options={'region': [2300, 40, 2560, 120]})
    assert forced[0] == 200 and forced[1]['extraction']['kind'] == 'raster'
    assert any('raster extraction' in w for w in forced[1]['warnings'])


def test_calibrate_json_body_limits(app):
    job = app.new_job()
    path = f'/api/import/{job}/calibrate'
    conn = http.client.HTTPConnection('127.0.0.1', app.port, timeout=5)
    conn.putrequest('POST', path)
    conn.putheader('Content-Length', str(srv.MAX_JSON_BYTES + 1))
    conn.endheaders()
    assert conn.getresponse().status == 413
    conn.close()
    conn = http.client.HTTPConnection('127.0.0.1', app.port, timeout=5)
    conn.putrequest('POST', path)
    conn.endheaders()
    assert conn.getresponse().status == 411
    conn.close()
    assert app.call('POST', path, b'{not json', JSON)[0] == 400
    assert app.call('POST', path, b'[1, 2]', JSON)[0] == 400
    assert app.call('POST', path, b'"x"', JSON)[0] == 400


# --- raster-only PDF (AE2) -----------------------------------------------------------------------

def test_raster_pdf_has_no_proposal_and_needs_calibration_before_a_preview(app):
    status, out, _ = app.upload(image_only_bytes())
    assert status == 201
    job = out['id']
    scale = out['pages'][0]['scale']
    assert out['pages'][0]['has_vectors'] is False and scale['raster'] is True
    assert scale['calibration_required'] is True and scale['proposal']['m_per_unit'] is None
    assert scale['proposal']['confidence'] == 'none'
    status, err, _ = app.calibrate(job, method='proposal')
    assert status == 400
    assert app.call('GET', f'/api/import/{job}/preview.pgm')[0] == 409
    assert app.call('GET', f'/api/import/{job}/preview.png')[0] == 409
    status, out, _ = app.calibrate(
        job, method='two_point', p1=[600, 900], p2=[1100, 900], metres=500 * M_PT,
        raster_options={'region': [500, 800, 1300, 1260], 'dark_threshold': 100,
                        'min_thickness_m': 0.06})
    assert status == 200 and out['extraction']['kind'] == 'raster'
    assert any('raster extraction' in w for w in out['warnings'])
    w, h = occupied_extent(app.preview(job))
    assert w == pytest.approx(FOOTPRINT_M[0], rel=0.1)
    assert h == pytest.approx(FOOTPRINT_M[1], rel=0.1)
    assert out['preview']['cells']['free'] > 20000


# --- recalibrate (AE4) ---------------------------------------------------------------------------

def test_recalibrating_regenerates_the_preview_and_install_sends_the_new_bytes(make_app, upstream):
    stub = upstream()
    app = make_app(stub.url)
    job = app.new_job()
    first = app.calibrate(job, method='proposal')[1]['preview']
    pgm1 = app.preview(job)
    status, out, _ = app.calibrate(job, method='two_point', p1=[0, 0], p2=[500, 0],
                                   metres=2 * 500 * M_PT)
    assert status == 200
    second = out['preview']
    pgm2 = app.preview(job)
    assert (second['width'], second['height']) != (first['width'], first['height'])
    assert second['occupied_extent_m']['width'] == pytest.approx(
        2 * first['occupied_extent_m']['width'], rel=0.05)
    assert pgm1 != pgm2 and app.job(job)['preview'] == second
    assert app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})[0] == 200
    sent = base64.b64decode(stub.payload()['pgm_b64'])
    assert sent == pgm2 and sent != pgm1


# --- install -------------------------------------------------------------------------------------

def test_install_payload_is_the_mapinstall_wire_format(make_app, upstream):
    stub = upstream()
    app = make_app(stub.url + '/')
    job = app.new_job()
    app.calibrate(job, method='proposal', rotation_deg=15, origin=[600, 900])
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})
    assert status == 200 and out == {'ok': True, 'replaced': False}
    req = stub.requests[0]
    assert req['path'] == '/api/maps/install'
    assert req['headers']['Content-Type'] == 'application/json'
    body = stub.payload()
    assert set(body) == {'name', 'yaml', 'pgm_b64', 'sidecar', 'source'}
    assert body['name'] == 'plan' and body['source'] == 'drawing'
    assert '\n' not in body['pgm_b64'] and body['pgm_b64'] == base64.b64encode(
        base64.b64decode(body['pgm_b64'])).decode()
    assert base64.b64decode(body['pgm_b64']) == app.preview(job)
    meta = app.job(job)
    yaml_lines = body['yaml'].splitlines()
    for line in ('mode: trinary', 'resolution: 0.05', 'negate: 0', 'occupied_thresh: 0.65',
                 'free_thresh: 0.196'):
        assert line in yaml_lines
    assert yaml_lines[3] == 'origin: [{}, {}, 0.0]'.format(*meta['preview']['origin'][:2])
    side = body['sidecar']
    assert side['importer_version'] == 'dev' and side['kind'] == 'pdf' and side['page'] == 0
    assert side['source_file'] == {'name': 'upload.pdf', 'sha256': meta['sha256'],
                                   'bytes': meta['bytes']}
    cal = side['calibration']
    assert cal['method'] == 'proposal' and cal['rotation_deg'] == 15
    assert cal['origin'] == [600.0, 900.0] and cal['resolution'] == 0.05 and cal['close_m'] == 1.2
    assert cal['m_per_unit'] == pytest.approx(M_PT)
    assert side['extraction']['kind'] == 'vector' and side['extraction']['diagnostics']['mode']
    assert side['created_utc'].endswith('+00:00')
    assert len(json.dumps(side)) < 64 * 1024
    assert app.job(job)['last_install']['status'] == 200
    assert app.job(job)['last_install']['response'] == {'ok': True, 'replaced': False}


def test_optional_install_fields_are_sent_only_when_given(make_app, upstream):
    stub = upstream()
    app = make_app(stub.url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    body = {'name': 'plan', 'label': 'Lobby', 'floor': 2, 'map_start_pose': [1, 2, 0.5]}
    assert app.jcall('POST', f'/api/import/{job}/install', body)[0] == 200
    sent = stub.payload()
    assert (sent['label'], sent['floor'], sent['map_start_pose']) == ('Lobby', 2, [1.0, 2.0, 0.5])
    unfloored = {'name': 'plan', 'floor': None}
    assert app.jcall('POST', f'/api/import/{job}/install', unfloored)[0] == 200
    assert 'label' not in stub.payload() and stub.payload()['floor'] is None


@pytest.mark.parametrize('body', [
    {}, {'name': ''}, {'name': 5}, {'name': 'plan', 'force': True},
    {'name': 'plan', 'source': 'scan'}, {'name': 'plan', 'yaml': 'x'},
    {'name': 'plan', 'fleet_status_url': 'http://evil'}, {'name': 'plan', 'label': 5},
    {'name': 'plan', 'floor': 'one'}, {'name': 'plan', 'floor': True},
    {'name': 'plan', 'map_start_pose': [1, 2]}, {'name': 'plan', 'map_start_pose': 'x'},
    {'name': 'plan', 'map_start_pose': [1, 2, float('nan')]},
])
def test_bad_install_bodies_are_400_and_send_nothing(make_app, upstream, body):
    stub = upstream()
    app = make_app(stub.url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    status, out, _ = app.call('POST', f'/api/import/{job}/install', json.dumps(body), JSON)
    assert status == 400 and not stub.requests, out


def test_install_before_calibrate_is_409(make_app, upstream):
    stub = upstream()
    app = make_app(stub.url)
    job = app.new_job()
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})
    assert status == 409 and 'calibrate' in out['error'] and not stub.requests


def test_install_without_a_configured_fleet_status_is_503(make_app):
    app = make_app(None)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})
    assert status == 503 and 'FLEET_STATUS_URL' in out['error']


@pytest.mark.parametrize('status, body', [
    (409, {'error': "map 'plan' is a scan map"}), (422, {'error': 'pgm header invalid'}),
    (400, {'error': 'invalid map name'}), (413, {'error': 'too big'}),
    (404, {'error': 'map install not managed on this box'}),
    (500, {'error': 'install failed: x'})])
def test_fleet_status_errors_are_relayed_unchanged(make_app, upstream, status, body):
    stub = upstream(status=status, body=json.dumps(body).encode())
    app = make_app(stub.url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    got_status, raw, _ = app.call('POST', f'/api/import/{job}/install',
                                  json.dumps({'name': 'plan'}), JSON)
    assert got_status == status and json.loads(raw) == body
    assert app.job(job)['last_install'] == {
        'status': status, 'at': app.job(job)['last_install']['at'], 'response': body}


def test_an_unreachable_fleet_status_is_502_and_the_job_survives(make_app):
    url = closed_port_url()
    app = make_app(url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})
    assert status == 502 and out['upstream'] == url + '/api/maps/install'
    assert 'unreachable' in out['error']
    state = app.job(job)
    assert state['last_install']['status'] == 502 and state['preview'] is not None
    assert app.preview(job)


def test_upstream_timeout_and_non_json_and_redirect_are_502(make_app, upstream):
    slow = upstream(delay=1.5)
    app = make_app(slow.url, upstream_timeout=0.3)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})
    assert status == 502 and out['upstream'].startswith(slow.url)
    for stub in (upstream(status=200, body=b'<html>hi</html>', content_type='text/html'),
                 upstream(status=500, body=b'oops', content_type='text/plain'),
                 upstream(status=302, body=b'', headers={'Location': 'http://127.0.0.1:9/x'})):
        app2 = make_app(stub.url)
        job2 = app2.new_job()
        app2.calibrate(job2, method='proposal')
        status, out, _ = app2.jcall('POST', f'/api/import/{job2}/install', {'name': 'plan'})
        assert status == 502 and out['upstream'].startswith(stub.url), stub.status


def test_the_upstream_url_never_comes_from_a_request(make_app, upstream):
    good, evil = upstream(), upstream()
    app = make_app(good.url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    path = f'/api/import/{job}/install?fleet_status_url={evil.url}&url={evil.url}'
    assert app.call('POST', path, json.dumps({'name': 'plan'}), JSON)[0] == 200
    assert good.requests and not evil.requests
    assert app.call('POST', path, json.dumps({'name': 'plan', 'url': evil.url}), JSON)[0] == 400
    assert not evil.requests


@pytest.mark.parametrize('url', ['ftp://h/x', 'http://u:p@h', 'http://h/?q=1', 'http://h#f',
                                 'file:///etc', 'localhost:9003', 'http://'])
def test_upstream_url_validation(url):
    with pytest.raises(ValueError):
        srv.Importer(url)


def test_upstream_url_normalisation():
    assert srv.Importer('http://pi.local:9003/').fleet_status_url == 'http://pi.local:9003'
    assert srv.Importer('  https://pi:9003  ').fleet_status_url == 'https://pi:9003'
    assert srv.Importer('').fleet_status_url is None
    assert srv.Importer(None).fleet_status_url is None


def test_an_unusable_upstream_url_starts_unconfigured_instead_of_crashing(capsys):
    imp = srv.importer_from_env({'FLEET_STATUS_URL': 'http://:9003'})
    assert imp.fleet_status_url is None
    assert 'ignoring FLEET_STATUS_URL' in capsys.readouterr().err
    assert srv.importer_from_env({}).fleet_status_url is None
    assert srv.importer_from_env({'FLEET_STATUS_URL': 'http://pi:9003'}).fleet_status_url \
        == 'http://pi:9003'


# --- limits: busy, eviction, ttl, delete ---------------------------------------------------------

def test_a_second_heavy_operation_gets_429_with_retry_after(app):
    job = app.new_job()
    with app.importer.heavy:
        for method, path, body in (
                ('POST', '/api/import', sheet_bytes()), ('GET', f'/api/import/{job}/page/0', None),
                ('GET', f'/api/import/{job}/page/0.png', None),
                ('POST', f'/api/import/{job}/calibrate', json.dumps({'method': 'proposal'}))):
            status, raw, resp = app.call(method, path, body, JSON if method == 'POST' else None)
            assert status == 429 and json.loads(raw) == {'error': 'busy'}, path
            assert int(resp.getheader('Retry-After')) >= 1
        assert app.jcall('GET', f'/api/import/{job}')[0] == 200
        assert app.jcall('GET', '/api/health')[0] == 200
    assert app.calibrate(job, method='proposal')[0] == 200


def test_a_job_that_is_mid_operation_answers_429(app):
    job = app.new_job()
    app.calibrate(job, method='proposal')
    held = app.importer.jobs[job]
    with held.lock:
        for method, path, body in (
                ('POST', f'/api/import/{job}/calibrate', json.dumps({'method': 'proposal'})),
                ('POST', f'/api/import/{job}/install', json.dumps({'name': 'plan'})),
                ('GET', f'/api/import/{job}/preview.pgm', None),
                ('GET', f'/api/import/{job}/preview.png', None),
                ('GET', f'/api/import/{job}/page/0', None),
                ('DELETE', f'/api/import/{job}', None)):
            status, raw, resp = app.call(method, path, body, JSON if body else None)
            assert status == 429, path
        assert app.jcall('GET', f'/api/import/{job}')[0] == 200


def test_eviction_drops_the_oldest_idle_job_and_closes_its_pdf(make_app):
    app = make_app(None, max_jobs=2)
    first = app.new_job()
    first_pdf = app.importer.jobs[first].pdf
    second = app.new_job()
    third = app.new_job()
    assert app.jcall('GET', f'/api/import/{first}')[0] == 404
    assert first_pdf.doc.is_closed
    assert app.jcall('GET', f'/api/import/{second}')[0] == 200
    assert app.jcall('GET', f'/api/import/{third}')[0] == 200
    assert app.jcall('GET', '/api/health')[1]['jobs'] == 2


def test_eviction_skips_busy_jobs_and_refuses_when_all_are_busy(make_app):
    app = make_app(None, max_jobs=2)
    first, second = app.new_job(), app.new_job()
    with app.importer.jobs[first].lock:
        third = app.new_job()
        assert app.jcall('GET', f'/api/import/{first}')[0] == 200
        assert app.jcall('GET', f'/api/import/{second}')[0] == 404
        with app.importer.jobs[third].lock:
            status, raw, resp = app.call('POST', '/api/import', sheet_bytes())
            assert status == 429 and resp.getheader('Retry-After')
    assert app.jcall('GET', '/api/health')[1]['jobs'] == 2


def test_the_job_cap_defaults_to_three(make_app):
    assert srv.MAX_JOBS == 3 and srv.JOB_TTL_S == 2 * 3600
    app = make_app(None)
    ids = [app.new_job() for _ in range(4)]
    assert app.jcall('GET', f'/api/import/{ids[0]}')[0] == 404
    assert all(app.jcall('GET', f'/api/import/{i}')[0] == 200 for i in ids[1:])


def test_a_bad_upload_does_not_evict_a_good_job(make_app):
    app = make_app(None, max_jobs=1)
    job = app.new_job()
    assert app.upload(b'not a pdf')[0] == 415
    assert app.jcall('GET', f'/api/import/{job}')[0] == 200


def test_jobs_expire_after_the_ttl_and_requests_keep_them_alive(make_app):
    now = [1000.0]
    app = make_app(None, clock=lambda: now[0])
    job = app.new_job()
    pdf = app.importer.jobs[job].pdf
    now[0] += srv.JOB_TTL_S - 100
    assert app.job(job)['expires_in'] > 0
    now[0] += srv.JOB_TTL_S - 100
    assert app.jcall('GET', f'/api/import/{job}')[0] == 200
    now[0] += srv.JOB_TTL_S + 1
    assert app.jcall('GET', f'/api/import/{job}')[0] == 404
    assert pdf.doc.is_closed and app.jcall('GET', '/api/health')[1]['jobs'] == 0


def test_an_expired_job_that_is_busy_is_not_swept_from_under_its_operation(make_app):
    now = [0.0]
    app = make_app(None, clock=lambda: now[0], ttl=10)
    job = app.new_job()
    held = app.importer.jobs[job]
    now[0] += 100
    with held.lock:
        app.importer.sweep()
        assert job in app.importer.jobs and not held.pdf.doc.is_closed
    app.importer.sweep()
    assert job not in app.importer.jobs


def test_delete_drops_the_job_and_closes_the_pdf(app):
    job = app.new_job()
    pdf = app.importer.jobs[job].pdf
    status, out, _ = app.jcall('DELETE', f'/api/import/{job}')
    assert status == 200 and out == {'ok': True, 'id': job} and pdf.doc.is_closed
    assert app.jcall('GET', f'/api/import/{job}')[0] == 404
    assert app.jcall('DELETE', f'/api/import/{job}')[0] == 404


# --- integration: the REAL fleet-status handler --------------------------------------------------

_FLEET_SIBLINGS = ('bluetooth', 'gridstore', 'mapinstall', 'mapsafe', 'poi', 'sitestore',
                   'tagpose')


@pytest.fixture(scope='module')
def fleet_server_module():
    docker = types.ModuleType('docker')
    errors = types.ModuleType('docker.errors')
    errors.NotFound = type('NotFound', (Exception,), {})
    docker.errors = errors
    docker.from_env = lambda **_kw: object()
    mp = pytest.MonkeyPatch()
    saved = {n: sys.modules.pop(n, None) for n in _FLEET_SIBLINGS}
    for name, mod in {'docker': docker, 'docker.errors': errors,
                      'requests': types.ModuleType('requests')}.items():
        mp.setitem(sys.modules, name, mod)
    mp.syspath_prepend(str(_FLEET))
    try:
        spec = importlib.util.spec_from_file_location('fp_fleet_server_for_importer',
                                                      _FLEET / 'server.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        mp.undo()
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod


@pytest.fixture
def real_fleet(fleet_server_module, tmp_path, monkeypatch):
    root = tmp_path / 'sites'
    site_dir = root / 'office'
    (site_dir / 'maps').mkdir(parents=True)
    os.symlink('office', root / 'active')
    (site_dir / 'site.json').write_text(json.dumps({
        'version': 2, 'active_map': 'floor1', 'slam_mode': 'auto',
        'maps': {'floor1': {'label': 'Lobby', 'floor': 1}}}))
    monkeypatch.setattr(fleet_server_module, 'SITES_DIR', str(root))
    monkeypatch.setattr(fleet_server_module, 'SITE_SCAFFOLD', 'pi')
    server = ThreadingHTTPServer(('127.0.0.1', 0), fleet_server_module.Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.01},
                              daemon=True)
    thread.start()
    yield types.SimpleNamespace(url=f'http://127.0.0.1:{server.server_address[1]}',
                                maps=site_dir / 'maps', site_json=site_dir / 'site.json')
    server.shutdown()
    server.server_close()
    thread.join(timeout=5)


def test_install_through_the_importer_into_the_real_fleet_status(make_app, real_fleet):
    app = make_app(real_fleet.url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    pgm = app.preview(job)
    body = {'name': 'plan', 'label': 'Lobby plan', 'floor': 1, 'map_start_pose': [1.0, 2.0, 0.0]}
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', body)
    assert status == 200 and out['ok'] is True and out['replaced'] is False
    assert out['source'] == 'drawing' and out['active_map'] == 'floor1'
    assert (out['width'], out['height']) == pgm_grid(pgm).shape[::-1]
    assert (real_fleet.maps / 'plan.pgm').read_bytes() == pgm
    sidecar = json.loads((real_fleet.maps / 'plan.drawing.json').read_text())
    assert sidecar['source_file']['sha256'] == app.job(job)['sha256']
    assert sidecar['calibration']['method'] == 'proposal'
    site = json.loads(real_fleet.site_json.read_text())
    assert site['active_map'] == 'floor1'
    assert (site['maps']['floor1']['label'], site['maps']['floor1']['floor']) == ('Lobby', 1)
    assert site['maps']['floor1']['source'] == 'scan' and 'plan' in site['maps']
    entry = site['maps']['plan']
    assert entry['source'] == 'drawing' and entry['label'] == 'Lobby plan' and entry['floor'] == 1
    yaml_lines = (real_fleet.maps / 'plan.yaml').read_text().splitlines()
    assert yaml_lines[0] == 'image: plan.pgm' and 'resolution: 0.05' in yaml_lines
    assert app.job(job)['last_install']['response']['ok'] is True


def test_reinstall_replaces_the_drawing_map_with_the_recalibrated_grid(make_app, real_fleet):
    app = make_app(real_fleet.url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    first = app.preview(job)
    first_install = app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})
    assert first_install[1]['replaced'] is False
    app.calibrate(job, method='typed', ratio=96)
    second = app.preview(job)
    assert first != second
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})
    assert status == 200 and out['replaced'] is True
    assert (real_fleet.maps / 'plan.pgm').read_bytes() == second
    assert json.loads(real_fleet.site_json.read_text())['active_map'] == 'floor1'
    assert not list(real_fleet.maps.glob('*.tmp'))
    assert not (real_fleet.maps / '_quarantine').exists()


def test_a_scan_map_is_never_replaced_through_the_importer(make_app, real_fleet):
    (real_fleet.maps / 'plan.posegraph').write_bytes(b'g' * 60_000)
    (real_fleet.maps / 'plan.data').write_bytes(b'd' * 5_000)
    app = make_app(real_fleet.url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', {'name': 'plan'})
    assert status == 409 and 'error' in out
    assert not (real_fleet.maps / 'plan.pgm').exists()
    assert app.job(job)['last_install']['status'] == 409


def test_fleet_status_validation_errors_reach_the_ui(make_app, real_fleet):
    app = make_app(real_fleet.url)
    job = app.new_job()
    app.calibrate(job, method='proposal')
    status, out, _ = app.jcall('POST', f'/api/import/{job}/install', {'name': 'Bad Name!'})
    assert status == 400 and 'name' in out['error'].lower()


# --- A-101 sample sheet (skips without the PDFs) -------------------------------------------------

SAMPLES = pathlib.Path(os.environ.get(
    'SCOUT_FLOORPLAN_SAMPLES', str(_ROOT / 'docs' / 'floorplans')))


def test_a101_upload_proposes_the_title_block_scale(app):
    path = SAMPLES / 'the_zone_A-101_existing_conditions.pdf'
    if not path.is_file():
        pytest.skip(f'sample sheet not present: {path}')
    status, out, _ = app.upload(path.read_bytes(), path.name)
    assert status == 201
    scale = out['pages'][0]['scale']
    text = ' | '.join(scale['proposal']['evidence'])
    assert scale['proposal']['m_per_unit'] == pytest.approx(M_PT, rel=1e-9)
    assert '1/4"' in text and 'ARCH D' in text and scale['proposal']['confidence'] == 'high'
    cal = app.calibrate(out['id'], method='proposal')
    assert cal[0] == 200 and cal[1]['preview']['cells']['occupied'] > 1000


# --- deployment files (docker cannot run here; CI builds the image) ------------------------------

def test_dockerfile_is_non_root_slim_and_installs_without_dependencies():
    text = (_DIR / 'Dockerfile').read_text()
    assert 'FROM python:3.12-slim' in text and '--no-deps -r requirements.txt' in text
    assert 'COPY *.py /app/' in text and 'USER importer' in text and 'EXPOSE 9004' in text
    assert 'ARG GIT_SHA' in text and 'IMPORTER_VERSION' in text
    assert (_DIR / 'requirements.txt').read_text().count('pymupdf') >= 1


def test_every_module_the_server_imports_is_in_the_image():
    import re
    src = (_DIR / 'server.py').read_text()
    local = {p.stem for p in _DIR.glob('*.py')} - {'server'}
    imported = set(re.findall(r'^import (\w+)', src, re.M)) & local
    assert {'calib', 'pdfsource', 'planraster'} <= imported
    assert 'COPY *.py' in (_DIR / 'Dockerfile').read_text()


def test_compose_service_is_unprivileged_host_networked_and_capped():
    yaml = pytest.importorskip('yaml')
    compose = yaml.safe_load((_ROOT / 'companion' / 'docker-compose.yaml').read_text())
    svc = compose['services']['floorplan_importer']
    assert svc['image'] == 'ghcr.io/bg-bgi/scout-floorplan-importer:${COMPANION_TAG:-latest}'
    assert svc['network_mode'] == 'host' and svc['restart'] == 'unless-stopped'
    assert not svc.get('privileged') and 'volumes' not in svc and 'devices' not in svc
    assert svc['cpus'] > 0 and svc['mem_limit']
    env = dict(e.split('=', 1) for e in svc['environment'])
    assert env['FLEET_STATUS_URL'] == 'http://${PI_IP}:9003'
    assert env['IMPORTER_PORT'] == '${IMPORTER_PORT:-9004}'


def test_workflows_build_deploy_and_test_the_importer():
    yaml = pytest.importorskip('yaml')
    wf = _ROOT / '.github' / 'workflows'
    image = yaml.safe_load((wf / 'floorplan-importer-image.yml').read_text())
    assert image[True]['workflow_call']['inputs']['ref']['required'] is True
    text = (wf / 'floorplan-importer-image.yml').read_text()
    assert 'ghcr.io/bg-bgi/scout-floorplan-importer' in text and 'linux/amd64,linux/arm64' in text
    assert 'imagetools inspect' in text and 'GIT_SHA' in text
    deploy = yaml.safe_load((wf / 'deploy.yml').read_text())['jobs']
    job = deploy['floorplan-importer-companion']
    assert job['uses'] == './.github/workflows/floorplan-importer-image.yml'
    assert 'floorplan-importer-companion' in deploy['deploy-companion']['needs']
    assert 'floorplan-importer-companion' not in deploy['deploy-pi']['needs']
    ci = yaml.safe_load((wf / 'ci.yml').read_text())['jobs']
    steps = ' '.join(str(s.get('run', '')) for s in ci['importer-tests']['steps'])
    assert '--no-deps -r docker/floorplan-importer/requirements.txt' in steps
    assert 'test_floorplan_importer.py' in steps and 'requirements-dev.txt' in steps
    assert 'pymupdf' not in (_ROOT / 'requirements-dev.txt').read_text()
