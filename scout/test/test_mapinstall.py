"""Grid-only map install + raw map files (docker/fleet-status/mapinstall.py).

mapinstall is imported with fleet-status on sys.path (it imports its siblings by
name). The HTTP routes run for real: server.py is loaded with `docker`/`requests`
stubbed and served by a ThreadingHTTPServer on a loopback port 0.
"""
import base64
import http.client
import importlib
import importlib.util
import json
import os
import pathlib
import socket
import socketserver
import sys
import threading
import types
from http.server import ThreadingHTTPServer

import pytest

from scout.core import sites

_FLEET = pathlib.Path(__file__).resolve().parents[2] / 'docker' / 'fleet-status'
_SIBLINGS = ('bluetooth', 'gridstore', 'mapinstall', 'mapsafe', 'poi', 'sitestore',
             'tagpose')


def _stub_docker():
    docker = types.ModuleType('docker')
    errors = types.ModuleType('docker.errors')
    errors.NotFound = type('NotFound', (Exception,), {})
    docker.errors = errors
    docker.from_env = lambda **_kw: object()
    return {'docker': docker, 'docker.errors': errors,
            'requests': types.ModuleType('requests')}


@pytest.fixture(scope='module')
def modules():
    mp = pytest.MonkeyPatch()
    saved = {n: sys.modules.pop(n, None) for n in _SIBLINGS}
    for name, mod in _stub_docker().items():
        mp.setitem(sys.modules, name, mod)
    mp.syspath_prepend(str(_FLEET))
    try:
        mi = importlib.import_module('mapinstall')
        spec = importlib.util.spec_from_file_location(
            'fleet_server_mapinstall_test', _FLEET / 'server.py')
        server = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(server)
        yield types.SimpleNamespace(mi=mi, ss=sys.modules['sitestore'], server=server)
    finally:
        mp.undo()
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod


@pytest.fixture
def fleet(modules, tmp_path, monkeypatch):
    root = tmp_path / 'sites'
    site_dir = root / 'office'
    (site_dir / 'maps').mkdir(parents=True)
    os.symlink('office', root / 'active')
    monkeypatch.setattr(modules.server, 'SITES_DIR', str(root))
    monkeypatch.setattr(modules.server, 'SITE_SCAFFOLD', 'pi')
    (site_dir / 'site.json').write_text(json.dumps({
        'version': 2, 'active_map': 'floor1', 'slam_mode': 'auto',
        'maps': {'floor1': {'label': 'Lobby', 'floor': 1}}}))
    return types.SimpleNamespace(
        mi=modules.mi, ss=modules.ss, server=modules.server, root=root,
        site_dir=site_dir, maps=site_dir / 'maps')


class _LoopbackServer(ThreadingHTTPServer):
    """Skips HTTPServer's getfqdn() reverse lookup (about 1 s per bind on macOS) and
    disables Nagle so a header write followed by a body write is not delayed."""

    def server_bind(self):
        socketserver.TCPServer.server_bind(self)
        self.server_port = self.socket.getsockname()[1]

    def get_request(self):
        conn, addr = super().get_request()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return conn, addr


@pytest.fixture
def http_server(fleet):
    srv = _LoopbackServer(('127.0.0.1', 0), fleet.server.Handler)
    thread = threading.Thread(
        target=srv.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True)
    thread.start()
    yield srv.server_address[1]
    srv.shutdown()
    srv.server_close()
    thread.join(timeout=5)


def call(port, method, path, body=None, headers=None):
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=10)
    try:
        data = body if isinstance(body, bytes) or body is None else json.dumps(body)
        conn.request(method, path, body=data, headers=headers or {})
        resp = conn.getresponse()
        raw = resp.read()
        return resp.status, raw, resp
    finally:
        conn.close()


def post(port, body):
    status, raw, _ = call(port, 'POST', '/api/maps/install', body,
                          {'Content-Type': 'application/json'})
    return status, json.loads(raw)


def pgm(w=10, h=8, fill=254, maxval=255, extra=b''):
    return (f'P5\n{w} {h}\n{maxval}\n'.encode()
            + bytes([fill]) * (w * h) + extra)


YAML = ('image: other.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [-1.0, -2.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n'
        'free_thresh: 0.25\n')


def req(name='plan', **over):
    body = {'name': name, 'yaml': YAML, 'pgm_b64': base64.b64encode(pgm()).decode(),
            'sidecar': {'file_sha256': 'ab12', 'page': 1}}
    body.update(over)
    return body


def snapshot(*roots):
    out = {}
    for root in roots:
        for p in sorted(pathlib.Path(root).rglob('*')):
            if p.is_file():
                out[str(p)] = p.read_bytes()
    return out


def seed_scan(fleet, name='floor1', graph=True, grid=True):
    if graph:
        (fleet.maps / f'{name}.posegraph').write_bytes(b'g' * 60_000)
        (fleet.maps / f'{name}.data').write_bytes(b'd' * 5_000)
    if grid:
        (fleet.maps / f'{name}.pgm').write_bytes(pgm(fill=0))
        (fleet.maps / f'{name}.yaml').write_text(YAML.replace('other', name))


def site_json(fleet):
    return json.loads((fleet.site_dir / 'site.json').read_text())


# --- happy path ----------------------------------------------------------------

def test_install_fresh_registers_a_drawing_map(fleet, http_server):
    body = req(label='Lobby plan', floor=1, map_start_pose=[1.5, -2.0, 0.25])
    status, out = post(http_server, body)
    assert status == 200
    assert out['ok'] and out['replaced'] is False and out['source'] == 'drawing'
    assert (out['width'], out['height'], out['resolution']) == (10, 8, 0.05)
    assert out['origin'] == [-1.0, -2.0, 0.0]
    assert out['entry'] == {'label': 'Lobby plan', 'floor': 1, 'source': 'drawing',
                            'map_start_pose': [1.5, -2.0, 0.25]}
    assert out['active_map'] == 'floor1'
    assert (fleet.maps / 'plan.pgm').read_bytes() == pgm()
    yaml_text = (fleet.maps / 'plan.yaml').read_text()
    assert yaml_text.splitlines()[0] == 'image: plan.pgm'
    assert 'other.pgm' not in yaml_text
    assert json.loads((fleet.maps / 'plan.drawing.json').read_text()) == body['sidecar']
    assert site_json(fleet)['maps']['plan']['source'] == 'drawing'
    assert not list(fleet.maps.glob('*.tmp')) and not (fleet.maps / '_quarantine').exists()


def test_installed_yaml_round_trips_through_gridstore(fleet, http_server):
    assert post(http_server, req())[0] == 200
    gs = sys.modules['gridstore']
    meta = gs.read_map_yaml(str(fleet.maps / 'plan.yaml'))
    assert meta['image'] == 'plan.pgm' and meta['resolution'] == 0.05
    assert meta['origin'] == [-1.0, -2.0, 0.0] and meta['negate'] == 0
    assert (meta['occupied_thresh'], meta['free_thresh']) == (0.65, 0.25)
    grid = gs.load_saved_map(str(fleet.maps / 'plan.yaml'))
    assert (grid['width'], grid['height']) == (10, 8)


def test_status_reports_grid_true_posegraph_false_source_drawing(fleet, http_server):
    assert post(http_server, req())[0] == 200
    plan = fleet.server._site_meta('office')['maps']['plan']
    assert (plan['grid'], plan['posegraph'], plan['source']) == (True, False, 'drawing')
    assert plan['label'] == 'plan' and plan['floor'] is None
    assert sites.load_site(str(fleet.site_dir))['maps']['plan']['source'] == 'drawing'


def test_install_never_changes_active_map(fleet, http_server):
    assert post(http_server, req())[0] == 200
    raw = site_json(fleet)
    assert raw['active_map'] == 'floor1' and raw['default_map'] == 'floor1'
    assert fleet.server._site_meta('office')['resolved_mode'] == 'new'


def test_install_with_no_active_map_leaves_it_null(fleet, http_server):
    (fleet.site_dir / 'site.json').write_text(json.dumps({'version': 2, 'maps': {}}))
    assert post(http_server, req())[0] == 200
    assert site_json(fleet)['active_map'] is None


def test_install_keeps_an_active_map_that_is_not_registered(fleet, http_server):
    (fleet.site_dir / 'site.json').write_text(json.dumps(
        {'version': 2, 'active_map': 'ghost', 'maps': {}}))
    assert post(http_server, req())[0] == 200
    assert site_json(fleet)['active_map'] == 'ghost'


def test_install_upgrades_a_v1_site_json(fleet, http_server):
    (fleet.site_dir / 'site.json').write_text(json.dumps(
        {'version': 1, 'default_map': 'floor1', 'map_start_pose': [1.0, 2.0, 0.5]}))
    assert post(http_server, req())[0] == 200
    raw = site_json(fleet)
    assert raw['version'] == 2 and raw['active_map'] == 'floor1'
    assert raw['maps']['floor1']['map_start_pose'] == [1.0, 2.0, 0.5]
    assert raw['maps']['plan']['source'] == 'drawing'


def test_install_does_not_touch_the_scan_map_or_its_entry(fleet, http_server):
    seed_scan(fleet)
    before = snapshot(fleet.maps)
    entry = site_json(fleet)['maps']['floor1']
    assert post(http_server, req('floor1_plan'))[0] == 200
    after = snapshot(fleet.maps)
    assert {k: v for k, v in after.items() if 'floor1_plan' not in k} == before
    assert site_json(fleet)['maps']['floor1'] == {
        **entry, 'source': 'scan', 'map_start_pose': [0.0, 0.0, 0.0]}


def test_reinstall_replaces_in_place_and_updates_the_sidecar(fleet, http_server):
    assert post(http_server, req(label='First', floor=2))[0] == 200
    status, out = post(http_server, req(
        pgm_b64=base64.b64encode(pgm(12, 9, fill=0)).decode(),
        sidecar={'file_sha256': 'cd34', 'scale': 0.0123}))
    assert status == 200 and out['replaced'] is True
    assert (fleet.maps / 'plan.pgm').read_bytes() == pgm(12, 9, fill=0)
    assert json.loads((fleet.maps / 'plan.drawing.json').read_text()) == {
        'file_sha256': 'cd34', 'scale': 0.0123}
    assert out['entry']['label'] == 'First' and out['entry']['floor'] == 2
    assert not (fleet.maps / '_quarantine').exists()
    assert not list(fleet.maps.glob('*.tmp'))


def test_reinstall_can_change_the_metadata(fleet, http_server):
    assert post(http_server, req(label='First'))[0] == 200
    _, out = post(http_server, req(label='Second', floor=None,
                                   map_start_pose=[3.0, 4.0, 1.0]))
    assert out['entry']['label'] == 'Second'
    assert out['entry']['map_start_pose'] == [3.0, 4.0, 1.0]


def test_orphaned_drawing_files_with_a_sidecar_can_be_reinstalled(fleet, http_server):
    (fleet.maps / 'plan.pgm').write_bytes(pgm())
    (fleet.maps / 'plan.yaml').write_text(YAML)
    (fleet.maps / 'plan.drawing.json').write_text('{}')
    status, out = post(http_server, req())
    assert status == 200 and out['replaced'] is True
    assert site_json(fleet)['maps']['plan']['source'] == 'drawing'


# --- R11 / AE7: a scan map is never replaced -----------------------------------

@pytest.mark.parametrize('with_entry', [True, False])
def test_posegraph_present_is_409_and_everything_is_byte_identical(
        fleet, http_server, with_entry):
    seed_scan(fleet, 'plan')
    if with_entry:
        raw = site_json(fleet)
        raw['maps']['plan'] = {'label': 'Scanned', 'floor': 1}
        (fleet.site_dir / 'site.json').write_text(json.dumps(raw))
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req('plan'))
    assert status == 409 and 'scanned map' in out['error']
    assert snapshot(fleet.site_dir) == before


def test_posegraph_data_file_alone_is_409(fleet, http_server):
    (fleet.maps / 'plan.data').write_bytes(b'd' * 10)
    before = snapshot(fleet.site_dir)
    assert post(http_server, req())[0] == 409
    assert snapshot(fleet.site_dir) == before


def test_scan_entry_without_posegraph_is_409(fleet, http_server):
    raw = site_json(fleet)
    raw['maps']['plan'] = {'label': 'Scanned', 'source': 'scan'}
    (fleet.site_dir / 'site.json').write_text(json.dumps(raw))
    (fleet.maps / 'plan.pgm').write_bytes(pgm(fill=0))
    (fleet.maps / 'plan.yaml').write_text(YAML)
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req())
    assert status == 409 and 'scan map' in out['error']
    assert snapshot(fleet.site_dir) == before


def test_scan_entry_with_no_files_is_409(fleet, http_server):
    raw = site_json(fleet)
    raw['maps']['plan'] = {'label': 'Planned scan'}
    (fleet.site_dir / 'site.json').write_text(json.dumps(raw))
    before = snapshot(fleet.site_dir)
    assert post(http_server, req())[0] == 409
    assert snapshot(fleet.site_dir) == before


@pytest.mark.parametrize('files', [('.pgm',), ('.yaml',), ('.pgm', '.yaml')])
def test_grid_files_without_a_sidecar_are_409(fleet, http_server, files):
    for ext in files:
        (fleet.maps / f'plan{ext}').write_bytes(b'old')
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req())
    assert status == 409 and 'no drawing sidecar' in out['error']
    assert snapshot(fleet.site_dir) == before


def test_drawing_entry_without_a_sidecar_is_409(fleet, http_server):
    raw = site_json(fleet)
    raw['maps']['plan'] = {'source': 'drawing'}
    (fleet.site_dir / 'site.json').write_text(json.dumps(raw))
    before = snapshot(fleet.site_dir)
    assert post(http_server, req())[0] == 409
    assert snapshot(fleet.site_dir) == before


def test_drawing_entry_whose_scan_twin_has_a_graph_is_409(fleet, http_server):
    assert post(http_server, req())[0] == 200
    (fleet.maps / 'plan.posegraph').write_bytes(b'g' * 60_000)
    before = snapshot(fleet.site_dir)
    assert post(http_server, req(label='again'))[0] == 409
    assert snapshot(fleet.site_dir) == before


def test_force_is_not_a_field(fleet, http_server):
    seed_scan(fleet, 'plan')
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req('plan', force=True))
    assert status == 400 and 'force' in out['error']
    assert snapshot(fleet.site_dir) == before


def test_corrupt_site_json_is_refused_not_overwritten(fleet, http_server):
    (fleet.site_dir / 'site.json').write_text('{not json')
    status, _ = post(http_server, req())
    assert status == 409
    assert (fleet.site_dir / 'site.json').read_text() == '{not json'
    assert not (fleet.maps / 'plan.pgm').exists()


def test_unreadable_site_json_is_500_and_writes_nothing(fleet, http_server):
    (fleet.site_dir / 'site.json').unlink()
    (fleet.site_dir / 'site.json').mkdir()
    status, out = post(http_server, req())
    assert status == 500 and 'install failed' in out['error']
    assert not list(fleet.maps.glob('plan*'))


# --- name, label, floor, pose, source ------------------------------------------

@pytest.mark.parametrize('name', [
    '_pending', '_autosave', '_quarantine', 'plan\n', '\nplan', 'Plan', 'a b', 'a.b',
    '../x', 'a/b', '', 'x' * 33, 123, None, ['plan']])
def test_bad_names_are_400_and_write_nothing(fleet, http_server, name):
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req(name=name))
    assert status == 400 and 'invalid map name' in out['error']
    assert snapshot(fleet.site_dir) == before


def test_missing_name_is_400(fleet, http_server):
    body = req()
    del body['name']
    assert post(http_server, body)[0] == 400


def test_name_of_32_chars_is_accepted(fleet, http_server):
    assert post(http_server, req('a' + 'b' * 31))[0] == 200


@pytest.mark.parametrize('label', [
    'x' * 65, 'a\nb', 'a\x00b', 'tab\there', 'bell\x07', '\x7f', 'c1\x85', 7, None, []])
def test_bad_labels_are_400(fleet, http_server, label):
    status, out = post(http_server, req(label=label))
    assert status == 400 and 'label' in out['error']
    assert not (fleet.maps / 'plan.pgm').exists()


def test_label_of_64_chars_and_unicode_is_accepted(fleet, http_server):
    label = 'é' * 64
    status, out = post(http_server, req(label=label))
    assert status == 200 and out['entry']['label'] == label


@pytest.mark.parametrize('floor', ['one', True, 1.5, [1]])
def test_bad_floors_are_400(fleet, http_server, floor):
    assert post(http_server, req(floor=floor))[0] == 400


@pytest.mark.parametrize('floor', [None, 0, -1, 12])
def test_good_floors(fleet, http_server, floor):
    status, out = post(http_server, req(floor=floor))
    assert status == 200 and out['entry']['floor'] == floor


@pytest.mark.parametrize('pose', [
    [0, 0], [0, 0, 0, 0], 'x', ['a', 0, 0], [True, 0, 0], [None, 0, 0],
    [float('nan'), 0, 0], [float('inf'), 0, 0], [1e9, 0, 0]])
def test_bad_poses_are_400(fleet, http_server, pose):
    status, out = post(http_server, req(map_start_pose=pose))
    assert status == 400 and 'map_start_pose' in out['error']


@pytest.mark.parametrize('source', ['scan', 'bim', '', None, 'Drawing', 3])
def test_source_other_than_drawing_is_400(fleet, http_server, source):
    status, out = post(http_server, req(source=source))
    assert status == 400 and 'source' in out['error']
    assert not (fleet.maps / 'plan.pgm').exists()


def test_source_drawing_is_accepted(fleet, http_server):
    assert post(http_server, req(source='drawing'))[0] == 200


# --- body, size guards, pgm ----------------------------------------------------

@pytest.mark.parametrize('raw', [b'', b'{', b'[]', b'"x"', b'null', b'\xff\xfe'])
def test_malformed_body_is_400(fleet, http_server, raw):
    status, _ = post(http_server, raw)
    assert status == 400


def test_check_body_length_rules(modules):
    mi = modules.mi
    assert mi.check_body_length(None)[0] == 411
    assert mi.check_body_length('abc')[0] == 400
    assert mi.check_body_length('-5')[0] == 400
    assert mi.check_body_length(str(mi.MAX_BODY_BYTES + 1))[0] == 413
    assert mi.check_body_length(str(mi.MAX_BODY_BYTES)) is None
    assert mi.check_body_length('0') is None


def test_oversize_content_length_is_413_before_the_body_is_read(fleet, http_server):
    conn = http.client.HTTPConnection('127.0.0.1', http_server, timeout=10)
    try:
        conn.putrequest('POST', '/api/maps/install')
        conn.putheader('Content-Type', 'application/json')
        conn.putheader('Content-Length', str(fleet.mi.MAX_BODY_BYTES + 1))
        conn.endheaders()
        resp = conn.getresponse()
        assert resp.status == 413
    finally:
        conn.close()
    assert not (fleet.maps / 'plan.pgm').exists()


def test_short_body_is_400(fleet, http_server):
    conn = http.client.HTTPConnection('127.0.0.1', http_server, timeout=10)
    try:
        conn.putrequest('POST', '/api/maps/install')
        conn.putheader('Content-Length', '500')
        conn.endheaders()
        conn.send(b'{"name": "plan"}')
        conn.sock.shutdown(1)
        assert conn.getresponse().status == 400
    finally:
        conn.close()


def test_decoded_pgm_over_the_cap_is_413(fleet, http_server, monkeypatch):
    monkeypatch.setattr(fleet.mi, 'MAX_INSTALL_BYTES', 50)
    status, out = post(http_server, req())
    assert status == 413 and 'pgm' in out['error']
    assert not (fleet.maps / 'plan.pgm').exists()


def test_cell_cap_comes_from_the_header(fleet, http_server):
    cap = fleet.mi.MAX_CELLS
    assert cap == 4000 * 4000
    over = base64.b64encode(b'P5\n4001 4000\n255\n' + b'\x00' * 16).decode()
    status, out = post(http_server, req(pgm_b64=over))
    assert status == 422 and 'cells' in out['error']
    assert not (fleet.maps / 'plan.pgm').exists()


def test_cell_cap_boundary(fleet, http_server, monkeypatch):
    monkeypatch.setattr(fleet.mi, 'MAX_CELLS', 80)
    assert post(http_server, req())[0] == 200
    too_big = base64.b64encode(pgm(9, 9)).decode()
    assert post(http_server, req(pgm_b64=too_big))[0] == 422


@pytest.mark.parametrize('payload', [
    b'P2\n2 2\n255\n0 0 0 0',
    b'P5\n10\n',
    b'P5\n0 5\n255\n',
    b'P5\n-3 5\n255\n',
    b'P5\nab cd\n255\n',
    pgm(maxval=65535),
    pgm(maxval=127),
    pgm()[:-1],
    pgm(extra=b'x'),
    b'not a pgm',
    b''])
def test_bad_pgm_is_422_and_writes_nothing(fleet, http_server, payload):
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req(pgm_b64=base64.b64encode(payload).decode()))
    assert status == 422, out
    assert snapshot(fleet.site_dir) == before


@pytest.mark.parametrize('value', ['not base64!!', 'AAA', 5, None, ['x']])
def test_bad_base64_is_400(fleet, http_server, value):
    assert post(http_server, req(pgm_b64=value))[0] == 400


def test_pgm_header_comment_is_accepted(fleet, http_server):
    payload = b'P5\n# made by importer\n4 3\n255\n' + b'\xfe' * 12
    status, out = post(http_server, req(pgm_b64=base64.b64encode(payload).decode()))
    assert status == 200 and (out['width'], out['height']) == (4, 3)
    assert (fleet.maps / 'plan.pgm').read_bytes() == payload


# --- yaml ----------------------------------------------------------------------

def _yaml_without(key):
    return '\n'.join(line for line in YAML.splitlines()
                     if not line.startswith(key)) + '\n'


@pytest.mark.parametrize('key', ['resolution', 'origin', 'negate',
                                 'occupied_thresh', 'free_thresh'])
def test_missing_yaml_key_is_422_and_names_it(fleet, http_server, key):
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req(yaml=_yaml_without(key)))
    assert status == 422 and key in out['error']
    assert snapshot(fleet.site_dir) == before


@pytest.mark.parametrize('old,new', [
    ('resolution: 0.05', 'resolution: 0'),
    ('resolution: 0.05', 'resolution: -0.05'),
    ('resolution: 0.05', 'resolution: nan'),
    ('resolution: 0.05', 'resolution: inf'),
    ('resolution: 0.05', 'resolution: 11'),
    ('resolution: 0.05', 'resolution: fine'),
    ('origin: [-1.0, -2.0, 0.0]', 'origin: [-1.0, -2.0]'),
    ('origin: [-1.0, -2.0, 0.0]', 'origin: [a, b, c]'),
    ('origin: [-1.0, -2.0, 0.0]', 'origin: [1, 2, 3, 4]'),
    ('negate: 0', 'negate: 2'),
    ('negate: 0', 'negate: x'),
    ('free_thresh: 0.25', 'free_thresh: 0.65'),
    ('free_thresh: 0.25', 'free_thresh: -0.1'),
    ('occupied_thresh: 0.65', 'occupied_thresh: 1.5'),
    ('mode: trinary', 'mode: bogus')])
def test_bad_yaml_values_are_422(fleet, http_server, old, new):
    status, _ = post(http_server, req(yaml=YAML.replace(old, new)))
    assert status == 422
    assert not (fleet.maps / 'plan.yaml').exists()


@pytest.mark.parametrize('value', [None, 5, ['x']])
def test_yaml_must_be_a_string(fleet, http_server, value):
    assert post(http_server, req(yaml=value))[0] == 400


def test_oversize_yaml_is_413(fleet, http_server):
    assert post(http_server, req(yaml=YAML + '# ' + 'x' * 20_000))[0] == 413


def test_yaml_is_regenerated_from_the_validated_values(fleet, http_server):
    text = ('image: ../../etc/passwd\nmode: scale\nresolution: 0.1  # coarse\n'
            'origin: [ 0, 0, 1.57 ]\nnegate: 1\noccupied_thresh: 0.9\nfree_thresh: 0.1\n'
            'extra_key: sneaky\nimage: again.pgm\n')
    assert post(http_server, req(yaml=text))[0] == 200
    stored = (fleet.maps / 'plan.yaml').read_text()
    assert stored == ('image: plan.pgm\nmode: scale\nresolution: 0.1\n'
                      'origin: [0.0, 0.0, 1.57]\nnegate: 1\n'
                      'occupied_thresh: 0.9\nfree_thresh: 0.1\n')


def test_yaml_mode_defaults_to_trinary(fleet, http_server):
    assert post(http_server, req(yaml=_yaml_without('mode')))[0] == 200
    assert 'mode: trinary' in (fleet.maps / 'plan.yaml').read_text()


# --- sidecar -------------------------------------------------------------------

@pytest.mark.parametrize('sidecar', [[1], 'x', 5, None])
def test_sidecar_must_be_an_object(fleet, http_server, sidecar):
    assert post(http_server, req(sidecar=sidecar))[0] == 400


def test_missing_sidecar_is_400(fleet, http_server):
    body = req()
    del body['sidecar']
    assert post(http_server, body)[0] == 400


def test_oversize_sidecar_is_413(fleet, http_server, monkeypatch):
    monkeypatch.setattr(fleet.mi, 'MAX_SIDECAR_BYTES', 40)
    assert post(http_server, req(sidecar={'k': 'v' * 100}))[0] == 413
    assert not (fleet.maps / 'plan.pgm').exists()


def test_empty_sidecar_object_is_accepted(fleet, http_server):
    assert post(http_server, req(sidecar={}))[0] == 200
    assert (fleet.maps / 'plan.drawing.json').exists()


# --- gates ---------------------------------------------------------------------

def test_plain_scaffold_is_404_and_writes_nothing(fleet, http_server, monkeypatch):
    monkeypatch.setattr(fleet.server, 'SITE_SCAFFOLD', 'plain')
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req())
    assert status == 404 and 'not managed' in out['error']
    assert snapshot(fleet.site_dir) == before


def test_no_sites_dir_is_404(fleet, http_server, monkeypatch):
    monkeypatch.setattr(fleet.server, 'SITES_DIR', '')
    assert post(http_server, req())[0] == 404


def test_no_active_site_is_409(fleet, http_server):
    (fleet.root / 'active').unlink()
    assert post(http_server, req())[0] == 409


def test_active_site_without_a_maps_dir_is_404(fleet, http_server):
    for p in fleet.maps.iterdir():
        p.unlink()
    fleet.maps.rmdir()
    assert post(http_server, req())[0] == 404


def test_install_needs_post(fleet, http_server):
    status, _, _ = call(http_server, 'GET', '/api/maps/install')
    assert status == 404
    status, _, resp = call(http_server, 'OPTIONS', '/api/maps/install')
    assert status == 204
    assert 'POST' in resp.getheader('Access-Control-Allow-Methods')
    assert 'GET' in resp.getheader('Access-Control-Allow-Methods')
    assert 'Content-Type' in resp.getheader('Access-Control-Allow-Headers')


# --- failure rollback ----------------------------------------------------------

def test_registration_failure_on_a_fresh_install_leaves_no_orphan(
        fleet, http_server, monkeypatch):
    def boom(*_a, **_kw):
        raise OSError(28, 'No space left on device')
    monkeypatch.setattr(fleet.ss, 'write_site_json', boom)
    before = snapshot(fleet.site_dir)
    status, out = post(http_server, req())
    assert status == 500 and 'No space left' in out['error']
    assert snapshot(fleet.site_dir) == before
    assert not list(fleet.maps.glob('plan*'))


def test_file_write_failure_on_a_fresh_install_removes_what_was_written(
        fleet, http_server, monkeypatch):
    real = os.replace
    calls = []

    def flaky(src, dst, *a, **kw):
        calls.append(dst)
        if len(calls) == 2:
            raise OSError(5, 'I/O error')
        return real(src, dst, *a, **kw)
    before = snapshot(fleet.site_dir)
    with monkeypatch.context() as m:
        m.setattr(os, 'replace', flaky)
        status, _ = post(http_server, req())
    assert status == 500
    assert snapshot(fleet.site_dir) == before
    assert not list(fleet.maps.glob('plan*'))


def test_registration_failure_on_a_replace_keeps_the_entry_a_drawing(
        fleet, http_server, monkeypatch):
    assert post(http_server, req(label='First'))[0] == 200
    new_pgm = base64.b64encode(pgm(6, 6, fill=0)).decode()

    def boom(*_a, **_kw):
        raise OSError(28, 'No space left on device')
    with monkeypatch.context() as m:
        m.setattr(fleet.ss, 'write_site_json', boom)
        status, _ = post(http_server, req(pgm_b64=new_pgm, label='Second'))
    assert status == 500
    assert (fleet.maps / 'plan.pgm').read_bytes() == pgm(6, 6, fill=0)
    entry = site_json(fleet)['maps']['plan']
    assert entry['source'] == 'drawing' and entry['label'] == 'First'
    assert post(http_server, req(pgm_b64=new_pgm, label='Second'))[0] == 200


# --- raw GET -------------------------------------------------------------------

def test_get_raw_files_round_trip(fleet, http_server):
    assert post(http_server, req())[0] == 200
    status, raw, resp = call(http_server, 'GET', '/api/maps/plan.pgm')
    assert status == 200 and raw == pgm()
    assert resp.getheader('Content-Type') == 'image/x-portable-graymap'
    assert resp.getheader('Cache-Control') == 'no-store'
    assert resp.getheader('Access-Control-Allow-Origin') == '*'
    assert resp.getheader('Content-Length') == str(len(raw))
    status, raw, resp = call(http_server, 'GET', '/api/maps/plan.yaml')
    assert status == 200 and raw == (fleet.maps / 'plan.yaml').read_bytes()
    assert resp.getheader('Content-Type').startswith('text/yaml')
    assert resp.getheader('Cache-Control') == 'no-store'


def test_get_serves_scan_maps_too(fleet, http_server):
    seed_scan(fleet)
    status, raw, _ = call(http_server, 'GET', '/api/maps/floor1.pgm')
    assert status == 200 and raw == pgm(fill=0)


def test_get_ignores_a_query_string(fleet, http_server):
    assert post(http_server, req())[0] == 200
    assert call(http_server, 'GET', '/api/maps/plan.pgm?t=123')[0] == 200


@pytest.mark.parametrize('path', [
    '/api/maps/nope.pgm', '/api/maps/nope.yaml', '/api/maps/plan.png',
    '/api/maps/plan.posegraph', '/api/maps/plan.drawing.json', '/api/maps/plan', '/api/maps/'])
def test_get_unknown_is_404_or_400(fleet, http_server, path):
    assert post(http_server, req())[0] == 200
    status, _, _ = call(http_server, 'GET', path)
    assert status in (400, 404)
    assert call(http_server, 'GET', '/api/maps/nope.pgm')[0] == 404


@pytest.mark.parametrize('path', [
    '/api/maps/..%2Fsite.json.pgm', '/api/maps/%2e%2e%2fsite.pgm',
    '/api/maps/..%2F..%2Fetc%2Fpasswd.yaml', '/api/maps/_pending.pgm',
    '/api/maps/_autosave.yaml', '/api/maps/plan%0A.pgm', '/api/maps/a%2Fb.pgm',
    '/api/maps/Plan.pgm', '/api/maps/.pgm', '/api/maps/%00.pgm'])
def test_get_bad_names_are_400(fleet, http_server, path):
    (fleet.maps / '_pending.pgm').write_bytes(b'secret')
    (fleet.site_dir / 'secret.yaml').write_text('x: 1')
    status, raw, _ = call(http_server, 'GET', path)
    assert status == 400 and b'secret' not in raw


def test_get_without_a_maps_dir_is_404(fleet, http_server):
    (fleet.root / 'active').unlink()
    assert call(http_server, 'GET', '/api/maps/plan.pgm')[0] == 404


# --- unit-level helpers --------------------------------------------------------

@pytest.mark.parametrize('name,ok', [
    ('plan', True), ('a', True), ('0abc', True), ('a-b_c', True), ('_x', False),
    ('-x', False), ('plan\n', False), ('', False), (None, False), (b'plan', False)])
def test_valid_name(modules, name, ok):
    assert modules.mi.valid_name(name) is ok


def test_name_pattern_matches_the_shared_contract(modules):
    assert modules.mi.NAME_RE.pattern == sites.SITE_NAME_RE.pattern


def test_concurrent_installs_of_different_names_both_register(fleet, http_server):
    results = {}

    def go(name):
        results[name] = post(http_server, req(name))[0]
    threads = [threading.Thread(target=go, args=(f'p{i}',)) for i in range(6)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=30)
    assert set(results.values()) == {200}
    assert set(site_json(fleet)['maps']) == {'floor1', *(f'p{i}' for i in range(6))}
