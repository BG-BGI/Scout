"""Plumbing shared by the importer's HTTP tests (not a test module).

load_server() imports docker/floorplan-importer/server.py with its siblings importable; App
drives a real ImporterServer on a loopback port 0; Upstream is a stub fleet_status; real_fleet()
stands up the REAL fleet-status handler (SITE_SCAFFOLD=pi, temp sites dir) in-process.
"""
import contextlib
import http.client
import importlib
import importlib.util
import json
import os
import pathlib
import socket
import sys
import threading
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
IMPORTER = ROOT / 'docker' / 'floorplan-importer'
FLEET = ROOT / 'docker' / 'fleet-status'
JSON = {'Content-Type': 'application/json'}
FLEET_SIBLINGS = ('bluetooth', 'gridstore', 'mapinstall', 'mapsafe', 'poi', 'sitestore',
                  'tagpose')


def load_server(name):
    sys.path.insert(0, str(IMPORTER))
    try:
        importlib.import_module('pdfsource')
        spec = importlib.util.spec_from_file_location(name, IMPORTER / 'server.py')
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(IMPORTER))


def pgm_grid(raw):
    head, _, rest = raw.partition(b'\n255\n')
    w, h = (int(v) for v in head.split(b'\n', 1)[1].split())
    return np.frombuffer(rest, dtype=np.uint8).reshape(h, w)


class App:
    def __init__(self, importer, server, thread):
        self.importer, self.server, self.thread = importer, server, thread
        self.port = server.server_address[1]

    def call(self, method, path, body=None, headers=None, timeout=60):
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
        suffix = f'?filename={filename}' if filename else ''
        return self.jcall('POST', '/api/import' + suffix, data,
                          {'Content-Type': 'application/octet-stream'})

    def state(self, job):
        return self.jcall('GET', f'/api/import/{job}')[1]

    def head_only(self, declared, head, path='/api/import', close_write=False):
        """Declare `declared` body bytes but send only `head`: proves the server stops early.
        With close_write the client then signals EOF, so a server still reading sees a short
        body instead of waiting."""
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=10)
        try:
            conn.putrequest('POST', path)
            conn.putheader('Content-Length', str(declared))
            conn.endheaders()
            if head:
                conn.send(head)
            if close_write:
                conn.sock.shutdown(socket.SHUT_WR)
            resp = conn.getresponse()
            return resp.status, json.loads(resp.read())
        finally:
            conn.close()


def start_app(srv, fleet_url=None, cors_origin='*', **kwargs):
    importer = srv.Importer(fleet_url, **kwargs)
    server = srv.ImporterServer(('127.0.0.1', 0), importer, cors_origin)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.01},
                              daemon=True)
    thread.start()
    return App(importer, server, thread)


def stop_app(srv, app):
    app.server.shutdown()
    app.server.server_close()
    app.thread.join(timeout=5)
    for job in list(app.importer.jobs.values()):
        app.importer._release(job)


class Upstream:
    """A stub fleet_status: records POSTs and answers a canned status/body."""

    def __init__(self, status=200, body=b'{"ok": true, "replaced": false}'):
        self.status, self.body, self.requests = status, body, []
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                raw = self.rfile.read(int(self.headers.get('Content-Length', 0)))
                outer.requests.append({'path': self.path, 'raw': raw})
                self.send_response(outer.status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(outer.body)))
                self.end_headers()
                self.wfile.write(outer.body)

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


def closed_port_url():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return f'http://127.0.0.1:{port}'


@contextlib.contextmanager
def real_fleet(tmp_path):
    """The real fleet-status handler serving a temp active site with one scan map."""
    docker = types.ModuleType('docker')
    errors = types.ModuleType('docker.errors')
    errors.NotFound = type('NotFound', (Exception,), {})
    docker.errors = errors
    docker.from_env = lambda **_kw: object()
    saved = {n: sys.modules.pop(n, None) for n in FLEET_SIBLINGS}
    stubs = {'docker': docker, 'docker.errors': errors, 'requests': types.ModuleType('requests')}
    shadowed = {n: sys.modules.get(n) for n in stubs}
    sys.modules.update(stubs)
    sys.path.insert(0, str(FLEET))
    root = tmp_path / 'sites'
    site_dir = root / 'office'
    (site_dir / 'maps').mkdir(parents=True)
    os.symlink('office', root / 'active')
    (site_dir / 'site.json').write_text(json.dumps({
        'version': 2, 'active_map': 'floor1', 'slam_mode': 'auto',
        'maps': {'floor1': {'label': 'Lobby', 'floor': 1}}}))
    server = thread = None
    try:
        spec = importlib.util.spec_from_file_location('fp_fleet_server_for_harness',
                                                      FLEET / 'server.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.SITES_DIR = str(root)
        module.SITE_SCAFFOLD = 'pi'
        server = ThreadingHTTPServer(('127.0.0.1', 0), module.Handler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.01},
                                  daemon=True)
        thread.start()
        yield types.SimpleNamespace(url=f'http://127.0.0.1:{server.server_address[1]}',
                                    maps=site_dir / 'maps', site_json=site_dir / 'site.json')
    finally:
        if server:
            server.shutdown()
            server.server_close()
            thread.join(timeout=5)
        sys.path.remove(str(FLEET))
        for name, mod in shadowed.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod
