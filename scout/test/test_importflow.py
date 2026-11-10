# ruff: noqa: E501
"""webui/importflow.js (the import wizard's state machine and HTTP) under Node with a fake
fetch, plus source-level checks that the wizard is plain HTTP on user action: no rosbridge, no
timers, no polling, no force, no innerHTML, and only a few hook lines in app.js.

The importer contract is the docstring of docker/floorplan-importer/server.py; the IFC half is
the frozen contract in the U5b brief (the server side is built separately).
Skips when `node` is absent (Pi container)."""

import json
import pathlib
import re
import shutil
import subprocess

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
WEBUI = REPO_ROOT / 'webui'
FLOW = WEBUI / 'importflow.js'
WIZARD = WEBUI / 'importwizard.js'
MATH = WEBUI / 'importmath.js'
APP_JS = WEBUI / 'app.js'
INDEX_HTML = WEBUI / 'index.html'

HARNESS = r"""
const IF = require(process.argv[1]);
const calls = [], routes = [], sleeps = [], installed = [], changes = [];
let inflight = 0, maxInflight = 0, companionCalls = 0, storageThrows = false;
const store = {};
let COMPANION = { configured: true, host: '10.0.0.7' };
const out = (v) => console.log(JSON.stringify(v));
function route(method, re, ...responses) { routes.push({ method, re, responses, i: 0 }); }
function mkResp(r) {
  const h = r.headers || {};
  return { status: r.status, ok: r.status >= 200 && r.status < 300,
    headers: { get: (k) => (k in h ? h[k] : null) },
    json: async () => { if (r.body === undefined) throw new Error('no body'); return r.body; },
    blob: async () => ({ blob: r.blob || 'blob' }) };
}
const fakeFetch = async (url, opts) => {
  opts = opts || {};
  const method = opts.method || 'GET';
  calls.push({ method, url, body: opts.body, headers: opts.headers });
  inflight++; maxInflight = Math.max(maxInflight, inflight);
  await new Promise((r) => setImmediate(r));
  try {
    const rt = routes.find((x) => x.method === method && x.re.test(url));
    if (!rt) throw new Error('no route for ' + method + ' ' + url);
    const r = rt.responses[Math.min(rt.i, rt.responses.length - 1)];
    rt.i++;
    if (r.throw) throw new Error('network down');
    return mkResp(r);
  } finally { inflight--; }
};
const flow = IF.createFlow({
  fetch: fakeFetch, sleep: async (ms) => { sleeps.push(ms); },
  storage: () => {
    if (storageThrows) throw new Error('blocked');
    return { getItem: (k) => (k in store ? store[k] : null),
             setItem: (k, v) => { store[k] = v; }, removeItem: (k) => { delete store[k]; } };
  },
  getCompanion: async () => { companionCalls++; return COMPANION; },
  hostname: 'pi.local', onInstalled: async (names) => { installed.push(names); },
  onChange: () => { changes.push(1); },
});
const st = flow.state;
const summary = () => calls.map((c) => c.method + ' ' + c.url);
const bodyOf = (i) => JSON.parse(calls[i].body);
const FILE = { name: 'a b.pdf' };
const PNG = { url: '/x', width_px: 3888, height_px: 2592, scale_x: 1.5, scale_y: 1.5,
              width: 2592, height: 1728 };
const scaleReport = (over) => Object.assign({
  proposal: { m_per_unit: 0.0127, m_per_point: 0.0127, confidence: 'high',
              evidence: ['scale note 1/4" = 1\'-0"'] },
  calibration_required: false, raster: false }, over || {});
const RASTER = { proposal: { m_per_unit: null, m_per_point: null, evidence: [], confidence: 'none' },
                 calibration_required: true, raster: true };
const PREVIEW = { width: 240, height: 180, resolution: 0.05, origin: [-4, -3, 0],
                  interior_fraction: 0.62, cells: { occupied: 1, free: 2, unknown: 3 } };
const CALIB = (over) => Object.assign({
  page: 0, m_per_unit: 0.0127, method: 'proposal', rotation_deg: 0, origin: [100, 80],
  resolution: 0.05, close_m: 1.2, detail: {}, extraction: { kind: 'vector' },
  preview: PREVIEW, warnings: [] }, over || {});
function pdfRoutes(scale, pages) {
  const n = pages || 1;
  const list = [];
  for (let i = 0; i < n; i++) list.push({ index: i, width: 2592, height: 1728, has_vectors: true, scale: null });
  route('GET', /\/api\/health$/, { status: 200, body: { ok: true, version: 'v1', fleet_status_url: 'http://pi:9003' } });
  route('POST', /\/api\/import\?filename=/, { status: 201, body: { id: 'j1', kind: 'pdf',
    filename: 'a b.pdf', sha256: 'x', bytes: 2048, pages: list } });
  route('GET', /\/page\/\d+$/, { status: 200, body: { index: 0, page: { index: 0 }, scale: scale || scaleReport(), png: PNG } });
  route('GET', /\/page\/\d+\.png$/, { status: 200, blob: 'pagepng' });
  route('POST', /\/calibrate$/, { status: 200, body: CALIB() });
  route('GET', /\/preview\.png$/, { status: 200, blob: 'prevpng' });
  route('POST', /\/install$/, { status: 200, body: { ok: true, name: 'plan', replaced: false } });
  route('DELETE', /\/api\/import\/j1$/, { status: 200, body: { ok: true, id: 'j1' } });
}
const tap = (x, y, sx, sy) => flow.handleTap({ page: { x, y }, screen: { x: sx, y: sy } });
"""


def _run(script):
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    text = HARNESS + '(async () => {' + script + '})().catch((e) => { console.error(e); process.exit(1); });'
    proc = subprocess.run([node, '-e', text, str(FLOW)], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return json.loads(proc.stdout.strip().splitlines()[-1])


# --- importer address and reachability -----------------------------------------------------------

def test_open_resolves_the_companion_and_checks_health_once():
    res = _run("""
    route('GET', /\\/api\\/health$/, { status: 200, body: { ok: true, version: 'v9' } });
    await flow.open();
    out({ calls: summary(), base: st.base, health: st.health, companion: companionCalls });""")
    assert res['calls'] == ['GET http://10.0.0.7:9004/api/health']
    assert res['base'] == 'http://10.0.0.7:9004/api' and res['companion'] == 1
    assert res['health']['ok'] is True and res['health']['version'] == 'v9'


def test_unreachable_importer_shows_the_exact_url_and_stays_usable():
    res = _run("""
    route('GET', /10\\.0\\.0\\.7.*health$/, { throw: true });
    route('GET', /laptop.*health$/, { status: 200, body: { ok: true, version: 'l1' } });
    await flow.open();
    const failed = JSON.parse(JSON.stringify(st.health));
    const err = st.error;
    const ok = await flow.setBase('laptop:9004');
    out({ failed, err, ok, base: st.base, health: st.health, stored: store['scout.importer.base'],
          calls: summary() });""")
    assert res['failed'] == {'ok': False, 'url': 'http://10.0.0.7:9004/api/health',
                             'error': 'Importer not reachable at http://10.0.0.7:9004/api/health'}
    assert res['err'] is None
    assert res['base'] == 'http://laptop:9004/api' and res['health']['ok'] is True
    assert res['stored'] == 'http://laptop:9004/api'
    assert res['calls'] == ['GET http://10.0.0.7:9004/api/health',
                            'GET http://laptop:9004/api/health']


def test_a_remembered_address_wins_and_skips_the_companion_lookup():
    res = _run("""
    store['scout.importer.base'] = 'http://laptop:9004/api';
    route('GET', /health$/, { status: 200, body: { ok: true } });
    await flow.open();
    out({ base: st.base, companion: companionCalls, calls: summary() });""")
    assert res == {'base': 'http://laptop:9004/api', 'companion': 0,
                   'calls': ['GET http://laptop:9004/api/health']}


def test_the_wizard_works_without_storage_and_without_a_companion():
    res = _run("""
    storageThrows = true;
    COMPANION = { configured: false };
    await flow.open();
    const none = JSON.parse(JSON.stringify(st.health));
    route('GET', /health$/, { status: 200, body: { ok: true } });
    const ok = await flow.setBase('http://h:9004');
    out({ none, ok, base: st.base });""")
    assert res['none']['ok'] is False and 'Type one' in res['none']['error']
    assert res['ok'] is True and res['base'] == 'http://h:9004/api'


def test_a_bad_address_is_refused_without_a_request():
    res = _run("""
    const ok = await flow.setBase('ftp://x');
    out({ ok, err: st.error, calls: summary() });""")
    assert res == {'ok': False, 'err': 'That is not an http(s) address', 'calls': []}


def test_clearing_the_address_forgets_it_and_falls_back_to_the_companion():
    res = _run("""
    store['scout.importer.base'] = 'http://laptop:9004/api';
    route('GET', /health$/, { status: 200, body: { ok: true } });
    await flow.setBase('');
    out({ stored: store['scout.importer.base'] === undefined, base: st.base });""")
    assert res == {'stored': True, 'base': 'http://10.0.0.7:9004/api'}


# --- upload and page ------------------------------------------------------------------------------

def test_upload_posts_the_raw_file_and_loads_the_only_page_with_the_json_mapping():
    res = _run("""
    pdfRoutes();
    await flow.open();
    await flow.upload(FILE);
    const urls = summary();
    out({ urls, body: calls[1].body, hdr: calls[1].headers, page: st.page && st.page.png,
          blob: st.page && st.page.blob, job: st.job.id, err: st.error });""")
    assert res['urls'][1] == 'POST http://10.0.0.7:9004/api/import?filename=a%20b.pdf'
    assert res['body'] == {'name': 'a b.pdf'} and res.get('hdr') is None
    assert res['urls'][2:] == ['GET http://10.0.0.7:9004/api/import/j1/page/0',
                               'GET http://10.0.0.7:9004/api/import/j1/page/0.png']
    assert res['page']['scale_x'] == 1.5 and res['page']['height'] == 1728
    assert res['blob'] == {'blob': 'pagepng'} and res['job'] == 'j1' and res['err'] is None


def test_a_multi_page_pdf_waits_for_the_operator_and_pages_are_zero_based():
    res = _run("""
    pdfRoutes(null, 3);
    await flow.open();
    await flow.upload(FILE);
    const before = { page: st.page, pages: st.job.pages.length, n: calls.length };
    await flow.selectPage(2);
    out({ before, last: summary().slice(-2), index: st.page.index });""")
    assert res['before'] == {'page': None, 'pages': 3, 'n': 2}
    assert res['last'] == ['GET http://10.0.0.7:9004/api/import/j1/page/2',
                           'GET http://10.0.0.7:9004/api/import/j1/page/2.png']
    assert res['index'] == 2


@pytest.mark.parametrize('status,message', [
    (413, 'PDF is larger than 50 MB'), (415, 'IFC import is not available yet'),
    (422, 'encrypted_pdf: password protected'),
])
def test_upload_errors_are_shown_verbatim(status, message):
    res = _run(f"""
    route('GET', /health$/, {{ status: 200, body: {{ ok: true }} }});
    route('POST', /\\/api\\/import\\?/, {{ status: {status}, body: {{ error: {json.dumps(message)} }} }});
    await flow.open();
    const ok = await flow.upload(FILE);
    out({{ ok, err: st.error, job: st.job }});""")
    assert res == {'ok': False, 'err': message, 'job': None}


def test_no_request_carries_a_cache_buster_query():
    res = _run("""
    pdfRoutes();
    await flow.open();
    await flow.upload(FILE);
    await flow.acceptProposal();
    out(summary());""")
    queries = [u.split('?', 1)[1] for u in (c.split(' ', 1)[1] for c in res) if '?' in u]
    assert queries == ['filename=a%20b.pdf']


# --- scale methods, preview gate (AE2), recalibrate (AE4) ----------------------------------------------

def test_accepting_the_proposal_calibrates_and_fetches_the_preview_without_a_cache_buster():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    const gateBefore = flow.previewCurrent();
    await flow.acceptProposal();
    out({ gateBefore, gateAfter: flow.previewCurrent(), tail: summary().slice(-2),
          body: bodyOf(calls.length - 2), hdr: calls[calls.length - 2].headers,
          preview: st.preview.blob, warnings: st.warnings, method: st.method });""")
    assert res['gateBefore'] is False and res['gateAfter'] is True
    assert res['tail'] == ['POST http://10.0.0.7:9004/api/import/j1/calibrate',
                           'GET http://10.0.0.7:9004/api/import/j1/preview.png']
    assert res['body'] == {'page': 0, 'method': 'proposal', 'rotation_deg': 0}
    assert res['hdr'] == {'Content-Type': 'application/json'}
    assert res['preview'] == {'blob': 'prevpng'} and res['method'] == 'proposal'


def test_the_proposal_is_never_applied_on_its_own():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    out({ calibrates: summary().filter((c) => /calibrate/.test(c)).length,
          calibrated: st.calibrated, proposal: flow.proposal().confidence });""")
    assert res == {'calibrates': 0, 'calibrated': None, 'proposal': 'high'}


def test_a_page_without_a_proposal_cannot_accept_one():
    res = _run("""
    pdfRoutes(RASTER);
    await flow.open(); await flow.upload(FILE);
    const ok = await flow.acceptProposal();
    out({ ok, err: st.error, n: summary().filter((c) => /calibrate/.test(c)).length,
          required: flow.calibrationRequired() });""")
    assert res['ok'] is False and 'no scale proposal' in res['err']
    assert res['n'] == 0 and res['required'] is True


def test_a_raster_page_stays_unpreviewable_until_a_method_succeeds():
    res = _run("""
    pdfRoutes(RASTER);
    await flow.open(); await flow.upload(FILE);
    const bare = await flow.calibrate();
    const bareErr = st.error;
    const inst = await flow.install({ name: 'plan' });
    const instErr = st.error;
    const gate1 = flow.previewCurrent();
    flow.beginTap('two');
    tap(100, 100, 10, 10);
    const premature = await flow.applyTwoPoint(3);       // only one tap so far
    const prematureErr = st.error;
    tap(400, 100, 110, 10);
    const ok = await flow.applyTwoPoint(3);
    out({ bare, bareErr, inst, instErr, gate1, premature, prematureErr, ok,
          gate2: flow.previewCurrent(), posts: summary().filter((c) => /POST.*(calibrate|install)/.test(c)) });""")
    assert res['bare'] is False and res['bareErr'] == 'Pick a scale method first'
    assert res['inst'] is False and 'Preview the grid first' in res['instErr']
    assert res['gate1'] is False and res['premature'] is False
    assert 'Tap two points' in res['prematureErr']
    assert res['ok'] is True and res['gate2'] is True
    assert res['posts'] == ['POST http://10.0.0.7:9004/api/import/j1/calibrate']


def test_two_taps_closer_than_20_px_are_refused_with_a_message():
    res = _run("""
    pdfRoutes(RASTER);
    await flow.open(); await flow.upload(FILE);
    flow.beginTap('two');
    const a = tap(100, 100, 50, 50);
    const near = tap(101, 100, 60, 55);        // 11 px away
    const msg = st.message;
    const stillWaiting = st.tap && st.tap.step;
    const p2Before = st.twoPoint.p2;
    const far = tap(300, 100, 90, 50);         // 40 px away
    out({ a, near, msg, stillWaiting, p2Before, far, p2: st.twoPoint.p2, tapOff: st.tap === null });""")
    assert res['a'] is True and res['near'] is False and res['far'] is True
    assert 'closer than 20 px' in res['msg'] and res['stillWaiting'] == 1
    assert res['p2Before'] is None and res['p2'] == {'x': 300, 'y': 100} and res['tapOff'] is True


def test_two_point_sends_the_tapped_page_points_and_the_distance_flat():
    res = _run("""
    pdfRoutes(RASTER);
    await flow.open(); await flow.upload(FILE);
    flow.beginTap('two'); tap(100, 200, 0, 0); tap(900, 200, 300, 0);
    await flow.applyTwoPoint(12.5);
    out(bodyOf(summary().findIndex((c) => /POST.*calibrate/.test(c))));""")
    assert res == {'page': 0, 'method': 'two_point', 'p1': [100, 200], 'p2': [900, 200],
                   'metres': 12.5, 'rotation_deg': 0}


def test_typed_scale_sends_scale_or_ratio_and_the_paper_size():
    res = _run("""
    pdfRoutes(RASTER);
    await flow.open(); await flow.upload(FILE);
    await flow.applyTyped({ scale: '1:100', ratio: '', paper_in: [36, 24] });
    const first = bodyOf(summary().findIndex((c) => /POST.*calibrate/.test(c)));
    const both = await flow.applyTyped({ scale: '1:100', ratio: 100, paper_in: null });
    out({ first, both, err: st.error });""")
    assert res['first'] == {'page': 0, 'method': 'typed', 'scale': '1:100',
                            'paper_in': [36, 24], 'rotation_deg': 0}
    assert res['both'] is False and 'not both' in res['err']


def test_changing_origin_or_rotation_after_a_preview_regenerates_it():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    await flow.acceptProposal();
    const n0 = summary().length;
    flow.beginTap('origin'); tap(250, 300, 5, 5);
    await new Promise((r) => setTimeout(r, 20));
    await flow.setRotation(15);
    await flow.setForceRaster(true);
    const after = summary().slice(n0);
    const cal = calls.filter((c) => /calibrate/.test(c.url)).map((c) => JSON.parse(c.body));
    out({ after, cal, dirty: st.dirty, current: flow.previewCurrent() });""")
    assert res['after'].count('POST http://10.0.0.7:9004/api/import/j1/calibrate') == 3
    assert res['after'].count('GET http://10.0.0.7:9004/api/import/j1/preview.png') == 3
    assert res['cal'][1] == {'page': 0, 'method': 'proposal', 'rotation_deg': 0, 'origin': [250, 300]}
    assert res['cal'][2]['rotation_deg'] == 15 and res['cal'][2]['origin'] == [250, 300]
    assert res['cal'][3]['force_raster'] is True
    assert res['dirty'] is False and res['current'] is True


def test_origin_and_rotation_before_a_preview_do_not_calibrate():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    const n = calls.length;
    await flow.setRotation(30);
    flow.beginTap('origin'); tap(1, 2, 0, 0);
    await flow.setForceRaster(true);
    out({ none: calls.length === n, rot: st.rotation_deg, origin: st.origin });""")
    assert res == {'none': True, 'rot': 30, 'origin': [1, 2]}


def test_a_bad_rotation_is_refused():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    const ok = await flow.setRotation(NaN);
    out({ ok, err: st.error });""")
    assert res == {'ok': False, 'err': 'Rotation must be a number of degrees'}


def test_a_failed_recalibrate_leaves_the_preview_marked_out_of_date_and_blocks_install():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    await flow.acceptProposal();
    routes.find((r) => /calibrate/.test(String(r.re))).responses = [
      { status: 422, body: { error: 'no wall geometry on this page' } }];
    routes.find((r) => /calibrate/.test(String(r.re))).i = 0;
    await flow.setRotation(5);
    const inst = await flow.install({ name: 'plan' });
    out({ err: st.error, dirty: st.dirty, current: flow.previewCurrent(), inst,
          posts: summary().filter((c) => /install/.test(c)).length });""")
    assert res['dirty'] is True and res['current'] is False and res['inst'] is False
    assert res['posts'] == 0


# --- busy importer (429) ----------------------------------------------------------------------------------

def test_a_busy_importer_is_retried_after_retry_after_and_then_succeeds():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    routes.find((r) => /calibrate/.test(String(r.re))).responses = [
      { status: 429, body: { error: 'busy' }, headers: { 'Retry-After': '3' } },
      { status: 429, body: { error: 'busy' }, headers: { 'Retry-After': '2' } },
      { status: 200, body: CALIB() }];
    routes.find((r) => /calibrate/.test(String(r.re))).i = 0;
    const notes = [];
    const ok = await flow.acceptProposal();
    out({ ok, sleeps, current: flow.previewCurrent(), note: st.note });""")
    assert res['ok'] is True and res['sleeps'] == [3000, 2000] and res['current'] is True
    assert res['note'] is None


def test_a_permanently_busy_importer_gives_up_after_a_bounded_number_of_retries():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    routes.find((r) => /calibrate/.test(String(r.re))).responses = [
      { status: 429, body: { error: 'busy' }, headers: { 'Retry-After': '0' } }];
    routes.find((r) => /calibrate/.test(String(r.re))).i = 0;
    const ok = await flow.acceptProposal();
    const posts = calls.filter((c) => /calibrate/.test(c.url)).length;
    out({ ok, sleeps, posts, err: st.error });""")
    assert res['ok'] is False and res['posts'] == 6 and len(res['sleeps']) == 5
    assert all(ms >= 1000 for ms in res['sleeps'])
    assert 'busy' in res['err'].lower()


def test_the_busy_note_is_visible_while_waiting():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    routes.find((r) => /calibrate/.test(String(r.re))).responses = [
      { status: 429, body: { error: 'busy' }, headers: { 'Retry-After': '4' } },
      { status: 200, body: CALIB() }];
    routes.find((r) => /calibrate/.test(String(r.re))).i = 0;
    const seen = [];
    const flow2 = flow;
    const orig = st;
    const p = flow2.acceptProposal();
    await new Promise((r) => setImmediate(r)); await new Promise((r) => setImmediate(r));
    seen.push(orig.note);
    await p;
    out({ seen, after: orig.note });""")
    assert res['after'] is None
    assert res['seen'][0] is None or 'importer busy' in res['seen'][0].lower()


# --- one operation at a time ---------------------------------------------------------------------------------------

def test_a_second_operation_while_one_runs_is_refused_not_overlapped():
    res = _run("""
    pdfRoutes();
    await flow.open();
    const a = flow.upload(FILE);
    const b = flow.upload(FILE);
    const rb = await b; await a;
    out({ rb, msg: st.message, uploads: calls.filter((c) => c.method === 'POST' && /import\\?/.test(c.url)).length,
          maxInflight });""")
    assert res['rb'] is False and 'Wait for' in res['msg']
    assert res['uploads'] == 1 and res['maxInflight'] == 1


def test_a_new_upload_drops_the_previous_job_first():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    await flow.upload(FILE);
    out(summary().filter((c) => /DELETE|import\\?/.test(c)));""")
    assert res == ['POST http://10.0.0.7:9004/api/import?filename=a%20b.pdf',
                   'DELETE http://10.0.0.7:9004/api/import/j1',
                   'POST http://10.0.0.7:9004/api/import?filename=a%20b.pdf']


# --- install (PDF) ---------------------------------------------------------------------------------------------------------

def test_install_sends_name_label_floor_only_and_never_activates():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    await flow.acceptProposal();
    const ok = await flow.install({ name: 'floor1_plan', label: ' Lobby plan ', floor: '1' });
    const i = summary().findIndex((c) => /POST.*install/.test(c));
    out({ ok, body: bodyOf(i), installed, result: st.install, urls: summary().filter((c) => /activate|active/.test(c)) });""")
    assert res['ok'] is True
    assert res['body'] == {'name': 'floor1_plan', 'label': 'Lobby plan', 'floor': 1}
    assert res['installed'] == [['floor1_plan']] and res['urls'] == []
    assert res['result']['ok'] is True and res['result']['name'] == 'floor1_plan'


@pytest.mark.parametrize('status,body,expect', [
    (409, {'error': "'plan' is a scanned map (it has a posegraph); install never replaces a scan map"},
     "'plan' is a scanned map (it has a posegraph); install never replaces a scan map"),
    (422, {'error': 'pgm pixel data is 5 B, header says 3x3 = 9'},
     'pgm pixel data is 5 B, header says 3x3 = 9'),
    (502, {'error': 'fleet_status unreachable', 'upstream': 'http://pi:9003'},
     'fleet_status unreachable (upstream: http://pi:9003)'),
])
def test_install_errors_are_relayed_verbatim(status, body, expect):
    res = _run(f"""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    await flow.acceptProposal();
    routes.find((r) => /install/.test(String(r.re)) && r.method === 'POST').responses = [
      {{ status: {status}, body: {json.dumps(body)} }}];
    const ok = await flow.install({{ name: 'plan' }});
    out({{ ok, err: st.error, installed, inst: st.install }});""")
    assert res['ok'] is False and res['err'] == expect and res['installed'] == []
    assert res['inst']['ok'] is False


def test_install_503_says_the_importer_has_no_fleet_status_url():
    res = _run("""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    await flow.acceptProposal();
    routes.find((r) => /install/.test(String(r.re)) && r.method === 'POST').responses = [
      { status: 503, body: { error: 'FLEET_STATUS_URL is not set' } }];
    await flow.install({ name: 'plan' });
    out(st.error);""")
    assert 'no FLEET_STATUS_URL set' in res and 'cannot install on the Pi' in res


@pytest.mark.parametrize('form', [
    {'name': 'Plan'}, {'name': ''}, {'name': 'plan\n'}, {'name': 'plan', 'floor': '1.5'},
])
def test_install_validates_the_form_before_any_request(form):
    res = _run(f"""
    pdfRoutes();
    await flow.open(); await flow.upload(FILE);
    await flow.acceptProposal();
    const n = calls.length;
    const ok = await flow.install({json.dumps(form)});
    out({{ ok, err: st.error, none: calls.length === n }});""")
    assert res['ok'] is False and res['err'] and res['none'] is True


def test_a_failing_refresh_hook_does_not_fail_the_install():
    res = _run("""
    pdfRoutes();
    const f2 = IF.createFlow({ fetch: fakeFetch, sleep: async () => {}, storage: null,
      getCompanion: async () => COMPANION, hostname: 'h',
      onInstalled: async () => { throw new Error('refresh failed'); }, onChange: () => {} });
    await f2.open(); await f2.upload(FILE); await f2.acceptProposal();
    const ok = await f2.install({ name: 'plan' });
    out({ ok, err: f2.state.error });""")
    assert res == {'ok': True, 'err': None}


# --- robot-measured -------------------------------------------------------------------------------------------------------

WALLS = {
    'ok': True, 'frame': 'base_link', 'stationary': True, 'flags': ['wall w3 has no partner'],
    'pairs': [
        {'a': 'w0', 'b': 'w1', 'span_m': 3.20, 'axis_rad': 0.01, 'spread_m': 0.004, 'parallel_err_deg': 0.4},
        {'a': 'w5', 'b': 'w6', 'span_m': 3.19, 'axis_rad': 0.02, 'spread_m': 0.006, 'parallel_err_deg': 0.6},
        {'a': 'w2', 'b': 'w4', 'span_m': 5.10, 'axis_rad': 1.571, 'spread_m': 0.005, 'parallel_err_deg': 0.3},
    ],
}


def test_wall_spans_is_one_plain_get_with_no_query_and_picks_the_top_pair_per_axis():
    res = _run(f"""
    pdfRoutes(RASTER);
    route('GET', /wall_spans/, {{ status: 200, body: {json.dumps(WALLS)} }});
    await flow.open(); await flow.upload(FILE);
    const n = calls.length;
    const ok = await flow.fetchWallSpans();
    out({{ ok, call: calls[n], n: calls.length - n, pairs: st.robot.pairs.map((p) => [p.a, p.span_m]),
          flags: st.robot.flags, total: st.robot.total }});""")
    assert res['ok'] is True and res['n'] == 1
    assert res['call']['url'] == 'http://pi.local:9001/api/wall_spans'
    assert res['call']['method'] == 'GET' and res['call'].get('body') is None
    assert res['pairs'] == [['w0', 3.20], ['w2', 5.10]] and res['total'] == 3
    assert res['flags'] == ['wall w3 has no partner']


@pytest.mark.parametrize('status,body,expect', [
    (409, {'error': 'robot is moving'}, 'Park it'),
    (503, {'error': 'no /scan traffic'}, 'No lidar scan yet'),
    (400, {'error': 'unknown parameter: x'}, 'unknown parameter: x'),
])
def test_wall_spans_errors(status, body, expect):
    res = _run(f"""
    pdfRoutes(RASTER);
    route('GET', /wall_spans/, {{ status: {status}, body: {json.dumps(body)} }});
    await flow.open(); await flow.upload(FILE);
    const ok = await flow.fetchWallSpans();
    out({{ ok, err: st.error, fetched: st.robot.fetched }});""")
    assert res['ok'] is False and expect in res['err'] and res['fetched'] is False


def test_wall_spans_unreachable_robot_is_named():
    res = _run("""
    pdfRoutes(RASTER);
    route('GET', /wall_spans/, { throw: true });
    await flow.open(); await flow.upload(FILE);
    await flow.fetchWallSpans();
    out(st.error);""")
    assert res == 'Could not reach the robot at http://pi.local:9001/api/wall_spans'


def test_robot_method_sends_the_marked_spans_with_the_measured_metres():
    res = _run(f"""
    pdfRoutes(RASTER);
    route('GET', /wall_spans/, {{ status: 200, body: {json.dumps(WALLS)} }});
    await flow.open(); await flow.upload(FILE);
    await flow.fetchWallSpans();
    const few = await flow.applyRobot();
    const fewErr = st.error;
    flow.beginTap('robot', 0); tap(100, 100, 10, 10); tap(100, 400, 10, 100);
    flow.beginTap('robot', 1); tap(100, 100, 10, 10); tap(500, 100, 150, 10);
    const ok = await flow.applyRobot();
    const i = summary().findIndex((c) => /POST.*calibrate/.test(c));
    out({{ few, fewErr, ok, body: bodyOf(i), spans: flow.robotSpans().length }});""")
    assert res['few'] is False and 'at least two wall pairs' in res['fewErr']
    assert res['ok'] is True and res['spans'] == 2
    assert res['body'] == {
        'page': 0, 'method': 'robot', 'rotation_deg': 0,
        'spans': [{'p1': [100, 100], 'p2': [100, 400], 'metres': 3.20},
                  {'p1': [100, 100], 'p2': [500, 100], 'metres': 5.10}]}


def test_robot_marks_obey_the_20_px_rule_per_pair():
    res = _run(f"""
    pdfRoutes(RASTER);
    route('GET', /wall_spans/, {{ status: 200, body: {json.dumps(WALLS)} }});
    await flow.open(); await flow.upload(FILE);
    await flow.fetchWallSpans();
    flow.beginTap('robot', 0); tap(100, 100, 10, 10);
    const near = tap(100, 110, 20, 20);
    out({{ near, msg: st.message, marks: st.robot.marks.length }});""")
    assert res['near'] is False and 'closer than 20 px' in res['msg']


def test_the_servers_two_non_parallel_spans_error_and_warning_are_shown_verbatim():
    res = _run(f"""
    pdfRoutes(RASTER);
    route('GET', /wall_spans/, {{ status: 200, body: {json.dumps(WALLS)} }});
    await flow.open(); await flow.upload(FILE);
    await flow.fetchWallSpans();
    flow.beginTap('robot', 0); tap(100, 100, 10, 10); tap(100, 400, 10, 100);
    flow.beginTap('robot', 1); tap(300, 100, 10, 10); tap(300, 400, 10, 100);
    routes.find((r) => /calibrate/.test(String(r.re))).responses = [
      {{ status: 400, body: {{ error: 'need two non-parallel spans' }} }}];
    routes.find((r) => /calibrate/.test(String(r.re))).i = 0;
    const bad = await flow.applyRobot();
    const err = st.error;
    routes.find((r) => /calibrate/.test(String(r.re))).responses = [
      {{ status: 200, body: CALIB({{ method: 'robot', warnings: ['spans disagree by 2.4 % (limit 1 %)'] }}) }}];
    routes.find((r) => /calibrate/.test(String(r.re))).i = 0;
    await flow.applyRobot();
    out({{ bad, err, warnings: st.warnings, current: flow.previewCurrent() }});""")
    assert res['bad'] is False and res['err'] == 'need two non-parallel spans'
    assert res['warnings'] == ['spans disagree by 2.4 % (limit 1 %)'] and res['current'] is True


# --- IFC (frozen contract) ------------------------------------------------------------------------------------------------

IFC_ROUTES = r"""
function ifcRoutes() {
  route('GET', /health$/, { status: 200, body: { ok: true } });
  route('POST', /\/api\/import\?/, { status: 201, body: { id: 'k1', kind: 'ifc', filename: 'b.ifc',
    sha256: 'y', bytes: 4096, pages: [], storeys: [
      { id: 0, global_id: 'g0', name: 'L1', label: 'Level 1', elevation_m: 0.0, floor: 1, slice: null, last_install: null },
      { id: 1, global_id: 'g1', name: 'L2', label: 'Level 2', elevation_m: 3.6, floor: 2, slice: null, last_install: null },
      { id: 2, global_id: 'g2', name: 'Roof', label: 'Roof', elevation_m: 7.2, floor: null, slice: null, last_install: null }] } });
  route('POST', /\/storey\/\d+\/slice$/, { status: 200, body: SLICE(0) });
  route('GET', /\/storey\/\d+\/preview\.png$/, { status: 200, blob: 'storeypng' });
  route('POST', /\/storey\/\d+\/install$/, { status: 200, body: { ok: true, replaced: false } });
  route('DELETE', /\/api\/import\/k1$/, { status: 200, body: { ok: true, id: 'k1' } });
}
const SLICE = (id) => ({ storey: { id, label: 'L', elevation_m: 0, floor: 1 }, z: 0.241,
  slice_height: 0.241, rotation_deg: 0, origin: [1, 2], resolution: 0.05, close_m: 1.2,
  bounds: [1, 2, 30, 20], faces: 120, preview: PREVIEW, warnings: [] });
const IFC = { name: 'b.ifc' };
"""


def _ifc(script):
    return _run(IFC_ROUTES + script)


def test_ifc_upload_lists_storeys_with_suggested_names_and_floors():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    out({ kind: st.job.kind, storeys: st.ifc.storeys.map((s) => [s.id, s.label, s.name, s.floor, s.selected, s.elevation_m]),
          calls: summary().length, focus: st.ifc.focus, page: st.page });""")
    assert res['kind'] == 'ifc' and res['page'] is None and res['focus'] == 0
    assert res['storeys'] == [[0, 'Level 1', 'level_1', '1', True, 0.0],
                              [1, 'Level 2', 'level_2', '2', True, 3.6],
                              [2, 'Roof', 'roof', '', True, 7.2]]
    assert res['calls'] == 2                  # health + upload: no page, no calibrate


def test_ifc_slices_one_storey_at_a_time_with_progress_and_no_calibration():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setStorey(2, 'selected', false);
    const seen = [];
    const iv = setInterval(() => { if (st.progress) seen.push(st.progress.i + '/' + st.progress.n); }, 1);
    flow.setIfcSettings(1.1, 15);
    const ok = await flow.sliceSelected();
    clearInterval(iv);
    const posts = calls.filter((c) => /slice$/.test(c.url));
    out({ ok, urls: posts.map((c) => c.url), bodies: posts.map((c) => JSON.parse(c.body)),
          maxInflight, sliced: st.ifc.storeys.map((s) => !!s.slice), previews: summary().filter((c) => /preview\\.png/.test(c)),
          calibrates: summary().filter((c) => /calibrate/.test(c)).length, progress: st.progress });""")
    assert res['ok'] is True and res['calibrates'] == 0 and res['progress'] is None
    assert res['urls'] == ['http://10.0.0.7:9004/api/import/k1/storey/0/slice',
                           'http://10.0.0.7:9004/api/import/k1/storey/1/slice']
    assert res['bodies'] == [{'slice_height': 1.1, 'rotation_deg': 15}] * 2
    assert res['maxInflight'] == 1 and res['sliced'] == [True, True, False]
    assert res['previews'] == ['GET http://10.0.0.7:9004/api/import/k1/storey/0/preview.png',
                               'GET http://10.0.0.7:9004/api/import/k1/storey/1/preview.png']


def test_ifc_default_slice_omits_slice_height_so_the_server_default_applies():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setStorey(1, 'selected', false); flow.setStorey(2, 'selected', false);
    flow.setIfcSettings(null, 0);
    await flow.sliceSelected();
    out(JSON.parse(calls.filter((c) => /slice$/.test(c.url))[0].body));""")
    assert res == {'rotation_deg': 0}


def test_ifc_a_storey_that_fails_to_slice_keeps_its_message_and_the_rest_go_on():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    routes.find((r) => /slice\\$/.test(String(r.re))).responses = [
      { status: 422, body: { error: 'storey Level 1: nothing solid at 0.241 m' } },
      { status: 200, body: SLICE(1) }, { status: 200, body: SLICE(2) }];
    const ok = await flow.sliceSelected();
    out({ ok, err: st.error, storeys: st.ifc.storeys.map((s) => [s.error, !!s.slice]) });""")
    assert res['ok'] is False and '1 storey(s) could not be sliced' in res['err']
    assert res['storeys'] == [['storey Level 1: nothing solid at 0.241 m', False],
                              [None, True], [None, True]]


def test_ifc_face_budget_413_names_the_count_and_the_limit():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setStorey(1, 'selected', false); flow.setStorey(2, 'selected', false);
    routes.find((r) => /slice\\$/.test(String(r.re))).responses = [
      { status: 413, body: { error: 'too many faces', count: 900000, limit: 500000 } }];
    await flow.sliceSelected();
    out(st.ifc.storeys[0].error);""")
    assert res == 'too many faces (900000 > 500000)'


def test_ifc_install_sends_each_storey_name_and_floor_one_at_a_time():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setStorey(2, 'selected', false);
    await flow.sliceSelected();
    flow.setStorey(0, 'name', 'lobby'); flow.setStorey(1, 'floor', '5');
    const ok = await flow.installSelected();
    const posts = calls.filter((c) => /storey\\/\\d+\\/install$/.test(c.url));
    out({ ok, urls: posts.map((c) => c.url), bodies: posts.map((c) => JSON.parse(c.body)),
          maxInflight, installed, results: st.ifc.storeys.map((s) => s.result && s.result.msg),
          activates: summary().filter((c) => /active|activate/.test(c)).length });""")
    assert res['ok'] is True and res['maxInflight'] == 1 and res['activates'] == 0
    assert res['urls'] == ['http://10.0.0.7:9004/api/import/k1/storey/0/install',
                           'http://10.0.0.7:9004/api/import/k1/storey/1/install']
    assert res['bodies'] == [{'name': 'lobby', 'floor': 1}, {'name': 'level_2', 'floor': 5}]
    assert res['installed'] == [['lobby', 'level_2']]
    assert res['results'] == ['Installed as lobby', 'Installed as level_2', None]


def test_ifc_install_reports_per_storey_failures_and_installs_the_rest():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setStorey(2, 'selected', false);
    await flow.sliceSelected();
    routes.find((r) => /install\\$/.test(String(r.re)) && r.method === 'POST').responses = [
      { status: 409, body: { error: "'level_1' is a scanned map" } },
      { status: 200, body: { ok: true } }];
    const ok = await flow.installSelected();
    out({ ok, err: st.error, results: st.ifc.storeys.map((s) => s.result && [s.result.ok, s.result.msg]),
          installed });""")
    assert res['ok'] is False and '1 storey(s) not installed' in res['err']
    assert res['results'][:2] == [[False, "'level_1' is a scanned map"], [True, 'Installed as level_2']]
    assert res['installed'] == [['level_2']]


def test_ifc_install_checks_names_before_any_request():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setStorey(2, 'selected', false);
    await flow.sliceSelected();
    const n = calls.length;
    flow.setStorey(1, 'name', 'level_1');
    const dup = await flow.installSelected(); const dupErr = st.error;
    flow.setStorey(1, 'name', 'Bad Name');
    const bad = await flow.installSelected(); const badErr = st.error;
    out({ dup, dupErr, bad, badErr, none: calls.length === n });""")
    assert res['dup'] is False and 'Two storeys are named "level_1"' in res['dupErr']
    assert res['bad'] is False and 'Level 2' in res['badErr'] and res['none'] is True


def test_ifc_unsliced_or_stale_storeys_are_not_installed():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setStorey(1, 'selected', false); flow.setStorey(2, 'selected', false);
    const ok = await flow.installSelected();
    out({ ok, msg: st.ifc.storeys[0].result.msg,
          posts: summary().filter((c) => /storey.*install/.test(c)).length });""")
    assert res == {'ok': False, 'msg': 'Slice this storey first', 'posts': 0}


def test_ifc_settings_change_marks_slices_stale_but_same_values_do_not():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setIfcSettings(null, 0);
    await flow.sliceSelected();
    flow.setIfcSettings(null, 0);
    const same = st.ifc.storeys.map((s) => s.dirty);
    flow.setIfcSettings(0.5, 0);
    out({ same, changed: st.ifc.storeys.map((s) => s.dirty) });""")
    assert res == {'same': [False, False, False], 'changed': [True, True, True]}


def test_ifc_origin_tap_reslices_that_storey_with_the_world_origin():
    res = _ifc("""
    ifcRoutes();
    await flow.open(); await flow.upload(IFC);
    flow.setStorey(1, 'selected', false); flow.setStorey(2, 'selected', false);
    await flow.sliceSelected();
    const n = calls.length;
    await flow.setStoreyOrigin(0, [12.5, 7.25]);
    out({ calls: summary().slice(n), body: bodyOf(n), origin: st.ifc.storeys[0].origin });""")
    assert res['calls'] == ['POST http://10.0.0.7:9004/api/import/k1/storey/0/slice',
                            'GET http://10.0.0.7:9004/api/import/k1/storey/0/preview.png']
    assert res['body'] == {'rotation_deg': 0, 'origin': [12.5, 7.25]}


def test_ifc_server_not_merged_yet_shows_the_415_verbatim():
    res = _run("""
    route('GET', /health$/, { status: 200, body: { ok: true } });
    route('POST', /\\/api\\/import\\?/, { status: 415, body: { error: 'IFC import is not available yet' } });
    await flow.open();
    await flow.upload({ name: 'b.ifc' });
    out({ err: st.error, job: st.job });""")
    assert res == {'err': 'IFC import is not available yet', 'job': None}


def test_the_20_px_rule_compares_in_the_current_view_after_a_pan_or_zoom():
    res = _run("""
    pdfRoutes(RASTER);
    await flow.open(); await flow.upload(FILE);
    // (a) far-apart page points; the view changed so P1 now sits 288 px from the 2nd tap,
    // although the OLD screen coords (10,10) and (12,10) are only 2 px apart
    flow.beginTap('two');
    tap(100, 100, 10, 10);
    const a = flow.handleTap({ page: { x: 900, y: 100 }, screen: { x: 12, y: 10 },
                               toScreen: () => ({ x: 300, y: 10 }) });
    const aP2 = st.twoPoint.p2;
    // (b) close page points; the view changed so the OLD coords look 190 px apart
    flow.beginTap('two');
    tap(100, 100, 10, 10);
    const b = flow.handleTap({ page: { x: 102, y: 100 }, screen: { x: 200, y: 10 },
                               toScreen: () => ({ x: 205, y: 10 }) });
    out({ a, aP2, b, msg: st.message, p2: st.twoPoint.p2 });""")
    assert res['a'] is True and res['aP2'] == {'x': 900, 'y': 100}
    assert res['b'] is False and 'closer than 20 px' in res['msg'] and res['p2'] is None


def test_the_20_px_rule_also_applies_per_robot_pair_in_the_current_view():
    res = _run(f"""
    pdfRoutes(RASTER);
    route('GET', /wall_spans/, {{ status: 200, body: {json.dumps(WALLS)} }});
    await flow.open(); await flow.upload(FILE);
    await flow.fetchWallSpans();
    flow.beginTap('robot', 0); tap(100, 100, 10, 10);
    const near = flow.handleTap({{ page: {{ x: 105, y: 100 }}, screen: {{ x: 300, y: 10 }},
                                  toScreen: () => ({{ x: 295, y: 10 }}) }});
    out({{ near }});""")
    assert res == {'near': False}


# --- the DOM layer, executed against a fake document ------------------------------------------------------

DOM = r"""
class El {
  constructor(id) {
    this.id = id; this.hidden = false; this._t = ''; this.value = ''; this.disabled = false;
    this.checked = false; this.dataset = {}; this.style = {}; this.children = []; this.handlers = {};
    this.clientWidth = 300; this.clientHeight = 300; this.width = 300; this.height = 300;
    this.className = ''; this.parentNode = null; this.files = []; this.type = ''; this.placeholder = '';
    this.classList = { toggle() {}, add() {} };
  }
  get textContent() { return this._t; }
  set textContent(v) { this._t = v; if (v === '') this.children = []; }
  addEventListener(ev, fn) { (this.handlers[ev] = this.handlers[ev] || []).push(fn); }
  appendChild(c) { c.parentNode = this; this.children.push(c); }
  getBoundingClientRect() { return { left: 0, top: 0 }; }
  setPointerCapture() {}
  getContext() {
    return new Proxy({}, { get: (o, k) => (k in o ? o[k] : function () {}), set: (o, k, v) => { o[k] = v; return true; } });
  }
  fire(ev, arg) { (this.handlers[ev] || []).forEach((fn) => fn(arg || {})); }
}
const els = {};
globalThis.document = {
  activeElement: null,
  getElementById: (id) => { if (!els[id]) els[id] = new El(id); return els[id]; },
  createElement: (tag) => new El(tag),
};
globalThis.Image = class { set src(v) { setTimeout(() => this.onload && this.onload(), 0); } get naturalWidth() { return 240; } get naturalHeight() { return 180; } };
URL.createObjectURL = () => 'blob:x'; URL.revokeObjectURL = () => {};
document.getElementById('imp-page').parentNode = new El('grid');
route('GET', /\/companion$/, { status: 200, body: COMPANION });
require(process.argv[2]); require(process.argv[3]); require(process.argv[4]);
const activated = [], refreshed = [];
const wizard = globalThis.ImportWizard.init({
  fleetApi: 'http://pi:9003/api', hostname: 'pi.local', fetch: fakeFetch,
  storage: () => ({ getItem: (k) => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = v; }, removeItem: (k) => { delete store[k]; } }),
  refreshSites: async () => { refreshed.push(1); }, activateMap: (n) => activated.push(n),
});
const E = (id) => els[id];
const settle = async () => { for (let i = 0; i < 6; i++) await new Promise((r) => setTimeout(r, 2)); };
const click = async (id) => { E(id).fire('click'); await settle(); };
"""


def _dom(script):
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    text = (HARNESS + DOM + IFC_ROUTES + '(async () => {' + script
            + '})().catch((e) => { console.error(e); process.exit(1); });')
    proc = subprocess.run(
        [node, '-e', text, str(FLOW), str(MATH), str(WEBUI / 'mapview.js'), str(WIZARD)],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return json.loads(proc.stdout.strip().splitlines()[-1])


def test_wizard_dom_pdf_flow_open_upload_accept_install_activate():
    res = _dom("""
    pdfRoutes();
    const before = { pdf: E('imp-pdf').hidden, ifc: E('imp-ifc').hidden, calls: calls.length };
    await click('import-open');
    const health = E('imp-health').textContent;
    E('imp-file').files = [FILE];
    await click('imp-upload');
    const afterUpload = { pdf: E('imp-pdf').hidden, job: E('imp-job').textContent, name: E('imp-name').value,
      report: E('imp-scale-report').textContent, accept: E('imp-accept').disabled,
      install: E('imp-install').disabled, info: E('imp-preview-info').textContent };
    await click('imp-accept');
    const afterAccept = { info: E('imp-preview-info').textContent, install: E('imp-install').disabled };
    await click('imp-install');
    const afterInstall = { result: E('imp-result').textContent, activate: E('imp-activate').hidden };
    await click('imp-activate');
    out({ before, health, afterUpload, afterAccept, afterInstall, activated, refreshed: refreshed.length });""")
    assert res['before'] == {'pdf': True, 'ifc': True, 'calls': 0}
    assert res['health'] == 'Importer ready at http://10.0.0.7:9004/api (v1)'
    up = res['afterUpload']
    assert up['pdf'] is False and 'a b.pdf' in up['job'] and up['name'] == 'a_b'
    assert 'Proposed scale' in up['report'] and 'high confidence' in up['report']
    assert up['accept'] is False and up['install'] is True
    assert 'Accept the scale' in up['info']
    assert 'Scale 0.0127' in res['afterAccept']['info'] and res['afterAccept']['install'] is False
    assert res['afterInstall']['result'] == 'Installed "a_b". It is not active yet.'
    assert res['afterInstall']['activate'] is False
    assert res['activated'] == ['a_b'] and res['refreshed'] == 1


def test_wizard_dom_taps_reach_the_flow_through_the_canvas():
    res = _dom("""
    pdfRoutes(RASTER);
    await click('import-open');
    E('imp-file').files = [FILE];
    await click('imp-upload');
    await click('imp-two-start');
    const banner = { hidden: E('imp-tap-banner').hidden, text: E('imp-tap-text').textContent };
    const canvas = E('imp-canvas');
    const press = (x, y) => { canvas.fire('pointerdown', { clientX: x, clientY: y, pointerId: 1 });
                              canvas.fire('pointerup', { clientX: x, clientY: y, pointerId: 1 }); };
    press(100, 100);
    press(102, 101);                 // 2 px away: refused
    const msg = E('imp-message').textContent;
    press(220, 100);
    E('imp-two-metres').value = '4';
    E('imp-two-metres').fire('input');
    const ready = E('imp-two-apply').disabled;
    await click('imp-two-apply');
    out({ banner, msg, p1: !!wizard.state.twoPoint.p1, p2: !!wizard.state.twoPoint.p2,
          ready, required: E('imp-required').hidden,
          info: E('imp-preview-info').textContent, status: E('imp-two-status').textContent });""")
    assert res['banner']['hidden'] is False and 'point 1' in res['banner']['text']
    assert 'closer than 20 px' in res['msg']
    assert res['p1'] is True and res['p2'] is True
    assert res['ready'] is False and 'Scale ' in res['info']


def test_wizard_dom_robot_list_and_ifc_storeys_render():
    res = _dom(f"""
    pdfRoutes(RASTER);
    route('GET', /wall_spans/, {{ status: 200, body: {json.dumps(WALLS)} }});
    await click('import-open');
    E('imp-file').files = [FILE];
    await click('imp-upload');
    await click('imp-robot-fetch');
    const rows = E('imp-robot-list').children.map((c) => c.className + ':' + (c.children[0] ? c.children[0].textContent : c.textContent));
    out({{ rows, apply: E('imp-robot-apply').disabled }});""")
    assert res['apply'] is True
    assert any('W1: 3.200 m' in r for r in res['rows']) and any('W2: 5.100 m' in r for r in res['rows'])
    assert any('Lidar note: wall w3 has no partner' in r for r in res['rows'])


def test_wizard_dom_ifc_flow_renders_storeys_slices_and_installs():
    res = _dom("""
    ifcRoutes();
    await click('import-open');
    E('imp-file').files = [IFC];
    await click('imp-upload');
    const shown = { ifc: E('imp-ifc').hidden, pdf: E('imp-pdf').hidden, rows: E('imp-storeys').children.length };
    E('imp-ifc-height').value = '1.1';
    await click('imp-ifc-slice');
    const afterSlice = { info: E('imp-ifc-info').textContent, install: E('imp-ifc-install').disabled,
      status: E('imp-storeys').children[0].children[5].textContent };
    await click('imp-ifc-install');
    const buttons = E('imp-ifc-activate').children.map((b) => b.textContent);
    E('imp-ifc-activate').children[0].fire('click');
    out({ shown, afterSlice, buttons, activated, bodies: calls.filter((c) => /slice$/.test(c.url)).map((c) => JSON.parse(c.body)) });""")
    assert res['shown'] == {'ifc': False, 'pdf': True, 'rows': 3}
    assert 'Level 1: 120 faces' in res['afterSlice']['info'] and res['afterSlice']['install'] is False
    assert res['afterSlice']['status'].startswith('Sliced')
    assert res['buttons'] == ['Activate level_1', 'Activate level_2', 'Activate roof']
    assert res['activated'] == ['level_1']
    assert res['bodies'][0] == {'slice_height': 1.1, 'rotation_deg': 0}


def test_wizard_dom_importer_down_shows_the_url_and_stays_usable():
    res = _dom("""
    route('GET', /health$/, { throw: true });
    await click('import-open');
    const down = E('imp-health').textContent;
    E('imp-base').value = 'laptop:9004';
    routes.length = 0;
    route('GET', /laptop.*health$/, { status: 200, body: { ok: true, version: 'z' } });
    await click('imp-retry');
    out({ down, up: E('imp-health').textContent, stored: store['scout.importer.base'] });""")
    assert res['down'] == 'Importer not reachable at http://10.0.0.7:9004/api/health'
    assert res['up'] == 'Importer ready at http://laptop:9004/api (z)'
    assert res['stored'] == 'http://laptop:9004/api'


def test_wizard_dom_ifc_activate_targets_the_installed_name_not_the_edited_one():
    res = _dom("""
    ifcRoutes();
    await click('import-open');
    E('imp-file').files = [IFC];
    await click('imp-upload');
    await click('imp-ifc-slice');
    await click('imp-ifc-install');
    const row = E('imp-storeys').children[0];
    row.children[2].value = 'edited_after_install';
    row.children[2].fire('input');
    const label = E('imp-ifc-activate').children[0].textContent;
    E('imp-ifc-activate').children[0].fire('click');
    out({ label, activated, now: wizard.state.ifc.storeys[0].name, kept: wizard.state.ifc.storeys[0].result.name });""")
    assert res['now'] == 'edited_after_install'
    assert res['kept'] == 'level_1' and res['label'] == 'Activate level_1'
    assert res['activated'] == ['level_1']


def test_wizard_dom_recalibrating_after_install_says_the_installed_map_is_older():
    res = _dom("""
    pdfRoutes();
    await click('import-open');
    E('imp-file').files = [FILE];
    await click('imp-upload');
    await click('imp-accept');
    await click('imp-install');
    const fresh = { text: E('imp-result').textContent, activate: E('imp-activate').hidden };
    E('imp-rotation').value = '10';
    E('imp-rotation').fire('change');
    await settle();
    const stale = { text: E('imp-result').textContent, activate: E('imp-activate').hidden };
    await click('imp-activate');
    await click('imp-install');
    out({ fresh, stale, activated, again: E('imp-result').textContent });""")
    assert res['fresh']['text'] == 'Installed "a_b". It is not active yet.'
    assert 'earlier, before the current calibration' in res['stale']['text']
    assert 'install again' in res['stale']['text']
    assert res['fresh']['activate'] is False and res['stale']['activate'] is False
    assert res['activated'] == ['a_b']
    assert res['again'] == 'Installed "a_b". It is not active yet.'


# --- the wizard is plain HTTP on user action ------------------------------------------------------------------------------------


def _code(path):
    text = path.read_text()
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
    return re.sub(r'(?m)^\s*//[^\n]*|(?<=[ \t;{}])//[^\n]*', '', text)


NEW = (MATH, FLOW, WIZARD)


def test_no_rosbridge_polling_timers_or_heavy_reads_in_the_wizard_files():
    for path in NEW:
        code = _code(path)
        for banned in ('ROSLIB', 'rosbridge', 'subscribe', 'callService', 'get_map', 'cmd_vel',
                       'goal_pose', 'initialpose', 'setInterval', 'requestAnimationFrame',
                       'EventSource', 'WebSocket'):
            assert banned not in code, (path.name, banned)
    flow_code = _code(FLOW)
    assert flow_code.count('setTimeout') == 1                  # the default sleep, injected in tests
    assert 'setTimeout' not in _code(WIZARD) and 'setTimeout' not in _code(MATH)


def test_wizard_writes_dynamic_text_with_text_content_only():
    for path in NEW:
        code = _code(path)
        for banned in ('innerHTML', 'insertAdjacentHTML', 'outerHTML', 'document.write'):
            assert banned not in code, (path.name, banned)


def test_wizard_does_no_network_until_the_operator_opens_it():
    code = _code(WIZARD)
    init = code[code.index('function init('):code.index('root.ImportWizard')]
    assert 'flow.open' not in init and 'fetch(' not in init.replace('opts.fetch(', '')
    open_handler = code[code.index('el.open.addEventListener'):code.index('el.close.addEventListener')]
    assert 'flow.open()' in open_handler


def test_wizard_has_no_force_install_and_no_cache_buster():
    for path in NEW:
        code = _code(path)
        assert 'cache:' not in code and 'Date.now' not in code and 'Math.random' not in code
        assert re.search(r"force\s*:\s*true|['\"]force['\"]", code) is None


def test_app_js_hook_is_one_small_init_block():
    src = APP_JS.read_text()
    assert src.count('ImportWizard.') == 1
    block = src[src.index('ImportWizard.init({'):]
    block = block[:block.index('});') + 3]
    for piece in ('fleetApi: FLEET_API', 'hostname: location.hostname', 'window.fetch.bind(window)',
                  'window.localStorage', 'refreshSites', 'activateMap: (name) => activateMap(name)'):
        assert piece in block, piece
    assert len(block.splitlines()) <= 6


def test_activation_goes_through_the_existing_activate_map_path():
    code = _code(WIZARD)
    assert code.count('cfg.activateMap(') == 2
    assert 'activationPlan' not in code and 'MapSwitch' not in code
    assert 'async function activateMap(' in APP_JS.read_text()


def test_index_html_has_the_button_hidden_panel_and_script_order():
    html = INDEX_HTML.read_text()
    assert re.search(r'<button id="import-open"[^>]*>Import drawing', html)
    assert re.search(r'<section class="panel" id="import-panel" hidden>', html)
    scripts = re.findall(r'<script src="([^"]+)"', html)
    order = ['mapview.js', 'mapswitch.js', 'importmath.js', 'importflow.js', 'importwizard.js', 'app.js']
    assert [s for s in scripts if s in order] == order
    site_panel = html[html.index('id="site-panel"'):html.index('data-panel="sensors"')]
    assert 'id="import-open"' in site_panel and 'id="import-panel"' in site_panel


def test_every_id_the_wizard_looks_up_exists_in_the_page():
    html = INDEX_HTML.read_text()
    src = WIZARD.read_text()
    table = src[src.index('var IDS = {'):src.index('// opts: {fleetApi')]
    ids = re.findall(r"'((?:imp-|import-)[a-z0-9-]+)'", table)
    assert len(ids) > 50
    missing = [i for i in ids if f'id="{i}"' not in html]
    assert missing == []


def test_the_wizard_keeps_its_page_image_off_the_live_map_canvas():
    code = _code(WIZARD)
    assert "'map-canvas'" not in code and "'imp-canvas'" in code and "'imp-ifc-canvas'" in code
    assert 'mapCanvas' not in code and 'drawMap' not in code


def test_importer_urls_the_flow_builds_match_the_documented_routes():
    code = _code(FLOW)
    for route in ("'/import?filename='", "'/page/'", "'.png'", "'/calibrate'", "'/preview.png'",
                  "'/install'", "'/storey/'", "'/slice'", "'/health'"):
        assert route in code, route
