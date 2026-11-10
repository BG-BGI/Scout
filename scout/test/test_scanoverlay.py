"""webui/scanoverlay.js (beam projection, fit, staleness) and webui/scanlayer.js (the
/scan subscription, banner and draw hook), exercised under Node with fakes, plus
source-level checks that app.js / index.html wire them in and never touch goals.

R10 / AE8: the overlay is an alignment hint, never localization proof, and never
blocks a goal. Skips when `node` is absent (Pi container) — CI / the dev Mac run it."""

import json
import math
import pathlib
import re
import shutil
import subprocess

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
WEBUI = REPO_ROOT / 'webui'
SCANOVERLAY = WEBUI / 'scanoverlay.js'
SCANLAYER = WEBUI / 'scanlayer.js'
APP_JS = WEBUI / 'app.js'
INDEX_HTML = WEBUI / 'index.html'
STYLE_CSS = WEBUI / 'style.css'
SCANMEASURE = REPO_ROOT / 'docker' / 'scout-skills' / 'scanmeasure.py'

PRELUDE = """
const SO = require(process.argv[1]);
const D = JSON.parse(process.argv[3]);
const num = (v) => (v === 'nan' ? NaN : v === 'inf' ? Infinity : v === '-inf' ? -Infinity : v);
const out = (v) => console.log(JSON.stringify(v));
"""


def _node(script, data=None):
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    proc = subprocess.run(
        [node, '-e', PRELUDE + script, str(SCANOVERLAY), str(SCANLAYER),
         json.dumps(data if data is not None else {})],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return json.loads(proc.stdout)


PI = math.pi
ORIGIN = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}


def _scan(ranges, angle_min=0.0, inc=0.1, lo=0.15, hi=12.0):
    return {'ranges': ranges, 'angle_min': angle_min, 'angle_increment': inc,
            'range_min': lo, 'range_max': hi}


def _project(scan, pose=ORIGIN, laser='default'):
    return _node(
        'D.scan.ranges = D.scan.ranges.map(num);'
        'out(SO.projectScan(D.scan, D.pose, D.laser === "default" ? undefined : D.laser));',
        {'scan': scan, 'pose': pose, 'laser': laser})


def _close(pt, x, y, tol=1e-9):
    return abs(pt['x'] - x) < tol and abs(pt['y'] - y) < tol


# --- the laser mount ------------------------------------------------------------

def test_laser_fallback_matches_scanmeasure():
    src = SCANMEASURE.read_text()
    assert re.search(r'LASER_FALLBACK = \(0\.073, 0\.0, math\.pi\)', src)
    fallback = _node('out(SO.LASER_FALLBACK);')
    assert fallback == {'x': 0.073, 'y': 0.0, 'yaw': PI}


def test_laser_fallback_is_named_and_commented_in_the_js():
    src = SCANOVERLAY.read_text()
    assert 'var LASER_FALLBACK = { x: 0.073, y: 0.0, yaw: Math.PI };' in src
    assert 'BACKWARDS' in src and 'CLAUDE.md' in src and 'scanmeasure.py' in src


# --- projection: the four axis cases, pi laser yaw and the 0.073 offset ----------

@pytest.mark.parametrize('angle,expect', [
    (0.0, (-1.0 + 0.073, 0.0)),        # laser 0 rad looks BACKWARDS from base_link
    (PI / 2, (0.073, -1.0)),           # laser +90 deg is the robot's RIGHT
    (PI, (1.0 + 0.073, 0.0)),          # laser 180 deg is straight AHEAD
    (-PI / 2, (0.073, 1.0)),           # laser -90 deg is the robot's LEFT
])
def test_axis_beams_at_the_origin(angle, expect):
    pts = _project(_scan([1.0], angle_min=angle))
    assert len(pts) == 1 and _close(pts[0], *expect)


@pytest.mark.parametrize('pose,angle,expect', [
    # robot at (2, 3) facing +y: a beam straight ahead (laser pi) lands 1.073 up
    ({'x': 2.0, 'y': 3.0, 'yaw': PI / 2}, PI, (2.0, 3.0 + 1.073)),
    # facing -x: laser 0 (backwards) lands behind = +x of the robot
    ({'x': 0.0, 'y': 0.0, 'yaw': PI}, 0.0, (1.0 - 0.073, 0.0)),
    # facing +y: laser +90 deg (robot right) lands on +x
    ({'x': 0.0, 'y': 0.0, 'yaw': PI / 2}, PI / 2, (1.0, 0.073)),
    # facing -y: laser -90 deg (robot left) lands on +x
    ({'x': 1.0, 'y': 1.0, 'yaw': -PI / 2}, -PI / 2, (2.0, 1.0 - 0.073)),
])
def test_axis_beams_with_a_rotated_translated_robot(pose, angle, expect):
    pts = _project(_scan([1.0], angle_min=angle), pose)
    assert len(pts) == 1 and _close(pts[0], *expect)


def test_a_zero_yaw_laser_would_mirror_the_overlay():
    # the failure this guards: forgetting the pi yaw puts "ahead" behind the robot
    pts = _project(_scan([1.0], angle_min=PI), laser={'x': 0.073, 'y': 0.0, 'yaw': 0.0})
    assert _close(pts[0], 0.073 - 1.0, 0.0)
    ahead = _project(_scan([1.0], angle_min=PI))
    assert ahead[0]['x'] > 1.0


def test_angle_increment_steps_through_the_beams():
    pts = _project(_scan([1.0, 1.0, 1.0], angle_min=PI, inc=-PI / 2))
    # laser pi, pi/2, 0: ahead, right, behind
    assert _close(pts[0], 1.073, 0.0)
    assert _close(pts[1], 0.073, -1.0)
    assert _close(pts[2], 0.073 - 1.0, 0.0)


# --- invalid beams ----------------------------------------------------------------

def test_invalid_beams_are_skipped():
    ranges = [None, 'nan', 'inf', '-inf', 0.0, -1.0, 0.1, 13.0, 'x', 1.0, 12.0, 0.15]
    pts = _project(_scan(ranges, inc=0.0, angle_min=PI))
    # only 1.0, 12.0 (== range_max) and 0.15 (== range_min) survive; all at angle pi
    xs = sorted(round(p['x'], 6) for p in pts)
    assert xs == [round(0.073 + 0.15, 6), round(0.073 + 1.0, 6), round(0.073 + 12.0, 6)]


def test_range_limits_default_to_unbounded_when_absent():
    scan = {'ranges': [50.0], 'angle_min': PI, 'angle_increment': 0.0}
    assert len(_project(scan)) == 1


@pytest.mark.parametrize('scan', [None, {}, {'ranges': []}])
def test_empty_or_missing_scan_projects_nothing(scan):
    assert _node('out(SO.projectScan(D.scan, D.pose));',
                 {'scan': scan, 'pose': ORIGIN}) == []


def test_missing_pose_projects_nothing():
    assert _node('out(SO.projectScan(D.scan, null));',
                 {'scan': _scan([1.0])}) == []


# --- decimation ----------------------------------------------------------------------

@pytest.mark.parametrize('n,expected', [
    (1, 1), (360, 360), (361, 181), (720, 360), (1000, 334), (1440, 360), (5000, 358),
])
def test_beams_are_decimated_to_at_most_360(n, expected):
    pts = _project(_scan([2.0] * n, angle_min=-PI, inc=2 * PI / n))
    assert len(pts) == expected <= 360


def test_decimation_keeps_the_scan_shape():
    n = 720
    pts = _project(_scan([2.0] * n, angle_min=-PI, inc=2 * PI / n))
    radii = {round(math.hypot(p['x'] - 0.073 * 0, p['y']), 1) for p in pts}
    assert len(pts) == 360 and radii.issubset({1.9, 2.0, 2.1})


# --- grid: row order, dilation --------------------------------------------------------

def _grid(w, h, res, ox, oy, occupied):
    data = [0] * (w * h)
    for (col, row) in occupied:
        data[row * w + col] = 100
    return {'info': {'width': w, 'height': h, 'resolution': res,
                     'origin': {'position': {'x': ox, 'y': oy}}}, 'data': data}


def _fit(grid, points, occ=None):
    return _node(
        'const m = SO.buildMask(D.grid, D.occ === null ? undefined : D.occ);'
        'out(SO.fit(D.points, m));',
        {'grid': grid, 'points': points, 'occ': occ})


def test_grid_row_zero_is_the_bottom_row():
    # one wall cell at col 55, row 2 of a 60x40 grid, origin (-1, -1), 5 cm cells
    grid = _grid(60, 40, 0.05, -1.0, -1.0, [(55, 2)])
    wall = {'x': -1.0 + 55.5 * 0.05, 'y': -1.0 + 2.5 * 0.05}
    mirror_rows = {'x': wall['x'], 'y': -1.0 + (40 - 2 - 0.5) * 0.05}   # row 37
    mirror_cols = {'x': -1.0 + 4.5 * 0.05, 'y': wall['y']}              # col 4
    assert _fit(grid, [wall]) == 1.0
    assert _fit(grid, [mirror_rows]) == 0.0
    assert _fit(grid, [mirror_cols]) == 0.0


def test_fit_tolerance_is_15_cm():
    grid = _grid(60, 40, 0.05, 0.0, 0.0, [(30, 20)])
    cx, cy = 30.5 * 0.05, 20.5 * 0.05
    assert _fit(grid, [{'x': cx, 'y': cy + 0.14}]) == 1.0
    assert _fit(grid, [{'x': cx, 'y': cy + 0.20}]) == 0.0
    assert _fit(grid, [{'x': cx + 0.14, 'y': cy}]) == 1.0
    assert _fit(grid, [{'x': cx - 0.20, 'y': cy}]) == 0.0


def test_fit_counts_points_off_the_grid_as_misses():
    grid = _grid(20, 20, 0.05, 0.0, 0.0, [(10, 10)])
    on = {'x': 10.5 * 0.05, 'y': 10.5 * 0.05}
    assert _fit(grid, [on, {'x': -5.0, 'y': -5.0}, {'x': 9.0, 'y': 9.0}]) == pytest.approx(1 / 3)


def test_fit_uses_the_occupied_threshold():
    grid = _grid(20, 20, 0.05, 0.0, 0.0, [])
    grid['data'][10 * 20 + 10] = 60
    pt = {'x': 10.5 * 0.05, 'y': 10.5 * 0.05}
    assert _fit(grid, [pt], occ=50) == 1.0
    assert _fit(grid, [pt], occ=65) == 0.0


def test_unknown_cells_are_not_walls():
    grid = _grid(20, 20, 0.05, 0.0, 0.0, [])
    grid['data'] = [-1] * 400
    assert _fit(grid, [{'x': 0.5, 'y': 0.5}]) == 0.0


@pytest.mark.parametrize('grid', [
    None, {}, {'info': {'width': 0, 'height': 5, 'resolution': 0.05,
                        'origin': {'position': {'x': 0, 'y': 0}}}, 'data': []},
    {'info': {'width': 4, 'height': 4, 'resolution': 0.05,
              'origin': {'position': {'x': 0, 'y': 0}}}, 'data': [0] * 3},
])
def test_malformed_grid_gives_no_mask(grid):
    assert _node('out(SO.buildMask(D.grid));', {'grid': grid}) is None


def test_no_points_or_no_mask_is_null_not_zero():
    assert _node('out(SO.fit([], SO.buildMask(D.grid)));',
                 {'grid': _grid(10, 10, 0.05, 0, 0, [(5, 5)])}) is None
    assert _node('out(SO.fit([{x: 1, y: 1}], null));') is None


# --- fit on a ray-cast room: aligned vs one room over -------------------------------

ROOMS = """
// walls: outer box x -4..8, y -3..3, plus an inner wall at x = 0 (two rooms, B wider)
const res = 0.05, ox = -5.0, oy = -4.0, w = 280, h = 160;
const data = new Array(w * h).fill(0);
const wallCell = (x, y) => {
  const c = Math.floor((x - ox) / res), r = Math.floor((y - oy) / res);
  if (c >= 0 && c < w && r >= 0 && r < h) data[r * w + c] = 100;
};
for (let x = -4; x <= 8; x += res / 2) { wallCell(x, -3); wallCell(x, 3); }
for (let y = -3; y <= 3; y += res / 2) { wallCell(-4, y); wallCell(8, y); wallCell(0, y); }
const grid = { info: { width: w, height: h, resolution: res,
  origin: { position: { x: ox, y: oy } } }, data };
// ray-cast the room [x0,x1] x [y0,y1] from the laser's pose in the map
function castScan(pose, room, n) {
  const L = SO.compose(pose, SO.LASER_FALLBACK);
  const ranges = [];
  const inc = 2 * Math.PI / n;
  for (let i = 0; i < n; i++) {
    const phi = L.yaw + (-Math.PI + i * inc);
    const dx = Math.cos(phi), dy = Math.sin(phi);
    let t = Infinity;
    if (dx > 1e-12) t = Math.min(t, (room[1] - L.x) / dx);
    if (dx < -1e-12) t = Math.min(t, (room[0] - L.x) / dx);
    if (dy > 1e-12) t = Math.min(t, (room[3] - L.y) / dy);
    if (dy < -1e-12) t = Math.min(t, (room[2] - L.y) / dy);
    ranges.push(t);
  }
  return { ranges, angle_min: -Math.PI, angle_increment: inc, range_min: 0.15, range_max: 30 };
}
const mask = SO.buildMask(grid, 50);
const roomA = [-4, 0, -3, 3];
const truth = { x: -2.0, y: 0.5, yaw: 0.3 };
const scan = castScan(truth, roomA, 720);
const fitAt = (pose) => SO.fit(SO.projectScan(scan, pose), mask);
"""


def test_fit_is_one_when_aligned_and_low_one_room_over():
    res = _node(ROOMS + """
    out({
      aligned: fitAt(truth),
      oneRoomOver: fitAt({ x: truth.x + 3.1, y: truth.y + 1.7, yaw: truth.yaw }),
      rotated: fitAt({ x: truth.x, y: truth.y, yaw: truth.yaw + 0.8 }),
      mirrored: fitAt({ x: truth.x, y: truth.y, yaw: truth.yaw + Math.PI }),
      halfMetreOff: fitAt({ x: truth.x + 0.5, y: truth.y, yaw: truth.yaw }),
    });""")
    assert res['aligned'] >= 0.99
    assert res['oneRoomOver'] < 0.30
    assert res['rotated'] < 0.30
    assert res['mirrored'] < 0.70
    assert res['halfMetreOff'] < 0.60


def test_a_wrong_laser_yaw_visibly_breaks_the_fit():
    res = _node(ROOMS + """
    const flipped = SO.projectScan(scan, truth, { x: 0.073, y: 0, yaw: 0 });
    out({ right: SO.fit(SO.projectScan(scan, truth), mask), wrong: SO.fit(flipped, mask) });""")
    assert res['right'] >= 0.99 and res['wrong'] < res['right']


def test_the_mask_is_built_once_not_per_beam():
    src = SCANOVERLAY.read_text()
    fit_body = src[src.index('function fit('):src.index('function formatFit(')]
    assert 'buildMask' not in fit_body and 'for (var' in fit_body
    assert 'new Uint8Array(w * h)' in src


# --- small pure helpers ----------------------------------------------------------------

@pytest.mark.parametrize('value,text', [(None, '–'), (0.0, '0%'), (0.456, '46%'), (1.0, '100%')])
def test_format_fit(value, text):
    assert _node('out(SO.formatFit(D.v));', {'v': value}) == text


@pytest.mark.parametrize('now,at,stale', [
    (5000, 3000, False), (5001, 3000, True), (5000, 5000, False), (5000, None, True),
    (5000, 4999, False),
])
def test_staleness_is_two_seconds(now, at, stale):
    assert _node('out(SO.isStale(D.now, D.at));', {'now': now, 'at': at}) is stale


def test_pose_from_message():
    yaw = 1.0
    msg = {'pose': {'pose': {'position': {'x': 1.5, 'y': -2.0, 'z': 0},
                             'orientation': {'x': 0, 'y': 0, 'z': math.sin(yaw / 2),
                                             'w': math.cos(yaw / 2)}}}}
    pose = _node('out(SO.poseFromMsg(D));', msg)
    assert pose['x'] == 1.5 and pose['y'] == -2.0 and pose['yaw'] == pytest.approx(yaw)
    assert _node('out(SO.poseFromMsg(null));') is None
    assert _node('out(SO.poseFromMsg({pose: {}}));') is None


def _meta(active='plan', mode='auto', resolved=None, name='office', **maps):
    meta = {'name': name, 'active_map': active, 'slam_mode': mode, 'maps': {
        n: {'source': s} for n, s in maps.items()}}
    if resolved is not None:
        meta['resolved_mode'] = resolved
    return meta


def _wanted(meta):
    return _node('out(SO.overlayWanted(D));', meta)


def test_overlay_wanted_only_for_a_drawing_under_localization():
    assert _wanted(_meta(resolved='localization', plan='drawing')) is True
    assert _wanted(_meta(mode='localization', plan='drawing')) is True
    assert _wanted(_meta(resolved='localization', scanned='scan', active='scanned')) is False
    assert _wanted(_meta(mode='localization', scanned='scan', active='scanned')) is False
    for mode in ('new', 'continue'):
        assert _wanted(_meta(mode=mode, resolved=mode, plan='drawing')) is False
    assert _wanted(_meta(resolved='new', plan='drawing')) is False


def test_overlay_wanted_falls_back_to_the_raw_mode_on_an_older_fleet_status():
    assert _wanted(_meta(plan='drawing')) is False          # auto, nothing resolved
    assert _wanted(_meta(mode='localization', plan='drawing')) is True


@pytest.mark.parametrize('meta', [None, {}, {'maps': {}}, {'active_map': None}])
def test_overlay_not_wanted_without_an_active_map(meta):
    assert _wanted(meta) is False


def test_map_key_and_switch_detection():
    assert _node('out(SO.mapKey(D));', _meta()) == 'office/plan'
    assert _node('out(SO.mapKey({active_map: null}));') is None
    assert _node('out(SO.dismissKey("a/b"));') == 'scout.scanoverlay.dismissed.a/b'
    assert _node('out([SO.isMapSwitch(null, "a/b"), SO.isMapSwitch("a/b", "a/b"),'
                 ' SO.isMapSwitch("a/b", "a/c")]);') == [False, False, True]


# --- scanlayer.js with fakes -----------------------------------------------------------

LAYER = """
const SL = require(process.argv[2]);
const topics = [];
class FakeTopic {
  constructor(o) { this.o = o; this.subscribed = false; this.unsubs = 0; topics.push(this); }
  subscribe(cb) { this.subscribed = true; this.cb = cb; }
  unsubscribe() { this.subscribed = false; this.unsubs++; }
}
let clock = 100000;
const timers = { fn: null, cleared: 0,
  setInterval(fn) { this.fn = fn; return 7; },
  clearInterval() { this.cleared++; this.fn = null; } };
const store = {};
let storageBroken = false;
const storage = () => {
  if (storageBroken) throw new Error('blocked');
  return { getItem: (k) => (k in store ? store[k] : null),
           setItem: (k, v) => { store[k] = v; }, removeItem: (k) => { delete store[k]; } };
};
let clicks = null;
const banner = { hidden: true, classList: { s: new Set(),
  toggle(c, on) { if (on) this.s.add(c); else this.s.delete(c); } } };
const fitEl = { textContent: '' };
const dismissBtn = { addEventListener: (ev, fn) => { clicks = fn; } };
let redraws = 0;
let poseReply = null;
let polls = 0;
const nextAmclPose = () => { polls++; return Promise.resolve(poseReply); };
let GRID = { info: { width: 10, height: 10, resolution: 0.05,
  origin: { position: { x: 0, y: 0 } } }, data: new Array(100).fill(0) };
SL.init({ ros: { id: 'ros' }, ROSLIB: { Topic: FakeTopic }, getGrid: () => GRID,
  occThreshold: 50, redraw: () => { redraws++; }, nextAmclPose, banner, fitEl,
  dismissBtn, storage, timers, now: () => clock });
const poseMsg = (p) => ({ pose: { pose: { position: { x: p.x, y: p.y, z: 0 },
  orientation: { x: 0, y: 0, z: Math.sin(p.yaw / 2), w: Math.cos(p.yaw / 2) } } } });
const mk = (active, mode, resolved) => ({ name: 'office', active_map: active,
  slam_mode: mode, resolved_mode: resolved, maps: { plan: { source: 'drawing' },
  plan2: { source: 'drawing' }, scanned: { source: 'scan' } } });
const DRAWING = mk('plan', 'auto', 'localization');
const DRAWING2 = mk('plan2', 'auto', 'localization');
const SCANMAP = mk('scanned', 'auto', 'continue');
const LOC_SCAN = mk('scanned', 'localization', 'localization');
const NOMODE = mk('plan', 'auto', 'new');
const ctxLog = [];
const ctx = { save() {}, restore() {}, set fillStyle(v) { ctxLog.push(['fill', v]); },
  fillRect(x, y) { ctxLog.push(['rect', x, y]); } };
const w2c = (x, y) => ({ x: x * 10, y: y * 10 });
const tick = async (ms) => {
  clock += ms; if (timers.fn) timers.fn();
  await Promise.resolve(); await Promise.resolve();
};
"""


def _layer(script, rooms=False):
    text = LAYER + '(async () => {' + (ROOMS if rooms else '') + script + '})();'
    return _node(text)


def test_layer_subscribes_to_scan_at_2hz_only_while_wanted():
    res = _layer("""
    SL.sync(DRAWING);
    SL.sync(DRAWING);
    const afterStart = topics.map((t) => ({ o: t.o, subscribed: t.subscribed }));
    SL.sync(NOMODE);
    const afterModeChange = topics.map((t) => t.subscribed);
    SL.sync(DRAWING);
    SL.sync(SCANMAP);
    out({ afterStart, afterModeChange, final: topics.map((t) => t.subscribed),
          n: topics.length, cleared: timers.cleared, banner: banner.hidden });""")
    assert res['n'] == 2
    start = res['afterStart']
    assert len(start) == 1 and start[0]['subscribed'] is True
    opts = start[0]['o']
    assert (opts['name'], opts['messageType']) == ('/scan', 'sensor_msgs/msg/LaserScan')
    assert (opts['throttle_rate'], opts['queue_length']) == (500, 1)
    assert res['afterModeChange'] == [False]
    assert res['final'] == [False, False]
    assert res['cleared'] == 2 and res['banner'] is True


def test_layer_never_subscribes_for_a_scan_map_or_without_a_site():
    res = _layer("""
    SL.sync(SCANMAP); SL.sync(LOC_SCAN); SL.sync(null); SL.sync({});
    out({ n: topics.length, banner: banner.hidden });""")
    assert res == {'n': 0, 'banner': True}


def test_layer_map_switch_resubscribes_and_resets():
    res = _layer("""
    SL.sync(DRAWING);
    SL.sync(DRAWING2);
    out({ subs: topics.map((t) => t.subscribed), snap: SL.snapshot() });""")
    assert res['subs'] == [False, True]
    assert res['snap']['key'] == 'office/plan2' and res['snap']['points'] == 0


def test_layer_drawn_points_are_live_then_grey_and_fit_is_a_dash_when_stale():
    res = _layer("""
    GRID = grid;
    poseReply = poseMsg(truth);
    SL.sync(DRAWING);
    await tick(0);
    topics[0].cb(scan);
    const live = SL.snapshot();
    const liveText = fitEl.textContent;
    SL.draw(ctx, w2c);
    const liveLog = ctxLog.splice(0);
    await tick(600);                 // scan older than 0.6 s: still live
    const stillLive = SL.snapshot().stale;
    await tick(2000);                // scan older than 2 s: stale (pose keeps answering)
    const stale = SL.snapshot();
    const staleText = fitEl.textContent;
    SL.draw(ctx, w2c);
    const rects = liveLog.filter((e) => e[0] === 'rect').length;
    out({ live, liveText, liveFill: liveLog[0], rects, stillLive, stale, staleText,
          staleFill: ctxLog[0], greyClass: banner.classList.s.has('stale') });
    """, rooms=True)
    assert res['live']['fit'] >= 0.99 and res['liveText'] == '100%'
    assert res['live']['points'] == res['rects'] > 100
    assert res['liveFill'] == ['fill', 'rgba(60, 235, 120, 0.9)']
    assert res['stillLive'] is False
    assert res['stale']['stale'] is True
    assert res['staleText'] == '–'
    assert res['staleFill'] == ['fill', 'rgba(150, 150, 150, 0.55)']
    assert res['greyClass'] is True


def test_layer_keeps_a_stationary_robots_pose_fresh_by_polling_the_latched_pose():
    # amcl publishes only after motion, so a pose that no message refreshes would
    # grey the layer on a robot standing still; the poll re-asks for the latched one
    res = _layer("""
    GRID = grid;
    poseReply = poseMsg(truth);
    SL.sync(DRAWING);
    await tick(0);
    const states = [];
    for (let i = 0; i < 12; i++) {       // 6 s of scans every 500 ms, no pose message
      topics[0].cb(scan);
      await tick(500);
      states.push(SL.snapshot().stale);
    }
    out({ states, polls, fit: SL.snapshot().fit });""", rooms=True)
    assert res['states'] == [False] * 12
    assert res['polls'] >= 4 and res['fit'] >= 0.99


def test_layer_goes_stale_when_amcl_stops_answering():
    res = _layer("""
    GRID = grid;
    poseReply = poseMsg(truth);
    SL.sync(DRAWING);
    await tick(0);
    poseReply = null;                    // amcl gone: no latched pose either
    const states = [];
    for (let i = 0; i < 10; i++) {
      topics[0].cb(scan);
      await tick(500);
      states.push(SL.snapshot().stale);
    }
    out({ states, text: fitEl.textContent });""", rooms=True)
    assert res['states'][0] is False and res['states'][-1] is True
    assert res['text'] == '–'


def test_layer_without_any_pose_shows_a_dash_not_a_number():
    res = _layer("""
    GRID = grid;
    SL.sync(DRAWING);
    await tick(0);
    topics[0].cb(scan);
    out({ snap: SL.snapshot(), text: fitEl.textContent });""", rooms=True)
    assert res['snap']['fit'] is None and res['snap']['points'] == 0
    assert res['text'] == '–' and res['snap']['stale'] is True


def test_layer_notepose_from_the_page_handler_feeds_the_fit():
    res = _layer("""
    GRID = grid;
    SL.sync(DRAWING);
    SL.notePose(truth);
    topics[0].cb(scan);
    out({ fit: SL.snapshot().fit, text: fitEl.textContent });""", rooms=True)
    assert res['fit'] >= 0.99 and res['text'] == '100%'


def test_layer_ignores_scans_and_poses_after_it_stopped():
    res = _layer("""
    GRID = grid;
    SL.sync(DRAWING);
    const t = topics[0];
    SL.sync(SCANMAP);
    t.cb(scan);
    SL.notePose(truth);
    out({ snap: SL.snapshot(), unsubs: t.unsubs });""", rooms=True)
    assert res['snap']['active'] is False and res['snap']['points'] == 0
    assert res['unsubs'] == 1


def test_banner_persists_until_dismissed_once_per_session_per_map():
    res = _layer("""
    SL.sync(DRAWING);
    const shown = banner.hidden === false;
    clicks();
    const hiddenAfter = banner.hidden;
    SL.sync(DRAWING);
    SL.sync(DRAWING);
    const stillHidden = banner.hidden;
    const stored = Object.keys(store);
    out({ shown, hiddenAfter, stillHidden, stored, active: SL.snapshot().active });""")
    assert res['shown'] is True and res['hiddenAfter'] is True
    assert res['stillHidden'] is True and res['active'] is True
    assert res['stored'] == ['scout.scanoverlay.dismissed.office/plan']


def test_banner_is_not_shown_again_in_the_same_session_after_a_page_reload():
    res = _layer("""
    store['scout.scanoverlay.dismissed.office/plan'] = '1';
    SL.sync(DRAWING);
    out({ hidden: banner.hidden, active: SL.snapshot().active });""")
    assert res == {'hidden': True, 'active': True}


def test_banner_reappears_after_a_map_switch():
    res = _layer("""
    SL.sync(DRAWING);
    clicks();                         // dismissed on plan
    SL.sync(DRAWING2);
    const onPlan2 = banner.hidden === false;
    clicks();                         // dismissed on plan2
    SL.sync(SCANMAP);                 // switch to a scan map and back
    SL.sync(DRAWING);
    const backOnPlan = banner.hidden === false;
    out({ onPlan2, backOnPlan });""")
    assert res == {'onPlan2': True, 'backOnPlan': True}


def test_banner_dismissal_survives_blocked_session_storage():
    res = _layer("""
    storageBroken = true;
    SL.sync(DRAWING);
    clicks();
    const hidden = banner.hidden;
    SL.sync(DRAWING);
    out({ hidden, stillHidden: banner.hidden });""")
    assert res == {'hidden': True, 'stillHidden': True}


def test_banner_stays_neutral_not_pass_fail():
    css = STYLE_CSS.read_text()
    block = css[css.index('.scan-banner {'):css.index('.scan-banner button')]
    for tint in ('--warn', 'red', 'green', '#E02840', '--ok', '--accent'):
        assert tint not in block, tint


# --- source-level wiring ------------------------------------------------------------------

def _code(path):
    text = path.read_text()
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


def test_new_modules_only_subscribe_and_never_touch_goals_or_heavy_reads():
    src = _code(SCANLAYER) + _code(SCANOVERLAY)
    for banned in ('get_map', 'callService', 'ROSLIB.Service', 'goal_pose', 'cmd_vel',
                   'Twist', 'navigate', '/initialpose', 'publish(', 'advertise'):
        assert banned not in src, banned
    for html in ('innerHTML', 'insertAdjacentHTML', 'outerHTML', 'document.write'):
        assert html not in src, html


def test_laser_to_base_transform_is_not_read_from_the_robot_by_the_page():
    # the page has no /tf_static subscription, so the named fallback is the transform
    assert '/tf_static' not in APP_JS.read_text()
    assert 'cfg.laser' in SCANLAYER.read_text()


def test_banner_text_and_caption_are_exact():
    html = INDEX_HTML.read_text()
    assert 'Confirm the scan sits on the walls before sending a goal' in html
    assert 'alignment hint, not localization proof' in html
    assert re.search(r'id="scan-banner"[^>]*\bhidden\b', html)
    assert 'id="scan-fit"' in html and 'id="scan-banner-dismiss"' in html


def test_index_html_loads_the_overlay_modules_before_app_js():
    scripts = re.findall(r'<script src="([^"]+)"', INDEX_HTML.read_text())
    assert scripts.index('mapswitch.js') < scripts.index('scanoverlay.js')
    assert scripts.index('scanoverlay.js') < scripts.index('scanlayer.js')
    assert scripts.index('scanlayer.js') < scripts.index('app.js')


def _fn(src, marker, end='\n}\n'):
    start = src.index(marker)
    return src[start:src.index(end, start)]


def test_app_js_hooks_are_small_and_in_the_named_places():
    src = APP_JS.read_text()
    assert src.count('ScanLayer.') == 4
    init = _fn(src, 'ScanLayer.init({', '});')
    for piece in ('ros', 'getGrid: () => liveMsg', 'occThreshold: () => OCC_THRESHOLD',
                  'redraw: drawMap', 'nextAmclPose', "getElementById('scan-banner')",
                  "getElementById('scan-fit')", "getElementById('scan-banner-dismiss')"):
        assert piece in init, piece
    assert 'ScanLayer.notePose(robotPose)' in _fn(src, 'function onPose(')
    assert 'ScanLayer.sync(activeSiteMeta)' in _fn(src, 'function renderSites(')
    draw = _fn(src, 'function drawMap(')
    assert 'ScanLayer.draw(mapCtx, worldToCanvas)' in draw
    assert draw.index('drawStructGrid()') < draw.index('ScanLayer.draw(')
    assert draw.index('ScanLayer.draw(') < draw.index('if (robotPose) {')


def test_app_js_goal_and_pose_publishing_code_does_not_mention_the_layer():
    src = APP_JS.read_text()
    for marker in ('function mapTap(', 'const goalPub', 'function publishTwist(',
                   "setPoseBtn.addEventListener('click'"):
        end = '\n}\n' if marker.startswith('function') else ');'
        assert 'ScanLayer' not in _fn(src, marker, end)


def test_app_js_has_one_scan_consumer_path_and_no_other_scan_subscription():
    src = APP_JS.read_text()
    assert "'/scan'" not in src and '"/scan"' not in src
    assert "name: '/scan'" in SCANLAYER.read_text()
