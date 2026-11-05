# ruff: noqa: E501
"""webui/mapview.js pure transform math, exercised under Node.

Skips when `node` is absent (Pi container) — CI / the dev Mac run it.
"""

import pathlib
import shutil
import subprocess

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
MAPVIEW = REPO_ROOT / 'webui' / 'mapview.js'

SCRIPT = r"""
const MV = require(process.argv[1]);
const assert = require('assert');
let seed = 12345;
const rnd = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296; };
const rr = (a, b) => a + (b - a) * rnd();
const near = (a, b, tol, msg) => assert(Math.abs(a - b) <= tol, msg + ': ' + a + ' vs ' + b);
const randView = () => ({ k: rr(1, 400), cx: rr(-50, 50), cy: rr(-50, 50) });
const randSize = () => ({ w: rr(100, 2000), h: rr(100, 1500) });

for (let i = 0; i < 2000; i++) {   // round trips, both directions
  const v = randView(), s = randSize(), wx = rr(-80, 80), wy = rr(-80, 80);
  const c = MV.worldToCanvas(v, s, wx, wy), w = MV.canvasToWorld(v, s, c.x, c.y);
  near(w.x, wx, 1e-9, 'rt x'); near(w.y, wy, 1e-9, 'rt y');
  const px = rr(0, s.w), py = rr(0, s.h);
  const w2 = MV.canvasToWorld(v, s, px, py), c2 = MV.worldToCanvas(v, s, w2.x, w2.y);
  near(c2.x, px, 1e-6, 'rt px'); near(c2.y, py, 1e-6, 'rt py');
}

const info = { width: 200, height: 100, resolution: 0.05, origin: { position: { x: -3, y: 2 } } };
const b = MV.boundsOf(info);
assert.deepStrictEqual([b.x0, b.y0, b.x1, b.y1], [-3, 2, 7, 7]);

for (let i = 0; i < 200; i++) {   // fit: whole map inside, centred, touching one axis
  const s = randSize(), f = MV.fit(b, s);
  const lo = MV.worldToCanvas(f, s, b.x0, b.y0), hi = MV.worldToCanvas(f, s, b.x1, b.y1);
  assert(lo.x >= -1e-6 && hi.x <= s.w + 1e-6 && hi.y >= -1e-6 && lo.y <= s.h + 1e-6, 'fit inside');
  near((lo.x + hi.x) / 2, s.w / 2, 1e-6, 'fit centre x'); near((lo.y + hi.y) / 2, s.h / 2, 1e-6, 'fit centre y');
  near(Math.max(hi.x - lo.x - s.w, lo.y - hi.y - s.h), 0, 1e-6, 'fit touches an axis');
  near(f.k, MV.fitK(b, s), 1e-12, 'fitK');
}

for (let i = 0; i < 2000; i++) {   // zoom keeps the anchor fixed (unclamped range)
  const s = randSize(), kfit = MV.fitK(b, s), v = { k: kfit * rr(1, 4), cx: rr(-3, 7), cy: rr(2, 7) };
  const px = rr(0, s.w), py = rr(0, s.h), a = MV.canvasToWorld(v, s, px, py);
  const z = MV.zoomAt(v, s, px, py, rr(0.5, 2), kfit), c = MV.worldToCanvas(z, s, a.x, a.y);
  near(c.x, px, 1e-6, 'anchor x'); near(c.y, py, 1e-6, 'anchor y');
}

{   // clamp: 0.5x .. 16x of fit, even for absurd factors, anchor still fixed
  const s = { w: 800, h: 600 }, kfit = MV.fitK(b, s), v = MV.fit(b, s);
  near(MV.zoomAt(v, s, 10, 10, 1e6, kfit).k, kfit * 16, 1e-9, 'max clamp');
  near(MV.zoomAt(v, s, 10, 10, 1e-6, kfit).k, kfit * 0.5, 1e-9, 'min clamp');
  const z = MV.zoomAt(v, s, 123, 321, 1e6, kfit), a = MV.canvasToWorld(v, s, 123, 321);
  const c = MV.worldToCanvas(z, s, a.x, a.y);
  near(c.x, 123, 1e-6, 'clamped anchor x'); near(c.y, 321, 1e-6, 'clamped anchor y');
  assert.strictEqual(MV.zoomPercent(kfit, kfit), 100);
  assert.strictEqual(MV.zoomPercent(kfit * 4, kfit), 400);
}

for (let i = 0; i < 500; i++) {   // pan: world point under the pointer follows it
  const v = randView(), s = randSize(), px = rr(0, s.w), py = rr(0, s.h);
  const dx = rr(-300, 300), dy = rr(-300, 300), a = MV.canvasToWorld(v, s, px, py);
  const p = MV.panBy(v, dx, dy), c = MV.worldToCanvas(p, s, a.x, a.y);
  near(c.x, px + dx, 1e-6, 'pan x'); near(c.y, py + dy, 1e-6, 'pan y');
  assert.strictEqual(p.k, v.k);
}

for (let i = 0; i < 500; i++) {   // pinch: start midpoint's world point lands under the new midpoint
  const s = randSize(), kfit = MV.fitK(b, s), v = { k: kfit * rr(1, 3), cx: rr(-3, 7), cy: rr(2, 7) };
  const m0 = { x: rr(0, s.w), y: rr(0, s.h) }, m1 = { x: rr(0, s.w), y: rr(0, s.h) };
  const d0 = rr(30, 300), d1 = d0 * rr(0.7, 1.4), a = MV.canvasToWorld(v, s, m0.x, m0.y);
  const p = MV.pinch(v, s, m0, d0, m1, d1, kfit), c = MV.worldToCanvas(p, s, a.x, a.y);
  near(c.x, m1.x, 1e-6, 'pinch x'); near(c.y, m1.y, 1e-6, 'pinch y');
  near(p.k, MV.clampK(v.k * d1 / d0, kfit), 1e-9, 'pinch k');
}

{   // constrain: never loses the map entirely; identity when comfortably inside
  const s = { w: 800, h: 600 }, f = MV.fit(b, s);
  const same = MV.constrain(f, s, b, 48);
  near(same.cx, f.cx, 1e-12, 'constrain id'); near(same.cy, f.cy, 1e-12, 'constrain id y');
  const far = MV.constrain({ k: f.k, cx: 1e5, cy: -1e5 }, s, b, 48);
  const lo = MV.worldToCanvas(far, s, b.x0, b.y1);   // top-left corner of the map
  assert(lo.x <= s.w - 48 + 1e-6 && lo.y <= s.h - 48 + 1e-6, 'map still reachable');
}

{   // centerOn + drasticChange
  const s = { w: 640, h: 480 }, c = MV.worldToCanvas(MV.centerOn(MV.fit(b, s), 1.5, 4.5), s, 1.5, 4.5);
  near(c.x, 320, 1e-9, 'center x'); near(c.y, 240, 1e-9, 'center y');
  const grown = { x0: b.x0 - 1, y0: b.y0 - 1, x1: b.x1 + 2, y1: b.y1 + 2 };
  assert.strictEqual(MV.drasticChange(b, grown), false, 'growth keeps view');
  assert.strictEqual(MV.drasticChange(b, b), false);
  assert.strictEqual(MV.drasticChange(null, b), true);
  assert.strictEqual(MV.drasticChange(b, { x0: 100, y0: 100, x1: 110, y1: 110 }), true, 'disjoint');
  assert.strictEqual(MV.drasticChange(b, { x0: b.x0, y0: b.y0, x1: b.x0 + 1, y1: b.y0 + 1 }), true, 'shrunk');
}
console.log('ok');
"""


def test_mapview_math():
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    proc = subprocess.run(
        [node, '-e', SCRIPT, str(MAPVIEW)], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert proc.stdout.strip() == 'ok'
