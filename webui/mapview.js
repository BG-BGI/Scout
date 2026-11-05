/* Scout WebUI — map view transform (pure math, no DOM). Classic script; exposes
 * window.MapView (and module.exports under Node for scout/test/test_mapview.py).
 *
 * A view is {k, cx, cy}: k = canvas CSS px per metre, (cx, cy) = the WORLD point
 * (map frame, m) at the centre of the canvas. Defining it in world metres, not
 * cells, means the live /map growing (new origin/size) never moves the view, and a
 * resize needs no fix-up. size = {w, h} in CSS px. Canvas y is flipped (+y up).
 */
(function (root) {
  'use strict';

  var MIN_ZOOM = 0.5;   // x of the fit view
  var MAX_ZOOM = 16;

  function boundsOf(info) {
    var x0 = info.origin.position.x, y0 = info.origin.position.y;
    return {
      x0: x0, y0: y0,
      x1: x0 + info.width * info.resolution,
      y1: y0 + info.height * info.resolution,
    };
  }

  // k that fits the whole bounds inside size (the "100%" reference).
  function fitK(b, size) {
    var bw = Math.max(b.x1 - b.x0, 1e-9), bh = Math.max(b.y1 - b.y0, 1e-9);
    return Math.min(size.w / bw, size.h / bh);
  }

  function fit(b, size) {
    return { k: fitK(b, size), cx: (b.x0 + b.x1) / 2, cy: (b.y0 + b.y1) / 2 };
  }

  function clampK(k, kfit) {
    return Math.min(Math.max(k, kfit * MIN_ZOOM), kfit * MAX_ZOOM);
  }

  function worldToCanvas(v, size, wx, wy) {
    return {
      x: size.w / 2 + v.k * (wx - v.cx),
      y: size.h / 2 - v.k * (wy - v.cy),
    };
  }

  function canvasToWorld(v, size, px, py) {
    return {
      x: v.cx + (px - size.w / 2) / v.k,
      y: v.cy - (py - size.h / 2) / v.k,
    };
  }

  // Zoom by `factor` keeping the world point under (px, py) fixed on screen.
  function zoomAt(v, size, px, py, factor, kfit) {
    var a = canvasToWorld(v, size, px, py);
    var k = clampK(v.k * factor, kfit);
    return { k: k, cx: a.x - (px - size.w / 2) / k, cy: a.y + (py - size.h / 2) / k };
  }

  // Drag by (dx, dy) canvas px: content follows the pointer.
  function panBy(v, dx, dy) {
    return { k: v.k, cx: v.cx - dx / v.k, cy: v.cy + dy / v.k };
  }

  function centerOn(v, wx, wy) {
    return { k: v.k, cx: wx, cy: wy };
  }

  // Two-finger gesture: the world point that was under startMid follows mid, and
  // the scale follows the finger spread. Computed from the gesture-start view so
  // it never accumulates error.
  function pinch(v0, size, startMid, startDist, mid, dist, kfit) {
    var a = canvasToWorld(v0, size, startMid.x, startMid.y);
    var k = clampK(v0.k * (dist / Math.max(startDist, 1e-9)), kfit);
    return { k: k, cx: a.x - (mid.x - size.w / 2) / k, cy: a.y + (mid.y - size.h / 2) / k };
  }

  // Keep at least `margin` px of the map on screen so it cannot be flung away.
  function constrain(v, size, b, margin) {
    var m = Math.min(margin, size.w / 2, size.h / 2);
    var dx = (size.w / 2 - m) / v.k, dy = (size.h / 2 - m) / v.k;
    return {
      k: v.k,
      cx: Math.min(Math.max(v.cx, b.x0 - dx), b.x1 + dx),
      cy: Math.min(Math.max(v.cy, b.y0 - dy), b.y1 + dy),
    };
  }

  function zoomPercent(k, kfit) {
    return Math.round(100 * k / kfit);
  }

  // A different map (vs the same map growing): old bounds mostly outside new ones.
  function drasticChange(oldB, newB) {
    if (!oldB) return true;
    var iw = Math.min(oldB.x1, newB.x1) - Math.max(oldB.x0, newB.x0);
    var ih = Math.min(oldB.y1, newB.y1) - Math.max(oldB.y0, newB.y0);
    if (iw <= 0 || ih <= 0) return true;
    var oldA = (oldB.x1 - oldB.x0) * (oldB.y1 - oldB.y0);
    var newA = (newB.x1 - newB.x0) * (newB.y1 - newB.y0);
    if (newA < 0.25 * oldA) return true;   // shrank: a different (smaller) map
    return iw * ih < 0.5 * Math.min(oldA, newA);
  }

  var api = {
    MIN_ZOOM: MIN_ZOOM, MAX_ZOOM: MAX_ZOOM,
    boundsOf: boundsOf, fitK: fitK, fit: fit, clampK: clampK,
    worldToCanvas: worldToCanvas, canvasToWorld: canvasToWorld,
    zoomAt: zoomAt, panBy: panBy, centerOn: centerOn, pinch: pinch,
    constrain: constrain, zoomPercent: zoomPercent, drasticChange: drasticChange,
  };
  root.MapView = api;
  if (typeof module === 'object' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
