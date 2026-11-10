/* Scout WebUI — floorplan import maths (pure, no DOM, no network). Classic script;
 * exposes window.ImportMath (and module.exports under Node for
 * scout/test/test_importmath.py).
 *
 * Frames (docker/floorplan-importer/server.py "Conventions" is the contract):
 *  - PDF page frame: page units, origin bottom-left, y UP. Every point sent to the
 *    importer is in it. The page PNG is that page drawn top-down: pixel (px, py) has py
 *    counted from the TOP, with scale_x / scale_y pixels per page unit and `height` page
 *    units, so  page_x = px / scale_x,  page_y = height - py / scale_y.
 *  - Map frame: calib.map_frame(m_per_unit, rotation_deg, origin): the page point
 *    `origin` is map (0, 0) and map +X points rotation_deg counter-clockwise from page
 *    +x. map = s * Rot(-theta) * (page - origin). IFC uses the same frame with s = 1
 *    and "page" = IFC world metres.
 *  - Preview grid: `preview` = {origin [x, y, yaw], resolution, width, height} in map
 *    metres / cells; its PNG has one pixel per cell with row 0 at the TOP (map y max).
 * A matrix is canvas-style [a, b, c, d, e, f]: x' = a x + c y + e, y' = b x + d y + f.
 */
(function (root) {
  'use strict';

  var NAME_RE = /^[a-z0-9][a-z0-9_-]{0,31}$/;   // fleet_status / site-name regex
  var TAP_MIN_PX = 20;          // two taps closer than this on screen are refused
  var SAME_AXIS_DEG = 15;       // wall pairs within this of each other share an axis
  var DEFAULT_PORT = 9004;      // the importer on the companion
  var RETRY_MAX = 5;            // 429 retries per request
  var RETRY_MIN_S = 1;
  var RETRY_MAX_S = 10;
  var RETRY_DEFAULT_S = 2;
  var STORE_KEY = 'scout.importer.base';
  var DEFAULT_SLICE_HEIGHT = 0.241;   // m above the storey: the lidar plane

  function isNum(v) { return typeof v === 'number' && isFinite(v); }
  function isPoint(p) { return !!p && isNum(p.x) && isNum(p.y); }

  // --- names ---------------------------------------------------------------------

  // JS `$` (no m flag) only matches at the very end, so 'plan\n' is refused.
  function validName(name) { return typeof name === 'string' && NAME_RE.test(name); }

  // A starting name for a storey label: lowercase a-z0-9_-, starts alphanumeric,
  // at most 32 chars. The operator edits it; validName still decides.
  function suggestName(label) {
    var s = String(label === undefined || label === null ? '' : label).toLowerCase()
      .replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '').slice(0, 32).replace(/_+$/, '');
    return s || 'floor';
  }

  // --- taps ------------------------------------------------------------------------

  function tooClose(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y) < TAP_MIN_PX;
  }

  // --- PNG pixel <-> page frame --------------------------------------------------------

  function pxToPage(png, px, py) {
    return { x: px / png.scale_x, y: png.height - py / png.scale_y };
  }

  function pageToPx(png, x, y) {
    return { x: x * png.scale_x, y: (png.height - y) * png.scale_y };
  }

  // --- page <-> map ------------------------------------------------------------------------

  function mapFrame(mPerUnit, rotationDeg, origin) {
    var th = rotationDeg * Math.PI / 180;
    return { s: mPerUnit, c: Math.cos(th), sn: Math.sin(th), ox: origin[0], oy: origin[1] };
  }

  function pageToMap(f, p) {
    var dx = p.x - f.ox, dy = p.y - f.oy;
    return { x: f.s * (f.c * dx + f.sn * dy), y: f.s * (-f.sn * dx + f.c * dy) };
  }

  function mapToPage(f, m) {
    var x = m.x / f.s, y = m.y / f.s;
    return { x: f.ox + f.c * x - f.sn * y, y: f.oy + f.sn * x + f.c * y };
  }

  // --- matrices ----------------------------------------------------------------------------

  function applyMatrix(m, p) {
    return { x: m[0] * p.x + m[2] * p.y + m[4], y: m[1] * p.x + m[3] * p.y + m[5] };
  }

  // A o B: apply B first, then A.
  function multiplyMatrix(A, B) {
    return [
      A[0] * B[0] + A[2] * B[1], A[1] * B[0] + A[3] * B[1],
      A[0] * B[2] + A[2] * B[3], A[1] * B[2] + A[3] * B[3],
      A[0] * B[4] + A[2] * B[5] + A[4], A[1] * B[4] + A[3] * B[5] + A[5],
    ];
  }

  // The affine map f(u, v) -> {x, y} as a matrix (f must be affine).
  function matrixOf(f) {
    var p0 = f(0, 0), p1 = f(1, 0), p2 = f(0, 1);
    return [p1.x - p0.x, p1.y - p0.y, p2.x - p0.x, p2.y - p0.y, p0.x, p0.y];
  }

  // --- page canvas view (pan / zoom) -----------------------------------------------------------

  // view = {k, cx, cy} exactly as mapview.js: k canvas px per page unit, (cx, cy) the
  // page point at the canvas centre, y up. size = {w, h} canvas CSS px.
  function pageToCanvas(view, size, p) {
    return { x: size.w / 2 + view.k * (p.x - view.cx), y: size.h / 2 - view.k * (p.y - view.cy) };
  }

  function canvasToPage(view, size, x, y) {
    return { x: view.cx + (x - size.w / 2) / view.k, y: view.cy - (y - size.h / 2) / view.k };
  }

  // page PNG pixel -> canvas px.
  function pagePxToCanvasMatrix(png, view, size) {
    return [
      view.k / png.scale_x, 0, 0, view.k / png.scale_y,
      size.w / 2 - view.k * view.cx, size.h / 2 - view.k * (png.height - view.cy),
    ];
  }

  // --- preview grid on the page -------------------------------------------------------------------

  // preview image pixel (u, v), v from the TOP -> map metres.
  function previewPixelToMap(pv, u, v) {
    return { x: pv.origin[0] + u * pv.resolution, y: pv.origin[1] + (pv.height - v) * pv.resolution };
  }

  function mapToPreviewPixel(pv, m) {
    return { x: (m.x - pv.origin[0]) / pv.resolution, y: pv.height - (m.y - pv.origin[1]) / pv.resolution };
  }

  // The calibrate response `calib` ({m_per_unit, rotation_deg, origin, preview}) places the
  // preview on the page: preview pixel -> page frame.
  function previewPixelToPage(calib, u, v) {
    var f = mapFrame(calib.m_per_unit, calib.rotation_deg, calib.origin);
    return mapToPage(f, previewPixelToMap(calib.preview, u, v));
  }

  // preview image pixel -> page PNG pixel. imgW / imgH: the decoded preview image size when
  // it is not one pixel per cell (default: it is).
  function previewToPagePxMatrix(calib, png, imgW, imgH) {
    var pv = calib.preview;
    var sx = imgW ? pv.width / imgW : 1, sy = imgH ? pv.height / imgH : 1;
    return matrixOf(function (u, v) {
      var p = previewPixelToPage(calib, u * sx, v * sy);
      return pageToPx(png, p.x, p.y);
    });
  }

  // preview image pixel -> canvas px: what ctx.transform wants to draw the preview over the page.
  function previewToCanvasMatrix(calib, png, view, size, imgW, imgH) {
    return multiplyMatrix(pagePxToCanvasMatrix(png, view, size),
                          previewToPagePxMatrix(calib, png, imgW, imgH));
  }

  // --- IFC: a storey preview tap -> the IFC world origin ---------------------------------------------

  // `slice` = the slice response ({origin, rotation_deg, preview}). A tap at preview pixel
  // (u, v) names the world point that should become map (0, 0): preview pixel -> map metres
  // -> world metres through the slice's current origin / rotation.
  function ifcOriginFromTap(slice, u, v) {
    var m = previewPixelToMap(slice.preview, u, v);
    var w = mapToPage(mapFrame(1, slice.rotation_deg, slice.origin), m);
    return [w.x, w.y];
  }

  // Draw an image of size iw x ih contained in a canvas cw x ch: canvas = off + k * pixel.
  function fitContain(iw, ih, cw, ch) {
    var k = Math.min(cw / iw, ch / ih);
    return { k: k, ox: (cw - iw * k) / 2, oy: (ch - ih * k) / 2 };
  }

  function canvasToImagePx(fit, x, y) {
    return { x: (x - fit.ox) / fit.k, y: (y - fit.oy) / fit.k };
  }

  // --- robot-measured spans (GET :9001/api/wall_spans) ----------------------------------------------------

  function axisMod(a) {
    return ((a % Math.PI) + Math.PI) % Math.PI;
  }

  // Smallest angle between two undirected axes, rad in [0, pi/2].
  function axisDiff(a, b) {
    var d = Math.abs(axisMod(a) - axisMod(b));
    return Math.min(d, Math.PI - d);
  }

  // `pairs` is pre-sorted best first and a doorway splits one wall in two, so the same axis
  // shows near-duplicate pairs: keep the FIRST pair of each axis (axis_rad mod pi).
  function topPairPerAxis(resp) {
    var out = [];
    var pairs = (resp && resp.pairs) || [];
    for (var i = 0; i < pairs.length; i++) {
      var p = pairs[i];
      if (!p || !isNum(p.span_m) || p.span_m <= 0 || !isNum(p.axis_rad)) continue;
      var dup = out.some(function (q) {
        return axisDiff(q.axis_rad, p.axis_rad) < SAME_AXIS_DEG * Math.PI / 180;
      });
      if (!dup) out.push(p);
    }
    return out;
  }

  // --- request bodies ------------------------------------------------------------------------------------------

  function pt(p) { return [p.x, p.y]; }

  function originOf(o) {
    var v = o.origin;
    if (v && !Array.isArray(v) && isPoint(v)) return [v.x, v.y];
    return Array.isArray(v) && v.length === 2 && isNum(v[0]) && isNum(v[1]) ? [v[0], v[1]] : null;
  }

  function methodParams(method, params, body) {
    var p = params || {};
    if (method === 'proposal') return null;
    if (method === 'two_point') {
      if (!isPoint(p.p1) || !isPoint(p.p2)) return 'Tap two points on the page first';
      if (!isNum(p.metres) || p.metres <= 0) return 'Enter the real distance in metres (more than 0)';
      body.p1 = pt(p.p1);
      body.p2 = pt(p.p2);
      body.metres = p.metres;
      return null;
    }
    if (method === 'typed') {
      var hasScale = typeof p.scale === 'string' && p.scale.trim() !== '';
      var hasRatio = p.ratio !== undefined && p.ratio !== null && p.ratio !== '';
      if (hasScale === hasRatio) return 'Give a scale text or a ratio, not both';
      if (hasScale) body.scale = p.scale.trim();
      else if (isNum(p.ratio) && p.ratio > 0) body.ratio = p.ratio;
      else return 'The ratio must be a number more than 0';
      if (p.paper_in) {
        if (!Array.isArray(p.paper_in) || !isNum(p.paper_in[0]) || !isNum(p.paper_in[1])
            || p.paper_in[0] <= 0 || p.paper_in[1] <= 0) return 'Paper size needs width and height in inches';
        body.paper_in = [p.paper_in[0], p.paper_in[1]];
      }
      return null;
    }
    if (method === 'robot') {
      var spans = p.spans || [];
      if (spans.length < 2) return 'Mark at least two wall pairs on the page';
      body.spans = [];
      for (var i = 0; i < spans.length; i++) {
        var s = spans[i];
        if (!s || !isPoint(s.p1) || !isPoint(s.p2) || !isNum(s.metres) || s.metres <= 0) {
          return 'Each wall pair needs two taps and a measured span';
        }
        body.spans.push({ p1: pt(s.p1), p2: pt(s.p2), metres: s.metres });
      }
      return null;
    }
    return 'Unknown scale method';
  }

  // o: {page (0-based), method, params, rotation_deg, origin, resolution, close_m, force_raster}.
  // -> {body} or {error}. Flat, as the importer wants; origin / rotation only when set.
  function calibrateBody(o) {
    if (!Number.isInteger(o.page) || o.page < 0) return { error: 'Pick a page first' };
    var body = { page: o.page, method: o.method };
    var err = methodParams(o.method, o.params, body);
    if (err) return { error: err };
    if (o.rotation_deg !== undefined && o.rotation_deg !== null) {
      if (!isNum(o.rotation_deg)) return { error: 'Rotation must be a number of degrees' };
      body.rotation_deg = o.rotation_deg;
    }
    var origin = originOf(o);
    if (origin) body.origin = origin;
    if (isNum(o.resolution)) body.resolution = o.resolution;
    if (isNum(o.close_m)) body.close_m = o.close_m;
    if (o.force_raster === true) body.force_raster = true;
    return { body: body };
  }

  // o: {slice_height, rotation_deg, origin, resolution, close_m}; only given fields are sent.
  function sliceBody(o) {
    var body = {};
    if (o.slice_height !== undefined && o.slice_height !== null) {
      if (!isNum(o.slice_height) || o.slice_height <= 0) return { error: 'Slice height must be more than 0 m' };
      body.slice_height = o.slice_height;
    }
    if (o.rotation_deg !== undefined && o.rotation_deg !== null) {
      if (!isNum(o.rotation_deg)) return { error: 'Rotation must be a number of degrees' };
      body.rotation_deg = o.rotation_deg;
    }
    var origin = originOf(o);
    if (origin) body.origin = origin;
    if (isNum(o.resolution)) body.resolution = o.resolution;
    if (isNum(o.close_m)) body.close_m = o.close_m;
    return { body: body };
  }

  // o: {name, label, floor, map_start_pose} -> {body} or {error}. No force, ever.
  function installBody(o) {
    if (!validName(o.name)) {
      return { error: 'Name: a-z 0-9 _ - only, start with a letter or digit, up to 32 characters' };
    }
    var body = { name: o.name };
    var label = typeof o.label === 'string' ? o.label.trim() : '';
    if (label) body.label = label;
    if (o.floor !== undefined && o.floor !== null && o.floor !== '') {
      var f = typeof o.floor === 'string' ? Number(o.floor) : o.floor;
      if (!Number.isInteger(f)) return { error: 'Floor must be a whole number' };
      body.floor = f;
    }
    if (o.map_start_pose !== undefined && o.map_start_pose !== null) {
      var mp = o.map_start_pose;
      if (!Array.isArray(mp) || mp.length !== 3 || !mp.every(isNum)) {
        return { error: 'Start pose must be [x, y, theta]' };
      }
      body.map_start_pose = mp.slice();
    }
    return { body: body };
  }

  // --- importer address / retry -----------------------------------------------------------------------------------

  // Typed text -> a base URL ending in /api, or null when it is not an http(s) address.
  function normalizeBase(text) {
    var t = String(text === undefined || text === null ? '' : text).trim();
    if (!t) return null;
    if (/^[a-z][a-z0-9+.-]*:\/\//i.test(t) && !/^https?:\/\//i.test(t)) return null;
    if (!/^https?:\/\//i.test(t)) t = 'http://' + t;
    t = t.replace(/\/+$/, '');
    if (!/^https?:\/\/[^\s/?#@]+(\/[^\s?#]*)?$/i.test(t)) return null;
    if (!/\/api$/.test(t)) t += '/api';
    return t;
  }

  // A remembered address wins; else the companion fleet_status reports ({configured, host});
  // else null (the operator types one).
  function resolveBase(stored, companion) {
    var s = normalizeBase(stored);
    if (s) return s;
    if (companion && companion.configured && typeof companion.host === 'string' && companion.host) {
      return normalizeBase(companion.host + ':' + DEFAULT_PORT);
    }
    return null;
  }

  // Retry-After seconds header -> ms, clamped so a busy importer is never hammered nor waited on forever.
  function retryDelayMs(header) {
    var s = parseFloat(header);
    if (!isFinite(s) || s <= 0) s = RETRY_DEFAULT_S;
    return Math.min(Math.max(s, RETRY_MIN_S), RETRY_MAX_S) * 1000;
  }

  // The robot-measured spans endpoint: plain REST on the Pi, NO query string (unknown
  // parameters are a 400, so there is no cache-buster).
  function wallSpansUrl(hostname) {
    return 'http://' + hostname + ':9001/api/wall_spans';
  }

  // An error response -> the message the panel shows. The importer's / fleet_status's own text
  // is passed through unchanged; 502 adds the upstream URL it names.
  function describeError(status, body) {
    var msg = body && typeof body.error === 'string' && body.error ? body.error : 'HTTP ' + status;
    if (status === 502 && body && body.upstream) msg += ' (upstream: ' + body.upstream + ')';
    if (status === 413 && body && body.count !== undefined && body.limit !== undefined) {
      msg += ' (' + body.count + ' > ' + body.limit + ')';
    }
    return msg;
  }

  var api = {
    NAME_RE: NAME_RE, TAP_MIN_PX: TAP_MIN_PX, SAME_AXIS_DEG: SAME_AXIS_DEG,
    DEFAULT_PORT: DEFAULT_PORT, RETRY_MAX: RETRY_MAX, STORE_KEY: STORE_KEY,
    DEFAULT_SLICE_HEIGHT: DEFAULT_SLICE_HEIGHT,
    validName: validName, suggestName: suggestName, tooClose: tooClose,
    pxToPage: pxToPage, pageToPx: pageToPx,
    mapFrame: mapFrame, pageToMap: pageToMap, mapToPage: mapToPage,
    applyMatrix: applyMatrix, multiplyMatrix: multiplyMatrix, matrixOf: matrixOf,
    pageToCanvas: pageToCanvas, canvasToPage: canvasToPage,
    pagePxToCanvasMatrix: pagePxToCanvasMatrix,
    previewPixelToMap: previewPixelToMap, mapToPreviewPixel: mapToPreviewPixel,
    previewPixelToPage: previewPixelToPage, previewToPagePxMatrix: previewToPagePxMatrix,
    previewToCanvasMatrix: previewToCanvasMatrix, ifcOriginFromTap: ifcOriginFromTap,
    fitContain: fitContain, canvasToImagePx: canvasToImagePx,
    axisDiff: axisDiff, topPairPerAxis: topPairPerAxis,
    calibrateBody: calibrateBody, sliceBody: sliceBody, installBody: installBody,
    normalizeBase: normalizeBase, resolveBase: resolveBase, retryDelayMs: retryDelayMs,
    wallSpansUrl: wallSpansUrl, describeError: describeError,
  };
  root.ImportMath = api;
  if (typeof module === 'object' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
