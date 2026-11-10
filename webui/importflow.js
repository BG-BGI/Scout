/* Scout WebUI — floorplan import flow (state machine + HTTP; no DOM, no ROS). Classic
 * script; exposes window.ImportFlow (and module.exports under Node for
 * scout/test/test_importflow.py, which injects a fake fetch).
 *
 * Plain HTTP on operator action only: the importer (docker/floorplan-importer/server.py,
 * its docstring is the contract) and, for the robot-measured scale, one GET of the Pi's
 * :9001/api/wall_spans. No rosbridge topic, service or timer lives here; nothing polls.
 * Importer calls run one at a time. A 429 is retried a bounded number of times after its
 * Retry-After and shows an "importer busy" note, never a tight loop.
 *
 * Preview is gated on a successful calibrate (AE2: a raster page has no proposal, so
 * nothing can be previewed until a scale method has worked). The proposal is never applied
 * on its own: acceptProposal is the operator's click. Changing the origin, rotation or
 * force-raster after a preview re-runs calibrate and re-fetches the preview (AE4: the grid
 * is regenerated, never overlaid); a new scale is applied with its apply button, which
 * does the same. Install needs a current preview and never activates the map.
 */
(function (root) {
  'use strict';

  var IM = root.ImportMath || (typeof require === 'function' ? require('./importmath.js') : null);
  var JSON_HEADERS = { 'Content-Type': 'application/json' };

  function createFlow(deps) {
    var fetchFn = deps.fetch;
    var sleep = deps.sleep || function (ms) {
      return new Promise(function (resolve) { setTimeout(resolve, ms); });
    };
    var state = freshState();
    var pending = false;

    function freshState() {
      return {
        base: null, health: null, busy: null, note: null, error: null, message: null,
        job: null, page: null, method: null,
        twoPoint: { p1: null, p2: null, s1: null, metres: null },
        typed: { scale: '', ratio: '', paper_in: null },
        robot: { fetched: false, pairs: [], flags: [], total: 0, marks: [] },
        rotation_deg: 0, origin: null, forceRaster: false,
        calibrated: null, preview: null, warnings: [], dirty: false,
        tap: null, install: null, ifc: null, progress: null,
      };
    }

    function notify() { if (deps.onChange) deps.onChange(state); }

    // --- storage (the importer address is remembered; the page works without it) ------------

    function readStored() {
      try { return deps.storage ? deps.storage().getItem(IM.STORE_KEY) : null; } catch (e) { return null; }
    }

    function writeStored(value) {
      try {
        var s = deps.storage ? deps.storage() : null;
        if (!s) return;
        if (value) s.setItem(IM.STORE_KEY, value); else s.removeItem(IM.STORE_KEY);
      } catch (e) { /* storage blocked: the address just is not remembered */ }
    }

    // --- HTTP -----------------------------------------------------------------------------------

    function apiError(status, body, url) {
      var e = new Error(status === 429 ? 'Importer busy — try again in a moment'
        : IM.describeError(status, body));
      e.status = status;
      e.body = body;
      e.url = url;
      return e;
    }

    async function http(method, url, opts) {
      var o = opts || {};
      var tries = 0;
      for (;;) {
        var res;
        try {
          res = await fetchFn(url, { method: method, body: o.body, headers: o.headers });
        } catch (e) {
          var err = new Error('Could not reach ' + url);
          err.status = 0;
          err.url = url;
          err.unreachable = true;
          throw err;
        }
        if (res.status === 429 && tries < IM.RETRY_MAX) {
          tries += 1;
          var ms = IM.retryDelayMs(res.headers && res.headers.get ? res.headers.get('Retry-After') : null);
          state.note = 'Importer busy — retrying in ' + Math.round(ms / 1000) + ' s ('
            + tries + '/' + IM.RETRY_MAX + ')';
          notify();
          await sleep(ms);
          continue;
        }
        state.note = null;
        if (res.ok) {
          if (o.blob) return { status: res.status, blob: await res.blob() };
          var json = null;
          try { json = await res.json(); } catch (e) { json = null; }
          return { status: res.status, body: json };
        }
        var body = null;
        try { body = await res.json(); } catch (e) { body = null; }
        throw apiError(res.status, body, url);
      }
    }

    function jobUrl(suffix) { return state.base + '/import/' + state.job.id + suffix; }

    // One importer operation at a time; its error lands in state.error.
    async function run(label, fn) {
      if (state.busy) {
        state.message = 'Wait for "' + state.busy + '" to finish';
        notify();
        return false;
      }
      state.busy = label;
      state.error = null;
      state.message = null;
      notify();
      var ok = true;
      try {
        await fn();
      } catch (e) {
        state.error = e && e.message ? e.message : String(e);
        ok = false;
      } finally {
        state.busy = null;
        state.note = null;
        state.progress = null;
        notify();
      }
      if (pending && !state.busy) {
        pending = false;
        await calibrate();
      }
      return ok;
    }

    // --- importer address ------------------------------------------------------------------------

    async function resolveBaseUrl() {
      var stored = readStored();
      var companion = null;
      if (!IM.normalizeBase(stored) && deps.getCompanion) {
        try { companion = await deps.getCompanion(); } catch (e) { companion = null; }
      }
      state.base = IM.resolveBase(stored, companion);
      notify();
    }

    // Reachability is checked on panel open and on Retry only.
    function checkHealth() {
      if (!state.base) {
        state.health = { ok: false, url: null, error: 'No importer address yet. Type one above.' };
        notify();
        return Promise.resolve(false);
      }
      var url = state.base + '/health';
      return run('Checking importer', async function () {
        try {
          var r = await http('GET', url);
          var b = r.body || {};
          state.health = { ok: true, url: url, version: b.version || null,
                           fleet_status_url: b.fleet_status_url || null };
        } catch (e) {
          state.health = { ok: false, url: url,
                           error: e.unreachable ? 'Importer not reachable at ' + url : e.message };
        }
      });
    }

    async function open() {
      if (!state.base) await resolveBaseUrl();
      return checkHealth();
    }

    async function setBase(text) {
      if (!String(text || '').trim()) {
        writeStored(null);
        state.base = null;
        return open();
      }
      var b = IM.normalizeBase(text);
      if (!b) {
        state.error = 'That is not an http(s) address';
        notify();
        return false;
      }
      state.error = null;
      state.base = b;
      writeStored(b);
      notify();
      return checkHealth();
    }

    // --- job: upload / discard ------------------------------------------------------------------------

    async function dropJob() {
      var old = state.job;
      state.job = null;
      state.page = null;
      state.method = null;
      state.calibrated = null;
      state.preview = null;
      state.warnings = [];
      state.dirty = false;
      state.origin = null;
      state.twoPoint = { p1: null, p2: null, s1: null, metres: null };
      state.robot = { fetched: false, pairs: [], flags: [], total: 0, marks: [] };
      state.tap = null;
      state.install = null;
      state.ifc = null;
      if (old) {
        try { await http('DELETE', state.base + '/import/' + old.id); } catch (e) { /* best effort */ }
      }
    }

    function ifcStorey(s) {
      return {
        id: s.id, label: s.label || s.name || ('Storey ' + s.id), elevation_m: s.elevation_m,
        selected: true, name: IM.suggestName(s.label || s.name),
        floor: s.floor === null || s.floor === undefined ? '' : String(s.floor),
        slice: s.slice || null, previewBlob: null, origin: null, dirty: false,
        result: null, error: null,
      };
    }

    function setJob(json) {
      state.job = { id: json.id, kind: json.kind, filename: json.filename, bytes: json.bytes,
                    pages: json.pages || [] };
      if (json.kind === 'ifc') {
        state.ifc = { storeys: (json.storeys || []).map(ifcStorey), slice_height: null,
                      rotation_deg: 0, focus: null };
        if (state.ifc.storeys.length) state.ifc.focus = state.ifc.storeys[0].id;
      }
    }

    function upload(file) {
      if (!state.base) {
        state.error = 'No importer address yet. Type one above.';
        notify();
        return Promise.resolve(false);
      }
      return run('Uploading', async function () {
        await dropJob();
        var url = state.base + '/import?filename=' + encodeURIComponent(file.name);
        var r = await http('POST', url, { body: file });
        setJob(r.body);
        if (state.job.kind === 'pdf' && state.job.pages.length === 1) await loadPage(0);
      });
    }

    function discard() {
      return run('Discarding', dropJob);
    }

    // --- PDF: page, scale methods ---------------------------------------------------------------------------

    async function loadPage(n) {
      var info = await http('GET', jobUrl('/page/' + n));
      var img = await http('GET', jobUrl('/page/' + n + '.png'), { blob: true });
      state.page = { index: n, info: info.body.page, png: info.body.png,
                     scale: info.body.scale, blob: img.blob };
      state.method = null;
      state.calibrated = null;
      state.preview = null;
      state.warnings = [];
      state.dirty = false;
      state.origin = null;
      state.twoPoint = { p1: null, p2: null, s1: null, metres: null };
      state.robot.marks = [];
      state.tap = null;
      state.install = null;
    }

    function selectPage(n) {
      return run('Loading page', function () { return loadPage(n); });
    }

    function proposal() {
      var sc = state.page && state.page.scale;
      return sc && sc.proposal && sc.proposal.m_per_unit !== null
        && sc.proposal.m_per_unit !== undefined ? sc.proposal : null;
    }

    function calibrationRequired() {
      return !!(state.page && state.page.scale && state.page.scale.calibration_required);
    }

    function robotSpans() {
      var out = [];
      state.robot.pairs.forEach(function (p, i) {
        var m = state.robot.marks[i];
        if (m && m.p1 && m.p2) out.push({ p1: m.p1, p2: m.p2, metres: p.span_m });
      });
      return out;
    }

    function methodParams() {
      if (state.method === 'two_point') {
        return { p1: state.twoPoint.p1, p2: state.twoPoint.p2, metres: state.twoPoint.metres };
      }
      if (state.method === 'typed') return state.typed;
      if (state.method === 'robot') return { spans: robotSpans() };
      return {};
    }

    function buildCalibrate() {
      return IM.calibrateBody({
        page: state.page ? state.page.index : -1, method: state.method, params: methodParams(),
        rotation_deg: state.rotation_deg, origin: state.origin, force_raster: state.forceRaster,
      });
    }

    async function calibrateInner() {
      var built = buildCalibrate();
      if (built.error) throw new Error(built.error);
      state.dirty = true;
      var r = await http('POST', jobUrl('/calibrate'),
                         { body: JSON.stringify(built.body), headers: JSON_HEADERS });
      state.calibrated = r.body;
      state.warnings = (r.body && r.body.warnings) || [];
      var p = await http('GET', jobUrl('/preview.png'), { blob: true });
      state.preview = { blob: p.blob };
      state.dirty = false;
    }

    function calibrate() {
      if (!state.page || !state.method) {
        state.error = state.page ? 'Pick a scale method first' : 'Pick a page first';
        notify();
        return Promise.resolve(false);
      }
      return run('Calibrating', calibrateInner);
    }

    function useMethod(method) {
      state.method = method;
      return calibrate();
    }

    function acceptProposal() {
      if (!proposal()) {
        state.error = 'This page has no scale proposal. Use two-click, typed or robot-measured.';
        notify();
        return Promise.resolve(false);
      }
      return useMethod('proposal');
    }

    function applyTwoPoint(metres) {
      state.twoPoint.metres = metres;
      return useMethod('two_point');
    }

    function applyTyped(typed) {
      state.typed = typed;
      return useMethod('typed');
    }

    function applyRobot() {
      return useMethod('robot');
    }

    // Changing something the grid depends on after a preview regenerates it.
    function recalibrate() {
      if (!state.calibrated || !state.method) return Promise.resolve(false);
      state.dirty = true;
      notify();
      if (state.busy) {
        pending = true;
        return Promise.resolve(false);
      }
      return calibrate();
    }

    function setRotation(deg) {
      if (typeof deg !== 'number' || !isFinite(deg)) {
        state.error = 'Rotation must be a number of degrees';
        notify();
        return Promise.resolve(false);
      }
      state.rotation_deg = deg;
      notify();
      return recalibrate();
    }

    function setOrigin(pagePoint) {
      state.origin = [pagePoint.x, pagePoint.y];
      notify();
      return recalibrate();
    }

    function setForceRaster(on) {
      state.forceRaster = !!on;
      notify();
      return recalibrate();
    }

    // --- taps ---------------------------------------------------------------------------------------------

    // mode: 'two' | 'origin' | 'robot' (index = the wall pair being marked)
    function beginTap(mode, index) {
      state.tap = { mode: mode, step: 0, index: index === undefined ? 0 : index };
      state.message = null;
      notify();
    }

    function cancelTap() {
      state.tap = null;
      state.message = null;
      notify();
    }

    // t = {page: {x, y} (page frame), screen: {x, y} (canvas CSS px)}
    function handleTap(t) {
      var tap = state.tap;
      if (!tap) return false;
      if (tap.mode === 'origin') {
        state.tap = null;
        setOrigin(t.page);
        return true;
      }
      var rec = tap.mode === 'two' ? state.twoPoint : (state.robot.marks[tap.index] || {});
      if (tap.step === 0) {
        rec.p1 = t.page;
        rec.p2 = null;
        rec.s1 = t.screen;
        tap.step = 1;
        state.message = null;
      } else {
        if (IM.tooClose(rec.s1, t.screen)) {
          state.message = 'Those taps are closer than ' + IM.TAP_MIN_PX + ' px. Tap a point further away.';
          notify();
          return false;
        }
        rec.p2 = t.page;
        state.tap = null;
        state.message = null;
      }
      if (tap.mode === 'robot') state.robot.marks[tap.index] = rec;
      notify();
      return true;
    }

    // --- robot-measured spans (plain REST on the Pi; read-only, nothing is commanded) ----------------------

    function fetchWallSpans() {
      return run('Reading the lidar', async function () {
        var url = IM.wallSpansUrl(deps.hostname);
        var res;
        try { res = await fetchFn(url); } catch (e) { throw new Error('Could not reach the robot at ' + url); }
        var body = null;
        try { body = await res.json(); } catch (e) { body = null; }
        if (res.status === 409) throw new Error('The robot is moving. Park it and retry.');
        if (res.status === 503) throw new Error('No lidar scan yet (/scan is not publishing). Check the robot is up, then retry.');
        if (!res.ok) throw new Error(IM.describeError(res.status, body));
        var pairs = IM.topPairPerAxis(body);
        state.robot = { fetched: true, pairs: pairs, flags: (body && body.flags) || [],
                        total: ((body && body.pairs) || []).length,
                        stationary: body ? body.stationary : undefined, marks: [] };
      });
    }

    // --- install (PDF) ----------------------------------------------------------------------------------------

    function installMessage(e) {
      if (e.status === 503) {
        return 'The importer has no FLEET_STATUS_URL set, so it cannot install on the Pi. (' + e.message + ')';
      }
      return e.message;
    }

    async function notifyInstalled(names) {
      if (!deps.onInstalled || !names.length) return;
      try { await deps.onInstalled(names); } catch (e) { /* the list refreshes on the next poll */ }
    }

    function previewCurrent() { return !!state.calibrated && !state.dirty; }

    function install(form) {
      return run('Installing', async function () {
        var built = IM.installBody(form);
        if (built.error) throw new Error(built.error);
        if (!previewCurrent()) throw new Error('Preview the grid first, then install');
        try {
          var r = await http('POST', jobUrl('/install'),
                             { body: JSON.stringify(built.body), headers: JSON_HEADERS });
          state.install = { ok: true, name: built.body.name, response: r.body };
        } catch (e) {
          state.install = { ok: false, error: installMessage(e) };
          throw new Error(installMessage(e));
        }
        await notifyInstalled([built.body.name]);
      });
    }

    // --- IFC ------------------------------------------------------------------------------------------------------

    function storeyById(sid) {
      return state.ifc ? state.ifc.storeys.filter(function (s) { return s.id === sid; })[0] : null;
    }

    function setStorey(sid, field, value) {
      var st = storeyById(sid);
      if (!st) return;
      st[field] = value;
      notify();
    }

    function focusStorey(sid) {
      if (state.ifc) { state.ifc.focus = sid; notify(); }
    }

    function setIfcSettings(sliceHeight, rotationDeg) {
      if (!state.ifc) return;
      var changed = state.ifc.slice_height !== sliceHeight || state.ifc.rotation_deg !== rotationDeg;
      state.ifc.slice_height = sliceHeight;
      state.ifc.rotation_deg = rotationDeg;
      if (changed) state.ifc.storeys.forEach(function (s) { if (s.slice) s.dirty = true; });
      notify();
    }

    async function sliceInner(st) {
      var built = IM.sliceBody({ slice_height: state.ifc.slice_height === null ? undefined : state.ifc.slice_height,
                                 rotation_deg: state.ifc.rotation_deg, origin: st.origin });
      if (built.error) throw new Error(built.error);
      var base = jobUrl('/storey/' + st.id);
      var r = await http('POST', base + '/slice', { body: JSON.stringify(built.body), headers: JSON_HEADERS });
      var p = await http('GET', base + '/preview.png', { blob: true });
      st.slice = r.body;
      st.previewBlob = p.blob;
      st.dirty = false;
    }

    // One storey at a time, in list order. A storey that fails keeps its message; the rest go on.
    function sliceSelected() {
      return run('Slicing', async function () {
        var todo = state.ifc.storeys.filter(function (s) { return s.selected; });
        if (!todo.length) throw new Error('Tick at least one storey');
        var failed = 0;
        for (var i = 0; i < todo.length; i++) {
          state.progress = { i: i + 1, n: todo.length, label: todo[i].label };
          notify();
          todo[i].error = null;
          try { await sliceInner(todo[i]); } catch (e) { todo[i].error = e.message; failed += 1; }
        }
        if (failed) throw new Error(failed + ' storey(s) could not be sliced. See the list.');
      });
    }

    // The operator tapped the storey preview: that world point becomes map (0, 0).
    function setStoreyOrigin(sid, worldXY) {
      var st = storeyById(sid);
      if (!st) return Promise.resolve(false);
      st.origin = [worldXY[0], worldXY[1]];
      return run('Slicing', function () {
        state.progress = { i: 1, n: 1, label: st.label };
        return sliceInner(st);
      });
    }

    function checkIfcNames(todo) {
      var seen = {};
      for (var i = 0; i < todo.length; i++) {
        var built = IM.installBody({ name: todo[i].name, floor: todo[i].floor });
        if (built.error) return todo[i].label + ': ' + built.error;
        if (seen[todo[i].name]) return 'Two storeys are named "' + todo[i].name + '"';
        seen[todo[i].name] = true;
      }
      return null;
    }

    function installSelected() {
      return run('Installing', async function () {
        var todo = state.ifc.storeys.filter(function (s) { return s.selected; });
        if (!todo.length) throw new Error('Tick at least one storey');
        var bad = checkIfcNames(todo);
        if (bad) throw new Error(bad);
        var done = [];
        var failed = 0;
        for (var i = 0; i < todo.length; i++) {
          var st = todo[i];
          state.progress = { i: i + 1, n: todo.length, label: st.label };
          notify();
          st.result = null;
          if (!st.slice || st.dirty) {
            st.result = { ok: false, msg: 'Slice this storey first' };
            failed += 1;
            continue;
          }
          var built = IM.installBody({ name: st.name, floor: st.floor });
          try {
            var r = await http('POST', jobUrl('/storey/' + st.id + '/install'),
                               { body: JSON.stringify(built.body), headers: JSON_HEADERS });
            st.result = { ok: true, msg: 'Installed as ' + built.body.name, response: r.body };
            done.push(built.body.name);
          } catch (e) {
            st.result = { ok: false, msg: installMessage(e) };
            failed += 1;
          }
        }
        await notifyInstalled(done);
        if (failed) throw new Error(failed + ' storey(s) not installed. See the list.');
      });
    }

    return {
      state: state, open: open, setBase: setBase, checkHealth: checkHealth, upload: upload,
      discard: discard, selectPage: selectPage, proposal: proposal,
      calibrationRequired: calibrationRequired, acceptProposal: acceptProposal,
      applyTwoPoint: applyTwoPoint, applyTyped: applyTyped, applyRobot: applyRobot,
      fetchWallSpans: fetchWallSpans, setRotation: setRotation, setOrigin: setOrigin,
      setForceRaster: setForceRaster, beginTap: beginTap, cancelTap: cancelTap,
      handleTap: handleTap, install: install, previewCurrent: previewCurrent,
      calibrate: calibrate, buildCalibrate: buildCalibrate, robotSpans: robotSpans,
      setStorey: setStorey, focusStorey: focusStorey, setIfcSettings: setIfcSettings,
      sliceSelected: sliceSelected, setStoreyOrigin: setStoreyOrigin,
      installSelected: installSelected, storeyById: storeyById,
    };
  }

  var api = { createFlow: createFlow };
  root.ImportFlow = api;
  if (typeof module === 'object' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
