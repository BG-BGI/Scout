/* Scout WebUI — floorplan import wizard (DOM + canvas around importflow.js). Classic
 * script; exposes window.ImportWizard. app.js owns one hook: ImportWizard.init({...}).
 *
 * The page image lives on this panel's OWN canvas (never the live map canvas), with
 * pan (drag), zoom (wheel, + / - / Fit) from mapview.js, which works in page units with
 * y up. All site / file / label text goes in through textContent. No rosbridge, no timer,
 * no polling: every request is the operator's click (see importflow.js).
 */
(function (root) {
  'use strict';

  var IM = root.ImportMath;
  var MV = root.MapView;
  var TAP_SLOP_PX = 6;
  var PREVIEW_ALPHA = 0.5;
  var STALE_ALPHA = 0.25;
  var ZOOM_STEP = 1.4;
  var MARK_COLORS = ['#e07020', '#2aa4d8', '#34c38f', '#c06ad0'];

  var cfg = null;
  var flow = null;
  var el = {};
  var view = null;                 // PDF page view {k, cx, cy}
  var imgs = { page: null, pageBlob: null, preview: null, previewBlob: null, ifc: null, ifcBlob: null };
  var drag = null;
  var ifcTapping = false;
  var lastJobId = null;
  var activateName = null;
  var storeySig = '';
  var lastPage = null;

  function $(id) { return document.getElementById(id); }

  function setText(node, text) { node.textContent = text; }

  function show(node, on) { node.hidden = !on; }

  function metresOf(v) {
    var n = parseFloat(v);
    return isFinite(n) ? n : null;
  }

  // --- images from blobs --------------------------------------------------------------------------

  function loadImage(slot, blobKey, blob, done) {
    if (!blob) { imgs[slot] = null; imgs[blobKey] = null; return; }
    if (imgs[blobKey] === blob) return;
    imgs[blobKey] = blob;
    var img = new Image();
    var url = URL.createObjectURL(blob);
    img.onload = function () {
      URL.revokeObjectURL(url);
      if (imgs[blobKey] === blob) { imgs[slot] = img; done(); }
    };
    img.onerror = function () { URL.revokeObjectURL(url); };
    img.src = url;
  }

  // --- the page canvas ---------------------------------------------------------------------------------

  function sizeCanvas(canvas) {
    var w = canvas.clientWidth || 300, h = canvas.clientHeight || 300;
    var dpr = root.devicePixelRatio || 1;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    return { w: w, h: h, dpr: dpr };
  }

  function pageBounds(png) { return { x0: 0, y0: 0, x1: png.width, y1: png.height }; }

  function fitView() {
    var st = flow.state;
    if (!st.page) { view = null; return; }
    var s = sizeCanvas(el.canvas);
    view = MV.fit(pageBounds(st.page.png), { w: s.w, h: s.h });
  }

  function marker(ctx, size, p, color, label) {
    var c = IM.pageToCanvas(view, size, p);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(c.x - 7, c.y); ctx.lineTo(c.x + 7, c.y);
    ctx.moveTo(c.x, c.y - 7); ctx.lineTo(c.x, c.y + 7);
    ctx.stroke();
    if (label) ctx.fillText(label, c.x + 8, c.y - 6);
  }

  function segment(ctx, size, a, b, color) {
    var ca = IM.pageToCanvas(view, size, a), cb = IM.pageToCanvas(view, size, b);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(ca.x, ca.y);
    ctx.lineTo(cb.x, cb.y);
    ctx.stroke();
  }

  function drawMarks(ctx, size) {
    var st = flow.state;
    ctx.font = '11px sans-serif';
    var tp = st.twoPoint;
    if (tp.p1) marker(ctx, size, tp.p1, '#e07020', 'P1');
    if (tp.p2) { marker(ctx, size, tp.p2, '#e07020', 'P2'); segment(ctx, size, tp.p1, tp.p2, '#e07020'); }
    st.robot.marks.forEach(function (m, i) {
      var color = MARK_COLORS[i % MARK_COLORS.length];
      if (m && m.p1) marker(ctx, size, m.p1, color, 'W' + (i + 1) + 'a');
      if (m && m.p2) { marker(ctx, size, m.p2, color, 'W' + (i + 1) + 'b'); segment(ctx, size, m.p1, m.p2, color); }
    });
    var origin = st.origin || (st.calibrated && st.calibrated.origin);
    if (origin) marker(ctx, size, { x: origin[0], y: origin[1] }, '#d02840', 'map 0,0');
  }

  function drawPage() {
    var st = flow.state;
    var s = sizeCanvas(el.canvas);
    var ctx = el.canvas.getContext('2d');
    ctx.setTransform(s.dpr, 0, 0, s.dpr, 0, 0);
    ctx.clearRect(0, 0, s.w, s.h);
    if (!st.page || !imgs.page) return;
    if (!view) fitView();
    var size = { w: s.w, h: s.h };
    var png = st.page.png;
    ctx.save();
    ctx.transform.apply(ctx, IM.pagePxToCanvasMatrix(png, view, size));
    ctx.drawImage(imgs.page, 0, 0, png.width_px, png.height_px);
    ctx.restore();
    if (st.calibrated && imgs.preview) {
      ctx.save();
      ctx.globalAlpha = st.dirty ? STALE_ALPHA : PREVIEW_ALPHA;
      ctx.imageSmoothingEnabled = false;
      ctx.transform.apply(ctx, IM.previewToCanvasMatrix(
        st.calibrated, png, view, size, imgs.preview.naturalWidth, imgs.preview.naturalHeight));
      ctx.drawImage(imgs.preview, 0, 0);
      ctx.restore();
    }
    drawMarks(ctx, size);
  }

  // --- the IFC preview canvas -----------------------------------------------------------------------------

  function ifcFocus() {
    var st = flow.state;
    return st.ifc ? flow.storeyById(st.ifc.focus) : null;
  }

  function ifcFit() {
    return IM.fitContain(imgs.ifc.naturalWidth, imgs.ifc.naturalHeight,
                         el.ifcCanvas.clientWidth || 300, el.ifcCanvas.clientHeight || 300);
  }

  function drawIfc() {
    var s = sizeCanvas(el.ifcCanvas);
    var ctx = el.ifcCanvas.getContext('2d');
    ctx.setTransform(s.dpr, 0, 0, s.dpr, 0, 0);
    ctx.clearRect(0, 0, s.w, s.h);
    var st = ifcFocus();
    if (!st || !st.slice || !imgs.ifc) return;
    var fit = IM.fitContain(imgs.ifc.naturalWidth, imgs.ifc.naturalHeight, s.w, s.h);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(imgs.ifc, fit.ox, fit.oy, imgs.ifc.naturalWidth * fit.k, imgs.ifc.naturalHeight * fit.k);
    var pv = st.slice.preview;
    var px = IM.mapToPreviewPixel(pv, { x: 0, y: 0 });
    var u = px.x * imgs.ifc.naturalWidth / pv.width, v = px.y * imgs.ifc.naturalHeight / pv.height;
    var cx = fit.ox + u * fit.k, cy = fit.oy + v * fit.k;
    ctx.strokeStyle = '#d02840';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx - 7, cy); ctx.lineTo(cx + 7, cy);
    ctx.moveTo(cx, cy - 7); ctx.lineTo(cx, cy + 7);
    ctx.stroke();
  }

  // --- render ---------------------------------------------------------------------------------------------------

  function renderStatus() {
    var st = flow.state;
    if (st.base !== null && document.activeElement !== el.base) el.base.value = st.base;
    var h = st.health;
    setText(el.health, !h ? '' : h.ok
      ? 'Importer ready at ' + st.base + (h.version ? ' (' + h.version + ')' : '')
      : h.error);
    show(el.busy, !!(st.busy || st.note));
    setText(el.busy, st.note || (st.busy ? st.busy + '…' : ''));
    if (st.progress) setText(el.busy, st.busy + ' ' + st.progress.i + ' of ' + st.progress.n + ': ' + st.progress.label + '…');
    show(el.error, !!st.error);
    setText(el.error, st.error || '');
    show(el.message, !!st.message);
    setText(el.message, st.message || '');
    el.upload.disabled = !!st.busy;
    el.retry.disabled = !!st.busy;
    var job = st.job;
    setText(el.job, job ? job.filename + ' (' + job.kind.toUpperCase() + ', ' + Math.round(job.bytes / 1024) + ' KB)' : '');
    show(el.pdf, !!job && job.kind === 'pdf');
    show(el.ifc, !!job && job.kind === 'ifc');
    if (job && job.id !== lastJobId) {
      lastJobId = job.id;
      el.name.value = IM.suggestName(String(job.filename || '').replace(/\.[^.]*$/, ''));
      view = null;
      activateName = null;
    }
  }

  function renderPages() {
    var st = flow.state;
    if (!st.job || st.job.kind !== 'pdf') return;
    var pages = st.job.pages;
    var sig = pages.length + ':' + (st.page ? st.page.index : '-');
    if (el.page.dataset.sig !== sig) {
      el.page.dataset.sig = sig;
      el.page.textContent = '';
      var none = document.createElement('option');
      none.value = '';
      none.textContent = 'Choose a page…';
      el.page.appendChild(none);
      pages.forEach(function (p) {
        var o = document.createElement('option');
        o.value = String(p.index);
        o.textContent = 'Page ' + (p.index + 1) + (p.has_vectors ? ' (vector)' : ' (image)');
        el.page.appendChild(o);
      });
      el.page.value = st.page ? String(st.page.index) : '';
    }
    show(el.page.parentNode, pages.length > 1);
    setText(el.pageInfo, st.page
      ? 'Page ' + (st.page.index + 1) + ': ' + Math.round(st.page.png.width) + ' x ' + Math.round(st.page.png.height) + ' units'
      : (pages.length > 1 ? 'Pick a page to start.' : ''));
  }

  function renderScale() {
    var st = flow.state;
    var prop = flow.proposal();
    var rep = '';
    if (st.page && st.page.scale) {
      var p = st.page.scale.proposal;
      if (prop) {
        rep = 'Proposed scale: 1 page unit = ' + prop.m_per_unit.toPrecision(5) + ' m (' + p.confidence
          + ' confidence). ' + (p.evidence || []).join('; ');
      } else {
        rep = 'No scale proposal for this page.';
      }
    }
    setText(el.scaleReport, rep);
    show(el.required, flow.calibrationRequired());
    var busy = !!st.busy;
    el.accept.disabled = busy || !prop;
    var tp = st.twoPoint;
    setText(el.twoStatus, tp.p2 ? 'P1 and P2 set.' : tp.p1 ? 'P1 set. Tap P2.' : 'Tap two points on the page.');
    el.twoApply.disabled = busy || !(tp.p1 && tp.p2) || !(metresOf(el.twoMetres.value) > 0);
    el.typedApply.disabled = busy || !st.page;
    el.twoStart.disabled = busy || !st.page;
    el.originTap.disabled = busy || !st.page;
    el.robotFetch.disabled = busy || !st.page;
    el.robotApply.disabled = busy || flow.robotSpans().length < 2;
    var o = st.origin || (st.calibrated && st.calibrated.origin);
    setText(el.originText, o ? 'Map origin at page point (' + o[0].toFixed(1) + ', ' + o[1].toFixed(1) + ').'
      : 'Origin defaults to the lower-left corner of the walls.');
  }

  function renderRobot() {
    var st = flow.state;
    var r = st.robot;
    var sig = r.fetched + ':' + r.pairs.length + ':' + r.marks.map(function (m) { return m && m.p2 ? 2 : m && m.p1 ? 1 : 0; }).join('');
    if (el.robotList.dataset.sig === sig) return;
    el.robotList.dataset.sig = sig;
    el.robotList.textContent = '';
    if (!r.fetched) return;
    r.flags.forEach(function (f) {
      var d = document.createElement('div');
      d.className = 'dim';
      d.textContent = 'Lidar note: ' + f;
      el.robotList.appendChild(d);
    });
    if (r.pairs.length < 2) {
      var warn = document.createElement('div');
      warn.className = 'imp-note';
      warn.textContent = 'Found ' + r.pairs.length + ' wall pair axis. Two non-parallel pairs are needed: move the robot so two directions are in view.';
      el.robotList.appendChild(warn);
    }
    r.pairs.forEach(function (p, i) {
      var row = document.createElement('div');
      row.className = 'imp-pair';
      var m = r.marks[i];
      var label = document.createElement('span');
      label.textContent = 'W' + (i + 1) + ': ' + p.span_m.toFixed(3) + ' m at '
        + Math.round(p.axis_rad * 180 / Math.PI) + '°  ' + (m && m.p2 ? '(marked)' : m && m.p1 ? '(tap the 2nd wall)' : '');
      var btn = document.createElement('button');
      btn.type = 'button';
      btn.textContent = m && m.p2 ? 'Re-mark' : 'Mark on page';
      btn.addEventListener('click', function () { flow.beginTap('robot', i); });
      row.appendChild(label);
      row.appendChild(btn);
      el.robotList.appendChild(row);
    });
  }

  function renderPreview() {
    var st = flow.state;
    if (st.page !== lastPage) { lastPage = st.page; view = null; }
    loadImage('page', 'pageBlob', st.page && st.page.blob, drawAll);
    loadImage('preview', 'previewBlob', st.preview && st.preview.blob, drawAll);
    var c = st.calibrated;
    var info = '';
    if (c) {
      var pv = c.preview || {};
      info = 'Scale ' + c.m_per_unit.toPrecision(5) + ' m/unit (' + c.method + '), rotation ' + c.rotation_deg
        + '°. Grid ' + pv.width + ' x ' + pv.height + ' cells at ' + c.resolution + ' m';
      if (pv.interior_fraction !== undefined) info += '; interior ' + Math.round(pv.interior_fraction * 100) + '%';
      if (st.dirty) info += '. Out of date: updating…';
    } else if (st.page) {
      info = flow.calibrationRequired() ? 'Calibrate to see a preview.' : 'Accept the scale or calibrate to see a preview.';
    }
    setText(el.previewInfo, info);
    var warns = st.warnings || [];
    show(el.warnings, warns.length > 0);
    setText(el.warnings, warns.join(' | '));
  }

  function renderTap() {
    var tap = flow.state.tap;
    show(el.tapBanner, !!tap);
    if (!tap) return;
    var text = 'Tap the page.';
    if (tap.mode === 'origin') text = 'Tap the page point that should be map (0, 0). Taps do not move the robot.';
    else if (tap.mode === 'two') text = tap.step === 0 ? 'Tap point 1 on the page.' : 'Tap point 2 (at least ' + IM.TAP_MIN_PX + ' px away).';
    else if (tap.mode === 'robot') text = 'Wall pair W' + (tap.index + 1) + ': tap a point on ' + (tap.step === 0 ? 'one wall.' : 'the opposite wall.');
    setText(el.tapText, text);
  }

  function renderInstall() {
    var st = flow.state;
    var name = el.name.value;
    setText(el.nameMsg, name && !IM.validName(name) ? 'Name: a-z 0-9 _ - only, start with a letter or digit, up to 32 characters.' : '');
    el.install.disabled = !!st.busy || !flow.previewCurrent() || !IM.validName(name);
    var inst = st.install;
    if (inst && inst.ok) {
      setText(el.result, 'Installed "' + inst.name + '". It is not active yet.');
      activateName = inst.name;
    } else {
      setText(el.result, '');
    }
    show(el.activate, !!(inst && inst.ok && activateName));
    el.activate.disabled = !!st.busy;
  }

  function storeySignature(ifc) {
    return ifc.storeys.map(function (s) {
      return [s.id, s.selected ? 1 : 0, s.slice ? 1 : 0, s.dirty ? 1 : 0, s.error || '', s.result ? s.result.msg : '', ifc.focus === s.id ? 1 : 0].join('|');
    }).join(';');
  }

  function storeyRow(s, ifc) {
    var row = document.createElement('div');
    row.className = 'imp-storey';
    var tick = document.createElement('input');
    tick.type = 'checkbox';
    tick.checked = s.selected;
    tick.addEventListener('change', function () { flow.setStorey(s.id, 'selected', tick.checked); });
    var title = document.createElement('span');
    title.textContent = s.label + ' (' + (typeof s.elevation_m === 'number' ? s.elevation_m.toFixed(2) : '?') + ' m)';
    var name = document.createElement('input');
    name.type = 'text';
    name.value = s.name;
    name.placeholder = 'map name';
    name.addEventListener('input', function () { flow.setStorey(s.id, 'name', name.value); });
    var floor = document.createElement('input');
    floor.type = 'number';
    floor.step = '1';
    floor.value = s.floor;
    floor.placeholder = 'floor #';
    floor.addEventListener('input', function () { flow.setStorey(s.id, 'floor', floor.value); });
    var see = document.createElement('button');
    see.type = 'button';
    see.textContent = ifc.focus === s.id ? 'Showing' : 'Show';
    see.addEventListener('click', function () { flow.focusStorey(s.id); });
    var status = document.createElement('div');
    status.className = s.error || (s.result && !s.result.ok) ? 'imp-error' : 'dim';
    status.textContent = s.result ? s.result.msg : s.error ? s.error
      : s.slice ? (s.dirty ? 'Sliced (settings changed: slice again)' : 'Sliced: ' + s.slice.faces + ' faces') : 'Not sliced yet';
    [tick, title, name, floor, see, status].forEach(function (n) { row.appendChild(n); });
    return row;
  }

  function renderIfc() {
    var st = flow.state;
    if (!st.ifc) return;
    var sig = storeySignature(st.ifc);
    if (storeySig !== sig) {
      storeySig = sig;
      el.storeys.textContent = '';
      st.ifc.storeys.forEach(function (s) { el.storeys.appendChild(storeyRow(s, st.ifc)); });
    }
    var busy = !!st.busy;
    el.ifcSlice.disabled = busy;
    el.ifcInstall.disabled = busy || !st.ifc.storeys.some(function (s) { return s.selected && s.slice; });
    var f = ifcFocus();
    el.ifcOrigin.disabled = busy || !(f && f.slice);
    loadImage('ifc', 'ifcBlob', f && f.previewBlob, drawIfc);
    var info = '';
    if (f && f.slice) {
      info = f.label + ': ' + f.slice.faces + ' faces at ' + f.slice.z.toFixed(2) + ' m, grid '
        + f.slice.preview.width + ' x ' + f.slice.preview.height + ' cells';
      if (f.slice.warnings && f.slice.warnings.length) info += '. ' + f.slice.warnings.join(' | ');
    }
    if (ifcTapping) info = 'Tap the preview where map (0, 0) should be. ' + info;
    setText(el.ifcInfo, info);
    el.ifcActivate.textContent = '';
    st.ifc.storeys.forEach(function (s) {
      if (s.result && s.result.ok) {
        var b = document.createElement('button');
        b.type = 'button';
        b.textContent = 'Activate ' + s.name;
        b.addEventListener('click', function () { cfg.activateMap(s.name); });
        el.ifcActivate.appendChild(b);
      }
    });
  }

  function drawAll() {
    drawPage();
    drawIfc();
  }

  function render() {
    if (!flow) return;
    renderStatus();
    renderPages();
    renderScale();
    renderRobot();
    renderPreview();
    renderTap();
    renderInstall();
    renderIfc();
    drawAll();
  }

  // --- events -----------------------------------------------------------------------------------------------------------

  function canvasPoint(ev) {
    var r = el.canvas.getBoundingClientRect();
    return { x: ev.clientX - r.left, y: ev.clientY - r.top };
  }

  function pageSize() { return { w: el.canvas.clientWidth || 300, h: el.canvas.clientHeight || 300 }; }

  function zoom(factor, x, y) {
    var st = flow.state;
    if (!st.page || !view) return;
    var size = pageSize();
    var b = pageBounds(st.page.png);
    var kfit = MV.fitK(b, size);
    view = MV.constrain(MV.zoomAt(view, size, x === undefined ? size.w / 2 : x, y === undefined ? size.h / 2 : y, factor, kfit), size, b, 48);
    drawPage();
  }

  function wireCanvas() {
    el.canvas.addEventListener('pointerdown', function (ev) {
      if (!view) return;
      var p = canvasPoint(ev);
      drag = { x0: p.x, y0: p.y, view0: view, moved: false };
      el.canvas.setPointerCapture(ev.pointerId);
    });
    el.canvas.addEventListener('pointermove', function (ev) {
      if (!drag) return;
      var p = canvasPoint(ev);
      var dx = p.x - drag.x0, dy = p.y - drag.y0;
      if (!drag.moved && Math.hypot(dx, dy) < TAP_SLOP_PX) return;
      drag.moved = true;
      var size = pageSize();
      view = MV.constrain(MV.panBy(drag.view0, dx, dy), size, pageBounds(flow.state.page.png), 48);
      drawPage();
    });
    el.canvas.addEventListener('pointerup', function (ev) {
      var d = drag;
      drag = null;
      if (!d || d.moved || !flow.state.tap) return;
      var p = canvasPoint(ev);
      flow.handleTap({ page: IM.canvasToPage(view, pageSize(), p.x, p.y), screen: p });
    });
    el.canvas.addEventListener('wheel', function (ev) {
      ev.preventDefault();
      var p = canvasPoint(ev);
      zoom(ev.deltaY < 0 ? ZOOM_STEP : 1 / ZOOM_STEP, p.x, p.y);
    }, { passive: false });
    el.zoomIn.addEventListener('click', function () { zoom(ZOOM_STEP); });
    el.zoomOut.addEventListener('click', function () { zoom(1 / ZOOM_STEP); });
    el.zoomFit.addEventListener('click', function () { fitView(); drawPage(); });
    el.ifcCanvas.addEventListener('click', function (ev) {
      var f = ifcFocus();
      if (!ifcTapping || !f || !f.slice || !imgs.ifc) return;
      var r = el.ifcCanvas.getBoundingClientRect();
      var px = IM.canvasToImagePx(ifcFit(), ev.clientX - r.left, ev.clientY - r.top);
      var pv = f.slice.preview;
      var u = px.x * pv.width / imgs.ifc.naturalWidth, v = px.y * pv.height / imgs.ifc.naturalHeight;
      if (u < 0 || v < 0 || u > pv.width || v > pv.height) return;
      ifcTapping = false;
      flow.setStoreyOrigin(f.id, IM.ifcOriginFromTap(f.slice, u, v));
      render();
    });
  }

  function typedFromForm() {
    var ratioText = el.typedRatio.value.trim();
    var w = el.paperW.value.trim(), h = el.paperH.value.trim();
    return {
      scale: el.typedScale.value,
      ratio: ratioText === '' ? '' : Number(ratioText),
      paper_in: w || h ? [Number(w), Number(h)] : null,
    };
  }

  function wire() {
    el.open.addEventListener('click', function () {
      show(el.panel, true);
      flow.open();
      render();
    });
    el.close.addEventListener('click', function () { show(el.panel, false); });
    el.retry.addEventListener('click', function () { flow.setBase(el.base.value); });
    el.base.addEventListener('change', function () { flow.setBase(el.base.value); });
    el.upload.addEventListener('click', function () {
      var file = el.file.files && el.file.files[0];
      if (!file) { flow.state.message = 'Choose a PDF or IFC file first'; render(); return; }
      flow.upload(file);
    });
    el.page.addEventListener('change', function () {
      if (el.page.value !== '') flow.selectPage(parseInt(el.page.value, 10));
    });
    el.accept.addEventListener('click', function () { flow.acceptProposal(); });
    el.twoStart.addEventListener('click', function () { flow.beginTap('two'); });
    el.twoMetres.addEventListener('input', renderScale);
    el.twoApply.addEventListener('click', function () { flow.applyTwoPoint(metresOf(el.twoMetres.value)); });
    el.typedApply.addEventListener('click', function () { flow.applyTyped(typedFromForm()); });
    el.robotFetch.addEventListener('click', function () { flow.fetchWallSpans(); });
    el.robotApply.addEventListener('click', function () { flow.applyRobot(); });
    el.rotation.addEventListener('change', function () { flow.setRotation(Number(el.rotation.value)); });
    el.originTap.addEventListener('click', function () { flow.beginTap('origin'); });
    el.forceRaster.addEventListener('change', function () { flow.setForceRaster(el.forceRaster.checked); });
    el.tapCancel.addEventListener('click', function () { flow.cancelTap(); });
    el.name.addEventListener('input', renderInstall);
    el.install.addEventListener('click', function () {
      flow.install({ name: el.name.value, label: el.label.value, floor: el.floor.value });
    });
    el.activate.addEventListener('click', function () { if (activateName) cfg.activateMap(activateName); });
    el.ifcSlice.addEventListener('click', function () {
      var h = el.ifcHeight.value.trim();
      flow.setIfcSettings(h === '' ? null : Number(h), Number(el.ifcRotation.value || 0));
      flow.sliceSelected();
    });
    el.ifcOrigin.addEventListener('click', function () { ifcTapping = true; render(); });
    el.ifcInstall.addEventListener('click', function () { flow.installSelected(); });
    wireCanvas();
  }

  var IDS = {
    open: 'import-open', panel: 'import-panel', close: 'import-close', base: 'imp-base',
    retry: 'imp-retry', health: 'imp-health', busy: 'imp-busy', error: 'imp-error',
    message: 'imp-message', file: 'imp-file', upload: 'imp-upload', job: 'imp-job',
    pdf: 'imp-pdf', page: 'imp-page', pageInfo: 'imp-page-info', required: 'imp-required',
    scaleReport: 'imp-scale-report', accept: 'imp-accept', twoStart: 'imp-two-start',
    twoMetres: 'imp-two-metres', twoApply: 'imp-two-apply', twoStatus: 'imp-two-status',
    typedScale: 'imp-typed-scale', typedRatio: 'imp-typed-ratio', paperW: 'imp-paper-w',
    paperH: 'imp-paper-h', typedApply: 'imp-typed-apply', robotFetch: 'imp-robot-fetch',
    robotApply: 'imp-robot-apply', robotList: 'imp-robot-list', rotation: 'imp-rotation',
    originTap: 'imp-origin-tap', originText: 'imp-origin-text', forceRaster: 'imp-force-raster',
    tapBanner: 'imp-tap-banner', tapText: 'imp-tap-text', tapCancel: 'imp-tap-cancel',
    canvas: 'imp-canvas', zoomIn: 'imp-zoom-in', zoomOut: 'imp-zoom-out', zoomFit: 'imp-zoom-fit',
    previewInfo: 'imp-preview-info', warnings: 'imp-warnings', name: 'imp-name', label: 'imp-label',
    floor: 'imp-floor', nameMsg: 'imp-name-msg', install: 'imp-install', result: 'imp-result',
    activate: 'imp-activate', ifc: 'imp-ifc', ifcHeight: 'imp-ifc-height',
    ifcRotation: 'imp-ifc-rotation', ifcSlice: 'imp-ifc-slice', storeys: 'imp-storeys',
    ifcOrigin: 'imp-ifc-origin', ifcCanvas: 'imp-ifc-canvas', ifcInfo: 'imp-ifc-info',
    ifcInstall: 'imp-ifc-install', ifcActivate: 'imp-ifc-activate',
  };

  // opts: {fleetApi, hostname, fetch, storage, refreshSites, activateMap}
  function init(opts) {
    cfg = opts;
    Object.keys(IDS).forEach(function (k) { el[k] = $(IDS[k]); });
    flow = root.ImportFlow.createFlow({
      fetch: opts.fetch,
      storage: opts.storage,
      hostname: opts.hostname,
      getCompanion: function () {
        return opts.fetch(opts.fleetApi + '/companion').then(function (r) { return r.json(); });
      },
      onInstalled: function () { return opts.refreshSites(); },
      onChange: render,
    });
    wire();
    render();
    return flow;
  }

  root.ImportWizard = { init: init };
})(typeof window !== 'undefined' ? window : globalThis);
