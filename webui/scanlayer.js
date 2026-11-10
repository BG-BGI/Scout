/* Scout WebUI — scan-over-drawing overlay (ROS + DOM glue around scanoverlay.js).
 * Classic script; exposes window.ScanLayer (and module.exports under Node for
 * scout/test/test_scanoverlay.py, which injects fakes for everything below).
 *
 * app.js calls four hooks and owns nothing else of this feature:
 *   ScanLayer.init({...})         once, with the page's ros / grid / redraw
 *   ScanLayer.sync(activeSiteMeta) whenever the site status is re-rendered
 *   ScanLayer.notePose(pose)      from the existing /amcl_pose handler
 *   ScanLayer.draw(ctx, w2c)      from drawMap(), above the grid, below the robot
 *
 * The layer is active only while the effective mode is localization AND the active
 * map is a drawing (ScanOverlay.overlayWanted). Then it holds one rosbridge
 * subscription, /scan at 2 Hz with queue_length 1, and drops it the moment the
 * condition goes false. It reads the occupancy grid the page already holds (the
 * /map subscription): it never calls get_map or any other heavy read (ADR-0039).
 *
 * It never blocks, delays or gates a goal. The banner is advice, and the fit it
 * shows is captioned "alignment hint, not localization proof".
 */
(function (root) {
  'use strict';

  var SO = root.ScanOverlay
    || (typeof require === 'function' ? require('./scanoverlay.js') : null);

  var SCAN_THROTTLE_MS = 500;   // 2 Hz on the wire
  var REFRESH_MS = 500;         // staleness re-check + redraw cadence
  // amcl publishes /amcl_pose only after the robot has moved (update_min_d/a), so a
  // robot standing still after Set pose sends nothing for minutes, and "no message
  // for 2 s" would grey the layer in exactly the situation it exists for. While the
  // layer is active the page therefore re-asks for the LATCHED pose (a fresh
  // subscription gets the latest one at once); a pose that answers is current, one
  // that does not (amcl down) goes stale like any other input.
  var POSE_POLL_MS = 1500;
  var POSE_POLL_WAIT_MS = 700;
  var POINT_LIVE = 'rgba(60, 235, 120, 0.9)';
  var POINT_STALE = 'rgba(150, 150, 150, 0.55)';

  var cfg = null;
  var st = blank();
  var lastMapKey = null;        // survives stop(): a switch A -> scan map -> A counts
  var dismissedMem = {};        // fallback when sessionStorage is unavailable

  function blank() {
    return {
      active: false, key: null, scanTopic: null, scanMsg: null, scanAt: null,
      pose: null, poseAt: null, points: [], fit: null, mask: null, maskFor: null,
      timer: null, polling: false, lastPollAt: null, wasStale: null,
    };
  }

  function now() {
    return cfg && cfg.now ? cfg.now() : Date.now();
  }

  // --- once-per-session-per-map dismissal ----------------------------------------

  function store() {
    try {
      return cfg.storage ? cfg.storage() : root.sessionStorage;
    } catch (e) {
      return null;
    }
  }

  function isDismissed(key) {
    if (key === null || dismissedMem[key]) return true;
    try {
      var s = store();
      return !!s && s.getItem(SO.dismissKey(key)) === '1';
    } catch (e) {
      return false;
    }
  }

  function dismiss(key) {
    dismissedMem[key] = true;
    try {
      var s = store();
      if (s) s.setItem(SO.dismissKey(key), '1');
    } catch (e) { /* storage blocked: the in-memory flag still holds for this page */ }
  }

  function clearDismissal(key) {
    delete dismissedMem[key];
    try {
      var s = store();
      if (s) s.removeItem(SO.dismissKey(key));
    } catch (e) { /* see dismiss */ }
  }

  // --- derived state --------------------------------------------------------------

  function stale() {
    var t = now();
    return SO.isStale(t, st.scanAt) || SO.isStale(t, st.poseAt);
  }

  function recompute() {
    var grid = cfg.getGrid ? cfg.getGrid() : null;
    if (grid && grid !== st.maskFor) {
      var thr = typeof cfg.occThreshold === 'function' ? cfg.occThreshold() : cfg.occThreshold;
      st.mask = SO.buildMask(grid, thr);
      st.maskFor = grid;
    }
    if (!st.scanMsg || !st.pose) {
      st.points = [];
      st.fit = null;
      return;
    }
    st.points = SO.projectScan(st.scanMsg, st.pose, cfg.laser);
    st.fit = st.mask ? SO.fit(st.points, st.mask) : null;
  }

  function renderBanner() {
    if (!cfg || !cfg.banner) return;
    var show = st.active && !isDismissed(st.key);
    cfg.banner.hidden = !show;
    var old = stale();
    if (cfg.fitEl) {
      cfg.fitEl.textContent = old ? '–' : SO.formatFit(st.fit);
    }
    if (cfg.banner.classList) cfg.banner.classList.toggle('stale', st.active && old);
  }

  function redraw() {
    if (cfg && cfg.redraw) cfg.redraw();
  }

  // --- subscriptions --------------------------------------------------------------

  function onScan(msg) {
    if (!st.active) return;
    st.scanMsg = msg;
    st.scanAt = now();
    recompute();
    renderBanner();
    redraw();
  }

  function notePose(pose) {
    if (!st.active || !pose) return;
    st.pose = pose;
    st.poseAt = now();
    recompute();
  }

  function pollPose() {
    if (st.polling || !cfg.nextAmclPose) return;
    st.polling = true;
    st.lastPollAt = now();
    var mine = st;
    cfg.nextAmclPose(POSE_POLL_WAIT_MS).then(function (msg) {
      mine.polling = false;
      if (st !== mine || !msg) return;
      notePose(SO.poseFromMsg(msg));
      renderBanner();
      redraw();
    }, function () {
      mine.polling = false;
    });
  }

  function refresh() {
    if (!st.active) return;
    if (st.lastPollAt === null || now() - st.lastPollAt >= POSE_POLL_MS) pollPose();
    var nowStale = stale();
    renderBanner();
    if (nowStale !== st.wasStale) {
      st.wasStale = nowStale;
      redraw();
    }
  }

  function start(key) {
    st = blank();
    st.active = true;
    st.key = key;
    st.scanTopic = new cfg.ROSLIB.Topic({
      ros: cfg.ros, name: '/scan', messageType: 'sensor_msgs/msg/LaserScan',
      throttle_rate: SCAN_THROTTLE_MS, queue_length: 1,
    });
    st.scanTopic.subscribe(onScan);
    var timers = cfg.timers || root;
    st.timer = timers.setInterval(refresh, REFRESH_MS);
    pollPose();
  }

  function stop() {
    if (st.scanTopic) st.scanTopic.unsubscribe();
    if (st.timer !== null) (cfg.timers || root).clearInterval(st.timer);
    st = blank();
    renderBanner();
    redraw();
  }

  // --- hooks ------------------------------------------------------------------------

  function init(opts) {
    cfg = opts;
    if (cfg.dismissBtn) {
      cfg.dismissBtn.addEventListener('click', function () {
        if (st.active) dismiss(st.key);
        renderBanner();
      });
    }
    renderBanner();
  }

  function sync(meta) {
    if (!cfg) return;
    var key = SO.mapKey(meta);
    if (key !== null) {
      if (SO.isMapSwitch(lastMapKey, key)) clearDismissal(key);
      lastMapKey = key;
    }
    var wanted = SO.overlayWanted(meta);
    if (st.active && (!wanted || st.key !== key)) stop();
    if (wanted && !st.active) start(key);
    renderBanner();
  }

  // ctx: the goal tool's 2D context. w2c(x, y) -> {x, y} CSS px (the page's one
  // world-to-canvas transform).
  function draw(ctx, w2c) {
    if (!st.active || !st.points.length) return;
    ctx.save();
    ctx.fillStyle = stale() ? POINT_STALE : POINT_LIVE;
    for (var i = 0; i < st.points.length; i++) {
      var c = w2c(st.points[i].x, st.points[i].y);
      ctx.fillRect(c.x - 1, c.y - 1, 2, 2);
    }
    ctx.restore();
  }

  function snapshot() {
    return {
      active: st.active, key: st.key, points: st.points.length, fit: st.fit,
      stale: st.active ? stale() : null, subscribed: !!st.scanTopic,
      dismissed: st.key === null ? null : isDismissed(st.key),
    };
  }

  var api = {
    SCAN_THROTTLE_MS: SCAN_THROTTLE_MS, REFRESH_MS: REFRESH_MS,
    POSE_POLL_MS: POSE_POLL_MS,
    init: init, sync: sync, notePose: notePose, draw: draw, refresh: refresh,
    snapshot: snapshot,
  };
  root.ScanLayer = api;
  if (typeof module === 'object' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
