/* Scout WebUI — scan-over-drawing overlay maths (pure, no DOM, no ROS). Classic
 * script; exposes window.ScanOverlay (and module.exports under Node for
 * scout/test/test_scanoverlay.py).
 *
 * On a drawing-sourced map the lidar scan is drawn over the grid so the operator can
 * see whether the robot's seeded pose puts the walls where the scan sees them (R10,
 * AE8). Everything here is a HINT: the fit is the share of scan points that land
 * near a wall cell of the grid painted on screen, which a wrong pose in a repetitive
 * building can also score well on. It is never localization proof and no caller may
 * gate a goal on it.
 *
 * Frames. A pose {x, y, yaw} is a rigid 2D transform parent<-child: robot pose is
 * map<-base_link, LASER is base_link<-laser. A beam at laser-frame angle a and range
 * r is the laser-frame point (r cos a, r sin a); map = pose o LASER o that point.
 */
(function (root) {
  'use strict';

  var MS = root.MapSwitch
    || (typeof require === 'function' ? require('./mapswitch.js') : null);

  // base_link<-laser. The scanner is mounted BACKWARDS: 0.073 m forward of base_link,
  // yawed pi (CLAUDE.md, measured 2026-07-30). Same constant as LASER_FALLBACK in
  // docker/scout-skills/scanmeasure.py. The page does not read /tf_static, so this
  // is what it uses; getting it wrong mirrors the overlay front-to-back.
  var LASER_FALLBACK = { x: 0.073, y: 0.0, yaw: Math.PI };

  var MAX_BEAMS = 360;        // beams considered per scan (valid or not)
  var FIT_RADIUS_M = 0.15;    // a point "fits" within this of an occupied cell
  var STALE_MS = 2000;        // /scan older than this: grey, no number
  // Mirrors robot_profile.yaml occupied_threshold (the page passes its own).
  var DEFAULT_OCC_THRESHOLD = 50;
  var MAX_DILATE_CELLS = 40;  // guards a pathologically fine grid
  var DISMISS_PREFIX = 'scout.scanoverlay.dismissed.';

  // --- when the layer is wanted -------------------------------------------------

  // Only while amcl is running AND the active map is a drawing: on a scan map the
  // map came from this lidar, so overlaying it proves nothing.
  function overlayWanted(meta) {
    if (!MS || !meta || !meta.active_map) return false;
    var entry = (meta.maps || {})[meta.active_map];
    return MS.effectiveSlamMode(meta) === 'localization'
      && MS.mapSource(entry) === 'drawing';
  }

  function mapKey(meta) {
    if (!meta || !meta.active_map) return null;
    return (meta.name || '') + '/' + meta.active_map;
  }

  function dismissKey(key) {
    return DISMISS_PREFIX + key;
  }

  // A different active map than last time (not the first one seen): the operator
  // switched maps, so the banner must be confirmed again for the new one.
  function isMapSwitch(prevKey, key) {
    return prevKey !== null && prevKey !== undefined && key !== prevKey;
  }

  // --- poses ---------------------------------------------------------------------

  // PoseWithCovarianceStamped -> {x, y, yaw}; yaw from the planar quaternion.
  function poseFromMsg(msg) {
    var p = msg && msg.pose && msg.pose.pose;
    if (!p || !p.position || !p.orientation) return null;
    var x = p.position.x, y = p.position.y;
    if (!isFinite(x) || !isFinite(y)) return null;
    return { x: x, y: y, yaw: 2 * Math.atan2(p.orientation.z, p.orientation.w) };
  }

  function compose(a, b) {
    var c = Math.cos(a.yaw), s = Math.sin(a.yaw);
    return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, yaw: a.yaw + b.yaw };
  }

  // --- projection ----------------------------------------------------------------

  function validRange(r, lo, hi) {
    return typeof r === 'number' && isFinite(r) && r > 0 && r >= lo && r <= hi;
  }

  // scan: LaserScan {ranges, angle_min, angle_increment, range_min, range_max}.
  // pose: robot in the map frame. laser: base_link<-laser (default LASER_FALLBACK).
  // rosbridge sends inf/NaN ranges as null: all of null, NaN, +-Infinity, <= 0,
  // < range_min and > range_max are skipped. At most MAX_BEAMS beams are looked at.
  function projectScan(scan, pose, laser) {
    var out = [];
    if (!scan || !pose || !scan.ranges || !scan.ranges.length) return out;
    var lz = laser || LASER_FALLBACK;
    var origin = compose(pose, lz);
    var c = Math.cos(origin.yaw), s = Math.sin(origin.yaw);
    var n = scan.ranges.length;
    var step = Math.max(1, Math.ceil(n / MAX_BEAMS));
    var lo = scan.range_min || 0;
    var hi = scan.range_max === undefined || scan.range_max === null
      ? Infinity : scan.range_max;
    for (var i = 0; i < n; i += step) {
      var r = scan.ranges[i];
      if (!validRange(r, lo, hi)) continue;
      var a = scan.angle_min + i * scan.angle_increment;
      var lx = r * Math.cos(a), ly = r * Math.sin(a);
      out.push({ x: origin.x + c * lx - s * ly, y: origin.y + s * lx + c * ly });
    }
    return out;
  }

  // --- fit -----------------------------------------------------------------------

  // OccupancyGrid message -> {width, height, resolution, ox, oy, mask}. mask[row *
  // width + col] is 1 for every cell whose centre is within FIT_RADIUS_M of an
  // occupied cell centre (row 0 = the BOTTOM row, world y-min, like the message).
  // Built once per grid message; fit() is then one lookup per beam.
  function buildMask(msg, occThreshold) {
    var info = msg && msg.info;
    if (!info || !msg.data) return null;
    var w = info.width, h = info.height, res = info.resolution;
    if (!(w > 0 && h > 0 && res > 0) || msg.data.length < w * h) return null;
    var thr = occThreshold === undefined ? DEFAULT_OCC_THRESHOLD : occThreshold;
    // + 1e-6: 0.15 / 0.05 is 2.9999999999999996, which would drop the 3-cell ring
    var rc = Math.min(FIT_RADIUS_M / res + 1e-6, MAX_DILATE_CELLS);
    var reach = Math.floor(rc);
    var offsets = [];
    for (var dy = -reach; dy <= reach; dy++) {
      for (var dx = -reach; dx <= reach; dx++) {
        if (dx * dx + dy * dy <= rc * rc) offsets.push([dx, dy]);
      }
    }
    var mask = new Uint8Array(w * h);
    for (var row = 0; row < h; row++) {
      for (var col = 0; col < w; col++) {
        if (msg.data[row * w + col] < thr) continue;
        for (var k = 0; k < offsets.length; k++) {
          var cc = col + offsets[k][0], rr = row + offsets[k][1];
          if (cc >= 0 && cc < w && rr >= 0 && rr < h) mask[rr * w + cc] = 1;
        }
      }
    }
    return {
      width: w, height: h, resolution: res,
      ox: info.origin.position.x, oy: info.origin.position.y, mask: mask,
    };
  }

  // Share of points within FIT_RADIUS_M of an occupied cell, 0..1. null (the UI
  // shows an en dash) when there are no points: a number from nothing would lie.
  // A point off the grid counts as a miss.
  function fit(points, m) {
    if (!points || !points.length || !m) return null;
    var hit = 0;
    for (var i = 0; i < points.length; i++) {
      var col = Math.floor((points[i].x - m.ox) / m.resolution);
      var row = Math.floor((points[i].y - m.oy) / m.resolution);
      if (col >= 0 && col < m.width && row >= 0 && row < m.height
          && m.mask[row * m.width + col]) hit++;
    }
    return hit / points.length;
  }

  function formatFit(f) {
    return f === null || f === undefined ? '–' : Math.round(f * 100) + '%';
  }

  // --- staleness -----------------------------------------------------------------

  // atMs: arrival time (the page's own clock) of the last /scan; never
  // compared with a message stamp, the robot's clock is not the browser's.
  function isStale(nowMs, atMs, limitMs) {
    if (atMs === null || atMs === undefined) return true;
    return nowMs - atMs > (limitMs === undefined ? STALE_MS : limitMs);
  }

  var api = {
    LASER_FALLBACK: LASER_FALLBACK, MAX_BEAMS: MAX_BEAMS, FIT_RADIUS_M: FIT_RADIUS_M,
    STALE_MS: STALE_MS, DEFAULT_OCC_THRESHOLD: DEFAULT_OCC_THRESHOLD,
    overlayWanted: overlayWanted, mapKey: mapKey, dismissKey: dismissKey,
    isMapSwitch: isMapSwitch, poseFromMsg: poseFromMsg, compose: compose,
    projectScan: projectScan, buildMask: buildMask, fit: fit, formatFit: formatFit,
    isStale: isStale,
  };
  root.ScanOverlay = api;
  if (typeof module === 'object' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
