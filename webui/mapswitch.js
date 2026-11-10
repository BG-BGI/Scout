/* Scout WebUI — map activation decision (pure, no DOM). Classic script; exposes
 * window.MapSwitch (and module.exports under Node for scout/test/test_mapswitch.py).
 *
 * Switching the active map is a LIVE map_server LoadMap only when amcl is running
 * (effective mode localization) AND the target has the same source (scan | drawing)
 * as the live map. amcl's parameters are fixed at launch, so a cross-source switch
 * must restart slam instead; the launch then composes the matching source overlay.
 * `meta` is one site from fleet_status's /sites payload.
 */
(function (root) {
  'use strict';

  var SOURCES = ['scan', 'drawing'];

  // Unknown, missing or non-string source reads as scan (same rule as core.sites).
  function mapSource(entry) {
    return entry && SOURCES.indexOf(entry.source) >= 0 ? entry.source : 'scan';
  }

  // The mode slam is actually running as. fleet_status's resolved_mode resolves
  // `auto` (a drawing map runs localization); an older fleet_status lacks the field
  // and the raw slam_mode is all there is.
  function effectiveSlamMode(meta) {
    return (meta && (meta.resolved_mode || meta.slam_mode)) || 'auto';
  }

  function activationPlan(meta, name) {
    var maps = (meta && meta.maps) || {};
    var target = maps[name] || {};
    var current = maps[meta && meta.active_map] || {};
    var targetSource = mapSource(target);
    var currentSource = mapSource(current);
    var crossSource = targetSource !== currentSource;
    var live = effectiveSlamMode(meta) === 'localization' && !crossSource;
    // Will slam launch amcl on the target (which then needs its grid)? Judged from
    // the raw policy: for `auto` only a drawing map resolves to localization.
    var rawMode = (meta && meta.slam_mode) || 'auto';
    var targetLocalizes = rawMode === 'localization'
      || (rawMode === 'auto' && targetSource === 'drawing');
    return {
      live: live,
      crossSource: crossSource,
      targetSource: targetSource,
      currentSource: currentSource,
      needsGrid: live || targetLocalizes,
    };
  }

  // After a restart: has `meta` (re-fetched) caught up with the switch AND does it
  // run amcl? Only then is there an amcl_pose to wait for and a tag relocalizer to
  // re-arm. A stale payload (active_map still the old one) must not trigger it.
  function needsReseed(meta, name) {
    return !!meta && meta.active_map === name
      && effectiveSlamMode(meta) === 'localization';
  }

  // --- telling the NEW amcl's pose from the old one's latched pose ---------------
  // nav2_amcl publishes /amcl_pose transient-local (KeepLast 1) and a stationary
  // amcl does not republish, so a subscriber opened right after a restart can be
  // handed the OLD amcl's last pose. header.stamp is the laser scan's stamp, so a
  // pose from the restarted amcl never carries the old stamp. Compare stamps only
  // against each other — never against a wall clock (the browser clock is not the
  // robot's, and the Pi's steps at boot).

  // 'sec:nanosec' of a message's header.stamp, or null when it has none.
  function amclStamp(msg) {
    var stamp = msg && msg.header && msg.header.stamp;
    if (!stamp || typeof stamp.sec !== 'number' || typeof stamp.nanosec !== 'number') {
      return null;
    }
    return stamp.sec + ':' + stamp.nanosec;
  }

  // baselineStamp: stamp of the pose latched BEFORE the restart (null = none, e.g.
  // mapping -> localization). A message counts only if it exists and its stamp
  // differs from that baseline.
  function isFreshAmclPose(baselineStamp, msg) {
    if (msg === null || msg === undefined) return false;
    if (baselineStamp === null || baselineStamp === undefined) return true;
    return amclStamp(msg) !== baselineStamp;
  }

  // feed(msg) -> true exactly once: on the first fresh message. A stale replay
  // followed by the real pose therefore triggers the caller's reseed once, after
  // the real one.
  function freshPoseGate(baselineStamp) {
    var fired = false;
    return function (msg) {
      if (fired || !isFreshAmclPose(baselineStamp, msg)) return false;
      fired = true;
      return true;
    };
  }

  var api = {
    mapSource: mapSource,
    effectiveSlamMode: effectiveSlamMode,
    activationPlan: activationPlan,
    needsReseed: needsReseed,
    amclStamp: amclStamp,
    isFreshAmclPose: isFreshAmclPose,
    freshPoseGate: freshPoseGate,
  };
  root.MapSwitch = api;
  if (typeof module === 'object' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
