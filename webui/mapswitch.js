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

  var api = {
    mapSource: mapSource,
    effectiveSlamMode: effectiveSlamMode,
    activationPlan: activationPlan,
    needsReseed: needsReseed,
  };
  root.MapSwitch = api;
  if (typeof module === 'object' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
