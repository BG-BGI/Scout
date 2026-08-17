/* Scout web teleop.
 *
 * cmd_vel contract (same as joystick_teleop.py): publish at robot_profile's
 * publish_hz ONLY while
 * an input is active, then a 0.3 s burst of zeros on release, then silence —
 * so Foxglove / nav2 / the robot-side pad can own /cmd_vel when we're idle.
 * The RoboClaw's 200 ms deadman is the backstop: if this page dies mid-drive,
 * the robot coasts to a stop within 200 ms of the last message.
 */
'use strict';

// --- constants -----------------------------------------------------------------
// Cross-surface values (publish rate, stop grace, nav-status names, speed caps)
// are loaded at startup from robot_profile.yaml (the SSOT the ROS nodes and
// scout-skills also read); these are the baked fallbacks if the fetch fails.
let PUBLISH_HZ = 25;  // profile-exempt: baked fallback if the profile fetch fails
let STOP_GRACE_MS = 300;
let BATT_WARN_V = 17.5;  // profile-exempt: baked fallback if the profile fetch fails
let BATT_CRIT_V = 16.5;  // profile-exempt: baked fallback if the profile fetch fails
const STICK_DEADZONE = 0.08;
const TURN_EXPO = 0.6;
const TRIGGER_DEADZONE = 0.03;

// --- ROS connection -----------------------------------------------------------
const ros = new ROSLIB.Ros({});
const connBadge = document.getElementById('conn');
let reconnectDelay = 1000;

function connect() {
  ros.connect('ws://' + location.hostname + ':9090');
}
ros.on('connection', () => {
  reconnectDelay = 1000;
  connBadge.textContent = 'connected';
  connBadge.className = 'badge connected';
});
ros.on('close', () => {
  connBadge.textContent = 'disconnected';
  connBadge.className = 'badge disconnected';
  setTimeout(connect, reconnectDelay);
  reconnectDelay = Math.min(reconnectDelay * 2, 5000);
});
ros.on('error', () => {});  // 'close' fires after and handles retry
connect();

// --- topics & services ----------------------------------------------------------
// Repointed to the profile's web topic (/cmd_vel_web -> twist_mux) once
// loadProfile() runs; the /cmd_vel default still works (lowest mux priority).
let cmdVel = new ROSLIB.Topic({
  ros, name: '/cmd_vel', messageType: 'geometry_msgs/msg/Twist',
});
const batteryTopic = new ROSLIB.Topic({
  ros, name: '/battery', messageType: 'sensor_msgs/msg/BatteryState',
});
const trickStatusTopic = new ROSLIB.Topic({
  ros, name: '/trick_status', messageType: 'std_msgs/msg/String',
});
const playTrickSrv = new ROSLIB.Service({
  ros, name: '/play_trick', serviceType: 'scout_interfaces/srv/PlayTrick',
});
const stopTrickSrv = new ROSLIB.Service({
  ros, name: '/stop_trick', serviceType: 'std_srvs/srv/Trigger',
});
const setUserLedSrv = new ROSLIB.Service({
  ros, name: '/set_user_led', serviceType: 'scout_interfaces/srv/SetLedMode',
});

// --- speed limits ----------------------------------------------------------------
const linSlider = document.getElementById('lin');
const angSlider = document.getElementById('ang');
linSlider.oninput = () => document.getElementById('lin-out').value = linSlider.value;
angSlider.oninput = () => document.getElementById('ang-out').value = angSlider.value;

// --- touch stick -------------------------------------------------------------------
// Y up = forward throttle (+1), X = turn input (-1 left .. +1 right).
const pad = document.getElementById('pad');
const stick = document.getElementById('stick');
let touch = { active: false, throttle: 0, turn: 0 };

function padVector(ev) {
  const r = pad.getBoundingClientRect();
  const x = (ev.clientX - r.left - r.width / 2) / (r.width / 2);
  const y = (ev.clientY - r.top - r.height / 2) / (r.height / 2);
  const mag = Math.hypot(x, y);
  const s = mag > 1 ? 1 / mag : 1;   // clamp to the pad circle
  return { x: x * s, y: y * s };
}
function padUpdate(ev) {
  const v = padVector(ev);
  touch.throttle = -v.y;             // screen-up is negative clientY
  touch.turn = v.x;
  stick.style.left = 50 + v.x * 33 + '%';
  stick.style.top = 50 + v.y * 33 + '%';
}
pad.addEventListener('pointerdown', (ev) => {
  pad.setPointerCapture(ev.pointerId);
  touch.active = true;
  pad.classList.add('active');
  padUpdate(ev);
});
pad.addEventListener('pointermove', (ev) => { if (touch.active) padUpdate(ev); });
function padRelease() {
  touch = { active: false, throttle: 0, turn: 0 };
  pad.classList.remove('active');
  stick.style.left = '50%';
  stick.style.top = '50%';
}
pad.addEventListener('pointerup', padRelease);
pad.addEventListener('pointercancel', padRelease);

// --- gamepad (Xbox pad paired to THIS device, browser "standard" mapping) ------------
// axes[0] = left stick X, buttons[7] = RT (forward), buttons[6] = LT (reverse).
// Mappings vary by browser/OS — the debug row shows live values to verify.
const gpState = document.getElementById('gamepad-state');
const gpDebug = document.getElementById('gamepad-debug');
let gamepadIndex = null;

window.addEventListener('gamepadconnected', (ev) => {
  gamepadIndex = ev.gamepad.index;
  gpState.textContent = ev.gamepad.id.slice(0, 40);
});
window.addEventListener('gamepaddisconnected', (ev) => {
  if (ev.gamepad.index === gamepadIndex) {
    gamepadIndex = null;
    gpState.textContent = 'no gamepad';
    gpDebug.textContent = '';
  }
});

let gpAWas = false;
function readGamepad() {
  if (gamepadIndex === null) return null;
  const gp = navigator.getGamepads()[gamepadIndex];
  if (!gp) return null;
  // A button (standard mapping index 0) marks a patrol waypoint, on press.
  const aNow = !!(gp.buttons[0] && gp.buttons[0].pressed);
  if (aNow && !gpAWas) patrolMark();
  gpAWas = aNow;
  const rt = gp.buttons[7] ? gp.buttons[7].value : 0;
  const lt = gp.buttons[6] ? gp.buttons[6].value : 0;
  const x = gp.axes[0] || 0;
  gpDebug.textContent =
    ' x=' + x.toFixed(2) + ' rt=' + rt.toFixed(2) + ' lt=' + lt.toFixed(2);
  const throttle = (rt < TRIGGER_DEADZONE ? 0 : rt) - (lt < TRIGGER_DEADZONE ? 0 : lt);
  let turn = Math.abs(x) < STICK_DEADZONE ? 0 : x;
  // Expo curve: gentle near center, unchanged at full deflection.
  turn = (1 - TURN_EXPO) * turn + TURN_EXPO * turn ** 3;
  return { active: throttle !== 0 || turn !== 0, throttle, turn };
}

// --- drive loop: the SOLE cmd_vel writer ----------------------------------------------
let lastActiveMs = 0;

function driveTick() {
  if (!ros.isConnected) return;

  // Touch wins when both are live; otherwise whichever is active.
  let input = touch.active ? touch : readGamepad();
  const active = !!(input && input.active !== false &&
                    (input.throttle !== 0 || input.turn !== 0));
  const now = performance.now();

  if (active) {
    lastActiveMs = now;
    const maxLin = parseFloat(linSlider.value);
    const maxAng = parseFloat(angSlider.value);
    // Push left (negative x) -> turn left = CCW = +z (REP-103).
    let wz = -input.turn * maxAng;
    // In reverse, invert turning so it steers like a car backing up.
    if (input.throttle < 0) wz = -wz;
    publishTwist(input.throttle * maxLin, wz);
  } else if (now - lastActiveMs < STOP_GRACE_MS) {
    publishTwist(0, 0);   // zero burst, then silence
  }
}
let driveTimer = null;
function startDriveLoop() {
  if (driveTimer) clearInterval(driveTimer);
  driveTimer = setInterval(driveTick, 1000 / PUBLISH_HZ);
}

// Tiny parser for robot_profile.yaml's deliberately flat format (2-space keys,
// JSON-flow lists, quoted strings, numbers). NOT a general YAML parser.
function parseProfile(text) {
  const p = {};
  for (const raw of text.split('\n')) {
    const line = raw.replace(/#.*$/, '');
    const m = line.match(/^ {2}([a-z0-9_]+):\s*(\S.*?)\s*$/);
    if (!m) continue;
    const [, key, val] = m;
    if (val[0] === '[') {
      try { p[key] = JSON.parse(val.replace(/'/g, '"')); } catch (e) { /* skip */ }
    } else if (val[0] === '"') {
      p[key] = val.slice(1, -1);
    } else {
      const n = parseFloat(val);
      p[key] = Number.isNaN(n) ? val : n;
    }
  }
  return p;
}

async function loadProfile() {
  try {
    const res = await fetch('robot_profile.yaml');
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const prof = parseProfile(await res.text());
    if (prof.publish_hz) PUBLISH_HZ = prof.publish_hz;
    if (prof.stop_grace_s) STOP_GRACE_MS = prof.stop_grace_s * 1000;
    if (Array.isArray(prof.goal_status_names)) {
      NAV_STATUS = {};
      prof.goal_status_names.forEach((name, i) => { NAV_STATUS[i] = name; });
    }
    if (prof.linear_cap) linSlider.max = prof.linear_cap;
    if (prof.angular_cap) angSlider.max = prof.angular_cap;
    if (prof.linear_floor) linSlider.min = prof.linear_floor;
    if (prof.angular_floor) angSlider.min = prof.angular_floor;
    if (prof.battery_warn_v) BATT_WARN_V = prof.battery_warn_v;
    if (prof.battery_critical_v) BATT_CRIT_V = prof.battery_critical_v;
    if (prof.topic_cmd_vel_web) {
      cmdVel = new ROSLIB.Topic({
        ros, name: prof.topic_cmd_vel_web, messageType: 'geometry_msgs/msg/Twist',
      });
    }
    startDriveLoop();  // restart at the profile's rate
  } catch (e) {
    console.warn('robot_profile.yaml fetch failed; using baked defaults:', e);
  }
}

startDriveLoop();  // start immediately with baked defaults (never gate driving on the fetch)
loadProfile();     // then override + restart from the SSOT

function publishTwist(vx, wz) {
  cmdVel.publish(new ROSLIB.Message({
    linear: { x: vx, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: wz },
  }));
}

function zeroBurst() {
  // Called from safety hooks: force the release path through the drive loop
  // and push one zero immediately in case the tab is about to be frozen.
  padRelease();
  lastActiveMs = performance.now();
  if (ros.isConnected) publishTwist(0, 0);
}
document.addEventListener('visibilitychange', () => {
  if (document.hidden) zeroBurst();
});
window.addEventListener('pagehide', zeroBurst);
window.addEventListener('beforeunload', zeroBurst);

// --- battery ------------------------------------------------------------------------
const batteryBadge = document.getElementById('battery');
batteryTopic.subscribe((msg) => {
  const pct = Number.isFinite(msg.percentage)
    ? ' · ' + Math.round(msg.percentage * 100) + '%' : '';
  batteryBadge.textContent = msg.voltage.toFixed(1) + ' V' + pct;
  batteryBadge.className = 'badge' +
    (msg.voltage <= BATT_CRIT_V ? ' batt-bad' :
     msg.voltage <= BATT_WARN_V ? ' batt-warn' : '');
});

// --- health strip (ADR-0014) ----------------------------------------------------------
// health_monitor aggregates battery/tilt/drivetrain onto /diagnostics at 1 Hz
// with a roll-up status named 'scout'. Badge shows the roll-up; the Health
// panel lists every item. If /diagnostics itself goes quiet (robot service
// down), the badge grays out — absence of data must not look like OK.
const healthBadge = document.getElementById('health');
const healthWorst = document.getElementById('health-worst');
const healthList = document.getElementById('health-list');
// DiagnosticStatus levels: 0 OK, 1 WARN, 2 ERROR, 3 STALE.
const HEALTH_CLS = { 0: '', 1: ' batt-warn', 2: ' batt-bad', 3: ' stale' };
const HEALTH_TXT = { 0: 'ok', 1: 'WARN', 2: 'ERROR', 3: 'STALE' };
let healthLastMs = 0;

new ROSLIB.Topic({
  ros, name: '/diagnostics', messageType: 'diagnostic_msgs/msg/DiagnosticArray',
  throttle_rate: 1000, queue_length: 1,
}).subscribe((msg) => {
  healthLastMs = performance.now();
  // level is a byte field — rosbridge may deliver number or 1-char string.
  const lvl = (s) => (typeof s.level === 'string' ? s.level.charCodeAt(0) : s.level);
  const rollup = msg.status.find((s) => s.name === 'scout');
  const items = msg.status.filter((s) => s.name !== 'scout');
  const worst = rollup ? lvl(rollup)
    : Math.max(0, ...items.map(lvl));
  healthBadge.textContent = 'health ' + (HEALTH_TXT[worst] || worst);
  healthBadge.className = 'badge' + (HEALTH_CLS[worst] || '');
  const worstItem = items.filter((s) => lvl(s) === worst)[0];
  healthWorst.textContent = worst === 0 ? 'ok'
    : (worstItem ? worstItem.message : HEALTH_TXT[worst]);
  healthList.innerHTML = '';
  items.forEach((s) => {
    const li = document.createElement('li');
    li.textContent = (HEALTH_TXT[lvl(s)] || '?') + ' — ' + s.message;
    li.className = 'health-item' + (HEALTH_CLS[lvl(s)] || '');
    healthList.appendChild(li);
  });
});

// Gray the badge when the aggregator itself goes silent (>3 s at its 1 Hz).
setInterval(() => {
  if (healthLastMs && performance.now() - healthLastMs > 3000) {
    healthBadge.textContent = 'health —';
    healthBadge.className = 'badge stale';
    healthWorst.textContent = 'no /diagnostics';
  }
}, 1000);

// --- tricks -------------------------------------------------------------------------
const trickState = document.getElementById('trick-state');
const trickButtons = document.querySelectorAll('#tricks button');
trickStatusTopic.subscribe((msg) => {
  // 'idle' or 'name|#RRGGBB|mode' (LED cue riding along for led_status).
  trickState.textContent = msg.data.split('|')[0];
  const busy = msg.data !== 'idle';
  trickButtons.forEach((b) => { b.disabled = busy; });
});
trickButtons.forEach((btn) => {
  btn.addEventListener('click', () => {
    playTrickSrv.callService(
      new ROSLIB.ServiceRequest({ name: btn.dataset.trick }),
      (res) => { trickState.textContent = res.success ? btn.dataset.trick : res.message; },
      (err) => { trickState.textContent = 'error: ' + err; });
  });
});

// --- follow me ------------------------------------------------------------------------
const followState = document.getElementById('follow-state');
const followStartSrv = new ROSLIB.Service({
  ros, name: '/follow_me/start', serviceType: 'std_srvs/srv/Trigger',
});
const followStopSrv = new ROSLIB.Service({
  ros, name: '/follow_me/stop', serviceType: 'std_srvs/srv/Trigger',
});
new ROSLIB.Topic({
  ros, name: '/follow_status', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  // 'idle' | 'searching' | 'locked|<dist m>|<bearing deg>'
  const parts = msg.data.split('|');
  followState.textContent = parts[0] === 'locked'
    ? 'locked ' + parts[1] + ' m @ ' + parts[2] + '°' : parts[0];
});
document.getElementById('follow-start').addEventListener('click', () => {
  followStartSrv.callService(new ROSLIB.ServiceRequest({}),
    (res) => { followState.textContent = res.success ? 'searching' : res.message; },
    (err) => { followState.textContent = 'error: ' + err; });
});
function stopFollow() {
  followStopSrv.callService(new ROSLIB.ServiceRequest({}), () => {}, () => {});
}
document.getElementById('follow-stop').addEventListener('click', stopFollow);

document.getElementById('stop').addEventListener('click', () => {
  stopTrickSrv.callService(new ROSLIB.ServiceRequest({}), () => {}, () => {});
  stopFollow();  // follow_me would keep chasing through the zero burst
  cancelNav();   // a live nav goal would keep driving through the zero burst
  patrolStop();  // a patrol would send the NEXT waypoint after the cancel
  // explore would dispatch the NEXT frontier goal after the cancel; a pause
  // publish with no explore node running is simply dropped.
  exploreResumePub.publish(new ROSLIB.Message({ data: false }));
  zeroBurst();
});

// E-STOP: the latching software e-stop (twist_mux lock + active brake), distinct
// from the one-shot STOP above. Reflects the /estop lock state live.
const estopBtn = document.getElementById('estop');
const estopEngageSrv = new ROSLIB.Service({
  ros, name: '/estop/engage', serviceType: 'std_srvs/srv/Trigger',
});
const estopReleaseSrv = new ROSLIB.Service({
  ros, name: '/estop/release', serviceType: 'std_srvs/srv/Trigger',
});
let estopEngaged = false;
new ROSLIB.Topic({
  ros, name: '/estop', messageType: 'std_msgs/msg/Bool',
}).subscribe((msg) => {
  estopEngaged = msg.data;
  estopBtn.classList.toggle('engaged', estopEngaged);
  estopBtn.textContent = estopEngaged ? 'E-STOP — release' : 'E-STOP';
});
estopBtn.addEventListener('click', () => {
  (estopEngaged ? estopReleaseSrv : estopEngageSrv)
    .callService(new ROSLIB.ServiceRequest({}), () => {}, () => {});
});

// --- collision-monitor bypass (ADR-0016 addendum) --------------------------------------
// Escape hatch for the direction-blind PolygonStop lockout: a plain polygon
// STOP zone zeroes cmd_vel in EVERY direction (even reverse) once tripped, so
// there is no way to drive out without this. Bounded — the node
// auto-releases ~30s after engage regardless of what this button does next.
const cmBypassBtn = document.getElementById('cm-bypass');
const cmBypassEngageSrv = new ROSLIB.Service({
  ros, name: '/collision_monitor/bypass_engage', serviceType: 'std_srvs/srv/Trigger',
});
const cmBypassReleaseSrv = new ROSLIB.Service({
  ros, name: '/collision_monitor/bypass_release', serviceType: 'std_srvs/srv/Trigger',
});
let cmBypassed = false;
new ROSLIB.Topic({
  ros, name: '/collision_monitor/bypassed', messageType: 'std_msgs/msg/Bool',
}).subscribe((msg) => {
  cmBypassed = msg.data;
  cmBypassBtn.classList.toggle('bypassed', cmBypassed);
  cmBypassBtn.textContent = cmBypassed
    ? 'SAFETY BYPASSED — tap to restore' : 'BYPASS COLLISION SAFETY';
});
new ROSLIB.Topic({
  ros, name: '/collision_monitor/zone_mode', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  document.getElementById('cm-zone-mode').textContent = msg.data;
});
cmBypassBtn.addEventListener('click', () => {
  (cmBypassed ? cmBypassReleaseSrv : cmBypassEngageSrv)
    .callService(new ROSLIB.ServiceRequest({}), () => {}, () => {});
});

// --- rosbag record-on-demand (bag_recorder, ADR-0017) ----------------------------------
// State comes from the latched /record/active, so a reloaded page still shows
// a recording started elsewhere (skills MCP, shell). The service response
// carries the bag directory.
const recState = document.getElementById('rec-state');
const recResult = document.getElementById('rec-result');
function recSrv(name) {
  const srv = new ROSLIB.Service({
    ros, name: '/record/' + name, serviceType: 'std_srvs/srv/Trigger',
  });
  return () => srv.callService(new ROSLIB.ServiceRequest({}),
    (res) => { recResult.textContent = res.message; },
    (err) => { recResult.textContent = 'error: ' + err; });
}
document.getElementById('rec-start').addEventListener('click', recSrv('start'));
document.getElementById('rec-stop').addEventListener('click', recSrv('stop'));
new ROSLIB.Topic({
  ros, name: '/record/active', messageType: 'std_msgs/msg/Bool',
}).subscribe((msg) => {
  recState.textContent = msg.data ? '● REC' : 'idle';
  recState.classList.toggle('rec-live', msg.data);
});
new ROSLIB.Topic({
  ros, name: '/record/path', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  if (msg.data) recResult.textContent = msg.data;
});

// --- map + tap-to-navigate ------------------------------------------------------------
// Grid drawn cell-per-pixel on an offscreen canvas, blitted flipped (row 0 of an
// OccupancyGrid is the bottom row in world coords). Robot pose from slam_toolbox's
// /pose, path overlay from /plan. A tap becomes a map-framed /goal_pose — always
// 'map', never the display frame, so the 10 s odom-frame TF trap can't happen.
const mapCanvas = document.getElementById('map-canvas');
const mapCtx = mapCanvas.getContext('2d');
const navState = document.getElementById('nav-state');
const gridCanvas = document.createElement('canvas');
let grid = null;      // latest OccupancyGrid info
let robotPose = null; // {x, y, yaw} in map frame
let plan = null;      // array of {x, y} in map frame

const mapTopic = new ROSLIB.Topic({
  ros, name: '/map', messageType: 'nav_msgs/msg/OccupancyGrid',
  throttle_rate: 2000, queue_length: 1,
});
mapTopic.subscribe((msg) => {
  grid = msg.info;
  const w = msg.info.width, h = msg.info.height;
  gridCanvas.width = w;
  gridCanvas.height = h;
  const img = gridCanvas.getContext('2d').createImageData(w, h);
  for (let i = 0; i < w * h; i++) {
    const v = msg.data[i];
    // unknown: dark gray; free: light; occupied: near-black.
    const c = v < 0 ? 26 : v < 50 ? 210 : 8;
    img.data[4 * i] = c;
    img.data[4 * i + 1] = c;
    img.data[4 * i + 2] = c + (v >= 50 ? 8 : 0);
    img.data[4 * i + 3] = 255;
  }
  gridCanvas.getContext('2d').putImageData(img, 0, 0);
  drawMap();
});

const poseTopic = new ROSLIB.Topic({
  ros, name: '/pose', messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
  throttle_rate: 500,
});
poseTopic.subscribe((msg) => {
  const p = msg.pose.pose;
  robotPose = {
    x: p.position.x, y: p.position.y,
    yaw: 2 * Math.atan2(p.orientation.z, p.orientation.w),
  };
  drawMap();
});

const planTopic = new ROSLIB.Topic({
  ros, name: '/plan', messageType: 'nav_msgs/msg/Path', throttle_rate: 1000,
});
planTopic.subscribe((msg) => {
  plan = msg.poses.map((ps) => ({ x: ps.pose.position.x, y: ps.pose.position.y }));
});

// Persistent under-lidar clutter (chair bases, shoes) from clutter_mapper,
// drawn as an orange overlay so low obstacles are visible in the map.
const clutterCanvas = document.createElement('canvas');
let clutter = null;   // latest /clutter_map info
const clutterTopic = new ROSLIB.Topic({
  ros, name: '/clutter_map', messageType: 'nav_msgs/msg/OccupancyGrid',
  throttle_rate: 2000, queue_length: 1,
});
clutterTopic.subscribe((msg) => {
  clutter = msg.info;
  const w = msg.info.width, h = msg.info.height;
  clutterCanvas.width = w;
  clutterCanvas.height = h;
  const img = clutterCanvas.getContext('2d').createImageData(w, h);
  for (let i = 0; i < w * h; i++) {
    if (msg.data[i] > 50) {
      img.data[4 * i] = 255;
      img.data[4 * i + 1] = 140;
      img.data[4 * i + 2] = 0;
      img.data[4 * i + 3] = 230;
    }
  }
  clutterCanvas.getContext('2d').putImageData(img, 0, 0);
  drawMap();
});

function worldToCanvas(wx, wy) {
  // map origin is the world coord of cell (0,0); canvas y is flipped.
  const sx = mapCanvas.width / grid.width;
  const sy = mapCanvas.height / grid.height;
  return {
    x: (wx - grid.origin.position.x) / grid.resolution * sx,
    y: mapCanvas.height - (wy - grid.origin.position.y) / grid.resolution * sy,
  };
}

function drawMap() {
  if (!grid) return;
  navState.textContent = robotPose ? 'live' : 'map only';
  // Fit the canvas backing store to the grid aspect (CSS scales to width).
  const aspect = grid.height / grid.width;
  const W = 960;  // backing store; CSS scales to fit the stage
  if (mapCanvas.width !== W || mapCanvas.height !== Math.round(W * aspect)) {
    mapCanvas.width = W;
    mapCanvas.height = Math.round(W * aspect);
    mapCanvas.style.aspectRatio = W + ' / ' + mapCanvas.height;
  }
  mapCtx.imageSmoothingEnabled = false;
  mapCtx.save();
  // Flip vertically so world +y is up.
  mapCtx.scale(1, -1);
  mapCtx.drawImage(gridCanvas, 0, -mapCanvas.height, mapCanvas.width, mapCanvas.height);
  if (clutter && clutter.width > 1) {
    // Place the clutter grid in the same flipped world coords as the map.
    const ppmX = mapCanvas.width / (grid.width * grid.resolution);
    const ppmY = mapCanvas.height / (grid.height * grid.resolution);
    const xPx = (clutter.origin.position.x - grid.origin.position.x) * ppmX;
    const yPx = (clutter.origin.position.y - grid.origin.position.y) * ppmY;
    const wPx = clutter.width * clutter.resolution * ppmX;
    const hPx = clutter.height * clutter.resolution * ppmY;
    mapCtx.drawImage(clutterCanvas, xPx, -(yPx + hPx), wPx, hPx);
  }
  mapCtx.restore();

  if (plan && plan.length > 1) {
    mapCtx.strokeStyle = '#00c878';
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    plan.forEach((p, i) => {
      const c = worldToCanvas(p.x, p.y);
      i ? mapCtx.lineTo(c.x, c.y) : mapCtx.moveTo(c.x, c.y);
    });
    mapCtx.stroke();
  }

  if (patrolRoute.length) {
    mapCtx.strokeStyle = 'rgba(64, 160, 255, 0.45)';
    mapCtx.lineWidth = 1.5;
    mapCtx.beginPath();
    patrolRoute.forEach((p, i) => {
      const c = worldToCanvas(p.x, p.y);
      i ? mapCtx.lineTo(c.x, c.y) : mapCtx.moveTo(c.x, c.y);
    });
    mapCtx.stroke();
    patrolRoute.forEach((p, i) => {
      const c = worldToCanvas(p.x, p.y);
      const wp = i + 1;   // status index is 1-based
      let color = '#5a6a7a';                       // pending
      let r = 3;
      if (patrolProg.active && wp < patrolProg.i) color = '#00c878';   // visited
      if (patrolProg.active && wp === patrolProg.i) {                  // current
        color = '#ffa028';
        r = 5;
      }
      mapCtx.fillStyle = color;
      mapCtx.beginPath();
      mapCtx.arc(c.x, c.y, r, 0, 2 * Math.PI);
      mapCtx.fill();
    });
  }

  if (frontierPts.length || frontierGoals.length) {
    mapCtx.fillStyle = 'rgba(0, 220, 255, 0.55)';
    frontierPts.forEach((p) => {
      const c = worldToCanvas(p.x, p.y);
      mapCtx.fillRect(c.x - 1, c.y - 1, 2, 2);
    });
    mapCtx.strokeStyle = '#00dcff';
    mapCtx.lineWidth = 1.5;
    frontierGoals.forEach((p) => {
      const c = worldToCanvas(p.x, p.y);
      mapCtx.beginPath();
      mapCtx.arc(c.x, c.y, 5, 0, 2 * Math.PI);
      mapCtx.stroke();
    });
  }

  if (robotPose) {
    const c = worldToCanvas(robotPose.x, robotPose.y);
    mapCtx.save();
    mapCtx.translate(c.x, c.y);
    mapCtx.rotate(-robotPose.yaw);   // canvas y is flipped, so negate yaw
    mapCtx.fillStyle = '#ff4040';
    mapCtx.beginPath();
    mapCtx.moveTo(10, 0);
    mapCtx.lineTo(-6, 6);
    mapCtx.lineTo(-6, -6);
    mapCtx.closePath();
    mapCtx.fill();
    mapCtx.restore();
  }

  // Saved zones (latched /zones, ADR-0019): keepout red, speed amber.
  Object.entries(zoneList).forEach(([name, zn]) => {
    const col = zn.type === 'keepout' ? '255,64,64' : '255,180,40';
    mapCtx.fillStyle = 'rgba(' + col + ',0.22)';
    mapCtx.strokeStyle = 'rgba(' + col + ',0.8)';
    mapCtx.lineWidth = 1.5;
    mapCtx.beginPath();
    zn.polygon.forEach(([x, y], i) => {
      const c = worldToCanvas(x, y);
      i ? mapCtx.lineTo(c.x, c.y) : mapCtx.moveTo(c.x, c.y);
    });
    mapCtx.closePath();
    mapCtx.fill();
    mapCtx.stroke();
    const c0 = worldToCanvas(zn.polygon[0][0], zn.polygon[0][1]);
    mapCtx.fillStyle = 'rgba(' + col + ',0.9)';
    mapCtx.font = '11px sans-serif';
    mapCtx.fillText(zn.type === 'speed' ? name + ' ' + zn.speed_pct + '%' : name,
      c0.x + 4, c0.y - 4);
  });

  // Zone being drawn right now.
  if (zonePts.length) {
    const col = zoneMode === 'keepout' ? '#ff4040' : '#ffb428';
    mapCtx.strokeStyle = col;
    mapCtx.fillStyle = col;
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    zonePts.forEach((p, i) => {
      const c = worldToCanvas(p.x, p.y);
      i ? mapCtx.lineTo(c.x, c.y) : mapCtx.moveTo(c.x, c.y);
    });
    mapCtx.stroke();
    zonePts.forEach((p) => {
      const c = worldToCanvas(p.x, p.y);
      mapCtx.beginPath();
      mapCtx.arc(c.x, c.y, 4, 0, 2 * Math.PI);
      mapCtx.fill();
    });
  }

  if (areaPts.length) {
    mapCtx.strokeStyle = '#40a0ff';
    mapCtx.fillStyle = '#40a0ff';
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    areaPts.forEach((p, i) => {
      const c = worldToCanvas(p.x, p.y);
      i ? mapCtx.lineTo(c.x, c.y) : mapCtx.moveTo(c.x, c.y);
    });
    mapCtx.stroke();
    if (areaPts.length > 2) {
      const first = worldToCanvas(areaPts[0].x, areaPts[0].y);
      const last = worldToCanvas(areaPts[areaPts.length - 1].x,
        areaPts[areaPts.length - 1].y);
      mapCtx.setLineDash([6, 4]);
      mapCtx.beginPath();
      mapCtx.moveTo(last.x, last.y);
      mapCtx.lineTo(first.x, first.y);
      mapCtx.stroke();
      mapCtx.setLineDash([]);
    }
    areaPts.forEach((p) => {
      const c = worldToCanvas(p.x, p.y);
      mapCtx.beginPath();
      mapCtx.arc(c.x, c.y, 4, 0, 2 * Math.PI);
      mapCtx.fill();
    });
  }
}

const goalPub = new ROSLIB.Topic({
  ros, name: '/goal_pose', messageType: 'geometry_msgs/msg/PoseStamped',
});
mapCanvas.addEventListener('click', (ev) => {
  if (!grid) return;
  if (areaMode) {
    areaPts.push(canvasToWorld(ev));
    areaBtn.textContent = 'Finish (' + areaPts.length + ')';
    drawMap();
    return;
  }
  if (zoneMode) {
    zonePts.push(canvasToWorld(ev));
    zoneBtn(zoneMode).textContent = 'Finish (' + zonePts.length + ')';
    drawMap();
    return;
  }
  const r = mapCanvas.getBoundingClientRect();
  const cx = (ev.clientX - r.left) * (mapCanvas.width / r.width);
  const cy = (ev.clientY - r.top) * (mapCanvas.height / r.height);
  const wx = cx / (mapCanvas.width / grid.width) * grid.resolution
    + grid.origin.position.x;
  const wy = (mapCanvas.height - cy) / (mapCanvas.height / grid.height)
    * grid.resolution + grid.origin.position.y;
  // Face the direction of travel (or map-x if pose unknown).
  const yaw = robotPose ? Math.atan2(wy - robotPose.y, wx - robotPose.x) : 0;
  goalPub.publish(new ROSLIB.Message({
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
    pose: {
      position: { x: wx, y: wy, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
    },
  }));
  navState.textContent = 'goal sent (' + wx.toFixed(2) + ', ' + wy.toFixed(2) + ')';
});

// Cancel = nav_manager's dispatcher-aware /nav/cancel (ADR-0018): stops
// patrol, pauses explore, then zero-uuid cancels BOTH bt_navigator actions —
// a bare action cancel would be re-overridden by patrol/explore within ~1 s.
// Robot stays drivable (unlike E-STOP).
const cancelNavSrv = new ROSLIB.Service({
  ros, name: '/nav/cancel', serviceType: 'std_srvs/srv/Trigger',
});
function cancelNav() {
  cancelNavSrv.callService(new ROSLIB.ServiceRequest({}),
    (res) => { navState.textContent = res.message; },
    (err) => { navState.textContent = 'cancel failed: ' + err; });
}
document.getElementById('cancel-goal').addEventListener('click', cancelNav);

// Nav readout from nav_manager's consolidated /nav_state — covers BOTH
// actions (taps, routes, patrols). Latched, so a reloaded page catches up.
// Grammar (core.status, SC9): 'idle' | '<status>|<dist 2dp or empty>|<recoveries>'.
new ROSLIB.Topic({
  ros, name: '/nav_state', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  const [state, dist, recov] = msg.data.split('|');
  let text = state;
  if (dist) text += ' — ' + dist + ' m left';
  if (recov && recov !== '0') text += ' (' + recov + ' recoveries)';
  navState.textContent = text;
});

// --- patrol ---------------------------------------------------------------------------
const patrolState = document.getElementById('patrol-state');
const patrolResult = document.getElementById('patrol-result');
function patrolSrv(name) {
  const srv = new ROSLIB.Service({
    ros, name: '/patrol/' + name, serviceType: 'std_srvs/srv/Trigger',
  });
  return () => srv.callService(new ROSLIB.ServiceRequest({}),
    (res) => { patrolResult.textContent = res.message; },
    (err) => { patrolResult.textContent = 'error: ' + err; });
}
const patrolStop = patrolSrv('stop');
const patrolMark = patrolSrv('mark');   // also fired by gamepad A (readGamepad)
document.getElementById('patrol-mark').addEventListener('click', patrolMark);
document.getElementById('patrol-clear').addEventListener('click', patrolSrv('clear'));
document.getElementById('patrol-start').addEventListener('click', patrolSrv('start'));
document.getElementById('patrol-stop').addEventListener('click', patrolStop);
let patrolRoute = [];   // [{x, y}] map-frame waypoints, drawn over the map
let patrolProg = { active: false, i: 0, n: 0 };
const patrolBar = document.getElementById('patrol-bar');
const patrolBarFill = document.getElementById('patrol-bar-fill');

new ROSLIB.Topic({
  ros, name: '/patrol_status', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  // 'idle|<n>' | '<state>|<n>|<i>/<n>' | 'plan|<coverage feedback text>'
  const parts = msg.data.split('|');
  if (parts[0] === 'plan') {
    patrolResult.textContent = parts.slice(1).join('|');
    return;
  }
  const wasActive = patrolProg.active;
  if (parts[0] === 'idle') {
    patrolProg = { active: false, i: 0, n: parseInt(parts[1], 10) || 0 };
    patrolState.textContent = 'idle · ' + parts[1] + ' wp';
    patrolBar.style.display = 'none';
  } else {
    const [i, n] = (parts[2] || '0/0').split('/').map((v) => parseInt(v, 10));
    patrolProg = { active: true, i: i || 0, n: n || 0 };
    patrolState.textContent = parts[0] + ' ' + parts[2];
    patrolBar.style.display = 'block';
    patrolBarFill.style.width =
      (patrolProg.n ? Math.round(100 * (patrolProg.i - 1) / patrolProg.n) : 0) + '%';
  }
  if (wasActive !== patrolProg.active) drawMap();
});

new ROSLIB.Topic({
  ros, name: '/patrol_route', messageType: 'nav_msgs/msg/Path',
  throttle_rate: 2000, queue_length: 1,
}).subscribe((msg) => {
  const pts = msg.poses.map((ps) => ({
    x: ps.pose.position.x, y: ps.pose.position.y,
  }));
  if (JSON.stringify(pts) !== JSON.stringify(patrolRoute)) {
    patrolRoute = pts;
    drawMap();
  }
});

// --- coverage area select (click-to-place polygon) --------------------------------
// 'Select area' arms point placement: each map tap adds a vertex, the outline
// draws live, and the button (now 'Finish (N)') closes the polygon and sends
// it to /coverage_box (map frame). patrol_capture replies on /patrol_status
// with 'plan|...'. Click-to-place instead of drag: touch drags scroll the page.
const coverageBoxPub = new ROSLIB.Topic({
  ros, name: '/coverage_box', messageType: 'geometry_msgs/msg/PolygonStamped',
});
// Advertise at load: a publish on a just-advertised topic races DDS discovery
// and is silently dropped — the first Finish press would vanish.
ros.on('connection', () => coverageBoxPub.advertise());
const areaBtn = document.getElementById('patrol-area');
let areaMode = false;
let areaPts = [];   // [{x, y}] map-frame vertices

function canvasToWorld(ev) {
  const r = mapCanvas.getBoundingClientRect();
  const cx = (ev.clientX - r.left) * (mapCanvas.width / r.width);
  const cy = (ev.clientY - r.top) * (mapCanvas.height / r.height);
  return {
    x: cx / (mapCanvas.width / grid.width) * grid.resolution
      + grid.origin.position.x,
    y: (mapCanvas.height - cy) / (mapCanvas.height / grid.height)
      * grid.resolution + grid.origin.position.y,
  };
}

function areaReset(msg) {
  areaMode = false;
  areaPts = [];
  areaBtn.textContent = 'Select area';
  areaBtn.classList.remove('selected');
  if (msg) patrolResult.textContent = msg;
  drawMap();
}

areaBtn.addEventListener('click', () => {
  if (!areaMode) {
    areaMode = true;
    areaPts = [];
    areaBtn.textContent = 'Finish (0)';
    areaBtn.classList.add('selected');
    patrolResult.textContent =
      'Tap the map to outline the area (3+ points), then press Finish.';
    return;
  }
  // Finish pressed
  if (areaPts.length < 3) {
    areaReset('Area select cancelled (needs 3+ points).');
    return;
  }
  coverageBoxPub.publish(new ROSLIB.Message({
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
    polygon: { points: areaPts.map((p) => ({ x: p.x, y: p.y, z: 0 })) },
  }));
  areaReset('Planning coverage…');
});

// --- keepout/speed zones (zone_manager, ADR-0019) ---------------------------------------
// Same draw interaction as the coverage area; the polygon travels as a
// |-grammar /zone_cmd string (frozen in scout.core.zones + test_zones.py):
//   add|<type>|<speed_pct or empty>|x,y;x,y;...  |  delete|<name>  |  clear|
// Saved zones come back on the latched /zones as JSON (the store schema).
const zoneCmdPub = new ROSLIB.Topic({
  ros, name: '/zone_cmd', messageType: 'std_msgs/msg/String',
});
ros.on('connection', () => zoneCmdPub.advertise());
const zoneStateEl = document.getElementById('zone-state');
const zoneResult = document.getElementById('zone-result');
const zoneListEl = document.getElementById('zone-list');
const zoneSpeedPct = document.getElementById('zone-speed-pct');
zoneSpeedPct.addEventListener('input', () => {
  document.getElementById('zone-speed-out').textContent = zoneSpeedPct.value;
});
let zoneMode = null;   // 'keepout' | 'speed' | null
let zonePts = [];
let zoneList = {};     // {name: {type, polygon, speed_pct?}} from /zones

function zoneBtn(mode) {
  return document.getElementById(mode === 'keepout' ? 'zone-keepout' : 'zone-speed');
}
function zoneReset(msg) {
  if (zoneMode) zoneBtn(zoneMode).classList.remove('selected');
  document.getElementById('zone-keepout').textContent = 'Draw keepout';
  document.getElementById('zone-speed').textContent = 'Draw speed zone';
  zoneMode = null;
  zonePts = [];
  if (msg) zoneResult.textContent = msg;
  drawMap();
}
function zoneDraw(mode) {
  if (zoneMode === mode) {   // Finish pressed
    if (zonePts.length < 3) {
      zoneReset('Zone cancelled (needs 3+ points).');
      return;
    }
    const pts = zonePts.map((p) => p.x.toFixed(3) + ',' + p.y.toFixed(3)).join(';');
    const spd = mode === 'speed' ? zoneSpeedPct.value : '';
    zoneCmdPub.publish(new ROSLIB.Message({
      data: 'add|' + mode + '|' + spd + '|' + pts,
    }));
    zoneReset('Zone sent. First-ever zone needs a nav2 restart; edits after that apply live.');
    return;
  }
  zoneReset();
  zoneMode = mode;
  zoneBtn(mode).textContent = 'Finish (0)';
  zoneBtn(mode).classList.add('selected');
  zoneResult.textContent = 'Tap the map to outline the '
    + (mode === 'speed' ? zoneSpeedPct.value + '% speed' : 'keepout')
    + ' zone (3+ points), then press Finish.';
}
document.getElementById('zone-keepout').addEventListener('click', () => zoneDraw('keepout'));
document.getElementById('zone-speed').addEventListener('click', () => zoneDraw('speed'));

new ROSLIB.Topic({
  ros, name: '/zones', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  try { zoneList = JSON.parse(msg.data) || {}; } catch (e) { zoneList = {}; }
  const names = Object.keys(zoneList).sort();
  zoneStateEl.textContent = names.length ? names.length + ' zone(s)' : 'none';
  zoneListEl.innerHTML = '';
  names.forEach((name) => {
    const zn = zoneList[name];
    const li = document.createElement('li');
    li.textContent = name + (zn.type === 'speed' ? ' (' + zn.speed_pct + '%) ' : ' ');
    const del = document.createElement('button');
    del.textContent = '✕';
    del.addEventListener('click', () => {
      zoneCmdPub.publish(new ROSLIB.Message({ data: 'delete|' + name }));
    });
    li.appendChild(del);
    zoneListEl.appendChild(li);
  });
  drawMap();
});

// --- explore (explore_lite, compose profile `explore`) ---------------------------------
// Pause/resume ride the /explore/resume Bool the skills server also uses;
// deliberately NOT container start/stop. Frontier markers (/explore/frontiers,
// published while the node plans) draw cyan on the map canvas. Markers go
// stale when paused/stopped, so they are cleared after 10 s of silence.
const exploreState = document.getElementById('explore-state');
const exploreResult = document.getElementById('explore-result');
const exploreResumePub = new ROSLIB.Topic({
  ros, name: '/explore/resume', messageType: 'std_msgs/msg/Bool',
});
// Advertise at load: same DDS-discovery race as /coverage_box.
ros.on('connection', () => exploreResumePub.advertise());
let frontierPts = [];       // [{x, y}] frontier cells (POINTS markers)
let frontierGoals = [];     // [{x, y}] centroid/goal spheres
let frontierLastMs = 0;

function setExplore(active) {
  exploreResumePub.publish(new ROSLIB.Message({ data: active }));
  exploreResult.textContent = active
    ? 'resume sent — frontiers should update within a few seconds'
    : 'pause sent — current drive continues (Cancel Goal / STOP to halt)';
}
document.getElementById('explore-resume')
  .addEventListener('click', () => setExplore(true));
document.getElementById('explore-pause')
  .addEventListener('click', () => setExplore(false));

new ROSLIB.Topic({
  ros, name: '/explore/frontiers', messageType: 'visualization_msgs/msg/MarkerArray',
  throttle_rate: 1000, queue_length: 1,
}).subscribe((msg) => {
  frontierLastMs = performance.now();
  const pts = [];
  const goals = [];
  msg.markers.forEach((m) => {
    if (m.action !== 0) return;   // ADD/MODIFY only
    if (m.points && m.points.length) {
      m.points.forEach((p) => pts.push({ x: p.x, y: p.y }));
    } else {
      goals.push({ x: m.pose.position.x, y: m.pose.position.y });
    }
  });
  frontierPts = pts;
  frontierGoals = goals;
  exploreState.textContent = goals.length + ' frontiers · live';
  drawMap();
});

setInterval(() => {
  if (frontierLastMs && performance.now() - frontierLastMs > 10000) {
    frontierLastMs = 0;
    frontierPts = [];
    frontierGoals = [];
    exploreState.textContent = 'quiet';
    drawMap();
  }
}, 2000);

// --- camera view ----------------------------------------------------------------------
// Subscribe ONLY while the panel is shown: compressed_image_transport JPEG-encodes
// per-subscriber, so a hidden panel costs the robot zero CPU. rosbridge JSON
// delivers the JPEG bytes base64-encoded — straight into a data URL.
const camImg = document.getElementById('cam-img');
const camState = document.getElementById('cam-state');
const camToggle = document.getElementById('cam-toggle');
let camTopic = null;

function camStart() {
  if (camTopic) return;
  camTopic = new ROSLIB.Topic({
    ros, name: '/camera/camera/color/image_raw/compressed',
    messageType: 'sensor_msgs/msg/CompressedImage',
    throttle_rate: 250, queue_length: 1,   // 4 Hz on the wire, ~1-2 Mbps
  });
  camTopic.subscribe((msg) => {
    camImg.src = 'data:image/jpeg;base64,' + msg.data;
    camImg.style.display = 'block';   // only once real pixels exist
    camState.textContent = 'live';
  });
  camState.textContent = 'waiting…';
  camToggle.textContent = 'Hide camera';
}
function camStop() {
  if (!camTopic) return;
  camTopic.unsubscribe();
  camTopic = null;
  camImg.style.display = 'none';
  camImg.removeAttribute('src');
  camState.textContent = 'off';
  camToggle.textContent = 'Show camera';
}
camToggle.addEventListener('click', () => (camTopic ? camStop() : camStart()));
// Coexists with the zeroBurst visibility hook: hidden tab = stop streaming.
document.addEventListener('visibilitychange', () => {
  if (document.hidden) camStop();
});
ros.on('close', camStop);   // resubscribe manually after a reconnect

// --- lights -------------------------------------------------------------------------
const ledResult = document.getElementById('led-result');
const ledBright = document.getElementById('led-bright');
const ledSpeed = document.getElementById('led-speed');
ledBright.oninput = () => document.getElementById('led-bright-out').value = ledBright.value;
ledSpeed.oninput = () => document.getElementById('led-speed-out').value = ledSpeed.value;

let ledMode = null;
document.querySelectorAll('#led-modes button').forEach((btn) => {
  btn.addEventListener('click', () => {
    ledMode = btn.dataset.mode;
    document.querySelectorAll('#led-modes button').forEach(
      (b) => b.classList.toggle('selected', b === btn));
    sendLed();
  });
});
document.getElementById('led-color').addEventListener('change', sendLed);
ledBright.addEventListener('change', sendLed);
ledSpeed.addEventListener('change', sendLed);

function sendLed() {
  if (!ledMode) return;
  setUserLedSrv.callService(new ROSLIB.ServiceRequest({
    mode: ledMode,
    color: document.getElementById('led-color').value,
    brightness: parseInt(ledBright.value, 10),
    speed: parseFloat(ledSpeed.value),
  }),
  (res) => { ledResult.textContent = res.message; },
  (err) => { ledResult.textContent = 'error: ' + err; });
}
