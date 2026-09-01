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
let CAM_TOPIC = '/camera/camera/color/image_raw/compressed';  // topic_camera_color fallback
let OCC_THRESHOLD = 50;  // occupied_threshold fallback (nav2 lethal convention)
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
    if (prof.topic_camera_color) CAM_TOPIC = prof.topic_camera_color;
    if (prof.occupied_threshold) OCC_THRESHOLD = prof.occupied_threshold;
    if (Array.isArray(prof.led_modes)) {
      // The SSOT owns the mode list; the static index.html buttons are only
      // the fetch-failed fallback (the exact drift the profile was made for).
      const wrap = document.getElementById('led-modes');
      wrap.innerHTML = prof.led_modes.map((m) =>
        `<button data-mode="${m}">${m.charAt(0).toUpperCase() + m.slice(1)}</button>`
      ).join('');
      bindLedModeButtons();
    }
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

// --- RoboClaw panel: /roboclaw_status ---------------------------------------------
// The driver's JSON String (schema driver-owned; core/status.py owns only the
// envelope). The driver polls the board over serial regardless of cmd_vel, so a
// stale/absent status means the SERIAL LINK is dead (board unpowered, UART fault)
// — the exact failure that otherwise only shows up as "joystick does nothing".
const rcState = document.getElementById('rc-state');
const rcVitals = document.getElementById('rc-vitals');
let rcLastMs = 0;
let rcLastStatus = null;

function rcRow(label, value, cls) {
  return `<span class="${cls || ''}">${label} ${value}</span>`;
}

function renderRoboclaw() {
  const s = rcLastStatus;
  const mainV = Number(s.main_battery);
  // Ladder anchored on the RoboClaw's own 16.0 V Min Main cutoff.
  const vCls = mainV <= 16.5 ? 'bad' : mainV <= 17.5 ? 'warn' : '';
  const t1 = Number(s.temperature1), t2 = Number(s.temperature2);
  const tCls = Math.max(t1, t2) >= 75 ? 'bad' : Math.max(t1, t2) >= 60 ? 'warn' : '';
  const errNum = Number(s.error_status);
  const errCls = errNum ? 'bad' : '';
  rcState.textContent = `${mainV.toFixed(1)} V`;
  rcState.className = 'badge' + (errNum || vCls === 'bad' ? ' batt-bad'
    : vCls === 'warn' ? ' batt-warn' : ' connected');
  rcVitals.innerHTML = [
    rcRow('Main', mainV.toFixed(2) + ' V', vCls),
    rcRow('Logic', Number(s.logic_battery).toFixed(2) + ' V'),
    rcRow('M1', Number(s.m1_current).toFixed(2) + ' A'),
    rcRow('M2', Number(s.m2_current).toFixed(2) + ' A'),
    rcRow('Temp', `${t1.toFixed(0)}/${t2.toFixed(0)} °C`, tCls),
    rcRow('Speed', `${s.m1_speed}/${s.m2_speed} c/s`),
    rcRow('Enc', `${s.m1_enc_value}/${s.m2_enc_value}`),
    rcRow('Err', errNum ? (s.decoded_error_status || '0x' + errNum.toString(16)) : 'none', errCls),
  ].join(' ');
}

new ROSLIB.Topic({
  ros, name: '/roboclaw_status', messageType: 'std_msgs/msg/String',
  throttle_rate: 1000, queue_length: 1,
}).subscribe((msg) => {
  let s;
  try { s = JSON.parse(msg.data); } catch (e) { return; }
  if (!s || typeof s !== 'object') return;
  rcLastMs = performance.now();
  rcLastStatus = s;
  renderRoboclaw();
});

// Driver publishes ~30 Hz (throttled to 1 Hz here); >3 s silent = serial link
// down or driver dead. Distinguish "never heard" (boot/board dark) from "went
// quiet" so today's dead-board case reads NO LINK at a glance.
setInterval(() => {
  if (!rcLastMs) return; // initial "no data" badge stands until first message
  const age = performance.now() - rcLastMs;
  if (age > 3000) {
    rcState.textContent = `NO LINK ${Math.round(age / 1000)}s`;
    rcState.className = 'badge disconnected';
  }
}, 1000);

document.getElementById('stop').addEventListener('click', () => {
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
// /pose while mapping, amcl's /amcl_pose in localization mode (ADR-0028 — same msg
// type; amcl republishes only after update_min_d/a of motion, so the arrow steps).
// Path overlay from /plan. A tap becomes a map-framed /goal_pose — always
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
    // unknown: dark gray; free: light; occupied: near-black. The split is the
    // profile's occupied_threshold (shared with render.py).
    const c = v < 0 ? 26 : v < OCC_THRESHOLD ? 210 : 8;
    img.data[4 * i] = c;
    img.data[4 * i + 1] = c;
    img.data[4 * i + 2] = c + (v >= OCC_THRESHOLD ? 8 : 0);
    img.data[4 * i + 3] = 255;
  }
  gridCanvas.getContext('2d').putImageData(img, 0, 0);
  drawMap();
});

function onPose(msg) {
  const p = msg.pose.pose;
  robotPose = {
    x: p.position.x, y: p.position.y,
    yaw: 2 * Math.atan2(p.orientation.z, p.orientation.w),
  };
  drawMap();
}
// Only one of these publishes per session (slam vs localization mode).
['/pose', '/amcl_pose'].forEach((name) => {
  new ROSLIB.Topic({
    ros, name, messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    throttle_rate: 500,
  }).subscribe(onPose);
});

const planTopic = new ROSLIB.Topic({
  ros, name: '/plan', messageType: 'nav_msgs/msg/Path', throttle_rate: 1000,
});
planTopic.subscribe((msg) => {
  plan = msg.poses.map((ps) => ({ x: ps.pose.position.x, y: ps.pose.position.y }));
  // Redraw now: in localization mode /map is latched (no periodic republish) and
  // /amcl_pose is sparse, so without this the plan waits for the next pose update.
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
    ros, name: CAM_TOPIC,
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
function bindLedModeButtons() {
  document.querySelectorAll('#led-modes button').forEach((btn) => {
    btn.addEventListener('click', () => {
      ledMode = btn.dataset.mode;
      document.querySelectorAll('#led-modes button').forEach(
        (b) => b.classList.toggle('selected', b === btn));
      sendLed();
    });
  });
}
bindLedModeButtons();
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

// --- RFID (Flipper Zero) -----------------------------------------------------------
// The manual gate (ADR-0025): flipper_node never scans until a human enables
// it here (/flipper/rfid_enable, SetBool). The badge renders from the node's
// latched /flipper/status — never from this page's own assumption — and the
// read list replays from /rfid/reads' latched depth-50 window on page load.
const rfidState = document.getElementById('rfid-state');
const rfidList = document.getElementById('rfid-list');
const rfidResult = document.getElementById('rfid-result');
const rfidEnableSrv = new ROSLIB.Service({
  ros, name: '/flipper/rfid_enable', serviceType: 'std_srvs/srv/SetBool',
});

function rfidSetEnabled(on) {
  rfidEnableSrv.callService(new ROSLIB.ServiceRequest({ data: on }),
    (res) => { rfidResult.textContent = res.message; },
    (err) => { rfidResult.textContent = 'error: ' + err; });
}
document.getElementById('rfid-enable').addEventListener('click', () => rfidSetEnabled(true));
document.getElementById('rfid-disable').addEventListener('click', () => rfidSetEnabled(false));

new ROSLIB.Topic({
  ros, name: '/flipper/status', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  let s;
  try { s = JSON.parse(msg.data); } catch (e) { return; }
  rfidState.textContent = !s.connected ? 'no flipper'
    : s.rfid_enabled ? 'scanning' : 'disabled';
  rfidState.classList.toggle('bad', !s.connected);
});

const rfidSeen = new Set();   // read_ids already rendered (latch replays)
new ROSLIB.Topic({
  ros, name: '/rfid/reads', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  let r;
  try { r = JSON.parse(msg.data); } catch (e) { return; }
  if (rfidSeen.has(r.read_id)) return;
  rfidSeen.add(r.read_id);
  const li = document.createElement('li');
  const where = r.pose ? `(${r.pose.x.toFixed(2)}, ${r.pose.y.toFixed(2)})` : 'no map pose';
  const when = (r.stamp_utc || '').replace(/^.*T/, '').replace(/\..*$/, '');
  li.textContent = `${r.protocol} ${r.data_hex} · ${where} · ${when}`;
  rfidList.prepend(li);
  while (rfidList.children.length > 20) rfidList.removeChild(rfidList.lastChild);
});

// --- NFC (Flipper Zero) ------------------------------------------------------------
// Mirror of the RFID panel for the 13.56 MHz radio (ADR-0026). Same manual
// gate (/flipper/nfc_enable, SetBool); the badge renders from the node's
// latched /flipper/status (nfc_enabled) and the list replays /nfc/reads. NFC
// and RFID are mutually exclusive on the one serial line — enabling one while
// the other is on is rejected by the node (shown in the result line).
const nfcState = document.getElementById('nfc-state');
const nfcList = document.getElementById('nfc-list');
const nfcResult = document.getElementById('nfc-result');
const nfcEnableSrv = new ROSLIB.Service({
  ros, name: '/flipper/nfc_enable', serviceType: 'std_srvs/srv/SetBool',
});

function nfcSetEnabled(on) {
  nfcEnableSrv.callService(new ROSLIB.ServiceRequest({ data: on }),
    (res) => { nfcResult.textContent = res.message; },
    (err) => { nfcResult.textContent = 'error: ' + err; });
}
document.getElementById('nfc-enable').addEventListener('click', () => nfcSetEnabled(true));
document.getElementById('nfc-disable').addEventListener('click', () => nfcSetEnabled(false));

new ROSLIB.Topic({
  ros, name: '/flipper/status', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  let s;
  try { s = JSON.parse(msg.data); } catch (e) { return; }
  nfcState.textContent = !s.connected ? 'no flipper'
    : s.nfc_enabled ? 'scanning' : 'disabled';
  nfcState.classList.toggle('bad', !s.connected);
});

const nfcSeen = new Set();   // read_ids already rendered (latch replays)
new ROSLIB.Topic({
  ros, name: '/nfc/reads', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  let r;
  try { r = JSON.parse(msg.data); } catch (e) { return; }
  if (nfcSeen.has(r.read_id)) return;
  nfcSeen.add(r.read_id);
  const li = document.createElement('li');
  const where = r.pose ? `(${r.pose.x.toFixed(2)}, ${r.pose.y.toFixed(2)})` : 'no map pose';
  const when = (r.stamp_utc || '').replace(/^.*T/, '').replace(/\..*$/, '');
  li.textContent = `${r.protocol} ${r.data_hex} · ${where} · ${when}`;
  nfcList.prepend(li);
  while (nfcList.children.length > 20) nfcList.removeChild(nfcList.lastChild);
});

// --- system panel: host vitals + per-container controls ---------------------------
// fleet_status (docker/fleet-status) is a standalone REST backend, not a ROS node —
// it holds the Docker socket, so it runs outside rosbridge entirely. Polls every 30s;
// any action that can touch the drivetrain (stop/restart/restart-all) gets a confirm()
// first, same as every other motion-adjacent control in this app.
const FLEET_API = 'http://' + location.hostname + ':9003/api';
const sysState = document.getElementById('sys-state');
const sysVitals = document.getElementById('sys-vitals');
const sysContainers = document.getElementById('sys-containers');
const sysRestartAll = document.getElementById('sys-restart-all');

function fmtUptime(s) {
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
  return h > 0 ? `${h}h${m}m` : `${m}m`;
}

function renderVitals(v, el = sysVitals) {
  const tempClass = v.temp_c == null ? '' : v.temp_c >= 80 ? 'bad' : v.temp_c >= 70 ? 'warn' : '';
  const cpuClass = v.cpu_percent >= 95 ? 'bad' : v.cpu_percent >= 85 ? 'warn' : '';
  el.innerHTML = `
    <span class="${cpuClass}">CPU ${v.cpu_percent}%</span>
    <span>Load ${v.load_avg.map((n) => n.toFixed(1)).join('/')}</span>
    <span>Mem ${(v.mem_used_mb / 1024).toFixed(1)}/${(v.mem_total_mb / 1024).toFixed(1)} GB</span>
    ${v.temp_c != null ? `<span class="${tempClass}">${v.temp_c}°C</span>` : ''}
    ${v.disk_used_gb != null ? `<span>Disk ${v.disk_used_gb}/${v.disk_total_gb} GB</span>` : ''}
    <span>Up ${fmtUptime(v.uptime_s)}</span>
  `;
}

function renderContainers(list) {
  sysContainers.innerHTML = '';
  for (const c of list) {
    const row = document.createElement('div');
    row.className = 'svc-row';
    const running = c.status === 'running';
    row.innerHTML = `
      <span class="svc-dot ${c.status}"></span>
      <span class="svc-name">${c.service}</span>
      <span class="svc-stat">${running ? `${c.cpu_percent}% · ${c.mem_mb}MB` : c.status}</span>
      <span class="svc-actions">
        <button data-act="stop" ${!running || c.self ? 'disabled' : ''}>Stop</button>
        <button data-act="restart" ${c.self ? 'disabled' : ''}>Restart</button>
      </span>
    `;
    row.querySelectorAll('button').forEach((btn) => {
      btn.addEventListener('click', () => postAction(c.service, btn.dataset.act));
    });
    sysContainers.appendChild(row);
  }
}

async function postAction(service, action) {
  if (!confirm(`${action} "${service}"? If it's on the drivetrain path (robot/behaviors/slam/nav2), the robot will coast to a stop.`)) return;
  try {
    await fetch(`${FLEET_API}/containers/${service}/${action}`, { method: 'POST' });
  } catch (e) { /* fleet_status offline; next poll will show it */ }
  setTimeout(refreshSystem, 1500);
}

sysRestartAll.addEventListener('click', async () => {
  if (!confirm('Restart ALL containers? This coasts the robot to a stop and drops nav/mapping until everything relaunches (staggered, ~30-60s).')) return;
  sysRestartAll.disabled = true;
  sysRestartAll.textContent = 'Restarting…';
  try {
    await fetch(`${FLEET_API}/restart-all`, { method: 'POST' });
  } catch (e) { /* ignore */ }
  setTimeout(() => {
    sysRestartAll.disabled = false;
    sysRestartAll.textContent = 'Restart All';
    refreshSystem();
  }, 10000);
});

async function refreshSystem() {
  try {
    const [stats, containers] = await Promise.all([
      fetch(FLEET_API + '/stats').then((r) => r.json()),
      fetch(FLEET_API + '/containers').then((r) => r.json()),
    ]);
    sysState.textContent = `${containers.filter((c) => c.status === 'running').length}/${containers.length} up`;
    renderVitals(stats);
    renderContainers(containers);
  } catch (e) {
    sysState.textContent = 'offline';
    sysVitals.textContent = 'fleet_status unreachable';
  }
}
refreshSystem();
setInterval(refreshSystem, 30000);

// --- companion panel: the box's OWN fleet_status (:9003) ---------------------------
// Same backend/code as the Pi's, running on the companion and scoped to its own
// compose project. We learn the box address from the Pi's /api/companion
// (COMPANION_HOST); if it's unconfigured the panel hides itself (Pi-standalone,
// spec §0.7). Browser->companion works directly (same as Foxglove :8766). Host
// reboot is a companion-only endpoint (FLEET_ALLOW_HOST_REBOOT).
const cmpState = document.getElementById('cmp-state');
const cmpVitals = document.getElementById('cmp-vitals');
const cmpContainers = document.getElementById('cmp-containers');
const cmpRestartAll = document.getElementById('cmp-restart-all');
const cmpRebootHost = document.getElementById('cmp-reboot-host');
const cmpPanel = document.getElementById('cmp-panel');
let CMP_API = null;

function renderCmpContainers(list) {
  cmpContainers.innerHTML = '';
  for (const c of list) {
    const row = document.createElement('div');
    row.className = 'svc-row';
    const running = c.status === 'running';
    row.innerHTML = `
      <span class="svc-dot ${c.status}"></span>
      <span class="svc-name">${c.service}</span>
      <span class="svc-stat">${running ? `${c.cpu_percent}% · ${c.mem_mb}MB` : c.status}</span>
      <span class="svc-actions">
        <button data-act="stop" ${!running || c.self ? 'disabled' : ''}>Stop</button>
        <button data-act="restart" ${c.self ? 'disabled' : ''}>Restart</button>
      </span>
    `;
    row.querySelectorAll('button').forEach((btn) => {
      btn.addEventListener('click', () => postCmpAction(c.service, btn.dataset.act));
    });
    cmpContainers.appendChild(row);
  }
}

async function postCmpAction(service, action) {
  if (!CMP_API) return;
  if (!confirm(`${action} companion "${service}"? Perception/mapping on the box may drop until it relaunches (the Pi/robot is unaffected).`)) return;
  try {
    await fetch(`${CMP_API}/containers/${service}/${action}`, { method: 'POST' });
  } catch (e) { /* companion offline; next poll will show it */ }
  setTimeout(refreshCompanion, 1500);
}

cmpRestartAll.addEventListener('click', async () => {
  if (!CMP_API) return;
  if (!confirm('Restart ALL companion containers? Mapping/perception on the box drops until they relaunch (staggered). The Pi/robot is unaffected.')) return;
  cmpRestartAll.disabled = true;
  cmpRestartAll.textContent = 'Restarting…';
  try { await fetch(`${CMP_API}/restart-all`, { method: 'POST' }); } catch (e) { /* ignore */ }
  setTimeout(() => {
    cmpRestartAll.disabled = false;
    cmpRestartAll.textContent = 'Restart All';
    refreshCompanion();
  }, 10000);
});

cmpRebootHost.addEventListener('click', async () => {
  if (!CMP_API) return;
  if (!confirm('REBOOT the companion machine? The box goes down ~1–2 min; all companion containers restart on boot. The Pi/robot is unaffected.')) return;
  cmpRebootHost.disabled = true;
  cmpRebootHost.textContent = 'Rebooting…';
  try { await fetch(`${CMP_API}/reboot-host`, { method: 'POST' }); } catch (e) { /* box going down */ }
  setTimeout(() => {
    cmpRebootHost.disabled = false;
    cmpRebootHost.textContent = 'Reboot Host';
    refreshCompanion();
  }, 15000);
});

async function refreshCompanion() {
  // Resolve the box address once, from the Pi's companion health.
  if (!CMP_API) {
    try {
      const c = await fetch(FLEET_API + '/companion').then((r) => r.json());
      if (!c || !c.configured || !c.host) { cmpPanel.style.display = 'none'; return; }
      CMP_API = 'http://' + c.host + ':9003/api';
    } catch (e) { cmpState.textContent = 'offline'; return; }
  }
  try {
    const [stats, containers] = await Promise.all([
      fetch(CMP_API + '/stats').then((r) => r.json()),
      fetch(CMP_API + '/containers').then((r) => r.json()),
    ]);
    cmpState.textContent = `${containers.filter((c) => c.status === 'running').length}/${containers.length} up`;
    renderVitals(stats, cmpVitals);
    renderCmpContainers(containers);
  } catch (e) {
    cmpState.textContent = 'unreachable';
    cmpVitals.textContent = 'companion fleet_status unreachable';
  }
}
refreshCompanion();
setInterval(refreshCompanion, 30000);

// --- site panel: location bundles (ADR-0023) ---------------------------------------
// Each site = maps + zones + waypoints + tags + captures for one location.
// fleet_status owns the sites/active symlink and restarts slam/nav2/behaviors
// on a switch; this panel is just the picker. Endpoints 404 (panel hides)
// until the Pi is migrated (scripts/migrate_sites.py). The companion mirrors
// the switch best-effort over its own fleet_status — offline never blocks.
const sitePanel = document.getElementById('site-panel');
const siteState = document.getElementById('site-state');
const siteList = document.getElementById('site-list');
const siteResult = document.getElementById('site-result');
const siteMapName = document.getElementById('site-map-name');

// Guards the switch: a goal in flight = busy, rec-live = bag writing.
// Literal copy of core.status NAV_BUSY_STATES (goal_status_names 1-3);
// frozen against the Python tuple by scout/test/test_status.py.
const NAV_BUSY_STATES = ['accepted', 'driving', 'canceling'];
let siteNavBusy = false;
new ROSLIB.Topic({
  ros, name: '/nav_state', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  siteNavBusy = NAV_BUSY_STATES.includes(msg.data.split('|')[0]);
});

let activeSiteMeta = null;

function renderSites(data) {
  activeSiteMeta = (data.sites || []).find((s) => s.name === data.active) || null;
  siteState.textContent = data.active
    ? `${data.active}${activeSiteMeta && activeSiteMeta.active_map ? ' · ' + activeSiteMeta.active_map : ''} · ${(activeSiteMeta && activeSiteMeta.slam_mode) || 'auto'}`
    : 'none';
  renderSlamMode();
  siteList.innerHTML = '';
  for (const s of data.sites || []) {
    const row = document.createElement('div');
    row.className = 'svc-row';
    const isActive = s.name === data.active;
    const mapLabel = s.active_map || s.default_map || 'no map yet';
    row.innerHTML = `
      <span class="svc-dot ${isActive ? 'running' : 'exited'}"></span>
      <span class="svc-name">${s.display_name || s.name}</span>
      <span class="svc-stat">${mapLabel}</span>
      <span class="svc-actions">
        <button data-site="${s.name}" ${isActive ? 'disabled' : ''}>Switch</button>
      </span>
    `;
    const btn = row.querySelector('button');
    if (btn) btn.addEventListener('click', () => switchSite(s.name));
    siteList.appendChild(row);
  }
  renderSiteMaps();
  const last = data.last_switch;
  if (last && last.restarts && last.restarts.some((r) => !r.ok)) {
    const failed = last.restarts.filter((r) => !r.ok).map((r) => r.service);
    siteResult.textContent = `last switch: ${failed.join(', ')} failed to restart — retry from the System panel.`;
  }
}

// --- per-site maps (ADR-0029) --------------------------------------------------
// A site holds multiple labeled maps (one per floor); active_map is the one
// slam/amcl runs on. Activating in localization mode swaps the grid live via
// map_server LoadMap (~1 s); any other mode restarts slam (map bound at launch).
const siteMapsEl = document.getElementById('site-maps');

function renderSiteMaps() {
  siteMapsEl.innerHTML = '';
  const maps = (activeSiteMeta && activeSiteMeta.maps) || {};
  const active = activeSiteMeta && activeSiteMeta.active_map;
  for (const name of Object.keys(maps).sort()) {
    const m = maps[name];
    const row = document.createElement('div');
    row.className = 'svc-row';
    const isActive = name === active;
    const floor = (m.floor !== null && m.floor !== undefined) ? ` · F${m.floor}` : '';
    const badges = `${m.posegraph ? ' [graph]' : ''}${m.grid ? ' [grid]' : ''}${m.unregistered ? ' (unregistered)' : ''}`;
    row.innerHTML = `
      <span class="svc-dot ${isActive ? 'running' : 'exited'}"></span>
      <span class="svc-name">${m.label && m.label !== name ? `${m.label} (${name})` : name}${floor}</span>
      <span class="svc-stat">${badges.trim() || 'no files'}</span>
      <span class="svc-actions">
        <button data-map="${name}" ${isActive ? 'disabled' : ''}>Activate</button>
      </span>
    `;
    const btn = row.querySelector('button');
    if (btn) btn.addEventListener('click', () => activateMap(name));
    siteMapsEl.appendChild(row);
  }
}

const loadMapSrv = new ROSLIB.Service({
  ros, name: '/map_server/load_map',
  serviceType: 'nav2_msgs/srv/LoadMap',
});
const initialPosePub = new ROSLIB.Topic({
  ros, name: '/initialpose',
  messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
});
const reseedSrv = new ROSLIB.Service({
  ros, name: '/tag_relocalizer/reseed',
  serviceType: 'std_srvs/srv/Trigger',
});

async function postActiveMap(name) {
  const res = await fetch(`${FLEET_API}/sites/active`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ active_map: name }),
  });
  const body = await res.json();
  if (!res.ok) throw new Error(body.error || res.status);
}

async function activateMap(name) {
  if (!activeSiteMeta) return;
  if (siteNavBusy) { alert('Navigation goal active — cancel it before switching maps.'); return; }
  if (recState.classList.contains('rec-live')) { alert('Recording active — stop it before switching maps.'); return; }
  const entry = (activeSiteMeta.maps || {})[name] || {};
  const mode = activeSiteMeta.slam_mode || 'auto';
  if (mode === 'localization') {
    if (!entry.grid) {
      alert(`"${name}" has no grid map (.yaml/.pgm), which amcl needs — re-save it from a mapping session first.`);
      return;
    }
    if (!confirm(`Switch to map "${name}" live? The grid swaps in ~1 s and the pose is seeded at the map's start pose — show the robot a registered tag to refine.`)) return;
    siteResult.textContent = `loading map "${name}"…`;
    loadMapSrv.callService(
      new ROSLIB.ServiceRequest({ map_url: `/ros_ws/src/sites/active/maps/${name}.yaml` }),
      async (res) => {
        if (res.result !== 0) { siteResult.textContent = `map load failed (result ${res.result})`; return; }
        try { await postActiveMap(name); } catch (e) {
          siteResult.textContent = `map "${name}" loaded but not persisted (${e.message}) — the next slam restart reverts.`;
          return;
        }
        // Seed amcl at the map's start pose; a registered-tag sighting refines.
        const pose = entry.map_start_pose || [0, 0, 0];
        const cov = new Array(36).fill(0);
        cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.0685; // ~15 deg
        initialPosePub.publish(new ROSLIB.Message({
          header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
          pose: {
            pose: {
              position: { x: pose[0], y: pose[1], z: 0 },
              orientation: { x: 0, y: 0, z: Math.sin(pose[2] / 2), w: Math.cos(pose[2] / 2) },
            },
            covariance: cov,
          },
        }));
        reseedSrv.callService(new ROSLIB.ServiceRequest({}), () => {}, () => {});
        siteResult.textContent = `now on map "${name}" (live swap) — pose seeded at its start pose.`;
        refreshSites();
      },
      (err) => { siteResult.textContent = 'map load failed: ' + err; },
    );
    return;
  }
  if (!confirm(`Switch to map "${name}"? slam + behaviors restart (~20 s); driving and camera stay up.`)) return;
  siteResult.textContent = `switching to map "${name}"…`;
  try {
    await postActiveMap(name);
    await fetch(`${FLEET_API}/containers/slam/restart`, { method: 'POST' });
    await fetch(`${FLEET_API}/containers/behaviors/restart`, { method: 'POST' });
    siteResult.textContent = `map "${name}" — restarting slam + behaviors…`;
  } catch (e) {
    siteResult.textContent = 'map switch failed: ' + e.message;
    return;
  }
  setTimeout(refreshSites, 5000);
  setTimeout(() => { refreshSites(); refreshSystem(); }, 25000);
}

// --- slam mode selector -------------------------------------------------------------
// Writes site.json's slam_mode (fleet_status validates it) and restarts
// slam + behaviors so mode:=site re-resolves it — the mode is which
// executable runs (ADR-0003/0028), so there is no live switch.
// Vocabulary is the SITE-level one (auto, not the launch-only 'site').
const SLAM_MODES = [
  ['auto', 'Continue the default map if it has a saved graph, else start a new one. Never localization.'],
  ['new', 'Fresh blank map. Save map to keep it and make it the site default.'],
  ['continue', 'Load the saved graph and keep extending it — the map stays savable.'],
  ['localization', 'amcl on the saved grid map: map is fixed, nothing savable. Best for repeatable nav on a finished map.'],
];
const siteModeEl = document.getElementById('site-mode');
const siteModeDesc = document.getElementById('site-mode-desc');

function renderSlamMode() {
  siteModeEl.innerHTML = '';
  if (!activeSiteMeta) { siteModeDesc.textContent = ''; return; }
  const current = activeSiteMeta.slam_mode || 'auto';
  for (const [mode, desc] of SLAM_MODES) {
    const btn = document.createElement('button');
    btn.textContent = mode;
    btn.title = desc;
    if (mode === current) btn.classList.add('selected');
    btn.addEventListener('click', () => setSlamMode(mode));
    siteModeEl.appendChild(btn);
  }
  siteModeDesc.textContent = `${current}: ${SLAM_MODES.find(([m]) => m === current)[1]}`;
}

async function setSlamMode(mode) {
  if (!activeSiteMeta || mode === (activeSiteMeta.slam_mode || 'auto')) return;
  if (siteNavBusy) { alert('Navigation goal active — cancel it before changing slam mode.'); return; }
  if (recState.classList.contains('rec-live')) { alert('Recording active — stop it before changing slam mode.'); return; }
  const map = activeSiteMeta.active_map || activeSiteMeta.default_map;
  const entry = ((activeSiteMeta.maps || {})[map]) || {};
  // Head off the two site.json states that make slam.launch.py refuse to
  // start (crash-loop under restart: unless-stopped).
  if ((mode === 'continue' || mode === 'localization') && !map) {
    alert(`"${mode}" needs an active map — Save map first.`);
    return;
  }
  if (mode === 'continue' && !entry.posegraph) {
    alert(`"${map}" has no saved graph (.posegraph) in this site — Save map first.`);
    return;
  }
  if (mode === 'localization' && !entry.grid) {
    alert(`"${map}" has no grid map (.yaml/.pgm), which amcl needs — re-save it from a mapping session (Save map writes both formats).`);
    return;
  }
  const desc = SLAM_MODES.find(([m]) => m === mode)[1];
  if (!confirm(`Set slam mode to "${mode}"?\n\n${desc}\n\nslam + behaviors restart (~20 s); driving and camera stay up.`)) return;
  siteResult.textContent = `setting mode "${mode}"…`;
  try {
    const res = await fetch(`${FLEET_API}/sites/active`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ slam_mode: mode }),
    });
    const body = await res.json();
    if (!res.ok) { siteResult.textContent = 'mode change failed: ' + (body.error || res.status); return; }
    await fetch(`${FLEET_API}/containers/slam/restart`, { method: 'POST' });
    await fetch(`${FLEET_API}/containers/behaviors/restart`, { method: 'POST' });
    siteResult.textContent = `slam mode "${mode}" — restarting slam + behaviors…`;
  } catch (e) {
    siteResult.textContent = 'fleet_status unreachable';
    return;
  }
  setTimeout(refreshSites, 5000);
  setTimeout(() => { refreshSites(); refreshSystem(); }, 25000);
}

async function switchSite(name) {
  if (siteNavBusy) { alert('Navigation goal active — cancel it before switching sites.'); return; }
  if (recState.classList.contains('rec-live')) { alert('Recording active — stop it before switching sites.'); return; }
  if (!confirm(`Switch to site "${name}"? slam/nav2/behaviors restart (~20 s); the map, zones and waypoints change to that site's. Driving and camera stay up.`)) return;
  siteResult.textContent = 'switching…';
  try {
    const res = await fetch(`${FLEET_API}/sites/${name}/activate`, { method: 'POST' });
    const body = await res.json();
    if (!res.ok) { siteResult.textContent = 'switch failed: ' + (body.error || res.status); return; }
    siteResult.textContent = `now on "${name}" — restarting ${(body.restarting || []).join(', ')}…`;
  } catch (e) {
    siteResult.textContent = 'fleet_status unreachable';
    return;
  }
  // Companion mirror (per-site rtabmap.db + inspection captures). Best-effort:
  // create the site there if it's new; an offline companion never blocks.
  if (CMP_API) {
    try {
      await fetch(`${CMP_API}/sites/${name}/activate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ create: true }),
      });
    } catch (e) { /* companion offline; next switch re-syncs */ }
  }
  setTimeout(refreshSites, 5000);
  setTimeout(() => { refreshSites(); refreshSystem(); }, 25000);
}

document.getElementById('site-add-form').addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const input = document.getElementById('site-add-name');
  const name = input.value.trim();
  if (!name) return;
  try {
    const res = await fetch(`${FLEET_API}/sites`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }),
    });
    const body = await res.json();
    siteResult.textContent = res.ok ? `created "${name}" — press Switch to start mapping there.`
      : 'create failed: ' + (body.error || res.status);
    if (res.ok) input.value = '';
  } catch (e) { siteResult.textContent = 'fleet_status unreachable'; }
  refreshSites();
});

// Save the live slam graph into the active site + make it the site's default
// map. Writes BOTH map formats (ADR-0028): serialize_map for the .posegraph/.data
// pair that continue mode loads, then save_map for the .yaml/.pgm grid that
// localization mode's amcl + map_server load. Both services append their own
// extensions. ⚠ In localization mode slam_toolbox is not running at all (amcl
// stack instead) — mode:=site auto policy never runs localization, so this only
// guards a hand-set mode.
const serializeSrv = new ROSLIB.Service({
  ros, name: '/slam_toolbox/serialize_map',
  serviceType: 'slam_toolbox/srv/SerializePoseGraph',
});
const saveGridSrv = new ROSLIB.Service({
  ros, name: '/slam_toolbox/save_map',
  serviceType: 'slam_toolbox/srv/SaveMap',
});
document.getElementById('site-save-map').addEventListener('click', () => {
  if (activeSiteMeta && activeSiteMeta.slam_mode === 'localization') {
    alert('This site is pinned to localization mode: slam_toolbox is not running (amcl localizes instead), so there is nothing to save. Set slam_mode to auto/continue first.');
    return;
  }
  const name = siteMapName.value.trim()
    || (activeSiteMeta && (activeSiteMeta.active_map || activeSiteMeta.name)) || 'map';
  if (!confirm(`Save the current map as "${name}" in the active site?`)) return;
  siteResult.textContent = 'serializing map…';
  serializeSrv.callService(
    new ROSLIB.ServiceRequest({ filename: '/ros_ws/src/sites/active/maps/' + name }),
    () => {
      siteResult.textContent = 'saving grid map…';
      const finish = async (gridErr) => {
        try {
          // Register the map entry (label/floor, ADR-0029) + make it active.
          const label = document.getElementById('site-map-label').value.trim();
          const floorRaw = document.getElementById('site-map-floor').value.trim();
          const entry = {};
          if (label) entry.label = label;
          if (floorRaw !== '') entry.floor = parseInt(floorRaw, 10);
          await fetch(`${FLEET_API}/sites/active`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ active_map: name, maps: { [name]: entry } }),
          });
        } catch (e) { /* metadata update is best-effort; the files are saved */ }
        siteResult.textContent = gridErr
          ? `map "${name}" saved (posegraph only — grid save failed: ${gridErr}; localization mode needs the grid)`
          : `map "${name}" saved (posegraph + grid).`;
        if (confirm('Map saved. Restart slam + behaviors now to continue on it?')) {
          try {
            await fetch(`${FLEET_API}/containers/slam/restart`, { method: 'POST' });
            await fetch(`${FLEET_API}/containers/behaviors/restart`, { method: 'POST' });
          } catch (e) { /* ignore */ }
        }
        refreshSites();
      };
      saveGridSrv.callService(
        new ROSLIB.ServiceRequest({ name: { data: '/ros_ws/src/sites/active/maps/' + name } }),
        // SaveMap reports failure in-band: result 0 = success, 255 = no map yet.
        (res) => finish(res.result === 0 ? null : `result ${res.result}`),
        (err) => finish(err),
      );
    },
    (err) => { siteResult.textContent = 'serialize failed: ' + err; },
  );
});

async function refreshSites() {
  try {
    const res = await fetch(FLEET_API + '/sites');
    if (res.status === 404) { sitePanel.style.display = 'none'; return; }
    sitePanel.style.display = '';
    renderSites(await res.json());
  } catch (e) { siteState.textContent = 'offline'; }
}
refreshSites();
setInterval(refreshSites, 30000);

// --- network panel: NM known networks + connect/forget ----------------------------
// Same fleet_status backend, /api/wifi/*. Switching networks risks stranding this
// very page (served over wlan0), so every mutating action gets an explicit warning
// in its confirm(). Scan is on-demand only (nmcli rescan is slow) — not polled.
const netState = document.getElementById('net-state');
const netStatus = document.getElementById('net-status');
const netConnections = document.getElementById('net-connections');
const netRescan = document.getElementById('net-rescan');
const netScanResults = document.getElementById('net-scan-results');
const netAddForm = document.getElementById('net-add-form');
const netAddSsid = document.getElementById('net-add-ssid');
const netAddPassword = document.getElementById('net-add-password');

function renderNetStatus(s) {
  if (s.error) {
    netStatus.textContent = s.error;
    return;
  }
  netStatus.innerHTML = s.connected
    ? `<span>Connected: ${s.ssid}</span><span>${s.ip4 || ''}</span>${s.signal != null ? `<span>${s.signal}%</span>` : ''}`
    : '<span>Not connected</span>';
}

function renderNetConnections(list) {
  netConnections.innerHTML = '';
  if (list.error) {
    netConnections.textContent = list.error;
    return;
  }
  for (const c of list) {
    const row = document.createElement('div');
    row.className = 'svc-row';
    row.innerHTML = `
      <span class="svc-dot ${c.active ? 'running' : 'exited'}"></span>
      <span class="svc-name">${c.name}</span>
      <span class="svc-stat">${c.active ? 'active' : c.autoconnect ? 'saved' : 'saved (manual)'}</span>
      <span class="svc-actions">
        <button data-act="connect" ${c.active ? 'disabled' : ''}>Connect</button>
        <button data-act="forget">Forget</button>
      </span>
    `;
    row.querySelectorAll('button').forEach((btn) => {
      btn.addEventListener('click', () => netAction(c.name, btn.dataset.act, c.active));
    });
    netConnections.appendChild(row);
  }
}

async function netAction(name, act, wasActive) {
  const warn = act === 'connect'
    ? `Switch to "${name}"? If this fails, this page (served over WiFi) may lose connectivity — NetworkManager should fall back automatically.`
    : `Forget "${name}"? ${wasActive ? 'This is the ACTIVE connection — forgetting it will likely disconnect this session.' : ''}`;
  if (!confirm(warn)) return;
  const path = act === 'connect' ? 'connect' : 'forget';
  const body = act === 'connect' ? { name } : { name, force: wasActive };
  try {
    await fetch(`${FLEET_API}/wifi/${path}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
    });
  } catch (e) { /* likely stranded mid-switch; next poll (if it comes back) will show it */ }
  setTimeout(refreshNetwork, 2000);
}

netRescan.addEventListener('click', async () => {
  netScanResults.textContent = 'scanning…';
  try {
    const results = await fetch(`${FLEET_API}/wifi/scan`).then((r) => r.json());
    if (results.error) { netScanResults.textContent = results.error; return; }
    netScanResults.innerHTML = results.map((r) =>
      `<div class="svc-row"><span class="svc-name">${r.ssid}</span><span class="svc-stat">${r.signal}% ${r.security || 'open'}</span></div>`,
    ).join('');
  } catch (e) { netScanResults.textContent = 'scan failed'; }
});

netAddForm.addEventListener('submit', async (e) => {
  e.preventDefault();
  const ssid = netAddSsid.value.trim();
  const password = netAddPassword.value;
  if (!ssid) return;
  if (!confirm(`Connect to "${ssid}"? If this fails, this page (served over WiFi) may lose connectivity.`)) return;
  try {
    await fetch(`${FLEET_API}/wifi/connect`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ ssid, password }),
    });
  } catch (e) { /* likely stranded mid-switch */ }
  netAddPassword.value = '';
  setTimeout(refreshNetwork, 2000);
});

async function refreshNetwork() {
  try {
    const [status, connections] = await Promise.all([
      fetch(FLEET_API + '/wifi/status').then((r) => r.json()),
      fetch(FLEET_API + '/wifi/connections').then((r) => r.json()),
    ]);
    netState.textContent = status.connected ? status.ssid : 'disconnected';
    renderNetStatus(status);
    renderNetConnections(connections);
  } catch (e) {
    netState.textContent = 'offline';
    netStatus.textContent = 'fleet_status unreachable';
  }
}
refreshNetwork();
setInterval(refreshNetwork, 30000);
