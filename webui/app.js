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
  document.body.classList.remove('offline');
});
ros.on('close', () => {
  connBadge.textContent = 'offline';
  connBadge.className = 'badge disconnected';
  document.body.classList.add('offline');
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
  const vCls = mainV <= BATT_CRIT_V ? 'bad' : mainV <= BATT_WARN_V ? 'warn' : '';
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
let navBusyNow = false; // from /nav_state; the plan is only meaningful while a goal is live
let uhfTags = [];     // registry rows WITH est_pose (ADR-0032 map markers)
let uhfShowTags = true;   // corner Tags toggle — gates UHF AND AprilTag markers
let aprilTags = [];   // surveyed AprilTag registry rows (fleet_status /api/apriltags)
let sgrid = null;     // grid of the active map as saved (fleet_status /api/grid)
let sgridView = null; // what is drawn/located: sgrid, or for mode 'auto' sgrid grown to the live /map extent
let liveExtent = null;    // known-cell bbox + 1 cell margin of the live /map (map frame, m)
let liveOccupied = 0;     // occupied cells in the live /map
let liveMsg = null;       // latest /map message (for the one-shot auto-grid POST)
let sgridShow = true;     // 'Grid' overlay toggle
let sgridBays = false;    // 'Bays' labels toggle
let uhfPopTag = null; // EPC of the marker whose popup is open

const mapTopic = new ROSLIB.Topic({
  ros, name: '/map', messageType: 'nav_msgs/msg/OccupancyGrid',
  throttle_rate: 2000, queue_length: 1,
});
mapTopic.subscribe((msg) => {
  grid = msg.info;
  const w = msg.info.width, h = msg.info.height;
  gridCanvas.width = w;
  gridCanvas.height = h;
  mapUpdateBounds(msg.info);
  const img = gridCanvas.getContext('2d').createImageData(w, h);
  let c0 = w, c1 = -1, r0 = h, r1 = -1, nOcc = 0;
  for (let i = 0; i < w * h; i++) {
    const v = msg.data[i];
    if (v >= 0) {
      const c = i % w, r = (i - c) / w;
      if (c < c0) c0 = c;
      if (c > c1) c1 = c;
      if (r < r0) r0 = r;
      if (r > r1) r1 = r;
      if (v >= OCC_THRESHOLD) nOcc++;
    }
    // unknown: dark gray; free: light; occupied: near-black. The split is the
    // profile's occupied_threshold (shared with render.py).
    const c = v < 0 ? 26 : v < OCC_THRESHOLD ? 210 : 8;
    img.data[4 * i] = c;
    img.data[4 * i + 1] = c;
    img.data[4 * i + 2] = c + (v >= OCC_THRESHOLD ? 8 : 0);
    img.data[4 * i + 3] = 255;
  }
  gridCanvas.getContext('2d').putImageData(img, 0, 0);
  const res = msg.info.resolution, ox = msg.info.origin.position.x, oy = msg.info.origin.position.y;
  liveExtent = c1 < 0 ? null : {
    min_x: ox + (c0 - 1) * res, max_x: ox + (c1 + 2) * res,
    min_y: oy + (r0 - 1) * res, max_y: oy + (r1 + 2) * res,
  };
  liveOccupied = nOcc;
  liveMsg = msg;
  sgRefreshView();
  sgMaybeAutoCreate();
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
  // /plan is never cleared by nav2 when a goal ends, and a final replan can land
  // after the terminal /nav_state — ignore it unless a goal is live.
  if (!navBusyNow) return;
  plan = msg.poses.map((ps) => ({ x: ps.pose.position.x, y: ps.pose.position.y }));
  // Redraw now: in localization mode /map is latched (no periodic republish) and
  // /amcl_pose is sparse, so without this the plan waits for the next pose update.
  drawMap();
});

// --- map view: zoom / pan / large view ------------------------------------------
// ONE transform (MapView, mapview.js) maps world <-> canvas CSS px. Everything that
// draws goes through worldToCanvas, every pointer->world conversion through
// canvasToWorld; nothing else may do its own map math. The backing store is
// CSS size x devicePixelRatio and drawMap() applies the dpr as a base transform,
// so all drawing code works in CSS px.
const MV = window.MapView;
const mapFrame = document.getElementById('map-frame');
const navPanel = document.getElementById('nav');
const mapZoomEl = document.getElementById('map-zoom');
const MAX_MAP_PIXELS = 16e6;
let mapSize = { w: 0, h: 0 };   // CSS px
let mapDpr = 1;
let mapView = null;             // {k, cx, cy}; null until first fit
let mapBounds = null;           // world bounds of the latest grid
let mapFollow = false;
let mapViewKey = null;          // map name the view belongs to
let mapSaveTimer = null;
let mapDragging = false;       // a pan/pinch gesture is in progress

function mapKey() {
  try { return (activeSiteMeta && activeSiteMeta.active_map) || 'default'; } catch (e) { return 'default'; }
}
function mapFitK() { return MV.fitK(mapBounds, mapSize); }
function mapStoreView() {
  if (!mapView) return;
  clearTimeout(mapSaveTimer);
  mapSaveTimer = setTimeout(() => {
    try {
      sessionStorage.setItem('scout.mapview.' + mapViewKey, JSON.stringify(mapView));
    } catch (e) { /* storage unavailable: view just is not remembered */ }
  }, 250);
}
function mapLoadView(key) {
  try {
    const v = JSON.parse(sessionStorage.getItem('scout.mapview.' + key));
    if (v && isFinite(v.k) && v.k > 0 && isFinite(v.cx) && isFinite(v.cy)) return v;
  } catch (e) { /* none */ }
  return null;
}
function mapFitView() {
  if (!mapBounds || !mapSize.w || !mapSize.h) return;
  mapView = MV.fit(mapBounds, mapSize);
  mapStoreView();
}
// Called on every /map message. Keeps the user's view unless it is the first map,
// a different map name, or the extent changed drastically (a different map loaded).
function mapUpdateBounds(info) {
  const nb = MV.boundsOf(info);
  const key = mapKey();
  const reset = !mapView || key !== mapViewKey || MV.drasticChange(mapBounds, nb);
  mapBounds = nb;
  mapSyncControls();
  if (!reset) return;
  const keyChanged = key !== mapViewKey;
  mapViewKey = key;
  const saved = mapLoadView(key);
  if (saved && mapSize.w && (keyChanged || !mapView)) {
    mapView = saved;
  } else {
    mapFitView();
  }
  mapSyncControls();
}
function mapSetView(v, keepFollow) {
  if (!mapBounds) return;
  mapView = MV.constrain(v, mapSize, mapBounds, 48);
  if (!keepFollow) mapFollow = false;
  mapSyncControls();
  mapStoreView();
  drawMap();
}
function mapSyncControls() {
  if (mapZoomEl && mapView && mapBounds) {
    mapZoomEl.textContent = MV.zoomPercent(mapView.k, mapFitK()) + '%';
  }
  const f = document.getElementById('map-follow');
  if (f) f.classList.toggle('selected', mapFollow);
}
function mapZoomBy(factor, px, py) {
  if (!mapView || !mapBounds) return;
  const x = px === undefined ? mapSize.w / 2 : px;
  const y = py === undefined ? mapSize.h / 2 : py;
  mapSetView(MV.zoomAt(mapView, mapSize, x, y, factor, mapFitK()), true);
}
function mapResize() {
  const w = Math.floor(mapFrame.clientWidth), h = Math.floor(mapFrame.clientHeight);
  if (w < 2 || h < 2) return;
  const first = !mapSize.w;
  mapSize = { w, h };
  mapDpr = Math.min(window.devicePixelRatio || 1, Math.sqrt(MAX_MAP_PIXELS / (w * h)));
  mapCanvas.width = Math.round(w * mapDpr);
  mapCanvas.height = Math.round(h * mapDpr);
  if (first && mapBounds) {
    const saved = mapLoadView(mapViewKey || mapKey());
    if (saved) mapView = saved; else mapFitView();
  }
  mapSyncControls();
  drawMap();
}
new ResizeObserver(mapResize).observe(mapFrame);

function worldToCanvas(wx, wy) {
  return MV.worldToCanvas(mapView, mapSize, wx, wy);
}
// Pointer event (or any {clientX, clientY}) -> canvas CSS px -> world.
function canvasPoint(ev) {
  const r = mapCanvas.getBoundingClientRect();
  return { x: ev.clientX - r.left, y: ev.clientY - r.top };
}
function canvasToWorld(ev) {
  const c = canvasPoint(ev);
  return MV.canvasToWorld(mapView, mapSize, c.x, c.y);
}

// Tag facing arrow: map_yaw points robot-in-front -> tag, so the face normal
// (the side the robot stands on) is map_yaw + pi. Canvas y is flipped.
function drawTagFacing(c, mapYaw, color) {
  const a = mapYaw + Math.PI;
  const ex = c.x + 22 * Math.cos(a), ey = c.y - 22 * Math.sin(a);
  mapCtx.save();
  mapCtx.strokeStyle = color || '#2E7BFF';
  mapCtx.fillStyle = color || '#2E7BFF';
  mapCtx.lineWidth = 2;
  mapCtx.beginPath();
  mapCtx.moveTo(c.x, c.y);
  mapCtx.lineTo(ex, ey);
  mapCtx.stroke();
  const ang = Math.atan2(ey - c.y, ex - c.x);   // canvas-space direction
  for (const d of [0.45, -0.45]) {
    mapCtx.moveTo(ex, ey);
    mapCtx.lineTo(ex - 7 * Math.cos(ang + d), ey - 7 * Math.sin(ang + d));
  }
  mapCtx.stroke();
  mapCtx.restore();
}

function drawMap() {
  if (!grid) return;
  navState.textContent = robotPose ? 'live' : 'map only';
  if (!mapView || !mapSize.w) return;
  if (mapFollow && robotPose && !mapDragging) {
    mapView = MV.centerOn(mapView, robotPose.x, robotPose.y);
  }
  mapCtx.setTransform(1, 0, 0, 1, 0, 0);
  mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  mapCtx.setTransform(mapDpr, 0, 0, mapDpr, 0, 0);
  const cellPx = mapView.k * grid.resolution;
  mapCtx.imageSmoothingEnabled = cellPx * mapDpr < 1;  // minify smooth, magnify crisp
  mapCtx.save();
  // Occupancy image: row 0 is world y-min, so anchor at the bottom-left corner
  // and draw flipped upward.
  const bl = worldToCanvas(grid.origin.position.x, grid.origin.position.y);
  mapCtx.translate(bl.x, bl.y);
  mapCtx.scale(1, -1);
  mapCtx.drawImage(gridCanvas, 0, 0, cellPx * grid.width, cellPx * grid.height);
  mapCtx.restore();

  drawStructGrid();

  if (plan && plan.length > 1) {
    mapCtx.strokeStyle = '#F2B200';
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
      if (patrolProg.active && wp < patrolProg.i) color = '#F2B200';   // visited
      if (patrolProg.active && wp === patrolProg.i) {                  // current
        color = '#E07020';
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
    mapCtx.fillStyle = '#E02840';
    mapCtx.beginPath();
    mapCtx.moveTo(10, 0);
    mapCtx.lineTo(-6, 6);
    mapCtx.lineTo(-6, -6);
    mapCtx.closePath();
    mapCtx.fill();
    mapCtx.restore();
  }

  // UHF tag markers (ADR-0032): centroid dot + EPC tail + spread ring — the
  // ring radius IS the confidence (RSSI-weighted RMS scatter from the
  // registry), so a wide ring reads as "somewhere around here".
  if (uhfShowTags && uhfTags.length) {
    const pxPerM = mapView.k;
    mapCtx.save();
    mapCtx.font = '10px monospace';
    mapCtx.textAlign = 'center';
    for (const t of uhfTags) {
      const c = worldToCanvas(t.est_pose.x, t.est_pose.y);
      if (t.spread_m > 0.05) {
        mapCtx.strokeStyle = 'rgba(225, 0, 225, 0.35)';
        mapCtx.lineWidth = 1;
        mapCtx.beginPath();
        mapCtx.arc(c.x, c.y, t.spread_m * pxPerM, 0, 2 * Math.PI);
        mapCtx.stroke();
      }
      // Magenta on a white halo: the fill pops on black walls, the halo
      // separates it from white free space — nothing on an occupancy grid
      // is magenta.
      mapCtx.beginPath();
      mapCtx.arc(c.x, c.y, 5, 0, 2 * Math.PI);
      mapCtx.strokeStyle = '#ffffff';
      mapCtx.lineWidth = t.epc === uhfPopTag ? 5 : 3;
      mapCtx.stroke();
      mapCtx.fillStyle = '#E100E1';
      mapCtx.strokeStyle = '#3a0030';
      mapCtx.lineWidth = 1;
      mapCtx.fill();
      mapCtx.stroke();
      mapCtx.fillStyle = '#E100E1';
      mapCtx.strokeStyle = 'rgba(255, 255, 255, 0.9)';
      mapCtx.lineWidth = 3;
      const label = t.epc.slice(-6);
      mapCtx.strokeText(label, c.x, c.y - 9);
      mapCtx.fillText(label, c.x, c.y - 9);
    }
    mapCtx.restore();
  }

  // AprilTag markers: surveyed registry tags (tags.db via fleet_status).
  // role=home draws a house — the boot-relocalization anchor. Shares the
  // corner Tags toggle with the UHF markers.
  if (uhfShowTags && aprilTags.length) {
    mapCtx.save();
    mapCtx.textAlign = 'center';
    for (const t of aprilTags) {
      const c = worldToCanvas(t.map_x, t.map_y);
      // Vivid blue on a white halo (an emoji can't be recolored, so home is
      // a drawn house): fill carries it on black walls, halo on white floor.
      mapCtx.beginPath();
      if (t.role === 'home') {
        mapCtx.moveTo(c.x, c.y - 9);       // roof apex
        mapCtx.lineTo(c.x + 8, c.y - 1);   // right eave
        mapCtx.lineTo(c.x + 5, c.y - 1);
        mapCtx.lineTo(c.x + 5, c.y + 8);   // right wall
        mapCtx.lineTo(c.x - 5, c.y + 8);   // floor
        mapCtx.lineTo(c.x - 5, c.y - 1);   // left wall
        mapCtx.lineTo(c.x - 8, c.y - 1);   // left eave
      } else {
        mapCtx.moveTo(c.x, c.y - 7);
        mapCtx.lineTo(c.x + 7, c.y);
        mapCtx.lineTo(c.x, c.y + 7);
        mapCtx.lineTo(c.x - 7, c.y);
      }
      mapCtx.closePath();
      mapCtx.strokeStyle = '#ffffff';
      mapCtx.lineWidth = 3;
      mapCtx.stroke();
      mapCtx.fillStyle = '#2E7BFF';
      mapCtx.strokeStyle = '#0a1f4d';
      mapCtx.lineWidth = 1;
      mapCtx.fill();
      mapCtx.stroke();
      mapCtx.font = '10px monospace';
      mapCtx.fillStyle = '#2E7BFF';
      mapCtx.strokeStyle = 'rgba(255, 255, 255, 0.9)';
      mapCtx.lineWidth = 3;
      mapCtx.strokeText(t.name, c.x, c.y - (t.role === 'home' ? 13 : 11));
      mapCtx.fillText(t.name, c.x, c.y - (t.role === 'home' ? 13 : 11));
      if (t.map_yaw !== null && t.map_yaw !== undefined) drawTagFacing(c, t.map_yaw);
    }
    mapCtx.restore();
  }

  drawTagPreview();

  drawGridDefPreview();

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

  if (posePin) {
    // Pose-pin position placed, awaiting the heading tap.
    const c = worldToCanvas(posePin.x, posePin.y);
    mapCtx.strokeStyle = '#ff9f40';
    mapCtx.fillStyle = '#ff9f40';
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    mapCtx.arc(c.x, c.y, 6, 0, 2 * Math.PI);
    mapCtx.stroke();
    mapCtx.beginPath();
    mapCtx.arc(c.x, c.y, 2, 0, 2 * Math.PI);
    mapCtx.fill();
  }
}

// --- set a tag's surveyed pose from the map (doghouse placement) ------------
// Two taps: (1) the tag position, (2) a point where the robot stands in front
// of it. yaw = atan2(tag - front) = the registry's map_yaw (standoff heading).
// While active, tap-to-navigate is fully suppressed (see click handler).
const tagSet = { active: false, tag: null, front: null, yaw: null };
const TAG_STANDOFF_M = 0.5;   // keep equal to tags.STANDOFF_M
const tagSetter = document.getElementById('tag-setter');
const tagNameSel = document.getElementById('tag-name');
const tagStep = document.getElementById('tag-step');
const tagConfirmBtn = document.getElementById('tag-confirm');
const tagResult = document.getElementById('tag-result');
const setTagBtn = document.getElementById('set-tag');

function tagBannerText() {
  return 'SETTING ' + tagNameSel.value.toUpperCase() + ' - taps do not send goals';
}
function tagSetUpdateUi() {
  document.getElementById('tag-banner').textContent = tagBannerText();
  if (!tagSet.tag) tagStep.textContent = 'Tap 1/2: the tag position';
  else if (!tagSet.front) tagStep.textContent = 'Tap 2/2: where the robot stands in front of the tag';
  else tagStep.textContent = 'Review the preview, then Confirm';
  tagConfirmBtn.disabled = !(tagSet.tag && tagSet.front);
}
function tagSetExit() {
  tagSet.active = false;
  tagSet.tag = tagSet.front = tagSet.yaw = null;
  tagSetter.hidden = true;
  setTagBtn.classList.remove('selected');
  drawMap();
}
function tagSetTap(w) {
  if (!tagSet.tag || (tagSet.tag && tagSet.front)) {
    tagSet.tag = w; tagSet.front = null; tagSet.yaw = null;   // (re)start
  } else {
    if (Math.hypot(w.x - tagSet.tag.x, w.y - tagSet.tag.y) < 0.1) {
      tagResult.textContent = 'second tap too close to the tag - tap further out';
      return;
    }
    tagSet.front = w;
    tagSet.yaw = Math.atan2(tagSet.tag.y - w.y, tagSet.tag.x - w.x);
  }
  tagSetUpdateUi();
  drawMap();
}
function drawTagPreview() {
  if (!tagSet.active || !tagSet.tag || !grid) return;
  const c = worldToCanvas(tagSet.tag.x, tagSet.tag.y);
  mapCtx.save();
  mapCtx.strokeStyle = '#ff9f40';
  mapCtx.fillStyle = '#ff9f40';
  mapCtx.lineWidth = 2;
  mapCtx.strokeRect(c.x - 6, c.y - 6, 12, 12);
  if (tagSet.front) {
    const f = worldToCanvas(tagSet.front.x, tagSet.front.y);
    drawTagFacing(c, tagSet.yaw, '#ff9f40');
    // 0.5 m standoff point (where go_to_waypoint will park the robot)
    const sx = tagSet.tag.x - TAG_STANDOFF_M * Math.cos(tagSet.yaw);
    const sy = tagSet.tag.y - TAG_STANDOFF_M * Math.sin(tagSet.yaw);
    const s = worldToCanvas(sx, sy);
    mapCtx.beginPath();
    mapCtx.arc(s.x, s.y, 5, 0, 2 * Math.PI);
    mapCtx.fill();
    mapCtx.setLineDash([4, 4]);
    mapCtx.beginPath();
    mapCtx.moveTo(f.x, f.y);
    mapCtx.lineTo(c.x, c.y);
    mapCtx.stroke();
  }
  mapCtx.restore();
}
async function tagSetStart() {
  tagResult.textContent = '';
  try {
    const res = await fetch(FLEET_API + '/apriltags');
    const body = await res.json();
    const names = (body.tags || []).map((t) => t.name);
    if (!names.length) { navState.textContent = 'no registered tags'; return; }
    tagNameSel.innerHTML = '';
    for (const n of names) {
      const o = document.createElement('option');
      o.value = o.textContent = n;
      tagNameSel.appendChild(o);
    }
    if (names.includes('doghouse')) tagNameSel.value = 'doghouse';
  } catch (e) { navState.textContent = 'tag list failed: ' + e; return; }
  if (gridDef.active) gridDefExit();
  if (poseMode) { poseMode = 0; posePin = null; setPoseBtn.textContent = 'Set pose'; }
  if (areaMode) areaReset();
  tagSet.active = true;
  tagSet.tag = tagSet.front = tagSet.yaw = null;
  tagSetter.hidden = false;
  setTagBtn.classList.add('selected');
  tagSetUpdateUi();
  drawMap();
}
setTagBtn.addEventListener('click', () => (tagSet.active ? tagSetExit() : tagSetStart()));
tagNameSel.addEventListener('change', tagSetUpdateUi);
document.getElementById('tag-cancel').addEventListener('click', tagSetExit);
tagConfirmBtn.addEventListener('click', async () => {
  if (!(tagSet.tag && tagSet.front)) return;
  const name = tagNameSel.value;
  const pose = { x: +tagSet.tag.x.toFixed(3), y: +tagSet.tag.y.toFixed(3),
    yaw: +tagSet.yaw.toFixed(3) };
  if (!confirm('Set "' + name + '" to (' + pose.x + ', ' + pose.y + ') yaw '
    + (pose.yaw * 180 / Math.PI).toFixed(0) + ' deg?\n\n'
    + 'This moves the robot\'s localization reference: tag_relocalizer corrects '
    + 'amcl toward this surveyed pose whenever the tag is in view. Confirm it '
    + 'matches the REAL location of the ' + name + '.')) return;
  tagConfirmBtn.disabled = true;
  try {
    const res = await fetch(FLEET_API + '/apriltags/' + encodeURIComponent(name) + '/pose', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(pose),
    });
    const body = await res.json();
    if (!res.ok) { tagResult.textContent = 'failed: ' + (body.error || res.status); tagConfirmBtn.disabled = false; return; }
    const w = body.waypoint;
    navState.textContent = name + ' set (' + pose.x + ', ' + pose.y + ') yaw ' + pose.yaw
      + '; standoff (' + w.x + ', ' + w.y + ')';
    tagSetExit();
    refreshAprilTags();
  } catch (e) {
    tagResult.textContent = 'failed: ' + e;
    tagConfirmBtn.disabled = false;
  }
});


// --- structural grid (construction-drawing lines over the map) --------------
// Per-map file <map>.grid.json served by fleet_status /api/grid (storage in
// meters, map frame). Local +u = origin.yaw; numbers are constant-u lines,
// letters constant-v lines; bay "1B" = number 1..2 x letter B..C.
const FT_M = 0.3048;
function sgToMap(g, u, v) {
  const c = Math.cos(g.origin.yaw_rad), s = Math.sin(g.origin.yaw_rad);
  return { x: g.origin.x + c * u - s * v, y: g.origin.y + s * u + c * v };
}
function sgToGrid(g, x, y) {
  const c = Math.cos(g.origin.yaw_rad), s = Math.sin(g.origin.yaw_rad);
  const dx = x - g.origin.x, dy = y - g.origin.y;
  return { u: c * dx + s * dy, v: -s * dx + c * dy };
}
function sgSlot(offs, p) {
  if (p < offs[0] || p > offs[offs.length - 1]) return -1;
  for (let k = 0; k < offs.length - 1; k++) if (p < offs[k + 1]) return k;
  return offs.length - 2;
}
function sgLocate(g, x, y) {
  const q = sgToGrid(g, x, y);
  const i = sgSlot(g.numbers.offsets_m, q.u), j = sgSlot(g.letters.offsets_m, q.v);
  return (i < 0 || j < 0) ? null : g.numbers.labels[i] + g.letters.labels[j];
}
// Label at the far end of a line, nudged outward along the line (canvas px),
// text kept upright.
function sgEndLabel(a, b, text) {
  const ca = worldToCanvas(a.x, a.y), cb = worldToCanvas(b.x, b.y);
  const d = Math.hypot(cb.x - ca.x, cb.y - ca.y) || 1;
  const ux = (cb.x - ca.x) / d, uy = (cb.y - ca.y) / d;
  for (const [p, sgn] of [[ca, -1], [cb, 1]]) {
    const tx = p.x + sgn * ux * 11, ty = p.y + sgn * uy * 11;
    mapCtx.strokeText(text, tx, ty);
    mapCtx.fillText(text, tx, ty);
  }
}
function sgDraw(g, color, dashed) {
  const no = g.numbers.offsets_m, lo = g.letters.offsets_m;
  mapCtx.save();
  mapCtx.lineWidth = 1;
  mapCtx.strokeStyle = color;
  if (dashed) mapCtx.setLineDash([6, 4]);
  mapCtx.beginPath();
  const segs = [];
  g.numbers.labels.forEach((lb, k) => {
    segs.push([sgToMap(g, no[k], lo[0]), sgToMap(g, no[k], lo[lo.length - 1]), lb]);
  });
  g.letters.labels.forEach((lb, k) => {
    segs.push([sgToMap(g, no[0], lo[k]), sgToMap(g, no[no.length - 1], lo[k]), lb]);
  });
  for (const [a, b] of segs) {
    const ca = worldToCanvas(a.x, a.y), cb = worldToCanvas(b.x, b.y);
    mapCtx.moveTo(ca.x, ca.y);
    mapCtx.lineTo(cb.x, cb.y);
  }
  mapCtx.stroke();
  mapCtx.setLineDash([]);
  mapCtx.font = 'bold 11px monospace';
  mapCtx.textAlign = 'center';
  mapCtx.textBaseline = 'middle';
  mapCtx.lineWidth = 3;
  mapCtx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
  mapCtx.fillStyle = color;
  for (const [a, b, lb] of segs) sgEndLabel(a, b, lb);
  if (sgridBays && !dashed) {
    mapCtx.font = '10px monospace';
    for (let i = 0; i < no.length - 1; i++) {
      for (let j = 0; j < lo.length - 1; j++) {
        const m = sgToMap(g, (no[i] + no[i + 1]) / 2, (lo[j] + lo[j + 1]) / 2);
        const c = worldToCanvas(m.x, m.y);
        const t = g.numbers.labels[i] + g.letters.labels[j];
        mapCtx.strokeText(t, c.x, c.y);
        mapCtx.fillText(t, c.x, c.y);
      }
    }
  }
  mapCtx.restore();
}
// --- auto grid (standard grid sized to the map) ----------------------------
// mode 'auto': square cells of cell_m, origin + yaw frozen at creation, lines
// APPENDED at either end as the map grows so a bay label never changes.
// Labels (same scheme as docker/scout-skills/grid.py): numbers i>=0 -> "1","2"..,
// i<0 -> ".1",".2"..; letters j>=0 -> A..Z,AA.., j<0 -> ".A",".B"..
function sgBij(n) {
  let t = '';
  while (n > 0) { t = String.fromCharCode(65 + ((n - 1) % 26)) + t; n = Math.floor((n - 1) / 26); }
  return t;
}
const sgNumLabel = (i) => (i >= 0 ? String(i + 1) : '.' + (-i));
const sgLetLabel = (j) => (j >= 0 ? sgBij(j + 1) : '.' + sgBij(-j));
function sgIdxRange(lo, hi, cell) {
  const i0 = Math.floor(lo / cell + 1e-6);
  return [i0, Math.max(Math.ceil(hi / cell - 1e-6), i0 + 1)];
}
function sgAutoAxis(i0, i1, cell, lab) {
  const labels = [], offs = [];
  for (let i = i0; i <= i1; i++) { labels.push(lab(i)); offs.push(i * cell); }
  return { labels, offsets_m: offs, index_start: i0 };
}
// Pure: grid grown to cover extent (never shrinks); same object when unchanged.
function sgExtendAuto(g, e) {
  if (!g || g.mode !== 'auto' || !e || !g.cell_m) return g;
  const q = [[e.min_x, e.min_y], [e.max_x, e.min_y], [e.max_x, e.max_y], [e.min_x, e.max_y]]
    .map(([x, y]) => sgToGrid(g, x, y));
  const [i0, i1] = sgIdxRange(Math.min(...q.map((p) => p.u)), Math.max(...q.map((p) => p.u)), g.cell_m);
  const [j0, j1] = sgIdxRange(Math.min(...q.map((p) => p.v)), Math.max(...q.map((p) => p.v)), g.cell_m);
  const a0 = Math.min(g.numbers.index_start, i0);
  const a1 = Math.max(g.numbers.index_start + g.numbers.labels.length - 1, i1);
  const b0 = Math.min(g.letters.index_start, j0);
  const b1 = Math.max(g.letters.index_start + g.letters.labels.length - 1, j1);
  if (a0 === g.numbers.index_start && b0 === g.letters.index_start
    && a1 - a0 + 1 === g.numbers.labels.length && b1 - b0 + 1 === g.letters.labels.length) return g;
  if (a1 - a0 + 1 > 200 || b1 - b0 + 1 > 200) return g;   // server cap; keep what we have
  return Object.assign({}, g, {
    numbers: sgAutoAxis(a0, a1, g.cell_m, sgNumLabel),
    letters: sgAutoAxis(b0, b1, g.cell_m, sgLetLabel),
  });
}
let sgPersistAt = 0, sgPersistBusy = false, sgAutoBusy = false, sgServerNone = false;
// Recompute the drawn grid from the saved one + the live extent (no server
// round-trip); persist growth in the background, at most every 20 s.
function sgRefreshView() {
  sgridView = sgrid ? sgExtendAuto(sgrid, liveExtent) : null;
  if (sgrid && sgridView !== sgrid && !sgPersistBusy && Date.now() - sgPersistAt > 20000) {
    sgPersistGrowth();
  }
}
async function sgPersistGrowth() {
  sgPersistBusy = true;
  sgPersistAt = Date.now();
  try {
    const res = await fetch(FLEET_API + '/grid/auto', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ extent: liveExtent }),
    });
    const body = await res.json();
    if (res.ok && body.grid && body.action !== 'kept_structural') { sgrid = body.grid; sgRefreshView(); }
  } catch (e) { /* fleet_status offline: still drawn live */ }
  sgPersistBusy = false;
}
function sgSavedCellFt() {
  try {
    const v = parseFloat(localStorage.getItem('gridCellFt'));
    if (Number.isFinite(v) && v >= 5 && v <= 200) return v;
  } catch (e) { /* storage unavailable */ }
  return 30;
}
function sgOccBody(m) {
  return {
    width: m.info.width, height: m.info.height, resolution: m.info.resolution,
    origin: { x: m.info.origin.position.x, y: m.info.origin.position.y },
    data_b64: sgB64(m.data),
  };
}
function sgB64(data) {
  const u8 = new Uint8Array(data.length);
  for (let i = 0; i < data.length; i++) u8[i] = data[i] & 255;
  let bin = '';
  for (let i = 0; i < u8.length; i += 0x8000) bin += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
  return btoa(bin);
}
// A map with content and no grid file gets the auto grid (one POST; the server
// estimates the wall yaw from the posted occupancy and freezes it).
async function sgMaybeAutoCreate() {
  if (!sgServerNone || sgrid || sgAutoBusy || !liveMsg || !liveExtent || liveOccupied < 600) return;
  if (Date.now() - sgPersistAt < 15000) return;
  sgAutoBusy = true;
  sgPersistAt = Date.now();
  try {
    const m = liveMsg;
    const res = await fetch(FLEET_API + '/grid/auto', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        cell_ft: sgSavedCellFt(), align: 'walls',
        occupancy: sgOccBody(m),
      }),
    });
    const body = await res.json();
    if (res.ok && body.grid) {
      sgrid = body.grid;
      sgServerNone = false;
      sgRefreshView();
      drawMap();
    }
  } catch (e) { /* fleet_status offline */ }
  sgAutoBusy = false;
}

function drawStructGrid() {
  if (!sgridView || !sgridShow || !grid) return;
  sgDraw(sgridView, 'rgba(0, 150, 120, 0.85)', false);
}
const gridToggleBtn = document.getElementById('grid-toggle');
const gridBaysBtn = document.getElementById('grid-bays');
const gridHoverEl = document.getElementById('grid-hover');
gridToggleBtn.addEventListener('click', () => {
  sgridShow = !sgridShow;
  gridToggleBtn.classList.toggle('selected', sgridShow);
  drawMap();
});
gridBaysBtn.addEventListener('click', () => {
  sgridBays = !sgridBays;
  gridBaysBtn.classList.toggle('selected', sgridBays);
  drawMap();
});
mapCanvas.addEventListener('mousemove', (ev) => {
  if (!grid || !mapView || !sgridView || !sgridShow) { gridHoverEl.textContent = ''; return; }
  const w = canvasToWorld(ev);
  const bay = sgLocate(sgridView, w.x, w.y);
  gridHoverEl.textContent = bay ? 'Bay ' + bay : '';
});
mapCanvas.addEventListener('mouseleave', () => { gridHoverEl.textContent = ''; });

// --- Grid settings panel: cell size + regenerate (auto grid) -----------------
// Only fetches /grid/auto; never a nav goal and no map-tap mode of its own.
const gridSettingsBtn = document.getElementById('grid-settings');
const gridPanel = document.getElementById('grid-panel');
const gpCell = document.getElementById('gp-cell');
const gpAlign = document.getElementById('gp-align');
const gpResult = document.getElementById('gp-result');
function gpFillCell() {
  const ft = sgrid && sgrid.cell_m ? sgrid.cell_m / FT_M : sgSavedCellFt();
  gpCell.value = +ft.toFixed(2);
}
gridSettingsBtn.addEventListener('click', () => {
  gridPanel.hidden = !gridPanel.hidden;
  gridSettingsBtn.classList.toggle('selected', !gridPanel.hidden);
  if (!gridPanel.hidden) { gpFillCell(); gpResult.textContent = ''; }
});
document.getElementById('gp-apply').addEventListener('click', async () => {
  const ft = parseFloat(gpCell.value);
  if (!Number.isFinite(ft) || ft < 5 || ft > 200) { gpResult.textContent = 'cell size must be 5..200 ft'; return; }
  if (!liveMsg) { gpResult.textContent = 'no live map yet'; return; }
  if (!window.confirm('This regenerates the grid; bay labels like 1B will refer to different areas.')) return;
  const req = { regenerate: true, cell_ft: ft, align: gpAlign.value, occupancy: sgOccBody(liveMsg) };
  const structural = sgrid && sgrid.mode === 'structural';
  if (structural) {
    if (!window.confirm('The current grid is STRUCTURAL (from drawings). Replace it with an auto grid?')) return;
    req.force = true;
  }
  gpResult.textContent = 'applying...';
  try {
    const res = await fetch(FLEET_API + '/grid/auto', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(req),
    });
    const body = await res.json();
    if (!res.ok || !body.grid) { gpResult.textContent = 'failed: ' + (body.error || res.status); return; }
    try { localStorage.setItem('gridCellFt', String(ft)); } catch (e) { /* storage unavailable */ }
    sgrid = body.grid;
    sgServerNone = false;
    sgPersistAt = Date.now();
    sgRefreshView();
    drawMap();
    gpResult.textContent = body.action + ': ' + (body.cell_ft || ft) + ' ft cells';
  } catch (e) { gpResult.textContent = 'fleet_status offline'; }
});

// Fetched on load and from the sites poll (an active-map change reaches
// refreshSites too), so each map shows its own file.
async function refreshStructGrid() {
  try {
    const res = await fetch(FLEET_API + '/grid');
    if (!res.ok) return;
    const next = (await res.json()).grid || null;
    sgServerNone = !next;
    if (JSON.stringify(next) !== JSON.stringify(sgrid)) {
      sgrid = next;
      sgRefreshView();
      drawMap();
    }
    if (!next) sgMaybeAutoCreate();
  } catch (e) { /* fleet_status offline */ }
}

// --- Define grid: labels + spacings, then two reference taps ----------------
// Own mode (same banner pattern as Set tag position): while active every map
// tap is a reference tap, never a nav goal.
const gridDef = { active: false, p1: null, p2: null, cand: null, residual: null };
const gdEl = (id) => document.getElementById(id);
const gridSetter = gdEl('grid-setter');
const gridDefineBtn = gdEl('grid-define');
function gdList(id) {
  return gdEl(id).value.split(',').map((t) => t.trim()).filter((t) => t.length);
}
function gdSpacing(id) {
  const l = gdList(id).map(Number);
  if (!l.length || l.some((n) => !Number.isFinite(n) || n <= 0)) return null;
  return l.length === 1 ? l[0] : l;
}
function gdOffsets(n, spacing, f) {
  const gaps = Array.isArray(spacing) ? spacing : new Array(n - 1).fill(spacing);
  if (gaps.length !== n - 1) return null;
  const offs = [0];
  for (const g of gaps) offs.push(offs[offs.length - 1] + g * f);
  return offs;
}
function gdFillSelect(sel, labels, want) {
  const keep = want || sel.value;
  sel.innerHTML = '';
  for (const l of labels) {
    const o = document.createElement('option');
    o.value = o.textContent = l;
    sel.appendChild(o);
  }
  if (labels.includes(keep)) sel.value = keep;
}
function gdSyncSelects(reset) {
  const nums = gdList('gd-numbers'), lets = gdList('gd-letters');
  gdFillSelect(gdEl('gd-r1n'), nums, reset ? nums[0] : null);
  gdFillSelect(gdEl('gd-r1l'), lets, reset ? lets[0] : null);
  gdFillSelect(gdEl('gd-r2n'), nums, reset ? nums[nums.length - 1] : null);
  gdFillSelect(gdEl('gd-r2l'), lets, reset ? lets[lets.length - 1] : null);
}
// Rebuild the candidate grid from the form + taps; sets gridDef.cand/residual.
function gdRecompute() {
  gridDef.cand = null;
  gridDef.residual = null;
  const step = gdEl('gd-step'), resEl = gdEl('gd-residual');
  const confirmBtn = gdEl('gd-confirm');
  confirmBtn.disabled = true;
  resEl.textContent = '';
  const nums = gdList('gd-numbers'), lets = gdList('gd-letters');
  const f = gdEl('gd-units').value === 'ft' ? FT_M : 1;
  const ns = gdSpacing('gd-nspace'), ls = gdSpacing('gd-lspace');
  if (nums.length < 2 || lets.length < 2) { step.textContent = 'Enter at least 2 number lines and 2 letter lines'; return; }
  if (new Set(nums.concat(lets).map((t) => t.toUpperCase())).size !== nums.length + lets.length) {
    step.textContent = 'Labels must be unique across both axes'; return;
  }
  if (ns === null || ls === null) { step.textContent = 'Enter spacings (positive numbers)'; return; }
  const no = gdOffsets(nums.length, ns, f), lo = gdOffsets(lets.length, ls, f);
  if (!no || !lo) { step.textContent = 'Need one spacing (uniform) or one per gap (' + (nums.length - 1) + ' numbers, ' + (lets.length - 1) + ' letters)'; return; }
  if (!gridDef.p1) { step.textContent = 'Tap 1/2: intersection ' + gdEl('gd-r1n').value + ' / ' + gdEl('gd-r1l').value + ' on the map'; return; }
  if (!gridDef.p2) { step.textContent = 'Tap 2/2: intersection ' + gdEl('gd-r2n').value + ' / ' + gdEl('gd-r2l').value + ' on the map'; return; }
  const ref = (sel) => ({ i: nums.indexOf(gdEl(sel + 'n').value), j: lets.indexOf(gdEl(sel + 'l').value) });
  const r1 = ref('gd-r1'), r2 = ref('gd-r2');
  const u1 = no[r1.i], v1 = lo[r1.j], u2 = no[r2.i], v2 = lo[r2.j];
  const dg = Math.hypot(u2 - u1, v2 - v1);
  const dx = gridDef.p2.x - gridDef.p1.x, dy = gridDef.p2.y - gridDef.p1.y;
  const dm = Math.hypot(dx, dy);
  if (dg < 1e-9) { step.textContent = 'Tap 1 and tap 2 must be different intersections'; return; }
  if (dm < 1e-9) { step.textContent = 'Taps coincide'; return; }
  const yaw = Math.atan2(dy, dx) - Math.atan2(v2 - v1, u2 - u1);
  const c = Math.cos(yaw), s = Math.sin(yaw);
  gridDef.cand = {
    mode: 'structural',
    version: 1, units: gdEl('gd-units').value,
    origin: { x: gridDef.p1.x - (c * u1 - s * v1), y: gridDef.p1.y - (s * u1 + c * v1),
      yaw_rad: Math.atan2(s, c) },
    numbers: { labels: nums, offsets_m: no }, letters: { labels: lets, offsets_m: lo },
    bay_anchor: 'lower',
  };
  gridDef.residual = Math.abs(dm - dg);
  step.textContent = 'Review the dashed preview, then Confirm (re-tap to redo)';
  resEl.textContent = 'residual ' + gridDef.residual.toFixed(2) + ' m ('
    + (gridDef.residual / FT_M).toFixed(1) + ' ft): measured vs configured distance between the two taps'
    + (gridDef.residual > 0.5 ? ' - LARGE, check spacings / taps' : '');
  resEl.style.color = gridDef.residual > 0.5 ? '#e5484d' : '';
  confirmBtn.disabled = false;
}
function drawGridDefPreview() {
  if (!gridDef.active || !grid) return;
  if (gridDef.cand) sgDraw(gridDef.cand, '#e8590c', true);
  mapCtx.save();
  mapCtx.fillStyle = '#e8590c';
  for (const p of [gridDef.p1, gridDef.p2]) {
    if (!p) continue;
    const c = worldToCanvas(p.x, p.y);
    mapCtx.beginPath();
    mapCtx.arc(c.x, c.y, 5, 0, 2 * Math.PI);
    mapCtx.fill();
  }
  mapCtx.restore();
}
function gridDefExit() {
  gridDef.active = false;
  gridDef.p1 = gridDef.p2 = gridDef.cand = gridDef.residual = null;
  gridSetter.hidden = true;
  gridDefineBtn.classList.remove('selected');
  drawMap();
}
function gridDefTap(w) {
  if (!gridDef.p1 || gridDef.p2) { gridDef.p1 = w; gridDef.p2 = null; } else { gridDef.p2 = w; }
  gdRecompute();
  drawMap();
}
function gridDefStart() {
  if (tagSet.active) tagSetExit();
  if (poseMode) { poseMode = 0; posePin = null; setPoseBtn.textContent = 'Set pose'; }
  if (areaMode) areaReset();
  gridDef.active = true;
  gridDef.p1 = gridDef.p2 = null;
  gdEl('gd-result').textContent = '';
  if (sgrid && sgrid.mode !== 'auto') {   // prefill from a saved STRUCTURAL grid (an auto grid has no drawing spacings to edit)
    const f = sgrid.units === 'ft' ? FT_M : 1;
    const gaps = (o) => o.slice(1).map((x, k) => +((x - o[k]) / f).toFixed(4));
    gdEl('gd-numbers').value = sgrid.numbers.labels.join(',');
    gdEl('gd-letters').value = sgrid.letters.labels.join(',');
    gdEl('gd-nspace').value = gaps(sgrid.numbers.offsets_m).join(',');
    gdEl('gd-lspace').value = gaps(sgrid.letters.offsets_m).join(',');
    gdEl('gd-units').value = sgrid.units;
  }
  gdSyncSelects(true);
  gridSetter.hidden = false;
  gridDefineBtn.classList.add('selected');
  gdRecompute();
  drawMap();
}
gridDefineBtn.addEventListener('click', (ev) => { ev.preventDefault(); if (gridDef.active) gridDefExit(); else gridDefStart(); });
gdEl('gd-cancel').addEventListener('click', gridDefExit);
for (const id of ['gd-numbers', 'gd-letters']) {
  gdEl(id).addEventListener('input', () => { gdSyncSelects(false); gdRecompute(); drawMap(); });
}
for (const id of ['gd-nspace', 'gd-lspace', 'gd-units', 'gd-r1n', 'gd-r1l', 'gd-r2n', 'gd-r2l']) {
  gdEl(id).addEventListener('input', () => { gdRecompute(); drawMap(); });
}
gdEl('gd-confirm').addEventListener('click', async () => {
  if (!gridDef.cand) return;
  const rEl = gdEl('gd-result');
  if (!confirm('Save this grid for the active map?'
    + (gridDef.residual > 0.5 ? '\n\nResidual is ' + gridDef.residual.toFixed(2) + ' m - the taps disagree with the spacings.' : ''))) return;
  gdEl('gd-confirm').disabled = true;
  try {
    const res = await fetch(FLEET_API + '/grid', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(gridDef.cand),
    });
    const body = await res.json();
    if (!res.ok) { rEl.textContent = 'failed: ' + (body.error || res.status); gdEl('gd-confirm').disabled = false; return; }
    sgrid = body.grid;
    sgRefreshView();
    navState.textContent = 'structural grid saved (replaces the auto grid)';
    gridDefExit();
  } catch (e) {
    rEl.textContent = 'failed: ' + e;
    gdEl('gd-confirm').disabled = false;
  }
});
gdEl('gd-clear').addEventListener('click', async () => {
  if (!confirm('Delete the saved grid for the active map?')) return;
  try {
    await fetch(FLEET_API + '/grid', { method: 'DELETE' });
    sgrid = null;
    sgServerNone = true;
    sgRefreshView();
    gdEl('gd-result').textContent = 'saved grid cleared (the auto grid is regenerated from the map)';
    drawMap();
    sgMaybeAutoCreate();
  } catch (e) { gdEl('gd-result').textContent = 'failed: ' + e; }
});

const goalPub = new ROSLIB.Topic({
  ros, name: '/goal_pose', messageType: 'geometry_msgs/msg/PoseStamped',
});

// --- manual pose pin (amcl /initialpose from the map) -----------------------
// Two taps, like the coverage polygon (touch drags scroll the page): first
// places the position, second points the heading (pin -> tap direction).
// Deliberately does NOT re-arm tag_relocalizer: a tag surveyed while
// mislocalized would win the bad pose right back — re-survey the tag
// (detect_tags) after pinning, then reseed.
const setPoseBtn = document.getElementById('set-pose');
let poseMode = 0;      // 0 off, 1 awaiting position tap, 2 awaiting heading tap
let posePin = null;    // {x, y} map frame, set by the first tap
setPoseBtn.addEventListener('click', () => {
  poseMode = poseMode ? 0 : 1;
  posePin = null;
  setPoseBtn.textContent = poseMode ? 'Tap position…' : 'Set pose';
  drawMap();
});
function mapTap(ev) {
  if (!grid || !mapView) return;
  if (tagSet.active) {       // tag placement owns every tap: never a goal
    tagSetTap(canvasToWorld(ev));
    return;
  }
  if (gridDef.active) {      // grid definition owns every tap: never a goal
    gridDefTap(canvasToWorld(ev));
    return;
  }
  if (poseMode === 1) {
    posePin = canvasToWorld(ev);
    poseMode = 2;
    setPoseBtn.textContent = 'Tap heading…';
    drawMap();
    return;
  }
  if (poseMode === 2) {
    const h = canvasToWorld(ev);
    const yaw = Math.atan2(h.y - posePin.y, h.x - posePin.x);
    const cov = new Array(36).fill(0);
    cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.0685; // ~15 deg, same as activateMap
    initialPosePub.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: {
        pose: {
          position: { x: posePin.x, y: posePin.y, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
        },
        covariance: cov,
      },
    }));
    navState.textContent = 'pose set (' + posePin.x.toFixed(2) + ', '
      + posePin.y.toFixed(2) + ') yaw ' + (yaw * 180 / Math.PI).toFixed(0) + '°';
    poseMode = 0;
    posePin = null;
    setPoseBtn.textContent = 'Set pose';
    drawMap();
    return;
  }
  if (areaMode) {
    areaPts.push(canvasToWorld(ev));
    areaBtn.textContent = 'Finish (' + areaPts.length + ')';
    drawMap();
    return;
  }
  // Tag-marker hit test first: a tap within 12 canvas px of a dot opens the
  // popup instead of sending a goal (tap empty space to navigate as before).
  if (uhfShowTags && uhfTags.length) {
    const pp = canvasPoint(ev);
    const px = pp.x, py = pp.y;
    for (const t of uhfTags) {
      const c = worldToCanvas(t.est_pose.x, t.est_pose.y);
      if ((c.x - px) ** 2 + (c.y - py) ** 2 <= 12 ** 2) {
        openTagPop(t);
        return;
      }
    }
    if (uhfPopTag) closeTagPop();   // tap elsewhere dismisses an open popup
  }
  const wp = canvasToWorld(ev);
  const wx = wp.x, wy = wp.y;
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
}

// --- map pointer interaction: tap vs drag vs pinch ------------------------------
// A press that stays within TAP_SLOP px and never became a 2-finger gesture is a
// TAP and goes to mapTap() on pointerup — exactly the old click behaviour. Anything
// that moved further is a pan and places/sends nothing. No click listener exists,
// so a drag can never also fire one. No double-tap zoom: it would have to delay
// every single tap to tell them apart (and a double tap would send two goals).
const TAP_SLOP = 5;
const mapPtrs = new Map();   // pointerId -> {x, y} canvas px
let mapGesture = null;       // {moved, multi, startPt, startView, pinch?}

function mapPtrMid() {
  const p = [...mapPtrs.values()];
  return { x: (p[0].x + p[1].x) / 2, y: (p[0].y + p[1].y) / 2,
    d: Math.hypot(p[0].x - p[1].x, p[0].y - p[1].y) };
}
mapCanvas.addEventListener('pointerdown', (ev) => {
  if (!grid || !mapView) return;
  if (ev.pointerType === 'mouse' && ev.button !== 0) return;
  mapCanvas.setPointerCapture(ev.pointerId);
  mapPtrs.set(ev.pointerId, canvasPoint(ev));
  if (mapPtrs.size === 1) {
    mapGesture = { moved: false, multi: false, startPt: canvasPoint(ev), startView: mapView };
  } else if (mapPtrs.size === 2 && mapGesture) {
    const m = mapPtrMid();
    mapGesture.multi = true;
    mapGesture.pinch = { mid: { x: m.x, y: m.y }, d: m.d, view: mapView };
    mapDragging = true;
  }
});
mapCanvas.addEventListener('pointermove', (ev) => {
  if (!mapPtrs.has(ev.pointerId) || !mapGesture) return;
  const pt = canvasPoint(ev);
  mapPtrs.set(ev.pointerId, pt);
  if (mapPtrs.size >= 2 && mapGesture.pinch) {
    const m = mapPtrMid();
    const g = mapGesture.pinch;
    mapSetView(MV.pinch(g.view, mapSize, g.mid, g.d, m, m.d, mapFitK()), true);
    return;
  }
  if (mapGesture.multi) return;   // one finger left over from a pinch: ignore
  const dx = pt.x - mapGesture.startPt.x, dy = pt.y - mapGesture.startPt.y;
  if (!mapGesture.moved) {
    if (Math.hypot(dx, dy) < TAP_SLOP) return;
    mapGesture.moved = true;
    mapDragging = true;
    mapCanvas.classList.add('panning');
  }
  mapSetView(MV.panBy(mapGesture.startView, dx, dy));
});
function mapPtrEnd(ev, cancelled) {
  if (!mapPtrs.delete(ev.pointerId) || !mapGesture) return;
  if (mapPtrs.size === 0) {
    const g = mapGesture;
    mapGesture = null;
    mapDragging = false;
    mapCanvas.classList.remove('panning');
    if (!cancelled && !g.moved && !g.multi) mapTap(ev);
  } else {
    mapGesture.pinch = null;   // pinch over; remaining finger must not tap or pan
    mapGesture.multi = true;
  }
}
mapCanvas.addEventListener('pointerup', (ev) => mapPtrEnd(ev, false));
mapCanvas.addEventListener('pointercancel', (ev) => mapPtrEnd(ev, true));
mapCanvas.addEventListener('wheel', (ev) => {
  if (!mapView) return;
  ev.preventDefault();
  const px = ev.deltaMode === 1 ? 16 : 1;
  const rate = ev.ctrlKey ? 0.01 : 0.0015;   // ctrl+wheel = trackpad pinch
  const p = canvasPoint(ev);
  mapZoomBy(Math.exp(-ev.deltaY * px * rate), p.x, p.y);
}, { passive: false });

// Header controls
document.getElementById('map-zoom-in').addEventListener('click', () => mapZoomBy(1.5));
document.getElementById('map-zoom-out').addEventListener('click', () => mapZoomBy(1 / 1.5));
document.getElementById('map-fit').addEventListener('click', () => {
  mapFollow = false;
  mapFitView();
  mapSyncControls();
  drawMap();
});
document.getElementById('map-center').addEventListener('click', () => {
  if (!robotPose || !mapView) return;
  mapSetView(MV.centerOn(mapView, robotPose.x, robotPose.y));
});
document.getElementById('map-follow').addEventListener('click', () => {
  mapFollow = !mapFollow;
  mapSyncControls();
  drawMap();
});

// Large view: fixed overlay filling the window (+ Fullscreen API when available).
// STOP / E-STOP in the header proxy the main buttons, so the same stop path runs.
const mapExpandBtn = document.getElementById('map-expand');
function mapSetExpanded(on) {
  navPanel.classList.toggle('expanded', on);
  document.body.classList.toggle('map-expanded', on);
  mapExpandBtn.textContent = on ? 'Close' : 'Expand';
  // Resize before the next paint is handled by the ResizeObserver.
}
mapExpandBtn.addEventListener('click', () => {
  const on = !navPanel.classList.contains('expanded');
  mapSetExpanded(on);
  try {
    if (on && navPanel.requestFullscreen) {
      Promise.resolve(navPanel.requestFullscreen()).catch(() => { /* overlay only */ });
    } else if (!on && document.fullscreenElement) {
      document.exitFullscreen();
    }
  } catch (e) { /* overlay only */ }
});
document.addEventListener('fullscreenchange', () => {
  // User left fullscreen (Esc handled by the browser): close the overlay too.
  if (!document.fullscreenElement && navPanel.classList.contains('expanded')) {
    mapSetExpanded(false);
  }
});
document.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape' && navPanel.classList.contains('expanded')) {
    mapSetExpanded(false);
    if (document.fullscreenElement) document.exitFullscreen();
  }
});
document.getElementById('map-stop').addEventListener('click', () => {
  document.getElementById('stop').click();
});
document.getElementById('map-estop').addEventListener('click', () => {
  document.getElementById('estop').click();
});
{
  // Mirror the main E-STOP's label/engaged state onto the expanded-view copy.
  const src = document.getElementById('estop'), dst = document.getElementById('map-estop');
  const sync = () => { dst.textContent = src.textContent; dst.classList.toggle('engaged', src.classList.contains('engaged')); };
  new MutationObserver(sync).observe(src, { attributes: true, childList: true, characterData: true, subtree: true });
  sync();
}

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
  navBusyNow = NAV_BUSY_STATES.includes(state);
  if (!navBusyNow && plan) { plan = null; drawMap(); }  // goal over: drop the stale path
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

// --- map/camera split slider -------------------------------------------------------
// Drag #stage-split to trade map vs camera width (desktop row layout only).
// The fraction lives in --stage-split on #stage; localStorage persists it
// per-browser (a convenience, not state — wrapped per the storage rules).
const stageEl = document.getElementById('stage');
const stageSplit = document.getElementById('stage-split');
const SPLIT_KEY = 'scout.stageSplit';
const SPLIT_DEFAULT = 62;         // % of the row given to the map

function applySplit(pct) {
  pct = Math.min(80, Math.max(25, pct));  // profile-exempt: UI split %, not publish_hz
  stageEl.style.setProperty('--stage-split', pct + '%');
  return pct;
}
try {
  const saved = parseFloat(localStorage.getItem(SPLIT_KEY));
  if (!isNaN(saved)) applySplit(saved);
} catch (e) { /* private mode etc. — default split stands */ }

stageSplit.addEventListener('pointerdown', (ev) => {
  ev.preventDefault();
  stageSplit.setPointerCapture(ev.pointerId);
  stageSplit.classList.add('dragging');
});
stageSplit.addEventListener('pointermove', (ev) => {
  if (!stageSplit.classList.contains('dragging')) return;
  const r = stageEl.getBoundingClientRect();
  const pct = applySplit(((ev.clientX - r.left) / r.width) * 100);
  try { localStorage.setItem(SPLIT_KEY, pct); } catch (e) { /* ignore */ }
});
stageSplit.addEventListener('pointerup', (ev) => {
  stageSplit.releasePointerCapture(ev.pointerId);
  stageSplit.classList.remove('dragging');
});
stageSplit.addEventListener('dblclick', () => {
  applySplit(SPLIT_DEFAULT);
  try { localStorage.removeItem(SPLIT_KEY); } catch (e) { /* ignore */ }
});
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

// --- UHF RFID (M7E Hecto, ADR-0032) -------------------------------------------------
// Same manual gate as the Flipper radios (/uhf/enable, SetBool). The badge
// renders from uhf_node's latched /uhf/status; the tag list renders from the
// companion's /uhf/registry (per-EPC centroid estimate), NOT raw /uhf/reads —
// at up to 10 batches/s the raw feed is the wrong browser surface, so it only
// feeds the reads/s counter.
const uhfState = document.getElementById('uhf-state');
const uhfList = document.getElementById('uhf-list');
const uhfRate = document.getElementById('uhf-rate');
const uhfResult = document.getElementById('uhf-result');
const uhfEnableSrv = new ROSLIB.Service({
  ros, name: '/uhf/enable', serviceType: 'std_srvs/srv/SetBool',
});

function uhfSetEnabled(on) {
  uhfEnableSrv.callService(new ROSLIB.ServiceRequest({ data: on }),
    (res) => { uhfResult.textContent = res.message; },
    (err) => { uhfResult.textContent = 'error: ' + err; });
}
document.getElementById('uhf-enable').addEventListener('click', () => uhfSetEnabled(true));
document.getElementById('uhf-disable').addEventListener('click', () => uhfSetEnabled(false));

new ROSLIB.Topic({
  ros, name: '/uhf/status', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  let s;
  try { s = JSON.parse(msg.data); } catch (e) { return; }
  uhfState.textContent = !s.connected ? 'no reader'
    : !s.enabled ? 'disabled'
      : s.throttled ? 'THROTTLED' : 'scanning';
  uhfState.classList.toggle('bad', !s.connected || s.throttled);
});

// reads/s over a rolling 2 s window, from batch sizes.
let uhfReadTimes = [];
new ROSLIB.Topic({
  ros, name: '/uhf/reads', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  let b;
  try { b = JSON.parse(msg.data); } catch (e) { return; }
  const now = Date.now();
  uhfReadTimes.push([now, (b.reads || []).length]);
  uhfReadTimes = uhfReadTimes.filter(([t]) => now - t < 2000);
  const n = uhfReadTimes.reduce((acc, [, c]) => acc + c, 0);
  uhfRate.textContent = n ? `${(n / 2).toFixed(0)} reads/s` : '';
});

new ROSLIB.Topic({
  ros, name: '/uhf/registry', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  let reg;
  try { reg = JSON.parse(msg.data); } catch (e) { return; }
  // Map markers: localized tags only; the corner Tags toggle shows up with them.
  uhfTags = (reg.tags || []).filter((t) => t.est_pose);
  document.getElementById('mo-layers').hidden = !(uhfTags.length || aprilTags.length);
  if (uhfPopTag && !uhfTags.some((t) => t.epc === uhfPopTag)) closeTagPop();
  drawMap();
  uhfList.innerHTML = '';
  for (const t of (reg.tags || []).slice(0, 20)) {
    const li = document.createElement('li');
    const where = t.est_pose
      ? `(${t.est_pose.x.toFixed(2)}, ${t.est_pose.y.toFixed(2)}) ±${t.spread_m}m`
      : 'no position yet';
    li.textContent = `${t.epc} · x${t.count} · ${where} · ${t.last_rssi_dbm} dBm`;
    uhfList.appendChild(li);
  }
});

// Tag marker popup + the corner Tags layer toggle.
const tagPop = document.getElementById('tag-pop');
const tagPopTitle = document.getElementById('tag-pop-title');
const tagPopMeta = document.getElementById('tag-pop-meta');
const tagsToggle = document.getElementById('uhf-tags-toggle');
let tagPopPose = null;   // est_pose of the open popup's tag (Drive here target)

function openTagPop(t) {
  uhfPopTag = t.epc;
  tagPopPose = t.est_pose;
  tagPopTitle.textContent = t.epc;
  tagPopMeta.textContent =
    `x${t.count} reads · ±${t.spread_m} m · last ${t.last_rssi_dbm} dBm · ` +
    (t.last_seen_utc || '').replace(/^.*T/, '').replace(/\..*$/, '') + ' UTC';
  tagPop.hidden = false;
  drawMap();
}

function closeTagPop() {
  uhfPopTag = null;
  tagPopPose = null;
  tagPop.hidden = true;
  drawMap();
}

document.getElementById('tag-pop-close').addEventListener('click', closeTagPop);
document.getElementById('tag-pop-goto').addEventListener('click', () => {
  if (!tagPopPose) return;
  const wx = tagPopPose.x, wy = tagPopPose.y;
  const yaw = robotPose ? Math.atan2(wy - robotPose.y, wx - robotPose.x) : 0;
  goalPub.publish(new ROSLIB.Message({
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
    pose: {
      position: { x: wx, y: wy, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
    },
  }));
  navState.textContent = 'goal sent (' + wx.toFixed(2) + ', ' + wy.toFixed(2) + ')';
  closeTagPop();
});

tagsToggle.addEventListener('click', () => {
  uhfShowTags = !uhfShowTags;
  tagsToggle.classList.toggle('selected', uhfShowTags);
  if (!uhfShowTags) closeTagPop(); else drawMap();
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

// --- Bluetooth controller recovery (fleet_status /api/bluetooth) ---------------------
// Never touches ROS or motion. Polls with the system poll; every 2 s while an op runs.
const btChip = document.getElementById('bt-chip');
const btOp = document.getElementById('bt-op');
const btBtns = ['bt-reconnect', 'bt-restart', 'bt-repair'].map((id) => document.getElementById(id));
let btTimer = null;

function renderBluetooth(b) {
  const p = b.pad || {};
  const running = b.op && b.op.state === 'running';
  let txt, cls = '';
  if (!p.mac || !p.paired) { txt = 'not paired'; cls = 'bad'; }
  else if (!p.connected) { txt = 'paired, not connected'; cls = 'warn'; }
  else txt = 'connected';
  if (p.battery != null) txt += ` · ${p.battery}%`;
  txt += ` · js0 ${b.js0_present ? '✓' : '✗'}`;
  if (!b.adapter_powered) txt += ' · adapter off';
  btChip.textContent = txt;
  btChip.className = 'svc-stat ' + cls;
  btBtns.forEach((x) => { x.disabled = running; });
  const o = b.op || {};
  btOp.textContent = o.state === 'running' ? `${o.name}: running…`
    : (o.state === 'ok' || o.state === 'error') ? `${o.name}: ${o.state} — ${o.msg}` : '';
  clearTimeout(btTimer);
  if (running) btTimer = setTimeout(refreshBluetooth, 2000);
}

async function refreshBluetooth() {
  try {
    renderBluetooth(await fetch(FLEET_API + '/bluetooth').then((r) => r.json()));
  } catch (e) {
    btChip.textContent = 'fleet_status unreachable';
    btChip.className = 'svc-stat dim';
    btBtns.forEach((x) => { x.disabled = true; });
  }
}

async function btAction(name, confirmMsg) {
  if (confirmMsg && !confirm(confirmMsg)) return;
  btBtns.forEach((x) => { x.disabled = true; });
  try {
    await fetch(`${FLEET_API}/bluetooth/${name}`, { method: 'POST' });
  } catch (e) { /* offline; refresh shows it */ }
  refreshBluetooth();
}
document.getElementById('bt-reconnect').addEventListener('click', () => btAction('reconnect'));
document.getElementById('bt-restart').addEventListener('click', () => btAction('restart',
  'Restart bluetooth.service on the Pi? The controller drops and must reconnect.'));
document.getElementById('bt-repair').addEventListener('click', () => btAction('repair',
  "Hold the controller's sync button until the light blinks fast, then press OK. Re-pair takes ~40 s."));
refreshBluetooth();
setInterval(refreshBluetooth, 30000);

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

// Plain-language phrase for each slam mode, shown in the "Active now" card.
const MODE_PHRASE = {
  localization: 'Localizing on the saved map',
  continue: 'Mapping — extending the saved map',
  new: 'Mapping — building a fresh map',
  auto: 'Auto — continues a saved map if one exists',
};

// The map's "NOW" chip (overlay #site-active): which site/map/floor is live and
// what the robot is doing. Everything comes from activeSiteMeta.
function renderActiveCard(activeName) {
  const card = document.getElementById('site-active');
  if (!activeSiteMeta || !activeName) { card.hidden = true; return; }
  const map = activeSiteMeta.active_map;
  const entry = (map && (activeSiteMeta.maps || {})[map]) || {};
  const mode = activeSiteMeta.slam_mode || 'auto';
  document.getElementById('active-site-name').textContent =
    (activeSiteMeta.display_name || activeName) + ' /';
  document.getElementById('active-map-name').textContent =
    map ? (entry.label || map) : 'no map yet';
  const floorEl = document.getElementById('active-map-floor');
  if (entry.floor !== null && entry.floor !== undefined) {
    floorEl.textContent = 'F' + entry.floor;
    floorEl.style.display = '';
  } else {
    floorEl.style.display = 'none';
  }
  let line = MODE_PHRASE[mode] || mode;
  document.getElementById('active-mode-line').innerHTML = line;
  card.hidden = false;
}

// Map overlay: the Navigate | Map intent toggle (#mo-intent). Two intents map
// onto the four slam modes — Navigate = localization (finished map, instant
// floor swaps), Map = auto (build/extend). New/Continue stay in
// the Site tab's "Advanced" section. Kept in sync with slam_mode.
function renderIntent() {
  const wrap = document.getElementById('mo-intent');
  if (!activeSiteMeta || !activeSiteMeta.active_map) { wrap.hidden = true; return; }
  wrap.hidden = false;
  const mode = activeSiteMeta.slam_mode || 'auto';
  const navBtn = wrap.querySelector('[data-intent="nav"]');
  const mapBtn = wrap.querySelector('[data-intent="map"]');
  navBtn.classList.toggle('on', mode === 'localization');
  mapBtn.classList.toggle('on', mode !== 'localization');
  // Precondition: Navigate (localization) needs a saved grid map.
  const entry = (activeSiteMeta.maps || {})[activeSiteMeta.active_map] || {};
  navBtn.disabled = !entry.grid;
  navBtn.title = entry.grid
    ? 'Drive the finished map — floors swap instantly'
    : 'Save a grid map first (switch to Map, drive, then Save)';
}

// Map overlay: the floor stack (#mo-floors). Floors are maps; tap to swap.
// Hidden unless the site has more than one map.
function renderFloorStack() {
  const wrap = document.getElementById('mo-floors');
  const maps = (activeSiteMeta && activeSiteMeta.maps) || {};
  const names = Object.keys(maps);
  if (names.length < 2) { wrap.hidden = true; return; }
  wrap.hidden = false;
  wrap.innerHTML = '<span class="fl-label">FLR</span>';
  const active = activeSiteMeta.active_map;
  // Top floor at the top; maps without a floor sort by name after.
  names.sort((a, b) => {
    const fa = maps[a].floor, fb = maps[b].floor;
    if (fa != null && fb != null) return fb - fa;
    if (fa != null) return -1;
    if (fb != null) return 1;
    return a.localeCompare(b);
  });
  for (const name of names) {
    const m = maps[name];
    const btn = document.createElement('button');
    btn.textContent = (m.floor != null) ? String(m.floor) : name.slice(0, 3);
    btn.title = m.label || name;
    if (name === active) btn.classList.add('on');
    btn.addEventListener('click', () => activateMap(name));
    wrap.appendChild(btn);
  }
}

// Intent toggle click -> the two-way mode switch (setSlamMode confirms + gates).
document.querySelectorAll('#mo-intent button').forEach((btn) => {
  btn.addEventListener('click', () => {
    setSlamMode(btn.dataset.intent === 'nav' ? 'localization' : 'auto');
  });
});

function renderSites(data) {
  activeSiteMeta = (data.sites || []).find((s) => s.name === data.active) || null;
  const mapCount = Object.keys((activeSiteMeta && activeSiteMeta.maps) || {}).length;
  siteState.textContent = data.active
    ? `${mapCount} map${mapCount === 1 ? '' : 's'}`
    : 'no active site';
  renderActiveCard(data.active);
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
    const isActive = name === active;
    row.className = 'map-row' + (isActive ? ' is-live' : '');
    // Readable capability chips instead of [graph]/[grid] shorthand.
    const chips = [];
    if (isActive) chips.push('<span class="cap live">● running</span>');
    chips.push(m.grid
      ? '<span class="cap on">Localize-ready</span>'
      : '<span class="cap off">Not localizable</span>');
    if (m.posegraph) chips.push('<span class="cap on">Extendable</span>');
    if (m.unregistered) chips.push('<span class="cap warn">unregistered</span>');
    const nameHtml = m.label && m.label !== name
      ? `${m.label} <span style="color:var(--dim)">(${name})</span>` : name;
    const floorHtml = (m.floor !== null && m.floor !== undefined)
      ? `<span class="map-row-floor">Floor ${m.floor}</span>` : '';
    row.innerHTML = `
      <div class="map-row-head">
        <span class="map-row-name">${nameHtml}</span>
        ${floorHtml}
        <span class="svc-actions">
          <button data-map="${name}" ${isActive ? 'disabled' : ''}>${isActive ? 'Live now' : 'Activate'}</button>
        </span>
      </div>
      <div class="caps">${chips.join('')}</div>
    `;
    const btn = row.querySelector('button');
    if (btn) btn.addEventListener('click', () => activateMap(name));
    siteMapsEl.appendChild(row);
  }
  renderFloorStack();                     // map floor stack (#mo-floors)
}

const loadMapSrv = new ROSLIB.Service({
  ros, name: '/map_server/load_map',
  serviceType: 'nav2_msgs/srv/LoadMap',
});
const initialPosePub = new ROSLIB.Topic({
  ros, name: '/initialpose',
  messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
});
// Advertise at load (same race as /coverage_box): a publish on a
// just-advertised topic can be dropped before DDS discovery matches, which
// would eat the first Set-pose pin after a page load.
ros.on('connection', () => initialPosePub.advertise());
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
  if (!activeSiteMeta) { siteModeDesc.textContent = ''; renderIntent(); return; }
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
  renderIntent();   // keep the map's Navigate|Map toggle in sync
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
  // Safe save: write to `_pending`, then fleet_status validates (refuses a
  // trivially small graph, never shrinks an existing good map without force)
  // and installs it as <name>. The grid goes first: SaveMap result 255 = slam
  // never received /map, i.e. there is nothing real to save.
  const PENDING = '/ros_ws/src/sites/active/maps/_pending';
  siteResult.textContent = 'saving grid map…';
  saveGridSrv.callService(
    new ROSLIB.ServiceRequest({ name: { data: PENDING } }),
    (gres) => {
      if (gres.result === 255) {
        siteResult.textContent = 'save REFUSED: slam_toolbox has not received /map yet (restarted?). Nothing written. Check the autosave: GET /api/map/autosave.';
        return;
      }
      const gridErr = gres.result === 0 ? null : `result ${gres.result}`;
      siteResult.textContent = 'serializing map…';
      serializeSrv.callService(
        new ROSLIB.ServiceRequest({ filename: PENDING }),
        () => commitMap(name, gridErr, false),
        (err) => { siteResult.textContent = 'serialize failed: ' + err; },
      );
    },
    (err) => { siteResult.textContent = 'grid save failed: ' + err; },
  );
});

async function commitMap(name, gridErr, force) {
  let res, body;
  try {
    res = await fetch(`${FLEET_API}/map/commit`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, force }),
    });
    body = await res.json();
  } catch (e) { siteResult.textContent = 'fleet_status unreachable — map NOT saved'; return; }
  if (res.status === 409 && body.needs_force) {
    if (confirm(body.error + '\n\nOverwrite anyway?')) return commitMap(name, gridErr, true);
    siteResult.textContent = 'save cancelled: ' + body.error;
    return;
  }
  if (!res.ok) { siteResult.textContent = 'save REFUSED: ' + (body.error || res.status); return; }
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
  try {   // carry a mapping-session grid over to the saved map's name
    await fetch(`${FLEET_API}/grid/adopt`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }),
    });
    // Extent re-fit against the saved files (creates the auto grid if none;
    // a structural grid is left alone).
    await fetch(`${FLEET_API}/grid/auto`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: '{}',
    });
    refreshStructGrid();
  } catch (e) { /* best-effort */ }
  siteResult.textContent = !body.grid
    ? `map "${name}" saved (posegraph only — grid save failed: ${gridErr}; localization mode needs the grid)`
    : `map "${name}" saved (posegraph + grid, ${body.bytes} B).`;
  if (confirm('Map saved. Restart slam + behaviors now to continue on it?')) {
    try {
      await fetch(`${FLEET_API}/containers/slam/restart`, { method: 'POST' });
      await fetch(`${FLEET_API}/containers/behaviors/restart`, { method: 'POST' });
    } catch (e) { /* ignore */ }
  }
  refreshSites();
}

async function refreshSites() {
  try {
    const res = await fetch(FLEET_API + '/sites');
    if (res.status === 404) { sitePanel.style.display = 'none'; return; }
    sitePanel.style.display = '';
    renderSites(await res.json());
  } catch (e) { siteState.textContent = 'offline'; }
  refreshAprilTags();
  refreshStructGrid();
}

// Surveyed AprilTags -> map markers. Rides the sites poll: after renderSites
// so activeSiteMeta is fresh (markers filter to the active map — a tag
// surveyed on another floor doesn't draw here).
async function refreshAprilTags() {
  try {
    const res = await fetch(FLEET_API + '/apriltags');
    if (!res.ok) return;
    const body = await res.json();
    const active = activeSiteMeta && activeSiteMeta.active_map;
    const next = (body.tags || []).filter((t) => t.map_x !== null
      && (!t.map_name || !active || t.map_name === active));
    if (JSON.stringify(next) !== JSON.stringify(aprilTags)) {
      aprilTags = next;
      document.getElementById('mo-layers').hidden = !(uhfTags.length || aprilTags.length);
      drawMap();
    }
  } catch (e) { /* fleet_status offline — markers just don't refresh */ }
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
