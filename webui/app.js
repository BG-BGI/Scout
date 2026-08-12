/* Scout web teleop.
 *
 * cmd_vel contract (same as joystick_teleop.py): publish at 25 Hz ONLY while
 * an input is active, then a 0.3 s burst of zeros on release, then silence —
 * so Foxglove / nav2 / the robot-side pad can own /cmd_vel when we're idle.
 * The RoboClaw's 200 ms deadman is the backstop: if this page dies mid-drive,
 * the robot coasts to a stop within 200 ms of the last message.
 */
'use strict';

// --- constants (mirror joystick_teleop.py) -----------------------------------
const PUBLISH_HZ = 25;
const STOP_GRACE_MS = 300;
const STICK_DEADZONE = 0.08;
const TURN_EXPO = 0.6;
const TRIGGER_DEADZONE = 0.03;
// Below this an in-place turn can't beat the flat front-left tire's drag —
// the left side stalls and only one side spins. Pure pivots get floored.
const PIVOT_FLOOR = 2.5;

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
const cmdVel = new ROSLIB.Topic({
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
    // Pure pivot: enforce the flat-tire floor so all four wheels turn.
    if (input.throttle === 0 && wz !== 0 && Math.abs(wz) < PIVOT_FLOOR) {
      wz = Math.sign(wz) * PIVOT_FLOOR;
    }
    publishTwist(input.throttle * maxLin, wz);
  } else if (now - lastActiveMs < STOP_GRACE_MS) {
    publishTwist(0, 0);   // zero burst, then silence
  }
}
setInterval(driveTick, 1000 / PUBLISH_HZ);

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
    (msg.voltage <= 16.5 ? ' batt-bad' : msg.voltage <= 17.5 ? ' batt-warn' : '');
});

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
  zeroBurst();
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
  const W = 480;
  if (mapCanvas.width !== W || mapCanvas.height !== Math.round(W * aspect)) {
    mapCanvas.width = W;
    mapCanvas.height = Math.round(W * aspect);
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
}

const goalPub = new ROSLIB.Topic({
  ros, name: '/goal_pose', messageType: 'geometry_msgs/msg/PoseStamped',
});
mapCanvas.addEventListener('click', (ev) => {
  if (!grid) return;
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

// Cancel = the NavigateToPose action's hidden cancel service; zeroed request
// cancels every active goal. This is the button Foxglove never had.
const cancelNavSrv = new ROSLIB.Service({
  ros, name: '/navigate_to_pose/_action/cancel_goal',
  serviceType: 'action_msgs/srv/CancelGoal',
});
function cancelNav() {
  cancelNavSrv.callService(new ROSLIB.ServiceRequest({
    goal_info: {
      goal_id: { uuid: new Array(16).fill(0) },
      stamp: { sec: 0, nanosec: 0 },
    },
  }),
  () => { navState.textContent = 'goal cancelled'; },
  (err) => { navState.textContent = 'cancel failed: ' + err; });
}
document.getElementById('cancel-goal').addEventListener('click', cancelNav);

// Nav status readout from the action's status topic (action_msgs/GoalStatus:
// 1 accepted, 2 executing, 3 canceling, 4 succeeded, 5 canceled, 6 aborted).
const navStatusTopic = new ROSLIB.Topic({
  ros, name: '/navigate_to_pose/_action/status',
  messageType: 'action_msgs/msg/GoalStatusArray',
});
const NAV_STATUS = { 1: 'accepted', 2: 'driving', 3: 'canceling', 4: 'arrived', 5: 'canceled', 6: 'aborted' };
navStatusTopic.subscribe((msg) => {
  if (!msg.status_list.length) return;
  const s = msg.status_list[msg.status_list.length - 1].status;
  if (NAV_STATUS[s]) navState.textContent = NAV_STATUS[s];
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
new ROSLIB.Topic({
  ros, name: '/patrol_status', messageType: 'std_msgs/msg/String',
}).subscribe((msg) => {
  // 'idle|<n>' or '<state>|<n>|<i>/<n>'
  const parts = msg.data.split('|');
  patrolState.textContent = parts[0] === 'idle'
    ? 'idle · ' + parts[1] + ' wp' : parts[0] + ' ' + parts[2];
});

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
    camState.textContent = 'live';
  });
  camImg.style.display = 'block';
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
