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

function readGamepad() {
  if (gamepadIndex === null) return null;
  const gp = navigator.getGamepads()[gamepadIndex];
  if (!gp) return null;
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

document.getElementById('stop').addEventListener('click', () => {
  stopTrickSrv.callService(new ROSLIB.ServiceRequest({}), () => {}, () => {});
  zeroBurst();
});

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
