import os
import struct
import threading

from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from scout.cmd_vel_source import CmdVelSource
from scout.node_util import run_node
from scout.robot_profile import load as _load_profile

# --- Xbox controller mapping (Linux joydev / xpad driver) --------------------
# Axis/button numbers come from the kernel joystick interface. If the robot
# behaves wrong, run `jstest /dev/input/js0` (or `ros2 topic echo /joy`) and
# read off the real numbers, then fix the constants below.
JOY_DEV = os.environ.get('JOY_DEV', '/dev/input/js0')

# NOTE: these are the numbers for the Xbox pad over Bluetooth (stock kernel
# driver). They differ from the USB/xpad layout (which puts LT=2, RT=5) —
# verified live with the resting/movement probe on this controller.
AXIS_LEFT_X = 0   # left stick horizontal: left = -1, right = +1  (turning)
AXIS_RT = 4       # right trigger: released = -32767, pressed = +32767  (forward)
AXIS_LT = 5       # left trigger:  released = -32767, pressed = +32767  (reverse)
AXIS_DPAD_X = 6   # D-pad left/right: left = -32767, right = +32767
AXIS_DPAD_Y = 7   # D-pad up/down:    up   = -32767, down  = +32767

# Linux joystick event: u32 time, s16 value, u8 type, u8 number (8 bytes).
_JS_EVENT = struct.Struct('<IhBB')
_JS_EVENT_AXIS = 0x02
_JS_EVENT_BUTTON = 0x01
_JS_INIT_FLAG = 0x80
BTN_A = 0   # Xbox A, both xpad USB and stock Bluetooth HID mappings

# Cross-surface cmd_vel contract + caps live in robot_profile.yaml (SSOT). The
# publish rate + stop-burst are now owned by CmdVelSource (also profile-driven).
_PROFILE = _load_profile()
PUBLISH_HZ = _PROFILE['publish_hz']   # compute cadence (CmdVelSource re-publishes)
STICK_DEADZONE = 0.08      # ignore small left-stick noise so the robot tracks straight
TURN_EXPO = 0.6            # turn-stick response curve: 0 = linear, 1 = pure cubic.
                           # Higher = gentler near center; full deflection still = max.
TRIGGER_DEADZONE = 0.03    # ignore trigger rest noise

# Live-adjustable speed limits (D-pad), with hard caps and per-press steps.
# NB the driver clamps at roboclaw.yaml's real caps (1.0 m/s, 3.0 rad/s).
LINEAR_MIN, LINEAR_MAX = 0.05, _PROFILE['linear_cap']      # m/s
ANGULAR_MIN, ANGULAR_MAX = 0.5, _PROFILE['angular_cap']    # rad/s
LINEAR_DEFAULT, ANGULAR_DEFAULT = 0.35, 1.5
LINEAR_STEP, ANGULAR_STEP = 0.05, 0.5


class JoystickTeleopNode(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        self._cmd = CmdVelSource(self, 'joy', hz=PUBLISH_HZ)

        # Input state, updated by the reader thread, read by the publish timer.
        self._rt = 0.0          # forward throttle  0..1
        self._lt = 0.0          # reverse throttle  0..1
        self._turn = 0.0        # turn command     -1..1
        self._dpad_x = 0        # -1 / 0 / +1, for edge detection
        self._dpad_y = 0
        self._max_linear = LINEAR_DEFAULT
        self._max_angular = ANGULAR_DEFAULT

        # Follow-me toggle (D-pad up). Actual state tracked from /follow_status
        # so the toggle stays truthful if the mode was started/stopped elsewhere
        # (web UI, service call).
        self._follow_active = False
        self._follow_start = self.create_client(Trigger, 'follow_me/start')
        self._follow_stop = self.create_client(Trigger, 'follow_me/stop')
        self.create_subscription(String, 'follow_status', self._on_follow_status, 10)

        # A button marks a patrol waypoint. The reader thread only sets a
        # flag; the publish timer (executor thread) makes the service call.
        self._mark_requested = False
        self._patrol_mark = self.create_client(Trigger, 'patrol/mark')

        self._stop = False
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info(
            'Joystick teleop on %s (max linear %.2f m/s, max angular %.2f rad/s)'
            % (JOY_DEV, self._max_linear, self._max_angular))

    # --- Controller reading (background thread) ------------------------------
    def _reader_loop(self):
        """Read joystick events until stop or the device disappears.

        enable_joystick on the launch file decides whether this node runs at all —
        no reconnect poll. Missing device at start is a hard error; unplug mid-run
        zeros inputs and ends the reader so a held throttle cannot keep driving.
        """
        try:
            dev = open(JOY_DEV, 'rb', buffering=0)
        except OSError as exc:
            self.get_logger().error('Cannot open %s: %s' % (JOY_DEV, exc))
            return

        try:
            with dev:
                while not self._stop:
                    data = dev.read(_JS_EVENT.size)
                    if not data or len(data) < _JS_EVENT.size:
                        break
                    _t, value, etype, number = _JS_EVENT.unpack(data)
                    self._handle_event(value, etype, number)
        except OSError as exc:
            self.get_logger().warn('Joystick read failed: %s' % exc)

        self._zero_inputs()
        if not self._stop:
            self.get_logger().warn('Controller gone — inputs zeroed')

    def _handle_event(self, value, etype, number):
        if etype & ~_JS_INIT_FLAG == _JS_EVENT_BUTTON:
            if number == BTN_A and value == 1 and not (etype & _JS_INIT_FLAG):
                self._mark_requested = True
            return
        if etype & ~_JS_INIT_FLAG != _JS_EVENT_AXIS:
            return  # other buttons unused
        if number == AXIS_RT:
            self._rt = self._trigger_frac(value)
        elif number == AXIS_LT:
            self._lt = self._trigger_frac(value)
        elif number == AXIS_LEFT_X:
            self._turn = self._stick_frac(value)
        elif number == AXIS_DPAD_Y:
            self._on_dpad_y(value)
        elif number == AXIS_DPAD_X:
            self._on_dpad_x(value)

    @staticmethod
    def _trigger_frac(value):
        # Released = -32767, fully pressed = +32767 -> 0..1
        frac = (value + 32767.0) / 65534.0
        frac = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)
        return 0.0 if frac < TRIGGER_DEADZONE else frac

    @staticmethod
    def _stick_frac(value):
        frac = max(-1.0, min(1.0, value / 32767.0))
        if abs(frac) < STICK_DEADZONE:
            return 0.0
        # Expo curve softens the stick near center; cubic keeps sign and the
        # full-deflection endpoint (+/-1), so top turn rate is unchanged.
        return (1.0 - TURN_EXPO) * frac + TURN_EXPO * frac ** 3

    # One button owns speed now that D-pad up is the follow toggle: each press
    # steps UP through clear presets and wraps back to the slowest.
    SPEED_PRESETS = (0.35, 0.6, 1.0)

    def _on_dpad_y(self, value):
        # Up toggles follow-me. Down cycles the speed presets upward.
        # Act once per press.
        state = -1 if value < -16000 else (1 if value > 16000 else 0)
        if state and state != self._dpad_y:
            if state < 0:
                self._toggle_follow()
            else:
                higher = [s for s in self.SPEED_PRESETS if s > self._max_linear + 1e-9]
                self._max_linear = higher[0] if higher else self.SPEED_PRESETS[0]
                self.get_logger().info('Max linear speed: %.2f m/s' % self._max_linear)
        self._dpad_y = state

    def _on_dpad_x(self, value):
        # Right increases angular max, left decreases it. Act once per press.
        state = -1 if value < -16000 else (1 if value > 16000 else 0)
        if state and state != self._dpad_x:
            self._adjust_angular(ANGULAR_STEP if state > 0 else -ANGULAR_STEP)
        self._dpad_x = state

    def _adjust_linear(self, delta):
        self._max_linear = max(LINEAR_MIN, min(LINEAR_MAX, self._max_linear + delta))
        self.get_logger().info('Max linear speed: %.2f m/s' % self._max_linear)

    # --- follow-me toggle ------------------------------------------------------
    def _on_follow_status(self, msg: String):
        self._follow_active = msg.data.split('|')[0] in (
            'searching', 'locked', 'blocked')

    def _toggle_follow(self):
        client = self._follow_stop if self._follow_active else self._follow_start
        verb = 'stop' if self._follow_active else 'start'
        if not client.service_is_ready():
            self.get_logger().warn('follow_me/%s service not available' % verb)
            return
        client.call_async(Trigger.Request())
        # Optimistic flip so a double-press acts sanely before /follow_status
        # confirms; the subscription overwrites with the truth.
        self._follow_active = not self._follow_active
        self.get_logger().info('Follow-me %s requested (D-pad up)' % verb)

    def _adjust_angular(self, delta):
        self._max_angular = max(ANGULAR_MIN, min(ANGULAR_MAX, self._max_angular + delta))
        self.get_logger().info('Max angular speed: %.2f rad/s' % self._max_angular)

    def _zero_inputs(self):
        self._rt = self._lt = self._turn = 0.0
        self._dpad_x = self._dpad_y = 0

    # --- Publishing ----------------------------------------------------------
    def _publish(self):
        if self._mark_requested:
            self._mark_requested = False
            if self._patrol_mark.service_is_ready():
                fut = self._patrol_mark.call_async(Trigger.Request())
                fut.add_done_callback(lambda f: self.get_logger().info(
                    'Mark: %s' % f.result().message))
            else:
                self.get_logger().warn('patrol/mark not available')
        # Drive only while the controller is active; CmdVelSource owns the
        # release zero-burst and hands /cmd_vel back to other sources when idle.
        active = self._rt > 0.0 or self._lt > 0.0 or self._turn != 0.0
        if active:
            # RT forward, LT reverse; both can be read at once so they just sum.
            throttle = self._rt - self._lt
            # Left stick: push left -> turn left (CCW, +z per REP-103).
            turn = -self._turn * self._max_angular
            # In reverse, invert turning so it steers like a car backing up:
            # the same stick direction flips the wheel differential.
            if throttle < 0.0:
                turn = -turn
            # No pivot floor since the 2026-08-14 deflated-tire measurements:
            # all four wheels turn at any commanded rate (the old flat-tire
            # stall is gone). Slow pivots just walk more (~10 cm/rev at 1.5
            # rad/s vs ~2.5 cm at 2.5) — visible to the operator, their call.
            self._cmd.command(throttle * self._max_linear, turn)
        else:
            self._cmd.idle()

    def stop(self):
        self._stop = True
        self._cmd.stop_now()  # explicit stop on shutdown


def main(args=None):
    run_node(JoystickTeleopNode, on_shutdown=lambda node: node.stop(),
             args=args)


if __name__ == '__main__':
    main()
