#!/usr/bin/env python3
"""Arbitrate the LED strip between status events, tricks, and user requests.

led_node stays a dumb renderer; this node is its only caller in normal
operation. Everything that wants the strip goes through a priority stack,
re-resolved on every input event and on a 1 Hz timer (which also expires
transient overlays):

  1. battery critical  -> red blink, persistent
  2. battery warning   -> orange breathe, persistent
  3. transient overlay -> timed flashes: ready at startup, client connect,
                          last client disconnect
  4. trick active      -> per-trick chase color (from /trick_status)
  5. user setting      -> whatever /set_user_led last asked for
  6. idle              -> off

Battery thresholds latch on at the battery_monitor values and clear only
`hysteresis_volts` above them — these are loaded readings that sag under
drive current and would otherwise flap on every acceleration.

/set_led_mode is called async only (never block the executor) and only when
the resolved pattern actually changes, so the service is not spammed at 1 Hz.

The rosbridge_msgs import is guarded: without rosbridge installed the
connect/disconnect events silently drop and everything else still works.
"""

import math
import time

from rclpy.node import Node
from scout_interfaces.srv import SetLedMode
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from scout.node_util import run_node
from scout.robot_profile import load as _load_profile

try:
    from rosbridge_msgs.msg import ConnectedClients
except ImportError:  # rosbridge not installed; run without client events
    ConnectedClients = None

_PROFILE = _load_profile()

# A pattern is (mode, color, brightness_pct, speed) — the SetLedMode fields.
PATTERN_CRITICAL = ('blink', '#FF0000', 60, 2.0)
PATTERN_WARN = ('breathe', '#FF8000', 50, 0.5)
PATTERN_READY = ('breathe', '#00FF00', 50, 0.7)
PATTERN_CONNECT = ('blink', '#00FF00', 50, 2.0)
PATTERN_DISCONNECT = ('blink', '#FF8000', 50, 1.0)
PATTERN_OFF = ('off', '', 0, 1.0)
# Solid red — outranks everything, and distinct from critical's red BLINK.
PATTERN_ESTOP = ('solid', '#FF0000', 80, 1.0)

VALID_MODES = tuple(_PROFILE['led_modes'])


class LedStatus(Node):
    """Watch battery/clients/tricks and drive led_node by priority."""

    def __init__(self):
        super().__init__('led_status')

        self.declare_parameter('warn_voltage', _PROFILE['battery_warn_v'])
        self.declare_parameter('critical_voltage', _PROFILE['battery_critical_v'])
        self.declare_parameter('hysteresis_volts', 0.4)
        self.declare_parameter('ready_seconds', 3.0)
        self.declare_parameter('connect_seconds', 2.0)
        self.declare_parameter('disconnect_seconds', 3.0)

        p = self.get_parameter
        self._warn_v = float(p('warn_voltage').value)
        self._critical_v = float(p('critical_voltage').value)
        self._hyst = float(p('hysteresis_volts').value)
        self._ready_s = float(p('ready_seconds').value)
        self._connect_s = float(p('connect_seconds').value)
        self._disconnect_s = float(p('disconnect_seconds').value)

        # Stack inputs.
        self._estop = False
        self._warn_active = False
        self._critical_active = False
        self._overlay = None            # (pattern, monotonic expiry)
        self._trick = 'idle'
        self._follow = 'idle'           # idle | searching | locked|dist|deg
        self._user_pattern = None       # set by /set_user_led
        self._seen_battery = False
        # NB: not `_clients` — that name is rclpy.Node's internal client list.
        self._client_count = 0

        # Output-side state.
        self._sent_pattern = None
        self._pending = None            # in-flight /set_led_mode future

        self._led_client = self.create_client(SetLedMode, 'set_led_mode')

        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.create_subscription(String, 'trick_status', self._on_trick, 10)
        self.create_subscription(String, 'follow_status', self._on_follow, 10)
        self.create_subscription(Bool, _PROFILE['topic_estop'], self._on_estop, 10)
        if ConnectedClients is not None:
            self.create_subscription(ConnectedClients, 'connected_clients',
                                     self._on_clients, 10)
        else:
            self.get_logger().warn(
                'rosbridge_msgs not available — connect/disconnect flashes disabled')

        self.create_service(SetLedMode, 'set_user_led', self._on_user_led)
        self.create_timer(1.0, self._resolve)

        self.get_logger().info(
            'LED status manager up: warn %.1f V / critical %.1f V (+%.1f V hysteresis)'
            % (self._warn_v, self._critical_v, self._hyst))

    # --- inputs ---------------------------------------------------------------
    def _on_battery(self, msg: BatteryState):
        if not self._seen_battery and msg.present:
            # First pack reading proves the driver link is up: ready flash.
            self._seen_battery = True
            self._set_overlay(PATTERN_READY, self._ready_s)

        v = msg.voltage
        if math.isnan(v) or not msg.present:
            return
        # Latch on at the threshold, clear only hysteresis above it.
        if v <= self._critical_v:
            self._critical_active = True
        elif v > self._critical_v + self._hyst:
            self._critical_active = False
        if v <= self._warn_v:
            self._warn_active = True
        elif v > self._warn_v + self._hyst:
            self._warn_active = False
        self._resolve()

    def _on_clients(self, msg):
        count = len(msg.clients)
        if self._client_count == 0 and count > 0:
            self._set_overlay(PATTERN_CONNECT, self._connect_s)
        elif self._client_count > 0 and count == 0:
            self._set_overlay(PATTERN_DISCONNECT, self._disconnect_s)
        self._client_count = count

    def _on_trick(self, msg: String):
        # trick_player sends 'idle' or 'name|#RRGGBB|mode' (color may change
        # per segment, e.g. countdown's red -> orange -> green).
        if msg.data != self._trick:
            self._trick = msg.data
            self._resolve()

    def _on_follow(self, msg: String):
        # follow_me sends 'idle', 'searching', or 'locked|dist|deg'.
        state = msg.data.split('|')[0]
        if state != self._follow:
            self._follow = state
            self._resolve()

    def _on_estop(self, msg: Bool):
        if msg.data != self._estop:
            self._estop = msg.data
            self._resolve()

    def _on_user_led(self, request, response):
        mode = (request.mode or '').strip().lower()
        if mode not in VALID_MODES:
            response.success = False
            response.message = ("unknown mode '%s'; valid: %s"
                                % (request.mode, ', '.join(VALID_MODES)))
            return response
        self._user_pattern = (mode, request.color, int(request.brightness),
                              float(request.speed))
        self._resolve()
        response.success = True
        response.message = ("user LED set to '%s' (applied when no alert/trick "
                            'outranks it; led_node may clamp brightness)') % mode
        return response

    def _set_overlay(self, pattern, seconds):
        self._overlay = (pattern, time.monotonic() + seconds)
        self._resolve()

    # --- priority resolution ----------------------------------------------------
    def _resolve(self):
        if self._overlay and self._overlay[1] <= time.monotonic():
            self._overlay = None

        if self._estop:
            pattern = PATTERN_ESTOP
        elif self._critical_active:
            pattern = PATTERN_CRITICAL
        elif self._warn_active:
            pattern = PATTERN_WARN
        elif self._overlay:
            pattern = self._overlay[0]
        elif self._trick != 'idle' and '|' in self._trick:
            _name, color, mode = (self._trick.split('|') + ['', 'chase'])[:3]
            pattern = (mode if mode in VALID_MODES else 'chase', color, 50, 2.0)
        elif self._follow == 'locked':
            pattern = ('chase', '#00FF40', 50, 2.0)
        elif self._follow == 'blocked':
            pattern = ('blink', '#FF8000', 50, 2.0)
        elif self._follow == 'searching':
            pattern = ('breathe', '#4060FF', 50, 1.0)
        elif self._user_pattern:
            pattern = self._user_pattern
        else:
            pattern = PATTERN_OFF

        self._apply(pattern)

    def _apply(self, pattern):
        if pattern == self._sent_pattern:
            return
        if self._pending is not None and not self._pending.done():
            return  # one call in flight; the 1 Hz timer retries
        if not self._led_client.service_is_ready():
            return  # led_node not up yet; the 1 Hz timer retries

        mode, color, brightness, speed = pattern
        req = SetLedMode.Request()
        req.mode = mode
        req.color = color
        req.brightness = max(0, min(100, int(brightness)))
        req.speed = float(speed)
        self._pending = self._led_client.call_async(req)
        self._sent_pattern = pattern
        self.get_logger().debug('LED -> %s %s' % (mode, color))


def main(args=None):
    run_node(LedStatus, args=args)


if __name__ == '__main__':
    main()
