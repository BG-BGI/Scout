#!/usr/bin/env python3
"""Play choreographed cmd_vel sequences ("tricks") on request.

Design mirrors led_node: the PlayTrick service only mutates target state and
returns promptly; a fixed-rate timer is the SOLE cmd_vel writer, which keeps
the 200 ms RoboClaw deadman fed (>= 20 Hz) from exactly one place. On finish
or /stop_trick the node bursts zero Twists for a short grace period and then
goes silent, handing /cmd_vel back to other sources — the shared CmdVelSource
contract (also used by joystick_teleop and follow_me).

Safety:
  * Segments are clamped to roboclaw.yaml's real caps (1.0 m/s, 3.0 rad/s).
  * Pure pivots must command >= min_pivot_rate (0.35 rad/s, the drivetrain's
    velocity-loop tracking floor). The old 2.5 flat-tire stall floor is gone
    (2026-08-14, tires deflated: all four wheels turn at any rate); fast
    pivots still walk less (~2.5 cm/rev at 2.5 rad/s vs ~10 at 1.5), which
    for tricks is cosmetic. Arcs (vx != 0) are exempt.
  * Tricks are refused below battery_floor_volts: they run at high duty and a
    sagging pack near the RoboClaw's 16.0 V Min Main trip would chatter.
"""

import math
import time

from rclpy.node import Node
from scout_interfaces.srv import PlayTrick
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from scout.cmd_vel_source import CmdVelSource
from scout.core.status import format_trick_status
from scout.core.tricks import TRICK_LED, TRICKS, validate_tricks
from scout.node_util import run_node
from scout.robot_profile import load as _load_profile

# Cross-surface caps live in robot_profile.yaml (SSOT). The stop-burst is now
# owned by CmdVelSource (also profile-driven).
_PROFILE = _load_profile()
MAX_LINEAR = _PROFILE['linear_cap']     # m/s  (= roboclaw.yaml max_linear_velocity)
MAX_ANGULAR = _PROFILE['angular_cap']   # rad/s (= roboclaw.yaml max_angular_velocity)

class TrickPlayer(Node):
    """Serves /play_trick and walks the segment list on a fixed timer."""

    def __init__(self):
        super().__init__('trick_player')

        # Parameter defaults come from robot_profile.yaml (SSOT) so every
        # surface agrees; a launch file can still override per scenario.
        self.declare_parameter('publish_hz', float(_PROFILE['publish_hz']))
        self.declare_parameter('min_pivot_rate', float(_PROFILE['angular_floor']))
        self.declare_parameter('battery_floor_volts',
                               float(_PROFILE['battery_activity_floor_v']))

        publish_hz = float(self.get_parameter('publish_hz').value)
        self._min_pivot_rate = float(self.get_parameter('min_pivot_rate').value)
        self._battery_floor = float(self.get_parameter('battery_floor_volts').value)

        validate_tricks(TRICKS, TRICK_LED, max_linear=MAX_LINEAR,
                        max_angular=MAX_ANGULAR, min_pivot_rate=self._min_pivot_rate)

        self._cmd = CmdVelSource(self, 'trick', hz=publish_hz)
        self._status_pub = self.create_publisher(String, 'trick_status', 10)

        self._voltage = math.nan
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)

        # Active-trick state, written by services, read by the timer.
        self._trick = None          # name, or None when idle
        self._segments = []
        self._started = 0.0         # monotonic start of the trick
        self._segment_color = None  # current segment's LED color override

        self.create_service(PlayTrick, 'play_trick', self._on_play)
        self.create_service(Trigger, 'stop_trick', self._on_stop)
        self.create_timer(1.0 / publish_hz, self._tick)
        # Status is a small string streamed at a fixed rate rather than latched:
        # rosbridge subscribers get it without any transient_local QoS matching.
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            'Trick player up: %s (publish %.0f Hz, battery floor %.1f V)'
            % (', '.join(sorted(TRICKS)), publish_hz, self._battery_floor))

    # --- inputs ---------------------------------------------------------------
    def _on_battery(self, msg: BatteryState):
        self._voltage = msg.voltage

    def _on_play(self, request, response):
        name = (request.name or '').strip().lower()
        if name not in TRICKS:
            response.success = False
            response.message = ("unknown trick '%s'; valid: %s"
                                % (request.name, ', '.join(sorted(TRICKS))))
            return response
        if self._trick is not None:
            response.success = False
            response.message = "busy: '%s' is running (call /stop_trick first)" % self._trick
            return response
        if not math.isnan(self._voltage) and self._voltage <= self._battery_floor:
            response.success = False
            response.message = ('battery %.1f V at or below the %.1f V trick floor'
                                % (self._voltage, self._battery_floor))
            return response

        self._trick = name
        self._segments = TRICKS[name]
        self._started = time.monotonic()
        self._segment_color = None
        self._publish_status()

        total = sum(seg[0] for seg in self._segments)
        response.success = True
        response.message = "playing '%s' (%.1f s, %d segments)" % (
            name, total, len(self._segments))
        self.get_logger().info(response.message)
        return response

    def _on_stop(self, request, response):
        if self._trick is None:
            response.success = True
            response.message = 'idle'
            return response
        stopped = self._trick
        self._finish()
        response.success = True
        response.message = "stopped '%s'" % stopped
        self.get_logger().info(response.message)
        return response

    # --- timer (drives the segment list; CmdVelSource owns publishing) ---------
    def _tick(self):
        if self._trick is None:
            return  # idle / zero-burst: CmdVelSource handles it
        elapsed = time.monotonic() - self._started
        for seg in self._segments:
            dur, vx, wz = seg[0], seg[1], seg[2]
            if elapsed < dur:
                # Segments may override the trick's LED color mid-run.
                color = seg[3] if len(seg) > 3 else None
                if color != self._segment_color:
                    self._segment_color = color
                    self._publish_status()
                # CmdVelSource re-clamps to the caps (== MAX_LINEAR/MAX_ANGULAR).
                self._cmd.command(vx, wz)
                return
            elapsed -= dur
        self._finish()  # walked past the last segment

    def _finish(self):
        """End the active trick and start the zero burst."""
        self._trick = None
        self._segments = []
        self._segment_color = None
        self._cmd.idle()
        self._publish_status()

    def _publish_status(self):
        """'idle', or 'name|#RRGGBB|mode' so led_status just renders it."""
        msg = String()
        if self._trick is None:
            msg.data = format_trick_status()
        else:
            color, mode = TRICK_LED[self._trick]
            msg.data = format_trick_status(
                self._trick, self._segment_color or color, mode)
        self._status_pub.publish(msg)

    def stop(self):
        self._cmd.stop_now()  # explicit stop on shutdown


def main(args=None):
    run_node(TrickPlayer, on_shutdown=lambda n: n.stop(), args=args)


if __name__ == '__main__':
    main()
