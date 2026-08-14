#!/usr/bin/env python3
"""Play choreographed cmd_vel sequences ("tricks") on request.

Design mirrors led_node: the PlayTrick service only mutates target state and
returns promptly; a fixed-rate timer is the SOLE cmd_vel writer, which keeps
the 200 ms RoboClaw deadman fed (>= 20 Hz) from exactly one place. On finish
or /stop_trick the node bursts zero Twists for STOP_GRACE seconds and then
goes silent, handing /cmd_vel back to other sources — the same convention
joystick_teleop uses, so there is no mux and no stomping.

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

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from scout_interfaces.srv import PlayTrick

MAX_LINEAR = 1.0     # m/s, roboclaw.yaml max_linear_velocity
MAX_ANGULAR = 3.0    # rad/s, roboclaw.yaml max_angular_velocity
STOP_GRACE = 0.3     # s of explicit zeros after a trick, then silence

# name: [(duration_s, vx m/s, wz rad/s[, '#RRGGBB' led override]), ...]
# A (dur, 0, 0) segment is a deliberate hard stop (zeros keep the deadman fed).
# Durations/magnitudes are first guesses pending on-robot tuning. The wheelie
# depends on the roboclaw.yaml `accel`: at 20000 counts/s^2 the -1 -> +1 m/s
# flip ramps over ~0.65 s (~3 m/s^2) and the nose stays down; 40000+ is what
# gives it a chance. Tune on a hard floor.
TRICKS = {
    'spin':      [(4.2, 0.0, 3.0)],
    'wiggle':    [(0.3, 0.0, 2.5), (0.3, 0.0, -2.5)] * 6,
    'figure8':   [(3.5, 0.4, 1.5), (3.5, 0.4, -1.5)],
    'wheelie':   [(0.4, -1.0, 0.0), (0.6, 1.0, 0.0)],
    # Lunge, hard stop, psych pause, lunge again, retreat.
    'fakeout':   [(0.3, 1.0, 0.0), (0.4, 0.0, 0.0), (0.5, 0.0, 0.0),
                  (0.3, 1.0, 0.0), (0.6, -0.6, 0.0)],
    # Wet-dog shake: fast small pivots at the scrub floor.
    'shiver':    [(0.15, 0.0, 2.5), (0.15, 0.0, -2.5)] * 8,
    # One full rev each way; the reversal snap is the show (2*pi/3 ~= 2.1 s).
    'whiplash':  [(2.1, 0.0, 3.0), (2.1, 0.0, -3.0)],
    # One clean circle around a point ~27 cm to the left (r = vx/wz).
    'orbit':     [(4.2, 0.4, 1.5)],
    # Snake down the hallway (~1.8 m of travel).
    'slalom':    [(0.5, 0.6, 1.2), (0.5, 0.6, -1.2)] * 3,
    # Slow reverse with lazy alternating arcs.
    'moonwalk':  [(0.7, -0.25, 0.8), (0.7, -0.25, -0.8)] * 4,
    # Butt-wiggle creep forward (arcs, so the pivot floor does not apply).
    'wag':       [(0.2, 0.15, 2.5), (0.2, 0.15, -2.5)] * 6,
    # LED steps red -> orange -> green while stationary, then a 1 s sprint.
    'countdown': [(1.0, 0.0, 0.0, '#FF0000'), (1.0, 0.0, 0.0, '#FF8000'),
                  (1.0, 0.0, 0.0, '#00FF00'), (1.0, 1.0, 0.0, '#00FF00'),
                  (0.4, 0.0, 0.0, '#00FF00')],
    # Stationary light show with pivot pulses on the beat.
    'disco':     [(0.25, 0.0, 2.5), (0.25, 0.0, -2.5)] * 8,
    # Full-accel launch; with accel >= 40000 the wheels chirp before hooking up.
    'burnout':   [(1.2, 1.0, 0.0), (0.5, 0.0, 0.0)],
}

# Default LED (color, led_node mode) per trick; 'rainbow' ignores color.
# A segment's 4th element overrides the color mid-trick (see countdown).
TRICK_LED = {
    'spin':      ('#0080FF', 'chase'),
    'wiggle':    ('#B040FF', 'chase'),
    'figure8':   ('#00FFFF', 'chase'),
    'wheelie':   ('#FF2000', 'chase'),
    'fakeout':   ('#FF2000', 'chase'),
    'shiver':    ('#4060FF', 'breathe'),
    'whiplash':  ('#FFFFFF', 'blink'),
    'orbit':     ('#00FFFF', 'chase'),
    'slalom':    ('#00FF40', 'chase'),
    'moonwalk':  ('', 'rainbow'),
    'wag':       ('#B040FF', 'chase'),
    'countdown': ('#FF0000', 'solid'),
    'disco':     ('', 'rainbow'),
    'burnout':   ('#FF6000', 'chase'),
}


def _validate_tricks(min_pivot_rate):
    """Raise if any trick segment violates the caps or the pivot floor."""
    for name, segments in TRICKS.items():
        for i, seg in enumerate(segments):
            dur, vx, wz = seg[0], seg[1], seg[2]
            if dur <= 0.0:
                raise ValueError('%s[%d]: duration %.2f <= 0' % (name, i, dur))
            if abs(vx) > MAX_LINEAR:
                raise ValueError('%s[%d]: |vx| %.2f > %.2f' % (name, i, vx, MAX_LINEAR))
            if abs(wz) > MAX_ANGULAR:
                raise ValueError('%s[%d]: |wz| %.2f > %.2f' % (name, i, wz, MAX_ANGULAR))
            if vx == 0.0 and wz != 0.0 and abs(wz) < min_pivot_rate:
                raise ValueError(
                    '%s[%d]: pivot at %.2f rad/s is under the %.2f tracking floor'
                    % (name, i, wz, min_pivot_rate))
        if name not in TRICK_LED:
            raise ValueError('%s: missing TRICK_LED entry' % name)


class TrickPlayer(Node):
    """Serves /play_trick and walks the segment list on a fixed timer."""

    def __init__(self):
        super().__init__('trick_player')

        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('min_pivot_rate', 0.35)
        self.declare_parameter('battery_floor_volts', 17.0)

        publish_hz = float(self.get_parameter('publish_hz').value)
        self._min_pivot_rate = float(self.get_parameter('min_pivot_rate').value)
        self._battery_floor = float(self.get_parameter('battery_floor_volts').value)

        _validate_tricks(self._min_pivot_rate)

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._status_pub = self.create_publisher(String, 'trick_status', 10)

        self._voltage = math.nan
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)

        # Active-trick state, written by services, read by the timer.
        self._trick = None          # name, or None when idle
        self._segments = []
        self._started = 0.0         # monotonic start of the trick
        self._stop_until = 0.0      # monotonic end of the zero burst
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
        self._stop_until = 0.0
        self._segment_color = None
        self._publish_status()

        total = sum(seg[0] for seg in self._segments)
        response.success = True
        response.message = "playing '%s' (%.1f s, %d segments)" % (
            name, total, len(self._segments))
        self.get_logger().info(response.message)
        return response

    def _on_stop(self, request, response):
        if self._trick is None and self._stop_until <= time.monotonic():
            response.success = True
            response.message = 'idle'
            return response
        stopped = self._trick or 'zero burst'
        self._finish()
        response.success = True
        response.message = "stopped '%s'" % stopped
        self.get_logger().info(response.message)
        return response

    # --- timer (sole cmd_vel writer) -------------------------------------------
    def _tick(self):
        now = time.monotonic()

        if self._trick is not None:
            elapsed = now - self._started
            for seg in self._segments:
                dur, vx, wz = seg[0], seg[1], seg[2]
                if elapsed < dur:
                    # Segments may override the trick's LED color mid-run.
                    color = seg[3] if len(seg) > 3 else None
                    if color != self._segment_color:
                        self._segment_color = color
                        self._publish_status()
                    twist = Twist()
                    # Publish-time clamp backs up the import-time validation.
                    twist.linear.x = max(-MAX_LINEAR, min(MAX_LINEAR, vx))
                    twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, wz))
                    self._pub.publish(twist)
                    return
                elapsed -= dur
            self._finish()  # walked past the last segment

        if self._stop_until > now:
            self._pub.publish(Twist())

    def _finish(self):
        """End the active trick and start the zero burst."""
        self._trick = None
        self._segments = []
        self._segment_color = None
        self._stop_until = time.monotonic() + STOP_GRACE
        self._publish_status()

    def _publish_status(self):
        """'idle', or 'name|#RRGGBB|mode' so led_status just renders it."""
        msg = String()
        if self._trick is None:
            msg.data = 'idle'
        else:
            color, mode = TRICK_LED[self._trick]
            msg.data = '%s|%s|%s' % (
                self._trick, self._segment_color or color, mode)
        self._status_pub.publish(msg)

    def stop(self):
        self._pub.publish(Twist())  # explicit stop on shutdown


def main():
    rclpy.init()
    node = TrickPlayer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
