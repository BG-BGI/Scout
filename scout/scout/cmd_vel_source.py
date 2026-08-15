"""Shared cmd_vel output contract for the robot-side motion producers.

joystick_teleop, trick_player and follow_me all obeyed the same unwritten
contract by hand: publish a Twist only while actively driving, burst a few
hundred ms of zeros on release so the RoboClaw stops promptly, then go silent
and hand /cmd_vel back to whoever else wants it. Three copies of the same
`_stop_until` state machine drifted apart. This is the one implementation.

A node computes its command however it likes (read a stick, walk a trick, run
a follow loop) and calls:

    src.command(vx, wz)   # while it wants to drive (vx/wz may be 0 to HOLD)
    src.idle()            # when it stops wanting to drive -> zero burst, silence
    src.stop_now()        # shutdown: one explicit zero immediately

The source owns:
  * the publisher, on the per-source topic from robot_profile.yaml (so the
    twist_mux fan-in is configured in one place — see M3/D1),
  * a fixed-rate timer that RE-publishes the last command every tick, which is
    what keeps the 200 ms deadman fed between a slow caller's updates,
  * the STOP_GRACE zero burst on idle(),
  * cap clamping to the profile's linear/angular caps (a safety net; callers
    with tighter limits still clamp first — NO floors here, floors are advisory),
  * a staleness guard: if command() has not been called within `stale_timeout`,
    the source auto-idles, so a caller loop that dies mid-command cannot latch a
    live velocity (a new safety property none of the three had).
"""

import time

from geometry_msgs.msg import Twist

from scout.robot_profile import load as _load_profile

_TOPIC_KEY = {
    'joy': 'topic_cmd_vel_joy',
    'web': 'topic_cmd_vel_web',
    'trick': 'topic_cmd_vel_trick',
    'follow': 'topic_cmd_vel_follow',
    'skills': 'topic_cmd_vel_skills',
}


class CmdVelSource:
    """One motion producer's cmd_vel output (see module docstring)."""

    def __init__(self, node, source, hz=None, stale_timeout=0.5):
        prof = _load_profile()
        if source not in _TOPIC_KEY:
            raise ValueError('unknown cmd_vel source %r' % source)
        self._topic = prof[_TOPIC_KEY[source]]
        self._hz = float(hz) if hz else float(prof['publish_hz'])
        self._grace = float(prof['stop_grace_s'])
        self._lin_cap = float(prof['linear_cap'])
        self._ang_cap = float(prof['angular_cap'])
        self._stale = float(stale_timeout)

        self._pub = node.create_publisher(Twist, self._topic, 10)
        self._last_cmd = None        # (vx, wz) while driving, None when idle
        self._last_command_t = 0.0   # monotonic of the last command()
        self._stop_until = 0.0       # monotonic end of the zero burst
        self._timer = node.create_timer(1.0 / self._hz, self._tick)

    def command(self, vx, wz):
        """Drive at (vx, wz) until the next command()/idle(). vx/wz may be 0 to
        actively hold position (still fed to the deadman). Clamped to caps."""
        self._last_cmd = (
            max(-self._lin_cap, min(self._lin_cap, float(vx))),
            max(-self._ang_cap, min(self._ang_cap, float(wz))),
        )
        self._last_command_t = time.monotonic()

    def idle(self):
        """Stop driving: begin the STOP_GRACE zero burst, then go silent."""
        if self._last_cmd is not None:
            self._stop_until = time.monotonic() + self._grace
        self._last_cmd = None

    def stop_now(self):
        """Publish one explicit zero immediately (shutdown path)."""
        self._last_cmd = None
        self._stop_until = 0.0
        self._pub.publish(Twist())

    def _tick(self):
        now = time.monotonic()
        if self._last_cmd is not None and now - self._last_command_t > self._stale:
            # Caller loop stalled mid-command — fail to idle, don't latch it.
            self.idle()
        if self._last_cmd is not None:
            vx, wz = self._last_cmd
            twist = Twist()
            twist.linear.x = vx
            twist.angular.z = wz
            self._pub.publish(twist)
        elif now < self._stop_until:
            self._pub.publish(Twist())
