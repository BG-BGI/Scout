"""Command-stream flow tracking + stop-reason classification (pure, no ROS).

The autonomy command chain is /cmd_vel_auto -> collision_monitor ->
/cmd_vel_safe -> final mux -> /cmd_vel_out (robot_profile.yaml's cmd_vel
contract). When the robot stops, WHERE the stream died says WHY: nav idle,
collision_monitor dead, a polygon stop, a bypass, zones out of sync, or the
final mux overriding autonomy (teleop/estop). Nothing in the stack answered
that in one place — incident forensics meant correlating four bag topics by
hand. health_monitor feeds one TopicFlow per hop and publishes
stop_reason()'s verdict as a `cmd_stream` diagnostic + a latched
/stop_reason breadcrumb (ADR-0036).

A plain Twist has no source timestamp, so a TopicFlow measures ARRIVAL times
only: callback age and inter-arrival gaps. That distinguishes "stream died"
from "stream flowing" per hop; it does not measure end-to-end latency (the
handoff's caveat — tracing would be needed for that). `now` is any monotonic
seconds source, injected like core.latch.

Zero convention: the stack stops with explicit zero Twists
(cmd_vel_source's zero burst, collision_monitor's stop output), so
`|vx| and |wz| < ZERO_EPS` is a deliberate stop command, not silence.
"""

ZERO_EPS = 1e-4

AUTO_IDLE = 'auto_idle'          # nav isn't asking to move — not a fault
CM_DEAD = 'cm_dead'              # auto flowing, /cmd_vel_safe silent: CM down
BYPASS = 'bypass'                # moving (or trying to) under an active bypass
CM_STOP = 'cm_stop'              # CM forwarding zeros against nonzero demand
ZONE_UNSYNCED = 'zone_unsynced'  # moving while the zone push is unacked
MUX_OVERRIDE = 'mux_override'    # final mux overrode autonomy (teleop/estop)
MOVING = 'moving'


def twist_is_zero(vx, wz, eps=ZERO_EPS):
    return abs(vx) < eps and abs(wz) < eps


class TopicFlow:
    """Arrival tracking for one hop: age of the last message, age of the
    last NONZERO command, and the worst inter-arrival gap over a rolling
    window (the number that catches intermittent starvation a point-in-time
    age misses)."""

    def __init__(self, window_s=10.0):
        self.window_s = float(window_s)
        self.last_t = None
        self.last_nonzero_t = None
        self._gaps = []  # (arrival_t, gap_s), pruned to window_s

    def on_msg(self, now, is_zero):
        if self.last_t is not None:
            self._gaps.append((now, now - self.last_t))
            cutoff = now - self.window_s
            while self._gaps and self._gaps[0][0] < cutoff:
                self._gaps.pop(0)
        self.last_t = now
        if not is_zero:
            self.last_nonzero_t = now

    def age(self, now):
        """Seconds since the last message; None = never seen."""
        return None if self.last_t is None else now - self.last_t

    def nonzero_age(self, now):
        """Seconds since the last nonzero command; None = never seen."""
        return None if self.last_nonzero_t is None else now - self.last_nonzero_t

    def max_gap(self, now):
        """Worst inter-arrival gap inside the window, including the
        currently-open gap; None until two messages have arrived."""
        cutoff = now - self.window_s
        gaps = [g for t, g in self._gaps if t >= cutoff]
        if self.last_t is not None and now - self.last_t > 0.0:
            gaps.append(now - self.last_t)
        return max(gaps) if gaps else None


def _fresh(age, fresh_s):
    return age is not None and age <= fresh_s


def stop_reason(now, auto, safe, out, bypassed, zone_mode, zone_synced,
                fresh_s=0.5):
    """One label for the state of the command chain, precedence-ordered so
    the FIRST sufficient explanation wins:

      auto_idle    — no fresh /cmd_vel_auto: autonomy isn't commanding.
                     Idle silence is normal (the handoff's caveat: never
                     read it as a fault).
      cm_dead      — fresh demand but /cmd_vel_safe is silent: the CM is
                     not forwarding AT ALL (crashed/starved). WARN-worthy.
      bypass       — the bounded bypass is active; whatever moves under it
                     is deliberately unguarded, so say so before blaming
                     polygons.
      cm_stop:<zone> — fresh NONZERO demand, CM forwarding zeros: a stop
                     polygon fired. The system working, not a fault.
      zone_unsynced — commands flowing while collision_monitor hasn't
                     acked the current zone state (push in doubt,
                     ADR-0036). WARN-worthy.
      mux_override — /cmd_vel_safe nonzero but /cmd_vel_out zero or
                     silent: teleop/estop/the final mux overrode autonomy.
      moving       — demand flowing all the way through.
    """
    auto_age = auto.age(now)
    if not _fresh(auto_age, fresh_s):
        return AUTO_IDLE
    if not _fresh(safe.age(now), fresh_s):
        return CM_DEAD
    if bypassed:
        return BYPASS
    demand = _fresh(auto.nonzero_age(now), fresh_s)
    forwarding = _fresh(safe.nonzero_age(now), fresh_s)
    if demand and not forwarding:
        return '%s:%s' % (CM_STOP, zone_mode)
    if not zone_synced:
        return ZONE_UNSYNCED
    if forwarding and not _fresh(out.nonzero_age(now), fresh_s):
        return MUX_OVERRIDE
    return MOVING
