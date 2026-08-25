"""Collision-monitor zone selection truth table (pure, no ROS) — which of the
three mutually-exclusive stop polygons is enabled for the current commanded
direction and bypass state (ADR-0016 addendum; collision_polygon_manager).

The turn/reverse *hysteresis* lives in scout.core.latch — this owns only the
decision the latches feed: mode precedence (turn > reverse > forward, because
a pivot sweeps every corner regardless of vx) and the (front, rear, turn)
enabled triple pushed into collision_monitor's parameters. Keeping the truth
table pure is what makes "bypass disables BOTH stop polygons" a test instead
of a hope.
"""

TURN = 'turn'
REVERSE = 'reverse'
FORWARD = 'forward'


def zone_mode(turning, reversing):
    """'turn' | 'reverse' | 'forward' — turn wins."""
    if turning:
        return TURN
    if reversing:
        return REVERSE
    return FORWARD


def desired_zone_state(bypassed, turning, reversing):
    """(front_enabled, rear_enabled, turn_enabled) — the single place that
    decides, so engage/release/cmd_vel transitions all funnel through it.
    Bypass disables all three (PolygonSlow is untouched — it only caps
    speed)."""
    if bypassed:
        return (False, False, False)
    mode = zone_mode(turning, reversing)
    if mode == TURN:
        return (False, False, True)
    if mode == REVERSE:
        return (False, True, False)
    return (True, False, False)
