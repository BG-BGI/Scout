#!/usr/bin/env python3
"""Direction-aware collision-monitor stop zone + a bounded safety bypass
(ADR-0016 addendum).

Two related findings from the first on-hardware N1 verification, both fixed
here because they mutate the SAME two enabled flags and would race as
separate nodes:

1. **Direction-blind stop zone.** A plain nav2_collision_monitor `polygon`
   STOP shape checks scan points geometrically against a STATIC box —
   verified against upstream source (polygon.cpp: getPointsInside /
   isTriggeredInternal take only a point count, never the commanded Twist).
   A single symmetric box therefore reads an obstacle to the SIDE as a stop
   condition even while driving straight PAST it — found passing between two
   obstacles narrower than the default ±0.42 m gap though the 0.334 m
   chassis physically fit. nav2's native fix (`velocity_polygon`) merged
   Iron+ and is absent from this Humble apt build (confirmed empirically: no
   VelocityPolygon symbol in libnav2_collision_monitor_core.so) — building it
   from source would need a newer nav2_util/nav2_costmap_2d than Humble
   ships, or the whole distro upgraded. Instead: TWO pre-defined,
   mutually-exclusive stop polygons in collision_monitor.yaml
   (PolygonStopStraight, narrow sides; PolygonStopTurn, the original wide
   sides), and this node watches `/cmd_vel_out` (the SAME commanded Twist
   collision_monitor reads — reacting to what's about to be commanded, not
   lagging behind measured odometry) and live-toggles which one is
   `enabled` via collision_monitor's own `set_parameters` service (the
   dynamic-parameter path verified in polygon.cpp: `enabled_` applies
   immediately, no lifecycle transition needed).

   ⚠ One-tick latency is inherent: our node and collision_monitor's own
   subscription both receive the same `/cmd_vel_out` message via DDS fan-out
   with no ordering guarantee, so a transition only reliably applies from
   the NEXT commanded message onward, not the one that triggered it. At
   20–50 Hz that is ≤50 ms — folded into the existing stop-box margin, not a
   new gap.

   Hysteresis: enter "turning" above `turn_enter_rad_s`, only return to
   "straight" after `turn_exit_rad_s` AND `turn_exit_dwell_s` of staying
   below it — avoids flapping (and spamming set_parameters) near the
   threshold. Only |angular.z| matters: this is a skid-steer, never holonomic
   (linear.y is always 0).

2. **Bounded bypass, kept from the first attempt (with a mechanism fix).**
   PAUSing collision_monitor's lifecycle (the first attempt) does NOT pass
   cmd_vel through — verified against collision_monitor_node.cpp:
   `on_deactivate()` deactivates the OUTPUT publisher while the input
   subscription stays live, so a paused node silently swallows commands
   instead of forwarding them. The fix uses the SAME live-parameter
   mechanism as the direction-aware split: bypass disables BOTH stop
   polygons via `set_parameters` while the node stays ACTIVE and keeps
   forwarding. `PolygonSlow` is untouched throughout (harmless — it only
   caps speed). Auto-releases after `bypass_max_duration_s` (default 30 s)
   and restores whatever the direction logic currently says, not
   necessarily what was active before the bypass.

`/collision_monitor/bypassed` (latched Bool) and `/collision_monitor/zone_mode`
(latched String: "straight"|"turn") are the status surfaces — webui/skills
should show both prominently, not bury them.
"""

import time

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from scout.node_util import run_node
from scout.qos import LATCHED_QOS

SET_PARAMS_SERVICE = '/collision_monitor/set_parameters'
STRAIGHT_PARAM = 'PolygonStopStraight.enabled'
TURN_PARAM = 'PolygonStopTurn.enabled'


def _bool_param(name, value):
    p = Parameter()
    p.name = name
    p.value = ParameterValue(
        type=ParameterType.PARAMETER_BOOL, bool_value=value)
    return p


class CollisionPolygonManager(Node):
    """Owns collision_monitor's two stop-polygon enabled flags: the
    direction-aware straight/turn split, and the bounded bypass — one owner
    so the two features never race on the same parameters."""

    def __init__(self):
        super().__init__('collision_polygon_manager')
        p = self.declare_parameter
        self._max_duration = float(
            p('bypass_max_duration_s', 30.0).value)
        self._turn_enter = float(p('turn_enter_rad_s', 0.15).value)
        self._turn_exit = float(p('turn_exit_rad_s', 0.05).value)
        self._turn_exit_dwell = float(p('turn_exit_dwell_s', 0.3).value)

        self._bypassed = False
        self._engaged_at = None
        self._turning = False          # current direction-derived decision
        self._below_exit_since = None  # monotonic time, for exit hysteresis
        self._pushed = None            # last (straight_enabled, turn_enabled) sent

        self._client = self.create_client(SetParameters, SET_PARAMS_SERVICE)
        self._bypassed_pub = self.create_publisher(
            Bool, '/collision_monitor/bypassed', LATCHED_QOS)
        self._mode_pub = self.create_publisher(
            String, '/collision_monitor/zone_mode', LATCHED_QOS)
        self.create_subscription(Twist, '/cmd_vel_out', self._on_cmd_vel, 10)
        self.create_service(
            Trigger, '/collision_monitor/bypass_engage', self._on_engage)
        self.create_service(
            Trigger, '/collision_monitor/bypass_release', self._on_release)
        self.create_timer(1.0, self._tick)
        self._publish_status()
        self.get_logger().info(
            'collision_polygon_manager up: turn >%.2f/<%.2f rad/s '
            '(%.1fs dwell), bypass auto-release %.0f s'
            % (self._turn_enter, self._turn_exit, self._turn_exit_dwell,
               self._max_duration))

    # --- direction-aware zone selection --------------------------------------

    def _on_cmd_vel(self, msg: Twist):
        w = abs(msg.angular.z)
        now = time.monotonic()
        if not self._turning:
            if w > self._turn_enter:
                self._turning = True
                self._below_exit_since = None
        else:
            if w > self._turn_exit:
                self._below_exit_since = None
            else:
                if self._below_exit_since is None:
                    self._below_exit_since = now
                elif now - self._below_exit_since >= self._turn_exit_dwell:
                    self._turning = False
        self._push_zone_state()

    def _desired_state(self):
        """(straight_enabled, turn_enabled) for the current bypass/turning
        state — the single place that decides, so engage/release/cmd_vel
        transitions all funnel through it."""
        if self._bypassed:
            return (False, False)
        return (not self._turning, self._turning)

    def _push_zone_state(self):
        state = self._desired_state()
        if state == self._pushed:
            return
        if not self._client.service_is_ready():
            return  # collision_monitor not up yet; next change retries
        req = SetParameters.Request()
        req.parameters = [
            _bool_param(STRAIGHT_PARAM, state[0]),
            _bool_param(TURN_PARAM, state[1]),
        ]

        def done(fut):
            res = fut.result()
            ok = bool(res and all(r.successful for r in res.results))
            if ok:
                self._pushed = state
                self._mode_pub.publish(String(
                    data='turn' if state[1] else 'straight'))
            else:
                self.get_logger().error(
                    'zone-state push failed (straight=%s turn=%s)' % state)
        self._client.call_async(req).add_done_callback(done)

    # --- bounded bypass --------------------------------------------------------

    def _on_engage(self, req, resp):
        if self._bypassed:
            resp.success = False
            resp.message = 'already bypassed'
            return resp
        self._bypassed = True
        self._engaged_at = time.monotonic()
        self._publish_status()
        self._push_zone_state()
        self.get_logger().warn(
            'COLLISION MONITOR STOP ZONES BYPASSED — auto-release in %.0f s'
            % self._max_duration)
        resp.success = True
        resp.message = 'bypass engaged'
        return resp

    def _on_release(self, req, resp):
        if not self._bypassed:
            resp.success = False
            resp.message = 'not bypassed'
            return resp
        self._release('operator released')
        resp.success = True
        resp.message = 'bypass released'
        return resp

    def _release(self, why):
        self._bypassed = False
        self._engaged_at = None
        self._publish_status()
        self._push_zone_state()
        self.get_logger().info('collision monitor stop zones RESTORED (%s)' % why)

    def _tick(self):
        if not self._bypassed:
            return
        elapsed = time.monotonic() - self._engaged_at
        if elapsed > self._max_duration:
            self.get_logger().warn('bypass auto-release after %.0f s' % elapsed)
            self._release('auto-release timeout')
        elif int(elapsed) % 5 == 0:
            self.get_logger().warn(
                'COLLISION MONITOR STOP ZONES BYPASSED — %.0f s remaining'
                % (self._max_duration - elapsed))

    def _publish_status(self):
        self._bypassed_pub.publish(Bool(data=self._bypassed))


def main(args=None):
    run_node(CollisionPolygonManager, args=args)


if __name__ == '__main__':
    main()
