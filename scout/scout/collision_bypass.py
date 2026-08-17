#!/usr/bin/env python3
"""Collision-monitor safety bypass — a bounded escape hatch for the
direction-blind PolygonStop lockout (ADR-0016 addendum).

nav2_collision_monitor's plain `polygon` shape with `action_type: stop`
checks scan points geometrically against a STATIC box — it has no idea what
direction the commanded Twist is (verified against upstream source,
nav2_collision_monitor/src/polygon.cpp: getPointsInside/isTriggeredInternal
take only a point count, never the commanded velocity). So once the stop
zone trips on a static obstacle, EVERY cmd_vel is zeroed — including a
reverse command meant to back away — and the robot is stuck until the
obstacle physically leaves the box. There is no code path in a plain
`polygon` type that lets the operator drive out.

⚠ FIRST ATTEMPT WAS WRONG — kept as a warning. PAUSing collision_monitor's
own lifecycle (via lifecycle_manager_safety's ManageLifecycleNodes) does NOT
turn it into a pass-through: verified against
nav2_collision_monitor/src/collision_monitor_node.cpp — `on_deactivate()`
deactivates the OUTPUT publisher while leaving the input subscription live,
so cmd_vel is silently swallowed rather than forwarded. That "bypass"
produced the identical stuck symptom (silent /cmd_vel_safe -> driver deadman
coasts), just for a different reason, and only surfaced on real hardware.

The actual fix: nav2_collision_monitor exposes each polygon's `enabled` flag
as a plain ROS parameter (`<PolygonName>.enabled`) with a live
dynamic-parameter callback (verified in polygon.cpp:
`dynamicParametersCallback` flips `enabled_` immediately, no lifecycle
transition) — so `PolygonStop.enabled=false` disables ONLY that zone's check
while the node stays ACTIVE and keeps forwarding cmd_vel normally.
PolygonSlow stays enabled throughout (harmless — it only caps speed, and
extra caution while backing out of a lockout is a feature, not a bug).

⚠ BOUNDED, NOT A GENERAL DISABLE. Bypass auto-releases after
`bypass_max_duration_s` (default 30 s — enough to back away a few feet, not a
substitute for the safety stage) and logs a WARN every few seconds while
active so it is loud, not stealth. /collision_monitor/bypassed (latched Bool)
is the single source of truth for "is the stop zone currently disabled" —
webui/skills should show it prominently.
"""

import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from scout.node_util import run_node
from scout.qos import LATCHED_QOS

SET_PARAMS_SERVICE = '/collision_monitor/set_parameters'
STOP_POLYGON_PARAM = 'PolygonStop.enabled'


def _bool_param(name, value):
    p = Parameter()
    p.name = name
    p.value = ParameterValue(
        type=ParameterType.PARAMETER_BOOL, bool_value=value)
    return p


class CollisionBypass(Node):
    """Bounded, logged bypass of the PolygonStop zone via a live parameter."""

    def __init__(self):
        super().__init__('collision_bypass')
        self._max_duration = float(
            self.declare_parameter('bypass_max_duration_s', 30.0).value)
        self._bypassed = False
        self._engaged_at = None

        self._client = self.create_client(
            SetParameters, SET_PARAMS_SERVICE)
        self._status_pub = self.create_publisher(
            Bool, '/collision_monitor/bypassed', LATCHED_QOS)
        self.create_service(
            Trigger, '/collision_monitor/bypass_engage', self._on_engage)
        self.create_service(
            Trigger, '/collision_monitor/bypass_release', self._on_release)
        self.create_timer(1.0, self._tick)
        self._publish_status()
        self.get_logger().info(
            'collision_bypass up: bypass_engage/release, auto-release %.0f s'
            % self._max_duration)

    def _set_stop_enabled(self, enabled, on_done):
        # SC11: async + done-callback only — a sync call here (we're already
        # inside a service callback) deadlocks the single-threaded executor.
        if not self._client.service_is_ready():
            on_done(False, 'collision_monitor set_parameters not reachable')
            return
        req = SetParameters.Request()
        req.parameters = [_bool_param(STOP_POLYGON_PARAM, enabled)]

        def done(fut):
            res = fut.result()
            ok = bool(res and res.results and res.results[0].successful)
            reason = (res.results[0].reason
                      if res and res.results and not ok else '')
            on_done(ok, reason)
        self._client.call_async(req).add_done_callback(done)

    def _on_engage(self, req, resp):
        if self._bypassed:
            resp.success = False
            resp.message = 'already bypassed'
            return resp

        def done(ok, err):
            if ok:
                self._bypassed = True
                self._engaged_at = time.monotonic()
                self._publish_status()
                self.get_logger().warn(
                    'COLLISION MONITOR STOP ZONE BYPASSED — auto-release '
                    'in %.0f s' % self._max_duration)
            else:
                self.get_logger().error(
                    'bypass_engage failed: %s' % (err or 'rejected'))
        self._set_stop_enabled(False, done)
        resp.success = True
        resp.message = 'bypass requested — watch /collision_monitor/bypassed'
        return resp

    def _on_release(self, req, resp):
        if not self._bypassed:
            resp.success = False
            resp.message = 'not bypassed'
            return resp
        self._release('operator released')
        resp.success = True
        resp.message = 'release requested — watch /collision_monitor/bypassed'
        return resp

    def _release(self, why):
        def done(ok, err):
            if ok:
                self._bypassed = False
                self._engaged_at = None
                self._publish_status()
                self.get_logger().info(
                    'collision monitor stop zone RESTORED (%s)' % why)
            else:
                self.get_logger().error(
                    'bypass_release failed: %s — stop zone may still be '
                    'disabled, retry' % (err or 'rejected'))
        self._set_stop_enabled(True, done)

    def _tick(self):
        if not self._bypassed:
            return
        elapsed = time.monotonic() - self._engaged_at
        if elapsed > self._max_duration:
            self.get_logger().warn('bypass auto-release after %.0f s' % elapsed)
            self._release('auto-release timeout')
        elif int(elapsed) % 5 == 0:
            self.get_logger().warn(
                'COLLISION MONITOR STOP ZONE BYPASSED — %.0f s remaining'
                % (self._max_duration - elapsed))

    def _publish_status(self):
        self._status_pub.publish(Bool(data=self._bypassed))


def main(args=None):
    run_node(CollisionBypass, args=args)


if __name__ == '__main__':
    main()
