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
`polygon` type that lets the operator drive out (nav2's `velocity_polygon`
type solves this properly; not adopted here — see ADR-0016 addendum).

This node is the operator's way out: /collision_monitor/bypass_engage PAUSES
collision_monitor via its OWN lifecycle manager's sanctioned
ManageLifecycleNodes service (not a raw lifecycle change_state call — the
manager owns the bond to collision_monitor, and PAUSE/RESUME is the control
surface designed for exactly this, so it tears down/recreates the bond
correctly instead of treating an externally-forced deactivate as a failure).

⚠ BOUNDED, NOT A GENERAL DISABLE. Bypass auto-releases after
`bypass_max_duration_s` (default 30 s — enough to back away a few feet, not a
substitute for the safety stage) and logs a WARN every few seconds while
active so it is loud, not stealth. /collision_monitor/bypassed (latched Bool)
is the single source of truth for "is the last-hop safety stage currently
off" — webui/skills should show it prominently.
"""

import time

from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from scout.node_util import run_node
from scout.qos import LATCHED_QOS

# nav2_msgs/srv/ManageLifecycleNodes command values (verified against the
# Humble .srv: STARTUP=0, PAUSE=1, RESUME=2, RESET=3, SHUTDOWN=4).
_PAUSE = 1
_RESUME = 2

MANAGER_SERVICE = '/lifecycle_manager_safety/manage_nodes'


class CollisionBypass(Node):
    """Bounded, logged bypass of collision_monitor via its lifecycle manager."""

    def __init__(self):
        super().__init__('collision_bypass')
        self._max_duration = float(
            self.declare_parameter('bypass_max_duration_s', 30.0).value)
        self._bypassed = False
        self._engaged_at = None

        self._client = self.create_client(
            ManageLifecycleNodes, MANAGER_SERVICE)
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

    def _call(self, command, on_done):
        # SC11: async + done-callback only — a sync call here (we're already
        # inside a service callback) deadlocks the single-threaded executor
        # with no warning.
        if not self._client.service_is_ready():
            on_done(False, 'lifecycle_manager_safety not reachable')
            return
        req = ManageLifecycleNodes.Request()
        req.command = command
        fut = self._client.call_async(req)
        fut.add_done_callback(
            lambda f: on_done(bool(f.result() and f.result().success), ''))

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
                    'COLLISION MONITOR BYPASSED — last-hop safety stage is '
                    'OFF, auto-release in %.0f s' % self._max_duration)
            else:
                self.get_logger().error(
                    'bypass_engage failed: %s' % (err or 'PAUSE rejected'))
        self._call(_PAUSE, done)
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
                    'collision monitor safety RESTORED (%s)' % why)
            else:
                self.get_logger().error(
                    'bypass_release failed: %s — safety stage may still be '
                    'OFF, retry' % (err or 'RESUME rejected'))
        self._call(_RESUME, done)

    def _tick(self):
        if not self._bypassed:
            return
        elapsed = time.monotonic() - self._engaged_at
        if elapsed > self._max_duration:
            self.get_logger().warn('bypass auto-release after %.0f s' % elapsed)
            self._release('auto-release timeout')
        elif int(elapsed) % 5 == 0:
            self.get_logger().warn(
                'COLLISION MONITOR BYPASSED — %.0f s remaining'
                % (self._max_duration - elapsed))

    def _publish_status(self):
        self._status_pub.publish(Bool(data=self._bypassed))


def main(args=None):
    run_node(CollisionBypass, args=args)


if __name__ == '__main__':
    main()
