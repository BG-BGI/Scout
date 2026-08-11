#!/usr/bin/env python3
"""Follow the nearest object in front of the robot at a fixed standoff.

Lidar-only: no ML, no camera. Each scan, beams inside the search sector and
range window are clustered (consecutive beams whose ranges differ < cluster_gap);
the tracked target is the cluster centroid nearest the previous target (gated),
or the nearest cluster overall while acquiring. A P-loop drives range error to
`standoff` and bearing to zero. Follows anything — a person's legs, a box on a
string — which is the point.

⚠ The scanner is mounted 180° backwards (see CLAUDE.md). TF corrects the URDF,
but raw /scan angles are lidar-frame, so robot-forward is scan angle pi:
`scan_yaw_offset` (default pi) maps scan angle -> base_link bearing. If the
robot backs away from you or orbits, verify the offset with a bench check
(stand in front, echo /follow_status, bearing should read ~0).

cmd_vel contract (same as joystick_teleop / trick_player): publish only while
locked, 0.3 s zero burst on stop/loss, then silence. Safety: hard zero inside
`stop_distance`, target loss drops to 'searching' and stops, /follow_me/stop
or the UI STOP ends it. Arcs dominate; pure pivots are capped at max_wz which
is deliberately below the 2.5 rad/s scrub floor — brief scrub wear while
turning in place is accepted here (command fidelity does not matter mid-chase).
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

STOP_GRACE = 0.3


class FollowMe(Node):
    """Nearest-cluster lidar follower with a P-loop to a fixed standoff."""

    def __init__(self):
        super().__init__('follow_me')

        p = self.declare_parameter
        self._standoff = float(p('standoff', 0.7).value)
        self._stop_dist = float(p('stop_distance', 0.45).value)
        self._min_range = float(p('min_range', 0.25).value)
        self._max_range = float(p('max_range', 3.0).value)
        self._sector = math.radians(float(p('sector_deg', 60.0).value))
        self._yaw_offset = float(p('scan_yaw_offset', math.pi).value)
        self._kp_lin = float(p('kp_lin', 0.8).value)
        self._kp_ang = float(p('kp_ang', 1.5).value)
        self._max_vx = float(p('max_vx', 0.6).value)
        self._max_wz = float(p('max_wz', 1.5).value)
        self._cluster_gap = float(p('cluster_gap', 0.15).value)
        self._min_beams = int(p('min_cluster_beams', 3).value)
        self._assoc_gate = float(p('association_gate', 0.5).value)
        self._lost_timeout = float(p('lost_timeout', 0.5).value)
        publish_hz = float(p('publish_hz', 20.0).value)

        self._active = False
        self._target = None          # (x, y) in base_link, robot-forward = +x
        self._target_seen = 0.0
        self._stop_until = 0.0
        self._status = 'idle'

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._status_pub = self.create_publisher(String, 'follow_status', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, 'follow_me/start', self._on_start)
        self.create_service(Trigger, 'follow_me/stop', self._on_stop)
        self.create_timer(1.0 / publish_hz, self._tick)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            'Follow-me up: standoff %.2f m, sector ±%.0f°, max %.2f m/s / %.2f rad/s'
            % (self._standoff, math.degrees(self._sector), self._max_vx, self._max_wz))

    # --- services ---------------------------------------------------------------
    def _on_start(self, request, response):
        self._active = True
        self._target = None
        self._set_status('searching')
        response.success = True
        response.message = 'following: searching for a target in the front sector'
        self.get_logger().info(response.message)
        return response

    def _on_stop(self, request, response):
        was = self._active
        self._halt()
        response.success = True
        response.message = 'stopped' if was else 'was idle'
        self.get_logger().info('Follow-me %s' % response.message)
        return response

    def _halt(self):
        self._active = False
        self._target = None
        self._stop_until = time.monotonic() + STOP_GRACE
        self._set_status('idle')

    # --- perception ---------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        if not self._active:
            return
        clusters = self._clusters(msg)
        if not clusters:
            return
        if self._target is None:
            # Acquire: nearest cluster wins.
            best = min(clusters, key=lambda c: math.hypot(c[0], c[1]))
        else:
            # Re-associate: nearest to the last position, gated.
            tx, ty = self._target
            best = min(clusters, key=lambda c: math.hypot(c[0] - tx, c[1] - ty))
            if math.hypot(best[0] - tx, best[1] - ty) > self._assoc_gate:
                return  # nothing near the old target this scan; keep waiting
        self._target = best
        self._target_seen = time.monotonic()
        if self._status != 'locked':
            self._set_status('locked')

    def _clusters(self, msg):
        """Cluster centroids (x fwd, y left in base_link) inside sector+range."""
        out = []
        beams = []   # (bearing, range) accumulator for the current cluster
        prev_r = None
        n = len(msg.ranges)
        for i in range(n):
            r = msg.ranges[i]
            ok = (math.isfinite(r) and self._min_range <= r <= self._max_range)
            if ok:
                bearing = self._wrap(msg.angle_min + i * msg.angle_increment
                                     + self._yaw_offset)
                ok = abs(bearing) <= self._sector
            if not ok or (prev_r is not None and abs(r - prev_r) > self._cluster_gap):
                self._flush(beams, out)
                beams = []
            if ok:
                beams.append((bearing, r))
                prev_r = r
            else:
                prev_r = None
        self._flush(beams, out)
        return out

    def _flush(self, beams, out):
        if len(beams) >= self._min_beams:
            xs = [r * math.cos(b) for b, r in beams]
            ys = [r * math.sin(b) for b, r in beams]
            out.append((sum(xs) / len(xs), sum(ys) / len(ys)))

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    # --- control (sole cmd_vel writer) ----------------------------------------------
    def _tick(self):
        now = time.monotonic()
        if self._active and self._target is not None:
            if now - self._target_seen > self._lost_timeout:
                self._target = None
                self._set_status('searching')
                self._stop_until = now + STOP_GRACE
                return
            dist = math.hypot(*self._target)
            bearing = math.atan2(self._target[1], self._target[0])
            twist = Twist()
            if dist > self._stop_dist:
                vx = self._kp_lin * (dist - self._standoff)
                # Follow only, never reverse toward the thing behind the error.
                twist.linear.x = max(0.0, min(self._max_vx, vx))
            twist.angular.z = max(-self._max_wz,
                                  min(self._max_wz, self._kp_ang * bearing))
            self._pub.publish(twist)
        elif self._stop_until > now:
            self._pub.publish(Twist())

    def _set_status(self, s):
        self._status = s
        self._publish_status()

    def _publish_status(self):
        msg = String()
        if self._status == 'locked' and self._target is not None:
            msg.data = 'locked|%.2f|%.0f' % (
                math.hypot(*self._target),
                math.degrees(math.atan2(self._target[1], self._target[0])))
        else:
            msg.data = self._status
        self._status_pub.publish(msg)

    def stop(self):
        self._pub.publish(Twist())


def main():
    rclpy.init()
    node = FollowMe()
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
