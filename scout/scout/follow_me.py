#!/usr/bin/env python3
"""Follow the nearest object in front of the robot at a fixed standoff.

Lidar tracks the target: each scan, beams inside the search sector and range
window are clustered (consecutive beams whose ranges differ < cluster_gap);
the tracked target is the cluster centroid nearest the previous target (gated),
or the nearest cluster overall while acquiring. A P-loop drives range error to
`standoff` and bearing to zero. Follows anything — a person's legs, a box on a
string — which is the point. No ML.

The D455 depth cloud guards the corridor: the lidar plane sits 24 cm up, so
shoes, toys and thresholds are invisible to it. Depth points in the
0.05–0.25 m base_link height band that fall inside the forward corridor merge
into the same clearance gate (camera FOV is forward-only, so flank repulsion
stays lidar's job). The cloud is already ×4-decimated for the nav2 costmap.

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

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
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
        # Obstacle avoidance: forward corridor gating + repulsive steering.
        # Half-width 0.30 covers the 0.238 m circumscribed radius while the
        # robot yaws to track the target, not just the 0.167 m box half-width —
        # telemetry showed obstacles sliding out of a 0.25 m strip mid-rotation.
        self._avoid_lookahead = float(p('avoid_lookahead', 0.9).value)
        self._avoid_hard_stop = float(p('avoid_hard_stop', 0.35).value)
        self._corridor_half_width = float(p('corridor_half_width', 0.30).value)
        # A clearance reading this old stops gating (min-hold window) — bridges
        # scans where a seen obstacle briefly reads inf, which telemetry showed
        # causing full-speed bursts right next to furniture.
        self._clearance_hold = float(p('clearance_hold', 1.0).value)
        self._max_accel = float(p('max_accel', 0.6).value)  # m/s^2 vx slew
        # Odom-anchored memory of low depth obstacles. The D455 cannot see
        # closer than ~0.4 m and low objects fall out of its vertical FOV on
        # approach (office-chair bases), so points seen earlier are remembered
        # in the odom frame and re-projected into base_link every tick.
        self._mem_ttl = float(p('memory_ttl', 8.0).value)
        self._mem_range = float(p('memory_range', 1.5).value)
        self._mem_half_width = float(p('memory_half_width', 0.6).value)
        self._mem_voxel = float(p('memory_voxel', 0.05).value)
        self._avoid_radius = float(p('avoid_radius', 1.0).value)
        self._k_avoid = float(p('k_avoid', 0.35).value)
        self._target_exclusion = float(p('target_exclusion', 0.35).value)
        # Depth-cloud corridor guard (under-lidar obstacles).
        self._depth_band_lo = float(p('depth_band_low', 0.05).value)
        self._depth_band_hi = float(p('depth_band_high', 0.25).value)
        self._depth_period = float(p('depth_period', 0.2).value)
        self._depth_stale = float(p('depth_stale', 1.0).value)

        self._active = False
        self._target = None          # (x, y) in base_link, robot-forward = +x
        self._target_seen = 0.0
        self._stop_until = 0.0
        self._status = 'idle'
        self._corridor_min = math.inf  # nearest non-target return in the corridor
        self._wz_avoid = 0.0           # repulsive steering from side obstacles
        self._clearance_log = []       # (stamp, clearance) ring for the min-hold
        self._vx_out = 0.0             # slew-limited forward command
        self._last_tick = time.monotonic()
        self._obstacle_mem = {}        # {(vx_odom, vy_odom) voxel: last-seen stamp}
        self._depth_corridor_min = math.inf
        self._depth_stamp = 0.0
        self._last_depth_proc = 0.0
        self._cam_rot = None           # camera optical -> base_link, cached
        self._cam_trans = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._status_pub = self.create_publisher(String, 'follow_status', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'camera/camera/depth/color/points',
                                 self._on_depth, qos_profile_sensor_data)
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
        self._clearance_log = []
        self._obstacle_mem = {}
        self._vx_out = 0.0
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
        self._clearance_log = []
        self._vx_out = 0.0
        self._set_status('idle')

    # --- perception ---------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        if not self._active:
            return
        clusters = self._clusters(msg)
        self._update_obstacles(msg)
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
        if self._status == 'searching':
            self._set_status('locked')  # blocked/locked handled by the tick

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

    def _update_obstacles(self, msg):
        """Corridor clearance + repulsive steering from everything non-target.

        Beams within `target_exclusion` of the tracked target are the thing we
        are following, not an obstacle — without the carve-out the standoff
        gate and the corridor gate fight each other and the robot never moves.
        """
        corridor_min = math.inf
        left_prox = 0.0    # y > 0
        right_prox = 0.0   # y < 0
        inv_r = 1.0 / self._avoid_radius
        tx, ty = self._target if self._target else (None, None)
        n = len(msg.ranges)
        for i in range(n):
            r = msg.ranges[i]
            if not (math.isfinite(r) and self._min_range <= r <= self._max_range):
                continue
            bearing = self._wrap(msg.angle_min + i * msg.angle_increment
                                 + self._yaw_offset)
            if abs(bearing) > math.pi / 2:
                continue  # behind the beam of travel; irrelevant while following
            x = r * math.cos(bearing)
            y = r * math.sin(bearing)
            if tx is not None and math.hypot(x - tx, y - ty) < self._target_exclusion:
                continue
            if 0.0 < x <= self._avoid_lookahead and abs(y) <= self._corridor_half_width:
                corridor_min = min(corridor_min, x)
            if r < self._avoid_radius:
                w = (1.0 / max(r, 0.05) - inv_r)
                if y > 0.0:
                    left_prox = max(left_prox, w)
                else:
                    right_prox = max(right_prox, w)
        self._corridor_min = corridor_min
        # Obstacle on the right pushes left (+z CCW) and vice versa.
        self._wz_avoid = self._k_avoid * (right_prox - left_prox)

    def _on_depth(self, msg: PointCloud2):
        """Depth-cloud corridor guard for obstacles under the lidar plane."""
        now = time.monotonic()
        if not self._active or now - self._last_depth_proc < self._depth_period:
            return
        if self._cam_rot is None and not self._resolve_camera_tf(msg.header.frame_id):
            return
        self._last_depth_proc = now
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except (AttributeError, TypeError):
            # Very old sensor_msgs_py without read_points_numpy: skip depth.
            return
        if pts.size == 0:
            self._depth_corridor_min = math.inf
            self._depth_stamp = now
            return
        p = pts.astype(np.float32) @ self._cam_rot.T + self._cam_trans
        x, y, z = p[:, 0], p[:, 1], p[:, 2]
        sel = ((z > self._depth_band_lo) & (z < self._depth_band_hi)
               & (x > 0.0) & (x <= self._avoid_lookahead)
               & (np.abs(y) <= self._corridor_half_width))
        if self._target is not None:
            tx, ty = self._target
            sel &= (np.hypot(x - tx, y - ty) > self._target_exclusion)
        self._depth_corridor_min = float(x[sel].min()) if sel.any() else math.inf
        self._depth_stamp = now
        # Remember low points in a wider apron than the live corridor, anchored
        # in odom, so they survive the camera's close-range blind zone.
        mem = ((z > self._depth_band_lo) & (z < self._depth_band_hi)
               & (x > 0.0) & (x <= self._mem_range)
               & (np.abs(y) <= self._mem_half_width))
        if self._target is not None:
            mem &= (np.hypot(x - self._target[0], y - self._target[1])
                    > self._target_exclusion)
        if mem.any():
            pose = self._odom_pose()
            if pose is not None:
                ox, oy, oyaw = pose
                c, s = math.cos(oyaw), math.sin(oyaw)
                wx = ox + x[mem] * c - y[mem] * s
                wy = oy + x[mem] * s + y[mem] * c
                v = self._mem_voxel
                for px, py in zip(np.round(wx / v) * v, np.round(wy / v) * v):
                    self._obstacle_mem[(float(px), float(py))] = now

    def _odom_pose(self):
        """(x, y, yaw) of base_link in odom, or None if TF is not up."""
        try:
            t = self._tf_buffer.lookup_transform('odom', 'base_link',
                                                 rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                2.0 * math.atan2(q.z, q.w))

    def _memory_corridor_min(self, now):
        """Nearest remembered obstacle inside the live corridor, base_link."""
        if not self._obstacle_mem:
            return math.inf
        cutoff = now - self._mem_ttl
        self._obstacle_mem = {k: t for k, t in self._obstacle_mem.items()
                              if t >= cutoff}
        if not self._obstacle_mem:
            return math.inf
        pose = self._odom_pose()
        if pose is None:
            return math.inf
        ox, oy, oyaw = pose
        pts = np.array(list(self._obstacle_mem.keys()), dtype=np.float32)
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = pts[:, 0] - ox, pts[:, 1] - oy
        bx = dx * c - dy * s
        by = dx * s + dy * c
        sel = (bx > 0.0) & (bx <= self._avoid_lookahead) \
            & (np.abs(by) <= self._corridor_half_width)
        if self._target is not None:
            sel &= (np.hypot(bx - self._target[0], by - self._target[1])
                    > self._target_exclusion)
        return float(bx[sel].min()) if sel.any() else math.inf

    def _resolve_camera_tf(self, frame_id):
        """Cache the static camera-optical -> base_link transform as a matrix."""
        try:
            t = self._tf_buffer.lookup_transform('base_link', frame_id,
                                                 rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        self._cam_rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)
        tr = t.transform.translation
        self._cam_trans = np.array([tr.x, tr.y, tr.z], dtype=np.float32)
        self.get_logger().info('Depth corridor guard active (camera TF cached)')
        return True

    # --- control (sole cmd_vel writer) ----------------------------------------------
    def _held_clearance(self, now):
        """Worst (smallest) corridor clearance seen in the hold window."""
        fresh = min(self._corridor_min, self._memory_corridor_min(now))
        if now - self._depth_stamp < self._depth_stale:
            fresh = min(fresh, self._depth_corridor_min)
        self._clearance_log.append((now, fresh))
        cutoff = now - self._clearance_hold
        self._clearance_log = [(t, c) for t, c in self._clearance_log if t >= cutoff]
        return min(c for _, c in self._clearance_log)

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if self._active and self._target is not None:
            if now - self._target_seen > self._lost_timeout:
                self._target = None
                self._set_status('searching')
                self._stop_until = now + STOP_GRACE
                return
            dist = math.hypot(*self._target)
            bearing = math.atan2(self._target[1], self._target[0])
            corridor = self._held_clearance(now)
            twist = Twist()
            if dist > self._stop_dist:
                vx = self._kp_lin * (dist - self._standoff)
                # Follow only, never reverse toward the thing behind the error.
                vx = max(0.0, min(self._max_vx, vx))
                # Corridor gate: scale to zero as the path ahead closes down,
                # against the min-held clearance so one blank scan can't
                # un-see an obstacle.
                zone = self._avoid_lookahead - self._avoid_hard_stop
                free = corridor - self._avoid_hard_stop
                scale = max(0.0, min(1.0, free / zone)) if zone > 0 else 1.0
                vx *= scale
                blocked = (scale == 0.0 and vx == 0.0 and dist > self._standoff)
            else:
                vx = 0.0
                blocked = False
            # Slew-limit vx so a reopened corridor ramps instead of stepping.
            step = self._max_accel * max(dt, 0.0)
            self._vx_out += max(-4 * step, min(step, vx - self._vx_out))
            self._vx_out = max(0.0, self._vx_out)
            twist.linear.x = self._vx_out
            twist.angular.z = max(-self._max_wz, min(
                self._max_wz, self._kp_ang * bearing + self._wz_avoid))
            self._pub.publish(twist)
            new_status = 'blocked' if blocked else 'locked'
            if self._status != new_status:
                self._set_status(new_status)
            # Avoidance telemetry, 1 Hz while driving (throttled).
            depth_fresh = now - self._depth_stamp < self._depth_stale
            self.get_logger().info(
                'dist %.2f brg %.0f° | corr lidar %.2f depth %s held %.2f '
                'mem %d | wz_avoid %+.2f | vx %.2f wz %+.2f%s' % (
                    dist, math.degrees(bearing), self._corridor_min,
                    ('%.2f' % self._depth_corridor_min) if depth_fresh else 'stale',
                    corridor, len(self._obstacle_mem),
                    self._wz_avoid, twist.linear.x, twist.angular.z,
                    ' BLOCKED' if blocked else ''),
                throttle_duration_sec=1.0)
        elif self._stop_until > now:
            self._pub.publish(Twist())

    def _set_status(self, s):
        self._status = s
        self._publish_status()

    def _publish_status(self):
        msg = String()
        if self._status in ('locked', 'blocked') and self._target is not None:
            msg.data = '%s|%.2f|%.0f' % (
                self._status, math.hypot(*self._target),
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
