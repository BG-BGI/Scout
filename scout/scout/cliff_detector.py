#!/usr/bin/env python3
"""Negative-obstacle (cliff / down-stair) detector — ADR-0024.

Subscribes the D455 depth cloud, finds below-floor returns, projects them back
to the floor plane so the mark lands on the LIP of the drop, and remembers the
hits on an odom-frame grid (scout.core.cliff). Two outputs:

  * cliff_points (odom frame, z raised into the STVL 0.05-0.22 mark band):
    every remembered cell, republished on every processed cloud so the
    costmap marks self-refresh faster than any STVL decay — feeds a
    marking-only stvl source in both costmaps (nav2.yaml).
  * cliff_stop_points (base_link): a fixed 5-point cluster INSIDE
    PolygonStopFront/Turn whenever a remembered cell sits in the forward stop
    corridor, else an empty cloud — feeds a collision_monitor pointcloud
    source (collision_monitor.yaml), which hard-stops /cmd_vel_auto.

Both publish on every processed cloud, empty or not: the collision monitor's
source_timeout (2 s) treats SILENCE as a fault and stops autonomy, so a dead
camera, dead TF, or dead node fails safe by simply not publishing. That is
also why TF-unavailable skips the publish instead of sending empty clouds.
"""

import numpy as np
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from scout.core.cliff import CliffMemory, find_cliff_cells, parse_xyz, stop_gate
from scout.core.geometry import anchor_to_base, base_to_anchor
from scout.node_util import lookup_matrix, lookup_pose2, run_node

# The stop cluster must contain MORE than max_points=3 (collision_monitor.yaml)
# points, sit inside PolygonStopFront (x <= 0.24) and PolygonStopTurn, and stay
# OUTSIDE PolygonStopRear (x > 0.06) so BackUp recovery can still escape a
# front ledge. Heights sit inside the CM cliff source's min/max_height band.
_STOP_CLUSTER = [(0.10, -0.05, 0.10), (0.10, 0.05, 0.10), (0.15, 0.0, 0.10),
                 (0.20, -0.05, 0.10), (0.20, 0.05, 0.10)]


class CliffDetector(Node):
    """Depth cloud in -> latched odom-frame cliff marks + CM stop cluster out."""

    def __init__(self):
        super().__init__('cliff_detector')

        self.declare_parameter('min_range', 0.4)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('drop_base', 0.05)
        self.declare_parameter('drop_slope', 0.02)
        self.declare_parameter('max_drop', 1.5)
        self.declare_parameter('cell_size', 0.05)
        self.declare_parameter('min_points_per_cell', 3)
        self.declare_parameter('memory_s', 300.0)
        self.declare_parameter('max_cells', 4000)
        self.declare_parameter('stop_distance', 0.6)
        self.declare_parameter('stop_half_width', 0.25)
        self.declare_parameter('mark_z', 0.12)
        self.declare_parameter('process_every_n', 1)

        p = self.get_parameter
        self._gate = dict(
            min_range=float(p('min_range').value),
            max_range=float(p('max_range').value),
            drop_base=float(p('drop_base').value),
            drop_slope=float(p('drop_slope').value),
            max_drop=float(p('max_drop').value),
            cell_size=float(p('cell_size').value),
            min_points=int(p('min_points_per_cell').value),
        )
        self._stop_x = float(p('stop_distance').value)
        self._stop_half_width = float(p('stop_half_width').value)
        self._mark_z = float(p('mark_z').value)
        self._every_n = max(1, int(p('process_every_n').value))
        self._memory = CliffMemory(cell_size=float(p('cell_size').value),
                                   ttl_s=float(p('memory_s').value),
                                   max_cells=int(p('max_cells').value))
        self._count = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._marks_pub = self.create_publisher(PointCloud2, 'cliff_points', 10)
        self._stop_pub = self.create_publisher(PointCloud2,
                                               'cliff_stop_points', 10)
        self.create_subscription(PointCloud2, 'points_in', self._on_cloud,
                                 qos_profile_sensor_data)

        self.get_logger().info(
            'cliff_detector up: drop %.2f+%.3f*r m over %.1f-%.1f m, '
            'cell %.3f m, memory %.0f s, stop corridor %.2f x ±%.2f m'
            % (self._gate['drop_base'], self._gate['drop_slope'],
               self._gate['min_range'], self._gate['max_range'],
               self._gate['cell_size'], float(p('memory_s').value),
               self._stop_x, self._stop_half_width))

    def _on_cloud(self, msg):
        self._count += 1
        if self._count % self._every_n:
            return

        cam = lookup_matrix(self._tf_buffer, 'base_link', msg.header.frame_id)
        pose = lookup_pose2(self._tf_buffer, 'odom', 'base_link')
        if cam is None or pose is None:
            # No publish at all: the collision monitor's source_timeout turns
            # this silence into a stop — blind must not read as "clear".
            self.get_logger().warn(
                'TF not ready (%s->base_link / odom): cloud skipped, CM '
                'source starving deliberately' % msg.header.frame_id,
                throttle_duration_sec=10.0)
            return

        offs = {f.name: f.offset for f in msg.fields}
        xyz = parse_xyz(msg.data, msg.point_step,
                        offs.get('x', 0), offs.get('y', 4), offs.get('z', 8))
        rot, trans = cam
        cells_base = find_cliff_cells(xyz, rot, trans, **self._gate)

        now = self.get_clock().now()
        if len(cells_base):
            ox, oy = base_to_anchor(pose, cells_base[:, 0], cells_base[:, 1])
            self._memory.add(np.column_stack([ox, oy]), now.nanoseconds * 1e-9)
        else:
            self._memory.add(np.empty((0, 2)), now.nanoseconds * 1e-9)

        remembered = self._memory.cells()
        header = Header(stamp=now.to_msg(), frame_id='odom')
        marks = [(float(x), float(y), self._mark_z) for x, y in remembered]
        self._marks_pub.publish(
            point_cloud2.create_cloud_xyz32(header, marks))

        bx, by = anchor_to_base(pose, remembered[:, 0], remembered[:, 1]) \
            if len(remembered) else (np.empty(0), np.empty(0))
        firing = stop_gate(np.column_stack([bx, by]) if len(remembered)
                           else np.empty((0, 2)),
                           self._stop_x, self._stop_half_width)
        stop_header = Header(stamp=now.to_msg(), frame_id='base_link')
        self._stop_pub.publish(point_cloud2.create_cloud_xyz32(
            stop_header, _STOP_CLUSTER if firing else []))
        if firing:
            self.get_logger().warn('cliff in the stop corridor — CM stop '
                                   'cluster active', throttle_duration_sec=5.0)


def main(args=None):
    run_node(CliffDetector, args=args)


if __name__ == '__main__':
    main()
