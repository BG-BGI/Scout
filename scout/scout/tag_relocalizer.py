#!/usr/bin/env python3
"""Boot relocalization from a registered AprilTag (the portable home base).

The site policy loads a saved map in localization mode, which has to guess a
start pose (map origin unless told otherwise) — and a robot powered on
anywhere else is silently mislocalized, planning through walls. This node
closes that gap: on the first sighting of a *registered* tag (scout-skills'
registry at tags.db, which stores each tag's last surveyed map pose), it
solves the robot's map pose from the live TF of the detection and publishes
/initialpose, which localization_slam_toolbox_node accepts as a new anchor.
Scan matching polishes from there.

One seed per boot, deliberately: after the first fix the node goes quiet so
it never fights slam_toolbox's own tracking. /tag_relocalizer/reseed
(std_srvs/Trigger) re-arms it for the next sighting.

Portable-base contract: the registry pose is only as fresh as the last
detect_tags/tag_watch sighting made while WELL-LOCALIZED. Move the base while
the robot is off and the seed will be confidently wrong — re-survey the tag
(drive to it and run detect_tags) after any move, before the next cold boot.

Geometry (SE2 — tags are assumed roughly vertical, like the base's face):
the registry stores map_x/map_y (tag centre) and map_yaw = the standoff
heading, i.e. pointing AT the face, so the face normal in map is
map_yaw + pi. The live detection gives the tag in base_link; the face normal
there is the tag frame's z-axis floor projection, sign-disambiguated toward
the robot (the robot is by definition on the visible side — same convention
as scout-skills' map_geometry). Then
    theta = (map_yaw + pi) - atan2(n_y, n_x)
    p_robot = p_tag_map - R(theta) @ p_tag_base
"""

import math
import os
import sqlite3

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from scout.core.geometry import yaw_to_quat_zw
from scout.node_util import lookup_matrix, run_node

# Same bind mount the maps come through; scout-skills writes it as /maps.
DEFAULT_TAGS_DB = '/ros_ws/src/sites/active/maps/tags.db'

# Below this floor-projection of the tag z-axis the face is lying flat and
# has no usable heading (mirrors scout-skills map_geometry).
MIN_NORMAL_PROJ = 0.2


def _norm_family(fam):
    """'tagStandard52h13' / 'Standard52h13' / '36h11' -> comparable form."""
    f = fam.lower()
    return f[3:] if f.startswith('tag') else f


class TagRelocalizer(Node):

    def __init__(self):
        super().__init__('tag_relocalizer')
        self.declare_parameter('tags_db', DEFAULT_TAGS_DB)
        # Beyond this the single-view tag pose (especially yaw) is too noisy
        # to seed from; wait for a closer look.
        self.declare_parameter('max_tag_dist_m', 3.0)
        self.declare_parameter('cov_xy', 0.05 ** 2)
        self.declare_parameter('cov_yaw', math.radians(5.0) ** 2)

        self._done = False
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._on_detections, 10)
        self.create_service(
            Trigger, '/tag_relocalizer/reseed', self._on_reseed)
        self.get_logger().info('waiting for a registered tag to seed /initialpose')

    # --- registry -------------------------------------------------------------

    def _registered(self, family, tag_id):
        """Registry row with a surveyed map pose, or None. Read fresh each
        sighting — the portable base is re-surveyed whenever detect_tags or
        tag_watch sees it, and this must pick up the newest pose."""
        path = self.get_parameter('tags_db').value
        if not os.path.exists(path):
            return None
        try:
            db = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
            db.row_factory = sqlite3.Row
            rows = db.execute(
                'SELECT * FROM tags WHERE map_x IS NOT NULL').fetchall()
            db.close()
        except sqlite3.Error as e:
            self.get_logger().warn(f'tags.db read failed: {e}')
            return None
        nf = _norm_family(family)
        for r in rows:
            if r['tag_id'] == tag_id and _norm_family(r['family']) == nf:
                return dict(r)
        return None

    # --- seeding ----------------------------------------------------------------

    def _on_reseed(self, _req, resp):
        self._done = False
        resp.success = True
        resp.message = 're-armed: next registered-tag sighting seeds /initialpose'
        return resp

    def _on_detections(self, msg):
        if self._done or not msg.detections:
            return
        # Nearest registered tag in this frame wins.
        best = None
        for det in msg.detections:
            entry = self._registered(det.family, det.id)
            if entry is None:
                continue
            solved = self._solve(det, entry)
            if solved and (best is None or solved[0] < best[0]):
                best = solved
        if best is None:
            return
        dist, name, theta, pose = best
        self._pub.publish(pose)
        self._done = True
        self.get_logger().info(
            'seeded /initialpose from tag %r %.2f m away: map (%.2f, %.2f) yaw %.1f deg'
            % (name, dist, pose.pose.pose.position.x,
               pose.pose.pose.position.y, math.degrees(theta)))

    def _solve(self, det, entry):
        """(distance, name, theta, PoseWithCovarianceStamped) or None."""
        # apriltag_ros 3.4.0 publishes the child frame as "<family>:<id>"
        # with no "tag" prefix; older configs carried one. Try both.
        m = None
        for frame in (f'{det.family}:{det.id}', f'tag{det.family}:{det.id}'):
            m = lookup_matrix(self._tf, 'base_link', frame)
            if m is not None:
                break
        if m is None:
            return None
        rot, t = m

        dist = math.hypot(float(t[0]), float(t[1]))
        if dist > self.get_parameter('max_tag_dist_m').value:
            return None

        # Face normal in base_link: tag z-axis (third rotation column),
        # floor-projected, pointed at the robot (the base_link origin).
        nx, ny = float(rot[0, 2]), float(rot[1, 2])
        n = math.hypot(nx, ny)
        if n < MIN_NORMAL_PROJ:
            return None  # tag lying flat — no heading to seed from
        nx, ny = nx / n, ny / n
        if nx * -float(t[0]) + ny * -float(t[1]) < 0.0:
            nx, ny = -nx, -ny

        # map_yaw points AT the face (standoff heading) => face normal in map.
        face_map = entry['map_yaw'] + math.pi
        theta = face_map - math.atan2(ny, nx)
        c, s = math.cos(theta), math.sin(theta)
        rx = entry['map_x'] - (c * float(t[0]) - s * float(t[1]))
        ry = entry['map_y'] - (s * float(t[0]) + c * float(t[1]))

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = rx
        pose.pose.pose.position.y = ry
        qz, qw = yaw_to_quat_zw(theta)
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        cov_xy = self.get_parameter('cov_xy').value
        pose.pose.covariance[0] = cov_xy
        pose.pose.covariance[7] = cov_xy
        pose.pose.covariance[35] = self.get_parameter('cov_yaw').value
        return dist, entry['name'], theta, pose


def main(args=None):
    run_node(TagRelocalizer, args=args)


if __name__ == '__main__':
    main()
