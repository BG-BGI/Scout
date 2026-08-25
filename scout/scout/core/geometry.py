"""Planar geometry helpers shared by the motion/perception nodes.

These were copy-pasted across the motion/perception nodes (and
the scout-skills server): the yaw-from-quaternion shortcut, the quaternion ->
rotation-matrix block, angle wrapping, and the base<->anchor-frame transforms.
Pure math (stdlib + numpy) so the tests need no ROS.
"""

import math

import numpy as np

# A planar pose in some anchor frame (odom / map): (x, y, yaw).
Pose2 = tuple


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def planar_yaw(qz, qw):
    """Yaw of a quaternion assumed planar (roll = pitch = 0): 2*atan2(z, w).

    This is the shortcut used everywhere a pose is known to be flat on the
    floor; it is NOT correct for a tilted quaternion (use a full conversion)."""
    return 2.0 * math.atan2(qz, qw)


def yaw_to_quat_zw(yaw):
    """(z, w) of the planar quaternion for `yaw` (x = y = 0)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_matrix(x, y, z, w):
    """3x3 float32 rotation matrix for quaternion (x, y, z, w)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


def base_to_anchor(pose, x, y):
    """(x, y) in the robot base frame -> the anchor frame (odom / map), given
    the robot's pose (ax, ay, ayaw) in that anchor frame. Scalars or arrays."""
    ax, ay, ayaw = pose
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + x * c - y * s, ay + x * s + y * c


def anchor_to_base(pose, x, y):
    """Inverse of base_to_anchor: (x, y) in the anchor frame -> base frame."""
    ax, ay, ayaw = pose
    c, s = math.cos(-ayaw), math.sin(-ayaw)
    dx, dy = x - ax, y - ay
    return dx * c - dy * s, dx * s + dy * c
