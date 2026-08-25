"""Vendored subset of scout.core.geometry — this container talks to ROS only
over rosbridge and cannot import the scout package, so the planar-quaternion
helpers are copied here. Function bodies are kept textually identical to
scout/scout/core/geometry.py and frozen by the SC10 sync test
(scout/test/test_profile_constants.py); fix drift there, not by editing one
copy.
"""

import math

import numpy as np


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
