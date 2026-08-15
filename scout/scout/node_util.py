"""ROS-side glue shared by the scout nodes (imports rclpy/tf2 — NOT pure, so it
lives here, not in scout.core).

- run_node: the one main() — init, spin, tidy shutdown — replacing twelve
  slightly-different copies (one of which, link_watchdog, forgot to call
  rclpy.shutdown() at all).
- lookup_pose2 / lookup_matrix: the TF-exception-wrapped lookups that were
  duplicated across follow_me, clutter_mapper and patrol_capture.
"""

import numpy as np
import rclpy
import tf2_ros
from rclpy.executors import ExternalShutdownException

from scout.core.geometry import planar_yaw, quat_to_matrix

_TF_EXC = (tf2_ros.LookupException, tf2_ros.ConnectivityException,
           tf2_ros.ExtrapolationException)


def run_node(node_cls, *, on_shutdown=None, args=None):
    """Standard node entry point: init, construct, spin, then destroy + shutdown.

    on_shutdown(node) runs once in the finally (e.g. blank the LED, save state,
    publish a stop) — best-effort, its exceptions are swallowed so cleanup of
    the rest still happens.
    """
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if on_shutdown is not None:
            try:
                on_shutdown(node)
            except Exception:  # noqa: BLE001 — cleanup must not mask shutdown
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def lookup_pose2(tf_buffer, target, source):
    """(x, y, yaw) of `source` in `target` at the latest time, or None on any
    TF exception (not yet available / disconnected / extrapolation)."""
    try:
        t = tf_buffer.lookup_transform(target, source, rclpy.time.Time())
    except _TF_EXC:
        return None
    tr = t.transform.translation
    q = t.transform.rotation
    return (tr.x, tr.y, planar_yaw(q.z, q.w))


def lookup_matrix(tf_buffer, target, source):
    """(3x3 rotation float32, translation float32[3]) of `source` in `target`,
    or None on any TF exception."""
    try:
        t = tf_buffer.lookup_transform(target, source, rclpy.time.Time())
    except _TF_EXC:
        return None
    q = t.transform.rotation
    tr = t.transform.translation
    return (quat_to_matrix(q.x, q.y, q.z, q.w),
            np.array([tr.x, tr.y, tr.z], dtype=np.float32))
