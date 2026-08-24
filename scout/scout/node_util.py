"""ROS-side glue shared by the scout nodes (imports rclpy/tf2 — NOT pure, so it
lives here, not in scout.core).

- run_node: the one main() — init, spin, tidy shutdown — replacing twelve
  slightly-different copies (one of which, link_watchdog, forgot to call
  rclpy.shutdown() at all).
- lookup_pose2 / lookup_matrix: the TF-exception-wrapped lookups that were
  duplicated across the depth-consumer nodes (now just patrol_capture).
- cancel_nav_goals: the zeroed-uuid cancel-all on the bt_navigator actions,
  shared by link_watchdog and nav_manager (ADR-0018) so a third copy of the
  CancelGoal plumbing never appears.
"""

import numpy as np
import rclpy
import tf2_ros
from action_msgs.srv import CancelGoal
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


def cancel_nav_goals(clients, active=None):
    """Fire a zeroed-uuid CancelGoal (= cancel ALL goals) at each ready cancel
    client. `clients` is {action_name: Client}; `active` optionally filters to
    the actions currently holding a goal (link_watchdog's stash logic). Fully
    async (SC11 — a sync call here deadlocks the executor silently); returns
    the action names actually fired so the caller can report them."""
    fired = []
    for action, client in clients.items():
        if active is not None and not active.get(action):
            continue
        if client.service_is_ready():
            client.call_async(CancelGoal.Request())
            fired.append(action)
    return fired


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
