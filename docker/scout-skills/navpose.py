"""Pure parsing helpers for pose / nav topics (no I/O), so they unit-test off the
robot. server.py does the rosbridge reads; this module only interprets them."""

import math

from geometry import planar_yaw


def pose_of(msg: dict | None) -> dict | None:
    """slam_toolbox/amcl /pose (PoseWithCovarianceStamped) -> {x, y, yaw}."""
    if msg is None:
        return None
    p = msg["pose"]["pose"]
    q = p["orientation"]
    return {
        "x": round(p["position"]["x"], 3),
        "y": round(p["position"]["y"], 3),
        "yaw": round(planar_yaw(q["z"], q["w"]), 3),
    }


def pose_from_tf(tf_msgs: list[dict]) -> dict | None:
    """Map-frame pose composed from map->odom (slam/amcl) and odom->base_link
    (EKF) in a batch of /tf messages; None until both have been seen. /tf never
    goes quiet, unlike /pose (keyframe-gated), so this works for a parked robot."""
    tfs: dict[str, tuple] = {}
    for m in tf_msgs:
        for t in m["transforms"]:
            tr, q = t["transform"]["translation"], t["transform"]["rotation"]
            tfs[t["header"]["frame_id"]] = (
                tr["x"], tr["y"], planar_yaw(q["z"], q["w"])
            )
    if not ({"map", "odom"} <= tfs.keys()):
        return None
    mx, my, myaw = tfs["map"]
    ox, oy, oyaw = tfs["odom"]
    return {
        "x": round(mx + ox * math.cos(myaw) - oy * math.sin(myaw), 3),
        "y": round(my + ox * math.sin(myaw) + oy * math.cos(myaw), 3),
        "yaw": round(math.atan2(math.sin(myaw + oyaw), math.cos(myaw + oyaw)), 3),
    }
