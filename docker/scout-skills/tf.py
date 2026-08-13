"""Just-enough TF: compose a point from one frame to another out of raw
/tf + /tf_static traffic. No tf2 — the tree here is small and static during a
one-shot tool call, so latest-transform-wins is fine (no time interpolation).

TransformStamped semantics: header.frame_id = parent, child_frame_id = child,
transform = the child frame's pose in the parent → p_parent = R @ p_child + t.
A point is carried to an ancestor by walking child→parent applying that.
"""

import numpy as np


def quat_to_mat(q: dict) -> np.ndarray:
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class TfTree:
    def __init__(self):
        # child frame → (parent frame, R, t); TF is a tree so one parent each.
        self._up: dict[str, tuple[str, np.ndarray, np.ndarray]] = {}

    def add_message(self, tf_msg: dict) -> None:
        """Feed a tf2_msgs/TFMessage (from /tf or /tf_static)."""
        for ts in tf_msg.get("transforms", []):
            parent = ts["header"]["frame_id"].lstrip("/")
            child = ts["child_frame_id"].lstrip("/")
            tr = ts["transform"]
            t = np.array(
                [
                    tr["translation"]["x"],
                    tr["translation"]["y"],
                    tr["translation"]["z"],
                ]
            )
            self._up[child] = (parent, quat_to_mat(tr["rotation"]), t)

    def to_ancestor(
        self, point: np.ndarray, frame: str, ancestor: str
    ) -> np.ndarray | None:
        """Carry `point` (3,) from `frame` up the tree to `ancestor`, or None
        when the chain is broken (a missing /tf_static replay shows up here)."""
        p = np.asarray(point, dtype=float)
        cur = frame.lstrip("/")
        ancestor = ancestor.lstrip("/")
        seen = 0
        while cur != ancestor:
            hop = self._up.get(cur)
            if hop is None or seen > 32:
                return None
            parent, rot, trans = hop
            p = rot @ p + trans
            cur = parent
            seen += 1
        return p

    def frames(self) -> list[str]:
        return sorted(self._up)
