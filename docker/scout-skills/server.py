"""Scout skills MCP endpoint (http://<pi>:9001/mcp, streamable-http).

Task-level companion to the generic ros-mcp container: where that exposes raw
ROS primitives, this exposes skills — the map as an image a vision model can
read, non-blocking navigation, and YOLO object detection fused with D455
aligned depth into map-frame positions. All ROS access rides the rosbridge
websocket on 127.0.0.1:9090 (host networking), so this image carries no
ROS/DDS. ⚠ No auth — LAN-trust only, same caveat as ros_mcp.

Nav model: go_to publishes a map-framed /goal_pose and RETURNS. It never
blocks a tool call on a drive — ros-mcp's send_action_goal blocks until
result/timeout and a timed-out call leaves the goal RUNNING. Poll nav_status;
nav_cancel clears every active goal (the same CancelGoal call as the webui's
cancel button). A goal outliving its client is still the standing hazard:
killing the MCP client, the chat, or this container does NOT stop the robot.
"""

import asyncio
import base64
import json
import math

import numpy as np
from fastmcp import FastMCP
from fastmcp.exceptions import ToolError
from fastmcp.utilities.types import Image

from detect import annotate, detect
from render import render_map
from rosbridge import RosBridge
from tf import TfTree

mcp = FastMCP("scout-skills")

# action_msgs/msg/GoalStatus values.
NAV_STATUS = {
    0: "unknown",
    1: "accepted",
    2: "driving",
    3: "canceling",
    4: "arrived",
    5: "canceled",
    6: "aborted",
}


def _pose_of(msg: dict | None) -> dict | None:
    """slam_toolbox /pose (PoseWithCovarianceStamped) → {x, y, yaw}."""
    if msg is None:
        return None
    p = msg["pose"]["pose"]
    q = p["orientation"]
    return {
        "x": round(p["position"]["x"], 3),
        "y": round(p["position"]["y"], 3),
        # Planar quaternion; same shortcut the webui uses.
        "yaw": round(2 * math.atan2(q["z"], q["w"]), 3),
    }


async def _robot_pose(rb: RosBridge) -> dict | None:
    return _pose_of(
        await rb.subscribe_once(
            "/pose", "geometry_msgs/msg/PoseWithCovarianceStamped", timeout=1.5
        )
    )


async def _nav_status(rb: RosBridge, timeout: float = 2.5) -> dict | None:
    """Latest GoalStatusArray entry, or None if nothing arrived in the window
    (idle, or no goal since nav2 start — the status topic only re-publishes on
    transitions unless rosbridge matched its transient_local durability)."""
    msg = await rb.subscribe_once(
        "/navigate_to_pose/_action/status",
        "action_msgs/msg/GoalStatusArray",
        timeout=timeout,
    )
    if not msg or not msg["status_list"]:
        return None
    latest = msg["status_list"][-1]
    return {
        "status": NAV_STATUS.get(latest["status"], str(latest["status"])),
        "goal_stamp": latest["goal_info"]["stamp"]["sec"],
    }


@mcp.tool(annotations={"readOnlyHint": True})
async def get_map() -> list:
    """Snapshot the SLAM occupancy grid as an image, with the robot's pose
    drawn as a red triangle (pointing along its heading). White = free,
    black = wall/obstacle, gray = unexplored. Returns metadata first (map
    size, resolution, robot pose, and the pixel→world formula for picking
    go_to coordinates off the image), then the image."""
    async with RosBridge() as rb:
        # slam_toolbox publishes /map at 0.5 Hz; 6 s covers three periods.
        grid = await rb.subscribe_once(
            "/map", "nav_msgs/msg/OccupancyGrid", timeout=6.0
        )
        if grid is None:
            raise ToolError(
                "no /map within 6 s — slam service down, or mapping not started"
            )
        robot = await _robot_pose(rb)

    png, scale = render_map(grid, robot)
    info = grid["info"]
    origin = info["origin"]["position"]
    meta = {
        "map_cells": [info["width"], info["height"]],
        "resolution_m_per_cell": info["resolution"],
        "origin_world": [round(origin["x"], 3), round(origin["y"], 3)],
        "image_scale_px_per_cell": scale,
        "robot": robot or "unknown (/pose silent — localization not running?)",
        "pixel_to_world": (
            "world_x = origin_world[0] + (px / scale) * resolution; "
            "world_y = origin_world[1] + ((image_height - py) / scale) * resolution"
        ),
    }
    return [json.dumps(meta), Image(data=png, format="png")]


@mcp.tool
async def go_to(x: float, y: float, yaw: float | None = None) -> dict:
    """Send Scout driving to a map-frame coordinate (meters; use get_map for
    the frame). Returns IMMEDIATELY while the robot drives — poll nav_status,
    stop with nav_cancel. yaw (radians) defaults to facing the direction of
    travel. Nav2 stops within its 0.15 m goal tolerance, so expect arrival
    ~0.13 m short of the exact point."""
    async with RosBridge() as rb:
        robot = await _robot_pose(rb)
        if yaw is None:
            yaw = math.atan2(y - robot["y"], x - robot["x"]) if robot else 0.0
        await rb.publish(
            "/goal_pose",
            "geometry_msgs/msg/PoseStamped",
            {
                # Always map-framed: an odom-framed goal works ~10 s then
                # fails on every replan once its stamp ages out of tf.
                "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": math.sin(yaw / 2),
                        "w": math.cos(yaw / 2),
                    },
                },
            },
        )
        # The accept transition republishes the status topic, so this window
        # confirms delivery regardless of durability matching.
        status = await _nav_status(rb, timeout=3.0)

    accepted = status is not None and status["status"] in ("accepted", "driving")
    return {
        "goal": {"x": x, "y": y, "yaw": round(yaw, 3)},
        "nav": status or "no status transition seen",
        "accepted": accepted,
        "note": (
            "driving; poll nav_status, stop with nav_cancel"
            if accepted
            else "goal publish not confirmed — check nav_status before resending"
        ),
    }


@mcp.tool(annotations={"readOnlyHint": True})
async def nav_status() -> dict:
    """Current navigation state (accepted/driving/canceling/arrived/canceled/
    aborted) and the robot's map-frame pose. 'no recent status traffic' means
    idle or no goal since nav2 started. ⚠ 'aborted' does NOT mean stopped —
    already-dispatched recovery behaviors keep the robot moving after the
    abort; nav_cancel if it must stop."""
    async with RosBridge() as rb:
        status = await _nav_status(rb)
        robot = await _robot_pose(rb)
    return {
        "nav": status or "no recent status traffic (idle, or no goal yet)",
        "robot": robot or "unknown (/pose silent)",
    }


COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"


def _img_to_np(msg: dict) -> np.ndarray:
    """sensor_msgs/Image → numpy. rosbridge serializes uint8[] as base64 in
    JSON (a plain int list from older bridges is tolerated). Handles the three
    encodings this robot produces: rgb8/bgr8 color, 16UC1 aligned depth (mm,
    little-endian — is_bigendian is always 0 here)."""
    raw = msg["data"]
    buf = base64.b64decode(raw) if isinstance(raw, str) else bytes(raw)
    h, w, step, enc = msg["height"], msg["width"], msg["step"], msg["encoding"]
    if enc in ("rgb8", "bgr8"):
        # Slice per row via step in case of padding, then drop it.
        arr = np.frombuffer(buf, np.uint8).reshape(h, step)[:, : w * 3]
        arr = arr.reshape(h, w, 3)
        return arr[:, :, ::-1] if enc == "bgr8" else arr
    if enc == "16UC1":
        return np.frombuffer(buf, "<u2").reshape(h, step // 2)[:, :w]
    raise ToolError(f"unsupported image encoding {enc!r}")


def _median_depth_m(depth: np.ndarray, box: list[float]) -> float | None:
    """Median valid depth over the central half of the box, meters. Zeros are
    'no stereo return'; <20 valid px means the object has effectively no depth
    (out of the D455's 0.6–6 m band, or all holes)."""
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    qw, qh = (x2 - x1) / 4, (y2 - y1) / 4
    patch = depth[
        max(0, round(cy - qh)) : round(cy + qh) + 1,
        max(0, round(cx - qw)) : round(cx + qw) + 1,
    ]
    valid = patch[patch > 0]
    if valid.size < 20:
        return None
    return float(np.median(valid)) / 1000.0


@mcp.tool(annotations={"readOnlyHint": True})
async def detect_objects(min_confidence: float = 0.35) -> list:
    """Detect objects in Scout's camera view (YOLO11n, 80 COCO classes) and
    locate them in 3D: each detection gets a distance from the camera and,
    when depth + TF cooperate, a map-frame position you can hand straight to
    go_to. Returns detection JSON first, then the camera frame annotated with
    labeled boxes. For objects outside the COCO label set, use the raw camera
    view instead (ros-mcp subscribe_once + view_saved_image) and read the
    frame visually."""
    async with RosBridge() as rb:
        color = await rb.subscribe_once(
            COLOR_TOPIC, "sensor_msgs/msg/Image", timeout=5.0
        )
        if color is None:
            raise ToolError(
                f"no frame on {COLOR_TOPIC} within 5 s — is the robot service up?"
            )
        info = await rb.subscribe_once(
            CAMERA_INFO_TOPIC, "sensor_msgs/msg/CameraInfo", timeout=3.0
        )
        # Aligned depth + TF are enrichment: detection still works without
        # them, the objects just come back with null distance/position.
        depth_msg = await rb.subscribe_once(
            DEPTH_TOPIC, "sensor_msgs/msg/Image", timeout=4.0
        )
        tree = TfTree()
        static_msg = await rb.subscribe_once(
            "/tf_static", "tf2_msgs/msg/TFMessage", timeout=2.0
        )
        if static_msg is not None:
            tree.add_message(static_msg)
        for m in await rb.subscribe_collect(
            "/tf", "tf2_msgs/msg/TFMessage", duration=0.8
        ):
            tree.add_message(m)

    rgb = _img_to_np(color)
    # ~0.5–1 s of CPU on the Pi — off the event loop so nav/status calls
    # keep answering.
    dets = await asyncio.to_thread(detect, rgb, min_confidence)

    depth = _img_to_np(depth_msg) if depth_msg is not None else None
    notes: list[str] = []
    if depth is None:
        notes.append(
            f"no aligned depth on {DEPTH_TOPIC} — distances/positions omitted "
            "(align_depth.enable false, or robot service predates it)"
        )
    if info is not None and depth is not None:
        k = info["k"]
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        cam_frame = info["header"]["frame_id"]
        tf_missing = False
        for d in dets:
            z = _median_depth_m(depth, d["box"])
            d["distance_m"] = round(z, 2) if z is not None else None
            if z is None:
                continue
            u = (d["box"][0] + d["box"][2]) / 2
            v = (d["box"][1] + d["box"][3]) / 2
            # Optical-frame ray through the box center at the median depth.
            pt = np.array([(u - cx) / fx * z, (v - cy) / fy * z, z])
            world = tree.to_ancestor(pt, cam_frame, "map")
            if world is not None:
                d["position_map"] = [round(float(c), 2) for c in world[:2]]
            else:
                tf_missing = True
        if tf_missing:
            notes.append(
                f"TF chain {cam_frame}→map incomplete — positions omitted. "
                "Static transforms replay only if rosbridge matched /tf_static's "
                "transient_local durability; map→odom needs slam up."
            )
    png = await asyncio.to_thread(annotate, rgb, dets)
    meta = {
        "objects": dets,
        "count": len(dets),
        "min_confidence": min_confidence,
        "notes": notes or "position_map feeds go_to directly",
    }
    return [json.dumps(meta), Image(data=png, format="png")]


@mcp.tool
async def nav_cancel() -> dict:
    """STOP navigation: cancel every active Nav2 goal (zeroed CancelGoal =
    cancel-all). The software e-stop — a goal survives its client dying, so
    this is the only way to clear one short of restarting nav2. Deceleration
    is a coast, not a brake (200 ms deadman, free-wheeling idle)."""
    async with RosBridge() as rb:
        values = await rb.call_service(
            "/navigate_to_pose/_action/cancel_goal",
            "action_msgs/srv/CancelGoal",
            {
                "goal_info": {
                    "goal_id": {"uuid": [0] * 16},
                    "stamp": {"sec": 0, "nanosec": 0},
                }
            },
        )
        status = await _nav_status(rb, timeout=2.0)
    # return_code 0 = none active (nothing to cancel), which still means "not
    # driving" — report it as success with the detail visible.
    return {
        "return_code": values.get("return_code"),
        "goals_canceling": len(values.get("goals_canceling", [])),
        "nav": status or "no status transition seen",
    }


# --- frontier exploration (explore_lite, compose profile `explore`) ---------
#
# The container is profile-gated and STAYS operator-started (`docker compose
# --profile explore up -d explore`) — mounting the docker socket into a
# no-auth LAN MCP container would let anyone on the LAN root the Pi, so this
# server only pauses/resumes a running explorer via its /explore/resume Bool
# subscription. rosapi (launched with rosbridge_websocket) tells us whether
# the node is up at all.

EXPLORE_RESUME_TOPIC = "/explore/resume"


async def _explore_running(rb: RosBridge) -> bool:
    values = await rb.call_service(
        "/rosapi/subscribers",
        "rosapi_msgs/srv/Subscribers",
        {"topic": EXPLORE_RESUME_TOPIC},
    )
    return bool(values.get("subscribers"))


async def _set_explore(active: bool) -> dict:
    async with RosBridge() as rb:
        if not await _explore_running(rb):
            raise ToolError(
                "explore node is not running. It is deliberately not startable "
                "from here — the operator must run "
                "`docker compose --profile explore up -d explore` on the Pi."
            )
        await rb.publish(
            EXPLORE_RESUME_TOPIC, "std_msgs/msg/Bool", {"data": active}
        )
        robot = await _robot_pose(rb)
    return {
        "explore": "resumed" if active else "paused",
        "robot": robot or "unknown (/pose silent)",
    }


@mcp.tool
async def explore_resume() -> dict:
    """START/RESUME autonomous frontier exploration — the robot drives itself
    to unexplored map frontiers until none remain (then returns to its start).
    The explore node must already be running (operator-started); this only
    un-pauses it. Pause with explore_pause; a Nav2 goal already dispatched
    also needs nav_cancel to actually stop the robot."""
    return await _set_explore(True)


@mcp.tool
async def explore_pause() -> dict:
    """Pause autonomous exploration. ⚠ Pausing stops NEW frontier goals, not
    the current drive — follow with nav_cancel to actually stop the robot."""
    return await _set_explore(False)


@mcp.tool(annotations={"readOnlyHint": True})
async def explore_status() -> dict:
    """Whether the explore node is up, how many frontier markers it last
    published (0 = space fully explored or exploration stopped), current nav
    state, and robot pose. Frontier check waits up to 6 s (the planner runs
    at 0.2 Hz)."""
    async with RosBridge() as rb:
        running = await _explore_running(rb)
        frontiers = None
        if running:
            markers = await rb.subscribe_once(
                "/explore/frontiers",
                "visualization_msgs/msg/MarkerArray",
                timeout=6.0,
            )
            if markers is not None:
                frontiers = len(markers.get("markers", []))
        status = await _nav_status(rb)
        robot = await _robot_pose(rb)
    return {
        "explore_node_running": running,
        "frontier_markers": (
            frontiers
            if frontiers is not None
            else "no marker traffic in 6 s"
            if running
            else None
        ),
        "nav": status or "no recent status traffic",
        "robot": robot or "unknown (/pose silent)",
    }


if __name__ == "__main__":
    mcp.run(transport="http", host="0.0.0.0", port=9001, path="/mcp")
