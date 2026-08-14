"""Scout skills MCP endpoint (http://<pi>:9001/mcp, streamable-http).

Task-level companion to the generic ros-mcp container: where that exposes raw
ROS primitives, this exposes skills — the map as an image a vision model can
read, non-blocking navigation, named waypoints ("go to the kitchen"),
closed-loop relative motion (move/rotate, Nav2 bypassed), and YOLO object
detection fused with D455 aligned depth into map-frame positions. All ROS
access rides the rosbridge websocket on 127.0.0.1:9090 (host networking), so
this image carries no ROS/DDS. ⚠ No auth — LAN-trust only, same caveat as
ros_mcp.

Nav model: go_to / go_to_waypoint publish a map-framed /goal_pose and RETURN.
They never block a tool call on a drive — ros-mcp's send_action_goal blocks
until result/timeout and a timed-out call leaves the goal RUNNING. Poll
nav_status; nav_cancel clears every active goal (the same CancelGoal call as
the webui's cancel button). A goal outliving its client is still the standing
hazard: killing the MCP client, the chat, or this container does NOT stop the
robot. move/rotate are the opposite trade: they BLOCK for the (short,
capped) drive and stream /cmd_vel themselves, so their motion dies with the
call — but they skip Nav2's costmaps entirely.
"""

import asyncio
import base64
import json
import math
import os
from datetime import datetime, timezone

import numpy as np
from fastmcp import FastMCP
from fastmcp.exceptions import ToolError
from fastmcp.utilities.types import Image

from detect import annotate, detect
from motion import MAX_MOVE_M, MAX_ROTATE_RAD, run_move, run_rotate
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
    """Map-frame pose. /pose first, but slam only publishes it per PROCESSED
    scan — a stationary robot goes silent (keyframe gating). Fall back to
    composing map→odom (slam, 50 Hz) ∘ odom→base_link (EKF, 30 Hz) from /tf,
    which never stops."""
    pose = _pose_of(
        await rb.subscribe_once(
            "/pose", "geometry_msgs/msg/PoseWithCovarianceStamped", timeout=1.5
        )
    )
    if pose is not None:
        return pose
    tfs: dict[str, tuple] = {}
    for m in await rb.subscribe_collect("/tf", "tf2_msgs/msg/TFMessage", duration=1.0):
        for t in m["transforms"]:
            tr, q = t["transform"]["translation"], t["transform"]["rotation"]
            tfs[t["header"]["frame_id"]] = (
                tr["x"], tr["y"], 2 * math.atan2(q["z"], q["w"])
            )
        if {"map", "odom"} <= tfs.keys():
            break
    if not ({"map", "odom"} <= tfs.keys()):
        return None
    mx, my, myaw = tfs["map"]
    ox, oy, oyaw = tfs["odom"]
    return {
        "x": round(mx + ox * math.cos(myaw) - oy * math.sin(myaw), 3),
        "y": round(my + ox * math.sin(myaw) + oy * math.cos(myaw), 3),
        "yaw": round(math.atan2(math.sin(myaw + oyaw), math.cos(myaw + oyaw)), 3),
    }


# Both bt_navigator actions; single-goal go_to rides the first, go_through/
# patrol ride the second. Feedback message types stream ~10 Hz while driving.
NAV_ACTIONS = ("/navigate_to_pose", "/navigate_through_poses")
FEEDBACK_TYPES = {
    "/navigate_to_pose": "nav2_msgs/action/NavigateToPose_FeedbackMessage",
    "/navigate_through_poses": "nav2_msgs/action/NavigateThroughPoses_FeedbackMessage",
}


async def _nav_status(
    rb: RosBridge, timeout: float = 2.5, action: str = "/navigate_to_pose"
) -> dict | None:
    """Latest GoalStatusArray entry for one nav action, or None if nothing
    arrived in the window (idle, or no goal since nav2 start — the status
    topic only re-publishes on transitions unless rosbridge matched its
    transient_local durability)."""
    msg = await rb.subscribe_once(
        f"{action}/_action/status",
        "action_msgs/msg/GoalStatusArray",
        timeout=timeout,
    )
    if not msg or not msg["status_list"]:
        return None
    latest = msg["status_list"][-1]
    return {
        "action": action.lstrip("/"),
        "status": NAV_STATUS.get(latest["status"], str(latest["status"])),
        "goal_stamp": latest["goal_info"]["stamp"]["sec"],
    }


async def _nav_feedback(rb: RosBridge, window: float = 0.8) -> dict | None:
    """Live drive telemetry: whichever nav action is streaming feedback right
    now (they publish ~10 Hz only while a goal runs — presence IS liveness,
    unlike the transition-only status topic)."""
    for action in NAV_ACTIONS:
        msg = await rb.subscribe_once(
            f"{action}/_action/feedback", FEEDBACK_TYPES[action], timeout=window
        )
        if msg is None:
            continue
        fb = msg["feedback"]
        out = {"action": action.lstrip("/")}
        if fb.get("distance_remaining") is not None:
            out["distance_remaining_m"] = round(fb["distance_remaining"], 2)
        if fb.get("number_of_poses_remaining") is not None:
            out["poses_remaining"] = fb["number_of_poses_remaining"]
        return out
    return None


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
    # .to_image_content(): fastmcp ≥2.14 JSON-serializes list items unless
    # they are already MCP content blocks — a bare Image in a list raises
    # "Unable to serialize unknown type".
    return [json.dumps(meta), Image(data=png, format="png").to_image_content()]


def _stamped_pose(x: float, y: float, yaw: float) -> dict:
    # Always map-framed: an odom-framed goal works ~10 s then fails on every
    # replan once its stamp ages out of tf.
    return {
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
    }


async def _dispatch_goal(x: float, y: float, yaw: float | None) -> dict:
    async with RosBridge() as rb:
        robot = await _robot_pose(rb)
        if yaw is None:
            yaw = math.atan2(y - robot["y"], x - robot["x"]) if robot else 0.0
        await rb.publish(
            "/goal_pose",
            "geometry_msgs/msg/PoseStamped",
            _stamped_pose(x, y, yaw),
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


@mcp.tool
async def go_to(x: float, y: float, yaw: float | None = None) -> dict:
    """Send Scout driving to a map-frame coordinate (meters; use get_map for
    the frame). Returns IMMEDIATELY while the robot drives — poll nav_status,
    stop with nav_cancel. Sending a new go_to mid-drive smoothly RE-ROUTES
    (preemption), so chain goals without waiting for arrival; for several
    points at once use go_through, which never stops in between. yaw (radians)
    defaults to facing the direction of travel. Nav2 stops within its 0.15 m
    goal tolerance, so expect arrival ~0.13 m short of the exact point."""
    return await _dispatch_goal(x, y, yaw)


async def _dispatch_through(points: list[list[float]], final_yaw: float | None) -> dict:
    """Shared NavigateThroughPoses dispatch for go_through/patrol. Poses face
    the next point; the last takes final_yaw or the incoming heading."""
    poses = []
    for i, (x, y) in enumerate(points):
        if i < len(points) - 1:
            nx, ny = points[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        elif final_yaw is not None:
            yaw = final_yaw
        else:
            px, py = points[i - 1]
            yaw = math.atan2(y - py, x - px)
        poses.append(_stamped_pose(x, y, yaw))

    action = "/navigate_through_poses"
    async with RosBridge() as rb:
        # Subscribe BEFORE sending so the accept transition cannot race in
        # ahead of the subscription.
        await rb.subscribe(
            f"{action}/_action/status", "action_msgs/msg/GoalStatusArray"
        )
        await rb.send_action_goal(
            action,
            "nav2_msgs/action/NavigateThroughPoses",
            {"poses": poses, "behavior_tree": ""},
        )
        status = None
        msg = await rb.recv_msg(f"{action}/_action/status", timeout=3.0)
        if msg and msg["status_list"]:
            latest = msg["status_list"][-1]
            status = {
                "action": action.lstrip("/"),
                "status": NAV_STATUS.get(latest["status"], str(latest["status"])),
                "goal_stamp": latest["goal_info"]["stamp"]["sec"],
            }

    accepted = status is not None and status["status"] in ("accepted", "driving")
    return {
        "points": len(points),
        "nav": status or "no status transition seen",
        "accepted": accepted,
        "note": (
            "driving through the points without stopping; poll nav_status "
            "(poses_remaining counts down), stop with nav_cancel"
            if accepted
            else "goal not confirmed — check nav_status before resending"
        ),
    }


@mcp.tool
async def go_through(points: list[list[float]], final_yaw: float | None = None) -> dict:
    """Drive fluidly THROUGH a series of map-frame [x, y] points WITHOUT
    stopping at any of them — the right way to cover several rooms or sweep an
    area in one command (get_map for the frame; 2–20 points). Returns
    IMMEDIATELY; poll nav_status (poses_remaining counts down), stop with
    nav_cancel. final_yaw (radians) sets the arrival heading."""
    if not (2 <= len(points) <= 20):
        raise ToolError("need 2–20 [x, y] points (one point → use go_to)")
    if any(len(p) != 2 for p in points):
        raise ToolError("each point must be [x, y] in map-frame meters")
    return await _dispatch_through([list(map(float, p)) for p in points], final_yaw)


@mcp.tool
async def patrol(names: list[str], loops: int = 1) -> dict:
    """Visit saved waypoints in order, fluidly (no stop at intermediate ones),
    optionally looping the circuit — one call covers a whole patrol round.
    loops 1–10, total ≤ 50 poses. Arrives in the last waypoint's saved
    orientation. Returns IMMEDIATELY; poll nav_status, stop with nav_cancel."""
    wp = _load_waypoints()
    missing = [n for n in names if n not in wp]
    if missing:
        raise ToolError(f"unknown waypoints {missing} — have: {sorted(wp) or 'none'}")
    if not names:
        raise ToolError("names is empty")
    loops = min(max(int(loops), 1), 10)
    circuit = [[wp[n]["x"], wp[n]["y"]] for n in names] * loops
    if len(circuit) > 50:
        raise ToolError(f"{len(circuit)} poses > 50 — fewer waypoints or loops")
    final_yaw = wp[names[-1]]["yaw"]
    if len(circuit) == 1:
        result = await _dispatch_goal(circuit[0][0], circuit[0][1], final_yaw)
    else:
        result = await _dispatch_through(circuit, final_yaw)
    return {"patrol": names, "loops": loops} | result


# --- named waypoints ---------------------------------------------------------
#
# name → map-frame pose, persisted to the ./maps bind mount so they survive
# container rebuilds and sit beside the posegraphs they belong to. Waypoints
# are only meaningful on the map they were saved on — a remap invalidates them.

WAYPOINTS_PATH = os.environ.get("WAYPOINTS_PATH", "/maps/waypoints.json")


def _load_waypoints() -> dict:
    try:
        with open(WAYPOINTS_PATH) as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


def _store_waypoints(wp: dict) -> None:
    tmp = WAYPOINTS_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(wp, f, indent=1, sort_keys=True)
    os.replace(tmp, WAYPOINTS_PATH)


@mcp.tool
async def save_waypoint(name: str) -> dict:
    """Save the robot's CURRENT map-frame pose under a name ("kitchen",
    "dock"), so go_to_waypoint can return here later. Drive the robot to the
    spot first. Overwrites an existing name. Waypoints belong to the current
    map — remapping invalidates them."""
    async with RosBridge() as rb:
        robot = await _robot_pose(rb)
    if robot is None:
        raise ToolError(
            "robot pose unknown (/pose silent) — slam/localization not running"
        )
    wp = _load_waypoints()
    wp[name] = robot | {
        "saved": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    }
    _store_waypoints(wp)
    return {"saved": {name: wp[name]}, "waypoint_count": len(wp)}


@mcp.tool(annotations={"readOnlyHint": True})
async def list_waypoints() -> dict:
    """Named waypoints usable with go_to_waypoint (map-frame poses saved with
    save_waypoint). Only valid on the map they were saved on."""
    wp = _load_waypoints()
    return {"waypoints": wp, "count": len(wp)}


@mcp.tool
async def delete_waypoint(name: str) -> dict:
    """Delete a named waypoint."""
    wp = _load_waypoints()
    if name not in wp:
        raise ToolError(f"no waypoint {name!r} — have: {sorted(wp) or 'none'}")
    del wp[name]
    _store_waypoints(wp)
    return {"deleted": name, "remaining": sorted(wp)}


@mcp.tool
async def go_to_waypoint(name: str) -> dict:
    """Send Scout driving to a named waypoint saved earlier with
    save_waypoint (arrives in its saved orientation). Same semantics as
    go_to: returns immediately, poll nav_status, stop with nav_cancel."""
    wp = _load_waypoints()
    if name not in wp:
        raise ToolError(f"no waypoint {name!r} — have: {sorted(wp) or 'none'}")
    target = wp[name]
    result = await _dispatch_goal(target["x"], target["y"], target["yaw"])
    return {"waypoint": name} | result


# --- relative motion (bypasses Nav2) -----------------------------------------


async def _require_motion_idle():
    """Refuse to stream /cmd_vel on top of another commander. The status
    topics only re-publish on transitions, so also sniff /cmd_vel itself —
    a live Nav2 drive means ~30 Hz of smoother output."""
    async with RosBridge() as rb:
        for action in NAV_ACTIONS:
            status = await _nav_status(rb, timeout=1.0, action=action)
            if status is not None and status["status"] in (
                "accepted",
                "driving",
                "canceling",
            ):
                raise ToolError(
                    f"{status['action']} goal is {status['status']} — "
                    "nav_cancel before relative motion"
                )
        tw = await rb.subscribe_once(
            "/cmd_vel", "geometry_msgs/msg/Twist", timeout=0.7
        )
    if tw is not None and (
        abs(tw["linear"]["x"]) > 1e-3 or abs(tw["angular"]["z"]) > 1e-3
    ):
        raise ToolError(
            "something is already streaming non-zero /cmd_vel (Nav2 or teleop)"
        )


@mcp.tool
async def move(distance_m: float, speed: float = 0.3) -> dict:
    """SMALL PRECISE ADJUSTMENTS ONLY (dock nudges, lining up a photo) — for
    any real travel use go_to/go_through/patrol, which avoid obstacles and
    flow. Drives straight distance_m meters (negative = reverse), closed-loop
    on wheel odometry. BYPASSES Nav2 — no obstacle avoidance, so check the
    path is clear (camera_snapshot/detect_objects) before reversing or moving
    blind. BLOCKS until done (~distance/speed seconds). speed clamps to
    0.05–1.0 m/s; |distance| ≤ 5 m."""
    if not distance_m or abs(distance_m) > MAX_MOVE_M:
        raise ToolError(f"distance_m must be non-zero and ≤ {MAX_MOVE_M} m")
    await _require_motion_idle()
    return await run_move(distance_m, speed)


@mcp.tool
async def rotate(angle_rad: float, speed: float = 2.5) -> dict:
    """SMALL PRECISE ADJUSTMENTS ONLY (facing a target for a photo) — for
    travel let go_to/go_through handle heading. Rotates in place angle_rad
    radians (positive = counterclockwise/left), closed-loop on the fused gyro
    yaw. BLOCKS until done. speed clamps to 0.35–3.0 rad/s; |angle| ≤ 2π.
    Keep the default 2.5 — slower pivots make the soft tires walk sideways
    (~10 cm/rev at 1.5 vs ~2.5 cm at 2.5); the heading is accurate at any
    speed (gyro-measured)."""
    if not angle_rad or abs(angle_rad) > MAX_ROTATE_RAD:
        raise ToolError(f"angle_rad must be non-zero and ≤ {MAX_ROTATE_RAD:.3f}")
    await _require_motion_idle()
    return await run_rotate(angle_rad, speed)


@mcp.tool(annotations={"readOnlyHint": True})
async def nav_status() -> dict:
    """Current navigation state (accepted/driving/canceling/arrived/canceled/
    aborted) across both nav actions (go_to and go_through/patrol), live drive
    telemetry (distance/poses remaining — present only while driving), and the
    robot's map-frame pose. 'no recent status traffic' means idle or no goal
    since nav2 started. ⚠ 'aborted' does NOT mean stopped — already-dispatched
    recovery behaviors keep the robot moving after the abort; nav_cancel if it
    must stop."""
    async with RosBridge() as rb:
        # Feedback first: it streams continuously while driving, so it is the
        # honest liveness signal; the status topics only show transitions.
        driving = await _nav_feedback(rb)
        statuses = [
            s
            for a in NAV_ACTIONS
            if (s := await _nav_status(rb, timeout=1.2, action=a)) is not None
        ]
        robot = await _robot_pose(rb)
    return {
        "nav": statuses or "no recent status traffic (idle, or no goal yet)",
        "driving": driving or False,
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
    # .to_image_content(): fastmcp ≥2.14 JSON-serializes list items unless
    # they are already MCP content blocks — a bare Image in a list raises
    # "Unable to serialize unknown type".
    return [json.dumps(meta), Image(data=png, format="png").to_image_content()]


@mcp.tool(annotations={"readOnlyHint": True})
async def camera_snapshot() -> Image:
    """Scout's current camera view (D455 color) as an image, unprocessed —
    for reading the scene directly: text, signage, layout, anything outside
    detect_objects' 80 COCO classes. Costs no YOLO inference."""
    async with RosBridge() as rb:
        color = await rb.subscribe_once(
            COLOR_TOPIC, "sensor_msgs/msg/Image", timeout=5.0
        )
    if color is None:
        raise ToolError(
            f"no frame on {COLOR_TOPIC} within 5 s — is the robot service up?"
        )
    rgb = _img_to_np(color)
    # annotate() with no detections is just the PNG encoder.
    png = await asyncio.to_thread(annotate, rgb, [])
    return Image(data=png, format="png")


@mcp.tool
async def nav_cancel() -> dict:
    """STOP navigation: cancel every active Nav2 goal (zeroed CancelGoal =
    cancel-all). The software e-stop — a goal survives its client dying, so
    this is the only way to clear one short of restarting nav2. Deceleration
    is a coast, not a brake (200 ms deadman, free-wheeling idle)."""
    canceling = 0
    codes = {}
    async with RosBridge() as rb:
        for action in NAV_ACTIONS:
            values = await rb.call_service(
                f"{action}/_action/cancel_goal",
                "action_msgs/srv/CancelGoal",
                {
                    "goal_info": {
                        "goal_id": {"uuid": [0] * 16},
                        "stamp": {"sec": 0, "nanosec": 0},
                    }
                },
            )
            codes[action.lstrip("/")] = values.get("return_code")
            canceling += len(values.get("goals_canceling", []))
        status = await _nav_status(rb, timeout=2.0)
    # return_code 0 = none active (nothing to cancel), which still means "not
    # driving" — report it as success with the detail visible.
    return {
        "return_codes": codes,
        "goals_canceling": canceling,
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

# explore_for's auto-pause. Module-level: one budget at a time; a new
# explore_for replaces it. ⚠ Dies with this server — if the container is
# killed mid-window the pause never fires and explore keeps driving until
# paused manually (same hazard as a manual resume today).
_explore_timer: asyncio.Task | None = None
_explore_deadline: float | None = None


def _cancel_explore_timer():
    global _explore_timer, _explore_deadline
    if _explore_timer is not None and not _explore_timer.done():
        _explore_timer.cancel()
    _explore_timer = None
    _explore_deadline = None


async def _auto_pause(delay_s: float):
    await asyncio.sleep(delay_s)
    try:
        await _set_explore(False)
    except Exception:
        pass  # explore node already gone — nothing left to pause


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
    Prefer explore_for, which auto-pauses on a time budget. The explore node
    must already be running (operator-started); this only un-pauses it. Pause
    with explore_pause; a Nav2 goal already dispatched also needs nav_cancel
    to actually stop the robot."""
    _cancel_explore_timer()
    return await _set_explore(True)


@mcp.tool
async def explore_for(minutes: float) -> dict:
    """Explore autonomously for a time budget, then auto-pause — 'go explore
    for 5 minutes' in one call, no babysitting. Clamped 0.5–30 min; a new call
    replaces the running budget. Stop early with explore_pause + nav_cancel.
    ⚠ The pause timer lives in this server: if it dies mid-window, explore
    keeps driving until paused manually."""
    global _explore_timer, _explore_deadline
    minutes = min(max(minutes, 0.5), 30.0)
    result = await _set_explore(True)
    _cancel_explore_timer()
    _explore_timer = asyncio.create_task(_auto_pause(minutes * 60))
    _explore_deadline = asyncio.get_event_loop().time() + minutes * 60
    return result | {"auto_pause_in_min": minutes}


@mcp.tool
async def explore_pause() -> dict:
    """Pause autonomous exploration (also cancels an explore_for budget).
    ⚠ Pausing stops NEW frontier goals, not the current drive — follow with
    nav_cancel to actually stop the robot."""
    _cancel_explore_timer()
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
    remaining = None
    if _explore_deadline is not None and _explore_timer and not _explore_timer.done():
        remaining = round(
            max(0.0, _explore_deadline - asyncio.get_event_loop().time()) / 60, 1
        )
    return {
        "explore_node_running": running,
        "frontier_markers": (
            frontiers
            if frontiers is not None
            else "no marker traffic in 6 s"
            if running
            else None
        ),
        "auto_pause_remaining_min": remaining,
        "nav": status or "no recent status traffic",
        "robot": robot or "unknown (/pose silent)",
    }


if __name__ == "__main__":
    mcp.run(transport="http", host="0.0.0.0", port=9001, path="/mcp")
