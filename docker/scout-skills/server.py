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
from contextlib import asynccontextmanager
from datetime import datetime, timezone

import httpx
import numpy as np
from detect import annotate, detect
from fastmcp import FastMCP
from fastmcp.exceptions import ToolError
from fastmcp.utilities.types import Image
from geometry import planar_yaw
from motion import (
    CMD_VEL,
    MAX_MOVE_M,
    MAX_ROTATE_RAD,
    TWIST_TYPE,
    _zero_twist,
    run_move,
    run_rotate,
)
from render import render_map
from robot_profile import load as _load_profile
from rosbridge import ADVERTISE_SETTLE_S, RosBridge, RosBridgeError
from tf import TfTree

import tags as tagdb

TAG_WATCH_PERIOD_S = 2.0


@asynccontextmanager
async def _lifespan(app):
    # Passive tag watcher, on by default: "the robot knows it's home when it
    # sees the doghouse tag" without anyone calling a tool. Dies with the
    # container; restart: unless-stopped brings it back.
    task = asyncio.create_task(_tag_watch_loop())
    try:
        yield
    finally:
        task.cancel()


mcp = FastMCP("scout-skills", lifespan=_lifespan)

# Fold ros-mcp's raw ROS primitives into this same endpoint (2026-08-18) so
# Magnus needs only one connector/one serverUrl instead of two. ros_mcp still
# runs as its own compose service on 127.0.0.1:9000 (host networking) — this
# is a live proxy mount, not a fork/vendor of its code, so its 3.1.0 pin and
# release cadence are untouched. No `prefix` -> tool names stay exactly what
# they were on port 9000 (ping_robots, publish_once, send_action_goal, ...),
# so nothing on the Magnus side has to be renamed. `mcp.mount` is sync and
# only registers the link; ros-mcp's HTTP endpoint just needs to be up by the
# time a client actually calls a proxied tool, not at import time.
from fastmcp import Client as _FastMcpClient  # noqa: E402

_ROS_MCP_URL = os.environ.get("ROS_MCP_URL", "http://127.0.0.1:9000/mcp")
mcp.mount(FastMCP.as_proxy(_FastMcpClient(_ROS_MCP_URL)))

# action_msgs/msg/GoalStatus code -> friendly name (robot_profile.yaml SSOT).
NAV_STATUS = dict(enumerate(_load_profile()["goal_status_names"]))

# twist_mux output — the honest "is anything driving?" wire (every source feeds it).
CMD_VEL_OUT = _load_profile()["topic_cmd_vel_out"]


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
        "yaw": round(planar_yaw(q["z"], q["w"]), 3),
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
                tr["x"], tr["y"], planar_yaw(q["z"], q["w"])
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
        # Mirror the route onto a plain topic for link_watchdog: action goals
        # are not observable on the wire, and the watchdog needs the poses to
        # re-dispatch after a link-loss pause.
        await rb.publish(
            "/route_poses",
            "geometry_msgs/msg/PoseArray",
            {
                "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
                "poses": [p["pose"] for p in poses],
            },
        )
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
    optionally looping the circuit — one call covers a whole patrol round. A
    single name that is a stored ROUTE (see list_waypoints.routes, e.g. the
    operator's "patrol" route) expands to that route's waypoints. loops 1–10,
    total ≤ 50 poses. Arrives in the last waypoint's saved orientation. Returns
    IMMEDIATELY; poll nav_status, stop with nav_cancel."""
    if not names:
        raise ToolError("names is empty")
    store = _load_waypoints()
    pts = store["waypoints"]
    if len(names) == 1 and names[0] in store.get("routes", {}):
        poses, label = [], {"route": names[0]}
        for it in store["routes"][names[0]]:
            if isinstance(it, str):
                if it not in pts:
                    raise ToolError(
                        f"route {names[0]!r} references missing waypoint {it!r}")
                poses.append(pts[it])
            else:
                poses.append(it)
    else:
        missing = [n for n in names if n not in pts]
        if missing:
            raise ToolError(
                f"unknown waypoints {missing} — have: {sorted(pts) or 'none'}")
        poses, label = [pts[n] for n in names], {"patrol": names}
    loops = min(max(int(loops), 1), 10)
    poses = poses * loops
    if len(poses) > 50:
        raise ToolError(f"{len(poses)} poses > 50 — fewer waypoints or loops")
    final_yaw = poses[-1]["yaw"]
    circuit = [[p["x"], p["y"]] for p in poses]
    if len(circuit) == 1:
        result = await _dispatch_goal(circuit[0][0], circuit[0][1], final_yaw)
    else:
        result = await _dispatch_through(circuit, final_yaw)
    return label | {"loops": loops} | result


# --- named waypoints ---------------------------------------------------------
#
# name → map-frame pose, persisted to the ./maps bind mount so they survive
# container rebuilds and sit beside the posegraphs they belong to. Waypoints
# are only meaningful on the map they were saved on — a remap invalidates them.

WAYPOINTS_PATH = os.environ.get("WAYPOINTS_PATH", "/maps/waypoints.json")
WAYPOINTS_VERSION = 2


def _load_waypoints() -> dict:
    """The v2 waypoint store (ADR-0011): {"version", "waypoints": {name: pose},
    "routes": {name: [names|inline poses]}}. Tolerates the legacy flat
    {name: pose} file. Schema is shared with scout.core.waypoints; the CODE is
    not (separate container), so this is a small hand copy kept in sync by the
    ADR-0011 contract + fixtures."""
    try:
        with open(WAYPOINTS_PATH) as f:
            data = json.load(f)
    except FileNotFoundError:
        data = {}
    if data.get("version") == WAYPOINTS_VERSION:
        data.setdefault("waypoints", {})
        data.setdefault("routes", {})
        return data
    # legacy flat {name: pose}
    return {
        "version": WAYPOINTS_VERSION,
        "waypoints": {k: v for k, v in data.items()
                      if isinstance(v, dict) and "x" in v},
        "routes": {},
    }


def _store_waypoints(store: dict) -> None:
    tmp = WAYPOINTS_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(store, f, indent=2, sort_keys=True)
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
    store = _load_waypoints()
    pts = store["waypoints"]
    pts[name] = robot | {
        "saved": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
        "source": "operator",
    }
    # Stamp the map the pose belongs to (ADR-0029). Absent = legacy = assume
    # the active map.
    active = tagdb.active_map_name()
    if active:
        pts[name]["map"] = active
    _store_waypoints(store)
    return {"saved": {name: pts[name]}, "waypoint_count": len(pts)}


@mcp.tool(annotations={"readOnlyHint": True})
async def list_waypoints() -> dict:
    """Named waypoints usable with go_to_waypoint (map-frame poses saved with
    save_waypoint), plus stored route names (usable with patrol). Only valid on
    the map they were saved on."""
    store = _load_waypoints()
    return {"waypoints": store["waypoints"], "count": len(store["waypoints"]),
            "routes": sorted(store["routes"])}


@mcp.tool
async def delete_waypoint(name: str) -> dict:
    """Delete a named waypoint."""
    store = _load_waypoints()
    pts = store["waypoints"]
    if name not in pts:
        raise ToolError(f"no waypoint {name!r} — have: {sorted(pts) or 'none'}")
    del pts[name]
    _store_waypoints(store)
    return {"deleted": name, "remaining": sorted(pts)}


@mcp.tool
async def go_to_waypoint(name: str) -> dict:
    """Send Scout driving to a named waypoint saved earlier with
    save_waypoint (arrives in its saved orientation). Same semantics as
    go_to: returns immediately, poll nav_status, stop with nav_cancel."""
    pts = _load_waypoints()["waypoints"]
    if name not in pts:
        raise ToolError(f"no waypoint {name!r} — have: {sorted(pts) or 'none'}")
    target = pts[name]
    wp_map, active = target.get("map"), tagdb.active_map_name()
    if wp_map and active and wp_map != active:
        raise ToolError(
            f"waypoint {name!r} belongs to map {wp_map!r} (active: {active!r})"
            " — switch maps first (switch_map / webui Site panel)"
        )
    result = await _dispatch_goal(target["x"], target["y"], target["yaw"])
    return {"waypoint": name} | result


# --- site maps (ADR-0029) ------------------------------------------------------
#
# A site holds multiple labeled maps (one per floor); site.json's active_map
# is the one slam/amcl runs on. In localization mode the grid can be swapped
# live through map_server's LoadMap; mapping modes bind the map at slam launch,
# so switching there restarts the slam container (~20 s, via fleet_status).

# The slam container's view of the same maps dir — LoadMap runs THERE, so the
# path must be its, not ours (same hardcode as the webui Save Map button).
SLAM_MAPS_DIR = "/ros_ws/src/sites/active/maps"


def _load_site() -> dict:
    """site.json normalized to {slam_mode, active_map, maps:{name: entry}}.
    Tolerates v1 (default_map + top-level map_start_pose) and v2 (ADR-0029)."""
    try:
        with open(tagdb.SITE_JSON) as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        data = {}
    if not isinstance(data, dict):
        data = {}
    site = {
        "slam_mode": data.get("slam_mode") or "auto",
        "active_map": data.get("active_map") or data.get("default_map"),
        "maps": data.get("maps") if isinstance(data.get("maps"), dict) else {},
    }
    if not site["maps"] and site["active_map"]:
        site["maps"] = {site["active_map"]: {
            "map_start_pose": data.get("map_start_pose") or [0.0, 0.0, 0.0]}}
    return site


@mcp.tool
async def switch_map(name: str) -> dict:
    """Switch the active site's map (e.g. to another floor). In localization
    mode the grid swaps live (~1 s) and the pose is re-seeded at the map's
    start pose — reseed via a registered AprilTag (or the webui) if the robot
    isn't there. In mapping modes this RESTARTS the slam container (~20 s of
    no /map and no map->odom). Refused mid-drive."""
    site = _load_site()
    if name not in site["maps"]:
        raise ToolError(
            f"no map {name!r} in the active site — have: "
            f"{sorted(site['maps']) or 'none'}"
        )
    if name == site["active_map"]:
        return {"active_map": name, "note": "already active"}
    await _require_motion_idle()

    mode = site["slam_mode"]
    out: dict = {"active_map": name, "previous": site["active_map"],
                 "slam_mode": mode}
    if mode == "localization":
        # Our view of the grid file; LoadMap gets the slam container's path.
        local = os.path.join(os.path.dirname(tagdb.SITE_JSON), "maps",
                             f"{name}.yaml")
        if not os.path.exists(local):
            raise ToolError(
                f"map {name!r} has no grid (.yaml/.pgm) — re-save it from a "
                "mapping session (webui Save Map) before localizing on it"
            )
        async with RosBridge() as rb:
            values = await rb.call_service(
                "/map_server/load_map",
                "nav2_msgs/srv/LoadMap",
                {"map_url": f"{SLAM_MAPS_DIR}/{name}.yaml"},
            )
            if values.get("result", 255) != 0:
                raise ToolError(
                    f"map_server LoadMap failed (result={values.get('result')})"
                )
            pose = (site["maps"][name].get("map_start_pose")
                    or [0.0, 0.0, 0.0])
            msg = _stamped_pose(float(pose[0]), float(pose[1]), float(pose[2]))
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.25
            cov[35] = math.radians(15.0) ** 2
            await rb.publish(
                "/initialpose",
                "geometry_msgs/msg/PoseWithCovarianceStamped",
                {"header": msg["header"],
                 "pose": {"pose": msg["pose"], "covariance": cov}},
            )
            # Re-arm the tag relocalizer so the next registered-tag sighting
            # refines the coarse start pose on the new map.
            try:
                await rb.call_service(
                    "/tag_relocalizer/reseed", "std_srvs/srv/Trigger")
            except RosBridgeError:
                pass
        out["switched"] = "live (map_server LoadMap)"
        out["note"] = ("pose seeded at the map's start pose — show the robot "
                       "a registered tag (or set /initialpose) to refine")
    else:
        out["switched"] = "slam restart pending (~20 s)"

    # Persist active_map — fleet_status owns site.json writes.
    try:
        async with httpx.AsyncClient(timeout=15.0) as http:
            resp = await http.post(
                f"{FLEET_STATUS_URL}/api/sites/active",
                json={"active_map": name},
            )
            resp.raise_for_status()
            if mode != "localization":
                for svc in ("slam", "behaviors"):
                    await http.post(
                        f"{FLEET_STATUS_URL}/api/containers/{svc}/restart")
    except httpx.HTTPError as e:
        raise ToolError(
            f"map loaded but active_map not persisted ({e!r}) — the next slam "
            "restart will revert; retry switch_map or set it in the webui"
        ) from e
    return out


# --- relative motion (bypasses Nav2) -----------------------------------------


async def _require_motion_idle():
    """Refuse to stream cmd_vel on top of another commander. The status topics
    only re-publish on transitions, so also sniff the mux output /cmd_vel_out —
    a live drive (Nav2, teleop, patrol) means steady non-zero output."""
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
            CMD_VEL_OUT, "geometry_msgs/msg/Twist", timeout=0.7
        )
    if tw is not None and (
        abs(tw["linear"]["x"]) > 1e-3 or abs(tw["angular"]["z"]) > 1e-3
    ):
        raise ToolError(
            "something is already driving (non-zero on the mux output) — "
            "nav_cancel / stop_all before relative motion"
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
    """ONE-SHOT visual inspection of the current camera frame (YOLO11n, 80
    COCO classes) with an annotated image — use when you need to SEE the
    scene or verify a single detection visually. NOT for counting, searching,
    or surveying a space: never loop rotate/move + detect_objects — that
    stop-and-shoot cadence is slow, ships a large image per call, and
    double/under-counts across frames. To count or enumerate objects, drive
    one smooth coverage pass (explore / patrol / go_through) and call
    world_query once. Each detection here gets a camera distance and, when
    depth + TF cooperate, a map-frame position usable with go_to. For objects
    outside the COCO label set, use camera_snapshot and read the frame
    visually."""
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
        # Collect, don't subscribe_once: /tf_static has multiple latched
        # publishers (camera internals + URDF chain) and one message is not
        # the whole tree — the cause of silently missing map positions.
        for m in await rb.subscribe_collect(
            "/tf_static", "tf2_msgs/msg/TFMessage", duration=1.0
        ):
            tree.add_message(m)
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


# --- World model (companion perception) --------------------------------------
#
# The companion runs continuous YOLO on the bridged D455 stream and maintains a
# map-frame object table (WORLDMODEL.md). It publishes /world/objects back over
# the zenoh bridge (Pi allowlist `subscribers` — the only inbound topic). This
# is the "no stop to look" path: query the live model instead of grabbing and
# reasoning over one frame per turn. Silent topic => companion stack absent, and
# the Pi runs fine without it (spec §0.7), so this degrades to an offline note
# rather than erroring.

WORLD_OBJECTS_TOPIC = "/world/objects"


@mcp.tool(annotations={"readOnlyHint": True})
async def whats_around_me() -> dict:
    """Objects Scout currently perceives around it, in the map frame — a live
    world-model kept continuously on the companion, so it needs no camera stop
    and no per-frame turn. LIVE VIEW ONLY (objects seen in the last ~5 s):
    good for "what's near me right now"; for counting or enumerating a space
    use world_query (persistent registry) after a coverage pass instead of
    polling this. Each object: class, map-frame xy (hand straight to go_to),
    height z, detection score, hit count, and seconds since last seen. If the
    companion perception stack is down the topic is silent and this reports
    offline (the Pi is unaffected)."""
    async with RosBridge() as rb:
        # Latched topic, but the detector republishes ~3 Hz, so a volatile
        # rosbridge sub catches the next message well inside the timeout.
        msg = await rb.subscribe_once(
            WORLD_OBJECTS_TOPIC, "std_msgs/msg/String", timeout=3.0
        )
    if msg is None:
        return {
            "status": "world-model offline (companion down or /world/objects "
            "not bridged)",
            "objects": [],
        }
    try:
        payload = json.loads(msg["data"])
    except (KeyError, ValueError) as e:
        raise ToolError(f"malformed /world/objects payload: {e}") from e
    objs = payload.get("objects", [])
    return {
        "frame": payload.get("frame", "map"),
        "stamp": payload.get("stamp"),
        "count": len(objs),
        "objects": objs,
    }


WORLD_REGISTRY_TOPIC = "/world/registry"


@mcp.tool(annotations={"readOnlyHint": True})
async def world_query(
    cls: str | None = None,
    min_score: float = 0.0,
    min_hits: int = 2,
) -> dict:
    """Query the PERSISTENT world-model registry — every object the companion
    has confirmed since its detector started, deduplicated in the map frame
    with stable ids, class votes across frames, and hit counts. THIS is how to
    count or enumerate objects: drive one smooth coverage pass (explore /
    patrol / go_through — no stops for pictures), then call this once with a
    class filter. Do NOT reassemble counts from detect_objects frames or
    whats_around_me polls. Filters: cls (e.g. "chair"), min_score, min_hits
    (raise to reject flaky tracks). Registry resets when the companion detector
    restarts."""
    async with RosBridge() as rb:
        msg = await rb.subscribe_once(
            WORLD_REGISTRY_TOPIC, "std_msgs/msg/String", timeout=3.0
        )
    if msg is None:
        return {
            "status": "world-model offline (companion down or /world/registry "
            "not bridged)",
            "count": 0,
            "objects": [],
        }
    try:
        payload = json.loads(msg["data"])
    except (KeyError, ValueError) as e:
        raise ToolError(f"malformed /world/registry payload: {e}") from e
    objs = [
        o
        for o in payload.get("objects", [])
        if (cls is None or o.get("cls") == cls)
        and o.get("score", 0) >= min_score
        and o.get("hits", 0) >= min_hits
    ]
    return {
        "frame": payload.get("frame", "map"),
        "stamp": payload.get("stamp"),
        "filter": {"cls": cls, "min_score": min_score, "min_hits": min_hits},
        "count": len(objs),
        "objects": objs,
    }


# --- RFID (Flipper Zero, ADR-0025) --------------------------------------------
#
# flipper_node (robot service) owns the Flipper's USB CLI and loops
# `rfid read` ONLY while a human has enabled scanning in the webui RFID panel
# (/flipper/rfid_enable). Reads land pose-stamped on /rfid/reads; the
# companion's rfid_recorder is the primary DB and republishes the deduped
# /rfid/registry back across the zenoh bridge.

RFID_STATUS_TOPIC = "/flipper/status"
RFID_READS_TOPIC = "/rfid/reads"
RFID_REGISTRY_TOPIC = "/rfid/registry"


@mcp.tool(annotations={"readOnlyHint": True})
async def wait_rfid_read(timeout_s: float = 30.0) -> dict:
    """Wait for the NEXT RFID card read from the Flipper Zero and return it
    (protocol, data_hex, map pose, stamp). Does NOT start scanning — the
    scan loop is enabled by a human in the webui RFID panel, never by tools;
    if scanning is disabled this fails immediately with instructions instead
    of waiting. Compose with go_to: drive to the spot, then call this and
    present the card/tag to the Flipper's back. Returns within timeout_s or
    reports that nothing was read."""
    timeout_s = max(1.0, min(timeout_s, 120.0))
    async with RosBridge() as rb:
        status_msg = await rb.subscribe_once(
            RFID_STATUS_TOPIC, "std_msgs/msg/String", timeout=3.0
        )
        if status_msg is None:
            raise ToolError(
                "flipper_node silent (/flipper/status) — robot service down "
                "or node not launched"
            )
        status = json.loads(status_msg["data"])
        if not status.get("connected"):
            raise ToolError("Flipper Zero not connected (USB)")
        if not status.get("rfid_enabled"):
            raise ToolError(
                "RFID scanning is disabled — enable it in the webui RFID "
                "panel first (manual gate, ADR-0025)"
            )
        # The latched depth-50 window replays PAST reads on subscribe; swallow
        # that backlog first, then wait for a read_id we have not seen.
        await rb.subscribe(RFID_READS_TOPIC, "std_msgs/msg/String")
        seen: set = set()
        deadline = asyncio.get_event_loop().time() + timeout_s
        settling = True
        while True:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                return {
                    "status": "no card read within %.0f s (scanning stays "
                    "enabled)" % timeout_s,
                    "read": None,
                }
            msg = await rb.recv_msg(
                RFID_READS_TOPIC, timeout=0.5 if settling else remaining
            )
            if msg is None:
                settling = False  # replay backlog drained; now block for new
                continue
            read = json.loads(msg["data"])
            if settling:
                seen.add(read.get("read_id"))
                continue
            if read.get("read_id") not in seen:
                return {"status": "read", "read": read}


@mcp.tool(annotations={"readOnlyHint": True})
async def list_rfid_tags() -> dict:
    """Every RFID tag Scout has ever read at this site, deduplicated by tag
    data with hit counts, last-seen time, and the map pose of the most recent
    localized read (hand pose straight to go_to to return to a tag). Served
    from the companion's persistent per-site DB via the latched
    /rfid/registry; empty with a note when the companion is offline or no
    reads exist yet."""
    async with RosBridge() as rb:
        msg = await rb.subscribe_once(
            RFID_REGISTRY_TOPIC, "std_msgs/msg/String", timeout=3.0
        )
    if msg is None:
        return {
            "status": "rfid registry offline (companion down, /rfid/registry "
            "not bridged, or no reads recorded yet)",
            "count": 0,
            "tags": [],
        }
    try:
        payload = json.loads(msg["data"])
    except (KeyError, ValueError) as e:
        raise ToolError(f"malformed /rfid/registry payload: {e}") from e
    tags = payload.get("tags", [])
    return {"count": len(tags), "tags": tags}


# --- NFC (Flipper Zero, ADR-0026) --------------------------------------------
#
# Mirror of the RFID tools for the Flipper's 13.56 MHz HF radio. flipper_node
# loops the `nfc`/`scanner` sub-shell ONLY while a human has enabled scanning in
# the webui NFC panel (/flipper/nfc_enable). One serial line => NFC and RFID
# scanning are mutually exclusive. Reads land pose-stamped on /nfc/reads (UID in
# `data_hex`); the companion's nfc_recorder is the primary DB and republishes
# the deduped /nfc/registry back across the zenoh bridge.

NFC_READS_TOPIC = "/nfc/reads"
NFC_REGISTRY_TOPIC = "/nfc/registry"


@mcp.tool(annotations={"readOnlyHint": True})
async def wait_nfc_read(timeout_s: float = 30.0) -> dict:
    """Wait for the NEXT NFC tag read from the Flipper Zero and return it
    (protocol/tech, data_hex UID, map pose, stamp). Does NOT start scanning —
    the scan loop is enabled by a human in the webui NFC panel, never by tools;
    if scanning is disabled this fails immediately with instructions instead
    of waiting. Compose with go_to: drive to the spot, then call this and
    present the tag to the Flipper's back. Returns within timeout_s or reports
    that nothing was read."""
    timeout_s = max(1.0, min(timeout_s, 120.0))
    async with RosBridge() as rb:
        status_msg = await rb.subscribe_once(
            RFID_STATUS_TOPIC, "std_msgs/msg/String", timeout=3.0
        )
        if status_msg is None:
            raise ToolError(
                "flipper_node silent (/flipper/status) — robot service down "
                "or node not launched"
            )
        status = json.loads(status_msg["data"])
        if not status.get("connected"):
            raise ToolError("Flipper Zero not connected (USB)")
        if not status.get("nfc_enabled"):
            raise ToolError(
                "NFC scanning is disabled — enable it in the webui NFC "
                "panel first (manual gate, ADR-0026)"
            )
        # The latched depth-50 window replays PAST reads on subscribe; swallow
        # that backlog first, then wait for a read_id we have not seen.
        await rb.subscribe(NFC_READS_TOPIC, "std_msgs/msg/String")
        seen: set = set()
        deadline = asyncio.get_event_loop().time() + timeout_s
        settling = True
        while True:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                return {
                    "status": "no tag read within %.0f s (scanning stays "
                    "enabled)" % timeout_s,
                    "read": None,
                }
            msg = await rb.recv_msg(
                NFC_READS_TOPIC, timeout=0.5 if settling else remaining
            )
            if msg is None:
                settling = False  # replay backlog drained; now block for new
                continue
            read = json.loads(msg["data"])
            if settling:
                seen.add(read.get("read_id"))
                continue
            if read.get("read_id") not in seen:
                return {"status": "read", "read": read}


@mcp.tool(annotations={"readOnlyHint": True})
async def list_nfc_tags() -> dict:
    """Every NFC tag Scout has ever read at this site, deduplicated by UID with
    hit counts, last-seen time, and the map pose of the most recent localized
    read (hand pose straight to go_to to return to a tag). Served from the
    companion's persistent per-site DB via the latched /nfc/registry; empty
    with a note when the companion is offline or no reads exist yet."""
    async with RosBridge() as rb:
        msg = await rb.subscribe_once(
            NFC_REGISTRY_TOPIC, "std_msgs/msg/String", timeout=3.0
        )
    if msg is None:
        return {
            "status": "nfc registry offline (companion down, /nfc/registry "
            "not bridged, or no reads recorded yet)",
            "count": 0,
            "tags": [],
        }
    try:
        payload = json.loads(msg["data"])
    except (KeyError, ValueError) as e:
        raise ToolError(f"malformed /nfc/registry payload: {e}") from e
    tags = payload.get("tags", [])
    return {"count": len(tags), "tags": tags}


# --- AprilTags ---------------------------------------------------------------
#
# Registry (sqlite, /maps/tags.db) + standoff geometry live in tags.py;
# detection itself is the apriltag_ros node (robot service) via /detections.
# A sighting of a registered tag refreshes a waypoint named after it at a
# 0.5 m standoff in front of the tag face — so go_to_waypoint("doghouse")
# is "go home", and the waypoint self-heals in whatever map frame is live.

_tag_watch_enabled = True
_tag_watch_last: dict = {"at": None, "seen": []}


DETECTIONS_TOPIC = "/detections"


async def _scan_tags(update_waypoints: bool = True):
    """apriltag_ros detections joined with the registry + TF geometry.
    Returns a result list, or None when /detections is silent (apriltag
    nodes or camera down). One node per FAMILY publishes here at 2 fps, so
    collect a window and merge — a single message is one family's view only.
    Cheap: no image transfer."""
    async with RosBridge() as rb:
        msgs = await rb.subscribe_collect(
            DETECTIONS_TOPIC,
            "apriltag_msgs/msg/AprilTagDetectionArray",
            duration=1.2,
        )
        if not msgs:
            return None
        merged: dict[tuple, dict] = {}
        for m in msgs:
            for d in m.get("detections", []):
                merged[(d["family"], d["id"])] = d
        dets = list(merged.values())
        tree = TfTree()
        child_frames: set[str] = set()
        if dets:
            # ⚠ /tf_static has MULTIPLE latched publishers (realsense's
            # camera-internal chain AND robot_state_publisher's URDF chain);
            # subscribe_once would take only the first replay and the tree
            # would dead-end between camera_link and base_link. Collect them.
            tf_msgs = await rb.subscribe_collect(
                "/tf_static", "tf2_msgs/msg/TFMessage", duration=1.0
            ) + await rb.subscribe_collect(
                "/tf", "tf2_msgs/msg/TFMessage", duration=0.8
            )
            for m in tf_msgs:
                tree.add_message(m)
                child_frames |= {t["child_frame_id"] for t in m["transforms"]}
            robot = await _robot_pose(rb)
        else:
            robot = None

    results = []
    for d in dets:
        family, tag_id = d["family"], d["id"]
        entry = tagdb.lookup(family, tag_id)
        out = {
            "family": family,
            "tag_id": tag_id,
            "center_px": [round(d["centre"]["x"], 1), round(d["centre"]["y"], 1)],
            "registered": entry is not None,
        }
        # apriltag_ros publishes the tag's TF child frame as "<family>:<id>"
        # (frame-name flavor varies by config) — find it by suffix + family.
        nf = tagdb.norm_family(family)
        frame = next(
            (
                f for f in child_frames
                if f.endswith(f":{tag_id}")
                and tagdb.norm_family(f.rsplit(":", 1)[0]) == nf
            ),
            None,
        )
        if frame:
            robot_xy = (robot["x"], robot["y"]) if robot else None
            out |= tagdb.map_geometry(tree, frame, robot_xy)
        if entry:
            out["name"] = entry["name"]
            if entry["role"]:
                out["role"] = entry["role"]
                if entry["role"] == "home":
                    out["home"] = True
            pose = out.get("standoff")
            active = tagdb.active_map_name()
            tagdb.record_sighting(
                family, tag_id,
                tuple(out["position_map"]) + (pose["yaw"],)
                if pose and "position_map" in out
                else None,
                map_name=active,
            )
            if update_waypoints and pose:
                store = _load_waypoints()
                store["waypoints"][entry["name"]] = pose | {
                    "saved": datetime.now(timezone.utc).strftime(
                        "%Y-%m-%d %H:%M UTC"
                    ),
                    "source": "tag",
                }
                if active:
                    store["waypoints"][entry["name"]]["map"] = active
                _store_waypoints(store)
                out["waypoint_refreshed"] = entry["name"]
        out["_box"] = [
            min(c["x"] for c in d["corners"]), min(c["y"] for c in d["corners"]),
            max(c["x"] for c in d["corners"]), max(c["y"] for c in d["corners"]),
        ]
        results.append(out)
    return results


async def _tag_watch_loop():
    global _tag_watch_last
    while True:
        await asyncio.sleep(TAG_WATCH_PERIOD_S)
        if not _tag_watch_enabled:
            continue
        try:
            results = await _scan_tags(update_waypoints=True)
        except Exception:  # noqa: BLE001 — apriltag node/rosbridge hiccup; retry next period
            continue
        if results is not None:
            _tag_watch_last = {
                "at": datetime.now(timezone.utc).strftime("%H:%M:%S UTC"),
                "seen": [
                    r.get("name", f'{r["family"]}:{r["tag_id"]}') for r in results
                ],
            }


@mcp.tool(annotations={"readOnlyHint": True})
async def detect_tags() -> list:
    """Current AprilTag detections (from the apriltag_ros node watching the
    camera). Registered tags come back with their name/role ("doghouse" is
    home), map position, and a standoff pose 0.5 m in front of the tag —
    each sighting also refreshes a waypoint named after the tag, so
    go_to_waypoint(name) drives there. Returns detection JSON, then the
    camera frame with tags boxed."""
    results = await _scan_tags(update_waypoints=True)
    if results is None:
        raise ToolError(
            f"nothing on {DETECTIONS_TOPIC} within 1.5 s — apriltag node or "
            "camera down (robot service up?)"
        )
    async with RosBridge() as rb:
        color = await rb.subscribe_once(
            COLOR_TOPIC, "sensor_msgs/msg/Image", timeout=4.0
        )
    rgb = _img_to_np(color) if color else np.zeros((8, 8, 3), np.uint8)
    boxes = [
        {
            "box": r.pop("_box"),
            "label": r.get("name", f'{r["family"]}:{r["tag_id"]}'),
            "confidence": 1.0,
            "distance_m": r.get("distance_m"),
        }
        for r in results
    ]
    png = await asyncio.to_thread(annotate, rgb, boxes)
    meta = {"tags": results, "count": len(results)}
    return [json.dumps(meta), Image(data=png, format="png").to_image_content()]


@mcp.tool
async def register_tag(
    name: str,
    tag_id: int,
    family: str = "tag36h11",
    role: str = "",
    size_m: float = 0.16,
) -> dict:
    """Register (or update) an AprilTag's MEANING: name it ("doghouse"), give
    it a role ("home" marks the robot's home), record its printed size.
    ⚠ Detection coverage is separate: the apriltag_ros node detects the
    family/size configured in scout/config/apriltag.yaml (robot-service
    restart to change) — registering here names tags that node can already
    see. A tag's surveyed pose is stamped with the map it was seen on
    (ADR-0029) — one surveyed pose per tag ID, so use a DISTINCT physical tag
    per floor/map."""
    if not (0.01 <= size_m <= 2.0):
        raise ToolError("size_m implausible — meters, black square edge only")
    return {"registered": tagdb.upsert(name, tag_id, family, role, size_m)}


@mcp.tool(annotations={"readOnlyHint": True})
async def list_tags() -> dict:
    """Registered AprilTags with last-seen info (incl. map_name — the map each
    tag's pose was surveyed on), plus the passive watcher state (scans every
    2 s and refreshes tag waypoints when the camera is up)."""
    return {
        "tags": tagdb.all_tags(),
        "watcher": {"enabled": _tag_watch_enabled, "last_scan": _tag_watch_last},
    }


@mcp.tool
async def delete_tag(name: str) -> dict:
    """Remove a tag from the registry (its waypoint, if any, stays until
    delete_waypoint)."""
    if not tagdb.delete(name):
        raise ToolError(f"no tag named {name!r}")
    return {"deleted": name}


@mcp.tool
async def tag_watch(enabled: bool) -> dict:
    """Turn the passive AprilTag watcher on/off (on by default). Off saves
    ~2-4% of a core and stops waypoint auto-refresh; detect_tags still works
    on demand."""
    global _tag_watch_enabled
    _tag_watch_enabled = enabled
    return {"watcher_enabled": enabled}


async def _cancel_all_nav(rb: RosBridge) -> tuple[dict, int]:
    """Zeroed CancelGoal (= cancel-all) on both bt_navigator actions. Returns
    ({action: return_code}, total goals canceling)."""
    codes: dict = {}
    canceling = 0
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
    return codes, canceling


@mcp.tool
async def nav_cancel() -> dict:
    """Cancel every active Nav2 goal (zeroed CancelGoal = cancel-all). ⚠ This
    is NOT the full e-stop — it stops only Nav2, leaving a running patrol
    driving. Use `stop_all` to halt everything. A goal
    survives its client dying, so this is the only way to clear a stray Nav2
    goal short of restarting nav2. Deceleration is a coast, not a brake
    (200 ms deadman, free-wheeling idle)."""
    async with RosBridge() as rb:
        codes, canceling = await _cancel_all_nav(rb)
        status = await _nav_status(rb, timeout=2.0)
    # return_code 0 = none active (nothing to cancel), which still means "not
    # driving" — report it as success with the detail visible.
    return {
        "return_codes": codes,
        "goals_canceling": canceling,
        "nav": status or "no status transition seen",
    }


@mcp.tool
async def stop_all() -> dict:
    """THE software e-stop: halt every motion source at once — the same thing
    the web UI STOP button does. Cancels Nav2 goals AND stops any running
    patrol (which keeps driving through a bare nav_cancel), then streams zero
    Twists so nothing stays latched past the 200 ms deadman. Safe to call even
    when idle; missing services are reported, not fatal."""
    stopped: dict = {}
    async with RosBridge() as rb:
        # Stop the higher-level driver first: a live patrol would re-issue
        # motion right through a nav cancel + zero burst.
        for name, svc in (
            ("patrol", "/patrol/stop"),
        ):
            try:
                values = await rb.call_service(svc, "std_srvs/srv/Trigger")
                stopped[name] = values.get("message", "ok")
            except RosBridgeError as exc:
                # Service absent (driver not running) or slow — not a failure
                # of the stop; keep going and stop the rest.
                stopped[name] = f"unavailable: {exc}"
        codes, canceling = await _cancel_all_nav(rb)
        # Explicit repeated zeros: a single lost frame would leave the last
        # command latched until the deadman coasts it out.
        await rb.advertise(CMD_VEL, TWIST_TYPE)
        await asyncio.sleep(ADVERTISE_SETTLE_S)
        for _ in range(3):
            await rb.publish_raw(CMD_VEL, _zero_twist())
            await asyncio.sleep(0.04)
    return {
        "stopped": stopped,
        "nav_return_codes": codes,
        "goals_canceling": canceling,
    }


@mcp.tool
async def estop(engaged: bool) -> dict:
    """Latching software e-stop (the twist_mux lock + active brake). engaged=true
    locks the robot out of ALL motion and actively brakes; nothing drives until
    engaged=false releases it. Unlike stop_all (a one-shot halt), this HOLDS
    until released — use it to make the robot safe to approach."""
    svc = "/estop/engage" if engaged else "/estop/release"
    async with RosBridge() as rb:
        values = await rb.call_service(svc, "std_srvs/srv/Trigger")
    return {
        "estop": "engaged" if engaged else "released",
        "message": values.get("message"),
    }


# --- collision-monitor bypass (collision_polygon_manager, ADR-0016 addenda) --
#
# Escape hatch for the direction-blind PolygonStop lockout: a plain polygon
# STOP zone zeroes cmd_vel in EVERY direction (even reverse) once tripped,
# with no reverse-to-escape path. Bounded — auto-releases ~30s after engage
# regardless of whether release is ever called. The same node also toggles
# a narrow/wide stop zone live based on commanded angular velocity (straight
# vs turning) — this tool only exposes the manual bypass, not that part.


@mcp.tool
async def collision_bypass(engaged: bool) -> dict:
    """Bounded bypass of the last-hop collision safety stage. engaged=true
    lets the robot drive out of a stuck PolygonStop lockout (auto-releases
    after ~30s on its own); engaged=false restores it immediately. Not a
    general safety disable — use only to escape a lockout, then release."""
    svc = ("/collision_monitor/bypass_engage" if engaged
           else "/collision_monitor/bypass_release")
    async with RosBridge() as rb:
        values = await rb.call_service(svc, "std_srvs/srv/Trigger")
    return {
        "bypass": "engaged" if engaged else "released",
        "message": values.get("message"),
    }


# --- rosbag record-on-demand (bag_recorder node, ADR-0017) -------------------


@mcp.tool
async def start_recording() -> dict:
    """Start a rosbag recording of the diagnosis topic set (odom chain, scan,
    cmd_vel, health) into captures/bags/<UTC>/ on the robot. Refuses if one is
    already running; auto-stops after the profile's record_max_duration_s.
    Returns the bag directory."""
    async with RosBridge() as rb:
        values = await rb.call_service("/record/start", "std_srvs/srv/Trigger")
    return {
        "started": values.get("success"),
        "bag": values.get("message"),
    }


@mcp.tool
async def stop_recording() -> dict:
    """Stop the running rosbag recording (clean SIGINT — the bag is finalized
    and playable). Returns the bag directory; refuses if nothing is recording."""
    async with RosBridge() as rb:
        values = await rb.call_service("/record/stop", "std_srvs/srv/Trigger")
    return {
        "stopped": values.get("success"),
        "bag": values.get("message"),
    }


@mcp.tool(annotations={"readOnlyHint": True})
async def recording_status() -> dict:
    """Whether a rosbag recording is running, and the current/last bag path
    (both latched by bag_recorder, so this answers even long after the fact)."""
    async with RosBridge() as rb:
        active = await rb.subscribe_once(
            "/record/active", "std_msgs/msg/Bool", timeout=2.0)
        path = await rb.subscribe_once(
            "/record/path", "std_msgs/msg/String", timeout=2.0)
    if active is None:
        return {"active": None, "detail": "bag_recorder not reachable"}
    return {
        "active": active.get("data"),
        "bag": (path or {}).get("data") or None,
    }


# --- frontier exploration (explore_lite, compose profile `explore`) ---------
#
# The container is profile-gated and pre-created (Created state, never
# started) by scout-switch at deploy time; `explore_start` below brings a
# Created/STOPPED container up through fleet_status's container API
# (http://127.0.0.1:9003, docker socket lives THERE, scoped to this compose
# project) — mounting the docker socket into this no-auth LAN MCP container
# directly would let anyone on the LAN root the Pi, so lifecycle goes through
# that narrower API instead. Pause/resume of a RUNNING explorer stays a ROS
# concern via its /explore/resume Bool subscription; rosapi (launched with
# rosbridge_websocket) tells us whether the node is up at all.

EXPLORE_RESUME_TOPIC = "/explore/resume"
# 9003 = fleet_status (9002 is observability_mcp — a former wrong default
# here silently broke explore_start).
FLEET_STATUS_URL = os.environ.get("FLEET_STATUS_URL", "http://127.0.0.1:9003")
# explore_lite takes a few seconds to boot + subscribe /explore/resume.
EXPLORE_NODE_WAIT_S = 25.0  # profile-exempt: a boot wait, not publish_hz

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
    except Exception:  # noqa: BLE001 — explore node already gone; nothing left to pause
        pass


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
                "explore node is not running — call explore_start first (it "
                "restarts the stopped container via fleet_status; if the "
                "container was never created, the operator must run "
                "`docker compose --profile explore up -d explore` on the Pi)."
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
async def explore_start() -> dict:
    """Start the explore container (explore_lite) — ⚠ the robot BEGINS
    AUTONOMOUS FRONTIER DRIVING within seconds of the node coming up; have
    your stop path ready (explore_pause + nav_cancel). No-op if the node is
    already running. The container is pre-created (never started) by
    scout-switch at deploy time; this starts it from Created/stopped via the
    fleet-status API. Prefer explore_for immediately after this to bound the
    run."""
    async with RosBridge() as rb:
        if await _explore_running(rb):
            return {"explore_node": "already running", "started": False}
    try:
        async with httpx.AsyncClient(timeout=15.0) as http:
            resp = await http.post(
                f"{FLEET_STATUS_URL}/api/containers/explore/start"
            )
    except httpx.HTTPError as e:
        raise ToolError(
            f"fleet_status API unreachable at {FLEET_STATUS_URL} ({e!r}) — "
            "is the fleet_status container up?"
        ) from e
    if resp.status_code == 404:
        raise ToolError(
            "explore container does not exist — scout-switch should have "
            "pre-created it (`docker compose --profile explore create "
            "explore` on the Pi recreates it without starting)."
        )
    if resp.status_code != 200:
        raise ToolError(
            f"fleet_status refused to start explore: HTTP {resp.status_code} "
            f"{resp.text[:200]}"
        )
    # Container is up; wait for explore_lite to boot and subscribe
    # /explore/resume so callers can immediately pause/budget it.
    deadline = asyncio.get_event_loop().time() + EXPLORE_NODE_WAIT_S
    while True:
        async with RosBridge() as rb:
            if await _explore_running(rb):
                robot = await _robot_pose(rb)
                return {
                    "explore_node": "running — robot is now driving itself",
                    "started": True,
                    "robot": robot or "unknown (/pose silent)",
                }
        if asyncio.get_event_loop().time() > deadline:
            raise ToolError(
                "explore container started but the node did not subscribe "
                f"{EXPLORE_RESUME_TOPIC} within {EXPLORE_NODE_WAIT_S:.0f} s — "
                "check `docker compose logs explore` on the Pi"
            )
        await asyncio.sleep(1.5)


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
