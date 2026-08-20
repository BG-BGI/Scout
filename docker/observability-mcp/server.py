"""Scout observability MCP endpoint (http://<pi>:9002/mcp, streamable-http).

Read-only introspection plus a narrow, explicitly-guarded restart action —
the "read + safe actions" tier: it can restart any compose service EXCEPT
`robot` (the one service that owns the RoboClaw UART/motors, per CLAUDE.md's
"never command motion without explicit confirmation" — restarting it is
indistinguishable from an unannounced motion-authority handoff, so it's
refused unconditionally rather than gated on a flag this server can't see).

Same standalone-image, LAN-trust-only, host-networking pattern as
ros_mcp/scout_skills: this talks to the docker socket (mounted rw, unlike
the read-only mount on observability_exporter) and to the rosbridge
websocket on 127.0.0.1:9090. No ROS/DDS underlay of its own — `ros2` CLI
calls run via `docker exec` into the already-running `robot` container,
which is the only place in the stack with a live ROS/DDS environment.
"""

import os
import re

import docker
from fastmcp import FastMCP
from fastmcp.exceptions import ToolError
from rosbridge import RosBridge

COMPOSE_PROJECT = os.environ.get("COMPOSE_PROJECT", "scout")

# The one hardware/motion service. Restart is refused unconditionally --
# see module docstring. Nothing else in the stack drives an actuator.
MOTION_SERVICES = {"robot"}

# Sourced before every `ros2` exec into `robot` -- must match &base's
# environment in docker-compose.yaml (ROS_DISCOVERY_SERVER) plus the
# super-client override (scout/config/super_client.xml) needed for a shell
# to see the DDS graph at all -- see CLAUDE.md's Discovery Server section.
ROS_EXEC_PREFIX = (
    "source /opt/ros/humble/setup.bash && "
    "source /opt/overlay/install/setup.bash && "
    "export ROS_DOMAIN_ID=17 ROS_DISCOVERY_SERVER=127.0.0.1:11811 "
    "ROS_SUPER_CLIENT=1 FASTDDS_DEFAULT_PROFILES_FILE=/ros_ws/src/scout/config/super_client.xml && "
)

# Known msg types for topics this stack publishes, so callers usually don't
# need to pass msg_type. Anything else needs it spelled out explicitly.
KNOWN_TOPIC_TYPES = {
    "/scan": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/wheel_odom": "nav_msgs/msg/Odometry",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/cmd_vel_nav": "geometry_msgs/msg/Twist",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/imu/data": "sensor_msgs/msg/Imu",
    "/map": "nav_msgs/msg/OccupancyGrid",
    "/goal_pose": "geometry_msgs/msg/PoseStamped",
}

docker_client = docker.from_env()
mcp = FastMCP("scout-observability")


def _service_name(container) -> str:
    return container.labels.get("com.docker.compose.service", container.name)


def _project_containers():
    return [
        c
        for c in docker_client.containers.list(all=True)
        if c.labels.get("com.docker.compose.project") == COMPOSE_PROJECT
    ]


def _find_service(service: str):
    for c in _project_containers():
        if _service_name(c) == service:
            return c
    raise ToolError(
        f"no container for service {service!r} in compose project {COMPOSE_PROJECT!r}"
    )


def _find_robot():
    try:
        c = _find_service("robot")
    except ToolError:
        raise
    if c.status != "running":
        raise ToolError("`robot` container is not running -- no ROS/DDS environment to exec into")
    return c


def _cpu_percent(stats: dict) -> float:
    try:
        cpu = stats["cpu_stats"]
        precpu = stats["precpu_stats"]
        cpu_delta = cpu["cpu_usage"]["total_usage"] - precpu["cpu_usage"]["total_usage"]
        sys_delta = cpu.get("system_cpu_usage", 0) - precpu.get("system_cpu_usage", 0)
        if sys_delta <= 0 or cpu_delta < 0:
            return 0.0
        ncpu = cpu.get("online_cpus") or len(cpu["cpu_usage"].get("percpu_usage") or [1])
        return round((cpu_delta / sys_delta) * ncpu * 100.0, 1)
    except (KeyError, ZeroDivisionError, TypeError):
        return 0.0


@mcp.tool
def list_containers() -> list[dict]:
    """Every container in this compose project: service name, status,
    restart count, image, and started-at. Start here -- it's the fastest way
    to see what's down or restart-looping before digging into any one
    service's logs or stats."""
    out = []
    for c in _project_containers():
        c.reload()
        state = c.attrs.get("State", {})
        out.append(
            {
                "service": _service_name(c),
                "status": c.status,
                "restart_count": c.attrs.get("RestartCount", 0),
                "image": c.image.tags[0] if c.image.tags else c.image.short_id,
                "started_at": state.get("StartedAt"),
                "exit_code": state.get("ExitCode"),
                "is_motion_service": _service_name(c) in MOTION_SERVICES,
            }
        )
    return out


@mcp.tool
def container_logs(
    service: str, lines: int = 200, grep: str | None = None
) -> list[str]:
    """Tail of a service's docker logs. `grep` (plain substring, case-
    insensitive) filters client-side after the tail, so pair a generous
    `lines` with `grep` rather than assuming the match is near the end."""
    c = _find_service(service)
    raw = c.logs(tail=lines, timestamps=True).decode(errors="replace")
    log_lines = raw.splitlines()
    if grep:
        needle = grep.lower()
        log_lines = [ln for ln in log_lines if needle in ln.lower()]
    return log_lines


@mcp.tool
def container_stats(service: str) -> dict:
    """Point-in-time CPU %, memory, and network counters for one service --
    the live-snapshot equivalent of `docker stats --no-stream`. For a trend
    over time, use the Prometheus/Grafana dashboard fed by
    observability_exporter instead of polling this tool in a loop."""
    c = _find_service(service)
    if c.status != "running":
        return {"service": service, "status": c.status}
    stats = c.stats(stream=False)
    mem = stats.get("memory_stats", {})
    nets = stats.get("networks") or {}
    return {
        "service": service,
        "status": c.status,
        "cpu_percent": _cpu_percent(stats),
        "mem_usage_bytes": mem.get("usage", 0),
        "mem_limit_bytes": mem.get("limit", 0),
        "net_rx_bytes": sum(n.get("rx_bytes", 0) for n in nets.values()),
        "net_tx_bytes": sum(n.get("tx_bytes", 0) for n in nets.values()),
        # network_mode: host on every Scout service means these may read 0 --
        # Docker only counts per-container network stats in a container's own
        # netns. Not a bug in this tool; see docker-compose.yaml.
    }


@mcp.tool
async def ros2_topic_hz(topic: str, msg_type: str | None = None, window_s: float = 2.0) -> dict:
    """Measured publish rate on `topic` over a live `window_s`-second sample
    via rosbridge -- the MCP equivalent of running `ros2 topic hz` by hand.
    `msg_type` is required for topics not in this server's known list (see
    KNOWN_TOPIC_TYPES); rosbridge needs the type to deserialize the wire
    format, it can't discover it from the topic name alone."""
    msg_type = msg_type or KNOWN_TOPIC_TYPES.get(topic)
    if not msg_type:
        raise ToolError(
            f"unknown msg type for {topic!r} -- pass msg_type explicitly "
            f"(e.g. 'geometry_msgs/msg/Twist')"
        )
    window_s = max(0.5, min(window_s, 10.0))
    async with RosBridge() as rb:
        msgs = await rb.subscribe_collect(topic, msg_type, duration=window_s)
    return {
        "topic": topic,
        "msg_type": msg_type,
        "window_s": window_s,
        "message_count": len(msgs),
        "hz": round(len(msgs) / window_s, 2),
    }


@mcp.tool
def ros2_topic_info(topic: str) -> str:
    """Raw `ros2 topic info -v` for `topic`, exec'd inside the `robot`
    container -- publisher/subscriber counts AND their QoS profiles (reuse/
    durability/reliability), which is what actually catches a QoS mismatch
    (e.g. a subscriber demanding RELIABLE against a BEST_EFFORT publisher --
    those two just never exchange data, with no error on either side)."""
    robot = _find_robot()
    rc, out = robot.exec_run(["bash", "-lc", ROS_EXEC_PREFIX + f"ros2 topic info {topic} -v"])
    text = (out or b"").decode(errors="replace")
    if rc != 0:
        raise ToolError(f"ros2 topic info {topic} exited {rc}: {text}")
    return text


@mcp.tool
def ros2_node_list() -> list[str]:
    """`ros2 node list` exec'd inside the `robot` container -- every DDS
    participant currently visible to a super-client (see
    scout/config/super_client.xml). A node missing here that should be up is
    a discovery problem, not necessarily a dead process."""
    robot = _find_robot()
    rc, out = robot.exec_run(["bash", "-lc", ROS_EXEC_PREFIX + "ros2 node list"])
    text = (out or b"").decode(errors="replace")
    if rc != 0:
        raise ToolError(f"ros2 node list exited {rc}: {text}")
    return [ln for ln in text.splitlines() if ln.strip()]


@mcp.tool
def restart_container(service: str) -> dict:
    """Restart one compose service by name. REFUSES `robot` unconditionally
    -- that container owns the RoboClaw UART and every motor, and restarting
    it is a motion-authority action this server will never take on its own
    (see CLAUDE.md: never command motion without explicit operator
    confirmation). Safe for everything else (slam, nav2, foxglove_bridge,
    rosbridge, webui, explore, discovery, ros_mcp, scout_skills,
    observability_exporter/_mcp) -- e.g. `docker compose restart nav2` is
    this repo's own documented way to clear a latched Nav2 goal."""
    if service in MOTION_SERVICES:
        raise ToolError(
            f"refusing to restart {service!r}: it owns the drivetrain/motors. "
            "Ask the operator to run `docker compose restart robot` by hand."
        )
    c = _find_service(service)
    c.restart(timeout=10)
    c.reload()
    return {"service": service, "status": c.status}


if __name__ == "__main__":
    mcp.run(transport="streamable-http", host="0.0.0.0", port=9002)
