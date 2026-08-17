"""Prometheus exporter for the Scout stack (http://<pi>:9100/metrics).

One process, three sources, all read-only:
  - docker-py against the host socket -> per-service CPU/mem/net/restarts
    for every container in this compose project (labels, not names, so it
    survives compose recreating containers).
  - rosbridge websocket (127.0.0.1:9090) -> measured Hz on a fixed topic
    list, exactly like a human would eyeball with `ros2 topic hz`.
  - `docker exec` into the `robot` container to run `ros2 topic info` for
    publisher/subscriber counts (DDS matching) — this is the one thing
    rosbridge's JSON protocol cannot answer, so it borrows the one
    container that already carries the full ROS/DDS environment instead of
    shipping a second one here.

Deliberately NOT scraping /var/run/docker.sock for host-level CPU/mem: this
container inherits network_mode: host like every other Scout service (see
docker-compose.yaml), so per-container network byte counters may read zero
under host networking (Docker only counts network stats for containers in
their own network namespace) — expect cpu/mem to be meaningful and rx/tx to
sometimes not be.
"""

import asyncio
import os
import re
import time
from concurrent.futures import ThreadPoolExecutor

import docker
from prometheus_client import Gauge, start_http_server
from rosbridge import RosBridge

COMPOSE_PROJECT = os.environ.get("COMPOSE_PROJECT", "scout")
# The Hz pass alone takes len(HZ_TOPICS) * HZ_WINDOW_S (sequential, one
# websocket at a time) -- default 7 topics * 2s = 14s, so POLL_S must clear
# that plus the docker/DDS passes or poll_hz starts overlapping itself.
POLL_S = float(os.environ.get("POLL_S", "20"))
HZ_WINDOW_S = float(os.environ.get("HZ_WINDOW_S", "2"))
METRICS_PORT = int(os.environ.get("METRICS_PORT", "9100"))

# topic -> rosbridge msg type, for the Hz sampler.
HZ_TOPICS = {
    "/scan": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/wheel_odom": "nav_msgs/msg/Odometry",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/cmd_vel_nav": "geometry_msgs/msg/Twist",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/imu/data": "sensor_msgs/msg/Imu",
}

# Extra topics to probe for DDS pub/sub counts via `ros2 topic info` (superset
# of HZ_TOPICS — /map and /goal_pose are low-rate/event-driven, not Hz-worthy).
DDS_TOPICS = sorted(set(HZ_TOPICS) | {"/map", "/goal_pose"})

# Sourced inside the `robot` container before every `ros2` exec — matches the
# &base environment in docker-compose.yaml so discovery-server introspection
# actually sees the graph (see scout/config/super_client.xml).
ROS_EXEC_PREFIX = (
    "source /opt/ros/humble/setup.bash && "
    "source /opt/overlay/install/setup.bash && "
    "export ROS_LOCALHOST_ONLY=1 ROS_DISCOVERY_SERVER=127.0.0.1:11811 "
    "ROS_SUPER_CLIENT=1 FASTDDS_DEFAULT_PROFILES_FILE=/ros_ws/src/scout/config/super_client.xml && "
)

docker_client = docker.from_env()

cpu_gauge = Gauge("scout_container_cpu_percent", "Container CPU %", ["service"])
mem_gauge = Gauge("scout_container_mem_bytes", "Container memory usage, bytes", ["service"])
mem_limit_gauge = Gauge("scout_container_mem_limit_bytes", "Container memory limit, bytes", ["service"])
net_rx_gauge = Gauge("scout_container_net_rx_bytes", "Container network rx, bytes", ["service"])
net_tx_gauge = Gauge("scout_container_net_tx_bytes", "Container network tx, bytes", ["service"])
restart_gauge = Gauge("scout_container_restart_count", "Docker restart count", ["service"])
up_gauge = Gauge("scout_container_up", "1 if the container is running", ["service"])

topic_hz_gauge = Gauge("scout_ros_topic_hz", "Measured message rate over the poll window", ["topic"])
topic_pub_gauge = Gauge("scout_ros_topic_publisher_count", "DDS matched publisher count", ["topic"])
topic_sub_gauge = Gauge("scout_ros_topic_subscriber_count", "DDS matched subscriber count", ["topic"])
dds_probe_ok_gauge = Gauge("scout_dds_probe_ok", "1 if the last `ros2 topic info` exec succeeded")


def _service_name(container) -> str:
    return container.labels.get("com.docker.compose.service", container.name)


def _cpu_percent(stats: dict) -> float:
    try:
        cpu = stats["cpu_stats"]
        precpu = stats["precpu_stats"]
        cpu_delta = cpu["cpu_usage"]["total_usage"] - precpu["cpu_usage"]["total_usage"]
        sys_delta = cpu.get("system_cpu_usage", 0) - precpu.get("system_cpu_usage", 0)
        if sys_delta <= 0 or cpu_delta < 0:
            return 0.0
        ncpu = cpu.get("online_cpus") or len(cpu["cpu_usage"].get("percpu_usage") or [1])
        return (cpu_delta / sys_delta) * ncpu * 100.0
    except (KeyError, ZeroDivisionError, TypeError):
        return 0.0


def poll_docker():
    for c in docker_client.containers.list(all=True):
        if c.labels.get("com.docker.compose.project") != COMPOSE_PROJECT:
            continue
        svc = _service_name(c)
        up_gauge.labels(svc).set(1 if c.status == "running" else 0)
        if c.status != "running":
            continue
        try:
            stats = c.stats(stream=False)
            cpu_gauge.labels(svc).set(_cpu_percent(stats))
            mem = stats.get("memory_stats", {})
            mem_gauge.labels(svc).set(mem.get("usage", 0))
            mem_limit_gauge.labels(svc).set(mem.get("limit", 0))
            nets = stats.get("networks") or {}
            net_rx_gauge.labels(svc).set(sum(n.get("rx_bytes", 0) for n in nets.values()))
            net_tx_gauge.labels(svc).set(sum(n.get("tx_bytes", 0) for n in nets.values()))
            c.reload()
            restart_gauge.labels(svc).set(c.attrs.get("RestartCount", 0))
        except docker.errors.APIError:
            pass


def _find_robot_container():
    for c in docker_client.containers.list():
        if c.labels.get("com.docker.compose.project") == COMPOSE_PROJECT and _service_name(c) == "robot":
            return c
    return None


def poll_dds():
    """`ros2 topic info` exec'd inside the running `robot` container — the
    only place in the stack with a live ROS/DDS environment. Absent/stopped
    `robot` just means the probe reports 0/ok=0; it never raises.
    """
    robot = _find_robot_container()
    if robot is None:
        dds_probe_ok_gauge.set(0)
        return
    ok = True
    for topic in DDS_TOPICS:
        try:
            rc, out = robot.exec_run(
                ["bash", "-lc", ROS_EXEC_PREFIX + f"ros2 topic info {topic} -v"],
                demux=False,
            )
            text = (out or b"").decode(errors="replace")
            if rc != 0:
                ok = False
                continue
            pub = re.search(r"Publisher count:\s*(\d+)", text)
            sub = re.search(r"Subscri(?:ber|ption) count:\s*(\d+)", text)
            topic_pub_gauge.labels(topic).set(int(pub.group(1)) if pub else 0)
            topic_sub_gauge.labels(topic).set(int(sub.group(1)) if sub else 0)
        except docker.errors.APIError:
            ok = False
    dds_probe_ok_gauge.set(1 if ok else 0)


async def _sample_hz(topic: str, msg_type: str) -> float:
    try:
        async with RosBridge() as rb:
            msgs = await rb.subscribe_collect(topic, msg_type, duration=HZ_WINDOW_S)
        return len(msgs) / HZ_WINDOW_S
    except (OSError, asyncio.TimeoutError):
        return 0.0


def poll_hz():
    async def _run():
        # Sequential on purpose: parallel subscribes over one process would
        # need one socket per topic anyway, and this whole pass already fits
        # inside POLL_S (7 topics x 2s << 10s default).
        for topic, msg_type in HZ_TOPICS.items():
            hz = await _sample_hz(topic, msg_type)
            topic_hz_gauge.labels(topic).set(hz)

    asyncio.run(_run())


def main():
    start_http_server(METRICS_PORT)
    pool = ThreadPoolExecutor(max_workers=3)
    while True:
        start = time.monotonic()
        pool.submit(poll_docker)
        pool.submit(poll_dds)
        pool.submit(poll_hz)
        elapsed = time.monotonic() - start
        time.sleep(max(0.0, POLL_S - elapsed))


if __name__ == "__main__":
    main()
