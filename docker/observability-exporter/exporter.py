"""Prometheus exporter for the Scout stack (http://<pi>:9100/metrics).

One process, four sources, all read-only:
  - docker-py against the host socket -> per-service CPU/mem/net/restarts
    plus cgroup throttling counters for every container in this compose
    project (labels, not names, so it survives compose recreating
    containers).
  - rosbridge websocket (127.0.0.1:9090) -> measured Hz on a fixed topic
    list, exactly like a human would eyeball with `ros2 topic hz`.
  - `docker exec` into a ROS-carrying container to run `ros2 topic info`
    for publisher/subscriber counts (DDS matching) — the one thing
    rosbridge's JSON protocol cannot answer. The exec target is
    ROS_EXEC_SERVICE (default foxglove_bridge: same image, same overlay
    volumes, same loopback DDS domain, but a 0.3-cpu cgroup that can only
    ever degrade the Foxglove UI — never the control path in `robot`,
    which is what this used to exec into). Falls back to `robot` if the
    preferred target is absent.
  - sysfs (/sys/class/thermal, /sys/devices/system/cpu/cpufreq) -> SoC
    temperature and per-core frequency, for catching thermal throttling —
    the mode where every service looks healthy but the whole Pi slows down.

Scheduling: one daemon thread per source, each pacing itself (single-flight
by construction — a slow pass delays only its own next pass, nothing ever
queues). Every pass is wrapped so an exception can't kill its loop, and each
source exports its own error counter, duration, and last-success timestamp
(`time() - scout_poll_last_success_timestamp_seconds` is the sample age), so
a dead source is visible instead of silently freezing its gauges — the old
ThreadPoolExecutor discarded futures and swallowed exactly those failures.

Deliberately NOT scraping /var/run/docker.sock for host-level CPU/mem: this
container inherits network_mode: host like every other Scout service (see
docker-compose.yaml), so per-container network byte counters may read zero
under host networking (Docker only counts network stats for containers in
their own network namespace) — expect cpu/mem to be meaningful and rx/tx to
sometimes not be.
"""

import asyncio
import glob
import os
import re
import threading
import time
from pathlib import Path

import requests
import websockets
from poll_loop import (
    SourceStats,
    build_dds_script,
    next_sleep,
    parse_dds_script_output,
    run_source_once,
)
from prometheus_client import Counter, Gauge, start_http_server
from rosbridge import RosBridge

import docker

COMPOSE_PROJECT = os.environ.get("COMPOSE_PROJECT", "scout")
POLL_S = float(os.environ.get("POLL_S", "20"))
# The Hz pass alone takes len(HZ_TOPICS) * HZ_WINDOW_S (sequential, one
# websocket at a time) -- 7 topics * 2s = 14s of rosbridge serialization per
# pass, so its cadence is independently tunable.
HZ_POLL_S = float(os.environ.get("HZ_POLL_S", str(POLL_S)))
HZ_WINDOW_S = float(os.environ.get("HZ_WINDOW_S", "2"))
# DDS wiring (pub/sub matching) changes on launch/restart, not continuously —
# probe it far slower than the vitals.
DDS_POLL_S = float(os.environ.get("DDS_POLL_S", "60"))
DDS_TOPIC_TIMEOUT_S = float(os.environ.get("DDS_TOPIC_TIMEOUT_S", "5"))
METRICS_PORT = int(os.environ.get("METRICS_PORT", "9100"))
# Which compose service carries the ROS environment the DDS probe execs into.
ROS_EXEC_SERVICE = os.environ.get("ROS_EXEC_SERVICE", "foxglove_bridge")

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

# Sourced inside the exec-target container before every `ros2` exec — matches
# the &base environment in docker-compose.yaml (simple discovery on loopback,
# ADR-0022; the discovery-server/SUPER_CLIENT era is over).
ROS_EXEC_PREFIX = (
    "source /opt/ros/humble/setup.bash && "
    "source /opt/overlay/install/setup.bash && "
    "export ROS_DOMAIN_ID=17 ROS_LOCALHOST_ONLY=1 && "
)

# timeout=15 bounds every blocking docker API call (stats/exec/reload) so a
# wedged dockerd can never hang a source thread forever.
docker_client = docker.from_env(timeout=15)

cpu_gauge = Gauge("scout_container_cpu_percent", "Container CPU %", ["service"])
mem_gauge = Gauge("scout_container_mem_bytes", "Container memory usage, bytes", ["service"])
mem_limit_gauge = Gauge("scout_container_mem_limit_bytes", "Container memory limit, bytes",
                        ["service"])
net_rx_gauge = Gauge("scout_container_net_rx_bytes", "Container network rx, bytes", ["service"])
net_tx_gauge = Gauge("scout_container_net_tx_bytes", "Container network tx, bytes", ["service"])
restart_gauge = Gauge("scout_container_restart_count", "Docker restart count", ["service"])
up_gauge = Gauge("scout_container_up", "1 if the container is running", ["service"])

# cgroup CPU throttling, straight out of the stats payload we already fetch.
# rate(throttled_periods)/rate(periods) on robot/nav2 during a stall is the
# direct test of the quota-starvation hypothesis.
throttle_periods_gauge = Gauge("scout_container_cpu_periods_total",
                               "cgroup cpu.stat nr_periods (cumulative)", ["service"])
throttled_periods_gauge = Gauge("scout_container_cpu_throttled_periods_total",
                                "cgroup cpu.stat nr_throttled (cumulative)", ["service"])
throttled_seconds_gauge = Gauge("scout_container_cpu_throttled_seconds_total",
                                "cgroup cpu.stat throttled time, seconds (cumulative)",
                                ["service"])

topic_hz_gauge = Gauge("scout_ros_topic_hz", "Measured message rate over the poll window",
                       ["topic"])
topic_pub_gauge = Gauge("scout_ros_topic_publisher_count", "DDS matched publisher count",
                        ["topic"])
topic_sub_gauge = Gauge("scout_ros_topic_subscriber_count", "DDS matched subscriber count",
                        ["topic"])
dds_probe_ok_gauge = Gauge("scout_dds_probe_ok", "1 if the last `ros2 topic info` exec succeeded")

host_temp_gauge = Gauge("scout_host_cpu_temp_celsius",
                        "SoC temperature (hottest thermal zone), degrees C")
host_freq_gauge = Gauge("scout_host_cpu_freq_khz",
                        "Per-core current frequency (scaling_cur_freq)", ["cpu"])

poll_errors_total = Counter("scout_poll_errors_total",
                            "Poll passes that raised, per source", ["source"])
poll_duration_gauge = Gauge("scout_poll_duration_seconds",
                            "Duration of the last poll pass, per source", ["source"])
poll_last_success_gauge = Gauge("scout_poll_last_success_timestamp_seconds",
                                "Unix time of the last successful poll pass, per source",
                                ["source"])

# Host vitals from sysfs (mounted read-only into every container — no host
# mount needed). The Pi 5 throttles at 85 C, so temp + scaling_cur_freq
# together catch thermal collapse: temp near 85 with freq dropping below
# 2400000 kHz = throttling. Both files are absent off-Pi, so the gauges just
# stay stale/zero there.
THERMAL_GLOB = "/sys/class/thermal/thermal_zone*/temp"
CPUFREQ_GLOB = "/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"


def poll_host():
    temps = []
    for path in glob.glob(THERMAL_GLOB):
        try:
            temps.append(int(Path(path).read_text().strip()) / 1000.0)
        except (OSError, ValueError):
            pass
    if temps:
        host_temp_gauge.set(max(temps))
    for path in glob.glob(CPUFREQ_GLOB):
        cpu = re.search(r"cpu(\d+)", path)
        try:
            khz = int(Path(path).read_text().strip())
        except (OSError, ValueError):
            continue
        if cpu:
            host_freq_gauge.labels(cpu.group(1)).set(khz)


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
            thr = stats.get("cpu_stats", {}).get("throttling_data") or {}
            throttle_periods_gauge.labels(svc).set(thr.get("periods", 0))
            throttled_periods_gauge.labels(svc).set(thr.get("throttled_periods", 0))
            throttled_seconds_gauge.labels(svc).set(thr.get("throttled_time", 0) / 1e9)
            c.reload()
            restart_gauge.labels(svc).set(c.attrs.get("RestartCount", 0))
        except (docker.errors.APIError, requests.exceptions.RequestException):
            # docker-py raises requests timeouts through — a slow dockerd or
            # one wedged container must not kill the whole sweep.
            pass


def _find_exec_container():
    """The container the DDS probe execs into: ROS_EXEC_SERVICE first, then
    `robot` as a fallback so the probe still works on deployments that strip
    the preferred target."""
    fallback = None
    for c in docker_client.containers.list():
        if c.labels.get("com.docker.compose.project") != COMPOSE_PROJECT:
            continue
        svc = _service_name(c)
        if svc == ROS_EXEC_SERVICE:
            return c
        if svc == "robot":
            fallback = c
    return fallback


def poll_dds():
    """One `docker exec` running one bash script that probes every topic —
    sources the ROS environment once instead of once per topic, bounds each
    probe with coreutils `timeout`, and bounds the whole exec too. Absent
    target just means the probe reports ok=0; it never raises."""
    target = _find_exec_container()
    if target is None:
        dds_probe_ok_gauge.set(0)
        return
    script = build_dds_script(ROS_EXEC_PREFIX, DDS_TOPICS, DDS_TOPIC_TIMEOUT_S)
    total_s = int(DDS_TOPIC_TIMEOUT_S * len(DDS_TOPICS) + 10)
    try:
        rc, out = target.exec_run(
            ["timeout", str(total_s), "bash", "-lc", script],
            demux=False,
        )
    except (docker.errors.APIError, requests.exceptions.RequestException):
        dds_probe_ok_gauge.set(0)
        return
    text = (out or b"").decode(errors="replace")
    counts = parse_dds_script_output(text, DDS_TOPICS)
    ok = rc == 0
    for topic, (pub, sub) in counts.items():
        if pub is None and sub is None:
            ok = False
            continue  # leave the gauges untouched rather than faking a zero
        topic_pub_gauge.labels(topic).set(pub or 0)
        topic_sub_gauge.labels(topic).set(sub or 0)
    dds_probe_ok_gauge.set(1 if ok else 0)


async def _sample_hz(topic: str, msg_type: str) -> float:
    try:
        async with RosBridge() as rb:
            msgs = await rb.subscribe_collect(topic, msg_type, duration=HZ_WINDOW_S)
        return len(msgs) / HZ_WINDOW_S
    except (OSError, asyncio.TimeoutError, websockets.WebSocketException):
        # WebSocketException is the verified leak: rosbridge dropping the
        # socket mid-window used to escape into a discarded future and freeze
        # scout_ros_topic_hz forever with no log.
        return 0.0


def poll_hz():
    async def _run():
        # Sequential on purpose: parallel subscribes over one process would
        # need one socket per topic anyway, and the loop paces itself — a
        # slow pass just delays its own next pass.
        for topic, msg_type in HZ_TOPICS.items():
            hz = await _sample_hz(topic, msg_type)
            topic_hz_gauge.labels(topic).set(hz)

    asyncio.run(_run())


def _source_loop(name: str, fn, period_s: float):
    stats = SourceStats()
    while True:
        start = time.monotonic()
        ok = run_source_once(fn, stats)
        if not ok:
            poll_errors_total.labels(name).inc()
        poll_duration_gauge.labels(name).set(stats.last_duration_s)
        if ok:
            poll_last_success_gauge.labels(name).set(time.time())
        time.sleep(next_sleep(period_s, time.monotonic() - start))


def main():
    start_http_server(METRICS_PORT)
    sources = [
        ("docker", poll_docker, POLL_S),
        ("dds", poll_dds, DDS_POLL_S),
        ("hz", poll_hz, HZ_POLL_S),
        ("host", poll_host, POLL_S),
    ]
    for name, fn, period in sources:
        threading.Thread(
            target=_source_loop, args=(name, fn, period), name=f"poll-{name}", daemon=True
        ).start()
    threading.Event().wait()


if __name__ == "__main__":
    main()
