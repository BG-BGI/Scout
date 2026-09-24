#!/usr/bin/env python3
"""perf_ros_probe.py — subscribe-only ROS-side probe for perf_capture.py.

Launched by perf_capture.py as a throwaway compose-run container off the
robot image (own cgroup, capped to 0.3 cpus via docker update — never inside
robot/nav2/slam quotas). Containers share the host kernel clock, so the
`ts_mono` written here is byte-comparable with host.csv's — no clock-sync
machinery. NEVER publishes anything; it observes the wire and writes
ros.jsonl at 1 Hz:

  scan/odom: inter-arrival count/mean/max over the window (arrival gaps at
             the subscriber — sensor starvation shows here first)
  cmd_vel:   max arrival gap (subscribe-only; idle silence is normal — only
             judge gaps during commanded motion, per the handoff caveat)
  tf:        map->odom and odom->base_link age (stale TF is the cliff/CM
             freshness-stop mechanism)
  nav_state: transitions verbatim; diagnostics: ERROR count per window

Standalone use (robot stack up):
  docker compose run --rm --name scout-perfprobe robot \
    python3 /ros_ws/src/scripts/perf_ros_probe.py --out /ros_ws/src/captures/perf/test
"""
import argparse
import json
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

ERROR_LEVEL = 2  # diagnostic_msgs ERROR (byte-typed constant, ADR-0014 note)


class Arrivals:
    def __init__(self):
        self.last_t = None
        self.gaps = []

    def hit(self, now):
        if self.last_t is not None:
            self.gaps.append(now - self.last_t)
        self.last_t = now

    def window(self, now):
        gaps, self.gaps = self.gaps, []
        open_gap = (now - self.last_t) if self.last_t is not None else None
        return {
            "count": len(gaps) + (1 if gaps or self.last_t else 0),
            "gap_mean_ms": round(sum(gaps) / len(gaps) * 1000, 1) if gaps else None,
            "gap_max_ms": round(max(gaps + ([open_gap] if open_gap else [])) * 1000, 1)
            if (gaps or open_gap) else None,
        }


class PerfProbe(Node):
    def __init__(self, out_path):
        super().__init__("perf_probe")
        self._out = open(out_path, "a")
        self._scan = Arrivals()
        self._odom = Arrivals()
        self._cmd = Arrivals()
        self._nav_state = None
        self._nav_transitions = []
        self._diag_errors = 0
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self, spin_thread=False)

        self.create_subscription(
            LaserScan, "/scan",
            lambda _m: self._scan.hit(time.monotonic()), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/odom",
            lambda _m: self._odom.hit(time.monotonic()), qos_profile_sensor_data)
        self.create_subscription(
            Twist, "/cmd_vel",
            lambda _m: self._cmd.hit(time.monotonic()), 10)
        self.create_subscription(String, "/nav_state", self._on_nav, 10)
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info("perf_probe up (subscribe-only) -> %s" % out_path)

    def _on_nav(self, msg):
        if msg.data != self._nav_state:
            self._nav_transitions.append(msg.data)
            self._nav_state = msg.data

    def _on_diag(self, msg):
        for s in msg.status:
            lvl = s.level[0] if isinstance(s.level, bytes) else int(s.level)
            if lvl == ERROR_LEVEL:
                self._diag_errors += 1

    def _tf_age(self, target, source):
        try:
            t = self._tf_buf.lookup_transform(target, source, rclpy.time.Time())
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            return round(time.time() - stamp, 3)
        except Exception:  # noqa: BLE001 — tf2 raises several lookup exception types; absence IS the datum
            return None

    def _tick(self):
        now = time.monotonic()
        row = {
            "ts_wall": round(time.time(), 3),
            "ts_mono": round(now, 3),
            "scan": self._scan.window(now),
            "odom": self._odom.window(now),
            "cmd_vel_gap_max_ms": self._cmd.window(now)["gap_max_ms"],
            "tf_map_odom_age_s": self._tf_age("map", "odom"),
            "tf_odom_base_age_s": self._tf_age("odom", "base_link"),
            "nav_state": self._nav_state,
            "nav_transitions": self._nav_transitions,
            "diag_errors": self._diag_errors,
        }
        self._nav_transitions = []
        self._diag_errors = 0
        self._out.write(json.dumps(row) + "\n")
        self._out.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="run directory for ros.jsonl")
    args = ap.parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = PerfProbe(out_dir / "ros.jsonl")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
