#!/usr/bin/env python3
"""Inspection MCAP recorder (companion; plan: confined-space inspection F1).

Records the bridged sensor topics to one .mcap per inspection run, reviewable
by drag-and-drop into Foxglove desktop (3D + image + map panels; layout in
companion/foxglove/inspection-review.json). Runs on the companion's LOCAL DDS
graph, so recording costs the Pi nothing.

Run boundaries, any of:
- /patrol_status (String 'state|len[|i/n]'): records while state is a moving
  one (not idle/plan) — a patrol_capture route IS an inspection pass.
- /explore/resume (Bool): true starts, false stops — explore_for(minutes)
  publishes exactly this pair around a frontier run.
- /inspection/start + /inspection/stop (std_srvs Trigger, LOCAL graph only):
  manual runs / testing.

⚠ The Humble split-bag ban (ros2/rosbag2#966, recorded for the Pi bag path)
does NOT apply here: it is a rosbag2 *playback* bug and Foxglove reads MCAP
directly, so long runs may split freely (--max-bag-duration below).

Post-run (F1b): best-effort `rtabmap-export` drops cloud.ply beside the .mcap
(per-site db tree mounted ro at /sites). Failure is logged, never fatal —
the .mcap is the primary artifact.

Sites (ADR-0023): runs land in /captures/inspection/<site>/<UTC>/, where
<site> is the name the /sites/active symlink points at — read fresh at each
run start, so a site switch needs no restart here (this service is still in
the switch's restart set only to cut an in-flight recording at the boundary).
No active symlink = the old flat layout, backward compatible.
"""
import os
import re
import shutil
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

RECORD_TOPICS = [
    "/camera/camera/color/image_raw/compressed",
    "/camera/camera/aligned_depth_to_color/image_raw/compressedDepth",
    "/camera/camera/color/camera_info",
    "/scan",
    "/odom",
    "/tf",
    "/tf_static",
    "/world/objects",
    "/world/registry",
    "/rtabmap/cloud_map",
]

OUT_ROOT = Path("/captures/inspection")
SITES_DIR = Path("/sites")
RTABMAP_DB = SITES_DIR / "active" / "rtabmap.db"
# Mirrors scout.core.sites.SITE_NAME_RE (shared contract, ADR-0011 style).
SITE_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
# patrol_status states that mean "not driving a route"
PATROL_IDLE_STATES = {"idle", "plan"}


def active_site() -> str:
    """Active site name, or '' (flat legacy layout). Read per run start."""
    try:
        name = os.path.basename(os.readlink(SITES_DIR / "active"))
    except OSError:
        return ""
    return name if SITE_NAME_RE.match(name) else ""


class InspectionRecorder(Node):
    def __init__(self):
        super().__init__("inspection_recorder")
        self.declare_parameter("split_duration_s", 600)  # foxglove-safe splits
        self.declare_parameter("stop_debounce_s", 5.0)   # both sources idle this long

        self._proc: subprocess.Popen | None = None
        self._run_dir: Path | None = None
        self._patrol_active = False
        self._explore_active = False
        self._manual_active = False
        self._idle_since: float | None = None

        self.create_subscription(String, "/patrol_status", self._on_patrol, 10)
        self.create_subscription(Bool, "/explore/resume", self._on_explore, 10)
        self.create_service(Trigger, "/inspection/start", self._srv_start)
        self.create_service(Trigger, "/inspection/stop", self._srv_stop)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            "inspection_recorder up — auto on patrol/explore, manual via "
            "/inspection/{start,stop}")

    # --- run-state inputs ---
    def _on_patrol(self, msg):
        state = msg.data.split("|", 1)[0]
        self._patrol_active = state not in PATROL_IDLE_STATES

    def _on_explore(self, msg):
        self._explore_active = bool(msg.data)

    def _srv_start(self, req, resp):
        self._manual_active = True
        resp.success = True
        resp.message = "manual inspection run armed"
        return resp

    def _srv_stop(self, req, resp):
        self._manual_active = False
        self._patrol_active = False
        self._explore_active = False
        resp.success = True
        resp.message = "manual stop; recorder will close the bag"
        return resp

    # --- supervisor ---
    def _tick(self):
        want = self._patrol_active or self._explore_active or self._manual_active
        running = self._proc is not None and self._proc.poll() is None

        if self._proc is not None and self._proc.poll() is not None:
            # recorder died on its own (disk full, bad topic) — surface it
            self.get_logger().error(
                f"ros2 bag record exited rc={self._proc.returncode} mid-run")
            self._finish_run()
            running = False

        if want and not running:
            self._start_run()
            self._idle_since = None
        elif not want and running:
            # debounce: patrol emits idle between waypoints only at abort, but
            # a brief flap on either source should not split the artifact
            now = time.monotonic()
            if self._idle_since is None:
                self._idle_since = now
            elif now - self._idle_since >= self.get_parameter(
                    "stop_debounce_s").value:
                self._stop_run()
                self._idle_since = None
        else:
            self._idle_since = None

    def _start_run(self):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        site = active_site()
        self._run_dir = OUT_ROOT / site / stamp if site else OUT_ROOT / stamp
        self._run_dir.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            "ros2", "bag", "record",
            "-s", "mcap",
            "-o", str(self._run_dir),
            "--max-bag-duration",
            str(self.get_parameter("split_duration_s").value),
        ] + RECORD_TOPICS
        self._proc = subprocess.Popen(cmd)
        self.get_logger().info(f"recording -> {self._run_dir}")

    def _stop_run(self):
        if self._proc is None:
            return
        self.get_logger().info("closing bag (SIGINT to ros2 bag record)")
        self._proc.send_signal(signal.SIGINT)
        try:
            self._proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self.get_logger().error("bag recorder hung on SIGINT — killed; "
                                    "last split may be unindexed")
        self._finish_run()

    def _finish_run(self):
        self._proc = None
        run_dir, self._run_dir = self._run_dir, None
        if run_dir is None:
            return
        self._export_ply(run_dir)

    def _export_ply(self, run_dir: Path):
        """F1b: as-built cloud beside the .mcap. Best-effort — needs the
        rtabmap service's db volume mounted and rtabmap-export in this image
        (both ship with ros-humble-rtabmap-ros)."""
        if shutil.which("rtabmap-export") is None:
            self.get_logger().warn("rtabmap-export not in image — no .ply")
            return
        if not RTABMAP_DB.exists():
            self.get_logger().warn(f"{RTABMAP_DB} absent — no .ply "
                                   "(sites tree not mounted, or fresh site "
                                   "with no rtabmap db yet?)")
            return
        try:
            r = subprocess.run(
                ["rtabmap-export", "--output", "cloud",
                 "--output_dir", str(run_dir), str(RTABMAP_DB)],
                capture_output=True, text=True, timeout=300)
            if r.returncode == 0:
                self.get_logger().info(f"cloud.ply exported to {run_dir}")
            else:
                self.get_logger().warn(
                    f"rtabmap-export rc={r.returncode}: {r.stderr[-400:]}")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("rtabmap-export timed out (300 s)")


def main():
    rclpy.init()
    node = InspectionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_run()  # never orphan a recording subprocess
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
