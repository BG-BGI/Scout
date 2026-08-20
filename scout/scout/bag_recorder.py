#!/usr/bin/env python3
"""rosbag2 record-on-demand: /record/start + /record/stop (ADR-0017).

All bench/calibration tooling was deleted (2026-07-30); rosbag2 is the
ROS-native permanent replacement for "rebuild the instrument". This node owns
the `ros2 bag record` subprocess lifecycle so any surface — skills MCP, webui,
a shell — can capture a bag with one Trigger call:

  /record/start   spawn `ros2 bag record` into captures/bags/<UTC>/
  /record/stop    clean SIGINT (rosbag2 finalizes the bag on SIGINT)
  /record/active  latched Bool — a late webui still shows the REC state
  /record/path    latched String — last (or current) bag directory

A subprocess, not rosbag2_py in-process: the Python API has documented
threading caveats under a spinning executor, and the CLI is the tested path.

Topic selection: the `topics` parameter defaults to the profile's
record_topics; a non-default set (e.g. + the depth cloud for the 5b bag) is
`ros2 param set /bag_recorder topics "[...]"` before /record/start — read at
spawn time, so a running recording is unaffected.

Runaway guard: --max-bag-size/--max-bag-duration are banned (split bags do
not play back on Humble — see scout.core.recording), so a forgotten
recording would fill the SD card. The node SIGINTs the child itself after
record_max_duration_s (profile; raise it for deliberate long captures).

QoS: without the overrides file a reliable-by-default recorder subscription
receives NOTHING from best-effort publishers (/imu/data — the same silent
trap as the EKF QoS note) — bag_qos_overrides.yaml rides every recording.

Discovery history (ADR-0022): under the retired Discovery Server the record
subprocess had to be spawned as a SUPER CLIENT or it captured zero messages
into a valid-looking bag (Discovery Server v2 blinds plain clients to graph
enumeration, which `ros2 bag record` needs for type resolution). Under simple
discovery the child sees the full graph natively, so no profile injection —
if a Discovery Server ever returns, that trap returns with it.
"""

import os
import signal
import subprocess
from datetime import datetime, timezone

from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from scout.core import recording
from scout.node_util import run_node
from scout.qos import LATCHED_QOS
from scout.robot_profile import load as _load_profile
from scout.robot_profile import resolve_config


class BagRecorder(Node):
    """Own the `ros2 bag record` subprocess: spawn, watch, SIGINT."""

    def __init__(self):
        super().__init__('bag_recorder')
        prof = _load_profile()
        self._max_duration = float(prof['record_max_duration_s'])

        p = self.declare_parameter
        # Same repo-root bind convention as patrol_capture's capture_dir;
        # bags get their own subtree so captures/<runstamp>/ stays the
        # patrol-photo namespace (CONTEXT.md).
        self._root = str(p('capture_dir', '/ros_ws/src/captures/bags').value)
        self._topics_param = p('topics', list(prof['record_topics']))
        self._qos_overrides = resolve_config('bag_qos_overrides.yaml')

        self._child = None
        self._started_at = None
        self._bag_path = ''

        self._active_pub = self.create_publisher(
            Bool, 'record/active', LATCHED_QOS)
        self._path_pub = self.create_publisher(
            String, 'record/path', LATCHED_QOS)
        self.create_service(Trigger, 'record/start', self._on_start)
        self.create_service(Trigger, 'record/stop', self._on_stop)
        self.create_timer(1.0, self._watch)
        self._publish_state(False)
        self.get_logger().info(
            'bag_recorder up: bags under %s, auto-stop %.0f s'
            % (self._root, self._max_duration))

    # --- services --------------------------------------------------------------

    def _on_start(self, req, resp):
        if self._recording():
            resp.success = False
            resp.message = 'already recording: %s' % self._bag_path
            return resp
        try:
            topics = recording.resolve_topics(
                self.get_parameter('topics').value)
            out_dir = recording.bag_dir(
                datetime.now(timezone.utc), self._root)
            # Parent only — rosbag2 creates the leaf and errors if it exists.
            os.makedirs(self._root, exist_ok=True)
            argv = recording.record_argv(topics, out_dir, self._qos_overrides)
            # Simple discovery (ADR-0022): the child enumerates the graph
            # natively — no SUPER_CLIENT profile injection (see module docstring
            # for the Discovery Server-era trap this replaced).
            self._child = subprocess.Popen(argv)
        except (ValueError, OSError) as exc:
            resp.success = False
            resp.message = 'record start failed: %s' % exc
            self.get_logger().error(resp.message)
            return resp
        self._started_at = self.get_clock().now()
        self._bag_path = out_dir
        self._publish_state(True)
        self.get_logger().info(
            'recording %d topics -> %s' % (len(topics), out_dir))
        resp.success = True
        resp.message = out_dir
        return resp

    def _on_stop(self, req, resp):
        if not self._recording():
            resp.success = False
            resp.message = 'not recording'
            return resp
        self._child.send_signal(signal.SIGINT)
        self.get_logger().info('stop requested — SIGINT sent (bag: %s)'
                               % self._bag_path)
        resp.success = True
        resp.message = self._bag_path
        return resp

    # --- child watch -----------------------------------------------------------

    def _recording(self):
        return self._child is not None and self._child.poll() is None

    def _watch(self):
        if self._child is None:
            return
        rc = self._child.poll()
        if rc is not None:
            # Reap: covers /record/stop, the auto-stop below, and the child
            # dying on its own — active goes False in all three.
            level = self.get_logger().info if rc in (0, -signal.SIGINT) \
                else self.get_logger().error
            level('recorder exited rc=%s: %s' % (rc, self._bag_path))
            self._child = None
            self._started_at = None
            self._publish_state(False)
            return
        elapsed = (self.get_clock().now()
                   - self._started_at).nanoseconds * 1e-9
        if elapsed > self._max_duration:
            self.get_logger().warn(
                'auto-stop after %.0f s (record_max_duration_s) — raise the '
                'profile value for deliberate long captures' % elapsed)
            self._child.send_signal(signal.SIGINT)

    def _publish_state(self, active):
        self._active_pub.publish(Bool(data=active))
        self._path_pub.publish(String(data=self._bag_path))

    def shutdown(self):
        """SIGINT + reap the child so the bag is finalized, not truncated."""
        if self._recording():
            self._child.send_signal(signal.SIGINT)
            try:
                self._child.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self._child.kill()


def main(args=None):
    run_node(BagRecorder, on_shutdown=BagRecorder.shutdown, args=args)


if __name__ == '__main__':
    main()
