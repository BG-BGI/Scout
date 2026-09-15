#!/usr/bin/env python3
"""M7E Hecto UHF reader bridge: enable-gated continuous EPC Gen2 inventory.

Design (ADR-0032, flipper_node conventions):
  * The node owns the single UhfSerial handle; the poll timer is the SOLE
    serial reader/writer. The enable service only mutates target state — the
    start/stop commands go out on the next tick.
  * Scanning is OFF at boot and only ever turned on by a human via /uhf/enable
    (std_srvs/SetBool — the webui UHF panel). A serial fault drops the flag:
    after an unplug the operator re-enables deliberately.
  * Reads are BATCHED (up to 150 tags/s from the module): tag records
    accumulate per batch_period_s window and go out as ONE latched JSON
    message on /uhf/reads with ONE map pose for the window (lookup_pose2 at
    flush — null when unlocalized), where the zenoh bridge carries it to the
    companion uhf_recorder (the primary DB). NO per-EPC dedup here — every
    read is geometry for the recorder's centroid (and stage-2 phase) solver.
  * The module keepalives once per second while scanning; keepalive silence
    beyond keepalive_timeout_s means a wedged module -> fault. Temp-throttle /
    high-return-loss keepalives set a sticky `throttled` status flag (cleared
    on the next enable) — scanning continues, health WARNs.
  * Reader absent is normal (tier 2): the node idles in DISCONNECTED with a
    throttled warn and keeps retrying; the robot stays fully drivable. The
    version handshake also rejects a mispinned port (a lidar never answers
    with a Mercury frame) without touching the device further.
"""

import time
import uuid
from datetime import datetime, timezone

import tf2_ros
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from scout.core import uhf
from scout.core.status import format_uhf_read_batch, format_uhf_status
from scout.node_util import lookup_pose2, run_node
from scout.qos import LATCHED_HISTORY_QOS, LATCHED_QOS
from scout.uhf_serial import UhfSerial

DISCONNECTED = 'disconnected'
IDLE = 'idle'
SCANNING = 'scanning'


class UhfNode(Node):
    """Owns the M7E serial port; continuous read gated by /uhf/enable."""

    def __init__(self):
        super().__init__('uhf_node')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('poll_hz', 30.0)
        self.declare_parameter('reconnect_period_s', 5.0)
        self.declare_parameter('handshake_timeout_s', 2.0)
        self.declare_parameter('batch_period_s', 0.1)
        self.declare_parameter('read_power_cdbm', uhf.READ_POWER_MAX_CDBM)
        self.declare_parameter('keepalive_timeout_s', 3.0)

        p = self.get_parameter
        self._ser = UhfSerial(str(p('port').value), int(p('baud').value))
        self._reconnect_period = float(p('reconnect_period_s').value)
        self._handshake_timeout = float(p('handshake_timeout_s').value)
        self._read_power = int(p('read_power_cdbm').value)
        self._keepalive_timeout = float(p('keepalive_timeout_s').value)

        self._state = DISCONNECTED
        self._enabled = False
        self._throttled = False
        self._last_error = ''
        self._acc = uhf.FrameAccumulator()
        self._pending = []             # parsed tag records awaiting a batch
        self._last_connect_attempt = 0.0
        self._keepalive_deadline = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._status_pub = self.create_publisher(String, 'uhf/status',
                                                 LATCHED_QOS)
        self._reads_pub = self.create_publisher(String, 'uhf/reads',
                                                LATCHED_HISTORY_QOS)
        self.create_service(SetBool, 'uhf/enable', self._on_enable)
        self.create_timer(1.0 / float(p('poll_hz').value), self._tick)
        self.create_timer(float(p('batch_period_s').value), self._flush_batch)

        self._publish_status()
        self.get_logger().info(
            'uhf_node up on %s: scanning DISABLED until /uhf/enable '
            '(webui UHF panel); read power capped at %d cdBm (ADR-0032)'
            % (str(p('port').value),
               min(self._read_power, uhf.READ_POWER_MAX_CDBM)))

    # --- status ---------------------------------------------------------------
    def _publish_status(self):
        self._status_pub.publish(String(data=format_uhf_status(
            self._state, self._ser.connected, self._enabled,
            self._throttled, self._last_error)))

    def _set_state(self, state, error=None):
        if error is not None:
            self._last_error = error
        if state != self._state or error is not None:
            self._state = state
            self._publish_status()

    def _fault(self, exc, where):
        self.get_logger().warn('serial fault (%s): %s — disconnected, '
                               'scanning disabled until re-enabled'
                               % (where, exc))
        try:
            self._ser.close()
        except OSError:
            pass
        self._enabled = False
        self._pending = []
        self._acc = uhf.FrameAccumulator()
        self._set_state(DISCONNECTED, error='%s: %s' % (where, exc))

    # --- service (mutates target state only) ------------------------------------
    def _on_enable(self, request, response):
        if request.data:
            if not self._ser.connected:
                response.success = False
                response.message = 'uhf reader not connected'
                return response
            self._enabled = True
            self._throttled = False       # a new session clears the sticky flag
        else:
            self._enabled = False
        self._publish_status()
        response.success = True
        response.message = ('UHF scanning enabled' if self._enabled
                            else 'UHF scanning disabled')
        self.get_logger().info(response.message)
        return response

    # --- the poll timer: sole serial reader/writer ------------------------------
    def _tick(self):
        try:
            if self._state == DISCONNECTED:
                self._tick_disconnected()
            elif self._state == IDLE:
                self._tick_idle()
            elif self._state == SCANNING:
                self._tick_scanning()
        except OSError as exc:            # covers serial.SerialException
            self._fault(exc, self._state)

    def _command(self, frame, name):
        """Blocking send-and-wait for the matching response (handshake only —
        never called while scanning). Returns the response frame or None."""
        self._ser.write(frame)
        acc = uhf.FrameAccumulator()
        deadline = time.monotonic() + self._handshake_timeout
        while time.monotonic() < deadline:
            for resp in acc.feed(self._ser.read_available()):
                if resp[2] == frame[2]:
                    if uhf.frame_status(resp) != 0:
                        self.get_logger().warn(
                            '%s failed: status 0x%04X'
                            % (name, uhf.frame_status(resp)))
                        return None
                    return resp
            time.sleep(0.02)
        return None

    def _tick_disconnected(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_period:
            return
        self._last_connect_attempt = now
        try:
            self._ser.open()
        except OSError as exc:
            self.get_logger().warn(
                'uhf reader not connected (%s) — UHF unavailable, retrying'
                % exc, throttle_duration_sec=30.0)
            return
        # Version probe doubles as the wrong-port guard: a mispinned lidar
        # answers nothing Mercury-framed, so we close and retry — never send
        # config writes to a device we have not identified.
        if self._command(uhf.cmd_version(), 'version') is None:
            self.get_logger().warn(
                'port opened but no Mercury version response — not an M7E? '
                '(check the host udev pin, ADR-0032)',
                throttle_duration_sec=30.0)
            self._ser.close()
            return
        for frame, name in (
                (uhf.cmd_set_region(), 'set region'),
                (uhf.cmd_set_tag_protocol(), 'set protocol'),
                (uhf.cmd_set_antenna_port(), 'set antenna'),
                (uhf.cmd_disable_read_filter(), 'disable read filter'),
                (uhf.cmd_set_read_power(self._read_power), 'set read power')):
            if self._command(frame, name) is None:
                self._ser.close()
                return
        self.get_logger().info('uhf reader connected and configured')
        self._acc = uhf.FrameAccumulator()
        self._set_state(IDLE, error='')

    def _tick_idle(self):
        self._ser.read_available()        # keep the buffer drained
        if not self._enabled:
            return
        self._ser.write(uhf.cmd_start_continuous())
        self._keepalive_deadline = time.monotonic() + self._keepalive_timeout
        self._set_state(SCANNING)

    def _tick_scanning(self):
        if not self._enabled:
            self._ser.write(uhf.cmd_stop_continuous())
            time.sleep(0.05)
            self._ser.read_available()    # drop the in-flight tail
            self._acc = uhf.FrameAccumulator()
            self._flush_batch()           # don't strand a partial window
            self._set_state(IDLE)
            return
        now = time.monotonic()
        for frame in self._acc.feed(self._ser.read_available()):
            kind = uhf.classify_frame(frame)
            if kind == 'tag':
                rec = uhf.parse_tag_record(frame)
                rec['read_id'] = str(uuid.uuid4())
                self._pending.append(rec)
                self._keepalive_deadline = now + self._keepalive_timeout
            elif kind == 'keepalive':
                self._keepalive_deadline = now + self._keepalive_timeout
            elif kind in ('temp_throttle', 'high_return_loss'):
                self._keepalive_deadline = now + self._keepalive_timeout
                if not self._throttled:
                    self._throttled = True
                    self.get_logger().warn('uhf module reports %s — scanning '
                                           'continues degraded' % kind)
                    self._publish_status()
        if now > self._keepalive_deadline:
            self._fault('no keepalive for %.1f s (module wedged?)'
                        % self._keepalive_timeout, 'scanning')

    # --- batch flush timer ------------------------------------------------------
    def _flush_batch(self):
        if not self._pending:
            return
        reads, self._pending = self._pending, []
        pose = lookup_pose2(self._tf_buffer, 'map', 'base_link')
        if pose is None:
            self.get_logger().warn('uhf reads without map localization — pose '
                                   'recorded as null',
                                   throttle_duration_sec=10.0)
        stamp = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        self._reads_pub.publish(String(data=format_uhf_read_batch(
            str(uuid.uuid4()), reads, pose, stamp)))

    def shutdown(self):
        """Stop the module's continuous read and release the port."""
        try:
            if self._ser.connected:
                self._ser.write(uhf.cmd_stop_continuous())
                self._ser.close()
        except Exception:  # noqa: BLE001 — best-effort cleanup on shutdown
            pass


def main(args=None):
    run_node(UhfNode, on_shutdown=lambda n: n.shutdown(), args=args)


if __name__ == '__main__':
    main()
