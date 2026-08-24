#!/usr/bin/env python3
"""Flipper Zero bridge: enable-gated RFID scan loop + generic CLI passthrough.

Design (ADR-0025, led_node conventions):
  * The node owns the single FlipperCli serial handle; the poll timer is the
    SOLE serial reader/writer. Services only mutate target state (the enable
    flag) or run one short bounded command inline.
  * RFID scanning is OFF at boot and only ever turned on by a human via
    /flipper/rfid_enable (std_srvs/SetBool — the webui RFID panel). While
    enabled the node loops `rfid read` (the CLI blocks alternating ASK/PSK
    until a card or Ctrl+C); each card is stamped with the robot's map pose
    (lookup_pose2 at detection time — null when unlocalized) and published as
    latched JSON on /rfid/reads, where the zenoh bridge carries it to the
    companion's rfid_recorder (the primary DB).
  * A serial fault drops the enable flag: after an unplug the operator
    re-enables deliberately instead of the robot resuming radio work alone.
  * Flipper absent is normal (tier 2): the node idles in DISCONNECTED with a
    throttled warn and keeps retrying; the robot stays fully drivable.
"""

import json
import time
import uuid
from datetime import datetime, timezone

import tf2_ros
from rclpy.node import Node
from scout_interfaces.srv import FlipperCli as FlipperCliSrv
from std_msgs.msg import String
from std_srvs.srv import SetBool

from scout.core.rfid import PROMPT, make_read_json, parse_read_output, strip_echo
from scout.flipper_cli import FlipperCli
from scout.node_util import lookup_pose2, run_node
from scout.qos import LATCHED_HISTORY_QOS, LATCHED_QOS

DISCONNECTED = 'disconnected'
IDLE = 'idle'
SCANNING = 'scanning'


class FlipperNode(Node):
    """Owns the Flipper CLI serial port; scan loop gated by /flipper/rfid_enable."""

    def __init__(self):
        super().__init__('flipper_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('poll_hz', 10.0)
        self.declare_parameter('reconnect_period_s', 5.0)
        self.declare_parameter('cli_timeout_s', 2.0)
        self.declare_parameter('duplicate_suppress_s', 10.0)

        p = self.get_parameter
        self._cli = FlipperCli(str(p('port').value), int(p('baud').value))
        self._reconnect_period = float(p('reconnect_period_s').value)
        self._cli_timeout = float(p('cli_timeout_s').value)
        self._dup_suppress = float(p('duplicate_suppress_s').value)

        self._state = DISCONNECTED
        self._enabled = False
        self._last_error = ''
        self._buf = ''
        self._last_connect_attempt = 0.0
        self._recent_hex = {}          # data_hex -> last publish (monotonic)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._status_pub = self.create_publisher(String, 'flipper/status',
                                                 LATCHED_QOS)
        self._reads_pub = self.create_publisher(String, 'rfid/reads',
                                                LATCHED_HISTORY_QOS)
        self.create_service(SetBool, 'flipper/rfid_enable', self._on_enable)
        self.create_service(FlipperCliSrv, 'flipper/cli', self._on_cli)
        self.create_timer(1.0 / float(p('poll_hz').value), self._tick)

        self._publish_status()
        self.get_logger().info(
            'flipper_node up on %s: RFID scanning DISABLED until '
            '/flipper/rfid_enable (webui RFID panel)' % str(p('port').value))

    # --- status ---------------------------------------------------------------
    def _publish_status(self):
        self._status_pub.publish(String(data=json.dumps({
            'state': self._state,
            'connected': self._cli.connected,
            'rfid_enabled': self._enabled,
            'last_error': self._last_error,
        })))

    def _set_state(self, state, error=None):
        if error is not None:
            self._last_error = error
        if state != self._state or error is not None:
            self._state = state
            self._publish_status()

    def _fault(self, exc, where):
        self.get_logger().warn('serial fault (%s): %s — disconnected, RFID '
                               'disabled until re-enabled' % (where, exc))
        try:
            self._cli.close()
        except OSError:
            pass
        self._enabled = False
        self._buf = ''
        self._set_state(DISCONNECTED, error='%s: %s' % (where, exc))

    # --- services (mutate state / short bounded I/O only) ----------------------
    def _on_enable(self, request, response):
        if request.data and not self._cli.connected:
            response.success = False
            response.message = 'flipper not connected'
            return response
        self._enabled = bool(request.data)
        self._publish_status()
        response.success = True
        response.message = ('RFID scanning enabled'
                            if self._enabled else 'RFID scanning disabled')
        self.get_logger().info(response.message)
        return response

    def _on_cli(self, request, response):
        if not self._cli.connected:
            response.success = False
            response.output = 'flipper not connected'
            return response
        if self._enabled or self._state == SCANNING:
            response.success = False
            response.output = 'busy: RFID scanning enabled — disable it first'
            return response
        timeout = request.timeout_s if request.timeout_s > 0.0 else self._cli_timeout
        timeout = min(timeout, self._cli_timeout)
        try:
            self._cli.send_line(request.command)
            deadline = time.monotonic() + timeout
            out = ''
            while time.monotonic() < deadline:
                out += self._cli.read_available()
                if PROMPT in out:
                    body = strip_echo(out, request.command)
                    response.success = True
                    response.output = body.split(PROMPT)[0].strip()
                    return response
                time.sleep(0.02)
        except OSError as exc:            # covers serial.SerialException
            self._fault(exc, 'cli')
            response.success = False
            response.output = 'serial fault: %s' % exc
            return response
        # Timed out mid-command: Ctrl+C so the shell is usable again.
        self._cli.send_ctrl_c()
        self._cli.drain_to_prompt(self._cli_timeout)
        response.success = False
        response.output = 'timeout waiting for prompt (%.1f s)' % timeout
        return response

    # --- the poll timer: sole serial reader/writer -----------------------------
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

    def _tick_disconnected(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_period:
            return
        self._last_connect_attempt = now
        try:
            ok = self._cli.open(settle_s=self._cli_timeout)
        except OSError as exc:
            self.get_logger().warn(
                'flipper not connected (%s) — RFID unavailable, retrying'
                % exc, throttle_duration_sec=30.0)
            return
        if not ok:
            self.get_logger().warn('port opened but no CLI prompt — not a '
                                   'flipper shell?', throttle_duration_sec=30.0)
            self._cli.close()
            return
        self.get_logger().info('flipper connected')
        self._set_state(IDLE, error='')

    def _tick_idle(self):
        self._cli.read_available()        # keep the buffer drained
        if self._enabled:
            self._buf = ''
            self._cli.send_line('rfid read')
            self._set_state(SCANNING)

    def _tick_scanning(self):
        if not self._enabled:
            self._cli.send_ctrl_c()
            self._cli.drain_to_prompt(self._cli_timeout)
            self._set_state(IDLE)
            return
        self._buf += self._cli.read_available()
        hit = parse_read_output(self._buf)
        if hit is None:
            return
        self._publish_read(hit)
        # Restart the read: back to the prompt, then IDLE re-arms next tick
        # (still enabled), so the loop keeps scanning until disabled.
        self._cli.send_ctrl_c()
        self._cli.drain_to_prompt(self._cli_timeout)
        self._buf = ''
        self._set_state(IDLE)

    def _publish_read(self, hit):
        now = time.monotonic()
        last = self._recent_hex.get(hit['data_hex'])
        if last is not None and now - last < self._dup_suppress:
            return                        # same card still in the field
        self._recent_hex[hit['data_hex']] = now

        pose = lookup_pose2(self._tf_buffer, 'map', 'base_link')
        if pose is None:
            self.get_logger().warn('RFID read without map localization — '
                                   'pose recorded as null',
                                   throttle_duration_sec=10.0)
        stamp = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        payload = make_read_json(hit['protocol'], hit['data_hex'], pose,
                                 stamp, str(uuid.uuid4()))
        self._reads_pub.publish(String(data=payload))
        self.get_logger().info('RFID read: %s %s pose=%s'
                               % (hit['protocol'], hit['data_hex'],
                                  'null' if pose is None else
                                  '(%.2f, %.2f)' % (pose[0], pose[1])))

    def shutdown(self):
        """Abort any in-flight read and release the port."""
        try:
            if self._cli.connected:
                self._cli.send_ctrl_c()
                self._cli.close()
        except Exception:  # noqa: BLE001 — best-effort cleanup on shutdown
            pass


def main(args=None):
    run_node(FlipperNode, on_shutdown=lambda n: n.shutdown(), args=args)


if __name__ == '__main__':
    main()
