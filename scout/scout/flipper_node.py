#!/usr/bin/env python3
"""Flipper Zero bridge: enable-gated RFID/NFC scan loop + generic CLI passthrough.

Design (ADR-0025/0026, led_node conventions):
  * The node owns the single FlipperCli serial handle; the poll timer is the
    SOLE serial reader/writer. Services only mutate target state (the enable
    flag / scan mode) or run one short bounded command inline.
  * Scanning is OFF at boot and only ever turned on by a human via
    /flipper/rfid_enable or /flipper/nfc_enable (std_srvs/SetBool — the webui
    RFID and NFC panels). One serial line means the two modes are MUTUALLY
    EXCLUSIVE: enabling one while the other is on is rejected.
      - RFID: loop `rfid read` (flat command, blocks ASK/PSK until a card).
      - NFC: no CLI verb prints a UID, so the loop DUMPS the card to a .nfc file
        and reads it back — `nfc` (enter the sub-shell) then `dump` each cycle,
        and on a successful dump `exit` to the top level to `storage read` /
        `storage remove` the file (storage is unavailable inside the sub-shell).
        `exit` also runs on disable to return to `>:` (ADR-0026, core/nfc.py).
    Each read is stamped with the robot's map pose (lookup_pose2 at detection
    time — null when unlocalized) and published as latched JSON on /rfid/reads
    or /nfc/reads, where the zenoh bridge carries it to the companion recorder
    (the primary DB).
  * A serial fault drops the enable flag AND the sub-shell state: after an
    unplug the operator re-enables deliberately instead of the robot resuming
    radio work alone.
  * Flipper absent is normal (tier 2): the node idles in DISCONNECTED with a
    throttled warn and keeps retrying; the robot stays fully drivable.
"""

import time
import uuid
from datetime import datetime, timezone

import tf2_ros
from rclpy.node import Node
from scout_interfaces.srv import FlipperCli as FlipperCliSrv
from std_msgs.msg import String
from std_srvs.srv import SetBool

from scout.core.nfc import (
    NFC_ENTER,
    NFC_EXIT,
    nfc_dump_cmd,
    nfc_mkdir_cmd,
    nfc_read_file_cmd,
    nfc_remove_cmd,
    parse_dump_output,
    parse_nfc_file,
)
from scout.core.rfid import PROMPT, parse_read_output, strip_echo
from scout.core.status import (
    format_flipper_status,
    format_nfc_read,
    format_rfid_read,
)
from scout.flipper_cli import FlipperCli
from scout.node_util import lookup_pose2, run_node
from scout.qos import LATCHED_HISTORY_QOS, LATCHED_QOS

DISCONNECTED = 'disconnected'
IDLE = 'idle'
SCANNING = 'scanning'

RFID = 'rfid'
NFC = 'nfc'

# NFC read phases (see core/nfc.py header): DUMP = waiting on `dump` inside the
# nfc sub-shell; READFILE = waiting on top-level `storage read` of the .nfc file.
NFC_DUMP = 'dump'
NFC_READFILE = 'readfile'


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
        self._scan_mode = RFID         # which radio the loop drives when enabled
        self._in_nfc_shell = False     # inside the Flipper `nfc` sub-shell
        self._nfc_phase = None         # NFC_DUMP / NFC_READFILE while scanning
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
        self._nfc_reads_pub = self.create_publisher(String, 'nfc/reads',
                                                    LATCHED_HISTORY_QOS)
        self.create_service(SetBool, 'flipper/rfid_enable',
                            lambda req, resp: self._on_enable(RFID, req, resp))
        self.create_service(SetBool, 'flipper/nfc_enable',
                            lambda req, resp: self._on_enable(NFC, req, resp))
        self.create_service(FlipperCliSrv, 'flipper/cli', self._on_cli)
        self.create_timer(1.0 / float(p('poll_hz').value), self._tick)

        self._publish_status()
        self.get_logger().info(
            'flipper_node up on %s: RFID/NFC scanning DISABLED until '
            '/flipper/rfid_enable or /flipper/nfc_enable (webui panels)'
            % str(p('port').value))

    # --- status ---------------------------------------------------------------
    def _publish_status(self):
        self._status_pub.publish(String(data=format_flipper_status(
            self._state, self._cli.connected,
            self._enabled and self._scan_mode == RFID,
            self._enabled and self._scan_mode == NFC,
            self._last_error)))

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
            self._cli.close()
        except OSError:
            pass
        self._enabled = False
        self._in_nfc_shell = False
        self._nfc_phase = None
        self._buf = ''
        self._set_state(DISCONNECTED, error='%s: %s' % (where, exc))

    # --- services (mutate state / short bounded I/O only) ----------------------
    def _on_enable(self, mode, request, response):
        """Shared handler for /flipper/rfid_enable and /flipper/nfc_enable.
        One serial line => the two modes are mutually exclusive; enabling one
        while the OTHER is scanning is rejected (disable it first)."""
        other = NFC if mode == RFID else RFID
        if request.data:
            if not self._cli.connected:
                response.success = False
                response.message = 'flipper not connected'
                return response
            if self._enabled and self._scan_mode == other:
                response.success = False
                response.message = ('busy: %s scanning enabled — disable it '
                                    'first' % other.upper())
                return response
            self._scan_mode = mode
            self._enabled = True
        else:
            # A disable only clears the flag if it names the active mode, so a
            # stale RFID-disable cannot silently stop an NFC scan.
            if self._enabled and self._scan_mode == mode:
                self._enabled = False
        self._publish_status()
        response.success = True
        response.message = ('%s scanning enabled' % mode.upper()
                            if (self._enabled and self._scan_mode == mode)
                            else '%s scanning disabled' % mode.upper())
        self.get_logger().info(response.message)
        return response

    def _on_cli(self, request, response):
        if not self._cli.connected:
            response.success = False
            response.output = 'flipper not connected'
            return response
        if self._enabled or self._state == SCANNING:
            response.success = False
            response.output = ('busy: %s scanning enabled — disable it first'
                               % self._scan_mode.upper())
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
        if not self._enabled:
            return
        self._buf = ''
        if self._scan_mode == RFID:
            self._cli.send_line('rfid read')
        else:
            self._start_nfc_dump()
        self._set_state(SCANNING)

    def _start_nfc_dump(self):
        """Begin one NFC read cycle. First entry (top level) clears any stale
        temp file and ensures /ext/nfc exists, then enters the `nfc` sub-shell;
        every cycle issues `dump` and waits in the NFC_DUMP phase. There is no
        UID on the CLI, so we dump the card to a .nfc file and read it back
        (core/nfc.py)."""
        if not self._in_nfc_shell:
            for cmd in (nfc_mkdir_cmd(), nfc_remove_cmd()):   # top level
                self._cli.send_line(cmd)
                self._cli.drain_to_prompt(self._cli_timeout)
            self._cli.send_line(NFC_ENTER)
            self._in_nfc_shell = True
        self._cli.send_line(nfc_dump_cmd())
        self._nfc_phase = NFC_DUMP

    def _tick_scanning(self):
        if not self._enabled:
            self._stop_scanning()
            return
        self._buf += self._cli.read_available()
        # TEMPORARY bench capture at INFO (revert to debug once the real
        # firmware shapes are confirmed on this unit): core/nfc.py's header says
        # the dump->storage-read shell dance is from firmware SOURCE, not yet
        # verified end to end on hardware.
        if self._buf:
            self.get_logger().info('%s raw buf: %r' % (self._scan_mode, self._buf),
                                   throttle_duration_sec=2.0)
        if self._scan_mode == RFID:
            self._tick_scanning_rfid()
        else:
            self._tick_scanning_nfc()

    def _stop_scanning(self):
        """Disable took effect mid-scan: Ctrl+C the in-flight command, leave the
        nfc sub-shell if we are in it (so the port is back at the top-level
        prompt), and idle."""
        self._cli.send_ctrl_c()
        self._cli.drain_to_prompt(self._cli_timeout)
        if self._scan_mode == NFC and self._in_nfc_shell:
            self._cli.send_line(NFC_EXIT)   # leave sub-shell -> top-level
            self._cli.drain_to_prompt(self._cli_timeout)
            self._in_nfc_shell = False
        self._nfc_phase = None
        self._buf = ''
        self._set_state(IDLE)

    def _tick_scanning_rfid(self):
        hit = parse_read_output(self._buf)
        if hit is None:
            return
        self._publish_read(hit)
        # Restart: Ctrl+C back to the top-level prompt; IDLE re-arms next tick
        # (still enabled) so the loop keeps reading.
        self._cli.send_ctrl_c()
        self._cli.drain_to_prompt(self._cli_timeout)
        self._buf = ''
        self._set_state(IDLE)

    def _tick_scanning_nfc(self):
        if self._nfc_phase == NFC_DUMP:
            result = parse_dump_output(self._buf)
            if result is None:
                return                     # dump still running (up to ~5 s)
            if result == 'error':          # no card / read fail: re-dump in shell
                self._buf = ''
                self._cli.send_line(nfc_dump_cmd())
                return
            # 'saved': storage is not available inside the sub-shell, so leave it
            # and stream the dumped .nfc file back at the top level.
            self._cli.send_line(NFC_EXIT)
            self._cli.drain_to_prompt(self._cli_timeout)
            self._in_nfc_shell = False
            self._buf = ''
            self._cli.send_line(nfc_read_file_cmd())
            self._nfc_phase = NFC_READFILE
            return
        # NFC_READFILE: the whole file streams before the prompt returns.
        if PROMPT not in self._buf:
            return
        hit = parse_nfc_file(self._buf)
        self._cli.send_line(nfc_remove_cmd())      # drop the temp file
        self._cli.drain_to_prompt(self._cli_timeout)
        self._buf = ''
        self._nfc_phase = None
        if hit is not None:
            self._publish_read(hit)
        # Back at the top level; next IDLE re-enters the sub-shell and dumps.
        self._set_state(IDLE)

    def _publish_read(self, hit):
        now = time.monotonic()
        last = self._recent_hex.get(hit['data_hex'])
        if last is not None and now - last < self._dup_suppress:
            return                        # same card still in the field
        self._recent_hex[hit['data_hex']] = now

        pose = lookup_pose2(self._tf_buffer, 'map', 'base_link')
        if pose is None:
            self.get_logger().warn('%s read without map localization — pose '
                                   'recorded as null'
                                   % self._scan_mode.upper(),
                                   throttle_duration_sec=10.0)
        stamp = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        if self._scan_mode == RFID:
            payload = format_rfid_read(hit['protocol'], hit['data_hex'], pose,
                                       stamp, str(uuid.uuid4()))
            self._reads_pub.publish(String(data=payload))
        else:
            payload = format_nfc_read(hit['protocol'], hit['data_hex'], pose,
                                      stamp, str(uuid.uuid4()))
            self._nfc_reads_pub.publish(String(data=payload))
        self.get_logger().info('%s read: %s %s pose=%s'
                               % (self._scan_mode.upper(), hit['protocol'],
                                  hit['data_hex'], 'null' if pose is None else
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
