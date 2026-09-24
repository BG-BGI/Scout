#!/usr/bin/env python3
"""Aggregate the robot's health onto one diagnostic_msgs/DiagnosticArray.

battery_monitor, tilt_monitor and roboclaw_driver each report health on their
own topic in their own shape (/battery, /tilt_alarm, /roboclaw_status), so no
single view — Foxglove's Diagnostics panel or the webui strip — can answer "is
the robot OK". This node subscribes to those, applies the shared OK/WARN/ERROR/
STALE logic in scout.core.health, and republishes them as standard diagnostics
on /diagnostics at 1 Hz, with an overall roll-up as the first status.

Subsystems in this version:
  * battery      — resting-voltage ladder (warn/critical from robot_profile)
  * tilt         — the tilt_monitor abort latch
  * drivetrain   — /roboclaw_status liveness: any parseable, recent status
                   message means the serial link is up and the driver is
                   publishing. Pack temperatures and error flags are a later
                   add, once the status JSON schema is confirmed on hardware.
  * traction     — /traction/status derates (WARN while a side is derated)
  * collision    — the CM bypass + zone mode (latched — see below)
  * flipper      — /flipper/status (latched; absent hardware is OK)
  * uhf          — /uhf/status (latched; absent hardware is OK, throttle WARNs)
  * cliff        — /cliff/stop_points freshness. cliff_detector goes
                   DELIBERATELY silent on camera/TF loss, so STALE here means
                   the negative-obstacle safeguard is blind (ADR-0024) — this
                   row is why that silence is safe to keep.
  * cmd_stream   — the autonomy command chain /cmd_vel_auto -> /cmd_vel_safe
                   -> /cmd_vel_out (scout.core.cmdflow, ADR-0036): one
                   stop-reason label (auto_idle / cm_dead / cm_stop:<zone> /
                   bypass / zone_unsynced / mux_override / moving) plus
                   per-hop ages and worst inter-arrival gaps. Also mirrored
                   on latched /stop_reason (published on change only) so one
                   bagged breadcrumb separates bridge congestion, CPU
                   throttling, a stale sensor stop, and a real collision
                   stop. cmd_stream is a classifier, not a gate — it never
                   touches the deadman, mux, or CM timeouts. An idle robot
                   reads auto_idle, which is OK, not a fault. The three
                   Twist subscriptions are ~4 float reads at <=50 Hz each.

Streamed subsystems are STALE until their topic delivers and STALE again if it
stops. LATCHED subsystems (collision, flipper, uhf) publish once per change, so
only never-seen means STALE for them — age has no meaning on a latched wire.
See ADR-0014.
"""

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, PointCloud2
from std_msgs.msg import Bool, String

from scout.core import health
from scout.core.cmdflow import (
    CM_DEAD,
    ZONE_UNSYNCED,
    TopicFlow,
    stop_reason,
    twist_is_zero,
)
from scout.core.status import (
    parse_flipper_status,
    parse_roboclaw_status,
    parse_traction_status,
    parse_uhf_status,
)
from scout.node_util import run_node
from scout.qos import LATCHED_QOS
from scout.robot_profile import load as _load_profile


class HealthMonitor(Node):
    """Republish battery/tilt/drivetrain health as a DiagnosticArray."""

    def __init__(self):
        super().__init__('health_monitor')
        # scout.core.health redeclares the DiagnosticStatus values as plain
        # ints to stay ROS-free (ADR-0012); fail loudly here if upstream ever
        # renumbers them. ⚠ Some rosidl_generator_py versions represent
        # `byte`-typed msg constants as single-byte `bytes` (b'\x00') rather
        # than int (seen on a diagnostic_msgs version bump) — normalize both
        # representations before comparing; a real renumber still trips this.
        def _as_int(v):
            return v[0] if isinstance(v, bytes) else int(v)
        assert (health.OK, health.WARN, health.ERROR, health.STALE) == tuple(
            _as_int(v) for v in (
                DiagnosticStatus.OK, DiagnosticStatus.WARN,
                DiagnosticStatus.ERROR, DiagnosticStatus.STALE))

        prof = _load_profile()
        self._warn_v = float(prof['battery_warn_v'])
        self._crit_v = float(prof['battery_critical_v'])

        self._publish_period = float(
            self.declare_parameter('publish_period', 1.0).value)
        # Per-subsystem staleness timeouts (per-node tunables, not profile).
        # battery_monitor publishes ~1 Hz, tilt heartbeats 1 Hz, the driver
        # streams status ~10 Hz — each timeout is a few missed cycles.
        self._battery_timeout = float(
            self.declare_parameter('battery_timeout_s', 5.0).value)
        self._tilt_timeout = float(
            self.declare_parameter('tilt_timeout_s', 5.0).value)
        self._drive_timeout = float(
            self.declare_parameter('drivetrain_timeout_s', 2.0).value)
        # traction publishes per driver status tick (~10 Hz); cliff per
        # processed depth cloud (~5 Hz).
        self._traction_timeout = float(
            self.declare_parameter('traction_timeout_s', 3.0).value)
        self._cliff_timeout = float(
            self.declare_parameter('cliff_timeout_s', 3.0).value)

        self._battery = None
        self._battery_t = None
        self._tilt = None
        self._tilt_t = None
        self._drive_t = None
        self._traction = None          # parsed /traction/status dict
        self._traction_t = None
        self._bypassed = None          # latched: None until first message
        self._zone_mode = 'forward'
        self._zone_sync = None         # latched: None until first message
        self._flipper = None           # latched: parsed dict or None
        self._uhf = None               # latched: parsed dict or None
        self._cliff_pts = None
        self._cliff_t = None
        # Command-chain flow trackers (core.cmdflow) on monotonic time —
        # arrival measurement only, plain Twists carry no source stamp.
        self._auto_flow = TopicFlow()
        self._safe_flow = TopicFlow()
        self._out_flow = TopicFlow()
        self._stop_reason_published = None

        self._pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._stop_reason_pub = self.create_publisher(
            String, '/stop_reason', LATCHED_QOS)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.create_subscription(Bool, 'tilt_alarm', self._on_tilt, 10)
        self.create_subscription(String, 'roboclaw_status', self._on_status, 10)
        self.create_subscription(String, '/traction/status',
                                 self._on_traction, 10)
        self.create_subscription(Bool, '/collision_monitor/bypassed',
                                 self._on_bypassed, LATCHED_QOS)
        self.create_subscription(String, '/collision_monitor/zone_mode',
                                 self._on_zone_mode, LATCHED_QOS)
        self.create_subscription(Bool, '/collision_monitor/zone_sync',
                                 self._on_zone_sync, LATCHED_QOS)
        # cmd_vel chain hops from the profile (SC6 — one owner for topic
        # names, same keys twist_mux.yaml is kept in step with).
        self.create_subscription(
            Twist, prof['topic_cmd_vel_auto'],
            lambda m: self._auto_flow.on_msg(
                time.monotonic(), twist_is_zero(m.linear.x, m.angular.z)), 10)
        self.create_subscription(
            Twist, prof['topic_cmd_vel_safe'],
            lambda m: self._safe_flow.on_msg(
                time.monotonic(), twist_is_zero(m.linear.x, m.angular.z)), 10)
        self.create_subscription(
            Twist, prof['topic_cmd_vel_out'],
            lambda m: self._out_flow.on_msg(
                time.monotonic(), twist_is_zero(m.linear.x, m.angular.z)), 10)
        self.create_subscription(String, '/flipper/status',
                                 self._on_flipper, LATCHED_QOS)
        self.create_subscription(String, '/uhf/status',
                                 self._on_uhf, LATCHED_QOS)
        self.create_subscription(PointCloud2, '/cliff/stop_points',
                                 self._on_cliff, qos_profile_sensor_data)
        self.create_timer(self._publish_period, self._publish)
        self.get_logger().info(
            'health_monitor up: /diagnostics at %.1f Hz' % (1.0 / self._publish_period))

    def _on_battery(self, msg: BatteryState):
        self._battery = msg
        self._battery_t = self.get_clock().now()

    def _on_tilt(self, msg: Bool):
        self._tilt = msg.data
        self._tilt_t = self.get_clock().now()

    def _on_status(self, msg: String):
        # Any parseable status message proves the serial link is alive and the
        # driver is publishing; the fields inside it are battery_monitor's job.
        try:
            parse_roboclaw_status(msg.data)
        except ValueError:
            return
        self._drive_t = self.get_clock().now()

    def _on_traction(self, msg: String):
        try:
            self._traction = parse_traction_status(msg.data)
        except ValueError:
            return
        self._traction_t = self.get_clock().now()

    def _on_bypassed(self, msg: Bool):
        self._bypassed = msg.data

    def _on_zone_mode(self, msg: String):
        self._zone_mode = msg.data

    def _on_zone_sync(self, msg: Bool):
        self._zone_sync = msg.data

    def _on_flipper(self, msg: String):
        try:
            self._flipper = parse_flipper_status(msg.data)
        except ValueError:
            pass

    def _on_uhf(self, msg: String):
        try:
            self._uhf = parse_uhf_status(msg.data)
        except ValueError:
            pass

    def _on_cliff(self, msg: PointCloud2):
        self._cliff_pts = msg.width
        self._cliff_t = self.get_clock().now()

    def _age(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _battery_status(self):
        lvl, msg = health.staleness_level(
            self._age(self._battery_t), self._battery_timeout, 'battery')
        values = []
        if lvl == health.OK and self._battery is not None:
            b = self._battery
            lvl, msg = health.battery_level(
                b.present, b.voltage, b.percentage, self._warn_v, self._crit_v)
            values = [KeyValue(key='voltage_v', value='%.2f' % b.voltage)]
        return self._status('battery', lvl, msg, values)

    def _tilt_status(self):
        lvl, msg = health.staleness_level(
            self._age(self._tilt_t), self._tilt_timeout, 'tilt')
        if lvl == health.OK:
            lvl, msg = health.tilt_level(self._tilt)
        return self._status('tilt', lvl, msg, [])

    def _drivetrain_status(self):
        lvl, msg = health.staleness_level(
            self._age(self._drive_t), self._drive_timeout, 'drivetrain')
        if lvl == health.OK:
            msg = 'drivetrain: serial link up'
        return self._status('drivetrain', lvl, msg, [])

    def _traction_status(self):
        lvl, msg = health.staleness_level(
            self._age(self._traction_t), self._traction_timeout, 'traction')
        if lvl == health.OK:
            t = self._traction
            left = t['left']
            right = 'm2' if left == 'm1' else 'm1'
            lvl, msg = health.traction_level(
                t['m1']['verdict'], t['m2']['verdict'],
                t[left]['derate'], t[right]['derate'])
        return self._status('traction', lvl, msg, [])

    def _collision_status(self):
        # Latched wire: only never-seen is stale (age is meaningless).
        if self._bypassed is None:
            lvl, msg = health.staleness_level(None, 0.0, 'collision')
        else:
            lvl, msg = health.bypass_level(self._bypassed, self._zone_mode)
        return self._status('collision', lvl, msg, [])

    def _flipper_status(self):
        if self._flipper is None:
            lvl, msg = health.staleness_level(None, 0.0, 'flipper')
        else:
            f = self._flipper
            lvl, msg = health.flipper_level(
                f.get('connected', False), f.get('rfid_enabled', False),
                f.get('last_error', ''), f.get('nfc_enabled', False))
        return self._status('flipper', lvl, msg, [])

    def _uhf_status(self):
        if self._uhf is None:
            lvl, msg = health.staleness_level(None, 0.0, 'uhf')
        else:
            u = self._uhf
            lvl, msg = health.uhf_level(
                u.get('connected', False), u.get('enabled', False),
                u.get('throttled', False), u.get('last_error', ''))
        return self._status('uhf', lvl, msg, [])

    def _cliff_status(self):
        lvl, msg = health.staleness_level(
            self._age(self._cliff_t), self._cliff_timeout, 'cliff')
        if lvl == health.OK:
            lvl, msg = health.cliff_level(self._cliff_pts)
        return self._status('cliff', lvl, msg, [])

    def _cmd_stream_status(self):
        now = time.monotonic()
        # zone_sync None = collision_polygon_manager not seen yet; don't warn
        # on a wire that hasn't latched (the collision row covers never-seen).
        reason = stop_reason(
            now, self._auto_flow, self._safe_flow, self._out_flow,
            bypassed=bool(self._bypassed), zone_mode=self._zone_mode,
            zone_synced=self._zone_sync is not False)
        # cm_stop/bypass/auto_idle are the system working as designed — only
        # a silent CM or an unacked zone push is a fault of the chain itself.
        lvl = health.WARN if reason in (
            CM_DEAD, ZONE_UNSYNCED) else health.OK
        values = []
        for name, flow in (('auto', self._auto_flow),
                           ('safe', self._safe_flow),
                           ('out', self._out_flow)):
            age = flow.age(now)
            gap = flow.max_gap(now)
            values.append(KeyValue(
                key='%s_age_s' % name,
                value='%.2f' % age if age is not None else 'never'))
            values.append(KeyValue(
                key='%s_max_gap_s' % name,
                value='%.2f' % gap if gap is not None else 'n/a'))
        if reason != self._stop_reason_published:
            self._stop_reason_published = reason
            self._stop_reason_pub.publish(String(data=reason))
        return self._status('cmd_stream', lvl, reason, values)

    def _status(self, name, level, message, values):
        s = DiagnosticStatus()
        s.name = name
        s.hardware_id = 'scout'
        s.level = bytes([level])
        s.message = message
        s.values = values
        return s

    def _publish(self):
        subs = [self._battery_status(), self._tilt_status(),
                self._drivetrain_status(), self._traction_status(),
                self._collision_status(), self._flipper_status(),
                self._uhf_status(), self._cliff_status(),
                self._cmd_stream_status()]
        level = health.worst([s.level[0] for s in subs])
        overall = self._status(
            'scout', level, 'OK' if level == health.OK else 'attention', [])
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [overall] + subs
        self._pub.publish(arr)


def main(args=None):
    run_node(HealthMonitor, args=args)


if __name__ == '__main__':
    main()
