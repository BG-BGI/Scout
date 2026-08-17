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

Every subsystem is STALE until its topic delivers and goes STALE again if it
stops — a dead driver, a crashed tilt_monitor or a silent battery_monitor all
surface here as a red item rather than an absent topic nobody is watching. See
ADR-0014.
"""

import json

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from scout.core import health
from scout.node_util import run_node
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

        self._battery = None
        self._battery_t = None
        self._tilt = None
        self._tilt_t = None
        self._drive_t = None

        self._pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.create_subscription(Bool, 'tilt_alarm', self._on_tilt, 10)
        self.create_subscription(String, 'roboclaw_status', self._on_status, 10)
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
            json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._drive_t = self.get_clock().now()

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
                self._drivetrain_status()]
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
