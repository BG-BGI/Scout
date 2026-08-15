#!/usr/bin/env python3
"""Republish the RoboClaw's main battery voltage as a sensor_msgs/BatteryState.

The driver already reads the pack every 0.5 s, but buries it in the JSON blob it
publishes on /roboclaw_status as a std_msgs/String, where no panel can plot it
and no node can act on it. This node parses that blob and re-emits the pack on
'battery' as a real message.

Charge is estimated from voltage alone, because nothing else here is
trustworthy. The RoboClaw's current telemetry is fiction below ~20% duty, and
was measured reading 0.84-0.96 A while duty swept 45->100% during a pivot. The
5 V buck also taps the pack upstream of the RoboClaw, so the Pi and LED draw
never appears in motor current at all. Coulomb counting off those numbers would
accumulate error in one direction only.

Voltage sags hard under drive current, so the estimate is only taken at rest:
both encoder speeds in the same status message must have been near zero for
`rest_seconds` before a sample counts. Between rest samples the last estimate is
held, and `percentage` stays NaN until the first one lands.

Resting samples are collected and medianed once per `estimate_period` rather
than fed to the curve one at a time. The pack reading quantises to 0.1 V, which
is worth ~3% of charge through the flat middle of the curve, so a single-step
flicker between adjacent readings would otherwise dither `percentage` by more
than the discharge moves it in several minutes. A median rejects that outright,
where an average would only halve it.

Calibrate `voltage_scale`/`voltage_offset` against a multimeter before trusting
the percentage. The RoboClaw's voltage readback is known to be off on this board
(its Max Main setting ratchets down through readback across sessions), and in
the flat middle of a li-ion curve 0.1 V is worth roughly 10% of charge.

There is no BMS on this pack. The RoboClaw's 16.0 V Min Main trip is the only
protection and it fires on *loaded* voltage, so at the end of a discharge it
chatters (motors cut, voltage recovers, motors restart) rather than stopping
cleanly. Warning well before that is the point of the low/critical thresholds.
"""

import json
import math

from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from scout.core.battery import (
    DEFAULT_CURVE_FRACTION,
    DEFAULT_CURVE_VOLTS,
    RestingSocEstimator,
    validate_curve,
)
from scout.node_util import run_node
from scout.robot_profile import load as _load_profile

# Below this the RoboClaw has not completed its first battery read (the status
# state machine cycles through five reads, so the field is 0.0 for up to 0.5 s
# after startup) or the link is broken. Either way it is not a pack reading.
_MIN_PLAUSIBLE_VOLTS = 5.0

# Only announce a new resting estimate once it has moved this far from the last
# one *announced* — comparing against the last one computed instead lets a value
# that creeps past the threshold in small steps go unreported forever.
#
# It has to clear the pack's own quantisation noise or it announces the noise.
# The RoboClaw reports main battery in 0.1 V steps, and through the flat middle
# of the curve (18-20 V) the slope is 0.25-0.30 charge per volt, so a one-step
# flicker between adjacent readings is worth 2.5-3% of charge. That is what the
# 2% this started at was reporting, ten times a second, while parked.
_LOG_DELTA_FRACTION = 0.05


class BatteryMonitor(Node):
    """Parse /roboclaw_status and publish the main pack as a BatteryState."""

    def __init__(self):
        super().__init__('battery_monitor')

        # Well under the driver's own 10 Hz status rate: the underlying value
        # only refreshes every 0.5 s, and nothing needs a pack voltage faster.
        self.publish_rate = max(0.1, self.declare_parameter('publish_rate', 1.0).value)
        # Stop publishing rather than repeat a stale voltage if the driver dies.
        self.status_timeout = self.declare_parameter('status_timeout', 3.0).value
        # Multimeter correction, applied as scale * raw + offset.
        self.voltage_scale = self.declare_parameter('voltage_scale', 1.0).value
        self.voltage_offset = self.declare_parameter('voltage_offset', 0.0).value
        # Stillness gate for a resting sample. 50 counts/s is well below the
        # velocity loop's own 300-500 counts/s tracking floor, so it means
        # stopped rather than crawling. Li-ion needs a few seconds to recover
        # after a load comes off, hence the delay before a sample counts.
        self.rest_speed_counts = self.declare_parameter('rest_speed_counts', 50.0).value
        self.rest_seconds = self.declare_parameter('rest_seconds', 3.0).value
        # How often a resting estimate is recomputed, from the median of the
        # samples gathered since the last one. The underlying value only
        # refreshes at 2 Hz and a pack does not move meaningfully in seconds,
        # so this is about collecting enough samples to median, not about lag.
        self.estimate_period = self.declare_parameter('estimate_period', 5.0).value
        # Threshold ladder defaults come from robot_profile.yaml (SSOT — the
        # webui badge and led_status read the same values). Warn is ~3.5 V/cell
        # (~20% left); critical sits just above the RoboClaw cutoff, and these
        # are loaded readings so they fire early under drive current — the
        # useful direction to be wrong in.
        _prof = _load_profile()
        self.warn_voltage = self.declare_parameter(
            'warn_voltage', float(_prof['battery_warn_v'])).value
        self.critical_voltage = self.declare_parameter(
            'critical_voltage', float(_prof['battery_critical_v'])).value
        self.design_capacity = self.declare_parameter('design_capacity', 5.0).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        volts = list(self.declare_parameter('curve_volts', DEFAULT_CURVE_VOLTS).value)
        fraction = list(
            self.declare_parameter('curve_fraction', DEFAULT_CURVE_FRACTION).value)
        self._curve_volts, self._curve_fraction = self._validate_curve(volts, fraction)
        self._estimator = RestingSocEstimator(
            self._curve_volts, self._curve_fraction,
            rest_speed_counts=self.rest_speed_counts,
            rest_seconds=self.rest_seconds,
            estimate_period=self.estimate_period)

        self._voltage = None
        self._voltage_stamp = None
        self._percentage = math.nan
        self._logged_percentage = math.nan

        self._pub = self.create_publisher(BatteryState, 'battery', 10)
        self.create_subscription(String, 'roboclaw_status', self._on_status, 10)
        self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().info(
            'Estimating charge from resting voltage (%.1f V empty to %.1f V full); '
            'percentage stays NaN until the robot has been still for %.1f s'
            % (self._curve_volts[0], self._curve_volts[-1], self.rest_seconds))

    def _validate_curve(self, volts, fraction):
        try:
            return validate_curve(volts, fraction)
        except ValueError as exc:
            self.get_logger().error(
                '%s — falling back to the built-in 5s li-ion curve' % exc)
            return list(DEFAULT_CURVE_VOLTS), list(DEFAULT_CURVE_FRACTION)

    def _on_status(self, msg: String):
        try:
            status = json.loads(msg.data)
            raw = float(status['main_battery'])
            speed = max(abs(float(status['m1_speed'])), abs(float(status['m2_speed'])))
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(
                'Could not read the pack out of /roboclaw_status: %s' % exc,
                throttle_duration_sec=10.0)
            return

        now = self.get_clock().now()
        self._voltage = self.voltage_scale * raw + self.voltage_offset
        self._voltage_stamp = now

        if self._voltage < _MIN_PLAUSIBLE_VOLTS:
            return

        estimate = self._estimator.update(
            now.nanoseconds * 1e-9, self._voltage, speed)
        if estimate is not None:
            self._log_estimate(self._estimator.last_median, estimate)

    def _log_estimate(self, volts, estimate):
        self._percentage = estimate
        moved = math.isnan(self._logged_percentage) or \
            abs(estimate - self._logged_percentage) >= _LOG_DELTA_FRACTION
        message = 'Resting pack %.2f V -> %.0f%% charge' % (volts, 100.0 * estimate)
        if moved:
            self._logged_percentage = estimate
            self.get_logger().info(message)
        else:
            self.get_logger().debug(message)

    def _publish(self):
        if self._voltage is None:
            return
        now = self.get_clock().now()
        age = (now - self._voltage_stamp).nanoseconds * 1e-9
        if age > self.status_timeout:
            self.get_logger().warn(
                'No /roboclaw_status for %.1f s — is the driver running?' % age,
                throttle_duration_sec=10.0)
            return

        present = self._voltage >= _MIN_PLAUSIBLE_VOLTS

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.voltage = self._voltage
        # Left unset deliberately. The RoboClaw's current reading is not
        # believable (see the module docstring), its temperature sensors measure
        # the driver board rather than the pack, and there is no coulomb
        # counter, so charge and capacity cannot be known. Splitting the pack
        # voltage by five would invent per-cell data the hardware never reports.
        msg.temperature = math.nan
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = float(self.design_capacity)
        msg.percentage = self._percentage if present else math.nan
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        if not present:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        elif self._voltage <= self.critical_voltage:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.present = present
        self._pub.publish(msg)

        if not present:
            return
        if self._voltage <= self.critical_voltage:
            self.get_logger().error(
                'Pack at %.2f V, at or below the %.2f V critical threshold — stop '
                'driving and swap it. There is no BMS; the RoboClaw 16.0 V cutoff '
                'is all that is left' % (self._voltage, self.critical_voltage),
                throttle_duration_sec=10.0)
        elif self._voltage <= self.warn_voltage:
            self.get_logger().warn(
                'Pack at %.2f V, below the %.2f V warning threshold'
                % (self._voltage, self.warn_voltage),
                throttle_duration_sec=30.0)


def main(args=None):
    run_node(BatteryMonitor, args=args)


if __name__ == '__main__':
    main()
