"""Per-side traction derate from RoboClaw current-vs-speed (traction spec).

The verdict ladder, derate walk, curve validation and Twist<->side split live
in scout.core.traction (tested off-ROS); the wire format in
scout.core.status. This node is I/O glue.

Left/right are single paralleled channels with only the rear encoder wired per
side, and the observed fault is consistently FRONT-wheel-only — so the rear
encoder keeps tracking true wheel speed and measured speed stays trustworthy.
An unloaded front wheel shows up as a side drawing LESS current than the
loaded baseline at the same measured speed (the free wheel carries none of
the propulsion torque). Detection is therefore: apparent load =
current / |measured_speed| against a calibrated expected_current(speed) curve.

Two jobs in one node (spec actuation option (a)):

  * MONITOR — parse /roboclaw_status (JSON String, 10 Hz), compare each
    channel's current against its calibrated curve, and walk a per-side
    derate factor in [derate_floor, 1.0] (fast down, slow up — asymmetric so
    PID current blips don't chatter it back to full speed).
  * ACTUATION — sole writer of the driver's cmd_vel: subscribes the final
    twist_mux output, splits the Twist into per-side wheel speeds
    (v -/+ w*track/2), scales each side by its derate, recomposes and
    republishes. Both sides at 1.0 = byte-identical passthrough. If this
    node dies the driver hears nothing and the deadman coasts (fail-safe;
    the launch also fail-fasts on it as a motion-chain member).

Gating: current telemetry is noise below ~20% duty (CLAUDE.md), so below
`gate_counts` measured speed there is NO VERDICT — the derate decays back
toward 1.0 rather than snapping (a derated side can fall below the gate; an
instant reset there would oscillate trip/reset).

UNCALIBRATED BY DEFAULT: the expected-current curves ship empty and the node
then never derates (passthrough + status only). Collect the baseline first —
set `calibration_log: true`, drive loaded and with one front wheel propped at
2-3 speeds >= the gate, and fit the curves from the CSV into traction.yaml.
Do not guess the curves; the spec forbids shipping default margins.
"""

import csv
import os
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String

from scout.core import traction
from scout.core.status import format_traction_status, parse_roboclaw_status
from scout.node_util import run_node
from scout.robot_profile import resolve_config_dir

# Geometric track, matching roboclaw.yaml's wheel_separation.
_DEFAULT_TRACK = 0.278


class TractionMonitor(Node):
    def __init__(self):
        super().__init__('traction_monitor')
        self.track = float(self.declare_parameter('track', _DEFAULT_TRACK).value)
        # ~20% of the 9240 QPPS limit — the current-telemetry noise floor.
        self.gate_counts = float(self.declare_parameter('gate_counts', 1850.0).value)
        # Flag when current falls this fraction below the expected curve.
        self.margin = float(self.declare_parameter('margin', 0.35).value)
        self.derate_floor = float(self.declare_parameter('derate_floor', 0.4).value)
        # Per 10 Hz status tick: full derate in ~0.4 s, recovery in ~3 s.
        self.step_down = float(self.declare_parameter('step_down', 0.15).value)
        self.step_up = float(self.declare_parameter('step_up', 0.02).value)
        self.status_timeout = float(
            self.declare_parameter('status_timeout', 1.0).value)
        # Which side channel M1 drives. The driver publishes joint_states as
        # left/right but nothing in this repo pins the channel mapping —
        # verify during calibration (prop the LEFT front wheel, see which
        # channel's current drops) and fix here if wrong.
        self.m1_side = str(self.declare_parameter('m1_side', 'left').value)

        # Calibration: expected loaded-baseline current (A) per channel at the
        # given measured speeds (counts/s). Empty = uncalibrated = no derating.
        speeds = list(self.declare_parameter('curve_speeds', [0.0]).value)
        m1_amps = list(self.declare_parameter('m1_curve_amps', [0.0]).value)
        m2_amps = list(self.declare_parameter('m2_curve_amps', [0.0]).value)
        self._curves, err = traction.validate_curves(speeds, m1_amps, m2_amps)
        if err:
            self.get_logger().error(err)

        self._derate = {'m1': 1.0, 'm2': 1.0}
        self._verdict = {'m1': traction.NO_DATA, 'm2': traction.NO_DATA}
        self._status_stamp = None

        self._log_file = None
        self._log_csv = None
        if bool(self.declare_parameter('calibration_log', False).value):
            # Empty default -> traction_logs next to the resolved config dir
            # (the bind-mounted repo copy in the container, so CSVs reach the
            # host — same policy as maps/, owned by robot_profile per SC6).
            log_dir = str(self.declare_parameter('log_dir', '').value)
            if not log_dir:
                log_dir = os.path.join(
                    os.path.dirname(resolve_config_dir()), 'traction_logs')
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(
                log_dir, 'traction_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
            self._log_file = open(path, 'w', newline='')
            self._log_csv = csv.writer(self._log_file)
            self._log_csv.writerow(
                ['t', 'm1_speed', 'm1_current', 'm2_speed', 'm2_current'])
            self.get_logger().info('Calibration log: %s' % path)
        self._t0 = self.get_clock().now()

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self._left_pub = self.create_publisher(Float32, 'traction/derate_left', 10)
        self._right_pub = self.create_publisher(Float32, 'traction/derate_right', 10)
        self._status_pub = self.create_publisher(String, 'traction/status', 10)
        self.create_subscription(Twist, 'cmd_vel_in', self._on_cmd, 10)
        self.create_subscription(String, 'roboclaw_status', self._on_status, 10)
        self.create_timer(1.0, self._check_stale)

        if self._curves is None:
            self.get_logger().warn(
                'UNCALIBRATED — no expected-current curves in traction.yaml; '
                'passthrough + status only, no derating. Run the calibration '
                'drive (calibration_log: true) and fill the curves.')
        else:
            self.get_logger().info(
                'Traction monitor: gate %.0f counts/s, margin %.0f%%, '
                'floor %.2f, m1=%s' % (self.gate_counts, 100.0 * self.margin,
                                       self.derate_floor, self.m1_side))

    # --- actuation: per-side scaled passthrough -------------------------

    def _on_cmd(self, msg: Twist):
        dl, dr = self._side_derates()
        if dl >= 1.0 and dr >= 1.0:
            self._cmd_pub.publish(msg)
            return
        v_left, v_right = traction.split_sides(
            msg.linear.x, msg.angular.z, self.track)
        out = Twist()
        out.linear.x, out.angular.z = traction.merge_sides(
            v_left * dl, v_right * dr, self.track)
        self._cmd_pub.publish(out)

    def _side_derates(self):
        if self.m1_side == 'left':
            return self._derate['m1'], self._derate['m2']
        return self._derate['m2'], self._derate['m1']

    # --- monitor: verdict per status tick --------------------------------

    def _on_status(self, msg: String):
        try:
            status = parse_roboclaw_status(msg.data)
            sample = {ch: (abs(float(status['%s_speed' % ch])),
                           abs(float(status['%s_current' % ch])))
                      for ch in ('m1', 'm2')}
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(
                'Could not parse /roboclaw_status: %s' % exc,
                throttle_duration_sec=10.0)
            return
        self._status_stamp = self.get_clock().now()

        if self._log_csv is not None:
            t = (self._status_stamp - self._t0).nanoseconds * 1e-9
            self._log_csv.writerow(['%.3f' % t,
                                    '%.1f' % sample['m1'][0],
                                    '%.3f' % sample['m1'][1],
                                    '%.1f' % sample['m2'][0],
                                    '%.3f' % sample['m2'][1]])

        chans = {}
        for ch, (speed, current) in sample.items():
            verdict, expected = traction.channel_verdict(
                speed, current,
                None if self._curves is None else self._curves[ch],
                self.gate_counts, self.margin)
            self._verdict[ch] = verdict
            if verdict != traction.UNCALIBRATED:
                self._derate[ch] = traction.step_derate(
                    self._derate[ch], verdict, self.step_up, self.step_down,
                    self.derate_floor)
            chans[ch] = {
                'speed': speed,
                'current': current,
                'expected': expected,
                'verdict': self._verdict[ch],
                'derate': self._derate[ch],
            }

        dl, dr = self._side_derates()
        self._left_pub.publish(Float32(data=dl))
        self._right_pub.publish(Float32(data=dr))
        self._status_pub.publish(String(data=format_traction_status(
            chans['m1'], chans['m2'],
            'm1' if self.m1_side == 'left' else 'm2')))

        if min(dl, dr) < 1.0:
            self.get_logger().warn(
                'Traction derate L=%.2f R=%.2f (m1 %s, m2 %s)'
                % (dl, dr, self._verdict['m1'], self._verdict['m2']),
                throttle_duration_sec=2.0)

    def _check_stale(self):
        if self._status_stamp is None:
            return
        age = (self.get_clock().now() - self._status_stamp).nanoseconds * 1e-9
        if age > self.status_timeout and (
                self._derate['m1'] < 1.0 or self._derate['m2'] < 1.0):
            # Stale telemetry gives no verdict; fail open rather than holding
            # the robot throttled on dead data (derate is performance, not
            # safety — the collision monitor and estop own safety).
            self.get_logger().warn(
                'No /roboclaw_status for %.1f s — releasing derates' % age)
            self._derate = {'m1': 1.0, 'm2': 1.0}
            self._verdict = {'m1': traction.NO_DATA, 'm2': traction.NO_DATA}

    def close(self):
        if self._log_file is not None:
            self._log_file.flush()
            self._log_file.close()


def main(args=None):
    run_node(TractionMonitor, on_shutdown=lambda n: n.close(), args=args)


if __name__ == '__main__':
    main()
