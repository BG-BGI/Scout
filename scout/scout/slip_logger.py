"""Phase 0 instrument for the traction-control feature: log wheel-vs-gyro slip.

This is a MEASUREMENT tool, not a runtime node — it publishes nothing and
commands no motion. It rebuilds the class of bench instrument that was deleted
in 2026-07-30 (same footing as camera_health.py / gyro_calibrator), and exists
to characterise slip signatures before any control law is written.

What it measures, at `rate_hz`, into a CSV under the src bind mount:

  * enc_v, enc_w  -- forward speed and yaw rate the REAR encoders imply
                     (from /wheel_odom; the driver derived these from the only
                     two wired encoders via forward kinematics).
  * v_left, v_right -- per-side rear-wheel ground-implied speed, recovered by
                     INVERTING that same kinematics (v ± w*track/2). No
                     joint_states needed and no information lost: the driver
                     built v,w out of exactly these two numbers.
  * fused_w       -- yaw rate from the fused /odom. The EKF fuses yaw from the
                     gyro (encoder yaw is scrub-corrupted and deliberately not
                     fused), so this is the frame-correct ground truth for yaw.
  * gyro_w        -- -angular_velocity.y off /imu/data, the raw documented yaw
                     convention (ROS yaw rate = -gyro_y). Logged only as a
                     cross-check on fused_w; they should track.

Derived slip signals (the point of the whole exercise):

  * yaw_slip = enc_w - fused_w
        The observable slip axis. Nonzero even with good traction because a
        skid-steer always scrubs (baseline wheel/gyro 1.93 CCW / 1.60 CW), so
        interpret it against that known scrub curve, not against zero.

  * The per-side GROUND-SPEED ESTIMATOR under one-side slip. With yaw known
    from the gyro and the GRIPPING side's wheel speed ~= its ground speed, the
    slipping side's true ground speed falls out:
        left slips  (right grips):  vg_left  = v_right - fused_w*track
        right slips (left grips):   vg_right = v_left  + fused_w*track
    Both hypotheses are logged every row; the Phase 0 towel test tape-measures
    the slipping side's creep to confirm which estimator holds. This estimator
    is load-bearing for the heading-hold trim law, so it gets validated first.

Symmetric linear slip (both sides equally on ice) stays UNOBSERVABLE here --
the EKF's forward speed comes from the wheels, so /odom v is not independent of
enc_v. Nothing in this log can catch it; that needs lidar/visual odometry.
"""

import csv
import os
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from scout.node_util import run_node
from scout.robot_profile import resolve_config_dir

# Geometric track, matching roboclaw.yaml's wheel_separation. The instrument
# reads it as a parameter so a recalibration there does not silently desync the
# per-side reconstruction here.
_DEFAULT_TRACK = 0.278


class SlipLogger(Node):
    """Subscribe cmd/wheel_odom/odom/imu, log slip signals to CSV at a fixed rate."""

    def __init__(self):
        super().__init__('slip_logger')
        self.track = self.declare_parameter('track', _DEFAULT_TRACK).value
        self.rate_hz = self.declare_parameter('rate_hz', 50.0).value
        self.print_period = self.declare_parameter('print_period', 1.0).value
        cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel_out').value
        wheel_topic = self.declare_parameter('wheel_odom_topic', '/wheel_odom').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
        # Empty default -> slip_logs next to the resolved config dir (the
        # bind-mounted repo copy, so CSVs reach the host — SC6: the bind path
        # is owned by robot_profile, same policy as traction_monitor).
        log_dir = self.declare_parameter('log_dir', '').value
        if not log_dir:
            log_dir = os.path.join(
                os.path.dirname(resolve_config_dir()), 'slip_logs')

        # Latest-sample cache; the timer reads whatever is freshest. A slip run is
        # short and hand-driven, so last-value sampling is the right model (not a
        # message-synchronised filter).
        self._cmd = (0.0, 0.0)
        self._enc = (0.0, 0.0)          # (v, w) from /wheel_odom
        self._fused_w = 0.0             # yaw truth from /odom
        self._fused_v = 0.0
        self._gyro_w = 0.0              # -gyro_y cross-check
        self._gyro_raw = (0.0, 0.0, 0.0)
        self._seen = set()             # which streams have arrived (startup guard)

        os.makedirs(log_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self._path = os.path.join(log_dir, 'slip_%s.csv' % stamp)
        self._file = open(self._path, 'w', newline='')
        self._csv = csv.writer(self._file)
        self._csv.writerow([
            't', 'cmd_v', 'cmd_w', 'enc_v', 'enc_w', 'fused_v', 'fused_w',
            'gyro_w', 'gyro_raw_x', 'gyro_raw_y', 'gyro_raw_z',
            'v_left', 'v_right', 'yaw_slip', 'vg_left_if_left_slips',
            'vg_right_if_right_slips'])
        self._rows = 0
        self._t0 = self.get_clock().now()

        # cmd is whatever twist_mux forwarded (the command the robot actually got,
        # before the collision monitor's guard). Reliable QoS — it is a control
        # topic, not a sensor stream.
        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self.create_subscription(Odometry, wheel_topic, self._on_wheel, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        # /imu/data is best-effort (qos_profile_sensor_data) — a default reliable
        # subscription here receives NOTHING and warns only once at discovery.
        self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)

        self._last_print = self.get_clock().now()
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            'slip_logger: writing %s at %.0f Hz. Drive the robot; Ctrl-C to stop. '
            'Waiting for cmd/wheel_odom/odom/imu ...' % (self._path, self.rate_hz))

    def _on_cmd(self, m: Twist):
        self._cmd = (m.linear.x, m.angular.z)
        self._seen.add('cmd')

    def _on_wheel(self, m: Odometry):
        self._enc = (m.twist.twist.linear.x, m.twist.twist.angular.z)
        self._seen.add('wheel')

    def _on_odom(self, m: Odometry):
        self._fused_v = m.twist.twist.linear.x
        self._fused_w = m.twist.twist.angular.z
        self._seen.add('odom')

    def _on_imu(self, m: Imu):
        g = m.angular_velocity
        self._gyro_raw = (g.x, g.y, g.z)
        self._gyro_w = -g.y            # ROS yaw rate = -gyro_y (CLAUDE.md)
        self._seen.add('imu')

    def _tick(self):
        # Do not log until every stream is live, or the first rows are misleading
        # zeros that read as perfect tracking.
        if len(self._seen) < 4:
            return
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        cmd_v, cmd_w = self._cmd
        enc_v, enc_w = self._enc
        b = self.track
        v_left = enc_v - enc_w * b / 2.0
        v_right = enc_v + enc_w * b / 2.0
        yaw_slip = enc_w - self._fused_w
        vg_left_if_left_slips = v_right - self._fused_w * b
        vg_right_if_right_slips = v_left + self._fused_w * b

        self._csv.writerow(['%.4f' % x for x in (
            t, cmd_v, cmd_w, enc_v, enc_w, self._fused_v, self._fused_w,
            self._gyro_w, *self._gyro_raw, v_left, v_right, yaw_slip,
            vg_left_if_left_slips, vg_right_if_right_slips)])
        self._rows += 1

        now = self.get_clock().now()
        if (now - self._last_print).nanoseconds * 1e-9 >= self.print_period:
            self._last_print = now
            self._file.flush()
            self.get_logger().info(
                'enc v=%+.3f w=%+.3f | fused w=%+.3f (gyro %+.3f) | '
                'yaw_slip=%+.3f | vL=%+.3f vR=%+.3f'
                % (enc_v, enc_w, self._fused_w, self._gyro_w, yaw_slip,
                   v_left, v_right))

    def close(self):
        self._file.flush()
        self._file.close()
        self.get_logger().info(
            'slip_logger: wrote %d rows to %s' % (self._rows, self._path))


def main(args=None):
    run_node(SlipLogger, on_shutdown=lambda n: n.close(), args=args)


if __name__ == '__main__':
    main()
