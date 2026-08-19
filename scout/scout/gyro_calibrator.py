import math

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from scout.node_util import run_node

# Defaults measured on this robot's D455 (BMI085) over a 30 s stationary run:
#   yaw-axis bias  -0.203 deg/s  (-0.00354 rad/s)
#   noise sd        0.152 deg/s  ( 0.00265 rad/s)  -> variance 7.0e-6 (rad/s)^2
# Uncorrected that bias integrates to -12.1 deg/min; subtracting it left 0.037 deg/min.
_DEFAULT_VARIANCE = 7.0e-6
# A stationary reading this far from the running bias is motion, not bias. 0.02 rad/s
# (1.1 deg/s) is ~5x the measured bias, so it gates motion without rejecting real drift.
_DEFAULT_STILLNESS = 0.02
# Only announce a refreshed bias when it actually moved by ~0.03 deg/s; otherwise the
# node would log every refresh window forever while the robot sits idle.
_LOG_DELTA = 0.0005


class GyroCalibrator(Node):
    """Remove the gyroscope's bias, and keep removing it as the sensor warms up.

    Startup: hold the robot still for `calibration_seconds`; the mean gyro reading
    over that window is the initial bias. Nothing is published until it closes, so a
    consumer that starts at the same moment never integrates uncorrected samples.

    Afterwards the bias is re-estimated whenever the robot is stationary. This is the
    point of the node: MEMS gyro bias moves with temperature, and the camera warms
    appreciably over its first several minutes, so a value captured once from a cold
    boot goes stale. Stationary stretches are accumulated and blended into the bias,
    which keeps yaw from creeping without ever needing a deliberate recalibration.

    Detecting stillness from the gyro itself is safe here rather than circular: the
    bias (~0.2 deg/s) sits far below the stillness threshold (~1.1 deg/s), so a
    biased-but-stationary sensor still reads as stationary.

    Subscribes 'imu_in', publishes the corrected stream on 'imu_out'.
    """

    def __init__(self):
        super().__init__('gyro_calibrator')
        self.calibration_seconds = self.declare_parameter(
            'calibration_seconds', 5.0).value
        self.stillness_threshold = self.declare_parameter(
            'stillness_threshold', _DEFAULT_STILLNESS).value
        # Stationary time that must accumulate before the bias is refreshed. Long
        # enough that the window mean is not dominated by noise.
        self.refresh_seconds = self.declare_parameter('refresh_seconds', 10.0).value
        # Fraction of each refresh window folded into the bias. Well below 1 so a
        # single odd window cannot yank the correction around.
        self.refresh_weight = self.declare_parameter('refresh_weight', 0.25).value
        self.angular_velocity_variance = self.declare_parameter(
            'angular_velocity_variance', _DEFAULT_VARIANCE).value
        # Every Nth corrected sample also goes out on 'imu_out_slow' (10 -> 20 Hz
        # from the 200 Hz stream). A python subscriber pays executor+deserialize
        # cost PER MESSAGE even if its callback discards most samples, so slow
        # consumers (tilt_monitor) read the decimated topic instead of the full one.
        self.slow_decimation = int(self.declare_parameter('slow_decimation', 10).value)

        self._bias = None            # None until the startup window closes
        self._sum = [0.0, 0.0, 0.0]  # accumulator, reused for startup and refresh
        self._count = 0
        self._window_start = None
        self._refreshes = 0
        self._slow_i = 0

        self._pub = self.create_publisher(Imu, 'imu_out', qos_profile_sensor_data)
        self._slow_pub = self.create_publisher(Imu, 'imu_out_slow',
                                               qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu_in', self._on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            'Calibrating gyro bias — hold the robot still for %.1f s '
            '(nothing is published until then)' % self.calibration_seconds)

    def _accumulate(self, g, now):
        if self._window_start is None:
            self._window_start = now
        self._sum[0] += g.x
        self._sum[1] += g.y
        self._sum[2] += g.z
        self._count += 1
        return (now - self._window_start).nanoseconds * 1e-9

    def _reset_window(self):
        self._sum = [0.0, 0.0, 0.0]
        self._count = 0
        self._window_start = None

    def _on_imu(self, msg: Imu):
        now = self.get_clock().now()
        g = msg.angular_velocity

        if self._bias is None:
            if self._accumulate(g, now) >= self.calibration_seconds:
                self._bias = [s / self._count for s in self._sum]
                self.get_logger().info(
                    'Gyro bias rad/s: x=%+.5f y=%+.5f z=%+.5f (%d samples) — '
                    'yaw axis will be refreshed whenever the robot is still'
                    % (*self._bias, self._count))
                if max(abs(b) for b in self._bias) > self.stillness_threshold:
                    self.get_logger().warn(
                        'Bias exceeds the stillness threshold (%.3f rad/s) — was the '
                        'robot actually stationary during calibration?'
                        % self.stillness_threshold)
                self._reset_window()
            return

        corrected = (g.x - self._bias[0], g.y - self._bias[1], g.z - self._bias[2])

        if max(abs(c) for c in corrected) < self.stillness_threshold:
            if self._accumulate(g, now) >= self.refresh_seconds:
                self._refresh_bias()
        elif self._count:
            # Any motion discards the partial window; a bias estimate is only
            # meaningful over an uninterrupted stationary stretch.
            self._reset_window()

        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = corrected
        v = self.angular_velocity_variance
        msg.angular_velocity_covariance = [v, 0.0, 0.0,
                                           0.0, v, 0.0,
                                           0.0, 0.0, v]
        # There is no orientation in this stream — the wrapper computes none, so the
        # field arrives as an all-zero (invalid) quaternion. sensor_msgs/Imu says to
        # advertise that with -1 as the first covariance entry, which makes a consumer
        # reject it instead of reading it as identity. Without this an EKF configured
        # to fuse yaw would quietly peg heading to zero rather than complain.
        msg.orientation_covariance[0] = -1.0
        self._pub.publish(msg)
        self._slow_i += 1
        if self._slow_i >= self.slow_decimation:
            self._slow_i = 0
            self._slow_pub.publish(msg)

    def _refresh_bias(self):
        window = [s / self._count for s in self._sum]
        previous = list(self._bias)
        self._bias = [b + self.refresh_weight * (w - b)
                      for b, w in zip(self._bias, window, strict=True)]
        self._refreshes += 1
        drift = max(abs(n - p) for n, p in zip(self._bias, previous, strict=True))
        message = ('Gyro bias refresh %d: x=%+.5f y=%+.5f z=%+.5f rad/s '
                   '(moved %.4f deg/s)'
                   % (self._refreshes, *self._bias, math.degrees(drift)))
        if drift > _LOG_DELTA:
            self.get_logger().info(message)
        else:
            self.get_logger().debug(message)
        self._reset_window()


def main(args=None):
    run_node(GyroCalibrator, args=args)


if __name__ == '__main__':
    main()
