import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

from scout.core.gyro_bias import GyroBias
from scout.core.imu_cdr import ImuPatcher
from scout.node_util import run_node

# Defaults measured on this robot's D455 (BMI085) over a 30 s stationary run:
#   yaw-axis bias  -0.203 deg/s  (-0.00354 rad/s)
#   noise sd        0.152 deg/s  ( 0.00265 rad/s)  -> variance 7.0e-6 (rad/s)^2
# Uncorrected that bias integrates to -12.1 deg/min; subtracting it left 0.037 deg/min.
_DEFAULT_VARIANCE = 7.0e-6
# A stationary reading this far from the running bias is motion, not bias. 0.02 rad/s
# (1.1 deg/s) is ~5x the measured bias, so it gates motion without rejecting real drift.
_DEFAULT_STILLNESS = 0.02


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

        self._est = GyroBias(self.calibration_seconds, self.stillness_threshold,
                             self.refresh_seconds, self.refresh_weight,
                             log=self._log)
        self._patcher = ImuPatcher(self.angular_velocity_variance)
        self._slow_i = 0
        self._slow_n = self.slow_decimation
        self._monotonic_ns = time.monotonic_ns
        self._warned_fallback = False

        self._pub = self.create_publisher(Imu, 'imu_out', qos_profile_sensor_data)
        self._slow_pub = self.create_publisher(Imu, 'imu_out_slow',
                                               qos_profile_sensor_data)
        # raw=True: hand us the serialized CDR bytes. Profiled on the Pi, per-sample
        # deserialize + Imu construction + re-serialize was ~40% of this node's CPU
        # (200 Hz); only 3 + 9 + 1 doubles change, so they are patched in place.
        self.create_subscription(Imu, 'imu_in', self._on_imu_raw,
                                 qos_profile_sensor_data, raw=True)
        self.get_logger().info(
            'Calibrating gyro bias — hold the robot still for %.1f s '
            '(nothing is published until then)' % self.calibration_seconds)

    def _log(self, level, msg):
        getattr(self.get_logger(), level)(msg)

    def _publish(self, out):
        self._pub.publish(out)
        self._slow_i += 1
        if self._slow_i >= self._slow_n:
            self._slow_i = 0
            self._slow_pub.publish(out)

    def _on_imu_raw(self, raw):
        lay = self._patcher.layout(raw)
        if lay is None:
            self._on_imu_fallback(raw)
            return
        x, y, z = self._patcher.angular_velocity(raw, lay)
        c = self._est.update(x, y, z, self._monotonic_ns())
        if c is None:
            return
        self._publish(self._patcher.patch(raw, lay, *c))

    def _on_imu_fallback(self, raw):
        """Unrecognised CDR layout: the slow, always-correct object path."""
        if not self._warned_fallback:
            self._warned_fallback = True
            self.get_logger().warn('Imu CDR layout not recognised; using the '
                                   'slow deserialize path')
        msg = deserialize_message(bytes(raw), Imu)
        g = msg.angular_velocity
        c = self._est.update(g.x, g.y, g.z, self._monotonic_ns())
        if c is None:
            return
        g.x, g.y, g.z = c
        v = self.angular_velocity_variance
        msg.angular_velocity_covariance = [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]
        msg.orientation_covariance[0] = -1.0  # no orientation (sensor_msgs/Imu)
        self._publish(msg)


def main(args=None):
    run_node(GyroCalibrator, args=args)


if __name__ == '__main__':
    main()
