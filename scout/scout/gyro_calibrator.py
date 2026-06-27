import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# A stationary gyro bias above this (rad/s) almost certainly means the robot
# was moving during calibration.
_STILLNESS_WARN = 0.1


class GyroCalibrator(Node):
    """Remove the gyroscope's residual bias using a stationary startup window.

    Hold the robot still for `calibration_seconds` after launch: the mean gyro
    reading over that window is the bias, which is then subtracted from
    angular_velocity on every subsequent message. This is the per-boot
    'calibrate_gyros' step that keeps yaw from creeping while stationary.

    Subscribes 'imu_in', publishes the corrected stream on 'imu_out'.
    """

    def __init__(self):
        super().__init__('gyro_calibrator')
        self.calibration_seconds = self.declare_parameter(
            'calibration_seconds', 5.0).value

        self._sum = [0.0, 0.0, 0.0]
        self._count = 0
        self._start = None
        self._bias = None  # None until the window closes

        self._pub = self.create_publisher(Imu, 'imu_out', qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu_in', self._on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            'Calibrating gyro bias — hold the robot still for %.1f s'
            % self.calibration_seconds)

    def _on_imu(self, msg: Imu):
        if self._bias is None:
            now = self.get_clock().now()
            if self._start is None:
                self._start = now
            g = msg.angular_velocity
            self._sum[0] += g.x
            self._sum[1] += g.y
            self._sum[2] += g.z
            self._count += 1
            if (now - self._start).nanoseconds * 1e-9 >= self.calibration_seconds:
                self._bias = [s / self._count for s in self._sum]
                self.get_logger().info(
                    'Gyro bias rad/s: x=%.5f y=%.5f z=%.5f (%d samples)'
                    % (*self._bias, self._count))
                if max(abs(b) for b in self._bias) > _STILLNESS_WARN:
                    self.get_logger().warn(
                        'Large bias — was the robot actually still during calibration?')
            self._pub.publish(msg)  # pass through while calibrating (still anyway)
            return

        msg.angular_velocity.x -= self._bias[0]
        msg.angular_velocity.y -= self._bias[1]
        msg.angular_velocity.z -= self._bias[2]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GyroCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
