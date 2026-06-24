import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from scout.dual_g2_hpmd_rpi import motors, MAX_SPEED

WATCHDOG_TIMEOUT = 0.5   # seconds
# Robot's wheel angular speed at full motor throttle, used to scale cmd_vel m/s to motor
# throttle. Calibrate it: drive at full throttle over a known distance, then
# MAX_WHEEL_RAD_S = (distance / time) / WHEEL_RADIUS.
MAX_WHEEL_RAD_S = 6.4

# Drive geometry (from urdf/robot.urdf): wheel joints at x = +/-0.10875; radius from the
# wheel inertia about its spin axis, r = sqrt(2 * ixx / m) = sqrt(2 * 0.0025279 / 0.66136).
WHEEL_RADIUS = 0.0875            # m
WHEEL_SEPARATION = 0.2175        # m  (track width = 2 * 0.10875)
MAX_LINEAR_SPEED = MAX_WHEEL_RAD_S * WHEEL_RADIUS  # m/s at full throttle (~0.56)

# Skid-steer slip/friction: all 4 tires must skid sideways to rotate, so the geometric
# track width badly under-drives turns. This multiplies the turn command to compensate.
# Higher = sharper turns. Turn saturates around omega = MAX_LINEAR_SPEED / (WHEEL_SEPARATION/2 * TURN_GAIN).
TURN_GAIN = 4.0


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.watchdog = self.create_timer(WATCHDOG_TIMEOUT, self.on_watchdog_timeout)
        self._moving = False
        self.get_logger().info('Motor driver node started')

    def on_cmd_vel(self, msg):
        # Differential drive: body twist (m/s, rad/s) -> per-wheel linear speed (m/s).
        # TURN_GAIN inflates the effective track width to overcome skid-steer slip/friction.
        turn = msg.angular.z * (WHEEL_SEPARATION / 2.0) * TURN_GAIN
        v_left = msg.linear.x - turn
        v_right = msg.linear.x + turn

        # Normalize to motor throttle [-1, 1]; negated for this drive's wiring polarity
        left = -max(-1.0, min(1.0, v_left / MAX_LINEAR_SPEED))
        right = -max(-1.0, min(1.0, v_right / MAX_LINEAR_SPEED))

        motors.setSpeeds(int(left * MAX_SPEED), int(right * MAX_SPEED))
        self._moving = True
        self.watchdog.reset()

    def on_watchdog_timeout(self):
        if self._moving:
            motors.setSpeeds(0, 0)
            # self.get_logger().warn('cmd_vel watchdog timeout — motors stopped')
            self._moving = False


def main():
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        motors.forceStop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
