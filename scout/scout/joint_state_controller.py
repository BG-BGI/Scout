import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# Estimated from URDF inertia: ixx = 0.5 * m * r^2 -> r = sqrt(2 * 0.002528 / 0.661) ~= 0.0875 m
WHEEL_RADIUS = 0.0875
# From URDF joint origins: wheel x-offset = 0.10875 m per side
HALF_WHEEL_SEPARATION = 0.10875

# (joint_name, urdf_sign, is_right) — right wheels have rpy yaw=0, left wheels have yaw=pi,
# flipping their local X axis, so they need opposite signs to roll the same direction.
WHEEL_JOINTS = [
    ('wheel1_to_base',  1, True),   # right rear
    ('wheel2_to_base', -1, False),  # left rear
    ('wheel3_to_base', -1, False),  # left front
    ('wheel4_to_base',  1, True),   # right front
]

LIDAR_JOINT = 'lidar1_to_lidar2'
LIDAR_OMEGA = -2.0 * math.pi * 10.0  # 10 Hz spin

ALL_JOINTS = [name for name, _, _ in WHEEL_JOINTS] + [LIDAR_JOINT]


class JointStateController(Node):
    def __init__(self):
        super().__init__('joint_state_controller')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        self.right_omega = 0.0
        self.left_omega = 0.0
        self.positions = [0.0] * len(ALL_JOINTS)
        self.last_time = self.get_clock().now()

        self.create_timer(0.02, self._publish)  # 50 Hz

    def _cmd_vel_cb(self, msg: Twist):
        self.right_omega = (msg.linear.x - msg.angular.z * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS
        self.left_omega  = (msg.linear.x + msg.angular.z * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS

    def _publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        wheel_velocities = []
        for i, (_, sign, is_right) in enumerate(WHEEL_JOINTS):
            omega = self.right_omega if is_right else self.left_omega
            self.positions[i] += sign * omega * dt
            wheel_velocities.append(sign * omega)

        # lidar_idx = len(WHEEL_JOINTS)
        # self.positions[lidar_idx] += LIDAR_OMEGA * dt

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ALL_JOINTS
        js.position = list(self.positions)
        js.velocity = wheel_velocities + [0.0]

        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
