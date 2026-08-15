"""Abort navigation if the chassis tips past a threshold (accel = gravity).

Uses /imu/data linear acceleration (optical frame: level ≈ gravity on -Y).
Gyro rates gate evaluation so 2.5 rad/s pivots do not false-trip.
On abort: latch, publish /tilt_alarm, pause explore_lite, cancel NavigateToPose.
"""

import math

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from scout.node_util import run_node

# Optical frame (camera_imu_optical_frame): gravity on -Y when the chassis is level.
_LEVEL_AXIS = (0.0, -1.0, 0.0)


class TiltMonitor(Node):
    def __init__(self):
        super().__init__('tilt_monitor')
        self.warn_deg = float(self.declare_parameter('warn_deg', 8.0).value)
        self.abort_deg = float(self.declare_parameter('abort_deg', 15.0).value)
        self.stillness_gyro = float(
            self.declare_parameter('stillness_gyro', 0.08).value)
        self.hold_s = float(self.declare_parameter('hold_s', 0.5).value)
        self.lpf_alpha = float(self.declare_parameter('lpf_alpha', 0.2).value)

        self._tilt_lpf = None
        self._over_since = None
        self._latched = False
        self._warned = False

        self._alarm_pub = self.create_publisher(Bool, 'tilt_alarm', 10)
        self._resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Imu, 'imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info(
            'Tilt monitor: warn=%.1f deg abort=%.1f deg stillness=%.3f rad/s'
            % (self.warn_deg, self.abort_deg, self.stillness_gyro))

    def _heartbeat(self):
        if not self._latched:
            self._alarm_pub.publish(Bool(data=False))

    def _on_imu(self, msg: Imu):
        if self._latched:
            return

        g = msg.angular_velocity
        spinning = max(abs(g.x), abs(g.y), abs(g.z)) >= self.stillness_gyro

        a = msg.linear_acceleration
        mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        if mag < 1.0:
            return
        ux, uy, uz = a.x / mag, a.y / mag, a.z / mag
        lx, ly, lz = _LEVEL_AXIS
        dot = max(-1.0, min(1.0, ux * lx + uy * ly + uz * lz))
        tilt_deg = math.degrees(math.acos(dot))

        if self._tilt_lpf is None:
            self._tilt_lpf = tilt_deg
        else:
            self._tilt_lpf += self.lpf_alpha * (tilt_deg - self._tilt_lpf)

        if spinning:
            self._over_since = None
            return

        now = self.get_clock().now()
        if self._tilt_lpf >= self.warn_deg and not self._warned:
            self.get_logger().warn('Tilt %.1f deg (warn %.1f)' % (
                self._tilt_lpf, self.warn_deg))
            self._warned = True
        if self._tilt_lpf < self.warn_deg:
            self._warned = False

        if self._tilt_lpf < self.abort_deg:
            self._over_since = None
            return

        if self._over_since is None:
            self._over_since = now
            return
        held = (now - self._over_since).nanoseconds * 1e-9
        if held < self.hold_s:
            return

        self._abort(self._tilt_lpf)

    def _abort(self, tilt_deg):
        self._latched = True
        self.get_logger().error(
            'TILT ABORT %.1f deg — pausing explore and cancelling NavigateToPose'
            % tilt_deg)
        self._alarm_pub.publish(Bool(data=True))
        self._resume_pub.publish(Bool(data=False))
        if self._nav_client.server_is_ready():
            self._nav_client.cancel_all_goals_async()
        else:
            self.get_logger().warn(
                'navigate_to_pose action server not ready — cancel skipped')


def main(args=None):
    run_node(TiltMonitor, args=args)


if __name__ == '__main__':
    main()
