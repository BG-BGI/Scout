"""Abort navigation if the chassis tips past a threshold (accel = gravity).

The tilt math, LPF, spin gate and abort dwell all live in
scout.core.tilt.TiltTracker (tested off-ROS); this node is glue: feed IMU
samples, act on WARN/ABORT events.
On abort: latch, publish /tilt_alarm, pause explore_lite, cancel BOTH nav
actions via node_util.cancel_nav_goals (its own cancel_all_goals_async once
covered only navigate_to_pose — a through-poses route survived the abort).
"""

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from scout.core import tilt
from scout.node_util import cancel_nav_goals, make_cancel_clients, run_node

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
        # Per-sample smoothing: effective time constant scales with the input
        # rate (fed 20 Hz /imu/data_slow, alpha 0.2 -> tau ~0.25 s).
        self.lpf_alpha = float(self.declare_parameter('lpf_alpha', 0.2).value)

        self._tracker = tilt.TiltTracker(
            _LEVEL_AXIS, self.warn_deg, self.abort_deg, self.stillness_gyro,
            self.hold_s, self.lpf_alpha)

        self._alarm_pub = self.create_publisher(Bool, 'tilt_alarm', 10)
        self._resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self._cancel_clients = make_cancel_clients(self)
        self.create_subscription(Imu, 'imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info(
            'Tilt monitor: warn=%.1f deg abort=%.1f deg stillness=%.3f rad/s'
            % (self.warn_deg, self.abort_deg, self.stillness_gyro))

    def _heartbeat(self):
        if not self._tracker.latched:
            self._alarm_pub.publish(Bool(data=False))

    def _on_imu(self, msg: Imu):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        now_s = self.get_clock().now().nanoseconds * 1e-9
        event = self._tracker.update((a.x, a.y, a.z), (g.x, g.y, g.z), now_s)
        if event == tilt.WARN:
            self.get_logger().warn('Tilt %.1f deg (warn %.1f)' % (
                self._tracker.tilt_deg, self.warn_deg))
        elif event == tilt.ABORT:
            self._abort(self._tracker.tilt_deg)

    def _abort(self, tilt_deg):
        self.get_logger().error(
            'TILT ABORT %.1f deg — pausing explore and cancelling nav goals'
            % tilt_deg)
        self._alarm_pub.publish(Bool(data=True))
        self._resume_pub.publish(Bool(data=False))
        fired = cancel_nav_goals(self._cancel_clients)
        if not fired:
            self.get_logger().warn('no nav cancel server ready — cancel skipped')


def main(args=None):
    run_node(TiltMonitor, args=args)


if __name__ == '__main__':
    main()
