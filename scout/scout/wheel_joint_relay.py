#!/usr/bin/env python3
"""Drive the URDF's four wheel joints from the driver's two encoder joints.

roboclaw_driver publishes /joint_states as `left_wheel_joint` and
`right_wheel_joint`, because it closes on one encoder per side (the two motors
on a side are paralleled on one RoboClaw channel and only the rear encoders are
wired). The URDF names four joints instead, so without a translation the
driver's real wheel angles are discarded and the wheels sit frozen.

Both sides report increasing angle when the robot drives forward, but the URDF's
right-hand joints carry a yaw of pi in their origins, which flips their axle
direction: a positive angle rolls wheel1/wheel4 (left) forward and wheel2/wheel3
(right) backward. So the right side is negated here. This is derived from the
joint origins rather than observed, so watch the wheels roll the right way the
first time the robot drives.

Front wheel angles are a stand-in, not a measurement: the front encoders are not
wired, so each front wheel is drawn at its rear neighbour's angle. That is
exactly why the front-left drive fault is invisible to the driver, and it stays
invisible here — the render will happily spin a wheel that is not turning.

This publishes on the same topic it subscribes to. That is safe because the two
name sets are disjoint: its own output carries none of the names it acts on.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Driver joint -> the URDF joints it drives, with the sign that makes a positive
# driver angle roll that wheel forward. Left is (front, rear) = wheel1, wheel4;
# note the rear-left joint is named `base_to_wheel`, an exporter quirk.
_LEFT_JOINTS = ('base_to_wheel1', 'base_to_wheel')
_RIGHT_JOINTS = ('base_to_wheel2', 'base_to_wheel3')
_LEFT_SIGN = 1.0
_RIGHT_SIGN = -1.0

_LEFT_SOURCE = 'left_wheel_joint'
_RIGHT_SOURCE = 'right_wheel_joint'


class WheelJointRelay(Node):
    """Map left/right_wheel_joint onto the four wheel joints in the URDF."""

    def __init__(self):
        super().__init__('wheel_joint_relay')

        # Matching the driver's own 30 Hz joint_states rate buys nothing; this
        # only feeds a visualisation.
        self.publish_rate = max(1.0, self.declare_parameter('publish_rate', 30.0).value)
        # Below this the driver is assumed gone and the wheels are held at their
        # last angle rather than drifting on a stale velocity.
        self.source_timeout = self.declare_parameter('source_timeout', 2.0).value

        self._names = list(_LEFT_JOINTS) + list(_RIGHT_JOINTS)
        self._signs = [_LEFT_SIGN] * len(_LEFT_JOINTS) + [_RIGHT_SIGN] * len(_RIGHT_JOINTS)
        # Zeros until the driver speaks, so the wheels render with it stopped.
        self._position = {_LEFT_SOURCE: 0.0, _RIGHT_SOURCE: 0.0}
        self._velocity = {_LEFT_SOURCE: 0.0, _RIGHT_SOURCE: 0.0}
        self._source_stamp = None

        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)
        self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().info(
            'Relaying %s/%s onto %s (right side negated); publishing zeros until the '
            'driver is heard' % (_LEFT_SOURCE, _RIGHT_SOURCE, ', '.join(self._names)))

    def _on_joint_states(self, msg):
        seen = False
        for i, name in enumerate(msg.name):
            if name not in self._position:
                continue
            seen = True
            if i < len(msg.position):
                self._position[name] = msg.position[i]
            if i < len(msg.velocity):
                self._velocity[name] = msg.velocity[i]
        if seen:
            if self._source_stamp is None:
                self.get_logger().info('Driver joint states received; wheels are live')
            self._source_stamp = self.get_clock().now()

    def _publish(self):
        if self._source_stamp is not None:
            age = (self.get_clock().now() - self._source_stamp).nanoseconds * 1e-9
            if age > self.source_timeout:
                # Hold the angle, but stop claiming the wheels are still turning.
                self._velocity = dict.fromkeys(self._velocity, 0.0)

        left_p, right_p = self._position[_LEFT_SOURCE], self._position[_RIGHT_SOURCE]
        left_v, right_v = self._velocity[_LEFT_SOURCE], self._velocity[_RIGHT_SOURCE]
        source_position = [left_p] * len(_LEFT_JOINTS) + [right_p] * len(_RIGHT_JOINTS)
        source_velocity = [left_v] * len(_LEFT_JOINTS) + [right_v] * len(_RIGHT_JOINTS)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._names)
        msg.position = [s * p for s, p in zip(self._signs, source_position)]
        msg.velocity = [s * v for s, v in zip(self._signs, source_velocity)]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelJointRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
