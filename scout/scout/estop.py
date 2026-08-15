#!/usr/bin/env python3
"""Software e-stop: a twist_mux lock plus an active-brake burst.

Publishes std_msgs/Bool on the profile's estop topic at 5 Hz — the twist_mux
lock. Default False (disengaged). /estop/engage sets it true, so twist_mux
blocks every source below the lock priority, AND bursts zero Twists on
/cmd_vel_stop (which sits ABOVE the lock, priority 255) so the RoboClaw's
velocity loop actively holds zero for ~0.5 s instead of free-wheeling on the
deadman. /estop/release clears it.

The 5 Hz heartbeat makes the lock fail-safe once twist_mux's estop-lock timeout
is > 0 (flipped to 1.0 s in this same change): if this node dies, the lock goes
stale = engaged, so a dead e-stop node parks the robot rather than leaving it
drivable. There is still no hardware e-stop; S3 is free for one (see CLAUDE.md).
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from scout.robot_profile import load as _load_profile


class EStop(Node):
    """Latching software e-stop: lock heartbeat + active-brake burst."""

    def __init__(self):
        super().__init__('estop')
        prof = _load_profile()
        hz = float(prof['publish_hz'])

        self._engaged = False
        self._brake_ticks = 0
        self._brake_total = max(1, int(round(prof['stop_grace_s'] * hz)))

        self._lock_pub = self.create_publisher(Bool, prof['topic_estop'], 10)
        self._stop_pub = self.create_publisher(Twist, prof['topic_cmd_vel_stop'], 10)

        self.create_service(Trigger, 'estop/engage', self._on_engage)
        self.create_service(Trigger, 'estop/release', self._on_release)

        self.create_timer(0.2, self._publish_lock)       # 5 Hz lock heartbeat
        self.create_timer(1.0 / hz, self._brake_tick)     # active-brake burst

        self.get_logger().info(
            'E-stop up (disengaged): lock on %s, brake on %s'
            % (prof['topic_estop'], prof['topic_cmd_vel_stop']))

    def _on_engage(self, request, response):
        if not self._engaged:
            self._engaged = True
            self._brake_ticks = self._brake_total  # start the active-zero burst
            self._publish_lock()
        response.success = True
        response.message = 'e-stop ENGAGED'
        self.get_logger().warn(response.message)
        return response

    def _on_release(self, request, response):
        self._engaged = False
        self._brake_ticks = 0
        self._publish_lock()
        response.success = True
        response.message = 'e-stop released'
        self.get_logger().info(response.message)
        return response

    def _publish_lock(self):
        msg = Bool()
        msg.data = self._engaged
        self._lock_pub.publish(msg)

    def _brake_tick(self):
        # Zero on /cmd_vel_stop (priority 255) passes the engaged lock, so the
        # driver holds zero. Only for the burst window; then it goes stale and
        # the lock alone keeps everything out (deadman already at rest).
        if self._brake_ticks > 0:
            self._stop_pub.publish(Twist())
            self._brake_ticks -= 1


def main(args=None):
    rclpy.init(args=args)
    node = EStop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
