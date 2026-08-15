"""Link-loss watchdog: pause nav goals when the operator link dies, resume
when it returns, forget after a deadline.

Born 2026-08-14: the robot drove into a WiFi dead zone with a NavigateToPose
goal latched. bt_navigator replans at 1 Hz and streams cmd_vel entirely
on-robot, so losing the network removes every software stop — the goal ran
until a human pulled the battery.

Policy (operator-specified):
  * link down `pause_after_s` (5 s)   -> cancel active nav goals, STASH them
  * link back within `forget_after_s` -> re-dispatch the stashed goal
  * link down `forget_after_s` (120 s)-> drop the stash; robot stays parked

"Pause" is implemented as cancel-plus-stash because Nav2 has no native pause:
canceling stops motion within a control cycle, and the stash lets recovery
look like a resume. Goals are captured from the wire — /goal_pose covers
every NavigateToPose client (Foxglove, webui, scout-skills go_to), and
/route_poses is published by scout-skills' go_through/patrol specifically so
this node can re-send a multi-point route (action goals are not otherwise
observable).

Link probe: TCP connect to the default gateway (port 80). A RST/refused
counts as UP — reachability is the question, not an open port. No ping
binary needed (ros:humble-ros-core ships none). The gateway is re-read from
`ip route` periodically so hotspot/corp handoffs keep working;
`gateway_override` exists mainly so a test can fake a dead link:
  ros2 param set /link_watchdog gateway_override 10.255.255.1
"""

import socket
import subprocess
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node

from scout.node_util import run_node
from scout.qos import LATCHED_QOS

NAV_ACTIONS = ('navigate_to_pose', 'navigate_through_poses')
# action_msgs/GoalStatus: STATUS_ACCEPTED=1, STATUS_EXECUTING=2.
ACTIVE_STATUSES = (1, 2)

# Action status topics are reliable + transient_local; a volatile subscriber
# would never see the latched last message after a restart.
STATUS_QOS = LATCHED_QOS


def _default_gateway() -> str | None:
    try:
        out = subprocess.run(
            ['ip', 'route', 'show', 'default'],
            capture_output=True, text=True, timeout=2.0,
        ).stdout.split()
        return out[out.index('via') + 1] if 'via' in out else None
    except Exception:  # noqa: BLE001 — any failure (no ip, no route, timeout) = no gateway
        return None


class LinkWatchdog(Node):

    def __init__(self):
        super().__init__('link_watchdog')
        self.declare_parameter('check_period_s', 1.0)
        self.declare_parameter('pause_after_s', 5.0)
        self.declare_parameter('forget_after_s', 120.0)
        self.declare_parameter('probe_timeout_s', 1.0)
        self.declare_parameter('gateway_refresh_s', 60.0)
        # Non-empty value replaces the auto-detected gateway (test hook).
        self.declare_parameter('gateway_override', '')

        self._gateway: str | None = None
        self._gateway_read_at = 0.0
        self._down_since: float | None = None
        self._paused = False

        # Latest goal seen per kind; whichever action is ACTIVE when the link
        # dies decides which stash is kept.
        self._last_pose: PoseStamped | None = None
        self._last_route: PoseArray | None = None
        self._stash_pose: PoseStamped | None = None
        self._stash_route: PoseArray | None = None
        self._active: dict[str, bool] = {a: False for a in NAV_ACTIONS}

        self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal_pose, 10)
        self.create_subscription(
            PoseArray, '/route_poses', self._on_route, 10)
        for action in NAV_ACTIONS:
            self.create_subscription(
                GoalStatusArray, '/%s/_action/status' % action,
                lambda msg, a=action: self._on_status(a, msg), STATUS_QOS)

        self._cancel_clients = {
            a: self.create_client(CancelGoal, '/%s/_action/cancel_goal' % a)
            for a in NAV_ACTIONS
        }
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._through_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')

        period = self.get_parameter('check_period_s').value
        self.create_timer(period, self._tick)
        self.get_logger().info(
            'link watchdog up: pause %.0fs, forget %.0fs'
            % (self.get_parameter('pause_after_s').value,
               self.get_parameter('forget_after_s').value))

    # --- wire taps -----------------------------------------------------------

    def _on_goal_pose(self, msg: PoseStamped):
        self._last_pose = msg

    def _on_route(self, msg: PoseArray):
        self._last_route = msg

    def _on_status(self, action: str, msg):
        self._active[action] = bool(
            msg.status_list and msg.status_list[-1].status in ACTIVE_STATUSES)

    # --- link probe ----------------------------------------------------------

    def _probe_target(self) -> str | None:
        override = self.get_parameter('gateway_override').value
        if override:
            return override
        now = time.monotonic()
        refresh = self.get_parameter('gateway_refresh_s').value
        if self._gateway is None or now - self._gateway_read_at > refresh:
            gw = _default_gateway()
            if gw:
                self._gateway = gw
            self._gateway_read_at = now
        return self._gateway

    def _link_up(self) -> bool:
        target = self._probe_target()
        if target is None:
            return False  # no default route at all
        timeout = self.get_parameter('probe_timeout_s').value
        try:
            socket.create_connection((target, 80), timeout=timeout).close()
            return True
        except ConnectionRefusedError:
            return True  # RST came back: host reachable, port closed
        except OSError:
            return False

    # --- state machine -------------------------------------------------------

    def _tick(self):
        now = time.monotonic()
        if self._link_up():
            if self._down_since is not None:
                self.get_logger().info(
                    'link restored after %.0f s' % (now - self._down_since))
            self._down_since = None
            if self._paused:
                self._resume()
            return

        if self._down_since is None:
            self._down_since = now
            return
        down_for = now - self._down_since

        if not self._paused and down_for >= self.get_parameter('pause_after_s').value:
            self._pause()
        if self._paused and down_for >= self.get_parameter('forget_after_s').value:
            if self._stash_pose or self._stash_route:
                self.get_logger().warn(
                    'link down %.0f s — dropping stashed goal' % down_for)
            self._stash_pose = self._stash_route = None

    def _pause(self):
        self._paused = True
        # Stash by whichever action is live right now (through-poses wins if
        # both report active — it is the newer dispatch style).
        self._stash_pose = self._stash_route = None
        if self._active['navigate_through_poses'] and self._last_route:
            self._stash_route = self._last_route
        elif self._active['navigate_to_pose'] and self._last_pose:
            self._stash_pose = self._last_pose
        had_goal = any(self._active.values())
        for action, client in self._cancel_clients.items():
            if not self._active[action]:
                continue
            if client.service_is_ready():
                req = CancelGoal.Request()  # zeroed uuid = cancel all
                client.call_async(req)
        self.get_logger().warn(
            'link down — paused nav (%s)'
            % ('goal stashed for resume' if had_goal else 'no active goal'))

    def _resume(self):
        self._paused = False
        if self._stash_route is not None:
            goal = NavigateThroughPoses.Goal()
            for pose in self._stash_route.poses:
                ps = PoseStamped()
                ps.header = self._stash_route.header
                ps.pose = pose
                goal.poses.append(ps)
            # Fire and forget: bt_navigator owns it from here, same contract
            # as every other client on this robot.
            self._through_client.send_goal_async(goal)
            self.get_logger().info(
                'link back — re-dispatched %d-pose route' % len(goal.poses))
        elif self._stash_pose is not None:
            self._goal_pub.publish(self._stash_pose)
            self.get_logger().info('link back — re-dispatched goal pose')
        else:
            self.get_logger().info('link back — nothing stashed to resume')
        self._stash_pose = self._stash_route = None


def main(args=None):
    # run_node adds the rclpy.shutdown() this node used to skip.
    run_node(LinkWatchdog, args=args)


if __name__ == '__main__':
    main()
