#!/usr/bin/env python3
"""Operator-grade nav cancel + consolidated /nav_state feedback (ADR-0018).

Two documented sharp edges: "goal failed ≠ robot stops" (behaviors already
dispatched keep running) and "a goal survives its client dying" — yet the only
cancel paths were the skills `nav_cancel` tool and a compose restart, and no
surface showed nav progress at all.

/nav/cancel (Trigger) is DISPATCHER-AWARE: an action cancel alone gets
re-overridden within a second — patrol_capture advances to its next waypoint
on goal end, explore re-dispatches frontier goals continuously. So the order
is: stop the dispatchers first (/patrol/stop, /explore/resume false), then
zeroed-uuid cancel BOTH bt_navigator actions (the shared
node_util.cancel_nav_goals — same plumbing as link_watchdog). All async: a
sync client call inside this service callback would deadlock the executor
silently (SC11). This cancels NAV only — the robot stays drivable; the webui
STOP/E-STOP remain the hard paths.

/nav_state (latched String, SC9 grammar in scout.core.status):
    'idle' | '<status_name>|<dist 2dp or empty>|<recoveries>'
Status names come from robot_profile's goal_status_names (indexed by
action_msgs/GoalStatus code); distance/recoveries from whichever action's
feedback is live. Consolidates navigate_to_pose AND navigate_through_poses,
so the webui shows progress for taps, routes and patrols alike.
"""

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from scout.core.status import format_nav_state
from scout.node_util import cancel_nav_goals, run_node
from scout.qos import LATCHED_QOS
from scout.robot_profile import load as _load_profile

NAV_ACTIONS = ('navigate_to_pose', 'navigate_through_poses')
FEEDBACK_TYPES = {
    'navigate_to_pose': NavigateToPose.Impl.FeedbackMessage,
    'navigate_through_poses': NavigateThroughPoses.Impl.FeedbackMessage,
}
# action_msgs/GoalStatus: accepted/executing/canceling are "live".
ACTIVE_STATUSES = (1, 2, 3)


class NavManager(Node):
    """Consolidate nav feedback onto /nav_state; cancel everything on request."""

    def __init__(self):
        super().__init__('nav_manager')
        # Index = GoalStatus code (0 unknown … 6 aborted) — the same friendly
        # names every UI already shows.
        self._status_names = list(_load_profile()['goal_status_names'])

        # Per action: last status code, feedback numbers, and a recency seq so
        # the consolidated view follows whichever action moved last.
        self._st = {a: {'code': 0, 'dist': None, 'recov': 0, 'seq': 0}
                    for a in NAV_ACTIONS}
        self._seq = 0
        self._last_wire = None

        for action in NAV_ACTIONS:
            # Action status topics are reliable + transient_local.
            self.create_subscription(
                GoalStatusArray, '/%s/_action/status' % action,
                lambda msg, a=action: self._on_status(a, msg), LATCHED_QOS)
            self.create_subscription(
                FEEDBACK_TYPES[action], '/%s/_action/feedback' % action,
                lambda msg, a=action: self._on_feedback(a, msg), 10)

        self._cancel_clients = {
            a: self.create_client(CancelGoal, '/%s/_action/cancel_goal' % a)
            for a in NAV_ACTIONS
        }
        self._patrol_stop = self.create_client(Trigger, '/patrol/stop')
        # False = pause. tilt_monitor publishes here too (multiple publishers
        # on a Bool topic are fine); with no explore node up it's just dropped.
        self._explore_pause = self.create_publisher(
            Bool, '/explore/resume', 10)

        self._state_pub = self.create_publisher(
            String, '/nav_state', LATCHED_QOS)
        self.create_service(Trigger, '/nav/cancel', self._on_cancel)
        self._publish_state()
        self.get_logger().info('nav_manager up: /nav/cancel + /nav_state')

    # --- consolidated state ------------------------------------------------------

    def _on_status(self, action, msg):
        if not msg.status_list:
            return
        code = msg.status_list[-1].status
        st = self._st[action]
        if code != st['code']:
            st['code'] = code
            self._seq += 1
            st['seq'] = self._seq
            if code not in ACTIVE_STATUSES:
                # Terminal: the next goal's feedback starts fresh.
                st['dist'] = None if code == 0 else st['dist']
        self._publish_state()

    def _on_feedback(self, action, msg):
        fb = msg.feedback
        st = self._st[action]
        st['dist'] = fb.distance_remaining
        st['recov'] = fb.number_of_recoveries
        self._publish_state()

    def _current(self):
        """The action to display: a live one wins; else the most recent
        terminal one; else None (never dispatched -> idle)."""
        live = [a for a in NAV_ACTIONS
                if self._st[a]['code'] in ACTIVE_STATUSES]
        pool = live or [a for a in NAV_ACTIONS if self._st[a]['seq']]
        if not pool:
            return None
        return max(pool, key=lambda a: self._st[a]['seq'])

    def _publish_state(self):
        action = self._current()
        if action is None:
            wire = format_nav_state('idle')
        else:
            st = self._st[action]
            wire = format_nav_state(
                self._status_names[st['code']], st['dist'], st['recov'])
        if wire != self._last_wire:
            self._last_wire = wire
            self._state_pub.publish(String(data=wire))

    # --- cancel --------------------------------------------------------------------

    def _on_cancel(self, req, resp):
        # Dispatchers FIRST — otherwise patrol/explore re-dispatch right
        # through the action cancel and the robot pauses for ~1 s.
        parts = []
        if self._patrol_stop.service_is_ready():
            self._patrol_stop.call_async(Trigger.Request())
            parts.append('patrol stopped')
        self._explore_pause.publish(Bool(data=False))
        parts.append('explore paused')
        fired = cancel_nav_goals(self._cancel_clients)
        parts.append('canceled: %s' % (', '.join(fired) if fired
                                       else 'no action server ready'))
        self.get_logger().info('nav cancel — ' + '; '.join(parts))
        resp.success = True
        resp.message = '; '.join(parts)
        return resp


def main(args=None):
    run_node(NavManager, args=args)


if __name__ == '__main__':
    main()
