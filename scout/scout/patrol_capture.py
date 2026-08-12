#!/usr/bin/env python3
"""Waypoint patrol with pose-stamped photo capture — progress documentation.

The construction use case: drive a marked route through a building and
capture a photo at each waypoint, stamped with the map pose, so successive
runs document progress from repeatable vantage points.

Route authoring is by demonstration: teleop the robot to each vantage point
and call /patrol/mark (web UI button) — the current map pose is appended to
the route file. /patrol/start replays the route as sequential NavigateToPose
goals (one at a time, so a failed waypoint is skipped, not fatal), settles,
grabs the freshest frame off the already-running color stream, and appends
to a per-run manifest. Requires slam (map frame) + nav2 running.

Files (bind-mounted, gitignored like maps/):
  /ros_ws/src/maps/patrol_route.yaml            the route
  /ros_ws/src/captures/<runstamp>/wpNN.jpg      photos
  /ros_ws/src/captures/<runstamp>/manifest.yaml waypoint, pose, time, result

Safety: /patrol/stop cancels the active nav goal and ends the run (web UI
STOP calls it too). A battery reading under `abort_voltage` aborts the run
at the next waypoint boundary. The robot only moves between /patrol/start
and route end / stop / abort; this node never publishes cmd_vel itself —
nav2 owns motion, so all costmap avoidance applies.
"""

import math
import os
import time

import rclpy
import tf2_ros
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PatrolCapture(Node):
    """Sequential NavigateToPose route runner with per-waypoint capture."""

    def __init__(self):
        super().__init__('patrol_capture')

        p = self.declare_parameter
        self._route_file = str(p('route_file',
                                 '/ros_ws/src/maps/patrol_route.yaml').value)
        self._capture_dir = str(p('capture_dir', '/ros_ws/src/captures').value)
        self._settle = float(p('settle_seconds', 1.5).value)
        self._frame_max_age = float(p('frame_max_age', 2.0).value)
        self._abort_voltage = float(p('abort_voltage', 17.0).value)
        self._goal_timeout = float(p('goal_timeout', 120.0).value)

        self._route = self._load_route()
        self._state = 'idle'      # idle | driving | settling | capturing
        self._wp_i = 0
        self._run_dir = None
        self._results = []
        self._settle_until = 0.0
        self._goal_deadline = 0.0
        self._goal_handle = None
        self._nav_result = None   # None while pending, else True/False
        self._last_frame = None   # (monotonic stamp, CompressedImage)
        self._battery_v = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(CompressedImage,
                                 'camera/camera/color/image_raw/compressed',
                                 self._on_frame, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self._status_pub = self.create_publisher(String, 'patrol_status', 10)

        self.create_service(Trigger, 'patrol/mark', self._on_mark)
        self.create_service(Trigger, 'patrol/clear', self._on_clear)
        self.create_service(Trigger, 'patrol/start', self._on_start)
        self.create_service(Trigger, 'patrol/stop', self._on_stop)
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Patrol up: %d waypoints in %s'
                               % (len(self._route), self._route_file))

    # --- route persistence -----------------------------------------------------
    def _load_route(self):
        try:
            with open(self._route_file) as f:
                data = yaml.safe_load(f) or {}
            return list(data.get('waypoints', []))
        except FileNotFoundError:
            return []
        except yaml.YAMLError as exc:
            self.get_logger().error('Bad route file: %s' % exc)
            return []

    def _save_route(self):
        os.makedirs(os.path.dirname(self._route_file), exist_ok=True)
        with open(self._route_file, 'w') as f:
            yaml.safe_dump({'waypoints': self._route}, f)

    # --- inputs ------------------------------------------------------------------
    def _on_frame(self, msg):
        self._last_frame = (time.monotonic(), msg)

    def _on_battery(self, msg):
        self._battery_v = msg.voltage

    def _map_pose(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link',
                                                 rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                2.0 * math.atan2(q.z, q.w))

    # --- services ---------------------------------------------------------------
    def _on_mark(self, request, response):
        pose = self._map_pose()
        if pose is None:
            response.success = False
            response.message = 'no map pose (slam running?)'
            return response
        self._route.append({'x': round(pose[0], 3), 'y': round(pose[1], 3),
                            'yaw': round(pose[2], 3)})
        self._save_route()
        response.success = True
        response.message = 'waypoint %d marked' % len(self._route)
        self.get_logger().info(response.message)
        return response

    def _on_clear(self, request, response):
        n = len(self._route)
        self._route = []
        self._save_route()
        response.success = True
        response.message = 'cleared %d waypoints' % n
        return response

    def _on_start(self, request, response):
        if self._state != 'idle':
            response.success = False
            response.message = 'already running'
            return response
        if not self._route:
            response.success = False
            response.message = 'route is empty — mark waypoints first'
            return response
        if not self._nav.server_is_ready() and \
                not self._nav.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = 'nav2 action server not up'
            return response
        if self._map_pose() is None:
            response.success = False
            response.message = 'no map pose (slam running?)'
            return response
        stamp = time.strftime('%Y%m%d-%H%M%S')
        self._run_dir = os.path.join(self._capture_dir, stamp)
        os.makedirs(self._run_dir, exist_ok=True)
        self._results = []
        self._wp_i = 0
        self._send_goal(0)
        response.success = True
        response.message = 'patrol started: %d waypoints -> %s' \
            % (len(self._route), self._run_dir)
        self.get_logger().info(response.message)
        return response

    def _on_stop(self, request, response):
        was = self._state
        self._finish('stopped by operator' if was != 'idle' else None)
        response.success = True
        response.message = 'stopped (was %s)' % was
        return response

    # --- run state machine ---------------------------------------------------------
    def _send_goal(self, i):
        wp = self._route[i]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'   # never the display/odom frame
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.orientation.z = math.sin(float(wp['yaw']) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(wp['yaw']) / 2.0)
        self._nav_result = None
        self._goal_handle = None
        self._state = 'driving'
        self._goal_deadline = time.monotonic() + self._goal_timeout
        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_ack)
        self.get_logger().info('wp %d/%d -> (%.2f, %.2f)'
                               % (i + 1, len(self._route), wp['x'], wp['y']))

    def _on_goal_ack(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self._nav_result = False
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, fut):
        # status 4 = SUCCEEDED (action_msgs/GoalStatus)
        self._nav_result = (fut.result().status == 4)

    def _tick(self):
        now = time.monotonic()
        if self._state == 'driving':
            if self._nav_result is None and now < self._goal_deadline:
                return
            if self._nav_result is None:   # timeout: cancel and move on
                self.get_logger().warn('wp %d timed out' % (self._wp_i + 1))
                self._cancel_nav()
            self._settle_until = now + self._settle
            self._state = 'settling'
        elif self._state == 'settling':
            if now < self._settle_until:
                return
            self._capture(self._nav_result is True)
            self._advance()

    def _advance(self):
        if self._battery_v is not None and self._battery_v < self._abort_voltage:
            self._finish('battery %.1f V < %.1f — aborted'
                         % (self._battery_v, self._abort_voltage))
            return
        self._wp_i += 1
        if self._wp_i >= len(self._route):
            self._finish('route complete')
            return
        self._send_goal(self._wp_i)

    def _cancel_nav(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _finish(self, reason):
        self._cancel_nav()
        self._state = 'idle'
        if reason:
            self.get_logger().info('Patrol: %s' % reason)
            if self._run_dir:
                self._write_manifest(reason)

    # --- capture -----------------------------------------------------------------
    def _capture(self, nav_ok):
        wp = self._route[self._wp_i]
        pose = self._map_pose()
        fresh = (self._last_frame is not None
                 and time.monotonic() - self._last_frame[0] < self._frame_max_age)
        fname = 'wp%02d.jpg' % (self._wp_i + 1)
        if fresh:
            with open(os.path.join(self._run_dir, fname), 'wb') as f:
                f.write(bytes(self._last_frame[1].data))
        self._results.append({
            'waypoint': self._wp_i + 1,
            'target': dict(wp),
            'reached': bool(nav_ok),
            'photo': fname if fresh else None,
            'pose': ({'x': round(pose[0], 3), 'y': round(pose[1], 3),
                      'yaw': round(pose[2], 3)} if pose else None),
            'battery_v': self._battery_v,
            'time': time.strftime('%Y-%m-%d %H:%M:%S'),
        })
        self._write_manifest(None)
        self.get_logger().info('wp %d captured (nav %s, photo %s)'
                               % (self._wp_i + 1, 'ok' if nav_ok else 'FAILED',
                                  'ok' if fresh else 'STALE-SKIPPED'))

    def _write_manifest(self, end_reason):
        data = {'route_file': self._route_file, 'captures': self._results}
        if end_reason:
            data['ended'] = end_reason
        with open(os.path.join(self._run_dir, 'manifest.yaml'), 'w') as f:
            yaml.safe_dump(data, f)

    # --- status -------------------------------------------------------------------
    def _publish_status(self):
        msg = String()
        if self._state == 'idle':
            msg.data = 'idle|%d' % len(self._route)
        else:
            msg.data = '%s|%d|%d/%d' % (self._state, len(self._route),
                                        self._wp_i + 1, len(self._route))
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = PatrolCapture()
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
