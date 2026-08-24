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

Route storage is the shared waypoint store (ADR-0011): /patrol/mark adds a
`mark-<n>` waypoint and appends its NAME to the route, and /patrol/start
RESOLVES the route names to poses — so a waypoint the scout-skills tag watcher
refreshes is driven at its fresh pose automatically.

Files (bind-mounted, gitignored like maps/):
  /ros_ws/src/sites/active/maps/waypoints.json  waypoint + route store (ADR-0011)
  /ros_ws/src/sites/active/captures/<runstamp>/wpNN.jpg      photos
  /ros_ws/src/sites/active/captures/<runstamp>/manifest.yaml waypoint, pose, time, result

Safety: /patrol/stop cancels the active nav goal and ends the run (web UI
STOP calls it too). A battery reading under `abort_voltage` aborts the run
at the next waypoint boundary. The robot only moves between /patrol/start
and route end / stop / abort; this node never publishes cmd_vel itself —
nav2 owns motion, so all costmap avoidance applies.
"""

import math
import os
import time

import numpy as np
import tf2_ros
import yaml
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from scout.core import waypoints as wpstore
from scout.core.coverage import plan_coverage
from scout.core.geometry import yaw_to_quat_zw
from scout.core.status import format_patrol_plan, format_patrol_status
from scout.node_util import lookup_pose2, run_node
from scout.robot_profile import load as _load_profile


class PatrolCapture(Node):
    """Sequential NavigateToPose route runner with per-waypoint capture."""

    def __init__(self):
        super().__init__('patrol_capture')

        p = self.declare_parameter
        self._waypoints_file = str(p('waypoints_file',
                                     '/ros_ws/src/sites/active/maps/waypoints.json').value)
        self._route_name = str(p('route_name', 'patrol').value)
        self._capture_dir = str(p('capture_dir', '/ros_ws/src/sites/active/captures').value)
        self._settle = float(p('settle_seconds', 1.5).value)
        self._frame_max_age = float(p('frame_max_age', 2.0).value)
        self._abort_voltage = float(
            p('abort_voltage',
              float(_load_profile()['battery_activity_floor_v'])).value)
        self._goal_timeout = float(p('goal_timeout', 120.0).value)
        # Coverage planning: a box dragged on the web UI map arrives on
        # /coverage_box; a serpentine route over its free/unknown cells
        # (obstacles inflated by coverage_inflation) replaces the current
        # patrol route. Spacing is the stripe pitch — 1.0 m suits photo
        # documentation; the lidar maps far wider than that regardless.
        self._cov_spacing = float(p('coverage_spacing', 1.0).value)
        self._cov_inflation = float(p('coverage_inflation', 0.30).value)
        self._cov_max_wp = int(p('coverage_max_waypoints', 120).value)

        self._route = self._resolve_route()   # resolved [{x,y,yaw}]; refreshed at start
        self._state = 'idle'      # idle | driving | settling | capturing
        self._wp_i = 0
        self._run_dir = None
        self._results = []
        self._settle_until = 0.0
        self._goal_deadline = 0.0
        self._goal_handle = None
        self._nav_result = None   # None while pending, else True/False
        # True only for a cancel WE initiate (timeout / operator stop). An
        # EXTERNAL cancel (webui Cancel, skills nav_cancel) leaves this False,
        # which is how _on_goal_result tells "skip this waypoint" from "stop
        # the patrol" — see _cancel_nav / _on_goal_result.
        self._self_cancel = False
        self._last_frame = None   # (monotonic stamp, CompressedImage)
        self._battery_v = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Compressed-image subscription is created only for the duration of a
        # run (_on_start / _finish) — otherwise rclpy deserializes every
        # camera frame forever just to have _on_frame store-and-discard it,
        # which measured as real idle CPU cost.
        self._frame_sub = None
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self._grid = None
        self.create_subscription(OccupancyGrid, 'map', self._on_grid, 1)
        self.create_subscription(PolygonStamped, 'coverage_box',
                                 self._on_coverage_box, 1)
        self._status_pub = self.create_publisher(String, 'patrol_status', 10)
        # Route republished at 1 Hz so the web UI can draw waypoints over the
        # map (late subscribers via rosbridge miss latched topics).
        self._route_pub = self.create_publisher(Path, 'patrol_route', 10)

        self.create_service(Trigger, 'patrol/mark', self._on_mark)
        self.create_service(Trigger, 'patrol/clear', self._on_clear)
        self.create_service(Trigger, 'patrol/start', self._on_start)
        self.create_service(Trigger, 'patrol/stop', self._on_stop)
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Patrol up: route %r, %d waypoints in %s'
                               % (self._route_name, len(self._route),
                                  self._waypoints_file))

    # --- route storage (shared waypoint store, ADR-0011) ------------------------
    def _resolve_route(self):
        """Resolved [{x, y, yaw}] for the configured route, or [] if absent."""
        try:
            return wpstore.resolve_route(
                wpstore.load(self._waypoints_file), self._route_name)
        except KeyError:
            return []

    # --- inputs ------------------------------------------------------------------
    def _on_frame(self, msg):
        self._last_frame = (time.monotonic(), msg)

    def _on_battery(self, msg):
        self._battery_v = msg.voltage

    def _on_grid(self, msg):
        self._grid = msg

    # --- coverage planning --------------------------------------------------------
    def _on_coverage_box(self, msg: PolygonStamped):
        if self._state != 'idle':
            self._plan_feedback('coverage ignored: patrol is running')
            return
        if self._grid is None:
            self._plan_feedback('coverage failed: no /map yet (slam running?)')
            return
        pts = msg.polygon.points
        if len(pts) < 2:
            return
        if len(pts) == 2:   # two corners = axis-aligned box
            poly = [(pts[0].x, pts[0].y), (pts[1].x, pts[0].y),
                    (pts[1].x, pts[1].y), (pts[0].x, pts[1].y)]
        else:
            poly = [(p.x, p.y) for p in pts]
        try:
            route = self._plan_coverage(poly)
        except Exception as exc:  # noqa: BLE001 — a bad box must not kill the node
            self._plan_feedback('coverage failed: %s' % exc)
            return
        if not route:
            self._plan_feedback('coverage failed: no reachable stripes in the box')
            return
        if len(route) > self._cov_max_wp:
            self._plan_feedback('coverage failed: %d waypoints > max %d — '
                                'smaller box or larger coverage_spacing'
                                % (len(route), self._cov_max_wp))
            return
        try:
            store = wpstore.load(self._waypoints_file)
            # Coverage points are inline poses (out of the name namespace).
            store['routes'][self._route_name] = [
                {'x': wp['x'], 'y': wp['y'], 'yaw': wp['yaw']} for wp in route]
            wpstore.save(self._waypoints_file, store)
        except Exception as exc:  # noqa: BLE001 — saving must not kill the node
            self._plan_feedback('coverage route not saved: %s' % exc)
        self._route = route
        dist = sum(math.hypot(route[i + 1]['x'] - route[i]['x'],
                              route[i + 1]['y'] - route[i]['y'])
                   for i in range(len(route) - 1))
        self._plan_feedback('coverage route: %d waypoints, ~%.0f m — press Start'
                            % (len(route), dist))

    def _plan_feedback(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = format_patrol_plan(text)
        self._status_pub.publish(msg)

    def _plan_coverage(self, poly):
        """Serpentine stripes over free/unknown cells inside the polygon
        (obstacles inflated by coverage_inflation). See scout.core.coverage."""
        info = self._grid.info
        grid = np.array(self._grid.data, dtype=np.int8).reshape(
            info.height, info.width)
        return plan_coverage(
            grid, (info.origin.position.x, info.origin.position.y),
            info.resolution, poly,
            spacing=self._cov_spacing, inflation=self._cov_inflation,
            occupied=int(_load_profile()['occupied_threshold']))

    def _map_pose(self):
        return lookup_pose2(self._tf_buffer, 'map', 'base_link')

    # --- services ---------------------------------------------------------------
    @staticmethod
    def _next_mark_name(store):
        existing = store.get('waypoints', {})
        n = 1
        while ('mark-%d' % n) in existing:
            n += 1
        return 'mark-%d' % n

    def _on_mark(self, request, response):
        pose = self._map_pose()
        if pose is None:
            response.success = False
            response.message = 'no map pose (slam running?)'
            return response
        store = wpstore.load(self._waypoints_file)
        name = self._next_mark_name(store)
        wpstore.set_waypoint(store, name, pose, 'mark',
                             saved=time.strftime('%Y-%m-%d %H:%M:%S'))
        store['routes'].setdefault(self._route_name, []).append(name)
        wpstore.save(self._waypoints_file, store)
        self._route = self._resolve_route()
        response.success = True
        response.message = 'marked %s (%d in route %r)' % (
            name, len(self._route), self._route_name)
        self.get_logger().info(response.message)
        return response

    def _on_clear(self, request, response):
        # Clears the route and deletes ITS mark-* waypoints only (named/tag
        # waypoints survive — semantic change from the old nuke-everything).
        store = wpstore.load(self._waypoints_file)
        route = store.get('routes', {}).get(self._route_name, [])
        marks = [it for it in route
                 if isinstance(it, str) and it.startswith('mark-')]
        for m in marks:
            store['waypoints'].pop(m, None)
        store['routes'][self._route_name] = []
        wpstore.save(self._waypoints_file, store)
        self._route = []
        response.success = True
        response.message = 'cleared route %r (%d entries, %d marks)' % (
            self._route_name, len(route), len(marks))
        return response

    def _on_start(self, request, response):
        if self._state != 'idle':
            response.success = False
            response.message = 'already running'
            return response
        # Resolve names -> poses NOW, so a tag-refreshed waypoint is driven at
        # its current pose (ADR-0011).
        self._route = self._resolve_route()
        if not self._route:
            response.success = False
            response.message = ('route %r is empty — mark waypoints first'
                                % self._route_name)
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
        self._last_frame = None
        if self._frame_sub is None:
            self._frame_sub = self.create_subscription(
                CompressedImage, 'camera/camera/color/image_raw/compressed',
                self._on_frame, qos_profile_sensor_data)
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
        (goal.pose.pose.orientation.z,
         goal.pose.pose.orientation.w) = yaw_to_quat_zw(float(wp['yaw']))
        self._nav_result = None
        self._goal_handle = None
        self._self_cancel = False
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
        # action_msgs/GoalStatus: 4 = SUCCEEDED, 5 = CANCELED.
        status = fut.result().status
        # An EXTERNAL cancel (webui Cancel / skills nav_cancel — the "software
        # e-stop") must END the patrol, not read as a failed waypoint and
        # advance to the next goal (which kept the robot driving through the
        # cancel). Our own timeout/stop cancels set _self_cancel, and those
        # fall through to the normal skip-and-continue path.
        if status == 5 and not self._self_cancel and self._state != 'idle':
            self._finish('nav goal canceled externally — patrol stopped')
            return
        self._nav_result = (status == 4)

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
        # Mark this as a self-initiated cancel so the result callback treats it
        # as skip-this-waypoint (timeout) or run-already-ending (stop/abort),
        # NOT as an external e-stop that should halt the patrol.
        self._self_cancel = True
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _finish(self, reason):
        self._cancel_nav()
        if self._frame_sub is not None:
            self.destroy_subscription(self._frame_sub)
            self._frame_sub = None
            self._last_frame = None
        self._state = 'idle'
        if self._frame_sub is not None:
            self.destroy_subscription(self._frame_sub)
            self._frame_sub = None
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
        data = {'route': self._route_name, 'captures': self._results}
        if end_reason:
            data['ended'] = end_reason
        with open(os.path.join(self._run_dir, 'manifest.yaml'), 'w') as f:
            yaml.safe_dump(data, f)

    # --- status -------------------------------------------------------------------
    def _publish_status(self):
        path = Path()
        path.header.frame_id = 'map'
        for wp in self._route:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(wp['x'])
            ps.pose.position.y = float(wp['y'])
            path.poses.append(ps)
        self._route_pub.publish(path)
        msg = String()
        msg.data = format_patrol_status(self._state, len(self._route), self._wp_i)
        self._status_pub.publish(msg)


def main(args=None):
    run_node(PatrolCapture, args=args)


if __name__ == '__main__':
    main()
