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

import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
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
        # Coverage planning: a box dragged on the web UI map arrives on
        # /coverage_box; a serpentine route over its free/unknown cells
        # (obstacles inflated by coverage_inflation) replaces the current
        # patrol route. Spacing is the stripe pitch — 1.0 m suits photo
        # documentation; the lidar maps far wider than that regardless.
        self._cov_spacing = float(p('coverage_spacing', 1.0).value)
        self._cov_inflation = float(p('coverage_inflation', 0.30).value)
        self._cov_max_wp = int(p('coverage_max_waypoints', 120).value)

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
        self._grid = None
        self.create_subscription(OccupancyGrid, 'map', self._on_grid, 1)
        self.create_subscription(PolygonStamped, 'coverage_box',
                                 self._on_coverage_box, 1)
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
        self._route = route
        self._save_route()
        dist = sum(math.hypot(route[i + 1]['x'] - route[i]['x'],
                              route[i + 1]['y'] - route[i]['y'])
                   for i in range(len(route) - 1))
        self._plan_feedback('coverage route: %d waypoints, ~%.0f m — press Start'
                            % (len(route), dist))

    def _plan_feedback(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = 'plan|%s' % text
        self._status_pub.publish(msg)

    @staticmethod
    def _scanline(poly, wy):
        """Sorted [(xa, xb), ...] where the horizontal line y=wy is inside poly."""
        xs = []
        n = len(poly)
        for i in range(n):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
            if (y1 <= wy < y2) or (y2 <= wy < y1):
                xs.append(x1 + (wy - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]

    def _plan_coverage(self, poly):
        """Serpentine stripes over free/unknown cells inside the polygon.

        Occupied cells (>=50) are inflated by coverage_inflation; unknown
        (-1) counts as coverable — mapping unexplored space is the point,
        and nav2 plans through unknown (allow_unknown). Stripes are clipped
        to the polygon by scanline, then split at obstacles; nav2 routes
        between segment endpoints on its own.
        """
        info = self._grid.info
        res = info.resolution
        grid = np.array(self._grid.data, dtype=np.int8).reshape(
            info.height, info.width)
        blocked = grid >= 50
        for _ in range(max(1, int(round(self._cov_inflation / res)))):
            d = blocked.copy()
            d[1:, :] |= blocked[:-1, :]
            d[:-1, :] |= blocked[1:, :]
            d[:, 1:] |= blocked[:, :-1]
            d[:, :-1] |= blocked[:, 1:]
            blocked = d

        def cell_x(wx):
            return int((wx - info.origin.position.x) / res)

        def cell_y(wy):
            return int((wy - info.origin.position.y) / res)

        min_run = max(2, int(round(0.45 / res)))   # skip slivers < robot length
        y0 = min(p[1] for p in poly)
        y1 = max(p[1] for p in poly)
        route = []
        flip = False
        wy = y0 + self._cov_spacing / 2.0
        while wy < y1:
            iy = cell_y(wy)
            if 0 <= iy < info.height:
                segs = []
                for xa, xb in self._scanline(poly, wy):
                    ca = max(0, cell_x(xa))
                    cb = min(info.width - 1, cell_x(xb))
                    if cb - ca < min_run:
                        continue
                    open_row = ~blocked[iy, ca:cb + 1]
                    idx = np.flatnonzero(np.diff(np.concatenate(
                        ([0], open_row.view(np.int8), [0]))))
                    segs.extend((ca + idx[i], ca + idx[i + 1] - 1)
                                for i in range(0, len(idx), 2)
                                if idx[i + 1] - idx[i] >= min_run)
                if flip:
                    segs = [(b, a) for a, b in reversed(segs)]
                for a, b in segs:
                    wxa = info.origin.position.x + (a + 0.5) * res
                    wxb = info.origin.position.x + (b + 0.5) * res
                    yaw = 0.0 if wxb >= wxa else math.pi
                    route.append({'x': round(wxa, 3), 'y': round(wy, 3),
                                  'yaw': round(yaw, 3)})
                    route.append({'x': round(wxb, 3), 'y': round(wy, 3),
                                  'yaw': round(yaw, 3)})
            flip = not flip
            wy += self._cov_spacing
        return route

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
