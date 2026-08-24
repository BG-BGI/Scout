#!/usr/bin/env python3
"""Persistent map-frame layer of low obstacles the lidar cannot see.

The D455's 0.05–0.25 m depth band (office-chair bases, shoes, thresholds —
everything under the lidar's 24 cm scan plane) is accumulated into a 2D
log-odds grid anchored in the map frame, so clutter persists across the
camera's narrow live window and across sessions.

Clearing is polar ray approximation: live cloud points (full height,
including floor hits) are binned by bearing; a marked cell is decremented
when a live ray in its bearing bin reaches beyond it — the camera saw
through where the mark was, so the chair has moved. Marks need two hits AND
sightings spanning `confirm_window` seconds to report — a walking person
sweeps through a 5 cm cell far too fast to dwell, so movers never confirm.
Unconfirmed cells expire after `unconfirmed_ttl`; confirmed ones decay only
by being seen-through, never by time — persistence is the point.

Outputs:
  /clutter_map     nav_msgs/OccupancyGrid, transient-local, for the web UI
  /clutter_points  PointCloud2 in map frame at z=0.1, republished at 1 Hz so
                   the nav2 global costmap's ObstacleLayer keeps marking them
                   (StaticLayer is not used: two static layers fight over the
                   master grid size)

Persistence: numpy .npz at `file` (default /ros_ws/src/sites/active/maps/clutter.npz),
loaded at startup, autosaved every `autosave_period` while dirty.

Needs map->base_link TF (slam running). Without it the node idles and says
so once — clutter in odom would smear as odom drifts, so it waits for map.
"""

import math
import os
import time

import numpy as np
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from scout.node_util import lookup_matrix, lookup_pose2, run_node
from scout.qos import LATCHED_QOS

MARK = 2          # log-odds increment per confirmed sighting
CLEAR = 1         # decrement per see-through ray
CAP = 10          # saturation
REPORT_AT = 4     # >= this counts as occupied (two marks)


class ClutterMapper(Node):
    """Accumulate under-lidar depth returns into a persistent map layer."""

    def __init__(self):
        super().__init__('clutter_mapper')

        p = self.declare_parameter
        self._res = float(p('resolution', 0.05).value)
        self._band_lo = float(p('band_low', 0.05).value)
        self._band_hi = float(p('band_high', 0.25).value)
        # 0.25: bench-measured camera MinZ is 0.30 (disparity shift 12), so
        # returns down to ~0.30 are real; below that the camera is blind.
        self._min_range = float(p('min_range', 0.25).value)
        self._max_range = float(p('max_range', 2.0).value)
        self._half_fov = math.radians(float(p('half_fov_deg', 40.0).value))
        self._period = float(p('process_period', 0.3).value)
        self._file = str(p('file', '/ros_ws/src/sites/active/maps/clutter.npz').value)
        self._autosave = float(p('autosave_period', 30.0).value)
        # Mover rejection: sightings must span this long before a cell reports
        # (a walker crosses a cell in well under a second; furniture dwells),
        # and cells that never confirm are dropped instead of accumulating.
        self._confirm_window = float(p('confirm_window', 1.0).value)
        self._unconfirmed_ttl = float(p('unconfirmed_ttl', 5.0).value)

        self._cells = {}          # {(ix, iy): [log-odds, first_seen, last_seen]}
        self._dirty = False
        self._last_proc = 0.0
        self._cam_rot = None
        self._cam_trans = None
        self._warned_no_map = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._grid_pub = self.create_publisher(
            OccupancyGrid, 'clutter_map', LATCHED_QOS)
        self._points_pub = self.create_publisher(PointCloud2, 'clutter_points', 10)

        # raw=True: the depth cloud arrives every camera frame (~30 Hz) but
        # only one in ~10 is ever processed (_period). Deserializing a
        # PointCloud2 is the expensive part, so it's skipped entirely for
        # frames that would be thrown away by the period/TF check below —
        # deserializing every frame just to discard most of them measured as
        # real CPU cost.
        self.create_subscription(PointCloud2, 'camera/camera/depth/color/points',
                                 self._on_depth_raw, qos_profile_sensor_data,
                                 raw=True)
        self.create_service(Trigger, 'clutter/save', self._on_save)
        self.create_service(Trigger, 'clutter/clear', self._on_clear)
        self.create_timer(1.0, self._publish)
        self.create_timer(self._autosave, self._autosave_tick)

        self._load()
        self.get_logger().info(
            'Clutter mapper up: %.2f m cells, band %.2f-%.2f m, %d cells loaded'
            % (self._res, self._band_lo, self._band_hi, len(self._cells)))

    # --- persistence -----------------------------------------------------------
    # ⚠ Marks are anchored in the MAP frame, so persistence is only sound when
    # the map itself persists (slam localization/continue mode). Under
    # mode:=new every boot resets the map frame and a loaded clutter file
    # paints phantom lethal cells at wrong coordinates — seen 2026-08-12 as
    # nav2 'failed to create plan' + recovery thrash. An empty `file` param
    # disables load/save entirely.
    def _load(self):
        if not self._file or not os.path.exists(self._file):
            return
        try:
            data = np.load(self._file)
            # Loaded cells were confirmed when saved: dwell pre-satisfied.
            now = time.monotonic()
            for ix, iy, v in data['cells']:
                self._cells[(int(ix), int(iy))] = [
                    int(v), now - self._confirm_window, now]
        except Exception as exc:  # noqa: BLE001 — corrupt file must not kill the node
            self.get_logger().error('Could not load %s: %s' % (self._file, exc))

    def _save(self):
        # Only confirmed cells persist — saving in-flight marks would resurrect
        # mover ghosts as confirmed on the next load.
        cells = np.array([(ix, iy, e[0]) for (ix, iy), e in self._cells.items()
                          if self._confirmed(e)],
                         dtype=np.int32).reshape(-1, 3)
        try:
            np.savez_compressed(self._file, cells=cells)
            self._dirty = False
            return True
        except OSError as exc:
            self.get_logger().error('Could not save %s: %s' % (self._file, exc))
            return False

    def _confirmed(self, entry):
        return (entry[0] >= REPORT_AT
                and entry[2] - entry[1] >= self._confirm_window)

    def _autosave_tick(self):
        if self._dirty:
            self._save()

    def _on_save(self, request, response):
        response.success = self._save()
        response.message = '%d cells -> %s' % (len(self._cells), self._file)
        return response

    def _on_clear(self, request, response):
        n = len(self._cells)
        self._cells = {}
        self._dirty = True
        response.success = True
        response.message = 'cleared %d cells' % n
        return response

    # --- input -------------------------------------------------------------------
    def _map_pose(self):
        pose = lookup_pose2(self._tf_buffer, 'map', 'base_link')
        if pose is None:
            if not self._warned_no_map:
                self._warned_no_map = True
                self.get_logger().warn(
                    'No map->base_link TF (slam not running?) — clutter mapping idle')
            return None
        self._warned_no_map = False
        return pose

    def _resolve_camera_tf(self, frame_id):
        result = lookup_matrix(self._tf_buffer, 'base_link', frame_id)
        if result is None:
            return False
        self._cam_rot, self._cam_trans = result
        return True

    def _on_depth_raw(self, raw_msg):
        now = time.monotonic()
        if now - self._last_proc < self._period:
            return
        pose = self._map_pose()
        if pose is None:
            return
        self._on_depth(deserialize_message(raw_msg, PointCloud2), now, pose)

    def _on_depth(self, msg: PointCloud2, now, pose):
        if self._cam_rot is None and not self._resolve_camera_tf(msg.header.frame_id):
            return
        self._last_proc = now
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except (AttributeError, TypeError):
            return
        if pts.size == 0:
            return
        p = pts.astype(np.float32) @ self._cam_rot.T + self._cam_trans
        x, y, z = p[:, 0], p[:, 1], p[:, 2]
        r = np.hypot(x, y)
        bearing = np.arctan2(y, x)
        in_view = ((x > 0.0) & (r >= self._min_range) & (r <= self._max_range)
                   & (np.abs(bearing) <= self._half_fov))

        # Polar free-space profile: farthest live return per bearing bin, all
        # heights (floor included) — the see-through evidence for clearing.
        nbins = 80
        bin_w = 2.0 * self._half_fov / nbins
        bins = np.clip(((bearing[in_view] + self._half_fov) / bin_w).astype(int),
                       0, nbins - 1)
        free_to = np.zeros(nbins, dtype=np.float32)
        np.maximum.at(free_to, bins, r[in_view])

        # Mark: low band only.
        low = in_view & (z > self._band_lo) & (z < self._band_hi)
        mx, my, myaw = pose
        c, s = math.cos(myaw), math.sin(myaw)
        if low.any():
            wx = mx + x[low] * c - y[low] * s
            wy = my + x[low] * s + y[low] * c
            for ix, iy in zip((wx / self._res).astype(int),
                              (wy / self._res).astype(int), strict=True):
                key = (int(ix), int(iy))
                entry = self._cells.get(key)
                if entry is None:
                    self._cells[key] = [MARK, now, now]
                else:
                    entry[0] = min(entry[0] + MARK, CAP)
                    entry[2] = now
            self._dirty = True

        # Clear: marked cells in view that a longer ray sees through.
        if self._cells:
            ci, si = math.cos(-myaw), math.sin(-myaw)
            for key in list(self._cells.keys()):
                cxw = (key[0] + 0.5) * self._res - mx
                cyw = (key[1] + 0.5) * self._res - my
                bx = cxw * ci - cyw * si
                by = cxw * si + cyw * ci
                cr = math.hypot(bx, by)
                if bx <= 0.0 or cr < self._min_range or cr > self._max_range:
                    continue
                cb = math.atan2(by, bx)
                if abs(cb) > self._half_fov:
                    continue
                b = min(nbins - 1, max(0, int((cb + self._half_fov) / bin_w)))
                if free_to[b] > cr + 2.0 * self._res:
                    entry = self._cells[key]
                    entry[0] -= CLEAR
                    if entry[0] <= 0:
                        del self._cells[key]
                    self._dirty = True

        # Expire marks that never confirmed — mover residue, not furniture.
        cutoff = now - self._unconfirmed_ttl
        stale = [k for k, e in self._cells.items()
                 if not self._confirmed(e) and e[2] < cutoff]
        for k in stale:
            del self._cells[k]
        if stale:
            self._dirty = True

    # --- output ------------------------------------------------------------------
    def _publish(self):
        occupied = [(k, e) for k, e in self._cells.items() if self._confirmed(e)]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # Points for the global costmap's ObstacleLayer (z well inside its
        # marking band). Republished every second so marks survive costmap
        # resets; observation_persistence handles the rest.
        fields = [PointField(name=n, offset=4 * i,
                             datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
        pts = [((k[0] + 0.5) * self._res, (k[1] + 0.5) * self._res, 0.1)
               for k, _ in occupied]
        self._points_pub.publish(point_cloud2.create_cloud(header, fields, pts))

        grid = OccupancyGrid()
        grid.header = header
        grid.info.resolution = self._res
        if occupied:
            ixs = [k[0] for k, _ in occupied]
            iys = [k[1] for k, _ in occupied]
            x0, x1 = min(ixs), max(ixs)
            y0, y1 = min(iys), max(iys)
            w, h = x1 - x0 + 1, y1 - y0 + 1
            data = np.zeros((h, w), dtype=np.int8)
            for (ix, iy), _v in occupied:
                data[iy - y0, ix - x0] = 100
            grid.info.width = w
            grid.info.height = h
            grid.info.origin.position.x = x0 * self._res
            grid.info.origin.position.y = y0 * self._res
            grid.data = data.flatten().tolist()
        else:
            grid.info.width = 1
            grid.info.height = 1
            grid.data = [0]
        self._grid_pub.publish(grid)


def _save_if_dirty(node):
    if node._dirty:
        node._save()


def main(args=None):
    run_node(ClutterMapper, on_shutdown=_save_if_dirty, args=args)


if __name__ == '__main__':
    main()
