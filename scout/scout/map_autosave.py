"""Periodic slam_toolbox pose-graph autosave (restart-proofing, see
scout.core.mapsafety for the incident and the rules).

Launched by slam.launch.py ONLY alongside async_slam_toolbox_node (modes new /
continue) -- never in localization (amcl; slam_toolbox's serialize_map is a
silent no-op there anyway).

Every autosave_period_s it calls /slam_toolbox/serialize_map to a temp basename
in the active site's maps dir, then core.mapsafety.install_autosave validates
the pair and atomically promotes it to `_autosave` (previous good copy kept as
`_autosave.prev`). Adaptive: a serialize that took longer than period/4
stretches the interval to 4x its duration. One call in flight at a time.

Params: autosave_period_s (60; <=0 disables), maps_dir, min_posegraph_bytes,
min_data_bytes.
"""

import os
import time

from rclpy.node import Node

from scout.core import mapsafety
from scout.node_util import run_node

TICK_S = 5.0


class MapAutosave(Node):

    def __init__(self):
        super().__init__('map_autosave')
        self.declare_parameter('autosave_period_s', 60.0)
        self.declare_parameter(
            'maps_dir',
            os.path.join(os.environ.get('SCOUT_SITES_ROOT', '/ros_ws/src/sites'),
                         'active', 'maps'))
        self.declare_parameter('min_posegraph_bytes', mapsafety.MIN_POSEGRAPH_BYTES)
        self.declare_parameter('min_data_bytes', mapsafety.MIN_DATA_BYTES)
        self._period = float(self.get_parameter('autosave_period_s').value)
        self._dir = self.get_parameter('maps_dir').value
        self._min_pg = int(self.get_parameter('min_posegraph_bytes').value)
        self._min_dat = int(self.get_parameter('min_data_bytes').value)

        # Imported here so a missing slam_toolbox msg package fails loudly in
        # the node log rather than at module import for unit tests.
        from slam_toolbox.srv import SerializePoseGraph
        self._srv_type = SerializePoseGraph
        self._cli = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')
        self._last_start = None
        self._last_duration = 0.0
        self._in_flight = False
        self._start_t = 0.0

        # A kill mid-serialize leaves tmp debris; it is never promoted.
        for p in mapsafety.pair_paths(self._dir, mapsafety.TMP_BASE):
            try:
                os.remove(p)
            except OSError:
                pass
        if self._period > 0:
            self.create_timer(TICK_S, self._tick)
        self.get_logger().info(
            f'autosave every {self._period:.0f} s to {self._dir}/'
            f'{mapsafety.AUTOSAVE_BASE}' if self._period > 0
            else 'autosave disabled (autosave_period_s <= 0)')

    def _tick(self):
        now = time.monotonic()
        if not mapsafety.should_autosave(
                now, self._last_start, self._last_duration, self._period,
                self._in_flight):
            return
        if not self._cli.service_is_ready():
            return
        if not os.path.isdir(self._dir):
            os.makedirs(self._dir, exist_ok=True)
        req = self._srv_type.Request()
        req.filename = os.path.join(self._dir, mapsafety.TMP_BASE)
        self._in_flight = True
        self._start_t = self._last_start = now
        self._cli.call_async(req).add_done_callback(self._done)

    def _done(self, fut):
        self._in_flight = False
        self._last_duration = time.monotonic() - self._start_t
        try:
            res = fut.result()
        except Exception as e:  # noqa: BLE001 -- service died mid-call
            self.get_logger().warn(f'serialize_map failed: {e!r}')
            return
        code = getattr(res, 'result', 0)
        if code != 0:
            self.get_logger().warn(f'serialize_map returned result={code}')
            return
        ok, reason = mapsafety.install_autosave(
            self._dir, min_posegraph=self._min_pg, min_data=self._min_dat)
        log = self.get_logger().info if ok else self.get_logger().warn
        log(f'{reason} ({self._last_duration:.2f} s)')


def main(args=None):
    run_node(MapAutosave, args=args)
