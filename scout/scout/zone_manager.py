#!/usr/bin/env python3
"""Keepout / speed zone manager: webui polygons -> zones.json -> nav2 masks
(ADR-0019).

The webui draws zone polygons on the map (the coverage-box interaction) and
sends them here on /zone_cmd (String, grammar frozen in scout.core.zones).
This node owns the persistence and the derived artifacts:

  maps/zones.json            source of truth (named polygons per map)
  maps/zone_keepout.pgm/.yaml  derived mask for nav2's KeepoutFilter
  maps/zone_speed.pgm/.yaml    derived mask for nav2's SpeedFilter

After every edit it re-renders both masks and hot-reloads the two mask
map_servers (started by scout's nav2.launch.py when the mask files exist) via
their load_map service — async, SC11. The FIRST zone ever drawn therefore
needs one nav2 restart (the servers + costmap `filters` entries only launch
when the mask files exist); every later edit applies live.

/zones (latched String) carries the active map's zones as JSON — the same
schema as the store file, so the webui draws from the single schema rather
than a second wire format. Like clutter persistence, zones are only
meaningful under a persistent map frame (slam localization/continue): under
mode:=new the map frame resets every boot and old zones sit at wrong
coordinates — same trap, same rule (CLAUDE.md / clutter_mapper header).
"""

import json
import os

import numpy as np
from nav2_msgs.srv import LoadMap
from rclpy.node import Node
from std_msgs.msg import String

from scout.core import zones
from scout.node_util import run_node
from scout.qos import LATCHED_QOS

MASKS = (('keepout', 'trinary'), ('speed', 'scale'))


class ZoneManager(Node):
    """Persist drawn zones; render + hot-reload the nav2 filter masks."""

    def __init__(self):
        super().__init__('zone_manager')
        p = self.declare_parameter
        # Same repo-root bind convention as patrol_capture / clutter_mapper.
        self._file = str(p('zones_file', '/ros_ws/src/sites/active/maps/zones.json').value)
        self._masks_dir = str(p('masks_dir', '/ros_ws/src/sites/active/maps').value)
        # Which map's zones are ACTIVE (rendered to the masks). Keep in step
        # with slam's map:= when running localization/continue.
        self._map = str(p('map_name', 'default').value)
        self._resolution = float(p('mask_resolution', 0.05).value)

        self._zones_pub = self.create_publisher(String, '/zones', LATCHED_QOS)
        self.create_subscription(String, '/zone_cmd', self._on_cmd, 10)
        self._load_clients = {
            kind: self.create_client(
                LoadMap, '/%s_mask_server/load_map' % kind)
            for kind, _ in MASKS
        }
        # Publish current state; refresh artifacts if zones already exist so
        # a store edited by hand still renders.
        store = zones.load(self._file)
        if zones.zones_for(store, self._map):
            self._render(store)
        self._publish(store)
        self.get_logger().info(
            'zone_manager up: map %r, %d zone(s), masks in %s'
            % (self._map, len(zones.zones_for(store, self._map)),
               self._masks_dir))

    # --- command wire ------------------------------------------------------------

    def _on_cmd(self, msg: String):
        try:
            cmd = zones.parse_zone_cmd(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        # Reload around every mutation — the skills container or a hand edit
        # may have touched the store since (same rule as the waypoint store).
        store = zones.load(self._file)
        try:
            if cmd[0] == 'add':
                _, ztype, pct, poly = cmd
                name = zones.next_name(zones.zones_for(store, self._map), ztype)
                zones.set_zone(store, self._map, name, ztype, poly, pct)
                self.get_logger().info('zone %s added (%d vertices)'
                                       % (name, len(poly)))
            elif cmd[0] == 'delete':
                if not zones.delete_zone(store, self._map, cmd[1]):
                    self.get_logger().warn('no zone %r to delete' % cmd[1])
                    return
                self.get_logger().info('zone %s deleted' % cmd[1])
            else:   # clear
                store['maps'].pop(self._map, None)
                self.get_logger().info('zones cleared for map %r' % self._map)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        zones.save(self._file, store)
        self._render(store)
        self._publish(store)

    # --- derived artifacts ---------------------------------------------------------

    def _render(self, store):
        zn = zones.zones_for(store, self._map)
        keep, speed, origin = zones.rasterize(zn, self._resolution)
        if keep is None:
            # Cleared: render 1-cell empty masks so running mask servers
            # reload to "no restriction" instead of serving stale zones.
            keep = speed = np.zeros((1, 1), dtype=np.uint8)
            origin = (0.0, 0.0)
        for grid, (kind, mode) in ((keep, MASKS[0]), (speed, MASKS[1])):
            pgm = 'zone_%s.pgm' % kind
            with open(os.path.join(self._masks_dir, pgm), 'wb') as f:
                f.write(zones.to_pgm(grid))
            yaml_path = os.path.join(self._masks_dir, 'zone_%s.yaml' % kind)
            with open(yaml_path, 'w') as f:
                f.write(zones.mask_yaml(pgm, self._resolution, origin, mode))
            client = self._load_clients[kind]
            if client.service_is_ready():
                req = LoadMap.Request()
                req.map_url = yaml_path
                client.call_async(req)
            else:
                # First-ever zones: nav2.launch.py only starts the mask
                # servers when the files exist — one nav2 restart applies it.
                self.get_logger().warn(
                    '%s mask written but its map_server is not up — restart '
                    'nav2 to activate the zone filters (first time only)' % kind)

    def _publish(self, store):
        self._zones_pub.publish(String(
            data=json.dumps(zones.zones_for(store, self._map), sort_keys=True)))


def main(args=None):
    run_node(ZoneManager, args=args)


if __name__ == '__main__':
    main()
