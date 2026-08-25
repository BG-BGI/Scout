#!/usr/bin/env python3
"""Migrate legacy waypoint files to the v2 store (ADR-0011).

Run once on the Pi after pulling the M7 change:

    python3 scripts/migrate_waypoints.py maps/

Merges a legacy `patrol_route.yaml` (its ordered poses become the inline
"patrol" route) and a legacy flat `waypoints.json` (its named poses) into a
single v2 `maps/waypoints.json`; the originals move to `*.bak`. Idempotent — a
store already at v2 is preserved, and a missing legacy file is skipped. The
loaders also auto-wrap a legacy flat waypoints.json on read, so only
patrol_route.yaml strictly requires this script.
"""

import os
import sys


def main(argv):
    maps_dir = argv[1] if len(argv) > 1 else "maps"
    # scout.core.waypoints is pure (json + stdlib) so it imports without ROS.
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scout"))
    from scout.core import waypoints as wp

    store_path = os.path.join(maps_dir, "waypoints.json")
    route_yaml = os.path.join(maps_dir, "patrol_route.yaml")

    store = wp.load(store_path)  # v2, or a legacy flat file auto-wrapped to v2

    if os.path.exists(route_yaml):
        try:
            import yaml
        except ImportError:
            print("pyyaml not available — cannot read patrol_route.yaml",
                  file=sys.stderr)
            return 1
        with open(route_yaml) as f:
            legacy = yaml.safe_load(f) or {}
        migrated = wp.migrate(legacy)
        store["routes"].update(migrated["routes"])
        os.replace(route_yaml, route_yaml + ".bak")
        n = len(migrated["routes"].get("patrol", []))
        print("migrated %s -> route 'patrol' (%d poses); original -> %s.bak"
              % (route_yaml, n, route_yaml))

    wp.save(store_path, store)
    print("wrote %s: %d waypoints, routes %s"
          % (store_path, len(store["waypoints"]), sorted(store["routes"])))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
