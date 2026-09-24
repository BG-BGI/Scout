# ADR-0029: multi-map sites (floors) + AprilTag floor transit

Status: accepted · Date: 2026-09-01

## Context

A site (ADR-0023) held exactly one map (`site.json` v1 `default_map`), which
cannot model a multi-floor building. And a floor transit (elevator, carried)
mislocalizes silently: the lidar cannot see a floor change, so amcl keeps
matching the old grid with real confidence. A registered AprilTag surveyed on
the destination floor's map can fix both the map and the pose —
tag_relocalizer already solves poses from tag sightings and seeds
`/initialpose` (ADR-0028).

## Decision

**site.json v2**: a flat `maps` dict of labeled maps plus `active_map`,
replacing `default_map` + top-level `map_start_pose`:

```json
{"version": 2, "display_name": "HQ", "active_map": "floor1", "slam_mode": "auto",
 "maps": {"floor1": {"label": "Lobby", "floor": 1, "map_start_pose": [0,0,0]},
          "yard":   {"label": "Yard",  "floor": null, "map_start_pose": [0,0,0]}}}
```

- Map files stay flat in `maps/<name>.{posegraph,data,yaml,pgm}` — nothing on
  disk moves, no migration script. Map names use the site-name regex.
- **Readers normalize v1→v2 in memory** (`scout.core.sites.load_site`; a hand
  copy in fleet-status — schema shared, not code, ADR-0011). **fleet_status is
  the only writer** and write-upgrades to v2 on its next write, mirroring a
  legacy `default_map: <active_map>` key for stale webui builds/rollback.
  `resolve_slam` keys `auto` and the localization start pose on `active_map`.
- `GET /api/sites` returns the per-map dict with `posegraph`/`grid` presence
  booleans (replacing the `maps`/`grids` arrays); on-disk map files with no
  site.json entry appear as synthetic `unregistered` rows.
- **tags.db gains `map_name`** (guarded ALTER TABLE): each survey is stamped
  with the map it was made on, only ever alongside a solved pose (a pose-less
  glimpse from the wrong floor cannot re-home a tag). The PK stays
  `(family, tag_id)` — one surveyed pose per tag, so **each floor's transit
  tag must be a distinct physical tag ID**. NULL = legacy = assume active map.
- **waypoints gain an optional `map` key** stamped at save time;
  `go_to_waypoint` refuses a waypoint whose map isn't the active one. Routes
  are per-map by convention (a cross-map route fails at `go_to` semantics,
  not silently).
- **Auto floor transit lives in tag_relocalizer**: a sighting of a tag whose
  `map_name` differs from `active_map` becomes a transit candidate; after
  `min_transit_sightings` (3) consistent frames — and only in localization
  mode, with no nav goal in flight (`/nav_state`), outside a 30 s cooldown,
  and with the target grid on disk — it calls `/map_server/load_map`
  (live, ~1 s), waits 0.7 s so amcl consumes the republished latched `/map`
  (amcl leaves `first_map_only` false; the global static layer subscribes
  transient-local), then publishes the tag-solved pose on `/initialpose` —
  already in the new map's frame, because the tag was surveyed there — and
  best-effort POSTs `{active_map}` to fleet_status so the switch survives
  restarts. POST failure is logged loudly: site.json then disagrees with the
  live map until the operator fixes it (visible in the webui header); the
  next slam restart reverts and the tag re-transits.
- **Manual switch surfaces**: webui Site panel map list (Activate = live
  LoadMap + `/initialpose` at the map's `map_start_pose` + reseed in
  localization mode; slam+behaviors restart otherwise) and the scout-skills
  `switch_map` MCP tool (same two paths). Save Map gains label/floor inputs
  and registers the entry via the `maps` patch key.

## Consequences

- One robot can now serve a 3-floor building from one site: per-floor maps,
  per-floor tag anchors, automatic re-homing on arrival at a floor.
- Transit quality is bounded by the tag survey (portable-base contract,
  unchanged) and by the single-view solve gates (≤3 m, non-flat face).
- Mapping modes still bind the map at slam launch — switching maps there is a
  ~20 s slam+behaviors restart, same as a mode change.
- Zones are out of scope: zone_manager/clutter_mapper were removed 2026-08-24.
- scout-skills' `FLEET_STATUS_URL` default was fixed from :9002
  (observability_mcp) to :9003 in passing — explore_start was silently
  targeting the wrong service.
