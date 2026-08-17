# ADR-0019: Keepout/speed zones — JSON polygons as truth, masks as artifacts

Status: accepted · Date: 2026-08-17

## Context

Operator confirmed wanting no-go and slow-down zones (2026-08-17 grill). nav2
1.1.20 ships `KeepoutFilter`/`SpeedFilter`, but they consume mask PGMs served
by a map_server + a CostmapFilterInfo server — an image-editing workflow that
makes zones uneditable after the fact and unreviewable as data. The webui
already has a polygon-drawing interaction (the patrol coverage box).

## Decision

**`maps/zones.json` is the source of truth** — named polygons per map with a
type (`keepout|speed`) and speed percentage, the waypoint-store pattern
(ADR-0011). The PGM+yaml mask pairs are **derived artifacts**, re-rendered by
`zone_manager` on every edit; pure store/grammar/rasterizer logic lives in
`scout.core.zones` (SC7 tested).

- **Edit wire:** the webui publishes `/zone_cmd` (String, `|`-grammar frozen in
  core.zones + test_zones.py — same contract style as the status formats);
  zone_manager reloads-mutates-saves the store, re-renders, and hot-reloads the
  mask map_servers via their `load_map` service (async, SC11). The latched
  `/zones` topic carries the active map's zones back as JSON — deliberately the
  store's own schema, not a second wire format.
- **Masks are self-sized** to the zones' bounding box (+1 m pad) at 0.05 m:
  costmap filters transform mask→costmap through TF, so the mask needs neither
  the slam map's origin nor its extent, and cells outside it are unfiltered.
- **Encoding:** keepout masks are binary 0/100 → default trinary map_server
  yaml reads them exactly. Speed masks carry the percentage as the cell value →
  `mode: scale` with `occupied_thresh: 0.996` (gray 0 still reads 100), ~1%
  quantization, CostmapFilterInfo type 1 (percent) with base 0 / multiplier 1.
- **nav2 wiring is conditional:** scout's `nav2.launch.py` starts the two mask
  servers + filter-info servers + a zones lifecycle manager, and deep-merges
  the costmap `filters` entries (keepout in BOTH costmaps, speed global-only —
  SpeedFilter publishes `/speed_limit`, which controller_server subscribes by
  default) **only when the mask files exist**. No zones → nav2 byte-identical
  to before; the FIRST zone drawn needs one nav2 restart, later edits apply
  live through `load_map`.
- **One active zone set:** zone_manager's `map_name` param selects which map's
  zones render to `maps/zone_keepout.*`/`zone_speed.*`. Like clutter
  persistence, zones are only sound under a persistent map frame (slam
  localization/continue) — under `mode:=new` they'd sit at wrong coordinates.

## Consequences

- Zones are drawn, listed and deleted from the webui; the JSON is diffable and
  survives mask-format changes (rerender = re-run zone_manager).
- Two extra tiny processes in the nav2 service, only once zones exist.
- The speed value is a percentage of the profile caps, quantized ~1% by the
  PGM path — fine for a speed limit, wrong tool for precision control.
- Verify (Pi, after the first zone + nav2 restart): plan a goal through a
  keepout → path routes around it; drive into a speed zone → controller slows
  (`/speed_limit` shows the percent); delete the zone → next replan crosses.
  Operator confirms every drive per the motion rules.
