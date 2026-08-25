# ADR-0023: location sites — per-location data bundles behind one symlink

Status: accepted · Date: 2026-08-22

## Context

The robot moves between locations (office one day, jobsite the next). Every
persistent store was one global pool: `maps/` (posegraph, zones.json,
waypoints.json, tags.db, clutter.npz, nav2 masks), `captures/`, companion
`captures/inspection/`, and a single `rtabmap.db` named volume. Map selection
was a hand-edited compose command (`mode:=localization map:=house` comment
toggling). Consequences: waypoints/zones from one location silently corrupt
operation at another (ADR-0011 already documents "a remap invalidates
waypoints" without enforcing it), cross-site loop closures pollute rtabmap,
and switching locations required editing a tracked file on the Pi — banned by
the deploy-via-git-only rule.

"Profile" was unavailable as a name: `SCOUT_PROFILE` is the scenario overlay
(ADR-0010) and `robot_profile.yaml` the cross-surface SSOT. These are
**sites**.

## Decision

`sites/<name>/` (repo root, gitignored) holds one location's full bundle:
`maps/` + `captures/` + `site.json` metadata. The **relative symlink
`sites/active -> <name>` is the single switch point** — it resolves
identically through every bind mount of the parent dir (`/ros_ws/src/sites`
in ROS containers, `/sites` in fleet_status/scout_skills). Mount the parent,
**never `./sites/active/...`** — mounting through the symlink freezes its
target at container create.

- **Switching is fleet_status's job** (`SITES_DIR` env gates the endpoints;
  unset = 404 = webui panel hides). `POST /api/sites/<name>/activate`
  atomically repoints the symlink (`os.symlink` to tmp + `os.replace` — the
  commit point), then restarts `SITE_RESTART_SERVICES` in order in a
  background thread, results surfaced via `GET /api/sites`.
- **The restart set is only the launch-bound holders**: `slam` (map file at
  launch), `nav2` (mask probe at launch), `behaviors` (zone_manager map_name
  + clutter file at init). Everything else resolves the symlink per
  operation and follows a switch live: scout-skills opens waypoints.json per
  call and tags.db per op, bag_recorder mkdirs per run, patrol_capture
  reloads at patrol start. **`robot` never restarts — the drivetrain,
  sensors and EKF stay up through a switch.** zone_manager moved from
  `robot` to `behaviors` to make that true (it touches no device; same
  fault-isolation rationale as the behaviors split).
- **`mode:=site`** in slam.launch.py reads `site.json` via pure
  `scout.core.sites` and resolves `slam_mode: auto` to `continue` (map
  exists) or `new` (doesn't) — never `localization`, because serialize_map
  silently no-ops there while reporting SUCCESS. Resolution feeds the
  existing three-way executable table; ADR-0003 and the absent-not-false
  param discipline are untouched. The compose command is now permanent —
  no more hand-editing.
- **zone_manager's `map_name` derives from `site.json` `default_map`**,
  killing the manual "keep map_name in step with slam's map:=" coupling.
  clutter_mapper's persistence follows the same signal (file set only when a
  default_map exists — the mode:=new phantom-obstacle guard, now automatic).
- **Companion mirrors the switch over HTTP, not zenoh** (allowlist is the
  security surface, ADR-0022; no services cross the bridge). The webui
  best-effort POSTs the same activate endpoint to the companion's
  fleet_status (`SITE_SCAFFOLD=plain`, restart set `rtabmap,
  inspection_recorder`). Per-site `rtabmap.db` lives at
  `companion/data/sites/<name>/rtabmap.db` via `database_path:=` through the
  same symlink pattern — the old `rtabmap_db` named volume is retired.
  Inspection runs land in `captures/inspection/<site>/<UTC>/`, site read per
  run start. Offline companion never blocks a Pi switch; the next switch
  re-syncs.
- **Webui Site panel** is the switch surface (list/switch/create/save-map).
  Save map = rosbridge `serialize_map` into `sites/active/maps/` + set
  `default_map` + offer a slam restart. The switch guard (no active nav
  goal, no recording) lives in the webui — fleet_status has no ROS view.
- **Migration**: `scripts/migrate_sites.py` (idempotent) moves the flat
  pools into `sites/default/`; slam `mode:=site` fails loudly with the
  script name if `sites/active` is missing. Companion migration is a
  documented one-time copy out of the named volume.

## Consequences

- One click in the webui re-homes the robot: map, zones, waypoints, tags,
  clutter, captures, and the companion's rtabmap db all follow, ~20 s of
  slam/nav2/behaviors downtime, drivetrain untouched.
- In-flight bag/patrol writes keep their pre-swap file handles, so they
  finish correctly attributed to the old site.
- A failed restart after the symlink swap converges on that container's next
  start (commands/params are site-agnostic); the webui surfaces which
  service to retry from the System panel.
- The switch guard is webui-only: an MCP client can switch mid-drive
  (LAN-trust posture, same as every other endpoint here).
- Waypoint validity *within* a site stays unenforced (ADR-0011 status quo);
  sites shrink the blast radius, don't fix it.
- Per-site captures multiply SD usage; no quota or delete/rename endpoints
  in v1 (SSH for those; System panel disk vitals are the backstop).
- Deferred: `/world/registry` persistence (in-memory today — a detector
  feature, not a switching feature); per-site `SCOUT_PROFILE` binding (baked
  into compose commands, needs `compose up`, which fleet_status cannot do).
- Verify on the Pi: migrate → `docker compose up -d` → switch to a fresh
  site from the webui → slam logs `new`, waypoints/zones land under the new
  site → switch back → the old map and stores return; companion run dirs
  gain the site segment and `data/sites/active` follows.
