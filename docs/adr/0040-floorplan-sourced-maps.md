# ADR-0040: floorplan-sourced maps — a drawing is a normal amcl grid map

Status: accepted · Date: 2026-11-10 · Amends ADR-0028, ADR-0029

Written from the code as merged (units U1 U2 U3 U4a U6a U6b U8a U9a U11). Where
the planning document (`docs/plans/2026-11-10-001-feat-floorplan-site-maps-plan.md`)
says something else, the code is the truth and the difference is stated.
**Nothing in this feature has run on the robot yet** — every number below that
says "tune on the robot" is a starting value.

## Context

A new site needed a mapping drive before anything could localize or navigate,
even when an accurate drawing existed (CAD PDF, scanned PDF, Revit/IFC export).
Three places assumed every map is a lidar map: the commit path refuses anything
without a ≥50 KB posegraph, `auto` never resolved to localization (ADR-0028),
and tags.db kept one pose per physical tag (ADR-0029), so a floor's scan map and
its drawing map could not both be surveyed.

## Decision

### 1. A drawing becomes a normal grid map, installed grid-only

No new runtime: a drawing is a `<name>.pgm` + `<name>.yaml` pair that amcl +
map_server load exactly as they load a scan map's grid (ADR-0028), with no
posegraph. Set pose, goals, POIs, waypoints and floor transit need no new path.

- **`POST /api/maps/install`** (fleet_status :9003, `docker/fleet-status/
  mapinstall.py`; `SITE_SCAFFOLD=pi` only, else 404). One JSON object, unknown
  fields are a 400: `name`, `yaml` (text), `pgm_b64`, `sidecar` (object), and
  optional `label`, `floor`, `map_start_pose`, `source` (only `"drawing"`).
  Caps: 16 MiB decoded pgm, 4000×4000 cells, 16 KiB yaml, 64 KiB sidecar. The
  stored yaml is regenerated from validated values (`image`, `mode`,
  `resolution`, `origin`, `negate`, thresholds); other keys are dropped.
- **No `force`.** Any request carrying one is a 400. A `.posegraph`/`.data`
  beside the name, or an entry that is not `source: drawing`, is a 409 and
  nothing changes: a scan map can never be replaced or quarantined through this
  door (requirement R11).
- **`<name>.drawing.json` sidecar** records provenance and its presence on disk
  is what marks `<name>` as a drawing map that may be **replaced in place** (the
  recalibrate path). Grid files or an entry without a sidecar are a 409.
- Install **never changes `active_map` and never reloads the running
  localizer**; the operator activates separately.
- `GET /api/maps/<name>.pgm|.yaml` serves the raw file (any map, scan or
  drawing, `Cache-Control: no-store`) so the browser decodes the pgm itself; no
  image library in fleet_status.

Rejected: relaxing `mapsafe.pair_check`/`commit_pending` (the strict scan-map
guard stays strict; the grid-only path is a separate module with its own
checks). Rejected, and **differs from the plan** (U2): a `force` flag that
quarantines an existing map — there is no way to destroy a scan map from here.

### 2. `source` on map entries; `auto` resolves to localization for a drawing

Map entries are `{label, floor, source, map_start_pose}`, `source ∈ {scan,
drawing}`, default `scan`; an unknown value reads as `scan` and fleet_status
rejects it on write. Two copies of the schema (ADR-0011: share schema, not
code): `scout.core.sites` and `docker/fleet-status/sitestore.py`, pinned by
`test_sitestore.py`.

**This supersedes one sentence of ADR-0028** — "Site `auto` policy still never
resolves to localization: with amcl running, slam_toolbox is absent entirely, so
nothing can be saved or extended there." The rule is now (`sites._auto_mode`):

| active map | files | `auto` resolves to |
|---|---|---|
| `source: drawing` | `<name>.yaml` exists | **`localization`** |
| `source: drawing` | no grid | `continue` if `.posegraph` exists, else `new` |
| `source: scan` | any | `continue` if `.posegraph` exists, else `new` (unchanged — never localization) |

It is keyed on the entry's **`source`**, not on a missing posegraph: a scan map
whose posegraph was quarantined must not silently flip into localization on a
stale grid. **Differs from the plan**, which said "grid and no posegraph"; a
drawing that also has a posegraph still resolves to localization.

**Effective mode vs pinned `slam_mode`.** `slam_mode` is the policy
(`auto|new|localization|continue`); `sites.effective_mode(site, maps_dir)` is
what it runs as — pinned modes pass through, `auto` resolves per the table.
`resolve_slam` uses it (one copy of the rule); fleet_status reports it as
`resolved_mode` in the site payload (a mirror in `sitestore.resolved_mode`).
Everything that asks "is amcl running?" uses the effective mode: webui
`activateMap`, scout-skills `switch_map` (both fall back to the raw `slam_mode`
against an older fleet_status), and tag_relocalizer's floor-transit gate.

### 3. Profile by source; restart on a cross-source switch

amcl's parameters are fixed at launch, so the drawing profile is selected by the
active map's `source` and applied at slam start.

- Overlays live in **`scout/config/source_overlays/<source>/`**, not
  `config/overlays/` (**differs from the plan**): `known_profiles()` lists every
  subdir of `overlays/` as an operator-selectable `SCOUT_PROFILE`, and a map
  source must never appear there.
- Resolution order **base ← `SCOUT_PROFILE` overlay ← source overlay**, later
  wins (`robot_profile.merged_params_chain(basename, [profile_overlay_dir,
  source_overlay_dir])`; `merged_params_for_source` is the launch's one call).
  A source with no overlay for the file — scan — takes the plain
  `merged_params` path, byte-identical to before. The source is validated against
  `sites.SOURCES` (a typo raises, never behaves as scan). An explicit
  `mode:=localization map:=X` launch takes X's source from the active site
  (`map_source_in_active_site`), scan when there is no site or entry.
- **Drawing amcl overlay** (`source_overlays/drawing/amcl.yaml`; scan base value
  in brackets). Why: the drawing shows walls only, so a live scan sees furniture,
  stock and equipment the map lacks, and those beams must be skipped or
  down-weighted rather than dragging the cloud off the walls.

  | key | drawing | [scan] |
  |---|---|---|
  | `laser_model_type` | `likelihood_field_prob` | [`likelihood_field`] |
  | `do_beamskip` | `true` | [`false`] |
  | `beam_skip_distance` / `_threshold` / `_error_threshold` | 0.5 / 0.3 / 0.9 | [same, unused] |
  | `max_beams` | 90 | [180] |
  | `laser_max_range` | 10.0 | [16.0] |
  | `z_hit` / `z_rand` | 0.6 / 0.4 | [0.9 / 0.1] |
  | `sigma_hit` | 0.15 | [0.1] |
  | `laser_likelihood_max_dist` | 2.0 (= nav2 default, absent from base) | [absent] |
  | `recovery_alpha_slow` / `_fast` | 0.0 / 0.0 (disabled) | [0.001 / 0.1] |

  Recovery is off because the average weight on a drawing is always low
  (clutter) and would scatter the cloud constantly. All values are
  plan/research starting points, **untuned on the robot**.
- **Cross-source switch restarts slam** (+ behaviors). Same-source switches keep
  the live `/map_server/load_map` path. Both surfaces (webui `activateMap`,
  `switch_map`) POST `active_map` then use the existing restart path, and refuse
  *before any change* when the target will run amcl but has no grid (otherwise
  `_grid_map_yaml` would crash-loop the container).
- **Re-seeding after a restart that leaves amcl running.** tag_relocalizer lives
  in the robot container, which a slam restart does not restart, so its "already
  seeded" flag still describes the old map; `/tag_relocalizer/reseed` re-arms
  it. nav2_amcl publishes `/amcl_pose` transient-local (KeepLast 1) with
  `header.stamp` = the laser scan's stamp, and a stationary amcl does not
  republish, so a subscriber opened right after the restart can be handed the
  *old* amcl's last pose. The gate therefore captures the latched pose's stamp
  **before** the restart and only a pose whose stamp **differs** counts as "amcl
  is up" (`webui/mapswitch.js`, mirrored in scout-skills; a parity test pins
  them). Stamps are compared with each other, never with a wall clock (the
  browser's clock is not the robot's and the Pi's steps at boot). The webui waits
  90 s, `switch_map` runs a background task (15 × [3 s window + 3 s pause]); on
  timeout the operator is told "Set pose after restart".
- **tag_relocalizer never transits live across sources** (log one throttled
  line; the restart is an operator action).

Rejected: one profile for all maps (scan-tuned amcl on a map that lacks the
clutter); switching amcl params at runtime (not possible); reseeding on any first
`/amcl_pose` (the stale-replay race above).

### 4. No global localization on a drawing; the scan overlay is a hint

Planning research found seeded AMCL on a plan tracks at about 5 cm while global
initialization fails on cluttered plans. So on a drawing map the operator (Set
pose, `/initialpose`) or a registered tag seeds the pose; amcl restarts at the
map's `map_start_pose`; `reinitialize_global_localization` is not offered as a
drawing recovery. **Scan-over-drawing check (decided; UI: pending):** before the
first goal the web UI draws the live scan over the drawing with a fit indicator.
It is an alignment hint, not proof — mislocalization looks valid (CLAUDE.md), and
a tight numeric correction does not show the pose is right.

### 5. Importer: placement, grid semantics, sources

- **Placement.** Its own service image (`scout-floorplan-importer`) on the
  companion host, **port 9004, HTTP only**: Pi CPU is protected, PyMuPDF and
  ifcopenshell never enter the Pi or fleet_status images, and no zenoh
  allowlist changes (ADR-0022). Multi-arch so the same image can run on a
  laptop. **Status: the library layer is merged** in `docker/floorplan-importer/`
  (`calib.py`, `planraster.py`, `pdfsource.py`, `ifcsource.py` — pure, no HTTP,
  no module state); **the HTTP server, Dockerfile, compose service and port
  wiring are pending** — none exist in the repo yet. (The plan's file list put
  the importer docs under `companion/`; the code lives under `docker/`.)
- **Grid semantics** (`planraster.py`; trinary pgm). Walls **occupied (0)**,
  enclosed interior **free (254)**, everything outside the building
  **unknown (205)**; `occupied_thresh` 0.65, `free_thresh` 0.196 (nav2 reads
  pixel 205 as occupancy 0.196, which the usual 0.25 would call free, so
  `free_thresh` is set to 0.196 to keep unknown unknown). Outside is told
  from inside on a sealed *copy* of the wall mask: doorways are bridged by a
  directional closing of **`close_m = 1.2` m** for the flood only — the returned
  grid keeps the doorways open; a gap wider than that leaves the interior
  unknown. Raster pages go through the same `rasterize()` as a pre-rasterized
  `Mask` (`extract_raster_walls`, after calibration). Why free inside / unknown
  outside: free interior gives recovery sampling and the planner room to route;
  unknown outside keeps `track_unknown_space` meaningful. Max 4000×4000 cells.
- **PDF** (`pdfsource.py`, PyMuPDF). **PyMuPDF is AGPL-3.0**: it stays inside the
  importer image, installed `pip install --no-deps -r requirements.txt` (2.x
  declares networkx/onnxruntime/etc. it does not need for drawings/text/render),
  and the importer is an internal service, not distributed; pdfplumber (MIT) is
  the fallback if that ever stops being acceptable. Walls are the dark fills or
  the heaviest dark strokes (whichever carries more ink), with layer filtering
  when OCGs exist; light/coloured paint, dashed lines, the sheet border, the
  title block, specks and paths under text are ignored. `kind` is `vector`,
  `raster` or `none`. Scale is a **proposal** with a confidence tag
  (`high|medium|low|none`) from the title-block note, UserUnit and paper size —
  never silently applied. Calibration is stored in PDF point space (page,
  rotation, points, metres) so re-rendering at any DPI does not invalidate it.
- **IFC** (`ifcsource.py`): ifcopenshell 0.9.0.post1 (geometry iterator, already
  metres) + shapely 2.2.0, **no trimesh** (**differs from the plan**; the section
  is built directly). One storey per call, plane at **z = storey elevation +
  0.241 m** (lidar height, `base_link→laser`) by absolute Z, so a wall crossing
  two floors shows on both. Included: IfcWall, IfcColumn, IfcSlab, IfcStair,
  IfcStairFlight, IfcWindow, IfcCurtainWall. **IfcDoor is excluded on purpose**
  (a door leaf is solid at 0.241 m and would seal every doorway; planraster
  bridges door gaps itself for inside/outside). Excluded: IfcSpace,
  IfcOpeningElement, IfcFurnishingElement, IfcDistributionElement. Memory is
  about 6× the IFC size per open model; face budget 3,000,000; input cap 256 MiB.
  The ifcopenshell/shapely pins are **not yet in the repo's `requirements.txt`**
  (it lists numpy and pymupdf only).

### 6. Operator decisions, 2026-11-10

1. **Tag pose is stored per tag per map** (section 7).
2. **Robot-measured scale is a plain REST GET on the Pi's scout-skills:**
   `GET :9001/api/wall_spans` (subscribe-only: `/scan`, `/tf_static`, `/odom`;
   needs no map or localization, so it works before any map exists; nothing is
   ever commanded). **CORS `*` is on that one route's own responses only** (plus
   its OPTIONS preflight) — never app-wide middleware and **never on `/mcp`**,
   which exposes motion tools: a CORS `/mcp` would let any web page in the
   operator's browser drive the robot. Rejected: relaying through the importer
   (it would need robot access — a new bridged topic).
3. **The web wizard owns the IFC storey picker;** the importer server stays
   UI-free (JSON API only).

### 7. Per-map tag poses (amends ADR-0029)

A floor can hold a scan map and a drawing map, each in its own frame, so a tag
is surveyed **per map**. **This supersedes two sentences of ADR-0029:** "The PK
stays `(family, tag_id)` — one surveyed pose per tag, so **each floor's transit
tag must be a distinct physical tag ID**" and "NULL = legacy = assume active
map" (kept only as the LEGACY rule below). The same physical tag can now be
surveyed on every map it appears on.

- **Schema.** `tags` keeps identity (name UNIQUE, role, size_m, last_seen). New
  `tag_poses(family, tag_id, map_name NOT NULL, map_x, map_y, map_yaw NOT NULL,
  surveyed_at, PRIMARY KEY(family, tag_id, map_name))`, plus an `AFTER DELETE ON
  tags` trigger that removes a tag's poses even when an older image deletes the
  tag (so poses cannot resurrect on re-register; chosen over a foreign key,
  which needs a per-connection PRAGMA the other processes would not set).
- **Migration** (`tags.migrate` in scout-skills, `tagpose.migrate` in
  fleet-status — duplicated, parity-tested): `PRAGMA user_version` < 3, then one
  `BEGIN IMMEDIATE` transaction with the version re-checked inside the lock (safe
  with scout-skills and fleet-status rw and tag_relocalizer `mode=ro` all open),
  idempotent, rolled back whole on error. Legacy poses are **copied, not
  moved**; the legacy pose columns on `tags` stay in place, unread and unwritten
  (a rolled-back older image still finds them — but surveys made by either
  image while the other runs are invisible to it).
- **LEGACY `''` rule.** A pre-U11 row with NULL `map_name` becomes `map_name =
  ''`; no map name is guessed at migration time. It keeps its old meaning —
  "the active map" — **only while the tag has no named pose anywhere**; once a
  tag is surveyed by name on any map, an unassigned pose never stands in for
  another map's survey. (Tightening: with a scan and a drawing map per floor, the
  frames differ, so "assume active" became dangerous.)
- **Writes target the active map only.** `record_sighting` inserts if absent
  (never replaces) and stores nothing when no map is active; webui Set tag
  position (`tagpose.set_pose`) upserts the active map's row and refuses 409
  without an active map; `clear_pose` clears the pose that applies to the active
  map (explicit map / all maps optional); `delete` removes the tag and all poses.
- **Reads.** `map_x/map_y/map_yaw/map_name` are the active map's pose (null when
  unsurveyed there — never another map's); a new `poses` object lists every named
  map `{x, y, yaw, surveyed_at}`.
- **First-survey rule.** A sighting may take a tag's *first* pose only if the tag
  has **no pose on any map**; otherwise surveying another map is explicit
  (`resurvey_apriltag` clears the active map's pose and arms the next sighting,
  in memory, so a server restart disarms; or webui Set tag position). Without
  this the 2 s watcher would first-survey a transit-candidate tag from the live
  robot pose (wrong if the robot is really on the other floor), polluting the
  map and killing the transit.
- **Transit home** (`core.tag_homes.choose_home`). A pose on the active map →
  relocalize there, never a transit candidate. Otherwise exactly **one other map
  with the same source** as the active map is the home and the existing transit
  path runs unchanged; zero or several → no transit and a throttled warning
  naming the candidates (a single other-source candidate is handed to the
  existing path, whose cross-source check refuses with its own message). Every
  existing gate (effective mode, nav-goal, grid exists, hysteresis, cooldown) is
  kept. The per-frame read is read-only and never raises on a corrupt, locked or
  half-migrated file.
- **Waypoints.** `waypoints.json` is unchanged: one waypoint per name, carrying
  one `map`. Every writer stamps the map of the coordinates it writes, and
  `go_to_waypoint` still refuses a waypoint of another map. A tag survey on map B
  replaces the tag's waypoint; the response reports the replaced waypoint
  (including its map), and a tag not surveyed on the active map gets no waypoint
  from live geometry.

## Known limits (stated plainly)

- The wall classifier was validated **only on Scout-generated sheets** (the
  A-101 samples: recall 1.000, precision ≈0.99; the tests assert ≥0.97 / ≥0.95
  and skip when the sample sheets are absent) — **not on real CAD or Revit
  exports.**
- A **double-hairline wall** matches neither wall class → `kind: none` → the
  operator falls back to the raster path (after calibrating).
- An **OCC boolean failure can silently seal a doorway** in an IFC slice: the
  geometry iterator subtracts openings, and a failed subtraction leaves the wall
  solid across the opening with no error.
- **Thin glass under one 0.05 m cell vanishes** in the grid.
- **Glass / curtain wall is solid on the drawing and invisible to the lidar:**
  amcl expects a wall the scan does not see (beam skipping mitigates, untested).
- The `wall_spans` stationary check (`STATIONARY_AGREE_M` 0.04 m, 75th-percentile
  first-vs-last scan disagreement; `STATIONARY_V` 0.02) is **untuned on the real
  lidar**.
- The webui Save Map button and mode selector still read the raw `slam_mode`:
  under `auto` on a drawing map Save Map falls through to a slam_toolbox service
  that is not running (fix pending with the UI units).
- A LEGACY pose applies to whichever map is active until the tag has a named
  pose, even when that map's frame does not match (re-survey it).
- **Pending:** importer HTTP server / image / compose service, web import wizard
  and IFC storey picker, source chips in the map list, the scan-over-drawing
  overlay UI, robot-measured calibration UI.

## Consequences

- Operating recipe for what exists today: `docs/slam.md` §6.
- A site can hold `floor1` (scan) and `floor1_plan` (drawing) side by side; each
  keeps its own tag surveys and relocalizes from its own.
- Deploy: fleet_status and scout-skills images and the scout package together;
  the first process to open each tags.db migrates it (a migrated file is not
  touched again). Existing single-map sites need nothing.
- Scan-map behaviour is unchanged: same amcl file, same `auto` result, same
  commit path.
