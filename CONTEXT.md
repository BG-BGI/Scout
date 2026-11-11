# Scout — domain context

Orientation for the software architecture. **CLAUDE.md** stays the hardware +
tuning log (measurements, PID, geometry); **docs/adr/** records the decisions.
This file names the concepts and maps the running system.

## Glossary

- **deadman** — the RoboClaw stops the motors if it gets no valid packet for
  `max_seconds_uncommanded_travel` (0.2 s). Idle mode is Free Wheeling, so this
  is a *coast*, not a brake. Every cmd_vel producer must stream ≥5 Hz.
- **zero-burst / STOP_GRACE** — a producer stops by publishing ~0.3 s of zero
  Twists (a commanded stop) then going silent (handing cmd_vel to others).
  Owned by `CmdVelSource`.
- **cmd_vel arbiter (twist_mux, two-stage)** — autonomous sources (skills,
  nav2) merge in `twist_mux_auto` → `/cmd_vel_auto` → collision_monitor →
  `/cmd_vel_safe`; the final `twist_mux` merges that with teleop (joy, web)
  and the estop brake into `/cmd_vel_out`. See ADR-0001/0016.
- **software e-stop** — the `/estop` twist_mux lock + an active-brake burst on
  `/cmd_vel_stop`. Latching; fail-safe (a dead estop node = locked). No hardware
  e-stop exists. See ADR-0001.
- **traction derate** — traction_monitor sits between the final mux and the
  driver (`/cmd_vel_out` → `/cmd_vel_trac`), walking a per-side factor down
  when a channel's current says a front wheel unloaded. Decisions in
  `scout.core.traction`; uncalibrated = pure passthrough.
- **velocity-loop floor** — ~0.05 m/s linear / ~0.35 rad/s pivot; below it the
  encoders quantization-stall. Advisory, not a clamp.
- **scrub floor / recommended pivot (2.5 rad/s)** — a skid-steer pivot walks;
  faster pivots walk less. A walk minimizer where position matters, NOT a stall
  clamp (the old flat-tire clamp is retired — ADR-0008).
- **under-lidar band (0.05–0.22 m)** — the height slice below the lidar's 24 cm
  plane where the D455 depth cloud catches chair bases / shoes / thresholds.
  Marked by the nav2 `stvl_layer` (replaced clutter_mapper, 2026-08-24).
- **negative obstacle / cliff** — a downward ledge. cliff_detector latches
  detections on an odom grid (`scout.core.cliff`), feeds a marking-only STVL
  source (`/cliff/points`) and a CM hard-stop cluster (`/cliff/stop_points`).
  Deliberately SILENT on camera/TF loss — CM `source_timeout` turns blind into
  stopped, and the `/diagnostics` cliff row shows it. See ADR-0024.
- **zone truth table** — collision_polygon_manager's (front, rear, turn)
  enabled triple over the three mutually-exclusive CM stop polygons, decided in
  `scout.core.collision` from turn/reverse latches (`scout.core.latch`);
  bypass disables all three, bounded at 30 s. See ADR-0016.
- **latch (core.latch.Latch)** — the one entry/exit/dwell hysteresis state
  machine (battery LED thresholds, tilt abort dwell, CM zone hysteresis).
- **waypoint / route / patrol run / mark** — a named map pose; an ordered list
  of them; one execution of a route with photo capture; the act of recording
  the current pose as a waypoint. See ADR-0011.
- **coverage box** — a polygon dragged on the web map; planned into a serpentine
  route (`scout.core.coverage`).
- **standoff / seek / reacquire** — RETIRED with follow_me (2026-08-24).
  trick_player, follow_me and zone_manager were removed as unused features;
  keepout/speed zones (ADR-0019) currently have no manager node.
- **tag registry vs detection coverage** — the AprilTag *meaning* (names, roles,
  home) lives in scout-skills' sqlite; *which family/size is detected* lives in
  `apriltag.yaml`. See ADR-0006.
- **tag families (ADR-0038)** — three separate registries: **AprilTag**
  (fiducial, `tags.db`, `*_apriltag` tools), **RFID/NFC/UHF** (read logs,
  `list_{rfid,nfc,uhf}_tags`), **POI** (operator-marked map point, no physical
  marker, `pois.db`, `*_poi` tools).
- **tag survey (per map) / `tag_poses`** — an AprilTag's identity is per physical
  tag (`tags`: name, role, size) but its surveyed pose is per **(family, tag_id,
  map)** in `tag_poses`, because a floor can hold a scan map and a drawing map in
  different frames. Writes (sighting, webui Set tag position, `clear_pose`) go to
  the **active** map only; reads give the active map's pose (null when unsurveyed
  there, never another map's) plus a `poses` object. **LEGACY pose** = a pre-U11
  row with no recorded map (`map_name ''`): it means "the active map" only while
  the tag has no named pose anywhere. A sighting first-surveys only a tag with no
  pose on any map; a second map is surveyed explicitly (`resurvey_apriltag` arms
  it, or Set tag position). Floor transit picks the one other map with the same
  source. See ADR-0040 §7 (amends ADR-0029).
- **RFID (LF, Flipper)** — the 125 kHz pipeline (ADR-0025): flipper_node's
  pose-stamped card reads (`/rfid/reads`, latched depth-50) → companion
  sqlite → `/rfid/registry`. Gated by `/flipper/rfid_enable`. **Being
  replaced by UHF (ADR-0032)** — removal lands once UHF is proven on-robot;
  the Flipper then serves NFC only (ADR-0026).
- **UHF read / registry** — uhf_node's BATCHED pose-stamped EPC Gen2 reads
  from the M7E Hecto (`/uhf/reads`, one message + one pose per 100 ms window,
  rssi/phase/freq raw per read) crossing zenoh to the companion's uhf.db; the
  per-EPC `/uhf/registry` (RSSI-weighted centroid `est_pose` + `spread_m`)
  crosses back. Human-gated by `/uhf/enable`. See ADR-0032.
- **profile (default / tight_tunnel)** — a named set of nav2/slam/realsense
  parameter deltas for a scenario. See ADR-0010.
- **map source (`source`) / drawing map** — each `site.json` map entry is
  `scan` (SLAM-built) or `drawing` (a floorplan rasterized to a grid pair, no
  posegraph); default `scan`, unknown reads as `scan`. A drawing map is a normal
  amcl map: no new runtime. See ADR-0040.
- **effective mode vs `slam_mode`** — `slam_mode` is the site's policy
  (`auto|new|localization|continue`); the effective mode is what it runs as
  (`sites.effective_mode`, fleet_status `resolved_mode`). `auto` →
  `localization` only for a `drawing` active map whose `.yaml` exists, otherwise
  `continue`/`new`; pinned modes pass through. Anything asking "is amcl
  running?" uses the effective mode.
- **source overlay** — amcl parameter deltas applied by the active map's
  `source`, in `scout/config/source_overlays/<source>/` (never `overlays/`,
  which lists operator profiles). Order: base ← `SCOUT_PROFILE` ← source
  (`merged_params_chain`). Only `drawing/amcl.yaml` exists. A cross-source map
  switch restarts slam (amcl params are fixed at launch).
- **drawing sidecar (`<name>.drawing.json`)** — provenance of a drawing map
  (written at install). Its presence on disk marks the map as a drawing map that
  may be replaced in place; a scan map can never be replaced through the install
  API.
- **wall_spans** — scout-skills' read-only lidar measurement of straight walls
  and wall-to-wall spans in `base_link`, needing no map or localization: MCP tool
  and plain `GET :9001/api/wall_spans`. The robot-measured scale reference for a
  drawing; CORS on that one route only, never `/mcp`.
- **floorplan importer** — off-Pi service (`docker/floorplan-importer/`,
  compose service `floorplan_importer` on the companion, `:9004`, HTTP only, no
  auth, CORS `*`) that turns a PDF or IFC into the grid pair and installs it by
  POSTing to the Pi's fleet_status `POST /api/maps/install`
  (`FLEET_STATUS_URL`). Optional: the Pi never depends on it. Operating recipe
  `docs/slam.md` §6.7; service env `companion/README.md`.
- **import job** — one uploaded PDF or IFC held in the importer's memory (≤ 3
  jobs, dropped after 2 h idle or on importer restart; one open IFC model at a
  time). Calibrated (PDF) or sliced per storey (IFC), previewed, then installed.
  Install never activates a map.
- **storey slice** — one IFC storey cut at a plane (storey elevation + slice
  height, default 0.241 m = the lidar height) and rasterized with scale 1. The
  plane cuts every element crossing it, whichever storey owns it; each storey's
  default origin is the lower-left of its own slice bounds.
- **import wizard** — the web UI panel (`webui/importwizard.js`,
  `importflow.js`, `importmath.js`; SITE tab → Import drawing…) that drives an
  import job over plain HTTP on operator clicks, including the robot-measured
  scale via `wall_spans`.
- **scan-over-drawing overlay** — the web UI's live `/scan` drawn over a drawing
  map under localization, with a banner and a `fit N%` alignment hint
  (`webui/scanlayer.js`, `scanoverlay.js`). Not localization proof; never blocks a
  goal. Distinct from a *source overlay* (amcl parameters).
- **robot profile (robot_profile.yaml)** — the cross-surface SSOT for velocity
  caps, floors, rates, topic names, LED modes, status names, thresholds. Read by
  the ROS nodes, scout-skills (ro mount) and the webui (compose bind-mounts the
  SSOT over a tracked comment-only placeholder in /webui — no value copy since
  2026-08-24). Distinct from a *profile* above.
- **wire format** — every cross-boundary status payload, pipe grammar or JSON,
  formatted/parsed only by `scout.core.status` and frozen by SC9
  (test_status.py). Nodes may not `json.dumps` a status inline.
- **overlay volume** — the `ros_overlay_install` named volume holding the built
  workspace; seeds once from the image. See ADR-0005.
- **elevator call / callStatus** — one Schindler RBL trip: POST /calls →
  `Enter` (door open, elevator waits on the robot) → confirm → ride → `Exit` →
  confirm → `Done`. Door-open windows are elevator-timed; PORT aborts lapsed
  calls. Cancel = DELETE (→ `Aborted`); a robot inside the car must POST a new
  call. See ADR-0030.
- **ride** — the orchestrated elevator trip run by `elevator_ride`
  (schindler-rbl SDK state machine + scout-skills rosbridge adapter), phases
  `nav_to_door → calling → wait_car → board → confirm_enter → riding →
  exit_move → confirm_exit`. Board/exit are dead-reckoned `run_move` — the car
  interior is unmapped.
- **floorNumber vs floorLabel** — RBL floorNumber is the 1-based index among
  SERVED stops, NOT the displayed label: sandbox floorNumber 2 = label "0"
  (Lobby). The off-by-lobby bug. Map via `elevator_floors`; elevator.json
  floor keys are floorNumbers.
- **PORT Gateway / sandbox** — the on-site RBL endpoint (mTLS client cert,
  private CA) and its shared cloud twin sandbox.schindler.com (full-chain PEM
  required, `identity` required on calls, constant sim traffic = `Busy`
  rejections are normal).

## System map (nodes → topics/services)

Core stack (`robot.launch.py`, compose `robot`):

| node | in | out | srv |
|---|---|---|---|
| roboclaw_driver | `/cmd_vel_trac` | `/wheel_odom`, `/roboclaw_status`, `/joint_states` | — |
| twist_mux_auto | `/cmd_vel_skills`, `/cmd_vel` (nav2) | `/cmd_vel_auto` | — |
| collision_monitor (nav2) | `/cmd_vel_auto`, `/scan`, `/cliff/stop_points` | `/cmd_vel_safe` | — |
| twist_mux (final) | `/cmd_vel_{joy,web}`, `/cmd_vel_safe`, `/cmd_vel_stop` + `/estop` | `/cmd_vel_out` | — |
| traction_monitor | `/cmd_vel_out`, `/roboclaw_status` | `/cmd_vel_trac`, `/traction/{status,derate_*}` | — |
| collision_polygon_manager | `/cmd_vel_auto` | `/collision_monitor/{bypassed,zone_mode}` (latched) | `bypass_{engage,release}` |
| estop | — | `/estop`, `/cmd_vel_stop` | `/estop/engage`,`/estop/release` |
| battery_monitor | `/roboclaw_status` | `/battery` | — |
| gyro_calibrator | `/camera/camera/imu` | `/imu/data` | — |
| ekf_filter_node | `/wheel_odom`,`/imu/data` | `/odom` (+ odom→base_link TF) | — |
| led_node | — | (APA102 SPI) | `/set_led_mode` |
| led_status | `/battery`,`/estop`,`/connected_clients` | → `/set_led_mode` | `/set_user_led` |
| joystick_teleop | joydev | `/cmd_vel_joy` | — |
| patrol_capture | color, `/battery`,`/map`,`/coverage_box` | (nav2 goals),`/patrol_status`,`/patrol_route` | `/patrol/{mark,clear,start,stop}` |
| link_watchdog | `/goal_pose`,`/route_poses`, action status | `/goal_pose`, cancels | — |
| tilt_monitor | `/imu/data` | `/tilt_alarm`,`/explore/resume`, cancels | — |
| cliff_detector | depth cloud, TF | `/cliff/points`,`/cliff/stop_points` | — |
| flipper_node | Flipper USB CDC | `/flipper/status` (latched),`/rfid/reads`,`/nfc/reads` (latched-50) | `/flipper/{rfid_enable,nfc_enable,cli}` |
| uhf_node | M7E USB serial | `/uhf/status` (latched),`/uhf/reads` (batched, latched-50) | `/uhf/enable` |
| apriltag (+throttle/relay) | 2 Hz color | `/detections` + tag TF | — |
| nav_manager | both actions' status+feedback | `/nav_state` (latched), `/explore/resume` | `/nav/cancel` |
| health_monitor | `/battery`,`/tilt_alarm`,`/roboclaw_status`,`/traction/status`,`/collision_monitor/*`,`/flipper/status`,`/uhf/status`,`/cliff/stop_points` | `/diagnostics` | — |
| bag_recorder | — | `/record/active`,`/record/path` (latched) | `/record/{start,stop}` |
| wheel_joint_relay | `/joint_states` | wheel TF | — |

Nav bookkeeping (`node_util`): `NAV_ACTIONS` covers both bt_navigator actions;
`ACTIVE_STATUSES = (1, 2, 3)`; every cancel path goes through
`make_cancel_clients` + `cancel_nav_goals` (SC11 bans
`cancel_all_goals_async`). link_watchdog's stash uses the narrower
`RESUMABLE_STATUSES = (1, 2)` — a canceling goal is not worth resuming.

Other stacks: `slam` (slam_toolbox → `/map`, map→odom), `nav2`
(navigation_launch.py → `/cmd_vel` at lowest auto-mux priority),
`foxglove_bridge`, `rosbridge`, `webui`, `ros_mcp`, `scout_skills` (MCP over
rosbridge :9001), `fleet_status`. Companion (over zenoh, ADR-0022): `rtabmap`,
`detector` (YOLO world model), `inspection_recorder`, `rfid_recorder`
(primary RFID DB), `nfc_recorder`, `uhf_recorder` (primary UHF DB + centroid
solver, ADR-0032), `captioner`, its own `fleet_status`.

## Files on the Pi (bind-mounted `./` into the container)

All per-location state lives in `sites/<name>/` behind the `sites/active`
symlink (ADR-0023); switch sites from the webui Site panel. Per site:

- `sites/<name>/site.json` — v2 (ADR-0029): display name, slam_mode policy,
  `maps` dict of labeled maps (label/floor/`source`/map_start_pose) +
  `active_map`. v1 (`default_map`) is normalized on read, upgraded on the next
  write. `source` is `scan` or `drawing` (ADR-0040).
- `sites/<name>/maps/waypoints.json` — named waypoints + routes (ADR-0011);
  waypoints carry the map they were saved on (ADR-0029).
- `sites/<name>/maps/tags.db` — AprilTag registry (sqlite): `tags` = identity,
  `tag_poses` = one surveyed pose per tag **per map** (schema version 3, the
  floor-transit anchor; the legacy pose columns on `tags` are frozen, ADR-0040).
- `sites/<name>/maps/pois.db` — POI registry (sqlite; map-stamped, ADR-0038).
- `sites/<name>/maps/*.posegraph`,`*.data` — slam_toolbox serialized maps.
- `sites/<name>/maps/<name>.yaml`,`.pgm` — the occupancy-grid pair amcl +
  map_server load (ADR-0028). A scan map's is written by Save Map beside its
  posegraph; a **drawing map has only this pair** (+ sidecar), installed via
  `POST /api/maps/install` (ADR-0040).
- `sites/<name>/maps/<name>.drawing.json` — drawing-map provenance sidecar
  (marks it replaceable in place).
- `sites/<name>/maps/<name>.grid.json` — the building's *structural column
  grid* (webui Grid tool, `gridstore.py`) — unrelated to the occupancy grid.
- `sites/<name>/maps/zones.json` — keepout/speed zone polygons (ADR-0019;
  currently no manager node regenerates the derived masks).
- `sites/<name>/rfid.db`, `nfc.db` — LF/NFC read logs (companion, ADR-0025/0026).
- `sites/<name>/uhf.db` — UHF read log incl. rssi/phase/freq per read + the
  centroid registry source (companion, ADR-0032).
- `sites/<name>/captures/<runstamp>/` — patrol photos + manifest.
- `sites/<name>/captures/bags/<UTC>/` — rosbags from bag_recorder (ADR-0017).
- `sites/<name>/elevator.json` — Schindler RBL building topology: equipment
  numbers, floorNumber→door-waypoint mapping, sides, board/exit distances
  (ADR-0030; hand-authored, schema in docker/scout-skills/elevator_config.py).
  Deployment identity lives in `.env` + gitignored `secrets/schindler/`, not
  here.

All gitignored; migrate a pre-sites checkout once with
`python3 scripts/migrate_sites.py`.

## Status wire formats (std_msgs/String — `scout.core.status` owns ALL of them)

Pipe grammars (split on `|`):

- `/patrol_status`: `idle|<n>` | `<state>|<n>|<i>/<n>` | `plan|<text>`
- `/nav_state`: `idle` | `<status_name>|<dist 2dp or empty>|<recoveries>`
  (ADR-0018). `NAV_BUSY_STATES = (accepted, driving, canceling)` is the
  "goal in flight" set; app.js carries a frozen literal copy.
- `/traction/status`, `/flipper/status`, `/rfid/reads`, `/nfc/reads`,
  `/uhf/status`, `/uhf/reads`: JSON, serialized with sort_keys by
  `core.status` formatters (exact strings frozen).
- `/roboclaw_status` (driver-owned JSON): envelope parsed only via
  `core.status.parse_roboclaw_status`.
- `/rfid/registry`, `/nfc/registry`, `/uhf/registry`, `/world/objects`,
  `/world/registry`: JSON produced on the companion (cannot import
  core.status); consumers parse defensively.

Kept as strings deliberately (ADR-0012); consumers on both sides of the
rosbridge/zenoh boundaries parse them, so the formats are frozen by tests —
`scout.core.status` owns every grammar, `scout/test/test_status.py` (SC9) pins
the exact strings, and `test_conventions.py` bans inline `json.dumps` in nodes.

## Conventions (machine-enforced — ADR-0013)

`ruff check .` + `cd scout && pytest` is the definition of done; CI runs both
off-ROS. Repo-specific rules are structural tests (SC1–SC12 in
`scout/test/test_conventions.py`, `test_profile_constants.py`,
`test_status.py`, `test_companion.py`, `test_cliff_cm_coupling.py`):
run_node-only mains, sensor QoS on sensor topics (Pi AND companion), TF via
node_util, Twist publishers only in cmd_vel_source/estop/traction_monitor,
no hand-rolled quaternions, one owner for the bind-mount path, core modules
adopted+tested, profile values never re-hardcoded (recursive scan incl.
companion), wire formats frozen + no inline json.dumps (SC9), deliberate
copies frozen (SC10: skills geometry, detect.py byte-identity,
_median_depth_m, waypoint schema version, companion QoS/regex/idle-state
mirrors), async-only clients + cancel via node_util (SC11), and the
cliff↔collision_monitor stop arithmetic (SC12). Failure messages state the
fix; waivers are reasoned `# noqa` / `ALLOW` entries / `profile-exempt:`
comments, all reviewed as code.
