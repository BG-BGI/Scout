# SLAM on Scout — pipeline, measurement, tuning, upgrade path

slam_toolbox 2.6.10 (apt), config `scout/config/slam.yaml`, launch
`scout/launch/slam.launch.py`. Operating recipes and the mode/serialization
traps live in CLAUDE.md (SLAM section) — this doc covers mechanism, measured
quality, tuning rationale, and where to go next. Drawing-sourced (floorplan)
maps: operating recipe in §6 (decisions: ADR-0040).

## 1. How this pipeline actually works

```
/scan (11.7 Hz, ~1590 pts/rev)──┐
odom→base_link (EKF: wheel vx + gyro vyaw, 30 Hz)──┤
base_link→laser (URDF, incl. the 180° mount fix)──┴─→ slam_toolbox ─→ /map (0.5 Hz raster)
                                                        │              /pose
                                                        └─→ map→odom (correction TF)
```

- **Keyframe gating.** A scan becomes a graph node only after 0.3 m of travel
  or 0.3 rad of rotation (and ≥0.5 s since the last). Everything below is
  per-keyframe work; driving style directly sets CPU load.
- **Scan matching.** Each keyframe is correlation-matched against the last 10
  keyframes (`scan_buffer_size`) inside a 0.5 m search window at 0.01 m
  resolution, seeded by the EKF odometry delta. The match result — not raw
  odometry — becomes the node's pose and an edge in the pose graph.
- **Loop closure.** Continuously: candidate nodes within 3.0 m
  (`loop_search_maximum_distance`) whose chain is ≥10 nodes away get a
  coarse match (response ≥0.35, variance ≤3.0) then a fine match (≥0.45).
  Passing adds a constraint edge and triggers a Ceres solve
  (SPARSE_NORMAL_CHOLESKY) that re-optimizes the whole graph — the map
  visibly "snaps" when this fires.
- **map→odom** is the difference between the graph's opinion of base_link and
  the EKF's. It is a slowly-varying correction; all fast motion rides on the
  EKF. Published every 20 ms, stamped 0.8 s into the future
  (`transform_timeout`) because Ceres solves block the publish loop on a
  loaded Pi — see the long comment in slam.yaml before "fixing" either.

**Where odometry quality enters.** The EKF prior seeds every scan match and
weights every odometry edge. A wheel_radius scale error (the deflated-tire
situation) biases the translation prior by the same percentage — at 0.3 m
keyframe spacing a 5% error is 1.5 cm, well inside the 0.5 m search window,
so scan matching absorbs it and **the map stays metric (scale comes from the
lidar, not the wheels)**. The costs are quieter: worse initial guesses
(more correlation work, occasional wrong-basin matches in self-similar
corridors), systematically stretched odometry edges fighting the scan-match
edges in every Ceres solve, and inflated map→odom corrections. Fix the
radius first; then measure.

## 2. Departures from upstream defaults (and why)

| Param | Ours | Upstream | Why |
|---|---|---|---|
| `base_frame` | base_link | base_footprint | URDF has no base_footprint; default fails every TF lookup |
| `min/max_laser_range` | 0.15 / 16.0 | 0.1 / 20.0 | This unit's real limits; the "0.1 m exceeds 0.2 m" startup warning is a float32/float64 printf artifact — ignore, do not raise |
| `minimum_travel_distance/heading` | 0.3 / 0.3 | 0.5 / 0.5 | Small robot that pivots a lot; 0.5 rad = 29° between keyframes. Judgement, not measurement. First lever to relax under CPU pressure |
| `map_update_interval` | 2.0 | 5.0 | Map fills in visibly while driving. The one periodic cost scaling with map AREA — second lever |
| `transform_timeout` | 0.8 | 0.2 | Future-stamps map→odom to cover Ceres blocking the publish loop (measured −0.606 s worst stall). Symptom fix for Pi load, deliberately |
| `enable_interactive_mode` | false | true | rviz-only feature; Foxglove is the viewer |
| `scan_queue_size` | 1 | — | Drop untransformable scans rather than queue stale ones |

Everything else (solver, correlation/loop search spaces, penalties) is
upstream defaults — slam_toolbox is tuned around them.

## 3. Map quality / drift measurement protocol (run AFTER wheel_radius recal)

Instrument from the **Mac** over rosbridge ws://pi:9090 — a `docker compose
run` container on the Pi during navigation starves the stack and aborts goals
(CLAUDE.md nav2 section).

1. Mark the start pose physically (tape at both wheel centerlines).
2. `localization` mode on the current saved map, or `new` if remapping.
3. Drive a closed perimeter loop (~15–20 m) with `patrol`/`go_through`,
   ending back on the mark.
4. Log during the drive (script subscribes `/tf`, filters map→odom):
   correction norm + yaw over time.
5. After: physical return error (tape vs `/pose`), loop-closure count
   (`docker compose logs slam | grep -ci loop`), CPU (`top` one shot).
6. Repeat once. Repeatability matters more than any single number.

| Metric | Baseline (2026-08-03, inflated tires) | Loop 1 (fresh 5-min map) | Loop 2 (same map, 10 min old) |
|---|---|---|---|
| map→odom net correction over loop | 0.30 m / 2.1° per ~17 m | **0.66 m / 12.2°** (young-graph re-solve mid-run) | **0.05 m / 2.2°** — 10× calmer one loop later |
| Return-to-mark error (tape) | — (never measured) | 20 cm total − ~14 cm tolerance stop-short ≈ **6 cm real** | **≈0 — dead on the marks** (tape precision) |
| map→odom idle jitter | — | 0.0000 m / 0.000° over 30 s (measured same day) | — |
| Loop closures per loop | — | ≥1 (−12° frame re-solve) | small (+2.2°) |

Protocol completed 2026-08-14, both loops ~6 m in the office main room,
deflated tires, 0.6 m/s cap. **Conclusions:** (a) fused localization returns
the robot to a physical mark within tape precision once the pose graph has a
few minutes of history; (b) a fresh map's frame swings hard (0.66 m/12°) for
the first loops — one more reason not to point-drive a young map (see the
lesson above); (c) deflated tires do not visibly degrade loop performance —
gyro yaw + rear encoders carry it. Earlier same-day datum on the day's messy
long-session map: 2–16 cm real return error per ~15–20 m including a ~40°
re-solve. The 2026-08-03 baseline is matched or beaten.

**Run 1 of the closed-loop protocol (2026-08-14 office/hallway): INVALID — three
compounding failures, all instructive.**
1. **Mislocalization in the corridor**: map-frame yaw flipped ±3 rad between
   5 s samples and the pose teleported (12.2,−1.3)↔(9.8,−7.9) while driving a
   long glass-walled hallway — classic corridor aliasing plus a map already
   distorted by the day's clutter churn.
2. **Rear-stall/front-spin regime** (operator observed, recurring): in high-
   torque phases the weighted rear wheel stalls, the velocity loop winds duty
   to max, and the unloaded soft front freewheels fast. Wheel odometry is
   garbage exactly then, which feeds failure 1.
3. **WiFi dead zone ended the run**: the robot drove out of coverage with a
   goal latched — no software stop exists at that point (bt_navigator replans
   and streams cmd_vel locally, forever). Operator physically recovered it.

**Mitigation shipped same day: `link_watchdog` node** (in `robot.launch.py`) —
gateway unreachable 5 s → cancel + stash active nav goals; link back within
2 min → re-dispatch; longer → forget. Live-tested to the pause deadline
(fake-dead link via `gateway_override`); the stash/resume leg is exercised
only in code review — the first test goal arrived before the 5 s deadline and
the operator waived a retest. Redo the drift protocol inside coverage, on a
fresh map, after the front/rear differential tire-pressure experiment.

**⚠ Measured lesson (2026-08-14): do NOT point-drive during active mapping.**
A graph optimization mid-goal rotated the entire map frame ~40°; goals held
their (now-wrong) coordinates, the planner chased teleporting targets
(distance_remaining oscillating 10→32 m on a 7 m route, poses never
"passed"), and only nav_cancel ended it. For LLM/Magnus point-coordinate
operation, run slam in `localization` mode on a saved map; `new`/`continue`
sessions are for exploring. If mapping is unavoidable, re-fetch map+pose
immediately before each dispatch and keep routes short — the frame can still
move mid-goal.

## 4. Tuning recommendations (apply only what §3's numbers justify)

Ranked by expected value:

1. **`loop_search_maximum_distance: 3.0` → 5.0–6.0** if corrections grow with
   loop length (§3 metric 1). 3 m is small for house-scale loops — a closure
   candidate must already be within 3 m *by odometry* to be considered, so
   accumulated drift can push a genuine revisit out of reach. Cost: candidate
   search CPU per keyframe.
2. **`minimum_travel_distance/heading: 0.3` → 0.4–0.5** if `Control loop
   missed` bursts or map→odom stalls persist — fewer keyframes is the
   cheapest real load cut (the transform_timeout comment names this the next
   knob).
3. **`ceres_loss_function: None` → `HuberLoss`** only if §3 ever shows a map
   "snap" to a wrong alignment (a bad loop closure is currently unbounded in
   influence).
4. **`resolution: 0.05` → 0.025** only for pipe work, and remember the global
   costmap inherits this (nav2.yaml's own resolution key is cosmetic).
   ~4× raster cost and posegraph files already run tens of MB.
5. **Leave alone:** correlation search space (matched to the 0.5 m window the
   EKF can always hit), `transform_timeout: 0.8`, laser ranges, base_frame.

## 5. Upgrade path: RTAB-Map / 3D

**Verdict: keep slam_toolbox for live navigation; add RTAB-Map offboard if
the mission needs 3D capture. Do not run RTAB-Map on the Pi.**

> **⟲ The bag-record-then-Mac workflow below is SUPERSEDED (2026-08-19):**
> RTAB-Map now runs live on the companion over shared DDS — see ADR-0020/0021
> and `companion/`. The "not on the Pi" verdict stands (it was CPU, not
> transport). The bag shape remains useful as the offline replay gate.

- What it buys: RGB-D 3D maps (cloud/mesh — the BIM/as-built deliverable),
  appearance-based loop closure (works where 2D scan geometry is ambiguous —
  long uniform pipes), multi-session mapping.
- Why not onboard: the Pi 5 has ~1 core of headroom and Ceres solves already
  starve the TF publish loop (§2, transform_timeout). RTAB-Map's visual
  feature extraction + graph optimization wants a core-plus by itself, and
  memory grows to GBs at house scale. `ros-humble-rtabmap-ros` exists for
  arm64, so nothing blocks an experiment — but every measured contention
  incident here says it ends in missed control loops and aborted goals.
- **Offboard shape that works:** record a bag during a patrol
  (`/camera/camera/color/image_raw`, aligned depth, `camera_info`, `/odom`,
  `/tf`, `/tf_static`, `/scan`) and run rtabmap on the Mac against the bag →
  a 3D model per patrol with zero Pi runtime cost beyond bag I/O. Transfer in
  retryable chunks (the Pi ethernet flaps under sustained load). The existing
  patrol_capture flow is the natural trigger point.
- Re-evaluate onboard only if compute changes (Jetson-class or offload).

## 6. Drawing-sourced maps (floorplan) — operating recipe for what exists today

Decisions and limits: ADR-0040. **Nothing in this section has run on the robot
yet** — steps are written from the merged code; the first on-robot pass will
correct them. A drawing map is an ordinary amcl grid map (`<name>.yaml` +
`.pgm`, no posegraph) whose `site.json` entry has `source: drawing`. The
importer that produces the pair from a PDF/IFC is not merged yet (see the stub
at the end), so today you install a pair you made yourself.

### 6.1 Install a drawing grid via the API

`POST http://<pi>:9003/api/maps/install` (fleet_status; only on the Pi, where
`SITE_SCAFFOLD=pi`, otherwise 404). Writes into the **active site**, never
changes `active_map`, never touches a scan map. Body is one JSON object:

```bash
# floor1_plan.yaml (nav2 map_server yaml; `image:` is ignored and rewritten)
#   image: floor1_plan.pgm
#   mode: trinary
#   resolution: 0.05
#   origin: [-12.0, -8.0, 0.0]
#   negate: 0
#   occupied_thresh: 0.65
#   free_thresh: 0.196
python3 -c '
import base64, json
print(json.dumps({
    "name": "floor1_plan",              # ^[a-z0-9][a-z0-9_-]{0,31}$, no leading "_"
    "label": "Floor 1 (plan)", "floor": 1,
    "map_start_pose": [0.0, 0.0, 0.0],  # where amcl starts; seed the real pose later
    "yaml": open("floor1_plan.yaml").read(),
    "pgm_b64": base64.b64encode(open("floor1_plan.pgm", "rb").read()).decode(),
    "sidecar": {"source_file": "A-101.pdf", "note": "hand-installed"},
    "source": "drawing",                # optional; only "drawing" is accepted
}))' > install.json
curl -sS -X POST http://<pi>:9003/api/maps/install \
     -H 'Content-Type: application/json' --data-binary @install.json
```

- pgm: binary P5, maxval 255, ≤ 4000×4000 cells, ≤ 16 MiB decoded. Use the
  importer's grid convention: walls **0**, free **254**, unknown **205**,
  `free_thresh: 0.196` (so 205 stays unknown).
- `200` → `{"ok": true, "name", "replaced", "source": "drawing", "width",
  "height", "resolution", "origin", "bytes", "entry", "active_map"}`.
  Installing the same name again **replaces it in place** (that is how you
  recalibrate) — only because its `.drawing.json` sidecar exists.
- Refusals leave nothing changed: `400` bad field / name / any `force` field
  (there is no `force`); `409` a `.posegraph`/`.data` exists for the name, the
  entry is not a drawing, files/entry exist without a sidecar, no active site;
  `411`/`413` missing/too large body; `422` bad pgm/yaml.
- Check it: `GET http://<pi>:9003/api/sites` shows the map with `source`,
  `grid: true`, `posegraph: false`; `GET /api/maps/floor1_plan.pgm|.yaml`
  returns the raw files.

### 6.2 Activate, and what `auto` does

Activate from the webui Site panel (Activate), or the scout-skills
`switch_map(name)` tool. Both pick the right path (6.4). Under the default
`slam_mode: auto` the **active map's `source` decides the mode**:

| active map | `auto` runs as |
|---|---|
| `drawing`, `<name>.yaml` exists | `localization` (amcl + map_server on the grid) |
| `drawing`, no grid | `continue` if a posegraph exists, else `new` |
| `scan` | `continue` if a posegraph exists, else `new` — never localization |

`GET /api/sites` reports it as `resolved_mode` next to the raw `slam_mode`. A
pinned `slam_mode` (e.g. `localization`) is used as set. In localization the
drawing amcl overlay (`scout/config/source_overlays/drawing/amcl.yaml`) loads on
top of any `SCOUT_PROFILE`; scan maps keep the unchanged `amcl.yaml`.

### 6.3 Set pose — mandatory

There is **no global localization on a drawing** (ADR-0040): amcl starts at the
map's `map_start_pose` and will not find the robot by itself. After the map is
live, use the webui **Set pose** (tap the position, then the heading →
`/initialpose`),
or show the robot a surveyed AprilTag (6.5). Then **look at the scan against the
map** before sending a goal — a tight numeric correction does not prove the
pose. (The scan-over-drawing overlay in the web UI is decided but not built yet:
UI pending.)

### 6.4 Switching maps

- **Same source** (drawing → drawing, scan → scan under localization): live
  `/map_server/load_map` swap, ~1 s, pose re-seeded at the new map's
  `map_start_pose`.
- **Across sources** (scan ↔ drawing): amcl's parameters are fixed at launch, so
  it **POSTs `active_map` then restarts slam + behaviors** (~20 s; driving and
  camera stay up). When amcl comes back the tag relocalizer is re-armed
  automatically once a *fresh* `/amcl_pose` appears (up to ~90 s); if it does not,
  the UI/`switch_map` says **"Set pose after restart"** — do that.
- Refused before any change: a target that will run amcl but has no grid.
- A floor-transit tag that points at a map of the *other* source is never
  switched live: tag_relocalizer logs one throttled line; switch manually.

### 6.5 Survey one tag on a second map of the same floor

A tag's pose is stored **per map**, so the same physical tag can be surveyed on
`floor1` (scan) and `floor1_plan` (drawing) and each map relocalizes from its
own. A sighting alone only first-surveys a tag with **no pose on any map**, so
for the second map do it explicitly, with the robot **verifiably well
localized on the active map** and the tag in view:

1. Activate the second map (6.2) and set pose (6.3).
2. `resurvey_apriltag(name)` (scout-skills) — clears the tag's pose on the
   *active map only* and arms the next sighting to survey it there; the watcher
   records it within a couple of seconds of the tag being in view (the passive
   watcher is on by default; `apriltag_watch`). Arming is in memory: a
   scout-skills restart disarms — call it again.
   **Or** webui **Set tag position** (tap the tag, tap where the robot stands in
   front of it): writes the active map's pose directly, and replaces the tag's
   waypoint (the response says which map's waypoint it replaced).
3. `list_apriltags` shows the pose for the active map in `map_x/map_y/map_yaw`
   and every surveyed map in `poses`. Surveying map B never touches map A.

Floor transit: a tag with a pose on the active map relocalizes there; with none
but poses elsewhere, the robot transits to the **one** other map of the same
source, and to none when that is ambiguous (a warning names the candidates).

**Legacy-pose caveat.** A pose surveyed before per-map poses (stored with no
map, shown with `map_name: null`) applies to **whichever map is active** until
the tag has any named pose. If you add a drawing map of a floor whose tags were
surveyed on the scan map and the drawing's frame differs, **re-survey those tags
on the drawing map (6.5)** — until then the old pose is also used there.

### 6.6 Measure a wall span with the robot (`wall_spans`)

Robot-measured scale reference: straight walls and wall-to-wall spans from the
live lidar, in `base_link`, with no map or localization. Subscribe-only; nothing
is commanded. Park the robot with two **non-parallel** wall pairs in view (e.g. a
corridor and its end wall), then:

```bash
curl -sS 'http://<pi>:9001/api/wall_spans?scans=3&max_range_m=10&min_length_m=0.6' \
  | python3 -m json.tool
```

Query (all optional): `scans` 1–10 (default 3), `max_range_m` 0.5–30 (10),
`min_length_m` 0.1–20 (0.6; shorter clutter is dropped), `require_stationary`
(default true), `include_points` (default false; adds the decimated scan points
as `points_xy`). Same JSON from the MCP tool `wall_spans`.

Response: `ok`, `frame` (`base_link`), `stamp`, `scans_used`, `stationary`,
`flags` (read these), and:

- `walls[]` — `id` (`w0`, longest first), `bearing_rad`, `distance_m`,
  `spread_m` (disagreement across scans), `length_m`, `points`, `rms_m`, `p0`,
  `p1` (segment ends in `base_link`).
- `pairs[]` — `a`, `b` (wall ids), `span_m` (perpendicular distance between two
  near-parallel walls with the laser between them), `spread_m`, `axis_rad`,
  `parallel_err_deg`.

Using it: compare each `span_m` with the same two walls measured on the drawing.
**Take the top pair per axis.** `pairs` is sorted by the shorter wall's length
(longest first), then by spread, and a doorway splits one wall into two segments,
so the same axis often shows near-duplicate pairs with almost the same span —
the first pair on an axis is the best-supported one. Calibration wants two
pairs on roughly orthogonal axes; the `flags` say when a wall has no partner or
no orthogonal pair exists.

Errors, always `{"error": "..."}`: `400` bad parameter; `409` the robot is
moving — park it and retry, or pass `require_stationary=false` (the moving
reason is then added to `flags` and the spans are less trustworthy); `503` no
`/scan` traffic or rosbridge down. The route answers CORS `*` for the web wizard;
`/mcp` never does. The stationary threshold (0.04 m scan-to-scan) is untuned on
the real lidar.

### 6.7 Import wizard (pending)

*Stub — a later docs pass fills this in once the importer HTTP server and the web
wizard merge:* PDF upload and page choice, the scale proposal and the three
calibration methods (two-click, robot-measured via 6.6, typed scale), rotation
and origin, occupancy preview, IFC storey picker, naming/floor, and the
scan-over-drawing overlay check. Until then 6.1 is the only way in.
