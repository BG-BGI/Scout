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
`.pgm`, no posegraph) whose `site.json` entry has `source: drawing`. The usual
way in is the web import wizard (6.7: PDF or IFC → importer → installed map);
6.1 is the API underneath it and the way to install a pair you made yourself.

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
pose.

The web UI does that for you: while the active map is a drawing **and** slam
runs as localization, the map view draws the live `/scan` over the grid (green
points, grey when no scan for 2 s) under a banner "Confirm the scan sits on the
walls before sending a goal" with `fit N%` (share of scan points within 0.15 m
of a wall cell). It subscribes to `/scan` at 2 Hz only while that holds, never
blocks or delays a goal, and the fit is an **alignment hint, not localization
proof** — a wrong pose in a repetitive building can score well. **Dismiss**
hides the banner for that map until you switch maps (or the tab session ends).
The scan is placed with a hard-coded laser mount (0.073 m forward of
`base_link`, yawed π; the page does not read `/tf_static`): a scan that looks
mirrored front-to-back means that constant is wrong. Built from
`webui/scanlayer.js` + `scanoverlay.js`; **not yet seen on the robot**.

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

### 6.7 Import wizard (PDF / IFC → installed drawing map)

Written from `webui/importwizard.js` / `importflow.js` / `importmath.js` and the
HTTP contract in `docker/floorplan-importer/server.py`'s docstring. **Not yet run
against the real importer image, companion or robot** (6.10). Needs the importer
running (compose service `floorplan_importer`, `:9004`, `companion/README.md`);
the operator's **browser** talks to it directly, so the browser must reach the
companion box on `:9004` as well as the Pi on `:9003` and `:9001`.

**Open it:** SITE tab → "Floors & saving" → **Import drawing…**. Close = the
panel's Close button. Nothing is sent until you click something (no polling, no
rosbridge topic).

**Importer address** (resolved when the panel opens):

1. A remembered address (browser localStorage `scout.importer.base`) wins.
2. Else the companion host fleet_status reports (`GET :9003/api/companion` →
   `{configured, host}`) with port **9004**.
3. Else blank: type `http://<box>:9004` (scheme optional, `/api` is appended).

The field is editable. Leaving it, or **Retry**, saves it and checks
`GET /api/health`: "Importer ready at … (version)" or "Importer not reachable
at …". An empty field + Retry forgets the saved address and resolves again.
Health is checked only on panel open and Retry.

**Rules for both flows**

- One importer call at a time; the panel shows what is running. A `429 busy` is
  retried up to 5 times after `Retry-After` ("Importer busy — retrying in N s").
- Jobs live in importer **memory**: at most 3, dropped after 2 h idle or when the
  importer restarts. A dead job answers "no such import (unknown or expired)":
  upload again.
- **Install never activates and never overwrites a scan map.** fleet_status
  refuses a name that is a scan map (409, shown as is). Installing a drawing's
  name again replaces it in place — that is how you recalibrate. The wizard sends
  no `map_start_pose`, so fleet_status stores `[0, 0, 0]` (the map origin you
  chose).
- Name `a-z 0-9 _ -`, starts with a letter or digit, ≤ 32 chars; floor is an
  optional whole number.
- Install errors: `503` the importer has no `FLEET_STATUS_URL`; `502` fleet_status
  unreachable / timed out (the job is kept, retry; the message names the
  upstream URL); fleet_status's own `409`/`422` are shown unchanged.
- After a successful install the Site list refreshes and an **Activate** button
  appears (IFC: one per installed storey). It is the same path as the Site
  panel's Activate (confirm dialog; live swap or cross-source restart, 6.4).
  **After activating, seed the pose (6.3)** — amcl starts at the map's
  `map_start_pose` and will not find the robot itself — then check the scan
  overlay.

**PDF flow**

1. Choose the file → **Upload**. The kind is sniffed from the bytes (`%PDF-`).
   One page loads at once; several give "Choose a page…" with each page marked
   `(vector)` or `(image)`.
2. **Scale.** The panel shows the importer's proposal ("Proposed scale: 1 page
   unit = X m (confidence), evidence"). **Accept proposed scale** applies it — it
   is never applied on its own, and the button is off when there is no proposal.
   "This page has no usable scale" means use one of the methods; each runs
   calibrate and draws the preview:
   - **Two taps:** **Tap two points** on a known length (≥ 20 screen px apart),
     type the real metres, **Use this scale**.
   - **Typed scale:** scale text (`1/4" = 1'-0"`, `1:100`) **or** a ratio (real ÷
     paper), not both. For a PDF that is a reduced print also give the full-size
     sheet width × height in inches (must match the PDF's aspect within 3 %).
   - **Robot-measured:** below.
3. **Rotation** (degrees, map +X counter-clockwise from the page's right edge),
   **Tap map origin** (the page point that becomes map (0, 0); default = lower-left
   of the extracted walls), **read as image** (raster extraction; use it when a
   vector page says "no wall geometry" or the walls look wrong). Each change
   after a preview re-runs calibrate and regenerates the grid — it is never
   layered over the old one.
4. **Preview:** the grid is drawn 50 % over the page (faded while out of date),
   with "Scale … m/unit (method), rotation …, Grid W x H cells at 0.05 m;
   interior N %" and any warnings. Check the walls land on the drawing's walls.
   "little enclosed interior (N % …)" means the walls do not close or an opening
   is ≥ 1.2 m wide (`close_m`), so the inside reads unknown. The warning says
   "raise close_m", but the wizard has **no control for it** (the API's
   `close_m`, ≤ 5 m, is reachable only by calling the importer yourself): fix the
   gap in the source or use **read as image** with a cleaner sheet.
   Drag pans, wheel or +/−/Fit zooms; taps only place points.
5. **Install** (needs a current preview and a valid name): `Installed "<name>". It
   is not active yet.` If you changed anything afterwards it says the preview
   moved on: install again to update the same name.

**Robot-measured scale** (read-only; 6.6 has the measurement itself)

1. Park the robot, **stationary**, with two non-parallel wall pairs in view.
2. **Read wall spans** = one plain `GET http://<pi>:9001/api/wall_spans` with no
   query string (6.6 defaults; stationary required; the Pi host is the page's own
   host). The panel lists the top pair per axis ("W1: 4.213 m at 90°") and the
   lidar's `flags` as "Lidar note: …". Fewer than two axes → move the robot so
   two directions are in view.
3. For each pair press **Mark on page** and tap one wall, then the opposite wall
   of the **same** pair on the drawing.
4. **Use robot scale** (needs ≥ 2 marked pairs that differ by ≥ 20°; the importer
   answers 400 otherwise). Spans that disagree by more than 1 % are **not** an
   error: the mean is applied and a warning is shown — read it.

Errors: `409` "The robot is moving. Park it and retry." (the wizard has no
override; `require_stationary=false` is curl-only, 6.6); `503` "No lidar scan yet
(/scan is not publishing)"; "Could not reach the robot at <url>" = scout-skills
`:9001` down or the page host is not the Pi.

**IFC flow**

1. **Upload** the `.ifc` (STEP text, sniffed from `ISO-10303-21`; `.ifczip` /
   `.ifcxml` are refused; `415` if the image was built without IFC support).
   The model is metric: there is no scale step. Storeys are listed in floor
   order: tick box, `label (elevation m)`, map name (suggested from the label,
   editable), floor # (by elevation), **Show**, status.
2. Optional **slice height** (blank = 0.241 m above each storey's elevation, the
   lidar plane) and **rotation**; both apply to every storey, and changing them
   marks sliced storeys "settings changed: slice again". **Slice ticked
   storeys** makes one importer call per storey, in order; a storey that fails
   keeps its message and the rest go on.
3. **Show** a storey to see its preview (red cross = map (0, 0)). **Tap map
   origin**, then tap its preview: that storey is sliced again with the tapped
   world point as its origin. Without a tap the origin is the lower-left of that
   storey's slice bounds (6.8).
4. **Install ticked storeys.** Every ticked storey must be sliced and current
   ("Slice this storey first"), and names must be valid and unique within the
   batch. Each storey installs as its own drawing map; failures are listed per
   storey, the others go on. Then **Activate <name>** per installed storey.

### 6.8 IFC operating notes

- **Origins differ per storey.** The default origin of a storey is the
  lower-left corner of **its own** slice bounds, so two floors do not share a map
  origin unless you tap the **same physical point** (a column, a stair core) on
  each storey's preview. Do it before installing if the same (x, y) should mean
  the same place on every floor. Slice height and rotation are shared, origin is
  not.
- **A slice cuts everything crossing the plane, whichever storey owns it** (z =
  storey elevation + slice height, by absolute Z): a wall or column spanning two
  floors shows on both. Doors are excluded on purpose (a leaf is solid at 0.241 m
  and would seal the doorway); a window shows only if the plane crosses it.
- **One IFC model is open at a time** (~7× its file in memory). Uploading another
  IFC evicts the open one — and other idle jobs if needed — **before** the new
  body is read, even if the upload is then refused or the file fails to parse;
  `429` if one of them is mid-operation. Install the storeys you want before
  uploading the next model. Upload checks run in this order: Content-Length
  (`411`/`400`/`413`) → importer busy (`429`) → kind sniffed from the first 2 KiB
  (`415`) → that kind's size cap (`413`) → evict → memory gate → stream → parse.
- **Upload cap 80 MiB by default** (`IMPORTER_MAX_IFC_BYTES`), sized for compose
  `mem_limit: 2g` (an open model ≈ 7× its file, a slice peak ≈ 15×). Over the cap
  is `413 {count, limit}` before the body is read; so is a model over the
  3,000,000-triangle face budget. To take bigger files **raise `mem_limit` first**
  (≈ 128 MiB at 3g, ≈ 256 MiB at 6g), then the cap.
- **Memory gate.** The gate judges the resident size **after** the eviction
  above, so replacing one IFC with another normally does not trip it. An IFC
  upload whose size would still push the resident set past 90 % of the container
  limit (current RSS + ≈ 15× the file) is `503` with `Retry-After: 15`: every
  open import is dropped and the process exits itself (code 75; compose
  `restart: unless-stopped` brings it back). Wait ~15 s, Retry, upload again. It
  is the answer to ifcopenshell giving back only part of a freed model's memory,
  so RSS creeps up model by model. It only fires when nothing else is in flight:
  if another import is mid-operation **or another upload is in flight** the
  answer is `429` with `Retry-After: 15` instead — nothing dropped, no restart;
  retry. At startup the importer deletes stale `importer-ifc-*` temp dirs left by
  a previous process. Watch it: `GET http://<box>:9004/api/health` → `rss_mb`
  (null off Linux), `ifc` (false = the image has no ifcopenshell / shapely),
  `jobs`, `busy`.
- **Cached storey grids are capped at 128 MiB per job.** Past that the **oldest**
  sliced storey's grid is dropped (never the one just sliced); the new slice's
  warning names it. A dropped storey is "not sliced" on the importer while the
  panel may still show it sliced, so its install fails with the importer's `409`
  in that storey's row: slice it again, then install. Slicing and installing in
  small batches avoids it.

### 6.9 First-run checklist (operator, in this order)

Mark each pass/fail in the session notes; a fail here is a finding, not a
failure of the operator. Use a scratch name (`test_plan`): the same name can be
installed again to replace it. There is no delete-map button, so a scratch map
stays in the site until its files are removed on the Pi.

1. **Importer down.** On the companion `docker compose stop floorplan_importer`;
   open the panel. Expect "Importer not reachable at …/api/health", Upload gives
   "Could not reach …", the rest of the web UI and the robot are untouched, and
   6.1 still installs. `docker compose start floorplan_importer`, **Retry** →
   "Importer ready". Also check one install with a wrong `FLEET_STATUS_URL` or
   the Pi unreachable from the box: expect `502 … (upstream: …)`, the job kept.
2. **Vector PDF** (a CAD export). Upload; a proposed scale with its evidence
   → check it against one dimension you can measure (two-tap method) before
   **Accept**. Preview overlay: walls on walls, interior % sane. Set origin,
   rotation. Install → Site list shows the map with the `drawing` chip,
   `GET :9003/api/sites` → `source: drawing`, `grid: true`, `posegraph: false`,
   `GET :9003/api/maps/<name>.pgm` returns the grid. **Not activated yet.**
3. **Raster PDF** (a scan or image-only page, marked `(image)`). No proposal:
   "no usable scale"; Accept is off. Calibrate with typed or two-tap; preview;
   install. Also on a *vector* page tick **read as image** and confirm the grid
   regenerates.
4. **Robot-measured.** Robot parked in a corridor with an end wall in view.
   **Read wall spans** → two axes; mark both pairs on the page; **Use robot
   scale**. Compare the resulting scale with the typed / two-tap scale for the
   same sheet and note the difference; read any > 1 % span-spread warning; a 409
   on a robot you know is parked is a finding (the stationary threshold is
   untuned).
5. **IFC.** A small model first (well under the cap). Storeys listed in floor
   order; slice all; tap one common point on two storeys; install two; `GET
   :9004/api/health` → `rss_mb` before/after. Then upload a second IFC from the
   wizard: it discards the first job first (`DELETE`), so `jobs` stays 1 and
   `rss_mb` shows what the freed model gave back.
6. **On the robot, with one of the above.** Activate (confirm dialog), watch the
   restart/live swap (6.4), **Set pose** (6.3), confirm the scan overlay sits on
   the walls and `fit` is plausible, drive a short goal. Anything that disagrees
   with 6.2–6.4 goes back into this document.

### 6.10 Known limits

- **The importer has no auth and answers CORS `*`** (`IMPORTER_CORS_ORIGIN`,
  default `*`): any page open in a browser that can reach `:9004` can upload,
  calibrate and install a drawing map through it. The importer itself cannot
  activate a map or replace a scan map (fleet_status refuses a scan map's name).
  It adds no capability beyond fleet_status `:9003`, which is also CORS `*` with no
  auth (LAN-trust, same as ros_mcp / scout-skills) and *can* activate maps and
  restart containers. Narrow `IMPORTER_CORS_ORIGIN` to the webui's origin if the
  box is reachable by more than the operator.
- **`GET :9001/api/wall_spans` is CORS `*`, read-only** (lidar subscribe only,
  nothing commanded). **`/mcp` is never CORS-open** (it exposes motion tools).
- **Legacy tag poses follow onto a new drawing map** until the tag is
  re-surveyed there (6.5).
- **An IFC boolean-geometry failure can silently seal a doorway:** the geometry
  iterator subtracts openings and a failed subtraction leaves the wall solid
  across the gap, with no error. **Always look at the preview.**
- **The PDF wall classifier has not been validated on a real CAD export** (only
  on Scout-generated sheets). A double-hairline wall gives `kind: none`; use
  **read as image**.
- **ifcopenshell memory give-back on Linux is unverified** (measured only on
  macOS); the memory gate (6.8) is the backstop.
- **Glass and thin walls:** thin glass under one cell (0.05 m) vanishes from the
  grid; glass / curtain wall is solid on the drawing and invisible to the lidar.
- Not yet verified on hardware: the image build on amd64 and arm64, the compose
  caps (`cpus: 1.0`, `mem_limit: 2g`), the scan overlay on the robot, the
  `wall_spans` stationarity threshold, and the real companion → Pi install path
  (the box reaching `http://<PI_IP>:9003`; the zenoh bridge only proves `:7447`).
