# Scout — remaining implementation plan (handoff)

Self-contained plan for the work left after the 2026-08-15 architecture pass.
Written so a fresh LLM + the operator can execute without the original session.
Read this top to bottom once, then work a milestone at a time.

---

## 0. Current state — read first

**Branch:** `web-ui`. Six commits landed, **nothing pushed**:

```
747312e M8: ADRs + CONTEXT.md
da87c78 M6 (part 2): adopt geometry + TF helpers + run_node
02867ce M6 (part 1): pure scout.core package + test suite
61c887a M3: cmd_vel arbitration (twist_mux) + CmdVelSource + e-stop
6f7f270 M2: robot profile — single source of truth
0c9f120 M1: safety + bug sweep
```

**⚠ On top of those 7 commits, an UNCOMMITTED convention pass (ADR-0013) is in
the working tree — commit it first.** It added ruff + structural convention
tests (SC1–SC10) + CI, and while landing them it finished several M6 deferrals.
Confirm with `git status` and land it before starting anything below. What it
did (verified 2026-08-15): `scout.core.status` (status formatters — §5d), the
`battery_monitor` → `scout.core.battery` adoption (§5a), `run_node` in every
node (§5c), `scout/scout/qos.py` (`LATCHED_QOS` + the sensor-QoS convention),
ruff replacing `ament_flake8` (`test_flake8.py` deleted), and moving config
resolution into `robot_profile.py` (`resolve_config` / `resolve_config_dir`,
SC6 — so **M4 does NOT create `launch_utils.py`**).

**What already exists (don't rebuild it):**
- `scout/config/robot_profile.yaml` — cross-surface SSOT (velocity caps/floors,
  cmd_vel topic names, LED modes, GoalStatus names, battery thresholds, occupied
  threshold). Loaders: `scout/scout/robot_profile.py` (ROS; also owns
  `resolve_config*` — the sole home of the `/ros_ws/src/scout` bind path, SC6),
  `docker/scout-skills/robot_profile.py` (mounted, baked fallback), webui
  `fetch()` + mini-parser.
- `scout/scout/cmd_vel_source.py` — `CmdVelSource`, the ONLY Twist publisher
  besides `estop` (SC4). `scout/scout/estop.py` — e-stop node + services.
- `scout/config/twist_mux.yaml` + twist_mux node; roboclaw remapped to
  `/cmd_vel_out`.
- `scout/scout/core/` — pure modules (stdlib+numpy only): `geometry`, `battery`,
  `coverage`, `colors`, `tricks`, `status`. `node_util.py` = `run_node` +
  `lookup_pose2` / `lookup_matrix`. `qos.py` = `LATCHED_QOS`.
- `scout/test/` — bare-pytest suite: the algorithm tests PLUS the ADR-0013
  structural tests (`test_conventions.py` SC1–SC10, `test_profile_constants.py`,
  `test_status.py`, `test_lint.py`). `CONTEXT.md`, `docs/adr/0001-0013`.
- Root `pyproject.toml` + `scout/ruff.toml` + `requirements-dev.txt` +
  `.github/workflows/ci.yml` (ruff + pytest, py310).

**Ground rules (non-negotiable — from CLAUDE.md):**
1. **Deploy is git only.** Never edit tracked files on the Pi. commit → push →
   `git pull` on the Pi → rebuild affected compose services.
2. **The operator gates every physical/motion/rebuild action.** State the ask at
   the end, then stop and wait. Never command motion without explicit per-run
   confirmation (direction, speed, duration, space).
3. **Rebuilding `robot`/`slam`/`nav2`/`scout_skills` restarts the drivetrain
   driver** (a coast, not a brake). Only with a clear floor and operator OK.
4. Terse. The operator is a ROS 2 expert. Findings and numbers, not tutorials.

**Deploy recipe (operator runs on the Pi):**
```bash
# Scout-python-only change (symlink-install, no image rebuild):
git pull && docker compose --profile build run --rm build_package \
  && docker compose up -d robot            # + slam/nav2/scout_skills as touched
# scout-skills change: docker compose build scout_skills && up -d scout_skills
# Dockerfile/image change: docker compose build && (see M5 volume note) && up -d
```

**Mac / CI test workflow (no ROS needed) — this IS the definition of done:**
```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt
.venv/bin/ruff check .                                    # ADR-0013 lint gate
PYTHONPATH=scout .venv/bin/python -m pytest scout/test -q  # algorithm + SC1-10 tests
# scout.core imports only stdlib+numpy (test_core_purity/SC7 enforce it), so this
# runs anywhere. CI (.github/workflows/ci.yml) runs both on every push; plain
# pytest without ruff still works (test_lint skips). `colcon test` on the Pi
# also runs pytest (ruff/structural tests skip when the tool is absent).
```

**Conventions gate (ADR-0013) — every change below must satisfy it.** `ruff
check .` + pytest is the bar; the structural tests fail with the fix in the
message. Rules that touch this remaining work:
- **SC1** a console-script `main()` is `def main(args=None)` delegating to
  `run_node`. **SC2** `sensor_msgs` subscriptions use `qos_profile_sensor_data`,
  never a bare depth. **SC3** no raw `lookup_transform` outside `node_util`.
  **SC4** `Twist` publishers only in `cmd_vel_source`/`estop`. **SC5** no
  hand-rolled planar-quaternion math (use `core.geometry`, or the vendored
  `docker/scout-skills/geometry.py`). **SC6** the `/ros_ws/src/scout` bind path
  only in `robot_profile.py`. **SC7** every `core/` module is imported by a node
  AND has a 1:1 test file. **SC8** profile-owned values are never bare literals
  (add new ones to `test_profile_constants.py`; escape a legitimate coincidence
  with a `profile-exempt: <reason>` comment). **SC9** the `|`-status wire formats
  are frozen in `core.status` + `test_status.py`. **SC10** the two deliberate
  copies (`webui/robot_profile.yaml`, skills `geometry.py`) stay in sync.
- **New convention?** Write it as a failing test first, remediate, land both,
  record the why in an ADR (the ADR-0013 loop).
- `scout.core` may never import ROS or yaml — profile values are *injected* by
  the ROS caller (e.g. `plan_coverage(occupied=...)`), pure defaults carry a
  `profile-exempt` marker.

**Where designs live:** this file (execution detail) + `docs/adr/` (the *why*) +
`CONTEXT.md` (glossary + node/topic map). The original approved plan is at
`~/.claude/plans/sequential-crunching-falcon.md` on the dev machine.

---

## 1. PREREQUISITE — deploy + verify M3 (twist_mux) on blocks

M3 is code-complete but **unverified on hardware**. It changed the drivetrain
topology, so it must pass an on-blocks pass before any floor driving and before
M4 (which restarts nav2 on top of it).

**Deploy:** image rebuild (adds `ros-humble-twist-mux`) + `build_package` +
`scout_skills` rebuild (profile mount + code). Then `up -d robot scout_skills`.

**On-blocks checklist (operator, wheels off ground):**
1. `ros2 topic info /cmd_vel_out -v` → exactly 2 endpoints: twist_mux pub +
   roboclaw_driver sub.
2. Joystick drive → `ros2 topic hz /cmd_vel_joy` and `/cmd_vel_out` ≈25 Hz;
   release → 0.3 s of zeros pass through, then `/cmd_vel_out` silent.
3. Webui pad drives; hold joystick simultaneously → joystick preempts (visible
   on `/cmd_vel_out`).
4. `/goal_pose` from Foxglove → nav flows to `/cmd_vel_out` ~30 Hz; blip the
   joystick mid-nav → nav masked, resumes ≤~0.8 s after release.
5. `ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"` mid-drive →
   `/cmd_vel_out` stops within a frame, wheels stop; `false` → drivable again.
   Then test the webui E-STOP button + skills `estop(true/false)` tool.
6. Trick + follow_me each on blocks. Then a short floor pass (teleop, one nav
   goal, one trick).

**If a source can't drive:** check its publisher topic matches
`robot_profile.yaml` `topic_cmd_vel_*` and the twist_mux `topics:` entry.

---

## 2. M4 — config overlays + profile knob (Pi param-dump gated)

**Goal:** replace the full-file `*_tight_tunnel.yaml` forks (~1,200 lines, ~23
real deltas + drifted rationale) with base + delta overlays and a single
`profile` switch. Default profile must be **byte-for-byte equivalent** (proven
by param dump). See ADR-0010.

### Mechanism per file pair
- **nav2**: RewrittenYaml can't rewrite the costmap `plugins` list (that's how
  `depth_layer` is removed), so use a ~25-line **PyYAML deep-merge** in a new
  `scout/launch/nav2.launch.py`: merge base + overlay → temp file → hand to
  upstream `nav2_bringup/navigation_launch.py` via `params_file:=`. Leftover
  `depth_layer.*` params are inert (Costmap2DROS only instantiates layers named
  in `plugins`).
- **slam**: native stacked params — `Node(parameters=[base, overlay, mode_params])`
  in the existing `slam.launch.py:143` list. 3 scalar deltas.
- **realsense**: same deep-merge helper (flat rs_launch dialect) → temp file →
  `rs_launch.py config_file:=`. Deleted keys become `null` (delete sentinel) or
  `''` (`json_file_path` → wrapper default).
- **explore**: `explore_tight_tunnel.yaml` has no base — rename it to
  `explore.yaml` (it is the only/default explore config; the suffix is a
  misnomer).

### Files
**Extend (do NOT create `launch_utils.py`):** config resolution already lives in
`scout/scout/robot_profile.py` — `resolve_config(name)` / `resolve_config_dir()`
own the `/ros_ws/src/scout` bind path, and **SC6 forbids that path anywhere
else** (the 3-dialect dedup the original plan proposed was done by the ADR-0013
pass). Add the profile-overlay helpers to `robot_profile.py`:
```python
def known_profiles() -> list              # ['default'] + subdirs of config/overlays/
def profile_overlay(profile, basename)    # RAISES on unknown profile listing known;
                                          # returns None if this file has no overlay
def deep_merge(base, overlay) -> dict      # dicts recurse; lists/scalars replace; None deletes
def merged_params(basename, profile) -> str
    # 'default' -> resolve_config(basename) UNCHANGED (byte-identical path, no temp file)
    # else -> deep_merge, write /tmp/scout_profile/<basename>, log path, return it
```
`deep_merge` is pure — if you'd rather keep it testable in `scout.core`, put it
in a new `core/` module (then SC7 needs a 1:1 test + a node importer) and have
`robot_profile.merged_params` call it. `merged_params` stays in `robot_profile`
(it reads files → not core-pure).
- `scout/launch/nav2.launch.py` — resolves+merges nav2 params, then
  `IncludeLaunchDescription(navigation_launch.py, launch_arguments={'params_file':
  merged, 'use_sim_time':'false'})`. Add the **coupling guard** in an
  `OpaqueFunction`: after merging, if any costmap `plugins` still lists
  `depth_layer`, compute the realsense effective config for the SAME profile and
  `raise RuntimeError` if its `pointcloud.enable` is false (ADR-0002 coupling).
- `scout/config/overlays/tight_tunnel/{nav2,slam,realsense}.yaml` — deltas +
  delta rationale ONLY (~30 lines total vs ~1,200 forked). Sketch for nav2:
  ```yaml
  bt_navigator: {ros__parameters: {default_nav_to_pose_bt_xml: "/ros_ws/src/scout/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"}}
  controller_server:
    ros__parameters:
      general_goal_checker: {xy_goal_tolerance: 0.08}
      FollowPath: {max_vel_x: 0.35, max_speed_xy: 0.35, min_vel_x: -0.15,
                   rotate_to_heading_angular_vel: 2.5, max_vel_theta: 2.5,
                   xy_goal_tolerance: 0.08, ObstacleFootprint.scale: 0.01}
  local_costmap: {local_costmap: {ros__parameters:
      {plugins: ["obstacle_layer", "inflation_layer"],   # <-- removes depth_layer
       inflation_layer: {inflation_radius: 0.17, cost_scaling_factor: 15.0}}}}
  global_costmap: {global_costmap: {ros__parameters:
      {resolution: 0.025,
       plugins: ["static_layer", "obstacle_layer", "inflation_layer"],
       inflation_layer: {inflation_radius: 0.17, cost_scaling_factor: 15.0}}}}
  velocity_smoother: {ros__parameters: {max_velocity: [0.35,0.0,2.5], min_velocity: [-0.35,0.0,-2.5]}}
  ```
  slam overlay: `slam_toolbox: {ros__parameters: {resolution: 0.025, minimum_travel_distance: 0.2, minimum_travel_heading: 0.25}}`.
  realsense overlay: `enable_depth: false`, `pointcloud.enable: false`,
  `align_depth.enable: false`, `json_file_path: ''`,
  `depth_module.enable_auto_exposure: ~`.
- `scout/test/test_profile_overlays.py` — pytest (Mac): assert
  `deep_merge(base, overlay)` == the parsed OLD fork **modulo an explicit
  expected-diff list**, AND every overlay key exists in its base (typo guard —
  a misspelled overlay key silently creates a new param otherwise). Run BEFORE
  deleting the fork files (step below), then the equivalence half retires and
  the typo-guard half stays.
- `.env.example` (`SCOUT_PROFILE=default`); add `.env` to `.gitignore`.

**Modified:** all 4 scout launch files (add `DeclareLaunchArgument('profile',
default_value='default')`; unknown → raise listing `known_profiles()`),
`docker-compose.yaml` (nav2 service → `ros2 launch scout nav2.launch.py
profile:=${SCOUT_PROFILE:-default}`; same `profile:=` on robot/slam/explore;
delete the commented tight_tunnel alt command lines), `scout/setup.py` (add the
overlays dir to `data_files` — `glob('config/*.yaml')` does NOT recurse:
`('share/scout/config/overlays/tight_tunnel', glob('config/overlays/tight_tunnel/*.yaml'))`),
`scout/config/nav2.yaml` (comment touch-up: the `:223`-ish cross-reference to
the fork's 0.35 value → point at the overlay). Drop the `camera_config` launch
arg on `robot.launch.py` in favor of `profile` (operator decision — recommend drop).

**Deleted:** `nav2_tight_tunnel.yaml`, `slam_tight_tunnel.yaml`,
`realsense_tight_tunnel.yaml`.

### Ordering + verification
1. Baseline (Pi, default stack up, before any change): `ros2 param dump` for
   `/controller_server /planner_server /smoother_server /behavior_server
   /bt_navigator /waypoint_follower /velocity_smoother
   /local_costmap/local_costmap /global_costmap/global_costmap /slam_toolbox
   /camera/camera` → `captures/params_baseline/`. **Ground truth.**
2. `robot_profile.py` overlay helpers + overlay files + `test_profile_overlays.py`
   (forks still in tree). `ruff check .` + `pytest` green — proves merge == fork
   modulo expected diffs + typo guard.
3. Rewire launch files + `nav2.launch.py` + coupling guard + setup.py. In the
   container: `ros2 launch scout <f> --print` for all 4 files × both profiles;
   confirm default-profile paths point at the ORIGINAL base files (no temp);
   `profile:=bogus` and a missing config both raise.
4. Compose knob + `.env.example`. `docker compose config` renders both; editing
   `.env` + `up -d` recreates robot/slam/nav2 (command changed).
5. Delete the 3 forks; rename `explore.yaml`; nav2.yaml comment touch-up.
   `grep -rn tight_tunnel.yaml` → only overlay paths.
6. **End-to-end (Pi):** default profile up → re-dump all nodes → **diff vs
   baseline MUST be empty** (allow only nav2_bringup's own autostart/use_sim_time
   rewrites). Then `SCOUT_PROFILE=tight_tunnel up -d`, spot-check sentinels
   (`FollowPath.max_vel_x 0.35`, local `plugins` without depth_layer, `slam
   resolution 0.025`, camera `enable_depth false`, `bt_navigator
   default_nav_to_pose_bt_xml` set) + coupling guard passes; switch back, re-dump,
   diff empty again (round-trip). Nav2 restarts — floor clear / on-blocks rules.

**Operator decisions:** (a) tight profile inherits post-fork base fixes like
`observation_persistence 1.0/2.0` — recommend yes (that inheritance is the point
of overlays); (b) drop `camera_config` arg — recommend yes.

---

## 3. M5 — deploy hardening (Pi rebuild gated)

Independent of M4; can go before or after. See ADR-0005.

### Items
1. **Overlay-volume stamp (the top-severity trap).** Last Dockerfile RUN that
   writes `$OVERLAY/install` (after the explore_lite layer):
   ```dockerfile
   RUN date +%s.%N > "$OVERLAY/.image_build_id" \
    && cp "$OVERLAY/.image_build_id" "$OVERLAY/install/.image_build_id"
   ```
   Layer caching gives exactly the right invalidation (reruns iff a fork/build
   layer above changed). Entrypoint check (hard-fail — operator confirmed):
   ```bash
   img="$OVERLAY/.image_build_id"; vol="$OVERLAY/install/.image_build_id"
   if [ -f "$img" ] && { [ ! -f "$vol" ] || ! cmp -s "$img" "$vol"; }; then
     echo "FATAL: ros_overlay_install volume is stale (older image)." >&2
     echo "  docker compose down -v && docker compose --profile build run --rm build_package && docker compose up -d" >&2
     exit 1
   fi
   ```
   Absent volume stamp counts as stale (forces the one-time `down -v` migration).
   Services that don't mount the volume see the image's own copy (always matches).
2. **Pin the 3 unpinned source forks to commit SHAs** (`git clone <url> dst &&
   git -C dst checkout --detach <sha>` — `--depth 1` can't fetch a bare SHA):
   `roboclaw_driver` (default branch), `rplidar_ros` (ros2 branch tip),
   `m-explore-ros2` (default). **Re-fetch current tips and record the SHAs at
   pin time** — the deployed image's actual commits are unrecoverable (clones are
   `rm -rf`'d), so pin whatever is current and accept the first rebuild may bump.
   While rebuilding anyway, **move the roboclaw layer BELOW librealsense/
   realsense-ros** so future roboclaw pin bumps stop costing the 13-min
   librealsense rebuild (makes the layer-order rule structural, not a comment).
3. **Compose hygiene:** move `build: .` off the `&base` anchor onto
   `build_package` only (runtime services become pure `image: scout:latest`
   consumers; with BuildKit the 8 parallel builds are cache hits, but it's still
   redundant + last-tag-wins noise). Update `scout-switch.sh:29` to
   `docker compose --profile build build`. Trim `webui` (drop the anchor:
   `image: scout:latest`, `network_mode: host`, `entrypoint: []`, one mount
   `./webui:/webui:ro`, no privileged) and `foxglove_bridge` (host net + `ipc:
   host` + `ROS_LOCALHOST_ONLY=1`, no privileged, no volumes). `rosbridge`
   qualifies for the same trim — flagged, optional.
4. **setup.py / package.xml TODOs:** version `0.1.0`, real description,
   maintainer `cdrew@brasfieldgorrie.com`, a license string (operator decision —
   suggest `Proprietary`). Keep `data_files` (share is the documented fallback).

### Verification
Rebuild on the Pi (~13 min once). `up -d` **hard-fails** with the stamp message
(existing volume, no stamp) → `down -v` → `--profile build run build_package` →
`up -d` clean. Force a trivial layer change + `--no-cache` rebuild → confirm the
mismatch fires again. `docker inspect` shows `Privileged: false` + expected
mounts for webui/foxglove; webui serves `http://scout.local`; Foxglove connects
and lists topics (proves DDS-over-loopback survives the trim).

**Operator decision:** confirm nothing external still targets `ros_mcp` on :9000
(operator kept it in this pass; do NOT delete without confirming Magnus).

---

## 4. M7 — waypoint unification (Mac-testable core; Pi migration gated)

Two systems named "patrol" write `./maps` and can't see each other. Unify on one
store. See ADR-0011. **The `scout.core.waypoints` module + tests are
Mac-verifiable — do them first and land them like the M6 modules.**

### Target schema — `maps/waypoints.json` v2
```json
{"version": 2,
 "waypoints": {"kitchen": {"x":1.2,"y":3.4,"yaw":0.5,"saved":"...","source":"operator"}},
 "routes": {"patrol": ["mark-1", "mark-2", {"x":6.0,"y":1.0,"yaw":3.14}]}}
```
`source ∈ operator|tag|mark|coverage` makes the AprilTag auto-refresh visible.
Route items are waypoint *names* or inline pose dicts (coverage's ≤120 generated
points stay inline, out of the name namespace). `maps/patrol_route.yaml` retires.

### New `scout/scout/core/waypoints.py` (pure, json only) + `test_waypoints.py`
```python
def load(path) -> dict            # Store; auto-wrap a legacy flat {name:pose} file
def save(path, store)             # atomic tmp + os.replace
def migrate(legacy_or_store) -> dict   # flat waypoints.json OR patrol_route.yaml data -> v2
def resolve_route(store, name) -> list # names -> poses; KeyError lists missing
def set_waypoint(store, name, pose, source)   # pure; caller reloads/saves around it
```
Tests: v2 round-trip; legacy flat file auto-wraps; patrol_route.yaml `{'waypoints':[...]}`
→ route "patrol" of inline poses; resolve_route raises listing missing names;
atomic-save leaves no partial file.

### Adopt (Pi-verifiable)
- **patrol_capture**: `route_file` param → `waypoints_file` + `route_name`
  (default `patrol`). `/patrol/mark` → reload store, add waypoint `mark-<n>`
  (`source: mark`), append its NAME to the route, save. Names resolve at
  `/patrol/start` (so a tag-refreshed waypoint used in the route is picked up
  automatically — that IS the auto-refresh answer: shared file + resolve-on-start,
  no event channel). `/patrol/clear` clears the route + its `mark-*` only (flag:
  semantic change from nuking everything). Coverage box → writes the route as
  inline poses. Photos/manifest unchanged. Re-read the store before each mutation
  (don't hold `self._route` across a session); atomic save.
- **scout-skills `server.py`**: `_load_waypoints`/`_store_waypoints` grow ~15
  lines — v2 wrap/unwrap + legacy tolerance (the container has no scout package,
  so **share the schema, not the code**; contract pinned by ADR-0011 + test
  fixtures). Tag refresh (`server.py:763-771`) sets `"source":"tag"`. New: the
  `patrol(names, loops)` tool accepts a stored route name (expand it if
  `len(names)==1 and names[0] in store["routes"]`) — ~6 lines, recommended.
- **Route topics: keep BOTH, do not merge** (verified different roles):
  `/patrol_route` (Path) = webui display; `/route_poses` (PoseArray) =
  link_watchdog re-dispatch mirror.
- **Migration:** `scripts/migrate_waypoints.py` (pure Python, Pi host or
  container): `patrol_route.yaml` + legacy `waypoints.json` → v2 `waypoints.json`,
  originals → `*.bak`. Loaders auto-wrap a legacy flat file in place, so pull-vs-
  run order can't brick — but `patrol_route.yaml` NEEDS the script (patrol_capture
  stops reading it). Runbook: `git pull && python3 scripts/migrate_waypoints.py
  maps/ && docker compose restart robot scout_skills`.

### Optional Phase 9 (opt-in, own commit) — fix the link-loss/patrol gap
link_watchdog publishes latched `std_msgs/Bool /nav_paused`; patrol_capture
holds in `driving` while paused and doesn't advance. ~25 lines. New behavior —
keep separate. (ADR-0009 records the gap.)

### Verification
Mac: `test_waypoints` green. Pi: run the migration script; `/patrol/mark` →
`/patrol/start` → photo on the v2 store; skills `save_waypoint` + `patrol(<route
name>)`; confirm a tag sighting updates a named waypoint the route uses and the
next run drives to the fresh pose.

---

## 5. M6 deferrals

The ADR-0013 convention pass finished most of these while enforcing SC1/SC7/SC9.
**Only 5b remains.**

### 5a. battery_monitor → `scout.core.battery` — ✅ DONE (ADR-0013 pass, SC7)
`battery_monitor.py` imports `RestingSocEstimator` + the curve from
`scout.core.battery`. Still Pi-verify after committing: `/battery` percentage at
a known resting voltage + the low/critical warnings.

### 5b. depth_grid + scan (the ~180-line under-lidar dedup — highest risk, OUTSTANDING)
The under-lidar obstacle algorithm is written twice: `follow_me._on_depth` +
`_memory_corridor_min` (odom-anchored) and `clutter_mapper._on_depth`
(map-anchored). Extract to `scout/scout/core/depth_grid.py`:
```python
@dataclass(frozen=True)
class GridConfig:   # resolution, mark_increment, cap, confirm_at, confirm_window_s,
                    # unconfirmed_ttl_s, confirmed_ttl_s (None=persist, clutter mode)
class ObstacleCellGrid:
    # cells: {(ix,iy): [score, first_seen, last_seen]} in a caller-chosen anchor
    # frame; times are floats the caller supplies (time.monotonic()) — no clocks.
    def mark(self, ax, ay, now); def expire(self, now); def confirmed(self)
    def clear_by_live_points(self, pose, live_base_xy, *, half_fov, min_range, max_range, clear_radius)  # follow_me policy
    def clear_by_ray_bins(self, pose, free_to, *, half_fov, min_range, max_range, nbins, decrement)      # clutter policy
    def min_corridor_distance(self, pose, *, lookahead, half_width, exclude_xy=None, exclude_r=0.0)
    def to_arrays(self); @classmethod from_arrays(cls, cfg, arr, now)   # clutter reload contract
def band_mask(x,y,z,*,band_lo,band_hi,x_max,half_width,exclude_xy=None,exclude_r=0.0)
def free_space_profile(r, bearing, *, half_fov, nbins)   # clutter's np.maximum.at
```
**Keep the two clearing policies as separate methods** — they are genuinely
different algorithms (live-point deletion vs polar ray-bin decrement); forcing
one shape changes behavior. Unify voxel keying on `floor(w/res)` (follow_me
currently rounds — a half-cell shift below its 0.05 m noise floor; call it out).
Also `scout/scout/core/scan.py`: follow_me's cluster extraction + corridor
metrics + the **180°-backwards-mount** encoding (`scan_yaw_offset=pi`).

Tests (Mac): mover never confirms; dweller confirms after the window; TTL
policies (unconfirmed vs confirmed vs persist); ray-bin clearing decrements the
seen-through cell; live-point clearing spares blind-zone voxels; corridor min
with target exclusion; `to_arrays`/`from_arrays` round-trip pre-satisfies dwell;
**scan yaw_offset=pi maps scan angle π to bearing 0** (the backwards-mount
regression). Then adopt **clutter_mapper first** (commit), then **follow_me**
(commit). Optional: a `rosbags`-fed Mac diff of old-vs-new corridor mins on a
recorded depth bag before merging — the best behavior-preservation check.
**Bench-verify** each: clutter mark/see-through-clear; follow acquire/avoid.

### 5c. run_node in every node — ✅ DONE (ADR-0013 pass, SC1)
All 13 console-script mains delegate to `run_node`; SC1 in `test_conventions.py`
keeps it that way.

### 5d. status wire formats — ✅ DONE (ADR-0013 pass, SC9)
`scout/scout/core/status.py` owns the `|`-format grammar; `test_status.py` (SC9)
freezes the exact strings. Still kept as strings, not `.msg` (ADR-0012/0013).

**When you build 5b:** it must clear the conventions gate — a `core/depth_grid.py`
and `core/scan.py` each need a 1:1 test file AND a node importer (SC7), any
profile-owned values are injected by the caller not hardcoded (SC8, ADR-0012
purity), and `ruff check .` must pass. Add the regression tests as SC-style
where they encode a rule (e.g. the backwards-mount `scan_yaw_offset=pi`).

---

## 6. Light follow-up (any time, low risk)
Comment-dedup pass: where rationale now exists twice (e.g. battery thresholds in
led_status/trick_player/patrol, the slam mode essay, apriltag revert note, the
clutter/nav2 layer interaction), replace the duplicate with `# see ADR-NNNN`.
**Keep every one-off measured number and tuning note** (nav2.yaml's ~400 lines
are mostly one-off operational tuning — those stay). Delete only what an ADR now
owns.

---

## 7. Suggested order for the next session
0. **Commit the uncommitted ADR-0013 convention pass** (see §0) — run `ruff
   check .` + pytest first; that's the definition of done for it.
1. Deploy + on-blocks verify **M3** (prerequisite for any floor work).
2. **M7 core** (`core/waypoints.py` + tests) — Mac-verifiable, lands clean.
3. **M4** (needs Pi param-dump; highest functional value after M3).
4. **M5** (one rebuild + volume migration).
5. **M7 adoption + migration**, then the one remaining M6 deferral: **5b
   depth_grid/scan** (Mac tests, then adopt clutter_mapper → follow_me, with
   bench checks). 5a/5c/5d are already done.
6. Comment-dedup pass (note: SC8 already test-enforces "no re-hardcoded profile
   values", so that slice is done; this is the prose-rationale cleanup).

Every milestone is an independent commit; deploy + verify per-milestone. Update
the relevant ADR if a decision changes during execution.
