# Scout — remaining implementation plan (handoff)

Self-contained plan for the work left after the 2026-08-15 architecture pass.
Written so a fresh LLM + the operator can execute without the original session.
Read this top to bottom once, then work a milestone at a time.

---

## 0. Current state — read first

**Branch:** `web-ui`, **nothing pushed.** All the software milestones are landed
and pass the gate (`ruff check .` + `PYTHONPATH=scout pytest scout/test` — 57
pass, 1 skip). M1–M3, M6-core, M8 (earlier), the ADR-0013 convention pass, then
this session: **M7, M4, M5**. What's left is **verification you can only do on
the Pi**, plus **one deliberately-unstarted code task (M6-5b)** — see the
checklist below.

### ✅ Done (code complete, gate-green)
- **M1** safety + bug sweep · **M2** robot_profile SSOT · **M3** twist_mux + e-stop
- **M6 core** `scout.core` (geometry/battery/coverage/colors/tricks/status) +
  node adoption + tests · **M8** ADRs + CONTEXT.md
- **ADR-0013** ruff + SC1–SC10 structural tests + CI (also finished M6-5a
  battery adoption, 5c run_node-everywhere, 5d status formatters)
- **M7** one waypoint/route store (`core/waypoints.py`, patrol_capture +
  scout-skills v2, `scripts/migrate_waypoints.py`) — ADR-0011
- **M4** config overlays + `SCOUT_PROFILE` knob (`overlays/tight_tunnel/*`,
  `nav2.launch.py`, deep-merge in robot_profile; forks deleted, 0-diff verified)
  — ADR-0010
- **M5** overlay-volume stamp guard + `privileged`→robot-only + `build`→
  build_package-only — ADR-0005

### ☐ OPEN TASKS

**A. On-Pi / on-bench verification (no code; operator gated). See the noted §.**
1. **M3 on-blocks gate** (twist_mux) — MUST pass before any floor driving. §1.
2. **M4 param-dump equivalence** — `ros2 param dump` default profile == baseline
   (empty diff); tight_tunnel sentinels + coupling guard. §2.6.
3. **M5 stamp guard** — rebuild, confirm it hard-fails on a stale volume then
   `down -v` recovers. §3.
4. **M7 migration + drive** — run `scripts/migrate_waypoints.py maps/`, then
   mark→start→photo and skills `patrol(<route>)`. §4.

**B. M5 leftovers — ✅ CODE COMPLETE (ADR-0005 addendum); Pi rebuild verify open. §3.**
5. ✅ Forks pinned: roboclaw `cc4d0e7`, rplidar ros2 `24cc9b6`, m-explore
   `326cf8a` (tips at pin time).
6. ✅ roboclaw layer moved below librealsense + apt layers (pin bumps no
   longer cost the 13-min rebuild).
7. ✅ webui/foxglove trimmed; `webui/robot_profile.yaml` converted symlink →
   SC10-synced real copy. Pi verify: rebuild (~13 min), stamp-guard `down -v`
   migration, webui serves, Foxglove connects (§3 verification).

**C. The one remaining code task — DELIBERATELY NOT STARTED. §5b.**
8. **M6-5b depth_grid + scan** extraction (the ~180-line under-lidar dedup).
   NOT attempted offline: it is a safety-critical perception refactor (decides
   whether the robot sees under-lidar obstacles), the two implementations
   diverge in load-bearing ways (cell-key type, mark increment, confirm metric,
   clear policy, expiry, follow-me's trail rejection), and **SC7 forbids landing
   the core module unadopted** — so it is all-or-nothing and cannot be verified
   without the robot or a recorded depth bag (the plan requires a rosbags
   old-vs-new corridor-min diff before merge). Do it at the bench, per §5b.

**D. New ROS 2 feature additions (2026-08-15). §8.** Four features grounded in
ROS 2 Humble capabilities the stack lacks.
9. **F1 unified diagnostics — CODE COMPLETE, gate-green (64 pass / 1 skip).**
   Only Pi/browser verification + the webui/Foxglove surface remain. §8.1.
10. **F2 rosbag2 record-on-demand** — NOT started (robot-coupled: the value is
    the `ros2 bag record` subprocess lifecycle). §8.2.
11. **F3 nav2 cancel + `/nav_state` feedback** — NOT started (robot-coupled:
    live action cancel/feedback semantics). §8.3.
12. **F4 managed lifecycle bring-up** — NOT started (robot-coupled: transition/
    ordering behavior; largest effort, lowest ROI — spike last). §8.4.

**E. Humble-capability adoptions (2026-08-15 audit) — CODE COMPLETE, Pi verify
open. §8.5.**
13. **nav2 composed bring-up** — verify on Pi: goal succeeds, CPU + control-loop
    misses vs `use_composition:=false` baseline. §8.5.
14. **Fast DDS Discovery Server** — verify on Pi: full stack rediscovers after
    `up -d`, Foxglove connects, super-client shell sees full graph. §8.5.

**F. Nav/autonomy additions (2026-08-15 corpus+nav2 audit) — NONE started. §9.**
15. **N1 nav2_collision_monitor** — ✅ CODE COMPLETE (ADR-0016); on-blocks +
    floor verify open. NOTE: changes M3's §1 checklist — driver now listens on
    `/cmd_vel_safe`. §9.1.
16. **N4 fail-fast bring-up** — ✅ CODE COMPLETE (ADR-0015); Pi kill-test
    verify open. §9.2.
17. **N5 SC11 rclpy no-sync-service-call rule** — ✅ DONE (test_conventions
    SC11 + ADR-0013 row; zero remediation needed). §9.3.
18. **N9 `ros2 doctor --report` in deploy runbook** — zero code. §9.4.
19. **N6 Fast DDS async publish + UDP-frag sysctls** — measure-first. §9.5.
20. **N7 keepout/speed costmap filters** — ask operator first. §9.6.
21. **N8 MPPI controller A/B overlay** — robot-coupled, after N1. §9.7.
22. **N10 EKF process-noise tuning** — robot-coupled measurement. §9.8.
    (N2/N3 are amendments folded into F2/F1 above, not separate items.)

**Not doing** (unless asked): the optional `/nav_paused` link-loss/patrol fix
(§4, new behavior); the prose comment-dedup pass (§6, low value — SC8 already
test-enforces the profile-value slice); the §9 Tier-B rejects (recorded there
so the audit isn't redone).

---

_Sections below are the execution detail. §2/§3/§4 are now DONE — kept as the
record of HOW, and as the spec for the on-Pi verification of each._

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
_(updated for ADR-0016: the collision monitor now sits between the mux and
the driver)_
1. `ros2 topic info /cmd_vel_out -v` → exactly 2 endpoints: twist_mux pub +
   collision_monitor sub. `ros2 topic info /cmd_vel_safe -v` → exactly 2:
   collision_monitor pub + roboclaw_driver sub.
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

## 2. M4 — config overlays + profile knob ✅ IMPLEMENTED (commit 905d4cd) — Pi param-dump verification still open

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

## 3. M5 — deploy hardening ✅ STAMP+COMPOSE IMPLEMENTED (commit 89c9603) — fork pins / layer reorder / mount trim + Pi rebuild verify still open

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

## 4. M7 — waypoint unification ✅ IMPLEMENTED (commit aac39fc) — Pi migration + drive verification still open

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

---

## 8. New feature additions (2026-08-15) — ROS 2 capabilities the stack lacks

Grounded in the ROS 2 Humble docs. All are pure-Python scout nodes → built by
`build_package` (colcon overlay), **no base-image rebuild** (diagnostic_msgs /
rosbag2 / nav2_msgs / rclpy.lifecycle already in `scout:latest` via nav2 +
robot_localization — confirm with `ros2 interface show` / `ros2 pkg list`).
Split by the operator's rule: what develops+validates offline is landed now
(F1); what needs the robot in the loop to develop is specced here.

Every one follows the house pattern: node in `scout/scout/`, `main()` =
`run_node(...)`, cross-surface constants → `robot_profile.yaml` (per-node
tunables stay `declare_parameter`), pure logic → `scout/scout/core/<x>.py` with a
1:1 `test_<x>.py` (SC7 requires a node importer too — core + node land together),
register in `setup.py` + `robot.launch.py`, one ADR per decision, gate = ruff +
pytest. Verify liveness from node logs + data topics, never throwaway-container
discovery. Never command motion without per-run operator confirmation.

### 8.1 F1 — unified diagnostics ✅ CODE COMPLETE (gate-green) — Pi/webui verify open

`health_monitor` aggregates `/battery` + `/tilt_alarm` + `/roboclaw_status`
liveness onto `/diagnostics` (DiagnosticArray) at 1 Hz with an overall roll-up;
levels in pure `scout.core.health`. ADR-0014. Landed: `scout/scout/health_monitor.py`,
`scout/scout/core/health.py`, `scout/test/test_health.py`, `setup.py`,
`robot.launch.py`, `docs/adr/0014-*.md`.
**Open (needs robot / browser):**
- Deploy (`build_package` + `up -d robot`); `ros2 topic echo /diagnostics` shows
  battery/tilt/drivetrain + roll-up; force a WARN (raise `battery_warn_v`
  temporarily) and confirm colour; kill `battery_monitor` → battery item goes
  STALE; stop `robot` → drivetrain STALE.
- Confirm `diagnostic_msgs` is in the image (`ros2 interface show
  diagnostic_msgs/msg/DiagnosticArray`); if absent, add `ros-humble-diagnostic-msgs`
  *after* the librealsense RUN (image rebuild).
- **webui health strip** (`webui/index.html`/`app.js`/`style.css`): subscribe
  `/diagnostics` over rosbridge (roslibjs), colour by worst level. Browser-only
  verification. **Foxglove:** add a Diagnostics panel to `Foxglove.json`.
- **Later (needs on-hardware JSON):** add drivetrain temps + RoboClaw error flags
  once the `/roboclaw_status` keys past `main_battery`/`m1_speed`/`m2_speed` are
  read on the robot — do not guess field names (ADR-0014).
- **Optional follow-up — QoS deadline events instead of hz-polling:** a
  subscription created with a `deadline` QoS gets a missed-deadline *callback*
  (event-driven, catches single dropouts a 1 Hz poll misses). Compatibility is
  request/offered — a subscriber deadline vs a publisher with no offered
  deadline = **no connection** — so it only works on topics whose publishers we
  own (cmd_vel_source's `/cmd_vel_*`, gyro_calibrator's `/imu/data`): offer the
  deadline on the publisher, request it on a dedicated monitor subscription in
  health_monitor. Do NOT put a deadline on the main consumer subscriptions.
  Same pass should add (N3, 2026-08-15 audit): **liveliness events**
  (MANUAL_BY_TOPIC on publishers we own + `liveliness_changed` callback —
  catches a wedged process a 1 Hz hz-poll reads as merely slow; same
  request/offered compat trap as deadline) and **`incompatible_qos` event
  callbacks** on health_monitor's own subscriptions → DiagnosticArray WARN for
  the documented silent best-effort/reliable mismatch class. rclpy Humble
  supports both via `SubscriptionEventCallbacks`/`PublisherEventCallbacks`.

### 8.2 F2 — rosbag2 record-on-demand (robot-coupled — NOT started)

**Why:** all bench/calibration tooling was deleted; rosbag2 is the ROS-native
permanent replacement for "rebuild the instrument". Robot-coupled: the feature IS
the `ros2 bag record` subprocess lifecycle — spawn/track/clean-SIGINT can only be
developed + validated with ros2 running.
- New `scout/scout/bag_recorder.py`: `record/start` + `record/stop`
  (`std_srvs/Trigger`), publishes `record/active` (Bool) + last bag path (String),
  both latched (`scout.qos.LATCHED_QOS`). On start, spawn `ros2 bag record` as a
  subprocess (robust vs the rosbag2_py in-process threading caveats) into
  `captures/<UTC-timestamp>/`; clean SIGINT on stop; refuse double-start.
- **(N2, 2026-08-15 audit) Add snapshot mode as the third service:** spawn with
  `--snapshot-mode` (buffers in RAM, writes nothing until triggered);
  `record/snapshot` (Trigger) calls the recorder's `~/snapshot` service to
  flush the buffer to disk — "capture the last N seconds before the incident"
  instead of continuous SD-card writes. Two recorder modes, one node:
  continuous (start/stop) and armed-snapshot.
- **⚠ Never pass `--max-bag-size`/`--max-bag-duration` in `record_argv`:**
  Humble known issue — bags split by size or duration do not play back
  correctly (only the last split plays; ros2/rosbag2#966).
- New pure `scout/scout/core/recording.py` (offline-testable, lands with the
  node for SC7): `bag_dir(now, root)`, `record_argv(topics, out_dir)`,
  `resolve_topics(profile)` → argv list; 1:1 `test_recording.py`.
- `robot_profile.yaml`: `record_topics` (flow list) — default `/odom /wheel_odom
  /imu/data /scan /cmd_vel_out /roboclaw_status /battery /diagnostics /tf
  /tf_static`. `robot.launch.py` + `setup.py` register the node.
- scout-skills MCP (`docker/scout-skills/server.py`): `start_recording` /
  `stop_recording` / `recording_status` tools calling the Trigger services via
  `rosbridge.py`. webui: a REC toggle lit from `record/active`.
- **Confirm `captures/` is bind-mounted to the host** in `docker-compose.yaml` so
  bags reach the operator (add the mount if missing). New ADR at build time
  (0015 was taken by fail-fast bring-up).
- **⚠ QoS overrides are mandatory or the bag silently misses `/imu/data`:**
  `gyro_calibrator` publishes best-effort (same trap as the EKF QoS note), and
  a reliable-by-default recorder subscription receives nothing. Ship
  `scout/config/bag_qos_overrides.yaml` (best-effort reliability for
  `/imu/data` + any sensor topics) and pass
  `--qos-profile-overrides-path` in `record_argv` (Humble how-to:
  Overriding-QoS-Policies-For-Recording-And-Playback).
- Verify (robot): `record/start` → short **operator-confirmed** drive →
  `record/stop`; `ros2 bag info captures/<ts>` lists topics + counts —
  **including a nonzero `/imu/data` count** — bag opens on the host.

### 8.3 F3 — nav2 cancel + `/nav_state` feedback (robot-coupled — NOT started)

**Why:** documented sharp edges — no operator nav-cancel (only the `nav_cancel`
skill / compose restart), "goal failed ≠ robot stops", no visible nav progress.
Robot-coupled: it is pure live-action (cancel + feedback) behavior.
- New `scout/scout/nav_manager.py`:
  - `/nav/cancel` (`std_srvs/Trigger`) → cancel BOTH `navigate_to_pose` and
    `navigate_through_poses` via `CancelGoal` clients (reuse the exact pattern in
    `link_watchdog.py:_pause`, zeroed-uuid = cancel all). Extract that into a
    shared `node_util.cancel_nav_goals(clients)` and have link_watchdog adopt it
    (avoids a third copy). **⚠ Async-only (SC11, §9.3): the cancel fires from
    inside a Trigger service callback — a sync `Client.call()` there deadlocks
    silently (no warning, no exception). `call_async` + done-callback only.**
  - Subscribe both actions' `feedback` + `_action/status`; republish a
    consolidated `/nav_state` (String JSON: status name from
    `robot_profile.goal_status_names`, `distance_remaining`,
    `number_of_recoveries`), latched.
- webui: two distinct controls — **CANCEL NAV** → `/nav/cancel` (stops the goal,
  robot stays drivable) vs the existing **STOP** → `/estop/engage` (hard, locks
  twist_mux + active-brake). Show `/nav_state` near the map. `robot.launch.py` +
  `setup.py`. New ADR at build time (0016 was taken by collision monitor).
- Verify (robot): dispatch a goal, CANCEL NAV → motion stops, still drivable,
  `/nav_state` = `canceled`; STOP still hard-estops. **Watch from host
  `docker compose logs`, never a throwaway container during a live goal** (the
  documented starvation-abort).

### 8.4 F4 — managed lifecycle bring-up (robot-coupled — NOT started, spike last)

**Why:** `robot.launch.py` starts ~19 nodes at once though real ordering deps
exist (EKF needs camera+description+gyro; slam/nav2 need EKF). Lifecycle gives
ordered activation + deactivate/reactivate of a wedged node without a container
restart (nav2 already uses `nav2_lifecycle_manager`). **Honest ROI: low** — most
own nodes are already inert-until-called; real value is only the always-on
perception path. Robot-coupled: transitions/ordering only exist live. Recommend a
**minimal spike**, not a blanket conversion. **Do §9.2 (N4 fail-fast
respawn/OnProcessExit) FIRST — it buys most of F4's motivation (dead node ≠
half-running stack) at ~5% of the cost; re-evaluate whether F4 is still worth
the spike afterwards.**
- Convert only always-on nodes whose restart needs a container bounce and whose
  ordering matters (candidates: `gyro_calibrator`, `health_monitor`, `estop`,
  `tilt_monitor`) from `rclpy.node.Node` to `rclpy.lifecycle.LifecycleNode` (move
  pub/sub/timer creation into `on_configure`/`on_activate`, teardown in
  `on_deactivate`). Manage with upstream `nav2_lifecycle_manager` in
  `robot.launch.py` (explicit `node_names` order, `autostart: true`).
- Add a `run_lifecycle_node` variant to `node_util.py` (keep the single-entry
  discipline). New ADR at build time (which nodes are managed and why; which
  stay plain).
- Verify (robot): `ros2 lifecycle get <node>` = `active`; `set <node>
  deactivate`/`activate` cycles without a container restart; ordered autostart.

### 8.5 Humble-capability adoptions ✅ CODE COMPLETE (2026-08-15) — Pi verify open

Two changes from the ROS 2 Humble core-docs audit (corpus has no nav2/slam docs;
these are the core capabilities that map to documented pain points).

**A. nav2 composed bring-up** (`scout/launch/nav2.launch.py`). Upstream
`navigation_launch.py` gained `use_composition` handling in the scout wrapper:
it only LOADS components (`bringup_launch.py` normally makes the container), so
the wrapper now starts `component_container_isolated` named `nav2_container`
itself and passes `use_composition`/`container_name` through. All 8 nav2 nodes
become components in one process — one executor, intra-process comms, 1 DDS
participant instead of 8. Targets the `Control loop missed 15 Hz` contention +
the throwaway-container goal abort (nav2 was ~39% of a core as 8 processes).
Default `use_composition:=true`; `false` restores one-process-per-node for A/B.
- Verify (Pi): nav2 up → all 8 lifecycle nodes active, node names unchanged
  (`/controller_server` etc. survive composition); one Foxglove goal succeeds;
  `docker stats` nav2 CPU + control-loop-miss count over one drive vs a
  `use_composition:=false` run. cmd_vel remaps are per-component in upstream's
  LoadComposableNodes (verified against 1.1.20 source) — `/cmd_vel_nav` →
  `/cmd_vel` plumbing unchanged.

**B. Fast DDS Discovery Server** (compose `discovery` service, id 0,
127.0.0.1:11811; `ROS_DISCOVERY_SERVER` on the `&base` env). Replaces multicast
simple discovery — fixes the documented throwaway-container discovery false
negatives and shrinks discovery traffic. Discovery Server v2 filters by topic,
so plain-client shells see a near-empty graph: diagnostic shells need
`FASTRTPS_DEFAULT_PROFILES_FILE=/ros_ws/src/scout/config/super_client.xml` +
`ros2 daemon stop && ros2 daemon start` (recipe in the XML header). Server-id 0
is encoded in the XML's RemoteServer prefix — change both together.
- Verify (Pi): `up -d` → every service's own log shows normal startup (data
  topics flowing: `/scan`, `/odom`, `/map`); Foxglove connects and lists topics;
  webui drives; a throwaway shell WITH the super-client profile lists the full
  graph in <2 s (the old 16 s false-negative case), and one WITHOUT it showing
  near-nothing confirms the server is actually in use. `fastdds` CLI presence:
  `docker compose run --rm discovery which fastdds` if the service fails to
  start.
- Rollback (either item): `use_composition:=false` on the nav2 command /
  remove `ROS_DISCOVERY_SERVER` + the `discovery` service. Independent knobs.

---

## 9. Nav/autonomy additions (2026-08-15 corpus + nav2 audit) — NONE started

Second audit pass: full ros-development Humble corpus (294 pages) + repo
gap-scan against nav2 1.1.20 features. Grounding facts: no collision monitor,
no costmap filters, NavFn+DWB only (Smac/MPPI exist solely as nav2.yaml
comments), `assisted_teleop` behavior configured but never called, zero
callback-group / parameter-callback / message_filters usage anywhere,
twist_mux → roboclaw has no safety stage, `always_send_full_costmap: True`
still on (TODO in nav2.yaml — flip both costmaps to False when Foxglove
costmap debugging is done), EKF process noise untuned (CLAUDE.md flags it).
N2/N3 from this audit were folded into §8.2/§8.1 directly. Suggested order:
9.3 (Mac, minutes) → 9.2 → 9.1 → 9.4 → operator priority among 9.5–9.8.

### 9.1 N1 — nav2_collision_monitor ✅ CODE COMPLETE (ADR-0016) — on-blocks verify open

Safety stage BETWEEN twist_mux and the driver: `/cmd_vel_out` →
`collision_monitor` → `/cmd_vel_safe`; roboclaw remap changes once in
`robot.launch.py`. Stop + slowdown polygons fed by `/scan` (optionally the
depth cloud later). Protects **every** cmd_vel source — webui pad, joystick,
tricks, follow_me, skills — not just nav2, and directly mitigates the
documented "goal failed ≠ robot stops" / latched-goal hazards at the last
hop. Config + launch only: package ships in nav2 1.1.20 apt (confirm on Pi:
`ros2 pkg list | grep collision`). Polygons from the measured footprint
(±0.169 × ±0.167); slowdown ring outside it. Runs in the robot service (it
must be up whenever the driver is). Decisions for the ADR: interaction with
estop (estop stays upstream in twist_mux — CM is a second, independent
layer); whether follow_me's intentional close-approach needs a CM exclusion
(likely tune `stop` polygon inside the follow standoff instead). Verify
on-blocks then floor: drive at an obstacle → slowdown zone trims speed, stop
zone zeroes it; confirm added latency doesn't break the 200 ms deadman
(CM republishes at input rate).

### 9.2 N4 — fail-fast bring-up ✅ CODE COMPLETE (ADR-0015) — Pi kill-test open

`robot.launch.py` starts 20 nodes; a dead rplidar/realsense/roboclaw process
today leaves a half-running stack (only led_node has respawn). Humble-native,
~2 lines/node: `respawn=True, respawn_delay=2.0` on recoverable drivers
(rplidar, apriltag, joystick) and `RegisterEventHandler(OnProcessExit(...))`
→ `EmitEvent(Shutdown())` on load-bearing ones (roboclaw_driver, camera, ekf,
robot_state_publisher) so compose `restart: unless-stopped` recycles the
whole service cleanly instead of limping. Deadman note: a respawning
roboclaw_driver coasts the drivetrain — same as today's crash behavior, no
new hazard. Buys most of F4 (§8.4) at ~5% cost. ADR: which node gets which
policy and why. Verify (Pi): `kill -9` a driver PID inside the container →
respawn case comes back publishing; shutdown case recycles the service and
the stack returns healthy.

### 9.3 N5 — SC11: no sync service/action calls ✅ DONE (test_conventions + ADR-0013)

Humble docs (Sync-Vs-Async how-to): `Client.call()` from inside any
subscription/timer/service callback deadlocks **with no warning, no
exception, no stack-trace evidence**. F2 (record services) and F3 (cancel
from a Trigger callback) are exactly this shape. Adopt now as a structural
rule: **SC11 — no `.call(` on rclpy service clients in `scout/scout/`**
(AST/grep check in `test_conventions.py`; async + done-callbacks only —
link_watchdog's CancelGoal pattern is the house reference). Land with a
one-paragraph note in ADR-0013's rule table. Gate: ruff + pytest.

### 9.4 N9 — `ros2 doctor --report` in the deploy runbook (zero code)

Emits a QOS COMPATIBILITY LIST naming publisher/subscriber node pairs (e.g.
"Best effort publisher and reliable subscription") — the silent-failure class
documented three times in CLAUDE.md — plus pub-without-sub warnings. Add one
line to the deploy/verify recipe: run it in a **super-client** shell
(`FASTRTPS_DEFAULT_PROFILES_FILE=.../super_client.xml` + daemon restart, per
§8.5) or Discovery Server filtering blinds it. Not a liveness oracle (§ DDS
discovery caveats still apply) — it's a QoS-mismatch linter.

### 9.5 N6 — Fast DDS async publish mode + UDP-frag sysctls (measure-first)

(a) Per-topic Fast DDS XML profile (XML infra exists — super_client.xml)
setting `publishMode>ASYNCHRONOUS` for the realsense pointcloud + `/scan`
publishers so a blocking network write can't stall the sensor callback
thread. Safe unconditionally; needs `FASTRTPS_DEFAULT_PROFILES_FILE` on the
robot service env. (b) Host sysctls from the Humble DDS-tuning guide:
`net.ipv4.ipfrag_time=3`, `net.ipv4.ipfrag_high_thresh=134217728`, raised
`net.core.rmem_max` — the documented multi-MB-message stall is one lost UDP
fragment jamming the 256 KB reassembly buffer for 30 s. Localhost-only DDS
lowers but doesn't eliminate exposure (loopback still fragments > MTU). Land
(b) only if `/scan`/depth dropouts are actually observed — record baseline
`ros2 topic hz` under load first.

### 9.6 N7 — costmap filters: keepout + speed-restriction zones (ask operator first)

nav2 1.1.20 ships `KeepoutFilter`/`SpeedFilter`. Mask PGM drawn over the
saved map (stairs, cable zones, slow-near-X), served by a standalone
`map_server` + `costmap_filter_info_server`, filter plugin added to both
costmaps. Fits the maps/ + overlay machinery (mask per map). Only build if
the operator actually wants zones — no current documented need. Medium
effort; robot only for the final drive test.

### 9.7 N8 — MPPI controller A/B (robot-coupled; after 9.1)

`nav2_mppi_controller` is in Humble ≥1.1.8 (image has 1.1.20 — confirm
`ros2 pkg list`). nav2.yaml already says "MPPI is the real answer for
reversing maneuvers"; DWB runs `min_vel_x: 0.0` and tight_tunnel wants
−0.15. Trial as a **profile overlay** (`FollowPath` plugin swap; keep the
RotationShim wrapper). Risk is CPU on the Pi 5 — MPPI is the heaviest nav2
controller; the composed bring-up + the control-loop-miss gauge are the
measurement. A/B one goal course: CPU, misses, arrival error vs DWB.

### 9.8 N10 — EKF process-noise tuning (robot-coupled measurement)

Promoted from CLAUDE.md (absent here until now): yaw covariance grows
~1.7 rad²/30 s against 0.07 deg/min real drift — wildly pessimistic. Harmless
today, load-bearing the moment anything gates on pose uncertainty (9.6
zones, F3 `/nav_state`, coverage). Edit `process_noise_covariance` (+
`initial_estimate_covariance`) in `ekf.yaml`; deliverable is the re-measured
`twist.covariance[35]`/pose-covariance growth stationary and over a standard
drive, sane vs the known drift numbers.

### 9.9 Surveyed and REJECTED/DEFERRED (recorded so the audit isn't redone)

- **SmacPlanner/Theta\*** — NavFn measured fine; no planning failure to fix.
  Revisit only if pipe work shows path-quality problems.
- **Assisted-teleop action wiring** — superseded by 9.1 (CM protects teleop
  lower in the chain, always-on, no action-client plumbing in the webui).
- **tf2_ros MessageFilter in clutter_mapper/follow_me** — latest-time lookups
  are deliberate; swapping changes a safety-adjacent path for a
  startup-window-only gain.
- **Waypoint-follower task executors (PhotoAtWaypoint)** — patrol_capture
  already owns this with more logic than the plugin offers.
- **Content-filtered topics** — Humble support is rclcpp + Fast DDS only; no
  rclpy API → no consumer in this stack.
- **Topic statistics** — rclcpp-only on Humble; our nodes are Python.
- **Loaned messages / zero-copy** — LaserScan/PointCloud2 are non-POD → Fast
  DDS can't loan them; know `ROS_DISABLE_LOANED_MESSAGES=1` exists as a kill
  switch.
- **ros2_tracing / LTTng** — the right instrument for per-callback durations,
  but needs a tracetools source build in the image; defer until a concrete
  CPU mystery demands it (first levers in nav2.yaml come first).
- **backward_ros** — only C++ code is upstream forks; not patching those.
- **Groot BT monitoring** — no Groot client in the toolchain (Foxglove only).
- **SROS2** — Fast DDS `-DSECURITY=ON` source rebuild + key management, no
  threat model on a lab LAN.
- **Per-node log levels / `--disable-rosout-logs` / THROTTLE macros** — real
  but tiny; note `--log-config-file` is UNIMPLEMENTED on Humble's spdlog
  backend. Ops-note territory.
- **`ros2 param dump`/`load` field-tuning loop** — NOTES.md recipe line, not
  a milestone. (Types are static since Galactic; YAML `off`/`on` parse as
  bools — `!!str` to escape.)
- **Iron+-only, do not chase on Humble:** service introspection, matched
  events, runtime logger-level services, `ROS_AUTOMATIC_DISCOVERY_RANGE`/
  `ROS_STATIC_PEERS`, message_filters LatestTime, rosbag2 loss observability,
  zenoh RMW.
