# SLAM on Scout — pipeline, measurement, tuning, upgrade path

slam_toolbox 2.6.10 (apt), config `scout/config/slam.yaml`, launch
`scout/launch/slam.launch.py`. Operating recipes and the mode/serialization
traps live in CLAUDE.md (SLAM section) — this doc covers mechanism, measured
quality, tuning rationale, and where to go next.

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

| Metric | Baseline (2026-08-03, inflated tires) | Post-recal run 1 | Run 2 |
|---|---|---|---|
| map→odom correction over loop | 0.30 m / 2.1° per ~17 m | TBD | TBD |
| Stationary relocalization | 1 mm | TBD | TBD |
| Return-to-mark error | — (never measured) | 15–30 cm total per ~15–20 m, ≈14 cm of it the xy_goal_tolerance stop-short → **2–16 cm real** (2026-08-14, deflated tires, INCLUDING a mid-drive graph re-solve; start reconstructed through odom) | TBD |
| Loop closures per loop | — | ≥1 (whole-map ~40° re-solve mid-drive) | TBD |

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

**Safety rule derived: do not send autonomous goals toward known WiFi dead
zones until an on-robot link-loss watchdog exists** (e.g. cancel nav goals
when rosbridge has had zero clients for N seconds — design open: conflicts
with wanting offline autonomy later). Redo this protocol inside coverage, on
a fresh map, after the front/rear differential tire-pressure experiment.

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
