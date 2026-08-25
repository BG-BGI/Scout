# Scout offboard architecture — consolidated spec (revised)

Status: **Phases 0–2 in implementation (2026-08-19)** — see ADR-0020
(shared DDS domain, discovery server LAN-bound), ADR-0021 (no bridge;
companion = bridged Linux VM on the Mac; deferred §6 decisions recorded),
`docs/platform.md` (baseline + wlan0 re-test), and `companion/`.
**Phase 1 (§3.3 twist_mux) was already shipped before this spec** — ADR-0001 +
ADR-0016 are authoritative; the §3.3 table here is historical (actual ladder:
stop 255 / joy 100 / web 90 / collision-monitored auto stage at 10, Nav2 on
plain `/cmd_vel` into the auto stage). "ADR 0001/0002" referenced below were
never written under those numbers; they exist as ADR-0020/0021.
This supersedes `offboard-architecture.md` v4 by
incorporating two accepted ADRs and several follow-on decisions made in review.
Target branch base: `robo-trials-v1`. Companion doc: `docs/slam.md` (SLAM
mechanism/measurements), `CLAUDE.md` (hardware, operating limits).

## Revision summary

| # | Decision | Status |
|---|---|---|
| 1 | Drop `ROS_LOCALHOST_ONLY=1`; Pi and companion share one `ROS_DOMAIN_ID`, native DDS discovery | Accepted — see ADR 0001 |
| 2 | Drop `companion_link` entirely; all cross-machine traffic (including paths/goals) rides plain DDS | Accepted — see ADR 0002 |
| 3 | RTAB-Map's live 3D-capture runs on the companion, consuming Pi topics directly over shared DDS | Accepted |
| 4 | WiFi (corp WiFi) is the **primary** link, not a fallback | Confirmed by user |
| 5 | The original wlan0/DDS lockup that motivated `ROS_LOCALHOST_ONLY=1` is **unconfirmed** — possibly observed over ethernet or during unrelated performance issues | Flagged; re-test recommended before field reliance |
| 6 | Authority model (§0.4/§0.6) must move from "one connection to watch" to "per-topic staleness" | Open — not yet designed, see §6 below |

This document carries forward everything from the original plan that ADRs
0001–0002 did not touch, and marks with **⟲ REVISED** every section that
changed as a result of them.

---

## 0. Constraints (revised)

### 0.1 ⟲ REVISED — Network discovery

~~`ROS_LOCALHOST_ONLY=1` stays set on every Pi service.~~ **Superseded by ADR
0001.** The Pi and companion share one `ROS_DOMAIN_ID` and discover each other
via native DDS. The original justification for isolating discovery — a ~10
minute wlan0 lockup after stack start — was never confirmed to be caused by
DDS specifically; it may have been observed over ethernet or during unrelated
performance-degrading changes. Per ADR 0001, this decision is accepted as the
working baseline, **contingent on a clean re-test** (Pi + a second ROS 2 node,
same corp WiFi, confirmed wlan0 active, no confounding changes in flight)
before this ships to a Pi that depends on it in the field. If the lockup
reproduces under a clean test, ADR 0001 should be reversed.

### 0.2 ⟲ REVISED — Cross-machine transport

~~rosbridge and the MCP endpoint stay on the Pi; `companion_link` is the ONLY
process that crosses the network.~~ **Superseded by ADR 0002.** There is no
custom transport. The companion subscribes to and publishes ROS topics/actions
on the Pi (and vice versa) natively, the same as any standard ROS 2
multi-machine deployment. rosbridge and the MCP endpoint (`scout_skills` on
`:9001`, speaking to `127.0.0.1:9090`) still stay on the Pi as the single
Magnus connector — that part of the original constraint is unaffected, since
it concerns Magnus's integration boundary, not the Pi/companion boundary.

### 0.3 Magnus integrates only via MCP. (unchanged)

No Magnus-specific code on the Pi or companion.

### 0.4 ⟲ REVISED — Autonomy authority

The Pi remains the final authority on whether *autonomous* motion may proceed.
Every offboard result — path, transform, exploration goal — is validated
locally before Nav2 may act on it. This authority applies to autonomous motion
only (see §0.6).

**What changed:** validation used to have one natural chokepoint —
`companion_link` — where "is the companion alive" was a single question. With
ADR 0002, there is no single connection to watch. The Pi must independently
judge staleness/health **per topic**: map age, path age, exploration-goal age,
and localization/pose age each need their own answer to "is this fresh enough
to act on." See §6 (open) for the per-topic design work still required before
Phase 3 or Phase 5 can rely on this.

### 0.5 Perception must never veto teleop. (unchanged — see §0.6 for the full authority model)

### 0.6 Authority model — autonomy is gated, teleop is not (unchanged in substance; validation mechanism revised per §0.4)

The distinction between autonomous and human-commanded control classes, and
the reasoning behind it, are unchanged from the original plan:

| Class | Sources | Gating |
|---|---|---|
| **Autonomous** | Nav2 goals, companion paths, exploration goals, patrol/coverage routes, `follow_me` | **Fully gated.** Costmap, collision monitor, path validation, map-version and staleness checks all apply. May be refused or stopped. |
| **Human-commanded** | webui joystick, Bluetooth gamepad, `/dev/input/js0`, MCP `move`/`rotate`, trick macros | **Not gated by perception.** Reaches `twist_mux` and the driver regardless of costmap state. Bounded only by kinematic/electrical limits. |

**Why.** The costmap is frequently wrong in ways the operator is not — inflation
making the robot's own required position read as occupied (why `nav2.yaml`
swapped `BaseObstacle` for `ObstacleFootprint` for tight-clearance work), depth
artifacts in the under-lidar band, a graph re-solve rotating the map frame ~40°
mid-goal (measured, `docs/slam.md` §3), and stale marks surviving a map-frame
change producing phantom lethal cells. A robot that immobilises itself on a
phantom obstacle while a human with line of sight is actively commanding it is
failing *stuck*, not failing safe.

**What still bounds teleop** (unchanged):
- kinematic limits (`max_linear_velocity: 1.0`, `max_angular_velocity: 3.0`)
- the velocity-loop floor (~0.05 m/s, ~0.35 rad/s)
- RoboClaw current limits and the 16.0 V Min Main cutoff
- the 200 ms deadman
- the `twist_mux` e-stop lock (§3.3), operator-commanded
- `tilt_monitor`, authoritative — a rollover is a physical fact, not a perception guess

**Required behaviour** (unchanged from original, still binding):
1. `twist_mux` priorities in §3.3 encode this: teleop 100, skills 80 outrank Nav2 at 40.
2. Nav2's collision monitor and costmap gate only `/cmd_vel_nav2` — never the
   mux output, never teleop inputs.
3. A perception-triggered stop must be visibly attributed (webui/LED
   distinguish "autonomy refused" from "robot unresponsive").
4. Taking manual control must cancel the autonomous goal, not merely outvote it.
5. `move`/`rotate` preempt and cancel an active Nav2 goal rather than deferring to it.
6. Nothing on the companion may gate teleop — a perception/planning input that
   crossed the network is the *least* trustworthy source of a stop decision.
   **This is now more directly relevant than before:** with plain DDS (ADR
   0002), the companion's outputs arrive as ordinary topics with no
   `companion_link` boundary to enforce this at. The `twist_mux`
   architecture in §3.3 is what actually enforces it (nothing from the
   companion ever publishes to a `/cmd_vel_*` teleop topic), and that
   enforcement needs to be verified explicitly now that there's no
   transport-level chokepoint backing it up implicitly.

**Where autonomy should still refuse** (unchanged): failed path validation,
map-version mismatch, stale localization, companion loss (redefined per §6),
tilt alarm, critical battery.

### 0.7 The companion is not required for the Pi to stop. (unchanged)

Companion-loss handling is enforced on the Pi. What "companion loss" means is
redefined in §6 (open).

---

## 1. Target architecture ⟲ REVISED

```
Magnus
  │ MCP (streamable-http)
  ▼
scout_skills :9001 ─── FastMCP proxy mounts ──┬── ros_mcp :9000 (Pi, rosbridge primitives)
  (Pi)                                        └── companion vision/map tools (Phase 5)
  │
  │ rosbridge ws://127.0.0.1:9090
  ▼
┌─────────────────────── Pi ROS graph ─────────────────────────────────────────┐
│ roboclaw_driver · rplidar · realsense · robot_state_publisher               │
│ gyro_calibrator · ekf_node · tilt_monitor · led_node · battery_monitor      │
│ behaviors: trick_player · follow_me · patrol_capture                        │
│ local_costmap · controller_server (rotation shim + DWB) · smoother_server   │
│ behavior_server · bt_navigator · waypoint_follower · velocity_smoother      │
│ twist_mux  ← last hop before the driver                                    │
└─────────────────────────────────────┬───────────────────────────────────────┘
                                      │ shared ROS_DOMAIN_ID, native DDS
                                      │ discovery over corp WiFi (PRIMARY link)
┌─────────────────────────────────────┴───────────────────────────────────────┐
│ Companion ROS graph (same domain — no isolation, no custom bridge)          │
│ planner_server + global_costmap · explore_lite                              │
│ slam_toolbox (Phase 6, gated) · YOLO detector (Phase 5)                     │
│ rtabmap (live 3D capture — consumes Pi camera/depth/odom/tf/scan directly) │
│ map + posegraph + waypoint storage · companion_agent                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

**What changed from the original diagram:** one shared ROS graph across two
machines instead of two isolated graphs bridged by a custom transport.
`companion_link` no longer exists. RTAB-Map is now shown explicitly as a
companion-side, always-available consumer of Pi sensor topics — not gated
behind any phase.

**What did not change:** the Magnus integration boundary (MCP only, `scout_skills`
on the Pi as the single connector), and the physical topology (Pi drives the
robot, companion never gates teleop).

---

## 2. How to read the existing repo notes (unchanged from original — still applies)

⚠ `CLAUDE.md` and `docs/slam.md` were written under a single-machine premise.
Classify every existing note before importing it:

| Class | Behaviour under the companion pattern | Examples |
|---|---|---|
| **Physical / hardware** | Carries forward unchanged | Lidar 0.15–16.0 m limits, `base_frame: base_link`, wheel radius, 200 ms deadman, no BMS, no RTC, ~15 V motor-volts per m/s |
| **Measured network fact** | **Now unconfirmed — see §0.1** | Cross-network DDS blackholing wlan0 |
| **Algorithmic property** | Carries forward; offload does not fix it | Graph re-solve can rotate the map frame mid-goal; skid-steer scrub; li-ion curve flatness |
| **CPU-forced compromise** | Invert — the point of the offload | `transform_timeout: 0.8`, DWB `transform_tolerance: 0.5`, keyframe gating at 0.3, `map_update_interval: 2.0`, `loop_search_maximum_distance: 3.0`, `controller_frequency: 15.0`, trimmed DWB samples, `resolution: 0.05`, RealSense filters off, "do not run RTAB-Map on the Pi" |
| **Single-machine workaround** | **Superseded — see §2.1** | Bag-record-then-RTAB-Map-on-Mac; `enable_interactive_mode: false` |

### 2.1 ⟲ REVISED — RTAB-Map path

`docs/slam.md` §5's "offboard shape that works" (record a bag, transfer in
retryable chunks, run rtabmap on the Mac) was a workaround for having *no live
link at all*. That constraint is gone twice over now: first because the
original plan's `companion_link` would have carried it live, and second and
more directly because shared DDS (ADR 0001/0002) lets the companion subscribe
to `/camera/camera/color/image_raw`, aligned depth, `camera_info`, `/odom`,
`/tf`, `/tf_static`, `/scan` **natively, with no bridge to build**. RTAB-Map's
live 3D-capture therefore runs on the companion as a standing capability, not
gated behind any particular phase — it only requires the companion to be on
the network and reachable, which is true from the moment the companion boots
onto the shared domain.

**Still true, unchanged:** do not run RTAB-Map's compute on the Pi. That
verdict was about Pi CPU headroom (Ceres already starves the TF publish
loop), which is orthogonal to the transport question and unaffected by ADRs
0001/0002.

**All other tuning-ladder inversions from the original §2.1 are unchanged**
(they concern CPU budget, not transport, and none of them depended on
`companion_link` or `ROS_LOCALHOST_ONLY`):
- `transform_timeout: 0.8` and DWB `transform_tolerance: 0.5` are revert
  candidates once Ceres is off the Pi (Phase 6) — reverting toward upstream
  defaults is an acceptance criterion, not a risk.
- `slam.yaml`'s tuning ladder inverts once CPU pressure is relieved:
  `minimum_travel_distance/heading` and `map_update_interval` move tighter,
  not looser; `loop_search_maximum_distance: 3.0 → 5.0–6.0`;
  `resolution: 0.05 → 0.025`; `ceres_loss_function: HuberLoss` has no CPU
  objection at all.
- `controller_frequency: 15.0` can rise toward 20 once headroom exists;
  DWB sample counts can rise toward upstream's 20×20.
- Per-service CPU caps need re-budgeting for the new steady state, not
  preservation of single-machine-contention values.

### 2.2 ⟲ REVISED — Sequencing rationale

| Phase | Scope | Reversible? | Gate to proceed |
|---|---|---|---|
| 0 | Re-baseline measurements **+ wlan0/DDS re-test (§0.1)** | n/a | — |
| 1 | `twist_mux` + Scout-owned nav2 launch | yes | Pi-local, no companion |
| 2 | Shared-domain DDS bring-up (companion joins `ROS_DOMAIN_ID`); **RTAB-Map live capture available from this point** | yes | Phase 0 re-test passes; recorded-data replay passes |
| 3 | `planner_server` + global costmap offboard, over plain DDS topics/actions | yes (local fallback) | Phase 2 stable; **per-topic staleness design from §6 in place** |
| 4 | Re-measure; decide on SLAM | n/a | Phase 3 in service ≥1 week |
| 5 | YOLO + map tools to companion proxy | yes | Phase 2 stable |
| 6 | `slam_toolbox` offboard (conditional) | hard | Phase 4 says headroom insufficient |

**What changed:** Phase 0 now explicitly includes the wlan0/DDS re-test — the
original constraint this whole plan is built on is unconfirmed (§0.1) and
should not be carried into Phase 1+ without it. RTAB-Map's live capture is
called out as available starting Phase 2 rather than tied to Phase 5. Phase 3
now has an explicit dependency on the per-topic staleness design (§6) since
that phase is exactly where "is this companion path fresh enough to act on"
first becomes load-bearing.

**Why planner-before-SLAM (unchanged).** Offloading `planner_server` is the
textbook global/local split Nav2 already implements — it consumes a path at
0.2–1 Hz, tolerates latency, and is validated locally. Offloading
`slam_toolbox` makes the robot's *pose* network-dependent. These are not the
same risk. Do the cheap one, re-measure, and only take the expensive one if
the numbers demand it.

---

## 3. Pi environment (unchanged from original — transport/discovery changes don't affect host config)

### 3.1 Host baseline

| Item | Value |
|---|---|
| Board | Raspberry Pi 5, 16 GB |
| OS | Debian (confirm + pin version) |
| Hostname | `scout` (mDNS → `http://scout.local`) |
| Required boot config | UART on GPIO14/15 → `/dev/ttyAMA0`; SPI0 for APA102 |
| Devices | `/dev/ttyAMA0` (RoboClaw), `/dev/ttyUSB0` (RPLIDAR), D455 USB 3.2, `/dev/input/js0` |
| Clock | No RTC. NTP gate blocks sensor containers until synced |
| Power | DeWALT 20V MAX, no BMS; RoboClaw 16.0 V Min Main is the only pack protection |
| Docker | Engine + Compose v2, `docker.service` enabled |
| **Network** | **Corp WiFi is the PRIMARY link (confirmed), not a fallback.** This raises the expected frequency of link degradation/dropout relative to a dedicated robot network — see §6. |

Add `docs/platform.md` and a non-destructive `scripts/preflight.sh` asserting:
64-bit, UART/SPI enabled, all four devices present, NTP synced, Docker
running, thermals nominal.

### 3.2 Compose services (target state) — ⟲ REVISED

Caps shown are the current single-machine values; per §2.1 they were sized
for contention the offload removes, so treat them as a starting point to
re-budget in Phase 4.

| Service | Profile | CPU cap | Change | Notes |
|---|---|---|---|---|
| `robot` | dev | 2.0 | — | drivers, EKF, LED, tilt |
| `behaviors` | dev | 0.5 | — | trick/follow/patrol |
| `rosbridge` | dev | 0.3 | — | ws://:9090, localhost consumers only |
| `webui` | dev | 0.2 | — | :80, bind-mounted `./webui` |
| `foxglove_bridge` | dev | 0.3 | — | :8765 |
| `fleet_status` | dev | 0.3 | extend | + companion pairing/health panel |
| `twist_mux` | dev | 0.1 | NEW | last hop before driver — see §3.3 |
| `nav2` | full | 1.5 → 1.0 | rewrite | Scout-owned launch, no planner_server |
| ~~`companion_link`~~ | ~~full~~ | ~~0.5~~ | **REMOVED (ADR 0002)** | superseded by shared DDS domain — no dedicated bridge process |
| `ros_mcp` | full | 0.5 | — | rosbridge primitives, internal only |
| `scout_skills` | full | 1.0 → 0.5 | extend | + companion proxy mount (Phase 5) |
| `slam` | full | 1.0 | Phase 6 | stays on Pi until Phase 4 says otherwise |
| `explore` | explore | 1.0 | moves | needs global costmap → companion (Phase 3) |

**⚠ Environment variable removal required (ADR 0001):** the `x-service` base
block's `environment: - ROS_LOCALHOST_ONLY=1` and the comment above it
("Keep DDS off wlan0...") need to be removed/rewritten across every service
that inherits `<<: *base`, and `ROS_DOMAIN_ID` set explicitly and consistently
on both the Pi and the companion.

Expected Pi reclaim: `nav2` −0.5, `explore` −1.0, `scout_skills` −0.5 (YOLO
offload). `companion_link`'s 0.5 no longer needs reclaiming since it's never
built. `slam` −1.0 only if Phase 6 proceeds.

**Spend part of the reclaim rather than banking all of it** (unchanged):
revert `transform_timeout` and DWB `transform_tolerance` toward upstream;
raise `controller_frequency` 15 → 20; restore DWB sample counts toward
upstream's 20×20.

### 3.3 `twist_mux` — new, do this first (unchanged from original)

`/cmd_vel` currently has multiple unarbitrated writers. Replace with priority
arbitration at the last hop.

Add `ros-humble-twist-mux` to the Dockerfile apt layer and
`<exec_depend>twist_mux</exec_depend>` to `scout/package.xml`.

`scout/config/twist_mux.yaml`:

| Input | Topic | Priority | Timeout |
|---|---|---|---|
| Webui / joystick teleop | `/cmd_vel_joy` | 100 | 0.3 s |
| Skills `move`/`rotate` | `/cmd_vel_skills` | 80 | 0.3 s |
| Trick player | `/cmd_vel_trick` | 60 | 0.5 s |
| Nav2 | `/cmd_vel_nav2` | 40 | 0.5 s |

Locks: `/twist_mux/estop` (latched `Bool`, priority 255) blocks every input
below it. Software stop, not a substitute for the hardware e-stop.

Two behaviors to preserve:
- `twist_mux` goes silent when all inputs time out rather than publishing
  zeros, so the RoboClaw's 200 ms deadman remains the actual stop.
- Keep the explicit zero-twist bursts `move`/`rotate` and the webui already send.

Retire the `move`/`rotate` "refuses while nav goal active" guard once
priority arbitration is verified. Per §0.6, these are human-commanded and
should **cancel** the Nav2 goal rather than defer to it.

**Do not add a perception filter downstream of `twist_mux`.** This is now
more important to verify explicitly, not less: with `companion_link` gone
(ADR 0002), there's no transport-level boundary stopping a companion-side
node from publishing directly to a `/cmd_vel_*` topic if someone wires it up
carelessly later. The enforcement is entirely in which topics `twist_mux` is
configured to arbitrate — audit this whenever a new companion capability is
added.

**Teleop activity must cancel the active goal** (unchanged): a live
`bt_navigator` goal keeps replanning at 1 Hz and resumes the moment teleop
goes silent.

---

## 4–5. [Carried forward from original plan, unmodified by these ADRs]

Sections 4 (companion environment) and 5 (companion-loss/staleness detail,
pre-revision) from the original `offboard-architecture.md` are not
reproduced here verbatim — they describe implementation detail that is now
superseded by §6 below and should be treated as historical context only,
not as current design. Consult the original doc for the pre-ADR detail if
needed, but do not implement anything from those sections without
reconciling it against §6 first.

---

## 6. OPEN — Per-topic authority/staleness model (not yet designed)

This is the direct consequence of ADR 0002 and is **not yet resolved.**
Nothing below should be treated as decided; it is scoped as the next design
task.

**What used to exist:** one `companion_link` connection. "Companion loss" was
a single, well-defined event — the link drops, the Pi stops trusting
everything from the companion.

**What has to exist now:** the Pi independently judges freshness/health for
each safety-relevant topic coming from the companion. Candidates identified
so far (not yet confirmed complete):
- `planner_server` path output
- `explore_lite` / exploration goal output
- the map itself (occupancy grid / map version)
- localization/pose freshness (already named in §0.6's refuse-list as "stale
  localization" — needs to be explicitly folded into this same per-topic model
  rather than treated as a separate legacy check)

**Still to decide, in order:**
1. Confirm the complete list of topics that need independent staleness
   tracking (the four above are a first pass, not final).
2. For each, define what "stale" means — a fixed timeout? A QoS
   liveliness/deadline policy enforced by DDS itself, so the Pi gets a
   callback rather than polling timestamps? Some mix?
3. Decide whether "companion loss" as a single concept still means anything
   (e.g. as a derived/aggregate state — "companion considered lost if N of
   its topics are simultaneously stale") or whether it should be retired in
   favor of purely per-topic refusal ("this specific path is stale, so this
   specific goal is refused" with no broader "companion is down" state at all).
4. Re-derive the Phase 6 grace-period number (previously "~1s, explicitly a
   guess" in the original plan) against whatever mechanism is chosen in (2) —
   it should not simply be carried forward unchanged, since it was sized
   against a link-level heartbeat that no longer exists.
5. Given corp WiFi is confirmed primary (not a fallback), size all of the
   above assuming degradation/dropout is a frequent, expected-to-fire
   condition rather than a rare edge case. Phase 0's re-baseline measurements
   should include measuring actual WiFi dropout frequency/duration for the
   continuous-streaming traffic profile (not the bulk-transfer profile
   `docs/slam.md` already measured) before any of the numbers in this section
   are finalized.

---

## 7. Decisions still pending user input

- The complete list of topics requiring per-topic staleness tracking (§6.1).
- The staleness mechanism per topic — fixed timeout vs. DDS QoS
  liveliness/deadline vs. a mix (§6.2).
- Whether an aggregate "companion loss" concept survives at all, or whether
  refusal becomes purely per-topic (§6.3).
- The Phase 6 grace-period re-derivation (§6.4).
- Scheduling/ownership of the Phase 0 wlan0/DDS re-test (§0.1) and the WiFi
  dropout-frequency measurement (§6.5) — both are prerequisites this document
  currently treats as blocking, but neither has an owner or date yet.

## References

- ADR 0001 — Drop `ROS_LOCALHOST_ONLY=1`; use native DDS discovery across Pi and companion
- ADR 0002 — Drop `companion_link`; all Pi↔companion traffic rides plain shared DDS
- `docs/slam.md` — SLAM pipeline, measurement protocol, tuning rationale (BG-BGI/Scout, `robo-trials-v1`)
- `CLAUDE.md` — hardware, operating limits (BG-BGI/Scout, `robo-trials-v1`)
- Original plan: `offboard-architecture.md` v4 (superseded by this document)
