# Architecture Decision Records

One page each, lightweight [MADR](https://adr.github.io/madr/) style. These
record *why*; CLAUDE.md records the hardware/tuning measurements and CONTEXT.md
names the concepts. Config files link here instead of restating rationale (so
forking a config no longer forks — and drifts — the reasoning).

| # | Decision |
|---|---|
| [0001](0001-cmd-vel-ownership.md) | cmd_vel arbitration via twist_mux + the software e-stop |
| [0002](0002-depth-costmap-layer.md) | Under-lidar depth as its own costmap layer, never shared with lidar clearing |
| [0003](0003-slam-mode-as-executable.md) | slam mode is the executable, not a parameter |
| [0004](0004-rsusb-librealsense.md) | librealsense built from source with the RSUSB backend |
| [0005](0005-overlay-volume-seeding.md) | One overlay install tree in a named volume; deploy = git pull |
| [0006](0006-apriltag-single-family.md) | One AprilTag family (tag36h11); registry ≠ detection coverage |
| [0007](0007-battery-rest-estimation.md) | Battery SoC from resting voltage only; threshold ladder |
| [0008](0008-deflated-tire-retune.md) | Deflated tires are the operating condition; retune supersedes inflated-era |
| [0009](0009-link-loss-pause.md) | Link-loss cancel-and-stash nav policy |
| [0010](0010-tight-tunnel-profile.md) | Scenario profiles as parameter overlays |
| [0011](0011-waypoint-store.md) | One JSON waypoint/route store shared by patrol + skills |
| [0012](0012-pure-core-testing.md) | Pure `scout.core` + bare-pytest testing |
| [0013](0013-conventions-as-tests.md) | Conventions enforced by ruff + structural tests, gated by CI |
| [0014](0014-unified-diagnostics.md) | Unified health on /diagnostics via one aggregator node |
| [0015](0015-fail-fast-bringup.md) | Three-tier process-exit policy in robot.launch.py (fail-fast bring-up) |
| [0016](0016-collision-monitor.md) | nav2_collision_monitor as the last-hop cmd_vel safety stage |
| [0017](0017-record-on-demand.md) | rosbag record-on-demand via one subprocess-owning node |
| [0018](0018-nav-cancel-and-state.md) | Dispatcher-aware nav cancel + consolidated /nav_state |
| [0019](0019-keepout-speed-zones.md) | Keepout/speed zones — JSON polygons as truth, masks as artifacts |
| [0020](0020-shared-dds-domain.md) | Shared DDS domain across Pi and companion; discovery server LAN-bound |
| [0021](0021-no-companion-bridge.md) | No companion bridge — plain DDS; companion is a Linux host on the LAN |
