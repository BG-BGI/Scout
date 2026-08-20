# ADR-0021: No companion bridge — plain DDS; companion is a Linux host on the LAN

Status: accepted · Date: 2026-08-19

## Context

The original offboard plan had a custom `companion_link` process as the only
thing crossing the network — one connection to build, watch, and declare
"companion lost" on. With ADR-0020 the Pi and companion share a DDS domain, so
the question became how the companion (the operator's Mac) physically joins.

**Docker Desktop on macOS cannot carry a DDS data plane**: containers sit
behind a Linux-VM NAT, advertised locators are VM-internal, multicast doesn't
cross, and the "host networking" option is a port-proxy that still advertises
wrong locators. A discovery server fixes discovery, not data. Do not retry
this.

## Decision

**No custom transport. The companion stack (`companion/`) runs on a native
Linux host on the robot's LAN (primary: the Debian box; alternative: a
bridged UTM Linux VM on the Mac), using `network_mode: host` Docker, joining
domain 17 via the Pi's LAN-bound discovery server.** From the graph's point
of view the companion is just another Linux host, same as the Pi. Host
distro is irrelevant — the stack is containerized (`ros:humble-ros-base`,
multi-arch), so Debian works even though ROS apt debs are Ubuntu-only.

Rejected:
- Docker Desktop directly — see above.
- Fast DDS TCP transport XML — participant-level XML on every Pi service,
  least-documented path, buys nothing over the VM.

**Sanctioned fallback: `zenoh-bridge-ros2dds`** (per-topic allowlist, single
outbound TCP, NAT-tolerant, runs fine under plain Docker Desktop) if the
ADR-0020 re-test reproduces the wlan0 lockup or raw DDS UDP proves unusable on
corp WiFi. Flipping to it is a config change, not a re-litigation —
`docs/offload-plan.md` §Transport already sketches the shape.

## Consequences

- Nothing on the Pi depends on the companion (spec §0.7): Mac sleep/roam just
  pauses rtabmap; discovery clients re-register on their own when it returns.
- No transport chokepoint exists any more, so "companion loss" is no longer a
  single event. **Deferred §6 decisions, recorded for Phase 3** (user,
  2026-08-19): staleness = a **mix** — DDS deadline/liveliness QoS for
  continuous topics (pose/map updates), timestamp-age checks for episodic ones
  (paths, exploration goals); loss model = **per-topic refusal only**, no
  aggregate "companion lost" state.
- Teleop-gating enforcement is purely the twist_mux input allowlist
  (ADR-0001) + SC4 — nothing companion-side may publish to a `/cmd_vel_*`
  teleop topic; audit whenever a companion capability is added.
- RTAB-Map live 3D capture becomes a standing companion capability (consumes
  compressed color/depth, `/odom`, `/tf`, `/scan` directly), superseding the
  bag-record-then-Mac workflow in `docs/slam.md` §5. RTAB-Map compute stays
  off the Pi — that verdict was CPU, not transport.
