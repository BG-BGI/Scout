# ADR-0020: Shared DDS domain across Pi and companion; discovery server LAN-bound

Status: **SUPERSEDED by ADR-0022 (2026-08-20)** — the re-test found corp
inter-VLAN filtering drops Pi↔companion UDP outright (zero packets arrived;
the wlan0-lockup question was never reached). `ROS_DOMAIN_ID=17` survives;
the LAN-bound discovery server and cross-machine DDS do not. · Date: 2026-08-19

## Context

`docs/offboard-architecture.md` moves planner/mapping compute to a companion
machine. The Pi's DDS was locked to loopback (`ROS_LOCALHOST_ONLY=1`) because
DDS on wlan0 was blamed for a ~10 min corp-WiFi blackhole after stack start —
but that cause was **never confirmed** (possibly ethernet or an unrelated
regression). Meanwhile the stack already runs a Fast DDS Discovery Server v2
(loopback `:11811`), not multicast simple discovery.

## Decision

**The Pi and companion share `ROS_DOMAIN_ID=17` with a native DDS data plane.
Discovery stays server-based — the existing discovery server, rebound from
`127.0.0.1` to `0.0.0.0:11811` — not multicast.**

- `ROS_LOCALHOST_ONLY=1` is removed from every service (compose `&base`,
  foxglove_bridge, rosbridge, and the exec prefixes in
  `docker/observability-exporter/exporter.py` /
  `docker/observability-mcp/server.py`).
- Pi services keep `ROS_DISCOVERY_SERVER=127.0.0.1:11811` — loopback still
  reaches the LAN-bound server, so the Pi standalone path is byte-identical
  when no companion exists (spec §0.7).
- Companion services set `ROS_DISCOVERY_SERVER=<pi-ip>:11811`. Data plane is
  peer-to-peer UDP; only discovery goes through the server.
- The spec's "native DDS discovery" wording predates knowledge of the
  discovery-server topology; this ADR amends it to **native data plane,
  server-based discovery**. Multicast stays off: corp WiFi frequently filters
  it, and point-to-point discovery keeps the re-test blast radius small.

## Consequences

- **Gate:** the two-step wlan0/DDS re-test in `docs/platform.md` must pass
  before this merges. If the lockup reproduces, adopt the ADR-0021 zenoh
  fallback and revert the rebind.
- Accepted risk: the ROS graph is now reachable from the corp LAN (`-i 0`
  server on `0.0.0.0`, unauthenticated DDS). Anything on the subnet can
  publish — including to `/cmd_vel_*` inputs. The twist_mux allowlist
  (ADR-0001) plus `test_conventions.py` SC4 remain the enforcement; audit on
  every new companion capability (spec §3.3 note).
- Shell introspection still needs SUPER_CLIENT (`super_client.xml`) —
  unchanged.
- Sensor publishers gain WiFi subscribers, so SYNCHRONOUS publish can stall a
  sensor callback on a blocked wlan0 write: enable
  `scout/config/sensor_publish_profiles.xml` on the `robot` service, gated by
  its own before/after `ros2 topic hz` check.
- Supersedes `docs/offload-plan.md` point 1 ("DDS must stay off the wire") and
  the compose comment that carried it.
- Phase 1 of the offboard spec (twist_mux) was already shipped by ADR-0001 +
  ADR-0016; the spec's §3.3 table is historical.
