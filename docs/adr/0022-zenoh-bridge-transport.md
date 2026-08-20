# ADR-0022: Pi↔companion transport is zenoh-bridge-ros2dds over one TCP port; DDS returns to loopback

Status: accepted · Date: 2026-08-20 · Supersedes ADR-0020's shared-domain
transport (the domain-17 identity and the companion-topology intent of
ADR-0021 carry forward)

## Context

ADR-0020's gate failed. The re-test measured (2026-08-20): Pi's discovery
server LAN-bound on `0.0.0.0:11812`, listener on the companion Debian box
(10.1.57.18, wired) dialing the Pi (10.1.80.31, wlan0) — **zero UDP packets
from the box ever arrived at the Pi** (`tcpdump -ni wlan0 udp port 11812`
silent while the client retried). The machines sit on different corp VLANs
and the network filters inter-VLAN UDP. Plain shared-domain DDS is therefore
unusable for this box placement; the wlan0-lockup question was never even
reached. ADR-0021 pre-named the fallback; this ADR exercises it.

## Decision

**Cross-machine topics ride `eclipse/zenoh-bridge-ros2dds:1.10.0` (the
standalone bridge from `eclipse-zenoh/zenoh-plugin-ros2dds`), one bridge
container per machine, one TCP connection: Pi listens on `:7447`, companion
dials out. Each machine keeps its own loopback-only DDS graph.**

- **The Pi's DDS returns to `ROS_LOCALHOST_ONLY=1` — with SIMPLE discovery,
  not the old discovery server.** The bridge is CycloneDDS-based and cannot
  register with a Fast DDS Discovery Server (server-based PDP is a Fast DDS
  extension), and discovery-server clients don't answer standard SPDP, so a
  bridged graph and a discovery-server graph are mutually invisible. The
  server, `super_client.xml`, and every SUPER_CLIENT workaround are retired:
  compose `discovery` service deleted, rosbridge env simplified, observability
  exec prefixes simplified, `bag_recorder` no longer injects the profile into
  its record subprocess (which would now hang looking for a server).
- **Knowingly reaccepted trade-offs of simple discovery:** throwaway-container
  discovery false negatives return (judge liveness from logs and data topics;
  re-check a negative after 10–20 s), and discovery traffic is multicast on
  loopback again.
- **Per-topic allowlists are the contract** (`scout/config/zenoh_bridge.json5`
  / `companion/config/zenoh_bridge.json5`, kept mirrored): compressed color,
  compressedDepth, color camera_info, `/odom`, `/scan`, `/tf`, `/tf_static` —
  outbound only. The Pi side's `subscribers` list is empty, so nothing
  companion-side can publish into the Pi graph at all — a stronger version of
  the ADR-0001 `/cmd_vel_*` audit, enforced at the transport for the first
  time since `companion_link` was dropped.
- ⚠ **The `ROS_LOCALHOST_ONLY` env var OVERRIDES the config-file key**
  (verified v1.10.0: file `true` + no env ⇒ effective `false`). Both compose
  services set the env var; never rely on the json5 key alone.
- Companion (Debian box, 10.1.57.18) runs its own loopback graph; rtabmap and
  foxglove_bridge consume the bridged topics as if local. The Pi-side bridge
  is the DDS subscriber that triggers the camera's lazy JPEG/PNG encoders, so
  the Pi pays compression cost only while the companion stack is up.
- If inter-VLAN TCP :7447 is also filtered, reverse the roles (companion
  listens, Pi dials) or request that single port — the bridge works either
  direction.

## Consequences

- Latency/HOL: a single TCP stream adds head-of-line blocking under loss —
  irrelevant for mapping topics, and control never crosses machines anyway.
- The ADR-0020 wlan0/DDS re-test is moot for now (DDS no longer touches
  wlan0); its Step-1 procedure in `docs/platform.md` stays as the recipe if
  same-subnet placement ever makes plain DDS attractive again.
- `docs/offload-plan.md`'s zenoh sketch is now the implemented design, with
  one correction: the bridge canNOT join a discovery-server graph, which that
  sketch assumed.
- Every future companion capability extends the allowlists explicitly —
  wildcards are a review-blocking change.
