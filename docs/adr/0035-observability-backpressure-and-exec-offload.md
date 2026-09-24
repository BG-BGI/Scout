# ADR-0035: Observability backpressure and exec offload

Status: accepted · Date: 2026-09-24

## Context

An external source review (handoff 2026-09-24, main @ 95a18a4) flagged the
observability stack as a load *amplifier*: the exporter submitted four poll
sources to a shared ThreadPoolExecutor every 20 s and discarded the futures —
no single-flight guard (slow passes queue behind the pool), and swallowed
exceptions (a websockets protocol error escaping `_sample_hz`'s narrow catch
froze `scout_ros_topic_hz` forever with no log). Worse, `poll_dds` ran nine
separate `docker exec ros2 topic info -v` calls per pass **inside the `robot`
container** — nine double-`source setup.bash` CLI launches charged to the
2.0-cpu cgroup that owns the drivetrain, with no timeout anywhere.
`observability-mcp` duplicated the same untimed exec-into-robot pattern, and
all three observability-profile services ran with no CPU limit on a 4-core Pi
whose capped services already total 8.1 cores of ceilings.

## Decision

- **One self-pacing daemon thread per source** (docker / dds / hz / host),
  each sleeping `max(0, period - elapsed)` — single-flight by construction; a
  slow pass delays only its own next pass. Every pass is wrapped
  (`run_source_once`), and each source exports `scout_poll_errors_total`,
  `scout_poll_duration_seconds`, and
  `scout_poll_last_success_timestamp_seconds` (`time() - it` = sample age),
  so a dead source is visible instead of silently stale.
- **DDS probe: one exec, not nine, into foxglove_bridge, not robot.**
  `build_dds_script` sources the ROS environment once and probes every topic
  with a per-topic coreutils `timeout`; the exec itself is wrapped in an
  overall `timeout`. The target is `ROS_EXEC_SERVICE` (default
  `foxglove_bridge`: same image, same overlay volumes, same loopback DDS
  domain, but its 0.3-cpu cgroup can only degrade the Foxglove UI, never the
  control path), falling back to `robot` when absent. Probe cadence drops to
  `DDS_POLL_S=60` — pub/sub wiring changes on launch/restart, not
  continuously. Failed/missing topic sections leave gauges untouched (no
  fake zeros) and flip `scout_dds_probe_ok`.
- **observability-mcp** uses the same exec target + `timeout` bound (exit
  124 → explicit ToolError) and a bounded docker client.
- **docker-py clients get `timeout=15/20`** so a wedged dockerd can't hang a
  source thread or tool call; per-container catches widened to include
  `requests.exceptions.RequestException` (docker-py raises those through).
- **Compose caps the observers**: observability_exporter 0.3,
  observability_mcp 0.3, dozzle_agent 0.2. An uncapped diagnostic service on
  a live robot is exactly the contention it exists to measure.
- **Throttling telemetry**: `poll_docker` now exports
  `scout_container_cpu_{periods,throttled_periods,throttled_seconds}_total`
  straight from the stats payload it already fetched —
  `rate(throttled_periods)/rate(periods)` on robot/nav2 during a stall is
  the direct test of the quota-starvation hypothesis.
- Pure helpers live in `docker/observability-exporter/poll_loop.py`,
  path-imported by `scout/test/test_observability_poll.py` (the
  elevator_config precedent), pinning the scheduling and parsing behavior.

## Consequences

- The first `ros2` CLI call starts a ros2cli daemon inside foxglove_bridge —
  small persistent RSS in its 0.3-cpu cgroup; acceptable.
- The observability profile stays opt-in (deploy leaves it down). The
  ROS-side stop-reason/freshness surface (ADR-0036) carries the incident
  timeline on a bare robot; these metrics extend it when the profile is up.
- Hardware verify on next deploy: `docker exec scout-foxglove_bridge-1 which
  timeout`, one manual probe pass, and `scout_poll_last_success_*` going
  stale (not frozen gauges) with rosbridge stopped.
