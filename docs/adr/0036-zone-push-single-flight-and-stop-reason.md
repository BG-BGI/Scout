# ADR-0036: Zone-push single-flight and the stop-reason timeline

Status: accepted · Date: 2026-09-24

## Context

Two findings from the same external review (handoff 2026-09-24). First,
collision_polygon_manager's push discipline: it compared the desired zone
triple against `_pushed` (set only in the async done-callback, only on
success) in every `/cmd_vel_auto` callback, so during every ack window each
20–50 Hz callback issued another `call_async` — no timeout, no coalescing,
futures never removed. A slow or restarting collision_monitor amplified
pending work without bound, and after a CM restart the node believed its old
state was still applied while the CM actually held its YAML defaults.
Second, no single surface said WHY the robot stopped: distinguishing
nav-idle, a polygon stop, a dead CM, a mux override, or bridge/CPU
starvation meant correlating four bag topics by hand, and a compute-starved
freshness stop reads exactly like an obstacle stop.

## Decision

- **`scout/core/zonepush.py`** (pure, SC7): a single-flight, coalescing,
  bounded push state machine. At most one set_parameters request
  outstanding; the latest desired state overwrites (never queues); an
  unacked request expires after `push_timeout_s` (default 2 s) and its
  pending rclpy future is dropped via `remove_pending_request`; stale/late
  acks are ignored by seq. **Fail-safe bias:** `acked` becomes known ONLY on
  an explicit successful ack — timeout, failure/partial reply, and any
  service-readiness transition (a CM that bounced holds YAML defaults) all
  reset it to unknown, forcing a re-push and forbidding a synced report.
  The node's hot path is now two Latch updates plus a tuple compare.
- **`/collision_monitor/zone_sync`** (latched Bool): True only while the CM
  has explicitly acked the current desired zone state. The zones being *in
  doubt* is now observable instead of assumed.
- **`scout/core/cmdflow.py`** (pure, SC7) + a `cmd_stream` row in
  health_monitor: one TopicFlow per hop of the autonomy chain
  (`/cmd_vel_auto` → `/cmd_vel_safe` → `/cmd_vel_out`, names from
  robot_profile) tracking arrival age, nonzero age, and worst inter-arrival
  gap over a window. `stop_reason()` classifies precedence-first:
  `auto_idle` / `cm_dead` / `bypass` / `cm_stop:<zone>` / `zone_unsynced` /
  `mux_override` / `moving`. WARN only on `cm_dead` and `zone_unsynced` —
  a polygon stop is the system working. The label is mirrored on latched
  **`/stop_reason`** (published on change only), and the chain topics + CM
  wires + `/stop_reason` joined `record_topics`, so one bag replay carries
  the incident timeline (ADR-0035's throttling gauges extend it when the
  observability profile is up).
- Observation only: no change to `source_timeout`, `stop_pub_timeout`, the
  deadman, or mux timeouts. Plain Twists carry no source stamp, so these
  are arrival measurements — per-hop liveness, not end-to-end latency.

## Consequences

- Zone transitions can apply up to `push_timeout_s` + one 1 Hz tick late
  when the CM is unresponsive — during which the CM's last-applied (or
  default, front-guarded) polygons remain active: conservative.
- `test_zonepush.py` pins coalescing under rapid flips, delayed/late/stale
  acks, timeout-expire-retry, rejected replies, restart mid-flight, and the
  never-synced-without-ack bias; `test_cmdflow.py` pins gap tracking and
  the full precedence table.
- Hardware verify: restart the CM stack while zones churn — expect
  `zone_sync` to dip and recover with a re-push, and no cmd_vel gap >200 ms
  (the deadman window) during zone churn.
