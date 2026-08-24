# ADR-0025: Flipper Zero RFID — webui-gated scan loop, reads over zenoh, companion sqlite as the primary DB

Status: accepted · Date: 2026-08-24

## Context

A Flipper Zero on the Pi's USB (CDC-ACM, `/dev/ttyACM0` — the by-id symlink
farm does not exist in-container) gives the robot a 125 kHz RFID reader via
the Flipper's text CLI (`rfid read` blocks alternating ASK/PSK until a card or
Ctrl+C). The operator wants each read stored with the robot's localized map
pose, the DB on the companion, and — explicitly — **reads only while manually
enabled in the webui**. The existing `tags.db` (AprilTags) is a last-write
registry on the Pi; RFID wants an append-only event log, and no ROS service
may cross the zenoh bridge (ADR-0022).

## Decision

- `flipper_node` (Pi, tier-2 respawn) owns the serial port, led_node-style:
  one timer is the sole reader/writer; `/flipper/rfid_enable`
  (`std_srvs/SetBool`, webui RFID panel) is the **manual gate — off at boot,
  never persisted, dropped on any serial fault**. While enabled it loops
  `rfid read`; each card is pose-stamped (`lookup_pose2('map','base_link')`,
  null when unlocalized) and published as JSON on `/rfid/reads` with a
  **latched depth-50** QoS (`LATCHED_HISTORY_QOS`) so a recorder outage can
  replay; `read_id` (uuid4) makes replay idempotent. `/flipper/cli`
  (`scout_interfaces/FlipperCli`) is the bounded generic passthrough for
  future ir/subghz/led use.
- `/rfid/reads` (Pi→companion) and `/rfid/registry` (companion→Pi) are added
  to both zenoh allowlists — read-only telemetry, same class as
  `/world/registry`; services/actions stay empty.
- Companion `rfid_recorder` (base image + bind-mounted script, the
  inspection_recorder pattern — NOT an extension of `detector`, whose
  registry deliberately resets) appends to sqlite at `/sites/active/rfid.db`
  (per-site: a pose only means anything on its map; open-per-op, tags.py
  style) via `INSERT OR IGNORE`, and republishes the deduped latched
  `/rfid/registry`, which is how the webui and MCP list tags with no HTTP
  surface.
- MCP: `wait_rfid_read` (waits for the next new read; **refuses when the gate
  is off** — tools never enable scanning) and `list_rfid_tags` (registry).

## Consequences

Drive-to-tag workflows compose from existing pieces (`go_to` → present card →
`wait_rfid_read`), and the whole feature degrades cleanly: no Flipper = idle
node + "no flipper" badge; no companion = reads still publish and latch (up
to 50 replay on reconnect — beyond that, reads taken during a long outage are
lost to the DB, accepted for v1); no map = `pose: null` rows. Costs: the
enable gate means fully autonomous scanning is impossible by design (the
point); zenoh's replay of a transient_local depth-50 window is unverified on
this bridge and must be bench-checked; the CLI success-line format is
firmware-dependent — the parser is fixture-driven (`test_rfid.py`), so a
firmware update means recapturing transcripts, not guessing.
