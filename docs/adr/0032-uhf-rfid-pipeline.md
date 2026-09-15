# ADR-0032: UHF RFID (M7E Hecto) — batched pose-stamped reads, companion centroid localization; replaces Flipper LF RFID

Status: accepted · Date: 2026-09-15
(0031 was the reverted BIM/OpenSpace integration; the number stays retired.)

## Context

The operator wants to read passive RFID tags **at distance while driving** and
estimate each tag's position in the map frame — asset location on jobsites.
The Flipper's 125 kHz LF radio (ADR-0025) reads at contact range only; a
SparkFun M7E Hecto (JADAK/ThingMagic module, EPC Gen2, 902–928 MHz, up to
150 tags/s, per-read RSSI *and* carrier phase) with an L-com HG903RD-RSP 3 dBi
omni antenna replaces it. The Flipper keeps NFC only (ADR-0026).

Constraints that shaped the design:

- **The antenna is omnidirectional and linearly polarized** — a single read
  carries no bearing, so tag position must come from aggregating many reads
  along the robot's path. Cross-polarized tags read up to ~20 dB worse.
- **The board rides a Pi 5 USB port** shared with the D455 and RPLIDAR; full
  27 dBm draws >720 mA @ 5 V and risks browning out the other sensors.
- **The CH340C enumerates as `/dev/ttyUSB*`**, colliding with the RPLIDAR's
  CP2102 by probe order, and `/dev/serial/by-id/` does not exist in-container.
- The module streams up to 150 tag records/s; the companion recorder pattern
  (ADR-0025) opens sqlite per operation.

## Decision

- **Pure-python protocol port, not mercuryapi.** `scout/scout/core/uhf.py`
  ports the ~15 ThingMagic Mercury opcodes (frames, CRC, continuous-read blob,
  tag-record offsets) from the SparkFun Arduino library onto bytes-in/bytes-out
  pure functions (ADR-0012), consumed by both `uhf_node` and the bench
  instrument `scripts/uhf_bench.py` — the bench validates the shipping framer.
  The JADAK C SDK would add an arm64 build for a protocol this thin.
- **Pipeline clones ADR-0025's shape**: `uhf_node` (robot service, tier-2
  respawn) owns the serial port; `/uhf/enable` is the same manual gate (OFF at
  boot, never persisted, dropped on serial fault); reads cross the zenoh
  bridge to `companion/uhf/recorder.py` → `/sites/active/uhf.db`; the registry
  rides `/uhf/registry` back (no services cross the bridge, ADR-0022).
- **Reads are BATCHED, the one structural deviation.** The node accumulates
  parsed records per `batch_period_s` (0.1 s) window and publishes ONE JSON
  message with ONE map pose — ≤10 msg/s at any tag rate, one sqlite
  transaction per batch, ~5 cm pose smear at 0.5 m/s. **No dedup on the Pi**:
  every read is geometry for the solver (the Flipper's 10 s
  `duplicate_suppress_s` would destroy exactly the data this exists for).
- **Two-stage localization.** Stage 1 (now): the recorder's registry computes
  a per-EPC **RSSI-weighted centroid** (weight `10^(rssi/10)`, localized reads
  only) plus `spread_m` — expect 1–3 m. Stage 2 (later, offline against
  uhf.db): multi-frequency **synthetic aperture** over the per-read
  phase/freq/pose to target <1 m. Stage 1's schema therefore stores
  `rssi_dbm/freq_khz/phase/timestamp_ms` RAW per read (frozen by
  `test_companion.py::test_uhf_recorder_stores_stage2_columns`) — stage 2 is a
  solver change, not a re-survey.
- **Read power capped at 2000 cdBm (20.00 dBm)**, clamped in
  `core/uhf.cmd_set_read_power` so no caller can exceed the USB budget.
  Costs roughly half the read range vs 27 dBm (~1.5–2.5 m expected); the cap
  lifts only when the reader gets its own 5 V feed. Region NORTHAMERICA.
- **Own recorder script**, not a third instance of `rfid/recorder.py`: the
  shared script's value was an identical schema for two identical radios; UHF
  differs on batch unpacking, signal columns, and the centroid. The shared
  script serves NFC only once the LF pipeline is removed.
- **Port pinned by host udev rule** (operator-side). Until it lands, the
  node's version handshake refuses a mispinned device: a lidar never answers
  a Mercury frame, and no config write goes to an unidentified port.
- **Flipper LF RFID removal is a separate, later commit** — only after UHF is
  proven on-robot, so the robot never has neither pipeline. ADR-0025's RFID
  half is superseded then; old per-site `rfid.db` files stay in place.

## Consequences

- Driving past tags while localized populates `/uhf/registry` with map
  positions and confidence; MCP `list_uhf_tags` / `wait_uhf_read` and the
  webui UHF panel consume it. `wait_uhf_read` returns whole batches (many
  EPCs) — callers filter.
- Registry recompute is O(all localized reads) per batch; memoize per-EPC
  running sums if a site accumulates far beyond tens of thousands of reads.
- Phase calibration (wire units, cable offset, behavior across frequency
  hops) is deliberately unresolved — bench questions for stage 2.
- The `throttled` status flag (temp-throttle / high-return-loss keepalives)
  WARNs in /diagnostics while scanning; persistent high return loss after
  mounting means an antenna/cable problem.
