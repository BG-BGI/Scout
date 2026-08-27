# ADR-0026: Flipper Zero NFC — mirror of the RFID scan/store pipeline, shared companion recorder

Status: accepted · Date: 2026-08-25

## Context

ADR-0025 gave Scout 125 kHz RFID via the Flipper's USB CLI: `flipper_node`
loops `rfid read`, pose-stamps each card onto latched `/rfid/reads`, and the
companion `rfid_recorder` is the primary per-site sqlite DB, republishing the
deduped `/rfid/registry` back over the zenoh bridge to the webui/MCP. The
operator wants the same for the Flipper's 13.56 MHz **NFC** radio
(MIFARE/NTAG/ISO14443…), stored the same way — UID + tech row, pose-stamped,
manually gated in the webui. Two facts make it not a pure copy-paste:

- The Flipper's `nfc` command opens a **sub-shell** (unlike the flat
  `rfid read`): `nfc` → `scanner` (identify a presented tag) → `exit`. There is
  no one-line `nfc scanner`.
- There is **one serial line and one CLI**, so RFID and NFC scanning cannot run
  at once.

## Decision

- **One node, one loop, a mode.** `flipper_node` gains `/flipper/nfc_enable`
  (`std_srvs/SetBool`, webui NFC panel) alongside `/flipper/rfid_enable`, a
  shared `_on_enable(mode, …)` handler, and a `_scan_mode` the poll loop
  branches on. The two gates are **mutually exclusive**: enabling one while the
  other scans is rejected ("disable it first"), the same shape as `/flipper/cli`
  refusing while scanning. NFC scanning enters the `nfc` sub-shell once
  (`_in_nfc_shell`), runs `scanner` each cycle, and `exit`s back to the
  top-level `>:` on disable/fault/shutdown. `FlipperCli.open()` recovery now
  sends `Ctrl+C` + `exit` so a crash left mid-`scanner` lands at the top level.
- **Same wire, same QoS.** `/nfc/reads` (latched depth-50) and `/nfc/registry`
  (latched) join both zenoh allowlists as read-only telemetry, exactly like the
  RFID pair. `core.status.format_nfc_read` mirrors `format_rfid_read` with the
  tag **UID in the `data_hex` field**; `/flipper/status` gains `nfc_enabled`
  beside `rfid_enabled` (SC9-frozen). The scanner-output parser lives in the new
  firmware-coupled `core/nfc.py`, fixture-driven like `core/rfid.py`.
- **One shared companion recorder.** `rfid/recorder.py` is parametrized
  (`db_path`, `reads_topic`, `registry_topic`; defaults = the RFID values) and
  run twice — `rfid_recorder` unchanged, `nfc_recorder` pointing at
  `nfc.db`/`/nfc/reads`/`/nfc/registry`, node names remapped so they don't
  collide on the DDS graph. Schema, dedup and QoS are literally identical, so
  the SC-frozen QoS match covers both.
- **MCP:** `wait_nfc_read` and `list_nfc_tags` mirror the RFID tools (gate on
  `nfc_enabled`; tools never enable scanning).

## Consequences

NFC composes into the same drive-to-tag workflow (`go_to` → present tag →
`wait_nfc_read`) and degrades identically to RFID (no Flipper = idle badge; no
companion = up-to-50 latched replay; no map = `pose: null`). The mutual
exclusion is by design — one radio at a time. Costs carried over from ADR-0025:
the enable gate makes autonomous scanning impossible (the point), and the CLI
output format is **firmware-dependent**, so `core/nfc.py`'s parser and the
sub-shell dance must be confirmed on the attached Flipper before they are
trusted.

## Amendment (2026-08-27): `scanner` yields no UID — read via `dump` → file

Checking the read mechanism against firmware source
(`applications/main/nfc/cli/…`, dev branch) before the bench run overturned the
original `scanner`-based plan:

- **`nfc` → `scanner` prints only protocol NAMES**, comma-separated
  (`Protocols detected: Mifare Classic, Iso14443-3a`), and **no UID**. No CLI
  verb prints a tag UID to stdout (`dump` writes a file; `mfu info` covers only
  Mifare Ultralight). So `scanner` cannot produce the per-tag identifier the
  RFID mirror stores — two cards of one type would be indistinguishable.
- **New read path (operator-chosen):** in the `nfc` sub-shell, `dump -f <path>`
  auto-detects the protocol, reads the card, and writes a Flipper `.nfc` file
  (printing `Dump saved to '<path>'`, or a red `Error: <reason>` when no card is
  present within the ~5 s timeout). `storage` is unavailable inside the
  sub-shell, so on a save the node `exit`s to the top level, `storage read
  <path>` streams the file back, and `core/nfc.py` `parse_nfc_file` reads its
  authoritative `Device type:` + `UID:` lines; the temp file is then
  `storage remove`d. `parse_dump_output` drives the phase machine
  (`NFC_DUMP` → `NFC_READFILE`) in `flipper_node`.
- **Unchanged:** the `/nfc/reads` wire format, QoS, companion recorder, MCP
  tools, and mutual exclusion. Only the read mechanism and `core/nfc.py`
  changed; `protocol` now carries the `.nfc` Device type verbatim (e.g.
  `Mifare Classic`, `NTAG215`), not a whitelisted tech string.
- **Still bench-pending:** the format strings are from firmware source
  (authoritative) but the end-to-end shell dance is not yet run on this unit.
  The temporary raw-buf INFO capture in `flipper_node._tick_scanning` stays
  until a real card confirms it.
