# ADR-0030: Schindler RBL elevator rides via a standalone SDK + scout-skills tools

Status: accepted · Date: 2026-09-01

## Context

Scout needs to call and ride Schindler PORT elevators (Robot Building
Logistics API). The API has two flavors — on-site PORT Gateway (mTLS client
cert) and cloud (Device Identity → JWE) — with near-identical endpoints, and a
strict call lifecycle: POST /calls → wait `Enter` (door open) → confirm →
ride → wait `Exit` → confirm, where the door-open windows are elevator-timed
and PORT aborts the call if they lapse. Verified against the shared sandbox
(sandbox.schindler.com, equipment `EQ-1-1-<car>`): mTLS needs the FULL-CHAIN
PEM (leaf-only from `-clcerts` is rejected by nginx as "The SSL certificate
error"), `identity` is REQUIRED on POST /calls, no bearer is enforced, and
"Elevator is busy" rejections are constant background sim traffic. And the
classic trap, confirmed live: `floorNumber` is the 1-based index among SERVED
stops, not the label — sandbox floorNumber 2 is label "0" (Lobby).

## Decision

- **HTTP client + ride state machine live in a standalone SDK,
  `BG-BGI/schindler-rbl` (private)** — reusable by other robots, testable with
  zero robot (`rbl` CLI drives the sandbox end to end). Robot motion plugs in
  through a 3-method `RobotAdapter` (`nav_to_door`, `board`, `exit_move`).
  scout-skills installs it SHA-pinned via a BuildKit secret `gh_token`
  (compose passes `GH_TOKEN`; Actions passes org PAT `SCHINDLER_RBL_TOKEN` —
  `GITHUB_TOKEN` cannot read sibling private repos).
- **Scout surface = MCP tools in scout-skills** (`elevator_floors/call/status/
  confirm/cancel/ride`), no ROS graph changes, no companion involvement (the
  zenoh allowlist carries no companion→Pi control, ADR-0022). `elevator_ride`
  runs the whole sequence as a background task (explore/patrol precedent)
  because LLM-driven turn-by-turn confirmation would routinely eat
  `Aborted (Timeout occurred)` in the doorway; the primitives remain for bench
  work and recovery.
- **Config split: env = identity + gateway, site = building topology.**
  `SCHINDLER_*` env vars + gitignored `secrets/schindler/` (`:ro` mount) are
  deployment facts; equipment numbers, floorNumber→door-waypoint mapping,
  sides, and board/exit distances live in hand-authored
  `sites/<name>/elevator.json` (opened per call — follows site switches live,
  ADR-0023; schema validated in `docker/scout-skills/elevator_config.py`,
  duplicated not imported, ADR-0011).
- **Safety shape**: nav to the door BEFORE posting the call (don't burn the
  PORT hold-timeout); board/exit are dead-reckoned `run_move` (the car
  interior is unmapped — Nav2 has no business there); a boarding shortfall
  (<80% of depth) fails WITHOUT confirming Enter and WITHOUT deleting — the
  open call holds the door while an operator intervenes; cancel/timeout
  DELETEs the call, and a robot inside the car must POST a new call to finish.

## Consequences

- Sandbox → real PORT Gateway is a config swap (`.env` + two PEMs +
  `SCHINDLER_TLS_VERIFY` pointing at the gateway's private CA). Cloud flavor,
  WSS push, and access-management integration are future SDK modules.
- v1 deliberately excludes multi-floor maps: after exiting on another floor
  the robot is OFF-MAP and localization is invalid until it returns. ADR-0029
  multi-map sites is the machinery to lift this — wiring `elevator_ride` to
  `switch_map` on arrival is the obvious phase 2.
- Every scout-skills image build now needs a GitHub token with read on
  BG-BGI/schindler-rbl (local: `GH_TOKEN=$(gh auth token) docker compose
  build scout_skills`).
- The elevator tools are inert (raise "not configured") until
  `SCHINDLER_BASE_URL` is set, so non-elevator deployments are unaffected.
