# ADR-0013: Conventions enforced by ruff + structural tests, gated by CI

Status: accepted · Date: 2026-08-15

## Context

The repo's conventions — pure `scout.core` (ADR-0012), `run_node` as the one
`main()`, `CmdVelSource` as the sole cmd_vel writer (ADR-0001),
`robot_profile.yaml` as the cross-surface SSOT, sensor QoS on sensor topics —
lived in prose and discipline. They drifted anyway: 7 of 13 nodes hand-rolled
`main()` (four with a shutdown bug), `core/battery.py` sat orphaned under five
passing tests while `battery_monitor` kept a verbatim copy, `publish_hz` forked
three ways one commit after the profile was declared the SSOT, and
`patrol_capture` re-derived the quaternion `core.geometry` already had. The
only lint gate (`ament_flake8` behind an importorskip) never ran off-ROS, had
no CI behind it, and was red without anyone knowing. Six `# noqa: BLE001`
comments referenced a ruff rule no installed tool read.

## Decision

Two enforcement layers, both off-ROS, both blocking in CI
(`.github/workflows/ci.yml`: ruff + bare pytest, Python 3.10 = Humble parity):

1. **ruff** (pinned in `requirements-dev.txt`) for standard rules. Root
   `pyproject.toml` covers scout-skills/scripts (py312, double quotes);
   `scout/ruff.toml` extends it for the ROS package (py310, single quotes).
   Selected: E, F, W, I, B, BLE, Q, RUF100. No pyupgrade — the in-package
   %-format style is deliberate. `scout/test/test_lint.py` shells out to ruff
   (skips when absent, e.g. `colcon test` on the Pi) so pytest-green implies
   lint-green wherever ruff exists. `ament_flake8` and `test_flake8.py` are
   deleted.

2. **Structural tests** (the `test_core_purity.py` pattern: `ast.parse` the
   source, never import ROS) for repo-specific rules, in
   `scout/test/test_conventions.py`, `test_profile_constants.py`,
   `test_status.py`:

   | ID | Rule |
   |----|------|
   | SC1 | console-script `main()` is `def main(args=None)` delegating to `run_node` |
   | SC2 | `sensor_msgs` subscriptions pass sensor QoS, never a bare depth |
   | SC3 | no raw `lookup_transform` outside `node_util` |
   | SC4 | `Twist` publishers only in `cmd_vel_source` / `estop` (ADR-0001) |
   | SC5 | no hand-rolled planar-quaternion math (use `core.geometry` / the vendored skills copy) |
   | SC6 | the `/ros_ws/src/scout` bind path only in `robot_profile.py` (`resolve_config*`) |
   | SC7 | every `core/` module is imported by a node and has a 1:1 test file |
   | SC8 | profile-owned values are never bare literals on any surface (table derived from the yaml) |
   | SC9 | the `\|`-status wire formats are frozen as exact strings (`core.status`) |
   | SC10 | the two deliberate copies stay in sync: `webui/robot_profile.yaml` byte-identical, skills `geometry.py` function-source-identical |

**Waivers.** ruff: `# noqa: CODE — reason`, kept honest by RUF100. Structural
rules: per-file `ALLOW = {path: reason}` dicts in the test (empty-reason
entries fail), plus a single inline escape for SC8 only —
`profile-exempt: <reason>` (`#`, `//`, or `<!-- -->`). Everything is an error;
a rule not worth blocking on is not a rule.

**Deliberate scope edges.**
- `docker/scout-skills` cannot import `scout.core` (separate container), so the
  geometry helpers are *vendored* there and SC10 freezes the copy instead of
  banning it.
- `scout.core` cannot read yaml (ADR-0012 purity), so profile values are
  *injected* by ROS callers (`plan_coverage(occupied=...)`); the pure default
  carries a `profile-exempt` marker.
- SC8 bans only distinctive values. `angular_floor` (0.35) was tried and
  dropped — 0.35 is a common tuning constant (YOLO confidence, seek speeds)
  and the ban drowned in coincidences; its one real fork was fixed instead.
- webui JS gets no eslint/node toolchain: one 782-line file, and its only
  convention worth gating (SC8/SC10) is enforced from pytest. Revisit if webui
  grows past one file.
- Launch files import `scout.robot_profile` for `resolve_config*`; fine in
  every real launch context because the package is always installed/overlaid.

## Consequences

- `pytest` + `ruff check .` (or just CI) is the definition of done; an agent
  or operator gets the fix instruction in the failure message itself.
- The four-shape `main()` drift, the orphaned-core-module class, the QoS
  silent-failure class, the profile forks, and wire-format drift are now test
  failures, not archaeology.
- New conventions follow the loop: write the rule as a failing test, remediate,
  land both together, record the why here.
- Cost: `requirements-dev.txt` on any machine running the suite with lint
  (plain pytest still works without ruff — one test skips).
