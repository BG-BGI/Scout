# ADR-0012: Pure `scout.core` + bare-pytest testing

Status: accepted · Date: 2026-08-15

## Context

Every algorithm was a method on a `Node` subclass, needing `rclpy.init()` to
instantiate, so nothing was testable without standing up ROS. The only tests
were the ament template linters, and since no scout file had a copyright header
they had never actually passed.

## Decision

Domain logic moves to `scout/scout/core/` — stdlib + numpy only, no ROS
(enforced by `test_core_purity`). Tests run under **bare pytest** on the dev Mac
/ CI (no ROS) and under `colcon test` on the Pi. `node_util.run_node` unifies
the twelve slightly-different `main()`s (and adds the `rclpy.shutdown()`
link_watchdog skipped). The never-passing `test_copyright`/`test_pep257` stubs
are deleted; linting is ruff, run off-ROS (ADR-0013 — this superseded the
original `test_flake8` importorskip arrangement).

The stringly `|`-joined status topics are kept (not converted to `.msg`): the
churn touches consumers on both sides of the rosbridge boundary for values that
change once a year. Instead the exact wire formats are frozen by tests
(`scout/test/test_status.py`, over the `scout.core.status` formatters — the
claim predated the test; ADR-0013 closed the gap).
`generate_parameter_library` was evaluated and rejected (codegen churn, no
payoff for one operator).

## Consequences

- Real algorithms (geometry, battery curve, coverage, under-lidar grid, scan)
  are testable through a plain function interface; the interface is the test
  surface.
- `scout.core` may never import ROS — the purity test fails the build otherwise.
