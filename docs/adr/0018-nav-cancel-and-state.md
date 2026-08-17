# ADR-0018: Dispatcher-aware nav cancel + consolidated /nav_state

Status: accepted · Date: 2026-08-17

## Context

Three documented sharp edges around Nav2 goals: a goal survives its client
dying (the latched-goal runaway), "goal failed" does not stop already-dispatched
behaviors, and there was no operator cancel short of the skills `nav_cancel`
tool or a compose restart — and no surface showed nav progress at all.

A goal-only cancel is also insufficient on this stack: the goal *dispatchers*
re-issue motion right through it — patrol_capture advances to its next waypoint
on goal end, explore re-dispatches frontier goals continuously, link_watchdog
mirrors `/route_poses` for re-dispatch. The webui STOP button had already
learned this lesson (it stops trick/follow/patrol/explore before the zero
burst); the ROS side had no equivalent.

## Decision

One node, `nav_manager`:

- **`/nav/cancel` (Trigger) is dispatcher-aware, dispatchers first:**
  `/patrol/stop` + `/explore/resume false`, then zeroed-uuid CancelGoal on BOTH
  `navigate_to_pose` and `navigate_through_poses`. All async — a sync client
  call inside the service callback deadlocks the executor silently (SC11).
  The cancel plumbing is shared as `node_util.cancel_nav_goals`, adopted by
  link_watchdog, so a third copy never appears. This stops NAV and leaves the
  robot drivable — distinct from STOP (one-shot halt of everything) and E-STOP
  (latching mux lock + brake).
- **`/nav_state` (latched String) consolidates both actions' `_action/status`
  + `_action/feedback`** into the SC9 `|`-grammar owned by `core.status`:
  `'idle' | '<status_name>|<dist 2dp or empty>|<recoveries>'`, names from
  robot_profile `goal_status_names`. **Deliberately NOT JSON** (2026-08-17
  grill): the house wire dialect is `|`-split strings frozen by test_status.py
  (ADR-0012/0013); a JSON topic would be a second dialect on the same
  rosbridge boundary. The distance field is empty until the first feedback —
  never a fake 0.00.
- webui: the map panel's Cancel Goal button now calls `/nav/cancel` (it
  previously zero-cancelled `navigate_to_pose` only — through-poses routes
  kept driving), and the nav readout subscribes `/nav_state` instead of one
  action's status topic.

## Consequences

- One cancel that actually parks the robot, from any surface; `/nav_state`
  gives patrols and routes the same progress readout as single goals.
- tilt_monitor and nav_manager both publish `/explore/resume` — fine for a
  Bool pause topic; with no explore node up the publish is dropped.
- The skills `nav_cancel`/`stop_all` tools keep their own paths (rosbridge
  cannot see service readiness the same way); folding them onto `/nav/cancel`
  is a later cleanup once it Pi-verifies.
- Verify (robot): dispatch a goal, CANCEL → motion stops and STAYS stopped
  mid-patrol (the dispatcher test), still drivable, `/nav_state` reads
  `canceled|…`; STOP/E-STOP unchanged. Watch from host `docker compose logs`,
  never a throwaway container during a live goal.
