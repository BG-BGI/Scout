# ADR-0034: AprilTag detection moves to the companion

Status: accepted · Date: 2026-09-18

## Context

ADR-0033's stream-tax removal landed, yet the Pi still measured load ~14 with
the robot service alone (CPU 53–71% idle, 60–90k context switches/s) and load
46 with nav2+slam stacked on — enough scheduler thrash that the D455's libusb
control transfers time out (`Resource temporarily unavailable`), the camera
streams die, the EKF misses its 30 Hz rate, odom→base_link smears the local
costmap into phantom arcs (reported as "obstacle over-sensitivity"), and the
marginal 5 V rail sags into a Pi reset. Killing the four newest robot-service
nodes live halved the load; of those, uhf_node and flipper_node are bound to
USB hardware riding the robot, and cliff_detector stays by ADR-0033's rule
(safety never crosses the bridge). The remaining movable perception is the
AprilTag trio: `apriltag_color_throttle` + `apriltag_info_relay` +
`apriltag_node` — three processes/DDS participants whose only inputs (color
JPEG + camera_info) already cross the bridge.

## Decision

- `apriltag_ros` runs as a companion service on the bridged 5 fps JPEG color
  (`image_transport: compressed`; camera_info pairs by identical stamps since
  image_transport derives its topic from the image namespace — the throttle
  and info relay die with nothing replacing them). Config:
  `companion/config/apriltag.yaml`; `scout/config/apriltag.yaml` is deleted.
- `/detections` crosses companion→Pi (both allowlists) — tag_relocalizer and
  scout-skills consume it under the unchanged name.
- Tag TF frames cross on a DEDICATED `^/tf_tags$` (the node's `/tf` is
  remapped): reverse-bridging `/tf` itself would carry the companion
  rtabmap's map→odom into the Pi graph and fight amcl/slam. A Pi-side
  `topic_tools relay /tf_tags /tf` (`tag_tf_relay`) folds them in, so every
  existing lookup (`base_link → "36h11:0"`) is byte-identical.
- `tag_relocalizer` STAYS on the Pi: it reads Pi-local files every frame
  (tags.db, site.json, the maps dir) and is the sole `/initialpose` author —
  which SC12 deliberately keeps unbridgeable.

## Consequences

- Pi sheds three processes (~8–10% CPU, three DDS participants) and gains one
  trivial C++ relay. Detection cadence improves 2 Hz → 5 fps (companion CPU
  is not the constraint the Pi's was).
- **Companion down = no tag refresh AND no tag boot-relocalization.** The
  Pi-local detector was immune to link state; now a cold boot away from the
  dock with the companion offline stays mislocalized until a human
  /initialpose. Accepted: same availability class as detect_objects'
  companion path (ADR-0033), and the failure is visible (no /detections).
- The companion gains indirect localization influence (its tag frames feed
  tag_relocalizer's /initialpose math) — the same influence the same code had
  on the Pi, still bounded by the registry + max_tag_dist gates, and still no
  control surface: /initialpose itself never crosses the bridge.
- apriltag_ros stays in the scout image (Dockerfile) as a dormant fallback —
  relaunching it locally is a launch-file revert away.
- This does NOT fix the context-switch storm (60–90k cs/s is spread across
  the remaining ~37 nodes + DDS); it trims the node count on the way to the
  real fix, which remains composition/offload at larger scale.
