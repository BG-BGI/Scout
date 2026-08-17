# Offloading Scout compute from the Pi to cloud/serverless

Status: plan only, nothing implemented (2026-08-15).

## Context

The Pi 5 is genuinely saturated: `scout/config/slam.yaml` records load avg 18.9 with containers at ~380% of 400%, Ceres grabbing all 4 threads per solve, and map→odom stamps drifting −0.6 s. The older "~1 core" figure in CLAUDE.md predates the full stack.

Two hard constraints already in the repo decide the shape of any offload:

1. **DDS must stay off the wire.** `ROS_LOCALHOST_ONLY=1` + the loopback discovery server exist because DDS multicast blackholed the Pi's corp-WiFi link for ~10 min (docker-compose.yaml:5–8). "Run a ROS node in the cloud" cannot mean extending the DDS graph over WAN.
2. **The link dies, and the robot must stay safe when it does.** `scout/scout/link_watchdog.py` (born of the 2026-08-14 WiFi dead-zone runaway) cancels nav goals after 5 s of link loss. Anything moved off-Pi must fail exactly this gracefully: degraded capability, never degraded safety.

## What can never leave the Pi

The 200 ms RoboClaw deadman needs `cmd_vel` at 20–50 Hz with no gaps; WiFi RTT jitter + dropouts violate that immediately. The entire safety/control tier stays local: `roboclaw_driver`, `twist_mux`, `estop`, `velocity_smoother`, EKF, `gyro_calibrator`, `collision_monitor`, `tilt_monitor`, `link_watchdog`, DWB controller + local costmap (15 Hz reactive loop), all sensor drivers, `robot_state_publisher`. Non-negotiable.

## Serverless verdict

**Serverless fits only request-shaped work, not streams.** Functions are stateless with cold starts; a SLAM node or controller is a long-lived stateful process holding a TF tree. Serverless-viable: per-frame YOLO inference, per-bag map post-processing, map rendering. Everything streaming needs a persistent server (LAN Mac or a small cloud VM).

## Transport: zenoh-bridge-ros2dds

The one new infrastructure piece. `zenoh-bridge-ros2dds` joins the *local* DDS graph on the Pi (loopback, existing discovery server — DDS never touches wlan0, so the corp-WiFi constraint stays satisfied) and speaks zenoh over a single TCP/QUIC connection to a peer bridge on the remote box, with an explicit per-topic allowlist. The remote box runs vanilla ROS 2 Humble nodes against its own local DDS. rosbridge (:9090) already covers the tool-call tier; zenoh covers the topic-stream tier.

Bandwidth sanity: `/scan` ≈ 160 KB/s (1590 pts × 12.3 Hz), `/odom` + `/tf` trivial, compressed color ≈ 1–2 MB/s on demand. Depth pointcloud stays local — never ship it.

## Offload tiers, in order of return-on-effort

### Tier 0 — free load cuts before any cloud (~30 min)

- Flip `always_send_full_costmap` back to `False` in both costmaps (`scout/config/nav2.yaml`) — flagged in CLAUDE.md as temporary; full grids are DDS/CPU only.
- Relax `minimum_travel_distance/heading` 0.3 → 0.5 in `scout/config/slam.yaml` — docs/slam.md §4 calls this the cheapest real load cut (fewer Ceres solves).
- Apply the §9.5b sysctls from `docs/remaining-implementation-plan.md` (`ipfrag_time=3`, `rmem_max`) — makes the link itself sturdier for everything below.

### Tier 1 — YOLO to a GPU endpoint (the true serverless fit, ~half day)

`docker/scout-skills/detect.py` runs YOLO11n ONNX on Pi CPU at 0.5–1 s/frame. It is already HTTP-call-shaped (invoked per tool call, not in a loop).

- Add `DETECT_ENDPOINT` env var: when set, POST the JPEG to a serverless GPU endpoint (Modal / RunPod / any HTTPS inference URL), parse boxes from JSON; on timeout or non-200, **fall back to the existing local ONNX path** so the skill degrades, never breaks.
- Result: ~50 ms inference + ~100 ms network vs 500–1000 ms local; larger models become possible; zero Pi CPU during detection.
- Files: `docker/scout-skills/detect.py`, `docker/scout-skills/server.py` (env plumb), `docker-compose.yaml` (env passthrough).

### Tier 2 — batch offboard mapping (already designed in docs/slam.md, ~half day to script)

Implement exactly the shape docs/slam.md prescribes: bag `color`, aligned depth, `camera_info`, `/odom`, `/tf`, `/tf_static`, `/scan` during a patrol; transfer in retryable chunks (rsync — the Pi ethernet flaps under sustained load); run RTAB-Map on the Mac (or a cloud VM/serverless job) against the bag. 3D model per patrol, zero live Pi cost beyond bag I/O. This is the "heavy perception in the cloud" pattern generalized: record → ship → process → return artifact.

### Tier 3 — live SLAM off-Pi (the big live win, ~2 days, LAN first)

`slam_toolbox` is the worst live offender (Ceres, 4 threads). Its I/O is offload-friendly: consumes `/scan` + `odom→base_link` TF (light, up), produces `/map` + `map→odom` (slow-varying correction transform, so ~100–300 ms of added latency is tolerable, unlike control).

- Run zenoh bridge on Pi + remote box; allowlist `/scan`, `/tf`, `/tf_static`, `/odom` up and `/map`, `/tf` (map→odom only) down. Run `slam.launch.py` on the remote box unchanged.
- **Failure contract**: remote SLAM dies → `map→odom` goes stale → nav goals fail/cancel (link_watchdog + bt_navigator already handle this class); teleop, estop, collision monitor, local costmap all keep working because they don't depend on `map`. Verify this degradation explicitly before trusting it.
- Do it on the LAN Mac first. Promote to a cloud VM (WireGuard/Tailscale under the zenoh TCP session) only if the LAN version proves out — WAN adds jitter and a second WiFi-like failure domain for marginal gain.
- Follow-on once the bridge exists: global costmap + planner (1 Hz replans, latency-tolerant) move the same way; the nav2 `use_composition:=false` path already exists for splitting.

### Not worth offloading

Controller/local costmap (latency), EKF/gyro (safety chain), camera/lidar drivers (hardware), foxglove/rosbridge/webui/MCP servers (already light; the MCP servers can move off-Pi anytime by pointing `ROSBRIDGE_URL` elsewhere, but there is no CPU reason to).

## Recommended execution order

Tier 0 now, Tier 1 next (biggest capability-per-effort, real serverless), Tier 2 when a mission needs 3D capture, Tier 3 only if load still bites after 0+1. Measure with the existing gauges (`Control loop missed its desired rate of 15.0000Hz` count, `docker stats`, map→odom stamp lag) before and after each tier.

## Verification

- Tier 0: `docker stats` + control-loop-miss count over a fixed nav mission, before/after.
- Tier 1: same image through local ONNX vs endpoint — box parity, then latency numbers; unplug WAN mid-call → confirm local fallback fires.
- Tier 3: drive the standard room loop with SLAM remote — map→odom correction magnitude vs the 0.30 m / 2.1° baseline over ~17 m; then kill the remote node mid-goal and confirm the robot stops planning but stays teleop-able and estop-able.
