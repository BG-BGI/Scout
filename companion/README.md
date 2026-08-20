# Companion stack — live RTAB-Map 3D capture (ADR-0020/0021)

Runs on a **native Linux host on the robot's LAN** — primary: the Debian box
(see `host-setup.md`; Docker Desktop on macOS cannot carry a DDS data plane,
do not retry it). Joins the robot's DDS graph
as domain 17 via the Pi's LAN-bound discovery server; consumes compressed
color + compressedDepth + `/odom` + `/tf` + `/scan` directly.

**Contract: the Pi must keep working with this stack absent** (spec §0.7).
Nothing here publishes to any `/cmd_vel_*` topic, and nothing may — the
twist_mux allowlist (ADR-0001) + SC4 are the enforcement; audit on every new
service added here.

## Bring-up

```bash
cp .env.example .env        # set PI_IP (scout.local's address)
docker compose build        # once; plain apt image, fast
docker compose up -d
```

View the live map: Foxglove (Mac-native) → `ws://<vm-ip>:8766` →
`/rtabmap/cloud_map` (+ `/rtabmap/mapGraph`). Deliberately a second bridge so
the cloud never hauls back through the Pi's `:8765`.

Sanity checks (inside the VM):
```bash
docker compose exec rtabmap bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic hz /scan'   # expect ~11.7 Hz
# topic list needs SUPER_CLIENT — see scout/config/super_client.xml recipe
```

## Teardown / absence

`docker compose down` (or Mac asleep, VM stopped, WiFi roamed) must cost the
robot nothing: rtabmap simply stops mapping and resumes on rediscovery. The
map DB persists in the `rtabmap_db` volume; wipe with
`docker compose down -v` to start a fresh session.

## Replay gate (offline, no network dependency)

Record a teleop bag on the Pi (ADR-0017 tooling) with:
`/camera/camera/color/image_raw/compressed`,
`/camera/camera/aligned_depth_to_color/image_raw/compressedDepth`,
`/camera/camera/color/camera_info`, `/odom`, `/tf`, `/tf_static`, `/scan`.
Copy it into the VM and:

```bash
docker compose run --rm -v /path/to/bag:/bag rtabmap bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 bag play /bag --clock' &
docker compose up rtabmap   # with use_sim_time:=true for the replay
```

Pass: nonzero-node database + coherent cloud in Foxglove.
