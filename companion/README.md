# Companion stack — live RTAB-Map 3D capture over the zenoh bridge (ADR-0022)

Runs on a **native Linux host** — the Debian box (10.1.57.18), see
`host-setup.md`. Each machine keeps its own loopback DDS graph; the only
cross-machine traffic is `zenoh_bridge`'s **single TCP connection** to the
Pi's `:7447` (corp inter-VLAN filtering drops UDP, so shared-domain DDS is
out — measured 2026-08-20, ADR-0022).

The box holds the **full Scout repo**; the companion services run from its
`companion/` subdir (`cd <repo>/companion`), which is where every command below
and in `host-setup.md` is run.

**Contract: the Pi must keep working with this stack absent** (spec §0.7).
The Pi-side bridge accepts only read-only world-model telemetry inbound
(`subscribers: ["^/world/objects$"]` — WORLDMODEL.md gate 2), so nothing here
can reach a control topic, `/cmd_vel_*` included. Extending what crosses =
editing BOTH allowlists (`config/zenoh_bridge.json5` here, mirrored in
`scout/config/zenoh_bridge.json5`) — deliberate, never wildcarded.

## Bring-up

```bash
cp .env.example .env        # set PI_IP (10.1.80.31)
./update.sh                 # git pull + compose pull (GHCR) + up -d
docker compose logs zenoh_bridge | tail
```

Images come from `ghcr.io/bg-bgi/scout-companion{,-detector}` (CI-built on
push, `.github/workflows/companion-images.yml`); one-time deploy-key +
`docker login ghcr.io` setup in `host-setup.md`. `docker compose build` is
for local dev only.

Bridge health, in order (ADR-0022 bring-up findings):
1. `New ROS 2 bridge detected: <zid>` — TCP session with the Pi is up.
2. `Route Subscriber ... created` lines — local DDS discovery works. Session
   up but **zero route lines = the cross-vendor localhost trap**: the bridge
   needs `ROS_LOCALHOST_ONLY=1` + `ROS_DISTRO=humble` + the `CYCLONEDDS_URI`
   peer-ping env (all three in docker-compose.yaml) or Fast DDS and Cyclone
   are mutually deaf on loopback with no error anywhere.
3. Data: `docker compose exec rtabmap bash -lc "source /opt/ros/humble/setup.bash && ros2 topic echo /scan sensor_msgs/msg/LaserScan --once"`.
   ⚠ `ros2 topic list` is NOT evidence of data — rtabmap's own subscriptions
   list the topics even when nothing flows.

View the live map: Foxglove → `ws://10.1.57.18:8766` → `/rtabmap/cloud_map`
(+ `/rtabmap/mapGraph`). Deliberately a local bridge so the cloud never hauls
back through the Pi's `:8765`.

Sanity (inside this box, topics arrive via the bridge):
```bash
docker compose exec rtabmap bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic hz /scan'   # expect ~11.7 Hz
```

## Teardown / absence

`docker compose down` (or the box going away entirely) must cost the robot
nothing: the Pi-side bridge idles and rtabmap resumes when the TCP session
re-establishes. Map DB persists in the `rtabmap_db` volume; fresh session =
`docker compose down -v`.

## Replay gate (offline, no network dependency)

Record a teleop bag on the Pi (ADR-0017 tooling) with:
`/camera/camera/color/image_raw/compressed`,
`/camera/camera/aligned_depth_to_color/image_raw/compressedDepth`,
`/camera/camera/color/camera_info`, `/odom`, `/tf`, `/tf_static`, `/scan`.
Copy it to this box and:

```bash
docker compose run --rm -v /path/to/bag:/bag rtabmap bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 bag play /bag --clock' &
docker compose up rtabmap   # with use_sim_time:=true for the replay
```

Pass: nonzero-node database + coherent cloud in Foxglove.
