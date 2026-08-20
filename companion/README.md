# Companion stack — live RTAB-Map 3D capture over the zenoh bridge (ADR-0022)

Runs on a **native Linux host** — the Debian box (10.1.57.18), see
`host-setup.md`. Each machine keeps its own loopback DDS graph; the only
cross-machine traffic is `zenoh_bridge`'s **single TCP connection** to the
Pi's `:7447` (corp inter-VLAN filtering drops UDP, so shared-domain DDS is
out — measured 2026-08-20, ADR-0022).

**Contract: the Pi must keep working with this stack absent** (spec §0.7).
The Pi-side bridge accepts nothing inbound (`subscribers: []`), so nothing
here can reach any Pi topic, `/cmd_vel_*` included. Extending what crosses =
editing BOTH allowlists (`config/zenoh_bridge.json5` here, mirrored in
`scout/config/zenoh_bridge.json5`) — deliberate, never wildcarded.

## Bring-up

```bash
cp .env.example .env        # set PI_IP (10.1.80.31)
docker compose build        # once; plain apt image (or docker load from USB)
docker compose up -d
docker compose logs zenoh_bridge | tail   # expect a session with the Pi, no reconnect loop
```

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
