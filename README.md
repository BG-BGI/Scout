# Scout Robot

<p align="center">
  <img src="scout/scout.png" alt="Scout robot" width="300">
</p>

## Running
```bash
docker compose build
```
```bash
docker compose --profile build run --rm build_package
```
```bash
docker compose up
```
(Starts every non-profiled service: `robot`, `slam`, `nav2`, `rosbridge`, `webui`,
`foxglove_bridge`, `ros_mcp`, `scout_skills` — `build_package` and `explore` are
profile-gated. Joystick on by default; disable with
`docker compose run --rm robot ros2 launch scout robot.launch.py enable_joystick:=false`.)

After switching off the old `ros_ws_install` volume, once:
```bash
docker compose down -v
```
then `docker compose build` and `build_package` so the new `ros_overlay_install` volume seeds from the image and Scout lands in `$OVERLAY`.

```bash
docker compose exec slam /ros_entrypoint.sh ros2 service call   /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph   "{filename: /ros_ws/src/maps/office}"
```

## Web UI (http://scout.local)

Touch/gamepad driving, speed + light settings, battery, and trick macros. Served by
the `webui` compose service (port 80) with `rosbridge` (ws://:9090) as the transport.

One-time host setup:

```bash
sudo hostnamectl set-hostname scout      # mDNS name -> http://scout.local
sudo apt install -y avahi-daemon         # usually already on Pi OS
systemctl is-enabled docker              # should be "enabled" (default)
```

Boot autostart — `robot`, `slam`, `nav2`, `rosbridge`, `webui`, `foxglove_bridge`,
`ros_mcp`, and `scout_skills` carry `restart: unless-stopped`, so after this one-time
bring-up they return on every power-on with no SSH:

```bash
docker compose up -d
```

(`restart` policies only apply to containers created with `up -d`, never `compose run`.
`slam`/`nav2` became always-on with patrol_capture (2026-08-12): marking waypoints needs
the map frame and patrols need the planner, and a fresh nav2 holds no goal so autostart
adds no unattended motion.)

Notes:
- `http://scout.local` needs mDNS; some Android builds don't resolve it — use
  `http://<pi-ip>` as the fallback.
- Xbox controller: pair it to the **phone/laptop** and the browser's Gamepad API drives
  through the page (RT forward, LT reverse, left stick turns; live values shown under
  the pad). The robot-side pad on `/dev/input/js0` still works independently.
- Stopping: releasing all inputs sends a short burst of zero Twists, then the page goes
  silent and the RoboClaw's 200 ms deadman is the backstop. Closing the tab mid-drive
  coasts the robot within ~200 ms. The STOP button also cancels a running trick; a trick
  survives a rosbridge/web outage (it runs robot-side) — `docker compose restart robot`
  is the hard clear.
- Lights: the UI talks to `/set_user_led` (led_status node). Battery warnings
  (orange ≤17.5 V, red ≤16.5 V) and trick/connection flashes override the user setting.

### Explore (frontier mapping)

Profile-gated; does not start with plain `docker compose up`. After `robot` + `slam` + `nav2` look healthy:

```bash
docker compose --profile explore up -d explore
```

Stop (stopping explore alone leaves any in-flight Nav2 goal running):

```bash
docker compose --profile explore stop explore
docker compose exec -T robot ros2 lifecycle set /bt_navigator deactivate
```

Resume navigation later with `activate` on the same node, or `docker compose restart nav2` for a full reset.
