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
(`robot` + `slam` + `nav2` + `foxglove_bridge`. Joystick on by default; disable with
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

Boot autostart — `robot`, `rosbridge`, `webui`, and `foxglove_bridge` carry
`restart: unless-stopped`, so after this one-time bring-up they return on every
power-on with no SSH:

```bash
docker compose up -d robot rosbridge webui foxglove_bridge
```

(`restart` policies only apply to containers created with `up -d`, never `compose run`.
`slam`/`nav2` deliberately do not autostart — start them manually when mapping/navigating.)

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
