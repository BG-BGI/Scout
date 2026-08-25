# ADR-0005: One overlay install tree in a named volume; deploy = git pull

Status: accepted · Date: 2026-08-15 (volume-stamp guard added)

## Context

Image-baked forks (roboclaw_driver, realsense, rplidar, explore_lite) and the
locally-built Scout packages all install into `$OVERLAY=/opt/overlay`. Compose
mounts the named volume `ros_overlay_install` over `/opt/overlay/install` so
`build_package` survives `run --rm`. The volume seeds from the image **once** —
so a rebuilt image with an unwiped volume silently runs the *old* forks and
looks completely healthy. That was the highest-severity deploy trap.

## Decision

Deploy is git only: commit → push → `git pull` on the Pi → rebuild affected
compose services (never edit tracked files on the Pi). Scout packages build via
`--symlink-install`, so a pull + rebuild of `build_package` is enough for
Scout-only changes; bumping an image-baked fork requires `docker compose down -v`
to re-seed the volume. The Dockerfile stamps `$OVERLAY/.image_build_id` and the
entrypoint hard-fails if the volume's copy differs (planned in the deploy-
hardening pass), turning the silent-shadow trap into a loud one-time migration.

## Consequences

- No second install path can shadow the overlay.
- A cache-busting image rebuild costs one `down -v` + reseed.

## Addendum (2026-08-15): fork pins, layer order, mount trim

- **All three source forks are pinned to commit SHAs** (clone-then-
  `checkout --detach`; `--depth 1` cannot fetch a bare SHA): roboclaw_driver
  `cc4d0e7`, rplidar_ros (ros2 branch) `24cc9b6`, m-explore-ros2 `326cf8a` —
  the tips at pin time; the previously deployed commits are unrecoverable
  (clones were `rm -rf`'d), so the first rebuild may bump behavior.
- **roboclaw_driver builds from the org fork `BG-BGI/roboclaw_driver`**
  (forked 2026-08-15 at `cc4d0e7`, same SHA as the pin) — operator rule: no
  build-time dependency on personal repos. Bump flow: push/merge to the
  BG-BGI fork, re-pin the SHA here + in the Dockerfile.
- **Layer-order rule is now structural:** roboclaw_driver (the most
  bump-prone fork) sits BELOW librealsense and the apt layers, so a pin bump
  rebuilds only roboclaw + explore + the stamp, never the 13-min
  librealsense layer.
- **Mount trim:** `foxglove_bridge` keeps host net/ipc + discovery env + the
  ROS entrypoint but loses the repo bind mount and overlay volume (apt
  package; without the volume the stamp guard self-compares the image);
  `webui` is a bare http server — no entrypoint, no DDS env, only
  `./webui:ro`. That required `webui/robot_profile.yaml` to become a **real
  file** (SC10 keeps it byte-identical to the SSOT) instead of a symlink
  into `scout/config`, which the trimmed mount could no longer resolve.
- `rosbridge` deliberately kept on the full anchor (flagged optional).
