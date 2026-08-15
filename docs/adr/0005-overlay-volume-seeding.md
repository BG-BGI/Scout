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
