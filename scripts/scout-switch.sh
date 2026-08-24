#!/bin/bash
# scout-switch <branch> — switch the Scout stack to another software version.
#
# Copy this OUTSIDE the repo on the Pi (e.g. ~/bin/scout-switch) so it exists no
# matter which branch is checked out:
#   install -D scripts/scout-switch.sh ~/bin/scout-switch
#
# What it does: fast-forward the branch from origin (BG-BGI/Scout), rebuild the
# image (Docker layer cache keeps already-built branches fast), rebuild the scout
# packages into the overlay volume, then `up -d` whichever always-on services the
# branch's compose file defines. ⚠ Restarts the drivetrain driver — never run
# while the robot is driving.
set -e
REPO="${SCOUT_REPO:-$HOME/Desktop/Scout}"
BRANCH="${1:?usage: scout-switch <branch>   (e.g. main, web-ui)}"

cd "$REPO"
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "working tree has local changes — commit or stash first:" >&2
  git status --short >&2
  exit 1
fi

git fetch origin
git checkout "$BRANCH"
# Fast-forward only: never invent merge commits on the robot.
git merge --ff-only "origin/$BRANCH" 2>/dev/null || true

# build: lives only on the build_package service now (profile-gated), so the
# image build needs --profile build; then build_package installs Scout into the
# overlay volume.
docker compose --profile build build
docker compose --profile build run --rm build_package

# Bring up only the always-on services this branch's compose defines.
# slam + nav2 are always-on since patrol_capture: marking waypoints needs the
# map frame and patrols need the planner, and a deploy must not strand them.
WANT="robot behaviors rosbridge webui foxglove_bridge fleet_status slam nav2 ros_mcp scout_skills"
HAVE=$(docker compose config --services)
UP=""
for s in $WANT; do echo "$HAVE" | grep -qx "$s" && UP="$UP $s"; done
# Location sites (ADR-0023): slam runs mode:=site and refuses to start
# without sites/active. One-time on a pre-sites data layout; idempotent.
if [ ! -e sites/active ] && [ -f scripts/migrate_sites.py ]; then
  echo "migrating maps/ + captures/ into sites/default (ADR-0023)..."
  python3 scripts/migrate_sites.py
fi

# Pi 5 has no RTC: on a cold boot the clock starts wrong and NTP corrects it
# with a step, not a slew. If slam/nav2 are already running when that step
# lands, every TF-timestamped topic (laser, camera_depth_optical_frame) looks
# like it jumped in time, and the pose graph/costmaps poison for the session
# (map stops updating, localization drifts wildly, goals just spin in place).
# Block here so sensor-driving containers never start before the step happens.
echo "waiting for NTP time sync..."
until [ "$(timedatectl show -p NTPSynchronized --value)" = "yes" ]; do sleep 1; done

# --remove-orphans clears containers for services the new branch doesn't define.
docker compose up -d --remove-orphans $UP

# Pre-create (NOT start) the profile-gated explore container so scout_skills'
# explore_start can lazily start it via fleet_status. `create` leaves it in
# Created state: no CPU, no node, no motion — and it inherits no restart
# policy, so it never comes up on boot. Runs after `up` so --remove-orphans
# can't have swept it on compose versions that treat profile-disabled
# services as orphans.
if docker compose --profile explore config --services 2>/dev/null | grep -qx explore; then
  docker compose --profile explore create explore
fi

echo "== now on $(git rev-parse --abbrev-ref HEAD) @ $(git rev-parse --short HEAD)"
docker ps --format '{{.Names}}  {{.Status}}' | grep scout || true
