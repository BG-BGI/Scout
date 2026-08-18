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

docker compose build
docker compose --profile build run --rm build_package

# Bring up only the always-on services this branch's compose defines.
# slam + nav2 are always-on since patrol_capture: marking waypoints needs the
# map frame and patrols need the planner, and a deploy must not strand them.
WANT="robot behaviors rosbridge webui foxglove_bridge slam nav2 ros_mcp"
HAVE=$(docker compose config --services)
UP=""
for s in $WANT; do echo "$HAVE" | grep -qx "$s" && UP="$UP $s"; done
# --remove-orphans clears containers for services the new branch doesn't define.
docker compose up -d --remove-orphans $UP

echo "== now on $(git rev-parse --abbrev-ref HEAD) @ $(git rev-parse --short HEAD)"
docker ps --format '{{.Names}}  {{.Status}}' | grep scout || true
