#!/usr/bin/env bash
# Companion update — the whole deploy loop on the box side.
# Prereqs (one-time, host-setup.md): deploy-key clone + `docker login ghcr.io`.
set -euo pipefail
cd "$(dirname "$0")"
git pull --ff-only
docker compose pull
docker compose up -d --remove-orphans
docker image prune -f
