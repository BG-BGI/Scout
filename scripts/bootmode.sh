#!/bin/bash
# bootmode.sh pulse|apply — the dev-mode gate (docs/deploy.md).
#
# pulse  (scout-bootpulse.service, Before=docker.service):
#   A leftover `interrupted` flag means last boot's pulse was cut by power —
#   consume it into `dev_next` so THIS boot is dev. Then arm the flag, run
#   the LED pulse, disarm. The flag survives only on a mid-pulse power cut.
#
# apply  (scout-bootmode.service, After=docker.service):
#   dev_next or a persistent SD flag -> dev mode: the whole project is
#   stopped except webui + fleet_status (the web UI's System panel is the
#   recovery surface; fleet_status is its API). Otherwise normal bring-up.
#   dev_next is consumed either way — dev mode is ONE-SHOT by design.
#
# State lives on the rootfs (written every boot). The SD override lives on
# the FAT /boot partition so it can be flipped from any laptop when SSH is
# unreachable.
set -uo pipefail

STATE=/var/lib/scout-bootmode
FLAG="$STATE/interrupted"
DEVNEXT="$STATE/dev_next"
SD_FLAGS="/boot/firmware/scout_dev /boot/scout_dev"
DEV_SERVICES="webui fleet_status"
# Same profile sweep as deploy-pi.sh — covers profile-gated stragglers too.
ALL_PROFILES="--profile full --profile observability --profile explore --profile build"
REPO="${SCOUT_REPO:-$HOME/Desktop/Scout}"

log() { logger -t scout-bootmode -- "$*"; echo "scout-bootmode: $*"; }

case "${1:-}" in
pulse)
  mkdir -p "$STATE"
  if [ -f "$FLAG" ]; then
    touch "$DEVNEXT"
    log "pulse was interrupted last boot -> dev mode this boot"
  fi
  : > "$FLAG"   # arm — persists only if power dies mid-pulse
  python3 "$(dirname "$0")/boot_led_pulse.py"
  rc=$?
  # Disarm only on a completed run; a crash leaves the flag armed -> dev
  # next boot, the safe direction for a broken arm window.
  [ "$rc" -eq 0 ] && rm -f "$FLAG"
  exit "$rc"
  ;;
apply)
  dev=0
  if [ -f "$DEVNEXT" ]; then
    dev=1
    rm -f "$DEVNEXT"
  fi
  for f in $SD_FLAGS; do
    [ -f "$f" ] && { dev=1; log "SD flag $f -> dev mode"; }
  done
  if ! cd "$REPO"; then
    log "repo $REPO missing — nothing applied"
    exit 1
  fi
  if [ "$dev" -eq 1 ]; then
    # Stop the whole project (docker autostart spawned it seconds ago),
    # then bring back just the recovery surface.
    docker compose $ALL_PROFILES stop
    docker compose up -d $DEV_SERVICES
    log "DEV MODE — only $DEV_SERVICES up. Exit: 'sudo scripts/devmode.sh off', a clean boot, or delete the SD flag."
  else
    docker compose --profile full up -d --remove-orphans
    log "normal boot — stack up"
  fi
  ;;
*)
  echo "usage: bootmode.sh pulse|apply" >&2
  exit 2
  ;;
esac
