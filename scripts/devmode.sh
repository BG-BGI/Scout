#!/bin/bash
# devmode.sh on|off|status — dev-mode toggle over SSH (docs/deploy.md).
#
#   on      dev mode NOW: stack stops, only webui + fleet_status run.
#           One-shot — the next clean boot returns to normal.
#   off     back to normal NOW (docker compose --profile full up -d).
#   status  show arm state + which containers are up.
#
# The power-switch gesture (cut power during the boot LED pulse) and the
# SD flag (/boot/firmware/scout_dev) arm the same state.
set -euo pipefail

STATE=/var/lib/scout-bootmode
DEVNEXT="$STATE/dev_next"
FLAG="$STATE/interrupted"
SD_FLAGS="/boot/firmware/scout_dev /boot/scout_dev"
HERE="$(cd "$(dirname "$0")" && pwd)"
export SCOUT_REPO="${SCOUT_REPO:-$(dirname "$HERE")}"

# State dir + docker stop both want root — re-exec under sudo if needed.
if [ "$EUID" -ne 0 ]; then
  exec sudo --preserve-env=SCOUT_REPO "$0" "$@"
fi

case "${1:-}" in
on)
  mkdir -p "$STATE"
  touch "$DEVNEXT"
  "$HERE/bootmode.sh" apply
  echo "dev mode ON — webui + fleet_status only. Next clean boot returns to normal."
  ;;
off)
  rm -f "$DEVNEXT" "$FLAG"
  "$HERE/bootmode.sh" apply
  for f in $SD_FLAGS; do
    [ -f "$f" ] && echo "note: SD flag $f still set — dev mode re-arms every boot until removed"
  done
  ;;
status)
  [ -f "$DEVNEXT" ] && echo "dev_next: ARMED (this/next boot is dev)" || echo "dev_next: clear"
  [ -f "$FLAG" ] && echo "interrupted flag: present (mid-pulse now, or last pulse was cut)" || true
  for f in $SD_FLAGS; do
    [ -f "$f" ] && echo "SD flag: $f (persistent dev)"
  done
  echo "-- running containers:"
  docker ps --format '  {{.Names}}  {{.Status}}' | sort || true
  ;;
*)
  echo "usage: devmode.sh on|off|status" >&2
  exit 2
  ;;
esac
