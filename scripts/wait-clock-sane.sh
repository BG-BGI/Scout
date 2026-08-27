#!/bin/sh
# Hold a ROS container's launch until the host clock has stopped stepping.
#
# The Pi 5 RTC has no battery, so every boot starts on a stale clock
# (fake-hwclock's last save) and systemd-timesyncd steps it forward — by
# +40 h on 2026-08-27 — ~15-60 s into boot, AFTER docker has started the
# containers. A step that size lands on running ROS nodes and silently kills
# every nav2 lifecycle bond: bt_navigator, planner_server etc. vanish from
# the graph while the container looks healthy. Gating the launch here means
# the step happens before rclcpp ever initializes.
#
# Sane = two samples 5 s apart agree (no step in between) AND the year is
# plausible. Offline boots never sync, so give up after MAX_LOOPS and start
# anyway — a late step (WiFi arriving mid-run) is not covered here; the real
# cure is the RTC battery (J5). Loop count, not wall-clock elapsed, bounds
# the wait: elapsed time is meaningless across a step.
#
# POSIX sh: runs under the image's /bin/sh from docker-compose `command:`.

MAX_LOOPS=${CLOCK_GATE_MAX_LOOPS:-18}   # ~90 s of real time
loops=0

while :; do
    t0=$(date +%s)
    sleep 5
    t1=$(date +%s)
    drift=$(( t1 - t0 - 5 ))
    if [ "$drift" -ge -1 ] && [ "$drift" -le 1 ] && [ "$(date +%Y)" -ge 2026 ]; then
        echo "wait-clock-sane: clock stable at $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
        exit 0
    fi
    echo "wait-clock-sane: clock stepped ${drift}s (now $(date -u '+%Y-%m-%d %H:%M:%S UTC')), waiting..."
    loops=$(( loops + 1 ))
    if [ "$loops" -ge "$MAX_LOOPS" ]; then
        echo "wait-clock-sane: no stable+plausible clock after $loops checks, starting anyway"
        exit 0
    fi
done
