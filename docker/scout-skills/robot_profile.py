"""Reader for the cross-surface robot profile (the SSOT the ROS nodes, webui
and Foxglove also draw from — scout/config/robot_profile.yaml).

Mounted read-only at /robot_profile.yaml (see docker-compose scout_skills). If
the mount is absent it falls back to baked defaults with a loud stderr log, so
a missing mount degrades to the values these tools hardcoded before, rather
than crashing the endpoint. Only the fields this container consumes are baked.
"""

import sys

import yaml

_PATH = "/robot_profile.yaml"

# Last-resort defaults == the yaml (kept minimal: only what scout-skills reads).
_BAKED = {
    "linear_floor": 0.05,
    "linear_cap": 1.0,
    "angular_floor": 0.35,
    "angular_cap": 3.0,
    "topic_cmd_vel_skills": "/cmd_vel",
    "goal_status_names": [
        "unknown", "accepted", "driving", "canceling",
        "arrived", "canceled", "aborted",
    ],
    "occupied_threshold": 50,
}

_cache = None


def load() -> dict:
    """Return the ``robot_profile:`` mapping (cached), merged over baked
    defaults so a missing key can never KeyError a tool."""
    global _cache
    if _cache is None:
        try:
            with open(_PATH) as f:
                data = (yaml.safe_load(f) or {}).get("robot_profile") or {}
            _cache = {**_BAKED, **data}
        except FileNotFoundError:
            print(
                f"WARNING: {_PATH} not mounted — using baked robot-profile "
                "defaults (velocity caps / status names / thresholds)",
                file=sys.stderr,
            )
            _cache = dict(_BAKED)
    return _cache
