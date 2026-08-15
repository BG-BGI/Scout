"""Loader for scout/config/robot_profile.yaml — the cross-surface SSOT.

See the YAML header for the contract and field list. The bind-mounted repo
copy wins over the installed share copy (same policy the launch files use), so
an edit under /ros_ws/src takes effect on the next node start with no rebuild.
The parsed mapping is cached process-wide; a node reads it once at construction.

Missing file raises rather than silently falling back to baked defaults: the
whole point of this file is that the value is never quietly wrong, and it ships
both bind-mounted and installed to share, so absence means a broken install.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

_BIND_PATH = '/ros_ws/src/scout/config/robot_profile.yaml'
_cache = None


def _resolve() -> str:
    if os.path.isfile(_BIND_PATH):
        return _BIND_PATH
    share = os.path.join(
        get_package_share_directory('scout'), 'config', 'robot_profile.yaml')
    if os.path.isfile(share):
        return share
    raise RuntimeError(
        'robot_profile.yaml not found (looked at %s and %s)'
        % (_BIND_PATH, share))


def load() -> dict:
    """Return the parsed ``robot_profile:`` mapping (cached)."""
    global _cache
    if _cache is None:
        with open(_resolve()) as f:
            data = yaml.safe_load(f) or {}
        prof = data.get('robot_profile')
        if not isinstance(prof, dict):
            raise RuntimeError(
                'robot_profile.yaml missing a top-level robot_profile: map')
        _cache = prof
    return _cache
