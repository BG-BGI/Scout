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

# The ONE place the bind-mount path may appear (SC6, ADR-0013): every other
# module and launch file resolves config through the helpers below.
_BIND_DIR = '/ros_ws/src/scout/config'
_cache = None


def resolve_config_dir() -> str:
    """The scout config directory — the bind-mounted repo copy wins over the
    installed share copy, so an edit under /ros_ws/src takes effect on the
    next start with no rebuild."""
    if os.path.isdir(_BIND_DIR):
        return _BIND_DIR
    return os.path.join(get_package_share_directory('scout'), 'config')


def resolve_config(name: str) -> str:
    """Absolute path of a config file. Basenames resolve bind-mount-first
    (per file, so a fresh repo file wins even before an install); absolute
    paths pass through. Raises if the file does not exist."""
    if os.path.isabs(name):
        path = name
    else:
        path = os.path.join(_BIND_DIR, name)
        if not os.path.isfile(path):
            path = os.path.join(
                get_package_share_directory('scout'), 'config', name)
    if not os.path.isfile(path):
        raise RuntimeError('scout config file not found: %s' % path)
    return path


def _resolve() -> str:
    return resolve_config('robot_profile.yaml')


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
