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
import tempfile

import yaml

# The ONE place the bind-mount path may appear (SC6, ADR-0013): every other
# module and launch file resolves config through the helpers below.
# ament_index is imported lazily (inside the resolvers) so this module — and
# the pure deep_merge below — imports on a plain Python box for the tests.
_BIND_DIR = '/ros_ws/src/scout/config'
_cache = None


def _share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('scout'), 'config')


def resolve_config_dir() -> str:
    """The scout config directory — the bind-mounted repo copy wins over the
    installed share copy, so an edit under /ros_ws/src takes effect on the
    next start with no rebuild."""
    if os.path.isdir(_BIND_DIR):
        return _BIND_DIR
    return _share_dir()


def resolve_config(name: str) -> str:
    """Absolute path of a config file. Basenames resolve bind-mount-first
    (per file, so a fresh repo file wins even before an install); absolute
    paths pass through. Raises if the file does not exist."""
    if os.path.isabs(name):
        path = name
    else:
        path = os.path.join(_BIND_DIR, name)
        if not os.path.isfile(path):
            path = os.path.join(_share_dir(), name)
    if not os.path.isfile(path):
        raise RuntimeError('scout config file not found: %s' % path)
    return path


# --- scenario profiles (ADR-0010): base config + a small delta overlay --------

def known_profiles() -> list:
    """['default'] plus each subdirectory of config/overlays/."""
    root = os.path.join(resolve_config_dir(), 'overlays')
    subs = sorted(os.listdir(root)) if os.path.isdir(root) else []
    return ['default'] + [d for d in subs
                          if os.path.isdir(os.path.join(root, d))]


def profile_overlay(profile: str, basename: str):
    """Absolute path of `basename`'s overlay under `profile`, or None if the
    profile has no overlay for that file. Raises on an unknown profile."""
    if profile == 'default':
        return None
    known = known_profiles()
    if profile not in known:
        raise RuntimeError('unknown profile %r (known: %s)' % (profile, known))
    path = os.path.join(resolve_config_dir(), 'overlays', profile, basename)
    return path if os.path.isfile(path) else None


def deep_merge(base: dict, overlay: dict) -> dict:
    """Recursively merge `overlay` into `base` and return a new dict: dict
    values recurse; list/scalar values REPLACE wholesale; an overlay value of
    None DELETES the key. Pure — no I/O, so the overlay tests run off-ROS."""
    out = dict(base)
    for key, val in overlay.items():
        if val is None:
            out.pop(key, None)
        elif isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], val)
        else:
            out[key] = val
    return out


def merged_params(basename: str, profile: str) -> str:
    """Effective params-file path for `basename` under `profile`. 'default' (or
    a profile with no overlay for this file) returns the base path UNCHANGED —
    byte-identical, no temp file. Otherwise base is deep-merged with the overlay
    into /tmp/scout_profile/<profile>-<basename> and that path is returned."""
    base = resolve_config(basename)
    overlay_path = profile_overlay(profile, basename)
    if overlay_path is None:
        return base
    with open(base) as f:
        base_data = yaml.safe_load(f) or {}
    with open(overlay_path) as f:
        overlay_data = yaml.safe_load(f) or {}
    merged = deep_merge(base_data, overlay_data)
    out_dir = os.path.join(tempfile.gettempdir(), 'scout_profile')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, '%s-%s' % (profile, basename))
    with open(out_path, 'w') as f:
        yaml.safe_dump(merged, f)
    return out_path


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
