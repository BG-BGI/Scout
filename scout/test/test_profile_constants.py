"""SC8 + SC10: robot_profile.yaml is the single source of truth (ADR-0013).

SC8 bans profile-owned numbers appearing as bare literals on any surface —
the profile was forked three ways (publish_hz 25/30/20) one commit after being
declared the SSOT. The banned table is DERIVED from the yaml so it cannot
drift. Escape hatch, reviewed like code: a line containing
`profile-exempt: <reason>` is skipped.

SC10 freezes the deliberate cross-container copies (files a container cannot
import): the vendored docker/scout-skills/geometry.py functions must stay
textually identical to scout.core.geometry's. (webui/robot_profile.yaml was a
frozen copy until 2026-08-24; compose now bind-mounts the SSOT into the served
dir instead, so the copy — and its byte-identity test — are gone.)
"""

import ast
import pathlib
import re

import yaml

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
PROFILE = REPO / 'scout' / 'config' / 'robot_profile.yaml'

# Profile keys whose values are distinctive enough to grep for. Deliberately
# NOT every key: generic values (1.0, 0.05, 50) would drown in false positives
# — those are guarded by the adoption fixes themselves, not a literal ban.
BANNED_KEYS = (
    'battery_warn_v',            # 17.5
    'battery_critical_v',        # 16.5
    'battery_activity_floor_v',  # 17.0
    'publish_hz',                # 25.0 (was forked as 30.0 / 20.0)
    # angular_floor (0.35) is deliberately NOT here: 0.35 is a common tuning
    # value (YOLO confidence, seek speeds, bench MinZ) and the ban drowned in
    # coincidences. Its one real fork (trick_player min_pivot_rate) now reads
    # the profile.
)

# Directories are scanned RECURSIVELY (a non-recursive glob silently skipped
# scout/scout/core and all of companion/ until 2026-08-24).
SCANNED = (
    'scout/scout', 'docker/scout-skills', 'scripts', 'companion',
    'webui/app.js', 'webui/index.html',
)


def _profile():
    with open(PROFILE) as f:
        return yaml.safe_load(f)['robot_profile']


def _scan_files():
    for entry in SCANNED:
        p = REPO / entry
        if p.is_file():
            yield p
        else:
            yield from sorted(p.rglob('*.py'))


def test_sc8_profile_values_not_hardcoded():
    prof = _profile()
    banned = {}
    for key in BANNED_KEYS:
        v = float(prof[key])
        # Match 17.5, 17.50; and the bare int form (35 is not 0.35's int form
        # so int aliases only apply to whole numbers).
        pats = [re.escape('%g' % v)]
        if v == int(v):
            pats.append(re.escape('%d' % int(v)) + r'\.0*')
        banned[key] = re.compile(r'(?<![\d.])(?:%s)(?![\d])' % '|'.join(pats))
    offenders = []
    for path in _scan_files():
        if path.samefile(PROFILE):
            continue
        for lineno, line in enumerate(path.read_text().splitlines(), 1):
            if 'profile-exempt:' in line:
                continue
            for key, pat in banned.items():
                if pat.search(line):
                    offenders.append('%s:%d looks like %s — read it from '
                                     'robot_profile instead'
                                     % (path.relative_to(REPO), lineno, key))
    assert not offenders, (
        'Profile-owned constants hardcoded (robot_profile.yaml is the SSOT, '
        'ADR-0013; waive a true coincidence with `profile-exempt: <reason>`):\n'
        + '\n'.join(offenders))


def test_sc10_webui_profile_is_placeholder_only():
    # The webui reads the SSOT via a compose bind mount shadowing this file
    # (a ro dir bind cannot create the mountpoint, so a tracked placeholder
    # must exist). Comments only — a real key means someone re-forked the
    # profile.
    text = (REPO / 'webui' / 'robot_profile.yaml').read_text()
    values = [ln for ln in text.splitlines()
              if ln.strip() and not ln.lstrip().startswith('#')]
    assert not values, (
        'webui/robot_profile.yaml grew real content — it is a mountpoint '
        'placeholder; the webui must read the SSOT via the compose mount '
        '(docker-compose.yaml webui service), never a copy:\n'
        + '\n'.join(values))


def _function_sources(path, names):
    tree = ast.parse(path.read_text())
    src = path.read_text().splitlines()
    out = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            out[node.name] = '\n'.join(src[node.lineno - 1:node.end_lineno])
    return out


def test_sc10_vendored_geometry_matches_core():
    shared = ('wrap_angle', 'planar_yaw', 'yaw_to_quat_zw', 'quat_to_matrix')
    core = _function_sources(
        REPO / 'scout' / 'scout' / 'core' / 'geometry.py', shared)
    vend = _function_sources(
        REPO / 'docker' / 'scout-skills' / 'geometry.py', shared)
    for name in shared:
        assert name in vend, ('vendored geometry.py lost %s — the skills '
                              'container needs it' % name)
        assert vend[name] == core[name], (
            'docker/scout-skills/geometry.py %s() drifted from '
            'scout.core.geometry — sync the vendored copy (ADR-0013)' % name)


def test_sc10_detect_copy_byte_identical():
    # The companion detector imports detect.py "verbatim reuse from
    # scout-skills" — verbatim is now a test, not a comment.
    skills = (REPO / 'docker' / 'scout-skills' / 'detect.py').read_bytes()
    companion = (REPO / 'companion' / 'detector' / 'detect.py').read_bytes()
    assert skills == companion, (
        'companion/detector/detect.py drifted from docker/scout-skills/'
        'detect.py — sync the copy (deliberate byte-identical reuse)')


def _function_body_dump(path, name):
    """ast dump of a module-level function's body, docstring stripped —
    compares logic while allowing docstring/annotation wording to differ."""
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            body = node.body
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)):
                body = body[1:]
            return '\n'.join(ast.dump(n) for n in body)
    raise AssertionError('%s lost %s()' % (path.name, name))


def test_sc10_median_depth_bodies_match():
    skills = _function_body_dump(
        REPO / 'docker' / 'scout-skills' / 'server.py', '_median_depth_m')
    companion = _function_body_dump(
        REPO / 'companion' / 'detector' / 'detector_node.py', '_median_depth_m')
    assert skills == companion, (
        '_median_depth_m drifted between docker/scout-skills/server.py and '
        'companion/detector/detector_node.py — sync the copies (same depth '
        'sampling must feed both world models)')


def test_sc10_waypoint_store_version_matches_core():
    # server.py vendors the ADR-0011 store SCHEMA (not the code). The version
    # constant is the compatibility gate both sides branch on.
    from scout.core import waypoints
    for node in ast.parse(
            (REPO / 'docker' / 'scout-skills' / 'server.py').read_text()).body:
        if (isinstance(node, ast.Assign)
                and getattr(node.targets[0], 'id', '') == 'WAYPOINTS_VERSION'):
            assert node.value.value == waypoints.VERSION, (
                'skills WAYPOINTS_VERSION != scout.core.waypoints.VERSION — '
                'the shared store would be migrated two different ways')
            return
    raise AssertionError('server.py lost WAYPOINTS_VERSION')
