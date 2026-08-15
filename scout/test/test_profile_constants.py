"""SC8 + SC10: robot_profile.yaml is the single source of truth (ADR-0013).

SC8 bans profile-owned numbers appearing as bare literals on any surface —
the profile was forked three ways (publish_hz 25/30/20) one commit after being
declared the SSOT. The banned table is DERIVED from the yaml so it cannot
drift. Escape hatch, reviewed like code: a line containing
`profile-exempt: <reason>` is skipped.

SC10 freezes the two deliberate copies: webui/robot_profile.yaml must stay
byte-identical to the SSOT, and the vendored docker/scout-skills/geometry.py
functions must stay textually identical to scout.core.geometry's.
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

SCANNED = (
    'scout/scout', 'docker/scout-skills', 'scripts',
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
            yield from sorted(p.glob('*.py'))


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


def test_sc10_webui_profile_copy_is_identical():
    webui = REPO / 'webui' / 'robot_profile.yaml'
    assert webui.read_bytes() == PROFILE.read_bytes(), (
        'webui/robot_profile.yaml has drifted from scout/config/'
        'robot_profile.yaml — copy the SSOT over it (it exists only because '
        'the webui container cannot reach scout/config)')


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
