"""webui/mapswitch.js (the live-swap vs slam-restart decision behind the webui's
activateMap), exercised under Node, plus source-level checks that app.js and
index.html actually use it. R9/AE6: a cross-source switch restarts slam.

Skips the Node cases when `node` is absent (Pi container) — CI / the dev Mac
run them."""

import json
import pathlib
import re
import shutil
import subprocess

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
MAPSWITCH = REPO_ROOT / 'webui' / 'mapswitch.js'
APP_JS = REPO_ROOT / 'webui' / 'app.js'
INDEX_HTML = REPO_ROOT / 'webui' / 'index.html'

RUNNER = """
const MS = require(process.argv[1]);
const call = JSON.parse(process.argv[2]);
console.log(JSON.stringify(MS[call.fn](...call.args)));
"""


def _call(fn, *args):
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    proc = subprocess.run(
        [node, '-e', RUNNER, str(MAPSWITCH),
         json.dumps({'fn': fn, 'args': list(args)})],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return json.loads(proc.stdout)


def _meta(active, mode='auto', resolved='_omit', **maps):
    meta = {'active_map': active, 'slam_mode': mode, 'maps': {
        name: {'source': src} for name, src in maps.items()}}
    if resolved != '_omit':
        meta['resolved_mode'] = resolved
    return meta


SITE = {'scanned': 'scan', 'other_scan': 'scan', 'plan': 'drawing',
        'other_plan': 'drawing'}


def _plan(active, name, mode='auto', resolved='_omit'):
    return _call('activationPlan', _meta(active, mode, resolved, **SITE), name)


def test_scan_to_drawing_under_auto_restarts_and_needs_grid():
    plan = _plan('scanned', 'plan', resolved='continue')
    assert plan == {'live': False, 'crossSource': True, 'targetSource': 'drawing',
                    'currentSource': 'scan', 'needsGrid': True}


def test_drawing_to_drawing_under_auto_is_a_live_swap():
    plan = _plan('plan', 'other_plan', resolved='localization')
    assert plan['live'] is True and plan['crossSource'] is False
    assert plan['needsGrid'] is True


def test_drawing_to_scan_under_auto_restarts_without_needing_a_grid():
    plan = _plan('plan', 'scanned', resolved='localization')
    assert plan['live'] is False and plan['crossSource'] is True
    assert plan['needsGrid'] is False


def test_scan_to_scan_under_auto_is_not_live():
    plan = _plan('scanned', 'other_scan', resolved='continue')
    assert plan['live'] is False and plan['crossSource'] is False
    assert plan['needsGrid'] is False


@pytest.mark.parametrize('active,target,live,cross,needs_grid', [
    ('scanned', 'other_scan', True, False, True),
    ('scanned', 'plan', False, True, True),
    ('plan', 'scanned', False, True, True),
    ('plan', 'other_plan', True, False, True),
])
def test_pinned_localization(active, target, live, cross, needs_grid):
    plan = _plan(active, target, mode='localization', resolved='localization')
    assert (plan['live'], plan['crossSource'], plan['needsGrid']) == (
        live, cross, needs_grid)


@pytest.mark.parametrize('mode', ['continue', 'new'])
def test_mapping_modes_never_go_live(mode):
    for active, target in [('scanned', 'other_scan'), ('scanned', 'plan')]:
        assert _plan(active, target, mode=mode, resolved=mode)['live'] is False


def test_older_fleet_status_falls_back_to_raw_slam_mode():
    # no resolved_mode field: pinned localization still goes live, `auto` does not
    pinned = _plan('scanned', 'other_scan', mode='localization')
    assert pinned['live'] is True
    auto_on_drawing = _plan('plan', 'other_plan', mode='auto')
    assert auto_on_drawing['live'] is False


def test_resolved_mode_beats_raw_slam_mode():
    assert _call('effectiveSlamMode', {'slam_mode': 'auto',
                                       'resolved_mode': 'localization'}) == 'localization'
    assert _call('effectiveSlamMode', {'slam_mode': 'localization'}) == 'localization'


@pytest.mark.parametrize('meta', [None, {}, {'maps': {}}])
def test_effective_mode_defaults_to_auto(meta):
    assert _call('effectiveSlamMode', meta) == 'auto'


@pytest.mark.parametrize('entry', [None, {}, {'source': 'bim'}, {'source': 7},
                                   {'source': ['drawing']}, 'drawing'])
def test_unknown_source_reads_as_scan(entry):
    assert _call('mapSource', entry) == 'scan'


def test_known_sources_pass_through():
    assert _call('mapSource', {'source': 'drawing'}) == 'drawing'
    assert _call('mapSource', {'source': 'scan'}) == 'scan'


def test_no_active_map_reads_current_as_scan():
    meta = _meta(None, 'auto', 'new', plan='drawing', scanned='scan')
    assert _call('activationPlan', meta, 'plan')['crossSource'] is True
    assert _call('activationPlan', meta, 'scanned')['crossSource'] is False


def test_target_missing_from_maps_reads_as_scan():
    plan = _call('activationPlan', _meta('plan', 'auto', 'localization',
                                         plan='drawing'), 'ghost')
    assert plan['targetSource'] == 'scan' and plan['live'] is False


def test_needs_reseed_only_when_caught_up_and_running_amcl():
    caught_up = _meta('plan', 'auto', 'localization', plan='drawing')
    assert _call('needsReseed', caught_up, 'plan') is True
    assert _call('needsReseed', caught_up, 'scanned') is False
    assert _call('needsReseed', _meta('scanned', 'auto', 'continue', scanned='scan'),
                 'scanned') is False
    assert _call('needsReseed', None, 'plan') is False


def _activate_map_body():
    src = APP_JS.read_text()
    start = src.index('async function activateMap(')
    return src[start:src.index('\n}\n', start)]


def test_app_js_activate_map_uses_the_plan_not_raw_slam_mode():
    body = _activate_map_body()
    assert 'MapSwitch.activationPlan(activeSiteMeta, name)' in body
    assert 'slam_mode' not in body
    assert 'plan.live' in body
    assert 'containers/slam/restart' in body
    assert 'containers/behaviors/restart' in body
    assert 'reseedAfterRestart(name, baselineStamp)' in body


def test_app_js_post_restart_reseed_calls_the_existing_service_once_amcl_is_up():
    src = APP_JS.read_text()
    start = src.index('async function reseedAfterRestart(')
    body = src[start:src.index('\n}\n', start)]
    assert body.index('waitForFreshAmclPose') < body.index('reseedSrv.callService')
    assert 'Set pose after restart' in body
    assert "name: '/tag_relocalizer/reseed'" in src


def test_app_js_renders_site_text_via_text_content_only():
    # Names and labels come from site data: the new activation/reseed code must
    # not write them with innerHTML.
    src = APP_JS.read_text()
    body = _activate_map_body()
    start = src.index('async function reseedAfterRestart(')
    reseed = src[start:src.index('\n}\n', start)]
    for chunk in (body, reseed):
        assert 'innerHTML' not in chunk
        assert 'insertAdjacentHTML' not in chunk


def test_index_html_loads_mapswitch_before_app_js():
    html = INDEX_HTML.read_text()
    scripts = re.findall(r'<script src="([^"]+)"', html)
    assert scripts.index('mapswitch.js') < scripts.index('app.js')


# --- stale latched /amcl_pose vs the restarted amcl's pose ---------------------------

GATE_RUNNER = """
const MS = require(process.argv[1]);
const a = JSON.parse(process.argv[2]);
const gate = MS.freshPoseGate(a.baseline);
console.log(JSON.stringify(a.msgs.map((m) => gate(m))));
"""


def _stamped(sec, nanosec=0):
    return {'header': {'stamp': {'sec': sec, 'nanosec': nanosec}}}


def _gate(baseline, msgs):
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    proc = subprocess.run(
        [node, '-e', GATE_RUNNER, str(MAPSWITCH),
         json.dumps({'baseline': baseline, 'msgs': msgs})],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return json.loads(proc.stdout)


def test_amcl_stamp_is_sec_and_nanosec():
    assert _call('amclStamp', _stamped(100, 5)) == '100:5'


@pytest.mark.parametrize('msg', [
    None, {}, {'header': {}}, {'header': {'stamp': None}},
    {'header': {'stamp': {'sec': 1}}}, {'header': {'stamp': {'nanosec': 1}}},
    {'header': {'stamp': {'sec': '1', 'nanosec': 2}}}, {'header': {'stamp': 7}},
])
def test_amcl_stamp_is_null_without_a_usable_stamp(msg):
    assert _call('amclStamp', msg) is None


def test_baseline_present_same_stamp_is_not_fresh():
    assert _call('isFreshAmclPose', '100:5', _stamped(100, 5)) is False


def test_baseline_present_different_stamp_is_fresh():
    assert _call('isFreshAmclPose', '100:5', _stamped(100, 6)) is True
    assert _call('isFreshAmclPose', '100:5', _stamped(101, 5)) is True


def test_no_baseline_first_message_is_fresh():
    assert _call('isFreshAmclPose', None, _stamped(100, 5)) is True


def test_no_message_is_never_fresh():
    assert _call('isFreshAmclPose', None, None) is False
    assert _call('isFreshAmclPose', '100:5', None) is False


def test_stamp_older_than_the_baseline_still_counts_as_new():
    # clock-independent on purpose: the Pi clock steps at boot, so "newer than"
    # would be wrong; only "different from the latched one" is meaningful
    assert _call('isFreshAmclPose', '100:5', _stamped(50, 0)) is True


def test_stale_replay_then_real_pose_fires_exactly_once_after_the_real_one():
    old, new = _stamped(100, 5), _stamped(180, 1)
    assert _gate('100:5', [old, old, new, new, _stamped(181)]) == [
        False, False, True, False, False]


def test_gate_with_no_baseline_fires_on_the_first_message_only():
    assert _gate(None, [_stamped(1), _stamped(2)]) == [True, False]


def test_gate_never_fires_while_only_the_stale_pose_arrives():
    assert _gate('100:5', [_stamped(100, 5)] * 4) == [False] * 4


def test_gate_ignores_null_messages():
    assert _gate('100:5', [None, _stamped(200)]) == [False, True]


def test_app_js_captures_the_baseline_before_any_restart_request():
    body = _activate_map_body()
    restart = body[body.index('const crossNote'):]
    assert restart.index('nextAmclPose') < restart.index('postActiveMap(name)')
    assert restart.index('nextAmclPose') < restart.index('containers/slam/restart')
    assert "effectiveSlamMode(activeSiteMeta) === 'localization'" in restart


def test_app_js_waits_for_a_fresh_pose_not_just_any_pose():
    src = APP_JS.read_text()
    start = src.index('function waitForFreshAmclPose(')
    body = src[start:src.index('\n}\n', start)]
    assert 'MapSwitch.freshPoseGate(baselineStamp)' in body
    assert 'throttle_rate' not in src[src.index('function amclPoseTopic('):start]
