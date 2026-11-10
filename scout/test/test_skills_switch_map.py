"""scout-skills switch_map: live LoadMap vs slam restart on the EFFECTIVE mode,
cross-source switches restart slam, and the post-restart tag-relocalizer re-arm
(R9/AE6). server.py needs fastmcp/httpx/rosbridge, so — like test_sitestore's
_skills_load_site — the functions are AST-extracted and exec'd with fakes at the
system boundary (HTTP to fleet_status, rosbridge). Nothing here touches a robot."""

import ast
import asyncio
import json
import math
import os
import pathlib
import shutil
import subprocess
import types

import pytest

from scout.core import sites

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
SERVER = REPO / 'docker' / 'scout-skills' / 'server.py'
MAPSWITCH = REPO / 'webui' / 'mapswitch.js'

_TREE = ast.parse(SERVER.read_text())


def _extract(name, ns):
    fn = next(n for n in _TREE.body
              if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
              and n.name == name)
    fn.decorator_list = []
    exec(compile(ast.Module(body=[fn], type_ignores=[]), name, 'exec'), ns)
    return ns[name]


def _helpers():
    ns = {}
    for name in ('_map_source', '_resolved_mode', '_switch_plan'):
        _extract(name, ns)
    return ns


H = _helpers()


# --- pure helpers ---------------------------------------------------------------

@pytest.mark.parametrize('entry,want', [
    ({'source': 'drawing'}, 'drawing'), ({'source': 'scan'}, 'scan'),
    ({}, 'scan'), (None, 'scan'), ('drawing', 'scan'), ({'source': 'bim'}, 'scan'),
    ({'source': ['drawing']}, 'scan'), ({'source': 7}, 'scan'),
])
def test_map_source(entry, want):
    assert H['_map_source'](entry) == want


def test_map_source_agrees_with_core_sites():
    for entry in ({'source': s} for s in sites.SOURCES):
        assert H['_map_source'](entry) == sites.entry_source(entry)
    for junk in (None, {}, {'source': 'bim'}, 'x', {'source': 3}):
        assert H['_map_source'](junk) == sites.entry_source(junk)


def _payload(resolved='localization', active='office', name='office'):
    site = {'name': name}
    if resolved is not None:
        site['resolved_mode'] = resolved
    return {'active': active, 'sites': [{'name': 'other', 'resolved_mode': 'new'},
                                        site]}


def test_resolved_mode_reads_the_active_site():
    assert H['_resolved_mode'](_payload('localization'), 'auto') == 'localization'
    assert H['_resolved_mode'](_payload('continue'), 'auto') == 'continue'


@pytest.mark.parametrize('payload', [
    None, {}, [], 'x', {'sites': None}, {'active': 'a', 'sites': []},
    _payload(None), _payload(''), _payload('localization', active='nope'),
    {'active': 'office', 'sites': ['office', None]},
])
def test_resolved_mode_falls_back_to_raw_mode(payload):
    assert H['_resolved_mode'](payload, 'auto') == 'auto'


def _site(active, mode='auto', **maps):
    return {'slam_mode': mode, 'active_map': active,
            'maps': {n: {'source': s} for n, s in maps.items()}}


SRC = {'scanned': 'scan', 'other_scan': 'scan', 'plan': 'drawing',
       'other_plan': 'drawing'}


@pytest.mark.parametrize('active,target,raw,mode,live,cross,grid', [
    ('scanned', 'plan', 'auto', 'continue', False, True, True),
    ('plan', 'other_plan', 'auto', 'localization', True, False, True),
    ('plan', 'scanned', 'auto', 'localization', False, True, False),
    ('scanned', 'other_scan', 'auto', 'continue', False, False, False),
    ('scanned', 'other_scan', 'localization', 'localization', True, False, True),
    ('scanned', 'plan', 'localization', 'localization', False, True, True),
    ('plan', 'scanned', 'localization', 'localization', False, True, True),
    ('scanned', 'other_scan', 'continue', 'continue', False, False, False),
    ('scanned', 'plan', 'new', 'new', False, True, False),
])
def test_switch_plan(active, target, raw, mode, live, cross, grid):
    plan = H['_switch_plan'](_site(active, raw, **SRC), target, mode)
    assert (plan['live'], plan['cross_source'], plan['needs_grid']) == (
        live, cross, grid)


def test_switch_plan_no_active_map_reads_scan():
    plan = H['_switch_plan'](_site(None, 'auto', plan='drawing'), 'plan', 'new')
    assert plan['cross_source'] is True and plan['current_source'] == 'scan'


def test_switch_plan_matches_webui_activation_plan():
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    runner = ('const MS = require(process.argv[1]);'
              'const cases = JSON.parse(process.argv[2]);'
              'console.log(JSON.stringify(cases.map(c => '
              'MS.activationPlan(c.meta, c.name))));')
    cases = []
    for raw in ('auto', 'localization', 'continue', 'new'):
        for mode in ('localization', 'continue', 'new'):
            for active in (None, 'scanned', 'plan'):
                for target in ('scanned', 'other_scan', 'plan', 'other_plan'):
                    meta = _site(active, raw, **SRC)
                    meta['resolved_mode'] = mode
                    cases.append({'meta': meta, 'name': target, 'mode': mode})
    got = subprocess.run(
        [node, '-e', runner, str(MAPSWITCH), json.dumps(cases)],
        capture_output=True, text=True, check=True).stdout
    for case, js in zip(cases, json.loads(got), strict=True):
        py = H['_switch_plan'](case['meta'], case['name'], case['mode'])
        assert (js['live'], js['crossSource'], js['needsGrid']) == (
            py['live'], py['cross_source'], py['needs_grid']), case


# --- _effective_mode / _reseed_when_amcl_up / switch_map with fakes --------------

class FakeHttpError(Exception):
    pass


class FakeResp:
    def __init__(self, payload=None):
        self._payload = payload

    def raise_for_status(self):
        return None

    def json(self):
        if isinstance(self._payload, Exception):
            raise self._payload
        return self._payload


class FakeFleet:
    """Stands in for `httpx` and records every call. /api/sites answers with
    `before` until the active_map POST lands, then `after`."""

    HTTPError = FakeHttpError

    def __init__(self, before='localization', after='localization',
                 get_fails=False, post_fails=False):
        self.before, self.after = before, after
        self.get_fails, self.post_fails = get_fails, post_fails
        self.switched = False
        self.calls = []

    def AsyncClient(self, timeout=None):
        return _FakeClient(self)

    def payload(self):
        return _payload(self.after if self.switched else self.before)


class _FakeClient:
    def __init__(self, fleet):
        self.fleet = fleet

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def get(self, url):
        self.fleet.calls.append(('GET', url))
        if self.fleet.get_fails:
            raise FakeHttpError('fleet_status down')
        return FakeResp(self.fleet.payload())

    async def post(self, url, json=None):
        self.fleet.calls.append(('POST', url, json))
        if self.fleet.post_fails:
            raise FakeHttpError('fleet_status down')
        if url.endswith('/api/sites/active'):
            self.fleet.switched = True
        return FakeResp({})


def test_effective_mode_prefers_resolved_mode():
    ns = dict(H, httpx=FakeFleet('localization'), FLEET_STATUS_URL='http://fs')
    fn = _extract('_effective_mode', ns)
    assert asyncio.run(fn({'slam_mode': 'auto'})) == 'localization'


def test_effective_mode_falls_back_when_fleet_status_is_down():
    ns = dict(H, httpx=FakeFleet(get_fails=True), FLEET_STATUS_URL='http://fs')
    fn = _extract('_effective_mode', ns)
    assert asyncio.run(fn({'slam_mode': 'localization'})) == 'localization'


def test_effective_mode_falls_back_on_bad_json():
    fleet = FakeFleet()
    fleet.payload = lambda: ValueError('not json')
    ns = dict(H, httpx=fleet, FLEET_STATUS_URL='http://fs')
    assert asyncio.run(_extract('_effective_mode', ns)({'slam_mode': 'auto'})) == 'auto'


def test_effective_mode_older_fleet_status_without_the_field():
    ns = dict(H, httpx=FakeFleet(before=None), FLEET_STATUS_URL='http://fs')
    assert asyncio.run(_extract('_effective_mode', ns)({'slam_mode': 'auto'})) == 'auto'


class FakeBridgeError(Exception):
    pass


class FakeBridge:
    """Stands in for RosBridge. `pose_after` = number of subscribe_once calls
    that come back empty before /amcl_pose publishes (None = never)."""

    log = []
    empties_left = 0
    reseed_fails = False
    busy_first = 0

    def __init__(self, *, background=False, **_):
        self.background = background

    async def __aenter__(self):
        if self.background and FakeBridge.busy_first > 0:
            FakeBridge.busy_first -= 1
            raise FakeBridgeError('busy')
        return self

    async def __aexit__(self, *exc):
        return False

    async def subscribe_once(self, topic, msg_type, timeout=5.0):
        FakeBridge.log.append(('subscribe_once', topic))
        if FakeBridge.empties_left > 0:
            FakeBridge.empties_left -= 1
            return None
        return {'pose': 'x'}

    async def call_service(self, name, srv_type, args=None):
        FakeBridge.log.append(('call_service', name, args))
        if name == '/tag_relocalizer/reseed' and FakeBridge.reseed_fails:
            raise FakeBridgeError('service gone')
        return {'result': 0}

    async def publish(self, topic, msg_type, msg):
        FakeBridge.log.append(('publish', topic))


@pytest.fixture
def bridge():
    FakeBridge.log = []
    FakeBridge.empties_left = 0
    FakeBridge.reseed_fails = False
    FakeBridge.busy_first = 0
    return FakeBridge


def _reseed_fn(attempts=3):
    ns = {'asyncio': asyncio, 'RosBridge': FakeBridge,
          'RosBridgeError': FakeBridgeError, 'AMCL_POLL_S': 0.0,
          'AMCL_POLL_ATTEMPTS': attempts}
    return _extract('_reseed_when_amcl_up', ns)


def _reseeds(log):
    return [e for e in log if e[:2] == ('call_service', '/tag_relocalizer/reseed')]


def test_reseed_waits_for_amcl_pose_then_calls_the_existing_service(bridge):
    bridge.empties_left = 2
    assert asyncio.run(_reseed_fn()()) is True
    kinds = [e[0] for e in bridge.log]
    assert kinds == ['subscribe_once'] * 3 + ['call_service']
    assert len(_reseeds(bridge.log)) == 1


def test_reseed_gives_up_when_amcl_never_publishes(bridge):
    bridge.empties_left = 99
    assert asyncio.run(_reseed_fn(attempts=3)()) is False
    assert _reseeds(bridge.log) == []
    assert [e[0] for e in bridge.log] == ['subscribe_once'] * 3


def test_reseed_retries_through_rosbridge_busy(bridge):
    bridge.busy_first = 2
    assert asyncio.run(_reseed_fn()()) is True
    assert len(_reseeds(bridge.log)) == 1


def test_reseed_reports_false_when_the_service_call_fails(bridge):
    bridge.reseed_fails = True
    assert asyncio.run(_reseed_fn()()) is False


class SwitchHarness:
    def __init__(self, tmp_path, site, fleet, grids=()):
        self.tasks = []
        (tmp_path / 'maps').mkdir()
        for g in grids:
            (tmp_path / 'maps' / (g + '.yaml')).write_text('x')
        self.fleet = fleet

        async def fake_reseed():
            self.tasks.append('reseed')
            return True

        async def idle():
            return None

        ns = dict(H)
        ns.update(
            asyncio=asyncio, os=os, math=math, httpx=fleet, RosBridge=FakeBridge,
            RosBridgeError=FakeBridgeError, ToolError=_ToolError,
            tagdb=types.SimpleNamespace(SITE_JSON=str(tmp_path / 'site.json')),
            FLEET_STATUS_URL='http://fs', SLAM_MAPS_DIR='/slam/maps',
            _load_site=lambda: site, _require_motion_idle=idle,
            _stamped_pose=lambda x, y, yaw: {'header': {}, 'pose': {}},
            _reseed_when_amcl_up=fake_reseed, _reseed_task=None)
        _extract('_effective_mode', ns)
        self.ns = ns
        self.switch = _extract('switch_map', ns)

    def run(self, name):
        async def go():
            out = await self.switch(name)
            await asyncio.sleep(0)
            return out
        return asyncio.run(go())

    @property
    def posts(self):
        return [c for c in self.fleet.calls if c[0] == 'POST']

    def restarts(self):
        return [c[1].split('/')[-2] for c in self.posts if c[1].endswith('/restart')]

    def load_maps(self):
        return [e for e in FakeBridge.log
                if e[:2] == ('call_service', '/map_server/load_map')]


class _ToolError(Exception):
    pass


def _full_site(active, raw='auto'):
    site = _site(active, raw, **SRC)
    for entry in site['maps'].values():
        entry['map_start_pose'] = [1.0, 2.0, 0.5]
    return site


ALL_GRIDS = tuple(SRC)


def test_same_source_drawing_switch_stays_live_under_auto(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('plan'), FakeFleet('localization'),
                      ALL_GRIDS)
    out = h.run('other_plan')
    assert out['switched'].startswith('live')
    assert len(h.load_maps()) == 1 and h.restarts() == [] and h.tasks == []
    assert [c[1] for c in h.posts] == ['http://fs/api/sites/active']


def test_scan_to_drawing_under_auto_restarts_slam_and_rearms(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned'),
                      FakeFleet('continue', 'localization'), ALL_GRIDS)
    out = h.run('plan')
    assert out['switched'].startswith('slam restart pending')
    assert 'scan -> drawing' in out['note']
    assert h.load_maps() == []
    assert h.restarts() == ['slam', 'behaviors']
    assert h.posts[0][1] == 'http://fs/api/sites/active'
    assert h.tasks == ['reseed']
    assert 'set pose after restart' in out['reseed'].lower()


def test_drawing_to_scan_under_auto_restarts_without_a_reseed(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('plan'),
                      FakeFleet('localization', 'continue'), ALL_GRIDS)
    out = h.run('scanned')
    assert h.load_maps() == [] and h.restarts() == ['slam', 'behaviors']
    assert h.tasks == [] and 'reseed' not in out


def test_pinned_localization_cross_source_restarts_and_rearms(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned', 'localization'),
                      FakeFleet('localization', 'localization'), ALL_GRIDS)
    h.run('plan')
    assert h.load_maps() == [] and h.restarts() == ['slam', 'behaviors']
    assert h.tasks == ['reseed']


def test_pinned_localization_same_source_is_live_as_before(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned', 'localization'),
                      FakeFleet('localization'), ALL_GRIDS)
    out = h.run('other_scan')
    assert out['switched'].startswith('live')
    assert h.restarts() == [] and h.tasks == []


def test_scan_switch_in_a_mapping_mode_restarts_as_before(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned'),
                      FakeFleet('continue', 'continue'), ALL_GRIDS)
    h.run('other_scan')
    assert h.load_maps() == [] and h.restarts() == ['slam', 'behaviors']
    assert h.tasks == []


def test_older_fleet_status_pinned_localization_still_goes_live(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned', 'localization'),
                      FakeFleet(before=None, after=None), ALL_GRIDS)
    assert h.run('other_scan')['switched'].startswith('live')


def test_older_fleet_status_auto_on_drawing_restarts(tmp_path, bridge):
    # raw `auto` is all it can see: today's behavior (restart), never a bad live swap
    h = SwitchHarness(tmp_path, _full_site('plan'),
                      FakeFleet(before=None, after=None), ALL_GRIDS)
    h.run('other_plan')
    assert h.load_maps() == [] and h.restarts() == ['slam', 'behaviors']


def test_fleet_status_unreachable_for_the_mode_falls_back_to_raw(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned', 'localization'),
                      FakeFleet(get_fails=True), ALL_GRIDS)
    assert h.run('other_scan')['switched'].startswith('live')


def test_target_needing_a_grid_without_one_is_refused_before_any_change(
        tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned'),
                      FakeFleet('continue', 'localization'),
                      grids=('scanned', 'other_scan'))
    with pytest.raises(_ToolError, match='no grid'):
        h.run('plan')
    assert h.posts == [] and h.tasks == []


def test_scan_target_without_a_grid_still_restarts_in_auto(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('plan'),
                      FakeFleet('localization', 'continue'), grids=('plan',))
    h.run('scanned')
    assert h.restarts() == ['slam', 'behaviors']


def test_unknown_map_and_already_active_short_circuit(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('plan'), FakeFleet(), ALL_GRIDS)
    with pytest.raises(_ToolError, match='no map'):
        h.run('ghost')
    assert h.run('plan')['note'] == 'already active'
    assert h.posts == []


def test_persist_failure_on_a_restart_switch_raises_and_does_not_rearm(
        tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned'),
                      FakeFleet('continue', 'localization', post_fails=True),
                      ALL_GRIDS)
    with pytest.raises(_ToolError, match='not persisted'):
        h.run('plan')
    assert h.tasks == []


def test_a_newer_switch_cancels_a_pending_rearm(tmp_path, bridge):
    h = SwitchHarness(tmp_path, _full_site('scanned', 'localization'),
                      FakeFleet('localization', 'localization'), ALL_GRIDS)

    class Pending:
        cancelled = False

        def cancel(self):
            Pending.cancelled = True

    h.ns['_reseed_task'] = Pending()
    h.run('plan')
    assert Pending.cancelled is True


def test_switch_map_source_branches_on_the_plan_not_raw_slam_mode():
    src = ast.get_source_segment(SERVER.read_text(), next(
        n for n in _TREE.body
        if isinstance(n, ast.AsyncFunctionDef) and n.name == 'switch_map'))
    assert '_switch_plan(site, name, mode)' in src
    assert 'mode == "localization"' not in src
    assert 'mode != "localization"' not in src
