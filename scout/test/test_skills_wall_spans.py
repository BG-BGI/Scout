"""docker/scout-skills/server.py wall_spans: the shared helper, the MCP tool and
the REST route, on synthetic scans served by a fake rosbridge.

fastmcp, starlette, httpx and websockets are not in the shared test venv, so
server.py is imported against small stubs (restored afterwards). The stub
FastMCP records tool/custom_route registrations and fails on any app-wide
middleware, so the route-level CORS contract is checked on what server.py
actually registers. The fake RosBridge offers collect_multi and nothing else:
touching publish/advertise/call_service fails the test, which is how
'subscribe-only, never moves' is enforced here."""

import ast
import asyncio
import importlib.util
import json
import math
import sys
import types
from pathlib import Path

import pytest

TEST_DIR = Path(__file__).resolve().parent
SKILLS = TEST_DIR.parents[1] / 'docker' / 'scout-skills'
SERVER = SKILLS / 'server.py'


def _helpers():
    spec = importlib.util.spec_from_file_location(
        'scanmeasure_helpers', TEST_DIR / 'test_scanmeasure.py')
    mod = importlib.util.module_from_spec(spec)
    sys.modules['scanmeasure_helpers'] = mod
    spec.loader.exec_module(mod)
    return mod


H = _helpers()
YAW20 = H.YAW20
CONTRACT_KEYS = ['ok', 'frame', 'stamp', 'scans_used', 'stationary', 'walls', 'pairs', 'flags']
WALL_KEYS = ['id', 'bearing_rad', 'distance_m', 'spread_m', 'length_m', 'points', 'rms_m',
             'p0', 'p1']
PAIR_KEYS = ['a', 'b', 'span_m', 'spread_m', 'axis_rad', 'parallel_err_deg']


class ToolError(Exception):
    pass


class WebSocketException(Exception):
    pass


class RosBridgeError(RuntimeError):
    pass


class RosBridgeBusy(RosBridgeError):
    pass


class FakeRosBridge:
    """Replays a canned collect_multi window; everything else is forbidden."""

    feed = {}
    enter_error = None
    subscribed = []
    entered = 0

    def __init__(self, url=None, *, heavy=False, background=False):
        pass

    @classmethod
    def reset(cls, feed=None, enter_error=None):
        cls.feed, cls.enter_error, cls.subscribed, cls.entered = feed or {}, enter_error, [], 0

    async def __aenter__(self):
        FakeRosBridge.entered += 1
        if FakeRosBridge.enter_error:
            raise FakeRosBridge.enter_error
        return self

    async def __aexit__(self, *exc):
        return False

    async def collect_multi(self, specs, duration):
        FakeRosBridge.subscribed.extend(specs)
        return {t: list(FakeRosBridge.feed.get(t, [])) for t in specs}

    def __getattr__(self, name):
        raise AssertionError('wall_spans must be subscribe-only; touched RosBridge.%s' % name)


class StubMCP:
    def __init__(self, *args, **kwargs):
        self.routes, self.tools, self.middleware_calls = [], {}, []

    def tool(self, *args, **kwargs):
        def deco(fn):
            self.tools[fn.__name__] = kwargs
            return fn
        return deco(args[0]) if args and callable(args[0]) else deco

    def custom_route(self, path, methods, name=None, include_in_schema=True):
        def deco(fn):
            self.routes.append((path, list(methods), fn))
            return fn
        return deco

    def add_middleware(self, *args, **kwargs):
        self.middleware_calls.append((args, kwargs))

    def mount(self, *args, **kwargs):
        pass

    @staticmethod
    def as_proxy(*args, **kwargs):
        return object()

    def __getattr__(self, name):
        return lambda *a, **k: (lambda fn: fn)


class StubResponse:
    def __init__(self, content=b'', status_code=200, headers=None):
        self.status_code, self.headers, self.body = status_code, dict(headers or {}), content


class StubJSONResponse(StubResponse):
    def __init__(self, content, status_code=200, headers=None):
        super().__init__(json.dumps(content, allow_nan=False).encode(), status_code, headers)


def _module(name, **attrs):
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    return mod


def _stubs():
    from unittest import mock
    robot_profile = _module('robot_profile', load=lambda: {
        'goal_status_names': ['UNKNOWN'], 'topic_cmd_vel_out': '/cmd_vel'})
    stubs = {
        'fastmcp': _module('fastmcp', FastMCP=StubMCP, Client=mock.MagicMock()),
        'fastmcp.exceptions': _module('fastmcp.exceptions', ToolError=ToolError),
        'fastmcp.utilities': _module('fastmcp.utilities'),
        'fastmcp.utilities.types': _module('fastmcp.utilities.types', Image=object),
        'starlette': _module('starlette'),
        'starlette.requests': _module('starlette.requests', Request=object),
        'starlette.responses': _module(
            'starlette.responses', Response=StubResponse, JSONResponse=StubJSONResponse),
        'websockets': _module('websockets'),
        'websockets.exceptions': _module(
            'websockets.exceptions', WebSocketException=WebSocketException),
        'rosbridge': _module(
            'rosbridge', RosBridge=FakeRosBridge, RosBridgeBusy=RosBridgeBusy,
            RosBridgeError=RosBridgeError, ADVERTISE_SETTLE_S=0.7),
        'robot_profile': robot_profile,
    }
    for name in ('grid', 'httpx', 'poi', 'detect', 'motion', 'navpose', 'render', 'tf', 'tags',
                 'elevator_config', 'schindler_rbl'):
        stubs[name] = mock.MagicMock()
    return stubs


@pytest.fixture(scope='module')
def srv():
    saved = dict(sys.modules)
    sys.modules.update(_stubs())
    sys.path.insert(0, str(SKILLS))
    try:
        spec = importlib.util.spec_from_file_location('skills_server_wall_spans', SERVER)
        mod = importlib.util.module_from_spec(spec)
        sys.modules['skills_server_wall_spans'] = mod
        spec.loader.exec_module(mod)
    finally:
        sys.path.remove(str(SKILLS))
        for name in set(sys.modules) - set(saved):
            del sys.modules[name]
        sys.modules.update({k: v for k, v in saved.items() if sys.modules.get(k) is not v})
    return mod


@pytest.fixture
def rb(srv):
    FakeRosBridge.reset()
    return FakeRosBridge


def odom(v=0.0, w=0.0):
    return {'twist': {'twist': {'linear': {'x': v, 'y': 0.0, 'z': 0.0},
                                'angular': {'x': 0.0, 'y': 0.0, 'z': w}}}}


def parked_feed(robots=((0.4, -0.3, YAW20),) * 3, segs=None, odoms=None, static=True,
                laser_x=0.073):
    segs = segs or H.rect(0, 0, 4.0, 3.0)
    scans = []
    for k, pose in enumerate(robots):
        scan = H.room_scan(pose, segs)
        scan['header']['stamp'] = {'sec': 10, 'nanosec': int(k * 85e6)}
        scans.append(scan)
    feed = {'/scan': scans, '/odom': odoms if odoms is not None else [odom()] * 5}
    if static:
        feed['/tf_static'] = [H.tfmsg('base_link', 'laser', laser_x, 0.0, math.pi)]
    return feed


def run(coro):
    return asyncio.run(coro)


def get(srv, query=None, method='GET'):
    handler = srv.mcp.routes[0][2]
    request = types.SimpleNamespace(method=method, query_params=query or {})
    return run(handler(request))


def header(resp, name):
    return {k.lower(): v for k, v in resp.headers.items()}.get(name.lower())


def test_contract_keys_and_room_spans(srv, rb):
    rb.reset(parked_feed())
    res = run(srv._wall_spans())
    assert list(res) == CONTRACT_KEYS
    assert res['ok'] is True and res['frame'] == 'base_link'
    assert res['scans_used'] == 3 and res['stationary'] is True
    assert res['stamp'] == pytest.approx(10.17, abs=1e-6)
    assert all(list(w) == WALL_KEYS for w in res['walls'])
    assert all(list(p) == PAIR_KEYS for p in res['pairs'])
    assert sorted(round(p['span_m'], 2) for p in res['pairs']) == [3.0, 4.0]
    assert res['flags'] == []
    json.dumps(res, allow_nan=False)


def test_subscribe_only_and_never_moves(srv, rb):
    rb.reset(parked_feed())
    run(srv._wall_spans(include_points=True))
    assert sorted(rb.subscribed) == ['/odom', '/scan', '/tf_static']
    source = SERVER.read_text()
    block = source[source.index('# --- wall_spans'):source.index('# --- RFID')]
    for banned in ('cmd_vel', 'publish', 'advertise', 'call_service', 'send_action', 'run_move',
                   'run_rotate', '_zero_twist'):
        assert banned not in block.replace('NO cmd_vel', '').replace('no cmd_vel', '')


def test_static_tf_is_used_over_the_fallback(srv, rb):
    rb.reset(parked_feed(robots=((0.0, 0.0, 0.0),) * 3, segs=H.rect(0.5, 0, 5.0, 3.0),
                         laser_x=0.100))
    res = run(srv._wall_spans())
    front = min(res['walls'], key=lambda w: abs(w['bearing_rad']))
    assert front['distance_m'] == pytest.approx(3.0 + 0.027, abs=0.004)
    assert not any('URDF constants' in f for f in res['flags'])


def test_fallback_flagged_when_static_tf_missing(srv, rb):
    rb.reset(parked_feed(robots=((0.0, 0.0, 0.0),) * 3, segs=H.rect(0.5, 0, 5.0, 3.0),
                         static=False))
    res = run(srv._wall_spans())
    front = min(res['walls'], key=lambda w: abs(w['bearing_rad']))
    assert abs(front['bearing_rad']) < 0.01 and front['distance_m'] == pytest.approx(3.0, abs=0.01)
    assert any('URDF constants' in f for f in res['flags'])


def test_include_points_is_capped_and_optional(srv, rb):
    rb.reset(parked_feed())
    assert 'points_xy' not in run(srv._wall_spans())
    pts = run(srv._wall_spans(include_points=True))['points_xy']
    assert 0 < len(pts) <= 1800
    assert all(len(p) == 2 and round(p[0], 3) == p[0] for p in pts)


def test_mcp_tool_returns_the_helper_dict(srv, rb):
    rb.reset(parked_feed())
    assert srv.mcp.tools['wall_spans'] == {'annotations': {'readOnlyHint': True}}
    assert run(srv.wall_spans(scans=3)) == run(srv._wall_spans(scans=3))


def test_moving_by_odom_refuses_409_and_tool_raises(srv, rb):
    rb.reset(parked_feed(odoms=[odom(0.3), odom(0.3)]))
    with pytest.raises(srv._WallSpansError) as e:
        run(srv._wall_spans())
    assert e.value.status == 409 and 'robot moving' in str(e.value)
    assert 'require_stationary=False' in str(e.value)
    with pytest.raises(ToolError, match='robot moving'):
        run(srv.wall_spans())
    resp = get(srv)
    assert resp.status_code == 409 and 'robot moving' in json.loads(resp.body)['error']


def test_rotating_odom_also_refuses(srv, rb):
    rb.reset(parked_feed(odoms=[odom(0.0, 0.5)]))
    with pytest.raises(srv._WallSpansError) as e:
        run(srv._wall_spans())
    assert e.value.status == 409


def test_moving_by_scan_agreement_refuses_even_with_quiet_odom(srv, rb):
    slid = ((0.4, -0.3, YAW20), (0.4, -0.3, YAW20), (0.65, -0.3, YAW20))
    rb.reset(parked_feed(robots=slid))
    with pytest.raises(srv._WallSpansError) as e:
        run(srv._wall_spans())
    assert e.value.status == 409 and 'scan-to-scan' in str(e.value)


def test_require_stationary_false_measures_and_flags(srv, rb):
    rb.reset(parked_feed(odoms=[odom(0.3)]))
    res = run(srv._wall_spans(require_stationary=False))
    assert res['stationary'] is False and res['ok'] is True
    assert any('robot moving' in f for f in res['flags'])


def test_stationarity_unverifiable_is_refused(srv, rb):
    rb.reset(parked_feed(robots=((0.4, -0.3, YAW20),), odoms=[]))
    with pytest.raises(srv._WallSpansError) as e:
        run(srv._wall_spans(scans=1))
    assert e.value.status == 409 and 'unverifiable' in str(e.value)


def test_single_scan_with_odom_is_flagged_but_allowed(srv, rb):
    rb.reset(parked_feed(robots=((0.4, -0.3, YAW20),)))
    res = run(srv._wall_spans(scans=1))
    assert res['stationary'] is True and res['scans_used'] == 1
    assert any('scan agreement unavailable' in f for f in res['flags'])


def test_fewer_scans_than_asked_is_flagged(srv, rb):
    rb.reset(parked_feed(robots=((0.4, -0.3, YAW20),) * 2))
    res = run(srv._wall_spans(scans=5))
    assert res['scans_used'] == 2 and any('only 2/5 scans' in f for f in res['flags'])


def test_no_scan_traffic_is_503(srv, rb):
    rb.reset({'/odom': [odom()]})
    with pytest.raises(srv._WallSpansError) as e:
        run(srv._wall_spans())
    assert e.value.status == 503 and '/scan' in str(e.value)
    with pytest.raises(ToolError, match='/scan'):
        run(srv.wall_spans())
    assert get(srv).status_code == 503


@pytest.mark.parametrize('error', [
    RosBridgeError('connect failed twice'), RosBridgeBusy('busy'), OSError('refused'),
    WebSocketException('closed'),
])
def test_rosbridge_failures_are_503(srv, rb, error):
    rb.reset(parked_feed(), enter_error=error)
    resp = get(srv)
    assert resp.status_code == 503 and 'rosbridge unavailable' in json.loads(resp.body)['error']
    assert header(resp, 'access-control-allow-origin') == '*'


@pytest.mark.parametrize('kwargs', [
    {'scans': 0}, {'scans': 11}, {'max_range_m': float('nan')}, {'max_range_m': 0.1},
    {'max_range_m': 99.0}, {'min_length_m': 0.0}, {'min_length_m': float('inf')},
])
def test_bad_helper_params_are_400_before_touching_rosbridge(srv, rb, kwargs):
    rb.reset(parked_feed())
    with pytest.raises(srv._WallSpansError) as e:
        run(srv._wall_spans(**kwargs))
    assert e.value.status == 400 and rb.entered == 0
    with pytest.raises(ToolError):
        run(srv.wall_spans(**kwargs))


@pytest.mark.parametrize('query', [
    {'scans': 'abc'}, {'scans': '2.5'}, {'scans': '99'}, {'max_range_m': 'far'},
    {'min_length_m': 'nan'}, {'include_points': 'maybe'}, {'require_stationary': '2'},
    {'scan': '3'}, {'scans': ''},
])
def test_bad_rest_params_are_400_json(srv, rb, query):
    rb.reset(parked_feed())
    resp = get(srv, query)
    assert resp.status_code == 400 and list(json.loads(resp.body)) == ['error']
    assert header(resp, 'access-control-allow-origin') == '*' and rb.entered == 0


def test_rest_get_serves_the_contract_with_cors(srv, rb):
    rb.reset(parked_feed())
    resp = get(srv, {'scans': '3', 'max_range_m': '8', 'min_length_m': '0.8',
                     'require_stationary': 'true', 'include_points': '1'})
    body = json.loads(resp.body)
    assert resp.status_code == 200 and header(resp, 'access-control-allow-origin') == '*'
    assert list(body)[:len(CONTRACT_KEYS)] == CONTRACT_KEYS and 'points_xy' in body
    assert sorted(round(p['span_m'], 2) for p in body['pairs']) == [3.0, 4.0]


def test_rest_defaults_match_the_tool_defaults(srv, rb):
    rb.reset(parked_feed())
    assert json.loads(get(srv).body) == run(srv.wall_spans())


def test_options_answers_with_cors_and_no_body(srv, rb):
    rb.reset(parked_feed())
    resp = get(srv, method='OPTIONS')
    assert resp.status_code == 204 and resp.body == b'' and rb.entered == 0
    assert header(resp, 'access-control-allow-origin') == '*'
    assert header(resp, 'access-control-allow-methods') == 'GET, OPTIONS'


def test_route_registration_is_get_and_options_only(srv):
    assert [(p, m) for p, m, _ in srv.mcp.routes] == [('/api/wall_spans', ['GET', 'OPTIONS'])]
    assert 'POST' not in srv.mcp.routes[0][1]


def test_cors_is_route_level_never_app_wide_or_on_mcp(srv):
    assert srv.mcp.middleware_calls == []
    tree = ast.parse(SERVER.read_text())
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    names |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    kwargs = {k.arg for n in ast.walk(tree) if isinstance(n, ast.Call) for k in n.keywords}
    assert not {'CORSMiddleware', 'add_middleware'} & names
    assert 'middleware' not in kwargs
    text = SERVER.read_text()
    start = text[:text.index('# --- wall_spans')].count('\n')
    end = text[:text.index('# --- RFID')].count('\n')
    cors_lines = [i for i, line in enumerate(text.splitlines()) if 'Access-Control-' in line]
    assert cors_lines and all(start <= i < end for i in cors_lines)
