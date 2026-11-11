"""docker/scout-skills/navpose.py: pose parsing shared by nav_status/go_to, and
the one-window collect_multi in rosbridge.py (loaded by path, stub websockets)."""

import asyncio
import importlib.util
import json
import math
import sys
import types
from pathlib import Path

import pytest

SKILLS = Path(__file__).resolve().parents[2] / 'docker' / 'scout-skills'


def load(name, monkeypatch=None):
    sys.path.insert(0, str(SKILLS))   # navpose imports geometry (sibling module)
    try:
        spec = importlib.util.spec_from_file_location(name + '_t', SKILLS / (name + '.py'))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name + '_t'] = mod
        spec.loader.exec_module(mod)
        return mod
    finally:
        sys.path.remove(str(SKILLS))


def tf(frame, child, x, y, yaw):
    return {'transforms': [{
        'header': {'frame_id': frame}, 'child_frame_id': child,
        'transform': {
            'translation': {'x': x, 'y': y, 'z': 0.0},
            'rotation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)},
        },
    }]}


def test_pose_of_planar():
    navpose = load('navpose')
    msg = {'pose': {'pose': {
        'position': {'x': 1.23456, 'y': -2.0},
        'orientation': {'z': math.sin(0.25), 'w': math.cos(0.25)},
    }}}
    assert navpose.pose_of(msg) == {'x': 1.235, 'y': -2.0, 'yaw': 0.5}
    assert navpose.pose_of(None) is None


def test_pose_from_tf_composes_map_odom_base():
    navpose = load('navpose')
    msgs = [tf('map', 'odom', 10.0, 0.0, math.pi / 2), tf('odom', 'base_link', 1.0, 0.0, 0.0)]
    got = navpose.pose_from_tf(msgs)
    # odom is rotated 90 deg in map: base at odom (1,0) -> map (10, 1), heading +90
    assert got == {'x': 10.0, 'y': 1.0, 'yaw': round(math.pi / 2, 3)}


def test_pose_from_tf_needs_both_links():
    navpose = load('navpose')
    assert navpose.pose_from_tf([tf('odom', 'base_link', 1.0, 0.0, 0.0)]) is None
    assert navpose.pose_from_tf([tf('map', 'odom', 1.0, 0.0, 0.0)]) is None
    assert navpose.pose_from_tf([]) is None


class QueueWs:
    """recv() yields queued frames (each after `gap` s), then blocks forever."""

    def __init__(self, frames, gap=0.0):
        self.frames, self.gap, self.sent = list(frames), gap, []

    async def send(self, raw):
        self.sent.append(json.loads(raw))

    async def recv(self):
        if not self.frames:
            await asyncio.sleep(3600)
        await asyncio.sleep(self.gap)
        return json.dumps(self.frames.pop(0))

    async def close(self):
        pass


@pytest.fixture
def rosbridge(monkeypatch):
    stub = types.ModuleType('websockets')
    monkeypatch.setitem(sys.modules, 'websockets', stub)
    return load('rosbridge')


def pub(topic, msg):
    return {'op': 'publish', 'topic': topic, 'msg': msg}


# rosbridge.py runs on the scout-skills image (py3.12) and uses asyncio.timeout (3.11+).
# The Humble-parity CI job is py3.10, so these run in ci.yml's `skills-tests` job instead.
needs_asyncio_timeout = pytest.mark.skipif(
    not hasattr(asyncio, 'timeout'), reason='asyncio.timeout needs Python 3.11+')


@needs_asyncio_timeout
def test_collect_multi_routes_by_topic_and_unsubscribes(rosbridge):
    ws = QueueWs([pub('/a', 1), pub('/b', 2), pub('/a', 3), pub('/ignored', 9)])
    rb = rosbridge.RosBridge()
    rb._ws = ws
    got = asyncio.run(rb.collect_multi({'/a': 'T', '/b': 'T', '/c': 'T'}, duration=0.2))
    assert got == {'/a': [1, 3], '/b': [2], '/c': []}
    ops = [f['op'] for f in ws.sent]
    assert ops.count('subscribe') == 3 and ops.count('unsubscribe') == 3


@needs_asyncio_timeout
def test_collect_multi_uses_one_shared_window(rosbridge):
    import time
    ws = QueueWs([])
    rb = rosbridge.RosBridge()
    rb._ws = ws
    t0 = time.monotonic()
    asyncio.run(rb.collect_multi({f'/t{i}': 'T' for i in range(6)}, duration=0.3))
    # six silent topics cost ONE window, not six
    assert time.monotonic() - t0 < 0.6
