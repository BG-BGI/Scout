"""docker/scout-skills/rosbridge.py concurrency model: parallel light calls,
heavy calls that fence off new handshakes, and a background caller that yields.

Loaded by path (separate container) with a stub `websockets` so no server or
extra dependency is needed. A fresh module per test: its asyncio.Lock binds to
the first loop that contends it, and every test runs in its own loop.
"""

import asyncio
import importlib.util
import sys
import types
from pathlib import Path

import pytest

ROSBRIDGE_PY = (
    Path(__file__).resolve().parents[2] / 'docker' / 'scout-skills' / 'rosbridge.py'
)


class FakeWs:
    async def close(self):
        pass


@pytest.fixture
def rb(monkeypatch):
    state = types.SimpleNamespace(
        delay=0.0, connecting=0, max_connecting=0, fail=False, connects=0
    )

    async def connect(url, max_size=None, open_timeout=None):
        state.connects += 1
        state.connecting += 1
        state.max_connecting = max(state.max_connecting, state.connecting)
        try:
            await asyncio.sleep(state.delay)
            if state.fail:
                raise OSError('refused')
            return FakeWs()
        finally:
            state.connecting -= 1

    stub = types.ModuleType('websockets')
    stub.connect = connect
    monkeypatch.setitem(sys.modules, 'websockets', stub)
    spec = importlib.util.spec_from_file_location('rosbridge_under_test', ROSBRIDGE_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['rosbridge_under_test'] = mod
    spec.loader.exec_module(mod)
    mod.state = state
    return mod


def run(coro):
    return asyncio.run(asyncio.wait_for(coro, 5))


def test_light_calls_overlap(rb):
    async def go():
        async with rb.RosBridge():
            async with rb.RosBridge():
                return rb.foreground_in_flight()

    # Before the change the second open would deadlock behind the first.
    assert run(go()) == 2
    assert rb.foreground_in_flight() == 0


def test_handshakes_never_interleave(rb):
    rb.state.delay = 0.02

    async def one():
        async with rb.RosBridge():
            await asyncio.sleep(0.05)

    async def go():
        await asyncio.gather(*(one() for _ in range(5)))

    run(go())
    assert rb.state.max_connecting == 1


def test_heavy_fences_new_handshakes_until_it_exits(rb):
    order = []

    async def heavy():
        async with rb.RosBridge(heavy=True):
            order.append('heavy-in')
            await asyncio.sleep(0.1)
            order.append('heavy-out')

    async def light():
        await asyncio.sleep(0.02)  # start after heavy holds the gate
        async with rb.RosBridge():
            order.append('light-in')

    async def go():
        await asyncio.gather(heavy(), light())

    run(go())
    assert order == ['heavy-in', 'heavy-out', 'light-in']


def test_background_yields_to_foreground_and_heavy(rb):
    async def go():
        async with rb.RosBridge():
            with pytest.raises(rb.RosBridgeBusy):
                async with rb.RosBridge(background=True):
                    pass
        async with rb.RosBridge(heavy=True):
            with pytest.raises(rb.RosBridgeBusy):
                async with rb.RosBridge(background=True):
                    pass

    run(go())
    assert rb.foreground_in_flight() == 0


def test_background_runs_when_idle_and_never_blocks_foreground(rb):
    async def go():
        async with rb.RosBridge(background=True):
            assert rb.foreground_in_flight() == 0
            # a tool call arriving mid-scan is not made to wait
            async with rb.RosBridge():
                return rb.foreground_in_flight()

    assert run(go()) == 1


def test_failed_connect_releases_gate_and_counter(rb):
    rb.state.fail = True

    async def go():
        with pytest.raises(rb.RosBridgeError):
            async with rb.RosBridge():
                pass
        rb.state.fail = False
        async with rb.RosBridge(heavy=True):
            return rb.foreground_in_flight()

    assert run(go()) == 1
    assert rb.foreground_in_flight() == 0
