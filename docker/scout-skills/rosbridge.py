"""Minimal async rosbridge v2 JSON client.

One websocket per tool call: rosbridge preserves op order within a
connection, and a fresh socket means a rosbridge restart can never strand
this server with a dead subscription it thinks is live.

Concurrency: calls run in parallel; only the websocket *handshake* is
serialized, and a ``heavy=True`` call (a ~1 MB camera frame, a multi-MB map)
holds that gate for its whole life so no handshake lands mid-frame. A
``background=True`` caller (the AprilTag watcher) never waits: it raises
``RosBridgeBusy`` whenever a heavy call or any foreground call is in flight.
"""

import asyncio
import itertools
import json
import os

import websockets

ROSBRIDGE_URL = os.environ.get("ROSBRIDGE_URL", "ws://127.0.0.1:9090")
# Handshake timeout per attempt (two attempts). Fail fast: a healthy rosbridge
# accepts in milliseconds; a stalled one should not hold a tool for ~20 s.
OPEN_TIMEOUT_S = float(os.environ.get("ROSBRIDGE_OPEN_TIMEOUT", "5"))

# DDS discovery delay between rosbridge creating a publisher and bt_navigator
# matching it. Publishing before the match is the same silent-loss trap as
# `ros2 topic pub --once` from a throwaway container — the frame goes nowhere
# and nothing reports it.
ADVERTISE_SETTLE_S = 0.7

_ids = itertools.count(1)


class RosBridgeError(RuntimeError):
    pass


class RosBridgeBusy(RosBridgeError):
    """A background caller yielded: foreground work (or a heavy transfer) is
    in flight. Not a failure — skip this cycle and try the next."""


# Handshake gate, process-wide. rosbridge serializes big frames (a raw D455
# image is ~1 MB of base64 JSON) on its side, and a handshake that arrives
# while it's mid-frame starves past open_timeout — observed as parallel
# camera_snapshot + rotate calls failing with "timed out during opening
# handshake". So the gate is held (a) briefly, around each connect, and (b)
# for the whole life of a heavy call, which keeps new handshakes out while a
# big frame streams. Light calls — nav_status, list_*, pose reads — no longer
# queue behind each other or behind the tag watcher: they run concurrently.
_gate = asyncio.Lock()
# Foreground (non-background) contexts currently open. The tag watcher yields
# while this is non-zero so it never competes with a tool call.
_foreground = 0


def foreground_in_flight() -> int:
    return _foreground


class RosBridge:
    def __init__(
        self,
        url: str = ROSBRIDGE_URL,
        *,
        heavy: bool = False,
        background: bool = False,
    ):
        self._url = url
        self._ws = None
        self._heavy = heavy
        self._background = background
        self._holds_gate = False
        self._counted = False

    async def __aenter__(self):
        global _foreground
        if self._background and (_gate.locked() or _foreground):
            raise RosBridgeBusy("foreground rosbridge work in flight")
        # Uncontended Lock.acquire() returns without yielding, so the
        # background check above cannot be raced between check and acquire.
        await _gate.acquire()
        held = True
        try:
            # max_size=None: a house-sized OccupancyGrid serializes to several MB
            # of JSON, past websockets' 1 MB default frame cap.
            for attempt in (1, 2):
                try:
                    self._ws = await websockets.connect(
                        self._url, max_size=None, open_timeout=OPEN_TIMEOUT_S
                    )
                    break
                except (OSError, TimeoutError) as e:
                    if attempt == 2:
                        raise RosBridgeError(
                            "rosbridge websocket connect failed twice "
                            f"(is the rosbridge container up?): {e!r}"
                        ) from e
                    await asyncio.sleep(0.5)
            if not self._heavy:
                _gate.release()
                held = False
            if not self._background:
                _foreground += 1
                self._counted = True
        except BaseException:
            if held:
                _gate.release()
            raise
        self._holds_gate = held
        return self

    async def __aexit__(self, *exc):
        global _foreground
        try:
            await self._ws.close()
        finally:
            if self._counted:
                _foreground -= 1
            if self._holds_gate:
                _gate.release()

    async def _send(self, frame: dict):
        await self._ws.send(json.dumps(frame))

    async def _recv_until(self, pred, timeout: float):
        try:
            async with asyncio.timeout(timeout):
                while True:
                    frame = json.loads(await self._ws.recv())
                    if pred(frame):
                        return frame
        except TimeoutError:
            return None

    async def subscribe_once(self, topic: str, msg_type: str, timeout: float = 5.0):
        """First message on `topic`, or None on timeout.

        A periodic publisher needs timeout > its period (/map is 0.5 Hz). A
        transient_local publisher only replays its last message if rosbridge's
        QoS inference matched durability — so treat None as "no traffic in the
        window", never "topic absent".
        """
        sid = f"sub:{next(_ids)}"
        await self._send(
            {
                "op": "subscribe",
                "id": sid,
                "topic": topic,
                "type": msg_type,
                "queue_length": 1,
            }
        )
        frame = await self._recv_until(
            lambda f: f.get("op") == "publish" and f.get("topic") == topic,
            timeout,
        )
        await self._send({"op": "unsubscribe", "id": sid, "topic": topic})
        return frame["msg"] if frame else None

    async def collect_multi(
        self, specs: dict[str, str], duration: float
    ) -> dict[str, list[dict]]:
        """Subscribe to several topics at once on this one socket and gather
        every message from each for ``duration`` seconds. One shared window
        replaces N back-to-back windows (nav_status used to spend ~6.5 s
        waiting out six of them in sequence). ``specs`` maps topic -> type."""
        sids = {topic: f"sub:{next(_ids)}" for topic in specs}
        for topic, msg_type in specs.items():
            await self._send(
                {"op": "subscribe", "id": sids[topic], "topic": topic, "type": msg_type}
            )
        got: dict[str, list[dict]] = {topic: [] for topic in specs}
        try:
            async with asyncio.timeout(duration):
                while True:
                    frame = json.loads(await self._ws.recv())
                    if frame.get("op") == "publish" and frame.get("topic") in got:
                        got[frame["topic"]].append(frame["msg"])
        except TimeoutError:
            pass
        for topic, sid in sids.items():
            await self._send({"op": "unsubscribe", "id": sid, "topic": topic})
        return got

    async def subscribe_collect(
        self, topic: str, msg_type: str, duration: float = 0.8
    ) -> list[dict]:
        """Every message on `topic` for `duration` seconds. For topics where
        one message is not the whole picture — /tf carries a different
        transform subset per publisher."""
        sid = f"sub:{next(_ids)}"
        await self._send(
            {"op": "subscribe", "id": sid, "topic": topic, "type": msg_type}
        )
        msgs: list[dict] = []
        try:
            async with asyncio.timeout(duration):
                while True:
                    frame = json.loads(await self._ws.recv())
                    if frame.get("op") == "publish" and frame.get("topic") == topic:
                        msgs.append(frame["msg"])
        except TimeoutError:
            pass
        await self._send({"op": "unsubscribe", "id": sid, "topic": topic})
        return msgs

    async def advertise(self, topic: str, msg_type: str):
        """Persistent advertise for streaming publishers (cmd_vel loops).
        Caller waits ADVERTISE_SETTLE_S before the first publish_raw; the
        socket close tears the publisher down."""
        await self._send(
            {
                "op": "advertise",
                "id": f"adv:{next(_ids)}",
                "topic": topic,
                "type": msg_type,
            }
        )

    async def publish_raw(self, topic: str, msg: dict):
        """Publish on an already-advertised topic. No settle, no teardown."""
        await self._send({"op": "publish", "topic": topic, "msg": msg})

    async def subscribe(self, topic: str, msg_type: str) -> str:
        """Persistent subscription; drain with recv_msg. queue_length 1 keeps
        only the latest sample, which is what a control loop wants."""
        sid = f"sub:{next(_ids)}"
        await self._send(
            {
                "op": "subscribe",
                "id": sid,
                "topic": topic,
                "type": msg_type,
                "queue_length": 1,
            }
        )
        return sid

    async def recv_msg(self, topic: str, timeout: float):
        """Next message on a persistent subscription, or None on timeout."""
        frame = await self._recv_until(
            lambda f: f.get("op") == "publish" and f.get("topic") == topic,
            timeout,
        )
        return frame["msg"] if frame else None

    async def send_action_goal(self, action: str, action_type: str, args: dict):
        """Fire-and-forget ROS action goal (op: send_action_goal), no wait for
        action_result. rosbridge's SendActionGoal capability has no finish(),
        so the goal keeps running after this socket closes — the deliberate
        goal-outlives-client contract the nav tools use. Cancel from any
        socket via the action's _action/cancel_goal service. Callers confirm
        acceptance on the action's _action/status topic (subscribe FIRST,
        then send, so the accept transition cannot race the subscription)."""
        await self._send(
            {
                "op": "send_action_goal",
                "id": f"act:{next(_ids)}",
                "action": action,
                "action_type": action_type,
                "args": args,
                "feedback": False,
            }
        )

    async def publish(self, topic: str, msg_type: str, msg: dict):
        await self._send(
            {
                "op": "advertise",
                "id": f"adv:{next(_ids)}",
                "topic": topic,
                "type": msg_type,
            }
        )
        await asyncio.sleep(ADVERTISE_SETTLE_S)
        await self._send({"op": "publish", "topic": topic, "msg": msg})
        # Let the publish land before unadvertise tears the publisher down.
        await asyncio.sleep(0.2)
        await self._send({"op": "unadvertise", "topic": topic})

    async def call_service(
        self, service: str, srv_type: str, args: dict | None = None, timeout: float = 5.0
    ):
        cid = f"srv:{next(_ids)}"
        await self._send(
            {
                "op": "call_service",
                "id": cid,
                "service": service,
                "type": srv_type,
                "args": args or {},
            }
        )
        frame = await self._recv_until(
            lambda f: f.get("op") == "service_response" and f.get("id") == cid,
            timeout,
        )
        if frame is None:
            raise RosBridgeError(f"{service} did not respond within {timeout} s")
        if not frame.get("result", True):
            raise RosBridgeError(f"{service} failed: {frame.get('values')}")
        return frame.get("values", {})
