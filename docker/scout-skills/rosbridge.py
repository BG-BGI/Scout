"""Minimal async rosbridge v2 JSON client.

One websocket per tool call: rosbridge preserves op order within a
connection, and a fresh socket means a rosbridge restart can never strand
this server with a dead subscription it thinks is live.
"""

import asyncio
import itertools
import json
import os

import websockets

ROSBRIDGE_URL = os.environ.get("ROSBRIDGE_URL", "ws://127.0.0.1:9090")

# DDS discovery delay between rosbridge creating a publisher and bt_navigator
# matching it. Publishing before the match is the same silent-loss trap as
# `ros2 topic pub --once` from a throwaway container — the frame goes nowhere
# and nothing reports it.
ADVERTISE_SETTLE_S = 0.7

_ids = itertools.count(1)


class RosBridgeError(RuntimeError):
    pass


class RosBridge:
    def __init__(self, url: str = ROSBRIDGE_URL):
        self._url = url
        self._ws = None

    async def __aenter__(self):
        # max_size=None: a house-sized OccupancyGrid serializes to several MB
        # of JSON, past websockets' 1 MB default frame cap.
        self._ws = await websockets.connect(
            self._url, max_size=None, open_timeout=5
        )
        return self

    async def __aexit__(self, *exc):
        await self._ws.close()

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
