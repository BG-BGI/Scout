"""Minimal async rosbridge v2 JSON client (trimmed copy of
docker/scout-skills/rosbridge.py — only what this server's tools need).
"""

import asyncio
import itertools
import json
import os

import websockets

ROSBRIDGE_URL = os.environ.get("ROSBRIDGE_URL", "ws://127.0.0.1:9090")

_ids = itertools.count(1)


class RosBridge:
    def __init__(self, url: str = ROSBRIDGE_URL):
        self._url = url
        self._ws = None

    async def __aenter__(self):
        self._ws = await websockets.connect(self._url, max_size=None, open_timeout=5)
        return self

    async def __aexit__(self, *exc):
        await self._ws.close()

    async def subscribe_collect(
        self, topic: str, msg_type: str, duration: float = 2.0
    ) -> list[dict]:
        sid = f"sub:{next(_ids)}"
        await self._ws.send(
            json.dumps({"op": "subscribe", "id": sid, "topic": topic, "type": msg_type})
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
        await self._ws.send(json.dumps({"op": "unsubscribe", "id": sid, "topic": topic}))
        return msgs
