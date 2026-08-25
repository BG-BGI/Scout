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


# One connection at a time, process-wide. rosbridge serializes big frames
# (a raw D455 image is ~1 MB of base64 JSON) on its side, and a handshake
# that arrives while it's mid-frame starves past open_timeout — observed as
# parallel camera_snapshot + rotate calls failing with "timed out during
# opening handshake". Serializing here means a tool waits its turn instead
# of erroring; the cost is nav_status polls queue behind a camera pull.
_conn_lock = asyncio.Lock()


class RosBridge:
    def __init__(self, url: str = ROSBRIDGE_URL):
        self._url = url
        self._ws = None

    async def __aenter__(self):
        await _conn_lock.acquire()
        try:
            # max_size=None: a house-sized OccupancyGrid serializes to several MB
            # of JSON, past websockets' 1 MB default frame cap.
            for attempt in (1, 2):
                try:
                    self._ws = await websockets.connect(
                        self._url, max_size=None, open_timeout=10
                    )
                    break
                except (OSError, TimeoutError) as e:
                    if attempt == 2:
                        raise RosBridgeError(
                            "rosbridge websocket connect failed twice "
                            f"(is the rosbridge container up?): {e!r}"
                        ) from e
                    await asyncio.sleep(0.5)
        except BaseException:
            _conn_lock.release()
            raise
        return self

    async def __aexit__(self, *exc):
        try:
            await self._ws.close()
        finally:
            _conn_lock.release()

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
