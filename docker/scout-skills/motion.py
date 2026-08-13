"""Closed-loop relative motion — move(distance) / rotate(angle) over rosbridge.

Text commands like "forward one meter" or "turn around" have no map goal, so
they bypass Nav2: stream /cmd_vel directly and measure progress on the fused
/odom (yaw is gyro-through-EKF, so rotation measurement is scrub-immune;
distance is the wheel odometry Nav2 itself trusts). Publishing is paced by
/odom receipt (~30 Hz — inside the driver's 20–50 Hz band, well above the
5 Hz deadman); the loop always ends with explicit zero Twists because the
deadman alone only free-wheels to a coast.

No obstacle avoidance here — the caller owns clearance.
"""

import asyncio
import math
import time

from rosbridge import ADVERTISE_SETTLE_S, RosBridge

CMD_VEL = "/cmd_vel"
ODOM = "/odom"
TWIST_TYPE = "geometry_msgs/msg/Twist"

# Velocity-loop quantization floor .. reachable-at-cutoff ceiling.
LIN_FLOOR, LIN_CAP = 0.05, 1.0
# DWB's min_speed_theta .. max_angular_velocity.
ANG_FLOOR, ANG_CAP = 0.35, 3.0
MAX_MOVE_M = 5.0
MAX_ROTATE_RAD = 2 * math.pi
# Taper spans: full speed outside, linear down to the floor inside.
LIN_TAPER_M = 0.25
ANG_TAPER_RAD = 0.6
ODOM_SILENCE_S = 1.0


def _zero_twist() -> dict:
    return {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


async def _closed_loop(goal: float, measure, twist_of, timeout: float):
    """Drive until measure(odom_msg) reaches `goal` (both in the commanded
    direction's positive sense). twist_of(remaining) shapes the command.
    Returns (progress, reason)."""
    progress, reason = 0.0, "arrived"
    async with RosBridge() as rb:
        await rb.advertise(CMD_VEL, TWIST_TYPE)
        await rb.subscribe(ODOM, "nav_msgs/msg/Odometry")
        await asyncio.sleep(ADVERTISE_SETTLE_S)
        deadline = time.monotonic() + timeout
        try:
            while True:
                msg = await rb.recv_msg(ODOM, timeout=ODOM_SILENCE_S)
                if msg is None:
                    reason = f"aborted: /odom silent for {ODOM_SILENCE_S} s"
                    break
                progress = measure(msg)
                if progress >= goal:
                    break
                if time.monotonic() > deadline:
                    reason = "aborted: watchdog timeout"
                    break
                await rb.publish_raw(CMD_VEL, twist_of(goal - progress))
        finally:
            # Explicit stop, repeated — a lost single frame would leave the
            # last real command latched until the deadman coasts it out.
            for _ in range(3):
                await rb.publish_raw(CMD_VEL, _zero_twist())
                await asyncio.sleep(0.04)
    return progress, reason


async def run_move(distance_m: float, speed: float) -> dict:
    direction = 1.0 if distance_m >= 0 else -1.0
    goal = abs(distance_m)
    speed = min(max(speed, LIN_FLOOR), LIN_CAP)
    start: dict = {}

    def measure(msg: dict) -> float:
        p = msg["pose"]["pose"]["position"]
        if not start:
            start["x"], start["y"] = p["x"], p["y"]
        return math.hypot(p["x"] - start["x"], p["y"] - start["y"])

    def twist_of(remaining: float) -> dict:
        v = min(speed, max(LIN_FLOOR, speed * remaining / LIN_TAPER_M))
        t = _zero_twist()
        t["linear"]["x"] = direction * v
        return t

    progress, reason = await _closed_loop(
        goal, measure, twist_of, timeout=goal / speed * 3 + 3
    )
    return {
        "commanded_m": distance_m,
        "achieved_m": round(direction * progress, 3),
        "result": reason,
    }


async def run_rotate(angle_rad: float, speed: float) -> dict:
    direction = 1.0 if angle_rad >= 0 else -1.0
    goal = abs(angle_rad)
    speed = min(max(speed, ANG_FLOOR), ANG_CAP)
    state = {"prev": None, "acc": 0.0}

    def measure(msg: dict) -> float:
        q = msg["pose"]["pose"]["orientation"]
        yaw = 2 * math.atan2(q["z"], q["w"])  # planar quaternion shortcut
        if state["prev"] is not None:
            d = yaw - state["prev"]
            state["acc"] += (d + math.pi) % (2 * math.pi) - math.pi
        state["prev"] = yaw
        return direction * state["acc"]

    def twist_of(remaining: float) -> dict:
        w = min(speed, max(ANG_FLOOR, speed * remaining / ANG_TAPER_RAD))
        t = _zero_twist()
        t["angular"]["z"] = direction * w
        return t

    progress, reason = await _closed_loop(
        goal, measure, twist_of, timeout=goal / speed * 3 + 3
    )
    return {
        "commanded_rad": angle_rad,
        "achieved_rad": round(direction * progress, 3),
        "result": reason,
    }
