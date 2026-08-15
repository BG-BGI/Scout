"""OccupancyGrid → PNG for a vision model.

Row 0 of an OccupancyGrid is the bottom row in world coords (same flip the
webui does), so the array is flipped vertically before drawing; the robot
triangle is drawn in image space with y down.
"""

import math
from io import BytesIO

import numpy as np
from PIL import Image, ImageDraw

from robot_profile import load as _load_profile

MAX_DIM = 1024
# nav2's lethal convention; webui uses the same split (robot_profile.yaml SSOT).
OCCUPIED_THRESHOLD = _load_profile()["occupied_threshold"]


def render_map(msg: dict, robot: dict | None) -> tuple[bytes, int]:
    """PNG bytes + the integer upscale factor (needed by the pixel→world
    formula in the tool's metadata)."""
    info = msg["info"]
    w, h, res = info["width"], info["height"], info["resolution"]
    grid = np.array(msg["data"], dtype=np.int16).reshape(h, w)

    img = np.full((h, w), 128, dtype=np.uint8)  # unknown: mid gray
    img[(grid >= 0) & (grid < OCCUPIED_THRESHOLD)] = 255  # free: white
    img[grid >= OCCUPIED_THRESHOLD] = 0  # occupied: black
    img = np.flipud(img)

    scale = max(1, min(MAX_DIM // max(w, h), 8))
    pil = Image.fromarray(img, "L").convert("RGB")
    if scale > 1:
        pil = pil.resize((w * scale, h * scale), Image.NEAREST)

    if robot is not None:
        _draw_robot(pil, info, scale, robot)

    buf = BytesIO()
    pil.save(buf, "PNG")
    return buf.getvalue(), scale


def _draw_robot(pil: Image.Image, info: dict, scale: int, robot: dict) -> None:
    res = info["resolution"]
    ox = info["origin"]["position"]["x"]
    oy = info["origin"]["position"]["y"]
    px = (robot["x"] - ox) / res * scale
    py = pil.height - (robot["y"] - oy) / res * scale
    # Triangle roughly the chassis' 0.34 m envelope, floor of 8 px so it stays
    # visible on coarse grids. Image y is down, so world yaw negates dy.
    size = max(8.0, 0.34 / res * scale)
    yaw = robot["yaw"]
    pts = []
    for ang, r in ((0, size), (2.5, size * 0.6), (-2.5, size * 0.6)):
        pts.append(
            (
                px + r * math.cos(yaw + ang),
                py - r * math.sin(yaw + ang),
            )
        )
    ImageDraw.Draw(pil).polygon(pts, fill=(220, 30, 30))
