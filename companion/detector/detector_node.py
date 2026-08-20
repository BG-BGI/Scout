#!/usr/bin/env python3
"""World-model object detector (companion, gate 1 of WORLDMODEL.md).

Continuous YOLO11n on the bridged D455 stream -> map-frame object table,
published as /world/objects (std_msgs/String JSON, latched). CPU-only,
onnxruntime — no torch (the .onnx is baked in a throwaway build stage, same as
scout-skills).

Runs on the companion's LOCAL DDS graph: color/depth/info/tf all arrive over
the zenoh bridge (already in both allowlists). Getting /world/objects TO the Pi
is gate 2 (a bridge allowlist edit); this node is gate-1 local only. The Pi is
unaware of it and unaffected (Pi-standalone contract, spec §0.7).

Reuses scout-skills' verified deprojection math: optical-frame ray through the
box centre at median box depth, then TF cam->map. tf2 here (native ROS on the
companion) replaces scout-skills' hand-rolled TfTree.
"""
import json
import math
from io import BytesIO

import numpy as np
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException

from detect import detect  # verbatim reuse from scout-skills


def _decode_color(msg: CompressedImage) -> np.ndarray:
    """JPEG CompressedImage -> RGB uint8. PIL reads the JPEG directly; the
    'rgb8; jpeg compressed bgr8' format string is metadata we don't need."""
    return np.asarray(PILImage.open(BytesIO(bytes(msg.data))).convert("RGB"))


def _decode_depth(msg: CompressedImage) -> np.ndarray | None:
    """compressedDepth CompressedImage -> uint16 mm array.

    ⚠ The payload is a compressed_depth_image_transport ConfigHeader (enum +
    two float params, 12 bytes) FOLLOWED by a 16-bit PNG. Slicing a fixed 12
    is fragile across ROS versions, so seek the PNG magic instead. 16UC1 depth
    decodes as PIL mode 'I;16' (millimetres)."""
    data = bytes(msg.data)
    i = data.find(b"\x89PNG")
    if i < 0:
        return None
    img = PILImage.open(BytesIO(data[i:]))
    return np.asarray(img).astype(np.uint16)


def _median_depth_m(depth: np.ndarray, box) -> float | None:
    """Median valid depth over the central half of the box, metres. Zeros are
    no-return; <20 valid px = effectively no depth (out of band / all holes)."""
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    qw, qh = (x2 - x1) / 4, (y2 - y1) / 4
    patch = depth[
        max(0, round(cy - qh)) : round(cy + qh) + 1,
        max(0, round(cx - qw)) : round(cx + qw) + 1,
    ]
    valid = patch[patch > 0]
    if valid.size < 20:
        return None
    return float(np.median(valid)) / 1000.0


def _quat_rotate(q, v):
    """Rotate vector v by quaternion q=(x,y,z,w). Hand-rolled so the node needs
    no tf2_geometry_msgs (absent from ros-base)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q_xyz, v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    # v' = v + w*t + cross(q_xyz, t)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return (rx, ry, rz)


class Detector(Node):
    def __init__(self):
        super().__init__("world_detector")
        self.declare_parameter("color_topic",
            "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw/compressedDepth")
        self.declare_parameter("info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("rate_hz", 3.0)
        self.declare_parameter("match_gate_m", 0.5)  # track association radius
        self.declare_parameter("ema_alpha", 0.4)     # position smoothing
        self.declare_parameter("ttl_s", 5.0)         # drop tracks unseen this long

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g("map_frame")
        self.min_conf = g("min_confidence")
        self.match_gate = g("match_gate_m")
        self.alpha = g("ema_alpha")
        self.ttl = g("ttl_s")

        sensor = qos_profile_sensor_data  # best_effort; matches realsense/bridge
        self.color = None
        self.depth = None
        self.info = None
        self.create_subscription(CompressedImage, g("color_topic"),
            self._on_color, sensor)
        self.create_subscription(CompressedImage, g("depth_topic"),
            self._on_depth, sensor)
        self.create_subscription(CameraInfo, g("info_topic"),
            self._on_info, sensor)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(String, "/world/objects", latched)

        self.tracks: dict[int, dict] = {}
        self._next_id = 1
        self._warned = set()
        self.create_timer(1.0 / max(g("rate_hz"), 0.1), self._tick)
        self.get_logger().info("world_detector up (gate 1: /world/objects local)")

    def _on_color(self, msg): self.color = msg
    def _on_depth(self, msg): self.depth = msg
    def _on_info(self, msg): self.info = msg

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(text)

    def _cam_to_map(self, pt, cam_frame):
        """Optical-frame point -> map (x,y,z), or None if TF incomplete."""
        try:
            t = self.tf_buf.lookup_transform(
                self.map_frame, cam_frame, rclpy.time.Time())
        except TransformException:
            return None
        q = (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w)
        r = _quat_rotate(q, pt)
        tr = t.transform.translation
        return (r[0] + tr.x, r[1] + tr.y, r[2] + tr.z)

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        # Prune stale tracks first so a quiet scene still publishes shrinkage.
        self.tracks = {i: tk for i, tk in self.tracks.items()
                       if now - tk["last_seen"] <= self.ttl}

        if self.color is None:
            return
        rgb = _decode_color(self.color)
        dets = detect(rgb, self.min_conf)

        depth = _decode_depth(self.depth) if self.depth is not None else None
        if depth is None:
            self._warn_once("nodepth", "no aligned depth yet — positions omitted")

        if self.info is not None and depth is not None \
                and depth.shape[:2] == rgb.shape[:2]:
            k = self.info.k
            fx, cx, fy, cy = k[0], k[2], k[4], k[5]
            cam_frame = self.info.header.frame_id
            for d in dets:
                z = _median_depth_m(depth, d["box"])
                if z is None:
                    continue
                u = (d["box"][0] + d["box"][2]) / 2
                v = (d["box"][1] + d["box"][3]) / 2
                pt = ((u - cx) / fx * z, (v - cy) / fy * z, z)
                w = self._cam_to_map(pt, cam_frame)
                if w is None:
                    self._warn_once("notf",
                        f"TF {cam_frame}->{self.map_frame} incomplete "
                        "(need slam up + /tf_static durability over bridge)")
                    continue
                self._associate(d["label"], d["confidence"], w, now)
        elif depth is not None and self.info is not None \
                and depth.shape[:2] != rgb.shape[:2]:
            self._warn_once("shape",
                f"depth {depth.shape[:2]} != color {rgb.shape[:2]} — "
                "aligned_depth_to_color required")

        self._publish(now)

    def _associate(self, cls, score, world, now):
        gate2 = self.match_gate ** 2
        best, bestd = None, gate2
        for i, tk in self.tracks.items():
            if tk["cls"] != cls:
                continue
            dx = tk["xyz"][0] - world[0]
            dy = tk["xyz"][1] - world[1]
            d2 = dx * dx + dy * dy
            if d2 < bestd:
                best, bestd = i, d2
        if best is None:
            i = self._next_id
            self._next_id += 1
            self.tracks[i] = {"cls": cls, "score": score,
                              "xyz": list(world), "last_seen": now}
        else:
            tk = self.tracks[best]
            a = self.alpha
            tk["xyz"] = [tk["xyz"][j] * (1 - a) + world[j] * a for j in range(3)]
            tk["score"] = max(tk["score"], score)
            tk["last_seen"] = now

    def _publish(self, now):
        objs = [{
            "id": i,
            "cls": tk["cls"],
            "score": round(tk["score"], 3),
            "xy": [round(tk["xyz"][0], 2), round(tk["xyz"][1], 2)],
            "z": round(tk["xyz"][2], 2),
            "last_seen": round(now - tk["last_seen"], 1),  # seconds ago
        } for i, tk in sorted(self.tracks.items())]
        self.pub.publish(String(data=json.dumps({
            "frame": self.map_frame, "stamp": round(now, 2), "objects": objs})))


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
