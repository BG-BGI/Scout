#!/usr/bin/env python3
"""World-model object detector (companion, gates 1-2 of WORLDMODEL.md).

Continuous YOLO11n on the bridged D455 stream -> map-frame object table.
CPU-only, onnxruntime — no torch (the .onnx is baked in a throwaway build
stage, same as scout-skills).

Two outputs (both latched std_msgs/String JSON, map frame):
- /world/objects  — LIVE view: tracks seen within ttl_s. Same schema as ever;
  ids are now registry ids, so they no longer churn when an object leaves FOV
  and comes back.
- /world/registry — PERSISTENT registry: every confirmed track since node
  start, never aged out. Class is a cross-frame vote (a chair that YOLO calls
  `skateboard` in three frames and `chair` in five stays a chair), position is
  a median over recent sightings, `hits` counts sightings so consumers can
  reject one-frame false positives. This is what "count the chairs" queries —
  the LLM never reassembles counts from live frames.

Runs on the companion's LOCAL DDS graph: color/depth/info/tf all arrive over
the zenoh bridge. /world/objects and /world/registry cross back to the Pi via
the bridge allowlists (read-only telemetry; ADR-0001 control surface
unchanged). The Pi runs fine without this node (Pi-standalone contract §0.7).

Registry lifetime is the node's lifetime — restart the detector container to
start a fresh count, or call the local /world/clear_registry Trigger service
(local-graph only; not bridged).

Reuses scout-skills' verified deprojection math: optical-frame ray through the
box centre at median box depth, then TF cam->map at the IMAGE stamp.
"""
import json
import math
from collections import deque
from io import BytesIO

import numpy as np
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
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
        self.declare_parameter("match_gate_m", 0.5)  # same-class association
        self.declare_parameter("cross_class_gate_m", 0.3)  # label-flip merge
        self.declare_parameter("ema_alpha", 0.4)     # association-position smoothing
        self.declare_parameter("ttl_s", 5.0)         # live-view visibility window
        # Chair recall (session 2026-08-21): office chairs score 0.25-0.4 under
        # desks. Detect at the lower band for these classes only.
        self.declare_parameter("low_conf_classes", ["chair"])
        self.declare_parameter("low_conf_threshold", 0.25)
        # Registry hygiene: tracks with < min_hits_confirm sightings are
        # unconfirmed (one-frame false positives); drop them after this long.
        # Confirmed tracks are NEVER dropped.
        self.declare_parameter("min_hits_confirm", 2)
        self.declare_parameter("prune_unconfirmed_after_s", 120.0)

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g("map_frame")
        self.min_conf = g("min_confidence")
        self.match_gate = g("match_gate_m")
        self.cross_gate = g("cross_class_gate_m")
        self.alpha = g("ema_alpha")
        self.ttl = g("ttl_s")
        self.low_classes = set(g("low_conf_classes") or [])
        self.low_conf = g("low_conf_threshold")
        self.min_hits = int(g("min_hits_confirm"))
        self.prune_after = g("prune_unconfirmed_after_s")

        sensor = qos_profile_sensor_data  # best_effort; matches realsense/bridge
        self.color = None
        self.depth = None
        self.info = None
        # Reentrant group + MultiThreadedExecutor (see main): the blocking YOLO
        # in _tick must NOT starve the image/TF callbacks, or self.color goes
        # stale and every cycle re-projects the same frame (identical scores,
        # objects that rotate with the robot instead of holding still).
        self.cbg = ReentrantCallbackGroup()
        self.create_subscription(CompressedImage, g("color_topic"),
            self._on_color, sensor, callback_group=self.cbg)
        self.create_subscription(CompressedImage, g("depth_topic"),
            self._on_depth, sensor, callback_group=self.cbg)
        self.create_subscription(CameraInfo, g("info_topic"),
            self._on_info, sensor, callback_group=self.cbg)

        # 30 s cache so a lookup at the image stamp still finds map->odom even
        # when slam future-dates it (~0.7 s ahead) and frames lag through the
        # bridge.
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(String, "/world/objects", latched)
        self.pub_reg = self.create_publisher(String, "/world/registry", latched)
        self.create_service(Trigger, "/world/clear_registry", self._on_clear)

        # tracks IS the persistent registry. A track:
        #   cls_votes: {label: [hits, best_score]}  — cross-frame class vote
        #   xyz: EMA position (association gate)
        #   pos: deque of recent sightings (median -> published position)
        #   first_seen / last_seen: node-clock seconds
        #   hits: total sightings
        self.tracks: dict[int, dict] = {}
        self._next_id = 1
        self._last_stamp = None  # latest processed image stamp (Pi clock, s)
        self._warned = set()
        self.create_timer(1.0 / max(g("rate_hz"), 0.1), self._tick,
            callback_group=self.cbg)
        self.get_logger().info(
            "world_detector up (/world/objects live + /world/registry persistent)")

    def _on_color(self, msg): self.color = msg
    def _on_depth(self, msg): self.depth = msg
    def _on_info(self, msg): self.info = msg

    def _on_clear(self, req, resp):
        n = len(self.tracks)
        self.tracks = {}
        self._next_id = 1
        resp.success = True
        resp.message = f"registry cleared ({n} tracks dropped)"
        self.get_logger().info(resp.message)
        return resp

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(text)

    def _cam_to_map(self, pt, cam_frame, stamp):
        """Optical-frame point -> map (x,y,z), or None if TF incomplete.

        Look up at the IMAGE stamp, not latest: composing a future-dated
        map->odom (slam post-dates it ~0.7 s) with the current odom->base_link
        mixes two times and smears a world-fixed object into rotation during a
        pivot. The image stamp is Pi-clock, matching every bridged transform."""
        try:
            t = self.tf_buf.lookup_transform(
                self.map_frame, cam_frame, Time.from_msg(stamp),
                timeout=Duration(seconds=0.2))
        except TransformException:
            return None
        q = (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w)
        r = _quat_rotate(q, pt)
        tr = t.transform.translation
        return (r[0] + tr.x, r[1] + tr.y, r[2] + tr.z)

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        # Prune only UNCONFIRMED stale tracks — confirmed registry entries are
        # permanent (D1: never age the registry out).
        self.tracks = {i: tk for i, tk in self.tracks.items()
                       if tk["hits"] >= self.min_hits
                       or now - tk["last_seen"] <= self.prune_after}

        color_msg = self.color  # local ref: a callback may replace it mid-tick
        if color_msg is None:
            return
        stamp = color_msg.header.stamp
        self._last_stamp = stamp.sec + stamp.nanosec * 1e-9
        rgb = _decode_color(color_msg)
        # Detect at the lower band, then filter: full threshold for everything,
        # low band only for the recall-boosted classes (chairs).
        floor = min(self.min_conf, self.low_conf) if self.low_classes \
            else self.min_conf
        dets = [d for d in detect(rgb, floor)
                if d["confidence"] >= self.min_conf
                or d["label"] in self.low_classes]

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
                w = self._cam_to_map(pt, cam_frame, stamp)
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

    @staticmethod
    def _label(tk):
        """Voted class: most hits wins, best_score breaks ties."""
        return max(tk["cls_votes"].items(),
                   key=lambda kv: (kv[1][0], kv[1][1]))[0]

    def _associate(self, cls, score, world, now):
        """Nearest-track association. Same voted class within match_gate; a
        DIFFERENT class within cross_class_gate also merges — that is the same
        physical object mislabeled this frame (chair 5-star base -> skateboard),
        and the class vote sorts it out across frames."""
        best, bestd = None, self.match_gate ** 2
        xgate2 = self.cross_gate ** 2
        for i, tk in self.tracks.items():
            dx = tk["xyz"][0] - world[0]
            dy = tk["xyz"][1] - world[1]
            d2 = dx * dx + dy * dy
            same = self._label(tk) == cls
            if d2 < bestd and (same or d2 < xgate2):
                best, bestd = i, d2
        if best is None:
            i = self._next_id
            self._next_id += 1
            self.tracks[i] = {
                "cls_votes": {cls: [1, score]},
                "xyz": list(world),
                "pos": deque([tuple(world)], maxlen=15),
                "first_seen": now, "last_seen": now, "hits": 1,
            }
        else:
            tk = self.tracks[best]
            a = self.alpha
            tk["xyz"] = [tk["xyz"][j] * (1 - a) + world[j] * a for j in range(3)]
            tk["pos"].append(tuple(world))
            v = tk["cls_votes"].setdefault(cls, [0, 0.0])
            v[0] += 1
            v[1] = max(v[1], score)
            tk["hits"] += 1
            tk["last_seen"] = now

    def _entry(self, i, tk, now):
        """One published object. Position = median of recent sightings (robust
        vs the EMA used for gating); class/score = the cross-frame vote."""
        cls = self._label(tk)
        p = np.median(np.asarray(tk["pos"]), axis=0)
        return {
            "id": i,
            "cls": cls,
            "score": round(tk["cls_votes"][cls][1], 3),
            "xy": [round(float(p[0]), 2), round(float(p[1]), 2)],
            "z": round(float(p[2]), 2),
            "hits": tk["hits"],
            "last_seen": round(now - tk["last_seen"], 1),  # seconds ago
        }

    def _publish(self, now):
        # Top-level stamp = the processed image stamp (Pi clock, wall-time).
        # get_clock().now() here jumped +2432 s over a real 5 s gap in the
        # 2026-08-21 session (companion clock step); the image stamp is the
        # timebase every bridged transform already uses.
        stamp = round(self._last_stamp, 2) if self._last_stamp else None
        live = [self._entry(i, tk, now) for i, tk in sorted(self.tracks.items())
                if now - tk["last_seen"] <= self.ttl]
        self.pub.publish(String(data=json.dumps(
            {"frame": self.map_frame, "stamp": stamp, "objects": live})))
        reg = [self._entry(i, tk, now) | {
                   "first_seen": round(now - tk["first_seen"], 1)}
               for i, tk in sorted(self.tracks.items())
               if tk["hits"] >= self.min_hits]
        self.pub_reg.publish(String(data=json.dumps(
            {"frame": self.map_frame, "stamp": stamp,
             "min_hits": self.min_hits, "objects": reg})))


def main():
    rclpy.init()
    node = Detector()
    # MultiThreaded so the blocking YOLO tick runs on one thread while image/TF
    # callbacks keep updating on others (single-threaded starves them -> stale
    # frame + stale TF).
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
