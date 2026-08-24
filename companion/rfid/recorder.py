#!/usr/bin/env python3
"""RFID read recorder (companion) — the primary tag DB (ADR-0025).

Subscribes the bridged /rfid/reads (pose-stamped JSON from the Pi's
flipper_node, latched depth-50 so a recorder outage replays recent reads) and
appends every read to a per-site sqlite DB at /sites/active/rfid.db. After
each insert it republishes /rfid/registry — a deduped latched tag table (one
row per data_hex, last non-null pose, hit count) that crosses the bridge back
to the Pi, which is how the webui and MCP list tags without any HTTP surface.

Storage follows the tags.db pattern (docker/scout-skills/tags.py): sqlite
opened per operation, CREATE TABLE IF NOT EXISTS every time, no migrations —
so a site switch (the /sites/active symlink repointing) applies on the very
next read even without the fleet_status-driven restart. read_id is the
PRIMARY KEY and inserts are INSERT OR IGNORE, so QoS replay after an outage
is idempotent.
"""
import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

DB_PATH = Path("/sites/active/rfid.db")

# Must match the Pi's LATCHED_HISTORY_QOS (scout/scout/qos.py): reliable +
# transient_local so the latched replay window arrives on a late join.
READS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)
REGISTRY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

SCHEMA = """
CREATE TABLE IF NOT EXISTS reads(
  read_id TEXT PRIMARY KEY,
  protocol TEXT NOT NULL,
  data_hex TEXT NOT NULL,
  map_x REAL, map_y REAL, map_yaw REAL,
  stamp_utc TEXT NOT NULL,
  received_utc TEXT NOT NULL);
CREATE INDEX IF NOT EXISTS idx_reads_hex ON reads(data_hex);
"""


def _connect():
    con = sqlite3.connect(DB_PATH)
    con.executescript(SCHEMA)
    return con


def insert_read(read: dict) -> bool:
    """INSERT OR IGNORE one read; True when the row is new."""
    pose = read.get("pose") or {}
    with _connect() as con:
        cur = con.execute(
            "INSERT OR IGNORE INTO reads"
            "(read_id, protocol, data_hex, map_x, map_y, map_yaw,"
            " stamp_utc, received_utc) VALUES (?,?,?,?,?,?,?,?)",
            (read["read_id"], read["protocol"], read["data_hex"],
             pose.get("x"), pose.get("y"), pose.get("yaw"),
             read.get("stamp_utc", ""),
             datetime.now(timezone.utc).isoformat(timespec="milliseconds")))
        return cur.rowcount > 0


def registry() -> dict:
    """Deduped tag table: one entry per data_hex with hit count, last seen,
    and the most recent NON-NULL pose (a localized read beats a null one)."""
    with _connect() as con:
        rows = con.execute(
            "SELECT data_hex, protocol, COUNT(*), MAX(stamp_utc) "
            "FROM reads GROUP BY data_hex").fetchall()
        tags = []
        for data_hex, protocol, count, last_seen in rows:
            pose_row = con.execute(
                "SELECT map_x, map_y, map_yaw FROM reads "
                "WHERE data_hex=? AND map_x IS NOT NULL "
                "ORDER BY stamp_utc DESC LIMIT 1", (data_hex,)).fetchone()
            tags.append({
                "data_hex": data_hex,
                "protocol": protocol,
                "count": count,
                "last_seen_utc": last_seen,
                "pose": (None if pose_row is None else
                         {"x": pose_row[0], "y": pose_row[1],
                          "yaw": pose_row[2]}),
            })
    return {"tags": sorted(tags, key=lambda t: t["last_seen_utc"],
                           reverse=True)}


class RfidRecorder(Node):
    def __init__(self):
        super().__init__("rfid_recorder")
        self._registry_pub = self.create_publisher(String, "/rfid/registry",
                                                   REGISTRY_QOS)
        self.create_subscription(String, "/rfid/reads", self._on_read,
                                 READS_QOS)
        self._publish_registry()
        self.get_logger().info("rfid_recorder up: %s" % DB_PATH)

    def _on_read(self, msg):
        try:
            read = json.loads(msg.data)
            new = insert_read(read)
        except (ValueError, KeyError, sqlite3.Error, OSError) as exc:
            # OSError covers a missing/broken /sites/active symlink; never die
            # over one message — the latched window redelivers on restart.
            self.get_logger().error("read dropped: %s (%r)"
                                    % (exc, msg.data[:200]))
            return
        if new:
            self.get_logger().info("stored %s %s" % (read.get("protocol"),
                                                     read.get("data_hex")))
            self._publish_registry()

    def _publish_registry(self):
        try:
            self._registry_pub.publish(String(data=json.dumps(registry())))
        except (sqlite3.Error, OSError) as exc:
            self.get_logger().error("registry publish failed: %s" % exc)


def main():
    rclpy.init()
    node = RfidRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
