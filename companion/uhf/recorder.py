#!/usr/bin/env python3
"""UHF tag read recorder (companion) — the primary UHF DB (ADR-0032).

Deliberately its own script, NOT another instance of rfid/recorder.py: that
script's value is an identical schema/dedup body for two identical radios
(RFID/NFC, ADR-0026). UHF differs on every axis — /uhf/reads carries BATCH
messages (up to 150 tags/s on the Pi collapse to <=10 msg/s), rows carry
signal columns (rssi/freq/phase) the stage-2 solver needs, and the registry
computes a position estimate instead of echoing a last-seen pose.

Subscribes the bridged /uhf/reads batches (pose-stamped JSON from the Pi's
uhf_node, latched depth-50 so a recorder outage replays recent batches) and
appends every read to /sites/active/uhf.db — one transaction per batch. After
each new batch it republishes /uhf/registry: one row per EPC with hit count,
last seen, and `est_pose` = the RSSI-weighted centroid of all localized read
positions (weight 10^(rssi/10) — linear power, so near reads dominate) plus
`spread_m` (weighted RMS distance from the centroid — a rough confidence).
Stage-1 accuracy target is 1-3 m; the per-read phase/freq/timestamp_ms columns
are stored untouched as stage-2 synthetic-aperture fuel (ADR-0032). The
registry crosses the bridge back to the Pi for the webui and MCP.

Storage follows the tags.db pattern: sqlite opened per operation, CREATE TABLE
IF NOT EXISTS every time, no migrations — a site switch (the /sites/active
symlink repointing) applies on the very next batch. read_id is the PRIMARY KEY
and inserts are INSERT OR IGNORE, so QoS replay after an outage is idempotent.

Registry recompute is O(all localized reads) per batch — fine into the tens of
thousands; memoize per-EPC running sums if a site ever accumulates far more.
"""
import json
import math
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

DB_PATH = "/sites/active/uhf.db"
READS_TOPIC = "/uhf/reads"
REGISTRY_TOPIC = "/uhf/registry"

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
  epc TEXT NOT NULL,
  rssi_dbm INTEGER,
  freq_khz INTEGER,
  phase INTEGER,
  antenna INTEGER,
  timestamp_ms INTEGER,
  batch_id TEXT,
  map_x REAL, map_y REAL, map_yaw REAL,
  stamp_utc TEXT NOT NULL,
  received_utc TEXT NOT NULL);
CREATE INDEX IF NOT EXISTS idx_reads_epc ON reads(epc);
"""


def _connect(db_path):
    con = sqlite3.connect(db_path)
    con.executescript(SCHEMA)
    return con


def insert_batch(db_path, batch: dict) -> int:
    """INSERT OR IGNORE every read of one batch in one transaction; returns
    how many rows were new. The batch pose stamps every read in the window."""
    pose = batch.get("pose") or {}
    received = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    new = 0
    with _connect(db_path) as con:
        for r in batch.get("reads", []):
            cur = con.execute(
                "INSERT OR IGNORE INTO reads"
                "(read_id, epc, rssi_dbm, freq_khz, phase, antenna,"
                " timestamp_ms, batch_id, map_x, map_y, map_yaw,"
                " stamp_utc, received_utc) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (r["read_id"], r["epc"], r.get("rssi_dbm"),
                 r.get("freq_khz"), r.get("phase"), r.get("antenna"),
                 r.get("timestamp_ms"), batch.get("batch_id"),
                 pose.get("x"), pose.get("y"), pose.get("yaw"),
                 batch.get("stamp_utc", ""), received))
            new += cur.rowcount
    return new


def _centroid(points):
    """RSSI-weighted centroid of [(x, y, rssi_dbm)] -> (est, spread_m).
    Weight is linear received power 10^(rssi/10), so the strongest (nearest)
    reads dominate; spread is the weighted RMS distance from the centroid."""
    wsum = xsum = ysum = 0.0
    for x, y, rssi in points:
        w = 10.0 ** ((rssi if rssi is not None else -70) / 10.0)
        wsum += w
        xsum += w * x
        ysum += w * y
    cx, cy = xsum / wsum, ysum / wsum
    var = sum((10.0 ** ((r if r is not None else -70) / 10.0))
              * ((x - cx) ** 2 + (y - cy) ** 2)
              for x, y, r in points) / wsum
    return {"x": round(cx, 3), "y": round(cy, 3)}, round(math.sqrt(var), 3)


def registry(db_path) -> dict:
    """One entry per EPC: hit count, last seen, last RSSI, and the stage-1
    position estimate over all LOCALIZED reads (null-pose reads count only
    toward `count`)."""
    with _connect(db_path) as con:
        rows = con.execute(
            "SELECT epc, COUNT(*), MAX(stamp_utc) "
            "FROM reads GROUP BY epc").fetchall()
        tags = []
        for epc, count, last_seen in rows:
            pts = con.execute(
                "SELECT map_x, map_y, rssi_dbm FROM reads "
                "WHERE epc=? AND map_x IS NOT NULL", (epc,)).fetchall()
            last_rssi = con.execute(
                "SELECT rssi_dbm FROM reads WHERE epc=? "
                "ORDER BY stamp_utc DESC LIMIT 1", (epc,)).fetchone()
            est, spread = _centroid(pts) if pts else (None, None)
            tags.append({
                "epc": epc,
                "count": count,
                "last_seen_utc": last_seen,
                "last_rssi_dbm": None if last_rssi is None else last_rssi[0],
                "est_pose": est,
                "spread_m": spread,
                "n_localized": len(pts),
            })
    return {"tags": sorted(tags, key=lambda t: t["last_seen_utc"],
                           reverse=True)}


class UhfReadRecorder(Node):
    def __init__(self):
        super().__init__("uhf_read_recorder")
        self._db_path = Path(self.declare_parameter("db_path", DB_PATH).value)
        reads_topic = self.declare_parameter(
            "reads_topic", READS_TOPIC).value
        registry_topic = self.declare_parameter(
            "registry_topic", REGISTRY_TOPIC).value
        self._registry_pub = self.create_publisher(String, registry_topic,
                                                   REGISTRY_QOS)
        self.create_subscription(String, reads_topic, self._on_batch,
                                 READS_QOS)
        self._publish_registry()
        self.get_logger().info("uhf_read_recorder up: %s <- %s -> %s"
                               % (self._db_path, reads_topic, registry_topic))

    def _on_batch(self, msg):
        try:
            batch = json.loads(msg.data)
            new = insert_batch(self._db_path, batch)
        except (ValueError, KeyError, sqlite3.Error, OSError) as exc:
            # OSError covers a missing/broken /sites/active symlink; never die
            # over one message — the latched window redelivers on restart.
            self.get_logger().error("batch dropped: %s (%r)"
                                    % (exc, msg.data[:200]))
            return
        if new:
            self.get_logger().info("stored %d reads (batch %s)"
                                   % (new, batch.get("batch_id")))
            self._publish_registry()

    def _publish_registry(self):
        try:
            self._registry_pub.publish(
                String(data=json.dumps(registry(self._db_path))))
        except (sqlite3.Error, OSError) as exc:
            self.get_logger().error("registry publish failed: %s" % exc)


def main():
    rclpy.init()
    node = UhfReadRecorder()
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
