"""Shared QoS profiles (ADR-0013).

Two bespoke latched profiles had already been hand-built (link_watchdog's
action-status subscription) — identical
settings, different shapes. The next latched topic imports from here instead
of rolling a third.

For sensor streams use rclpy's own `qos_profile_sensor_data` (best-effort);
SC2 in test_conventions.py enforces that — a default reliable subscription to
a best-effort sensor publisher receives NOTHING and says so only in a one-line
discovery warning.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# Reliable + transient_local + last-1: a late subscriber still sees the most
# recent message (action status after a restart).
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
