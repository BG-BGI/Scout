"""Pure scheduling and parsing helpers for the exporter.

No docker/prometheus/websockets imports on purpose: scout/test path-imports
this module (the test_elevator_config.py precedent) to pin the backpressure
and parsing behavior without standing up any of the exporter's I/O.
"""

import re
import time
from collections.abc import Callable
from dataclasses import dataclass

_PUB_RE = re.compile(r"Publisher count:\s*(\d+)")
# Humble prints "Subscription count", older docs say "Subscriber count" —
# tolerate both (frozen behavior from the original poll_dds regex).
_SUB_RE = re.compile(r"Subscri(?:ber|ption) count:\s*(\d+)")
_TOPIC_DELIM = "=== TOPIC "


@dataclass
class SourceStats:
    """Per-source accounting a poll loop carries across passes."""

    errors: int = 0
    last_success_t: float | None = None
    last_duration_s: float = 0.0


def next_sleep(period_s: float, elapsed_s: float) -> float:
    """Sleep needed to hold `period_s` between pass starts; never negative,
    so a pass slower than the period just runs back-to-back — it can delay
    only its own next pass, nothing ever queues behind it."""
    return max(0.0, period_s - elapsed_s)


def run_source_once(
    fn: Callable[[], None],
    stats: SourceStats,
    now_fn: Callable[[], float] = time.monotonic,
) -> bool:
    """One guarded pass of a poll source. Any exception is swallowed into
    `stats.errors` (a poll must never kill its loop); success stamps
    `last_success_t`. Returns whether the pass succeeded."""
    start = now_fn()
    try:
        fn()
    except Exception:  # noqa: BLE001 — sources do arbitrary I/O; the loop must survive all of it
        stats.errors += 1
        stats.last_duration_s = now_fn() - start
        return False
    stats.last_duration_s = now_fn() - start
    stats.last_success_t = now_fn()
    return True


def build_dds_script(prefix: str, topics: list[str], per_topic_timeout_s: float) -> str:
    """One bash script probing every topic: sources the ROS environment ONCE
    (the 9x double-source was the dominant cost of the old per-topic execs),
    bounds each `ros2 topic info` with coreutils timeout, and delimits output
    per topic so a hung/failed probe loses only its own section."""
    probes = "; ".join(
        f'echo "{_TOPIC_DELIM}{t}"; timeout {per_topic_timeout_s:g} ros2 topic info {t} -v'
        for t in topics
    )
    return f"{prefix}{probes}; true"


def parse_topic_info(text: str) -> tuple[int | None, int | None]:
    """(publisher count, subscription count) out of `ros2 topic info -v`
    output; None for a count the text does not contain."""
    pub = _PUB_RE.search(text)
    sub = _SUB_RE.search(text)
    return (int(pub.group(1)) if pub else None, int(sub.group(1)) if sub else None)


def parse_dds_script_output(
    text: str, topics: list[str]
) -> dict[str, tuple[int | None, int | None]]:
    """Split build_dds_script() output back into per-topic (pub, sub) counts.
    Topics whose section is missing (script cut off) or unparseable (probe
    timed out / errored) come back as (None, None) — callers leave their
    gauges untouched rather than reporting a false zero."""
    sections: dict[str, str] = {}
    current: str | None = None
    lines: dict[str, list[str]] = {}
    for line in text.splitlines():
        if line.startswith(_TOPIC_DELIM):
            current = line[len(_TOPIC_DELIM):].strip()
            lines[current] = []
        elif current is not None:
            lines[current].append(line)
    sections = {t: "\n".join(body) for t, body in lines.items()}
    return {t: parse_topic_info(sections.get(t, "")) for t in topics}
