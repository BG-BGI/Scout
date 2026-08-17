"""Pure argv/path builders for the rosbag2 record-on-demand node (ADR-0017).

bag_recorder spawns `ros2 bag record` as a subprocess; everything about that
invocation that can be decided without ROS lives here so it is testable on a
plain Python box: the output-directory naming, the topic-list validation, and
the argv assembly. The node injects the profile values (record_topics) and the
clock — same contract as the rest of scout.core.

⚠ record_argv NEVER emits --max-bag-size / --max-bag-duration: bags split by
size or duration do not play back correctly on Humble (only the last split
plays — ros2/rosbag2#966). The runaway guard is the node's own auto-stop
timer instead; test_recording.py pins the flags' absence.
"""

from datetime import datetime, timezone

# Colon-free UTC stamp: safe as a directory name on every filesystem and
# sorts lexicographically by time.
_STAMP_FMT = '%Y-%m-%dT%H-%M-%SZ'


def bag_dir(now: datetime, root: str) -> str:
    """`<root>/<UTC stamp>` for a recording started at `now`. Naive datetimes
    are refused — a local-time stamp labeled Z would lie by the UTC offset."""
    if now.tzinfo is None:
        raise ValueError('bag_dir needs an aware (UTC) datetime')
    stamp = now.astimezone(timezone.utc).strftime(_STAMP_FMT)
    return '%s/%s' % (root.rstrip('/'), stamp)


def resolve_topics(topics) -> list:
    """Validate a topic list (profile record_topics or the `topics` param
    override): non-empty, every entry a '/'-rooted string. Returns a fresh
    list. Raises ValueError naming the offender — a typoed topic would
    otherwise record nothing and say so nowhere."""
    topics = list(topics or [])
    if not topics:
        raise ValueError('record topic list is empty')
    bad = [t for t in topics
           if not (isinstance(t, str) and t.startswith('/'))]
    if bad:
        raise ValueError('record topics must be /-rooted strings: %r' % bad)
    return topics


def record_argv(topics, out_dir: str, qos_overrides_path: str = None) -> list:
    """The `ros2 bag record` command line: explicit topics (never -a — an
    all-topics bag on this stack includes two camera streams and fills the SD
    card), -o into a not-yet-existing dir (rosbag2 creates it and errors if it
    exists, which is exactly the double-start guard we want at the filesystem
    level too), and the QoS overrides file so best-effort sensor publishers
    (/imu/data — the documented silent-miss trap) are actually received."""
    argv = ['ros2', 'bag', 'record', '-o', out_dir]
    if qos_overrides_path:
        argv += ['--qos-profile-overrides-path', qos_overrides_path]
    return argv + resolve_topics(topics)
