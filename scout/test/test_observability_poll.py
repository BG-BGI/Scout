"""Exporter backpressure + DDS-probe parsing (docker/observability-exporter/
poll_loop.py, loaded by path — separate container, pure helpers extracted so
the single-flight scheduling and one-exec probe script are pinned here)."""

import importlib.util
import sys
from pathlib import Path

MOD_PY = (Path(__file__).resolve().parents[2] / 'docker' / 'observability-exporter'
          / 'poll_loop.py')

spec = importlib.util.spec_from_file_location('exporter_poll_loop', MOD_PY)
pl = importlib.util.module_from_spec(spec)
sys.modules['exporter_poll_loop'] = pl
spec.loader.exec_module(pl)

TOPIC_INFO_HUMBLE = """\
Type: sensor_msgs/msg/LaserScan

Publisher count: 1

Node name: rplidar_node
...
Subscription count: 2
"""

TOPIC_INFO_OLD_SPELLING = """\
Publisher count: 3
Subscriber count: 4
"""


def test_next_sleep_never_negative():
    assert pl.next_sleep(20.0, 5.0) == 15.0
    assert pl.next_sleep(20.0, 20.0) == 0.0
    assert pl.next_sleep(20.0, 45.0) == 0.0  # slow pass runs back-to-back, never queues


def test_next_sleep_full_period_when_instant():
    assert pl.next_sleep(20.0, 0.0) == 20.0


def test_parse_topic_info_both_spellings():
    assert pl.parse_topic_info(TOPIC_INFO_HUMBLE) == (1, 2)
    assert pl.parse_topic_info(TOPIC_INFO_OLD_SPELLING) == (3, 4)
    assert pl.parse_topic_info('garbage') == (None, None)


def test_build_dds_script_sources_once_times_out_each():
    topics = ['/scan', '/odom', '/map']
    prefix = 'source /opt/ros/humble/setup.bash && '
    script = pl.build_dds_script(prefix, topics, 5)
    assert script.count('source /opt/ros/humble/setup.bash') == 1  # not once per topic
    assert script.count('timeout 5 ros2 topic info') == len(topics)
    for t in topics:
        assert f'=== TOPIC {t}' in script
    assert script.endswith('; true')  # a failed probe must not fail the exec


def test_parse_dds_script_output_round_trip():
    text = (
        '=== TOPIC /scan\n' + TOPIC_INFO_HUMBLE +
        '=== TOPIC /odom\n' + TOPIC_INFO_OLD_SPELLING
    )
    out = pl.parse_dds_script_output(text, ['/scan', '/odom'])
    assert out == {'/scan': (1, 2), '/odom': (3, 4)}


def test_parse_dds_script_output_tolerates_failed_and_missing_sections():
    text = (
        '=== TOPIC /scan\n' + TOPIC_INFO_HUMBLE +
        '=== TOPIC /odom\ntimeout: sending signal TERM\n'
        # /map section missing entirely (script cut off by the outer timeout)
    )
    out = pl.parse_dds_script_output(text, ['/scan', '/odom', '/map'])
    assert out['/scan'] == (1, 2)
    assert out['/odom'] == (None, None)  # probe failed -> no fake zero
    assert out['/map'] == (None, None)


def test_run_source_once_swallows_exception_and_counts_it():
    stats = pl.SourceStats()
    clock = iter([10.0, 11.0, 12.0, 13.0])

    def boom():
        raise RuntimeError('poll died')

    ok = pl.run_source_once(boom, stats, now_fn=lambda: next(clock))
    assert ok is False
    assert stats.errors == 1
    assert stats.last_success_t is None  # failure never stamps success
    assert stats.last_duration_s == 1.0


def test_run_source_once_success_stamps_time():
    stats = pl.SourceStats(errors=2)
    clock = iter([10.0, 12.0, 12.5])
    ok = pl.run_source_once(lambda: None, stats, now_fn=lambda: next(clock))
    assert ok is True
    assert stats.errors == 2  # untouched on success
    assert stats.last_duration_s == 2.0
    assert stats.last_success_t == 12.5
