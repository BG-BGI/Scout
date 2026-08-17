"""scout.core.recording — argv/path builders for bag_recorder (ADR-0017)."""

from datetime import datetime, timezone

import pytest

from scout.core import recording as r


def test_bag_dir_utc_stamp_exact():
    now = datetime(2026, 8, 17, 14, 30, 5, tzinfo=timezone.utc)
    assert r.bag_dir(now, '/x/captures/bags') == \
        '/x/captures/bags/2026-08-17T14-30-05Z'


def test_bag_dir_converts_to_utc_and_strips_trailing_slash():
    # UTC-5: 09:30 local == 14:30 Z.
    from datetime import timedelta, tzinfo

    class Minus5(tzinfo):
        def utcoffset(self, dt):
            return timedelta(hours=-5)

        def dst(self, dt):
            return timedelta(0)

    now = datetime(2026, 8, 17, 9, 30, 5, tzinfo=Minus5())
    assert r.bag_dir(now, '/x/bags/') == '/x/bags/2026-08-17T14-30-05Z'


def test_bag_dir_refuses_naive_datetime():
    with pytest.raises(ValueError):
        r.bag_dir(datetime(2026, 8, 17, 14, 30, 5), '/x')


def test_resolve_topics_valid_list_passes_through_as_copy():
    src = ['/odom', '/tf']
    out = r.resolve_topics(src)
    assert out == src and out is not src


def test_resolve_topics_rejects_empty_and_unrooted():
    with pytest.raises(ValueError):
        r.resolve_topics([])
    with pytest.raises(ValueError, match='odom'):
        r.resolve_topics(['odom'])
    with pytest.raises(ValueError):
        r.resolve_topics(['/ok', 42])


def test_record_argv_shape():
    argv = r.record_argv(['/odom', '/scan'], '/x/bags/t', '/cfg/qos.yaml')
    assert argv == ['ros2', 'bag', 'record', '-o', '/x/bags/t',
                    '--qos-profile-overrides-path', '/cfg/qos.yaml',
                    '/odom', '/scan']


def test_record_argv_qos_path_optional():
    argv = r.record_argv(['/odom'], '/x/bags/t')
    assert '--qos-profile-overrides-path' not in argv


def test_record_argv_never_splits_bags():
    # Humble known issue: split bags do not play back (ros2/rosbag2#966).
    argv = r.record_argv(['/odom'], '/x/bags/t', '/cfg/qos.yaml')
    assert '--max-bag-size' not in argv
    assert '--max-bag-duration' not in argv
    assert '-a' not in argv  # explicit topics only


def test_record_argv_validates_topics():
    with pytest.raises(ValueError):
        r.record_argv([], '/x/bags/t')
