"""Command-chain flow tracking + stop-reason classification
(scout.core.cmdflow, ADR-0036) — pure, plain floats for `now`.

The precedence contract under test: the FIRST sufficient explanation wins —
stale auto beats everything (idle is normal, never a fault), a silent CM
beats polygon blame, bypass beats cm_stop (whatever moves under a bypass is
deliberately unguarded), and only a fully-flowing chain reads `moving`.
"""

from scout.core.cmdflow import TopicFlow, stop_reason, twist_is_zero

FRESH = 0.5


def flow(msgs, window_s=10.0):
    """TopicFlow fed (t, is_zero) tuples."""
    f = TopicFlow(window_s=window_s)
    for t, z in msgs:
        f.on_msg(t, z)
    return f


def moving_chain(t=10.0):
    """All three hops flowing nonzero right up to `t`."""
    msgs = [(t - d, False) for d in (0.30, 0.25, 0.20, 0.15, 0.10, 0.05)]
    return flow(msgs), flow(msgs), flow(msgs)


def test_twist_is_zero_epsilon():
    assert twist_is_zero(0.0, 0.0)
    assert twist_is_zero(5e-5, -5e-5)
    assert not twist_is_zero(0.05, 0.0)
    assert not twist_is_zero(0.0, -0.35)


def test_topic_flow_age_and_nonzero_age():
    f = flow([(1.0, False), (2.0, True)])
    assert f.age(2.5) == 0.5
    assert f.nonzero_age(2.5) == 1.5  # the zero at t=2 didn't refresh it
    empty = TopicFlow()
    assert empty.age(5.0) is None
    assert empty.nonzero_age(5.0) is None


def test_topic_flow_max_gap_includes_open_gap_and_rolls_off():
    f = flow([(0.0, False), (3.0, False), (3.1, False)], window_s=10.0)
    assert f.max_gap(3.2) == 3.0          # the recorded 0->3 gap
    assert f.max_gap(8.0) == 4.9          # open gap since 3.1 now dominates
    # Past the window the 3.0 gap sample rolls off; only the open gap remains.
    f2 = flow([(0.0, False), (3.0, False), (12.9, False)], window_s=10.0)
    assert f2.max_gap(13.0) == 9.9


def test_auto_idle_beats_everything():
    auto, safe, out = moving_chain()
    # Stale auto (idle) even with bypass engaged and zones unsynced: idle.
    assert stop_reason(20.0, auto, safe, out, bypassed=True,
                       zone_mode='turn', zone_synced=False,
                       fresh_s=FRESH) == 'auto_idle'
    never = TopicFlow()
    assert stop_reason(1.0, never, safe, out, False, 'forward', True,
                       FRESH) == 'auto_idle'


def test_cm_dead_when_demand_flows_but_safe_is_silent():
    auto, _, out = moving_chain()
    dead_safe = flow([(5.0, False)])  # last forwarded long ago
    assert stop_reason(10.0, auto, dead_safe, out, False, 'forward', True,
                       FRESH) == 'cm_dead'


def test_bypass_wins_over_cm_stop_shaped_inputs():
    auto, _, out = moving_chain()
    zero_safe = flow([(9.9, True), (9.95, True)])  # CM forwarding zeros
    assert stop_reason(10.0, auto, zero_safe, out, bypassed=True,
                       zone_mode='forward', zone_synced=True,
                       fresh_s=FRESH) == 'bypass'


def test_cm_stop_tags_the_armed_zone():
    auto, _, out = moving_chain()
    zero_safe = flow([(9.9, True), (9.95, True)])
    assert stop_reason(10.0, auto, zero_safe, out, False, 'turn', True,
                       FRESH) == 'cm_stop:turn'
    assert stop_reason(10.0, auto, zero_safe, out, False, 'forward', True,
                       FRESH) == 'cm_stop:forward'


def test_zone_unsynced_flags_motion_with_unacked_zones():
    auto, safe, out = moving_chain()
    assert stop_reason(10.0, auto, safe, out, False, 'forward',
                       zone_synced=False, fresh_s=FRESH) == 'zone_unsynced'


def test_mux_override_when_final_mux_drops_autonomy():
    auto, safe, _ = moving_chain()
    zero_out = flow([(9.9, True), (9.95, True)])  # estop/teleop zero winning
    assert stop_reason(10.0, auto, safe, zero_out, False, 'forward', True,
                       FRESH) == 'mux_override'


def test_moving_when_chain_flows_end_to_end():
    auto, safe, out = moving_chain()
    assert stop_reason(10.0, auto, safe, out, False, 'forward', True,
                       FRESH) == 'moving'


def test_fresh_boundary_is_inclusive():
    auto, safe, out = moving_chain(t=10.0)
    # Last auto message at 9.95: at now=10.45 the age is exactly fresh_s.
    assert stop_reason(10.45, auto, safe, out, False, 'forward', True,
                       FRESH) != 'auto_idle'
    assert stop_reason(10.46, auto, safe, out, False, 'forward', True,
                       FRESH) == 'auto_idle'
