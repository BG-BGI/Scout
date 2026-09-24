"""Single-flight/coalescing zone push (scout.core.zonepush, ADR-0036) — the
request discipline between collision_polygon_manager and collision_monitor's
set_parameters service. Pure state machine, plain floats for `now`, exactly
like test_latch.py.

The invariant under test everywhere: at most ONE request outstanding, and
`synced` is True only after an explicit successful ack of the current desired
state — every doubt path (timeout, failure, restart) forces a re-push.
"""

from scout.core.zonepush import Expire, Send, ZonePush

FWD = (True, False, False)
REV = (False, True, False)
TURN = (False, False, True)
BYPASS = (False, False, False)


def make_synced(zp, state=FWD, t=0.0):
    send = zp.set_desired(state, t, service_ready=True)
    assert zp.on_ack(send.seq, True, t + 0.1) is None
    assert zp.synced
    return send


def test_first_desired_sends_once_then_coalesces():
    zp = ZonePush(timeout_s=2.0)
    send = zp.set_desired(FWD, 0.0, service_ready=True)
    assert isinstance(send, Send) and send.state == FWD
    # In flight: rapid cmd_vel flips coalesce, nothing else is sent.
    assert zp.set_desired(REV, 0.1, service_ready=True) is None
    assert zp.set_desired(TURN, 0.2, service_ready=True) is None
    # Ack of the first request chases the LATEST desired, skipping REV.
    follow = zp.on_ack(send.seq, True, 0.3)
    assert isinstance(follow, Send) and follow.state == TURN
    assert not zp.synced  # TURN not acked yet


def test_delayed_ack_within_timeout_syncs():
    zp = ZonePush(timeout_s=2.0)
    send = zp.set_desired(FWD, 0.0, service_ready=True)
    assert zp.on_tick(1.0, service_ready=True) == []  # not expired yet
    assert zp.on_ack(send.seq, True, 1.5) is None
    assert zp.synced


def test_timeout_expires_and_resends_late_ack_ignored():
    zp = ZonePush(timeout_s=2.0)
    send = zp.set_desired(FWD, 0.0, service_ready=True)
    actions = zp.on_tick(2.0, service_ready=True)
    assert actions[0] == Expire(send.seq)
    assert isinstance(actions[1], Send) and actions[1].state == FWD
    assert actions[1].seq != send.seq
    assert not zp.synced
    # The expired request's ack arrives late: ignored, retry still in flight.
    assert zp.on_ack(send.seq, True, 2.5) is None
    assert not zp.synced
    assert zp.on_ack(actions[1].seq, True, 2.6) is None
    assert zp.synced


def test_stale_ack_after_success_is_ignored():
    zp = ZonePush(timeout_s=2.0)
    send = make_synced(zp)
    zp.on_ack(send.seq, False, 1.0)  # duplicate/late callback for a done seq
    assert zp.synced  # unchanged


def test_rejected_reply_leaves_unsynced_and_tick_retries():
    zp = ZonePush(timeout_s=2.0)
    send = zp.set_desired(FWD, 0.0, service_ready=True)
    assert zp.on_ack(send.seq, False, 0.5) is None  # failure never auto-chases
    assert not zp.synced
    actions = zp.on_tick(1.0, service_ready=True)
    assert len(actions) == 1 and isinstance(actions[0], Send)
    assert actions[0].state == FWD


def test_service_restart_resets_ack_and_repushes():
    zp = ZonePush(timeout_s=2.0)
    make_synced(zp, FWD)
    # collision_monitor goes down: acked is voided (it holds nothing now).
    assert zp.on_tick(5.0, service_ready=False) == []
    assert not zp.synced
    # It comes back holding YAML defaults: FWD is re-pushed even though it
    # was acked before the restart.
    actions = zp.on_tick(6.0, service_ready=True)
    assert len(actions) == 1 and actions[0].state == FWD
    assert not zp.synced


def test_restart_mid_flight_expires_request_and_ignores_its_late_ack():
    zp = ZonePush(timeout_s=10.0)  # long timeout: restart, not expiry, drives
    send = zp.set_desired(FWD, 0.0, service_ready=True)
    zp.on_tick(1.0, service_ready=False)   # service dies with request pending
    actions = zp.on_tick(2.0, service_ready=True)  # and comes back
    # The pre-restart request is untrustworthy: expired, then re-sent.
    assert actions[0] == Expire(send.seq)
    assert isinstance(actions[1], Send) and actions[1].state == FWD
    # Its ack straggles in anyway: ignored — only the retry's ack syncs.
    assert zp.on_ack(send.seq, True, 2.5) is None
    assert not zp.synced
    assert zp.on_ack(actions[1].seq, True, 2.6) is None
    assert zp.synced


def test_never_synced_without_explicit_ack():
    zp = ZonePush(timeout_s=2.0)
    # Bypass (the MORE permissive state) that never acks: re-sent every tick
    # window, never reported synced — the node may not assume zones dropped.
    send = zp.set_desired(BYPASS, 0.0, service_ready=True)
    assert not zp.synced
    for t in (2.0, 4.0, 6.0):
        actions = zp.on_tick(t, service_ready=True)
        assert actions and isinstance(actions[-1], Send)
        assert not zp.synced
        send = actions[-1]
    assert zp.on_ack(send.seq, True, 6.5) is None
    assert zp.synced


def test_noop_when_desired_equals_acked():
    zp = ZonePush(timeout_s=2.0)
    make_synced(zp, FWD)
    assert zp.set_desired(FWD, 1.0, service_ready=True) is None
    assert zp.on_tick(2.0, service_ready=True) == []
    assert zp.synced


def test_not_ready_records_desired_for_later():
    zp = ZonePush(timeout_s=2.0)
    assert zp.set_desired(TURN, 0.0, service_ready=False) is None
    actions = zp.on_tick(1.0, service_ready=True)
    assert len(actions) == 1 and actions[0].state == TURN
