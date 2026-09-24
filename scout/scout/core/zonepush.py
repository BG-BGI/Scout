"""Single-flight, coalescing push of the collision-monitor zone state (pure,
no ROS) — the request discipline collision_polygon_manager wraps around
`set_parameters` calls.

The node's original scheme kept only `_pushed = last state acked` and compared
against it in every `/cmd_vel_auto` callback (20–50 Hz). Between issuing a
request and its ack, `state != _pushed` stayed true, so every callback in the
ack window issued ANOTHER call_async: a slow or restarting collision_monitor
amplified pending work without bound, and futures were never cancelled or
timed out. This state machine makes the push single-flight (at most one
request outstanding), coalescing (the latest desired state simply overwrites
— no queue), and bounded (an unacked request expires after `timeout_s` so the
node can drop the pending future and retry).

Fail-safe bias: `acked` becomes known ONLY on an explicit successful ack.
Every doubt path — timeout, failure/partial reply, collision_monitor restart
(service ready False→True means it rebooted with its YAML defaults regardless
of anything pushed before) — resets `acked` to unknown, which both forces a
re-push and forbids reporting the zones as in sync.

The caller (the node) owns the actual service I/O: every `Send` this returns
MUST be dispatched as one call_async whose done-callback feeds `on_ack` with
the same seq; every `Expire` means "drop the pending future for that seq"
(late acks for an expired seq are ignored here by the seq guard). `now` is
any monotonic seconds source — tests use plain floats, like core.latch.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Send:
    """Dispatch one set_parameters request carrying `state`."""
    seq: int
    state: tuple


@dataclass(frozen=True)
class Expire:
    """The request with this seq timed out — drop its pending future."""
    seq: int


class ZonePush:
    """See module docstring. Sync status is readable as `.synced` (bool)."""

    def __init__(self, timeout_s=2.0):
        self.timeout_s = float(timeout_s)
        self._desired = None    # latest (front, rear, turn) wanted
        self._acked = None      # last state EXPLICITLY acked; None = unknown
        self._inflight = None   # (seq, state, sent_at) or None
        self._seq = 0
        # Assume steady-state ready: transitions are what void `acked`, and a
        # service that was simply up before the first tick is not a restart.
        # (A service genuinely down at boot shows as True->False->True, which
        # voids an acked that is still None — harmless.)
        self._last_ready = True

    @property
    def synced(self):
        """True only when collision_monitor explicitly acked the current
        desired state — never assumed."""
        return self._acked is not None and self._acked == self._desired

    def _mark_send(self, now):
        self._seq += 1
        send = Send(self._seq, self._desired)
        self._inflight = (send.seq, send.state, now)
        return send

    def set_desired(self, state, now, service_ready):
        """Record the latest desired state; returns a Send only when nothing
        is in flight, the service is up, and the state isn't already acked.
        Otherwise it coalesces — on_ack/on_tick chase the latest desired."""
        self._desired = state
        if not service_ready or self._inflight is not None:
            return None
        if state == self._acked:
            return None
        return self._mark_send(now)

    def on_ack(self, seq, success, now):
        """Feed one done-callback result. Stale seqs (expired or superseded
        requests) are ignored. Success may return a follow-up Send chasing a
        desired state that changed while the request was in flight."""
        if self._inflight is None or seq != self._inflight[0]:
            return None
        state = self._inflight[1]
        self._inflight = None
        if not success:
            # Rejected or partial reply: what collision_monitor now holds is
            # unknown (SetParameters can apply a subset). Re-push on tick.
            self._acked = None
            return None
        self._acked = state
        if self._desired != self._acked:
            return self._mark_send(now)
        return None

    def on_tick(self, now, service_ready):
        """Periodic drive (the node's 1 Hz timer): expires a stuck request,
        detects a collision_monitor restart, and retries an unsynced push.
        Returns actions in dispatch order (an Expire is followed by its
        retry Send in the same tick)."""
        actions = []
        if service_ready != self._last_ready:
            # Any readiness transition voids what we thought was acked: a
            # service that went away holds nothing, and one that (re)appeared
            # holds its YAML defaults, not anything pushed before.
            self._acked = None
            if service_ready and self._inflight is not None:
                # A request from before the restart can no longer be trusted
                # even if its ack straggles in — expire it now.
                actions.append(Expire(self._inflight[0]))
                self._inflight = None
        self._last_ready = service_ready
        if self._inflight is not None and now - self._inflight[2] >= self.timeout_s:
            actions.append(Expire(self._inflight[0]))
            self._inflight = None
            self._acked = None
        if (service_ready and self._inflight is None
                and self._desired is not None and self._desired != self._acked):
            actions.append(self._mark_send(now))
        return actions
