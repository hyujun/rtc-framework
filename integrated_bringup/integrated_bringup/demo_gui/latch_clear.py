"""Clear E-STOP / Reset fault header buttons for demo_controller_gui (S9a, D-S9-H).

Two latches, two clears, deliberately separate (E-8, ``rtc_msgs/srv/ClearEstop.srv``
and ``ResetFault.srv``): ``/rtc_cm/clear_estop`` lowers the CM's GLOBAL E-STOP
and ``/rtc_cm/reset_fault`` lowers the ACTIVE controller's own fault latch.
Neither clears the other, and each reply says when the other is still up. The
buttons live in the header next to the E-STOP readout rather than in any one
controller's panel, because both latches belong to whichever controller is
running.

**Clearing the E-STOP is two calls, and the operator is between them.** The
service's authority rule is that the caller echoes the CURRENT latched reason
(``reason_ack``); the reason is on no topic, only in the refusal an empty ack
gets back. So the first call is discovery — it cannot clear anything — and the
second, sent only after the operator has read the reason and confirmed, carries
exactly the string the first reply named. A reply this module cannot parse
ends the flow with the reply shown verbatim: guessing a reason would turn the
confirmation step into a formality, and an empty ack would only be refused
again.

**A clear re-arms nothing.** The catching controller's ``catching.enable`` was
lowered by its own tick when the latch rose, and nothing here writes it back;
re-arming stays an explicit operator action in the catching panel.

Pure Python — no Tk, no rclpy — so this is unit-testable without a display or a
ROS graph, following ``demo_gui.ball_launch`` / ``demo_gui.catching``.

Public surface (imported by app.py):
- CLEAR_ESTOP_SERVICE, RESET_FAULT_SERVICE
- parse_estop_reason, summarize_reply, reset_fault_target
- EstopClearFlow
"""

from __future__ import annotations

from dataclasses import dataclass

CLEAR_ESTOP_SERVICE = "/rtc_cm/clear_estop"
RESET_FAULT_SERVICE = "/rtc_cm/reset_fault"
# A reset_fault call with no reply after this long is presumed lost and no
# longer blocks the button (the CM waits a few control periods, far below it).
RESET_FAULT_LOST_S = 10.0

# The refusal an empty reason_ack gets, verbatim up to the reason. Source:
# rtc_controller_manager/src/rt_controller_node_services.cpp, the clear_estop
# lambda's empty-ack branch:
#   "reason_ack is required — echo the latched reason to confirm: '" +
#   latched_reason + "'"
# Nothing is appended after the closing quote on this branch (the other-latch
# note is only added to the later replies), so the reason is everything between
# the prefix and the final character. The dash is U+2014, as in the source.
_ACK_REQUIRED_PREFIX = "reason_ack is required — echo the latched reason to confirm: '"

# The other-latch notes, as the CM appends them AT REPLY TIME (same source
# file: the clear_estop lambda's fault note, and the reset_fault lambda's
# E-STOP note).
_FAULT_STILL_LATCHED = "still has a latched fault"
_ESTOP_STILL_LATCHED = "global E-STOP still latched"

# clear_estop's one ok=false reply in which the latch IS down: the RT loop
# completed no tick inside the observation window, so the clear is unverified
# (ClearEstop.srv, 4th refusal; same source file, "the latch is DOWN (was: ...")
# Reading it as "refused" is the misreading the CM's own comment warns about —
# an operator who thinks nothing happened re-issues the call and is surprised
# by an arm that is no longer held.
_LATCH_DOWN_UNVERIFIED = "the latch is DOWN"


def parse_estop_reason(message: str) -> str | None:
    """The latched E-STOP reason named by an empty-ack refusal, or None.

    None for anything that is not that refusal — a mismatch refusal, a
    re-latch, a no-op, a format this build does not know — and for an empty
    reason, which cannot be echoed: an empty ack is the discovery call itself.
    """
    if not message.startswith(_ACK_REQUIRED_PREFIX) or not message.endswith("'"):
        return None
    reason = message[len(_ACK_REQUIRED_PREFIX) : -1]
    return reason or None


@dataclass(frozen=True)
class ReplySummary:
    text: str
    # True when the operator should look again: the call was refused, or it
    # succeeded while the OTHER latch is still holding the arm.
    alarm: bool


def summarize_reply(service: str, ok: bool, message: str) -> ReplySummary:
    """One header line for a clear_estop / reset_fault reply.

    The CM's message is kept verbatim — it is what distinguishes the refusal
    cases (ClearEstop.srv / ResetFault.srv) — and only prefixed with the
    service and the verdict. ``ok`` with the other latch still up is flagged,
    so "cleared" is never read as "the arm is free to move" when it is not.
    """
    name = service.rsplit("/", 1)[-1]
    other_up = _FAULT_STILL_LATCHED in message or _ESTOP_STILL_LATCHED in message
    if ok:
        verdict = "OK"
    elif _LATCH_DOWN_UNVERIFIED in message:
        verdict = "LATCH DOWN, UNVERIFIED"
    else:
        verdict = "REFUSED"
    return ReplySummary(text=f"{name}: {verdict} — {message}", alarm=(not ok) or other_up)


def reset_fault_target(entries, active_config_key: str) -> str | None:
    """``controller_name`` for /rtc_cm/reset_fault, or None if unknown.

    reset_fault compares against the controller's ``Name()`` — the
    ``ControllerState.name`` of /rtc_cm/list_controllers — NOT against the
    config key /rtc_cm/active_controller_name carries (the two differ:
    ``DemoJointController`` vs ``demo_joint_controller``). So the name is
    looked up in the catalog by the active config key, falling back to the one
    entry the catalog reports active. A stale catalog costs nothing but a
    refusal: the CM rejects a name that is not the active controller's.
    """
    if active_config_key:
        for e in entries:
            if e.config_key == active_config_key and e.controller_name:
                return e.controller_name
    active = [e for e in entries if e.is_active and e.controller_name]
    if len(active) == 1:
        return active[0].controller_name
    return None


class EstopClearFlow:
    """The two-call clear, as a state object the Tk wiring drives.

    ``begin`` → send an empty ack → ``on_query_reply`` → (operator confirms)
    ``confirm`` → send the returned ack → ``on_clear_reply``. Replies carry the
    generation ``begin`` handed out, so a reply that arrives after the flow was
    abandoned (timed out, cancelled, restarted) is dropped instead of being
    applied to a newer attempt.
    """

    IDLE = "idle"
    QUERYING = "querying"
    AWAITING_CONFIRM = "awaiting_confirm"
    CLEARING = "clearing"

    # A call with no reply after this long is presumed lost (the CM died with
    # the request pending) and no longer blocks a new attempt. The service's
    # own wait is a few tens of control periods, far below this.
    IN_FLIGHT_TIMEOUT_S = 10.0

    def __init__(self) -> None:
        self.state = self.IDLE
        self.reason: str | None = None
        self.generation = 0
        self._sent_at = 0.0

    def begin(self, now: float) -> int | None:
        """Start a discovery call. Returns its generation, or None if busy.

        The discovery request's ``reason_ack`` is always empty.
        """
        in_flight = self.state in (self.QUERYING, self.CLEARING)
        if in_flight and now - self._sent_at < self.IN_FLIGHT_TIMEOUT_S:
            return None
        if self.state == self.AWAITING_CONFIRM:
            return None
        self.generation += 1
        self.state = self.QUERYING
        self.reason = None
        self._sent_at = now
        return self.generation

    def on_query_reply(self, generation: int, ok: bool, message: str) -> ReplySummary | None:
        """Apply the discovery reply.

        Returns None when the flow now waits for the operator (``reason`` is
        set); otherwise the flow is over and the summary says why — nothing
        was latched, or the reply did not name a reason this module can read.
        A stale generation returns None and changes nothing.
        """
        if generation != self.generation or self.state != self.QUERYING:
            return None
        if ok:
            self.state = self.IDLE
            return summarize_reply(CLEAR_ESTOP_SERVICE, ok, message)
        reason = parse_estop_reason(message)
        if reason is None:
            self.state = self.IDLE
            return ReplySummary(
                text=(
                    "clear_estop: latched reason not readable from the reply — nothing "
                    f"was cleared. Reply: {message}"
                ),
                alarm=True,
            )
        self.reason = reason
        self.state = self.AWAITING_CONFIRM
        return None

    def awaiting_confirm(self, generation: int) -> bool:
        """True iff THIS attempt's reply left the flow waiting for the operator
        — a late reply to an older attempt must not raise the dialog."""
        return generation == self.generation and self.state == self.AWAITING_CONFIRM

    def confirm(self, now: float) -> str | None:
        """Operator confirmed: the ``reason_ack`` for the clearing call.

        Exactly the reason the discovery reply named — never re-derived, never
        trimmed. None unless the flow is waiting for this confirmation.
        """
        if self.state != self.AWAITING_CONFIRM or self.reason is None:
            return None
        self.state = self.CLEARING
        self._sent_at = now
        return self.reason

    def cancel(self) -> None:
        """Operator declined. Nothing was sent, so nothing needs undoing."""
        if self.state == self.AWAITING_CONFIRM:
            self.state = self.IDLE
            self.reason = None

    def on_clear_reply(self, generation: int, ok: bool, message: str) -> ReplySummary | None:
        if generation != self.generation or self.state != self.CLEARING:
            return None
        self.state = self.IDLE
        self.reason = None
        return summarize_reply(CLEAR_ESTOP_SERVICE, ok, message)

    def on_call_failed(self, generation: int, error: str) -> ReplySummary | None:
        """The service call itself raised (no reply). Ends the flow."""
        if generation != self.generation or self.state not in (self.QUERYING, self.CLEARING):
            return None
        self.state = self.IDLE
        self.reason = None
        return ReplySummary(text=f"clear_estop: call failed — {error}", alarm=True)
