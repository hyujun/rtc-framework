"""Clear E-STOP / Reset fault header buttons (S9a, D-S9-H).

Three contracts are pinned here, all about the GUI never clearing a latch the
operator did not see:

* the latched reason is read from the CM's REAL refusal text, and a reply that
  is not that refusal yields no reason at all — the GUI must not guess one;
* the confirming call carries exactly the reason the first reply named;
* reset_fault is addressed by the controller's ``Name()``, not by the config key
  the active-controller topic carries (the CM compares against ``Name()``).
"""

import re
from pathlib import Path
from types import SimpleNamespace

import pytest

from integrated_bringup.demo_gui.catalog import build_entries
from integrated_bringup.demo_gui.latch_clear import (
    CLEAR_ESTOP_SERVICE,
    RESET_FAULT_SERVICE,
    EstopClearFlow,
    parse_estop_reason,
    reset_fault_target,
    summarize_reply,
)

_SERVICES_CPP = (
    Path(__file__).resolve().parents[2]
    / "rtc_controller_manager"
    / "src"
    / "rt_controller_node_services.cpp"
)

# The CM's empty-ack refusal, built the way the CM builds it. Source:
# rtc_controller_manager/src/rt_controller_node_services.cpp (clear_estop's
# empty-ack branch)
#   resp->message = "reason_ack is required — echo the latched reason to confirm: '" +
#                   latched_reason + "'";
_PREFIX = "reason_ack is required — echo the latched reason to confirm: '"


def _refusal(reason: str) -> str:
    return _PREFIX + reason + "'"


def test_the_prefix_is_the_one_the_cm_writes():
    """The format above is a copy; this is what keeps it one. If the CM's
    wording changes, every clear would end at "reason not readable" — safe, but
    the button would be dead, so the drift is caught here instead."""
    if not _SERVICES_CPP.exists():  # pragma: no cover - 단독 배포 시
        pytest.skip(f"CM source not present at {_SERVICES_CPP}")
    src = _SERVICES_CPP.read_text(encoding="utf-8")
    assert f'"{_PREFIX}"' in src
    # The refusal ends at the reason's closing quote: nothing is appended on
    # that branch, which is what lets the parser take everything up to it.
    # Whitespace-tolerant, so a clang-format re-wrap is not a failure.
    tail = re.escape(f'"{_PREFIX}"') + r"\s*\+\s*latched_reason\s*\+\s*\"'\";"
    assert re.search(tail, src), "the empty-ack refusal no longer ends at the reason"


@pytest.mark.parametrize(
    "reason",
    [
        "device timeout: ur5e",
        "consecutive overrun (12 ticks)",
        # Quotes and the em dash inside the reason must survive: the parser
        # strips only the one closing quote the CM adds.
        "output invalid: 'q_cmd' non-finite — joint 3",
        "x",
    ],
)
def test_the_reason_is_read_from_the_real_refusal(reason):
    assert parse_estop_reason(_refusal(reason)) == reason


@pytest.mark.parametrize(
    "message",
    [
        "",
        # The other replies of the same service (rt_controller_node_services.cpp
        # :299, :319-320, :378-380, :400). None of them is a request to confirm.
        "global E-STOP is not latched (no-op)",
        "reason_ack 'a' does not match the latched reason 'b'",
        "cleared, then re-latched within the observation window — the cause is still present: 'b'",
        "global E-STOP cleared (was: 'b')",
        # Truncated / reworded / hyphen instead of the em dash.
        _PREFIX + "device timeout",
        "reason_ack is required - echo the latched reason to confirm: 'b'",
        # An empty reason cannot be echoed: an empty ack IS the discovery call.
        _refusal(""),
    ],
)
def test_anything_else_yields_no_reason(message):
    assert parse_estop_reason(message) is None


def test_the_second_call_carries_exactly_the_parsed_reason():
    reason = "device timeout: 'hand' (last 0.51 s ago)"
    flow = EstopClearFlow()
    gen = flow.begin(now=0.0)
    assert gen is not None
    assert flow.on_query_reply(gen, False, _refusal(reason)) is None
    assert flow.awaiting_confirm(gen)
    assert flow.reason == reason
    ack = flow.confirm(now=0.1)
    assert ack == reason
    summary = flow.on_clear_reply(gen, True, f"global E-STOP cleared (was: '{reason}')")
    assert summary is not None and not summary.alarm
    assert flow.state == EstopClearFlow.IDLE


def test_an_unreadable_reply_ends_the_flow_without_a_guess():
    flow = EstopClearFlow()
    gen = flow.begin(now=0.0)
    reply = "reason_ack is required (reworded in a later CM)"
    summary = flow.on_query_reply(gen, False, reply)
    assert summary is not None and summary.alarm
    assert reply in summary.text, "the CM's own reply must be shown verbatim"
    assert "nothing was cleared" in summary.text
    assert not flow.awaiting_confirm(gen)
    assert flow.confirm(now=0.1) is None, "no second call may follow an unreadable reply"


def test_nothing_latched_ends_the_flow_at_the_first_reply():
    flow = EstopClearFlow()
    gen = flow.begin(now=0.0)
    summary = flow.on_query_reply(gen, True, "global E-STOP is not latched (no-op)")
    assert summary is not None and not summary.alarm
    assert flow.confirm(now=0.1) is None


def test_declining_sends_nothing():
    flow = EstopClearFlow()
    gen = flow.begin(now=0.0)
    flow.on_query_reply(gen, False, _refusal("r"))
    flow.cancel()
    assert flow.state == EstopClearFlow.IDLE
    assert flow.confirm(now=0.1) is None


def test_a_late_reply_to_an_abandoned_attempt_is_dropped():
    flow = EstopClearFlow()
    old = flow.begin(now=0.0)
    # No reply; after the timeout a new attempt may start.
    assert flow.begin(now=1.0) is None, "a call in flight blocks a second one"
    new = flow.begin(now=EstopClearFlow.IN_FLIGHT_TIMEOUT_S + 1.0)
    assert new is not None and new != old
    flow.on_query_reply(new, False, _refusal("new reason"))
    # The old reply must neither overwrite the reason nor raise a dialog.
    assert flow.on_query_reply(old, False, _refusal("old reason")) is None
    assert not flow.awaiting_confirm(old)
    assert flow.reason == "new reason"
    assert flow.begin(now=100.0) is None, "a pending confirmation blocks a new attempt"


def test_a_failed_call_ends_the_flow():
    flow = EstopClearFlow()
    gen = flow.begin(now=0.0)
    summary = flow.on_call_failed(gen, "context shut down")
    assert summary is not None and summary.alarm
    assert flow.begin(now=0.1) is not None


def test_a_clear_that_leaves_the_controller_fault_up_is_flagged():
    """ok=True is not "the arm is free to move" while the other latch holds it.
    Note format: rt_controller_node_services.cpp, clear_estop's fault note."""
    msg = (
        "global E-STOP cleared (was: 'x') (controller 'DemoCatchingController' still has a "
        "latched fault — /rtc_cm/reset_fault clears that one)"
    )
    summary = summarize_reply(CLEAR_ESTOP_SERVICE, True, msg)
    assert summary.alarm
    assert msg in summary.text
    assert summary.text.startswith("clear_estop: OK")


def test_a_reset_that_leaves_the_estop_up_is_flagged():
    """Note format: rt_controller_node_services.cpp, reset_fault's E-STOP note."""
    msg = "fault latch cleared on 'DemoCatchingController' (global E-STOP still latched — clear it separately)"
    summary = summarize_reply(RESET_FAULT_SERVICE, True, msg)
    assert summary.alarm
    assert summary.text.startswith("reset_fault: OK")


def test_a_clear_whose_latch_is_down_but_unverified_is_not_called_refused():
    """ok=False with the latch DOWN (the RT loop ran no tick in the window):
    "REFUSED" would tell the operator nothing happened, and the next press would
    meet an arm that is no longer held. Still an alarm — it is unverified."""
    msg = (
        "the latch is DOWN (was: 'x') but unverified — the RT loop completed no tick "
        "within the deadline, so no detector re-evaluated the cause"
    )
    summary = summarize_reply(CLEAR_ESTOP_SERVICE, False, msg)
    assert summary.alarm
    assert summary.text.startswith("clear_estop: LATCH DOWN, UNVERIFIED")
    assert "REFUSED" not in summary.text


def test_the_unverified_wording_is_the_one_the_cm_writes():
    if not _SERVICES_CPP.exists():  # pragma: no cover - 단독 배포 시
        pytest.skip(f"CM source not present at {_SERVICES_CPP}")
    assert '"the latch is DOWN (was: \'"' in _SERVICES_CPP.read_text(encoding="utf-8")


def test_a_refusal_is_flagged_and_a_clean_reset_is_not():
    assert summarize_reply(RESET_FAULT_SERVICE, False, "no active controller").alarm
    assert not summarize_reply(
        RESET_FAULT_SERVICE, True, "fault latch cleared on 'DemoCatchingController'"
    ).alarm


def _cs(name, key, *, is_active=False):
    return SimpleNamespace(
        name=name,
        type=key,
        state="active" if is_active else "inactive",
        is_active=is_active,
        claimed_groups=[],
    )


def test_reset_fault_is_addressed_by_name_not_config_key():
    """/rtc_cm/active_controller_name carries the config key; reset_fault
    compares against Name(). Sending the key would always be refused."""
    entries = build_entries(
        [
            _cs("DemoJointController", "demo_joint_controller"),
            _cs("DemoCatchingController", "demo_catching_controller", is_active=True),
        ],
        schema_keys=(),
    )
    assert reset_fault_target(entries, "demo_catching_controller") == "DemoCatchingController"
    # Before the latched topic arrived: the one entry the catalog reports active.
    assert reset_fault_target(entries, "") == "DemoCatchingController"


def test_reset_fault_target_is_none_when_unknown():
    assert reset_fault_target((), "demo_catching_controller") is None
    assert reset_fault_target((), "") is None
    # Two entries claiming to be active is not a name the GUI can pick from.
    entries = build_entries(
        [_cs("A", "a", is_active=True), _cs("B", "b", is_active=True)], schema_keys=()
    )
    assert reset_fault_target(entries, "") is None
