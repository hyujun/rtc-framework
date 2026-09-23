"""Catching controller panel (dynamic_catching §13 S5).

The panel's job is to show what the CONTROLLER reported, so most of these cases
are about refusing to show anything else: a number for a block the tick did not
compute, an arming state the operator asked for rather than the one the tick
settled on, and a row of zero reject counters that hides the one that is not
zero.

The rendering is pinned through `lines()` rather than through Tk: the widget
half is three `config(text=...)` calls and the interesting part is which
sentence gets built. Visual confirmation is the `verify` skill's job.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from integrated_bringup.demo_gui.ball_launch import FEED_STALE_AFTER_S
from integrated_bringup.demo_gui.catching import (
    MODE_NAMES,
    REASON_NAMES,
    CatchingStatus,
    mode_name,
    reason_name,
)

# The CloudReject enum's order, as SetupCatchingStatePublisher stamps it.
REJECT_NAMES = [
    "none",
    "big_endian",
    "shape",
    "size",
    "missing_field",
    "field_type",
    "field_bounds",
    "frame_id",
    "future_stamp",
    "stamp_overflow",
    "not_evaluated",
    "stale_sequence",
    "inconsistent_id",
    "malformed",
]


def make_msg(**overrides):
    """A CatchingState as the publisher fills it. Duck-typed — the panel reads
    attributes, so this needs no ROS message and no message build."""
    msg = SimpleNamespace(
        mode=3,  # APPROACH
        reason=0,  # NONE
        armed=True,
        estop_active=False,
        fault_latched=False,
        armable=True,
        law_enabled=True,
        tick=1234,
        input_valid=True,
        input_stale=False,
        input_expired=False,
        input_n=8,
        input_generation=77,
        input_snapshot_sequence=11,
        input_age_s=0.012,
        input_horizon_s=0.55,
        input_accept_count=42,
        input_reject_counts=[0] * len(REJECT_NAMES),
        input_reject_names=list(REJECT_NAMES),
        plan_valid=True,
        plan_p_c=[0.5, -0.2, 0.9],
        plan_t_c_s=0.42,
        plan_gamma_f=0.3,
        ref_valid=True,
        ref_saturated=False,
        track_err_rad=0.0123,
        clik_ran=True,
        clik_converged=True,
        clik_status=0,
        clik_iterations=12,
        clik_solve_us=180.0,
        clik_bound_conflict=False,
        qp_fail_streak=0,
    )
    for k, v in overrides.items():
        setattr(msg, k, v)
    return msg


def text(status: CatchingStatus, now_s: float = 100.0) -> str:
    return "\n".join(status.lines(now_s))


# ── The feed itself ─────────────────────────────────────────────────────────


def test_a_feed_that_never_started_says_so_and_does_not_invent_a_mode():
    # Before anything arrives the panel must not show IDLE — that is a real
    # state the controller can be in, and during bring-up "nothing is running"
    # and "it is idle" call for opposite actions.
    status = CatchingStatus()
    out = text(status)
    assert "never received" in out
    # A parked controller also never publishes (A-S5-12): the panel names that
    # cause too, so a parked bring-up does not read as a dead topic.
    assert "parked" in out
    assert "IDLE" not in out


def test_a_stopped_feed_is_marked_without_losing_the_last_body():
    status = CatchingStatus()
    status.update(make_msg(), now_s=100.0)
    fresh = text(status, now_s=100.1)
    assert "STALE FEED" not in fresh
    stale = text(status, now_s=100.0 + FEED_STALE_AFTER_S + 0.1)
    assert "STALE FEED" in stale
    # The body is still shown: the numbers are old, not absent, and hiding them
    # would remove the evidence of what the controller was doing when it stopped.
    assert "APPROACH" in stale


# ── Supervisor + naming ─────────────────────────────────────────────────────


def test_mode_and_reason_are_named():
    status = CatchingStatus()
    status.update(make_msg(mode=9, reason=12), now_s=100.0)
    out = text(status)
    assert "ABORT_SAFE" in out
    assert "TRACK_ERR" in out


def test_an_unnamed_value_shows_the_number_rather_than_unknown():
    # An unnamed value means the GUI and the controller are from different
    # builds, and the NUMBER is what identifies which — "UNKNOWN" would throw
    # away the only diagnostic information in the event.
    assert mode_name(len(MODE_NAMES)) == f"mode:{len(MODE_NAMES)}"
    assert reason_name(len(REASON_NAMES)) == f"reason:{len(REASON_NAMES)}"


@pytest.mark.parametrize(
    ("overrides", "expected"),
    [
        ({"estop_active": True}, True),
        ({"fault_latched": True}, True),
        ({"mode": 9}, True),  # ABORT_SAFE
        ({"mode": 10}, True),  # FAULT
        ({}, False),
    ],
)
def test_alarm_covers_the_states_mode_alone_would_miss(overrides, expected):
    # E-STOP and a latched fault are not modes, so a colour driven by `mode`
    # alone would stay calm through both.
    status = CatchingStatus()
    status.update(make_msg(**overrides), now_s=100.0)
    assert status.alarm() is expected


# ── Arming: observed against requested ──────────────────────────────────────


def test_a_request_is_pending_until_a_later_tick_has_spoken():
    status = CatchingStatus()
    status.update(make_msg(armed=False, tick=10), now_s=100.0)
    status.note_request(True)
    assert status.request_pending
    # In flight is NOT a disagreement — the controller has not answered yet.
    assert not status.arming_disagrees()
    assert "requested ARMED …" in text(status)

    status.update(make_msg(armed=True, tick=11), now_s=100.1)
    assert not status.request_pending
    assert not status.arming_disagrees()
    assert "arm: ARMED (observed)" in text(status)


def test_a_request_the_tick_refuses_outright_stops_pending_and_shows_it():
    """The case an agreement-based rule cannot see.

    An arm pressed while a fault is latched is refused by the tick and the
    latch never reaches the requested value. Waiting for agreement would leave
    the request "in flight" forever — and `arming_disagrees()` suppresses the
    banner while a request is in flight, so the panel would go permanently
    quiet about exactly the state it exists to report.
    """
    status = CatchingStatus()
    status.update(make_msg(armed=False, fault_latched=True, mode=10, tick=50), now_s=100.0)
    status.note_request(True)
    assert status.request_pending

    # The tick runs and says no: still disarmed, fault still latched.
    status.update(make_msg(armed=False, fault_latched=True, mode=10, tick=51), now_s=100.1)
    assert not status.request_pending, "a refused request stayed in flight"
    assert status.arming_disagrees()
    out = text(status)
    assert "arm: DISARMED (observed)" in out
    assert "a latched FAULT lowered it" in out


def test_a_stop_that_lowers_the_latch_is_shown_as_a_disagreement_and_named():
    # The failure this exists for: the operator armed the controller, the tick
    # lowered the latch on an E-STOP (P-1 (c)), and a panel that painted the
    # REQUESTED value would show ARMED while nothing would happen.
    status = CatchingStatus()
    status.update(make_msg(armed=False, tick=1), now_s=100.0)
    status.note_request(True)
    status.update(make_msg(armed=True, tick=2), now_s=100.1)  # the request landed
    assert not status.request_pending

    status.update(make_msg(armed=False, estop_active=True, mode=9, tick=3), now_s=100.2)
    assert status.arming_disagrees()
    out = text(status)
    assert "arm: DISARMED (observed)" in out
    assert "requested ARMED" in out
    assert "E-STOP lowered it" in out
    assert "no automatic resume" in out


def test_a_fault_latch_is_named_as_the_cause_when_there_is_no_stop():
    status = CatchingStatus()
    status.update(make_msg(armed=False, tick=1), now_s=100.0)
    status.note_request(True)
    status.update(make_msg(armed=True, tick=2), now_s=100.1)
    status.update(make_msg(armed=False, fault_latched=True, mode=10, tick=3), now_s=100.2)
    assert "a latched FAULT lowered it" in text(status)


def test_a_profile_that_cannot_arm_says_so_before_the_operator_tries():
    status = CatchingStatus()
    status.update(make_msg(armed=False, armable=False), now_s=100.0)
    assert "profile NOT armable" in text(status)


def test_a_controller_holding_the_arm_by_design_says_the_law_is_not_wired():
    status = CatchingStatus()
    status.update(make_msg(law_enabled=False), now_s=100.0)
    assert "law not wired" in text(status)


# ── The input lane ──────────────────────────────────────────────────────────


def test_a_usable_prediction_reports_its_identity_and_age():
    status = CatchingStatus()
    status.update(make_msg(), now_s=100.0)
    out = text(status)
    assert "n=8" in out
    assert "gen=77" in out
    assert "seq=11" in out
    assert "age=12.0 ms" in out
    assert "horizon=0.550 s" in out


def test_an_unusable_prediction_reports_no_numbers_at_all():
    # PROC-7 zeroes the block, so printing n / gen / horizon here would present
    # zeros as measurements of a snapshot that does not exist.
    status = CatchingStatus()
    status.update(
        make_msg(input_valid=False, input_stale=True, input_n=0, input_horizon_s=0.0),
        now_s=100.0,
    )
    out = text(status)
    assert "no usable prediction" in out
    assert "n=0" not in out
    assert "horizon=0.000" not in out


def test_only_the_reject_counters_that_moved_are_shown():
    counts = [0] * len(REJECT_NAMES)
    counts[REJECT_NAMES.index("frame_id")] = 3
    status = CatchingStatus()
    status.update(make_msg(input_reject_counts=counts), now_s=100.0)
    assert status.reject_summary() == "frame_id=3"
    out = text(status)
    assert "input rejects: frame_id=3" in out
    # And a lane that has refused nothing says nothing — a row of zeroes is
    # what hides the one counter that is not zero.
    clean = CatchingStatus()
    clean.update(make_msg(), now_s=100.0)
    assert clean.reject_summary() == ""
    assert "input rejects" not in text(clean)


def test_a_counter_without_a_name_still_appears():
    # The names are stamped by the publisher; a shorter list than the counter
    # array means a version skew, and dropping the count would hide the very
    # thing that skew produced.
    counts = [0] * len(REJECT_NAMES)
    counts[-1] = 5
    status = CatchingStatus()
    status.update(make_msg(input_reject_counts=counts, input_reject_names=["none"]), now_s=100.0)
    assert status.reject_summary() == f"#{len(REJECT_NAMES) - 1}=5"


# ── The law ─────────────────────────────────────────────────────────────────


def test_a_tick_that_did_not_solve_never_claims_status_zero():
    # 0 is ProxQP's SOLVED. A tick that did not reach the solve carries −1, and
    # a panel that printed "status 0" for it would report a successful solve.
    status = CatchingStatus()
    status.update(
        make_msg(
            clik_ran=False,
            clik_converged=False,
            clik_status=-1,
            clik_solve_us=0.0,
            ref_valid=False,
            track_err_rad=0.0,
        ),
        now_s=100.0,
    )
    out = text(status)
    assert "not solved this tick" in out
    assert "status 0" not in out
    assert "SOLVED" not in out
    # And the tracking error of a tick that computed none is not zero, it is
    # unknown.
    assert "track err -- rad" in out


def test_a_failed_solve_reports_the_status_and_the_streak():
    status = CatchingStatus()
    status.update(
        make_msg(clik_converged=False, clik_status=1, qp_fail_streak=2, clik_bound_conflict=True),
        now_s=100.0,
    )
    out = text(status)
    assert "FAILED (status 1)" in out
    assert "QP fail streak 2" in out
    assert "BOUND CONFLICT" in out


def test_a_saturated_reference_is_called_out():
    status = CatchingStatus()
    status.update(make_msg(ref_saturated=True), now_s=100.0)
    assert "reference SATURATED" in text(status)


# ── The plan ────────────────────────────────────────────────────────────────


def test_no_plan_is_said_rather_than_drawn_as_the_origin():
    status = CatchingStatus()
    status.update(make_msg(plan_valid=False, plan_p_c=[0.0, 0.0, 0.0]), now_s=100.0)
    out = text(status)
    assert "plan: none" in out
    assert "p_c=" not in out


def test_a_plan_reports_its_catch_point_and_instant():
    status = CatchingStatus()
    status.update(make_msg(), now_s=100.0)
    out = text(status)
    assert "p_c=(+0.500, -0.200, +0.900)" in out
    assert "t_c=+0.420 s" in out
    assert "gamma_f=0.30" in out
