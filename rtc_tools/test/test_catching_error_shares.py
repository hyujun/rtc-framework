"""catching_trials — the t_c gap in the catch frame and the last segment's wake (#807).

Every quantity is recovered from a trial whose parts are KNOWN (an offset
injected in the catch frame, a crossing at a chosen instant, a snapshot that
arrives a chosen number of ticks after a wake), and each has a positive
control: the same trial read the wrong way must give a different answer.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from rtc_tools.analysis import catching_trials as ct

DT = 0.002


# ── The hand's capture set ───────────────────────────────────────────────────


def _docking_yaml(**over):
    """A 10 × 20 mm box around ``rho_ref`` (−5, −5) mm, entrance plane at 7 mm."""
    lateral = {
        "n_faces": 4,
        "faces_a": [1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0],
        "faces_b": [0.0, 0.005, 0.010, 0.015],
        "rho_ref": [-0.005, -0.005],
    }
    doc = {"s_ent": 0.007, "lateral": lateral}
    for key, value in over.items():
        (lateral if key in lateral else doc)[key] = value
    return doc


def _profile(docking):
    return SimpleNamespace(hand_yaml={} if docking is None else {"docking": docking})


DOCKING = ct.hand_docking(_profile(_docking_yaml()))


def test_hand_docking_reads_the_entrance_plane_and_the_lateral_set():
    assert DOCKING.s_ent == 0.007
    assert DOCKING.faces_a.shape == (4, 2) and DOCKING.faces_b.shape == (4,)
    # rho_ref is 5 mm from the nearest faces of the box it sits in.
    assert DOCKING.lateral_margin(DOCKING.rho_ref) == pytest.approx(0.005)
    assert DOCKING.lateral_margin([-0.009, -0.005]) == pytest.approx(0.001)
    assert DOCKING.lateral_margin([0.003, -0.005]) == pytest.approx(-0.003)


@pytest.mark.parametrize(
    "docking",
    [
        None,
        _docking_yaml(s_ent="TBD"),
        _docking_yaml(rho_ref=["TBD", "TBD"]),
        _docking_yaml(faces_b=[0.0, 0.005, 0.010]),  # one short of n_faces
        _docking_yaml(faces_a=[1.0, 0.0, 0.0, 1.0, -1.0, 0.0]),
        _docking_yaml(n_faces=0),
        {"s_ent": 0.007},  # no lateral set at all
    ],
)
def test_a_hand_without_a_complete_set_has_none(docking):
    assert ct.hand_docking(_profile(docking)) is None


# ── The gap as vectors in the catch frame ────────────────────────────────────


def _frame(v_ball):
    """A catch frame whose +z faces the ball (the ball travels toward −z)."""
    e3 = -np.asarray(v_ball, dtype=float) / np.linalg.norm(v_ball)
    e1 = np.cross([0.0, 0.0, 1.0], e3)
    e1 /= np.linalg.norm(e1)
    return np.column_stack([e1, np.cross(e3, e1), e3])


class _FrameFk:
    """ "Joints" that are the catch frame's position, with a fixed orientation."""

    def __init__(self, rotation):
        self.rotation = np.asarray(rotation, dtype=float)

    def __call__(self, q):
        return np.asarray(q, dtype=float)

    def placement(self, q):
        return self.rotation, np.asarray(q, dtype=float)

    def to_model(self, p):
        return np.asarray(p, dtype=float)


V_BALL = np.array([-2.0, 0.3, -0.4])
P_CATCH = np.array([0.6, 0.1, 0.4])  # the ball at the instant it is meant to cross
T_X = 0.8
ROT = _frame(V_BALL)
# Injected, in the catch frame (x, y, s) [m].
BALL_IN_REF = np.array([-0.005, -0.005, 0.007])  # where the plan puts the ball: on its target
CLIK = np.array([0.002, 0.0, 0.001])
SERVO = np.array([-0.004, 0.006, 0.003])


def _line_truth(t_cross=T_X, velocity=V_BALL):
    """A ball on a straight line through ``P_CATCH`` at ``t_cross``."""
    tt = np.arange(0.0, 1.2, 0.01)
    velocity = np.asarray(velocity, dtype=float)
    return ct.Truth(
        tt, P_CATCH + np.outer(tt - t_cross, velocity), np.tile(velocity, (len(tt), 1))
    )


def _static_ctx(*, rotation=ROT, lead=0.0, hand_velocity=None):
    """A hand at rest (or moving at ``hand_velocity``) whose reference, command
    and measurement differ by the injected catch-frame offsets at ``T_X``."""
    t = np.arange(0.0, 1.2, DT)
    n = len(t)
    mode = np.full(n, ct.MODE_APPROACH)
    mode[100:] = ct.MODE_COMMITTED
    ref_point = P_CATCH - ROT @ BALL_IN_REF
    drift = np.zeros((n, 3)) if hand_velocity is None else np.outer(t - T_X, hand_velocity)
    ref = ref_point + drift
    q_cmd = ref - ROT @ CLIK
    q_meas = q_cmd - ROT @ SERVO
    return ct.TrialContext(
        t=t,
        mode=mode,
        hand_phase=np.zeros(n, int),
        plan_id=np.ones(n, int),
        plan_t_c=T_X - t,
        plan_p_c=np.tile(P_CATCH, (n, 1)),
        plan_gamma_f=np.full(n, 0.4),
        ref=np.zeros((n, 3)),
        ref_valid=np.zeros(n, bool),
        ref_saturated=np.zeros(n, bool),
        q_cmd=q_cmd,
        q_meas=q_meas,
        fk=_FrameFk(rotation),
        t_lead=np.full(n, lead),
        segment_following=np.ones(n, bool),
        segment_p_d=ref,
    )


def _terms(rec, name):
    return np.array([rec[f"cf_{name}_{axis}_mm"] for axis in "xys"])


def test_the_shares_are_the_injected_offsets_in_the_catch_frame():
    rec = ct.catch_frame_shares(_static_ctx(), _line_truth(), T_X, 0.0, DOCKING, 0.12)
    assert set(rec) == set(ct.CATCH_FRAME_KEYS)
    assert _terms(rec, "ref") == pytest.approx(BALL_IN_REF * 1e3, abs=1e-6)
    assert _terms(rec, "clik") == pytest.approx(CLIK * 1e3, abs=1e-6)
    assert _terms(rec, "servo") == pytest.approx(SERVO * 1e3, abs=1e-6)
    # The ball in the measured hand is the sum, and its length is total_mm.
    assert _terms(rec, "tot") == pytest.approx((BALL_IN_REF + CLIK + SERVO) * 1e3, abs=1e-6)
    whole = ct.decompose_at_tc(_static_ctx(), _line_truth(), 0.12)
    assert np.linalg.norm(_terms(rec, "tot")) == pytest.approx(whole["total_mm"], abs=1e-6)
    assert np.linalg.norm(_terms(rec, "servo")) == pytest.approx(whole["servo_mm"], abs=1e-6)
    # The reference has the ball on rho_ref (5 mm inside); the hand that got
    # there has it (−7, +1) mm: 2 mm past the x face and 4 mm inside y.
    assert rec["cf_ref_lateral_margin_mm"] == pytest.approx(5.0, abs=1e-6)
    assert rec["cf_tot_lateral_margin_mm"] == pytest.approx(3.0, abs=1e-6)


def test_the_same_trial_read_in_the_world_axes_gives_other_components():
    """Positive control: without the frame's rotation the 7 mm of approach
    offset is spread over three axes and nothing lands on ``s``."""
    world = ct.catch_frame_shares(
        _static_ctx(rotation=np.eye(3)), _line_truth(), T_X, 0.0, DOCKING, 0.12
    )
    assert np.linalg.norm(_terms(world, "ref")) == pytest.approx(np.linalg.norm(BALL_IN_REF) * 1e3)
    assert abs(world["cf_ref_s_mm"] - 7.0) > 2.0


def test_the_command_side_is_read_at_the_lead_tick():
    # A hand moving with the lead on: the command aimed at T_X was written
    # `lead` earlier, and read at T_X it is 0.2 s × 0.5 m/s further along.
    drift = np.array([0.0, 0.5, 0.0])
    lead = 0.2
    ctx = _static_ctx(lead=lead, hand_velocity=drift)
    ctx.q_cmd = ctx.q_cmd + drift * lead  # aimed `lead` ahead
    ctx.segment_p_d = ctx.segment_p_d + drift * lead
    rec = ct.catch_frame_shares(ctx, _line_truth(), T_X, lead, DOCKING, 0.12)
    assert _terms(rec, "clik") == pytest.approx(CLIK * 1e3, abs=1e-6)
    assert _terms(rec, "servo") == pytest.approx(SERVO * 1e3, abs=1e-6)
    no_lead = ct.catch_frame_shares(ctx, _line_truth(), T_X, 0.0, DOCKING, 0.12)
    assert np.linalg.norm(_terms(no_lead, "servo")) > 90.0


def test_without_a_commit_a_truth_or_a_set_the_columns_are_nan():
    ctx = _static_ctx()
    nothing = ct.catch_frame_shares(ctx, _line_truth(), math.nan, 0.0, DOCKING, 0.12)
    assert set(nothing) == set(ct.CATCH_FRAME_KEYS)
    assert all(math.isnan(v) for v in nothing.values())
    assert all(
        math.isnan(v) for v in ct.catch_frame_shares(ctx, None, T_X, 0.0, DOCKING, 0.12).values()
    )
    no_set = ct.catch_frame_shares(ctx, _line_truth(), T_X, 0.0, None, 0.12)
    assert math.isfinite(no_set["cf_tot_s_mm"])
    assert math.isnan(no_set["cf_tot_lateral_margin_mm"]) and math.isnan(no_set["ent_cross_ms"])
    # A tick that followed no segment has no reference: the two terms that read it are NaN.
    ctx.segment_following = np.zeros(len(ctx.t), bool)
    bare = ct.catch_frame_shares(ctx, _line_truth(), T_X, 0.0, DOCKING, 0.12)
    assert math.isnan(bare["cf_ref_s_mm"]) and math.isnan(bare["cf_clik_x_mm"])
    assert math.isfinite(bare["cf_servo_s_mm"]) and math.isfinite(bare["cf_tot_s_mm"])


# ── The crossing of the entrance plane ───────────────────────────────────────


def _crossing_ctx(hand_velocity=None):
    """A measured hand that has the ball at catch-frame (3, −4, s_ent) mm at T_X."""
    t = np.arange(0.0, 1.2, DT)
    n = len(t)
    hand = P_CATCH - ROT @ np.array([0.003, -0.004, DOCKING.s_ent])
    if hand_velocity is not None:
        hand = hand + np.outer(t - T_X, hand_velocity)
    q = np.broadcast_to(hand, (n, 3)).copy()
    return ct.TrialContext(
        t=t,
        mode=np.full(n, ct.MODE_COMMITTED),
        hand_phase=np.zeros(n, int),
        plan_id=np.ones(n, int),
        plan_t_c=T_X - t,
        plan_p_c=np.tile(P_CATCH, (n, 1)),
        plan_gamma_f=np.full(n, 0.4),
        ref=np.zeros((n, 3)),
        ref_valid=np.zeros(n, bool),
        ref_saturated=np.zeros(n, bool),
        q_cmd=q,
        q_meas=q,
        fk=_FrameFk(ROT),
    )


def test_the_crossing_is_found_where_and_when_it_was_planted():
    speed = float(np.linalg.norm(V_BALL))
    # The plan's t_c 6 ms before the ball really crosses.
    rec = ct.entrance_crossing(_crossing_ctx(), _line_truth(), T_X - 0.006, DOCKING.s_ent, 0.12)
    assert rec["ent_cross_ms"] == pytest.approx(6.0, abs=1e-6)
    assert (rec["ent_x_mm"], rec["ent_y_mm"]) == pytest.approx((3.0, -4.0), abs=1e-6)
    assert rec["ent_c_m_s"] == pytest.approx(speed, abs=1e-9)
    assert rec["ent_v_perp_m_s"] == pytest.approx(0.0, abs=1e-9)
    # Positive control: the plane 20 mm further out is crossed 20 mm / speed earlier.
    far = ct.entrance_crossing(_crossing_ctx(), _line_truth(), T_X, DOCKING.s_ent + 0.020, 0.12)
    assert far["ent_cross_ms"] == pytest.approx(-20.0 / speed, abs=1e-6)


def test_the_rates_at_the_crossing_are_relative_to_a_moving_hand():
    # The hand backs away along the ball's travel at 0.5 m/s and slides at 0.3 m/s.
    speed = float(np.linalg.norm(V_BALL))
    hand_velocity = -0.5 * ROT[:, 2] + 0.3 * ROT[:, 0]
    rec = ct.entrance_crossing(
        _crossing_ctx(hand_velocity), _line_truth(), T_X, DOCKING.s_ent, 0.12
    )
    assert rec["ent_cross_ms"] == pytest.approx(0.0, abs=1e-6)
    assert rec["ent_c_m_s"] == pytest.approx(speed - 0.5, abs=1e-9)
    assert rec["ent_v_perp_m_s"] == pytest.approx(0.3, abs=1e-9)


def test_a_ball_that_never_passes_the_plane_has_no_crossing():
    away = _line_truth(velocity=-V_BALL)  # through the same point, leaving the hand
    rec = ct.entrance_crossing(_crossing_ctx(), away, T_X, DOCKING.s_ent, 0.12)
    assert all(math.isnan(v) for v in rec.values())
    # ...and one whose crossing is outside the window around t_c.
    late = ct.entrance_crossing(_crossing_ctx(), _line_truth(), T_X - 0.3, DOCKING.s_ent, 0.12)
    assert all(math.isnan(v) for v in late.values())


def test_the_shares_carry_the_crossing_and_its_margin():
    rec = ct.catch_frame_shares(_crossing_ctx(), _line_truth(), T_X, 0.0, DOCKING, 0.12)
    assert rec["ent_cross_ms"] == pytest.approx(0.0, abs=1e-6)
    # (3, −4) mm is 3 mm past the x <= 0 face of the set.
    assert rec["ent_lateral_margin_mm"] == pytest.approx(-3.0, abs=1e-6)
    assert rec["cf_tot_s_mm"] == pytest.approx(7.0, abs=1e-6)


# ── Which snapshot a wake read ───────────────────────────────────────────────


def _snapshot_ctx(arrivals, *, columns=True):
    """200 ticks of 2 ms; snapshot ``k`` (1-based) is first read at tick ``arrivals[k-1]``."""
    t = np.arange(200) * DT
    seq = np.zeros(200)
    for number, tick in enumerate(arrivals, start=1):
        seq[tick:] = number
    return SimpleNamespace(
        t=t,
        input_seq=seq if columns else None,
        input_gen=np.ones(200) if columns else None,
        input_age=None,
    )


def test_a_wake_between_two_arrivals_reads_the_snapshot_of_its_ticks():
    ctx = _snapshot_ctx([10, 50, 90])
    assert ct.wake_snapshot(ctx, 60 * DT + 1e-4, 0.0) == (60, ct.WAKE_JOIN_EXACT)
    assert ctx.input_seq[60] == 2


def test_a_snapshot_that_arrives_just_after_the_wake_is_the_one_it_read():
    # The planner wakes on the arrival and the tick reads it 2 ticks (4 ms) later.
    ctx = _snapshot_ctx([10, 50, 90])
    tick, join = ct.wake_snapshot(ctx, 48 * DT + 1e-4, 0.0)
    assert (tick, join) == (50, ct.WAKE_JOIN_AMBIGUOUS) and ctx.input_seq[tick] == 2
    # Positive control: past the tolerance (4 ticks = 8 ms) it is the older one.
    tick, join = ct.wake_snapshot(ctx, 46 * DT + 1e-4, 0.0)
    assert (tick, join) == (46, ct.WAKE_JOIN_EXACT) and ctx.input_seq[tick] == 1
    # ...and with no tolerance the first wake reads the older one too.
    assert ct.wake_snapshot(ctx, 48 * DT + 1e-4, 0.0, tolerance_s=0.0)[1] == ct.WAKE_JOIN_EXACT


def test_the_planners_own_record_outranks_the_ticks():
    ctx = _snapshot_ctx([10, 50, 90])
    # It recorded snapshot 2 and a tick of the span has it.
    assert ct.wake_snapshot(ctx, 48 * DT + 1e-4, 2.0) == (50, ct.WAKE_JOIN_PLANNER)
    assert ct.wake_snapshot(ctx, 60 * DT + 1e-4, 2.0) == (60, ct.WAKE_JOIN_PLANNER)
    # It recorded one no tick near the wake carries: the wake is off this axis.
    assert ct.wake_snapshot(ctx, 60 * DT + 1e-4, 3.0) == (None, ct.WAKE_JOIN_CONFLICT)


def test_a_wake_with_nothing_to_join_says_so():
    ctx = _snapshot_ctx([10, 50, 90])
    assert ct.wake_snapshot(ctx, -0.01, 0.0) == (None, ct.WAKE_JOIN_NONE)
    assert ct.wake_snapshot(ctx, 1.0, 0.0) == (None, ct.WAKE_JOIN_NONE)
    assert ct.wake_snapshot(ctx, math.nan, 0.0) == (None, ct.WAKE_JOIN_NONE)
    assert ct.wake_snapshot(ctx, 4 * DT, 0.0) == (None, ct.WAKE_JOIN_NONE)  # before any snapshot
    assert ct.wake_snapshot(_snapshot_ctx([10], columns=False), 0.1, 0.0)[1] == ct.WAKE_JOIN_NONE


# ── The wake of the segment the arm was on at t_c ────────────────────────────

N_ELASTIC = len(ct.planner_solves.DOCKING_ELASTIC_GROUPS)
N_VIOL = len(ct.planner_solves.DOCKING_ROW_GROUPS)


def _events(rows):
    """``rows``: (wake_t, seq, published, kind) → (wakes_t, SegmentEvents)."""
    n = len(rows)
    elastic = np.zeros((n, N_ELASTIC))
    elastic[:, ct.planner_solves.DOCKING_ELASTIC_GROUPS.index("lateral")] = 0.002
    viol = np.zeros((n, N_VIOL))
    events = ct.SegmentEvents(
        seq=np.array([r[1] for r in rows], float),
        published=np.array([r[2] for r in rows], bool),
        kind=np.array([r[3] for r in rows], object),
        k=np.full(n, -2.0),
        solve_us=np.arange(n) * 1000.0 + 4000.0,
        slack_c=np.full(n, 0.01),
        slack_v=np.full(n, 0.3),
        elastic=elastic,
        viol=viol,
        snapshot_seq=np.zeros(n),
    )
    return np.array([r[0] for r in rows], float), events


def _segment_ctx(seq_at_lead=7, *, following=True):
    ctx = _static_ctx(lead=0.05)
    n = len(ctx.t)
    ctx.segment_seq = np.full(n, seq_at_lead)
    ctx.segment_following = np.full(n, following)
    ctx.input_seq = np.full(n, 40.0)
    ctx.input_gen = np.ones(n)
    ctx.input_age = np.full(n, 0.004)
    return ctx


def test_the_last_segments_wake_is_the_row_that_published_it_in_this_trial():
    wakes_t, events = _events(
        [
            (-5.0, 7, True, "first"),  # the same number in an earlier trial's rows
            (0.40, 6, True, "first"),
            (0.501, 7, True, "advance"),
            (0.55, 7, False, "same"),  # a row with the number that published nothing
            (0.90, 7, True, "same"),  # after t_c: not what the arm was on
        ]
    )
    rec = ct.last_segment_columns(_segment_ctx(), T_X, 0.05, wakes_t, events)
    assert set(rec) == set(ct.LAST_SEGMENT_KEYS)
    assert (rec["last_seg_seq"], rec["last_seg_kind"], rec["last_seg_k"]) == (7, "advance", -2.0)
    assert rec["last_seg_wake_to_tc_ms"] == pytest.approx(299.0)
    assert rec["last_seg_solve_ms"] == pytest.approx(6.0)  # the third row's
    assert (rec["last_seg_slack_c"], rec["last_seg_slack_v"]) == (0.01, 0.3)
    assert (rec["last_seg_elastic_max"], rec["last_seg_elastic_group"]) == (0.002, "lateral")
    assert (rec["last_seg_viol_max"], rec["last_seg_viol_group"]) == (0.0, "")
    # The wake's snapshot is the one its ticks read: received 4 ms before the
    # tick at 0.500, so 5 ms before the wake.
    assert (rec["last_seg_wake_join"], rec["last_seg_snapshot_seq"]) == ("exact", 40.0)
    assert rec["last_seg_pred_age_ms"] == pytest.approx(5.0, abs=1e-6)


def test_the_segment_is_the_one_followed_at_the_lead_tick():
    # The arm switched to segment 8 one tick after the lead tick: 7 is still the answer.
    ctx = _segment_ctx()
    kl, _ = ct.lead_tick(ctx, T_X, 0.05)
    ctx.segment_seq[kl + 1 :] = 8
    wakes_t, events = _events([(0.50, 7, True, "advance"), (0.60, 8, True, "same")])
    assert ct.last_segment_columns(ctx, T_X, 0.05, wakes_t, events)["last_seg_seq"] == 7
    # Positive control: read without the lead it is the later segment.
    assert ct.last_segment_columns(ctx, T_X, 0.0, wakes_t, events)["last_seg_seq"] == 8


def test_a_planner_record_no_tick_carries_is_a_conflict_and_is_kept():
    wakes_t, events = _events([(0.50, 7, True, "first")])
    events = events._replace(snapshot_seq=np.array([55.0]))
    rec = ct.last_segment_columns(_segment_ctx(), T_X, 0.05, wakes_t, events)
    assert (rec["last_seg_wake_join"], rec["last_seg_snapshot_seq"]) == ("conflict", 55.0)
    assert math.isnan(rec["last_seg_pred_age_ms"])


@pytest.mark.parametrize(
    "case", ["no_events", "no_wakes", "not_following", "no_commit", "no_row", "other_trial"]
)
def test_without_a_followed_segment_or_its_wake_the_columns_are_empty(case):
    wakes_t, events = _events([(0.50, 7, True, "first")])
    ctx, t_c = _segment_ctx(following=case != "not_following"), T_X
    if case == "no_events":
        events = None
    elif case == "no_wakes":
        wakes_t = None
    elif case == "no_commit":
        t_c = math.nan
    elif case == "no_row":
        ctx = _segment_ctx(seq_at_lead=9)
    elif case == "other_trial":  # the number was published, before this trial's rows
        wakes_t, events = _events([(-5.0, 7, True, "first")])
    rec = ct.last_segment_columns(ctx, t_c, 0.05, wakes_t, events)
    assert set(rec) == set(ct.LAST_SEGMENT_KEYS)
    text = ("last_seg_kind", "last_seg_elastic_group", "last_seg_viol_group", "last_seg_wake_join")
    assert all(rec[k] == "" for k in text)
    assert all(math.isnan(v) for k, v in rec.items() if k not in text)


def test_the_segment_events_are_read_as_far_as_the_log_has_them(tmp_path):
    pd = pytest.importorskip("pandas")
    ctl = tmp_path
    frame = pd.DataFrame(
        {
            "wake_ns": [1, 2, 3],
            "snapshot_sequence": [5, 0, 0],
            "segment_outcome": ["published", "off", "published"],
            "segment_kind": ["first", "none", "same"],
            "segment_seq": [1, 0, 2],
            "segment_k": [-3, 0, -2],
            "segment_solve_us": [30.0, 0.0, 5000.0],
            "segment_slack_max": [0.0, 0.0, 0.02],
            "segment_viol_lateral": [0.0, 0.0, 1e-7],
        }
    )
    frame.to_csv(ctl / ct.PLANNER_EVENTS_CSV, index=False)
    events = ct._planner_segment_events(ctl)
    assert events.published.tolist() == [True, False, True]
    assert events.kind.tolist() == ["first", "none", "same"]
    assert events.seq.tolist() == [1.0, 0.0, 2.0] and events.snapshot_seq.tolist() == [
        5.0,
        0.0,
        0.0,
    ]
    lateral = ct.planner_solves.DOCKING_ROW_GROUPS.index("lateral")
    assert events.viol[2, lateral] == 1e-7
    # Columns this log does not have are NaN, not zero.
    assert np.isnan(events.slack_v).all() and np.isnan(events.elastic).all()
    assert np.isnan(np.delete(events.viol, lateral, axis=1)).all()
    # A log from before the segment columns has no events at all.
    frame[["wake_ns", "snapshot_sequence"]].to_csv(ctl / ct.PLANNER_EVENTS_CSV, index=False)
    assert ct._planner_segment_events(ctl) is None
    assert ct._planner_segment_events(tmp_path / "absent") is None


# ── The summary block ────────────────────────────────────────────────────────


def _summary_rows(n):
    return [
        {
            "cf_tot_s_mm": 10.0 + i,
            "cf_ref_s_mm": 8.0,
            "cf_tot_lateral_margin_mm": 1.0 if i < 2 else -4.0,
            "ent_cross_ms": math.nan if i == 0 else 5.0,
            "ent_lateral_margin_mm": math.nan if i == 0 else (2.0 if i < 4 else -1.0),
            "last_seg_wake_to_tc_ms": 300.0,
            "last_seg_pred_age_ms": 1.5,
            "last_seg_wake_join": "exact" if i % 2 else "ambiguous",
        }
        for i in range(n)
    ]


def test_the_summary_counts_the_set_and_holds_its_medians_below_ten():
    rows = [*_summary_rows(9), {"cf_tot_s_mm": math.nan}]  # the last never committed
    block = ct.catch_frame_summary(rows, DOCKING)
    assert block["n"] == 9
    assert (block["s_ent_mm"], block["rho_ref_mm"], block["lateral_faces"]) == (
        7.0,
        [-5.0, -5.0],
        4,
    )
    assert (block["in_lateral_at_tc"], block["crossed_entrance"]) == (2, 8)
    assert block["in_lateral_at_entrance"] == 3
    assert block["last_seg_wake_join"] == {"ambiguous": 5, "exact": 4}
    assert block["medians"] == "NOT_EVALUATED(n < 10)"
    assert "medians NOT_EVALUATED" in ct.catch_frame_line(block)
    full = ct.catch_frame_summary(_summary_rows(10), DOCKING)
    assert full["medians"]["cf_tot_s_mm"] == pytest.approx(14.5)
    assert full["medians"]["ent_cross_ms"] == 5.0
    assert "300 ms before t_c" in ct.catch_frame_line(full)
    # A hand without a set still has the frame's components; no trial, no block.
    bare = ct.catch_frame_summary(_summary_rows(10), None)
    assert "s_ent_mm" not in bare and bare["n"] == 10
    assert ct.catch_frame_summary([{"cf_tot_s_mm": math.nan}], DOCKING) is None


# ── End to end on the pilot session ──────────────────────────────────────────

FIXTURE = Path(__file__).parent / "data" / "catching_pilot_260924_1218"


def test_the_pilot_session_carries_the_columns_through_the_real_frame():
    """The golden soft-catch session (25 committed throws, no segment, a profile
    from before the hand's capture set): the components come from the real FK's
    rotation, so their lengths must be the norms the tool already reports."""
    pytest.importorskip("pinocchio")
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    result = ct.analyse_session(
        FIXTURE / "session",
        FIXTURE / "trials",
        profile,
        (FIXTURE / "robot.urdf").read_text(),
        ct.Settings(n_boot=20),
        FIXTURE / "session" / "sim" / "clock_lane.csv.gz",
        FIXTURE / "session" / "sim" / "ball_contact_lane.csv.gz",
    )
    committed = [r for r in result.rows if math.isfinite(r.get("t_c", math.nan))]
    assert len(committed) == 25
    text = ("last_seg_kind", "last_seg_elastic_group", "last_seg_viol_group", "last_seg_wake_join")
    for row in committed:
        for term, norm in (("tot", "total_mm"), ("ref", "ref_vs_true_mm"), ("servo", "servo_mm")):
            assert np.linalg.norm(_terms(row, term)) == pytest.approx(row[norm], abs=1e-6)
        assert _terms(row, "tot") == pytest.approx(
            _terms(row, "ref") + _terms(row, "clik") + _terms(row, "servo"), abs=1e-6
        )
        assert all(row[k] == "" for k in text)
        assert all(math.isnan(row[k]) for k in ct.LAST_SEGMENT_KEYS if k not in text)
    assert ct.hand_docking(profile) is None
    assert all(math.isnan(r["ent_cross_ms"]) for r in committed)
    block = result.summary["catch_frame"]
    assert block["n"] == 25 and "s_ent_mm" not in block
    assert math.isfinite(block["medians"]["cf_tot_s_mm"])
