"""catching_trials (dynamic_catching S8-A) — golden pilot + statistics with positive controls.

The golden test replays the S8 pilot session ``260924_1218`` (25 throws at the
sim, lead off; fixture cut by ``data/catching_pilot_260924_1218/make_fixture.py``)
and pins the numbers the S8 plan was built on (sub-plan §4, #537): τ̂ 200 ms on
every arm joint, the t_c servo gap 122 mm, CLIK 2 mm, 25/25 supervisor Missed,
D-3 ε 12 mm valid 7/25. Every statistic also gets a POSITIVE control — an
injected defect it must catch — because a test that only checks "passes on
clean data" passes an implementation that always says pass.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import numpy as np
import pytest
import yaml

from rtc_tools.analysis import catching_trials as ct

FIXTURE = Path(__file__).parent / "data" / "catching_pilot_260924_1218"
MODULE = Path(ct.__file__)
# plan §4.4 S3b: the fastest catch speed the S3.5b gate map opened for this
# robot. The pilot's D-3 table (#537 5806998220) used it with clock_phase's
# default a_bound.
PILOT_V_MAX_M_S = 3.85


# ── Golden: the pilot session ────────────────────────────────────────────────


@pytest.fixture(scope="module")
def pilot():
    pytest.importorskip("pinocchio")
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    settings = ct.Settings(v_max=PILOT_V_MAX_M_S, eps_mm=(12.0, 22.8, 41.8, 49.8), n_boot=200)
    return ct.analyse_session(
        FIXTURE / "session",
        FIXTURE / "trials",
        profile,
        (FIXTURE / "robot.urdf").read_text(),
        settings,
        FIXTURE / "session" / "sim" / "clock_lane.csv.gz",
        FIXTURE / "session" / "sim" / "ball_contact_lane.csv.gz",
    )


def test_golden_servo_lag_is_200ms_on_every_joint(pilot):
    assert len(pilot.lag) == 6
    for lag in pilot.lag:
        assert lag.tau_s == pytest.approx(0.200, abs=0.005), lag
        assert lag.ci_low_s <= lag.tau_s <= lag.ci_high_s
        assert lag.r2 > 0.99, "a first-order plant should explain the tracking error"


def test_golden_tc_decomposition(pilot):
    med = pilot.summary["medians"]
    assert med["servo_mm"] == pytest.approx(122.0, abs=5.0)
    assert med["clik_mm"] == pytest.approx(2.0, abs=1.0)
    # The ball meets the hand about 40 ms before t_c (the pilot's finding).
    assert med["arrival_ms"] == pytest.approx(-40.0, abs=5.0)
    # Planned γ_f, not ref_gamma (which reads 1.0 on the DECEL entry tick).
    lo, hi = pilot.summary["gamma_f_planned_range"]
    assert lo > 0.45 and hi < 0.75


def test_golden_is_lead_off_so_the_lead_aware_servo_is_the_same_tick_gap(pilot):
    """The pilot ran without the lead and predates the diag ``t_arm_s`` column
    and the runner mirror, so T_lead falls back to 0 and the lead-aware servo
    residual must equal the recorded same-tick gap on every trial (S8-B)."""
    assert pilot.summary["t_lead_source"] == "0 (no diag column, no runner mirror)"
    assert pilot.summary["t_lead_s_range"] == [0.0, 0.0]
    rows = [r for r in pilot.rows if r.get("accepted")]
    assert rows and all(r["servo_mm"] == r["cmd_meas_gap_mm"] for r in rows)
    assert pilot.summary["medians"]["cmd_meas_gap_mm"] == pytest.approx(122.0, abs=5.0)


def test_golden_supervisor_verdicts_and_truth(pilot):
    assert pilot.summary["supervisor_verdicts"] == {"MISSED": 25}
    truth = pilot.summary["truth"]
    assert truth["successes"] == 0 and truth["n"] == 25
    assert truth["confusion_supervisor_vs_truth"] == {
        "MISSED": {"truth_success": 0, "truth_fail": 25}
    }


def test_golden_d3_validity(pilot):
    valid = pilot.summary["d3"]["valid"]
    assert valid["12mm"] == 7
    # The rest of the pilot's table (24 % / 8 % / 4 % invalid).
    assert (valid["22.8mm"], valid["41.8mm"], valid["49.8mm"]) == (19, 23, 24)
    assert pilot.summary["d3"]["dropped_total"] == 0
    # The lane and the runner share one monotonic clock; the pairing is checked.
    assert pilot.summary["d3"]["clock_steady_offset_spread_ms"] < 5.0


def test_golden_contacts_and_streaks(pilot):
    rows = [r for r in pilot.rows if r["accepted"]]
    assert pilot.summary["ref_saturated_max_streak"] == 49
    # 23 of 25 balls touch the hand, always before t_c.
    hand = [r for r in rows if r["contact_body"]]
    assert len(hand) == 23
    assert all(r["contact_t_minus_tc_ms"] < 0 for r in hand)
    assert all(r["contact_impulse_ns"] > 0 for r in hand)


def test_frames_compose_to_the_measured_half_turn():
    """Sim world → model world is Rz(π) on this robot; without it gaps are metres."""
    pytest.importorskip("pinocchio")
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    urdf = (FIXTURE / "robot.urdf").read_text()
    joints = ct.arm_joints_from_diag(ct._csv_header(_diag_path()))
    fk = ct.CatchFrameFk(urdf, joints, profile)
    rz_pi = np.diag([-1.0, -1.0, 1.0])
    assert np.allclose(fk.model_t_world[:3, :3], rz_pi, atol=1e-9)


def _diag_path() -> Path:
    return FIXTURE / "session" / "controllers" / "demo_catching_controller" / "catching_diag.csv"


LANE = FIXTURE / "session" / "sim" / "clock_lane.csv.gz"


def test_clock_lane_pairs_a_subset_of_its_launches_with_the_same_launches():
    # A session can hold launches from another run (a second runner call, a GUI
    # throw). Drop trials from the front and the middle: the rest must pair
    # with exactly the launches they paired with when all trials were given —
    # not shift by one, which order-pairing would do.
    trials, _ = ct.load_trials(FIXTURE / "trials")
    full = ct.load_clock_lane(LANE, trials).trial_to_seq
    assert len(full) == len([t for t in trials if t.accepted])
    subset = [t for k, t in enumerate(trials) if k not in (0, 12)]
    part = ct.load_clock_lane(LANE, subset)
    assert part.trial_to_seq == {t.idx: full[t.idx] for t in subset}
    assert part.offset_spread_s < 5e-3


def test_clock_lane_refuses_a_run_it_does_not_contain():
    # The same trials with their launch instants scrambled (another run's
    # timing): no single clock offset pairs them, so the lane is refused.
    import dataclasses

    trials, _ = ct.load_trials(FIXTURE / "trials")
    rng = np.random.default_rng(0)
    shuffled = [
        dataclasses.replace(t, t_launch=t.t_launch + float(rng.uniform(-30.0, 30.0)))
        for t in trials
    ]
    with pytest.raises(SystemExit, match="not this run"):
        ct.load_clock_lane(LANE, shuffled)


def test_an_unpaired_trial_gets_no_clock_covariate_instead_of_a_borrowed_one():
    import dataclasses

    trials, _ = ct.load_trials(FIXTURE / "trials")
    moved = trials[:]
    moved[3] = dataclasses.replace(trials[3], t_launch=trials[3].t_launch + 0.5)
    lane = ct.load_clock_lane(LANE, moved)
    assert trials[3].idx not in lane.trial_to_seq
    assert len(lane.trial_to_seq) == len(trials) - 1


def test_an_unpaired_trial_has_no_contact_episode_instead_of_a_borrowed_one(monkeypatch):
    """End to end: a trial the clock lane has no launch for (seen in S8-B,
    tuning t3) used to reach the contact-episode analysis with no lane segment
    of its own — a KeyError, and without it the previous trial's segment."""
    pytest.importorskip("pinocchio")
    real = ct.load_clock_lane
    dropped = {}

    def drop_one(path, trials, *args):
        # A .gz lane re-enters load_clock_lane once decompressed; drop only once.
        lane = real(path, trials, *args)
        if "idx" not in dropped:
            dropped["idx"] = sorted(lane.trial_to_seq)[3]
            del lane.trial_to_seq[dropped["idx"]]
        return lane

    monkeypatch.setattr(ct, "load_clock_lane", drop_one)
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    result = ct.analyse_session(
        FIXTURE / "session",
        FIXTURE / "trials",
        profile,
        (FIXTURE / "robot.urdf").read_text(),
        ct.Settings(v_max=PILOT_V_MAX_M_S, n_boot=20),
        LANE,
        FIXTURE / "session" / "sim" / "ball_contact_lane.csv.gz",
    )
    rows = {r["idx"]: r for r in result.rows}
    assert "contact_impulse_ns" not in rows[dropped["idx"]]
    assert rows[dropped["idx"]].get("d3_unpaired")
    paired = [r for i, r in rows.items() if i != dropped["idx"] and r.get("accepted")]
    assert sum("contact_impulse_ns" in r for r in paired) == len(paired)


def test_profile_comes_from_config():
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    assert profile.devices[0] == profile.arm_device
    assert profile.diag_log and profile.catch_frame.parent
    assert profile.ball_diameter_m > 0 and profile.ball_mass_kg > 0


def test_arch1_module_has_no_robot_constants():
    """ARCH-1: every robot fact must come from config / URDF / the CSV header."""
    source = MODULE.read_text()
    for literal in ("ur5e", "p1b", "iiwa", "leap", "palm", "shoulder", "elbow", "wrist"):
        assert re.search(literal, source, re.IGNORECASE) is None, literal
    for literal in ('"base"', "'base'", "0.015", "0.145", "0.052", "0.002", "500.0"):
        assert literal not in source, literal


# ── Wilson / S0.9 power ──────────────────────────────────────────────────────


def test_wilson_interval_known_values():
    lo, hi = ct.wilson_interval(0, 25)
    assert lo == 0.0 and hi == pytest.approx(0.1332, abs=1e-4)
    lo, hi = ct.wilson_interval(20, 25)
    assert (lo, hi) == pytest.approx((0.6087, 0.9114), abs=1e-4)
    with pytest.raises(ValueError):
        ct.wilson_interval(26, 25)


@pytest.mark.parametrize(
    ("p", "floor", "n"), [(0.9, 0.7, 35), (0.8, 0.6, 44), (0.6, 0.5, 198), (0.95, 0.9, 254)]
)
def test_required_n_reproduces_the_s09_table(p, floor, n):
    """Four cells of IMPLEMENTATION_PLAN.md §1a S0.9 (z 1.96, power 0.8)."""
    assert ct.required_n(p, floor) == n


def test_required_n_is_impossible_at_or_below_the_floor():
    assert ct.required_n(0.7, 0.7) is None
    assert ct.wilson_power(35, 0.9, 0.7) >= 0.8
    assert ct.wilson_power(35, 0.7, 0.7) < 0.1  # at p = floor the test almost never passes


# ── Servo lag ────────────────────────────────────────────────────────────────


def _first_order_trials(tau, n_trials=6, dt=0.002, seconds=1.0, seed=3):
    rng = np.random.default_rng(seed)
    t_all, cmd_all, meas_all, cl = [], [], [], []
    for trial in range(n_trials):
        t = np.arange(0.0, seconds, dt) + trial * 10.0
        freq = rng.uniform(0.5, 2.0, 2)
        u = np.column_stack([np.sin(2 * np.pi * f * (t - t[0])) for f in freq])
        q = np.zeros_like(u)
        for k in range(1, len(t)):  # exact discretisation of q̇ = (u − q)/τ
            q[k] = u[k - 1] + (q[k - 1] - u[k - 1]) * math.exp(-dt / tau)
        t_all.append(t)
        cmd_all.append(u)
        meas_all.append(q)
        cl.append(np.full(len(t), trial))
    return map(np.concatenate, (t_all, cmd_all, meas_all, cl))


def test_servo_lag_recovers_an_injected_tau():
    t, cmd, meas, cl = _first_order_trials(tau=0.15)
    lag = ct.servo_lag_ls(t, cmd, meas, np.ones(len(t), bool), cl, ["a", "b"], n_boot=200)
    for x in lag:
        # The zero-order hold on u puts the error half a tick behind q̇.
        assert x.tau_s == pytest.approx(0.15, rel=0.02)
        assert x.ci_low_s <= x.tau_s <= x.ci_high_s
        assert x.r2 > 0.99


def test_servo_lag_distinguishes_tau():
    t, cmd, meas, cl = _first_order_trials(tau=0.05)
    (lag,) = ct.servo_lag_ls(t, cmd, meas, np.ones(len(t), bool), cl, ["a", "b"], n_boot=50)[:1]
    assert lag.tau_s == pytest.approx(0.05, rel=0.05)


# ── A⊥B ──────────────────────────────────────────────────────────────────────


def _clustered_gaussians(n_trials=40, per_trial=5, seed=0):
    rng = np.random.default_rng(seed)
    clusters = np.repeat(np.arange(n_trials), per_trial)
    # A trial-level offset plus per-sample noise: samples of a trial are correlated.
    a = rng.normal(size=(n_trials, 3))[clusters] + 0.5 * rng.normal(size=(len(clusters), 3))
    b = rng.normal(size=(n_trials, 3))[clusters] * [1.0, 2.0, 0.5] + 0.5 * rng.normal(
        size=(len(clusters), 3)
    )
    return a, b, clusters


def test_independence_passes_on_independent_gaussians():
    a, b, clusters = _clustered_gaussians()
    res = ct.independence_test(a, b, clusters, n_boot=500)
    assert res.passed, res.cross
    assert res.n_clusters == 40


def test_independence_fails_on_injected_correlation():
    a, b, clusters = _clustered_gaussians()
    b_corr = b + 0.8 * a @ np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    res = ct.independence_test(a, b_corr, clusters, n_boot=500)
    assert not res.passed


def test_whitening_makes_scale_irrelevant():
    a, b, _ = _clustered_gaussians()
    c1 = ct.whitened_cross_covariance(a, b)
    c2 = ct.whitened_cross_covariance(a * 1000.0, b * 1e-3)
    assert np.allclose(c1, c2)


# ── NEES ─────────────────────────────────────────────────────────────────────


def _nees_data(n=200, seed=1, bias=(0.0, 0.0, 0.0), cov_scale=1.0):
    rng = np.random.default_rng(seed)
    root = rng.normal(size=(n, 3, 3)) * 0.3 + np.eye(3)
    covs = root @ np.transpose(root, (0, 2, 1))
    errors = np.einsum("nij,nj->ni", np.linalg.cholesky(covs), rng.normal(size=(n, 3)))
    return errors + np.asarray(bias), covs * cov_scale


def test_nees_passes_when_consistent():
    e, p = _nees_data()
    res = ct.nees_summary(e, p)
    assert res.passed_raw and res.passed_centred
    assert res.mean_raw == pytest.approx(3.0, abs=0.5)


def test_nees_bias_fails_raw_but_not_centred():
    e, p = _nees_data(bias=(1.5, 0.0, -1.0))
    res = ct.nees_summary(e, p)
    assert not res.passed_raw
    assert res.passed_centred
    assert res.bias == pytest.approx([1.5, 0.0, -1.0], abs=0.3)


def test_nees_is_two_sided():
    e, p = _nees_data(cov_scale=0.25)  # overconfident: NEES ≈ 12
    assert not ct.nees_summary(e, p).passed_raw
    e, p = _nees_data(cov_scale=4.0)  # underconfident: NEES ≈ 0.75, one-sided would pass
    res = ct.nees_summary(e, p)
    assert not res.passed_raw and res.p_raw < 0.05


# ── Lead-aware t_c decomposition (S8-B) ──────────────────────────────────────


class _IdentityFk:
    """ "Joints" that ARE the catch-frame position, in a model world = sim world."""

    def __call__(self, q):
        return np.asarray(q, dtype=float)

    def to_model(self, p):
        return np.asarray(p, dtype=float)


_P0, _V0, _G = np.array([1.5, 0.0, 0.5]), np.array([-3.0, 0.4, 3.0]), np.array([0.0, 0.0, -9.81])
_REF_OFF, _CLIK_OFF, _SERVO_OFF = (
    np.array([0.0, 0.0, -0.010]),
    np.array([0.002, 0.0, 0.0]),
    np.array([0.0, 0.003, 0.0]),
)


def _ball(t):
    t = np.atleast_1d(t)[:, None]
    return _P0 + _V0 * t + 0.5 * _G * t**2


def _lead_trial(lead_s, t_lead_recorded):
    """A trial whose command side is aimed ``lead_s`` ahead, with KNOWN offsets:
    ref 10 mm off the ball, CLIK 2 mm, servo 3 mm (three orthogonal axes)."""
    t = np.arange(0.0, 1.2, 0.002)
    n = len(t)
    t_c = 0.8
    mode = np.full(n, ct.MODE_APPROACH)
    mode[100:] = ct.MODE_COMMITTED
    ref = _ball(t + lead_s) + _REF_OFF
    q_cmd = ref + _CLIK_OFF
    q_meas = _ball(t) + _REF_OFF + _CLIK_OFF + _SERVO_OFF  # = q_cmd(t − lead) + servo
    ctx = ct.TrialContext(
        t=t,
        mode=mode,
        hand_phase=np.zeros(n, int),
        plan_id=np.ones(n, int),
        plan_t_c=t_c - t,
        plan_p_c=np.tile(_ball(t_c)[0], (n, 1)),
        plan_gamma_f=np.full(n, 0.4),
        ref=ref,
        ref_valid=np.ones(n, bool),
        ref_saturated=np.zeros(n, bool),
        q_cmd=q_cmd,
        q_meas=q_meas,
        fk=_IdentityFk(),
        t_lead=np.full(n, t_lead_recorded),
    )
    tt = np.arange(0.0, 1.2, 0.01)
    truth = ct.Truth(tt, _ball(tt), _V0 + np.outer(tt, _G))
    return ct.decompose_at_tc(ctx, truth, window_s=0.12)


def test_lead_aware_decomposition_recovers_the_injected_parts():
    rec = _lead_trial(0.2, 0.2)
    assert rec["t_lead_s"] == 0.2
    assert rec["servo_mm"] == pytest.approx(3.0, abs=0.05)
    assert rec["clik_mm"] == pytest.approx(2.0, abs=0.05)
    assert rec["ref_vs_true_mm"] == pytest.approx(10.0, abs=0.2)
    assert rec["total_mm"] == pytest.approx(np.linalg.norm([2.0, 3.0, 10.0]), abs=0.2)
    # The same-tick gap carries the whole 0.2 s lead of a ~3 m/s path.
    assert rec["cmd_meas_gap_mm"] > 300.0


def test_reading_the_same_trial_without_its_lead_is_the_false_fail():
    """Positive control: ignore the recorded lead and the servo residual is
    the same-tick gap again — the S8-B pre-check's false G8-E FAIL."""
    rec = _lead_trial(0.2, 0.0)
    assert rec["servo_mm"] == rec["cmd_meas_gap_mm"] > 300.0
    assert rec["ref_vs_true_mm"] > 300.0


def test_lead_off_trial_is_unchanged():
    rec = _lead_trial(0.0, 0.0)
    assert rec["servo_mm"] == rec["cmd_meas_gap_mm"] == pytest.approx(3.0, abs=0.05)
    assert rec["ref_vs_true_mm"] == pytest.approx(10.0, abs=0.2)


def test_lead_per_tick_sources():
    pd = pytest.importorskip("pandas")
    on = {"controller_mirror": {"joint_cmd.lag.lead_enable": True, "joint_cmd.lag.T_arm": 0.2}}
    off = {"controller_mirror": {"joint_cmd.lag.lead_enable": False, "joint_cmd.lag.T_arm": 0.2}}
    with_col = pd.DataFrame({"t_arm_s": [0.2, 0.2]})
    lead, src = ct.lead_per_tick(with_col, on)
    assert list(lead) == [0.2, 0.2] and src == "diag t_arm_s"
    # The column is what the RT tick used; a mirror that disagrees means the
    # trials directory is another session's.
    with pytest.raises(SystemExit):
        ct.lead_per_tick(with_col, off)
    bare = pd.DataFrame({"mode": [0, 0]})
    assert list(ct.lead_per_tick(bare, on)[0]) == [0.2, 0.2]
    assert list(ct.lead_per_tick(bare, off)[0]) == [0.0, 0.0]
    lead, src = ct.lead_per_tick(bare, {})
    assert list(lead) == [0.0, 0.0] and src.startswith("0 ")
    # Lead on with T_arm TBD: the controller leads by 0 (lifecycle.cpp), so the
    # mirror must say 0 too — not NaN, and not a mismatch with a 0 column.
    for tbd in (math.nan, None):
        m = {"controller_mirror": {"joint_cmd.lag.lead_enable": True, "joint_cmd.lag.T_arm": tbd}}
        assert list(ct.lead_per_tick(bare, m)[0]) == [0.0, 0.0]
        assert list(ct.lead_per_tick(pd.DataFrame({"t_arm_s": [0.0, 0.0]}), m)[0]) == [0.0, 0.0]


# ── Truth success, free flight, streaks ──────────────────────────────────────


def _ballistic_truth(t_cut=None):
    t = np.arange(0.0, 1.0, 0.01)
    p0, v0, g = np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.5, 3.0]), np.array([0, 0, -9.81])
    p = p0 + np.outer(t, v0) + 0.5 * np.outer(t**2, g)
    v = v0 + np.outer(t, g)
    if t_cut is not None:  # a rebound: mirror the velocity after the cut
        after = t >= t_cut
        k = np.argmax(after)
        v[after] = -v[after]
        p[after] = p[k - 1] + np.outer(t[after] - t[k - 1], v[k])
    return ct.Truth(t, p, v)


def test_free_flight_ignores_the_rebound():
    clean = _ballistic_truth()
    hit = _ballistic_truth(t_cut=0.5)
    t_impact = hit.first_impact(0.0, accel_limit=2 * 11.43)
    assert t_impact == pytest.approx(0.5, abs=0.011)
    p, v, rms, t_last = hit.free_flight([0.58], t_impact)
    p_true, v_true = clean.at([0.58])
    assert np.allclose(p, p_true, atol=1e-9) and np.allclose(v, v_true, atol=1e-9)
    assert rms < 1e-9 and t_last < t_impact
    assert clean.first_impact(0.0, accel_limit=2 * 11.43) == math.inf


def _trial_ticks(n=200):
    t = np.arange(n) * 0.01
    mode = np.full(n, ct.MODE_HOLD)
    mode[100:] = ct.MODE_RETREAT
    phase = np.full(n, 3)
    phase[160:] = ct.HAND_PHASE_RELEASE
    return t, mode, phase


def test_truth_success_when_the_ball_rides_the_hand():
    t, mode, phase = _trial_ticks()
    hand = np.column_stack([t, np.zeros_like(t), np.ones_like(t)])
    truth = ct.Truth(t, hand + [0.0, 0.0, 0.01], np.zeros_like(hand))
    rec = ct.truth_success(t, mode, phase, truth, lambda k: hand[k], hold_radius_m=0.067)
    assert rec["truth_success"] and rec["d_max_mm"] == pytest.approx(10.0)
    assert rec["t_release"] == pytest.approx(1.6)


def test_truth_fails_when_the_ball_drops_during_retreat():
    t, mode, phase = _trial_ticks()
    hand = np.column_stack([t, np.zeros_like(t), np.ones_like(t)])
    ball = hand.copy()
    ball[130:, 2] = 0.03  # on the floor before the release
    truth = ct.Truth(t, ball, np.zeros_like(ball))
    rec = ct.truth_success(t, mode, phase, truth, lambda k: hand[k], hold_radius_m=0.067)
    assert not rec["truth_success"] and rec["truth_reason"] == "ball left the hand"


def test_truth_needs_a_release():
    t, mode, _ = _trial_ticks()
    rec = ct.truth_success(t, mode, np.full(len(t), 3), None, None, hold_radius_m=0.067)
    assert not rec["truth_success"] and rec["truth_reason"] == "no release in RETREAT"


def test_max_streak():
    assert ct.max_streak([0, 1, 1, 0, 1, 1, 1, 0]) == 3
    assert ct.max_streak([]) == 0


# ── Hand-joint capture witness (#537 S8-C, D-S8-8 (b)) ───────────────────────


def _hold_then_retreat_mode(n_hold=2, n_retreat=2):
    mode = np.concatenate([np.full(n_hold, ct.MODE_HOLD), np.full(n_retreat, ct.MODE_RETREAT)])
    return mode


def test_hand_witness_reads_the_last_hold_row_and_the_first_retreat_row():
    """The judge columns and ``outcome_source`` come from DIFFERENT rows: the
    hand-stage witness is the LAST HOLD tick's (index 1), ``outcome_source``
    is the FIRST RETREAT tick's (index 2) — the tick ``JudgeOutcome`` runs on,
    already showing mode RETREAT because ``AdvanceMode`` flips the mode before
    ``PublishTickRecord``. Swapping either row is a silent regression a
    mutation must catch (see the mutation check below)."""
    mode = _hold_then_retreat_mode()
    stalled = np.array([0.0, 1.0, 5.0, 5.0])  # last HOLD=1, first RETREAT=5
    effort = np.array([0.1, 0.6, 0.99, 0.99])  # last HOLD=0.6, first RETREAT=0.99
    blocked_s = np.array([0.0, 0.08, 0.10, 0.12])  # last HOLD=0.08, first RETREAT=0.10
    src = np.array([0.0, 0.0, 2.0, 2.0])  # only becomes meaningful at RETREAT
    out = ct.hand_witness_at_verdict(mode, stalled, effort, blocked_s, src, "CAPTURED")
    assert out["hand_stalled_n_judge"] == 1.0
    assert out["hand_effort_frac_judge"] == pytest.approx(0.6)
    assert out["hand_blocked_s_judge"] == pytest.approx(0.08)
    assert out["outcome_source"] == 2.0
    assert out["tips_only_verdict"] == "MISSED"


def test_hand_witness_is_nan_when_retreat_is_entered_from_abort_safe():
    """RETREAT reached from ABORT_SAFE (no HOLD) never ran ``JudgeOutcome`` or
    the hand-capture witness for an attempt — reading ``k_ret - 1`` blindly
    would silently pick up ABORT_SAFE's row instead. Every witness column must
    read as unavailable, exactly like a trial that never reaches RETREAT."""
    mode = np.array(
        [ct.MODE_DECEL, ct.MODE_ABORT_SAFE, ct.MODE_ABORT_SAFE, ct.MODE_RETREAT, ct.MODE_RETREAT]
    )
    stalled = np.array([0.0, 0.0, 0.0, 5.0, 5.0])
    effort = np.array([0.1, 0.1, 0.1, 0.99, 0.99])
    blocked_s = np.array([0.0, 0.0, 0.0, 0.10, 0.12])
    src = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # OnModeEntered sets kNone on a non-HOLD entry
    out = ct.hand_witness_at_verdict(mode, stalled, effort, blocked_s, src, "MISSED")
    assert math.isnan(out["hand_stalled_n_judge"])
    assert math.isnan(out["hand_effort_frac_judge"])
    assert math.isnan(out["hand_blocked_s_judge"])
    assert math.isnan(out["outcome_source"])
    assert out["tips_only_verdict"] == ""


def test_hand_persist_met_needs_the_judge_row_plus_one_tick():
    """``hand_blocked_s_judge`` is read one control period BEFORE the tick that
    actually compared against ``t_persist`` (the last HOLD row vs. the first
    RETREAT row that runs ``JudgeOutcome``) — so the offline re-check must add
    ``dt`` back on. A boundary case that only passes with the ``+ dt`` pins
    the off-by-one-tick note in the docstring."""
    # 0.08 + 0.02 == 0.10: meets t_persist only once dt is added back.
    assert ct.hand_persist_met(0.08, 0.02, 0.10) == 1.0
    # Without the +dt correction (i.e. comparing 0.08 >= 0.10 directly) this
    # would read False — the mistake this function exists to prevent.
    assert ct.hand_persist_met(0.08, 0.01, 0.10) == 0.0


@pytest.mark.parametrize(
    ("hand_blocked_s_judge", "dt", "t_persist_s"),
    [(math.nan, 0.002, 0.1), (0.08, math.nan, 0.1), (0.08, 0.002, math.nan)],
)
def test_hand_persist_met_is_nan_when_any_input_is_unavailable(
    hand_blocked_s_judge, dt, t_persist_s
):
    assert math.isnan(ct.hand_persist_met(hand_blocked_s_judge, dt, t_persist_s))


def test_hand_capture_t_persist_s_reads_the_yaml_or_nans():
    import dataclasses

    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    with_value = dataclasses.replace(profile, hand_yaml={"capture": {"t_persist": 0.05}})
    assert ct.hand_capture_t_persist_s(with_value) == pytest.approx(0.05)
    tbd = dataclasses.replace(profile, hand_yaml={"capture": {"t_persist": "TBD"}})
    assert math.isnan(ct.hand_capture_t_persist_s(tbd))
    absent = dataclasses.replace(profile, hand_yaml={})
    assert math.isnan(ct.hand_capture_t_persist_s(absent))


@pytest.mark.parametrize(
    ("supervisor", "source", "expected"),
    [
        ("CAPTURED", 1.0, "CAPTURED"),  # tips alone
        ("CAPTURED", 2.0, "MISSED"),  # only the hand witness promoted it
        ("CAPTURED", 3.0, "CAPTURED"),  # tips agreed too
        ("MISSED", 0.0, "MISSED"),  # no witness at all
    ],
)
def test_tips_only_verdict_reconstruction(supervisor, source, expected):
    mode = _hold_then_retreat_mode()
    src = np.array([0.0, 0.0, source, source])
    out = ct.hand_witness_at_verdict(mode, None, None, None, src, supervisor)
    assert out["tips_only_verdict"] == expected


def test_tips_only_verdict_is_empty_when_outcome_source_is_unavailable():
    mode = _hold_then_retreat_mode()
    out = ct.hand_witness_at_verdict(mode, None, None, None, None, "CAPTURED")
    assert out["tips_only_verdict"] == ""
    assert math.isnan(out["outcome_source"])
    out_no_retreat = ct.hand_witness_at_verdict(
        np.full(4, ct.MODE_HOLD), None, None, None, None, "CAPTURED"
    )
    assert out_no_retreat["tips_only_verdict"] == ""
    assert all(math.isnan(v) for v in out_no_retreat.values() if isinstance(v, float))


def test_t_release_to_pre_ms():
    t = np.arange(0.0, 1.0, 0.01)
    n = len(t)
    mode = np.full(n, ct.MODE_RETREAT)
    mode[:50] = ct.MODE_HOLD
    phase = np.full(n, 3)
    phase[60:] = ct.HAND_PHASE_RELEASE
    phase[80:] = ct.HAND_PHASE_PRESHAPE
    ms = ct.hand_release_to_preshape_ms(t, mode, phase)
    assert ms == pytest.approx((t[80] - t[60]) * 1e3)


def test_t_release_to_pre_ms_is_nan_when_the_hand_never_comes_back():
    t = np.arange(0.0, 1.0, 0.01)
    n = len(t)
    mode = np.full(n, ct.MODE_RETREAT)
    mode[:50] = ct.MODE_HOLD
    phase = np.full(n, 3)
    phase[60:] = ct.HAND_PHASE_RELEASE  # never reaches PRESHAPE
    assert math.isnan(ct.hand_release_to_preshape_ms(t, mode, phase))
    # No release at all either.
    assert math.isnan(ct.hand_release_to_preshape_ms(t, mode, np.full(n, 3)))


def _synthetic_hand(n=100, dt=0.01):
    t = np.arange(n) * dt
    q = np.zeros((n, 3))
    qd = np.zeros((n, 3))
    tau = np.zeros((n, 3))
    q_pre = np.array([0.0, 1.0, 0.0])
    q_close = np.array([1.0, 0.0, 1.0])  # joint 1 travels the NEGATIVE direction
    caging_mask = np.array([True, True, False])  # joint 2 is not in the caging set
    tau_max = np.array([1.0, 1.0, 1.0])
    hand = ct.HandCagingProfile(["j0", "j1", "j2"], q_pre, q_close, caging_mask, tau_max)
    return t, q, qd, tau, hand


def test_hand_hold_window_extremes_negative_direction_and_non_caging():
    t, q, qd, tau, hand = _synthetic_hand()
    sel = t >= 0.9
    q[sel, 0], qd[sel, 0], tau[sel, 0] = 0.8, 0.01, 0.9  # s=+1: frac = 0.9
    q[sel, 1], qd[sel, 1], tau[sel, 1] = 0.2, 0.02, -0.85  # s=-1: rho=0.8, frac=0.85
    q[sel, 2] = 5.0  # non-caging: must never show up in the output
    rows = ct.hand_hold_window_extremes(t, q, qd, tau, hand, t_hold_end=1.0, window_s=0.1)
    by_joint = {r["joint"]: r for r in rows}
    assert set(by_joint) == {"j0", "j1"}  # j2 (non-caging) is skipped
    assert by_joint["j0"]["rho_lo"] == pytest.approx(0.8)
    assert by_joint["j0"]["frac_lo"] == pytest.approx(0.9)
    assert by_joint["j1"]["rho_lo"] == pytest.approx(0.8)  # sign folds the negative travel back
    assert by_joint["j1"]["frac_lo"] == pytest.approx(0.85)
    assert by_joint["j0"]["n"] == by_joint["j1"]["n"] == 10


def test_capture_would_fire_boundaries_are_inclusive_and_respect_min_joints():
    rows = [
        {"rho_lo": 0.7, "rho_hi": 0.95, "qd_absmax": 0.05, "frac_lo": 0.8, "n": 10},
        {"rho_lo": 0.7, "rho_hi": 0.95, "qd_absmax": 0.05, "frac_lo": 0.8, "n": 10},
    ]
    assert ct.capture_would_fire(rows, 0.7, 0.95, 0.05, 0.8, min_joints=2)  # every bound exact
    assert not ct.capture_would_fire(rows, 0.7, 0.95, 0.05, 0.8, min_joints=3)
    # Nudge one row just outside each bound in turn: min_joints=2 must fail every time.
    for key, bad in (
        ("rho_lo", 0.699999),
        ("rho_hi", 0.950001),
        ("qd_absmax", 0.050001),
        ("frac_lo", 0.799999),
    ):
        bent = [dict(rows[0]), {**rows[1], key: bad}]
        assert not ct.capture_would_fire(bent, 0.7, 0.95, 0.05, 0.8, min_joints=2)
        assert ct.capture_would_fire(bent, 0.7, 0.95, 0.05, 0.8, min_joints=1)


def test_capture_would_fire_rejects_a_nan_extreme():
    rows = [{"rho_lo": math.nan, "rho_hi": 0.95, "qd_absmax": 0.05, "frac_lo": 0.8, "n": 0}]
    assert not ct.capture_would_fire(rows, 0.7, 0.95, 0.05, 0.8, min_joints=1)


def test_hand_caging_profile_is_none_without_q_pre_q_close():
    """The pilot fixture's controller YAML has a hand device (``p1b``) but no
    ``catching.robot.hand.q_pre``/``q_close`` — this feature predates it — so
    there is no caging profile to calibrate against, not an error."""
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    assert profile.hand_device == "p1b"
    assert profile.hand_yaml.get("q_pre") is None
    assert ct.hand_caging_profile(profile) is None


def test_hand_caging_profile_degrades_when_joint_limits_are_missing(capsys):
    """Regression (#537 S8-C code review): ``q_pre``/``q_close`` present but
    the hand device's ``joint_limits`` is entirely absent must NOT abort —
    this used to read the torque limit through
    ``derive_accel_limits.arm_spec_from_params``, which also requires
    ``max_velocity`` and raised ``SystemExit`` for a device that only needs
    to supply a torque limit for THIS calibration. The calibration is
    optional and degrades: ``None`` plus a one-line stderr note."""
    import dataclasses

    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    broken = dataclasses.replace(
        profile,
        hand_yaml={"q_pre": [0.0, 0.1, 0.2], "q_close": [1.0, 1.1, 1.2]},
        robot_params={
            **profile.robot_params,
            "devices": {"p1b": {"joint_state_names": ["j0", "j1", "j2"]}},  # no joint_limits
        },
    )
    assert ct.hand_caging_profile(broken) is None
    assert "joint_limits.max_torque" in capsys.readouterr().err


def test_hand_caging_profile_degrades_on_a_per_joint_tbd(capsys):
    """Regression (#537 S8-C code review round 2): a profile mid-search can
    have ``q_pre``/``q_close`` as LISTS whose elements are not all numbers yet
    (one joint still literally ``'TBD'``) — ``np.asarray(..., dtype=float)``
    would otherwise raise ``ValueError`` straight out of
    ``hand_caging_profile`` and abort the whole session."""
    import dataclasses

    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    broken = dataclasses.replace(
        profile,
        hand_yaml={"q_pre": [0.0, "TBD", 0.2], "q_close": [1.0, 1.1, 1.2]},
        robot_params={
            **profile.robot_params,
            "devices": {"p1b": {"joint_state_names": ["j0", "j1", "j2"]}},
        },
    )
    assert ct.hand_caging_profile(broken) is None
    assert "non-numeric" in capsys.readouterr().err


def test_analyse_session_survives_a_hand_device_without_joint_limits():
    """End to end: the same broken profile as above must still produce
    ``catching_trials`` rows — the hand-hold-window calibration is skipped,
    nothing else is."""
    pytest.importorskip("pinocchio")
    import dataclasses

    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    broken = dataclasses.replace(
        profile,
        hand_yaml={"q_pre": [0.0], "q_close": [1.0], "caging_mask": [True]},
        robot_params={**profile.robot_params, "devices": {"p1b": {"joint_state_names": ["j0"]}}},
    )
    result = ct.analyse_session(
        FIXTURE / "session",
        FIXTURE / "trials",
        broken,
        (FIXTURE / "robot.urdf").read_text(),
        ct.Settings(n_boot=20),
    )
    assert result.rows and any(r.get("accepted") for r in result.rows)
    assert result.hand_window_rows == []


def test_analyse_session_survives_a_hand_state_csv_without_velocity_or_effort(tmp_path, capsys):
    """End to end, the other half of the calibration pipeline:
    ``hand_caging_profile`` succeeds (q_pre/q_close/joint_limits all fine) but
    the hand device CSV predates ``actual_vel_*``/``effort_*`` — an older
    session. ``_hand_kinematics`` must degrade (``None`` + stderr note), not
    raise ``SystemExit``, and ``analyse_session`` must still produce rows."""
    pytest.importorskip("pinocchio")
    import dataclasses
    import shutil

    session = tmp_path / "session"
    shutil.copytree(FIXTURE / "session", session)
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    ctl = session / "controllers" / profile.controller
    hand_log = profile.device_logs[profile.hand_device]
    (ctl / f"{hand_log}.csv").write_text("t_relative_s,actual_pos_j0\n0.0,0.0\n")
    broken = dataclasses.replace(
        profile,
        hand_yaml={"q_pre": [0.0], "q_close": [1.0], "caging_mask": [True]},
        robot_params={
            **profile.robot_params,
            "devices": {
                profile.hand_device: {
                    "joint_state_names": ["j0"],
                    "joint_limits": {"max_torque": [1.0]},
                }
            },
        },
    )
    result = ct.analyse_session(
        session,
        FIXTURE / "trials",
        broken,
        (FIXTURE / "robot.urdf").read_text(),
        ct.Settings(n_boot=20),
    )
    assert result.rows and any(r.get("accepted") for r in result.rows)
    assert result.hand_window_rows == []
    assert "lacks hand state column" in capsys.readouterr().err


def test_cli_hold_window_s_default_matches_settings():
    """Item 2 regression: one source for the default, not two that can drift."""
    assert ct.Settings().hold_window_s == ct.DEFAULT_HOLD_WINDOW_S


def test_recorded_dt_prefers_the_runner_mirror():
    t = np.arange(10) * 0.004
    assert ct.recorded_dt({"meta": {"control.dt": 0.001}, "records": []}, t)[0] == 0.001
    doc = {"meta": {}, "records": [{"mirror": {"control": {"dt": 0.0025}}}]}
    assert ct.recorded_dt(doc, t)[0] == 0.0025
    value, source = ct.recorded_dt({"meta": {}, "records": [{}]}, t)
    assert value == pytest.approx(0.004) and "median" in source


def test_recorded_dt_reads_the_shape_catching_sim_trials_writes(tmp_path):
    # The runner records the controller's mirror per trial and in run_meta.json
    # as `controller_mirror: {"control.dt": ...}`. A dt that differs from the
    # CSV spacing proves the mirror, not the fallback, was read.
    t = np.arange(10) * 0.004
    doc = {"meta": {}, "records": [{"controller_mirror": {"control.dt": 0.001}}]}
    assert ct.recorded_dt(doc, t) == (0.001, "trial_results.json control.dt")
    (tmp_path / "trial_results.json").write_text("[]")
    (tmp_path / "run_meta.json").write_text('{"controller_mirror": {"control.dt": 0.0005}}')
    _, loaded = ct.load_trials(tmp_path)
    assert ct.recorded_dt(loaded, t)[0] == 0.0005


def test_cli_writes_csv_and_json(tmp_path, capsys):
    pytest.importorskip("pinocchio")
    rc = ct.main(
        [
            str(FIXTURE / "session"),
            str(FIXTURE / "trials"),
            "--config-dir",
            str(FIXTURE / "config"),
            "--urdf",
            str(FIXTURE / "robot.urdf"),
            "--out",
            str(tmp_path),
            "--n-boot",
            "50",
        ]
    )
    assert rc == 0
    assert (tmp_path / "catching_trials.csv").is_file()
    summary = (tmp_path / "catching_trials_summary.json").read_text()
    # Without --v-max the D-3 validity is not judged, and says so.
    assert "NOT_EVALUATED(v_max not given)" in summary
    assert "supervisor verdicts" in capsys.readouterr().out


# ── G3-D (i): per-trial plan validity ratio (#537 S8-D) ──────────────────────


def test_plan_validity_window_counts_cycles_between_launch_and_commit():
    wake = np.array([0.0, 1.0, 2.0, 3.0, 5.0])
    valid = np.array([0.0, 1.0, 1.0, 0.0, 1.0])
    out = ct.plan_validity_window(wake, valid, t_launch=0.5, t_commit=3.5, t_end=10.0)
    assert out["planner_cycles"] == 3  # wake 1, 2, 3 fall in [0.5, 3.5]
    assert out["plan_valid_cycles"] == 2
    assert out["plan_valid_ratio"] == pytest.approx(2 / 3)
    # The last cycle at or before the commit (wake=3) was invalid.
    assert out["plan_valid_at_commit"] == 0.0


def test_plan_validity_window_falls_back_to_trial_end_without_a_commit():
    wake = np.array([0.0, 1.0, 2.0, 3.0])
    valid = np.array([1.0, 1.0, 0.0, 1.0])
    out = ct.plan_validity_window(wake, valid, t_launch=0.0, t_commit=math.nan, t_end=2.5)
    assert out["planner_cycles"] == 3  # wake 0, 1, 2 <= 2.5; wake 3 is past the trial's end
    assert out["plan_valid_cycles"] == 2
    # Never committed: "at commit" has no meaning, regardless of cycle content.
    assert math.isnan(out["plan_valid_at_commit"])


def test_plan_validity_window_ratio_is_nan_with_zero_cycles():
    wake = np.array([10.0, 11.0])
    valid = np.array([1.0, 1.0])
    out = ct.plan_validity_window(wake, valid, t_launch=0.0, t_commit=1.0, t_end=1.0)
    assert out["planner_cycles"] == 0
    assert math.isnan(out["plan_valid_ratio"])
    # A commit happened, but no recorded cycle falls in the window.
    assert math.isnan(out["plan_valid_at_commit"])


def test_golden_g3d_plan_validity_ratio_in_range_with_positive_cycles(pilot):
    """The clock-lane mapping in :func:`ct._planner_cycle_times` must place
    every wake inside its own trial's window (not spill into a neighbour's) —
    a sanity range check, since there is no independent oracle for the exact
    count in the golden fixture."""
    g3d = pilot.summary["g3d"]
    assert g3d["n_trials_with_cycles"] == 25  # every pilot trial reached APPROACH
    median, p05, p95 = g3d["plan_valid_ratio_p50_p05_p95"]
    assert 0.0 <= p05 <= median <= p95 <= 1.0
    rows = [r for r in pilot.rows if r.get("accepted")]
    assert all(r["planner_cycles"] > 0 for r in rows)
    assert all(0.0 <= r["plan_valid_ratio"] <= 1.0 for r in rows)
    assert g3d["approach_plan_switches_distribution"] == {"0": 25}


def test_golden_g3d_pins_the_first_trials_cycle_count(pilot):
    """A concrete pin (not just a range) on one trial, cross-checked by hand
    against the fixture's raw ``planner_events.csv.gz``/clock lane."""
    rows = {r["idx"]: r for r in pilot.rows if r.get("accepted")}
    assert rows[0]["planner_cycles"] == 5
    assert rows[0]["plan_valid_cycles"] == 1
    assert rows[0]["plan_valid_ratio"] == pytest.approx(0.2)
    assert rows[0]["plan_valid_at_commit"] == 0.0


def test_g3d_windows_each_trial_with_its_own_clock_offset(monkeypatch):
    """Under sim-sync at RTF < 1 the steady − t_relative offset drifts across a
    session, so the session median is wrong for late trials. Push the median
    10 s off every trial's own offset: the per-trial windows must not move
    (trial 0 keeps its hand-checked pin). A trial the lane left without an
    offset gets no G3-D columns rather than the median's."""
    pytest.importorskip("pinocchio")
    real = ct.load_clock_lane
    state = {}

    def skew(path, trials, *args):
        # A .gz lane re-enters load_clock_lane once decompressed; skew only once.
        lane = real(path, trials, *args)
        if "idx" not in state:
            state["idx"] = sorted(lane.trial_offsets)[3]
            del lane.trial_offsets[state["idx"]]
            lane.steady_offset += 10.0
        return lane

    monkeypatch.setattr(ct, "load_clock_lane", skew)
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    result = ct.analyse_session(
        FIXTURE / "session",
        FIXTURE / "trials",
        profile,
        (FIXTURE / "robot.urdf").read_text(),
        ct.Settings(v_max=PILOT_V_MAX_M_S, n_boot=20),
        LANE,
    )
    rows = {r["idx"]: r for r in result.rows}
    assert rows[0]["planner_cycles"] == 5
    assert rows[0]["plan_valid_cycles"] == 1
    assert "planner_cycles" not in rows[state["idx"]]
    others = [r for i, r in rows.items() if i != state["idx"] and r.get("accepted")]
    assert all(r["planner_cycles"] > 0 for r in others)


def test_analyse_session_has_no_g3d_without_a_clock_lane():
    """Without ``--clock-lane`` there is no established wake_ns -> t_relative_s
    conversion (sim-sync ``t_relative_s`` is a tick count, not a clock reading
    — see :func:`ct._planner_cycle_times`), so G3-D must be entirely absent,
    not guessed at."""
    pytest.importorskip("pinocchio")
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    result = ct.analyse_session(
        FIXTURE / "session",
        FIXTURE / "trials",
        profile,
        (FIXTURE / "robot.urdf").read_text(),
        ct.Settings(n_boot=20),
    )
    assert "g3d" not in result.summary
    assert all("planner_cycles" not in r for r in result.rows if r.get("accepted"))


# ── Gate-map verdict (S8-D, #537, optional --gate-map) ───────────────────────


def _write_csv_rows(path: Path, rows: list[dict]) -> None:
    import csv

    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _tiny_gate_map_dirs(tmp_path: Path) -> tuple[Path, Path]:
    """A 2×2 grid (distance_m × speed_m_s; the other three axes constant, so
    they must not enter the normalised distance) with throws 0 and 3 open on
    the torque layer, throws 1 and 2 closed."""
    map_dir = tmp_path / "catchability_map"
    map_dir.mkdir()
    gate_dir = tmp_path / "gate_map"
    gate_dir.mkdir()
    grid = []
    idx = 0
    for d in (0.9, 1.0):
        for s in (4.6, 4.8):
            grid.append(
                {
                    "throw_index": idx,
                    "wait_pose_seed_id": 0,
                    "distance_m": d,
                    "azimuth_deg": 0.0,
                    "release_height_m": 0.2,
                    "aim_deviation_deg": 0.0,
                    "speed_m_s": s,
                    "elevation_deg": 63.0,
                }
            )
            idx += 1
    _write_csv_rows(map_dir / "throw_summary.csv", grid)
    _write_csv_rows(
        gate_dir / "gate_map.csv",
        [
            {"id": "c0", "seed_id": 0, "throw_index": 0, "reason_torque": "none"},
            {"id": "c1", "seed_id": 0, "throw_index": 1, "reason_torque": "reach_time_torque"},
            {"id": "c2", "seed_id": 0, "throw_index": 2, "reason_torque": "reach_time_torque"},
            {"id": "c3", "seed_id": 0, "throw_index": 3, "reason_torque": "none"},
            # A different seed's candidate must never leak into this seed's open set.
            {"id": "c4", "seed_id": 1, "throw_index": 1, "reason_torque": "none"},
        ],
    )
    import yaml

    (gate_dir / "gate_map_summary.yaml").write_text(
        yaml.safe_dump({"map_dir": str(map_dir), "seed_id": 0})
    )
    return gate_dir, map_dir


def test_load_gate_map_reads_the_grid_open_set_and_axis_steps(tmp_path):
    gate_dir, map_dir = _tiny_gate_map_dirs(tmp_path)
    gm = ct.load_gate_map(gate_dir)
    assert gm.seed_id == 0
    assert gm.map_dir == map_dir
    assert gm.open_throw_indices == frozenset({0, 3})
    assert gm.axis_steps == pytest.approx({"distance_m": 0.1, "speed_m_s": 0.2})
    # release_height_m/aim_deviation_deg/elevation_deg are constant on the grid.
    assert set(gm.axis_steps) == {"distance_m", "speed_m_s"}


def test_nearest_grid_throw_matches_on_azimuth_when_the_grid_varies_it(tmp_path):
    """A grid over azimuths (-30, 0, 30), as catchability_map builds by
    default. The throws differ ONLY in azimuth, and the -30 one has the
    lowest index and is closed. A trial thrown at 0 must match the open
    azimuth-0 throw, not tie across all three and fall to index 0."""
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    gate_dir = tmp_path / "gate_map"
    gate_dir.mkdir()
    base = {
        "wait_pose_seed_id": 0,
        "distance_m": 1.0,
        "release_height_m": 0.15,
        "aim_deviation_deg": 0.0,
        "speed_m_s": 2.95,
        "elevation_deg": 79.0,
    }
    _write_csv_rows(
        map_dir / "throw_summary.csv",
        [{"throw_index": i, **base, "azimuth_deg": az} for i, az in enumerate((-30.0, 0.0, 30.0))],
    )
    _write_csv_rows(
        gate_dir / "gate_map.csv",
        [
            {"id": "c0", "seed_id": 0, "throw_index": 0, "reason_torque": "reach_time_torque"},
            {"id": "c1", "seed_id": 0, "throw_index": 1, "reason_torque": "none"},
            {"id": "c2", "seed_id": 0, "throw_index": 2, "reason_torque": "reach_time_torque"},
        ],
    )
    import yaml

    (gate_dir / "gate_map_summary.yaml").write_text(
        yaml.safe_dump({"map_dir": str(map_dir), "seed_id": 0})
    )
    gm = ct.load_gate_map(gate_dir)
    assert set(gm.axis_steps) == {"azimuth_deg"}
    verdict = ct.gate_map_verdict({**base, "azimuth_deg": 0.0}, gm)
    assert verdict["map_throw_index"] == 1
    assert verdict["map_open"] is True
    assert verdict["map_distance"] == pytest.approx(0.0)


def test_grid_axis_steps_omits_a_single_valued_axis():
    throw_axes = {0: {"p": 0.0, "q": 5.0}, 1: {"p": 1.0, "q": 5.0}, 2: {"p": 2.0, "q": 5.0}}
    steps = ct.grid_axis_steps(throw_axes, axes=("p", "q"))
    assert steps == pytest.approx({"p": 1.0})
    assert "q" not in steps


def test_nearest_grid_throw_ties_go_to_the_lowest_throw_index():
    """Documents the tie rule: a trial exactly between two grid points ties on
    normalised distance, and the LOWER ``throw_index`` wins."""
    throw_axes = {0: {"p": 0.0}, 1: {"p": 1.0}}
    steps = {"p": 1.0}
    idx, dist = ct.nearest_grid_throw({"p": 0.5}, throw_axes, steps)
    assert idx == 0
    assert dist == pytest.approx(0.5)


def test_nearest_grid_throw_normalises_by_each_axis_own_step():
    """Positive control: axis ``p``'s grid step (1.0) is 10× axis ``q``'s
    (0.1). The SAME raw offset (0.4) picks throw 0 (the origin) when it sits
    on ``p`` — 0.4 is less than half a p-step — but picks throw 2 (a q-step
    away) when it sits on ``q`` instead, because 0.4 is 4 q-steps, well past
    it. Swapping which axis carries the offset therefore flips the verdict —
    proof the normalisation uses each axis's OWN step, not a shared one."""
    throw_axes = {0: {"p": 0.0, "q": 0.0}, 1: {"p": 1.0, "q": 0.0}, 2: {"p": 0.0, "q": 0.1}}
    steps = {"p": 1.0, "q": 0.1}
    on_p, _ = ct.nearest_grid_throw({"p": 0.4, "q": 0.0}, throw_axes, steps)
    on_q, _ = ct.nearest_grid_throw({"p": 0.0, "q": 0.4}, throw_axes, steps)
    assert on_p == 0
    assert on_q == 2


def test_trial_axis_values_is_none_for_a_reference_series_record():
    assert ct.trial_axis_values({"kind": "reference", "pos": (1.0, 0.0, 0.2)}) is None


def test_trial_axis_values_reads_a_dist_box_record():
    record = {axis: float(i) for i, axis in enumerate(ct.GATE_MAP_AXES)}
    record["kind"] = "s35b"
    assert ct.trial_axis_values(record) == {
        axis: float(i) for i, axis in enumerate(ct.GATE_MAP_AXES)
    }


def test_gate_map_verdict_open_closed_and_no_axes():
    gm = ct.GateMap(
        map_dir=Path("map"),
        seed_id=0,
        throw_axes={
            0: dict.fromkeys(ct.GATE_MAP_AXES, 0.0),
            1: {**dict.fromkeys(ct.GATE_MAP_AXES, 0.0), "distance_m": 1.0},
        },
        open_throw_indices=frozenset({0}),
        axis_steps={"distance_m": 1.0},
    )
    near0 = dict.fromkeys(ct.GATE_MAP_AXES, 0.0)
    out0 = ct.gate_map_verdict(near0, gm)
    assert out0 == {"map_open": True, "map_throw_index": 0, "map_distance": pytest.approx(0.0)}
    near1 = {**near0, "distance_m": 0.9}
    out1 = ct.gate_map_verdict(near1, gm)
    assert out1["map_open"] is False and out1["map_throw_index"] == 1
    assert ct.gate_map_verdict(None, gm) == {
        "map_open": None,
        "map_throw_index": None,
        "map_distance": None,
    }


def test_analyse_session_gate_map_verdict_is_none_for_the_pilots_reference_series():
    """End to end: the pilot's trials are all `reference` kind (no gate-map
    axes), so every row must get a `None` verdict, never a guess."""
    pytest.importorskip("pinocchio")
    gate_map = ct.GateMap(
        map_dir=Path("unused"),
        seed_id=0,
        throw_axes={0: dict.fromkeys(ct.GATE_MAP_AXES, 0.0)},
        open_throw_indices=frozenset({0}),
        axis_steps={"distance_m": 1.0},
    )
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    result = ct.analyse_session(
        FIXTURE / "session",
        FIXTURE / "trials",
        profile,
        (FIXTURE / "robot.urdf").read_text(),
        ct.Settings(n_boot=20),
        gate_map=gate_map,
    )
    rows = [r for r in result.rows if r.get("accepted")]
    assert rows and all(r["map_open"] is None for r in rows)
    gm_summary = result.summary["gate_map"]
    assert gm_summary["verdicted"] == 0 and gm_summary["open"] == 0
    assert math.isnan(gm_summary["open_fraction"])
    assert (
        gm_summary["truth_whole"] != "NOT_EVALUATED(no hold radius)"
    )  # profile has a ball diameter


def test_cli_gate_map_option_wires_through(tmp_path):
    pytest.importorskip("pinocchio")
    gate_dir, _map_dir = _tiny_gate_map_dirs(tmp_path)
    out_dir = tmp_path / "out"
    rc = ct.main(
        [
            str(FIXTURE / "session"),
            str(FIXTURE / "trials"),
            "--config-dir",
            str(FIXTURE / "config"),
            "--urdf",
            str(FIXTURE / "robot.urdf"),
            "--out",
            str(out_dir),
            "--n-boot",
            "20",
            "--gate-map",
            str(gate_dir),
        ]
    )
    assert rc == 0
    summary = (out_dir / "catching_trials_summary.json").read_text()
    assert '"gate_map"' in summary
    assert '"map_dir"' in summary


# ── robot config lookup (S8-D: a sim-only profile has no _base.yaml) ─────────


def _write_params(path, params):
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))


def test_robot_config_prefers_base_yaml_and_falls_back_to_sim_yaml(tmp_path):
    from rtc_tools.analysis.catching_trials import robot_config_params

    _write_params(tmp_path / "sim.yaml", {"urdf": {"from": "sim"}})
    assert robot_config_params(tmp_path)["urdf"] == {"from": "sim"}
    _write_params(tmp_path / "_base.yaml", {"urdf": {"from": "base"}})
    assert robot_config_params(tmp_path)["urdf"] == {"from": "base"}


def test_robot_config_skips_a_file_without_urdf_and_refuses_when_none_has_it(tmp_path):
    from rtc_tools.analysis.catching_trials import robot_config_params

    _write_params(tmp_path / "_base.yaml", {"control_rate": 500.0})
    with pytest.raises(SystemExit, match="carries a `urdf` block"):
        robot_config_params(tmp_path)
    _write_params(tmp_path / "sim.yaml", {"urdf": {"from": "sim"}})
    assert robot_config_params(tmp_path)["urdf"] == {"from": "sim"}


# ── Validity (D-S8-16 ①): rig failures are invalid, not failures ─────────────


def test_golden_every_trial_is_valid_and_truth_is_unchanged(pilot):
    v = pilot.summary["validity"]
    assert v["n_total"] == 25 and v["n_valid"] == 25
    assert v["n_invalid"] == dict.fromkeys(ct.INVALID_REASONS, 0)
    assert v["lane_rules_evaluated"] is True
    assert all(r["invalid_reason"] == "" for r in pilot.rows)
    truth = pilot.summary["truth"]
    assert (truth["successes"], truth["n"]) == (0, 25)
    assert truth["itt"]["successes"] == 0 and truth["itt"]["n"] == 25
    assert truth["lower_975"] == truth["wilson95"][0] == ct.wilson_interval(0, 25)[0]
    assert truth["confusion_supervisor_vs_truth"] == {
        "MISSED": {"truth_success": 0, "truth_fail": 25}
    }
    # The pilot session has no timing/ dir — tolerated, and said so.
    assert pilot.summary["tick_overrun"] == "NOT_EVALUATED(no timing log)"


# Trial idx → the rig failure injected into a copy of the pilot fixture. Two
# trials carry two defects each (precedence); the rest must stay valid.
INJECTED = {
    2: "srv_refused",  # + n_truth_rows 0 (rule 1 outranks rule 2)
    3: "not_launched",  # + no offset (rule 2 outranks rule 3)
    5: "controller_silent",  # no wall_t_relative_offset
    6: "controller_silent",  # window moved off the diag (rule 3 outranks the lane's 4)
    8: "lane_drop",  # its launch removed from the lane
    10: "lane_drop",  # dropped_total rises inside its rows
    12: "sim_stall",  # 10 consecutive rows cut out of its rows
    14: "lane_drop",  # dropped rise AND a gap (rule 4 outranks rule 5)
    18: "not_launched",  # no truth rows AND no lane launch (rule 2 outranks rule 4)
}
TIMING_TRIAL = 16


def _mutated_fixture(tmp: Path) -> tuple[Path, Path, Path]:
    import gzip
    import json
    import shutil

    import pandas as pd

    session, trials_dir = tmp / "session", tmp / "trials"
    shutil.copytree(FIXTURE / "session", session)
    shutil.copytree(FIXTURE / "trials", trials_dir)
    trials, _ = ct.load_trials(FIXTURE / "trials")
    lane0 = ct.load_clock_lane(LANE, trials)
    seq = lane0.trial_to_seq

    records = json.loads((trials_dir / "trial_results.json").read_text())
    by_idx = {r["idx"]: r for r in records}
    for r in records:
        r["seed"] = 601
    by_idx[2].update(accepted=False, n_truth_rows=0)
    by_idx[3].update(n_truth_rows=0, wall_t_relative_offset=None)
    by_idx[5]["wall_t_relative_offset"] = None
    by_idx[18]["n_truth_rows"] = 0
    by_idx[6]["launch_wall_time"] += 1000.0
    by_idx[6]["mode_log"] = [[m[0] + 1000.0, *m[1:]] for m in by_idx[6]["mode_log"]]
    (trials_dir / "trial_results.json").write_text(json.dumps(records))

    with gzip.open(LANE, "rt") as handle:
        df = pd.read_csv(handle)
    df = df[~df["launch_seq"].isin((seq[8], seq[18]))]
    for idx in (10, 14):
        rows = df.index[df["launch_seq"] == seq[idx]]
        # Cumulative counter: every row from the middle of this flight on.
        df.loc[df.index >= rows[len(rows) // 2], "dropped_total"] += 1
    for idx in (12, 14):
        rows = df.index[df["launch_seq"] == seq[idx]]
        mid = len(rows) // 3
        df = df.drop(rows[mid : mid + 10])
    lane_path = session / "sim" / "clock_lane.csv"
    df.to_csv(lane_path, index=False)

    # A tick timing lane (steady clock, like the lane) with three overruns and
    # one jitter spike inside trial 16's window, and one of each just outside.
    t16 = next(t for t in trials if t.idx == TIMING_TRIAL)
    off = lane0.trial_offsets[TIMING_TRIAL]
    lo, hi = t16.t_launch + off, t16.t_end + off
    t = np.arange(lo - 0.1, hi + 0.1, 0.002)
    total = np.full(t.size, 100.0)
    jitter = np.zeros(t.size)
    inside = np.nonzero((t >= lo) & (t <= hi))[0]
    total[inside[[5, 50, 100]]] = 5000.0
    jitter[inside[70]] = 123.0
    total[0], jitter[0] = 9000.0, 999.0  # before the launch — must not count
    (session / "timing").mkdir()
    pd.DataFrame(
        {
            "t_wall_ns": (t * 1e9).astype(np.int64),
            "tick_count": np.arange(t.size),
            "run_id": 0,
            "t_state_us": 0.0,
            "t_compute_us": 0.0,
            "t_publish_us": 0.0,
            "t_total_us": total,
            "jitter_us": jitter,
        }
    ).to_csv(session / "timing" / "cm_timing_log.csv", index=False)
    return session, trials_dir, lane_path


@pytest.fixture(scope="module")
def mutated(tmp_path_factory):
    pytest.importorskip("pinocchio")
    session, trials_dir, lane_path = _mutated_fixture(tmp_path_factory.mktemp("rig"))
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    urdf = (FIXTURE / "robot.urdf").read_text()
    settings = ct.Settings(n_boot=20, floor=0.35, n_valid_target=200)
    with_lane = ct.analyse_session(session, trials_dir, profile, urdf, settings, lane_path)
    without = ct.analyse_session(session, trials_dir, profile, urdf, settings)
    return with_lane, without


def test_each_injected_rig_failure_gets_exactly_its_reason(mutated):
    result, _ = mutated
    reasons = {r["idx"]: r["invalid_reason"] for r in result.rows}
    for idx, reason in INJECTED.items():
        assert reasons[idx] == reason, idx
    assert all(reasons[i] == "" for i in reasons if i not in INJECTED)
    v = result.summary["validity"]
    assert v["n_invalid"] == {
        "srv_refused": 1,
        "not_launched": 2,
        "controller_silent": 2,
        "lane_drop": 3,
        "sim_stall": 1,
    }
    assert (v["n_total"], v["n_valid"]) == (25, 25 - len(INJECTED))


def test_invalid_trials_do_not_leak_into_the_failure_count(mutated):
    result, _ = mutated
    truth = result.summary["truth"]
    n_valid = 25 - len(INJECTED)
    assert truth["n"] == n_valid
    cells = truth["confusion_supervisor_vs_truth"].values()
    assert sum(c["truth_success"] + c["truth_fail"] for c in cells) == n_valid
    # ITT keeps them — as failures, over every trial.
    assert truth["itt"]["n"] == 25
    assert truth["itt"]["lower_975"] == ct.wilson_interval(truth["successes"], 25)[0]
    assert truth["verdict"] == f"INSUFFICIENT_N({n_valid} < 200)"


def test_lane_rule_diagnostics_show_the_injected_defect(mutated):
    result, _ = mutated
    rows = {r["idx"]: r for r in result.rows}
    nominal = result.summary["validity"]["nominal_step_s"]
    assert nominal == pytest.approx(0.002, rel=1e-3)
    assert rows[10]["lane_dropped_delta"] == 1 and rows[14]["lane_dropped_delta"] == 1
    assert rows[12]["lane_dropped_delta"] == 0
    assert rows[12]["lane_max_sim_gap_s"] == pytest.approx(11 * nominal, rel=1e-3)
    assert rows[0]["lane_max_sim_gap_s"] == pytest.approx(nominal, rel=1e-3)
    # The trial AFTER the drop starts from the raised baseline: not a drop.
    assert rows[11]["invalid_reason"] == "" and rows[11]["lane_dropped_delta"] == 0


def test_without_a_lane_only_the_record_rules_apply_and_the_summary_says_so(mutated):
    _, result = mutated
    reasons = {r["idx"]: r["invalid_reason"] for r in result.rows}
    for idx, reason in INJECTED.items():
        expected = "" if reason in ("lane_drop", "sim_stall") else reason
        assert reasons[idx] == expected, idx
    v = result.summary["validity"]
    assert v["lane_rules_evaluated"] is False
    assert "NOT_EVALUATED" in v["lane_rules"]
    assert v["n_invalid"]["lane_drop"] == v["n_invalid"]["sim_stall"] == 0
    # No lane → no steady-clock join for the timing log either.
    assert result.summary["tick_overrun"].startswith("NOT_EVALUATED(no clock lane")


def test_tick_overrun_is_joined_on_the_steady_clock_through_the_lane(mutated):
    result, _ = mutated
    rows = {r["idx"]: r for r in result.rows}
    assert rows[TIMING_TRIAL]["tick_overrun_n"] == 3
    assert rows[TIMING_TRIAL]["tick_jitter_max_us"] == 123.0
    tick = result.summary["tick_overrun"]
    assert tick["n_trials_with_overrun"] == 1 and tick["overrun_ticks_total"] == 3
    assert tick["period_us"] == pytest.approx(2000.0)


def test_seed_column_is_the_records_seed(mutated):
    result, _ = mutated
    assert all(r["seed"] == 601 for r in result.rows)
    assert result.summary["seeds"] == [601]


def test_record_invalid_reason_rules():
    assert ct.record_invalid_reason({"accepted": False, "n_truth_rows": 0}) == "srv_refused"
    ok = {"accepted": True, "n_truth_rows": 5, "wall_t_relative_offset": 1.0}
    assert ct.record_invalid_reason(ok) == ""
    # A record older than n_truth_rows is not judged on it.
    assert ct.record_invalid_reason({"accepted": True, "wall_t_relative_offset": 1.0}) == ""


# ── Verdict statistics ───────────────────────────────────────────────────────


def test_wilson_lower_bound_hand_value_and_the_floor_line():
    # k=5, n=10, z=1.96: centre 0.5, half 1.96·√(0.025 + 3.8416/400)/1.38416.
    lo, hi = ct.wilson_interval(5, 10)
    assert lo == pytest.approx(0.2366, abs=1e-4) and hi == pytest.approx(0.7634, abs=1e-4)
    # The S8-E pass line at floor 0.35 (plan §1a: pass = ≥ 84/200).
    assert ct.wilson_interval(84, 200)[0] == pytest.approx(0.3537, abs=1e-4)
    assert ct.wilson_interval(83, 200)[0] == pytest.approx(0.3489, abs=1e-4)
    assert ct.floor_verdict(84, 200, 0.35, 200) == "PASS"
    assert ct.floor_verdict(83, 200, 0.35, 200) == "FAIL"
    assert ct.floor_verdict(84, 199, 0.35, 200) == "INSUFFICIENT_N(199 < 200)"


def test_truth_block_valid_and_itt():
    valid = [{"truth_success": True, "supervisor": "CAUGHT"}] * 3 + [
        {"truth_success": False, "supervisor": "MISSED"}
    ] * 3
    tb = ct.truth_block(valid, 8, floor=0.2)
    assert (tb["successes"], tb["n"]) == (3, 6)
    assert tb["lower_975"] == ct.wilson_interval(3, 6)[0]
    assert (tb["itt"]["successes"], tb["itt"]["n"]) == (3, 8)
    assert tb["itt"]["lower_975"] == ct.wilson_interval(3, 8)[0]
    assert tb["confusion_supervisor_vs_truth"] == {
        "CAUGHT": {"truth_success": 3, "truth_fail": 0},
        "MISSED": {"truth_success": 0, "truth_fail": 3},
    }
    assert tb["verdict"] == ("PASS" if ct.wilson_interval(3, 6)[0] >= 0.2 else "FAIL")
    # A NaN cell (not analysed) is not a success.
    assert ct.truth_block([{"truth_success": math.nan}], 1)["successes"] == 0


def test_mcnemar_exact_hand_value():
    # b=7, c=1: 2·P(X ≤ 1 | Bin(8, ½)) = 2·9/256.
    assert ct.mcnemar_exact(7, 1) == pytest.approx(18 / 256)
    assert ct.mcnemar_exact(1, 7) == pytest.approx(18 / 256)
    assert ct.mcnemar_exact(0, 0) == 1.0


def test_streak_distribution_bins():
    rows = [{"ref_saturated_max_streak": v} for v in (0, 0, 1, 10, 11, 25)] + [{}]
    d = ct.streak_distribution(rows)
    assert (d["n_trials"], d["n_streak_gt0"], d["max"]) == (6, 4, 25)
    assert d["histogram"] == {"0": 2, "1-10": 2, "11-20": 1, "21-30": 1}
    assert "no verdict" in d["note"]


def _linear_contacts(n, seed=0):
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.05, 0.4, n)
    y = 0.8 * x + 0.01 + rng.normal(0.0, 0.002, n)
    f = 200.0 * x + rng.normal(0.0, 0.5, n)
    return [
        {"contact_mv_rel_ns": a, "contact_impulse_ns": b, "contact_peak_force_n": c}
        for a, b, c in zip(x, y, f, strict=True)
    ]


def test_impulse_correlation_recovers_a_linear_law():
    rows = _linear_contacts(60) + [{"contact_mv_rel_ns": math.nan, "contact_impulse_ns": 0.1}]
    b3 = ct.impulse_correlation(rows, n_boot=300, seed=1)
    assert b3["n"] == 60
    assert b3["ols_slope"] == pytest.approx(0.8, abs=0.02)
    assert b3["ols_intercept_ns"] == pytest.approx(0.01, abs=0.005)
    lo, hi = b3["ols_slope_ci95"]
    assert lo < 0.8 < hi and hi - lo < 0.05
    assert b3["spearman_rho_impulse"] > 0.95 and b3["spearman_p_impulse"] < 1e-6
    assert b3["spearman_rho_peak_force"] > 0.95
    assert b3["spearman_verdict"] == "REPORTED(no pass threshold defined)"
    assert b3["torque"] == "NOT_EVALUATED(sim clamp)"


def test_impulse_correlation_below_50_is_not_evaluated():
    b3 = ct.impulse_correlation(_linear_contacts(20), n_boot=50, seed=1)
    assert b3["spearman_verdict"] == "NOT_EVALUATED(n < 50)"
    assert b3["ols_slope"] == pytest.approx(0.8, abs=0.05)
    empty = ct.impulse_correlation([], n_boot=50, seed=1)
    assert empty["n"] == 0 and math.isnan(empty["ols_slope"])


# ── Lane time alignment (S8-E C3): robust to a sim slower than the wall ──────

DT = 0.002
C_TRUE = 7 * DT  # sim − t_relative_s: the steps the sim took before the CM's first tick
K_WALL = 1.7e9  # CLOCK_REALTIME − CLOCK_MONOTONIC
LAUNCH_SIMS = (2.0, 6.0, 10.0, 14.0, 18.0)
TRIAL_RTF = (1.0, 0.6, 0.8, 0.5, 1.0)  # wall RTF from launch to the trial's end
EVENT_SIMS = (0.1, 0.4, 0.5, 2.5)  # mode changes after the launch; the last closes the cycle
RECV_LATENCY_S = 0.001


def _slow_sim_session(tmp: Path):
    """A lane, trials dir and truth CSVs of a sim that runs at TRIAL_RTF during each
    trial — what the S8-E loaded unit did (RTF 0.62). Returns (lane path, trials dir,
    steady(sim) function)."""
    import json

    import pandas as pd

    n = int(22.0 / DT)
    sim = np.arange(1, n + 1) * DT
    rtf = np.ones(n)
    for launch, r in zip(LAUNCH_SIMS, TRIAL_RTF, strict=True):
        rtf[(sim > launch) & (sim <= launch + 3.0)] = r
    steady = 100.0 + np.cumsum(DT / rtf)
    seq = np.zeros(n, dtype=int)
    active = np.zeros(n, dtype=int)
    for k, launch in enumerate(LAUNCH_SIMS):
        seq[sim > launch + DT / 2] = k + 1
        active[(sim > launch + DT / 2) & (sim <= launch + 0.7)] = 1
    lane_path = tmp / "clock_lane.csv"
    pd.DataFrame(
        {
            "step": np.arange(1, n + 1),
            "sim_time_sec": sim,
            "steady_ns": (steady * 1e9).astype(np.int64),
            "launch_seq": seq,
            "ball_active": active,
            "dropped_total": 0,
        }
    ).to_csv(lane_path, index=False)

    def steady_at(s):
        return float(np.interp(s, np.concatenate([[0.0], sim]), np.concatenate([[100.0], steady])))

    trials_dir = tmp / "trials"
    trials_dir.mkdir()
    records = []
    for k, launch in enumerate(LAUNCH_SIMS):
        anchor = K_WALL + steady_at(launch)  # the sim stamps the launch sample at this instant
        ticks = np.arange(launch, launch + EVENT_SIMS[-1], DT)
        # The runner's offset: the MEDIAN of (receipt wall − t_relative_s) over the trial.
        offs = [K_WALL + steady_at(s) + RECV_LATENCY_S - (s - C_TRUE) for s in ticks]
        truth = [
            [K_WALL + steady_at(launch + j * 0.01) + RECV_LATENCY_S, anchor + j * 0.01]
            + [1.0 - j * 0.02, 0.0, 0.2 + j * 0.03, -2.0, 0.0, 3.0]
            for j in range(60)
        ]
        pd.DataFrame(
            truth, columns=["wall_recv_s", "stamp_s", "x", "y", "z", "vx", "vy", "vz"]
        ).to_csv(trials_dir / f"truth_trial_{k:02d}.csv", index=False)
        records.append(
            {
                "idx": k,
                "accepted": True,
                "pos": [1.0, 0.0, 0.2],
                "n_truth_rows": 60,
                "truth_csv": f"/elsewhere/truth_trial_{k:02d}.csv",
                "launch_wall_time": anchor + 0.0003,
                "mode_log": [
                    [K_WALL + steady_at(launch + e) + RECV_LATENCY_S, "M", 0] for e in EVENT_SIMS
                ],
                "wall_t_relative_offset": float(np.median(offs)),
            }
        )
    (trials_dir / "trial_results.json").write_text(json.dumps(records))
    return lane_path, trials_dir, steady_at


def test_a_slow_sim_run_pairs_on_the_wall_clock_where_t_relative_pairing_fails(tmp_path):
    lane_path, trials_dir, _ = _slow_sim_session(tmp_path)
    trials, _ = ct.load_trials(trials_dir)
    lane = ct.load_clock_lane(lane_path, trials)
    assert sorted(lane.trial_to_seq.items()) == [(k, k + 1) for k in range(5)]
    assert lane.wall_offset_s == pytest.approx(K_WALL, abs=5e-3)
    # Positive control: the old key — t_launch from the runner's drifting median
    # offset — no longer pairs the slow trials under one offset.
    steady0 = np.array([lane.segments[k + 1][0, 1] for k in range(5)])
    t_old = np.array([t.t_launch for t in trials])
    assert len(ct._pair_by_offset(steady0, t_old, 0.05)) < 5


def _commit_diag(steady_at, c=C_TRUE):
    """Diag arrays with one commit per trial 0.3 s after launch; the tick runs 0.5 ms
    after the lane row of its state. Returns (t, mode, plan_id, plan_t_c, bridge)."""
    t, mode, pid, ptc = [], [], [], []
    wake, lead = {}, {}
    for k, launch in enumerate(LAUNCH_SIMS):
        sims = np.arange(launch + 0.25, launch + 0.35, DT)
        commit = int(np.argmin(np.abs(sims - (launch + 0.3))))
        for i, s in enumerate(sims):
            t.append(s - c)
            mode.append(ct.MODE_COMMITTED if i >= commit else ct.MODE_APPROACH)
            pid.append(k + 1)
            ptc.append(0.37)
        tick_steady = steady_at(sims[commit]) + 0.0005
        lead[k + 1] = 0.47
        wake[k + 1] = tick_steady + 0.37 - 0.47
    bridge = ct.PlanBridge(wake_s=wake, lead_s=lead, snapshot={})
    return np.array(t), np.array(mode), np.array(pid), np.array(ptc), bridge


def test_sim_minus_t_relative_is_read_from_the_commit_ticks(tmp_path):
    lane_path, trials_dir, steady_at = _slow_sim_session(tmp_path)
    trials, _ = ct.load_trials(trials_dir)
    lane = ct.load_clock_lane(lane_path, trials)
    t, mode, pid, ptc, bridge = _commit_diag(steady_at)
    c, info = ct.estimate_sim_minus_t_rel(t, mode, pid, ptc, bridge, lane)
    assert c == pytest.approx(C_TRUE, abs=1e-9)
    assert info["n"] == 5 and info["spread_s"] < 1e-9
    # A diag shifted by one tick is read as a c one tick smaller — the estimate
    # follows the data, it is not a constant.
    c2, _ = ct.estimate_sim_minus_t_rel(t + DT, mode, pid, ptc, bridge, lane)
    assert c2 == pytest.approx(C_TRUE - DT, abs=1e-9)
    assert ct.estimate_sim_minus_t_rel(t, mode, pid, ptc, None, lane)[0] is None


def test_lane_alignment_is_exact_where_the_median_offset_drifts(tmp_path):
    lane_path, trials_dir, steady_at = _slow_sim_session(tmp_path)
    trials, doc = ct.load_trials(trials_dir)
    records = {r["idx"]: r for r in doc["records"]}
    lane = ct.load_clock_lane(lane_path, trials)
    lane.c_s = C_TRUE
    aligned = ct.align_trials_to_lane(trials, records, lane)
    for old, new, launch, rtf in zip(trials, aligned, LAUNCH_SIMS, TRIAL_RTF, strict=True):
        assert new.alignment == "lane" and new.stamp_anchor == "truth"
        assert new.t_launch == pytest.approx(launch - C_TRUE, abs=1e-6)
        t_end_true = launch + EVENT_SIMS[-1] - C_TRUE
        # Off only by the runner's receipt latency plus the launch-pairing wall
        # offset's few-ms bias, both run through the lane at this trial's RTF.
        assert new.t_end == pytest.approx(t_end_true, abs=5e-3)
        truth = ct.load_truth(new.truth_path, new.stamp_offset)
        assert np.allclose(truth.t, launch + np.arange(60) * 0.01 - C_TRUE, atol=1e-6)
        if rtf < 0.9:
            # Positive control: the runner's median offset puts the launch and
            # the end off by (1 − RTF) × a share of the trial.
            assert abs(old.t_launch - new.t_launch) > 0.1
            assert abs(ct.load_truth(old.truth_path, old.offset).t[0] - truth.t[0]) > 0.1
    # The recv axis goes through the same lane map (wall → steady → sim).
    recv = ct.load_truth(aligned[3].truth_path, math.nan, "recv", lane.wall_to_t_rel)
    assert recv.t[10] == pytest.approx(LAUNCH_SIMS[3] + 0.1 - C_TRUE, abs=5e-3)


def test_rtf_covariates_read_the_lane(tmp_path):
    lane_path, trials_dir, _ = _slow_sim_session(tmp_path)
    trials, _ = ct.load_trials(trials_dir)
    lane = ct.load_clock_lane(lane_path, trials)
    for k, rtf in enumerate(TRIAL_RTF):
        launch = LAUNCH_SIMS[k]
        out = ct.trial_rtf(lane, k + 1, launch + 0.4, launch + 2.5)
        assert out["rtf_flight"] == pytest.approx(rtf, rel=1e-3)
        assert out["rtf_trial_min"] == pytest.approx(rtf, rel=1e-3)


def test_truth_of_another_run_is_refused_and_the_local_copy_wins(tmp_path, capsys):
    import json
    import shutil

    lane_path, trials_dir, _ = _slow_sim_session(tmp_path)
    doc = json.loads((trials_dir / "trial_results.json").read_text())
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    # The recorded absolute path now holds a DIFFERENT run's file (same throw,
    # stamps 1606 s later) and the local copy is gone for trial 0.
    for k, rec in enumerate(doc):
        rec["truth_csv"] = str(elsewhere / f"truth_trial_{k:02d}.csv")
        text = (trials_dir / f"truth_trial_{k:02d}.csv").read_text()
        lines = text.splitlines()
        shifted = [lines[0]] + [
            ",".join(
                [f"{float(v) + 1606.0:.6f}" if i < 2 else v for i, v in enumerate(ln.split(","))]
            )
            for ln in lines[1:]
        ]
        (elsewhere / f"truth_trial_{k:02d}.csv").write_text("\n".join(shifted) + "\n")
    (trials_dir / "trial_results.json").write_text(json.dumps(doc))
    (trials_dir / "truth_trial_00.csv").unlink()
    trials, _ = ct.load_trials(trials_dir)
    assert trials[0].truth_path is None
    assert "another run's truth" in capsys.readouterr().err
    assert trials[1].truth_path == trials_dir / "truth_trial_01.csv"
    shutil.copy(trials_dir / "truth_trial_01.csv", elsewhere / "truth_trial_01.csv")
    assert ct.load_trials(trials_dir)[0][1].truth_path == trials_dir / "truth_trial_01.csv"


def _drifted_fixture(tmp: Path, drift_s: float, drifted: tuple[int, ...]):
    """The pilot's trials dir with the runner's offset pushed by drift_s on some
    trials — the error a median offset makes when the sim ran slow."""
    import json
    import shutil

    trials_dir = tmp / "trials"
    shutil.copytree(FIXTURE / "trials", trials_dir)
    records = json.loads((trials_dir / "trial_results.json").read_text())
    for r in records:
        if r["idx"] in drifted:
            r["wall_t_relative_offset"] += drift_s
    (trials_dir / "trial_results.json").write_text(json.dumps(records))
    return trials_dir


def test_drifted_runner_offsets_do_not_move_the_lane_aligned_analysis(pilot, tmp_path):
    """End to end on the pilot: the runner's median offset pushed 0.4 s on three
    trials. The lane path reads only the launch instant and the lane, so every
    row is the pilot's; the median-offset path (no lane) moves those trials."""
    drifted = (3, 7, 11)
    trials_dir = _drifted_fixture(tmp_path, 0.4, drifted)
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    urdf = (FIXTURE / "robot.urdf").read_text()
    settings = ct.Settings(v_max=PILOT_V_MAX_M_S, n_boot=20)
    lane = ct.analyse_session(FIXTURE / "session", trials_dir, profile, urdf, settings, LANE)
    assert lane.summary["time_alignment"] == "lane"
    detail = lane.summary["time_alignment_detail"]
    assert detail["sim_minus_t_rel_s"] == pytest.approx(0.014, abs=1e-9)
    assert detail["stamp_anchor"] == {"truth": 25, "lane_estimate": 0}
    ref = {r["idx"]: r for r in pilot.rows}
    for r in lane.rows:
        for key in ("t_launch", "first_plan_s", "arrival_ms", "pred_mm"):
            assert r[key] == pytest.approx(ref[r["idx"]][key], abs=1e-9, nan_ok=True), (
                r["idx"],
                key,
            )
        assert r["truth_success"] == ref[r["idx"]]["truth_success"]
    old = ct.analyse_session(FIXTURE / "session", trials_dir, profile, urdf, settings)
    assert old.summary["time_alignment"] == "median_offset"
    rows = {r["idx"]: r for r in old.rows}
    for idx in drifted:
        assert rows[idx]["first_plan_s"] - ref[idx]["first_plan_s"] == pytest.approx(0.4, abs=0.01)


def test_golden_g8b_and_c2_end_to_end(pilot, tmp_path):
    """G8-B binding and G8-C2 through analyse_session on the pilot's own axes.

    Samples: one at the launch (kept), one whose target is after the ball hit
    the robot and one from before the launch (both NEES 1e6 — they must be
    dropped). Dump: the planner snapshot holds p_c (mapped into the sim world)
    at the pilot's t_c, and a snapshot received before the commit (the pilot
    diag predates the input-key columns → the approximate join) holds p_c + 1 cm
    in x, so B is exactly (0.01, 0, 0) and |A + B| is the decomposition's pred.
    """
    import pandas as pd

    row0 = next(
        r
        for r in pilot.rows
        if np.isfinite(r.get("t_first_impact", math.nan)) and np.isfinite(r.get("t_c", math.nan))
    )
    off = row0["stamp_minus_t_rel_s"]
    launch_ns = int(round((row0["t_launch"] + off) * 1e9))
    impact_ns = int(round((row0["t_first_impact"] + off) * 1e9))
    ms = 1_000_000
    samples = pd.DataFrame(
        {
            "record": ["prediction"] * 3,
            "time_ns": [launch_ns, impact_ns, launch_ns - 1000 * ms],
            "horizon_ns": [100 * ms] * 3,
            "ground_truth_stamp_ns": [0] * 3,
            "position_error_x_m": [0.01, 1.0, 1.0],
            "position_error_y_m": [0.0, 1.0, 1.0],
            "position_error_z_m": [0.0, 1.0, 1.0],
            "position_nees": [2.0, 1e6, 1e6],
        }
    )
    samples_path = tmp_path / "eval_samples.csv"
    samples.to_csv(samples_path, index=False)

    ctl = FIXTURE / "session" / "controllers" / "demo_catching_controller"
    diag = ct._read_csv(ctl / "catching_diag.csv")
    k = int(np.argmin(np.abs(diag["t_relative_s"].to_numpy() - row0["t_commit"])))
    plan_id = int(diag["plan_id"].iloc[k])
    p_c_model = diag[["plan_p_c_x", "plan_p_c_y", "plan_p_c_z"]].to_numpy(float)[k]
    events = ct._read_csv(ctl / "planner_events.csv")
    ev = events[(events["plan_id"] == plan_id) & (events["publish_ns"] > 0)].iloc[0]
    profile = ct.load_profile(FIXTURE / "config", session=FIXTURE / "session")
    urdf = (FIXTURE / "robot.urdf").read_text()
    fk = ct.CatchFrameFk(urdf, ct.arm_joints_from_diag(ct._csv_header(_diag_path())), profile)
    p_c_world = ct.transform_point(p_c_model[None], fk.world_t_model)[0]
    t_c_ns = int(round((row0["t_c"] + off) * 1e9))
    plan_t_c = float(diag["plan_t_c_s"].iloc[k])
    commit_steady_ns = int((ev["wake_ns"] + ev["lead_s"] * 1e9) - plan_t_c * 1e9)
    rows = []
    for seq, gen, recv, dx in (
        (
            int(ev["snapshot_sequence"]),
            int(ev["track_generation"]),
            commit_steady_ns - 90 * ms,
            0.0,
        ),
        (99_999, 1, commit_steady_ns - 10 * ms, 0.01),
    ):
        for i in range(20):
            h = (i + 1) * 50 * ms
            stamp = t_c_ns - 500 * ms
            p = p_c_world + [dx, 0.0, 0.0]
            rows.append([recv, "reliable", stamp, "world", seq, gen, h, *p, 0, 0, 0, 0, 0, 0])
    dump = pd.DataFrame(
        rows,
        columns=[
            "recv_ns",
            "sub",
            "stamp_ns",
            "frame_id",
            "snapshot_sequence",
            "generation",
            "horizon_ns",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "ax",
            "ay",
            "az",
        ],
    )
    # Only the 500 ms point of the planner snapshot is p_c (the others are
    # moved away), so the plan-point match picks exactly t_c.
    planner = dump["snapshot_sequence"] == int(ev["snapshot_sequence"])
    dump.loc[planner & (dump["horizon_ns"] != 500 * ms), "x"] += 1.0
    dump_path = tmp_path / "lane_prediction_dump.csv"
    dump.to_csv(dump_path, index=False)

    settings = ct.Settings(v_max=PILOT_V_MAX_M_S, n_boot=20)
    res = ct.analyse_session(
        FIXTURE / "session",
        FIXTURE / "trials",
        profile,
        urdf,
        settings,
        LANE,
        FIXTURE / "session" / "sim" / "ball_contact_lane.csv.gz",
        eval_samples_path=samples_path,
        probe_dump_path=dump_path,
    )
    r0 = next(r for r in res.rows if r["idx"] == row0["idx"])
    assert r0["nees_h100ms_n"] == 1 and r0["nees_h100ms_mean"] == pytest.approx(2.0)
    assert all(r.get("nees_h100ms_n", 0) == 0 for r in res.rows if r["idx"] != row0["idx"])
    assert r0["c2_tc_source"] == "plan_point" and r0["c2_pc_match_mm"] < 1e-6
    assert r0["c2_tc_minus_tc_ms"] == pytest.approx(0.0, abs=1e-3)
    assert r0["c2_join"] == "approx"  # no input-key columns in the pilot diag
    assert (r0["c2_B_x"], r0["c2_B_y"], r0["c2_B_z"]) == pytest.approx((0.01, 0.0, 0.0))
    a_plus_b = np.array([r0[f"c2_A_{x}"] + r0[f"c2_B_{x}"] for x in "xyz"])
    # t_c went through an int ns stamp (~0.2 µs of float resolution at epoch scale).
    assert np.linalg.norm(a_plus_b) * 1e3 == pytest.approx(r0["pred_mm"], abs=1e-2)
    c2 = res.summary["c2"]
    assert c2["n_approx"] >= 1 and c2["independence"] == "NOT_EVALUATED(n < 100)"
