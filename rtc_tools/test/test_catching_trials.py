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
