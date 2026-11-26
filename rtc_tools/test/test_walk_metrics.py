"""rtc_tools.analysis.walk_metrics on synthetic sessions.

The measured planar velocity reproduces the gait scheduler's cases
(rtc_mpc test_locomotion_phase_manager, TheSnapshotVelocityOnTheYawAlignedAxes:
the same inputs, the same expected values). The rest is a synthetic walk that
tracks its command exactly: every error is then 0 and every count is known.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from rtc_tools.analysis import walk_metrics as wm

GAITS = ["stance", "slow_walk", "walk"]
DT = 0.002
A = 5.0  # the ramp's start
WEIGHT = 330.0
DELAY = 0.02  # the measured contact edge after the planned one [s]
# The ramp is found where cmd_vx first exceeds 0.01: that far into its 0.25 m/s² slope.
A_FOUND = A + 0.01 / 0.25
CYCLE = 1.4  # the synthetic gait's walk cycle [s]


def test_the_planar_velocity_is_the_schedulers():
    psi = 0.3
    vx, vy, wz = wm.planar_velocity(
        (0.0, 0.0, math.sin(0.5 * psi), math.cos(0.5 * psi)), (0.2, 0.1, 0.05), (0.0, 0.0, 0.4)
    )
    assert vx[0] == pytest.approx(0.2, abs=1e-15)
    assert vy[0] == pytest.approx(0.1, abs=1e-15)
    assert wz[0] == pytest.approx(0.4, abs=1e-15)
    theta = 0.2
    vx, vy, wz = wm.planar_velocity(
        (0.0, math.sin(0.5 * theta), 0.0, math.cos(0.5 * theta)), (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    )
    assert vx[0] == pytest.approx(math.cos(theta), abs=1e-15)
    assert vy[0] == pytest.approx(0.0, abs=1e-15)
    assert wz[0] == pytest.approx(-math.sin(theta), abs=1e-15)
    vx, _, wz = wm.planar_velocity((0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert vx[0] == 0.0 and wz[0] == 0.0  # no rotation at all


def test_a_yawed_and_pitched_base_reads_its_ground_speed():
    # Yaw ψ then pitch θ (R = Rz(ψ) Ry(θ)): a LOCAL forward speed u is u cos θ
    # along the heading, whatever ψ is.
    psi, theta, u = 1.1, 0.15, 0.4
    qz = (0.0, 0.0, math.sin(psi / 2), math.cos(psi / 2))
    qy = (0.0, math.sin(theta / 2), 0.0, math.cos(theta / 2))
    x1, y1, z1, w1 = qz[0], qz[1], qz[2], qz[3]
    x2, y2, z2, w2 = qy
    q = (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )
    vx, vy, _ = wm.planar_velocity(q, (u, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert vx[0] == pytest.approx(u * math.cos(theta), abs=1e-12)
    assert vy[0] == pytest.approx(0.0, abs=1e-12)
    assert float(wm.yaw(*q)) == pytest.approx(psi, abs=1e-12)


def _gait_mask(p: float) -> int:
    """Both soles, left only, both, right only over a CYCLE (1.4 s) cycle."""
    if p < 0.1:
        return 3
    if p < 0.7:
        return 1
    if p < 0.8:
        return 3
    return 2


def synthetic(swap_slots: bool = False, settle_noise_s: float = 0.0):
    t = np.arange(0.0, 40.0, DT)
    n = len(t)
    cmd = np.array([wm.command_at(wm.WALK_SCRIPT, x - A) for x in t])
    e = A + 16.0
    psi = np.cumsum(cmd[:, 2]) * DT
    vxw = cmd[:, 0] * np.cos(psi)
    vyw = cmd[:, 0] * np.sin(psi)
    px = np.cumsum(vxw) * DT
    py = np.cumsum(vyw) * DT
    walking = (t >= A + 1.0) & (t < e + 2.0)
    mask = np.where(walking, [_gait_mask((x - A - 1.0) % CYCLE) for x in t], 3)
    meas_mask = np.where(walking, [_gait_mask((x - A - 1.0 - DELAY) % CYCLE) for x in t], 3)
    meas_mask[t < A + 1.0 + DELAY] = 3
    fz = np.zeros((n, 2))
    for c in range(2):
        on = (meas_mask >> c) & 1 == 1
        both = meas_mask == 3
        fz[:, c] = np.where(both, WEIGHT / 2, np.where(on, WEIGHT, 0.0))  # ground → foot
    if swap_slots:
        fz = fz[:, ::-1]
    gait = np.select(
        [t < A + 1.0, t < A + 3.0, t < e + 1.0, t < e + 2.5], [0, 1, 2, 1], default=0
    ).astype(float)
    vx_meas = cmd[:, 0].copy()
    if settle_noise_s > 0:
        vx_meas[(t >= e) & (t < e + settle_noise_s)] = 0.1
    zeros = np.zeros(n)
    diag = {
        "t_relative_s": t,
        "tick": np.arange(n, dtype=float),
        "mode": np.ones(n),
        "fault_latched": zeros,
        "fault_count": zeros,
        "fault_cause": zeros,
        "pose_guard_mask": zeros,
        "source_status": zeros,
        "health": zeros,
        "tau_valid": np.ones(n),
        "qp_status": zeros,
        "rows_dropped": zeros,
        "eom_residual_inf": np.full(n, 1e-9),
        "margins_stale": zeros,
        "base_px": px,
        "base_py": py,
        "base_pz": np.full(n, 0.75),
        "base_qx": zeros,
        "base_qy": zeros,
        "base_qz": np.sin(psi / 2),
        "base_qw": np.cos(psi / 2),
        "base_vx": vx_meas,  # LOCAL: the heading's forward speed under a pure yaw
        "base_vy": zeros,
        "base_vz": zeros,
        "base_wx": zeros,
        "base_wy": zeros,
        "base_wz": cmd[:, 2],
        "com_x": px,
        "com_y": py,
        "ref_source": np.ones(n),
        "plausibility_rejects": zeros,
        "contact_mask": mask.astype(float),
        "policy_age_us": np.full(n, 50e3),
        "wbc_tick_us": np.full(n, 200.0),
        "cmd_vx": cmd[:, 0],
        "cmd_vy": zeros,
        "cmd_wz": cmd[:, 2],
        "cmd_valid": np.ones(n),
        "gait_latest": gait,
        "ref_base_vx": cmd[:, 0],
        "ref_base_vy": zeros,
        "ref_base_vz": zeros,
        # The contact names (and their order) come from this block.
        "lambda_left_fz": zeros,
        "lambda_right_fz": zeros,
    }
    for m in wm.MARGINS:
        diag[m] = np.ones(n)
    header = list(diag)
    # The truth file on the diag rows: bit 0 the base lane, bit 1 + c contact c.
    truth = {
        "tick": diag["tick"],
        "truth_mask": np.full(n, 7.0),
        "truth_grf_left_fz": fz[:, 0],
        "truth_grf_right_fz": fz[:, 1],
    }
    return header, diag, truth


def test_the_segments_are_placed_from_the_command_column():
    header, diag, truth = synthetic()
    seg = wm.find_segments(diag["t_relative_s"], diag["cmd_vx"])
    assert seg is not None
    assert seg.a == pytest.approx(A_FOUND, abs=2 * DT)
    assert seg.e == pytest.approx(A + 16.0, abs=2 * DT)
    assert seg.bounds["B"] == pytest.approx((seg.a + 2.0, seg.a + 10.0))
    assert seg.bounds["E"] == pytest.approx((seg.e, seg.e + 10.0))


def test_the_windows_are_whole_walk_cycles():
    assert wm.window_lengths(1.1) == pytest.approx((4.4, 2.2, 1.1))
    # The yaw window ends A_FOUND − A into D (ω 0): that share of 0.3 rad/s over
    # two cycles of the period the caller gives, not of a fixed length.
    header, diag, truth = synthetic()
    s = wm.walk_summary(header, diag, GAITS, walk_period_s=1.1, truth=truth)
    assert s["tracking"]["yaw_rate_mean_err"] == pytest.approx(
        0.3 * (A_FOUND - A) / 2.2, abs=0.3 * 2 * DT / 2.2
    )
    # Four cycles longer than B (8 s) would read A: refused, not averaged over.
    assert "error" in wm.walk_summary(header, diag, GAITS, walk_period_s=2.1, truth=truth)


def test_the_infeasible_rows_are_the_two_infeasibility_verdicts():
    # proxsuite QPSolverOutput: 2 PRIMAL_INFEASIBLE, 3 SOLVED_CLOSEST_PRIMAL_FEASIBLE,
    # 4 DUAL_INFEASIBLE. A NaN status (an unreadable row) counts too.
    header, diag, truth = synthetic()
    base = wm.walk_summary(header, diag, GAITS, walk_period_s=1.1, truth=truth)
    assert base["counters"]["qp_infeasible_rows"] == 0
    t = diag["t_relative_s"]
    rows = np.flatnonzero((t > A + 5.0) & (t < A + 6.0))[:45]
    status = np.zeros(len(t))
    status[rows[0:10]] = 2
    status[rows[10:15]] = 3
    status[rows[15:35]] = 4
    status[rows[35:45]] = np.nan
    s = wm.walk_summary(
        header, {**diag, "qp_status": status}, GAITS, walk_period_s=1.1, truth=truth
    )
    assert s["counters"]["qp_infeasible_rows"] == 10 + 20 + 10


@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf])
def test_a_walk_period_that_is_not_positive_is_refused(bad):
    with pytest.raises(ValueError):
        wm.window_lengths(bad)


def _with_length(name: str, length_s: float) -> tuple:
    """WALK_SCRIPT with segment `name` `length_s` long."""
    return tuple(
        wm.Segment(s.name, length_s, s.vx0, s.vx1, s.wz) if s.name == name else s
        for s in wm.WALK_SCRIPT
    )


@pytest.mark.parametrize(
    "period, script, fits",
    [
        (2.0, wm.WALK_SCRIPT, True),  # 8 s in B, 4 s in C: exactly fits
        (2.1, wm.WALK_SCRIPT, False),  # the v_x window outgrows B
        (1.0, _with_length("C", 2.0), True),  # 2 cycles = 2 s = C
        (1.1, _with_length("C", 2.0), False),  # the yaw window outgrows C
        (1.0, _with_length("B", 4.0), True),  # 4 cycles = 4 s = B
        (1.1, _with_length("B", 4.0), False),  # v_x outgrows B, yaw (2.2 s) fits C
    ],
)
def test_a_window_longer_than_its_segment_is_refused(period, script, fits):
    assert (wm.windows_fit(period, script) is None) == fits


def test_an_exact_tracker_scores_zero_error_and_known_counts():
    header, diag, truth = synthetic()
    s = wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)
    assert s["tracking"]["vx_err_rms"] == pytest.approx(0.0, abs=1e-12)
    assert s["tracking"]["vx_mean_err"] == pytest.approx(0.0, abs=1e-9)
    assert s["tracking"]["yaw_rate_err_rms"] == pytest.approx(0.0, abs=1e-12)
    # The yaw window ends A_FOUND − A into D (ω 0): that share of 0.3 rad/s.
    assert s["tracking"]["yaw_rate_mean_err"] == pytest.approx(
        0.3 * (A_FOUND - A) / 2.8, abs=0.3 * 2 * DT / 2.8
    )
    # B ends A_FOUND − A into the turn: ½ v ω t² of curve, under a millimetre.
    assert s["drift"]["lateral_drift_m"] == pytest.approx(0.0, abs=1e-3)
    assert s["drift"]["heading_drift_rad"] == pytest.approx(0.0, abs=1e-3)  # the same turn
    assert s["stop"]["stop_settle_s"] == pytest.approx(0.0, abs=2 * DT)
    assert s["stop"]["stop_to_stance_gait_s"] == pytest.approx(2.5, abs=2 * DT)
    assert s["gait"]["walk_entry_s_after_ramp_end"] == pytest.approx(
        1.0 - (A_FOUND - A), abs=2 * DT
    )
    assert s["gait"]["walk_fraction_from_entry"] == 1.0
    assert s["gait"]["changes_b_to_d"] == 1  # slow_walk → walk inside B
    assert s["gait"]["down_changes_b_to_d"] == 0
    c = s["contacts"]
    assert c["names"] == ["left", "right"]
    assert c["weight_n"] == pytest.approx(WEIGHT)
    assert c["missed_edges"] == 0 and c["extra_edges"] == 0
    assert c["mismatch_abs_max_ms"] == pytest.approx(DELAY * 1e3, abs=2 * DT * 1e3)
    assert c["mismatch_signed_mean_ms"] == pytest.approx(DELAY * 1e3, abs=2 * DT * 1e3)
    assert c["slot_check_failures"] == 0
    assert c["single_support_phases"] > 0
    # Touch-downs in [B start, D end) = [A + 2, A + 16): each 1.4 s cycle from
    # A + 1 has two (left back in at 0.7 s... both bits rise once per cycle).
    cycle_start = A + 1.0
    expected = 0
    for k in range(20):
        for rise in (0.7, 1.4):  # bit 1 at 0.7 (mask 1 → 3), bit 0 at 1.4 (2 → 3)
            tr = cycle_start + k * 1.4 + rise
            expected += A + 2.0 <= tr < A + 16.0
    assert s["steps_completed"] == expected
    assert s["counters"]["cmd_invalid_rows_b_to_d"] == 0
    assert s["events"]["tau_invalid_rows"] == 0
    assert s["events"]["rows"] > 0
    assert s["report"]["plan_vx_bias"] == pytest.approx(0.0, abs=1e-12)
    assert s["report"]["policy_age_p99_ms"] == pytest.approx(50.0)


def test_swapped_force_slots_fail_the_slot_check():
    header, diag, truth = synthetic(swap_slots=True)
    c = wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)["contacts"]
    assert c["slot_check_failures"] == c["single_support_phases"] > 0


def test_the_stop_waits_for_a_continuous_second_of_stillness():
    header, diag, truth = synthetic(settle_noise_s=2.0)
    s = wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)
    assert s["stop"]["stop_settle_s"] == pytest.approx(2.0, abs=2 * DT)


def test_a_stale_force_sample_is_not_a_contact_change():
    header, diag, truth = synthetic()
    truth["truth_mask"][(diag["t_relative_s"] > 30.0)] = 5.0  # base + right: left stale
    truth["truth_grf_left_fz"][diag["t_relative_s"] > 30.0] = 0.0  # a held value, not read
    c = wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)["contacts"]
    assert c["extra_edges"] == 0
    assert c["invalid_force_rows"] > 0


def test_edges_pair_one_to_one():
    planned = [(1.0, True), (1.05, True)]
    measured = [(1.02, True)]
    m = wm.match_edges(planned, measured, 0.2)
    assert m.missed == 1 and m.extra == 0 and m.dt_s == pytest.approx([0.02])
    m = wm.match_edges([(1.0, True)], [(1.02, False), (1.5, True)], 0.2)
    assert m.missed == 1 and m.extra == 2, "a falling edge and one too far are not a match"


def test_the_measured_contact_has_hysteresis_and_holds_through_nan():
    f = np.array([0.0, 20.0, 10.0, math.nan, 5.0, 1.0])
    assert wm.measured_contact(f, 15.0, 6.0).tolist() == [False, True, True, True, False, False]


def test_a_touch_down_must_hold():
    t = np.arange(10) * DT
    bits = np.array([[0], [1], [1], [0], [1], [1], [1], [1], [0], [0]], dtype=bool)
    assert wm.steps(bits, t, 0.0, 1.0, hold_ticks=3) == 1


def test_no_script_is_an_error_not_a_pass():
    header, diag, truth = synthetic()
    diag["cmd_vx"] = np.zeros_like(diag["cmd_vx"])
    assert "error" in wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)


def test_an_infinite_residual_or_speed_is_judged_not_dropped():
    header, diag, truth = synthetic()
    diag["eom_residual_inf"] = diag["eom_residual_inf"].copy()
    diag["eom_residual_inf"][:] = math.inf
    diag["base_vx"] = diag["base_vx"].copy()
    diag["base_vx"][diag["t_relative_s"] > A + 6.0] = math.inf
    s = wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)
    assert s["events"]["eom_residual_inf_max"] == math.inf
    assert s["tracking"]["vx_err_rms"] == math.inf


def test_a_nan_flag_counts_as_a_failed_row():
    header, diag, truth = synthetic()
    for col in ("cmd_valid", "tau_valid", "fault_latched", "mode"):
        diag[col] = diag[col].copy()
        diag[col][(diag["t_relative_s"] > A + 3.0) & (diag["t_relative_s"] < A + 3.1)] = math.nan
    s = wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)
    assert s["counters"]["cmd_invalid_rows_b_to_d"] > 0
    assert s["counters"]["fault_latched_rows"] > 0
    assert s["counters"]["degraded_fraction"] > 0


def test_a_mostly_nan_window_reports_its_finite_fraction():
    header, diag, truth = synthetic()
    diag["base_vx"] = diag["base_vx"].copy()
    diag["base_vx"][(diag["t_relative_s"] > A + 4.0) & (diag["t_relative_s"] < A + 10.0)] = (
        math.nan
    )
    s = wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)
    assert s["tracking"]["vx_window_finite_fraction"] < 0.1


def test_an_unreadable_first_force_sample_makes_no_edge():
    f = np.array([math.nan, 20.0, 20.0, 1.0])
    assert wm.measured_contact(f, 15.0, 6.0).tolist() == [True, True, True, False]


@pytest.mark.parametrize("name", sorted(wm.COMMAND_SCRIPTS))
def test_every_command_script_is_as_long_as_the_walk_and_ends_in_its_stop(name):
    script = wm.COMMAND_SCRIPTS[name]
    assert wm.script_length(script) == pytest.approx(wm.script_length(wm.WALK_SCRIPT))
    assert script[-1].length_s == wm.WALK_SCRIPT[-1].length_s
    assert (script[-1].vx0, script[-1].vx1, script[-1].wz) == (0.0, 0.0, 0.0)
    assert len({s.name for s in script}) == len(script)


def test_the_walk_command_script_is_the_judged_one():
    assert wm.COMMAND_SCRIPTS["walk"] is wm.WALK_SCRIPT


@pytest.mark.parametrize(("name", "wz"), [("turn", 0.4), ("turn_slow", 0.3)])
def test_the_turn_scripts_turn_in_place_both_ways_with_no_net_heading(name, wz):
    walking = wm.COMMAND_SCRIPTS[name][:-1]
    assert all(s.vx0 == s.vx1 == 0.0 for s in walking)
    assert sorted({s.wz for s in walking}) == [-wz, wz]
    assert sum(s.wz * s.length_s for s in walking) == pytest.approx(0.0)
    assert wm.script_length(walking) == pytest.approx(16.0)


@pytest.mark.parametrize(("name", "wz"), [("curve", 0.4), ("curve_slow", 0.3)])
def test_the_curve_scripts_walk_with_a_3_s_turn_each_way(name, wz):
    script = wm.COMMAND_SCRIPTS[name]
    turns = [s for s in script if s.wz != 0.0]
    assert [(s.wz, s.length_s, s.vx0, s.vx1) for s in turns] == [
        (wz, 3.0, 0.5, 0.5),
        (-wz, 3.0, 0.5, 0.5),
    ]
    assert script[0] == wm.WALK_SCRIPT[0]  # the same ramp
    assert wm.command_at(script, 7.5) == (0.5, 0.0, wz)


def test_without_a_truth_file_the_contact_checks_have_no_numbers():
    header, diag, _ = synthetic()
    s = wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE)
    assert s["contacts"]["names"] == ["left", "right"]
    assert "error" in s["contacts"] and "missed_edges" not in s["contacts"]
    # What the plan alone gives is still there.
    assert s["steps_completed"] > 0 and s["events"]["rows"] > 0


def test_a_log_that_names_no_contact_is_an_error_not_a_pass():
    header, diag, truth = synthetic()
    header = [h for h in header if not h.startswith("lambda_")]
    assert "error" in wm.walk_summary(header, diag, GAITS, walk_period_s=CYCLE, truth=truth)
