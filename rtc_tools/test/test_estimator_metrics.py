"""rtc_tools.analysis.estimator_metrics on synthetic replays with known answers.

Each test builds a replay table the way ``estimator_replay`` writes one — a
truth trajectory, an estimate made from it by a known error and a known gauge
(yaw and translation) — and checks the metric against the value the
construction implies, to 1e-9.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from rtc_tools.analysis import estimator_metrics as em

TOL = 1e-9
DT = 0.002
NAMES = ["left_foot", "right_foot"]


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def exp_so3(w):
    th = np.linalg.norm(w)
    k = em.skew(np.asarray(w, dtype=float)[None])[0]
    if th < 1e-12:
        return np.eye(3) + k
    return np.eye(3) + math.sin(th) / th * k + (1 - math.cos(th)) / th**2 * (k @ k)


def left_jacobian(w):
    th = np.linalg.norm(w)
    k = em.skew(np.asarray(w, dtype=float)[None])[0]
    return np.eye(3) + (1 - math.cos(th)) / th**2 * k + (th - math.sin(th)) / th**3 * (k @ k)


def quat_xyzw(r):
    """A rotation matrix's (x, y, z, w) quaternion (w > 0 branch; no π rotations here)."""
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    return (
        (r[2, 1] - r[1, 2]) / (4 * w),
        (r[0, 2] - r[2, 0]) / (4 * w),
        (r[1, 0] - r[0, 1]) / (4 * w),
        w,
    )


R_BI = rx(0.1) @ rz(-0.05)
P_BI = np.array([0.04, 0.0, -0.08])
META = {"contacts": NAMES, "R_BI": list(R_BI.reshape(-1)), "p_BI": list(P_BI), "gravity": 9.81,
        "weight_n": 300.0}  # fmt: skip


def base_table(n: int) -> dict:
    """Zeros / ones for every column the metrics read; the tests fill the rest."""
    z = np.zeros(n)
    cols = {
        "tick": np.arange(n, dtype=float),
        "t_relative_s": np.arange(n) * DT,
        "gap": z.copy(),
        "initialized": np.ones(n),
        "state_epoch": np.ones(n),
        "reset_count": np.ones(n),
        "periods": np.ones(n),
        "report_valid": np.zeros(n),
        "truth_valid": np.ones(n),
        "in_imu_valid": np.ones(n),
        "planned_mask": np.full(n, 3.0),
        "est_mask": np.full(n, 3.0),
    }
    for c in NAMES:
        cols[f"truth_fz_{c}"] = np.full(n, 150.0)
        cols[f"mode_{c}"] = np.full(n, 2.0)
        cols[f"update_{c}"] = z.copy()
        cols[f"m2_{c}"] = z.copy()
        for a in "xyz":
            cols[f"truth_v_{c}_{a}"] = z.copy()
    for i in range(6):
        for j in range(i, 6):
            cols[f"P_{i}_{j}"] = np.full(n, 1.0 if i == j else 0.0)
    for a in "xyz":
        cols[f"in_imu_a{a}"] = z.copy()
    return cols


def put_pose(cols, prefix, rots, pos):
    q = np.array([quat_xyzw(r) for r in rots])
    for i, a in enumerate("xyzw"):
        cols[f"{prefix}q{a}"] = q[:, i]
    for i, a in enumerate("xyz"):
        cols[f"{prefix}p{a}"] = pos[:, i].copy()


def put_vec(cols, prefix, v):
    for i, a in enumerate("xyz"):
        cols[f"{prefix}{a}"] = v[:, i].copy()


def walking_truth(n, yaw_rate=0.2):
    """Pure-yaw base walking along x with a lateral sway; base-frame twist."""
    t = np.arange(n) * DT
    psi = yaw_rate * t
    rots = [rz(p) for p in psi]
    pos = np.stack([0.5 * t, 0.1 * np.sin(t), np.full(n, 0.7)], axis=1)
    vel_w = np.stack([np.full(n, 0.5), 0.1 * np.cos(t), np.zeros(n)], axis=1)
    v_b = np.einsum("nji,nj->ni", np.array(rots), vel_w)
    w_b = np.tile([0.0, 0.0, yaw_rate], (n, 1))
    return t, np.array(rots), pos, vel_w, v_b, w_b


def put_truth(cols, rots, pos, v_b, w_b):
    put_pose(cols, "truth_", rots, pos)
    put_vec(cols, "truth_v", v_b)
    put_vec(cols, "truth_w", w_b)


def put_imu_state(cols, rots_wb, pos_wb, vel_w_imu, gauge=np.eye(3), xi_phi=None, xi_v=None):
    """The filter's IMU state on `gauge` (its own world), with a known error."""
    r_wi = np.array([gauge @ r @ R_BI for r in rots_wb])
    v = np.einsum("ij,nj->ni", gauge, vel_w_imu)
    if xi_phi is not None:
        r_hat = np.array([exp_so3(-xi_phi) @ r for r in r_wi])
        e = np.array([r @ rh.T for r, rh in zip(r_wi, r_hat, strict=False)])
        v_hat = np.array(
            [er.T @ (vi - left_jacobian(xi_phi) @ xi_v) for er, vi in zip(e, v, strict=False)]
        )
    else:
        r_hat, v_hat = r_wi, v
    p = np.einsum("ij,nj->ni", gauge, pos_wb)
    put_pose(cols, "imu_", r_hat, p)
    put_vec(cols, "imu_v", v_hat)


# ── SO(3) and χ² ───────────────────────────────────────────────────────────


def test_the_log_and_the_left_jacobian_inverse_invert_their_forward_maps():
    w = np.array([[0.3, -0.2, 0.5], [1e-8, 0.0, 2e-8], [0.0, 0.0, 0.0]])
    for wi in w:
        assert np.allclose(em.so3_log(exp_so3(wi)[None])[0], wi, atol=TOL)
        if np.linalg.norm(wi) > 1e-6:
            assert np.allclose(em.left_jacobian_inverse(wi[None])[0] @ left_jacobian(wi), np.eye(3),
                               atol=TOL)  # fmt: skip


@pytest.mark.parametrize(
    ("p", "dof", "want"),
    [(0.975, 5, 12.832502), (0.025, 5, 0.831212), (0.975, 3, 9.348404), (0.025, 40, 24.433039),
     (0.975, 40, 59.341707), (0.025, 9, 2.700389)],
)  # fmt: skip
def test_the_chi2_quantiles_are_the_tables(p, dof, want):
    assert em.chi2_quantile(p, dof) == pytest.approx(want, abs=1e-5)


def test_the_k_run_bands_are_the_registered_ones():
    lo, hi = em.mean_band(8 * 5)
    assert (round(lo, 3), round(hi, 3)) == (0.611, 1.484)
    lo, hi = em.mean_band(3 * 3)
    assert (round(lo, 3), round(hi, 3)) == (0.300, 2.114)


# ── attitude, velocity, position under a gauge ─────────────────────────────


def test_tilt_yaw_and_velocity_errors_are_the_injected_ones_on_any_gauge():
    n = 500
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    alpha, beta, dv = 0.02, 0.01, np.array([0.03, -0.04, 0.0])
    gauge = rz(0.7)
    shift = np.array([1.0, -2.0, 0.3])
    t0 = t[0]
    est_rot = np.array(
        [gauge @ rz(beta * (ti - t0)) @ r @ rx(alpha) for ti, r in zip(t, rots, strict=False)]
    )
    est_pos = np.einsum("ij,nj->ni", gauge, pos) + shift
    put_pose(cols, "base_", est_rot, est_pos)
    put_vec(cols, "base_v", v_b + dv)
    put_imu_state(cols, rots, pos, vel_w, gauge)
    m = em.run_metrics(cols, META)
    assert m["scored_ticks"] == n
    assert m["tilt_rms_truth"] == pytest.approx(alpha, abs=TOL)
    # The gauge yaw is the one of E = R R̂ᵀ at the epoch's first tick
    # (atan2(E10, E00)): with a tilt present it is not 0.7 exactly.
    e0 = rots[0] @ est_rot[0].T
    psi0 = math.atan2(e0[1, 0], e0[0, 0])
    assert abs(psi0 + 0.7) < 1e-3
    rel = [r @ (rz(psi0) @ rh).T for r, rh in zip(rots, est_rot, strict=False)]
    want_yaw = np.array([math.atan2(e[1, 0], e[0, 0]) for e in rel])
    assert np.max(np.abs(want_yaw + beta * (t - t0))) < 1e-3
    assert m["yaw_rms_truth"] == pytest.approx(math.sqrt(np.mean(want_yaw**2)), abs=TOL)
    assert m["vel_err_rms_truth"] == pytest.approx(np.linalg.norm(dv), abs=TOL)
    geo = [math.acos(min(1.0, (np.trace(e) - 1) / 2)) for e in rel]
    assert m["att_err_rms_truth"] == pytest.approx(math.sqrt(np.mean(np.square(geo))), abs=TOL)


def test_the_gauge_is_aligned_again_at_every_state_epoch():
    n = 400
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    half = n // 2
    est_rot, est_pos = [], []
    for k in range(n):
        g = rz(0.3) if k < half else rz(-1.1)  # a reset restarts the estimate on another gauge
        s = np.array([0.5, 0.0, 0.0]) if k < half else np.array([-3.0, 4.0, 0.0])
        est_rot.append(g @ rots[k])
        est_pos.append(g @ pos[k] + s)
    cols["state_epoch"][half:] = 2
    put_pose(cols, "base_", np.array(est_rot), np.array(est_pos))
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, vel_w)
    m = em.run_metrics(cols, META)
    assert m["yaw_rms_truth"] == pytest.approx(0.0, abs=TOL)
    assert m["pos_drift_m_truth"] == pytest.approx(0.0, abs=TOL)


def test_the_position_drift_per_second_per_step_and_without_the_yaw_error():
    n = 1001
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n, yaw_rate=0.0)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    d = 0.01  # m/s of drift along x
    est_pos = pos + np.stack([d * t, np.zeros(n), np.zeros(n)], axis=1)
    put_pose(cols, "base_", rots, est_pos)
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, vel_w)
    # Four touchdowns of the left foot, held well past the dwell.
    fz = np.full(n, 150.0)
    for k0 in (100, 300, 500, 700):
        fz[k0 - 60 : k0] = 0.0
    cols["truth_fz_left_foot"] = fz
    m = em.run_metrics(cols, META)
    total = d * t[-1]
    assert m["pos_drift_m_truth"] == pytest.approx(total, abs=TOL)
    assert m["pos_drift_per_s_truth"] == pytest.approx(d, abs=TOL)
    assert m["touchdowns_truth"] == 4
    assert m["pos_drift_per_step_truth"] == pytest.approx(total / 4, abs=TOL)
    assert m["pos_drift_noyaw_m_truth"] == pytest.approx(total, abs=TOL)

    # A pure yaw error moves the estimate without a drift the yaw does not explain.
    beta = 0.05
    est_rot = np.array([rz(beta * ti) @ r for ti, r in zip(t, rots, strict=False)])
    est_pos = np.array(
        [rz(beta * ti) @ (p - pos[0]) + pos[0] for ti, p in zip(t, pos, strict=False)]
    )
    put_pose(cols, "base_", est_rot, est_pos)
    m = em.run_metrics(cols, META)
    assert m["pos_drift_noyaw_m_truth"] == pytest.approx(0.0, abs=TOL)
    assert m["pos_drift_m_truth"] > 1e-3


# ── NEES / NIS ─────────────────────────────────────────────────────────────


def test_the_nees_is_the_injected_error_over_the_tilt_velocity_block_on_any_gauge():
    n = 50
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    gauge = rz(-0.4)
    put_pose(
        cols, "base_", np.array([gauge @ r for r in rots]), np.einsum("ij,nj->ni", gauge, pos)
    )
    put_vec(cols, "base_v", v_b)
    vel_wi = np.einsum("nij,nj->ni", rots, v_b + np.cross(w_b, P_BI))
    xi_phi = np.array([0.01, -0.02, 0.3])  # the yaw part is not scored
    xi_v = np.array([0.05, 0.02, -0.01])
    put_imu_state(cols, rots, pos, vel_wi, gauge, xi_phi, xi_v)
    p = np.diag([4e-4, 1e-4, 9.0, 1e-3, 2e-3, 4e-3])
    p[0, 3] = p[3, 0] = 1e-4  # a cross term the block keeps
    p[2, 4] = p[4, 2] = 0.5  # one with the yaw row, which the projection drops
    for i in range(6):
        for j in range(i, 6):
            cols[f"P_{i}_{j}"] = np.full(n, p[i, j])
    cols["periods"][::5] = 0  # held ticks are not scored
    for i in range(6):  # one row whose block is not positive definite: no NEES, counted
        cols[f"P_{i}_{i}"][1] = 0.0
    m = em.run_metrics(cols, META)
    assert m["nees_non_spd_ticks"] == 1
    e = np.array([xi_phi[0], xi_phi[1], *xi_v])
    keep = [0, 1, 3, 4, 5]
    want = float(e @ np.linalg.solve(p[np.ix_(keep, keep)], e))
    assert m["nees_ticks"] == n - n // 5 - 1
    assert m["nees_mean_per_dof_truth"] == pytest.approx(want / 5, rel=1e-9)


def test_the_nis_counts_every_used_and_gated_update_and_nothing_else():
    n = 10
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    put_pose(cols, "base_", rots, pos)
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, vel_w)
    cols["report_valid"][:] = 1
    cols["report_valid"][9] = 0
    cols["update_left_foot"][:] = [2, 3, 1, 4, 0, 2, 2, 3, 2, 2]
    cols["m2_left_foot"][:] = [1, 30, 99, 99, 99, 2, 3, 12, 6, 99]
    m = em.run_metrics(cols, META)
    used = [1, 30, 2, 3, 12, 6]
    assert m["nis_updates"] == len(used)
    assert m["nis_mean_per_dof"] == pytest.approx(np.mean(used) / 3, abs=TOL)


def test_the_k_run_verdict_uses_the_run_means():
    runs = [{"nees_mean_per_dof_truth": x, "nis_mean_per_dof": 1.0} for x in (0.9, 1.1, 1.3, 1.5)]
    agg = em.aggregate(runs)
    assert agg["anees_truth"]["value"] == pytest.approx(1.2, abs=TOL)
    assert agg["anees_truth"]["k"] == 4
    lo, hi = em.mean_band(20)
    assert agg["anees_truth"]["band"] == [lo, hi]
    assert agg["anees_truth"]["pass"] is (lo <= 1.2 <= hi)


# ── contact ────────────────────────────────────────────────────────────────


def contact_case():
    """Left foot: swing [200, 300) and [600, 700) by truth force; planned 5 ticks early."""
    n = 1000
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    put_pose(cols, "base_", rots, pos)
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, vel_w)
    gt = np.ones(n, dtype=bool)
    for a, b in ((200, 300), (600, 700)):
        gt[a:b] = False
    cols["truth_fz_left_foot"] = np.where(gt, 150.0, 0.0)
    planned = np.ones(n, dtype=bool)
    for a, b in ((195, 295), (595, 695)):
        planned[a:b] = False
    cols["planned_mask"] = np.where(planned, 3.0, 2.0)
    return n, cols, gt, planned


def test_contact_precision_recall_latency_against_the_truth_force():
    n, cols, gt, planned = contact_case()
    est = gt.copy()
    # The estimate: touchdowns 7 ticks late, liftoffs 3 ticks late, one false
    # stance tick far from every edge.
    for a, b in ((200, 300), (600, 700)):
        est[a : a + 3] = True
        est[b : b + 7] = False
    est[450] = False  # a missed stance tick in a long stance
    est[250] = True  # a false stance tick mid-swing
    cols["est_mask"] = np.where(est, 3.0, 2.0)
    m = em.run_metrics(cols, META)
    c = m["contact"]["left_foot"]
    tp = int(np.sum(est & gt))
    fp = int(np.sum(est & ~gt))
    fn = int(np.sum(~est & gt))
    assert c["estimate_all"]["precision"] == pytest.approx(tp / (tp + fp), abs=TOL)
    assert c["estimate_all"]["recall"] == pytest.approx(tp / (tp + fn), abs=TOL)
    # Far from the edges (±(60 ms + 5 ticks) = ±35 ticks): only the two single ticks remain.
    # Inclusive in time: |t − t_edge| ≤ 60 ms + |t_planned − t_edge| (5 ticks).
    t = cols["t_relative_s"]
    far = np.ones(n, dtype=bool)
    for e, ep in ((200, 195), (300, 295), (600, 595), (700, 695)):
        far &= ~(np.abs(t - t[e]) <= 0.06 + abs(t[ep] - t[e]))
    assert 60 <= np.sum(~far) <= 4 * 71
    tp_f = int(np.sum(est & gt & far))
    fp_f = int(np.sum(est & ~gt & far))
    fn_f = int(np.sum(~est & gt & far))
    assert c["estimate_far_from_edges"]["precision"] == pytest.approx(
        tp_f / (tp_f + fp_f), abs=TOL
    )
    assert c["estimate_far_from_edges"]["recall"] == pytest.approx(tp_f / (tp_f + fn_f), abs=TOL)
    assert c["planned_all"]["recall"] == pytest.approx(np.sum(planned & gt) / gt.sum(), abs=TOL)
    assert c["touchdown_latency_s"]["median"] == pytest.approx(7 * DT, abs=TOL)
    assert c["liftoff_latency_s"]["median"] == pytest.approx(3 * DT, abs=TOL)


def test_a_slip_while_the_foot_is_still_on_a_planned_truth_contact_is_a_false_positive():
    n, cols, gt, planned = contact_case()
    cols["est_mask"] = np.where(gt, 3.0, 2.0)
    mode = np.where(gt, 2.0, 0.0)
    mode[400:410] = 3  # slip, foot still
    mode[410:415] = 4  # unknown, foot still
    cols["mode_left_foot"] = mode
    cols["truth_v_left_foot_x"][800:900] = 0.05  # moving: not a still tick
    mode[850] = 3
    m = em.run_metrics(cols, META)
    c = m["contact"]["left_foot"]
    still = gt & planned
    still[800:900] = False
    assert c["still_ticks"] == int(still.sum())
    assert c["slip_false_positive"] == pytest.approx(10 / still.sum(), abs=TOL)
    assert c["out_of_stance_when_still"] == pytest.approx(15 / still.sum(), abs=TOL)


def test_a_truth_contact_shorter_than_the_dwell_is_ignored():
    t = np.arange(100) * DT
    fz = np.full(100, 150.0)
    fz[50:55] = 0.0  # 10 ms of lost force
    assert em.truth_contact(fz, t, 300.0).all()
    fz[50:65] = 0.0  # 30 ms
    assert not em.truth_contact(fz, t, 300.0)[50:65].any()


# ── accelerometer, online comparison ───────────────────────────────────────


def test_the_accelerometer_check_is_zero_on_the_trajectory_it_was_made_from():
    n = 200
    t = np.arange(n) * DT
    acc = np.array([0.3, -0.2, 0.1])
    rots = np.array([rz(0.1) for _ in t])
    pos = np.outer(0.5 * t**2, acc)
    v_w = np.outer(t, acc)
    v_b = np.einsum("nji,nj->ni", rots, v_w)
    w_b = np.zeros((n, 3))
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    put_pose(cols, "base_", rots, pos)
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, v_w)
    f_i = (rots[0] @ R_BI).T @ (acc - np.array([0.0, 0.0, -9.81]))
    put_vec(cols, "in_imu_a", np.tile(f_i, (n, 1)))
    m = em.run_metrics(cols, META)
    a = m["accel_check"]
    assert a["ticks"] == n - 2
    assert max(abs(x) for x in a["residual_rms"]) < 1e-9


def test_the_online_comparison_stops_at_the_first_divergence_or_gap():
    n = 50
    cols = base_table(n)
    rots = np.array([rz(0.0) for _ in range(n)])
    pos = np.zeros((n, 3))
    put_pose(cols, "base_", rots, pos)
    put_pose(cols, "online_base_", rots, pos)
    put_vec(cols, "base_v", np.zeros((n, 3)))
    put_vec(cols, "online_base_v", np.zeros((n, 3)))
    for f in em.DISCRETE:
        cols[f"online_{f}"] = cols[f].copy() if f in cols else np.zeros(n)
        cols.setdefault(f, cols[f"online_{f}"].copy())
    for c in NAMES:
        for f in ("mode", "update"):
            cols[f"online_{f}_{c}"] = cols[f"{f}_{c}"].copy()
    cols["base_px"][:30] = 5e-4  # under the bound before the divergence
    cols["base_px"][30:] = 1.0  # past it, not judged
    cols["mode_right_foot"][30] = 3
    out = em.compare_online(cols, META)
    assert out["first_divergence"]["row"] == 30
    assert out["first_divergence"]["fields"] == ["mode_right_foot"]
    assert out["max_pos_m"] == pytest.approx(5e-4, abs=TOL)
    assert out["within_bounds"] is True
    cols["gap"][20] = 3
    out = em.compare_online(cols, META)
    assert out["compared_rows"] == 20
    assert out["stop"] == "gap"
    assert out["covers_scored_rows"] is False  # the run is scored to its end


def test_the_online_comparison_needs_to_cover_only_the_scored_rows():
    n = 50
    cols = base_table(n)
    rots = np.array([rz(0.0) for _ in range(n)])
    for prefix in ("base_", "online_base_"):
        put_pose(cols, prefix, rots, np.zeros((n, 3)))
    put_vec(cols, "base_v", np.zeros((n, 3)))
    put_vec(cols, "online_base_v", np.zeros((n, 3)))
    for f in em.DISCRETE:
        cols[f"online_{f}"] = cols[f].copy() if f in cols else np.zeros(n)
        cols.setdefault(f, cols[f"online_{f}"].copy())
    for c in NAMES:
        for f in ("mode", "update"):
            cols[f"online_{f}_{c}"] = cols[f"{f}_{c}"].copy()
    cols["mode_left_foot"][40] = 3  # a discrete divergence at row 40
    cols["online_fault_latched"] = np.zeros(n)
    assert em.compare_online(cols, META)["covers_scored_rows"] is False
    cols["online_fault_latched"][30:] = 1.0  # cut at 30: the divergence is after it
    out = em.compare_online(cols, META)
    assert out["stop"] == "diverged" and out["covers_scored_rows"] is True


# ── walking / standing parts, gyro bias ────────────────────────────────────


def exact_run(n):
    """A run whose estimate is the truth (any metric not under test reads 0)."""
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    put_pose(cols, "base_", rots, pos)
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, np.einsum("nij,nj->ni", rots, v_b + np.cross(w_b, P_BI)))
    return cols, rots, v_b, w_b


def test_the_nees_splits_at_the_first_planned_lift_off():
    n, lift = 400, 150
    cols, rots, v_b, w_b = exact_run(n)
    vel_wi = np.einsum("nij,nj->ni", rots, v_b + np.cross(w_b, P_BI))
    xi_phi, xi_v = np.array([0.01, -0.02, 0.0]), np.array([0.05, 0.02, -0.01])
    put_imu_state(cols, rots, cols_pos(cols), vel_wi, np.eye(3), xi_phi, xi_v)
    cols["planned_mask"][lift:] = np.where(np.arange(n - lift) % 100 < 50, 1.0, 3.0)
    cols["planned_mask"][lift + 100 :] = 2.0  # a later lift-off is not the split
    p = np.diag([4e-4, 1e-4, 9.0, 1e-3, 2e-3, 4e-3])
    for i in range(6):
        cols[f"P_{i}_{i}"] = np.full(n, p[i, i])
        cols[f"P_{i}_{i}"][lift:] *= 4.0  # the walking part's NEES is a quarter of the stand's
    m = em.run_metrics(cols, META)
    e = np.array([xi_phi[0], xi_phi[1], *xi_v])
    keep = [0, 1, 3, 4, 5]
    want = float(e @ np.linalg.solve(p[np.ix_(keep, keep)], e)) / 5
    assert m["lift_t_s"] == pytest.approx(lift * DT, abs=TOL)
    assert (m["nees_stand_ticks"], m["nees_walk_ticks"]) == (lift, n - lift)
    assert m["nees_stand_mean_per_dof_truth"] == pytest.approx(want, rel=1e-9)
    assert m["nees_walk_mean_per_dof_truth"] == pytest.approx(want / 4, rel=1e-9)
    cols["planned_mask"][:] = 3.0  # a stand run: no walking part
    m = em.run_metrics(cols, META)
    assert m["lift_t_s"] is None and m["nees_walk_mean_per_dof_truth"] is None
    assert m["nees_stand_ticks"] == n


def cols_pos(cols):
    return np.stack([cols[f"truth_p{a}"] for a in "xyz"], axis=1)


def test_the_gyro_bias_is_held_against_the_measured_offset_over_the_walking_part():
    n, lift, bias = 300, 100, 2e-3
    cols, rots, v_b, w_b = exact_run(n)
    w_imu = w_b @ R_BI  # R_BIᵀ ω_B per row
    for i, a in enumerate("xyz"):
        cols[f"in_imu_g{a}"] = w_imu[:, i] + (bias if a == "z" else 0.0)
        cols[f"bg_{a}"] = np.zeros(n)
    cols["in_imu_gz"][::2] += 1e-3  # zero-mean noise on the offset
    cols["in_imu_gz"][1::2] -= 1e-3
    cols["bg_z"][:] = bias
    cols["bg_z"][50] = bias + 0.5  # before the lift-off: not scored
    cols["bg_z"][200] = bias - 1e-3
    cols["planned_mask"][lift:] = 1.0
    m = em.run_metrics(cols, META)
    assert m["gyro_bias_z_truth"] == pytest.approx(bias, abs=TOL)
    assert m["gyro_bias_z_dev_max_truth"] == pytest.approx(1e-3, abs=TOL)


def test_a_run_without_the_gyro_columns_has_no_bias_numbers():
    cols, *_ = exact_run(50)
    m = em.run_metrics(cols, META)
    assert m["gyro_bias_z_truth"] is None and m["gyro_bias_z_dev_max_truth"] is None


# ── resets after the start, slip cycling (WD-359) ──────────────────────────

# A grid of 1/512 s: the window (0.5 s = 256 rows) and the span (1 s = 512 rows) are exact.
SLIP_DT = 1.0 / 512


def slip_modes(n, entries):
    """Stance everywhere, slip on the rows in `entries` (each an entry: none adjacent)."""
    mode = np.full(n, float(em.MODE_STANCE))
    mode[list(entries)] = em.MODE_SLIP
    return mode


def cycling_entries(first_on, last_on, middle_step=20):
    """Entries whose count over the 256-row window is ≥ 12 on exactly [first_on, last_on].

    12 entries 2 rows apart end at `first_on`; the twelfth from last sits 255 rows before
    `last_on` (it leaves the window on the next row) and 11 follow it; between, a step of
    ≤ 21 rows keeps 12 inside every window.
    """
    head = [first_on - 2 * (11 - k) for k in range(12)]
    leave = last_on - 255
    middle = list(range(head[-1] + middle_step, leave, middle_step))
    tail = [leave + 2 * k for k in range(12)]
    rows = head + middle + tail
    assert all(b - a >= 2 for a, b in zip(rows, rows[1:], strict=False))
    assert all(b - a <= 21 for a, b in zip(rows, rows[1:], strict=False))
    return rows


def slip_table(n, left_entries, right_entries=()):
    return {
        "t_relative_s": np.arange(n) * SLIP_DT,
        "mode_left_foot": slip_modes(n, left_entries),
        "mode_right_foot": slip_modes(n, right_entries),
    }


def test_the_window_counts_the_entries_in_the_last_half_second():
    t = np.arange(600) * SLIP_DT
    mode = slip_modes(600, [10, 12, 300])
    mode[13] = em.MODE_SLIP  # 12 → 13 stays in slip: one entry, not two
    count = em.slip_entries(mode, t)
    assert count[9] == 0 and count[10] == 1 and count[12] == 2 and count[13] == 2
    assert count[10 + 255] == 2  # row 10 is 255 rows back: still inside (t − 0.5, t]
    assert count[10 + 256] == 1  # exactly 0.5 s back: out
    assert count[300] == 1  # rows 10 and 12 have left


def test_a_cycle_is_twelve_entries_held_for_one_second():
    n = 3000
    exact = slip_table(n, cycling_entries(600, 600 + 512))
    cyc = em.slip_cycle(exact, NAMES)
    left = cyc["feet"]["left_foot"]
    assert left["cycle_span_s"] == pytest.approx(1.0, abs=TOL)
    assert left["cycle"] is True and cyc["feet_cycling"] == 1
    assert cyc["feet"]["right_foot"] == {"window_max": 0, "cycle_span_s": 0.0,
                                         "report_span_s": 0.0, "cycle": False}  # fmt: skip
    short = em.slip_cycle(slip_table(n, cycling_entries(600, 600 + 507)), NAMES)
    assert short["feet"]["left_foot"]["cycle_span_s"] == pytest.approx(507 / 512, abs=TOL)
    assert short["feet"]["left_foot"]["cycle"] is False and short["feet_cycling"] == 0


def test_eleven_entries_in_every_window_is_no_cycle_however_long():
    n = 6000
    # One entry every 23.3 rows: a 256-row window holds 11 at most, 10 at least.
    rows = [int(round(100 + 23.3 * k)) for k in range(240)]
    cyc = em.slip_cycle(slip_table(n, rows), NAMES)
    left = cyc["feet"]["left_foot"]
    assert left["window_max"] == 11
    assert left["cycle_span_s"] == 0.0 and left["cycle"] is False
    assert left["report_span_s"] > 9.0  # ≥ 8 the whole stretch: reported, not judged


def slip_run(n, fault_row=None):
    """A run_metrics table whose left foot cycles from row 600 to the end."""
    cols, *_ = exact_run(n)
    cols["t_relative_s"] = np.arange(n) * SLIP_DT
    cols["mode_left_foot"] = slip_modes(n, range(600, n - 1, 2))
    cols["online_fault_latched"] = np.zeros(n)
    if fault_row is not None:
        cols["online_fault_latched"][fault_row:] = 1.0
    return cols


def test_the_slip_after_the_cut_is_not_counted():
    n = 3000
    assert em.run_metrics(slip_run(n), META)["slip_cycle"]["feet_cycling"] == 1
    cut = em.run_metrics(slip_run(n, fault_row=700), META)
    assert cut["cut"] == "fault"
    assert cut["slip_cycle"]["feet_cycling"] == 0
    assert cut["slip_cycle"]["feet"]["left_foot"]["window_max"] == 50  # rows 600 … 698


def test_resets_after_the_start_skip_the_activation_and_stop_at_the_cut():
    n = 600
    cols, *_ = exact_run(n)
    cols["initialized"][:20] = 0.0
    cols["reset_count"][:] = 1.0  # the activation reset
    m = em.run_metrics(cols, META)
    assert m["resets"] == 1 and m["resets_after_start"] == 0
    cols["reset_count"][300:] = 2.0
    cols["initialized"][300:310] = 0.0  # a reset restarts the estimate
    assert em.run_metrics(cols, META)["resets_after_start"] == 1
    cols["online_fault_latched"] = np.zeros(n)
    cols["online_fault_latched"][250:] = 1.0  # the reset after the fault is the fall's
    assert em.run_metrics(cols, META)["resets_after_start"] == 0
    cols["initialized"][:] = 0.0
    assert em.resets_after_start(cols) is None


def test_an_inekf_driven_run_reports_the_online_estimates_own_counts():
    n = 3000
    cols = slip_run(n)
    cols["online_initialized"] = np.ones(n)
    cols["online_reset_count"] = np.ones(n)
    cols["online_reset_count"][900:] = 3.0
    cols["online_mode_left_foot"] = np.full(n, float(em.MODE_STANCE))  # the online never slipped
    cols["online_mode_right_foot"] = slip_modes(n, range(1000, n - 1, 2))
    v = np.stack([cols[f"truth_v{a}"] for a in "xyz"], axis=1)
    v[-513:, 0] += 0.03  # the last second (both ends: 513 rows) off by 3 cm/s
    put_vec(cols, "online_base_v", v)
    m = em.run_metrics(cols, META)
    on = m["online_source"]
    assert on["resets_after_start"] == 2
    assert on["slip_cycle"]["feet"]["left_foot"]["cycle"] is False
    assert on["slip_cycle"]["feet"]["right_foot"]["cycle"] is True
    assert on["vel_err_rms_last_s_truth"] == pytest.approx(0.03, abs=1e-6)
    assert m["slip_cycle"]["feet"]["left_foot"]["cycle"] is True  # the replay's own
    cols["online_initialized"][:] = 0.0  # a truth-driven run has no online estimate
    assert "online_source" not in em.run_metrics(cols, META)


def test_the_k_run_verdict_has_the_walking_and_standing_parts():
    runs = [{"nees_walk_mean_per_dof_truth": x, "nees_stand_mean_per_dof_truth": 0.2,
             "slip_cycle": {"feet_cycling": int(x > 1.4)}} for x in (0.9, 1.1, 1.3, 1.5)]  # fmt: skip
    agg = em.aggregate(runs)
    lo, hi = em.mean_band(20)
    assert agg["anees_walk_truth"]["value"] == pytest.approx(1.2, abs=TOL)
    assert agg["anees_walk_truth"]["pass"] is (lo <= 1.2 <= hi)
    stand = agg["anees_stand_truth"]
    assert stand["pass"] is False and stand["below_upper"] is True  # conservative, not failed
    assert agg["anees_walk_truth"]["below_upper"] is True  # inside the band: under its upper end
    assert agg["anees_truth"] is None
    assert agg["runs_cycling"] == 1


# ── height ─────────────────────────────────────────────────────────────────


def test_the_height_error_and_its_slope_over_the_stepping_window_only():
    n = 1001
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n, yaw_rate=0.0)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    rate = -1e-3  # m/s while stepping (rows 100 – 700), then a standing tail
    dz = np.zeros(n)
    dz[100:701] = rate * (t[100:701] - t[100])
    dz[701:] = dz[700]
    est = pos + np.stack([np.zeros(n), np.zeros(n), dz], axis=1)
    put_pose(cols, "base_", rots, est)
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, vel_w)
    planned = np.full(n, 3.0)
    planned[100:701:50] = 1.0  # a lift-off now and then; full stance after row 700
    cols["planned_mask"] = planned
    m = em.run_metrics(cols, META)
    assert m["z_err_mean_truth"] == pytest.approx(float(np.mean(dz)), abs=1e-12)
    assert m["z_err_rms_truth"] == pytest.approx(float(np.sqrt(np.mean(dz**2))), abs=1e-12)
    assert m["z_err_max_truth"] == pytest.approx(float(np.max(np.abs(dz))), abs=1e-12)
    # The slope of the stepping window (lift-off at row 100 to the last partial
    # stance at row 700) — the tail does not dilute it.
    assert m["z_drift_per_s_truth"] == pytest.approx(rate, rel=1e-6)
    whole = float(np.polyfit(t[100:], dz[100:], 1)[0])
    assert abs(whole) < 0.8 * abs(rate)  # what the tail would have made of it


def test_the_height_nees_adds_the_imu_heights_error_with_its_tilt_coupling():
    n = 40
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    dz = 1e-3 * np.arange(n)  # the estimate rises: truth − estimate = −dz
    put_pose(cols, "base_", rots, pos + np.stack([np.zeros(n), np.zeros(n), dz], axis=1))
    put_vec(cols, "base_v", v_b)
    vel_wi = np.einsum("nij,nj->ni", rots, v_b + np.cross(w_b, P_BI))
    xi_phi = np.array([0.01, -0.02, 0.0])
    xi_v = np.array([0.05, 0.02, -0.01])
    put_imu_state(cols, rots, pos, vel_wi, np.eye(3), xi_phi, xi_v)
    p = np.diag([4e-4, 1e-4, 9.0, 1e-3, 2e-3, 4e-3, 1.0, 1.0, 2e-4])
    p[0, 8] = p[8, 0] = 2e-6  # tilt x ↔ height of the IMU point
    p[4, 8] = p[8, 4] = 1e-5
    for i in range(6):
        for j in range(i, 6):
            cols[f"P_{i}_{j}"] = np.full(n, p[i, j])
    for i in range(9):
        for j in range(max(6, i), 9):
            cols[f"Pp_{i}_{j}"] = np.full(n, p[i, j])
    cols["planned_mask"][5] = 1.0  # walking from row 5
    m = em.run_metrics(cols, META)
    want = []
    for k in range(n):
        ph = np.array([cols["imu_px"][k], cols["imu_py"][k], cols["imu_pz"][k]])
        h = np.zeros(9)
        h[0], h[1], h[8] = ph[1], -ph[0], 1.0
        sel = np.zeros((6, 9))
        for row, col in enumerate((0, 1, 3, 4, 5)):
            sel[row, col] = 1.0
        sel[5] = h
        e = np.array([xi_phi[0], xi_phi[1], *xi_v, -(dz[k] - dz[0])])
        want.append(float(e @ np.linalg.solve(sel @ p @ sel.T, e)))
    assert m["nees_z_mean_per_dof_truth"] == pytest.approx(np.mean(want) / 6, rel=1e-6)
    assert m["nees_z_walk_mean_per_dof_truth"] == pytest.approx(np.mean(want[5:]) / 6, rel=1e-6)
    # A replay without the position columns has no height NEES.
    for key in [k for k in cols if k.startswith("Pp_")]:
        del cols[key]
    assert em.run_metrics(cols, META)["nees_z_mean_per_dof_truth"] is None


def test_the_pins_are_counted_and_their_nis_pooled_against_the_band():
    n = 20
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    put_pose(cols, "base_", rots, pos)
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, vel_w)
    for c in NAMES:
        cols[f"ground_{c}"] = np.zeros(n)
        cols[f"ground_nis_{c}"] = np.zeros(n)
        cols[f"ground_nu_{c}"] = np.zeros(n)
    cols["ground_left_foot"][[2, 9]] = em.GROUND_USED
    cols["ground_nis_left_foot"][[2, 9]] = [0.5, 1.5]
    cols["ground_right_foot"][4] = em.GROUND_GATED
    cols["ground_nis_right_foot"][4] = 12.0
    cols["ground_right_foot"][6] = em.GROUND_INFLATED  # re-acquisition: not a χ² sample
    cols["ground_nis_right_foot"][6] = 40.0
    m = em.run_metrics(cols, META)
    assert m["ground_pins"] == {"used": 2, "gated": 1, "inflated": 1}
    assert m["ground_nis_count"] == 3
    assert m["ground_nis_mean"] == pytest.approx((0.5 + 1.5 + 12.0) / 3)
    agg = em.aggregate([m, dict(m)])
    assert agg["ground_nis"]["k"] == 6
    lo, hi = em.mean_band(6)
    assert agg["ground_nis"]["band"] == [lo, hi]
    assert agg["ground_nis"]["pass"] is bool(lo <= m["ground_nis_mean"] <= hi)
    # A replay without the pin columns reports none.
    assert em.run_metrics(base_table_with_truth(), META)["ground_pins"] is None


def base_table_with_truth():
    n = 10
    _, rots, pos, vel_w, v_b, w_b = walking_truth(n)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    put_pose(cols, "base_", rots, pos)
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, vel_w)
    return cols


def test_the_height_slope_is_fitted_per_epoch():
    # Two epochs drifting at the same rate: each starts the error over at 0, so a
    # single fit across the reset would see a sawtooth, not the rate.
    n = 1001
    t, rots, pos, vel_w, v_b, w_b = walking_truth(n, yaw_rate=0.0)
    cols = base_table(n)
    put_truth(cols, rots, pos, v_b, w_b)
    rate = -2e-3
    cols["state_epoch"][500:] = 2.0
    k0 = np.where(np.arange(n) < 500, 0, 500)
    dz = rate * (t - t[k0])
    put_pose(cols, "base_", rots, pos + np.stack([np.zeros(n), np.zeros(n), dz], axis=1))
    put_vec(cols, "base_v", v_b)
    put_imu_state(cols, rots, pos, vel_w)
    planned = np.full(n, 3.0)
    planned[10:1000:40] = 1.0
    cols["planned_mask"] = planned
    m = em.run_metrics(cols, META)
    assert m["z_drift_per_s_truth"] == pytest.approx(rate, rel=1e-6)
    assert m["ground_nis_count"] == 0  # no pin columns: the count is there, 0
