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
