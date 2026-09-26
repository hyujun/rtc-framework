"""catching_vision (dynamic_catching S8-E) — G8-B NEES binding and G8-C2 A/B, with positive controls.

Every check here has a defect it must catch: a post-contact sample with a huge
NEES that binding must drop (and a pre-launch one), NEES scaled away from 3
that must FAIL, a NaN share over 10 % that must not be judged, a planner point
that only matches once the model → world frame is applied, and a live snapshot
missing from the dump that must fall back to the approximate join and say so.
"""

from __future__ import annotations

import math

import numpy as np
import pandas as pd
import pytest

from rtc_tools.analysis import catching_vision as cv

T0 = 1_790_000_000_000_000_000  # a launch-anchored stamp [ns]
MS = 1_000_000


def _samples(rows):
    df = pd.DataFrame(
        rows,
        columns=[
            "record",
            "time_ns",
            "horizon_ns",
            "position_nees",
            "position_error_x_m",
            "position_error_y_m",
            "position_error_z_m",
        ],
    )
    df["time_ns"] = df["time_ns"].astype(np.int64)
    df["horizon_ns"] = df["horizon_ns"].astype(np.int64)
    return df.sort_values("time_ns").reset_index(drop=True)


def test_binding_keeps_launch_to_contact_and_drops_the_rest():
    hi = T0 + 400 * MS  # first contact
    rows = [
        # before the launch (another flight's tail) — must be dropped
        ("prediction", T0 - 50 * MS, 100 * MS, 1e6, 0.0, 0.0, 0.0),
        # at the launch instant — kept
        ("prediction", T0, 100 * MS, 2.0, 0.01, 0.0, 0.0),
        ("prediction", T0 + 100 * MS, 100 * MS, 4.0, 0.03, 0.0, 0.0),
        ("prediction", T0 + 300 * MS, 100 * MS, math.nan, 0.02, 0.0, 0.0),  # target == hi: kept
        # target after the contact: the evaluator compared it with the ball in the hand
        ("prediction", T0 + 350 * MS, 100 * MS, 1e6, 5.0, 5.0, 5.0),
        ("prediction", T0 + 100 * MS, 250 * MS, 3.0, 0.0, 0.0, 0.0),
    ]
    out = cv.nees_by_trial(_samples(rows), T0, hi)
    assert out["nees_h100ms_n"] == 3 and out["nees_h100ms_nan"] == 1
    assert out["nees_h100ms_mean"] == pytest.approx(3.0)  # (2 + 4) / 2 — no 1e6
    assert out["err_h100ms_x"] == pytest.approx(0.02)
    assert out["nees_h100ms_cov95"] == pytest.approx(1.0)
    assert out["nees_h250ms_n"] == 1 and out["nees_h250ms_mean"] == pytest.approx(3.0)


def test_binding_with_an_unknown_window_binds_nothing():
    rows = [("prediction", T0, 100 * MS, 2.0, 0.0, 0.0, 0.0)]
    out = cv.nees_by_trial(_samples(rows), T0, math.nan)
    assert out["nees_h100ms_n"] == 0 and math.isnan(out["nees_h100ms_mean"])


def _trial_rows(n_trials, per_trial, scale=1.0, nan_every=0, seed=0):
    """Per-trial columns as nees_by_trial writes them, NEES ~ scale·χ²₃."""
    rng = np.random.default_rng(seed)
    rows, allv = [], []
    for i in range(n_trials):
        v = scale * rng.chisquare(3, per_trial)
        n_nan = per_trial // nan_every if nan_every else 0
        finite = v[n_nan:]
        allv.extend(finite)
        rows.append(
            {
                "idx": i,
                "nees_h100ms_n": per_trial,
                "nees_h100ms_nan": n_nan,
                "nees_h100ms_mean": float(finite.mean()),
                "nees_h100ms_cov95": float(np.mean(finite <= cv.CHI2_3_95)),
                "err_h100ms_x": 0.001 * (i % 3),
                "err_h100ms_y": 0.0,
                "err_h100ms_z": -0.002,
            }
        )
    return rows, np.array(allv)


def test_g8b_consistent_nees_passes_and_the_pooled_mean_is_exact():
    rows, allv = _trial_rows(40, 30)
    # One trial with a different count: a mean of means would differ.
    rows[0]["nees_h100ms_n"] = 5
    rows[0]["nees_h100ms_mean"] = 50.0
    allv = np.concatenate([[50.0] * 5, allv[30:]])
    g = cv.g8b_summary(rows, n_boot=400, seed=1)["horizons"]["100"]
    assert g["mean_nees"] == pytest.approx(allv.mean())
    assert g["n_samples"] == 5 + 39 * 30 and g["n_trials"] == 40
    lo, hi = g["ci95"]
    assert lo <= 3.0 <= hi and g["verdict"] == "PASS"
    assert g["bias_m"][2] == pytest.approx(-0.002)


def test_g8b_inconsistent_nees_fails():
    rows, _ = _trial_rows(40, 30, scale=0.3)  # covariance 3× too large → NEES ≈ 1
    g = cv.g8b_summary(rows, n_boot=400, seed=1)["horizons"]["100"]
    assert g["verdict"] == "FAIL" and g["ci95"][1] < 3.0
    assert g["coverage_95"] > 0.99


def test_g8b_nan_share_over_ten_percent_is_not_evaluated():
    rows, _ = _trial_rows(20, 30, nan_every=5)  # 20 % NaN
    g = cv.g8b_summary(rows, n_boot=100, seed=1)["horizons"]["100"]
    assert g["nan_share"] == pytest.approx(0.2)
    assert g["verdict"] == "NOT_EVALUATED(NaN > 10 %)"


def test_g8b_without_bound_samples_says_so():
    assert cv.g8b_summary([{"idx": 0}], 10, 0).startswith("NOT_EVALUATED")
    g = cv.g8b_summary([{"nees_h500ms_n": 0, "nees_h500ms_nan": 0}], 10, 0)
    assert g["horizons"]["500"]["verdict"].startswith("NOT_EVALUATED(no sample")


# ── G8-C2 ────────────────────────────────────────────────────────────────────

G = np.array([0.0, 0.0, -9.81])


def _dump_csv(tmp_path, snapshots, subs=("best_effort", "reliable")):
    """Write a probe dump: snapshots = [(recv_ns, stamp_ns, seq, gen, p0, v0)], ballistic."""
    rows = []
    for recv, stamp, seq, gen, p0, v0 in snapshots:
        for i in range(20):
            h = (i + 1) * 50 * MS
            tau = h * 1e-9
            p = np.asarray(p0) + np.asarray(v0) * tau + 0.5 * G * tau * tau
            v = np.asarray(v0) + G * tau
            for sub in subs:
                rows.append(
                    [recv, sub, stamp, "world", 20, i, seq, gen, h, "VALID", *p, *v, *G]
                    + [0.0] * 36
                )
    cols = [
        "recv_ns",
        "sub",
        "stamp_ns",
        "frame_id",
        "n_points",
        "point_index",
        "snapshot_sequence",
        "generation",
        "horizon_ns",
        "validity",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "ax",
        "ay",
        "az",
    ] + [f"cov_{r}{c}" for r in range(6) for c in range(6)]
    path = tmp_path / "lane_prediction_dump.csv"
    pd.DataFrame(rows, columns=cols).to_csv(path, index=False)
    return path


def test_probe_dump_keeps_one_subscription_and_predicts_between_points(tmp_path):
    p0, v0 = [1.0, 0.0, 0.5], [-2.0, 0.1, 3.0]
    dump = cv.load_probe_dump(_dump_csv(tmp_path, [(100, T0, 7, 2, p0, v0)]))
    assert dump.sub == "reliable" and dump.frame_id == "world"
    arr, _ = dump.points[(7, 2)]
    assert len(arr) == 20  # not 40: the second subscription is the same messages
    t = T0 + 275 * MS  # between the 250 and 300 ms points
    tau = 0.275
    expect = np.asarray(p0) + np.asarray(v0) * tau + 0.5 * G * tau * tau
    assert np.allclose(cv.p_hat_at(dump, (7, 2), t), expect, atol=1e-9)
    assert np.all(np.isnan(cv.p_hat_at(dump, (7, 2), T0 + 5_000 * MS)))  # outside
    assert np.all(np.isnan(cv.p_hat_at(dump, (9, 9), t)))  # no such snapshot


def test_plan_point_matches_only_in_the_dump_frame(tmp_path):
    """p_c lives in the controller's model world; the dump in the sim world. A
    half-turn about z between them (a real robot's mounting) must be applied
    before the match — without it the planner's own point is metres away."""
    p0, v0 = [1.0, 0.0, 0.5], [-2.0, 0.1, 3.0]
    dump = cv.load_probe_dump(_dump_csv(tmp_path, [(100, T0, 7, 2, p0, v0)]))
    arr, t_int = dump.points[(7, 2)]
    p_world = arr[3, 1:4]  # the 200 ms point (x 0.6 m: the half-turn moves it 1.2 m)
    rz_pi = np.diag([-1.0, -1.0, 1.0])
    p_model = rz_pi @ p_world
    t_ns, d = cv.plan_point_stamp(dump, (7, 2), rz_pi.T @ p_model)
    assert t_ns == t_int[3] == T0 + 200 * MS and d < 1e-12
    t_bad, d_bad = cv.plan_point_stamp(dump, (7, 2), p_model)  # frame forgotten
    assert math.isnan(t_bad) and d_bad > 0.5


def test_last_snapshot_before_is_the_approximate_join(tmp_path):
    p0, v0 = [1.0, 0.0, 0.5], [-2.0, 0.1, 3.0]
    dump = cv.load_probe_dump(
        _dump_csv(tmp_path, [(100, T0, 7, 2, p0, v0), (200, T0 + 10 * MS, 8, 2, p0, v0)])
    )
    assert cv.last_snapshot_before(dump, 150) == (7, 2)
    assert cv.last_snapshot_before(dump, 200) == (8, 2)
    assert cv.last_snapshot_before(dump, 50) is None
    assert cv.last_snapshot_before(dump, math.nan) is None


def test_c2_summary_identity_and_the_n_gate():
    rng = np.random.default_rng(3)
    rows = []
    for i in range(120):
        a = rng.normal(0, 0.02, 3)
        b = rng.normal(0, 0.01, 3)
        rows.append(
            {
                **{f"c2_A_{x}": a[k] for k, x in enumerate("xyz")},
                **{f"c2_B_{x}": b[k] for k, x in enumerate("xyz")},
                "c2_join": "exact" if i % 10 else "approx",
            }
        )
    rows.append({"c2_A_x": math.nan, "c2_join": "none"})  # not counted
    s = cv.c2_summary(rows, n_boot=200, seed=0)
    assert (s["n"], s["n_exact"], s["n_approx"]) == (120, 108, 12)
    assert s["E_A_plus_B_sq_mm2"] == pytest.approx(
        s["E_A_sq_plus_E_B_sq_mm2"] + s["two_E_A_dot_B_mm2"]
    )
    assert s["independence"]["passed"] is True
    small = cv.c2_summary(rows[:50], n_boot=50, seed=0)
    assert small["independence"] == "NOT_EVALUATED(n < 100)"
    # B = −A (the prediction error fully explained by the plan's drift) is the
    # dependence A⊥B must reject.
    dep = [{**r, **{f"c2_B_{x}": -r[f"c2_A_{x}"] * 0.5 for x in "xyz"}} for r in rows[:120]]
    dep_s = cv.c2_summary(dep, n_boot=200, seed=0)
    assert dep_s["independence"]["passed"] is False


def test_eval_report_is_echoed_as_unrestricted(tmp_path):
    import json

    path = tmp_path / "eval_report.json"
    path.write_text(
        json.dumps(
            {
                "prediction": {
                    "horizons": [
                        {
                            "horizon_ns": 500 * MS,
                            "evaluated": 10,
                            "position_nees_coverage_95": 0.15,
                            "mean_position_error_m": [0, 0, -1.19],
                        }
                    ]
                }
            }
        )
    )
    out = cv.eval_report_horizons(path)
    assert "post-contact" in out["note"]
    assert out["500"]["mean_position_error_m"][2] == -1.19
