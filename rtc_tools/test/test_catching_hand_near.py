"""catching_hand_near (dynamic_catching S8-F) — synthetic positive controls.

Trials are generated from KNOWN logistic laws — a commit probability falling
with speed and a catch-given-commit probability falling with speed and offset —
and the analyser must recover the planted v50 values from 600 draws to within
±0.2 m/s (the tolerance the S8-F spec asks of the positive control). The
model-free tables are checked against direct counts; the McNemar pairing
against hand-built discordant counts; the loader against a unit written the
way the runner and catching_trials write theirs.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
import pytest

from rtc_tools.analysis import catching_hand_near as hn

FACTOR_BOX = {
    "speed_m_s": (3.5, 7.0),
    "flight_time_s": (0.65, 0.8),
    "offset_m": (0.0, 0.2),
    "offset_angle_deg": (0.0, 360.0),
    "incidence_offset_deg": (-10.0, 15.0),
}
# Planted laws (logits). v50 at the design centre: plan 6.0 at r 0 (−0.6 per
# 0.1 m of offset... no: slope in r below), catch|plan 4.5 at r 0.
BETA_PLAN = {
    "intercept": 12.0,
    "v": -2.0,
    "r": -10.0,
}  # v50_plan(r) = (12 − 10 r) / 2 → 6.0 at r 0, 5.0 at r 0.2
BETA_CATCH = {
    "intercept": 13.5,
    "v": -3.0,
    "r": -15.0,
}  # v50_catch(r) = (13.5 − 15 r) / 3 → 4.5 at r 0, 3.5 at r 0.2


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def synthetic_trials(
    n: int, seed: int, *, arm: str = "A", kind: str = "hand_lhs", flip: float = 0.0
) -> list[hn.Trial]:
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        f = {k: float(rng.uniform(*box)) for k, box in FACTOR_BOX.items()}
        v, r = f["speed_m_s"], f["offset_m"]
        p_plan = _sigmoid(BETA_PLAN["intercept"] + BETA_PLAN["v"] * v + BETA_PLAN["r"] * r)
        committed = bool(rng.random() < p_plan)
        p_catch = (
            _sigmoid(BETA_CATCH["intercept"] + BETA_CATCH["v"] * v + BETA_CATCH["r"] * r)
            if committed
            else 0.0
        )
        caught = bool(rng.random() < p_catch)
        if flip and rng.random() < flip:
            caught = not caught
        v_rel = 0.4 * v + rng.normal(0.0, 0.2) if committed else math.nan
        out.append(
            hn.Trial(
                unit="synthetic",
                arm=arm,
                kind=kind,
                idx=i,
                seed=seed,
                sample_idx=i,
                factors=f,
                derived={},
                accepted=True,
                invalid_reason="",
                committed=committed,
                truth_success=caught,
                first_plan_s=0.2,
                plan_valid_ratio=0.15,
                contact_v_rel=v_rel,
                gamma_f_planned=0.6,
                ball_speed_tc=v,
                d_min_mm=20.0,
                rtf_trial_min=1.0,
                aim_error_mm=0.5,
                aim_pass_speed_m_s=v,
            )
        )
    return out


def test_wilson_matches_the_known_interval():
    p, lo, hi = hn.wilson(93, 200)
    assert p == pytest.approx(0.465)
    # 95 % two-sided Wilson for 93/200: [0.397, 0.534] (the S8-E G8-D numbers).
    assert lo == pytest.approx(0.3972, abs=5e-4) and hi == pytest.approx(0.5343, abs=5e-4)
    assert all(math.isnan(x) for x in hn.wilson(0, 0))


def test_the_positive_control_recovers_the_planted_v50():
    trials = synthetic_trials(600, 1)
    result = hn.two_stage(trials, n_boot=60, seed=0)
    plan = result["plan"]["v50_by_offset_m"]
    catch = result["catch_given_plan"]["v50_by_offset_m"]
    assert plan["0.00"] == pytest.approx(6.0, abs=0.2)
    assert plan["0.20"] == pytest.approx(5.0, abs=0.25)
    assert catch["0.00"] == pytest.approx(4.5, abs=0.2)
    assert catch["0.20"] == pytest.approx(3.5, abs=0.3)
    # The product crosses 0.5 below the catch v50 (P(plan) < 1 there).
    prod = result["product_v50_by_offset_m"]["0.00"]
    assert prod < catch["0.00"] and prod > 3.0
    lo, hi, n_ok = result["plan"]["v50_ci95_by_offset_m"]["0.00"]
    assert n_ok >= 20 and lo < 6.0 < hi and hi - lo < 1.5


def test_a_flipped_outcome_moves_the_estimate_and_the_control_catches_it():
    """Negative control: with 30 % of catch verdicts flipped the catch law is
    no longer the planted one — the estimate must move out of tolerance, or
    the positive control above proves nothing."""
    trials = synthetic_trials(600, 1, flip=0.3)
    result = hn.two_stage(trials, n_boot=0, seed=0)
    catch = result["catch_given_plan"]["v50_by_offset_m"]["0.00"]
    assert not math.isfinite(catch) or abs(catch - 4.5) > 0.2


def test_speed_table_counts_directly():
    trials = []
    for v, n, k_plan, k_catch in ((3.5, 8, 8, 6), (5.0, 8, 6, 2), (7.0, 8, 2, 0)):
        base = synthetic_trials(n, int(v * 10), kind="hand_cliff")
        for i, t in enumerate(base):
            t.factors["speed_m_s"] = v
            t.committed = i < k_plan
            t.truth_success = i < k_catch
        trials += base
    table = hn.speed_table(trials)
    assert [r["speed_m_s"] for r in table] == [3.5, 5.0, 7.0]
    assert [(r["n"], r["committed"], r["caught"]) for r in table] == [
        (8, 8, 6),
        (8, 6, 2),
        (8, 2, 0),
    ]
    assert table[0]["p_catch"] == pytest.approx(0.75) and table[1][
        "p_catch_given_plan"
    ] == pytest.approx(2 / 6)
    assert math.isnan(hn.speed_table([])[0]["p_catch"]) if hn.speed_table([]) else True
    g = hn.grid_v50(trials, n_boot=30, seed=0)
    assert 3.5 < g["catch"]["v50"] < 7.0 and g["plan"]["v50"] > g["catch"]["v50"]


def test_invalid_trials_are_excluded_from_every_table():
    trials = synthetic_trials(50, 5, kind="hand_cliff")
    for t in trials[:10]:
        t.invalid_reason = "srv_refused"
    assert sum(r["n"] for r in hn.speed_table(trials)) == 40
    assert hn.two_stage(trials, n_boot=0, seed=0)["n_valid"] == 40
    assert sum(c["n"] for c in hn.wilson_cells(trials)) == 40


def test_mcnemar_pairs_by_seed_and_sample_and_counts_discordance():
    a = synthetic_trials(40, 3, arm="A")
    b = synthetic_trials(40, 3, arm="B")  # same draws → identical outcomes
    for t in b[:6]:
        t.truth_success = not t.truth_success  # six discordant pairs, all B-flipped
    res = hn.paired_arms(a, b)
    assert res["pairs"] == 40
    ts = res["truth_success"]
    assert ts["a_only"] + ts["b_only"] == 6
    assert ts["mcnemar_p"] == pytest.approx(hn.mcnemar_exact(ts["a_only"], ts["b_only"]))
    assert hn.mcnemar_exact(0, 0) == 1.0
    assert hn.mcnemar_exact(0, 10) == pytest.approx(2 * 0.5**10)
    # An unpaired throw is reported, not silently paired.
    b.append(synthetic_trials(1, 99, arm="B")[0])
    assert hn.paired_arms(a, b)["unpaired_b"] == 1


def test_mcnemar_pairs_within_a_kind_so_same_seed_designs_do_not_collide():
    # Every design restarts sample_idx at 0: a cliff and an lhs unit of the
    # same seed share (seed, sample_idx) and must NOT be paired or overwrite
    # each other (PR #583 review).
    a = synthetic_trials(20, 5, arm="A", kind="hand_cliff") + synthetic_trials(
        30, 5, arm="A", kind="hand_lhs"
    )
    b = synthetic_trials(20, 5, arm="B", kind="hand_cliff") + synthetic_trials(
        30, 5, arm="B", kind="hand_lhs"
    )
    res = hn.paired_arms(a, b)
    assert res["pairs"] == 50 and res["unpaired_a"] == 0 and res["unpaired_b"] == 0
    assert res["truth_success"]["a_only"] == 0 and res["truth_success"]["b_only"] == 0
    # Arms of different composition pair only the kind they share.
    res = hn.paired_arms(a, b[20:])
    assert res["pairs"] == 30 and res["unpaired_a"] == 20


def test_fit_logistic_survives_a_perfectly_separated_cliff():
    v = np.linspace(3.5, 7.0, 40)
    y = (v < 5.0).astype(float)
    beta = hn.fit_logistic(np.column_stack([np.ones_like(v), v]), y)
    assert beta is not None and beta[1] < 0
    assert -beta[0] / beta[1] == pytest.approx(5.0, abs=0.15)
    assert hn.fit_logistic(np.column_stack([np.ones_like(v), v]), np.ones_like(v)) is None


def test_aim_check_flags_the_trials_over_tolerance():
    trials = synthetic_trials(5, 2)
    trials[3].aim_error_mm = 2.4
    res = hn.aim_check(trials, 2.0)
    assert res["over_tol_idx"] == [3] and res["pass"] is False and res["max_mm"] == 2.4
    assert hn.aim_check(trials[:3], 2.0)["pass"] is True
    # A model that flew a different ball than the sim shows up in model_rms,
    # not in aim_error (which is computed with the aiming parameters).
    trials[1].model_rms_mm = 12.0
    res = hn.aim_check(trials[:3], 2.0)
    assert res["pass"] is False and res["model_rms_over_tol_idx"] == [1]
    assert res["model_rms_max_mm"] == 12.0 and res["over_tol_idx"] == []
    # No aim data at all is "no data", not a miss.
    for t in trials:
        t.aim_error_mm = math.nan
        t.model_rms_mm = math.nan
    res = hn.aim_check(trials, 2.0)
    assert res["pass"] is None and res["n"] == 0


def _write_unit(
    tmp_path: Path, arm: str, trials: list[hn.Trial], planner: bool = True
) -> tuple[Path, Path | None]:
    unit = tmp_path / arm
    (unit / "trials").mkdir(parents=True)
    (unit / "ct").mkdir()
    records = []
    for t in trials:
        records.append(
            {
                "idx": t.idx,
                "kind": t.kind,
                "seed": t.seed,
                "sample_idx": t.sample_idx,
                "accepted": True,
                **t.factors,
                "target_m": [0.0, 0.0, 0.0],
                "incidence_deg": 43.6,
                "aim_error_m": t.aim_error_mm / 1e3,
                "aim_pass_speed_m_s": t.aim_pass_speed_m_s,
                "model_rms_m": t.model_rms_mm / 1e3,
                **t.derived,
            }
        )
    (unit / "trials" / "trial_results.json").write_text(json.dumps(records))
    (unit / "trials" / "run_meta.json").write_text(
        json.dumps({"arm": arm, "hand_geometry": {"p_c_m": [0, 0, 0]}})
    )
    with (unit / "ct" / "catching_trials.csv").open("w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "idx",
                "invalid_reason",
                "t_commit",
                "truth_success",
                "first_plan_s",
                "plan_valid_ratio",
                "contact_v_rel",
                "gamma_f_planned",
                "ball_speed_tc",
                "d_min_mm",
                "rtf_trial_min",
            ],
        )
        w.writeheader()
        for t in trials:
            w.writerow(
                {
                    "idx": t.idx,
                    "invalid_reason": t.invalid_reason,
                    "t_commit": "1.5" if t.committed else "",
                    "truth_success": str(t.truth_success),
                    "first_plan_s": t.first_plan_s,
                    "plan_valid_ratio": t.plan_valid_ratio,
                    "contact_v_rel": "" if math.isnan(t.contact_v_rel) else t.contact_v_rel,
                    "gamma_f_planned": t.gamma_f_planned,
                    "ball_speed_tc": t.ball_speed_tc,
                    "d_min_mm": t.d_min_mm,
                    "rtf_trial_min": t.rtf_trial_min,
                }
            )
    events = None
    if planner:
        events = unit / "planner_events.csv"
        with events.open("w", newline="") as f:
            w = csv.DictWriter(
                f, fieldnames=["plan_valid", "rank_reach", "rank_gamma", "rej_workspace", "rej_ik"]
            )
            w.writeheader()
            w.writerow(
                {
                    "plan_valid": "1",
                    "rank_reach": "1",
                    "rank_gamma": "0",
                    "rej_workspace": "3",
                    "rej_ik": "0",
                }
            )
            w.writerow(
                {
                    "plan_valid": "1",
                    "rank_reach": "0",
                    "rank_gamma": "0",
                    "rej_workspace": "0",
                    "rej_ik": "1",
                }
            )
            w.writerow(
                {
                    "plan_valid": "0",
                    "rank_reach": "0",
                    "rank_gamma": "0",
                    "rej_workspace": "5",
                    "rej_ik": "0",
                }
            )
    return unit, events


def test_a_unit_round_trips_through_the_loader_and_the_cli(tmp_path):
    a = synthetic_trials(60, 7, arm="reach_first")
    b = synthetic_trials(60, 7, arm="shipped_score")
    a[2].invalid_reason = "lane_drop"
    for t in a:
        t.model_rms_mm = 1.5
    # Records of an older runner lack a derived key a newer one writes; the
    # trials CSV must take the union of keys (PR #583 review).
    b[0].derived = {"apex_z_m": 1.2}
    unit_a, events = _write_unit(tmp_path, "reach_first", a)
    unit_b, _ = _write_unit(tmp_path, "shipped_score", b, planner=False)
    loaded = hn.load_unit(unit_a, events)
    assert loaded.arm == "reach_first" and len(loaded.trials) == 60
    assert loaded.trials[2].invalid_reason == "lane_drop" and not loaded.trials[2].valid
    assert (
        loaded.trials[0].committed == a[0].committed
        and loaded.trials[0].truth_success == a[0].truth_success
    )
    assert loaded.trials[0].factors == a[0].factors
    assert loaded.planner["plans_published"] == 2 and loaded.planner["rank_reach_rate"] == 0.5
    assert loaded.planner["judge_rejects"]["rej_workspace"] == 8
    out = tmp_path / "out"
    rc = hn.main(
        [
            f"{unit_a}:{events}",
            str(unit_b),
            "--out",
            str(out),
            "--n-boot",
            "10",
            "--ab",
            "reach_first",
            "shipped_score",
        ]
    )
    assert rc == 0
    summary = json.loads((out / "hand_near_summary.json").read_text())
    assert set(summary["arms"]) == {"reach_first", "shipped_score"}
    assert summary["aim"]["model_rms_max_mm"] == pytest.approx(1.5)
    assert summary["aim"]["pass"] is True
    assert summary["ab"]["pairs"] == 59  # the invalid trial drops out of the pairing
    assert (
        (out / "hand_near_trials.csv").is_file()
        and (out / "v50_map.csv").is_file()
        and (out / "wilson_cells.csv").is_file()
    )
    with (out / "hand_near_trials.csv").open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 120 and {r["arm"] for r in rows} == {"reach_first", "shipped_score"}
    assert "apex_z_m" in rows[0] and rows[0]["apex_z_m"] == ""  # union header, empty cell
    assert float(next(r for r in rows if r["arm"] == "shipped_score")["apex_z_m"]) == 1.2
    assert float(rows[0]["model_rms_mm"]) == pytest.approx(1.5)


def test_a_unit_of_non_hand_throws_is_refused(tmp_path):
    unit = tmp_path / "s35b"
    (unit / "trials").mkdir(parents=True)
    (unit / "ct").mkdir()
    (unit / "trials" / "trial_results.json").write_text(
        json.dumps([{"idx": 0, "kind": "s35b", "accepted": True}])
    )
    (unit / "ct" / "catching_trials.csv").write_text("idx\n0\n")
    with pytest.raises(SystemExit, match="hand-near"):
        hn.load_unit(unit)
