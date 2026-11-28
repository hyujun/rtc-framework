"""tools/standing_eval/estimator_gate.py (not installed — read from the source tree).

The rule's rounding, every baseline / standing / walking item failing on its
own, the verdict's precedence, the A/A size of the three walking tests, the
standing run's reader, and the constants the document states.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

_PATH = Path(__file__).resolve().parents[1] / "tools" / "standing_eval" / "estimator_gate.py"
_SPEC = importlib.util.spec_from_file_location("standing_eval_estimator_gate", _PATH)
gate = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gate)

DOC = Path(__file__).resolve().parents[1] / "docs" / "locomotion-controller.md"
FEET = ("left_foot", "right_foot")


def names(checks):
    return {c["name"] for c in checks if not c["pass"] and not c.get("record")}


# ── open loop ─────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("baseline", "want"),
    # The dry run of the rule on the WE-F05c replays (#1150 D5): the numbers it gave.
    [(0.00537, 0.009), (0.00749, 0.012), (0.00882, 0.014), (0.01685, 0.026), (0.00238, 0.004),
     (0.00093, 0.002), (0.006, 0.009), (0.004, 0.006)],
)  # fmt: skip
def test_a_threshold_is_the_baseline_times_the_scale_rounded_up(baseline, want):
    assert gate.ceil_step(gate.SCALE * baseline) == pytest.approx(want, abs=1e-12)


def replay(**over):
    """A replay's estimator_metrics numbers that pass every check."""
    m = {
        "file": over.pop("file", "run"),
        "tilt_rms_truth": 0.004,
        "yaw_rms_truth": 0.01,
        "vel_err_rms_truth": 0.005,
        "pos_drift_per_step_truth": 0.002,
        "gyro_bias_z_dev_max_truth": 1e-3,
        "slip_cycle": {"feet_cycling": 0},
        "contact": {
            c: {
                "estimate_far_from_edges": {"recall": 0.97, "precision": 1.0},
                "slip_false_positive": 0.001,
            }
            for c in FEET
        },  # fmt: skip
        "nees_mean_per_dof_truth": 0.9,
        "nees_walk_mean_per_dof_truth": 1.0,
        "nees_stand_mean_per_dof_truth": 0.2,
        "nis_mean_per_dof": 0.02,
        "online_source": {
            "resets_after_start": 0,
            "slip_cycle": {"feet_cycling": 0},
            "vel_err_rms_last_s_truth": 0.02,
        },  # fmt: skip
    }
    m.update(over)
    return m


def foot(m, part, key, value):
    m = copy.deepcopy(m)
    target = m["contact"]["right_foot"]
    (target[part] if part else target)[key] = value
    return m


RUN_BREAKS = {
    "cap_tilt_rms_truth": lambda m: {**m, "tilt_rms_truth": 0.051},
    "cap_yaw_rms_truth": lambda m: {**m, "yaw_rms_truth": 0.051},
    "cap_vel_err_rms_truth": lambda m: {**m, "vel_err_rms_truth": 0.051},
    "cap_pos_drift_per_step_truth": lambda m: {**m, "pos_drift_per_step_truth": 0.051},
    "slip_cycle_feet": lambda m: {**m, "slip_cycle": {"feet_cycling": 1}},
    "gyro_bias_z_dev_max": lambda m: {**m, "gyro_bias_z_dev_max_truth": 3.1e-3},
    "recall_far_min": lambda m: foot(m, "estimate_far_from_edges", "recall", 0.899),
    "precision_far_min": lambda m: foot(m, "estimate_far_from_edges", "precision", 0.989),
    "slip_false_positive_max": lambda m: foot(m, "", "slip_false_positive", 0.0051),
}


def test_a_good_run_passes_every_run_check():
    assert names(gate.run_checks(replay(), walking=True)) == set()
    assert names(gate.run_checks(replay(), walking=False)) == set()


@pytest.mark.parametrize("item", sorted(RUN_BREAKS))
def test_each_run_check_fails_on_its_own(item):
    assert names(gate.run_checks(RUN_BREAKS[item](replay()), walking=True)) == {item}


def test_a_standing_run_is_not_held_to_the_walking_items():
    bad = foot(replay(gyro_bias_z_dev_max_truth=1.0), "estimate_far_from_edges", "recall", 0.0)
    assert names(gate.run_checks(bad, walking=False)) == set()
    assert "cap_pos_drift_per_step_truth" not in {
        c["name"] for c in gate.run_checks(replay(pos_drift_per_step_truth=None), walking=False)
    }  # no touchdown: no drift per step


def baseline(walk_over=None, stand_over=None, k=36):
    walk = [replay(file=f"w{i}", **(walk_over or {})) for i in range(k)]
    stand_kw = {"nees_mean_per_dof_truth": 0.2, **(stand_over or {})}
    stand = [replay(file=f"s{i}", **stand_kw) for i in range(5)]
    return walk, stand


def test_the_baseline_passes_the_thresholds_derived_from_it():
    walk, stand = baseline()
    d = gate.derive_thresholds(walk, stand)
    assert d["thresholds"]["walk_vel_rms_median"] == pytest.approx(0.008, abs=1e-12)
    assert d["thresholds"]["stand_vel_rms_max"] == pytest.approx(0.008, abs=1e-12)
    out = gate.baseline_checks(walk, stand, d["thresholds"])
    assert out["failed"] == [] and out["anees_walk_in_band"] is True


def test_a_walking_anees_outside_its_band_is_recorded_not_failed():
    walk, stand = baseline({"nees_walk_mean_per_dof_truth": 1.4})
    out = gate.baseline_checks(walk, stand, gate.derive_thresholds(walk, stand)["thresholds"])
    assert out["anees_walk_in_band"] is False and out["failed"] == []
    rec = {c["name"]: c for c in out["checks"]}["anees_walk_in_band_high"]
    assert rec["pass"] is False and rec["record"] is True


@pytest.mark.parametrize(
    ("walk_over", "stand_over", "item"),
    [(None, {"nees_mean_per_dof_truth": 2.0}, "anees_stand_below_upper"),  # K 5: upper 1.65
     ({"nis_mean_per_dof": 1.3}, None, "anis_walk_below_upper"),
     (None, {"nis_mean_per_dof": 2.0}, "anis_stand_below_upper")],
)  # fmt: skip
def test_the_one_sided_consistency_checks_fail_above_the_upper_end(walk_over, stand_over, item):
    walk, stand = baseline(walk_over, stand_over)
    out = gate.baseline_checks(walk, stand, gate.derive_thresholds(*baseline())["thresholds"])
    assert set(out["failed"]) == {item}


@pytest.mark.parametrize("key", gate.THRESHOLD_KEYS)
def test_each_registered_threshold_fails_a_batch_above_it(key):
    walk, stand = baseline()
    thresholds = dict(gate.derive_thresholds(walk, stand)["thresholds"])
    thresholds[key] = 1e-6
    assert set(gate.baseline_checks(walk, stand, thresholds)["failed"]) == {key}


def test_without_thresholds_the_threshold_checks_are_absent():
    walk, stand = baseline()
    out = gate.baseline_checks(walk, stand, None)
    assert not {c["name"] for c in out["checks"]} & set(gate.THRESHOLD_KEYS)


# ── (b) standing ──────────────────────────────────────────────────────────

CONTROL = {"foot_slip_max": 1e-3, "base_tilt_max": 5e-3, "height_vs_plan_max": 1e-3,
           "base_pos_drift_max": 2e-3}  # fmt: skip


def stand_rec(**over):
    numbers = {"fallen": False, "fault_rows": 0, "source_not_ok_rows": 0, "source_stale_rows": 1,
               "qp_infeasible_rows": 0, "walk_rows": 0, "failed_solves": 0, "refused_epoch": 0,
               "refused_dim": 0, "stand_fraction": 0.99, "foot_slip_max": 1e-3,
               "base_tilt_max": 5e-3, "height_vs_plan_max": 1e-3, "base_pos_drift_max": 2e-3}  # fmt: skip
    rp = replay(vel_err_rms_truth=1e-3)
    on = {"within_bounds": True, "covers_scored_rows": True}
    for k, v in over.items():
        if k in numbers:
            numbers[k] = v
        elif k in on:
            on[k] = v
        elif k in ("resets_after_start", "slip_cycle"):
            rp["online_source"][k] = v
        else:
            rp[k] = v
    return {"run": "s", "numbers": numbers, "replay": rp, "online": on}


THRESHOLDS = dict.fromkeys(gate.THRESHOLD_KEYS, 0.002)

STAND_BREAKS = {
    "not_fallen": {"fallen": True},
    "fault_rows": {"fault_rows": 1},
    "source_not_ok_rows": {"source_not_ok_rows": 1},
    "qp_infeasible_rows": {"qp_infeasible_rows": 1},
    "walk_rows": {"walk_rows": 1},
    "failed_solves": {"failed_solves": 1},
    "refused_epoch": {"refused_epoch": 1},
    "refused_dim": {"refused_dim": 1},
    "resets_after_start": {"resets_after_start": 1},
    "stand_fraction": {"stand_fraction": 0.899},
    "foot_slip_max": {"foot_slip_max": 1e-3 + 1.6e-3},
    "base_tilt_max": {"base_tilt_max": 5e-3 + 0.021},
    "height_vs_plan_max": {"height_vs_plan_max": 1e-3 + 10.1e-3},
    "base_pos_drift_max": {"base_pos_drift_max": 2e-3 + 10.1e-3},
    "online_within_bounds": {"within_bounds": False},
    "online_covers_scored_rows": {"covers_scored_rows": False},
    "slip_cycle_feet": {"slip_cycle": {"feet_cycling": 1}},
    "vel_err_rms_truth": {"vel_err_rms_truth": 0.0021},
    "cap_tilt_rms_truth": {"tilt_rms_truth": 0.051},
}


def test_a_good_standing_run_passes_and_a_stale_lane_is_not_judged():
    assert names(gate.stand_checks(stand_rec(source_stale_rows=3), CONTROL, THRESHOLDS)) == set()


@pytest.mark.parametrize("item", sorted(STAND_BREAKS))
def test_each_standing_check_fails_on_its_own(item):
    assert names(gate.stand_checks(stand_rec(**STAND_BREAKS[item]), CONTROL, THRESHOLDS)) == {item}


@pytest.mark.parametrize(
    ("key", "bound"),
    [("foot_slip_max", 5e-3), ("base_tilt_max", 0.05), ("height_vs_plan_max", 20e-3),
     ("base_pos_drift_max", 20e-3)],
)  # fmt: skip
def test_the_standing_bounds_hold_whatever_the_control(key, bound):
    loose = dict.fromkeys(CONTROL, 1.0)
    assert names(gate.stand_checks(stand_rec(**{key: bound * 1.01}), loose, THRESHOLDS)) == {
        f"{key}_bound"
    }


def test_the_control_is_the_truth_runs_max_and_a_missing_value_fails_everything():
    truth = [stand_rec(foot_slip_max=x) for x in (1e-3, 3e-3, 2e-3)]
    assert gate.stand_control(truth)["foot_slip_max"] == 3e-3
    truth.append(stand_rec(height_vs_plan_max=None))
    assert math.isnan(gate.stand_control(truth)["height_vs_plan_max"])


# ── (c) walking ───────────────────────────────────────────────────────────


def walk_run(completed=True, survival=40.0, v=0.2, cause=None):
    return {"completed": completed, "survival_s": survival, "v_ratio_truth": v,
            "fault_cause": cause, "v_ratio_est": v, "lateral_mps_truth": 0.01,
            "lateral_mps_est": 0.01}  # fmt: skip


def test_a_collapse_of_completion_survival_or_tracking_is_rejected():
    truth = [walk_run() for _ in range(12)] + [walk_run(False, 8.0 + i) for i in range(12)]
    same = gate.walk_tests(truth, copy.deepcopy(truth), n_perm=999)
    assert not any(same[k]["reject"] for k in ("completion", "survival", "v_ratio_truth"))
    worse = [walk_run(False, 5.0 + 0.1 * i, v=0.1 + 0.001 * i) for i in range(24)]
    out = gate.walk_tests(truth, worse, n_perm=999)
    assert out["completion"]["reject"] and out["survival"]["reject"]
    assert out["v_ratio_truth"]["reject"]
    assert out["alpha_each"] == pytest.approx(gate.ALPHA / gate.TESTS)


def test_tracking_with_too_few_speed_windows_is_undecidable_not_rejected():
    truth = [walk_run(v=0.2 if i < gate.V_RATIO_MIN_RUNS - 1 else None) for i in range(24)]
    worse = [walk_run(v=0.05) for _ in range(24)]
    out = gate.walk_tests(truth, worse, n_perm=199)
    assert out["v_ratio_truth"]["undecidable"] is True and out["v_ratio_truth"]["reject"] is False


def test_the_three_tests_together_falsely_fail_at_most_alpha():
    """A/A: both arms drawn from one law shaped like the matched-plant baseline
    (completion ≈ 0.24, survival median ≈ 9 s censored at 40 s, ratio 0.2 ± 0.04)."""
    rng = np.random.default_rng(1150)
    trials, k = 200, 36
    false_fail = 0
    for trial in range(trials):
        arms = []
        for _ in range(2):
            runs = []
            for _ in range(k):
                done = rng.random() < 0.24
                runs.append(walk_run(done, 40.0 if done else float(rng.exponential(12.0)),
                                     float(rng.normal(0.2, 0.04)) if rng.random() < 0.6 else None))  # fmt: skip
            arms.append(runs)
        out = gate.walk_tests(*arms, n_perm=199, seed=trial)
        false_fail += any(out[t]["reject"] for t in ("completion", "survival", "v_ratio_truth"))
    slack = 3 * math.sqrt(gate.ALPHA * (1 - gate.ALPHA) / trials)
    assert false_fail / trials <= gate.ALPHA + slack


def inekf_rec(**over):
    m = walk_run()
    rp = replay()
    on = {"within_bounds": True, "covers_scored_rows": True}
    for k, v in over.items():
        if k in m:
            m[k] = v
        elif k in on:
            on[k] = v
        else:
            rp["online_source"][k] = v
    return {"run": "i", "metrics": m, "replay": rp, "online": on}


WALK_BREAKS = {
    "resets_after_start": {"resets_after_start": 1},
    "source_fault": {"fault_cause": "source"},
    "slip_cycle_feet": {"slip_cycle": {"feet_cycling": 1}},
    "online_within_bounds": {"within_bounds": False},
    "online_covers_scored_rows": {"covers_scored_rows": False},
    "v_ratio_agree": {"v_ratio_est": 0.2 + 0.0201},
    "lateral_agree_mps": {"lateral_mps_est": 0.01 - 0.0101},
}


def test_a_good_inekf_run_passes_and_another_fault_cause_is_not_the_estimators():
    assert names(gate.deterministic_checks(inekf_rec(fault_cause="plan loss"))) == set()


@pytest.mark.parametrize("item", sorted(WALK_BREAKS))
def test_each_deterministic_check_fails_on_its_own(item):
    assert names(gate.deterministic_checks(inekf_rec(**WALK_BREAKS[item]))) == {item}


def test_a_run_without_a_speed_window_has_no_agreement_checks():
    got = {c["name"] for c in gate.deterministic_checks(inekf_rec(v_ratio_truth=None))}
    assert not got & {"v_ratio_agree", "lateral_agree_mps"}


# ── the verdict ───────────────────────────────────────────────────────────


def batch(stand_over=None, walk_v=0.2, k=24):
    walk = {"arms": {
        "truth": [{"run": f"t{i}", "metrics": walk_run(v=walk_v), "replay": replay(file=f"t{i}"),
                   "online": None} for i in range(k)],
        "inekf": [{**inekf_rec(), "run": f"i{i}"} for i in range(k)],
    }, "walk_eval": {}}  # fmt: skip
    stand = {"truth": [stand_rec() for _ in range(5)],
             "inekf": [stand_rec(**(stand_over or {})) for _ in range(5)]}  # fmt: skip
    return {"walk": walk}, stand


def test_a_good_batch_passes():
    scripts, stand = batch()
    # A batch on the registered condition: its walking ANEES is the registered baseline's
    # (G1 is a same-condition regression bar, so a batch far from it is not "good").
    for r in scripts["walk"]["arms"]["truth"]:
        r["replay"]["nees_walk_mean_per_dof_truth"] = gate.ANEES_WALK_BASELINE
    thresholds = gate.derive_thresholds(
        [r["replay"] for r in scripts["walk"]["arms"]["truth"]],
        [r["replay"] for r in stand["truth"]],
    )["thresholds"]
    out = gate.judge(scripts, stand, thresholds, n_perm=199)
    assert out["verdict"] == "PASS", (out["open_loop"]["failed"], out["stand"]["failed"],
                                      out["walk"]["failed"])  # fmt: skip
    assert gate.judge(scripts, stand, None, n_perm=199)["verdict"].endswith(
        "(unregistered thresholds)"
    )


def test_a_failure_outranks_an_undecidable_test():
    scripts, stand = batch({"fault_rows": 1}, walk_v=None)
    out = gate.judge(scripts, stand, None, n_perm=199)
    assert out["walk"]["undecidable"] is True
    assert out["verdict"].startswith("FAIL")
    scripts, stand = batch(walk_v=None)
    assert gate.judge(scripts, stand, None, n_perm=199)["verdict"].startswith("UNDECIDABLE")


def test_a_batch_without_the_walk_script_or_runs_is_refused_or_failed():
    scripts, stand = batch()
    with pytest.raises(ValueError):
        gate.judge({"curve": scripts["walk"]}, stand, None, n_perm=19)
    stand["inekf"] = []
    assert "no standing runs" in gate.judge(scripts, stand, None, n_perm=19)["stand"]["failed"]


def test_arm_groups_are_the_name_before_the_dot():
    assert gate.group_of("truth.s2") == "truth" and gate.group_of("inekf") == "inekf"
    with pytest.raises(ValueError):
        gate.group_of("candidate")
    with pytest.raises(ValueError):
        gate._groups({"truth.s1": {}, "truth.s2": {}})


def test_g1_is_judged_only_with_the_registered_thresholds():
    scripts, stand = batch()
    for r in scripts["walk"]["arms"]["truth"]:
        r["replay"]["nees_walk_mean_per_dof_truth"] = 3.0  # above 1.5 × the registered baseline
    thresholds = gate.derive_thresholds(
        [r["replay"] for r in scripts["walk"]["arms"]["truth"]],
        [r["replay"] for r in stand["truth"]],
    )["thresholds"]
    out = gate.judge(scripts, stand, thresholds, n_perm=199)
    assert "anees_walk_g1" in out["open_loop"]["failed"] and out["verdict"] == "FAIL"
    assert out["open_loop"]["g1_anees_walk_max"] == gate.g1_bound(gate.ANEES_WALK_BASELINE)
    derived = gate.judge(scripts, stand, None, n_perm=199)  # a (re-)registration step
    assert "anees_walk_g1" not in derived["open_loop"]["failed"]
    assert derived["open_loop"]["g1_anees_walk_max"] == gate.g1_bound(3.0)


def test_the_judge_refuses_to_run_without_registered_thresholds(tmp_path, monkeypatch):
    spec = tmp_path / "spec.yaml"
    spec.write_text("scripts: {}\nstand: {arms: {}}\n")
    monkeypatch.setattr(gate, "REGISTERED", None)
    with pytest.raises(SystemExit):
        gate.main([str(tmp_path), "--spec", str(spec)])


# ── the standing run's reader ─────────────────────────────────────────────


def write_csv(path, cols):
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(cols)
    rows = zip(*(cols[k] for k in keys), strict=True)
    path.write_text(",".join(keys) + "\n" + "".join(",".join(map(str, r)) + "\n" for r in rows))


def test_the_standing_reader_splits_stale_lanes_and_reads_the_truth_height(tmp_path):
    n, dt = 15000, 0.002  # 30 s of log
    t = np.arange(n) * dt
    ref = np.full(n, 1)  # stand
    ref[:100] = 0  # idle before the controller took over: not counted
    ref[5000] = 2  # one Walk row
    status = np.zeros(n, dtype=int)
    status[50] = 4  # before the start: not counted
    status[6000] = 4  # base stale: a stall
    status[7000] = 10  # not initialized: the estimate's
    fault = np.zeros(n, dtype=int)
    qp = np.zeros(n)
    qp[8000] = 2
    ref_z = np.full(n, 0.75)
    truth_z = np.full(n, 0.75)
    truth_z[3000] = 0.70  # 5 cm low, but in the first 10 s: outside the last 20 s
    truth_z[12000] = 0.746  # 4 mm low, inside
    write_csv(tmp_path / walk_gate_diag(), {
        "tick": np.arange(n), "t_relative_s": t, "ref_source": ref, "source_status": status,
        "fault_cause": fault, "qp_status": qp, "ref_base_pz": ref_z,
    })  # fmt: skip
    write_csv(tmp_path / walk_gate_diag().with_name("locomotion_truth.csv"),
              {"tick": np.arange(n), "truth_base_pz": truth_z})  # fmt: skip
    (tmp_path / "stance.json").write_text(json.dumps(
        {"fallen": False, "foot_slip_max": 1e-3, "base_tilt_max": 4e-3, "base_pos_drift_max": 2e-3}
    ))  # fmt: skip
    (tmp_path / "launch.log").write_text(
        "[locomotion] mpc: solves 100 (failed 2, deadline miss 0, withheld 0, overrun 0), "
        "solve p50 10.0 ms p99 20.0 ms; refused: epoch 0 state 0 age 0 dim 1 implausible 0\n"
    )
    nb = gate.stand_numbers(tmp_path)
    assert nb["source_stale_rows"] == 1 and nb["source_not_ok_rows"] == 1
    assert nb["walk_rows"] == 1 and nb["qp_infeasible_rows"] == 1 and nb["fault_rows"] == 0
    assert nb["stand_fraction"] == pytest.approx(1 - 1 / (n - 100), abs=1e-12)
    assert nb["height_vs_plan_max"] == pytest.approx(0.004, abs=1e-9)
    assert (nb["failed_solves"], nb["refused_epoch"], nb["refused_dim"]) == (2, 0, 1)


def walk_gate_diag():
    return gate.walk_gate.DIAG


def test_the_registered_numbers_are_on_the_rule_grid():
    regs = gate.registered()
    assert set(regs) == set(gate.THRESHOLD_KEYS)
    for k, v in regs.items():
        assert v > 0 and math.isclose(v / gate.STEP, round(v / gate.STEP)), k
        assert v <= gate.ABS_CAP, k
    assert gate.ANEES_WALK_BASELINE is not None and gate.ANEES_WALK_BASELINE > 0


def test_g1_fails_a_walking_anees_above_the_registered_bound():
    walk, stand = baseline({"nees_walk_mean_per_dof_truth": 1.4})
    thresholds = gate.derive_thresholds(walk, stand)["thresholds"]
    names = [c["name"] for c in gate.baseline_checks(walk, stand, thresholds)["checks"]]
    assert "anees_walk_g1" not in names  # no registered baseline: no G1
    ok = gate.baseline_checks(walk, stand, thresholds, anees_walk_baseline=1.4 / 1.5)
    assert ok["failed"] == []  # at the bound
    out = gate.baseline_checks(walk, stand, thresholds, anees_walk_baseline=0.9)
    assert out["failed"] == ["anees_walk_g1"]
    assert gate.g1_bound(1.591) == 2.3865


def test_the_registered_walking_anees_baseline_was_outside_its_band():
    from rtc_tools.analysis import estimator_metrics as em

    # The registered baseline's 42 walking replays: G1 exists only for an out-of-band one.
    lo, hi = em.mean_band(42 * em.NEES_DOF)
    assert not lo <= gate.ANEES_WALK_BASELINE <= hi


def test_the_registered_numbers_match_only_their_own_baseline():
    regs = gate.registered()
    derived = {"thresholds": dict(regs), "anees_walk_baseline": gate.ANEES_WALK_BASELINE + 1e-4}
    assert gate.registered_matches(regs, derived)
    assert not gate.registered_matches(regs, {**derived, "anees_walk_baseline": None})
    off = gate.ANEES_WALK_BASELINE + 2 * gate.ANEES_WALK_BASELINE_TOL
    assert not gate.registered_matches(regs, {**derived, "anees_walk_baseline": off})
    for k in gate.THRESHOLD_KEYS:
        moved = {**derived, "thresholds": {**regs, k: regs[k] + gate.STEP}}
        assert not gate.registered_matches(regs, moved), k
        assert not gate.registered_matches(regs, {**derived, "thresholds": {**regs, k: None}}), k


# ── the document ──────────────────────────────────────────────────────────


def test_the_document_states_the_registered_constants():
    doc = DOC.read_text()
    from rtc_tools.analysis import estimator_metrics as em

    for token in (
        f"⌈{gate.SCALE:g} × baseline 통계⌉ ({gate.STEP:.0e} 단위)".replace("e-0", "e-"),
        f"ANEES ≤ {gate.ANEES_RATIO:g} × baseline",
        f"cap {gate.ABS_CAP:g}",
        f"gyro bias 오차 ≤ {gate.BIAS_Z_DEV_MAX:.0e} rad/s".replace("e-0", "e-"),
        f"recall ≥ {gate.RECALL_FAR_MIN:.2f}",
        f"precision ≥ {gate.PRECISION_FAR_MIN:.2f}",
        f"slip false positive ≤ {gate.SLIP_FP_MAX:g}",
        f"Stand 비율 ≥ {gate.STAND_FRACTION_MIN:g}",
        f"마지막 {gate.STAND_WINDOW_S:g} s",
        f"각 α {gate.ALPHA:g} / {gate.TESTS}",
        f"permutation {gate.N_PERM:,}".replace(",", " ") + " 회",
        f"arm 마다 {gate.V_RATIO_MIN_RUNS} 이상",
        f"|Δ추종비| ≤ {gate.V_RATIO_AGREE:g}",
        f"|Δ횡속도| ≤ {gate.LATERAL_AGREE_MPS:g} m/s",
        f"{em.SLIP_WINDOW_S:g} s 창의 slip 진입이 {em.SLIP_CYCLE_ENTRIES} 회 이상",
        f"{em.SLIP_CYCLE_S:.1f} s 이상",
        f"{em.SLIP_REPORT_ENTRIES} 회 이상의 길이는 보고",
        *(f"`{k}` {v:g}" for k, v in gate.registered().items()),
        f"보행 ANEES baseline {gate.ANEES_WALK_BASELINE:g}",
        f"뒤 G1 상한은 {gate.g1_bound(gate.ANEES_WALK_BASELINE):g}",
    ):
        assert token in doc, token


def test_each_test_is_held_at_alpha_over_three_not_alpha():
    # 8 / 24 against 2 / 24: one-sided Fisher p 0.036 — under α, over α / 3.
    truth = [walk_run(i < 8, 40.0 if i < 8 else 9.0) for i in range(24)]
    inekf = [walk_run(i < 2, 40.0 if i < 2 else 9.0) for i in range(24)]
    out = gate.walk_tests(truth, inekf, n_perm=199)
    assert gate.ALPHA / gate.TESTS < out["completion"]["p"] < gate.ALPHA
    assert out["completion"]["reject"] is False


def test_a_script_entry_is_scored_against_its_own_script(tmp_path, monkeypatch):
    seen = []
    monkeypatch.setattr(gate.we, "evaluate_batch",
                        lambda root, spec: seen.append(spec["script"]) or {"arms": {}})  # fmt: skip
    arms = {"truth": {"runs": "x"}, "inekf": {"runs": "y"}}
    gate.collect_scripts(tmp_path, {"scripts": {"curve": {"arms": arms}}})
    assert seen == ["curve"]
    with pytest.raises(ValueError):
        gate.collect_scripts(tmp_path, {"scripts": {"curve": {"script": "walk", "arms": arms}}})


def test_an_unreadable_standing_run_is_listed_not_fatal(tmp_path):
    (tmp_path / "stand_truth_1" / "mpc_0").mkdir(parents=True)  # no run.json
    (tmp_path / "stand_inekf_1" / "mpc_0").mkdir(parents=True)
    (tmp_path / "stand_inekf_1" / "mpc_0" / "run.json").write_text("{}")
    spec = {"arms": {"truth": {"runs": "stand_truth_*/mpc_0"},
                     "inekf": {"runs": "stand_inekf_*/mpc_0"}}}  # fmt: skip
    out, errors = gate.collect_stand(tmp_path, spec)
    assert out == {"truth": [], "inekf": []} and len(errors) == 2


def test_a_derive_run_exits_0_when_the_baseline_passes(tmp_path, monkeypatch):
    scripts, stand = batch()
    monkeypatch.setattr(gate, "collect_scripts", lambda root, spec: (scripts, []))
    monkeypatch.setattr(gate, "collect_stand", lambda root, spec: (stand, []))
    monkeypatch.setattr(gate, "N_PERM", 99)
    monkeypatch.setattr(gate, "judge", lambda s, st, t, n_perm=99, _j=gate.judge: _j(s, st, t, 99))
    spec = tmp_path / "spec.yaml"
    spec.write_text("scripts: {}\nstand: {arms: {}}\n")
    assert gate.main([str(tmp_path), "--spec", str(spec), "--derive"]) == 0
    assert json.loads((tmp_path / "estimator_gate.json").read_text())["verdict"].startswith("PASS")
