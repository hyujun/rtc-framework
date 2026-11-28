"""rtc_tools.analysis.walk_eval on synthetic runs.

The synthetic run follows the walk script at a known fraction of the commanded
speed with a known contact schedule, so every count and ratio is known. Its
truth file can hold a different base from the diag file, which is how the tests
tell which file a tiered metric read.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
import pytest

from rtc_tools.analysis import locomotion_diag as ld, walk_eval as we, walk_metrics as wm

DT = 0.002
START = 2.0  # the script's start on the log's axis [s]
STEP = 0.5  # one single support + one double support [s]
DOUBLE = 0.05
WEIGHT = 330.0
CONTACTS = ("left", "right")
PLAN_LOSS = ld.FAULT_CAUSE_NAMES.index("plan loss")
OTHER_FAULT = next(i for i in range(1, len(ld.FAULT_CAUSE_NAMES)) if i != PLAN_LOSS)


def synthetic(
    ratio=0.4, truth_ratio=None, fault_at=None, fault_code=OTHER_FAULT, length=30.0,
    script=wm.WALK_SCRIPT, freeze_after_fault=False,
):  # fmt: skip
    """(diag, truth) tables; the base advances at `ratio` (T: `truth_ratio`) of the command.

    `freeze_after_fault`: the logged command holds its value from the fault on, as the
    controller logs it (the command is no longer recorded once it latched a fault).
    """
    n = int(round(length / DT)) + 1
    t = np.arange(n) * DT
    cmd = np.array([wm.command_at(script, x - START) for x in t])
    if freeze_after_fault and fault_at is not None:
        cmd[t >= fault_at] = cmd[np.searchsorted(t, fault_at)]
    stepping = (t >= START + 0.5) & (t < START + 16.5)
    phase = np.mod(t - (START + 0.5), 2 * STEP)
    mask = np.full(n, 3)
    mask[stepping & (phase < STEP - DOUBLE)] = 1
    mask[stepping & (phase >= STEP) & (phase < 2 * STEP - DOUBLE)] = 2
    fault = np.zeros(n)
    if fault_at is not None:
        fault[t >= fault_at] = fault_code

    def base(r):
        psi = np.concatenate([[0.0], np.cumsum(cmd[:-1, 2] * DT)])
        c, sn = np.cos(psi[:-1]), np.sin(psi[:-1])
        ux, uy = r * cmd[:-1, 0], r * cmd[:-1, 1]  # the heading frame's forward / left
        px = np.concatenate([[0.0], np.cumsum((ux * c - uy * sn) * DT)])
        py = np.concatenate([[0.0], np.cumsum((ux * sn + uy * c) * DT)])
        zeros = np.zeros(n)
        return {
            "px": px, "py": py, "pz": np.full(n, 0.75),
            "qx": zeros, "qy": zeros, "qz": np.sin(psi / 2), "qw": np.cos(psi / 2),
            "vx": r * cmd[:, 0], "vy": zeros, "vz": zeros,
            "wx": zeros, "wy": zeros, "wz": cmd[:, 2],
        }  # fmt: skip

    zeros = np.zeros(n)
    diag = {
        "t_relative_s": t,
        "tick": np.arange(n, dtype=float),
        "fault_cause": fault,
        "contact_mask": mask.astype(float),
        "cmd_vx": cmd[:, 0],
        "cmd_vy": cmd[:, 1],
        "cmd_wz": cmd[:, 2],
        "ref_source": np.ones(n),
        "plausibility_rejects": zeros,
        "plan_holds": zeros,
        "qp_status": zeros,
        "margins_stale": zeros,
        "margin_friction_n": np.full(n, 5.0),
        "margin_cop_nm": np.full(n, 2.0),
        "margin_torque_nm": np.full(n, 9.0),
        "u_fb_norm": zeros,
        "mpc_last_seq": np.floor(t / 0.02) + 1.0,
        "mpc_last_outcome": np.where(np.mod(np.floor(t / 0.02), 10) == 0, 1.0, 0.0),
    }
    diag.update({f"base_{k}": v for k, v in base(ratio).items()})
    load = np.stack([(mask & 1) > 0, (mask & 2) > 0], axis=1) * 1.0
    load = WEIGHT * load / load.sum(axis=1, keepdims=True)
    for c, name in enumerate(CONTACTS):
        diag[f"lambda_{name}_fz"] = load[:, c]
        diag[f"wrench_des_{name}_fz"] = load[:, c]
    truth = {"tick": diag["tick"].copy(), "truth_mask": np.full(n, 7.0)}
    truth.update(
        {
            f"truth_base_{k}": v
            for k, v in base(ratio if truth_ratio is None else truth_ratio).items()
        }
    )
    for c, name in enumerate(CONTACTS):
        truth[f"truth_grf_{name}_fz"] = load[:, c]
    return diag, truth


def run_of(diag, truth=None, drop_truth_rows=None, script=wm.WALK_SCRIPT):
    header = list(diag)
    aligned = None
    if truth is not None:
        if drop_truth_rows is not None:
            keep = np.ones(len(truth["tick"]), dtype=bool)
            keep[drop_truth_rows] = False
            truth = {k: v[keep] for k, v in truth.items()}
        aligned = ld.align_on_tick(diag["tick"], truth)
    log = we.RunLog(
        header=header, diag=diag, contacts=ld.parse_diag_names(header).contacts, truth=aligned
    )
    return we.Run(log, script)


# ── tiers ─────────────────────────────────────────────────────────────────


def test_a_tiered_metric_reads_its_own_file():
    diag, truth = synthetic(ratio=0.4, truth_ratio=0.6)
    values = we.evaluate(run_of(diag, truth), ["v_ratio_est", "v_ratio_truth", "v_mps_truth"])
    assert values["v_ratio_est"] == pytest.approx(0.4, rel=1e-9)
    assert values["v_ratio_truth"] == pytest.approx(0.6, rel=1e-9)
    assert values["v_mps_truth"] == pytest.approx(0.6 * 0.5, rel=1e-9)


def test_the_truth_tier_is_refused_without_a_truth_file():
    diag, _ = synthetic()
    run = run_of(diag)
    assert we.evaluate(run, ["v_ratio_est"])["v_ratio_est"] == pytest.approx(0.4, rel=1e-9)
    with pytest.raises(we.MetricUnavailable, match="locomotion_truth.csv"):
        we.evaluate(run, ["v_ratio_truth"])
    with pytest.raises(we.MetricUnavailable, match="locomotion_truth.csv"):
        we.evaluate(run, ["contact_fz_error_truth"])
    values, refused = we.evaluate_available(run)
    assert "v_ratio_truth" in refused and "v_ratio_est" in values and "touchdowns" in values


@pytest.mark.parametrize(
    "key", ["v_ratio", "touchdowns_truth", "touchdowns_est", "weight_n_est", "weight_n", "nope"]
)
def test_a_key_outside_the_metrics_declared_tiers_is_refused(key):
    diag, truth = synthetic()
    with pytest.raises(we.MetricUnavailable):
        we.evaluate(run_of(diag, truth), [key])


def test_every_declared_key_parses_back_to_its_metric_and_tier():
    for name, metric in we.METRICS.items():
        for tier in metric.tiers:
            assert we.split_key(we.metric_key(name, tier)) == (name, tier)
    assert set(we.TIERS) == {t for m in we.METRICS.values() for t in m.tiers}


def test_a_truth_lane_that_skips_rows_gives_the_same_speed():
    diag, truth = synthetic(ratio=0.4, truth_ratio=0.4)
    whole = we.evaluate(run_of(diag, truth), ["v_mps_truth"])["v_mps_truth"]
    gaps = we.evaluate(run_of(diag, truth, np.arange(3000, 3400)), ["v_mps_truth"])["v_mps_truth"]
    assert gaps == pytest.approx(whole, rel=1e-9)


def test_a_missing_column_is_refused_not_read_as_zero():
    diag, truth = synthetic()
    del diag["mpc_last_outcome"]
    with pytest.raises(we.MetricUnavailable, match="mpc_last_outcome"):
        we.evaluate(run_of(diag, truth), ["withheld_rate"])
    assert we.constraints(run_of(diag, truth))["withheld_rate"] is None


# ── run metrics ───────────────────────────────────────────────────────────


def test_a_completed_run_counts_its_steps():
    diag, truth = synthetic()
    v = we.evaluate(
        run_of(diag, truth),
        ["completed", "touchdowns", "survival_s", "single_support_s", "fault_cause", "exposed",
         "straight_fault", "transition_fault", "plan_loss", "withheld_rate", "deadline_rate"],
    )  # fmt: skip
    assert v["completed"] is True and v["fault_cause"] is None
    assert v["touchdowns"] == 32  # 16 s of 0.5 s steps
    assert v["survival_s"] == pytest.approx(30.0 - (START + 0.5), abs=2 * DT)
    assert v["single_support_s"] == pytest.approx(STEP - DOUBLE, abs=2 * DT)
    assert v["exposed"] is True
    assert (v["straight_fault"], v["transition_fault"], v["plan_loss"]) == (False, False, False)
    assert v["withheld_rate"] == pytest.approx(0.1, abs=0.002)
    assert v["deadline_rate"] == 0.0


def test_the_speed_windows_are_placed_by_the_command():
    run = run_of(*synthetic())
    # B starts at START + 2 (the ramp reaches 90 % at START + 1.8) and the turn at START + 10.
    assert run.t[run.straight[0]] == pytest.approx(START + 1.8, abs=2 * DT)
    assert run.speed_window == pytest.approx((START + 2.8, START + 10.0), abs=2 * DT)
    assert run.crossings["turn_start"] == pytest.approx(START + 10.0, abs=2 * DT)
    assert run.crossings["turn_end"] == pytest.approx(START + 14.0, abs=2 * DT)
    assert run.crossings["stop"] == pytest.approx(START + 16.0, abs=2 * DT)


def test_the_command_rates_are_the_steps_over_the_window():
    v = we.evaluate(run_of(*synthetic()), ["rate_turn_start", "rate_turn_end", "rate_stop"])
    assert v["rate_turn_start"] == pytest.approx(0.3 / we.RATE_WINDOW_S)
    assert v["rate_turn_end"] == pytest.approx(0.3 / we.RATE_WINDOW_S)
    assert v["rate_stop"] == pytest.approx(0.5 / we.RATE_WINDOW_S)


def test_a_fault_before_the_turn_is_a_straight_fault():
    diag, truth = synthetic(fault_at=START + 6.02)
    run = run_of(diag, truth)
    v = we.evaluate(
        run,
        ["completed", "survival_s", "touchdowns", "straight_fault", "transition_fault", "exposed",
         "plan_loss", "fault_cause", "v_ratio_est"],
    )  # fmt: skip
    assert v["completed"] is False
    assert v["survival_s"] == pytest.approx(5.52, abs=2 * DT)
    assert v["touchdowns"] == 11
    assert (v["straight_fault"], v["transition_fault"], v["exposed"]) == (True, False, False)
    assert v["plan_loss"] is False and v["fault_cause"] == ld.FAULT_CAUSE_NAMES[OTHER_FAULT]
    # The speed window stops 0.5 s before the fault and is still over the minimum.
    assert run.speed_window[1] == pytest.approx(START + 5.52, abs=2 * DT)
    assert v["v_ratio_est"] == pytest.approx(0.4, rel=1e-9)


def test_a_fault_too_early_leaves_no_speed():
    run = run_of(*synthetic(fault_at=START + 4.0))  # 0.7 s of window
    assert we.evaluate(run, ["v_ratio_est"])["v_ratio_est"] is None


def test_a_fault_within_two_touchdowns_of_a_transition_is_a_transition_fault():
    at_turn = we.evaluate(
        run_of(*synthetic(fault_at=START + 10.6)), ["transition_fault", "straight_fault"]
    )
    assert at_turn == {"transition_fault": True, "straight_fault": False}
    later = we.evaluate(run_of(*synthetic(fault_at=START + 12.6)), ["transition_fault"])
    assert later["transition_fault"] is False


def test_plan_loss_is_the_fault_cause_at_the_first_fault_row():
    v = we.evaluate(run_of(*synthetic(fault_at=START + 12.6, fault_code=PLAN_LOSS)), ["plan_loss"])
    assert v["plan_loss"] is True


def test_the_contact_force_error_is_in_body_weights():
    diag, truth = synthetic()
    for name in CONTACTS:
        diag[f"wrench_des_{name}_fz"] = diag[f"wrench_des_{name}_fz"] + 33.0
    v = we.evaluate(run_of(diag, truth), ["weight_n_truth", "contact_fz_error_truth"])
    assert v["weight_n_truth"] == pytest.approx(WEIGHT)
    assert v["contact_fz_error_truth"] == pytest.approx(0.1, rel=1e-9)


def test_a_stale_contact_lane_is_left_out_of_the_force():
    diag, truth = synthetic()
    truth["truth_mask"][5000:6000] = 1.0  # the base only
    run = run_of(diag, truth)
    assert np.isnan(we.truth_normal_force(run)[5000:6000]).all()
    assert we.evaluate(run, ["contact_fz_error_truth"])["contact_fz_error_truth"] == 0.0


# ── the ranking cost ──────────────────────────────────────────────────────


def test_j_is_the_weighted_sum_of_scaled_errors():
    diag, truth = synthetic(ratio=0.4, truth_ratio=0.4)
    out = we.j_eval(run_of(diag, truth))
    assert out["partial"] is False and out["tier"] == we.TIER_TRUTH
    assert out["terms"]["forward_speed"] == pytest.approx((0.5 - 0.2) / 0.05, rel=1e-6)
    assert out["terms"]["yaw_rate"] == pytest.approx(0.0, abs=1e-9)
    assert out["terms"]["contact_fz"] == 0.0
    assert out["J"] == pytest.approx(sum(out["terms"].values()))


def test_a_faster_base_has_a_lower_j():
    slow = we.j_eval(run_of(*synthetic(ratio=0.3, truth_ratio=0.3)))["J"]
    fast = we.j_eval(run_of(*synthetic(ratio=0.8, truth_ratio=0.8)))["J"]
    assert fast < slow


def test_a_faulted_run_has_no_j():
    out = we.j_eval(run_of(*synthetic(fault_at=START + 15.0)))
    assert out["J"] is None and "fault" in out["why"]


def test_j_is_refused_without_the_truth_tier():
    diag, _ = synthetic()
    assert we.j_eval(run_of(diag))["J"] is None
    assert we.j_eval(run_of(diag), tier=we.TIER_EST)["J"] is None  # the force term is truth only
    partial = we.j_eval(run_of(diag), tier=we.TIER_EST, terms=["forward_speed", "tilt"])
    assert partial["partial"] is True and partial["J"] > 0.0


def test_the_constraint_vector_keeps_out_of_j():
    c = we.constraints(run_of(*synthetic()))
    assert set(c) == set(we.CONSTRAINT_KEYS)
    assert c["completed"] is True and c["margin_cop_min"] == 2.0 and c["qp_infeasible_rows"] == 0


# ── validity ──────────────────────────────────────────────────────────────

PROVENANCE = {
    "commit": "abc1234",
    "dirty": False,
    "config": {"files": 18, "installed_sha256": "f00", "source_sha256": "f00"},
    "overlay": None,
    "model_match": {"mass_kg": 9e-7, "com_m": 7e-9, "inertia_kgm2": 6e-7},
}
SHAPE = {"turn_start": 1.5, "turn_end": 1.5, "stop": 2.5}


def write_run(
    run_dir: Path, diag, truth=None, rtf=0.7, note=None, wip=False, appended=None, log_lines=()
):
    ctl = run_dir / we.CONTROLLER_DIR
    ctl.mkdir(parents=True)
    for name, table in ((ld.DIAG_FILE, diag), (we.TRUTH_FILE, truth)):
        if table is None:
            continue
        with (ctl / name).open("w", newline="") as handle:
            writer = csv.writer(handle)
            cols = {k: np.asarray(v)[::10] for k, v in table.items()}
            if table is diag and appended:
                cols.update(appended)  # at the stored rate, appended last like the controller's
            writer.writerow(cols)
            writer.writerows(zip(*cols.values(), strict=True))
    wall0 = 1000.0
    lines = [
        f"[sim-1] [INFO] [{wall0 + i:.3f}] [mujoco_simulator]: step sim_time={rtf * i:.3f} s"
        for i in range(60)
    ]
    (run_dir / "launch.log").write_text(
        "\n".join(lines + list(log_lines) + (["[wip] probe"] if wip else []))
    )
    (run_dir / "stance.csv").write_text("t\n" + "\n".join(f"{wall0 + 5 + i},0" for i in range(45)))
    base = {"recorder_rc": 0, "rtf": 0.7, "launch_args": [], "session": run_dir.name}
    (run_dir / "run.json").write_text(json.dumps({**base, **(note or {})}))


def coarse(table):
    """Tables at the 10× coarser rate write_run stores (0.02 s)."""
    return {k: np.asarray(v)[::10] for k, v in table.items()}


def validity_of(tmp_path, spec, overlay=None, run=None, **kw):
    diag, truth = run if run is not None else synthetic()
    write_run(tmp_path, diag, truth, **kw)
    note = json.loads((tmp_path / "run.json").read_text())
    run = we.Run(we.load_run(tmp_path / we.CONTROLLER_DIR), wm.WALK_SCRIPT)
    return we.run_validity(run, tmp_path, note, spec, overlay)


FULL_SPEC = we.ValiditySpec(
    single_support_s=(0.40, 0.50), command_rate=SHAPE, zero_columns=("u_fb_norm",)
)


def test_a_clean_run_is_valid(tmp_path):
    v = validity_of(tmp_path, FULL_SPEC, note={"provenance": PROVENANCE})
    assert v["valid"], v["checks"]
    assert v["measured"]["rtf"] == pytest.approx(0.7)
    assert set(v["checks"]) == {
        "recorder", "no_wip", "overlay", "rtf", "single_support", "command_shape",
        "zero_u_fb_norm", "commit_clean", "config_installed", "model_match",
        "log_readable", "log_spans_script",
    }  # fmt: skip


@pytest.mark.parametrize(
    ("kw", "failed"),
    [
        ({"note": {"provenance": PROVENANCE, "recorder_rc": 1}}, "recorder"),
        ({"note": {"provenance": PROVENANCE}, "rtf": 0.6}, "rtf"),
        ({"note": {"provenance": PROVENANCE}, "wip": True}, "no_wip"),
        ({"note": {"provenance": PROVENANCE, "launch_args": ["sim_overlay:=x.yaml"]}}, "overlay"),
        ({"note": {}}, "commit_clean"),
        ({"note": {"provenance": {**PROVENANCE, "dirty": True}}}, "commit_clean"),
        (
            {"note": {"provenance": {**PROVENANCE, "config": {"files": 18, "installed_sha256": "a", "source_sha256": "b"}}}},
            "config_installed",
        ),
        # No file was hashed: two equal hashes of nothing are not a match.
        (
            {"note": {"provenance": {**PROVENANCE, "config": {
                "files": 0, "installed_sha256": "e", "source_sha256": "e"}}}},
            "config_installed",
        ),
        # The model could not be compared: an error instead of numbers.
        (
            {"note": {"provenance": {**PROVENANCE, "model_match": {"error": "no mujoco"}}}},
            "model_match",
        ),
        # A run that names no real-time factor has nothing to be held to.
        ({"note": {"provenance": PROVENANCE, "rtf": None}}, "rtf"),
        # A plant 0.93 kg lighter than the control model: a body is missing.
        (
            {"note": {"provenance": {**PROVENANCE, "model_match": {
                "mass_kg": -0.93, "com_m": 8.3e-3, "inertia_kgm2": 0.12}}}},
            "model_match",
        ),
    ],
)  # fmt: skip
def test_each_check_fails_alone(tmp_path, kw, failed):
    v = validity_of(tmp_path, FULL_SPEC, **kw)
    assert not v["valid"]
    assert not v["checks"][failed]


def test_the_registered_shape_and_support_are_checked(tmp_path):
    spec = we.ValiditySpec(
        single_support_s=(0.30, 0.40), command_rate={"stop": 1.0}, provenance=False
    )
    v = validity_of(tmp_path, spec)
    assert not v["checks"]["single_support"] and not v["checks"]["command_shape"]
    assert "commit_clean" not in v["checks"]  # switched off, not passed


SHAPE_SPEC = we.ValiditySpec(command_rate=SHAPE, provenance=False)


def test_a_transition_after_the_fault_is_not_judged(tmp_path):
    # A fault in the yaw step (script 10 – 14 s): the controller logs no turn end or stop.
    run = synthetic(fault_at=START + 12.0, freeze_after_fault=True)
    v = validity_of(tmp_path, SHAPE_SPEC, run=run)
    assert v["measured"]["command_rate"]["turn_end"] is None
    assert v["checks"]["command_shape"], v["measured"]


def test_a_reached_transition_is_still_held_to_its_rate(tmp_path):
    run = synthetic(fault_at=START + 12.0, freeze_after_fault=True)
    spec = we.ValiditySpec(command_rate={**SHAPE, "turn_start": 3.0}, provenance=False)
    assert not validity_of(tmp_path, spec, run=run)["checks"]["command_shape"]


def test_a_missing_transition_without_a_fault_fails(tmp_path):
    diag, truth = synthetic()
    diag["cmd_wz"] = np.zeros_like(diag["cmd_wz"])  # the yaw step never came
    assert not validity_of(tmp_path, SHAPE_SPEC, run=(diag, truth))["checks"]["command_shape"]


def test_a_missing_transition_before_a_measured_one_fails(tmp_path):
    # Faulted after the stop, but the yaw step never came: not explained by the fault.
    diag, truth = synthetic(fault_at=START + 17.0, freeze_after_fault=True)
    diag["cmd_wz"] = np.zeros_like(diag["cmd_wz"])
    assert not validity_of(tmp_path, SHAPE_SPEC, run=(diag, truth))["checks"]["command_shape"]


def test_an_unknown_registered_transition_fails(tmp_path):
    spec = we.ValiditySpec(command_rate={**SHAPE, "jump": 1.0}, provenance=False)
    assert not validity_of(tmp_path, spec)["checks"]["command_shape"]


def test_the_overlay_is_matched_by_suffix_or_hash():
    note = {
        "launch_args": ["sim_overlay:=/a/b/arm.yaml"],
        "provenance": {"overlay": {"sha256": "h"}},
    }
    assert we.overlay_ok(note, "b/arm.yaml") and we.overlay_ok(note, "h")
    assert not we.overlay_ok(note, None) and not we.overlay_ok(note, "other.yaml")
    assert we.overlay_ok({"launch_args": ["max_rtf:=0.7"]}, None)


def test_a_reset_ends_the_measured_rtf_span():
    line = "[INFO] [{:.1f}] [mujoco_simulator]: sim_time={:.1f} s"
    log = "\n".join(line.format(100 + i, 0.5 * i) for i in range(10))
    log += "\n" + "\n".join(line.format(110 + i, 2.0 * i) for i in range(10))
    assert we.measured_rtf(log, (100.0, 200.0)) == pytest.approx(0.5)
    assert we.measured_rtf(log, (100.0, 101.5)) is None  # too few lines


# ── arms ──────────────────────────────────────────────────────────────────


def test_a_batch_judges_its_arms(tmp_path):
    for k in range(3):
        diag, truth = synthetic(ratio=0.4, truth_ratio=0.4)
        write_run(tmp_path / f"C_{k}", diag, truth, note={"provenance": PROVENANCE})
    for k, fault in enumerate((None, START + 6.0, START + 6.0)):
        diag, truth = synthetic(ratio=0.6, truth_ratio=0.6, fault_at=fault, fault_code=PLAN_LOSS)
        note = {
            "provenance": {**PROVENANCE, "overlay": {"sha256": "h"}},
            "launch_args": ["sim_overlay:=/x/t.yaml"],
        }
        write_run(tmp_path / f"T_{k}", diag, truth, note=note)
    write_run(tmp_path / "C_9", *synthetic(), rtf=0.5, note={"provenance": PROVENANCE})
    (tmp_path / "T_7").mkdir()
    spec = {
        "arms": {"C": {"runs": "C_*", "overlay": None}, "T": {"runs": "T_*", "overlay": "h"}},
        "control": "C",
        "validity": {"single_support_s": [0.40, 0.50]},
    }
    out = we.evaluate_batch(tmp_path, spec)
    c, t = out["arms"]["C"], out["arms"]["T"]
    assert c["summary"]["n"] == 3 and c["summary"]["completed"] == 3
    assert [r["run"] for r in c["runs"] if not r.get("valid", True)] == ["C_9"]
    assert t["summary"]["n"] == 3 and t["summary"]["completed"] == 1
    assert t["summary"]["plan_loss"] == 2 and t["summary"]["straight_fault"] == 2
    assert t["summary"]["v_ratio_truth"] == pytest.approx(0.6, rel=1e-6)
    assert t["summary"]["n_v"] == 3
    assert [r["run"] for r in t["runs"] if r.get("missing")] == ["T_7"]
    assert c["sessions_distinct"] and c["one_build_and_overlay"] and t["one_build_and_overlay"]
    assert out["completion"]["T"]["fisher_p"] == pytest.approx(0.2)  # 3/3 against 1/3
    assert out["completion"]["T"]["reject"] is False
    assert [r["j_eval"]["J"] is None for r in t["runs"] if not r.get("missing")] == [
        False, True, True,
    ]  # fmt: skip


def test_an_arm_below_the_minimum_is_void(tmp_path):
    write_run(tmp_path / "C_0", *synthetic(), note={"provenance": PROVENANCE})
    write_run(tmp_path / "T_0", *synthetic(), note={"provenance": PROVENANCE})
    spec = {
        "arms": {"C": {"runs": "C_*"}, "T": {"runs": "T_*"}},
        "control": "C",
        "min_valid": 2,
    }
    assert we.evaluate_batch(tmp_path, spec)["completion"]["T"] == {"void": True}


def test_runs_of_two_builds_do_not_read_as_one_arm(tmp_path):
    write_run(tmp_path / "C_0", *synthetic(), note={"provenance": PROVENANCE})
    other = {**PROVENANCE, "commit": "def5678"}
    write_run(tmp_path / "C_1", *synthetic(), note={"provenance": other})
    out = we.evaluate_batch(tmp_path, {"arms": {"C": {"runs": "C_*"}}})
    assert out["arms"]["C"]["one_build_and_overlay"] is False


def test_the_completion_constraint_only_rejects():
    control = {"n": 54, "completed": 27}
    worse = we.compare_completion(control, {"n": 54, "completed": 12})
    assert worse["reject"] and worse["fisher_p"] < 0.05
    better = we.compare_completion(control, {"n": 54, "completed": 40})
    assert not better["reject"] and better["fisher_p"] > 0.5
    assert math.isclose(worse["power_at_minus_0.2"], 0.61, abs_tol=0.02)


def test_a_log_cut_short_without_a_fault_is_not_a_completed_run(tmp_path):
    diag, truth = synthetic(length=START + 9.0)  # ends in the straight walk, no fault row
    write_run(tmp_path, diag, truth, note={"provenance": PROVENANCE})
    note = json.loads((tmp_path / "run.json").read_text())
    run = we.Run(we.load_run(tmp_path / we.CONTROLLER_DIR), wm.WALK_SCRIPT)
    v = we.run_validity(run, tmp_path, note, we.ValiditySpec(), None)
    assert not v["checks"]["log_spans_script"] and not v["valid"]
    # A run that faulted that early is a failure the arm counts, not a cut log.
    write_run(
        tmp_path / "f",
        *synthetic(length=START + 9.0, fault_at=START + 8.0),
        note={"provenance": PROVENANCE},
    )
    faulted = we.Run(we.load_run(tmp_path / "f" / we.CONTROLLER_DIR), wm.WALK_SCRIPT)
    vf = we.run_validity(faulted, tmp_path / "f", note, we.ValiditySpec(), None)
    assert vf["checks"]["log_spans_script"] and vf["valid"]


@pytest.mark.parametrize("column", ["contact_mask", "fault_cause", "cmd_vx"])
def test_a_cell_that_did_not_parse_in_a_placing_column_invalidates_the_run(column):
    diag, truth = synthetic()
    diag[column] = diag[column].copy()
    diag[column][4000] = math.nan
    run = run_of(diag, truth)
    assert not we.log_readable(run) and not we.log_spans_script(run)
    assert we.log_readable(run_of(*synthetic()))


def test_an_unreadable_run_is_listed_and_the_batch_goes_on(tmp_path):
    for k in range(2):
        write_run(tmp_path / f"C_{k}", *synthetic(), note={"provenance": PROVENANCE})
    write_run(tmp_path / "C_2", *synthetic(), note={"provenance": PROVENANCE})
    (tmp_path / "C_2" / we.CONTROLLER_DIR / ld.DIAG_FILE).write_text("")  # an empty log
    write_run(tmp_path / "C_3", *synthetic(), note={"provenance": PROVENANCE})
    (tmp_path / "C_3" / "launch.log").unlink()
    diag, truth = synthetic()
    del diag["plausibility_rejects"]  # a column no core metric reads
    write_run(tmp_path / "C_4", diag, truth, note={"provenance": PROVENANCE})
    diag, truth = synthetic()
    del diag["cmd_wz"]  # a placing column
    write_run(tmp_path / "C_5", diag, truth, note={"provenance": PROVENANCE})
    out = we.evaluate_batch(tmp_path, {"arms": {"C": {"runs": "C_*"}}})["arms"]["C"]
    by_run = {r["run"]: r for r in out["runs"]}
    assert "error" in by_run["C_2"] and "error" in by_run["C_3"]
    assert by_run["C_4"]["valid"] and "plausibility_rejects" in by_run["C_4"]["refused"]
    assert not by_run["C_5"]["valid"]
    assert out["summary"]["n"] == 3 and out["summary"]["completed"] == 3
    # A column the registration asks to be zero and the log lacks fails that check.
    spec = {"arms": {"C": {"runs": "C_*"}}, "validity": {"zero_columns": ["no_such_column"]}}
    again = we.evaluate_batch(tmp_path, spec)["arms"]["C"]
    assert again["summary"] == {"n": 0}
    assert not {r["run"]: r for r in again["runs"]}["C_0"]["checks"]["zero_no_such_column"]


# ── other directions ──────────────────────────────────────────────────────


@pytest.mark.parametrize(("name", "direction"), [("back", (-1.0, 0.0)), ("side", (0.0, 1.0))])
def test_a_backward_or_sideways_script_is_measured_along_its_own_direction(name, direction):
    script = wm.COMMAND_SCRIPTS[name]
    run = run_of(*synthetic(ratio=0.5, truth_ratio=0.5, script=script), script=script)
    speed = math.hypot(script[1].vx1, script[1].vy1)
    assert run.direction == pytest.approx(direction) and run.v_cmd == pytest.approx(speed)
    assert run.speed_window == pytest.approx((START + 2.8, START + 16.0), abs=2 * DT)
    v = we.evaluate(
        run, ["v_ratio_truth", "v_mps_est", "lateral_drift_m_est", "completed", "rate_stop"]
    )
    assert v["v_ratio_truth"] == pytest.approx(0.5, rel=1e-9)  # half the command, that way
    assert v["v_mps_est"] == pytest.approx(0.5 * speed, rel=1e-9)  # a speed along it: positive
    assert v["lateral_drift_m_est"] == pytest.approx(0.0, abs=1e-9)
    assert v["completed"] is True
    assert v["rate_stop"] == pytest.approx(speed / we.RATE_WINDOW_S)
    assert run.crossings["turn_start"] is None and run.crossings["stop"] == pytest.approx(
        START + 16.0, abs=2 * DT
    )
    assert we.log_spans_script(run)


def test_a_base_that_goes_the_wrong_way_has_a_negative_ratio():
    script = wm.COMMAND_SCRIPTS["back"]
    diag, truth = synthetic(ratio=-0.2, script=script)  # commanded back, walks forward
    assert we.evaluate(run_of(diag, truth, script=script), ["v_ratio_est"])[
        "v_ratio_est"
    ] == pytest.approx(-0.2, rel=1e-9)


def test_every_script_is_as_long_as_the_walk_and_ends_in_the_same_stop():
    for name, script in wm.COMMAND_SCRIPTS.items():
        assert wm.script_length(script) == wm.script_length(wm.WALK_SCRIPT), name
        assert script[-1].length_s == 10.0, name
        assert wm.command_at(script, 20.0) == (0.0, 0.0, 0.0), name  # at rest in the stop
    assert wm.command_at(wm.COMMAND_SCRIPTS["side"], 1.0) == pytest.approx((0.0, 0.1, 0.0))
    assert wm.command_at(wm.COMMAND_SCRIPTS["back"], 5.0) == pytest.approx((-0.3, 0.0, 0.0))
    assert wm.command_at(wm.WALK_SCRIPT, 1.0) == pytest.approx((0.25, 0.0, 0.0))


def test_an_arm_can_declare_the_model_difference_it_runs_with(tmp_path):
    heavy = {**PROVENANCE, "model_match": {"mass_kg": 0.8, "com_m": 4e-3, "inertia_kgm2": 0.05}}
    for arm in ("C", "P"):
        write_run(tmp_path / f"{arm}_0", *synthetic(), note={"provenance": heavy})
    spec = {
        "arms": {
            "C": {"runs": "C_*"},
            "P": {
                "runs": "P_*",
                "validity": {"model": {"mass_kg": 1.0, "com_m": 0.01, "inertia_kgm2": 0.1}},
            },
        },
        "validity": {"single_support_s": [0.40, 0.50]},
    }
    out = we.evaluate_batch(tmp_path, spec)["arms"]
    assert not out["C"]["runs"][0]["checks"]["model_match"]  # the batch's tolerance
    assert out["P"]["runs"][0]["valid"]  # the arm's declared one
    assert "single_support" in out["P"]["runs"][0]["checks"]  # the rest of the batch's still holds


# ── foot gap ──────────────────────────────────────────────────────────────

GAP = 0.2  # the stance width across the heading [m]
NARROW = (START + 4.0, START + 5.0)  # a straight-walking second with the feet close


def with_feet(diag, narrow_gap=0.03, left_side=1.0):
    """`diag` plus foot_<c>_x/y: the feet GAP apart across the base heading, a
    fore-aft offset during the turn (it must not count), and `narrow_gap` in NARROW.

    `left_side` −1 puts the left foot on the right (the stand orients the sign).
    """
    t = diag["t_relative_s"]
    psi = we._yaw({f"q{k}": diag[f"base_q{k}"] for k in "xyzw"})
    half = np.where((t >= NARROW[0]) & (t < NARROW[1]), narrow_gap, GAP) / 2
    turning = np.abs(diag["cmd_wz"]) > 0
    for name, lat, fwd in (
        ("left", left_side * half, np.where(turning, 0.2, 0.0)),
        ("right", -left_side * half, np.zeros_like(t)),
    ):
        diag[f"foot_{name}_x"] = diag["base_px"] + fwd * np.cos(psi) - lat * np.sin(psi)
        diag[f"foot_{name}_y"] = diag["base_py"] + fwd * np.sin(psi) + lat * np.cos(psi)
    return diag


def _expected_narrow(run):
    rows = run.walking_rows()
    narrow = (run.t >= NARROW[0]) & (run.t < NARROW[1])
    return (rows & narrow).sum() / rows.sum()


@pytest.mark.parametrize("left_side", [1.0, -1.0])
def test_the_foot_gap_is_across_the_heading_and_oriented_by_the_stand(left_side):
    diag, truth = synthetic()
    run = run_of(with_feet(diag, left_side=left_side), truth)
    run.foot_width_m = 0.06
    values = we.evaluate(run, ["foot_gap_min_est", "foot_gap_narrow_fraction_est"])
    assert values["foot_gap_min_est"] == pytest.approx(0.03, abs=1e-12)
    expected = _expected_narrow(run)
    assert 0.0 < expected < 1.0
    assert values["foot_gap_narrow_fraction_est"] == pytest.approx(expected, rel=1e-12)
    gap = we.foot_gap(run, we.TIER_EST)
    assert gap[run.walking_rows()].max() == pytest.approx(GAP, abs=1e-12)  # the turn's offset


def test_crossed_feet_read_a_negative_gap():
    diag, truth = synthetic()
    run = run_of(with_feet(diag, narrow_gap=-0.01), truth)
    assert we.evaluate(run, ["foot_gap_min_est"])["foot_gap_min_est"] == pytest.approx(-0.01)


def test_the_foot_gap_is_refused_without_what_it_needs():
    diag, truth = synthetic()
    run = run_of(with_feet(diag), truth)
    with pytest.raises(we.MetricUnavailable, match="foot_width_m"):
        we.evaluate(run, ["foot_gap_narrow_fraction_est"])
    assert we.evaluate(run, ["foot_gap_min_est"])["foot_gap_min_est"] == pytest.approx(0.03)
    for key in ("foot_gap_min_truth", "foot_gap_min"):
        with pytest.raises(we.MetricUnavailable):
            we.split_key(key)
    with pytest.raises(we.MetricUnavailable, match="foot_left_x"):
        we.evaluate(run_of(synthetic()[0], truth), ["foot_gap_min_est"])
    no_stand = with_feet(synthetic()[0])
    no_stand["ref_source"] = np.full(len(no_stand["tick"]), 2.0)  # walk from the first row
    with pytest.raises(we.MetricUnavailable, match="stand rows"):
        we.evaluate(run_of(no_stand, truth), ["foot_gap_min_est"])
    three = with_feet(synthetic()[0])
    three["lambda_extra_fz"] = np.zeros(len(three["tick"]))
    with pytest.raises(we.MetricUnavailable, match="exactly two contacts"):
        we.evaluate(run_of(three, truth), ["foot_gap_min_est"])


def test_a_batch_passes_its_foot_width_to_the_runs(tmp_path):
    diag, truth = synthetic()
    write_run(tmp_path / "C_0", with_feet(diag), truth, note={"provenance": PROVENANCE})
    spec = {"arms": {"C": {"runs": "C_*"}}, "foot_width_m": 0.06}
    out = we.evaluate_batch(tmp_path, spec)
    assert out["foot_width_m"] == 0.06
    metrics = out["arms"]["C"]["runs"][0]["metrics"]
    run = we.Run(we.load_run(tmp_path / "C_0" / we.CONTROLLER_DIR), wm.WALK_SCRIPT)
    assert metrics["foot_gap_narrow_fraction_est"] == pytest.approx(_expected_narrow(run))
    refused = we.evaluate_batch(tmp_path, {"arms": spec["arms"]})["arms"]["C"]["runs"][0]
    assert "foot_gap_narrow_fraction_est" in refused["refused"]
    for bad in (0.0, -0.06, "0.06", math.nan, True):
        with pytest.raises(ValueError, match="foot_width_m"):
            we.evaluate_batch(tmp_path, {**spec, "foot_width_m": bad})


# ── MPC lock-step runs ─────────────────────────────────────────────────────

STORED_ROWS = 1501  # synthetic() at the stored (0.02 s) rate
MPC_LINE = (
    "[locomotion_controller]: mpc: solves 500 (failed 0, deadline miss 0, withheld 0, overrun 0), "
    "solve p50 30.0 ms p99 50.0 ms; refused: epoch 0 state 0 age {age} dim 0 implausible 0"
)


def lockstep_columns(age_us=70000.0, period=35, n=STORED_ROWS):
    """The columns lock-step appends: a plan every `period` ticks, first used at `age_us`."""
    idx = np.arange(n)
    return {
        "policy_age_us": age_us + 2000.0 * (idx % period),
        "plan_id": (idx // period + 1).astype(float),
        "mpc_sync_lockstep": np.ones(n),
        "mpc_trigger_tick": (idx // period * period).astype(float),
        "mpc_wait_us": np.zeros(n),
        "mpc_wait_count": np.zeros(n),
        "mpc_pending_superseded": np.zeros(n),
    }


def lockstep_validity(tmp_path, appended=None, refused_age=0, sync="lockstep", rtf=0.7):
    diag, truth = synthetic()
    write_run(
        tmp_path, diag, truth, rtf=rtf, appended=appended or lockstep_columns(),
        note={"provenance": PROVENANCE, "mpc_sync": sync, "rtf": 0.7},
        log_lines=[MPC_LINE.format(age=refused_age)],
    )  # fmt: skip
    note = json.loads((tmp_path / "run.json").read_text())
    run = we.Run(we.load_run(tmp_path / we.CONTROLLER_DIR), wm.WALK_SCRIPT)
    return we.run_validity(run, tmp_path, note, FULL_SPEC, None)


def test_a_lockstep_run_checks_the_invariants_not_the_rtf(tmp_path):
    # the measured rtf (0.5) is far from the note's 0.7: lock-step does not hold it to one
    v = lockstep_validity(tmp_path, rtf=0.5)
    assert v["valid"], v["checks"]
    assert "rtf" not in v["checks"]
    assert {"lockstep_age", "lockstep_period", "lockstep_no_age_refusal"} <= set(v["checks"])
    assert v["measured"]["rtf"] == pytest.approx(0.5)


@pytest.mark.parametrize(
    ("kw", "failed"),
    [
        ({"appended": lockstep_columns(age_us=72000.0)}, "lockstep_age"),
        ({"appended": lockstep_columns(period=36)}, "lockstep_period"),
        ({"refused_age": 1}, "lockstep_no_age_refusal"),
    ],
)
def test_a_lockstep_invariant_broken_fails_its_check(tmp_path, kw, failed):
    v = lockstep_validity(tmp_path, **kw)
    assert not v["valid"]
    assert [k for k, ok in v["checks"].items() if not ok] == [failed]


def test_a_plan_rolled_back_to_an_earlier_id_is_not_judged_as_a_new_plan(tmp_path):
    # rows 420..454 (plan 13's span) return to plan 11 (a sanity refusal) with
    # the old plan's age: not a plan's first use, so the age invariant still holds
    cols = lockstep_columns()
    cols["plan_id"][420:455] = 11.0
    cols["policy_age_us"][420:455] = 140000.0
    v = lockstep_validity(tmp_path, appended=cols)
    assert v["checks"]["lockstep_age"], v["checks"]


def test_a_free_run_note_keeps_the_rtf_check_and_has_no_lockstep_checks(tmp_path):
    v = lockstep_validity(tmp_path, appended=lockstep_columns(), sync="free_run", rtf=0.5)
    assert "rtf" in v["checks"] and not v["checks"]["rtf"]
    assert not any(k.startswith("lockstep") for k in v["checks"])
