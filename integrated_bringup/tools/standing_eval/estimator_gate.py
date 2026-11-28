#!/usr/bin/env python3
"""Judge the state estimator of a recorded batch: open-loop thresholds and the closed loop.

One batch holds, per command script, runs on the truth base (``estimator.source:
truth``, the control) and on the estimate (``inekf``, the candidate), and
standing runs of both (WE-F06, WD-356 ~ 362). Each run directory is a
``run_arms.py`` output with the estimator replay of its session beside it
(``replay.csv`` and its sidecar — the driver runs ``estimator_replay`` on every
run). The spec (YAML) names the runs:

``scripts``
    ``{script: walk_eval spec}`` — one ``rtc_tools.analysis.walk_eval`` batch
    spec per command script (run globs under the root, overlay, validity). An
    arm's group is its name up to the first ``.``: ``truth`` or ``inekf``
    (``truth.s2`` is a truth arm with its own overlay). The ``walk`` script is
    the closed-loop one; every script's truth runs are the walking baseline.
``stand``
    ``{arms: {name: {runs, overlay}}, validity: {provenance, model}}`` — the
    standing runs (run_arms ``mpc`` arm), the same groups.

Only runs valid by walk_eval (standing: recorder · switch · overlay ·
provenance) count, and each must have its replay.

Open loop (WD-360) — on the truth runs' replays (``estimator_metrics``)
    The rules are fixed here; the numbers they give on the registered baseline are
    :data:`REGISTERED` and :data:`ANEES_WALK_BASELINE` (``--derive`` computes
    them; a registered judge reports whether they equal this batch's — true on
    the baseline itself). A continuous threshold is
    ⌈:data:`SCALE` × the baseline statistic⌉ on :data:`STEP`: a regression bar
    for later batches under the same condition (the baseline passes its own by
    construction). Gates on the baseline: walking-part ANEES inside its χ² band
    (recorded, not failed — outside it, the later G1 is the estimator replay
    ANEES of the later batch's truth runs ≤ :data:`ANEES_RATIO` × the
    baseline's, judged with the registered thresholds; an upper bound only, it
    catches error growth under the same noise values and not a covariance that
    grows more conservative), standing ANEES and every ANIS under the band's
    upper end, every run's tilt · yaw · velocity rms and drift
    per step under :data:`ABS_CAP`, on walking runs the z gyro bias error, the
    far-from-edge recall and precision of the worse foot and its slip false
    positives, and no slip cycle on any run.

Closed loop (b) — standing, every inekf run
    Counters 0 (fallen, fault rows, source not ok — a stale or missing lane is
    a host stall both arms show, counted apart and reported, resets after the start,
    QP infeasible rows, failed solves, epoch / dim refusals, Walk rows), the
    Stand fraction ≥ :data:`STAND_FRACTION_MIN`; the recorder's slip, drift and
    tilt and the truth height against the plan at ``T ≤ C + tol`` and under a
    bound (``walk_gate.STOP_WINDOW``, C = the truth runs' max); the online
    estimate equal to the replay over the scored rows, the replay's velocity
    rms under the standing threshold and the cap, no slip cycle.

Closed loop (c) — the walk script, truth against inekf
    Three one-sided tests, each at :data:`ALPHA` / :data:`TESTS` — completion
    (Fisher), survival from the first lift-off (RMST, censored at the script's
    end) and ``v_ratio_truth`` (median; ≥ :data:`V_RATIO_MIN_RUNS` runs with a
    speed window per arm, else undecidable), the permutation ones over
    :data:`N_PERM` relabellings at :data:`SEED`. A rejection fails. Every inekf
    run: no reset after the start before the cut, no source fault, no slip
    cycle (the online estimate's), online = replay (within WD-305's bounds over
    rows that cover the scored ones — a discrete divergence after the cut, from
    the 9-digit log, does not count), and where it has a speed
    window ``|v_ratio_est − v_ratio_truth|`` ≤ :data:`V_RATIO_AGREE` and
    ``|lateral_mps_est − lateral_mps_truth|`` ≤ :data:`LATERAL_AGREE_MPS`.

Reported, not judged: the tests' power, fault causes, touchdowns, the online
velocity error over the second before the cut, the inekf replays' metrics
against the cap, the estimator update time, the other scripts' completion.

Writes ``<root>/estimator_gate.json`` (or ``--out``); exit 0 only on PASS
(under ``--derive``: the baseline passing the thresholds derived from it).
Not installed — run from the source tree next to walk_gate.py.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis import (
    estimator_metrics as em,
    locomotion_diag as ld,
    walk_eval as we,
    walk_metrics as wm,
)
from rtc_tools.analysis.binomial_stats import fisher_one_sided, fisher_power, wilson_interval
from rtc_tools.analysis.permutation_stats import median_test, rmst_test


def _sibling(name: str):
    """A tool of this directory (not installed, so not importable by package name)."""
    path = Path(__file__).resolve().with_name(f"{name}.py")
    spec = importlib.util.spec_from_file_location(f"standing_eval_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


walk_gate = _sibling("walk_gate")  # the counter line, check(), the stop-window tolerances

REPLAY = "replay.csv"
ARMS = ("truth", "inekf")
CLOSED_LOOP_SCRIPT = "walk"

# ── open loop: the rules (WD-360) ─────────────────────────────────────────
SCALE = 1.5
STEP = 1e-3
ABS_CAP = 0.05  # WD-307: tilt · yaw [rad], velocity rms [m/s], drift per step [m]
BIAS_Z_DEV_MAX = 3e-3  # [rad/s] G5
RECALL_FAR_MIN = 0.90  # G6
PRECISION_FAR_MIN = 0.99  # G7
SLIP_FP_MAX = 0.005
ANEES_RATIO = 1.5
# The numbers the rules give on the registered baseline: the replays of 42 walking
# (walk 36 · turn_slow 3 · curve 3) and 5 standing truth runs on the shipped values of
# 2026-11-28 (matched plant, MPC lock-step, commands on sim time, a cell per run: start
# 3.0 – 3.7 s, yaw-step shift ±0.5 s, command scale 0.97 – 1.03). A regression bar for that
# condition; another condition (noise values, gait, plant) needs its own baseline.
THRESHOLD_KEYS = (
    "walk_vel_rms_median",
    "walk_vel_rms_max",
    "walk_tilt_rms_max",
    "walk_yaw_rms_max",
    "walk_drift_per_step_max",
    "stand_vel_rms_max",
)
REGISTERED: dict | None = {
    "walk_vel_rms_median": 0.003,  # [m/s]
    "walk_vel_rms_max": 0.004,  # [m/s]
    "walk_tilt_rms_max": 0.007,  # [rad]
    "walk_yaw_rms_max": 0.041,  # [rad]
    "walk_drift_per_step_max": 0.004,  # [m]
    "stand_vel_rms_max": 0.002,  # [m/s]
}
# Its walking-part ANEES, outside the χ² band (recorded; below it — the covariance is about
# four times the error): G1 — a later batch's walking replay ANEES ≤ ANEES_RATIO × this —
# is judged with the registered thresholds. None (REGISTERED too) is the state before a
# (re-)registration: --derive only.
ANEES_WALK_BASELINE: float | None = 0.257
ANEES_WALK_BASELINE_TOL = 5e-4  # its rounding

# ── closed loop (WD-358) ──────────────────────────────────────────────────
ALPHA = 0.05
TESTS = 3
N_PERM = 10_000
SEED = 20261121
V_RATIO_MIN_RUNS = 12
V_RATIO_AGREE = 0.02
LATERAL_AGREE_MPS = 0.01
STAND_FRACTION_MIN = 0.9
STAND_WINDOW_S = 20.0  # the standing run's recording: the last this many s of its log
SOURCE_FAULT = ld.FAULT_CAUSE_NAMES.index("source")
REF_WALK = ld.REFERENCE_SOURCE_NAMES.index("walk")
# Source statuses of a lane that did not arrive in time: a sensor stall of the
# host (~0.34 s on the dev PC), which truth-driven runs show too (12 of 68
# control runs, one row each). Not the estimate's: reported, not judged.
SOURCE_STALE = tuple(
    ld.SOURCE_STATUS_NAMES.index(n)
    for n in ("body unreadable", "body stale", "base missing", "base stale", "imu missing",
              "imu stale")
)  # fmt: skip

check = walk_gate.check


def record(name: str, value, limit, op: str) -> dict:
    """A check that is reported with its outcome and never fails the verdict."""
    return {**check(name, value, limit, op), "record": True}


def failed(checks: list[dict]) -> list[str]:
    return [c["name"] for c in checks if not c["pass"] and not c.get("record")]


# ── open loop ─────────────────────────────────────────────────────────────


def ceil_step(x: float) -> float:
    """x rounded up to the STEP grid (a value already on it stays)."""
    return round(math.ceil(round(x / STEP, 6)) * STEP, 9)


def _values(runs: list[dict], key: str) -> list[float]:
    return [r[key] for r in runs if r.get(key) is not None and math.isfinite(r[key])]


def derive_thresholds(walk: list[dict], stand: list[dict]) -> dict:
    """The rule's numbers from the baseline replays (``estimator_metrics.run_metrics`` dicts)."""
    vel = _values(walk, "vel_err_rms_truth")
    out = {
        "walk_vel_rms_median": statistics.median(vel) if vel else None,
        "walk_vel_rms_max": max(vel) if vel else None,
        "walk_tilt_rms_max": max(_values(walk, "tilt_rms_truth"), default=None),
        "walk_yaw_rms_max": max(_values(walk, "yaw_rms_truth"), default=None),
        "walk_drift_per_step_max": max(_values(walk, "pos_drift_per_step_truth"), default=None),
        "stand_vel_rms_max": max(_values(stand, "vel_err_rms_truth"), default=None),
    }
    agg = em.aggregate(walk)
    return {
        "baseline": out,
        "thresholds": {k: (None if v is None else ceil_step(SCALE * v)) for k, v in out.items()},
        "anees_walk_baseline": (agg.get("anees_walk_truth") or {}).get("value"),
    }


def _worse_foot(m: dict, part: str, key: str):
    vals = [c[part][key] if part else c[key] for c in m["contact"].values()]
    vals = [v for v in vals if v is not None]
    return vals


def run_checks(m: dict, walking: bool) -> list[dict]:
    """The checks every baseline run passes on its own (module docstring)."""
    out = [check(f"cap_{k}", m.get(k), ABS_CAP, "<=")
           for k in ("tilt_rms_truth", "yaw_rms_truth", "vel_err_rms_truth")]  # fmt: skip
    if m.get("pos_drift_per_step_truth") is not None:  # a run without a touchdown has none
        out.append(check("cap_pos_drift_per_step_truth", m["pos_drift_per_step_truth"],
                         ABS_CAP, "<="))  # fmt: skip
    out.append(check("slip_cycle_feet", m["slip_cycle"]["feet_cycling"], 0, "=="))
    if walking:
        recall = _worse_foot(m, "estimate_far_from_edges", "recall")
        precision = _worse_foot(m, "estimate_far_from_edges", "precision")
        fp = _worse_foot(m, "", "slip_false_positive")
        out += [
            check("gyro_bias_z_dev_max", m.get("gyro_bias_z_dev_max_truth"), BIAS_Z_DEV_MAX, "<="),
            check("recall_far_min", min(recall) if recall else None, RECALL_FAR_MIN, ">="),
            check(
                "precision_far_min", min(precision) if precision else None, PRECISION_FAR_MIN, ">="
            ),  # fmt: skip
            check("slip_false_positive_max", max(fp) if fp else None, SLIP_FP_MAX, "<="),
        ]
    return out


def baseline_checks(
    walk: list[dict],
    stand: list[dict],
    thresholds: dict | None,
    anees_walk_baseline: float | None = None,
) -> dict:
    """The baseline's set and run checks; the threshold ones need `thresholds`, G1
    (walking ANEES ≤ ANEES_RATIO × a registered out-of-band baseline) `anees_walk_baseline`."""
    w, s = em.aggregate(walk), em.aggregate(stand)
    anees = w.get("anees_walk_truth")
    lo, hi = anees["band"] if anees else (math.nan, math.nan)
    checks = [
        record("anees_walk_in_band_low", anees and anees["value"], lo, ">="),
        record("anees_walk_in_band_high", anees and anees["value"], hi, "<="),
        check(
            "anees_stand_below_upper",
            (s.get("anees_truth") or {}).get("value"),
            ((s.get("anees_truth") or {}).get("band") or [0, math.nan])[1],
            "<=",
        ),  # fmt: skip
        check(
            "anis_walk_below_upper",
            (w.get("anis") or {}).get("value"),
            ((w.get("anis") or {}).get("band") or [0, math.nan])[1],
            "<=",
        ),  # fmt: skip
        check(
            "anis_stand_below_upper",
            (s.get("anis") or {}).get("value"),
            ((s.get("anis") or {}).get("band") or [0, math.nan])[1],
            "<=",
        ),  # fmt: skip
    ]
    stats = derive_thresholds(walk, stand)["baseline"]
    if thresholds is not None:
        for key in THRESHOLD_KEYS:
            checks.append(check(key, stats[key], thresholds[key], "<="))
        if anees_walk_baseline is not None:
            checks.append(check("anees_walk_g1", anees and anees["value"],
                                g1_bound(anees_walk_baseline), "<="))  # fmt: skip
    runs = {}
    for group, walking in ((walk, True), (stand, False)):
        for m in group:
            runs[m["file"]] = run_checks(m, walking)
    return {
        "aggregate_walk": w,
        "aggregate_stand": s,
        "checks": checks,
        "runs": runs,
        "anees_walk_in_band": bool(anees and anees["pass"]),
        "failed": failed(checks) + [f"{r}:{n}" for r, c in runs.items() for n in failed(c)],
    }


# ── closed loop (b): standing ─────────────────────────────────────────────


def _bad(ok: np.ndarray) -> np.ndarray:
    """Rows not ok — a NaN cell (a value that did not parse) is not ok either."""
    return ~np.asarray(ok, dtype=bool)


def stand_numbers(run: Path) -> dict:
    """A standing run's counters and windows: the recorder's whole recording, the
    controller's diag from its first non-idle row, its truth file, its MPC counter line."""
    stance = json.loads((run / "stance.json").read_text())
    _, diag = walk_gate.load_diag(run)
    truth = walk_gate.load_truth(run, diag)
    t, ref = diag["t_relative_s"], diag["ref_source"]
    n = len(t)
    if n == 0:
        raise ValueError(f"{run}: the diag log has no rows")
    active = np.flatnonzero(ref != wm.REF_SOURCE_IDLE)
    rows = np.arange(n) >= (int(active[0]) if active.size else n)
    stand = ref == wm.REF_SOURCE_STAND
    height = None
    if truth is not None and "truth_base_pz" in truth:
        tail = stand & (t >= t[-1] - STAND_WINDOW_S)
        d = np.abs(truth["truth_base_pz"][tail] - diag["ref_base_pz"][tail])
        d = d[np.isfinite(d)]
        height = float(d.max()) if d.size else None
    mpc = walk_gate.mpc_counters(run / "launch.log") or {}
    qp_bad = np.isin(diag["qp_status"], wm.QP_INFEASIBLE_STATUSES) | np.isnan(diag["qp_status"])
    return {
        "fallen": stance.get("fallen"),
        "foot_slip_max": stance.get("foot_slip_max"),
        "base_tilt_max": stance.get("base_tilt_max"),
        "base_pos_drift_max": stance.get("base_pos_drift_max"),
        "height_vs_plan_max": height,
        "stand_fraction": float(np.mean(stand[rows])) if rows.any() else None,
        "walk_rows": int((rows & (ref == REF_WALK)).sum()),
        "fault_rows": int((rows & _bad(diag["fault_cause"] == 0)).sum()),
        "source_not_ok_rows": int(
            (
                rows
                & _bad(diag["source_status"] == 0)
                & ~np.isin(diag["source_status"], SOURCE_STALE)
            ).sum()
        ),
        "source_stale_rows": int((rows & np.isin(diag["source_status"], SOURCE_STALE)).sum()),
        "qp_infeasible_rows": int((rows & qp_bad).sum()),
        "failed_solves": mpc.get("failed"),
        "refused_epoch": mpc.get("refused_epoch"),
        "refused_dim": mpc.get("refused_dim"),
    }


def stand_validity(run: Path, note: dict, overlay: str | None, model: we.ModelTolerance) -> dict:
    log = (
        (run / "launch.log").read_text(errors="replace") if (run / "launch.log").is_file() else ""
    )
    checks = {
        "recorder": note.get("recorder_rc") == 0,
        "switch": note.get("switch_ok") is True,
        "no_error": "error" not in note,
        "no_wip": "[wip]" not in log,
        "overlay": we.overlay_ok(note, overlay),
        **we.provenance_checks(note, model),
    }
    return {"checks": checks, "valid": all(checks.values())}


def stand_control(truth: list[dict]) -> dict:
    """C of every continuous standing item: the truth runs' max (a NaN anywhere is NaN)."""
    out = {}
    for key in walk_gate.STOP_WINDOW:
        vals = [r["numbers"][key] for r in truth]
        bad = any(v is None or not math.isfinite(v) for v in vals) or not vals
        out[key] = math.nan if bad else max(vals)
    return out


def stand_checks(rec: dict, control: dict, thresholds: dict | None) -> list[dict]:
    """(b) on one inekf standing run (module docstring)."""
    nb, m, on = rec["numbers"], rec["replay"], rec["online"]
    src = m.get("online_source") or {}
    out = [check("not_fallen", nb["fallen"] is False, True, "==")]
    for key in ("fault_rows", "source_not_ok_rows", "qp_infeasible_rows", "walk_rows",
                "failed_solves", "refused_epoch", "refused_dim"):  # fmt: skip
        out.append(check(key, nb[key], 0, "=="))
    out += [
        check("resets_after_start", src.get("resets_after_start"), 0, "=="),
        check("stand_fraction", nb["stand_fraction"], STAND_FRACTION_MIN, ">="),
    ]
    for key, (tol, bound) in walk_gate.STOP_WINDOW.items():
        out.append(check(key, nb[key], control.get(key, math.nan) + tol, "<="))
        out.append(check(f"{key}_bound", nb[key], bound, "<="))
    out += [
        check("online_within_bounds", on.get("within_bounds"), True, "=="),
        check("online_covers_scored_rows", on.get("covers_scored_rows"), True, "=="),
        check("slip_cycle_feet", (src.get("slip_cycle") or {}).get("feet_cycling"), 0, "=="),
    ]
    out += [c for c in run_checks(m, walking=False) if c["name"].startswith("cap_")]
    if thresholds is not None:
        out.append(check("vel_err_rms_truth", m.get("vel_err_rms_truth"),
                         thresholds["stand_vel_rms_max"], "<="))  # fmt: skip
    return out


# ── closed loop (c): walking ──────────────────────────────────────────────


def walk_tests(
    truth: list[dict], inekf: list[dict], n_perm: int = N_PERM, seed: int = SEED
) -> dict:
    """The three tests on the walk_eval metrics of the valid runs (truth: the control)."""
    alpha = ALPHA / TESTS
    done_t = sum(bool(r["completed"]) for r in truth)
    done_i = sum(bool(r["completed"]) for r in inekf)
    p_c = fisher_one_sided(done_t, len(truth), done_i, len(inekf))
    rate = done_t / len(truth) if truth else math.nan
    n = min(len(truth), len(inekf))
    completion = {
        "truth": [done_t, len(truth)],
        "inekf": [done_i, len(inekf)],
        "wilson95": {
            "truth": wilson_interval(done_t, len(truth)),
            "inekf": wilson_interval(done_i, len(inekf)),
        },  # fmt: skip
        "p": p_c,
        "reject": p_c < alpha,
        "power_collapse": fisher_power(rate, 0.0, n, alpha) if n and rate == rate else None,
        "power_minus_0.2": fisher_power(rate, max(rate - 0.2, 0.0), n, alpha)
        if n and rate == rate
        else None,
    }
    surv = rmst_test([r["survival_s"] for r in truth], [not r["completed"] for r in truth],
                     [r["survival_s"] for r in inekf], [not r["completed"] for r in inekf],
                     n_perm, seed)  # fmt: skip
    surv["reject"] = surv["p"] < alpha
    v_t = [r.get("v_ratio_truth") for r in truth]
    v_i = [r.get("v_ratio_truth") for r in inekf]
    ratio = median_test(v_t, v_i, n_perm, seed + 1)
    ratio["undecidable"] = min(ratio["n_a"], ratio["n_b"]) < V_RATIO_MIN_RUNS
    ratio["reject"] = (not ratio["undecidable"]) and ratio["p"] < alpha
    return {
        "alpha_each": alpha,
        "completion": completion,
        "survival": surv,
        "v_ratio_truth": ratio,
    }


def _abs_diff(m: dict, a: str, b: str) -> float | None:
    if m.get(a) is None or m.get(b) is None:
        return None
    return abs(m[a] - m[b])


def deterministic_checks(rec: dict) -> list[dict]:
    """(c) on one inekf walking run (module docstring)."""
    m, rp, on = rec["metrics"], rec["replay"], rec["online"]
    src = rp.get("online_source") or {}
    out = [
        check("resets_after_start", src.get("resets_after_start"), 0, "=="),
        check(
            "source_fault", m.get("fault_cause") == ld.FAULT_CAUSE_NAMES[SOURCE_FAULT], False, "=="
        ),  # fmt: skip
        check("slip_cycle_feet", (src.get("slip_cycle") or {}).get("feet_cycling"), 0, "=="),
        check("online_within_bounds", on.get("within_bounds"), True, "=="),
        check("online_covers_scored_rows", on.get("covers_scored_rows"), True, "=="),
    ]
    if m.get("v_ratio_truth") is not None:  # a run with a speed window
        out += [
            check(
                "v_ratio_agree", _abs_diff(m, "v_ratio_est", "v_ratio_truth"), V_RATIO_AGREE, "<="
            ),  # fmt: skip
            check(
                "lateral_agree_mps",
                _abs_diff(m, "lateral_mps_est", "lateral_mps_truth"),
                LATERAL_AGREE_MPS,
                "<=",
            ),  # fmt: skip
        ]
    return out


def walk_report(runs: list[dict]) -> dict:
    ms = [r["metrics"] for r in runs]
    last = [(r["replay"].get("online_source") or {}).get("vel_err_rms_last_s_truth") for r in runs]
    return {
        "n": len(runs),
        "fault_causes": dict(Counter(m.get("fault_cause") or "none" for m in ms)),
        "touchdowns_median": we._median([m.get("touchdowns") for m in ms]),
        "survival_s_median": we._median([m.get("survival_s") for m in ms]),
        "v_ratio_truth_median": we._median([m.get("v_ratio_truth") for m in ms]),
        "online_vel_err_last_s": [x for x in last if x is not None],
    }


# ── the batch ─────────────────────────────────────────────────────────────


def _replay(run_dir: Path, online: bool) -> tuple[dict, dict | None]:
    cols, meta = em.load_replay(run_dir / REPLAY)
    m = em.run_metrics(cols, meta)
    m["file"] = str(run_dir)
    return m, (em.compare_online(cols, meta) if online else None)


def group_of(arm: str) -> str:
    """The arm's group (module docstring); refuses a name outside ARMS."""
    group = arm.split(".", 1)[0]
    if group not in ARMS:
        raise ValueError(f"arm {arm!r}: its group must be one of {ARMS}")
    return group


def _groups(arms: dict) -> None:
    present = {group_of(a) for a in arms}
    if present != set(ARMS):
        raise ValueError(f"the arms {sorted(arms)} do not cover both groups {ARMS}")


def collect_scripts(root: Path, spec: dict) -> tuple[dict, list[str]]:
    """walk_eval per script, the valid runs with their replays, and the unusable runs."""
    out, errors = {}, []
    for script, sspec in spec["scripts"].items():
        _groups(sspec["arms"])
        if sspec.get("script", script) != script:
            raise ValueError(f"script {script}: its spec names script {sspec['script']!r}")
        batch = we.evaluate_batch(
            root, {**{k: v for k, v in sspec.items() if k != "control"}, "script": script}
        )
        arms: dict[str, list] = {a: [] for a in ARMS}
        for name, arm in batch["arms"].items():
            group = group_of(name)
            for r in arm["runs"]:
                if not r.get("valid"):
                    errors.append(f"{script}/{name}: invalid {r['run']}")
                    continue
                try:
                    rp, on = _replay(root / r["run"], online=group == "inekf")
                except Exception as exc:  # one unreadable run must not take the batch with it
                    errors.append(f"{script}/{name}: no replay {r['run']} ({exc})")
                    continue
                arms[group].append(
                    {"run": r["run"], "metrics": r["metrics"], "replay": rp, "online": on}
                )
        summary = {
            a: we.arm_summary([r["metrics"] for r in arms[a]], we.TIER_TRUTH)
            if arms[a]
            else {"n": 0}
            for a in ARMS
        }
        out[script] = {"arms": arms, "walk_eval": summary}
    return out, errors


def collect_stand(root: Path, spec: dict) -> tuple[dict, list[str]]:
    """The valid standing runs of each group with their numbers and replays."""
    vspec = spec.get("validity") or {}
    model = we.ModelTolerance(**(vspec.get("model") or {}))
    _groups(spec["arms"])
    out: dict[str, list] = {a: [] for a in ARMS}
    errors = []
    for name, arm in spec["arms"].items():
        group = group_of(name)
        for run in sorted(root.glob(arm["runs"])):
            rel = str(run.relative_to(root))
            try:
                note = json.loads((run / "run.json").read_text())
                validity = stand_validity(run, note, arm.get("overlay"), model)
                if not vspec.get("provenance", True):
                    for k in ("commit_clean", "config_installed", "model_match"):
                        validity["checks"].pop(k)
                    validity["valid"] = all(validity["checks"].values())
                if not validity["valid"]:
                    errors.append(f"stand/{name}: invalid {rel} {validity['checks']}")
                    continue
                rp, on = _replay(run, online=group == "inekf")
                numbers = stand_numbers(run)
            except Exception as exc:  # one unreadable run must not take the batch with it
                errors.append(f"stand/{name}: unreadable {rel} ({type(exc).__name__}: {exc})")
                continue
            out[group].append({"run": rel, "numbers": numbers, "replay": rp, "online": on})
    return out, errors


def judge(scripts: dict, stand: dict, thresholds: dict | None, n_perm: int = N_PERM) -> dict:
    """Every verdict of the batch from the collected runs (pure: no file access)."""
    walk_truth = [r["replay"] for s in scripts.values() for r in s["arms"]["truth"]]
    stand_truth = [r["replay"] for r in stand["truth"]]
    derived = derive_thresholds(walk_truth, stand_truth)
    use = thresholds if thresholds is not None else derived["thresholds"]
    g1 = ANEES_WALK_BASELINE if thresholds is not None else None  # registered only
    base = baseline_checks(walk_truth, stand_truth, use, g1)
    control = stand_control(stand["truth"])
    b = {r["run"]: stand_checks(r, control, use) for r in stand["inekf"]}
    walk = scripts.get(CLOSED_LOOP_SCRIPT)
    if walk is None:
        raise ValueError(f"the spec has no `{CLOSED_LOOP_SCRIPT}` script (the closed-loop one)")
    t_runs, i_runs = walk["arms"]["truth"], walk["arms"]["inekf"]
    tests = walk_tests([r["metrics"] for r in t_runs], [r["metrics"] for r in i_runs], n_perm)
    det = {r["run"]: deterministic_checks(r) for r in i_runs}
    stand_failed = [f"{r}:{n}" for r, c in b.items() for n in failed(c)]
    if not stand["inekf"] or not stand["truth"]:
        stand_failed.append("no standing runs")
    walk_failed = [k for k in ("completion", "survival", "v_ratio_truth") if tests[k]["reject"]]
    walk_failed += [f"{r}:{n}" for r, c in det.items() for n in failed(c)]
    if not t_runs or not i_runs:
        walk_failed.append("no walking runs")
    inekf_replays = {
        r["run"]: {
            k: r["replay"].get(k)
            for k in (
                "tilt_rms_truth",
                "yaw_rms_truth",
                "vel_err_rms_truth",
                "pos_drift_per_step_truth",
                "nees_walk_mean_per_dof_truth",
                "nis_mean_per_dof",
            )
        }  # fmt: skip
        for s in scripts.values()
        for r in s["arms"]["inekf"]
    }
    for v in inekf_replays.values():
        v["under_cap"] = all(v[k] is None or v[k] <= ABS_CAP for k in
                             ("tilt_rms_truth", "yaw_rms_truth", "vel_err_rms_truth",
                              "pos_drift_per_step_truth"))  # fmt: skip
    update = [r["replay"].get("update_ns") for s in scripts.values() for a in ARMS
              for r in s["arms"][a]]  # fmt: skip
    open_failed = list(base["failed"])
    out = {
        "thresholds": use,
        "thresholds_registered": thresholds is not None,
        "derived": derived,
        "open_loop": {**base, "pass": not open_failed},
        "stand": {
            "control": control,
            "runs": b,
            "failed": stand_failed,
            "pass": not stand_failed,
            "source_stale_rows": {
                r["run"]: r["numbers"]["source_stale_rows"] for g in ARMS for r in stand[g]
            },
        },
        "walk": {
            "tests": tests,
            "runs": det,
            "undecidable": tests["v_ratio_truth"]["undecidable"],
            "failed": walk_failed,
            "pass": not walk_failed and not tests["v_ratio_truth"]["undecidable"],
            "report": {a: walk_report(walk["arms"][a]) for a in ARMS},
        },
        "report": {
            "scripts": {k: sc["walk_eval"] for k, sc in scripts.items()},
            "inekf_replays": inekf_replays,
            "update_ns": [u for u in update if u],
        },
    }
    if thresholds is not None:  # the registered bound (none if the baseline was in band)
        if ANEES_WALK_BASELINE is not None:
            out["open_loop"]["g1_anees_walk_max"] = g1_bound(ANEES_WALK_BASELINE)
    elif not base["anees_walk_in_band"] and derived["anees_walk_baseline"] is not None:
        out["open_loop"]["g1_anees_walk_max"] = g1_bound(derived["anees_walk_baseline"])
    fails = open_failed or stand_failed or walk_failed
    verdict = "FAIL" if fails else ("UNDECIDABLE" if out["walk"]["undecidable"] else "PASS")
    out["verdict"] = verdict if thresholds is not None else f"{verdict} (unregistered thresholds)"
    return out


def g1_bound(anees_walk_baseline: float) -> float:
    """G1's upper end: ANEES_RATIO × an out-of-band walking-ANEES baseline."""
    return round(ANEES_RATIO * anees_walk_baseline, 9)


def registered_matches(thresholds: dict, derived: dict) -> bool:
    """The registered numbers equal the ones the rules give on this batch's baseline."""
    same = all(
        math.isclose(thresholds[k], math.nan if (v := derived["thresholds"][k]) is None else v)
        for k in THRESHOLD_KEYS
    )
    b = derived["anees_walk_baseline"]
    return (
        same
        and ANEES_WALK_BASELINE is not None
        and b is not None
        and abs(ANEES_WALK_BASELINE - b) <= ANEES_WALK_BASELINE_TOL
    )


def registered() -> dict | None:
    if REGISTERED is None:
        return None
    missing = [k for k in THRESHOLD_KEYS if REGISTERED.get(k) is None]
    if missing:
        raise ValueError(f"REGISTERED lacks {missing}")
    return dict(REGISTERED)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("root", type=Path, help="the batch directory the spec's run globs are under")
    p.add_argument("--spec", type=Path, required=True, help="the batch's run registration (YAML)")
    p.add_argument("--out", type=Path, help="default: <root>/estimator_gate.json")
    p.add_argument(
        "--derive",
        action="store_true",
        help="judge with the thresholds the rules give on this batch's baseline (the "
        "registration step), not the registered ones",
    )
    args = p.parse_args(argv)
    spec = yaml.safe_load(args.spec.read_text())
    thresholds = None if args.derive else registered()
    if thresholds is None and not args.derive:
        p.error("no registered thresholds yet: --derive computes them from a baseline")
    scripts, errors = collect_scripts(args.root, spec)
    stand, stand_errors = collect_stand(args.root, spec["stand"])
    result = judge(scripts, stand, thresholds)
    result["unusable_runs"] = errors + stand_errors
    if thresholds is not None:
        result["registered_equals_derived"] = registered_matches(thresholds, result["derived"])
    out = args.out or args.root / "estimator_gate.json"
    out.write_text(json.dumps(result, indent=1, default=walk_gate._json) + "\n")
    print("verdict", result["verdict"])
    for part in ("open_loop", "stand", "walk"):
        r = result[part]
        state = (
            "PASS"
            if r["pass"]
            else ("UNDECIDABLE" if r.get("undecidable") and not r["failed"] else "FAIL")
        )
        print(f"{part:10s} {state} {r['failed']}")
    print("derived", json.dumps(result["derived"], default=float))
    for e in result["unusable_runs"]:
        print("unusable:", e)
    # --derive: PASS (unregistered thresholds) is the registration step passing.
    return 0 if result["verdict"].startswith("PASS") else 1


if __name__ == "__main__":
    sys.exit(main())
