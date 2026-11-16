#!/usr/bin/env python3
"""Judge the walking arms of a run_arms.py output directory.

Reads ``<out>/walk_<k>/``, ``<out>/mpc_<k>/`` (the standing control) and
``<out>/walk_estop_<k>/`` and writes ``<out>/walk_gate.json``. Every limit is
fixed here before the runs (a continuous value passes at ``T ≤ C + tol``, a
quantity with no control is an absolute bound, a counter is 0 only where the
control's is 0); a run passes when every check passes, the arm when every
run does. ``--probe`` instead reports what fixes the dev-PC real-time factor
(one walk run): no plan refused for its age, no row without a plan once
standing, the plan age p99.

Sources per run: the controller's ``locomotion_diag.csv`` (walking numbers:
rtc_tools.analysis.walk_metrics), the recorder's ``stance.json`` (a fall
anywhere) and its stop / hold window in ``run.json`` (slip, tilt, drift,
time to fall), ``launch.log`` (the CM's refused outputs, the controller's
cumulative MPC counter line). Performance is reported, not judged (the
control PCs judge it).
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis import locomotion_diag as ld, stance_metrics as sm, walk_metrics as wm

DIAG = Path("session") / "controllers" / "locomotion_controller" / "locomotion_diag.csv"
TIMING = Path("session") / "timing" / "mpc_timing_log.csv"

# ── The limits (fixed before the runs) ─────────────────────────────────────
WALK_ENTRY_S = 5.0  # after the ramp's end
WALK_FRACTION = 0.90  # of the rows from the first walk row to D's end
GAIT_CHANGES_B_TO_D = 2
STEPS_MIN = 14
VX_ERR_RMS = 0.10  # [m/s]
VX_MEAN_ERR = 0.05
YAW_ERR_RMS = 0.10  # [rad/s]
YAW_MEAN_ERR = 0.05
LATERAL_DRIFT_M = 0.30
HEADING_DRIFT_RAD = 0.15
STOP_SETTLE_S = 6.0
# The stop window against the standing control's max over its runs: C + tol, and the bound.
STOP_WINDOW = {
    "foot_slip_max": (1.5e-3, 5e-3),
    "base_tilt_max": (0.02, 0.05),
    "height_vs_plan_max": (10e-3, 20e-3),
    "base_pos_drift_max": (10e-3, 20e-3),
}
MISMATCH_P95_MS = 70.0
MISMATCH_MAX_MS = 140.0
EOM_RESIDUAL = 1e-5
MARGIN_MIN = -1e-5
DEGRADED_FRACTION = 0.01
ROWS_FRACTION = 0.95  # validity
SCRIPT_LENGTH_TOL = 0.05  # validity: the logged script within 5 % of its sim-second length
PROBE_AGE_P99_MS = 100.0

MPC_LINE = re.compile(
    r"mpc: solves (?P<solves>\d+) \(failed (?P<failed>\d+), deadline miss (?P<deadline_miss>\d+), "
    r"withheld (?P<withheld>\d+), overrun (?P<overrun>\d+)\), "
    r"solve p50 (?P<solve_p50_ms>[\d.]+) ms p99 (?P<solve_p99_ms>[\d.]+) ms; "
    r"refused: epoch (?P<refused_epoch>\d+) state (?P<refused_state>\d+) age (?P<refused_age>\d+) "
    r"dim (?P<refused_dim>\d+) implausible (?P<implausible>\d+)"
)
CM_REJECTED = re.compile(r"rejected on the RT path|output.{0,40}rejected", re.IGNORECASE)


def mpc_counters(launch_log: Path) -> dict | None:
    """The controller's last cumulative MPC counter line."""
    if not launch_log.is_file():
        return None
    found = list(MPC_LINE.finditer(launch_log.read_text(errors="replace")))
    if not found:
        return None
    return {
        k: (float(v) if k.endswith("_ms") else int(v)) for k, v in found[-1].groupdict().items()
    }


def cm_rejected_lines(launch_log: Path) -> int:
    if not launch_log.is_file():
        return 0
    return sum(
        bool(CM_REJECTED.search(x)) for x in launch_log.read_text(errors="replace").splitlines()
    )


def gait_names(gait_yaml: Path) -> list[str]:
    """The gait sequence (the controller's gait index is a position in it)."""

    def find(node):
        if isinstance(node, dict):
            if isinstance(node.get("sequence"), list):
                return node["sequence"]
            for v in node.values():
                found = find(v)
                if found is not None:
                    return found
        return None

    seq = find(yaml.safe_load(gait_yaml.read_text()))
    if seq is None:
        raise ValueError(f"{gait_yaml}: no gait `sequence:`")
    return [str(g) for g in seq]


def check(name: str, value, limit, op: str) -> dict:
    """One check; a missing value fails."""
    ok = value is not None and not (isinstance(value, float) and math.isnan(value))
    if ok:
        ok = {"<=": value <= limit, ">=": value >= limit, "==": value == limit}[op]
    return {"name": name, "value": value, "limit": limit, "op": op, "pass": bool(ok)}


def load_diag(run: Path) -> tuple[list[str], dict]:
    return ld.read_table(run / DIAG)


def height_vs_plan(diag: dict, t0: float, t1: float) -> float | None:
    win = wm.in_window(diag["t_relative_s"], t0, t1) & (diag["ref_source"] == wm.REF_SOURCE_STAND)
    d = np.abs(diag["base_pz"][win] - diag["ref_base_pz"][win])
    d = d[np.isfinite(d)]
    return float(d.max()) if d.size else None


def control_stop_window(
    mpc_runs: list[Path], window_s: float, rtf: float | None, z_ref: float
) -> dict:
    """The standing control's stop-window numbers, max over its runs: the last
    `window_s` sim seconds of its recording and of its log."""
    wall = window_s / (rtf or 1.0)
    out: dict[str, float] = {}
    for run in mpc_runs:
        samples = sm.read_csv(run / "stance.csv")
        if not samples:
            raise ValueError(f"{run}: empty stance.csv")
        tail = [s for s in samples if s.t >= samples[-1].t - wall]
        m = sm.compute(tail, settle_s=0.0, force_threshold_n=5.0, z_ref=z_ref)
        _, diag = load_diag(run)
        t = diag["t_relative_s"]
        h = height_vs_plan(diag, t[-1] - window_s, t[-1] + 1.0)
        values = {
            "foot_slip_max": m.foot_slip_max,
            "base_tilt_max": m.base_tilt_max,
            "base_pos_drift_max": m.base_pos_drift_max,
            "height_vs_plan_max": h if h is not None else math.nan,
        }
        for k, v in values.items():
            # A NaN in any run is the control's value: it fails every check against it.
            prev = out.get(k, -math.inf)
            out[k] = math.nan if (math.isnan(prev) or math.isnan(v)) else max(prev, v)
    return out


def timing_report(run: Path) -> dict | None:
    path = run / TIMING
    if not path.is_file():
        return None
    _, cols = ld.read_table(path)
    return {
        k: {
            "p50": float(np.nanpercentile(cols[k], 50)),
            "p99": float(np.nanpercentile(cols[k], 99)),
        }
        for k in ("t_compute_us", "t_total_us")
        if k in cols and np.isfinite(cols[k]).any()
    }


def judge_walk(run: Path, gaits: list[str], control: dict, stop_window_s: float) -> dict:
    note = json.loads((run / "run.json").read_text())
    header, diag = load_diag(run)
    s = wm.walk_summary(header, diag, gaits)
    if "error" in s:
        return {
            "run": run.name,
            "pass": False,
            "error": s["error"],
            "note_error": note.get("error"),
        }
    stance = (
        json.loads((run / "stance.json").read_text()) if (run / "stance.json").is_file() else {}
    )
    stop = note.get("windows", {}).get("stop", {})
    # A log without the contact lane has no contacts / events numbers: their
    # checks then read None and fail.
    c, g, tr, dr = (s[k] for k in ("counters", "gait", "tracking", "drift"))
    ct, ev = s.get("contacts", {}), s.get("events", {})
    # The stop window as the publisher scheduled it (run_arms marks the same span).
    e1 = s["script_end_scheduled_s"]
    checks = [
        check("script_events_ok", bool(note.get("events_ok")), True, "=="),
        check("not_fallen_whole_run", stance.get("fallen") is False, True, "=="),
        check("not_fallen_stop_window", stop.get("fallen") is False, True, "=="),
    ]
    for key in (
        "fault_latched_rows",
        "fault_count_increase",
        "pose_guard_rows",
        "plan_loss_rows",
        "plausibility_reject_increase",
        "qp_infeasible_rows",
        "health_fault_rows",
        "rows_dropped_rows",
        "source_not_ok_rows",
        "cmd_invalid_rows_b_to_d",
    ):
        checks.append(check(key, c[key], 0, "=="))
    checks += [
        check("gait_down_changes_b_to_d", g["down_changes_b_to_d"], 0, "=="),
        check("walk_entry_s_after_ramp_end", g["walk_entry_s_after_ramp_end"], WALK_ENTRY_S, "<="),
        check("walk_fraction_from_entry", g["walk_fraction_from_entry"], WALK_FRACTION, ">="),
        check("gait_changes_b_to_d", g["changes_b_to_d"], GAIT_CHANGES_B_TO_D, "<="),
        check("steps_completed", s.get("steps_completed"), STEPS_MIN, ">="),
        check("vx_err_rms", tr["vx_err_rms"], VX_ERR_RMS, "<="),
        check("vx_mean_err", tr["vx_mean_err"], VX_MEAN_ERR, "<="),
        check("yaw_rate_err_rms", tr["yaw_rate_err_rms"], YAW_ERR_RMS, "<="),
        check("yaw_rate_mean_err", tr["yaw_rate_mean_err"], YAW_MEAN_ERR, "<="),
        check("lateral_drift_m", dr["lateral_drift_m"], LATERAL_DRIFT_M, "<="),
        check("heading_drift_rad", dr["heading_drift_rad"], HEADING_DRIFT_RAD, "<="),
        check("stop_settle_s", s["stop"]["stop_settle_s"], STOP_SETTLE_S, "<="),
        check("missed_edges", ct.get("missed_edges"), 0, "=="),
        check("extra_edges", ct.get("extra_edges"), 0, "=="),
        check("slot_check_failures", ct.get("slot_check_failures"), 0, "=="),
        check("mismatch_abs_p95_ms", ct.get("mismatch_abs_p95_ms"), MISMATCH_P95_MS, "<="),
        check("mismatch_abs_max_ms", ct.get("mismatch_abs_max_ms"), MISMATCH_MAX_MS, "<="),
        check("event_eom_residual_inf_max", ev.get("eom_residual_inf_max"), EOM_RESIDUAL, "<="),
        check("event_tau_invalid_rows", ev.get("tau_invalid_rows"), 0, "=="),
        check("degraded_fraction", c["degraded_fraction"], DEGRADED_FRACTION, "<="),
    ]
    for m, v in ev.get("margin_min", {}).items():
        checks.append(check(f"event_{m}_min", v, MARGIN_MIN, ">="))
    measured = {
        "foot_slip_max": stop.get("foot_slip_max"),
        "base_tilt_max": stop.get("base_tilt_max"),
        "base_pos_drift_max": stop.get("base_pos_drift_max"),
        "height_vs_plan_max": height_vs_plan(diag, e1 - stop_window_s, e1),
    }
    for k, (tol, bound) in STOP_WINDOW.items():
        checks.append(check(f"stop_{k}", measured[k], control.get(k, math.nan) + tol, "<="))
        checks.append(check(f"stop_{k}_bound", measured[k], bound, "<="))
    validity = [
        check("rows_fraction", s["rows"] / max(1, s["rows_expected"]), ROWS_FRACTION, ">="),
        # The windows the rms and means reduce over hold enough finite samples.
        check("vx_window_finite_fraction", tr["vx_window_finite_fraction"], ROWS_FRACTION, ">="),
        check("yaw_window_finite_fraction", tr["yaw_window_finite_fraction"], ROWS_FRACTION, ">="),
        check("force_valid_fraction", ct.get("force_valid_fraction"), ROWS_FRACTION, ">="),
        check(
            "script_length_rel_err",
            abs(s["script_length_s"] / s["script_length_nominal_s"] - 1.0),
            SCRIPT_LENGTH_TOL,
            "<=",
        ),
    ]
    return {
        "run": run.name,
        "pass": all(x["pass"] for x in checks + validity),
        "failed": [x["name"] for x in checks + validity if not x["pass"]],
        "checks": checks,
        "validity": validity,
        "summary": s,
        "stop_window": measured,
        "report": {
            **s["report"],
            "mpc": mpc_counters(run / "launch.log"),
            "timing": timing_report(run),
            "recorder_gap_max_s": stop.get("sample_gap_max_s"),
        },
    }


def judge_estop(run: Path) -> dict:
    note = json.loads((run / "run.json").read_text())
    _, diag = load_diag(run)
    est = diag["estop"] > 0.5
    marks = {e["name"]: e for e in note.get("events", [])}
    checks = [check("trigger_success", bool(marks.get("trigger_estop", {}).get("ok")), True, "==")]
    first = int(np.argmax(est)) if est.any() else None
    if first is None:
        checks.append(check("estop_rows", 0, 1, ">="))
    else:
        after = np.arange(len(est)) >= first
        checks += [
            check("estop_held_to_the_end", int((~est & after).sum()), 0, "=="),
            check("not_waiting_under_estop", int((est & (diag["mode"] != 0)).sum()), 0, "=="),
            check(
                "fault_count_change",
                int(np.nanmax(diag["fault_count"]) - np.nanmin(diag["fault_count"])),
                0,
                "==",
            ),
        ]
    checks.append(check("cm_rejected_lines", cm_rejected_lines(run / "launch.log"), 0, "=="))
    hold = note.get("windows", {}).get("hold", {})
    z = diag["base_pz"][est] if first is not None else np.zeros(0)
    return {
        "run": run.name,
        "pass": all(x["pass"] for x in checks),
        "failed": [x["name"] for x in checks if not x["pass"]],
        "checks": checks,
        "report": {
            "hold_fallen": hold.get("fallen"),
            "hold_time_to_fall_s": hold.get("time_to_fall_s"),
            "hold_tilt_max": hold.get("base_tilt_max"),
            "z_at_trigger": float(z[0]) if z.size else None,
            "z_min_under_estop": float(np.nanmin(z)) if z.size else None,
            "z_end": float(z[-1]) if z.size else None,
            "trigger_t_s": float(diag["t_relative_s"][first]) if first is not None else None,
        },
    }


def probe(run: Path) -> dict:
    _, diag = load_diag(run)
    stand = np.flatnonzero(diag["ref_source"] == wm.REF_SOURCE_STAND)
    after = np.arange(len(diag["tick"])) >= (int(stand[0]) if stand.size else len(diag["tick"]))
    age = diag["policy_age_us"]
    p99 = (
        float(np.percentile(age[after & (age >= 0)], 99)) * 1e-3
        if (after & (age >= 0)).any()
        else None
    )
    counters = mpc_counters(run / "launch.log") or {}
    checks = [
        check("refused_age", counters.get("refused_age"), 0, "=="),
        check(
            "rows_without_a_plan_after_stand", int((after & (diag["plan_id"] == 0)).sum()), 0, "=="
        ),
        check("policy_age_p99_ms", p99, PROBE_AGE_P99_MS, "<="),
    ]
    return {
        "run": run.name,
        "pass": all(x["pass"] for x in checks),
        "checks": checks,
        "mpc": counters,
    }


def run_settings(run: Path, fallback: dict) -> dict:
    """The settings run_arms recorded in run.json (rtf from its ``max_rtf``
    launch argument, None = real time); `fallback` for a key a run recorded
    before run.json carried it."""
    note = json.loads((run / "run.json").read_text())
    rtf = next(
        (
            float(a.split(":=", 1)[1])
            for a in note.get("launch_args", [])
            if a.startswith("max_rtf:=")
        ),
        None,
    )
    return {"rtf": rtf, **{k: note.get(k, fallback[k]) for k in ("z_ref", "stop_window")}}


def runs(out: Path, arm: str) -> list[Path]:
    return sorted(
        p for p in out.glob(f"{arm}_*") if p.is_dir() and p.name[len(arm) + 1 :].isdigit()
    )


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("out", type=Path, help="the run_arms.py --out directory")
    p.add_argument("--gait-yaml", type=Path, required=True, help="the profile's gait table")
    # run.json carries these; the flags only stand in for runs recorded without them.
    p.add_argument("--z-ref", type=float, default=None, help="fallback: run_arms --z-ref [m]")
    p.add_argument(
        "--stop-window", type=float, default=None, help="fallback: run_arms --stop-window [s]"
    )
    p.add_argument("--probe", action="store_true", help="the real-time-factor probe only")
    p.add_argument(
        "--walk-only",
        action="store_true",
        help="judge the walk arm alone (a tuning batch); otherwise walk_estop runs are required",
    )
    args = p.parse_args(argv)

    if args.probe:
        result = {"probe": [probe(r) for r in runs(args.out, "walk")]}
        ok = bool(result["probe"]) and all(r["pass"] for r in result["probe"])
    else:
        gaits = gait_names(args.gait_yaml)
        mpc_runs = runs(args.out, "mpc")
        fallback = {"z_ref": args.z_ref, "stop_window": args.stop_window}
        settings = {
            json.dumps(run_settings(r, fallback), sort_keys=True)
            for r in mpc_runs + runs(args.out, "walk")
        }
        if len(settings) != 1:
            p.error(f"the runs disagree on their settings: {sorted(settings)}")
        st = json.loads(settings.pop())
        if st["z_ref"] is None or st["stop_window"] is None:
            p.error("run.json carries no z_ref / stop_window: pass --z-ref and --stop-window")
        control = (
            control_stop_window(mpc_runs, st["stop_window"], st["rtf"], st["z_ref"])
            if mpc_runs
            else {}
        )
        walk = [judge_walk(r, gaits, control, st["stop_window"]) for r in runs(args.out, "walk")]
        estop = [judge_estop(r) for r in runs(args.out, "walk_estop")]
        result = {
            "settings": st,
            "gaits": gaits,
            "control_stop_window": control,
            "control_runs": [r.name for r in mpc_runs],
            "walk_pass": bool(walk) and all(r["pass"] for r in walk),
            "walk_estop_pass": bool(estop) and all(r["pass"] for r in estop),
            "walk": walk,
            "walk_estop": estop,
        }
        ok = result["walk_pass"] and (args.walk_only or result["walk_estop_pass"])
    name = "walk_probe.json" if args.probe else "walk_gate.json"
    (args.out / name).write_text(json.dumps(result, indent=2, default=_json) + "\n")
    for section in ("probe", "walk", "walk_estop"):
        for r in result.get(section, []):
            print(
                f"{section:10s} {r['run']:14s} {'PASS' if r['pass'] else 'FAIL'} {r.get('failed', '')}"
            )
    return 0 if ok else 1


def _json(x):
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if hasattr(x, "__dataclass_fields__"):
        return asdict(x)
    return str(x)


if __name__ == "__main__":
    sys.exit(main())
