"""Prediction-grid sweep of catching sim trials (MPC · dual-arm plan E0-F04, #647).

The sweep runs the v1 planner on several vision prediction grids (horizon ×
point count) and throws the SAME throws in every arm (same seeds), so the arms
are compared pair by pair rather than by their absolute rates. An arm is one
grid condition on one robot: ``--arm NAME UNIT...``, each unit a
``catching_sim_trials`` output with its ``catching_trials`` evaluation
(``<unit>/ct``) and the session (``<unit>/session``).

Per arm:

* truth success of the valid trials, Wilson 95 %, and the ITT count;
* what the controller was told to expect — the read-only mirror's
  ``prediction.dt_expected`` / ``io.n_min`` / ``planner.slice.dt`` from
  ``run_meta.json`` — and what it received: the mode of the diag's
  ``input_n`` over the ticks that took a new snapshot. A unit whose mirror
  lacks any of the three (recorded before the controller mirrored them) is
  refused, since its grid is unknown, unless ``--allow-unknown-grid`` says the
  caller knows what those units ran; a unit whose mirror disagrees with the
  arm's other units is refused (it ran another condition);
* the prediction message as received: the interval between consecutive new
  snapshots within a flight (gaps above ``--flight-gap-s`` separate flights),
  on the diag's ``t_relative_s`` axis, and its size ``point_step × input_n``
  (the decoder refuses any other ``point_step``, so an accepted message has
  exactly :data:`POINT_STEP` bytes per point; the header is not counted);
* the prediction error at the catch instant (``pred_mm``) and the total error
  (``total_mm``), from trials whose ``tc_axis`` is not ``shifted``; the
  contact relative speed; the distribution of ``approach_plan_switches``;
* the planner's cycle time: ``search_us`` of the ``planner_events.csv`` cycles
  that had a candidate in the window, p50 / p99 / max, the budget hits and the
  largest candidate and IK counts. The controller opens that file best-effort,
  so a unit without it is counted (``planner_events_missing``), not refused;
* trials whose ``rtf_trial_min`` is below ``--rtf-min`` — the unit rule
  (D-S8-17) reruns such a unit; this tool only counts them — and trials with
  no RTF value at all (``rtf_unknown``), which are not "fine".

Between arms: trials are paired by ``(kind, seed, sample_idx)``
(:func:`catching_decel.pair_table`). Every arm is compared with ``--ref``, and
``--pair A:B`` adds comparisons. For each comparison the paired difference
``p_B − p_A`` with a Wald 95 % interval for paired proportions, the exact
McNemar p, and the Holm-adjusted p within its family (the comparisons with the
reference are one family, the ``--pair`` comparisons another).

The tool knows no condition and no robot (ARCH-1): the grid comes from the
units' own mirror, the budget from ``--budget-s``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path

import numpy as np

from rtc_tools.analysis import (
    catching_arm_budget as ab,
    catching_decel as cd,
    catching_trials as ct,
)
from rtc_tools.analysis.catching_decel import _stats
from rtc_tools.analysis.catching_hand_near import _num
from rtc_tools.analysis.vision_lane import EXPECTED_POINT_STEP as POINT_STEP

TOOL = "catching_grid_sweep"
FLIGHT_GAP_S = 0.5
RTF_MIN = 0.95
GRID_KEYS = ("prediction.dt_expected", "io.n_min", "planner.slice.dt")
# catching_trials.csv columns carried into the trial rows, converted below.
CT_COLUMNS = (
    "pred_mm",
    "total_mm",
    "tc_axis",
    "contact_v_rel",
    "approach_plan_switches",
    "rtf_trial_min",
)
DIAG = "catching_diag.csv"
PLANNER_EVENTS = "planner_events.csv"


# ── Statistics ───────────────────────────────────────────────────────────────
def paired_difference(pairs: Mapping, z: float = 1.96) -> dict:
    """``p_b − p_a`` of a :func:`catching_decel.pair_table` with a Wald interval.

    Paired proportions: with b = only_a, c = only_b over n pairs the difference
    is ``(c − b) / n`` and its variance ``((b + c) − (c − b)² / n) / n²``.
    """
    n = pairs["n_pairs"]
    if not n:
        return {"diff": math.nan, "ci95": [math.nan, math.nan]}
    b, c = pairs["only_a"], pairs["only_b"]
    d = (c - b) / n
    se = math.sqrt(max((b + c) - (c - b) ** 2 / n, 0.0)) / n
    return {"diff": d, "ci95": [d - z * se, d + z * se]}


def holm(pvalues: Sequence[float]) -> list[float]:
    """Holm step-down adjusted p-values, in the input order (capped at 1)."""
    m = len(pvalues)
    order = sorted(range(m), key=lambda i: pvalues[i])
    out = [math.nan] * m
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (m - rank) * pvalues[i]))
        out[i] = running
    return out


# ── One unit ─────────────────────────────────────────────────────────────────
def _controller_dir(session: Path) -> Path:
    """The one controller directory of the session that holds the catching diag (or its .gz)."""
    root = session / "controllers"
    found = (
        sorted(d for d in root.iterdir() if d.is_dir() and ct._exists(d / DIAG))
        if root.is_dir()
        else []
    )
    if len(found) != 1:
        raise SystemExit(f"{session}: expected one controller with {DIAG}, found {len(found)}")
    return found[0]


def message_stream(diag: Path, flight_gap_s: float = FLIGHT_GAP_S) -> dict:
    """Intervals between new snapshots within a flight, and the points they carried."""
    df = ct._read_csv(diag, usecols=["t_relative_s", "input_new", "input_n"])
    new = df[(df["input_new"].astype(float) > 0) & (df["input_n"].astype(float) > 0)]
    t = new["t_relative_s"].to_numpy(dtype=float)
    n = new["input_n"].to_numpy(dtype=float).astype(int)
    gaps = np.diff(t)
    gaps = gaps[gaps < flight_gap_s]
    values, counts = np.unique(n, return_counts=True) if len(n) else ([], [])
    mode = int(values[int(np.argmax(counts))]) if len(n) else None
    return {
        "n_messages": len(t),
        "interval_ms": _stats(1e3 * gaps),
        "points": {int(v): int(c) for v, c in zip(values, counts, strict=True)},
        "points_mode": mode,
    }


def planner_cycles(events: Path) -> dict | None:
    """``search_us`` of the cycles with a candidate in the window; None without the file."""
    if ct._exists(events) is None:
        return None
    df = ct._read_csv(events, usecols=["search_us", "n_in_window", "n_ik", "budget_hit"])
    busy = df[df["n_in_window"].astype(float) > 0]
    return {
        "search_us": [float(x) for x in busy["search_us"].to_numpy(dtype=float)],
        "budget_hit": int(busy["budget_hit"].astype(float).sum()),
        "n_in_window_max": int(busy["n_in_window"].max()) if len(busy) else 0,
        "n_ik_max": int(busy["n_ik"].max()) if len(busy) else 0,
    }


def analyse_unit(unit: Path, session: Path, flight_gap_s: float = FLIGHT_GAP_S) -> dict:
    trials = cd._trial_table(unit, unit / "ct", extra=CT_COLUMNS)
    meta = json.loads((unit / "trials" / "run_meta.json").read_text())
    mirror = meta.get("controller_mirror") or {}
    ctl = _controller_dir(session)
    for r in trials:
        for k in CT_COLUMNS:
            if k != "tc_axis":
                r[k] = _num(r[k])
    return {
        "unit": str(unit),
        "arm_label": meta.get("arm"),
        "grid": {k: mirror.get(k) for k in GRID_KEYS},
        "trials": trials,
        "stream": message_stream(ctl / DIAG, flight_gap_s),
        "planner": planner_cycles(ctl / PLANNER_EVENTS),
    }


# ── One arm ──────────────────────────────────────────────────────────────────
def summarise_arm(
    name: str,
    units: Sequence[dict],
    rtf_min: float = RTF_MIN,
    budget_s=None,
    allow_unknown_grid: bool = False,
) -> dict:
    unknown = [Path(u["unit"]).name for u in units if any(v is None for v in u["grid"].values())]
    if unknown and not allow_unknown_grid:
        raise SystemExit(
            f"arm {name}: units {unknown} carry no prediction-grid mirror (recorded before the "
            "controller mirrored it?) — their grid is unknown; --allow-unknown-grid accepts them"
        )
    grids = {json.dumps(u["grid"], sort_keys=True) for u in units}
    if len(grids) != 1:
        raise SystemExit(f"arm {name}: its units ran different grids {sorted(grids)}")
    rows = [r for u in units for r in u["trials"]]
    valid = [r for r in rows if not r["invalid_reason"]]
    k = sum(1 for r in valid if r["truth_success"])
    n = len(valid)
    lo, hi = ct.wilson_interval(k, n) if n else (math.nan, math.nan)
    ok_axis = [r for r in valid if r["tc_axis"] != "shifted"]
    switches: dict = {}
    for r in valid:
        v = r["approach_plan_switches"]
        key = "nan" if not math.isfinite(v) else str(int(v))
        switches[key] = switches.get(key, 0) + 1
    planners = [u["planner"] for u in units if u["planner"] is not None]
    search = [x for p in planners for x in p["search_us"]]
    s_stats = _stats(search, (50, 99))
    points: dict = {}
    for u in units:
        for p, c in u["stream"]["points"].items():
            points[p] = points.get(p, 0) + c
    per_unit_intervals = [u["stream"]["interval_ms"] for u in units]
    intervals = {
        "p50_per_unit": [s["p50"] for s in per_unit_intervals],
        "p95_per_unit": [s["p95"] for s in per_unit_intervals],
        "n": sum(s["n"] for s in per_unit_intervals),
    }
    mode = max(points, key=points.get) if points else None
    rtf = [r["rtf_trial_min"] for r in rows]
    return {
        "arm": name,
        "units": [Path(u["unit"]).name for u in units],
        "grid": units[0]["grid"],
        "grid_known": not unknown,
        "n_trials": len(rows),
        "n_valid": n,
        "truth_success": k,
        "rate": k / n if n else math.nan,
        "truth_ci95": [lo, hi],
        "itt": {"success": k, "n": len(rows)},
        "rtf_below_min": sum(1 for x in rtf if math.isfinite(x) and x < rtf_min),
        "rtf_unknown": sum(1 for x in rtf if not math.isfinite(x)),
        "units_input_n_mode": [u["stream"]["points_mode"] for u in units],
        "input_n_mode": mode,
        "message_bytes": None if mode is None else POINT_STEP * mode,
        "message_interval_ms": intervals,
        "pred_mm": _stats([r["pred_mm"] for r in ok_axis]),
        "total_mm": _stats([r["total_mm"] for r in ok_axis]),
        "tc_axis_shifted": len(valid) - len(ok_axis),
        "contact_v_rel": _stats([r["contact_v_rel"] for r in valid]),
        "approach_plan_switches": dict(sorted(switches.items())),
        "planner_events_missing": len(units) - len(planners),
        "planner_search_us": s_stats,
        "planner_budget_hit": sum(p["budget_hit"] for p in planners),
        "planner_over_budget": (
            None if budget_s is None or not s_stats["n"] else bool(s_stats["p99"] > 1e6 * budget_s)
        ),
        "n_in_window_max": max((p["n_in_window_max"] for p in planners), default=0),
        "n_ik_max": max((p["n_ik_max"] for p in planners), default=0),
    }


def outcomes(units: Sequence[dict]) -> dict[tuple, bool]:
    return cd.outcome_map(
        [{"all_trials": u["trials"], "summary": {"unit": u["unit"]}} for u in units]
    )


def compare(arms: Mapping[str, Sequence[dict]], family: Sequence[tuple[str, str]]) -> list[dict]:
    """Paired comparisons ``(a, b)`` of one family, Holm-adjusted together."""
    out = []
    for a, b in family:
        t = cd.pair_table(outcomes(arms[a]), outcomes(arms[b]))
        out.append({"a": a, "b": b, **t, **paired_difference(t)})
    for row, p in zip(out, holm([r["mcnemar_p"] for r in out]), strict=True):
        row["holm_p"] = p
    return out


# ── CLI ──────────────────────────────────────────────────────────────────────
def _f(s: Mapping, keys=("p50", "p95", "max"), nd: int = 1) -> str:
    return "—" if not s["n"] else "/".join(f"{s[k]:.{nd}f}" for k in keys)


def report(summaries: Sequence[dict], families: Mapping[str, list[dict]]) -> str:
    lines = [f"{TOOL}: {len(summaries)} arm(s)"]
    for s in summaries:
        g = s["grid"]
        ci = s["truth_ci95"]
        mi = s["message_interval_ms"]
        grid = (
            f"dt {g['prediction.dt_expected']} n_min {g['io.n_min']} slice {g['planner.slice.dt']}"
            if s["grid_known"]
            else "grid UNKNOWN (no mirror)"
        )
        lines.append(
            f"[{s['arm']}] {grid} · input_n {s['input_n_mode']} (units {s['units_input_n_mode']})"
            f" · {s['truth_success']}/{s['n_valid']} ({s['rate']:.3f}, [{ci[0]:.3f}, {ci[1]:.3f}])"
            f" · ITT {s['itt']['success']}/{s['itt']['n']} · rtf < min {s['rtf_below_min']}"
            f" unknown {s['rtf_unknown']}"
        )
        lines.append(
            f"  message {s['message_bytes']} B · interval ms per unit p50 "
            f"{[round(x, 1) for x in mi['p50_per_unit']]} p95 "
            f"{[round(x, 1) for x in mi['p95_per_unit']]} · pred_mm {_f(s['pred_mm'])} · total_mm "
            f"{_f(s['total_mm'])} (shifted {s['tc_axis_shifted']}) · v_rel {_f(s['contact_v_rel'], nd=2)}"
        )
        lines.append(
            f"  planner search_us p50/p99/max {_f(s['planner_search_us'], ('p50', 'p99', 'max'), 0)}"
            f" · budget hits {s['planner_budget_hit']} · over budget {s['planner_over_budget']}"
            f" · candidates max {s['n_in_window_max']} · IK max {s['n_ik_max']}"
            f" · t_c switches {s['approach_plan_switches']}"
            + (
                f" · planner_events missing in {s['planner_events_missing']} unit(s)"
                if s["planner_events_missing"]
                else ""
            )
        )
    for fam, rows in families.items():
        lines.append(f"[{fam}] paired, Holm within the family")
        for r in rows:
            ci = r["ci95"]
            lines.append(
                f"  {r['b']} − {r['a']}: pairs {r['n_pairs']} · only {r['a']} {r['only_a']} only "
                f"{r['b']} {r['only_b']} · diff {r['diff']:+.3f} [{ci[0]:+.3f}, {ci[1]:+.3f}] · "
                f"McNemar p {r['mcnemar_p']:.3g} · Holm {r['holm_p']:.3g}"
            )
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog=TOOL, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--arm",
        nargs="+",
        action="append",
        required=True,
        metavar=("NAME", "UNIT"),
        help="an arm: its name, then its units (<unit>[:<session>])",
    )
    ap.add_argument("--ref", required=True, help="the arm every other arm is compared with")
    ap.add_argument("--pair", action="append", default=[], help="A:B, an extra comparison (B − A)")
    ap.add_argument(
        "--budget-s", type=float, help="planner.budget_s — flags arms whose p99 exceeds it"
    )
    ap.add_argument("--flight-gap-s", type=float, default=FLIGHT_GAP_S)
    ap.add_argument("--rtf-min", type=float, default=RTF_MIN)
    ap.add_argument(
        "--allow-unknown-grid",
        action="store_true",
        help="accept units whose mirror has no prediction-grid keys (recorded before the "
        "controller mirrored them); the arm is then reported with grid_known false",
    )
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)

    arms: dict[str, list[dict]] = {}
    for spec in args.arm:
        if len(spec) < 2:
            ap.error(f"--arm {spec[0]}: no unit")
        if spec[0] in arms:
            ap.error(f"--arm {spec[0]} given twice")
        arms[spec[0]] = [analyse_unit(*cd.parse_unit_arg(v), args.flight_gap_s) for v in spec[1:]]
    if args.ref not in arms:
        ap.error(f"--ref {args.ref} is not an arm")
    extra = []
    for p in args.pair:
        a, _, b = p.partition(":")
        if a not in arms or b not in arms:
            ap.error(f"--pair {p}: unknown arm")
        extra.append((a, b))
    summaries = [
        summarise_arm(n, u, args.rtf_min, args.budget_s, args.allow_unknown_grid)
        for n, u in arms.items()
    ]
    families = {f"vs {args.ref}": compare(arms, [(args.ref, n) for n in arms if n != args.ref])}
    if extra:
        families["pairs"] = compare(arms, extra)

    args.out.mkdir(parents=True, exist_ok=True)
    doc = {"arms": summaries, "comparisons": families}
    (args.out / "grid_sweep_summary.json").write_text(
        json.dumps(doc, indent=1, default=ab._json_default)
    )
    rows = [
        {"arm": n, "unit": Path(u["unit"]).name, **r}
        for n, units in arms.items()
        for u in units
        for r in u["trials"]
    ]
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with (args.out / "grid_sweep_trials.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, restval="")
        w.writeheader()
        w.writerows(rows)
    print(report(summaries, families))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
