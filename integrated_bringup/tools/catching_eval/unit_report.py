#!/usr/bin/env python3
"""Catching sim evaluation: what ONE unit recorded, read off its logs (E1-F18, #744).

    unit_report.py <unit dir>... [--json out.json]

A unit is one ``run_unit.sh`` output directory. Per unit, as it was recorded —
nothing is judged here:

  * throws — how many, how many reached HOLD, how many entered ABORT_SAFE
    (``trials/trial_results.json``: the modes each throw's ``mode_log`` names)
  * the planner's wakes by cycle outcome, and where replacement attempts ended
    (``planner_events.csv`` ``outcome`` / ``replace_step``)
  * segment solves by kind × outcome × core reason × infeasible row group:
    iterations, QPs, the time of the solves that ENDED, the count of the ones
    the core CUT at its deadline, the QP solver's share of the time, and the
    violation per row group at the returned iterate
  * the NLP search's wakes: its reason, the candidate funnel, what removed the
    candidates
  * the RT segment lane's events (``catching_diag.csv`` ``segment_event``) and
    the RT tick's compute time on the ticks of a mode that runs the segment law
    (``timing/cm_timing_log.csv``)

A solve the core cut at its deadline records the instant it was cut at, not a
solve time. The rule and the table are ``rtc_tools.analysis.planner_solves``;
this tool prints from it and adds what only a unit has. A log from before a
column simply has fewer lines here: columns are read by name.
"""

import argparse
import ast
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd

from rtc_tools.analysis import planner_solves as ps
from rtc_tools.plotting.plotters.catching import SEGMENT_EVENT_NAMES
from rtc_tools.utils.catching_keys import read_csv_normalized

CTRL = "demo_catching_controller"
# Modes in which the RT tick runs the segment law (catching_diag.csv mode_name).
LAW_MODES = ("approach", "committed", "closing", "decel", "hold")


def _read(path: Path, usecols=None) -> pd.DataFrame | None:
    for candidate in (path, path.with_name(path.name + ".gz")):
        if candidate.is_file():
            return read_csv_normalized(pd.read_csv, candidate, usecols=usecols, low_memory=False)
    return None


def throws(trial_results) -> dict:
    """Counts over ``trial_results.json`` (a list, or ``{"trials": [...]}``): the
    throws whose ``mode_log`` names HOLD / ABORT_SAFE, and the recorded outcomes."""
    trials = trial_results["trials"] if isinstance(trial_results, dict) else trial_results
    hold = abort = 0
    outcomes: Counter = Counter()
    for t in trials:
        log = t.get("mode_log")
        if isinstance(log, str):
            log = ast.literal_eval(log)
        modes = {str(m[1]).upper() for m in (log or [])}
        hold += "HOLD" in modes
        abort += "ABORT_SAFE" in modes
        outcomes[str(t.get("final_outcome") or t.get("outcome") or "?")] += 1
    return {"n": len(trials), "hold": hold, "abort_safe": abort, "outcomes": dict(outcomes)}


def lane_events(diag: pd.DataFrame) -> dict:
    """The RT segment lane's events by name (non-zero ``segment_event`` rows)."""
    if "segment_event" not in diag.columns:
        return {}
    ev = pd.to_numeric(diag["segment_event"], errors="coerce").fillna(0).astype(int)
    counts = ev[ev != 0].value_counts().sort_index()
    return {
        (SEGMENT_EVENT_NAMES[k] if 0 <= k < len(SEGMENT_EVENT_NAMES) else str(k)): int(v)
        for k, v in counts.items()
    }


def law_tick_times(diag: pd.DataFrame, timing: pd.DataFrame) -> dict:
    """``t_compute_us`` of the RT ticks whose diag row is in a mode that runs the
    segment law: count and nearest-rank p50 / p99 / p99.9 / max."""
    if "mode_name" not in diag.columns or "tick" not in diag.columns:
        return {"n": 0}
    law = diag["mode_name"].astype(str).str.lower().isin(LAW_MODES)
    ticks = set(pd.to_numeric(diag.loc[law, "tick"], errors="coerce").dropna().astype(np.int64))
    hit = timing[timing["tick_count"].isin(ticks)].drop_duplicates("tick_count")
    v = pd.to_numeric(hit["t_compute_us"], errors="coerce").dropna().to_numpy(float)
    out = {"n": int(v.size), "law_ticks": len(ticks)}
    if v.size:
        out.update(
            p50=ps.nearest_rank(v, 0.5),
            p99=ps.nearest_rank(v, 0.99),
            p999=ps.nearest_rank(v, 0.999),
            max=float(v.max()),
        )
    return out


def planner_account(pe: pd.DataFrame) -> dict:
    """What ``planner_events.csv`` says, through ``planner_solves``."""
    out = {"rows": len(pe)}
    for col in ("outcome", "decision"):
        if col in pe.columns:
            out[col] = {str(k): int(v) for k, v in pe[col].astype(str).value_counts().items()}
    out["solves"] = ps.solve_groups(pe)
    out["nlp"] = ps.nlp_summary(pe)
    out["replace"] = ps.replace_summary(pe)
    return out


def report(unit: Path) -> dict:
    """One unit's report as a dict (see the module docstring for its parts)."""
    unit = Path(unit)
    status = unit / "status"
    out = {"unit": unit.name, "status": status.read_text().strip() if status.is_file() else "?"}
    results = unit / "trials" / "trial_results.json"
    if results.is_file():
        out["throws"] = throws(json.loads(results.read_text()))
    ctl = unit / "session" / "controllers" / CTRL
    pe = _read(ctl / "planner_events.csv")
    if pe is not None:
        out["planner"] = planner_account(pe)
    diag = _read(
        ctl / "catching_diag.csv", usecols=lambda c: c in ("tick", "mode_name", "segment_event")
    )
    if diag is not None:
        out["lane_events"] = lane_events(diag)
        timing = _read(unit / "session" / "timing" / "cm_timing_log.csv")
        if timing is not None and {"tick_count", "t_compute_us"} <= set(timing.columns):
            out["rt_tick_us"] = law_tick_times(diag, timing)
    return out


def _counts(d) -> str:
    return ", ".join(f"{k} {v}" for k, v in d.items()) if d else "-"


def format_report(r: dict) -> list[str]:
    lines = [f"== {r['unit']} ({r['status']})"]
    t = r.get("throws")
    if t:
        lines.append(
            f"throws: {t['n']} | reached HOLD {t['hold']} | entered ABORT_SAFE {t['abort_safe']} "
            f"| outcomes: {_counts(t['outcomes'])}"
        )
    p = r.get("planner")
    if p:
        lines.append(
            f"planner wakes recorded: {p['rows']} | cycle outcome: {_counts(p.get('outcome'))}"
        )
        if p.get("decision"):
            lines.append(f"switching decision: {_counts(p['decision'])}")
        if p.get("replace") is not None:
            lines.append(f"replacement attempts ended: {_counts(p['replace'])}")
        lines.append("segment solves (kind / outcome / core reason / infeasible group):")
        lines += ps.format_solve_groups(p["solves"])
        nlp = p.get("nlp")
        if nlp:
            lines.append(
                f"nlp search wakes: {nlp['wakes']} | reason: {_counts(nlp.get('reasons'))}"
            )
            lines.append(
                "nlp candidates per wake (mean): "
                + ", ".join(f"{k} {v:.1f}" for k, v in nlp["funnel"].items())
            )
            lines.append(f"nlp candidates removed, by reason: {_counts(nlp['rejects'])}")
            times = nlp.get("solve_ms_max")
            if times and times["n"]:
                done = times["done"]
                text = "nlp slowest candidate solve per wake [ms]:"
                if done is not None:
                    text += (
                        f" ended n={times['n_done']} p50 {done['p50']:.2f} p99 {done['p99']:.2f} "
                        f"max {done['max']:.2f}"
                    )
                if times["n_cut"]:
                    text += (
                        f" | wakes that cut a candidate: {times['n_cut']} "
                        f"(>= {times['cut_min_ms']:.2f} ms)"
                    )
                lines.append(text)
    if "lane_events" in r:
        lines.append(f"RT segment lane events: {_counts(r['lane_events'])}")
    tick = r.get("rt_tick_us")
    if tick:
        if tick["n"]:
            lines.append(
                f"RT tick t_compute_us on law ticks (n={tick['n']}): p50 {tick['p50']:.1f} "
                f"p99 {tick['p99']:.1f} p99.9 {tick['p999']:.1f} max {tick['max']:.1f}"
            )
        else:
            lines.append("RT tick t_compute_us on law ticks: n=0")
    return lines


def _json(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return None if not math.isfinite(float(o)) else float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    raise TypeError(type(o).__name__)


def _strict(o):
    """NaN / ±inf → null, tuples → lists: the JSON a strict reader takes."""
    if isinstance(o, dict):
        return {k: _strict(v) for k, v in o.items()}
    if isinstance(o, list | tuple):
        return [_strict(v) for v in o]
    if isinstance(o, float) and not math.isfinite(o):
        return None
    return o


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("units", nargs="+", type=Path, help="run_unit.sh output directories")
    ap.add_argument("--json", type=Path, help="also write every unit's report as JSON")
    args = ap.parse_args(argv)
    reports = [report(u) for u in args.units]
    for r in reports:
        print("\n".join(format_report(r)))
    if args.json:
        args.json.write_text(
            json.dumps(_strict(reports), indent=2, default=_json, allow_nan=False) + "\n"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
