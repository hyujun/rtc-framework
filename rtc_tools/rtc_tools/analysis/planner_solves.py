"""
planner_solves.py — the solves a ``planner_events.csv`` records, read without mixing the two kinds

A solve is recorded with the time it took, ``segment_solve_us``. For a solve the
core CUT at its deadline that number is the instant it was cut at, not the time
the solve takes: the true time is at least that, and unknown. A distribution
that mixes the two reads as "the solve takes about its budget" when it does not
finish at all. Everything here keeps them apart, by ONE rule:

    a solve was cut  ⇔  its ``segment_core_reason`` is ``deadline``

The solve's OUTCOME does not say it. ``budget`` is "ended past the budget":
a core without a deadline (the QP planner's) ends past it with a complete
solve, and a core with one may end for another reason — infeasible — a moment
past it; both are times the solve really took.

The same holds for the NLP search's slowest candidate solve, ``nlp_solve_us_max``:
on a wake with ``nlp_rej_deadline`` > 0 it may be a candidate's cut instant.

:func:`solve_groups` is the per-group table (kind × outcome × core reason ×
infeasible row group), :func:`summarise_times` one group's times, and
:func:`quantile_with_bound` a quantile of the two kinds together that says when
it is only a lower bound. The plotter's statistics and the unit report both
print from these.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
import pandas as pd

#: The core reason of a solve that was cut (MpcDockingReasonName(kDeadline)).
CUT_CORE_REASON = "deadline"

#: Row groups of the docking core, in the order of the ``segment_viol_*`` columns
#: (DockingRowGroup); the first seven carry a ``segment_elastic_*`` column.
DOCKING_ROW_GROUPS = (
    "torque",
    "gap",
    "entrance",
    "lateral",
    "timing",
    "velocity_set",
    "impact",
    "box",
    "terminal",
)
DOCKING_ELASTIC_GROUPS = DOCKING_ROW_GROUPS[:7]

#: A group's violation at or below this is rounding, not a violated row: the
#: linear rows read ~1e-9 at an iterate that satisfies them. It is the docking
#: core's default tolerance (``tol_violation``) and only decides what
#: :func:`solve_groups` LISTS — the numbers themselves are the log's.
VIOLATION_FLOOR = 1e-6

#: Stages of a docking solve, in the order of the ``segment_*_us`` columns.
SOLVE_STAGES = ("start", "linearize", "assemble", "qp", "merit")

#: Candidate reasons of the NLP search, in the order of the ``nlp_rej_*`` columns
#: (NlpReject between ``none`` and the wake-only reasons).
NLP_REJECT_REASONS = (
    "follow_window",
    "lead_short",
    "ball_invalid",
    "workspace",
    "covariance",
    "no_source",
    "ik",
    "manipulability",
    "reach",
    "speed_window",
    "not_ranked",
    "deadline",
    "solver_rejected",
    "hard_row",
    "chance",
    "unconverged",
)

#: ReplaceStepName, in enum order.
REPLACE_STEPS = (
    "none",
    "too_late_followed",
    "too_late_new",
    "withheld",
    "superseded",
    "published",
)

_GROUP_KEYS = (
    "segment_kind",
    "segment_outcome",
    "segment_core_reason",
    "segment_infeasible_group",
)


def solved_rows(df: pd.DataFrame) -> np.ndarray:
    """Rows whose segment step ran a solve: a positive ``segment_solve_us``.

    A step that waited, or was refused before the core was called, records 0.
    """
    if "segment_solve_us" not in df.columns:
        return np.zeros(len(df), bool)
    us = pd.to_numeric(df["segment_solve_us"], errors="coerce").to_numpy(float)
    return np.isfinite(us) & (us > 0)


def cut_at_deadline(df: pd.DataFrame) -> np.ndarray:
    """Solved rows whose time is the instant the core was cut at (module docstring).

    All false on a log without ``segment_core_reason`` — nothing can be said to
    have been cut.
    """
    if "segment_core_reason" not in df.columns:
        return np.zeros(len(df), bool)
    reason = df["segment_core_reason"].astype(str).to_numpy()
    return solved_rows(df) & (reason == CUT_CORE_REASON)


def nearest_rank(values: Sequence[float], q: float) -> float:
    """The nearest-rank ``q`` quantile (an observed value); NaN of nothing."""
    v = np.sort(np.asarray(values, float))
    if v.size == 0:
        return math.nan
    return float(v[min(v.size - 1, max(0, math.ceil(q * v.size) - 1))])


def quantile_with_bound(values: Sequence[float], cut: Sequence[bool], q: float):
    """``(value, is_lower_bound)`` — the ``q`` quantile of solve times of which
    some were cut.

    The value is the nearest-rank quantile with every cut time entered as it
    was recorded. A cut solve's true time is at least its recorded one, so
    replacing each by the truth can only move order statistics up: the value is
    a lower bound of the true quantile. It IS the true quantile when it lies
    below the earliest cut instant — every sample at or below it then ran to
    its end, and no cut solve can belong below it.
    """
    v = np.asarray(values, float)
    c = np.asarray(cut, bool)
    if v.size == 0:
        return math.nan, False
    value = nearest_rank(v, q)
    return value, bool(c.any() and value >= float(v[c].min()))


def summarise_times(ms: Sequence[float], cut: Sequence[bool]) -> dict:
    """One group's solve times [ms], the two kinds apart.

    ``done`` is the distribution of the solves that ran to their end (``None``
    with none). The cut ones get a count and their earliest cut instant — no
    distribution: where a cut solve would have ended is not in the log.
    ``p50`` / ``p99`` are over both kinds, each a ``(value, is_lower_bound)``
    pair (:func:`quantile_with_bound`).
    """
    v = np.asarray(ms, float)
    c = np.asarray(cut, bool)
    done = v[~c]
    out = {
        "n": int(v.size),
        "n_done": int(done.size),
        "n_cut": int(c.sum()),
        "done": None,
        "cut_min_ms": float(v[c].min()) if c.any() else None,
        "cut_max_ms": float(v[c].max()) if c.any() else None,
        "p50": quantile_with_bound(v, c, 0.5),
        "p99": quantile_with_bound(v, c, 0.99),
    }
    if done.size:
        out["done"] = {
            "min": float(done.min()),
            "p50": nearest_rank(done, 0.5),
            "p90": nearest_rank(done, 0.9),
            "p99": nearest_rank(done, 0.99),
            "max": float(done.max()),
        }
    return out


def _numeric(df: pd.DataFrame, name: str) -> np.ndarray | None:
    if name not in df.columns:
        return None
    return pd.to_numeric(df[name], errors="coerce").to_numpy(float)


def _median(values: np.ndarray) -> float | None:
    values = values[np.isfinite(values)]
    return float(np.median(values)) if values.size else None


def solve_groups(df: pd.DataFrame) -> list[dict]:
    """The solves of a log, grouped by what they were and how they ended.

    One entry per (``segment_kind``, ``segment_outcome``, ``segment_core_reason``,
    ``segment_infeasible_group``) among :func:`solved_rows`, largest first. A key
    the log lacks groups as ``""`` — a log from before a column simply has
    coarser groups. Each entry carries the count, the iteration and QP counts
    (medians; ``None`` without the column), :func:`summarise_times`, the QP
    solver's share of the stage times (median over the group's solves), and
    the violation per row group (median and max over the group, only groups
    some solve violated by more than :data:`VIOLATION_FLOOR`).
    """
    rows = solved_rows(df)
    if not rows.any():
        return []
    sub = df.loc[rows]
    cut = cut_at_deadline(df)[rows]
    ms = pd.to_numeric(sub["segment_solve_us"], errors="coerce").to_numpy(float) / 1e3
    keys = pd.DataFrame(
        {
            k: (sub[k].astype(str).to_numpy() if k in sub.columns else np.full(len(sub), ""))
            for k in _GROUP_KEYS
        }
    )
    iterations = _numeric(sub, "segment_iterations")
    qp_solves = _numeric(sub, "segment_qp_solves")
    qp_iterations = _numeric(sub, "segment_qp_iterations")
    stages = {s: _numeric(sub, f"segment_{s}_us") for s in SOLVE_STAGES}
    have_stages = all(v is not None for v in stages.values())
    out = []
    for key, idx in keys.groupby(list(_GROUP_KEYS), sort=False).indices.items():
        idx = np.asarray(idx)
        entry = dict(zip(("kind", "outcome", "core_reason", "infeasible_group"), key, strict=True))
        entry["n"] = int(idx.size)
        entry["iterations_p50"] = None if iterations is None else _median(iterations[idx])
        entry["iterations_max"] = (
            None
            if iterations is None or not np.isfinite(iterations[idx]).any()
            else float(np.nanmax(iterations[idx]))
        )
        entry["qp_solves_p50"] = None if qp_solves is None else _median(qp_solves[idx])
        entry["qp_iterations_per_qp_p50"] = None
        if qp_solves is not None and qp_iterations is not None:
            with np.errstate(divide="ignore", invalid="ignore"):
                per = qp_iterations[idx] / qp_solves[idx]
            entry["qp_iterations_per_qp_p50"] = _median(per[qp_solves[idx] > 0])
        entry["time_ms"] = summarise_times(ms[idx], cut[idx])
        entry["qp_share_p50"] = None
        if have_stages:
            total = sum(stages[s][idx] for s in SOLVE_STAGES)
            with np.errstate(divide="ignore", invalid="ignore"):
                share = stages["qp"][idx] / total
            entry["qp_share_p50"] = _median(share[total > 0])
        violation = {}
        for g in DOCKING_ROW_GROUPS:
            v = _numeric(sub, f"segment_viol_{g}")
            if v is None:
                continue
            v = v[idx]
            v = v[np.isfinite(v)]
            if v.size and v.max() > VIOLATION_FLOOR:
                violation[g] = {"p50": float(np.median(v)), "max": float(v.max())}
        entry["violation"] = violation
        out.append(entry)
    out.sort(key=lambda e: -e["n"])
    return out


def _fmt_bound(pair) -> str:
    value, bound = pair
    if not math.isfinite(value):
        return "-"
    return f"{'>=' if bound else ''}{value:.2f}"


def format_solve_groups(groups: Sequence[dict]) -> list[str]:
    """:func:`solve_groups` as text lines. A cut solve's time is never printed
    as a time: the ``done`` columns are of the solves that ended, ``cut`` is a
    count with the earliest cut instant, and a quantile over both carries
    ``>=`` when it is only a lower bound."""
    if not groups:
        return ["  (no solve recorded)"]
    lines = [
        f"  {'kind':8} {'outcome':13} {'core_reason':16} {'infeasible':13} {'n':>5} "
        f"{'it p50':>6} {'QP p50':>6} {'done':>5} {'done ms p50/p90/max':>22} {'cut':>5} "
        f"{'cut >= ms':>9} {'all p50':>9} {'all p99':>9} {'QP share':>8}"
    ]
    for g in groups:
        t = g["time_ms"]
        done = t["done"]
        done_txt = (
            f"{done['p50']:.2f}/{done['p90']:.2f}/{done['max']:.2f}" if done is not None else "-"
        )
        cut_txt = f"{t['cut_min_ms']:.2f}" if t["cut_min_ms"] is not None else "-"
        it = "-" if g["iterations_p50"] is None else f"{g['iterations_p50']:.0f}"
        qp = "-" if g["qp_solves_p50"] is None else f"{g['qp_solves_p50']:.0f}"
        share = "-" if g["qp_share_p50"] is None else f"{100.0 * g['qp_share_p50']:.0f}%"
        lines.append(
            f"  {g['kind']:8} {g['outcome']:13} {g['core_reason']:16} "
            f"{g['infeasible_group'] or '-':13} {g['n']:5d} {it:>6} {qp:>6} {t['n_done']:5d} "
            f"{done_txt:>22} {t['n_cut']:5d} {cut_txt:>9} {_fmt_bound(t['p50']):>9} "
            f"{_fmt_bound(t['p99']):>9} {share:>8}"
        )
        if g["violation"]:
            parts = ", ".join(
                f"{name} p50 {v['p50']:.3g} max {v['max']:.3g}"
                for name, v in g["violation"].items()
            )
            lines.append(f"      violation at the returned iterate: {parts}")
    if any(g["time_ms"]["n_cut"] for g in groups):
        lines.append(
            "  'cut' solves were stopped at the core's deadline: their recorded time is the cut "
            "instant, a lower bound ('>=') of the time the solve takes."
        )
    return lines


def nlp_summary(df: pd.DataFrame) -> dict | None:
    """The NLP search's wakes of a log; ``None`` when it has none (an older
    log, or a session of another search).

    ``reasons`` counts the wakes by ``nlp_reason``; ``funnel`` is the mean
    candidate count per wake at each stage; ``rejects`` sums the candidates
    each check removed over the wakes. ``solve_ms_max`` summarises
    ``nlp_solve_us_max`` with the wakes that cut a candidate
    (``nlp_rej_deadline`` > 0) kept apart, as :func:`summarise_times` does.
    """
    if "nlp_ran" not in df.columns:
        return None
    ran = pd.to_numeric(df["nlp_ran"], errors="coerce").to_numpy(float) > 0.5
    if not ran.any():
        return None
    sub = df.loc[ran]
    out = {"wakes": int(ran.sum())}
    if "nlp_reason" in sub.columns:
        out["reasons"] = {
            str(k): int(v) for k, v in sub["nlp_reason"].astype(str).value_counts().items()
        }
    funnel = {}
    for name in ("nlp_n_lattice", "nlp_n_screened", "nlp_n_solved", "nlp_n_valid"):
        v = _numeric(sub, name)
        if v is not None:
            funnel[name] = float(np.nanmean(v))
    out["funnel"] = funnel
    rejects = {}
    for reason in NLP_REJECT_REASONS:
        v = _numeric(sub, f"nlp_rej_{reason}")
        if v is not None and np.nansum(v) > 0:
            rejects[reason] = int(np.nansum(v))
    out["rejects"] = rejects
    us = _numeric(sub, "nlp_solve_us_max")
    cut = _numeric(sub, "nlp_rej_deadline")
    if us is not None:
        solved = np.isfinite(us) & (us > 0)
        was_cut = (cut > 0) if cut is not None else np.zeros(len(sub), bool)
        out["solve_ms_max"] = summarise_times(us[solved] / 1e3, was_cut[solved])
    return out


def replace_summary(df: pd.DataFrame) -> dict | None:
    """Where the wakes' replacements ended (``replace_step``), counted; ``None``
    on a log without the column. Wakes that attempted none are left out."""
    if "replace_step" not in df.columns:
        return None
    steps = df["replace_step"].astype(str)
    counts = steps[steps != "none"].value_counts()
    return {str(k): int(v) for k, v in counts.items()}
