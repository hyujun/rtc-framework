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

A wake whose replacement was withheld records TWO solves: the replan in the
``segment_*`` columns and the replacement's first solve in ``replacement_*``.
Both are solves here (:func:`solve_table`), the second under the kind
:data:`WITHHELD_REPLACEMENT_KIND`.

The NLP search's slowest candidate solve, ``nlp_solve_us_max``, cannot be sorted
the same way from a wake's row. ``nlp_rej_deadline`` > 0 says a candidate was
rejected for the deadline, and that reason covers a solve the core cut, a
solve that ended past its deadline, and a valid one the wake finished too
late for. On such a wake the value MAY be a cut instant; it is kept apart from
the wakes where it is certainly a solve time, and never called one.

:func:`solve_groups` is the per-group table (kind × outcome × core reason ×
infeasible row group), :func:`summarise_times` one group's times, and
:func:`quantile_with_bound` a quantile of the two kinds together that says when
it is only a lower bound — for a set that mixes the two kinds, which a group
of :func:`solve_groups` never does (the core reason is one of its keys). The
plotter's statistics, the unit report and ``catching_trials`` read from these.
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
    "covariance",
    "no_source",
    "too_far",
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

#: Candidate reasons the search no longer has, still in the ``nlp_rej_*``
#: columns and the ``nlp_reason`` of sessions recorded while it had them:
#: ``workspace`` — the catch box, removed 2026-10-09 (L3 §4.9).
NLP_RETIRED_REJECT_REASONS = ("workspace",)

#: ReplaceStepName, in enum order.
REPLACE_STEPS = (
    "none",
    "too_late_followed",
    "too_late_new",
    "withheld",
    "superseded",
    "published",
)

#: The ``kind`` :func:`solve_table` gives the first solve of a replacement that
#: was withheld (the ``replacement_*`` columns). It is a first solve; it is
#: named apart because the log has only its outcome, reason, iterations and time.
WITHHELD_REPLACEMENT_KIND = "repl_first"

_GROUP_KEYS = ("kind", "outcome", "core_reason", "infeasible_group")


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


def replacement_solved_rows(df: pd.DataFrame) -> np.ndarray:
    """Rows that carry the first solve of a withheld replacement: a positive
    ``replacement_solve_us`` (all false on a log without the column)."""
    if "replacement_solve_us" not in df.columns:
        return np.zeros(len(df), bool)
    us = pd.to_numeric(df["replacement_solve_us"], errors="coerce").to_numpy(float)
    return np.isfinite(us) & (us > 0)


def replacement_cut_at_deadline(df: pd.DataFrame) -> np.ndarray:
    """:func:`cut_at_deadline` for the withheld replacement's first solve."""
    if "replacement_core_reason" not in df.columns:
        return np.zeros(len(df), bool)
    reason = df["replacement_core_reason"].astype(str).to_numpy()
    return replacement_solved_rows(df) & (reason == CUT_CORE_REASON)


def first_solves_cut(df: pd.DataFrame) -> np.ndarray:
    """Per row, how many FIRST solves the core cut at its deadline: the wake's
    own (``segment_kind`` ``first``) and a withheld replacement's."""
    own = cut_at_deadline(df)
    if "segment_kind" in df.columns:
        own = own & (df["segment_kind"].astype(str).to_numpy() == "first")
    return own.astype(int) + replacement_cut_at_deadline(df).astype(int)


def _text(df: pd.DataFrame, name: str) -> np.ndarray:
    if name not in df.columns:
        return np.full(len(df), "", dtype=object)
    return df[name].astype(str).to_numpy(dtype=object)


def _numeric(df: pd.DataFrame, name: str) -> np.ndarray:
    """The column as floats; all-NaN when the log does not have it."""
    if name not in df.columns:
        return np.full(len(df), np.nan)
    return pd.to_numeric(df[name], errors="coerce").to_numpy(float)


#: Numeric columns of a solve :func:`solve_table` carries over, as
#: ``(table name, segment column)``.
_SOLVE_VALUES = (
    ("iterations", "segment_iterations"),
    ("qp_solves", "segment_qp_solves"),
    ("qp_iterations", "segment_qp_iterations"),
    *((f"{s}_us", f"segment_{s}_us") for s in ("start", "linearize", "assemble", "qp", "merit")),
    *((f"viol_{g}", f"segment_viol_{g}") for g in DOCKING_ROW_GROUPS),
)


def solve_table(df: pd.DataFrame) -> pd.DataFrame:
    """Every solve the log records, one row each.

    The wake's segment solve (:func:`solved_rows`) and, on a wake whose
    replacement was withheld, that replacement's first solve
    (:func:`replacement_solved_rows`, kind :data:`WITHHELD_REPLACEMENT_KIND`).
    Columns: ``kind`` · ``outcome`` · ``core_reason`` · ``infeasible_group``
    (``""`` where the log has no such column), ``ms``, ``cut``, and the
    numeric account (``iterations``, ``qp_solves``, ``qp_iterations``, the
    five ``*_us`` stages, ``viol_<group>``) — NaN where the log does not have
    the value, which is every docking value of a withheld replacement.
    """
    own = solved_rows(df)
    table = pd.DataFrame(
        {
            "kind": _text(df, "segment_kind")[own],
            "outcome": _text(df, "segment_outcome")[own],
            "core_reason": _text(df, "segment_core_reason")[own],
            "infeasible_group": _text(df, "segment_infeasible_group")[own],
            "ms": _numeric(df, "segment_solve_us")[own] / 1e3,
            "cut": cut_at_deadline(df)[own],
            **{name: _numeric(df, column)[own] for name, column in _SOLVE_VALUES},
        }
    )
    # The writer leaves a COUNT at 0 on a row whose planner did not fill the
    # docking block; a docking solve that reached an iterate has solved a QP.
    # 0 QPs is "not recorded", not a number of QPs.
    unrecorded = ~(table["qp_solves"].to_numpy(float) > 0)
    table.loc[unrecorded, ["qp_solves", "qp_iterations"]] = np.nan
    other = replacement_solved_rows(df)
    if not other.any():
        return table
    withheld = pd.DataFrame(
        {
            "kind": np.full(int(other.sum()), WITHHELD_REPLACEMENT_KIND, dtype=object),
            "outcome": _text(df, "replacement_outcome")[other],
            "core_reason": _text(df, "replacement_core_reason")[other],
            "infeasible_group": np.full(int(other.sum()), "", dtype=object),
            "ms": _numeric(df, "replacement_solve_us")[other] / 1e3,
            "cut": replacement_cut_at_deadline(df)[other],
            "iterations": _numeric(df, "replacement_iterations")[other],
        }
    )
    return pd.concat([table, withheld], ignore_index=True)


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


def _median(values: np.ndarray) -> float | None:
    values = values[np.isfinite(values)]
    return float(np.median(values)) if values.size else None


def solve_groups(df: pd.DataFrame) -> list[dict]:
    """The solves of a log, grouped by what they were and how they ended.

    One entry per (``kind``, ``outcome``, ``core_reason``, ``infeasible_group``)
    of :func:`solve_table`, largest first. A key the log lacks groups as ``""``
    — a log from before a column simply has coarser groups. Each entry carries
    the count, the iteration and QP counts (medians; ``None`` where the log has
    no value), :func:`summarise_times`, the QP solver's share of the stage
    times (median over the group's solves), and the violation per row group
    (median and max over the group, only groups some solve violated by more
    than :data:`VIOLATION_FLOOR`). A group is all cut or all ended: the core
    reason is a key.
    """
    table = solve_table(df)
    if table.empty:
        return []
    stage_total = sum(table[f"{s}_us"].to_numpy(float) for s in SOLVE_STAGES)
    with np.errstate(divide="ignore", invalid="ignore"):
        qp_share = table["qp_us"].to_numpy(float) / stage_total
        per_qp = table["qp_iterations"].to_numpy(float) / table["qp_solves"].to_numpy(float)
    qp_share = np.where(stage_total > 0, qp_share, np.nan)
    per_qp = np.where(table["qp_solves"].to_numpy(float) > 0, per_qp, np.nan)
    iterations = table["iterations"].to_numpy(float)
    qp_solves = table["qp_solves"].to_numpy(float)
    ms = table["ms"].to_numpy(float)
    cut = table["cut"].to_numpy(bool)
    violations = {g: table[f"viol_{g}"].to_numpy(float) for g in DOCKING_ROW_GROUPS}
    out = []
    for key, idx in table.groupby(list(_GROUP_KEYS), sort=False).indices.items():
        idx = np.asarray(idx)
        entry = dict(zip(_GROUP_KEYS, key, strict=True))
        entry["n"] = int(idx.size)
        entry["iterations_p50"] = _median(iterations[idx])
        finite = iterations[idx][np.isfinite(iterations[idx])]
        entry["iterations_max"] = float(finite.max()) if finite.size else None
        entry["qp_solves_p50"] = _median(qp_solves[idx])
        entry["qp_iterations_per_qp_p50"] = _median(per_qp[idx])
        entry["time_ms"] = summarise_times(ms[idx], cut[idx])
        entry["qp_share_p50"] = _median(qp_share[idx])
        violation = {}
        for g, values in violations.items():
            v = values[idx]
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
    count with the earliest cut instant."""
    if not groups:
        return ["  (no solve recorded)"]
    lines = [
        f"  {'kind':10} {'outcome':13} {'core_reason':16} {'infeasible':13} {'n':>5} "
        f"{'it p50':>6} {'QP p50':>6} {'done':>5} {'done ms p50/p90/max':>22} {'cut':>5} "
        f"{'cut >= ms':>9} {'QP share':>8}"
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
            f"  {g['kind']:10} {g['outcome']:13} {g['core_reason']:16} "
            f"{g['infeasible_group'] or '-':13} {g['n']:5d} {it:>6} {qp:>6} {t['n_done']:5d} "
            f"{done_txt:>22} {t['n_cut']:5d} {cut_txt:>9} {share:>8}"
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
            "instant, a lower bound of the time the solve takes."
        )
    if any(g["kind"] == WITHHELD_REPLACEMENT_KIND for g in groups):
        lines.append(
            f"  '{WITHHELD_REPLACEMENT_KIND}' is the first solve of a replacement that was "
            "withheld (the replacement_* columns)."
        )
    return lines


def format_time_summary(times: dict) -> str:
    """One :func:`summarise_times` as a phrase for a set that may MIX the two
    kinds: the solves that ended, the cut ones as a count, and — only when
    both are present — the quantiles over both, ``>=`` where they are bounds."""
    parts = []
    done = times["done"]
    if done is not None:
        parts.append(
            f"solve [ms] (n={times['n_done']}): p50 {done['p50']:.2f}  p99 {done['p99']:.2f}  "
            f"max {done['max']:.2f}"
        )
    if times["n_cut"]:
        parts.append(
            f"cut at the deadline: {times['n_cut']} "
            f"(>= {times['cut_min_ms']:.2f} ms, not a solve time)"
        )
    if done is not None and times["n_cut"]:
        parts.append(
            f"over both (n={times['n']}): p50 {_fmt_bound(times['p50'])}  "
            f"p99 {_fmt_bound(times['p99'])}"
        )
    return " | ".join(parts)


def nlp_summary(df: pd.DataFrame) -> dict | None:
    """The NLP search's wakes of a log; ``None`` when it has none (an older
    log, or a session of another search).

    ``reasons`` counts the wakes by ``nlp_reason``; ``funnel`` is the mean
    candidate count per wake at each stage; ``rejects`` sums the candidates
    each check removed over the wakes. ``solve_ms_max`` is
    :func:`summarise_times` of ``nlp_solve_us_max`` in which "cut" means a
    wake with ``nlp_rej_deadline`` > 0 — where the value MAY be a cut instant
    (module docstring): its ``done`` distribution is of the wakes where the
    value is certainly a solve time, and it leaves out slow solves that ended
    on a wake with such a reject.
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
    out["funnel"] = {
        name: float(np.nanmean(_numeric(sub, name)))
        for name in ("nlp_n_lattice", "nlp_n_screened", "nlp_n_solved", "nlp_n_valid")
        if name in sub.columns
    }
    rejects = {}
    for reason in (*NLP_REJECT_REASONS, *NLP_RETIRED_REJECT_REASONS):
        total = np.nansum(_numeric(sub, f"nlp_rej_{reason}"))
        if total > 0:
            rejects[reason] = int(total)
    out["rejects"] = rejects
    if "nlp_solve_us_max" in sub.columns:
        us = _numeric(sub, "nlp_solve_us_max")
        solved = np.isfinite(us) & (us > 0)
        deadline = _numeric(sub, "nlp_rej_deadline") > 0
        out["solve_ms_max"] = summarise_times(us[solved] / 1e3, deadline[solved])
    return out


def format_nlp_solve_time(times: dict) -> str:
    """``nlp_summary()["solve_ms_max"]`` as a phrase — the wakes with a
    deadline reject are a count, and are not called cut: they may be."""
    parts = []
    done = times["done"]
    if done is not None:
        parts.append(
            f"no deadline reject (n={times['n_done']}): p50 {done['p50']:.2f}  "
            f"p99 {done['p99']:.2f}  max {done['max']:.2f}"
        )
    if times["n_cut"]:
        parts.append(
            f"wakes with a deadline-rejected candidate: {times['n_cut']} "
            f"(>= {times['cut_min_ms']:.2f} ms — that value may be a cut instant)"
        )
    return " | ".join(parts)


def replace_summary(df: pd.DataFrame) -> dict | None:
    """Where the wakes' replacements ended (``replace_step``), counted; ``None``
    on a log without the column. Wakes that attempted none are left out."""
    if "replace_step" not in df.columns:
        return None
    steps = df["replace_step"].astype(str)
    counts = steps[steps != "none"].value_counts()
    return {str(k): int(v) for k, v in counts.items()}
