"""Planner-events plotter for the controller-owned planner_events.csv (S6-B).

WHAT ONE FIGURE HAS TO ANSWER. This channel is the planner thread's own
account of each non-idle wake — not the RT tick record `catching_diag.csv`
already covers, but the search that produced (or failed to produce) the plan
the RT tick reads. The question a search-tuning session is read for is "did
the search stay in budget, did it find candidates, and why did the switching
rule do what it did" — so the figure stacks search/IK/rollout timing, publish
latency, the candidate funnel, the judgement-reject histogram, the
outcome/decision events, and the chosen candidate's rank-gate failures on one
time axis.

TIME AXIS. The writer never adds a `t_relative_s`/`t_wall_ns` column to this
file (unlike the RT-tick CSVs) — `wake_ns` is the only clock it carries, so
the time axis here is computed locally as seconds since the first logged
wake rather than through the shared `timestamp` normalization boundary
(io/csv_loader.py). That boundary stays generic to the RT-tick schemas; this
file's derivation lives with its one consumer.

ONE ROW PER NON-IDLE WAKE (see the C++ header comment). At 1 Hz aux-timer
drain and non-uniform wake spacing, a straight line between rows is not "the
value between two wakes" — it is just how far apart the wakes happened to
land. The categorical panels (outcome/decision, rank-gate bits) are plotted
as markers rather than connected lines for that reason; the numeric panels
(search/IK time, latency, funnel, rejects) are still drawn as lines because
that is what every other per-tick/per-event channel in this package does,
and a reader comparing this figure to `catching_diag.csv` benefits from the
same convention.

WHAT IS NOT HERE. There is no search-time budget column in this CSV (the
budget lives in the controller's YAML, not the log), so the search/IK panel
does not draw a budget line the way `plot_timing_breakdown` does — a
threshold with no data behind it would be a guess this module cannot check.
`budget_hit` (whether the search's own budget fired) is a per-row flag and is
covered in `print_planner_events_statistics` instead.
"""

from pathlib import Path

import matplotlib.pyplot as plt

# rtc::catching::CycleOutcomeName order (integrated_bringup/logging/
# planner_events_csv.hpp / rtc_controllers/catching/planner_cycle.hpp). This
# file only logs non-idle wakes (see the C++ header), but `idle` is kept in
# the category order because PlannerEventWorthRecording can still record one
# (a reset or a monitor-only sigma_l on an otherwise idle wake).
OUTCOME_ORDER = ("idle", "no_input", "published", "superseded", "held", "unknown")

# rtc::catching::SwitchDecisionName order (rtc_controllers/catching/
# planner_search.hpp).
DECISION_ORDER = (
    "no_current",
    "replaced",
    "held_hysteresis",
    "held_jump",
    "held_freeze",
    "held_no_candidate",
    "unknown",
)

# Candidate funnel, in the order candidates are narrowed (S6-B decision E).
FUNNEL_COLUMNS = ("n_in_window", "n_ik", "n_pass")

# JudgeReject order (rtc_controllers/catching/planner_search.hpp).
REJECT_COLUMNS = (
    "rej_input",
    "rej_ik",
    "rej_manipulability",
    "rej_workspace",
    "rej_not_evaluated",
)

# RankGateBit order (rtc_controllers/catching/planner_search.hpp). A `1`
# means that gate FAILED for the chosen candidate, not that it passed.
# `rank_rollout` (§4.8) is the whole-interval rollout check: no (gamma, T_w)
# on the grid passed it.
RANK_COLUMNS = (
    "rank_uncertainty",
    "rank_reach",
    "rank_gamma",
    "rank_commit_lead",
    "rank_error_budget",
    "rank_rollout",
)


def _time_axis(df):
    """Seconds since the first logged wake.

    Callers are expected to have already guarded on `wake_ns` being present
    with at least one row (`plot_planner_events` does); this assumes both.
    """
    wake = df["wake_ns"].astype(float)
    return (wake - wake.iloc[0]) / 1e9


def _categorical_codes(series, order):
    """Map string category values to their index in `order`.

    Unrecognised values (schema drift, or a name this module's `order` tuple
    has not caught up with) map to the trailing "unknown" slot rather than
    dropping the row — a plan-search session should never lose an event
    silently because a new decision name was added on the C++ side.
    """
    lookup = {name: i for i, name in enumerate(order)}
    fallback = len(order) - 1
    return series.astype(str).map(lambda v: lookup.get(v, fallback))


def plot_planner_events(df, save_dir=None):
    """Search/IK/rollout timing, publish latency, candidate funnel, rejects,
    outcome/decision events, rank-gate failures — one row per non-idle wake.
    """
    if "wake_ns" not in df.columns:
        print("  Skipping planner events plot (wake_ns not found)")
        return
    if len(df) == 0:
        print("  Skipping planner events plot (no rows)")
        return

    t = _time_axis(df)

    fig, axes = plt.subplots(6, 1, figsize=(14, 18), sharex=True)
    fig.suptitle(
        "Planner Events — search timing, funnel, rejects, decisions, rank gates",
        fontsize=16,
        fontweight="bold",
    )

    # ── 1. Search / IK / rollout timing ─────────────────────────────────────
    for col, colour in (("search_us", "C0"), ("ik_us_max", "C1"), ("rollout_us_max", "C2")):
        if col in df.columns:
            axes[0].plot(t, df[col].astype(float), linewidth=1.0, color=colour, label=col)
    axes[0].set_ylabel("time (µs)")
    axes[0].legend(fontsize=8, loc="upper right")
    axes[0].grid(True, alpha=0.3)

    # ── 2. Receive → publish latency (NaN on every non-published row, from
    #    the writer itself — see planner_events_csv.hpp) ────────────────────
    if "recv_to_publish_ms" in df.columns:
        axes[1].plot(
            t,
            df["recv_to_publish_ms"].astype(float),
            marker=".",
            markersize=4,
            linewidth=0.8,
            color="C3",
            label="recv_to_publish_ms",
        )
        axes[1].legend(fontsize=8, loc="upper right")
    axes[1].set_ylabel("latency (ms)")
    axes[1].grid(True, alpha=0.3)

    # ── 3. Candidate funnel ──────────────────────────────────────────────────
    for col, colour in zip(FUNNEL_COLUMNS, ("C0", "C1", "C2"), strict=False):
        if col in df.columns:
            axes[2].plot(t, df[col].astype(float), linewidth=1.2, color=colour, label=col)
    axes[2].set_ylabel("candidates")
    axes[2].legend(fontsize=8, loc="upper right")
    axes[2].grid(True, alpha=0.3)

    # ── 4. Judgement-reject histogram (stacked) ─────────────────────────────
    available_rejects = [c for c in REJECT_COLUMNS if c in df.columns]
    if available_rejects:
        axes[3].stackplot(
            t,
            *[df[c].astype(float) for c in available_rejects],
            labels=available_rejects,
            alpha=0.7,
        )
        axes[3].legend(fontsize=7, loc="upper right", ncol=2)
    axes[3].set_ylabel("rejects")
    axes[3].grid(True, alpha=0.3)

    # ── 5. Outcome / decision events ────────────────────────────────────────
    n_outcome = len(OUTCOME_ORDER)
    n_decision = len(DECISION_ORDER)
    if "outcome" in df.columns:
        codes = _categorical_codes(df["outcome"], OUTCOME_ORDER)
        axes[4].scatter(t, codes, s=12, color="C0", marker="o", label="outcome")
    if "decision" in df.columns:
        codes = _categorical_codes(df["decision"], DECISION_ORDER) + n_outcome + 1
        axes[4].scatter(t, codes, s=12, color="C1", marker="x", label="decision")
    yticks = list(range(n_outcome)) + [n_outcome + 1 + i for i in range(n_decision)]
    yticklabels = list(OUTCOME_ORDER) + list(DECISION_ORDER)
    axes[4].set_yticks(yticks)
    axes[4].set_yticklabels(yticklabels, fontsize=7)
    axes[4].set_ylabel("event")
    axes[4].legend(fontsize=8, loc="upper right")
    axes[4].grid(True, alpha=0.3)

    # ── 6. Rank-gate failures of the chosen candidate ───────────────────────
    for i, col in enumerate(RANK_COLUMNS):
        if col not in df.columns:
            continue
        failed = df[col].astype(float) > 0.5
        if failed.any():
            axes[5].scatter(
                t[failed], [i] * int(failed.sum()), s=16, color="C3", marker="|", linewidths=2
            )
    axes[5].set_yticks(range(len(RANK_COLUMNS)))
    axes[5].set_yticklabels(RANK_COLUMNS, fontsize=7)
    axes[5].set_ylim(-0.5, len(RANK_COLUMNS) - 0.5)
    axes[5].set_ylabel("rank gate\n(failed)")
    axes[5].set_xlabel("Time (s)")
    axes[5].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "planner_events.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def print_planner_events_statistics(df):
    """Console summary. Counts and rates are the headline, not the figure."""
    print("\n=== Planner Events — per-wake search record ===")
    n = len(df)
    print(f"Samples (non-idle wakes): {n}")
    if n == 0:
        return

    if "outcome" in df.columns:
        counts = df["outcome"].value_counts()
        share = ", ".join(f"{k} {100.0 * v / n:.1f}%" for k, v in counts.items())
        print(f"Outcome: {share}")
    if "decision" in df.columns:
        counts = df["decision"].value_counts()
        share = ", ".join(f"{k} {100.0 * v / n:.1f}%" for k, v in counts.items())
        print(f"Decision: {share}")

    if "search_us" in df.columns:
        us = df["search_us"].astype(float).dropna()
        if len(us) > 0:
            print(
                f"Search time [µs]: median {us.median():.1f}  p99 {us.quantile(0.99):.1f}  "
                f"max {us.max():.1f}"
            )
    if "ik_us_max" in df.columns:
        us = df["ik_us_max"].astype(float).dropna()
        if len(us) > 0:
            print(
                f"IK max time [µs]: median {us.median():.1f}  p99 {us.quantile(0.99):.1f}  "
                f"max {us.max():.1f}"
            )
    if "rollout_us_max" in df.columns:
        us = df["rollout_us_max"].astype(float).dropna()
        if len(us) > 0:
            print(
                f"Rollout max time [µs]: median {us.median():.1f}  p99 {us.quantile(0.99):.1f}  "
                f"max {us.max():.1f}"
            )
    if "n_rollouts" in df.columns:
        nr = df["n_rollouts"].astype(float).dropna()
        if len(nr) > 0:
            print(f"Rollouts run (mean per wake): {nr.mean():.1f}")
    if "budget_hit" in df.columns:
        hits = int((df["budget_hit"].astype(float) > 0.5).sum())
        if hits > 0:
            print(f"Search budget hit: {hits}/{n} wake(s)")

    if "recv_to_publish_ms" in df.columns:
        ms = df["recv_to_publish_ms"].astype(float).dropna()
        if len(ms) > 0:
            print(
                f"Receive→publish latency [ms] (published wakes only, n={len(ms)}): "
                f"median {ms.median():.1f}  p99 {ms.quantile(0.99):.1f}  max {ms.max():.1f}"
            )

    funnel_present = [c for c in FUNNEL_COLUMNS if c in df.columns]
    if funnel_present:
        means = ", ".join(f"{c} {df[c].astype(float).mean():.1f}" for c in funnel_present)
        print(f"Candidate funnel (mean per wake): {means}")

    # `rej_*` are per-wake COUNTS (a search cycle can reject several
    # candidates for the same reason), not booleans — sum them rather than
    # counting the wakes where they fired at all.
    fired = {c: int(df[c].astype(float).sum()) for c in REJECT_COLUMNS if c in df.columns}
    fired = {k: v for k, v in fired.items() if v > 0}
    if fired:
        print("Judgement rejects fired: " + ", ".join(f"{k}×{v}" for k, v in fired.items()))

    gate_fails = {
        c: int((df[c].astype(float) > 0.5).sum()) for c in RANK_COLUMNS if c in df.columns
    }
    gate_fails = {k: v for k, v in gate_fails.items() if v > 0}
    if gate_fails:
        print(
            "Rank-gate failures (chosen candidate): "
            + ", ".join(f"{k}×{v}" for k, v in gate_fails.items())
        )
