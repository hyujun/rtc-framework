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
import numpy as np

# rtc::catching::CycleOutcomeName order (integrated_bringup/logging/
# planner_events_csv.hpp / rtc_controllers/catching/planner_cycle.hpp). This
# file only logs non-idle wakes (see the C++ header), but `idle` is kept in
# the category order because PlannerEventWorthRecording can still record one
# (a reset or a monitor-only sigma_l on an otherwise idle wake).
OUTCOME_ORDER = ("idle", "no_input", "published", "superseded", "held", "unknown")

# rtc::catching::SwitchDecisionName order (rtc_controllers/catching/
# search_stats.hpp).
DECISION_ORDER = (
    "no_current",
    "replaced",
    "held_hysteresis",
    "held_jump",
    "held_freeze",
    "held_no_candidate",
    "refreshed",
    "unknown",
)

# rtc::catching SegmentKindName order (planner_events_csv.hpp). `unknown` is the
# trailing slot `_categorical_codes` maps unrecognised names to.
SEGMENT_KIND_ORDER = ("none", "first", "same", "advance", "stop", "unknown")

# SegmentOutcomeName order (rtc_controllers/src/catching/mpc_segment_planner.cpp), plus
# `not_due`: the stop-only planner's wait, in logs from before it was removed
# (MPC plan MD-70).
SEGMENT_OUTCOME_ORDER = (
    "off",
    "no_state",
    "stale_state",
    "not_due",
    "up_to_date",
    "past_replan_window",
    "input_non_finite",
    "solve_failed",
    "budget",
    "late",
    "slack",
    "ready",
    "published",
    "superseded",
    "not_at_rest",
    "too_late",
    "not_followed",
    "no_ball",
    "catch_error",
    "speed",
    "unknown",
)

# Outcomes of a segment step that solved nothing (SegmentStepWorthRecording in
# planner_events_csv.hpp, with the removed `not_due`).
_SEGMENT_WAITED = ("off", "not_due", "up_to_date", "past_replan_window")

# One colour per solve kind, shared by the segment panels so a kind reads the same
# on all of them. `unknown` is grey: it is a name this module has not caught up with.
_SEGMENT_KIND_COLOURS = {
    "first": "C0",
    "same": "C1",
    "advance": "C2",
    "stop": "C3",
    "none": "0.6",
    "unknown": "0.3",
}

# Candidate funnel, in the order candidates are narrowed (S6-B decision E).
FUNNEL_COLUMNS = ("n_in_window", "n_ik", "n_pass")

# JudgeReject order (rtc_controllers/catching/search_stats.hpp).
REJECT_COLUMNS = (
    "rej_input",
    "rej_ik",
    "rej_manipulability",
    "rej_workspace",
    "rej_not_evaluated",
)

# RankGateBit order (rtc_controllers/catching/grid_catch_search.hpp). A `1`
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


def _segment_kind_series(df):
    """The `segment_kind` names as strings, or None when the column is absent."""
    return df["segment_kind"].astype(str) if "segment_kind" in df.columns else None


def _segment_rows(df):
    """Rows on which the segment planner did something.

    `segment_kind != "none"` is the writer's own statement of that. A log from
    before that column has only `segment_outcome`: there a step that only waited
    (the writer's own "not worth a row" set) is not counted.
    """
    if "segment_kind" in df.columns:
        return df["segment_kind"].astype(str) != "none"
    return ~df["segment_outcome"].astype(str).isin(_SEGMENT_WAITED)


def _segment_panels(df):
    """The segment panels this log has something for. A log from before the
    segment planner has no columns; a `closed_form` session has them and every
    row says the planner was off — neither gets an empty panel."""
    cols = df.columns
    if not ("segment_kind" in cols or "segment_outcome" in cols) or not _segment_rows(df).any():
        return []
    panels = []
    if "segment_outcome" in cols:
        panels.append("outcome")
    if "segment_solve_us" in cols:
        panels.append("solve")
    if "segment_catch_pos_err" in cols:
        panels.append("catch")
    return panels


def _draw_segment_outcome(ax, df, t):
    """Outcome of each segment solve at its category, coloured by solve kind."""
    rows = _segment_rows(df)
    kind = _segment_kind_series(df)
    codes = _categorical_codes(df["segment_outcome"], SEGMENT_OUTCOME_ORDER)
    if kind is None:
        if rows.any():
            ax.scatter(t[rows], codes[rows], s=14, color="C0", marker="o", label="solve")
    else:
        for name in SEGMENT_KIND_ORDER:
            sel = rows & (kind.where(kind.isin(SEGMENT_KIND_ORDER), "unknown") == name)
            if sel.any():
                ax.scatter(
                    t[sel],
                    codes[sel],
                    s=14,
                    color=_SEGMENT_KIND_COLOURS[name],
                    marker="o",
                    label=f"kind: {name}",
                )
    ax.set_yticks(range(len(SEGMENT_OUTCOME_ORDER)))
    ax.set_yticklabels(SEGMENT_OUTCOME_ORDER, fontsize=7)
    ax.set_ylabel("segment outcome")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)


def _draw_segment_solve(ax, df, t):
    """Segment solve time per solve kind (ms), with the iteration count on a twin."""
    solve_ms = df["segment_solve_us"].astype(float) / 1e3
    has_solve = df["segment_solve_us"].astype(float) > 0
    kind = _segment_kind_series(df)
    if kind is None:
        groups = [("solve", has_solve, "C0")]
    else:
        kind = kind.where(kind.isin(SEGMENT_KIND_ORDER), "unknown")
        groups = [
            (name, has_solve & (kind == name), _SEGMENT_KIND_COLOURS[name])
            for name in SEGMENT_KIND_ORDER
        ]
    for name, sel, colour in groups:
        if sel.any():
            ax.scatter(t[sel], solve_ms[sel], s=14, color=colour, marker="o", label=name)
    ax.set_ylabel("segment solve (ms)")
    ax.grid(True, alpha=0.3)
    if "segment_iterations" in df.columns:
        ax_it = ax.twinx()
        # Markers, not a line: a solve between two wakes that solved nothing
        # is one finite value between NaNs, which a line does not draw.
        ax_it.plot(
            t[has_solve],
            df.loc[has_solve, "segment_iterations"].astype(float),
            linestyle="none",
            marker="+",
            markersize=4,
            color="0.4",
            alpha=0.8,
            label="iterations",
        )
        ax_it.set_ylabel("QP iterations")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax_it.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper right")
    elif ax.get_legend_handles_labels()[0]:
        ax.legend(fontsize=8, loc="upper right")


def _draw_segment_catch(ax, df, t):
    """The catch node as each solve left it, and the speed a first solve
    started from.

    Markers only, on the rows that have a value: only a solve with the catch
    terms writes these, so most rows are NaN — a line would draw nothing for a
    value between two NaN rows, and a line through the values would join one
    trial's last solve to the next trial's first. The two errors
    (position in mm, approach axis in degrees) share the left axis; the
    velocity terms (‖v_rel‖ in m/s, γ) and `segment_x0_speed` (rad/s, first
    solves only) the right one.
    """

    def draw(axis, name, scale, label, colour, marker):
        if name not in df.columns:
            return
        values = df[name].astype(float) * scale
        ok = values.notna()
        if ok.any():
            axis.plot(
                t[ok],
                values[ok],
                linestyle="none",
                marker=marker,
                markersize=3.5,
                color=colour,
                label=label,
            )

    draw(ax, "segment_catch_pos_err", 1e3, "catch pos err (mm)", "C0", "o")
    draw(ax, "segment_catch_axis_err", float(np.degrees(1.0)), "catch axis err (deg)", "C2", "s")
    ax.set_ylabel("catch error (mm, deg)")
    ax.grid(True, alpha=0.3)
    ax_r = ax.twinx()
    draw(ax_r, "segment_catch_v_rel", 1.0, "catch ‖v_rel‖ (m/s)", "C1", "o")
    draw(ax_r, "segment_catch_gamma", 1.0, "catch γ", "C4", "x")
    draw(ax_r, "segment_x0_speed", 1.0, "x0 speed (rad/s)", "C3", "^")
    ax_r.set_ylabel("‖v_rel‖ (m/s), γ, x0 speed (rad/s)")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax_r.get_legend_handles_labels()
    if h1 or h2:
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper right", ncol=2)


_SEGMENT_DRAWERS = {
    "outcome": _draw_segment_outcome,
    "solve": _draw_segment_solve,
    "catch": _draw_segment_catch,
}


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

    # The segment panels come after the six search panels and only when their
    # columns exist, so a log from before the segment planner still gets six.
    segment_panels = _segment_panels(df)
    n_panels = 6 + len(segment_panels)
    fig, axes = plt.subplots(n_panels, 1, figsize=(14, 3 * n_panels), sharex=True)
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
    axes[5].grid(True, alpha=0.3)

    # ── 7-9. Segment planner (only the panels whose columns exist) ────────────
    for ax, name in zip(axes[6:], segment_panels, strict=True):
        _SEGMENT_DRAWERS[name](ax, df, t)

    axes[-1].set_xlabel("Time (s)")

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

    _print_segment_statistics(df)


def _print_segment_statistics(df):
    """Per-kind solve record, outcome histogram, publish cadence, search vs plan."""
    n = len(df)
    if "segment_kind" in df.columns:
        kind = df["segment_kind"].astype(str)
        outcome = df["segment_outcome"].astype(str) if "segment_outcome" in df.columns else None
        us = df["segment_solve_us"].astype(float) if "segment_solve_us" in df.columns else None
        for name in SEGMENT_KIND_ORDER[:-1]:
            if name == "none":
                continue
            sel = kind == name
            if not sel.any():
                continue
            line = f"Segment kind {name}: {int(sel.sum())}"
            if outcome is not None:
                line += f", published {int((sel & (outcome == 'published')).sum())}"
            if us is not None:
                ms = us[sel & (us > 0)].dropna() / 1e3
                if len(ms) > 0:
                    line += (
                        f" | solve [ms]: p50 {ms.quantile(0.5):.2f}  "
                        f"p99 {ms.quantile(0.99):.2f}  max {ms.max():.2f}"
                    )
            print(line)
        if outcome is not None:
            active = outcome[kind != "none"]
            if len(active) > 0:
                counts = active.value_counts()
                print(
                    "Segment outcome (kind != none): "
                    + ", ".join(f"{k}×{v}" for k, v in counts.items())
                )
    if "segment_publish_ns" in df.columns:
        pub = df["segment_publish_ns"].astype(float)
        pub = np.sort(pub[pub > 0].unique())
        if len(pub) >= 2:
            print(
                f"Segment publish interval [ms]: p50 {np.median(np.diff(pub)) / 1e6:.1f} "
                f"(n={len(pub)} segments)"
            )
    if "search_valid" in df.columns and "plan_valid" in df.columns:
        sv = int((df["search_valid"].astype(float) > 0.5).sum())
        pv = int((df["plan_valid"].astype(float) > 0.5).sum())
        # Of the wakes that SEARCHED: a replan-only wake (mode mpc, once the RT
        # follows a plan) earns a row and runs no search — its outcome is idle.
        ran = n
        if "outcome" in df.columns:
            ran = int((~df["outcome"].astype(str).isin(("idle", "no_input"))).sum())
        print(
            f"Searches: {ran} of {n} wakes; search_valid {sv}, plan_valid {pv}. The difference "
            f"({sv - pv}) is plans the search found and the wake did not publish — under mode "
            f"mpc a plan goes out only with its first segment."
        )
