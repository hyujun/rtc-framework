"""Dynamic-catching per-tick diagnostics for the controller-owned catching_diag.csv (S5.4).

WHAT ONE FIGURE HAS TO ANSWER. The question a catching run is read for is
"where was the hand told to go, where did it go, and why did the supervisor
decide what it decided" — and those three are only comparable on one time axis.
So the figure stacks the reference against the realised task position, the
tracking error, the CLIK solve, and the supervisor's mode, sharing x.

EVERY TICK IS A ROW, INCLUDING THE ONES THAT DID NOTHING (PROC-7). A tick that
did not run the law writes zeros with its `*_valid` companion false rather than
repeating the previous tick's numbers, which is exactly why this module filters
on those flags before it averages anything: a run that spent half its ticks
disarmed would otherwise report a reference of zero as a measurement.

THE TWO ACCELERATIONS ARE BOTH PLOTTED ON PURPOSE. `ref_xdd` is what the
reference realised and `ref_u_des` is what it wanted before saturation; a
saturated interval is uninterpretable from either one alone, and `ref_saturated`
alone says it happened without saying by how much.

THE S7 CYCLE (plan §13 S7). Every panel carries the supervisor's mode
transitions as thin vertical lines (derived from the per-tick `mode` column —
there is no separate transition log, L7 §5.2), so an edge can be read against
the reference and the error it happened on. A second figure, `catching_hand`,
stacks the hand sequencer (phase, ρ, timeout) and the fingertip lane (|F − b|,
debounced contact, freshness) with the attempt's verdict.

WHAT IS NOT HERE. There is no ball-truth column in this file — the controller
does not have one — so whether the verdict was RIGHT is not a question this
plot answers. The sim trial analysis owns that (G7-E, plan §8).
"""

from pathlib import Path

import matplotlib.pyplot as plt

# rtc::catching::Mode wire values, in the enum's own order. Used for the mode
# band's y ticks so the trace reads as states rather than as small integers.
MODE_NAMES = (
    "idle",
    "armed",
    "tracking",
    "approach",
    "committed",
    "closing",
    "decel",
    "hold",
    "retreat",
    "abort_safe",
    "fault",
)


# rtc::catching::HandPhase / Outcome wire values (rtc_msgs/CatchingState).
HAND_PHASE_NAMES = ("open", "preshape", "close", "hold", "release")
OUTCOME_NAMES = ("none", "captured", "missed", "undetermined", "aborted")


def mode_transitions(df):
    """(t, from_mode, to_mode) for every tick whose mode differs from the last.

    Derived from the per-tick column rather than logged separately (L7 §5.2):
    the CSV already carries the mode on every row, and a second log would be a
    second account of the same fact that could disagree with the first.
    """
    time_col = "timestamp" if "timestamp" in df.columns else "t_relative_s"
    if "mode" not in df.columns or time_col not in df.columns or len(df) < 2:
        return []
    mode = df["mode"].astype(float)
    t = df[time_col].astype(float)
    changed = mode.ne(mode.shift()).to_numpy()
    changed[0] = False
    out = []
    for i in changed.nonzero()[0]:
        out.append((float(t.iloc[i]), int(mode.iloc[i - 1]), int(mode.iloc[i])))
    return out


def _mode_label(value):
    return MODE_NAMES[value] if 0 <= value < len(MODE_NAMES) else str(value)


def _draw_transitions(axes, transitions, label_axis=None):
    for t_edge, _, to_mode in transitions:
        for ax in axes:
            ax.axvline(t_edge, color="0.6", linewidth=0.6, linestyle="-", alpha=0.7, zorder=0)
        if label_axis is not None:
            label_axis.annotate(
                _mode_label(to_mode),
                xy=(t_edge, 1.0),
                xycoords=("data", "axes fraction"),
                fontsize=6,
                rotation=90,
                va="top",
                ha="right",
                color="0.35",
            )


def _tip_names(df):
    prefix = "tip_force_"
    return [c[len(prefix) :] for c in df.columns if c.startswith(prefix)]


def _live(df):
    """Ticks on which the law actually produced a reference.

    Not `mode == approach`: the mode is the supervisor's intent and the flag is
    what the tick did, and they differ on exactly the ticks worth finding — an
    APPROACH tick whose reference was refused is a tracking failure, not an
    absence of data.
    """
    if "ref_valid" not in df.columns:
        return df
    return df[df["ref_valid"].astype(float) > 0.5]


def _solved(df):
    if "clik_ran" not in df.columns:
        return df
    return df[df["clik_ran"].astype(float) > 0.5]


def _pct(series):
    if len(series) == 0:
        return 0.0
    return float(series.astype(float).mean()) * 100.0


def _masked(df, column, flag):
    """`column` with the rows where `flag` is false replaced by NaN.

    PROC-7 zeroes a block the tick did not compute, so plotting the raw column
    draws a line down to the origin on every disarmed tick — which reads as the
    hand having gone there. NaN breaks the line instead, which is what the row
    means: no value.
    """
    series = df[column].astype(float)
    if flag not in df.columns:
        return series
    return series.where(df[flag].astype(float) > 0.5)


def plot_catching_diag(df, save_dir=None):
    """Reference vs realised, tracking error, solver health, supervisor mode."""
    if "ref_x_x" not in df.columns:
        print("  Skipping catching diag plot (ref_x_x not found)")
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    fig.suptitle(
        "Dynamic Catching — reference, tracking, solver, supervisor",
        fontsize=16,
        fontweight="bold",
    )
    t = df["timestamp"]

    # ── 1. The task reference, per axis, with the plan's catch point ────────
    for axis, colour in zip("xyz", ("C0", "C1", "C2"), strict=True):
        axes[0].plot(
            t,
            _masked(df, f"ref_x_{axis}", "ref_valid"),
            linewidth=1.2,
            color=colour,
            label=f"ref_{axis}",
        )
        pc = f"plan_p_c_{axis}"
        if pc in df.columns:
            # Dashed, because the catch point is where the reference is HEADED
            # rather than a signal the reference is tracking tick by tick.
            axes[0].plot(
                t,
                _masked(df, pc, "plan_valid"),
                linewidth=0.9,
                color=colour,
                linestyle="--",
                label=f"p_c_{axis}",
            )
    axes[0].set_ylabel("task position (m)")
    axes[0].legend(fontsize=7, ncol=3)
    axes[0].grid(True, alpha=0.3)

    # ── 2. Saturation: realised acceleration against the demand ────────────
    for axis, colour in zip("xyz", ("C0", "C1", "C2"), strict=True):
        axes[1].plot(
            t,
            _masked(df, f"ref_xdd_{axis}", "ref_valid"),
            linewidth=1.0,
            color=colour,
            label=f"xdd_{axis}",
        )
        u = f"ref_u_des_{axis}"
        if u in df.columns:
            axes[1].plot(
                t,
                _masked(df, u, "ref_valid"),
                linewidth=0.8,
                color=colour,
                linestyle=":",
                label=f"u_des_{axis}",
            )
    axes[1].set_ylabel("accel (m/s²)")
    axes[1].legend(fontsize=7, ncol=3)
    axes[1].grid(True, alpha=0.3)

    # ── 3. Tracking error and the solve ────────────────────────────────────
    if "track_err_rad" in df.columns:
        axes[2].plot(
            t,
            _masked(df, "track_err_rad", "clik_ran"),
            linewidth=1.2,
            color="C3",
            label="‖q_meas − q_cmd‖ (rad)",
        )
    ax_solve = axes[2].twinx()
    if "clik_solve_us" in df.columns:
        ax_solve.plot(
            t,
            _masked(df, "clik_solve_us", "clik_ran"),
            linewidth=0.8,
            color="C4",
            alpha=0.6,
            label="solve (µs)",
        )
        ax_solve.set_ylabel("solve time (µs)")
    axes[2].set_ylabel("track error (rad)")
    axes[2].legend(fontsize=7, loc="upper left")
    ax_solve.legend(fontsize=7, loc="upper right")
    axes[2].grid(True, alpha=0.3)

    # ── 4. The supervisor ──────────────────────────────────────────────────
    if "mode" in df.columns:
        axes[3].step(t, df["mode"], where="post", linewidth=1.4, color="C5")
        axes[3].set_yticks(range(len(MODE_NAMES)))
        axes[3].set_yticklabels(MODE_NAMES, fontsize=7)
    axes[3].set_ylabel("supervisor mode")
    axes[3].set_xlabel("Time (s)")
    axes[3].grid(True, alpha=0.3)
    _draw_transitions(axes, mode_transitions(df), label_axis=axes[0])

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "catching_diag.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_catching_hand(df, save_dir=None):
    """Hand sequencer and fingertip lane against the mode transitions (S7)."""
    if "hand_phase" not in df.columns:
        print("  Skipping catching hand plot (hand_phase not found)")
        return
    valid = (
        df["hand_phase_valid"].astype(float) > 0.5 if "hand_phase_valid" in df.columns else None
    )
    if valid is not None and not valid.any():
        print("  Skipping catching hand plot (the sequencer never owned the hand)")
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(
        "Dynamic Catching — hand sequencer and fingertips",
        fontsize=16,
        fontweight="bold",
    )
    t = df["timestamp"]

    # ── 1. Phase (NaN while the latch owns the hand) ─────────────────────────
    axes[0].step(t, _masked(df, "hand_phase", "hand_phase_valid"), where="post", color="C0")
    axes[0].set_yticks(range(len(HAND_PHASE_NAMES)))
    axes[0].set_yticklabels(HAND_PHASE_NAMES, fontsize=7)
    axes[0].set_ylabel("hand phase")
    axes[0].grid(True, alpha=0.3)

    # ── 2. Closure ρ, with the timeout flag ─────────────────────────────────
    if "hand_rho" in df.columns:
        axes[1].plot(t, _masked(df, "hand_rho", "hand_phase_valid"), color="C1", label="ρ")
    if "hand_timeout" in df.columns:
        timed_out = df["hand_timeout"].astype(float) > 0.5
        if timed_out.any():
            axes[1].plot(
                t[timed_out],
                df.loc[timed_out, "hand_rho"].astype(float),
                "x",
                color="C3",
                markersize=3,
                label="close timeout",
            )
    axes[1].set_ylim(-0.05, 1.05)
    axes[1].set_ylabel("closure ρ")
    axes[1].legend(fontsize=7)
    axes[1].grid(True, alpha=0.3)

    # ── 3. Fingertip force above the bias; contact ticks marked ─────────────
    for i, name in enumerate(_tip_names(df)):
        colour = f"C{i % 10}"
        axes[2].plot(
            t, df[f"tip_force_{name}"].astype(float), linewidth=1.0, color=colour, label=name
        )
        contact = f"tip_contact_{name}"
        if contact in df.columns:
            on = df[contact].astype(float) > 0.5
            if on.any():
                axes[2].plot(
                    t[on],
                    df.loc[on, f"tip_force_{name}"].astype(float),
                    ".",
                    color=colour,
                    markersize=3,
                )
    axes[2].set_ylabel("|F − b| (N)")
    axes[2].legend(fontsize=7, ncol=4)
    axes[2].grid(True, alpha=0.3)

    # ── 4. Freshness and the verdict ────────────────────────────────────────
    for i, name in enumerate(_tip_names(df)):
        fresh = f"tip_fresh_{name}"
        if fresh in df.columns:
            # Offset per fingertip so the bands do not overlap.
            axes[3].step(
                t,
                df[fresh].astype(float) * 0.8 + i,
                where="post",
                linewidth=0.9,
                color=f"C{i % 10}",
                label=f"{name} fresh",
            )
    if "outcome" in df.columns:
        ax_out = axes[3].twinx()
        ax_out.step(t, df["outcome"].astype(float), where="post", color="k", linewidth=1.3)
        ax_out.set_yticks(range(len(OUTCOME_NAMES)))
        ax_out.set_yticklabels(OUTCOME_NAMES, fontsize=7)
        ax_out.set_ylabel("last attempt")
    axes[3].set_ylabel("fingertip fresh")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(fontsize=6, ncol=4, loc="upper left")
    axes[3].grid(True, alpha=0.3)
    _draw_transitions(axes, mode_transitions(df), label_axis=axes[0])

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "catching_hand.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def print_catching_diag_statistics(df):
    """Console summary. The gate numbers are the headline."""
    print("\n=== Dynamic Catching — per-tick diagnostics ===")
    n = len(df)
    print(f"Samples: {n}")
    if n == 0:
        return

    # One row per RT tick, so a gap is a dropped row and nothing else.
    if "tick" in df.columns and n > 1:
        ticks = df["tick"].dropna().astype("int64")
        if len(ticks) > 1:
            gaps = ticks.diff().iloc[1:]
            dropped = int((gaps - 1).clip(lower=0).sum())
            if dropped > 0:
                print(f"Dropped rows (tick gaps): {dropped} over {len(ticks)} logged ticks")
            if (gaps <= 0).any():
                print("WARNING: tick column is not strictly increasing (concatenated sessions?)")

    if "mode_name" in df.columns:
        counts = df["mode_name"].value_counts()
        share = ", ".join(f"{k} {100.0 * v / n:.1f}%" for k, v in counts.items())
        print(f"Mode occupancy: {share}")
    transitions = mode_transitions(df)
    if transitions:
        path = " → ".join(
            _mode_label(m) for m in [transitions[0][1]] + [e[2] for e in transitions]
        )
        if len(path) > 400:
            path = path[:400] + " …"
        print(f"Mode transitions ({len(transitions)}): {path}")
    if "outcome" in df.columns and transitions:
        # One verdict per attempt: the value on the tick RETREAT is entered —
        # HOLD judges on that edge and every abort sets it there. Counting the
        # column's changes instead would merge two consecutive catches into one.
        retreat = MODE_NAMES.index("retreat")
        verdicts = {}
        t = df["timestamp" if "timestamp" in df.columns else "t_relative_s"].astype(float)
        out = df["outcome"].astype(float)
        for t_edge, _, to_mode in transitions:
            if to_mode != retreat:
                continue
            k = int(out[t == t_edge].iloc[0])
            name = OUTCOME_NAMES[k] if 0 <= k < len(OUTCOME_NAMES) else str(k)
            verdicts[name] = verdicts.get(name, 0) + 1
        if verdicts:
            print("Attempt verdicts: " + ", ".join(f"{k}×{v}" for k, v in verdicts.items()))
    if "reason_name" in df.columns:
        # NONE dominates every healthy run, so it is dropped: what is worth
        # reading is which reasons FIRED and how often.
        fired = df[df["reason_name"] != "none"]["reason_name"].value_counts()
        if len(fired) == 0:
            print("Reasons fired: none")
        else:
            print("Reasons fired: " + ", ".join(f"{k}×{v}" for k, v in fired.items()))

    # ── The input lane ─────────────────────────────────────────────────────
    if "input_stale" in df.columns:
        print(
            f"\nInput stale: {_pct(df['input_stale']):.1f}% of ticks"
            f" | expired: {_pct(df.get('input_expired', df['input_stale'] * 0)):.1f}%"
        )
    if "input_age_s" in df.columns:
        # A NEGATIVE age is the "never received" sentinel, not a measurement --
        # it must never reach a median or a max. The staleness filter below
        # happens to exclude those rows too (an unreceived lane is always
        # stale), but the two are different questions and a reader of this
        # block should not have to know they coincide.
        received = df[df["input_age_s"].astype(float) >= 0.0]
        never = len(df) - len(received)
        if "input_stale" in df.columns:
            fresh = received[received["input_stale"].astype(float) < 0.5]
        else:
            fresh = received
        if never > 0:
            print(f"Input never received on {never} tick(s) ({100.0 * never / len(df):.1f}%)")
        if len(fresh) > 0:
            ages = fresh["input_age_s"].astype(float) * 1e3
            print(
                f"Input age on the receive axis [ms]: "
                f"median {ages.median():.1f}  p99 {ages.quantile(0.99):.1f}  max {ages.max():.1f}"
            )
    if "input_horizon_s" in df.columns:
        usable = df[df["input_valid"].astype(float) > 0.5] if "input_valid" in df.columns else df
        if len(usable) > 0:
            print(
                f"Prediction horizon [s]: min {usable['input_horizon_s'].min():.3f}  "
                f"median {usable['input_horizon_s'].median():.3f}"
            )

    # ── The law ────────────────────────────────────────────────────────────
    live = _live(df)
    print(f"\nTicks with a reference: {len(live)}")
    if len(live) == 0:
        print("  Nothing below is measurable — the law never ran.")
        return
    if "ref_saturated" in live.columns:
        print(f"Reference saturated: {_pct(live['ref_saturated']):.1f}% of those ticks")

    solved = _solved(df)
    if len(solved) > 0 and "clik_solve_us" in solved.columns:
        us = solved["clik_solve_us"].astype(float)
        # The G5-C budget is stated on p99 and max, so those are what is
        # printed — a mean would hide exactly the tail the budget is about.
        print(
            f"CLIK solve [µs]: median {us.median():.1f}  p99 {us.quantile(0.99):.1f}  "
            f"max {us.max():.1f}  (budget p99 ≤ 400, max ≤ 1500)"
        )
    if "clik_converged" in df.columns:
        failed = int(
            (df["clik_ran"].astype(float) > 0.5).sum()
            - (df["clik_converged"].astype(float) > 0.5).sum()
        )
        print(f"CLIK solves that did not converge: {failed}")
    if "clik_bound_conflict" in df.columns:
        conflicts = int((df["clik_bound_conflict"].astype(float) > 0.5).sum())
        if conflicts > 0:
            print(
                f"Bound conflicts: {conflicts} tick(s) — the acceleration box overrode "
                "velocity ∩ position"
            )
    if "qp_fail_streak" in df.columns:
        print(f"Longest QP failure streak: {int(df['qp_fail_streak'].max())}")
    if "track_err_rad" in live.columns:
        err = live["track_err_rad"].astype(float)
        print(f"Tracking error [rad]: median {err.median():.4f}  max {err.max():.4f}")
