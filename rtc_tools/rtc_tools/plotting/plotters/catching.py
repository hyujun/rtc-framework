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

WHAT IS NOT HERE. There is no ball-truth column in this file — the controller
does not have one — so "did it catch" is not a question this plot answers. The
offline evaluation scripts own that (plan §8).
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

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "catching_diag.png"
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
