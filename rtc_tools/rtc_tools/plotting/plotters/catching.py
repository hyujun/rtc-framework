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

THE S7 CYCLE (L8 §11). Every panel carries the supervisor's mode
transitions as thin vertical lines (derived from the per-tick `mode` column —
there is no separate transition log, L7 §5.2), so an edge can be read against
the reference and the error it happened on. A second figure, `catching_hand`,
stacks the hand sequencer (phase, ρ, timeout) and the fingertip lane (|F − b|,
debounced contact, freshness) with the attempt's verdict.

THE TWO LATCHES ARE SHADED (S9a, D-S9-H). Every panel of `catching_diag` shades
the ticks with `estop_active` (the CM's global E-STOP) and `fault_latched` (this
controller's own latch) set. They are separate latches with separate clears, so
they get separate colours: a run where the fault was reset but the E-STOP was
not reads as one band ending while the other goes on. The mode trace alone
cannot say this — an E-STOP does not move the supervisor to FAULT.

WHAT IS NOT HERE. There is no ball-truth column in this file — the controller
does not have one — so whether the verdict was RIGHT is not a question this
plot answers. The sim trial analysis owns that (G7-E, L7 §9).
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from rtc_tools.plotting.columns.detect import detect_joint_columns
from rtc_tools.utils.smoothing import COMMAND_SMOOTH_ROWS, box_smooth

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
# The CSV-only fault columns (#537 S9b), CatchingDiagLogPod's enums in order.
FAULT_CAUSE_NAMES = ("none", "qp_failures", "stop_deadline", "return_deadline")
FAULT_RESET_REFUSAL_NAMES = ("none", "command_moving", "arm_moving", "velocity_unreadable")


# CatchingDiagLogPod's decel_event / decel_refusal wire values, in the enums'
# own order (catching_diag_log_pod.hpp). `decel_refusal` is meaningful only on a
# tick whose `decel_judged` is set.
DECEL_EVENT_NAMES = (
    "none",
    "admitted",
    "deferred",
    "workspace",
    "switched",
    "gate_refused",
    "plan_mismatch",
    "sample_failed",
    "not_due",
    "no_segment",
    "replaced",
)
DECEL_REFUSAL_NAMES = (
    "none",
    "invalid",
    "activation",
    "plan",
    "repeat",
    "aged",
    "before_reset",
    "malformed",
)
# The codes this module reads are looked up in the two tables above, never
# written as numbers: the tables are what the test pins against the C++ enums.
# `repeat` is judged on almost every tick once a segment was taken — it is
# bookkeeping, not an event, so neither the lane nor the statistics list it.
_DECEL_REFUSAL_QUIET = (DECEL_REFUSAL_NAMES.index("none"), DECEL_REFUSAL_NAMES.index("repeat"))
_DECEL_EVENT_SWITCHED = DECEL_EVENT_NAMES.index("switched")
# Events on which the switch gate wrote its account (rho, dq_max, ...).
_DECEL_GATE_EVENTS = (_DECEL_EVENT_SWITCHED, DECEL_EVENT_NAMES.index("gate_refused"))
# Smallest command rate / acceleration / jerk the kinematics panel draws.
_KINEMATICS_FLOOR = 1e-3


def _code_name(names, k):
    """The name of wire code `k` in `names`, or the raw code if it is unknown."""
    return names[k] if 0 <= k < len(names) else str(k)


def _named_counts(values, names):
    """`k×n` per distinct code, named from `names` (the raw code if unknown)."""
    counts = values.map(lambda k: _code_name(names, k)).value_counts()
    return ", ".join(f"{k}×{v}" for k, v in counts.items())


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


def flag_intervals(df, column):
    """(t_start, t_end) for every run of consecutive rows where `column` is set.

    A row's flag holds from its own time to the NEXT row's, the same "post"
    convention the mode trace is stepped with, so a band and the mode edge it
    caused line up. A run that is still set on the last row ends AT the last
    row (there is no next row to end on) — a flag raised on the final tick is
    therefore a zero-width interval, returned rather than dropped so the count
    of latch episodes stays honest.

    A missing column, or one whose cells do not parse as numbers (NaN), yields
    no intervals: an absent flag is not evidence that the latch was up.
    """
    time_col = "timestamp" if "timestamp" in df.columns else "t_relative_s"
    if column not in df.columns or time_col not in df.columns or len(df) == 0:
        return []
    t = pd.to_numeric(df[time_col], errors="coerce")
    on = pd.to_numeric(df[column], errors="coerce") > 0.5  # NaN compares False
    keep = t.notna()
    t = t[keep].to_numpy(dtype=float)
    on = on[keep].to_numpy(dtype=bool)
    # Run edges, vectorised (a long session is millions of rows): +1 where a
    # run starts, -1 one past where it ends — the padding closes a run that is
    # still set on the last row.
    edges = np.diff(np.concatenate(([0], on.astype(np.int8), [0])))
    starts = np.flatnonzero(edges == 1)
    stops = np.flatnonzero(edges == -1)  # exclusive: the first row after the run
    return [
        (float(t[a]), float(t[b] if b < len(t) else t[-1]))
        for a, b in zip(starts, stops, strict=True)
    ]


# (column, legend label, colour). E-STOP and the controller fault are distinct
# latches with distinct clears (/rtc_cm/clear_estop vs /rtc_cm/reset_fault), so
# they never share a colour.
LATCH_SHADES = (
    ("estop_active", "E-STOP", "crimson"),
    ("fault_latched", "fault latched", "darkorange"),
)


def _shade_latches(axes, df, legend_axis=None):
    """Shade every latch interval on every panel; label each latch once.

    Returns True when anything was shaded, so the caller knows whether a legend
    entry exists to show.
    """
    shaded = False
    for column, label, colour in LATCH_SHADES:
        for k, (t0, t1) in enumerate(flag_intervals(df, column)):
            for ax in axes:
                ax.axvspan(
                    t0,
                    t1,
                    color=colour,
                    alpha=0.15,
                    linewidth=0,
                    zorder=0,
                    gid=f"latch_{column}",
                    label=label if (k == 0 and ax is legend_axis) else None,
                )
            shaded = True
    return shaded


def _mode_label(value):
    return _code_name(MODE_NAMES, value)


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
    # Under `mode: mpc` the soft-catch reference never runs (`ref_valid` is 0 on
    # every tick) and the arm follows the decel segment instead, so a log whose
    # law ran only through `decel_following` must not read as "never ran".
    flags = [
        df[c].astype(float) > 0.5 for c in ("ref_valid", "decel_following") if c in df.columns
    ]
    if not flags:
        return df
    live = flags[0]
    for f in flags[1:]:
        live = live | f
    return df[live]


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


def _command_kinematics(df, t):
    """(|q̇|max, |q̈|max, |jerk|max) of the arm command per row, or None.

    None when there is no `q_cmd_*` block or the time axis is not strictly
    increasing (a concatenated session): np.gradient would divide by a zero or
    negative step and draw a number that never happened. Rows where any
    command is NaN are masked.
    """
    cols, _ = detect_joint_columns(df, "q_cmd_")
    t = np.asarray(t, dtype=float)
    if not cols or len(t) < 3 or not np.all(np.diff(t) > 0.0):
        return None
    q = df[cols].astype(float).to_numpy()
    bad = np.isnan(q).any(axis=1)
    # The command is differentiated three times; at a 2 ms tick the raw triple
    # difference is quantisation noise, so each derivative is smoothed before
    # it is differentiated again (the kernel catching_trials' cmd_* columns use).
    qd = np.gradient(q, t, axis=0)
    qdd = box_smooth(
        np.gradient(box_smooth(qd, COMMAND_SMOOTH_ROWS), t, axis=0), COMMAND_SMOOTH_ROWS
    )
    jerk = np.gradient(qdd, t, axis=0)
    out = []
    for d in (qd, qdd, jerk):
        m = np.abs(d).max(axis=1)
        m[bad] = np.nan
        out.append(m)
    return tuple(out)


def plot_catching_diag(df, save_dir=None):
    """Reference vs realised, tracking error, solver health, supervisor mode.

    The panel list follows what the log HAS, so an older schema still plots
    the panels it can and a log is not padded with panels it has nothing for:

    - acceleration (the soft-catch law's saturation) needs `ref_xdd_x` — a
      pilot schema lacks it — and at least one `ref_valid` tick: under `mode:
      mpc` that law never runs;
    - the segment feedforward and the decel lane need the decel block and a
      log in which the lane did something — a `closed_form` log has the
      columns, all zero;
    - command kinematics needs `q_cmd_*`.
    """
    if "ref_x_x" not in df.columns:
        print("  Skipping catching diag plot (ref_x_x not found)")
        return

    has = df.columns.__contains__
    t = df["timestamp"]
    kin = _command_kinematics(df, t)
    ref_ran = not has("ref_valid") or bool((df["ref_valid"].astype(float) > 0.5).any())
    following = has("decel_following") and bool((df["decel_following"].astype(float) > 0.5).any())
    panels = ["pos"]
    if has("ref_xdd_x") and ref_ran:
        panels.append("acc")
    panels.append("track")
    if following and all(has(f"decel_v_ff_{a}") for a in "xyz"):
        panels.append("vff")
    if has("decel_event") and (following or _decel_lane_active(df)):
        panels.append("lane")
    if kin is not None:
        panels.append("kin")
    panels.append("mode")

    n = len(panels)
    fig, axes = plt.subplots(n, 1, figsize=(14, 3.25 * n), sharex=True, squeeze=False)
    axes = list(axes[:, 0])
    ax_of = dict(zip(panels, axes, strict=True))
    fig.suptitle(
        "Dynamic Catching — reference, tracking, solver, supervisor",
        fontsize=16,
        fontweight="bold",
    )

    # ── 1. The task reference, per axis, with the plan's catch point ────────
    ax = ax_of["pos"]
    for axis, colour in zip("xyz", ("C0", "C1", "C2"), strict=True):
        ax.plot(
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
            ax.plot(
                t,
                _masked(df, pc, "plan_valid"),
                linewidth=0.9,
                color=colour,
                linestyle="--",
                label=f"p_c_{axis}",
            )
        seg = f"decel_p_d_{axis}"
        if seg in df.columns:
            # Under mode mpc `ref_valid` is 0 on every tick and THIS is the
            # reference the arm follows; dash-dot keeps it apart from the
            # soft-catch reference (solid) and the catch point (dashed).
            ax.plot(
                t,
                _masked(df, seg, "decel_following"),
                linewidth=1.2,
                color=colour,
                linestyle="-.",
                label=f"seg_{axis}",
            )
    ax.set_ylabel("task position (m)")
    ax.legend(fontsize=7, ncol=3)
    ax.grid(True, alpha=0.3)

    # ── 2. Saturation: realised acceleration against the demand ────────────
    if "acc" in ax_of:
        ax = ax_of["acc"]
        for axis, colour in zip("xyz", ("C0", "C1", "C2"), strict=True):
            ax.plot(
                t,
                _masked(df, f"ref_xdd_{axis}", "ref_valid"),
                linewidth=1.0,
                color=colour,
                label=f"xdd_{axis}",
            )
            u = f"ref_u_des_{axis}"
            if u in df.columns:
                ax.plot(
                    t,
                    _masked(df, u, "ref_valid"),
                    linewidth=0.8,
                    color=colour,
                    linestyle=":",
                    label=f"u_des_{axis}",
                )
        ax.set_ylabel("accel (m/s²)")
        ax.legend(fontsize=7, ncol=3)
        ax.grid(True, alpha=0.3)

    # ── 3. Tracking error and the solve ────────────────────────────────────
    ax = ax_of["track"]
    if "track_err_rad" in df.columns:
        ax.plot(
            t,
            _masked(df, "track_err_rad", "clik_ran"),
            linewidth=1.2,
            color="C3",
            label="‖q_meas − q_cmd‖ (rad)",
        )
    ax_solve = ax.twinx()
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
    ax.set_ylabel("track error (rad)")
    ax.legend(fontsize=7, loc="upper left")
    ax_solve.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)

    # ── 4. Segment velocity feedforward ────────────────────────────────────
    if "vff" in ax_of:
        ax = ax_of["vff"]
        for axis, colour in zip("xyz", ("C0", "C1", "C2"), strict=True):
            ax.plot(
                t,
                _masked(df, f"decel_v_ff_{axis}", "decel_following"),
                linewidth=1.0,
                color=colour,
                label=f"v_ff_{axis}",
            )
        ax.set_ylabel("segment v_ff (m/s)")
        ax.legend(fontsize=7, ncol=3)
        ax.grid(True, alpha=0.3)

    # ── 5. The decel lane: events, refusals, and the switch gate's ρ ───────
    if "lane" in ax_of:
        _draw_decel_lane(ax_of["lane"], df, t)

    # ── 6. Command kinematics ──────────────────────────────────────────────
    if "kin" in ax_of:
        # One log axis for all three: the orders of magnitude between rad/s and
        # rad/s³ would flatten a shared linear axis, and a twin axis can carry
        # only two of the three honestly. Values under the floor are not drawn:
        # a held command differentiates to rounding noise (1e-12 and below),
        # which a log axis would stretch over most of the panel.
        ax = ax_of["kin"]
        for values, label, colour in zip(
            kin,
            ("max|q̇| (rad/s)", "max|q̈| (rad/s²)", "max|jerk| (rad/s³)"),
            ("C0", "C1", "C3"),
            strict=True,
        ):
            ax.plot(
                t,
                np.where(values >= _KINEMATICS_FLOOR, values, np.nan),
                linewidth=0.9,
                color=colour,
                label=label,
            )
        ax.set_yscale("log")
        ax.set_ylim(bottom=_KINEMATICS_FLOOR)
        ax.set_ylabel("command kinematics\n(max over joints)")
        ax.legend(fontsize=7, ncol=3, loc="upper right")
        ax.grid(True, alpha=0.3, which="both")

    # ── 7. The supervisor ──────────────────────────────────────────────────
    ax = ax_of["mode"]
    if "mode" in df.columns:
        ax.step(t, df["mode"], where="post", linewidth=1.4, color="C5")
        ax.set_yticks(range(len(MODE_NAMES)))
        ax.set_yticklabels(MODE_NAMES, fontsize=7)
    ax.set_ylabel("supervisor mode")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    _draw_transitions(axes, mode_transitions(df), label_axis=axes[0])
    # After the other panels' legend() calls, so the bands do not crowd those
    # legends; the supervisor panel has none of its own and carries the key.
    if _shade_latches(axes, df, legend_axis=ax):
        ax.legend(fontsize=7, loc="upper right")

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "catching_diag.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def _decel_shown_refusals(df):
    """(mask, codes) of the lane's refusals worth showing: judged, and neither
    `none` nor `repeat`."""
    judged = (
        df["decel_judged"].astype(float) > 0.5
        if "decel_judged" in df.columns
        else pd.Series(True, index=df.index)
    )
    refusal = df["decel_refusal"].astype(float).fillna(0).astype(int)
    return judged & ~refusal.isin(_DECEL_REFUSAL_QUIET), refusal


def _decel_lane_active(df):
    """Whether the lane did anything in this log (an event or a refusal)."""
    if (df["decel_event"].astype(float).fillna(0) != 0).any():
        return True
    return "decel_refusal" in df.columns and bool(_decel_shown_refusals(df)[0].any())


def _draw_decel_lane(ax, df, t):
    """Decel events on a categorical axis, refusals above them, ρ on a twin.

    Refusals sit on one row of their own ABOVE the events (one series per
    refusal reason) rather than on the event axis: they are a different enum,
    judged on a different set of ticks. Above, because the twin's ρ = 0 — every
    first switch — lands on the bottom row, which must not read as "refused".
    `repeat` is dropped: it is judged on almost every tick once a segment was
    taken and would paint the whole lane.
    """
    refused_y = len(DECEL_EVENT_NAMES)
    t = pd.Series(np.asarray(t, dtype=float), index=df.index)
    event = df["decel_event"].astype(float).fillna(0).astype(int)
    fired = event != 0
    if fired.any():
        ax.scatter(t[fired], event[fired], s=14, color="C0", marker="o", label="event")
    if "decel_refusal" in df.columns:
        shown, refusal = _decel_shown_refusals(df)
        for k in sorted(refusal[shown].unique()):
            sel = shown & (refusal == k)
            ax.scatter(
                t[sel],
                np.full(int(sel.sum()), float(refused_y)),
                s=14,
                marker="x",
                color=f"C{(int(k) + 2) % 10}",
                label=f"refused: {_code_name(DECEL_REFUSAL_NAMES, int(k))}",
            )
    ax.set_yticks(range(refused_y + 1))
    ax.set_yticklabels([*DECEL_EVENT_NAMES, "refused"], fontsize=7)
    ax.set_ylim(-0.8, refused_y + 0.8)
    ax.set_ylabel("decel lane")
    ax.grid(True, alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if "decel_rho" in df.columns:
        # ρ only means something on the ticks the switch gate judged.
        gate = event.isin(_DECEL_GATE_EVENTS)
        ax_rho = ax.twinx()
        if gate.any():
            ax_rho.scatter(
                t[gate],
                df.loc[gate, "decel_rho"].astype(float),
                s=26,
                marker="D",
                facecolors="none",
                edgecolors="C3",
                label="switch ρ",
            )
        ax_rho.set_ylabel("ρ")
        h2, l2 = ax_rho.get_legend_handles_labels()
        handles, labels = handles + h2, labels + l2
    if handles:
        ax.legend(handles, labels, fontsize=7, ncol=3, loc="upper left")


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


def _print_decel_statistics(df):
    """The decel block: segments followed, events, refusals, switch-gate ρ."""
    if "decel_event" not in df.columns:
        return
    event = df["decel_event"].astype(float).fillna(0).astype(int)
    following = (
        df["decel_following"].astype(float) > 0.5
        if "decel_following" in df.columns
        else pd.Series(False, index=df.index)
    )
    if not (following.any() or (event != 0).any()):
        return
    print("\nDecel segment:")
    print(f"  Ticks following a segment: {int(following.sum())}")
    if "decel_seq" in df.columns:
        print(f"  Distinct segments followed: {df.loc[following, 'decel_seq'].nunique()}")
    fired = event[event != 0]
    if len(fired) > 0:
        print("  Events: " + _named_counts(fired, DECEL_EVENT_NAMES))
    if "decel_refusal" in df.columns:
        shown, refusal = _decel_shown_refusals(df)
        refused = refusal[shown]
        if len(refused) > 0:
            print("  Refusals: " + _named_counts(refused, DECEL_REFUSAL_NAMES))
    if "decel_rho" in df.columns:
        rho = df.loc[event == _DECEL_EVENT_SWITCHED, "decel_rho"].astype(float).dropna()
        if len(rho) > 0:
            print(
                f"  Switch ρ [{len(rho)} switch(es)]: p50 {rho.quantile(0.5):.3f}  "
                f"p95 {rho.quantile(0.95):.3f}  max {rho.max():.3f}"
            )


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
            name = _code_name(OUTCOME_NAMES, k)
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
    # #537 S9b: every latch escalates as ABORT_ESCALATED, so the reason column
    # cannot say what raised it — `fault_cause` can. One count per latch (its
    # first tick), and the resets the controller refused because the arm moved.
    if "fault_cause" in df.columns:
        cause = df["fault_cause"].fillna(0).astype(int)
        raised = cause[(cause > 0) & (cause.shift(fill_value=0) != cause)]
        if len(raised) > 0:
            print("Fault latches: " + _named_counts(raised, FAULT_CAUSE_NAMES))
    if "fault_reset_refused" in df.columns:
        refused = df["fault_reset_refused"].fillna(0).astype(int)
        refused = refused[refused > 0]
        if len(refused) > 0:
            print("Fault resets refused: " + _named_counts(refused, FAULT_RESET_REFUSAL_NAMES))

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

    _print_decel_statistics(df)

    # ── The law ────────────────────────────────────────────────────────────
    live = _live(df)
    print(f"\nTicks with a reference: {len(live)}")
    if len(live) == 0:
        print("  Nothing below is measurable — the law never ran.")
        return
    if "ref_saturated" in live.columns:
        # Saturation belongs to the soft-catch reference; segment-following
        # ticks (mpc) have none, so counting them would dilute it toward 0.
        ref_ticks = live
        if "ref_valid" in live.columns:
            ref_ticks = live[live["ref_valid"].astype(float) > 0.5]
        if len(ref_ticks) > 0:
            print(f"Reference saturated: {_pct(ref_ticks['ref_saturated']):.1f}% of those ticks")

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
        # Trials in a row ended by a CLIK failure (#537 S9b), not failed solves.
        print(f"Longest QP failure streak (trials): {int(df['qp_fail_streak'].max())}")
    if "track_err_rad" in live.columns:
        err = live["track_err_rad"].astype(float)
        print(f"Tracking error [rad]: median {err.median():.4f}  max {err.max():.4f}")
