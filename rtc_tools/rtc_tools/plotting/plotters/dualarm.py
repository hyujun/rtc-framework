"""Plotters for the multi-frame CLIK diagnostics log (`dualarm_diag.csv`).

ONE ROW IS ONE TICK, AND EVERY TICK THE CONTROLLER RUNS WRITES ONE. A tick that
did not run the solve still writes its row with `clik_ran = 0` and the fields it
did not compute at zero. Three things follow, and every figure and statistic
here honours them:

  - a value is used on the ticks that computed it only. A zero on a tick that
    did not is "not computed", not "zero". There are three such sets: the ticks
    that ran the CLIK step (`clik_ran`; the tracking error is computed there),
    the ticks whose call reached the QP (`reached_solve` — a call refused on its
    inputs has `clik_ran = 1` and no solve time, iteration count or status),
    and per task the ticks with `<task>_valid` (the measured pose:
    `<task>_meas_valid`);
  - `tick` is the controller manager's loop counter, which keeps counting while
    another controller is active. A gap in it is therefore either rows that
    were dropped or a stretch this controller was not running. The statistics
    separate the two by what follows the gap: a controller that comes back
    re-seeds;
  - lines are broken at a tick gap rather than drawn across it.

THE COLUMN COUNT FOLLOWS THE RUN. Task names and joint names are part of the
column names (`<task>_err_lin`, `q_cmd_<joint>`), and how many of each there are
is whatever that run configured. Nothing here knows a name or a count: tasks
come from `detect_task_prefixes`, joints from the `q_cmd_` prefix, and a figure
grows a row per task.

THREE POSES PER TASK, all in that task's own base frame: `ref` is the reference
the solve was given, `cmd` the frame at the command state it was evaluated at,
`meas` the frame at this tick's measured joint state. `err_*` is ref against
cmd — the error the solve fed back. It carries NO servo lag. The cmd-to-meas
gap drawn in `dualarm_diag_task_pose` is the raw same-tick difference: the lag
is in it, unshifted, so it reads as tracking error while the frame moves and
that is what it is meant to show.

THE BRAKE MASKS ARE NOT DECODED. Their bits are model velocity indices, and the
file does not say which joint an index is. The figure counts set bits and the
statistics list the bit numbers.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator

from rtc_tools.plotting.columns import (
    detect_joint_columns,
    detect_task_prefixes,
    task_counter_columns,
)
from rtc_tools.plotting.columns.views import GROUP_GOAL_REJECTS
from rtc_tools.plotting.layout import auto_subplot_grid, hide_unused_axes

# DualArmDiagLogPod::Hold wire values, in the enum's own order. 0 is "the solve
# ran" and is never shaded. This table and FAULT_CAUSE_NAMES are pinned to the
# C++ enums by test_plot_dualarm_diag.py, which reads the POD header.
HOLD_NOT_SEEDED = 4
HOLD_NAMES = (
    "solved",
    "E-STOP",
    "fault latch",
    "body unreadable",
    "not seeded",
    "no model",
)
_HOLD_COLORS = ("none", "#d62728", "#ff7f0e", "#9467bd", "#7f7f7f", "#8c564b")

# DualArmDiagLogPod::FaultCause wire values.
FAULT_CAUSE_NAMES = ("none", "QP fail streak", "tracking error", "seed outside box")

# Per-tick solve outcome flags that mark a tick as abnormal when set.
_FAILURE_FLAGS = (
    "rejected_input",
    "non_finite",
    "command_mismatch",
    "accel_rows_violated",
    "brake_box_empty",
)

_AXES = ("x", "y", "z")
_QUAT = ("qw", "qx", "qy", "qz")


def _finish(save_dir, stem):
    """Save the current figure as `<stem>.png` under `save_dir`, or show it."""
    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / f"{stem}.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def _name(table, code):
    """`table[code]`, or the bare number for a code this file does not know."""
    code = int(code)
    return table[code] if 0 <= code < len(table) else f"code {code}"


def _ints(series):
    """An integer column as int64, a cell that did not parse counted as 0."""
    return series.fillna(0).to_numpy().astype(np.int64)


def _flag(df, col):
    """Boolean array: `col` is 1. A missing column, or a NaN cell, is False."""
    if col not in df.columns:
        return np.zeros(len(df), dtype=bool)
    return (df[col] == 1).to_numpy()


def _ran(df):
    """Ticks that ran the CLIK step (all of them if the file does not say)."""
    return _flag(df, "clik_ran") if "clik_ran" in df.columns else np.ones(len(df), dtype=bool)


def _solved(df):
    """Ticks whose call reached the QP — the ones with solve diagnostics.

    A call refused on its inputs counts as a run tick (and as a failed solve)
    but leaves solve time, iterations and status at their reset values.
    """
    return _ran(df) & _flag(df, "reached_solve") if "reached_solve" in df.columns else _ran(df)


def _tick_gaps(df):
    """Row indices `i` with ticks missing between row `i - 1` and row `i`, and
    how many are missing at each. Empty without a usable `tick` column."""
    if "tick" not in df.columns or len(df) < 2:
        return np.array([], dtype=int), np.array([], dtype=np.int64)
    tick = df["tick"].ffill().fillna(0).to_numpy().astype(np.int64)
    missing = np.diff(tick) - 1
    at = np.flatnonzero(missing > 0) + 1
    return at, missing[at - 1]


def _with_line_breaks(df):
    """`df` with an all-NaN row inserted at every tick gap.

    matplotlib breaks a line at NaN, so a stretch the file has no rows for is
    left blank instead of being bridged by a straight segment that reads as
    data. The inserted row is timed one typical tick after the row before the
    gap, so a band that row carries (see `_shade`) ends there and not somewhere
    inside the gap.
    """
    at, _ = _tick_gaps(df)
    if len(at) == 0:
        return df
    t = df["timestamp"].to_numpy(dtype=float)
    blanks = pd.DataFrame(np.nan, index=at - 0.5, columns=df.columns)
    steps = np.diff(t)
    tick_s = float(np.median(steps[steps > 0])) if (steps > 0).any() else 0.0
    blanks["timestamp"] = np.minimum(t[at - 1] + tick_s, 0.5 * (t[at - 1] + t[at]))
    return pd.concat([df, blanks]).sort_index(kind="stable").reset_index(drop=True)


def _shade(ax, t, where, color, alpha):
    """Paint the full height of `ax` over the rows where `where` is set.

    A row's flag holds from its own time to the NEXT row's, so each run is
    extended by the row that follows it. Without that a run of one row has no
    width at all — `fill_between(where=...)` only fills between two set
    samples, and so does an `axvspan` from a run's first row to its last — and
    the legend would name a hold nobody can see. One `fill_between` over a
    blended transform, so a long session costs one artist per axis, not one
    per run. A run on the very last row has no next row and stays zero-width.
    """
    where = np.asarray(where, dtype=bool)
    where = where | np.concatenate(([False], where[:-1]))
    ax.fill_between(
        t,
        0,
        1,
        where=where,
        transform=ax.get_xaxis_transform(),
        color=color,
        alpha=alpha,
        linewidth=0,
    )


def _shade_holds(axes, df):
    """Shade every held tick on all `axes`, one colour per hold cause.

    Returns the legend handles for the causes that occurred, so the caller can
    put the key on one axis instead of on each.
    """
    if "hold" not in df.columns:
        return []
    t = df["timestamp"].to_numpy(dtype=float)
    hold = _ints(df["hold"])
    handles = []
    for code in sorted(set(hold.tolist()) - {0}):
        color = _HOLD_COLORS[code] if 0 < code < len(_HOLD_COLORS) else "#17becf"
        for ax in axes:
            _shade(ax, t, hold == code, color, 0.15)
        handles.append(Patch(color=color, alpha=0.3, label=f"hold: {_name(HOLD_NAMES, code)}"))
    return handles


def _raster(ax, t, rows):
    """Draw `rows` = [(label, bool mask), ...] as one tick-mark lane each."""
    for i, (_, mask) in enumerate(rows):
        hits = t[np.asarray(mask, dtype=bool)]
        ax.plot(hits, np.full(len(hits), i), "|", markersize=10, color=f"C{i % 10}")
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([label for label, _ in rows], fontsize=8)
    ax.set_ylim(-0.6, max(len(rows), 1) - 0.4)
    ax.grid(True, axis="x", alpha=0.3)


def _count_axis(ax):
    """Integer ticks and a floor at zero, for an axis that carries a count.

    A count that never moved is a flat line; left to autoscale it gets a range
    of fractions around zero that reads as if it were a continuous quantity.
    """
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_ylim(bottom=min(ax.get_ylim()[0], -0.5), top=max(ax.get_ylim()[1], 1.5))


def _bit(series, k):
    """Boolean array: bit `k` of an integer mask column is set."""
    return ((_ints(series) >> k) & 1).astype(bool)


def _popcount(series):
    """Number of set bits per row of an integer mask column."""
    values, inverse = np.unique(_ints(series), return_inverse=True)
    return np.array([int(v).bit_count() for v in values])[inverse]


def _masked(df, col, mask):
    """`df[col]` as float with the rows outside `mask` blanked (NaN)."""
    return df[col].astype(float).where(mask)


def _task_valid(df, task):
    return _flag(df, f"{task}_valid")


def _new_goal_rows(df, task):
    """Row indices where `task` took a new goal.

    The goal sequence also changes when a re-seed resets it to 0 (the task goes
    back to holding the pose it is at). That is not a goal, so only a change TO
    a non-zero sequence counts. A cell that did not parse keeps the last value.
    """
    col = f"{task}_goal_sequence"
    if col not in df.columns:
        return np.array([], dtype=int)
    seq = df[col].ffill().fillna(0).to_numpy()
    return np.flatnonzero((np.diff(seq) != 0) & (seq[1:] != 0)) + 1


def _has_pose(df, task, kind):
    return all(f"{task}_{kind}_{a}" in df.columns for a in _AXES + _QUAT)


def plot_dualarm_diag_solver(df, save_dir=None):
    """Solve health: time, iterations / status, abnormal ticks, watchdogs."""
    if "solve_time_us" not in df.columns:
        print("  Skipping CLIK solver plot (solve_time_us not found)")
        return

    df = _with_line_breaks(df)
    fig, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    fig.suptitle("Multi-frame CLIK — Solver", fontsize=16, fontweight="bold")
    t = df["timestamp"]
    ran = _ran(df)
    solved = _solved(df)

    axes[0].plot(t, _masked(df, "solve_time_us", solved), linewidth=1.0, color="C0")
    axes[0].set_ylabel("solve time (µs)")
    axes[0].grid(True, alpha=0.3)

    if "iterations" in df.columns:
        axes[1].plot(t, _masked(df, "iterations", solved), linewidth=1.0, color="C1")
    axes[1].set_ylabel("iterations", color="C1")
    axes[1].grid(True, alpha=0.3)
    if "status" in df.columns:
        ax_status = axes[1].twinx()
        ax_status.step(
            t, _masked(df, "status", solved), where="post", linewidth=1.0, color="C3", alpha=0.8
        )
        ax_status.set_ylabel("solver status code", color="C3")
        _count_axis(ax_status)
    _count_axis(axes[1])

    rows = []
    if "converged" in df.columns:
        rows.append(("not converged", ran & ~_flag(df, "converged")))
    rows += [(flag, _flag(df, flag)) for flag in _FAILURE_FLAGS if flag in df.columns]
    _raster(axes[2], t.to_numpy(), rows)
    axes[2].set_ylabel("abnormal ticks")

    if "qp_fail_streak" in df.columns:
        axes[3].step(t, df["qp_fail_streak"], where="post", linewidth=1.2, color="C3")
    axes[3].set_ylabel("qp_fail_streak (ticks)", color="C3")
    _count_axis(axes[3])
    axes[3].grid(True, alpha=0.3)
    if "track_err" in df.columns:
        ax_track = axes[3].twinx()
        ax_track.plot(t, _masked(df, "track_err", ran), linewidth=1.0, color="C0", alpha=0.8)
        ax_track.set_ylabel("track_err: max |q_meas − q_cmd| (rad)", color="C0")
    axes[3].set_xlabel("Time (s)")

    handles = _shade_holds(axes, df)
    if handles:
        axes[0].legend(handles=handles, fontsize=8, loc="upper right")
    _finish(save_dir, "dualarm_diag_solver")


def plot_dualarm_diag_task_error(df, save_dir=None):
    """Per task: the pose error the solve fed back (reference against command)."""
    tasks = detect_task_prefixes(df)
    if not tasks:
        print("  Skipping CLIK task error plot (no <task>_err_lin columns)")
        return

    df = _with_line_breaks(df)
    fig, axes = plt.subplots(
        len(tasks), 1, figsize=(14, 3.6 * len(tasks) + 1), sharex=True, squeeze=False
    )
    fig.suptitle(
        "Multi-frame CLIK — Task Error (reference vs command, no servo lag)",
        fontsize=16,
        fontweight="bold",
    )
    axes = axes.flatten()
    t = df["timestamp"]
    t_np = t.to_numpy(dtype=float)

    for ax, task in zip(axes, tasks, strict=True):
        valid = _task_valid(df, task)
        ax.plot(t, _masked(df, f"{task}_err_lin", valid), linewidth=1.2, color="C0")
        ax.set_ylabel("‖position error‖ (m)", color="C0")
        ax.set_title(task, fontsize=10)
        ax.grid(True, alpha=0.3)
        ax_ang = ax.twinx()
        ax_ang.plot(
            t, _masked(df, f"{task}_err_ang", valid), linewidth=1.2, color="C1", alpha=0.85
        )
        ax_ang.set_ylabel("‖rotation error‖ (rad)", color="C1")

        _shade(ax, t_np, _flag(df, f"{task}_traj_active"), "0.5", 0.12)
        for idx in _new_goal_rows(df, task):
            ax.axvline(t_np[idx], color="0.3", linewidth=0.8, linestyle=":")
    axes[-1].set_xlabel("Time (s)   —   grey band: reference moving, dotted line: new goal")

    handles = _shade_holds(axes, df)
    if handles:
        axes[0].legend(handles=handles, fontsize=8, loc="upper right")
    _finish(save_dir, "dualarm_diag_task_error")


def _cmd_meas_gap(df, task):
    """(distance [m], angle [rad]) between the commanded and the measured pose.

    Same-tick difference, not time-aligned. NaN where either pose was not
    computed. The angle is the geodesic one, so q and −q read as the same
    rotation.
    """
    both = _task_valid(df, task)
    if f"{task}_meas_valid" in df.columns:
        both = both & _flag(df, f"{task}_meas_valid")
    cmd_p = df[[f"{task}_cmd_{a}" for a in _AXES]].to_numpy(dtype=float)
    meas_p = df[[f"{task}_meas_{a}" for a in _AXES]].to_numpy(dtype=float)
    cmd_q = df[[f"{task}_cmd_{a}" for a in _QUAT]].to_numpy(dtype=float)
    meas_q = df[[f"{task}_meas_{a}" for a in _QUAT]].to_numpy(dtype=float)
    dist = np.linalg.norm(cmd_p - meas_p, axis=1)
    dot = np.clip(np.abs(np.einsum("ij,ij->i", cmd_q, meas_q)), 0.0, 1.0)
    angle = 2.0 * np.arccos(dot)
    return np.where(both, dist, np.nan), np.where(both, angle, np.nan)


def plot_dualarm_diag_task_pose(df, save_dir=None):
    """Per task: reference / commanded / measured position, and the cmd-meas gap."""
    tasks = [
        task
        for task in detect_task_prefixes(df)
        if all(_has_pose(df, task, kind) for kind in ("ref", "cmd", "meas"))
    ]
    if not tasks:
        print("  Skipping CLIK task pose plot (no <task>_{ref,cmd,meas}_* columns)")
        return

    df = _with_line_breaks(df)
    fig, axes = plt.subplots(
        len(tasks), 4, figsize=(22, 3.6 * len(tasks) + 1), sharex=True, squeeze=False
    )
    fig.suptitle(
        "Multi-frame CLIK — Task Pose (each task in its own base frame)",
        fontsize=16,
        fontweight="bold",
    )
    t = df["timestamp"]

    for row, task in zip(axes, tasks, strict=True):
        valid = _task_valid(df, task)
        meas_ok = valid
        if f"{task}_meas_valid" in df.columns:
            meas_ok = _flag(df, f"{task}_meas_valid")
        for ax, axis in zip(row[:3], _AXES, strict=True):
            ax.plot(t, _masked(df, f"{task}_ref_{axis}", valid), label="ref", linewidth=1.4)
            ax.plot(
                t,
                _masked(df, f"{task}_cmd_{axis}", valid),
                label="cmd",
                linewidth=1.2,
                linestyle="--",
            )
            ax.plot(
                t,
                _masked(df, f"{task}_meas_{axis}", meas_ok),
                label="meas",
                linewidth=1.0,
                linestyle=":",
            )
            ax.set_title(f"{task} — {axis}", fontsize=10)
            ax.set_ylabel(f"{axis} (m)")
            ax.grid(True, alpha=0.3)
        row[0].legend(fontsize=8)

        dist, angle = _cmd_meas_gap(df, task)
        gap_ax = row[3]
        gap_ax.plot(t, dist, linewidth=1.2, color="C0")
        gap_ax.set_ylabel("‖cmd − meas‖ (m)", color="C0")
        gap_ax.set_title(f"{task} — cmd vs meas, same tick (not time-aligned)", fontsize=10)
        gap_ax.grid(True, alpha=0.3)
        gap_ang = gap_ax.twinx()
        gap_ang.plot(t, angle, linewidth=1.2, color="C1", alpha=0.85)
        gap_ang.set_ylabel("angle (rad)", color="C1")
    for ax in axes[-1]:
        ax.set_xlabel("Time (s)")

    _finish(save_dir, "dualarm_diag_task_pose")


def plot_dualarm_diag_limits(df, save_dir=None):
    """What bounded the solve: torque rows, feedback caps, braking bounds."""
    tasks = detect_task_prefixes(df)
    df = _with_line_breaks(df)
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Multi-frame CLIK — Active Limits", fontsize=16, fontweight="bold")
    t = df["timestamp"]
    solved = _solved(df)

    if "accel_rows" in df.columns:
        axes[0].step(
            t,
            _masked(df, "accel_rows", solved),
            where="post",
            linewidth=1.0,
            linestyle="--",
            color="0.5",
            label="rows in the QP",
        )
    if "accel_rows_binding" in df.columns:
        axes[0].step(
            t,
            _masked(df, "accel_rows_binding", solved),
            where="post",
            linewidth=1.2,
            color="C3",
            label="rows binding",
        )
    axes[0].set_ylabel("acceleration rows")
    _count_axis(axes[0])
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Bit 2k is task k's linear feedback cap, 2k+1 its angular one; bit k of
    # rot_near_pi is task k. k is the task's position in the header.
    rows = []
    for k, task in enumerate(tasks):
        if "fb_saturated" in df.columns:
            rows.append((f"{task}: linear feedback capped", _bit(df["fb_saturated"], 2 * k)))
            rows.append((f"{task}: angular feedback capped", _bit(df["fb_saturated"], 2 * k + 1)))
        if "rot_near_pi" in df.columns:
            rows.append((f"{task}: rotation error near π", _bit(df["rot_near_pi"], k)))
    _raster(axes[1], t.to_numpy(), rows)
    axes[1].set_ylabel("per-task limits")

    for col, color in (("brake_active", "C0"), ("brake_static_infeasible", "C3")):
        if col in df.columns:
            axes[2].step(
                t, _popcount(df[col]), where="post", linewidth=1.2, color=color, label=col
            )
    axes[2].set_ylabel("joints (set bits)")
    _count_axis(axes[2])
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    _shade_holds(axes, df)
    _finish(save_dir, "dualarm_diag_limits")


def plot_dualarm_diag_goals(df, save_dir=None):
    """Cumulative goal counters: accepted, refused at ingress, dropped on a tick."""
    tasks = [task for task in detect_task_prefixes(df) if task_counter_columns(df, task)]
    has_group = GROUP_GOAL_REJECTS in df.columns
    n_rows = len(tasks) + (1 if has_group else 0)
    if n_rows == 0:
        print("  Skipping CLIK goal counter plot (no counter columns)")
        return

    df = _with_line_breaks(df)
    fig, axes = plt.subplots(n_rows, 1, figsize=(14, 3.2 * n_rows + 1), sharex=True, squeeze=False)
    fig.suptitle("Multi-frame CLIK — Goal Counters (cumulative)", fontsize=16, fontweight="bold")
    axes = axes.flatten()
    t = df["timestamp"]

    for ax, task in zip(axes, tasks, strict=False):
        for col in task_counter_columns(df, task):
            ax.step(t, df[col], where="post", linewidth=1.3, label=col[len(task) + 1 :])
        ax.set_title(task, fontsize=10)
        ax.set_ylabel("count")
        _count_axis(ax)
        ax.legend(fontsize=8, ncol=4)
        ax.grid(True, alpha=0.3)
    if has_group:
        axes[-1].step(t, df[GROUP_GOAL_REJECTS], where="post", linewidth=1.3, color="C3")
        axes[-1].set_title("goals refused on a device-group lane", fontsize=10)
        axes[-1].set_ylabel("count")
        _count_axis(axes[-1])
        axes[-1].grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")

    _finish(save_dir, "dualarm_diag_goals")


def plot_dualarm_diag_joint_cmd(df, save_dir=None):
    """The joint command the solve integrated, one panel per joint."""
    cols, names = detect_joint_columns(df, "q_cmd_")
    if not cols:
        print("  Skipping CLIK joint command plot (q_cmd_* columns not found)")
        return

    df = _with_line_breaks(df)
    nrows, ncols = auto_subplot_grid(len(cols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3.2 * nrows), sharex=True)
    fig.suptitle("Multi-frame CLIK — Joint Command", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()
    t = df["timestamp"]

    for ax, col, name in zip(axes, cols, names, strict=False):
        ax.plot(t, df[col], linewidth=1.2, color="C0")
        ax.set_title(name, fontsize=9)
        ax.set_ylabel("q_cmd (rad)")
        ax.grid(True, alpha=0.3)
    hide_unused_axes(axes, len(cols))

    _shade_holds(axes[: len(cols)], df)
    _finish(save_dir, "dualarm_diag_joint_cmd")


def _set_bits(series):
    """Sorted bit numbers set in any row of an integer mask column."""
    union = 0
    for value in np.unique(_ints(series)):
        union |= int(value)
    return [k for k in range(union.bit_length()) if (union >> k) & 1]


def _max_int(series):
    """Largest value of an integer column, 0 if no cell parsed."""
    return int(np.nan_to_num(series.max()))


def _last_int(series):
    """Last value of a cumulative counter that parsed (a file cut mid-row
    leaves the cells of its last line empty), 0 if none did."""
    parsed = series.dropna()
    return int(parsed.iloc[-1]) if len(parsed) else 0


def _print_tick_gaps(df):
    """Report missing ticks, split by whether the controller was away.

    After a gap the controller either carries on — rows were dropped — or
    starts over: its first row back re-seeds, or holds as not yet seeded. The
    second kind is a stretch it was not running, and no row is missing.
    """
    if "tick" not in df.columns:
        return
    at, missing = _tick_gaps(df)
    if len(at) == 0:
        print("Tick gaps: none — every tick has a row")
        return
    restarted = _flag(df, "reseeded")[at]
    if "hold" in df.columns:
        restarted = restarted | (_ints(df["hold"])[at] == HOLD_NOT_SEEDED)
    away, lost = missing[restarted], missing[~restarted]
    print(f"Tick gaps: {len(at)} ({int(missing.sum())} ticks)")
    if len(away):
        print(
            f"  controller not running: {len(away)} gap(s), {int(away.sum())} ticks "
            "(the row after each starts over — no row is missing)"
        )
    if len(lost):
        print(
            f"  DROPPED ROWS: {int(lost.sum())} ticks in {len(lost)} gap(s) "
            f"(largest {int(lost.max())})"
        )


def print_dualarm_diag_statistics(df):
    """Console summary: what the ticks did, solve health, per-task error, goals."""
    print("\n=== Multi-frame CLIK Diagnostics ===")
    n = len(df)
    t = df["timestamp"].to_numpy(dtype=float)
    duration = float(t[-1] - t[0]) if n > 1 else 0.0
    print(f"Duration: {duration:.2f} s | Rows: {n}")
    _print_tick_gaps(df)

    ran = _ran(df)
    solved = _solved(df)
    n_ran, n_solved = int(ran.sum()), int(solved.sum())
    print(f"Ticks that ran the solve: {n_ran}/{n} ({100.0 * n_ran / n:.1f}%)")
    if n_ran != n_solved:
        print(f"  refused before the QP (no solve data): {n_ran - n_solved}")
    if "hold" in df.columns:
        counts = pd.Series(_ints(df["hold"])).value_counts().sort_index()
        held = ", ".join(f"{_name(HOLD_NAMES, code)}: {c}" for code, c in counts.items() if code)
        print(f"Held ticks: {held if held else 'none'}")
    if "fault_latched" in df.columns:
        causes = (
            sorted(set(_ints(df["fault_cause"]).tolist()) - {0}) if "fault_cause" in df else []
        )
        cause = f" (cause: {', '.join(_name(FAULT_CAUSE_NAMES, c) for c in causes)})"
        print(
            f"Fault latched: {int(_flag(df, 'fault_latched').sum())} ticks{cause if causes else ''}"
        )
    if "reseeded" in df.columns:
        print(f"Re-seeds: {int(_flag(df, 'reseeded').sum())}")

    if n_solved and "solve_time_us" in df.columns:
        us = df["solve_time_us"].to_numpy(dtype=float)[solved]
        print(
            f"Solve time (µs, ticks that reached the QP): p50={np.nanpercentile(us, 50):.1f} "
            f"p99={np.nanpercentile(us, 99):.1f} max={np.nanmax(us):.1f}"
        )
    if n_ran and "converged" in df.columns:
        print(
            f"Converged: {100.0 * (ran & _flag(df, 'converged')).sum() / n_ran:.2f}% of run ticks"
        )
    if n_solved and "iterations" in df.columns:
        it = df["iterations"].to_numpy(dtype=float)[solved]
        print(f"Iterations: mean={np.nanmean(it):.2f} max={int(np.nanmax(it))}")
    flagged = [
        f"{flag}: {int(_flag(df, flag).sum())}" for flag in _FAILURE_FLAGS if _flag(df, flag).any()
    ]
    print(f"Abnormal ticks: {', '.join(flagged) if flagged else 'none'}")
    if "qp_fail_streak" in df.columns:
        print(f"Longest QP fail streak: {_max_int(df['qp_fail_streak'])} ticks")
    if n_ran and "track_err" in df.columns:
        track = df["track_err"].to_numpy(dtype=float)[ran]
        print(f"Tracking error max |q_meas − q_cmd| (run ticks): {np.nanmax(track):.6f} rad")
    if "accel_rows_binding" in df.columns:
        print(
            f"Ticks with a binding acceleration row: {int((df['accel_rows_binding'] > 0).sum())}"
        )
    for col in ("brake_active", "brake_static_infeasible"):
        if col in df.columns:
            bits = _set_bits(df[col])
            print(f"{col} — model velocity indices ever set: {bits if bits else 'none'}")

    tasks = detect_task_prefixes(df)
    print(f"\nTasks ({len(tasks)}): {', '.join(tasks) if tasks else 'none'}")
    for task in tasks:
        valid = _task_valid(df, task)
        n_valid = int(valid.sum())
        goals = len(_new_goal_rows(df, task))
        print(f"  {task}: valid on {n_valid}/{n} ticks, {goals} goal(s) taken up")
        if n_valid:
            lin = df[f"{task}_err_lin"].to_numpy(dtype=float)[valid]
            ang = df[f"{task}_err_ang"].to_numpy(dtype=float)[valid]
            print(
                f"    error (ref vs cmd): position rms={np.sqrt(np.nanmean(lin**2)):.6f} m "
                f"max={np.nanmax(lin):.6f} m | rotation rms={np.sqrt(np.nanmean(ang**2)):.6f} rad "
                f"max={np.nanmax(ang):.6f} rad"
            )
        if all(_has_pose(df, task, kind) for kind in ("cmd", "meas")):
            dist, angle = _cmd_meas_gap(df, task)
            if np.isfinite(dist).any():
                print(
                    "    cmd vs meas (same tick, lag included): "
                    f"position max={np.nanmax(dist):.6f} m | rotation max={np.nanmax(angle):.6f} rad"
                )
        counters = task_counter_columns(df, task)
        if counters:
            final = ", ".join(f"{c[len(task) + 1 :]}={_last_int(df[c])}" for c in counters)
            print(f"    counters (final): {final}")
    if GROUP_GOAL_REJECTS in df.columns:
        print(f"Goals refused on a device-group lane (final): {_last_int(df[GROUP_GOAL_REJECTS])}")
