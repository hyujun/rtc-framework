"""Plotters for the multi-frame CLIK diagnostics log (`dualarm_diag.csv`).

ONE ROW IS ONE TICK, AND EVERY TICK WRITES ONE. A tick that did not run the
solve still writes its row with `clik_ran = 0` and the fields it did not compute
at zero, so a gap in `tick` is a dropped row and nothing else. Two things
follow, and every figure here honours both:

  - solve diagnostics are drawn on the ticks that ran the solve only. A zero
    solve time on a held tick is "not computed", not "instant";
  - a task's error and poses are drawn where `<task>_valid` is 1 only, and the
    measured pose where `<task>_meas_valid` is 1.

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
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator

from rtc_tools.plotting.columns import (
    detect_joint_columns,
    detect_task_prefixes,
    task_counter_columns,
)
from rtc_tools.plotting.columns.views import GROUP_GOAL_REJECTS
from rtc_tools.plotting.layout import auto_subplot_grid

# DualArmDiagLogPod::Hold wire values, in the enum's own order (the header
# writer of that POD is the source). 0 is "the solve ran" and is never shaded.
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


def _solved(df):
    """Boolean mask of the ticks that ran the solve (all of them if unmarked)."""
    if "clik_ran" not in df.columns:
        return np.ones(len(df), dtype=bool)
    return (df["clik_ran"] == 1).to_numpy()


def _runs(mask):
    """`[(start, stop), ...]` index ranges of the True runs of a boolean array."""
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return []
    edges = np.flatnonzero(np.diff(np.concatenate(([False], mask, [False])).astype(int)))
    return list(zip(edges[::2], edges[1::2], strict=True))


def _shade_holds(axes, df):
    """Shade every held stretch on all `axes`, one colour per hold cause.

    Returns the legend handles for the causes that occurred, so the caller can
    put the key on one axis instead of on each.
    """
    if "hold" not in df.columns:
        return []
    t = df["timestamp"].to_numpy()
    hold = df["hold"].fillna(0).to_numpy().astype(int)
    handles = []
    for code in sorted(set(hold) - {0}):
        color = _HOLD_COLORS[code] if 0 < code < len(_HOLD_COLORS) else "#17becf"
        for start, stop in _runs(hold == code):
            for ax in axes:
                ax.axvspan(t[start], t[stop - 1], color=color, alpha=0.15, linewidth=0)
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
    return ((series.fillna(0).to_numpy().astype(np.int64) >> k) & 1).astype(bool)


def _popcount(series):
    """Number of set bits per row of an integer mask column."""
    values = series.fillna(0).to_numpy().astype(np.int64)
    counts = {v: int(v).bit_count() for v in np.unique(values)}
    return np.array([counts[v] for v in values])


def _masked(df, col, mask):
    """`df[col]` as float with the rows outside `mask` blanked (NaN)."""
    return df[col].astype(float).where(mask)


def _task_valid(df, task):
    return (df[f"{task}_valid"] == 1).to_numpy()


def _has_pose(df, task, kind):
    return all(f"{task}_{kind}_{a}" in df.columns for a in _AXES + _QUAT)


def plot_dualarm_diag_solver(df, save_dir=None):
    """Solve health: time, iterations / status, abnormal ticks, watchdogs."""
    if "solve_time_us" not in df.columns:
        print("  Skipping CLIK solver plot (solve_time_us not found)")
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    fig.suptitle("Multi-frame CLIK — Solver", fontsize=16, fontweight="bold")
    t = df["timestamp"]
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
        rows.append(("not converged", solved & (df["converged"] == 0).to_numpy()))
    rows += [(flag, (df[flag] == 1).to_numpy()) for flag in _FAILURE_FLAGS if flag in df.columns]
    _raster(axes[2], t.to_numpy(), rows)
    axes[2].set_ylabel("abnormal ticks")

    if "qp_fail_streak" in df.columns:
        axes[3].step(t, df["qp_fail_streak"], where="post", linewidth=1.2, color="C3")
    axes[3].set_ylabel("qp_fail_streak (ticks)", color="C3")
    _count_axis(axes[3])
    axes[3].grid(True, alpha=0.3)
    if "track_err" in df.columns:
        ax_track = axes[3].twinx()
        ax_track.plot(t, df["track_err"], linewidth=1.0, color="C0", alpha=0.8)
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
    t_np = t.to_numpy()

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

        traj_col = f"{task}_traj_active"
        if traj_col in df.columns:
            for start, stop in _runs((df[traj_col] == 1).to_numpy()):
                ax.axvspan(t_np[start], t_np[stop - 1], color="0.5", alpha=0.12, linewidth=0)
        seq_col = f"{task}_goal_sequence"
        if seq_col in df.columns:
            changed = np.flatnonzero(np.diff(df[seq_col].fillna(0).to_numpy()) != 0) + 1
            for idx in changed:
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
    meas_valid = f"{task}_meas_valid"
    if meas_valid in df.columns:
        both = both & (df[meas_valid] == 1).to_numpy()
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
            meas_ok = (df[f"{task}_meas_valid"] == 1).to_numpy()
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
    for idx in range(len(cols), len(axes)):
        axes[idx].set_visible(False)

    _shade_holds(axes[: len(cols)], df)
    _finish(save_dir, "dualarm_diag_joint_cmd")


def _set_bits(series):
    """Sorted bit numbers set in any row of an integer mask column."""
    union = 0
    for value in np.unique(series.fillna(0).to_numpy().astype(np.int64)):
        union |= int(value)
    return [k for k in range(union.bit_length()) if (union >> k) & 1]


def print_dualarm_diag_statistics(df):
    """Console summary: what the ticks did, solve health, per-task error, goals."""
    print("\n=== Multi-frame CLIK Diagnostics ===")
    n = len(df)
    t = df["timestamp"].to_numpy(dtype=float)
    duration = float(t[-1] - t[0]) if n > 1 else 0.0
    rate = f"{(n - 1) / duration:.1f} Hz" if duration > 0 else "n/a"
    print(f"Duration: {duration:.2f} s | Rows: {n} | Rate: {rate}")

    if "tick" in df.columns and n > 1:
        gaps = np.diff(df["tick"].to_numpy(dtype=np.int64))
        dropped = int((gaps[gaps > 1] - 1).sum())
        print(
            f"Dropped rows (tick gaps): {dropped}"
            + (" — every tick has a row" if not dropped else "")
        )

    solved = _solved(df)
    n_solved = int(solved.sum())
    print(f"Ticks that ran the solve: {n_solved}/{n} ({100.0 * n_solved / n:.1f}%)")
    if "hold" in df.columns:
        counts = df["hold"].fillna(0).astype(int).value_counts().sort_index()
        held = ", ".join(f"{_name(HOLD_NAMES, code)}: {c}" for code, c in counts.items() if code)
        print(f"Held ticks: {held if held else 'none'}")
    if "fault_latched" in df.columns:
        print(f"Fault latched: {int((df['fault_latched'] == 1).sum())} ticks", end="")
        causes = sorted(set(df.get("fault_cause", [0])) - {0})
        print(
            f" (cause: {', '.join(_name(FAULT_CAUSE_NAMES, c) for c in causes)})" if causes else ""
        )
    if "reseeded" in df.columns:
        print(f"Re-seeds: {int((df['reseeded'] == 1).sum())}")

    if n_solved and "solve_time_us" in df.columns:
        s = df["solve_time_us"].to_numpy(dtype=float)[solved]
        print(
            f"Solve time (µs, solved ticks): p50={np.percentile(s, 50):.1f} "
            f"p99={np.percentile(s, 99):.1f} max={s.max():.1f}"
        )
    if n_solved and "converged" in df.columns:
        conv = df["converged"].to_numpy(dtype=float)[solved]
        print(f"Converged: {100.0 * conv.mean():.2f}% of solved ticks")
    if n_solved and "iterations" in df.columns:
        it = df["iterations"].to_numpy(dtype=float)[solved]
        print(f"Iterations: mean={it.mean():.2f} max={int(it.max())}")
    flagged = [
        f"{flag}: {int((df[flag] == 1).sum())}"
        for flag in _FAILURE_FLAGS
        if flag in df.columns and (df[flag] == 1).any()
    ]
    print(f"Abnormal ticks: {', '.join(flagged) if flagged else 'none'}")
    if "qp_fail_streak" in df.columns:
        print(f"Longest QP fail streak: {int(df['qp_fail_streak'].max())} ticks")
    if "track_err" in df.columns:
        print(f"Tracking error max |q_meas − q_cmd|: {df['track_err'].max():.6f} rad")
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
        print(f"  {task}: valid on {n_valid}/{n} ticks", end="")
        seq_col = f"{task}_goal_sequence"
        if seq_col in df.columns:
            print(f", goal sequence reached {int(df[seq_col].max())}", end="")
        print()
        if n_valid:
            lin = df[f"{task}_err_lin"].to_numpy(dtype=float)[valid]
            ang = df[f"{task}_err_ang"].to_numpy(dtype=float)[valid]
            print(
                f"    error (ref vs cmd): position rms={np.sqrt(np.mean(lin**2)):.6f} m "
                f"max={lin.max():.6f} m | rotation rms={np.sqrt(np.mean(ang**2)):.6f} rad "
                f"max={ang.max():.6f} rad"
            )
        if all(_has_pose(df, task, kind) for kind in ("cmd", "meas")):
            dist, angle = _cmd_meas_gap(df, task)
            if np.isfinite(dist).any():
                print(
                    f"    cmd vs meas (same tick, lag included): position max={np.nanmax(dist):.6f} m "
                    f"| rotation max={np.nanmax(angle):.6f} rad"
                )
        counters = task_counter_columns(df, task)
        if counters:
            final = ", ".join(f"{c[len(task) + 1 :]}={int(df[c].iloc[-1])}" for c in counters)
            print(f"    counters (final): {final}")
    if GROUP_GOAL_REJECTS in df.columns:
        print(
            f"Goals refused on a device-group lane (final): {int(df[GROUP_GOAL_REJECTS].iloc[-1])}"
        )
