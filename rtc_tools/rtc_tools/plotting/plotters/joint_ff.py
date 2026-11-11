"""Plotters for the joint feedforward log (`joint_ff.csv`).

ONE ROW IS ONE TICK THE CONTROLLER RAN. An E-STOP or unreadable tick still
writes its row, with `ff_finite = 0` / `readable = 0` and the torque columns at
whatever the wire carried. `tick` is the controller manager's loop counter and
keeps counting while another controller is active, so a gap in it is either
dropped rows or a stretch this controller was not running; lines are broken at
a gap rather than drawn across it.

THE JOINT LIST FOLLOWS THE HEADER. Joint names are read off the `tau_ff_`
block (`q_` is also the prefix of `q_ref_`, so it cannot name them alone) and
every figure grows a panel per joint, in device order.

THE A/B/A STRUCTURE IS THE POINT. A hold run switches the feedforward lane on,
off and on again; the figures shade the `ff_enabled = 0` stretches so the
sag without feedforward and its removal with it read side by side. The PASS /
FAIL judgement of such a run is `analyze_hand_gravity_ff`
(rtc_tools.analysis.hand_gravity_ff_gate) — these figures show, they do not
judge, and the statistics printed here are whole-segment numbers, not the
judge's trailing windows.

THE INERTIA COLUMNS ARE OPTIONAL. A file that carries `tau_inertia_<joint>`
(k_M · M(q) · q̈_ref, the inertia term as the law added it) draws it on the
torque panel, so tau_ff = clamp(k_g · g + tau_inertia) reads off the figure; a
file without them renders as before.

THE `mode` COLUMN IS OPTIONAL. When present it is an integer code of the
controller's run mode: 0 tracking (the reference is the commanded one), 1
damping (the reference follows the measurement, feedforward zero), 2
damping_feedforward (the reference follows the measurement, the feedforward
lane stays active). Non-tracking stretches are shaded, the status figure gets a
mode lane, and the statistics split segments by `(ff_enabled, mode)`; in a
non-tracking segment q_ref equals q by construction, so drift is printed
instead of the tracking error. A file without the column renders as before.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator

from rtc_tools.analysis.hand_gravity_ff_gate import (
    G_PREFIX,
    Q_PREFIX,
    Q_REF_PREFIX,
    TAU_FF_PREFIX,
    joint_names_of,
)
from rtc_tools.plotting.layout import auto_subplot_grid, hide_unused_axes

# Optional per-joint block (a joint_ff.csv written before the inertia term has none).
TAU_INERTIA_PREFIX = "tau_inertia_"

# Status lanes of the raster figure, in drawing order: (column, label).
STATUS_LANES = (
    ("ff_enabled", "ff_enabled"),
    ("ff_finite", "ff_finite"),
    ("held", "held"),
    ("model_fresh", "model_fresh"),
    ("readable", "readable"),
    ("estop", "estop"),
    ("seeded", "seeded"),
    ("traj_active", "traj_active"),
    ("singular", "singular"),
)

_OFF_COLOR = "0.5"
_OFF_ALPHA = 0.12

# Wire codes of the `mode` column: code -> (name, shade colour or None).
MODE_NAMES = {0: "tracking", 1: "damping", 2: "damping_feedforward"}
_MODE_COLORS = {1: "tab:orange", 2: "tab:purple"}
_MODE_ALPHA = 0.14


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


def _ints(series):
    """An integer column as int64, a cell that did not parse counted as 0."""
    return series.fillna(0).to_numpy().astype(np.int64)


def _flag(df, col):
    """Boolean array: `col` is 1. A missing column, or a NaN cell, is False."""
    if col not in df.columns:
        return np.zeros(len(df), dtype=bool)
    return (df[col] == 1).to_numpy()


def joints_of(df):
    """The joints this file names, in device order, with their four columns present."""
    return [
        j
        for j in joint_names_of(df.columns)
        if all(f"{p}{j}" in df.columns for p in (Q_PREFIX, Q_REF_PREFIX, G_PREFIX))
    ]


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
    """`df` with an all-NaN row inserted at every tick gap, so matplotlib leaves
    the gap blank instead of bridging it with a segment that reads as data."""
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

    A row's flag holds until the next row, so each run is extended by the row
    that follows it; a run of one row therefore has one tick of width.
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


def _shade_off(axes, df):
    """Shade the `ff_enabled = 0` stretches on all `axes`; returns legend handles."""
    if "ff_enabled" not in df.columns:
        return []
    off = ~_flag(df, "ff_enabled") & ~df["ff_enabled"].isna().to_numpy()
    if not off.any():
        return []
    t = df["timestamp"].to_numpy(dtype=float)
    for ax in axes:
        _shade(ax, t, off, _OFF_COLOR, _OFF_ALPHA)
    return [Patch(color=_OFF_COLOR, alpha=0.3, label="feedforward off")]


def _mode_codes(df):
    """Integer mode code per row, -1 where the cell is NaN (a line-break row or
    an unparsed cell). All -1 without a `mode` column."""
    if "mode" not in df.columns:
        return np.full(len(df), -1, dtype=np.int64)
    return df["mode"].fillna(-1).to_numpy(dtype=float).astype(np.int64)


def _mode_name(code):
    return MODE_NAMES.get(int(code), f"mode {int(code)}")


def _shade_modes(axes, df):
    """Shade the non-tracking `mode` stretches on all `axes`; returns legend handles.

    NaN rows (inserted by `_with_line_breaks`) belong to no mode and stay
    unshaded. Tracking is not shaded.
    """
    codes = _mode_codes(df)
    t = df["timestamp"].to_numpy(dtype=float)
    handles = []
    for code in sorted(set(codes.tolist()) - {0, -1}):
        color = _MODE_COLORS.get(code, "tab:red")
        where = codes == code
        for ax in axes:
            _shade(ax, t, where, color, _MODE_ALPHA)
        handles.append(Patch(color=color, alpha=0.3, label=_mode_name(code)))
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
    """Integer ticks and a floor at zero, for an axis that carries a count."""
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_ylim(bottom=min(ax.get_ylim()[0], -0.5), top=max(ax.get_ylim()[1], 1.5))


def _bit(series, k):
    """Boolean array: bit `k` of an integer mask column is set."""
    return ((_ints(series) >> k) & 1).astype(bool)


def _joint_grid(n, title):
    nrows, ncols = auto_subplot_grid(n)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3.2 * nrows), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight="bold")
    return fig, np.atleast_1d(axes).flatten()


def plot_joint_ff_tracking(df, save_dir=None):
    """Per joint: measured position against the reference on the wire."""
    joints = joints_of(df)
    if not joints:
        print(f"  Skipping joint FF tracking plot ({TAU_FF_PREFIX}* columns not found)")
        return

    df = _with_line_breaks(df)
    _, axes = _joint_grid(len(joints), "Joint Feedforward — Joint Tracking (q vs q_ref)")
    t = df["timestamp"]
    for ax, j in zip(axes, joints, strict=False):
        ax.plot(t, df[f"{Q_REF_PREFIX}{j}"], linewidth=1.2, color="C1", label="q_ref")
        ax.plot(t, df[f"{Q_PREFIX}{j}"], linewidth=1.0, color="C0", label="q")
        ax.set_title(j, fontsize=9)
        ax.set_ylabel("position (rad)")
        ax.grid(True, alpha=0.3)
    hide_unused_axes(axes, len(joints))
    handles = _shade_off(axes[: len(joints)], df) + _shade_modes(axes[: len(joints)], df)
    lines, labels = axes[0].get_legend_handles_labels()
    axes[0].legend(
        handles=lines + handles, labels=labels + [h.get_label() for h in handles], fontsize=8
    )
    _finish(save_dir, "joint_ff_tracking")


def plot_joint_ff_torque(df, save_dir=None):
    """Per joint: the feedforward torque on the wire next to the raw model term."""
    joints = joints_of(df)
    if not joints:
        print(f"  Skipping joint FF torque plot ({TAU_FF_PREFIX}* columns not found)")
        return

    df = _with_line_breaks(df)
    _, axes = _joint_grid(len(joints), "Joint Feedforward — Torque (tau_ff vs model terms)")
    t = df["timestamp"]
    t_np = t.to_numpy(dtype=float)
    for k, (ax, j) in enumerate(zip(axes, joints, strict=False)):
        ax.plot(
            t, df[f"{G_PREFIX}{j}"], linewidth=1.0, color="C2", linestyle="--", label="g (model)"
        )
        ax.plot(t, df[f"{TAU_FF_PREFIX}{j}"], linewidth=1.2, color="C3", label="tau_ff (wire)")
        if f"{TAU_INERTIA_PREFIX}{j}" in df.columns:
            ax.plot(
                t,
                df[f"{TAU_INERTIA_PREFIX}{j}"],
                linewidth=1.0,
                color="C0",
                linestyle=":",
                label="inertia (k_M·M·q̈)",
            )
        if "clamped_mask" in df.columns:
            clamped = _bit(df["clamped_mask"], k)
            if clamped.any():
                ax.plot(
                    t_np[clamped],
                    df[f"{TAU_FF_PREFIX}{j}"].to_numpy(dtype=float)[clamped],
                    ".",
                    markersize=3,
                    color="k",
                    label="clamped",
                )
        ax.set_title(j, fontsize=9)
        ax.set_ylabel("torque (N·m)")
        ax.grid(True, alpha=0.3)
    hide_unused_axes(axes, len(joints))
    handles = _shade_off(axes[: len(joints)], df) + _shade_modes(axes[: len(joints)], df)
    lines, labels = axes[0].get_legend_handles_labels()
    axes[0].legend(
        handles=lines + handles, labels=labels + [h.get_label() for h in handles], fontsize=8
    )
    _finish(save_dir, "joint_ff_torque")


def plot_joint_ff_status(df, save_dir=None):
    """What each tick did: the flag lanes, the hold counter, the closure error."""
    if "ff_enabled" not in df.columns:
        print("  Skipping joint FF status plot (ff_enabled not found)")
        return

    df = _with_line_breaks(df)
    has_mode = "mode" in df.columns
    fig, axes = plt.subplots(
        4 if has_mode else 3, 1, figsize=(14, 12 if has_mode else 10), sharex=True
    )
    fig.suptitle("Joint Feedforward — Status", fontsize=16, fontweight="bold")
    t = df["timestamp"]
    t_np = t.to_numpy(dtype=float)

    _raster(axes[0], t_np, [(label, _flag(df, col)) for col, label in STATUS_LANES if col in df])
    axes[0].set_ylabel("flag set")

    if "held_ticks" in df.columns:
        axes[1].step(t, df["held_ticks"], where="post", linewidth=1.2, color="C3")
    axes[1].set_ylabel("held_ticks", color="C3")
    _count_axis(axes[1])
    axes[1].grid(True, alpha=0.3)
    if "clamped_mask" in df.columns:
        ax_clamp = axes[1].twinx()
        counts = np.array([int(v).bit_count() for v in _ints(df["clamped_mask"])])
        ax_clamp.step(t, counts, where="post", linewidth=1.0, color="C0", alpha=0.8)
        ax_clamp.set_ylabel("joints clamped (set bits)", color="C0")
        _count_axis(ax_clamp)

    if "closure_error" in df.columns:
        axes[2].plot(t, df["closure_error"], linewidth=1.0, color="C4")
    axes[2].set_ylabel("closure_error")
    axes[2].grid(True, alpha=0.3)

    if has_mode:
        axes[3].step(t, df["mode"], where="post", linewidth=1.2, color="C5")
        axes[3].set_yticks(list(MODE_NAMES))
        axes[3].set_yticklabels([MODE_NAMES[k] for k in MODE_NAMES], fontsize=8)
        axes[3].set_ylim(-0.4, max(MODE_NAMES) + 0.4)
        axes[3].set_ylabel("mode")
        axes[3].grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)   —   grey band: feedforward off")

    handles = _shade_off(axes, df) + _shade_modes(axes, df)
    if handles:
        axes[1].legend(handles=handles, fontsize=8, loc="upper right")
    _finish(save_dir, "joint_ff_status")


def _segments(df):
    """(ff_on, mode, first row, last row) of each contiguous run of `ff_enabled`,
    split further where `mode` changes when the column exists (`mode` is None
    otherwise)."""
    if "ff_enabled" not in df.columns or len(df) == 0:
        return []
    on = _flag(df, "ff_enabled")
    change = np.diff(on.astype(np.int64)) != 0
    modes = None
    if "mode" in df.columns:
        modes = _mode_codes(df)
        change |= np.diff(modes) != 0
    starts = np.concatenate(([0], np.flatnonzero(change) + 1))
    ends = np.concatenate((starts[1:] - 1, [len(df) - 1]))
    return [
        (bool(on[s]), None if modes is None else int(modes[s]), int(s), int(e))
        for s, e in zip(starts, ends, strict=True)
    ]


def _drift_line(name, q, t):
    """`name: drift … rad, mean |dq/dt| … rad/s` over the rows given, or None."""
    ok = np.isfinite(q) & np.isfinite(t)
    q, t = q[ok], t[ok]
    if q.size < 2:
        return None
    dt = np.diff(t)
    valid = dt > 0
    rate = np.abs(np.diff(q))[valid] / dt[valid]
    rate_txt = f"{np.mean(rate):.5f}" if rate.size else "n/a"
    return f"    {name}: drift q(end) − q(start)={q[-1] - q[0]:+.5f} rad mean |dq/dt|={rate_txt} rad/s"


def print_joint_ff_statistics(df):
    """Console summary: segments, per-segment tracking error, flag counts."""
    print("\n=== Joint Feedforward Log ===")
    n = len(df)
    t = df["timestamp"].to_numpy(dtype=float)
    duration = float(t[-1] - t[0]) if n > 1 else 0.0
    print(f"Duration: {duration:.2f} s | Rows: {n}")
    at, missing = _tick_gaps(df)
    if len(at) == 0:
        print("Tick gaps: none — every tick has a row")
    else:
        print(
            f"Tick gaps: {len(at)} ({int(missing.sum())} ticks; largest {int(missing.max())}) "
            "— dropped rows, or a stretch this controller was not running"
        )
    for col, label in STATUS_LANES:
        if col in df.columns:
            print(f"  {label}: {int(_flag(df, col).sum())}/{n} ticks")
    if "held_ticks" in df.columns:
        print(f"  held_ticks max: {int(np.nan_to_num(df['held_ticks'].max()))}")
    if "closure_error" in df.columns:
        ce = df["closure_error"].to_numpy(dtype=float)
        if np.isfinite(ce).any():
            print(f"  closure_error max: {np.nanmax(ce):.3e}")

    joints = joints_of(df)
    segments = _segments(df)
    print(f"\nJoints ({len(joints)}): {', '.join(joints) if joints else 'none'}")
    split_by = "(ff_enabled, mode)" if "mode" in df.columns else "ff_enabled"
    print(f"Segments by {split_by} ({len(segments)}):")
    usable = _flag(df, "readable") & ~_flag(df, "estop")
    for ff_on, mode, s, e in segments:
        rows = usable[s : e + 1]
        mode_txt = "" if mode is None else f" mode={_mode_name(mode)}"
        print(
            f"  [{s}:{e}] ff {'on ' if ff_on else 'off'}{mode_txt} t={t[s]:.2f}..{t[e]:.2f} s "
            f"({t[e] - t[s]:.2f} s, {int(rows.sum())}/{e - s + 1} readable non-E-STOP rows)"
        )
        if mode not in (None, 0):
            # q_ref == q by construction here, so the tracking error says nothing.
            for j in joints:
                q = df[f"{Q_PREFIX}{j}"].to_numpy(dtype=float)[s : e + 1][rows]
                line = _drift_line(j, q, t[s : e + 1][rows])
                if line:
                    tau = np.abs(df[f"{TAU_FF_PREFIX}{j}"].to_numpy(dtype=float)[s : e + 1][rows])
                    if np.isfinite(tau).any():
                        line += f" mean |tau_ff|={np.nanmean(tau):.5f} N·m"
                    print(line)
            continue
        for j in joints:
            err = np.abs(
                df[f"{Q_REF_PREFIX}{j}"].to_numpy(dtype=float)[s : e + 1]
                - df[f"{Q_PREFIX}{j}"].to_numpy(dtype=float)[s : e + 1]
            )[rows]
            if err.size and np.isfinite(err).any():
                print(
                    f"    {j}: |q_ref − q| mean={np.nanmean(err):.5f} max={np.nanmax(err):.5f} rad"
                )
    if "clamped_mask" in df.columns:
        union = 0
        for value in np.unique(_ints(df["clamped_mask"])):
            union |= int(value)
        bits = [k for k in range(union.bit_length()) if (union >> k) & 1]
        named = [joints[k] if k < len(joints) else f"bit {k}" for k in bits]
        print(f"Clamped at least once: {', '.join(named) if named else 'none'}")
    print(
        "PASS/FAIL on the trailing windows: ros2 run rtc_tools analyze_hand_gravity_ff <this csv>"
    )
