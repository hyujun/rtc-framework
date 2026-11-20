"""Figures of the simulator ground truth the locomotion controller logs (`locomotion_truth.csv`).

A ROW EXISTS ONLY ON A TICK WHERE A TRUTH LANE WAS FRESH. The file is opened on
the first such row, so it is absent from a run with no truth lane, and `tick`
jumps over the stretches without one: that gap is "no truth", not dropped rows.
Lines are broken at a `tick` gap rather than drawn across it. A lane that was
not fresh on a row it shares with a fresh one reads NaN.

THE CONTACT LIST FOLLOWS THE HEADER. Contacts are named by the `truth_grf_<c>_fz`
columns, in header order; `truth_mask` bit 0 is the base lane and bit 1 + k the
k-th contact. No contact name is known here, and every figure grows a row per
contact.

Frames: `truth_base_*` is the base lane's raw value (the quaternion is not
normalised here either; the twist is base LOCAL). `truth_grf_<c>_*` is the
floor-on-foot wrench in the sole sensor site's axes and origin; its moments are
NaN when the sensor has no native moment, and then no moment figure is drawn.
`truth_cop_<c>_x/y` is the site-axes CoP, NaN while the normal force is below
the controller's threshold. An age of -1 means nothing was ever received.

These figures show; they do not judge. Importing this module imports
matplotlib.pyplot, so the backend must be configured first.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from rtc_tools.plotting.plotters.locomotion_diag import save_figure

BASE_BIT = 0
_FORCE_AXES = ("fx", "fy", "fz")
_MOMENT_AXES = ("tx", "ty", "tz")
_GRF_PREFIX = "truth_grf_"


def contact_names(df: pd.DataFrame) -> list[str]:
    """Contacts of the file, in header order, from the `truth_grf_<c>_fz` columns."""
    suffix = "_fz"
    return [
        c[len(_GRF_PREFIX) : -len(suffix)]
        for c in df.columns
        if c.startswith(_GRF_PREFIX) and c.endswith(suffix) and len(c) > len(_GRF_PREFIX) + 3
    ]


def _time(df: pd.DataFrame) -> np.ndarray:
    return df["t_relative_s"].to_numpy(dtype=float)


def _break_gaps(df: pd.DataFrame, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Time and values with a NaN inserted wherever `tick` skips, so a line breaks there."""
    t = _time(df)
    values = np.asarray(values, dtype=float)
    if "tick" not in df.columns or len(t) < 2:
        return t, values
    gap = np.flatnonzero(np.diff(df["tick"].to_numpy(dtype=float)) > 1)
    if gap.size == 0:
        return t, values
    at = gap + 1
    return np.insert(t, at, np.nan), np.insert(values, at, np.nan)


def _line(ax, df: pd.DataFrame, column: str, **kwargs) -> None:
    x, y = _break_gaps(df, df[column].to_numpy(dtype=float))
    ax.plot(x, y, **kwargs)


def _stack(n: int, height: float = 2.4):
    fig, axes = plt.subplots(
        max(1, n), 1, figsize=(10, height * max(1, n) + 1), sharex=True, squeeze=False
    )
    return fig, axes[:, 0]


def _finish(fig, axes, title: str, path: Path) -> None:
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(title)
    save_figure(fig, path)


def yaw_rad(qx, qy, qz, qw) -> np.ndarray:
    """Unwrapped yaw of an (x, y, z, w) quaternion, ZYX convention."""
    q = np.stack([qx, qy, qz, qw]).astype(float)
    norm = np.linalg.norm(q, axis=0)
    norm[norm == 0.0] = np.nan
    x, y, z, w = q / norm
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    finite = np.isfinite(yaw)
    out = np.full_like(yaw, np.nan)
    out[finite] = np.unwrap(yaw[finite])
    return out


def plot_truth_base_pose(df: pd.DataFrame, path: Path) -> None:
    fig, axes = _stack(3)
    for axis in "xyz":
        _line(axes[0], df, f"truth_base_p{axis}", label=axis)
    axes[0].set_ylabel("position [m]")
    axes[0].legend(fontsize=8, ncol=3)
    cols = [df[f"truth_base_q{a}"].to_numpy(dtype=float) for a in "xyzw"]
    qx, qy = cols[0], cols[1]
    tilt = np.arccos(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0))
    x, y = _break_gaps(df, tilt)
    axes[1].plot(x, y)
    axes[1].set_ylabel("tilt [rad]")
    x, y = _break_gaps(df, yaw_rad(*cols))
    axes[2].plot(x, y)
    axes[2].set_ylabel("yaw [rad]")
    _finish(fig, axes, "Truth: base pose (raw lane)", path)


def plot_truth_base_twist(df: pd.DataFrame, path: Path) -> None:
    fig, axes = _stack(2)
    for axis in "xyz":
        _line(axes[0], df, f"truth_base_v{axis}", label=axis)
        _line(axes[1], df, f"truth_base_w{axis}", label=axis)
    axes[0].set_ylabel("linear [m/s]")
    axes[1].set_ylabel("angular [rad/s]")
    for ax in axes:
        ax.legend(fontsize=8, ncol=3)
    _finish(fig, axes, "Truth: base twist (base local)", path)


def _plot_wrench(df: pd.DataFrame, axes_names, unit: str, title: str, path: Path) -> None:
    contacts = contact_names(df)
    fig, axes = _stack(len(contacts))
    for ax, c in zip(axes, contacts, strict=False):
        for a in axes_names:
            col = f"{_GRF_PREFIX}{c}_{a}"
            if col in df.columns:
                _line(ax, df, col, label=a)
        ax.set_ylabel(f"{c}\n[{unit}]", fontsize=8)
        ax.legend(fontsize=7, ncol=3)
    _finish(fig, axes, title, path)


def plot_truth_grf(df: pd.DataFrame, path: Path) -> None:
    _plot_wrench(
        df, _FORCE_AXES, "N", "Truth: ground reaction force (floor on foot, sole axes)", path
    )


def plot_truth_grm(df: pd.DataFrame, path: Path) -> None:
    _plot_wrench(df, _MOMENT_AXES, "N m", "Truth: ground reaction moment (sole site origin)", path)


def plot_truth_cop(df: pd.DataFrame, path: Path) -> None:
    contacts = contact_names(df)
    fig, axes = _stack(len(contacts))
    for ax, c in zip(axes, contacts, strict=False):
        for a in "xy":
            col = f"truth_cop_{c}_{a}"
            if col in df.columns:
                _line(ax, df, col, label=a)
        ax.set_ylabel(f"{c}\nCoP [m]", fontsize=8)
        ax.legend(fontsize=7, ncol=2)
    _finish(
        fig, axes, "Truth: centre of pressure (sole axes; NaN below the force threshold)", path
    )


def plot_truth_ages(df: pd.DataFrame, path: Path) -> None:
    lanes = [("base", "truth_base_age_us")] + [
        (c, f"truth_age_{c}_us") for c in contact_names(df) if f"truth_age_{c}_us" in df.columns
    ]
    fig, axes = _stack(1, height=4.0)
    for name, col in lanes:
        age = df[col].to_numpy(dtype=float)
        age = np.where(age < 0.0, np.nan, age)  # -1 = nothing received
        x, y = _break_gaps(df, age)
        axes[0].plot(x, y, label=name)
    axes[0].set_ylabel("age [us]")
    axes[0].legend(fontsize=8)
    _finish(fig, axes, "Truth: lane ages (-1 = never received, not drawn)", path)


def plot_truth_mask(df: pd.DataFrame, path: Path) -> None:
    lanes = ["base", *contact_names(df)]
    mask = df["truth_mask"].to_numpy(dtype=float)
    fig, axes = _stack(len(lanes), height=1.2)
    for bit, (ax, name) in enumerate(zip(axes, lanes, strict=True)):
        fresh = np.where(
            np.isfinite(mask), (np.nan_to_num(mask).astype(np.int64) >> bit) & 1, np.nan
        )
        ax.step(_time(df), fresh, where="post")
        ax.set_yticks([0, 1])
        ax.set_ylim(-0.1, 1.1)
        ax.set_ylabel(name, fontsize=8)
    _finish(
        fig,
        axes,
        "Truth: fresh lanes per row (truth_mask bit; base = 0, contacts in header order)",
        path,
    )


# ── plot_rtc_log pipeline adapters ─────────────────────────────────────────


def _adapter(draw, name: str):
    def run(df: pd.DataFrame, save_dir) -> None:
        if save_dir:
            draw(df, Path(save_dir) / f"{name}.png")

    run.__name__ = f"plot_{name}"
    return run


plot_truth_base_pose_auto = _adapter(plot_truth_base_pose, "truth_base_pose")
plot_truth_base_twist_auto = _adapter(plot_truth_base_twist, "truth_base_twist")
plot_truth_grf_auto = _adapter(plot_truth_grf, "truth_grf")
plot_truth_grm_auto = _adapter(plot_truth_grm, "truth_grm")
plot_truth_cop_auto = _adapter(plot_truth_cop, "truth_cop")
plot_truth_ages_auto = _adapter(plot_truth_ages, "truth_ages")
plot_truth_mask_auto = _adapter(plot_truth_mask, "truth_mask")
