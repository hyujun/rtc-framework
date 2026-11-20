"""Figures of the locomotion controller's per-tick diagnostics (`locomotion_diag.csv`).

These functions are the ONE implementation of the locomotion-diag figures: the
`locomotion_diag` pipeline of `plot_rtc_log` and the `plot_locomotion_diag`
console entry (`rtc_tools.analysis.locomotion_diag`, which also owns the loader,
the derived series and the JSON summary) both call them. They take a
`Session`, so the same drawing code serves a session directory (joint_ff and
state CSVs joined on `tick`) and the single CSV `plot_rtc_log` is handed.

Contact and joint names come from the header (`lambda_<c>_fz`,
`tau_servo_<j>`), never from a list here. Importing this module imports
matplotlib.pyplot, so the backend must be configured first.
"""

import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from rtc_tools.analysis.locomotion_diag import (
    FAULT_CAUSE_NAMES,
    FIGURE_FILES,
    HEALTH_NAMES,
    JOINT_FF_FILE,
    MARGINS,
    MODE_NAMES,
    REFERENCE_SOURCE_NAMES,
    SOURCE_STATUS_NAMES,
    Session,
    base_series,
    foot_slip,
    margin_series,
    parse_diag_names,
)


def grid(n: int, ncols: int = 2):
    nrows = max(1, math.ceil(n / ncols))
    fig, axes = plt.subplots(
        nrows, ncols, figsize=(6.5 * ncols, 2.4 * nrows + 1), sharex=True, squeeze=False
    )
    return fig, axes.ravel()


def note(fig, text: str) -> None:
    fig.text(0.5, 0.5, text, ha="center", va="center")


def save_figure(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_base(sess: Session, z_ref: float | None, path: Path) -> None:
    t = sess.diag["t_relative_s"]
    base = base_series(sess.diag, z_ref)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    label = "height - z_ref [m]" if z_ref is not None else "height - first sample [m]"
    axes[0].plot(t, base["base_height_dev_m"])
    axes[0].set_ylabel(label)
    axes[1].plot(t, base["base_xy_drift_m"])
    axes[1].set_ylabel("xy drift [m]")
    axes[2].plot(t, base["base_tilt_rad"])
    axes[2].set_ylabel("tilt [rad]")
    axes[2].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("Base pose")
    save_figure(fig, path)


def plot_contacts(sess: Session, stance: list | None, path: Path) -> None:
    t = sess.diag["t_relative_s"]
    nrows = 2 if stance else 1
    fig, axes = plt.subplots(nrows, 1, figsize=(10, 4 * nrows), sharex=True, squeeze=False)
    ax = axes[0, 0]
    for c in sess.diag_names.contacts:
        ax.plot(t, sess.diag[f"lambda_{c}_fz"], label=f"QP {c}")
    if stance:
        ts = np.array([s.t for s in stance]) - stance[0].t + t[0]
        for i in range(len(stance[0].loads)):
            ax.plot(ts, [s.loads[i] for s in stance], "--", label=f"recorded foot{i}")
        slip_ax = axes[1, 0]
        for i in range(len(stance[0].feet)):
            xy = np.array([s.feet[i][:2] for s in stance])
            slip_ax.plot(ts, foot_slip(xy), label=f"foot{i}")
        slip_ax.set_ylabel("foot xy slip [m]")
        slip_ax.legend(fontsize=8)
        slip_ax.grid(alpha=0.3)
    ax.set_ylabel("normal force [N]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    axes[-1, 0].set_xlabel("t [s]")
    fig.suptitle("Contact normal forces")
    save_figure(fig, path)


def plot_joint_torque(sess: Session, limits: dict[str, float] | None, path: Path) -> None:
    if sess.ff is None:
        fig, _ = grid(1)
        note(fig, f"{JOINT_FF_FILE} not found")
        save_figure(fig, path)
        return
    t = sess.diag["t_relative_s"]
    fig, axes = grid(len(sess.ff_joints))
    for ax, j in zip(axes, sess.ff_joints, strict=False):
        ax.plot(t, sess.ff[f"tau_ff_{j}"], label="tau_ff")
        if j in sess.effort:
            ax.plot(t, sess.effort[j], label="measured effort", alpha=0.8)
        if limits and j in limits:
            ax.axhline(limits[j], color="r", ls=":", lw=1, label="+/- max_torque")
            ax.axhline(-limits[j], color="r", ls=":", lw=1)
        ax.set_title(j, fontsize=9)
        ax.set_ylabel("[N m]")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.suptitle("Joint torque: feedforward vs measured")
    save_figure(fig, path)


def plot_servo_tracking(sess: Session, path: Path) -> None:
    t = sess.diag["t_relative_s"]
    joints = sess.diag_names.joints
    fig, axes = plt.subplots(
        max(1, len(joints)),
        2,
        figsize=(13, 2.2 * max(1, len(joints)) + 1),
        sharex=True,
        squeeze=False,
    )
    for row, j in zip(axes, joints, strict=False):
        if sess.ff is not None and f"q_ref_{j}" in sess.ff and f"q_{j}" in sess.ff:
            row[0].plot(t, sess.ff[f"q_ref_{j}"] - sess.ff[f"q_{j}"])
        row[0].set_ylabel(f"{j}\nq_ref - q [rad]", fontsize=8)
        row[1].plot(t, sess.diag[f"tau_servo_{j}"])
        row[1].set_ylabel("tau_servo [N m]", fontsize=8)
        for ax in row:
            ax.grid(alpha=0.3)
    if sess.ff is None:
        fig.text(0.25, 0.97, f"{JOINT_FF_FILE} not found", ha="center")
    axes[-1, 0].set_xlabel("t [s]")
    axes[-1, 1].set_xlabel("t [s]")
    fig.suptitle("Servo tracking error and servo torque")
    save_figure(fig, path)


def plot_qp(sess: Session, path: Path) -> None:
    t = sess.diag["t_relative_s"]
    d = sess.diag
    fig, axes = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
    axes[0].step(t, d["qp_status"], where="post")
    axes[0].set_ylabel("qp_status")
    axes[1].step(t, d["qp_iterations"], where="post")
    axes[1].set_ylabel("qp_iterations")
    axes[2].plot(t, d["solve_time_us"])
    axes[2].set_ylabel("solve_time_us")
    axes[3].plot(t, d["wbc_tick_us"])
    axes[3].set_ylabel("wbc_tick_us")
    axes[4].step(t, d["tau_valid"], where="post")
    axes[4].set_ylabel("tau_valid")
    axes[4].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("QP timeline")
    save_figure(fig, path)


def plot_margins(sess: Session, path: Path) -> None:
    t = sess.diag["t_relative_s"]
    stale = sess.diag["margins_stale"] > 0.5
    fig, axes = plt.subplots(len(MARGINS), 1, figsize=(10, 2.2 * len(MARGINS) + 1), sharex=True)
    for ax, m, live in zip(axes, MARGINS, margin_series(sess.diag).values(), strict=True):
        ax.plot(t, live)
        ax.axhline(0.0, color="r", lw=1)
        ax.fill_between(
            t,
            0,
            1,
            where=stale,
            transform=ax.get_xaxis_transform(),
            color="grey",
            alpha=0.25,
            lw=0,
        )
        ax.set_ylabel(m, fontsize=8)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("t [s]  (grey = stale rows)")
    fig.suptitle("Margins (positive = inside)")
    save_figure(fig, path)


def plot_modes(sess: Session, path: Path) -> None:
    t = sess.diag["t_relative_s"]
    specs = (
        ("mode", MODE_NAMES),
        ("fault_cause", FAULT_CAUSE_NAMES),
        ("health", HEALTH_NAMES),
        ("source_status", SOURCE_STATUS_NAMES),
    )
    if "ref_source" in sess.diag:
        specs = (*specs, ("ref_source", REFERENCE_SOURCE_NAMES))
    fig, axes = plt.subplots(len(specs), 1, figsize=(10, 8), sharex=True)
    for ax, (col, names) in zip(axes, specs, strict=True):
        ax.step(t, sess.diag[col], where="post")
        ax.set_yticks(range(len(names)))
        ax.set_yticklabels(names, fontsize=8)
        ax.set_ylabel(col, fontsize=8)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Mode, fault cause, health, source status (and reference source)")
    save_figure(fig, path)


# ── plot_rtc_log pipeline adapters ─────────────────────────────────────────
#
# `plot_rtc_log` hands a pipeline one CSV, so what the session loader joins in
# (joint_ff.csv, the state CSV, the stance recorder, a torque-limit YAML) is
# absent here: the adapters draw the figures that need the diag CSV alone. The
# joint-torque figure is only a "not found" note without joint_ff, so it has no
# adapter; the servo-tracking figure keeps its torque column and leaves the
# tracking-error column empty. Each file is `FIGURE_FILES[i]`, and the pipeline
# entry carries that stem as its name.


def session_from_frame(df: pd.DataFrame) -> Session:
    """A `Session` over the numeric columns of one loaded `locomotion_diag.csv`."""
    diag = {str(c): df[c].to_numpy(dtype=float) for c in df.columns}
    return Session(diag=diag, diag_names=parse_diag_names(list(df.columns)))


def _adapter(index: int, draw):
    filename = FIGURE_FILES[index]

    def run(df: pd.DataFrame, save_dir) -> None:
        if not save_dir:
            return
        draw(session_from_frame(df), Path(save_dir) / filename)

    run.__name__ = f"plot_locomotion_diag_{Path(filename).stem}"
    return run


plot_locomotion_base = _adapter(0, lambda sess, path: plot_base(sess, None, path))
plot_locomotion_contacts = _adapter(1, lambda sess, path: plot_contacts(sess, None, path))
plot_locomotion_servo_tracking = _adapter(3, plot_servo_tracking)
plot_locomotion_qp = _adapter(4, plot_qp)
plot_locomotion_margins = _adapter(5, plot_margins)
plot_locomotion_modes = _adapter(6, plot_modes)
