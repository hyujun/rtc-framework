"""WBC plotters for the controller-owned DeviceWbcLog / WbcDiagLog CSVs.

DeviceWbcLog is a superset of state_log, so the generic robot/motor plotters
still apply to it (positions, velocities, torques, …). These functions cover
the WBC-only fields the generic plotters don't know about:

  - TSID acceleration (`accel_*`)                 — per-joint q̈ from the QP
  - SE3 trajectory tracking (`traj_task_pos_*`)   — arm role
  - per-fingertip |F| (`fingertip_force_*`)       — hand role

WbcDiagLog is the per-tick TSID/QP diagnostics file (single `wbc_diag.csv`):

  - solver health (`solve_time_us`, `qp_converged`, `solve_levels`, …)
  - QP contact wrench solution (`lambda_*`) + grasp aggregates

`plot_*` functions render a figure and either save to `save_dir` or
`plt.show()`. They print "Skipping …" and return when required columns are
missing, mirroring the robot/sensor plotters.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rtc_tools.plotting.columns import (
    detect_joint_columns as _detect_joint_columns,
    has_columns as _has_columns,
)
from rtc_tools.plotting.layout import auto_subplot_grid as _auto_subplot_grid

# SE3 task-space axis order emitted by the DeviceWbcLog writer.
_TASK_AXES = ("x", "y", "z", "roll", "pitch", "yaw")


def _task_col(df, prefix, i):
    """Resolve i-th task-space axis column: named (`task_pos_x`) or numeric."""
    named = f"{prefix}{_TASK_AXES[i]}"
    if named in df.columns:
        return named
    return f"{prefix}{i}"  # legacy numeric fallback


def plot_wbc_accelerations(df, save_dir=None):
    """Per-joint TSID solution acceleration q̈ (`accel_*`)."""
    accel_cols, display_names = _detect_joint_columns(df, "accel_")
    n_joints = len(accel_cols)
    if n_joints == 0:
        print("  Skipping WBC acceleration plot (accel_* columns not found)")
        return

    nrows, ncols = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("WBC Joint Accelerations (TSID a_opt)", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    t = df["timestamp"]
    for i in range(n_joints):
        ax = axes[i]
        ax.plot(t, df[accel_cols[i]], label="a_opt", linewidth=1.2, color="C3")
        ax.set_title(display_names[i], fontsize=10)
        ax.set_ylabel("accel (rad/s²)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    for j in range(n_joints, len(axes)):
        axes[j].axis("off")
    axes[min(n_joints, len(axes)) - 1].set_xlabel("Time (s)")

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "wbc_accelerations.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_wbc_task_trajectory(df, save_dir=None):
    """SE3 task tracking (6x1): actual vs goal vs trajectory setpoint.

    Arm-role DeviceWbcLog only. Overlays the quintic SE3 ramp setpoint
    (`traj_task_pos_*`) on top of the actual (`task_pos_*`) and goal
    (`task_goal_*`) poses.
    """
    if not _has_columns(df, "traj_task_pos_", 3) or not _has_columns(df, "task_pos_", 3):
        print("  Skipping WBC task trajectory plot (traj_task_pos_*/task_pos_* not found)")
        return

    units = ("m", "m", "m", "rad", "rad", "rad")
    fig, axes = plt.subplots(6, 1, figsize=(14, 16))
    fig.suptitle("WBC SE3 Task Tracking", fontsize=16, fontweight="bold")

    has_task_goal = _has_columns(df, "task_goal_", 3)
    t = df["timestamp"]
    for i in range(6):
        ax = axes[i]
        ax.plot(t, df[_task_col(df, "task_pos_", i)], label="Actual", linewidth=1.5)
        ax.plot(
            t,
            df[_task_col(df, "traj_task_pos_", i)],
            label="Trajectory",
            linestyle="--",
            linewidth=1.5,
            color="C1",
        )
        if has_task_goal:
            ax.plot(
                t,
                df[_task_col(df, "task_goal_", i)],
                label="Goal",
                linestyle=":",
                linewidth=1.5,
                alpha=0.8,
                color="C2",
            )
        ax.set_ylabel(f"{_TASK_AXES[i]} ({units[i]})")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "wbc_task_trajectory.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_wbc_fingertip_force(df, save_dir=None):
    """Per-fingertip contact force magnitude |F| (`fingertip_force_*`).

    Hand-role DeviceWbcLog only. One overlaid line per fingertip.
    """
    force_cols, display_names = _detect_joint_columns(df, "fingertip_force_")
    if not force_cols:
        print("  Skipping WBC fingertip force plot (fingertip_force_* not found)")
        return

    fig, ax = plt.subplots(figsize=(14, 6))
    fig.suptitle("WBC Fingertip Contact Force |F|", fontsize=16, fontweight="bold")
    t = df["timestamp"]
    for col, name in zip(force_cols, display_names, strict=False):
        ax.plot(t, df[col], label=name, linewidth=1.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|F| (N)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "wbc_fingertip_force.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_wbc_diag_solver(df, save_dir=None):
    """WbcDiagLog solver health: solve time, levels, fail count, convergence."""
    if "solve_time_us" not in df.columns:
        print("  Skipping WBC solver diag plot (solve_time_us not found)")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    fig.suptitle("WBC TSID/QP Solver Diagnostics", fontsize=16, fontweight="bold")
    t = df["timestamp"]

    axes[0].plot(t, df["solve_time_us"], linewidth=1.2, color="C0")
    axes[0].set_ylabel("solve time (µs)")
    axes[0].grid(True, alpha=0.3)

    if "qp_converged" in df.columns:
        axes[1].plot(t, df["qp_converged"], linewidth=1.2, color="C2", label="qp_converged")
    if "solve_levels" in df.columns:
        axes[1].plot(t, df["solve_levels"], linewidth=1.0, color="C1", label="solve_levels")
    axes[1].set_ylabel("converged / levels")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    if "qp_fail_count" in df.columns:
        axes[2].plot(t, df["qp_fail_count"], linewidth=1.2, color="C3")
    axes[2].set_ylabel("qp_fail_count")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "wbc_diag_solver.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_wbc_diag_contacts(df, save_dir=None):
    """WbcDiagLog contact solution: λ wrench, active count, max force, grasp."""
    lambda_cols, lambda_names = _detect_joint_columns(df, "lambda_")
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    fig.suptitle("WBC Contact / Grasp Diagnostics", fontsize=16, fontweight="bold")
    t = df["timestamp"]

    if lambda_cols:
        for col, name in zip(lambda_cols, lambda_names, strict=False):
            axes[0].plot(t, df[col], label=name, linewidth=0.9)
        axes[0].legend(fontsize=7, ncol=4)
    else:
        axes[0].text(0.5, 0.5, "no lambda_* columns", ha="center", transform=axes[0].transAxes)
    axes[0].set_ylabel("λ (contact wrench)")
    axes[0].grid(True, alpha=0.3)

    if "num_active_contacts" in df.columns:
        axes[1].plot(
            t, df["num_active_contacts"], label="num_active_contacts", linewidth=1.2, color="C0"
        )
    if "max_force" in df.columns:
        axes[1].plot(t, df["max_force"], label="max_force (N)", linewidth=1.2, color="C3")
    if "grasp_detected" in df.columns:
        axes[1].plot(
            t, df["grasp_detected"], label="grasp_detected", linewidth=1.0, color="C2", alpha=0.7
        )
    axes[1].set_ylabel("contacts / force")
    axes[1].set_xlabel("Time (s)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "wbc_diag_contacts.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def print_wbc_diag_statistics(df):
    """Console summary of WbcDiagLog solver + grasp aggregates."""
    print("\n=== WBC TSID/QP Diagnostics ===")
    n = len(df)
    print(f"Samples: {n}")
    if "solve_time_us" in df.columns:
        s = df["solve_time_us"]
        print(f"Solve time (µs): mean={s.mean():.1f} p50={s.median():.1f} " f"max={s.max():.1f}")
    if "qp_converged" in df.columns and n > 0:
        conv = df["qp_converged"].astype(float)
        print(f"QP converged: {conv.mean() * 100:.1f}% of ticks")
    if "qp_fail_count" in df.columns:
        print(f"QP fail count (final): {int(df['qp_fail_count'].iloc[-1])}")
    if "num_active_contacts" in df.columns:
        print(
            f"Active contacts: mean={df['num_active_contacts'].mean():.2f} max={int(df['num_active_contacts'].max())}"
        )
    if "max_force" in df.columns:
        print(f"Max force (N): peak={df['max_force'].max():.2f}")
    if "grasp_detected" in df.columns and n > 0:
        print(f"Grasp detected: {df['grasp_detected'].astype(float).mean() * 100:.1f}% of ticks")
    if "phase" in df.columns:
        phases = df["phase"].astype(str).value_counts()
        summary = ", ".join(f"{p}:{c}" for p, c in phases.items())
        print(f"Phase histogram: {summary}")
