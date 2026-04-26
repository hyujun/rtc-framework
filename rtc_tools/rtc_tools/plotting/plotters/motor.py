"""Motor plotters — auxiliary motor channels (`motor_pos_*`, `motor_vel_*`,
`motor_eff_*`) optionally present in state_log when the device exposes them.

`has_motor_columns(df)` from rtc_tools.plotting.columns gates whether these
get rendered.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from rtc_tools.plotting.columns import detect_joint_columns as _detect_joint_columns
from rtc_tools.plotting.layout import auto_subplot_grid as _auto_subplot_grid


def plot_motor_positions(df, save_dir=None):
    """Motor positions (motor_pos_*) per motor subplot."""
    cols, names = _detect_joint_columns(df, "motor_pos_")
    if not cols:
        print("  Skipping motor positions plot (motor_pos_* columns not found)")
        return

    n = len(cols)
    nrows, ncols = _auto_subplot_grid(n)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("Motor Positions", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    t = df["timestamp"]
    for i in range(n):
        ax = axes[i]
        ax.plot(t, df[cols[i]], label="Position", linewidth=1.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (rad)")
        ax.set_title(f"Motor {i}: {names[i]}")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    for idx in range(n, len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "motor_positions.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_motor_velocities(df, save_dir=None):
    """Motor velocities (motor_vel_*) per motor subplot."""
    cols, names = _detect_joint_columns(df, "motor_vel_")
    if not cols:
        print("  Skipping motor velocities plot (motor_vel_* columns not found)")
        return

    n = len(cols)
    nrows, ncols = _auto_subplot_grid(n)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("Motor Velocities", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    t = df["timestamp"]
    for i in range(n):
        ax = axes[i]
        ax.plot(t, df[cols[i]], label="Velocity", linewidth=1.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (rad/s)")
        ax.set_title(f"Motor {i}: {names[i]}")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    for idx in range(n, len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "motor_velocities.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_motor_efforts(df, save_dir=None):
    """Motor efforts/currents (motor_eff_*) per motor subplot."""
    cols, names = _detect_joint_columns(df, "motor_eff_")
    if not cols:
        print("  Skipping motor efforts plot (motor_eff_* columns not found)")
        return

    n = len(cols)
    nrows, ncols = _auto_subplot_grid(n)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("Motor Efforts (Current)", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    t = df["timestamp"]
    for i in range(n):
        ax = axes[i]
        ax.plot(t, df[cols[i]], label="Effort", linewidth=1.5, color="C3")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Effort")
        ax.set_title(f"Motor {i}: {names[i]}")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    for idx in range(n, len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "motor_efforts.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def print_motor_statistics(df):
    """Print motor state statistics if motor columns exist."""
    pos_cols, names = _detect_joint_columns(df, "motor_pos_")
    if not pos_cols:
        return

    print("\n=== Motor State Statistics ===")
    print(f"Motor channels: {len(pos_cols)}")

    print("\nMotor Positions (range):")
    for i, (col, name) in enumerate(zip(pos_cols, names)):
        data = pd.to_numeric(df[col], errors="coerce")
        print(
            f"  Motor {i} ({name}): "
            f"[{data.min():.4f}, {data.max():.4f}]  "
            f"mean={data.mean():.4f}"
        )

    vel_cols, _ = _detect_joint_columns(df, "motor_vel_")
    if vel_cols:
        print("\nMotor Velocities (range):")
        for i, (col, name) in enumerate(zip(vel_cols, names)):
            data = pd.to_numeric(df[col], errors="coerce")
            print(
                f"  Motor {i} ({name}): "
                f"[{data.min():.4f}, {data.max():.4f}]  "
                f"mean={data.mean():.4f}"
            )

    eff_cols, _ = _detect_joint_columns(df, "motor_eff_")
    if eff_cols:
        print("\nMotor Efforts (range):")
        for i, (col, name) in enumerate(zip(eff_cols, names)):
            data = pd.to_numeric(df[col], errors="coerce")
            print(
                f"  Motor {i} ({name}): "
                f"[{data.min():.4f}, {data.max():.4f}]  "
                f"mean={data.mean():.4f}"
            )

