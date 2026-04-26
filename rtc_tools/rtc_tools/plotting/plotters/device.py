"""Hand device motor-side plotters + statistics: positions, velocities,
sensor summary print.

Sensor (baro/ToF/FT) plots live in plotters/sensors.py.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from rtc_tools.plotting.columns import (
    detect_ft_labels as _detect_ft_labels,
    detect_joint_columns as _detect_joint_columns,
)
from rtc_tools.plotting.layout import auto_subplot_grid as _auto_subplot_grid


def plot_device_positions(df, save_dir=None):
    """Figure 1: Hand motor positions (2x5 subplot)."""
    actual_cols, motor_names = _detect_joint_columns(df, "hand_actual_pos_")
    cmd_cols, _ = _detect_joint_columns(df, "hand_cmd_")
    goal_cols, _ = _detect_joint_columns(df, "hand_goal_pos_")
    if not actual_cols:
        print("  Skipping hand positions plot (columns not found)")
        return

    n_motors = len(actual_cols)
    nrows, ncols_g = _auto_subplot_grid(n_motors)
    fig, axes = plt.subplots(nrows, ncols_g, figsize=(5 * ncols_g, 4 * nrows))
    fig.suptitle("Device Motor Positions", fontsize=16, fontweight="bold")
    axes = axes.flatten()

    t = df["timestamp"]
    for i in range(len(actual_cols)):
        ax = axes[i]
        ax.plot(t, df[actual_cols[i]], label="Actual", linewidth=1.5)
        if i < len(cmd_cols):
            ax.plot(t, df[cmd_cols[i]], label="Command", linestyle="--", linewidth=1.5)
        if i < len(goal_cols):
            ax.plot(
                t,
                df[goal_cols[i]],
                label="Goal",
                linestyle=":",
                linewidth=1.5,
                alpha=0.8,
            )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position")
        ax.set_title(f"Motor {i} ({motor_names[i]})")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "device_positions.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_device_velocities(df, save_dir=None):
    """Figure 2: Hand motor velocities (2x5 subplot)."""
    vel_cols, motor_names = _detect_joint_columns(df, "hand_actual_vel_")
    if not vel_cols:
        print("  Skipping hand velocities plot (columns not found)")
        return

    n_motors = len(vel_cols)
    nrows, ncols_g = _auto_subplot_grid(n_motors)
    fig, axes = plt.subplots(nrows, ncols_g, figsize=(5 * ncols_g, 4 * nrows))
    fig.suptitle("Device Motor Velocities", fontsize=16, fontweight="bold")
    axes = axes.flatten()

    t = df["timestamp"]
    for i in range(len(vel_cols)):
        ax = axes[i]
        ax.plot(t, df[vel_cols[i]], label="Actual", linewidth=1.5)

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity")
        ax.set_title(f"Motor {i} ({motor_names[i]})")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "device_velocities.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()

def print_device_statistics(df):
    """Print hand trajectory statistics."""
    duration = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]
    rate = len(df) / duration if duration > 0 else 0.0
    print("\n=== Hand Trajectory Statistics ===")
    print(f"Duration: {duration:.2f} s | Samples: {len(df)} | Rate: {rate:.1f} Hz")

    if "hand_valid" in df.columns:
        valid_ratio = df["hand_valid"].sum() / len(df) * 100
        print(f"Hand valid: {valid_ratio:.1f}%")

    goal_cols, _ = _detect_joint_columns(df, "hand_goal_pos_")
    actual_cols, motor_names = _detect_joint_columns(df, "hand_actual_pos_")
    if goal_cols and actual_cols:
        print("\nPosition Tracking Error (RMS):")
        for i in range(min(len(goal_cols), len(actual_cols))):
            goal_s = pd.to_numeric(df[goal_cols[i]], errors="coerce")
            act_s = pd.to_numeric(df[actual_cols[i]], errors="coerce")
            err = goal_s - act_s
            rms = np.sqrt(np.nanmean(err**2))
            print(f"  Motor {i} ({motor_names[i]}): {rms:.6f}")

    # Sensor summary (filtered 컬럼만, raw 제외)
    baro_cols = [
        c for c in df.columns if c.startswith("baro_") and not c.startswith("baro_raw_")
    ]
    tof_cols = [
        c for c in df.columns if c.startswith("tof_") and not c.startswith("tof_raw_")
    ]
    if baro_cols:
        baro_vals = df[baro_cols].values.flatten()
        print(f"\nBarometer range: [{baro_vals.min():.1f}, {baro_vals.max():.1f}]")
    if tof_cols:
        tof_vals = df[tof_cols].values.flatten()
        print(f"ToF range: [{tof_vals.min():.1f}, {tof_vals.max():.1f}]")

    # FT inference output statistics
    ft_labels = _detect_ft_labels(df)
    if ft_labels:
        # 3-head 모델 여부 감지 (contact 컬럼 존재)
        has_contact = f"ft_{ft_labels[0]}_contact" in df.columns
        if has_contact:
            ft_comps = ["contact", "fx", "fy", "fz", "ux", "uy", "uz"]
        else:
            ft_comps = ["fx", "fy", "fz", "tx", "ty", "tz"]
        print("\nF/T Inference Output:")
        if "ft_valid" in df.columns:
            ft_valid_ratio = df["ft_valid"].sum() / len(df) * 100
            print(f"  FT valid: {ft_valid_ratio:.1f}%")
        for label in ft_labels:
            print(f"  {label}:")
            for comp in ft_comps:
                col = f"ft_{label}_{comp}"
                if col in df.columns:
                    data = df[col]
                    print(
                        f"    {comp}: mean={data.mean():.4f}"
                        f"  std={data.std():.4f}"
                        f"  min={data.min():.4f}"
                        f"  max={data.max():.4f}"
                    )

