"""Robot state_log plotters: positions, velocities, commands, torques,
task-space position, tracking errors, statistics.

`plot_*` functions render a figure (or print "Skipping ..." if columns are
missing) and either save to `save_dir` or call `plt.show()`.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from rtc_tools.plotting.columns import (
    detect_joint_columns as _detect_joint_columns,
    has_columns as _has_columns,
)
from rtc_tools.plotting.layout import auto_subplot_grid as _auto_subplot_grid


def plot_robot_positions(df, save_dir=None):
    """Figure 1: Robot joint positions — goal vs trajectory vs actual."""
    actual_cols, display_names = _detect_joint_columns(df, "actual_pos_")
    n_joints = len(actual_cols)
    if n_joints == 0:
        print("  Skipping position plot (actual_pos_* columns not found)")
        return

    nrows, ncols = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("Robot Joint Positions", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    goal_cols, _ = _detect_joint_columns(df, "goal_pos_")
    traj_cols, _ = _detect_joint_columns(df, "traj_pos_")
    # 하위 호환: 이전 CSV의 target_pos_ 컬럼 지원
    legacy_cols, _ = (
        _detect_joint_columns(df, "target_pos_") if not traj_cols else ([], [])
    )

    # GUI joint goal overlay: goal_type=="joint" 구간만 표시
    joint_goal_cols, _ = _detect_joint_columns(df, "joint_goal_")
    has_goal_type = "goal_type" in df.columns
    joint_goal_mask = None
    if joint_goal_cols and has_goal_type:
        joint_goal_mask = df["goal_type"] == "joint"

    for i in range(n_joints):
        ax = axes[i]
        t = df["timestamp"]
        ax.plot(t, df[actual_cols[i]], label="Actual", linewidth=1.5)

        if traj_cols:
            ax.plot(
                t, df[traj_cols[i]], label="Trajectory", linestyle="--", linewidth=1.5
            )
        elif legacy_cols:
            ax.plot(
                t,
                df[legacy_cols[i]],
                label="Target (traj)",
                linestyle="--",
                linewidth=1.5,
            )

        if goal_cols:
            ax.plot(
                t,
                df[goal_cols[i]],
                label="Goal",
                linestyle=":",
                linewidth=1.5,
                alpha=0.8,
            )

        # Joint goal from GUI (goal_type=="joint" 구간만)
        if joint_goal_cols and i < len(joint_goal_cols) and joint_goal_mask is not None:
            jg = df[joint_goal_cols[i]].copy()
            jg[~joint_goal_mask] = np.nan
            ax.plot(
                t,
                jg,
                label="Joint Goal (GUI)",
                linestyle="-.",
                linewidth=1.5,
                alpha=0.7,
                color="C4",
            )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (rad)")
        ax.set_title(f"Joint {i}: {display_names[i]}")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "robot_positions.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_robot_velocities(df, save_dir=None):
    """Figure 2: Robot joint velocities — trajectory vs actual."""
    actual_cols, display_names = _detect_joint_columns(df, "actual_vel_")
    n_joints = len(actual_cols)
    if n_joints == 0:
        print("  Skipping velocity plot (actual_vel_* columns not found)")
        return

    nrows, ncols = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("Robot Joint Velocities", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    traj_cols, _ = _detect_joint_columns(df, "traj_vel_")
    legacy_cols, _ = (
        _detect_joint_columns(df, "target_vel_") if not traj_cols else ([], [])
    )

    for i in range(n_joints):
        ax = axes[i]
        t = df["timestamp"]
        ax.plot(t, df[actual_cols[i]], label="Actual", linewidth=1.5)

        if traj_cols:
            ax.plot(
                t, df[traj_cols[i]], label="Trajectory", linestyle="--", linewidth=1.5
            )
        elif legacy_cols:
            ax.plot(
                t,
                df[legacy_cols[i]],
                label="Target (traj)",
                linestyle="--",
                linewidth=1.5,
            )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (rad/s)")
        ax.set_title(f"Joint {i}: {display_names[i]}")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "robot_velocities.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_robot_commands(df, save_dir=None):
    """Figure 3: Robot control commands — command output per joint."""
    cmd_cols, display_names = _detect_joint_columns(df, "command_")
    n_joints = len(cmd_cols)
    if n_joints == 0:
        print("  Skipping command plot (command_* columns not found)")
        return

    nrows, ncols_grid = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols_grid, figsize=(5 * ncols_grid, 4 * nrows))

    # command_type: 0=position, 1=torque
    has_type = "command_type" in df.columns
    if has_type:
        cmd_type = df["command_type"].mode().iloc[0] if len(df) > 0 else 0
        type_label = "Position" if cmd_type == 0 else "Torque"
        unit = "rad" if cmd_type == 0 else "Nm"
    else:
        type_label = "Command"
        unit = ""

    fig.suptitle(
        f"Robot Control Commands ({type_label})", fontsize=16, fontweight="bold"
    )
    axes = np.atleast_1d(axes).flatten()

    actual_cols, _ = _detect_joint_columns(df, "actual_pos_")

    for i in range(n_joints):
        ax = axes[i]
        t = df["timestamp"]
        ax.plot(t, df[cmd_cols[i]], label=type_label, linewidth=1.5, color="C2")
        if actual_cols:
            ax.plot(
                t,
                df[actual_cols[i]],
                label="Actual pos",
                linewidth=1.0,
                alpha=0.5,
                linestyle="--",
            )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"{type_label} ({unit})" if unit else type_label)
        ax.set_title(f"Joint {i}: {display_names[i]}")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "robot_commands.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_robot_torques(df, save_dir=None):
    """Figure 4: Robot actual torques."""
    torque_cols, display_names = _detect_joint_columns(df, "actual_torque_")
    if not torque_cols:
        torque_cols, display_names = _detect_joint_columns(df, "effort_")
    n_joints = len(torque_cols)
    if n_joints == 0:
        print("  Skipping torque plot (actual_torque_*/effort_* columns not found)")
        return

    nrows, ncols = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle("Robot Actual Torques", fontsize=16, fontweight="bold")
    axes = np.atleast_1d(axes).flatten()

    for i in range(n_joints):
        ax = axes[i]
        t = df["timestamp"]
        ax.plot(t, df[torque_cols[i]], label="Torque", linewidth=1.5, color="C3")

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Torque (Nm)")
        ax.set_title(f"Joint {i}: {display_names[i]}")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "robot_torques.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_robot_task_position(df, save_dir=None):
    """Figure 5: TCP task-space position (3x1: x, y, z)."""
    if not _has_columns(df, "task_pos_", 3):
        print("  Skipping task position plot (task_pos_* columns not found)")
        return

    labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle("TCP Task-Space Position", fontsize=16, fontweight="bold")

    # Task goal overlay: goal_type=="task" 구간만 표시
    has_goal_type = "goal_type" in df.columns
    has_task_goal = _has_columns(df, "task_goal_", 3)
    task_goal_mask = None
    if has_task_goal and has_goal_type:
        task_goal_mask = df["goal_type"] == "task"

    has_traj_task = _has_columns(df, "traj_task_pos_", 3)

    t = df["timestamp"]
    for i in range(3):
        ax = axes[i]
        ax.plot(t, df[f"task_pos_{i}"], label=f"Actual {labels[i]}", linewidth=1.5)

        # Task-space trajectory reference
        if has_traj_task:
            ax.plot(
                t,
                df[f"traj_task_pos_{i}"],
                label="Trajectory Ref",
                linestyle="--",
                linewidth=1.5,
                alpha=0.8,
                color="C1",
            )

        # Task goal from GUI (goal_type=="task" 구간만)
        if task_goal_mask is not None:
            tg = df[f"task_goal_{i}"].copy()
            tg[~task_goal_mask] = np.nan
            unit = "m" if i < 3 else "rad"
            ax.plot(
                t,
                tg,
                label="Task Goal (GUI)",
                linestyle="-.",
                linewidth=1.5,
                alpha=0.7,
                color="C4",
            )

        ax.set_ylabel(f"{labels[i]} (m)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "robot_task_position.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_robot_tracking_error(df, save_dir=None):
    """Position & velocity tracking error (trajectory - actual)."""
    # trajectory 컬럼 결정 (신규 named 또는 레거시 numeric)
    traj_pos_cols, traj_names = _detect_joint_columns(df, "traj_pos_")
    if not traj_pos_cols:
        traj_pos_cols, traj_names = _detect_joint_columns(df, "target_pos_")
    actual_pos_cols, _ = _detect_joint_columns(df, "actual_pos_")
    if not traj_pos_cols or not actual_pos_cols:
        print("  Skipping tracking error plot (trajectory columns not found)")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle("Robot Tracking Errors", fontsize=16, fontweight="bold")
    t = df["timestamp"]

    for i in range(len(traj_pos_cols)):
        pos_err = df[traj_pos_cols[i]] - df[actual_pos_cols[i]]
        ax1.plot(t, pos_err, label=f"J{i} ({traj_names[i]})", alpha=0.7)
    ax1.set_ylabel("Position Error (rad)")
    ax1.set_title("Position Tracking Error (trajectory - actual)")
    ax1.legend(fontsize=8, ncol=3)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color="k", linewidth=0.5)

    # Velocity tracking error
    traj_vel_cols, _ = _detect_joint_columns(df, "traj_vel_")
    if not traj_vel_cols:
        traj_vel_cols, _ = _detect_joint_columns(df, "target_vel_")
    actual_vel_cols, _ = _detect_joint_columns(df, "actual_vel_")
    if traj_vel_cols and actual_vel_cols:
        for i in range(len(traj_vel_cols)):
            vel_err = df[traj_vel_cols[i]] - df[actual_vel_cols[i]]
            ax2.plot(t, vel_err, label=f"J{i} ({traj_names[i]})", alpha=0.7)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity Error (rad/s)")
    ax2.set_title("Velocity Tracking Error (trajectory - actual)")
    ax2.legend(fontsize=8, ncol=3)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color="k", linewidth=0.5)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "robot_tracking_error.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_robot_task_tracking_error(df, save_dir=None):
    """Task-space tracking error (trajectory reference - actual TCP)."""
    if not _has_columns(df, "traj_task_pos_", 3) or not _has_columns(
        df, "task_pos_", 3
    ):
        print("  Skipping task tracking error plot (traj_task_pos_* columns not found)")
        return

    pos_labels = ["X", "Y", "Z"]
    rot_labels = ["Roll", "Pitch", "Yaw"]
    has_rot = _has_columns(df, "traj_task_pos_", 6) and _has_columns(df, "task_pos_", 6)
    nrows = 2 if has_rot else 1
    fig, axes = plt.subplots(nrows, 1, figsize=(14, 5 * nrows))
    fig.suptitle(
        "Task-Space Tracking Error (Trajectory Ref - Actual)",
        fontsize=16,
        fontweight="bold",
    )
    if nrows == 1:
        axes = [axes]

    t = df["timestamp"]

    # Position error (m)
    ax = axes[0]
    for i in range(3):
        err = df[f"traj_task_pos_{i}"] - df[f"task_pos_{i}"]
        ax.plot(t, err, label=pos_labels[i], alpha=0.8, linewidth=1.2)
    ax.set_ylabel("Position Error (m)")
    ax.set_title("Translation Tracking Error")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color="k", linewidth=0.5)

    # Orientation error (rad)
    if has_rot:
        ax = axes[1]
        for i in range(3):
            err = df[f"traj_task_pos_{i + 3}"] - df[f"task_pos_{i + 3}"]
            ax.plot(t, err, label=rot_labels[i], alpha=0.8, linewidth=1.2)
        ax.set_ylabel("Orientation Error (rad)")
        ax.set_title("Rotation Tracking Error")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color="k", linewidth=0.5)

    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "robot_task_tracking_error.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def print_robot_statistics(df):
    """Print robot trajectory statistics."""
    duration = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]
    rate = len(df) / duration if duration > 0 else 0.0
    print("\n=== Robot Trajectory Statistics ===")
    print(f"Duration: {duration:.2f} s | Samples: {len(df)} | Rate: {rate:.1f} Hz")

    # trajectory 컬럼 결정 (named 또는 numeric)
    traj_pos_cols, traj_names = _detect_joint_columns(df, "traj_pos_")
    if not traj_pos_cols:
        traj_pos_cols, traj_names = _detect_joint_columns(df, "target_pos_")
    actual_pos_cols, _ = _detect_joint_columns(df, "actual_pos_")

    if traj_pos_cols and actual_pos_cols:
        print("\nPosition Tracking Error (RMS):")
        for i in range(min(len(traj_pos_cols), len(actual_pos_cols))):
            traj_s = pd.to_numeric(df[traj_pos_cols[i]], errors="coerce")
            act_s = pd.to_numeric(df[actual_pos_cols[i]], errors="coerce")
            err = traj_s - act_s
            rms = np.sqrt(np.nanmean(err**2))
            print(
                f"  Joint {i} ({traj_names[i]}): "
                f"{rms:.6f} rad ({np.rad2deg(rms):.4f} deg)"
            )

    traj_vel_cols, _ = _detect_joint_columns(df, "traj_vel_")
    if not traj_vel_cols:
        traj_vel_cols, _ = _detect_joint_columns(df, "target_vel_")
    actual_vel_cols, _ = _detect_joint_columns(df, "actual_vel_")

    if traj_vel_cols and actual_vel_cols:
        print("\nVelocity Tracking Error (RMS):")
        for i in range(min(len(traj_vel_cols), len(actual_vel_cols))):
            traj_s = pd.to_numeric(df[traj_vel_cols[i]], errors="coerce")
            act_s = pd.to_numeric(df[actual_vel_cols[i]], errors="coerce")
            err = traj_s - act_s
            rms = np.sqrt(np.nanmean(err**2))
            print(f"  Joint {i} ({traj_names[i]}): {rms:.6f} rad/s")

    if "command_type" in df.columns:
        cmd_type = df["command_type"].mode().iloc[0] if len(df) > 0 else 0
        print(f"\nCommand type: {'position' if cmd_type == 0 else 'torque'}")


