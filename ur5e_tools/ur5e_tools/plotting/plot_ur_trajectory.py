#!/usr/bin/env python3
"""
plot_ur_trajectory.py - v3

Visualize UR5e trajectory data from split CSV logs (4-category convention).

CSV 컬럼 카테고리:
  1. Goal State — 외부 입력 목표
  2. Current State — 센서 피드백
  3. Control Command — 액추에이터 출력
  4. Trajectory State — 궤적 보간 내부 상태

파일 이름으로 robot/hand 데이터를 자동 구분:
  - robot_log*.csv → Robot 모드
  - hand_log*.csv  → Hand 모드
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse
import sys


# ── Robot 모드 ──────────────────────────────────────────────────────────────

# 기본 표시 이름 (인덱스 기반 CSV 하위 호환용)
JOINT_NAMES_DEFAULT = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']


def _detect_joint_columns(df, prefix, count=6):
    """CSV 헤더에서 prefix로 시작하는 컬럼을 감지하여 (column_names, display_names) 반환.

    v5.14.0 named headers: 'goal_pos_shoulder_pan_joint' 형태
    Legacy numeric headers: 'goal_pos_0' 형태
    둘 다 자동 감지.
    """
    # 1. 이름 기반 컬럼 탐색 (prefix로 시작하는 모든 컬럼)
    named_cols = [c for c in df.columns if c.startswith(prefix)]
    if len(named_cols) >= count:
        # prefix 제거하여 표시 이름 추출
        display = [c[len(prefix):] for c in named_cols[:count]]
        return named_cols[:count], display

    # 2. 숫자 인덱스 기반 fallback
    numeric_cols = [f'{prefix}{i}' for i in range(count)]
    if all(c in df.columns for c in numeric_cols):
        return numeric_cols, JOINT_NAMES_DEFAULT[:count]

    return [], []


def _has_columns(df, prefix, count):
    """Check if df has columns like '{prefix}0' .. '{prefix}{count-1}'.
    Also supports named columns (v5.14.0)."""
    # Named columns
    named = [c for c in df.columns if c.startswith(prefix)]
    if len(named) >= count:
        return True
    # Numeric fallback
    return all(f'{prefix}{i}' in df.columns for i in range(count))


# 하위 호환 alias
JOINT_NAMES = JOINT_NAMES_DEFAULT


def plot_robot_positions(df, save_dir=None):
    """Figure 1: Robot joint positions — goal vs trajectory vs actual (3x2)."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Robot Joint Positions', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    actual_cols, display_names = _detect_joint_columns(df, 'actual_pos_', 6)
    goal_cols, _ = _detect_joint_columns(df, 'goal_pos_', 6)
    traj_cols, _ = _detect_joint_columns(df, 'traj_pos_', 6)
    # 하위 호환: 이전 CSV의 target_pos_ 컬럼 지원
    legacy_cols, _ = _detect_joint_columns(df, 'target_pos_', 6) if not traj_cols else ([], [])

    for i in range(min(6, len(actual_cols))):
        ax = axes[i]
        t = df['timestamp']
        ax.plot(t, df[actual_cols[i]], label='Actual', linewidth=1.5)

        if traj_cols:
            ax.plot(t, df[traj_cols[i]], label='Trajectory',
                    linestyle='--', linewidth=1.5)
        elif legacy_cols:
            ax.plot(t, df[legacy_cols[i]], label='Target (traj)',
                    linestyle='--', linewidth=1.5)

        if goal_cols:
            ax.plot(t, df[goal_cols[i]], label='Goal',
                    linestyle=':', linewidth=1.5, alpha=0.8)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.set_title(f'Joint {i}: {display_names[i]}')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'robot_positions.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_robot_velocities(df, save_dir=None):
    """Figure 2: Robot joint velocities — trajectory vs actual (3x2)."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Robot Joint Velocities', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    actual_cols, display_names = _detect_joint_columns(df, 'actual_vel_', 6)
    traj_cols, _ = _detect_joint_columns(df, 'traj_vel_', 6)
    legacy_cols, _ = _detect_joint_columns(df, 'target_vel_', 6) if not traj_cols else ([], [])

    for i in range(min(6, len(actual_cols))):
        ax = axes[i]
        t = df['timestamp']
        ax.plot(t, df[actual_cols[i]], label='Actual', linewidth=1.5)

        if traj_cols:
            ax.plot(t, df[traj_cols[i]], label='Trajectory',
                    linestyle='--', linewidth=1.5)
        elif legacy_cols:
            ax.plot(t, df[legacy_cols[i]], label='Target (traj)',
                    linestyle='--', linewidth=1.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.set_title(f'Joint {i}: {display_names[i]}')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'robot_velocities.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_robot_commands(df, save_dir=None):
    """Figure 3: Robot control commands — command output per joint (3x2)."""
    if not _has_columns(df, 'command_', 6):
        print('  Skipping command plot (command_* columns not found)')
        return

    fig, axes = plt.subplots(3, 2, figsize=(15, 12))

    # command_type: 0=position, 1=torque
    has_type = 'command_type' in df.columns
    if has_type:
        cmd_type = df['command_type'].mode().iloc[0] if len(df) > 0 else 0
        type_label = 'Position' if cmd_type == 0 else 'Torque'
        unit = 'rad' if cmd_type == 0 else 'Nm'
    else:
        type_label = 'Command'
        unit = ''

    fig.suptitle(f'Robot Control Commands ({type_label})',
                 fontsize=16, fontweight='bold')
    axes = axes.flatten()

    cmd_cols, display_names = _detect_joint_columns(df, 'command_', 6)
    actual_cols, _ = _detect_joint_columns(df, 'actual_pos_', 6)

    for i in range(min(6, len(cmd_cols))):
        ax = axes[i]
        t = df['timestamp']
        ax.plot(t, df[cmd_cols[i]], label=type_label,
                linewidth=1.5, color='C2')
        if actual_cols:
            ax.plot(t, df[actual_cols[i]], label='Actual pos',
                    linewidth=1.0, alpha=0.5, linestyle='--')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel(f'{type_label} ({unit})' if unit else type_label)
        ax.set_title(f'Joint {i}: {display_names[i]}')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'robot_commands.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_robot_torques(df, save_dir=None):
    """Figure 4: Robot actual torques (3x2)."""
    if not _has_columns(df, 'actual_torque_', 6):
        print('  Skipping torque plot (actual_torque_* columns not found)')
        return

    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Robot Actual Torques', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    for i in range(6):
        ax = axes[i]
        t = df['timestamp']
        ax.plot(t, df[f'actual_torque_{i}'], label='Torque',
                linewidth=1.5, color='C3')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        ax.set_title(f'Joint {i}: {JOINT_NAMES[i]}')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'robot_torques.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_robot_task_position(df, save_dir=None):
    """Figure 5: TCP task-space position (3x1: x, y, z)."""
    if not _has_columns(df, 'task_pos_', 3):
        print('  Skipping task position plot (task_pos_* columns not found)')
        return

    labels = ['X', 'Y', 'Z']
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('TCP Task-Space Position', fontsize=16, fontweight='bold')

    t = df['timestamp']
    for i in range(3):
        ax = axes[i]
        ax.plot(t, df[f'task_pos_{i}'], label=f'{labels[i]}',
                linewidth=1.5)
        ax.set_ylabel(f'{labels[i]} (m)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'robot_task_position.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_robot_tracking_error(df, save_dir=None):
    """Position & velocity tracking error (trajectory - actual)."""
    # trajectory 컬럼 결정 (신규 named 또는 레거시 numeric)
    traj_pos_cols, traj_names = _detect_joint_columns(df, 'traj_pos_', 6)
    if not traj_pos_cols:
        traj_pos_cols, traj_names = _detect_joint_columns(df, 'target_pos_', 6)
    actual_pos_cols, _ = _detect_joint_columns(df, 'actual_pos_', 6)
    if not traj_pos_cols or not actual_pos_cols:
        print('  Skipping tracking error plot (trajectory columns not found)')
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Robot Tracking Errors', fontsize=16, fontweight='bold')
    t = df['timestamp']

    for i in range(min(6, len(traj_pos_cols))):
        pos_err = df[traj_pos_cols[i]] - df[actual_pos_cols[i]]
        ax1.plot(t, pos_err, label=f'J{i} ({traj_names[i]})', alpha=0.7)
    ax1.set_ylabel('Position Error (rad)')
    ax1.set_title('Position Tracking Error (trajectory - actual)')
    ax1.legend(fontsize=8, ncol=3)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linewidth=0.5)

    # Velocity tracking error
    traj_vel_cols, _ = _detect_joint_columns(df, 'traj_vel_', 6)
    if not traj_vel_cols:
        traj_vel_cols, _ = _detect_joint_columns(df, 'target_vel_', 6)
    actual_vel_cols, _ = _detect_joint_columns(df, 'actual_vel_', 6)
    if traj_vel_cols and actual_vel_cols:
        for i in range(min(6, len(traj_vel_cols))):
            vel_err = df[traj_vel_cols[i]] - df[actual_vel_cols[i]]
            ax2.plot(t, vel_err, label=f'J{i} ({traj_names[i]})', alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity Error (rad/s)')
    ax2.set_title('Velocity Tracking Error (trajectory - actual)')
    ax2.legend(fontsize=8, ncol=3)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linewidth=0.5)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'robot_tracking_error.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def print_robot_statistics(df):
    """Print robot trajectory statistics."""
    duration = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
    print('\n=== Robot Trajectory Statistics ===')
    print(f'Duration: {duration:.2f} s | Samples: {len(df)}'
          f' | Rate: {len(df) / duration:.1f} Hz')

    # trajectory 컬럼 결정 (named 또는 numeric)
    traj_pos_cols, traj_names = _detect_joint_columns(df, 'traj_pos_', 6)
    if not traj_pos_cols:
        traj_pos_cols, traj_names = _detect_joint_columns(df, 'target_pos_', 6)
    actual_pos_cols, _ = _detect_joint_columns(df, 'actual_pos_', 6)

    if traj_pos_cols and actual_pos_cols:
        print('\nPosition Tracking Error (RMS):')
        for i in range(min(6, len(traj_pos_cols))):
            err = df[traj_pos_cols[i]] - df[actual_pos_cols[i]]
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Joint {i} ({traj_names[i]}): '
                  f'{rms:.6f} rad ({np.rad2deg(rms):.4f} deg)')

    traj_vel_cols, _ = _detect_joint_columns(df, 'traj_vel_', 6)
    if not traj_vel_cols:
        traj_vel_cols, _ = _detect_joint_columns(df, 'target_vel_', 6)
    actual_vel_cols, _ = _detect_joint_columns(df, 'actual_vel_', 6)

    if traj_vel_cols and actual_vel_cols:
        print('\nVelocity Tracking Error (RMS):')
        for i in range(min(6, len(traj_vel_cols))):
            err = df[traj_vel_cols[i]] - df[actual_vel_cols[i]]
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Joint {i} ({traj_names[i]}): {rms:.6f} rad/s')

    if 'command_type' in df.columns:
        cmd_type = df['command_type'].mode().iloc[0] if len(df) > 0 else 0
        print(f'\nCommand type: {"position" if cmd_type == 0 else "torque"}')


# ── Hand 모드 ──────────────────────────────────────────────────────────────

NUM_HAND_MOTORS = 10
NUM_FINGERTIPS = 4
BARO_COUNT = 8
TOF_COUNT = 3


def plot_hand_positions(df, save_dir=None):
    """Figure 1: Hand motor positions (2x5 subplot)."""
    actual_cols, motor_names = _detect_joint_columns(df, 'hand_actual_pos_', NUM_HAND_MOTORS)
    cmd_cols, _ = _detect_joint_columns(df, 'hand_cmd_', NUM_HAND_MOTORS)
    goal_cols, _ = _detect_joint_columns(df, 'hand_goal_pos_', NUM_HAND_MOTORS)
    if not actual_cols:
        print('  Skipping hand positions plot (columns not found)')
        return

    fig, axes = plt.subplots(2, 5, figsize=(20, 8))
    fig.suptitle('Hand Motor Positions', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    t = df['timestamp']
    for i in range(min(NUM_HAND_MOTORS, len(actual_cols))):
        ax = axes[i]
        ax.plot(t, df[actual_cols[i]], label='Actual', linewidth=1.5)
        if i < len(cmd_cols):
            ax.plot(t, df[cmd_cols[i]], label='Command',
                    linestyle='--', linewidth=1.5)
        if i < len(goal_cols):
            ax.plot(t, df[goal_cols[i]], label='Goal',
                    linestyle=':', linewidth=1.5, alpha=0.8)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position')
        ax.set_title(f'Motor {i} ({motor_names[i]})')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'hand_positions.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_hand_velocities(df, save_dir=None):
    """Figure 2: Hand motor velocities (2x5 subplot)."""
    vel_cols, motor_names = _detect_joint_columns(df, 'hand_actual_vel_', NUM_HAND_MOTORS)
    if not vel_cols:
        print('  Skipping hand velocities plot (columns not found)')
        return

    fig, axes = plt.subplots(2, 5, figsize=(20, 8))
    fig.suptitle('Hand Motor Velocities', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    t = df['timestamp']
    for i in range(min(NUM_HAND_MOTORS, len(vel_cols))):
        ax = axes[i]
        ax.plot(t, df[vel_cols[i]], label='Actual', linewidth=1.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.set_title(f'Motor {i} ({motor_names[i]})')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'hand_velocities.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_hand_sensors(df, save_dir=None):
    """Figure 3: Hand sensor data (2 rows x 4 cols: barometer + ToF)."""
    fig, axes = plt.subplots(2, NUM_FINGERTIPS, figsize=(20, 8))
    fig.suptitle('Hand Sensor Data', fontsize=16, fontweight='bold')

    t = df['timestamp']

    # Row 1: Barometer (8ch per fingertip)
    for f_idx in range(NUM_FINGERTIPS):
        ax = axes[0, f_idx]
        for b in range(BARO_COUNT):
            col = f'baro_f{f_idx}_{b}'
            if col in df.columns:
                ax.plot(t, df[col], label=f'B{b}', alpha=0.7, linewidth=0.8)
        ax.set_title(f'Fingertip {f_idx} — Barometer')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Pressure')
        ax.legend(fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)

    # Row 2: ToF (3ch per fingertip)
    for f_idx in range(NUM_FINGERTIPS):
        ax = axes[1, f_idx]
        for t_idx in range(TOF_COUNT):
            col = f'tof_f{f_idx}_{t_idx}'
            if col in df.columns:
                ax.plot(t, df[col], label=f'ToF{t_idx}',
                        linewidth=1.2, marker='.', markersize=1)
        ax.set_title(f'Fingertip {f_idx} — ToF')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'hand_sensors.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def print_hand_statistics(df):
    """Print hand trajectory statistics."""
    duration = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
    print('\n=== Hand Trajectory Statistics ===')
    print(f'Duration: {duration:.2f} s | Samples: {len(df)}'
          f' | Rate: {len(df) / duration:.1f} Hz')

    valid_ratio = df['hand_valid'].sum() / len(df) * 100
    print(f'Hand valid: {valid_ratio:.1f}%')

    goal_cols, _ = _detect_joint_columns(df, 'hand_goal_pos_', NUM_HAND_MOTORS)
    actual_cols, motor_names = _detect_joint_columns(df, 'hand_actual_pos_', NUM_HAND_MOTORS)
    if goal_cols and actual_cols:
        print('\nPosition Tracking Error (RMS):')
        for i in range(min(NUM_HAND_MOTORS, len(goal_cols), len(actual_cols))):
            err = df[goal_cols[i]] - df[actual_cols[i]]
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Motor {i} ({motor_names[i]}): {rms:.6f}')

    # Sensor summary
    baro_cols = [c for c in df.columns if c.startswith('baro_')]
    tof_cols = [c for c in df.columns if c.startswith('tof_')]
    if baro_cols:
        baro_vals = df[baro_cols].values.flatten()
        print(f'\nBarometer range: [{baro_vals.min():.1f}, {baro_vals.max():.1f}]')
    if tof_cols:
        tof_vals = df[tof_cols].values.flatten()
        print(f'ToF range: [{tof_vals.min():.1f}, {tof_vals.max():.1f}]')


# ── Timing 모드 ─────────────────────────────────────────────────────────────


def plot_timing_breakdown(df, save_dir=None):
    """Figure 1: Per-phase timing breakdown (stacked area)."""
    fig, ax = plt.subplots(figsize=(14, 6))
    fig.suptitle('Control Loop Timing Breakdown', fontsize=16, fontweight='bold')

    t = df['timestamp']
    phases = ['t_state_acquire_us', 't_compute_us', 't_publish_us']
    labels = ['State Acquire', 'Compute', 'Publish']
    colors = ['#2196F3', '#FF9800', '#4CAF50']

    available = [(p, l, c) for p, l, c in zip(phases, labels, colors)
                 if p in df.columns]
    if available:
        ax.stackplot(t, *[df[p] for p, _, _ in available],
                     labels=[l for _, l, _ in available],
                     colors=[c for _, _, c in available], alpha=0.7)

    # Budget line
    if len(df) > 1:
        dt = t.iloc[1] - t.iloc[0]
        budget = 1e6 * dt if dt > 0 else 2000.0
    else:
        budget = 2000.0
    ax.axhline(y=budget, color='red', linestyle='--', linewidth=1.5,
               label=f'Budget ({budget:.0f} µs)')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Duration (µs)')
    ax.legend(fontsize=9, loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'timing_breakdown.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_timing_total_and_jitter(df, save_dir=None):
    """Figure 2: Total loop time and jitter (2 subplots)."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Loop Timing & Jitter', fontsize=16, fontweight='bold')

    t = df['timestamp']

    # Budget estimation
    if len(df) > 1:
        dt = t.iloc[1] - t.iloc[0]
        budget = 1e6 * dt if dt > 0 else 2000.0
    else:
        budget = 2000.0

    # Total loop time
    if 't_total_us' in df.columns:
        ax1.plot(t, df['t_total_us'], linewidth=0.5, alpha=0.7, label='t_total')
        ax1.axhline(y=budget, color='red', linestyle='--', linewidth=1.5,
                     label=f'Budget ({budget:.0f} µs)')
        overruns = (df['t_total_us'] > budget).sum()
        ax1.set_title(f'Total Loop Time (overruns: {overruns})')
    ax1.set_ylabel('Duration (µs)')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Jitter
    if 'jitter_us' in df.columns:
        ax2.plot(t, df['jitter_us'], linewidth=0.5, alpha=0.7,
                 color='C1', label='Jitter')
        ax2.set_title('Period Jitter')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Jitter (µs)')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'timing_total_jitter.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_timing_histograms(df, save_dir=None):
    """Figure 3: Histograms of total loop time and jitter."""
    cols = []
    if 't_total_us' in df.columns:
        cols.append(('t_total_us', 'Total Loop Time'))
    if 'jitter_us' in df.columns:
        cols.append(('jitter_us', 'Jitter'))
    if not cols:
        print('  Skipping timing histograms (columns not found)')
        return

    fig, axes = plt.subplots(1, len(cols), figsize=(7 * len(cols), 5))
    fig.suptitle('Timing Distributions', fontsize=16, fontweight='bold')
    if len(cols) == 1:
        axes = [axes]

    for ax, (col, label) in zip(axes, cols):
        data = df[col]
        ax.hist(data, bins=100, edgecolor='black', linewidth=0.3, alpha=0.7)
        p50 = data.quantile(0.50)
        p95 = data.quantile(0.95)
        p99 = data.quantile(0.99)
        ax.axvline(p50, color='green', linestyle='--',
                   label=f'P50: {p50:.1f} µs')
        ax.axvline(p95, color='orange', linestyle='--',
                   label=f'P95: {p95:.1f} µs')
        ax.axvline(p99, color='red', linestyle='--',
                   label=f'P99: {p99:.1f} µs')
        ax.set_xlabel(f'{label} (µs)')
        ax.set_ylabel('Count')
        ax.set_title(label)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'timing_histograms.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def print_timing_statistics(df):
    """Print timing statistics summary."""
    duration = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
    rate = len(df) / duration if duration > 0 else 0

    print('\n=== Timing Statistics ===')
    print(f'Duration: {duration:.2f} s | Samples: {len(df)}'
          f' | Rate: {rate:.1f} Hz')

    # Budget estimation
    budget = 1e6 / rate if rate > 0 else 2000.0

    for col, label in [('t_state_acquire_us', 'State Acquire'),
                       ('t_compute_us', 'Compute'),
                       ('t_publish_us', 'Publish'),
                       ('t_total_us', 'Total Loop'),
                       ('jitter_us', 'Jitter')]:
        if col not in df.columns:
            continue
        data = df[col]
        print(f'\n{label}:')
        print(f'  Mean: {data.mean():.2f} µs | Std: {data.std():.2f} µs')
        print(f'  Min: {data.min():.2f} µs | Max: {data.max():.2f} µs')
        print(f'  P50: {data.quantile(0.50):.2f} µs'
              f' | P95: {data.quantile(0.95):.2f} µs'
              f' | P99: {data.quantile(0.99):.2f} µs')

    if 't_total_us' in df.columns:
        overruns = (df['t_total_us'] > budget).sum()
        pct = overruns / len(df) * 100
        print(f'\nOverruns (>{budget:.0f} µs): {overruns} ({pct:.3f}%)')


# ── Auto-detect & Main ────────────────────────────────────────────────────

def detect_log_type(filepath):
    """Detect log type from filename prefix."""
    stem = Path(filepath).stem
    if stem.startswith('robot_log'):
        return 'robot'
    elif stem.startswith('hand_log'):
        return 'hand'
    elif stem.startswith('timing_log'):
        return 'timing'
    else:
        return 'unknown'


def main():
    parser = argparse.ArgumentParser(
        description='Plot UR5e trajectory from split CSV logs (4-category)')
    parser.add_argument('csv_file', type=str,
                        help='Path to robot_log*.csv or hand_log*.csv')
    parser.add_argument('--save-dir', type=str, default=None,
                        help='Directory to save plots (PNG). '
                             '미지정 시 UR5E_SESSION_DIR/plots/ 사용')
    parser.add_argument('--stats', action='store_true',
                        help='Print statistics only (no plots)')
    parser.add_argument('--error', action='store_true',
                        help='Also plot tracking error figures (robot only)')
    parser.add_argument('--command', action='store_true',
                        help='Also plot control command figures (robot only)')
    parser.add_argument('--torque', action='store_true',
                        help='Also plot actual torque figures (robot only)')
    parser.add_argument('--task-pos', action='store_true',
                        help='Also plot TCP task-space position (robot only)')
    parser.add_argument('--all', action='store_true',
                        help='Plot all available figures')

    args = parser.parse_args()

    # --all 플래그로 모든 플롯 활성화
    if args.all:
        args.error = True
        args.command = True
        args.torque = True
        args.task_pos = True

    # --save-dir 미지정 시 세션 디렉토리의 plots/ 서브디렉토리를 기본값으로 사용
    if args.save_dir is None:
        session = os.environ.get('UR5E_SESSION_DIR', '')
        if session:
            args.save_dir = os.path.join(session, 'plots')

    log_type = detect_log_type(args.csv_file)
    if log_type == 'unknown':
        print(f'Error: Cannot detect log type from filename: {args.csv_file}')
        print('Expected: robot_log*.csv, hand_log*.csv, or timing_log*.csv')
        sys.exit(1)

    print(f'Loading ({log_type}): {args.csv_file}')
    df = pd.read_csv(args.csv_file)

    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)

    if log_type == 'robot':
        print_robot_statistics(df)
        if not args.stats:
            plot_robot_positions(df, args.save_dir)
            plot_robot_velocities(df, args.save_dir)
            if args.error:
                plot_robot_tracking_error(df, args.save_dir)
            if args.command:
                plot_robot_commands(df, args.save_dir)
            if args.torque:
                plot_robot_torques(df, args.save_dir)
            if args.task_pos:
                plot_robot_task_position(df, args.save_dir)
    elif log_type == 'hand':
        print_hand_statistics(df)
        if not args.stats:
            plot_hand_positions(df, args.save_dir)
            plot_hand_velocities(df, args.save_dir)
            plot_hand_sensors(df, args.save_dir)
    elif log_type == 'timing':
        print_timing_statistics(df)
        if not args.stats:
            plot_timing_breakdown(df, args.save_dir)
            plot_timing_total_and_jitter(df, args.save_dir)
            plot_timing_histograms(df, args.save_dir)


if __name__ == '__main__':
    main()
