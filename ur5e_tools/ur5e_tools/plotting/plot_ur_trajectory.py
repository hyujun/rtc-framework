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
    # trajectory 컬럼 결정 (신규 traj_pos_ 또는 레거시 target_pos_)
    if _has_columns(df, 'traj_pos_', 6):
        pos_prefix, vel_prefix = 'traj_pos_', 'traj_vel_'
    elif _has_columns(df, 'target_pos_', 6):
        pos_prefix, vel_prefix = 'target_pos_', 'target_vel_'
    else:
        print('  Skipping tracking error plot (trajectory columns not found)')
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Robot Tracking Errors', fontsize=16, fontweight='bold')
    t = df['timestamp']

    for i in range(6):
        pos_err = df[f'{pos_prefix}{i}'] - df[f'actual_pos_{i}']
        ax1.plot(t, pos_err, label=f'J{i} ({JOINT_NAMES[i]})', alpha=0.7)
    ax1.set_ylabel('Position Error (rad)')
    ax1.set_title('Position Tracking Error (trajectory - actual)')
    ax1.legend(fontsize=8, ncol=3)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linewidth=0.5)

    has_vel = _has_columns(df, vel_prefix, 6)
    if has_vel:
        for i in range(6):
            vel_err = df[f'{vel_prefix}{i}'] - df[f'actual_vel_{i}']
            ax2.plot(t, vel_err, label=f'J{i} ({JOINT_NAMES[i]})', alpha=0.7)
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

    # trajectory 컬럼 결정
    if _has_columns(df, 'traj_pos_', 6):
        pos_prefix, vel_prefix = 'traj_pos_', 'traj_vel_'
    elif _has_columns(df, 'target_pos_', 6):
        pos_prefix, vel_prefix = 'target_pos_', 'target_vel_'
    else:
        pos_prefix, vel_prefix = None, None

    if pos_prefix:
        print('\nPosition Tracking Error (RMS):')
        for i in range(6):
            err = df[f'{pos_prefix}{i}'] - df[f'actual_pos_{i}']
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Joint {i} ({JOINT_NAMES[i]}): '
                  f'{rms:.6f} rad ({np.rad2deg(rms):.4f} deg)')

    if vel_prefix and _has_columns(df, vel_prefix, 6):
        print('\nVelocity Tracking Error (RMS):')
        for i in range(6):
            err = df[f'{vel_prefix}{i}'] - df[f'actual_vel_{i}']
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Joint {i} ({JOINT_NAMES[i]}): {rms:.6f} rad/s')

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
    fig, axes = plt.subplots(2, 5, figsize=(20, 8))
    fig.suptitle('Hand Motor Positions', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    t = df['timestamp']
    for i in range(NUM_HAND_MOTORS):
        ax = axes[i]
        ax.plot(t, df[f'hand_actual_pos_{i}'], label='Actual', linewidth=1.5)
        ax.plot(t, df[f'hand_cmd_{i}'], label='Command',
                linestyle='--', linewidth=1.5)
        ax.plot(t, df[f'hand_goal_pos_{i}'], label='Goal',
                linestyle=':', linewidth=1.5, alpha=0.8)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position')
        ax.set_title(f'Motor {i}')
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
    fig, axes = plt.subplots(2, 5, figsize=(20, 8))
    fig.suptitle('Hand Motor Velocities', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    t = df['timestamp']
    for i in range(NUM_HAND_MOTORS):
        ax = axes[i]
        ax.plot(t, df[f'hand_actual_vel_{i}'], label='Actual', linewidth=1.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.set_title(f'Motor {i}')
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

    print('\nPosition Tracking Error (RMS):')
    for i in range(NUM_HAND_MOTORS):
        goal_col = f'hand_goal_pos_{i}'
        actual_col = f'hand_actual_pos_{i}'
        if goal_col in df.columns and actual_col in df.columns:
            err = df[goal_col] - df[actual_col]
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Motor {i}: {rms:.6f}')

    # Sensor summary
    baro_cols = [c for c in df.columns if c.startswith('baro_')]
    tof_cols = [c for c in df.columns if c.startswith('tof_')]
    if baro_cols:
        baro_vals = df[baro_cols].values.flatten()
        print(f'\nBarometer range: [{baro_vals.min():.1f}, {baro_vals.max():.1f}]')
    if tof_cols:
        tof_vals = df[tof_cols].values.flatten()
        print(f'ToF range: [{tof_vals.min():.1f}, {tof_vals.max():.1f}]')


# ── Auto-detect & Main ────────────────────────────────────────────────────

def detect_log_type(filepath):
    """Detect log type from filename prefix."""
    stem = Path(filepath).stem
    if stem.startswith('robot_log'):
        return 'robot'
    elif stem.startswith('hand_log'):
        return 'hand'
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
        print('Expected: robot_log*.csv or hand_log*.csv')
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


if __name__ == '__main__':
    main()
