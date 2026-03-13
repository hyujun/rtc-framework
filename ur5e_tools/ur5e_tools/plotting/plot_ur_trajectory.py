#!/usr/bin/env python3
"""
plot_ur_trajectory.py - v2

Visualize UR5e trajectory data from split CSV logs.

파일 이름으로 robot/hand 데이터를 자동 구분:
  - robot_log_*.csv → Robot 모드 (Figure 1: Position, Figure 2: Velocity)
  - hand_log_*.csv  → Hand 모드 (Figure 1: Position, Figure 2: Velocity,
                                  Figure 3: Barometer/ToF 센서)
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse
import sys


# ── Robot 모드 ──────────────────────────────────────────────────────────────

JOINT_NAMES = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']


def plot_robot_positions(df, save_dir=None):
    """Figure 1: Robot joint positions (3x2 subplot)."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Robot Joint Positions', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    for i in range(6):
        ax = axes[i]
        t = df['timestamp']
        ax.plot(t, df[f'actual_pos_{i}'], label='Actual', linewidth=1.5)
        ax.plot(t, df[f'target_pos_{i}'], label='Target (traj)',
                linestyle='--', linewidth=1.5)
        ax.plot(t, df[f'goal_pos_{i}'], label='Goal',
                linestyle=':', linewidth=1.5, alpha=0.8)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.set_title(f'Joint {i}: {JOINT_NAMES[i]}')
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
    """Figure 2: Robot joint velocities (3x2 subplot)."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Robot Joint Velocities', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    for i in range(6):
        ax = axes[i]
        t = df['timestamp']
        ax.plot(t, df[f'actual_vel_{i}'], label='Actual', linewidth=1.5)
        ax.plot(t, df[f'target_vel_{i}'], label='Target (traj)',
                linestyle='--', linewidth=1.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.set_title(f'Joint {i}: {JOINT_NAMES[i]}')
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


def plot_robot_tracking_error(df, save_dir=None):
    """Optional: position & velocity tracking error."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Robot Tracking Errors', fontsize=16, fontweight='bold')
    t = df['timestamp']

    for i in range(6):
        pos_err = df[f'target_pos_{i}'] - df[f'actual_pos_{i}']
        ax1.plot(t, pos_err, label=f'J{i} ({JOINT_NAMES[i]})', alpha=0.7)
    ax1.set_ylabel('Position Error (rad)')
    ax1.set_title('Position Tracking Error (target - actual)')
    ax1.legend(fontsize=8, ncol=3)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linewidth=0.5)

    for i in range(6):
        vel_err = df[f'target_vel_{i}'] - df[f'actual_vel_{i}']
        ax2.plot(t, vel_err, label=f'J{i} ({JOINT_NAMES[i]})', alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity Error (rad/s)')
    ax2.set_title('Velocity Tracking Error (target - actual)')
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

    print('\nPosition Tracking Error (RMS):')
    for i in range(6):
        err = df[f'target_pos_{i}'] - df[f'actual_pos_{i}']
        rms = np.sqrt(np.mean(err ** 2))
        print(f'  Joint {i} ({JOINT_NAMES[i]}): '
              f'{rms:.6f} rad ({np.rad2deg(rms):.4f} deg)')

    print('\nVelocity Tracking Error (RMS):')
    for i in range(6):
        err = df[f'target_vel_{i}'] - df[f'actual_vel_{i}']
        rms = np.sqrt(np.mean(err ** 2))
        print(f'  Joint {i} ({JOINT_NAMES[i]}): {rms:.6f} rad/s')


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
    if stem.startswith('robot_log_'):
        return 'robot'
    elif stem.startswith('hand_log_'):
        return 'hand'
    else:
        return 'unknown'


def main():
    parser = argparse.ArgumentParser(
        description='Plot UR5e trajectory from split CSV logs')
    parser.add_argument('csv_file', type=str,
                        help='Path to robot_log_*.csv or hand_log_*.csv')
    parser.add_argument('--save-dir', type=str, default=None,
                        help='Directory to save plots (PNG). '
                             '미지정 시 UR5E_SESSION_DIR/plots/ 사용')
    parser.add_argument('--stats', action='store_true',
                        help='Print statistics only (no plots)')
    parser.add_argument('--error', action='store_true',
                        help='Also plot tracking error figures (robot only)')

    args = parser.parse_args()

    # --save-dir 미지정 시 세션 디렉토리의 plots/ 서브디렉토리를 기본값으로 사용
    if args.save_dir is None:
        session = os.environ.get('UR5E_SESSION_DIR', '')
        if session:
            args.save_dir = os.path.join(session, 'plots')

    log_type = detect_log_type(args.csv_file)
    if log_type == 'unknown':
        print(f'Error: Cannot detect log type from filename: {args.csv_file}')
        print('Expected: robot_log_*.csv or hand_log_*.csv')
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
    elif log_type == 'hand':
        print_hand_statistics(df)
        if not args.stats:
            plot_hand_positions(df, args.save_dir)
            plot_hand_velocities(df, args.save_dir)
            plot_hand_sensors(df, args.save_dir)


if __name__ == '__main__':
    main()
