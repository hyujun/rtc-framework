#!/usr/bin/env python3
"""
plot_rtc_log.py - v4

Visualize robot trajectory data from split CSV logs (4-category convention).

CSV 컬럼 카테고리:
  1. Goal State — 외부 입력 목표
  2. Current State — 센서 피드백
  3. Control Command — 액추에이터 출력
  4. Trajectory State — 궤적 보간 내부 상태

파일 이름으로 robot/hand 데이터를 자동 구분:
  - robot_log*.csv → Robot 모드
  - hand_log*.csv  → Hand 모드

v4 변경사항:
  - 파일 이름 변경: plot_ur_trajectory.py → plot_ur_log.py
  - Hand raw sensor (pre-LPF) 플롯 추가 (--raw)
  - Hand F/T inference 출력 플롯 추가 (--ft)
  - Raw vs Filtered 센서 비교 플롯 추가 (--sensor-compare)
  - Agg backend 최적화 (--save-dir 지정 시)
  - 컬럼 감지 결과 캐싱
"""

import os
import numpy as np
import math
import pandas as pd
from pathlib import Path
import argparse
import sys

# matplotlib.pyplot는 main()에서 backend 설정 후 import (Agg 최적화)
plt = None


# ── 컬럼 감지 캐시 ──────────────────────────────────────────────────────────
# _detect_joint_columns()의 결과를 캐싱하여 동일 prefix 반복 호출 방지
_column_cache = {}


def _cache_key(df_id, prefix, count):
    """DataFrame id + prefix + count로 캐시 키 생성."""
    return (df_id, prefix, count)


def _invalidate_column_cache():
    """캐시 초기화 (새 DataFrame 로드 시)."""
    _column_cache.clear()


# ── 가변 DOF 유틸리티 ────────────────────────────────────────────────────────


def _detect_num_channels(df, prefix):
    """Auto-detect number of channels from CSV headers matching prefix."""
    return len([c for c in df.columns if c.startswith(prefix)])


def _auto_subplot_grid(n, max_cols=None):
    """Calculate optimal (nrows, ncols) for n subplots.

    Picks the layout closest to square with ncols >= nrows.
    Allows a small number of empty cells to avoid tall 1-column layouts.

    Examples:
      1 → (1,1)   2 → (1,2)   3 → (1,3)   4 → (2,2)
      5 → (2,3)   6 → (2,3)   7 → (3,3)   8 → (2,4)
      9 → (3,3)  10 → (2,5)  12 → (3,4)  16 → (4,4)
    """
    import math
    if n <= 0:
        return (1, 1)
    if n <= 3:
        return (1, n)

    if max_cols is not None:
        ncols = min(n, max_cols)
        nrows = math.ceil(n / ncols)
        return (nrows, ncols)

    # Start from ceil(sqrt(n)) columns and pick ncols >= nrows
    ncols = math.ceil(math.sqrt(n))
    nrows = math.ceil(n / ncols)
    # Ensure wider-than-tall
    if nrows > ncols:
        nrows, ncols = ncols, nrows
        nrows = math.ceil(n / ncols)
    return (nrows, ncols)


# ── Robot 모드 ──────────────────────────────────────────────────────────────

def _detect_joint_columns(df, prefix, count=None):
    """CSV 헤더에서 prefix로 시작하는 컬럼을 감지하여 (column_names, display_names) 반환.

    count=None이면 CSV에서 자동 감지.
    Named headers: 'goal_pos_shoulder_pan_joint' 형태
    Numeric headers: 'goal_pos_0' 형태
    둘 다 자동 감지. 결과는 캐싱됨.
    """
    if count is None:
        count = _detect_num_channels(df, prefix)
    if count == 0:
        return ([], [])
    key = _cache_key(id(df), prefix, count)
    if key in _column_cache:
        return _column_cache[key]

    # 1. 이름 기반 컬럼 탐색 (prefix로 시작하는 모든 컬럼)
    named_cols = [c for c in df.columns if c.startswith(prefix)]
    if len(named_cols) >= count:
        # prefix 제거하여 표시 이름 추출
        display = [c[len(prefix):] for c in named_cols[:count]]
        result = (named_cols[:count], display)
        _column_cache[key] = result
        return result

    # 2. 숫자 인덱스 기반 fallback
    numeric_cols = [f'{prefix}{i}' for i in range(count)]
    if all(c in df.columns for c in numeric_cols):
        result = (numeric_cols, [f"J{i}" for i in range(count)])
        _column_cache[key] = result
        return result

    result = ([], [])
    _column_cache[key] = result
    return result


def _has_columns(df, prefix, count):
    """Check if df has columns like '{prefix}0' .. '{prefix}{count-1}'.
    Also supports named columns (v5.14.0)."""
    # Named columns
    named = [c for c in df.columns if c.startswith(prefix)]
    if len(named) >= count:
        return True
    # Numeric fallback
    return all(f'{prefix}{i}' in df.columns for i in range(count))




def plot_robot_positions(df, save_dir=None):
    """Figure 1: Robot joint positions — goal vs trajectory vs actual."""
    actual_cols, display_names = _detect_joint_columns(df, 'actual_pos_')
    n_joints = len(actual_cols)
    if n_joints == 0:
        print('  Skipping position plot (actual_pos_* columns not found)')
        return

    nrows, ncols = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle('Robot Joint Positions', fontsize=16, fontweight='bold')
    axes = np.atleast_1d(axes).flatten()

    goal_cols, _ = _detect_joint_columns(df, 'goal_pos_')
    traj_cols, _ = _detect_joint_columns(df, 'traj_pos_')
    # 하위 호환: 이전 CSV의 target_pos_ 컬럼 지원
    legacy_cols, _ = _detect_joint_columns(df, 'target_pos_') if not traj_cols else ([], [])

    for i in range(n_joints):
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
    """Figure 2: Robot joint velocities — trajectory vs actual."""
    actual_cols, display_names = _detect_joint_columns(df, 'actual_vel_')
    n_joints = len(actual_cols)
    if n_joints == 0:
        print('  Skipping velocity plot (actual_vel_* columns not found)')
        return

    nrows, ncols = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle('Robot Joint Velocities', fontsize=16, fontweight='bold')
    axes = np.atleast_1d(axes).flatten()

    traj_cols, _ = _detect_joint_columns(df, 'traj_vel_')
    legacy_cols, _ = _detect_joint_columns(df, 'target_vel_') if not traj_cols else ([], [])

    for i in range(n_joints):
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
    """Figure 3: Robot control commands — command output per joint."""
    cmd_cols, display_names = _detect_joint_columns(df, 'command_')
    n_joints = len(cmd_cols)
    if n_joints == 0:
        print('  Skipping command plot (command_* columns not found)')
        return

    nrows, ncols_grid = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols_grid, figsize=(5 * ncols_grid, 4 * nrows))

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
    axes = np.atleast_1d(axes).flatten()

    actual_cols, _ = _detect_joint_columns(df, 'actual_pos_')

    for i in range(n_joints):
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
    """Figure 4: Robot actual torques."""
    torque_cols, display_names = _detect_joint_columns(df, 'actual_torque_')
    if not torque_cols:
        torque_cols, display_names = _detect_joint_columns(df, 'effort_')
    n_joints = len(torque_cols)
    if n_joints == 0:
        print('  Skipping torque plot (actual_torque_*/effort_* columns not found)')
        return

    nrows, ncols = _auto_subplot_grid(n_joints)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    fig.suptitle('Robot Actual Torques', fontsize=16, fontweight='bold')
    axes = np.atleast_1d(axes).flatten()

    for i in range(n_joints):
        ax = axes[i]
        t = df['timestamp']
        ax.plot(t, df[torque_cols[i]], label='Torque',
                linewidth=1.5, color='C3')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        ax.set_title(f'Joint {i}: {display_names[i]}')
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
    traj_pos_cols, traj_names = _detect_joint_columns(df, 'traj_pos_')
    if not traj_pos_cols:
        traj_pos_cols, traj_names = _detect_joint_columns(df, 'target_pos_')
    actual_pos_cols, _ = _detect_joint_columns(df, 'actual_pos_')
    if not traj_pos_cols or not actual_pos_cols:
        print('  Skipping tracking error plot (trajectory columns not found)')
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Robot Tracking Errors', fontsize=16, fontweight='bold')
    t = df['timestamp']

    for i in range(len(traj_pos_cols)):
        pos_err = df[traj_pos_cols[i]] - df[actual_pos_cols[i]]
        ax1.plot(t, pos_err, label=f'J{i} ({traj_names[i]})', alpha=0.7)
    ax1.set_ylabel('Position Error (rad)')
    ax1.set_title('Position Tracking Error (trajectory - actual)')
    ax1.legend(fontsize=8, ncol=3)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linewidth=0.5)

    # Velocity tracking error
    traj_vel_cols, _ = _detect_joint_columns(df, 'traj_vel_')
    if not traj_vel_cols:
        traj_vel_cols, _ = _detect_joint_columns(df, 'target_vel_')
    actual_vel_cols, _ = _detect_joint_columns(df, 'actual_vel_')
    if traj_vel_cols and actual_vel_cols:
        for i in range(len(traj_vel_cols)):
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
    traj_pos_cols, traj_names = _detect_joint_columns(df, 'traj_pos_')
    if not traj_pos_cols:
        traj_pos_cols, traj_names = _detect_joint_columns(df, 'target_pos_')
    actual_pos_cols, _ = _detect_joint_columns(df, 'actual_pos_')

    if traj_pos_cols and actual_pos_cols:
        print('\nPosition Tracking Error (RMS):')
        for i in range(len(traj_pos_cols)):
            err = df[traj_pos_cols[i]] - df[actual_pos_cols[i]]
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Joint {i} ({traj_names[i]}): '
                  f'{rms:.6f} rad ({np.rad2deg(rms):.4f} deg)')

    traj_vel_cols, _ = _detect_joint_columns(df, 'traj_vel_')
    if not traj_vel_cols:
        traj_vel_cols, _ = _detect_joint_columns(df, 'target_vel_')
    actual_vel_cols, _ = _detect_joint_columns(df, 'actual_vel_')

    if traj_vel_cols and actual_vel_cols:
        print('\nVelocity Tracking Error (RMS):')
        for i in range(len(traj_vel_cols)):
            err = df[traj_vel_cols[i]] - df[actual_vel_cols[i]]
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Joint {i} ({traj_names[i]}): {rms:.6f} rad/s')

    if 'command_type' in df.columns:
        cmd_type = df['command_type'].mode().iloc[0] if len(df) > 0 else 0
        print(f'\nCommand type: {"position" if cmd_type == 0 else "torque"}')


# ── Hand 모드 ──────────────────────────────────────────────────────────────







def _detect_fingertip_labels(df):
    """CSV 헤더에서 fingertip 라벨 목록을 자동 감지.

    baro_*_0 패턴의 컬럼을 찾아 중간 라벨을 추출.
      - Named:   baro_thumb_0  → 'thumb'
      - Numeric:  baro_f0_0    → 'f0'
    baro_raw_* 컬럼은 제외.
    """
    labels = []
    for col in df.columns:
        if col.startswith('baro_') and not col.startswith('baro_raw_') \
                and col.endswith('_0'):
            middle = col[len('baro_'):-len('_0')]
            if f'baro_{middle}_1' in df.columns:
                labels.append(middle)
    return labels


def _detect_fingertip_labels_raw(df):
    """CSV 헤더에서 raw sensor fingertip 라벨 목록을 자동 감지.

    baro_raw_*_0 패턴의 컬럼을 찾아 중간 라벨을 추출.
      - Named:   baro_raw_thumb_0  → 'thumb'
      - Numeric:  baro_raw_f0_0    → 'f0'
    """
    labels = []
    for col in df.columns:
        if col.startswith('baro_raw_') and col.endswith('_0'):
            middle = col[len('baro_raw_'):-len('_0')]
            if f'baro_raw_{middle}_1' in df.columns:
                labels.append(middle)
    return labels


def _detect_ft_labels(df):
    """CSV 헤더에서 FT output fingertip 라벨 목록을 자동 감지.

    ft_{label}_contact 패턴의 컬럼을 찾아 라벨을 추출.
    Legacy fallback: ft_{label}_fx 패턴도 지원.
    """
    labels = []
    for col in df.columns:
        if col.startswith('ft_') and col.endswith('_contact'):
            middle = col[len('ft_'):-len('_contact')]
            if middle != 'valid' and f'ft_{middle}_fx' in df.columns:
                labels.append(middle)
    if not labels:
        # Legacy fallback (6-output 모델)
        for col in df.columns:
            if col.startswith('ft_') and col.endswith('_fx'):
                middle = col[len('ft_'):-len('_fx')]
                if middle != 'valid' and f'ft_{middle}_fy' in df.columns:
                    labels.append(middle)
    return labels


def plot_device_positions(df, save_dir=None):
    """Figure 1: Hand motor positions (2x5 subplot)."""
    actual_cols, motor_names = _detect_joint_columns(df, 'hand_actual_pos_')
    cmd_cols, _ = _detect_joint_columns(df, 'hand_cmd_')
    goal_cols, _ = _detect_joint_columns(df, 'hand_goal_pos_')
    if not actual_cols:
        print('  Skipping hand positions plot (columns not found)')
        return

    n_motors = len(actual_cols)
    nrows, ncols_g = _auto_subplot_grid(n_motors)
    fig, axes = plt.subplots(nrows, ncols_g, figsize=(5 * ncols_g, 4 * nrows))
    fig.suptitle('Device Motor Positions', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    t = df['timestamp']
    for i in range(len(actual_cols)):
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
        path = Path(save_dir) / 'device_positions.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_device_velocities(df, save_dir=None):
    """Figure 2: Hand motor velocities (2x5 subplot)."""
    vel_cols, motor_names = _detect_joint_columns(df, 'hand_actual_vel_')
    if not vel_cols:
        print('  Skipping hand velocities plot (columns not found)')
        return

    n_motors = len(vel_cols)
    nrows, ncols_g = _auto_subplot_grid(n_motors)
    fig, axes = plt.subplots(nrows, ncols_g, figsize=(5 * ncols_g, 4 * nrows))
    fig.suptitle('Device Motor Velocities', fontsize=16, fontweight='bold')
    axes = axes.flatten()

    t = df['timestamp']
    for i in range(len(vel_cols)):
        ax = axes[i]
        ax.plot(t, df[vel_cols[i]], label='Actual', linewidth=1.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.set_title(f'Motor {i} ({motor_names[i]})')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'device_velocities.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_device_sensors(df, save_dir=None):
    """Figure 3: Hand sensor data (2 rows x N cols: barometer + ToF)."""
    labels = _detect_fingertip_labels(df)
    if not labels:
        print('  Skipping hand sensors plot (sensor columns not found)')
        return

    num_ft = len(labels)
    fig, axes = plt.subplots(2, num_ft, figsize=(5 * num_ft, 8))
    fig.suptitle('Device Sensor Data', fontsize=16, fontweight='bold')

    # fingertip이 1개면 axes가 1D가 되므로 2D로 보정
    if num_ft == 1:
        axes = axes.reshape(2, 1)

    t = df['timestamp']

    # Row 1: Barometer (8ch per fingertip)
    for f_idx, label in enumerate(labels):
        ax = axes[0, f_idx]
        for b in range(16):  # auto-stopped by column check
            col = f'baro_{label}_{b}'
            if col in df.columns:
                ax.plot(t, df[col], label=f'B{b}', alpha=0.7, linewidth=0.8)
        ax.set_title(f'{label} — Barometer')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Pressure')
        ax.legend(fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)

    # Row 2: ToF (3ch per fingertip)
    for f_idx, label in enumerate(labels):
        ax = axes[1, f_idx]
        for t_idx in range(8):  # auto-stopped by column check
            col = f'tof_{label}_{t_idx}'
            if col in df.columns:
                ax.plot(t, df[col], label=f'ToF{t_idx}',
                        linewidth=1.2, marker='.', markersize=1)
        ax.set_title(f'{label} — ToF')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'device_sensors.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_device_sensors_raw(df, fingertip_labels, save_dir=None):
    """Hand raw sensor data (pre-LPF): 2 rows x N cols (barometer + ToF)."""
    labels = fingertip_labels
    if not labels:
        print('  Skipping raw sensor plot (baro_raw_* columns not found)')
        return

    num_ft = len(labels)
    fig, axes = plt.subplots(2, num_ft, figsize=(5 * num_ft, 8))
    fig.suptitle('Device Sensor Data (Raw, pre-LPF)', fontsize=16, fontweight='bold')

    if num_ft == 1:
        axes = axes.reshape(2, 1)

    t = df['timestamp']

    # Row 1: Raw Barometer (8ch per fingertip)
    for f_idx, label in enumerate(labels):
        ax = axes[0, f_idx]
        for b in range(16):  # auto-stopped by column check
            col = f'baro_raw_{label}_{b}'
            if col in df.columns:
                ax.plot(t, df[col], label=f'B{b}', alpha=0.7, linewidth=0.8)
        ax.set_title(f'{label} — Barometer (Raw)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Pressure')
        ax.legend(fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)

    # Row 2: Raw ToF (3ch per fingertip)
    for f_idx, label in enumerate(labels):
        ax = axes[1, f_idx]
        for t_idx in range(8):  # auto-stopped by column check
            col = f'tof_raw_{label}_{t_idx}'
            if col in df.columns:
                ax.plot(t, df[col], label=f'ToF{t_idx}',
                        linewidth=1.2, marker='.', markersize=1)
        ax.set_title(f'{label} — ToF (Raw)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'device_sensors_raw.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def _plot_device_ft_legacy(df, labels, save_dir=None):
    """Legacy 6-output 모델: Force(3) + Torque(3)."""
    num_ft = len(labels)
    ncols = 2
    fig, axes = plt.subplots(num_ft, ncols, figsize=(14, 4 * num_ft),
                             squeeze=False)
    fig.suptitle('Fingertip Force/Torque (ONNX inference)',
                 fontsize=16, fontweight='bold')

    t = df['timestamp']
    for f_idx, label in enumerate(labels):
        ax_f = axes[f_idx, 0]
        for comp in ['fx', 'fy', 'fz']:
            col = f'ft_{label}_{comp}'
            if col in df.columns:
                ax_f.plot(t, df[col], label=comp.upper(), linewidth=1.2)
        ax_f.set_title(f'{label} — Force')
        ax_f.set_xlabel('Time (s)')
        ax_f.set_ylabel('Force (N)')
        ax_f.legend(fontsize=8)
        ax_f.grid(True, alpha=0.3)

        ax_t = axes[f_idx, 1]
        for comp in ['tx', 'ty', 'tz']:
            col = f'ft_{label}_{comp}'
            if col in df.columns:
                ax_t.plot(t, df[col], label=comp.upper(), linewidth=1.2)
        ax_t.set_title(f'{label} — Torque')
        ax_t.set_xlabel('Time (s)')
        ax_t.set_ylabel('Torque (Nm)')
        ax_t.legend(fontsize=8)
        ax_t.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'device_ft_output.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_device_ft_output(df, fingertip_labels, save_dir=None):
    """Fingertip sensor + inference combined view (3-head model).

    Grid: 4 rows x N_fingertips columns:
      Row 0: Barometer (8ch)
      Row 1: Contact Probability
      Row 2: Force F (Fx, Fy, Fz)
      Row 3: Direction u (uX, uY, uZ)

    같은 column(fingertip) 내 모든 row가 x축을 공유하여
    확대 시 동기화됨.
    """
    labels = fingertip_labels
    if not labels:
        print('  Skipping F/T plot (ft_*_contact / ft_*_fx columns not found)')
        return

    num_ft = len(labels)

    # 3-head 모델 여부 감지
    has_contact = f'ft_{labels[0]}_contact' in df.columns
    if not has_contact:
        _plot_device_ft_legacy(df, labels, save_dir)
        return

    # barometer 라벨 감지 (sensor 데이터 존재 여부)
    sensor_labels = _detect_fingertip_labels(df)

    nrows = 4  # Barometer | Contact | Force | Direction
    fig, axes = plt.subplots(nrows, num_ft, figsize=(5.5 * num_ft, 14),
                             squeeze=False)
    fig.suptitle('Fingertip Sensor + Inference Output',
                 fontsize=16, fontweight='bold')

    # 같은 column(fingertip) 내 모든 row가 x축을 공유하여 확대 시 동기화
    for col_idx in range(num_ft):
        for row_idx in range(1, nrows):
            axes[row_idx, col_idx].sharex(axes[0, col_idx])

    t = df['timestamp']

    for f_idx, label in enumerate(labels):
        # ── Row 0: Barometer (8ch) ──
        ax = axes[0, f_idx]
        if label in sensor_labels:
            for b in range(16):  # auto-stopped by column check
                col = f'baro_{label}_{b}'
                if col in df.columns:
                    ax.plot(t, df[col], label=f'B{b}', alpha=0.7,
                            linewidth=0.8)
            ax.legend(fontsize=6, ncol=4)
        ax.set_title(f'{label}', fontweight='bold')
        ax.grid(True, alpha=0.3)
        if f_idx == 0:
            ax.set_ylabel('Barometer\nPressure')

        # ── Row 1: Contact probability ──
        ax = axes[1, f_idx]
        col = f'ft_{label}_contact'
        if col in df.columns:
            ax.plot(t, df[col], linewidth=1.2, color='C0')
            ax.fill_between(t, 0, df[col], alpha=0.2, color='C0')
            ax.axhline(y=0.1, color='red', linestyle='--', alpha=0.5,
                       label='threshold (0.1)')
        ax.set_ylim(-0.05, 1.05)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        if f_idx == 0:
            ax.set_ylabel('Contact\nProbability')

        # ── Row 2: Force vector F ──
        ax = axes[2, f_idx]
        for comp, c_label in [('fx', 'Fx'), ('fy', 'Fy'), ('fz', 'Fz')]:
            col = f'ft_{label}_{comp}'
            if col in df.columns:
                ax.plot(t, df[col], label=c_label, linewidth=1.2)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        if f_idx == 0:
            ax.set_ylabel('Force (N)')

        # ── Row 3: Direction vector u ──
        ax = axes[3, f_idx]
        for comp, c_label in [('ux', 'uX'), ('uy', 'uY'), ('uz', 'uZ')]:
            col = f'ft_{label}_{comp}'
            if col in df.columns:
                ax.plot(t, df[col], label=c_label, linewidth=1.2)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Time (s)')
        if f_idx == 0:
            ax.set_ylabel('Direction\nUnit vector')

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'device_ft_output.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def plot_device_sensor_comparison(df, fingertip_labels, save_dir=None):
    """Sensor comparison: raw (alpha=0.3) vs filtered (alpha=1.0) overlay per fingertip."""
    labels = fingertip_labels
    if not labels:
        print('  Skipping sensor comparison plot (raw sensor columns not found)')
        return

    # filtered 라벨도 존재하는지 확인
    filtered_labels = _detect_fingertip_labels(df)
    if not filtered_labels:
        print('  Skipping sensor comparison plot (filtered sensor columns not found)')
        return

    # 두 라벨 집합의 교집합 사용
    common_labels = [l for l in labels if l in filtered_labels]
    if not common_labels:
        print('  Skipping sensor comparison plot (no matching raw/filtered labels)')
        return

    num_ft = len(common_labels)
    fig, axes = plt.subplots(2, num_ft, figsize=(5 * num_ft, 8))
    fig.suptitle('Sensor Comparison: Raw vs Filtered',
                 fontsize=16, fontweight='bold')

    if num_ft == 1:
        axes = axes.reshape(2, 1)

    t = df['timestamp']
    colors = plt.cm.tab10.colors

    # Row 1: Barometer comparison
    for f_idx, label in enumerate(common_labels):
        ax = axes[0, f_idx]
        for b in range(16):  # auto-stopped by column check
            color = colors[b % len(colors)]
            raw_col = f'baro_raw_{label}_{b}'
            filt_col = f'baro_{label}_{b}'
            if raw_col in df.columns:
                ax.plot(t, df[raw_col], alpha=0.3, linewidth=0.6,
                        color=color)
            if filt_col in df.columns:
                ax.plot(t, df[filt_col], alpha=1.0, linewidth=0.8,
                        color=color, label=f'B{b}')
        ax.set_title(f'{label} — Barometer (solid=filtered, faint=raw)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Pressure')
        ax.legend(fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)

    # Row 2: ToF comparison
    for f_idx, label in enumerate(common_labels):
        ax = axes[1, f_idx]
        for t_idx in range(8):  # auto-stopped by column check
            color = colors[t_idx % len(colors)]
            raw_col = f'tof_raw_{label}_{t_idx}'
            filt_col = f'tof_{label}_{t_idx}'
            if raw_col in df.columns:
                ax.plot(t, df[raw_col], alpha=0.3, linewidth=0.8,
                        color=color)
            if filt_col in df.columns:
                ax.plot(t, df[filt_col], alpha=1.0, linewidth=1.2,
                        color=color, label=f'ToF{t_idx}',
                        marker='.', markersize=1)
        ax.set_title(f'{label} — ToF (solid=filtered, faint=raw)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / 'device_sensor_comparison.png'
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f'Saved: {path}')
    else:
        plt.show()
    plt.close()


def print_device_statistics(df):
    """Print hand trajectory statistics."""
    duration = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
    print('\n=== Hand Trajectory Statistics ===')
    print(f'Duration: {duration:.2f} s | Samples: {len(df)}'
          f' | Rate: {len(df) / duration:.1f} Hz')

    if 'hand_valid' in df.columns:
        valid_ratio = df['hand_valid'].sum() / len(df) * 100
        print(f'Hand valid: {valid_ratio:.1f}%')

    goal_cols, _ = _detect_joint_columns(df, 'hand_goal_pos_')
    actual_cols, motor_names = _detect_joint_columns(df, 'hand_actual_pos_')
    if goal_cols and actual_cols:
        print('\nPosition Tracking Error (RMS):')
        for i in range(min(len(goal_cols), len(actual_cols))):
            err = df[goal_cols[i]] - df[actual_cols[i]]
            rms = np.sqrt(np.mean(err ** 2))
            print(f'  Motor {i} ({motor_names[i]}): {rms:.6f}')

    # Sensor summary (filtered 컬럼만, raw 제외)
    baro_cols = [c for c in df.columns
                 if c.startswith('baro_') and not c.startswith('baro_raw_')]
    tof_cols = [c for c in df.columns
                if c.startswith('tof_') and not c.startswith('tof_raw_')]
    if baro_cols:
        baro_vals = df[baro_cols].values.flatten()
        print(f'\nBarometer range: [{baro_vals.min():.1f}, {baro_vals.max():.1f}]')
    if tof_cols:
        tof_vals = df[tof_cols].values.flatten()
        print(f'ToF range: [{tof_vals.min():.1f}, {tof_vals.max():.1f}]')

    # FT inference output statistics
    ft_labels = _detect_ft_labels(df)
    if ft_labels:
        # 3-head 모델 여부 감지 (contact 컬럼 존재)
        has_contact = f'ft_{ft_labels[0]}_contact' in df.columns
        if has_contact:
            ft_comps = ['contact', 'fx', 'fy', 'fz',
                        'ux', 'uy', 'uz']
        else:
            ft_comps = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']
        print('\nF/T Inference Output:')
        if 'ft_valid' in df.columns:
            ft_valid_ratio = df['ft_valid'].sum() / len(df) * 100
            print(f'  FT valid: {ft_valid_ratio:.1f}%')
        for label in ft_labels:
            print(f'  {label}:')
            for comp in ft_comps:
                col = f'ft_{label}_{comp}'
                if col in df.columns:
                    data = df[col]
                    print(f'    {comp}: mean={data.mean():.4f}'
                          f'  std={data.std():.4f}'
                          f'  min={data.min():.4f}'
                          f'  max={data.max():.4f}')


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
    """Detect log type from filename pattern.

    New patterns (topic-role based):
      *_state_log.csv  → state_log  (DeviceStateLog fields)
      *_sensor_log.csv → sensor_log (DeviceSensorLog fields)
    Legacy patterns (backward compat):
      robot_log*.csv   → robot
      device_log*.csv / hand_log*.csv → device
      timing_log*.csv  → timing
    """
    stem = Path(filepath).stem
    if stem.endswith('state_log'):
        return 'state_log'
    elif stem.endswith('sensor_log'):
        return 'sensor_log'
    elif stem.startswith('robot_log'):
        return 'robot'
    elif stem.startswith('device_log') or stem.startswith('hand_log'):
        return 'device'
    elif stem.startswith('timing_log'):
        return 'timing'
    else:
        return 'unknown'


def main():
    parser = argparse.ArgumentParser(
        description='Plot RTC log data from split CSV logs (4-category)')
    parser.add_argument('csv_file', type=str,
                        help='Path to robot_log*.csv, device_log*.csv, or timing_log*.csv')
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
    parser.add_argument('--raw', action='store_true',
                        help='Plot raw sensor data (pre-LPF) (hand only)')
    parser.add_argument('--ft', action='store_true',
                        help='Plot F/T inference output (hand only)')
    parser.add_argument('--sensor-compare', action='store_true',
                        help='Plot raw vs filtered sensor comparison (hand only)')
    parser.add_argument('--all', action='store_true',
                        help='Plot all available figures')

    args = parser.parse_args()

    # --all 플래그로 모든 플롯 활성화
    if args.all:
        args.error = True
        args.command = True
        args.torque = True
        args.task_pos = True
        args.raw = True
        args.ft = True
        args.sensor_compare = True

    # --save-dir 미지정 시 세션 디렉토리의 plots/ 서브디렉토리를 기본값으로 사용
    if args.save_dir is None:
        session = os.environ.get('RTC_SESSION_DIR', os.environ.get('UR5E_SESSION_DIR', ''))
        if session:
            args.save_dir = os.path.join(session, 'plots')

    # Agg backend 최적화: --save-dir 지정 시 GUI 없이 렌더링
    global plt
    if args.save_dir:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as _plt
    plt = _plt

    log_type = detect_log_type(args.csv_file)
    if log_type == 'unknown':
        print(f'Error: Cannot detect log type from filename: {args.csv_file}')
        print('Expected: *_state_log.csv, *_sensor_log.csv, timing_log*.csv, '
              'robot_log*.csv, device_log*.csv, hand_log*.csv')
        sys.exit(1)

    print(f'Loading ({log_type}): {args.csv_file}')
    df = pd.read_csv(args.csv_file)

    # 새 DataFrame 로드 시 컬럼 감지 캐시 초기화
    _invalidate_column_cache()

    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)

    if log_type in ('robot', 'state_log'):
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
    elif log_type == 'sensor_log':
        print_device_statistics(df)
        if not args.stats:
            plot_device_sensors(df, args.save_dir)
            if args.raw:
                raw_labels = _detect_fingertip_labels_raw(df)
                plot_device_sensors_raw(df, raw_labels, args.save_dir)
            if args.ft:
                ft_labels = _detect_ft_labels(df)
                plot_device_ft_output(df, ft_labels, args.save_dir)
            if args.sensor_compare:
                raw_labels = _detect_fingertip_labels_raw(df)
                plot_device_sensor_comparison(df, raw_labels, args.save_dir)
    elif log_type == 'device':
        print_device_statistics(df)
        if not args.stats:
            plot_device_positions(df, args.save_dir)
            plot_device_velocities(df, args.save_dir)
            plot_device_sensors(df, args.save_dir)
            if args.raw:
                raw_labels = _detect_fingertip_labels_raw(df)
                plot_device_sensors_raw(df, raw_labels, args.save_dir)
            if args.ft:
                ft_labels = _detect_ft_labels(df)
                plot_device_ft_output(df, ft_labels, args.save_dir)
            if args.sensor_compare:
                raw_labels = _detect_fingertip_labels_raw(df)
                plot_device_sensor_comparison(df, raw_labels, args.save_dir)
    elif log_type == 'timing':
        print_timing_statistics(df)
        if not args.stats:
            plot_timing_breakdown(df, args.save_dir)
            plot_timing_total_and_jitter(df, args.save_dir)
            plot_timing_histograms(df, args.save_dir)


if __name__ == '__main__':
    main()
