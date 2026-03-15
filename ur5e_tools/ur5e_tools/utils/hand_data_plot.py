#!/usr/bin/env python3
"""
핸드 데이터 CSV 플롯 — hand_udp_sender_example.py에서 저장한 CSV 시각화

사용법:
  python3 hand_data_plot.py <csv_file>
  python3 hand_data_plot.py hand_poll_20260313_143000.csv
  python3 hand_data_plot.py hand_read_20260313_143000.csv --motors 0 1 2
  python3 hand_data_plot.py hand_read_20260313_143000.csv --sensors-only
  python3 hand_data_plot.py hand_poll_20260313_143000.csv --timing-only

CSV 컬럼 구조:
  timestamp, cycle,
  pos_0..pos_9, vel_0..vel_9,
  s0_baro_0..s0_baro_7, s0_tof_0..s0_tof_2,
  s1_baro_0..s1_baro_7, s1_tof_0..s1_tof_2,
  ...,
  t_cycle, t_write_position, t_read_position, t_read_velocity, t_read_sensors, t_timeouts
"""

import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

NUM_HAND_MOTORS = 10
BAROMETER_COUNT = 8
TOF_COUNT = 3
SENSOR_VALUES_PER_FINGERTIP = BAROMETER_COUNT + TOF_COUNT  # 11


def detect_num_sensors(df: pd.DataFrame) -> int:
    """CSV 컬럼에서 센서 수 자동 감지"""
    for i in range(4, -1, -1):
        if f"s{i}_baro_0" in df.columns:
            return i + 1
    return 0


def plot_motor_positions(df: pd.DataFrame, motor_indices: list[int] | None = None):
    """모터 위치 플롯"""
    pos_cols = [c for c in df.columns if c.startswith("pos_")]
    if not pos_cols:
        print("모터 위치 데이터 없음")
        return

    if motor_indices:
        pos_cols = [f"pos_{i}" for i in motor_indices if f"pos_{i}" in df.columns]

    fig, ax = plt.subplots(figsize=(12, 5))
    for col in pos_cols:
        valid = pd.to_numeric(df[col], errors='coerce')
        ax.plot(df["timestamp"], valid, label=col, linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position")
    ax.set_title("Motor Positions")
    ax.legend(fontsize=7, ncol=5)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()


def plot_motor_velocities(df: pd.DataFrame, motor_indices: list[int] | None = None):
    """모터 속도 플롯"""
    vel_cols = [c for c in df.columns if c.startswith("vel_")]
    if not vel_cols:
        print("모터 속도 데이터 없음")
        return

    if motor_indices:
        vel_cols = [f"vel_{i}" for i in motor_indices if f"vel_{i}" in df.columns]

    fig, ax = plt.subplots(figsize=(12, 5))
    for col in vel_cols:
        valid = pd.to_numeric(df[col], errors='coerce')
        ax.plot(df["timestamp"], valid, label=col, linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity")
    ax.set_title("Motor Velocities")
    ax.legend(fontsize=7, ncol=5)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()


def plot_sensor_barometer(df: pd.DataFrame, num_sensors: int):
    """핑거팁별 기압 센서 데이터 플롯 (uint32)"""
    fig, axes = plt.subplots(num_sensors, 1, figsize=(12, 3 * num_sensors),
                              sharex=True)
    if num_sensors == 1:
        axes = [axes]

    for s in range(num_sensors):
        ax = axes[s]
        for j in range(BAROMETER_COUNT):
            col = f"s{s}_baro_{j}"
            if col in df.columns:
                valid = pd.to_numeric(df[col], errors='coerce')
                ax.plot(df["timestamp"], valid, label=f"baro_{j}", linewidth=0.8)
        ax.set_ylabel(f"Fingertip {s}\nBarometer (uint32)")
        ax.legend(fontsize=7, ncol=4, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Barometer Sensor Data (uint32)", fontsize=13)
    fig.tight_layout()


def plot_sensor_tof(df: pd.DataFrame, num_sensors: int):
    """핑거팁별 ToF 센서 데이터 플롯 (uint32)"""
    fig, axes = plt.subplots(num_sensors, 1, figsize=(12, 3 * num_sensors),
                              sharex=True)
    if num_sensors == 1:
        axes = [axes]

    for s in range(num_sensors):
        ax = axes[s]
        for j in range(TOF_COUNT):
            col = f"s{s}_tof_{j}"
            if col in df.columns:
                valid = pd.to_numeric(df[col], errors='coerce')
                ax.plot(df["timestamp"], valid, label=f"tof_{j}", linewidth=0.8)
        ax.set_ylabel(f"Fingertip {s}\nToF (uint32)")
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("ToF Sensor Data (uint32)", fontsize=13)
    fig.tight_layout()


TIMING_COLS = ["t_cycle", "t_write_position", "t_read_position",
               "t_read_velocity", "t_read_sensors"]
TIMING_LABELS = {"t_cycle": "cycle_total", "t_write_position": "write_pos",
                 "t_read_position": "read_pos", "t_read_velocity": "read_vel",
                 "t_read_sensors": "read_sensors"}


def has_timing_data(df: pd.DataFrame) -> bool:
    """CSV에 타이밍 컬럼이 존재하는지 확인"""
    return any(c in df.columns for c in TIMING_COLS)


def plot_timing(df: pd.DataFrame):
    """UDP 통신 타이밍 플롯 (시계열 + 히스토그램 + 요약 통계)"""
    available = [c for c in TIMING_COLS if c in df.columns]
    if not available:
        print("타이밍 데이터 없음 (t_cycle 등 컬럼 미존재)")
        return

    # 시계열 플롯
    fig, axes = plt.subplots(len(available), 1,
                              figsize=(12, 2.5 * len(available)), sharex=True)
    if len(available) == 1:
        axes = [axes]

    for ax, col in zip(axes, available):
        data = pd.to_numeric(df[col], errors='coerce')
        label = TIMING_LABELS.get(col, col)
        ax.plot(df["timestamp"], data, linewidth=0.5, alpha=0.7, label=label)
        avg = data.mean()
        ax.axhline(avg, color='r', linestyle='--', linewidth=0.8,
                   label=f"avg={avg:.3f}ms")
        ax.set_ylabel(f"{label} (ms)")
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("UDP Communication Timing (ms)", fontsize=13)
    fig.tight_layout()

    # 히스토그램 플롯
    fig2, axes2 = plt.subplots(1, len(available),
                                figsize=(4 * len(available), 4))
    if len(available) == 1:
        axes2 = [axes2]

    for ax, col in zip(axes2, available):
        data = pd.to_numeric(df[col], errors='coerce').dropna()
        label = TIMING_LABELS.get(col, col)
        ax.hist(data, bins=50, edgecolor='black', linewidth=0.5, alpha=0.7)
        avg, std = data.mean(), data.std()
        p99 = np.percentile(data, 99)
        ax.axvline(avg, color='r', linestyle='--', linewidth=1,
                   label=f"avg={avg:.3f}")
        ax.axvline(p99, color='orange', linestyle='--', linewidth=1,
                   label=f"p99={p99:.3f}")
        ax.set_xlabel(f"{label} (ms)")
        ax.set_ylabel("Count")
        ax.set_title(f"{label}\navg={avg:.3f} std={std:.3f}\n"
                     f"min={data.min():.3f} max={data.max():.3f} p99={p99:.3f}")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig2.suptitle("UDP Timing Distribution (ms)", fontsize=13)
    fig2.tight_layout()

    # timeout 플롯
    if "t_timeouts" in df.columns:
        timeout_data = pd.to_numeric(df["t_timeouts"], errors='coerce')
        total_timeouts = int(timeout_data.sum())
        if total_timeouts > 0:
            fig3, ax3 = plt.subplots(figsize=(12, 3))
            ax3.plot(df["timestamp"], timeout_data, linewidth=0.8,
                     color='red', alpha=0.7)
            ax3.set_xlabel("Time (s)")
            ax3.set_ylabel("Timeouts (per cycle)")
            ax3.set_title(f"UDP Timeouts (total={total_timeouts})")
            ax3.grid(True, alpha=0.3)
            fig3.tight_layout()

    # 요약 통계 출력
    print("\n── UDP Timing Stats (ms) ──")
    print(f"  {'metric':<15s} {'avg':>8s} {'std':>8s} {'min':>8s} {'max':>8s} {'p99':>8s}")
    for col in available:
        data = pd.to_numeric(df[col], errors='coerce').dropna()
        label = TIMING_LABELS.get(col, col)
        print(f"  {label:<15s} {data.mean():8.3f} {data.std():8.3f} "
              f"{data.min():8.3f} {data.max():8.3f} {np.percentile(data, 99):8.3f}")
    if "t_timeouts" in df.columns:
        timeout_data = pd.to_numeric(df["t_timeouts"], errors='coerce')
        print(f"  {'timeouts':<15s} total={int(timeout_data.sum())}")


def main():
    parser = argparse.ArgumentParser(description="핸드 데이터 CSV 플롯")
    parser.add_argument("csv_file", help="CSV 파일 경로")
    parser.add_argument("--motors", type=int, nargs="+",
                        help="표시할 모터 인덱스 (예: 0 1 2). 기본=전체")
    parser.add_argument("--sensors-only", action="store_true",
                        help="센서 데이터만 표시 (모터 생략)")
    parser.add_argument("--motors-only", action="store_true",
                        help="모터 데이터만 표시 (센서 생략)")
    parser.add_argument("--timing-only", action="store_true",
                        help="타이밍 데이터만 표시 (모터·센서 생략)")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors='coerce')
    print(f"CSV 로드: {args.csv_file}")
    print(f"  행 수: {len(df)}, 컬럼 수: {len(df.columns)}")
    print(f"  시간 범위: {df['timestamp'].iloc[0]:.3f} ~ {df['timestamp'].iloc[-1]:.3f} s")

    num_sensors = detect_num_sensors(df)
    has_timing = has_timing_data(df)
    print(f"  감지된 센서 수: {num_sensors}")
    print(f"  타이밍 데이터: {'있음' if has_timing else '없음'}")

    if args.timing_only:
        if has_timing:
            plot_timing(df)
        else:
            print("타이밍 데이터가 CSV에 없습니다.")
        plt.show()
        return

    if not args.sensors_only:
        plot_motor_positions(df, args.motors)
        plot_motor_velocities(df, args.motors)

    if not args.motors_only and num_sensors > 0:
        plot_sensor_barometer(df, num_sensors)
        plot_sensor_tof(df, num_sensors)

    if has_timing:
        plot_timing(df)

    plt.show()


if __name__ == "__main__":
    main()
