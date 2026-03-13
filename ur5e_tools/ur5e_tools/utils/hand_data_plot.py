#!/usr/bin/env python3
"""
핸드 데이터 CSV 플롯 — hand_udp_sender_example.py에서 저장한 CSV 시각화

사용법:
  python3 hand_data_plot.py <csv_file>
  python3 hand_data_plot.py hand_poll_20260313_143000.csv
  python3 hand_data_plot.py hand_read_20260313_143000.csv --motors 0 1 2
  python3 hand_data_plot.py hand_read_20260313_143000.csv --sensors-only

CSV 컬럼 구조:
  timestamp, cycle,
  pos_0..pos_9, vel_0..vel_9,
  s0_baro_0..s0_baro_7, s0_tof_0..s0_tof_2,
  s1_baro_0..s1_baro_7, s1_tof_0..s1_tof_2,
  ...
"""

import argparse
import sys

import matplotlib.pyplot as plt
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


def main():
    parser = argparse.ArgumentParser(description="핸드 데이터 CSV 플롯")
    parser.add_argument("csv_file", help="CSV 파일 경로")
    parser.add_argument("--motors", type=int, nargs="+",
                        help="표시할 모터 인덱스 (예: 0 1 2). 기본=전체")
    parser.add_argument("--sensors-only", action="store_true",
                        help="센서 데이터만 표시 (모터 생략)")
    parser.add_argument("--motors-only", action="store_true",
                        help="모터 데이터만 표시 (센서 생략)")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors='coerce')
    print(f"CSV 로드: {args.csv_file}")
    print(f"  행 수: {len(df)}, 컬럼 수: {len(df.columns)}")
    print(f"  시간 범위: {df['timestamp'].iloc[0]:.3f} ~ {df['timestamp'].iloc[-1]:.3f} s")

    num_sensors = detect_num_sensors(df)
    print(f"  감지된 센서 수: {num_sensors}")

    if not args.sensors_only:
        plot_motor_positions(df, args.motors)
        plot_motor_velocities(df, args.motors)

    if not args.motors_only and num_sensors > 0:
        plot_sensor_barometer(df, num_sensors)
        plot_sensor_tof(df, num_sensors)

    plt.show()


if __name__ == "__main__":
    main()
