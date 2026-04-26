#!/usr/bin/env python3
"""
plot_rtc_log.py - v6

Visualize robot trajectory data from split CSV logs (4-category convention).

CSV 컬럼 카테고리:
  1. Goal State — 외부 입력 목표
  2. Current State — 센서 피드백
  3. Control Command — 액추에이터 출력
  4. Trajectory State — 궤적 보간 내부 상태

파일 이름으로 robot/hand 데이터를 자동 구분:
  - robot_log*.csv / *_state_log.csv  → Robot 모드
  - hand_log*.csv  / *_sensor_log.csv → Hand/sensor 모드
  - timing_log*.csv                    → CM RT loop timing
  - mpc_solve_timing.csv               → MPC solver latency

이 파일은 thin orchestration layer다 — actual implementations live in:
  - io/         CSV load, log-type detect, save-dir resolve
  - columns/    variable-DOF column detection
  - layout.py   subplot grid + backend + save/show
  - plotters/   Phase 2에서 분리됨
  - pipelines/  Phase 4에서 분리됨
"""

import argparse
import os
import sys
from pathlib import Path

import pandas as pd

from rtc_tools.plotting.columns import invalidate_column_cache
from rtc_tools.plotting.io import (
    detect_log_type,
    detect_log_type_by_columns,
    load_log_csv,
    peek_csv_header,
    resolve_default_save_dir,
)
from rtc_tools.plotting import layout

# ── Main ──────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Plot RTC log data from split CSV logs (4-category)"
    )
    parser.add_argument(
        "csv_file",
        type=str,
        help="Path to robot_log*.csv, device_log*.csv, or timing_log*.csv",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default=None,
        help="Directory to save plots (PNG). "
        "미지정 시 RTC_SESSION_DIR/plots/ (없으면 "
        "현재 colcon ws logging_data 의 최신 세션) 사용",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display plots in GUI in addition to saving. "
        "Without this flag, --save-dir (auto-resolved by default) forces the "
        "Agg backend and only PNGs are written.",
    )
    parser.add_argument(
        "--stats", action="store_true", help="Print statistics only (no plots)"
    )
    parser.add_argument(
        "--error",
        action="store_true",
        help="Also plot tracking error figures (robot only)",
    )
    parser.add_argument(
        "--command",
        action="store_true",
        help="Also plot control command figures (robot only)",
    )
    parser.add_argument(
        "--torque",
        action="store_true",
        help="Also plot actual torque figures (robot only)",
    )
    parser.add_argument(
        "--task-pos",
        action="store_true",
        help="Also plot TCP task-space position (robot only)",
    )
    parser.add_argument(
        "--raw", action="store_true", help="Plot raw sensor data (pre-LPF) (hand only)"
    )
    parser.add_argument(
        "--ft", action="store_true", help="Plot F/T inference output (hand only)"
    )
    parser.add_argument(
        "--sensor-compare",
        action="store_true",
        help="Plot raw vs filtered sensor comparison (hand only)",
    )
    parser.add_argument("--all", action="store_true", help="Plot all available figures")

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

    # --save-dir 미지정 시 세션 디렉토리의 plots/ 서브디렉토리를 기본값으로 사용.
    if args.save_dir is None:
        args.save_dir = resolve_default_save_dir()

    # Backend 설정. plotters/*는 import 시점에 matplotlib.pyplot을 import하므로
    # 반드시 configure_backend() 이후에 lazy-import 한다.
    plt = layout.configure_backend(save_only=bool(args.save_dir) and not args.show)
    if args.show:
        layout.disable_close()

    # pipelines.registry imports plotters/* (which import matplotlib.pyplot),
    # so it must be imported AFTER configure_backend().
    from rtc_tools.plotting.pipelines import run_pipeline

    if not os.path.isfile(args.csv_file):
        print(f"Error: file not found: {args.csv_file}")
        sys.exit(1)

    log_type = detect_log_type(args.csv_file)
    if log_type == "unknown":
        header = peek_csv_header(args.csv_file)
        log_type = detect_log_type_by_columns(header)
        if log_type != "unknown":
            print(
                f"Note: filename did not match known patterns; "
                f"detected type from CSV columns: {log_type}"
            )

    if log_type == "unknown":
        print(
            f"Error: Cannot detect log type from filename or columns: {args.csv_file}"
        )
        print(
            "Expected filenames: *_state_log.csv, *_sensor_log.csv, "
            "timing_log*.csv, robot_log*.csv, device_log*.csv, hand_log*.csv, "
            "mpc_solve_timing.csv"
        )
        sys.exit(1)

    print(f"Loading ({log_type}): {args.csv_file}")
    try:
        df = load_log_csv(args.csv_file, log_type)
    except pd.errors.EmptyDataError:
        print(f"Error: CSV file is empty or has no columns: {args.csv_file}")
        print(
            "The log file may not have been written (e.g. controller crashed before logging)."
        )
        sys.exit(1)
    except pd.errors.ParserError as e:
        print(f'Warning: CSV parse error — retrying with on_bad_lines="warn": {e}')
        df = pd.read_csv(args.csv_file, on_bad_lines="warn")
        if df.empty:
            print(
                f"Error: No valid rows after skipping malformed lines: {args.csv_file}"
            )
            sys.exit(1)
        print(f"Loaded {len(df)} valid rows (some malformed lines were skipped).")

    if df.empty:
        print(f"Error: No valid data rows in {args.csv_file}")
        sys.exit(1)

    # 새 DataFrame 로드 시 컬럼 감지 캐시 초기화
    invalidate_column_cache()

    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)

    run_pipeline(log_type, df, args, args.save_dir)

    if args.show and not args.stats:
        plt.show()


if __name__ == "__main__":
    main()
