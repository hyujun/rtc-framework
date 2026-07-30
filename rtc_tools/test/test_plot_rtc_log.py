"""Tests for rtc_tools.plotting.plot_rtc_log module.

CSV 유틸리티, 컬럼 감지, 로그 타입 분류, subplot 그리드 계산 등
플롯 의존 없이 테스트할 수 있는 순수 로직을 검증합니다.
"""

from __future__ import annotations

import csv

import numpy as np
import pandas as pd
import pytest

from rtc_tools.plotting.columns.detect import (
    detect_fingertip_labels as _detect_fingertip_labels,
    detect_fingertip_labels_raw as _detect_fingertip_labels_raw,
    detect_ft_labels as _detect_ft_labels,
    detect_joint_columns as _detect_joint_columns,
    detect_num_channels as _detect_num_channels,
    has_columns as _has_columns,
    invalidate_column_cache as _invalidate_column_cache,
)
from rtc_tools.plotting.io import detect_log_type
from rtc_tools.plotting.io.csv_loader import LegacyCsvError, load_log_csv
from rtc_tools.plotting.io.log_type import (
    detect_log_type_by_columns,
    peek_csv_header as _peek_csv_header,
)
from rtc_tools.plotting.layout import auto_subplot_grid as _auto_subplot_grid

# ═══════════════════════════════════════════════════════════════════════════
# Fixtures
# ═══════════════════════════════════════════════════════════════════════════


@pytest.fixture(autouse=True)
def _clear_cache():
    """테스트 간 컬럼 감지 캐시 격리."""
    _invalidate_column_cache()
    yield
    _invalidate_column_cache()


def _write_csv(path, header: list[str], rows: list[list]):
    """테스트용 CSV 파일 생성 헬퍼."""
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


# ═══════════════════════════════════════════════════════════════════════════
# detect_log_type — 파일명으로 로그 타입 분류
# ═══════════════════════════════════════════════════════════════════════════


class TestDetectLogType:
    """detect_log_type: 파일명 패턴으로 로그 종류를 자동 분류."""

    def test_state_log(self):
        assert detect_log_type("arm_state_log.csv") == "state_log"

    def test_sensor_log(self):
        assert detect_log_type("hand_sensor_log.csv") == "sensor_log"

    def test_cm_timing_log(self):
        assert detect_log_type("cm_timing_log.csv") == "cm_timing"

    def test_mpc_timing_log(self):
        assert detect_log_type("mpc_timing_log.csv") == "mpc_timing"

    def test_unknown(self):
        assert detect_log_type("random_file.csv") == "unknown"

    def test_full_path(self):
        assert detect_log_type("/data/logs/arm_state_log.csv") == "state_log"

    def test_sensor_log_with_prefix(self):
        assert detect_log_type("hand_sensor_log.csv") == "sensor_log"

    def test_state_log_with_prefix(self):
        assert detect_log_type("device0_state_log.csv") == "state_log"

    def test_hand_udp_timing_log(self):
        assert detect_log_type("hand_udp_timing_log.csv") == "cm_timing"

    def test_phase_c_controller_owned_returns_unknown_for_column_fallback(self, tmp_path):
        # Phase C: <session>/controllers/<key>/<instance>.csv — stem alone
        # carries no type, parent-of-parent is "controllers". Filename
        # stage returns "unknown" so the column fallback decides.
        ctrl_dir = tmp_path / "260501_1430" / "controllers" / "demo_controller"
        ctrl_dir.mkdir(parents=True)
        f = ctrl_dir / "arm.csv"
        f.write_text("t_relative_s,actual_pos_shoulder\n0.0,0.1\n")
        assert detect_log_type(f) == "unknown"


# ═══════════════════════════════════════════════════════════════════════════
# detect_log_type_by_columns — 컬럼 기반 fallback 분류
# ═══════════════════════════════════════════════════════════════════════════


class TestDetectLogTypeByColumns:
    """detect_log_type_by_columns: 파일명이 안 맞을 때 CSV 헤더로 분류."""

    def test_timing_by_t_total_us(self):
        cols = ["timestamp", "t_total_us", "jitter_us"]
        assert detect_log_type_by_columns(cols) == "cm_timing"

    def test_timing_by_jitter_only(self):
        assert detect_log_type_by_columns(["timestamp", "jitter_us"]) == "cm_timing"

    def test_state_log_by_actual_pos(self):
        cols = ["timestamp", "actual_pos_0", "actual_pos_1", "goal_pos_0"]
        assert detect_log_type_by_columns(cols) == "state_log"

    def test_sensor_log_by_baro_raw(self):
        cols = ["timestamp", "baro_raw_thumb_0", "tof_raw_thumb_0"]
        assert detect_log_type_by_columns(cols) == "sensor_log"

    def test_empty_columns(self):
        assert detect_log_type_by_columns([]) == "unknown"

    def test_only_timestamp(self):
        assert detect_log_type_by_columns(["timestamp"]) == "unknown"

    def test_timing_priority_over_state(self):
        # Defensive: timing columns win if both appear.
        cols = ["timestamp", "t_total_us", "actual_pos_0"]
        assert detect_log_type_by_columns(cols) == "cm_timing"

    def test_phase_c_state_log_by_joint_named_columns(self):
        # Phase C POD writes per-joint named columns instead of numeric
        # suffixes (e.g. actual_pos_shoulder_pan).
        cols = [
            "t_relative_s",
            "actual_pos_shoulder_pan",
            "joint_goal_shoulder_pan",
            "traj_pos_shoulder_pan",
        ]
        assert detect_log_type_by_columns(cols) == "state_log"

    def test_phase_c_sensor_log_by_named_fingertip_columns(self):
        # Phase C POD: <fingertip>_raw_<i>, <fingertip>_filt_<i>,
        # ft_<fingertip>_<col>.
        cols = [
            "t_relative_s",
            "thumb_raw_0",
            "thumb_filt_0",
            "ft_thumb_contact",
        ]
        assert detect_log_type_by_columns(cols) == "sensor_log"


# ═══════════════════════════════════════════════════════════════════════════
# _peek_csv_header — 헤더 미리보기
# ═══════════════════════════════════════════════════════════════════════════


class TestPeekCsvHeader:
    """_peek_csv_header: CSV 첫 줄을 컬럼 리스트로 반환."""

    def test_reads_header(self, tmp_path):
        path = tmp_path / "timing_stat.csv"
        _write_csv(path, ["timestamp", "t_total_us", "jitter_us"], [[0.0, 100, 5]])
        assert _peek_csv_header(str(path)) == ["timestamp", "t_total_us", "jitter_us"]

    def test_missing_file_returns_empty(self, tmp_path):
        assert _peek_csv_header(str(tmp_path / "nope.csv")) == []

    def test_renamed_timing_csv_classifies_as_timing(self, tmp_path):
        # 통합 시나리오: timing_stat.csv 같은 임의 이름도 컬럼으로 인식.
        path = tmp_path / "timing_stat.csv"
        _write_csv(path, ["timestamp", "t_total_us", "jitter_us"], [[0.0, 100, 5]])
        assert detect_log_type(str(path)) == "unknown"
        assert detect_log_type_by_columns(_peek_csv_header(str(path))) == "cm_timing"


# ═══════════════════════════════════════════════════════════════════════════
# _auto_subplot_grid — 최적 subplot 배치 계산
# ═══════════════════════════════════════════════════════════════════════════


class TestAutoSubplotGrid:
    """_auto_subplot_grid: n개 subplot의 (nrows, ncols) 최적 배치."""

    def test_zero(self):
        assert _auto_subplot_grid(0) == (1, 1)

    def test_negative(self):
        assert _auto_subplot_grid(-5) == (1, 1)

    def test_one(self):
        assert _auto_subplot_grid(1) == (1, 1)

    def test_two(self):
        assert _auto_subplot_grid(2) == (1, 2)

    def test_three(self):
        assert _auto_subplot_grid(3) == (1, 3)

    def test_four(self):
        assert _auto_subplot_grid(4) == (2, 2)

    def test_six(self):
        nrows, ncols = _auto_subplot_grid(6)
        assert nrows * ncols >= 6
        assert ncols >= nrows  # wider than tall

    def test_nine(self):
        assert _auto_subplot_grid(9) == (3, 3)

    def test_ten(self):
        nrows, ncols = _auto_subplot_grid(10)
        assert nrows * ncols >= 10
        assert ncols >= nrows

    def test_sixteen(self):
        assert _auto_subplot_grid(16) == (4, 4)

    def test_wider_than_tall(self):
        """모든 케이스에서 ncols >= nrows."""
        for n in range(1, 25):
            nrows, ncols = _auto_subplot_grid(n)
            assert ncols >= nrows, f"n={n}: {nrows}x{ncols} is taller than wide"
            assert nrows * ncols >= n, f"n={n}: {nrows}x{ncols} has fewer cells than n"

    def test_max_cols_constraint(self):
        nrows, ncols = _auto_subplot_grid(10, max_cols=3)
        assert ncols == 3
        assert nrows * ncols >= 10


# ═══════════════════════════════════════════════════════════════════════════
# _detect_num_channels — prefix로 컬럼 수 감지
# ═══════════════════════════════════════════════════════════════════════════


class TestDetectNumChannels:
    def test_basic(self):
        df = pd.DataFrame(
            {
                "actual_pos_0": [1],
                "actual_pos_1": [2],
                "actual_pos_2": [3],
                "actual_vel_0": [4],
            }
        )
        assert _detect_num_channels(df, "actual_pos_") == 3
        assert _detect_num_channels(df, "actual_vel_") == 1

    def test_named_columns(self):
        df = pd.DataFrame(
            {
                "actual_pos_shoulder": [1],
                "actual_pos_elbow": [2],
            }
        )
        assert _detect_num_channels(df, "actual_pos_") == 2

    def test_no_match(self):
        df = pd.DataFrame({"x": [1], "y": [2]})
        assert _detect_num_channels(df, "actual_pos_") == 0


# ═══════════════════════════════════════════════════════════════════════════
# _detect_joint_columns — 조인트 컬럼 감지 + 캐싱
# ═══════════════════════════════════════════════════════════════════════════


class TestDetectJointColumns:
    def test_numeric_columns(self):
        df = pd.DataFrame(
            {
                "goal_pos_0": [0.1],
                "goal_pos_1": [0.2],
                "goal_pos_2": [0.3],
            }
        )
        cols, names = _detect_joint_columns(df, "goal_pos_", 3)
        assert cols == ["goal_pos_0", "goal_pos_1", "goal_pos_2"]
        assert names == ["0", "1", "2"]

    def test_named_columns(self):
        df = pd.DataFrame(
            {
                "actual_pos_shoulder_pan_joint": [0.1],
                "actual_pos_elbow_joint": [0.2],
            }
        )
        cols, names = _detect_joint_columns(df, "actual_pos_")
        assert len(cols) == 2
        assert "shoulder_pan_joint" in names[0]

    def test_auto_count(self):
        df = pd.DataFrame(
            {
                "cmd_0": [0.1],
                "cmd_1": [0.2],
                "cmd_2": [0.3],
            }
        )
        cols, names = _detect_joint_columns(df, "cmd_")
        assert len(cols) == 3

    def test_empty_result(self):
        df = pd.DataFrame({"x": [1]})
        cols, names = _detect_joint_columns(df, "nonexistent_")
        assert cols == []
        assert names == []

    def test_cache_hit(self):
        """동일 prefix + count 재호출 시 캐시 사용."""
        df = pd.DataFrame({"p_0": [1], "p_1": [2]})
        cols1, _ = _detect_joint_columns(df, "p_", 2)
        cols2, _ = _detect_joint_columns(df, "p_", 2)
        assert cols1 is cols2  # same object from cache


# ═══════════════════════════════════════════════════════════════════════════
# _has_columns — 컬럼 존재 여부 확인
# ═══════════════════════════════════════════════════════════════════════════


class TestHasColumns:
    def test_numeric_present(self):
        df = pd.DataFrame({"task_pos_0": [1], "task_pos_1": [2], "task_pos_2": [3]})
        assert _has_columns(df, "task_pos_", 3) is True
        assert _has_columns(df, "task_pos_", 4) is False

    def test_named_present(self):
        df = pd.DataFrame(
            {
                "task_pos_x": [1],
                "task_pos_y": [2],
                "task_pos_z": [3],
            }
        )
        assert _has_columns(df, "task_pos_", 3) is True

    def test_missing(self):
        df = pd.DataFrame({"x": [1]})
        assert _has_columns(df, "task_pos_", 1) is False


# ═══════════════════════════════════════════════════════════════════════════
# _detect_fingertip_labels / _detect_fingertip_labels_raw / _detect_ft_labels
# ═══════════════════════════════════════════════════════════════════════════


class TestDetectFingertipLabels:
    def test_filtered_labels(self):
        cols = {
            "baro_thumb_0": [1],
            "baro_thumb_1": [2],
            "baro_index_0": [3],
            "baro_index_1": [4],
        }
        df = pd.DataFrame(cols)
        labels = _detect_fingertip_labels(df)
        assert set(labels) == {"thumb", "index"}

    def test_excludes_raw(self):
        """baro_raw_ 컬럼은 filtered 감지에서 제외."""
        cols = {
            "baro_raw_thumb_0": [1],
            "baro_raw_thumb_1": [2],
            "baro_thumb_0": [3],
            "baro_thumb_1": [4],
        }
        df = pd.DataFrame(cols)
        labels = _detect_fingertip_labels(df)
        assert labels == ["thumb"]

    def test_no_labels(self):
        df = pd.DataFrame({"x": [1]})
        assert _detect_fingertip_labels(df) == []


class TestDetectFingertipLabelsRaw:
    def test_raw_labels(self):
        cols = {
            "baro_raw_thumb_0": [1],
            "baro_raw_thumb_1": [2],
            "baro_raw_index_0": [3],
            "baro_raw_index_1": [4],
        }
        df = pd.DataFrame(cols)
        labels = _detect_fingertip_labels_raw(df)
        assert set(labels) == {"thumb", "index"}

    def test_no_raw(self):
        df = pd.DataFrame({"baro_thumb_0": [1], "baro_thumb_1": [2]})
        labels = _detect_fingertip_labels_raw(df)
        assert labels == []


class TestDetectFtLabels:
    def test_contact_model(self):
        """3-head model: ft_{label}_contact + ft_{label}_fx."""
        cols = {
            "ft_thumb_contact": [0.8],
            "ft_thumb_fx": [1.0],
            "ft_index_contact": [0.3],
            "ft_index_fx": [0.5],
        }
        df = pd.DataFrame(cols)
        labels = _detect_ft_labels(df)
        assert set(labels) == {"thumb", "index"}

    def test_legacy_model(self):
        """Legacy 6-output model: ft_{label}_fx + ft_{label}_fy."""
        cols = {
            "ft_thumb_fx": [1.0],
            "ft_thumb_fy": [0.5],
            "ft_index_fx": [0.3],
            "ft_index_fy": [0.2],
        }
        df = pd.DataFrame(cols)
        labels = _detect_ft_labels(df)
        assert set(labels) == {"thumb", "index"}

    def test_excludes_ft_valid(self):
        """ft_valid 라벨은 제외."""
        cols = {
            "ft_valid_contact": [1],
            "ft_valid_fx": [0],
            "ft_thumb_contact": [0.8],
            "ft_thumb_fx": [1.0],
        }
        df = pd.DataFrame(cols)
        labels = _detect_ft_labels(df)
        assert labels == ["thumb"]

    def test_no_ft(self):
        df = pd.DataFrame({"x": [1]})
        assert _detect_ft_labels(df) == []


# ═══════════════════════════════════════════════════════════════════════════
# load_log_csv — header/data column-count contract
# ═══════════════════════════════════════════════════════════════════════════


class TestLoadLogCsv:
    """load_log_csv: header와 data 컬럼 수 일치를 강제. legacy 복구 미지원."""

    def test_normal_load(self, tmp_path):
        path = tmp_path / "hand_sensor_log.csv"
        _write_csv(path, ["timestamp", "baro_0", "tof_0"], [[0.0, 100, 50], [0.002, 101, 51]])
        df = load_log_csv(str(path), "sensor_log")
        assert len(df) == 2
        assert list(df.columns) == ["timestamp", "baro_0", "tof_0"]

    def test_state_log_passthrough(self, tmp_path):
        path = tmp_path / "arm_state_log.csv"
        _write_csv(
            path,
            ["timestamp", "actual_pos_0"],
            [[0.0, 1.0], [0.002, 1.01]],
        )
        df = load_log_csv(str(path), "state_log")
        assert len(df) == 2

    def test_legacy_truncated_header_raises(self, tmp_path):
        """헤더 컬럼 수보다 데이터 행이 더 넓으면 LegacyCsvError."""
        path = tmp_path / "hand_sensor_log.csv"
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "baro_raw_thumb_0"])
            writer.writerow([0.0, 100, 0.8, 1.0])

        with pytest.raises(LegacyCsvError):
            load_log_csv(str(path), "sensor_log")

    def test_empty_file_raises(self, tmp_path):
        path = tmp_path / "empty.csv"
        path.write_text("")

        with pytest.raises(pd.errors.EmptyDataError):
            load_log_csv(str(path), "sensor_log")

    def test_derives_timestamp_from_t_wall_ns(self, tmp_path):
        """Timing CSVs use t_wall_ns; loader normalizes to seconds-since-start."""
        path = tmp_path / "cm_timing_log.csv"
        _write_csv(
            path,
            ["t_wall_ns", "tick_count", "t_total_us", "jitter_us"],
            [
                [1_000_000_000, 1, 100, 5],
                [1_002_000_000, 2, 110, 4],
                [1_004_000_000, 3, 105, 6],
            ],
        )
        df = load_log_csv(str(path), "cm_timing")
        assert "timestamp" in df.columns
        assert df["timestamp"].iloc[0] == pytest.approx(0.0)
        assert df["timestamp"].iloc[1] == pytest.approx(0.002)
        assert df["timestamp"].iloc[2] == pytest.approx(0.004)

    def test_derives_timestamp_from_t_relative_s(self, tmp_path):
        """Phase C state/sensor CSVs use t_relative_s; loader aliases to timestamp."""
        path = tmp_path / "arm_state_log.csv"
        _write_csv(
            path,
            ["t_relative_s", "actual_pos_0"],
            [[0.0, 0.1], [0.002, 0.11], [0.004, 0.12]],
        )
        df = load_log_csv(str(path), "state_log")
        assert "timestamp" in df.columns
        assert df["timestamp"].iloc[0] == pytest.approx(0.0)
        assert df["timestamp"].iloc[2] == pytest.approx(0.004)

    def test_existing_timestamp_passthrough(self, tmp_path):
        """If timestamp already present (legacy fixtures), don't overwrite."""
        path = tmp_path / "hand_sensor_log.csv"
        _write_csv(path, ["timestamp", "baro_0"], [[10.0, 100], [10.002, 101]])
        df = load_log_csv(str(path), "sensor_log")
        assert df["timestamp"].iloc[0] == 10.0
        assert df["timestamp"].iloc[1] == pytest.approx(10.002)


# ═══════════════════════════════════════════════════════════════════════════
# 통계 함수 — print_robot_statistics / print_timing_statistics
# ═══════════════════════════════════════════════════════════════════════════


class TestStatistics:
    """통계 출력 함수가 예외 없이 실행되는지 검증 (출력 내용은 smoke test)."""

    def test_robot_statistics_no_crash(self, capsys):
        from rtc_tools.plotting.plotters.robot import print_robot_statistics

        df = pd.DataFrame(
            {
                "timestamp": [0.0, 0.002, 0.004, 0.006, 0.008],
                "actual_pos_0": [0.1, 0.11, 0.12, 0.13, 0.14],
                "actual_pos_1": [0.2, 0.21, 0.22, 0.23, 0.24],
                "traj_pos_0": [0.1, 0.12, 0.14, 0.16, 0.18],
                "traj_pos_1": [0.2, 0.22, 0.24, 0.26, 0.28],
                "actual_vel_0": [5.0, 5.0, 5.0, 5.0, 5.0],
                "actual_vel_1": [5.0, 5.0, 5.0, 5.0, 5.0],
                "traj_vel_0": [5.0, 5.0, 5.0, 5.0, 5.0],
                "traj_vel_1": [5.0, 5.0, 5.0, 5.0, 5.0],
            }
        )
        print_robot_statistics(df)
        captured = capsys.readouterr()
        assert "Robot Trajectory Statistics" in captured.out
        assert "RMS" in captured.out

    def test_robot_statistics_zero_duration(self, capsys):
        """duration=0 시 ZeroDivisionError 방지."""
        from rtc_tools.plotting.plotters.robot import print_robot_statistics

        df = pd.DataFrame(
            {
                "timestamp": [5.0, 5.0],
                "actual_pos_0": [0.1, 0.1],
            }
        )
        print_robot_statistics(df)  # no crash
        captured = capsys.readouterr()
        assert "0.0 Hz" in captured.out

    def test_timing_statistics_no_crash(self, capsys):
        from rtc_tools.plotting.plotters.timing import print_timing_statistics

        df = pd.DataFrame(
            {
                "timestamp": np.arange(0, 1.0, 0.002),
                "t_state_us": np.random.uniform(10, 50, 500),
                "t_compute_us": np.random.uniform(50, 200, 500),
                "t_publish_us": np.random.uniform(5, 30, 500),
                "t_total_us": np.random.uniform(100, 300, 500),
                "jitter_us": np.random.uniform(-50, 50, 500),
            }
        )
        print_timing_statistics(df)
        captured = capsys.readouterr()
        assert "Timing Statistics" in captured.out
        assert "P50" in captured.out
        assert "P99" in captured.out

    def test_device_statistics_no_crash(self, capsys):
        from rtc_tools.plotting.plotters.device import print_device_statistics

        df = pd.DataFrame(
            {
                "timestamp": [0.0, 0.002, 0.004],
                "hand_actual_pos_m0": [0.1, 0.2, 0.3],
                "hand_actual_pos_m1": [0.4, 0.5, 0.6],
                "hand_goal_pos_m0": [0.1, 0.2, 0.3],
                "hand_goal_pos_m1": [0.4, 0.5, 0.6],
                "baro_thumb_0": [100, 101, 102],
                "baro_thumb_1": [200, 201, 202],
                "tof_thumb_0": [50, 51, 52],
            }
        )
        print_device_statistics(df)
        captured = capsys.readouterr()
        assert "Hand Trajectory Statistics" in captured.out

    def test_motor_statistics_no_crash(self, capsys):
        from rtc_tools.plotting.plotters.motor import print_motor_statistics

        df = pd.DataFrame(
            {
                "timestamp": [0.0, 0.002, 0.004],
                "motor_pos_m0": [0.1, 0.2, 0.3],
                "motor_pos_m1": [0.4, 0.5, 0.6],
                "motor_vel_m0": [1.0, 1.1, 1.2],
                "motor_vel_m1": [2.0, 2.1, 2.2],
                "motor_eff_m0": [0.01, 0.02, 0.03],
                "motor_eff_m1": [0.04, 0.05, 0.06],
            }
        )
        print_motor_statistics(df)
        captured = capsys.readouterr()
        assert "Motor State Statistics" in captured.out


# ═══════════════════════════════════════════════════════════════════════════
# Timing outlier detection — first-tick init spike must not poison stats
# ═══════════════════════════════════════════════════════════════════════════


class TestTimingOutlierFilter:
    """`detect_outlier_indices` / `filter_outliers` / `print_timing_statistics`
    behaviour around the first-tick init spike (auto-hold + lazy RCLCPP_INFO)
    observed in CM sim mode. Raw CSV must remain untouched; only stats
    excludes the spike.
    """

    @staticmethod
    def _make_steady_df(n=500, total_us=30.0):
        # Steady-state: t_total ~= total_us with mild noise.
        rng = np.random.default_rng(42)
        return pd.DataFrame(
            {
                "timestamp": np.arange(n) * 0.002,
                "tick_count": np.arange(1, n + 1),
                "t_state_us": rng.uniform(0.5, 2.0, n),
                "t_compute_us": rng.uniform(15.0, 30.0, n),
                "t_publish_us": rng.uniform(3.0, 10.0, n),
                "t_total_us": rng.uniform(total_us - 5, total_us + 5, n),
                "jitter_us": np.zeros(n),
            }
        )

    def test_no_outliers_on_clean_steady_state(self):
        from rtc_tools.plotting.plotters.timing import detect_outlier_indices

        df = self._make_steady_df()
        idx = detect_outlier_indices(df)
        assert len(idx) == 0

    def test_first_tick_spike_is_flagged(self):
        from rtc_tools.plotting.plotters.timing import (
            detect_outlier_indices,
            filter_outliers,
        )

        df = self._make_steady_df()
        # Inject the observed sim-mode signature: tick 2 t_total = 340 ms.
        df.loc[1, "t_total_us"] = 340_469.0
        df.loc[1, "t_state_us"] = 340_335.0

        idx = detect_outlier_indices(df)
        assert list(idx) == [1], f"expected only row 1 flagged, got {list(idx)}"

        clean, outliers = filter_outliers(df)
        assert len(clean) == len(df) - 1
        assert len(outliers) == 1
        # Cleaned p99 must reflect steady-state (~35 µs), not the 340 ms spike.
        assert clean["t_total_us"].quantile(0.99) < 100.0

    def test_mid_run_spike_also_flagged(self):
        from rtc_tools.plotting.plotters.timing import detect_outlier_indices

        df = self._make_steady_df()
        # Anomaly far past the first_ticks window.
        df.loc[300, "t_total_us"] = 50_000.0

        idx = detect_outlier_indices(df)
        assert 300 in list(idx)

    def test_filter_does_not_mutate_input(self):
        from rtc_tools.plotting.plotters.timing import filter_outliers

        df = self._make_steady_df()
        df.loc[0, "t_total_us"] = 200_000.0
        before = df.copy()
        filter_outliers(df)
        # filter_outliers must return new frames; raw stays intact.
        pd.testing.assert_frame_equal(df, before)

    def test_print_statistics_lists_outlier_and_excludes_from_stats(self, capsys):
        from rtc_tools.plotting.plotters.timing import print_timing_statistics

        df = self._make_steady_df()
        df.loc[1, "t_total_us"] = 340_469.0
        df.loc[1, "t_state_us"] = 340_335.0
        df.loc[1, "t_compute_us"] = 121.2
        df.loc[1, "t_publish_us"] = 12.6

        print_timing_statistics(df)
        out = capsys.readouterr().out

        assert "Outliers excluded from stats: 1" in out
        # The outlier row's tick + t_total must appear in the listing.
        assert "tick=" in out
        assert "340469" in out.replace(" ", "") or "340469.0" in out
        # Steady-state Total Loop p99 must NOT carry the spike.
        # Sloppy parsing: just ensure "340" doesn't appear in the Total Loop
        # block's numeric stats (it's only in the outlier list above).
        total_block = out.split("Total Loop:")[1].split("\n\n")[0]
        assert "340" not in total_block, (
            f"steady-state stats appear contaminated by outlier:\n{total_block}"
        )

    def test_empty_df_returns_no_outliers(self):
        from rtc_tools.plotting.plotters.timing import detect_outlier_indices

        df = pd.DataFrame(columns=["timestamp", "tick_count", "t_total_us"])
        idx = detect_outlier_indices(df)
        assert len(idx) == 0


class TestInferBudget:
    """`infer_budget_us` must use median(diff) so a first-tick gap (340 ms
    in observed sim mode) does not poison the inferred period budget that
    `plot_timing_*` and `print_timing_statistics` use for the budget line
    and overrun count.
    """

    def test_steady_state_returns_period(self):
        from rtc_tools.plotting.plotters.timing import infer_budget_us

        # 500 Hz steady cadence -> 2000 µs.
        df = pd.DataFrame({"timestamp": np.arange(500) * 0.002})
        assert abs(infer_budget_us(df) - 2000.0) < 1e-6

    def test_first_tick_gap_is_ignored(self):
        from rtc_tools.plotting.plotters.timing import infer_budget_us

        # tick 0 → tick 1 = 340 ms (sim init), tick 1 → onwards = 2 ms.
        timestamps = [0.0, 0.340488] + [0.340488 + i * 0.002 for i in range(1, 500)]
        df = pd.DataFrame({"timestamp": timestamps})
        budget = infer_budget_us(df)
        # Must be ~2000 µs (steady cadence), not 340 488 µs.
        assert 1900.0 < budget < 2100.0, f"budget contaminated by first-tick gap: {budget}"

    def test_fallback_when_too_few_rows(self):
        from rtc_tools.plotting.plotters.timing import infer_budget_us

        assert infer_budget_us(pd.DataFrame({"timestamp": []})) == 2000.0
        assert infer_budget_us(pd.DataFrame({"timestamp": [0.0]})) == 2000.0
        assert infer_budget_us(pd.DataFrame()) == 2000.0

    def test_fallback_on_non_monotonic(self):
        from rtc_tools.plotting.plotters.timing import infer_budget_us

        # All-equal timestamps → no positive diffs → fallback.
        df = pd.DataFrame({"timestamp": [1.0, 1.0, 1.0]})
        assert infer_budget_us(df, fallback_us=1234.0) == 1234.0


# ═══════════════════════════════════════════════════════════════════════════
# Writer-schema round-trip — build the exact CSV header the C++ POD writers
# emit (DeviceStateLog / DeviceSensorLog), load it through load_log_csv, then
# drive the plotters. This is the coverage gap that let the column-mapping
# bugs (command_type as a joint, task_pos_{i} KeyError, baro_/tof_ vs
# <name>_raw_/_filt_) ship undetected: the prior fixtures used synthetic
# column names matching neither writer.
# ═══════════════════════════════════════════════════════════════════════════

import matplotlib  # noqa: E402

matplotlib.use("Agg")  # headless: must precede the pyplot import in the plotters

from rtc_tools.plotting.columns.detect import (  # noqa: E402
    baro_col as _baro_col,
    tof_col as _tof_col,
)
from rtc_tools.plotting.columns.views import (  # noqa: E402
    has_fingertip_sensors as _has_fingertip_sensors,
    has_raw_sensors as _has_raw_sensors,
)
from rtc_tools.plotting.plotters.robot import (  # noqa: E402
    command_type_label as _command_type_label,
    plot_robot_commands as _plot_robot_commands,
    plot_robot_task_position as _plot_robot_task_position,
    plot_robot_task_tracking_error as _plot_robot_task_tracking_error,
    print_robot_statistics as _print_robot_statistics,
)
from rtc_tools.plotting.plotters.sensors import (  # noqa: E402
    plot_device_ft_output_auto as _plot_device_ft_output_auto,
    plot_device_sensor_comparison_auto as _plot_device_sensor_comparison_auto,
    plot_sensor_barometer_combined as _plot_sensor_barometer_combined,
    plot_sensor_tof_combined as _plot_sensor_tof_combined,
)

_TASK_AXES = ("x", "y", "z", "roll", "pitch", "yaw")


def _state_log_header(joints, motors):
    """Column order emitted by WriteDeviceStateLogHeader (device_state_log_pod.hpp)."""
    cols = ["t_relative_s"]
    for field in (
        "actual_pos",
        "actual_vel",
        "effort",
        "command",
        "joint_goal",
        "traj_pos",
        "traj_vel",
    ):
        cols += [f"{field}_{j}" for j in joints]
    cols += [f"task_goal_{a}" for a in _TASK_AXES]
    cols += [f"task_pos_{a}" for a in _TASK_AXES]
    for field in ("motor_pos", "motor_vel", "motor_eff"):
        cols += [f"{field}_{m}" for m in motors]
    cols += ["command_type", "goal_type"]
    return cols


def _sensor_log_header(fingertips, n_vals=11):
    """Column order emitted by WriteDeviceSensorLogHeader (device_sensor_log_pod.hpp).

    n_vals == kSensorValuesPerFingertip (11 = 8 baro + 3 ToF).
    """
    cols = ["t_relative_s"]
    for kind in ("raw", "filt"):
        for name in fingertips:
            cols += [f"{name}_{kind}_{v}" for v in range(n_vals)]
    cols += ["inference_valid"]
    for name in fingertips:
        cols += [f"ft_{name}_{c}" for c in ("contact", "fx", "fy", "fz", "ux", "uy", "uz")]
    return cols


def _build_log(tmp_path, name, header, str_vals, n=3):
    """Write `n` numeric rows (string columns from `str_vals`) and load via the loader."""
    rows = []
    for i in range(n):
        row = [str_vals.get(c, float(i)) for c in header]
        row[0] = i * 0.002  # t_relative_s
        rows.append(row)
    path = tmp_path / name
    _write_csv(str(path), header, rows)
    log_type = "sensor_log" if "sensor" in name else "state_log"
    return load_log_csv(str(path), log_type)


class TestStateLogRoundTrip:
    JOINTS = ["a1", "a2"]
    MOTORS = ["m1"]

    def _df(self, tmp_path, command_type="position", goal_type="task"):
        header = _state_log_header(self.JOINTS, self.MOTORS)
        return _build_log(
            tmp_path,
            "arm_state_log.csv",
            header,
            {"command_type": command_type, "goal_type": goal_type},
        )

    def test_command_type_not_detected_as_joint(self, tmp_path):
        """Bug #1: command_type must not be counted as a per-joint channel."""
        df = self._df(tmp_path)
        cols, _ = _detect_joint_columns(df, "command_")
        assert cols == ["command_a1", "command_a2"]  # NOT a 3rd 'command_type'

    def test_command_type_label_string_and_legacy(self, tmp_path):
        """Bug #2: command_type is a string; legacy int 0/1 still accepted."""
        assert _command_type_label(self._df(tmp_path, command_type="position")) == (
            "Position",
            "rad",
        )
        assert _command_type_label(self._df(tmp_path, command_type="torque")) == ("Torque", "Nm")
        df_int = self._df(tmp_path)
        df_int["command_type"] = 0
        assert _command_type_label(df_int) == ("Position", "rad")

    def test_command_plot_renders(self, tmp_path):
        _plot_robot_commands(self._df(tmp_path), save_dir=str(tmp_path))
        assert (tmp_path / "robot_commands.png").exists()

    def test_statistics_command_type_summary(self, tmp_path, capsys):
        """Console summary must read the string command_type (not legacy ==0)."""
        _print_robot_statistics(self._df(tmp_path, command_type="position"))
        assert "Command type: position" in capsys.readouterr().out
        _print_robot_statistics(self._df(tmp_path, command_type="torque"))
        assert "Command type: torque" in capsys.readouterr().out
        df_int = self._df(tmp_path)
        df_int["command_type"] = 0
        _print_robot_statistics(df_int)
        assert "Command type: position" in capsys.readouterr().out

    def test_task_plots_render_named_axes(self, tmp_path):
        """Bug #3/#4: named task_pos_x.. axes, no traj_task_pos_* KeyError."""
        df = self._df(tmp_path)
        _plot_robot_task_position(df, save_dir=str(tmp_path))
        _plot_robot_task_tracking_error(df, save_dir=str(tmp_path))
        assert (tmp_path / "robot_task_position.png").exists()
        assert (tmp_path / "robot_task_tracking_error.png").exists()


class TestSensorLogRoundTrip:
    FINGERTIPS = ["thumb", "index"]

    def _df(self, tmp_path):
        header = _sensor_log_header(self.FINGERTIPS)
        return _build_log(tmp_path, "hand_sensor_log.csv", header, {})

    def test_fingertip_detection(self, tmp_path):
        df = self._df(tmp_path)
        assert _detect_fingertip_labels(df) == self.FINGERTIPS
        assert _detect_fingertip_labels_raw(df) == self.FINGERTIPS
        assert _detect_ft_labels(df) == self.FINGERTIPS

    def test_baro_tof_resolver_split(self, tmp_path):
        """Bug #5: <name>_raw_/_filt_ block, values 0..7 baro / 8..10 ToF."""
        df = self._df(tmp_path)
        assert _baro_col(df, "thumb", 0) == "thumb_filt_0"
        assert _baro_col(df, "thumb", 7) == "thumb_filt_7"
        assert _baro_col(df, "thumb", 8) is None  # 8..10 are ToF, not baro
        assert _tof_col(df, "thumb", 0) == "thumb_filt_8"
        assert _tof_col(df, "thumb", 2) == "thumb_filt_10"
        assert _tof_col(df, "thumb", 3) is None
        assert _baro_col(df, "index", 0, raw=True) == "index_raw_0"
        assert _tof_col(df, "index", 2, raw=True) == "index_raw_10"

    def test_sensor_plots_render(self, tmp_path):
        df = self._df(tmp_path)
        _plot_sensor_barometer_combined(df, save_dir=str(tmp_path))
        _plot_sensor_tof_combined(df, save_dir=str(tmp_path))
        _plot_device_sensor_comparison_auto(df, save_dir=str(tmp_path))
        _plot_device_ft_output_auto(df, save_dir=str(tmp_path))
        assert (tmp_path / "sensor_barometer.png").exists()
        assert (tmp_path / "sensor_tof.png").exists()
        assert (tmp_path / "device_sensor_comparison.png").exists()
        assert (tmp_path / "device_ft_output.png").exists()

    def test_view_predicates_schema_aware(self, tmp_path):
        """has_fingertip_sensors / has_raw_sensors must fire on the Phase C
        `<name>_raw_/_filt_` schema (not just the legacy baro_/tof_ columns)."""
        df = self._df(tmp_path)
        assert _has_fingertip_sensors(df) is True
        assert _has_raw_sensors(df) is True
        # legacy schema still recognised
        legacy = pd.DataFrame(
            {
                "baro_thumb_0": [1],
                "baro_thumb_1": [2],
                "baro_raw_thumb_0": [1],
                "baro_raw_thumb_1": [2],
            }
        )
        assert _has_fingertip_sensors(legacy) is True
        assert _has_raw_sensors(legacy) is True
        # state-log columns must not look like fingertip sensors
        assert _has_fingertip_sensors(pd.DataFrame({"actual_pos_a1": [0.0]})) is False
        assert _has_raw_sensors(pd.DataFrame({"actual_pos_a1": [0.0]})) is False


# ═══════════════════════════════════════════════════════════════════════════
# WBC writer-schema round-trip — DeviceWbcLog (arm/hand) + WbcDiagLog. Build
# the exact CSV headers the C++ POD writers emit (device_wbc_log_pod.hpp /
# wbc_diag_log_pod.hpp), load through load_log_csv, then detect + plot.
# ═══════════════════════════════════════════════════════════════════════════

from rtc_tools.plotting.columns.views import (  # noqa: E402
    has_wbc_accel as _has_wbc_accel,
    has_wbc_fingertip_force as _has_wbc_fingertip_force,
    has_wbc_lambda as _has_wbc_lambda,
    has_wbc_task_traj as _has_wbc_task_traj,
)
from rtc_tools.plotting.plotters.wbc import (  # noqa: E402
    plot_wbc_accelerations as _plot_wbc_accelerations,
    plot_wbc_diag_contacts as _plot_wbc_diag_contacts,
    plot_wbc_diag_solver as _plot_wbc_diag_solver,
    plot_wbc_fingertip_force as _plot_wbc_fingertip_force,
    plot_wbc_task_trajectory as _plot_wbc_task_trajectory,
    print_wbc_diag_statistics as _print_wbc_diag_statistics,
)

_WBC_JOINT_FIELDS = (
    "actual_pos",
    "actual_vel",
    "effort",
    "accel",
    "command",
    "joint_goal",
    "traj_pos",
    "traj_vel",
)


def _wbc_arm_header(joints):
    """Column order of WriteDeviceWbcLogHeader(role=0) (device_wbc_log_pod.hpp)."""
    cols = ["t_relative_s"]
    for field in _WBC_JOINT_FIELDS:
        cols += [f"{field}_{j}" for j in joints]
    for block in ("task_goal", "task_pos", "traj_task_pos", "traj_task_vel"):
        cols += [f"{block}_{a}" for a in _TASK_AXES]
    cols += ["command_type", "goal_type"]
    return cols


def _wbc_hand_header(joints, motors, fingertips, max_fingertips=8):
    """Column order of WriteDeviceWbcLogHeader(role=1).

    The fingertip-force blocks are FIXED-WIDTH (kMaxFingertips=8): labelled by
    `fingertips` where provided, numeric index otherwise. This mirrors the C++
    writer, which can't size the block from the runtime fingertip count. The
    |F| magnitude block is followed by the Fx/Fy/Fz vector blocks.
    """
    cols = ["t_relative_s"]
    for field in _WBC_JOINT_FIELDS:
        cols += [f"{field}_{j}" for j in joints]
    for field in ("motor_pos", "motor_vel", "motor_eff"):
        cols += [f"{field}_{m}" for m in motors]
    for prefix in ("fingertip_force_", "fingertip_fx_", "fingertip_fy_", "fingertip_fz_"):
        for i in range(max_fingertips):
            label = fingertips[i] if i < len(fingertips) else str(i)
            cols.append(f"{prefix}{label}")
    cols += ["command_type", "goal_type"]
    return cols


def _wbc_diag_header(n_lambda):
    """Column order of WriteWbcDiagLogHeader (wbc_diag_log_pod.hpp)."""
    cols = [
        "t_relative_s",
        "phase",
        "solve_time_us",
        "qp_converged",
        "solve_levels",
        "qp_fail_count",
        "num_active_contacts",
        "grasp_detected",
        "max_force",
    ]
    cols += [f"lambda_{i}" for i in range(n_lambda)]
    return cols


def _build_wbc_log(tmp_path, name, header, str_vals, log_type, n=3):
    rows = []
    for i in range(n):
        row = [str_vals.get(c, float(i)) for c in header]
        row[0] = i * 0.002  # t_relative_s
        rows.append(row)
    path = tmp_path / name
    _write_csv(str(path), header, rows)
    return load_log_csv(str(path), log_type)


class TestWbcLogRoundTrip:
    JOINTS = ["a1", "a2"]
    MOTORS = ["m1", "m2"]
    FINGERTIPS = ["thumb", "index"]

    def _arm_df(self, tmp_path):
        header = _wbc_arm_header(self.JOINTS)
        return _build_wbc_log(
            tmp_path,
            "iiwa7_state.csv",
            header,
            {"command_type": "position", "goal_type": "task"},
            "wbc_log",
        )

    def _hand_df(self, tmp_path):
        header = _wbc_hand_header(self.JOINTS, self.MOTORS, self.FINGERTIPS)
        return _build_wbc_log(
            tmp_path,
            "leap_state.csv",
            header,
            {"command_type": "position", "goal_type": "joint"},
            "wbc_log",
        )

    def test_arm_columns_detect_as_wbc_log(self, tmp_path):
        header = _wbc_arm_header(self.JOINTS)
        assert detect_log_type_by_columns(header) == "wbc_log"

    def test_hand_columns_detect_as_wbc_log(self, tmp_path):
        header = _wbc_hand_header(self.JOINTS, self.MOTORS, self.FINGERTIPS)
        assert detect_log_type_by_columns(header) == "wbc_log"

    def test_accel_not_detected_as_joint_artifact(self, tmp_path):
        df = self._arm_df(tmp_path)
        cols, _ = _detect_joint_columns(df, "accel_")
        assert cols == ["accel_a1", "accel_a2"]

    def test_arm_predicates(self, tmp_path):
        df = self._arm_df(tmp_path)
        assert _has_wbc_accel(df) is True
        assert _has_wbc_task_traj(df) is True
        assert _has_wbc_fingertip_force(df) is False

    def test_hand_predicates(self, tmp_path):
        df = self._hand_df(tmp_path)
        assert _has_wbc_accel(df) is True
        assert _has_wbc_fingertip_force(df) is True
        assert _has_wbc_task_traj(df) is False

    def test_arm_plots_render(self, tmp_path):
        df = self._arm_df(tmp_path)
        _plot_wbc_accelerations(df, save_dir=str(tmp_path))
        _plot_wbc_task_trajectory(df, save_dir=str(tmp_path))
        assert (tmp_path / "wbc_accelerations.png").exists()
        assert (tmp_path / "wbc_task_trajectory.png").exists()

    def test_hand_plots_render(self, tmp_path):
        df = self._hand_df(tmp_path)
        _plot_wbc_fingertip_force(df, save_dir=str(tmp_path))
        assert (tmp_path / "wbc_fingertip_force.png").exists()

    def test_hand_fingertip_force_legacy_magnitude_only(self, tmp_path):
        """Pre-vector CSVs (only fingertip_force_* |F|) fall back to the
        single overlaid-lines figure instead of the per-finger subfigures."""
        header = _wbc_hand_header(self.JOINTS, self.MOTORS, self.FINGERTIPS)
        header = [
            c
            for c in header
            if not c.startswith(("fingertip_fx_", "fingertip_fy_", "fingertip_fz_"))
        ]
        df = _build_wbc_log(
            tmp_path,
            "hand_state_legacy.csv",
            header,
            {"command_type": "position", "goal_type": "joint"},
            "wbc_log",
        )
        assert _has_wbc_fingertip_force(df) is True
        _plot_wbc_fingertip_force(df, save_dir=str(tmp_path))
        assert (tmp_path / "wbc_fingertip_force.png").exists()

    def test_reused_robot_plotters_render(self, tmp_path):
        """DeviceWbcLog is a state_log superset → robot plotters still work."""
        from rtc_tools.plotting.plotters.robot import (
            plot_robot_positions as _plot_robot_positions,
        )

        _plot_robot_positions(self._arm_df(tmp_path), save_dir=str(tmp_path))
        assert (tmp_path / "robot_positions.png").exists()


class TestWbcDiagRoundTrip:
    N_LAMBDA = 6

    def _df(self, tmp_path):
        header = _wbc_diag_header(self.N_LAMBDA)
        return _build_wbc_log(
            tmp_path,
            "wbc_diag.csv",
            header,
            {"phase": "closure", "qp_converged": 1, "grasp_detected": 1},
            "wbc_diag",
        )

    def test_filename_detection(self):
        assert detect_log_type("/s/controllers/demo_wbc/wbc_diag.csv") == "wbc_diag"

    def test_column_detection(self):
        assert detect_log_type_by_columns(_wbc_diag_header(self.N_LAMBDA)) == "wbc_diag"

    def test_lambda_predicate(self, tmp_path):
        assert _has_wbc_lambda(self._df(tmp_path)) is True

    def test_phase_stays_string(self, tmp_path):
        """phase is categorical — loader must not coerce it to NaN."""
        df = self._df(tmp_path)
        assert df["phase"].iloc[0] == "closure"

    def test_diag_plots_render(self, tmp_path):
        df = self._df(tmp_path)
        _plot_wbc_diag_solver(df, save_dir=str(tmp_path))
        _plot_wbc_diag_contacts(df, save_dir=str(tmp_path))
        assert (tmp_path / "wbc_diag_solver.png").exists()
        assert (tmp_path / "wbc_diag_contacts.png").exists()

    def test_diag_statistics(self, tmp_path, capsys):
        _print_wbc_diag_statistics(self._df(tmp_path))
        out = capsys.readouterr().out
        assert "WBC TSID/QP Diagnostics" in out
        assert "closure" in out


# ═══════════════════════════════════════════════════════════════════════════
# PullEstimatorLog (#167) — detection / predicate / plot / stats round-trip
# ═══════════════════════════════════════════════════════════════════════════

from rtc_tools.plotting.plotters.pull import (  # noqa: E402
    plot_pull_estimator as _plot_pull_estimator,
    print_pull_estimator_statistics as _print_pull_estimator_statistics,
)


def _pull_estimator_header(mask_roles=("thumb", "index", "middle")):
    """Column order emitted by WritePullEstimatorLogHeader (pull_estimator_log_pod.hpp).

    `mask_roles` reproduces the bit-order stamp the C++ header writer appends to
    the mask column names (#234 P-14); pass () for the unstamped form.
    """
    suffix = "[" + "|".join(mask_roles) + "]" if mask_roles else ""
    return [
        "t_relative_s",
        "tick",
        "force_prefilter_x",
        "force_prefilter_y",
        "force_prefilter_z",
        "force_x",
        "force_y",
        "force_z",
        "inplane_x",
        "inplane_y",
        "plane_normal_x",
        "plane_normal_y",
        "plane_normal_z",
        "basis_x_x",
        "basis_x_y",
        "basis_x_z",
        "basis_source",
        "invalid_reason",
        "magnitude",
        "directional",
        "friction_utilization",
        "leakage_bound",
        "valid_contact_count",
        "valid",
        "slip_risk",
        "any_saturated",
        "baseline_applied",
        f"contact_mask{suffix}",
        f"touch_mask{suffix}",
        f"opposing_mask{suffix}",
    ]


def _legacy_pull_estimator_header():
    """The pre-#234 schema, still readable so archived sessions keep plotting."""
    return [
        "t_relative_s",
        "force_raw_x",
        "force_raw_y",
        "force_raw_z",
        "force_x",
        "force_y",
        "force_z",
        "inplane_x",
        "inplane_y",
        "magnitude",
        "directional",
        "friction_utilization",
        "leakage_bound",
        "valid_contact_count",
        "valid",
        "slip_risk",
        "any_saturated",
        "baseline_applied",
        "opposing_mask",
    ]


class TestPullEstimatorRoundTrip:
    def _df(self, tmp_path, header=None):
        return _build_wbc_log(
            tmp_path,
            "pull_estimator.csv",
            header or _pull_estimator_header(),
            {},
            "pull_estimator",
        )

    def test_filename_detection(self):
        assert (
            detect_log_type("/s/controllers/demo_task_controller/pull_estimator.csv")
            == "pull_estimator"
        )

    def test_column_detection(self):
        assert detect_log_type_by_columns(_pull_estimator_header()) == "pull_estimator"

    def test_legacy_column_detection(self):
        assert detect_log_type_by_columns(_legacy_pull_estimator_header()) == "pull_estimator"

    def test_column_detection_beats_sensor_log_raw_token(self):
        """force_raw_* contains the generic `_raw_` sensor token — the pull
        branch must win over the sensor_log fallback."""
        cols = ["t_relative_s", "force_raw_x", "friction_utilization"]
        assert detect_log_type_by_columns(cols) == "pull_estimator"

    def test_mask_role_stamp_is_stripped_into_attrs(self, tmp_path):
        """Plotters key on a bare `contact_mask`; the bit order lands in attrs."""
        df = self._df(tmp_path)
        assert "contact_mask" in df.columns
        assert not any("[" in c for c in df.columns)
        assert df.attrs["mask_roles"]["contact_mask"] == ["thumb", "index", "middle"]
        assert df.attrs["mask_roles"]["touch_mask"] == ["thumb", "index", "middle"]

    def test_eight_contact_role_stamp(self, tmp_path):
        roles = tuple(f"c{i}" for i in range(8))
        df = self._df(tmp_path, header=_pull_estimator_header(roles))
        assert df.attrs["mask_roles"]["opposing_mask"] == list(roles)

    def test_unstamped_masks_still_load(self, tmp_path):
        df = self._df(tmp_path, header=_pull_estimator_header(()))
        assert "contact_mask" in df.columns
        assert df.attrs["mask_roles"] == {}

    def test_plot_renders(self, tmp_path):
        _plot_pull_estimator(self._df(tmp_path), save_dir=str(tmp_path))
        assert (tmp_path / "pull_estimator.png").exists()

    def test_plot_renders_legacy_schema(self, tmp_path):
        """A pre-#234 CSV must still plot — the new panels just stay empty."""
        df = self._df(tmp_path, header=_legacy_pull_estimator_header())
        _plot_pull_estimator(df, save_dir=str(tmp_path))
        assert (tmp_path / "pull_estimator.png").exists()

    def test_statistics(self, tmp_path, capsys):
        _print_pull_estimator_statistics(self._df(tmp_path))
        out = capsys.readouterr().out
        assert "Pull Force Estimator" in out
        assert "Friction utilization" in out
        assert "Contact set" in out

    def test_statistics_legacy_schema(self, tmp_path, capsys):
        _print_pull_estimator_statistics(self._df(tmp_path, _legacy_pull_estimator_header()))
        out = capsys.readouterr().out
        assert "Pull Force Estimator" in out

    def test_statistics_names_mask_bits_by_role(self, tmp_path, capsys):
        """With the role stamp present the dominant mask reads as finger names,
        not as a bare integer nobody can decode (#234 P-14)."""
        import pandas as pd

        df = self._df(tmp_path)
        df["opposing_mask"] = 0b110  # index + middle
        df.attrs["mask_roles"] = {"opposing_mask": ["thumb", "index", "middle"]}
        assert isinstance(df, pd.DataFrame)
        _print_pull_estimator_statistics(df)
        out = capsys.readouterr().out
        assert "index+middle" in out


# ═══════════════════════════════════════════════════════════════════════════
# Force-only fingertip view (hand with no barometer/ToF lane)
# ═══════════════════════════════════════════════════════════════════════════

from rtc_tools.plotting.columns import (  # noqa: E402
    ft_force_col as _ft_force_col,
    ft_force_guard_rejected_col as _ft_guard_rejected_col,
    ft_force_guarded_col as _ft_force_guarded_col,
    has_force_only_fingertips as _has_force_only_fingertips,
    has_ft_force_guard as _has_ft_force_guard,
)
from rtc_tools.plotting.plotters.sensors import (  # noqa: E402
    plot_fingertip_force_only_auto as _plot_fingertip_force_only_auto,
)


def _force_only_df(labels=("thumb", "index", "middle", "ring"), n=40, filtered=True, guard=False):
    """CSV frame for a hand whose sensor_layout.values_per_group is 0.

    No `<label>_raw_*` / `<label>_filt_*` block at all — that absence is what
    the force-only gate keys on.
    """
    t = np.linspace(0.0, 1.0, n)
    data = {"timestamp": t, "inference_valid": np.ones(n, dtype=int)}
    for k, label in enumerate(labels):
        data[f"ft_{label}_contact"] = np.zeros(n)
        for j, axis in enumerate(("fx", "fy", "fz")):
            data[f"ft_{label}_{axis}"] = np.sin(t * (k + 1)) + 0.1 * j
            if filtered:
                data[f"ft_{label}_{axis}_filt"] = np.sin(t * (k + 1)) + 0.1 * j
        for axis in ("ux", "uy", "uz"):
            data[f"ft_{label}_{axis}"] = np.zeros(n)
    if guard:
        # A 2-tick spike on fz, held out of the filter input — the 260730_1017
        # p1b shape. Raw keeps the spike; guarded holds the pre-spike value.
        for label in labels:
            rejected = np.zeros(n, dtype=int)
            rejected[5:7] = 1
            data[f"ft_{label}_force_guard_rejected"] = rejected
            for axis in ("fx", "fy", "fz"):
                guarded = data[f"ft_{label}_{axis}"].copy()
                guarded[5:7] = guarded[4]
                data[f"ft_{label}_{axis}_guarded"] = guarded
            raw_fz = data[f"ft_{label}_fz"].copy()
            raw_fz[5:7] = 6.4
            data[f"ft_{label}_fz"] = raw_fz
    return pd.DataFrame(data)


class TestForceOnlyDetection:
    def test_force_only_frame_is_detected(self):
        assert _has_force_only_fingertips(_force_only_df())

    def test_frame_with_baro_block_is_not_force_only(self):
        """A hand that DOES have the sensor lane keeps the barometer figures."""
        df = _force_only_df()
        for v in range(11):
            df[f"thumb_filt_{v}"] = np.zeros(len(df))
        assert not _has_force_only_fingertips(df)

    def test_frame_without_inference_is_not_force_only(self):
        assert not _has_force_only_fingertips(pd.DataFrame({"timestamp": [0.0, 1.0]}))

    def test_filtered_column_resolves_separately_from_raw(self):
        df = _force_only_df()
        assert _ft_force_col(df, "thumb", "fx") == "ft_thumb_fx"
        assert _ft_force_col(df, "thumb", "fx", filt=True) == "ft_thumb_fx_filt"
        assert _ft_force_col(df, "thumb", "tx") is None

    def test_filtered_columns_do_not_register_as_extra_ft_labels(self):
        """`ft_thumb_fx_filt` must not be mistaken for a fingertip named
        `thumb_fx` — the raw-force detector matches on the trailing token."""
        assert _detect_ft_labels(_force_only_df(labels=("thumb",))) == ["thumb"]


def _collection_labels(ax):
    """Labels of an axis's collections — `vlines` draws a LineCollection, which
    does not show up in `get_lines()`."""
    return [c.get_label() for c in ax.collections]


class TestForceGuardColumns:
    """The guard lane is what makes a session's hold decisions reconstructible.

    Raw and filtered alone cannot separate "the guard held this tick" from "the
    filter merely lagged", so these columns are the observability contract for
    the 4 N delta-spike guard.
    """

    def test_guard_lane_detected_only_when_present(self):
        assert _has_ft_force_guard(_force_only_df(guard=True))
        assert not _has_ft_force_guard(_force_only_df())

    def test_guarded_and_verdict_columns_resolve(self):
        df = _force_only_df(guard=True)
        assert _ft_force_guarded_col(df, "thumb", "fz") == "ft_thumb_fz_guarded"
        assert _ft_guard_rejected_col(df, "thumb") == "ft_thumb_force_guard_rejected"
        assert _ft_force_guarded_col(df, "thumb", "tz") is None

    def test_guard_columns_absent_on_pre_guard_sessions(self):
        df = _force_only_df()
        assert _ft_force_guarded_col(df, "thumb", "fz") is None
        assert _ft_guard_rejected_col(df, "thumb") is None

    def test_guarded_columns_do_not_register_as_extra_ft_labels(self):
        """`ft_thumb_fz_guarded` must not be read as a fingertip named
        `thumb_fz`, and must not be picked up as a second raw-force set."""
        df = _force_only_df(labels=("thumb",), guard=True)
        assert _detect_ft_labels(df) == ["thumb"]
        assert _ft_force_col(df, "thumb", "fz") == "ft_thumb_fz"

    def test_raw_column_keeps_the_spike_the_guard_held_out(self):
        """The guard never launders raw: the CSV's raw column still carries the
        6.4 N wire spike that the guarded column holds out."""
        df = _force_only_df(labels=("thumb",), guard=True)
        assert df["ft_thumb_fz"].max() > 6.0
        assert df["ft_thumb_fz_guarded"].max() < 6.0


class TestPlotFingertipForceOnly:
    def test_renders_one_row_per_fingertip_and_two_columns(self, tmp_path):
        import matplotlib.pyplot as plt

        df = _force_only_df()
        _plot_fingertip_force_only_auto(df, save_dir=str(tmp_path))
        assert (tmp_path / "fingertip_force.png").exists()
        plt.close("all")

    def test_axes_share_x_globally_and_y_per_column(self, monkeypatch, tmp_path):
        """Zoom must propagate across every subplot on time, but the signed
        components and the non-negative magnitude must not share a y range."""
        import matplotlib.pyplot as plt

        from rtc_tools.plotting.plotters import sensors as _sensors

        # The plotter closes its figure; keep it alive to inspect the axes.
        monkeypatch.setattr(_sensors.plt, "close", lambda *a, **k: None)
        _plot_fingertip_force_only_auto(_force_only_df(), save_dir=str(tmp_path))

        axes = plt.gcf().axes
        assert len(axes) == 8  # 4 fingertips x (components, magnitude)

        x_sibs = axes[0].get_shared_x_axes().get_siblings(axes[0])
        assert set(map(id, x_sibs)) == set(map(id, axes)), "time axis must be shared by all 8"

        left, right = axes[0], axes[1]
        y_left = set(map(id, left.get_shared_y_axes().get_siblings(left)))
        assert id(right) not in y_left, "signed components must not share y with the magnitude"
        assert id(axes[2]) in y_left, "same column must share y across fingertips"

        plt.close("all")

    def test_renders_without_filtered_columns(self, tmp_path, capsys):
        """A session recorded before the LPF columns existed still plots."""
        import matplotlib.pyplot as plt

        _plot_fingertip_force_only_auto(_force_only_df(filtered=False), save_dir=str(tmp_path))
        assert (tmp_path / "fingertip_force.png").exists()
        assert "no ft_*_filt columns" in capsys.readouterr().out
        plt.close("all")

    def test_guard_hold_labels_appear_only_for_guarded_sessions(self, monkeypatch, tmp_path):
        import matplotlib.pyplot as plt

        captured = {}

        real_subplots = plt.subplots

        def spy(*args, **kwargs):
            fig, axes = real_subplots(*args, **kwargs)
            captured["axes"] = axes
            return fig, axes

        monkeypatch.setattr(plt, "subplots", spy)
        _plot_fingertip_force_only_auto(
            _force_only_df(labels=("thumb",), guard=True), save_dir=str(tmp_path)
        )
        axes = captured["axes"]
        left_labels = [ln.get_label() for ln in axes[0, 0].get_lines()]
        title = axes[0, 0].get_title()
        assert "guard hold" in left_labels, left_labels
        # Labelled once per row, not once per axis.
        assert left_labels.count("guard hold") == 1, left_labels
        assert "guard-held tick(s)" in title, title
        # The magnitude panel marks the same ticks, so the two panels cannot
        # disagree about when the guard fired.
        assert any("guard hold (n=2)" in str(lbl) for lbl in _collection_labels(axes[0, 1])), (
            _collection_labels(axes[0, 1])
        )
        plt.close("all")

        captured.clear()
        monkeypatch.setattr(plt, "subplots", spy)
        _plot_fingertip_force_only_auto(_force_only_df(labels=("thumb",)), save_dir=str(tmp_path))
        axes = captured["axes"]
        plain_labels = [ln.get_label() for ln in axes[0, 0].get_lines()]
        assert "guard hold" not in plain_labels, plain_labels
        assert "guard-held" not in axes[0, 0].get_title()
        plt.close("all")

        # Guard lane present but nothing rejected — the overlay must stay off.
        # guarded == raw on every accepted tick, so drawing it unconditionally
        # would add three lines exactly on top of the raw traces and a legend
        # entry that claims something happened.
        quiet = _force_only_df(labels=("thumb",), guard=True)
        quiet["ft_thumb_force_guard_rejected"] = 0
        captured.clear()
        monkeypatch.setattr(plt, "subplots", spy)
        _plot_fingertip_force_only_auto(quiet, save_dir=str(tmp_path))
        axes = captured["axes"]
        quiet_labels = [ln.get_label() for ln in axes[0, 0].get_lines()]
        assert "guard hold" not in quiet_labels, quiet_labels
        assert "guard-held" not in axes[0, 0].get_title()
        assert not any("guard hold" in str(lbl) for lbl in _collection_labels(axes[0, 1]))
        plt.close("all")

    def test_no_labels_is_a_noop(self, tmp_path, capsys):
        import matplotlib.pyplot as plt

        _plot_fingertip_force_only_auto(pd.DataFrame({"timestamp": [0.0]}), save_dir=str(tmp_path))
        assert not (tmp_path / "fingertip_force.png").exists()
        assert "Skipping force-only plot" in capsys.readouterr().out
        plt.close("all")


class TestRegistryForceOnlyRouting:
    def test_force_only_selects_force_figure_and_skips_baro_tof(self):
        from rtc_tools.plotting.pipelines.registry import PIPELINES

        df = _force_only_df()
        fired = {e.name for e in PIPELINES["sensor_log"] if e.flag is None and e.available(df)}
        assert fired == {"fingertip_force"}

    def test_hand_with_sensor_lane_keeps_the_original_figures(self):
        from rtc_tools.plotting.pipelines.registry import PIPELINES

        df = _force_only_df()
        for label in ("thumb", "index", "middle", "ring"):
            for v in range(11):
                df[f"{label}_filt_{v}"] = np.zeros(len(df))
        fired = {e.name for e in PIPELINES["sensor_log"] if e.flag is None and e.available(df)}
        assert fired == {"sensor_barometer", "sensor_tof", "device_ft_output"}


class TestNarrowRowWarning:
    def test_row_narrower_than_header_warns_but_loads(self, tmp_path, capsys):
        """The DeviceSensorLog writer legitimately emits short rows when the
        device reported no fingertips; pandas NaN-fills them silently, so the
        loader has to say so."""
        path = tmp_path / "p_sensor.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_relative_s", "inference_valid", "ft_thumb_fx"])
            w.writerow([0.0, 0])
            w.writerow([0.002, 0])

        df = load_log_csv(str(path), "sensor_log")
        assert len(df) == 2
        assert df["ft_thumb_fx"].isna().all()
        assert "narrower than its header" in capsys.readouterr().out

    def test_full_width_rows_do_not_warn(self, tmp_path, capsys):
        path = tmp_path / "p_sensor.csv"
        _write_csv(
            path,
            ["t_relative_s", "inference_valid", "ft_thumb_fx"],
            [[0.0, 1, 0.5], [0.002, 1, 0.6]],
        )
        load_log_csv(str(path), "sensor_log")
        assert "narrower than its header" not in capsys.readouterr().out


# ═══════════════════════════════════════════════════════════════════════════
# resolve_default_save_dir — CSV 경로에서 세션 역산
# ═══════════════════════════════════════════════════════════════════════════

import os  # noqa: E402

from rtc_tools.plotting.io.session import (  # noqa: E402
    find_enclosing_session as _find_enclosing_session,
    resolve_default_save_dir as _resolve_default_save_dir,
)

_TIMING_HEADER = ["t_wall_ns", "tick_count", "t_total_us", "jitter_us"]
_TIMING_ROWS = [[i * 1_000_000, i + 1, 100.0 + i, float(i)] for i in range(20)]


def _make_session(root, name="260101_0900", *, with_plots=True):
    """Create <root>/<name>/ (+ plots/) and return the session path."""
    session = root / name
    (session / "controllers" / "demo_wbc_controller").mkdir(parents=True)
    if with_plots:
        (session / "plots").mkdir()
    (session / "timing").mkdir()
    return session


@pytest.fixture
def _isolated_session_env(monkeypatch, tmp_path):
    """세션 해석에 영향을 주는 env 를 지우고 cwd 를 marker 없는 dir 로 옮긴다.

    colcon test 는 COLCON_PREFIX_PATH 를 세팅한 채 실제 ws 안에서 돌기 때문에,
    격리하지 않으면 폴백 단계가 개발자의 실제 logging_data 를 집어 테스트가
    호스트 상태에 의존한다.
    """
    for name in ("RTC_SESSION_DIR", "COLCON_PREFIX_PATH"):
        monkeypatch.delenv(name, raising=False)
    bare = tmp_path / "bare_cwd"
    bare.mkdir()
    monkeypatch.chdir(bare)


class TestFindEnclosingSession:
    """find_enclosing_session: CSV 경로 상위에서 YYMMDD_HHMM 디렉토리를 찾는다."""

    def test_csv_directly_in_session_root(self, tmp_path):
        session = _make_session(tmp_path)
        csv_path = session / "p1b_state.csv"
        csv_path.touch()
        assert _find_enclosing_session(str(csv_path)) == str(session)

    def test_csv_in_nested_subdir(self, tmp_path):
        session = _make_session(tmp_path)
        csv_path = session / "controllers" / "demo_wbc_controller" / "p1b_state.csv"
        csv_path.touch()
        assert _find_enclosing_session(str(csv_path)) == str(session)

    def test_relative_path_is_resolved(self, tmp_path, monkeypatch):
        session = _make_session(tmp_path)
        (session / "timing" / "cm_timing_log.csv").touch()
        monkeypatch.chdir(session / "timing")
        assert _find_enclosing_session("cm_timing_log.csv") == str(session)

    def test_returns_none_outside_session_tree(self, tmp_path):
        loose = tmp_path / "loose.csv"
        loose.touch()
        assert _find_enclosing_session(str(loose)) is None

    def test_lookalike_dir_name_is_not_a_session(self, tmp_path):
        """길이 11 + '_' 위치만 보던 옛 판정은 이런 이름을 세션으로 오인했다."""
        fake = tmp_path / "2601xx_0900"
        fake.mkdir()
        csv_path = fake / "p1b_state.csv"
        csv_path.touch()
        assert _find_enclosing_session(str(csv_path)) is None


class TestResolveDefaultSaveDir:
    """resolve_default_save_dir: CSV 세션 → RTC_SESSION_DIR → 최신 세션 순."""

    def test_csv_session_beats_env_session(self, tmp_path, monkeypatch, _isolated_session_env):
        root = tmp_path / "logging_data"
        root.mkdir()
        old = _make_session(root, "260101_0900")
        new = _make_session(root, "260202_1000")
        monkeypatch.setenv("RTC_SESSION_DIR", str(new))

        csv_path = old / "controllers" / "demo_wbc_controller" / "p1b_state.csv"
        csv_path.touch()

        assert _resolve_default_save_dir(str(csv_path)) == str(old / "plots")

    def test_csv_session_beats_latest_session(self, tmp_path, _isolated_session_env):
        root = tmp_path / "logging_data"
        root.mkdir()
        old = _make_session(root, "260101_0900")
        _make_session(root, "260202_1000")

        csv_path = old / "timing" / "cm_timing_log.csv"
        csv_path.touch()

        assert _resolve_default_save_dir(str(csv_path)) == str(old / "plots")

    def test_creates_plots_dir_when_absent(self, tmp_path, _isolated_session_env):
        session = _make_session(tmp_path, with_plots=False)
        csv_path = session / "p1b_state.csv"
        csv_path.touch()

        save_dir = _resolve_default_save_dir(str(csv_path))
        assert save_dir == str(session / "plots")
        assert os.path.isdir(save_dir)

    def test_falls_back_to_env_outside_session_tree(
        self, tmp_path, monkeypatch, _isolated_session_env
    ):
        session = _make_session(tmp_path / "elsewhere", "260303_1100")
        monkeypatch.setenv("RTC_SESSION_DIR", str(session))

        loose = tmp_path / "loose_state.csv"
        loose.touch()

        assert _resolve_default_save_dir(str(loose)) == str(session / "plots")

    def test_falls_back_to_latest_session_without_env(
        self, tmp_path, monkeypatch, _isolated_session_env
    ):
        ws = tmp_path / "ws"
        (ws / "install").mkdir(parents=True)
        (ws / "src").mkdir()
        root = ws / "logging_data"
        root.mkdir()
        _make_session(root, "260101_0900")
        newest = _make_session(root, "260202_1000")
        monkeypatch.chdir(ws)

        loose = tmp_path / "loose_state.csv"
        loose.touch()

        assert _resolve_default_save_dir(str(loose)) == str(newest / "plots")

    def test_returns_none_when_no_session_anywhere(self, tmp_path, _isolated_session_env):
        loose = tmp_path / "loose_state.csv"
        loose.touch()
        assert _resolve_default_save_dir(str(loose)) is None

    def test_no_csv_arg_keeps_env_chain(self, tmp_path, monkeypatch, _isolated_session_env):
        session = _make_session(tmp_path, "260404_1200")
        monkeypatch.setenv("RTC_SESSION_DIR", str(session))
        assert _resolve_default_save_dir() == str(session / "plots")


class TestMainSaveDirRouting:
    """main() end-to-end: PNG 가 CSV 와 같은 세션에 떨어지는지."""

    def _write_timing_csv(self, session):
        path = session / "timing" / "cm_timing_log.csv"
        _write_csv(path, _TIMING_HEADER, _TIMING_ROWS)
        return path

    def test_figures_land_in_the_csv_own_session(
        self, tmp_path, monkeypatch, _isolated_session_env
    ):
        from rtc_tools.plotting.plot_rtc_log import main

        root = tmp_path / "logging_data"
        root.mkdir()
        old = _make_session(root, "260101_0900")
        newest = _make_session(root, "260202_1000")
        monkeypatch.setenv("RTC_SESSION_DIR", str(newest))

        csv_path = self._write_timing_csv(old)
        monkeypatch.setattr("sys.argv", ["plot_rtc_log", str(csv_path), "--no-show"])
        main()

        assert list((old / "plots").glob("*.png")), "CSV 세션에 figure 가 없다"
        assert not list((newest / "plots").glob("*.png")), "env 세션으로 새어나갔다"

    def test_explicit_save_dir_still_wins(self, tmp_path, monkeypatch, _isolated_session_env):
        from rtc_tools.plotting.plot_rtc_log import main

        session = _make_session(tmp_path, "260101_0900")
        csv_path = self._write_timing_csv(session)
        out = tmp_path / "explicit_out"

        monkeypatch.setattr(
            "sys.argv",
            ["plot_rtc_log", str(csv_path), "--no-show", "--save-dir", str(out)],
        )
        main()

        assert list(out.glob("*.png"))
        assert not list((session / "plots").glob("*.png"))
