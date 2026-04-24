"""Tests for rtc_tools.plotting.plot_rtc_log module.

CSV 유틸리티, 컬럼 감지, 로그 타입 분류, subplot 그리드 계산 등
플롯 의존 없이 테스트할 수 있는 순수 로직을 검증합니다.
"""

from __future__ import annotations

import csv

import numpy as np
import pandas as pd
import pytest

from rtc_tools.plotting.plot_rtc_log import (
    _auto_subplot_grid,
    _detect_csv_column_mismatch,
    _detect_fingertip_labels,
    _detect_fingertip_labels_raw,
    _detect_ft_labels,
    _detect_joint_columns,
    _detect_num_channels,
    _has_columns,
    _invalidate_column_cache,
    _load_sensor_log_csv,
    _rebuild_sensor_log_header,
    detect_log_type,
)


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
        assert detect_log_type("ur5e_state_log.csv") == "state_log"

    def test_sensor_log(self):
        assert detect_log_type("hand_sensor_log.csv") == "sensor_log"

    def test_robot_log(self):
        assert detect_log_type("robot_log_20250101.csv") == "robot"

    def test_device_log(self):
        assert detect_log_type("device_log_20250101.csv") == "device"

    def test_hand_log(self):
        assert detect_log_type("hand_log_20250101.csv") == "device"

    def test_timing_log(self):
        assert detect_log_type("timing_log_20250101.csv") == "timing"

    def test_unknown(self):
        assert detect_log_type("random_file.csv") == "unknown"

    def test_full_path(self):
        assert detect_log_type("/data/logs/robot_log_001.csv") == "robot"

    def test_sensor_log_with_prefix(self):
        assert detect_log_type("ur5e_hand_sensor_log.csv") == "sensor_log"

    def test_state_log_with_prefix(self):
        assert detect_log_type("device0_state_log.csv") == "state_log"


# ═══════════════════════════════════════════════════════════════════════════
# detect_log_type_by_columns — 컬럼 기반 fallback 분류
# ═══════════════════════════════════════════════════════════════════════════


class TestDetectLogTypeByColumns:
    """detect_log_type_by_columns: 파일명이 안 맞을 때 CSV 헤더로 분류."""

    def test_timing_by_t_total_us(self):
        cols = ["timestamp", "t_total_us", "jitter_us"]
        assert detect_log_type_by_columns(cols) == "timing"

    def test_timing_by_jitter_only(self):
        assert detect_log_type_by_columns(["timestamp", "jitter_us"]) == "timing"

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
        assert detect_log_type_by_columns(cols) == "timing"


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
        assert detect_log_type_by_columns(_peek_csv_header(str(path))) == "timing"


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
# CSV 컬럼 불일치 감지 + 헤더 복구
# ═══════════════════════════════════════════════════════════════════════════


class TestDetectCsvColumnMismatch:
    def test_no_mismatch(self, tmp_path):
        path = tmp_path / "ok.csv"
        _write_csv(path, ["a", "b", "c"], [[1, 2, 3], [4, 5, 6]])

        h_count, d_count, header = _detect_csv_column_mismatch(str(path))
        assert h_count == 3
        assert d_count == 3
        assert header == ["a", "b", "c"]

    def test_data_has_more_columns(self, tmp_path):
        path = tmp_path / "mismatch.csv"
        # 헤더 3개, 데이터 5개
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["a", "b", "c"])
            writer.writerow([1, 2, 3, 4, 5])

        h_count, d_count, header = _detect_csv_column_mismatch(str(path))
        assert h_count == 3
        assert d_count == 5

    def test_empty_file(self, tmp_path):
        path = tmp_path / "empty.csv"
        path.write_text("")

        h_count, d_count, header = _detect_csv_column_mismatch(str(path))
        assert h_count == 0
        assert d_count == 0
        assert header == []


class TestRebuildSensorLogHeader:
    def test_no_missing(self):
        header = ["timestamp", "baro_raw_thumb_0", "baro_raw_thumb_1"]
        result = _rebuild_sensor_log_header(header, len(header))
        assert result == header

    def test_adds_inference_columns(self):
        header = ["timestamp", "baro_raw_thumb_0"]
        result = _rebuild_sensor_log_header(header, 9)
        assert len(result) == 9
        # 누락된 7개 컬럼 추가됨
        added = result[2:]
        assert len(added) == 7
        # ft_thumb_{comp} 패턴
        assert "ft_thumb_contact" in added
        assert "ft_thumb_fx" in added

    def test_generic_fallback(self):
        """fingertip 라벨을 찾을 수 없으면 generic 이름 사용."""
        header = ["timestamp", "x", "y"]
        result = _rebuild_sensor_log_header(header, 6)
        assert len(result) == 6
        added = result[3:]
        assert all(c.startswith("inference_") for c in added)

    def test_multiple_fingertips(self):
        header = [
            "timestamp",
            "baro_raw_thumb_0",
            "baro_raw_index_0",
        ]
        result = _rebuild_sensor_log_header(header, 17)
        assert len(result) == 17
        # thumb + index 각각 7개 = 14, 하지만 missing = 14
        added = result[3:]
        assert "ft_thumb_contact" in added
        assert "ft_index_contact" in added


class TestLoadSensorLogCsv:
    def test_normal_load(self, tmp_path):
        path = tmp_path / "hand_sensor_log.csv"
        _write_csv(
            path, ["timestamp", "baro_0", "tof_0"], [[0.0, 100, 50], [0.002, 101, 51]]
        )

        df, repaired = _load_sensor_log_csv(str(path))
        assert not repaired
        assert len(df) == 2
        assert "timestamp" in df.columns

    def test_repaired_load(self, tmp_path):
        path = tmp_path / "hand_sensor_log.csv"
        # 헤더 2개, 데이터 4개 (inference 누락 시뮬레이션)
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "baro_raw_thumb_0"])
            writer.writerow([0.0, 100, 0.8, 1.0])
            writer.writerow([0.002, 101, 0.9, 1.1])

        df, repaired = _load_sensor_log_csv(str(path))
        assert repaired
        assert len(df.columns) == 4

    def test_empty_file_raises(self, tmp_path):
        path = tmp_path / "empty_sensor_log.csv"
        path.write_text("")

        with pytest.raises(pd.errors.EmptyDataError):
            _load_sensor_log_csv(str(path))


# ═══════════════════════════════════════════════════════════════════════════
# 통계 함수 — print_robot_statistics / print_timing_statistics
# ═══════════════════════════════════════════════════════════════════════════


class TestStatistics:
    """통계 출력 함수가 예외 없이 실행되는지 검증 (출력 내용은 smoke test)."""

    def test_robot_statistics_no_crash(self, capsys):
        from rtc_tools.plotting.plot_rtc_log import print_robot_statistics

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
        from rtc_tools.plotting.plot_rtc_log import print_robot_statistics

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
        from rtc_tools.plotting.plot_rtc_log import print_timing_statistics

        df = pd.DataFrame(
            {
                "timestamp": np.arange(0, 1.0, 0.002),
                "t_state_acquire_us": np.random.uniform(10, 50, 500),
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
        from rtc_tools.plotting.plot_rtc_log import print_device_statistics

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
        from rtc_tools.plotting.plot_rtc_log import print_motor_statistics

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
