"""Log-type detection — by filename pattern, falling back to column fingerprint."""

import csv
from pathlib import Path


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
    if stem.endswith("state_log"):
        return "state_log"
    elif stem.endswith("sensor_log"):
        return "sensor_log"
    elif stem.startswith("robot_log"):
        return "robot"
    elif stem.startswith("device_log") or stem.startswith("hand_log"):
        return "device"
    elif stem.startswith("timing_log"):
        return "timing"
    elif stem == "mpc_solve_timing":
        return "mpc_solve_timing"
    else:
        return "unknown"


def detect_log_type_by_columns(columns):
    """Fallback: infer log type from CSV header columns.

    Used when filename doesn't match a known pattern.
    Priority: mpc_solve_timing > timing > state_log > sensor_log.
    """
    cols = set(columns)
    if {"t_wall_ns", "p50_ns", "p99_ns", "max_ns"}.issubset(cols):
        return "mpc_solve_timing"
    if any(
        c in cols
        for c in (
            "t_total_us",
            "t_compute_us",
            "t_state_acquire_us",
            "t_publish_us",
            "jitter_us",
        )
    ):
        return "timing"
    if any(
        c.startswith(("actual_pos_", "goal_pos_", "traj_pos_", "target_pos_"))
        for c in cols
    ):
        return "state_log"
    if any(
        c.startswith(("baro_raw_", "tof_raw_", "baro_filt_", "tof_filt_", "ft_"))
        for c in cols
    ):
        return "sensor_log"
    return "unknown"


def peek_csv_header(filepath):
    """Read the first row of a CSV as column names. Empty list on error."""
    try:
        with open(filepath, "r") as f:
            return next(csv.reader(f), [])
    except (OSError, StopIteration):
        return []
