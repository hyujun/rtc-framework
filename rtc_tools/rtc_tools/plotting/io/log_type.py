"""Log-type detection — by filename pattern, falling back to column fingerprint."""

import csv
from pathlib import Path


def detect_log_type(filepath):
    """Detect log type from filename pattern.

    Topic-role based (DataLogger emits):
      *_state_log.csv  → state_log  (DeviceStateLog fields)
      *_sensor_log.csv → sensor_log (DeviceSensorLog fields)
    Per-tick timing (unified 7-col schema):
      cm_timing_log*.csv  → cm_timing   (CM RT loop)
      mpc_timing_log*.csv → mpc_timing  (MPC main loop)
    """
    stem = Path(filepath).stem
    if stem.endswith("state_log"):
        return "state_log"
    elif stem.endswith("sensor_log"):
        return "sensor_log"
    elif stem.startswith("cm_timing_log"):
        return "cm_timing"
    elif stem.startswith("mpc_timing_log"):
        return "mpc_timing"
    else:
        return "unknown"


def detect_log_type_by_columns(columns):
    """Fallback: infer log type from CSV header columns.

    Used when filename doesn't match a known pattern. CM and MPC per-tick
    timing CSVs share the same 7-col schema, so column-based detection
    cannot tell them apart — return ``cm_timing`` and let the caller rely
    on the filename to pick the cm/mpc pipeline if needed.
    """
    cols = set(columns)
    if any(
        c in cols
        for c in (
            "t_total_us",
            "t_compute_us",
            "t_state_us",
            "t_publish_us",
            "jitter_us",
        )
    ):
        return "cm_timing"
    if any(
        c.startswith(("actual_pos_", "goal_pos_", "traj_pos_"))
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
