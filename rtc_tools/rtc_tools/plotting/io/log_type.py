"""Log-type detection — by filename pattern, falling back to column fingerprint."""

import csv
from pathlib import Path


def detect_log_type(filepath):
    """Detect log type from filename pattern + parent-directory hint.

    Legacy CM-side DataLogger paths (Phase C deletes these):
      *_state_log.csv  → state_log  (DeviceStateLog fields)
      *_sensor_log.csv → sensor_log (DeviceSensorLog fields)

    Per-tick timing (unified 7-col schema):
      cm_timing_log*.csv  → cm_timing   (CM RT loop)
      mpc_timing_log*.csv → mpc_timing  (MPC main loop)
      hand_udp_timing_log*.csv → cm_timing  (hand UDP EventLoop —
                                             same 7-col schema, treated
                                             as cm_timing for plotting)

    Phase C controller-owned CSVs live under
    <session>/controllers/<config_key>/<instance>.csv. The instance
    stems do not encode their type, so we use the parent directory as
    a hint — files inside a `controllers/<key>/` parent default to
    state_log, with the column-based fallback overriding when needed.
    """
    p = Path(filepath)
    stem = p.stem
    # WBC controller-owned channels (Path A POD-only). The per-tick TSID/QP
    # diagnostics file has a fixed `wbc_diag` stem; the WBC arm/hand state
    # files reuse the generic `<dev>_state` stem, so they fall through to the
    # column fallback (which keys on the WBC-only `accel_*` columns).
    if stem == "wbc_diag" or stem.endswith("wbc_diag"):
        return "wbc_diag"
    elif stem == "grasp_diag" or stem.endswith("grasp_diag"):
        # Per-tick Force-PI servo + stiffness-estimator diagnostics (#428) —
        # fixed instance stem shared by the joint and task demo controllers.
        return "grasp_diag"
    elif stem == "pull_estimator" or stem.endswith("pull_estimator"):
        # In-plane pull-force estimator (#167) — fixed instance stem shared by
        # the three demo controllers.
        return "pull_estimator"
    elif stem.endswith("state_log"):
        return "state_log"
    elif stem.endswith("sensor_log"):
        return "sensor_log"
    elif stem.startswith(("cm_timing_log", "hand_udp_timing_log")):
        return "cm_timing"
    elif stem.startswith("mpc_timing_log"):
        return "mpc_timing"

    # Phase C controller-owned: <session>/controllers/<key>/<instance>.csv —
    # let the column fallback decide between state_log / sensor_log.
    if p.parent.parent.name == "controllers":
        return "unknown"
    return "unknown"


def detect_log_type_by_columns(columns):
    """Fallback: infer log type from CSV header columns.

    Used when filename doesn't match a known pattern. CM and MPC per-tick
    timing CSVs share the same 7-col schema, so column-based detection
    cannot tell them apart — return ``cm_timing`` and let the caller rely
    on the filename to pick the cm/mpc pipeline if needed.
    """
    cols = set(columns)
    # WBC diagnostics: per-tick TSID/QP solver telemetry (single wbc_diag.csv).
    # Distinct from timing CSVs (`solve_time_us`, not `t_*_us`).
    if "solve_time_us" in cols and (
        "qp_converged" in cols or any(c.startswith("lambda_") for c in cols)
    ):
        return "wbc_diag"
    # Force-PI grasp diagnostics (#428): per-tick grasp_diag.csv. MUST be
    # checked before the sensor_log branch — `k_inst_raw_<finger>` contains the
    # generic `_raw_` token that branch matches on, so a grasp_diag file reaching
    # the fallback would be classified as a sensor log and plotted with the wrong
    # pipeline. `k_est_` is the discriminating prefix: no other pod emits it.
    if any(c.startswith("k_est_") for c in cols):
        return "grasp_diag"
    # Pull-force estimator (#167): per-tick pull_estimator.csv. Must be
    # checked before the sensor_log branch — the legacy force_raw_* columns
    # would otherwise match the generic `_raw_` sensor token.
    # `force_prefilter_*` is the current name; `force_raw_*` is the pre-#234
    # spelling and is still accepted so archived sessions keep plotting.
    if "friction_utilization" in cols and ("force_prefilter_x" in cols or "force_raw_x" in cols):
        return "pull_estimator"
    # WBC device state: superset of state_log with TSID a_opt acceleration.
    # The `accel_*` prefix is unique to DeviceWbcLog, so it disambiguates the
    # WBC arm/hand state CSVs from the generic state_log before that branch.
    if any(c.startswith("accel_") for c in cols):
        return "wbc_log"
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
    # Phase C state-log POD prefixes: actual_pos_<joint>, joint_goal_<joint>,
    # traj_pos_<joint>, motor_pos_<joint>, task_pos_*. Legacy CM-side CSV
    # uses the same actual_pos_ / goal_pos_ / traj_pos_ stems with numeric
    # suffixes — both match this list.
    if any(
        c.startswith(
            (
                "actual_pos_",
                "goal_pos_",
                "joint_goal_",
                "traj_pos_",
                "motor_pos_",
                "task_pos_",
            )
        )
        for c in cols
    ):
        return "state_log"
    # Sensor: legacy uses baro_raw_*, tof_raw_*, baro_filt_*, tof_filt_*,
    # ft_*. Phase C POD uses <sensor_name>_raw_*, <sensor_name>_filt_*,
    # ft_<sensor_name>_*. Both match the trailing _raw_/_filt_ tokens.
    if any(
        c.startswith(("baro_raw_", "tof_raw_", "baro_filt_", "tof_filt_", "ft_"))
        or "_raw_" in c
        or "_filt_" in c
        for c in cols
    ):
        return "sensor_log"
    return "unknown"


def peek_csv_header(filepath):
    """Read the first row of a CSV as column names. Empty list on error."""
    try:
        with open(filepath) as f:
            return next(csv.reader(f), [])
    except (OSError, StopIteration):
        return []
