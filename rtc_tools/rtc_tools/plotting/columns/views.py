"""Column predicates that gate per-controller plot variants.

These are the "does this CSV have X?" decisions that today live inline in
`main()`. Phase 4's pipeline registry uses them as `available=` callbacks
on each `PlotEntry`, so adding a new controller variant becomes a single
predicate + registry row instead of another `elif` in `main()`.

Predicates take `df` and return `bool`. Cheap to call repeatedly
(no DataFrame copies).
"""

from .detect import (
    detect_fingertip_labels,
    detect_fingertip_labels_raw,
    has_columns,
    has_motor_columns,
)


def has_task_goal(df):
    """state_log has a `goal_type` column with at least one `task` value
    AND `task_pos_*` columns to plot. Triggers task-space TCP plots.
    """
    if "goal_type" not in df.columns:
        return False
    if not has_columns(df, "task_pos_", 3):
        return False
    return (df["goal_type"] == "task").any()


def has_command_type(df):
    """`command_type` column exists — distinguishes position vs torque commands."""
    return "command_type" in df.columns


def has_joint_goal_gui(df):
    """`joint_goal_*` columns from GUI exist (overlay on position plot)."""
    return any(c.startswith("joint_goal_") for c in df.columns)


def has_motor(df):
    """state_log has motor_*/auxiliary motor channels.

    Wraps `has_motor_columns` for symmetry with the other view predicates.
    """
    return has_motor_columns(df)


def has_fingertip_sensors(df):
    """sensor_log/device has at least one fingertip's sensor columns.

    Schema-agnostic: matches the Phase C `<name>_filt_*` block and the legacy
    `baro_*`/`tof_*` columns (both handled by `detect_fingertip_labels`).
    """
    return bool(detect_fingertip_labels(df))


def has_ft_inference(df):
    """sensor_log has FT inference output (ft_*_contact or legacy ft_*_fx)."""
    return any(c.startswith("ft_") and c.endswith(("_contact", "_fx")) for c in df.columns)


def has_raw_sensors(df):
    """sensor_log has raw (pre-LPF) sensor columns.

    Schema-agnostic: matches the Phase C `<name>_raw_*` block and the legacy
    `baro_raw_*`/`tof_raw_*` columns (both handled by
    `detect_fingertip_labels_raw`).
    """
    return bool(detect_fingertip_labels_raw(df))


def has_ft_force_filtered(df):
    """sensor_log has the per-axis LPF'd force columns (`ft_*_fx_filt`)."""
    return any(c.startswith("ft_") and c.endswith("_fx_filt") for c in df.columns)


def has_ft_force_guard(df):
    """sensor_log carries the delta-spike guard lane (`ft_*_force_guard_rejected`).

    Sessions recorded before the guard landed have raw + `_filt` only; the
    guard overlay is skipped for them rather than drawn as all-zero.
    """
    return any(c.startswith("ft_") and c.endswith("_force_guard_rejected") for c in df.columns)


def has_force_only_fingertips(df):
    """Fingertips report force but carry no barometer/ToF lane.

    True for a hand whose `sensor_layout.values_per_group` is 0: the CSV writer
    then emits no `<name>_raw_*` / `<name>_filt_*` block at all, so column
    presence alone separates "no such hardware" from "hardware reading zero".

    Sessions recorded before the writer honoured that stride still carry the
    zero-filled block and are NOT matched here — they keep routing to the
    barometer/ToF figures, which is what those files' columns claim to be.
    """
    if not has_ft_inference(df):
        return False
    return not detect_fingertip_labels(df) and not detect_fingertip_labels_raw(df)


# ── WBC-specific (DeviceWbcLog / WbcDiagLog) predicates ────────────────────


def has_wbc_accel(df):
    """DeviceWbcLog has per-joint TSID acceleration columns (`accel_*`)."""
    return any(c.startswith("accel_") for c in df.columns)


def has_wbc_task_traj(df):
    """DeviceWbcLog arm role has the SE3 trajectory setpoint block
    (`traj_task_pos_*`) alongside the actual task columns."""
    return has_columns(df, "traj_task_pos_", 3) and has_columns(df, "task_pos_", 3)


def has_wbc_fingertip_force(df):
    """DeviceWbcLog hand role has per-fingertip |F| columns
    (`fingertip_force_*`)."""
    return any(c.startswith("fingertip_force_") for c in df.columns)


def has_wbc_lambda(df):
    """WbcDiagLog has QP contact-wrench solution columns (`lambda_*`)."""
    return any(c.startswith("lambda_") for c in df.columns)


# ── MomentumObserverLog (#135 / #455) predicates ───────────────────────────
#
# These gate on CONTENT, not on column presence, and that difference is the
# whole point. The payload/inertial sub-blocks ship `enabled: false`, so a
# normal session carries the columns pinned at zero with reason=kNotInitialized
# for every row. A column-presence gate would render three all-zero panels for
# such a session; the reason code is what separates "the estimator ran and was
# gated" (worth plotting — the reason mix is exactly what a tuner reads) from
# "the estimator was never configured" (nothing to show). The producer pins
# that contract: "서브블록이 없으면 전 컬럼 0 + payload_reason=1(미초기화)".

# rtc::estimation::{Payload,Inertial}InvalidReason::kNotInitialized.
_REASON_NOT_INITIALIZED = 1


def _estimator_was_configured(df, reason_col):
    """True when `reason_col` shows anything other than a never-initialized run.

    Absent column → False (a session recorded before that layer existed).
    All-NaN column → False; a file truncated mid-write carries no evidence
    either way, and an empty figure is the worse of the two failure modes.
    """
    if reason_col not in df.columns:
        return False
    reasons = df[reason_col].dropna()
    if reasons.empty:
        return False
    return bool((reasons != _REASON_NOT_INITIALIZED).any())


def has_payload_estimate(df):
    """MomentumObserverLog carries a Layer 2A payload estimate that ran."""
    return _estimator_was_configured(df, "payload_reason")


def has_inertial_estimate(df):
    """MomentumObserverLog carries a Layer 2B inertial regression that ran.

    Sessions recorded before #455 have no `inertial_*` columns at all, so the
    inertial series are dropped from the payload figure rather than drawn flat.
    """
    return _estimator_was_configured(df, "inertial_reason")
