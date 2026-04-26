"""Column predicates that gate per-controller plot variants.

These are the "does this CSV have X?" decisions that today live inline in
`main()`. Phase 4's pipeline registry uses them as `available=` callbacks
on each `PlotEntry`, so adding a new controller variant becomes a single
predicate + registry row instead of another `elif` in `main()`.

Predicates take `df` and return `bool`. Cheap to call repeatedly
(no DataFrame copies).
"""

from .detect import has_columns, has_motor_columns


def has_task_goal(df):
    """state_log has a `goal_type` column with at least one `task` value
    AND `task_pos_*` columns to plot. Triggers task-space TCP plots.
    """
    if "goal_type" not in df.columns:
        return False
    if not has_columns(df, "task_pos_", 3):
        return False
    return (df["goal_type"] == "task").any()


def has_traj_task(df):
    """`traj_task_pos_*` columns exist (task-space trajectory reference).
    Required for task tracking-error plot.
    """
    return has_columns(df, "traj_task_pos_", 3)


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
    """sensor_log/device has at least one fingertip's barometer or ToF columns."""
    for c in df.columns:
        if c.startswith(("baro_", "tof_")):
            return True
    return False


def has_ft_inference(df):
    """sensor_log has FT inference output (ft_*_contact or legacy ft_*_fx)."""
    for c in df.columns:
        if c.startswith("ft_") and (c.endswith("_contact") or c.endswith("_fx")):
            return True
    return False


def has_raw_sensors(df):
    """sensor_log has raw (pre-LPF) `baro_raw_*` or `tof_raw_*` columns."""
    for c in df.columns:
        if c.startswith(("baro_raw_", "tof_raw_")):
            return True
    return False
