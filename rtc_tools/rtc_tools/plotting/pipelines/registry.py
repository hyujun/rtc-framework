"""Declarative dispatch table: log_type → ordered list of `PlotEntry`.

Each entry carries:
  - `fn(df, save_dir)` — the plot function (or `_auto` adapter)
  - `available(df)` — column-set predicate; False → skip silently
  - `flag` — opt-in CLI flag (None = always-on if `available` passes)

Behaviour rule (matches the original main() dispatch):
  • If `available(df)` is False → skip.
  • Else if `flag` is None → run.
  • Else if `args.<flag>` is True → run; otherwise skip.

Order matters — figures are produced in registry order so that the existing
PNG file naming and per-session output ordering stays stable. Adding a new
controller-specific plot is one row; nothing in main() needs to change.
"""

from collections.abc import Callable
from dataclasses import dataclass

from rtc_tools.plotting import plotters
from rtc_tools.plotting.columns import (
    has_motor,
    has_task_goal,
)


@dataclass(frozen=True)
class PlotEntry:
    name: str
    fn: Callable  # (df, save_dir) -> None
    available: Callable = lambda df: True  # column-driven gate
    flag: str | None = None  # CLI opt-in flag name


# Match original behaviour: state_log task plots fire if EITHER the
# user passes --task-pos OR the CSV genuinely contains task-mode data.
def _state_or(predicate, flag_name):
    def _gate(df, args, _p=predicate, _f=flag_name):
        return _p(df) or getattr(args, _f, False)

    return _gate


# Per-tick timing CSVs (cm_timing_log.csv / mpc_timing_log.csv) share the
# unified 7-col schema, so the same stats printer + plotter list applies to
# both. PNG output filename is disambiguated by the source CSV's parent
# directory.
_TIMING_STATS = [
    PlotEntry("print_timing_stats", plotters.print_timing_statistics),
]
_TIMING_PLOTS = [
    PlotEntry("timing_breakdown", plotters.plot_timing_breakdown),
    PlotEntry("timing_total_jitter", plotters.plot_timing_total_and_jitter),
    PlotEntry("timing_histograms", plotters.plot_timing_histograms),
]


# `STATS_PRINTERS` runs before any plotting and respects `available()` only.
STATS_PRINTERS: dict[str, list[PlotEntry]] = {
    "state_log": [
        PlotEntry("print_robot_stats", plotters.print_robot_statistics),
        PlotEntry("print_motor_stats", plotters.print_motor_statistics, has_motor),
    ],
    "sensor_log": [
        PlotEntry("print_device_stats", plotters.print_device_statistics),
    ],
    "cm_timing": list(_TIMING_STATS),
    "mpc_timing": list(_TIMING_STATS),
}


# `PIPELINES` runs unless --stats was passed. Order is significant.
PIPELINES: dict[str, list[PlotEntry]] = {
    "state_log": [
        PlotEntry("robot_positions", plotters.plot_robot_positions),
        PlotEntry("robot_velocities", plotters.plot_robot_velocities),
        PlotEntry("robot_torques", plotters.plot_robot_torques),
        # task-pos plots fire on auto-detect OR --task-pos
        PlotEntry(
            "robot_task_position",
            plotters.plot_robot_task_position,
            available=has_task_goal,
            flag="task_pos",
        ),
        PlotEntry(
            "robot_task_tracking_error",
            plotters.plot_robot_task_tracking_error,
            available=has_task_goal,
            flag="task_pos",
        ),
        # opt-in only
        PlotEntry("robot_tracking_error", plotters.plot_robot_tracking_error, flag="error"),
        PlotEntry("robot_commands", plotters.plot_robot_commands, flag="command"),
        # auto when motor channels present
        PlotEntry("motor_positions", plotters.plot_motor_positions, available=has_motor),
        PlotEntry("motor_velocities", plotters.plot_motor_velocities, available=has_motor),
        PlotEntry("motor_efforts", plotters.plot_motor_efforts, available=has_motor),
    ],
    "sensor_log": [
        PlotEntry("sensor_barometer", plotters.plot_sensor_barometer_combined),
        PlotEntry("sensor_tof", plotters.plot_sensor_tof_combined),
        PlotEntry("device_ft_output", plotters.plot_device_ft_output_auto),
        PlotEntry(
            "device_sensor_comparison",
            plotters.plot_device_sensor_comparison_auto,
            flag="sensor_compare",
        ),
    ],
    "cm_timing": list(_TIMING_PLOTS),
    "mpc_timing": list(_TIMING_PLOTS),
}


def _entry_fires(entry, df, args):
    """Combine `available(df)` and CLI flag opt-in.

    For task-pos entries (flag="task_pos"), original behaviour is OR:
    fire if predicate True OR flag set. For other flags, fire only if
    predicate True AND flag set (or flag is None).
    """
    avail = entry.available(df)
    if entry.flag is None:
        return avail
    flag_set = getattr(args, entry.flag, False)
    if entry.flag == "task_pos":
        # task-pos auto-detect OR opt-in
        return avail or flag_set
    return avail and flag_set


def run_pipeline(log_type, df, args, save_dir):
    """Run STATS_PRINTERS + PIPELINES[log_type] honouring `--stats` and flags.

    Returns silently if log_type is not in the registry — caller is expected
    to have validated log_type already.
    """
    for entry in STATS_PRINTERS.get(log_type, []):
        if entry.available(df):
            entry.fn(df)

    if args.stats:
        return

    for entry in PIPELINES.get(log_type, []):
        if _entry_fires(entry, df, args):
            entry.fn(df, save_dir)
