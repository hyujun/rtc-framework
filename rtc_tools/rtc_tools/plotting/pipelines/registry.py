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
    has_force_only_fingertips,
    has_motor,
    has_payload_estimate,
    has_task_goal,
    has_wbc_accel,
    has_wbc_fingertip_force,
    has_wbc_task_traj,
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


def _not_force_only(df):
    """Gate for the barometer/ToF figures — skip when the hand has no such lane."""
    return not has_force_only_fingertips(df)


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
    # DeviceWbcLog is a state_log superset → reuse robot/motor stats.
    "wbc_log": [
        PlotEntry("print_robot_stats", plotters.print_robot_statistics),
        PlotEntry("print_motor_stats", plotters.print_motor_statistics, has_motor),
    ],
    "wbc_diag": [
        PlotEntry("print_wbc_diag_stats", plotters.print_wbc_diag_statistics),
    ],
    "pull_estimator": [
        PlotEntry("print_pull_stats", plotters.print_pull_estimator_statistics),
    ],
    # The stats printer is the point of this channel, not the figure: the
    # hardware session ships text back and #426's force-noise sigma has to be a
    # number on stdout before anyone opens a plot (#428).
    # Stats only, no figure entry (#469 S4): the deliverable is the envelope /
    # freshness / bias numbers S5 tunes against, and STATS_PRINTERS run on every
    # invocation — `--stats` suppresses figures, not these.
    "compliance_diag": [
        PlotEntry("print_compliance_diag_stats", plotters.print_compliance_diag_statistics),
    ],
    "grasp_diag": [
        PlotEntry("print_grasp_diag_stats", plotters.print_grasp_diag_statistics),
    ],
    # Same judgement as grasp_diag: the number is the deliverable. #135's sim
    # negative control is stated as "no load ⇒ ‖r‖∞ below tol", so the printer
    # covers Layer 1b/2A/2B in one pass and gates its own sections.
    "momentum_observer": [
        PlotEntry("print_momentum_stats", plotters.print_momentum_observer_statistics),
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
    # A force-only hand (sensor_layout.values_per_group == 0) has no
    # barometer/ToF columns at all, so those three figures would each render an
    # empty axis and `device_ft_output` would spend 3 of its 4 rows on channels
    # the hardware does not have. `_not_force_only` routes such a session to the
    # single `fingertip_force` figure instead; a hand WITH the sensor lane is
    # unaffected and still gets the full set.
    "sensor_log": [
        PlotEntry(
            "sensor_barometer",
            plotters.plot_sensor_barometer_combined,
            available=_not_force_only,
        ),
        PlotEntry("sensor_tof", plotters.plot_sensor_tof_combined, available=_not_force_only),
        PlotEntry(
            "device_ft_output",
            plotters.plot_device_ft_output_auto,
            available=_not_force_only,
        ),
        PlotEntry(
            "fingertip_force",
            plotters.plot_fingertip_force_only_auto,
            available=has_force_only_fingertips,
        ),
        PlotEntry(
            "device_sensor_comparison",
            plotters.plot_device_sensor_comparison_auto,
            flag="sensor_compare",
        ),
    ],
    # DeviceWbcLog (state_log superset): reuse the robot/motor plots, then add
    # the WBC-only acceleration / SE3-trajectory / fingertip-force figures.
    # Role-gated columns mean the arm CSV fires task_trajectory and the hand
    # CSV fires fingertip_force; available() skips the irrelevant ones.
    "wbc_log": [
        PlotEntry("robot_positions", plotters.plot_robot_positions),
        PlotEntry("robot_velocities", plotters.plot_robot_velocities),
        PlotEntry("robot_torques", plotters.plot_robot_torques),
        PlotEntry("wbc_accelerations", plotters.plot_wbc_accelerations, available=has_wbc_accel),
        PlotEntry(
            "wbc_task_trajectory",
            plotters.plot_wbc_task_trajectory,
            available=has_wbc_task_traj,
        ),
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
        PlotEntry("robot_tracking_error", plotters.plot_robot_tracking_error, flag="error"),
        PlotEntry("robot_commands", plotters.plot_robot_commands, flag="command"),
        PlotEntry("motor_positions", plotters.plot_motor_positions, available=has_motor),
        PlotEntry("motor_velocities", plotters.plot_motor_velocities, available=has_motor),
        PlotEntry("motor_efforts", plotters.plot_motor_efforts, available=has_motor),
        PlotEntry(
            "wbc_fingertip_force",
            plotters.plot_wbc_fingertip_force,
            available=has_wbc_fingertip_force,
        ),
    ],
    "wbc_diag": [
        PlotEntry("wbc_diag_solver", plotters.plot_wbc_diag_solver),
        PlotEntry("wbc_diag_contacts", plotters.plot_wbc_diag_contacts),
    ],
    "pull_estimator": [
        PlotEntry("pull_estimator", plotters.plot_pull_estimator),
    ],
    "grasp_diag": [
        PlotEntry("grasp_diag", plotters.plot_grasp_diag),
    ],
    # The residual figure is unconditional — Layer 1b is always present when the
    # file exists at all. The payload figure is gated on the REASON code, not on
    # column presence: the 2A/2B sub-blocks ship disabled, so a normal session
    # carries their columns pinned at zero and would otherwise get four empty
    # panels (see columns/views.py has_payload_estimate).
    "momentum_observer": [
        PlotEntry("momentum_observer", plotters.plot_momentum_observer),
        PlotEntry(
            "momentum_payload",
            plotters.plot_momentum_payload,
            available=has_payload_estimate,
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
