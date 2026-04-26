"""Plotter implementations split by domain.

Each `plotters/*.py` is independent; a higher-level pipeline (Phase 4)
selects which plotters to run for each log type.

Note: matplotlib backend must be configured (`layout.configure_backend(...)`)
*before* importing this package, since each plotter does `import matplotlib.pyplot`
at import time.
"""

from .device import (
    plot_device_positions,
    plot_device_velocities,
    print_device_statistics,
)
from .mpc import plot_mpc_solve_timing, print_mpc_timing_statistics
from .motor import (
    plot_motor_efforts,
    plot_motor_positions,
    plot_motor_velocities,
    print_motor_statistics,
)
from .robot import (
    plot_robot_commands,
    plot_robot_positions,
    plot_robot_task_position,
    plot_robot_task_tracking_error,
    plot_robot_torques,
    plot_robot_tracking_error,
    plot_robot_velocities,
    print_robot_statistics,
)
from .sensors import (
    plot_device_ft_output,
    plot_device_ft_output_auto,
    plot_device_sensor_comparison,
    plot_device_sensor_comparison_auto,
    plot_device_sensors,
    plot_device_sensors_raw,
    plot_sensor_barometer_combined,
    plot_sensor_tof_combined,
)
from .timing import (
    plot_timing_breakdown,
    plot_timing_histograms,
    plot_timing_total_and_jitter,
    print_timing_statistics,
)

__all__ = [
    # robot
    "plot_robot_commands",
    "plot_robot_positions",
    "plot_robot_task_position",
    "plot_robot_task_tracking_error",
    "plot_robot_torques",
    "plot_robot_tracking_error",
    "plot_robot_velocities",
    "print_robot_statistics",
    # motor
    "plot_motor_efforts",
    "plot_motor_positions",
    "plot_motor_velocities",
    "print_motor_statistics",
    # device
    "plot_device_positions",
    "plot_device_velocities",
    "print_device_statistics",
    # sensors
    "plot_device_ft_output",
    "plot_device_ft_output_auto",
    "plot_device_sensor_comparison",
    "plot_device_sensor_comparison_auto",
    "plot_device_sensors",
    "plot_device_sensors_raw",
    "plot_sensor_barometer_combined",
    "plot_sensor_tof_combined",
    # timing
    "plot_timing_breakdown",
    "plot_timing_histograms",
    "plot_timing_total_and_jitter",
    "print_timing_statistics",
    # mpc
    "plot_mpc_solve_timing",
    "print_mpc_timing_statistics",
]
