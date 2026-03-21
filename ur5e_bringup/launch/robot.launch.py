# robot.launch.py - UR5e robot bringup
#
# Launches the UR robot driver and the RT controller manager.
# Copied from ur5e_rt_controller/launch/ur_control.launch.py with
# package references updated to the new RTC package structure.

import os
import re
import shutil
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_logging_root():
    """colcon workspace logging_data root path."""
    try:
        share_dir = get_package_share_directory('ur5e_bringup')
        ws_dir = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(share_dir))))
        return os.path.join(ws_dir, 'logging_data')
    except Exception:
        return os.path.expanduser('~/ros2_ws/ur5e_ws/logging_data')


def _cleanup_old_sessions(logging_root, max_sessions):
    """Keep at most max_sessions session folders (YYMMDD_HHMM pattern)."""
    if not os.path.isdir(logging_root):
        return
    pattern = re.compile(r'^\d{6}_\d{4}$')
    dirs = sorted([
        d for d in os.listdir(logging_root)
        if os.path.isdir(os.path.join(logging_root, d)) and pattern.match(d)
    ])
    while len(dirs) > max_sessions:
        oldest = os.path.join(logging_root, dirs.pop(0))
        shutil.rmtree(oldest, ignore_errors=True)


def _launch_setup(context):
    """Resolve launch arguments and build actions that need runtime values."""
    use_mock = context.launch_configurations.get('use_mock_hardware', 'false')
    use_fake = context.launch_configurations.get('use_fake_hardware', 'false')
    mock_enabled = 'true' if use_mock == 'true' or use_fake == 'true' else 'false'

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_mock_hardware': mock_enabled,
            'use_fake_hardware': mock_enabled,
            'launch_rviz': 'false',
        }.items()
    )
    return [ur_driver_launch]


def generate_launch_description():
    # ── Session directory (YYMMDD_HHMM) ──────────────────────────────────────
    logging_root = _resolve_logging_root()
    session_ts = datetime.now().strftime('%y%m%d_%H%M')
    session_dir = os.path.join(logging_root, session_ts)
    for sub in ('controller', 'monitor', 'hand', 'sim', 'plots', 'motions'):
        os.makedirs(os.path.join(session_dir, sub), exist_ok=True)
    _cleanup_old_sessions(logging_root, 10)

    # ── Arguments ──────────────────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the UR robot'
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for testing (Jazzy)'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='[Deprecated] Alias kept for compatibility'
    )

    use_cpu_affinity_arg = DeclareLaunchArgument(
        'use_cpu_affinity',
        default_value='true',
        description='Pin UR driver process to Core 0-1 via taskset'
    )

    # ── Paths ──────────────────────────────────────────────────────────────────
    ur5e_robot_config = PathJoinSubstitution([
        FindPackageShare('ur5e_bringup'),
        'config',
        'ur5e_robot.yaml'
    ])

    hand_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_driver'),
        'config',
        'hand_udp_node.yaml',
    ])

    ft_inferencer_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_driver'),
        'config',
        'fingertip_ft_inferencer.yaml'
    ])

    set_session_dir = SetEnvironmentVariable(
        name='UR5E_SESSION_DIR',
        value=session_dir
    )

    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    # ── UR robot driver launch ────────────────────────────────────────────────
    ur_driver_launch_action = OpaqueFunction(function=_launch_setup)

    # ── CPU pinning for UR driver ─────────────────────────────────────────────
    pin_ur_driver = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'PID=$(pgrep -nf ur_ros2_driver) && [ -n "$PID" ] && '
                    'taskset -cp 0-1 "$PID" && '
                    'echo "[RT] ur_ros2_driver (PID=$PID) pinned to Core 0-1" || '
                    'echo "[RT] WARNING: ur_ros2_driver not found"'
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_cpu_affinity'))
            )
        ]
    )

    # ── RT controller manager node ────────────────────────────────────────────
    rt_controller_node = Node(
        package='rtc_controller_manager',
        executable='rt_controller',
        name='rt_controller',
        output='screen',
        parameters=[
            ur5e_robot_config,
            hand_config,
            ft_inferencer_config,
            {
                'log_dir': session_dir,
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_ip_arg,
        use_mock_hardware_arg,
        use_fake_hardware_arg,
        use_cpu_affinity_arg,
        set_session_dir,
        set_rmw,
        ur_driver_launch_action,
        pin_ur_driver,
        rt_controller_node,
    ])
