# robot.launch.py - UR5e real robot bringup
#
# Core allocation optimizations applied:
#   C) UR driver process pinned to Core 0-1 via delayed taskset (use_cpu_affinity:=true)
#   D) rt_controller DDS threads pinned to Core 0-1 (prevents 100-350us jitter on Jazzy)
#   E) CycloneDDS threads restricted to Core 0-1 via CYCLONEDDS_URI env var

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
    # ROS 2 Jazzy renamed use_fake_hardware -> use_mock_hardware.
    # Accept either flag; if either is 'true', enable mock hardware.
    use_mock = context.launch_configurations.get('use_mock_hardware', 'false')
    use_fake = context.launch_configurations.get('use_fake_hardware', 'false')
    mock_enabled = 'true' if use_mock == 'true' or use_fake == 'true' else 'false'

    # Humble uses 'use_fake_hardware', Jazzy+ uses 'use_mock_hardware'.
    # Pass both so the correct one is picked up regardless of ROS distro.
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
        description='[Deprecated — use use_mock_hardware] Alias kept for compatibility'
    )

    use_cpu_affinity_arg = DeclareLaunchArgument(
        'use_cpu_affinity',
        default_value='true',
        description=(
            'Pin UR driver process to Core 0-1 via taskset (3 s after launch). '
            'Set false when running with fake hardware or in CI.'
        )
    )

    # ── Paths ──────────────────────────────────────────────────────────────────
    ur_control_config = PathJoinSubstitution([
        FindPackageShare('ur5e_bringup'),
        'config',
        'ur5e_robot.yaml'
    ])

    # Status Monitor config (ur5e_status_monitor package)
    status_monitor_config = PathJoinSubstitution([
        FindPackageShare('ur5e_status_monitor'),
        'config',
        'ur5e_status_monitor.yaml'
    ])

    # Hand UDP config (ur5e_hand_udp package)
    hand_udp_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_udp'),
        'config',
        'hand_udp_node.yaml'
    ])

    # Fingertip F/T inferencer config (ur5e_hand_udp package)
    ft_inferencer_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_udp'),
        'config',
        'fingertip_ft_inferencer.yaml'
    ])

    cyclone_dds_xml = PathJoinSubstitution([
        FindPackageShare('rtc_controller_manager'),
        'config',
        'cyclone_dds.xml'
    ])

    # ── CycloneDDS thread restriction ─────────────────────────────────────────
    set_cyclone_uri = SetEnvironmentVariable(
        name='CYCLONEDDS_URI',
        value=['file://', cyclone_dds_xml]
    )

    set_session_dir = SetEnvironmentVariable(
        name='UR5E_SESSION_DIR',
        value=session_dir
    )

    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    # ── UR robot driver launch (via OpaqueFunction for mock_hardware compat) ──
    ur_driver_launch_action = OpaqueFunction(function=_launch_setup)

    # ── CPU Shield ────────────────────────────────────────────────────────────
    _pkg_prefix = get_package_share_directory('rtc_controller_manager')
    _shield_script = os.path.join(
        os.path.dirname(os.path.dirname(_pkg_prefix)),
        'lib', 'rtc_controller_manager', 'cpu_shield.sh')

    enable_cpu_shield = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'if [ -f "{_shield_script}" ]; then '
            '  ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null); '
            '  if [ -z "$ISOLATED" ]; then '
            '    echo "[RT] CPU shield not active — enabling robot mode..."; '
            f'    sudo "{_shield_script}" on --robot; '
            '  else '
            '    echo "[RT] CPU shield already active: Core $ISOLATED isolated"; '
            '  fi; '
            'else '
            f'  echo "[RT] WARNING: cpu_shield.sh not found: {_shield_script}"; '
            'fi'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cpu_affinity'))
    )

    # ── UR driver CPU pinning ─────────────────────────────────────────────────
    pin_ur_driver = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'PID=$(pgrep -nf ur_ros2_driver) && [ -n "$PID" ] && '
                    'taskset -cp 0-1 "$PID" && '
                    'echo "[RT] ur_ros2_driver (PID=$PID) pinned to Core 0-1" || '
                    'echo "[RT] WARNING: ur_ros2_driver not found — CPU pinning skipped"'
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_cpu_affinity'))
            )
        ]
    )

    # ── rt_controller DDS thread pinning ──────────────────────────────────────
    pin_rt_controller_dds = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'PID=$(pgrep -nf "rt_controller"); '
                    'if [ -z "$PID" ]; then '
                    '  echo "[RT] WARNING: rt_controller not found — DDS thread pinning skipped"; '
                    '  exit 0; '
                    'fi; '
                    'taskset -cp 0-1 "$PID" 2>/dev/null; '
                    'PINNED=0; '
                    'for TID in $(ls /proc/$PID/task/ 2>/dev/null); do '
                    '  COMM=$(cat /proc/$PID/task/$TID/comm 2>/dev/null || echo ""); '
                    '  POLICY=$(chrt -p $TID 2>/dev/null | grep -o "SCHED_FIFO" || echo ""); '
                    '  if [ -n "$POLICY" ]; then continue; fi; '
                    '  taskset -cp 0-1 "$TID" 2>/dev/null && PINNED=$((PINNED+1)); '
                    'done; '
                    'echo "[RT] rt_controller (PID=$PID): $PINNED DDS/aux threads pinned to Core 0-1"'
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_cpu_affinity'))
            )
        ]
    )

    # ── RT controller node ─────────────────────────────────────────────────────
    rt_controller_node = Node(
        package='rtc_controller_manager',
        executable='rt_controller',
        name='rt_controller',
        output='screen',
        parameters=[
            ur_control_config,
            status_monitor_config,
            hand_udp_config,
            ft_inferencer_config,
            {
                'log_dir': session_dir,
                'status_monitor.log_output_dir':
                    os.path.join(session_dir, 'monitor'),
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
        set_cyclone_uri,
        enable_cpu_shield,
        ur_driver_launch_action,
        pin_ur_driver,
        rt_controller_node,
        pin_rt_controller_dds,
    ])
