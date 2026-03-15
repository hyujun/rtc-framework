# ur_control.launch.py - v2 (core allocation optimized)
#
# Core allocation optimizations applied:
#   C) UR driver process pinned to Core 0-1 via delayed taskset (use_cpu_affinity:=true)
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
    """colcon workspace 기반 logging_data 루트 경로 결정."""
    try:
        share_dir = get_package_share_directory('ur5e_rt_controller')
        ws_dir = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(share_dir))))
        return os.path.join(ws_dir, 'logging_data')
    except Exception:
        return os.path.expanduser('~/ros2_ws/ur5e_ws/logging_data')


def _cleanup_old_sessions(logging_root, max_sessions):
    """YYMMDD_HHMM 패턴 세션 폴더를 max_sessions 개수 이하로 유지."""
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
    # ROS 2 Jazzy renamed use_fake_hardware → use_mock_hardware.
    # Accept either flag; if either is 'true', enable mock hardware.
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
            'launch_rviz': 'false',
        }.items()
    )
    return [ur_driver_launch]


def generate_launch_description():
    # ── 세션 디렉토리 생성 (YYMMDD_HHMM) ─────────────────────────────────────
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

    # ROS 2 Jazzy ur_robot_driver renamed use_fake_hardware → use_mock_hardware.
    # Both are accepted for backwards compatibility.
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
        FindPackageShare('ur5e_rt_controller'),
        'config',
        'ur5e_rt_controller.yaml'
    ])

    # Status Monitor 설정 (ur5e_status_monitor 패키지 소유)
    status_monitor_config = PathJoinSubstitution([
        FindPackageShare('ur5e_status_monitor'),
        'config',
        'ur5e_status_monitor.yaml'
    ])

    # Hand UDP 설정 (ur5e_hand_udp 패키지 소유)
    hand_udp_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_udp'),
        'config',
        'hand_udp_node.yaml'
    ])

    cyclone_dds_xml = PathJoinSubstitution([
        FindPackageShare('ur5e_rt_controller'),
        'config',
        'cyclone_dds.xml'
    ])

    # ── [방안 E] CycloneDDS thread restriction ─────────────────────────────────
    # Restricts DDS internal recv/send threads to Core 0-1, preventing them
    # from preempting RT threads on Core 2-3.
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

    # ── [방안 C] UR driver CPU pinning ─────────────────────────────────────────
    # Applies taskset to pin ur_ros2_driver process to Core 0-1 after startup.
    # Delayed 3 s to allow the UR driver process to fully start before pinning.
    # Only runs when use_cpu_affinity:=true (default).
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

    # ── RT controller node ─────────────────────────────────────────────────────
    rt_controller_node = Node(
        package='ur5e_rt_controller',
        executable='rt_controller',
        name='rt_controller',
        output='screen',
        parameters=[
            ur_control_config,
            status_monitor_config,
            hand_udp_config,
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
        ur_driver_launch_action,
        pin_ur_driver,
        rt_controller_node,
    ])
