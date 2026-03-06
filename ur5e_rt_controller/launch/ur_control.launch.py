# ur_control.launch.py - v2 (core allocation optimized)
#
# Core allocation optimizations applied:
#   C) UR driver process pinned to Core 0-1 via delayed taskset (use_cpu_affinity:=true)
#   E) CycloneDDS threads restricted to Core 0-1 via CYCLONEDDS_URI env var

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ── Log directory (colcon workspace log/) ──────────────────────────────────
    log_dir = os.path.expanduser('~/ur_ws/log')
    log_path = os.path.join(log_dir, 'ur5e_control_log.csv')

    # ── Arguments ──────────────────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the UR robot'
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing'
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

    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    # ── UR robot driver launch ─────────────────────────────────────────────────
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
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
        }.items()
    )

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

    # ── Custom controller node ─────────────────────────────────────────────────
    custom_controller_node = Node(
        package='ur5e_rt_controller',
        executable='custom_controller',
        name='custom_controller',
        output='screen',
        parameters=[ur_control_config, {'log_path': log_path}],
        emulate_tty=True,
    )

    # ── Data health monitor ────────────────────────────────────────────────────
    monitor_node = Node(
        package='ur5e_tools',
        executable='monitor_data_health.py',
        name='data_health_monitor',
        output='screen',
        parameters=[{
            'check_rate': 10.0,
            'timeout_threshold': 0.2,
        }],
    )

    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        use_cpu_affinity_arg,
        set_rmw,
        set_cyclone_uri,
        ur_driver_launch,
        pin_ur_driver,
        custom_controller_node,
        monitor_node,
    ])
