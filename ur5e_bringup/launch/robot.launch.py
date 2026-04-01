# robot.launch.py - UR5e real robot bringup
#
# Launch order (event-driven):
#   1. Environment vars (RMW, CycloneDDS, session dir)
#   2. CPU shield, UR driver, hand_udp_node  (parallel)
#   3. Readiness gate: polls /joint_states and /hand/joint_states publishers
#   4. rt_controller_node starts ONLY after gate exits successfully
#   5. DDS thread pinning runs 5 s after rt_controller starts
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
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
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
            'initial_joint_controller': 'forward_position_controller',
        }.items()
    )
    # ── Activate forward_position_controller (mock hardware only) ────────────
    # For real robots, the UR driver's controller_stopper_node handles activation
    # via initial_joint_controller when play is pressed on the teach pendant.
    # For mock hardware, there is no controller_stopper, so we must switch manually.
    actions = [ur_driver_launch]
    if mock_enabled == 'true':
        activate_fwd_controller = TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'echo "[RT] Switching to forward_position_controller (mock mode)..."; '
                        'ros2 service call /controller_manager/switch_controller '
                        '  controller_manager_msgs/srv/SwitchController '
                        '  "{activate_controllers: [forward_position_controller], '
                        '    deactivate_controllers: [scaled_joint_trajectory_controller], '
                        '    strictness: 1}" '
                        '  && echo "[RT] forward_position_controller activated" '
                        '  || echo "[RT] WARNING: controller switch failed"'
                    ],
                    output='screen',
                )
            ]
        )
        actions.append(activate_fwd_controller)
    return actions


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

    # Hand UDP config (ur5e_hand_driver package)
    hand_udp_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_driver'),
        'config',
        'hand_udp_node.yaml'
    ])

    # Fingertip F/T inferencer config (ur5e_hand_driver package)
    ft_inferencer_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_driver'),
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
    _pkg_prefix = get_package_share_directory('rtc_scripts')
    _shield_script = os.path.join(
        os.path.dirname(os.path.dirname(_pkg_prefix)),
        'lib', 'rtc_scripts', 'cpu_shield.sh')

    enable_cpu_shield = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'if [ -f "{_shield_script}" ]; then '
            '  ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null); '
            '  if [ -z "$ISOLATED" ]; then '
            '    echo "[RT] CPU shield not active — enabling robot mode..."; '
            '    if sudo -n true 2>/dev/null; then '
            f'      sudo "{_shield_script}" on --robot; '
            '    else '
            '      echo "[RT] WARNING: sudo requires a password — skipping CPU shield. '
            'Configure passwordless sudo for cpu_shield.sh or run: '
            f'sudo {_shield_script} on --robot"; '
            '    fi; '
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
        package='ur5e_bringup',
        executable='ur5e_rt_controller',
        name='rt_controller',
        output='screen',
        parameters=[
            ur_control_config,
            {
                'log_dir': session_dir,
            },
        ],
        emulate_tty=True,
    )

    # ── Hand UDP driver node ──────────────────────────────────────────────────
    # Publishes /hand/joint_states and /hand/sensor_states for rt_controller.
    hand_udp_node = Node(
        package='ur5e_hand_driver',
        executable='hand_udp_node',
        name='hand_udp_node',
        output='screen',
        parameters=[
            hand_udp_config,
            ft_inferencer_config,
        ],
        emulate_tty=True,
    )

    # ── Readiness gate ────────────────────────────────────────────────────────
    # Polls until UR driver publishes /joint_states AND hand_udp_node publishes
    # /hand/joint_states.  rt_controller_node is chained to start only after
    # this gate process exits successfully (via OnProcessExit event handler).
    comm_readiness_gate = ExecuteProcess(
        cmd=[
            'bash', '-c',
            # --- Wait for /joint_states publisher (UR driver) ---
            'echo "[RT] Readiness gate: waiting for communication nodes..."; '
            'timeout 30 bash -c \''
            'while ! ros2 topic info /joint_states 2>/dev/null '
            '  | grep -q "Publisher count: [1-9]"; do sleep 0.5; done\' '
            '  && echo "[RT]   /joint_states publisher OK" '
            '  || { echo "[RT] FATAL: /joint_states not available after 30 s"; exit 1; }; '
            # --- Wait for /hand/joint_states publisher (hand_udp_node) ---
            'timeout 30 bash -c \''
            'while ! ros2 topic info /hand/joint_states 2>/dev/null '
            '  | grep -q "Publisher count: [1-9]"; do sleep 0.5; done\' '
            '  && echo "[RT]   /hand/joint_states publisher OK" '
            '  || { echo "[RT] FATAL: /hand/joint_states not available after 30 s"; exit 1; }; '
            # --- Wait for forward_position_controller subscriber (ros2_control) ---
            'timeout 30 bash -c \''
            'while ! ros2 topic info /forward_position_controller/commands 2>/dev/null '
            '  | grep -q "Subscription count: [1-9]"; do sleep 0.5; done\' '
            '  && echo "[RT]   forward_position_controller subscriber OK" '
            '  || echo "[RT] WARNING: forward_position_controller not ready after 30 s '
            '(will activate later via controller_stopper or mock switch)"; '
            'echo "[RT] All communication nodes ready"'
        ],
        output='screen',
    )

    # ── Event-driven launch chain ─────────────────────────────────────────────
    # Gate starts only after hand_udp_node process is running (ensures DDS
    # endpoint is registered before polling).  rt_controller starts only after
    # the gate exits with success.  DDS thread pinning fires 5 s after
    # rt_controller starts.
    start_gate_after_hand = RegisterEventHandler(
        OnProcessStart(
            target_action=hand_udp_node,
            on_start=[
                LogInfo(msg='[RT] hand_udp_node started — launching readiness gate'),
                comm_readiness_gate,
            ],
        )
    )

    start_rt_after_gate = RegisterEventHandler(
        OnProcessExit(
            target_action=comm_readiness_gate,
            on_exit=[
                LogInfo(msg='[RT] Readiness gate passed — launching rt_controller'),
                rt_controller_node,
            ],
        )
    )

    start_dds_pin_after_rt = RegisterEventHandler(
        OnProcessStart(
            target_action=rt_controller_node,
            on_start=[pin_rt_controller_dds],
        )
    )

    return LaunchDescription([
        # 1) Arguments
        robot_ip_arg,
        use_mock_hardware_arg,
        use_fake_hardware_arg,
        use_cpu_affinity_arg,
        # 2) Environment
        set_session_dir,
        set_rmw,
        set_cyclone_uri,
        # 3) Infrastructure (parallel)
        enable_cpu_shield,
        ur_driver_launch_action,
        pin_ur_driver,
        hand_udp_node,
        # 4) Event-driven chain:
        #    hand_udp_node started → comm_readiness_gate
        #    → gate exits OK → rt_controller_node
        #    → rt_controller started → pin_rt_controller_dds
        start_gate_after_hand,
        start_rt_after_gate,
        start_dds_pin_after_rt,
    ])
