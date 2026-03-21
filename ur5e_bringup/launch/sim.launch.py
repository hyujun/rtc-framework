"""
sim.launch.py — MuJoCo simulation launch file for UR5e
======================================================

Usage:
  # Default (uses YAML config)
  ros2 launch ur5e_bringup sim.launch.py

  # sync_step mode override
  ros2 launch ur5e_bringup sim.launch.py sim_mode:=sync_step

  # Headless mode (no display)
  ros2 launch ur5e_bringup sim.launch.py enable_viewer:=false

  # External Menagerie model
  ros2 launch ur5e_bringup sim.launch.py \
      model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml

  # PD gain tuning
  ros2 launch ur5e_bringup sim.launch.py kp:=10.0 kd:=1.0

  # max_rtf override
  ros2 launch ur5e_bringup sim.launch.py max_rtf:=10.0

Nodes launched:
  1. mujoco_simulator_node  — MuJoCo physics simulator (replaces UR driver)
  2. rt_controller          — 500Hz controller (unchanged)
"""

import os
import re
import shutil
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


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


def launch_setup(context, *args, **kwargs):
    """Setup function executed with launch context for conditional parameter loading."""

    # ── Session directory (YYMMDD_HHMM) ──────────────────────────────────────
    logging_root = _resolve_logging_root()
    session_ts = datetime.now().strftime('%y%m%d_%H%M')
    session_dir = os.path.join(logging_root, session_ts)
    for sub in ('controller', 'monitor', 'hand', 'sim', 'plots', 'motions'):
        os.makedirs(os.path.join(session_dir, sub), exist_ok=True)

    max_sessions = int(
        LaunchConfiguration('max_log_sessions').perform(context) or '10')
    _cleanup_old_sessions(logging_root, max_sessions)

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_sim = FindPackageShare('rtc_mujoco_sim')
    pkg_bringup = FindPackageShare('ur5e_bringup')

    sim_config = PathJoinSubstitution(
        [pkg_sim,  'config', 'mujoco_simulator.yaml'])
    ctrl_config = PathJoinSubstitution(
        [pkg_bringup, 'config', 'ur5e_robot.yaml'])

    # ur5e_hand_driver is optional — may not be built in sim-only installs
    hand_config = None
    try:
        hand_share = get_package_share_directory('ur5e_hand_driver')
        hand_config = os.path.join(hand_share, 'config', 'hand_udp_node.yaml')
    except Exception:
        pass

    # ── Build simulator parameters (YAML first, then conditional overrides) ───
    sim_params = [sim_config]
    sim_overrides = {}

    # Check each launch argument - only add to overrides if explicitly provided
    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path != '':
        sim_overrides['model_path'] = model_path

    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    if sim_mode != '':
        sim_overrides['sim_mode'] = sim_mode

    enable_viewer = LaunchConfiguration('enable_viewer').perform(context)
    if enable_viewer != '':
        sim_overrides['enable_viewer'] = enable_viewer.lower() in (
            'true', '1', 'yes')

    publish_decimation = LaunchConfiguration(
        'publish_decimation').perform(context)
    if publish_decimation != '':
        sim_overrides['publish_decimation'] = int(publish_decimation)

    sync_timeout_ms = LaunchConfiguration('sync_timeout_ms').perform(context)
    if sync_timeout_ms != '':
        sim_overrides['sync_timeout_ms'] = float(sync_timeout_ms)

    max_rtf = LaunchConfiguration('max_rtf').perform(context)
    if max_rtf != '':
        sim_overrides['max_rtf'] = float(max_rtf)

    use_yaml_servo_gains = LaunchConfiguration(
        'use_yaml_servo_gains').perform(context)
    if use_yaml_servo_gains != '':
        sim_overrides['use_yaml_servo_gains'] = (
            use_yaml_servo_gains.lower() in ('true', '1', 'yes'))

    # ── Fake hand response + control_rate ─────────────────────────────────────
    import yaml

    try:
        ctrl_yaml_path = os.path.join(
            get_package_share_directory('ur5e_bringup'),
            'config', 'ur5e_robot.yaml')
        with open(ctrl_yaml_path, 'r') as f:
            ctrl_yaml = yaml.safe_load(f)
        control_rate = (ctrl_yaml.get('/**', {})
                        .get('ros__parameters', {})
                        .get('control_rate', 500.0))
        sim_overrides['control_rate'] = float(control_rate)
    except Exception:
        sim_overrides['control_rate'] = 500.0

    if sim_overrides:
        sim_params.append(sim_overrides)

    # ── Build controller parameters (YAML + overrides + launch args) ──────────
    ctrl_params = [ctrl_config, sim_config]
    if hand_config is not None:
        ctrl_params.append(hand_config)
    ctrl_overrides = {}

    kp = LaunchConfiguration('kp').perform(context)
    if kp != '':
        ctrl_overrides['kp'] = float(kp)

    kd = LaunchConfiguration('kd').perform(context)
    if kd != '':
        ctrl_overrides['kd'] = float(kd)

    ctrl_overrides['log_dir'] = session_dir

    use_fake_hand = LaunchConfiguration('use_fake_hand').perform(context)
    if use_fake_hand.lower() in ('true', '1', 'yes'):
        ctrl_overrides['use_fake_hand'] = True

    # Fake hand response: pass MuJoCo config to rt_controller
    sim_yaml_path = os.path.join(
        get_package_share_directory('rtc_mujoco_sim'),
        'config', 'mujoco_simulator.yaml')
    try:
        with open(sim_yaml_path, 'r') as f:
            sim_yaml = yaml.safe_load(f)
        fake_hand = (sim_yaml.get('mujoco_simulator', {})
                     .get('ros__parameters', {})
                     .get('fake_hand_response', {}))
        if fake_hand.get('enable', False):
            ctrl_overrides['hand_sim_enabled'] = True
            ctrl_overrides['hand_command_topic'] = fake_hand.get(
                'command_topic', '/hand/command')
            ctrl_overrides['hand_state_topic'] = fake_hand.get(
                'state_topic', '/hand/joint_states')
            ctrl_overrides['target_ip'] = ''
            ctrl_overrides['target_port'] = 0
    except Exception:
        pass

    if ctrl_overrides:
        ctrl_params.append(ctrl_overrides)

    # ── Environment variables ─────────────────────────────────────────────────
    set_session_dir = SetEnvironmentVariable(
        name='UR5E_SESSION_DIR',
        value=session_dir
    )

    # ── CPU Shield (Tier 1 only for simulation) ───────────────────────────────
    use_affinity = LaunchConfiguration('use_cpu_affinity').perform(context)
    actions = [set_session_dir]

    if use_affinity.lower() in ('true', '1', 'yes'):
        enable_sim_cpu_shield = ExecuteProcess(
            cmd=[
                'bash', '-c',
                'SCRIPT_DIR="$(ros2 pkg prefix rtc_controller_manager 2>/dev/null)/lib/rtc_controller_manager" && '
                'if [ -f "$SCRIPT_DIR/cpu_shield.sh" ]; then '
                '  ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null); '
                '  if [ -z "$ISOLATED" ]; then '
                '    echo "[SIM] CPU shield not active — enabling sim mode Tier 1 isolation..."; '
                '    sudo "$SCRIPT_DIR/cpu_shield.sh" on --sim; '
                '  else '
                '    echo "[SIM] CPU shield already active: Core $ISOLATED isolated"; '
                '  fi; '
                'fi'
            ],
            output='screen',
        )
        actions.append(enable_sim_cpu_shield)

    # ── Node 1: MuJoCo Simulator ──────────────────────────────────────────────
    mujoco_node = Node(
        package='rtc_mujoco_sim',
        executable='mujoco_simulator_node',
        name='mujoco_simulator',
        output='screen',
        emulate_tty=True,
        parameters=sim_params,
    )

    # ── Node 2: Custom Controller ─────────────────────────────────────────────
    rt_controller_node = Node(
        package='rtc_controller_manager',
        executable='rt_controller',
        name='rt_controller',
        output='screen',
        emulate_tty=True,
        parameters=ctrl_params,
    )

    actions.extend([mujoco_node, rt_controller_node])

    # ── MuJoCo sim_thread CPU pinning ─────────────────────────────────────────
    if use_affinity.lower() in ('true', '1', 'yes'):
        pin_mujoco_sim = TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'PHYS=$(lscpu -p=Core,Socket 2>/dev/null | grep -v "^#" | sort -u | wc -l); '
                        'if [ "$PHYS" -ge 8 ]; then '
                        '  PID=$(pgrep -nf mujoco_simulator_node); '
                        '  if [ -n "$PID" ]; then '
                        '    SIM_CORE=$((PHYS >= 10 ? 7 : 6)); '
                        '    taskset -cp $SIM_CORE "$PID" && '
                        '    echo "[SIM] mujoco_simulator (PID=$PID) pinned to Core $SIM_CORE"; '
                        '  fi; '
                        'fi'
                    ],
                    output='screen',
                )
            ]
        )
        actions.append(pin_mujoco_sim)

    return actions


def generate_launch_description():
    # ── Launch arguments with empty defaults (YAML values take precedence) ───
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description=(
            'Override model_path from YAML. '
            'Empty -> use YAML value (ur5e_description/scene.xml). '
            'Absolute path -> use specified MuJoCo scene.xml'
        ),
    )

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='',
        description=(
            'Override sim_mode from YAML. '
            'Empty -> use YAML value (free_run). '
            'Options: "free_run" (max speed) or "sync_step" (1:1 sync)'
        ),
    )

    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='',
        description=(
            'Override enable_viewer from YAML. '
            'Empty -> use YAML value (true). '
            'Set to "false" for headless mode'
        ),
    )

    publish_decimation_arg = DeclareLaunchArgument(
        'publish_decimation',
        default_value='',
        description=(
            'Override publish_decimation from YAML. '
            'Empty -> use YAML value (1). '
            'free_run only: publish /joint_states every N physics steps'
        ),
    )

    sync_timeout_ms_arg = DeclareLaunchArgument(
        'sync_timeout_ms',
        default_value='',
        description=(
            'Override sync_timeout_ms from YAML. '
            'Empty -> use YAML value (50.0). '
            'sync_step only: command wait timeout in milliseconds'
        ),
    )

    max_rtf_arg = DeclareLaunchArgument(
        'max_rtf',
        default_value='',
        description=(
            'Override max_rtf from YAML. '
            'Empty -> use YAML value (1.0). '
            'Maximum Real-Time Factor (0.0 = unlimited). '
            'Examples: 1.0 for real-time, 10.0 for 10x speed'
        ),
    )

    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='',
        description=(
            'Override kp from YAML. '
            'Empty -> use YAML value. '
            'PD controller proportional gain'
        ),
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='',
        description=(
            'Override kd from YAML. '
            'Empty -> use YAML value. '
            'PD controller derivative gain'
        ),
    )

    use_yaml_servo_gains_arg = DeclareLaunchArgument(
        'use_yaml_servo_gains',
        default_value='',
        description=(
            'Override use_yaml_servo_gains from YAML. '
            'Empty -> use YAML value (false). '
            'true: servo_kp/kd gains from YAML, false: XML gainprm/biasprm'
        ),
    )

    max_log_sessions_arg = DeclareLaunchArgument(
        'max_log_sessions',
        default_value='10',
        description='Maximum number of session folders to keep (YYMMDD_HHMM)',
    )

    use_cpu_affinity_arg = DeclareLaunchArgument(
        'use_cpu_affinity',
        default_value='true',
        description=(
            'Enable CPU shield (Tier 1 isolation) and MuJoCo core pinning. '
            'Set false for CI or environments without sudo.'
        )
    )

    use_fake_hand_arg = DeclareLaunchArgument(
        'use_fake_hand',
        default_value='false',
        description=(
            'Use fake hand echo-back mode (no UDP/ROS communication). '
            'Command is echoed back as position state immediately.'
        )
    )

    return LaunchDescription([
        # Arguments
        model_path_arg,
        sim_mode_arg,
        enable_viewer_arg,
        publish_decimation_arg,
        sync_timeout_ms_arg,
        max_rtf_arg,
        kp_arg,
        kd_arg,
        use_yaml_servo_gains_arg,
        max_log_sessions_arg,
        use_cpu_affinity_arg,
        use_fake_hand_arg,
        # Nodes (via OpaqueFunction for conditional parameter loading)
        OpaqueFunction(function=launch_setup),
    ])
