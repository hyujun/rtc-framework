"""
sim.launch.py — MuJoCo simulation launch for UR5e bringup
==========================================================

Launches MuJoCo simulator and RT controller manager for simulation.
Copied from ur5e_mujoco_sim/launch/mujoco_sim.launch.py with
package references updated to the new RTC package structure.

Usage:
  ros2 launch ur5e_bringup sim.launch.py
  ros2 launch ur5e_bringup sim.launch.py sim_mode:=sync_step
  ros2 launch ur5e_bringup sim.launch.py enable_viewer:=false
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
    """Keep at most max_sessions session folders."""
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
    """Setup function executed with launch context."""

    # ── Session directory ─────────────────────────────────────────────────────
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
    ur5e_robot_config = PathJoinSubstitution(
        [pkg_bringup, 'config', 'ur5e_robot.yaml'])

    # ur5e_hand_driver is optional
    hand_config = None
    try:
        hand_share = get_package_share_directory('ur5e_hand_driver')
        hand_config = os.path.join(hand_share, 'config', 'hand_udp_node.yaml')
    except Exception:
        pass

    # ── Build simulator parameters ────────────────────────────────────────────
    sim_params = [sim_config]
    sim_overrides = {}

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

    max_rtf = LaunchConfiguration('max_rtf').perform(context)
    if max_rtf != '':
        sim_overrides['max_rtf'] = float(max_rtf)

    # control_rate from ur5e_robot config
    import yaml
    try:
        bringup_share = get_package_share_directory('ur5e_bringup')
        robot_yaml_path = os.path.join(bringup_share, 'config', 'ur5e_robot.yaml')
        with open(robot_yaml_path, 'r') as f:
            robot_yaml = yaml.safe_load(f)
        control_rate = (robot_yaml.get('/**', {})
                        .get('ros__parameters', {})
                        .get('control_rate', 500.0))
        sim_overrides['control_rate'] = float(control_rate)
    except Exception:
        sim_overrides['control_rate'] = 500.0

    if sim_overrides:
        sim_params.append(sim_overrides)

    # ── Build controller parameters ───────────────────────────────────────────
    ctrl_params = [ur5e_robot_config, sim_config]
    if hand_config is not None:
        ctrl_params.append(hand_config)
    ctrl_overrides = {}

    ctrl_overrides['log_dir'] = session_dir

    use_fake_hand = LaunchConfiguration('use_fake_hand').perform(context)
    if use_fake_hand.lower() in ('true', '1', 'yes'):
        ctrl_overrides['use_fake_hand'] = True

    # Fake hand response from MuJoCo config
    try:
        sim_share = get_package_share_directory('rtc_mujoco_sim')
        sim_yaml_path = os.path.join(sim_share, 'config', 'mujoco_simulator.yaml')
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

    actions = [set_session_dir]

    # ── Node 1: MuJoCo Simulator ──────────────────────────────────────────────
    mujoco_node = Node(
        package='rtc_mujoco_sim',
        executable='mujoco_simulator_node',
        name='mujoco_simulator',
        output='screen',
        emulate_tty=True,
        parameters=sim_params,
    )

    # ── Node 2: RT Controller Manager ─────────────────────────────────────────
    rt_controller_node = Node(
        package='rtc_controller_manager',
        executable='rt_controller',
        name='rt_controller',
        output='screen',
        emulate_tty=True,
        parameters=ctrl_params,
    )

    actions.extend([mujoco_node, rt_controller_node])
    return actions


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='Override model_path from YAML')

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode', default_value='',
        description='Override sim_mode: "free_run" or "sync_step"')

    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer', default_value='',
        description='Override enable_viewer from YAML')

    max_rtf_arg = DeclareLaunchArgument(
        'max_rtf', default_value='',
        description='Maximum Real-Time Factor')

    max_log_sessions_arg = DeclareLaunchArgument(
        'max_log_sessions', default_value='10',
        description='Max session folders to keep')

    use_cpu_affinity_arg = DeclareLaunchArgument(
        'use_cpu_affinity', default_value='true',
        description='Enable CPU shield and core pinning')

    use_fake_hand_arg = DeclareLaunchArgument(
        'use_fake_hand', default_value='false',
        description='Use fake hand echo-back mode')

    return LaunchDescription([
        model_path_arg,
        sim_mode_arg,
        enable_viewer_arg,
        max_rtf_arg,
        max_log_sessions_arg,
        use_cpu_affinity_arg,
        use_fake_hand_arg,
        OpaqueFunction(function=launch_setup),
    ])
