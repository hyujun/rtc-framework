"""
sim.launch.py — MuJoCo simulation launch file for UR5e
======================================================

Usage:
  # Default (uses YAML config)
  ros2 launch ur5e_bringup sim.launch.py

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
  2. rt_controller          — 500Hz controller (CV-based wakeup in sim mode)
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.actions import EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

from rtc_tools.utils.session_dir import (
    cleanup_old_sessions,
    create_session_dir,
    resolve_logging_root,
)


def launch_setup(context, *args, **kwargs):
    """Setup function executed with launch context for conditional parameter loading."""

    # ── Session directory (YYMMDD_HHMM) ──────────────────────────────────────
    logging_root = resolve_logging_root()
    session_dir = create_session_dir(logging_root)

    max_sessions = int(
        LaunchConfiguration('max_log_sessions').perform(context) or '10')
    cleanup_old_sessions(logging_root, max_sessions)

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_sim = FindPackageShare('rtc_mujoco_sim')
    pkg_bringup = FindPackageShare('ur5e_bringup')

    sim_config = PathJoinSubstitution(
        [pkg_sim,  'config', 'mujoco_simulator.yaml'])
    ctrl_config = PathJoinSubstitution(
        [pkg_bringup, 'config', 'ur5e_sim.yaml'])

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

    enable_viewer = LaunchConfiguration('enable_viewer').perform(context)
    if enable_viewer != '':
        sim_overrides['enable_viewer'] = enable_viewer.lower() in (
            'true', '1', 'yes')

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
            'config', 'ur5e_sim.yaml')
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

    # Optional: override initial_controller (Phase 4B: select demo_wbc_controller)
    initial_controller = LaunchConfiguration(
        'initial_controller').perform(context)
    if initial_controller != '':
        ctrl_overrides['initial_controller'] = initial_controller

    # Phase 5 + controller-override follow-up: `enable_mpc` drives the
    # `demo_wbc_controller.mpc.enabled` ROS parameter, which rtc_controller_
    # manager's `ApplyControllerParamOverrides` helper writes into the
    # YAML::Node handed to `LoadConfig`. The runtime gains topic (index 7)
    # can also toggle MPC on/off dynamically without restarting the launch.
    enable_mpc = LaunchConfiguration('enable_mpc').perform(context)
    if enable_mpc.lower() in ('true', '1', 'yes'):
        ctrl_overrides['demo_wbc_controller.mpc.enabled'] = True
    elif enable_mpc.lower() in ('false', '0', 'no'):
        ctrl_overrides['demo_wbc_controller.mpc.enabled'] = False

    # Phase 7b: `mpc_engine` selects between the Phase 5 MockMPCThread
    # placeholder and the Phase 7 HandlerMPCThread (real Aligator ProxDDP
    # via MPCFactory + GraspPhaseManager). Default "" leaves the YAML's
    # `mpc.engine: "mock"` untouched.
    mpc_engine = LaunchConfiguration('mpc_engine').perform(context)
    if mpc_engine.strip() != '':
        engine_str = mpc_engine.strip().lower()
        if engine_str not in ('mock', 'handler'):
            raise RuntimeError(
                f"Invalid mpc_engine='{mpc_engine}'. "
                "Must be 'mock' or 'handler' (or empty to use YAML default)."
            )
        ctrl_overrides['demo_wbc_controller.mpc.engine'] = engine_str

    if ctrl_overrides:
        ctrl_params.append(ctrl_overrides)

    # ── Environment variables ─────────────────────────────────────────────────
    # RTC_SESSION_DIR 우선, UR5E_SESSION_DIR 은 하위 호환을 위해 함께 세팅.
    set_session_dir = SetEnvironmentVariable(
        name='RTC_SESSION_DIR',
        value=session_dir
    )
    set_session_dir_legacy = SetEnvironmentVariable(
        name='UR5E_SESSION_DIR',
        value=session_dir
    )

    # ── CPU Shield (Tier 1 only for simulation) ───────────────────────────────
    use_affinity = LaunchConfiguration('use_cpu_affinity').perform(context)
    actions = [set_session_dir, set_session_dir_legacy]

    if use_affinity.lower() in ('true', '1', 'yes'):
        enable_sim_cpu_shield = ExecuteProcess(
            cmd=[
                'bash', '-c',
                'SCRIPT_DIR="$(ros2 pkg prefix rtc_scripts 2>/dev/null)/lib/rtc_scripts" && '
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

    # ── Node 1: MuJoCo Simulator (LifecycleNode) ────────────────────────────
    # `namespace=''` is required by launch_ros >= jazzy (keyword-only arg in
    # LifecycleNode.__init__); earlier ROS distros defaulted it implicitly.
    mujoco_node = LifecycleNode(
        package='rtc_mujoco_sim',
        executable='mujoco_simulator_node',
        name='mujoco_simulator',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=sim_params,
    )

    # ── Node 2: Custom Controller (LifecycleNode) ─────────────────────────
    rt_controller_node = LifecycleNode(
        package='ur5e_bringup',
        executable='ur5e_rt_controller',
        name='rt_controller',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=ctrl_params,
    )

    # ── Lifecycle chain: mujoco configure→activate → rt_controller configure→activate
    mujoco_auto_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mujoco_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda n: n == mujoco_node,
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )
    )
    chain_rt_after_mujoco = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mujoco_node,
            start_state='activating',
            goal_state='active',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda n: n == rt_controller_node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))],
        )
    )
    rt_auto_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=rt_controller_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda n: n == rt_controller_node,
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )
    )
    trigger_mujoco_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == mujoco_node,
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    actions.extend([
        mujoco_node, rt_controller_node,
        mujoco_auto_activate, chain_rt_after_mujoco, rt_auto_activate,
        trigger_mujoco_configure,
    ])

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

        # rt_controller DDS/aux threads → Core 0-1 (mirror of robot.launch.py).
        # ApplyThreadConfig() already pins SCHED_FIFO executors (rt_loop, sensor,
        # udp_recv); this timer only catches DDS-internal reader/writer threads
        # that are spawned outside our control.
        pin_rt_controller_dds = TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'PID=$(pgrep -nf "rt_controller"); '
                        'if [ -z "$PID" ]; then '
                        '  echo "[SIM] WARNING: rt_controller not found — DDS thread pinning skipped"; '
                        '  exit 0; '
                        'fi; '
                        'taskset -cp 0-1 "$PID" 2>/dev/null; '
                        'PINNED=0; '
                        'for TID in $(ls /proc/$PID/task/ 2>/dev/null); do '
                        '  POLICY=$(chrt -p $TID 2>/dev/null | grep -o "SCHED_FIFO" || echo ""); '
                        '  if [ -n "$POLICY" ]; then continue; fi; '
                        '  taskset -cp 0-1 "$TID" 2>/dev/null && PINNED=$((PINNED+1)); '
                        'done; '
                        'echo "[SIM] rt_controller (PID=$PID): $PINNED DDS/aux threads pinned to Core 0-1"'
                    ],
                    output='screen',
                )
            ]
        )
        actions.append(pin_rt_controller_dds)

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

    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='',
        description=(
            'Override enable_viewer from YAML. '
            'Empty -> use YAML value (true). '
            'Set to "false" for headless mode'
        ),
    )

    sync_timeout_ms_arg = DeclareLaunchArgument(
        'sync_timeout_ms',
        default_value='',
        description=(
            'Override sync_timeout_ms from YAML. '
            'Empty -> use YAML value (50.0). '
            'Command wait timeout per step in milliseconds'
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

    initial_controller_arg = DeclareLaunchArgument(
        'initial_controller',
        default_value='',
        description=(
            'Override initial controller name (e.g. demo_wbc_controller). '
            'Empty = use value from ur5e_sim.yaml.'
        )
    )

    enable_mpc_arg = DeclareLaunchArgument(
        'enable_mpc',
        default_value='',
        description=(
            'Enable the MPC thread in DemoWbcController (Phase 5). '
            'Takes effect only when initial_controller:=demo_wbc_controller. '
            'Empty = use demo_wbc_controller.yaml default. '
            'Runtime toggle is also available via gains index 7.'
        )
    )

    mpc_engine_arg = DeclareLaunchArgument(
        'mpc_engine',
        default_value='',
        description=(
            'Select MPC engine in DemoWbcController (Phase 7b): '
            '"mock" = Phase 5 MockMPCThread placeholder (default); '
            '"handler" = HandlerMPCThread + MPCFactory + GraspPhaseManager '
            '(real Aligator ProxDDP solve, requires phase_config.yaml + '
            'mpc_kinodynamics.yaml + mpc_fulldynamics.yaml in the package share). '
            'Empty = use demo_wbc_controller.yaml default.'
        )
    )

    return LaunchDescription([
        # Arguments
        model_path_arg,
        enable_viewer_arg,
        sync_timeout_ms_arg,
        max_rtf_arg,
        kp_arg,
        kd_arg,
        use_yaml_servo_gains_arg,
        max_log_sessions_arg,
        use_cpu_affinity_arg,
        initial_controller_arg,
        enable_mpc_arg,
        mpc_engine_arg,
        # Nodes (via OpaqueFunction for conditional parameter loading)
        OpaqueFunction(function=launch_setup),
    ])
