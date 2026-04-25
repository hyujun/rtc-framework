"""
mujoco_sim.launch.py — Robot-agnostic MuJoCo standalone launch
==============================================================

Brings up `mujoco_simulator_node` only — no controller, no robot-specific
config. Useful as a smoke test for the simulator itself or as a building
block when a robot bringup package needs the simulator without the rest of
its stack.

Robot-specific demos (UR5e + RT controller + hand bridge) live in
`ur5e_bringup/launch/sim.launch.py` and load `mujoco_simulator.yaml` from
`ur5e_bringup/config/`.

Usage:
  # Smoke test with an external MJCF (no robot params loaded)
  ros2 launch rtc_mujoco_sim mujoco_sim.launch.py \\
      model_path:=/path/to/scene.xml

  # Headless mode
  ros2 launch rtc_mujoco_sim mujoco_sim.launch.py \\
      model_path:=/path/to/scene.xml enable_viewer:=false

  # Override params file (must define robot_response.* groups)
  ros2 launch rtc_mujoco_sim mujoco_sim.launch.py \\
      params_file:=/path/to/my_robot_sim.yaml

Defaults loaded: rtc_mujoco_sim/config/mujoco_default.yaml (agnostic only;
robot-specific groups MUST be supplied via params_file or the node will
fail to configure).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

from rtc_tools.utils.session_dir import (
    cleanup_old_sessions,
    create_session_dir,
    resolve_logging_root,
)


def launch_setup(context, *args, **kwargs):
    # ── Session directory ────────────────────────────────────────────────────
    logging_root = resolve_logging_root()
    session_dir = create_session_dir(logging_root)
    max_sessions = int(
        LaunchConfiguration('max_log_sessions').perform(context) or '10')
    cleanup_old_sessions(logging_root, max_sessions)

    # ── Params: default file + optional override + CLI overrides ─────────────
    pkg_sim = FindPackageShare('rtc_mujoco_sim')
    default_params = PathJoinSubstitution(
        [pkg_sim, 'config', 'mujoco_default.yaml'])

    params_file = LaunchConfiguration('params_file').perform(context)
    sim_params: list = [default_params]
    if params_file != '':
        sim_params.append(params_file)

    overrides: dict = {}
    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path != '':
        overrides['model_path'] = model_path

    enable_viewer = LaunchConfiguration('enable_viewer').perform(context)
    if enable_viewer != '':
        overrides['enable_viewer'] = enable_viewer.lower() in (
            'true', '1', 'yes')

    max_rtf = LaunchConfiguration('max_rtf').perform(context)
    if max_rtf != '':
        overrides['max_rtf'] = float(max_rtf)

    if overrides:
        sim_params.append(overrides)

    # ── Environment variables ────────────────────────────────────────────────
    set_session_dir = SetEnvironmentVariable(
        name='RTC_SESSION_DIR', value=session_dir)

    # ── MuJoCo simulator (LifecycleNode) ─────────────────────────────────────
    mujoco_node = LifecycleNode(
        package='rtc_mujoco_sim',
        executable='mujoco_simulator_node',
        name='mujoco_simulator',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=sim_params,
    )

    # ── Lifecycle: configure → activate ──────────────────────────────────────
    auto_activate = RegisterEventHandler(
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
    trigger_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == mujoco_node,
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    return [set_session_dir, mujoco_node, auto_activate, trigger_configure]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description=(
                'Optional ROS params YAML overlaid on top of mujoco_default.yaml. '
                'Must supply robot_response.groups + per-group joint/topic config.'
            ),
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Override model_path. Empty -> use YAML value.',
        ),
        DeclareLaunchArgument(
            'enable_viewer',
            default_value='',
            description='Override enable_viewer. Empty -> use YAML value.',
        ),
        DeclareLaunchArgument(
            'max_rtf',
            default_value='',
            description='Override max_rtf. Empty -> use YAML value.',
        ),
        DeclareLaunchArgument(
            'max_log_sessions',
            default_value='10',
            description='Maximum number of session folders to keep.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
