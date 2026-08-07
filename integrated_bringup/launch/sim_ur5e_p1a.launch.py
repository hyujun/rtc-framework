"""
sim_ur5e_p1a.launch.py — MuJoCo simulation launch file for UR5e
======================================================

Usage:
  # Default (uses YAML config)
  ros2 launch integrated_bringup sim_ur5e_p1a.launch.py

  # Headless mode (no display)
  ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_viewer:=false

  # External Menagerie model
  ros2 launch integrated_bringup sim_ur5e_p1a.launch.py \
      model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml

  # PD gain tuning
  ros2 launch integrated_bringup sim_ur5e_p1a.launch.py kp:=10.0 kd:=1.0

  # max_rtf override
  ros2 launch integrated_bringup sim_ur5e_p1a.launch.py max_rtf:=10.0

Nodes launched:
  1. mujoco_simulator_node  — MuJoCo physics simulator (replaces UR driver)
  2. integrated_rt_controller     — 500Hz controller (CV-based wakeup in sim mode)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

from rtc_tools.launch import cpu_shield as shield
from rtc_tools.launch.pinning import pin_dds_threads_to_slot, pin_process_to_slot
from rtc_tools.launch.thread_layout import get_rt_callback_core, get_sim_core
from rtc_tools.launch.trace_action import make_trace_action
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

    max_sessions = int(LaunchConfiguration("max_log_sessions").perform(context) or "10")
    cleanup_old_sessions(logging_root, max_sessions)

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_sim = FindPackageShare("rtc_mujoco_sim")
    pkg_bringup = FindPackageShare("integrated_bringup")

    # MuJoCo sim params: agnostic defaults + solver SSoT (rtc_mujoco_sim/
    # config/solver_param.yaml) + UR5e-specific robot_response groups, joint
    # names, topics, model_path (integrated_bringup/config/ur5e_p1a/
    # mujoco_simulator.yaml — overlaid on top).
    sim_default = PathJoinSubstitution([pkg_sim, "config", "solver_param.yaml"])
    sim_config = PathJoinSubstitution([pkg_bringup, "config", "ur5e_p1a", "mujoco_simulator.yaml"])
    # Mode-agnostic base (URDF/model topology, rosters, limits, control_rate);
    # sim.yaml overlays only the sim-specific delta on top.
    base_config = PathJoinSubstitution([pkg_bringup, "config", "ur5e_p1a", "_base.yaml"])
    ctrl_config = PathJoinSubstitution([pkg_bringup, "config", "ur5e_p1a", "sim.yaml"])

    # Hand UDP config -- p1a self-contained overlay (in integrated_bringup, always
    # present). The driver's own config stays generic (/hand/) for standalone use.
    hand_config = os.path.join(
        get_package_share_directory("integrated_bringup"),
        "config",
        "ur5e_p1a",
        "udp_hand_node_p1a.yaml",
    )

    # ── Build simulator parameters (defaults → robot YAML → CLI overrides) ──
    sim_params = [sim_default, sim_config]
    sim_overrides = {}

    # Check each launch argument - only add to overrides if explicitly provided
    model_path = LaunchConfiguration("model_path").perform(context)
    if model_path != "":
        sim_overrides["model_path"] = model_path

    enable_viewer = LaunchConfiguration("enable_viewer").perform(context)
    if enable_viewer != "":
        sim_overrides["enable_viewer"] = enable_viewer.lower() in ("true", "1", "yes")

    sync_timeout_ms = LaunchConfiguration("sync_timeout_ms").perform(context)
    if sync_timeout_ms != "":
        sim_overrides["sync_timeout_ms"] = float(sync_timeout_ms)

    max_rtf = LaunchConfiguration("max_rtf").perform(context)
    if max_rtf != "":
        sim_overrides["max_rtf"] = float(max_rtf)

    use_yaml_servo_gains = LaunchConfiguration("use_yaml_servo_gains").perform(context)
    if use_yaml_servo_gains != "":
        sim_overrides["use_yaml_servo_gains"] = use_yaml_servo_gains.lower() in (
            "true",
            "1",
            "yes",
        )

    # ── Fake hand response + control_rate ─────────────────────────────────────
    import yaml

    try:
        # control_rate is mode-agnostic → SSoT lives in _base.yaml (sim.yaml no
        # longer carries it). Read from _base so the MuJoCo fake-hand rate stays
        # locked to the RT node's rate instead of silently defaulting to 500.0.
        ctrl_yaml_path = os.path.join(
            get_package_share_directory("integrated_bringup"), "config", "ur5e_p1a", "_base.yaml"
        )
        with open(ctrl_yaml_path) as f:
            ctrl_yaml = yaml.safe_load(f)
        control_rate = (
            ctrl_yaml.get("/**", {}).get("ros__parameters", {}).get("control_rate", 500.0)
        )
        sim_overrides["control_rate"] = float(control_rate)
    except Exception:
        sim_overrides["control_rate"] = 500.0

    if sim_overrides:
        sim_params.append(sim_overrides)

    # ── Build controller parameters (YAML + overrides + launch args) ──────────
    # hand_config lives in integrated_bringup (this launch's own package), so it
    # is always present — no optional/None guard needed.
    ctrl_params = [base_config, ctrl_config, sim_config, hand_config]
    ctrl_overrides = {}

    kp = LaunchConfiguration("kp").perform(context)
    if kp != "":
        ctrl_overrides["kp"] = float(kp)

    kd = LaunchConfiguration("kd").perform(context)
    if kd != "":
        ctrl_overrides["kd"] = float(kd)

    ctrl_overrides["log_dir"] = session_dir
    # The controller refuses to activate a structurally MPC-enabled config under
    # a profile that dropped the MPC cores — otherwise the shield hands those
    # cores back while a SCHED_FIFO thread still runs on one (#350).
    ctrl_overrides["rt_layout_profile"] = layout_profile

    # Optional: override initial_controller (e.g. select demo_wbc_controller)
    initial_controller = LaunchConfiguration("initial_controller").perform(context)
    if initial_controller != "":
        ctrl_overrides["initial_controller"] = initial_controller

    # `enable_mpc` drives the `demo_wbc_controller.mpc.enabled` ROS parameter,
    # which integrated_rt_controller's `ApplyControllerParamOverrides` helper
    # writes into the YAML::Node handed to `LoadConfig`. The runtime gains
    # topic (index 7) can also toggle MPC on/off dynamically without
    # restarting the launch.
    enable_mpc = LaunchConfiguration("enable_mpc").perform(context)
    # One mapping for both consumers: the cset shield and the controller's
    # activation gate must agree on which profile is in force (#350).
    layout_profile = shield.mpc_layout_profile(enable_mpc)
    if enable_mpc.lower() in ("true", "1", "yes"):
        ctrl_overrides["demo_wbc_controller.mpc.enabled"] = True
    elif enable_mpc.lower() in ("false", "0", "no"):
        ctrl_overrides["demo_wbc_controller.mpc.enabled"] = False

    # `mpc_engine` selects between the MockMPCThread placeholder and the
    # HandlerMPCThread (real Aligator ProxDDP via MPCFactory +
    # GraspPhaseManager). Default "" leaves the YAML's `mpc.engine: "mock"`
    # untouched.
    mpc_engine = LaunchConfiguration("mpc_engine").perform(context)
    if mpc_engine.strip() != "":
        engine_str = mpc_engine.strip().lower()
        if engine_str not in ("mock", "handler"):
            raise RuntimeError(
                f"Invalid mpc_engine='{mpc_engine}'. "
                "Must be 'mock' or 'handler' (or empty to use YAML default)."
            )
        ctrl_overrides["demo_wbc_controller.mpc.engine"] = engine_str

    if ctrl_overrides:
        ctrl_params.append(ctrl_overrides)

    # ── Environment variables ─────────────────────────────────────────────────
    set_session_dir = SetEnvironmentVariable(name="RTC_SESSION_DIR", value=session_dir)

    # ── CPU Shield (Tier 1 only for simulation) ───────────────────────────────
    use_affinity = LaunchConfiguration("use_cpu_affinity").perform(context)
    actions = [set_session_dir]

    if use_affinity.lower() in ("true", "1", "yes"):
        # Mirror robot_ur5e_p1a.launch.py: probe `sudo -n true` first.  Launch's stdin
        # isn't a tty so an interactive sudo prompt would silently hang and
        # leave the cores un-isolated — better to warn loudly and skip.
        enable_sim_cpu_shield = shield.enable_cpu_shield(
            "--sim",
            log_prefix="[SIM]",
            gated=False,
            layout_profile=layout_profile,
        )
        actions.append(enable_sim_cpu_shield)

    # ── Node 1: MuJoCo Simulator (LifecycleNode) ────────────────────────────
    # `namespace=''` is required by launch_ros >= jazzy (keyword-only arg in
    # LifecycleNode.__init__); earlier ROS distros defaulted it implicitly.
    mujoco_node = LifecycleNode(
        package="rtc_mujoco_sim",
        executable="mujoco_simulator_node",
        name="mujoco_simulator",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=sim_params,
    )

    # ── Node 2: Custom Controller (LifecycleNode) ─────────────────────────
    # Node name = executable name (= "integrated_rt_controller"). The robot-specific
    # bringup owns runtime identity; rtc_controller_manager is library-only.
    # See agent_docs/design-principles.md.
    rt_controller_node = LifecycleNode(
        package="integrated_bringup",
        executable="integrated_rt_controller",
        name="integrated_rt_controller",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=ctrl_params,
        # Aligator's contact_rich MPC (gar StageFactor / ArenaMatrix) requires
        # mimalloc as the process allocator: it performs aligned/interior frees
        # that glibc free() aborts on ("free(): invalid pointer"), crashing the
        # WBC handler-MPC on grasp closure (contact_light → contact_rich swap).
        # libmimalloc is otherwise only *partially* adopted (transitive link
        # overrides C++ new/delete but not C malloc/free + Eigen) → cross-
        # allocator free. Preload it so the allocator is uniform. Bare soname
        # resolves via LD_LIBRARY_PATH (deps/install/lib).
        # TODO(rt-validate): confirm no RT-loop jitter regression on real
        # hardware before relying on this in production.
        additional_env={"LD_PRELOAD": "libmimalloc.so.2"},
    )

    # ── Lifecycle chain: mujoco configure→activate → integrated_rt_controller configure→activate
    mujoco_auto_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mujoco_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda n: n == mujoco_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    chain_rt_after_mujoco = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mujoco_node,
            start_state="activating",
            goal_state="active",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda n: n == rt_controller_node,
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )
    # Adopt the controller into the cset shield between configure and
    # activate, then gate ACTIVATE on that adopt's exit code. Same reason as
    # the robot launches: RT threads are spawned in on_activate and only
    # inherit the shielded cpuset if the process moved first, and a failed
    # adopt must not activate (issues #151, #344). --sim and --robot shield
    # the same CM-spanning range, so the sim path has the same exposure.
    _, rt_adopt_after_configure, rt_activate_after_adopt = shield.shield_adopt_then_activate(
        rt_controller_node, log_prefix="[SIM]"
    )
    trigger_mujoco_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda n: n == mujoco_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    actions.extend(
        [
            mujoco_node,
            rt_controller_node,
            mujoco_auto_activate,
            chain_rt_after_mujoco,
            rt_adopt_after_configure,
            rt_activate_after_adopt,
            trigger_mujoco_configure,
        ]
    )

    # ── MuJoCo sim_thread CPU pinning (Phase 6) ───────────────────────────────
    # SystemThreadConfigs.sim_thread is the C++ SSoT; the same tier dispatch
    # lives in rtc_tools.launch.thread_layout. The MuJoCo node also calls
    # ApplyThreadConfig(...sim_thread) from inside the SimLoop thread — this
    # taskset is a redundant safety net for any auxiliary process-level work
    # that runs before SimLoop starts. cpu_core == -1 → skip (no pinning).
    #
    # thread_layout returns *slot indices*; pin_*_to_slot resolves them to
    # logical CPU ids via rt_common.sh::slot_to_logical_cpu (issue #163).
    if use_affinity.lower() in ("true", "1", "yes"):
        sim_slot = get_sim_core()
        if sim_slot >= 0:
            actions.append(
                TimerAction(
                    period=2.0,
                    actions=[
                        pin_process_to_slot("mujoco_simulator", "mujoco_simulator_node", sim_slot)
                    ],
                )
            )

        # Layout v4.1: integrated_rt_controller DDS/aux threads co-pinned to
        # the rt_callback core (tier-aware via get_rt_callback_core) for cache
        # locality. ApplyThreadConfig() already pins SCHED_FIFO executors
        # (rt_control, rt_callback, mpc_*); this timer only catches
        # DDS-internal reader/writer threads that are spawned outside our
        # control. exec name = ROS node name = "integrated_rt_controller".
        rt_callback_slot = get_rt_callback_core()
        actions.append(
            TimerAction(
                period=5.0,
                actions=[
                    pin_dds_threads_to_slot(
                        "integrated_rt_controller", "integrated_rt_controller", rt_callback_slot
                    )
                ],
            )
        )

    # ── ros2_tracing capture (LTTng UST + kernel, opt-in) ────────────────────
    actions.extend(make_trace_action(context, session_dir=session_dir))

    return actions


def generate_launch_description():
    # ── Launch arguments with empty defaults (YAML values take precedence) ───
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description=(
            "Override model_path from YAML. "
            "Empty -> use YAML value (robot_descriptions/scene.xml). "
            "Absolute path -> use specified MuJoCo scene.xml"
        ),
    )

    enable_viewer_arg = DeclareLaunchArgument(
        "enable_viewer",
        default_value="",
        description=(
            "Override enable_viewer from YAML. "
            "Empty -> use YAML value (true). "
            'Set to "false" for headless mode'
        ),
    )

    sync_timeout_ms_arg = DeclareLaunchArgument(
        "sync_timeout_ms",
        default_value="",
        description=(
            "Override sync_timeout_ms from YAML. "
            "Empty -> use YAML value (50.0). "
            "Command wait timeout per step in milliseconds"
        ),
    )

    max_rtf_arg = DeclareLaunchArgument(
        "max_rtf",
        default_value="",
        description=(
            "Override max_rtf from YAML. "
            "Empty -> use YAML value (1.0). "
            "Maximum Real-Time Factor (0.0 = unlimited). "
            "Examples: 1.0 for real-time, 10.0 for 10x speed"
        ),
    )

    kp_arg = DeclareLaunchArgument(
        "kp",
        default_value="",
        description=(
            "Override kp from YAML. Empty -> use YAML value. PD controller proportional gain"
        ),
    )

    kd_arg = DeclareLaunchArgument(
        "kd",
        default_value="",
        description=(
            "Override kd from YAML. Empty -> use YAML value. PD controller derivative gain"
        ),
    )

    use_yaml_servo_gains_arg = DeclareLaunchArgument(
        "use_yaml_servo_gains",
        default_value="",
        description=(
            "Override use_yaml_servo_gains from YAML. "
            "Empty -> use YAML value (false). "
            "true: servo_kp/kd gains from YAML, false: XML gainprm/biasprm"
        ),
    )

    max_log_sessions_arg = DeclareLaunchArgument(
        "max_log_sessions",
        default_value="10",
        description="Maximum number of session folders to keep (YYMMDD_HHMM)",
    )

    use_cpu_affinity_arg = DeclareLaunchArgument(
        "use_cpu_affinity",
        default_value="true",
        description=(
            "Enable CPU shield (Tier 1 isolation) and MuJoCo core pinning. "
            "Set false for CI or environments without sudo."
        ),
    )

    initial_controller_arg = DeclareLaunchArgument(
        "initial_controller",
        default_value="",
        description=(
            "Override initial controller name (e.g. demo_wbc_controller). "
            "Empty = use value from config/ur5e_p1a/sim.yaml."
        ),
    )

    enable_mpc_arg = DeclareLaunchArgument(
        "enable_mpc",
        default_value="",
        description=(
            "Enable the MPC thread in DemoWbcController. "
            "Takes effect only when initial_controller:=demo_wbc_controller. "
            "Empty = use demo_wbc_controller.yaml default. "
            "Runtime toggle is also available via gains index 7."
        ),
    )

    mpc_engine_arg = DeclareLaunchArgument(
        "mpc_engine",
        default_value="",
        description=(
            "Select MPC engine in DemoWbcController: "
            '"mock" = MockMPCThread placeholder (default); '
            '"handler" = HandlerMPCThread + MPCFactory + GraspPhaseManager '
            "(real Aligator ProxDDP solve, requires mpc/phase_config.yaml + "
            "mpc/contact_light.yaml + mpc/contact_rich.yaml in the package "
            "share). Empty = use demo_wbc_controller.yaml default."
        ),
    )

    enable_tracing_arg = DeclareLaunchArgument(
        "enable_tracing",
        default_value="false",
        description=(
            "Capture an LTTng trace via ros2_tracing for the active launch. "
            "Output: <session_dir>/tracing/<trace_session_name>/ (open with "
            "babeltrace2 or convert via repo_scripts/scripts/timeline.sh). "
            "Run ./install.sh --tracing for one-time setup."
        ),
    )

    trace_session_name_arg = DeclareLaunchArgument(
        "trace_session_name",
        default_value="",
        description=(
            "LTTng session name. Becomes the leaf directory under "
            '<session_dir>/tracing/. Empty (default) = "trace".'
        ),
    )

    trace_events_ust_arg = DeclareLaunchArgument(
        "trace_events_ust",
        default_value="",
        description=(
            "Comma-separated UST events to enable. Empty (default) = "
            "ros2_tracing's DEFAULT_EVENTS_ROS (broad ros2:* coverage). "
            "Example narrow set: 'ros2:callback_start,ros2:callback_end'."
        ),
    )

    trace_events_kernel_arg = DeclareLaunchArgument(
        "trace_events_kernel",
        default_value="sched_switch,sched_waking,sched_wakeup,irq_handler_entry,irq_handler_exit",
        description=(
            "Comma-separated kernel events. Default = sched_switch + IRQ "
            "entry/exit, enough to see which thread ran on which core when. "
            "Empty disables kernel tracing (UST only). Requires "
            "lttng-modules-dkms and 'tracing' group membership."
        ),
    )

    return LaunchDescription(
        [
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
            enable_tracing_arg,
            trace_session_name_arg,
            trace_events_ust_arg,
            trace_events_kernel_arg,
            # Nodes (via OpaqueFunction for conditional parameter loading)
            OpaqueFunction(function=launch_setup),
        ]
    )
