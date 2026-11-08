# robot_p2.launch.py - proto2 right hand on its own (CAN FD), real-hardware bringup
#
# Launch order (event-driven):
#   1. Environment vars (RMW, CycloneDDS, session dir)
#   2. CPU shield — alone. udp_hand_node starts from its exit, not beside it: a
#      cpuset attach overwrites affinity, so a shield that lands after a pin
#      silently undoes it (#405).
#   3. udp_hand_node (LifecycleNode, namespace "", absolute /p2/right/* topics
#      from config/p2/udp_hand_node_p2.yaml) → auto configure → activate.
#      sil_mode:=firmware adds the CAN FD fake firmware on vcan0 beside it.
#   4. Readiness gate waits for the /p2/right/joint_states publisher.
#   5. integrated_rt_controller (LifecycleNode) → configure → shield adopt →
#      activate; `hand_joint_controller` is the only controller this profile
#      ships (config/p2/robot.yaml initial_controller).
#
# No arm driver: there is no ros2_control controller_manager here, so none of
# the UR arguments (robot_ip, kinematics_params_file, …) exist. `use_mock_hardware`
# is kept for parity with the other robot launches and means "no hardware":
# it selects the driver's in-process loop model (sil_mode loopmodel) unless
# `sil_mode:=` is given explicitly.
#
# Topics: /p2/right/{joint_states, joint_command, motor_states, sensor_states,
# link_status}, /p2/estop. The controller's own topics live under
# /hand_joint_controller/ (p2_right/joint_goal, transforms).

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

from rtc_tools.launch import cpu_shield as shield
from rtc_tools.launch.pinning import pin_dds_threads_to_slot
from rtc_tools.launch.session import max_log_sessions_from_yaml, open_session
from rtc_tools.launch.thread_layout import (
    get_hand_driver_core,
    get_os_slot,
    get_rt_callback_core,
)
from rtc_tools.launch.trace_action import make_trace_action

PROFILE = "p2"
HAND_YAML = "udp_hand_node_p2.yaml"
# The driver forces this device in firmware SIL mode (udp_hand.launch.py
# _FW_CAN_INTERFACE); the fake has to sit on the same one.
FIRMWARE_CAN_INTERFACE = "vcan0"


def _load_hand_params(pkg, *relpath):
    """Return the `/**` ros__parameters dict from an installed hand-node yaml.

    Used at launch-build time to seed the sil_mode arg default and the fake
    firmware peer's protocol / cmd_ack. Returns {} on any error.
    """
    path = os.path.join(get_package_share_directory(pkg), *relpath)
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("/**", {}).get("ros__parameters", {}) or {}
    except (OSError, yaml.YAMLError):
        return {}


def _base_yaml_path(config_variant):
    """Installed path of the variant's mode-agnostic node parameter file."""
    return os.path.join(
        get_package_share_directory("integrated_bringup"), "config", config_variant, "_base.yaml"
    )


def _open_session(context):
    """Create this run's session directory and export it to the child processes.

    An ``OpaqueFunction`` body rather than a line in
    ``generate_launch_description()``: building a description must not create
    or delete anything, and only a context can read ``max_log_sessions`` (#402).
    """
    return open_session(context).actions


def _build_trace_action(context):
    """Wrap ``make_trace_action``, which needs both a context and the session."""
    return make_trace_action(context, session_dir=context.launch_configurations["session_dir"])


def generate_launch_description():
    hand_params = _load_hand_params("integrated_bringup", "config", PROFILE, HAND_YAML)
    yaml_sil_mode = str(hand_params.get("sil_mode", "off"))

    # ── Arguments ──────────────────────────────────────────────────────────────
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description=(
            "No hardware: run the hand driver's in-process loop model "
            "(sil_mode loopmodel). Ignored when sil_mode:= is given explicitly. "
            "Kept for parity with the UR robot launches."
        ),
    )
    sil_mode_arg = DeclareLaunchArgument(
        "sil_mode",
        default_value=yaml_sil_mode,
        choices=["off", "loopmodel", "firmware"],
        description=(
            f"SIL mode (default from config/{PROFILE}/{HAND_YAML}). firmware → the CAN FD "
            f"fake_hand_firmware on {FIRMWARE_CAN_INTERFACE} (repo_scripts/scripts/setup_vcan.sh)."
        ),
    )
    max_log_sessions_arg = DeclareLaunchArgument(
        "max_log_sessions",
        default_value=str(max_log_sessions_from_yaml(_base_yaml_path(PROFILE))),
        description=(
            "Maximum number of session folders to keep (YYMMDD_HHMM). Default "
            f"comes from config/{PROFILE}/_base.yaml, which is also the SSoT for "
            "the RT node's own max_log_sessions parameter."
        ),
    )
    use_cpu_affinity_arg = DeclareLaunchArgument(
        "use_cpu_affinity",
        default_value="true",
        description=(
            "Pin the hand driver process and the controller's DDS threads via "
            "tier-aware taskset (3-5 s after launch), and enable the CPU shield. "
            "Set false when running without hardware or in CI."
        ),
    )
    enable_mpc_arg = DeclareLaunchArgument(
        "enable_mpc",
        default_value="",
        description=(
            "Layout profile for the cset shield (no controller on this profile has "
            "an MPC thread). 'false' returns the MPC cores to the system cpuset; "
            "empty/true keeps them reserved."
        ),
    )
    enable_tracing_arg = DeclareLaunchArgument(
        "enable_tracing",
        default_value="false",
        description=(
            "Capture an LTTng trace via ros2_tracing for the active launch. "
            "Output: <session_dir>/tracing/<trace_session_name>/."
        ),
    )
    trace_session_name_arg = DeclareLaunchArgument(
        "trace_session_name",
        default_value="",
        description='LTTng session name under <session_dir>/tracing/. Empty = "trace".',
    )
    trace_events_ust_arg = DeclareLaunchArgument(
        "trace_events_ust",
        default_value="",
        description="Comma-separated UST events. Empty = ros2_tracing's DEFAULT_EVENTS_ROS.",
    )
    trace_events_kernel_arg = DeclareLaunchArgument(
        "trace_events_kernel",
        default_value="sched_switch,sched_waking,sched_wakeup,irq_handler_entry,irq_handler_exit",
        description="Comma-separated kernel events. 'none' disables kernel tracing.",
    )

    trace_action = OpaqueFunction(function=_build_trace_action)

    # ── Paths ──────────────────────────────────────────────────────────────────
    base_config = PathJoinSubstitution(
        [FindPackageShare("integrated_bringup"), "config", PROFILE, "_base.yaml"]
    )
    robot_config = PathJoinSubstitution(
        [FindPackageShare("integrated_bringup"), "config", PROFILE, "robot.yaml"]
    )
    hand_config = PathJoinSubstitution(
        [FindPackageShare("integrated_bringup"), "config", PROFILE, HAND_YAML]
    )
    cyclone_dds_xml = PathJoinSubstitution(
        [FindPackageShare("rtc_controller_manager"), "config", "cyclone_dds.xml"]
    )

    # Effective SIL mode: an explicit sil_mode:= wins; otherwise
    # use_mock_hardware:=true means the loop model; otherwise the YAML's value.
    effective_sil_mode = PythonExpression(
        [
            "'loopmodel' if ('",
            LaunchConfiguration("use_mock_hardware"),
            "' == 'true' and '",
            LaunchConfiguration("sil_mode"),
            "' == '",
            yaml_sil_mode,
            "') else '",
            LaunchConfiguration("sil_mode"),
            "'",
        ]
    )

    set_cyclone_uri = SetEnvironmentVariable(
        name="CYCLONEDDS_URI", value=["file://", cyclone_dds_xml]
    )
    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")
    open_session_action = OpaqueFunction(function=_open_session)

    # ── CPU Shield ────────────────────────────────────────────────────────────
    # One mapping for both consumers: the cset shield and the controller's
    # activation gate must agree on which profile is in force (#350).
    layout_profile = shield.mpc_layout_profile(LaunchConfiguration("enable_mpc"))
    # gated=False + pin_enabled, NOT an IfCondition: every node below starts
    # from this action's OnProcessExit, and a condition that suppresses the
    # process suppresses the exit event with it (#405).
    enable_cpu_shield = shield.enable_cpu_shield(
        "--robot",
        log_prefix="[RT]",
        gated=False,
        layout_profile=layout_profile,
        pin_enabled=LaunchConfiguration("use_cpu_affinity"),
    )

    # ── Driver / controller DDS pinning ───────────────────────────────────────
    # The hand process pins its own threads (#345); only its DDS threads need
    # an outside pin, with hand_aux_io protected by name.
    hand_driver_slot = get_hand_driver_core()
    pin_hand_driver = TimerAction(
        period=3.0,
        actions=[
            pin_dds_threads_to_slot(
                "udp_hand_node",
                "udp_hand_node",
                hand_driver_slot,
                extra_protected_names={"hand_aux_io"},
            )
        ],
    )
    rt_callback_slot = get_rt_callback_core()
    pin_rt_controller_dds = TimerAction(
        period=5.0,
        actions=[
            pin_dds_threads_to_slot(
                "integrated_rt_controller", "integrated_rt_controller", rt_callback_slot
            )
        ],
    )

    # ── RT controller node ─────────────────────────────────────────────────────
    rt_controller_node = LifecycleNode(
        package="integrated_bringup",
        executable="integrated_rt_controller",
        name="integrated_rt_controller",
        namespace="",
        output="screen",
        parameters=[
            base_config,
            robot_config,
            {
                "max_log_sessions": ParameterValue(
                    LaunchConfiguration("max_log_sessions"), value_type=int
                ),
                "rt_layout_profile": layout_profile,
            },
        ],
        emulate_tty=True,
        # Same allocator arrangement as every other launch of this package
        # (aligator's MPC links into the one binary; see sim_ur5e_p1b).
        additional_env={"LD_PRELOAD": "libmimalloc.so.2"},
    )

    # ── Hand driver node (LifecycleNode, CAN FD) ──────────────────────────────
    udp_hand_node = LifecycleNode(
        package="udp_hand_driver",
        executable="udp_hand_node",
        name="udp_hand_node",
        namespace="",
        output="screen",
        parameters=[
            hand_config,
            # value_type=str: "off" is a YAML 1.1 boolean, so an unqualified
            # LaunchConfiguration would coerce it and clash with the node's
            # string declaration during on_configure.
            {"sil_mode": ParameterValue(effective_sil_mode, value_type=str)},
            {
                "use_cpu_affinity": ParameterValue(
                    LaunchConfiguration("use_cpu_affinity"), value_type=bool
                )
            },
            # Core placement is the tier table's, not the YAML's: both slots
            # after the YAML file so a YAML value cannot override them.
            {"hand_cpu_slot": hand_driver_slot, "aux_cpu_slot": get_os_slot()},
        ],
        emulate_tty=True,
    )

    # ── Device-side firmware simulator (sil_mode=firmware only) ───────────────
    # The CAN FD fake on vcan0, same hand_side / cmd_ack as the node
    # (udp_hand.launch.py _firmware_fakes, one hand).
    fake_hand_firmware_node = Node(
        package="udp_hand_driver",
        executable="fake_hand_firmware",
        name="fake_hand_firmware",
        namespace="",
        output="screen",
        condition=IfCondition(EqualsSubstitution(effective_sil_mode, "firmware")),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("udp_hand_driver"), "config", "fake_hand_firmware.yaml"]
            ),
            {
                "protocol_version": str(hand_params.get("protocol_version", "2")),
                "can_interface": FIRMWARE_CAN_INTERFACE,
                "cmd_ack": str(hand_params.get("cmd_ack", "none")),
                "hand_side": str((hand_params.get("hand_sides") or ["right"])[0]),
            },
        ],
        emulate_tty=True,
    )

    # ── Lifecycle auto-configure/activate for udp_hand_node ───────────────────
    hand_auto_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=udp_hand_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda n: n == udp_hand_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    hand_trigger_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda n: n == udp_hand_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # ── RT: configure → shield-adopt → activate (#151, #344) ──────────────────
    _, rt_adopt_after_configure, rt_activate_after_adopt = shield.shield_adopt_then_activate(
        rt_controller_node
    )
    rt_trigger_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda n: n == rt_controller_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # ── Readiness gate: the hand publishes before the controller starts ───────
    comm_readiness_gate = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'echo "[RT] Readiness gate: waiting for the hand driver..."; '
            "timeout 30 bash -c '"
            "while ! ros2 topic info /p2/right/joint_states 2>/dev/null "
            '  | grep -q "Publisher count: [1-9]"; do sleep 0.5; done\' '
            '  && echo "[RT]   /p2/right/joint_states publisher OK" '
            '  || { echo "[RT] FATAL: /p2/right/joint_states not available after 30 s"; exit 1; }; '
            'echo "[RT] Hand driver ready"',
        ],
        output="screen",
    )

    start_gate_after_hand = RegisterEventHandler(
        OnProcessStart(
            target_action=udp_hand_node,
            on_start=[
                LogInfo(msg="[RT] udp_hand_node started — launching readiness gate"),
                comm_readiness_gate,
            ],
        )
    )
    start_rt_after_gate = RegisterEventHandler(
        OnProcessExit(
            target_action=comm_readiness_gate,
            on_exit=[
                LogInfo(msg="[RT] Readiness gate passed — launching integrated_rt_controller"),
                rt_controller_node,
                rt_adopt_after_configure,
                rt_activate_after_adopt,
                rt_trigger_configure,
            ],
        )
    )
    start_dds_pin_after_rt = RegisterEventHandler(
        OnProcessStart(
            target_action=rt_controller_node,
            on_start=[pin_rt_controller_dds],
        )
    )

    # ── Shield-first ordering (#405) ──────────────────────────────────────────
    start_infra_after_shield = RegisterEventHandler(
        OnProcessExit(
            target_action=enable_cpu_shield,
            on_exit=[
                LogInfo(msg="[RT] CPU shield settled — launching the hand driver"),
                fake_hand_firmware_node,
                udp_hand_node,
                pin_hand_driver,
                hand_auto_activate,
                hand_trigger_configure,
            ],
        )
    )

    return LaunchDescription(
        [
            # 1) Arguments
            use_mock_hardware_arg,
            sil_mode_arg,
            use_cpu_affinity_arg,
            enable_mpc_arg,
            enable_tracing_arg,
            trace_session_name_arg,
            trace_events_ust_arg,
            trace_events_kernel_arg,
            max_log_sessions_arg,
            # 2) Session + environment (#402: below the arguments, above every
            #    consumer of the session path)
            open_session_action,
            set_rmw,
            set_cyclone_uri,
            # 3) Infrastructure — the shield runs alone; everything that pins
            #    hangs off its exit
            enable_cpu_shield,
            start_infra_after_shield,
            # 4) Event-driven chain: hand started → gate → RT controller → DDS pin
            start_gate_after_hand,
            start_rt_after_gate,
            start_dds_pin_after_rt,
            # 5) Tracing (no-op when enable_tracing:=false)
            trace_action,
        ]
    )
