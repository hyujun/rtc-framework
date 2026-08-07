# robot_ur5e_p1b.launch.py - UR5e + proto_1b real robot bringup
#
# Launch order (event-driven):
#   1. Environment vars (RMW, CycloneDDS, session dir)
#   2. CPU shield, UR driver, udp_hand_node  (parallel)
#   3. Readiness gate: polls /joint_states and /p1b/joint_states publishers
#   4. integrated_rt_controller node starts ONLY after gate exits successfully
#   5. DDS thread pinning runs 5 s after the CM process starts
#
# Core allocation optimizations applied:
#   C) udp_hand_node pins its own threads from inside — main to the hand_driver
#      slot, hand_aux_io to the aux slot (issue #345) — so the launch only
#      co-pins the DDS threads rclcpp::init() creates before the node exists.
#      The UR driver's RT loop goes through controller_manager's native
#      cpu_affinity parameter — taskset reaches only a process's main thread,
#      which for ros2_control_node is the executor, not the 500 Hz loop
#      (issue #343). Slots come from
#      rtc_tools.launch.thread_layout.get_{arm,hand}_driver_core (SSoT in
#      rtc::SystemThreadConfigs.{arm,hand}_driver) when use_cpu_affinity:=true.
#   D) integrated_rt_controller DDS threads co-pinned to the rt_callback core
#      (tier-aware via rtc_tools.launch.thread_layout.get_rt_callback_core)
#      so DDS dispatch and the FIFO 70 rt_callback executor share L1/L2 cache.
#   E) CycloneDDS threads also fall under the launch taskset above, because
#      CycloneDDS has no CPU-affinity setting to hand it to: <Threads><Thread>
#      exists in 0.10.5 (the version in use) but carries StackSize and
#      scheduling only. An earlier note here blamed "removed in 0.11+", which
#      was wrong on all three counts and invited a downgrade (issue #164).
#      The sweep is one-shot because CycloneDDS creates its threads at domain
#      init and adds none for later discovery or node creation (measured).
#
# The numbers thread_layout returns are *slot indices*, not logical CPU ids;
# rtc_tools.launch.pinning resolves them through rt_common.sh::slot_to_logical_cpu
# before calling taskset (issue #163). Never pass a slot to taskset directly.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
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
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

from rtc_tools.launch import cpu_shield as shield
from rtc_tools.launch.cm_rt_params import control_rate_from_yaml, write_cm_rt_param_file
from rtc_tools.launch.pinning import (
    pin_dds_threads_to_slot,
)
from rtc_tools.launch.thread_layout import (
    get_arm_driver_core,
    get_hand_driver_core,
    get_rt_callback_core,
)
from rtc_tools.launch.trace_action import make_trace_action
from rtc_tools.utils.session_dir import (
    cleanup_old_sessions,
    create_session_dir,
    resolve_logging_root,
)


def _load_hand_params(pkg, *relpath):
    """Return the `/**` ros__parameters dict from an installed hand-node yaml.

    Used at launch-build time to seed the sil_mode arg default and the
    fake_hand_firmware peer's port/protocol_version. Returns {} on any error.
    """
    path = os.path.join(get_package_share_directory(pkg), *relpath)
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("/**", {}).get("ros__parameters", {}) or {}
    except (OSError, yaml.YAMLError):
        return {}


def _build_cm_rt_params(context, session_dir, config_variant):
    """Write this run's controller_manager RT parameter file; return (path, LogInfo).

    Replaces the arm-driver ``taskset`` action. ``ros2_control_node``'s 500 Hz
    loop runs in a thread that is created before any launch timer can fire and
    is never the main thread, so a process pin moved the executor instead and the
    verifier — reading that same main thread — called it PASS (issue #343). The
    native ``cpu_affinity`` parameter is applied from *inside* the loop thread,
    which is the only place its TID is knowable.

    ``use_cpu_affinity:=false`` drops only ``cpu_affinity``; ``update_rate`` still
    has to reach the node, and priority/memory settings are not affinity.
    """
    pin_enabled = context.launch_configurations.get("use_cpu_affinity", "true") == "true"
    base_yaml = os.path.join(
        get_package_share_directory("integrated_bringup"), "config", config_variant, "_base.yaml"
    )
    params_file, params = write_cm_rt_param_file(
        session_dir,
        control_rate=control_rate_from_yaml(base_yaml),
        slot=get_arm_driver_core() if pin_enabled else None,
    )
    if "cpu_affinity" in params:
        detail = f"cpu_affinity=logical {params['cpu_affinity']} (slot {get_arm_driver_core()})"
    elif pin_enabled:
        detail = "cpu_affinity OMITTED — slot could not be resolved to a logical cpu"
    else:
        detail = "cpu_affinity disabled (use_cpu_affinity:=false)"
    log = LogInfo(
        msg=(
            f"[RT] UR controller_manager RT params → {params_file}: "
            f"update_rate={params['update_rate']}, {detail}, "
            f"thread_priority={params['thread_priority']}, lock_memory={params['lock_memory']}"
        )
    )
    return params_file, log


def _launch_setup(context, *, session_dir):
    """Resolve launch arguments and build actions that need runtime values."""
    # ROS 2 Jazzy renamed use_fake_hardware -> use_mock_hardware.
    # Accept either flag; if either is 'true', enable mock hardware.
    use_mock = context.launch_configurations.get("use_mock_hardware", "false")
    use_fake = context.launch_configurations.get("use_fake_hardware", "false")
    mock_enabled = "true" if use_mock == "true" or use_fake == "true" else "false"

    cm_rt_params_file, cm_rt_params_log = _build_cm_rt_params(context, session_dir, "ur5e_p1b")

    # Humble uses 'use_fake_hardware', Jazzy+ uses 'use_mock_hardware'.
    # Pass both so the correct one is picked up regardless of ROS distro.
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": mock_enabled,
            "use_fake_hardware": mock_enabled,
            "launch_rviz": "false",
            # UR driver calibration + runtime controls (issue #133).
            # kinematics_params_file is NOT declared by ur_control.launch.py;
            # IncludeLaunchDescription propagates it as a launch configuration
            # so the nested ur_rsp.launch.py consumes it for the xacro
            # kinematics_params argument (robot-specific factory calibration).
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "launch_dashboard_client": LaunchConfiguration("launch_dashboard_client"),
            "controller_spawner_timeout": LaunchConfiguration("controller_spawner_timeout"),
            "activate_joint_controller": LaunchConfiguration("activate_joint_controller"),
            "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
            # Upstream declares this argument and passes it as the node's FIRST
            # parameter file; the default carries update_rate alone. Ours adds
            # the RT contract (cpu_affinity / thread_priority / lock_memory) —
            # see rtc_tools.launch.cm_rt_params, issue #343.
            "update_rate_config_file": cm_rt_params_file,
        }.items(),
    )
    # ── Activate forward_position_controller (mock hardware only) ────────────
    # For real robots, the UR driver's controller_stopper_node handles activation
    # via initial_joint_controller when play is pressed on the teach pendant.
    # For mock hardware, there is no controller_stopper, so we must switch manually.
    actions = [cm_rt_params_log, ur_driver_launch]
    if mock_enabled == "true":
        activate_fwd_controller = TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-c",
                        'echo "[RT] Switching to forward_position_controller (mock mode)..."; '
                        "ros2 service call /controller_manager/switch_controller "
                        "  controller_manager_msgs/srv/SwitchController "
                        '  "{activate_controllers: [forward_position_controller], '
                        "    deactivate_controllers: [scaled_joint_trajectory_controller], "
                        '    strictness: 1}" '
                        '  && echo "[RT] forward_position_controller activated" '
                        '  || echo "[RT] WARNING: controller switch failed"',
                    ],
                    output="screen",
                )
            ],
        )
        actions.append(activate_fwd_controller)
    return actions


def generate_launch_description():
    # ── Session directory (YYMMDD_HHMM) ──────────────────────────────────────
    logging_root = resolve_logging_root()
    session_dir = create_session_dir(logging_root)
    cleanup_old_sessions(logging_root, 10)

    # ── Arguments ──────────────────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.0.3", description="IP address of the UR robot"
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock hardware for testing (Jazzy)",
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="[Deprecated — use use_mock_hardware] Alias kept for compatibility",
    )

    use_cpu_affinity_arg = DeclareLaunchArgument(
        "use_cpu_affinity",
        default_value="true",
        description=(
            "Pin the UR/hand driver processes and the controller's DDS threads "
            "via tier-aware taskset (3-5 s after launch), and enable the CPU "
            "shield. Set false when running with fake hardware or in CI."
        ),
    )

    # ── UR driver calibration + runtime controls (issue #133) ────────────────
    kinematics_params_file_arg = DeclareLaunchArgument(
        "kinematics_params_file",
        default_value=os.path.expanduser("~/ur5e_calibration.yaml"),
        description=(
            "UR factory calibration YAML generated by ur_calibration "
            "(calibration_correction.launch.py). Forwarded to the nested "
            "ur_rsp.launch.py so the driver's robot_description/TF reflect the "
            "actual robot. Default = $HOME/ur5e_calibration.yaml."
        ),
    )
    headless_mode_arg = DeclareLaunchArgument(
        "headless_mode",
        default_value="true",
        description=(
            "Run the UR driver in headless control mode (no External Control "
            "program play on the teach pendant). Default true for real-robot use."
        ),
    )
    reverse_ip_arg = DeclareLaunchArgument(
        "reverse_ip",
        default_value="0.0.0.0",
        description=(
            "PC-side IP the UR controller dials back for the reverse connection. "
            "0.0.0.0 = all interfaces; set explicitly on multi-NIC / RT-only NIC."
        ),
    )
    launch_dashboard_client_arg = DeclareLaunchArgument(
        "launch_dashboard_client",
        default_value="true",
        description=(
            "Launch the UR Dashboard client node (robot mode, program "
            "start/stop, power/brake). Set false when the dashboard port is "
            "blocked or for fully manual operation."
        ),
    )
    controller_spawner_timeout_arg = DeclareLaunchArgument(
        "controller_spawner_timeout",
        default_value="10",
        description=(
            "Seconds the controller_manager spawner waits for controller "
            "load/activate. Raise if UR controller startup is slow under load."
        ),
    )
    activate_joint_controller_arg = DeclareLaunchArgument(
        "activate_joint_controller",
        default_value="true",
        description=(
            "Start initial_joint_controller active. False = load but leave "
            "inactive for an external controller-switching policy."
        ),
    )
    initial_joint_controller_arg = DeclareLaunchArgument(
        "initial_joint_controller",
        default_value="forward_position_controller",
        description=(
            "Joint controller the UR driver loads/activates first. RTC backend "
            "publishes Float64MultiArray to /forward_position_controller/commands."
        ),
    )

    enable_mpc_arg = DeclareLaunchArgument(
        "enable_mpc",
        default_value="",
        description=(
            "Enable the MPC thread in DemoWbcController. "
            "Takes effect only when initial_controller is demo_wbc_controller. "
            "Empty = use demo_wbc_controller.yaml default. "
            "Runtime toggle is also available via gains index 7."
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

    # Closure-bound helper: trace_action.make_trace_action needs a launch context.
    # session_dir is captured from the enclosing scope.
    def _build_trace_action(context):
        return make_trace_action(context, session_dir=session_dir)

    trace_action = OpaqueFunction(function=_build_trace_action)

    # ── Paths ──────────────────────────────────────────────────────────────────
    # Mode-agnostic base (URDF/model topology, rosters, limits, control_rate);
    # robot.yaml overlays only the real-hardware delta on top.
    base_config = PathJoinSubstitution(
        [FindPackageShare("integrated_bringup"), "config", "ur5e_p1b", "_base.yaml"]
    )
    ur_control_config = PathJoinSubstitution(
        [FindPackageShare("integrated_bringup"), "config", "ur5e_p1b", "robot.yaml"]
    )

    # Hand UDP config -- p1b self-contained overlay (symmetric with p1a's
    # integrated_bringup/config/ur5e_p1a/udp_hand_node_p1a.yaml). Carries every
    # p1b-specific value (bulk/1b, target_ip, /p1b/* topics, joint/fingertip
    # names). The driver's own config/udp_hand_node.yaml stays generic (/hand/).
    p1b_hand_udp_config = PathJoinSubstitution(
        [FindPackageShare("integrated_bringup"), "config", "ur5e_p1b", "udp_hand_node_p1b.yaml"]
    )

    # No fingertip F/T inferencer config for p1b: the 1b force path has no
    # barometer input, so the node's ft_inferencer.enabled=false default keeps
    # inference disabled (single yaml input for the node).

    # SIL mode: default from the p1b hand yaml (SSoT); CLI `sil_mode:=` overrides.
    # firmware mode spawns fake_hand_firmware on loopback (below); the node C++
    # derives use_fake_hand / target_ip=127.0.0.1 from sil_mode.
    _p1b_hand_params = _load_hand_params(
        "integrated_bringup", "config", "ur5e_p1b", "udp_hand_node_p1b.yaml"
    )
    sil_mode_arg = DeclareLaunchArgument(
        "sil_mode",
        default_value=str(_p1b_hand_params.get("sil_mode", "off")),
        choices=["off", "loopmodel", "firmware"],
        description="SIL mode (default from udp_hand_node_p1b.yaml). "
        "firmware → fake_hand_firmware over loopback.",
    )

    cyclone_dds_xml = PathJoinSubstitution(
        [FindPackageShare("rtc_controller_manager"), "config", "cyclone_dds.xml"]
    )

    # ── CycloneDDS thread restriction ─────────────────────────────────────────
    set_cyclone_uri = SetEnvironmentVariable(
        name="CYCLONEDDS_URI", value=["file://", cyclone_dds_xml]
    )

    set_session_dir = SetEnvironmentVariable(name="RTC_SESSION_DIR", value=session_dir)

    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    # ── UR robot driver launch (via OpaqueFunction for mock_hardware compat) ──
    # session_dir goes in as a kwarg because the CM RT parameter file is written
    # next to this run's logs, and only the OpaqueFunction can read the resolved
    # use_cpu_affinity value that decides whether it carries a pin (issue #343).
    ur_driver_launch_action = OpaqueFunction(
        function=_launch_setup, kwargs={"session_dir": session_dir}
    )

    # ── CPU Shield ────────────────────────────────────────────────────────────
    # Probe + enable live in rtc_tools.launch.cpu_shield so all five launches
    # share one cset-aware implementation (issues #151, #344).
    enable_cpu_shield = shield.enable_cpu_shield(
        "--robot",
        log_prefix="[RT]",
        enable_mpc=LaunchConfiguration("enable_mpc"),
    )

    # ── External driver CPU pinning (Phase 6) ─────────────────────────────────
    # Pins via tier-aware slot resolution. SystemThreadConfigs.{arm,hand}_driver
    # is the C++ SSoT; rtc_tools.launch.thread_layout mirrors it for launch.
    # These are slot indices — the pinning helpers resolve them to logical CPU
    # ids via rt_common.sh::slot_to_logical_cpu (issue #163).
    #
    # The arm driver is NOT here: taskset cannot reach its RT loop, so it is
    # pinned through controller_manager's own cpu_affinity parameter instead
    # (_build_cm_rt_params above, issue #343).
    hand_driver_slot = get_hand_driver_core()
    # The `taskset -a` sweep this replaces is gone (issue #345). The hand process
    # now pins its own threads from inside — main to hand_driver, hand_aux_io to
    # the aux slot — so the sweep is both redundant and harmful: `-a` would drag
    # hand_aux_io back onto the hand_driver core.
    #
    # What still needs an outside pin is the DDS threads. rclcpp::init() creates
    # them before the node exists, so no in-process self-pin can reach them, and
    # the acceptance criterion still puts them on the hand_driver slot. This is
    # the same helper the controller process uses; it skips RTC-placed threads by
    # name and any SCHED_FIFO thread, with hand_aux_io named explicitly because it
    # lives outside the thread_config.hpp SSoT the default list mirrors.
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

    # ── integrated_rt_controller DDS thread pinning ─────────────────────────────────
    # exec name = ROS node name = "integrated_rt_controller" (Phase 3 정렬).
    # Layout v4.1: rt_callback slot is tier-aware (slot 2 on every tier); the
    # logical CPU it maps to is topology-dependent (issue #163).
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
    # The `enable_mpc` launch arg is declared below but takes effect through
    # the runtime gains topic (index 7) rather than a param override here —
    # robot_ur5e_p1b.launch.py does not use OpaqueFunction, and nested YAML overrides
    # would require restructuring the launch. The sim_ur5e_p1a.launch.py flow does
    # inject the override directly via its OpaqueFunction setup.
    # `namespace=''` is required by launch_ros >= jazzy (keyword-only arg
    # in LifecycleNode.__init__); earlier distros defaulted it implicitly.
    # Node name = executable name (= "integrated_rt_controller"). The robot-specific
    # bringup owns runtime identity; rtc_controller_manager is library-only.
    # See agent_docs/design-principles.md.
    rt_controller_node = LifecycleNode(
        package="integrated_bringup",
        executable="integrated_rt_controller",
        name="integrated_rt_controller",
        namespace="",
        output="screen",
        parameters=[
            base_config,
            ur_control_config,
            {
                "log_dir": session_dir,
            },
        ],
        emulate_tty=True,
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

    # ── Hand UDP driver node (LifecycleNode) ──────────────────────────────────
    # Publishes /p1b/joint_states and /p1b/sensor_states for integrated_rt_controller.
    udp_hand_node = LifecycleNode(
        package="udp_hand_driver",
        executable="udp_hand_node",
        name="udp_hand_node",
        namespace="",
        output="screen",
        parameters=[
            p1b_hand_udp_config,
            # sil_mode override: default == yaml value (no drift); CLI wins when set.
            # value_type=str pins the type: "off"/"on"/etc. are YAML 1.1 booleans,
            # so an unqualified LaunchConfiguration coerces to bool and clashes with
            # the node's string declaration during on_configure.
            {"sil_mode": ParameterValue(LaunchConfiguration("sil_mode"), value_type=str)},
            # The hand process pins its own threads now (issue #345): the main
            # thread to the hand_driver slot and hand_aux_io to the aux slot. It
            # has to see the same switch the launch-level pinning obeys, or
            # use_cpu_affinity:=false would silently leave the in-process pins
            # in place. value_type=bool converts the "true"/"false" arg string.
            {
                "use_cpu_affinity": ParameterValue(
                    LaunchConfiguration("use_cpu_affinity"), value_type=bool
                )
            },
        ],
        emulate_tty=True,
    )

    # ── Device-side firmware simulator (sil_mode=firmware only) ───────────────
    # Binds the hand target_port on loopback and echoes protocol_version so the
    # node (which forces target_ip=127.0.0.1 in firmware mode) has a real peer.
    # Unpinned: SIL helper, not part of the RT hot path.
    fake_hand_firmware_node = Node(
        package="udp_hand_driver",
        executable="fake_hand_firmware",
        name="fake_hand_firmware",
        namespace="",
        output="screen",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("sil_mode"), "firmware")),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("udp_hand_driver"), "config", "fake_hand_firmware.yaml"]
            ),
            {
                "port": int(_p1b_hand_params.get("target_port", 55151)),
                "protocol_version": str(_p1b_hand_params.get("protocol_version", "1b")),
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

    # ── Lifecycle auto-configure → shield-adopt → activate ──────────────────────────
    # RT + nrt threads spawn in on_activate (rt_controller_node.cpp), and they
    # self-pin to logical CPUs inside the shield's "user" cpuset. If the CM is
    # still in "system" when that happens the pins EINVAL and thread_utils.hpp
    # drops SCHED_FIFO too (issue #151). So, once configure completes (inactive),
    # move the CM into "user" and gate ACTIVATE on that move *finishing* — a
    # deterministic order, not a race between the async sudo call and thread
    # creation. adopt runs ungated (it must fire, and exit, even when
    # use_cpu_affinity:=false — where it finds no shield and no-ops) so the
    # controller still activates in fake-hardware / CI runs.
    # ACTIVATE is now gated on the adopt action's *exit code*, not merely on it
    # having exited: a failed adopt under an active shield holds the controller in
    # `inactive` instead of running the RT loop unpinned and off SCHED_FIFO
    # (issue #344). Shared with p1a and the sim launches, which drifted for three
    # consecutive fixes while this chain lived only here.
    _, rt_adopt_after_configure, rt_activate_after_adopt = shield.shield_adopt_then_activate(
        rt_controller_node
    )
    rt_trigger_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda n: n == rt_controller_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # ── Readiness gate ────────────────────────────────────────────────────────
    # Polls until UR driver publishes /joint_states AND udp_hand_node publishes
    # /p1b/joint_states.  integrated_rt_controller is chained to start only after
    # this gate process exits successfully (via OnProcessExit event handler).
    comm_readiness_gate = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            # --- Wait for /joint_states publisher (UR driver) ---
            'echo "[RT] Readiness gate: waiting for communication nodes..."; '
            "timeout 30 bash -c '"
            "while ! ros2 topic info /joint_states 2>/dev/null "
            '  | grep -q "Publisher count: [1-9]"; do sleep 0.5; done\' '
            '  && echo "[RT]   /joint_states publisher OK" '
            '  || { echo "[RT] FATAL: /joint_states not available after 30 s"; exit 1; }; '
            # --- Wait for /p1b/joint_states publisher (udp_hand_node) ---
            "timeout 30 bash -c '"
            "while ! ros2 topic info /p1b/joint_states 2>/dev/null "
            '  | grep -q "Publisher count: [1-9]"; do sleep 0.5; done\' '
            '  && echo "[RT]   /p1b/joint_states publisher OK" '
            '  || { echo "[RT] FATAL: /p1b/joint_states not available after 30 s"; exit 1; }; '
            # --- Wait for forward_position_controller subscriber (ros2_control) ---
            "timeout 30 bash -c '"
            "while ! ros2 topic info /forward_position_controller/commands 2>/dev/null "
            '  | grep -q "Subscription count: [1-9]"; do sleep 0.5; done\' '
            '  && echo "[RT]   forward_position_controller subscriber OK" '
            '  || echo "[RT] WARNING: forward_position_controller not ready after 30 s '
            '(will activate later via controller_stopper or mock switch)"; '
            'echo "[RT] All communication nodes ready"',
        ],
        output="screen",
    )

    # ── Event-driven launch chain ─────────────────────────────────────────────
    # Gate starts only after udp_hand_node process is running (ensures DDS
    # endpoint is registered before polling).  integrated_rt_controller starts
    # only after the gate exits with success.  DDS thread pinning fires 5 s
    # after the CM process starts.
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

    return LaunchDescription(
        [
            # 1) Arguments
            robot_ip_arg,
            use_mock_hardware_arg,
            use_fake_hardware_arg,
            use_cpu_affinity_arg,
            kinematics_params_file_arg,
            headless_mode_arg,
            reverse_ip_arg,
            launch_dashboard_client_arg,
            controller_spawner_timeout_arg,
            activate_joint_controller_arg,
            initial_joint_controller_arg,
            enable_mpc_arg,
            enable_tracing_arg,
            trace_session_name_arg,
            trace_events_ust_arg,
            trace_events_kernel_arg,
            sil_mode_arg,
            # 2) Environment
            set_session_dir,
            set_rmw,
            set_cyclone_uri,
            # 3) Infrastructure (parallel)
            enable_cpu_shield,
            ur_driver_launch_action,
            fake_hand_firmware_node,
            udp_hand_node,
            pin_hand_driver,
            hand_auto_activate,
            hand_trigger_configure,
            # 4) Event-driven chain:
            #    udp_hand_node started → comm_readiness_gate
            #    → gate exits OK → integrated_rt_controller
            #    → CM process started → pin_rt_controller_dds
            start_gate_after_hand,
            start_rt_after_gate,
            start_dds_pin_after_rt,
            # 5) Tracing (no-op when enable_tracing:=false)
            trace_action,
        ]
    )
