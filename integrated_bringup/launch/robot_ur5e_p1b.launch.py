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
#   C) UR driver + udp_hand_node processes pinned via tier-aware taskset
#      (rtc_tools.launch.thread_layout.get_{arm,hand}_driver_core; SSoT in
#      rtc::SystemThreadConfigs.{arm,hand}_driver) when use_cpu_affinity:=true
#   D) integrated_rt_controller DDS threads co-pinned to the rt_callback core
#      (tier-aware via rtc_tools.launch.thread_layout.get_rt_callback_core)
#      so DDS dispatch and the FIFO 70 rt_callback executor share L1/L2 cache.
#   E) CycloneDDS threads also fall under the launch taskset above (no
#      separate CYCLONEDDS_URI thread affinity since CycloneDDS 0.11+)

import os

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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

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


def _pin_external_driver(label: str, process_grep: str, core: int) -> ExecuteProcess:
    """Build a taskset ExecuteProcess that pins ``process_grep`` to ``core``.

    ``core < 0`` is the "no pinning" sentinel from ``select_thread_layout`` —
    in that case the resulting action logs the skip and returns immediately,
    so launch files do not need their own guard.
    """
    # Defense in depth: the f-string below interpolates ``label`` and
    # ``process_grep`` into a ``bash -c`` command. Current callers pass
    # hardcoded literals, but reject any embedded double quote outright so a
    # future caller cannot accidentally inject shell syntax.
    if '"' in label or '"' in process_grep:
        raise ValueError(
            f"_pin_external_driver: embedded double quote disallowed in "
            f"label={label!r} or process_grep={process_grep!r}"
        )
    if core < 0:
        return ExecuteProcess(
            cmd=[
                "bash",
                "-c",
                f'echo "[RT] {label}: cpu_core=-1, no taskset pinning"',
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
        )
    return ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f'PID=$(pgrep -nf "{process_grep}") && [ -n "$PID" ] && '
            f'taskset -cp {core} "$PID" && '
            f'echo "[RT] {label} (PID=$PID) pinned to Core {core}" || '
            f'echo "[RT] WARNING: {label} not found — CPU pinning skipped"',
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
    )


def _launch_setup(context):
    """Resolve launch arguments and build actions that need runtime values."""
    # ROS 2 Jazzy renamed use_fake_hardware -> use_mock_hardware.
    # Accept either flag; if either is 'true', enable mock hardware.
    use_mock = context.launch_configurations.get("use_mock_hardware", "false")
    use_fake = context.launch_configurations.get("use_fake_hardware", "false")
    mock_enabled = "true" if use_mock == "true" or use_fake == "true" else "false"

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
        }.items(),
    )
    # ── Activate forward_position_controller (mock hardware only) ────────────
    # For real robots, the UR driver's controller_stopper_node handles activation
    # via initial_joint_controller when play is pressed on the teach pendant.
    # For mock hardware, there is no controller_stopper, so we must switch manually.
    actions = [ur_driver_launch]
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
        "robot_ip", default_value="192.168.1.10", description="IP address of the UR robot"
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
            "Pin UR driver process to Core 0-1 via taskset (3 s after launch). "
            "Set false when running with fake hardware or in CI."
        ),
    )

    # ── UR driver calibration + runtime controls (issue #133) ────────────────
    kinematics_params_file_arg = DeclareLaunchArgument(
        "kinematics_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
        ),
        description=(
            "UR factory calibration YAML generated by ur_calibration "
            "(calibration_correction.launch.py). Forwarded to the nested "
            "ur_rsp.launch.py so the driver's robot_description/TF reflect the "
            "actual robot. Default = ur_description ur5e factory calibration."
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

    # Hand UDP config -- p1b standalone (self-contained; NOT layered on
    # udp_hand_driver/config/udp_hand_node.yaml, which is p1a-only). This file
    # carries every p1b-specific value (bulk/1b, target_ip, /p1b/* topics,
    # joint/fingertip names) so no p1a base leaks in.
    p1b_hand_udp_config = PathJoinSubstitution(
        [FindPackageShare("integrated_bringup"), "config", "ur5e_p1b", "udp_hand_node_p1b.yaml"]
    )

    # Fingertip F/T inferencer config (udp_hand_driver package)
    ft_inferencer_config = PathJoinSubstitution(
        [FindPackageShare("udp_hand_driver"), "config", "fingertip_ft_inferencer.yaml"]
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
    ur_driver_launch_action = OpaqueFunction(function=_launch_setup)

    # ── CPU Shield ────────────────────────────────────────────────────────────
    _pkg_prefix = get_package_share_directory("repo_scripts")
    _shield_script = os.path.join(
        os.path.dirname(os.path.dirname(_pkg_prefix)), "lib", "repo_scripts", "cpu_shield.sh"
    )

    enable_cpu_shield = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f'if [ -f "{_shield_script}" ]; then '
            "  ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null); "
            '  if [ -z "$ISOLATED" ]; then '
            '    echo "[RT] CPU shield not active — enabling robot mode..."; '
            "    if sudo -n true 2>/dev/null; then "
            f'      sudo "{_shield_script}" on --robot; '
            "    else "
            '      echo "[RT] WARNING: sudo requires a password — skipping CPU shield. '
            "Configure passwordless sudo for cpu_shield.sh or run: "
            f'sudo {_shield_script} on --robot"; '
            "    fi; "
            "  else "
            '    echo "[RT] CPU shield already active: Core $ISOLATED isolated"; '
            "  fi; "
            "else "
            f'  echo "[RT] WARNING: cpu_shield.sh not found: {_shield_script}"; '
            "fi",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
    )

    # ── External driver CPU pinning (Phase 6) ─────────────────────────────────
    # Pins via tier-aware core resolution. SystemThreadConfigs.{arm,hand}_driver
    # is the C++ SSoT; rtc_tools.launch.thread_layout mirrors it for launch.
    arm_driver_core = get_arm_driver_core()
    hand_driver_core = get_hand_driver_core()
    pin_ur_driver = TimerAction(
        period=3.0,
        actions=[_pin_external_driver("ur_ros2_driver", "ur_ros2_driver", arm_driver_core)],
    )
    pin_hand_driver = TimerAction(
        period=3.0,
        actions=[_pin_external_driver("udp_hand_node", "udp_hand_node", hand_driver_core)],
    )

    # ── integrated_rt_controller DDS thread pinning ─────────────────────────────────
    # exec name = ROS node name = "integrated_rt_controller" (Phase 3 정렬).
    # Layout v4.1: rt_callback core is tier-aware (Core 2 on every tier).
    rt_callback_core = get_rt_callback_core()
    pin_rt_controller_dds = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    'PID=$(pgrep -nf "integrated_rt_controller"); '
                    'if [ -z "$PID" ]; then '
                    '  echo "[RT] WARNING: integrated_rt_controller not found — DDS thread pinning skipped"; '
                    "  exit 0; "
                    "fi; "
                    # Layout v4.1: DDS receive thread is co-pinned to the
                    # rt_callback core so DDS message dispatch and state
                    # callback execution share L1/L2 cache. SCHED_OTHER is
                    # preserved — only affinity changes. SCHED_FIFO threads
                    # (rt_control / rt_callback / mpc_*) are skipped so this
                    # loop only touches non-RT (DDS / aux) threads.
                    f'taskset -cp {rt_callback_core} "$PID" 2>/dev/null; '
                    "PINNED=0; "
                    "for TID in $(ls /proc/$PID/task/ 2>/dev/null); do "
                    '  COMM=$(cat /proc/$PID/task/$TID/comm 2>/dev/null || echo ""); '
                    '  POLICY=$(chrt -p $TID 2>/dev/null | grep -o "SCHED_FIFO" || echo ""); '
                    '  if [ -n "$POLICY" ]; then continue; fi; '
                    f'  taskset -cp {rt_callback_core} "$TID" 2>/dev/null && PINNED=$((PINNED+1)); '
                    "done; "
                    f'echo "[RT] integrated_rt_controller (PID=$PID): $PINNED DDS/aux threads pinned to Core {rt_callback_core}"',
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
            )
        ],
    )

    # ── RT controller node ─────────────────────────────────────────────────────
    # The `enable_mpc` launch arg is declared below but takes effect through
    # the runtime gains topic (index 7) rather than a param override here —
    # robot_ur5e_p1b.launch.py does not use OpaqueFunction, and nested YAML overrides
    # would require restructuring the launch. The sim.launch.py flow does
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
            ft_inferencer_config,
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

    # ── Lifecycle auto-configure/activate for integrated_rt_controller ──────────────
    rt_auto_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=rt_controller_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda n: n == rt_controller_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
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
                rt_auto_activate,
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
            # 2) Environment
            set_session_dir,
            set_rmw,
            set_cyclone_uri,
            # 3) Infrastructure (parallel)
            enable_cpu_shield,
            ur_driver_launch_action,
            pin_ur_driver,
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
