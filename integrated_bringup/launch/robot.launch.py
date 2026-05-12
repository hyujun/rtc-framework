# robot.launch.py - UR5e real robot bringup
#
# Launch order (event-driven):
#   1. Environment vars (RMW, CycloneDDS, session dir)
#   2. CPU shield, UR driver, udp_hand_node  (parallel)
#   3. Readiness gate: polls /joint_states and /hand/joint_states publishers
#   4. integrated_rt_controller node starts ONLY after gate exits successfully
#   5. DDS thread pinning runs 5 s after the CM process starts
#
# Core allocation optimizations applied:
#   C) UR driver process pinned to Core 0-1 via delayed taskset (use_cpu_affinity:=true)
#   D) integrated_rt_controller DDS threads pinned to Core 0-1 (prevents 100-350us jitter on Jazzy)
#   E) CycloneDDS threads restricted to Core 0-1 via CYCLONEDDS_URI env var

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

from rtc_tools.utils.session_dir import (
    cleanup_old_sessions,
    create_session_dir,
    resolve_logging_root,
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
            "initial_joint_controller": "forward_position_controller",
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

    enable_perf_arg = DeclareLaunchArgument(
        "enable_perf",
        default_value="false",
        description=(
            "Capture Linux perf data for the rt-controller, MPC and hand UDP threads. "
            "Output: <session>/perf/perf.data (open with `hotspot <path>`). "
            "Requires perf_event_paranoid<=1 OR passwordless sudo — "
            "run ./install.sh --perf for one-time setup."
        ),
    )

    perf_targets_arg = DeclareLaunchArgument(
        "perf_targets",
        default_value="integrated_rt_controller|udp_hand_node|ur_ros2_control_node",
        description=(
            "Regex of process names to attach perf record to. "
            "Matched via `pgrep -f`. Only used when enable_perf:=true."
        ),
    )

    perf_duration_arg = DeclareLaunchArgument(
        "perf_duration",
        default_value="",
        description=(
            "Capture duration in seconds. Empty = run until launch SIGINT "
            "(perf flushes on Ctrl+C)."
        ),
    )

    perf_start_delay_arg = DeclareLaunchArgument(
        "perf_start_delay",
        default_value="0",
        description=(
            "Seconds to wait after launch before invoking perf. Default 0 = "
            "start immediately (captures lifecycle bring-up). Set 5+ to skip "
            "configure/activate noise and only profile steady-state."
        ),
    )

    perf_stack_size_arg = DeclareLaunchArgument(
        "perf_stack_size",
        default_value="4096",
        description=(
            "DWARF unwind stack size (bytes) per sample. Default 4096 keeps "
            "Hotspot loading fast; raise to 8192/16384 if deep template stacks "
            "appear truncated in flame graphs. Trace size scales linearly."
        ),
    )

    perf_frequency_arg = DeclareLaunchArgument(
        "perf_frequency",
        default_value="999",
        description=(
            "perf sampling frequency in Hz. Default 999 gives sub-millisecond "
            "resolution suitable for RT tick analysis. Lower to 99 for lighter "
            "exploratory captures (~10× smaller trace)."
        ),
    )

    perf_event_arg = DeclareLaunchArgument(
        "perf_event",
        default_value="cycles:P",
        description=(
            "perf event to sample on. Default 'cycles:P' weights samples by CPU "
            "cycles consumed — accurate for hot CPU users (mpc_main, hand_udp) "
            "but under-samples burst-then-sleep RT loops (rt_control 30 µs / 2 ms). "
            "Use 'task-clock' for periodic RT loops to get more uniform sampling "
            "across active intervals regardless of CPU intensity."
        ),
    )

    # Closure-bound helper: perf_action.make_perf_action needs a launch context.
    # session_dir is captured from the enclosing scope.
    def _build_perf_action(context):
        return make_perf_action(context, session_dir=session_dir)

    perf_action = OpaqueFunction(function=_build_perf_action)

    # ── Paths ──────────────────────────────────────────────────────────────────
    ur_control_config = PathJoinSubstitution(
        [FindPackageShare("integrated_bringup"), "config", "ur5e_hand", "robot.yaml"]
    )

    # Hand UDP config (udp_hand_driver package)
    hand_udp_config = PathJoinSubstitution(
        [FindPackageShare("udp_hand_driver"), "config", "udp_hand_node.yaml"]
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

    # ── UR driver CPU pinning ─────────────────────────────────────────────────
    pin_ur_driver = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    'PID=$(pgrep -nf ur_ros2_driver) && [ -n "$PID" ] && '
                    'taskset -cp 0-1 "$PID" && '
                    'echo "[RT] ur_ros2_driver (PID=$PID) pinned to Core 0-1" || '
                    'echo "[RT] WARNING: ur_ros2_driver not found — CPU pinning skipped"',
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
            )
        ],
    )

    # ── integrated_rt_controller DDS thread pinning ─────────────────────────────────
    # exec name = ROS node name = "integrated_rt_controller" (Phase 3 정렬).
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
                    'taskset -cp 0-1 "$PID" 2>/dev/null; '
                    "PINNED=0; "
                    "for TID in $(ls /proc/$PID/task/ 2>/dev/null); do "
                    '  COMM=$(cat /proc/$PID/task/$TID/comm 2>/dev/null || echo ""); '
                    '  POLICY=$(chrt -p $TID 2>/dev/null | grep -o "SCHED_FIFO" || echo ""); '
                    '  if [ -n "$POLICY" ]; then continue; fi; '
                    '  taskset -cp 0-1 "$TID" 2>/dev/null && PINNED=$((PINNED+1)); '
                    "done; "
                    'echo "[RT] integrated_rt_controller (PID=$PID): $PINNED DDS/aux threads pinned to Core 0-1"',
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_cpu_affinity")),
            )
        ],
    )

    # ── RT controller node ─────────────────────────────────────────────────────
    # The `enable_mpc` launch arg is declared below but takes effect through
    # the runtime gains topic (index 7) rather than a param override here —
    # robot.launch.py does not use OpaqueFunction, and nested YAML overrides
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
            ur_control_config,
            {
                "log_dir": session_dir,
            },
        ],
        emulate_tty=True,
    )

    # ── Hand UDP driver node (LifecycleNode) ──────────────────────────────────
    # Publishes /hand/joint_states and /hand/sensor_states for integrated_rt_controller.
    udp_hand_node = LifecycleNode(
        package="udp_hand_driver",
        executable="udp_hand_node",
        name="udp_hand_node",
        namespace="",
        output="screen",
        parameters=[
            hand_udp_config,
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
    # /hand/joint_states.  integrated_rt_controller is chained to start only after
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
            # --- Wait for /hand/joint_states publisher (udp_hand_node) ---
            "timeout 30 bash -c '"
            "while ! ros2 topic info /hand/joint_states 2>/dev/null "
            '  | grep -q "Publisher count: [1-9]"; do sleep 0.5; done\' '
            '  && echo "[RT]   /hand/joint_states publisher OK" '
            '  || { echo "[RT] FATAL: /hand/joint_states not available after 30 s"; exit 1; }; '
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
            enable_mpc_arg,
            enable_perf_arg,
            perf_targets_arg,
            perf_duration_arg,
            perf_start_delay_arg,
            perf_stack_size_arg,
            perf_frequency_arg,
            perf_event_arg,
            # 2) Environment
            set_session_dir,
            set_rmw,
            set_cyclone_uri,
            # 3) Infrastructure (parallel)
            enable_cpu_shield,
            ur_driver_launch_action,
            pin_ur_driver,
            udp_hand_node,
            hand_auto_activate,
            hand_trigger_configure,
            # 4) Event-driven chain:
            #    udp_hand_node started → comm_readiness_gate
            #    → gate exits OK → integrated_rt_controller
            #    → CM process started → pin_rt_controller_dds
            start_gate_after_hand,
            start_rt_after_gate,
            start_dds_pin_after_rt,
            # 5) Profiling (no-op when enable_perf:=false)
            perf_action,
        ]
    )
