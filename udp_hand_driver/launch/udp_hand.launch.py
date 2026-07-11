# udp_hand.launch.py -- UdpHandNode 런치 (SIL 진입점 단일화)
# Event-driven request-response polling 기반 통합 핸드 UDP 노드.
#
# `sil_mode` (enum, default `off`) 하나로 두 상보적 SIL 계층을 선택한다:
#   off       — 실 하드웨어. use_fake_hand=false, firmware 미실행, target_ip=인자값.
#   loopmodel — Mode A (controller-side loop-model SIL). use_fake_hand=true
#               (노드 in-process 1차 LPF 모델), firmware 미실행, 소켓 미오픈.
#               thread/RT 구조를 실 모드와 동일하게 검증.
#   firmware  — Mode B (device-side network-level SIL). use_fake_hand=false +
#               fake_hand_firmware 조건부 실행 + target_ip:=127.0.0.1. 실 loopback
#               소켓으로 전체 transport(send/recv, framing, echo/MODE 검증, decode)를 검증.
#
# 두 직교 개념(in-process fake 여부 / 실 소켓 peer 존재)을 하나의 enum 으로 매핑한다.
# 노드 C++ 는 무변경 — mode 분기는 순수 launch(Python) 레이어.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
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


def generate_launch_description():
    pkg_share = FindPackageShare("udp_hand_driver")

    # ── Launch arguments ────────────────────────────────────────────────
    target_ip_arg = DeclareLaunchArgument(
        "target_ip",
        default_value="192.168.0.100",
        description="Target IP for hand controller (ignored when sil_mode=firmware "
        "→ auto-rewritten to 127.0.0.1)",
    )

    target_port_arg = DeclareLaunchArgument(
        "target_port",
        default_value="55151",
        description="Target port for hand controller (also firmware bind port when "
        "sil_mode=firmware)",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="100.0",
        description="Link status decimation base rate (Hz)",
    )

    communication_mode_arg = DeclareLaunchArgument(
        "communication_mode",
        default_value="bulk",
        description='Communication mode: "individual" or "bulk"',
    )

    recv_timeout_ms_arg = DeclareLaunchArgument(
        "recv_timeout_ms",
        default_value="0.4",
        description="ppoll recv timeout in ms (sub-ms supported)",
    )

    protocol_version_arg = DeclareLaunchArgument(
        "protocol_version",
        default_value="1a",
        description='Sensor protocol version: "1a" (int32 baro/ToF) or "1b" (float force). '
        "Propagated to both node and firmware (sil_mode=firmware).",
    )

    sil_mode_arg = DeclareLaunchArgument(
        "sil_mode",
        default_value="off",
        choices=["off", "loopmodel", "firmware"],
        description="SIL entry point. off = real hardware; loopmodel = controller-side "
        "in-process LPF (use_fake_hand, no socket); firmware = device-side "
        "fake_hand_firmware over loopback (target_ip auto 127.0.0.1).",
    )

    # ── Derived node params (sil_mode → use_fake_hand / target_ip) ───────
    # use_fake_hand must be a real bool param: wrap the PythonExpression in
    # ParameterValue(value_type=bool) so the "True"/"False" substitution string
    # is not mis-typed as a string param (launch gotcha).
    use_fake_hand = ParameterValue(
        PythonExpression(["'", LaunchConfiguration("sil_mode"), "' == 'loopmodel'"]),
        value_type=bool,
    )
    # firmware mode forces loopback; otherwise honour the target_ip argument.
    target_ip = PythonExpression(
        [
            "'127.0.0.1' if '",
            LaunchConfiguration("sil_mode"),
            "' == 'firmware' else '",
            LaunchConfiguration("target_ip"),
            "'",
        ]
    )

    # ── Config files ────────────────────────────────────────────────────
    hand_config = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "udp_hand_node.yaml",
        ]
    )

    ft_config = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "fingertip_ft_inferencer.yaml",
        ]
    )

    firmware_config = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "fake_hand_firmware.yaml",
        ]
    )

    # ── Device-side firmware simulator (sil_mode=firmware only) ──────────
    # Plain node with its own recv loop; binds target_port and echoes the
    # selected protocol_version so node/firmware stay wire-consistent.
    firmware_node = Node(
        package="udp_hand_driver",
        executable="fake_hand_firmware",
        name="fake_hand_firmware",
        namespace="",
        output="screen",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("sil_mode"), "firmware")),
        parameters=[
            firmware_config,
            {
                "port": LaunchConfiguration("target_port"),
                "protocol_version": LaunchConfiguration("protocol_version"),
            },
        ],
        emulate_tty=True,
    )

    # ── Lifecycle Node ──────────────────────────────────────────────────
    # `namespace=''` is required by launch_ros >= jazzy (keyword-only arg
    # in LifecycleNode.__init__); earlier distros defaulted it implicitly.
    udp_hand_node = LifecycleNode(
        package="udp_hand_driver",
        executable="udp_hand_node",
        name="udp_hand_node",
        namespace="",
        output="screen",
        parameters=[
            hand_config,
            ft_config,
            {
                "target_ip": target_ip,
                "target_port": LaunchConfiguration("target_port"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "communication_mode": LaunchConfiguration("communication_mode"),
                "recv_timeout_ms": LaunchConfiguration("recv_timeout_ms"),
                "protocol_version": LaunchConfiguration("protocol_version"),
                "use_fake_hand": use_fake_hand,
            },
        ],
        emulate_tty=True,
    )

    # ── Auto-configure → auto-activate chain ─────────────────────────
    auto_activate = RegisterEventHandler(
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
    trigger_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda n: n == udp_hand_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription(
        [
            target_ip_arg,
            target_port_arg,
            publish_rate_arg,
            communication_mode_arg,
            recv_timeout_ms_arg,
            protocol_version_arg,
            sil_mode_arg,
            firmware_node,
            udp_hand_node,
            auto_activate,
            trigger_configure,
        ]
    )
