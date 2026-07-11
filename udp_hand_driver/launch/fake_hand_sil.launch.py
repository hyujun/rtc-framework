# fake_hand_sil.launch.py -- full-SIL loopback: fake_hand_firmware + real udp_hand_node.
#
# Starts the device-side firmware simulator (fake_hand_firmware) and an unmodified
# udp_hand_node (use_fake_hand:=false, target_ip:=127.0.0.1) talking to it over the
# loopback UDP socket. This exercises the entire transport path (send/recv,
# framing, echo/MODE validation, failure detection, decode, publish) with no
# hardware — complementary to the controller's own use_fake_hand (loop-model SIL).
#
# The udp_hand_node lifecycle is auto-configured then auto-activated (same chain
# as udp_hand.launch.py).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = FindPackageShare("udp_hand_driver")

    # ── Launch arguments ────────────────────────────────────────────────
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="55151",
        description="Loopback UDP port shared by firmware (bind) and node (target_port)",
    )
    protocol_version_arg = DeclareLaunchArgument(
        "protocol_version",
        default_value="1a",
        description='Sensor protocol version: "1a" (int32 baro/ToF) or "1b" (float force)',
    )

    # ── Config files ────────────────────────────────────────────────────
    firmware_config = PathJoinSubstitution([pkg_share, "config", "fake_hand_firmware.yaml"])
    hand_config = PathJoinSubstitution([pkg_share, "config", "udp_hand_node.yaml"])
    ft_config = PathJoinSubstitution([pkg_share, "config", "fingertip_ft_inferencer.yaml"])

    # ── Device-side firmware simulator (plain node, own recv loop) ──────
    firmware_node = Node(
        package="udp_hand_driver",
        executable="fake_hand_firmware",
        name="fake_hand_firmware",
        namespace="",
        output="screen",
        parameters=[
            firmware_config,
            {
                "port": LaunchConfiguration("port"),
                "protocol_version": LaunchConfiguration("protocol_version"),
            },
        ],
        emulate_tty=True,
    )

    # ── Real udp_hand_node against 127.0.0.1 (use_fake_hand:=false) ─────
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
                "use_fake_hand": False,
                "target_ip": "127.0.0.1",
                "target_port": LaunchConfiguration("port"),
                "protocol_version": LaunchConfiguration("protocol_version"),
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
            port_arg,
            protocol_version_arg,
            firmware_node,
            udp_hand_node,
            auto_activate,
            trigger_configure,
        ]
    )
