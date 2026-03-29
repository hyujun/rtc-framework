# hand_udp.launch.py -- HandUdpNode 런치
# Event-driven request-response polling 기반 통합 핸드 UDP 노드
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ur5e_hand_driver')

    # ── Launch arguments ────────────────────────────────────────────────
    target_ip_arg = DeclareLaunchArgument(
        'target_ip',
        default_value='192.168.1.2',
        description='Target IP for hand controller',
    )

    target_port_arg = DeclareLaunchArgument(
        'target_port',
        default_value='55151',
        description='Target port for hand controller',
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Link status decimation base rate (Hz)',
    )

    communication_mode_arg = DeclareLaunchArgument(
        'communication_mode',
        default_value='bulk',
        description='Communication mode: "individual" or "bulk"',
    )

    recv_timeout_ms_arg = DeclareLaunchArgument(
        'recv_timeout_ms',
        default_value='0.4',
        description='ppoll recv timeout in ms (sub-ms supported)',
    )

    use_fake_hand_arg = DeclareLaunchArgument(
        'use_fake_hand',
        default_value='false',
        description='Use fake hand echo-back mock (no UDP socket)',
    )

    # ── Config files ────────────────────────────────────────────────────
    hand_config = PathJoinSubstitution([
        pkg_share, 'config', 'hand_udp_node.yaml',
    ])

    ft_config = PathJoinSubstitution([
        pkg_share, 'config', 'fingertip_ft_inferencer.yaml',
    ])

    # ── Node ────────────────────────────────────────────────────────────
    hand_udp_node = Node(
        package='ur5e_hand_driver',
        executable='hand_udp_node',
        name='hand_udp_node',
        output='screen',
        parameters=[
            hand_config,
            ft_config,
            {
                'target_ip': LaunchConfiguration('target_ip'),
                'target_port': LaunchConfiguration('target_port'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'communication_mode': LaunchConfiguration('communication_mode'),
                'recv_timeout_ms': LaunchConfiguration('recv_timeout_ms'),
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        target_ip_arg,
        target_port_arg,
        publish_rate_arg,
        communication_mode_arg,
        recv_timeout_ms_arg,
        use_fake_hand_arg,
        hand_udp_node,
    ])
