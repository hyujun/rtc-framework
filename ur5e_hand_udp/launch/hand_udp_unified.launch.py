# hand_udp_unified.launch.py - v2
# Launches the unified HandUdpNode (recv + send in one process).
# Replaces the legacy two-node launch (hand_udp.launch.py).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    recv_port_arg = DeclareLaunchArgument(
        'recv_port',
        default_value='50001',
        description='UDP port for hand state reception',
    )

    target_ip_arg = DeclareLaunchArgument(
        'target_ip',
        default_value='192.168.1.100',
        description='Target IP for hand motor commands',
    )

    target_port_arg = DeclareLaunchArgument(
        'target_port',
        default_value='50002',
        description='Target port for hand motor commands',
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publish rate for /hand/joint_states (Hz)',
    )

    hand_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_udp'),
        'config',
        'hand_udp.yaml',
    ])

    hand_udp_node = Node(
        package='ur5e_hand_udp',
        executable='hand_udp_node',
        name='hand_udp_node',
        output='screen',
        parameters=[
            hand_config,
            {
                'recv_port': LaunchConfiguration('recv_port'),
                'target_ip': LaunchConfiguration('target_ip'),
                'target_port': LaunchConfiguration('target_port'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        recv_port_arg,
        target_ip_arg,
        target_port_arg,
        publish_rate_arg,
        hand_udp_node,
    ])
