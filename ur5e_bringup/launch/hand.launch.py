# hand.launch.py — HandUdpNode launch for UR5e bringup
# Request-response polling based unified hand UDP node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        description='Publish rate for /hand/joint_states (Hz)',
    )

    hand_config = PathJoinSubstitution([
        FindPackageShare('ur5e_hand_driver'),
        'config',
        'hand_udp_node.yaml',
    ])

    hand_udp_node = Node(
        package='ur5e_hand_driver',
        executable='hand_udp_node',
        name='hand_udp_node',
        output='screen',
        parameters=[
            hand_config,
            {
                'target_ip': LaunchConfiguration('target_ip'),
                'target_port': LaunchConfiguration('target_port'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        target_ip_arg,
        target_port_arg,
        publish_rate_arg,
        hand_udp_node,
    ])
