# hand_udp.launch.py - v1
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='50001',
        description='UDP port for hand data reception'
    )
    
    target_ip_arg = DeclareLaunchArgument(
        'target_ip',
        default_value='192.168.1.100',
        description='Target IP for hand commands'
    )
    
    target_port_arg = DeclareLaunchArgument(
        'target_port',
        default_value='50002',
        description='Target port for hand commands'
    )
    
    # Get config file
    hand_config = PathJoinSubstitution([
        FindPackageShare('ur5e_rt_controller'),
        'config',
        'hand_udp_receiver.yaml'
    ])
    
    # Hand UDP receiver node
    receiver_node = Node(
        package='ur5e_rt_controller',
        executable='hand_udp_receiver_node',
        name='hand_udp_receiver',
        output='screen',
        parameters=[
            hand_config,
            {
                'udp_port': LaunchConfiguration('udp_port'),
            }
        ],
        emulate_tty=True,
    )
    
    # Hand UDP sender node
    sender_node = Node(
        package='ur5e_rt_controller',
        executable='hand_udp_sender_node',
        name='hand_udp_sender',
        output='screen',
        parameters=[{
            'target_ip': LaunchConfiguration('target_ip'),
            'target_port': LaunchConfiguration('target_port'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        udp_port_arg,
        target_ip_arg,
        target_port_arg,
        receiver_node,
        sender_node,
    ])
