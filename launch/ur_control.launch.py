# ur_control.launch.py - v1
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the UR robot'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing'
    )
    
    # Get package paths
    ur_control_config = PathJoinSubstitution([
        FindPackageShare('ur5e_rt_controller'),
        'config',
        'ur5e_rt_controller.yaml'
    ])
    
    # UR robot driver launch
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
        }.items()
    )
    
    # Custom controller node
    custom_controller_node = Node(
        package='ur5e_rt_controller',
        executable='custom_controller',
        name='custom_controller',
        output='screen',
        parameters=[ur_control_config],
        emulate_tty=True,
    )
    
    # Data health monitor
    monitor_node = Node(
        package='ur5e_rt_controller',
        executable='monitor_data_health_v2.py',
        name='data_health_monitor',
        output='screen',
        parameters=[{
            'check_rate': 10.0,
            'timeout_threshold': 0.2,
        }],
    )
    
    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        ur_driver_launch,
        custom_controller_node,
        monitor_node,
    ])
