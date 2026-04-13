"""shape_estimation_node 단독 실행 launch 파일."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('shape_estimation')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'shape_estimation_node.yaml'
        ]),
        description='shape_estimation_node YAML 설정 파일 경로')

    shape_node = Node(
        package='shape_estimation',
        executable='shape_estimation_node',
        name='shape_estimation_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        shape_node,
    ])
