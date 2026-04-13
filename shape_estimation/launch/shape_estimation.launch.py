"""shape_estimation_node + rviz2를 함께 실행하는 통합 launch 파일.

개별 실행:
  ros2 launch shape_estimation shape_estimation_node.launch.py   # 노드만
  ros2 launch shape_estimation shape_estimation_rviz.launch.py   # RViz만
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('shape_estimation')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='RViz2 시각화 실행 여부')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'shape_estimation_node.yaml'
        ]),
        description='shape_estimation_node YAML 설정 파일 경로')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'shape_estimation.rviz'
        ]),
        description='RViz 설정 파일 경로')

    node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, 'launch', 'shape_estimation_node.launch.py'
            ])
        ),
        launch_arguments={'config_file': LaunchConfiguration('config_file')}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, 'launch', 'shape_estimation_rviz.launch.py'
            ])
        ),
        launch_arguments={'rviz_config': LaunchConfiguration('rviz_config')}.items(),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        config_file_arg,
        rviz_config_arg,
        node_launch,
        rviz_launch,
    ])
