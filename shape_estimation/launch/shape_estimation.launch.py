"""shape_estimation_node + rviz2를 함께 실행하는 launch 파일."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('shape_estimation')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='RViz2 시각화 실행 여부')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'shape_estimation.rviz'
        ]),
        description='RViz 설정 파일 경로')

    shape_node = Node(
        package='shape_estimation',
        executable='shape_estimation_node',
        name='shape_estimation_node',
        parameters=[{
            'voxel_resolution': 0.002,
            'max_points': 2048,
            'point_expiry_sec': 5.0,
            'flat_curvature_threshold': 5.0,
            'curvature_uniformity_threshold': 2.0,
            'min_points_for_fitting': 10,
            'publish_rate_hz': 10.0,
            'viz_rate_hz': 5.0,
            'frame_id': 'base_link',
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        shape_node,
        rviz_node,
    ])
