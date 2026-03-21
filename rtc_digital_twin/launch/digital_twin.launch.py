"""Launch file for UR5e + Hand Digital Twin RViz2 visualization.

Nodes launched:
  1. robot_state_publisher — loads combined URDF, subscribes to
     /digital_twin/joint_states, publishes TF tree
  2. digital_twin_node — subscribes to /joint_states & /hand/joint_states,
     publishes combined JointState + MarkerArray at display_rate Hz
  3. rviz2 (optional) — RViz2 with pre-configured display settings

Usage:
  ros2 launch rtc_digital_twin digital_twin.launch.py
  ros2 launch rtc_digital_twin digital_twin.launch.py enable_hand:=false
  ros2 launch rtc_digital_twin digital_twin.launch.py display_rate:=30.0
  ros2 launch rtc_digital_twin digital_twin.launch.py use_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Paths ────────────────────────────────────────────────────────────
    description_share = FindPackageShare('ur5e_description')
    digital_twin_share = FindPackageShare('rtc_digital_twin')

    urdf_xacro_path = PathJoinSubstitution([
        description_share, 'robots', 'ur5e', 'urdf', 'ur5e_with_hand.urdf.xacro',
    ])

    config_yaml_path = PathJoinSubstitution([
        digital_twin_share, 'config', 'digital_twin.yaml',
    ])

    rviz_config_path = PathJoinSubstitution([
        digital_twin_share, 'config', 'digital_twin.rviz',
    ])

    # ── Launch arguments ─────────────────────────────────────────────────
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 with pre-configured display',
    )
    declare_display_rate = DeclareLaunchArgument(
        'display_rate', default_value='60.0',
        description='RViz display refresh rate (Hz)',
    )
    declare_enable_hand = DeclareLaunchArgument(
        'enable_hand', default_value='true',
        description='Enable hand visualization',
    )

    use_rviz = LaunchConfiguration('use_rviz')
    display_rate = LaunchConfiguration('display_rate')
    enable_hand = LaunchConfiguration('enable_hand')

    # ── xacro → URDF string ─────────────────────────────────────────────
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_xacro_path,
    ])

    # ── robot_state_publisher ────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='digital_twin',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 60.0,
        }],
        remappings=[
            ('joint_states', '/digital_twin/joint_states'),
        ],
        output='screen',
    )

    # ── digital_twin_node ────────────────────────────────────────────────
    digital_twin_node = Node(
        package='rtc_digital_twin',
        executable='digital_twin_node',
        name='digital_twin_node',
        parameters=[
            config_yaml_path,
            {
                'display_rate': display_rate,
                'enable_hand': enable_hand,
            },
        ],
        output='screen',
    )

    # ── rviz2 (conditional) ──────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_display_rate,
        declare_enable_hand,
        robot_state_publisher_node,
        digital_twin_node,
        rviz_node,
    ])
