"""Generalized Digital Twin launch file.

Launches:
  1. robot_state_publisher — loads URDF/xacro, subscribes to
     /digital_twin/joint_states, publishes TF tree
  2. digital_twin_node — subscribes to configurable JointState sources,
     merges and publishes combined JointState, validates URDF joints
  3. rviz2 (optional) — RViz2 with pre-configured display settings

Usage:
  # Package-based URDF (UR5e + hand)
  ros2 launch rtc_digital_twin digital_twin.launch.py \
      robot_description_package:=ur5e_description \
      robot_description_path:=robots/ur5e/urdf/ur5e_with_hand.urdf.xacro

  # Absolute path URDF
  ros2 launch rtc_digital_twin digital_twin.launch.py \
      robot_description_file:=/path/to/robot.urdf

  # Custom config + no RViz
  ros2 launch rtc_digital_twin digital_twin.launch.py \
      robot_description_package:=ur5e_description \
      robot_description_path:=robots/ur5e/urdf/ur5e.urdf \
      config_file:=/path/to/my_config.yaml \
      use_rviz:=false
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # ── Resolve robot description ─────────────────────────────────────────
    desc_file = LaunchConfiguration('robot_description_file').perform(context)
    desc_pkg = LaunchConfiguration('robot_description_package').perform(context)
    desc_path = LaunchConfiguration('robot_description_path').perform(context)

    # Fallback to YAML config if launch args are empty
    if not desc_file and not desc_pkg and not desc_path:
        config_file = LaunchConfiguration('config_file').perform(context)
        if not config_file:
            config_file = os.path.join(
                get_package_share_directory('rtc_digital_twin'),
                'config', 'digital_twin.yaml')

        import yaml
        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)
        params = cfg.get('/**', {}).get('ros__parameters', {})
        desc_file = params.get('robot_description_file', '')
        desc_pkg = params.get('robot_description_package', '')
        desc_path = params.get('robot_description_path', '')

    if desc_file:
        urdf_path = desc_file
    elif desc_pkg and desc_path:
        pkg_share = get_package_share_directory(desc_pkg)
        urdf_path = os.path.join(pkg_share, desc_path)
    else:
        raise RuntimeError(
            'Must provide robot_description_file OR '
            'robot_description_package + robot_description_path. '
            'Set via launch args or in the YAML config file.')

    if not os.path.isfile(urdf_path):
        raise RuntimeError(f'Robot description file not found: {urdf_path}')

    # ── Process xacro or read URDF directly ───────────────────────────────
    if urdf_path.endswith('.xacro'):
        robot_description = subprocess.check_output(
            ['xacro', urdf_path], text=True)
    else:
        with open(urdf_path, 'r') as f:
            robot_description = f.read()

    # ── Config file ───────────────────────────────────────────────────────
    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        config_file = os.path.join(
            get_package_share_directory('rtc_digital_twin'),
            'config', 'digital_twin.yaml')

    # ── RViz config ───────────────────────────────────────────────────────
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if not rviz_config:
        rviz_config = os.path.join(
            get_package_share_directory('rtc_digital_twin'),
            'config', 'digital_twin.rviz')

    # ── Display rate override ─────────────────────────────────────────────
    display_rate = LaunchConfiguration('display_rate').perform(context)
    dt_overrides = {}
    if display_rate:
        dt_overrides['display_rate'] = float(display_rate)

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='digital_twin',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 60.0,
        }],
        remappings=[
            ('joint_states', '/digital_twin/joint_states'),
        ],
        output='screen',
    )

    # ── digital_twin_node ─────────────────────────────────────────────────
    dt_params = [
        config_file,
        {'robot_description': robot_description},
    ]
    if dt_overrides:
        dt_params.append(dt_overrides)

    digital_twin_node = Node(
        package='rtc_digital_twin',
        executable='digital_twin_node',
        name='digital_twin_node',
        parameters=dt_params,
        output='screen',
    )

    # ── rviz2 (conditional) ───────────────────────────────────────────────
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return [robot_state_publisher_node, digital_twin_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description_file', default_value='',
            description='Absolute path to URDF/xacro file',
        ),
        DeclareLaunchArgument(
            'robot_description_package', default_value='',
            description='Package containing the robot description',
        ),
        DeclareLaunchArgument(
            'robot_description_path', default_value='',
            description='Relative path within the description package',
        ),
        DeclareLaunchArgument(
            'config_file', default_value='',
            description='Path to digital_twin YAML config (empty = default)',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description='Path to RViz config file (empty = default)',
        ),
        DeclareLaunchArgument(
            'display_rate', default_value='',
            description='Override display_rate from YAML',
        ),
        OpaqueFunction(function=launch_setup),
    ])
