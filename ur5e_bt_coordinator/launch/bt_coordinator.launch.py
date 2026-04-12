"""
bt_coordinator.launch.py — BehaviorTree coordinator launch file
================================================================

Usage:
  # Default (hand_motions.xml, YAML config + poses)
  ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py

  # Pick and Place (contact_stop grasp)
  ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place_contact_stop.xml

  # Pick and Place (Force-PI grasp)
  ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place_force_pi.xml

  # Towel Unfold
  ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=towel_unfold.xml

  # Hand Motions with repeat
  ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=hand_motions.xml repeat:=true

  # Groot2 visualization
  ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py groot2_port:=1667

  # Custom tick rate + paused start
  ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tick_rate:=60.0 paused:=true

Prerequisites:
  RT controller + simulator (or real robot) must be running:
    ros2 launch ur5e_bringup sim.launch.py
    ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ur5e_bt_coordinator')

    declared_args = [
        DeclareLaunchArgument(
            'tree', default_value='',
            description='BT tree XML filename (e.g. pick_and_place_contact_stop.xml). '
                        'Empty = use YAML default'),
        DeclareLaunchArgument(
            'tick_rate', default_value='0.0',
            description='BT tick rate [Hz]. 0 = use YAML default (80 Hz)'),
        DeclareLaunchArgument(
            'repeat', default_value='',
            description='Auto-repeat on SUCCESS (true/false). '
                        'Empty = use YAML default'),
        DeclareLaunchArgument(
            'repeat_delay', default_value='0.0',
            description='Delay before repeat [s]. 0 = use YAML default'),
        DeclareLaunchArgument(
            'paused', default_value='',
            description='Start paused (true/false). Empty = use YAML default'),
        DeclareLaunchArgument(
            'groot2_port', default_value='0',
            description='Groot2 ZMQ port (0 = disabled, 1667 = default)'),
        DeclareLaunchArgument(
            'grip', default_value='',
            description='Grip strength for pose-based grasp: soft, medium, hard. '
                        'Empty = use YAML default (medium)'),
    ]

    def launch_setup(context):
        config_yaml = PathJoinSubstitution(
            [pkg_share, 'config', 'bt_coordinator.yaml'])
        poses_yaml = PathJoinSubstitution(
            [pkg_share, 'config', 'poses.yaml'])

        # Resolve launch arguments
        tree = LaunchConfiguration('tree').perform(context)
        tick_rate = float(LaunchConfiguration('tick_rate').perform(context))
        repeat = LaunchConfiguration('repeat').perform(context)
        repeat_delay = float(
            LaunchConfiguration('repeat_delay').perform(context))
        paused = LaunchConfiguration('paused').perform(context)
        groot2_port = int(
            LaunchConfiguration('groot2_port').perform(context))
        grip = LaunchConfiguration('grip').perform(context)

        # Only override parameters that the user explicitly set
        overrides = {}
        if tree:
            overrides['tree_file'] = tree
        if tick_rate > 0.0:
            overrides['tick_rate_hz'] = tick_rate
        if repeat.lower() in ('true', 'false'):
            overrides['repeat'] = repeat.lower() == 'true'
        if repeat_delay > 0.0:
            overrides['repeat_delay_s'] = repeat_delay
        if paused.lower() in ('true', 'false'):
            overrides['paused'] = paused.lower() == 'true'
        if groot2_port > 0:
            overrides['groot2_port'] = groot2_port
        if grip in ('soft', 'medium', 'hard'):
            overrides['bb.hand_close_pose'] = f'hand_close_{grip}'

        bt_node = Node(
            package='ur5e_bt_coordinator',
            executable='bt_coordinator_node',
            name='bt_coordinator',
            output='screen',
            parameters=[config_yaml, poses_yaml, overrides],
        )
        return [bt_node]

    return LaunchDescription(
        declared_args + [OpaqueFunction(function=launch_setup)]
    )
