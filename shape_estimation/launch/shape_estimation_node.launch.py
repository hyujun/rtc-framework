"""shape_estimation_node 단독 실행 launch 파일."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = FindPackageShare('shape_estimation')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'shape_estimation_node.yaml'
        ]),
        description='shape_estimation_node YAML 설정 파일 경로')

    # `namespace=''` is required by launch_ros >= jazzy (keyword-only arg
    # in LifecycleNode.__init__); earlier distros defaulted it implicitly.
    shape_node = LifecycleNode(
        package='shape_estimation',
        executable='shape_estimation_node',
        name='shape_estimation_node',
        namespace='',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
    )

    auto_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=shape_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda n: n == shape_node,
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )
    )
    trigger_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == shape_node,
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    return LaunchDescription([
        config_file_arg,
        shape_node,
        auto_activate,
        trigger_configure,
    ])
