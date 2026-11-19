"""Teleop input: virtual pad GUI or real pad -> sensor_msgs/Joy.

    ros2 launch rtc_teleop_input teleop_input.launch.py joy_topic:=<topic> \
        [input:=virtual|pad] [skin:=f710|xbox] [controller_name:=<name>] \
        [status_topics:=<topic>,<topic>] [rate_hz:=50.0] [pad_topic:=joy] \
        [params_file:=<yaml>]

``joy_topic`` has no default (the consumer decides it). ``controller_name``
enables the reset_fault button for that controller. ``params_file`` sets any
other node parameter (key bindings, deflection, ...); an explicit launch
argument wins over it. What is started: ``rtc_teleop_input.launch_params``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rtc_teleop_input.launch_params import node_specs

_OPTIONAL = {
    "skin": "pad skin: f710 | xbox (node default f710)",
    "controller_name": "controller whose fault reset_fault clears; empty disables the button",
    "status_topics": "comma-separated diagnostic_msgs/DiagnosticStatus topics to show",
    "rate_hz": "Joy publish rate (node default 50)",
    "pad_topic": "upstream Joy of the real pad (node default joy)",
    "params_file": "YAML parameter file for the input node",
}


def _launch_setup(context):
    arguments = {
        name: LaunchConfiguration(name).perform(context)
        for name in ("input", "joy_topic", *_OPTIONAL)
    }
    actions = []
    for spec in node_specs(arguments):
        parameters = [spec.parameters]
        if spec.executable == "teleop_input" and arguments["params_file"]:
            parameters.insert(0, arguments["params_file"])
        actions.append(
            Node(
                package=spec.package,
                executable=spec.executable,
                name=spec.name,
                parameters=parameters,
                remappings=list(spec.remappings),
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_topic", description="Joy topic to publish (required)"),
            DeclareLaunchArgument(
                "input",
                default_value="virtual",
                choices=["virtual", "pad"],
                description="virtual pad GUI or a real pad",
            ),
            *(
                DeclareLaunchArgument(name, default_value="", description=text)
                for name, text in _OPTIONAL.items()
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
