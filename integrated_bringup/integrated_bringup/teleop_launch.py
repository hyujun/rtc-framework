"""``teleop:=`` of the humanoid sim launches: the pad / keyboard operation loop.

``teleop:=virtual|pad`` adds rtc_teleop_input (its own launch file, with the
profile's ``config/<profile>/teleop_input.yaml`` as ``params_file``) and the
``sim_recover`` node its recover button calls. ``none`` — the default — adds
nothing, so the default launch starts what it did before the argument existed.

``resolve_command_input`` decides sim_g1_loco's ``command_input`` from the
two arguments: ``auto`` (its default) is ``joy`` with a teleop and ``twist``
without; an explicit ``twist`` with a teleop is refused, since the controller
would then ignore every stick the operator moves.
"""

from __future__ import annotations

import os

TELEOP_MODES = ("none", "virtual", "pad")
COMMAND_AUTO = "auto"
TELEOP_PARAMS = "teleop_input.yaml"


def teleop_mode(value: str) -> str:
    value = value.strip() or "none"
    if value not in TELEOP_MODES:
        raise ValueError(f"teleop must be one of {TELEOP_MODES}, got '{value}'")
    return value


def resolve_command_input(command_input: str, teleop: str) -> str:
    """``command_input:=`` after ``auto``: ``twist`` or ``joy`` (validated later)."""
    teleop = teleop_mode(teleop)
    value = command_input.strip() or COMMAND_AUTO
    if value == COMMAND_AUTO:
        return "twist" if teleop == "none" else "joy"
    if value == "twist" and teleop != "none":
        raise ValueError(
            f"teleop:={teleop} publishes a Joy, but command_input:=twist makes the "
            "controller read a Twist; drop command_input (auto) or set it to joy"
        )
    return value


def joy_topic(group: str) -> str:
    """The Joy topic locomotion_controller reads (its ``<group>/joy``)."""
    return f"/locomotion_controller/{group}/joy"


def teleop_params_file(share_dir: str, profile: str) -> str:
    return os.path.join(share_dir, "config", profile, TELEOP_PARAMS)


def teleop_launch_arguments(teleop: str, share_dir: str, profile: str, group: str) -> dict:
    """rtc_teleop_input/teleop_input.launch.py's arguments, or {} for none."""
    teleop = teleop_mode(teleop)
    if teleop == "none":
        return {}
    return {
        "input": teleop,
        "joy_topic": joy_topic(group),
        "params_file": teleop_params_file(share_dir, profile),
    }


def sim_recover_parameters(profile: str, group: str) -> dict:
    """The ``sim_recover`` node's parameters for this profile."""
    return {"profile": profile, "group": group}


def teleop_actions(teleop: str, profile: str, group: str) -> list:
    """Launch actions for ``teleop:=``: the teleop launch and sim_recover, or []."""
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    arguments = teleop_launch_arguments(
        teleop, get_package_share_directory("integrated_bringup"), profile, group
    )
    if not arguments:
        return []
    teleop_launch = os.path.join(
        get_package_share_directory("rtc_teleop_input"), "launch", "teleop_input.launch.py"
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
            launch_arguments=arguments.items(),
        ),
        Node(
            package="integrated_bringup",
            executable="sim_recover",
            name="sim_recover",
            parameters=[sim_recover_parameters(profile, group)],
            output="screen",
        ),
    ]
