"""What ``teleop_input.launch.py`` starts, as plain data (tested without launch).

``game_controller_node`` gets its parameters from here, not from the joy
package's ``joy-params.yaml`` (deadzone 0.5 there). The values are the node's
own defaults, written out so that a joy upgrade cannot change them silently.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import yaml

GAME_CONTROLLER_PARAMETERS = {
    "deadzone": 0.05,
    "autorepeat_rate": 20.0,
    "sticky_buttons": False,
    "coalesce_interval_ms": 1,
}


# Launch arguments decide these (which nodes start, the pad driver's remap),
# so a params_file must not set them behind the launch's back.
LAUNCH_OWNED = ("input", "output_topic", "pad_topic")


def check_params_file(path: str) -> None:
    """Refuses a params file that sets a launch-owned parameter for any node."""
    with open(path) as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"params_file {path}: not a ROS parameter file")
    for node, section in data.items():
        params = (section or {}).get("ros__parameters", {}) if isinstance(section, dict) else {}
        owned = sorted(set(params) & set(LAUNCH_OWNED))
        if owned:
            raise ValueError(
                f"params_file {path}: '{node}' sets {', '.join(owned)} -- "
                "give these as launch arguments (input:=, joy_topic:=, pad_topic:=)"
            )


@dataclass(frozen=True)
class NodeSpec:
    package: str
    executable: str
    name: str
    parameters: dict = field(default_factory=dict)
    remappings: tuple[tuple[str, str], ...] = ()


def teleop_parameters(
    *,
    input: str,
    joy_topic: str,
    skin: str = "",
    controller_name: str = "",
    status_topics: str = "",
    rate_hz: str = "",
    pad_topic: str = "",
) -> dict:
    """Launch arguments (strings) -> node parameters. Empty = the node's default."""
    params: dict = {"input": input, "output_topic": joy_topic}
    if skin:
        params["skin"] = skin
    if controller_name:
        params["reset_controller_name"] = controller_name
    topics = [t.strip() for t in status_topics.split(",") if t.strip()]
    if topics:
        params["status_topics"] = topics
    if rate_hz:
        params["rate_hz"] = float(rate_hz)
    if pad_topic:
        params["pad_topic"] = pad_topic
    return params


def node_specs(arguments: dict[str, str]) -> list[NodeSpec]:
    """The nodes for these launch arguments: the input node, plus the pad driver for pad."""
    params = teleop_parameters(
        input=arguments["input"],
        joy_topic=arguments["joy_topic"],
        skin=arguments.get("skin", ""),
        controller_name=arguments.get("controller_name", ""),
        status_topics=arguments.get("status_topics", ""),
        rate_hz=arguments.get("rate_hz", ""),
        pad_topic=arguments.get("pad_topic", ""),
    )
    specs = [NodeSpec("rtc_teleop_input", "teleop_input", "teleop_input", params)]
    if arguments["input"] == "pad":
        # game_controller_node publishes "joy"; send it where the input node listens
        upstream = params.get("pad_topic", "joy")
        specs.append(
            NodeSpec(
                "joy",
                "game_controller_node",
                "game_controller_node",
                dict(GAME_CONTROLLER_PARAMETERS),
                (("joy", upstream),),
            )
        )
    return specs
