"""Every topic ``udp_hand_node`` declares must be set, absolutely, in each robot overlay.

**The failure this exists for is silent.** The robot launches start
``udp_hand_node`` with ``namespace=""``, and the node declares every topic as a
*relative* name (``"estop"``, ``"joint_states"``, ...) so the standalone launch
can push them under ``/hand``. A topic key an overlay leaves out therefore
falls back to that relative default and resolves to a *global* name — ``/estop``
rather than ``/p1b/estop``. Nothing fails: the node starts, the topic exists,
and it is wired to nothing the robot publishes or reads. ``estop_topic`` shipped
that way in both overlays.

Checking only the keys an overlay already has would miss exactly that case, so
the list of keys comes from the node: the ``"*_topic"`` string literals that
``udp_hand_node_lifecycle.cpp`` declares (``declare_parameter`` or its
re-configure guard ``declare_once``). Scanning C++ as text is the
weaker instrument and is guarded the way ``test_shipped_sim_param_declared``
guards it — comments are stripped first, and the scan must find known keys
before its result is trusted, so a regex that stops matching fails loudly
instead of passing vacuously.

A hand config run through ``udp_hand.launch.py`` (``params_file`` +
``namespace:=``) is the opposite case: there the namespace is the prefix, and
with two hands an absolute per-hand topic is refused at configure (both hands
would share it). Those files must leave every topic relative.
"""

from __future__ import annotations

import os
import re

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

# Robot profile → (overlay file a robot launch hands to udp_hand_node
# (namespace=""), the robot namespace every topic in it must sit under). The
# namespace is the robot's, not the profile key's: p2_left is the p2 robot.
OVERLAYS = {
    "ur5e_p1a": ("udp_hand_node_p1a.yaml", "/p1a/"),
    "ur5e_p1b": ("udp_hand_node_p1b.yaml", "/p1b/"),
    "p2": ("udp_hand_node_p2.yaml", "/p2/"),
    "p2_left": ("udp_hand_node_p2.yaml", "/p2/"),
}

# Profile → hand config run through udp_hand.launch.py params_file with namespace:=.
NAMESPACED_CONFIGS = {
    "b7_p2": "udp_hand_node_p2.yaml",
}

# Topic keys that have shipped in both overlays for a long time; their absence
# from the scan means the scanner broke, not the config.
SCANNER_CANARIES = {"command_topic", "joint_state_topic"}


def _node_source() -> str:
    """``udp_hand_node_lifecycle.cpp`` with comments stripped (repo source, not install)."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(os.path.dirname(here))
    path = os.path.join(repo, "udp_hand_driver", "src", "udp_hand_node_lifecycle.cpp")
    with open(path) as handle:
        text = handle.read()
    # One left-to-right pass over strings and both comment kinds: stripping block
    # comments first would read the `/*` in a line comment's "/hand/*" as an
    # opener and swallow everything up to the next `*/`.
    token = re.compile(r'"(?:\\.|[^"\\\n])*"|//[^\n]*|/\*.*?\*/', re.DOTALL)
    return token.sub(lambda m: m.group(0) if m.group(0).startswith('"') else "", text)


@pytest.fixture(scope="module")
def declared_topics() -> set[str]:
    keys = set(re.findall(r'declare_(?:parameter|once)\(\s*"([^"]+_topic)"', _node_source()))
    missing = SCANNER_CANARIES - keys
    assert not missing, (
        f"declare_parameter scan found none of {sorted(missing)} — the scanner is "
        f"broken, not the config. Found {sorted(keys)}."
    )
    # The key this test exists for; if the scan stops seeing it (declared through
    # a constant, moved to another file), the coverage went with it.
    assert "estop_topic" in keys, f"scan no longer finds estop_topic — test is vacuous: {keys}"
    return keys


def _params(profile: str, filename: str) -> dict:
    share = get_package_share_directory("integrated_bringup")
    with open(os.path.join(share, "config", profile, filename)) as handle:
        return yaml.safe_load(handle)["/**"]["ros__parameters"]


@pytest.mark.parametrize("profile", sorted(OVERLAYS))
def test_every_declared_topic_is_set_under_the_robot_namespace(
    profile: str, declared_topics: set[str]
) -> None:
    filename, prefix = OVERLAYS[profile]
    params = _params(profile, filename)
    for key in sorted(declared_topics):
        assert key in params, (
            f"{filename}: '{key}' is not set — under namespace=\"\" the "
            "node's relative default resolves to a global topic nothing on this "
            "robot uses"
        )
        value = params[key]
        assert isinstance(value, str) and value.startswith(prefix), (
            f"{filename}: '{key}' = {value!r} must be an absolute name under {prefix}"
        )


@pytest.mark.parametrize("profile", sorted(NAMESPACED_CONFIGS))
def test_namespaced_config_leaves_every_topic_relative(
    profile: str, declared_topics: set[str]
) -> None:
    filename = NAMESPACED_CONFIGS[profile]
    params = _params(profile, filename)
    absolute = {
        key: params[key]
        for key in sorted(declared_topics)
        if isinstance(params.get(key), str) and params[key].startswith("/")
    }
    assert not absolute, (
        f"{filename}: {absolute} — this file runs under namespace:=, where an absolute "
        "topic escapes the namespace (and two hands refuse it at configure)"
    )


def test_p2_sides_agree_on_the_launch_build_time_values() -> None:
    """robot_p2 / sim_p2 ``side:=`` picks config/p2 or config/p2_left at run
    time, but a launch argument's default is built before any argument is read,
    so the right hand's files supply ``sil_mode`` / ``max_log_sessions`` and
    the fake firmware's ``protocol_version`` / ``cmd_ack`` for both sides. That
    holds only while the two profiles agree on them; the fake's ``hand_side``
    is the ``side`` argument itself, so each profile's driver must name its
    own side."""
    right = _params("p2", "udp_hand_node_p2.yaml")
    left = _params("p2_left", "udp_hand_node_p2.yaml")
    for key in ("sil_mode", "protocol_version", "cmd_ack"):
        assert right[key] == left[key], key
    assert right["hand_sides"] == ["right"]
    assert left["hand_sides"] == ["left"]
    assert (
        _params("p2", "_base.yaml")["max_log_sessions"]
        == _params("p2_left", "_base.yaml")["max_log_sessions"]
    )
