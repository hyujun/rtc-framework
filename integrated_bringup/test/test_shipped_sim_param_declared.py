"""Every shipped ``mujoco_simulator.yaml`` key must be one the node DECLARES.

**The failure this exists for is silent.** ``rclcpp`` returns nothing useful for
an undeclared parameter, ``mujoco_simulator_node`` wraps its group reads in
try/catch, and the config struct then keeps its compiled-in default. So a key
that is spelled correctly in YAML, documented in the README, and reviewed by a
human still does *nothing* — the node starts, every topic appears, and the only
trace is one word in a startup log line nobody is grepping for.

That is not hypothetical. ``contact_wrench.reference_frame`` shipped as
``"site"`` in ``ur5e_p1b`` and ran as ``"body"`` — publishing every fingertip
wrench rotated 90 degrees into the wrong frame — because the key was added to
the config struct, the YAML, the README and the unit tests, but not to
``DeclareGroupParams``. Only a runtime launch caught it. This test is that
launch, made cheap.

**Why it reads the C++ as text.** The declaration list is not exported anywhere
a Python test can query without standing up a ROS node, and standing one up
needs MuJoCo, an MJCF and a working DDS — none of which a config-lint test
should require. Scanning ``declare_parameter("...")`` string literals is the
weaker instrument, and it is guarded accordingly: comments are stripped before
scanning (so this file cannot pass by reading a commented-out declare), and the
extracted set is checked against known-present keys before it is used, so a
regex that silently stops matching fails loudly instead of vacuously passing.
"""

from __future__ import annotations

import os
import re

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

PROFILES = ["ur5e_p1a", "ur5e_p1b", "iiwa7_leap"]

# Blocks whose keys are checked. Both are nested dictionaries in YAML but flat
# dotted names in declare_parameter, which is exactly the seam a key falls
# through: the YAML nesting looks right on its own.
GROUP_BLOCKS = ["contact_wrench"]
TOPLEVEL_BLOCKS = ["object_state", "object_pool"]

# Keys that must appear in the scan for it to be considered working. Chosen
# because they have shipped for a long time and are not what this test is
# about, so their absence means the scanner broke rather than a real regression.
SCANNER_CANARIES = {
    "contact_wrench.enabled",
    "contact_wrench.topic_prefix",
    "object_pool.enabled",
    "model_path",
}


def _node_source() -> str:
    """``mujoco_simulator_node.cpp`` with comments stripped.

    Source lives in the repo, not the install tree, so it is resolved relative
    to this file rather than through ament.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(os.path.dirname(here))
    path = os.path.join(repo, "rtc_mujoco_sim", "src", "mujoco_simulator_node.cpp")
    with open(path) as handle:
        text = handle.read()
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return "\n".join(re.sub(r"//.*$", "", line) for line in text.splitlines())


def _declared_keys() -> set[str]:
    """Parameter names appearing in a ``declare_parameter("...")`` call.

    Group parameters are declared as ``prefix + "<name>"`` where prefix is
    built at runtime, so what this captures is the *suffix* — which is the part
    a YAML key contributes and the part that goes missing.
    """
    return set(re.findall(r'declare_parameter\(\s*(?:prefix\s*\+\s*)?"([^"]+)"', _node_source()))


@pytest.fixture(scope="module")
def declared() -> set[str]:
    keys = _declared_keys()
    missing = SCANNER_CANARIES - keys
    assert not missing, (
        "declare_parameter scan found neither of "
        f"{sorted(missing)} — the scanner is broken, not the config. "
        f"Found {len(keys)} keys."
    )
    return keys


def _params(profile: str) -> dict:
    share = get_package_share_directory("integrated_bringup")
    with open(os.path.join(share, "config", profile, "mujoco_simulator.yaml")) as handle:
        return yaml.safe_load(handle)["mujoco_simulator"]["ros__parameters"]


@pytest.mark.parametrize("profile", PROFILES)
def test_group_block_keys_are_declared(profile: str, declared: set[str]) -> None:
    response = _params(profile).get("robot_response", {})
    checked = 0
    for group in response.get("groups", []):
        for block in GROUP_BLOCKS:
            for key in response.get(group, {}).get(block, {}):
                name = f"{block}.{key}"
                assert name in declared, (
                    f"{profile}/{group}: '{name}' is in the shipped YAML but "
                    "mujoco_simulator_node never declares it — the value is "
                    "silently ignored and the compiled default is used instead"
                )
                checked += 1
    if profile == "ur5e_p1b":
        # This profile is the reason the test exists; if its contact_wrench
        # block ever empties out, the coverage went with it.
        assert checked, f"{profile}: no group block keys checked — test is vacuous"


@pytest.mark.parametrize("profile", PROFILES)
def test_toplevel_block_keys_are_declared(profile: str, declared: set[str]) -> None:
    params = _params(profile)
    for block in TOPLEVEL_BLOCKS:
        for key in params.get(block, {}):
            name = f"{block}.{key}"
            assert name in declared, (
                f"{profile}: '{name}' is in the shipped YAML but "
                "mujoco_simulator_node never declares it — the value is "
                "silently ignored and the compiled default is used instead"
            )


def test_p1b_fingertip_wrench_frame_is_the_bracket_frame() -> None:
    """p1b must publish in the site frame, not the body frame.

    Pinned separately from the declare check because the two fail differently:
    a missing declare makes the key inert, while a value flipped back to
    ``"body"`` is honoured and simply wrong. Both put the same 90-degree
    rotation into every fingertip force, and neither shows up as an error.
    """
    p1b = _params("ur5e_p1b")["robot_response"]["p1b"]
    block = p1b.get("contact_wrench", {})
    assert block.get("enabled") is True, "p1b contact_wrench is disabled"
    assert block.get("reference_frame") == "site", (
        "p1b fingertip wrenches must be published in the ft-site frame: the "
        "URDF fingertip frames consumers apply R_link(q) for are the "
        "l_*_tip_bracket links, which are rotated relative to the "
        "l_*_tip_link bodies the sites hang under"
    )
    # The suffix pair is what resolves sensor -> site; the defaults do not
    # match this hand's MJCF, so a dropped override silently unresolves every
    # sensor.
    assert block.get("sensor_name_suffixes") == ["_contact"]
    assert block.get("reference_site_suffixes") == ["_bracket"]
