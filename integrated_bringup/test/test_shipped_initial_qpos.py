"""Shipped ``robot_response.<group>.initial_qpos`` — size, type, and keyframe agreement.

Two independent lanes, deliberately not merged:

**Structural (always runs).** Every ``initial_qpos`` must have exactly as many
entries as its group's ``command_joint_names``, and every entry must be a float.
Both failures abort the simulator *while the node is being constructed* — a wrong
length fails ``Initialize``, and a sequence written with integer literals
(``[0, -1, 1]``) parses as an integer array that never matches the declared
double array — so neither reaches any runtime check, and the YAML parse gate in
``verify-changes.sh`` does not look at types or lengths. This lane needs nothing
but PyYAML, which is why it is the one that actually gates a local commit.

**Keyframe agreement (needs the ``mujoco`` module).** ``ur5e_p1a`` and
``iiwa7_leap`` ship an ``initial_qpos`` that duplicates a pose their MJCF already
carries as its first keyframe. That duplication is the price of making the key
explicit for all three profiles, and this lane is what stops the two copies from
drifting: the YAML now *wins*, so a keyframe edit that nobody mirrors here would
silently have no effect on the sim. ``ur5e_p1b`` has no keyframe at all — that is
the whole reason the key exists — so it is skipped by the same rule that checks
the others, not by an exception listing its name.

Values are compared **by joint name**, never as a qpos slice. The LEAP hand is
the reason: its joints occupy qpos addresses 8, 7, 9, 10, 12, 11, ... so
``command_joint_names`` order and qpos order genuinely differ, and a slice-based
comparison would be reading a different joint's value while passing.

**Where the keyframe lane actually runs: nowhere automatic — only by hand.**
Two independent gaps stack up, and it is worth spelling both out because each
one alone looks survivable:

1. a local ``colcon test`` runs pytest under ``/usr/bin/python3``, which has no
   ``mujoco`` (it is a pip package in the workspace venv, not a ROS dependency),
   so the lane skips;
2. CI's ``python-test`` job does ``pip install ... mujoco``, but it builds and
   tests only ``.github/ci-packages.yml``'s ``test_python`` list —
   ``rtc_tools``, ``rtc_digital_twin``, ``robot_descriptions``. This package is
   not in it, so the file is never collected there at all.

So this lane is a manual check, not a gate. Run it deliberately after touching
either copy of a pose::

    .venv/bin/python -m pytest \
        src/rtc-framework/integrated_bringup/test/test_shipped_initial_qpos.py

The structural lane above has neither problem — it needs only PyYAML and runs
under a plain ``colcon test`` — which is why the failure that actually aborts a
node lives there and not here.
"""

from __future__ import annotations

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

# Profile -> the MJCF that profile's mujoco_simulator.yaml loads. The path is
# read out of the config itself rather than listed here, so this map only needs
# to name the profiles.
PROFILES = ["ur5e_p1a", "ur5e_p1b", "iiwa7_leap"]


def _config_path(profile: str) -> str:
    share = get_package_share_directory("integrated_bringup")
    return os.path.join(share, "config", profile, "mujoco_simulator.yaml")


def _robot_response(profile: str) -> dict:
    with open(_config_path(profile)) as handle:
        params = yaml.safe_load(handle)
    return params["mujoco_simulator"]["ros__parameters"]["robot_response"]


def _resolve_package_uri(uri: str) -> str | None:
    """``package://<pkg>/<rel>`` -> filesystem path, or None when unresolvable."""
    if not uri.startswith("package://"):
        return uri if os.path.isfile(uri) else None
    pkg, _, rel = uri[len("package://") :].partition("/")
    try:
        share = get_package_share_directory(pkg)
    except Exception:
        # hand_description is a separate source tree that need not be present
        # in every checkout (CI included) — a missing package is a skip, not a
        # failure of this profile's YAML.
        return None
    path = os.path.join(share, rel)
    return path if os.path.isfile(path) else None


# ── Structural lane (PyYAML only) ────────────────────────────────────────────


@pytest.mark.parametrize("profile", PROFILES)
def test_initial_qpos_length_matches_command_joints(profile: str) -> None:
    response = _robot_response(profile)
    checked = 0
    for group in response["groups"]:
        block = response[group]
        if "initial_qpos" not in block:
            continue
        names = block.get("command_joint_names") or block.get("joint_names")
        assert names, f"{profile}/{group}: no command_joint_names to size against"
        assert len(block["initial_qpos"]) == len(names), (
            f"{profile}/{group}: initial_qpos has {len(block['initial_qpos'])} entries "
            f"but the group has {len(names)} command joints — Initialize would fail"
        )
        checked += 1
    assert checked, f"{profile}: no initial_qpos found — this test would be vacuous"


@pytest.mark.parametrize("profile", PROFILES)
def test_initial_qpos_entries_are_floats(profile: str) -> None:
    response = _robot_response(profile)
    for group in response["groups"]:
        block = response[group]
        for i, value in enumerate(block.get("initial_qpos", [])):
            # bool first: it is a subclass of int and would otherwise read as
            # "not a float" with a confusing message.
            assert not isinstance(value, bool) and isinstance(value, float), (
                f"{profile}/{group}: initial_qpos[{i}] is {value!r} ({type(value).__name__}). "
                "A ROS 2 params sequence takes its type from its entries, so one integer "
                "literal makes the whole array an integer array and the node throws at "
                "construction. Write 0.0, not 0."
            )


# ── Keyframe agreement lane (needs mujoco) ───────────────────────────────────


@pytest.mark.parametrize("profile", PROFILES)
def test_initial_qpos_matches_mjcf_keyframe(profile: str) -> None:
    mujoco = pytest.importorskip(
        "mujoco",
        reason=(
            "mujoco absent — this lane is manual-only: local colcon runs pytest under "
            "/usr/bin/python3, and CI's python-test job does not build this package. "
            "Run: .venv/bin/python -m pytest <this file>"
        ),
    )

    response = _robot_response(profile)
    with open(_config_path(profile)) as handle:
        params = yaml.safe_load(handle)["mujoco_simulator"]["ros__parameters"]
    path = _resolve_package_uri(params["model_path"])
    if path is None:
        pytest.skip(f"{profile}: model_path {params['model_path']!r} is not resolvable here")

    model = mujoco.MjModel.from_xml_path(path)
    if model.nkey == 0:
        # ur5e_p1b lands here: nothing to agree with, which is exactly why its
        # initial_qpos is the only source of a home pose.
        pytest.skip(f"{profile}: scene has no keyframe")

    for group in response["groups"]:
        block = response[group]
        if "initial_qpos" not in block:
            continue
        names = block.get("command_joint_names") or block.get("joint_names")
        for i, joint in enumerate(names):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(joint))
            assert jid >= 0, f"{profile}/{group}: joint {joint!r} not in {path}"
            expected = float(model.key_qpos[0][model.jnt_qposadr[jid]])
            assert block["initial_qpos"][i] == pytest.approx(expected, abs=1e-4), (
                f"{profile}/{group}: initial_qpos[{i}] (joint {joint!r}) is "
                f"{block['initial_qpos'][i]} but the MJCF keyframe says {expected}. "
                "The YAML wins at runtime, so this drift would make the keyframe edit "
                "silently ineffective — mirror it here or drop the YAML key."
            )
