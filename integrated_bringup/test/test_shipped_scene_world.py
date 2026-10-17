"""Shipped ``urdf.extra_frames.scene_world`` — where each profile defines the scene's world.

A task goal's ``header.frame_id`` names its frame: the arm's ``root_link`` or
``scene_world`` (#779). ``scene_world`` is a model frame the CM adds from
``urdf.extra_frames``, and where it is defined decides which bring-ups take
world goals at all:

- ``sim.yaml`` — the shipped scene's world, so sim takes world goals.
- ``_base.yaml`` / ``robot.yaml`` — never. A real arm has no measured world
  until someone calibrates one; a sim value there would be read on hardware
  without any error, displaced by the pedestal.
- every sim overlay — its own. ROS params overlay key by key, so an overlay
  that swaps the scene and says nothing about the world keeps ``sim.yaml``'s,
  which belongs to a different scene.
- ``g1_p1b`` — none (its dual-arm targets name model frames, not this one).

The other frame a goal may name is the arm's ``root_link``. The producers that
send task goals (``ur5e_bt_coordinator``, ``shape_estimation``) stamp their
``base_frame`` parameter, so its shipped default has to be the root_link of the
profiles they run with (read from the source tree: neither is a dependency).
The controllers publish their poses under that root_link too, and the digital
twin hangs its TCP marker off the frame they publish under unless
``tcp_viz.frame_id`` names one — which then has to be that root_link (``base``
is not a frame of the arm on every URDF).

Structural only. The values themselves were checked against each scene when
they were written: MuJoCo body positions against Pinocchio frames mapped
through the placement, over random arm poses (#779).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

PROFILES = ["ur5e_p1a", "ur5e_p1b", "iiwa7_leap"]
NO_WORLD = ["g1_p1b"]
RT_NODE = "integrated_rt_controller"


def _config_dir(profile: str) -> Path:
    return Path(get_package_share_directory("integrated_bringup")) / "config" / profile


def _rt_params(path: Path) -> dict:
    """The RT node's parameters in a params file (``/**`` or its own name)."""
    with open(path) as handle:
        doc = yaml.safe_load(handle) or {}
    merged: dict = {}
    for key in ("/**", RT_NODE):
        merged.update((doc.get(key) or {}).get("ros__parameters") or {})
    return merged


def _scene_world(params: dict) -> dict | None:
    return ((params.get("urdf") or {}).get("extra_frames") or {}).get("scene_world")


def _arm_root(profile: str) -> str:
    """The arm's root_link: the first sub-model (controllers take it as the arm)."""
    config_dir = _config_dir(profile)
    for name in ("_base.yaml", "sim.yaml"):
        path = config_dir / name
        if path.exists():
            sub_models = (_rt_params(path).get("urdf") or {}).get("sub_models")
            if sub_models:
                return next(iter(sub_models.values()))["root_link"]
    raise AssertionError(f"{profile}: no urdf.sub_models")


def _assert_well_formed(frame: dict, where: str, root: str) -> None:
    assert frame.get("parent") == root, (
        f"{where}: scene_world.parent is {frame.get('parent')!r}, not the arm root {root!r} — "
        "the controllers convert a world goal into their root_link through this frame"
    )
    assert frame.get("provisional") is False, (
        f"{where}: scene_world.provisional is {frame.get('provisional')!r} — an unset flag parses "
        "as provisional, and the controllers do not use a provisional world"
    )
    for key in ("xyz", "rpy"):
        values = frame.get(key)
        assert isinstance(values, list) and len(values) == 3, f"{where}: scene_world.{key}"
        for i, value in enumerate(values):
            assert isinstance(value, float) and not isinstance(value, bool), (
                f"{where}: scene_world.{key}[{i}] is {value!r} — write 0.0, not 0 "
                "(a ROS parameter array takes its type from its entries)"
            )


@pytest.mark.parametrize("profile", PROFILES)
def test_sim_defines_the_scene_world_on_the_arm_root(profile: str) -> None:
    frame = _scene_world(_rt_params(_config_dir(profile) / "sim.yaml"))
    assert frame is not None, f"{profile}: sim.yaml defines no urdf.extra_frames.scene_world"
    _assert_well_formed(frame, f"{profile}/sim.yaml", _arm_root(profile))


@pytest.mark.parametrize("profile", PROFILES)
@pytest.mark.parametrize("mode_file", ["_base.yaml", "robot.yaml"])
def test_no_world_outside_the_sim(profile: str, mode_file: str) -> None:
    path = _config_dir(profile) / mode_file
    if not path.exists():
        pytest.skip(f"{profile} ships no {mode_file}")
    assert _scene_world(_rt_params(path)) is None, (
        f"{profile}/{mode_file} defines scene_world — a real arm would take world goals "
        "against the sim scene's world"
    )


@pytest.mark.parametrize("profile", PROFILES)
def test_every_overlay_states_its_own_world(profile: str) -> None:
    overlays = sorted((_config_dir(profile) / "sim_overlays").glob("*.yaml"))
    if not overlays:
        pytest.skip(f"{profile} ships no sim overlay")
    root = _arm_root(profile)
    for overlay in overlays:
        frame = _scene_world(_rt_params(overlay))
        assert frame is not None, (
            f"{profile}/sim_overlays/{overlay.name} defines no scene_world for the "
            f"{RT_NODE} node — it would keep sim.yaml's, which is another scene's"
        )
        _assert_well_formed(frame, f"{profile}/sim_overlays/{overlay.name}", root)


@pytest.mark.parametrize("profile", NO_WORLD)
def test_profiles_without_a_world_define_none(profile: str) -> None:
    for path in sorted(_config_dir(profile).glob("*.yaml")):
        assert _scene_world(_rt_params(path)) is None, f"{profile}/{path.name}"


# The producers run with the UR5e profiles; their configs live in sibling
# packages of the source tree.
PRODUCERS = {
    "ur5e_bt_coordinator": ("config/bt_coordinator.yaml", "bt_coordinator"),
    "shape_estimation": ("config/shape_estimation_node.yaml", "shape_estimation_node"),
}
PRODUCER_PROFILES = ["ur5e_p1a", "ur5e_p1b"]


@pytest.mark.parametrize("package", sorted(PRODUCERS))
def test_producers_stamp_the_arm_root(package: str) -> None:
    rel, node = PRODUCERS[package]
    path = Path(__file__).resolve().parents[2] / package / rel
    if not path.exists():
        pytest.skip(f"{path} not in this checkout")
    with open(path) as handle:
        params = (yaml.safe_load(handle).get(node) or {}).get("ros__parameters") or {}
    assert "base_frame" in params, f"{package}: {rel} ships no base_frame"
    for profile in PRODUCER_PROFILES:
        assert params["base_frame"] == _arm_root(profile), (
            f"{package}: base_frame {params['base_frame']!r} is not {profile}'s root_link "
            f"{_arm_root(profile)!r} — every task goal it sends would be refused"
        )


# The twin node's name in the launch (rtc_digital_twin digital_twin.launch.py).
TWIN_NODE = "digital_twin_node"
# The twin's own defaults (a sibling package of the source tree).
TWIN_DEFAULTS = (
    Path(__file__).resolve().parents[2] / "rtc_digital_twin" / "config" / "digital_twin.yaml"
)


def _twin_params(path: Path) -> dict:
    """The twin's parameters in a params file (``/**`` or its own name)."""
    with open(path) as handle:
        doc = yaml.safe_load(handle) or {}
    merged: dict = {}
    for key in ("/**", TWIN_NODE):
        merged.update((doc.get(key) or {}).get("ros__parameters") or {})
    return merged


def test_the_twin_takes_the_tcp_parent_from_the_controller_by_default() -> None:
    if not TWIN_DEFAULTS.exists():
        pytest.skip(f"{TWIN_DEFAULTS} not in this checkout")
    assert _twin_params(TWIN_DEFAULTS)["tcp_viz.frame_id"] == "", (
        "rtc_digital_twin's tcp_viz.frame_id default names a frame — no frame name is the arm "
        "root on every URDF; empty takes the frame the controller publishes the pose under"
    )


@pytest.mark.parametrize("profile", [*PROFILES, *NO_WORLD])
def test_a_twin_tcp_parent_a_profile_names_is_the_arm_root(profile: str) -> None:
    path = _config_dir(profile) / "digital_twin.yaml"
    if not path.exists():
        pytest.skip(f"{profile} ships no digital_twin.yaml")
    frame = _twin_params(path).get("tcp_viz.frame_id")
    if not frame:
        return  # the frame the controllers publish under: their root_link
    assert frame == _arm_root(profile), (
        f"{profile}/digital_twin.yaml: tcp_viz.frame_id is {frame!r}, not the arm root "
        f"{_arm_root(profile)!r} — the marker and the virtual_tcp TF would hang off another "
        "frame than the one the controllers publish the pose under"
    )
