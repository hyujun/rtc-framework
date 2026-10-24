"""Shipped ``scene_world`` vs each shipped scene — the arm root where the scene puts it.

**Two files in two nodes state one fact.** The simulator's ``model_path`` scene
places the arm in the world; the RT node's ``urdf.extra_frames.scene_world``
(#779, ``p_root = R(rpy) p_world + xyz``) tells every consumer where that is:
the catching controller reads it as the vision world (applied to every
prediction), the inference controller as the object lane's source frame, and
the task-goal controllers convert ``scene_world`` goals through it. Nothing at
runtime compares the two: a scene that moves the base and a frame that does not
follow it give poses that are entirely plausible and uniformly displaced. The
offline tools (``rtc_tools`` ``catching_trials``) read the same frame, through
the same ``_base.yaml`` + ``sim.yaml`` (+ overlay) merge the launch does — this
test reads it through them too, so a tool that stopped merging fails here.

**Structural lane (always runs).** No shipped catching config carries the
``catching.io`` pair the frame replaced (``arm_base_frame`` / ``base_T_world``);
the controller refuses it. The frame's own shape (parent, float types,
``provisional``) is ``test_shipped_scene_world``'s.

**Scene lane (needs the ``mujoco`` module).** For ``sim.yaml`` and every sim
overlay of every profile that defines a world, the scene that bring-up loads is
compiled and the world position of the body named ``scene_world.parent`` must
be the point the frame maps to the root origin, ``-R^T xyz``. Positions only:
an MJCF body and the URDF frame of the same name share an origin but not
necessarily an orientation convention (``ur5e_p1b``'s ``base`` is one such
pair), so the rotation is not judged here — the unit tests pin it
(``test_catching_planner_lane`` / ``test_catching_trials``, an asymmetric
frame on both sides).

Only scenes in ``robot_descriptions`` are compiled. A bring-up whose scene is in
another package (``ur5e_p1b``'s are in ``hand_description``, a separate project
this workspace does not depend on) is not a case of this lane at all: a test
here never resolves a package outside the repo (#952).

Like the keyframe lane of ``test_shipped_initial_qpos``, the scene lane is
manual: a local ``colcon test`` runs under ``/usr/bin/python3``, which has no
``mujoco``. Run it after touching a scene, ``sim.yaml`` or an overlay::

    .venv/bin/python -m pytest \
        src/rtc-framework/integrated_bringup/test/test_shipped_base_T_world.py
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from rtc_tools.analysis.catching_trials import (
    RETIRED_IO_FRAME_KEYS,
    robot_config_params,
    vision_frame_from_params,
)

# The profiles that define the scene's world (test_shipped_scene_world.PROFILES).
PROFILES = ["ur5e_p1a", "ur5e_p1b", "iiwa7_leap"]
# The profiles that ship a catching controller.
CATCHING_PROFILES = ["ur5e_p1b", "iiwa7_leap"]
CONTROLLER = "demo_catching_controller"
SIMULATOR = "mujoco_simulator"


def _config_dir(profile: str) -> Path:
    return Path(get_package_share_directory("integrated_bringup")) / "config" / profile


# The one package whose scenes the scene lane compiles.
SCENE_PACKAGE = "robot_descriptions"


def _bring_ups() -> list[tuple[str, str | None]]:
    """``(profile, overlay file name or None for sim.yaml alone)`` of every shipped sim bring-up."""
    cases: list[tuple[str, str | None]] = []
    for profile in PROFILES:
        cases.append((profile, None))
        overlays = _config_dir(profile) / "sim_overlays"
        cases.extend((profile, p.name) for p in sorted(overlays.glob("*.yaml")))
    return cases


def _layered(profile: str, overlay: str | None) -> list[Path]:
    config_dir = _config_dir(profile)
    overlays = [config_dir / "sim_overlays" / overlay] if overlay else []
    return [config_dir / f"{SIMULATOR}.yaml", *overlays]


def _model_path(files: list[Path]) -> str:
    """The simulator's ``model_path`` after the files are layered (later wins)."""
    model_path = None
    for path in files:
        with open(path) as handle:
            doc = yaml.safe_load(handle) or {}
        value = ((doc.get(SIMULATOR) or {}).get("ros__parameters") or {}).get("model_path")
        if value is not None:
            model_path = value
    assert model_path is not None, f"no {SIMULATOR} model_path in {[p.name for p in files]}"
    return str(model_path)


def _scene_bring_ups() -> list[tuple[str, str | None]]:
    """The bring-ups whose scene is in :data:`SCENE_PACKAGE`."""
    prefix = f"package://{SCENE_PACKAGE}/"
    return [c for c in _bring_ups() if _model_path(_layered(*c)).startswith(prefix)]


def _scene_file(model_path: str) -> str:
    """A :data:`SCENE_PACKAGE` ``package://`` URI -> the installed file (asserted to exist)."""
    rel = model_path[len(f"package://{SCENE_PACKAGE}/") :]
    path = os.path.join(get_package_share_directory(SCENE_PACKAGE), rel)
    assert os.path.isfile(path), f"{model_path!r} is not installed at {path}"
    return path


def test_the_scene_lane_has_cases() -> None:
    # Without this a filter that matched nothing would leave the lane vacuous.
    assert {profile for profile, _ in _scene_bring_ups()} >= {"ur5e_p1a", "iiwa7_leap"}


@pytest.mark.parametrize("profile", CATCHING_PROFILES)
def test_no_catching_config_carries_the_retired_frame_keys(profile: str) -> None:
    path = _config_dir(profile) / "controllers" / f"{CONTROLLER}.yaml"
    with open(path) as handle:
        io = yaml.safe_load(handle)[CONTROLLER]["catching"].get("io") or {}
    for key in RETIRED_IO_FRAME_KEYS:
        assert key not in io, (
            f"{profile}: catching.io.{key} is gone (#779) — the controller refuses it; the "
            "vision world is urdf.extra_frames.scene_world"
        )


@pytest.mark.parametrize(("profile", "overlay"), _scene_bring_ups())
def test_scene_world_puts_the_arm_root_where_the_scene_does(
    profile: str, overlay: str | None
) -> None:
    mujoco = pytest.importorskip(
        "mujoco",
        reason=(
            "mujoco absent — this lane is manual-only (colcon runs pytest under "
            "/usr/bin/python3). Run: .venv/bin/python -m pytest <this file>"
        ),
    )
    config_dir = _config_dir(profile)
    overlays = [config_dir / "sim_overlays" / overlay] if overlay else []
    where = f"{profile}/{overlay or 'sim.yaml'}"
    scene = _scene_file(_model_path(_layered(profile, overlay)))

    params = robot_config_params(config_dir, overlays)
    parent, parent_t_world = vision_frame_from_params(params, where)
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, parent)
    assert body >= 0, f"{where}: no body {parent!r} in {scene}"

    # p_root = R p_world + t, so the root origin (p_root = 0) is at -R^T t.
    rotation = parent_t_world[:3, :3]
    translation = parent_t_world[:3, 3]
    expected = -(rotation.T @ translation)
    actual = data.xpos[body]
    gap = math.dist(actual, expected)
    assert gap < 1e-6, (
        f"{where}: the scene {os.path.basename(scene)} has the arm root {parent!r} at world "
        f"{actual.round(4).tolist()}, but urdf.extra_frames.scene_world places it at "
        f"{expected.round(4).tolist()} ({gap:.3f} m apart). Every world pose would be read "
        "that far off — move the frame with the scene."
    )
