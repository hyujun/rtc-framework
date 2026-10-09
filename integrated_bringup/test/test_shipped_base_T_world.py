"""Shipped ``catching.io.base_T_world`` — the arm base where the shipped scene puts it.

**Two files in two nodes state one fact.** The simulator's ``model_path`` scene
places the arm base in the world; the catching controller's
``catching.io.base_T_world`` tells the planner where that is
(``p_base = Rz(yaw) p_world + t``, applied to every vision prediction). Nothing
at runtime compares them: a scene that moves the base and a transform that does
not follow it give predictions that are entirely plausible and uniformly
displaced — the planner refuses every throw as outside its workspace, or worse,
plans against a ball that is somewhere else. The offline tools
(``rtc_tools`` ``catching_trials``) read the same key to carry the hand into
the sim world, so the success verdict moves with it.

**Structural lane (always runs).** The translation is three floats — a ROS
parameter array takes its type from its entries.

**Scene lane (needs the ``mujoco`` module).** The scene is compiled and the
world position of the body named ``catching.io.arm_base_frame`` must be the
point the transform maps to the base origin, ``-Rz(yaw)^T t``. Positions only:
an MJCF body and the URDF frame of the same name share an origin but not
necessarily an orientation convention (``ur5e_p1b``'s ``base`` is one such
pair), so the yaw is not judged here.

Like the keyframe lane of ``test_shipped_initial_qpos``, the scene lane is
manual: a local ``colcon test`` runs under ``/usr/bin/python3``, which has no
``mujoco``. Run it after touching either file::

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

from rtc_tools.analysis.catching_trials import load_profile

# The profiles that ship a catching controller with a sim scene.
PROFILES = ["ur5e_p1b", "iiwa7_leap"]
CONTROLLER = "demo_catching_controller"


def _config_dir(profile: str) -> Path:
    return Path(get_package_share_directory("integrated_bringup")) / "config" / profile


def _resolve_package_uri(uri: str) -> str | None:
    """``package://<pkg>/<rel>`` -> filesystem path, or None when unresolvable."""
    if not uri.startswith("package://"):
        return uri if os.path.isfile(uri) else None
    pkg, _, rel = uri[len("package://") :].partition("/")
    try:
        share = get_package_share_directory(pkg)
    except Exception:
        # hand_description is a separate source tree that need not be present
        # in every checkout — a missing package is a skip, not a failure.
        return None
    path = os.path.join(share, rel)
    return path if os.path.isfile(path) else None


@pytest.mark.parametrize("profile", PROFILES)
def test_base_T_world_translation_is_three_floats(profile: str) -> None:
    path = _config_dir(profile) / "controllers" / f"{CONTROLLER}.yaml"
    with open(path) as handle:
        io = yaml.safe_load(handle)[CONTROLLER]["catching"]["io"]
    translation = io["base_T_world"]["translation"]
    assert len(translation) == 3
    for i, value in enumerate(translation):
        assert not isinstance(value, bool) and isinstance(value, float), (
            f"{profile}: base_T_world.translation[{i}] is {value!r} "
            f"({type(value).__name__}) — write 0.0, not 0"
        )
    assert isinstance(io["base_T_world"]["yaw_deg"], float)


@pytest.mark.parametrize("profile", PROFILES)
def test_base_T_world_puts_the_arm_base_where_the_shipped_scene_does(profile: str) -> None:
    mujoco = pytest.importorskip(
        "mujoco",
        reason=(
            "mujoco absent — this lane is manual-only (colcon runs pytest under "
            "/usr/bin/python3). Run: .venv/bin/python -m pytest <this file>"
        ),
    )
    config_dir = _config_dir(profile)
    with open(config_dir / "mujoco_simulator.yaml") as handle:
        model_path = yaml.safe_load(handle)["mujoco_simulator"]["ros__parameters"]["model_path"]
    scene = _resolve_package_uri(model_path)
    if scene is None:
        pytest.skip(f"{profile}: model_path {model_path!r} is not resolvable here")

    catching = load_profile(config_dir)
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, catching.arm_base_frame)
    assert body >= 0, f"{profile}: no body {catching.arm_base_frame!r} in {scene}"

    # p_base = R p_world + t, so the base origin (p_base = 0) is at -R^T t.
    rotation = catching.base_t_world[:3, :3]
    translation = catching.base_t_world[:3, 3]
    expected = -(rotation.T @ translation)
    actual = data.xpos[body]
    gap = math.dist(actual, expected)
    assert gap < 1e-6, (
        f"{profile}: the shipped scene {os.path.basename(scene)} has the arm base "
        f"{catching.arm_base_frame!r} at world {actual.round(4).tolist()}, but "
        f"catching.io.base_T_world places it at {expected.round(4).tolist()} "
        f"({gap:.3f} m apart). Every vision prediction would be read that far off — "
        "move the transform with the scene."
    )
