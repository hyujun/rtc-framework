"""rtc_tools.validation.model_match: a matching pair, a missing body, the real G1 pair.

Skipped where mujoco is not importable (colcon runs pytest on the system
python; mujoco lives in the workspace's .venv) — run it there:

    PYTHONPATH=rtc_tools:$PYTHONPATH <rtc_ws>/.venv/bin/python -m pytest rtc_tools/test/test_model_match.py
"""

from __future__ import annotations

import pytest
from conftest import robots_dir

pytest.importorskip("mujoco")
pytest.importorskip("pinocchio")

from rtc_tools.validation.model_match import model_match  # noqa: E402

URDF = """<robot name="r">
  <link name="base"><inertial><origin xyz="0 0 0.1"/><mass value="2"/>
    <inertia ixx="0.02" iyy="0.03" izz="0.04" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="arm"><inertial><origin xyz="0.1 0 0" rpy="0 0 0.5"/><mass value="1"/>
    <inertia ixx="0.001" iyy="0.004" izz="0.004" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="j" type="revolute"><parent link="base"/><child link="arm"/>
    <origin xyz="0 0.2 0.3" rpy="0.3 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  {extra}
</robot>"""
HAND = """<link name="hand"><inertial><origin xyz="0 0 0.05"/><mass value="0.5"/>
    <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="h" type="fixed"><parent link="arm"/><child link="hand"/><origin xyz="0.2 0 0"/></joint>"""
# The same two bodies; the free joint starts 1 m up and yawed, which must not matter.
MJCF = """<mujoco><compiler angle="radian"/><worldbody>
  <body name="base" pos="0 0 1" quat="0.9238795 0 0 0.3826834"><freejoint/>
    <inertial pos="0 0 0.1" mass="2" diaginertia="0.02 0.03 0.04"/>
    <body name="arm" pos="0 0.2 0.3" euler="0.3 0 0"><joint name="j" axis="0 0 1"/>
      <inertial pos="0.1 0 0" euler="0 0 0.5" mass="1" diaginertia="0.001 0.004 0.004"/>
    </body></body></worldbody></mujoco>"""


def pair(tmp_path, extra=""):
    urdf, mjcf = tmp_path / "r.urdf", tmp_path / "r.xml"
    urdf.write_text(URDF.format(extra=extra))
    mjcf.write_text(MJCF)
    return urdf, mjcf


def test_the_same_bodies_match_wherever_the_plant_starts(tmp_path):
    m = model_match(*pair(tmp_path))
    assert m["control_mass_kg"] == pytest.approx(3.0)
    assert abs(m["mass_kg"]) < 1e-12 and m["com_m"] < 1e-9 and m["inertia_kgm2"] < 1e-9


def test_a_body_only_the_controller_has_shows_in_all_three(tmp_path):
    m = model_match(*pair(tmp_path, HAND))
    assert m["mass_kg"] == pytest.approx(-0.5)
    assert m["com_m"] > 1e-2 and m["inertia_kgm2"] > 1e-3


def test_the_shipped_g1_scene_matches_the_body_urdf_and_not_the_one_with_a_hand():
    robots = robots_dir()
    if robots is None:
        pytest.skip("robot_descriptions is not next to this package")
    scene = robots / "g1/mjcf/g1_29dof_floating.xml"
    body = model_match(robots / "g1/urdf/g1_29dof_floating.urdf", scene)
    assert abs(body["mass_kg"]) < 1e-5 and body["com_m"] < 1e-6 and body["inertia_kgm2"] < 1e-5
    hand = model_match(robots / "g1/urdf/g1_29dof_floating_with_p1b_surrogate.urdf.xacro", scene)
    assert hand["mass_kg"] < -0.1  # the surrogate hand is missing from the scene
    assert hand["com_m"] > 5e-3
