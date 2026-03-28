"""Tests for rtc_tools.conversion.urdf_to_mjcf module."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from rtc_tools.conversion.urdf_to_mjcf import (
    JointClassification,
    JointInfo,
    MimicParams,
    classify_joints,
    remove_closed_chain_joints,
    resolve_robot_dir_paths,
    postprocess_mjcf,
    generate_scene,
    _add_equality_constraints,
    _add_actuators,
    _clean_mesh_paths,
    _fix_compiler,
)


# ════════════════════════════════════════════════════════════════════════════
# Test URDF Fixtures
# ════════════════════════════════════════════════════════════════════════════

URDF_BASIC = """\
<robot name="test">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="150" lower="-3.14" upper="3.14" velocity="3.14"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="28" lower="-6.28" upper="6.28" velocity="3.14"/>
  </joint>
</robot>
"""

URDF_WITH_FIXED = """\
<robot name="test">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="tool"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
  <joint name="tool_fixed" type="fixed">
    <parent link="link1"/>
    <child link="tool"/>
  </joint>
</robot>
"""

URDF_WITH_MIMIC = """\
<robot name="test">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="link3"/>
  <joint name="master_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
  <joint name="mimic_joint" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 0 1"/>
    <mimic joint="master_joint" multiplier="2.0" offset="0.5"/>
    <limit effort="50" lower="-6.28" upper="6.28" velocity="1.0"/>
  </joint>
  <joint name="mimic_default" type="revolute">
    <parent link="link1"/>
    <child link="link3"/>
    <axis xyz="0 0 1"/>
    <mimic joint="master_joint"/>
    <limit effort="50" lower="-6.28" upper="6.28" velocity="1.0"/>
  </joint>
</robot>
"""

URDF_CLOSED_CHAIN = """\
<robot name="test">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="shared_link"/>
  <joint name="normal_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
  <joint name="tree_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shared_link"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
  <joint name="loop_joint" type="revolute">
    <parent link="link1"/>
    <child link="shared_link"/>
    <origin xyz="0.2 0.1 0.3"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
</robot>
"""

URDF_MIXED = """\
<robot name="test">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="link3"/>
  <link name="shared_link"/>
  <joint name="active1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="150" lower="-6.28" upper="6.28" velocity="3.14"/>
  </joint>
  <joint name="active2" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit effort="28" lower="0" upper="0" velocity="3.14"/>
  </joint>
  <joint name="mimic1" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <axis xyz="0 0 1"/>
    <mimic joint="active1" multiplier="-1.0" offset="0.0"/>
    <limit effort="50" lower="-6.28" upper="6.28" velocity="1.0"/>
  </joint>
  <joint name="tree_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shared_link"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
  <joint name="loop_joint" type="revolute">
    <parent link="link1"/>
    <child link="shared_link"/>
    <origin xyz="0.1 0.2 0.3"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
  <joint name="fixed1" type="fixed">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
</robot>
"""


# ════════════════════════════════════════════════════════════════════════════
# Joint Classification Tests
# ════════════════════════════════════════════════════════════════════════════

class TestClassifyJoints:
    def test_basic_active_joints(self):
        c = classify_joints(URDF_BASIC)
        assert set(c.active.keys()) == {"joint1", "joint2"}
        assert len(c.passive_mimic) == 0
        assert len(c.passive_closed_chain) == 0
        assert len(c.fixed) == 0

    def test_joint_info_parsing(self):
        c = classify_joints(URDF_BASIC)
        j1 = c.active["joint1"]
        assert j1.parent_link == "base_link"
        assert j1.child_link == "link1"
        assert j1.axis == (0.0, 0.0, 1.0)
        assert j1.origin_xyz == (0.0, 0.0, 0.5)
        assert j1.effort_limit == 150.0
        assert j1.lower_limit == pytest.approx(-3.14)
        assert j1.upper_limit == pytest.approx(3.14)

    def test_fixed_joints_excluded(self):
        c = classify_joints(URDF_WITH_FIXED)
        assert "joint1" in c.active
        assert "tool_fixed" not in c.active
        assert "tool_fixed" in c.fixed

    def test_mimic_detection(self):
        c = classify_joints(URDF_WITH_MIMIC)
        assert "master_joint" in c.active
        assert set(c.passive_mimic.keys()) == {"mimic_joint", "mimic_default"}

        mimic_info = c.passive_mimic["mimic_joint"]
        assert mimic_info.mimic is not None
        assert mimic_info.mimic.master_joint == "master_joint"
        assert mimic_info.mimic.multiplier == 2.0
        assert mimic_info.mimic.offset == 0.5

    def test_mimic_default_params(self):
        c = classify_joints(URDF_WITH_MIMIC)
        default_info = c.passive_mimic["mimic_default"]
        assert default_info.mimic is not None
        assert default_info.mimic.multiplier == 1.0
        assert default_info.mimic.offset == 0.0

    def test_closed_chain_detection(self):
        c = classify_joints(URDF_CLOSED_CHAIN)
        assert "tree_joint" in c.active
        assert "normal_joint" in c.active
        assert "loop_joint" in c.passive_closed_chain

        info = c.passive_closed_chain["loop_joint"]
        assert info.is_closed_chain is True
        assert info.parent_link == "link1"
        assert info.child_link == "shared_link"
        assert info.origin_xyz == (0.2, 0.1, 0.3)

    def test_mixed_classification(self):
        c = classify_joints(URDF_MIXED)
        assert set(c.active.keys()) == {"active1", "active2", "tree_joint"}
        assert set(c.passive_mimic.keys()) == {"mimic1"}
        assert set(c.passive_closed_chain.keys()) == {"loop_joint"}
        assert "fixed1" in c.fixed

    def test_passive_names_property(self):
        c = classify_joints(URDF_MIXED)
        assert c.passive_names == {"mimic1", "loop_joint"}

    def test_no_overlap(self):
        """Active, mimic, closed-chain, and fixed sets must not overlap."""
        for urdf in [URDF_BASIC, URDF_WITH_FIXED, URDF_WITH_MIMIC,
                     URDF_CLOSED_CHAIN, URDF_MIXED]:
            c = classify_joints(urdf)
            all_names = (
                set(c.active.keys())
                | set(c.passive_mimic.keys())
                | set(c.passive_closed_chain.keys())
                | set(c.fixed)
            )
            total = (
                len(c.active) + len(c.passive_mimic)
                + len(c.passive_closed_chain) + len(c.fixed)
            )
            assert len(all_names) == total, "Joint sets overlap"


# ════════════════════════════════════════════════════════════════════════════
# URDF Pre-processing Tests
# ════════════════════════════════════════════════════════════════════════════

class TestRemoveClosedChainJoints:
    def test_removes_loop_joint(self):
        c = classify_joints(URDF_CLOSED_CHAIN)
        modified = remove_closed_chain_joints(URDF_CLOSED_CHAIN, c.passive_closed_chain)
        root = ET.fromstring(modified)
        joint_names = {j.get("name") for j in root.findall("joint")}
        assert "loop_joint" not in joint_names
        assert "tree_joint" in joint_names
        assert "normal_joint" in joint_names

    def test_noop_when_no_closed_chain(self):
        c = classify_joints(URDF_BASIC)
        modified = remove_closed_chain_joints(URDF_BASIC, c.passive_closed_chain)
        root = ET.fromstring(modified)
        joint_names = {j.get("name") for j in root.findall("joint")}
        assert joint_names == {"joint1", "joint2"}

    def test_mixed_removes_only_closed_chain(self):
        c = classify_joints(URDF_MIXED)
        modified = remove_closed_chain_joints(URDF_MIXED, c.passive_closed_chain)
        root = ET.fromstring(modified)
        joint_names = {j.get("name") for j in root.findall("joint")}
        assert "loop_joint" not in joint_names
        # mimic and active joints remain
        assert "mimic1" in joint_names
        assert "active1" in joint_names


# ════════════════════════════════════════════════════════════════════════════
# MJCF Post-Processing Tests
# ════════════════════════════════════════════════════════════════════════════

class TestAddEqualityConstraints:
    def _make_root(self) -> ET.Element:
        return ET.fromstring('<mujoco model="test"></mujoco>')

    def test_mimic_equality(self):
        root = self._make_root()
        c = classify_joints(URDF_WITH_MIMIC)
        n = _add_equality_constraints(root, c)

        assert n == 2
        eq = root.find("equality")
        assert eq is not None

        joints = eq.findall("joint")
        assert len(joints) == 2

        # Check mimic_joint constraint
        j_map = {j.get("joint1"): j for j in joints}
        mj = j_map["mimic_joint"]
        assert mj.get("joint2") == "master_joint"
        assert mj.get("polycoef") == "0.5 2.0 0 0 0"

        # Check mimic_default constraint (multiplier=1, offset=0)
        md = j_map["mimic_default"]
        assert md.get("joint2") == "master_joint"
        assert md.get("polycoef") == "0.0 1.0 0 0 0"

    def test_closed_chain_connect(self):
        root = self._make_root()
        c = classify_joints(URDF_CLOSED_CHAIN)
        n = _add_equality_constraints(root, c)

        assert n == 1
        eq = root.find("equality")
        connects = eq.findall("connect")
        assert len(connects) == 1

        conn = connects[0]
        assert conn.get("body1") == "link1"
        assert conn.get("body2") == "shared_link"
        assert conn.get("anchor") == "0.2 0.1 0.3"

    def test_mixed_equality(self):
        root = self._make_root()
        c = classify_joints(URDF_MIXED)
        n = _add_equality_constraints(root, c)

        assert n == 2  # 1 mimic + 1 closed-chain
        eq = root.find("equality")
        assert len(eq.findall("joint")) == 1
        assert len(eq.findall("connect")) == 1

    def test_no_constraints_for_basic(self):
        root = self._make_root()
        c = classify_joints(URDF_BASIC)
        n = _add_equality_constraints(root, c)

        assert n == 0
        assert root.find("equality") is None


class TestAddActuators:
    def _make_root(self) -> ET.Element:
        return ET.fromstring('<mujoco model="test"></mujoco>')

    def test_basic_actuators(self):
        root = self._make_root()
        c = classify_joints(URDF_BASIC)
        n = _add_actuators(root, c)

        assert n == 2
        actuator = root.find("actuator")
        generals = actuator.findall("general")
        assert len(generals) == 2

        names = {g.get("name") for g in generals}
        joints = {g.get("joint") for g in generals}
        assert names == {"joint1", "joint2"}
        assert joints == {"joint1", "joint2"}

    def test_actuator_force_range(self):
        root = self._make_root()
        c = classify_joints(URDF_BASIC)
        _add_actuators(root, c)

        actuator = root.find("actuator")
        for g in actuator.findall("general"):
            if g.get("joint") == "joint1":
                assert g.get("forcerange") == "-150 150"
            elif g.get("joint") == "joint2":
                assert g.get("forcerange") == "-28 28"

    def test_no_actuator_for_mimic(self):
        root = self._make_root()
        c = classify_joints(URDF_WITH_MIMIC)
        n = _add_actuators(root, c)

        # Only master_joint should get an actuator
        assert n == 1
        actuator = root.find("actuator")
        generals = actuator.findall("general")
        assert len(generals) == 1
        assert generals[0].get("joint") == "master_joint"

    def test_no_actuator_for_closed_chain(self):
        root = self._make_root()
        c = classify_joints(URDF_CLOSED_CHAIN)
        _add_actuators(root, c)

        actuator = root.find("actuator")
        joint_names = {g.get("joint") for g in actuator.findall("general")}
        assert "loop_joint" not in joint_names

    def test_mixed_actuators(self):
        root = self._make_root()
        c = classify_joints(URDF_MIXED)
        n = _add_actuators(root, c)

        # Only active joints: active1, active2, tree_joint
        assert n == 3
        actuator = root.find("actuator")
        joint_names = {g.get("joint") for g in actuator.findall("general")}
        assert joint_names == {"active1", "active2", "tree_joint"}
        # No passive joints
        assert "mimic1" not in joint_names
        assert "loop_joint" not in joint_names

    def test_joint_suffix_stripped(self):
        """Actuator name should strip '_joint' suffix."""
        root = self._make_root()
        c = classify_joints(URDF_WITH_MIMIC)
        _add_actuators(root, c)

        actuator = root.find("actuator")
        g = actuator.findall("general")[0]
        assert g.get("name") == "master"
        assert g.get("joint") == "master_joint"

    def test_replaces_existing_actuator(self):
        root = ET.fromstring(
            '<mujoco model="test"><actuator><general name="old" joint="old_j"/>'
            "</actuator></mujoco>"
        )
        c = classify_joints(URDF_BASIC)
        _add_actuators(root, c)

        actuators = root.findall("actuator")
        assert len(actuators) == 1
        names = {g.get("name") for g in actuators[0].findall("general")}
        assert "old" not in names


# ════════════════════════════════════════════════════════════════════════════
# Mesh / Compiler Post-Processing Tests
# ════════════════════════════════════════════════════════════════════════════

class TestCleanMeshPaths:
    def test_strips_directory(self):
        root = ET.fromstring(
            '<mujoco><asset><mesh file="/abs/path/to/base.obj"/>'
            '<mesh file="relative/shoulder.obj"/>'
            "</asset></mujoco>"
        )
        _clean_mesh_paths(root)
        files = [m.get("file") for m in root.iter("mesh")]
        assert files == ["base.obj", "shoulder.obj"]


class TestFixCompiler:
    def test_sets_attributes(self):
        root = ET.fromstring('<mujoco model="test"></mujoco>')
        _fix_compiler(root, "../meshes/assets")
        compiler = root.find("compiler")
        assert compiler is not None
        assert compiler.get("angle") == "radian"
        assert compiler.get("meshdir") == "../meshes/assets"
        assert compiler.get("autolimits") == "true"

    def test_updates_existing_compiler(self):
        root = ET.fromstring(
            '<mujoco model="test"><compiler angle="degree" meshdir="/old"/></mujoco>'
        )
        _fix_compiler(root, "../meshes")
        compiler = root.find("compiler")
        assert compiler.get("angle") == "radian"
        assert compiler.get("meshdir") == "../meshes"


# ════════════════════════════════════════════════════════════════════════════
# MJCF Full Post-Processing Test
# ════════════════════════════════════════════════════════════════════════════

class TestPostprocessMjcf:
    def test_full_postprocess(self, tmp_path):
        # Write a minimal raw MJCF
        raw_mjcf = (
            '<mujoco model="test">'
            '<compiler meshdir="/abs/meshes"/>'
            "<worldbody>"
            '<body name="base_link"><joint name="joint1"/></body>'
            "</worldbody>"
            "</mujoco>"
        )
        mjcf_file = tmp_path / "robot.xml"
        mjcf_file.write_text(raw_mjcf)

        meshdir = tmp_path / "meshes"
        meshdir.mkdir()

        c = classify_joints(URDF_BASIC)
        n_eq, n_act = postprocess_mjcf(mjcf_file, c, meshdir)

        assert n_eq == 0
        assert n_act == 2

        tree = ET.parse(str(mjcf_file))
        root = tree.getroot()

        # Check compiler was fixed
        compiler = root.find("compiler")
        assert compiler.get("angle") == "radian"
        assert compiler.get("autolimits") == "true"

        # Check option added
        assert root.find("option") is not None

        # Check actuators added
        actuator = root.find("actuator")
        assert actuator is not None
        assert len(actuator.findall("general")) == 2


# ════════════════════════════════════════════════════════════════════════════
# Scene Generation Test
# ════════════════════════════════════════════════════════════════════════════

class TestGenerateScene:
    def test_scene_content(self, tmp_path):
        scene_path = generate_scene(tmp_path, "ur5e.xml", "ur5e")
        assert scene_path.exists()

        content = scene_path.read_text()
        assert '<include file="ur5e.xml"/>' in content
        assert 'model="ur5e scene"' in content
        assert 'name="floor"' in content
        assert "groundplane" in content


# ════════════════════════════════════════════════════════════════════════════
# Directory Convention Tests
# ════════════════════════════════════════════════════════════════════════════

class TestResolveRobotDirPaths:
    def _make_robot_dir(self, tmp_path, robot_name="test_robot"):
        robot_dir = tmp_path / robot_name
        (robot_dir / "urdf").mkdir(parents=True)
        (robot_dir / "mjcf").mkdir(parents=True)
        (robot_dir / "meshes" / "assets").mkdir(parents=True)
        return robot_dir

    def test_auto_discover_urdf(self, tmp_path):
        robot_dir = self._make_robot_dir(tmp_path)
        (robot_dir / "urdf" / "test_robot.urdf").write_text("<robot/>")

        urdf, output, meshdir = resolve_robot_dir_paths(robot_dir)
        assert urdf.name == "test_robot.urdf"
        assert output.parent.name == "mjcf"
        assert output.name == "test_robot.xml"

    def test_prefer_robot_name_urdf(self, tmp_path):
        robot_dir = self._make_robot_dir(tmp_path)
        (robot_dir / "urdf" / "test_robot.urdf").write_text("<robot/>")
        (robot_dir / "urdf" / "other.urdf").write_text("<robot/>")

        urdf, _, _ = resolve_robot_dir_paths(robot_dir)
        assert urdf.name == "test_robot.urdf"

    def test_explicit_urdf_file(self, tmp_path):
        robot_dir = self._make_robot_dir(tmp_path)
        (robot_dir / "urdf" / "custom.urdf.xacro").write_text("<robot/>")

        urdf, output, _ = resolve_robot_dir_paths(robot_dir, "custom.urdf.xacro")
        assert urdf.name == "custom.urdf.xacro"
        assert output.name == "custom.xml"

    def test_meshdir_prefers_assets(self, tmp_path):
        robot_dir = self._make_robot_dir(tmp_path)
        (robot_dir / "urdf" / "test_robot.urdf").write_text("<robot/>")
        (robot_dir / "meshes" / "assets" / "base.obj").write_text("")

        _, _, meshdir = resolve_robot_dir_paths(robot_dir)
        assert meshdir.name == "assets"

    def test_meshdir_falls_back_to_meshes(self, tmp_path):
        robot_dir = self._make_robot_dir(tmp_path)
        (robot_dir / "urdf" / "test_robot.urdf").write_text("<robot/>")
        # assets dir exists but has no OBJ files

        _, _, meshdir = resolve_robot_dir_paths(robot_dir)
        assert meshdir.name == "meshes"
