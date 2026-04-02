"""Tests for UrdfParser — URDF loading, graph construction, and joint classification."""

import os
import textwrap

import pytest

from rtc_digital_twin.urdf_parser import (
    JointClassification,
    JointMeta,
    LinkNode,
    MimicParams,
    UrdfParser,
)

# ── Test URDF snippets ─────────────────────────────────────────────────────

URDF_BASIC = """\
<robot name="test">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
  <joint name="joint2" type="prismatic">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
"""

URDF_WITH_FIXED = """\
<robot name="test">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="active_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
  <joint name="fixed_joint" type="fixed">
    <parent link="link1"/>
    <child link="link2"/>
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
  </joint>
  <joint name="mimic_joint" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <mimic joint="master_joint" multiplier="2.0" offset="0.5"/>
  </joint>
  <joint name="mimic_default" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <mimic joint="master_joint"/>
  </joint>
</robot>
"""

URDF_CLOSED_CHAIN = """\
<robot name="test">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="shared_link"/>
  <joint name="tree_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shared_link"/>
  </joint>
  <joint name="loop_joint" type="revolute">
    <parent link="link1"/>
    <child link="shared_link"/>
  </joint>
  <joint name="normal_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
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
  </joint>
  <joint name="active2" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
  <joint name="mimic1" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <mimic joint="active1" multiplier="-1.0" offset="0.0"/>
  </joint>
  <joint name="tree_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shared_link"/>
  </joint>
  <joint name="loop_joint" type="revolute">
    <parent link="link1"/>
    <child link="shared_link"/>
  </joint>
  <joint name="fixed1" type="fixed">
    <parent link="link3"/>
    <child link="base_link"/>
  </joint>
</robot>
"""

URDF_WITH_AXIS_LIMITS = """\
<robot name="test_limits">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="1.0 0.0 0.0"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
</robot>
"""


# ── Tests: Construction ──────────────────────────────────────────────────


class TestConstruction:
    def test_from_xml(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        assert p.robot_name == 'test'
        assert p.urdf_xml == URDF_BASIC

    def test_from_file(self, tmp_path):
        urdf_file = tmp_path / 'robot.urdf'
        urdf_file.write_text(URDF_BASIC)
        p = UrdfParser.from_file(str(urdf_file))
        assert p.robot_name == 'test'
        assert '<robot name="test">' in p.urdf_xml

    def test_missing_file_raises(self):
        with pytest.raises(FileNotFoundError):
            UrdfParser.from_file('/nonexistent/robot.urdf')

    def test_both_sources_raises(self, tmp_path):
        urdf_file = tmp_path / 'robot.urdf'
        urdf_file.write_text(URDF_BASIC)
        with pytest.raises(ValueError, match='exactly one'):
            UrdfParser(urdf_path=str(urdf_file), urdf_xml=URDF_BASIC)

    def test_no_source_raises(self):
        with pytest.raises(ValueError, match='exactly one'):
            UrdfParser()

    def test_invalid_xml_raises(self):
        with pytest.raises(Exception):
            UrdfParser.from_xml('not valid xml')


# ── Tests: Link Graph ────────────────────────────────────────────────────


class TestLinkGraph:
    def test_basic_link_count(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        assert p.num_links == 3

    def test_basic_joint_count(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        assert p.num_joints == 2

    def test_root_link(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        root = p.link_nodes[p.root_index]
        assert root.link_name == 'base_link'
        assert root.parent_index == -1

    def test_parent_child_relationships(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        root_idx = p.get_link_index('base_link')
        link1_idx = p.get_link_index('link1')
        link2_idx = p.get_link_index('link2')

        assert link1_idx in p.link_nodes[root_idx].child_indices
        assert p.link_nodes[link1_idx].parent_index == root_idx
        assert p.link_nodes[link1_idx].parent_joint_name == 'joint1'

        assert link2_idx in p.link_nodes[link1_idx].child_indices
        assert p.link_nodes[link2_idx].parent_index == link1_idx

    def test_get_link_index_raises(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        with pytest.raises(KeyError):
            p.get_link_index('nonexistent')


# ── Tests: Joint Classification ──────────────────────────────────────────


class TestClassifyJoints:
    def test_basic_all_active(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        c = p.classification
        assert c.active_names == {'joint1', 'joint2'}
        assert len(c.passive_mimic) == 0
        assert len(c.passive_closed_chain) == 0
        assert len(c.fixed) == 0

    def test_fixed_filtered(self):
        p = UrdfParser.from_xml(URDF_WITH_FIXED)
        c = p.classification
        assert c.active_names == {'active_joint'}
        assert c.fixed == ['fixed_joint']
        assert 'fixed_joint' not in c.active
        assert 'fixed_joint' not in c.passive_mimic

    def test_mimic_detection(self):
        p = UrdfParser.from_xml(URDF_WITH_MIMIC)
        c = p.classification
        assert c.active_names == {'master_joint'}
        assert set(c.passive_mimic.keys()) == {'mimic_joint', 'mimic_default'}

        mimic_info = c.passive_mimic['mimic_joint']
        assert mimic_info.mimic is not None
        assert mimic_info.mimic.master_joint == 'master_joint'
        assert mimic_info.mimic.multiplier == 2.0
        assert mimic_info.mimic.offset == 0.5

    def test_mimic_default_params(self):
        p = UrdfParser.from_xml(URDF_WITH_MIMIC)
        default_info = p.classification.passive_mimic['mimic_default']
        assert default_info.mimic is not None
        assert default_info.mimic.multiplier == 1.0
        assert default_info.mimic.offset == 0.0

    def test_closed_chain_detection(self):
        p = UrdfParser.from_xml(URDF_CLOSED_CHAIN)
        c = p.classification
        assert 'tree_joint' in c.active
        assert 'normal_joint' in c.active
        assert 'loop_joint' in c.passive_closed_chain
        info = c.passive_closed_chain['loop_joint']
        assert info.is_closed_chain is True

    def test_mixed_classification(self):
        p = UrdfParser.from_xml(URDF_MIXED)
        c = p.classification
        assert c.active_names == {'active1', 'active2', 'tree_joint'}
        assert set(c.passive_mimic.keys()) == {'mimic1'}
        assert set(c.passive_closed_chain.keys()) == {'loop_joint'}
        assert c.fixed == ['fixed1']

    def test_passive_names_property(self):
        p = UrdfParser.from_xml(URDF_MIXED)
        assert p.classification.passive_names == {'mimic1', 'loop_joint'}

    def test_get_active_joint_names(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        names = p.get_active_joint_names()
        assert names == sorted(['joint1', 'joint2'])

    def test_get_mimic_joint_names(self):
        p = UrdfParser.from_xml(URDF_WITH_MIMIC)
        names = p.get_mimic_joint_names()
        assert names == sorted(['mimic_joint', 'mimic_default'])

    def test_get_fixed_joint_names(self):
        p = UrdfParser.from_xml(URDF_WITH_FIXED)
        assert p.get_fixed_joint_names() == ['fixed_joint']


# ── Tests: Joint Metadata ────────────────────────────────────────────────


class TestJointMeta:
    def test_basic_fields(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        meta = p.get_joint_meta('joint1')
        assert meta.name == 'joint1'
        assert meta.joint_type == 'revolute'
        assert meta.parent_link == 'base_link'
        assert meta.child_link == 'link1'
        assert meta.mimic is None
        assert meta.is_closed_chain is False

    def test_axis_and_limits(self):
        p = UrdfParser.from_xml(URDF_WITH_AXIS_LIMITS)
        meta = p.get_joint_meta('joint1')
        assert meta.axis == (1.0, 0.0, 0.0)
        assert meta.lower == pytest.approx(-3.14)
        assert meta.upper == pytest.approx(3.14)

    def test_default_axis(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        meta = p.get_joint_meta('joint1')
        assert meta.axis == (0.0, 0.0, 1.0)

    def test_nonexistent_joint_raises(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        with pytest.raises(KeyError):
            p.get_joint_meta('nonexistent')


# ── Tests: Mimic Computation ─────────────────────────────────────────────


class TestComputeMimicPositions:
    def test_basic_computation(self):
        p = UrdfParser.from_xml(URDF_WITH_MIMIC)
        mimic_pos = p.compute_mimic_positions({'master_joint': 1.0})
        assert mimic_pos['mimic_joint'] == pytest.approx(2.0 * 1.0 + 0.5)
        assert mimic_pos['mimic_default'] == pytest.approx(1.0 * 1.0 + 0.0)

    def test_missing_master_skipped(self):
        p = UrdfParser.from_xml(URDF_WITH_MIMIC)
        mimic_pos = p.compute_mimic_positions({})
        assert len(mimic_pos) == 0

    def test_no_mimic_joints(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        mimic_pos = p.compute_mimic_positions({'joint1': 1.0})
        assert len(mimic_pos) == 0

    def test_negative_multiplier(self):
        p = UrdfParser.from_xml(URDF_MIXED)
        mimic_pos = p.compute_mimic_positions({'active1': 2.0})
        assert mimic_pos['mimic1'] == pytest.approx(-1.0 * 2.0 + 0.0)


# ── Tests: Joint Validation ──────────────────────────────────────────────


class TestValidateJoints:
    def test_all_covered(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        covered, missing = p.validate_joints({'joint1', 'joint2', 'extra'})
        assert covered == {'joint1', 'joint2'}
        assert missing == set()

    def test_some_missing(self):
        p = UrdfParser.from_xml(URDF_BASIC)
        covered, missing = p.validate_joints({'joint1'})
        assert covered == {'joint1'}
        assert missing == {'joint2'}

    def test_mimic_not_required(self):
        p = UrdfParser.from_xml(URDF_WITH_MIMIC)
        covered, missing = p.validate_joints({'master_joint'})
        assert missing == set()


# ── Tests: File Loading ──────────────────────────────────────────────────


class TestFileLoading:
    def test_load_from_file(self, tmp_path):
        urdf_file = tmp_path / 'robot.urdf'
        urdf_file.write_text(URDF_WITH_MIMIC)
        p = UrdfParser.from_file(str(urdf_file))
        assert p.robot_name == 'test'
        assert p.classification.active_names == {'master_joint'}
        assert set(p.classification.passive_mimic.keys()) == {
            'mimic_joint', 'mimic_default'}

    def test_load_from_subdirectory(self, tmp_path):
        sub = tmp_path / 'urdf'
        sub.mkdir()
        urdf_file = sub / 'robot.urdf'
        urdf_file.write_text(URDF_BASIC)
        p = UrdfParser.from_file(str(urdf_file))
        assert p.num_links == 3
