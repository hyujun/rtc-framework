"""Tests for URDF joint classification and validation."""

import pytest

from rtc_digital_twin.urdf_validator import (
    JointClassification,
    JointInfo,
    MimicParams,
    classify_joints,
    compute_mimic_positions,
    parse_required_joints,
    validate_joints,
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


# ── Tests ──────────────────────────────────────────────────────────────────


class TestClassifyJoints:
    def test_basic_all_active(self):
        c = classify_joints(URDF_BASIC)
        assert c.active_names == {'joint1', 'joint2'}
        assert len(c.passive_mimic) == 0
        assert len(c.passive_closed_chain) == 0
        assert len(c.fixed) == 0

    def test_fixed_filtered(self):
        c = classify_joints(URDF_WITH_FIXED)
        assert c.active_names == {'active_joint'}
        assert c.fixed == ['fixed_joint']
        assert 'fixed_joint' not in c.active
        assert 'fixed_joint' not in c.passive_mimic

    def test_mimic_detection(self):
        c = classify_joints(URDF_WITH_MIMIC)
        assert c.active_names == {'master_joint'}
        assert set(c.passive_mimic.keys()) == {'mimic_joint', 'mimic_default'}

        mimic_info = c.passive_mimic['mimic_joint']
        assert mimic_info.mimic is not None
        assert mimic_info.mimic.master_joint == 'master_joint'
        assert mimic_info.mimic.multiplier == 2.0
        assert mimic_info.mimic.offset == 0.5

    def test_mimic_default_params(self):
        c = classify_joints(URDF_WITH_MIMIC)
        default_info = c.passive_mimic['mimic_default']
        assert default_info.mimic is not None
        assert default_info.mimic.multiplier == 1.0
        assert default_info.mimic.offset == 0.0

    def test_closed_chain_detection(self):
        c = classify_joints(URDF_CLOSED_CHAIN)
        assert 'tree_joint' in c.active
        assert 'normal_joint' in c.active
        assert 'loop_joint' in c.passive_closed_chain
        info = c.passive_closed_chain['loop_joint']
        assert info.is_closed_chain is True

    def test_mixed_classification(self):
        c = classify_joints(URDF_MIXED)
        assert c.active_names == {'active1', 'active2', 'tree_joint'}
        assert set(c.passive_mimic.keys()) == {'mimic1'}
        assert set(c.passive_closed_chain.keys()) == {'loop_joint'}
        assert c.fixed == ['fixed1']

    def test_joint_info_fields(self):
        c = classify_joints(URDF_BASIC)
        info = c.active['joint1']
        assert info.name == 'joint1'
        assert info.joint_type == 'revolute'
        assert info.parent_link == 'base_link'
        assert info.child_link == 'link1'
        assert info.mimic is None
        assert info.is_closed_chain is False

    def test_passive_names_property(self):
        c = classify_joints(URDF_MIXED)
        assert c.passive_names == {'mimic1', 'loop_joint'}


class TestParseRequiredJointsBackwardCompat:
    def test_matches_active_names(self):
        for urdf in [URDF_BASIC, URDF_WITH_FIXED, URDF_WITH_MIMIC,
                     URDF_CLOSED_CHAIN, URDF_MIXED]:
            required = parse_required_joints(urdf)
            classification = classify_joints(urdf)
            assert required == classification.active_names

    def test_basic_result(self):
        assert parse_required_joints(URDF_BASIC) == {'joint1', 'joint2'}

    def test_excludes_mimic(self):
        required = parse_required_joints(URDF_WITH_MIMIC)
        assert 'mimic_joint' not in required
        assert 'master_joint' in required


class TestComputeMimicPositions:
    def test_basic_computation(self):
        c = classify_joints(URDF_WITH_MIMIC)
        positions = {'master_joint': 1.0}
        mimic_pos = compute_mimic_positions(c, positions)
        assert mimic_pos['mimic_joint'] == pytest.approx(2.0 * 1.0 + 0.5)
        assert mimic_pos['mimic_default'] == pytest.approx(1.0 * 1.0 + 0.0)

    def test_missing_master_skipped(self):
        c = classify_joints(URDF_WITH_MIMIC)
        positions: dict[str, float] = {}
        mimic_pos = compute_mimic_positions(c, positions)
        assert len(mimic_pos) == 0

    def test_no_mimic_joints(self):
        c = classify_joints(URDF_BASIC)
        positions = {'joint1': 1.0}
        mimic_pos = compute_mimic_positions(c, positions)
        assert len(mimic_pos) == 0

    def test_negative_multiplier(self):
        c = classify_joints(URDF_MIXED)
        positions = {'active1': 2.0}
        mimic_pos = compute_mimic_positions(c, positions)
        assert mimic_pos['mimic1'] == pytest.approx(-1.0 * 2.0 + 0.0)


class TestValidateJoints:
    def test_all_covered(self):
        covered, missing = validate_joints({'a', 'b'}, {'a', 'b', 'c'})
        assert covered == {'a', 'b'}
        assert missing == set()

    def test_some_missing(self):
        covered, missing = validate_joints({'a', 'b', 'c'}, {'a'})
        assert covered == {'a'}
        assert missing == {'b', 'c'}
