"""Tests for rtc_tools.validation.compare_mjcf_urdf module.

MJCF/URDF 파서, 데이터 클래스, 비교 로직, 유틸리티 함수를 검증합니다.
파일 I/O 가 필요한 테스트는 tmp_path 에 XML 파일을 생성합니다.
"""

from __future__ import annotations

import math

import pytest

from rtc_tools.validation.compare_mjcf_urdf import (
    InertialParams,
    JointParams,
    _axes_parallel,
    _close,
    _compose_transform,
    _compute_mjcf_world_frames,
    _compute_urdf_world_frames,
    _fmt,
    _identity_3x3,
    _mjcf_body_rotation,
    _parse_floats,
    _quat_to_rot3,
    _rpy_to_rot3,
    _vec_close_3,
    compare,
    parse_mjcf,
    parse_urdf,
)

# ═══════════════════════════════════════════════════════════════════════════
# Helper utilities
# ═══════════════════════════════════════════════════════════════════════════


class TestParseFloats:
    def test_basic(self):
        assert _parse_floats("1.0 2.0 3.0") == [1.0, 2.0, 3.0]

    def test_single(self):
        assert _parse_floats("42") == [42.0]

    def test_negative(self):
        assert _parse_floats("-1.5 0 3.14") == [-1.5, 0.0, 3.14]

    def test_scientific(self):
        result = _parse_floats("1e-4 2.5e3")
        assert result[0] == pytest.approx(1e-4)
        assert result[1] == pytest.approx(2500.0)


class TestClose:
    def test_equal(self):
        assert _close(1.0, 1.0, 1e-6) is True

    def test_within_tolerance(self):
        assert _close(1.0, 1.0 + 1e-5, 1e-4) is True

    def test_outside_tolerance(self):
        assert _close(1.0, 1.1, 1e-4) is False

    def test_zero(self):
        assert _close(0.0, 0.0, 1e-10) is True

    def test_near_zero(self):
        assert _close(0.0, 1e-5, 1e-4) is True
        assert _close(0.0, 1e-3, 1e-4) is False


class TestFmt:
    def test_zero(self):
        assert _fmt(0.0) == "0"

    def test_near_zero(self):
        assert _fmt(1e-12) == "0"

    def test_integer_like(self):
        assert _fmt(150.0) == "150"

    def test_decimal(self):
        assert _fmt(3.14159) == "3.14159"

    def test_negative(self):
        result = _fmt(-2.5)
        assert result == "-2.5"


# ═══════════════════════════════════════════════════════════════════════════
# Data classes
# ═══════════════════════════════════════════════════════════════════════════


class TestInertialParams:
    def test_defaults(self):
        ip = InertialParams()
        assert ip.mass == 0.0
        assert ip.origin_xyz == [0.0, 0.0, 0.0]
        assert ip.diag_inertia == [0.0, 0.0, 0.0]
        assert ip.off_diag_inertia == [0.0, 0.0, 0.0]
        assert ip.origin_rpy == [0.0, 0.0, 0.0]

    def test_no_shared_mutable_defaults(self):
        """두 인스턴스의 mutable 필드가 독립적인지 확인."""
        a = InertialParams()
        b = InertialParams()
        a.origin_xyz[0] = 999
        assert b.origin_xyz[0] == 0.0


class TestJointParams:
    def test_defaults(self):
        jp = JointParams()
        assert jp.axis == [0.0, 0.0, 0.0]
        assert jp.lower == 0.0
        assert jp.upper == 0.0
        assert jp.effort == 0.0
        assert jp.velocity == 0.0
        assert jp.armature == 0.0

    def test_no_shared_mutable_defaults(self):
        a = JointParams()
        b = JointParams()
        a.axis[0] = 999
        assert b.axis[0] == 0.0


# ═══════════════════════════════════════════════════════════════════════════
# parse_urdf
# ═══════════════════════════════════════════════════════════════════════════

URDF_TEMPLATE = """\
<robot name="test_robot">
  <link name="base_link_inertia">
    <inertial>
      <mass value="4.0"/>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <inertia ixx="0.00443" iyy="0.00443" izz="0.0072"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="shoulder_link">
    <inertial>
      <mass value="3.7"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0102" iyy="0.0102" izz="0.00666"
               ixy="0.001" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link_inertia"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1625" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.2832" upper="6.2832" effort="150" velocity="3.14"/>
  </joint>
</robot>
"""

# Test fixtures — names baked into URDF_TEMPLATE / MJCF_TEMPLATE.
# MJCF body "base" maps to URDF link "base_link_inertia"; "shoulder_link" matches both sides.
LINK_MAP_FIXTURE = {"base": "base_link_inertia", "shoulder_link": "shoulder_link"}
URDF_LINK_NAMES = set(LINK_MAP_FIXTURE.values())
MJCF_LINK_NAMES = set(LINK_MAP_FIXTURE.keys())
JOINT_NAMES_FIXTURE = ["shoulder_pan_joint"]
JOINT_NAMES_SET = set(JOINT_NAMES_FIXTURE)


class TestParseUrdf:
    def test_link_mass(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, joints = parse_urdf(urdf, URDF_LINK_NAMES, JOINT_NAMES_SET)

        assert "base_link_inertia" in links
        assert links["base_link_inertia"].mass == pytest.approx(4.0)

    def test_link_inertia(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, _ = parse_urdf(urdf, URDF_LINK_NAMES, JOINT_NAMES_SET)

        ip = links["base_link_inertia"]
        assert ip.diag_inertia[0] == pytest.approx(0.00443)  # ixx
        assert ip.diag_inertia[1] == pytest.approx(0.00443)  # iyy
        assert ip.diag_inertia[2] == pytest.approx(0.0072)  # izz

    def test_off_diagonal_inertia(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, _ = parse_urdf(urdf, URDF_LINK_NAMES, JOINT_NAMES_SET)

        ip = links["shoulder_link"]
        assert ip.off_diag_inertia[0] == pytest.approx(0.001)  # ixy
        assert ip.off_diag_inertia[1] == pytest.approx(0.0)  # ixz
        assert ip.off_diag_inertia[2] == pytest.approx(0.0)  # iyz

    def test_link_origin(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, _ = parse_urdf(urdf, URDF_LINK_NAMES, JOINT_NAMES_SET)

        ip = links["base_link_inertia"]
        assert ip.origin_xyz == [0.0, 0.0, 0.025]

    def test_joint_axis(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        _, joints = parse_urdf(urdf, URDF_LINK_NAMES, JOINT_NAMES_SET)

        jp = joints["shoulder_pan_joint"]
        assert jp.axis == [0.0, 0.0, 1.0]

    def test_joint_limits(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        _, joints = parse_urdf(urdf, URDF_LINK_NAMES, JOINT_NAMES_SET)

        jp = joints["shoulder_pan_joint"]
        assert jp.lower == pytest.approx(-6.2832)
        assert jp.upper == pytest.approx(6.2832)
        assert jp.effort == pytest.approx(150.0)
        assert jp.velocity == pytest.approx(3.14)

    def test_joint_origin(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        _, joints = parse_urdf(urdf, URDF_LINK_NAMES, JOINT_NAMES_SET)

        jp = joints["shoulder_pan_joint"]
        assert jp.origin_xyz == [0.0, 0.0, 0.1625]

    def test_ignores_unknown_links(self, tmp_path):
        """link_names set 에 없는 link 는 결과 dict 에 들어오지 않는다."""
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        # Only request "base_link_inertia"; "shoulder_link" must be filtered out.
        links, _ = parse_urdf(urdf, {"base_link_inertia"}, JOINT_NAMES_SET)
        assert "base_link_inertia" in links
        assert "shoulder_link" not in links

    def test_ignores_unknown_joints(self, tmp_path):
        """joint_names set 에 없는 joint 는 무시."""
        urdf_text = """\
<robot name="test">
  <link name="base_link_inertia"/>
  <link name="custom_link"/>
  <joint name="custom_joint" type="revolute">
    <parent link="base_link_inertia"/>
    <child link="custom_link"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>
"""
        urdf = tmp_path / "test.urdf"
        urdf.write_text(urdf_text)
        _, joints = parse_urdf(urdf, {"base_link_inertia"}, {"shoulder_pan_joint"})
        assert "custom_joint" not in joints


# ═══════════════════════════════════════════════════════════════════════════
# parse_mjcf
# ═══════════════════════════════════════════════════════════════════════════

MJCF_TEMPLATE = """\
<mujoco model="test_robot">
  <default>
    <default class="test_robot">
      <joint axis="0 1 0" armature="0.1"/>
      <general forcerange="-150 150"/>
    </default>
    <default class="size3">
      <joint axis="0 1 0"/>
    </default>
  </default>
  <worldbody>
    <body name="base" pos="0 0 0">
      <inertial mass="4.0" pos="0 0 0.025" diaginertia="0.00443 0.00443 0.0072"/>
      <body name="shoulder_link" pos="0 0 0.1625">
        <joint name="shoulder_pan_joint" class="test_robot" axis="0 0 1"
               range="-6.2832 6.2832"/>
        <!-- URDF shoulder has ixy=0.001 → principal moments [0.0112, 0.0092, 0.00666]
             after diagonalizing the xy-block.  MJCF diaginertia stores those
             eigenvalues so the validator (commit 6033081) sees no mismatch. -->
        <inertial mass="3.7" pos="0 0 0" diaginertia="0.0112 0.0092 0.00666"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class TestParseMjcf:
    def test_link_mass(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        links, _ = parse_mjcf(mjcf, MJCF_LINK_NAMES, JOINT_NAMES_SET)

        assert "base" in links
        assert links["base"].mass == pytest.approx(4.0)

    def test_link_inertia(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        links, _ = parse_mjcf(mjcf, MJCF_LINK_NAMES, JOINT_NAMES_SET)

        ip = links["base"]
        assert ip.diag_inertia == [0.00443, 0.00443, 0.0072]

    def test_joint_axis(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf, MJCF_LINK_NAMES, JOINT_NAMES_SET)

        jp = joints["shoulder_pan_joint"]
        assert jp.axis == [0.0, 0.0, 1.0]

    def test_joint_range(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf, MJCF_LINK_NAMES, JOINT_NAMES_SET)

        jp = joints["shoulder_pan_joint"]
        assert jp.lower == pytest.approx(-6.2832)
        assert jp.upper == pytest.approx(6.2832)

    def test_joint_effort_from_defaults(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf, MJCF_LINK_NAMES, JOINT_NAMES_SET)

        jp = joints["shoulder_pan_joint"]
        assert jp.effort == pytest.approx(150.0)

    def test_joint_armature(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf, MJCF_LINK_NAMES, JOINT_NAMES_SET)

        jp = joints["shoulder_pan_joint"]
        assert jp.armature == pytest.approx(0.1)

    def test_joint_origin_from_body_pos(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf, MJCF_LINK_NAMES, JOINT_NAMES_SET)

        jp = joints["shoulder_pan_joint"]
        assert jp.origin_xyz == [0.0, 0.0, 0.1625]


# ═══════════════════════════════════════════════════════════════════════════
# compare — 전체 비교
# ═══════════════════════════════════════════════════════════════════════════


class TestCompare:
    def test_identical_models_zero_mismatches(self, tmp_path, capsys):
        """동일한 파라미터를 가진 MJCF/URDF 비교 시 mismatch=0."""
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )
        assert mismatches == 0

        captured = capsys.readouterr()
        assert "SUMMARY" in captured.out

    def test_mass_mismatch(self, tmp_path, capsys):
        """질량이 다르면 mismatch 카운트 증가."""
        mjcf_text = MJCF_TEMPLATE.replace('mass="4.0"', 'mass="5.0"', 1)
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )
        assert mismatches >= 1

        captured = capsys.readouterr()
        assert "MASS MISMATCH" in captured.out

    def test_joint_range_mismatch(self, tmp_path, capsys):
        """관절 범위가 다르면 mismatch 카운트 증가."""
        mjcf_text = MJCF_TEMPLATE.replace('range="-6.2832 6.2832"', 'range="-3.14 3.14"')
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )
        assert mismatches >= 1

        captured = capsys.readouterr()
        assert "RANGE MISMATCH" in captured.out

    def test_custom_tolerance(self, tmp_path):
        """큰 tolerance 사용 시 작은 차이가 무시됨."""
        # mass를 아주 약간만 변경
        mjcf_text = MJCF_TEMPLATE.replace('mass="4.0"', 'mass="4.001"', 1)
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        assert (
            compare(
                mjcf,
                urdf,
                link_map=LINK_MAP_FIXTURE,
                joint_names=JOINT_NAMES_FIXTURE,
                tolerance=0.01,
            )
            == 0
        )
        assert (
            compare(
                mjcf,
                urdf,
                link_map=LINK_MAP_FIXTURE,
                joint_names=JOINT_NAMES_FIXTURE,
                tolerance=1e-4,
            )
            >= 1
        )

    def test_off_diagonal_warning(self, tmp_path, capsys):
        """URDF에 off-diagonal inertia가 있으면 NOTE 출력."""
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        compare(mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE)
        captured = capsys.readouterr()
        assert "off-diagonal" in captured.out

    def test_missing_link_warning(self, tmp_path, capsys):
        """MJCF에 link가 없으면 WARN 출력."""
        # shoulder_link 를 삭제한 MJCF
        mjcf_text = """\
<mujoco model="test">
  <default>
    <default class="test_robot">
      <joint axis="0 1 0"/>
      <general forcerange="-150 150"/>
    </default>
  </default>
  <worldbody>
    <body name="base" pos="0 0 0">
      <inertial mass="4.0" pos="0 0 0.025" diaginertia="0.00443 0.00443 0.0072"/>
    </body>
  </worldbody>
</mujoco>
"""
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        compare(mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE)
        captured = capsys.readouterr()
        assert "WARN" in captured.out


# ═══════════════════════════════════════════════════════════════════════════
# SE(3) transform utilities
# ═══════════════════════════════════════════════════════════════════════════


class TestSE3Utilities:
    def test_identity_3x3(self):
        I = _identity_3x3()
        assert I == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    def test_rpy_identity(self):
        R = _rpy_to_rot3(0, 0, 0)
        for i in range(3):
            for j in range(3):
                expected = 1.0 if i == j else 0.0
                assert abs(R[i][j] - expected) < 1e-12

    def test_rpy_90_roll(self):
        R = _rpy_to_rot3(math.pi / 2, 0, 0)
        assert abs(R[0][0] - 1.0) < 1e-12
        assert abs(R[1][2] - (-1.0)) < 1e-12
        assert abs(R[2][1] - 1.0) < 1e-12

    def test_rpy_90_yaw(self):
        R = _rpy_to_rot3(0, 0, math.pi / 2)
        assert abs(R[0][1] - (-1.0)) < 1e-12
        assert abs(R[1][0] - 1.0) < 1e-12

    def test_quat_identity(self):
        R = _quat_to_rot3(1, 0, 0, 0)
        for i in range(3):
            for j in range(3):
                expected = 1.0 if i == j else 0.0
                assert abs(R[i][j] - expected) < 1e-12

    def test_quat_180_z(self):
        R = _quat_to_rot3(0, 0, 0, 1)
        assert abs(R[0][0] - (-1.0)) < 1e-12
        assert abs(R[1][1] - (-1.0)) < 1e-12
        assert abs(R[2][2] - 1.0) < 1e-12

    def test_quat_unnormalized(self):
        R = _quat_to_rot3(0, 0, 0, -2)
        assert abs(R[0][0] - (-1.0)) < 1e-12

    def test_compose_identity(self):
        I = _identity_3x3()
        R, p = _compose_transform(I, [0, 0, 0], I, [1, 2, 3])
        assert _vec_close_3(p, [1, 2, 3], 1e-12)

    def test_compose_rotation_then_translate(self):
        R_parent = _rpy_to_rot3(0, 0, math.pi / 2)
        _, p_world = _compose_transform(R_parent, [0, 0, 0], _identity_3x3(), [1, 0, 0])
        assert _vec_close_3(p_world, [0, 1, 0], 1e-12)

    def test_vec_close_3(self):
        assert _vec_close_3([1, 2, 3], [1, 2, 3], 1e-12) is True
        assert _vec_close_3([1, 2, 3], [1, 2, 4], 0.5) is False


# ═══════════════════════════════════════════════════════════════════════════
# FK chain computation — URDF and MJCF world-frame comparison
# ═══════════════════════════════════════════════════════════════════════════

URDF_FK = """\
<?xml version="1.0"?>
<robot name="fk_test">
  <link name="world"/>
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="base_fixed" type="fixed">
    <parent link="world"/>
    <child link="base"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0" rpy="1.5707963267948966 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" lower="-3.14" upper="3.14" velocity="1"/>
  </joint>
</robot>
"""

MJCF_FK = """\
<mujoco model="fk_test">
  <compiler angle="radian" autolimits="true"/>
  <default>
    <default class="fk_test">
      <joint axis="0 1 0" range="-3.14 3.14"/>
    </default>
  </default>
  <worldbody>
    <body name="base" childclass="fk_test">
      <body name="link1" pos="0 0 1">
        <joint name="joint1" axis="0 0 1"/>
        <body name="link2" pos="0 0 0" quat="1 0 1 0">
          <joint name="joint2"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class TestURDFWorldFrames:
    def test_joint1_position_and_axis(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_FK)
        frames = _compute_urdf_world_frames(urdf, {"joint1", "joint2"})
        pos1, axis1 = frames["joint1"]
        assert _vec_close_3(pos1, [0, 0, 1], 1e-6)
        assert _vec_close_3(axis1, [0, 0, 1], 1e-6)

    def test_joint2_axis_rotated(self, tmp_path):
        """joint2 has rpy=(pi/2,0,0) so local Z maps to world -Y via Rx(90)."""
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_FK)
        frames = _compute_urdf_world_frames(urdf, {"joint1", "joint2"})
        pos2, axis2 = frames["joint2"]
        assert _vec_close_3(pos2, [0, 0, 1], 1e-6)
        assert _vec_close_3(axis2, [0, -1, 0], 1e-6)


class TestMJCFWorldFrames:
    def test_joint1_position_and_axis(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_FK)
        frames = _compute_mjcf_world_frames(mjcf, {"joint1", "joint2"})
        pos1, axis1 = frames["joint1"]
        assert _vec_close_3(pos1, [0, 0, 1], 1e-6)
        assert _vec_close_3(axis1, [0, 0, 1], 1e-6)

    def test_joint2_inherited_axis_rotated(self, tmp_path):
        """joint2 inherits default axis=(0,1,0), body quat Ry(90) preserves Y."""
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_FK)
        frames = _compute_mjcf_world_frames(mjcf, {"joint1", "joint2"})
        pos2, axis2 = frames["joint2"]
        assert _vec_close_3(pos2, [0, 0, 1], 1e-6)
        assert _vec_close_3(axis2, [0, 1, 0], 1e-6)


class TestCrossFormatWorldFrameMatch:
    """Both formats describe the same physical robot — axes must be parallel."""

    def test_all_joints_positions_and_axes_parallel(self, tmp_path):
        """URDF Rx(90)+Z and MJCF Ry(90)+Y produce anti-parallel world axes.
        This is a valid convention difference — same rotation line."""
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_FK)
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_FK)

        joints = {"joint1", "joint2"}
        u_frames = _compute_urdf_world_frames(urdf, joints)
        m_frames = _compute_mjcf_world_frames(mjcf, joints)

        for jname in joints:
            u_pos, u_axis = u_frames[jname]
            m_pos, m_axis = m_frames[jname]
            assert _vec_close_3(u_pos, m_pos, 1e-4), f"{jname} position mismatch"
            is_parallel, _ = _axes_parallel(u_axis, m_axis)
            assert is_parallel, f"{jname} axes not parallel: {u_axis} vs {m_axis}"


# ═══════════════════════════════════════════════════════════════════════════
# _axes_parallel — fixed cosine tolerance, anti-parallel detection
# ═══════════════════════════════════════════════════════════════════════════


class TestAxesParallel:
    def test_same_direction(self):
        ok, same = _axes_parallel([0, 0, 1], [0, 0, 1])
        assert ok is True
        assert same is True

    def test_anti_parallel(self):
        ok, same = _axes_parallel([0, 0, 1], [0, 0, -1])
        assert ok is True
        assert same is False

    def test_not_parallel(self):
        ok, _ = _axes_parallel([1, 0, 0], [0, 1, 0])
        assert ok is False

    def test_zero_vector(self):
        ok, _ = _axes_parallel([0, 0, 0], [1, 0, 0])
        assert ok is False


# ═══════════════════════════════════════════════════════════════════════════
# _mjcf_body_rotation — multiple MuJoCo orientation formats
# ═══════════════════════════════════════════════════════════════════════════


class TestMjcfBodyRotation:
    def _make_body(self, **attribs):
        import xml.etree.ElementTree as ET

        body = ET.Element("body")
        for k, v in attribs.items():
            body.set(k, v)
        return body

    def test_identity_no_attrs(self):
        R = _mjcf_body_rotation(self._make_body())
        assert _identity_3x3() == R

    def test_quat(self):
        R = _mjcf_body_rotation(self._make_body(quat="0 0 0 1"))
        assert abs(R[0][0] - (-1.0)) < 1e-12
        assert abs(R[2][2] - 1.0) < 1e-12

    def test_euler(self):
        R = _mjcf_body_rotation(self._make_body(euler=f"{math.pi / 2} 0 0"))
        assert abs(R[0][0] - 1.0) < 1e-12
        assert abs(R[1][2] - (-1.0)) < 1e-12

    def test_axisangle(self):
        R = _mjcf_body_rotation(self._make_body(axisangle=f"0 0 1 {math.pi}"))
        assert abs(R[0][0] - (-1.0)) < 1e-12
        assert abs(R[1][1] - (-1.0)) < 1e-12
        assert abs(R[2][2] - 1.0) < 1e-12
