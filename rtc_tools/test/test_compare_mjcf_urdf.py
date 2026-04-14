"""Tests for rtc_tools.validation.compare_mjcf_urdf module.

MJCF/URDF 파서, 데이터 클래스, 비교 로직, 유틸리티 함수를 검증합니다.
파일 I/O 가 필요한 테스트는 tmp_path 에 XML 파일을 생성합니다.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from rtc_tools.validation.compare_mjcf_urdf import (
    InertialParams,
    JointParams,
    _close,
    _fmt,
    _parse_floats,
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


class TestParseUrdf:
    def test_link_mass(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, joints = parse_urdf(urdf)

        assert "base_link_inertia" in links
        assert links["base_link_inertia"].mass == pytest.approx(4.0)

    def test_link_inertia(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, _ = parse_urdf(urdf)

        ip = links["base_link_inertia"]
        assert ip.diag_inertia[0] == pytest.approx(0.00443)  # ixx
        assert ip.diag_inertia[1] == pytest.approx(0.00443)  # iyy
        assert ip.diag_inertia[2] == pytest.approx(0.0072)   # izz

    def test_off_diagonal_inertia(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, _ = parse_urdf(urdf)

        ip = links["shoulder_link"]
        assert ip.off_diag_inertia[0] == pytest.approx(0.001)  # ixy
        assert ip.off_diag_inertia[1] == pytest.approx(0.0)    # ixz
        assert ip.off_diag_inertia[2] == pytest.approx(0.0)    # iyz

    def test_link_origin(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, _ = parse_urdf(urdf)

        ip = links["base_link_inertia"]
        assert ip.origin_xyz == [0.0, 0.0, 0.025]

    def test_joint_axis(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        _, joints = parse_urdf(urdf)

        jp = joints["shoulder_pan_joint"]
        assert jp.axis == [0.0, 0.0, 1.0]

    def test_joint_limits(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        _, joints = parse_urdf(urdf)

        jp = joints["shoulder_pan_joint"]
        assert jp.lower == pytest.approx(-6.2832)
        assert jp.upper == pytest.approx(6.2832)
        assert jp.effort == pytest.approx(150.0)
        assert jp.velocity == pytest.approx(3.14)

    def test_joint_origin(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        _, joints = parse_urdf(urdf)

        jp = joints["shoulder_pan_joint"]
        assert jp.origin_xyz == [0.0, 0.0, 0.1625]

    def test_ignores_unknown_links(self, tmp_path):
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        links, _ = parse_urdf(urdf)
        # Only links in MJCF_TO_URDF_LINK.values() are parsed
        for name in links:
            from rtc_tools.validation.compare_mjcf_urdf import MJCF_TO_URDF_LINK
            assert name in MJCF_TO_URDF_LINK.values()

    def test_ignores_unknown_joints(self, tmp_path):
        """JOINT_NAMES 에 없는 joint 는 무시."""
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
        _, joints = parse_urdf(urdf)
        assert "custom_joint" not in joints


# ═══════════════════════════════════════════════════════════════════════════
# parse_mjcf
# ═══════════════════════════════════════════════════════════════════════════

MJCF_TEMPLATE = """\
<mujoco model="test_robot">
  <default>
    <default class="ur5e">
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
        <joint name="shoulder_pan_joint" class="ur5e" axis="0 0 1"
               range="-6.2832 6.2832"/>
        <inertial mass="3.7" pos="0 0 0" diaginertia="0.0102 0.0102 0.00666"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class TestParseMjcf:
    def test_link_mass(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        links, _ = parse_mjcf(mjcf)

        assert "base" in links
        assert links["base"].mass == pytest.approx(4.0)

    def test_link_inertia(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        links, _ = parse_mjcf(mjcf)

        ip = links["base"]
        assert ip.diag_inertia == [0.00443, 0.00443, 0.0072]

    def test_joint_axis(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf)

        jp = joints["shoulder_pan_joint"]
        assert jp.axis == [0.0, 0.0, 1.0]

    def test_joint_range(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf)

        jp = joints["shoulder_pan_joint"]
        assert jp.lower == pytest.approx(-6.2832)
        assert jp.upper == pytest.approx(6.2832)

    def test_joint_effort_from_defaults(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf)

        jp = joints["shoulder_pan_joint"]
        assert jp.effort == pytest.approx(150.0)

    def test_joint_armature(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf)

        jp = joints["shoulder_pan_joint"]
        assert jp.armature == pytest.approx(0.1)

    def test_joint_origin_from_body_pos(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        _, joints = parse_mjcf(mjcf)

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

        mismatches = compare(mjcf, urdf)
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

        mismatches = compare(mjcf, urdf)
        assert mismatches >= 1

        captured = capsys.readouterr()
        assert "MASS MISMATCH" in captured.out

    def test_joint_range_mismatch(self, tmp_path, capsys):
        """관절 범위가 다르면 mismatch 카운트 증가."""
        mjcf_text = MJCF_TEMPLATE.replace(
            'range="-6.2832 6.2832"', 'range="-3.14 3.14"')
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        mismatches = compare(mjcf, urdf)
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

        assert compare(mjcf, urdf, tolerance=0.01) == 0
        assert compare(mjcf, urdf, tolerance=1e-4) >= 1

    def test_off_diagonal_warning(self, tmp_path, capsys):
        """URDF에 off-diagonal inertia가 있으면 NOTE 출력."""
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        compare(mjcf, urdf)
        captured = capsys.readouterr()
        assert "off-diagonal" in captured.out

    def test_missing_link_warning(self, tmp_path, capsys):
        """MJCF에 link가 없으면 WARN 출력."""
        # shoulder_link 를 삭제한 MJCF
        mjcf_text = """\
<mujoco model="test">
  <default>
    <default class="ur5e">
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

        compare(mjcf, urdf)
        captured = capsys.readouterr()
        assert "WARN" in captured.out
