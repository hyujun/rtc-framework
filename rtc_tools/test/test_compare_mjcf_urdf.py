"""Tests for rtc_tools.validation.compare_mjcf_urdf module.

MJCF/URDF 파서, 데이터 클래스, 비교 로직, 유틸리티 함수를 검증합니다.
파일 I/O 가 필요한 테스트는 tmp_path 에 XML 파일을 생성합니다.
"""

from __future__ import annotations

import math
import sys

import pytest

from rtc_tools.validation.compare_mjcf_urdf import (
    InertialParams,
    JointParams,
    _axes_parallel,
    _axis_line_offset,
    _close,
    _compose_transform,
    _compute_mjcf_world_frames,
    _compute_urdf_world_frames,
    _detect_joint_types,
    _fmt,
    _fuse_urdf_inertials,
    _identity_3x3,
    _invert_transform,
    _load_link_map,
    _mjcf_body_rotation,
    _mjcf_body_world_frames,
    _mjcf_named_frames,
    _parse_floats,
    _quat_to_rot3,
    _resolve_alignment,
    _resolve_link_map,
    _rpy_to_rot3,
    _urdf_link_world_frames,
    _validate_fuse,
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
        # mujoco 부재 시 _compare_structure() 가 통째로 skip 되고 (0, 1) 을 돌려주므로
        # `mismatches == 0` 이 "비교했더니 일치" 인지 "비교를 안 함" 인지 구분되지 않는다.
        # compare() 의 반환은 -> int 라 그 사실이 호출자 경계를 못 넘는다 → skip 으로 드러낸다 (#385).
        pytest.importorskip("mujoco")
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
        pytest.importorskip("mujoco")  # == 0 단언이 구조 비교를 실제로 태워야 한다 (#385)
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
        # 맨 "WARN" 은 mujoco 부재 시의 폴백 경고 문구로도 충족되므로 (#385) 이 테스트가
        # 겨냥한 경고를 지목한다 — 없는 body 를 link_map 이 가리킬 때 나오는 것이다.
        assert "shoulder_link: not found in MJCF" in captured.out

    def test_link_map_naming_difference_not_reported_missing(self, tmp_path, capsys):
        """MJCF body 와 URDF link 이름이 다를 뿐이면 missing-link 오탐이 없어야 한다.

        픽스처는 MJCF body ``base`` ↔ URDF link ``base_link_inertia`` 로 의도적으로
        다르게 짓고 LINK_MAP_FIXTURE 로 잇는다 — ``--link-map`` 이 지원하는 바로 그
        경우다.  구조 비교가 그 map 을 못 받던 시절엔 원시 이름 차집합이라 흡수가
        없는데도 MISMATCH 를 냈고, 툴 자신의 fusestatic Hint 가 오진을 유도했다 (#385).
        """
        pytest.importorskip("mujoco")
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)

        compare(mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE)

        captured = capsys.readouterr()
        assert "absent from MJCF" not in captured.out
        # 질량이 하나도 안 샜다는 것이 "흡수가 아니라 이름 차이" 라는 증거다.
        assert "total mass" in captured.out
        assert "[MISMATCH] total mass" not in captured.out

    def test_mapped_link_absorbed_still_reported(self, tmp_path, capsys):
        """link_map 이 가리켜도 컴파일된 모델에 없으면 여전히 MISMATCH 여야 한다.

        위 테스트의 오탐 제거가 *filter* 가 아니라 *translate* 임을 고정한다 —
        실제로 흡수된 body 는 컴파일 결과에 없으므로 map 의 역참조가 실패하고,
        진짜 질량 손실 신호는 살아남는다.  이 단언이 없으면 훗날 "단순화" 가
        map 에 있는 링크를 통째로 면제해 진짜 결함을 조용히 삼킬 수 있다.
        """
        pytest.importorskip("mujoco")
        # shoulder_link 를 지운 MJCF — link_map 은 여전히 그것을 가리킨다.
        mjcf_text = """\
<mujoco model="test">
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

        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )

        captured = capsys.readouterr()
        assert "absent from MJCF" in captured.out
        assert "shoulder_link" in captured.out
        assert "mass=3.7 kg lost" in captured.out
        assert mismatches >= 1


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


# ═══════════════════════════════════════════════════════════════════════════
# Axis-line joint comparison (#392)
#
# A revolute joint is defined by its axis LINE, not by where its origin sits
# on that line.  MJCF and URDF routinely place the origin differently on the
# same line, so only the perpendicular offset is a real disagreement.
# ═══════════════════════════════════════════════════════════════════════════


class TestAxisLineOffset:
    def test_pure_along_axis(self):
        along, perp, mag = _axis_line_offset([0, 0, 0], [0, 0, 0.5], [0, 0, 1])
        assert abs(along - 0.5) < 1e-12
        assert mag < 1e-12
        assert _vec_close_3(perp, [0, 0, 0], 1e-12)

    def test_pure_perpendicular(self):
        along, _perp, mag = _axis_line_offset([0, 0, 0], [0.3, 0, 0], [0, 0, 1])
        assert abs(along) < 1e-12
        assert abs(mag - 0.3) < 1e-12

    def test_mixed(self):
        along, _perp, mag = _axis_line_offset([0, 0, 0], [0.3, 0, 0.5], [0, 0, 1])
        assert abs(along - 0.5) < 1e-12
        assert abs(mag - 0.3) < 1e-12

    def test_unnormalized_axis(self):
        """Magnitude of the axis vector must not scale the decomposition."""
        along, _perp, mag = _axis_line_offset([0, 0, 0], [0.3, 0, 0.5], [0, 0, 7.0])
        assert abs(along - 0.5) < 1e-12
        assert abs(mag - 0.3) < 1e-12

    def test_degenerate_axis_reports_full_difference(self):
        """A zero-length axis must not silently absorb the difference."""
        along, perp, mag = _axis_line_offset([0, 0, 0], [0, 0, 0.5], [0, 0, 0])
        assert along == 0.0
        assert _vec_close_3(perp, [0, 0, 0.5], 1e-12)
        assert abs(mag - 0.5) < 1e-12


class TestAxisLineCompare:
    """compare() must accept origin slide along the axis but catch any offset
    perpendicular to it — right down to the tolerance boundary."""

    #: The joint origin URDF_TEMPLATE ships with.
    DEFAULT_ORIGIN = (0.0, 0.0, 0.1625)

    def _files(self, tmp_path, urdf_origin, *, prismatic=False, compensate_com=True):
        mjcf_text = MJCF_TEMPLATE
        urdf_text = URDF_TEMPLATE.replace('xyz="0 0 0.1625"', f'xyz="{urdf_origin}"')
        if compensate_com:
            # Moving the joint origin carries the child link with it, so the
            # child's mass ends up somewhere else in the world.  A real model
            # that shifts a joint origin along its axis compensates the child's
            # inertial origin to keep the physics put (ur5e agrees to 5e-11);
            # do the same here so these tests keep isolating the *joint*
            # signal instead of also tripping the world-COM check (#416).
            dx, dy, dz = (
                float(v) - d for v, d in zip(urdf_origin.split(), self.DEFAULT_ORIGIN, strict=True)
            )
            urdf_text = urdf_text.replace(
                '<origin xyz="0 0 0" rpy="0 0 0"/>',
                f'<origin xyz="{-dx} {-dy} {-dz}" rpy="0 0 0"/>',
            )
        if prismatic:
            mjcf_text = mjcf_text.replace(
                '<joint name="shoulder_pan_joint" class="test_robot"',
                '<joint name="shoulder_pan_joint" type="slide" class="test_robot"',
            )
            urdf_text = urdf_text.replace(
                '<joint name="shoulder_pan_joint" type="revolute">',
                '<joint name="shoulder_pan_joint" type="prismatic">',
            )
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(urdf_text)
        return mjcf, urdf

    def test_slide_along_axis_is_not_a_mismatch(self, tmp_path, capsys):
        """URDF origin 0.5 m further along its own Z axis — same axis line."""
        pytest.importorskip("mujoco")
        mjcf, urdf = self._files(tmp_path, "0 0 0.6625")
        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )
        captured = capsys.readouterr()
        assert "AXIS-LINE MISMATCH" not in captured.out
        assert "along the joint axis" in captured.out
        assert mismatches == 0

    def test_perpendicular_offset_above_tolerance_is_caught(self, tmp_path, capsys):
        """2e-4 perpendicular, just above the 1e-4 default tolerance."""
        mjcf, urdf = self._files(tmp_path, "0.0002 0 0.1625")
        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )
        captured = capsys.readouterr()
        assert "AXIS-LINE MISMATCH" in captured.out
        assert mismatches >= 1

    def test_perpendicular_offset_below_tolerance_passes(self, tmp_path, capsys):
        """5e-5 perpendicular, just under tolerance — the boundary is tight,
        so the test above is not passing on gross magnitude alone."""
        pytest.importorskip("mujoco")
        mjcf, urdf = self._files(tmp_path, "0.00005 0 0.1625")
        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )
        captured = capsys.readouterr()
        assert "AXIS-LINE MISMATCH" not in captured.out
        assert mismatches == 0

    def test_uncompensated_slide_moves_the_link_and_is_caught(self, tmp_path, capsys):
        """The axis-line relaxation must not excuse a link that actually moved.

        Sliding the joint origin without compensating the child's inertial
        origin puts 3.7 kg half a metre away.  The axis *line* is unchanged and
        the joint section is right to stay quiet — the world-COM check is the
        only thing that sees it (#416)."""
        pytest.importorskip("mujoco")
        mjcf, urdf = self._files(tmp_path, "0 0 0.6625", compensate_com=False)
        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )
        captured = capsys.readouterr()
        assert "AXIS-LINE MISMATCH" not in captured.out
        assert "COM MISMATCH (world)" in captured.out
        assert mismatches >= 1

    def test_prismatic_slide_along_axis_is_still_a_mismatch(self, tmp_path, capsys):
        """A prismatic origin sets its zero position, so it stays a POINT test."""
        mjcf, urdf = self._files(tmp_path, "0 0 0.6625", prismatic=True)
        mismatches = compare(
            mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE
        )
        captured = capsys.readouterr()
        assert "POSITION MISMATCH" in captured.out
        assert "AXIS-LINE MISMATCH" not in captured.out
        assert mismatches >= 1


class TestWorldComComparison:
    """The COM is only comparable in a frame both files agree on (#416)."""

    def _run(self, tmp_path, capsys, *, mjcf_text=MJCF_TEMPLATE, urdf_text=URDF_TEMPLATE):
        pytest.importorskip("mujoco")
        mjcf = tmp_path / "c.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "c.urdf"
        urdf.write_text(urdf_text)
        n = compare(mjcf, urdf, link_map=LINK_MAP_FIXTURE, joint_names=JOINT_NAMES_FIXTURE)
        return n, capsys.readouterr().out

    def test_matching_models_report_com_ok(self, tmp_path, capsys):
        n, out = self._run(tmp_path, capsys)
        assert "COM (world)" in out
        assert "COM MISMATCH" not in out
        assert n == 0

    def test_shifted_com_is_caught_while_mass_and_inertia_stay_put(self, tmp_path, capsys):
        """Mass and principal moments are blind to a translated COM — moving it
        by 1 cm changes neither, so this fires only if the COM axis exists."""
        mjcf_text = MJCF_TEMPLATE.replace(
            '<inertial mass="3.7" pos="0 0 0" diaginertia',
            '<inertial mass="3.7" pos="0 0 0.01" diaginertia',
        )
        n, out = self._run(tmp_path, capsys, mjcf_text=mjcf_text)
        assert "COM MISMATCH (world)" in out
        assert "MASS MISMATCH" not in out
        assert "INERTIA MISMATCH" not in out
        assert n >= 1

    def test_body_frame_convention_difference_is_not_flagged(self, tmp_path, capsys):
        """ur5e's shape: MJCF puts the body origin 5 cm further along the joint
        axis and pulls the inertial origin back by the same amount.  Local COMs
        then differ by 0.05 while the physics is identical — comparing the raw
        local vectors would false-positive on every such model."""
        mjcf_text = MJCF_TEMPLATE.replace(
            '<body name="shoulder_link" pos="0 0 0.1625">',
            '<body name="shoulder_link" pos="0 0 0.2125">',
        ).replace(
            '<inertial mass="3.7" pos="0 0 0" diaginertia',
            '<inertial mass="3.7" pos="0 0 -0.05" diaginertia',
        )
        n, out = self._run(tmp_path, capsys, mjcf_text=mjcf_text)
        assert "COM MISMATCH" not in out
        assert n == 0

    def test_alignment_is_applied_to_the_com(self, tmp_path, capsys):
        """Without --align-frames the Rz(pi) base makes every COM disagree;
        declaring the common frame must fix the COM axis too, not just joints."""
        pytest.importorskip("mujoco")
        mjcf = tmp_path / "a.xml"
        mjcf.write_text(MJCF_ALIGN)
        urdf = tmp_path / "a.urdf"
        urdf.write_text(URDF_ALIGN)
        compare(mjcf, urdf, link_map=ALIGN_LINK_MAP, joint_names=["j1"])
        assert "COM MISMATCH (world)" in capsys.readouterr().out

        align = _resolve_alignment(mjcf, urdf, "base", "base_link")
        compare(mjcf, urdf, link_map=ALIGN_LINK_MAP, joint_names=["j1"], align=align)
        assert "COM MISMATCH" not in capsys.readouterr().out


class TestDetectJointTypes:
    def test_revolute_when_both_agree(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        types = _detect_joint_types(mjcf, urdf, JOINT_NAMES_SET)
        assert types["shoulder_pan_joint"] == "revolute"

    def test_not_revolute_when_urdf_is_prismatic(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(MJCF_TEMPLATE)
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE.replace('type="revolute"', 'type="prismatic"'))
        types = _detect_joint_types(mjcf, urdf, JOINT_NAMES_SET)
        assert types["shoulder_pan_joint"] == "other"

    def test_not_revolute_when_mjcf_is_slide(self, tmp_path):
        mjcf = tmp_path / "test.xml"
        mjcf.write_text(
            MJCF_TEMPLATE.replace(
                '<joint name="shoulder_pan_joint" class="test_robot"',
                '<joint name="shoulder_pan_joint" type="slide" class="test_robot"',
            )
        )
        urdf = tmp_path / "test.urdf"
        urdf.write_text(URDF_TEMPLATE)
        types = _detect_joint_types(mjcf, urdf, JOINT_NAMES_SET)
        assert types["shoulder_pan_joint"] == "other"


# ═══════════════════════════════════════════════════════════════════════════
# Declared base-frame alignment (#392)
#
# The two formats need not agree on where "world" is.  The caller declares one
# physically identical frame per side rather than letting the tool fit the
# transform, which would absorb a genuine divergence into the fit.
# ═══════════════════════════════════════════════════════════════════════════

MJCF_ALIGN = """\
<mujoco model="align_test">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="base" quat="0 0 0 -1">
      <inertial mass="1.0" pos="0 0 0" diaginertia="0.01 0.01 0.01"/>
      <body name="link1" pos="0.3 0 0.2">
        <inertial mass="2.0" pos="0 0 0" diaginertia="0.02 0.02 0.02"/>
        <joint name="j1" axis="0 0 1" range="-3.14 3.14"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general joint="j1" forcerange="-100 100"/>
  </actuator>
</mujoco>
"""

URDF_ALIGN = """\
<?xml version="1.0"?>
<robot name="align_test">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="link1">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.3 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>
</robot>
"""

ALIGN_LINK_MAP = {"base": "base_link", "link1": "link1"}


class TestFrameCollections:
    def test_mjcf_world_is_identity(self, tmp_path):
        mjcf = tmp_path / "a.xml"
        mjcf.write_text(MJCF_ALIGN)
        frames = _mjcf_body_world_frames(mjcf)
        r, p = frames["world"]
        assert r == _identity_3x3()
        assert _vec_close_3(p, [0, 0, 0], 1e-12)

    def test_mjcf_body_pose_accumulates(self, tmp_path):
        """base carries Rz(pi), so link1's local +x lands on world -x."""
        mjcf = tmp_path / "a.xml"
        mjcf.write_text(MJCF_ALIGN)
        _r, p = _mjcf_body_world_frames(mjcf)["link1"]
        assert _vec_close_3(p, [-0.3, 0, 0.2], 1e-12)

    def test_urdf_link_frames_include_roots(self, tmp_path):
        urdf = tmp_path / "a.urdf"
        urdf.write_text(URDF_ALIGN)
        frames = _urdf_link_world_frames(urdf)
        assert _vec_close_3(frames["base_link"][1], [0, 0, 0], 1e-12)
        assert _vec_close_3(frames["link1"][1], [0.3, 0, 0.2], 1e-12)

    def test_invert_transform_round_trips(self):
        r = _rpy_to_rot3(0.3, -0.7, 1.1)
        p = [0.4, -0.2, 0.9]
        r_inv, p_inv = _invert_transform(r, p)
        r_back, p_back = _compose_transform(r, p, r_inv, p_inv)
        for i in range(3):
            assert abs(p_back[i]) < 1e-12
            for j in range(3):
                assert abs(r_back[i][j] - _identity_3x3()[i][j]) < 1e-12


class TestResolveAlignment:
    def test_recovers_rz180(self, tmp_path):
        mjcf = tmp_path / "a.xml"
        mjcf.write_text(MJCF_ALIGN)
        urdf = tmp_path / "a.urdf"
        urdf.write_text(URDF_ALIGN)
        r, p = _resolve_alignment(mjcf, urdf, "base", "base_link")
        expected = _quat_to_rot3(0, 0, 0, 1)
        for i in range(3):
            assert abs(p[i]) < 1e-12
            for j in range(3):
                assert abs(r[i][j] - expected[i][j]) < 1e-12

    def test_unknown_mjcf_frame_raises(self, tmp_path):
        mjcf = tmp_path / "a.xml"
        mjcf.write_text(MJCF_ALIGN)
        urdf = tmp_path / "a.urdf"
        urdf.write_text(URDF_ALIGN)
        with pytest.raises(SystemExit, match="no body named 'nope'"):
            _resolve_alignment(mjcf, urdf, "nope", "base_link")

    def test_unknown_urdf_frame_raises(self, tmp_path):
        mjcf = tmp_path / "a.xml"
        mjcf.write_text(MJCF_ALIGN)
        urdf = tmp_path / "a.urdf"
        urdf.write_text(URDF_ALIGN)
        with pytest.raises(SystemExit, match="no link named 'nope'"):
            _resolve_alignment(mjcf, urdf, "base", "nope")


class TestCompareWithAlignment:
    """The same pair must fail without the declaration and pass with it —
    otherwise --align-frames could be a no-op and nothing would reveal it."""

    def _files(self, tmp_path):
        mjcf = tmp_path / "a.xml"
        mjcf.write_text(MJCF_ALIGN)
        urdf = tmp_path / "a.urdf"
        urdf.write_text(URDF_ALIGN)
        return mjcf, urdf

    def test_without_alignment_reports_the_whole_robot_offset(self, tmp_path, capsys):
        mjcf, urdf = self._files(tmp_path)
        mismatches = compare(mjcf, urdf, link_map=ALIGN_LINK_MAP, joint_names=["j1"])
        captured = capsys.readouterr()
        assert "AXIS-LINE MISMATCH" in captured.out
        assert mismatches >= 1

    def test_with_alignment_matches(self, tmp_path, capsys):
        pytest.importorskip("mujoco")
        mjcf, urdf = self._files(tmp_path)
        align = _resolve_alignment(mjcf, urdf, "base", "base_link")
        mismatches = compare(mjcf, urdf, link_map=ALIGN_LINK_MAP, joint_names=["j1"], align=align)
        captured = capsys.readouterr()
        assert "AXIS-LINE MISMATCH" not in captured.out
        assert "Base alignment: declared" in captured.out
        assert mismatches == 0


# ═══════════════════════════════════════════════════════════════════════════
# Structural comparison: massless frames and the link_map-aware presence test
# (#392)
#
# A zero-mass URDF link is a pure coordinate frame; MuJoCo not giving it a body
# is correct behaviour.  Excusing them must NOT weaken the fusestatic signal
# #385 added, and must not break models that DO materialise their massless
# frames (iiwa7 has 1, leap_hand 5).
# ═══════════════════════════════════════════════════════════════════════════

URDF_STRUCT = """\
<?xml version="1.0"?>
<robot name="struct_test">
  <link name="base_link_inertia">
    <inertial>
      <mass value="4.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="shoulder_link">
    <inertial>
      <mass value="3.7"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="tool0"/>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link_inertia"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1625" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>
  <joint name="tool_fixed" type="fixed">
    <parent link="shoulder_link"/>
    <child link="tool0"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
"""

MJCF_STRUCT = """\
<mujoco model="struct_test">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="base">
      <inertial mass="4.0" pos="0 0 0" diaginertia="0.01 0.01 0.01"/>
      <body name="shoulder_link" pos="0 0 0.1625">
        <inertial mass="3.7" pos="0 0 0" diaginertia="0.02 0.02 0.02"/>
        <joint name="shoulder_pan_joint" axis="0 0 1" range="-3.14 3.14"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general joint="shoulder_pan_joint" forcerange="-100 100"/>
  </actuator>
</mujoco>
"""

STRUCT_LINK_MAP = {"base": "base_link_inertia", "shoulder_link": "shoulder_link"}
STRUCT_JOINTS = ["shoulder_pan_joint"]


class TestMasslessFrameAccounting:
    def _run(self, tmp_path, capsys, urdf_text=URDF_STRUCT, mjcf_text=MJCF_STRUCT):
        mjcf = tmp_path / "s.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "s.urdf"
        urdf.write_text(urdf_text)
        n = compare(mjcf, urdf, link_map=STRUCT_LINK_MAP, joint_names=STRUCT_JOINTS)
        return n, capsys.readouterr().out

    def test_massless_frame_absent_is_noted_not_counted(self, tmp_path, capsys):
        pytest.importorskip("mujoco")
        n, out = self._run(tmp_path, capsys)
        assert "massless URDF frames with no MJCF body" in out
        assert "tool0" in out
        assert "body count: 2  OK" in out
        assert n == 0

    def test_massive_link_absent_is_still_counted(self, tmp_path, capsys):
        """The #385 fusestatic signal must survive the massless exemption."""
        # Structural comparison needs the compiled model; without mujoco
        # _compare_structure() returns (0, 1) and asserts nothing (#385).
        pytest.importorskip("mujoco")
        urdf = URDF_STRUCT.replace(
            '<link name="tool0"/>',
            '<link name="tool0"><inertial><mass value="0.5"/>'
            '<origin xyz="0 0 0" rpy="0 0 0"/>'
            '<inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>'
            "</inertial></link>",
        )
        n, out = self._run(tmp_path, capsys, urdf_text=urdf)
        assert "URDF links with mass absent from MJCF" in out
        assert "0.5 kg lost" in out
        assert n >= 1

    def test_materialised_massless_frame_still_counts_toward_body_count(self, tmp_path, capsys):
        """iiwa7 / leap_hand shape: MuJoCo DOES create bodies for their massless
        frames, so a blanket 'ignore massless links' rule would break them."""
        pytest.importorskip("mujoco")
        urdf = URDF_STRUCT.replace(
            "</robot>",
            '  <link name="ee_frame"/>\n'
            '  <joint name="ee_fixed" type="fixed">\n'
            '    <parent link="shoulder_link"/><child link="ee_frame"/>\n'
            '    <origin xyz="0 0 0.2" rpy="0 0 0"/>\n'
            "  </joint>\n</robot>",
        )
        mjcf = MJCF_STRUCT.replace(
            '<joint name="shoulder_pan_joint" axis="0 0 1" range="-3.14 3.14"/>',
            '<joint name="shoulder_pan_joint" axis="0 0 1" range="-3.14 3.14"/>\n'
            '        <body name="ee_frame" pos="0 0 0.2">\n'
            '          <inertial mass="0" pos="0 0 0" diaginertia="0 0 0"/>\n'
            "        </body>",
        )
        n, out = self._run(tmp_path, capsys, urdf_text=urdf, mjcf_text=mjcf)
        # 4 URDF links, tool0 excused, ee_frame materialised -> 3 expected bodies
        assert "body count: 3  OK" in out
        assert n == 0

    def test_name_collision_does_not_hide_a_massive_link(self, tmp_path, capsys):
        """ur5e shape: URDF has BOTH a massless `base` frame and the massive
        `base_link_inertia`; the MJCF's single `base` body is the massive one.
        Raw-name matching paired the massless frame with it and reported the
        real link as lost (#392)."""
        pytest.importorskip("mujoco")
        urdf = URDF_STRUCT.replace(
            '<link name="tool0"/>',
            '<link name="tool0"/>\n  <link name="base"/>\n'
            '  <joint name="base_fixed" type="fixed">\n'
            '    <parent link="base_link_inertia"/><child link="base"/>\n'
            '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
            "  </joint>",
        )
        n, out = self._run(tmp_path, capsys, urdf_text=urdf)
        # The massless `base` frame is what went missing — not the 4 kg link.
        assert "URDF links with mass absent" not in out
        assert "base, tool0" in out
        # And the massive link is actually compared, via link_map.
        assert "[base (URDF: base_link_inertia)]" in out
        assert "body count: 2  OK" in out
        assert n == 0


# ═══════════════════════════════════════════════════════════════════════════
# UNVERIFIED accounting (#392)
#
# Without mujoco the structural section checks nothing, yet the run used to end
# in "Mismatches: 0" and exit 0.  That is the false green this issue is about,
# and it is the path colcon actually takes (it runs pytest under
# /usr/bin/python3, which cannot see the venv where mujoco lives).
# ═══════════════════════════════════════════════════════════════════════════


class TestUnverifiedAccounting:
    def _run(self, tmp_path, capsys, **kw):
        mjcf = tmp_path / "s.xml"
        mjcf.write_text(MJCF_STRUCT)
        urdf = tmp_path / "s.urdf"
        urdf.write_text(URDF_STRUCT)
        n = compare(mjcf, urdf, link_map=STRUCT_LINK_MAP, joint_names=STRUCT_JOINTS, **kw)
        return n, capsys.readouterr().out

    def test_missing_mujoco_is_reported_as_unverified(self, tmp_path, capsys, monkeypatch):
        # None in sys.modules makes `import mujoco` raise ImportError.
        monkeypatch.setitem(sys.modules, "mujoco", None)
        n, out = self._run(tmp_path, capsys)
        assert "[UNVERIFIED]" in out
        assert "UNVERIFIED: 1" in out
        assert "NOT a clean pass" in out
        assert "ALL PARAMETERS MATCH" not in out
        assert n == 0  # default stays permissive for interactive use

    def test_fail_on_unverified_makes_it_count(self, tmp_path, capsys, monkeypatch):
        monkeypatch.setitem(sys.modules, "mujoco", None)
        n, out = self._run(tmp_path, capsys, fail_on_unverified=True)
        assert "treated as failure" in out
        assert n >= 1

    def test_clean_run_says_so(self, tmp_path, capsys):
        pytest.importorskip("mujoco")
        n, out = self._run(tmp_path, capsys, fail_on_unverified=True)
        assert "UNVERIFIED" not in out
        assert n == 0


# ═══════════════════════════════════════════════════════════════════════════
# Tool / tip frame comparison (#392)
#
# The joint section compares axis LINES, which cannot see an offset applied
# past the last joint: it slides along that joint's own axis and (measured)
# moves no link COM either.  Only the tool frame observes it.
# ═══════════════════════════════════════════════════════════════════════════

MJCF_TIP = MJCF_ALIGN.replace(
    '<joint name="j1" axis="0 0 1" range="-3.14 3.14"/>',
    '<joint name="j1" axis="0 0 1" range="-3.14 3.14"/>\n'
    '        <site name="tcp" pos="0 0 0.15"/>',
)

URDF_TIP = URDF_ALIGN.replace(
    "</robot>",
    '  <link name="tcp"/>\n'
    '  <joint name="tcp_fixed" type="fixed">\n'
    '    <parent link="link1"/><child link="tcp"/>\n'
    '    <origin xyz="0 0 0.15" rpy="0 0 0"/>\n'
    "  </joint>\n</robot>",
)


class TestTipFrame:
    def _run(self, tmp_path, capsys, urdf_text=URDF_TIP, tip=("tcp", "tcp")):
        mjcf = tmp_path / "t.xml"
        mjcf.write_text(MJCF_TIP)
        urdf = tmp_path / "t.urdf"
        urdf.write_text(urdf_text)
        align = _resolve_alignment(mjcf, urdf, "base", "base_link")
        n = compare(
            mjcf,
            urdf,
            link_map=ALIGN_LINK_MAP,
            joint_names=["j1"],
            align=align,
            tip_frames=tip,
        )
        return n, capsys.readouterr().out

    def test_site_frame_is_found_and_matches(self, tmp_path, capsys):
        pytest.importorskip("mujoco")
        n, out = self._run(tmp_path, capsys)
        assert "[tcp (URDF: tcp)]" in out
        assert "TIP POSITION MISMATCH" not in out
        assert n == 0

    def test_tail_offset_is_caught(self, tmp_path, capsys):
        """The d6 shape: an offset past the last joint, along its own axis.
        The joint section stays clean; only this check sees it."""
        urdf = URDF_TIP.replace('xyz="0 0 0.15" rpy="0 0 0"', 'xyz="0 0 0.1504" rpy="0 0 0"')
        n, out = self._run(tmp_path, capsys, urdf_text=urdf)
        assert "TIP POSITION MISMATCH" in out
        assert "AXIS-LINE MISMATCH" not in out  # invisible to the joint section
        assert n >= 1

    def test_orientation_difference_is_caught(self, tmp_path, capsys):
        urdf = URDF_TIP.replace('xyz="0 0 0.15" rpy="0 0 0"', 'xyz="0 0 0.15" rpy="0 0 0.5"')
        n, out = self._run(tmp_path, capsys, urdf_text=urdf)
        assert "TIP ORIENTATION MISMATCH" in out
        assert n >= 1

    def test_unknown_name_warns_rather_than_silently_passing(self, tmp_path, capsys):
        n, out = self._run(tmp_path, capsys, tip=("nope", "tcp"))
        assert "no body or site named 'nope'" in out
        assert n == 0


class TestMjcfNamedFrames:
    def test_sites_are_included(self, tmp_path):
        mjcf = tmp_path / "t.xml"
        mjcf.write_text(MJCF_TIP)
        frames = _mjcf_named_frames(mjcf)
        assert "tcp" in frames
        # base carries Rz(pi); link1 sits at local (0.3, 0, 0.2), site +0.15 z
        assert _vec_close_3(frames["tcp"][1], [-0.3, 0, 0.35], 1e-12)

    def test_bodies_still_present(self, tmp_path):
        mjcf = tmp_path / "t.xml"
        mjcf.write_text(MJCF_TIP)
        frames = _mjcf_named_frames(mjcf)
        assert "world" in frames and "base" in frames and "link1" in frames


# ═══════════════════════════════════════════════════════════════════════════
# link_map is an exception list, not a worklist (#411)
#
# A partial map used to *filter* the per-link comparison: bodies it did not
# name were parsed but never compared, and nothing in the report said so.  On
# ur5e+hand that hid four real mass mismatches behind "Mismatches: 3".  The
# fixture below reproduces the shape — one name-differing pair (declared) plus
# one same-name pair (undeclared) whose mass disagrees.
# ═══════════════════════════════════════════════════════════════════════════

URDF_PARTIAL = """\
<?xml version="1.0"?>
<robot name="partial_map">
  <link name="base_link_inertia">
    <inertial>
      <mass value="4.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="wrist_link">
    <inertial>
      <mass value="0.02"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="wrist_joint" type="revolute">
    <parent link="base_link_inertia"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>
</robot>
"""

# `wrist_link` carries 0.03 here against the URDF's 0.02 — the divergence the
# narrowed comparison used to swallow.
MJCF_PARTIAL = """\
<mujoco model="partial_map">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="base">
      <inertial mass="4.0" pos="0 0 0" diaginertia="0.01 0.01 0.01"/>
      <body name="wrist_link" pos="0 0 0.2">
        <inertial mass="0.03" pos="0 0 0" diaginertia="0.001 0.001 0.001"/>
        <joint name="wrist_joint" axis="0 0 1" range="-3.14 3.14"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general joint="wrist_joint" forcerange="-100 100"/>
  </actuator>
</mujoco>
"""

PARTIAL_MAP = {"base": "base_link_inertia"}


class TestResolveLinkMap:
    def _files(self, tmp_path):
        mjcf = tmp_path / "p.xml"
        mjcf.write_text(MJCF_PARTIAL)
        urdf = tmp_path / "p.urdf"
        urdf.write_text(URDF_PARTIAL)
        return mjcf, urdf

    def test_same_name_pair_outside_the_map_is_added(self, tmp_path):
        mjcf, urdf = self._files(tmp_path)
        effective, _, _ = _resolve_link_map(mjcf, urdf, PARTIAL_MAP)
        assert effective == {"base": "base_link_inertia", "wrist_link": "wrist_link"}

    def test_empty_map_falls_back_to_same_name_pairs(self, tmp_path):
        mjcf, urdf = self._files(tmp_path)
        for explicit in ({}, None):
            effective, _, _ = _resolve_link_map(mjcf, urdf, explicit)
            assert effective == {"wrist_link": "wrist_link"}

    def test_urdf_link_an_explicit_entry_targets_is_not_re_paired(self, tmp_path):
        """ur5e's shape.  `base: wrist_link` claims the URDF `wrist_link`, so
        the same-named MJCF body must not grab it back — otherwise the
        declaration is silently undone and one link is compared twice."""
        mjcf, urdf = self._files(tmp_path)
        effective, _, _ = _resolve_link_map(mjcf, urdf, {"base": "wrist_link"})
        assert effective == {"base": "wrist_link"}

    def test_explicit_entry_overrides_the_same_name_pair_for_its_body(self, tmp_path):
        """An MJCF body named in the exception list is compared against the
        link that entry names, not against its own name."""
        mjcf, urdf = self._files(tmp_path)
        effective, _, _ = _resolve_link_map(mjcf, urdf, {"wrist_link": "base_link_inertia"})
        assert effective == {"wrist_link": "base_link_inertia"}

    def test_unpaired_names_are_reported(self, tmp_path):
        mjcf, urdf = self._files(tmp_path)
        _, unpaired_mjcf, unpaired_urdf = _resolve_link_map(mjcf, urdf, None)
        assert unpaired_mjcf == ["base"]
        assert unpaired_urdf == ["base_link_inertia"]

    def test_declared_pair_leaves_nothing_unpaired(self, tmp_path):
        mjcf, urdf = self._files(tmp_path)
        _, unpaired_mjcf, unpaired_urdf = _resolve_link_map(mjcf, urdf, PARTIAL_MAP)
        assert unpaired_mjcf == [] and unpaired_urdf == []


class TestPartialLinkMapDoesNotNarrow:
    def _run(self, tmp_path, capsys, link_map=PARTIAL_MAP):
        mjcf = tmp_path / "p.xml"
        mjcf.write_text(MJCF_PARTIAL)
        urdf = tmp_path / "p.urdf"
        urdf.write_text(URDF_PARTIAL)
        n = compare(mjcf, urdf, link_map=link_map, joint_names=["wrist_joint"])
        return n, capsys.readouterr().out

    def test_mismatch_outside_the_map_is_still_reported(self, tmp_path, capsys):
        n, out = self._run(tmp_path, capsys)
        assert "MASS MISMATCH" in out
        assert "MJCF=0.03" in out and "URDF=0.02" in out
        assert n >= 1

    def test_coverage_is_stated(self, tmp_path, capsys):
        _, out = self._run(tmp_path, capsys)
        assert "Link pairs compared: 2" in out

    def test_unpaired_links_are_listed(self, tmp_path, capsys):
        """With no map at all, `base` / `base_link_inertia` go unpaired — the
        report must say so instead of quietly comparing one link."""
        _, out = self._run(tmp_path, capsys, link_map={})
        assert "Link pairs compared: 1" in out
        assert "not compared per-link — MJCF bodies (1): base" in out
        assert "not compared per-link — URDF links (1): base_link_inertia" in out

    def test_declared_pair_is_compared_under_its_own_name(self, tmp_path, capsys):
        _, out = self._run(tmp_path, capsys)
        assert "[base (URDF: base_link_inertia)]" in out
        assert "not compared per-link" not in out


# ═══════════════════════════════════════════════════════════════════════════
# Fused links — MJCF folded a fixed-joint child into its parent (#412)
#
# Folding is legitimate modelling, but the tool used to report it as two
# defects at once (child's mass "lost" + parent overweight), so such a model
# could not be gated at all.  A `fuse:` declaration composes the children in
# before comparison — mass, COM and inertia all against the combined body, so
# declaring cannot be used to make a real mismatch disappear.
#
# The numbers mirror the real assm_v1 hand (verified by hand against its MJCF)
# scaled up so the inertia terms clear the 1e-4 default tolerance:
#   parent  m=2  com_z=0.125   I=diag(0.01,  0.01,  0.001)
#   tip     m=1  com_z=0.125   I=diag(0.001, 0.001, 0.0005)   fixed @ z=0.25
#   fused   m=3  com_z=(2·0.125 + 1·0.375)/3 = 0.2083333
#           Ixx = [0.01 + 2·0.0833333²] + [0.001 + 1·0.1666667²] = 0.0526667
#           Izz = 0.001 + 0.0005 = 0.0015          (displacement is along z)
# ═══════════════════════════════════════════════════════════════════════════

URDF_FUSE = """\
<?xml version="1.0"?>
<robot name="fuse_test">
  <link name="base_link_inertia">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="distal_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="tip_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.0005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="distal_joint" type="revolute">
    <parent link="base_link_inertia"/>
    <child link="distal_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="tip_joint" type="fixed">
    <parent link="distal_link"/>
    <child link="tip_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
</robot>
"""

MJCF_FUSE = """\
<mujoco model="fuse_test">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="base">
      <inertial mass="1.0" pos="0 0 0" diaginertia="0.001 0.001 0.001"/>
      <body name="distal_link" pos="0 0 0.1">
        <inertial mass="3.0" pos="0 0 0.2083333333"
                  diaginertia="0.0526666667 0.0526666667 0.0015"/>
        <joint name="distal_joint" axis="0 0 1" range="-3.14 3.14"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general joint="distal_joint" forcerange="-10 10"/>
  </actuator>
</mujoco>
"""

FUSE_LINK_MAP = {"base": "base_link_inertia"}
FUSE_DECL = {"distal_link": ["tip_link"]}
FUSE_JOINTS = ["distal_joint"]


class TestFuseUrdfInertials:
    """The composition itself, against hand-computed values."""

    def _params(self, tmp_path, urdf_text=URDF_FUSE):
        urdf = tmp_path / "f.urdf"
        urdf.write_text(urdf_text)
        links, _ = parse_urdf(urdf, {"distal_link", "tip_link"}, set())
        return _fuse_urdf_inertials(
            "distal_link", ["tip_link"], links, _urdf_link_world_frames(urdf)
        )

    def test_mass_is_summed(self, tmp_path):
        assert _close(self._params(tmp_path).mass, 3.0, 1e-12)

    def test_com_is_mass_weighted(self, tmp_path):
        com = self._params(tmp_path).origin_xyz
        assert _vec_close_3(com, [0.0, 0.0, 0.2083333333], 1e-9)

    def test_inertia_uses_the_parallel_axis_theorem(self, tmp_path):
        p = self._params(tmp_path)
        assert _close(p.diag_inertia[0], 0.0526666667, 1e-9)
        assert _close(p.diag_inertia[1], 0.0526666667, 1e-9)
        # No parallel-axis term along the displacement direction.
        assert _close(p.diag_inertia[2], 0.0015, 1e-12)

    def test_summing_inertia_without_the_shift_would_be_wrong(self, tmp_path):
        """Guards the shift specifically: a naive tensor sum gives 0.011."""
        assert not _close(self._params(tmp_path).diag_inertia[0], 0.011, 1e-6)

    #: URDF_FUSE with Rx(pi/2) on the fixed joint.
    ROTATED = (
        '<origin xyz="0 0 0.25" rpy="0 0 0"/>',
        '<origin xyz="0 0 0.25" rpy="1.5707963268 0 0"/>',
    )

    def test_fixed_joint_rotation_moves_the_composed_com(self, tmp_path):
        """Rx(pi/2) swings the tip onto -y.  With the rotation dropped both
        links stay on z and the composed COM has no y component at all."""
        com = self._params(tmp_path, urdf_text=URDF_FUSE.replace(*self.ROTATED)).origin_xyz
        # tip COM lands at (0, -0.125, 0.25); (2·0 + 1·-0.125)/3 = -0.0416667
        assert _vec_close_3(com, [0.0, -0.0416666667, 0.1666666667], 1e-9)

    def test_fixed_joint_rotation_reorients_the_child_tensor(self, tmp_path):
        """The child's inertia must be rotated into the shared frame before it
        is summed, not just its COM.

        The trace cannot see this — ``trace(R·I·Rᵀ) == trace(I)`` and the
        parallel-axis additions are identical either way — so the individual
        components are the only witness.  Values below were derived by hand
        (Rx(pi/2) maps the tip's diag(0.001, 0.001, 0.0005) to
        diag(0.001, 0.0005, 0.001)) and agree with the implementation to 1e-10.
        """
        p = self._params(tmp_path, urdf_text=URDF_FUSE.replace(*self.ROTATED))
        assert _close(p.diag_inertia[0], 0.0318333333, 1e-9)
        assert _close(p.diag_inertia[1], 0.0209166667, 1e-9)
        assert _close(p.diag_inertia[2], 0.0124166667, 1e-9)
        assert _close(p.off_diag_inertia[2], 0.0104166667, 1e-9)  # Iyz
        # Skipping the child's rotation would give yy=0.0214166667,
        # zz=0.0119166667 — same trace, wrong distribution.
        assert not _close(p.diag_inertia[1], 0.0214166667, 1e-9)


class TestValidateFuse:
    """A declaration must not become a way to switch checking off."""

    def _urdf(self, tmp_path, text=URDF_FUSE):
        urdf = tmp_path / "v.urdf"
        urdf.write_text(text)
        return urdf

    def test_valid_declaration_has_no_errors(self, tmp_path):
        assert (
            _validate_fuse(self._urdf(tmp_path), FUSE_DECL, {"distal_link": "distal_link"}) == []
        )

    def test_movable_child_is_rejected(self, tmp_path):
        """The whole point: folding is only legitimate across fixed joints."""
        text = URDF_FUSE.replace(
            '<joint name="tip_joint" type="fixed">',
            '<joint name="tip_joint" type="revolute">',
        )
        errors = _validate_fuse(
            self._urdf(tmp_path, text), FUSE_DECL, {"distal_link": "distal_link"}
        )
        assert len(errors) == 1
        assert "only fixed joints may be folded" in errors[0]

    def test_unknown_child_is_rejected(self, tmp_path):
        errors = _validate_fuse(
            self._urdf(tmp_path), {"distal_link": ["nope"]}, {"distal_link": "distal_link"}
        )
        assert len(errors) == 1 and "does not exist" in errors[0]

    def test_unmapped_parent_is_rejected(self, tmp_path):
        errors = _validate_fuse(self._urdf(tmp_path), FUSE_DECL, {})
        assert len(errors) == 1 and "not a mapped MJCF body" in errors[0]

    def test_non_descendant_is_rejected(self, tmp_path):
        """Folding runs child → parent.  Naming an *ancestor* as the folded
        link walks off the top of the tree and must not be accepted."""
        errors = _validate_fuse(
            self._urdf(tmp_path),
            {"distal_link": ["base_link_inertia"]},
            {"distal_link": "distal_link"},
        )
        assert len(errors) == 1 and "not a descendant" in errors[0]


class TestCompareWithFusedLinks:
    def _run(self, tmp_path, capsys, *, mjcf_text=MJCF_FUSE, fuse=FUSE_DECL):
        pytest.importorskip("mujoco")
        mjcf = tmp_path / "f.xml"
        mjcf.write_text(mjcf_text)
        urdf = tmp_path / "f.urdf"
        urdf.write_text(URDF_FUSE)
        n = compare(mjcf, urdf, link_map=FUSE_LINK_MAP, joint_names=FUSE_JOINTS, fuse=fuse)
        return n, capsys.readouterr().out

    def test_declared_model_is_clean(self, tmp_path, capsys):
        n, out = self._run(tmp_path, capsys)
        assert "[FUSED]" in out
        assert "kg lost" not in out
        assert n == 0

    def test_without_the_declaration_the_same_model_fails(self, tmp_path, capsys):
        """Otherwise `fuse` could be a no-op and nothing would reveal it."""
        n, out = self._run(tmp_path, capsys, fuse=None)
        assert "tip_link  mass=1 kg lost" in out
        assert "MASS MISMATCH" in out
        assert n >= 2

    def test_wrong_fused_mass_is_still_caught(self, tmp_path, capsys):
        """Declaring must not waive the check — only redirect it at the
        combined body."""
        mjcf_text = MJCF_FUSE.replace('mass="3.0"', 'mass="2.5"')
        n, out = self._run(tmp_path, capsys, mjcf_text=mjcf_text)
        assert "MASS MISMATCH" in out
        assert n >= 1

    def test_wrong_fused_inertia_is_still_caught(self, tmp_path, capsys):
        """The naive tensor sum (no parallel-axis shift) must not pass."""
        mjcf_text = MJCF_FUSE.replace(
            'diaginertia="0.0526666667 0.0526666667 0.0015"',
            'diaginertia="0.011 0.011 0.0015"',
        )
        n, out = self._run(tmp_path, capsys, mjcf_text=mjcf_text)
        assert "INERTIA MISMATCH" in out
        assert n >= 1

    def test_wrong_fused_com_is_still_caught(self, tmp_path, capsys):
        mjcf_text = MJCF_FUSE.replace('pos="0 0 0.2083333333"', 'pos="0 0 0.19"')
        n, out = self._run(tmp_path, capsys, mjcf_text=mjcf_text)
        assert "COM MISMATCH (world)" in out
        assert n >= 1

    def test_invalid_declaration_is_a_mismatch(self, tmp_path, capsys):
        n, out = self._run(tmp_path, capsys, fuse={"distal_link": ["nope"]})
        assert "does not exist" in out
        assert n >= 1


class TestLoadLinkMapSchema:
    def _write(self, tmp_path, text):
        path = tmp_path / "m.yaml"
        path.write_text(text)
        return path

    def test_flat_form_still_loads(self, tmp_path):
        links, fuse = _load_link_map(self._write(tmp_path, "base: base_link_inertia\n"))
        assert links == {"base": "base_link_inertia"} and fuse == {}

    def test_structured_form(self, tmp_path):
        text = "links:\n  base: base_link_inertia\nfuse:\n  distal_link: [tip_link]\n"
        links, fuse = _load_link_map(self._write(tmp_path, text))
        assert links == {"base": "base_link_inertia"}
        assert fuse == {"distal_link": ["tip_link"]}

    def test_fuse_accepts_a_bare_name(self, tmp_path):
        _, fuse = _load_link_map(self._write(tmp_path, "fuse:\n  distal_link: tip_link\n"))
        assert fuse == {"distal_link": ["tip_link"]}

    def test_unknown_top_level_key_is_rejected(self, tmp_path):
        text = "links:\n  a: b\nfused:\n  c: [d]\n"
        with pytest.raises(SystemExit, match="unknown top-level key"):
            _load_link_map(self._write(tmp_path, text))

    def test_a_link_named_links_is_not_mistaken_for_the_schema(self, tmp_path):
        """The discriminator is the value type, so a robot may own such a link."""
        links, fuse = _load_link_map(self._write(tmp_path, "links: links\nfuse: fuse\n"))
        assert links == {"links": "links", "fuse": "fuse"} and fuse == {}
