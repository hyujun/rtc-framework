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
    _axis_line_offset,
    _close,
    _compose_transform,
    _compute_mjcf_world_frames,
    _compute_urdf_world_frames,
    _detect_joint_types,
    _fmt,
    _identity_3x3,
    _invert_transform,
    _mjcf_body_rotation,
    _mjcf_body_world_frames,
    _parse_floats,
    _quat_to_rot3,
    _resolve_alignment,
    _rpy_to_rot3,
    _urdf_link_world_frames,
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

    def _files(self, tmp_path, urdf_origin, *, prismatic=False):
        mjcf_text = MJCF_TEMPLATE
        urdf_text = URDF_TEMPLATE.replace('xyz="0 0 0.1625"', f'xyz="{urdf_origin}"')
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
