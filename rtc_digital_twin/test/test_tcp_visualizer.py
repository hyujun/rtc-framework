"""Tests for TcpVisualizer — TCP pose → MarkerArray / TF conversion.

Pure math + message building: rotation conversions (quat↔rpy↔matrix) and marker
builders. No ROS node or spin; stamps use builtin_interfaces/Time and tf2 inputs
use a real geometry_msgs/TransformStamped.
"""

import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped

from rtc_digital_twin.tcp_visualizer import TcpVisualizer

SQRT_HALF = math.sqrt(0.5)  # ≈ 0.70710678


def _stamp():
    return Time(sec=0, nanosec=0)


def _make_tf(x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    tf = TransformStamped()
    tf.transform.translation.x = float(x)
    tf.transform.translation.y = float(y)
    tf.transform.translation.z = float(z)
    tf.transform.rotation.w = float(qw)
    tf.transform.rotation.x = float(qx)
    tf.transform.rotation.y = float(qy)
    tf.transform.rotation.z = float(qz)
    return tf


# ── Rotation math ─────────────────────────────────────────────────────────────


class TestQuatToRpy:
    def test_identity(self):
        roll, pitch, yaw = TcpVisualizer._quat_to_rpy(1.0, 0.0, 0.0, 0.0)
        assert (roll, pitch, yaw) == pytest.approx((0.0, 0.0, 0.0))

    def test_yaw_90(self):
        # quat for yaw=+90°: w=cos(45°), z=sin(45°)
        roll, pitch, yaw = TcpVisualizer._quat_to_rpy(SQRT_HALF, 0.0, 0.0, SQRT_HALF)
        assert yaw == pytest.approx(math.pi / 2.0)
        assert roll == pytest.approx(0.0)
        assert pitch == pytest.approx(0.0)

    def test_gimbal_lock_positive(self):
        # pitch=+90° → sinp == 1.0 → copysign branch
        roll, pitch, yaw = TcpVisualizer._quat_to_rpy(SQRT_HALF, 0.0, SQRT_HALF, 0.0)
        assert pitch == pytest.approx(math.pi / 2.0)

    def test_gimbal_lock_negative(self):
        # pitch=-90° → sinp == -1.0 → copysign(-pi/2)
        roll, pitch, yaw = TcpVisualizer._quat_to_rpy(SQRT_HALF, 0.0, -SQRT_HALF, 0.0)
        assert pitch == pytest.approx(-math.pi / 2.0)


class TestRpyToQuaternion:
    def test_identity(self):
        qx, qy, qz, qw = TcpVisualizer._rpy_to_quaternion(0.0, 0.0, 0.0)
        assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, 0.0, 1.0))

    def test_roll_90(self):
        qx, qy, qz, qw = TcpVisualizer._rpy_to_quaternion(math.pi / 2.0, 0.0, 0.0)
        assert (qx, qy, qz, qw) == pytest.approx((SQRT_HALF, 0.0, 0.0, SQRT_HALF))

    def test_roundtrip_through_quat_to_rpy(self):
        r0, p0, y0 = 0.1, -0.2, 0.3
        qx, qy, qz, qw = TcpVisualizer._rpy_to_quaternion(r0, p0, y0)
        roll, pitch, yaw = TcpVisualizer._quat_to_rpy(qw, qx, qy, qz)
        assert (roll, pitch, yaw) == pytest.approx((r0, p0, y0))


class TestRpyToMatrix:
    def test_identity(self):
        m = TcpVisualizer._rpy_to_matrix(0.0, 0.0, 0.0)
        assert m[0] == pytest.approx([1.0, 0.0, 0.0])
        assert m[1] == pytest.approx([0.0, 1.0, 0.0])
        assert m[2] == pytest.approx([0.0, 0.0, 1.0])

    def test_roll_90_about_x(self):
        m = TcpVisualizer._rpy_to_matrix(math.pi / 2.0, 0.0, 0.0)
        assert m[0] == pytest.approx([1.0, 0.0, 0.0])
        assert m[1] == pytest.approx([0.0, 0.0, -1.0])
        assert m[2] == pytest.approx([0.0, 1.0, 0.0])

    def test_columns_are_unit_vectors(self):
        m = TcpVisualizer._rpy_to_matrix(0.3, -0.4, 0.5)
        for col in range(3):
            norm = math.sqrt(sum(m[row][col] ** 2 for row in range(3)))
            assert norm == pytest.approx(1.0)


# ── create_markers ────────────────────────────────────────────────────────────


class TestCreateMarkers:
    def test_none_yields_empty(self):
        viz = TcpVisualizer({})
        assert viz.create_markers(None, _stamp()).markers == []

    def test_too_short_yields_empty(self):
        viz = TcpVisualizer({})
        assert viz.create_markers([0.1, 0.2], _stamp()).markers == []

    def test_all_zero_yields_empty(self):
        viz = TcpVisualizer({})
        assert viz.create_markers([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], _stamp()).markers == []

    def test_pose_yields_sphere_plus_axes(self):
        viz = TcpVisualizer({})
        out = viz.create_markers([0.1, 0.2, 0.3, 0.0, 0.0, 0.0], _stamp())
        assert len(out.markers) == 4
        assert {m.ns for m in out.markers} == {
            "tcp_sphere",
            "tcp_axis_0",
            "tcp_axis_1",
            "tcp_axis_2",
        }

    def test_position_only_defaults_orientation(self):
        # exactly 3 elements → roll/pitch/yaw default to 0, still 4 markers
        viz = TcpVisualizer({})
        out = viz.create_markers([0.1, 0.0, 0.0], _stamp())
        assert len(out.markers) == 4

    def test_goal_sphere_added(self):
        viz = TcpVisualizer({"show_goal": True})
        out = viz.create_markers([0.1, 0.2, 0.3], _stamp(), goal_positions=[0.5, 0.5, 0.5])
        assert len(out.markers) == 5
        assert out.markers[-1].ns == "tcp_goal"

    def test_zero_goal_skipped(self):
        viz = TcpVisualizer({"show_goal": True})
        out = viz.create_markers([0.1, 0.2, 0.3], _stamp(), goal_positions=[0.0, 0.0, 0.0])
        assert len(out.markers) == 4

    def test_show_goal_false_skips_goal(self):
        viz = TcpVisualizer({"show_goal": False})
        out = viz.create_markers([0.1, 0.2, 0.3], _stamp(), goal_positions=[0.5, 0.5, 0.5])
        assert len(out.markers) == 4

    def test_sphere_position_matches_pose(self):
        viz = TcpVisualizer({})
        out = viz.create_markers([0.1, 0.2, 0.3, 0.0, 0.0, 0.0], _stamp())
        sphere = next(m for m in out.markers if m.ns == "tcp_sphere")
        assert (
            sphere.pose.position.x,
            sphere.pose.position.y,
            sphere.pose.position.z,
        ) == pytest.approx((0.1, 0.2, 0.3))


# ── create_tf ─────────────────────────────────────────────────────────────────


class TestCreateTf:
    def test_none_yields_none(self):
        assert TcpVisualizer({}).create_tf(None, _stamp(), "virtual_tcp") is None

    def test_too_short_yields_none(self):
        assert TcpVisualizer({}).create_tf([0.1], _stamp(), "virtual_tcp") is None

    def test_all_zero_yields_none(self):
        assert TcpVisualizer({}).create_tf([0.0, 0.0, 0.0], _stamp(), "virtual_tcp") is None

    def test_valid_pose(self):
        viz = TcpVisualizer({"frame_id": "base"})
        tf = viz.create_tf([0.1, 0.2, 0.3, 0.0, 0.0, 0.0], _stamp(), "virtual_tcp")
        assert tf is not None
        assert tf.header.frame_id == "base"
        assert tf.child_frame_id == "virtual_tcp"
        assert (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ) == pytest.approx((0.1, 0.2, 0.3))
        # identity orientation
        assert tf.transform.rotation.w == pytest.approx(1.0)

    def test_orientation_from_rpy(self):
        viz = TcpVisualizer({})
        tf = viz.create_tf([0.1, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0], _stamp(), "c")
        rot = tf.transform.rotation
        assert (rot.x, rot.y, rot.z, rot.w) == pytest.approx((SQRT_HALF, 0.0, 0.0, SQRT_HALF))


# ── Phase 4 tf2 entry points ──────────────────────────────────────────────────


class TestFromTf:
    def test_markers_from_tf_delegates(self):
        viz = TcpVisualizer({})
        tf = _make_tf(0.1, 0.2, 0.3)  # identity rotation
        out = viz.create_markers_from_tf(tf, _stamp())
        assert len(out.markers) == 4
        sphere = next(m for m in out.markers if m.ns == "tcp_sphere")
        assert (
            sphere.pose.position.x,
            sphere.pose.position.y,
            sphere.pose.position.z,
        ) == pytest.approx((0.1, 0.2, 0.3))

    def test_markers_from_tf_zero_pose_empty(self):
        viz = TcpVisualizer({})
        out = viz.create_markers_from_tf(_make_tf(0.0, 0.0, 0.0), _stamp())
        assert out.markers == []

    def test_markers_from_tf_passes_goal(self):
        viz = TcpVisualizer({"show_goal": True})
        out = viz.create_markers_from_tf(
            _make_tf(0.1, 0.2, 0.3), _stamp(), goal_positions=[0.5, 0.5, 0.5]
        )
        assert any(m.ns == "tcp_goal" for m in out.markers)

    def test_create_tf_from_tf_copies_transform(self):
        viz = TcpVisualizer({"frame_id": "world"})
        src = _make_tf(0.1, 0.2, 0.3, qw=SQRT_HALF, qz=SQRT_HALF)
        out = viz.create_tf_from_tf(src, _stamp(), "virtual_tcp")
        assert out.header.frame_id == "world"
        assert out.child_frame_id == "virtual_tcp"
        assert out.transform.translation.x == pytest.approx(0.1)
        assert out.transform.rotation.z == pytest.approx(SQRT_HALF)


# ── Config defaults ───────────────────────────────────────────────────────────


class TestConfig:
    def test_defaults(self):
        viz = TcpVisualizer({})
        assert viz.frame_id == "base"
        assert viz.show_goal is True
        assert viz.sphere_radius == pytest.approx(0.012)

    def test_overrides(self):
        viz = TcpVisualizer({"frame_id": "odom", "sphere_radius": 0.05, "show_goal": False})
        assert viz.frame_id == "odom"
        assert viz.sphere_radius == pytest.approx(0.05)
        assert viz.show_goal is False
