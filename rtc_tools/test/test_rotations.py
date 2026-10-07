"""rtc_tools.utils.rotations — the one roll-pitch-yaw convention."""

import math

import numpy as np
import pytest

from rtc_tools.utils.rotations import rpy_matrix


def _about(axis: int, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    out = np.eye(3)
    out[i, i] = out[j, j] = c
    out[j, i], out[i, j] = s, -s
    return out


def test_each_angle_turns_about_its_own_axis():
    assert rpy_matrix((0.0, 0.0, 0.0)) == pytest.approx(np.eye(3))
    # Roll a quarter turn takes +y to +z; yaw a quarter turn takes +x to +y.
    assert rpy_matrix((math.pi / 2, 0.0, 0.0)) @ [0, 1, 0] == pytest.approx([0, 0, 1])
    assert rpy_matrix((0.0, 0.0, math.pi / 2)) @ [1, 0, 0] == pytest.approx([0, 1, 0])
    # Pitch a quarter turn takes +x to −z.
    assert rpy_matrix((0.0, math.pi / 2, 0.0)) @ [1, 0, 0] == pytest.approx([0, 0, -1])


def test_the_order_is_yaw_then_pitch_then_roll():
    rpy = (0.3, -0.7, 1.1)
    expected = _about(2, rpy[2]) @ _about(1, rpy[1]) @ _about(0, rpy[0])
    assert rpy_matrix(rpy) == pytest.approx(expected)
    assert rpy_matrix(rpy) != pytest.approx(
        _about(0, rpy[0]) @ _about(1, rpy[1]) @ _about(2, rpy[2])
    )


def test_the_matrix_is_a_rotation():
    rng = np.random.default_rng(7)
    for rpy in rng.uniform(-math.pi, math.pi, size=(20, 3)):
        rot = rpy_matrix(rpy)
        assert rot.T @ rot == pytest.approx(np.eye(3))
        assert np.linalg.det(rot) == pytest.approx(1.0)
    assert rpy_matrix(np.array([0.1, 0.2, 0.3])).shape == (3, 3)  # any sequence of three
