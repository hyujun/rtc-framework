"""catchability_map (dynamic_catching S3.5a) — the pure numeric core.

The oracles are independent of the module under test:

* drag-free flight against the closed-form parabola, plus a Richardson step
  refinement that pins the integrator's ORDER (a first-order or second-order
  mistake converges at the wrong rate even where it still looks plausible),
* drag-on flight against physically necessary inequalities (shorter range,
  speed falling through the ascent, terminal velocity sqrt(g/k) on a long drop),
* frame conversion against arithmetic written out longhand in the test, on a
  point and a transform with no symmetry left to hide a transpose or a
  180-degree error behind.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from rtc_tools.analysis import catchability_map as cm

# The shipped tennis preset, with the file:line each value lives at. Mirrored
# here on purpose: the point of the test is that the module makes the caller
# supply them, so the test IS a caller.
CD_TENNIS = 0.55  # rtc_mujoco_sim/src/projectile_ball.cpp:30 (kTennisPhysics)
CD_SOURCE = "rtc_mujoco_sim/src/projectile_ball.cpp:30"
RHO_AIR = 1.204  # rtc_mujoco_sim/include/rtc_mujoco_sim/projectile_ball.hpp:95
RHO_SOURCE = "rtc_mujoco_sim/include/rtc_mujoco_sim/projectile_ball.hpp:95"
RADIUS_M = 0.0335
MASS_KG = 0.057

G = 9.81


def params(*, drag_coefficient: float = CD_TENNIS, air_density: float = RHO_AIR) -> cm.BallParams:
    return cm.BallParams(
        radius_m=RADIUS_M,
        mass_kg=MASS_KG,
        drag_coefficient=drag_coefficient,
        air_density_kg_m3=air_density,
        sources={
            "radius_m": "test:projectile_ball.radius_m",
            "mass_kg": "test:projectile_ball.mass_kg",
            "drag_coefficient": CD_SOURCE,
            "air_density_kg_m3": RHO_SOURCE,
        },
    )


def drag_free() -> cm.BallParams:
    return params(drag_coefficient=0.0)


def launch_velocity(speed: float, elevation_deg: float, azimuth_deg: float = 0.0) -> np.ndarray:
    el = math.radians(elevation_deg)
    az = math.radians(azimuth_deg)
    return np.array(
        [
            speed * math.cos(el) * math.cos(az),
            speed * math.cos(el) * math.sin(az),
            speed * math.sin(el),
        ]
    )


# ── Ball parameters and provenance ────────────────────────────────────────────


def test_ball_params_demands_a_source_label_per_quantity():
    with pytest.raises(ValueError, match="provenance label"):
        cm.BallParams(
            radius_m=RADIUS_M,
            mass_kg=MASS_KG,
            drag_coefficient=CD_TENNIS,
            air_density_kg_m3=RHO_AIR,
            sources={"radius_m": "x", "mass_kg": "y"},
        )


def test_ball_params_rejects_degenerate_shape_and_negative_drag():
    labels = dict.fromkeys(cm._AERO_KEYS, "test")
    with pytest.raises(ValueError, match="mass_kg"):
        cm.BallParams(RADIUS_M, 0.0, CD_TENNIS, RHO_AIR, labels)
    with pytest.raises(ValueError, match="drag_coefficient"):
        cm.BallParams(RADIUS_M, MASS_KG, -0.1, RHO_AIR, labels)
    # Zero is allowed: it is `projectile_ball.aerodynamics: false`.
    assert cm.BallParams(RADIUS_M, MASS_KG, 0.0, RHO_AIR, labels).drag_k_per_m == 0.0


def test_derived_k_matches_the_closed_form_and_differs_from_the_docs_value():
    p = params()
    area = math.pi * RADIUS_M**2
    expected = RHO_AIR * CD_TENNIS * area / (2.0 * MASS_KG)
    assert p.drag_k_per_m == pytest.approx(expected, rel=1e-15)
    # L0 §7 says the documented scalar and the preset Cd are not convertible
    # into one another; this pins that they indeed do not agree numerically, so
    # nobody "fixes" the model by substituting one for the other.
    assert p.drag_k_per_m == pytest.approx(0.020480, abs=1e-6)
    assert cm.DOCS_REPRESENTATIVE_DRAG_K_PER_M / p.drag_k_per_m == pytest.approx(1.118, abs=1e-3)


def test_ball_shape_from_config_reads_the_yaml_and_names_the_file(tmp_path: Path):
    base = tmp_path / "base.yaml"
    override = tmp_path / "robot.yaml"
    base.write_text(
        yaml.safe_dump(
            {"sim": {"ros__parameters": {"projectile_ball": {"radius_m": 0.025, "mass_kg": 0.05}}}}
        )
    )
    override.write_text(
        yaml.safe_dump({"sim": {"ros__parameters": {"projectile_ball": {"radius_m": RADIUS_M}}}})
    )
    shape = cm.ball_shape_from_config([base, override])
    assert shape.radius_m == RADIUS_M
    assert shape.mass_kg == 0.05
    # The override is visible: radius came from the second file, mass from the first.
    assert shape.sources["radius_m"] == f"{override}:projectile_ball.radius_m"
    assert shape.sources["mass_kg"] == f"{base}:projectile_ball.mass_kg"


def test_provenance_records_source_labels_and_the_derived_scalar(tmp_path: Path):
    config = tmp_path / "sim.yaml"
    config.write_text(
        yaml.safe_dump(
            {
                "sim": {
                    "ros__parameters": {
                        "projectile_ball": {"radius_m": RADIUS_M, "mass_kg": MASS_KG}
                    }
                }
            }
        )
    )
    shape = cm.ball_shape_from_config([config])
    p = cm.ball_params_from_shape(
        shape,
        drag_coefficient=CD_TENNIS,
        drag_coefficient_source=CD_SOURCE,
        air_density_kg_m3=RHO_AIR,
        air_density_source=RHO_SOURCE,
    )
    grid = cm.generate_throw_grid(speeds_m_s=(6.0,), elevations_deg=(20.0,))
    report = cm.build_provenance(
        p, configs=[config], grid=grid, integration={"step_s": 0.002, "horizon_s": 1.5}
    )

    labels = report["ball"]["sources"]
    assert set(labels) == set(cm._AERO_KEYS)
    assert labels["drag_coefficient"] == CD_SOURCE
    assert labels["air_density_kg_m3"] == RHO_SOURCE
    assert labels["radius_m"] == f"{config}:projectile_ball.radius_m"
    assert report["derived_drag_k_per_m"] == pytest.approx(p.drag_k_per_m, rel=1e-15)
    assert "REFERENCE ONLY" in report["derived_drag_k_note"]
    assert report["docs_representative_drag_k_per_m"] == cm.DOCS_REPRESENTATIVE_DRAG_K_PER_M
    assert report["grid"]["throws"] == len(grid)
    assert report["integration"]["integrator"] == "fixed-step RK4"
    assert str(config) in report["robot_config_sha256"]


# ── Ball flight: drag-free closed form and integrator order ───────────────────


def test_drag_free_flight_matches_the_closed_form_parabola():
    p0 = np.array([0.3, -1.1, 1.7])
    v0 = launch_velocity(8.4, 27.0, 41.0)
    traj = cm.integrate_flight(p0, v0, drag_free(), horizon_s=1.2, step_s=0.01)
    g = np.asarray(cm.GRAVITY_W_M_S2)

    t = traj.time_s[:, None]
    expected_p = p0 + v0 * t + 0.5 * g * t**2
    expected_v = v0 + g * t
    assert np.allclose(traj.position_m, expected_p, atol=1e-12, rtol=0.0)
    assert np.allclose(traj.velocity_m_s, expected_v, atol=1e-12, rtol=0.0)


def test_integrator_is_fourth_order_under_drag():
    # Drag-free flight is exact for RK4 (and for RK2), so the order can only be
    # measured on the nonlinear term. Reference = the same integrator at h/64,
    # whose own error is (1/64)^4 ~ 6e-8 of the coarse one.
    p0 = np.array([0.0, 0.0, 1.5])
    v0 = launch_velocity(9.0, 25.0)
    p = params()
    horizon = 0.96

    def endpoint(step: float) -> np.ndarray:
        return cm.integrate_flight(p0, v0, p, horizon_s=horizon, step_s=step).position_m[-1]

    coarse = horizon / 6.0
    reference = endpoint(coarse / 64.0)
    err_coarse = float(np.linalg.norm(endpoint(coarse) - reference))
    err_fine = float(np.linalg.norm(endpoint(coarse / 2.0) - reference))

    assert err_coarse > 1e-9, "error too small to measure a rate — shorten the reference step"
    ratio = err_coarse / err_fine
    # 16 for RK4; 2 for Euler, 4 for RK2, 8 for a 3rd-order scheme.
    assert 12.0 < ratio < 20.0, f"convergence rate {ratio:.2f} is not fourth order"


def test_drag_shortens_the_range_and_slows_the_ascent():
    p0 = np.array([0.0, 0.0, 1.5])
    v0 = launch_velocity(9.0, 30.0)
    kwargs = {"horizon_s": 1.4, "step_s": 0.002}
    with_drag = cm.integrate_flight(p0, v0, params(), **kwargs)
    without = cm.integrate_flight(p0, v0, drag_free(), **kwargs)

    # A sign error on the drag term would push the ball FURTHER than vacuum.
    horizontal_drag = np.linalg.norm(with_drag.position_m[-1][:2] - p0[:2])
    horizontal_vacuum = np.linalg.norm(without.position_m[-1][:2] - p0[:2])
    assert horizontal_drag < horizontal_vacuum
    # Shaved, not dominated: a drag term off by orders of magnitude (a missing
    # /2 or a radius used where the area belongs) would blow through this.
    assert horizontal_drag > 0.7 * horizontal_vacuum

    # d|v|/dt = -(g v_z + k |v|^3) / |v| < 0 while the ball is still climbing,
    # so the speed must fall strictly and monotonically through the ascent.
    ascending = with_drag.velocity_m_s[:, 2] > 0.0
    speeds = with_drag.speed_m_s[ascending]
    assert speeds.size > 100
    assert np.all(np.diff(speeds) < 0.0)


def test_long_vertical_drop_reaches_terminal_velocity():
    p = params()
    v_terminal = math.sqrt(G / p.drag_k_per_m)
    traj = cm.integrate_flight([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], p, horizon_s=40.0, step_s=0.002)
    v_end = traj.velocity_m_s[-1]
    assert v_end[2] == pytest.approx(-v_terminal, rel=1e-9)
    assert np.allclose(v_end[:2], 0.0, atol=1e-15)
    # And it approaches from above without overshooting into |v| > v_terminal.
    assert np.all(traj.speed_m_s <= v_terminal + 1e-9)


def test_integrate_flight_rejects_a_nonpositive_step():
    with pytest.raises(ValueError, match="step_s"):
        cm.integrate_flight([0, 0, 1], [1, 0, 1], params(), horizon_s=1.0, step_s=0.0)


# ── Frame conversion ──────────────────────────────────────────────────────────

# No symmetry left to hide an error behind: all three point components differ in
# magnitude and sign, the translation components differ from each other and from
# zero, and the rotation mixes all three axes (so R != R.T and R @ R != I).
POINT_W = np.array([0.37, -1.73, 2.91])
VECTOR_W = np.array([-4.1, 0.63, 7.2])
TRANSLATION = np.array([0.11, -0.29, 0.53])
ANGLE_Z = math.radians(37.0)
ANGLE_X = math.radians(23.0)


def _base_t_world() -> np.ndarray:
    return cm.make_transform(
        cm.rotation_z(ANGLE_Z) @ cm.rotation_x(ANGLE_X),
        TRANSLATION,
    )


def _expected_rotated(vec: np.ndarray) -> np.ndarray:
    """Rz(37) @ Rx(23) @ vec, written out longhand as an independent oracle."""
    cx, sx = math.cos(ANGLE_X), math.sin(ANGLE_X)
    cz, sz = math.cos(ANGLE_Z), math.sin(ANGLE_Z)
    qx = vec[0]
    qy = cx * vec[1] - sx * vec[2]
    qz = sx * vec[1] + cx * vec[2]
    return np.array([cz * qx - sz * qy, sz * qx + cz * qy, qz])


def test_world_to_base_matches_longhand_arithmetic():
    point_b, vector_b = cm.world_to_base(_base_t_world(), POINT_W, VECTOR_W)
    assert point_b == pytest.approx(_expected_rotated(POINT_W) + TRANSLATION, rel=0.0, abs=1e-14)
    # A free vector takes the rotation only — a translation leaking into a
    # velocity would be invisible on a point-only test.
    assert vector_b == pytest.approx(_expected_rotated(VECTOR_W), rel=0.0, abs=1e-14)
    assert not np.allclose(point_b, vector_b)


def test_transposed_rotation_gives_a_different_answer():
    correct = _base_t_world()
    transposed = cm.make_transform(correct[:3, :3].T, TRANSLATION)
    point_correct, _ = cm.world_to_base(correct, POINT_W, VECTOR_W)
    point_wrong, _ = cm.world_to_base(transposed, POINT_W, VECTOR_W)
    assert np.linalg.norm(point_correct - point_wrong) > 0.5


def test_rz_180_where_identity_is_correct_flips_downrange():
    # The pinned trap: for a robot whose URDF offers both a `base` and a
    # `base_link` frame at the same origin 180 degrees apart, picking the wrong
    # one leaves every magnitude intact and only mirrors x and y — the ball then
    # "arrives from behind" the arm while the map still looks sensible.
    # Measured for ur5e_p1b: world -> `base` is identity, world -> `base_link`
    # is Rz(180 deg) with zero translation. This module must never embed either.
    identity = np.eye(4)
    rz180 = cm.make_transform(cm.rotation_z(math.pi), np.zeros(3))

    release_w = np.array([3.2, 0.9, 1.7])
    velocity_w = np.array([-6.4, -1.8, 2.1])  # flying towards the base
    right_p, right_v = cm.world_to_base(identity, release_w, velocity_w)
    wrong_p, wrong_v = cm.world_to_base(rz180, release_w, velocity_w)

    assert right_p == pytest.approx(release_w, abs=1e-15)
    assert wrong_p[0] == pytest.approx(-release_w[0], abs=1e-14)
    assert wrong_p[1] == pytest.approx(-release_w[1], abs=1e-14)
    assert wrong_p[2] == pytest.approx(release_w[2], abs=1e-15)
    # Same distance from the base axis — which is exactly why the error is quiet.
    assert np.linalg.norm(wrong_p[:2]) == pytest.approx(np.linalg.norm(right_p[:2]), rel=1e-14)
    # ... but the downrange sign is inverted in both the point and the velocity.
    assert right_p[0] > 0.0 > wrong_p[0]
    assert right_v[0] < 0.0 < wrong_v[0]


def test_frame_conversion_round_trips():
    base_t_world = _base_t_world()
    world_t_base = cm.invert_transform(base_t_world)
    point_b, vector_b = cm.world_to_base(base_t_world, POINT_W, VECTOR_W)
    point_w, vector_w = cm.world_to_base(world_t_base, point_b, vector_b)
    assert point_w == pytest.approx(POINT_W, rel=0.0, abs=1e-13)
    assert vector_w == pytest.approx(VECTOR_W, rel=0.0, abs=1e-13)


def test_transform_accepts_a_rotation_translation_pair_and_rejects_a_scaled_one():
    pair = (cm.rotation_z(ANGLE_Z) @ cm.rotation_x(ANGLE_X), TRANSLATION)
    assert np.allclose(cm.as_transform(pair), _base_t_world())
    with pytest.raises(ValueError, match="orthonormal"):
        cm.as_transform((2.0 * cm.rotation_z(ANGLE_Z), TRANSLATION))


def test_transform_applies_to_a_trajectory_stack():
    stack = np.array([POINT_W, VECTOR_W, np.zeros(3)])
    out = cm.transform_point(stack, _base_t_world())
    assert out.shape == (3, 3)
    assert out[0] == pytest.approx(_expected_rotated(POINT_W) + TRANSLATION, abs=1e-14)
    assert out[2] == pytest.approx(TRANSLATION, abs=1e-15)


# ── Throw grid ────────────────────────────────────────────────────────────────


def test_grid_count_is_the_product_of_the_axis_lengths():
    axes = {
        "distances_m": (3.0, 4.0, 5.5),
        "azimuths_deg": (-25.0, 0.0),
        "release_heights_m": (1.1, 1.9, 2.4, 2.8),
        "aim_deviations_deg": (-8.0, 0.0, 8.0),
        "speeds_m_s": (5.0, 7.5),
        "elevations_deg": (15.0, 25.0, 35.0, 45.0, 55.0),
    }
    throws = cm.generate_throw_grid(**axes)
    expected = 1
    for values in axes.values():
        expected *= len(values)
    assert len(throws) == expected
    assert len({tuple(t.position_m) + tuple(t.velocity_m_s) for t in throws}) == expected
    # The default grid is also a product, and distance is an axis (default: one value).
    assert len(cm.generate_throw_grid()) == (
        len(cm.DEFAULT_DISTANCES_M)
        * len(cm.DEFAULT_AZIMUTHS_DEG)
        * len(cm.DEFAULT_RELEASE_HEIGHTS_M)
        * len(cm.DEFAULT_AIM_DEVIATIONS_DEG)
        * len(cm.DEFAULT_SPEEDS_M_S)
        * len(cm.DEFAULT_ELEVATIONS_DEG)
    )


def test_every_throw_reproduces_its_requested_speed_elevation_and_geometry():
    base_xy = (0.4, -0.7)
    throws = cm.generate_throw_grid(
        base_xy_m=base_xy,
        distances_m=(3.0, 4.5),
        azimuths_deg=(-35.0, 0.0, 110.0),
        release_heights_m=(1.3, 2.1),
        aim_deviations_deg=(-12.0, 0.0, 12.0),
        speeds_m_s=(5.0, 8.5),
        elevations_deg=(-5.0, 18.0, 40.0),
    )
    for t in throws:
        speed = float(np.linalg.norm(t.velocity_m_s))
        assert speed == pytest.approx(t.speed_m_s, rel=1e-14)
        elevation = math.degrees(math.asin(t.velocity_m_s[2] / speed))
        assert elevation == pytest.approx(t.elevation_deg, abs=1e-12)

        # Release geometry: world z is the release height, and the horizontal
        # offset from the base axis is the requested distance at the requested
        # azimuth.
        assert t.position_m[2] == pytest.approx(t.release_height_m, rel=1e-15)
        offset = t.position_m[:2] - np.asarray(base_xy)
        assert float(np.linalg.norm(offset)) == pytest.approx(t.distance_m, rel=1e-14)
        assert math.degrees(math.atan2(offset[1], offset[0])) == pytest.approx(
            t.azimuth_deg if t.azimuth_deg <= 180.0 else t.azimuth_deg - 360.0, abs=1e-12
        )

        # Downrange: the horizontal velocity points towards the base axis. An
        # inward-aim sign error would leave speed and elevation intact and only
        # send every ball away from the robot.
        downrange = float(np.dot(t.velocity_m_s[:2], t.inward_horizontal[:2]))
        assert downrange > 0.0
        expected_downrange = (
            t.speed_m_s
            * math.cos(math.radians(t.elevation_deg))
            * math.cos(math.radians(t.aim_deviation_deg))
        )
        assert downrange == pytest.approx(expected_downrange, rel=1e-12)


def test_throw_maps_onto_the_launch_ball_request_fields():
    throw = cm.generate_throw_grid(
        distances_m=(4.0,),
        azimuths_deg=(15.0,),
        release_heights_m=(1.9,),
        aim_deviations_deg=(0.0,),
        speeds_m_s=(7.0,),
        elevations_deg=(22.0,),
    )[0]
    request = cm.throw_to_launch_request(throw)
    assert set(request) == {"position", "velocity", "angular_velocity"}
    for field in request.values():
        assert set(field) == {"x", "y", "z"}
        assert all(isinstance(v, float) for v in field.values())
    assert [request["position"][k] for k in "xyz"] == pytest.approx(throw.position_m)
    assert [request["velocity"][k] for k in "xyz"] == pytest.approx(throw.velocity_m_s)
    # Zero spin by default: it is the assumption the flight model rests on.
    assert [request["angular_velocity"][k] for k in "xyz"] == [0.0, 0.0, 0.0]


def test_grid_rejects_an_empty_axis_and_a_degenerate_elevation():
    with pytest.raises(ValueError, match="speeds_m_s must not be empty"):
        cm.generate_throw_grid(speeds_m_s=())
    with pytest.raises(ValueError, match="elevations_deg"):
        cm.generate_throw_grid(elevations_deg=(90.0,))
    with pytest.raises(ValueError, match="distances_m"):
        cm.generate_throw_grid(distances_m=(0.0,))


def test_grid_throws_close_on_the_base_axis_when_integrated():
    p = params()
    for throw in cm.generate_throw_grid(speeds_m_s=(7.0, 9.0)):
        traj = cm.integrate_flight(
            throw.position_m, throw.velocity_m_s, p, horizon_s=1.5, step_s=0.004
        )
        assert traj.position_m.shape == (traj.time_s.size, 3)
        start = float(np.linalg.norm(throw.position_m[:2]))
        horizontal = np.linalg.norm(traj.position_m[:, :2], axis=1)
        # The ball passes the base axis somewhere in the window (it may then
        # recede again past it, so the minimum is the right statistic).
        assert float(horizontal.min()) < 0.25 * start
        assert traj.position_m[-1, 2] < throw.release_height_m


# ── ModelConfig translation ───────────────────────────────────────────────────

# A three-link chain: `root` (the model root), `mount` bolted on 180 degrees
# about z, and `moving` behind a LIMITED revolute joint. The limits matter: a
# limitless joint would leave the rigidity probe in
# `frame_placement_in_model_world` sampling the neutral configuration twice and
# the check would pass vacuously.
TINY_URDF = """<?xml version="1.0"?>
<robot name="tiny">
  <link name="root"><inertial><mass value="1.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="mount"><inertial><mass value="1.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="moving"><inertial><mass value="1.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="tip"><inertial><mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <joint name="mount_joint" type="fixed">
    <parent link="root"/><child link="mount"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0 3.141592653589793"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="mount"/><child link="moving"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="moving"/><child link="tip"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
</robot>
"""


def _shipped_style_config(tmp_path: Path) -> Path:
    """A robot config in the SHIPPED schema: sub_models as a map, flange tip."""
    path = tmp_path / "robot.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "urdf": {
                            "package": "unused_because_urdf_override_is_given",
                            "path": "unused",
                            "root_joint_type": "fixed",
                            # The flange, which is UPSTREAM of the catch frame's parent.
                            "sub_models": {"arm": {"root_link": "mount", "tip_link": "moving"}},
                            "tree_models": {"hand": {"root_link": "moving", "tip_links": ["tip"]}},
                            "extra_frames": {
                                "catch_frame": {
                                    "parent": "tip",
                                    "xyz": [0.01, 0.02, 0.03],
                                    "rpy": [0.0, 0.0, 0.0],
                                    "provisional": False,
                                }
                            },
                        }
                    }
                }
            }
        )
    )
    return path


def test_model_config_translation_adds_a_sub_model_reaching_the_catch_frame(tmp_path: Path):
    config = _shipped_style_config(tmp_path)
    urdf = tmp_path / "tiny.urdf"
    urdf.write_text(TINY_URDF)

    artifacts = cm.write_model_config([config], tmp_path / "out", urdf_override=urdf)
    doc = yaml.safe_load(artifacts.model_config_path.read_text())

    # Schema, not just content: LoadModelConfig wants a flat root, `urdf_path`
    # and a SEQUENCE of sub-models. The shipped map shape would parse as
    # nothing at all.
    assert set(doc) >= {"urdf_path", "root_joint_type", "sub_models", "extra_frames"}
    assert isinstance(doc["sub_models"], list)
    assert doc["urdf_path"] == str(artifacts.urdf_path)
    assert Path(doc["urdf_path"]).read_text() == TINY_URDF
    # tree_models are deliberately dropped (the judge only asks for a sub-model).
    assert "tree_models" not in doc

    by_name = {entry["name"]: entry for entry in doc["sub_models"]}
    assert by_name["arm"] == {"name": "arm", "root_link": "mount", "tip_link": "moving"}
    # The point of the whole function: the extra sub-model runs from the arm's
    # own root to the catch frame's PARENT, not to the flange.
    assert by_name[cm.CATCH_SUB_MODEL_NAME] == {
        "name": cm.CATCH_SUB_MODEL_NAME,
        "root_link": "mount",
        "tip_link": "tip",
    }
    assert artifacts.arm_tip_link == "moving"
    assert artifacts.catch_frame_parent == "tip"
    # extra_frames keeps the shipped MAP shape — the one key that does not change.
    assert doc["extra_frames"]["catch_frame"]["parent"] == "tip"
    assert doc["extra_frames"]["catch_frame"]["provisional"] is False
    assert artifacts.provenance["urdf_sha256"] == hashlib.sha256(TINY_URDF.encode()).hexdigest()
    assert str(config) in artifacts.provenance["robot_config_sha256"]


def _with_shipped_catch_sub_model(tmp_path: Path, tip: str) -> Path:
    """The shipped config, renamed arm, plus a shipped `<arm>_catch` entry (S6-B R-3)."""
    path = _shipped_style_config(tmp_path)
    doc = yaml.safe_load(path.read_text())
    urdf = doc["/**"]["ros__parameters"]["urdf"]
    urdf["sub_models"] = {
        "robo": {"root_link": "mount", "tip_link": "moving"},
        "robo_catch": {"root_link": "mount", "tip_link": tip},
    }
    path.write_text(yaml.safe_dump(doc))
    return path


def test_model_config_uses_the_shipped_catch_sub_model_by_name(tmp_path: Path):
    # The runtime planner plans in `urdf.sub_models.<arm>_catch`; the map must
    # judge in the SAME sub-model (G3-I), so it takes the shipped one by name
    # instead of emitting its own under another name.
    config = _with_shipped_catch_sub_model(tmp_path, tip="tip")
    urdf = tmp_path / "tiny.urdf"
    urdf.write_text(TINY_URDF)
    artifacts = cm.write_model_config([config], tmp_path / "out", urdf_override=urdf)
    assert artifacts.catch_sub_model == "robo_catch"
    doc = yaml.safe_load(artifacts.model_config_path.read_text())
    names = [entry["name"] for entry in doc["sub_models"]]
    assert names == ["robo", "robo_catch"]


def test_model_config_refuses_a_shipped_catch_sub_model_that_misses_the_catch_parent(
    tmp_path: Path,
):
    config = _with_shipped_catch_sub_model(tmp_path, tip="moving")
    urdf = tmp_path / "tiny.urdf"
    urdf.write_text(TINY_URDF)
    with pytest.raises(SystemExit, match="catch frame's parent"):
        cm.write_model_config([config], tmp_path / "out", urdf_override=urdf)


def test_model_config_translation_refuses_an_undeclared_catch_frame(tmp_path: Path):
    config = _shipped_style_config(tmp_path)
    urdf = tmp_path / "tiny.urdf"
    urdf.write_text(TINY_URDF)
    with pytest.raises(SystemExit, match="extra_frames.pocket_frame"):
        cm.write_model_config(
            [config], tmp_path / "out", catch_frame="pocket_frame", urdf_override=urdf
        )


def test_frame_placement_reads_the_model_world_transform_and_refuses_a_moving_frame():
    pytest.importorskip("pinocchio")
    mount = cm.frame_placement_in_model_world(TINY_URDF, "mount")
    # The whole trap in miniature: `mount` is at the model root's origin, turned
    # 180 degrees about z. Composing world->base with THIS is what keeps the
    # judge's model-world coordinates from being the arm-base ones.
    assert mount == pytest.approx(
        cm.make_transform(cm.rotation_z(math.pi), [0.1, 0.2, 0.3]), abs=1e-12
    )
    with pytest.raises(SystemExit, match="not rigid"):
        cm.frame_placement_in_model_world(TINY_URDF, "tip")


# ── Catch-candidate sampling ──────────────────────────────────────────────────


def _straight_drag_free_flight() -> cm.Trajectory:
    """x(t) = 4 - 4t, z(t) = 2 - g t^2 / 2, sampled every 0.1 s (RK4 is exact here)."""
    return cm.integrate_flight(
        [4.0, 0.0, 2.0], [-4.0, 0.0, 0.0], drag_free(), horizon_s=1.5, step_s=0.1
    )


def test_candidate_sampling_hits_the_hand_computed_instants():
    traj = _straight_drag_free_flight()
    got = cm.sample_catch_candidates(
        traj, throw_index=7, window_s=(0.0, 1.0), stride_s=0.2, min_flight_time_s=0.0
    )
    assert [round(c.time_s, 6) for c in got] == [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
    for c in got:
        t = c.time_s
        assert c.throw_index == 7
        assert c.position_m == pytest.approx(
            [4.0 - 4.0 * t, 0.0, 2.0 - 0.5 * G * t * t], abs=1e-12
        )
        assert c.velocity_m_s == pytest.approx([-4.0, 0.0, -G * t], abs=1e-12)
        assert c.speed_m_s == pytest.approx(math.hypot(4.0, G * t), rel=1e-14)


def test_candidate_sampling_flight_time_floor_removes_and_re_anchors():
    traj = _straight_drag_free_flight()
    got = cm.sample_catch_candidates(
        traj, window_s=(0.0, 1.0), stride_s=0.2, min_flight_time_s=0.5
    )
    # The floor is a filter AND the anchor: 0.0/0.2/0.4 are gone and the stride
    # grid now starts at 0.5, so the surviving instants are not a subset of the
    # unfiltered ones. That is documented behaviour, and pinning it here is what
    # stops someone "fixing" it into a silent off-by-one-stride.
    assert [round(c.time_s, 6) for c in got] == [0.5, 0.7, 0.9]


def test_candidate_sampling_reach_prefilter_keeps_only_the_near_instant():
    traj = _straight_drag_free_flight()
    # |p| at t = 0.5 / 0.7 / 0.9 is 2.1445 / 1.2660 / 2.0132 m from the origin,
    # so a 2.0 m radius keeps exactly the middle one — the ball is too far
    # downrange before it and has fallen too low after it.
    got = cm.sample_catch_candidates(
        traj,
        window_s=(0.0, 1.0),
        stride_s=0.2,
        min_flight_time_s=0.5,
        reach_filter=cm.max_distance_filter([0.0, 0.0, 0.0], 2.0),
    )
    assert [round(c.time_s, 6) for c in got] == [0.7]
    assert float(np.linalg.norm(got[0].position_m)) == pytest.approx(1.26600, abs=1e-4)


def test_candidate_sampling_rejects_a_reversed_window_and_a_zero_stride():
    traj = _straight_drag_free_flight()
    with pytest.raises(ValueError, match="ordered"):
        cm.sample_catch_candidates(traj, window_s=(1.0, 0.5), stride_s=0.1)
    with pytest.raises(ValueError, match="stride_s"):
        cm.sample_catch_candidates(traj, window_s=(0.0, 1.0), stride_s=0.0)


# ── Judge CSV contracts ───────────────────────────────────────────────────────

RESULT_HEADER = (
    "id,seed_id,accepted,reason,reason_name,iterations,pos_error,theta,w5,w6,w5_valid,w6_valid,"
    "manip_converged,manip_grad_norm,manip_grad_failures,sigma_min,lambda_sq,qp_status,"
    "qp_iterations,qp_failures,nv,q0,q1,q2"
)
# A converged, accepted pose.
RESULT_POSED = "0,0,1,0,none,7,0.001,0.012,0.5,0.31,1,1,1,0,0,0.2,0,0,4,0,3,0.1,-0.2,0.3"
# `not_converged` leaves no pose, so the three q columns are EMPTY — the row
# ends in three commas. This is the row a `float(cell)` turns into a posture of
# zeros, and the row a trailing-empty-dropping splitter reads as too short.
RESULT_POSELESS = "1,0,0,11,not_converged,20,0,0,0,0,0,0,1,0,0,0.03,0,0,18,0,3,,,"


def test_result_csv_keeps_a_poseless_row_distinct_from_a_zero_pose():
    rows = cm.parse_result_csv("\n".join([RESULT_HEADER, RESULT_POSED, RESULT_POSELESS]) + "\n")
    assert [r.id for r in rows] == [0, 1]

    posed, poseless = rows
    assert posed.accepted is True
    assert posed.reason_name == "none"
    assert posed.q is not None
    assert posed.q == pytest.approx([0.1, -0.2, 0.3])
    assert posed.w5 == pytest.approx(0.5)
    assert posed.w6 == pytest.approx(0.31)

    # The assertion the whole parser exists for: absent, not zero.
    assert poseless.accepted is False
    assert poseless.reason_name == "not_converged"
    assert poseless.q is None
    assert poseless.nv == 3


def test_result_csv_rejects_a_half_written_pose():
    garbled = "2,0,0,13,below_manip_min,9,0.001,0.01,0.05,0.02,1,1,1,0,0,0.1,0,0,4,0,3,0.1,,0.3"
    with pytest.raises(ValueError, match="partially written pose"):
        cm.parse_result_csv("\n".join([RESULT_HEADER, garbled]) + "\n")


def test_shard_completeness_detects_a_short_file(tmp_path: Path):
    complete = tmp_path / "shard_out.csv"
    complete.write_text("\n".join([RESULT_HEADER, RESULT_POSED, RESULT_POSELESS]) + "\n")
    assert cm.shard_is_complete(complete, [0, 1]) is True

    # The failure that would otherwise read as "those throws were not
    # catchable": the judge died after the first row.
    short = tmp_path / "short_out.csv"
    short.write_text("\n".join([RESULT_HEADER, RESULT_POSED]) + "\n")
    assert cm.shard_is_complete(short, [0, 1]) is False

    # A stale file from a different shard has the right row count and the wrong ids.
    assert cm.shard_is_complete(complete, [4, 5]) is False
    assert cm.shard_is_complete(tmp_path / "absent.csv", [0]) is False
    header_only = tmp_path / "header_out.csv"
    header_only.write_text(RESULT_HEADER + "\n")
    assert cm.shard_is_complete(header_only, [0]) is False


def test_candidate_csv_refuses_duplicate_ids_and_non_finite_values(tmp_path: Path):
    good = cm.JudgeCandidate(
        id=0, seed_id=0, p_c_model_m=np.array([0.4, 0.1, 0.5]), v_model_m_s=np.array([-4.0, 0, -2])
    )
    with pytest.raises(ValueError, match="unique"):
        cm.write_candidate_csv(tmp_path / "a.csv", [good, good])
    bad = cm.JudgeCandidate(
        id=1, seed_id=0, p_c_model_m=np.array([np.nan, 0.0, 0.0]), v_model_m_s=np.zeros(3)
    )
    with pytest.raises(ValueError, match="non-finite"):
        cm.write_candidate_csv(tmp_path / "b.csv", [bad])
    assert cm.write_candidate_csv(tmp_path / "c.csv", [good]) == 1
    header = (tmp_path / "c.csv").read_text().splitlines()[0]
    assert header.split(",") == list(cm.CANDIDATE_CSV_HEADER)


def test_seed_csv_refuses_ragged_seeds(tmp_path: Path):
    with pytest.raises(ValueError, match="same width"):
        cm.write_seed_csv(tmp_path / "s.csv", [[0.0, 0.0], [0.0]])
    cm.write_seed_csv(tmp_path / "s.csv", [[0.0, 0.1], [0.2, 0.3]])
    lines = (tmp_path / "s.csv").read_text().splitlines()
    assert lines[0] == "seed_id,q0,q1"
    assert lines[1].startswith("0,")
    assert lines[2].startswith("1,")


def test_to_judge_candidates_applies_the_model_world_transform_and_crosses_seeds():
    traj = _straight_drag_free_flight()
    sampled = cm.sample_catch_candidates(
        traj, window_s=(0.0, 0.2), stride_s=0.2, min_flight_time_s=0.0
    )
    assert len(sampled) == 2
    rz180 = cm.make_transform(cm.rotation_z(math.pi), np.zeros(3))
    rows = cm.to_judge_candidates(sampled, model_world_t_world=rz180, seed_ids=(0, 1))

    assert [r.id for r in rows] == [0, 1, 2, 3]
    assert [r.seed_id for r in rows] == [0, 1, 0, 1]
    for row in rows:
        assert row.p_c_world_m is not None
        # The judge's frame is not the world frame: x and y are mirrored while
        # z and every magnitude survive, which is the quiet part of the trap.
        assert row.p_c_model_m[0] == pytest.approx(-row.p_c_world_m[0], abs=1e-14)
        assert row.p_c_model_m[1] == pytest.approx(-row.p_c_world_m[1], abs=1e-14)
        assert row.p_c_model_m[2] == pytest.approx(row.p_c_world_m[2], abs=1e-15)
        assert row.v_model_m_s[0] == pytest.approx(-row.v_world_m_s[0], abs=1e-14)


def test_perturb_candidates_makes_six_axis_aligned_copies():
    base = cm.JudgeCandidate(
        id=5,
        seed_id=2,
        p_c_model_m=np.array([0.4, 0.1, 0.5]),
        v_model_m_s=np.array([-4.0, 0.5, -2.0]),
        throw_index=3,
        time_s=1.1,
    )
    copies, sources = cm.perturb_candidates([base], epsilon_m=0.01, id_offset=100)
    assert len(copies) == 6
    assert sorted(sources) == list(range(100, 106))
    assert set(sources.values()) == {5}
    deltas = [np.asarray(c.p_c_model_m) - base.p_c_model_m for c in copies]
    assert sorted(tuple(np.round(d, 12)) for d in deltas) == sorted(
        [
            (-0.01, 0.0, 0.0),
            (0.01, 0.0, 0.0),
            (0.0, -0.01, 0.0),
            (0.0, 0.01, 0.0),
            (0.0, 0.0, -0.01),
            (0.0, 0.0, 0.01),
        ]
    )
    for c in copies:
        assert c.v_model_m_s == pytest.approx(base.v_model_m_s)
        assert c.seed_id == 2
        assert c.throw_index == 3


# ── Aggregation fixtures ──────────────────────────────────────────────────────


def judged(
    cid: int,
    throw: int,
    seed: int,
    *,
    accepted: bool,
    w5: float = 0.5,
    w6: float | None = None,
    reason: str | None = None,
    theta: float = 0.0,
    time_s: float = 1.0,
    speed: float = 6.0,
    pose: list[float] | None = None,
    w5_valid: bool = True,
    w6_valid: bool = True,
) -> cm.JudgedCandidate:
    candidate = cm.JudgeCandidate(
        id=cid,
        seed_id=seed,
        p_c_model_m=np.array([0.4, 0.1, 0.5]),
        v_model_m_s=np.array([-speed, 0.0, 0.0]),
        throw_index=throw,
        time_s=time_s,
        speed_m_s=speed,
        p_c_world_m=np.array([-0.4, -0.1, 0.5]),
        v_world_m_s=np.array([speed, 0.0, 0.0]),
    )
    result = cm.JudgeResult(
        id=cid,
        seed_id=seed,
        accepted=accepted,
        reason=0 if accepted else 13,
        reason_name=("none" if accepted else (reason or "below_manip_min")),
        iterations=8,
        pos_error=1e-3,
        theta=theta,
        w5=w5,
        w6=0.5 * w5 if w6 is None else w6,
        w5_valid=w5_valid,
        w6_valid=w6_valid,
        manip_converged=True,
        manip_grad_norm=0.0,
        manip_grad_failures=0,
        sigma_min=0.1,
        lambda_sq=0.0,
        qp_status=0,
        qp_iterations=3,
        qp_failures=0,
        nv=3,
        q=None if pose is None else np.asarray(pose, dtype=float),
    )
    return cm.JudgedCandidate(candidate=candidate, result=result)


def test_join_results_refuses_a_partial_join():
    candidates = [
        cm.JudgeCandidate(id=i, seed_id=0, p_c_model_m=np.zeros(3), v_model_m_s=np.zeros(3))
        for i in range(3)
    ]
    results = [judged(0, 0, 0, accepted=True).result, judged(1, 0, 0, accepted=False).result]
    with pytest.raises(ValueError, match="have no result"):
        cm.join_results(candidates, results)
    with pytest.raises(ValueError, match="no candidate"):
        cm.join_results(candidates[:1], results)


def test_summarize_throws_picks_the_max_w_accepted_candidate():
    rows = [
        judged(0, 0, 0, accepted=True, w5=0.2, time_s=1.0, pose=[0.1, 0.2, 0.3]),
        judged(1, 0, 0, accepted=True, w5=0.9, time_s=1.4, speed=7.0, pose=[0.4, 0.5, 0.6]),
        judged(2, 0, 0, accepted=False, reason="not_converged"),
        judged(3, 1, 0, accepted=False, reason="not_converged"),
    ]
    outcomes = cm.summarize_throws(rows)
    assert [o.throw_index for o in outcomes] == [0, 1]
    first, second = outcomes
    assert first.accepted is True
    assert first.candidates == 3
    assert first.accepted_candidates == 2
    assert first.best_id == 1
    assert first.best_w == pytest.approx(0.9)
    assert first.best_time_s == pytest.approx(1.4)
    assert first.best_speed_m_s == pytest.approx(7.0)
    assert first.best_q == pytest.approx([0.4, 0.5, 0.6])
    assert second.accepted is False
    assert second.best_q is None

    # Ranking by w6 is a different question about the same poses, and the
    # function must not silently keep answering the w5 one.
    by_w6 = cm.summarize_throws(
        [
            judged(0, 0, 0, accepted=True, w5=0.2, w6=0.9),
            judged(1, 0, 0, accepted=True, w5=0.9, w6=0.1),
        ],
        manip_column="w6",
    )
    assert by_w6[0].best_id == 0


def test_reason_histogram_counts_rejections_only():
    rows = [
        judged(0, 0, 0, accepted=True),
        judged(1, 0, 0, accepted=True),
        judged(2, 1, 0, accepted=False, reason="not_converged"),
        judged(3, 1, 0, accepted=False, reason="not_converged"),
        judged(4, 2, 0, accepted=False, reason="not_converged"),
        judged(5, 2, 0, accepted=False, reason="below_manip_min"),
        judged(6, 3, 0, accepted=False, reason="below_manip_min"),
        judged(7, 3, 0, accepted=False, reason="speed_too_low"),
    ]
    histogram = cm.reason_histogram(rows)
    assert histogram == {"not_converged": 3, "below_manip_min": 2, "speed_too_low": 1}
    # Most frequent first, and `none` is never a bar in a rejection histogram.
    assert list(histogram) == ["not_converged", "below_manip_min", "speed_too_low"]
    assert sum(histogram.values()) == 6


def test_manipulability_distributions_keep_w5_and_w6_apart():
    rows = [
        judged(0, 0, 0, accepted=True, w5=0.2, w6=2.0),
        judged(1, 0, 0, accepted=True, w5=0.4, w6=4.0),
        judged(2, 1, 0, accepted=True, w5=0.6, w6=6.0, w6_valid=False),
        judged(3, 1, 0, accepted=False, w5=99.0, w6=99.0),
    ]
    report = cm.manipulability_distributions(rows)
    assert report["accepted_candidates"] == 3
    # Rejected candidates never enter either distribution.
    assert report["w5"]["n"] == 3
    assert report["w5"]["max"] == pytest.approx(0.6)
    # ... and an invalid factorisation is dropped from ITS OWN distribution only.
    assert report["w6"]["n"] == 2
    assert report["w6"]["max"] == pytest.approx(4.0)
    assert report["w6_invalid"] == 1
    assert report["w5_invalid"] == 0
    # No pooled statistic exists to be misread.
    assert "w56" not in report
    assert set(report["w5"]) == set(report["w6"])


def test_theta_report_gives_both_readings_of_near_alpha_max():
    rows = [
        judged(0, 0, 0, accepted=True, theta=0.05),
        judged(1, 0, 0, accepted=True, theta=0.10),
        judged(2, 1, 0, accepted=True, theta=0.20),
        judged(3, 1, 0, accepted=True, theta=0.25),
        judged(4, 2, 0, accepted=False, theta=9.0),
    ]
    report = cm.theta_report(rows, alpha_max=0.26, near_fraction=0.5)
    assert report["near_threshold_rad"] == pytest.approx(0.13)
    assert report["accepted_candidates"] == 4
    assert report["count_below"] == 2
    assert report["count_near_limit"] == 2
    assert report["fraction_below"] == pytest.approx(0.5)
    assert report["fraction_near_limit"] == pytest.approx(0.5)
    assert report["max_over_alpha_max"] == pytest.approx(0.25 / 0.26)
    # The rejected candidate's theta is not in the distribution.
    assert report["theta_rad"]["max"] == pytest.approx(0.25)


def test_seed_ranking_puts_coverage_before_conditioning():
    # Four throws, three seeds, constructed so the two criteria DISAGREE:
    #   seed 0: catches throws 0,1      -> 0.50 coverage, mean log w5 = 1.0
    #   seed 1: catches throws 2,3      -> 0.50 coverage, mean log w5 = 3.0
    #   seed 2: catches throws 0,1,2    -> 0.75 coverage, mean log w5 = 0.1
    # Ranking by mean log w5 alone would put seed 1 first and seed 2 last.
    plan = {
        0: ({0, 1}, math.e**1.0),
        1: ({2, 3}, math.e**3.0),
        2: ({0, 1, 2}, math.e**0.1),
    }
    rows = []
    cid = 0
    for seed, (accepted_throws, w5) in plan.items():
        for throw in range(4):
            rows.append(judged(cid, throw, seed, accepted=throw in accepted_throws, w5=w5))
            cid += 1

    comparison = cm.rank_wait_pose_seeds(rows)
    assert [r.seed_id for r in comparison.ranking] == [2, 1, 0]
    assert comparison.best.seed_id == 2
    assert comparison.best.accepted_fraction == pytest.approx(0.75)
    assert comparison.best.mean_log_w5 == pytest.approx(0.1)
    # The tie between seeds 0 and 1 is broken by mean log w5, not by seed_id.
    assert comparison.ranking[1].mean_log_w5 == pytest.approx(3.0)
    assert comparison.ranking[2].mean_log_w5 == pytest.approx(1.0)
    assert comparison.runner_up.seed_id == 1
    assert comparison.coverage_gap == pytest.approx(0.25)
    assert comparison.throws == 4


def test_seed_ranking_refuses_seeds_over_different_throw_sets():
    rows = [
        judged(0, 0, 0, accepted=True),
        judged(1, 1, 0, accepted=True),
        judged(2, 0, 1, accepted=True),
    ]
    with pytest.raises(ValueError, match="SAME throw set"):
        cm.rank_wait_pose_seeds(rows)


def test_boundary_count_uses_the_judge_on_the_perturbed_copies():
    nominal = [
        judged(0, 0, 0, accepted=True).result,
        judged(1, 1, 0, accepted=True).result,
        judged(2, 2, 0, accepted=False).result,
    ]
    # id 0's copies all survive; one of id 1's copies is rejected, so id 1 is on
    # the boundary at this epsilon and id 0 is not.
    perturbed = [judged(10 + i, 0, 0, accepted=True).result for i in range(6)]
    perturbed += [judged(20 + i, 1, 0, accepted=True).result for i in range(5)]
    perturbed += [judged(25, 1, 0, accepted=False, reason="below_manip_min").result]
    sources = {10 + i: 0 for i in range(6)} | {20 + i: 1 for i in range(6)}

    report = cm.count_boundary_candidates(nominal, perturbed, sources, epsilon_m=0.01)
    assert report["accepted"] == 2
    assert report["probed_accepted"] == 2
    assert report["on_boundary"] == 1
    assert report["on_boundary_ids"] == [1]
    assert report["fraction_of_probed_accepted"] == pytest.approx(0.5)
    assert report["flip_reasons"] == {"below_manip_min": 1}
    assert "epsilon" in report["definition"]

    with pytest.raises(ValueError, match="no source candidate"):
        cm.count_boundary_candidates(nominal, perturbed, {}, epsilon_m=0.01)


def test_throw_region_proposal_covers_the_accepted_throws_by_hand():
    # 6 throws: azimuth in {-30, 0, 30} x speed in {5, 7}, everything else fixed.
    throws = cm.generate_throw_grid(
        distances_m=(4.0,),
        azimuths_deg=(-30.0, 0.0, 30.0),
        release_heights_m=(1.8,),
        aim_deviations_deg=(0.0,),
        speeds_m_s=(5.0, 7.0),
        elevations_deg=(20.0,),
    )
    assert len(throws) == 6
    # Accept (azimuth 0, speed 5) and (azimuth 30, speed 7). The covering box is
    # then azimuth [0, 30] x speed [5, 7], which also contains the two throws
    # that were REJECTED — 4 throws in the box, 2 of them accepted.
    accepted = {
        i
        for i, t in enumerate(throws)
        if (t.azimuth_deg, t.speed_m_s) in {(0.0, 5.0), (30.0, 7.0)}
    }
    assert len(accepted) == 2
    rows = [
        judged(i, i, 0, accepted=i in accepted, time_s=1.2 if i in accepted else 1.0)
        for i in range(len(throws))
    ]
    outcomes = cm.summarize_throws(rows)
    proposal = cm.propose_throw_region(throws, outcomes, rows, base_frame="base")["sim"][
        "throw_region"
    ]

    assert proposal["base_frame"] == "base"
    assert proposal["accepted_throws"] == 2
    assert proposal["accepted_fraction"] == pytest.approx(2 / 6)
    assert proposal["azimuth_base_rad"] == pytest.approx([0.0, math.radians(30.0)])
    assert proposal["covered_axes_deg"]["azimuth_base_deg"] == pytest.approx([0.0, 30.0])
    assert proposal["speed_m_s"] == pytest.approx([5.0, 7.0])
    assert proposal["elevation_rad"] == pytest.approx([math.radians(20.0)] * 2)
    assert proposal["heading_offset_rad"] == pytest.approx([0.0, 0.0])
    assert proposal["z_world_m"] == pytest.approx([1.8, 1.8])
    assert proposal["distance_base_m"] == pytest.approx([4.0, 4.0])
    assert proposal["flight_time_s"] == pytest.approx([1.2, 1.2])
    # The honesty figure: the box is a superset of the accepted set.
    assert proposal["box_contains_throws"] == 4
    assert proposal["box_accepted_fraction"] == pytest.approx(0.5)
    assert proposal["provisional"] is True
    # The tool does not invent an aim point.
    assert "aim_point_base_m" not in proposal


def test_throw_region_proposal_says_nothing_when_nothing_was_accepted():
    throws = cm.generate_throw_grid(
        distances_m=(4.0,),
        azimuths_deg=(0.0,),
        release_heights_m=(1.8,),
        aim_deviations_deg=(0.0,),
        speeds_m_s=(5.0,),
        elevations_deg=(20.0,),
    )
    rows = [judged(0, 0, 0, accepted=False)]
    proposal = cm.propose_throw_region(
        throws, cm.summarize_throws(rows), rows, base_frame="link_0"
    )["sim"]["throw_region"]
    assert proposal["accepted_throws"] == 0
    assert "speed_m_s" not in proposal
    assert "no throw was accepted" in proposal["note"]


def test_generated_yaml_carries_the_header_and_the_provenance():
    text = cm.render_generated_yaml({"sim": {"throw_region": {"base_frame": "base"}}}, {"a": 1})
    assert text.startswith("# Generated by rtc_tools.analysis.catchability_map")
    doc = yaml.safe_load(text)
    assert doc["sim"]["throw_region"]["base_frame"] == "base"
    assert doc["provenance"] == {"a": 1}


def test_csv_outputs_leave_a_poseless_row_empty(tmp_path: Path):
    rows = [
        judged(0, 0, 0, accepted=True, pose=[0.1, 0.2, 0.3]),
        judged(1, 1, 0, accepted=False, reason="not_converged"),
    ]
    path = cm.write_candidate_result_csv(tmp_path / "candidates.csv", rows)
    lines = path.read_text().splitlines()
    header = lines[0].split(",")
    assert header[-3:] == ["q0", "q1", "q2"]
    assert lines[1].split(",")[-3:] == ["0.1", "0.2", "0.3"]
    # Empty, not "0.0": the map's own record keeps the judge's three states.
    assert lines[2].split(",")[-3:] == ["", "", ""]

    histogram = cm.reason_histogram(rows)
    hist_path = cm.write_reason_histogram_csv(tmp_path / "hist.csv", histogram)
    assert hist_path.read_text().splitlines()[1].startswith("not_converged,1,")


# ── A stand-in judge: same CLI and CSV, a verdict that is a pure function of its inputs ──

# The point of the stand-in is that a STALE verdict is observable as a WRONG
# verdict, not only as a missing subprocess call: every input the fingerprint
# covers moves a number in the output.
#
#   accepted  <=>  |p_c.xy| <= fake_reach (params file, default 1.0)
#                  and sign(v_y) == sign(seed q0)
#   w5        =    |seed q0|
#   theta     =    0.30 on every accepted row
_FAKE_JUDGE = r"""#!{python}
import csv
import sys
from pathlib import Path

argv = sys.argv[1:]
if "--print-options" in argv:
    # Optional capability: offered only when the test drops this file next to it.
    report = Path(__file__).with_name("print_options.txt")
    if not report.is_file():
        sys.exit(2)
    print(report.read_text(), end="")
    sys.exit(0)
if "--dump-frame" in argv:
    print("frame catch_frame parent tip nv 2")
    print("residual_translation_m 0.0")
    print("residual_rotation_fro 0.0")
    sys.exit(0)
args = dict(zip(argv[::2], argv[1::2]))
with Path(__file__).with_name("calls.log").open("a") as log:
    log.write(args["--candidates"] + "\n")
reach = 1.0
if "--params" in args:
    for line in Path(args["--params"]).read_text().splitlines():
        if line.strip().startswith("fake_reach:"):
            reach = float(line.split(":")[1])
seeds = {{}}
with open(args["--seeds"], newline="") as handle:
    for row in csv.DictReader(handle):
        seeds[int(row["seed_id"])] = float(row["q0"])
lines = [
    "id,seed_id,accepted,reason,reason_name,iterations,pos_error,theta,w5,w6,w5_valid,"
    "w6_valid,manip_converged,manip_grad_norm,manip_grad_failures,sigma_min,lambda_sq,"
    "qp_status,qp_iterations,qp_failures,nv,q0,q1"
]
with open(args["--candidates"], newline="") as handle:
    for row in csv.DictReader(handle):
        q0 = seeds[int(row["seed_id"])]
        radius = (float(row["p_c_x"]) ** 2 + float(row["p_c_y"]) ** 2) ** 0.5
        ok = radius <= reach and (float(row["v_y"]) > 0.0) == (q0 > 0.0)
        head = f"{{row['id']}},{{row['seed_id']}},"
        if ok:
            tail = "0.05,1,1,1,0,0,0.2,0,0,4,0,2,0.1,0.2"
            lines.append(head + f"1,0,none,5,0.001,0.3,{{abs(q0)}}," + tail)
        else:
            lines.append(head + "0,11,not_converged,20,0,0,0,0,0,0,1,0,0,0.03,0,0,18,0,2,,")
Path(args["--out"]).write_text("\n".join(lines) + "\n")
"""


def _fake_judge(tmp_path: Path) -> Path:
    path = tmp_path / "fake_judge.py"
    path.write_text(_FAKE_JUDGE.format(python=sys.executable))
    path.chmod(0o755)
    return path


def _judge_calls(judge: Path) -> int:
    log = judge.with_name("calls.log")
    return len(log.read_text().splitlines()) if log.is_file() else 0


class _FakeRun:
    """One judge invocation's worth of input files, each rewritable in place."""

    def __init__(self, tmp_path: Path):
        self.judge = _fake_judge(tmp_path)
        self.urdf = tmp_path / "model.urdf"
        self.model_config = tmp_path / "model_config.yaml"
        self.seeds = tmp_path / "seeds.csv"
        self.params = tmp_path / "params.yaml"
        self.work = tmp_path / "shards"
        self.urdf.write_text(TINY_URDF)
        self.model_config.write_text(yaml.safe_dump({"urdf_path": str(self.urdf)}))
        cm.write_seed_csv(self.seeds, [[0.5, 0.0]])
        self.params.write_text("fake_reach: 1.0\n")
        self.x = (0.4, 2.0)

    def candidates(self) -> list[cm.JudgeCandidate]:
        return [
            cm.JudgeCandidate(
                id=i,
                seed_id=0,
                p_c_model_m=np.array([x, 0.0, 0.5]),
                v_model_m_s=np.array([-4.0, 1.0, -2.0]),
                throw_index=i,
            )
            for i, x in enumerate(self.x)
        ]

    def run(self) -> list[cm.JudgeResult]:
        invocation = cm.JudgeInvocation(
            judge=self.judge,
            model_config=self.model_config,
            seeds=self.seeds,
            params=self.params,
        )
        return cm.run_judge_batch(invocation, self.candidates(), work_dir=self.work, workers=1)


def test_a_completed_shard_is_reused_when_no_input_changed(tmp_path: Path):
    run = _FakeRun(tmp_path)
    first = run.run()
    assert [r.accepted for r in first] == [True, False]
    assert _judge_calls(run.judge) == 1
    sidecar = run.work / "shard_00000_fingerprint.json"
    assert set(json.loads(sidecar.read_text())["parts"]) >= {
        "candidates",
        "model_config",
        "urdf_path",
        "seeds",
        "params",
        "sub_model",
        "catch_frame",
        "judge",
    }

    # The CLI rewrites the model config, the URDF and the seed file on EVERY
    # run, so reuse has to survive a rewrite with identical bytes (new mtime):
    # a fingerprint on mtimes would make resume a no-op.
    for path in (run.urdf, run.model_config, run.seeds, run.params):
        path.write_bytes(path.read_bytes())
    again = run.run()
    assert _judge_calls(run.judge) == 1, "an unchanged shard must not be re-judged"
    assert [(r.id, r.accepted, r.w5) for r in again] == [(r.id, r.accepted, r.w5) for r in first]


def test_a_shard_is_not_reused_when_only_the_params_file_content_changes(tmp_path: Path):
    run = _FakeRun(tmp_path)
    assert [r.accepted for r in run.run()] == [True, False]
    # Same PATH, different content: the 2.0 m candidate is now within reach.
    run.params.write_text("fake_reach: 3.0\n")
    again = run.run()
    assert _judge_calls(run.judge) == 2
    assert [r.accepted for r in again] == [True, True], "stale verdicts of the old params"


def test_a_shard_is_not_reused_when_only_a_seed_value_changes(tmp_path: Path):
    run = _FakeRun(tmp_path)
    assert [r.w5 for r in run.run() if r.accepted] == [0.5]
    cm.write_seed_csv(run.seeds, [[0.75, 0.0]])
    again = run.run()
    assert _judge_calls(run.judge) == 2
    assert [r.w5 for r in again if r.accepted] == [0.75], "stale verdicts of the old seed"


def test_a_shard_is_not_reused_when_coordinates_change_under_the_same_ids(tmp_path: Path):
    run = _FakeRun(tmp_path)
    assert [r.accepted for r in run.run()] == [True, False]
    # The ids are 0 and 1 on both runs — which is exactly why an id comparison
    # cannot tell these two grids apart.
    run.x = (1.5, 0.2)
    assert [c.id for c in run.candidates()] == [0, 1]
    again = run.run()
    assert _judge_calls(run.judge) == 2
    assert [r.accepted for r in again] == [False, True], "stale verdicts of the old grid"


def test_a_shard_is_not_reused_across_a_model_a_judge_or_a_missing_sidecar(tmp_path: Path):
    run = _FakeRun(tmp_path)
    run.run()
    sidecar = run.work / "shard_00000_fingerprint.json"

    # The model config's BYTES are unchanged here; only the URDF it names moved.
    run.urdf.write_text(TINY_URDF.replace('xyz="0.5 0 0"', 'xyz="0.6 0 0"'))
    run.run()
    assert _judge_calls(run.judge) == 2

    # The judge is identified by path + size + mtime, not by content.
    stat = run.judge.stat()
    os.utime(run.judge, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000_000))
    run.run()
    assert _judge_calls(run.judge) == 3

    # An output directory written before fingerprints existed has no sidecar,
    # and "cannot tell" has to mean "re-judge".
    out = run.work / "shard_00000_out.csv"
    assert cm.shard_is_complete(out, [0, 1]) is True
    sidecar.unlink()
    assert cm.shard_is_reusable(out, [0, 1], sidecar, {"fingerprint": "x", "parts": {}}) is False
    run.run()
    assert _judge_calls(run.judge) == 4
    assert sidecar.is_file()

    # A sidecar is only as good as the file next to it: a truncated output is
    # not reusable even under a matching fingerprint.
    expected = json.loads(sidecar.read_text())
    assert cm.shard_is_reusable(out, [0, 1], sidecar, expected) is True
    out.write_text("\n".join(out.read_text().splitlines()[:-1]) + "\n")
    assert cm.shard_is_reusable(out, [0, 1], sidecar, expected) is False
    # ... and inputs that cannot be fingerprinted reuse nothing.
    assert cm.shard_is_reusable(out, [0, 1], sidecar, None) is False


# ── One wait pose, not a union over seeds ─────────────────────────────────────


def _two_disjoint_seeds() -> list[cm.JudgedCandidate]:
    """4 throws x 2 seeds: seed 0 catches throws 0,1 and seed 1 catches throw 2 only."""
    plan = {0: {0, 1}, 1: {2}}
    rows = []
    cid = 0
    for throw in range(4):
        for seed, accepted_throws in plan.items():
            rows.append(judged(cid, throw, seed, accepted=throw in accepted_throws, time_s=1.1))
            cid += 1
    return rows


def test_the_headline_is_one_seed_and_never_the_sum_of_disjoint_seeds():
    rows = _two_disjoint_seeds()
    # The defect: grouping by throw alone called 3 of 4 throws accepted, a
    # number no wait pose delivers. It is now refused outright ...
    with pytest.raises(ValueError, match="ONE wait pose"):
        cm.summarize_throws(rows)
    # ... and available only under a name that says what it is.
    assert cm.count_throws_accepted_by_any_seed(rows) == 3

    best = cm.rank_wait_pose_seeds(rows, throw_count=4).best
    assert best.seed_id == 0
    outcomes = cm.summarize_throws(rows, seed_id=best.seed_id, throw_count=4)
    assert [o.accepted for o in outcomes] == [True, True, False, False]
    assert sum(o.accepted for o in outcomes) == 2 == best.accepted_throws
    assert {o.best_seed_id for o in outcomes if o.accepted} == {0}
    assert [o.accepted for o in cm.summarize_throws(rows, seed_id=1)] == [
        False,
        False,
        True,
        False,
    ]
    with pytest.raises(ValueError, match="not among the judged seeds"):
        cm.summarize_throws(rows, seed_id=7)


def test_the_throw_region_describes_one_wait_pose_and_names_it():
    throws = cm.generate_throw_grid(
        distances_m=(4.0,),
        azimuths_deg=(-30.0, 0.0, 30.0, 60.0),
        release_heights_m=(1.8,),
        aim_deviations_deg=(0.0,),
        speeds_m_s=(5.0,),
        elevations_deg=(20.0,),
    )
    rows = _two_disjoint_seeds()
    outcomes = cm.summarize_throws(rows, seed_id=0, throw_count=len(throws))
    # `judged` feeds flight_time_s, so rows of several seeds would make THAT a union.
    with pytest.raises(ValueError, match="ONE wait pose"):
        cm.propose_throw_region(throws, outcomes, rows, base_frame="base")

    region = cm.propose_throw_region(
        throws,
        outcomes,
        cm.judged_for_seed(rows, 0),
        base_frame="base",
        wait_pose_seed_id=0,
        wait_pose_q=[0.1, 0.2],
        accepted_throws_any_seed=cm.count_throws_accepted_by_any_seed(rows),
    )["sim"]["throw_region"]
    assert region["accepted_throws"] == 2
    assert region["accepted_fraction"] == pytest.approx(0.5)
    assert region["accepted_fraction_denominator"] == cm.DENOMINATOR_GRID_THROWS
    assert region["wait_pose_seed_id"] == 0
    assert region["wait_pose_q"] == [0.1, 0.2]
    # Seed 1's throw (azimuth 30) is NOT in seed 0's box.
    assert region["covered_axes_deg"]["azimuth_base_deg"] == pytest.approx([-30.0, 0.0])
    assert region["accepted_throws_any_seed"] == 3
    assert "NOT coverage" in region["accepted_throws_any_seed_note"]


def test_a_throw_with_no_candidate_still_has_a_summary_row(tmp_path: Path):
    throws = cm.generate_throw_grid(
        distances_m=(4.0,),
        azimuths_deg=(-30.0, 0.0, 30.0, 60.0),
        release_heights_m=(1.8,),
        aim_deviations_deg=(0.0,),
        speeds_m_s=(5.0,),
        elevations_deg=(20.0,),
    )
    # Throw 2 lost every catch instant to the pre-filters: it never reached the
    # judge, so it is simply absent from `judged`.
    rows = [
        judged(0, 0, 0, accepted=True, pose=[0.1, 0.2, 0.3]),
        judged(1, 1, 0, accepted=False),
        judged(2, 3, 0, accepted=True, pose=[0.4, 0.5, 0.6]),
    ]
    assert [o.throw_index for o in cm.summarize_throws(rows)] == [0, 1, 3]

    outcomes = cm.summarize_throws(rows, throw_count=len(throws))
    assert [o.throw_index for o in outcomes] == [0, 1, 2, 3]
    assert (outcomes[2].candidates, outcomes[2].accepted, outcomes[2].best_q) == (0, False, None)
    with pytest.raises(ValueError, match="outside the grid"):
        cm.summarize_throws(rows, throw_count=3)

    # The writer holds the "one row per grid throw" contract on its own, so it
    # is fed the outcomes WITHOUT the fill-in here.
    path = cm.write_throw_summary_csv(
        tmp_path / "throw_summary.csv", throws, cm.summarize_throws(rows), seed_id=0
    )
    with path.open(newline="") as handle:
        written = list(csv.DictReader(handle))
    assert [int(r["throw_index"]) for r in written] == [0, 1, 2, 3]
    assert (written[2]["candidates"], written[2]["accepted"]) == ("0", "0")
    assert written[2]["azimuth_deg"] == "30.0"
    assert written[2]["best_q0"] == ""
    assert {r["wait_pose_seed_id"] for r in written} == {"0"}
    assert sum(int(r["accepted"]) for r in written) == 2


def test_the_seed_fraction_and_the_headline_fraction_share_the_full_grid():
    throws = cm.generate_throw_grid(
        distances_m=(4.0,),
        azimuths_deg=(-30.0, 0.0, 30.0, 60.0),
        release_heights_m=(1.8,),
        aim_deviations_deg=(0.0,),
        speeds_m_s=(5.0,),
        elevations_deg=(20.0,),
    )
    # 4 grid throws, 3 reached the judge, 2 accepted: 2/3 over the judged throws
    # against 2/4 over the grid — the disagreement this pins.
    rows = [
        judged(0, 0, 0, accepted=True),
        judged(1, 1, 0, accepted=False),
        judged(2, 3, 0, accepted=True),
    ]
    legacy = cm.rank_wait_pose_seeds(rows)
    assert legacy.best.accepted_fraction == pytest.approx(2 / 3)
    assert legacy.denominator == cm.DENOMINATOR_THROWS_WITH_CANDIDATES

    comparison = cm.rank_wait_pose_seeds(rows, throw_count=len(throws))
    outcomes = cm.summarize_throws(rows, seed_id=0, throw_count=len(throws))
    region = cm.propose_throw_region(throws, outcomes, rows, base_frame="base")["sim"][
        "throw_region"
    ]
    assert comparison.best.accepted_fraction == pytest.approx(0.5)
    assert comparison.best.accepted_fraction == pytest.approx(region["accepted_fraction"])
    assert comparison.best.throws == comparison.throws == region["throws"] == 4
    assert comparison.best.throws_with_candidates == 3
    assert comparison.denominator == cm.DENOMINATOR_GRID_THROWS
    assert comparison.best.denominator == region["accepted_fraction_denominator"]
    with pytest.raises(ValueError, match="outside the grid"):
        cm.rank_wait_pose_seeds(rows, throw_count=2)


# ── alpha_max: the bound the judge applied, from one source ───────────────────


def _params_file(tmp_path: Path, doc: object, name: str = "params.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(doc))
    return path


def test_alpha_max_comes_from_the_params_file_when_it_sets_one(tmp_path: Path):
    path = _params_file(tmp_path, {"catching": {"planner": {"ik": {"alpha_max": 0.35}}}})
    resolved = cm.resolve_alpha_max(path, None)
    assert resolved.value_rad == pytest.approx(0.35)
    assert resolved.origin == "params"
    assert str(path) in resolved.source
    assert resolved.params_shape == cm.PARAMS_SHAPE_CATCHING_AT_ROOT

    # What the fix is FOR: theta up to 0.35 was accepted, so against the old
    # hardcoded 0.26 the report claimed a theta 1.27x past its own bound.
    rows = [judged(0, 0, 0, accepted=True, theta=0.33), judged(1, 1, 0, accepted=True, theta=0.1)]
    report = cm.theta_report(rows, alpha_max=resolved.value_rad)
    assert report["alpha_max_rad"] == pytest.approx(0.35)
    assert report["max_over_alpha_max"] == pytest.approx(0.33 / 0.35)
    assert report["max_over_alpha_max"] <= 1.0


def test_alpha_max_reads_the_controller_config_shape_and_the_bare_tree(tmp_path: Path):
    wrapped = _params_file(
        tmp_path,
        {
            "some_controller": {
                "gains": [1.0],
                "catching": {"planner": {"ik": {"alpha_max": 0.31}}},
            }
        },
        "wrapped.yaml",
    )
    resolved = cm.resolve_alpha_max(wrapped, None)
    assert resolved.value_rad == pytest.approx(0.31)
    assert resolved.params_shape == cm.PARAMS_SHAPE_CONTROLLER_CONFIG

    bare = _params_file(tmp_path, {"planner": {"ik": {"alpha_max": 0.29}}}, "bare.yaml")
    resolved = cm.resolve_alpha_max(bare, None)
    assert resolved.value_rad == pytest.approx(0.29)
    assert resolved.params_shape == cm.PARAMS_SHAPE_TREE_ITSELF

    # A file whose tree cannot be FOUND is not a file that leaves the value open.
    for name, doc in (
        ("ros.yaml", {"/**": {"ros__parameters": {"planner": {"ik": {"alpha_max": 0.3}}}}}),
        ("list.yaml", [1, 2]),
        ("two.yaml", {"a": {"catching": {}}, "b": {"catching": {}}}),
        ("both.yaml", {"catching": {"planner": {}}, "planner": {"ik": {"alpha_max": 0.3}}}),
    ):
        with pytest.raises(ValueError, match="no `catching` tree|root must be a map|ambiguous"):
            cm.resolve_alpha_max(_params_file(tmp_path, doc, name), None)
    with pytest.raises(ValueError, match="must be a number"):
        cm.resolve_alpha_max(
            _params_file(tmp_path, {"planner": {"ik": {"alpha_max": "wide"}}}, "bad.yaml"), None
        )


def test_alpha_max_flag_alone_and_neither_source(tmp_path: Path):
    flag_only = cm.resolve_alpha_max(None, 0.26)
    assert (flag_only.value_rad, flag_only.origin, flag_only.warning) == (0.26, "flag", None)
    # The judge was handed nothing, so it ran on its in-code default: a flag
    # that says otherwise is reported, not silently believed.
    off_default = cm.resolve_alpha_max(None, 0.3)
    assert off_default.value_rad == pytest.approx(0.3)
    assert "in-code default" in off_default.warning

    neither = cm.resolve_alpha_max(None, None)
    assert neither.value_rad == cm.JUDGE_DEFAULT_ALPHA_MAX_RAD == 0.26
    assert neither.origin == "judge_default"
    assert "in-code provisional default" in neither.source
    assert "catch_pose_ik.hpp" in neither.source
    assert neither.as_provenance()["origin"] == "judge_default"

    # "TBD" — and a tree with no planner.ik at all — leave the value open, which
    # is NOT the same as the tree being missing.
    for name, ik in (("tbd.yaml", {"alpha_max": "TBD"}), ("absent.yaml", {"max_iter": 40})):
        path = _params_file(tmp_path, {"catching": {"planner": {"ik": ik}}}, name)
        resolved = cm.resolve_alpha_max(path, None)
        assert (resolved.value_rad, resolved.origin) == (0.26, "judge_default")
        assert str(path) in resolved.source
        assert cm.resolve_alpha_max(path, 0.26).origin == "flag"
    with pytest.raises(ValueError, match="finite and > 0"):
        cm.resolve_alpha_max(None, 0.0)


def test_alpha_max_from_both_sources_must_agree(tmp_path: Path):
    path = _params_file(tmp_path, {"catching": {"planner": {"ik": {"alpha_max": 0.35}}}})
    agreed = cm.resolve_alpha_max(path, 0.35)
    assert (agreed.value_rad, agreed.origin, agreed.flag_value_rad) == (0.35, "params", 0.35)
    # Neither side wins silently: whichever did, the report would be wrong
    # about the other.
    with pytest.raises(ValueError, match="two sources of truth"):
        cm.resolve_alpha_max(path, 0.26)


# ── The CLI, end to end on the stand-in judge ─────────────────────────────────


def _run_cli(tmp_path: Path, judge: Path, params: Path, *extra: str) -> Path:
    config = _shipped_style_config(tmp_path)
    urdf = tmp_path / "tiny.urdf"
    urdf.write_text(TINY_URDF)
    ball = tmp_path / "ball.yaml"
    ball.write_text(
        yaml.safe_dump(
            {
                "sim": {
                    "ros__parameters": {
                        "projectile_ball": {"radius_m": RADIUS_M, "mass_kg": MASS_KG}
                    }
                }
            }
        )
    )
    out_dir = tmp_path / "map"
    argv = [
        *("--robot-config", str(config), "--ball-config", str(ball), "--out-dir", str(out_dir)),
        *("--urdf", str(urdf), "--arm-base-frame", "mount", "--judge", str(judge)),
        *("--params", str(params)),
        *("--drag-coefficient", str(CD_TENNIS), "--drag-coefficient-source", CD_SOURCE),
        *("--air-density", str(RHO_AIR), "--air-density-source", RHO_SOURCE),
        # 3 azimuths x 2 aims = 6 throws. Aim 80 deg misses the base axis by
        # 4 sin(80) = 3.9 m, outside --max-reach-m: those 3 throws produce NO
        # candidate. Of the other 3, the stand-in's verdict follows sign(v_y), so
        # seed 0 (q0 > 0) takes azimuths 30 and 60 and seed 1 takes -30: disjoint.
        *("--azimuths-deg", "-30 30 60", "--aim-deviations-deg", "0 80"),
        *("--release-heights-m", "1.2", "--speeds-m-s", "7", "--elevations-deg", "20"),
        *("--window-s", "0.3", "0.9", "--min-flight-time-s", "0.3", "--max-reach-m", "2.0"),
        *("--seed", "1.0 0.0", "--seed", "-1.0 0.0", "--shard-size", "4", "--no-plots"),
        *extra,
    ]
    assert cm.main(argv) == 0
    return out_dir


def test_cli_headline_is_the_best_single_seed_with_the_union_named_apart(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
):
    pytest.importorskip("pinocchio")
    judge = _fake_judge(tmp_path)
    params = tmp_path / "params.yaml"
    params.write_text("catching:\n  planner:\n    ik:\n      alpha_max: 0.35\nfake_reach: 10.0\n")
    out_dir = _run_cli(tmp_path, judge, params)
    stderr = capsys.readouterr().err

    results = yaml.safe_load((out_dir / "provenance.yaml").read_text())["provenance"]["results"]
    assert results["throws"] == 6
    assert results["headline_seed"]["seed_id"] == 0
    assert results["headline_seed"]["q"] == [1.0, 0.0]
    # 2, not 3: the union over the two wait poses is not something a robot does.
    assert results["accepted_throws"] == 2
    assert results["accepted_throws_any_seed"] == 3
    assert "wait-pose seed 0: accepted 2/6 grid throws" in stderr
    assert "accepted_throws_any_seed = 3/6" in stderr

    # One denominator: the ranking and the headline both divide by the 6-throw
    # grid (the ranking used to divide by the 3 throws that reached the judge).
    assert results["throws_with_candidates"] == 3
    best = results["seed_ranking"][0]
    assert best["seed_id"] == 0
    assert best["accepted_fraction"] == pytest.approx(2 / 6)
    assert best["accepted_fraction"] == pytest.approx(results["accepted_fraction"])
    assert best["denominator"] == results["accepted_fraction_denominator"]

    region = yaml.safe_load((out_dir / "throw_region.yaml").read_text())["sim"]["throw_region"]
    assert region["accepted_throws"] == 2
    assert region["wait_pose_seed_id"] == 0
    assert region["wait_pose_q"] == [1.0, 0.0]
    assert region["accepted_throws_any_seed"] == 3
    assert region["covered_axes_deg"]["azimuth_base_deg"] == pytest.approx([30.0, 60.0])

    with (out_dir / "throw_summary.csv").open(newline="") as handle:
        summary = list(csv.DictReader(handle))
    assert len(summary) == 6, "one row per GRID throw, the empty ones included"
    assert sorted(int(r["candidates"]) == 0 for r in summary) == [False] * 3 + [True] * 3
    assert sum(int(r["accepted"]) for r in summary) == 2
    assert {r["wait_pose_seed_id"] for r in summary} == {"0"}
    assert {r["best_seed_id"] for r in summary if r["accepted"] == "1"} == {"0"}

    # alpha_max is the params file's, in the report AND in the provenance.
    assert results["alpha_max"]["origin"] == "params"
    assert results["theta"]["alpha_max_rad"] == pytest.approx(0.35)
    assert results["theta"]["max_over_alpha_max"] == pytest.approx(0.30 / 0.35)
    judge_block = yaml.safe_load((out_dir / "provenance.yaml").read_text())["provenance"]["judge"]
    assert judge_block["params_sha256"] == hashlib.sha256(params.read_bytes()).hexdigest()

    # Same --out-dir, same params PATH, different content: nothing is within
    # reach any more, and the map has to say so rather than replay the shards.
    params.write_text("catching:\n  planner:\n    ik:\n      alpha_max: 0.35\nfake_reach: 0.0\n")
    calls = _judge_calls(judge)
    _run_cli(tmp_path, judge, params)
    assert _judge_calls(judge) > calls
    results = yaml.safe_load((out_dir / "provenance.yaml").read_text())["provenance"]["results"]
    assert results["accepted_throws"] == 0
    assert results["accepted_throws_any_seed"] == 0
    # ... while an unchanged re-run replays every shard.
    calls = _judge_calls(judge)
    _run_cli(tmp_path, judge, params)
    assert _judge_calls(judge) == calls


def test_alpha_max_is_held_against_the_judges_own_report(tmp_path: Path):
    judge = _fake_judge(tmp_path)
    # A judge that cannot say leaves the resolution alone, and says so.
    assert cm.judge_reported_alpha_max(judge, None) is None
    unchecked = cm.confirm_alpha_max_with_judge(cm.resolve_alpha_max(None, None), None)
    assert unchecked.judge_reported_rad is None
    assert "NOT cross-checked" in unchecked.as_provenance()["judge_report"]

    judge.with_name("print_options.txt").write_text(
        "params_tree catching\nplanner.ik.max_iter 40\nplanner.ik.alpha_max 0.26000000000000001\n"
    )
    reported = cm.judge_reported_alpha_max(judge, None)
    assert reported == 0.26
    confirmed = cm.confirm_alpha_max_with_judge(cm.resolve_alpha_max(None, None), reported)
    assert confirmed.judge_reported_rad == 0.26
    assert "confirmed" in confirmed.as_provenance()["judge_report"]

    # The hole a lone flag leaves: the judge ran on ITS default, whatever the
    # flag says. With the judge's report in hand that is an error, not a warning.
    lone_flag = cm.resolve_alpha_max(None, 0.3)
    assert lone_flag.warning is not None
    with pytest.raises(ValueError, match="the judge reports"):
        cm.confirm_alpha_max_with_judge(lone_flag, reported)


def test_cli_refuses_an_alpha_max_the_judge_does_not_report(tmp_path: Path):
    judge = _fake_judge(tmp_path)
    judge.with_name("print_options.txt").write_text("planner.ik.alpha_max 0.26\n")
    params = tmp_path / "params.yaml"
    params.write_text("catching:\n  planner:\n    ik:\n      alpha_max: 0.35\n")
    with pytest.raises(SystemExit, match="the judge reports"):
        _run_cli(tmp_path, judge, params)
    assert _judge_calls(judge) == 0


def test_cli_refuses_an_alpha_max_flag_that_contradicts_the_params_file(tmp_path: Path):
    judge = _fake_judge(tmp_path)
    params = tmp_path / "params.yaml"
    params.write_text("catching:\n  planner:\n    ik:\n      alpha_max: 0.35\n")
    with pytest.raises(SystemExit, match="two sources of truth"):
        _run_cli(tmp_path, judge, params, "--alpha-max-rad", "0.26")
    assert _judge_calls(judge) == 0, "the contradiction must not cost a sweep to find"


# ── The real judge (skipped cleanly when it is not built) ─────────────────────

# Repo-relative rather than through the ament index: `integrated_bringup`
# exec_depends on rtc_tools, so making it a test_depend here would be a cycle.
SHIPPED_UR5E_P1B = (
    Path(__file__).resolve().parents[2] / "integrated_bringup/config/ur5e_p1b/_base.yaml"
)


def _judge_or_skip() -> Path:
    try:
        return cm.find_judge()
    except SystemExit as exc:
        pytest.skip(f"{cm.JUDGE_EXECUTABLE} is not available: {exc}")


def test_judge_csv_round_trip_on_the_shipped_model(tmp_path: Path):
    """End to end against the REAL judge: translation, --dump-frame, sharded run.

    Mocking the judge here would leave the one thing that cannot be re-derived
    in python — the verdict and its CSV encoding — untested, so this asks the
    binary. It skips with a message when the binary, the shipped config or the
    URDF toolchain is absent rather than passing vacuously.
    """
    if not SHIPPED_UR5E_P1B.is_file():
        pytest.skip(f"shipped robot config is not in this tree: {SHIPPED_UR5E_P1B}")
    judge = _judge_or_skip()
    pytest.importorskip("pinocchio")
    try:
        artifacts = cm.write_model_config([SHIPPED_UR5E_P1B], tmp_path / "model")
    except (SystemExit, OSError, subprocess.CalledProcessError) as exc:
        pytest.skip(f"cannot expand the shipped URDF here: {exc}")

    # The trap this whole sub-model exists for: the shipped arm sub-model stops
    # at the flange, which is upstream of the catch frame's parent.
    assert artifacts.arm_tip_link == "tool0"
    assert artifacts.catch_frame_parent == "l_palm_link"
    assert artifacts.arm_tip_link != artifacts.catch_frame_parent

    seeds_path = tmp_path / "seeds.csv"
    invocation = cm.JudgeInvocation(
        judge=judge,
        model_config=artifacts.model_config_path,
        seeds=seeds_path,
        sub_model=artifacts.catch_sub_model,
        catch_frame=artifacts.catch_frame,
    )
    probe = cm.dump_catch_frame(invocation)
    assert probe["parent"] == "l_palm_link"
    assert probe["nv"] == 6, "the arm_catch sub-model should lock the whole hand"
    assert probe["residual_translation_m"] < 1e-12

    # The judge's MODEL WORLD is the URDF root (`base_link`), which is Rz(180)
    # from the arm base frame (`base`) this robot's CLIK config names. Pinned
    # because feeding world coordinates straight in is silently plausible.
    model_world_t_base = cm.frame_placement_in_model_world(artifacts.urdf_text, "base")
    assert model_world_t_base == pytest.approx(
        cm.make_transform(cm.rotation_z(math.pi), np.zeros(3)), abs=1e-12
    )

    cm.write_seed_csv(seeds_path, [[0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]])
    candidates = [
        # Reachable: converges on a pose (the gate may still reject it).
        cm.JudgeCandidate(
            id=0,
            seed_id=0,
            p_c_model_m=np.array([0.5, 0.2, 0.6]),
            v_model_m_s=np.array([-5.0, 0.0, -2.0]),
            throw_index=0,
            time_s=1.1,
            speed_m_s=5.39,
        ),
        # Zero velocity: `speed_too_low`, which leaves NO pose — the empty-q row.
        cm.JudgeCandidate(
            id=1,
            seed_id=0,
            p_c_model_m=np.array([0.5, 0.2, 0.6]),
            v_model_m_s=np.zeros(3),
            throw_index=1,
            time_s=1.2,
            speed_m_s=0.0,
        ),
        # Far out of reach: `not_converged`, also poseless.
        cm.JudgeCandidate(
            id=2,
            seed_id=0,
            p_c_model_m=np.array([3.0, 0.0, 0.5]),
            v_model_m_s=np.array([-5.0, 0.0, -1.0]),
            throw_index=2,
            time_s=1.3,
            speed_m_s=5.1,
        ),
    ]
    work = tmp_path / "shards"
    # shard_size 2 with 2 workers: two shards, so the sharding path is exercised.
    results = cm.run_judge_batch(invocation, candidates, work_dir=work, shard_size=2, workers=2)
    assert [r.id for r in results] == [0, 1, 2], "results must come back in input order"
    by_id = {r.id: r for r in results}
    assert by_id[1].reason_name == "speed_too_low"
    assert by_id[1].q is None
    assert by_id[2].q is None
    assert by_id[0].q is not None
    assert by_id[0].q.size == 6
    assert by_id[0].nv == 6
    assert math.isfinite(by_id[0].w5) and math.isfinite(by_id[0].w6)

    joined = cm.join_results(candidates, results)
    assert len(joined) == 3
    assert len(cm.summarize_throws(joined)) == 3

    # Resumability is a CHECK: a truncated shard is not accepted as an answer.
    shard_out = sorted(work.glob("shard_*_out.csv"))
    assert len(shard_out) == 2
    first = shard_out[0]
    lines = first.read_text().splitlines()
    first.write_text("\n".join(lines[:-1]) + "\n")
    assert cm.shard_is_complete(first, [0, 1]) is False
    # ... and re-running repairs it rather than reporting a shorter map.
    again = cm.run_judge_batch(invocation, candidates, work_dir=work, shard_size=2, workers=2)
    assert [r.id for r in again] == [0, 1, 2]
    assert [r.reason_name for r in again] == [r.reason_name for r in results]


def test_judge_batch_reports_a_failing_shard_with_its_stderr(tmp_path: Path):
    judge = _judge_or_skip()
    invocation = cm.JudgeInvocation(
        judge=judge,
        model_config=tmp_path / "does_not_exist.yaml",
        seeds=tmp_path / "also_missing.csv",
    )
    candidate = cm.JudgeCandidate(
        id=0, seed_id=0, p_c_model_m=np.zeros(3), v_model_m_s=np.array([-1.0, 0.0, 0.0])
    )
    # A shard that dies must raise WITH its stderr: a silently short shard would
    # read as "those throws were not catchable" and nothing later contradicts it.
    with pytest.raises(RuntimeError, match="stderr"):
        cm.run_judge_batch(invocation, [candidate], work_dir=tmp_path / "w", workers=1)
