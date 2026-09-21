"""catch_speed_budget (dynamic_catching S4.4) on synthetic models.

Oracles are independent of the code under test: closed forms for the LP, the
stroke and the ramp; finite-difference FK for the speed solution; RNEA +
second-order FK for the acceleration LP. The fixture arm is asymmetric on
purpose (link lengths, masses, speed and torque limits all differ per joint)
so that a swapped joint order or a swapped frame cannot give the same number.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

pin = pytest.importorskip("pinocchio")
pytest.importorskip("scipy")

from rtc_tools.analysis import catch_speed_budget as csb  # noqa: E402

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
AXES = ["0 0 1", "0 1 0", "0 1 0", "1 0 0", "0 1 0", "1 0 0"]
LENGTHS = [0.17, 0.41, 0.37, 0.13, 0.11, 0.09]
MASSES = [3.7, 8.1, 2.3, 1.2, 1.1, 0.6]
QD_MAX = np.array([2.0, 1.7, 3.0, 2.6, 3.4, 4.1])
TAU_MAX = np.array([150.0, 140.0, 90.0, 28.0, 25.0, 20.0])
ROTOR = np.array([0.20, 0.18, 0.12, 0.05, 0.04, 0.03])
FRAME = csb.ExtraFrame("tool", (0.015, 0.045, 0.052), (0.3, -0.2, 0.1))
Q_GENERIC = np.array([0.3, -0.9, 1.1, 0.4, -0.7, 0.2])


def arm_urdf() -> str:
    links = ['  <link name="root"/>', '  <link name="base"/>']
    joints = [
        '  <joint name="mount" type="fixed"><parent link="root"/><child link="base"/>'
        '<origin xyz="0 0 0" rpy="0 0 3.141592653589793"/></joint>'
    ]
    parent = "base"
    for i, (name, axis, length, mass) in enumerate(
        zip(JOINTS, AXES, LENGTHS, MASSES, strict=True)
    ):
        child = f"l{i + 1}" if i < len(JOINTS) - 1 else "tool"
        links.append(
            f'  <link name="{child}"><inertial><origin xyz="{length / 2} 0.01 0"/>'
            f'<mass value="{mass}"/><inertia ixx="0.01" iyy="0.02" izz="0.015" ixy="0" ixz="0" '
            'iyz="0"/></inertial></link>'
        )
        origin = LENGTHS[i - 1] if i else 0.0
        joints.append(
            f'  <joint name="{name}" type="revolute"><parent link="{parent}"/>'
            f'<child link="{child}"/><origin xyz="{origin} 0 0.02"/><axis xyz="{axis}"/>'
            f'<limit lower="-3" upper="3" effort="{TAU_MAX[i]}" velocity="{QD_MAX[i]}"/></joint>'
        )
        parent = child
    return (
        '<?xml version="1.0"?>\n<robot name="fixture">\n'
        + "\n".join(links + joints)
        + "\n</robot>\n"
    )


@pytest.fixture(scope="module")
def arm() -> csb.ArmKinematics:
    return csb.ArmKinematics(arm_urdf(), JOINTS, FRAME, ROTOR)


def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


# ── LP / DLS ──────────────────────────────────────────────────────────────────


def test_lp_matches_the_two_joint_closed_form():
    # Two joints, no approach-axis rows: J_p q̇ = s v̂ has the single solution q̇ = s J⁺ v̂,
    # so s = min_i q̇_max,i / |(J⁺ v̂)_i|. Lengths and limits differ so the binding joint matters.
    jp = np.array([[-0.31, -0.12], [0.52, 0.27], [0.0, 0.0]])
    jw = np.zeros((2, 2))
    v_hat = unit([0.6, 0.8, 0.0])
    qd_max = np.array([1.3, 2.9])
    unit_rate = np.linalg.lstsq(jp, v_hat, rcond=None)[0]
    expected = float(np.min(qd_max / np.abs(unit_rate)))
    got, qd = csb.directional_speed_lp(jp, jw, v_hat, qd_max)
    assert got.v_dir_max == pytest.approx(expected, rel=1e-9)
    assert jp @ qd == pytest.approx(expected * v_hat, abs=1e-9)
    # swapping the limits must change the answer — otherwise the test cannot see joint order
    swapped, _ = csb.directional_speed_lp(jp, jw, v_hat, qd_max[::-1])
    assert abs(swapped.v_dir_max - expected) > 1e-3


@pytest.mark.parametrize("n", [6, 7])
def test_minimum_norm_speed_never_exceeds_the_lp(n):
    rng = np.random.default_rng(n)
    ratios = []
    for _ in range(120):
        jp, jw = rng.normal(size=(3, n)), rng.normal(size=(2, n))
        v_hat = unit(rng.normal(size=3))
        qd_max = rng.uniform(1.0, 4.0, size=n)
        lp, _ = csb.directional_speed_lp(jp, jw, v_hat, qd_max)
        dls = csb.directional_speed_dls(jp, jw, v_hat, qd_max)
        assert dls.v_dir_max <= lp.v_dir_max * (1 + 1e-6) + 1e-9
        ratios.append(dls.v_dir_max / lp.v_dir_max)
    # the redundant arm leaves real speed on the table; the square one barely does
    assert min(ratios) < (0.9 if n == 7 else 1.0 + 1e-6)


def test_dls_numerator_is_the_projection_on_v_hat_not_the_norm():
    # With heavy damping the DLS solution does not reach [v̂; 0]: part of what it achieves points
    # off v̂. L3 §4.5 counts only the v̂ component — the norm would report speed the ball never sees.
    rng = np.random.default_rng(3)
    jp, jw = rng.normal(size=(3, 6)), rng.normal(size=(2, 6))
    v_hat = unit([0.3, -0.5, 0.8])
    damping = 1.0
    j5 = np.vstack([jp, jw])
    qd_unit = j5.T @ np.linalg.solve(j5 @ j5.T + damping**2 * np.eye(5), [*v_hat, 0.0, 0.0])
    achieved = jp @ qd_unit
    denom = np.max(np.abs(qd_unit) / QD_MAX)
    got = csb.directional_speed_dls(jp, jw, v_hat, QD_MAX, damping=damping)
    assert got.v_dir_max == pytest.approx(float(v_hat @ achieved) / denom, rel=1e-12)
    assert got.v_dir_max < 0.99 * float(np.linalg.norm(achieved)) / denom


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_speed_limit_is_flagged_and_reports_zero(bad):
    rng = np.random.default_rng(1)
    jp, jw, v_hat = rng.normal(size=(3, 6)), rng.normal(size=(2, 6)), unit([1.0, 2.0, -1.0])
    qd_max = QD_MAX.copy()
    qd_max[2] = bad
    lp, qd = csb.directional_speed_lp(jp, jw, v_hat, qd_max)
    dls = csb.directional_speed_dls(jp, jw, v_hat, qd_max)
    assert lp.limits_invalid and lp.v_dir_max == 0.0 and not np.any(qd)
    assert dls.limits_invalid and dls.v_dir_max == 0.0


def test_non_unit_direction_is_an_input_error():
    rng = np.random.default_rng(2)
    jp, jw = rng.normal(size=(3, 6)), rng.normal(size=(2, 6))
    lp, _ = csb.directional_speed_lp(jp, jw, np.array([2.0, 0.0, 0.0]), QD_MAX)
    assert lp.input_invalid and lp.v_dir_max == 0.0


# ── Model integration ─────────────────────────────────────────────────────────


def test_speed_solution_moves_the_frame_along_v_hat_with_the_axis_held(arm):
    v_hat = unit([0.5, -0.3, -0.8])
    t = arm.terms(Q_GENERIC, np.zeros(6))
    lp, qd = csb.directional_speed_lp(t["jp"], t["jw"], v_hat, QD_MAX)
    assert lp.v_dir_max > 0.05
    dt = 1e-6
    p0, z0 = arm.frame_position(Q_GENERIC), arm.frame_axis(Q_GENERIC)
    p1, z1 = arm.frame_position(Q_GENERIC + qd * dt), arm.frame_axis(Q_GENERIC + qd * dt)
    assert (p1 - p0) / dt == pytest.approx(lp.v_dir_max * v_hat, abs=1e-4)
    assert np.linalg.norm(z1 - z0) / dt < 1e-4  # approach axis does not turn
    assert np.max(np.abs(qd) / QD_MAX) == pytest.approx(1.0, abs=1e-9)  # a limit is active


def test_direction_given_in_the_wrong_frame_gives_a_different_answer(arm):
    # The fixture mounts the arm base half a turn from the model root, the trap plan §11 records:
    # a direction expressed in the base frame is NOT the same input as one in the model world.
    v_hat = unit([0.5, -0.3, -0.8])
    flipped = np.array([-v_hat[0], -v_hat[1], v_hat[2]])
    t = arm.terms(Q_GENERIC, np.zeros(6))
    right, _ = csb.directional_speed_lp(t["jp"], t["jw"], v_hat, QD_MAX)
    wrong, _ = csb.directional_speed_lp(t["jp"], t["jw"], flipped, QD_MAX)
    assert abs(right.v_dir_max - wrong.v_dir_max) > 0.02


def test_per_joint_vectors_follow_the_given_joint_names_not_the_model_order():
    perm = [3, 0, 5, 1, 4, 2]
    names = [JOINTS[i] for i in perm]
    shuffled = csb.ArmKinematics(arm_urdf(), names, FRAME, ROTOR[perm])
    straight = csb.ArmKinematics(arm_urdf(), JOINTS, FRAME, ROTOR)
    v_hat = unit([0.2, 0.7, -0.4])
    a = straight.terms(Q_GENERIC, np.zeros(6))
    b = shuffled.terms(Q_GENERIC[perm], np.zeros(6))
    ref, _ = csb.directional_speed_lp(a["jp"], a["jw"], v_hat, QD_MAX)
    same, _ = csb.directional_speed_lp(b["jp"], b["jw"], v_hat, QD_MAX[perm])
    assert same.v_dir_max == pytest.approx(ref.v_dir_max, rel=1e-9)
    # limits left in MODEL order against permuted names attach to the wrong joints
    mixed, _ = csb.directional_speed_lp(b["jp"], b["jw"], v_hat, QD_MAX)
    assert abs(mixed.v_dir_max - ref.v_dir_max) > 1e-3
    assert shuffled.model_velocity_limits() == pytest.approx(QD_MAX[perm])


def test_accel_lp_is_reproduced_by_inverse_dynamics(arm):
    v_hat = unit([0.4, 0.1, -0.9])
    rest = arm.terms(Q_GENERIC, np.zeros(6))
    _, qd_dir = csb.directional_speed_lp(rest["jp"], rest["jw"], v_hat, QD_MAX)
    tau_limit = 0.8 * TAU_MAX
    for scale, direction in ((0.0, v_hat), (0.6, v_hat), (0.6, -v_hat)):
        qd = scale * qd_dir
        t = arm.terms(Q_GENERIC, qd)
        a, qdd = csb.directional_accel_lp(
            t["mass"], t["bias"], t["jp"], t["jw"], t["drift_linear"], t["drift_angular_xy"],
            direction, tau_limit,
        )  # fmt: skip
        tau, lin = arm.inverse_dynamics(Q_GENERIC, qd, qdd)
        assert np.all(np.abs(tau) <= tau_limit + 1e-6)
        assert np.max(np.abs(tau) / tau_limit) == pytest.approx(1.0, abs=1e-6)  # tight somewhere
        assert lin == pytest.approx(a * direction, abs=1e-6)
        assert a > 1.0


def test_rotor_inertia_lowers_the_acceleration(arm):
    bare = csb.ArmKinematics(arm_urdf(), JOINTS, FRAME, np.zeros(6))
    v_hat = unit([0.4, 0.1, -0.9])
    out = []
    for model in (bare, arm):
        t = model.terms(Q_GENERIC, np.zeros(6))
        out.append(
            csb.directional_accel_lp(
                t["mass"], t["bias"], t["jp"], t["jw"], t["drift_linear"], t["drift_angular_xy"],
                v_hat, 0.8 * TAU_MAX,
            )[0]
        )  # fmt: skip
    assert out[1] < 0.9 * out[0]


def test_accel_lp_reports_nan_when_gravity_alone_exceeds_the_limit(arm):
    t = arm.terms(Q_GENERIC, np.zeros(6))
    a, qdd = csb.directional_accel_lp(
        t["mass"], t["bias"], t["jp"], t["jw"], t["drift_linear"], t["drift_angular_xy"],
        unit([0.0, 0.0, 1.0]), np.full(6, 1e-3),
    )  # fmt: skip
    assert math.isnan(a) and not np.any(qdd)


# ── Stroke, ramp, lead ────────────────────────────────────────────────────────


def test_stroke_is_the_chord_to_the_sphere_or_the_floor():
    centre = np.array([0.1, -0.2, 0.3])
    p = centre + np.array([0.3, 0.0, 0.0])
    assert csb.stroke_to_boundary(p, unit([1, 0, 0]), centre, 1.0, 5.0) == pytest.approx(0.7)
    assert csb.stroke_to_boundary(p, unit([-1, 0, 0]), centre, 1.0, 5.0) == pytest.approx(1.3)
    # straight down: the floor comes first
    assert csb.stroke_to_boundary(p, unit([0, 0, -1]), centre, 1.0, 0.25) == pytest.approx(0.25)
    # already outside the sphere, or below the floor
    outside = centre + np.array([1.5, 0.0, 0.0])
    assert csb.stroke_to_boundary(outside, unit([0, 1, 0]), centre, 1.0, 5.0) == 0.0
    assert csb.stroke_to_boundary(p, unit([1, 0, 0]), centre, 1.0, -0.01) == 0.0


def test_ramp_limit_matches_constant_acceleration_kinematics():
    speeds = np.linspace(0.0, 4.0, 401)
    a, d = np.full(401, 8.0), np.full(401, 5.0)
    big = 1e9
    got = csb.ramp_speed_limit(speeds, a, d, 0.25, big, big, 4.0)
    assert got == pytest.approx(math.sqrt(2 * 8.0 * 0.25), abs=0.011)  # stroke before
    got = csb.ramp_speed_limit(speeds, a, d, big, 0.4, big, 4.0)
    assert got == pytest.approx(math.sqrt(2 * 5.0 * 0.4), abs=0.011)  # braking stroke
    got = csb.ramp_speed_limit(speeds, a, d, big, big, 0.2, 4.0)
    assert got == pytest.approx(8.0 * 0.2, abs=0.011)  # time
    assert csb.ramp_speed_limit(speeds, a, d, big, big, big, 1.234) == pytest.approx(1.234)
    # an acceleration that turns non-positive ends the ramp where it does
    fading = np.where(speeds < 1.0, 8.0, -1.0)
    # (the trapezoid still counts the half-step whose mean acceleration is positive)
    assert 0.98 <= csb.ramp_speed_limit(speeds, fading, d, big, big, big, 4.0) <= 1.0 + 1e-9


def test_min_flight_time_counts_the_closing_time_and_window_is_inclusive():
    assert csb.min_flight_time(0.10, 0.14, 0.2815, 0.05, 0.02) == pytest.approx(0.5915)
    assert csb.window_open(2.4, 1.5, 1.0, 0.1)
    assert not csb.window_open(2.41, 1.5, 1.0, 0.1)


# ── Aggregation ───────────────────────────────────────────────────────────────


def test_cell_table_keeps_throws_that_never_reached_the_judge():
    throws = {
        i: {"throw_index": str(i), "release_height_m": "1.0", "distance_m": "1.5"}
        for i in range(5)
    }
    throws[5] = {"throw_index": "5", "release_height_m": "1.2", "distance_m": "1.5"}
    rows = [
        {"throw_index": 0, "t_c_s": 0.6, "speed_m_s": 2.0, "v_arm": 1.2},  # opens at 1.0
        {"throw_index": 0, "t_c_s": 0.7, "speed_m_s": 2.0, "v_arm": 1.2},  # same throw, once
        {"throw_index": 1, "t_c_s": 0.3, "speed_m_s": 1.0, "v_arm": 1.2},  # too early
        {"throw_index": 2, "t_c_s": 0.6, "speed_m_s": 4.0, "v_arm": 1.2},  # too fast
    ]
    table = csb.cell_table(rows, throws, [1.0], 0.1, 0.5)
    by_cell = {(r["release_height_m"], r["distance_m"]): r for r in table}
    assert by_cell[(1.0, 1.5)] == {
        "release_height_m": 1.0, "distance_m": 1.5, "grid_throws": 5,
        "kinematic_throws": 3, "open_throws@1": 1,
    }  # fmt: skip
    assert by_cell[(1.2, 1.5)]["grid_throws"] == 1 and by_cell[(1.2, 1.5)]["kinematic_throws"] == 0


def test_drop_table_bins_by_drop_and_filters_by_flight_time():
    rows = [
        {"drop_m": -0.6, "t_c_s": 0.6, "speed_m_s": 3.0, "v_arm": 1.0},
        {"drop_m": -0.1, "t_c_s": 0.6, "speed_m_s": 2.0, "v_arm": 1.5},
        {"drop_m": -0.1, "t_c_s": 0.6, "speed_m_s": 3.4, "v_arm": 1.5},
        {"drop_m": -0.1, "t_c_s": 0.2, "speed_m_s": 0.5, "v_arm": 1.5},  # filtered
    ]
    table = csb.drop_table(rows, [-1.0, -0.25, 0.25], [1.0, 2.0], 0.1, 0.5)
    assert [r["candidates"] for r in table] == [1, 2]
    assert table[0]["open_share@1"] == 0.0 and table[0]["need_min_m_s"] == pytest.approx(2.1)
    assert table[1]["open_share@1"] == 0.5 and table[1]["open_share@2"] == 1.0


# ── CLI end to end ────────────────────────────────────────────────────────────


def _write_run(tmp_path: Path, arm: csb.ArmKinematics, *, shift_m: float = 0.0) -> list[str]:
    urdf = tmp_path / "fixture.urdf"
    urdf.write_text(arm_urdf())
    config = tmp_path / "robot.yaml"
    config.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "urdf": {
                            "extra_frames": {
                                "catch_frame": {
                                    "parent": FRAME.parent,
                                    "xyz": list(FRAME.xyz),
                                    "rpy": list(FRAME.rpy),
                                }
                            }
                        },
                        "devices": {
                            "arm": {
                                "joint_state_names": JOINTS,
                                "joint_limits": {
                                    "max_torque": TAU_MAX.tolist(),
                                    "max_velocity": (0.5 * QD_MAX).tolist(),
                                },
                            }
                        },
                    }
                }
            }
        )
    )
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    rng = np.random.default_rng(7)
    header = ["id", "throw_index", "t_c_s", "p_world_z", "accepted", "nv"]
    header += [f"p_model_{a}" for a in "xyz"] + [f"v_model_{a}" for a in "xyz"]
    header += [f"q{i}" for i in range(6)]
    with (map_dir / "candidates.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for i in range(4):
            q = Q_GENERIC + rng.uniform(-0.2, 0.2, size=6)
            p = arm.frame_position(q) + np.array([shift_m, 0.0, 0.0])
            v = -2.5 * arm.frame_axis(q)
            writer.writerow([i, i % 2, 0.7, p[2] + 0.9, "1", 6, *p, *v, *q])
        writer.writerow([9, 1, 0.7, 1.0, "0", 6, 0, 0, 0, 1, 0, 0, *np.zeros(6)])  # rejected
    with (map_dir / "throw_summary.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["throw_index", "release_height_m", "distance_m"])
        writer.writerows([[0, 1.0, 1.5], [1, 1.0, 1.5], [2, 1.0, 1.5]])
    return [
        "--robot-config", str(config), "--group", "arm", "--map-dir", str(map_dir),
        "--out-dir", str(tmp_path / "out"), "--urdf", str(urdf), "--velocity-source", "model",
        "--eta-v", "0.9", "--eta-tau", "0.8", "--rotor-inertia", " ".join(map(str, ROTOR)),
        "--rotor-inertia-source", "test fixture", "--arm-base-frame", "base",
        "--max-reach-m", "1.6", "--floor-world-z-m", "0.0", "--detection-s", "0.10",
        "--latency-s", "0.14", "--close-total-s", "0.10", "--arm-delay-s", "0.05",
        "--time-margin-s", "0.02", "--relative-speed-m-s", "0.5 5.0",
    ]  # fmt: skip


def test_cli_writes_tables_with_full_grid_denominators(tmp_path, arm):
    assert csb.main(_write_run(tmp_path, arm)) == 0
    summary = yaml.safe_load((tmp_path / "out" / "speed_budget_summary.yaml").read_text())
    assert summary["accepted_candidates"] == 4 and summary["grid_throws"] == 3
    assert summary["qd_max"] == pytest.approx(QD_MAX.tolist())  # "model" = the URDF rating
    assert summary["fk_residual_max_m"] < 1e-9
    assert summary["min_flight_time_s"] == pytest.approx(0.41)
    with (tmp_path / "out" / "cell_table.csv").open() as handle:
        (cell,) = list(csv.DictReader(handle))
    assert cell["grid_throws"] == "3" and cell["kinematic_throws"] == "2"
    assert int(cell["open_throws@0.5"]) <= int(cell["open_throws@5"]) == 2


def test_cli_config_velocity_source_reads_the_robot_config(tmp_path, arm):
    argv = _write_run(tmp_path, arm)
    argv[argv.index("--velocity-source") + 1] = "config"
    assert csb.main(argv) == 0
    summary = yaml.safe_load((tmp_path / "out" / "speed_budget_summary.yaml").read_text())
    assert summary["qd_max"] == pytest.approx((0.5 * QD_MAX).tolist())


def test_cli_refuses_a_map_whose_poses_do_not_match_this_model(tmp_path, arm):
    with pytest.raises(SystemExit, match="disagree about joint order"):
        csb.main(_write_run(tmp_path, arm, shift_m=0.02))
