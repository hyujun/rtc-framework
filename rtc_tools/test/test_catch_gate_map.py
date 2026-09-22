"""catch_gate_map (dynamic_catching S3.5b).

The gate verdicts themselves belong to ``catch_gate_batch`` (C++, tested in
rtc_controllers). Pinned here: the torque-checked reach time against closed
forms, that each constant reaches the gate it names, the layering of reasons,
the workspace bound, the denominators, and the CLI end to end THROUGH the real
binary — rtc_tools exec-depends on rtc_controllers, so its absence is a failure
of the build, not a reason to skip.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

pytest.importorskip("pinocchio")
pytest.importorskip("scipy")

sys.path.insert(0, str(Path(__file__).parent))

import test_catch_speed_budget as base  # noqa: E402

from rtc_tools.analysis import (
    catch_gate_map as cgm,  # noqa: E402
    catch_speed_budget as csb,  # noqa: E402
)

QDD_BOX = np.array([60.0, 50.0, 70.0, 90.0, 80.0, 100.0])
BIG_TAU = np.full(6, 1e6)
BIG_QD = np.full(6, 1e3)


@pytest.fixture(scope="module")
def arm() -> csb.ArmKinematics:
    return csb.ArmKinematics(base.arm_urdf(), base.JOINTS, base.FRAME, base.ROTOR)


def pan_only(arm, distance):
    """A move of joint 0 alone: its axis is vertical, so gravity does no work on it and
    M₀₀ is constant along the move — the one case with a closed form."""
    q0 = base.Q_GENERIC.copy()
    q1 = q0.copy()
    q1[0] += distance
    inertia = arm.terms(q0, np.zeros(6))["mass"][0, 0]
    return q0, q1, inertia


# ── path profile ──────────────────────────────────────────────────────────────


def test_profile_duration_is_triangle_below_the_cap_and_trapezoid_above():
    assert cgm.profile_duration(4.0, 10.0) == pytest.approx(1.0)  # 2/√a
    assert cgm.profile_duration(4.0, 1.0) == pytest.approx(1.0 / 1.0 + 1.0 / 4.0)


@pytest.mark.parametrize("cap", [10.0, 1.0])
def test_profile_samples_cover_the_unit_path_and_respect_the_cap(cap):
    samples = cgm.profile_samples(4.0, cap, 8)
    s = np.array([x[0] for x in samples])
    sd = np.array([x[1] for x in samples])
    assert s.min() == pytest.approx(0.0) and s.max() == pytest.approx(1.0)
    assert sd.max() == pytest.approx(min(2.0, cap))
    assert {x[2] for x in samples} >= {4.0, -4.0}
    assert (0.0 in {x[2] for x in samples}) == (cap < 2.0)  # a cruise phase only when capped


# ── torque-checked reach time ─────────────────────────────────────────────────


def test_torque_reach_time_matches_the_single_joint_closed_form(arm):
    q0, q1, inertia = pan_only(arm, 0.6)
    tau = BIG_TAU.copy()
    tau[0] = 40.0
    a_max = tau[0] / inertia
    got = cgm.torque_reach_time(arm, q0, q1, BIG_QD, tau)
    assert got == pytest.approx(2.0 * math.sqrt(0.6 / a_max), rel=1e-3)


def test_torque_reach_time_honours_the_speed_limit(arm):
    q0, q1, inertia = pan_only(arm, 0.6)
    tau = BIG_TAU.copy()
    tau[0] = 40.0
    a_max = tau[0] / inertia
    qd = BIG_QD.copy()
    qd[0] = 0.5 * math.sqrt(a_max * 0.6)  # half the triangle's peak speed
    got = cgm.torque_reach_time(arm, q0, q1, qd, tau)
    assert got == pytest.approx(0.6 / qd[0] + qd[0] / a_max, rel=1e-3)


def test_torque_reach_time_uses_the_limit_of_the_joint_that_moves(arm):
    # Per-joint limits and inertias all differ in the fixture: tightening a joint
    # that does not move (and is not loaded) must not change the answer, while
    # tightening the mover must.
    q0, q1, _ = pan_only(arm, 0.6)
    tau = BIG_TAU.copy()
    tau[0] = 40.0
    reference = cgm.torque_reach_time(arm, q0, q1, BIG_QD, tau)
    slow_wrist = BIG_QD.copy()
    slow_wrist[5] = 1e-3
    assert cgm.torque_reach_time(arm, q0, q1, slow_wrist, tau) == pytest.approx(reference)
    tight = tau.copy()
    tight[0] = 10.0
    assert cgm.torque_reach_time(arm, q0, q1, BIG_QD, tight) == pytest.approx(
        2.0 * reference, rel=2e-3
    )


def test_rotor_inertia_lengthens_the_move(arm):
    bare = csb.ArmKinematics(base.arm_urdf(), base.JOINTS, base.FRAME, np.zeros(6))
    q0, q1, _ = pan_only(arm, 0.6)
    tau = BIG_TAU.copy()
    tau[0] = 40.0
    with_rotor = cgm.torque_reach_time(arm, q0, q1, BIG_QD, tau)
    without = cgm.torque_reach_time(bare, q0, q1, BIG_QD, tau)
    ratio = arm.terms(q0, np.zeros(6))["mass"][0, 0] / bare.terms(q0, np.zeros(6))["mass"][0, 0]
    assert with_rotor == pytest.approx(without * math.sqrt(ratio), rel=2e-3)


def test_the_found_move_really_is_inside_the_limits_and_a_faster_one_is_not(arm):
    q0 = base.Q_GENERIC.copy()
    q1 = q0 + np.array([0.3, -0.25, 0.4, 0.2, -0.3, 0.5])
    tau_limit = 0.8 * base.TAU_MAX
    qd_limit = 0.9 * base.QD_MAX
    t = cgm.torque_reach_time(arm, q0, q1, qd_limit, tau_limit)
    assert math.isfinite(t) and t > 0.0
    delta = q1 - q0
    cap = float(np.min(qd_limit / np.abs(delta)))

    def worst(path_accel):
        ratios = []
        for s, sd, sdd in cgm.profile_samples(path_accel, cap, 64):
            tau = arm.joint_torques(q0 + s * delta, sd * delta, sdd * delta)
            ratios.append(np.max(np.abs(tau) / tau_limit))
            assert np.all(np.abs(sd * delta) <= qd_limit * (1 + 1e-9))
        return max(ratios)

    # invert T → path accel on whichever branch T is on
    accel = 4.0 / t**2 if 2.0 / t <= cap else cap / (t - 1.0 / cap)
    assert worst(accel) <= 1.0 + 1e-3  # denser sampling than the solver's own
    if 2.0 / t <= cap:
        assert worst(1.3 * accel) > 1.0  # torque, not speed, is what binds


def test_gate_inputs_refuse_an_accepted_row_without_a_velocity(arm):
    q = base.Q_GENERIC
    row = {"id": "42", "nv": "6", **{f"q{i}": repr(float(x)) for i, x in enumerate(q)}}
    row.update({f"v_model_{a}": "0.0" for a in "xyz"})
    row.update(
        {f"p_model_{a}": repr(float(x)) for a, x in zip("xyz", arm.frame_position(q), strict=True)}
    )
    with pytest.raises(SystemExit, match="42"):
        cgm.gate_inputs(arm, row, 0.9 * base.QD_MAX, damping=1e-3)


def test_torque_reach_time_is_nan_when_gravity_alone_is_over_the_limit(arm):
    q0 = base.Q_GENERIC.copy()
    q1 = q0 + 0.2
    assert math.isnan(cgm.torque_reach_time(arm, q0, q1, BIG_QD, np.full(6, 1e-3)))
    assert cgm.torque_reach_time(arm, q0, q0, BIG_QD, BIG_TAU) == 0.0


# ── judge inputs ──────────────────────────────────────────────────────────────


def test_gate_inputs_hand_over_what_the_dls_velocity_achieves_not_what_was_asked(arm):
    # With heavy damping q̇ᵘ falls well short of unit speed. The judge's numerator is
    # v̂ᵀ(J_p q̇ᵘ): handing it v̂ instead would report a speed the arm cannot reach (L3 §4.5).
    q = base.Q_GENERIC
    v = -2.5 * arm.frame_axis(q)
    row = {"nv": "6", **{f"q{i}": repr(float(x)) for i, x in enumerate(q)}}
    row.update({f"v_model_{a}": repr(float(x)) for a, x in zip("xyz", v, strict=True)})
    row.update(
        {f"p_model_{a}": repr(float(x)) for a, x in zip("xyz", arm.frame_position(q), strict=True)}
    )
    item = cgm.gate_inputs(arm, row, 0.9 * base.QD_MAX, damping=0.5)
    terms = arm.terms(q, np.zeros(6))
    assert item.jp_qd_unit == pytest.approx(terms["jp"] @ item.qd_unit)
    assert float(base.unit(v) @ item.jp_qd_unit) < 0.9
    assert item.fk_residual_m < 1e-12


# ── workspace bound, reasons, aggregation ─────────────────────────────────────


def test_stop_bound_is_the_reach_sphere_and_the_floor_in_world_height():
    centre = np.array([0.0, 0.0, 0.3])
    inside = {"reach_centre": centre, "reach_m": 1.0, "floor_world_z_m": 0.1}
    # catch point: model z 0.5 is world z 0.9 (model world sits 0.4 m below world)
    assert cgm.stop_inside_workspace(np.array([0.5, 0.0, 0.5]), 0.5, 0.9, **inside)
    assert not cgm.stop_inside_workspace(np.array([1.2, 0.0, 0.5]), 0.5, 0.9, **inside)
    # 0.85 m below the catch point is world z 0.05: inside the sphere, under the floor
    assert not cgm.stop_inside_workspace(np.array([0.0, 0.0, -0.35]), 0.5, 0.9, **inside)
    assert not cgm.stop_inside_workspace(np.array([np.nan, 0.0, 0.5]), 0.5, 0.9, **inside)


def judged(reason="none", gamma_ok=True, stop_valid=True, invalid=False, undetermined=False):
    return {
        "reason_name": reason,
        "gamma_ok": "1" if gamma_ok else "0",
        "stop_gmin_valid": "1" if stop_valid else "0",
        "window_input_invalid": "1" if invalid else "0",
        "dir_limits_invalid": "0",
        "dir_input_invalid": "0",
        "dir_undetermined": "1" if undetermined else "0",
    }


def test_layers_differ_only_in_the_reach_gate():
    late = judged("reach_time")
    assert cgm.layer_reasons(late, True, 0.2, 0.3) == {"box": "reach_time", "torque": "none"}
    assert cgm.layer_reasons(late, True, 0.4, 0.3)["torque"] == "reach_time_torque"
    assert cgm.layer_reasons(late, True, math.nan, 0.3)["torque"] == "reach_torque_infeasible"
    on_time = judged()
    assert cgm.layer_reasons(on_time, True, 0.4, 0.3) == {
        "box": "none",
        "torque": "reach_time_torque",
    }


def test_reasons_follow_planner_order_after_the_reach_gate():
    # the torque layer passes reach here, so it must find the γ gate the judge
    # never reported as the FIRST failure
    late_and_fast = judged("reach_time", gamma_ok=False)
    assert cgm.layer_reasons(late_and_fast, True, 0.1, 0.3) == {
        "box": "reach_time",
        "torque": "gamma_window_empty",
    }
    assert cgm.layer_reasons(judged(gamma_ok=False, invalid=True), True, 0.1, 0.3)["box"] == (
        "gamma_invalid"
    )
    # q̇ᵘ = 0 is "speed not determinable", never an empty window
    undetermined = judged(gamma_ok=False, undetermined=True)
    assert cgm.layer_reasons(undetermined, True, 0.1, 0.3)["torque"] == "gamma_invalid"
    assert cgm.layer_reasons(judged(), False, 0.1, 0.3) == {
        "box": "stop_outside_workspace",
        "torque": "stop_outside_workspace",
    }
    assert cgm.layer_reasons(judged(stop_valid=False), False, 0.1, 0.3)["box"] == "stop_invalid"
    assert cgm.layer_reasons(judged("reach_invalid"), True, 0.1, 0.3)["box"] == "reach_invalid"
    # the commit lead comes first and is the same in both layers
    assert cgm.layer_reasons(judged(), True, 0.1, 0.3, commit_ok=False) == {
        "box": "commit_lead",
        "torque": "commit_lead",
    }


def test_wait_pose_proposal_is_the_per_joint_midrange():
    poses = [np.array([0.0, 1.0, -2.0]), np.array([1.0, 1.0, 0.0]), np.array([0.2, 3.0, -1.0])]
    assert cgm.propose_wait_pose(poses) == pytest.approx([0.5, 2.0, -1.0])
    assert cgm.propose_wait_pose([]) is None


def test_cell_table_keeps_the_full_grid_in_the_denominator():
    throws = {
        0: {"release_height_m": "0.2", "distance_m": "1.0"},
        1: {"release_height_m": "0.2", "distance_m": "1.0"},
        2: {"release_height_m": "0.5", "distance_m": "1.0"},  # never reached the judge
    }
    rows = [
        {"throw_index": 0, "reason_box": "reach_time", "reason_torque": "none"},
        {"throw_index": 0, "reason_box": "reach_time", "reason_torque": "none"},
        {
            "throw_index": 1,
            "reason_box": "gamma_window_empty",
            "reason_torque": "reach_time_torque",
        },
    ]
    low, high = cgm.cell_table(rows, throws)
    assert (low["grid_throws"], low["kinematic_throws"]) == (2, 2)
    assert (low["open_throws_box"], low["open_throws_torque"]) == (0, 1)  # throws, not candidates
    assert (high["grid_throws"], high["kinematic_throws"], high["open_throws_torque"]) == (1, 0, 0)


# ── CLI, through the real judge ───────────────────────────────────────────────


def _write_run(
    tmp_path: Path, arm: csb.ArmKinematics, box_scale: float = 1.0, **overrides
) -> list[str]:
    argv = base._write_run(tmp_path, arm)  # noqa: SLF001 — same fixture map on purpose
    map_dir = tmp_path / "map"
    wait = base.Q_GENERIC + 0.05
    (map_dir / "seeds.csv").write_text("seed_id," + ",".join(f"q{i}" for i in range(6)) + "\n")
    with (map_dir / "seeds.csv").open("a") as handle:
        handle.write("0," + ",".join(repr(float(x)) for x in wait) + "\n")
    # the speed-budget fixture predates seed_id / speed_m_s: add them
    with (map_dir / "candidates.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row["seed_id"] = "0"
        row["speed_m_s"] = repr(float(np.linalg.norm([float(row[f"v_model_{a}"]) for a in "xyz"])))
    with (map_dir / "candidates.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    limits = tmp_path / "limits.yaml"
    limits.write_text(
        yaml.safe_dump(
            {"derived_accel_limits": {"arm": {"qdd_max": (box_scale * QDD_BOX).tolist()}}}
        )
    )
    keep = {"--robot-config", "--group", "--map-dir", "--out-dir", "--urdf", "--velocity-source",
            "--eta-v", "--eta-tau", "--rotor-inertia", "--rotor-inertia-source",
            "--arm-base-frame", "--max-reach-m", "--floor-world-z-m", "--detection-s",
            "--latency-s", "--close-total-s", "--arm-delay-s"}  # fmt: skip
    pairs = dict(zip(argv[0::2], argv[1::2], strict=True))
    pairs = {k: v for k, v in pairs.items() if k in keep}
    pairs.update(
        {
            "--accel-limits": str(limits),
            "--v-max-m-s": "derived",
            "--d-eff-m": "0.2",
            "--d-eff-source": "test",
            "--gamma-margin-m-s": "0.1",
            "--a-dec-m-s2": "8.0",
            "--a-dec-source": "test",
            "--time-margin-s": "0.03",
        }
    )
    pairs.update(overrides)
    return [token for pair in pairs.items() for token in pair]


def _summary(tmp_path: Path) -> dict:
    return yaml.safe_load((tmp_path / "out" / "gate_map_summary.yaml").read_text())


def test_cli_runs_the_real_judge_and_reports_both_layers(tmp_path, arm):
    assert cgm.main(_write_run(tmp_path, arm)) == 0
    summary = _summary(tmp_path)
    assert summary["kinematic_candidates"] == 4 and summary["grid_throws"] == 3
    assert summary["fk_residual_max_m"] < 1e-9
    assert summary["first_plan_s"] == pytest.approx(0.24)
    assert set(summary["open_throws"]) == {"box", "torque"}
    # ≤ 0.25 rad from the wait pose with 0.7 s to go: both reach layers pass, and
    # the ball (2.5 m/s, d_eff/T_close = 2 m/s) is inside the γ window
    assert summary["open_throws"] == {"box": 2, "torque": 2}
    with (tmp_path / "out" / "gate_map.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 4
    for row in rows:
        assert 0.0 < float(row["t_min_box_s"]) < float(row["reach_budget_s"])
        assert float(row["reach_budget_s"]) == pytest.approx(0.7 - 0.24 - 0.05 - 0.03)
        assert float(row["v_dir_max_dls"]) <= float(row["v_dir_max_lp"]) * (1 + 1e-6)
        assert float(row["g_min"]) == pytest.approx(1.0 - 0.2 / (2.5 * 0.10))
    assert summary["commit_lead_s"] == pytest.approx(
        0.10 + 0.05 + 0.03
    )  # T_close + T_arm + margin
    assert summary["min_flight_time_s"] == pytest.approx(0.24 + 0.18)
    # python's own inputs, recomputed independently for one candidate: the LP speed uses the
    # η_v-scaled RATED limits, and the torque layer starts from the map's wait pose
    first = rows[0]
    q_star = np.array([float(first[f"q{i}"]) for i in range(6)])
    with (tmp_path / "map" / "candidates.csv").open() as handle:
        source = next(r for r in csv.DictReader(handle) if r["id"] == first["id"])
    v_hat = base.unit([float(source[f"v_model_{a}"]) for a in "xyz"])
    terms = arm.terms(q_star, np.zeros(6))
    lp, _ = csb.directional_speed_lp(terms["jp"], terms["jw"], v_hat, 0.9 * base.QD_MAX)
    assert float(first["v_dir_max_lp"]) == pytest.approx(lp.v_dir_max, rel=1e-9)
    expected = cgm.torque_reach_time(
        arm, base.Q_GENERIC + 0.05, q_star, 0.9 * base.QD_MAX, 0.8 * base.TAU_MAX
    )
    assert float(first["t_min_torque_s"]) == pytest.approx(expected, rel=1e-9)
    assert expected > 0.0
    # v_max is derived so that η_v·v_max is the largest LP speed: the TCP term never binds
    best_lp = max(float(r["v_dir_max_lp"]) for r in rows)
    assert summary["v_max_m_s"] == pytest.approx(best_lp / 0.9)


def test_each_constant_reaches_its_own_gate(tmp_path, arm):
    # One change per run and the gate that must flip. `also` names a gate that
    # legitimately shares the constant; every other gate must stay untouched.
    cases = {
        # commit lead 0.50 + 0.05 + 0.03 > 0.7 − 0.24. T_close,tot also sets g_min (§4.5)
        "commit": ({"--close-total-s": "0.50"}, "commit_lead", {"gamma"}),
        # budget 0.38 s: a 1e-4 scale of the box makes every move take tens of seconds
        "reach_box": ({"box_scale": 1e-4}, "reach_time", set()),
        "gamma": ({"--d-eff-m": "0.001"}, "gamma_window_empty", set()),
        "stop": ({"--a-dec-m-s2": "0.05"}, "stop_outside_workspace", set()),
    }
    for gate, (override, reason, also) in cases.items():
        run = tmp_path / gate
        run.mkdir()
        assert cgm.main(_write_run(run, arm, **override)) == 0
        summary = _summary(run)
        assert summary["open_throws"]["box"] == 0, gate
        assert summary["candidate_reasons"]["box"] == {reason: 4}, gate
        alone = summary["candidates_stopped_by_each_gate_alone"]
        assert alone[gate] == 4, gate
        untouched = {"commit", "reach_box", "gamma", "stop"} - {gate} - also
        assert all(alone[other] == 0 for other in untouched), (gate, alone)


def test_the_acceleration_box_binds_the_box_layer_only(tmp_path, arm):
    argv = _write_run(tmp_path, arm)
    (tmp_path / "limits.yaml").write_text(
        yaml.safe_dump({"derived_accel_limits": {"arm": {"qdd_max": [1e-3] * 6}}})
    )
    assert cgm.main(argv) == 0
    assert _summary(tmp_path)["open_throws"] == {"box": 0, "torque": 2}


def test_cli_refuses_a_union_over_wait_poses(tmp_path, arm):
    argv = _write_run(tmp_path, arm)
    path = tmp_path / "map" / "candidates.csv"
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    rows[0]["seed_id"] = "1"
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with pytest.raises(SystemExit, match="seed-id"):
        cgm.main(argv)
    assert cgm.main([*argv, "--seed-id", "0"]) == 0
    assert _summary(tmp_path)["kinematic_candidates"] == 3


def test_cli_refuses_a_map_whose_poses_do_not_match_this_model(tmp_path, arm):
    argv = _write_run(tmp_path, arm)
    path = tmp_path / "map" / "candidates.csv"
    text = path.read_text().splitlines()
    head = text[0].split(",")
    i, j = head.index("q1"), head.index("q2")
    head[i], head[j] = head[j], head[i]  # a joint-order mix-up between the map and this tool
    path.write_text("\n".join([",".join(head), *text[1:]]) + "\n")
    with pytest.raises(SystemExit, match="FK"):
        cgm.main(argv)
