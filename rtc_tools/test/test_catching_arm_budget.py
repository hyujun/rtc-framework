"""catching_arm_budget — synthetic positive controls (dynamic_catching S8-G, #537).

A unit is built from KNOWN laws: a first-order plant of lag τ, a critically
damped reference of frequency ω from a planted commit-time error, a planted
saturation stretch, a command ramp of planted acceleration, a planted torque
fraction, planted rank bits. The tool must read each one back — and the
closed-form reach time is checked against hand-derived cases, because the
planner's gate is re-judged with it.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from rtc_tools.analysis import catching_arm_budget as ab, catching_trials as ct

JOINTS = ("j_a", "j_b")
ARM, HAND = "armdev", "handdev"
CONTROLLER = "demo_catching_controller"
DT = 0.002
TAU = 0.05
OMEGA = 10.0
A_MAX = 21.0
E0_MM = 100.0
RAMP_ACCEL = 8.0  # rad/s², joint j_a
N_APPROACH, N_COMMITTED, N_CLOSING = 10, 50, 100
N_SAT = 20  # saturated ticks right after commit
TORQUE = (25.0, 10.0)  # effort planted on the lane
TAU_MAX = (50.0, 20.0)  # → utilisation 0.5 on both joints
BOX = (2.0, 2.0)


def _write_yaml(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(doc, sort_keys=False))


def make_config(
    share: Path, *, sim_velocity: float | None = 1.0, box: tuple[float, ...] | None = BOX
) -> Path:
    """A minimal robot profile under ``<share>/config/robot`` (the launch's layout)."""
    cfg = share / "config" / "robot"
    _write_yaml(
        cfg / "_base.yaml",
        {
            "/**": {
                "ros__parameters": {
                    "control_rate": 500.0,
                    "urdf": {
                        "extra_frames": {
                            "catch_frame": {"parent": "palm", "xyz": [0, 0, 0], "rpy": [0, 0, 0]}
                        }
                    },
                    "devices": {
                        ARM: {
                            "joint_limits": {
                                "max_torque": list(TAU_MAX),
                                "max_velocity": [2.0, 2.0],
                            }
                        },
                        HAND: {"joint_limits": {"max_torque": [1.0]}},
                    },
                }
            }
        },
    )
    if sim_velocity is not None:
        _write_yaml(
            cfg / "sim.yaml",
            {
                "/**": {
                    "ros__parameters": {
                        "devices": {ARM: {"joint_limits": {"max_velocity": [sim_velocity] * 2}}}
                    }
                }
            },
        )
    arm_block = {}
    if box is not None:
        _write_yaml(
            cfg / "derived_accel_limits.yaml",
            {
                "derived_accel_limits": {
                    ARM: {"qdd_max": list(box), "adopted": True, "provisional": True}
                }
            },
        )
        arm_block = {
            "accel_limits_package": "some_pkg",
            "accel_limits_path": "config/robot/derived_accel_limits.yaml",
            "accel_limits_group": ARM,
        }
    _write_yaml(
        cfg / "controllers" / f"{CONTROLLER}.yaml",
        {
            CONTROLLER: {
                "catching": {
                    "io": {
                        "arm_base_frame": "base",
                        "base_T_world": {"yaw_deg": 0.0, "translation": [0, 0, 0]},
                    },
                    "reference": {"omega": OMEGA, "a_max": A_MAX, "v_max": 3.5},
                    "planner": {"gamma": {"eta_v": 0.9}},
                    "robot": {"arm": arm_block},
                },
                "topics": {ARM: {"subscribe": []}, HAND: {"subscribe": []}},
                "logs": [
                    {"msg_type": "rtc_msgs/DeviceStateLog", "instance": f"{ARM}_state"},
                    {
                        "msg_type": "integrated_bringup/CatchingDiagLog",
                        "instance": "catching_diag",
                    },
                ],
            }
        },
    )
    return cfg


def _first_order(q_cmd: np.ndarray, tau: float, dt: float) -> np.ndarray:
    q = np.zeros_like(q_cmd)
    q[0] = q_cmd[0]
    for k in range(1, len(q_cmd)):
        q[k] = q[k - 1] + dt / tau * (q_cmd[k - 1] - q[k - 1])
    return q


def make_session(root: Path, *, n_trials: int = 3, mirror: bool = True) -> tuple[Path, Path, dict]:
    """One synthetic unit + session: ``n_trials`` active stretches, known laws throughout."""
    unit = root / "unit"
    session = unit / "session_copy"
    ctl = session / "controllers" / CONTROLLER
    ctl.mkdir(parents=True)
    (unit / "trials").mkdir()
    (unit / "ct").mkdir()
    n_seg = N_APPROACH + N_COMMITTED + N_CLOSING
    n_idle = 200
    rows = []
    effort_rows = []
    ct_rows = []
    t = 0.0
    q_a = 0.0
    for _trial in range(n_trials):
        for _ in range(n_idle):
            rows.append(_tick(t, ct.MODE_ARMED, 0.0, 0.0, 0.0, 0, q_a, 0.0))
            effort_rows.append((t, 0.0, 0.0))
            t += DT
        t_commit = t + N_APPROACH * DT
        ct_rows.append(t_commit)
        for k in range(n_seg):
            if k < N_APPROACH:
                mode, e, u, xdd, sat = ct.MODE_APPROACH, E0_MM, 30.0, A_MAX, 1
            else:
                since = (k - N_APPROACH) * DT
                mode = ct.MODE_COMMITTED if k < N_APPROACH + N_COMMITTED else ct.MODE_CLOSING
                e = E0_MM * (1.0 + OMEGA * since) * math.exp(-OMEGA * since)
                sat = 1 if k - N_APPROACH < N_SAT else 0
                u, xdd = (30.0, A_MAX) if sat else (10.0, 10.0)
            # joint j_a: constant acceleration for the first 50 ticks of the stretch, then cruise
            k_acc = min(k, 50)
            q_a_k = (
                q_a
                + 0.5 * RAMP_ACCEL * (k_acc * DT) ** 2
                + RAMP_ACCEL * (50 * DT) * max(0, k - 50) * DT
            )
            rows.append(
                _tick(
                    t,
                    mode,
                    e,
                    u,
                    xdd,
                    sat,
                    q_a_k,
                    0.37 - (k - N_APPROACH) * DT if k >= N_APPROACH else 0.4,
                )
            )
            effort_rows.append((t, *TORQUE))
            t += DT
        q_a = rows[-1]["q_cmd_j_a"]
        for _ in range(50):
            rows.append(_tick(t, ct.MODE_DECEL, 0.0, 0.0, 0.0, 0, q_a, 0.0))
            effort_rows.append((t, 0.0, 0.0))
            t += DT
    q_cmd = np.array([[r["q_cmd_j_a"], r["q_cmd_j_b"]] for r in rows])
    q_meas = _first_order(q_cmd, TAU, DT)
    for r, qm in zip(rows, q_meas, strict=True):
        r["q_meas_j_a"], r["q_meas_j_b"] = float(qm[0]), float(qm[1])
    _write_csv(ctl / "catching_diag.csv", rows)
    _write_csv(
        ctl / f"{ARM}_state.csv",
        [{"t_relative_s": tt, "effort_j_a": ea, "effort_j_b": eb} for tt, ea, eb in effort_rows],
    )
    events = [
        {
            "plan_valid": 0,
            "rank_reach": 0,
            "rank_gamma": 0,
            "rank_rollout": 0,
            "rank_error_budget": 0,
            "rank_uncertainty": 0,
            "rank_commit_lead": 0,
            "lead_s": 0.0,
            "gamma_f": 0.0,
        }
    ] * 5
    events += [
        {
            "plan_valid": 1,
            "rank_reach": int(i < 9),
            "rank_gamma": 1,
            "rank_rollout": 0,
            "rank_error_budget": 1,
            "rank_uncertainty": 1,
            "rank_commit_lead": 0,
            "lead_s": 0.45,
            "gamma_f": 0.4,
        }
        for i in range(10)
    ]
    _write_csv(ctl / "planner_events.csv", events)
    _write_csv(
        unit / "ct" / "catching_trials.csv",
        [
            {
                "idx": i,
                "t_commit": tc,
                "ref_vs_true_mm": 40.0,
                "total_mm": 45.0,
                "d_min_mm": 30.0,
                "pred_mm": 80.0,
                "clik_mm": 3.0,
                "servo_mm": 3.0,
                "ball_speed_tc": 4.0,
                "gamma_f_planned": 0.4,
                "contact_v_rel": 2.0,
                "truth_success": "True" if i == 0 else "False",
            }
            for i, tc in enumerate(ct_rows)
        ],
    )
    meta = {"arm": "synthetic", "controller_mirror": {"control.dt": DT}}
    if mirror:
        meta["controller_mirror"].update(
            {
                "reference.omega": OMEGA,
                "reference.a_max": A_MAX,
                "reference.v_max": 3.5,
                "planner.gamma.eta_v": 0.9,
                "robot.arm.qdd_max": list(BOX),
            }
        )
    (unit / "trials" / "run_meta.json").write_text(json.dumps(meta))
    return unit, session, meta


def _tick(t, mode, e_mm, u, xdd, sat, q_a, plan_t_c) -> dict:
    return {
        "t_relative_s": t,
        "mode": mode,
        "plan_t_c_s": plan_t_c,
        "ref_saturated": sat,
        "clik_bound_conflict": 0,
        "track_err_rad": 0.0,
        "t_arm_s": 0.05,
        "ref_xd_x": 1.0,
        "ref_xd_y": 0.0,
        "ref_xd_z": 0.0,
        "ref_u_des_x": u,
        "ref_u_des_y": 0.0,
        "ref_u_des_z": 0.0,
        "ref_xdd_x": xdd,
        "ref_xdd_y": 0.0,
        "ref_xdd_z": 0.0,
        "ref_e_x": e_mm / 1e3,
        "ref_e_y": 0.0,
        "ref_e_z": 0.0,
        "q_cmd_j_a": q_a,
        "q_cmd_j_b": 0.0,
    }


def _write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


# ── closed-form reach time (L3 §4.3) ─────────────────────────────────────────
@pytest.mark.parametrize(
    ("d", "w", "w_max", "a", "expected"),
    [
        (1.0, 0.0, 1e9, 2.0, 2.0 * math.sqrt(0.5)),  # rest-to-rest triangle
        (1.0, 0.0, 0.5, 1.0, 0.5 + 0.5 + (1.0 - 0.25) / 0.5),  # trapezoid
        (1.0, 0.5, 10.0, 1.0, (2.0 * math.sqrt(1.125) - 0.5)),  # already moving towards
        (
            1.0,
            -0.5,
            10.0,
            1.0,
            0.5 + 2.0 * math.sqrt(1.125),
        ),  # moving away: stop, then rest-to-rest
        (0.1, 1.0, 10.0, 1.0, 1.0 + 2.0 * math.sqrt(0.4)),  # overshoot: stop, come back
        (-1.0, 0.0, 1e9, 2.0, 2.0 * math.sqrt(0.5)),  # sign of d does not matter
        (0.0, 0.0, 1.0, 1.0, 0.0),
    ],
)
def test_t_min_joint_matches_the_hand_derivation(d, w, w_max, a, expected):
    assert ab.t_min_joint(d, w, w_max, a) == pytest.approx(expected, rel=1e-9)


def test_t_min_joint_fails_closed_on_bad_limits_or_an_overspeed_start():
    assert math.isnan(ab.t_min_joint(1.0, 0.0, 1.0, 0.0))
    assert math.isnan(ab.t_min_joint(1.0, 0.0, -1.0, 1.0))
    assert math.isnan(ab.t_min_joint(1.0, 2.0, 1.0, 1.0))
    assert math.isnan(ab.reach_time([1.0, 1.0], [0.0, 0.0], [1.0, 1.0], [1.0, float("nan")]))
    assert ab.reach_time([1.0, 0.1], [0.0, 0.0], [10.0, 10.0], [1.0, 1.0]) == pytest.approx(2.0)


def test_ds_residual_fraction_is_the_critically_damped_law():
    assert ab.ds_residual_fraction(10.0, 0.318) == pytest.approx((1 + 3.18) * math.exp(-3.18))
    assert ab.ds_residual_fraction(10.0, 0.0) == 1.0
    assert math.isnan(ab.ds_residual_fraction(0.0, 1.0))


# ── the synthetic unit ───────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def synthetic(tmp_path_factory) -> dict:
    root = tmp_path_factory.mktemp("s8g")
    cfg = make_config(root / "share")
    unit, session, meta = make_session(root)
    result = ab.analyse_unit(unit, session, cfg, n_boot=20)
    return {"cfg": cfg, "unit": unit, "session": session, "result": result}


def test_the_budget_comes_from_the_mirror_and_the_sim_overlay_of_the_rating(synthetic):
    b = synthetic["result"]["summary"]["budget"]
    assert b["joints"] == list(JOINTS)
    assert (b["omega"], b["a_max"], b["v_max"], b["eta_v"]) == (OMEGA, A_MAX, 3.5, 0.9)
    assert b["qdd_box"] == list(BOX)
    assert (
        b["source"]["omega"] == "controller_mirror"
        and b["source"]["qdd_box"] == "controller_mirror"
    )
    assert b["tau_max"] == list(TAU_MAX)
    assert b["qd_box"] == [1.0, 1.0] and b["source"]["max_velocity"] == "sim.yaml"
    assert b["source"]["max_torque"] == "_base.yaml"


def test_the_plant_lag_and_torque_utilisation_are_read_back(synthetic):
    p = synthetic["result"]["summary"]["plant"]
    assert p["tau_hat_s"][0] == pytest.approx(
        TAU, rel=0.05
    )  # j_a moves; the discrete plant is τ within 5 %
    assert (
        math.isnan(p["tau_hat_s"][1]) or p["tau_hat_r2"][1] != p["tau_hat_r2"][1]
    )  # j_b never moves: no fit
    assert p["torque_util_max"] == pytest.approx([0.5, 0.5])
    assert p["torque_source"] == f"{ARM}_state.csv"
    assert p["qd_meas_max"][0] > 0.5


def test_the_reference_residual_saturation_and_theory_line_up(synthetic):
    r = synthetic["result"]["summary"]["reference"]
    n_active = N_APPROACH + N_COMMITTED + N_CLOSING
    assert r["saturated_frac"] == pytest.approx((N_APPROACH + N_SAT) / n_active)
    assert r["a_max_hit_frac"] == pytest.approx((N_APPROACH + N_SAT) / n_active)
    assert r["u_des_max"] == 30.0 and r["u_des_p50"] == 10.0
    duration = (N_COMMITTED + N_CLOSING - 1) * DT
    assert r["duration_commit_to_last_p50_s"] == pytest.approx(duration)
    theory = (1 + OMEGA * duration) * math.exp(-OMEGA * duration)
    assert r["residual_fraction_theory"] == pytest.approx(theory)
    assert r["residual_fraction_measured_p50"] == pytest.approx(theory, rel=1e-6)


def test_the_executed_envelope_and_the_velocity_box_hits_are_read_back(synthetic):
    c = synthetic["result"]["summary"]["clik"]
    assert c["envelope_p95_rad_s2"][0] == pytest.approx(RAMP_ACCEL, rel=0.05)
    assert c["envelope_p95_rad_s2"][1] == pytest.approx(0.0, abs=1e-9)
    # j_a cruises at 8 · 0.1 = 0.8 rad/s against a sim box of 1.0: never at the box (0.98)
    assert c["velocity_box_hit_frac"] == 0.0
    # every tick of the ramp is above the 2.0 rad/s² planner box; the cruise is not
    assert 0.2 < c["ticks_over_planner_box_frac"] < 0.5


def test_the_planner_box_is_re_judged_against_the_envelope(synthetic):
    pl = synthetic["result"]["summary"]["planner"]
    trials = synthetic["result"]["trials"]
    assert len(trials) == 3
    # the same motion: slow box → longer time than the executed envelope's
    assert pl["t_reach_box_p50_s"] > pl["t_reach_envelope_p50_s"] > 0.0
    assert pl["valid_plans"] == 10 and pl["rank_reach"] == pytest.approx(0.9)
    assert pl["rank_gamma"] == 1.0 and pl["rank_rollout"] == 0.0
    assert pl["lead_s_p50"] == 0.45 and pl["gamma_f_p50"] == 0.4
    for row in trials:
        assert row["lead_avail_s"] == pytest.approx(0.37 - 0.05 - ab.PLANNER_TIME_MARGIN_S)
        assert row["reach_ok_envelope"] in (True, False) and row["reach_ok_box"] in (True, False)


def test_trials_join_the_catching_trials_row_by_commit_time(synthetic):
    s = synthetic["result"]["summary"]
    trials = synthetic["result"]["trials"]
    assert [r["idx"] for r in trials] == [0, 1, 2]
    assert s["success"] == 1
    assert s["gap_mm"]["e_commit_mm"]["p50"] == pytest.approx(E0_MM)
    theory = (1 + OMEGA * (N_COMMITTED + N_CLOSING - 1) * DT) * math.exp(
        -OMEGA * (N_COMMITTED + N_CLOSING - 1) * DT
    )
    assert s["gap_mm"]["e_last_mm"]["p50"] == pytest.approx(E0_MM * theory, rel=1e-6)
    assert s["gap_mm"]["pred_live_lb_mm"]["p50"] == pytest.approx(40.0 - E0_MM * theory)
    assert s["gap_mm"]["total_mm"]["p50"] == 45.0 and s["gap_mm"]["d_min_mm"]["p50"] == 30.0


def test_without_a_mirror_the_budget_comes_from_the_profile_and_the_overlay(tmp_path):
    cfg = make_config(tmp_path / "share")
    unit, session, _ = make_session(tmp_path, n_trials=1, mirror=False)
    overlay = tmp_path / "ov.yaml"
    _write_yaml(
        overlay,
        {
            "integrated_rt_controller": {
                "ros__parameters": {
                    CONTROLLER: {"catching": {"reference": {"omega": 15.0, "a_max": 30.0}}}
                }
            }
        },
    )
    res = ab.analyse_unit(unit, session, cfg, overlays=[overlay], n_boot=5)
    b = res["summary"]["budget"]
    assert (b["omega"], b["a_max"], b["v_max"]) == (15.0, 30.0, 3.5)
    assert b["source"]["omega"] == "profile+overlays"
    assert b["qdd_box"] == list(BOX) and "derived_accel_limits.yaml" in b["source"]["qdd_box"]
    assert res["summary"]["reference"]["residual_fraction_theory"] == pytest.approx(
        ab.ds_residual_fraction(15.0, res["summary"]["reference"]["duration_commit_to_last_p50_s"])
    )


def test_a_box_that_is_not_adopted_or_absent_reads_as_no_box(tmp_path):
    cfg = make_config(tmp_path / "share", box=None)
    unit, session, _ = make_session(tmp_path, n_trials=1, mirror=False)
    res = ab.analyse_unit(unit, session, cfg, n_boot=5)
    assert res["summary"]["budget"]["qdd_box"] == []
    assert math.isnan(res["summary"]["planner"]["t_reach_box_p50_s"])
    assert math.isnan(res["summary"]["clik"]["ticks_over_planner_box_frac"])
    assert res["trials"][0]["reach_ok_box"] is None
    box = tmp_path / "share" / "config" / "robot" / "derived_accel_limits.yaml"
    _write_yaml(box, {"derived_accel_limits": {ARM: {"qdd_max": [5.0, 5.0], "adopted": False}}})
    assert ab._box_from_file(cfg, "pkg", "config/robot/derived_accel_limits.yaml", ARM) == []


def test_the_envelope_box_document_pools_the_per_joint_max_and_carries_provenance():
    units = [
        {
            "summary": {
                "arm": "a",
                "unit": "/x/u1",
                "trials_committed": 3,
                "budget": {"joints": list(JOINTS)},
                "clik": {"envelope_p95_rad_s2": [8.0, 1.0]},
            }
        },
        {
            "summary": {
                "arm": "b",
                "unit": "/x/u2",
                "trials_committed": 4,
                "budget": {"joints": list(JOINTS)},
                "clik": {"envelope_p95_rad_s2": [6.0, 3.0]},
            }
        },
    ]
    doc = ab.envelope_box_document(units, ARM)
    entry = doc["derived_accel_limits"][ARM]
    assert entry["qdd_max"] == [8.0, 3.0]
    assert entry["adopted"] is True and entry["provisional"] is True
    assert entry["provenance"]["tool"] == "rtc_tools.analysis.catching_arm_budget"
    assert [u["unit"] for u in entry["provenance"]["units"]] == ["u1", "u2"]
    assert entry["provenance"]["sim_only"] is True
    with pytest.raises(ValueError):
        ab.envelope_box_document([], ARM)


def test_main_writes_the_outputs_and_the_envelope_box(tmp_path, capsys):
    cfg = make_config(tmp_path / "share")
    unit, session, _ = make_session(tmp_path, n_trials=2)
    out = tmp_path / "out"
    box = tmp_path / "env_box.yaml"
    rc = ab.main(
        [
            str(unit),
            "--config-dir",
            str(cfg),
            "--out",
            str(out),
            "--n-boot",
            "5",
            "--write-envelope-box",
            str(box),
        ]
    )
    assert rc == 0
    summary = json.loads((out / "arm_budget_summary.json").read_text())
    assert summary[0]["arm"] == "synthetic" and summary[0]["trials_committed"] == 2
    with (out / "arm_budget_trials.csv").open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2 and rows[0]["arm"] == "synthetic"
    doc = yaml.safe_load(box.read_text())
    assert doc["derived_accel_limits"][ARM]["qdd_max"][0] == pytest.approx(RAMP_ACCEL, rel=0.05)
    text = capsys.readouterr().out
    assert "P plant" in text and "B plan" in text and "envelope box →" in text


def test_parse_unit_arg_defaults_the_session_to_session_copy():
    assert ab.parse_unit_arg("/u") == (Path("/u"), Path("/u/session_copy"))
    assert ab.parse_unit_arg("/u:/s") == (Path("/u"), Path("/s"))
