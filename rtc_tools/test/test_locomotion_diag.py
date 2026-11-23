"""locomotion_diag: header parsing, the tick join, summary numbers and a plot smoke test."""

import json
import math
from pathlib import Path

import pytest

from rtc_tools.analysis import locomotion_diag as ld

CONTACTS = ["lfoot", "rfoot"]
JOINTS = ["j_a", "j_b", "j_c"]
N = 10


def diag_header() -> list[str]:
    cols = [
        "t_relative_s",
        "tick",
        "tick_count",
        "mode",
        "fault_cause",
        "fault_latched",
        "fault_count",
        "source_status",
        "source_fresh_mask",
        "source_valid_mask",
        "source_bad_ticks",
        "source_sequence",
        "source_age_us",
        "health",
        "health_bad_run",
        "tau_valid",
        "qp_converged",
        "qp_status",
        "qp_iterations",
        "rows_dropped",
        "solve_time_us",
        "wbc_tick_us",
        "qp_pri_res",
        "qp_dua_res",
        "eom_residual_inf",
        "margin_friction_n",
        "margin_cop_nm",
        "margin_yaw_nm",
        "margin_torque_nm",
        "margin_joint_accel",
        "margins_stale",
        "base_px",
        "base_py",
        "base_pz",
        "base_qx",
        "base_qy",
        "base_qz",
        "base_qw",
        "base_vx",
        "base_vy",
        "base_vz",
        "base_wx",
        "base_wy",
        "base_wz",
        "ref_base_px",
        "ref_base_py",
        "ref_base_pz",
        "com_x",
        "com_y",
        "com_z",
        "ref_source",
        "plan_id",
        "policy_age_us",
        "mpc_converged",
        "mpc_solve_us",
        "schedule_epoch",
        "contact_mask",
        "plausibility_rejects",
        "u_fb_norm",
        "mpc_state_epoch",
    ]
    for c in CONTACTS:
        cols += [f"foot_{c}_{a}" for a in "xyz"]
    cols += [
        "a_base_vx",
        "a_base_vy",
        "a_base_vz",
        "a_base_wx",
        "a_base_wy",
        "a_base_wz",
        "servo_norm",
        "ff_norm",
    ]
    for c in CONTACTS:
        cols += [f"lambda_{c}_{a}" for a in ("fx", "fy", "fz", "tx", "ty", "tz")]
    for prefix in ("v_", "tau_servo_"):
        cols += [prefix + j for j in JOINTS]
    return cols


def ff_header() -> list[str]:
    cols = [
        "t_relative_s",
        "tick",
        "estop",
        "readable",
        "seeded",
        "traj_active",
        "ff_enabled",
        "mode",
        "ff_finite",
    ]
    cols += [
        "model_fresh",
        "held",
        "held_ticks",
        "singular",
        "closure_error",
        "clamped_mask",
        "inertia_gain",
        "num_joints",
    ]
    for prefix in ("q_", "q_ref_", "g_", "tau_ff_", "qdd_ref_", "tau_inertia_"):
        cols += [prefix + j for j in JOINTS]
    return cols


TICKS = [0, 1, 2, 4, 5, 6, 7, 8, 9, 10]  # tick 3 is missing


def diag_rows() -> list[dict]:
    rows = []
    for i, tick in enumerate(TICKS):
        rows.append(
            {
                "t_relative_s": 0.1 * i,
                "tick": tick,
                "mode": 1 if i < 6 else 3,
                "fault_latched": 1 if i >= 6 else 0,
                "source_status": 0 if i != 2 else 2,
                "tau_valid": 0 if i == 3 else 1,
                "qp_status": 0 if i < 8 else 3,
                "solve_time_us": 100.0 + i,
                "wbc_tick_us": 200.0,
                "eom_residual_inf": math.nan if i == 4 else 1e-6 * (i + 1),
                "margin_friction_n": 5.0,
                "margins_stale": 1 if i == 0 else 0,
                "base_px": 0.01 * i,  # drift 0.09 at the end
                "base_pz": 0.8 - 0.001 * i,
                "base_qw": 1.0,
                "servo_norm": 1.0,
                "ff_norm": 4.0 if i else 0.0,
                "lambda_lfoot_fz": 100.0,
                "lambda_rfoot_fz": 90.0,
                "tau_servo_j_a": 0.5,
                # idle for 2 rows, stand for 4, then the fault
                "ref_source": 0 if i < 2 else (1 if i < 6 else 3),
                "policy_age_us": -1.0 if i < 2 else 1000.0 * i,
                "ref_base_pz": 0.79 if 2 <= i < 6 else 0.7,
                "plausibility_rejects": 1 if i >= 4 else 0,
                "mpc_state_epoch": 1,
            }
        )
    return rows


def write_csv(path: Path, header: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [",".join(header)]
    for r in rows:
        lines.append(",".join(repr(float(r.get(h, 0.0))) for h in header))
    path.write_text("\n".join(lines) + "\n")


@pytest.fixture
def session(tmp_path: Path) -> Path:
    d = tmp_path / "sess" / ld.SUBDIR
    write_csv(d / ld.DIAG_FILE, diag_header(), diag_rows())
    ff_rows = [
        {
            "t_relative_s": 0.1 * i,
            "tick": i,
            "tau_ff_j_a": 2.0 * i,
            "q_j_a": 0.1,
            "q_ref_j_a": 0.3,
        }
        for i in range(11)
        if i not in (5,)  # joint_ff lacks tick 5, which diag has
    ]
    write_csv(d / ld.JOINT_FF_FILE, ff_header(), ff_rows)
    state_header = ["t_relative_s"] + [f"effort_{j}" for j in JOINTS]
    write_csv(
        d / "body_state.csv",
        state_header,
        [{"t_relative_s": 0.1 * i, "effort_j_a": 10.0 * i} for i in range(11)],
    )
    return tmp_path / "sess"


def test_parse_names():
    names = ld.parse_diag_names(diag_header())
    assert names.contacts == CONTACTS
    assert names.joints == JOINTS
    assert ld.parse_ff_joints(ff_header()) == JOINTS


def test_parse_names_with_the_r_tier_columns_appended():
    # The R tier appends its blocks after the existing columns; none of their
    # prefixes may read as a contact or a joint.
    header = [*diag_header(), "state_source"]
    header += [
        f"wrench_des_{c}_{a}" for c in CONTACTS for a in ("fx", "fy", "fz", "tx", "ty", "tz")
    ]
    header += [f"servo_kp_{j}" for j in JOINTS] + [f"servo_kd_{j}" for j in JOINTS]
    header += ["mpc_last_seq", "mpc_last_outcome", "mpc_last_plan_id"]
    names = ld.parse_diag_names(header)
    assert names.contacts == CONTACTS
    assert names.joints == JOINTS


def test_join_on_tick(session: Path):
    sess = ld.load_session(session, group="body")
    assert len(sess.diag["tick"]) == N
    assert sess.ff is not None
    # diag row 4 is tick 5: joint_ff has no such row
    assert math.isnan(sess.ff["tau_ff_j_a"][4])
    assert sess.ff["tau_ff_j_a"][3] == pytest.approx(8.0)  # tick 4
    assert int(sess.ff["_present"].sum()) == N - 1
    assert sess.effort["j_a"][2] == pytest.approx(20.0)


def test_window(session: Path):
    sess = ld.load_session(session, t0=0.25, t1=0.65)
    assert list(sess.diag["tick"]) == [4, 5, 6, 7]


def test_summary(session: Path):
    diag = ld.load_session(session).diag
    s = ld.summarize(diag)
    assert s["rows"] == N
    assert s["rows_per_mode"] == {"nominal": 6, "fault": 4}
    assert s["tau_valid_false_rows"] == 1
    assert s["qp_status_histogram"] == {"0": 8, "3": 2}
    assert s["qp_status_nonzero_rows"] == 2
    assert s["fault_latches"] == 1
    assert s["source_not_ok_rows"] == 1
    assert s["max_tick_gap"] == 2.0
    assert s["missing_ticks"] == 1
    drift = s["series"]["base_xy_drift_m"]
    assert drift["max"] == pytest.approx(0.09)
    assert s["series"]["base_height_dev_abs_m"]["max"] == pytest.approx(0.009)
    eom = s["series"]["eom_residual_inf"]
    assert eom["n_nonfinite"] == 1
    assert eom["n"] == N - 1
    # row 0 has margins_stale: its margin is excluded and not counted as non-finite data
    assert s["margins_stale_rows"] == 1
    assert s["series"]["margin_friction_n"]["n"] == N - 1
    # servo share: row 0 has ff_norm 0 and is left out
    assert s["series"]["servo_share"]["p50"] == pytest.approx(0.25)
    assert s["series"]["servo_share"]["n"] == N - 1
    assert s["rows_per_reference_source"] == {"idle": 2, "stand": 4, "fault": 4}
    assert s["stand_fraction"] == pytest.approx(0.4)
    assert s["time_to_stand_s"] == pytest.approx(0.2)
    assert s["walk_rows"] == 0
    assert s["plausibility_rejects"] == 1
    assert s["stand_ref_base_z_mean_m"] == pytest.approx(0.79)
    assert s["policy_age_us"]["n"] == N - 2
    assert s["policy_age_us"]["min"] == pytest.approx(2000.0)


def test_summary_of_a_log_without_the_mpc_columns(tmp_path: Path):
    header = [h for h in diag_header() if h not in ("ref_source", "policy_age_us")]
    write_csv(tmp_path / "old" / ld.SUBDIR / ld.DIAG_FILE, header, diag_rows())
    s = ld.summarize(ld.load_session(tmp_path / "old").diag)
    assert "stand_fraction" not in s
    assert s["rows"] == N
    assert s["pose_guard_refused_rows"] is None, "a log without the column reports none"


def test_summary_counts_the_pose_guards_refusals(tmp_path: Path):
    header = [*diag_header(), "pose_guard_mask"]
    rows = [
        {**r, "pose_guard_mask": 8 if i in (1, 2, 3) else 0} for i, r in enumerate(diag_rows())
    ]
    write_csv(tmp_path / "pg" / ld.SUBDIR / ld.DIAG_FILE, header, rows)
    s = ld.summarize(ld.load_session(tmp_path / "pg").diag)
    assert s["pose_guard_refused_rows"] == 3
    assert ld.FAULT_CAUSE_NAMES[6] == "pose guard"


def test_z_ref_and_tilt(session: Path):
    diag = ld.load_session(session).diag
    base = ld.base_series(diag, z_ref=0.9)
    assert base["base_height_dev_m"][0] == pytest.approx(-0.1)
    assert float(ld.np.nanmax(base["base_tilt_rad"])) == 0.0
    # The row whose source was not ok carries no measured base.
    assert math.isnan(base["base_height_dev_m"][2])
    assert math.isnan(base["base_xy_drift_m"][2])
    tilt = ld.tilt_rad(ld.np.array([math.sin(0.25)]), ld.np.array([0.0]))
    assert tilt[0] == pytest.approx(0.5)


def test_main_writes_figures_and_json(session: Path, tmp_path: Path, capsys):
    base_yaml = tmp_path / "_base.yaml"
    base_yaml.write_text(
        "/**:\n  ros__parameters:\n    devices:\n      body:\n"
        "        joint_state_names: [j_a, j_b, j_c]\n"
        "        joint_limits:\n          max_torque: [50.0, 50.0, 20.0]\n"
    )
    stance = tmp_path / "stance.csv"
    stance.write_text(
        "t,px,py,pz,qw,qx,qy,qz,foot0_x,foot0_y,foot0_z,foot0_load,foot1_x,foot1_y,foot1_z,foot1_load\n"
        "5.0,0,0,0.8,1,0,0,0,0,0,0,100,0,0.2,0,90\n"
        "5.5,0,0,0.8,1,0,0,0,0.01,0,0,100,0,0.2,0,90\n"
    )
    rc = ld.main(
        [
            str(session),
            "--stance-csv",
            str(stance),
            "--base-yaml",
            str(base_yaml),
            "--group",
            "body",
            "--no-show",
        ]
    )
    assert rc == 0
    out = session / "plots" / "locomotion_diag"
    for name in ld.FIGURE_FILES:
        assert (out / name).stat().st_size > 0
    summary = json.loads((out / ld.SUMMARY_FILE).read_text())
    assert summary["contacts"] == CONTACTS
    assert summary["joints"] == JOINTS
    assert summary["fault_latches"] == 1
    capsys.readouterr()


def test_main_without_joint_ff(session: Path, tmp_path: Path):
    (session / ld.SUBDIR / ld.JOINT_FF_FILE).unlink()
    out = tmp_path / "o"
    assert ld.main([str(session), "--out", str(out)]) == 0
    assert len(list(out.glob("*.png"))) == 7


def test_missing_diag_is_an_error(tmp_path: Path, capsys):
    assert ld.main([str(tmp_path)]) == 2
    assert "error" in capsys.readouterr().err
