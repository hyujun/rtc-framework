"""catching_wait_pose_search on a synthetic 6R arm (dynamic_catching S8-I).

The fixture arm mirrors ``test_catch_speed_budget.py``'s (asymmetric link
lengths, mixed joint axes) rather than the module docstring's illustrative "3R"
sketch: the self-consistent objective holds 5 rows (3 position + 2
approach-axis-orientation) per candidate, so fewer than 5 joints makes that
system generically OVERdetermined — the LP's exact-equality solution collapses
to ~0 almost everywhere and the (least-squares) DLS speed then reports above
it, which is exactly the "DLS never exceeds LP" invariant this tool fails
closed on. 6 joints (1 redundant DoF, matching every real arm this tool
targets) keeps the LP non-degenerate while the mixed axes still make posture
(which "elbow" the redundancy picks) change the speed along the pose's own
axis. Oracles are independent of the tool: ``catch_speed_budget``'s own
``directional_speed_lp``/``directional_speed_dls`` called directly with a
freshly-built ``ArmKinematics`` (not the tool's config loader) for the
per-pose numbers, and a coarse brute-force joint grid for the search claim.
"""

from __future__ import annotations

import csv
import itertools
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

pin = pytest.importorskip("pinocchio")
pytest.importorskip("scipy")

from rtc_tools.analysis import (  # noqa: E402
    catch_speed_budget as csb,
    catching_wait_pose_search as cws,
)

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
AXES = ["0 0 1", "0 1 0", "0 1 0", "1 0 0", "0 1 0", "1 0 0"]
LENGTHS = [0.30, 0.28, 0.22, 0.15, 0.12, 0.09]
ARM = "arm"
CONTROLLER = "demo_catching_controller"
BASE_QD = [2.0, 1.7, 3.0, 2.6, 3.4, 4.1]
SIM_QD = [3.0, 2.5, 4.0, 3.5, 4.5, 5.0]
ETA_V = 0.9
WAIT_POSE = [0.4, -0.9, 0.6, -0.3, 0.5, -0.4]
FRAME = csb.ExtraFrame("tool", (0.05, 0.0, 0.03), (0.3, -0.2, 0.1))
Q_LO, Q_HI = -3.0, 3.0
N = len(JOINTS)


def arm_urdf() -> str:
    links = ['  <link name="base"/>']
    joints = []
    parent = "base"
    for i, (name, axis, length) in enumerate(zip(JOINTS, AXES, LENGTHS, strict=True)):
        child = "tool" if i == len(JOINTS) - 1 else f"l{i + 1}"
        links.append(
            f'  <link name="{child}"><inertial><origin xyz="{length / 2} 0.01 0"/>'
            '<mass value="1.0"/><inertia ixx="0.01" iyy="0.02" izz="0.015" ixy="0" ixz="0" '
            'iyz="0"/></inertial></link>'
        )
        origin = LENGTHS[i - 1] if i else 0.0
        joints.append(
            f'  <joint name="{name}" type="revolute"><parent link="{parent}"/>'
            f'<child link="{child}"/><origin xyz="{origin} 0 0"/><axis xyz="{axis}"/>'
            f'<limit lower="{Q_LO}" upper="{Q_HI}" effort="50" velocity="5.0"/></joint>'
        )
        parent = child
    return (
        '<?xml version="1.0"?>\n<robot name="fixture">\n'
        + "\n".join(links + joints)
        + "\n</robot>\n"
    )


def _write_yaml(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(doc, sort_keys=False))


def make_config(
    share: Path,
    *,
    sim_velocity: list | None = SIM_QD,
    wait_pose: list | None = WAIT_POSE,
    eta_v: float | None = ETA_V,
    base_velocity: list = BASE_QD,
) -> Path:
    cfg = share / "config" / "robot"
    _write_yaml(
        cfg / "_base.yaml",
        {
            "/**": {
                "ros__parameters": {
                    "control_rate": 500.0,
                    "urdf": {
                        "package": "unused_pkg",
                        "path": "unused.urdf",
                        "extra_frames": {
                            "catch_frame": {
                                "parent": FRAME.parent,
                                "xyz": list(FRAME.xyz),
                                "rpy": list(FRAME.rpy),
                            }
                        },
                    },
                    "devices": {
                        ARM: {
                            "joint_state_names": JOINTS,
                            "joint_limits": {"max_velocity": list(base_velocity)},
                        }
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
                        "devices": {ARM: {"joint_limits": {"max_velocity": list(sim_velocity)}}}
                    }
                }
            },
        )
    catching: dict = {
        "io": {
            "arm_base_frame": "base",
            "base_T_world": {"yaw_deg": 0.0, "translation": [0, 0, 0]},
        },
        "planner": {},
    }
    if wait_pose is not None:
        catching["planner"]["wait_pose"] = list(wait_pose)
    if eta_v is not None:
        catching["planner"]["gamma"] = {"eta_v": eta_v}
    _write_yaml(
        cfg / "controllers" / f"{CONTROLLER}.yaml",
        {
            CONTROLLER: {
                "catching": catching,
                "topics": {ARM: {"subscribe": []}},
                "logs": [
                    {"msg_type": "integrated_bringup/CatchingDiagLog", "instance": "catching_diag"}
                ],
            }
        },
    )
    return cfg


def _urdf_file(tmp_path: Path) -> Path:
    urdf = tmp_path / "fixture.urdf"
    urdf.write_text(arm_urdf())
    return urdf


def oracle_arm(urdf_path: Path) -> csb.ArmKinematics:
    """An independently-built ArmKinematics — not through the tool's config loader."""
    return csb.ArmKinematics(urdf_path.read_text(), JOINTS, FRAME, np.zeros(N))


# ── (a) --evaluate-pose reproduces catch_speed_budget called directly ─────────


def test_evaluate_pose_matches_direct_lp_dls_computation(tmp_path, capsys):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    oracle = oracle_arm(urdf)
    q = np.array(WAIT_POSE)
    axis = oracle.frame_axis(q)
    v_hat = -axis
    terms = oracle.terms(q, np.zeros(N))
    qd_plan = ETA_V * np.asarray(SIM_QD)  # default make_config() ships sim.yaml — sim wins (g)
    lp, _ = csb.directional_speed_lp(terms["jp"], terms["jw"], v_hat, qd_plan)
    dls = csb.directional_speed_dls(terms["jp"], terms["jw"], v_hat, qd_plan)
    assert lp.v_dir_max > 1e-3  # the fixture must be non-degenerate, not the accident under test

    rc = cws.main(
        [
            "--config-dir",
            str(cfg),
            "--urdf",
            str(urdf),
            "--evaluate-pose",
            " ".join(str(v) for v in WAIT_POSE),
        ]
    )
    assert rc == 0
    out = capsys.readouterr().out
    got_lp = float(out.split("v_dir_lp = ")[1].split(" m/s")[0])
    got_dls = float(out.split("v_dir_dls = ")[1].split(" m/s")[0])
    assert got_lp == pytest.approx(lp.v_dir_max, rel=1e-4)
    assert got_dls == pytest.approx(dls.v_dir_max, rel=1e-4)


# ── (b) the search beats (or ties) a brute-force joint grid ───────────────────


def _load_setup(cfg: Path, urdf: Path):
    return cws.load_search_setup(cfg, urdf_override=urdf)


def test_search_beats_brute_force_grid(tmp_path):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    setup = _load_setup(cfg, urdf)
    arm = setup["arm"]
    qd_plan = setup["eta_v"] * np.asarray(setup["qd_box"])
    q_ref = np.array(setup["wait_pose"])

    result = cws.search_wait_poses(
        arm,
        qd_plan,
        q_ref,
        radius_m=None,
        axis_tol_deg=60.0,
        box=None,
        min_z=None,
        samples=4000,
        seed=3,
        walk_frac=0.6,
        refine_top=10,
        objective="lp",  # the grid below is an LP grid
    )
    assert result["refined"]
    best_search = max(r["v_dir_lp"] for r in result["refined"])
    ref_lp, _, _ = cws.eval_self_consistent(arm, q_ref, qd_plan)

    axis_ref = arm.frame_axis(q_ref)
    grid = np.linspace(Q_LO, Q_HI, 4)  # 4**6 = 4096 postures — coarse but independent of the tool
    best_grid = 0.0
    for combo in itertools.product(grid, repeat=N):
        q = np.array(combo)
        axis = arm.frame_axis(q)
        ang = math.degrees(math.acos(float(np.clip(axis @ axis_ref, -1.0, 1.0))))
        if ang > 60.0 + 1e-6:
            continue
        lp, _, valid = cws.eval_self_consistent(arm, q, qd_plan)
        if valid and lp > best_grid:
            best_grid = lp

    assert best_grid > ref_lp  # otherwise this grid cannot exercise the claim at all
    assert best_search >= ref_lp - 1e-9
    assert best_search >= 0.95 * best_grid


def test_the_objective_picks_the_largest_of_its_own_key(tmp_path):
    """``dls`` (default) ranks by the runtime's value, ``lp`` by the ceiling; each
    result's best row is the maximum of its own key and both keys are reported."""
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    setup = _load_setup(cfg, urdf)
    arm = setup["arm"]
    qd_plan = setup["eta_v"] * np.asarray(setup["qd_box"])
    q_ref = np.array(setup["wait_pose"])
    common = {
        "radius_m": None,
        "axis_tol_deg": 60.0,
        "box": None,
        "min_z": None,
        "samples": 1500,
        "seed": 5,
        "walk_frac": 0.6,
        "refine_top": 4,
    }
    by_dls = cws.search_wait_poses(arm, qd_plan, q_ref, **common)
    by_lp = cws.search_wait_poses(arm, qd_plan, q_ref, objective="lp", **common)
    assert by_dls["objective_key"] == "v_dir_dls" and by_lp["objective_key"] == "v_dir_lp"
    for result, key in ((by_dls, "v_dir_dls"), (by_lp, "v_dir_lp")):
        rows = result["raw"] + result["refined"]
        assert rows and all(math.isfinite(r[key]) for r in rows)
        best = max(result["refined"], key=lambda r: r[key])
        assert best[key] >= max(r[key] for r in rows) - 1e-9
        assert all(
            r["v_dir_dls"] <= r["v_dir_lp"] * (1 + cws.DLS_LP_REL_TOL) + cws.DLS_LP_ABS_TOL
            for r in rows
        )
    # The DLS optimum's DLS is at least the LP optimum's DLS (it optimised for it).
    assert max(r["v_dir_dls"] for r in by_dls["refined"]) >= (
        max(r["v_dir_dls"] for r in by_lp["refined"]) - 1e-9
    )
    with pytest.raises(SystemExit, match="objective"):
        cws.search_wait_poses(arm, qd_plan, q_ref, objective="fast", **common)


# ── (c) every reported row satisfies its own constraints ──────────────────────


def test_every_reported_row_satisfies_constraints(tmp_path):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    out = tmp_path / "out"
    rc = cws.main(
        [
            "--config-dir",
            str(cfg),
            "--urdf",
            str(urdf),
            "--radius-m",
            "0.5",
            "--axis-tol-deg",
            "20",
            "--samples",
            "3000",
            "--seed",
            "5",
            "--out",
            str(out),
        ]
    )
    assert rc == 0
    with (out / "wait_pose_candidates.csv").open() as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) > 1
    for row in rows:
        assert float(row["dist_m"]) <= 0.5 + 1e-6
        assert float(row["axis_deg"]) <= 20.0 + 1e-6
        assert row["within_limits"] == "True"
        lp, dls = float(row["v_dir_lp"]), float(row["v_dir_dls"])
        if math.isfinite(dls):
            assert dls <= lp * (1 + cws.DLS_LP_REL_TOL) + cws.DLS_LP_ABS_TOL


# ── (d) seed reproducibility ────────────────────────────────────────────────


def test_seed_reproducibility(tmp_path):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    argv_common = [
        "--config-dir",
        str(cfg),
        "--urdf",
        str(urdf),
        "--radius-m",
        "0.5",
        "--axis-tol-deg",
        "30",
        "--samples",
        "1500",
        "--seed",
        "11",
    ]
    out_a, out_b = tmp_path / "a", tmp_path / "b"
    assert cws.main([*argv_common, "--out", str(out_a)]) == 0
    assert cws.main([*argv_common, "--out", str(out_b)]) == 0
    text_a = (out_a / "wait_pose_candidates.csv").read_text()
    text_b = (out_b / "wait_pose_candidates.csv").read_text()
    assert text_a == text_b


# ── (e) --box excludes poses outside it ────────────────────────────────────


def test_box_constraint_excludes_outside_poses():
    p = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [0.5, 0.5, 0.5]])
    axis = np.tile(np.array([0.0, 0.0, 1.0]), (3, 1))
    axis_ref = np.array([0.0, 0.0, 1.0])
    p_ref = np.array([0.0, 0.0, 0.0])
    box = (np.array([-0.1, -0.1, -0.1]), np.array([0.6, 0.6, 0.6]))
    mask, _ = cws.constraint_mask(
        p, axis, p_ref, axis_ref, radius_m=None, box=box, axis_tol_deg=None, min_z=None
    )
    assert mask.tolist() == [True, False, True]


def test_box_constraint_end_to_end_keeps_every_row_inside(tmp_path):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    out = tmp_path / "out"
    box = "-0.6 -0.6 -0.6 0.6 0.6 0.6"
    rc = cws.main(
        [
            "--config-dir",
            str(cfg),
            "--urdf",
            str(urdf),
            "--box",
            box,
            "--axis-tol-deg",
            "45",
            "--samples",
            "3000",
            "--seed",
            "9",
            "--out",
            str(out),
        ]
    )
    assert rc == 0
    with (out / "wait_pose_candidates.csv").open() as fh:
        rows = [r for r in csv.DictReader(fh) if r["stage"] != "reference"]
    # the reference row is reported regardless of the box (it is the comparison baseline,
    # not a search candidate) — only the search's own raw/refined rows are constrained to it.
    assert len(rows) > 1
    for row in rows:
        assert row["in_box"] == "True"
        p = np.array([float(row["p_x"]), float(row["p_y"]), float(row["p_z"])])
        assert np.all(p >= -0.6 - 1e-6) and np.all(p <= 0.6 + 1e-6)


# ── (f) overlay_snippet parses to the best q ───────────────────────────────


def test_overlay_snippet_parses_to_the_best_pose(tmp_path):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    out = tmp_path / "out"
    rc = cws.main(
        [
            "--config-dir",
            str(cfg),
            "--urdf",
            str(urdf),
            "--radius-m",
            "0.5",
            "--axis-tol-deg",
            "30",
            "--samples",
            "2000",
            "--seed",
            "13",
            "--out",
            str(out),
        ]
    )
    assert rc == 0
    summary = yaml.safe_load((out / "wait_pose_search_summary.yaml").read_text())
    best = summary["best"]
    parsed = yaml.safe_load(best["overlay_snippet"])
    assert parsed["planner"]["wait_pose"] == pytest.approx(best["q"], abs=5e-4)


# ── (g) sim.yaml wins over _base.yaml, and the source is named ────────────


def test_sim_yaml_velocity_wins_over_base(tmp_path):
    cfg_with_sim = make_config(tmp_path / "with_sim", sim_velocity=SIM_QD)
    cfg_no_sim = make_config(tmp_path / "no_sim", sim_velocity=None)
    urdf = _urdf_file(tmp_path)
    setup_sim = cws.load_search_setup(cfg_with_sim, urdf_override=urdf)
    setup_base = cws.load_search_setup(cfg_no_sim, urdf_override=urdf)
    assert setup_sim["qd_box"] == pytest.approx(SIM_QD)
    assert setup_sim["qd_source"]["max_velocity"] == "sim.yaml"
    assert setup_base["qd_box"] == pytest.approx(BASE_QD)
    assert setup_base["qd_source"]["max_velocity"] == "_base.yaml"


# ── (h) fail-closed on a corrupt (NaN) limit ───────────────────────────────


def test_nan_velocity_limit_fails_closed(tmp_path):
    cfg = make_config(tmp_path / "share", sim_velocity=[float("nan"), 2.5, 4.0, 3.5, 4.5, 5.0])
    urdf = _urdf_file(tmp_path)
    out = tmp_path / "out"
    with pytest.raises(SystemExit, match="invalid|NaN|fails closed"):
        cws.main(
            [
                "--config-dir",
                str(cfg),
                "--urdf",
                str(urdf),
                "--axis-tol-deg",
                "20",
                "--samples",
                "200",
                "--out",
                str(out),
            ]
        )


# ── Misc CLI validation ─────────────────────────────────────────────────────


def test_evaluate_pose_wrong_length_is_rejected(tmp_path):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    with pytest.raises(SystemExit, match=f"needs {N} values"):
        cws.main(["--config-dir", str(cfg), "--urdf", str(urdf), "--evaluate-pose", "0.1 0.2"])


def test_missing_wait_pose_requires_reference_pose_override(tmp_path):
    cfg = make_config(tmp_path / "share", wait_pose=None)
    urdf = _urdf_file(tmp_path)
    with pytest.raises(SystemExit, match="wait_pose is not set"):
        cws.main(
            [
                "--config-dir",
                str(cfg),
                "--urdf",
                str(urdf),
                "--axis-tol-deg",
                "20",
                "--out",
                str(tmp_path / "out"),
            ]
        )


# ── (i) --objective robust (S8-I-2): neighbourhood p10, nominal direction, LP cap ──


def test_robust_speed_matches_direct_oracle_and_uses_the_nominal_direction(tmp_path):
    """The robust value is the p10 over ``q + δ`` of catch_speed_budget's own DLS toward
    the NOMINAL pose's -axis — recomputed here without the tool's helpers. Scoring each
    perturbed pose toward its OWN axis instead gives a different number (the ball's
    direction is fixed when it is thrown, so that variant would be the wrong quantity)."""
    urdf = _urdf_file(tmp_path)
    arm = oracle_arm(urdf)
    qd_plan = ETA_V * np.asarray(SIM_QD)
    q = np.array(WAIT_POSE)
    lo, hi = np.full(N, Q_LO), np.full(N, Q_HI)
    deltas = cws.perturbation_set(7, N, 0.1, 24)
    assert deltas.shape == (24, N) and np.all(np.abs(deltas) <= 0.1)
    assert np.array_equal(deltas, cws.perturbation_set(7, N, 0.1, 24))  # deterministic

    v_hat = -arm.frame_axis(q)
    nominal, own = [], []
    for d in deltas:
        qk = np.clip(q + d, lo, hi)
        t = arm.terms(qk, np.zeros(N))
        nominal.append(csb.directional_speed_dls(t["jp"], t["jw"], v_hat, qd_plan).v_dir_max)
        own.append(
            csb.directional_speed_dls(t["jp"], t["jw"], -arm.frame_axis(qk), qd_plan).v_dir_max
        )
    expected = float(np.percentile(nominal, cws.ROBUST_PERCENTILE))
    got = cws.robust_speed(arm, q, qd_plan, deltas, lo, hi)
    assert got == pytest.approx(expected, rel=1e-12, abs=1e-12)
    assert got != pytest.approx(float(np.percentile(own, cws.ROBUST_PERCENTILE)), rel=1e-6)
    # ε = 0 collapses the neighbourhood onto the point: robust == the point DLS.
    zero = cws.perturbation_set(7, N, 0.0, 8)
    _, dls, _ = cws.eval_self_consistent(arm, q, qd_plan)
    assert cws.robust_speed(arm, q, qd_plan, zero, lo, hi) == pytest.approx(dls, rel=1e-12)
    with pytest.raises(ValueError):
        cws.perturbation_set(1, N, -0.1, 8)


def test_sigma_min_is_the_smallest_singular_value_of_the_five_row_jacobian(tmp_path):
    urdf = _urdf_file(tmp_path)
    arm = oracle_arm(urdf)
    q = np.array(WAIT_POSE)
    t = arm.terms(q, np.zeros(N))
    expected = np.linalg.svd(np.vstack([t["jp"], t["jw"]]), compute_uv=False).min()
    assert cws.sigma_min_5(arm, q) == pytest.approx(float(expected), rel=1e-12)
    assert cws.sigma_min_5(arm, q) > 0.0


def test_robust_objective_ranks_by_its_own_key_and_never_exceeds_the_lp(tmp_path):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    setup = _load_setup(cfg, urdf)
    arm = setup["arm"]
    qd_plan = setup["eta_v"] * np.asarray(setup["qd_box"])
    q_ref = np.array(setup["wait_pose"])
    result = cws.search_wait_poses(
        arm,
        qd_plan,
        q_ref,
        radius_m=None,
        axis_tol_deg=60.0,
        box=None,
        min_z=None,
        samples=600,
        seed=5,
        walk_frac=0.6,
        refine_top=2,
        objective="robust",
        robust_eps_rad=0.1,
        robust_samples=8,
    )
    assert result["objective_key"] == "v_dir_robust"
    assert result["deltas"].shape == (8, N)
    rows = result["raw"] + result["refined"]
    assert rows
    for r in rows:
        assert math.isfinite(r["v_dir_robust"]) and math.isfinite(r["sigma_min"])
        assert r["v_dir_robust"] <= r["v_dir_lp"] + 1e-12  # capped by the nominal LP
        # the p10 of a neighbourhood is at most its best point, so it cannot beat the
        # point value's own ceiling by more than the perturbation can move it — and it
        # is recomputable from the returned perturbation set.
        assert r["v_dir_robust"] == pytest.approx(
            min(
                cws.robust_speed(
                    arm, r["q"], qd_plan, result["deltas"], result["lo"], result["hi"]
                ),
                r["v_dir_lp"],
            ),
            rel=1e-12,
        )
    best = max(result["refined"], key=lambda r: r["v_dir_robust"])
    assert best["v_dir_robust"] >= max(r["v_dir_robust"] for r in rows) - 1e-9
    # A point-DLS search of the same draw reports NaN for the robust column (not computed).
    point = cws.search_wait_poses(
        arm,
        qd_plan,
        q_ref,
        radius_m=None,
        axis_tol_deg=60.0,
        box=None,
        min_z=None,
        samples=600,
        seed=5,
        walk_frac=0.6,
        refine_top=2,
    )
    assert point["deltas"] is None
    assert all(math.isnan(r["v_dir_robust"]) for r in point["raw"] + point["refined"])
    assert all(math.isfinite(r["sigma_min"]) for r in point["raw"] + point["refined"])


def test_min_sigma_drops_candidates_and_every_reported_row_clears_it(tmp_path):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    setup = _load_setup(cfg, urdf)
    arm = setup["arm"]
    qd_plan = setup["eta_v"] * np.asarray(setup["qd_box"])
    q_ref = np.array(setup["wait_pose"])
    common = {
        "radius_m": None,
        "axis_tol_deg": 60.0,
        "box": None,
        "min_z": None,
        "samples": 800,
        "seed": 5,
        "walk_frac": 0.6,
        "refine_top": 3,
    }
    free = cws.search_wait_poses(arm, qd_plan, q_ref, **common)
    assert free["n_sigma_dropped"] == 0
    # A floor at the median sigma of the free search's own candidates must drop some.
    sigmas = sorted(r["sigma_min"] for r in free["raw"])
    floor = sigmas[len(sigmas) // 2]
    gated = cws.search_wait_poses(arm, qd_plan, q_ref, min_sigma=floor, **common)
    assert gated["n_sigma_dropped"] > 0
    for r in gated["raw"] + gated["refined"]:
        assert r["sigma_min"] >= floor - 1e-6
    assert gated["n_lp_valid"] + gated["n_sigma_dropped"] <= free["n_lp_valid"] + 1


def test_cli_robust_objective_writes_the_columns_and_the_summary(tmp_path, capsys):
    cfg = make_config(tmp_path / "share")
    urdf = _urdf_file(tmp_path)
    out = tmp_path / "out"
    rc = cws.main(
        [
            "--config-dir",
            str(cfg),
            "--urdf",
            str(urdf),
            "--radius-m",
            "0.5",
            "--axis-tol-deg",
            "30",
            "--samples",
            "600",
            "--seed",
            "17",
            "--refine-top",
            "2",
            "--objective",
            "robust",
            "--robust-eps-rad",
            "0.08",
            "--robust-samples",
            "8",
            "--out",
            str(out),
        ]
    )
    assert rc == 0
    with (out / "wait_pose_candidates.csv").open() as fh:
        rows = list(csv.DictReader(fh))
    assert {"v_dir_robust", "sigma_min"} <= set(rows[0])
    for row in rows:  # the reference row carries the robust value too
        robust, lp = float(row["v_dir_robust"]), float(row["v_dir_lp"])
        assert math.isfinite(robust) and robust <= lp + 1e-9
        assert float(row["sigma_min"]) > 0.0
    summary = yaml.safe_load((out / "wait_pose_search_summary.yaml").read_text())
    assert summary["objective"] == "robust"
    assert summary["robust"] == {
        "eps_rad": 0.08,
        "samples": 8,
        "percentile": cws.ROBUST_PERCENTILE,
        "direction": "nominal -axis(q); capped by the nominal LP",
    }
    assert math.isfinite(summary["reference_pose"]["v_dir_robust"])
    assert math.isfinite(summary["best"]["v_dir_robust"]) and summary["best"]["sigma_min"] > 0
    assert "v_dir_robust" in capsys.readouterr().out
    # --evaluate-pose prints the same two quantities for one pose.
    rc = cws.main(
        [
            "--config-dir",
            str(cfg),
            "--urdf",
            str(urdf),
            "--evaluate-pose",
            " ".join(str(v) for v in WAIT_POSE),
            "--robust-eps-rad",
            "0.08",
            "--robust-samples",
            "8",
        ]
    )
    assert rc == 0
    text = capsys.readouterr().out
    assert "v_dir_robust" in text and "sigma_min" in text
