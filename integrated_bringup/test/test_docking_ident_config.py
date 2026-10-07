"""tools/docking_ident: what the rig reads from the shipped config, and the
arithmetic between its raw verdicts and the report — without a simulator.

Three things are pinned here, all under a plain ``colcon test`` (no ``mujoco``):

* both shipped catching profiles give the rig everything it needs, with the
  lengths that have to agree;
* the rig's COPY of the simulator's ball (material constants and the way the
  contact is built live only in C++) still equals the C++;
* report.py turns planted stores into the boxes, the lateral set and the
  entrance plane they were planted to give.

The rig itself needs ``mujoco`` and is tested in test_docking_ident_rig.py.
"""

from __future__ import annotations

import dataclasses
import json
import re
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "integrated_bringup" / "tools" / "docking_ident"))
import report as rp  # noqa: E402
import rig_config as rc  # noqa: E402
import run_ident as ri  # noqa: E402

PROFILES = ["ur5e_p1b", "iiwa7_leap"]


# ── The shipped profiles ──────────────────────────────────────────────────────


@pytest.mark.parametrize("profile", PROFILES)
def test_a_shipped_profile_gives_the_rig_everything(profile):
    cfg = rc.load_rig_config(profile)
    assert Path(cfg.model_path).is_file()
    assert cfg.control_period > 0.0 and cfg.n_substeps >= 1

    hand = len(cfg.hand_joints)
    assert hand > 0 and len(cfg.q_pre) == len(cfg.q_close) == len(cfg.caging_mask) == hand
    assert any(cfg.caging_mask)
    # Every caging joint travels (rho divides by its travel).
    for i, on in enumerate(cfg.caging_mask):
        assert not on or abs(cfg.q_close[i] - cfg.q_pre[i]) > 1e-3
    assert 0.0 < cfg.eta_close <= 1.0 and cfg.t_close_e2e > 0.0
    assert len(cfg.arm_pose) == len(cfg.arm_joints) > 0

    # Each device joint is driven by exactly one simulator group, and the
    # simulator's gain override names a gain for every joint it drives.
    sim_joints = [j for g in cfg.groups for j in g.joints]
    assert len(sim_joints) == len(set(sim_joints))
    assert set(cfg.arm_joints) | set(cfg.hand_joints) <= set(sim_joints)
    if cfg.use_yaml_servo_gains:
        for group in cfg.groups:
            assert len(group.servo_kp) == len(group.servo_kd) == len(group.joints), group.name

    assert cfg.catch_parent and len(cfg.catch_xyz) == len(cfg.catch_rpy) == 3
    assert cfg.ball.kind in rc.BALL_PRESETS
    assert cfg.ball.radius > 0.0 and cfg.ball.mass > 0.0
    assert cfg.ball.contype > 0 and cfg.ball.conaffinity > 0


@pytest.mark.parametrize("profile", PROFILES)
def test_the_simulator_stack_is_defaults_then_the_profile(profile):
    config_dir = rc.CONFIG_ROOT / profile
    defaults = rc._ros_parameters(rc.SIM_DEFAULTS)
    own = rc._ros_parameters(config_dir / "mujoco_simulator.yaml")
    merged = rc.simulator_parameters(config_dir)
    # Every solver key the rig applies comes out of the stack...
    for key in ("solver", "cone", "integrator", "iterations", "ls_iterations", "tolerance"):
        assert key in merged["solver"], key
    # ...the profile's own leaves win, and the defaults fill the rest.
    for key, value in (own.get("solver") or {}).items():
        assert merged["solver"][key] == value
    for key, value in defaults["solver"].items():
        if key not in (own.get("solver") or {}):
            assert merged["solver"][key] == value
    assert merged["model_path"] == own["model_path"]


def test_a_profile_that_is_not_one_is_refused(tmp_path):
    with pytest.raises(SystemExit, match="no such profile"):
        rc.load_rig_config("no_such_robot")
    (tmp_path / "bare").mkdir()
    with pytest.raises(SystemExit):
        rc.load_rig_config("bare", config_root=tmp_path)


@pytest.mark.parametrize("profile", PROFILES)
def test_an_empty_caging_set_is_refused(profile):
    cfg = rc.load_rig_config(profile)
    with pytest.raises(SystemExit, match="caging_mask"):
        dataclasses.replace(cfg, caging_mask=tuple(False for _ in cfg.caging_mask))


def test_a_store_flown_at_other_conditions_is_refused(tmp_path):
    spec = ri._spec("v000", (0.01, 0.0), 1.0, 0.0)
    flown = {k: spec[k] for k in ("rho", "c", "delta_o", "nu", "s_pass")}
    store = _store(tmp_path / "verify_w020.json", {"v000": {**_result(True), **flown}})
    ri.fly_all(None, store, [spec], "same")  # nothing to fly, nothing to refuse
    with pytest.raises(SystemExit, match="other conditions"):
        ri.fly_all(None, store, [ri._spec("v000", (0.02, 0.0), 1.0, 0.0)], "moved")
    # A result without `accel` was a straight flight; one in the catch frame is
    # compared, one given in the world frame is not (its record is rotated).
    with pytest.raises(SystemExit, match="other conditions"):
        ri.fly_all(None, store, [{**spec, "accel": [0.0, 0.0, -9.81]}], "accel")
    ri.fly_all(None, store, [{**spec, "accel": [0.0, 0.0, -9.81], "accel_frame": "world"}], "w")
    # A spec the store does not hold is simply still to fly.
    assert "v001" not in store.items


# ── The copy of the simulator's ball ──────────────────────────────────────────


def _cpp(rel: str) -> str:
    return (REPO / "rtc_mujoco_sim" / rel).read_text()


def _numbers(block: str) -> list[float]:
    code = re.sub(r"//[^\n]*", "", block)
    return [float(v) for v in re.findall(r"-?\d+\.?\d*(?:[eE]-?\d+)?", code)]


@pytest.mark.parametrize(
    ("kind", "symbol"),
    [("tennis", "kTennisPhysics"), ("beanbag", "kBeanbagPhysics"), ("hard", "kHardPhysics")],
)
def test_the_ball_presets_are_the_simulators(kind, symbol):
    source = _cpp("src/projectile_ball.cpp")
    block = re.search(rf"constexpr ProjectileBallPhysics {symbol}\{{(.*?)\}};", source, re.S)
    assert block, symbol
    # inertia_ratio, restitution, sliding, torsional ratio, rolling ratio lead the struct.
    assert tuple(_numbers(block.group(1))[:5]) == rc.BALL_PRESETS[kind]


def test_the_ball_contact_is_built_the_simulators_way():
    ball_cpp = _cpp("src/projectile_ball.cpp")
    sim_cpp = _cpp("src/mujoco_simulator.cpp")
    node_cpp = _cpp("src/mujoco_simulator_node.cpp")
    header = _cpp("include/rtc_mujoco_sim/projectile_ball.hpp")

    def one(pattern: str, text: str) -> str:
        found = re.search(pattern, text, re.S)
        assert found, pattern
        return found.group(1)

    assert float(one(r"kContactSubsteps = ([\d.]+);", ball_cpp)) == rc.CONTACT_SUBSTEPS
    assert tuple(_numbers(one(r"solimp\{([^}]*)\}", header))) == rc.BALL_SOLIMP
    assert int(one(r"kProjectileBallGeomPriority = (\d+);", sim_cpp)) == rc.BALL_PRIORITY
    assert int(one(r"geom->condim = (\d+);", sim_cpp)) == rc.BALL_CONDIM
    assert tuple(_numbers(one(r"park_position_m\{([^}]*)\}", header))) == rc.BALL_PARK
    assert (
        int(one(r'"projectile_ball\.collision_contype", (\d+)\)', node_cpp))
        == rc.DEFAULT_BALL_CONTYPE
    )
    assert (
        int(one(r'"projectile_ball\.collision_conaffinity", (\d+)\)', node_cpp))
        == rc.DEFAULT_BALL_CONAFFINITY
    )
    # The damping search: the same model, bracket and number of bisections.
    assert "std::max(-2.0 * zeta * v - x, 0.0)" in ball_cpp
    assert "iteration < 50" in ball_cpp and "double high = 5.0;" in ball_cpp
    # stiffness = w^2, damping = 2 zeta w, w = pi / (substeps * substep).
    assert "kPi / (kContactSubsteps * substep_s)" in ball_cpp
    assert "contact.damping = 2.0 * zeta * natural_frequency;" in ball_cpp


@pytest.mark.parametrize("restitution", [0.1, 0.55, 0.75])
def test_the_damping_ratio_gives_back_its_restitution(restitution):
    zeta = rc.damping_ratio_for(restitution)
    assert rc.spring_damper_restitution(zeta) == pytest.approx(restitution, abs=2e-3)
    assert rc.spring_damper_restitution(0.0) == pytest.approx(1.0, abs=2e-3)


# ── report.py on planted stores ───────────────────────────────────────────────


def _store(path: Path, items: dict) -> rp.Store:
    path.write_text(json.dumps(items))
    return rp.Store(path)


def _result(held: bool, s_first: float | None = 0.01) -> dict:
    return {"held": held, "why": "held" if held else "left", "s_first": s_first, "stray": None}


def test_the_lattices_are_the_protocols():
    assert rp.lattice(rp.COARSE_C)[[0, -1]] == pytest.approx([0.25, 5.0])
    assert rp.lattice(rp.COARSE_D)[[0, -1]] == pytest.approx([-0.30, 0.30])
    assert rp.lattice(rp.LATERAL)[[0, 12, -1]] == pytest.approx([-0.06, 0.0, 0.06])
    assert rp.lattice(rp.FIELD_S)[[0, -1]] == pytest.approx([-0.010, 0.250])
    # Fine cells end on round numbers: [0.1 a, 0.1 (a + 1)] and [4 b, 4 (b + 1)] ms.
    assert rp.fine_c(5) == pytest.approx(0.55) and rp.fine_d(-3) == pytest.approx(-0.010)


def test_the_fine_map_covers_one_coarse_cell_around_what_held(tmp_path):
    held = np.zeros((rp.COARSE_C[2], rp.COARSE_D[2]), dtype=bool)
    assert rp.fine_cells(held) == []
    i, j = 3, 30  # c = 1.0 m/s, delta_o = 0
    held[i, j] = True
    cells = rp.fine_cells(held)
    a = sorted({a for a, _ in cells})
    b = sorted({b for _, b in cells})
    # c within 1.0 ± 0.375 -> cell centres 0.65 … 1.35; delta within ± 15 ms.
    assert (rp.fine_c(a[0]), rp.fine_c(a[-1])) == pytest.approx((0.65, 1.35))
    assert (rp.fine_d(b[0]), rp.fine_d(b[-1])) == pytest.approx((-0.014, 0.014))
    assert len(cells) == len(a) * len(b)
    # The slowest coarse speed does not ask for cells below 0.2 m/s.
    slow = np.zeros_like(held)
    slow[0, j] = True
    assert min(a for a, _ in rp.fine_cells(slow)) == rp.FINE_A_MIN


def test_a_cell_is_held_only_with_all_its_flights(tmp_path):
    items = {}
    for a in (5, 6):
        for b in range(6):
            for k in range(rp.TRIALS):
                items[f"a{a}_b{b}_k{k}"] = _result(True, s_first=0.001 * (a + b + k))
    items["a6_b2_k3"] = _result(False)  # one of four
    del items["a5_b5_k3"]  # an interrupted cell: three flown, all held
    fine = rp.FineMap(_store(tmp_path / "map_fine.json", items))
    assert fine.c == pytest.approx([0.55, 0.65]) and fine.d[0] == pytest.approx(0.002)
    assert fine.counts[1, 2] == 3 and not fine.held[1, 2]
    assert fine.trials[0, 5] == 3 and not fine.held[0, 5]
    assert fine.held.sum() == 10
    assert fine.s_first[0, 0] == pytest.approx(0.001 * (5 + 0 + 3))  # the highest of the cell

    boxes = fine.boxes()
    # 20 ms = five cells: only the a = 5 row has five in a run (b 0 … 4).
    box = boxes[0.020]
    assert (box.i0, box.i1, box.j0, box.j1) == (0, 0, 0, 4)
    assert (box.c_lo, box.c_hi) == pytest.approx((0.5, 0.6))
    assert (box.delta_o_lo, box.delta_o_hi) == pytest.approx((0.0, 0.020))
    assert boxes[0.040] is None and boxes[0.080] is None
    assert list(rp.distinct_boxes(boxes)) == ["w020"]
    assert fine.cell(box, "centre") == (5, 2)
    assert fine.cell(box, "lo_lo") == (5, 0) and fine.cell(box, "hi_hi") == (5, 4)


def test_two_widths_asking_for_one_box_measure_it_once():
    box = rp.cs.Box(0.5, 0.6, 0.0, 0.08, 0, 0, 0, 19)
    other = rp.cs.Box(0.5, 0.9, 0.0, 0.02, 0, 3, 0, 4)
    assert rp.distinct_boxes({0.02: other, 0.04: box, 0.08: box}) == {"w020": other, "w040": box}
    assert rp.distinct_boxes({0.02: None, 0.04: None}) == {}


def _plant_lateral(first: set[tuple[int, int]], lost: dict | None = None) -> dict:
    """A lateral store flown to the end: the first pass at every cell (held at
    ``first``), then every candidate under every condition. A fly-in holds
    where the first pass did, except the first ``lost[(q, i, j)]`` fly-ins
    after fly-in 0 of that condition and cell."""
    lost = lost or {}
    n = rp.LATERAL[2]
    items = {f"q0_x{i}_y{j}_k0": _result((i, j) in first) for i in range(n) for j in range(n)}
    candidates = np.zeros((n, n), dtype=bool)
    for i, j in first:
        r = rp.LATERAL_RINGS
        candidates[max(0, i - r) : i + r + 1, max(0, j - r) : j + r + 1] = True
    for i, j in zip(*np.nonzero(candidates), strict=True):
        for q in range(len(rp.CONDITIONS)):
            for k in range(rp.LATERAL_TRIALS):
                held = (i, j) in first and not 1 <= k <= lost.get((q, int(i), int(j)), 0)
                items[f"q{q}_x{i}_y{j}_k{k}"] = _result(bool(held))
    return items


def test_the_lateral_candidates_are_the_first_pass_and_two_cells_around_it(tmp_path):
    store = _store(tmp_path / "lateral_w020.json", _plant_lateral({(12, 12), (0, 24)}))
    candidates = rp.lateral_candidates(store)
    # 5 x 5 around the cell in the middle; 3 x 3 of the corner's 5 x 5 is on the lattice.
    assert candidates.sum() == 25 + 9
    assert candidates[10:15, 10:15].all() and candidates[0:3, 22:25].all()
    assert not candidates[9, 12] and not candidates[3, 24]
    # A fly-in other than the first pass's does not make a candidate.
    other = {"q0_x5_y5_k1": _result(True), "q1_x5_y5_k0": _result(True)}
    assert not rp.lateral_candidates(_store(tmp_path / "other.json", other)).any()


def test_a_lateral_cell_is_judged_by_its_hold_rate_against_the_reference_cell(tmp_path):
    full = rp.LATERAL_TRIALS * len(rp.CONDITIONS)
    assert (full, rp.LATERAL_DROP) == (40, 0.05)
    first = {(12, 12), (13, 12), (14, 12), (15, 12)}
    lost = {
        (3, 13, 12): 2,  # 38 of 40: two below the reference, within 5 points
        (1, 14, 12): 3,  # 37 of 40: beyond them
        (0, 15, 12): 1,  # 36 of 40, and one of them in the centre condition
        (2, 15, 12): 3,
    }
    verdict = rp.lateral_verdict(
        _store(tmp_path / "lateral_w020.json", _plant_lateral(first, lost))
    )
    assert verdict.complete and verdict.reference == (12, 12)
    assert verdict.reference_counts == (8, 8, 8, 8, 8) and verdict.reference_rate == 1.0
    assert verdict.counts[12, 12] == 40 and verdict.counts[13, 12] == 38
    assert verdict.counts[14, 12] == 37 and verdict.counts[15, 12] == 36
    assert verdict.held[12, 12] and verdict.held[13, 12]
    assert not verdict.held[14, 12] and not verdict.held[15, 12]
    assert verdict.held.sum() == 2
    # "Every fly-in of every condition holds" would keep the one cell only.
    assert int((verdict.counts == full).sum()) == 1
    # A candidate around the four that never held is flown, and is not in the set.
    assert verdict.candidates[10, 10] and verdict.flown[10, 10] == full
    assert verdict.counts[10, 10] == 0 and not verdict.held[10, 10]


def test_the_reference_cell_is_where_the_centre_condition_holds_most(tmp_path):
    # (12, 12) loses one centre fly-in; (13, 12) loses three elsewhere but none there.
    lost = {(0, 12, 12): 1, (4, 13, 12): 3}
    store = _store(tmp_path / "lateral_w020.json", _plant_lateral({(12, 12), (13, 12)}, lost))
    verdict = rp.lateral_verdict(store)
    assert verdict.reference == (13, 12) and verdict.reference_rate == pytest.approx(37 / 40)
    assert verdict.reference_counts == (8, 8, 8, 8, 5)
    # The other cell holds more often than the reference: it is in the set.
    assert verdict.held[12, 12] and verdict.held[13, 12] and verdict.held.sum() == 2
    # Among cells the centre condition holds equally at, the higher rate is the reference.
    lost = {(4, 13, 12): 3, (4, 12, 12): 1}
    store = _store(tmp_path / "again.json", _plant_lateral({(12, 12), (13, 12)}, lost))
    assert rp.lateral_verdict(store).reference == (12, 12)


def test_an_interrupted_lateral_stage_has_no_verdict(tmp_path):
    items = _plant_lateral({(12, 12)})
    whole = rp.lateral_verdict(_store(tmp_path / "whole.json", items))
    assert whole.complete and whole.held.sum() == 1
    del items["q4_x11_y11_k7"]  # a candidate's last fly-in
    cut = rp.lateral_verdict(_store(tmp_path / "cut.json", items))
    assert not cut.complete and cut.missing == 1 and not cut.held.any()
    items = _plant_lateral({(12, 12)})
    del items["q0_x0_y0_k0"]  # the first pass did not reach a cell
    assert rp.lateral_verdict(_store(tmp_path / "first.json", items)).missing == 1
    # A store of the rule before this one (four fly-ins, stopped at the first failure).
    old = {
        f"q{q}_x{i}_y{j}_k{k}": _result(True)
        for q in range(len(rp.CONDITIONS))
        for i in range(rp.LATERAL[2])
        for j in range(rp.LATERAL[2])
        for k in range(4)
    }
    verdict = rp.lateral_verdict(_store(tmp_path / "old.json", old))
    assert not verdict.complete and not verdict.held.any()


def test_a_hand_that_holds_nowhere_has_no_lateral_set(tmp_path):
    items = {key: _result(False) for key in _plant_lateral({(12, 12)})}
    verdict = rp.lateral_verdict(_store(tmp_path / "lateral_w020.json", items))
    assert verdict.complete and verdict.reference is None and not verdict.held.any()
    # Held by the first pass and never again: one in forty is a rate, not a set of
    # cells that never held around it.
    items = _plant_lateral({(12, 12)}, {(q, 12, 12): 7 for q in range(len(rp.CONDITIONS))})
    verdict = rp.lateral_verdict(_store(tmp_path / "once.json", items))
    assert verdict.reference == (12, 12) and verdict.counts[12, 12] == 5
    assert verdict.held.sum() == 1


def _plant_rings(failures: dict[int, int], rings: int = 4) -> dict:
    """Speed rings 0 … ``rings`` − 1 (ring 0: no lateral speed), all held but
    for ``failures[ring]`` fly-ins."""
    items = {}
    for r in range(rings):
        left = failures.get(r, 0)
        for q in range(len(rp.CONDITIONS)):
            for d in range(rp.VPERP_DIRECTIONS):
                for k in range(rp.TRIALS):
                    items[f"q{q}_r{r}_d{d}_k{k}"] = _result(left <= 0)
                    left -= 1
    return items


def test_a_speed_ring_is_judged_by_its_hold_rate_against_the_zero_speed_ring(tmp_path):
    ring = len(rp.CONDITIONS) * rp.VPERP_DIRECTIONS * rp.TRIALS
    assert ring == 160
    # The reference loses 8 (0.95). Within 5 points: 0.90 = 16 lost. Ring 1 loses
    # 16 and passes, ring 2 loses 17 and does not, ring 3 holds everything.
    store = _store(tmp_path / "vperp_w020.json", _plant_rings({0: 8, 1: 16, 2: 17}))
    held, flown = rp.vperp_counts(store)
    assert flown[:4].tolist() == [ring] * 4 and not flown[4:].any()
    assert held[:4].tolist() == [ring - 8, ring - 16, ring - 17, ring]
    passed = rp.cs.rings_within_drop(held, flown, rp.VPERP_DROP)
    assert passed[:4].tolist() == [True, True, False, True]
    # Ring 1 is 0.05 m/s: the limit is its outer edge, and ring 3 does not count.
    assert rp.cs.largest_held_radius(rp.lattice(rp.VPERP), passed[1:]) == pytest.approx(0.075)
    # A store flown under the earlier rule is refused, not read as these rings.
    old = _store(tmp_path / "old.json", {"q0_m0_d0_k0": _result(True)})
    with pytest.raises(SystemExit, match="fly vperp again"):
        rp.vperp_counts(old)


def test_the_speed_rings_are_aimed_at_the_centre_of_the_lateral_set(tmp_path):
    box = _plant_identification(tmp_path)
    fine = rp.FineMap(rp.Store(tmp_path / "map_fine.json"))
    assert rp.vperp_aim(rp.identify(tmp_path, "w020", box, fine)) == pytest.approx((0.0, 0.0))
    # The set moved 15 mm along +x and 10 mm along −y: so does the aim.
    path = tmp_path / "lateral_w020.json"
    moved = _plant_lateral({(i + 3, j - 2) for i in range(10, 15) for j in range(10, 15)})
    path.write_text(json.dumps(moved))
    assert rp.vperp_aim(rp.identify(tmp_path, "w020", box, fine)) == pytest.approx((0.015, -0.010))
    # No lateral set, no aim (the stage is skipped).
    path.write_text(json.dumps({key: _result(False) for key in moved}))
    assert rp.vperp_aim(rp.identify(tmp_path, "w020", box, fine)) is None


def _plant_identification(directory: Path) -> rp.cs.Box:
    """A hand whose every answer can be read off: a box of 3 x 5 fine cells, a
    lateral square of 5 x 5 cells, two speed rings, and a palm with a rim."""
    directory.mkdir(parents=True, exist_ok=True)
    fine = {}
    for a in (5, 6, 7):
        for b in range(5):
            for k in range(rp.TRIALS):
                fine[f"a{a}_b{b}_k{k}"] = _result(True, s_first=0.004)
    _store(directory / "map_fine.json", fine)
    lateral = _plant_lateral({(i, j) for i in range(10, 15) for j in range(10, 15)})
    _store(directory / "lateral_w020.json", lateral)
    # The zero-speed ring and the rings at 0.05 and 0.10 m/s hold; 0.15 loses 9
    # of 160, one more than 5 points of rate allow.
    _store(directory / "vperp_w020.json", _plant_rings({3: 9}))
    xs, ss = rp.lattice(rp.FIELD_XY), rp.lattice(rp.FIELD_S)
    gx, gy, gs = np.meshgrid(xs, xs, ss, indexing="ij")
    # The palm at and below s = 0, and a rim 30 mm high beyond 40 mm from the axis.
    hand = (gs <= 1e-9) | ((np.hypot(gx, gy) > 0.040) & (gs <= 0.030 + 1e-9))
    np.savez_compressed(
        directory / "static.npz", xs=xs, ys=xs, ss=ss, hand=hand, stray=np.zeros_like(hand)
    )
    verify = {f"v{n:03d}": {**_result(True), "s_pass": 0.001} for n in range(rp.VERIFY_N)}
    verify["v007"] = {**_result(False), "s_pass": 0.001}
    _store(directory / "verify_w020.json", verify)
    return rp.FineMap(rp.Store(directory / "map_fine.json")).boxes()[0.020]


def test_one_box_is_identified_all_the_way(tmp_path):
    box = _plant_identification(tmp_path)
    fine = rp.FineMap(rp.Store(tmp_path / "map_fine.json"))
    assert (box.c_lo, box.c_hi) == pytest.approx((0.5, 0.8))
    assert (box.delta_o_lo, box.delta_o_hi) == pytest.approx((0.0, 0.020))
    ident = rp.identify(tmp_path, "w020", box, fine)

    # The lateral square of cells ±10 mm: ±12.5 mm with its cells' extent.
    polygon = ident["polygon"]
    assert ident["lateral_held"].sum() == 25
    assert ident["circle"][0] == pytest.approx((0.0, 0.0)) and ident["circle"][1] == pytest.approx(
        0.0125
    )
    for face in (0, 2, 4, 6):
        assert 0.0115 < polygon.offsets[face] <= 0.0125 + 1e-9
    assert ident["v_perp_max"] == pytest.approx(0.125)  # rings 0.05 and 0.10 pass
    assert ident["vperp_pass"][:4].tolist() == [True, True, True, False]
    assert ident["tan_max"] == pytest.approx(0.125 / 0.5)

    # Tilt 0.25 rounds up to 0.3. From the set's corner (12.5 mm out, 17.7 mm
    # from the axis diagonally) a line at 0.3 is over the rim (beyond 40 mm,
    # read from 35 mm on) well above the rim's 30 mm: the palm alone decides.
    whole = ident["entrance"]
    assert whole.holds and whole.s_ent == pytest.approx(0.001) and not whole.scan_limited
    rows = ident["rows"]
    assert [r.c_lo for r in rows] == pytest.approx([0.5, 0.6, 0.7])
    assert all(r.s_ent == pytest.approx(0.001) for r in rows)
    assert rows[0].width == pytest.approx(0.020 - 0.001 * (1 / 0.5 - 1 / 0.8))
    assert ident["widest"] == rows[-1]
    corridor = ident["corridor"]
    assert 0.030 < corridor.r_ent <= 0.040 and corridor.tan_theta >= 0.0
    assert ident["verify"]["n"] == rp.VERIFY_N and ident["verify"]["held"] == rp.VERIFY_N - 1
    assert ident["verify"]["lower"] == pytest.approx(
        rp.cs.clopper_pearson_lower(rp.VERIFY_N - 1, rp.VERIFY_N)
    )


def _plant_accel(directory: Path) -> None:
    """Every verification condition again at each level: the first level loses
    three that the straight flight held and holds the one it dropped."""
    again = {
        f"g{g}_v{n:03d}": {**_result(True), "accel": [0.1 * g, 0.0, -5.0]}
        for g in range(len(rp.ACCEL_CASES))
        for n in range(rp.VERIFY_N)
    }
    for n in (1, 2, 3):
        again[f"g0_v{n:03d}"] = _result(False)
    again["g1_v007"] = _result(False)
    _store(directory / "accel_w020.json", again)


def test_the_accelerated_flights_are_compared_condition_by_condition(tmp_path):
    box = _plant_identification(tmp_path)
    fine = rp.FineMap(rp.Store(tmp_path / "map_fine.json"))
    assert "accel" not in rp.identify(tmp_path, "w020", box, fine)  # not flown
    _plant_accel(tmp_path)
    first, second, third = rp.identify(tmp_path, "w020", box, fine)["accel"]
    assert [r["label"] for r in (first, second, third)] == [c[0] for c in rp.ACCEL_CASES]
    # What the report shows is the vector the flights record, not the protocol's.
    assert first["accel"] == [0.0, 0.0, -5.0] and third["accel"] == [0.2, 0.0, -5.0]
    assert (third["held"], third["lost"], third["gained"]) == (rp.VERIFY_N, 0, 1)
    assert (first["n"], first["held"]) == (rp.VERIFY_N, rp.VERIFY_N - 3)
    assert (first["lost"], first["gained"]) == (3, 1)  # v007 held here, not straight
    assert first["lower"] == pytest.approx(
        rp.cs.clopper_pearson_lower(rp.VERIFY_N - 3, rp.VERIFY_N)
    )
    assert (second["held"], second["lost"], second["gained"]) == (rp.VERIFY_N - 1, 0, 0)
    # A level that was not flown is absent, and without the straight flights
    # there is nothing to compare with.
    path = tmp_path / "accel_w020.json"
    items = {k: v for k, v in json.loads(path.read_text()).items() if k.startswith("g1_")}
    path.write_text(json.dumps(items))
    assert [r["label"] for r in rp.identify(tmp_path, "w020", box, fine)["accel"]] == [
        rp.ACCEL_CASES[1][0]
    ]
    (tmp_path / "verify_w020.json").unlink()
    assert "accel" not in rp.identify(tmp_path, "w020", box, fine)


def test_stages_that_have_not_run_are_absent_not_guessed(tmp_path):
    box = _plant_identification(tmp_path)
    fine = rp.FineMap(rp.Store(tmp_path / "map_fine.json"))
    (tmp_path / "verify_w020.json").unlink()
    (tmp_path / "static.npz").unlink()
    ident = rp.identify(tmp_path, "w020", box, fine)
    assert "v_perp_max" in ident and "entrance" not in ident and "verify" not in ident
    (tmp_path / "lateral_w020.json").unlink()
    assert set(rp.identify(tmp_path, "w020", box, fine)) == {"tag", "box"}


def test_the_report_states_what_was_planted(tmp_path, monkeypatch):
    _plant_identification(tmp_path / "some_robot")
    monkeypatch.setenv("DATA", str(tmp_path))
    assert rp.main(["some_robot"]) == 0
    text = (tmp_path / "some_robot" / "report.md").read_text()
    assert "| 20 ms | 0.5 – 0.8 | +0 … +20 | 20 | 15 |" in text
    assert "| 40 ms | 없음 |" in text
    assert "- lateral: 후보 81 셀" in text and "- 집합: 25 셀 (6.2 cm²)" in text
    assert "유지율 40/40 = 1.000" in text and "5 %p" in text
    assert "무접촉 일관성: 성립" in text and "= 1.0 mm" in text
    assert "= 0.125 m/s" in text
    assert "| 0.00 | 160 | 160 | 1.000 |" in text and "| 기준 |" in text
    assert "| 0.15 | 160 | 151 | 0.944 |" in text and "| 아니오 |" in text
    assert f"{rp.VERIFY_N} 조건 가운데 유지 {rp.VERIFY_N - 1}" in text
    assert "상대 가속도" not in text
    _plant_accel(tmp_path / "some_robot")
    assert rp.main(["some_robot"]) == 0
    text = (tmp_path / "some_robot" / "report.md").read_text()
    assert f"| 접근축 5 | (+0.00, +0.00, -5.00) | {rp.VERIFY_N} | {rp.VERIFY_N - 3} |" in text
    assert "| 3 | 1 |" in text
    assert f"| 접근축 9.81 | (+0.10, +0.00, -5.00) | {rp.VERIFY_N} | {rp.VERIFY_N - 1} |" in text
    assert f"| 대기 자세의 중력 | (+0.20, +0.00, -5.00) | {rp.VERIFY_N} | {rp.VERIFY_N} |" in text
    monkeypatch.delenv("DATA")
    with pytest.raises(SystemExit, match="DATA is not set"):
        rp.data_dir("some_robot")


class _DiscPool:
    """Stands in for the worker pool: a fly-in holds within 11 mm of the axis."""

    def __init__(self) -> None:
        self.flown: list[str] = []

    def imap_unordered(self, _, specs, chunksize=1):
        for spec in specs:
            self.flown.append(spec["id"])
            held = float(np.hypot(*spec["rho"])) < 0.011
            flown = {k: spec[k] for k in ("rho", "c", "delta_o", "nu", "s_pass")}
            yield spec["id"], {**_result(held), **flown}


def test_the_lateral_stage_flies_every_candidate_forty_times(tmp_path, monkeypatch, capsys):
    _plant_identification(tmp_path)
    (tmp_path / "lateral_w020.json").unlink()
    pool = _DiscPool()
    ri.stage_lateral("some_robot", tmp_path, pool)
    store = rp.Store(tmp_path / "lateral_w020.json")
    n = rp.LATERAL[2]
    candidates = rp.lateral_candidates(store)
    full = rp.LATERAL_TRIALS * len(rp.CONDITIONS)
    # The first pass at every cell, then what is left of forty at each candidate
    # — also at those whose first fly-in, or any other, did not hold.
    assert pool.flown[: n * n] == [f"q0_x{i}_y{j}_k0" for i in range(n) for j in range(n)]
    assert len(pool.flown) == len(set(pool.flown)) == n * n + int(candidates.sum()) * (full - 1)
    verdict = rp.lateral_verdict(store)
    assert verdict.complete and (verdict.flown[candidates] == full).all()
    assert (verdict.flown[~candidates] == 1).all()
    assert 0 < verdict.held.sum() < candidates.sum()
    assert (verdict.counts[candidates & ~verdict.held] < full).all()
    # A fly-in's conditions follow from its id alone: the stage run again flies nothing.
    again = _DiscPool()
    ri.stage_lateral("some_robot", tmp_path, again)
    assert again.flown == []
    capsys.readouterr()
    # --box keeps the per-box stages to the boxes named; a box the map lacks is refused.
    monkeypatch.setattr(ri, "_ONLY_BOXES", ("w020",))
    assert list(ri._boxes(tmp_path)[1]) == ["w020"]
    monkeypatch.setattr(ri, "_ONLY_BOXES", ("w040",))
    with pytest.raises(SystemExit, match="w040"):
        ri._boxes(tmp_path)


def test_a_lateral_store_of_another_rule_is_reported_as_not_flown(tmp_path, monkeypatch):
    _plant_identification(tmp_path / "some_robot")
    path = tmp_path / "some_robot" / "lateral_w020.json"
    items = json.loads(path.read_text())
    path.write_text(json.dumps({k: v for k, v in items.items() if int(k.split("_k")[1]) < 4}))
    monkeypatch.setenv("DATA", str(tmp_path))
    assert rp.main(["some_robot"]) == 0
    text = (tmp_path / "some_robot" / "report.md").read_text()
    assert "끝까지 날려지지 않았다" in text and "무접촉 일관성" not in text


def test_an_empty_lateral_set_is_reported_as_empty(tmp_path, monkeypatch):
    _plant_identification(tmp_path / "some_robot")
    path = tmp_path / "some_robot" / "lateral_w020.json"
    # Nothing holds at all. (One condition that never holds lowers every cell's
    # rate alike, the reference cell's too: the report shows that rate.)
    items = {key: _result(False) for key in json.loads(path.read_text())}
    path.write_text(json.dumps(items))
    monkeypatch.setenv("DATA", str(tmp_path))
    assert rp.main(["some_robot"]) == 0
    assert "가 빈다" in (tmp_path / "some_robot" / "report.md").read_text()
