#!/usr/bin/env python3
"""Gate-catchable map: the rest of the planner's gate chain over a kinematic map (S3.5b).

``catchability_map`` answers "is there a good catch posture". This takes its
accepted candidates and asks what the planner asks next (plan §11, L3 §4):

    commit lead (§4.11) is there a plan early enough to command the hand: the closing
                        time, the arm delay and a margin all fit between the first plan and t_c
    reach time (§4.3)   can the arm be at q* by t_c, starting at rest in the wait pose
    γ window   (§4.5)   is there a γ both the hand (closing time) and the arm (speed) allow
    stopping   (§4.9)   does the arm come to rest inside the workspace afterwards

The reach-time, γ-window and stopping-point VERDICTS are not computed here.
They come from ``catch_gate_batch`` (rtc_controllers), which calls the runtime
functions of ``time_feasibility.hpp`` — the map has to describe the system that
flies, and a python re-derivation would describe a different one (G3-I). What
python owns is everything those functions take as input and leave as output:

- the commit lead. It is one subtraction and has no runtime function to share
  (``CommitDue`` compares against a T_freeze that is still TBD), so the lower bound
  T_close,tot + T_arm + T_margin of L3 §4.11 is applied here;
- q̇ᵘ, the damped least-squares unit-speed joint velocity behind v_dir,max. It has
  no runtime producer before S6.2, so it is computed here and handed over;
- whether p_stop is inside the workspace. ``planner.workspace.catch_box`` is TBD,
  so the bound is the reach sphere and the floor — the ones the kinematic map used;
- a second, PROVISIONAL reach layer (D-16 revision, plan §9). The shipped
  acceleration limit is one constant box from a worst-sign sufficient condition.
  This layer instead asks whether THIS move fits the torque limits: all joints
  follow one bang-bang / trapezoid path profile from the wait pose to q*, and the
  largest path acceleration with |M q̈ + h| ≤ η_τ τ_max along the whole move is
  found by bisection (rotor inertia included). It is a sufficient condition — a
  straight, synchronised joint path is not time-optimal — and it has no runtime
  counterpart yet, which is why it is a layer next to the box and not a replacement.

Not judged: the γ rollout (L3 §4.8, no function before S6.3). An open cell is
therefore PASS(provisional); an empty map is conclusive.

Every robot value is an argument. Both layers are always reported, and every
denominator is the full throw grid.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
from collections import Counter, defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis.catch_speed_budget import (
    DEFAULT_CATCH_FRAME,
    ArmKinematics,
    directional_speed_lp,
    dls_unit_velocity,
    extra_frame_from_params,
)
from rtc_tools.analysis.catchability_map import distribution
from rtc_tools.analysis.derive_accel_limits import (
    arm_spec_from_params,
    load_robot_params,
    resolve_urdf_text,
)

GATE_EXECUTABLE = "catch_gate_batch"
DEFAULT_FK_TOLERANCE_M = 2.5e-3
DEFAULT_PROFILE_SAMPLES = 16
DEFAULT_ACCEL_TOLERANCE = 1e-3  # relative, on the path acceleration (~0.05 % on time)
PATH_ACCEL_BOUNDS = (1e-3, 1e5)  # [1/s²] on the normalised path; the bracket of the bisection
_TRUE = ("1", "True", "true")

LAYERS = ("box", "torque")
REASON_NONE = "none"
REASON_COMMIT = "commit_lead"
REASON_STOP = "stop_outside_workspace"
REASON_REACH_TORQUE = "reach_time_torque"
REASON_REACH_TORQUE_INFEASIBLE = "reach_torque_infeasible"


# ── Torque-checked reach time (provisional layer) ─────────────────────────────


def profile_duration(path_accel: float, path_speed_cap: float) -> float:
    """Rest-to-rest time over the unit path at ``path_accel``, peak speed capped."""
    if math.sqrt(path_accel) <= path_speed_cap:
        return 2.0 / math.sqrt(path_accel)
    return 1.0 / path_speed_cap + path_speed_cap / path_accel


def profile_samples(path_accel: float, path_speed_cap: float, count: int) -> list[tuple]:
    """(s, ṡ, s̈) over the profile, both signs at each phase switch.

    Acceleration is piecewise constant, so torque extremes sit at the ends of
    each phase (speed and posture terms are monotone inside one only to first
    order — hence interior samples as well).
    """
    peak = min(math.sqrt(path_accel), path_speed_cap)
    t_ramp = peak / path_accel
    s_ramp = 0.5 * path_accel * t_ramp**2
    s_cruise = max(0.0, 1.0 - 2.0 * s_ramp)
    out = []
    for k in range(count + 1):
        t = t_ramp * k / count
        out.append((0.5 * path_accel * t**2, path_accel * t, path_accel))
        out.append((1.0 - 0.5 * path_accel * t**2, path_accel * t, -path_accel))
    if s_cruise > 0.0:
        for k in range(count + 1):
            out.append((s_ramp + s_cruise * k / count, peak, 0.0))
    return out


def torque_reach_time(
    arm: ArmKinematics,
    q_from: np.ndarray,
    q_to: np.ndarray,
    qd_limit: np.ndarray,
    tau_limit: np.ndarray,
    samples: int = DEFAULT_PROFILE_SAMPLES,
    tolerance: float = DEFAULT_ACCEL_TOLERANCE,
) -> float:
    """Shortest synchronised rest-to-rest move inside the torque and speed limits [s].

    NaN if even a near-static move violates the torque limits (gravity alone
    is over the limit somewhere on the path); 0 for a null move.
    """
    delta = np.asarray(q_to, dtype=float) - np.asarray(q_from, dtype=float)
    if not np.all(np.isfinite(delta)):
        return float("nan")
    span = float(np.max(np.abs(delta)))
    if span == 0.0:
        return 0.0
    speed_cap = float(np.min(qd_limit[delta != 0.0] / np.abs(delta[delta != 0.0])))

    def feasible(path_accel: float) -> bool:
        for s, sd, sdd in profile_samples(path_accel, speed_cap, samples):
            tau = arm.joint_torques(q_from + s * delta, sd * delta, sdd * delta)
            if np.any(np.abs(tau) > tau_limit):
                return False
        return True

    lo, hi = PATH_ACCEL_BOUNDS
    if not feasible(lo):
        return float("nan")
    if feasible(hi):
        return profile_duration(hi, speed_cap)
    while hi / lo > 1.0 + tolerance:
        mid = math.sqrt(lo * hi)
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    return profile_duration(lo, speed_cap)


# ── Per-candidate inputs for the C++ judge ────────────────────────────────────


@dataclass(frozen=True)
class GateInputs:
    """What one accepted kinematic candidate hands to ``catch_gate_batch``."""

    row: Mapping[str, str]
    q_star: np.ndarray
    qd_unit: np.ndarray
    jp_qd_unit: np.ndarray
    v_dir_max_lp: float
    fk_residual_m: float


def gate_inputs(
    arm: ArmKinematics, row: Mapping[str, str], qd_plan: np.ndarray, damping: float
) -> GateInputs:
    n = int(row["nv"])
    if n != arm.n:
        raise SystemExit(f"candidate nv = {n} but the arm has {arm.n} joints")
    q = np.array([float(row[f"q{i}"]) for i in range(n)])
    v = np.array([float(row[f"v_model_{a}"]) for a in "xyz"])
    p = np.array([float(row[f"p_model_{a}"]) for a in "xyz"])
    speed = float(np.linalg.norm(v))
    if not speed > 0.0:
        # The kinematic judge rejects a resting ball (kSpeedTooLow), so an accepted
        # row without a velocity is a broken generator, not a candidate.
        raise SystemExit(f"candidate {row['id']}: accepted with zero ball velocity")
    v_hat = v / speed
    terms = arm.terms(q, np.zeros(n))
    qd_unit = dls_unit_velocity(terms["jp"], terms["jw"], v_hat, damping)
    lp, _ = directional_speed_lp(terms["jp"], terms["jw"], v_hat, qd_plan)
    return GateInputs(
        row=row,
        q_star=q,
        qd_unit=qd_unit,
        jp_qd_unit=terms["jp"] @ qd_unit,
        v_dir_max_lp=lp.v_dir_max,
        fk_residual_m=float(np.linalg.norm(arm.frame_position(q) - p)),
    )


def gate_candidate_csv(inputs: Sequence[GateInputs]) -> str:
    """The judge's candidate CSV. ``repr`` round-trips a double exactly."""
    n = len(inputs[0].q_star)
    head = ["id", "seed_id", "t_c_s"]
    head += [f"p_c_{a}" for a in "xyz"] + [f"v_{a}" for a in "xyz"] + [f"jpu_{a}" for a in "xyz"]
    head += [f"qs{i}" for i in range(n)] + [f"qu{i}" for i in range(n)]
    lines = [",".join(head)]
    for item in inputs:
        row = item.row
        cells = [row["id"], row["seed_id"], repr(float(row["t_c_s"]))]
        cells += [repr(float(row[f"p_model_{a}"])) for a in "xyz"]
        cells += [repr(float(row[f"v_model_{a}"])) for a in "xyz"]
        cells += [repr(float(x)) for x in item.jp_qd_unit]
        cells += [repr(float(x)) for x in item.q_star] + [repr(float(x)) for x in item.qd_unit]
        lines.append(",".join(cells))
    return "\n".join(lines) + "\n"


@dataclass(frozen=True)
class JudgeSettings:
    """The constants ``catch_gate_batch`` requires, none defaulted."""

    qdot_max: np.ndarray
    qddot_max: np.ndarray
    eta_v: float
    v_max: float
    d_eff: float
    t_close_total: float
    gamma_margin: float
    a_dec: float
    first_plan_s: float
    t_arm_s: float
    t_margin_s: float

    def argv(self) -> list[str]:
        def vec(values) -> str:
            return " ".join(repr(float(x)) for x in values)

        pairs = {
            "--qdot-max": vec(self.qdot_max),
            "--qddot-max": vec(self.qddot_max),
            "--eta-v": repr(self.eta_v),
            "--v-max": repr(self.v_max),
            "--d-eff": repr(self.d_eff),
            "--t-close-total": repr(self.t_close_total),
            "--gamma-margin": repr(self.gamma_margin),
            "--a-dec": repr(self.a_dec),
            "--first-plan-s": repr(self.first_plan_s),
            "--t-arm-s": repr(self.t_arm_s),
            "--t-margin-s": repr(self.t_margin_s),
        }
        return [token for pair in pairs.items() for token in pair]


def run_gate_judge(
    judge: Path, candidates_csv: Path, seeds_csv: Path, out_csv: Path, settings: JudgeSettings
) -> list[dict]:
    cmd = [str(judge), "--candidates", str(candidates_csv), "--seeds", str(seeds_csv)]
    cmd += ["--out", str(out_csv), *settings.argv()]
    done = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if done.returncode != 0:
        raise SystemExit(f"{judge.name} failed ({done.returncode}): {done.stderr.strip()}")
    with out_csv.open() as handle:
        return list(csv.DictReader(handle))


# ── Verdict layers ────────────────────────────────────────────────────────────


def stop_inside_workspace(
    p_stop_model: np.ndarray,
    p_catch_model_z: float,
    p_catch_world_z: float,
    reach_centre: np.ndarray,
    reach_m: float,
    floor_world_z_m: float,
) -> bool:
    """Reach sphere and floor — the stand-in for the TBD ``planner.workspace.catch_box``.

    Model world and world differ by a yaw and a translation only, so heights
    transfer by difference.
    """
    if not np.all(np.isfinite(p_stop_model)):
        return False
    inside_sphere = float(np.linalg.norm(p_stop_model - reach_centre)) <= reach_m
    stop_world_z = p_catch_world_z + (float(p_stop_model[2]) - p_catch_model_z)
    return inside_sphere and stop_world_z >= floor_world_z_m


def layer_reasons(
    judged: Mapping[str, str],
    stop_ok: bool,
    torque_time_s: float,
    reach_budget_s: float,
    commit_ok: bool = True,
) -> dict[str, str]:
    """First gate, in planner order, that stops the candidate — per reach layer.

    ``judged`` is one ``catch_gate_batch`` row. The box layer is the judge's own
    chain plus the workspace bound; the torque layer swaps only the reach gate.
    """
    gamma_ok = judged["gamma_ok"] in _TRUE
    stop_valid = judged["stop_gmin_valid"] in _TRUE
    gamma_reason = (
        "gamma_invalid"
        if judged["window_input_invalid"] in _TRUE
        or judged["dir_limits_invalid"] in _TRUE
        or judged["dir_input_invalid"] in _TRUE
        or judged["dir_undetermined"] in _TRUE
        else "gamma_window_empty"
    )

    def after_reach() -> str:
        if not gamma_ok:
            return gamma_reason
        if not stop_valid:
            return "stop_invalid"
        return REASON_NONE if stop_ok else REASON_STOP

    if not commit_ok:
        return dict.fromkeys(LAYERS, REASON_COMMIT)
    box = judged["reason_name"]
    if box in (REASON_NONE, "gamma_invalid", "gamma_window_empty", "stop_invalid"):
        box = after_reach()
    if not math.isfinite(torque_time_s):
        torque = REASON_REACH_TORQUE_INFEASIBLE
    elif torque_time_s > reach_budget_s:
        torque = REASON_REACH_TORQUE
    else:
        torque = after_reach()
    return {"box": box, "torque": torque}


def propose_wait_pose(postures: Sequence[np.ndarray]) -> np.ndarray | None:
    """Per-joint midrange of the given q*: the rest pose with the smallest worst-case move."""
    if not postures:
        return None
    stack = np.vstack(postures)
    return 0.5 * (stack.min(axis=0) + stack.max(axis=0))


# ── Aggregation (denominator = the full grid, always) ─────────────────────────


def cell_table(rows: Sequence[Mapping], throws: Mapping[int, Mapping[str, str]]) -> list[dict]:
    """(release height × distance): grid throws, kinematic throws, open throws per layer."""
    grid: dict = defaultdict(int)
    for t in throws.values():
        grid[(float(t["release_height_m"]), float(t["distance_m"]))] += 1
    kinematic: dict = defaultdict(set)
    opened: dict = {layer: defaultdict(set) for layer in LAYERS}
    for r in rows:
        t = throws[r["throw_index"]]
        key = (float(t["release_height_m"]), float(t["distance_m"]))
        kinematic[key].add(r["throw_index"])
        for layer in LAYERS:
            if r[f"reason_{layer}"] == REASON_NONE:
                opened[layer][key].add(r["throw_index"])
    return [
        {
            "release_height_m": key[0],
            "distance_m": key[1],
            "grid_throws": grid[key],
            "kinematic_throws": len(kinematic.get(key, ())),
            **{f"open_throws_{layer}": len(opened[layer].get(key, ())) for layer in LAYERS},
        }
        for key in sorted(grid)
    ]


def open_candidate_stats(rows: Sequence[Mapping], layer: str) -> dict | None:
    """Where the OPEN candidates of one layer sit in time — the vision-horizon input (S3.6).

    ``t_c_s`` is the catch time after release, i.e. the flight time the vision
    horizon has to cover; ``speed_m_s`` the ball speed at that point. The per-throw
    catch window is [min t_c, max t_c] over that throw's open candidates, and its
    end is what the horizon requirement (plan §4.4 S3.6) is read from. Spreads are
    ``catchability_map.distribution`` (finite values only, same keys as the
    kinematic map's summary). ``None`` when the layer opened nothing.
    """
    opened = [r for r in rows if r[f"reason_{layer}"] == REASON_NONE]
    if not opened:
        return None
    windows: dict = defaultdict(list)
    for r in opened:
        windows[r["throw_index"]].append(float(r["t_c_s"]))
    return {
        "candidates": len(opened),
        "throws": len(windows),
        "t_c_s": distribution(r["t_c_s"] for r in opened),
        "speed_m_s": distribution(r["speed_m_s"] for r in opened),
        "window_start_s": distribution(min(w) for w in windows.values()),
        "window_end_s": distribution(max(w) for w in windows.values()),
    }


def gate_alone_counts(rows: Sequence[Mapping]) -> dict[str, int]:
    """How many candidates EACH gate stops on its own, regardless of the others."""
    return {
        "commit": sum(1 for r in rows if not r["commit_ok"]),
        "reach_box": sum(1 for r in rows if not r["reach_ok_box"]),
        "reach_torque": sum(1 for r in rows if not r["reach_ok_torque"]),
        "gamma": sum(1 for r in rows if not r["gamma_ok"]),
        "stop": sum(1 for r in rows if not r["stop_ok"]),
    }


def _write_csv(path: Path, rows: Sequence[Mapping]) -> None:
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _floats(text: str) -> list[float]:
    return [float(x) for x in text.replace(",", " ").split()]


def load_accel_box(path: Path, group: str, n: int, *, require_adopted: bool = False) -> np.ndarray:
    """``derived_accel_limits.<group>.qdd_max`` from a plan §9 limits file.

    ``require_adopted`` returns an empty array when the entry is not
    ``adopted: true`` — the controller's rule (it loads no box then).
    """
    doc = yaml.safe_load(path.read_text())
    try:
        entry = doc["derived_accel_limits"][group]
        box = np.asarray(entry["qdd_max"], dtype=float)
    except (KeyError, TypeError) as exc:
        raise SystemExit(f"{path} has no derived_accel_limits.{group}.qdd_max") from exc
    if require_adopted and not entry.get("adopted", False):
        return np.empty(0)
    if box.shape != (n,):
        raise SystemExit(f"{path}: qdd_max has {box.size} entries, the arm has {n} joints")
    return box


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--robot-config", type=Path, nargs="+", required=True)
    ap.add_argument("--group", required=True, help="device group of the arm (devices.<group>)")
    ap.add_argument("--map-dir", type=Path, required=True, help="a catchability_map output dir")
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--catch-frame", default=DEFAULT_CATCH_FRAME)
    ap.add_argument("--urdf", type=Path, help="URDF/xacro override (default: urdf.package/path)")
    ap.add_argument(
        "--judge", type=Path, help=f"path to {GATE_EXECUTABLE} (default: ament prefix)"
    )
    ap.add_argument(
        "--velocity-source",
        choices=["config", "model"],
        required=True,
        help="joint speed limits: the robot config's max_velocity or the URDF rating",
    )
    ap.add_argument("--accel-limits", type=Path, required=True, help="plan §9 limits YAML")
    ap.add_argument("--eta-v", type=float, required=True, help="planner.gamma.eta_v")
    ap.add_argument("--eta-tau", type=float, required=True, help="torque fraction (plan §9)")
    ap.add_argument("--rotor-inertia", type=_floats, required=True, help="[kg m²], arm order")
    ap.add_argument("--rotor-inertia-source", required=True, help="file:line or datasheet")
    ap.add_argument(
        "--v-max-m-s",
        required=True,
        help="reference.v_max: a number, or 'derived' = the largest LP v_dir,max over the "
        "accepted candidates at the unscaled joint limits (S4.4)",
    )
    ap.add_argument("--d-eff-m", type=float, required=True, help="planner.hand.d_eff")
    ap.add_argument("--d-eff-source", required=True, help="where that d_eff comes from")
    ap.add_argument("--close-total-s", type=float, required=True, help="T_close,e2e + h/2")
    ap.add_argument("--gamma-margin-m-s", type=float, required=True, help="planner.gamma.margin")
    ap.add_argument("--a-dec-m-s2", type=float, required=True, help="supervisor.decel.a_dec")
    ap.add_argument("--a-dec-source", required=True, help="where that a_dec comes from")
    ap.add_argument("--detection-s", type=float, required=True)
    ap.add_argument("--latency-s", type=float, required=True)
    ap.add_argument("--arm-delay-s", type=float, required=True)
    ap.add_argument("--time-margin-s", type=float, required=True, help="planner.time.margin")
    ap.add_argument("--arm-base-frame", required=True, help="centre of the reach sphere")
    ap.add_argument("--max-reach-m", type=float, required=True)
    ap.add_argument("--floor-world-z-m", type=float, required=True)
    ap.add_argument(
        "--seed-id",
        type=int,
        help="which wait pose to gate. Required when the map was judged from several: the "
        "robot waits in ONE posture, and a union over seeds overstates every count",
    )
    ap.add_argument("--dls-damping", type=float, default=1e-3)
    ap.add_argument("--fk-tolerance-m", type=float, default=DEFAULT_FK_TOLERANCE_M)
    args = ap.parse_args(argv)

    from rtc_tools.analysis.catchability_map import (  # noqa: PLC0415
        find_judge,
        frame_placement_in_model_world,
    )

    params = load_robot_params(list(args.robot_config))
    spec = arm_spec_from_params(params, args.group)
    urdf_text, urdf_label = resolve_urdf_text(params, args.urdf)
    arm = ArmKinematics(
        urdf_text,
        spec.joint_names,
        extra_frame_from_params(params, args.catch_frame),
        args.rotor_inertia,
        args.catch_frame,
    )
    qd_max = np.asarray(
        spec.v_max if args.velocity_source == "config" else arm.model_velocity_limits(),
        dtype=float,
    )
    qd_plan = args.eta_v * qd_max
    tau_limit = args.eta_tau * np.asarray(spec.tau_max, dtype=float)
    box = load_accel_box(args.accel_limits, args.group, arm.n)
    reach_centre = frame_placement_in_model_world(urdf_text, args.arm_base_frame)[:3, 3]

    with (args.map_dir / "throw_summary.csv").open() as handle:
        throws = {int(r["throw_index"]): r for r in csv.DictReader(handle)}
    with (args.map_dir / "candidates.csv").open() as handle:
        accepted = [r for r in csv.DictReader(handle) if r["accepted"] in _TRUE]
    seed_ids = sorted({int(r["seed_id"]) for r in accepted})
    if args.seed_id is None and len(seed_ids) > 1:
        raise SystemExit(f"the map holds seeds {seed_ids}: pass --seed-id (one wait pose)")
    seed_id = seed_ids[0] if args.seed_id is None and seed_ids else args.seed_id
    accepted = [r for r in accepted if int(r["seed_id"]) == seed_id]
    if not accepted:
        raise SystemExit("the map has no accepted candidate for that wait pose — nothing to gate")
    seeds_csv = args.map_dir / "seeds.csv"
    seeds = {}
    for line in seeds_csv.read_text().splitlines():
        cells = [c.strip() for c in line.split(",")]
        if cells and cells[0].lstrip("-").isdigit():
            seeds[int(cells[0])] = np.array([float(c) for c in cells[1:]])

    inputs = [gate_inputs(arm, row, qd_plan, args.dls_damping) for row in accepted]
    worst_fk = max(item.fk_residual_m for item in inputs)
    if worst_fk > args.fk_tolerance_m:
        raise SystemExit(
            f"FK(q*) misses p_model by {worst_fk * 1e3:.2f} mm (> {args.fk_tolerance_m * 1e3:.2f}): "
            "this tool and the judge disagree about joint order, the frame, or the model"
        )
    if args.v_max_m_s == "derived":
        v_max = max(item.v_dir_max_lp for item in inputs) / args.eta_v
    else:
        v_max = float(args.v_max_m_s)

    settings = JudgeSettings(
        qdot_max=qd_max,
        qddot_max=box,
        eta_v=args.eta_v,
        v_max=v_max,
        d_eff=args.d_eff_m,
        t_close_total=args.close_total_s,
        gamma_margin=args.gamma_margin_m_s,
        a_dec=args.a_dec_m_s2,
        first_plan_s=args.detection_s + args.latency_s,
        t_arm_s=args.arm_delay_s,
        t_margin_s=args.time_margin_s,
    )
    commit_lead = args.close_total_s + args.arm_delay_s + args.time_margin_s  # L3 §4.11
    args.out_dir.mkdir(parents=True, exist_ok=True)
    gate_in = args.out_dir / "gate_candidates.csv"
    gate_in.write_text(gate_candidate_csv(inputs))
    judged = run_gate_judge(
        find_judge(args.judge, GATE_EXECUTABLE),
        gate_in,
        seeds_csv,
        args.out_dir / "gate_judged.csv",
        settings,
    )
    if [j["id"] for j in judged] != [item.row["id"] for item in inputs]:
        raise SystemExit("the judge's rows do not line up with the candidates it was given")

    rows = []
    for item, verdict in zip(inputs, judged, strict=True):
        row = item.row
        q_wait = seeds[int(row["seed_id"])]
        t_torque = torque_reach_time(arm, q_wait, item.q_star, qd_plan, tau_limit)
        p_stop = np.array([float(verdict[f"stop_gmin_{a}"]) for a in "xyz"])
        stop_ok = verdict["stop_gmin_valid"] in _TRUE and stop_inside_workspace(
            p_stop,
            float(row["p_model_z"]),
            float(row["p_world_z"]),
            reach_centre,
            args.max_reach_m,
            args.floor_world_z_m,
        )
        budget = float(verdict["lead_s"]) - args.time_margin_s
        commit_slack = float(row["t_c_s"]) - settings.first_plan_s - commit_lead
        reasons = layer_reasons(verdict, stop_ok, t_torque, budget, commit_slack >= 0.0)
        rows.append(
            {
                "id": row["id"],
                "seed_id": int(row["seed_id"]),
                "throw_index": int(row["throw_index"]),
                "t_c_s": float(row["t_c_s"]),
                "speed_m_s": float(row["speed_m_s"]),
                "drop_m": float(row["p_world_z"])
                - float(throws[int(row["throw_index"])]["release_height_m"]),
                "commit_slack_s": commit_slack,
                "commit_ok": commit_slack >= 0.0,
                "reach_budget_s": budget,
                "t_min_box_s": float(verdict["t_min_s"]),
                "t_min_torque_s": t_torque,
                "reach_ok_box": verdict["reach_ok"] in _TRUE,
                "reach_ok_torque": math.isfinite(t_torque) and t_torque <= budget,
                "v_dir_max_dls": float(verdict["v_dir_max"]),
                "v_dir_max_lp": item.v_dir_max_lp,
                "g_min": float(verdict["g_min"]),
                "g_max": float(verdict["g_max"]),
                "max_catchable_m_s": float(verdict["max_catchable_m_s"]),
                "gamma_ok": verdict["gamma_ok"] in _TRUE,
                "stop_distance_m": float(verdict["stop_gmin_distance"]),
                "stop_ok": stop_ok,
                "reason_box": reasons["box"],
                "reason_torque": reasons["torque"],
                **{f"q{i}": float(x) for i, x in enumerate(item.q_star)},
            }
        )

    cells = cell_table(rows, throws)
    _write_csv(args.out_dir / "gate_map.csv", rows)
    _write_csv(args.out_dir / "gate_cell_table.csv", cells)
    # The wait pose enters the reach gate only, so the postures that pass every
    # OTHER gate are what a better wait pose should sit between.
    reachable_targets = [
        np.array([r[f"q{i}"] for i in range(arm.n)])
        for r in rows
        if r["commit_ok"] and r["gamma_ok"] and r["stop_ok"]
    ]
    proposal = propose_wait_pose(reachable_targets)
    summary = {
        "tool": "rtc_tools.analysis.catch_gate_map",
        # Absolute: catching_trials --gate-map reads the grid back through
        # this path, from whatever directory it runs in.
        "map_dir": str(Path(args.map_dir).resolve()),
        "urdf": urdf_label,
        "arm_joints": spec.joint_names,
        "velocity_source": args.velocity_source,
        "qd_max": [float(x) for x in qd_max],
        "qdd_box": [float(x) for x in box],
        "accel_limits": str(args.accel_limits),
        "eta_v": args.eta_v,
        "eta_tau": args.eta_tau,
        "rotor_inertia": [float(x) for x in arm.rotor_inertia],
        "rotor_inertia_source": args.rotor_inertia_source,
        "v_max_m_s": v_max,
        "v_max_source": "derived" if args.v_max_m_s == "derived" else "argument",
        "d_eff_m": args.d_eff_m,
        "d_eff_source": args.d_eff_source,
        "close_total_s": args.close_total_s,
        "gamma_margin_m_s": args.gamma_margin_m_s,
        "a_dec_m_s2": args.a_dec_m_s2,
        "a_dec_source": args.a_dec_source,
        "first_plan_s": settings.first_plan_s,
        "arm_delay_s": args.arm_delay_s,
        "time_margin_s": args.time_margin_s,
        "commit_lead_s": commit_lead,
        "min_flight_time_s": settings.first_plan_s + commit_lead,
        "seed_id": seed_id,
        "wait_pose": [float(x) for x in seeds[seed_id]],
        "grid_throws": len(throws),
        "kinematic_throws": len({r["throw_index"] for r in rows}),
        "kinematic_candidates": len(rows),
        "fk_residual_max_m": worst_fk,
        "open_throws": {
            layer: len({r["throw_index"] for r in rows if r[f"reason_{layer}"] == REASON_NONE})
            for layer in LAYERS
        },
        "candidate_reasons": {
            layer: dict(Counter(r[f"reason_{layer}"] for r in rows).most_common())
            for layer in LAYERS
        },
        "candidates_stopped_by_each_gate_alone": gate_alone_counts(rows),
        "open_candidates": {layer: open_candidate_stats(rows, layer) for layer in LAYERS},
        "open_but_for_reach_candidates": len(reachable_targets),
        "proposed_wait_pose": None if proposal is None else [float(x) for x in proposal],
        "note": "rollout (L3 §4.8) is not judged: an open cell is PASS(provisional), an empty "
        "map is conclusive. The torque layer is a sufficient condition without a runtime "
        "counterpart (plan §9, D-16 revision).",
    }
    (args.out_dir / "gate_map_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False))
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
