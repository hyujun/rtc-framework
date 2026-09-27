"""Wait-pose search: which posture gives the most catching speed toward its OWN axis.

dynamic_catching S8-H/S8-I follow-up (#537-adjacent). ``catch_speed_budget``
answers "how fast can the arm move with the ball at an ACCEPTED catch pose";
this tool asks a narrower, offline, no-runtime question about the ONE pose the
robot waits in before a throw is even seen — the IK seed and, until the arm
homes there every cycle, the posture the planner starts every search from
(``planner.wait_pose``). A different wait pose changes the joint-space lever
the planner has to work with, so this searches nearby postures for one with a
larger directional-speed ceiling.

The objective is **self-consistent**: for a candidate pose ``q`` the aim
direction is

    v_hat(q) = -axis(q)      (axis = the catch frame's own +z, model world)

not a fixed pooled ball direction. This matters because the sim throw
generator that produces hand-near throws aims each ball along the MIRRORED
wait pose's own palm normal (``catchability_map.aim_at_hand``) — a candidate
that tilts its approach axis sees a throw that tilts WITH it, so scoring a
fixed external direction would overstate the lever a tilted pose actually
gets. The default objective (``--objective dls``) is ``directional_speed_dls``
— what the runtime's ``DirectionalSpeedMax`` feeds the γ window, so the pose
that maximises it maximises the planner's own γ_max; ``--objective lp`` ranks
by ``directional_speed_lp``, the physical ceiling. Both are reported for every
row, the DLS never above the LP.

Every limit is read from the robot profile, never from code (ARCH-1): joints
from ``devices.<arm>.joint_state_names``, the joint speed box from
``devices.<arm>.joint_limits.max_velocity`` composed the same way
``catching_arm_budget._device_limits`` composes it (``_base.yaml`` with
``sim.yaml``'s overlay of the rating when it has one — the launch composes
them in that order), ``planner.gamma.eta_v`` and ``planner.wait_pose`` from
the catching controller YAML. Kinematics are ``catch_speed_budget.
ArmKinematics`` built from the profile's URDF and extra ``catch_frame``; the
rotor inertia argument it takes is irrelevant here (the LP/DLS speed solves
never touch mass) and is always zero.

Method: uniform samples over the joint-limit box plus a log-scaled random walk
from the reference pose, filtered by the CLI's position/orientation/height
constraints, ranked by the self-consistent LP, then the top ``--refine-top``
locally refined with Nelder-Mead under a constraint-violation penalty (same
recipe as the private S8-H pre-analysis this tool productises).

Caveats (also written to the summary): the LP holds the approach-axis
ROTATION RATE at zero — it does not evaluate whether holding that orientation
while translating is itself achievable, or wanted. There is no self-collision
or environment-collision check (``ArmKinematics`` is a kinematic model only,
no geometry) and no IK-acceptance / gate-map check — this is a kinematic
speed-ceiling search only; the sim smoke run is what tells you a candidate
pose is actually reachable and safe.

Self-check (fail-closed). Every reported row — the reference pose, every raw
sample kept, every refined pose — must recompute to the same FK position it
is reported with (1e-9 m), sit inside the URDF joint limits, and never report
a DLS speed above its own LP speed (beyond the LP solver's tolerance,
relative 1e-3 + 1e-6 m/s — the damped minimum-norm solution meets the axis
constraint only to O(λ), so the two can cross where they coincide). Any violation refuses to write
output rather than report a number this tool and its own model disagree about.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import math
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis import catching_arm_budget as cab, catching_trials as ct
from rtc_tools.analysis.catch_speed_budget import (
    DEFAULT_CATCH_FRAME,
    ArmKinematics,
    directional_speed_dls,
    directional_speed_lp,
)
from rtc_tools.analysis.derive_accel_limits import resolve_urdf_text

TOOL = "catching_wait_pose_search"
DEFAULT_SAMPLES = 20000
DEFAULT_SEED = 0
DEFAULT_WALK_FRAC = 0.6
DEFAULT_REFINE_TOP = 10
NM_MAXITER = 1500
# Penalty weights for the Nelder-Mead refine step (same order of magnitude as
# the private S8-H pre-analysis this tool productises: position/limit/height
# violations dominate the objective, the axis tolerance is a softer nudge
# since the sampler and the outer filter already enforce it before refining).
PEN_POS = 50.0
PEN_ANG = 1.0
PEN_CLIP = 50.0
PEN_Z = 50.0
FK_SANITY_TOL_M = 1e-9
LIMIT_TOL_RAD = 1e-9
# "DLS never above LP" is not exact: the damped (λ = 1e-3) minimum-norm solution
# meets the approach-axis constraint only to O(λ), so where the DLS objective
# drives a pose to the LP optimum the DLS can sit a little ABOVE the constrained
# LP (1.7e-4 relative seen on a 6-axis profile). The check keeps a 1e-3 relative margin
# plus 1e-6 m/s; anything larger is a real inconsistency.
DLS_LP_REL_TOL = 1e-3
DLS_LP_ABS_TOL = 1e-6


# ── Configuration: joints, velocity box, reference pose (all from the profile) ──


def _joint_names(params: Mapping, device: str) -> list[str]:
    try:
        return [str(n) for n in params["devices"][device]["joint_state_names"]]
    except KeyError as exc:
        raise SystemExit(f"robot config lacks devices.{device}.{exc.args[0]}") from exc


def _catching_node(config_dir: Path, controller: str) -> dict:
    return ct._catching_controllers(config_dir)[controller]


def load_search_setup(
    config_dir: Path,
    *,
    controller: str | None = None,
    catch_frame: str = DEFAULT_CATCH_FRAME,
    urdf_override: Path | None = None,
) -> dict:
    """Everything the search needs from the profile, with provenance per key.

    Returns a dict: ``arm`` (:class:`ArmKinematics`), ``joints``, ``qd_box``
    (rad/s, before η_v), ``qd_source`` (per-key file name), ``eta_v``,
    ``eta_v_source``, ``wait_pose`` (``None`` if the controller has none),
    ``controller``, ``urdf_label``, ``config_dir``.
    """
    config_dir = Path(config_dir)
    profile = ct.load_profile(config_dir, controller, catch_frame=catch_frame)
    node = _catching_node(config_dir, profile.controller)
    joints = _joint_names(profile.robot_params, profile.arm_device)
    n = len(joints)

    limits, source = cab._device_limits(config_dir, profile.arm_device)
    qd_box = [float(v) for v in (limits.get("max_velocity") or [math.nan] * n)]
    if len(qd_box) != n:
        raise SystemExit(
            f"devices.{profile.arm_device}.joint_limits.max_velocity must have {n} entries "
            f"(joint_state_names), got {len(qd_box)}"
        )

    catching = node.get("catching") or {}
    planner = catching.get("planner") or {}
    gamma = planner.get("gamma") or {}
    eta_v = gamma.get("eta_v")
    if eta_v is None:
        raise SystemExit(f"{profile.controller}: catching.planner.gamma.eta_v is not set")
    wait_pose = planner.get("wait_pose")

    urdf_text, urdf_label = resolve_urdf_text(profile.robot_params, urdf_override)
    arm = ArmKinematics(
        urdf_text, joints, profile.catch_frame, np.zeros(n), profile.catch_frame_name
    )
    return {
        "arm": arm,
        "joints": joints,
        "qd_box": qd_box,
        "qd_source": {"max_velocity": source.get("max_velocity")},
        "eta_v": float(eta_v),
        "eta_v_source": f"controllers/{profile.controller}.yaml",
        "wait_pose": None if wait_pose is None else [float(v) for v in wait_pose],
        "controller": profile.controller,
        "urdf_label": urdf_label,
        "config_dir": str(config_dir),
    }


# ── Pure numerics: self-consistent LP/DLS, sampling, refine ────────────────────


def eval_self_consistent(
    arm: ArmKinematics, q: np.ndarray, qd_plan: np.ndarray
) -> tuple[float, float, bool]:
    """``v_dir_lp``, ``v_dir_dls`` at ``q`` toward ``v_hat = -frame_axis(q)``; third = LP validity."""
    zeros = np.zeros(arm.n)
    axis = arm.frame_axis(q)
    v_hat = -axis
    terms = arm.terms(q, zeros)
    lp, _ = directional_speed_lp(terms["jp"], terms["jw"], v_hat, qd_plan)
    if lp.limits_invalid or lp.input_invalid or lp.undetermined:
        return math.nan, math.nan, False
    dls = directional_speed_dls(terms["jp"], terms["jw"], v_hat, qd_plan)
    dls_v = (
        dls.v_dir_max
        if not (dls.limits_invalid or dls.input_invalid or dls.undetermined)
        else math.nan
    )
    return lp.v_dir_max, dls_v, True


def elevation_below_horizontal_deg(v_hat: np.ndarray) -> float:
    """Positive = the inbound direction descends toward the pose (as the private analysis defines it)."""
    return -math.degrees(math.asin(float(np.clip(v_hat[2], -1.0, 1.0))))


def sample_batch(
    rng: np.random.Generator,
    lo: np.ndarray,
    hi: np.ndarray,
    q_ref: np.ndarray,
    n: int,
    walk_max_rad: float,
    walk_frac: float,
) -> np.ndarray:
    """Uniform samples over the joint box plus a log-scaled random walk from ``q_ref``."""
    n_walk = int(round(n * walk_frac))
    n_uniform = n - n_walk
    uni = rng.uniform(lo, hi, size=(n_uniform, len(lo)))
    dirs = rng.normal(size=(n_walk, len(lo)))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True) + 1e-12
    radii = np.exp(rng.uniform(math.log(1e-3), math.log(max(walk_max_rad, 1e-3)), size=n_walk))
    walk = np.clip(q_ref[None, :] + dirs * radii[:, None], lo, hi)
    return np.vstack([uni, walk])


def fk_batch(arm: ArmKinematics, qs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    p = np.empty((len(qs), 3))
    ax = np.empty((len(qs), 3))
    for i, q in enumerate(qs):
        p[i] = arm.frame_position(q)
        ax[i] = arm.frame_axis(q)
    return p, ax


def constraint_mask(
    p: np.ndarray,
    axis: np.ndarray,
    p_ref: np.ndarray,
    axis_ref: np.ndarray,
    *,
    radius_m: float | None,
    box: tuple[np.ndarray, np.ndarray] | None,
    axis_tol_deg: float | None,
    min_z: float | None,
) -> tuple[np.ndarray, np.ndarray]:
    """(mask, angle-to-reference-axis-deg) over a batch of candidates."""
    ang = np.degrees(np.arccos(np.clip(axis @ axis_ref, -1.0, 1.0)))
    mask = np.ones(len(p), dtype=bool)
    if axis_tol_deg is not None:
        mask &= ang <= axis_tol_deg
    if radius_m is not None:
        mask &= np.linalg.norm(p - p_ref[None, :], axis=1) <= radius_m
    if box is not None:
        lo_b, hi_b = box
        mask &= np.all((p >= lo_b[None, :]) & (p <= hi_b[None, :]), axis=1)
    if min_z is not None:
        mask &= p[:, 2] >= min_z
    return mask, ang


OBJECTIVES = ("dls", "lp")
OBJECTIVE_KEY = {"dls": "v_dir_dls", "lp": "v_dir_lp"}


def dls_artefact(v_dir_dls: float, v_dir_lp: float) -> bool:
    """True when the damped minimum-norm speed sits above the LP ceiling beyond the
    solver tolerance — the DLS solution broke the approach-axis constraint at O(λ) or
    worse (near-singular ``jw``), so its speed is not one the constrained arm can give.
    Such a candidate is dropped from the search rather than reported."""
    return (
        math.isfinite(v_dir_dls) and v_dir_dls > v_dir_lp * (1.0 + DLS_LP_REL_TOL) + DLS_LP_ABS_TOL
    )


def refine_pose(
    arm: ArmKinematics,
    q0: np.ndarray,
    qd_plan: np.ndarray,
    p_ref: np.ndarray,
    axis_ref: np.ndarray,
    *,
    radius_m: float | None,
    box: tuple[np.ndarray, np.ndarray] | None,
    axis_tol_deg: float | None,
    min_z: float | None,
    lo: np.ndarray,
    hi: np.ndarray,
    objective: str = "dls",
) -> np.ndarray:
    """Nelder-Mead on the self-consistent speed (``objective`` = the runtime's DLS, or the
    LP ceiling) under a constraint-violation penalty."""
    from scipy.optimize import minimize  # noqa: PLC0415

    def obj(qvec: np.ndarray) -> float:
        q = np.clip(qvec, lo, hi)
        p = arm.frame_position(q)
        axis = arm.frame_axis(q)
        v_lp, v_dls, valid = eval_self_consistent(arm, q, qd_plan)
        # The DLS objective is capped by the LP: a damped solution that breaks the
        # approach-axis constraint reads as a speed the constrained arm cannot
        # give (see dls_artefact), and the optimiser must not climb into it.
        chosen = min(v_dls, v_lp) if objective == "dls" else v_lp
        base = chosen if valid and math.isfinite(chosen) else 0.0
        pos_pen = 0.0
        if radius_m is not None:
            pos_pen += max(0.0, float(np.linalg.norm(p - p_ref)) - radius_m)
        if box is not None:
            lo_b, hi_b = box
            pos_pen += float(np.sum(np.maximum(0.0, lo_b - p)) + np.sum(np.maximum(0.0, p - hi_b)))
        ang = math.degrees(math.acos(float(np.clip(axis @ axis_ref, -1.0, 1.0))))
        ang_pen = max(0.0, ang - axis_tol_deg) if axis_tol_deg is not None else 0.0
        clip_pen = float(np.sum(np.maximum(0.0, lo - qvec)) + np.sum(np.maximum(0.0, qvec - hi)))
        z_pen = max(0.0, min_z - p[2]) if min_z is not None else 0.0
        return -base + PEN_POS * pos_pen + PEN_ANG * ang_pen + PEN_CLIP * clip_pen + PEN_Z * z_pen

    res = minimize(
        obj,
        q0,
        method="Nelder-Mead",
        options={"maxiter": NM_MAXITER, "xatol": 1e-6, "fatol": 1e-8},
    )
    return np.clip(res.x, lo, hi)


def describe_pose(
    arm: ArmKinematics,
    q: np.ndarray,
    qd_plan: np.ndarray,
    q_ref: np.ndarray,
    p_ref: np.ndarray,
    axis_ref: np.ndarray,
    lo: np.ndarray,
    hi: np.ndarray,
    box: tuple[np.ndarray, np.ndarray] | None,
) -> dict:
    p = arm.frame_position(q)
    axis = arm.frame_axis(q)
    v_dir_lp, v_dir_dls, valid = eval_self_consistent(arm, q, qd_plan)
    ang = math.degrees(math.acos(float(np.clip(axis @ axis_ref, -1.0, 1.0))))
    within_limits = bool(np.all(q >= lo - LIMIT_TOL_RAD) and np.all(q <= hi + LIMIT_TOL_RAD))
    in_box = None
    if box is not None:
        lo_b, hi_b = box
        in_box = bool(np.all(p >= lo_b) and np.all(p <= hi_b))
    return {
        "q": np.asarray(q, dtype=float),
        "p": p,
        "axis": axis,
        "v_dir_lp": v_dir_lp,
        "v_dir_dls": v_dir_dls,
        "valid": valid,
        "max_dq_rad": float(np.max(np.abs(np.asarray(q) - q_ref))),
        "dist_m": float(np.linalg.norm(p - p_ref)),
        "axis_deg": ang,
        "in_box": in_box,
        "within_limits": within_limits,
    }


def _feasible(
    row: dict,
    *,
    radius_m: float | None,
    axis_tol_deg: float | None,
    box: tuple[np.ndarray, np.ndarray] | None,
    min_z: float | None,
    tol: float = 1e-6,
) -> bool:
    """Hard check of the same constraints the penalty in :func:`refine_pose` only discourages.

    Nelder-Mead minimises a PENALISED objective, not a hard-constrained one —
    a large enough speed gain can outweigh the penalty and land just outside
    the budget. Every reported row must satisfy the constraints exactly, so
    :func:`search_wait_poses` calls this after refining and falls back to the
    (already constraint-satisfying) raw sample when it does not.
    """
    if not row["valid"] or not row["within_limits"]:
        return False
    if radius_m is not None and row["dist_m"] > radius_m + tol:
        return False
    if axis_tol_deg is not None and row["axis_deg"] > axis_tol_deg + tol:
        return False
    if box is not None and row["in_box"] is False:
        return False
    return not (min_z is not None and row["p"][2] < min_z - tol)


def search_wait_poses(
    arm: ArmKinematics,
    qd_plan: np.ndarray,
    q_ref: np.ndarray,
    *,
    radius_m: float | None,
    axis_tol_deg: float | None,
    box: tuple[np.ndarray, np.ndarray] | None,
    min_z: float | None,
    samples: int,
    seed: int,
    walk_frac: float,
    refine_top: int,
    objective: str = "dls",
) -> dict:
    """Sample, filter, rank, and refine. Returns raw/refined candidate dicts + counts.

    ``objective`` ranks and refines by ``v_dir_dls`` (default — what the runtime's
    ``DirectionalSpeedMax`` feeds the γ window, so the pose that maximises it
    maximises the planner's own γ_max) or by ``v_dir_lp`` (the physical ceiling).
    Both numbers are reported for every row either way.
    """
    if objective not in OBJECTIVES:
        raise SystemExit(f"--objective must be one of {OBJECTIVES}, got {objective!r}")
    key = OBJECTIVE_KEY[objective]
    lo = np.asarray(arm.model.lowerPositionLimit, dtype=float)[arm.iq]
    hi = np.asarray(arm.model.upperPositionLimit, dtype=float)[arm.iq]
    p_ref = arm.frame_position(q_ref)
    axis_ref = arm.frame_axis(q_ref)

    rng = np.random.default_rng(seed)
    diagonal = float(np.linalg.norm(hi - lo))
    # A tight Cartesian radius around the reference pose is a LOCAL search — most of the
    # full joint-limit diagonal is wasted reach that only dilutes the log-scaled walk's
    # density near q_ref (and gets filtered out by the radius anyway). A box or no position
    # constraint is a GLOBAL search, which keeps the full diagonal. The 1/10 fraction is a
    # generic (robot-agnostic) choice, not a per-robot tuned constant.
    walk_max_rad = diagonal / 10.0 if radius_m is not None else diagonal
    qs = sample_batch(rng, lo, hi, q_ref, samples, walk_max_rad, walk_frac)
    qs = np.vstack([qs, q_ref[None, :]])  # guaranteed seed: angle-to-reference 0, always kept
    p, axis = fk_batch(arm, qs)
    mask, ang = constraint_mask(
        p,
        axis,
        p_ref,
        axis_ref,
        radius_m=radius_m,
        box=box,
        axis_tol_deg=axis_tol_deg,
        min_z=min_z,
    )
    n_pass = int(mask.sum())
    idxs = np.nonzero(mask)[0]
    evaluated = []
    for i in idxs:
        v_lp, v_dls, valid = eval_self_consistent(arm, qs[i], qd_plan)
        if not valid:
            continue
        evaluated.append(
            {
                "q": qs[i],
                "p": p[i],
                "axis": axis[i],
                "v_dir_lp": v_lp,
                "v_dir_dls": v_dls,
                "ang": float(ang[i]),
            }
        )
    n_artefact = sum(dls_artefact(d["v_dir_dls"], d["v_dir_lp"]) for d in evaluated)
    evaluated = [
        d
        for d in evaluated
        if math.isfinite(d[key]) and not dls_artefact(d["v_dir_dls"], d["v_dir_lp"])
    ]
    evaluated.sort(key=lambda d: d[key], reverse=True)
    top = evaluated[:refine_top]
    raw_rows = [
        describe_pose(arm, c["q"], qd_plan, q_ref, p_ref, axis_ref, lo, hi, box) for c in top
    ]
    refined_rows = []
    for raw_row, cand in zip(raw_rows, top, strict=True):
        q_r = refine_pose(
            arm,
            cand["q"],
            qd_plan,
            p_ref,
            axis_ref,
            radius_m=radius_m,
            box=box,
            axis_tol_deg=axis_tol_deg,
            min_z=min_z,
            lo=lo,
            hi=hi,
            objective=objective,
        )
        refined_row = describe_pose(arm, q_r, qd_plan, q_ref, p_ref, axis_ref, lo, hi, box)
        feasible = _feasible(
            refined_row, radius_m=radius_m, axis_tol_deg=axis_tol_deg, box=box, min_z=min_z
        )
        if (
            not feasible
            or not math.isfinite(refined_row[key])
            or dls_artefact(refined_row["v_dir_dls"], refined_row["v_dir_lp"])
            or refined_row[key] < raw_row[key] - 1e-9
        ):
            refined_row = raw_row  # NM's penalty is soft: keep the already-feasible raw sample
        refined_rows.append(refined_row)
    return {
        "n_sampled": int(len(qs)),
        "n_pass_constraints": n_pass,
        "n_lp_valid": len(evaluated),
        "n_dls_artefact_dropped": int(n_artefact),
        "raw": raw_rows,
        "refined": refined_rows,
        "lo": lo,
        "hi": hi,
        "p_ref": p_ref,
        "axis_ref": axis_ref,
        "objective": objective,
        "objective_key": key,
    }


# ── Sanity / fail-closed ─────────────────────────────────────────────────────


def _check_row(
    arm: ArmKinematics, stage: str, rank: int, row: dict, lo: np.ndarray, hi: np.ndarray
) -> None:
    if not row["valid"]:
        return
    p_check = arm.frame_position(row["q"])
    if float(np.linalg.norm(p_check - row["p"])) > FK_SANITY_TOL_M:
        raise SystemExit(
            f"{stage} rank {rank}: FK(q) does not reproduce the reported position "
            f"(residual {float(np.linalg.norm(p_check - row['p'])):.3e} m) — refusing to report"
        )
    if not (np.all(row["q"] >= lo - LIMIT_TOL_RAD) and np.all(row["q"] <= hi + LIMIT_TOL_RAD)):
        raise SystemExit(f"{stage} rank {rank}: q is outside the URDF joint limits")
    if dls_artefact(row["v_dir_dls"], row["v_dir_lp"]):
        raise SystemExit(
            f"{stage} rank {rank}: v_dir_dls ({row['v_dir_dls']:.6f}) exceeds v_dir_lp "
            f"({row['v_dir_lp']:.6f}) — refusing to report"
        )


def sanity_check(
    arm: ArmKinematics, rows: Sequence[tuple[str, int, dict]], lo: np.ndarray, hi: np.ndarray
) -> None:
    for stage, rank, row in rows:
        _check_row(arm, stage, rank, row, lo, hi)


# ── CLI ──────────────────────────────────────────────────────────────────────


def _floats(text: str) -> list[float]:
    return [float(tok) for tok in text.replace(",", " ").split()]


def _box_arg(text: str) -> tuple[np.ndarray, np.ndarray]:
    vals = _floats(text)
    if len(vals) != 6:
        raise argparse.ArgumentTypeError("--box needs 6 values: xmin ymin zmin xmax ymax zmax")
    lo, hi = np.array(vals[:3], dtype=float), np.array(vals[3:], dtype=float)
    if np.any(hi < lo):
        raise argparse.ArgumentTypeError("--box: max must be >= min on every axis")
    return lo, hi


def overlay_snippet(joints: Sequence[str], q: np.ndarray) -> str:
    values = ", ".join(f"{float(v):.4f}" for v in q)
    return f"planner:\n  wait_pose: [{values}]  # {len(joints)} joints, {joints}\n"


def _row_dict(joints: Sequence[str], rank: int, stage: str, row: dict) -> dict:
    out = {"rank": rank, "stage": stage}
    for j, v in zip(joints, row["q"], strict=True):
        out[f"q_{j}"] = v
    out["p_x"], out["p_y"], out["p_z"] = row["p"]
    out["axis_x"], out["axis_y"], out["axis_z"] = row["axis"]
    out["v_dir_lp"] = row["v_dir_lp"]
    out["v_dir_dls"] = row["v_dir_dls"]
    out["max_dq_rad"] = row["max_dq_rad"]
    out["dist_m"] = row["dist_m"]
    out["axis_deg"] = row["axis_deg"]
    out["in_box"] = "" if row["in_box"] is None else row["in_box"]
    out["within_limits"] = row["within_limits"]
    return out


def write_candidates_csv(
    path: Path, joints: Sequence[str], rows: Sequence[tuple[str, int, dict]]
) -> None:
    fieldnames = (
        ["rank", "stage"]
        + [f"q_{j}" for j in joints]
        + [
            "p_x",
            "p_y",
            "p_z",
            "axis_x",
            "axis_y",
            "axis_z",
            "v_dir_lp",
            "v_dir_dls",
            "max_dq_rad",
            "dist_m",
            "axis_deg",
            "in_box",
            "within_limits",
        ]
    )
    with Path(path).open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames)
        w.writeheader()
        for stage, rank, row in rows:
            w.writerow(_row_dict(joints, rank, stage, row))


def report(setup: dict, ref_row: dict, result: dict, runtime_s: float) -> str:
    lines = [f"{TOOL}: controller={setup['controller']} config_dir={setup['config_dir']}"]
    lines.append(
        f"  reference pose: v_dir_lp={ref_row['v_dir_lp']:.4f} m/s v_dir_dls={ref_row['v_dir_dls']:.4f} m/s "
        f"elevation={elevation_below_horizontal_deg(-ref_row['axis']):.2f} deg"
    )
    lines.append(
        f"  sampled {result['n_sampled']}, {result['n_pass_constraints']} pass the constraints, "
        f"{result['n_lp_valid']} LP-valid"
    )
    if result["refined"]:
        best = max(result["refined"], key=lambda d: d[result["objective_key"]])
        lines.append(
            f"  best refined: v_dir_lp={best['v_dir_lp']:.4f} m/s v_dir_dls={best['v_dir_dls']:.4f} m/s "
            f"(reference {ref_row['v_dir_lp']:.4f}/{ref_row['v_dir_dls']:.4f}) "
            f"dist={best['dist_m']:.4f} m axis_deg={best['axis_deg']:.2f} within_limits={best['within_limits']}"
        )
    else:
        lines.append("  no candidate passed the constraints — nothing to refine")
    lines.append(f"  runtime {runtime_s:.1f} s")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog=TOOL, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--config-dir", type=Path, required=True, help="robot profile directory")
    ap.add_argument("--controller", help="catching controller name when the profile has several")
    ap.add_argument("--catch-frame", default=DEFAULT_CATCH_FRAME)
    ap.add_argument("--urdf", type=Path, help="URDF/xacro override (default: urdf.package/path)")
    ap.add_argument(
        "--reference-pose",
        type=_floats,
        help="override the profile's planner.wait_pose, arm joint order",
    )
    ap.add_argument(
        "--evaluate-pose",
        type=_floats,
        help="evaluate this one pose (self-consistent v_dir_lp/dls, elevation, FK) and exit — "
        "no search, no --out needed",
    )
    ap.add_argument("--radius-m", type=float, help="‖p - p_ref‖ <= r around the reference pose")
    ap.add_argument(
        "--axis-tol-deg",
        type=float,
        help="max angle between the candidate approach axis and the reference axis "
        "(required unless --evaluate-pose)",
    )
    ap.add_argument("--box", type=_box_arg, help="position box 'xmin ymin zmin xmax ymax zmax'")
    ap.add_argument("--min-z", type=float, help="catch-frame position z floor")
    ap.add_argument(
        "--objective",
        choices=OBJECTIVES,
        default="dls",
        help="rank/refine by the runtime's DLS speed (default; the planner's γ_max uses it) or "
        "by the LP ceiling — both are reported per row",
    )
    ap.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED)
    ap.add_argument("--walk-frac", type=float, default=DEFAULT_WALK_FRAC)
    ap.add_argument("--refine-top", type=int, default=DEFAULT_REFINE_TOP)
    ap.add_argument("--out", type=Path, help="output directory (required unless --evaluate-pose)")
    args = ap.parse_args(argv)

    setup = load_search_setup(
        args.config_dir,
        controller=args.controller,
        catch_frame=args.catch_frame,
        urdf_override=args.urdf,
    )
    arm, joints = setup["arm"], setup["joints"]
    n = len(joints)
    qd_plan = setup["eta_v"] * np.asarray(setup["qd_box"], dtype=float)

    if args.evaluate_pose is not None:
        if len(args.evaluate_pose) != n:
            raise SystemExit(f"--evaluate-pose needs {n} values (arm joint order {joints})")
        q = np.array(args.evaluate_pose, dtype=float)
        p = arm.frame_position(q)
        axis = arm.frame_axis(q)
        v_dir_lp, v_dir_dls, valid = eval_self_consistent(arm, q, qd_plan)
        elev = elevation_below_horizontal_deg(-axis)
        print(f"{TOOL} --evaluate-pose: q={q.tolist()}")
        print(f"  p = {p.tolist()}")
        print(f"  axis (approach) = {axis.tolist()}")
        print(f"  v_hat = -axis = {(-axis).tolist()}")
        print(
            f"  v_dir_lp = {v_dir_lp:.4f} m/s  v_dir_dls = {v_dir_dls:.4f} m/s  (lp_valid={valid})"
        )
        print(f"  elevation below horizontal = {elev:.2f} deg")
        return 0

    if args.axis_tol_deg is None:
        ap.error("--axis-tol-deg is required unless --evaluate-pose is given")
    if args.out is None:
        ap.error("--out is required unless --evaluate-pose is given")

    if args.reference_pose is not None:
        if len(args.reference_pose) != n:
            raise SystemExit(f"--reference-pose needs {n} values (arm joint order {joints})")
        q_ref = np.array(args.reference_pose, dtype=float)
        ref_source = "--reference-pose"
    else:
        if setup["wait_pose"] is None:
            raise SystemExit(
                f"{setup['controller']}: catching.planner.wait_pose is not set — pass --reference-pose"
            )
        if len(setup["wait_pose"]) != n:
            raise SystemExit(
                f"{setup['controller']}: planner.wait_pose has {len(setup['wait_pose'])} entries, "
                f"the arm has {n}"
            )
        q_ref = np.array(setup["wait_pose"], dtype=float)
        ref_source = f"controllers/{setup['controller']}.yaml: planner.wait_pose"

    ref_lp, ref_dls, ref_valid = eval_self_consistent(arm, q_ref, qd_plan)
    if not ref_valid:
        raise SystemExit(
            "the reference pose gives an invalid/undetermined directional speed — check "
            "devices.<arm>.joint_limits.max_velocity (a NaN or non-positive entry fails closed)"
        )
    p_ref, axis_ref = arm.frame_position(q_ref), arm.frame_axis(q_ref)
    ref_row = {
        "q": q_ref,
        "p": p_ref,
        "axis": axis_ref,
        "v_dir_lp": ref_lp,
        "v_dir_dls": ref_dls,
        "valid": ref_valid,
        "max_dq_rad": 0.0,
        "dist_m": 0.0,
        "axis_deg": 0.0,
        "in_box": None
        if args.box is None
        else bool(np.all(p_ref >= args.box[0]) and np.all(p_ref <= args.box[1])),
        "within_limits": True,
    }

    t_start = time.time()
    result = search_wait_poses(
        arm,
        qd_plan,
        q_ref,
        radius_m=args.radius_m,
        axis_tol_deg=args.axis_tol_deg,
        box=args.box,
        min_z=args.min_z,
        samples=args.samples,
        seed=args.seed,
        walk_frac=args.walk_frac,
        refine_top=args.refine_top,
        objective=args.objective,
    )
    runtime_s = time.time() - t_start

    all_rows = [("reference", 0, ref_row)]
    all_rows += [("raw", i + 1, r) for i, r in enumerate(result["raw"])]
    all_rows += [("refined", i + 1, r) for i, r in enumerate(result["refined"])]
    sanity_check(arm, all_rows, result["lo"], result["hi"])

    args.out.mkdir(parents=True, exist_ok=True)
    write_candidates_csv(args.out / "wait_pose_candidates.csv", joints, all_rows)

    best = (
        max(result["refined"], key=lambda d: d[result["objective_key"]])
        if result["refined"]
        else None
    )
    summary = {
        "objective": args.objective,
        "tool": TOOL,
        "config_dir": setup["config_dir"],
        "controller": setup["controller"],
        "urdf": setup["urdf_label"],
        "joints": list(joints),
        "eta_v": setup["eta_v"],
        "eta_v_source": setup["eta_v_source"],
        "qd_max_rad_s": [float(v) for v in setup["qd_box"]],
        "qd_max_source": setup["qd_source"],
        "reference_pose": {
            "source": ref_source,
            "q": [float(v) for v in q_ref],
            "p": [float(v) for v in p_ref],
            "axis": [float(v) for v in axis_ref],
            "v_dir_lp": ref_lp,
            "v_dir_dls": ref_dls,
            "elevation_below_horizontal_deg": elevation_below_horizontal_deg(-axis_ref),
        },
        "constraints": {
            "radius_m": args.radius_m,
            "axis_tol_deg": args.axis_tol_deg,
            "box": None if args.box is None else [args.box[0].tolist(), args.box[1].tolist()],
            "min_z": args.min_z,
        },
        "sampling": {
            "samples": args.samples,
            "seed": args.seed,
            "walk_frac": args.walk_frac,
            "refine_top": args.refine_top,
        },
        "n_sampled": result["n_sampled"],
        "n_pass_constraints": result["n_pass_constraints"],
        "n_lp_valid": result["n_lp_valid"],
        "n_dls_artefact_dropped": result["n_dls_artefact_dropped"],
        "runtime_s": runtime_s,
        "date": _dt.date.today().isoformat(),
        "note": "v_hat = -axis(q): the objective is the pose's OWN inbound direction, matching "
        "the sim's aim_at_hand model. No collision / IK-acceptance check.",
    }
    if best is not None:
        summary["best"] = {
            "q": [float(v) for v in best["q"]],
            "p": [float(v) for v in best["p"]],
            "axis": [float(v) for v in best["axis"]],
            "v_dir_lp": best["v_dir_lp"],
            "v_dir_dls": best["v_dir_dls"],
            "dist_m": best["dist_m"],
            "axis_deg": best["axis_deg"],
            "within_limits": best["within_limits"],
            "overlay_snippet": overlay_snippet(joints, best["q"]),
        }
    (args.out / "wait_pose_search_summary.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False, allow_unicode=True, width=100)
    )
    text = report(setup, ref_row, result, runtime_s)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
