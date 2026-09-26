#!/usr/bin/env python3
"""Catch speed budget: how fast a ball can each catch candidate actually take?

dynamic_catching S4.4 (go / no-go). ``catchability_map`` answers "can the arm
REACH this catch pose"; this tool answers the next question for the poses it
accepted — "can the arm move WITH the ball fast enough there that the hand
closes in time". The planner's γ window (L3 §4.5) opens when

    ‖v‖ + margin  ≤  v_arm  +  v_rel

where ``v_arm`` is the speed the arm can hold along the ball direction v̂ at the
catch posture q*, and ``v_rel`` is the relative speed the hand absorbs
(d_eff / T_close,tot by the formula, or a measured value). Per candidate:

* ``v_dir_max_lp``  — max s with  J_p q̇ = s v̂,  approach-axis rate = 0,
  |q̇_i| ≤ η_v q̇_max,i.  The physical ceiling (linear program).
* ``v_dir_max_dls`` — the L3 §4.5 minimum-norm (damped least squares) estimate,
  i.e. what the runtime ``DirectionalSpeedMax`` is fed. Never above the LP;
  the gap is the redundancy the minimum-norm solution leaves unused.
* torque-limited DIRECTIONAL acceleration along the speed ramp:
      max a   s.t.  J₅ q̈ + J̇₅ q̇ = [a v̂; 0 0],   |M q̈ + h(q, q̇)| ≤ η_τ τ_max
  for speeding up before the catch (+v̂) and braking after it (−v̂). ``M``
  carries the reflected rotor inertia, which a URDF does not: it is a required
  argument, never a silent zero.
* ``v_arm``: the fastest speed that can be reached inside the stroke and the
  time available and still be stopped inside the stroke,
      s_acc = ∫ v/a_acc dv ≤ s₋,  t_acc = ∫ dv/a_acc ≤ t_c − lead,
      s_dec = ∫ v/a_dec dv ≤ s₊.

Everything here is OPTIMISTIC on purpose (bang-bang ramp, posture frozen at
q*, reach sphere instead of the accepted region, no jerk limit). An EMPTY
window is therefore conclusive; an OPEN one is provisional until the full
gate chain (rollout, reach time, stopping distance) has run.

Frames. The judge's ``p_model`` / ``v_model`` columns are in the MODEL WORLD
(the URDF root), which is what ``LOCAL_WORLD_ALIGNED`` Jacobians use, so they
are consumed as they are. The height floor is a WORLD quantity and uses the
``*_world`` columns. Nothing robot-specific is baked in: joints, limits, the
extra frame, and the reach centre all come from the robot config and the URDF.

Self-check. FK of the catch frame at every accepted q* must land on the
candidate's ``p_model`` (the judge accepted it within its own tolerance). A
larger residual means this tool and the judge disagree about joint order, the
frame or the model — the run is refused rather than reported.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis.derive_accel_limits import (
    arm_spec_from_params,
    load_robot_params,
    resolve_urdf_text,
)

DEFAULT_CATCH_FRAME = "catch_frame"
DEFAULT_GAMMA_MARGIN_M_S = 0.1  # L3 §6 planner.gamma.margin
DEFAULT_SPEED_STEP_M_S = 0.25
DEFAULT_DLS_DAMPING = 1e-3
DEFAULT_FK_TOLERANCE_M = 2.5e-3  # the judge's eps_pos (2 mm) plus rounding room
_TRUE = ("1", "True", "true")


# ── Pure numerics ─────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class DirectionalSpeed:
    """Mirror of the C++ ``DirectionalSpeed``: 0 whenever a flag is set."""

    v_dir_max: float = 0.0
    limits_invalid: bool = False
    input_invalid: bool = False
    undetermined: bool = False


def _limits_ok(limits: np.ndarray) -> bool:
    return bool(np.all(np.isfinite(limits)) and np.all(limits > 0.0))


def _direction_ok(v_hat: np.ndarray) -> bool:
    return bool(np.all(np.isfinite(v_hat)) and abs(float(np.linalg.norm(v_hat)) - 1.0) <= 1e-6)


def directional_speed_lp(
    jp: np.ndarray, jw: np.ndarray, v_hat: np.ndarray, qd_max: np.ndarray
) -> tuple[DirectionalSpeed, np.ndarray]:
    """Largest speed along ``v_hat`` with the approach axis held, inside the joint speed box.

    ``jp`` is the 3×n translational Jacobian in the frame ``v_hat`` is given in,
    ``jw`` the 2×n approach-axis (LOCAL x, y) angular rows. Returns the speed and
    the joint velocity that achieves it.
    """
    from scipy.optimize import linprog  # noqa: PLC0415

    n = jp.shape[1]
    qd_max = np.asarray(qd_max, dtype=float)
    if qd_max.shape != (n,) or not _limits_ok(qd_max):
        return DirectionalSpeed(limits_invalid=True), np.zeros(n)
    if not _direction_ok(v_hat) or not (np.all(np.isfinite(jp)) and np.all(np.isfinite(jw))):
        return DirectionalSpeed(input_invalid=True), np.zeros(n)
    a_eq = np.zeros((5, n + 1))
    a_eq[:3, :n], a_eq[:3, n], a_eq[3:, :n] = jp, -v_hat, jw
    cost = np.zeros(n + 1)
    cost[n] = -1.0
    res = linprog(
        cost,
        A_eq=a_eq,
        b_eq=np.zeros(5),
        bounds=[(-b, b) for b in qd_max] + [(0.0, None)],
        method="highs",
    )
    if res.status != 0:
        return DirectionalSpeed(undetermined=True), np.zeros(n)
    return DirectionalSpeed(v_dir_max=float(res.x[n])), np.asarray(res.x[:n])


def dls_unit_velocity(
    jp: np.ndarray, jw: np.ndarray, v_hat: np.ndarray, damping: float = DEFAULT_DLS_DAMPING
) -> np.ndarray:
    """q̇ᵘ of L3 §4.5: damped least squares for unit speed along v̂, approach axis held."""
    j5 = np.vstack([jp, jw])
    rhs = np.concatenate([v_hat, [0.0, 0.0]])
    return j5.T @ np.linalg.solve(j5 @ j5.T + damping**2 * np.eye(5), rhs)


def directional_speed_dls(
    jp: np.ndarray,
    jw: np.ndarray,
    v_hat: np.ndarray,
    qd_max: np.ndarray,
    damping: float = DEFAULT_DLS_DAMPING,
) -> DirectionalSpeed:
    """L3 §4.5: max(0, v̂ᵀ J_p q̇ᵘ) / max_i(|q̇ᵘ_i| / q̇_max,i) for the DLS unit-speed solution."""
    n = jp.shape[1]
    qd_max = np.asarray(qd_max, dtype=float)
    if qd_max.shape != (n,) or not _limits_ok(qd_max):
        return DirectionalSpeed(limits_invalid=True)
    if not _direction_ok(v_hat) or not (np.all(np.isfinite(jp)) and np.all(np.isfinite(jw))):
        return DirectionalSpeed(input_invalid=True)
    qd_unit = dls_unit_velocity(jp, jw, v_hat, damping)
    denom = float(np.max(np.abs(qd_unit) / qd_max))
    if not (denom > 0.0) or not math.isfinite(denom):
        return DirectionalSpeed(undetermined=True)
    return DirectionalSpeed(v_dir_max=max(0.0, float(v_hat @ (jp @ qd_unit))) / denom)


def directional_accel_lp(
    mass: np.ndarray,
    bias: np.ndarray,
    jp: np.ndarray,
    jw: np.ndarray,
    drift_linear: np.ndarray,
    drift_angular_xy: np.ndarray,
    direction: np.ndarray,
    tau_limit: np.ndarray,
) -> tuple[float, np.ndarray]:
    """Largest acceleration along ``direction`` inside |M q̈ + h| ≤ τ, approach axis held.

    Returns ``(nan, zeros)`` when no joint acceleration satisfies the torque
    limits at all (the bias alone already exceeds them). The result may be
    negative: the arm can hold the axis but only while decelerating.
    """
    from scipy.optimize import linprog  # noqa: PLC0415

    n = jp.shape[1]
    a_eq = np.zeros((5, n + 1))
    a_eq[:3, :n], a_eq[:3, n], a_eq[3:, :n] = jp, -direction, jw
    b_eq = np.concatenate([-np.asarray(drift_linear), -np.asarray(drift_angular_xy)])
    a_ub = np.zeros((2 * n, n + 1))
    a_ub[:n, :n], a_ub[n:, :n] = mass, -mass
    b_ub = np.concatenate([tau_limit - bias, tau_limit + bias])
    cost = np.zeros(n + 1)
    cost[n] = -1.0
    res = linprog(
        cost,
        A_ub=a_ub,
        b_ub=b_ub,
        A_eq=a_eq,
        b_eq=b_eq,
        bounds=[(None, None)] * (n + 1),
        method="highs",
    )
    if res.status != 0:
        return float("nan"), np.zeros(n)
    return float(res.x[n]), np.asarray(res.x[:n])


def stroke_to_boundary(
    point: np.ndarray,
    direction: np.ndarray,
    centre: np.ndarray,
    reach_m: float,
    height_above_floor_m: float,
) -> float:
    """Distance from ``point`` along ``direction`` to the reach sphere or the height floor.

    ``height_above_floor_m`` is the point's clearance over the floor (a WORLD
    quantity; ``direction[2]`` is the same in both frames because only a yaw and
    a translation separate them). 0 when the point is already outside.
    """
    rel = np.asarray(point, dtype=float) - np.asarray(centre, dtype=float)
    b = float(rel @ direction)
    disc = b * b - (float(rel @ rel) - reach_m * reach_m)
    if disc < 0.0 or height_above_floor_m < 0.0:
        return 0.0
    travel = -b + math.sqrt(disc)
    if direction[2] < 0.0:
        travel = min(travel, height_above_floor_m / -float(direction[2]))
    return max(0.0, travel)


def ramp_speed_limit(
    speeds: np.ndarray,
    accel: np.ndarray,
    decel: np.ndarray,
    stroke_before_m: float,
    stroke_after_m: float,
    time_available_s: float,
    speed_cap_m_s: float,
) -> float:
    """Fastest grid speed reachable inside stroke and time, and stoppable inside stroke.

    Trapezoid rule on s = ∫ v/a dv and t = ∫ dv/a. A non-positive (or NaN)
    acceleration ends the ramp. When the ramp passes ``speed_cap_m_s`` between
    two grid points the cap itself is returned.
    """
    best, s_acc, t_acc, s_dec = 0.0, 0.0, 0.0, 0.0
    for k in range(1, len(speeds)):
        if speeds[k] > speed_cap_m_s + 1e-9:
            return speed_cap_m_s if best >= speeds[k - 1] - 1e-9 else best
        a_up = 0.5 * (accel[k - 1] + accel[k])
        a_down = 0.5 * (decel[k - 1] + decel[k])
        if not (a_up > 1e-9 and a_down > 1e-9):
            break
        v_mid, dv = 0.5 * (speeds[k - 1] + speeds[k]), speeds[k] - speeds[k - 1]
        s_acc += v_mid / a_up * dv
        t_acc += dv / a_up
        s_dec += v_mid / a_down * dv
        if s_acc > stroke_before_m or t_acc > time_available_s or s_dec > stroke_after_m:
            break
        best = float(speeds[k])
    return best


def min_flight_time(
    detection_s: float, latency_s: float, close_total_s: float, arm_delay_s: float, margin_s: float
) -> float:
    """Commit lead (plan S0.7 R1): T_det + L + T_freeze, T_freeze = T_close,tot + T_arm + T_margin.

    The hand's closing time is part of the lead: the posture freezes that long
    before the catch, and a plan has to exist before it can freeze.
    """
    return detection_s + latency_s + close_total_s + arm_delay_s + margin_s


def window_open(speed_m_s: float, v_arm_m_s: float, v_rel_m_s: float, margin_m_s: float) -> bool:
    return bool(speed_m_s + margin_m_s <= v_arm_m_s + v_rel_m_s)


# ── Model access ──────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ExtraFrame:
    parent: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def extra_frame_from_params(params: Mapping, name: str) -> ExtraFrame:
    try:
        node = params["urdf"]["extra_frames"][name]
        return ExtraFrame(
            str(node["parent"]), tuple(map(float, node["xyz"])), tuple(map(float, node["rpy"]))
        )
    except KeyError as exc:
        raise SystemExit(f"robot config lacks urdf.extra_frames.{name}.{exc.args[0]}") from exc


class ArmKinematics:
    """Pinocchio model + the extra frame + the arm joints, addressed by NAME.

    Joint order is whatever ``joint_names`` says; every per-joint vector this
    class takes or returns is in that order. The extra frame is added the way
    the production model builder adds it: a fixed placement on its parent frame.
    """

    def __init__(
        self,
        urdf_text: str,
        joint_names: Sequence[str],
        frame: ExtraFrame,
        rotor_inertia: Sequence[float],
        frame_name: str = DEFAULT_CATCH_FRAME,
    ) -> None:
        import pinocchio as pin  # noqa: PLC0415

        self._pin = pin
        self.model = pin.buildModelFromXML(urdf_text)
        if not self.model.existFrame(frame.parent):
            raise SystemExit(f"the URDF has no frame '{frame.parent}' (extra frame parent)")
        parent_id = self.model.getFrameId(frame.parent)
        parent = self.model.frames[parent_id]
        offset = pin.SE3(pin.rpy.rpyToMatrix(*frame.rpy), np.asarray(frame.xyz, dtype=float))
        self.frame_id = self.model.addFrame(
            pin.Frame(
                frame_name,
                parent.parentJoint,
                parent_id,
                parent.placement * offset,
                pin.FrameType.OP_FRAME,
            )
        )
        self.data = self.model.createData()
        self.joint_names = list(joint_names)
        ids = []
        for name in self.joint_names:
            if not self.model.existJointName(name):
                raise SystemExit(f"the URDF has no joint '{name}'")
            ids.append(self.model.getJointId(name))
        if any(self.model.joints[j].nq != 1 or self.model.joints[j].nv != 1 for j in ids):
            raise SystemExit("arm joints must be single-DoF with nq == nv == 1")
        self.iq = [self.model.joints[j].idx_q for j in ids]
        self.iv = [self.model.joints[j].idx_v for j in ids]
        self.rotor_inertia = np.asarray(rotor_inertia, dtype=float)
        if self.rotor_inertia.shape != (len(ids),) or not np.all(self.rotor_inertia >= 0.0):
            raise SystemExit(f"--rotor-inertia needs {len(ids)} non-negative entries")

    @property
    def n(self) -> int:
        return len(self.iv)

    def model_velocity_limits(self) -> np.ndarray:
        return np.asarray(self.model.velocityLimit, dtype=float)[self.iv]

    def _full(self, q_arm: np.ndarray, qd_arm: np.ndarray | None = None):
        q = self._pin.neutral(self.model)
        v = np.zeros(self.model.nv)
        for i, value in zip(self.iq, q_arm, strict=True):
            q[i] = value
        if qd_arm is not None:
            for i, value in zip(self.iv, qd_arm, strict=True):
                v[i] = value
        return q, v

    def frame_placement(self, q_arm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(rotation 3×3, position) of the extra frame in the model world at ``q_arm``."""
        q, _ = self._full(np.asarray(q_arm, dtype=float))
        self._pin.framesForwardKinematics(self.model, self.data, q)
        placement = self.data.oMf[self.frame_id]
        return np.array(placement.rotation), np.array(placement.translation)

    def frame_position(self, q_arm: np.ndarray) -> np.ndarray:
        q, _ = self._full(q_arm)
        self._pin.framesForwardKinematics(self.model, self.data, q)
        return np.array(self.data.oMf[self.frame_id].translation)

    def frame_axis(self, q_arm: np.ndarray) -> np.ndarray:
        """The frame's +z (approach axis) in the model world."""
        q, _ = self._full(q_arm)
        self._pin.framesForwardKinematics(self.model, self.data, q)
        return np.array(self.data.oMf[self.frame_id].rotation[:, 2])

    def terms(self, q_arm: np.ndarray, qd_arm: np.ndarray) -> dict:
        """M (with rotor inertia), bias h, Jacobian rows and the J̇q̇ drift at (q, q̇)."""
        pin, m, d = self._pin, self.model, self.data
        q, v = self._full(q_arm, qd_arm)
        mass = pin.crba(m, d, q)
        mass = np.triu(mass) + np.triu(mass, 1).T
        bias = pin.nonLinearEffects(m, d, q, v).copy()
        pin.forwardKinematics(m, d, q, v, np.zeros(m.nv))
        pin.computeJointJacobians(m, d, q)
        pin.updateFramePlacements(m, d)
        j_world = pin.getFrameJacobian(m, d, self.frame_id, pin.LOCAL_WORLD_ALIGNED)[:, self.iv]
        j_local = pin.getFrameJacobian(m, d, self.frame_id, pin.LOCAL)[:, self.iv]
        drift_lin = pin.getFrameClassicalAcceleration(
            m, d, self.frame_id, pin.LOCAL_WORLD_ALIGNED
        ).linear
        drift_ang = pin.getFrameAcceleration(m, d, self.frame_id, pin.LOCAL).angular
        return {
            "mass": mass[np.ix_(self.iv, self.iv)] + np.diag(self.rotor_inertia),
            "bias": bias[self.iv],
            "jp": j_world[:3].copy(),
            "jw": j_local[3:5].copy(),
            "drift_linear": np.array(drift_lin),
            "drift_angular_xy": np.array(drift_ang)[:2],
        }

    def _acceleration(self, qdd_arm) -> np.ndarray:
        acc = np.zeros(self.model.nv)
        for i, value in zip(self.iv, qdd_arm, strict=True):
            acc[i] = value
        return acc

    def _torques(self, q, v, acc, qdd_arm) -> np.ndarray:
        tau = self._pin.rnea(self.model, self.data, q, v, acc)
        return np.asarray(tau)[self.iv] + self.rotor_inertia * np.asarray(qdd_arm)

    def joint_torques(self, q_arm, qd_arm, qdd_arm) -> np.ndarray:
        """Arm joint torques for (q, q̇, q̈): RNEA plus the rotor inertia the URDF lacks."""
        q, v = self._full(q_arm, qd_arm)
        return self._torques(q, v, self._acceleration(qdd_arm), qdd_arm)

    def inverse_dynamics(self, q_arm, qd_arm, qdd_arm) -> tuple[np.ndarray, np.ndarray]:
        """(arm joint torques incl. rotor inertia, frame linear acceleration) — the LP's oracle."""
        pin, m, d = self._pin, self.model, self.data
        q, v = self._full(q_arm, qd_arm)
        acc = self._acceleration(qdd_arm)
        tau = self._torques(q, v, acc, qdd_arm)
        pin.forwardKinematics(m, d, q, v, acc)
        lin = pin.getFrameClassicalAcceleration(
            m, d, self.frame_id, pin.LOCAL_WORLD_ALIGNED
        ).linear
        return np.asarray(tau), np.array(lin)


# ── Per-candidate budget ──────────────────────────────────────────────────────


@dataclass(frozen=True)
class BudgetSettings:
    qd_max: np.ndarray  # joint speed limits in arm order, BEFORE eta_v
    tau_max: np.ndarray
    eta_v: float
    eta_tau: float
    reach_centre: np.ndarray  # model world
    reach_m: float
    floor_world_z_m: float
    lead_s: float  # time before t_c that is not available for the ramp
    speed_step_m_s: float = DEFAULT_SPEED_STEP_M_S


def budget_candidate(arm: ArmKinematics, row: Mapping[str, str], cfg: BudgetSettings) -> dict:
    n = int(row["nv"])
    if n != arm.n:
        raise SystemExit(f"candidate nv = {n} but the arm has {arm.n} joints")
    q = np.array([float(row[f"q{i}"]) for i in range(n)])
    v = np.array([float(row[f"v_model_{a}"]) for a in "xyz"])
    p = np.array([float(row[f"p_model_{a}"]) for a in "xyz"])
    speed = float(np.linalg.norm(v))
    v_hat = v / speed
    rest = arm.terms(q, np.zeros(n))
    lp, qd_dir = directional_speed_lp(rest["jp"], rest["jw"], v_hat, cfg.eta_v * cfg.qd_max)
    dls = directional_speed_dls(rest["jp"], rest["jw"], v_hat, cfg.eta_v * cfg.qd_max)
    clearance = float(row["p_world_z"]) - cfg.floor_world_z_m
    s_before = stroke_to_boundary(p, -v_hat, cfg.reach_centre, cfg.reach_m, clearance)
    s_after = stroke_to_boundary(p, v_hat, cfg.reach_centre, cfg.reach_m, clearance)
    tau_limit = cfg.eta_tau * cfg.tau_max
    speeds = np.append(np.arange(0.0, lp.v_dir_max, cfg.speed_step_m_s), lp.v_dir_max)
    accel, decel = [], []
    for vk in speeds:
        qd = qd_dir * (vk / lp.v_dir_max) if lp.v_dir_max > 0.0 else np.zeros(n)
        t = arm.terms(q, qd)
        common = (t["mass"], t["bias"], t["jp"], t["jw"], t["drift_linear"], t["drift_angular_xy"])
        accel.append(directional_accel_lp(*common, v_hat, tau_limit)[0])
        decel.append(directional_accel_lp(*common, -v_hat, tau_limit)[0])
    accel, decel = np.array(accel), np.array(decel)
    t_c = float(row["t_c_s"])
    v_arm = ramp_speed_limit(
        speeds, accel, decel, s_before, s_after, max(0.0, t_c - cfg.lead_s), lp.v_dir_max
    )
    return {
        "id": row["id"],
        "throw_index": int(row["throw_index"]),
        "t_c_s": t_c,
        "speed_m_s": speed,
        "drop_m": float("nan"),  # filled by the caller, which knows the release height
        "v_dir_max_lp": lp.v_dir_max,
        "v_dir_max_dls": dls.v_dir_max,
        "speed_flags": int(lp.limits_invalid or lp.input_invalid or lp.undetermined),
        "a_dir_rest": float(accel[0]),
        "a_brake_rest": float(decel[0]),
        "a_dir_top": float(accel[-1]),
        "stroke_before_m": s_before,
        "stroke_after_m": s_after,
        "v_arm": v_arm,
        "fk_residual_m": float(np.linalg.norm(arm.frame_position(q) - p)),
        "axis_angle_rad": math.acos(max(-1.0, min(1.0, float(arm.frame_axis(q) @ -v_hat)))),
    }


# ── Aggregation (denominator = the full grid, always) ─────────────────────────


def drop_table(
    rows: Sequence[Mapping],
    drop_edges_m: Sequence[float],
    v_rel_values: Sequence[float],
    margin_m_s: float,
    min_t_c_s: float,
) -> list[dict]:
    """Rows = drop bins, columns = relative-speed allowance; cell = share of candidates that open."""
    out = []
    for lo, hi in zip(drop_edges_m[:-1], drop_edges_m[1:], strict=True):
        sel = [r for r in rows if lo <= r["drop_m"] < hi and r["t_c_s"] >= min_t_c_s]
        need = np.array([r["speed_m_s"] + margin_m_s - r["v_arm"] for r in sel])
        line = {"drop_lo_m": lo, "drop_hi_m": hi, "candidates": len(sel)}
        line["need_min_m_s"] = float(need.min()) if len(need) else float("nan")
        line["need_median_m_s"] = float(np.median(need)) if len(need) else float("nan")
        for v_rel in v_rel_values:
            line[f"open_share@{v_rel:g}"] = float(np.mean(need <= v_rel)) if len(need) else 0.0
        out.append(line)
    return out


def cell_table(
    rows: Sequence[Mapping],
    throws: Mapping[int, Mapping[str, str]],
    v_rel_values: Sequence[float],
    margin_m_s: float,
    min_t_c_s: float,
) -> list[dict]:
    """Rows = (release height, distance) cells of the throw grid; counts are THROWS.

    Every grid throw is in a denominator, including those with no candidate at
    all (``throws`` is the full ``throw_summary.csv``), so a cell's share cannot
    grow just because the pre-filters removed its hard throws.
    """
    grid: dict[tuple[float, float], int] = defaultdict(int)
    for t in throws.values():
        grid[(float(t["release_height_m"]), float(t["distance_m"]))] += 1
    kin: dict[tuple[float, float], set[int]] = defaultdict(set)
    opened = {v: defaultdict(set) for v in v_rel_values}
    for r in rows:
        t = throws[r["throw_index"]]
        cell = (float(t["release_height_m"]), float(t["distance_m"]))
        kin[cell].add(r["throw_index"])
        if r["t_c_s"] < min_t_c_s:
            continue
        for v_rel in v_rel_values:
            if window_open(r["speed_m_s"], r["v_arm"], v_rel, margin_m_s):
                opened[v_rel][cell].add(r["throw_index"])
    out = []
    for cell in sorted(grid):
        line = {
            "release_height_m": cell[0],
            "distance_m": cell[1],
            "grid_throws": grid[cell],
            "kinematic_throws": len(kin[cell]),
        }
        for v_rel in v_rel_values:
            line[f"open_throws@{v_rel:g}"] = len(opened[v_rel][cell])
        out.append(line)
    return out


def _write_csv(path: Path, rows: Sequence[Mapping]) -> None:
    with Path(path).open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else [])
        writer.writeheader()
        writer.writerows(rows)


# ── CLI ───────────────────────────────────────────────────────────────────────


def _floats(text: str) -> list[float]:
    return [float(tok) for tok in text.replace(",", " ").split()]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--robot-config", type=Path, nargs="+", required=True)
    ap.add_argument("--group", required=True, help="device group of the arm (devices.<group>)")
    ap.add_argument("--map-dir", type=Path, required=True, help="a catchability_map output dir")
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--catch-frame", default=DEFAULT_CATCH_FRAME)
    ap.add_argument("--urdf", type=Path, help="URDF/xacro override (default: urdf.package/path)")
    ap.add_argument(
        "--velocity-source",
        choices=["config", "model"],
        required=True,
        help="joint speed limits: the robot config's max_velocity (what execution enforces) or "
        "the URDF <limit velocity> (the rating). No default: it is a decision",
    )
    ap.add_argument("--eta-v", type=float, required=True, help="speed margin, applied to q̇_max")
    ap.add_argument("--eta-tau", type=float, required=True, help="torque fraction (plan §9)")
    ap.add_argument(
        "--rotor-inertia",
        type=_floats,
        required=True,
        help="reflected rotor inertia per arm joint [kg m²], arm joint order. The URDF has "
        "none, and leaving it out overstates every acceleration — pass zeros to say so",
    )
    ap.add_argument("--rotor-inertia-source", required=True, help="file:line or datasheet")
    ap.add_argument("--arm-base-frame", required=True, help="centre of the reach sphere")
    ap.add_argument("--max-reach-m", type=float, required=True)
    ap.add_argument("--floor-world-z-m", type=float, required=True)
    ap.add_argument("--detection-s", type=float, required=True)
    ap.add_argument("--latency-s", type=float, required=True)
    ap.add_argument("--close-total-s", type=float, required=True, help="T_close,e2e + T_tick")
    ap.add_argument("--arm-delay-s", type=float, required=True)
    ap.add_argument("--time-margin-s", type=float, required=True)
    ap.add_argument(
        "--relative-speed-m-s",
        type=_floats,
        required=True,
        help="relative speed(s) the hand absorbs: d_eff / T_close,tot or a measured value",
    )
    ap.add_argument("--gamma-margin-m-s", type=float, default=DEFAULT_GAMMA_MARGIN_M_S)
    ap.add_argument(
        "--drop-edges-m",
        type=_floats,
        default=[-2.0, -1.5, -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.7],
    )
    ap.add_argument("--speed-step-m-s", type=float, default=DEFAULT_SPEED_STEP_M_S)
    ap.add_argument("--fk-tolerance-m", type=float, default=DEFAULT_FK_TOLERANCE_M)
    args = ap.parse_args(argv)

    from rtc_tools.analysis.catchability_map import (  # noqa: PLC0415
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
    qd_max = spec.v_max if args.velocity_source == "config" else arm.model_velocity_limits()
    lead = min_flight_time(
        args.detection_s, args.latency_s, args.close_total_s, args.arm_delay_s, args.time_margin_s
    )
    cfg = BudgetSettings(
        qd_max=np.asarray(qd_max, dtype=float),
        tau_max=np.asarray(spec.tau_max, dtype=float),
        eta_v=args.eta_v,
        eta_tau=args.eta_tau,
        reach_centre=frame_placement_in_model_world(urdf_text, args.arm_base_frame)[:3, 3],
        reach_m=args.max_reach_m,
        floor_world_z_m=args.floor_world_z_m,
        # only detection + latency eat into the RAMP; the closing time runs in parallel with it
        lead_s=args.detection_s + args.latency_s,
        speed_step_m_s=args.speed_step_m_s,
    )
    with (args.map_dir / "throw_summary.csv").open() as handle:
        throws = {int(r["throw_index"]): r for r in csv.DictReader(handle)}
    with (args.map_dir / "candidates.csv").open() as handle:
        accepted = [r for r in csv.DictReader(handle) if r["accepted"] in _TRUE]
    if not accepted:
        raise SystemExit("the map has no accepted candidate — nothing to budget")
    rows = []
    for row in accepted:
        rec = budget_candidate(arm, row, cfg)
        rec["drop_m"] = float(row["p_world_z"]) - float(
            throws[rec["throw_index"]]["release_height_m"]
        )
        rows.append(rec)
    worst_fk = max(r["fk_residual_m"] for r in rows)
    if worst_fk > args.fk_tolerance_m:
        raise SystemExit(
            f"FK(q*) misses p_model by {worst_fk * 1e3:.2f} mm (> {args.fk_tolerance_m * 1e3:.2f}): "
            "this tool and the judge disagree about joint order, the frame, or the model"
        )
    if any(r["v_dir_max_dls"] > r["v_dir_max_lp"] * (1 + 1e-6) + 1e-9 for r in rows):
        raise SystemExit("minimum-norm speed exceeds the LP optimum — refusing to report")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(args.out_dir / "speed_budget.csv", rows)
    drops = drop_table(
        rows, args.drop_edges_m, args.relative_speed_m_s, args.gamma_margin_m_s, lead
    )
    cells = cell_table(rows, throws, args.relative_speed_m_s, args.gamma_margin_m_s, lead)
    _write_csv(args.out_dir / "drop_table.csv", drops)
    _write_csv(args.out_dir / "cell_table.csv", cells)
    summary = {
        "tool": "rtc_tools.analysis.catch_speed_budget",
        "map_dir": str(args.map_dir),
        "urdf": urdf_label,
        "arm_joints": spec.joint_names,
        "velocity_source": args.velocity_source,
        "qd_max": [float(x) for x in cfg.qd_max],
        "eta_v": args.eta_v,
        "eta_tau": args.eta_tau,
        "tau_max": [float(x) for x in cfg.tau_max],
        "rotor_inertia": [float(x) for x in arm.rotor_inertia],
        "rotor_inertia_source": args.rotor_inertia_source,
        "min_flight_time_s": lead,
        "ramp_lead_s": cfg.lead_s,
        "relative_speed_m_s": list(args.relative_speed_m_s),
        "gamma_margin_m_s": args.gamma_margin_m_s,
        "grid_throws": len(throws),
        "accepted_candidates": len(rows),
        "fk_residual_max_m": worst_fk,
        "axis_angle_max_rad": max(r["axis_angle_rad"] for r in rows),
        "v_dir_max_lp_median_m_s": float(np.median([r["v_dir_max_lp"] for r in rows])),
        "dls_over_lp_median": float(
            np.median(
                [r["v_dir_max_dls"] / r["v_dir_max_lp"] for r in rows if r["v_dir_max_lp"] > 0]
            )
        ),
        "a_dir_rest_median_m_s2": float(np.nanmedian([r["a_dir_rest"] for r in rows])),
        "open_throws": {
            f"{v:g}": int(sum(c[f"open_throws@{v:g}"] for c in cells))
            for v in args.relative_speed_m_s
        },
        "note": "upper bounds: an empty window is conclusive, an open one is provisional",
    }
    (args.out_dir / "speed_budget_summary.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False)
    )
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
