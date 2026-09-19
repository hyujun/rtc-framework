"""Derive a constant joint-acceleration box from torque limits (dynamic_catching D-16).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §9. There are no
acceleration data for the arms, but there are torque limits
(``devices.<group>.joint_limits.max_torque`` = URDF ``effort`` = MJCF
``forcerange``). For every sampled state (q, q̇) of the arm:

    τ_dyn,i = η_τ·τ_max,i − |g_i(q)| − |c_i(q, q̇)|

and the box a = s·w is admissible at that state when, for every arm row i,

    Σ_j |M_ij(q)| · w_j · s  ≤  τ_dyn,i

(the worst sign combination of the joint accelerations — a sufficient
condition, typically 2–4× conservative). That LP in the single scalar s has the
closed form s(q, q̇) = min_i τ_dyn,i / (|M| w)_i. The box is s* · w with
s* = min over all samples; the row achieving the minimum is the binding joint.

**Degenerate results are not adopted** (plan §9 step 6): if any sample has
τ_dyn,i ≤ 0 (gravity + velocity terms alone exceed the budget) or s* is below
``--min-accel``, the YAML carries ``adopted: false`` and the tool exits 2 — the
user narrows the sampling range, changes η_τ or considers a pose-dependent
limit instead.

Scope and simplifications (all recorded in the provenance):

* The model is the robot's full URDF model (arm + hand). Hand joints are held at
  a fixed configuration (default: zero) with zero velocity and acceleration, so
  the hand's mass enters M, g and c of the arm rows; the hand-acceleration
  coupling block M_arm,hand is not bounded and its largest magnitude is
  reported instead.
* For closed-chain hands (a spanning-tree URDF + closure sidecar) the zero hand
  configuration need not close the loops. The effect on the arm rows is the
  hand's mass distribution only.
* The sampling range defaults to the whole joint-limit box (URDF ∩ robot
  config) and ±max_velocity — the most conservative choice. Narrow it with
  ``--q-center/--q-halfwidth`` once the wait pose is known (S3.5a).

Cross-checks (plan §9): ``--check rnea`` recomputes τ = RNEA(q, q̇, σ·a) for
every sign pattern σ at fresh samples (an independent code path from the
|M|-bound above) and requires |τ_i| ≤ η_τ·τ_max,i. ``--check mujoco`` (needs the
``mujoco`` module and an MJCF whose joint names match) does the same with
``mj_inverse``, which includes the MJCF armature / damping terms Pinocchio does
not know, against the actuator ``forcerange`` — "derived ≤ achievable".
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import itertools
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

EXIT_OK = 0
EXIT_DEGENERATE = 2
EXIT_CHECK_FAILED = 3


# ── Robot config (ROS 2 ``/**: ros__parameters`` files, later files override) ──


def load_robot_params(paths: list[Path]) -> dict:
    """Merge the ``ros__parameters`` trees of the given files, later wins per key."""
    merged: dict = {}

    def merge(dst: dict, src: dict) -> None:
        for k, v in src.items():
            if isinstance(v, dict) and isinstance(dst.get(k), dict):
                merge(dst[k], v)
            else:
                dst[k] = v  # arrays are replaced wholesale, like rclcpp overlays

    for p in paths:
        doc = yaml.safe_load(Path(p).read_text()) or {}
        for node_params in doc.values():
            if isinstance(node_params, dict) and "ros__parameters" in node_params:
                merge(merged, node_params["ros__parameters"])
    return merged


@dataclass
class ArmSpec:
    group: str
    joint_names: list[str]
    tau_max: np.ndarray
    v_max: np.ndarray
    q_lower: np.ndarray | None = None
    q_upper: np.ndarray | None = None


def arm_spec_from_params(params: dict, group: str) -> ArmSpec:
    try:
        dev = params["devices"][group]
        names = list(dev["joint_state_names"])
        lim = dev["joint_limits"]
        tau = np.asarray(lim["max_torque"], dtype=float)
        vmax = np.asarray(lim["max_velocity"], dtype=float)
    except KeyError as e:
        raise SystemExit(f"robot config lacks devices.{group}.{e.args[0]}") from e
    n = len(names)
    if tau.shape != (n,) or vmax.shape != (n,):
        raise SystemExit(f"devices.{group}: max_torque/max_velocity must have {n} entries")
    lo = lim.get("position_lower")
    hi = lim.get("position_upper")
    return ArmSpec(
        group,
        names,
        tau,
        vmax,
        None if lo is None else np.asarray(lo, dtype=float),
        None if hi is None else np.asarray(hi, dtype=float),
    )


def resolve_urdf_text(params: dict, urdf_override: Path | None) -> tuple[str, str]:
    """Return (urdf_xml, source label). xacro files are expanded."""
    if urdf_override is not None:
        path = urdf_override
        label = str(urdf_override)
    else:
        from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415

        urdf = params.get("urdf", {})
        pkg, rel = urdf.get("package"), urdf.get("path")
        if not pkg or not rel:
            raise SystemExit("robot config lacks urdf.package / urdf.path (or pass --urdf)")
        path = Path(get_package_share_directory(pkg)) / rel
        label = f"package://{pkg}/{rel}"
    if path.suffix == ".xacro":
        out = subprocess.run(
            ["xacro", str(path)], check=True, capture_output=True, text=True, cwd=path.parent
        )
        return out.stdout, label
    return path.read_text(), label


# ── Derivation ────────────────────────────────────────────────────────────────


@dataclass
class ArmIndex:
    q: np.ndarray  # idx_q of each arm joint in the full model
    v: np.ndarray  # idx_v
    other_v: np.ndarray  # every non-arm velocity index


def arm_index(model, joint_names: list[str]) -> ArmIndex:
    q_idx, v_idx = [], []
    for name in joint_names:
        if not model.existJointName(name):
            raise SystemExit(f"joint '{name}' not in the URDF model")
        j = model.joints[model.getJointId(name)]
        if j.nq != 1 or j.nv != 1:
            raise SystemExit(f"joint '{name}' is not a 1-DoF joint")
        q_idx.append(j.idx_q)
        v_idx.append(j.idx_v)
    v_idx_arr = np.asarray(v_idx)
    other = np.setdiff1d(np.arange(model.nv), v_idx_arr)
    return ArmIndex(np.asarray(q_idx), v_idx_arr, other)


@dataclass
class Sampling:
    q_lower: np.ndarray
    q_upper: np.ndarray
    qd_max: np.ndarray
    n: int
    seed: int


def default_sampling(
    model,
    idx: ArmIndex,
    spec: ArmSpec,
    n: int,
    seed: int,
    v_frac: float,
    q_center=None,
    q_halfwidth=None,
) -> Sampling:
    lo = np.asarray(model.lowerPositionLimit)[idx.q].copy()
    hi = np.asarray(model.upperPositionLimit)[idx.q].copy()
    if spec.q_lower is not None:
        lo = np.maximum(lo, spec.q_lower)
    if spec.q_upper is not None:
        hi = np.minimum(hi, spec.q_upper)
    if q_center is not None:
        c = np.asarray(q_center, dtype=float)
        w = np.asarray(q_halfwidth, dtype=float) * np.ones_like(c)
        lo = np.maximum(lo, c - w)
        hi = np.minimum(hi, c + w)
    if not (np.all(np.isfinite(lo)) and np.all(np.isfinite(hi)) and np.all(lo <= hi)):
        raise SystemExit(f"empty or unbounded sampling box: lower={lo}, upper={hi}")
    return Sampling(lo, hi, v_frac * spec.v_max, n, seed)


@dataclass
class Derivation:
    s_star: float
    a_max: np.ndarray
    weights: np.ndarray
    infeasible_ratio: float
    binding_count: np.ndarray
    worst_q: np.ndarray
    worst_qd: np.ndarray
    max_abs_m_arm_hand: float
    samples: int
    s_sampled: float  # min over the random samples, before local refinement
    per_sample_s: np.ndarray = field(repr=False)


def per_state_scale(model, data, q, v, idx: ArmIndex, tau_budget, weights):
    """s(q, q̇) = min_i τ_dyn,i / (|M_arm| w)_i and the binding row; τ_dyn may be ≤ 0."""
    import pinocchio as pin  # noqa: PLC0415

    m_full = pin.crba(model, data, q)
    m_full = np.triu(m_full) + np.triu(m_full, 1).T  # crba fills the upper triangle
    g = pin.computeGeneralizedGravity(model, data, q)
    nle = pin.nonLinearEffects(model, data, q, v)
    c = nle - g
    m_arm = m_full[np.ix_(idx.v, idx.v)]
    tau_dyn = tau_budget - np.abs(g[idx.v]) - np.abs(c[idx.v])
    demand = np.abs(m_arm) @ weights
    ratios = tau_dyn / demand
    i = int(np.argmin(ratios))
    coupling = np.abs(m_full[np.ix_(idx.v, idx.other_v)]).max() if idx.other_v.size else 0.0
    return float(ratios[i]), i, tau_dyn, float(coupling)


def derive(
    model,
    idx: ArmIndex,
    spec: ArmSpec,
    sampling: Sampling,
    eta_tau: float,
    weights: np.ndarray,
    hand_q: np.ndarray,
    refine_starts: int = 10,
) -> Derivation:
    data = model.createData()
    rng = np.random.default_rng(sampling.seed)
    tau_budget = eta_tau * spec.tau_max
    n_arm = len(spec.joint_names)
    binding = np.zeros(n_arm, dtype=int)
    s_all = np.empty(sampling.n)
    infeasible = 0
    worst = (np.inf, None, None)
    coupling_max = 0.0
    starts: list[tuple[float, np.ndarray, np.ndarray]] = []
    for k in range(sampling.n):
        q = hand_q.copy()
        q[idx.q] = rng.uniform(sampling.q_lower, sampling.q_upper)
        v = np.zeros(model.nv)
        v[idx.v] = rng.uniform(-sampling.qd_max, sampling.qd_max)
        s, i, tau_dyn, coupling = per_state_scale(model, data, q, v, idx, tau_budget, weights)
        s_all[k] = s
        binding[i] += 1
        coupling_max = max(coupling_max, coupling)
        if np.any(tau_dyn <= 0.0):
            infeasible += 1
        if s < worst[0]:
            worst = (s, q[idx.q].copy(), v[idx.v].copy())
        starts.append((s, q[idx.q].copy(), v[idx.v].copy()))
        if len(starts) > 4 * refine_starts:
            starts = sorted(starts, key=lambda t: t[0])[:refine_starts]
    s_sampled = float(worst[0])

    # The sample minimum keeps falling as the sample count grows (a heavy tail
    # near the stretched, gravity-loaded poses): refine the worst samples with
    # a bounded local minimisation so the adopted box does not depend on luck.
    # s is non-smooth (min over rows, |·|), hence derivative-free Powell.
    from scipy.optimize import minimize  # noqa: PLC0415

    n_q = len(idx.q)
    lower = np.concatenate([sampling.q_lower, -sampling.qd_max])
    upper = np.concatenate([sampling.q_upper, sampling.qd_max])

    def scale_at(x: np.ndarray) -> float:
        q = hand_q.copy()
        q[idx.q] = np.clip(x[:n_q], sampling.q_lower, sampling.q_upper)
        v = np.zeros(model.nv)
        v[idx.v] = np.clip(x[n_q:], -sampling.qd_max, sampling.qd_max)
        return per_state_scale(model, data, q, v, idx, tau_budget, weights)[0]

    for _, q0, v0 in sorted(starts, key=lambda t: t[0])[:refine_starts]:
        res = minimize(
            scale_at,
            np.concatenate([q0, v0]),
            method="Powell",
            bounds=list(zip(lower, upper, strict=True)),
            options={"maxiter": 200, "xtol": 1e-6, "ftol": 1e-9},
        )
        s_ref = scale_at(res.x)
        if s_ref < worst[0]:
            worst = (
                s_ref,
                np.clip(res.x[:n_q], sampling.q_lower, sampling.q_upper),
                np.clip(res.x[n_q:], -sampling.qd_max, sampling.qd_max),
            )
    s_star = float(worst[0])
    return Derivation(
        s_sampled=s_sampled,
        s_star=s_star,
        a_max=s_star * weights,
        weights=weights,
        infeasible_ratio=infeasible / sampling.n,
        binding_count=binding,
        worst_q=worst[1],
        worst_qd=worst[2],
        max_abs_m_arm_hand=coupling_max,
        samples=sampling.n,
        per_sample_s=s_all,
    )


def make_weights(mode: str, spec: ArmSpec) -> np.ndarray:
    if mode == "uniform":
        return np.ones(len(spec.joint_names))
    if mode == "tau_max":
        return spec.tau_max / spec.tau_max.max()
    raise SystemExit(f"unknown weight mode '{mode}'")


# ── Cross-checks ──────────────────────────────────────────────────────────────


def sign_patterns(n: int):
    return (np.asarray(s, dtype=float) for s in itertools.product((-1.0, 1.0), repeat=n))


def rnea_check(
    model,
    idx: ArmIndex,
    spec: ArmSpec,
    sampling: Sampling,
    eta_tau: float,
    a_max: np.ndarray,
    hand_q: np.ndarray,
    n_states: int,
    seed: int,
) -> dict:
    """max over states × sign patterns of |RNEA_i| / (η_τ τ_max,i); must be ≤ 1."""
    import pinocchio as pin  # noqa: PLC0415

    data = model.createData()
    rng = np.random.default_rng(seed)
    budget = eta_tau * spec.tau_max
    worst = 0.0
    for _ in range(n_states):
        q = hand_q.copy()
        q[idx.q] = rng.uniform(sampling.q_lower, sampling.q_upper)
        v = np.zeros(model.nv)
        v[idx.v] = rng.uniform(-sampling.qd_max, sampling.qd_max)
        for sigma in sign_patterns(len(spec.joint_names)):
            a = np.zeros(model.nv)
            a[idx.v] = sigma * a_max
            tau = pin.rnea(model, data, q, v, a)
            worst = max(worst, float(np.max(np.abs(tau[idx.v]) / budget)))
    return {
        "method": "rnea",
        "states": n_states,
        "patterns": 2 ** len(spec.joint_names),
        "worst_ratio": worst,
        "limit": "eta_tau * tau_max",
        "pass": bool(worst <= 1.0 + 1e-9),
    }


def mujoco_check(
    mjcf: Path, spec: ArmSpec, sampling: Sampling, a_max: np.ndarray, n_states: int, seed: int
) -> dict:
    """mj_inverse torque over actuator forcerange; must be ≤ 1 ("derived ≤ achievable")."""
    import mujoco  # noqa: PLC0415

    m = mujoco.MjModel.from_xml_path(str(mjcf))
    # Free-space motion only: random poses self-collide (a first run saw 2.5 kN·m
    # of contact force on A6 inside qfrc_inverse) and joint-limit constraints
    # are not actuator effort either. Friction loss and equality constraints
    # stay — the motors do have to overcome those.
    m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT) | int(
        mujoco.mjtDisableBit.mjDSBL_LIMIT
    )
    d = mujoco.MjData(m)
    # Joint-space torque range each actuator can deliver: joint torque = gear ·
    # actuator force, so the range is gear · forcerange (sign-aware — an
    # asymmetric range or a negative gear flips which bound applies).
    qadr, dadr, tau_lo, tau_hi = [], [], [], []
    for name in spec.joint_names:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise SystemExit(f"MJCF {mjcf} has no joint '{name}'")
        qadr.append(m.jnt_qposadr[jid])
        dadr.append(m.jnt_dofadr[jid])
        acts = [
            a
            for a in range(m.nu)
            if m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT and m.actuator_trnid[a, 0] == jid
        ]
        if len(acts) != 1 or not m.actuator_forcelimited[acts[0]]:
            raise SystemExit(f"MJCF joint '{name}' needs exactly one force-limited actuator")
        gear = float(m.actuator_gear[acts[0], 0])
        ends = gear * m.actuator_forcerange[acts[0]]
        tau_lo.append(float(ends.min()))
        tau_hi.append(float(ends.max()))
    tau_lo = np.asarray(tau_lo)
    tau_hi = np.asarray(tau_hi)
    if np.any(tau_hi <= 0.0) or np.any(tau_lo >= 0.0):
        raise SystemExit(f"MJCF {mjcf}: an arm actuator cannot drive both directions")
    rng = np.random.default_rng(seed)
    worst = 0.0
    for _ in range(n_states):
        qa = rng.uniform(sampling.q_lower, sampling.q_upper)
        va = rng.uniform(-sampling.qd_max, sampling.qd_max)
        for sigma in sign_patterns(len(spec.joint_names)):
            mujoco.mj_resetData(m, d)
            d.qpos[qadr] = qa
            d.qvel[dadr] = va
            d.qacc[:] = 0.0
            d.qacc[dadr] = sigma * a_max
            mujoco.mj_inverse(m, d)
            tau = d.qfrc_inverse[dadr]
            ratio = np.where(tau >= 0.0, tau / tau_hi, tau / tau_lo)
            worst = max(worst, float(np.max(ratio)))
    return {
        "method": "mujoco mj_inverse",
        "mjcf": str(mjcf),
        "states": n_states,
        "patterns": 2 ** len(spec.joint_names),
        "worst_ratio": worst,
        "limit": "gear * actuator forcerange (per direction)",
        "disabled": ["contact", "joint limit"],
        "pass": bool(worst <= 1.0),
    }


# ── Output ────────────────────────────────────────────────────────────────────


def _git_head(start: Path) -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=start,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def build_report(
    spec: ArmSpec,
    der: Derivation,
    sampling: Sampling,
    eta_tau: float,
    weight_mode: str,
    urdf_label: str,
    urdf_text: str,
    configs: list[Path],
    hand_q_note: str,
    min_accel: float,
    checks: list[dict],
) -> dict:
    degenerate_reasons = []
    if der.infeasible_ratio > 0.0:
        degenerate_reasons.append(
            f"{der.infeasible_ratio:.4f} of samples have tau_dyn <= 0 on some joint"
        )
    if not der.s_star >= min_accel:  # also catches NaN
        degenerate_reasons.append(f"s* = {der.s_star:.4g} < min_accel {min_accel}")
    checks_ok = all(c["pass"] for c in checks)
    adopted = not degenerate_reasons and checks_ok
    r = lambda x: [round(float(v), 6) for v in np.atleast_1d(x)]  # noqa: E731
    return {
        "derived_accel_limits": {
            spec.group: {
                "qdd_max": r(der.a_max),
                "adopted": adopted,
                # eta_tau, the sampling range and the weights await the user's
                # confirmation (plan §9, S2.5); the range becomes the wait-pose
                # neighbourhood after S3.5a.
                "provisional": True,
                "degenerate_reasons": degenerate_reasons,
                "cross_checks": checks,
                "provenance": {
                    "tool": "rtc_tools.analysis.derive_accel_limits",
                    "date": _dt.date.today().isoformat(),
                    "git_head": _git_head(Path(__file__).parent),
                    "urdf": urdf_label,
                    "urdf_sha256": hashlib.sha256(urdf_text.encode()).hexdigest(),
                    "robot_config": [str(p) for p in configs],
                    "joint_names": spec.joint_names,
                    "tau_max": r(spec.tau_max),
                    "eta_tau": eta_tau,
                    "weight_mode": weight_mode,
                    "weights": r(der.weights),
                    "s_star": round(der.s_star, 6),
                    "s_sampled_min": round(der.s_sampled, 6),
                    "refinement": "Powell from the 10 worst samples, bounded to the sampling box",
                    "samples": der.samples,
                    "seed": sampling.seed,
                    "q_range": {"lower": r(sampling.q_lower), "upper": r(sampling.q_upper)},
                    "qd_range_abs": r(sampling.qd_max),
                    "hand_joints": hand_q_note,
                    "infeasible_ratio": der.infeasible_ratio,
                    "binding_count": {
                        n: int(c) for n, c in zip(spec.joint_names, der.binding_count, strict=True)
                    },
                    "worst_state": {"q": r(der.worst_q), "qd": r(der.worst_qd)},
                    "max_abs_M_arm_hand": round(der.max_abs_m_arm_hand, 6),
                    "method": "sufficient condition sum_j |M_ij| a_j <= eta*tau_max - |g_i| - "
                    "|c_i| over all samples (worst-sign, 2-4x conservative)",
                },
            }
        }
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument(
        "--robot-config",
        type=Path,
        nargs="+",
        required=True,
        help="robot config YAML(s), later files override (e.g. _base.yaml sim.yaml)",
    )
    ap.add_argument("--group", required=True, help="device group of the arm (devices.<group>)")
    ap.add_argument(
        "--eta-tau",
        type=float,
        required=True,
        help="torque fraction for the dynamic budget, 0 < eta <= 1 (no default: "
        "it is a decision, plan §9)",
    )
    ap.add_argument("--urdf", type=Path, help="URDF/xacro override (default: urdf.package/path)")
    ap.add_argument("--samples", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--v-frac", type=float, default=1.0, help="sample |q̇| up to v_frac·max_velocity"
    )
    ap.add_argument("--weights", choices=["uniform", "tau_max"], default="uniform")
    ap.add_argument("--q-center", type=float, nargs="+", help="restrict q to center ± halfwidth")
    ap.add_argument("--q-halfwidth", type=float, nargs="+")
    ap.add_argument(
        "--min-accel",
        type=float,
        default=0.1,
        help="s* below this [rad/s²] is degenerate (not adopted)",
    )
    ap.add_argument("--check", choices=["rnea", "mujoco"], action="append", default=[])
    ap.add_argument("--check-states", type=int, default=200)
    ap.add_argument("--mjcf", type=Path, help="MJCF for --check mujoco")
    ap.add_argument("--out", type=Path, help="write the YAML here (default: stdout)")
    args = ap.parse_args(argv)

    if not (0.0 < args.eta_tau <= 1.0):
        raise SystemExit("--eta-tau must be in (0, 1]")
    if (args.q_center is None) != (args.q_halfwidth is None):
        raise SystemExit("--q-center and --q-halfwidth go together")

    import pinocchio as pin  # noqa: PLC0415

    params = load_robot_params(args.robot_config)
    spec = arm_spec_from_params(params, args.group)
    urdf_text, urdf_label = resolve_urdf_text(params, args.urdf)
    model = pin.buildModelFromXML(urdf_text)
    idx = arm_index(model, spec.joint_names)
    sampling = default_sampling(
        model, idx, spec, args.samples, args.seed, args.v_frac, args.q_center, args.q_halfwidth
    )
    hand_q = pin.neutral(model)
    weights = make_weights(args.weights, spec)

    der = derive(model, idx, spec, sampling, args.eta_tau, weights, hand_q)
    checks = []
    for c in args.check:
        if c == "rnea":
            checks.append(
                rnea_check(
                    model,
                    idx,
                    spec,
                    sampling,
                    args.eta_tau,
                    der.a_max,
                    hand_q,
                    args.check_states,
                    args.seed + 1,
                )
            )
        else:
            if args.mjcf is None:
                raise SystemExit("--check mujoco needs --mjcf")
            checks.append(
                mujoco_check(
                    args.mjcf, spec, sampling, der.a_max, args.check_states, args.seed + 2
                )
            )
    report = build_report(
        spec,
        der,
        sampling,
        args.eta_tau,
        args.weights,
        urdf_label,
        urdf_text,
        args.robot_config,
        "held at the URDF neutral configuration, zero velocity/acceleration",
        args.min_accel,
        checks,
    )
    text = (
        "# Generated by rtc_tools.analysis.derive_accel_limits — rerun the tool, do not\n"
        "# hand-edit the values (dynamic_catching D-16, plan §9).\n"
        + yaml.safe_dump(report, sort_keys=False, default_flow_style=None, width=100)
    )
    if args.out:
        args.out.write_text(text)
    else:
        sys.stdout.write(text)

    entry = report["derived_accel_limits"][spec.group]
    if entry["degenerate_reasons"]:
        print("degenerate: " + "; ".join(entry["degenerate_reasons"]), file=sys.stderr)
        return EXIT_DEGENERATE
    if not all(c["pass"] for c in checks):
        print("cross-check failed: " + str([c for c in checks if not c["pass"]]), file=sys.stderr)
        return EXIT_CHECK_FAILED
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
