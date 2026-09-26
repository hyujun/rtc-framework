"""Arm-budget attribution (dynamic_catching S8-G, #537).

"The arm cannot get there" has four layers, and a catch that fails at t_c does
not say which one bound:

* **P** the plant — servo lag τ, torque headroom, joint speed actually reached;
* **R** the L4 reference — ω (how fast the DS converges), ``a_max`` / ``v_max``
  (where it saturates);
* **C** the CLIK step — the joint velocity box (the device rating) and the
  acceleration constraint;
* **B** the planner — the D-16 acceleration box its reach time judges with,
  which since decision K (``joint_cmd.accel_constraint: dynamic``) is not what
  the CLIK executes.

This tool reads one launch's UNIT (``catching_sim_trials`` output + its
``catching_trials`` evaluation) together with the controller session that
produced it and attributes, per trial and per unit:

* the reference's own residual ``‖e‖`` at commit and at the last active tick
  (t_c − T_arm, the tick whose command aims at t_c) — the arm-side share of the
  hand–ball gap — next to the reference-vs-truth and hand-vs-truth distances
  ``catching_trials`` measured, so the prediction share is the difference;
* what the reference asked for (``‖u_des‖``) against ``a_max``, and how often it
  saturated;
* what the CLIK commanded: the joint acceleration ENVELOPE (p95 / max of the
  smoothed ``q̈_cmd`` over the active ticks) and how often the velocity box bound;
* what the plant did: τ̂ per joint (``catching_trials.servo_lag_ls``), torque
  utilisation from the device lane, the joint speed reached;
* the planner's reach time re-judged twice for the SAME motion — with the D-16
  box the controller loaded and with the executed envelope — against the lead
  it had, so the box's pessimism is a number, not an impression;
* the rank-gate failure rates among the planner's valid plans.

The convergence check is analytic: a critically damped second-order reference
from rest leaves ``(1 + ωT) e^{−ωT}`` of its initial error after ``T``; the tool
prints that fraction beside the measured one so a residual that is just ω × T
is not mistaken for saturation or tracking.

Every limit comes from the run, never from code (ARCH-1): joints from the
diag's ``q_cmd_*`` columns, torque and velocity ratings from the profile's
device roster (``_base.yaml``, with ``sim.yaml``'s overlay of the rating when it
has one — the launch composes them the same way), ω / ``a_max`` / ``v_max`` /
η_v / the D-16 box from the controller's read-only MIRROR in ``run_meta.json``
when the runner recorded them (S8-G), else from the profile plus the overlays
named on the command line. The source is reported.

``--write-envelope-box`` turns the pooled executed envelope into a
``derived_accel_limits`` file (the format ``robot.arm.accel_limits_path``
loads) with provenance, so a sim overlay can point the planner's reach time at
what the arm actually did. It is marked ``provisional`` and sim-only.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis import catching_trials as ct
from rtc_tools.analysis.catch_gate_map import load_accel_box

TOOL = "catching_arm_budget"
ACTIVE_MODES = (ct.MODE_APPROACH, ct.MODE_COMMITTED, ct.MODE_CLOSING)
MIN_SEGMENT_TICKS = 25
ACCEL_SMOOTH_TICKS = 5
ENVELOPE_PERCENTILE = 95.0
VELOCITY_BOX_HIT = 0.98  # |q̇_cmd| / box above this counts as "at the box"
A_MAX_HIT = 0.999
MOVING_RAD_S = 0.05  # q̇_meas above this enters the servo-lag fit
COMMIT_JOIN_TOL_S = 0.02
DEFAULT_N_BOOT = 200
DEFAULT_SEED = 0
PLANNER_TIME_MARGIN_S = 0.03  # planner.time.margin default (PlannerParams::time_margin, L3 §4.3)


# ── Reach time (L3 §4.3 closed form, the runtime's `TMinChecked`) ────────────
def t_min_joint(d: float, w: float, w_max: float, a_max: float) -> float:
    """Minimum time from (0, w) to (d, 0) under |q̇| ≤ w_max, |q̈| ≤ a_max.

    ``w`` is the initial velocity signed towards the target. NaN when the
    limits are not positive-finite or |w| > w_max (the problem is undefined —
    the runtime flags it and fails the gate).
    """
    if not (a_max > 0.0 and w_max > 0.0 and math.isfinite(a_max) and math.isfinite(w_max)):
        return math.nan
    if not (math.isfinite(d) and math.isfinite(w)) or abs(w) > w_max:
        return math.nan
    if d < 0.0:
        d, w = -d, -w
    if d == 0.0 and w == 0.0:
        return 0.0
    if w < 0.0:  # moving away: stop first, then a rest-to-rest move
        return -w / a_max + _t_rest(d + w * w / (2.0 * a_max), w_max, a_max)
    d_stop = w * w / (2.0 * a_max)
    if d_stop > d:  # overshoot: stop, then come back
        return w / a_max + _t_rest(d_stop - d, w_max, a_max)
    w_peak = math.sqrt(a_max * d + 0.5 * w * w)
    if w_peak <= w_max:  # triangle
        return (2.0 * w_peak - w) / a_max
    cruise = d - (w_max * w_max - w * w) / (2.0 * a_max) - w_max * w_max / (2.0 * a_max)
    return (w_max - w) / a_max + w_max / a_max + cruise / w_max


def _t_rest(d: float, w_max: float, a_max: float) -> float:
    if d <= 0.0:
        return 0.0
    w_peak = math.sqrt(a_max * d)
    return 2.0 * math.sqrt(d / a_max) if w_peak <= w_max else d / w_max + w_max / a_max


def reach_time(
    dq: Sequence[float], w0: Sequence[float], w_max: Sequence[float], a: Sequence[float]
) -> float:
    """max over joints of :func:`t_min_joint` (NaN if any joint is NaN)."""
    times = [
        t_min_joint(float(d), float(w), float(wm), float(am))
        for d, w, wm, am in zip(dq, w0, w_max, a, strict=True)
    ]
    return math.nan if any(math.isnan(t) for t in times) else max(times)


def ds_residual_fraction(omega: float, duration_s: float) -> float:
    """Critically damped second-order reference from rest: e(T)/e0 = (1 + ωT) e^{−ωT}."""
    if not (omega > 0.0 and duration_s >= 0.0):
        return math.nan
    x = omega * duration_s
    return (1.0 + x) * math.exp(-x)


# ── Configuration: where each limit comes from ───────────────────────────────
@dataclass
class ArmBudget:
    """The budget layers one unit ran with, and where each number came from."""

    joints: list[str]
    tau_max: list[float]  # N·m, device roster
    qd_box: list[float]  # rad/s, the CLIK velocity box (rating, sim overlay applied)
    omega: float
    a_max: float
    v_max: float
    eta_v: float
    qdd_box: list[float]  # rad/s², the D-16 box the planner judged with ([] = none)
    t_arm_s: float
    dt: float
    time_margin_s: float = PLANNER_TIME_MARGIN_S  # planner.time.margin the reach gate used
    source: dict = field(default_factory=dict)

    @property
    def qd_plan(self) -> list[float]:
        return [self.eta_v * v for v in self.qd_box]


def _device_limits(config_dir: Path, device: str) -> tuple[dict, dict]:
    """``devices.<device>.joint_limits`` of ``_base.yaml`` with ``sim.yaml``'s keys laid over it.

    Overlays replace a key, so a sim profile that lifts the velocity rating
    (dynamic_catching decision D) changes only that list; everything else is
    the real roster's. Returns (limits, source-per-key).
    """
    limits: dict = {}
    source: dict = {}
    for name in ct.ROBOT_CONFIG_FILES:
        path = Path(config_dir) / name
        if not path.is_file():
            continue
        params = ct._ros_params(ct._load_yaml(path))
        dev = ((params.get("devices") or {}).get(device) or {}).get("joint_limits") or {}
        for key, value in dev.items():
            limits[key] = value
            source[key] = name
    return limits, source


def _deep_merge(base: Mapping, over: Mapping) -> dict:
    out = dict(base)
    for k, v in over.items():
        out[k] = (
            _deep_merge(out[k], v)
            if isinstance(v, Mapping) and isinstance(out.get(k), Mapping)
            else v
        )
    return out


def _overlay_catching(path: Path, controller: str) -> dict:
    """The ``catching`` subtree a node-level YAML writes for ``controller`` ({} if none).

    Serves both a ``sim_overlays/*.yaml`` and the robot config's ``sim.yaml``:
    the launch feeds both to the RT node, which lays ``<controller>.catching``
    over the controller YAML (``ApplyControllerParamOverrides``).
    """
    doc = ct._load_yaml(path)
    for node in doc.values():
        params = (node or {}).get("ros__parameters") or {}
        tree = params.get(controller)
        if isinstance(tree, Mapping) and "catching" in tree:
            return dict(tree["catching"])
    return {}


def _box_from_file(
    config_dir: Path, package: str, rel_path: str, group: str, n: int
) -> list[float]:
    """``derived_accel_limits.<group>.qdd_max`` from the profile's package-relative path.

    [] when the file is absent or its box is not ``adopted`` — what the
    controller does (``LoadDerivedAccelLimits`` refuses a non-adopted box).
    """
    share = Path(config_dir).parent.parent  # <share>/config/<robot> → <share>
    candidates = [share / rel_path]
    try:
        from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415

        candidates.append(Path(get_package_share_directory(package)) / rel_path)
    except Exception:  # noqa: BLE001 — no ament on the analysis host
        pass
    for path in candidates:
        if path.is_file():
            return [float(v) for v in load_accel_box(path, group, n, require_adopted=True)]
    return []


def resolve_budget(
    profile: ct.CatchingProfile,
    node: Mapping,
    config_dir: Path,
    joints: Sequence[str],
    meta: Mapping,
    overlays: Sequence[Path],
    dt: float,
    t_arm_s: float,
) -> ArmBudget:
    """The limits one unit ran with: the controller's mirror first, the files else.

    The file path composes what the launch composes, in its order: the
    controller YAML, then the robot config's ``sim.yaml`` (which may carry a
    ``<controller>.catching`` override — R2 of D-S8-18 points the sim planner's
    box there), then the ``--overlay`` files.
    """
    limits, lim_src = _device_limits(config_dir, profile.arm_device)
    n = len(joints)
    tau_max = [float(v) for v in (limits.get("max_torque") or [math.nan] * n)]
    qd_box = [float(v) for v in (limits.get("max_velocity") or [math.nan] * n)]
    if len(tau_max) != n or len(qd_box) != n:
        raise SystemExit(
            f"devices.{profile.arm_device}.joint_limits: max_torque / max_velocity must have {n} "
            f"entries (the diag's arm joints), got {len(tau_max)} / {len(qd_box)}"
        )
    source = {"max_torque": lim_src.get("max_torque"), "max_velocity": lim_src.get("max_velocity")}
    mirror = meta.get("controller_mirror") or {}
    catching = dict(node.get("catching") or {})
    composed = ["profile"]
    sim_yaml = Path(config_dir) / "sim.yaml"
    if sim_yaml.is_file():
        sim_tree = _overlay_catching(sim_yaml, profile.controller)
        if sim_tree:
            catching = _deep_merge(catching, sim_tree)
            composed.append("sim.yaml")
    for path in overlays:
        catching = _deep_merge(catching, _overlay_catching(path, profile.controller))
    if overlays:
        composed.append("overlays")
    files = "+".join(composed)
    ref = catching.get("reference") or {}
    planner = catching.get("planner") or {}
    gamma = planner.get("gamma") or {}
    arm = (catching.get("robot") or {}).get("arm") or {}

    def pick(mirror_key: str, file_value, label: str) -> float:
        if mirror.get(mirror_key) is not None:
            source[label] = "controller_mirror"
            return float(mirror[mirror_key])
        source[label] = files
        return math.nan if file_value is None else float(file_value)

    omega = pick("reference.omega", ref.get("omega", 10.0), "omega")
    a_max = pick("reference.a_max", ref.get("a_max"), "a_max")
    v_max = pick("reference.v_max", ref.get("v_max"), "v_max")
    eta_v = pick("planner.gamma.eta_v", gamma.get("eta_v", 0.9), "eta_v")
    time_margin = pick(
        "planner.time.margin",
        (planner.get("time") or {}).get("margin", PLANNER_TIME_MARGIN_S),
        "time_margin",
    )
    if mirror.get("robot.arm.qdd_max") is not None:
        qdd_box = [float(v) for v in mirror["robot.arm.qdd_max"]]
        source["qdd_box"] = "controller_mirror"
    else:
        qdd_box = _box_from_file(
            config_dir,
            str(arm.get("accel_limits_package", "")),
            str(arm.get("accel_limits_path", "")),
            str(arm.get("accel_limits_group", "")),
            n,
        )
        source["qdd_box"] = f"{arm.get('accel_limits_path')} ({files})"
    if qdd_box and len(qdd_box) != n:
        raise SystemExit(f"the D-16 box has {len(qdd_box)} entries, the arm {n} joints")
    return ArmBudget(
        list(joints),
        tau_max,
        qd_box,
        omega,
        a_max,
        v_max,
        eta_v,
        qdd_box,
        t_arm_s,
        dt,
        time_margin,
        source,
    )


# ── One unit ─────────────────────────────────────────────────────────────────
@dataclass
class Segment:
    """One active stretch (APPROACH → COMMITTED → CLOSING) of the diag."""

    start: int
    end: int  # exclusive
    commit: int | None  # first COMMITTED tick, None when the stretch never committed


def active_segments(mode: np.ndarray, min_ticks: int = MIN_SEGMENT_TICKS) -> list[Segment]:
    active = np.isin(mode, ACTIVE_MODES)
    edges = np.flatnonzero(np.diff(np.concatenate(([0], active.astype(int), [0]))))
    out = []
    for a, b in zip(edges[::2], edges[1::2], strict=True):
        if b - a < min_ticks:
            continue
        commits = np.flatnonzero(mode[a:b] == ct.MODE_COMMITTED)
        out.append(Segment(int(a), int(b), int(a + commits[0]) if commits.size else None))
    return out


def smoothed_accel(
    q_cmd: np.ndarray, dt: float, window: int = ACCEL_SMOOTH_TICKS
) -> tuple[np.ndarray, np.ndarray]:
    """(q̇, q̈) of a command trajectory; q̈ box-averaged over ``window`` ticks (QP output is stepwise)."""
    qd = np.gradient(q_cmd, dt, axis=0)
    qdd = np.gradient(qd, dt, axis=0)
    kernel = np.ones(window) / window
    qdd_s = np.column_stack(
        [np.convolve(qdd[:, j], kernel, mode="same") for j in range(q_cmd.shape[1])]
    )
    return qd, qdd_s


def _diag_columns(header: Sequence[str], joints: Sequence[str]) -> list[str]:
    fixed = [
        "t_relative_s",
        "mode",
        "plan_t_c_s",
        "ref_saturated",
        "clik_bound_conflict",
        "track_err_rad",
    ]
    vec = [f"{p}_{ax}" for p in ("ref_xd", "ref_u_des", "ref_xdd", "ref_e") for ax in "xyz"]
    missing = [c for c in fixed + vec if c not in header]
    if missing:
        raise SystemExit(f"catching diag lacks column(s) {missing}")
    lead = [ct.DIAG_LEAD_COLUMN] if ct.DIAG_LEAD_COLUMN in header else []
    return fixed + vec + lead + [f"q_cmd_{j}" for j in joints] + [f"q_meas_{j}" for j in joints]


def _norm(df, prefix: str) -> np.ndarray:
    return np.linalg.norm(
        df[[f"{prefix}_x", f"{prefix}_y", f"{prefix}_z"]].to_numpy(dtype=float), axis=1
    )


def _q(values: np.ndarray, p: float) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, p)) if values.size else math.nan


def _effort_utilisation(
    session: Path,
    profile: ct.CatchingProfile,
    joints: Sequence[str],
    tau_max: Sequence[float],
    t: np.ndarray,
    active: np.ndarray,
    dt: float,
) -> dict:
    """max / p99 of |effort| / τ_max per joint over the active ticks (NaN without a device lane).

    The device lane is written by the same RT tick as the diag but is its own
    file, and a row can be missing on either side (one dropped lane row was
    seen in a 130 k-tick session); rows are therefore matched by
    ``t_relative_s`` (nearest within half a tick), never by position.
    """
    log = profile.device_logs.get(profile.arm_device)
    n = len(joints)
    empty = {"max": [math.nan] * n, "p99": [math.nan] * n, "source": None, "matched_ticks": 0}
    if not log:
        return empty
    path = session / "controllers" / profile.controller / f"{log}.csv"
    found = ct._exists(path)
    if found is None:
        return empty
    cols = [f"effort_{j}" for j in joints]
    header = ct._csv_header(found)
    if any(c not in header for c in cols) or "t_relative_s" not in header:
        return {**empty, "source": f"{path.name}: no effort_* / t_relative_s columns"}
    lane = ct._read_csv(path, usecols=["t_relative_s", *cols])
    t_lane = lane["t_relative_s"].to_numpy(dtype=float)
    t_active = t[active]
    if len(t_lane) == 0 or len(t_active) == 0:
        return {**empty, "source": f"{path.name}: no lane row (header only)"}
    idx = np.clip(np.searchsorted(t_lane, t_active), 0, len(t_lane) - 1)
    left = np.clip(idx - 1, 0, len(t_lane) - 1)
    take = np.where(np.abs(t_lane[left] - t_active) < np.abs(t_lane[idx] - t_active), left, idx)
    ok = np.abs(t_lane[take] - t_active) <= 0.5 * dt
    if not ok.any():
        return {
            **empty,
            "source": f"{path.name}: no lane row within half a tick of an active tick",
        }
    util = np.abs(lane[cols].to_numpy(dtype=float)[take[ok]]) / np.asarray(tau_max, dtype=float)
    return {
        "max": [float(v) for v in util.max(axis=0)],
        "p99": [float(v) for v in np.percentile(util, 99, axis=0)],
        "source": path.name,
        "matched_ticks": int(ok.sum()),
    }


def _rank_rates(path: Path | None) -> dict:
    if path is None or ct._exists(path) is None:
        return {"valid_plans": 0}
    ev = ct._read_csv(path)
    valid = ev[ev["plan_valid"] == 1] if "plan_valid" in ev else ev.iloc[0:0]
    out = {"valid_plans": int(len(valid))}
    for k in (
        "rank_reach",
        "rank_gamma",
        "rank_rollout",
        "rank_error_budget",
        "rank_uncertainty",
        "rank_commit_lead",
    ):
        out[k] = float(valid[k].mean()) if k in valid and len(valid) else math.nan
    out["lead_s_p50"] = (
        float(valid["lead_s"].median()) if "lead_s" in valid and len(valid) else math.nan
    )
    out["gamma_f_p50"] = (
        float(valid["gamma_f"].median()) if "gamma_f" in valid and len(valid) else math.nan
    )
    return out


def analyse_unit(
    unit: Path,
    session: Path,
    config_dir: Path,
    *,
    controller: str | None = None,
    overlays: Sequence[Path] = (),
    time_margin_s: float | None = None,
    n_boot: int = DEFAULT_N_BOOT,
    seed: int = DEFAULT_SEED,
) -> dict:
    """Everything the module docstring lists, for one unit.

    ``time_margin_s`` None = the ``planner.time.margin`` the unit ran with
    (mirror, else the composed files); a number overrides it.
    """
    meta = json.loads((unit / "trials" / "run_meta.json").read_text())
    profile = ct.load_profile(config_dir, controller, session)
    node = ct._catching_controllers(config_dir)[profile.controller]
    ctl = session / "controllers" / profile.controller
    diag_path = ctl / f"{profile.diag_log or 'catching_diag'}.csv"
    header = ct._csv_header(ct._exists(diag_path) or diag_path)
    joints = ct.arm_joints_from_diag(header)
    diag = ct._read_csv(diag_path, usecols=_diag_columns(header, joints))
    t = diag["t_relative_s"].to_numpy(dtype=float)
    mode = diag["mode"].to_numpy()
    dt = float(
        (meta.get("controller_mirror") or {}).get("control.dt")
        or np.median(np.diff(t[: min(len(t), 2000)]))
    )
    t_arm = (
        float(diag[ct.DIAG_LEAD_COLUMN].iloc[0])
        if ct.DIAG_LEAD_COLUMN in diag
        else (ct._mirror_lead(meta) or 0.0)
    )
    budget = resolve_budget(profile, node, config_dir, joints, meta, overlays, dt, t_arm)
    if time_margin_s is None:
        time_margin_s = budget.time_margin_s

    q_cmd = diag[[f"q_cmd_{j}" for j in joints]].to_numpy(dtype=float)
    q_meas = diag[[f"q_meas_{j}" for j in joints]].to_numpy(dtype=float)
    e_ref = 1e3 * _norm(diag, "ref_e")
    xd = _norm(diag, "ref_xd")
    u_des = _norm(diag, "ref_u_des")
    xdd = _norm(diag, "ref_xdd")
    sat = diag["ref_saturated"].to_numpy(dtype=float) > 0.5
    conflict = diag["clik_bound_conflict"].to_numpy(dtype=float) > 0.5
    plan_t_c = diag["plan_t_c_s"].to_numpy(dtype=float)

    segments = active_segments(mode)
    active = np.zeros(len(t), dtype=bool)
    clusters = np.full(len(t), -1)
    qd_all = np.zeros_like(q_cmd)
    qdd_all = np.zeros_like(q_cmd)
    for i, s in enumerate(segments):
        active[s.start : s.end] = True
        clusters[s.start : s.end] = i
        qd_all[s.start : s.end], qdd_all[s.start : s.end] = smoothed_accel(
            q_cmd[s.start : s.end], dt
        )
    if not segments:
        raise SystemExit(
            f"{unit}: no active (APPROACH/COMMITTED/CLOSING) stretch of ≥ {MIN_SEGMENT_TICKS} ticks"
        )

    # ── C: the executed envelope and the velocity box ─────────────────────────
    qdd_abs = np.abs(qdd_all[active])
    qd_abs = np.abs(qd_all[active])
    envelope_p95 = [float(v) for v in np.percentile(qdd_abs, ENVELOPE_PERCENTILE, axis=0)]
    envelope_max = [float(v) for v in qdd_abs.max(axis=0)]
    # A joint that never accelerated has an envelope of 0, which the closed form
    # reads as an invalid limit (NaN, fail closed). It needs no acceleration for
    # a zero move, so the re-judgement floors the envelope at a hair above 0: a
    # joint that did move against a zero envelope still gets an honest, huge time.
    envelope_for_reach = [max(v, 1e-6) for v in envelope_p95]
    qd_ratio = qd_abs / np.asarray(budget.qd_box, dtype=float)
    velocity_box_hit_frac = float((qd_ratio.max(axis=1) >= VELOCITY_BOX_HIT).mean())
    over_box_frac = math.nan
    if budget.qdd_box:
        # Per joint against ITS box entry — the envelope box is far from uniform.
        over_box_frac = float(
            (qdd_abs > np.asarray(budget.qdd_box, dtype=float)).any(axis=1).mean()
        )

    # ── P: plant ─────────────────────────────────────────────────────────────
    qd_meas = np.zeros_like(q_meas)
    for s in segments:
        qd_meas[s.start : s.end] = np.gradient(q_meas[s.start : s.end], dt, axis=0)
    moving = active & (np.abs(qd_meas).max(axis=1) > MOVING_RAD_S)
    lags = ct.servo_lag_ls(t, q_cmd, q_meas, moving, clusters, joints, n_boot=n_boot, seed=seed)
    torque = _effort_utilisation(session, profile, joints, budget.tau_max, t, active, dt)
    qd_meas_max = [float(v) for v in np.abs(qd_meas[active]).max(axis=0)]

    # ── R: reference ─────────────────────────────────────────────────────────
    ref = {
        "saturated_frac": float(sat[active].mean()),
        "u_des_p50": _q(u_des[active], 50),
        "u_des_max": float(u_des[active].max()),
        "a_max_hit_frac": float((xdd[active] >= A_MAX_HIT * budget.a_max).mean())
        if budget.a_max > 0
        else math.nan,
        "xd_max": float(xd[active].max()),
        "v_max_hit_frac": float((xd[active] >= A_MAX_HIT * budget.eta_v * budget.v_max).mean())
        if budget.v_max > 0
        else math.nan,
        "bound_conflict_frac": float(conflict[active].mean()),
    }

    # ── per trial: residuals and the reach time re-judged ───────────────────
    trials_csv = unit / "ct" / "catching_trials.csv"
    ct_rows = ct._read_csv(trials_csv) if ct._exists(trials_csv) else None
    rows = []
    for i, s in enumerate(segments):
        if s.commit is None:
            continue
        c, last = s.commit, s.end - 1
        dq = q_cmd[last] - q_cmd[c]
        w0 = qd_all[c]
        avail = float(plan_t_c[c]) - budget.t_arm_s - time_margin_s
        row = {
            "segment": i,
            "t_commit": float(t[c]),
            "duration_s": float(t[last] - t[c]),
            "e_commit_mm": float(e_ref[c]),
            "e_last_mm": float(e_ref[last]),
            "e_min_mm": float(e_ref[c : s.end].min()),
            "saturated_frac": float(sat[c : s.end].mean()),
            "dq_max_rad": float(np.abs(dq).max()),
            "lead_avail_s": avail,
            "t_reach_box_s": reach_time(dq, w0, budget.qd_plan, budget.qdd_box)
            if budget.qdd_box
            else math.nan,
            "t_reach_envelope_s": reach_time(dq, w0, budget.qd_plan, envelope_for_reach),
        }
        row["reach_ok_box"] = (
            bool(row["t_reach_box_s"] <= avail) if math.isfinite(row["t_reach_box_s"]) else None
        )
        row["reach_ok_envelope"] = (
            bool(row["t_reach_envelope_s"] <= avail)
            if math.isfinite(row["t_reach_envelope_s"])
            else None
        )
        if ct_rows is not None and "t_commit" in ct_rows:
            tc = ct_rows["t_commit"].to_numpy(dtype=float)
            k = int(np.nanargmin(np.abs(tc - t[c]))) if np.isfinite(tc).any() else -1
            if k >= 0 and abs(tc[k] - t[c]) <= COMMIT_JOIN_TOL_S:
                r = ct_rows.iloc[k]
                for col in (
                    "idx",
                    "ref_vs_true_mm",
                    "total_mm",
                    "d_min_mm",
                    "pred_mm",
                    "clik_mm",
                    "servo_mm",
                    "ball_speed_tc",
                    "gamma_f_planned",
                    "contact_v_rel",
                ):
                    if col in r:
                        row[col] = float(r[col]) if col != "idx" else int(r[col])
                row["truth_success"] = str(r.get("truth_success", "")).strip().lower() == "true"
                if math.isfinite(row.get("ref_vs_true_mm", math.nan)):
                    row["pred_live_lb_mm"] = max(0.0, row["ref_vs_true_mm"] - row["e_last_mm"])
        rows.append(row)

    def col(name: str) -> np.ndarray:
        return np.asarray([r.get(name, math.nan) for r in rows], dtype=float)

    durations = col("duration_s")
    duration_p50 = _q(durations, 50)
    reach_box = [r["reach_ok_box"] for r in rows if r["reach_ok_box"] is not None]
    reach_env = [r["reach_ok_envelope"] for r in rows if r["reach_ok_envelope"] is not None]
    summary = {
        "tool": TOOL,
        "unit": str(unit),
        "session": str(session),
        "arm": meta.get("arm"),
        "budget": {
            "joints": joints,
            "omega": budget.omega,
            "a_max": budget.a_max,
            "v_max": budget.v_max,
            "eta_v": budget.eta_v,
            "qd_box": budget.qd_box,
            "qdd_box": budget.qdd_box,
            "tau_max": budget.tau_max,
            "t_arm_s": budget.t_arm_s,
            "time_margin_s": time_margin_s,
            "dt": dt,
            "source": budget.source,
        },
        "segments": len(segments),
        "trials_committed": len(rows),
        "plant": {
            "tau_hat_s": [lag.tau_s for lag in lags],
            "tau_hat_ci_s": [[lag.ci_low_s, lag.ci_high_s] for lag in lags],
            "tau_hat_r2": [lag.r2 for lag in lags],
            "torque_util_max": torque["max"],
            "torque_util_p99": torque["p99"],
            "torque_source": torque["source"],
            "torque_matched_ticks": torque["matched_ticks"],
            "qd_meas_max": qd_meas_max,
        },
        "reference": {
            **ref,
            "duration_commit_to_last_p50_s": duration_p50,
            "residual_fraction_theory": ds_residual_fraction(budget.omega, duration_p50),
            "residual_fraction_measured_p50": _q(col("e_last_mm") / col("e_commit_mm"), 50),
        },
        "clik": {
            "envelope_p95_rad_s2": envelope_p95,
            "envelope_max_rad_s2": envelope_max,
            "velocity_box_hit_frac": velocity_box_hit_frac,
            "ticks_over_planner_box_frac": over_box_frac,
        },
        "planner": {
            "reach_ok_frac_box": float(np.mean(reach_box)) if reach_box else math.nan,
            "reach_ok_frac_envelope": float(np.mean(reach_env)) if reach_env else math.nan,
            "t_reach_box_p50_s": _q(col("t_reach_box_s"), 50),
            "t_reach_envelope_p50_s": _q(col("t_reach_envelope_s"), 50),
            "lead_avail_p50_s": _q(col("lead_avail_s"), 50),
            **_rank_rates(ct._exists(ctl / "planner_events.csv")),
        },
        "gap_mm": {
            k: {"p50": _q(col(k), 50), "p90": _q(col(k), 90)}
            for k in (
                "e_commit_mm",
                "e_last_mm",
                "e_min_mm",
                "ref_vs_true_mm",
                "pred_live_lb_mm",
                "total_mm",
                "d_min_mm",
                "pred_mm",
                "clik_mm",
                "servo_mm",
            )
        },
        "success": int(sum(1 for r in rows if r.get("truth_success"))),
    }
    return {"summary": summary, "trials": rows}


# ── Envelope box file ────────────────────────────────────────────────────────
def envelope_box_document(
    units: Sequence[dict], group: str, *, percentile: float = ENVELOPE_PERCENTILE
) -> dict:
    """A ``derived_accel_limits`` document whose box is the pooled executed envelope.

    Pooled = the per-joint MAX over units of the per-unit p95 (a unit that
    moved harder sets the joint's line). ``provisional`` and meant for a sim
    overlay's ``robot.arm.accel_limits_path``; the provenance names every unit.
    """
    if not units:
        raise ValueError("no units")
    joints = list(units[0]["summary"]["budget"]["joints"])
    for u in units:
        got = list(u["summary"]["budget"]["joints"])
        if got != joints:
            raise SystemExit(
                f"{u['summary']['unit']}: arm joints {got} differ from {units[0]['summary']['unit']}'s "
                f"{joints} — an envelope box pools one arm, joint order included"
            )
    env = np.array([u["summary"]["clik"]["envelope_p95_rad_s2"] for u in units], dtype=float)
    box = [round(float(v), 4) for v in env.max(axis=0)]
    return {
        "derived_accel_limits": {
            group: {
                "qdd_max": box,
                "adopted": True,
                "provisional": True,
                "degenerate_reasons": [],
                "provenance": {
                    "tool": f"rtc_tools.analysis.{TOOL}",
                    "date": _dt.date.today().isoformat(),
                    "method": (
                        f"executed joint-acceleration envelope: per unit the p{percentile:g} of the "
                        f"{ACCEL_SMOOTH_TICKS}-tick box-averaged |q̈_cmd| over APPROACH/COMMITTED/CLOSING "
                        "ticks; per joint the max over units. Not a torque derivation — what the CLIK's "
                        "dynamic constraint actually commanded on this sim plant (S8-G, #537)."
                    ),
                    "joint_names": list(joints),
                    "units": [
                        {
                            "arm": u["summary"]["arm"],
                            "unit": Path(u["summary"]["unit"]).name,
                            "trials": u["summary"]["trials_committed"],
                            "envelope_p95": [
                                round(float(v), 3)
                                for v in u["summary"]["clik"]["envelope_p95_rad_s2"]
                            ],
                        }
                        for u in units
                    ],
                    "sim_only": True,
                },
            }
        }
    }


# ── CLI ──────────────────────────────────────────────────────────────────────
def parse_unit_arg(value: str) -> tuple[Path, Path]:
    """``<unit>[:<session>]`` — the session defaults to ``<unit>/session_copy``."""
    unit, _, session = value.partition(":")
    u = Path(unit)
    return u, Path(session) if session else u / "session_copy"


def _json_default(o):
    if isinstance(o, np.floating | np.integer):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(type(o))


def _fmt(v, nd=1) -> str:
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, int | float | np.floating):
        return "nan" if not math.isfinite(float(v)) else f"{float(v):.{nd}f}"
    return str(v)


def report(units: Sequence[dict]) -> str:
    lines = [f"{TOOL}: {len(units)} unit(s)"]
    for u in units:
        s = u["summary"]
        b, p, r, c, pl, g = (
            s["budget"],
            s["plant"],
            s["reference"],
            s["clik"],
            s["planner"],
            s["gap_mm"],
        )
        lines.append(
            f"\n[{s['arm']}] {s['unit']}  trials {s['trials_committed']} success {s['success']}"
        )
        lines.append(
            f"  budget  ω {_fmt(b['omega'])} a_max {_fmt(b['a_max'])} v_max {_fmt(b['v_max'], 2)} η_v {_fmt(b['eta_v'], 2)} box {[_fmt(v, 2) for v in b['qdd_box']]} ({b['source'].get('qdd_box')}); ω/a_max from {b['source'].get('omega')}"
        )
        lines.append(
            f"  P plant τ̂ {[_fmt(v, 4) for v in p['tau_hat_s']]} s · torque util max {[_fmt(v, 2) for v in p['torque_util_max']]} p99 {[_fmt(v, 2) for v in p['torque_util_p99']]} · q̇_meas max {[_fmt(v, 2) for v in p['qd_meas_max']]}"
        )
        lines.append(
            f"  R ref   sat {_fmt(100 * r['saturated_frac'])} % of active ticks · ‖u_des‖ p50 {_fmt(r['u_des_p50'])} max {_fmt(r['u_des_max'])} (a_max hit {_fmt(100 * r['a_max_hit_frac'])} %) · ‖ẋ‖ max {_fmt(r['xd_max'], 2)} (η_v v_max hit {_fmt(100 * r['v_max_hit_frac'])} %)"
        )
        lines.append(
            f"          residual e(t_c−T_arm)/e(commit): measured p50 {_fmt(r['residual_fraction_measured_p50'], 3)} · theory (1+ωT)e^−ωT at T {_fmt(r['duration_commit_to_last_p50_s'], 3)} s = {_fmt(r['residual_fraction_theory'], 3)}"
        )
        lines.append(
            f"  C clik  q̈ envelope p95 {[_fmt(v) for v in c['envelope_p95_rad_s2']]} max {[_fmt(v) for v in c['envelope_max_rad_s2']]} rad/s² · velocity box hit {_fmt(100 * c['velocity_box_hit_frac'])} % · ticks over planner box {_fmt(100 * c['ticks_over_planner_box_frac'])} %"
        )
        lines.append(
            f"  B plan  reach ok: box {_fmt(100 * pl['reach_ok_frac_box'])} % (t p50 {_fmt(pl['t_reach_box_p50_s'], 2)} s) vs envelope {_fmt(100 * pl['reach_ok_frac_envelope'])} % (t p50 {_fmt(pl['t_reach_envelope_p50_s'], 2)} s), lead avail p50 {_fmt(pl['lead_avail_p50_s'], 2)} s · valid plans {pl.get('valid_plans')} rank fail reach {_fmt(100 * pl.get('rank_reach', math.nan))} % gamma {_fmt(100 * pl.get('rank_gamma', math.nan))} % rollout {_fmt(100 * pl.get('rank_rollout', math.nan))} %"
        )
        lines.append(
            f"  gap mm  e(commit) {_fmt(g['e_commit_mm']['p50'])} → e(t_c−T_arm) {_fmt(g['e_last_mm']['p50'])} (p90 {_fmt(g['e_last_mm']['p90'])}) · ref_vs_true {_fmt(g['ref_vs_true_mm']['p50'])} · live pred ≥ {_fmt(g['pred_live_lb_mm']['p50'])} · hand–ball {_fmt(g['total_mm']['p50'])} · d_min {_fmt(g['d_min_mm']['p50'])} · CLIK {_fmt(g['clik_mm']['p50'])} · servo {_fmt(g['servo_mm']['p50'])}"
        )
    return "\n".join(lines)


def write_outputs(units: Sequence[dict], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "arm_budget_summary.json").write_text(
        json.dumps([u["summary"] for u in units], indent=1, default=_json_default)
    )
    rows = [{"arm": u["summary"]["arm"], **r} for u in units for r in u["trials"]]
    fields = list(dict.fromkeys(k for r in rows for k in r))
    import csv  # noqa: PLC0415

    with (out_dir / "arm_budget_trials.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, restval="")
        w.writeheader()
        w.writerows(rows)


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog=TOOL, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "units", nargs="+", help="<unit>[:<session>] — session defaults to <unit>/session_copy"
    )
    ap.add_argument(
        "--config-dir",
        type=Path,
        required=True,
        help="robot profile directory (integrated_bringup config/<robot>)",
    )
    ap.add_argument("--controller", help="catching controller name when the profile has several")
    ap.add_argument(
        "--overlay",
        type=Path,
        action="append",
        default=[],
        help="sim overlay YAML the units ran with (only when run_meta has no mirror of reference.*)",
    )
    ap.add_argument(
        "--time-margin-s",
        type=float,
        help="override planner.time.margin in the reach re-judgement (default: what the unit ran "
        "with — mirror, else the composed files)",
    )
    ap.add_argument("--n-boot", type=int, default=DEFAULT_N_BOOT)
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument(
        "--write-envelope-box",
        type=Path,
        help="write a derived_accel_limits YAML whose box is the pooled executed envelope",
    )
    ap.add_argument(
        "--group",
        help="device group name for --write-envelope-box (default: the profile's arm device)",
    )
    args = ap.parse_args(argv)

    units = []
    group = args.group
    for value in args.units:
        unit, session = parse_unit_arg(value)
        units.append(
            analyse_unit(
                unit,
                session,
                args.config_dir,
                controller=args.controller,
                overlays=args.overlay,
                time_margin_s=args.time_margin_s,
                n_boot=args.n_boot,
                seed=args.seed,
            )
        )
    if group is None:
        group = ct.load_profile(args.config_dir, args.controller).arm_device
    write_outputs(units, args.out)
    print(report(units))
    if args.write_envelope_box:
        doc = envelope_box_document(units, group)
        args.write_envelope_box.parent.mkdir(parents=True, exist_ok=True)
        args.write_envelope_box.write_text(
            "# Generated by rtc_tools.analysis.catching_arm_budget — the EXECUTED joint-acceleration\n"
            "# envelope of the units named in the provenance, not a torque derivation. Sim only,\n"
            "# provisional: a sim overlay's robot.arm.accel_limits_path may point here (S8-G, #537).\n"
            + yaml.safe_dump(doc, sort_keys=False, allow_unicode=True, width=100)
        )
        print(
            f"envelope box → {args.write_envelope_box}: {doc['derived_accel_limits'][group]['qdd_max']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
