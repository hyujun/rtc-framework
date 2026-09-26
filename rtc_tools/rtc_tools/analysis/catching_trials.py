#!/usr/bin/env python3
"""Catching sim trials: offline per-trial evaluation (dynamic_catching S8-A).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §4.4 S8 (the S8-A
measurement substrate), §1a (G8-D success definition, S0.9 power table), §5
(D-3 validity condition). One run of ``catching_sim_trials`` leaves three
things behind — the controller's session CSVs, the runner's trials directory
(``trial_results.json`` + one ground-truth CSV per trial) and, when the sim was
started with its lanes on, the clock and contact truth lanes. This module joins
them into one row per trial:

* **servo lag** — the arm's first-order lag τ̂ per joint, by least squares on
  ``q_cmd − q_meas ≈ τ q̇_meas`` over the moving ticks, with a trial-cluster
  bootstrap CI and R². Velocity cross-correlation does NOT give τ for a
  first-order lag (its peak sits well below τ), which is why it is not offered.
* **t_c decomposition** — at the plan's catch instant ``t_c`` (read on the first
  COMMITTED tick as ``t + plan_t_c_s``; the column is the time REMAINING):
  CLIK ``‖FK(q_cmd) − ref‖``, servo ``‖FK(q_meas(t_c)) − FK(q_cmd(t_c − T_lead))‖``,
  prediction ``‖p_c − p_true(t_c)‖`` and the total; the ball's arrival at the
  hand relative to t_c; the PLANNED γ_f (``plan_gamma_f`` — ``ref_gamma`` jumps
  to 1.0 on the DECEL entry tick and is not the plan); first-plan latency; plan
  switches during APPROACH. See :func:`decompose_at_tc` for why the command
  side is read ``T_lead`` earlier (the actuation-lag lead, diag ``t_arm_s``).
* **truth success** (G8-D, plan §1a) and the supervisor-vs-truth confusion
  matrix — see :func:`truth_success`.
* **D-3 covariate** (plan §5, D-S8-4 (c)) from the clock lane, reusing
  :mod:`rtc_tools.analysis.clock_phase`.
* **first hand–ball contact episode** from the contact lane (G7-B3 input).
* **``ref_saturated`` max streak** (G8-C3).
* **plan validity rate** (G3-D (i), #537 S8-D) — the fraction of ``planner_
  events.csv`` cycles between launch and commit whose ``plan_valid`` is true;
  see :func:`plan_validity_window` for the metric and :func:`_planner_cycle_times`
  for how a planner wake (a steady-clock instant) is mapped onto the
  controller's ``t_relative_s`` axis (needs ``--clock-lane``).
* **gate-map verdict** (S8-D, #537, optional ``--gate-map``) — whether a
  ``--dist`` box throw's nearest grid throw is open on the torque layer of a
  ``catch_gate_map`` output, and truth success over that open subset; see
  :func:`gate_map_verdict`.
* **validity** (S8-E, plan §4.4 D-S8-16 ①) — each trial's ``invalid_reason``
  (a rig failure, one of :data:`INVALID_REASONS` in precedence order, "" =
  valid; see :func:`record_invalid_reason` and :func:`lane_invalid_reason`).
  Every summary statistic is over the valid trials; the ITT bound counts the
  invalid ones as failures. :mod:`rtc_tools.analysis.catching_pool` pools
  these outputs across units and arms.
* **tick overrun** covariate from the RT loop's timing lane — see
  :func:`_cm_timing` for why it joins through the clock lane.
* **time alignment** (S8-E) — with a clock lane every trial is put on the
  lane's clocks (:class:`ClockLane`, :func:`estimate_sim_minus_t_rel`,
  :func:`align_trials_to_lane`) rather than on the runner's median wall
  offset, which drifts when the sim runs slower than the wall; plus the RTF
  covariates (:func:`trial_rtf`).
* **G8-B / G8-C2** (S8-E, optional ``--eval-samples`` / ``--probe-dump``) —
  :mod:`rtc_tools.analysis.catching_vision` on the ball's stamp axis;
  :func:`c2_for_trial` for the per-trial join.

and offers the statistics the S8 gates are judged with: Wilson intervals and
the S0.9 power / required-n computation, a whitened cross-covariance test with
a trial-cluster bootstrap (A⊥B, G8-C2) and an NEES summary (G8-B).

Frames. Ground truth is in the SIM WORLD; the controller's ``ref``, ``p_c`` and
FK are in the MODEL WORLD (the URDF root). They are composed exactly the way
the catching controller composes them — ``model_world_T_base`` read from the
URDF at ``catching.io.arm_base_frame`` (:func:`frame_placement_in_model_world`)
times ``base_T_world`` from ``catching.io.base_T_world`` — so a gap is never a
frame mismatch reported in metres.

Nothing robot-specific is baked in (ARCH-1): the catch frame is the profile's
``urdf.extra_frames`` entry, the arm joints are the diag's ``q_cmd_*`` columns,
the devices and log names are the catching controller's ``topics`` / ``logs``,
the hand bodies are the URDF subtree under the catch frame's parent link, and
the tick period is the runner's recorded mirror ``control.dt`` (or the median
spacing of ``t_relative_s``).
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis import catching_vision as cv, clock_phase, hand_close
from rtc_tools.analysis.catch_gate_map import REASON_NONE
from rtc_tools.analysis.catch_speed_budget import (
    DEFAULT_CATCH_FRAME,
    ExtraFrame,
    extra_frame_from_params,
)
from rtc_tools.analysis.catchability_map import (
    as_transform,
    frame_placement_in_model_world,
    invert_transform,
    make_transform,
    rotation_z,
    transform_direction,
    transform_point,
)

# ── rtc_msgs/CatchingState ABI (framework message constants, not robot values) ─
MODE_ARMED = 1
MODE_TRACKING = 2
MODE_APPROACH = 3
MODE_COMMITTED = 4
MODE_CLOSING = 5
MODE_DECEL = 6
MODE_HOLD = 7
MODE_RETREAT = 8
MODE_ABORT_SAFE = 9
HAND_PHASE_PRESHAPE = 1
HAND_PHASE_RELEASE = 4
MOVING_MODES = (MODE_APPROACH, MODE_COMMITTED, MODE_CLOSING, MODE_DECEL)

DIAG_LOG_TYPE = "CatchingDiagLog"
DEVICE_LOG_TYPE = "DeviceStateLog"
PLANNER_EVENTS_CSV = "planner_events.csv"
DEFAULT_ARRIVAL_WINDOW_S = 0.3
FREE_FIT_SPAN_S = 0.25
IMPACT_ACCEL_FACTOR = 2.0  # a contact = acceleration above this × the free-flight bound
DEFAULT_N_BOOT = 2000
DEFAULT_SEED = 0
DEFAULT_HOLD_WINDOW_S = 0.1  # hand-joint capture calibration window before t_hold_end
TRUTH_TIME_AXES = ("stamp", "recv")
# Rig-failure (invalid) reasons, in precedence order — plan §4.4 D-S8-16 ①: a
# trial gets the FIRST that matches, anything else that goes wrong (no plan,
# abort, HAND_TIMEOUT, cycle not closed, supervisor Missed) is a FAILURE.
INVALID_REASONS = ("srv_refused", "not_launched", "controller_silent", "lane_drop", "sim_stall")
SIM_STALL_STEP_FACTOR = 5.0  # sim_stall: a sim_time_sec gap above this × the nominal step
CM_TIMING_LOG = ("timing", "cm_timing_log.csv")  # the RT loop's per-tick timing lane
STREAK_BIN_WIDTH = 10  # G8-C3 ref_saturated streak histogram bin width [ticks]
B3_SPEARMAN_N_MIN = 50  # G7-B3: below this n the rank correlations are not evaluated
# The catchability_map / catch_gate_map throw-grid axes a --dist box trial's
# raw JSON record carries (S8-D, #537) — see :func:`trial_axis_values`. Every
# axis the map grid can vary must be here: an axis left out makes grid throws
# that differ only on it tie, and the tie goes to the lowest index.
GATE_MAP_AXES = (
    "distance_m",
    "azimuth_deg",
    "release_height_m",
    "aim_deviation_deg",
    "speed_m_s",
    "elevation_deg",
)


# ══ Statistics ════════════════════════════════════════════════════════════════


def wilson_interval(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for k successes in n trials.

    ``z`` is stated, never implied: 1.96 is the two-sided 95 % interval, whose
    lower bound is the 97.5 % one-sided bound G8-D uses (plan §4.4 S8).
    """
    if n <= 0:
        return (math.nan, math.nan)
    if not 0 <= k <= n:
        raise ValueError(f"k={k} outside [0, n={n}]")
    p = k / n
    z2 = z * z
    denom = 1.0 + z2 / n
    centre = (p + z2 / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def _wilson_lower_vec(k: np.ndarray, n: int, z: float) -> np.ndarray:
    p = k / n
    z2 = z * z
    denom = 1.0 + z2 / n
    centre = (p + z2 / (2 * n)) / denom
    half = z * np.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / denom
    return centre - half


def wilson_power(n: int, p: float, floor: float, z: float = 1.96) -> float:
    """P(Wilson lower bound of K/n ≥ floor) with K ~ Binomial(n, p), exactly."""
    from scipy.stats import binom  # noqa: PLC0415

    k = np.arange(n + 1)
    passing = np.nonzero(_wilson_lower_vec(k, n, z) >= floor)[0]
    if passing.size == 0:
        return 0.0
    # The lower bound is monotone in k, so passing is a tail k ≥ k_min.
    return float(binom.sf(passing[0] - 1, n, p))


def required_n(
    p: float, floor: float, power: float = 0.8, z: float = 1.96, n_max: int = 5000
) -> int | None:
    """S0.9 required n: smallest n whose power stays ≥ ``power`` for every n' ≤ n_max.

    Power is saw-toothed in n, so the first n that crosses the target can fall
    below it again; plan §1a takes the n after which it never does (up to
    ``n_max``). ``None`` when p ≤ floor or the target is never held.
    """
    if p <= floor:
        return None
    last_below = 0
    for n in range(1, n_max + 1):
        if wilson_power(n, p, floor, z) < power:
            last_below = n
    return None if last_below >= n_max else last_below + 1


def floor_verdict(
    k: int, n: int, floor: float, n_valid_target: int | None = None, z: float = 1.96
) -> str:
    """G8-D verdict: PASS when the Wilson lower bound of k/n is ≥ ``floor``.

    ``INSUFFICIENT_N(<n> < <target>)`` first when a target n is given and not
    reached (D-S8-3 n_valid 200) — the verdict is never taken on a short set.
    """
    if n_valid_target is not None and n < n_valid_target:
        return f"INSUFFICIENT_N({n} < {n_valid_target})"
    if n <= 0:
        return "INSUFFICIENT_N(0 valid)"
    return "PASS" if wilson_interval(k, n, z)[0] >= floor else "FAIL"


def mcnemar_exact(b: int, c: int) -> float:
    """Exact two-sided McNemar p-value: the discordant pairs under Binomial(b + c, ½)."""
    if b + c == 0:
        return 1.0
    from scipy.stats import binomtest  # noqa: PLC0415

    return float(binomtest(b, b + c, 0.5, alternative="two-sided").pvalue)


def ols_with_bootstrap(
    x: np.ndarray, y: np.ndarray, n_boot: int, seed: int
) -> tuple[float, float, list[float] | None, list[float] | None]:
    """OLS ``y = slope·x + intercept`` and trial-bootstrap 95 % percentile CIs.

    Each point is one trial, so resampling points IS the trial bootstrap. A
    resample whose x is constant has no slope and is skipped.
    """
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    if x.size < 3 or np.ptp(x) == 0:
        return math.nan, math.nan, None, None
    slope, intercept = (float(v) for v in np.polyfit(x, y, 1))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, x.size, size=(n_boot, x.size))
    xb, yb = x[idx], y[idx]
    xc = xb - xb.mean(axis=1, keepdims=True)
    sxx = (xc**2).sum(axis=1)
    ok = sxx > 0
    if not ok.any():
        return slope, intercept, None, None
    b_slope = (xc[ok] * (yb[ok] - yb[ok].mean(axis=1, keepdims=True))).sum(axis=1) / sxx[ok]
    b_icpt = yb[ok].mean(axis=1) - b_slope * xb[ok].mean(axis=1)
    ci = [float(v) for v in np.percentile(b_slope, [2.5, 97.5])]
    ci_i = [float(v) for v in np.percentile(b_icpt, [2.5, 97.5])]
    return slope, intercept, ci, ci_i


def spearman(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """Spearman ρ and its two-sided p-value (NaN when either side is constant or n < 3)."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    if x.size < 3 or np.ptp(x) == 0 or np.ptp(y) == 0:
        return math.nan, math.nan
    from scipy.stats import spearmanr  # noqa: PLC0415

    res = spearmanr(x, y)
    return float(res[0]), float(res[1])


def _whitener(samples: np.ndarray) -> np.ndarray:
    cov = np.cov(samples, rowvar=False)
    vals, vecs = np.linalg.eigh(cov)
    if np.any(vals <= 1e-15 * max(1.0, float(vals.max()))):
        raise ValueError("covariance is singular — cannot whiten")
    return vecs @ np.diag(vals**-0.5) @ vecs.T


def whitened_cross_covariance(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """3×3 cross-covariance of centred, individually whitened A and B.

    Under A⊥B every entry is 0; each is a correlation-like number in [-1, 1].
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if a.shape != b.shape or a.ndim != 2:
        raise ValueError("A and B must be (n, d) arrays of the same shape")
    ac = a - a.mean(axis=0)
    bc = b - b.mean(axis=0)
    aw = ac @ _whitener(ac)
    bw = bc @ _whitener(bc)
    return aw.T @ bw / (len(a) - 1)


@dataclass
class IndependenceResult:
    cross: list[list[float]]
    ci_low: list[list[float]]
    ci_high: list[list[float]]
    alpha: float
    n_samples: int
    n_clusters: int
    passed: bool  # True = no entry's CI excludes 0 (A⊥B not rejected)


def independence_test(
    a: np.ndarray,
    b: np.ndarray,
    clusters: Sequence | None = None,
    alpha: float = 0.05,
    n_boot: int = DEFAULT_N_BOOT,
    seed: int = DEFAULT_SEED,
) -> IndependenceResult:
    """A⊥B test (plan S8 sub-plan §6.5): whitened cross-covariance, cluster bootstrap.

    Samples of one trial are not independent of each other, so the bootstrap
    resamples whole CLUSTERS (trials). Each of the d² entries gets a percentile
    CI at the Bonferroni level ``alpha / d²``; A⊥B is rejected when any CI
    excludes 0.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    labels = np.arange(len(a)) if clusters is None else np.asarray(clusters)
    groups = [np.nonzero(labels == g)[0] for g in np.unique(labels)]
    rng = np.random.default_rng(seed)
    cross = whitened_cross_covariance(a, b)
    boots = []
    for _ in range(n_boot):
        pick = rng.integers(0, len(groups), len(groups))
        idx = np.concatenate([groups[i] for i in pick])
        try:
            boots.append(whitened_cross_covariance(a[idx], b[idx]))
        except ValueError:
            continue  # a degenerate resample (too few distinct clusters)
    boots = np.asarray(boots)
    level = alpha / cross.size
    lo = np.quantile(boots, level / 2, axis=0)
    hi = np.quantile(boots, 1 - level / 2, axis=0)
    passed = bool(np.all((lo <= 0.0) & (hi >= 0.0)))
    return IndependenceResult(
        cross.tolist(), lo.tolist(), hi.tolist(), alpha, len(a), len(groups), passed
    )


@dataclass
class NeesResult:
    dim: int
    n: int
    mean_raw: float
    mean_centred: float
    p_raw: float  # two-sided chi² p-value on the NEES sum
    p_centred: float
    coverage_raw: float  # fraction with NEES ≤ chi²_dim(level)
    coverage_centred: float
    coverage_level: float
    coverage_ci_raw: tuple[float, float]
    coverage_ci_centred: tuple[float, float]
    bias: list[float]
    passed_raw: bool
    passed_centred: bool


def _nees(errors: np.ndarray, covs: np.ndarray) -> np.ndarray:
    return np.einsum("ni,nij,nj->n", errors, np.linalg.inv(covs), errors)


def nees_summary(
    errors: np.ndarray, covs: np.ndarray, alpha: float = 0.05, coverage_level: float = 0.95
) -> NeesResult:
    """Raw and centred NEES, two-sided chi² test and coverage (S8 sub-plan §6.4).

    Two-sided on purpose: an estimator whose covariance is too LARGE (NEES far
    below dim — the expected state of a gravity-only filter at long horizons)
    passes a one-sided test vacuously. ``centred`` removes the mean error first,
    so a biased-but-honest covariance shows up as raw FAIL / centred PASS and
    the bias is reported next to it. The chi² test treats samples as independent;
    feed one value per trial (or per trial and horizon bin) to keep it honest.
    """
    from scipy.stats import chi2  # noqa: PLC0415

    e = np.asarray(errors, dtype=float)
    p = np.asarray(covs, dtype=float)
    n, dim = e.shape
    bias = e.mean(axis=0)
    raw = _nees(e, p)
    centred = _nees(e - bias, p)
    bound = chi2.ppf(coverage_level, dim)

    def two_sided(values: np.ndarray, dof: int) -> float:
        cdf = chi2.cdf(float(values.sum()), dof)
        return float(min(1.0, 2 * min(cdf, 1 - cdf)))

    p_raw = two_sided(raw, n * dim)
    # Centring spends dim degrees of freedom on the estimated mean.
    p_centred = two_sided(centred, (n - 1) * dim)
    k_raw = int((raw <= bound).sum())
    k_cen = int((centred <= bound).sum())
    ci_raw = wilson_interval(k_raw, n)
    ci_cen = wilson_interval(k_cen, n)

    def ok(pv: float, ci: tuple[float, float]) -> bool:
        return pv >= alpha and ci[0] <= coverage_level <= ci[1]

    return NeesResult(
        dim,
        n,
        float(raw.mean()),
        float(centred.mean()),
        p_raw,
        p_centred,
        k_raw / n,
        k_cen / n,
        coverage_level,
        ci_raw,
        ci_cen,
        bias.tolist(),
        ok(p_raw, ci_raw),
        ok(p_centred, ci_cen),
    )


# ══ Servo lag ═════════════════════════════════════════════════════════════════


@dataclass
class ServoLag:
    joint: str
    tau_s: float
    ci_low_s: float
    ci_high_s: float
    r2: float
    n_ticks: int


def servo_lag_ls(
    t: np.ndarray,
    q_cmd: np.ndarray,
    q_meas: np.ndarray,
    moving: np.ndarray,
    clusters: np.ndarray,
    joint_names: Sequence[str],
    n_boot: int = DEFAULT_N_BOOT,
    seed: int = DEFAULT_SEED,
) -> list[ServoLag]:
    """First-order lag per joint: ``q_cmd − q_meas = τ · q̇_meas`` by least squares.

    A first-order servo ``q̇ = (u − q)/τ`` makes the tracking error proportional
    to the measured velocity with slope τ. ``q̇_meas`` is the time derivative of
    ``q_meas`` taken inside each cluster (trial) so a derivative never spans two
    trials; only ``moving`` ticks enter the fit. The CI is a percentile bootstrap
    over clusters (ticks within a trial are strongly autocorrelated, so a tick
    bootstrap would be far too narrow). R² is the centred coefficient of
    determination of the no-intercept model.
    """
    t = np.asarray(t, dtype=float)
    q_cmd = np.asarray(q_cmd, dtype=float)
    q_meas = np.asarray(q_meas, dtype=float)
    moving = np.asarray(moving, dtype=bool)
    clusters = np.asarray(clusters)
    qd = np.full_like(q_meas, np.nan)
    for g in np.unique(clusters):
        idx = np.nonzero(clusters == g)[0]
        if len(idx) >= 3:
            qd[idx] = np.gradient(q_meas[idx], t[idx], axis=0)
    use = moving & np.all(np.isfinite(qd), axis=1) & np.all(np.isfinite(q_cmd), axis=1)
    err = q_cmd - q_meas
    labels = clusters[use]
    groups = [np.nonzero(labels == g)[0] for g in np.unique(labels)]
    rng = np.random.default_rng(seed)
    picks = [rng.integers(0, len(groups), len(groups)) for _ in range(n_boot)]
    out = []
    for j, name in enumerate(joint_names):
        e = err[use, j]
        v = qd[use, j]
        vv = float(np.dot(v, v))
        tau = float(np.dot(e, v) / vv) if vv > 0 else math.nan
        resid = e - tau * v
        sst = float(np.sum((e - e.mean()) ** 2))
        r2 = 1.0 - float(np.dot(resid, resid)) / sst if sst > 0 else math.nan
        sev = np.array([np.dot(e[g], v[g]) for g in groups])
        svv = np.array([np.dot(v[g], v[g]) for g in groups])
        boots = [sev[p].sum() / svv[p].sum() for p in picks if svv[p].sum() > 0]
        lo, hi = np.quantile(boots, [0.025, 0.975]) if boots else (math.nan, math.nan)
        out.append(ServoLag(name, tau, float(lo), float(hi), r2, int(use.sum())))
    return out


# ══ Configuration ═════════════════════════════════════════════════════════════


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text()) or {}


def _ros_params(doc: Mapping) -> dict:
    merged: dict = {}
    for node in doc.values():
        if isinstance(node, Mapping) and isinstance(node.get("ros__parameters"), Mapping):
            merged.update(node["ros__parameters"])
    return merged


@dataclass
class CatchingProfile:
    """What this tool reads from a robot profile directory (never from code)."""

    controller: str
    devices: list[str]  # catching controller topic groups; [0] is the arm
    diag_log: str
    device_logs: dict[str, str]
    arm_base_frame: str
    base_t_world: np.ndarray
    catch_frame: ExtraFrame
    catch_frame_name: str
    robot_params: dict
    ball_diameter_m: float | None
    ball_mass_kg: float | None
    # ``catching.robot.hand`` verbatim (q_pre/q_close/caging_mask/capture/…) —
    # kept as a raw mapping rather than parsed fields because this tool only
    # ever reads a handful of keys from it (see :func:`hand_caging_profile`)
    # and the controller's own YAML schema is the SSoT for the rest.
    hand_yaml: dict = field(default_factory=dict)

    @property
    def arm_device(self) -> str:
        return self.devices[0]

    @property
    def hand_device(self) -> str | None:
        return self.devices[1] if len(self.devices) > 1 else None


def _catching_controllers(config_dir: Path) -> dict[str, dict]:
    found = {}
    for path in sorted((config_dir / "controllers").glob("*.yaml")):
        doc = _load_yaml(path)
        for name, node in doc.items():
            if not isinstance(node, Mapping) or not isinstance(node.get("catching"), Mapping):
                continue
            logs = node.get("logs") or []
            if any(str(e.get("msg_type", "")).endswith(DIAG_LOG_TYPE) for e in logs):
                found[str(name)] = dict(node)
    return found


# Where a profile keeps its robot config (``urdf``, ``urdf.extra_frames``): the
# shared ``_base.yaml`` when the profile has one, else ``sim.yaml`` — a
# sim-only profile has no hardware twin to share a base with.
ROBOT_CONFIG_FILES = ("_base.yaml", "sim.yaml")


def robot_config_params(config_dir: Path) -> dict:
    """The ``/**`` parameters of the first of :data:`ROBOT_CONFIG_FILES` that has ``urdf``."""
    config_dir = Path(config_dir)
    for name in ROBOT_CONFIG_FILES:
        path = config_dir / name
        if path.is_file():
            params = _ros_params(_load_yaml(path))
            if "urdf" in params:
                return params
    raise SystemExit(
        f"{config_dir}: none of {', '.join(ROBOT_CONFIG_FILES)} carries a `urdf` block "
        "(urdf.extra_frames lives there)"
    )


def load_profile(
    config_dir: Path,
    controller: str | None = None,
    session: Path | None = None,
    catch_frame: str = DEFAULT_CATCH_FRAME,
) -> CatchingProfile:
    """Read the catching controller's config and the profile base from ``config_dir``.

    The controller is the one entry under ``controllers/*.yaml`` that carries a
    ``catching`` tree and logs a ``CatchingDiagLog`` (and, given a session,
    exists under ``<session>/controllers/``); pass ``controller`` to choose.
    """
    config_dir = Path(config_dir)
    candidates = _catching_controllers(config_dir)
    if controller is not None:
        if controller not in candidates:
            raise SystemExit(f"{config_dir}: no catching controller named '{controller}'")
        name = controller
    else:
        names = sorted(candidates)
        if session is not None:
            names = [n for n in names if (Path(session) / "controllers" / n).is_dir()]
        if len(names) != 1:
            raise SystemExit(
                f"{config_dir}: {len(names)} catching controller(s) match {names} — pass "
                "--controller"
            )
        name = names[0]
    node = candidates[name]
    devices = [str(g) for g in (node.get("topics") or {})]
    if not devices:
        raise SystemExit(f"{name}: `topics` is empty — the arm device is its first group")
    logs = node.get("logs") or []
    diag = [str(e["instance"]) for e in logs if str(e["msg_type"]).endswith(DIAG_LOG_TYPE)]
    dev_logs = [str(e["instance"]) for e in logs if str(e["msg_type"]).endswith(DEVICE_LOG_TYPE)]
    device_logs = {}
    for i, group in enumerate(devices):
        match = [inst for inst in dev_logs if inst.startswith(group)]
        if match:
            device_logs[group] = match[0]
        elif i < len(dev_logs):
            device_logs[group] = dev_logs[i]
    io = node["catching"].get("io") or {}
    try:
        base_frame = str(io["arm_base_frame"])
        btw = io["base_T_world"]
        base_t_world = make_transform(
            rotation_z(math.radians(float(btw["yaw_deg"]))), [float(x) for x in btw["translation"]]
        )
    except KeyError as exc:
        raise SystemExit(
            f"{name}: catching.io lacks {exc.args[0]} — the sim-world → model-world "
            "composition cannot be read, and guessing it flips the frame silently"
        ) from exc
    params = robot_config_params(config_dir)
    frame = extra_frame_from_params(params, catch_frame)
    ball = (node["catching"].get("core") or {}).get("ball") or {}
    diameter = ball.get("diameter")
    mass = None
    sim_yaml = config_dir / "mujoco_simulator.yaml"
    if sim_yaml.is_file():
        mass = (_ros_params(_load_yaml(sim_yaml)).get("projectile_ball") or {}).get("mass_kg")
    hand_yaml = (node["catching"].get("robot") or {}).get("hand") or {}
    return CatchingProfile(
        controller=name,
        devices=devices,
        diag_log=diag[0] if diag else "",
        device_logs=device_logs,
        arm_base_frame=base_frame,
        base_t_world=base_t_world,
        catch_frame=frame,
        catch_frame_name=catch_frame,
        robot_params=params,
        ball_diameter_m=None if diameter is None else float(diameter),
        ball_mass_kg=None if mass is None else float(mass),
        hand_yaml=dict(hand_yaml) if isinstance(hand_yaml, Mapping) else {},
    )


# ══ Gate map (S8-D throw-box verdict, #537) ═══════════════════════════════════
# A ``catch_gate_map`` output judges the ``catchability_map`` throw GRID the
# S8-D box was drawn from (rtc_tools.analysis.catch_gate_map's module
# docstring): each grid throw is OPEN on the torque layer when at least one of
# its accepted candidates (at the map's one wait-pose ``seed_id`` —
# ``catch_gate_map.py``'s ``accepted = [r for r in accepted if
# int(r["seed_id"]) == seed_id]`` already narrows ``gate_map.csv`` to it)
# has ``reason_torque == "none"``. A --dist box trial is drawn continuously
# inside the grid's box (``catching_sim_trials.frozen_throws``), so it is
# never exactly one grid throw — this module hands it the NEAREST one, in
# axis space normalised by each axis's own grid step, so an axis with a
# coarser grid does not dominate the distance.


@dataclass
class GateMap:
    """What one ``catch_gate_map`` output + its ``catchability_map`` grid says
    about the S8-D throw box, read once per run (:func:`load_gate_map`)."""

    map_dir: Path  # the catchability_map dir the gate map was judged from
    seed_id: int
    throw_axes: dict[int, dict[str, float]]  # throw_index -> {axis: value}
    open_throw_indices: frozenset[int]
    axis_steps: dict[str, float]  # axis -> grid step; an axis with one value is absent


def load_gate_map_summary(gate_map_dir: Path) -> tuple[Path, int]:
    """``(map_dir, seed_id)`` from a ``catch_gate_map`` ``gate_map_summary.yaml``."""
    doc = _load_yaml(Path(gate_map_dir) / "gate_map_summary.yaml")
    try:
        return Path(doc["map_dir"]), int(doc["seed_id"])
    except KeyError as exc:
        raise SystemExit(f"{gate_map_dir}: gate_map_summary.yaml lacks {exc.args[0]}") from exc


def load_throw_grid_axes(throw_summary_csv: Path) -> dict[int, dict[str, float]]:
    """``throw_index -> {axis: value}`` from a ``catchability_map`` ``throw_summary.csv`` grid."""
    out: dict[int, dict[str, float]] = {}
    with Path(throw_summary_csv).open(newline="") as handle:
        for row in csv.DictReader(handle):
            out[int(row["throw_index"])] = {axis: float(row[axis]) for axis in GATE_MAP_AXES}
    return out


def load_open_throw_indices(gate_map_csv: Path, seed_id: int) -> frozenset[int]:
    """Grid ``throw_index`` values with ≥ 1 candidate open on the torque layer at ``seed_id``."""
    out: set[int] = set()
    with Path(gate_map_csv).open(newline="") as handle:
        for row in csv.DictReader(handle):
            if int(row["seed_id"]) == seed_id and row["reason_torque"] == REASON_NONE:
                out.add(int(row["throw_index"]))
    return frozenset(out)


def grid_axis_steps(
    throw_axes: Mapping[int, Mapping[str, float]], axes: Sequence[str] = GATE_MAP_AXES
) -> dict[str, float]:
    """Per-axis grid step: the median spacing of its unique grid values.

    An axis whose grid holds only one value is OMITTED, not given a step of 0
    — a 0 step would make every trial infinitely far from the grid on that
    axis (division by 0), when in fact the axis carries no information to
    normalise by at all.
    """
    steps: dict[str, float] = {}
    for axis in axes:
        values = sorted({row[axis] for row in throw_axes.values()})
        if len(values) < 2:
            continue
        steps[axis] = float(np.median(np.diff(values)))
    return steps


def nearest_grid_throw(
    trial_axes: Mapping[str, float],
    throw_axes: Mapping[int, Mapping[str, float]],
    axis_steps: Mapping[str, float],
) -> tuple[int, float]:
    """The grid throw nearest ``trial_axes``, Euclidean in axis space normalised by
    ``axis_steps`` (an axis absent from ``axis_steps`` — a single-valued grid,
    :func:`grid_axis_steps` — does not enter the distance). Ties go to the
    LOWEST ``throw_index``: throws are visited in ascending index order and a
    later one must beat, not just match, the best distance so far to replace it.
    """
    best_idx, best_d = -1, math.inf
    for idx in sorted(throw_axes):
        axes = throw_axes[idx]
        d = math.sqrt(
            sum(((trial_axes[axis] - axes[axis]) / step) ** 2 for axis, step in axis_steps.items())
        )
        if d < best_d:
            best_idx, best_d = idx, d
    return best_idx, best_d


def load_gate_map(gate_map_dir: Path) -> GateMap:
    """Read a ``catch_gate_map`` output dir into a :class:`GateMap`."""
    gate_map_dir = Path(gate_map_dir)
    map_dir, seed_id = load_gate_map_summary(gate_map_dir)
    throw_axes = load_throw_grid_axes(map_dir / "throw_summary.csv")
    return GateMap(
        map_dir=map_dir,
        seed_id=seed_id,
        throw_axes=throw_axes,
        open_throw_indices=load_open_throw_indices(gate_map_dir / "gate_map.csv", seed_id),
        axis_steps=grid_axis_steps(throw_axes),
    )


def trial_axis_values(record: Mapping) -> dict[str, float] | None:
    """The gate-map axis values a raw ``trial_results.json`` record carries.

    Only a ``--dist`` box throw (``catching_sim_trials.frozen_throws``) writes
    all of ``GATE_MAP_AXES`` onto its record; the ``reference``/``varied``
    series (``trial_throws``) writes none of them — ``None``, not a guess, so
    such a trial gets no gate-map verdict (:func:`gate_map_verdict`).
    """
    try:
        return {axis: float(record[axis]) for axis in GATE_MAP_AXES}
    except (KeyError, TypeError, ValueError):
        return None


def gate_map_verdict(trial_axes: Mapping[str, float] | None, gate_map: GateMap) -> dict:
    """``map_open`` / ``map_throw_index`` / ``map_distance`` for one trial.

    All ``None`` when ``trial_axes`` is ``None`` (:func:`trial_axis_values`) —
    a reference-series trial was never drawn from the gate map's grid, so
    "nearest grid throw" has no meaning for it.
    """
    if trial_axes is None:
        idx, dist = None, None
    else:
        idx, dist = nearest_grid_throw(trial_axes, gate_map.throw_axes, gate_map.axis_steps)
    return {
        "map_open": None if idx is None else idx in gate_map.open_throw_indices,
        "map_throw_index": idx,
        "map_distance": dist,
    }


def urdf_subtree(urdf_text: str, root_link: str) -> list[str]:
    """Links at and below ``root_link`` in the URDF tree (the hand, under the catch frame parent)."""
    root = ET.fromstring(urdf_text)
    children: dict[str, list[str]] = {}
    for joint in root.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(
            joint.find("child").get("link")
        )
    out, stack = [], [root_link]
    while stack:
        link = stack.pop()
        out.append(link)
        stack.extend(children.get(link, []))
    return out


def urdf_robot_links(urdf_text: str) -> list[str]:
    """Every URDF link except the tree root(s).

    The root is the anchor the robot is fixed to, and it may share its name
    with the simulator's own world body (a URDF ``world`` link, a MuJoCo
    ``world`` body): counting it would call every floor bounce a robot contact.
    """
    root = ET.fromstring(urdf_text)
    children = {j.find("child").get("link") for j in root.findall("joint")}
    return [link.get("name") for link in root.findall("link") if link.get("name") in children]


class CatchFrameFk:
    """FK of the catch frame in the model world, arm joints addressed by name."""

    def __init__(self, urdf_text: str, joint_names: Sequence[str], profile: CatchingProfile):
        from rtc_tools.analysis.catch_speed_budget import ArmKinematics  # noqa: PLC0415

        self._arm = ArmKinematics(
            urdf_text,
            list(joint_names),
            profile.catch_frame,
            np.zeros(len(joint_names)),  # kinematics only: no rotor inertia is used
            profile.catch_frame_name,
        )
        model_world_t_base = frame_placement_in_model_world(urdf_text, profile.arm_base_frame)
        self.model_t_world = as_transform(model_world_t_base @ profile.base_t_world)
        self.world_t_model = invert_transform(self.model_t_world)

    def __call__(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        if q.ndim == 1:
            return self._arm.frame_position(q)
        return np.array([self._arm.frame_position(row) for row in q])

    def to_model(self, p_world: np.ndarray) -> np.ndarray:
        return transform_point(p_world, self.model_t_world)

    def dir_to_world(self, v_model: np.ndarray) -> np.ndarray:
        return transform_direction(v_model, self.world_t_model)


# ══ Session loading ═══════════════════════════════════════════════════════════


def _read_csv(path: Path, usecols=None):
    import pandas as pd  # noqa: PLC0415

    for candidate in (path, path.with_name(path.name + ".gz")):
        if candidate.is_file():
            return pd.read_csv(candidate, usecols=usecols, low_memory=False)
    raise SystemExit(f"missing {path} (or {path.name}.gz)")


def _exists(path: Path) -> Path | None:
    for candidate in (path, path.with_name(path.name + ".gz")):
        if candidate.is_file():
            return candidate
    return None


DIAG_COLUMNS = (
    "t_relative_s",
    "mode",
    "plan_id",
    "plan_t_c_s",
    "plan_p_c_x",
    "plan_p_c_y",
    "plan_p_c_z",
    "plan_gamma_f",
    "ref_valid",
    "ref_saturated",
    "ref_x_x",
    "ref_x_y",
    "ref_x_z",
    "hand_phase",
)
# The actuation-lag lead the RT tick ran with (L5 §4.5, ``t_arm_s``; 0 when
# ``joint_cmd.lag.lead_enable`` is off). Optional so a diag recorded before the
# column existed still reads — see :func:`lead_per_tick` for the fallback.
DIAG_LEAD_COLUMN = "t_arm_s"
# The hand-joint capture witness (#537 S8-C, D-S8-8 (b)), right after
# ``hand_timeout`` in the diag schema. Optional like ``DIAG_LEAD_COLUMN`` so a
# diag recorded before the columns existed still reads — see
# :func:`hand_witness_at_verdict` for the NaN fallback.
HAND_WITNESS_COLUMNS = ("hand_stalled_n", "hand_effort_frac", "hand_blocked_s", "outcome_source")
# The vision snapshot the tick read (L8 §5.2) — G8-C2's exact join key into
# the probe dump. Optional: a diag recorded before the columns has none.
DIAG_INPUT_COLUMNS = ("input_snapshot_sequence", "input_generation")


def diag_columns(header: Sequence[str]) -> list[str]:
    """The diag columns this tool reads: fixed schema fields + every arm joint pair."""
    missing = [c for c in DIAG_COLUMNS if c not in header]
    if missing:
        raise SystemExit(f"catching diag lacks column(s) {missing}")
    joints = [c for c in header if c.startswith(("q_cmd_", "q_meas_"))]
    lead = [DIAG_LEAD_COLUMN] if DIAG_LEAD_COLUMN in header else []
    hand_witness = [c for c in HAND_WITNESS_COLUMNS if c in header]
    inputs = [c for c in DIAG_INPUT_COLUMNS if c in header]
    return list(DIAG_COLUMNS) + lead + hand_witness + inputs + joints


def arm_joints_from_diag(columns: Sequence[str]) -> list[str]:
    cmd = [c[len("q_cmd_") :] for c in columns if c.startswith("q_cmd_")]
    meas = [c[len("q_meas_") :] for c in columns if c.startswith("q_meas_")]
    if not cmd or cmd != meas:
        raise SystemExit(f"diag q_cmd_*/q_meas_* joint sets differ or are empty: {cmd} / {meas}")
    return cmd


@dataclass
class Trial:
    idx: int
    kind: str
    outcome: str | None
    accepted: bool
    t_launch: float  # on the controller's t_relative_s axis
    t_end: float
    offset: float  # wall − t_relative_s (the runner's median; drifts when RTF < 1)
    truth_path: Path | None
    mode_log: list = field(default_factory=list)
    # The runner's launch instant, time.time() right after the launch srv
    # returned (CLOCK_REALTIME). Recorded for every trial, even one without
    # an offset.
    launch_wall: float = math.nan
    # stamp − t_relative_s for the ball's launch-anchored stamp axis (truth,
    # predictions, eval samples). The runner's wall offset until
    # :func:`align_trials_to_lane` replaces it with the lane's.
    stamp_offset: float = math.nan
    # How t_launch / t_end / stamp_offset were obtained: "median_offset" (the
    # runner's drifting wall offset) or "lane" (:func:`align_trials_to_lane`).
    alignment: str = "median_offset"
    stamp_anchor: str = ""  # "truth" / "lane_estimate" (lane alignment only)

    def wall_launch(self) -> float:
        """The launch on the runner's wall clock, as this trial's own fields imply.

        ``t_launch + offset`` when both are known (for a loaded record that is
        exactly ``launch_wall_time``, and a caller that shifts ``t_launch``
        shifts the launch with it), else the raw ``launch_wall``.
        """
        if np.isfinite(self.t_launch) and np.isfinite(self.offset):
            return self.t_launch + self.offset
        return self.launch_wall


def load_trials(trials_dir: Path) -> tuple[list[Trial], dict]:
    """Read ``trial_results.json``: a list of records, or ``{"trials": [...], ...}``."""
    trials_dir = Path(trials_dir)
    doc = json.loads((trials_dir / "trial_results.json").read_text())
    records = doc if isinstance(doc, list) else doc.get("trials", [])
    meta = {} if isinstance(doc, list) else {k: v for k, v in doc.items() if k != "trials"}
    # The runner's sidecar (catching_sim_trials): args and the controller's
    # read-only mirror (`controller_mirror`, incl. `control.dt`) for the run.
    run_meta = trials_dir / "run_meta.json"
    if run_meta.is_file():
        meta.update(json.loads(run_meta.read_text()))
    out = []
    for r in records:
        off = r.get("wall_t_relative_offset")
        accepted = bool(r.get("accepted"))
        launch_wall = r.get("launch_wall_time")
        launch_wall = math.nan if launch_wall is None else float(launch_wall)
        if off is None or not accepted:
            out.append(
                Trial(
                    r["idx"],
                    r.get("kind", ""),
                    r.get("outcome"),
                    accepted,
                    *[math.nan] * 3,
                    _truth_path(trials_dir, r.get("truth_csv"), launch_wall, r["idx"])
                    if accepted
                    else None,
                    list(r.get("mode_log") or []),
                    launch_wall=launch_wall,
                )
            )
            continue
        t_launch = float(r["launch_wall_time"]) - off
        ends = [float(m[0]) - off for m in r.get("mode_log") or []]
        t_end = max(ends) if ends else t_launch
        out.append(
            Trial(
                int(r["idx"]),
                str(r.get("kind", "")),
                r.get("outcome"),
                accepted,
                t_launch,
                t_end,
                float(off),
                _truth_path(trials_dir, r.get("truth_csv"), launch_wall, r["idx"]),
                list(r.get("mode_log") or []),
                launch_wall=launch_wall,
                stamp_offset=float(off),
            )
        )
    return out, {"records": records, "meta": meta}


TRUTH_LAUNCH_TOL_S = 5.0  # a trial's first truth stamp sits within ms of its launch


def _truth_path(
    trials_dir: Path, recorded: str | None, launch_wall: float = math.nan, idx=None
) -> Path | None:
    """The trial's truth CSV — next to ``trial_results.json`` first.

    The runner records an ABSOLUTE path. A trials dir that was renamed after
    its run and whose old path was reused (a unit re-run into the same
    ``--out-dir``) points at the OTHER run's file, with the same throw and a
    plausible shape; so the local copy wins, and a file whose first stamp is
    not within :data:`TRUTH_LAUNCH_TOL_S` of this trial's ``launch_wall_time``
    is refused as another run's (seen in the S8-E smoke: 1606 s apart).
    """
    if not recorded:
        return None
    path = Path(recorded)
    for candidate in (trials_dir / path.name, trials_dir / path, path):
        found = _exists(candidate)
        if found is None:
            continue
        first = _first_truth_row(found)
        gap = first[0] - launch_wall if first is not None else math.nan
        if np.isfinite(gap) and abs(gap) > TRUTH_LAUNCH_TOL_S:
            print(
                f"catching_trials: trial {idx}: {found} starts {gap:+.1f} s from the "
                "trial's launch — another run's truth, ignored",
                file=sys.stderr,
            )
            continue
        return found
    return None


def recorded_dt(doc: Mapping, t: np.ndarray) -> tuple[float, str]:
    """The tick period: the runner's recorded mirror ``control.dt``, else the CSV spacing."""

    def find(node) -> float | None:
        if isinstance(node, Mapping):
            if "control.dt" in node:
                return float(node["control.dt"])
            if isinstance(node.get("control"), Mapping) and "dt" in node["control"]:
                return float(node["control"]["dt"])
            for key in ("controller_mirror", "mirror", "profile", "params"):
                if key in node:
                    value = find(node[key])
                    if value is not None:
                        return value
        return None

    for node in [doc.get("meta", {})] + list(doc.get("records", [])):
        value = find(node)
        if value is not None:
            return value, "trial_results.json control.dt"
    return float(np.median(np.diff(t))), "median diff of t_relative_s"


def _mirror_lead(meta: Mapping) -> float | None:
    """The lead the runner's mirror says the controller was configured with."""
    mirror = meta.get("controller_mirror")
    if not isinstance(mirror, Mapping) or "joint_cmd.lag.lead_enable" not in mirror:
        return None
    if not mirror["joint_cmd.lag.lead_enable"]:
        return 0.0
    # The controller leads by 0 when T_arm is TBD (the mirror declares NaN,
    # JSON may carry it as null) — the lead switch alone does not move the axis.
    t_arm = mirror.get("joint_cmd.lag.T_arm")
    t_arm = math.nan if t_arm is None else float(t_arm)
    return t_arm if math.isfinite(t_arm) else 0.0


def lead_per_tick(diag, meta: Mapping) -> tuple[np.ndarray, str]:
    """``T_lead`` per diag row [s] and where it came from.

    The diag column is what the RT tick actually used, so it wins. Without it
    (a diag older than the column) the runner's mirror decides, and with
    neither the lead is 0 — the only configuration such a session can have
    run, since lead compensation shipped after both. A column that disagrees
    with the mirror means the trials directory belongs to another session, and
    every lead-aware number would be read on the wrong axis — refused.
    """
    mirror = _mirror_lead(meta)
    if DIAG_LEAD_COLUMN in diag.columns:
        lead = diag[DIAG_LEAD_COLUMN].to_numpy(float)
        if mirror is not None and lead.size and not np.allclose(lead, mirror, atol=1e-6):
            raise SystemExit(
                f"diag {DIAG_LEAD_COLUMN} {np.unique(lead)} disagrees with the runner's "
                f"mirror lead {mirror} — trials directory from another session?"
            )
        return lead, f"diag {DIAG_LEAD_COLUMN}"
    n = len(diag)
    if mirror is not None:
        return np.full(n, mirror), "runner mirror joint_cmd.lag (no diag column)"
    return np.zeros(n), "0 (no diag column, no runner mirror)"


@dataclass
class Truth:
    t: np.ndarray  # controller t_relative_s axis
    p_world: np.ndarray
    v_world: np.ndarray

    def at(self, times) -> tuple[np.ndarray, np.ndarray]:
        """Linearly interpolated position and velocity (NaN outside the record).

        Linear interpolation on the 10 ms truth grid errs by at most g·dt²/8
        (0.12 mm) on a ballistic path; nearest-sample lookup would quantise the
        time by ±dt/2 (±15 mm at 3 m/s).
        """
        times = np.atleast_1d(np.asarray(times, dtype=float))
        p = np.column_stack(
            [np.interp(times, self.t, self.p_world[:, a], np.nan, np.nan) for a in range(3)]
        )
        v = np.column_stack(
            [np.interp(times, self.t, self.v_world[:, a], np.nan, np.nan) for a in range(3)]
        )
        return p, v

    def first_impact(self, t_from: float, accel_limit: float) -> float:
        """First sample time after ``t_from`` whose |Δv|/Δt exceeds ``accel_limit``.

        A free ball's acceleration is bounded by gravity plus drag (the D-3
        ``a_bound``); a contact is not. ``+inf`` when the record shows none.
        """
        dt = np.diff(self.t)
        accel = np.linalg.norm(np.diff(self.v_world, axis=0), axis=1) / np.where(
            dt > 0, dt, np.inf
        )
        hit = np.nonzero((self.t[1:] > t_from) & (accel > accel_limit))[0]
        return float(self.t[hit[0] + 1]) if hit.size else math.inf

    def free_flight(
        self, times, t_cut: float, fit_span_s: float = FREE_FIT_SPAN_S
    ) -> tuple[np.ndarray, np.ndarray, float, float]:
        """The ball's FREE-FLIGHT position/velocity at ``times``, ignoring anything after ``t_cut``.

        Before the last pre-cut sample this is the interpolated record. After it
        — the ball has hit something — each axis is extrapolated with a quadratic
        fitted to the last ``fit_span_s`` of pre-cut samples (constant
        acceleration: gravity plus a drag that barely changes over the span).
        Returns (p, v, fit rms [m], time of the last pre-cut sample).
        """
        times = np.atleast_1d(np.asarray(times, dtype=float))
        pre = self.t < t_cut
        if not pre.any():
            nan = np.full((len(times), 3), np.nan)
            return nan, nan, math.nan, math.nan
        t_last = float(self.t[pre][-1])
        p, v = self.at(np.minimum(times, t_last))
        late = times > t_last
        fit = pre & (self.t >= t_last - fit_span_s)
        rms = math.nan
        if late.any():
            if fit.sum() < 5:
                p[late], v[late] = np.nan, np.nan
                return p, v, rms, t_last
            tf = self.t[fit] - t_last
            coef = [np.polyfit(tf, self.p_world[fit, a], 2) for a in range(3)]
            resid = np.column_stack(
                [np.polyval(coef[a], tf) - self.p_world[fit, a] for a in range(3)]
            )
            rms = float(np.sqrt(np.mean(resid**2)))
            dt = times[late] - t_last
            p[late] = np.column_stack([np.polyval(coef[a], dt) for a in range(3)])
            v[late] = np.column_stack([np.polyval(np.polyder(coef[a]), dt) for a in range(3)])
        return p, v, rms, t_last


def load_truth(path: Path, offset: float, axis: str = "stamp", wall_to_t=None) -> Truth | None:
    """A runner truth CSV on the controller's time axis.

    ``stamp`` (default) uses the message stamp — launch-anchored sim time, a
    uniform 10 ms grid — minus ``offset`` (stamp − t_relative_s: the lane's
    per-trial stamp offset, or the runner's ``wall_t_relative_offset`` without
    a lane). ``recv`` uses the receive wall time (the runner's callback,
    jittered by its spin loop), mapped by ``wall_to_t`` when given (the lane)
    and by ``offset`` otherwise.
    """
    df = _read_csv(path)
    if df.empty:
        return None
    col = "stamp_s" if axis == "stamp" else "wall_recv_s"
    df = df.sort_values(col)
    raw = df[col].to_numpy(float)
    return Truth(
        wall_to_t(raw) if (axis != "stamp" and wall_to_t is not None) else raw - offset,
        df[["x", "y", "z"]].to_numpy(float),
        df[["vx", "vy", "vz"]].to_numpy(float),
    )


# ══ Lanes ═════════════════════════════════════════════════════════════════════


@dataclass
class ClockLane:
    """The clock lane joined to the controller's t_relative_s axis.

    **Two alignments.** Under sim-sync (the S8 sim profiles) the controller's
    ``t_relative_s`` is ``iteration × dt`` — one tick per physics step — so it
    IS the sim-time axis up to one session constant ``c_s = sim − t_relative_s``
    (:func:`estimate_sim_minus_t_rel`). With ``c_s`` known the lane maps every
    clock exactly: sim ↔ steady by interpolating the lane rows, wall ↔ steady
    by ``wall_offset_s`` (CLOCK_REALTIME − CLOCK_MONOTONIC, one constant per
    boot). Without it (``c_s is None``) the old path remains: steady −
    ``steady_offset`` (the median at the launches), which is exact only at
    RTF 1 — when the sim runs slower than the wall, steady and sim drift apart
    over a trial.
    """

    trials: list  # clock_phase.Trial, in launch order
    segments: dict[int, np.ndarray]  # launch_seq → (n, 2) [sim_s, steady_s] of active rows
    steady_offset: float  # steady_s − t_relative_s (median over launches; fallback path)
    offset_spread_s: float
    dropped_total: int
    trial_to_seq: dict[int, int]
    # trial idx → that trial's own steady_s − t_relative_s at launch. Under
    # sim-sync the offset drifts when RTF < 1, so a per-trial window uses its
    # own offset, not the session median (see :func:`_planner_cycle_times`).
    trial_offsets: dict[int, float] = field(default_factory=dict)
    # Rig checks (D-S8-16 ① lane_drop / sim_stall) read EVERY row of a launch,
    # not only the in-flight ones: launch_seq → (n, 3) [sim_s, steady_s,
    # dropped_total] in lane order, and the dropped_total of the row just
    # before that launch's first row (a drop that swallowed the launch rows
    # themselves shows only against it).
    seq_rows: dict[int, np.ndarray] = field(default_factory=dict)
    dropped_before: dict[int, float] = field(default_factory=dict)
    # Median sim_time_sec spacing over the whole lane — the physics step.
    nominal_step_s: float = math.nan
    # CLOCK_REALTIME − CLOCK_MONOTONIC [s]: from the launch pairing (the
    # runner's launch_wall_time vs the lane's first in-flight row, a few ms
    # of srv/step latency) unless refined (``wall_offset_source``).
    wall_offset_s: float = math.nan
    wall_offset_source: str = "launch_pairing"
    # sim_time_sec − t_relative_s, set by the caller once estimated; None =
    # the fallback (median-offset) path.
    c_s: float | None = None
    # Every lane row, in lane order, for the steady → sim map.
    all_sim: np.ndarray = field(default_factory=lambda: np.zeros(0))
    all_steady: np.ndarray = field(default_factory=lambda: np.zeros(0))

    @property
    def aligned(self) -> bool:
        return self.c_s is not None

    def launch_sim(self, seq: int) -> float:
        """Sim time of the launch: one step before the flight's first row.

        The sim loop handles a launch at the top of an iteration and skips the
        step (``mujoco_sim_loop.cpp``), so the ball leaves at the sim time of the
        row BEFORE the first ``ball_active`` row — the first published truth
        sample carries exactly that instant.
        """
        return float(self.segments[seq][0, 0]) - self.nominal_step_s

    def seq_sim_to_steady(self, seq: int, sim_s):
        """Sim → steady over the rows of one launch, slope 1 beyond them."""
        rows = self.seq_rows.get(seq)
        if rows is None or len(rows) < 2:
            rows = self.segments[seq]
        return _interp_slope1(sim_s, rows[:, 0], rows[:, 1])

    def steady_to_sim(self, steady_s):
        """Steady → sim over every lane row (steady is monotonic), slope 1 beyond."""
        return _interp_slope1(steady_s, self.all_steady, self.all_sim)

    def steady_to_t_rel(self, steady_s):
        return self.steady_to_sim(steady_s) - self.c_s

    def wall_to_t_rel(self, wall_s):
        return self.steady_to_t_rel(np.asarray(wall_s, float) - self.wall_offset_s)

    def t_rel_to_steady(self, seq: int, t_rel):
        return self.seq_sim_to_steady(seq, np.asarray(t_rel, float) + self.c_s)

    def rig_check(self, seq: int, steady_end_s: float = math.inf) -> tuple[float, float]:
        """(dropped_total increase, largest sim_time_sec gap [s]) over one trial's rows.

        The rows are those carrying ``seq`` up to ``steady_end_s`` (the trial's
        end on the steady clock). ``launch_seq`` stays at a launch's number
        until the NEXT launch, so without the cut the rows would run on
        through the next throw's homing — a rig failure there never touched
        this trial. The first row is always kept.
        """
        rows = self.seq_rows.get(seq)
        if rows is None or len(rows) == 0:
            return math.nan, math.nan
        keep = rows[:, 1] <= steady_end_s
        keep[0] = True
        rows = rows[keep]
        dropped = float(rows[-1, 2] - self.dropped_before.get(seq, rows[0, 2]))
        gap = float(np.max(np.diff(rows[:, 0]))) if len(rows) >= 2 else 0.0
        return dropped, gap

    def to_t_relative(self, seq: int, sim_s: float) -> float:
        if self.aligned:
            return float(sim_s) - self.c_s
        seg = self.segments[seq]
        return float(np.interp(sim_s, seg[:, 0], seg[:, 1])) - self.steady_offset

    def delta_at(self, seq: int, t_rel: float) -> float:
        """δ(t) of plan §5 (clock_phase's definition) at controller time ``t_rel``."""
        seg = self.segments[seq]
        if self.aligned:
            sim = t_rel + self.c_s
            if sim < seg[0, 0] or sim > seg[-1, 0]:
                return math.nan
            steady = float(np.interp(sim, seg[:, 0], seg[:, 1]))
        else:
            steady = t_rel + self.steady_offset
            if steady < seg[0, 1] or steady > seg[-1, 1]:
                return math.nan
            sim = float(np.interp(steady, seg[:, 1], seg[:, 0]))
        return (steady - seg[0, 1]) - (sim - seg[0, 0])


def _interp_slope1(x, xp: np.ndarray, fp: np.ndarray):
    """``np.interp`` inside [xp[0], xp[-1]], and slope-1 extrapolation outside.

    The clocks this maps between all run at ~1 s/s, so beyond the rows the
    nearest row plus the elapsed time is the best guess (np.interp would
    clamp to a constant).
    """
    x = np.asarray(x, float)
    y = np.interp(x, xp, fp)
    y = np.where(x < xp[0], fp[0] + (x - xp[0]), y)
    y = np.where(x > xp[-1], fp[-1] + (x - xp[-1]), y)
    return float(y) if y.ndim == 0 else y


def load_clock_lane(
    path: Path, trials: Sequence[Trial], interval_tol_s: float = 0.05
) -> ClockLane:
    """Read the lane, segment it with clock_phase, and pair launches with trials.

    A session can hold more launches than one trials directory — two runner
    invocations in one sim session, or a throw from the GUI — so launches are
    NOT paired by order. The runner's launch instant is ``time.time()``
    (CLOCK_REALTIME, :meth:`Trial.wall_launch`) and the lane's is its first
    in-flight row's ``steady_ns`` (CLOCK_MONOTONIC); the two differ by one
    constant per boot, so the offset that pairs the most trials (each within
    ``interval_tol_s``) wins — it becomes ``wall_offset_s``. Lane launches it
    leaves unpaired belong to another run and are ignored, and a trial it
    leaves unpaired gets no clock covariate (``trial_to_seq`` has no entry).
    A lane that pairs fewer than half the trials is refused as not this run.
    """
    import pandas as pd  # noqa: PLC0415

    path = Path(path)
    if path.suffix == ".gz":
        # clock_phase.read_lane opens a plain file; hand it a decompressed copy.
        import gzip  # noqa: PLC0415
        import shutil  # noqa: PLC0415
        import tempfile  # noqa: PLC0415

        with tempfile.TemporaryDirectory() as tmp:
            plain = Path(tmp) / path.stem
            with gzip.open(path, "rb") as src, plain.open("wb") as dst:
                shutil.copyfileobj(src, dst)
            return load_clock_lane(plain, trials, interval_tol_s)
    stats = clock_phase.read_lane(path)
    df = pd.read_csv(path).reset_index(drop=True)
    active = df[df["ball_active"].astype(str).isin(("1", "true", "True"))]
    sim_all = df["sim_time_sec"].to_numpy(float)
    dropped_all = (
        df["dropped_total"].to_numpy(float) if "dropped_total" in df.columns else np.zeros(len(df))
    )
    steady_all = df["steady_ns"].to_numpy(float) * 1e-9
    seq_all = df["launch_seq"].to_numpy(int)
    seq_rows, dropped_before = {}, {}
    for seq in np.unique(seq_all[seq_all > 0]):
        pos = np.nonzero(seq_all == seq)[0]
        seq_rows[int(seq)] = np.column_stack([sim_all[pos], steady_all[pos], dropped_all[pos]])
        dropped_before[int(seq)] = float(dropped_all[pos[0] - 1] if pos[0] > 0 else dropped_all[0])
    nominal = float(np.median(np.diff(sim_all))) if len(sim_all) >= 2 else math.nan
    segments = {
        int(seq): np.column_stack(
            [g["sim_time_sec"].to_numpy(float), g["steady_ns"].to_numpy(float) * 1e-9]
        )
        for seq, g in active.groupby("launch_seq")
        if int(seq) > 0 and len(g) >= 2
    }
    lane_all = [t for t in stats.trials if t.launch_seq in segments]
    # Paired on the runner's WALL launch instant, not on t_relative_s: the
    # runner's t_relative_s offset is a median over the trial and drifts by
    # (1 − RTF) × the trial's length under load, while wall and steady differ
    # by one constant per boot (CLOCK_REALTIME − CLOCK_MONOTONIC).
    runs = [t for t in trials if t.accepted and np.isfinite(t.wall_launch())]
    if not runs or not lane_all:
        raise SystemExit(f"{path}: {len(lane_all)} launches in the lane, {len(runs)} trials")
    steady0 = np.array([segments[t.launch_seq][0, 1] for t in lane_all])
    t0 = np.array([r.wall_launch() for r in runs])
    pairs = _pair_by_offset(steady0, t0 - t0[0], interval_tol_s)
    if 2 * len(pairs) < len(runs):
        raise SystemExit(
            f"{path}: only {len(pairs)} of {len(runs)} trials pair with a launch under one "
            f"clock offset (tol {interval_tol_s * 1e3:.0f} ms) — the lane is not this run"
        )
    walls = np.array([t0[j] - steady0[i] for j, i in pairs])
    wall_offset = float(np.median(walls))
    # The fallback path's steady − t_relative_s, from the trials that have one.
    rel = [(j, i) for j, i in pairs if np.isfinite(runs[j].t_launch)]
    rel_offsets = np.array([steady0[i] - runs[j].t_launch for j, i in rel])
    return ClockLane(
        trials=[lane_all[i] for _, i in pairs],
        segments=segments,
        steady_offset=float(np.median(rel_offsets)) if rel else math.nan,
        offset_spread_s=float(np.max(np.abs(walls - wall_offset))),
        dropped_total=stats.dropped_total,
        trial_to_seq={runs[j].idx: lane_all[i].launch_seq for j, i in pairs},
        trial_offsets={runs[j].idx: float(steady0[i] - runs[j].t_launch) for j, i in rel},
        seq_rows=seq_rows,
        dropped_before=dropped_before,
        nominal_step_s=nominal,
        wall_offset_s=wall_offset,
        all_sim=sim_all,
        all_steady=steady_all,
    )


def _pair_by_offset(steady0: np.ndarray, t0: np.ndarray, tol_s: float) -> list[tuple[int, int]]:
    """``(trial j, launch i)`` pairs under the one offset that pairs the most.

    Every (launch, trial) pair proposes an offset; for each, a trial pairs with
    the nearest launch within ``tol_s`` (each launch used once). Ties go to the
    smaller spread. O(L·T·T) — trial counts are hundreds, not millions.
    """
    best: list[tuple[int, int]] = []
    best_spread = math.inf
    for i0 in range(len(steady0)):
        for j0 in range(len(t0)):
            offset = steady0[i0] - t0[j0]
            used: set[int] = set()
            pairs = []
            for j, t in enumerate(t0):
                err = np.abs(steady0 - (t + offset))
                for i in np.argsort(err):
                    if err[i] > tol_s:
                        break
                    if int(i) not in used:
                        used.add(int(i))
                        pairs.append((j, int(i)))
                        break
            if len(pairs) < len(best):
                continue
            res = np.array([steady0[i] - t0[j] for j, i in pairs])
            spread = float(np.ptp(res)) if len(res) else math.inf
            if len(pairs) > len(best) or spread < best_spread:
                best, best_spread = pairs, spread
    return best


# The sim profiles' max_rtf (1.0): ball stamps advance 1 s per sim second, so
# stamp − t_relative_s is one constant per flight. A profile with another
# max_rtf would need the stamp axis scaled — not supported, not in use.
STAMP_RTF = 1.0
TRUTH_ANCHOR_TOL_M = 1e-6  # the first truth row IS the launch state when it equals the record


@dataclass
class PlanBridge:
    """``planner_events.csv`` rows by ``plan_id`` — the planner's own steady instants.

    ``wake_ns + lead_s`` is the plan's catch instant ``t_c`` on the steady
    clock (``lead_s`` = t_c − wake, ``planner_search.cpp``), and the diag's
    ``plan_t_c_s`` at a tick is ``t_c − that tick's steady now``, so the two
    give the steady instant of any tick that carries the plan.
    """

    wake_s: dict[int, float]
    lead_s: dict[int, float]
    snapshot: dict[int, tuple[int, int]]  # plan_id → (snapshot_sequence, track_generation)

    def t_c_steady(self, plan_id: int) -> float:
        if plan_id not in self.wake_s:
            return math.nan
        return self.wake_s[plan_id] + self.lead_s[plan_id]


def load_plan_bridge(ctl: Path) -> PlanBridge | None:
    path = _exists(ctl / PLANNER_EVENTS_CSV)
    if path is None:
        return None
    df = _read_csv(path)
    need = {"wake_ns", "publish_ns", "plan_id", "lead_s"}
    if not need <= set(df.columns):
        return None
    df = df[(df["publish_ns"] > 0) & (df["plan_id"] > 0)].drop_duplicates("plan_id")
    snap_cols = {"snapshot_sequence", "track_generation"} <= set(df.columns)
    return PlanBridge(
        wake_s={int(p): w * 1e-9 for p, w in zip(df["plan_id"], df["wake_ns"], strict=True)},
        lead_s={int(p): float(v) for p, v in zip(df["plan_id"], df["lead_s"], strict=True)},
        snapshot={
            int(p): (int(q), int(g))
            for p, q, g in zip(
                df["plan_id"], df["snapshot_sequence"], df["track_generation"], strict=True
            )
        }
        if snap_cols
        else {},
    )


def commit_ticks(t: np.ndarray, mode: np.ndarray) -> np.ndarray:
    """Indices of the first COMMITTED tick of every commit."""
    committed = mode == MODE_COMMITTED
    prev = np.concatenate([[False], committed[:-1]])
    return np.nonzero(committed & ~prev)[0]


def estimate_sim_minus_t_rel(
    t: np.ndarray,
    mode: np.ndarray,
    plan_id: np.ndarray,
    plan_t_c: np.ndarray,
    bridge: PlanBridge | None,
    lane: ClockLane,
) -> tuple[float | None, dict]:
    """The session constant ``c = sim_time_sec − t_relative_s`` (sim-sync), and how.

    At a commit tick the diag's ``plan_t_c_s`` and the planner's ``t_c`` on the
    steady clock (:class:`PlanBridge`) give that tick's steady instant. The RT
    tick runs while the sim waits for its command, after the lane row of the
    state it was handed and before the next step, so the last lane row at or
    before that instant carries the tick's sim time: ``c = sim(row) − t``.
    Checked on the S8-E smoke units of both robots (one ran at RTF 0.62) and
    the pilot fixture: within a session every commit gave the same ``c`` (a
    whole number of ticks — how many steps the sim took before the CM's first
    tick, so it differs between sessions), matching the CM timing lane's
    ``tick_count`` ↔ lane ``step`` offset. ``None`` when no commit carries a
    planner row.
    """
    info: dict = {"source": "plan_bridge", "n": 0}
    if bridge is None:
        info["source"] = "none (no planner_events.csv with lead_s)"
        return None, info
    values = []
    for k in commit_ticks(t, mode):
        steady = bridge.t_c_steady(int(plan_id[k])) - float(plan_t_c[k])
        if not np.isfinite(steady):
            continue
        j = int(np.searchsorted(lane.all_steady, steady, side="right")) - 1
        if j < 0:
            continue
        values.append(float(lane.all_sim[j]) - float(t[k]))
    if not values:
        info["source"] = "none (no commit with a planner row on the lane)"
        return None, info
    c = float(np.median(values))
    info.update(n=len(values), spread_s=float(np.ptp(values)), value_s=c)
    if info["spread_s"] > 1.5 * lane.nominal_step_s:
        print(
            f"catching_trials: sim − t_relative_s varies over the session by "
            f"{info['spread_s'] * 1e3:.1f} ms (a sim reset?); using the median {c:.4f} s",
            file=sys.stderr,
        )
    return c, info


def _first_truth_row(path: Path | None):
    if path is None:
        return None
    import pandas as pd  # noqa: PLC0415

    try:
        df = pd.read_csv(path, nrows=1)
    except (OSError, ValueError):
        return None
    if df.empty or "stamp_s" not in df.columns:
        return None
    return float(df["stamp_s"].iloc[0]), df[["x", "y", "z"]].to_numpy(float)[0]


def align_trials_to_lane(
    trials: Sequence[Trial], records_by_idx: Mapping[int, Mapping], lane: ClockLane
) -> list[Trial]:
    """Put every lane-paired trial on the lane's clocks (``lane.c_s`` must be set).

    * ``t_launch`` = the launch's sim time (:meth:`ClockLane.launch_sim`) − c;
    * ``t_end`` = the last ``mode_log`` wall receipt → steady (− the wall
      offset) → sim (lane rows) − c;
    * ``stamp_offset`` = the flight's stamp anchor − ``t_launch``: stamps are
      ``anchor + (sim − launch sim) / max_rtf`` (:data:`STAMP_RTF`). The anchor
      is the first truth row's stamp when that row is the launch state
      (equal to the record's ``pos``); otherwise the lane's steady instant of
      the launch plus the wall offset (``stamp_anchor = "lane_estimate"``, a
      few ms off).

    None of it reads the runner's ``wall_t_relative_offset``, which drifts by
    (1 − RTF) × the trial's length. Unpaired trials are returned unchanged.
    """
    out = []
    for trial in trials:
        seq = lane.trial_to_seq.get(trial.idx)
        if seq is None or not trial.accepted:
            out.append(trial)
            continue
        launch_sim = lane.launch_sim(seq)
        t_launch = launch_sim - lane.c_s
        walls = [float(m[0]) for m in trial.mode_log]
        t_end = max(float(np.max(lane.wall_to_t_rel(walls))), t_launch) if walls else t_launch
        anchor, how = math.nan, "lane_estimate"
        first = _first_truth_row(trial.truth_path)
        pos = records_by_idx.get(trial.idx, {}).get("pos")
        if (
            first is not None
            and pos is not None
            and np.max(np.abs(first[1] - np.asarray(pos, float))) <= TRUTH_ANCHOR_TOL_M
        ):
            anchor, how = first[0], "truth"
        if not np.isfinite(anchor):
            anchor = lane.wall_offset_s + float(lane.seq_sim_to_steady(seq, launch_sim))
        if trial.idx in lane.trial_offsets:
            lane.trial_offsets[trial.idx] = float(lane.segments[seq][0, 1]) - t_launch
        out.append(
            dataclasses.replace(
                trial,
                t_launch=t_launch,
                t_end=t_end,
                stamp_offset=anchor - t_launch,
                alignment="lane",
                stamp_anchor=how,
            )
        )
    return out


RTF_FLIGHT_S = 0.6  # rtf_flight: launch to first contact, at most this much sim time
RTF_WINDOW_S = 0.25  # rtf_trial_min: sim-time windows


def trial_rtf(lane: ClockLane, seq: int, sim_contact: float, sim_end: float) -> dict:
    """Wall-clock speed of the sim over one trial (a covariate, not a rule).

    ``rtf_flight`` = sim span / steady span from the launch to the first
    contact (or :data:`RTF_FLIGHT_S` of sim time); ``rtf_trial_min`` = the
    slowest :data:`RTF_WINDOW_S` window from the launch to the trial's end.
    """
    out = {"rtf_flight": math.nan, "rtf_trial_min": math.nan}
    rows = lane.seq_rows.get(seq)
    if rows is None or len(rows) < 2:
        rows = lane.segments.get(seq)
    if rows is None or len(rows) < 2:
        return out
    sim, steady = rows[:, 0], rows[:, 1]
    s0 = float(sim[0])
    hi = s0 + RTF_FLIGHT_S
    if np.isfinite(sim_contact):
        hi = min(hi, float(sim_contact))
    hi = min(hi, float(sim[-1]))
    if hi > s0:
        out["rtf_flight"] = (hi - s0) / (float(np.interp(hi, sim, steady)) - float(steady[0]))
    end = min(float(sim_end), float(sim[-1])) if np.isfinite(sim_end) else float(sim[-1])
    sel = sim <= end
    bins = np.floor((sim[sel] - s0) / RTF_WINDOW_S).astype(int)
    rtfs = []
    for b in np.unique(bins):
        k = np.nonzero(bins == b)[0]
        ds = sim[sel][k[-1]] - sim[sel][k[0]]
        dw = steady[sel][k[-1]] - steady[sel][k[0]]
        if len(k) >= 2 and ds >= RTF_WINDOW_S / 2 and dw > 0:
            rtfs.append(ds / dw)
    if rtfs:
        out["rtf_trial_min"] = float(min(rtfs))
    return out


# ══ Per-trial analysis ════════════════════════════════════════════════════════


def max_streak(flags: np.ndarray) -> int:
    best = run = 0
    for f in np.asarray(flags, dtype=bool):
        run = run + 1 if f else 0
        best = max(best, run)
    return best


def _first(mask: np.ndarray) -> int | None:
    hit = np.nonzero(mask)[0]
    return int(hit[0]) if hit.size else None


def truth_success(
    t: np.ndarray,
    mode: np.ndarray,
    hand_phase: np.ndarray,
    truth: Truth | None,
    fk_at_tick,
    hold_radius_m: float,
) -> dict:
    """G8-D truth success (plan §1a): the ball is in the hand from HOLD end to release.

    Operational definition (documented, not tuned):

    * window = [first RETREAT tick (HOLD end), first RETREAT tick whose hand
      phase is RELEASE (the release at the wait pose)]. A trial that never
      reaches both ends is NOT a success (no hold, or no release to hold until).
    * "in the hand" at a truth sample = the ball centre within ``hold_radius_m``
      of the catch frame FK(q_meas) at that instant. The catch frame origin IS
      where a held ball's centre sits (profile ``urdf.extra_frames``), so the
      radius is a tolerance on "moved away from the pocket"; the CLI uses one
      ball diameter from the profile.
    * success = every truth sample in the window is in the hand.

    The distances at both ends and the worst one are returned so a borderline
    case (e.g. a ball lying in the open hand) can be read rather than trusted.
    """
    out = {
        "t_hold_end": math.nan,
        "t_release": math.nan,
        "d_hold_end_mm": math.nan,
        "d_release_mm": math.nan,
        "d_max_mm": math.nan,
        "truth_success": False,
        "truth_reason": "",
    }
    k_ret = _first(mode == MODE_RETREAT)
    if k_ret is None:
        out["truth_reason"] = "no RETREAT"
        return out
    k_rel = _first((mode == MODE_RETREAT) & (hand_phase == HAND_PHASE_RELEASE))
    if k_rel is None:
        out["truth_reason"] = "no release in RETREAT"
        return out
    out["t_hold_end"], out["t_release"] = float(t[k_ret]), float(t[k_rel])
    if truth is None:
        out["truth_reason"] = "no truth"
        return out
    sel = (truth.t >= t[k_ret]) & (truth.t <= t[k_rel])
    times = np.concatenate([[t[k_ret]], truth.t[sel], [t[k_rel]]])
    p_ball, _ = truth.at(times)
    if not np.all(np.isfinite(p_ball)):
        out["truth_reason"] = "truth does not cover the window"
        return out
    ticks = np.clip(np.searchsorted(t, times), 0, len(t) - 1)
    d = np.linalg.norm(p_ball - fk_at_tick(ticks), axis=1)
    out["d_hold_end_mm"], out["d_release_mm"] = float(d[0] * 1e3), float(d[-1] * 1e3)
    out["d_max_mm"] = float(d.max() * 1e3)
    out["truth_success"] = bool(d.max() <= hold_radius_m)
    out["truth_reason"] = "held" if out["truth_success"] else "ball left the hand"
    return out


def _hold_end_tick(mode: np.ndarray) -> int | None:
    """The tick HOLD ends — the index of the first RETREAT row — but ONLY when
    the row immediately before it is HOLD. RETREAT reached any other way (e.g.
    from ABORT_SAFE, no HOLD in between) never ran ``JudgeOutcome`` or the
    hand-capture witness for an attempt: it is "no verdict tick", the same as
    a trial that never reaches RETREAT at all, not a tick whose witness
    happens to belong to whatever the previous mode was doing.
    """
    k_ret = _first(mode == MODE_RETREAT)
    if k_ret is None or k_ret == 0 or mode[k_ret - 1] != MODE_HOLD:
        return None
    return k_ret


def hand_witness_at_verdict(
    mode: np.ndarray,
    hand_stalled_n: np.ndarray | None,
    hand_effort_frac: np.ndarray | None,
    hand_blocked_s: np.ndarray | None,
    outcome_source_tick: np.ndarray | None,
    supervisor: str | None,
) -> dict:
    """The hand-joint witness the HOLD-end verdict rested on (#537 S8-C, D-S8-8 (b)).

    Two DIFFERENT rows, not one: ``JudgeOutcome`` runs (and decides
    ``outcome_source``) before the mode changes and before this tick's hand
    stage runs, so:

    * ``outcome_source`` first becomes the verdict's value on the diag row the
      CSV already shows as mode RETREAT — the FIRST RETREAT tick, because
      ``AdvanceMode`` (which flips ``mode_``) runs before ``PublishTickRecord``
      on the very tick HOLD ends.
    * ``hand_blocked_s``/``hand_stalled_n``/``hand_effort_frac`` for THAT
      judgement are the ones the hand stage filled on the tick BEFORE it — the
      LAST HOLD row — because the current tick's ``RunHandStage`` (which would
      overwrite them) has not run yet when ``JudgeOutcome`` reads
      ``hand_blocked_since_ns_``.

    Reading the wrong row for either quietly swaps in the next attempt's
    witness or the previous attempt's judgement (see the test suite's two
    mutation checks). NaN (``hand_*_judge``) / empty (``tips_only_verdict``)
    when the trial never reaches RETREAT, the diag predates these columns, or
    RETREAT was entered from anywhere but HOLD (:func:`_hold_end_tick`) — an
    aborted attempt was never judged and ran no witness for one.

    ``hand_blocked_s_judge`` IS THEREFORE ONE CONTROL PERIOD SHORT of what the
    verdict itself compared against ``t_persist``: the controller's own
    ``JudgeOutcome`` reads ``hand_blocked_s`` on the tick it runs (the first
    RETREAT tick, one tick AFTER the last HOLD tick this reads), so the run
    length it actually compared against ``t_persist`` is
    ``hand_blocked_s_judge + dt`` (``dt`` = the tick period, e.g.
    :func:`recorded_dt`'s result), not ``hand_blocked_s_judge`` alone. An
    offline re-check of the persist gate must use
    ``hand_blocked_s_judge + dt >= t_persist`` — see :func:`hand_persist_met`.
    """
    out: dict = {
        "outcome_source": math.nan,
        "hand_blocked_s_judge": math.nan,
        "hand_stalled_n_judge": math.nan,
        "hand_effort_frac_judge": math.nan,
        "tips_only_verdict": "",
    }
    k_ret = _hold_end_tick(mode)
    if k_ret is None:
        return out
    if outcome_source_tick is not None:
        out["outcome_source"] = float(outcome_source_tick[k_ret])
    k_hold = k_ret - 1
    if hand_blocked_s is not None:
        out["hand_blocked_s_judge"] = float(hand_blocked_s[k_hold])
    if hand_stalled_n is not None:
        out["hand_stalled_n_judge"] = float(hand_stalled_n[k_hold])
    if hand_effort_frac is not None:
        out["hand_effort_frac_judge"] = float(hand_effort_frac[k_hold])
    src = out["outcome_source"]
    if np.isfinite(src):
        out["tips_only_verdict"] = (
            "MISSED" if supervisor == "CAPTURED" and int(src) == 2 else supervisor
        )
    return out


def hand_persist_met(hand_blocked_s_judge: float, dt: float, t_persist_s: float) -> float:
    """Whether the verdict's OWN ``t_persist`` comparison (run on the first
    RETREAT tick, one tick after ``hand_blocked_s_judge`` was read — see
    :func:`hand_witness_at_verdict`) would have read True, reconstructed
    offline: ``hand_blocked_s_judge + dt >= t_persist_s``. Returns ``1.0``/
    ``0.0`` (not ``bool``, so it stays NaN-able like every other ``_judge``
    column) and NaN when any input is unavailable — most commonly
    ``t_persist_s`` itself, which this module never requires a profile to
    carry (``catching.robot.hand.capture.t_persist`` — see
    :func:`hand_capture_t_persist_s`).
    """
    if not (np.isfinite(hand_blocked_s_judge) and np.isfinite(dt) and np.isfinite(t_persist_s)):
        return math.nan
    return float(hand_blocked_s_judge + dt >= t_persist_s)


def hand_release_to_preshape_ms(t: np.ndarray, mode: np.ndarray, hand_phase: np.ndarray) -> float:
    """Time [ms] from the release tick (``truth_success``'s ``t_release``: the
    first RETREAT tick whose hand phase is RELEASE) to the first LATER tick
    whose hand phase is PRESHAPE — the hand back at q_pre. NaN if the trial
    never releases, or releases but never reaches PRESHAPE in the window.
    """
    k_rel = _first((mode == MODE_RETREAT) & (hand_phase == HAND_PHASE_RELEASE))
    if k_rel is None:
        return math.nan
    later = _first(hand_phase[k_rel + 1 :] == HAND_PHASE_PRESHAPE)
    if later is None:
        return math.nan
    k_pre = k_rel + 1 + later
    return float((t[k_pre] - t[k_rel]) * 1e3)


@dataclass
class TrialContext:
    """Arrays of one trial's diag window, with FK cached per tick."""

    t: np.ndarray
    mode: np.ndarray
    hand_phase: np.ndarray
    plan_id: np.ndarray
    plan_t_c: np.ndarray
    plan_p_c: np.ndarray
    plan_gamma_f: np.ndarray
    ref: np.ndarray
    ref_valid: np.ndarray
    ref_saturated: np.ndarray
    q_cmd: np.ndarray
    q_meas: np.ndarray
    fk: CatchFrameFk
    # Actuation-lag lead per tick [s] (``lead_per_tick``): the command and the
    # reference written at t are aimed at t + t_lead.
    t_lead: np.ndarray | None = None
    # The hand-joint capture witness per tick (#537 S8-C, D-S8-8 (b); optional
    # — ``None`` on a diag recorded before the columns existed). See
    # :func:`hand_witness_at_verdict` for how they are read.
    hand_stalled_n: np.ndarray | None = None
    hand_effort_frac: np.ndarray | None = None
    hand_blocked_s: np.ndarray | None = None
    outcome_source_tick: np.ndarray | None = None
    # The vision snapshot key per tick (DIAG_INPUT_COLUMNS), None when absent.
    input_seq: np.ndarray | None = None
    input_gen: np.ndarray | None = None
    _cache: dict = field(default_factory=dict)

    def fk_meas(self, ticks) -> np.ndarray:
        return self._fk("meas", self.q_meas, ticks)

    def fk_cmd(self, ticks) -> np.ndarray:
        return self._fk("cmd", self.q_cmd, ticks)

    def _fk(self, key: str, q: np.ndarray, ticks) -> np.ndarray:
        ticks = np.atleast_1d(ticks)
        out = np.empty((len(ticks), 3))
        for i, k in enumerate(ticks):
            k = int(k)
            hit = self._cache.get((key, k))
            if hit is None:
                hit = self._cache[(key, k)] = self.fk(q[k])
            out[i] = hit
        return out


def decompose_at_tc(
    ctx: TrialContext, truth: Truth | None, window_s: float, t_impact: float = math.inf
) -> dict:
    """The t_c gap decomposition of one trial (model world, metres → mm).

    **Lead.** With the actuation-lag lead on, the RT tick samples the reference
    (and the DECEL and γ-ramp clocks) at ``now + T_lead``, so ``q_cmd`` and
    ``ref`` written at tick t are aimed at ``t + T_lead`` (L5 §4.5). The command
    that was meant for t_c is therefore the one written at ``t_c − T_lead``
    (tick ``kl``), and the decomposition reads the command side there:

        FK(q_meas(t_c)) − p_true(t_c) = [FK(q_meas(t_c)) − FK(q_cmd(kl))]   servo
                                      + [FK(q_cmd(kl)) − ref(kl)]         CLIK
                                      + [ref(kl) − p_true(t_c)]           ref_vs_true

    ``cmd_meas_gap_mm`` is the same-tick ``‖FK(q_meas(t_c)) − FK(q_cmd(t_c))‖``,
    recorded but not a servo error under lead: it adds the intended lead to the
    residual, so lead-on reads WORSE even when the arm is closer (S8-B pre-check,
    #537 5808509678). With the lead off ``kl == kc`` and the two are equal.

    ``pred`` / ``total`` / ``ref_vs_true`` compare against where the ball WOULD
    be at t_c in free flight: when it has already hit the robot before t_c
    (``t_impact``), the recorded position at t_c is a rebound, not the target.
    ``truth_extrapolated_ms`` says how far the free flight was extrapolated.
    The arrival instant, by contrast, is read from the actual record.
    """
    t, mode = ctx.t, ctx.mode
    nan = math.nan
    rec = dict.fromkeys(
        (
            "t_commit",
            "t_c",
            "gamma_f_planned",
            "t_lead_s",
            "clik_mm",
            "servo_mm",
            "cmd_meas_gap_mm",
            "pred_mm",
            "total_mm",
            "ref_vs_true_mm",
            "arrival_ms",
            "d_min_mm",
            "ball_speed_tc",
            "truth_extrapolated_ms",
            "free_fit_rms_mm",
        ),
        nan,
    )
    k = _first(mode == MODE_COMMITTED)
    if k is None:
        return rec
    t_c = float(t[k] + ctx.plan_t_c[k])  # plan_t_c_s is t_c − now
    kc = int(np.argmin(np.abs(t - t_c)))
    t_lead = 0.0 if ctx.t_lead is None else float(ctx.t_lead[k])
    kl = int(np.argmin(np.abs(t - (t_c - t_lead)))) if t_lead > 0.0 else kc
    rec.update(
        t_commit=float(t[k]),
        t_c=t_c,
        gamma_f_planned=float(ctx.plan_gamma_f[k]),
        t_lead_s=t_lead,
    )
    f_cmd = ctx.fk_cmd(kl)[0]
    f_meas = ctx.fk_meas(kc)[0]
    rec["clik_mm"] = float(np.linalg.norm(f_cmd - ctx.ref[kl]) * 1e3)
    rec["servo_mm"] = float(np.linalg.norm(f_meas - f_cmd) * 1e3)
    rec["cmd_meas_gap_mm"] = float(np.linalg.norm(f_meas - ctx.fk_cmd(kc)[0]) * 1e3)
    if truth is None:
        return rec
    p_w, v_w, rms, t_last = truth.free_flight([t_c], t_impact)
    rec["truth_extrapolated_ms"] = max(0.0, (t_c - t_last) * 1e3)
    rec["free_fit_rms_mm"] = rms * 1e3
    p_true = ctx.fk.to_model(p_w[0])
    rec["pred_mm"] = float(np.linalg.norm(ctx.plan_p_c[kc] - p_true) * 1e3)
    rec["total_mm"] = float(np.linalg.norm(f_meas - p_true) * 1e3)
    rec["ref_vs_true_mm"] = float(np.linalg.norm(ctx.ref[kl] - p_true) * 1e3)
    rec["ball_speed_tc"] = float(np.linalg.norm(v_w[0]))
    # Arrival: the instant of closest approach between ball and measured catch
    # frame within ±window of t_c, evaluated on every tick.
    win = np.nonzero((t > t_c - window_s) & (t < t_c + window_s))[0]
    p_ball, _ = truth.at(t[win])
    ok = np.all(np.isfinite(p_ball), axis=1)
    if ok.any():
        win, p_ball = win[ok], p_ball[ok]
        d = np.linalg.norm(ctx.fk.to_model(p_ball) - ctx.fk_meas(win), axis=1)
        i = int(np.argmin(d))
        rec["arrival_ms"] = float((t[win[i]] - t_c) * 1e3)
        rec["d_min_mm"] = float(d[i] * 1e3)
    return rec


def planner_timing(ctx: TrialContext, t_launch: float) -> dict:
    approach = ctx.mode == MODE_APPROACH
    k = _first(approach)
    ids = ctx.plan_id[approach]
    ids = ids[ids > 0]
    return {
        "first_plan_s": math.nan if k is None else float(ctx.t[k] - t_launch),
        "approach_plan_switches": max(len(np.unique(ids)) - 1, 0),
    }


def plan_validity_window(
    wake_t_relative_s: np.ndarray,
    plan_valid: np.ndarray,
    t_launch: float,
    t_commit: float,
    t_end: float,
) -> dict:
    """G3-D (i): per-trial plan validity rate (L3 gate G3-D, #537 S8-D).

    ``wake_t_relative_s``/``plan_valid`` are every ``planner_events.csv`` row's
    wake instant (mapped onto the controller's ``t_relative_s`` axis — see
    :func:`_planner_cycle_times` for the mapping and its evidence) and
    ``plan_valid`` column, for the WHOLE session. The window is
    ``[t_launch, t_commit]`` when the trial committed (``t_commit`` finite),
    else ``[t_launch, t_end]`` — a trial that never reaches COMMITTED has no
    catch to judge plan validity against, so every wake up to RETREAT/timeout
    counts instead. ``plan_valid_ratio`` is NaN with 0 cycles in the window
    (nothing to divide by); ``plan_valid_at_commit`` — the LAST cycle at or
    before the window end — is NaN unless the trial actually committed, kept
    ``float`` (1.0/0.0) rather than ``bool`` so it stays NaN-able like this
    module's other ``_judge``-style columns (see :func:`hand_persist_met`).
    """
    has_commit = math.isfinite(t_commit)
    window_end = t_commit if has_commit else t_end
    sel = np.nonzero((wake_t_relative_s >= t_launch) & (wake_t_relative_s <= window_end))[0]
    n = int(sel.size)
    at_commit = math.nan
    if has_commit and n:
        at_commit = float(plan_valid[sel[-1]])
    return {
        "planner_cycles": n,
        "plan_valid_cycles": int(plan_valid[sel].sum()) if n else 0,
        "plan_valid_ratio": float(plan_valid[sel].mean()) if n else math.nan,
        "plan_valid_at_commit": at_commit,
    }


def first_hand_contact(
    contacts,
    seg_sim: tuple[float, float],
    hand_bodies: set[str],
    lane: ClockLane,
    seq: int,
    ctx: TrialContext,
    t_c: float,
    ball_mass_kg: float | None,
) -> dict:
    """The first contact episode between the ball and a hand body in this flight."""
    keys = (
        "contact_body",
        "contact_t_minus_tc_ms",
        "contact_impulse_ns",
        "contact_impulse_x",
        "contact_impulse_y",
        "contact_impulse_z",
        "contact_peak_force_n",
        "contact_duration_ms",
        "contact_ball_speed",
        "contact_hand_speed",
        "contact_v_rel",
        "contact_mv_rel_ns",
    )
    rec = dict.fromkeys(keys, math.nan)
    rec["contact_body"] = ""
    rec["robot_contacts"] = 0
    if contacts is None:
        return rec
    mine = trial_contacts(contacts, seg_sim)
    rec["robot_contacts"] = int(mine["is_robot"].sum())
    hand = mine[mine["first_body"].isin(hand_bodies)]
    if hand.empty:
        return rec
    row = hand.sort_values("begin_sim_time_sec").iloc[0]
    imp = row[["impulse_x", "impulse_y", "impulse_z"]].to_numpy(float)
    t_contact = lane.to_t_relative(seq, float(row["begin_sim_time_sec"]))
    v_ball = row[["vel_x", "vel_y", "vel_z"]].to_numpy(float)
    rec.update(
        contact_body=str(row["first_body"]),
        contact_t_minus_tc_ms=(t_contact - t_c) * 1e3 if np.isfinite(t_c) else math.nan,
        contact_impulse_ns=float(np.linalg.norm(imp)),
        contact_impulse_x=float(imp[0]),
        contact_impulse_y=float(imp[1]),
        contact_impulse_z=float(imp[2]),
        contact_peak_force_n=float(row["peak_force_n"]),
        contact_duration_ms=float(row["end_sim_time_sec"] - row["begin_sim_time_sec"]) * 1e3,
        contact_ball_speed=float(np.linalg.norm(v_ball)),
    )
    k = int(np.clip(np.searchsorted(ctx.t, t_contact), 2, len(ctx.t) - 3))
    span = ctx.t[k + 2] - ctx.t[k - 2]
    v_hand_model = (ctx.fk_meas(k + 2)[0] - ctx.fk_meas(k - 2)[0]) / span
    v_hand = ctx.fk.dir_to_world(v_hand_model)
    v_rel = float(np.linalg.norm(v_ball - v_hand))
    rec.update(contact_hand_speed=float(np.linalg.norm(v_hand)), contact_v_rel=v_rel)
    if ball_mass_kg is not None:
        rec["contact_mv_rel_ns"] = ball_mass_kg * v_rel
    return rec


def load_contacts(path: Path, robot_links: set[str]):
    df = _read_csv(path)
    df["is_robot"] = df["first_body"].isin(robot_links)
    return df


def trial_contacts(contacts, seg_sim: tuple[float, float]):
    """Contact episodes that began inside one flight's active sim window."""
    lo, hi = seg_sim
    begin = contacts["begin_sim_time_sec"]
    return contacts[(begin >= lo) & (begin <= hi)].sort_values("begin_sim_time_sec")


def first_robot_contact(contacts, seg_sim, lane: ClockLane, seq: int) -> float:
    """t_relative of the flight's first contact with ANY robot body, else +inf."""
    if contacts is None:
        return math.inf
    robot = trial_contacts(contacts, seg_sim)
    robot = robot[robot["is_robot"]]
    if robot.empty:
        return math.inf
    return lane.to_t_relative(seq, float(robot["begin_sim_time_sec"].iloc[0]))


# ══ Session driver ════════════════════════════════════════════════════════════


@dataclass
class Settings:
    truth_axis: str = "stamp"
    arrival_window_s: float = DEFAULT_ARRIVAL_WINDOW_S
    hold_radius_m: float | None = None
    v_max: float | None = None
    a_bound: float = clock_phase.DEFAULT_A_BOUND_M_S2
    eps_mm: tuple[float, ...] = ()
    n_boot: int = DEFAULT_N_BOOT
    seed: int = DEFAULT_SEED
    window_margin_s: float = 0.05
    hold_window_s: float = DEFAULT_HOLD_WINDOW_S
    floor: float | None = None  # G8-D floor (D-S8-3); None → no verdict
    n_valid_target: int | None = None  # verdict is INSUFFICIENT_N below this n_valid


@dataclass
class SessionResult:
    rows: list[dict]
    lag: list[ServoLag]
    summary: dict
    hand_window_rows: list[dict] = field(default_factory=list)


def analyse_session(
    session: Path,
    trials_dir: Path,
    profile: CatchingProfile,
    urdf_text: str,
    settings: Settings,
    clock_lane_path: Path | None = None,
    contact_lane_path: Path | None = None,
    gate_map: GateMap | None = None,
    eval_samples_path: Path | None = None,
    eval_report_path: Path | None = None,
    probe_dump_path: Path | None = None,
) -> SessionResult:
    """Join one session's CSVs, trials and lanes into the per-trial table.

    With a clock lane and a sim-sync session, every trial is first put on the
    lane's clocks (:func:`align_trials_to_lane`, ``time_alignment: "lane"``);
    without one, on the runner's median wall offset (``"median_offset"``,
    wrong by up to (1 − RTF) × the trial's length when the sim ran slow).
    ``eval_samples_path`` (``sim_capture_evaluate``) adds G8-B,
    ``probe_dump_path`` (``vision_lane_probe --dump``) G8-C2 —
    :mod:`rtc_tools.analysis.catching_vision`.
    """
    ctl = Path(session) / "controllers" / profile.controller
    header = _csv_header(ctl / f"{profile.diag_log}.csv")
    diag = _read_csv(ctl / f"{profile.diag_log}.csv", usecols=diag_columns(header))
    joints = arm_joints_from_diag(diag.columns)
    trials, doc = load_trials(trials_dir)
    t_all = diag["t_relative_s"].to_numpy(float)
    dt, dt_source = recorded_dt(doc, t_all)
    lead_all, lead_source = lead_per_tick(diag, doc["meta"])
    fk = CatchFrameFk(urdf_text, joints, profile)

    hand_bodies = set(urdf_subtree(urdf_text, profile.catch_frame.parent))
    robot_links = set(urdf_robot_links(urdf_text))
    # Raw records (not the trimmed `Trial` dataclass) — the gate-map axis
    # values (:func:`trial_axis_values`), the seed, the launch position and the
    # rig-failure fields (:func:`record_invalid_reason`) live only there.
    records_by_idx = {int(r["idx"]): r for r in doc["records"]}
    bridge = load_plan_bridge(ctl)
    dump = cv.load_probe_dump(probe_dump_path) if probe_dump_path is not None else None
    samples = cv.load_eval_samples(eval_samples_path) if eval_samples_path is not None else None
    lane = None
    clock_info: dict = {}
    if clock_lane_path is not None:
        lane = load_clock_lane(clock_lane_path, trials)
        c, clock_info = estimate_sim_minus_t_rel(
            t_all,
            diag["mode"].to_numpy(int),
            diag["plan_id"].to_numpy(int),
            diag["plan_t_c_s"].to_numpy(float),
            bridge,
            lane,
        )
        if dump is not None and bridge is not None:
            refine_wall_offset(lane, diag, bridge, dump, fk)
        if c is not None:
            lane.c_s = c
            trials = align_trials_to_lane(trials, records_by_idx, lane)
    contacts = None
    if contact_lane_path is not None:
        contacts = load_contacts(contact_lane_path, robot_links)
    planner_wakes = _planner_cycle_times(ctl, lane)
    wakes_t_rel = None
    if planner_wakes is not None and lane.aligned:
        # One map for the whole session: steady → sim (lane rows) − c.
        wakes_t_rel = lane.steady_to_t_rel(planner_wakes[0])
    timing, timing_status = _cm_timing(Path(session), lane)
    hand_effort = _hand_effort(ctl, profile)
    hand_profile = hand_caging_profile(profile)
    hand_kin = _hand_kinematics(ctl, profile, hand_profile) if hand_profile is not None else None
    t_persist_s = hand_capture_t_persist_s(profile)
    hold_radius = settings.hold_radius_m
    if hold_radius is None:
        hold_radius = profile.ball_diameter_m

    rows, hand_window_rows = [], []
    lag_t, lag_cmd, lag_meas, lag_moving, lag_cluster = [], [], [], [], []
    for trial in trials:
        record = records_by_idx.get(trial.idx, {})
        row = {"idx": trial.idx, "kind": trial.kind, "supervisor": trial.outcome}
        row["accepted"] = trial.accepted
        row["seed"] = record.get("seed")
        # D-S8-16 ①: rules 1–3 from the record; 3's "no diag rows" and the
        # lane rules 4–5 below, once the window and the lane segment are known.
        reason = record_invalid_reason(record)
        row["invalid_reason"] = reason
        if gate_map is not None:
            # Independent of `accepted`/diag data — the axis values are the
            # runner's own launch request, known whether or not it landed.
            axes = trial_axis_values(record)
            row.update(gate_map_verdict(axes, gate_map))
        if not trial.accepted or not np.isfinite(trial.t_launch):
            # Not accepted → srv_refused; accepted without an offset →
            # controller_silent (or not_launched, which outranks it).
            row["invalid_reason"] = reason or "controller_silent"
            rows.append(row)
            continue
        m = settings.window_margin_s
        sel = (t_all >= trial.t_launch - m) & (t_all <= trial.t_end + m)
        if not sel.any():
            row["invalid_reason"] = reason or "controller_silent"
            rows.append(row)
            continue
        w = diag[sel]
        ctx = TrialContext(
            t=w["t_relative_s"].to_numpy(float),
            mode=w["mode"].to_numpy(int),
            hand_phase=w["hand_phase"].to_numpy(int),
            plan_id=w["plan_id"].to_numpy(int),
            plan_t_c=w["plan_t_c_s"].to_numpy(float),
            plan_p_c=w[["plan_p_c_x", "plan_p_c_y", "plan_p_c_z"]].to_numpy(float),
            plan_gamma_f=w["plan_gamma_f"].to_numpy(float),
            ref=w[["ref_x_x", "ref_x_y", "ref_x_z"]].to_numpy(float),
            ref_valid=w["ref_valid"].to_numpy(int) != 0,
            ref_saturated=w["ref_saturated"].to_numpy(int) != 0,
            q_cmd=w[[f"q_cmd_{j}" for j in joints]].to_numpy(float),
            q_meas=w[[f"q_meas_{j}" for j in joints]].to_numpy(float),
            fk=fk,
            t_lead=lead_all[sel],
            hand_stalled_n=w["hand_stalled_n"].to_numpy(float)
            if "hand_stalled_n" in w.columns
            else None,
            hand_effort_frac=w["hand_effort_frac"].to_numpy(float)
            if "hand_effort_frac" in w.columns
            else None,
            hand_blocked_s=w["hand_blocked_s"].to_numpy(float)
            if "hand_blocked_s" in w.columns
            else None,
            outcome_source_tick=w["outcome_source"].to_numpy(float)
            if "outcome_source" in w.columns
            else None,
            input_seq=w["input_snapshot_sequence"].to_numpy(float)
            if "input_snapshot_sequence" in w.columns
            else None,
            input_gen=w["input_generation"].to_numpy(float)
            if "input_generation" in w.columns
            else None,
        )
        lag_t.append(ctx.t)
        lag_cmd.append(ctx.q_cmd)
        lag_meas.append(ctx.q_meas)
        lag_moving.append(np.isin(ctx.mode, MOVING_MODES))
        lag_cluster.append(np.full(len(ctx.t), trial.idx))
        on_lane = lane is not None and lane.aligned and trial.alignment == "lane"
        row["time_alignment"] = trial.alignment
        row["stamp_anchor"] = trial.stamp_anchor
        row["t_launch"] = trial.t_launch
        row["stamp_minus_t_rel_s"] = trial.stamp_offset  # ball stamp axis − t_relative_s
        row["truth_file"] = trial.truth_path is not None
        truth = None
        if trial.truth_path is not None:
            truth = load_truth(
                trial.truth_path,
                trial.stamp_offset if settings.truth_axis == "stamp" else trial.offset,
                settings.truth_axis,
                wall_to_t=lane.wall_to_t_rel if on_lane else None,
            )
        seq = seg = None
        t_impact = math.inf
        if lane is not None:
            seq = lane.trial_to_seq.get(trial.idx)
        if seq is not None:
            seg = lane.segments[seq]
            seg_sim = (float(seg[0, 0]), float(seg[-1, 0]))
            t_impact = first_robot_contact(contacts, seg_sim, lane, seq)
        if truth is not None:
            accel_limit = IMPACT_ACCEL_FACTOR * settings.a_bound
            t_impact = min(t_impact, truth.first_impact(trial.t_launch, accel_limit))
        row["t_first_impact"] = t_impact if np.isfinite(t_impact) else math.nan
        row.update(decompose_at_tc(ctx, truth, settings.arrival_window_s, t_impact))
        row.update(planner_timing(ctx, trial.t_launch))
        if planner_wakes is not None and trial.idx in lane.trial_offsets:
            wakes_t = (
                wakes_t_rel
                if (on_lane and wakes_t_rel is not None)
                else planner_wakes[0] - lane.trial_offsets[trial.idx]
            )
            row.update(
                plan_validity_window(
                    wakes_t,
                    planner_wakes[1],
                    trial.t_launch,
                    row.get("t_commit", math.nan),
                    trial.t_end,
                )
            )
        row["ref_saturated_max_streak"] = max_streak(ctx.ref_valid & ctx.ref_saturated)

        def fk_world(ticks, ctx=ctx):
            return transform_point(ctx.fk_meas(ticks), fk.world_t_model)

        if hold_radius is not None:
            row.update(
                truth_success(ctx.t, ctx.mode, ctx.hand_phase, truth, fk_world, hold_radius)
            )
        if lane is not None:
            row.update(_clock_covariate(lane, trial, row, settings))
        # The contact episode is located on the lane's launch segment, so a
        # trial the lane has no launch for has none — `seg_sim` would otherwise
        # be the PREVIOUS trial's segment (it is only assigned when paired).
        if seq is not None:
            row.update(
                first_hand_contact(
                    contacts,
                    seg_sim,
                    hand_bodies,
                    lane,
                    seq,
                    ctx,
                    row.get("t_c", math.nan),
                    profile.ball_mass_kg,
                )
            )
        if hand_effort is not None:
            row["hand_effort_max"] = _effort_max(hand_effort, trial, m)
        row.update(
            hand_witness_at_verdict(
                ctx.mode,
                ctx.hand_stalled_n,
                ctx.hand_effort_frac,
                ctx.hand_blocked_s,
                ctx.outcome_source_tick,
                row.get("supervisor"),
            )
        )
        row["t_release_to_pre_ms"] = hand_release_to_preshape_ms(ctx.t, ctx.mode, ctx.hand_phase)
        if lane is not None:
            lane_reason, lane_diag = lane_invalid_reason(lane, trial, settings.window_margin_s)
            row.update(lane_diag)
            reason = reason or lane_reason
        row["invalid_reason"] = reason
        if lane is not None and seq is not None:
            sim_c = t_impact + lane.c_s if (on_lane and np.isfinite(t_impact)) else math.nan
            sim_end = trial.t_end + lane.c_s if on_lane else math.inf
            row.update(trial_rtf(lane, seq, sim_c, sim_end))
        if timing is not None:
            if on_lane:
                lo, hi = (
                    float(v) for v in lane.t_rel_to_steady(seq, [trial.t_launch, trial.t_end])
                )
            elif trial.idx in lane.trial_offsets:
                off = lane.trial_offsets[trial.idx]
                lo, hi = trial.t_launch + off, trial.t_end + off
            else:
                lo = hi = math.nan
            row.update(tick_overrun(timing, lo, hi, dt))
        if samples is not None:
            lo_ns = (trial.t_launch + trial.stamp_offset) * 1e9
            t_hi = t_impact if np.isfinite(t_impact) else (truth.t[-1] if truth else math.nan)
            row.update(cv.nees_by_trial(samples, lo_ns, (t_hi + trial.stamp_offset) * 1e9))
        if dump is not None:
            row.update(c2_for_trial(ctx, trial, row, truth, t_impact, dump, bridge, lane, seq, fk))
        row["hand_persist_met"] = hand_persist_met(
            row.get("hand_blocked_s_judge", math.nan), dt, t_persist_s
        )
        if hand_profile is not None and hand_kin is not None:
            # Same gate as the judge columns (_hold_end_tick): RETREAT entered
            # from ABORT_SAFE never ran a HOLD-end hand-capture window either.
            k_ret = _hold_end_tick(ctx.mode)
            if k_ret is not None:
                t_hold_end = float(ctx.t[k_ret])
                th, tq, tqd, ttau = hand_kin
                extremes = hand_hold_window_extremes(
                    th, tq, tqd, ttau, hand_profile, t_hold_end, settings.hold_window_s
                )
                hand_window_rows.extend(
                    {"idx": trial.idx, "supervisor": row.get("supervisor"), **e} for e in extremes
                )
        rows.append(row)

    lag = []
    if lag_t:
        lag = servo_lag_ls(
            np.concatenate(lag_t),
            np.vstack(lag_cmd),
            np.vstack(lag_meas),
            np.concatenate(lag_moving),
            np.concatenate(lag_cluster),
            joints,
            settings.n_boot,
            settings.seed,
        )
    summary = _summarise(
        rows, lag, settings, lane, hold_radius, profile, joints, dt, dt_source, gate_map
    )
    valid = [r for r in rows if not r.get("invalid_reason")]
    summary["tick_overrun"] = tick_overrun_summary(valid, timing_status, dt)
    summary.update(time_alignment_summary(trials, lane, clock_info))
    summary["rtf"] = rtf_summary(valid)
    if samples is not None:
        summary["g8b"] = cv.g8b_summary(valid, settings.n_boot, settings.seed)
    if eval_report_path is not None:
        summary["g8b_unrestricted"] = cv.eval_report_horizons(eval_report_path)
    if dump is not None:
        summary["c2"] = cv.c2_summary(valid, settings.n_boot, settings.seed)
    summary["t_lead_s_range"] = _range(rows, "t_lead_s")
    summary["t_lead_source"] = lead_source
    summary["planner_events"] = _planner_events(ctl)
    return SessionResult(rows, lag, summary, hand_window_rows)


def _csv_header(path: Path) -> list[str]:
    import pandas as pd  # noqa: PLC0415

    found = _exists(path)
    if found is None:
        raise SystemExit(f"missing {path}")
    return list(pd.read_csv(found, nrows=0).columns)


def _hand_effort(ctl: Path, profile: CatchingProfile):
    hand = profile.hand_device
    if hand is None or hand not in profile.device_logs:
        return None
    path = _exists(ctl / f"{profile.device_logs[hand]}.csv")
    if path is None:
        return None
    cols = [c for c in _csv_header(path) if c.startswith("effort_")]
    if not cols:
        return None
    df = _read_csv(path, usecols=["t_relative_s", *cols])
    return df["t_relative_s"].to_numpy(float), np.abs(df[cols].to_numpy(float)).max(axis=1)


def _effort_max(effort, trial: Trial, margin: float) -> float:
    t, e = effort
    sel = (t >= trial.t_launch - margin) & (t <= trial.t_end + margin)
    return float(e[sel].max()) if sel.any() else math.nan


# ══ Hand-joint capture calibration (#537 S8-C, D-S8-8 (b)) ════════════════════
# Offline mirror of rtc_controllers/catching/hand_capture.hpp's per-tick
# EvaluateHandCapture, over a WINDOW instead of one tick, so rho_min/rho_max/
# qd_tol/effort_frac_min can be re-derived from real closures rather than
# guessed. Nothing here is robot-specific: joint names, poses and torque
# limits all come from the profile's YAML / device roster.


@dataclass
class HandCagingProfile:
    """``catching.robot.hand``'s caging pair joined to the hand device's torque
    limit, in the DEVICE's own joint order (a shipped profile's ``q_pre``
    comment documents this: "in the devices.<group>.joint_state_names
    order")."""

    joint_names: list[str]
    q_pre: np.ndarray
    q_close: np.ndarray
    caging_mask: np.ndarray  # bool, per joint
    tau_max: np.ndarray


def _calibration_skipped(profile: CatchingProfile, reason: str) -> None:
    """One-line stderr note + ``None``: the shared "give up on the OPTIONAL
    hand-hold-window calibration, not on the trial analysis" exit used by
    every degrade path below (#537 S8-C code review) — the read of
    ``catching.robot.hand``/the device roster/the hand device CSV can fail in
    several distinct ways (a length mismatch, a per-joint ``TBD``, a missing
    torque limit, an older CSV without the newer columns), and every one of
    them is a reason to skip this one calibration, never to abort
    ``analyse_session`` for the whole trial table.
    """
    print(
        f"catching_trials: hand-hold-window calibration skipped for '{profile.controller}' — "
        f"{reason}",
        file=sys.stderr,
    )
    return None


def hand_capture_t_persist_s(profile: CatchingProfile) -> float:
    """``catching.robot.hand.capture.t_persist`` [s], or NaN when the section
    is absent, still ``TBD``, or not a plain number — read for
    :func:`hand_persist_met`'s offline re-check only; nothing else in this
    module requires it, so an unusable value is NaN, never a refusal.
    """
    raw = (profile.hand_yaml.get("capture") or {}).get("t_persist")
    try:
        return float(raw)
    except (TypeError, ValueError):
        return math.nan


def hand_caging_profile(profile: CatchingProfile) -> HandCagingProfile | None:
    """Read ``HandCagingProfile`` from the profile, or ``None`` (via
    :func:`_calibration_skipped`) when the hand-hold-window calibration cannot
    run — not an error, since the calibration is an OPTIONAL extra on top of
    whatever else the profile is used for. Every one of the following degrades
    the same way, each reported with the specific reason:

    * no hand device, or ``q_pre``/``q_close`` still ``TBD``/absent (the whole
      block) — most sessions predate this feature;
    * ``q_pre``/``q_close`` is a list but one of its ELEMENTS is not numeric
      (a per-joint ``TBD``, the profile mid-search) — would otherwise raise
      ``ValueError`` out of ``np.asarray``;
    * ``catching.robot.hand.caging_mask`` present but the wrong length;
    * the hand device's ``joint_state_names`` (the DOF mapping between
      ``q_pre`` and the device roster) is absent or a different length;
    * the hand device's ``joint_limits.max_torque`` is absent or shorter than
      the caging pair — a real gap in the profile, but one that only this
      calibration needs, so it is reported (one line, not fatal) rather than
      aborting the whole session the way ``max_velocity`` would if this read
      it through ``derive_accel_limits.arm_spec_from_params`` (an earlier
      version of this function did exactly that).

    ``q_pre``/``q_close`` disagreeing on LENGTH with each other (as opposed to
    with the device), or being empty, is left a hard ``SystemExit``: the two
    arrays come from the same YAML block and cannot legitimately differ.
    """
    if profile.hand_device is None:
        return None
    hand = profile.hand_yaml
    q_pre_raw, q_close_raw = hand.get("q_pre"), hand.get("q_close")
    if not isinstance(q_pre_raw, list) or not isinstance(q_close_raw, list):
        return None
    try:
        q_pre = np.asarray(q_pre_raw, dtype=float)
        q_close = np.asarray(q_close_raw, dtype=float)
    except (TypeError, ValueError):
        return _calibration_skipped(
            profile,
            "catching.robot.hand.q_pre/q_close has a non-numeric element (a per-joint TBD?)",
        )
    if q_pre.shape != q_close.shape or q_pre.size == 0:
        raise SystemExit(
            f"{profile.controller}: catching.robot.hand.q_pre/q_close must be equal-length, "
            "non-empty arrays"
        )
    n = q_pre.size
    mask_raw = hand.get("caging_mask")
    caging_mask = np.ones(n, dtype=bool) if mask_raw is None else np.asarray(mask_raw, dtype=bool)
    if caging_mask.shape != (n,):
        return _calibration_skipped(
            profile, f"catching.robot.hand.caging_mask does not have {n} entries"
        )
    dev = (profile.robot_params.get("devices") or {}).get(profile.hand_device) or {}
    joint_names = [str(j) for j in dev.get("joint_state_names") or []]
    if len(joint_names) != n:
        return _calibration_skipped(
            profile,
            f"devices.{profile.hand_device} has {len(joint_names)} joints, "
            f"catching.robot.hand.q_pre has {n} — they must be the same hand in the same order",
        )
    tau_max_raw = ((dev.get("joint_limits") or {}).get("max_torque")) or []
    if len(tau_max_raw) < n:
        return _calibration_skipped(
            profile,
            f"devices.{profile.hand_device}.joint_limits.max_torque is missing or shorter "
            f"than {n} joints",
        )
    return HandCagingProfile(
        joint_names, q_pre, q_close, caging_mask, np.asarray(tau_max_raw[:n], dtype=float)
    )


def _hand_kinematics(ctl: Path, profile: CatchingProfile, hand: HandCagingProfile):
    """(t, q, qd, tau) from the hand device state CSV, columns in ``hand.joint_names``
    order (``DeviceStateLogPod``'s ``actual_pos_<joint>``/``actual_vel_<joint>``/
    ``effort_<joint>``). ``None`` when the device log is not recorded at all —
    the same "not an error" contract as :func:`_hand_effort` — or (via
    :func:`_calibration_skipped`) when it IS recorded but predates one of
    those column blocks, e.g. an older session logged before ``actual_vel_*``
    or ``effort_*`` were added.
    """
    dev = profile.hand_device
    if dev is None or dev not in profile.device_logs:
        return None
    path = _exists(ctl / f"{profile.device_logs[dev]}.csv")
    if path is None:
        return None
    pos_cols = [f"actual_pos_{j}" for j in hand.joint_names]
    vel_cols = [f"actual_vel_{j}" for j in hand.joint_names]
    eff_cols = [f"effort_{j}" for j in hand.joint_names]
    header = _csv_header(path)
    missing = [c for c in (*pos_cols, *vel_cols, *eff_cols) if c not in header]
    if missing:
        return _calibration_skipped(profile, f"{path} lacks hand state column(s) {missing}")
    df = _read_csv(path, usecols=["t_relative_s", *pos_cols, *vel_cols, *eff_cols])
    return (
        df["t_relative_s"].to_numpy(float),
        df[pos_cols].to_numpy(float),
        df[vel_cols].to_numpy(float),
        df[eff_cols].to_numpy(float),
    )


def hand_hold_window_extremes(
    t: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    tau: np.ndarray,
    hand: HandCagingProfile,
    t_hold_end: float,
    window_s: float,
) -> list[dict]:
    """Per caging joint, the extremes over ``[t_hold_end - window_s, t_hold_end)``:
    ``rho_lo``/``rho_hi`` (the ρ band the joint actually sat in), ``qd_absmax``,
    ``frac_lo`` (the WORST — minimum — signed torque fraction s·τ/τ_max, so a
    threshold set at or below it would have held for the whole window) and
    ``n`` samples. ρ and the sign convention are exactly
    ``rtc_controllers/catching/hand_capture.hpp``'s (ρ_i = (q_i − q_pre_i)·s_i /
    |q_close_i − q_pre_i|, s_i = sign(q_close_i − q_pre_i)). A non-caging joint
    is skipped entirely (not in the output); a joint with no finite sample in
    the window gets NaN extremes and ``n`` = 0.
    """
    sel = (t >= t_hold_end - window_s) & (t < t_hold_end)
    out: list[dict] = []
    for i, name in enumerate(hand.joint_names):
        if not hand.caging_mask[i]:
            continue
        row = {
            "joint": name,
            "rho_lo": math.nan,
            "rho_hi": math.nan,
            "qd_absmax": math.nan,
            "frac_lo": math.nan,
            "n": 0,
        }
        span = hand.q_close[i] - hand.q_pre[i]
        if np.isfinite(span) and span != 0.0 and sel.any():
            s = 1.0 if span > 0.0 else -1.0
            qi, qdi, taui = q[sel, i], qd[sel, i], tau[sel, i]
            # The sign/progress convention is hand_close's ``joint_progress``
            # (P5: do not re-derive it) — identical to
            # ``rtc_controllers/catching/hand_capture.hpp``'s ρ_i.
            rho = hand_close.joint_progress(qi, hand.q_pre[i], hand.q_close[i])
            tau_max_i = hand.tau_max[i]
            if np.isfinite(tau_max_i) and tau_max_i > 0.0:
                frac = s * taui / tau_max_i
            else:
                frac = np.full_like(taui, math.nan)
            finite = np.isfinite(rho) & np.isfinite(qdi)
            n = int(finite.sum())
            if n:
                row["rho_lo"] = float(rho[finite].min())
                row["rho_hi"] = float(rho[finite].max())
                row["qd_absmax"] = float(np.abs(qdi[finite]).max())
                finite_frac = finite & np.isfinite(frac)
                row["frac_lo"] = float(frac[finite_frac].min()) if finite_frac.any() else math.nan
                row["n"] = n
        out.append(row)
    return out


def capture_would_fire(
    window_rows_for_one_trial: Sequence[Mapping],
    rho_min: float,
    rho_max: float,
    qd_tol: float,
    effort_frac_min: float,
    min_joints: int,
) -> bool:
    """The offline mirror of "held unbroken over the window": at least
    ``min_joints`` caging joints whose window stayed inside
    ``[rho_min, rho_max]``, ``|q̇| <= qd_tol`` and whose worst-case torque
    fraction still met ``effort_frac_min`` — the same four clauses
    ``EvaluateHandCapture`` (hand_capture.hpp) checks per tick, each bound
    inclusive, evaluated over the whole window (``hand_hold_window_extremes``'
    rows) instead of one tick so a joint that only glances into range does not
    count. A row with a NaN extreme (no finite sample) never passes.
    """
    n = 0
    for row in window_rows_for_one_trial:
        rho_lo, rho_hi = row["rho_lo"], row["rho_hi"]
        qd_absmax, frac_lo = row["qd_absmax"], row["frac_lo"]
        if not (
            np.isfinite(rho_lo)
            and np.isfinite(rho_hi)
            and np.isfinite(qd_absmax)
            and np.isfinite(frac_lo)
        ):
            continue
        if (
            rho_lo >= rho_min
            and rho_hi <= rho_max
            and qd_absmax <= qd_tol
            and frac_lo >= effort_frac_min
        ):
            n += 1
    return n >= min_joints


def record_invalid_reason(record: Mapping) -> str:
    """D-S8-16 ① rules 1–3 as far as the runner's record decides them ("" = none).

    1. ``srv_refused`` — the launch srv answered ``accepted == false``;
    2. ``not_launched`` — accepted, but the runner recorded no ground-truth
       row (``n_truth_rows == 0``; a record older than the field is not judged);
    3. ``controller_silent`` — no ``wall_t_relative_offset``: the runner saw no
       controller state message in the trial. The other half of rule 3 (the
       window holds no diag row) needs the diag and is applied by the caller.
    """
    if not record.get("accepted"):
        return "srv_refused"
    n_truth = record.get("n_truth_rows")
    if n_truth is not None and int(n_truth) == 0:
        return "not_launched"
    if record.get("wall_t_relative_offset") is None:
        return "controller_silent"
    return ""


def lane_invalid_reason(lane: ClockLane, trial: Trial, margin_s: float) -> tuple[str, dict]:
    """D-S8-16 ① rules 4–5 from the clock lane, and the diagnostics they read.

    4. ``lane_drop`` — no launch segment on the lane pairs with this trial
       (:func:`load_clock_lane`; a segment needs ≥ 2 in-flight rows), or
       ``dropped_total`` rises across the trial's rows;
    5. ``sim_stall`` — a consecutive ``sim_time_sec`` gap inside the trial's
       rows above :data:`SIM_STALL_STEP_FACTOR` × the lane's nominal step
       (median spacing over the whole lane).

    The trial's rows are those carrying its ``launch_seq`` from the launch to
    the trial's end (+ ``margin_s``, the diag window's margin) on the steady
    clock — through the lane's sim ↔ steady map when the trial is
    lane-aligned, else its own launch offset (:meth:`ClockLane.rig_check`).
    """
    seq = lane.trial_to_seq.get(trial.idx)
    if seq is None:
        return "lane_drop", {"lane_dropped_delta": math.nan, "lane_max_sim_gap_s": math.nan}
    if lane.aligned and trial.alignment == "lane":
        end = float(lane.t_rel_to_steady(seq, trial.t_end + margin_s))
    else:
        offset = lane.trial_offsets.get(trial.idx)
        end = trial.t_end + margin_s + offset if offset is not None else math.inf
    dropped, gap = lane.rig_check(seq, end)
    diag = {"lane_dropped_delta": dropped, "lane_max_sim_gap_s": gap}
    if dropped > 0:
        return "lane_drop", diag
    if np.isfinite(gap) and gap > SIM_STALL_STEP_FACTOR * lane.nominal_step_s:
        return "sim_stall", diag
    return "", diag


def _cm_timing(session: Path, lane: ClockLane | None):
    """The RT loop's per-tick timing lane as ``(t_steady_s, t_total_us, jitter_us)``.

    ``t_wall_ns`` is NOT wall time: ``ThreadTimingProducer::NowNs`` reads
    ``std::chrono::steady_clock`` (rtc_base/include/rtc_base/timing/
    thread_timing_producer.hpp), the same CLOCK_MONOTONIC the clock lane's
    ``steady_ns`` uses, while the runner's ``launch_wall_time`` is
    ``time.time()`` (epoch). The join is therefore through the lane (its sim ↔
    steady map, or the per-trial launch offset on the fallback path), not the
    runner's wall offset, and without a lane there is none. The lane read is ``cm_timing_log.csv`` (one row per RT
    tick); ``rt_callback_timing_log.csv`` is one row per device state
    callback, not per tick, so a per-tick overrun cannot be read from it.
    Returns ``(None, "NOT_EVALUATED(<why>)")`` when unavailable.
    """
    path = _exists(session.joinpath(*CM_TIMING_LOG))
    if path is None:
        return None, "NOT_EVALUATED(no timing log)"
    if lane is None:
        return None, (
            "NOT_EVALUATED(no clock lane — timing t_wall_ns is steady_clock and only the "
            "lane maps it onto t_relative_s)"
        )
    df = _read_csv(path, usecols=["t_wall_ns", "t_total_us", "jitter_us"])
    df = df.sort_values("t_wall_ns")
    timing = (
        df["t_wall_ns"].to_numpy(float) * 1e-9,
        df["t_total_us"].to_numpy(float),
        df["jitter_us"].to_numpy(float),
    )
    return timing, None


def tick_overrun(timing, steady_lo: float, steady_hi: float, dt: float) -> dict:
    """Ticks over the control period and the largest jitter in a steady window."""
    out = {"tick_overrun_n": math.nan, "tick_jitter_max_us": math.nan}
    if not (np.isfinite(steady_lo) and np.isfinite(steady_hi)):
        return out
    t_s, total_us, jitter_us = timing
    lo = np.searchsorted(t_s, steady_lo, side="left")
    hi = np.searchsorted(t_s, steady_hi, side="right")
    if hi <= lo:
        return out
    out["tick_overrun_n"] = int(np.sum(total_us[lo:hi] > dt * 1e6))
    out["tick_jitter_max_us"] = float(np.max(jitter_us[lo:hi]))
    return out


def tick_overrun_summary(
    rows: Sequence[Mapping], status: str | None, dt: float | None
) -> dict | str:
    """Tick-overrun covariate over ``rows`` (a covariate, not a verdict — plan §4.4 S8-E)."""
    if status is not None:
        return status
    have = [r for r in rows if np.isfinite(_num(r.get("tick_overrun_n")))]
    if not have:
        return "NOT_EVALUATED(no trial window on the timing log)"
    out = {
        "source": "/".join(CM_TIMING_LOG),
        "n_trials": len(have),
        "n_trials_with_overrun": sum(1 for r in have if _num(r["tick_overrun_n"]) > 0),
        "overrun_ticks_total": int(sum(_num(r["tick_overrun_n"]) for r in have)),
        "tick_overrun_n_p50_p95_max": _p50_p95_max(have, "tick_overrun_n"),
        "tick_jitter_max_us_p50_p95_max": _p50_p95_max(have, "tick_jitter_max_us"),
    }
    if dt is not None:
        out["period_us"] = dt * 1e6
    return out


def _is_true(value) -> bool:
    """A real boolean True (a NaN / None / missing cell is not a success)."""
    return isinstance(value, bool | np.bool_) and bool(value)


def _num(value) -> float:
    """A row cell as a float (None / non-numeric → NaN)."""
    if value is None or isinstance(value, str):
        return math.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def streak_distribution(rows: Sequence[Mapping], bin_width: int = STREAK_BIN_WIDTH) -> dict:
    """G8-C3 frequency distribution of per-trial ``ref_saturated_max_streak``.

    The gate was deleted (D-S8-16) — this is a record, not a verdict.
    Histogram bins: ``0``, then ``1-w``, ``w+1-2w``, … up to the largest streak.
    """
    values = [
        int(_num(r.get("ref_saturated_max_streak")))
        for r in rows
        if np.isfinite(_num(r.get("ref_saturated_max_streak")))
    ]
    out: dict = {"note": "G8-C3 deleted (D-S8-16) — distribution recorded, no verdict"}
    out["n_trials"] = len(values)
    out["n_streak_gt0"] = sum(1 for v in values if v > 0)
    if not values:
        out.update(p50=None, p95=None, p99=None, max=None, histogram={})
        return out
    out["p50"] = float(np.median(values))
    out["p95"] = float(clock_phase.quantile(values, 0.95))
    out["p99"] = float(clock_phase.quantile(values, 0.99))
    out["max"] = int(max(values))
    hist = {"0": sum(1 for v in values if v == 0)}
    for lo in range(1, max(values) + 1, bin_width):
        hi = lo + bin_width - 1
        hist[f"{lo}-{hi}"] = sum(1 for v in values if lo <= v <= hi)
    out["bin_width"] = bin_width
    out["histogram"] = hist
    return out


def impulse_correlation(rows: Sequence[Mapping], n_boot: int, seed: int) -> dict:
    """G7-B3: predicted Δp = m·v_rel vs the measured contact impulse ∫F dt.

    First hand–ball contact episode only (:func:`first_hand_contact`), over
    the rows with both finite. OLS slope/intercept with a trial-bootstrap
    95 % CI, and Spearman ρ (with p) of Δp vs the impulse and vs the episode's
    peak force. No pass threshold is defined, so ρ is reported, and below
    :data:`B3_SPEARMAN_N_MIN` trials it is marked not evaluated. The joint
    torque half is not evaluated in sim: the hand's forcerange clamps it.
    """
    pts = [
        (
            _num(r.get("contact_mv_rel_ns")),
            _num(r.get("contact_impulse_ns")),
            _num(r.get("contact_peak_force_n")),
        )
        for r in rows
    ]
    pts = [p for p in pts if np.isfinite(p[0]) and np.isfinite(p[1])]
    x = np.array([p[0] for p in pts], float)
    y = np.array([p[1] for p in pts], float)
    slope, intercept, slope_ci, icpt_ci = ols_with_bootstrap(x, y, n_boot, seed)
    rho, p = spearman(x, y)
    has_f = np.array([np.isfinite(q[2]) for q in pts], bool)
    f = np.array([q[2] for q in pts], float)
    rho_f, p_f = spearman(x[has_f], f[has_f]) if pts else (math.nan, math.nan)
    n = len(pts)
    return {
        "episode": "first hand–ball contact",
        "x": "predicted Δp = m·v_rel [N·s] (contact_mv_rel_ns)",
        "y": "measured ∫F dt [N·s] (contact_impulse_ns)",
        "n": n,
        "ols_slope": slope,
        "ols_slope_ci95": slope_ci,
        "ols_intercept_ns": intercept,
        "ols_intercept_ci95": icpt_ci,
        "spearman_rho_impulse": rho,
        "spearman_p_impulse": p,
        "n_peak_force": int(has_f.sum()) if pts else 0,
        "spearman_rho_peak_force": rho_f,
        "spearman_p_peak_force": p_f,
        "spearman_verdict": f"NOT_EVALUATED(n < {B3_SPEARMAN_N_MIN})"
        if n < B3_SPEARMAN_N_MIN
        else "REPORTED(no pass threshold defined)",
        "torque": "NOT_EVALUATED(sim clamp)",
    }


def truth_block(
    valid: Sequence[Mapping],
    n_total: int,
    floor: float | None = None,
    n_valid_target: int | None = None,
    z: float = 1.96,
) -> dict:
    """G8-D over the VALID trials, with the ITT bound (invalid counted as failure).

    ``lower_975`` is the Wilson lower bound at ``z`` — the 97.5 % one-sided
    bound at the default 1.96 (plan §1a).
    """
    k = sum(1 for r in valid if _is_true(r.get("truth_success")))
    n = len(valid)
    confusion: dict[str, dict[str, int]] = {}
    for r in valid:
        cell = confusion.setdefault(
            str(r.get("supervisor")), {"truth_success": 0, "truth_fail": 0}
        )
        cell["truth_success" if _is_true(r.get("truth_success")) else "truth_fail"] += 1
    lo, hi = wilson_interval(k, n, z)
    itt = wilson_interval(k, n_total, z)
    out = {
        "successes": k,
        "n": n,
        "p_hat": k / n if n else math.nan,
        "wilson95": (lo, hi),
        "lower_975": lo,
        "z": z,
        "itt": {
            "successes": k,
            "n": n_total,
            "p_hat": k / n_total if n_total else math.nan,
            "wilson95": itt,
            "lower_975": itt[0],
            "note": "invalid trials counted as failures",
        },
        "confusion_supervisor_vs_truth": confusion,
    }
    if floor is not None:
        out["floor"] = floor
        out["verdict"] = floor_verdict(k, n, floor, n_valid_target, z)
    elif n_valid_target is not None:
        out["n_valid_target"] = n_valid_target
    return out


def validity_block(rows: Sequence[Mapping], lane_rules_evaluated: bool) -> dict:
    """D-S8-16 ①: every trial counted once — valid, or invalid for exactly one reason."""
    n_invalid = dict.fromkeys(INVALID_REASONS, 0)
    for r in rows:
        reason = r.get("invalid_reason")
        if reason:
            if reason not in n_invalid:
                raise ValueError(f"trial {r.get('idx')}: unknown invalid_reason {reason!r}")
            n_invalid[reason] += 1
    out = {
        "n_total": len(rows),
        "n_invalid": n_invalid,
        "n_invalid_total": sum(n_invalid.values()),
        "n_valid": len(rows) - sum(n_invalid.values()),
        "lane_rules_evaluated": lane_rules_evaluated,
    }
    if not lane_rules_evaluated:
        out["lane_rules"] = "NOT_EVALUATED(no clock lane — lane_drop / sim_stall not checked)"
    return out


def refine_wall_offset(lane: ClockLane, diag, bridge: PlanBridge, dump, fk: CatchFrameFk) -> None:
    """Replace the launch-pairing wall offset by the exact one, when the dump allows.

    The plan's ``p_c`` is one sample of the planner's snapshot
    (:func:`catching_vision.plan_point_stamp`), whose stamp + horizon is ``t_c``
    on the stamp axis; the planner row gives the same instant on the steady
    clock. Their difference is CLOCK_REALTIME − CLOCK_MONOTONIC (the ball
    stamps are steady instants shifted by it, ``mujoco_simulator_node.cpp``).
    On the S8-E smoke units it matched the probe's own (wall, steady) receipt
    pairs to 1 µs.
    """
    t = diag["t_relative_s"].to_numpy(float)
    mode = diag["mode"].to_numpy(int)
    pid = diag["plan_id"].to_numpy(int)
    p_c = diag[["plan_p_c_x", "plan_p_c_y", "plan_p_c_z"]].to_numpy(float)
    values = []
    for k in commit_ticks(t, mode):
        key = bridge.snapshot.get(int(pid[k]))
        if key is None:
            continue
        p_w = transform_point(p_c[k][None], fk.world_t_model)[0]
        t_ns, _ = cv.plan_point_stamp(dump, key, p_w)
        if np.isfinite(t_ns):
            values.append(t_ns * 1e-9 - bridge.t_c_steady(int(pid[k])))
    if values:
        lane.wall_offset_s = float(np.median(values))
        lane.wall_offset_source = (
            f"plan_point (n {len(values)}, spread {np.ptp(values) * 1e6:.1f} µs)"
        )


def c2_for_trial(
    ctx: TrialContext,
    trial: Trial,
    row: Mapping,
    truth: Truth | None,
    t_impact: float,
    dump,
    bridge: PlanBridge | None,
    lane: ClockLane | None,
    seq: int | None,
    fk: CatchFrameFk,
) -> dict:
    """G8-C2 inputs of one trial: A = p_true − p̂_live and B = p̂_live − p_c at t_c.

    All three in the SIM world (the dump's ``world``: ``p_c`` goes through the
    same ``world_t_model`` as the truth, and :func:`catching_vision.plan_point_stamp`
    finds it among the planner snapshot's points to µm — the frame check).

    * ``t_c`` on the stamp axis — ``c2_tc_source``: ``plan_point`` (the stamp of
      the planner snapshot's point that IS ``p_c``: exact), ``planner_wake``
      (the planner's steady ``t_c`` + the wall offset), else ``t_rel``
      (``t_c`` column + the trial's stamp offset). Not the ``t_c`` column
      itself: ``plan_t_c_s`` counts STEADY time, which under RTF < 1 runs
      ahead of the sim axis (``c2_tc_minus_tc_ms`` shows by how much).
    * p̂_live — the snapshot the commit tick read (diag input key, ``c2_join``
      ``exact``); missing from the dump (or no key columns), the last snapshot
      the probe received at or before the commit tick's steady instant
      (``approx``).
    """
    k = _first(ctx.mode == MODE_COMMITTED)
    if k is None:
        return {}
    out: dict = {"c2_join": "none", "c2_tc_source": "", "c2_pc_match_mm": math.nan}
    pid = int(ctx.plan_id[k])
    p_c_w = transform_point(ctx.plan_p_c[k][None], fk.world_t_model)[0]
    t_c_ns = math.nan
    if bridge is not None and pid in bridge.snapshot:
        t_c_ns, dist = cv.plan_point_stamp(dump, bridge.snapshot[pid], p_c_w)
        out["c2_pc_match_mm"] = dist * 1e3
        if np.isfinite(t_c_ns):
            out["c2_tc_source"] = "plan_point"
    if not np.isfinite(t_c_ns) and bridge is not None and lane is not None:
        t_c_steady = bridge.t_c_steady(pid)
        if np.isfinite(t_c_steady) and np.isfinite(lane.wall_offset_s):
            t_c_ns = (t_c_steady + lane.wall_offset_s) * 1e9
            out["c2_tc_source"] = "planner_wake"
    if not np.isfinite(t_c_ns) and np.isfinite(row.get("t_c", math.nan)):
        t_c_ns = (row["t_c"] + trial.stamp_offset) * 1e9
        out["c2_tc_source"] = "t_rel"
    t_c_rel = t_c_ns * 1e-9 - trial.stamp_offset
    out["c2_tc_minus_tc_ms"] = (t_c_rel - row.get("t_c", math.nan)) * 1e3
    commit_steady = math.nan
    if bridge is not None and pid in bridge.wake_s:
        commit_steady = bridge.t_c_steady(pid) - float(ctx.plan_t_c[k])
    elif lane is not None and lane.aligned and seq is not None and trial.alignment == "lane":
        commit_steady = float(lane.t_rel_to_steady(seq, float(ctx.t[k])))
    key = None
    if ctx.input_seq is not None and ctx.input_gen is not None:
        cand = (int(ctx.input_seq[k]), int(ctx.input_gen[k]))
        if cand in dump.points:
            key, out["c2_join"] = cand, "exact"
    if key is None:
        key = cv.last_snapshot_before(dump, commit_steady * 1e9)
        if key is not None:
            out["c2_join"] = "approx"
    p_live = cv.p_hat_at(dump, key, t_c_ns) if key is not None else np.full(3, math.nan)
    p_true = np.full(3, math.nan)
    if truth is not None and np.isfinite(t_c_rel):
        p_true = truth.free_flight([t_c_rel], t_impact)[0][0]
    out.update(cv.c2_values(p_true, p_live, p_c_w))
    return out


def time_alignment_summary(trials: Sequence[Trial], lane: ClockLane | None, info: Mapping) -> dict:
    """How the trials were put on the controller's clock (``time_alignment``)."""
    runs = [t for t in trials if t.accepted]
    n_lane = sum(1 for t in runs if t.alignment == "lane")
    if n_lane == 0:
        method = "median_offset"
    elif n_lane == len(runs):
        method = "lane"
    else:
        method = f"mixed(lane {n_lane}, median_offset {len(runs) - n_lane})"
    detail: dict = {"n_lane": n_lane, "n_median_offset": len(runs) - n_lane}
    if lane is not None:
        detail.update(
            sim_minus_t_rel_s=lane.c_s,
            sim_minus_t_rel=dict(info),
            realtime_minus_steady_s=lane.wall_offset_s,
            realtime_minus_steady_source=lane.wall_offset_source,
            stamp_rtf=STAMP_RTF,
            stamp_anchor={
                how: sum(1 for t in runs if t.stamp_anchor == how)
                for how in ("truth", "lane_estimate")
            },
        )
    else:
        detail["note"] = "no clock lane — the runner's median wall offset (drifts when RTF < 1)"
    return {"time_alignment": method, "time_alignment_detail": detail}


def rtf_summary(rows: Sequence[Mapping]) -> dict | str:
    """p05 / p50 / min of the per-trial RTF covariates (not a validity rule)."""
    out = {}
    for key in ("rtf_flight", "rtf_trial_min"):
        values = [_num(r.get(key)) for r in rows]
        values = [v for v in values if np.isfinite(v)]
        out[key] = (
            {
                "n": len(values),
                "p05": float(clock_phase.quantile(values, 0.05)),
                "p50": float(np.median(values)),
                "min": float(min(values)),
            }
            if values
            else None
        )
    if all(v is None for v in out.values()):
        return "NOT_EVALUATED(no RTF column — no clock lane)"
    return out


def _clock_covariate(lane: ClockLane, trial: Trial, row: Mapping, settings: Settings) -> dict:
    seq = lane.trial_to_seq.get(trial.idx)
    if seq is None:
        # No launch on the lane pairs with this trial under the run's clock
        # offset (load_clock_lane) — no covariate rather than a borrowed one.
        return {"launch_seq": None, "d3_unpaired": True}
    ct = next(t for t in lane.trials if t.launch_seq == seq)
    out = {
        "launch_seq": seq,
        "delta_max_ms": ct.delta_max_s * 1e3,
        "max_pause_ms": ct.max_pause_s * 1e3,
        "delta_commit_ms": math.nan,
        "delta_tc_ms": math.nan,
    }
    for key, t_key in (("delta_commit_ms", "t_commit"), ("delta_tc_ms", "t_c")):
        t_rel = row.get(t_key, math.nan)
        if np.isfinite(t_rel):
            out[key] = lane.delta_at(seq, t_rel) * 1e3
    if settings.v_max is not None:
        need = clock_phase.required_eps(ct, settings.v_max, settings.a_bound)
        out["eps_required_mm"] = need * 1e3
        for eps in settings.eps_mm:
            out[f"d3_valid@{eps:g}mm"] = bool(need * 1e3 <= eps)
    return out


def _planner_events(ctl: Path) -> dict | None:
    path = _exists(ctl / PLANNER_EVENTS_CSV)
    if path is None:
        return None
    df = _read_csv(path)
    out = {"rows": len(df)}
    for col in ("decision", "outcome"):
        if col in df.columns:
            out[col] = {str(k): int(v) for k, v in df[col].value_counts().items()}
    return out


def _planner_cycle_times(
    ctl: Path, lane: ClockLane | None
) -> tuple[np.ndarray, np.ndarray] | None:
    """Every ``planner_events.csv`` wake as a STEADY-clock second, with its
    ``plan_valid`` — what G3-D (i) (#537 S8-D) windows by trial after shifting
    it onto ``t_relative_s``. ``None`` when either input is unavailable: a diag
    predating the planner, or no ``--clock-lane`` (below).

    **The mapping and its evidence.** ``wake_ns`` is a STEADY-clock instant —
    ``PlannerCycleRecord::wake_ns`` is documented "Steady instants"
    (rtc_controllers/include/rtc_controllers/catching/planner_cycle.hpp:75-79)
    and set from ``rtc::SteadyNowNs()``, i.e. ``std::chrono::steady_clock``
    (rtc_base/include/rtc_base/types/types.hpp:265-272). But a sim profile can
    run the RT loop in SIM-SYNC mode (``use_sim_time_sync``, declared at
    rtc_controller_manager/src/rt_controller_node_params.cpp:279), under which
    ``t_relative_s`` is NOT a wall/steady clock reading at all: it is
    ``iteration × dt`` (rtc_controller_manager/src/rt_controller_node_rt_loop.
    cpp:457-459), a pure simulator tick count decoupled from real elapsed
    time (the sim can run faster or slower than 1×). A steady-clock instant
    therefore has no FIXED additive offset to ``t_relative_s`` in general —
    exactly the clock-drift problem the D-3 clock lane already exists to
    measure (:class:`ClockLane`, plan §5).

    The clock lane bridges the two clocks by recording ``(sim_time_sec,
    steady_ns)`` pairs from the SAME ``std::chrono::steady_clock``
    (rtc_mujoco_sim/src/mujoco_sim_loop.cpp:929-937), and
    :func:`load_clock_lane` pairs each trial's launch with its lane launch,
    which gives that trial's own offset ``ClockLane.trial_offsets[idx]`` =
    steady_s − t_relative_s at launch. The caller converts with
    ``t_rel = steady − trial_offsets[idx]`` per trial, NOT with the session
    median ``steady_offset``: when the sim runs below 1× the offset grows
    across a session, and a median shifts late trials' windows by that drift.
    Within one flight (~1 s) the drift is negligible. A trial the lane left
    unpaired has no offset and gets no G3-D columns, the same rule as the D-3
    clock covariate. See the golden test for a numerical check against real
    fixture data (every ``wake_ns`` of a trial with a commit lands inside that
    trial's own diag window once converted).
    """
    if lane is None:
        return None
    path = _exists(ctl / PLANNER_EVENTS_CSV)
    if path is None:
        return None
    df = _read_csv(path, usecols=["wake_ns", "plan_valid"])
    return df["wake_ns"].to_numpy(float) * 1e-9, df["plan_valid"].to_numpy(float)


def _median(rows: Sequence[Mapping], key: str) -> float:
    values = [r[key] for r in rows if key in r and np.isfinite(r[key])]
    return float(np.median(values)) if values else math.nan


def _summarise(
    rows, lag, settings, lane, hold_radius, profile, joints, dt, dt_source, gate_map=None
) -> dict:
    run = [r for r in rows if r.get("accepted")]
    # D-S8-16 ①: every statistic below is over the VALID trials — invalid is a
    # rig failure, not an attempt (the ITT bound is the one exception).
    valid = [r for r in rows if not r.get("invalid_reason")]
    verdicts: dict[str, int] = {}
    for r in run:
        verdicts[str(r["supervisor"])] = verdicts.get(str(r["supervisor"]), 0) + 1
    summary: dict = {
        "tool": "rtc_tools.analysis.catching_trials",
        "controller": profile.controller,
        "arm_device": profile.arm_device,
        "arm_joints": joints,
        "catch_frame_parent": profile.catch_frame.parent,
        "arm_base_frame": profile.arm_base_frame,
        "dt_s": dt,
        "dt_source": dt_source,
        "truth_time_axis": settings.truth_axis,
        "trials": len(rows),
        "accepted": len(run),
        "seeds": sorted({r["seed"] for r in rows if r.get("seed") is not None}),
        "validity": validity_block(rows, lane is not None),
        "supervisor_verdicts": verdicts,
        "servo_lag": [asdict(x) for x in lag],
        "medians": {
            k: _median(valid, k)
            for k in (
                "clik_mm",
                "servo_mm",
                "cmd_meas_gap_mm",
                "pred_mm",
                "ref_vs_true_mm",
                "total_mm",
                "arrival_ms",
                "gamma_f_planned",
                "first_plan_s",
                "contact_t_minus_tc_ms",
            )
        },
        "gamma_f_planned_range": _range(valid, "gamma_f_planned"),
        "approach_plan_switches_total": int(
            sum(r.get("approach_plan_switches", 0) for r in valid)
        ),
        "ref_saturated_max_streak": int(
            max((r.get("ref_saturated_max_streak", 0) for r in valid), default=0)
        ),
        "ref_saturated_streak": streak_distribution(valid),
        "g7b3": impulse_correlation(valid, settings.n_boot, settings.seed),
    }
    # Not a rig-failure rule (D-S8-16 ①): a valid trial whose truth file is
    # missing or belongs to another run counts as a failure — listed so a
    # pipeline fault is not read as a miss.
    summary["validity"]["valid_without_truth_file"] = [
        r["idx"] for r in valid if r.get("truth_file") is False
    ]
    if lane is not None:
        summary["validity"]["nominal_step_s"] = lane.nominal_step_s
        summary["validity"]["sim_stall_gap_factor"] = SIM_STALL_STEP_FACTOR
    if hold_radius is not None:
        summary["truth"] = {
            "hold_radius_m": hold_radius,
            **truth_block(valid, len(rows), settings.floor, settings.n_valid_target),
        }
    if lane is not None:
        d3: dict = {
            "clock_steady_offset_spread_ms": lane.offset_spread_s * 1e3,
            "dropped_total": lane.dropped_total,
            # Trials the lane has a launch for (the rest have no covariate and
            # are not counted as valid — see load_clock_lane).
            "paired": sum(1 for r in valid if r.get("launch_seq") is not None),
            # Over every accepted trial: an unpaired trial is lane_drop-invalid,
            # so a valid-only list would always be empty.
            "unpaired_trials": [r["idx"] for r in run if r.get("d3_unpaired")],
            "delta_max_ms_p50_p95_max": _p50_p95_max(valid, "delta_max_ms"),
            "delta_commit_ms_p50_p95_max": _p50_p95_max(valid, "delta_commit_ms", absolute=True),
            "delta_tc_ms_p50_p95_max": _p50_p95_max(valid, "delta_tc_ms", absolute=True),
        }
        if settings.v_max is not None:
            d3.update(v_max_m_s=settings.v_max, a_bound_m_s2=settings.a_bound)
            d3["valid"] = {
                f"{eps:g}mm": int(sum(1 for r in valid if r.get(f"d3_valid@{eps:g}mm")))
                for eps in settings.eps_mm
            }
        else:
            d3["valid"] = "NOT_EVALUATED(v_max not given)"
        summary["d3"] = d3
    if any("planner_cycles" in r for r in valid):
        with_cycles = [r for r in valid if r.get("planner_cycles", 0) > 0]
        ratios = [r["plan_valid_ratio"] for r in with_cycles if np.isfinite(r["plan_valid_ratio"])]
        switches: dict[str, int] = {}
        for r in valid:
            key = str(int(r.get("approach_plan_switches", 0)))
            switches[key] = switches.get(key, 0) + 1
        summary["g3d"] = {
            "n_trials_with_cycles": len(with_cycles),
            "plan_valid_ratio_p50_p05_p95": [
                float(np.median(ratios)),
                float(clock_phase.quantile(ratios, 0.05)),
                float(clock_phase.quantile(ratios, 0.95)),
            ]
            if ratios
            else None,
            "approach_plan_switches_distribution": switches,
        }
    if gate_map is not None:
        verdicted = [r for r in valid if r.get("map_open") is not None]
        open_rows = [r for r in verdicted if r["map_open"]]
        gm: dict = {
            "map_dir": str(gate_map.map_dir),
            "seed_id": gate_map.seed_id,
            "verdicted": len(verdicted),
            "open": len(open_rows),
            "open_fraction": len(open_rows) / len(verdicted) if verdicted else math.nan,
        }
        if hold_radius is not None:
            # Both over the VERDICTED rows: a reference/varied trial carries
            # no grid axes, so counting it in "whole" would compare the open
            # subset against a different throw population.
            k_whole = sum(1 for r in verdicted if r.get("truth_success"))
            k_open = sum(1 for r in open_rows if r.get("truth_success"))
            gm["truth_whole"] = {
                "successes": k_whole,
                "n": len(verdicted),
                "wilson95": wilson_interval(k_whole, len(verdicted)),
            }
            gm["truth_open"] = {
                "successes": k_open,
                "n": len(open_rows),
                "wilson95": wilson_interval(k_open, len(open_rows)),
            }
        else:
            gm["truth_whole"] = gm["truth_open"] = "NOT_EVALUATED(no hold radius)"
        summary["gate_map"] = gm
    return summary


def _range(rows, key):
    values = [r[key] for r in rows if key in r and np.isfinite(r[key])]
    return [float(min(values)), float(max(values))] if values else None


def _p50_p95_max(rows, key, absolute=False):
    values = [_num(r[key]) for r in rows if key in r]
    values = [abs(v) if absolute else v for v in values if np.isfinite(v)]
    if not values:
        return None
    return [
        float(np.median(values)),
        float(clock_phase.quantile(values, 0.95)),
        float(max(values)),
    ]


# ══ CLI ═══════════════════════════════════════════════════════════════════════


def _json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def _strict(value):
    """NaN / inf → null, so the summary is standard JSON."""
    if isinstance(value, Mapping):
        return {k: _strict(v) for k, v in value.items()}
    if isinstance(value, list | tuple):
        return [_strict(v) for v in value]
    if isinstance(value, float | np.floating) and not math.isfinite(value):
        return None
    return value


HAND_WINDOW_FIELDS = (
    "idx",
    "joint",
    "supervisor",
    "rho_lo",
    "rho_hi",
    "qd_absmax",
    "frac_lo",
    "n",
)


def write_outputs(result: SessionResult, out_dir: Path) -> tuple[Path, Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    keys: list[str] = []
    for row in result.rows:
        keys.extend(k for k in row if k not in keys)
    csv_path = out_dir / "catching_trials.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(result.rows)
    json_path = out_dir / "catching_trials_summary.json"
    json_path.write_text(
        json.dumps(_strict(result.summary), indent=2, default=_json_default, allow_nan=False)
        + "\n"
    )
    hand_csv_path = out_dir / "hand_hold_window.csv"
    with hand_csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=HAND_WINDOW_FIELDS)
        writer.writeheader()
        writer.writerows(result.hand_window_rows)
    return csv_path, json_path, hand_csv_path


def _fmt(value, spec: str = ".4f") -> str:
    return "n/a" if value is None or not np.isfinite(value) else format(value, spec)


def _fmt_ci(ci) -> str:
    return "[n/a]" if ci is None else f"[{_fmt(ci[0], '.3f')}, {_fmt(ci[1], '.3f')}]"


def validity_line(v: Mapping) -> str:
    reasons = " · ".join(f"{k} {n}" for k, n in v["n_invalid"].items())
    lane = "" if v["lane_rules_evaluated"] else " — lane rules NOT evaluated (no clock lane)"
    return (
        f"validity: n_total {v['n_total']} · n_valid {v['n_valid']} · invalid "
        f"{v['n_invalid_total']} ({reasons}){lane}"
    )


def verdict_line(tr: Mapping) -> str:
    itt = tr["itt"]
    out = (
        f"G8-D: p̂ {_fmt(tr['p_hat'], '.3f')} · lower_975 {_fmt(tr['lower_975'])} (z "
        f"{tr['z']:g}) · ITT {itt['successes']}/{itt['n']} lower_975 {_fmt(itt['lower_975'])}"
    )
    if "verdict" in tr:
        out += f" · floor {tr['floor']:g} → {tr['verdict']}"
    return out


def streak_line(c3: Mapping) -> str:
    return (
        f"G8-C3 (record only): ref_saturated streak > 0 in {c3['n_streak_gt0']}/"
        f"{c3['n_trials']} · p50/p95/p99/max {c3['p50']}/{c3['p95']}/{c3['p99']}/{c3['max']} · "
        f"histogram {c3['histogram']}"
    )


def b3_line(b3: Mapping) -> str:
    return (
        f"G7-B3: n {b3['n']} · OLS slope {_fmt(b3['ols_slope'], '.3f')} "
        f"{_fmt_ci(b3['ols_slope_ci95'])} · intercept {_fmt(b3['ols_intercept_ns'], '+.4f')} N·s · "
        f"Spearman ρ(Δp, impulse) {_fmt(b3['spearman_rho_impulse'], '.3f')} "
        f"(p {_fmt(b3['spearman_p_impulse'], '.2g')}), ρ(Δp, peak force) "
        f"{_fmt(b3['spearman_rho_peak_force'], '.3f')} (p {_fmt(b3['spearman_p_peak_force'], '.2g')})"
        f" · {b3['spearman_verdict']} · torque {b3['torque']}"
    )


def tick_line(tick) -> str:
    if not isinstance(tick, Mapping):
        return f"tick overrun: {tick}"
    period = tick.get("period_us")
    head = "tick overrun" + (f" (> {period:.0f} µs)" if period is not None else " (> the period)")
    return (
        f"{head}: {tick['n_trials_with_overrun']}"
        f"/{tick['n_trials']} trials, {tick['overrun_ticks_total']} ticks · per-trial p50/p95/max "
        f"{tick['tick_overrun_n_p50_p95_max']} · jitter max µs {tick['tick_jitter_max_us_p50_p95_max']}"
    )


def alignment_line(s: Mapping) -> str:
    d = s.get("time_alignment_detail", {})
    out = f"time alignment: {s.get('time_alignment')}"
    if d.get("sim_minus_t_rel_s") is not None:
        info = d.get("sim_minus_t_rel", {})
        out += (
            f" · sim − t_rel {d['sim_minus_t_rel_s']:.4f} s ({info.get('source')}, n "
            f"{info.get('n')}) · realtime − steady via {d.get('realtime_minus_steady_source')}"
            f" · stamp anchors {d.get('stamp_anchor')}"
        )
    elif "sim_minus_t_rel" in d:
        out += f" · sim − t_rel not estimated: {d['sim_minus_t_rel'].get('source')}"
    return out


def rtf_line(rtf) -> str:
    if not isinstance(rtf, Mapping):
        return f"RTF: {rtf}"
    parts = []
    for key, v in rtf.items():
        if v is not None:
            parts.append(f"{key} p05/p50/min {v['p05']:.2f}/{v['p50']:.2f}/{v['min']:.2f}")
    return "RTF (covariate): " + " · ".join(parts)


def g8b_lines(g8b) -> list[str]:
    if not isinstance(g8b, Mapping):
        return [f"G8-B: {g8b}"]
    out = []
    for lab, h in g8b["horizons"].items():
        if "mean_nees" not in h:
            out.append(f"G8-B h {lab} ms: {h['verdict']} (n {h['n_samples']})")
            continue
        bias = ", ".join(f"{v * 1e3:+.1f}" for v in h["bias_m"])
        out.append(
            f"G8-B h {lab} ms: mean NEES {h['mean_nees']:.3f} CI {_fmt_ci(h['ci95'])} → "
            f"{h['verdict']} · coverage_95 {h['coverage_95']:.3f} · bias [{bias}] mm · n "
            f"{h['n_samples']} over {h['n_trials']} trials, NaN {h['nan_share']:.1%}"
        )
    return out


def c2_line(c2) -> str:
    if not isinstance(c2, Mapping):
        return f"G8-C2: {c2}"
    ind = c2["independence"]
    verdict = ind if isinstance(ind, str) else ("PASS" if ind["passed"] else "FAIL")
    return (
        f"G8-C2: n {c2['n']} (exact {c2['n_exact']}, approx {c2['n_approx']}) · |A| "
        f"{c2['A_norm_mm_median']:.1f} mm · |B| {c2['B_norm_mm_median']:.1f} mm · E|A+B|² "
        f"{c2['E_A_plus_B_sq_mm2']:.0f} vs E|A|²+E|B|² {c2['E_A_sq_plus_E_B_sq_mm2']:.0f} mm² · "
        f"A⊥B {verdict}"
    )


def report(result: SessionResult) -> str:
    s = result.summary
    med = s["medians"]
    lines = [
        f"trials {s['trials']} (accepted {s['accepted']}), controller {s['controller']}, "
        f"dt {s['dt_s'] * 1e3:.3f} ms ({s['dt_source']})",
        f"supervisor verdicts: {s['supervisor_verdicts']}",
        "servo lag τ̂ (LS q_cmd−q_meas = τ q̇_meas, trial-bootstrap 95 % CI):",
    ]
    for x in result.lag:
        lines.append(
            f"  {x.joint:24s} {x.tau_s * 1e3:6.1f} ms  [{x.ci_low_s * 1e3:6.1f}, "
            f"{x.ci_high_s * 1e3:6.1f}]  R² {x.r2:.3f}  n {x.n_ticks}"
        )
    lines.append(
        f"t_c medians [mm] (T_lead {s.get('t_lead_s_range')} s, {s.get('t_lead_source')}): "
        f"ref vs truth {med['ref_vs_true_mm']:.1f} · CLIK {med['clik_mm']:.1f} · servo "
        f"{med['servo_mm']:.1f} (same-tick cmd–meas gap {med['cmd_meas_gap_mm']:.1f}) · "
        f"pred {med['pred_mm']:.1f} · total "
        f"{med['total_mm']:.1f}; arrival − t_c "
        f"{med['arrival_ms']:+.1f} ms; first hand contact − t_c "
        f"{med['contact_t_minus_tc_ms']:+.1f} ms"
    )
    lines.append(
        f"planned γ_f {s['gamma_f_planned_range']} · first plan {med['first_plan_s']:.3f} s · "
        f"APPROACH plan switches {s['approach_plan_switches_total']} · ref_saturated max "
        f"streak {s['ref_saturated_max_streak']}"
    )
    lines.append(alignment_line(s))
    lines.append(rtf_line(s.get("rtf")))
    lines.append(validity_line(s["validity"]))
    if s["validity"].get("valid_without_truth_file"):
        lines.append(
            f"  valid trials WITHOUT a truth file (counted as failures): "
            f"{s['validity']['valid_without_truth_file']}"
        )
    if "truth" in s:
        tr = s["truth"]
        lo, hi = tr["wilson95"]
        lines.append(
            f"truth success {tr['successes']}/{tr['n']} valid (Wilson 95 % [{lo:.2f}, {hi:.2f}], "
            f"hold radius {tr['hold_radius_m'] * 1e3:.1f} mm); supervisor × truth "
            f"{tr['confusion_supervisor_vs_truth']}"
        )
        lines.append(verdict_line(tr))
    lines.append(streak_line(s["ref_saturated_streak"]))
    lines.append(b3_line(s["g7b3"]))
    lines.append(tick_line(s.get("tick_overrun")))
    if "g8b" in s:
        lines.extend(g8b_lines(s["g8b"]))
    if "g8b_unrestricted" in s:
        lines.append(
            f"G8-B unrestricted (eval_report, post-contact included): {s['g8b_unrestricted']}"
        )
    if "c2" in s:
        lines.append(c2_line(s["c2"]))
    if "d3" in s:
        d3 = s["d3"]
        lines.append(
            f"D-3: δ_max p50/p95/max {d3['delta_max_ms_p50_p95_max']} ms · |δ(t_commit)| "
            f"{d3['delta_commit_ms_p50_p95_max']} · |δ(t_c)| {d3['delta_tc_ms_p50_p95_max']} · "
            f"valid {d3['valid']} (a covariate, not a verdict — D-S8-4 (c))"
        )
    if "g3d" in s:
        g3d = s["g3d"]
        lines.append(
            f"G3-D: plan validity ratio p50/p05/p95 {g3d['plan_valid_ratio_p50_p05_p95']} over "
            f"{g3d['n_trials_with_cycles']} trials with ≥1 planner cycle · APPROACH plan "
            f"switches distribution {g3d['approach_plan_switches_distribution']}"
        )
    if "gate_map" in s:
        gm = s["gate_map"]
        lines.append(
            f"gate map {gm['map_dir']} (seed {gm['seed_id']}): {gm['open']}/{gm['verdicted']} "
            f"trials map-open ({gm['open_fraction']:.2f}) · truth whole {gm['truth_whole']} · "
            f"truth open {gm['truth_open']}"
        )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("session", type=Path, help="controller session dir (logging_data/<stamp>)")
    ap.add_argument("trials_dir", type=Path, help="catching_sim_trials output dir")
    ap.add_argument("--config-dir", type=Path, required=True, help="robot profile config dir")
    ap.add_argument("--controller", help="catching controller key (default: the only one)")
    ap.add_argument("--catch-frame", default=DEFAULT_CATCH_FRAME)
    ap.add_argument("--urdf", type=Path, help="expanded URDF (default: urdf.package/path)")
    ap.add_argument("--out", type=Path, help="output dir (default: <trials_dir>/catching_trials)")
    ap.add_argument(
        "--clock-lane", type=Path, help="clock lane CSV (default: <session>/sim/clock_lane.csv)"
    )
    ap.add_argument(
        "--contact-lane",
        type=Path,
        help="ball contact lane CSV (default: <session>/sim/ball_contact_lane.csv)",
    )
    ap.add_argument("--truth-time", choices=TRUTH_TIME_AXES, default="stamp")
    ap.add_argument(
        "--hold-radius-m",
        type=float,
        help="truth 'in the hand' radius around the catch frame (default: the profile's ball "
        "diameter, catching.core.ball.diameter)",
    )
    ap.add_argument(
        "--v-max",
        type=float,
        help="D-3: fastest catch speed of the target distribution [m/s] (plan §5). No "
        "default — without it the validity is NOT_EVALUATED",
    )
    ap.add_argument(
        "--a-bound",
        type=float,
        default=clock_phase.DEFAULT_A_BOUND_M_S2,
        help="D-3 acceleration bound [m/s²] (default: clock_phase's g + drag ceiling)",
    )
    ap.add_argument("--eps-mm", type=float, nargs="*", default=[], help="D-3 ε_clk,alloc [mm]")
    ap.add_argument("--n-boot", type=int, default=DEFAULT_N_BOOT)
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED)
    ap.add_argument(
        "--hold-window-s",
        type=float,
        default=DEFAULT_HOLD_WINDOW_S,
        help="hand-joint capture calibration: window before t_hold_end [s] "
        f"(default {DEFAULT_HOLD_WINDOW_S})",
    )
    ap.add_argument(
        "--gate-map",
        type=Path,
        help="a catch_gate_map output dir (S8-D): per-trial box-open verdict against its "
        "catchability_map grid, and truth success over the map-open subset. Only trials drawn "
        "from a --dist box (catching_sim_trials) carry the axis values a verdict needs",
    )
    ap.add_argument(
        "--floor",
        type=float,
        help="G8-D floor (D-S8-3): truth.verdict PASS when the valid trials' Wilson lower bound "
        "(z 1.96, the 97.5 %% one-sided bound) is ≥ this. No default — without it no verdict",
    )
    ap.add_argument(
        "--n-valid-target",
        type=int,
        help="the verdict is INSUFFICIENT_N while n_valid is below this (D-S8-3: 200)",
    )
    ap.add_argument(
        "--eval-samples",
        type=Path,
        help="G8-B: sim_capture_evaluate eval_samples.csv of this session's capture",
    )
    ap.add_argument(
        "--eval-report",
        type=Path,
        help="G8-B: its eval_report.json — echoed as the unrestricted (post-contact "
        "included) reference, not judged",
    )
    ap.add_argument(
        "--probe-dump",
        type=Path,
        help="G8-C2: vision_lane_probe --dump lane_prediction_dump.csv of this session",
    )
    args = ap.parse_args(argv)

    from rtc_tools.analysis.derive_accel_limits import resolve_urdf_text  # noqa: PLC0415

    profile = load_profile(args.config_dir, args.controller, args.session, args.catch_frame)
    urdf_text, _ = resolve_urdf_text(profile.robot_params, args.urdf)
    clock = args.clock_lane or _exists(args.session / "sim" / "clock_lane.csv")
    contact = args.contact_lane or _exists(args.session / "sim" / "ball_contact_lane.csv")
    gate_map = load_gate_map(args.gate_map) if args.gate_map else None
    settings = Settings(
        truth_axis=args.truth_time,
        hold_radius_m=args.hold_radius_m,
        v_max=args.v_max,
        a_bound=args.a_bound,
        eps_mm=tuple(args.eps_mm),
        n_boot=args.n_boot,
        seed=args.seed,
        hold_window_s=args.hold_window_s,
        floor=args.floor,
        n_valid_target=args.n_valid_target,
    )
    result = analyse_session(
        args.session,
        args.trials_dir,
        profile,
        urdf_text,
        settings,
        clock,
        contact,
        gate_map,
        eval_samples_path=args.eval_samples,
        eval_report_path=args.eval_report,
        probe_dump_path=args.probe_dump,
    )
    csv_path, json_path, hand_csv_path = write_outputs(
        result, args.out or args.trials_dir / "catching_trials"
    )
    print(report(result))
    print(f"\n-> {csv_path}\n-> {json_path}\n-> {hand_csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
