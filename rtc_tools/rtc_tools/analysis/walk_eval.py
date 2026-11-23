"""Evaluate walking runs: tiered metrics, run validity, arm statistics and a ranking cost.

A run is one ``run_arms.py`` output directory (``run.json``, ``launch.log``,
``stance.csv`` and the controller's log session); an arm is the runs of one
configuration; a batch is the arms compared with each other. This module turns
a batch into numbers a tuning decision can cite, and says which runs it may not
cite.

Tiers
    The controller writes two files. ``locomotion_diag.csv`` (R) is what a real
    robot also has; ``locomotion_truth.csv`` (T) is the simulator's ground truth
    and exists only when a truth lane was live. A metric names the tier it is
    computed from as its key's suffix:

    ``<name>_truth``  from T (base pose / twist, contact wrench)
    ``<name>_est``    from R's state-source columns (``base_*``) — the state the
                      controller ran on, whatever produced it
    ``<name>``        from R's command, plan and solver columns only

    A key whose tier the metric does not declare, or whose file the run does not
    have, is refused (:class:`MetricUnavailable`) — never computed from the
    other tier under the asked name. A run on the truth lane has
    ``<name>_est`` ≈ ``<name>_truth`` (R holds the normalised copy); that is a
    check, not a reason to rename one into the other.

Run metrics (:data:`METRICS`)
    Every window is placed by the log's own command columns and the command
    script (:data:`walk_metrics.COMMAND_SCRIPTS`), so nothing here holds a robot
    name or a command value: "walking straight" is the planar command, along
    the script's own direction (forward, backward or sideways), at ≥ 90 % of the
    script's forward speed with no yaw command, a transition is the command
    crossing half its script value.

    * the run ends at the first row with a fault cause (``end``); it *completed*
      if there is none. ``survival_s`` runs from the first lift-off to ``end``.
    * ``touchdowns`` counts the rows from the first lift-off to ``end`` on which
      a planned contact comes on.
    * ``v_mps`` is the base's displacement along its own heading over the
      straight window (from 1 s after it starts to its end, or to 0.5 s before
      the run's last counted row if that is earlier; at least 2 s), divided by
      the window; ``v_ratio`` divides that
      by the mean commanded speed. ``v_ratio`` is a ratio, not a speed.
    * a fault is *straight* before the first turn command and a *transition*
      fault within two touchdowns after a command transition.
    * the *foot gap* is the two contact frames' separation (R's ``foot_<c>_x/y``,
      the state source's forward kinematics) across the base's heading, signed
      so that the stand before the command reads positive — crossed feet read
      negative. ``foot_gap_min_est`` is its least value over the walking rows
      and ``foot_gap_narrow_fraction_est`` the share of those rows on which it
      is under the batch spec's ``foot_width_m`` (the sole's width: a gap under
      it lets the soles overlap). It needs exactly two contacts and stand rows
      before the command; the fraction also needs ``foot_width_m``. It reads the
      separation across the heading only, so a fore-aft offset that keeps the
      soles apart still counts as narrow.

Validity (:func:`run_validity`)
    A run counts only if the recorder exited cleanly, the simulator held the
    asked real-time factor, the command had the registered shape, the gait's
    single support had the registered length, the run carried the arm's overlay
    and — for runs that record it — the build, the installed configuration and
    the plant model are consistent: a clean checkout, an installed
    configuration equal to it, a plant that matches the controller's model
    (what it cannot show: that the installed binaries were built from that
    commit). A log the numbers cannot be trusted on is invalid too — cells
    that did not parse in the columns every window is placed by, or a log
    that ends before the script does without a fault. Invalid runs are listed
    with the check that failed, and a run that cannot be read at all is
    listed with its error; neither stops the batch.

Arm statistics
    Counts and medians over the valid runs (:func:`arm_summary`), the Wilson
    interval of the completion rate, and a one-sided Fisher test that only ever
    *rejects* a candidate for completing less often than the control
    (:func:`compare_completion`): at the arm sizes in use a passed test is not
    evidence of no difference, and the power at the batch's size is reported
    next to it (:func:`binomial_stats.fisher_power`).

Ranking cost (:func:`j_eval`)
    ``J = Σ β · error / scale`` over forward speed, yaw rate, lateral drift,
    tilt and contact normal force, every scale a fixed physical quantity
    (:data:`J_TERMS`) so a batch's own control does not move it. It orders
    candidates inside a grid; adoption is by the per-metric rules. A run that
    faulted has no J, and neither has one without the truth tier — a missing
    term is not treated as zero. The constraint vector (:func:`constraints`) is
    reported beside J and never folded into it.

Pure Python plus numpy and yaml.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis import locomotion_diag as ld, walk_metrics as wm
from rtc_tools.analysis.binomial_stats import fisher_one_sided, fisher_power, wilson_interval

Table = dict[str, np.ndarray]

TIER_TRUTH = "truth"
TIER_EST = "est"
TIER_NONE = ""
TIERS = (TIER_TRUTH, TIER_EST, TIER_NONE)

TRUTH_FILE = "locomotion_truth.csv"
TRUTH_BASE_BIT = 0  # truth_mask bit of the base lane; bit 1 + c is contact c

# Window rules (module docstring), in script fractions and seconds.
STRAIGHT_FRACTION = 0.9
YAW_ZERO = 0.01  # [rad/s] |cmd_wz| below this is "no yaw command"
CROSS_FRACTION = 0.5
CMD_ALONG = "cmd_along"  # not a log column: Run.cmd_along
SPEED_LEAD_S = 1.0
SPEED_FAULT_MARGIN_S = 0.5
SPEED_MIN_WINDOW_S = 2.0
RATE_WINDOW_S = 0.2
RATE_SPAN_S = 1.0

# mpc_last_outcome codes (rtc_mpc thread/solve_record.hpp).
OUTCOME_WITHHELD = (1, 3)
OUTCOME_DEADLINE = 2

SIM_TIME_LINE = re.compile(r"\[INFO\] \[(\d+\.\d+)\] \[mujoco_simulator\].*sim_time=([\d.]+) s")


class MetricUnavailable(ValueError):
    """The run cannot give this metric: its tier's file or a column it reads is missing."""


# ── a run's logs ──────────────────────────────────────────────────────────


@dataclass
class RunLog:
    header: list[str]
    diag: Table
    contacts: list[str]
    truth: Table | None = None  # T columns on the diag rows (NaN = no T row); None = no T file

    def column(self, name: str) -> np.ndarray:
        if name not in self.diag:
            raise MetricUnavailable(f"locomotion_diag.csv has no column {name}")
        return self.diag[name]

    def truth_column(self, name: str) -> np.ndarray:
        if self.truth is None:
            raise MetricUnavailable(f"the run has no {TRUTH_FILE} (truth tier)")
        if name not in self.truth:
            raise MetricUnavailable(f"{TRUTH_FILE} has no column {name}")
        return self.truth[name]

    def base(self, tier: str) -> Table:
        """px … wz of the base on the diag rows, from the tier's file."""
        keys = [
            f"{a}{b}"
            for a, axes in (("p", "xyz"), ("q", "xyzw"), ("v", "xyz"), ("w", "xyz"))
            for b in axes
        ]
        if tier == TIER_EST:
            return {k: self.column(f"base_{k}") for k in keys}
        if tier == TIER_TRUTH:
            return {k: self.truth_column(f"truth_base_{k}") for k in keys}
        raise MetricUnavailable("a base metric needs a tier (est or truth)")


def load_run(controller_dir: Path) -> RunLog:
    """The controller's R file, and its T file on the same rows when there is one."""
    header, diag = ld.read_table(controller_dir / ld.DIAG_FILE)
    contacts = ld.parse_diag_names(header).contacts
    truth_path = controller_dir / TRUTH_FILE
    truth = None
    if truth_path.is_file():
        _, raw = ld.read_table(truth_path)
        truth = ld.align_on_tick(diag["tick"], raw)
    return RunLog(header=header, diag=diag, contacts=contacts, truth=truth)


# ── the windows of one run ────────────────────────────────────────────────


def _first_crossing(t: np.ndarray, x: np.ndarray, level: float, rising: bool) -> float | None:
    if rising:
        hit = np.flatnonzero((x[1:] > level) & (x[:-1] <= level))
    else:
        hit = np.flatnonzero((x[1:] < level) & (x[:-1] >= level))
    return float(t[hit[0] + 1]) if hit.size else None


@dataclass
class Run:
    """One run's log with the windows every metric shares."""

    log: RunLog
    script: Sequence[wm.Segment]
    foot_width_m: float | None = None  # the batch spec's sole width (foot gap metrics)

    @cached_property
    def t(self) -> np.ndarray:
        return self.log.column("t_relative_s")

    @cached_property
    def dt(self) -> float:
        return float(np.median(np.diff(self.t)))

    @cached_property
    def _fastest(self) -> tuple[float, float]:
        """(v_x, v_y) of the script's fastest planar command (a segment end)."""
        ends = [(s.vx0, s.vy0) for s in self.script] + [(s.vx1, s.vy1) for s in self.script]
        return max(ends, key=lambda v: math.hypot(*v))

    @cached_property
    def v_cmd(self) -> float:
        """The script's planar speed [m/s]: forward, backward or sideways alike."""
        return math.hypot(*self._fastest)

    @cached_property
    def direction(self) -> tuple[float, float]:
        """Unit vector of the script's planar command on the heading-aligned axes."""
        return (
            (self._fastest[0] / self.v_cmd, self._fastest[1] / self.v_cmd)
            if self.v_cmd
            else (1.0, 0.0)
        )

    @cached_property
    def cmd_along(self) -> np.ndarray:
        """The planar command along the script's direction, per row [m/s]."""
        cx, cy = self.direction
        return self.log.column("cmd_vx") * cx + self.log.column("cmd_vy") * cy

    @cached_property
    def wz_cmd(self) -> float:
        return max(abs(s.wz) for s in self.script)

    @cached_property
    def mask(self) -> np.ndarray:
        m = self.log.column("contact_mask")
        return np.where(np.isfinite(m), m, 0).astype(np.int64)

    @cached_property
    def full_mask(self) -> int:
        if not self.log.contacts:
            raise MetricUnavailable("the log names no contact (no lambda_<c>_fz column)")
        return (1 << len(self.log.contacts)) - 1

    @cached_property
    def fault_rows(self) -> np.ndarray:
        return np.flatnonzero(self.log.column("fault_cause") > 0)

    @property
    def faulted(self) -> bool:
        return bool(self.fault_rows.size)

    @cached_property
    def end(self) -> int:
        """The last row that counts: the first fault row, else the log's last."""
        return int(self.fault_rows[0]) if self.faulted else len(self.t) - 1

    @cached_property
    def fault_cause(self) -> str | None:
        if not self.faulted:
            return None
        code = int(self.log.column("fault_cause")[self.end])
        names = ld.FAULT_CAUSE_NAMES
        return names[code] if 0 <= code < len(names) else str(code)

    @cached_property
    def lift(self) -> int | None:
        """The first row on which a contact of the full stance is planned off."""
        # full_mask refuses a log that names no contact.
        return wm.first_lift_row(self.mask, self.full_mask.bit_length())

    @cached_property
    def touchdown_rows(self) -> np.ndarray:
        """Rows on which a planned contact comes on (the whole log)."""
        m = self.mask
        return np.flatnonzero(((m[1:] & ~m[:-1]) & self.full_mask) != 0) + 1

    @cached_property
    def crossings(self) -> dict[str, float | None]:
        """The command transitions: the yaw command up / down, the planar command down."""
        t = self.t
        wz, vx = self.log.column("cmd_wz"), self.cmd_along
        out: dict[str, float | None] = {"turn_start": None, "turn_end": None, "stop": None}
        if self.wz_cmd > 0.0:
            level = CROSS_FRACTION * self.wz_cmd
            out["turn_start"] = _first_crossing(t, wz, level, rising=True)
            out["turn_end"] = _first_crossing(t, wz, level, rising=False)
        level = CROSS_FRACTION * self.v_cmd
        walking = np.flatnonzero(vx > level)
        if walking.size:
            i0 = int(walking[0])
            stop = _first_crossing(t[i0:], vx[i0:], level, rising=False)
            out["stop"] = stop
        return out

    @cached_property
    def straight(self) -> tuple[int, int] | None:
        """[first, last] row of the first straight-walking stretch of the command."""
        if self.v_cmd <= 0.0:
            return None  # a script with no forward speed has no straight stretch
        ok = (self.cmd_along >= STRAIGHT_FRACTION * self.v_cmd) & (
            np.abs(self.log.column("cmd_wz")) <= YAW_ZERO
        )
        if not ok.any():
            return None
        i0 = int(np.argmax(ok))
        stops = np.flatnonzero(~ok[i0:])
        i1 = i0 + int(stops[0]) - 1 if stops.size else len(ok) - 1
        return i0, i1

    @cached_property
    def speed_window(self) -> tuple[float, float] | None:
        """[t0, t1] the forward speed is measured over; None if it is under the minimum."""
        if self.straight is None:
            return None
        i0, i1 = self.straight
        t0 = self.t[i0] + SPEED_LEAD_S
        t1 = min(self.t[i1], self.t[self.end] - SPEED_FAULT_MARGIN_S)
        return (float(t0), float(t1)) if t1 - t0 >= SPEED_MIN_WINDOW_S else None

    def walking_rows(self) -> np.ndarray:
        """Rows up to `end` on which the command is at the script's speed.

        The forward command where the script has one, else its yaw command (a
        turn in place).
        """
        if self.v_cmd > 0.0:
            rows = self.cmd_along >= STRAIGHT_FRACTION * self.v_cmd
        else:
            rows = np.abs(self.log.column("cmd_wz")) >= STRAIGHT_FRACTION * self.wz_cmd
        rows[self.end + 1 :] = False
        return rows


# ── metrics ───────────────────────────────────────────────────────────────


def _completed(run: Run, _tier: str) -> bool:
    return run.lift is not None and not run.faulted


def _survival(run: Run, _tier: str) -> float:
    return 0.0 if run.lift is None else float(run.t[run.end] - run.t[run.lift])


def _touchdowns(run: Run, _tier: str) -> int:
    if run.lift is None:
        return 0
    rows = run.touchdown_rows
    return int(((rows > run.lift) & (rows <= run.end)).sum())


def _fault_cause(run: Run, _tier: str) -> str | None:
    return run.fault_cause


def _plan_loss(run: Run, _tier: str) -> bool:
    return run.fault_cause == ld.FAULT_CAUSE_NAMES[wm.FAULT_CAUSE_PLAN_LOSS]


def _exposed(run: Run, _tier: str) -> bool:
    """The run reached the first turn command."""
    start = run.crossings["turn_start"]
    return start is not None and (not run.faulted or run.t[run.end] > start)


def _straight_fault(run: Run, _tier: str) -> bool:
    if not run.faulted:
        return False
    start = run.crossings["turn_start"]
    return start is None or bool(run.t[run.end] < start)


def _transition_fault(run: Run, _tier: str) -> bool:
    if not run.faulted:
        return False
    t_fault = run.t[run.end]
    t_down = run.t[run.touchdown_rows]
    for t_cross in run.crossings.values():
        if t_cross is None or t_fault < t_cross:
            continue
        after = t_down[t_down > t_cross]
        if t_fault <= (after[1] if after.size >= 2 else math.inf):
            return True
    return False


def single_supports(run: Run) -> list[float]:
    """Lengths [s] of the single supports after the second touchdown that end before `end`."""
    m, t, end = run.mask, run.t, run.end
    downs = run.touchdown_rows[run.touchdown_rows <= end]
    out: list[float] = []
    if downs.size < 2:
        return out
    one = np.array([bin(int(v)).count("1") == 1 for v in m])
    i = int(downs[1])
    while i < end:
        if one[i]:
            j = i
            while j < end and m[j] == m[i]:
                j += 1
            if j < end:
                out.append(float(t[j] - t[i]))
            i = j
        else:
            i += 1
    return out


def _single_support(run: Run, _tier: str) -> float | None:
    lengths = single_supports(run)
    return float(np.median(lengths)) if lengths else None


def command_rate(run: Run, column: str, t_cross: float | None) -> float | None:
    """Largest change of the command over RATE_WINDOW_S within ±RATE_SPAN_S of a transition."""
    if t_cross is None:
        return None
    t, x = run.t, (run.cmd_along if column == CMD_ALONG else run.log.column(column))
    n = int(round(RATE_WINDOW_S / run.dt))
    a = int(np.searchsorted(t, t_cross - RATE_SPAN_S))
    b = min(len(x) - n, int(np.searchsorted(t, t_cross + RATE_SPAN_S)))
    if b <= a:
        return None
    return float(np.max(np.abs(x[a + n : b + n] - x[a:b])) / RATE_WINDOW_S)


def _rate(event: str, column: str) -> Callable[[Run, str], float | None]:
    return lambda run, _tier: command_rate(run, column, run.crossings[event])


def _commanded(run: Run) -> np.ndarray:
    """Rows on which the script commands motion (above 1 % of its speeds)."""
    command = np.abs(run.cmd_along) + np.abs(run.log.column("cmd_wz"))
    return command > 0.01 * (run.v_cmd + run.wz_cmd)


def _yaw(base: Table) -> np.ndarray:
    return wm.yaw(base["qx"], base["qy"], base["qz"], base["qw"])


def planar_motion(run: Run, tier: str) -> tuple[float, float, float, float] | None:
    """(along [m], across [m], window [s], mean command [m/s]) over the speed window.

    Displacement row to row, resolved on the heading of the earlier row and
    then on the script's command direction (forward for a forward walk, so
    "along" is the forward advance there), over the rows that have a base (a
    truth lane can skip rows; the step across the gap still counts).
    """
    base = run.log.base(tier)
    if run.speed_window is None or run.lift is None:
        return None
    t0, t1 = run.speed_window
    rows = np.flatnonzero((run.t >= t0) & (run.t <= t1) & np.isfinite(base["px"] + base["qw"]))
    if rows.size < 2:
        return None
    psi = _yaw(base)[rows[:-1]]
    dx, dy = np.diff(base["px"][rows]), np.diff(base["py"][rows])
    ahead = dx * np.cos(psi) + dy * np.sin(psi)
    left = -dx * np.sin(psi) + dy * np.cos(psi)
    cx, cy = run.direction
    forward = float(np.sum(ahead * cx + left * cy))
    lateral = float(np.sum(left * cx - ahead * cy))
    span = float(run.t[rows[-1]] - run.t[rows[0]])
    return forward, lateral, span, float(np.mean(run.cmd_along[rows]))


def _v_mps(run: Run, tier: str) -> float | None:
    m = planar_motion(run, tier)
    return None if m is None else m[0] / m[2]


def _v_ratio(run: Run, tier: str) -> float | None:
    m = planar_motion(run, tier)
    return None if m is None else m[0] / m[2] / m[3]


def _lateral_mps(run: Run, tier: str) -> float | None:
    m = planar_motion(run, tier)
    return None if m is None else m[1] / m[2]


def _lateral_drift(run: Run, tier: str) -> float | None:
    m = planar_motion(run, tier)
    return None if m is None else abs(m[1])


def _yaw_rate_error(run: Run, tier: str) -> float | None:
    """|mean commanded − mean measured| world yaw rate between the turn's two transitions."""
    start, stop = run.crossings["turn_start"], run.crossings["turn_end"]
    if start is None or stop is None:
        return None
    base = run.log.base(tier)
    rows = (run.t >= start) & (run.t < stop)
    rows[run.end + 1 :] = False
    _, _, wz = wm.planar_velocity(
        tuple(base[f"q{a}"] for a in "xyzw"),
        tuple(base[f"v{a}"] for a in "xyz"),
        tuple(base[f"w{a}"] for a in "xyz"),
    )
    rows &= np.isfinite(wz)
    if not rows.any():
        return None
    return abs(float(np.mean(run.log.column("cmd_wz")[rows]) - np.mean(wz[rows])))


def _tilt_rms(run: Run, tier: str) -> float | None:
    base = run.log.base(tier)
    tilt = ld.tilt_rad(base["qx"], base["qy"])[run.walking_rows()]
    tilt = tilt[np.isfinite(tilt)]
    return float(np.sqrt(np.mean(tilt * tilt))) if tilt.size else None


def contact_valid(run: Run) -> np.ndarray:
    """(rows, contacts) — the truth contact lane was fresh on that row."""
    mask = run.log.truth_column("truth_mask")
    bits = wm.mask_bits(mask, len(run.log.contacts) + 1)
    return bits[:, 1:]


def truth_normal_force(run: Run) -> np.ndarray:
    """(rows, contacts) ground → foot normal force [N]; NaN where the lane was not fresh."""
    fz = np.stack([run.log.truth_column(f"truth_grf_{c}_fz") for c in run.log.contacts], axis=1)
    return np.where(contact_valid(run), fz, math.nan)


def _weight(run: Run, _tier: str) -> float | None:
    """The standing load [N]: the summed normal force over the stand rows before the command."""
    fz = truth_normal_force(run)
    rows = _stand_before_command(run) & np.isfinite(fz).all(axis=1)
    return float(np.mean(fz[rows].sum(axis=1))) if rows.any() else None


def _contact_fz_error(run: Run, tier: str) -> float | None:
    """RMS of (planned − true) contact normal force over the walking rows, in body weights.

    The plan's force is on the world z axis and the truth on the contact
    frame's; they are the same axis while the foot is flat on level ground.
    """
    weight = _weight(run, tier)
    if weight is None or weight <= 0.0:
        return None
    true = truth_normal_force(run)
    plan = np.stack([run.log.column(f"wrench_des_{c}_fz") for c in run.log.contacts], axis=1)
    err = (plan - true)[run.walking_rows()]
    err = err[np.isfinite(err)]
    return float(np.sqrt(np.mean(err * err)) / weight) if err.size else None


def _stand_before_command(run: Run) -> np.ndarray:
    """Rows before the first commanded row on which the reference is the stand."""
    moving = np.flatnonzero(_commanded(run))
    before = np.arange(len(run.t)) < (moving[0] if moving.size else len(run.t))
    return before & (run.log.column("ref_source") == wm.REF_SOURCE_STAND)


def foot_gap(run: Run, tier: str) -> np.ndarray:
    """Per row, the foot separation across the base heading [m] (module docstring).

    Positive in the stand before the command; NaN where a column is not finite.
    """
    contacts = run.log.contacts
    if len(contacts) != 2:
        raise MetricUnavailable(
            f"the foot gap needs exactly two contacts (the log has {contacts})"
        )
    base = run.log.base(tier)
    a, b = contacts
    dx = run.log.column(f"foot_{a}_x") - run.log.column(f"foot_{b}_x")
    dy = run.log.column(f"foot_{a}_y") - run.log.column(f"foot_{b}_y")
    psi = _yaw(base)
    across = -np.sin(psi) * dx + np.cos(psi) * dy
    stand = across[_stand_before_command(run) & np.isfinite(across)]
    if not stand.size:
        raise MetricUnavailable("no stand rows before the command to orient the foot gap")
    side = float(np.median(stand))
    if side == 0.0:
        raise MetricUnavailable("the feet stand on one line across the heading")
    return across * math.copysign(1.0, side)


def _walking_gap(run: Run, tier: str) -> np.ndarray:
    gap = foot_gap(run, tier)[run.walking_rows()]
    return gap[np.isfinite(gap)]


def _foot_gap_min(run: Run, tier: str) -> float | None:
    gap = _walking_gap(run, tier)
    return float(gap.min()) if gap.size else None


def _foot_gap_narrow(run: Run, tier: str) -> float | None:
    if run.foot_width_m is None:
        raise MetricUnavailable("the batch spec names no foot_width_m")
    gap = _walking_gap(run, tier)
    return float((gap < run.foot_width_m).mean()) if gap.size else None


def _solve_records(run: Run) -> np.ndarray:
    """mpc_last_outcome of every distinct solve record up to `end`."""
    seq = run.log.column("mpc_last_seq")[: run.end + 1]
    outcome = run.log.column("mpc_last_outcome")[: run.end + 1]
    new = np.flatnonzero(np.diff(seq, prepend=0.0) > 0)
    return outcome[new]


def _withheld_rate(run: Run, _tier: str) -> float | None:
    rec = _solve_records(run)
    return float(np.isin(rec, OUTCOME_WITHHELD).mean()) if rec.size else None


def _deadline_rate(run: Run, _tier: str) -> float | None:
    rec = _solve_records(run)
    return float((rec == OUTCOME_DEADLINE).mean()) if rec.size else None


def _increase(column: str) -> Callable[[Run, str], int]:
    def fn(run: Run, _tier: str) -> int:
        col = run.log.column(column)[: run.end + 1]
        col = col[np.isfinite(col)]
        return int(col[-1] - col[0]) if col.size else 0

    return fn


def _qp_infeasible(run: Run, _tier: str) -> int:
    status = run.log.column("qp_status")[: run.end + 1]
    return int(np.isin(status, wm.QP_INFEASIBLE_STATUSES).sum())


def _margin_min(column: str) -> Callable[[Run, str], float | None]:
    def fn(run: Run, _tier: str) -> float | None:
        col = run.log.column(column)[: run.end + 1].copy()
        col[~(run.log.column("margins_stale")[: run.end + 1] < 0.5)] = math.nan
        col = col[run.walking_rows()[: run.end + 1] & np.isfinite(col)]
        return float(col.min()) if col.size else None

    return fn


@dataclass(frozen=True)
class Metric:
    tiers: tuple[str, ...]
    unit: str
    fn: Callable[[Run, str], object]


_BASE = (TIER_EST, TIER_TRUTH)
_PLAN = (TIER_NONE,)
_TRUTH = (TIER_TRUTH,)
_EST = (TIER_EST,)  # R's forward kinematics: the state source's, there is no truth copy

METRICS: dict[str, Metric] = {
    "completed": Metric(_PLAN, "bool", _completed),
    "survival_s": Metric(_PLAN, "s", _survival),
    "touchdowns": Metric(_PLAN, "count", _touchdowns),
    "fault_cause": Metric(_PLAN, "name", _fault_cause),
    "plan_loss": Metric(_PLAN, "bool", _plan_loss),
    "exposed": Metric(_PLAN, "bool", _exposed),
    "straight_fault": Metric(_PLAN, "bool", _straight_fault),
    "transition_fault": Metric(_PLAN, "bool", _transition_fault),
    "single_support_s": Metric(_PLAN, "s", _single_support),
    "rate_turn_start": Metric(_PLAN, "rad/s^2", _rate("turn_start", "cmd_wz")),
    "rate_turn_end": Metric(_PLAN, "rad/s^2", _rate("turn_end", "cmd_wz")),
    "rate_stop": Metric(_PLAN, "m/s^2", _rate("stop", CMD_ALONG)),
    "withheld_rate": Metric(_PLAN, "fraction", _withheld_rate),
    "deadline_rate": Metric(_PLAN, "fraction", _deadline_rate),
    "plan_holds": Metric(_PLAN, "count", _increase("plan_holds")),
    "plausibility_rejects": Metric(_PLAN, "count", _increase("plausibility_rejects")),
    "qp_infeasible_rows": Metric(_PLAN, "count", _qp_infeasible),
    "margin_friction_min": Metric(_PLAN, "N", _margin_min("margin_friction_n")),
    "margin_cop_min": Metric(_PLAN, "N m", _margin_min("margin_cop_nm")),
    "margin_torque_min": Metric(_PLAN, "N m", _margin_min("margin_torque_nm")),
    "v_mps": Metric(_BASE, "m/s", _v_mps),
    "v_ratio": Metric(_BASE, "ratio", _v_ratio),
    "lateral_mps": Metric(_BASE, "m/s", _lateral_mps),
    "lateral_drift_m": Metric(_BASE, "m", _lateral_drift),
    "yaw_rate_error": Metric(_BASE, "rad/s", _yaw_rate_error),
    "tilt_rms": Metric(_BASE, "rad", _tilt_rms),
    "weight_n": Metric(_TRUTH, "N", _weight),
    "contact_fz_error": Metric(_TRUTH, "body weight", _contact_fz_error),
    "foot_gap_min": Metric(_EST, "m", _foot_gap_min),
    "foot_gap_narrow_fraction": Metric(_EST, "fraction", _foot_gap_narrow),
}


def metric_key(name: str, tier: str) -> str:
    return f"{name}_{tier}" if tier else name


def split_key(key: str) -> tuple[str, str]:
    """(metric name, tier) of a key; refuses a name or a tier that is not declared."""
    for tier in (TIER_TRUTH, TIER_EST):
        if key.endswith(f"_{tier}") and key[: -len(tier) - 1] in METRICS:
            name = key[: -len(tier) - 1]
            if tier not in METRICS[name].tiers:
                raise MetricUnavailable(f"{name} has no {tier} tier (it is {METRICS[name].tiers})")
            return name, tier
    if key not in METRICS:
        raise MetricUnavailable(f"unknown metric {key}")
    if TIER_NONE not in METRICS[key].tiers:
        tiers = " or ".join(f"{key}_{t}" for t in METRICS[key].tiers)
        raise MetricUnavailable(f"{key} needs its tier named: {tiers}")
    return key, TIER_NONE


def all_keys() -> list[str]:
    return [metric_key(name, tier) for name, m in METRICS.items() for tier in m.tiers]


def evaluate(run: Run, keys: Sequence[str]) -> dict:
    """The asked metrics; raises :class:`MetricUnavailable` at the first the run cannot give."""
    out = {}
    for key in keys:
        name, tier = split_key(key)
        out[key] = METRICS[name].fn(run, tier)
    return out


def evaluate_available(run: Run) -> tuple[dict, dict[str, str]]:
    """(every metric the run can give, {key: why not} for the rest)."""
    values, refused = {}, {}
    for key in all_keys():
        try:
            values.update(evaluate(run, [key]))
        except MetricUnavailable as exc:
            refused[key] = str(exc)
    return values, refused


# ── the ranking cost and the constraint vector ────────────────────────────


@dataclass(frozen=True)
class JTerm:
    key: str  # the metric it reads (an error, ≥ 0)
    scale: float  # the error that costs β
    beta: float


# Scales are fixed physical quantities (module docstring); the forward term is
# built from v_mps below because its error is against the run's own command.
J_TERMS: dict[str, JTerm] = {
    "forward_speed": JTerm("v_mps", 0.05, 1.0),  # [m/s]
    "yaw_rate": JTerm("yaw_rate_error", 0.05, 0.5),  # [rad/s]
    "lateral_drift": JTerm("lateral_drift_m", 0.30, 0.25),  # [m]
    "tilt": JTerm("tilt_rms", 0.05, 0.25),  # [rad]
    "contact_fz": JTerm("contact_fz_error", 0.10, 0.25),  # [body weight]
}


def j_eval(run: Run, tier: str = TIER_TRUTH, terms: Sequence[str] | None = None) -> dict:
    """``{"J": …, "terms": {…}}``, or ``{"J": None, "why": …}`` when the run has no J.

    `terms` names a subset of :data:`J_TERMS`; a subset is a partial cost and
    is labelled so (``"partial": True``) — it does not compare with a full J.
    """
    names = list(J_TERMS) if terms is None else list(terms)
    if run.faulted or run.lift is None:
        return {"J": None, "why": "the run faulted" if run.faulted else "the run never stepped"}
    parts: dict[str, float] = {}
    for name in names:
        term = J_TERMS[name]
        key_tier = TIER_TRUTH if METRICS[term.key].tiers == _TRUTH else tier
        try:
            value = METRICS[term.key].fn(run, key_tier)
        except MetricUnavailable as exc:
            return {"J": None, "why": f"{name}: {exc}"}
        if name == "forward_speed" and value is not None:
            motion = planar_motion(run, key_tier)
            value = abs(motion[3] - value)
        if value is None or not math.isfinite(value):
            return {"J": None, "why": f"{name}: no value in this run"}
        parts[name] = term.beta * value / term.scale
    return {
        "J": float(sum(parts.values())),
        "terms": parts,
        "tier": tier,
        "partial": set(names) != set(J_TERMS),
    }


CONSTRAINT_KEYS = (
    "completed",
    "plan_loss",
    "touchdowns",
    "withheld_rate",
    "deadline_rate",
    "plausibility_rejects",
    "qp_infeasible_rows",
    "margin_friction_min",
    "margin_cop_min",
    "margin_torque_min",
)


def constraints(run: Run) -> dict:
    """The run's constraint vector (module docstring); None where a column is missing."""
    out = {}
    for key in CONSTRAINT_KEYS:
        try:
            out.update(evaluate(run, [key]))
        except MetricUnavailable:
            out[key] = None
    return out


# ── validity ──────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ModelTolerance:
    mass_kg: float = 1e-4
    com_m: float = 1e-5
    inertia_kgm2: float = 1e-4


@dataclass(frozen=True)
class ValiditySpec:
    """What a run must show to count. None switches a check off: it is then absent from
    the run's ``checks`` (never listed as passed)."""

    rtf_tol: float | None = 0.05
    single_support_s: tuple[float, float] | None = None
    command_rate: Mapping[str, float] | None = None  # event → expected rate
    command_rate_tol: float = 0.05
    zero_columns: tuple[str, ...] = ()
    provenance: bool = True  # commit clean, installed config = source, model match
    model: ModelTolerance = field(default_factory=ModelTolerance)


def measured_rtf(launch_log: str, wall_span: tuple[float, float]) -> float | None:
    """Simulated ÷ wall seconds over the recorder's span, from the simulator's 1 Hz lines."""
    points = [(float(a), float(b)) for a, b in SIM_TIME_LINE.findall(launch_log)]
    # A simulator reset (its time going back) ends the measured span.
    cut = next((i for i in range(1, len(points)) if points[i][1] < points[i - 1][1]), len(points))
    window = [p for p in points[:cut] if wall_span[0] <= p[0] <= wall_span[1]]
    if len(window) <= 2:
        return None
    return (window[-1][1] - window[0][1]) / (window[-1][0] - window[0][0])


# MPC lock-step (launch mpc_sync:=lockstep): a new plan is used L ticks after its
# trigger, so its first row's age is L × tick exactly, and triggers are N = L
# ticks apart. The shipped G1 at 500 Hz: L = N = 35, tick 2000 µs.
LOCKSTEP_AGE_US = 70000.0
LOCKSTEP_PERIOD_TICKS = 35
REFUSED_AGE_LINE = re.compile(r"mpc: solves .*?refused: epoch \d+ state \d+ age (\d+)")


def lockstep_checks(diag: Mapping[str, np.ndarray], launch_log: str) -> dict[str, bool]:
    """The lock-step invariants: plan age at first use, trigger spacing, no age refusal."""
    out = dict.fromkeys(("lockstep_age", "lockstep_period", "lockstep_no_age_refusal"), False)
    if all(c in diag for c in ("ref_source", "plan_id", "policy_age_us")):
        stand = np.flatnonzero(diag["ref_source"] == wm.REF_SOURCE_STAND)
        if stand.size:
            plan = diag["plan_id"]
            # The first row of each plan id seen for the first time after the
            # first stand row. A return to an earlier id (a sanity refusal
            # rolled the active plan back) is not a new plan's first use.
            ids, idx = np.unique(plan, return_index=True)
            first = idx[(idx > stand[0]) & (ids > 0)]
            out["lockstep_age"] = first.size > 0 and bool(
                np.all(diag["policy_age_us"][first] == LOCKSTEP_AGE_US)
            )
    if "mpc_trigger_tick" in diag:
        trig = diag["mpc_trigger_tick"]
        distinct = trig[np.isfinite(trig)]
        distinct = (
            distinct[np.concatenate([[True], np.diff(distinct) != 0])]
            if distinct.size
            else distinct
        )
        steps = np.diff(distinct)
        out["lockstep_period"] = steps.size > 0 and bool(np.all(steps == LOCKSTEP_PERIOD_TICKS))
    found = REFUSED_AGE_LINE.findall(launch_log)
    out["lockstep_no_age_refusal"] = bool(found) and int(found[-1]) == 0
    return out


def recorder_span(stance_csv: Path) -> tuple[float, float] | None:
    """Wall stamps of the recorder's first and last sample."""
    if not stance_csv.is_file():
        return None
    rows = stance_csv.read_text().splitlines()[1:]
    if not rows:
        return None
    return float(rows[0].split(",")[0]), float(rows[-1].split(",")[0])


def provenance_checks(note: Mapping, model: ModelTolerance) -> dict[str, bool]:
    """Checks on the ``provenance`` block ``run_arms.py`` records; all False without one."""
    prov = note.get("provenance")
    if not isinstance(prov, Mapping):
        return {"commit_clean": False, "config_installed": False, "model_match": False}
    match = prov.get("model_match") or {}
    config = prov.get("config") or {}
    return {
        "commit_clean": bool(prov.get("commit")) and prov.get("dirty") is False,
        "config_installed": bool(config)
        and bool(config.get("files"))
        and config.get("installed_sha256") is not None
        and config.get("installed_sha256") == config.get("source_sha256"),
        "model_match": all(
            isinstance(match.get(k), (int, float)) and abs(match[k]) <= limit
            for k, limit in (
                ("mass_kg", model.mass_kg),
                ("com_m", model.com_m),
                ("inertia_kgm2", model.inertia_kgm2),
            )
        ),
    }


def overlay_ok(note: Mapping, want: str | None) -> bool:
    """The run carried exactly the arm's overlay (`want`: a path suffix or a sha256; None = none)."""
    args = [a for a in note.get("launch_args", []) if a.startswith("sim_overlay")]
    if want is None:
        return not args
    if len(args) != 1:
        return False
    recorded = ((note.get("provenance") or {}).get("overlay") or {}).get("sha256")
    return args[0].endswith(want) or recorded == want


# The columns every window and count is placed by: a cell of these that did not
# parse would otherwise read as "no contact", "no fault" or "no command".
PLACING_COLUMNS = ("t_relative_s", "contact_mask", "fault_cause", "cmd_vx", "cmd_vy", "cmd_wz")


def log_readable(run: Run) -> bool:
    """Every placing column is there, has rows and holds only numbers."""
    return all(
        c in run.log.diag and run.log.diag[c].size > 0 and bool(np.isfinite(run.log.diag[c]).all())
        for c in PLACING_COLUMNS
    )


def log_spans_script(run: Run) -> bool:
    """The log runs from the first command to the end of the script's moving part.

    A log cut short with no fault row (a simulator that died, a recorder
    stopped early) has not shown that the run completed.
    """
    if not log_readable(run):
        return False
    moving = np.flatnonzero(_commanded(run))
    if not moving.size:
        return False
    return bool(run.t[-1] - run.t[moving[0]] >= wm.script_length(run.script[:-1]))


CORE_METRICS = (
    "completed",
    "survival_s",
    "touchdowns",
    "straight_fault",
    "transition_fault",
    "exposed",
    "plan_loss",
)


def run_validity(
    run: Run, run_dir: Path, note: Mapping, spec: ValiditySpec, overlay: str | None
) -> dict:
    """{check: bool} of one run plus the measured values under ``"measured"``."""
    log_text = (run_dir / "launch.log").read_text(errors="replace")
    span = recorder_span(run_dir / "stance.csv")
    rtf = measured_rtf(log_text, span) if span is not None else None
    target = note.get("rtf")
    checks: dict[str, bool] = {
        "recorder": note.get("recorder_rc") == 0,
        "no_wip": "[wip]" not in log_text,
        "overlay": overlay_ok(note, overlay),
    }
    lockstep = note.get("mpc_sync") == "lockstep"
    if spec.rtf_tol is not None and not lockstep:
        # A run that names no target has nothing to be held to: that fails, it does not skip.
        checks["rtf"] = (
            bool(target) and rtf is not None and abs(rtf / target - 1.0) <= spec.rtf_tol
        )
    if lockstep:  # the rtf follows the solve: the invariants replace its tolerance
        checks.update(lockstep_checks(run.log.diag, log_text))
    checks["log_readable"] = log_readable(run)
    checks["log_spans_script"] = run.faulted or log_spans_script(run)
    support = _single_support(run, TIER_NONE)
    if spec.single_support_s is not None:
        lo, hi = spec.single_support_s
        checks["single_support"] = support is not None and lo <= support <= hi
    rates = {
        event: command_rate(run, column, run.crossings[event])
        for event, column in (
            ("turn_start", "cmd_wz"),
            ("turn_end", "cmd_wz"),
            ("stop", CMD_ALONG),
        )
    }
    if spec.command_rate is not None:
        checks["command_shape"] = all(
            rates.get(event) is not None
            and abs(rates[event] / want - 1.0) <= spec.command_rate_tol
            for event, want in spec.command_rate.items()
        )
    for column in spec.zero_columns:
        checks[f"zero_{column}"] = column in run.log.diag and bool(
            np.all(run.log.diag[column] == 0.0)
        )
    if spec.provenance:
        checks.update(provenance_checks(note, spec.model))
    return {
        "checks": checks,
        "valid": all(checks.values()),
        "measured": {"rtf": rtf, "single_support_s": support, "command_rate": rates},
    }


# ── arms ──────────────────────────────────────────────────────────────────


def _median(values: Sequence) -> float | None:
    finite = [v for v in values if v is not None and math.isfinite(v)]
    return float(np.median(finite)) if finite else None


def arm_summary(runs: Sequence[Mapping], tier: str) -> dict:
    """Counts and medians over an arm's valid runs (each a dict of metric values)."""
    n = len(runs)
    done = sum(bool(r["completed"]) for r in runs)
    ratio = [r.get(metric_key("v_ratio", tier)) for r in runs]
    lo, hi = wilson_interval(done, n)
    return {
        "n": n,
        "completed": done,
        "completed_wilson95": [lo, hi],
        "touchdowns_median": _median([r["touchdowns"] for r in runs]),
        "survival_s_median": _median([r["survival_s"] for r in runs]),
        metric_key("v_ratio", tier): _median(ratio),
        metric_key("v_mps", tier): _median([r.get(metric_key("v_mps", tier)) for r in runs]),
        "n_v": sum(v is not None and math.isfinite(v) for v in ratio),
        "straight_fault": sum(bool(r["straight_fault"]) for r in runs),
        "transition_fault": sum(bool(r["transition_fault"]) for r in runs),
        "exposed": sum(bool(r["exposed"]) for r in runs),
        "plan_loss": sum(bool(r["plan_loss"]) for r in runs),
    }


def compare_completion(control: Mapping, candidate: Mapping, alpha: float = 0.05) -> dict:
    """The completion constraint: reject a candidate that completes less often than the control.

    One-sided Fisher (control > candidate). ``power`` is the chance this test
    rejects at the smaller arm's size when the control completes at its
    observed rate and the candidate at 0.2 below it — what a pass is worth.
    """
    p = fisher_one_sided(
        control["completed"], control["n"], candidate["completed"], candidate["n"]
    )
    n = min(control["n"], candidate["n"])
    rate = control["completed"] / control["n"] if control["n"] else math.nan
    power = fisher_power(rate, max(rate - 0.2, 0.0), n, alpha) if n and rate == rate else None
    return {"fisher_p": p, "alpha": alpha, "reject": p < alpha, "power_at_minus_0.2": power}


def pooling_key(note: Mapping) -> tuple | None:
    """Runs pool only with the same build and overlay; None for a run that records neither."""
    prov = note.get("provenance")
    if not isinstance(prov, Mapping):
        return None
    return (prov.get("commit"), prov.get("dirty"), (prov.get("overlay") or {}).get("sha256"))


# ── a batch ───────────────────────────────────────────────────────────────

CONTROLLER_DIR = Path("session") / ld.SUBDIR


def validity_spec(data: Mapping | None) -> ValiditySpec:
    data = dict(data or {})
    if "single_support_s" in data and data["single_support_s"] is not None:
        data["single_support_s"] = tuple(data["single_support_s"])
    if "zero_columns" in data:
        data["zero_columns"] = tuple(data["zero_columns"])
    if "model" in data:
        data["model"] = ModelTolerance(**data["model"])
    return ValiditySpec(**data)


def evaluate_batch(root: Path, spec: Mapping) -> dict:
    """Every arm of a batch (module docstring); `spec` is the batch's registration.

    ``spec``: ``script`` (a :data:`walk_metrics.COMMAND_SCRIPTS` name), ``tier``,
    ``arms`` ({name: {runs: glob under `root`, overlay: suffix | sha256 | null,
    validity: fields that replace the batch's for this arm — how a deliberately
    perturbed plant declares the model difference it runs with}}),
    ``control`` (an arm name), ``validity`` (:class:`ValiditySpec` fields),
    ``min_valid``, ``j_terms`` (a :data:`J_TERMS` subset, default all) and
    ``foot_width_m`` (the sole's width [m] the foot gap is judged against;
    without it the narrow fraction is refused).
    """
    script = wm.COMMAND_SCRIPTS[spec.get("script", "walk")]
    tier = spec.get("tier", TIER_TRUTH)
    width = spec.get("foot_width_m")
    if width is not None and not (
        isinstance(width, (int, float))
        and not isinstance(width, bool)
        and math.isfinite(width)
        and width > 0
    ):
        raise ValueError(f"foot_width_m must be a positive length [m], not {width!r}")
    vspec = validity_spec(spec.get("validity"))
    arms: dict[str, dict] = {}
    for name, arm in spec["arms"].items():
        runs = []
        arm_vspec = (
            validity_spec({**(spec.get("validity") or {}), **arm["validity"]})
            if arm.get("validity")
            else vspec
        )
        for run_dir in sorted(root.glob(arm["runs"])):
            note_path = run_dir / "run.json"
            if not note_path.is_file():
                runs.append({"run": str(run_dir.relative_to(root)), "missing": True})
                continue
            try:
                note = json.loads(note_path.read_text())
                run = Run(load_run(run_dir / CONTROLLER_DIR), script, width)
                validity = run_validity(run, run_dir, note, arm_vspec, arm.get("overlay"))
                values, refused = evaluate_available(run)
            except Exception as exc:  # one unreadable run must not take the batch with it
                runs.append(
                    {
                        "run": str(run_dir.relative_to(root)),
                        "valid": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                )
                continue
            # An arm is summed over these: a run that cannot give one does not count.
            validity["checks"]["core_metrics"] = all(k in values for k in CORE_METRICS)
            validity["valid"] = all(validity["checks"].values())
            runs.append(
                {
                    "run": str(run_dir.relative_to(root)),
                    "session": note.get("session"),
                    "pooling_key": pooling_key(note),
                    **validity,
                    "metrics": values,
                    "refused": refused,
                    "j_eval": j_eval(run, tier, spec.get("j_terms")),
                    "constraints": constraints(run),
                }
            )
        present = [r for r in runs if not r.get("missing") and "error" not in r]
        sessions = [r["session"] for r in present]
        keys = {r["pooling_key"] for r in present}
        valid = [r["metrics"] for r in present if r["valid"]]
        arms[name] = {
            "runs": runs,
            "sessions_distinct": len(set(sessions)) == len(sessions),
            "one_build_and_overlay": len(keys) <= 1,
            "summary": arm_summary(valid, tier) if valid else {"n": 0},
        }
    out: dict = {"tier": tier, "foot_width_m": width, "arms": arms}
    control = spec.get("control")
    min_valid = spec.get("min_valid", 0)
    if control is not None:
        c = arms[control]["summary"]
        out["completion"] = {
            name: (
                {"void": True}
                if min(arm["summary"]["n"], c["n"]) < max(min_valid, 1)
                else compare_completion(c, arm["summary"])
            )
            for name, arm in arms.items()
            if name != control
        }
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("root", type=Path, help="the batch directory the spec's run globs are under")
    p.add_argument("--spec", type=Path, required=True, help="the batch registration (YAML)")
    p.add_argument("--out", type=Path, help="write the full result here (JSON)")
    args = p.parse_args(argv)
    result = evaluate_batch(args.root, yaml.safe_load(args.spec.read_text()))
    if args.out:
        args.out.write_text(json.dumps(result, indent=1, default=float))
    for name, arm in result["arms"].items():
        print(name, json.dumps(arm["summary"], default=float))
        for r in arm["runs"]:
            if r.get("missing"):
                print("  missing:", r["run"])
            elif "error" in r:
                print("  unreadable:", r["run"], r["error"])
            elif not r["valid"]:
                failed = [k for k, ok in r["checks"].items() if not ok]
                print("  invalid:", r["run"], failed)
    for name, verdict in result.get("completion", {}).items():
        print("completion", name, json.dumps(verdict, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
