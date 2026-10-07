"""Hand closure time T_close,e2e(eta) from a device state CSV (dynamic_catching S4.2).

Stage S4a (``docs/dynamic_catching/ID_INDEX.md`` §3), law: L6 §4.2.

    rho(t)          = min over the caging set C of
                      (q_i(t) - q_i^pre) * s_i / |q_i^cls - q_i^pre|,  s_i = sign(q_i^cls - q_i^pre)
    T_close,e2e(eta) = inf { t - t_cmd : rho(t) >= eta }

``rho`` is a MINIMUM, not a mean: one finger that never arrives is the whole
hand failing to cage, and a mean would average that away against three fingers
that did. The caging set is the profile's, not "every joint" — the shipped p1b
posture EXTENDS one DIP joint while the MCPs flex, and counting it would read
that extension as closure progress.

TWO TIME AXES, BOTH REPORTED, because in a lock-step simulator they answer
different questions and neither alone is honest:

  * ``steady``  — the CSV's ``t_relative_s``, the RT loop's steady clock. This
    is what L6 §4.2 defines and what a real hand would be measured with, and in
    sim it INCLUDES host stalls (S3a measured delta p95 4.9 ms, max 18.2 ms).
  * ``tick``    — (row index difference) * dt, i.e. simulated time. Deterministic
    and free of host stalls, but it is not wall time and cannot be compared to a
    real-hardware measurement.

A gap between the two IS the host stall, not a bug in either. The tick axis is
only valid while no CSV row was dropped, so this tool checks the sample spacing
and says so when it cannot trust it — a dropped row shifts every later tick
count silently.

Beside the mean, maximum and p99 the report gives each axis's sample standard
deviation and ``mean − T_close_lead`` (:func:`nominal_closure_offset`; an older
sidecar without a lead falls back to ``T_close_e2e``): the
spread and the nominal instant of the closure relative to the catch instant,
which a planner with a closure WINDOW needs (dynamic_catching E1-F15, #741).

The profile comes from a JSON sidecar written by ``run_hand_close_trials``,
which reads it from the CONTROLLER's read-only parameters rather than from the
shipped YAML. A run whose controller read something else must not be analysed
against the file on disk.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

# Sample-spacing tolerance for trusting the tick axis. Generous on purpose: the
# steady clock jitters in sim, and the check is meant to catch a DROPPED ROW
# (one missing sample doubles the spacing), not to police jitter.
TICK_AXIS_SPACING_TOLERANCE = 0.5

# How close a command row must sit to a shipped pose, as a fraction of the
# profile's own |q_close - q_pre| travel, to count as that pose at all. Rows
# outside every band are 'other' — see _nearest_pose. A quarter of the travel
# is wide enough for the real steps (the controller clamps them to the joint
# limits, so they are near but rarely exact) and far tighter than the ~0.9x
# travel at which the shipped p1b activation pose sits.
POSE_MATCH_BAND = 0.25


@dataclass
class HandProfile:
    """The L6 §5.1 subset rho needs, as the controller loaded it."""

    joint_names: list[str]
    q_pre: list[float]
    q_close: list[float]
    caging_mask: list[bool]
    eta_close: float
    rho_eps: float = 0.02
    dt: float = 0.002
    # False when the sidecar carried no `dt` and the default above was assumed.
    # The tick axis and the dropped-row gate BOTH scale with dt, so an assumed
    # value does not just stretch the axis — it widens the drop gate and can
    # call a run trusted that dropped rows. Reported, never silently used.
    dt_is_assumed: bool = False
    # `hand.T_close_e2e` as the controller had it loaded for the run (sidecar
    # `T_close_e2e_at_run`): the MEASURED closure time. NaN when the sidecar
    # does not say.
    t_close_e2e: float = math.nan
    # `hand.T_close_lead_from_t_c` as run (sidecar `T_close_lead_at_run`):
    # what the hand sequencer subtracts from the catch instant to time the close
    # command. NaN when the sidecar does not say (a controller built before the
    # lead was split from T_close_e2e, where the two were one value).
    t_close_lead: float = math.nan

    @property
    def sequencer_lead(self) -> tuple[float, str]:
        """(value, parameter name) the sequencer subtracted from t_c at the run.

        The lead when the sidecar has one, else T_close_e2e (an older run).
        """
        if not math.isnan(self.t_close_lead):
            return self.t_close_lead, "T_close_lead"
        return self.t_close_e2e, "T_close_e2e"

    @property
    def caging_indices(self) -> list[int]:
        return [i for i, on in enumerate(self.caging_mask) if on]

    def validate(self) -> None:
        n = len(self.joint_names)
        for name, seq in (
            ("q_pre", self.q_pre),
            ("q_close", self.q_close),
            ("caging_mask", self.caging_mask),
        ):
            if len(seq) != n:
                raise ValueError(f"{name} has {len(seq)} entries, joint_names has {n}")
        if not self.caging_indices:
            raise ValueError("caging_mask selects no joint — rho would be a minimum over nothing")
        for i in self.caging_indices:
            gap = abs(self.q_close[i] - self.q_pre[i])
            if gap <= self.rho_eps:
                raise ValueError(
                    f"caging joint {self.joint_names[i]} travels {gap:.4f} rad "
                    f"<= rho_eps {self.rho_eps} — rho would divide by ~0 "
                    "(the C++ validator refuses this profile too)"
                )
        if not 0.0 < self.eta_close <= 1.0:
            raise ValueError(f"eta_close {self.eta_close} outside (0, 1]")


def load_profile(path: Path) -> HandProfile:
    data = json.loads(path.read_text())
    profile = HandProfile(
        joint_names=list(data["joint_names"]),
        q_pre=[float(v) for v in data["q_pre"]],
        q_close=[float(v) for v in data["q_close"]],
        caging_mask=[bool(v) for v in data["caging_mask"]],
        eta_close=float(data["eta_close"]),
        rho_eps=float(data.get("rho_eps", 0.02)),
        dt=float(data.get("dt", 0.002)),
        dt_is_assumed="dt" not in data,
        t_close_e2e=float(data.get("T_close_e2e_at_run", math.nan)),
        t_close_lead=float(data.get("T_close_lead_at_run", math.nan)),
    )
    profile.validate()
    return profile


def joint_progress(q_i: float, q_pre_i: float, q_close_i: float) -> float:
    """Signed closure progress of ONE joint toward ``q_close_i``, in [.., 1] at
    q_close (see the module docstring's ``rho`` for the caging-set minimum
    this feeds). Direction, not magnitude: a joint whose closed pose is BELOW
    its preshape closes by decreasing, and an unsigned ratio would report it
    as moving backwards for the whole trial.
    """
    span = q_close_i - q_pre_i
    return (q_i - q_pre_i) * (1.0 if span > 0 else -1.0) / abs(span)


def rho(q: list[float], profile: HandProfile) -> float:
    """Closure progress in [.., 1] — see the module docstring. Minimum over C."""
    worst = math.inf
    for i in profile.caging_indices:
        worst = min(worst, joint_progress(q[i], profile.q_pre[i], profile.q_close[i]))
    return worst


@dataclass
class Trial:
    index: int
    t_cmd_s: float
    rows: int
    t_close_steady_s: float = math.nan  # nan = never reached eta within the trial
    t_close_tick_s: float = math.nan
    rho_final: float = math.nan


@dataclass
class RunStats:
    trials: list[Trial] = field(default_factory=list)
    rows: int = 0
    tick_axis_trusted: bool = True
    spacing_note: str = ""


def _columns(header: list[str], prefix: str, profile: HandProfile) -> list[int]:
    """Indices of `<prefix>_<joint>` for every profile joint, in profile order.

    A missing column is fatal rather than skipped: silently analysing a subset
    of the hand would report the fastest fingers as the closure time.
    """
    out = []
    for name in profile.joint_names:
        column = f"{prefix}_{name}"
        if column not in header:
            raise SystemExit(
                f"CSV has no column '{column}'. The profile's joint_names must be the "
                "hand device's joint_state_names, which is what the CSV header is built "
                f"from. Header has: {[h for h in header if h.startswith(prefix)]}"
            )
        out.append(header.index(column))
    return out


def _nearest_pose(command: list[float], profile: HandProfile) -> str:
    """Which shipped pose this command row is: 'pre', 'close' or 'other'.

    Compared by distance rather than equality because the controller clamps the
    target to the device limits before commanding it, so a pose that sits on a
    limit is commanded as the clamped value and an equality test would classify
    every row as 'other'.

    'other' exists because a nearest-of-two vote has no way to say "neither".
    Every command row that is not a step — the activation hold pose, a
    `q_open` step from the GUI — is forced into 'pre' or 'close' by proximity
    alone, and a row that lands on 'close' OPENS A TRIAL. That trial never
    reaches eta, so it is counted as a timeout: it does not move the mean or
    the p99 (those filter NaN) but it inflates the trial count and the
    exclusion count, which is exactly the success rate S4.2 hands onward.

    The margin is not hypothetical. On a shipped 10-joint hand profile the
    activation hold pose (every joint at zero in sim) sat 0.107% from being
    classified 'close' — 3.033 vs 3.036 in squared distance — against poses
    the profile itself marks provisional. The named case and its numbers live
    with that profile, in its controller YAML.

    The band is a fraction of the profile's OWN travel, so it scales with the
    hand rather than assuming a joint count or a unit.
    """
    n = len(profile.joint_names)
    d_pre = sum((command[i] - profile.q_pre[i]) ** 2 for i in range(n))
    d_close = sum((command[i] - profile.q_close[i]) ** 2 for i in range(n))
    travel_sq = sum((profile.q_close[i] - profile.q_pre[i]) ** 2 for i in range(n))
    band_sq = (POSE_MATCH_BAND * POSE_MATCH_BAND) * travel_sq
    if min(d_pre, d_close) > band_sq:
        return "other"
    return "pre" if d_pre <= d_close else "close"


def analyse(csv_path: Path, profile: HandProfile) -> RunStats:
    with csv_path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        pos_cols = _columns(header, "actual_pos", profile)
        cmd_cols = _columns(header, "command", profile)
        if "t_relative_s" not in header:
            raise SystemExit(f"{csv_path}: no t_relative_s column")
        t_col = header.index("t_relative_s")
        rows = [
            (
                float(r[t_col]),
                [float(r[c]) for c in pos_cols],
                [float(r[c]) for c in cmd_cols],
            )
            for r in reader
            if r
        ]

    stats = RunStats(rows=len(rows))
    if len(rows) < 2:
        return stats

    # Tick-axis trust: one dropped row doubles the spacing and shifts every
    # later tick count, so the axis is reported as untrusted rather than quietly
    # wrong.
    spacings = sorted(rows[i][0] - rows[i - 1][0] for i in range(1, len(rows)))
    median_spacing = spacings[len(spacings) // 2]
    worst_spacing = spacings[-1]
    if profile.dt_is_assumed:
        # Both the axis and the gate that guards it scale with dt, so an
        # assumed dt cannot police itself: at 1 kHz it would compare a 2 ms
        # drop gap against a 3 ms threshold and call the run clean.
        stats.tick_axis_trusted = False
        stats.spacing_note = (
            f"the sidecar carried no 'dt', so {profile.dt * 1e3:.2f} ms was ASSUMED. Re-run the "
            "trials with a runner that records the controller's control.dt, or add the key by "
            "hand if you know the rate"
        )
    elif worst_spacing > profile.dt * (1.0 + TICK_AXIS_SPACING_TOLERANCE):
        stats.tick_axis_trusted = False
        stats.spacing_note = (
            f"largest sample gap {worst_spacing * 1e3:.2f} ms exceeds dt "
            f"{profile.dt * 1e3:.2f} ms by more than "
            f"{TICK_AXIS_SPACING_TOLERANCE * 100:.0f}% (median {median_spacing * 1e3:.2f} ms)"
        )

    # Segment on the command lane: a trial starts on the row where the command
    # first equals the closed pose and ends when it leaves it again.
    poses = [_nearest_pose(cmd, profile) for _, _, cmd in rows]
    trial_index = 0
    start = None
    for i, pose in enumerate(poses):
        entering = pose == "close" and (i == 0 or poses[i - 1] != "close")
        leaving = pose != "close" and start is not None
        if entering:
            start = i
        elif leaving:
            trial_index += 1
            stats.trials.append(_summarise(trial_index, rows[start:i], start, profile))
            start = None
    if start is not None:
        trial_index += 1
        stats.trials.append(_summarise(trial_index, rows[start:], start, profile))
    return stats


def _summarise(index: int, window: list, start_row: int, profile: HandProfile) -> Trial:
    del start_row  # tick counts are relative to the window's own first row
    t_cmd = window[0][0]
    trial = Trial(index=index, t_cmd_s=t_cmd, rows=len(window))
    for offset, (t_s, q, _cmd) in enumerate(window):
        value = rho(q, profile)
        trial.rho_final = value
        if math.isnan(trial.t_close_steady_s) and value >= profile.eta_close:
            trial.t_close_steady_s = t_s - t_cmd
            trial.t_close_tick_s = offset * profile.dt
            # Do not break: rho_final should describe where the hand ended up,
            # not where it first crossed.
    return trial


def quantile(values: list[float], q: float) -> float:
    """Order statistic, NOT an interpolated quantile.

    p99 of fewer than 100 samples is the maximum by construction; keeping it an
    order statistic makes that visible instead of inventing a value between
    samples. The caller prints the sample count next to it for the same reason.
    """
    if not values:
        return math.nan
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil(q * len(ordered))) - 1))
    return ordered[idx]


def summarise_axis(values: list[float]) -> dict[str, float]:
    """n, mean, max, p99 and the sample standard deviation of the finite values.

    ``std`` divides by n − 1 and is NaN for fewer than two closures.
    """
    finite = [v for v in values if not math.isnan(v)]
    if not finite:
        return {"n": 0, "mean": math.nan, "max": math.nan, "p99": math.nan, "std": math.nan}
    mean = sum(finite) / len(finite)
    std = math.nan
    if len(finite) > 1:
        std = math.sqrt(sum((v - mean) ** 2 for v in finite) / (len(finite) - 1))
    return {
        "n": len(finite),
        "mean": mean,
        "max": max(finite),
        "p99": quantile(finite, 0.99),
        "std": std,
    }


def nominal_closure_offset(mean_t_close_s: float, t_close_lead_s: float) -> float:
    """How long after the catch instant the closure is complete, on average.

    The hand sequencer commands the close at ``t_c − T_close_lead`` (L6 §4.3), so
    a closure that takes ``mean_t_close_s`` completes ``mean − T_close_lead``
    after ``t_c``. ``t_close_lead_s`` is what the sequencer subtracted at the
    run: ``T_close_lead``, or ``T_close_e2e`` for a run that predates the split
    (``HandProfile.sequencer_lead`` picks). When the lead is a high quantile of
    the same distribution this is small and NEGATIVE: the hand is, on average,
    closed slightly before the instant it was timed for. It is the ``delta_0``
    of a closure window measured from ``t_c`` (reference §8.4, §17.8). NaN when
    either input is.
    """
    return mean_t_close_s - t_close_lead_s


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", type=Path, help="<hand>_state.csv from the catching controller")
    parser.add_argument(
        "--profile",
        type=Path,
        required=True,
        help="hand_close_run.json written by run_hand_close_trials (the profile the "
        "CONTROLLER loaded, not the shipped YAML)",
    )
    parser.add_argument("--plot", type=Path, default=None, help="write a rho/T_close plot here")
    args = parser.parse_args(argv)

    profile = load_profile(args.profile)
    stats = analyse(args.csv, profile)
    if not stats.trials:
        print(
            f"{args.csv}: no closure trial found in {stats.rows} rows. The command lane "
            "never moved to the closed pose — was diagnostic.hand_step true, and did the "
            "runner target the hand group?",
            file=sys.stderr,
        )
        return 2

    steady = [t.t_close_steady_s for t in stats.trials]
    tick = [t.t_close_tick_s for t in stats.trials]
    timeouts = [t for t in stats.trials if math.isnan(t.t_close_steady_s)]

    print(f"csv        : {args.csv}")
    print(f"profile    : {args.profile}")
    print(f"rows       : {stats.rows}")
    print(f"trials     : {len(stats.trials)}   (eta = {profile.eta_close})")
    print(f"caging set : {[profile.joint_names[i] for i in profile.caging_indices]}")
    if timeouts:
        print(
            f"!! {len(timeouts)} trial(s) never reached eta "
            f"(worst final rho {min(t.rho_final for t in timeouts):.3f}). They are EXCLUDED "
            "from the statistics below, which are therefore conditioned on success — "
            "quote the exclusion count with them."
        )
    if not stats.tick_axis_trusted:
        print(f"!! tick axis NOT trusted: {stats.spacing_note}")
        print(
            "   A dropped CSV row shifts every later tick count. Check the controller's "
            "drop WARN before quoting the tick column."
        )
    print()
    print("  T_close,e2e            n       mean        max        p99")
    for label, values in (("steady [ms]", steady), ("tick   [ms]", tick)):
        s = summarise_axis(values)
        print(
            f"  {label}      {s['n']:5d}  {s['mean'] * 1e3:9.3f}  "
            f"{s['max'] * 1e3:9.3f}  {s['p99'] * 1e3:9.3f}"
        )
    n_ok = summarise_axis(steady)["n"]
    if n_ok < 100:
        print()
        print(
            f"  NOTE: p99 of {n_ok} samples is the {min(n_ok, 100)}th order statistic, i.e. "
            "the maximum. A 99th percentile needs >= 100 successful trials."
        )
    print()
    print("  steady is L6 §4.2's definition and includes host stalls; tick is simulated")
    print("  time and cannot be compared with a real-hardware measurement. Report both.")
    print()
    print("  spread (sample std, n - 1) and the closure's offset from the catch instant")
    lead, lead_name = profile.sequencer_lead
    for label, values in (("steady [ms]", steady), ("tick   [ms]", tick)):
        s = summarise_axis(values)
        offset = nominal_closure_offset(s["mean"], lead)
        print(f"  {label}      std {s['std'] * 1e3:9.3f}   mean - {lead_name} {offset * 1e3:9.3f}")
    if math.isnan(lead):
        print(
            "  !! the sidecar carries no T_close_lead_at_run and no T_close_e2e_at_run:"
            " the offset cannot be computed."
        )
    else:
        if not math.isnan(profile.t_close_e2e):
            print(
                f"  T_close_e2e at the run: {profile.t_close_e2e * 1e3:.3f} ms (measured closure)"
            )
        if lead_name == "T_close_lead":
            print(f"  T_close_lead at the run: {lead * 1e3:.3f} ms (the sequencer closes")
        else:
            print(
                "  !! the sidecar has no T_close_lead_at_run (a run before the lead was split"
                " from T_close_e2e): falling back to T_close_e2e as what the sequencer"
                f" subtracts, {lead * 1e3:.3f} ms (the sequencer closes"
            )
        print(f"  at t_c - {lead_name}, so the offset is where the closure lands after t_c).")
    if profile.dt_is_assumed:
        print("  !! dt was ASSUMED: the size of one tick's quantisation is not known.")
    else:
        print(
            f"  One tick is {profile.dt * 1e3:.3f} ms. Both axes are read on CSV rows, so a "
            "std may already"
        )
        print(
            f"  hold up to the quantisation of a tick ({profile.dt / math.sqrt(12.0) * 1e3:.3f} "
            "ms = dt / sqrt(12)): it is"
        )
        print("  reported beside the std, never added to it here.")

    if args.plot is not None:
        _write_plot(args.plot, stats.trials, profile)
        print(f"\n  plot -> {args.plot}")
    return 0


def _write_plot(path: Path, trials: list[Trial], profile: HandProfile) -> None:
    import matplotlib

    # Set before pyplot is imported: these runs are headless.
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    steady = [t.t_close_steady_s * 1e3 for t in trials if not math.isnan(t.t_close_steady_s)]
    tick = [t.t_close_tick_s * 1e3 for t in trials if not math.isnan(t.t_close_tick_s)]
    seqs = [t.index for t in trials if not math.isnan(t.t_close_steady_s)]

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    axes[0][0].plot(seqs, steady, ".", ms=4)
    axes[0][0].set_title("T_close per trial (steady clock)")
    axes[0][0].set_xlabel("trial")
    axes[0][0].set_ylabel("T_close [ms]")

    axes[0][1].hist(steady, bins=40)
    axes[0][1].set_title("T_close distribution (steady)")
    axes[0][1].set_xlabel("T_close [ms]")

    axes[1][0].plot(seqs, tick, ".", ms=4, color="tab:green")
    axes[1][0].set_title("T_close per trial (tick x dt)")
    axes[1][0].set_xlabel("trial")
    axes[1][0].set_ylabel("T_close [ms]")

    if steady and tick:
        axes[1][1].plot(tick, steady, ".", ms=4, color="tab:red")
        lo = min(min(tick), min(steady))
        hi = max(max(tick), max(steady))
        axes[1][1].plot([lo, hi], [lo, hi], "-", lw=1, color="0.5")
        axes[1][1].set_title("steady vs tick (gap = host stall)")
        axes[1][1].set_xlabel("tick x dt [ms]")
        axes[1][1].set_ylabel("steady [ms]")

    for row in axes:
        for ax in row:
            ax.grid(alpha=0.3)
    fig.suptitle(f"T_close,e2e(eta={profile.eta_close}) — {len(trials)} trials")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
