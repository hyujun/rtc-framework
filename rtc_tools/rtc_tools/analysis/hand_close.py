"""Hand closure time T_close,e2e(eta) from a device state CSV (dynamic_catching S4.2).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §4.4 S4a, law: ``L6_hand.md`` §4.2.

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
    )
    profile.validate()
    return profile


def rho(q: list[float], profile: HandProfile) -> float:
    """Closure progress in [.., 1] — see the module docstring. Minimum over C."""
    worst = math.inf
    for i in profile.caging_indices:
        span = profile.q_close[i] - profile.q_pre[i]
        # Direction, not magnitude: a joint whose closed pose is BELOW its
        # preshape closes by decreasing, and an unsigned ratio would report it
        # as moving backwards for the whole trial.
        progress = (q[i] - profile.q_pre[i]) * (1.0 if span > 0 else -1.0) / abs(span)
        worst = min(worst, progress)
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
    """Which shipped pose this command row is, by nearest distance.

    Compared by distance rather than equality because the controller clamps the
    target to the device limits before commanding it, so a pose that sits on a
    limit is commanded as the clamped value and an equality test would classify
    every row as 'other'.
    """
    d_pre = sum((command[i] - profile.q_pre[i]) ** 2 for i in range(len(profile.joint_names)))
    d_close = sum((command[i] - profile.q_close[i]) ** 2 for i in range(len(profile.joint_names)))
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
    if worst_spacing > profile.dt * (1.0 + TICK_AXIS_SPACING_TOLERANCE):
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
    finite = [v for v in values if not math.isnan(v)]
    if not finite:
        return {"n": 0, "mean": math.nan, "max": math.nan, "p99": math.nan}
    return {
        "n": len(finite),
        "mean": sum(finite) / len(finite),
        "max": max(finite),
        "p99": quantile(finite, 0.99),
    }


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
