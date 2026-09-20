"""Clock phase error per launch trial (dynamic_catching D-3 / S3.1a).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §5. D-3 is judged on the
**per-trial clock phase error**, not on a rate. The existing RTF signal averages
over 200 steps, so a 0.5 s flight carries one or two samples and a short stall is
erased by the mean; and the throttle baseline is only reset when ``max_rtf``
changes, so a stall is followed by a catch-up stretch with RTF > 1. Neither
survives as evidence.

This reads the simulator's clock lane CSV
(``clock_lane.csv_path``, written by ``rtc_mujoco_sim``) and computes, per trial,
exactly what §5 defines:

    delta(t)  = (steady(t) - steady_0) - (sim(t) - sim_0)
    delta_max = max |delta| over the flight
    max pause = max over single steps of (d_wall - d_sim)

with t = 0 at the launch instant. Both directions are kept: falling behind and
catching up are both phase error, and a signed summary that averaged them would
report a stalling simulator as a healthy one.

**The window is the FLIGHT, not the run.** delta accumulates from the launch
instant, so a window left open until the next launch would report the idle time
between throws as clock error and delta_max would grow with the gap rather than
with the defect. Trials are cut by ``launch_seq`` and ``ball_active``, which the
lane records for this reason.

**The verdict is NOT_EVALUATED and this tool does not change that.** The
validity condition

    v_max * delta_max + 0.5 * a_bound * delta_max^2 <= eps_clk_alloc
    max pause                                       <= eps_clk_alloc / v_max

needs ``eps_clk_alloc``, the share of the L3 §4.6 error budget given to the clock
term, and that share depends on ``r_cap`` (TBD-HAND-04), which is not decided.
So the tool reports the distributions and *inverts* the condition instead: for
the observed trials it prints the smallest ``eps_clk_alloc`` that would admit a
given fraction of them. That number is a **proposal for the user to confirm**
(plan §5, §7.3) — reading it back as a measured threshold would be circular, and
the tool says so in its own output rather than leaving the reader to remember.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

# Plan §5: v_max is the fastest ball speed in the target throw distribution and
# a_bound is g plus the drag acceleration ceiling. Until S3.5b fixes the
# distribution these are the S0.7 assumed values, and the provenance says so.
DEFAULT_V_MAX_M_S = 8.4  # S0.7 sweep: catch speed up to 8.4 m/s at T_f 1.5 s
DEFAULT_A_BOUND_M_S2 = 9.81 + 0.0229 * 8.4 * 8.4  # g + k|v|^2, k from L0 §4.1


@dataclass
class Trial:
    launch_seq: int
    steps: int
    flight_sim_s: float
    delta_max_s: float
    delta_signed_extreme_s: float  # the extreme with its sign kept
    max_pause_s: float


@dataclass
class LaneStats:
    trials: list[Trial]
    dropped_total: int
    rows: int


def read_lane(path: Path) -> LaneStats:
    """Parse the clock lane CSV into per-trial statistics."""
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"step", "sim_time_sec", "steady_ns", "launch_seq", "ball_active"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(
                f"{path}: missing column(s) {sorted(missing)}. This tool needs a lane "
                "written by a simulator that records launch_seq/ball_active; an older "
                "CSV cannot be segmented into trials and silently averaging the whole "
                "run instead would report the idle time between throws as clock error."
            )
        rows = list(reader)

    dropped = 0
    if rows and "dropped_total" in rows[-1]:
        dropped = int(rows[-1]["dropped_total"])

    trials: list[Trial] = []
    current: list[tuple[float, float]] = []  # (sim_s, steady_s)
    current_seq = -1

    def close() -> None:
        nonlocal current, current_seq
        if len(current) >= 2 and current_seq > 0:
            trials.append(_summarise(current_seq, current))
        current = []

    for row in rows:
        seq = int(row["launch_seq"])
        active = row["ball_active"] not in ("0", "false", "False", "")
        if not active or seq != current_seq:
            close()
            current_seq = seq
        if active:
            current.append((float(row["sim_time_sec"]), float(row["steady_ns"]) * 1e-9))
    close()
    return LaneStats(trials=trials, dropped_total=dropped, rows=len(rows))


def _summarise(seq: int, samples: list[tuple[float, float]]) -> Trial:
    sim0, steady0 = samples[0]
    delta_max = 0.0
    signed = 0.0
    max_pause = 0.0
    for i, (sim_s, steady_s) in enumerate(samples):
        delta = (steady_s - steady0) - (sim_s - sim0)
        if abs(delta) > delta_max:
            delta_max = abs(delta)
            signed = delta
        if i > 0:
            prev_sim, prev_steady = samples[i - 1]
            # One step's wall time minus one step's sim time: the stall, if any,
            # that this single step took. Negative values are catch-up and are
            # not pauses.
            max_pause = max(max_pause, (steady_s - prev_steady) - (sim_s - prev_sim))
    return Trial(
        launch_seq=seq,
        steps=len(samples),
        flight_sim_s=samples[-1][0] - sim0,
        delta_max_s=delta_max,
        delta_signed_extreme_s=signed,
        max_pause_s=max_pause,
    )


def required_eps(trial: Trial, v_max: float, a_bound: float) -> float:
    """Smallest eps_clk_alloc that would make this trial valid (plan §5)."""
    from_delta = v_max * trial.delta_max_s + 0.5 * a_bound * trial.delta_max_s**2
    from_pause = trial.max_pause_s * v_max
    return max(from_delta, from_pause)


def quantile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil(q * len(ordered))) - 1))
    return ordered[idx]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", type=Path, help="clock lane CSV (clock_lane.csv_path)")
    parser.add_argument(
        "--v-max",
        type=float,
        default=DEFAULT_V_MAX_M_S,
        help="fastest ball speed [m/s] (S0.7 assumption until S3.5b)",
    )
    parser.add_argument(
        "--a-bound",
        type=float,
        default=DEFAULT_A_BOUND_M_S2,
        help="acceleration ceiling [m/s^2] = g + drag",
    )
    parser.add_argument(
        "--admit",
        type=float,
        default=0.95,
        help="fraction of trials the proposed eps must admit (plan: 1 - 5%%)",
    )
    parser.add_argument("--plot", type=Path, default=None, help="write a delta/pause plot here")
    args = parser.parse_args(argv)

    stats = read_lane(args.csv)
    if not stats.trials:
        print(
            f"{args.csv}: no complete flight found ({stats.rows} rows). "
            "Was the ball launched, and was the lane enabled for the whole run?",
            file=sys.stderr,
        )
        return 2

    deltas = [t.delta_max_s for t in stats.trials]
    pauses = [t.max_pause_s for t in stats.trials]
    eps = [required_eps(t, args.v_max, args.a_bound) for t in stats.trials]

    print(f"clock lane : {args.csv}")
    print(f"rows       : {stats.rows}")
    print(
        f"trials     : {len(stats.trials)}"
        f"   (flight {min(t.flight_sim_s for t in stats.trials):.3f}"
        f"–{max(t.flight_sim_s for t in stats.trials):.3f} s sim)"
    )
    if stats.dropped_total:
        # Not a footnote. A lane that overflowed lost samples exactly where the
        # phase error was worst, so the tail below is a lower bound on the real
        # one and every quantile past the drop is unsupported.
        print(
            f"!! DROPPED  : {stats.dropped_total} samples — the tail below is a LOWER BOUND. "
            "Raise clock_lane.drain_rate_hz and re-run before quoting these numbers."
        )
    print()
    print("                         p50        p95        max")
    print(
        f"  delta_max      [ms]  {quantile(deltas, 0.5) * 1e3:9.3f}  "
        f"{quantile(deltas, 0.95) * 1e3:9.3f}  {max(deltas) * 1e3:9.3f}"
    )
    print(
        f"  max pause      [ms]  {quantile(pauses, 0.5) * 1e3:9.3f}  "
        f"{quantile(pauses, 0.95) * 1e3:9.3f}  {max(pauses) * 1e3:9.3f}"
    )
    behind = sum(1 for t in stats.trials if t.delta_signed_extreme_s > 0)
    print(f"  extreme sign        : {behind} behind / {len(stats.trials) - behind} ahead")
    print()
    proposal = quantile(eps, args.admit)
    print(
        f"  eps_clk_alloc that admits {args.admit * 100:.0f}% of trials : "
        f"{proposal * 1e3:.3f} mm  (v_max {args.v_max} m/s, a_bound {args.a_bound:.2f} m/s^2)"
    )
    print()
    print("  VERDICT: NOT_EVALUATED — eps_clk_alloc is not decided (it needs r_cap,")
    print("  TBD-HAND-04). The number above is a PROPOSAL for the user to confirm")
    print("  (plan §5, §7.3); adopting it because this run produced it would make the")
    print("  threshold a restatement of the measurement rather than a budget.")

    if args.plot is not None:
        _write_plot(args.plot, stats.trials)
        print(f"\n  plot -> {args.plot}")
    return 0


def _write_plot(path: Path, trials: list[Trial]) -> None:
    import matplotlib

    # Set before pyplot is imported: these runs are headless.
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    deltas = [t.delta_max_s * 1e3 for t in trials]
    pauses = [t.max_pause_s * 1e3 for t in trials]
    seqs = [t.launch_seq for t in trials]

    axes[0][0].plot(seqs, deltas, ".", ms=4)
    axes[0][0].set_title("delta_max per trial")
    axes[0][0].set_xlabel("launch_seq")
    axes[0][0].set_ylabel("delta_max [ms]")

    axes[0][1].hist(deltas, bins=40)
    axes[0][1].set_title("delta_max distribution")
    axes[0][1].set_xlabel("delta_max [ms]")

    axes[1][0].plot(seqs, pauses, ".", ms=4, color="tab:red")
    axes[1][0].set_title("max pause per trial")
    axes[1][0].set_xlabel("launch_seq")
    axes[1][0].set_ylabel("max pause [ms]")

    axes[1][1].hist(pauses, bins=40, color="tab:red")
    axes[1][1].set_title("max pause distribution")
    axes[1][1].set_xlabel("max pause [ms]")

    for row in axes:
        for ax in row:
            ax.grid(alpha=0.3)
    fig.suptitle("D-3 clock phase error per launch trial (verdict: NOT_EVALUATED)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
