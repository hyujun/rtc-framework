"""MPC solver-timing plotter (`<session>/controllers/<config_key>/mpc_solve_timing.csv`).

Columns are nanoseconds (`min_ns`, `p50_ns`, `p99_ns`, `max_ns`, `mean_ns`,
`last_ns`, `count`); displayed values are in microseconds.
"""

from pathlib import Path

import matplotlib.pyplot as plt


def plot_mpc_solve_timing(df, save_dir=None):
    """Plot MPC solver latency p50/p99/max over time from
    `<session>/controllers/<config_key>/mpc_solve_timing.csv`. Columns are
    nanoseconds; this function converts them to microseconds for display.
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("MPC Solve Timing", fontsize=16, fontweight="bold")

    # X axis: relative wall time in seconds. t_wall_ns is steady_clock since
    # epoch, so anchor to the first sample for readability.
    t0 = df["t_wall_ns"].iloc[0]
    t = (df["t_wall_ns"] - t0) / 1e9

    # Convert ns → us
    p50_us = df["p50_ns"] / 1e3
    p99_us = df["p99_ns"] / 1e3
    max_us = df["max_ns"] / 1e3
    last_us = df["last_ns"] / 1e3
    mean_us = df["mean_ns"] / 1e3

    ax1.plot(t, p50_us, linewidth=1.0, label="p50", color="#2196F3")
    ax1.plot(t, p99_us, linewidth=1.0, label="p99", color="#FF9800")
    ax1.plot(t, max_us, linewidth=1.0, label="max", color="#F44336")
    ax1.plot(
        t, mean_us, linewidth=0.8, alpha=0.6, label="mean", color="#4CAF50"
    )
    ax1.set_ylabel("Solve time (µs)")
    ax1.legend(fontsize=9, loc="upper right")
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, last_us, linewidth=0.6, alpha=0.7, label="last sample", color="#9C27B0")
    ax2.set_xlabel("Time since first sample (s)")
    ax2.set_ylabel("Last solve (µs)")
    ax2.legend(fontsize=9, loc="upper right")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "mpc_solve_timing.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def print_mpc_timing_statistics(df):
    """Print MPC solve-timing summary in microseconds."""
    duration_s = (df["t_wall_ns"].iloc[-1] - df["t_wall_ns"].iloc[0]) / 1e9
    print("\n=== MPC Solve Timing ===")
    print(
        f"Duration: {duration_s:.2f} s | Snapshots: {len(df)} "
        f"| Last count: {int(df['count'].iloc[-1])}"
    )
    for col, label in [
        ("min_ns", "Min"),
        ("p50_ns", "P50 (median)"),
        ("p99_ns", "P99"),
        ("max_ns", "Max"),
        ("mean_ns", "Mean"),
    ]:
        if col not in df.columns:
            continue
        data_us = df[col] / 1e3
        print(
            f"{label:<14}  window-mean: {data_us.mean():.2f} µs"
            f"  window-max: {data_us.max():.2f} µs"
        )
