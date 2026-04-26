"""CM RT loop timing plotters: per-phase breakdown, total + jitter, histograms,
and statistics summary.
"""

from pathlib import Path

import matplotlib.pyplot as plt


def plot_timing_breakdown(df, save_dir=None):
    """Figure 1: Per-phase timing breakdown (stacked area)."""
    fig, ax = plt.subplots(figsize=(14, 6))
    fig.suptitle("Control Loop Timing Breakdown", fontsize=16, fontweight="bold")

    t = df["timestamp"]
    phases = ["t_state_acquire_us", "t_compute_us", "t_publish_us"]
    labels = ["State Acquire", "Compute", "Publish"]
    colors = ["#2196F3", "#FF9800", "#4CAF50"]

    available = [
        (p, l, c) for p, l, c in zip(phases, labels, colors) if p in df.columns
    ]
    if available:
        ax.stackplot(
            t,
            *[df[p] for p, _, _ in available],
            labels=[l for _, l, _ in available],
            colors=[c for _, _, c in available],
            alpha=0.7,
        )

    # Budget line
    if len(df) > 1:
        dt = t.iloc[1] - t.iloc[0]
        budget = 1e6 * dt if dt > 0 else 2000.0
    else:
        budget = 2000.0
    ax.axhline(
        y=budget,
        color="red",
        linestyle="--",
        linewidth=1.5,
        label=f"Budget ({budget:.0f} µs)",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Duration (µs)")
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "timing_breakdown.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_timing_total_and_jitter(df, save_dir=None):
    """Figure 2: Total loop time and jitter (2 subplots)."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle("Loop Timing & Jitter", fontsize=16, fontweight="bold")

    t = df["timestamp"]

    # Budget estimation
    if len(df) > 1:
        dt = t.iloc[1] - t.iloc[0]
        budget = 1e6 * dt if dt > 0 else 2000.0
    else:
        budget = 2000.0

    # Total loop time
    if "t_total_us" in df.columns:
        ax1.plot(t, df["t_total_us"], linewidth=0.5, alpha=0.7, label="t_total")
        ax1.axhline(
            y=budget,
            color="red",
            linestyle="--",
            linewidth=1.5,
            label=f"Budget ({budget:.0f} µs)",
        )
        overruns = (df["t_total_us"] > budget).sum()
        ax1.set_title(f"Total Loop Time (overruns: {overruns})")
    ax1.set_ylabel("Duration (µs)")
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Jitter
    if "jitter_us" in df.columns:
        ax2.plot(
            t, df["jitter_us"], linewidth=0.5, alpha=0.7, color="C1", label="Jitter"
        )
        ax2.set_title("Period Jitter")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Jitter (µs)")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "timing_total_jitter.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def plot_timing_histograms(df, save_dir=None):
    """Figure 3: Histograms of total loop time and jitter."""
    cols = []
    if "t_total_us" in df.columns:
        cols.append(("t_total_us", "Total Loop Time"))
    if "jitter_us" in df.columns:
        cols.append(("jitter_us", "Jitter"))
    if not cols:
        print("  Skipping timing histograms (columns not found)")
        return

    fig, axes = plt.subplots(1, len(cols), figsize=(7 * len(cols), 5))
    fig.suptitle("Timing Distributions", fontsize=16, fontweight="bold")
    if len(cols) == 1:
        axes = [axes]

    for ax, (col, label) in zip(axes, cols):
        data = df[col]
        ax.hist(data, bins=100, edgecolor="black", linewidth=0.3, alpha=0.7)
        p50 = data.quantile(0.50)
        p95 = data.quantile(0.95)
        p99 = data.quantile(0.99)
        ax.axvline(p50, color="green", linestyle="--", label=f"P50: {p50:.1f} µs")
        ax.axvline(p95, color="orange", linestyle="--", label=f"P95: {p95:.1f} µs")
        ax.axvline(p99, color="red", linestyle="--", label=f"P99: {p99:.1f} µs")
        ax.set_xlabel(f"{label} (µs)")
        ax.set_ylabel("Count")
        ax.set_title(label)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "timing_histograms.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def print_timing_statistics(df):
    """Print timing statistics summary."""
    duration = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]
    rate = len(df) / duration if duration > 0 else 0

    print("\n=== Timing Statistics ===")
    print(f"Duration: {duration:.2f} s | Samples: {len(df)} | Rate: {rate:.1f} Hz")

    # Budget estimation
    budget = 1e6 / rate if rate > 0 else 2000.0

    for col, label in [
        ("t_state_acquire_us", "State Acquire"),
        ("t_compute_us", "Compute"),
        ("t_publish_us", "Publish"),
        ("t_total_us", "Total Loop"),
        ("jitter_us", "Jitter"),
    ]:
        if col not in df.columns:
            continue
        data = df[col]
        print(f"\n{label}:")
        print(f"  Mean: {data.mean():.2f} µs | Std: {data.std():.2f} µs")
        print(f"  Min: {data.min():.2f} µs | Max: {data.max():.2f} µs")
        print(
            f"  P50: {data.quantile(0.50):.2f} µs"
            f" | P95: {data.quantile(0.95):.2f} µs"
            f" | P99: {data.quantile(0.99):.2f} µs"
        )

    if "t_total_us" in df.columns:
        overruns = (df["t_total_us"] > budget).sum()
        pct = overruns / len(df) * 100
        print(f"\nOverruns (>{budget:.0f} µs): {overruns} ({pct:.3f}%)")


