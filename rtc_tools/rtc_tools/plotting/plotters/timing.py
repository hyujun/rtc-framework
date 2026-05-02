"""CM RT loop timing plotters: per-phase breakdown, total + jitter, histograms,
and statistics summary.
"""

from pathlib import Path

import matplotlib.pyplot as plt

# First-tick init cost (auto-hold InitializeHoldPosition + first RCLCPP_INFO
# lazy-init inside ControlLoop) shows up as one-time t_total_us spike on
# tick 1-2. The raw CSV preserves it for debugging; plotters and percentile
# stats exclude it via detect_outlier_indices() so a single 340 ms tick
# doesn't drown out the steady-state distribution.
DEFAULT_OUTLIER_FACTOR = 50.0
DEFAULT_FIRST_TICKS_SUSPECT = 2


def detect_outlier_indices(
    df,
    outlier_factor: float = DEFAULT_OUTLIER_FACTOR,
    first_ticks: int = DEFAULT_FIRST_TICKS_SUSPECT,
):
    """Return the row indices of t_total_us outliers in `df`.

    A row is flagged when EITHER:
      - it is among the first `first_ticks` rows AND its t_total_us exceeds
        `outlier_factor` × median(t_total_us of remaining rows), OR
      - its t_total_us exceeds `outlier_factor` × median(t_total_us) anywhere
        in the dataset (catches mid-run anomalies, not just startup).

    The first-tick clause uses the median of *non-first* rows so a spike on
    row 0 doesn't inflate its own baseline. Returns an empty Index when no
    outliers are detected — safe for both robot mode (typically clean) and
    sim mode (first-tick init spike).
    """
    if "t_total_us" not in df.columns or len(df) == 0:
        return df.index[:0]

    tail = df.iloc[first_ticks:] if len(df) > first_ticks else df
    baseline = tail["t_total_us"].median() if len(tail) > 0 else df["t_total_us"].median()
    if not (baseline > 0):
        return df.index[:0]

    threshold = baseline * outlier_factor
    mask = df["t_total_us"] > threshold
    return df.index[mask]


def filter_outliers(df, **kwargs):
    """Return (df_clean, df_outliers) split by detect_outlier_indices()."""
    outlier_idx = detect_outlier_indices(df, **kwargs)
    return df.drop(outlier_idx), df.loc[outlier_idx]


def infer_budget_us(df, fallback_us: float = 2000.0) -> float:
    """Return the per-tick period budget in microseconds, robust to outliers.

    Uses the median of `diff(timestamp)` rather than `t.iloc[1] - t.iloc[0]`
    so a first-tick init spike (e.g. 340 ms gap between row 0 and row 1 in
    sim mode) does not poison the inferred budget. Falls back to
    `fallback_us` when the dataframe has fewer than 2 rows or when
    timestamps are missing/non-monotonic.
    """
    if "timestamp" not in df.columns or len(df) < 2:
        return fallback_us
    diffs = df["timestamp"].diff().dropna()
    diffs = diffs[diffs > 0]
    if len(diffs) == 0:
        return fallback_us
    period_s = diffs.median()
    return float(period_s * 1e6) if period_s > 0 else fallback_us


def plot_timing_breakdown(df, save_dir=None):
    """Figure 1: Per-phase timing breakdown (stacked area)."""
    df, _ = filter_outliers(df)
    fig, ax = plt.subplots(figsize=(14, 6))
    fig.suptitle("Control Loop Timing Breakdown", fontsize=16, fontweight="bold")

    t = df["timestamp"]
    phases = ["t_state_us", "t_compute_us", "t_publish_us"]
    labels = ["State Acquire", "Compute", "Publish"]
    colors = ["#2196F3", "#FF9800", "#4CAF50"]

    available = [
        (p, l, c) for p, l, c in zip(phases, labels, colors, strict=False) if p in df.columns
    ]
    if available:
        ax.stackplot(
            t,
            *[df[p] for p, _, _ in available],
            labels=[l for _, l, _ in available],
            colors=[c for _, _, c in available],
            alpha=0.7,
        )

    # Budget line — median(diff) is robust to first-tick init spikes
    # (raw t.iloc[1] - t.iloc[0] would absorb the 340 ms gap in sim mode).
    budget = infer_budget_us(df)
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
    df, _ = filter_outliers(df)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle("Loop Timing & Jitter", fontsize=16, fontweight="bold")

    t = df["timestamp"]

    # Budget estimation — see infer_budget_us docstring.
    budget = infer_budget_us(df)

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
        ax2.plot(t, df["jitter_us"], linewidth=0.5, alpha=0.7, color="C1", label="Jitter")
        title = "Period Jitter"
        if len(df) > 50 and df["jitter_us"].max() == 0.0:
            # Sim mode (CV-driven wakeup): the producer suppresses jitter
            # because |actual_period − budget| would just measure sim
            # cadence noise, not RT timing. Make this self-evident.
            title += " (not measured — non-deadline wakeup)"
        ax2.set_title(title)
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
    df, _ = filter_outliers(df)
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

    for ax, (col, label) in zip(axes, cols, strict=False):
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
    """Print timing statistics summary.

    Outliers (typically the first-tick init spike — see
    detect_outlier_indices) are listed separately and excluded from
    mean/std/percentile/overrun computation so a single 340 ms tick
    doesn't drown out the steady-state distribution. The raw CSV is
    untouched.
    """
    duration = df["timestamp"].iloc[-1] - df["timestamp"].iloc[0]
    rate = len(df) / duration if duration > 0 else 0

    print("\n=== Timing Statistics ===")
    print(f"Duration: {duration:.2f} s | Samples: {len(df)} | Rate: {rate:.1f} Hz")

    df_clean, df_outliers = filter_outliers(df)
    if len(df_outliers) > 0:
        print(f"\nOutliers excluded from stats: {len(df_outliers)}")
        for _, row in df_outliers.iterrows():
            tick = int(row["tick_count"]) if "tick_count" in row else -1
            t_total = row.get("t_total_us", float("nan"))
            t_state = row.get("t_state_us", float("nan"))
            t_compute = row.get("t_compute_us", float("nan"))
            t_publish = row.get("t_publish_us", float("nan"))
            print(
                f"  tick={tick:>6}  t_total={t_total:>10.1f} µs  "
                f"(state={t_state:.1f} compute={t_compute:.1f} publish={t_publish:.1f})"
            )

    # Budget estimation uses median(diff) on the cleaned dataframe so the
    # first-tick spike doesn't bias the inferred period (which feeds the
    # overrun threshold). See infer_budget_us docstring.
    budget = infer_budget_us(df_clean)

    for col, label in [
        ("t_state_us", "State Acquire"),
        ("t_compute_us", "Compute"),
        ("t_publish_us", "Publish"),
        ("t_total_us", "Total Loop"),
        ("jitter_us", "Jitter"),
    ]:
        if col not in df_clean.columns:
            continue
        data = df_clean[col]
        print(f"\n{label}:")
        print(f"  Mean: {data.mean():.2f} µs | Std: {data.std():.2f} µs")
        print(f"  Min: {data.min():.2f} µs | Max: {data.max():.2f} µs")
        print(
            f"  P50: {data.quantile(0.50):.2f} µs"
            f" | P95: {data.quantile(0.95):.2f} µs"
            f" | P99: {data.quantile(0.99):.2f} µs"
        )

    if "t_total_us" in df_clean.columns and len(df_clean) > 0:
        overruns = (df_clean["t_total_us"] > budget).sum()
        pct = overruns / len(df_clean) * 100
        print(f"\nOverruns (>{budget:.0f} µs): {overruns} ({pct:.3f}%)")
