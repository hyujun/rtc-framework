"""Pull-force estimator plotters for the controller-owned pull_estimator.csv (#167).

PullEstimatorLog is a per-tick single-row channel shared by the three demo
controllers (joint / task / wbc):

  - reference-frame estimate (`force_x/y/z` filtered + `force_raw_x/y/z`
    pre-filter — the CSV is the only surface carrying the raw series)
  - plane coordinates (`inplane_x/y`), `magnitude`, `directional`
  - slip diagnostics (`friction_utilization`, `leakage_bound`)
  - validity gates (`valid`, `valid_contact_count`, `slip_risk`,
    `any_saturated`, `baseline_applied`)
  - observed grasp shape (`opposing_mask` — bit k set means contact k opposed
    the thumb on that tick, so a normal that jumps because the grasp was
    regripped is distinguishable from one that jumps because of a force step)

`plot_pull_estimator` renders one N×1 sharex figure so zooming any panel
stays synchronized (same convention as `plot_wbc_diag_solver`).
"""

from pathlib import Path

import matplotlib.pyplot as plt

_FORCE_COMPONENTS = ("x", "y", "z")


def plot_pull_estimator(df, save_dir=None):
    """Pull-force estimate overview (4×1, sharex).

    ① filtered Fx/Fy/Fz (+ raw overlays at low alpha)
    ② in-plane components, |F|, directional scalar
    ③ friction utilization (slip ratio) + grip→in-plane leakage bound
    ④ validity/risk flags (step) + valid contact count
    """
    required = [f"force_{c}" for c in _FORCE_COMPONENTS] + ["magnitude"]
    if any(c not in df.columns for c in required):
        print("  Skipping pull estimator plot (force_*/magnitude columns not found)")
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    fig.suptitle("Pull Force Estimate", fontsize=16, fontweight="bold")
    t = df["timestamp"]

    # ── ① Reference-frame force: filtered solid, raw faint ──
    ax = axes[0]
    for i, comp in enumerate(_FORCE_COMPONENTS):
        color = f"C{i}"
        ax.plot(t, df[f"force_{comp}"], label=f"F{comp}", linewidth=1.3, color=color)
        raw_col = f"force_raw_{comp}"
        if raw_col in df.columns:
            ax.plot(t, df[raw_col], linewidth=0.9, color=color, alpha=0.3)
    ax.set_ylabel("Force (N)")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)
    ax.set_title("Reference-frame estimate (solid = filtered, faint = raw)", fontsize=10)

    # ── ② In-plane coordinates + magnitude + directional ──
    ax = axes[1]
    for i, comp in enumerate(("x", "y")):
        col = f"inplane_{comp}"
        if col in df.columns:
            ax.plot(t, df[col], label=f"inplane_{comp}", linewidth=1.2, color=f"C{i}")
    ax.plot(t, df["magnitude"], label="|F|", linewidth=1.4, color="C3")
    if "directional" in df.columns:
        ax.plot(t, df["directional"], label="directional", linewidth=1.0, color="C4", alpha=0.8)
    ax.set_ylabel("Force (N)")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    # ── ③ Slip diagnostics ──
    ax = axes[2]
    if "friction_utilization" in df.columns:
        ax.plot(
            t, df["friction_utilization"], label="friction_utilization", linewidth=1.2, color="C3"
        )
        ax.axhline(y=1.0, color="red", linestyle="--", alpha=0.5, label="slip boundary (1.0)")
    if "leakage_bound" in df.columns:
        ax.plot(t, df["leakage_bound"], label="leakage_bound (N)", linewidth=1.0, color="C0")
    ax.set_ylabel("ratio / N")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── ④ Validity gates + contact count ──
    ax = axes[3]
    flag_colors = (
        ("valid", "C2"),
        ("slip_risk", "C3"),
        ("any_saturated", "C1"),
        ("baseline_applied", "C4"),
    )
    for col, color in flag_colors:
        if col in df.columns:
            ax.step(t, df[col], label=col, where="post", linewidth=1.1, color=color)
    if "valid_contact_count" in df.columns:
        ax.plot(
            t,
            df["valid_contact_count"],
            label="valid_contact_count",
            linewidth=1.0,
            color="C0",
            alpha=0.7,
        )
    if "opposing_mask" in df.columns:
        # The mask is a bitmask, not a magnitude — step it so a shape change
        # reads as one discrete transition rather than a slope, and keep it on
        # this panel so it lines up with the validity gates it explains.
        ax.step(
            t,
            df["opposing_mask"],
            label="opposing_mask",
            where="post",
            linewidth=1.0,
            color="C5",
            alpha=0.8,
        )
    ax.set_ylabel("flags / count")
    ax.set_xlabel("Time (s)")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "pull_estimator.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()


def print_pull_estimator_statistics(df):
    """Console summary of the pull estimate + validity gates."""
    print("\n=== Pull Force Estimator ===")
    n = len(df)
    print(f"Samples: {n}")
    if "magnitude" in df.columns and n > 0:
        m = df["magnitude"]
        print(f"|F| (N): mean={m.mean():.3f} p50={m.median():.3f} max={m.max():.3f}")
    if "friction_utilization" in df.columns and n > 0:
        fu = df["friction_utilization"]
        print(f"Friction utilization: mean={fu.mean():.3f} max={fu.max():.3f}")
    if "leakage_bound" in df.columns and n > 0:
        print(f"Leakage bound (N): max={df['leakage_bound'].max():.3f}")
    for flag in ("valid", "slip_risk", "any_saturated", "baseline_applied"):
        if flag in df.columns and n > 0:
            pct = df[flag].astype(float).mean() * 100
            print(f"{flag}: {pct:.1f}% of ticks")
    if "opposing_mask" in df.columns and n > 0:
        # How many distinct grasp shapes the run went through, and which one it
        # spent most of its time in — a run that never settles on one mask is
        # the signature of a marginal tip flapping in and out of the pinch.
        masks = df["opposing_mask"].dropna().astype(int)
        if not masks.empty:
            counts = masks.value_counts()
            top = counts.index[0]
            pct = counts.iloc[0] / len(masks) * 100
            print(
                f"Opposing set: {counts.size} distinct shape(s), "
                f"dominant mask=0b{top:04b} ({pct:.1f}% of ticks), "
                f"changes={int((masks.diff().fillna(0) != 0).sum())}"
            )
    if "valid_contact_count" in df.columns and n > 0:
        # dropna() guards int(NaN) on a truncated/partial CSV (all-NaN column).
        vcc = df["valid_contact_count"].dropna()
        if not vcc.empty:
            print(f"Valid contacts: mean={vcc.mean():.2f} max={int(vcc.max())}")
