"""Pull-force estimator plotters for the controller-owned pull_estimator.csv (#167).

PullEstimatorLog is a per-tick single-row channel shared by the three demo
controllers (joint / task / wbc):

  - reference-frame estimate (`force_x/y/z` filtered + `force_prefilter_x/y/z`
    — the CSV is the only surface carrying the pre-filter series)
  - plane coordinates (`inplane_x/y`), `magnitude`, `directional`
  - the plane and basis that name those coordinates (`plane_normal_*`,
    `basis_x_*`, `basis_source`) — without them the in-plane pair is not
    comparable across ticks, since the basis is rebuilt from the observed
    pinch axis every tick (#234 P-5)
  - slip diagnostics (`friction_utilization`, `leakage_bound`)
  - validity gates (`valid`, `invalid_reason`, `valid_contact_count`,
    `slip_risk`, `any_saturated`, `baseline_applied`)
  - per-contact sets: `contact_mask` (in the force sum), `touch_mask`
    (pressing, axis-independent) and `opposing_mask` (opposed the thumb, i.e.
    the observed grasp shape) — so a normal that jumps because the grasp was
    regripped is distinguishable from one that jumps because of a force step,
    and a dropped tip is attributable to a finger

Schema note: sessions recorded before #234 carry `force_raw_*` instead of
`force_prefilter_*` and lack every column added there. Both are accepted; the
panels that have no data are simply left empty.

`plot_pull_estimator` renders one N×1 sharex figure so zooming any panel
stays synchronized (same convention as `plot_wbc_diag_solver`).
"""

from pathlib import Path

import matplotlib.pyplot as plt

_FORCE_COMPONENTS = ("x", "y", "z")

# Legacy (pre-#234) name for the pre-filter series, still read so archived
# sessions keep plotting.
_PREFILTER_PREFIXES = ("force_prefilter_", "force_raw_")

# rtc::grasp::PullBasisSource / PullInvalidReason wire values.
_BASIS_SOURCE_NAMES = {0: "none", 1: "reference", 2: "carry", 3: "seed"}
_INVALID_REASON_NAMES = {
    0: "valid",
    1: "not-initialized",
    2: "degenerate-normal",
    3: "required-contact",
    4: "insufficient-contacts",
}


def _prefilter_col(df, comp):
    """Return the pre-filter column name for `comp`, or None if absent."""
    for prefix in _PREFILTER_PREFIXES:
        col = f"{prefix}{comp}"
        if col in df.columns:
            return col
    return None


def _format_mask(value, roles):
    """Render a mask as role names when the bit order is known, else binary.

    `roles` comes from `df.attrs["mask_roles"]` (stamped into the CSV header);
    an empty list means the file predates the stamp, so fall back to a width-8
    binary string — the mask is uint8 (kMaxPullContacts <= 8), and the old
    4-bit assumption silently truncated a 5+-contact hand.
    """
    v = int(value)
    if not roles:
        return f"0b{v:08b}"
    names = [r for i, r in enumerate(roles) if v & (1 << i)]
    return "+".join(names) if names else "none"


def plot_pull_estimator(df, save_dir=None):
    """Pull-force estimate overview (5×1, sharex).

    ① filtered Fx/Fy/Fz (+ pre-filter overlays at low alpha)
    ② in-plane components, |F|, directional scalar
    ③ plane normal + in-plane basis x (the frame that names panel ②)
    ④ friction utilization (slip ratio) + grip→in-plane leakage bound
    ⑤ validity/risk flags (step) + valid contact count + contact/touch masks
    """
    required = [f"force_{c}" for c in _FORCE_COMPONENTS] + ["magnitude"]
    if any(c not in df.columns for c in required):
        print("  Skipping pull estimator plot (force_*/magnitude columns not found)")
        return

    fig, axes = plt.subplots(5, 1, figsize=(14, 17), sharex=True)
    fig.suptitle("Pull Force Estimate", fontsize=16, fontweight="bold")
    t = df["timestamp"]

    # ── ① Reference-frame force: filtered solid, pre-filter faint ──
    ax = axes[0]
    for i, comp in enumerate(_FORCE_COMPONENTS):
        color = f"C{i}"
        ax.plot(t, df[f"force_{comp}"], label=f"F{comp}", linewidth=1.3, color=color)
        pre_col = _prefilter_col(df, comp)
        if pre_col is not None:
            ax.plot(t, df[pre_col], linewidth=0.9, color=color, alpha=0.3)
    ax.set_ylabel("Force (N)")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)
    # "pre-filter" is post-projection/gravity/baseline, not a raw sensor sum —
    # the old "raw" wording made the baseline-capture step read as a filter
    # artefact (#234 P-13).
    ax.set_title("Reference-frame estimate (solid = filtered, faint = pre-filter)", fontsize=10)

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

    # ── ③ The plane + basis that name panel ② ──
    # The basis is rebuilt from the observed pinch axis every tick, so an
    # in-plane trace can step purely because the axis rotated. Plotting the
    # normal and e_x directly under it is what separates the two (#234 P-5);
    # basis_source is overlaid because only `reference` (1) means
    # inplane_x really is the configured reference direction.
    ax = axes[2]
    plotted_frame = False
    for i, comp in enumerate(_FORCE_COMPONENTS):
        col = f"plane_normal_{comp}"
        if col in df.columns:
            ax.plot(t, df[col], label=f"n_{comp}", linewidth=1.1, color=f"C{i}")
            plotted_frame = True
    for i, comp in enumerate(_FORCE_COMPONENTS):
        col = f"basis_x_{comp}"
        if col in df.columns:
            ax.plot(
                t,
                df[col],
                label=f"ex_{comp}",
                linewidth=1.0,
                color=f"C{i}",
                linestyle="--",
                alpha=0.7,
            )
            plotted_frame = True
    if "basis_source" in df.columns:
        ax.step(
            t,
            df["basis_source"],
            label="basis_source (1=reference)",
            where="post",
            linewidth=1.0,
            color="C6",
            alpha=0.8,
        )
        plotted_frame = True
    ax.set_ylabel("unit vector / enum")
    if plotted_frame:
        ax.legend(fontsize=8, ncol=4)
    else:
        # Pre-#234 session: the columns simply do not exist.
        ax.text(
            0.5,
            0.5,
            "no plane/basis columns (pre-#234 log)",
            ha="center",
            va="center",
            transform=ax.transAxes,
            fontsize=9,
            alpha=0.6,
        )
    ax.grid(True, alpha=0.3)

    # ── ④ Slip diagnostics ──
    ax = axes[3]
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

    # ── ⑤ Validity gates + contact count ──
    ax = axes[4]
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
    # Masks are bitmasks, not magnitudes — step them so a shape change reads as
    # one discrete transition rather than a slope, and keep them on this panel
    # so they line up with the validity gates they explain. contact_mask is the
    # set that actually entered the force sum; opposing_mask (touch-derived) is
    # not (#234 P-12).
    for col, color in (("contact_mask", "C7"), ("touch_mask", "C8"), ("opposing_mask", "C5")):
        if col in df.columns:
            ax.step(t, df[col], label=col, where="post", linewidth=1.0, color=color, alpha=0.8)
    if "invalid_reason" in df.columns:
        ax.step(
            t,
            df["invalid_reason"],
            label="invalid_reason",
            where="post",
            linewidth=1.0,
            color="C9",
            alpha=0.8,
        )
    ax.set_ylabel("flags / count / mask")
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
    mask_roles = df.attrs.get("mask_roles", {})
    for col, label in (
        ("opposing_mask", "Opposing set"),
        ("contact_mask", "Contact set (in the force sum)"),
        ("touch_mask", "Touch set"),
    ):
        if col not in df.columns or n == 0:
            continue
        # How many distinct sets the run went through, and which one it spent
        # most of its time in — a run that never settles on one mask is the
        # signature of a marginal tip flapping in and out.
        masks = df[col].dropna().astype(int)
        if masks.empty:
            continue
        counts = masks.value_counts()
        top = counts.index[0]
        pct = counts.iloc[0] / len(masks) * 100
        print(
            f"{label}: {counts.size} distinct shape(s), "
            f"dominant={_format_mask(top, mask_roles.get(col, []))} "
            f"({pct:.1f}% of ticks), "
            f"changes={int((masks.diff().fillna(0) != 0).sum())}"
        )
    if "invalid_reason" in df.columns and n > 0:
        # Which gate actually failed — `valid=0` alone cannot distinguish a
        # degenerate pinch axis from a dropped thumb from too few tips, which
        # is what made joint-vs-WBC traces incomparable (#234 P-11).
        reasons = df["invalid_reason"].dropna().astype(int)
        nonzero = reasons[reasons != 0]
        if not nonzero.empty:
            parts = [
                f"{_INVALID_REASON_NAMES.get(int(k), int(k))}={c / len(reasons) * 100:.1f}%"
                for k, c in nonzero.value_counts().items()
            ]
            print("Invalid ticks by reason: " + ", ".join(parts))
    if "basis_source" in df.columns and n > 0:
        sources = df["basis_source"].dropna().astype(int)
        if not sources.empty:
            parts = [
                f"{_BASIS_SOURCE_NAMES.get(int(k), int(k))}={c / len(sources) * 100:.1f}%"
                for k, c in sources.value_counts().items()
            ]
            print("Basis source: " + ", ".join(parts))
    if "valid_contact_count" in df.columns and n > 0:
        # dropna() guards int(NaN) on a truncated/partial CSV (all-NaN column).
        vcc = df["valid_contact_count"].dropna()
        if not vcc.empty:
            print(f"Valid contacts: mean={vcc.mean():.2f} max={int(vcc.max())}")
    if "tick" in df.columns and n > 1:
        # Rows are pushed once per RT tick, so a gap here is a dropped row (SPSC
        # ring overflow), not a skipped tick — E-STOP ticks now log valid=0
        # rather than nothing (#234 P-20 / #235).
        ticks = df["tick"].dropna().astype("int64")
        if len(ticks) > 1:
            gaps = ticks.diff().iloc[1:]
            dropped = int((gaps - 1).clip(lower=0).sum())
            if dropped > 0:
                print(f"Dropped rows (tick gaps): {dropped} over {len(ticks)} logged ticks")
            if (gaps <= 0).any():
                print("WARNING: tick column is not strictly increasing (concatenated sessions?)")
