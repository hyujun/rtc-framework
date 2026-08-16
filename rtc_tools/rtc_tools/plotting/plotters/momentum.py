"""Momentum-observer plotters for the controller-owned momentum_observer.csv.

MomentumObserverLog is a per-tick single-row channel shared by the three demo
controllers (joint / task / wbc) — the observer is one shared component, so the
channel shape is controller-independent. Three layers land on the SAME row,
because they are produced in one tick from one set of inputs and a reader
diagnosing a suspicious mass needs the residual that produced it on the same
line:

  - Layer 1b (#135) — joint-space residual `r_<joint>` in arm DEVICE order,
    `residual_inf_norm`, and the gate columns `valid` / `invalid_reason` /
    `ticks_since_seed`
  - Layer 2A (#135) — quasi-static payload wrench `payload_f*`/`payload_t*`,
    `payload_mass`, the §6.5 damping diagnostics `payload_sigma_min` /
    `payload_lambda_sq`, and `payload_fit_error`
  - Layer 2B (#455) — four-parameter inertial regression `inertial_mass`,
    `inertial_mc*` (m̂·ĉ), `inertial_c*` (ĉ), plus `inertial_sigma_min` /
    `inertial_rank` as the pose-diversity meter

Two figures, not one. Layers 2A/2B ship `enabled: false`, so a normal session
carries their columns pinned at zero; `has_payload_estimate` gates the second
figure on the reason code rather than on column presence so those sessions get
one clean residual figure instead of four empty panels. Sessions recorded
before #455 have no `inertial_*` columns at all and simply lose those series.

READING RULE that the plots and stats are both built around: on a row with
`valid=0` the residual is FROZEN at its last accepted value, not zeroed — a
zero would assert "no external torque" on a tick whose inputs were refused.
So the trace keeps drawing a number there that is not a measurement, which is
why invalid spans are shaded and why every ‖r‖∞ statistic is taken over valid
rows only.
"""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.transforms import blended_transform_factory

from rtc_tools.plotting.columns import (
    detect_joint_columns,
    has_inertial_estimate,
    has_payload_estimate,
)

# rtc::estimation::MomentumInvalidReason (momentum_observer.hpp). Decoded by
# name for the same reason PullEstimatorLog decodes its own: `valid=0` alone
# cannot separate a closed lane gate from a NaN input from a stalled clock,
# and a bare integer in a plot legend is not something a tuner can read.
_MOMENTUM_REASON_NAMES = {
    0: "valid",
    1: "not-initialized",
    2: "held",
    3: "short-input",
    4: "non-finite-input",
    5: "non-positive-dt",
}

# rtc::estimation::PayloadInvalidReason (payload_estimator.hpp).
_PAYLOAD_REASON_NAMES = {
    0: "valid",
    1: "not-initialized",
    2: "held",
    3: "short-input",
    4: "non-finite-input",
    5: "observer-invalid",
    6: "not-converged",
    7: "arm-moving",
    8: "hand-moving",
    9: "degenerate-gravity",
    10: "solver-failed",
    11: "rank-deficient",
    12: "poor-fit",
}

# rtc::estimation::InertialInvalidReason (inertial_estimator.hpp).
_INERTIAL_REASON_NAMES = {
    0: "valid",
    1: "not-initialized",
    2: "held",
    3: "short-input",
    4: "non-finite-input",
    5: "upstream-invalid",
    6: "insufficient-pose-diversity",
    7: "dynamic-excitation",
    8: "solver-failed",
    9: "non-positive-mass",
    10: "com-out-of-bounds",
}

_FORCE_COMPONENTS = ("fx", "fy", "fz")
_TORQUE_COMPONENTS = ("tx", "ty", "tz")
_AXES = ("x", "y", "z")


def _valid_mask(df, flag_col):
    """Boolean mask for `flag_col == 1`, or None when the column is absent.

    NaN counts as invalid: a truncated row has not shown that its inputs were
    accepted, and treating it as valid would let it into the ‖r‖∞ statistics.
    """
    if flag_col not in df.columns:
        return None
    return df[flag_col].fillna(0).astype(float) > 0.5


def _shade_invalid(ax, t, valid, label=None):
    """Shade the spans where the estimator refused the tick.

    `fill_between` over a blended transform paints the full y-range in ONE
    artist; a per-span `axvspan` loop would create thousands of artists on an
    800k-row session, which is the size these files actually are.
    """
    if valid is None or bool(valid.all()):
        return
    trans = blended_transform_factory(ax.transData, ax.transAxes)
    ax.fill_between(
        t,
        0,
        1,
        where=~valid,
        transform=trans,
        step="post",
        color="red",
        alpha=0.10,
        linewidth=0,
        label=label,
    )


def _name_reason_ticks(ax, series, names):
    """Label the y-axis with reason NAMES when few enough distinct ones appear."""
    seen = sorted(int(v) for v in series.dropna().unique())
    if not seen or len(seen) > 8:
        return
    ax.set_yticks(seen)
    ax.set_yticklabels([names.get(v, str(v)) for v in seen], fontsize=8)


def _reason_histogram(series, names):
    """`["held=41.2%", "hand-moving=3.0%"]` for the non-zero reasons present."""
    reasons = series.dropna().astype(int)
    if reasons.empty:
        return []
    nonzero = reasons[reasons != 0]
    if nonzero.empty:
        return []
    return [
        f"{names.get(int(k), int(k))}={c / len(reasons) * 100:.1f}%"
        for k, c in nonzero.value_counts().items()
    ]


def _reseed_count(df):
    """Number of re-seeds after the first, from `ticks_since_seed` stepping back.

    The counter is monotonic within one seed epoch, so a decrease is exactly a
    re-seed. Returns None when the column is absent.
    """
    if "ticks_since_seed" not in df.columns:
        return None
    seed = df["ticks_since_seed"].dropna()
    if len(seed) < 2:
        return 0
    return int((seed.diff().iloc[1:] < 0).sum())


def _legend_across_twin(ax, ax2, **kwargs):
    """One legend for a panel that carries a twin y-axis."""
    handles, labels = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    if handles or h2:
        ax.legend(handles + h2, labels + l2, **kwargs)


def _save(fig, save_dir, filename):
    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / filename
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close(fig)


# ── Layer 1b: the residual ─────────────────────────────────────────────────


def plot_momentum_observer(df, save_dir=None):
    """Joint-space residual overview (3×1, sharex) → `momentum_observer.png`.

    ① per-joint residual `r_<joint>` — arm DEVICE order, taken from the column
       names so the file decodes without that run's YAML
    ② ‖r‖∞ — the quantity the acceptance criterion is stated in
    ③ `invalid_reason` (named ticks) + `ticks_since_seed` on a twin axis, so a
       re-seed reads as a sawtooth right under the residual it explains

    Panels ① and ② shade the held spans: the residual there is the frozen
    previous value, not a measurement.
    """
    if "residual_inf_norm" not in df.columns:
        print("  Skipping momentum observer plot (residual_inf_norm column not found)")
        return

    joint_cols, joint_names = detect_joint_columns(df, "r_")
    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    fig.suptitle("Momentum Observer — joint-space residual", fontsize=16, fontweight="bold")
    t = df["timestamp"]
    valid = _valid_mask(df, "valid")

    # ── ① Per-joint residual ──
    ax = axes[0]
    for i, (col, name) in enumerate(zip(joint_cols, joint_names, strict=True)):
        ax.plot(t, df[col], label=name, linewidth=1.0, color=f"C{i % 10}")
    if not joint_cols:
        ax.text(
            0.5,
            0.5,
            "no r_<joint> columns",
            ha="center",
            va="center",
            transform=ax.transAxes,
            fontsize=9,
            alpha=0.6,
        )
    _shade_invalid(ax, t, valid, label="held (residual frozen)")
    ax.set_ylabel("r (N·m)")
    ax.set_title("Per-joint residual, arm device order", fontsize=10)
    ax.legend(fontsize=8, ncol=4)
    ax.grid(True, alpha=0.3)

    # ── ② ‖r‖∞ — the AC quantity ──
    ax = axes[1]
    ax.plot(t, df["residual_inf_norm"], linewidth=1.3, color="C3", label="‖r‖∞")
    _shade_invalid(ax, t, valid)
    ax.set_ylabel("‖r‖∞ (N·m)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── ③ Gate + convergence ──
    ax = axes[2]
    if "invalid_reason" in df.columns:
        ax.step(
            t,
            df["invalid_reason"],
            where="post",
            linewidth=1.1,
            color="C3",
            label="invalid_reason",
        )
        _name_reason_ticks(ax, df["invalid_reason"], _MOMENTUM_REASON_NAMES)
    ax.set_ylabel("gate")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    if "ticks_since_seed" in df.columns:
        ax2 = ax.twinx()
        ax2.plot(
            t,
            df["ticks_since_seed"],
            linewidth=1.0,
            color="C0",
            alpha=0.8,
            label="ticks_since_seed",
        )
        ax2.set_ylabel("ticks since seed")
        _legend_across_twin(ax, ax2, fontsize=8, ncol=2)
    else:
        ax.legend(fontsize=8)

    _save(fig, save_dir, "momentum_observer.png")


# ── Layers 2A / 2B: payload wrench, mass, inertial regression ──────────────


def plot_momentum_payload(df, save_dir=None):
    """Payload + inertial estimate (4×1, sharex) → `momentum_payload.png`.

    ① Layer 2A wrench at the payload frame (force solid, torque dashed)
    ② m̂ from both layers + ĉ on a twin axis — the two halves of one estimate
    ③ conditioning: σ_min / λ² / fit error on a log axis, because the choice to
       keep the §6.5 damping adaptive is only reviewable if the damping that
       was actually applied is visible
    ④ which gate closed, by name, plus `inertial_rank` (3 ⇒ still one pose)

    Layer 2B series are dropped when the session predates #455.
    """
    if "payload_mass" not in df.columns:
        print("  Skipping payload plot (payload_mass column not found)")
        return

    has_inertial = has_inertial_estimate(df)
    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    fig.suptitle("Momentum Observer — payload estimate", fontsize=16, fontweight="bold")
    t = df["timestamp"]
    payload_valid = _valid_mask(df, "payload_valid")

    # ── ① Wrench ──
    ax = axes[0]
    for i, comp in enumerate(_FORCE_COMPONENTS):
        col = f"payload_{comp}"
        if col in df.columns:
            ax.plot(t, df[col], label=comp, linewidth=1.2, color=f"C{i}")
    for i, comp in enumerate(_TORQUE_COMPONENTS):
        col = f"payload_{comp}"
        if col in df.columns:
            ax.plot(
                t, df[col], label=comp, linewidth=1.0, color=f"C{i}", linestyle="--", alpha=0.8
            )
    _shade_invalid(ax, t, payload_valid, label="gated (estimate frozen)")
    ax.set_ylabel("N / N·m")
    ax.set_title(
        "Payload wrench, LOCAL_WORLD_ALIGNED (solid = force, dashed = torque)", fontsize=10
    )
    ax.legend(fontsize=8, ncol=6)
    ax.grid(True, alpha=0.3)

    # ── ② Mass (both layers) + CoM ──
    ax = axes[1]
    ax.plot(t, df["payload_mass"], label="payload_mass (2A)", linewidth=1.3, color="C0")
    if has_inertial and "inertial_mass" in df.columns:
        ax.plot(t, df["inertial_mass"], label="inertial_mass (2B)", linewidth=1.3, color="C1")
    _shade_invalid(ax, t, payload_valid)
    ax.set_ylabel("m̂ (kg)")
    ax.grid(True, alpha=0.3)
    com_cols = [f"inertial_c{a}" for a in _AXES]
    if has_inertial and all(c in df.columns for c in com_cols):
        ax2 = ax.twinx()
        for i, (col, axis) in enumerate(zip(com_cols, _AXES, strict=True)):
            ax2.plot(t, df[col], label=f"ĉ{axis}", linewidth=0.9, color=f"C{i + 2}", alpha=0.75)
        ax2.set_ylabel("ĉ (m), payload frame")
        _legend_across_twin(ax, ax2, fontsize=8, ncol=3)
    else:
        ax.legend(fontsize=8, ncol=2)

    # ── ③ Conditioning / damping / fit ──
    ax = axes[2]
    diag_series = [
        ("payload_sigma_min", "σ_min (2A)", "C0"),
        ("payload_lambda_sq", "λ² applied (2A)", "C1"),
        ("payload_fit_error", "‖Jᵀŵ−r‖∞ (2A)", "C3"),
    ]
    if has_inertial:
        diag_series += [
            ("inertial_sigma_min", "σ_min (2B, pose diversity)", "C2"),
            ("inertial_fit_error", "fit error (2B)", "C4"),
        ]
    plotted_any = False
    for col, label, color in diag_series:
        if col in df.columns:
            ax.plot(t, df[col].abs(), label=label, linewidth=1.0, color=color)
            plotted_any = True
    if plotted_any:
        # Spans orders of magnitude — σ_min near a singularity and a fit error in
        # N·m do not share a linear axis. `symlog` keeps the exact zeros that a
        # well-conditioned tick legitimately reports.
        ax.set_yscale("symlog", linthresh=1e-9)
        ax.legend(fontsize=8, ncol=2)
    ax.set_ylabel("magnitude (symlog)")
    ax.grid(True, alpha=0.3)

    # ── ④ Which gate closed ──
    ax = axes[3]
    if "payload_reason" in df.columns:
        ax.step(
            t,
            df["payload_reason"],
            where="post",
            linewidth=1.1,
            color="C0",
            label="payload_reason",
        )
        _name_reason_ticks(ax, df["payload_reason"], _PAYLOAD_REASON_NAMES)
    ax.set_ylabel("payload gate")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    if has_inertial and ("inertial_reason" in df.columns or "inertial_rank" in df.columns):
        ax2 = ax.twinx()
        if "inertial_reason" in df.columns:
            ax2.step(
                t,
                df["inertial_reason"],
                where="post",
                linewidth=1.0,
                color="C1",
                alpha=0.85,
                label="inertial_reason",
            )
        if "inertial_rank" in df.columns:
            ax2.step(
                t,
                df["inertial_rank"],
                where="post",
                linewidth=1.0,
                color="C2",
                alpha=0.85,
                label="inertial_rank (3 = one pose)",
            )
        ax2.set_ylabel("inertial gate / rank")
        _legend_across_twin(ax, ax2, fontsize=8, ncol=3)
    else:
        ax.legend(fontsize=8)

    _save(fig, save_dir, "momentum_payload.png")


# ── Statistics ─────────────────────────────────────────────────────────────


def print_momentum_observer_statistics(df):
    """Console summary. This, not the figure, is the point of the channel.

    The sim negative control for #135 is stated as a number — "no load ⇒ ‖r‖∞
    below tol" — so the acceptance criterion has to be readable on stdout
    without opening a plot (the judgement `grasp_diag` already made).
    """
    print("\n=== Momentum Observer (Layer 1b) ===")
    n = len(df)
    print(f"Samples: {n}")
    if n == 0:
        return

    valid = _valid_mask(df, "valid")
    if valid is not None:
        n_valid = int(valid.sum())
        print(f"Valid ticks: {n_valid / n * 100:.1f}% ({n_valid}/{n})")

    if "residual_inf_norm" in df.columns:
        # Valid rows ONLY. A held row carries the frozen previous residual, so
        # including them weights whichever value happened to be held longest —
        # the statistic would describe the gate, not the arm.
        r = df["residual_inf_norm"]
        r = r[valid] if valid is not None else r
        r = r.dropna()
        if r.empty:
            print("‖r‖∞: no valid ticks — every row was refused by an input gate")
        else:
            print(
                f"‖r‖∞ over valid ticks (N·m): mean={r.mean():.4f} p50={r.median():.4f} "
                f"p99={r.quantile(0.99):.4f} max={r.max():.4f}"
            )

    if "invalid_reason" in df.columns:
        parts = _reason_histogram(df["invalid_reason"], _MOMENTUM_REASON_NAMES)
        if parts:
            print("Invalid ticks by reason: " + ", ".join(parts))

    reseeds = _reseed_count(df)
    if reseeds:
        # The window is ~3/K_I seconds, and neither K_I nor control_rate is in
        # the CSV — the tool cannot subtract it, so it says so instead of
        # quietly reporting a max that includes the converging ramp.
        print(
            f"Re-seeds: {reseeds} — the residual converges from ZERO after each, so the "
            f"~3/K_I s that follow are not yet evidence of 'no external torque'. "
            f"Check `ticks_since_seed` before reading a small ‖r‖∞."
        )

    if "tick" in df.columns and n > 1:
        # Rows are pushed once per RT tick, held and E-STOP ticks included, so a
        # gap here is a DROPPED row (SPSC ring overflow), never a tick the
        # controller chose not to log.
        ticks = df["tick"].dropna().astype("int64")
        if len(ticks) > 1:
            gaps = ticks.diff().iloc[1:]
            dropped = int((gaps - 1).clip(lower=0).sum())
            if dropped > 0:
                print(f"Dropped rows (tick gaps): {dropped} over {len(ticks)} logged ticks")
            if (gaps <= 0).any():
                print("WARNING: tick column is not strictly increasing (concatenated sessions?)")

    _print_payload_statistics(df, n)
    _print_inertial_statistics(df, n)


def _print_payload_statistics(df, n):
    if not has_payload_estimate(df):
        if "payload_reason" in df.columns:
            print("Payload estimator (Layer 2A): not configured this run")
        return
    print("\n=== Payload estimate (Layer 2A) ===")
    valid = _valid_mask(df, "payload_valid")
    if valid is not None:
        n_valid = int(valid.sum())
        print(f"Valid ticks: {n_valid / n * 100:.1f}% ({n_valid}/{n})")
    if "payload_mass" in df.columns:
        m = df["payload_mass"]
        m = m[valid] if valid is not None else m
        m = m.dropna()
        if not m.empty:
            print(
                f"m̂ over valid ticks (kg): mean={m.mean():.4f} p50={m.median():.4f} max={m.max():.4f}"
            )
    for col, label in (
        ("payload_fit_error", "Fit error ‖Jᵀŵ−r‖∞ (N·m)"),
        ("payload_sigma_min", "σ_min(J)"),
        ("payload_lambda_sq", "λ² applied"),
    ):
        if col in df.columns:
            s = df[col].dropna()
            if not s.empty:
                print(f"{label}: max={s.max():.4g}")
    if "payload_reason" in df.columns:
        parts = _reason_histogram(df["payload_reason"], _PAYLOAD_REASON_NAMES)
        if parts:
            # The gates fire at very different rates (the hand gate dominates in
            # sim), so collapsing them into payload_valid hides what a tuner needs.
            print("Gated ticks by reason: " + ", ".join(parts))


def _print_inertial_statistics(df, n):
    if not has_inertial_estimate(df):
        if "inertial_reason" in df.columns:
            print("Inertial regression (Layer 2B): not configured this run")
        return
    print("\n=== Inertial regression (Layer 2B) ===")
    valid = _valid_mask(df, "inertial_valid")
    if valid is not None:
        n_valid = int(valid.sum())
        print(f"Valid ticks: {n_valid / n * 100:.1f}% ({n_valid}/{n})")
    if "inertial_rank" in df.columns:
        ranks = df["inertial_rank"].dropna().astype(int)
        if not ranks.empty:
            # rank 3 is the normal startup state, not a failure: one pose cannot
            # observe the gravity-direction component of m·c.
            parts = [
                f"{int(k)}={c / len(ranks) * 100:.1f}%" for k, c in ranks.value_counts().items()
            ]
            print("Rank distribution (3 = still one pose): " + ", ".join(parts))
    if "inertial_sigma_min" in df.columns:
        s = df["inertial_sigma_min"].dropna()
        if not s.empty:
            print(f"σ_min of the 4×4 normal matrix (pose diversity): max={s.max():.4g}")
    if "inertial_mass" in df.columns:
        m = df["inertial_mass"]
        m = m[valid] if valid is not None else m
        m = m.dropna()
        if not m.empty:
            print(
                f"m̂ over valid ticks (kg): mean={m.mean():.4f} p50={m.median():.4f} max={m.max():.4f}"
            )
    com_cols = [f"inertial_c{a}" for a in _AXES]
    if all(c in df.columns for c in com_cols) and valid is not None and bool(valid.any()):
        com = df.loc[valid, com_cols].dropna()
        if not com.empty:
            means = ", ".join(
                f"{a}={com[c].mean():.4f}" for c, a in zip(com_cols, _AXES, strict=True)
            )
            print(f"ĉ over valid ticks (m): {means}")
    if "inertial_fit_error" in df.columns:
        s = df["inertial_fit_error"].dropna()
        if not s.empty:
            print(f"Fit error: max={s.max():.4g}")
    if "inertial_reason" in df.columns:
        parts = _reason_histogram(df["inertial_reason"], _INERTIAL_REASON_NAMES)
        if parts:
            print("Gated ticks by reason: " + ", ".join(parts))
