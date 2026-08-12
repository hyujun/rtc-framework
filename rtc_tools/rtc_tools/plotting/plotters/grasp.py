"""Force-PI grasp diagnostics plotters for the controller-owned grasp_diag.csv (#428).

GraspDiagLog is a per-tick single-row channel written by the joint and task demo
controllers whenever a `force_pi_grasp` block is configured:

  - servo state per finger (`s_*`, `f_desired_*`, `f_measured_*`, `f_error_*`)
  - the online stiffness estimator (`k_est_*` = the EMA, `k_inst_raw_*` = the
    pre-clamp instantaneous sample, `delta_s_*`/`delta_f_*` = the pair it was
    computed from, `est_updated_*` = whether the guards accepted it)
  - the applied gain multiplier (`gain_scale_*`) and the estimator parameters in
    force that tick (`beta`, `alpha_ema`, `K_est_max`)
  - FSM context (`grasp_phase`, `target_force`) and `valid`

Why `f_measured_*` and not the sensor CSV: this column is the 25 Hz Force-PI
filter bank, the lane the PI law and the stiffness estimator actually read.
`<device>_sensor.csv`'s `force_filtered_*` is a DIFFERENT bank (contact_stop,
deployed at 50 Hz) with its own YAML cutoff. Measuring force noise from the
sensor CSV answers a question about a lane the estimator never sees — and both
columns are called force, so nothing about the mistake is visible downstream.

`print_grasp_diag_statistics` is the primary output, not the figures: the point
of the channel is to turn #426's assumed force noise sigma into a measured one,
and the hardware session ships text back before anyone opens a plot.

Sigma is reported over the stationary hold (`grasp_phase == kHolding`, valid
rows only) rather than the whole file. Approach and force ramp are transients
by construction, so a whole-file standard deviation measures the trajectory
rather than the noise floor and would read several times too high.
"""

from pathlib import Path

import matplotlib.pyplot as plt

# rtc::grasp::GraspPhase wire values.
_PHASE_NAMES = {
    0: "idle",
    1: "approaching",
    2: "contact",
    3: "force_control",
    4: "holding",
    5: "releasing",
}
_PHASE_HOLDING = 4

_MEASURED_PREFIX = "f_measured_"


def grasp_finger_names(df):
    """Finger names stamped into the per-finger column headers, in file order."""
    return [c[len(_MEASURED_PREFIX) :] for c in df.columns if c.startswith(_MEASURED_PREFIX)]


def _valid_rows(df):
    """Rows where the Force-PI law actually ran.

    Every per-finger field on a `valid == 0` row is zero by construction (E-STOP
    and non-force_pi ticks are logged rather than skipped so a gap can only mean
    a dropped row), so including them would drag every mean and standard
    deviation toward zero.
    """
    if "valid" not in df.columns:
        return df
    return df[df["valid"].astype(float) > 0.5]


def _hold_rows(df):
    """Valid rows in the stationary hold phase — the segment sigma is defined on."""
    rows = _valid_rows(df)
    if "grasp_phase" not in rows.columns:
        return rows
    return rows[rows["grasp_phase"].astype(float).round() == _PHASE_HOLDING]


def print_grasp_diag_statistics(df):
    """Console summary. The force-noise sigma per finger is the headline number."""
    print("\n=== Force-PI Grasp Diagnostics ===")
    n = len(df)
    print(f"Samples: {n}")
    if n == 0:
        return

    fingers = grasp_finger_names(df)
    print(f"Fingers: {', '.join(fingers) if fingers else '(none named in header)'}")

    if "valid" in df.columns:
        valid_pct = df["valid"].astype(float).mean() * 100
        print(f"valid: {valid_pct:.1f}% of ticks (0 = E-STOP or another law drove the hand)")

    # Rows are one per RT tick, so a gap is a dropped row — never a skipped tick.
    if "tick" in df.columns and n > 1:
        ticks = df["tick"].dropna().astype("int64")
        if len(ticks) > 1:
            gaps = ticks.diff().iloc[1:]
            dropped = int((gaps - 1).clip(lower=0).sum())
            if dropped > 0:
                print(f"Dropped rows (tick gaps): {dropped} over {len(ticks)} logged ticks")
            if (gaps <= 0).any():
                print("WARNING: tick column is not strictly increasing (concatenated sessions?)")

    if "grasp_phase" in df.columns:
        # Valid rows only. A valid=0 row carries grasp_phase 0 because the whole
        # per-tick block is zeroed rather than frozen, so counting those would
        # report halted ticks as time spent "idle" — the one phase name that
        # sounds like a real FSM state rather than an absence of one.
        phases = _valid_rows(df)["grasp_phase"].dropna().astype(int)
        if not phases.empty:
            parts = [
                f"{_PHASE_NAMES.get(int(k), int(k))}={c / len(phases) * 100:.1f}%"
                for k, c in phases.value_counts().items()
            ]
            print("Phase occupancy: " + ", ".join(parts))

    # Estimator parameters that were live this run. Logged per tick because
    # set_params() can move them, and K_est_max in particular decides whether a
    # flat k_est means "converged" or "pinned at the cap".
    live = _valid_rows(df)
    for key in ("beta", "alpha_ema", "K_est_max"):
        if key in df.columns and not live.empty:
            col = live[key].dropna()
            if col.empty:
                continue
            if col.nunique() == 1:
                print(f"{key}: {col.iloc[0]:g}")
            else:
                print(f"{key}: CHANGED during run — {col.min():g} .. {col.max():g}")

    hold = _hold_rows(df)
    print(f"\nStationary hold (grasp_phase=holding, valid): {len(hold)} ticks")
    if hold.empty:
        print("  No hold segment — force-noise sigma is undefined for this run.")
        print("  (Sigma is measured on the hold, not the approach/ramp transient.)")
    else:
        print("  Force noise sigma on the 25 Hz Force-PI bank (the estimator's own lane):")
        for f in fingers:
            col = f"{_MEASURED_PREFIX}{f}"
            if col not in hold.columns:
                continue
            series = hold[col].dropna()
            if len(series) < 2:
                continue
            sigma = float(series.std())
            print(
                f"    {f:<10} sigma={sigma * 1000.0:8.3f} mN   "
                f"mean={float(series.mean()):7.4f} N   "
                f"p2p={float(series.max() - series.min()) * 1000.0:8.3f} mN"
            )

    if live.empty:
        return

    print("\n  Stiffness estimator:")
    for f in fingers:
        k_col, raw_col, upd_col = f"k_est_{f}", f"k_inst_raw_{f}", f"est_updated_{f}"
        if k_col not in live.columns:
            continue
        k = live[k_col].dropna()
        if k.empty:
            continue
        line = f"    {f:<10} k_est: mean={float(k.mean()):8.3f} last={float(k.iloc[-1]):8.3f}"
        # A k_est that never leaves 1.0 is the deployed pinned state, not a
        # converged estimate — the two are indistinguishable without saying so.
        if float(k.max()) - float(k.min()) < 1e-9:
            line += "  [CONSTANT]"
        print(line)
        if upd_col in live.columns:
            accepted = float(live[upd_col].astype(float).mean()) * 100
            print(f"               samples accepted by the guards: {accepted:.1f}% of ticks")
        if raw_col in live.columns:
            raw = live[live[upd_col].astype(float) > 0.5][raw_col] if upd_col in live.columns \
                else live[raw_col]
            raw = raw.dropna()
            if len(raw) > 1:
                # The spread of the raw samples against the EMA they feed is the
                # SNR claim #426 rests on: if sigma(k_inst_raw) is the size of
                # k_est itself, the estimator is integrating noise.
                print(
                    f"               k_inst_raw (pre-clamp, accepted): "
                    f"mean={float(raw.mean()):8.3f} sigma={float(raw.std()):8.3f} "
                    f"min={float(raw.min()):8.3f} max={float(raw.max()):8.3f}"
                )

    if any(f"gain_scale_{f}" in live.columns for f in fingers):
        print("\n  Applied gain multiplier 1/(1+beta*k_est):")
        for f in fingers:
            col = f"gain_scale_{f}"
            if col not in live.columns:
                continue
            g = live[col].dropna()
            if g.empty:
                continue
            flat = " [CONSTANT]" if float(g.max()) - float(g.min()) < 1e-9 else ""
            print(f"    {f:<10} mean={float(g.mean()):.4f} min={float(g.min()):.4f}{flat}")


def _phase_bands(ax, df):
    """Shade phase changes so servo behaviour is readable against the FSM."""
    if "grasp_phase" not in df.columns or "timestamp" not in df.columns:
        return
    phases = df["grasp_phase"].astype(float).round()
    changes = phases.ne(phases.shift()).to_numpy().nonzero()[0]
    for idx in changes:
        ax.axvline(
            float(df["timestamp"].iloc[idx]), color="gray", linestyle=":", linewidth=0.8, alpha=0.6
        )


def plot_grasp_diag(df, save_dir=None):
    """Force-PI grasp overview (4x1, sharex).

    (1) measured vs desired force per finger + phase-change markers — is the
        servo tracking, and does a hard object actually ring?
    (2) grasp parameter s per finger
    (3) stiffness estimator: k_est (EMA) against k_inst_raw (pre-clamp sample),
        with accepted-sample density underneath
    (4) gain_scale — flat at 1/(1+beta) means the estimate is pinned
    """
    fingers = grasp_finger_names(df)
    if not fingers or "timestamp" not in df.columns:
        print("  Skipping grasp diag plot (no per-finger columns found)")
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 15), sharex=True)
    fig.suptitle("Force-PI Grasp Diagnostics", fontsize=16, fontweight="bold")
    t = df["timestamp"]

    # (1) Force tracking. Desired is dashed so a converged loop reads as one line.
    ax = axes[0]
    for i, f in enumerate(fingers):
        color = f"C{i}"
        meas = f"{_MEASURED_PREFIX}{f}"
        des = f"f_desired_{f}"
        if meas in df.columns:
            ax.plot(t, df[meas], label=f"{f} measured", linewidth=1.2, color=color)
        if des in df.columns:
            ax.plot(t, df[des], label=f"{f} desired", linewidth=1.0, color=color, linestyle="--",
                    alpha=0.7)
    _phase_bands(ax, df)
    ax.set_ylabel("Force (N)")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_title("25 Hz Force-PI bank — the lane the estimator reads", fontsize=10)

    # (2) Grasp parameter.
    ax = axes[1]
    for i, f in enumerate(fingers):
        col = f"s_{f}"
        if col in df.columns:
            ax.plot(t, df[col], label=f"s_{f}", linewidth=1.2, color=f"C{i}")
    _phase_bands(ax, df)
    ax.set_ylabel("s [0,1]")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)

    # (3) Estimator. Raw samples are scattered rather than lined: they are
    # per-tick point estimates, and joining them implies a continuity the
    # rejected ticks do not have.
    ax = axes[2]
    for i, f in enumerate(fingers):
        color = f"C{i}"
        k_col, raw_col, upd_col = f"k_est_{f}", f"k_inst_raw_{f}", f"est_updated_{f}"
        if k_col in df.columns:
            ax.plot(t, df[k_col], label=f"k_est_{f}", linewidth=1.4, color=color)
        if raw_col in df.columns:
            mask = df[upd_col].astype(float) > 0.5 if upd_col in df.columns else None
            sub = df[mask] if mask is not None else df
            if not sub.empty:
                ax.scatter(sub["timestamp"], sub[raw_col], s=3, alpha=0.35, color=color,
                           label=f"k_inst_raw_{f} (accepted)")
    if "K_est_max" in df.columns:
        ax.plot(t, df["K_est_max"], color="red", linestyle="--", linewidth=1.0, alpha=0.6,
                label="K_est_max (clamp)")
    ax.set_ylabel("stiffness [N/delta_s]")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_title("EMA vs pre-clamp instantaneous samples", fontsize=10)

    # (4) Applied gain multiplier.
    ax = axes[3]
    for i, f in enumerate(fingers):
        col = f"gain_scale_{f}"
        if col in df.columns:
            ax.plot(t, df[col], label=col, linewidth=1.2, color=f"C{i}")
    if "valid" in df.columns:
        ax.step(t, df["valid"], label="valid", where="post", linewidth=1.0, color="C7", alpha=0.6)
    ax.set_ylabel("gain_scale / flag")
    ax.set_xlabel("Time (s)")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / "grasp_diag.png"
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()
