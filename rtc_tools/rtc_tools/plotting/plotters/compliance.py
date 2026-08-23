"""§7 task-admittance diagnostics for the controller-owned compliance_diag.csv (#469 S4).

The console summary is the deliverable here, not a figure — the same call
`grasp_diag` made. What S5 has to read off a hardware run is a handful of
numbers (how close the deviation came to its §7.5 envelope, how much of the run
the wrench was stale or low-quality, how often the bias re-armed), and those are
answered by aggregates rather than by looking at a trace.

TWO READINGS THIS FILE EXISTS TO SEPARATE. A flat x̃ is either the law converged
or the law pinned against the displacement box, and the trace alone cannot tell
them apart; `disp_limited` is the integrator's own report of which it was, and
the K_p/K_d/Λ_d snapshot on every row is what makes the number interpretable
without that run's YAML. Likewise an arm that did not move is either "no force
was measured" or "the §10.7 ramp was still fading it in" — `alpha` separates
those, so it is reported rather than assumed to be 1.

Held ticks (valid=0) are excluded from every statistic. Their wrench and
deviation columns are zeroed rather than frozen, so averaging them in would
report the E-STOP as a measurement of zero force.
"""

# rtc::compliance::ComplianceState wire values.
_STATE_NAMES = {
    0: "BIAS_CALIBRATING",
    1: "HOLDING",
    2: "RUNNING_FREE_SPACE",
    3: "RUNNING_CONTACT",
    4: "DEGRADED",
    5: "SAFE_STOP",
}

# rtc::grasp::PullInvalidReason wire values. 0 is not listed: a valid tick is
# reported by count, not by name, so "none" never reads as a failure mode.
_REASON_NAMES = {
    1: "not_initialised",
    2: "degenerate_normal",
    3: "required_contact_missing",
    4: "insufficient_contacts",
}

# integrated_bringup::ComplianceWrenchSource wire values.
_SOURCE_NAMES = {0: "pull_estimator"}

_LIN_AXES = ("x", "y", "z")
_ANG_AXES = ("rx", "ry", "rz")


def _valid_rows(df):
    """Rows where the §7 law actually ran. See the module docstring."""
    if "valid" not in df.columns:
        return df
    return df[df["valid"].astype(float) > 0.5]


def _norm(df, prefix, axes):
    """Row-wise Euclidean norm over `prefix`+axis columns, or None if absent."""
    cols = [f"{prefix}{a}" for a in axes]
    if any(c not in df.columns for c in cols):
        return None
    return (df[cols] ** 2).sum(axis=1) ** 0.5


def _pct(series):
    return float(series.astype(float).mean()) * 100.0


def print_compliance_diag_statistics(df):
    """Console summary. The envelope and freshness numbers are the headline."""
    print("\n=== Task-Admittance (§7) Diagnostics ===")
    n = len(df)
    print(f"Samples: {n}")
    if n == 0:
        return

    if "wrench_source" in df.columns:
        ids = df["wrench_source"].dropna().astype(int).unique()
        names = ", ".join(_SOURCE_NAMES.get(int(i), f"unknown({int(i)})") for i in sorted(ids))
        print(f"Wrench source: {names}")

    if "valid" in df.columns:
        print(
            f"valid: {_pct(df['valid']):.1f}% of ticks "
            "(0 = E-STOP, unreadable arm, or no TCP frame — the law did not run)"
        )

    # One row per RT tick, so a gap is a dropped row and nothing else.
    if "tick" in df.columns and n > 1:
        ticks = df["tick"].dropna().astype("int64")
        if len(ticks) > 1:
            gaps = ticks.diff().iloc[1:]
            dropped = int((gaps - 1).clip(lower=0).sum())
            if dropped > 0:
                print(f"Dropped rows (tick gaps): {dropped} over {len(ticks)} logged ticks")
            if (gaps <= 0).any():
                print("WARNING: tick column is not strictly increasing (concatenated sessions?)")

    live = _valid_rows(df)
    print(f"\nTicks with the law running: {len(live)}")
    if live.empty:
        print("  Nothing below is measurable — every row is a held tick.")
        return

    if "fsm_state" in live.columns:
        states = live["fsm_state"].dropna().astype(int)
        if not states.empty:
            parts = [
                f"{_STATE_NAMES.get(int(k), int(k))}={c / len(states) * 100:.1f}%"
                for k, c in states.value_counts().items()
            ]
            print("FSM occupancy: " + ", ".join(parts))

    # ── §10.7 ramp ────────────────────────────────────────────────────────
    if "alpha" in live.columns:
        alpha = live["alpha"].dropna()
        if not alpha.empty:
            ramping = int((alpha < 1.0).sum())
            print(
                f"Activation ramp: {ramping} tick(s) below alpha=1 "
                f"({ramping / len(alpha) * 100:.1f}% of running ticks); "
                f"min alpha={float(alpha.min()):.3f}"
            )

    # ── Wrench freshness / quality ────────────────────────────────────────
    print("\nWrench lane:")
    f_norm = _norm(live, "wrench_f", _LIN_AXES)
    if f_norm is not None and not f_norm.empty:
        print(
            f"  |f| mean={float(f_norm.mean()):.3f} N  max={float(f_norm.max()):.3f} N "
            "(post-conditioning: bias, deadband, filter and fade applied)"
        )
    for key, label in (
        ("wrench_valid", "a sample has arrived"),
        ("wrench_stale", "age > timeout → DEGRADED"),
        ("in_contact", "contact latch"),
        ("quality_low", "estimate may be all leakage / slipping / saturated"),
    ):
        if key in live.columns:
            print(f"  {key}: {_pct(live[key]):.1f}% of running ticks ({label})")
    if "wrench_age" in live.columns:
        age = live["wrench_age"].dropna()
        if not age.empty:
            print(
                f"  age mean={float(age.mean()) * 1e3:.1f} ms  max={float(age.max()) * 1e3:.1f} ms"
            )
    if "rejected_samples" in live.columns:
        rej = live["rejected_samples"].dropna().astype("int64")
        if not rej.empty and int(rej.max()) > 0:
            # Monotonic counter: the run's own increment, not the lifetime value.
            grew = int(rej.max() - rej.min())
            print(
                f"  rejected_samples: +{grew} over this run "
                "(non-finite samples the pipeline dropped — an intermittently bad producer)"
            )

    if "invalid_reason" in live.columns:
        reasons = live["invalid_reason"].dropna().astype(int)
        bad = reasons[reasons > 0]
        if bad.empty:
            print("  invalid_reason: none — every consumed sample came from a valid estimate")
        else:
            parts = [
                f"{_REASON_NAMES.get(int(k), int(k))}={c / len(reasons) * 100:.1f}%"
                for k, c in bad.value_counts().items()
            ]
            print("  invalid_reason (share of running ticks): " + ", ".join(parts))
        # Reported on BOTH sides of that branch, and deliberately: the pairing
        # (quality_low=1, reason=0) is the one signature no invalid tick can
        # produce, so a file where every reason is 0 is exactly where it is most
        # readable — and where it would be hidden if this sat under `else`.
        if "quality_low" in live.columns:
            both = live[(live["quality_low"].astype(float) > 0.5) & (reasons == 0)]
            if not both.empty:
                print(
                    f"  quality_low with reason=0: {len(both)} tick(s) — the pull was fine, "
                    "the numbers or the application point were not"
                )

    # ── §3.2.1 bias ───────────────────────────────────────────────────────
    if "bias_calibrated" in live.columns:
        print(
            f"\nBias: calibrated on {_pct(live['bias_calibrated']):.1f}% of running ticks"
            + (
                f", re-armed {int(live['bias_begin'].astype(float).sum())}x"
                if "bias_begin" in live.columns
                else ""
            )
        )

    # ── §7.5 envelope — the number S5 tunes against ───────────────────────
    print("\nCompliant frame:")
    x_lin = _norm(live, "x_tilde_", _LIN_AXES)
    if x_lin is not None and not x_lin.empty:
        peak = float(x_lin.max())
        line = f"  |x_tilde| linear: mean={float(x_lin.mean()) * 1e3:.1f} mm  peak={peak * 1e3:.1f} mm"
        if "max_disp_lin" in live.columns:
            bound = live["max_disp_lin"].dropna()
            if not bound.empty and float(bound.max()) > 0.0:
                b = float(bound.max())
                line += f"  ({peak / b * 100:.0f}% of the {b * 1e3:.0f} mm bound)"
                if bound.nunique() > 1:
                    line += "  [BOUND CHANGED during run]"
        print(line)
    x_ang = _norm(live, "x_tilde_", _ANG_AXES)
    if x_ang is not None and not x_ang.empty:
        print(
            f"  |x_tilde| angular: mean={float(x_ang.mean()):.4f} rad  "
            f"peak={float(x_ang.max()):.4f} rad"
        )
    nu = _norm(live, "nu_c_", _LIN_AXES)
    if nu is not None and not nu.empty:
        print(f"  |nu_c| linear: mean={float(nu.mean()):.4f} m/s  peak={float(nu.max()):.4f} m/s")
    for key, label in (
        ("disp_limited", "§7.5 displacement barrier engaged"),
        ("vel_limited", "velocity guard scaled the step back"),
    ):
        if key in live.columns:
            share = _pct(live[key])
            note = (
                "  ← a flat x_tilde here is PINNED, not converged"
                if (key == "disp_limited" and share > 0.0)
                else ""
            )
            print(f"  {key}: {share:.1f}% of running ticks ({label}){note}")
    if "adm_finite" in live.columns:
        refused = int((live["adm_finite"].astype(float) < 0.5).sum())
        if refused > 0:
            print(
                f"  REFUSED steps (non-finite input): {refused} tick(s) — the frame did not move"
            )

    # ── Parameter snapshot ────────────────────────────────────────────────
    # Constant for a run today (configure-time), printed so a stored CSV is
    # readable on its own and so a future live-writable path is visible.
    gains = []
    for prefix, label in (("kp", "K_p"), ("kd", "K_d"), ("md", "Lambda_d")):
        cols = [f"{prefix}_{a}" for a in _LIN_AXES + _ANG_AXES]
        if any(c not in live.columns for c in cols):
            continue
        row = live[cols].iloc[0]
        changed = any(live[c].nunique() > 1 for c in cols)
        gains.append(
            f"  {label}: [{', '.join(f'{float(v):g}' for v in row)}]"
            + ("  [CHANGED during run]" if changed else "")
        )
    if gains:
        print("\nGains in force (diagonal, [lin x3, ang x3]):")
        print("\n".join(gains))
