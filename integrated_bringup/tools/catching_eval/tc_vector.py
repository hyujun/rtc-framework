#!/usr/bin/env python3
"""Catching sim evaluation (from E1-F10): the t_c gap as VECTORS (along / across the ball's travel).

    tc_vector.py <unit_dir> <config_dir> <out.csv>

Runs rtc_tools.analysis.catching_trials over <unit>/session + <unit>/trials with
decompose_at_tc wrapped, so the ctx/truth of every trial is seen exactly as the
tool sees it. Writes one row per committed trial.
"""

import math
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

from rtc_tools.analysis import catching_trials as ct

ROWS = []
_orig = ct.decompose_at_tc


def _wrapped(ctx, truth, window_s, t_impact=math.inf):
    rec = _orig(ctx, truth, window_s, t_impact)
    t_c = rec.get("t_c", math.nan)
    if truth is None or not math.isfinite(t_c):
        return rec
    t_lead = rec["t_lead_s"]
    kl, kc = ct.lead_tick(ctx, t_c, t_lead)
    p_w, v_w, _, _ = truth.free_flight([t_c], t_impact)
    p_true = ctx.fk.to_model(p_w[0])
    v_true = ctx.fk.model_t_world[:3, :3] @ v_w[0]
    d = v_true / np.linalg.norm(v_true)
    f_meas = ctx.fk_meas(kc)[0]
    f_cmd = ctx.fk_cmd(kl)[0]

    def split(e):
        a = float(e @ d)
        return a * 1e3, float(np.linalg.norm(e - a * d)) * 1e3

    out = {"t_c": t_c, "t_commit": rec["t_commit"], "ball_speed": float(np.linalg.norm(v_true))}
    out["tot_par"], out["tot_perp"] = split(f_meas - p_true)
    out["servo_par"], out["servo_perp"] = split(f_meas - f_cmd)
    out["cmdtrue_par"], out["cmdtrue_perp"] = split(f_cmd - p_true)
    out["plan_par"], out["plan_perp"] = split(ctx.plan_p_c[kc] - p_true)
    ref, src = ct.reference_at_lead(ctx, kl)
    if ref is not None:
        out["ref_par"], out["ref_perp"] = split(ref - p_true)
        out["clik_par"], out["clik_perp"] = split(f_cmd - ref)

    # measured hand velocity at t_c and the command's at kl (central difference, 5 rows each side)
    def vel(fkfun, k, n=3):
        lo, hi = max(0, k - n), min(len(ctx.t) - 1, k + n)
        p = fkfun([lo, hi])
        return (p[1] - p[0]) / (ctx.t[hi] - ctx.t[lo])

    vm = vel(ctx.fk_meas, kc)
    vc = vel(ctx.fk_cmd, kl)
    out["gamma_meas"] = float(vm @ d) / out["ball_speed"]
    out["gamma_cmd"] = float(vc @ d) / out["ball_speed"]
    out["vmeas_perp"] = float(np.linalg.norm(vm - (vm @ d) * d))
    out["vcmd_perp"] = float(np.linalg.norm(vc - (vc @ d) * d))
    # command acceleration vector at kl (second difference over +-5 rows)
    n = 5
    if kl - n >= 0 and kl + n < len(ctx.t):
        p = ctx.fk_cmd([kl - n, kl, kl + n])
        h = 0.5 * (ctx.t[kl + n] - ctx.t[kl - n])
        a = (p[2] - 2 * p[1] + p[0]) / (h * h)
        out["acmd_par"], out["acmd_perp"] = float(a @ d), float(np.linalg.norm(a - (a @ d) * d))
    # the command's travel: first APPROACH tick .. kl
    k_app = ct._first(ctx.mode[: kl + 1] == ct.MODE_APPROACH)
    if k_app is not None:
        p0 = ctx.fk_cmd(k_app)[0]
        out["cmd_dist"] = float(np.linalg.norm(f_cmd - p0))
        ks = np.arange(k_app, kl + 1, 5)
        pp = ctx.fk_cmd(ks)
        out["cmd_path"] = float(np.sum(np.linalg.norm(np.diff(pp, axis=0), axis=1)))
        out["approach_to_tc_s"] = float(ctx.t[kl] - ctx.t[k_app])
        # joint-space travel and peak joint accel of the command over the approach
        dq = ctx.q_cmd[kl] - ctx.q_cmd[k_app]
        out["dq_max"] = float(np.max(np.abs(dq)))
        out["dq_norm"] = float(np.linalg.norm(dq))
    # closest approach of the ball to the measured catch frame, as a vector split
    win = np.nonzero((ctx.t > t_c - window_s) & (ctx.t < t_c + window_s))[0]
    p_ball, v_ball = truth.at(ctx.t[win])
    ok = np.all(np.isfinite(p_ball), axis=1)
    if ok.any():
        win, p_ball = win[ok], p_ball[ok]
        e = ctx.fk_meas(win) - ctx.fk.to_model(p_ball)
        dist = np.linalg.norm(e, axis=1)
        i = int(np.argmin(dist))
        out["dmin_par"], out["dmin_perp"] = split(e[i])
        out["dmin_t_ms"] = float((ctx.t[win[i]] - t_c) * 1e3)
    out["ref_source"] = src
    ROWS.append(out)
    return rec


ct.decompose_at_tc = _wrapped

if __name__ == "__main__":
    # tc_vector.py <unit_dir> <config_dir> <out.csv> [<ct_out_dir>]
    # With <ct_out_dir> the tool's own output (catching_trials.csv, summary
    # JSON) is kept there; otherwise it goes to a temp dir.
    unit, cfg, out = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
    keep_dir = sys.argv[4] if len(sys.argv) > 4 else None
    with tempfile.TemporaryDirectory() as tmp:
        ct_out = keep_dir or tmp
        sys.argv = [
            "catching_trials",
            str(unit / "session"),
            str(unit / "trials"),
            "--config-dir",
            cfg,
            "--out",
            ct_out,
        ]
        # A unit run with PROBE_DUMP=1 (run_unit.sh) has the predictions its planner read.
        dump = unit / "probe" / "lane_prediction_dump.csv"
        if dump.is_file():
            sys.argv += ["--probe-dump", str(dump)]
        import contextlib
        import io

        with contextlib.redirect_stdout(io.StringIO()):
            ct.main()
        base = pd.read_csv(Path(ct_out) / "catching_trials.csv")
    v = pd.DataFrame(ROWS)
    base = base[np.isfinite(base.t_c)].reset_index(drop=True)
    assert len(v) == len(base) and (len(v) == 0 or np.allclose(v.t_c, base.t_c)), (
        len(v),
        len(base),
    )
    keep = [
        "idx",
        "truth_success",
        "supervisor",
        "servo_mm",
        "total_mm",
        "d_min_mm",
        "pred_mm",
        "arrival_ms",
        "cmd_speed_tc",
        "cmd_dvdt_tc",
        "cmd_accel_tc",
        "cmd_move_s",
        "cmd_hold_s",
        "gamma_f_planned",
        "contact_t_minus_tc_ms",
        "contact_body",
        "first_plan_s",
        "segment_workspace_refused",
        "segment_n_followed",
        "segment_wait_node0_ms",
        "t_launch",
    ]
    cols = [base[[c for c in keep if c in base]]]
    if len(v):
        cols.append(v.drop(columns=["t_c"]))
    pd.concat(cols, axis=1).to_csv(out, index=False)
    print(out, len(v))
