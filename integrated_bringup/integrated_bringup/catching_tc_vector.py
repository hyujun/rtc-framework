"""A catching sim unit's analysis: ``catching_trials`` plus the t_c gap as VECTORS.

    python3 -m integrated_bringup.catching_tc_vector <unit_dir> <config_dir> <out.csv> [<ct_out_dir>]

Runs ``rtc_tools.analysis.catching_trials`` over ``<unit>/session`` +
``<unit>/trials`` with ``decompose_at_tc`` wrapped, so the ctx / truth of every
trial is seen exactly as the tool sees it, and writes one row per committed
trial to ``<out.csv>`` (``vec.csv``): the gap at t_c split along and across the
ball's travel (from E1-F10). With ``<ct_out_dir>`` the tool's own output
(``catching_trials.csv``, the summary JSON) is kept there; otherwise it goes to
a temporary directory. A unit that has ``probe/lane_prediction_dump.csv`` (run
with the probe dump, #807) has it handed to ``catching_trials`` as
``--probe-dump``.

``catch_population_trials`` runs this after a unit, and ``tools/catching_eval/
tc_vector.py`` (what ``analyse_unit.sh`` calls) is a shim over :func:`main`.

**Which python runs it is part of the result.** ``analyse_unit.sh`` calls the
script with ``/usr/bin/python3``, not the ``python3`` of the environment: the
installed ``catching_trials`` entry point is bound to the distribution's python
(its shebang), and a workspace venv carries another pandas and scipy. Every unit
is therefore analysed by the interpreter ``ros2 run rtc_tools catching_trials``
would use, and two units' ``ct/`` are the same tool's numbers.
:func:`analysis_python` reads that interpreter off the installed entry point for
a caller that starts this module itself.
"""

from __future__ import annotations

import contextlib
import io
import math
import os
import sys
import tempfile
from pathlib import Path

# The columns of catching_trials.csv a vec.csv row repeats (those the file has).
KEEP_COLUMNS = (
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
)


def analysis_python(entry_point: str | os.PathLike | None = None) -> str:
    """The interpreter the installed ``catching_trials`` runs under (its shebang).

    ``entry_point`` defaults to ``<rtc_tools prefix>/lib/rtc_tools/catching_trials``.
    Falls back to this process's interpreter when the entry point cannot be read
    or names none — the analysis then still runs, under the caller's python.
    """
    if entry_point is None:
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix

        try:
            prefix = get_package_prefix("rtc_tools")
        except PackageNotFoundError:
            return sys.executable
        entry_point = os.path.join(prefix, "lib", "rtc_tools", "catching_trials")
    try:
        with open(entry_point, encoding="utf-8", errors="replace") as handle:
            first = handle.readline().strip()
    except OSError:
        return sys.executable
    return shebang_interpreter(first) or sys.executable


def shebang_interpreter(first_line: str) -> str | None:
    """The absolute interpreter a ``#!`` line names, or None (``#!/usr/bin/env python3``
    names none: it is whatever the environment's PATH finds)."""
    if not first_line.startswith("#!"):
        return None
    words = first_line[2:].split()
    if not words or os.path.basename(words[0]) == "env" or not os.path.isabs(words[0]):
        return None
    return words[0]


def _wrap(ct, rows: list):
    """``ct.decompose_at_tc`` wrapped to append one vector record per committed trial."""
    import numpy as np

    original = ct.decompose_at_tc

    def wrapped(ctx, truth, window_s, t_impact=math.inf):
        rec = original(ctx, truth, window_s, t_impact)
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

        out = {
            "t_c": t_c,
            "t_commit": rec["t_commit"],
            "ball_speed": float(np.linalg.norm(v_true)),
        }
        out["tot_par"], out["tot_perp"] = split(f_meas - p_true)
        out["servo_par"], out["servo_perp"] = split(f_meas - f_cmd)
        out["cmdtrue_par"], out["cmdtrue_perp"] = split(f_cmd - p_true)
        out["plan_par"], out["plan_perp"] = split(ctx.plan_p_c[kc] - p_true)
        ref, src = ct.reference_at_lead(ctx, kl)
        if ref is not None:
            out["ref_par"], out["ref_perp"] = split(ref - p_true)
            out["clik_par"], out["clik_perp"] = split(f_cmd - ref)

        # measured hand velocity at t_c and the command's at kl (central difference, n rows each side)
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
            out["acmd_par"], out["acmd_perp"] = (
                float(a @ d),
                float(np.linalg.norm(a - (a @ d) * d)),
            )
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
        p_ball, _ = truth.at(ctx.t[win])
        ok = np.all(np.isfinite(p_ball), axis=1)
        if ok.any():
            win, p_ball = win[ok], p_ball[ok]
            e = ctx.fk_meas(win) - ctx.fk.to_model(p_ball)
            dist = np.linalg.norm(e, axis=1)
            i = int(np.argmin(dist))
            out["dmin_par"], out["dmin_perp"] = split(e[i])
            out["dmin_t_ms"] = float((ctx.t[win[i]] - t_c) * 1e3)
        out["ref_source"] = src
        rows.append(out)
        return rec

    return original, wrapped


def trials_argv(unit: Path, config_dir: str, ct_out: str) -> list[str]:
    """The ``catching_trials`` command line of ``unit`` (the probe dump when it has one)."""
    argv = [str(unit / "session"), str(unit / "trials"), "--config-dir", config_dir]
    argv += ["--out", ct_out]
    # A unit run with the probe dump has the predictions its planner read.
    dump = unit / "probe" / "lane_prediction_dump.csv"
    if dump.is_file():
        argv += ["--probe-dump", str(dump)]
    return argv


def run(unit: Path, config_dir: str, out: str, keep_dir: str | None = None) -> int:
    """Analyse ``unit`` and write ``out``; returns the number of vector rows."""
    import numpy as np
    import pandas as pd

    from rtc_tools.analysis import catching_trials as ct

    rows: list[dict] = []
    original, wrapped = _wrap(ct, rows)
    with tempfile.TemporaryDirectory() as tmp:
        ct_out = keep_dir or tmp
        ct.decompose_at_tc = wrapped
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                ct.main(trials_argv(unit, config_dir, ct_out))
        finally:
            ct.decompose_at_tc = original
        base = pd.read_csv(Path(ct_out) / "catching_trials.csv")
    v = pd.DataFrame(rows)
    base = base[np.isfinite(base.t_c)].reset_index(drop=True)
    assert len(v) == len(base) and (len(v) == 0 or np.allclose(v.t_c, base.t_c)), (
        len(v),
        len(base),
    )
    cols = [base[[c for c in KEEP_COLUMNS if c in base]]]
    if len(v):
        cols.append(v.drop(columns=["t_c"]))
    pd.concat(cols, axis=1).to_csv(out, index=False)
    return len(v)


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) not in (3, 4):
        print(__doc__.split("\n\n")[1].strip(), file=sys.stderr)
        return 2
    out = args[2]
    count = run(Path(args[0]), args[1], out, args[3] if len(args) > 3 else None)
    print(out, count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
