#!/usr/bin/env python3
"""E1-F06 (G-1, #632): one robot's G-1 verdict from its judged units.

    summarize.py --cfg <robot config dir> --overlay-cf <yaml> --overlay-mpc <yaml> \
        --cf <unit>... --mpc <unit>... [--regression pass|fail|unknown] [--json out.json]

The rules are the ones fixed on #632 before the first throw (plan
the #632 rules comment); this script applies them mechanically.
``--cf`` / ``--mpc`` are the JUDGED units (one per seed × arm, status DONE);
retries are read off ``<unit>.fail<N>`` beside each. Needs ``<unit>/ct``
(``analyse_unit.sh`` — catching_trials with ``t_end`` · ``hold_verdict`` ·
``abort_in_window``, and tc_vector's
``vec.csv``). Never while a unit is being collected.

Statistics come from the repo (``rtc_tools.analysis.catching_decel``): the
Tango test, its score interval, the Wald interval and the exact power.

A RECORD of N arms (#746) — what each arm did on the same throws, nothing judged:

    summarize.py --cfg <robot config dir> --unit <unit>... [--all-pairs] [--json out.json]

An arm is search x segment x ball, read off each unit's ``mirror.txt``
(``planner.search.mode`` · ``planner.segment.mode`` · ``ball_type``) and refused
when the unit's ``conditions.txt`` was told to expect another
(``expect_search`` · ``expect_mode`` · ``expect_ball``). A unit's overlay is the
one its ``conditions.txt`` names. Units are joined throw by throw inside one
repetition — the same estimator seed and the same throw list — so a list thrown
under two seeds is two strata, tabled apart and pooled. Arms are tabled against
each other when they differ in the planner with the ball the same, or in the
ball with the planner the same (``--all-pairs``: every two). Per pair: the 2x2
table of ``truth_success`` and of D4, each arm's rate with its Wilson interval,
the paired difference with its score interval, McNemar's exact p, and the Tango
row as a REFERENCE (the margin decides nothing here). Per arm: the rates, the
plan verdicts and reject reasons, the entrance crossing (#807), the solves —
ended ones as a distribution, the ones the core cut at its deadline as a count
and a share (``rtc_tools.analysis.planner_solves``) — against the budget of
that arm's own planner, the limits, and the same seed-to-seed table of one arm
with itself (how much a repetition moves the numbers).
"""

import argparse
import collections
import itertools
import json
import math
import re
from pathlib import Path

import numpy as np
import pandas as pd

from rtc_tools.analysis import (
    catching_arm_budget as ab,
    catching_decel as cd,
    catching_trials as ct,
    planner_solves as ps,
)
from rtc_tools.analysis.catching_throw_list import throw_group, throw_key
from rtc_tools.utils.catching_keys import (
    RENAMED_TRIALS_COLUMNS,
    RENAMED_TRIALS_SUMMARY_KEYS,
    OldToolOutputError,
    normalize_mirror,
    reject_old_output_names,
)

MARGIN = 0.10
ALPHA = 0.025
DESIGN_PSI = 0.43
POWER_DIFFS = (0.0, -0.05)
CTRL = "demo_catching_controller"
POST_SOLVE = {
    "solve_failed",
    "budget",
    "late",
    "slack",
    "ready",
    "published",
    "superseded",
    "catch_error",
    "speed",
}
PRE_SOLVE = {
    "off",
    "no_state",
    "stale_state",
    "up_to_date",
    "past_replan_window",
    "input_non_finite",
    "not_at_rest",
    "too_late",
    "not_followed",
    "no_ball",
}
REPLAN = ("same", "advance", "stop")
HELD = ("budget", "late", "too_late")
RT_MODES = (ct.MODE_APPROACH, ct.MODE_COMMITTED, ct.MODE_CLOSING, ct.MODE_DECEL, ct.MODE_HOLD)
NO_DECEL_END = (ct.MODE_RETREAT, ct.MODE_ARMED, ct.MODE_TRACKING)
EVENT_SWITCHED, EVENT_GATE_REFUSED = 4, 5
TICK_US_LIMIT = 120.0
DROP_SEED_RERUN = 6


# ── small numerics ───────────────────────────────────────────────────────────
def nearest_rank(values, q):
    """The ⌈q·N⌉-th smallest value (1-based), or None for no value."""
    v = np.sort(np.asarray([x for x in values if x is not None and math.isfinite(x)], float))
    if not v.size:
        return None
    return float(v[max(1, math.ceil(q * v.size)) - 1])


def dist(values, qs=(0.5, 0.99)):
    v = [x for x in values if x is not None and math.isfinite(x)]
    out = {"n": len(v)}
    for q in qs:
        out[f"p{q * 100:g}"] = nearest_rank(v, q)
    out["max"] = max(v) if v else None
    return out


def median(values):
    v = [x for x in values if x is not None and math.isfinite(x)]
    return float(np.median(v)) if v else None


def mode_log_verdict(mode_log):
    """(HOLD → RETREAT reached, ABORT_SAFE seen) from the runner's mode transitions."""
    names = [m[1] for m in mode_log or []]
    abort = "ABORT_SAFE" in names
    if "RETREAT" not in names:
        return False, abort
    i = names.index("RETREAT")
    return i > 0 and names[i - 1] == "HOLD", abort


def kv_file(path):
    """``name: value`` lines (a unit's ``mirror.txt``), the names under their current
    spelling: a unit recorded before #711 carries the old mirror names, read through
    the alias table; a file holding an old name and its new name is refused."""
    out = {}
    if path.is_file():
        for line in path.read_text().splitlines():
            k, _, v = line.partition(":")
            out[k.strip()] = v.strip()
    return normalize_mirror(out, source=str(path))


def mirror_value(mirror, key):
    m = re.search(r"value is: (\S+)$", mirror.get(key, ""))
    return float(m.group(1)) if m else None


# ── one trial's windows ──────────────────────────────────────────────────────
def follow_window(mode):
    """(k0, k1) inclusive: the first APPROACH tick to the last DECEL tick; with no
    DECEL, to the tick before the first RETREAT / ARMED / TRACKING after it."""
    ka = ct._first(mode == ct.MODE_APPROACH)
    if ka is None:
        return None
    rest = mode[ka:]
    dec = np.flatnonzero(rest == ct.MODE_DECEL)
    if dec.size:
        return ka, ka + int(dec[-1])
    end = np.flatnonzero(np.isin(rest, NO_DECEL_END))
    return ka, (ka + int(end[0]) - 1) if end.size else len(mode) - 1


def limit_check(q, qd, tau, lim):
    lo, hi = np.asarray(lim["position_lower"]), np.asarray(lim["position_upper"])
    vmax, tmax = np.asarray(lim["max_velocity"]), np.asarray(lim["max_torque"])
    return {
        "pos": int(np.sum((q < lo) | (q > hi))),
        "vel": int(np.sum(np.abs(qd) > vmax)),
        "torque": int(np.sum(np.abs(tau) > tmax)),
        "pos_margin_min": float(np.min(np.minimum(q - lo, hi - q))) if len(q) else math.nan,
        "vel_ratio_max": float(np.max(np.abs(qd) / vmax)) if len(q) else math.nan,
        "torque_ratio_max": float(np.max(np.abs(tau) / tmax)) if len(q) else math.nan,
    }


# ── one unit ─────────────────────────────────────────────────────────────────
def reject_old_ct_outputs(unit, summ, ctdf, vec):
    """Refuse a ``ct/`` that an older ``catching_trials`` / ``tc_vector`` wrote (the segment
    lane's metrics said ``decel_*``): reading it would skip the lane silently."""
    ct_dir = unit / "ct"
    try:
        reject_old_output_names(
            [*summ, *summ.get("medians", {})],
            {**RENAMED_TRIALS_COLUMNS, **RENAMED_TRIALS_SUMMARY_KEYS},
            source=str(ct_dir / "catching_trials_summary.json"),
            tool="catching_trials",
        )
        reject_old_output_names(
            ctdf.columns,
            RENAMED_TRIALS_COLUMNS,
            source=str(ct_dir / "catching_trials.csv"),
            tool="catching_trials",
        )
        if vec is not None:
            reject_old_output_names(
                vec.columns,
                RENAMED_TRIALS_COLUMNS,
                source=str(ct_dir / "vec.csv"),
                tool="tc_vector",
            )
    except OldToolOutputError as err:
        raise SystemExit(str(err)) from err
    # A catching_trials from before the lane metrics wrote neither name: nothing above
    # refuses it, and counting its aged segments as 0 would be the silent skip again.
    if "segment_aged" not in ctdf.columns:
        raise SystemExit(
            f"{ct_dir / 'catching_trials.csv'}: no segment_aged column — written by a "
            "catching_trials older than the segment lane's metrics; regenerate it with the "
            "current tool"
        )


def load_unit(unit, cfg, overlay):
    unit = Path(unit)
    if (unit / "status").read_text().strip() != "DONE":
        raise SystemExit(f"{unit}: status is not DONE")
    summ = json.loads((unit / "ct" / "catching_trials_summary.json").read_text())
    if summ.get("validity", {}).get("lane_rules_evaluated") is not True:
        raise SystemExit(f"{unit}: lane_rules_evaluated is not true — analysis stops (rule)")
    dec = cd.analyse_unit(unit, unit / "session", cfg, overlays=[Path(overlay)])
    ctdf = pd.read_csv(unit / "ct" / "catching_trials.csv").set_index("idx")
    vec = pd.read_csv(unit / "ct" / "vec.csv") if (unit / "ct" / "vec.csv").is_file() else None
    reject_old_ct_outputs(unit, summ, ctdf, vec)
    recs = json.loads((unit / "trials" / "trial_results.json").read_text())
    recs = recs["trials"] if isinstance(recs, dict) else recs
    rec = {int(r["idx"]): r for r in recs}
    ctl = unit / "session" / "controllers" / CTRL
    joints = summ["arm_joints"]
    arm = summ["arm_device"]
    want = {"t_relative_s", "tick", "mode", "reason_name", "segment_event", "segment_rho"}
    want |= {f"plan_a_d_{a}" for a in "xyz"} | {f"q_cmd_{j}" for j in joints}
    want |= {f"q_meas_{j}" for j in joints}
    diag = ct._read_csv(ctl / "catching_diag.csv", usecols=lambda c: c in want)
    state = ct._read_csv(ctl / f"{arm}_state.csv")
    pe = ct._read_csv(ctl / "planner_events.csv")
    cm = ct._read_csv(unit / "session" / "timing" / "cm_timing_log.csv")
    ptl = unit / "session" / "timing" / "planner_timing_log.csv"
    ptiming = pd.read_csv(ptl, usecols=["t_total_us"]) if ptl.is_file() else None
    cond = kv_file(unit / "conditions.txt")
    mirror = kv_file(unit / "mirror.txt")
    retries = len(list(unit.parent.glob(unit.name + ".fail*")))
    return {
        "unit": unit,
        "name": unit.name,
        "summ": summ,
        "dec": dec,
        "ct": ctdf,
        "vec": vec,
        "rec": rec,
        "diag": diag,
        "state": state,
        "pe": pe,
        "cm": cm,
        "ptiming": ptiming,
        "cond": cond,
        "mirror": mirror,
        "retries": retries,
        "joints": joints,
        "arm": arm,
    }


def trial_rows(u, cfg, fk):
    """Per-trial facts of a unit: success, mode-log check, windows, limits, peaks, axis error."""
    d = u["diag"]
    t = d["t_relative_s"].to_numpy(float)
    mode = d["mode"].to_numpy(int)
    reason = d["reason_name"].to_numpy(object)
    tick = d["tick"].to_numpy(np.int64)
    qc = d[[f"q_cmd_{j}" for j in u["joints"]]].to_numpy(float)
    qm = d[[f"q_meas_{j}" for j in u["joints"]]].to_numpy(float)
    a_d = d[[f"plan_a_d_{a}" for a in "xyz"]].to_numpy(float)
    st = u["state"]
    st_t = st["t_relative_s"].to_numpy(float)
    sq = st[[f"actual_pos_{j}" for j in u["joints"]]].to_numpy(float)
    sqd = st[[f"actual_vel_{j}" for j in u["joints"]]].to_numpy(float)
    stau = st[[f"effort_{j}" for j in u["joints"]]].to_numpy(float)
    lim, _ = ab._device_limits(cfg, u["arm"])
    dt = float(np.median(np.diff(t[:2000])))
    out = []
    for r in u["dec"]["all_trials"]:
        idx = r["idx"]
        c = u["ct"].loc[idx]
        rec = u["rec"].get(idx, {})
        row = {
            "unit": u["name"],
            "idx": idx,
            "kind": r["kind"],
            "seed": r["seed"],
            "sample_idx": r["sample_idx"],
            "throw_id": r.get("throw_id"),
            "throws_file_sha256": r.get("throws_file_sha256"),
            "invalid_reason": r["invalid_reason"] or "",
            "truth": bool(r["truth_success"]),
            "hold_no_abort": r["hold_no_abort"],
        }
        row["d4"] = bool(row["truth"] and row["hold_no_abort"])
        hold_ml, abort_ml = mode_log_verdict(rec.get("mode_log"))
        row["mode_log_ok"] = hold_ml and not abort_ml
        out.append(row)
        if not (math.isfinite(r["t_launch"]) and math.isfinite(r["t_end"])):
            continue
        w = np.flatnonzero((t >= r["t_launch"]) & (t <= r["t_end"]))
        if not w.size:
            continue
        m = mode[w]
        enter = np.r_[True, m[1:] != m[:-1]]
        ab_enter = np.flatnonzero(enter & (m == ct.MODE_ABORT_SAFE))
        row["abort_reasons"] = [str(reason[w[i]]) for i in ab_enter]
        row["approach"] = bool(np.any(m == ct.MODE_APPROACH))
        row["no_plan_reason"] = None if row["approach"] else _dominant(reason[w])
        row["plan_adopted"] = math.isfinite(_f(c.get("first_plan_s")))
        row["committed"] = math.isfinite(_f(c.get("t_c")))
        row["ticks_rt"] = tick[w][np.isin(m, RT_MODES)]
        hold = w[m == ct.MODE_HOLD]
        if hold.size:
            sel = (st_t >= t[hold[0]]) & (st_t <= t[hold[-1]])
            row["hold_limits"] = limit_check(sq[sel], sqd[sel], stau[sel], lim)
        fw = follow_window(m)
        if fw is not None:
            k0, k1 = w[fw[0]], w[fw[1]]
            sel = (st_t >= t[k0]) & (st_t <= t[k1])
            row["limits"] = limit_check(sq[sel], sqd[sel], stau[sel], lim)
            pad = cd.PAD_TICKS
            lo, hi = max(0, k0 - pad), min(len(t), k1 + 1 + pad)
            for name, q in (("cmd", qc), ("meas", qm)):
                der = cd.derivatives(q[lo:hi], dt)
                cut = slice(k0 - lo, k1 - lo + 1)
                row[f"qdd_{name}_peak"] = float(np.max(np.abs(der["qdd"][cut])))
                row[f"jerk_{name}_peak"] = float(np.max(np.abs(der["jerk"][cut])))
        if row["committed"] and not row["invalid_reason"]:
            kc = int(np.clip(np.searchsorted(t, float(c["t_c"])), 0, len(t) - 1))
            rot, _ = fk._arm.frame_placement(qm[kc])
            z = rot[:, ct_axis()]
            a = a_d[kc] / np.linalg.norm(a_d[kc])
            row["axis_err_deg"] = math.degrees(math.acos(float(np.clip(z @ a, -1.0, 1.0))))
    return out


def ct_axis():
    return cd.APPROACH_AXIS


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


def _dominant(values):
    vals = [v for v in values if v and v != "none"]
    return collections.Counter(vals).most_common(1)[0][0] if vals else "none"


# ── arm-level blocks ─────────────────────────────────────────────────────────
def solve_block(units, arm_is_mpc):
    pe = pd.concat([u["pe"] for u in units], ignore_index=True)
    budgets = sorted({u["cond"].get("planner_budget_s", "") for u in units})
    if len(budgets) != 1 or not budgets[0]:
        raise SystemExit(f"planner_budget_s differs or is missing across units: {budgets}")
    budget_us = float(budgets[0]) * 1e6
    s = pe[pe["n_in_window"] > 0]
    out = {
        "search": {
            "n": int(len(s)),
            "p99_us": nearest_rank(s["search_us"], 0.99),
            "budget_us": budget_us,
            "budget_hit_rate": float(s["budget_hit"].mean()) if len(s) else None,
            **{k: v for k, v in dist(s["search_us"]).items() if k != "n"},
        }
    }
    sr = out["search"]
    sr["pass"] = None if sr["p99_us"] is None else sr["p99_us"] <= budget_us
    if "segment_kind" in pe:
        unknown = set(pe["segment_outcome"].dropna()) - POST_SOLVE - PRE_SOLVE
        if unknown:
            raise SystemExit(
                f"segment_outcome outside both lists: {sorted(unknown)} — analysis stops"
            )
        solved = pe[pe["segment_outcome"].isin(POST_SOLVE)]
        out["solve_us_sanity"] = {
            "post_solve_zero": int((solved["segment_solve_us"] <= 0).sum()),
            "pre_solve_nonzero": int(
                (pe[pe["segment_outcome"].isin(PRE_SOLVE)]["segment_solve_us"] > 0).sum()
            ),
        }
        out["held"] = int(pe["segment_outcome"].isin(HELD).sum())
        out["outcomes"] = pe["segment_outcome"].value_counts().to_dict()
        firsts = {mirror_value(u["mirror"], "planner.segment.mpc.budget.first_s") for u in units}
        replans = {mirror_value(u["mirror"], "planner.segment.mpc.budget.replan_s") for u in units}
        for key, kinds, budgets_s in (("first", ("first",), firsts), ("replan", REPLAN, replans)):
            v = solved[solved["segment_kind"].isin(kinds)]["segment_solve_us"]
            blk = {"n": int(len(v)), **{k: x for k, x in dist(v).items() if k != "n"}}
            if arm_is_mpc:
                if len(budgets_s) != 1 or None in budgets_s:
                    raise SystemExit(
                        f"segment budget {key} differs or missing in mirrors: {budgets_s}"
                    )
                blk["budget_us"] = budgets_s.pop() * 1e6
                blk["p99_us"] = nearest_rank(v, 0.99)
                blk["pass"] = None if blk["p99_us"] is None else blk["p99_us"] <= blk["budget_us"]
            out[key] = blk
    if any(u["ptiming"] is not None for u in units):
        tt = pd.concat([u["ptiming"] for u in units if u["ptiming"] is not None])["t_total_us"]
        out["wakes_over_33ms"] = int((tt > 33333.3).sum())
        out["wake_ms_max"] = float(tt.max() / 1e3)
    return out


def replan_block(units):
    rho, switches, refused, aged = [], 0, 0, 0
    for u in units:
        d = u["diag"]
        if "segment_event" not in d:
            return None
        m = d["mode"].to_numpy(int)
        ev = d["segment_event"].to_numpy(int)
        enter = np.r_[True, m[1:] != m[:-1]]
        trial = np.cumsum(enter & (m == ct.MODE_APPROACH))
        sw = np.flatnonzero(ev == EVENT_SWITCHED)
        later = (
            pd.Series(trial[sw]).duplicated().to_numpy()
        )  # a trial's first switch is not a replan
        switches += int(later.sum())
        rho += list(d["segment_rho"].to_numpy(float)[sw][later])
        refused += int(np.sum(ev == EVENT_GATE_REFUSED))
        aged += int(pd.to_numeric(u["ct"]["segment_aged"], errors="coerce").fillna(0).sum())
    return {
        "replan_switches": switches,
        "gate_refused": refused,
        "gate_refusal_rate": refused / (switches + refused) if switches + refused else None,
        "rho": dist(rho, (0.5, 0.95)),
        "aged": aged,
    }


def rt_tick_block(units, rows_by_unit):
    comp, total = [], []
    for u in units:
        cm = u["cm"].drop_duplicates("tick_count").set_index("tick_count")
        ticks = np.concatenate(
            [r["ticks_rt"] for r in rows_by_unit[u["name"]] if "ticks_rt" in r] or [[]]
        )
        hit = cm.reindex(ticks.astype(np.int64)).dropna(subset=["t_compute_us"])
        comp += list(hit["t_compute_us"])
        total += list(hit["t_total_us"])
    out = {}
    for name, v in (("t_compute_us", comp), ("t_total_us", total)):
        out[name] = dist(v, (0.5, 0.99, 0.999))
        out[name]["over_120us_per_10k"] = (
            1e4 * sum(1 for x in v if x > TICK_US_LIMIT) / len(v) if v else None
        )
    return out


def unit_t_c(units):
    keys = (
        "total_mm",
        "d_min_mm",
        "ref_vs_true_mm",
        "servo_mm",
        "arrival_ms",
        "contact_v_rel",
        "contact_hand_speed",
        "gamma_f_planned",
        "ref_saturated_max_streak",
    )
    out = {}
    for k in keys:
        vals = []
        for u in units:
            c = u["ct"]
            sel = (
                c["accepted"].astype(str).str.lower().eq("true")
                & np.isfinite(pd.to_numeric(c["t_c"], errors="coerce"))
                & c["invalid_reason"].fillna("").eq("")
            )
            if k in c:
                vals += list(pd.to_numeric(c.loc[sel, k], errors="coerce"))
        out[k] = median(vals)
    for k in ("tot_par", "tot_perp", "gamma_meas"):
        vals = [x for u in units if u["vec"] is not None and k in u["vec"] for x in u["vec"][k]]
        out[k] = median(vals)
    return out


# ── pairing, test, verdict ───────────────────────────────────────────────────
def pair_rows(a_rows, b_rows, names=("a", "b")):
    """Two arms' trials joined by throw (``throw_key``: a list's sha256 and
    throw_id, or kind · seed · sample_idx). A trial without a key (#798: a
    list unit whose rows lack the list, a series without a seed) pairs with
    nothing and is counted in ``unkeyed``. ``names`` label the two arms in the
    counts (``unpaired_<name>``, the ``<name>_only`` side of a dropped pair)."""
    unkeyed = collections.Counter()

    def keyed(rows, arm):
        out = {}
        for r in rows:
            k = throw_key(r)
            if k is None:
                unkeyed[arm] += 1
                continue
            if k in out:
                raise SystemExit(f"throw {k} twice in one arm")
            out[k] = r
        return out

    a, b = keyed(a_rows, names[0]), keyed(b_rows, names[1])
    keys = sorted(set(a) & set(b))
    dropped = collections.Counter()
    valid, per_seed_drop = [], collections.Counter()
    for k in keys:
        ra, rb = a[k], b[k]
        if ra["invalid_reason"] or rb["invalid_reason"]:
            side = (
                "both"
                if ra["invalid_reason"] and rb["invalid_reason"]
                else (f"{names[0]}_only" if ra["invalid_reason"] else f"{names[1]}_only")
            )
            dropped[(side, ra["invalid_reason"] or "-", rb["invalid_reason"] or "-")] += 1
            per_seed_drop[throw_group(k)] += 1
            continue
        valid.append((k, ra, rb))
    return {
        "keys": len(keys),
        f"unpaired_{names[0]}": len(set(a) - set(b)),
        f"unpaired_{names[1]}": len(set(b) - set(a)),
        "unkeyed": dict(unkeyed),
        "valid": valid,
        "dropped": {"/".join(k): v for k, v in dropped.items()},
        "dropped_n": sum(dropped.values()),
        "seed_drops": dict(per_seed_drop),
        # seeds (int) and lists ("list:…") can share one run: order by text
        "seeds_to_rerun": sorted(
            (s for s, n in per_seed_drop.items() if n >= DROP_SEED_RERUN), key=str
        ),
    }


def pair(cf_rows, mpc_rows):
    """:func:`pair_rows` of the G-1 verdict's two arms."""
    return pair_rows(cf_rows, mpc_rows, ("cf", "mpc"))


def table(valid, field):
    both = only_a = only_b = neither = 0
    for _, ra, rb in valid:
        sa, sb = ra[field], rb[field]
        both += sa and sb
        only_a += sa and not sb
        only_b += sb and not sa
        neither += not sa and not sb
    return {
        "n": len(valid),
        "both": both,
        "mpc_only": only_b,
        "cf_only": only_a,
        "neither": neither,
    }


def test_block(tab):
    n, xa, xb = tab["n"], tab["cf_only"], tab["mpc_only"]
    pairs = {"n_pairs": n, "only_a": xa, "only_b": xb}
    t = cd.tango_noninferiority(xa, xb, n, MARGIN, alpha=ALPHA)
    psi, d = (xa + xb) / n, (xb - xa) / n

    def power(at_psi, diff):
        return (
            None
            if abs(diff) > at_psi
            else cd.paired_noninferiority_power(n, at_psi, diff, MARGIN, alpha=ALPHA)
        )

    score = cd.tango_score_ci(xa, xb, n)
    return {
        **tab,
        "p_cf": (tab["both"] + xa) / n,
        "p_mpc": (tab["both"] + xb) / n,
        "diff": d,
        "psi": psi,
        "z": t["z"],
        "p": t["p"],
        "noninferior": t["reject"],
        "score_ci95": score,
        "wald_ci95": cd.paired_difference(pairs)["ci95"],
        "ci_excludes_0_below": score[1] < 0.0,
        "mcnemar_exact_p": ct.mcnemar_exact(xa, xb),
        "power_design": {f"{x:+.2f}": power(DESIGN_PSI, x) for x in POWER_DIFFS},
        "power_observed": {f"{x:+.3f}": power(psi, x) for x in (*POWER_DIFFS, d)},
        "size_at_psi_hat": power(psi, -MARGIN),
    }


def seed_block(valid):
    by = collections.defaultdict(list)
    for k, ra, rb in valid:
        by[throw_group(k)].append((ra["d4"], rb["d4"]))
    out, terms = {}, []
    for s, prs in sorted(by.items(), key=lambda kv: str(kv[0])):
        n = len(prs)
        xa = sum(a and not b for a, b in prs)
        xb = sum(b and not a for a, b in prs)
        d, psi = (xb - xa) / n, (xa + xb) / n
        var = (psi - d * d) / n
        out[s] = {"n": n, "d": d, "psi": psi}
        if var > 0:
            terms.append((d, var))
    if len(terms) >= 2:
        w = [1 / v for _, v in terms]
        dbar = sum(wi * d for wi, (d, _) in zip(w, terms, strict=False)) / sum(w)
        chi2 = sum(wi * (d - dbar) ** 2 for wi, (d, _) in zip(w, terms, strict=False))
        from scipy.stats import chi2 as chi2_dist

        het = {
            "chi2": chi2,
            "df": len(terms) - 1,
            "p": float(chi2_dist.sf(chi2, len(terms) - 1)),
            "d_weighted": dbar,
        }
    else:
        het = None
    return {"per_seed": out, "heterogeneity": het}


def worst_case(test_tab, dropped_n):
    n = test_tab["n"] + dropped_n
    xa = test_tab["cf_only"] + dropped_n
    t = cd.tango_noninferiority(xa, test_tab["mpc_only"], n, MARGIN, alpha=ALPHA)
    return {"n": n, "cf_only": xa, "z": t["z"], "p": t["p"]}


def verdict(test, limits, solves_cf, solves_mpc, regression):
    items = {}
    items["1_noninferiority"] = "PASS" if test["noninferior"] else "FAIL"
    if limits["trials_with_window"] == 0:
        items["2_limits"] = "NOT_EVALUABLE"
    else:
        items["2_limits"] = "PASS" if limits["violating_trials"] == 0 else "FAIL"
    parts = [
        ("search_cf", solves_cf["search"]),
        ("search_mpc", solves_mpc["search"]),
        ("first", solves_mpc.get("first", {})),
        ("replan", solves_mpc.get("replan", {})),
    ]
    sub = {}
    for name, blk in parts:
        p = blk.get("pass")
        sub[name] = "NOT_EVALUABLE" if p is None else ("PASS" if p else "FAIL")
    items["3_solve_p99"] = (
        "FAIL"
        if "FAIL" in sub.values()
        else "NOT_EVALUABLE"
        if "NOT_EVALUABLE" in sub.values()
        else "PASS"
    )
    items["3_detail"] = sub
    items["4_regression"] = {"pass": "PASS", "fail": "FAIL"}.get(regression, "NOT_EVALUABLE")
    top = [v for k, v in items.items() if k[0].isdigit() and not k.endswith("detail")]
    if "FAIL" in top:
        label = "FAIL"
    elif "NOT_EVALUABLE" in top or "NOT_EVALUABLE" in sub.values():
        missing = [k for k, v in items.items() if v == "NOT_EVALUABLE"]
        missing += [f"3.{k}" for k, v in sub.items() if v == "NOT_EVALUABLE"]
        label = f"PASS 아님 (평가 불가: {', '.join(missing)})"
    else:
        label = "PASS"
    return items, label


def limits_block(rows):
    win = [r for r in rows if "limits" in r]
    bad = [r for r in win if r["limits"]["pos"] or r["limits"]["vel"] or r["limits"]["torque"]]
    return {
        "trials": len(rows),
        "trials_with_window": len(win),
        "violating_trials": len(bad),
        "violations": {k: sum(r["limits"][k] for r in win) for k in ("pos", "vel", "torque")},
        "pos_margin_min": min((r["limits"]["pos_margin_min"] for r in win), default=None),
        "vel_ratio_max": max((r["limits"]["vel_ratio_max"] for r in win), default=None),
        "torque_ratio_max": max((r["limits"]["torque_ratio_max"] for r in win), default=None),
        "violating": [(r["unit"], r["idx"]) for r in bad],
    }


def hold_limits(rows):
    h = [r["hold_limits"] for r in rows if "hold_limits" in r]
    return {
        "trials": len(h),
        "violations": {k: sum(x[k] for x in h) for k in ("pos", "vel", "torque")},
        "vel_ratio_max": max((x["vel_ratio_max"] for x in h), default=None),
        "torque_ratio_max": max((x["torque_ratio_max"] for x in h), default=None),
    }


def failure_modes(rows):
    valid = [r for r in rows if not r["invalid_reason"] and "approach" in r]
    aborts = collections.Counter(a for r in valid for a in r.get("abort_reasons", []))
    return {
        "valid_trials": len(valid),
        "abort_trials": sum(1 for r in valid if r.get("abort_reasons")),
        "abort_reasons": dict(aborts),
        "no_approach": sum(1 for r in valid if not r["approach"]),
        "no_approach_reasons": dict(
            collections.Counter(r["no_plan_reason"] for r in valid if not r["approach"])
        ),
        "plan_adopted": sum(1 for r in valid if r["plan_adopted"]),
        "committed": sum(1 for r in valid if r["committed"]),
        "ref_saturated_aborts": aborts.get("ref_saturated", 0),
    }


def peaks(rows):
    out = {}
    for k in ("qdd_cmd_peak", "qdd_meas_peak", "jerk_cmd_peak", "jerk_meas_peak"):
        out[k] = dist([r.get(k) for r in rows if not r["invalid_reason"]], (0.5, 0.95))
    out["axis_err_deg_median"] = median([r.get("axis_err_deg") for r in rows])
    return out


def refuse_shared_names(units):
    """The rows of a unit are kept by its directory name. Two units of one name
    (closed_form/p1b_631 and mpc/p1b_631, or one unit under both flags) would
    both read the later one's rows, and the test would compare an arm with
    itself — a pass that says nothing."""
    names = collections.Counter(Path(u).name for u in units)
    twice = sorted(n for n, c in names.items() if c > 1)
    if twice:
        raise SystemExit(
            f"unit name twice across the units given: {', '.join(twice)} (rows are kept by name)"
        )


# ── N arms, a record (#746) ──────────────────────────────────────────────────
# What each arm did on the same throws. Nothing is judged: no arm is the control,
# no margin decides, and the Tango row is printed as a reference beside the
# descriptive numbers.
Arm = collections.namedtuple("Arm", "search segment ball")
# (field, the mirror that says it, what conditions.txt was told to expect)
ARM_FIELDS = (
    ("search", "planner.search.mode", "expect_search"),
    ("segment", "planner.segment.mode", "expect_mode"),
    ("ball", "ball_type", "expect_ball"),
)
# The budget of each segment planner, by its own mirror names [s].
SEGMENT_BUDGET_KEYS = {
    "mpc": {
        "first": "planner.segment.mpc.budget.first_s",
        "replan": "planner.segment.mpc.budget.replan_s",
    },
    "mpc_docking": {
        "first": "planner.segment.mpc_docking.budget.first_s",
        "replan": "planner.segment.mpc_docking.budget.replan_s",
    },
}
NLP_BUDGET_KEYS = {
    "wake_s": "planner.search.nlp.budget.budget_s",
    "solve_s": "planner.search.nlp.budget.solve_s",
    "max_solves": "planner.search.nlp.budget.max_solves",
}
# planner_solves' kinds, grouped as the budgets are: a withheld replacement's first
# solve runs under the first-solve budget.
SOLVE_KINDS = (
    ("first", ("first",), "first"),
    ("replan", REPLAN, "replan"),
    ("repl_first", (ps.WITHHELD_REPLACEMENT_KIND,), "first"),
)
# #807: a ball that crosses the hand's entrance plane within this of rho_ref is caught.
ENT_NEAR_MM = 10.0
CATCH_FRAME_MEDIANS = (
    "cf_tot_s_mm",
    "cf_ref_s_mm",
    "cf_clik_s_mm",
    "cf_servo_s_mm",
    "cf_tot_lateral_margin_mm",
    "cf_ref_lateral_margin_mm",
    "ent_cross_ms",
    "ent_lat_mm",
    "ent_lateral_margin_mm",
    "ent_v_perp_m_s",
    "last_seg_wake_to_tc_ms",
    "last_seg_solve_ms",
    "last_seg_pred_age_ms",
)
TANGO_NOTE = "reference row (b not worse than a by the margin) — not a verdict of this record"


def mirror_text(mirror, key):
    """A string mirror's value (``<key>: String value is: <value>``), or None."""
    m = re.search(r"String value is: (\S+)$", mirror.get(key, ""))
    return m.group(1) if m else None


def arm_label(arm):
    return f"{arm.search}×{arm.segment}·{arm.ball}"


def arm_order(arm):
    """Tennis first, then by search and segment name: A, B, C, C′ of #746 in that order."""
    return (arm.ball != "tennis", arm.ball, arm.search, arm.segment)


def list_tag(sha):
    """A throw list's name in a record: its sha256 prefix (``series`` for a seeded run)."""
    return f"list:{sha[:12]}" if sha else "series"


def unit_identity(unit):
    """A unit's arm, repetition and overlay, read off the unit (``mirror.txt`` and
    ``conditions.txt``) — never off a flag or its directory name. Refused when a mirror
    does not say the arm, when ``conditions.txt`` was told to expect another (a unit from
    before #746 has no ``expect_search`` / ``expect_ball``: the mirror alone then says
    it), or when it has no seed."""
    unit = Path(unit)
    cond, mirror = kv_file(unit / "conditions.txt"), kv_file(unit / "mirror.txt")
    values = {}
    for field, key, expect in ARM_FIELDS:
        got = mirror_text(mirror, key)
        if got is None:
            raise SystemExit(
                f"{unit}: mirror.txt has no {key} — the arm cannot be read off the unit"
            )
        told = cond.get(expect)
        if told not in (None, "", got):
            raise SystemExit(
                f"{unit}: conditions.txt says {expect}: {told}, the mirror says {key}: {got}"
            )
        values[field] = got
    if not cond.get("seed"):
        raise SystemExit(f"{unit}: conditions.txt has no seed — the repetition is not known")
    return {
        "arm": Arm(**values),
        "seed": cond["seed"],
        "list": cond.get("throws_file_sha256", ""),
        "overlay": cond.get("overlay", ""),
        "robot": cond.get("robot", ""),
    }


def record_pairs(arms, all_pairs=False):
    """Which arms are tabled against each other, and what the two differ in: the planner
    (search or segment) with the ball the same, or the ball with the planner the same.
    ``all_pairs`` adds the pairs that differ in both."""
    out = []
    for a, b in itertools.combinations(sorted(arms, key=arm_order), 2):
        planner = (a.search, a.segment) != (b.search, b.segment)
        ball = a.ball != b.ball
        if all_pairs or planner != ball:
            out.append(
                (a, b, "planner+ball" if planner and ball else "planner" if planner else "ball")
            )
    return out


def ab_table(valid, field):
    """:func:`table` with the two arms called a and b."""
    t = table(valid, field)
    return {
        "n": t["n"],
        "both": t["both"],
        "a_only": t["cf_only"],
        "b_only": t["mpc_only"],
        "neither": t["neither"],
    }


def describe(tab):
    """A 2x2 table of paired throws as descriptive statistics: each arm's rate with its
    Wilson interval, the paired difference b − a with its score interval, McNemar's exact
    p, and the Tango row — a reference, at the margin the G-1 verdict used. An empty
    table is returned as it is."""
    n, xa, xb = tab["n"], tab["a_only"], tab["b_only"]
    out = dict(tab)
    if n == 0:
        return out
    ka, kb = tab["both"] + xa, tab["both"] + xb
    t = cd.tango_noninferiority(xa, xb, n, MARGIN, alpha=ALPHA)
    out.update(
        k_a=ka,
        k_b=kb,
        p_a=ka / n,
        p_b=kb / n,
        wilson95_a=list(ct.wilson_interval(ka, n)),
        wilson95_b=list(ct.wilson_interval(kb, n)),
        diff=(xb - xa) / n,
        score_ci95=cd.tango_score_ci(xa, xb, n),
        mcnemar_exact_p=ct.mcnemar_exact(xa, xb),
        tango_reference={
            "margin": MARGIN,
            "alpha": ALPHA,
            "z": t["z"],
            "p": t["p"],
            "note": TANGO_NOTE,
        },
    )
    return out


def rates(rows):
    """An arm's own rates over its valid throws (paired or not)."""
    valid = [r for r in rows if not r["invalid_reason"]]
    n = len(valid)
    out = {"thrown": len(rows), "valid": n}
    for field in ("truth", "d4"):
        k = sum(1 for r in valid if r[field])
        out[field] = {
            "k": k,
            "p": k / n if n else None,
            "wilson95": list(ct.wilson_interval(k, n)) if n else None,
        }
    return out


def pair_record(units_a, units_b, rows, names=("a", "b")):
    """Two arms' units (``{(seed, list): unit}``) joined stratum by stratum — a stratum is
    one repetition of one list — and tabled per stratum, per list and pooled. The pooled
    and per-list tables count a throw once per repetition it was thrown in."""
    strata = sorted(set(units_a) & set(units_b))
    pooled, by_list, by_stratum = [], collections.defaultdict(list), {}
    for key in strata:
        ua, ub = units_a[key], units_b[key]
        pr = pair_rows(rows[ua["name"]], rows[ub["name"]], names)
        valid = pr.pop("valid")
        by_stratum[f"seed {key[0]} · {list_tag(key[1])}"] = {
            "units": [ua["name"], ub["name"]],
            "pairs": pr,
            **{f: describe(ab_table(valid, f)) for f in ("truth", "d4")},
        }
        pooled += valid
        by_list[list_tag(key[1])] += valid
    return {
        "strata": len(strata),
        "strata_only_a": [
            f"seed {k[0]} · {list_tag(k[1])}" for k in sorted(set(units_a) - set(units_b))
        ],
        "strata_only_b": [
            f"seed {k[0]} · {list_tag(k[1])}" for k in sorted(set(units_b) - set(units_a))
        ],
        "pooled": {f: describe(ab_table(pooled, f)) for f in ("truth", "d4")},
        "by_list": {
            tag: {f: describe(ab_table(v, f)) for f in ("truth", "d4")}
            for tag, v in sorted(by_list.items())
        },
        "by_stratum": by_stratum,
    }


def repeat_record(units, rows):
    """One arm against itself: its units of one list under two seeds, joined throw by
    throw. How much a repetition (the estimator's noise alone) moves the outcome."""
    by_list = collections.defaultdict(dict)
    for (seed, sha), u in units.items():
        by_list[sha][seed] = u
    out = {}
    for sha, seeds in sorted(by_list.items()):
        for s1, s2 in itertools.combinations(sorted(seeds), 2):
            pr = pair_rows(rows[seeds[s1]["name"]], rows[seeds[s2]["name"]])
            valid = pr.pop("valid")
            out[f"{list_tag(sha)} · seed {s1} (a) vs {s2} (b)"] = {
                "pairs": pr,
                **{f: describe(ab_table(valid, f)) for f in ("truth", "d4")},
            }
    return out


def _valid_ct(units, extra=None):
    """The arm's valid throws as ``catching_trials.csv`` rows, units stacked. ``extra`` adds
    columns per unit (``extra(unit, frame) -> {name: values}``)."""
    frames = []
    for u in units:
        c = u["ct"]
        c = c[c["invalid_reason"].fillna("").eq("")]
        frames.append(c.assign(**extra(u, c)) if extra else c)
    return pd.concat(frames, ignore_index=True)


def _truth(frame):
    return frame["truth_success"].astype(str).str.lower().eq("true")


def arm_plan(units):
    """What the planner did with the arm's valid throws, as ``catching_trials`` counted it
    per throw: the plan verdict (published · withheld · …), the reject reasons, the throws
    the search accepted on some wake, the throws that lost a first solve to the deadline,
    and the catches under each verdict. A column the tool did not write is left out."""
    c = _valid_ct(units)
    out = {"valid": int(len(c))}

    def tally(col):
        v = c[col].fillna("").astype(str)
        return {str(k): int(n) for k, n in v[v != ""].value_counts().items()}

    for name, col in (
        ("verdict", "plan_verdict"),
        ("reject_most_frequent", "plan_reject"),
        ("reject_last", "plan_reject_last"),
    ):
        if col in c:
            out[name] = tally(col)
    if "search_valid_cycles" in c:
        out["search_accepted"] = int(
            (pd.to_numeric(c["search_valid_cycles"], errors="coerce") > 0).sum()
        )
    if "first_solve_cut" in c:
        out["throws_with_a_first_solve_cut"] = int(
            (pd.to_numeric(c["first_solve_cut"], errors="coerce") > 0).sum()
        )
    if "plan_verdict" in c:
        verdict, truth = c["plan_verdict"].fillna("").astype(str), _truth(c)
        out["caught_by_verdict"] = {k: int((truth & (verdict == k)).sum()) for k in out["verdict"]}
    return out


def arm_catch_frame(units):
    """Where the ball crossed the hand's entrance plane, on the arm's valid throws a plan
    was published for (#807): how many crossed, how many within ``ENT_NEAR_MM`` of
    ``rho_ref`` and how many of those were caught, how many inside the lateral set, and
    the medians of the catch-frame shares and of the last segment's timing. None when the
    units carry no entrance columns (a hand without a docking set, an older tool)."""

    def lateral(u, c):
        rho = (u["summ"].get("catch_frame") or {}).get("rho_ref_mm")
        if rho is None or "ent_x_mm" not in c:
            return {"ent_lat_mm": np.full(len(c), np.nan)}
        x = pd.to_numeric(c["ent_x_mm"], errors="coerce").to_numpy(float)
        y = pd.to_numeric(c["ent_y_mm"], errors="coerce").to_numpy(float)
        return {"ent_lat_mm": np.hypot(x - rho[0], y - rho[1])}

    c = _valid_ct(units, lateral)
    if "plan_verdict" not in c or "ent_x_mm" not in c:
        return None
    c = c[c["plan_verdict"].astype(str) == ct.PLAN_VERDICT_PUBLISHED]
    lat = c["ent_lat_mm"].to_numpy(float)
    near, truth = lat <= ENT_NEAR_MM, _truth(c).to_numpy()
    out = {
        "published": int(len(c)),
        "caught": int(truth.sum()),
        "crossed_entrance": int(np.isfinite(lat).sum()),
        "within_near_mm_of_rho_ref": int(near.sum()),
        "caught_of_those": int((near & truth).sum()),
        "near_mm": ENT_NEAR_MM,
        "medians": {},
    }
    if "ent_lateral_margin_mm" in c:
        out["inside_lateral_set"] = int(
            (pd.to_numeric(c["ent_lateral_margin_mm"], errors="coerce") > 0).sum()
        )
    for k in CATCH_FRAME_MEDIANS:
        if k in c:
            out["medians"][k] = median(pd.to_numeric(c[k], errors="coerce"))
    return out


def _mirror_number(units, key):
    """A numeric mirror the arm's units agree on; None when no unit has it (the
    parameter of another planner); refused when they differ."""
    values = {mirror_value(u["mirror"], key) for u in units}
    if len(values) != 1:
        raise SystemExit(f"{key} differs across the units of one arm: {sorted(map(str, values))}")
    return values.pop()


def arm_solves(units, arm):
    """The arm's solves against the budget of ITS planner. The grid search's time and the
    wake counts are :func:`solve_block`'s. The segment solves come from ``planner_solves``:
    per kind, the solves that ended as a distribution, the ones the core cut at its
    deadline as a count and a share — a cut solve's time is the instant it was cut at and
    is in no distribution here — and the p99 over both with the flag that says when it is
    only a lower bound. The NLP search's wakes are ``planner_solves.nlp_summary``."""
    base = solve_block(units, False)
    out = {
        k: base[k]
        for k in ("search", "held", "outcomes", "wakes_over_33ms", "wake_ms_max")
        if k in base
    }
    pe = pd.concat([u["pe"] for u in units], ignore_index=True)
    solves = ps.solve_table(pe)
    keys = SEGMENT_BUDGET_KEYS.get(arm.segment, {})
    segment = {}
    for name, kinds, budget in SOLVE_KINDS:
        sel = solves[solves["kind"].isin(kinds)]
        if sel.empty:
            continue
        blk = ps.summarise_times(sel["ms"].to_numpy(float), sel["cut"].to_numpy(bool))
        blk["cut_share"] = blk["n_cut"] / blk["n"]
        blk["outcomes"] = {str(k): int(v) for k, v in sel["outcome"].value_counts().items()}
        budget_s = _mirror_number(units, keys[budget]) if budget in keys else None
        blk["budget_ms"] = None if budget_s is None else budget_s * 1e3
        done = blk["done"]
        blk["done_p99_within_budget"] = (
            None if done is None or blk["budget_ms"] is None else done["p99"] <= blk["budget_ms"]
        )
        segment[name] = blk
    out["segment"] = segment
    nlp = ps.nlp_summary(pe)
    if nlp is not None:
        nlp["budget"] = {name: _mirror_number(units, key) for name, key in NLP_BUDGET_KEYS.items()}
        times = nlp.get("solve_ms_max")
        if times and times["n"]:
            nlp["wakes_with_a_deadline_reject_share"] = times["n_cut"] / times["n"]
        out["nlp"] = nlp
    return out


def arm_record(arm, units, rows):
    """Everything the record says of one arm (``units``: ``{(seed, list): unit}``)."""
    loaded = [units[k] for k in sorted(units)]
    arm_rows = [r for u in loaded for r in rows[u["name"]]]
    by_list = collections.defaultdict(list)
    for (_, sha), u in sorted(units.items()):
        by_list[list_tag(sha)] += rows[u["name"]]
    return {
        "search": arm.search,
        "segment": arm.segment,
        "ball": arm.ball,
        "units": {f"seed {k[0]} · {list_tag(k[1])}": units[k]["name"] for k in sorted(units)},
        "rates": {
            "all": rates(arm_rows),
            "by_list": {tag: rates(v) for tag, v in sorted(by_list.items())},
            "by_unit": {u["name"]: rates(rows[u["name"]]) for u in loaded},
        },
        "plan": arm_plan(loaded),
        "catch_frame": arm_catch_frame(loaded),
        "solves": arm_solves(loaded, arm),
        "limits": limits_block(arm_rows),
        "hold_limits": hold_limits(arm_rows),
        "failure_modes": failure_modes(arm_rows),
        "peaks_approach_to_decel_end": peaks(arm_rows),
        "decel_stop": cd.pool([u["dec"] for u in loaded])["decel"],
        "t_c": unit_t_c(loaded),
        "replan": replan_block(loaded),
        "rt_tick": rt_tick_block(loaded, rows),
        "repeat": repeat_record(units, rows),
        "conditions": {
            "retries": {u["name"]: u["retries"] for u in loaded},
            "rtf_trial_min": min(
                float(pd.to_numeric(u["ct"]["rtf_trial_min"], errors="coerce").min())
                for u in loaded
            ),
            "loadavg_start": {u["name"]: u["cond"].get("loadavg_start") for u in loaded},
            "overlay": sorted({u["cond"].get("overlay", "") for u in loaded}),
            **{
                k: sorted({u["cond"].get(k, "") for u in loaded})
                for k in (
                    "rtc_framework_rev",
                    "rtc_framework_dirty",
                    "ball_perception_rev",
                    "profile_sha256",
                    "planner_budget_s",
                )
            },
        },
    }


def record(cfg, unit_dirs, all_pairs=False, regression="unknown"):
    """The record of N arms as a dict (module docstring)."""
    from rtc_tools.analysis.derive_accel_limits import resolve_urdf_text

    refuse_shared_names(unit_dirs)
    ids = {Path(u).name: unit_identity(u) for u in unit_dirs}
    robots = sorted({i["robot"] for i in ids.values()})
    if len(robots) != 1:
        raise SystemExit(f"a record is one robot's: the units say {robots}")
    by_arm = collections.defaultdict(dict)
    for u in unit_dirs:
        i = ids[Path(u).name]
        key = (i["seed"], i["list"])
        if key in by_arm[i["arm"]]:
            raise SystemExit(
                f"{arm_label(i['arm'])}: two units of seed {key[0]} · {list_tag(key[1])} "
                f"({by_arm[i['arm']][key]} and {Path(u).name}) — one unit per arm, seed and list"
            )
        by_arm[i["arm"]][key] = Path(u).name
    for name, i in ids.items():
        if not Path(i["overlay"]).is_file():
            raise SystemExit(
                f"{name}: the overlay its conditions.txt names is gone: {i['overlay']!r}"
            )
    loaded = {Path(u).name: load_unit(u, cfg, ids[Path(u).name]["overlay"]) for u in unit_dirs}
    first = next(iter(loaded.values()))
    profile = ct.load_profile(cfg, None, first["unit"] / "session")
    urdf_text, _ = resolve_urdf_text(profile.robot_params, None)
    fk = ct.CatchFrameFk(urdf_text, first["joints"], profile)
    rows = {name: trial_rows(u, cfg, fk) for name, u in loaded.items()}
    arms = sorted(by_arm, key=arm_order)
    units = {arm: {key: loaded[name] for key, name in by_arm[arm].items()} for arm in arms}
    mismatch = [
        (r["unit"], r["idx"], r["hold_no_abort"], r["mode_log_ok"])
        for unit_rows in rows.values()
        for r in unit_rows
        if not r["invalid_reason"]
        and r["hold_no_abort"] is not None
        and r["hold_no_abort"] != r["mode_log_ok"]
    ]
    return {
        "record": "N arms on the same throws — nothing is judged (#746)",
        "robot": robots[0],
        "arms": {arm_label(arm): arm_record(arm, units[arm], rows) for arm in arms},
        "pairs": {
            f"{arm_label(a)} (a) | {arm_label(b)} (b)": {
                "a": arm_label(a),
                "b": arm_label(b),
                "differs_in": what,
                **pair_record(units[a], units[b], rows),
            }
            for a, b, what in record_pairs(arms, all_pairs)
        },
        "mode_log_mismatch": mismatch,
        "regression": regression,
    }


def _fmt_ci(ci):
    return "–" if ci is None else f"[{ci[0]:.3f}, {ci[1]:.3f}]"


def _fmt_table(name, t):
    """One described 2x2 table as a line."""
    if t["n"] == 0:
        return f"    {name}: no valid pair"
    ref = t["tango_reference"]
    return (
        f"    {name}: n {t['n']} · both {t['both']} · a only {t['a_only']} · b only {t['b_only']} · "
        f"neither {t['neither']} | a {t['k_a']}/{t['n']} = {t['p_a']:.3f} {_fmt_ci(t['wilson95_a'])} · "
        f"b {t['k_b']}/{t['n']} = {t['p_b']:.3f} {_fmt_ci(t['wilson95_b'])} | b − a {t['diff']:+.4f} "
        f"score [{t['score_ci95'][0]:+.4f}, {t['score_ci95'][1]:+.4f}] · McNemar p "
        f"{t['mcnemar_exact_p']:.3g} · (Tango δ {ref['margin']:g}: Z {ref['z']:.3f} p {ref['p']:.3g} — reference)"
    )


def _fmt_rate(r, field):
    v = r[field]
    if not r["valid"]:
        return f"{field} –"
    return f"{field} {v['k']}/{r['valid']} = {v['p']:.3f} {_fmt_ci(v['wilson95'])}"


def format_record(res):
    """The record as text lines."""
    lines = [f"{res['record']} — {res['robot']}, {len(res['arms'])} arms"]
    for label, a in res["arms"].items():
        r = a["rates"]["all"]
        lines.append(
            f"arm {label}: {len(a['units'])} units · thrown {r['thrown']} · valid {r['valid']}"
        )
        lines.append(f"  all: {_fmt_rate(r, 'truth')} · {_fmt_rate(r, 'd4')}")
        for tag, v in a["rates"]["by_list"].items():
            lines.append(
                f"  {tag} (valid {v['valid']}): {_fmt_rate(v, 'truth')} · {_fmt_rate(v, 'd4')}"
            )
        for name, v in a["rates"]["by_unit"].items():
            lines.append(
                f"    {name} (valid {v['valid']}): {_fmt_rate(v, 'truth')} · {_fmt_rate(v, 'd4')}"
            )
        lines.append(f"  plan: {json.dumps(a['plan'], default=_json, ensure_ascii=False)}")
        if a["catch_frame"] is not None:
            lines.append(f"  entrance (#807): {json.dumps(a['catch_frame'], default=_json)}")
        s = a["solves"]
        lines.append(f"  search: {json.dumps(s['search'], default=_json)}")
        for kind, blk in s["segment"].items():
            done = blk["done"]
            ended = (
                "none ended"
                if done is None
                else f"ended {blk['n_done']}: p50 {done['p50']:.2f} p99 {done['p99']:.2f} max {done['max']:.2f} ms"
            )
            budget = "–" if blk["budget_ms"] is None else f"{blk['budget_ms']:.1f} ms"
            lines.append(
                f"  segment {kind}: {blk['n']} solves · {ended} · cut at the deadline {blk['n_cut']} "
                f"({blk['cut_share']:.1%}) · budget {budget} · ended p99 within budget "
                f"{blk['done_p99_within_budget']} · outcomes {blk['outcomes']}"
            )
        if "nlp" in s:
            n = s["nlp"]
            lines.append(
                f"  nlp search: {n['wakes']} wakes · reasons {n.get('reasons')} · rejects {n['rejects']} "
                f"· budget {n['budget']}"
            )
            times = n.get("solve_ms_max")
            if times and times["n"]:
                lines.append(
                    "    slowest candidate solve per wake [ms]: " + ps.format_nlp_solve_time(times)
                )
        lim = {k: v for k, v in a["limits"].items() if k != "violating"}
        lines.append(f"  limits (APPROACH → DECEL end): {json.dumps(lim, default=_json)}")
        lines.append(f"  failure modes: {json.dumps(a['failure_modes'], default=_json)}")
        for name, rep in a["repeat"].items():
            lines.append(f"  repeat {name}:")
            lines += [_fmt_table(f, rep[f]) for f in ("truth", "d4")]
    for name, p in res["pairs"].items():
        lines.append(f"pair {name} — differs in the {p['differs_in']}, {p['strata']} strata")
        if p["strata_only_a"] or p["strata_only_b"]:
            lines.append(
                f"  strata of one arm only: a {p['strata_only_a']} · b {p['strata_only_b']}"
            )
        # One stratum is its own pool, and one list its own per-list table: said once.
        if p["strata"] > 1:
            lines.append("  pooled (a throw counts once per repetition):")
            lines += [_fmt_table(f, p["pooled"][f]) for f in ("truth", "d4")]
        if len(p["by_list"]) > 1:
            for tag, v in p["by_list"].items():
                lines.append(f"  {tag}:")
                lines += [_fmt_table(f, v[f]) for f in ("truth", "d4")]
        for tag, v in p["by_stratum"].items():
            pr = v["pairs"]
            lines.append(
                f"  {tag} ({' | '.join(v['units'])}): thrown keys {pr['keys']} · dropped {pr['dropped_n']} "
                f"{pr['dropped']} · unkeyed {pr['unkeyed']} · to rerun {pr['seeds_to_rerun']}"
            )
            lines += [_fmt_table(f, v[f]) for f in ("truth", "d4")]
    lines.append(
        f"mode_log mismatches: {len(res['mode_log_mismatch'])} {res['mode_log_mismatch'][:5]}"
    )
    lines.append(f"regression: {res['regression']}")
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", type=Path, required=True)
    ap.add_argument("--overlay-cf", type=Path)
    ap.add_argument("--overlay-mpc", type=Path)
    ap.add_argument("--cf", nargs="+")
    ap.add_argument("--mpc", nargs="+")
    ap.add_argument(
        "--unit",
        nargs="+",
        help="a record of N arms (#746): the units of every arm; each unit says its own arm",
    )
    ap.add_argument(
        "--all-pairs",
        action="store_true",
        help="with --unit: also table the arms that differ in the planner AND the ball",
    )
    ap.add_argument("--regression", choices=("pass", "fail", "unknown"), default="unknown")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    verdict_flags = {
        "--overlay-cf": a.overlay_cf,
        "--overlay-mpc": a.overlay_mpc,
        "--cf": a.cf,
        "--mpc": a.mpc,
    }
    if a.unit:
        given = [k for k, v in verdict_flags.items() if v]
        if given:
            ap.error(f"--unit is the record of N arms: it takes no {' / '.join(given)}")
        res = record(a.cfg, a.unit, a.all_pairs, a.regression)
        if a.json:
            a.json.write_text(json.dumps(res, indent=1, default=_json))
        print("\n".join(format_record(res)))
        return
    missing = [k for k, v in verdict_flags.items() if not v]
    if missing or a.all_pairs:
        ap.error(
            f"the G-1 verdict needs {' '.join(verdict_flags)} (missing: {' '.join(missing) or '-'}) "
            "and no --all-pairs; a record of N arms is --unit"
        )
    from rtc_tools.analysis.derive_accel_limits import resolve_urdf_text

    refuse_shared_names(a.cf + a.mpc)
    CF = [load_unit(u, a.cfg, a.overlay_cf) for u in a.cf]
    MP = [load_unit(u, a.cfg, a.overlay_mpc) for u in a.mpc]
    profile = ct.load_profile(a.cfg, None, CF[0]["unit"] / "session")
    urdf_text, _ = resolve_urdf_text(profile.robot_params, None)
    fk = ct.CatchFrameFk(urdf_text, CF[0]["joints"], profile)
    rows = {u["name"]: trial_rows(u, a.cfg, fk) for u in CF + MP}
    cf_rows = [r for u in CF for r in rows[u["name"]]]
    mp_rows = [r for u in MP for r in rows[u["name"]]]

    # The mode-path cross-check: diag window vs the runner's mode_log.
    mismatch = [
        (r["unit"], r["idx"], r["hold_no_abort"], r["mode_log_ok"])
        for r in cf_rows + mp_rows
        if not r["invalid_reason"]
        and r["hold_no_abort"] is not None
        and r["hold_no_abort"] != r["mode_log_ok"]
    ]

    pr = pair(cf_rows, mp_rows)
    d4 = test_block(table(pr["valid"], "d4"))
    truth = test_block(table(pr["valid"], "truth"))
    sol_cf, sol_mpc = solve_block(CF, False), solve_block(MP, True)
    lim = limits_block(mp_rows)
    items, label = verdict(d4, lim, sol_cf, sol_mpc, a.regression)
    res = {
        "units": {"cf": [u["name"] for u in CF], "mpc": [u["name"] for u in MP]},
        "verdict": label,
        "items": items,
        "mode_log_mismatch": mismatch,
        "pairs": {k: v for k, v in pr.items() if k != "valid"},
        "test_d4": d4,
        "sensitivity_truth_only": truth,
        "sensitivity_worst_case": worst_case(table(pr["valid"], "d4"), pr["dropped_n"]),
        "seed": seed_block(pr["valid"]),
        "limits_mpc": lim,
        "limits_cf_report": limits_block(cf_rows),
        "hold_limits_report": {"cf": hold_limits(cf_rows), "mpc": hold_limits(mp_rows)},
        "solves": {"cf": sol_cf, "mpc": sol_mpc},
        "failure_modes": {"cf": failure_modes(cf_rows), "mpc": failure_modes(mp_rows)},
        "peaks_approach_to_decel_end": {"cf": peaks(cf_rows), "mpc": peaks(mp_rows)},
        "decel_stop": {
            "cf": cd.pool([u["dec"] for u in CF])["decel"],
            "mpc": cd.pool([u["dec"] for u in MP])["decel"],
        },
        "t_c": {"cf": unit_t_c(CF), "mpc": unit_t_c(MP)},
        "replan": replan_block(MP),
        "rt_tick": {"cf": rt_tick_block(CF, rows), "mpc": rt_tick_block(MP, rows)},
        "conditions": {
            arm: {
                "retries": {u["name"]: u["retries"] for u in U},
                "rtf_trial_min": min(
                    float(pd.to_numeric(u["ct"]["rtf_trial_min"], errors="coerce").min())
                    for u in U
                ),
                "loadavg_start": {u["name"]: u["cond"].get("loadavg_start") for u in U},
                "rev": sorted({u["cond"].get("rtc_framework_rev") for u in U}),
                "dirty": sorted({u["cond"].get("rtc_framework_dirty") for u in U}),
                "planner_budget_s": sorted({u["cond"].get("planner_budget_s") for u in U}),
            }
            for arm, U in (("cf", CF), ("mpc", MP))
        },
    }
    if a.json:
        a.json.write_text(json.dumps(res, indent=1, default=_json))
    t = d4
    print(f"verdict: {label}")
    print(f"items: {json.dumps(items, default=_json)}")
    print(
        f"pairs: thrown keys {pr['keys']} · valid {t['n']} · dropped {pr['dropped_n']} {pr['dropped']}"
        f" · unkeyed {pr['unkeyed']} · seeds (lists) to rerun {pr['seeds_to_rerun']}"
    )
    print(
        f"D4 table (both / mpc only / cf only / neither): {t['both']} / {t['mpc_only']} / "
        f"{t['cf_only']} / {t['neither']} · p_mpc {t['p_mpc']:.3f} p_cf {t['p_cf']:.3f}"
    )
    print(
        f"  d̂ {t['diff']:+.4f} · Tango Z {t['z']:.4f} p {t['p']:.4g} → "
        f"{'non-inferior' if t['noninferior'] else 'non-inferiority not shown'} · score "
        f"[{t['score_ci95'][0]:+.4f}, {t['score_ci95'][1]:+.4f}] · Wald "
        f"[{t['wald_ci95'][0]:+.4f}, {t['wald_ci95'][1]:+.4f}] · McNemar p {t['mcnemar_exact_p']:.3g}"
    )
    print(
        f"  power design {t['power_design']} · observed {t['power_observed']} · size {t['size_at_psi_hat']}"
    )
    print(
        f"truth-only: {truth['both']}/{truth['mpc_only']}/{truth['cf_only']}/{truth['neither']} "
        f"Z {truth['z']:.4f} p {truth['p']:.4g}"
    )
    print(f"mode_log mismatches: {len(mismatch)} {mismatch[:5]}")
    print(
        f"limits (mpc, criterion 2): {json.dumps({k: v for k, v in lim.items() if k != 'violating'}, default=_json)}"
    )
    for arm in ("cf", "mpc"):
        s = res["solves"][arm]
        print(f"solves {arm}: search {json.dumps(s['search'], default=_json)}")
        for k in ("first", "replan"):
            if k in s:
                print(f"    {k} {json.dumps(s[k], default=_json)}")
    print(f"failure modes: {json.dumps(res['failure_modes'], default=_json)}")
    print(f"seed: {json.dumps(res['seed'], default=_json)}")


def _json(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    return str(o)


if __name__ == "__main__":
    main()
