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
"""

import argparse
import collections
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
)
from rtc_tools.utils.catching_keys import normalize_mirror

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
    recs = json.loads((unit / "trials" / "trial_results.json").read_text())
    recs = recs["trials"] if isinstance(recs, dict) else recs
    rec = {int(r["idx"]): r for r in recs}
    ctl = unit / "session" / "controllers" / CTRL
    joints = summ["arm_joints"]
    arm = summ["arm_device"]
    want = {"t_relative_s", "tick", "mode", "reason_name", "decel_event", "decel_rho"}
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
    if "decel_kind" in pe:
        unknown = set(pe["decel_outcome"].dropna()) - POST_SOLVE - PRE_SOLVE
        if unknown:
            raise SystemExit(
                f"decel_outcome outside both lists: {sorted(unknown)} — analysis stops"
            )
        solved = pe[pe["decel_outcome"].isin(POST_SOLVE)]
        out["solve_us_sanity"] = {
            "post_solve_zero": int((solved["decel_solve_us"] <= 0).sum()),
            "pre_solve_nonzero": int(
                (pe[pe["decel_outcome"].isin(PRE_SOLVE)]["decel_solve_us"] > 0).sum()
            ),
        }
        out["held"] = int(pe["decel_outcome"].isin(HELD).sum())
        out["outcomes"] = pe["decel_outcome"].value_counts().to_dict()
        firsts = {mirror_value(u["mirror"], "planner.segment.mpc.budget.first_s") for u in units}
        replans = {mirror_value(u["mirror"], "planner.segment.mpc.budget.replan_s") for u in units}
        for key, kinds, budgets_s in (("first", ("first",), firsts), ("replan", REPLAN, replans)):
            v = solved[solved["decel_kind"].isin(kinds)]["decel_solve_us"]
            blk = {"n": int(len(v)), **{k: x for k, x in dist(v).items() if k != "n"}}
            if arm_is_mpc:
                if len(budgets_s) != 1 or None in budgets_s:
                    raise SystemExit(
                        f"decel budget {key} differs or missing in mirrors: {budgets_s}"
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
        if "decel_event" not in d:
            return None
        m = d["mode"].to_numpy(int)
        ev = d["decel_event"].to_numpy(int)
        enter = np.r_[True, m[1:] != m[:-1]]
        trial = np.cumsum(enter & (m == ct.MODE_APPROACH))
        sw = np.flatnonzero(ev == EVENT_SWITCHED)
        later = (
            pd.Series(trial[sw]).duplicated().to_numpy()
        )  # a trial's first switch is not a replan
        switches += int(later.sum())
        rho += list(d["decel_rho"].to_numpy(float)[sw][later])
        refused += int(np.sum(ev == EVENT_GATE_REFUSED))
        if "decel_aged" in u["ct"]:
            aged += int(pd.to_numeric(u["ct"]["decel_aged"], errors="coerce").fillna(0).sum())
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
def pair(cf_rows, mpc_rows):
    def keyed(rows):
        out = {}
        for r in rows:
            k = (r["kind"], int(r["seed"]), int(r["sample_idx"]))
            if k in out:
                raise SystemExit(f"throw {k} twice in one arm")
            out[k] = r
        return out

    a, b = keyed(cf_rows), keyed(mpc_rows)
    keys = sorted(set(a) & set(b))
    dropped = collections.Counter()
    valid, per_seed_drop = [], collections.Counter()
    for k in keys:
        ra, rb = a[k], b[k]
        if ra["invalid_reason"] or rb["invalid_reason"]:
            side = (
                "both"
                if ra["invalid_reason"] and rb["invalid_reason"]
                else ("cf_only" if ra["invalid_reason"] else "mpc_only")
            )
            dropped[(side, ra["invalid_reason"] or "-", rb["invalid_reason"] or "-")] += 1
            per_seed_drop[k[1]] += 1
            continue
        valid.append((k, ra, rb))
    return {
        "keys": len(keys),
        "unpaired_cf": len(set(a) - set(b)),
        "unpaired_mpc": len(set(b) - set(a)),
        "valid": valid,
        "dropped": {"/".join(k): v for k, v in dropped.items()},
        "dropped_n": sum(dropped.values()),
        "seed_drops": dict(per_seed_drop),
        "seeds_to_rerun": sorted(s for s, n in per_seed_drop.items() if n >= DROP_SEED_RERUN),
    }


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
        by[k[1]].append((ra["d4"], rb["d4"]))
    out, terms = {}, []
    for s, prs in sorted(by.items()):
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
            f"unit name twice across --cf and --mpc: {', '.join(twice)} (rows are kept by name)"
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", type=Path, required=True)
    ap.add_argument("--overlay-cf", type=Path, required=True)
    ap.add_argument("--overlay-mpc", type=Path, required=True)
    ap.add_argument("--cf", nargs="+", required=True)
    ap.add_argument("--mpc", nargs="+", required=True)
    ap.add_argument("--regression", choices=("pass", "fail", "unknown"), default="unknown")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
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
        f" · seeds to rerun {pr['seeds_to_rerun']}"
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
