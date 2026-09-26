#!/usr/bin/env python3
"""Pool catching_trials outputs across units and arms (dynamic_catching S8-E).

Plan: ``docs/dynamic_catching/IMPLEMENTATION_PLAN.md`` §4.4 D-S8-16 and the
S8-E plan; §1a (G8-D definition, floor). One S8-E arm is several units (one sim
session + one ``catching_sim_trials`` trials dir each, 50 throws per unit), and
each unit has been through :mod:`rtc_tools.analysis.catching_trials`, which
leaves ``catching_trials.csv`` (one row per trial, with ``invalid_reason``) and
``catching_trials_summary.json`` in its output dir. This module pools those
dirs per arm and judges the arm:

* **truncation** (D-S8-16 ①b, the pre-declared top-up) — units are read IN THE
  ORDER GIVEN (the pre-declared seed order, top-up units last) and trials by
  ``idx``; trials count until the cumulative VALID count reaches
  ``--n-valid-target``, and every trial after the target-th valid one is
  ``beyond_target`` — out of both the valid and the ITT counts. Fewer valid
  trials than the target in total → ``INSUFFICIENT_N``;
* **G8-D** over the included valid trials: successes (sim truth), p̂, Wilson
  two-sided interval, ``lower_975`` ≥ ``--floor`` → PASS / FAIL; ITT (invalid
  counted as failure) over every included trial; supervisor × truth confusion;
  per-unit breakdown;
* the pooled covariates over the same trials: D-3 |δ(t_commit)|, |δ(t_c)|,
  δ_max; tick overrun; G8-C3 streak distribution; G7-B3 impulse correlation;
  gate-map whole / map-open subset when the rows carry ``map_open``;
* with ≥ 2 arms, an exact McNemar test per arm pair over the trials that are
  included and valid in BOTH, paired by ``(seed, idx)`` (D-S8-16 ④);
* the D-3 S3.1b count (D-S8-16 ⑤): trials with a clock covariate over all
  arms, plus the ``d3.paired`` counts of ``--extra-d3`` summaries, against the
  ≥ 200 requirement;
* G8-B (per-horizon pooled NEES, trial bootstrap) and G8-C2 (A⊥B, n ≥ 100)
  recomputed from the pooled per-trial columns — :mod:`catching_vision` — and
  each unit's ``time_alignment`` and the RTF covariates.

The statistics are :mod:`catching_trials`' / :mod:`catching_vision`'s own (one
implementation, P5).
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import sys
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from rtc_tools.analysis import catching_trials as ct, catching_vision as cv

TRIALS_CSV = "catching_trials.csv"
SUMMARY_JSON = "catching_trials_summary.json"
DEFAULT_N_VALID_TARGET = 200  # D-S8-3
D3_S31B_N_MIN = 200  # plan §5 S3.1b: trials with a clock covariate
POOL_COLUMNS = (
    "arm",
    "unit",
    "unit_dir",
    "seed",
    "idx",
    "included",
    "beyond_target",
    "invalid_reason",
    "truth_success",
    "supervisor",
    "delta_commit_ms",
    "delta_tc_ms",
    "delta_max_ms",
    "tick_overrun_n",
    "tick_jitter_max_us",
    "ref_saturated_max_streak",
    "contact_mv_rel_ns",
    "contact_impulse_ns",
    "contact_peak_force_n",
    "map_open",
    "time_alignment",
)
STRING_COLUMNS = frozenset(
    {
        "kind",
        "supervisor",
        "invalid_reason",
        "contact_body",
        "truth_reason",
        "time_alignment",
        "stamp_anchor",
        "c2_join",
        "c2_tc_source",
    }
)
# Per-trial columns copied into pool_trials.csv when present (G8-B, G8-C2, RTF).
POOL_COLUMN_PREFIXES = ("nees_h", "err_h", "c2_", "rtf_")


def parse_cell(key: str, text: str | None):
    """A ``catching_trials.csv`` cell back to its Python value ("" → None)."""
    if text is None or text == "":
        return None
    if key in STRING_COLUMNS:
        return text
    if text in ("True", "False"):
        return text == "True"
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


@dataclass
class Unit:
    ct_dir: Path
    rows: list[dict]
    summary: dict
    # Filled by :func:`truncate`: per row, included / beyond_target.
    included: list[bool] = field(default_factory=list)

    @property
    def seeds(self) -> list:
        seeds = {r["seed"] for r in self.rows if r.get("seed") is not None}
        return sorted(seeds) or list(self.summary.get("seeds", []))

    @property
    def lane_rules_evaluated(self) -> bool:
        return bool(self.summary.get("validity", {}).get("lane_rules_evaluated"))


def read_unit(ct_dir: Path) -> Unit:
    ct_dir = Path(ct_dir)
    csv_path, json_path = ct_dir / TRIALS_CSV, ct_dir / SUMMARY_JSON
    for path in (csv_path, json_path):
        if not path.is_file():
            raise SystemExit(f"{ct_dir}: missing {path.name} — not a catching_trials output dir")
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if "invalid_reason" not in (reader.fieldnames or []):
            raise SystemExit(
                f"{csv_path}: no invalid_reason column — written by a catching_trials older "
                "than the D-S8-16 validity rules; re-run catching_trials on this unit"
            )
        if "truth_success" not in (reader.fieldnames or []):
            raise SystemExit(f"{csv_path}: no truth_success column — run with a hold radius")
        rows = [{k: parse_cell(k, v) for k, v in r.items()} for r in reader]
    summary = json.loads(json_path.read_text())
    if "validity" not in summary:
        raise SystemExit(f"{json_path}: no validity block — re-run catching_trials")
    rows.sort(key=lambda r: r["idx"])
    return Unit(ct_dir, rows, summary)


def truncate(units: Sequence[Unit], n_valid_target: int) -> int:
    """Mark every row included or ``beyond_target`` (D-S8-16 ①b); returns the beyond count."""
    n_valid = 0
    beyond = 0
    for unit in units:
        unit.included = []
        for row in unit.rows:
            take = n_valid < n_valid_target
            unit.included.append(take)
            if not take:
                beyond += 1
            elif not row.get("invalid_reason"):
                n_valid += 1
    return beyond


def _included(units: Sequence[Unit]) -> list[dict]:
    return [r for u in units for r, inc in zip(u.rows, u.included, strict=True) if inc]


def _valid(rows: Sequence[Mapping]) -> list[Mapping]:
    return [r for r in rows if not r.get("invalid_reason")]


def d3_block(valid: Sequence[Mapping]) -> dict:
    return {
        "n_with_clock_covariate": sum(
            1 for r in valid if math.isfinite(ct._num(r.get("delta_max_ms")))
        ),
        "delta_commit_ms_abs_p50_p95_max": ct._p50_p95_max(
            valid, "delta_commit_ms", absolute=True
        ),
        "delta_tc_ms_abs_p50_p95_max": ct._p50_p95_max(valid, "delta_tc_ms", absolute=True),
        "delta_max_ms_p50_p95_max": ct._p50_p95_max(valid, "delta_max_ms"),
    }


def gate_map_block(valid: Sequence[Mapping], z: float) -> dict | None:
    """The session's gate-map block (:func:`catching_trials.gate_map_truth`) over pooled rows."""
    out = ct.gate_map_truth(valid, z)
    return out if out["verdicted"] else None


def tick_block(valid: Sequence[Mapping]) -> dict | str:
    if not any("tick_overrun_n" in r for r in valid):
        return "NOT_EVALUATED(no unit had a timing log joined to its trials)"
    return ct.tick_overrun_summary(valid, None, None)


def arm_summary(
    label: str,
    units: Sequence[Unit],
    floor: float,
    n_valid_target: int,
    z: float,
    n_boot: int,
    seed: int,
) -> dict:
    beyond = truncate(units, n_valid_target)
    included = _included(units)
    valid = _valid(included)
    unit_rows = []
    for u in units:
        inc = [r for r, i in zip(u.rows, u.included, strict=True) if i]
        v = _valid(inc)
        unit_rows.append(
            {
                "dir": str(u.ct_dir),
                "seeds": u.seeds,
                "n_rows": len(u.rows),
                "n_total": len(inc),
                "n_valid": len(v),
                "successes": sum(1 for r in v if ct._is_true(r.get("truth_success"))),
                "beyond_target": len(u.rows) - len(inc),
                "lane_rules_evaluated": u.lane_rules_evaluated,
            }
        )
    validity = ct.validity_block(included, all(u.lane_rules_evaluated for u in units))
    if not validity["lane_rules_evaluated"]:
        validity["lane_rules"] = "NOT_EVALUATED in unit(s) " + ", ".join(
            str(u.ct_dir) for u in units if not u.lane_rules_evaluated
        )
    validity["beyond_target"] = beyond
    alignment = {str(u.ct_dir): u.summary.get("time_alignment", "unknown") for u in units}
    validity["n_valid_target"] = n_valid_target
    return {
        "label": label,
        "units": unit_rows,
        "validity": validity,
        "truth": ct.truth_block(valid, len(included), floor, n_valid_target, z),
        "d3": d3_block(valid),
        "tick_overrun": tick_block(valid),
        "ref_saturated_streak": ct.streak_distribution(valid),
        "g7b3": ct.impulse_correlation(valid, n_boot, seed),
        "gate_map": gate_map_block(valid, z),
        "time_alignment": alignment,
        "rtf": ct.rtf_summary(valid),
        # Recomputed from the pooled per-trial columns (exact sums, trials resampled).
        "g8b": cv.g8b_summary(valid, n_boot, seed),
        # One (A, B) per trial, so each sample is its own trial cluster.
        "c2": cv.c2_summary(valid, n_boot, seed),
    }


def _pair_key(row: Mapping) -> tuple | None:
    return None if row.get("seed") is None else (row["seed"], row["idx"])


def check_unique_keys(label: str, units: Sequence[Unit]) -> None:
    """A (seed, idx) twice in one arm is two units run with one seed — ambiguous."""
    seen: dict[tuple, Path] = {}
    for u in units:
        for r in u.rows:
            key = _pair_key(r)
            if key is None:
                continue
            if key in seen:
                raise SystemExit(
                    f"arm {label}: (seed, idx) {key} in both {seen[key]} and {u.ct_dir} — a "
                    "re-run unit replaces its original (D-S8-16 ①), pass only one of them"
                )
            seen[key] = u.ct_dir


def mcnemar(label_a: str, a: Sequence[Mapping], label_b: str, b: Sequence[Mapping]) -> dict:
    """Exact McNemar over the (seed, idx) pairs included and valid in both arms."""
    rows_a = {_pair_key(r): r for r in _valid(a) if _pair_key(r) is not None}
    rows_b = {_pair_key(r): r for r in _valid(b) if _pair_key(r) is not None}
    no_seed = sum(1 for r in [*_valid(a), *_valid(b)] if _pair_key(r) is None)
    keys = sorted(set(rows_a) & set(rows_b))
    cells = {"both_success": 0, "a_success_b_fail": 0, "a_fail_b_success": 0, "both_fail": 0}
    for key in keys:
        sa = ct._is_true(rows_a[key].get("truth_success"))
        sb = ct._is_true(rows_b[key].get("truth_success"))
        name = {
            (True, True): "both_success",
            (True, False): "a_success_b_fail",
            (False, True): "a_fail_b_success",
            (False, False): "both_fail",
        }[(sa, sb)]
        cells[name] += 1
    bb, cc = cells["a_success_b_fail"], cells["a_fail_b_success"]
    return {
        "a": label_a,
        "b": label_b,
        "n_pairs": len(keys),
        **cells,
        "discordant": bb + cc,
        "p_exact_two_sided": ct.mcnemar_exact(bb, cc),
        "valid_rows_without_seed": no_seed,
    }


def load_extra_d3(path: Path) -> dict:
    doc = json.loads(Path(path).read_text())
    d3 = doc.get("d3")
    if not isinstance(d3, Mapping) or "paired" not in d3:
        raise SystemExit(f"{path}: no d3.paired — not a catching_trials summary with a clock lane")
    return {
        "source": str(path),
        "paired": int(d3["paired"]),
        "delta_max_ms_p50_p95_max": d3.get("delta_max_ms_p50_p95_max"),
        "delta_commit_ms_abs_p50_p95_max": d3.get("delta_commit_ms_p50_p95_max"),
        "delta_tc_ms_abs_p50_p95_max": d3.get("delta_tc_ms_p50_p95_max"),
    }


def d3_s31b_block(arms: Mapping[str, Sequence[Mapping]], extras: Sequence[Mapping]) -> dict:
    per_arm = {
        label: d3_block(_valid(rows))["n_with_clock_covariate"] for label, rows in arms.items()
    }
    extra = sum(e["paired"] for e in extras)
    total = sum(per_arm.values()) + extra
    pooled = d3_block([r for rows in arms.values() for r in _valid(rows)])
    return {
        "requirement_n": D3_S31B_N_MIN,
        "per_arm": per_arm,
        "extra_paired": extra,
        "total": total,
        "met": total >= D3_S31B_N_MIN,
        "pooled_over_arms": pooled,
        "extra_sources": list(extras),
        "note": "quantiles of --extra-d3 sources are listed per source — quantiles cannot be "
        "pooled from summaries; only their paired counts are summed",
    }


def pool(
    arms: Mapping[str, Sequence[Path]],
    floor: float,
    n_valid_target: int = DEFAULT_N_VALID_TARGET,
    z: float = 1.96,
    n_boot: int = ct.DEFAULT_N_BOOT,
    seed: int = ct.DEFAULT_SEED,
    extra_d3: Sequence[Path] = (),
) -> tuple[dict, list[dict]]:
    """Pool the arms; returns (summary, pool_trials rows)."""
    summary: dict = {
        "tool": "rtc_tools.analysis.catching_pool",
        "floor": floor,
        "n_valid_target": n_valid_target,
        "z": z,
        "arms": {},
    }
    included_by_arm: dict[str, list[dict]] = {}
    trial_rows: list[dict] = []
    for label, dirs in arms.items():
        units = [read_unit(d) for d in dirs]
        check_unique_keys(label, units)
        summary["arms"][label] = arm_summary(label, units, floor, n_valid_target, z, n_boot, seed)
        included_by_arm[label] = _included(units)
        for k, u in enumerate(units):
            for r, inc in zip(u.rows, u.included, strict=True):
                out = {c: r.get(c) for c in POOL_COLUMNS}
                out.update({c: v for c, v in r.items() if c.startswith(POOL_COLUMN_PREFIXES)})
                out.update(
                    arm=label,
                    unit=k,
                    unit_dir=str(u.ct_dir),
                    included=inc,
                    beyond_target=not inc,
                )
                trial_rows.append(out)
    labels = list(arms)
    summary["mcnemar"] = [
        mcnemar(a, included_by_arm[a], b, included_by_arm[b])
        for a, b in itertools.combinations(labels, 2)
    ]
    summary["d3_s31b"] = d3_s31b_block(included_by_arm, [load_extra_d3(p) for p in extra_d3])
    return summary, trial_rows


def write_outputs(
    summary: dict, trial_rows: Sequence[Mapping], out_dir: Path
) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "pool_summary.json"
    json_path.write_text(
        json.dumps(ct._strict(summary), indent=2, default=ct._json_default, allow_nan=False) + "\n"
    )
    csv_path = out_dir / "pool_trials.csv"
    fields = list(POOL_COLUMNS)
    for row in trial_rows:
        fields.extend(k for k in row if k not in fields)
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(trial_rows)
    return json_path, csv_path


def _q(values) -> str:
    """A p50/p95/max triple, rounded for the report (the JSON keeps full precision)."""
    return "n/a" if values is None else "/".join(ct._fmt(v, ".2f") for v in values)


def report(summary: Mapping) -> str:
    lines = [
        f"catching_pool: floor {summary['floor']:g} · n_valid target {summary['n_valid_target']} · "
        f"z {summary['z']:g}"
    ]
    for label, arm in summary["arms"].items():
        tr = arm["truth"]
        lo, hi = tr["wilson95"]
        lines.append(f"── arm {label}")
        for u in arm["units"]:
            lines.append(
                f"  unit {u['dir']} seeds {u['seeds']}: {u['n_total']}/{u['n_rows']} included · "
                f"valid {u['n_valid']} · successes {u['successes']}"
                + (f" · beyond_target {u['beyond_target']}" if u["beyond_target"] else "")
            )
        lines.append("  " + ct.validity_line(arm["validity"]))
        lines.append(f"  beyond_target (excluded) {arm['validity']['beyond_target']}")
        lines.append(
            f"  truth success {tr['successes']}/{tr['n']} valid (Wilson 95 % "
            f"[{ct._fmt(lo)}, {ct._fmt(hi)}]); supervisor × truth "
            f"{tr['confusion_supervisor_vs_truth']}"
        )
        lines.append("  " + ct.verdict_line(tr))
        d3 = arm["d3"]
        lines.append(
            f"  D-3 (covariate, p50/p95/max ms): {d3['n_with_clock_covariate']} trials · "
            f"|δ(t_commit)| {_q(d3['delta_commit_ms_abs_p50_p95_max'])} · |δ(t_c)| "
            f"{_q(d3['delta_tc_ms_abs_p50_p95_max'])} · δ_max {_q(d3['delta_max_ms_p50_p95_max'])}"
        )
        lines.append("  " + ct.tick_line(arm["tick_overrun"]))
        lines.append("  " + ct.streak_line(arm["ref_saturated_streak"]))
        lines.append("  " + ct.b3_line(arm["g7b3"]))
        lines.append(f"  time alignment per unit: {arm['time_alignment']}")
        lines.append("  " + ct.rtf_line(arm["rtf"]))
        lines.extend("  " + line for line in ct.g8b_lines(arm["g8b"]))
        lines.append("  " + ct.c2_line(arm["c2"]))
        if arm["gate_map"] is not None:
            gm = arm["gate_map"]
            lines.append(
                f"  gate map: {gm['open']}/{gm['verdicted']} open · truth whole "
                f"{gm['truth_whole']['successes']}/{gm['truth_whole']['n']} · open "
                f"{gm['truth_open']['successes']}/{gm['truth_open']['n']}"
            )
    for m in summary["mcnemar"]:
        lines.append(
            f"McNemar {m['a']} vs {m['b']}: {m['n_pairs']} pairs · {m['a']} only "
            f"{m['a_success_b_fail']} · {m['b']} only {m['a_fail_b_success']} · exact p "
            f"{m['p_exact_two_sided']:.4g}"
            + (
                f" · {m['valid_rows_without_seed']} valid rows without a seed not paired"
                if m["valid_rows_without_seed"]
                else ""
            )
        )
    s = summary["d3_s31b"]
    lines.append(
        f"D-3 S3.1b: {s['total']} trials with a clock covariate (arms {s['per_arm']} + extra "
        f"{s['extra_paired']}) vs ≥ {s['requirement_n']} → {'met' if s['met'] else 'NOT met'}"
    )
    for e in s["extra_sources"]:
        lines.append(
            f"  extra {e['source']}: paired {e['paired']} · δ_max p50/p95/max "
            f"{_q(e['delta_max_ms_p50_p95_max'])} ms (not pooled)"
        )
    return "\n".join(lines)


def parse_arms(values: Sequence[Sequence[str]]) -> dict[str, list[Path]]:
    arms: dict[str, list[Path]] = {}
    for value in values:
        if len(value) < 2:
            raise SystemExit(f"--arm {' '.join(value)}: needs LABEL and at least one CT_DIR")
        label, *dirs = value
        if label in arms:
            raise SystemExit(f"--arm {label} given twice")
        arms[label] = [Path(d) for d in dirs]
    return arms


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument(
        "--arm",
        nargs="+",
        action="append",
        required=True,
        metavar=("LABEL", "CT_DIR"),
        help="an arm: its label and its catching_trials output dirs IN THE PRE-DECLARED ORDER "
        "(top-up units last). Repeat per arm",
    )
    ap.add_argument("--floor", type=float, required=True, help="G8-D floor (D-S8-3)")
    ap.add_argument(
        "--n-valid-target",
        type=int,
        default=DEFAULT_N_VALID_TARGET,
        help=f"valid trials per arm; later trials are beyond_target (default {DEFAULT_N_VALID_TARGET})",
    )
    ap.add_argument(
        "--z", type=float, default=1.96, help="Wilson z (1.96: 97.5 %% one-sided lower bound)"
    )
    ap.add_argument(
        "--out",
        type=Path,
        default=Path("catching_pool"),
        help="output dir (default ./catching_pool)",
    )
    ap.add_argument("--n-boot", type=int, default=ct.DEFAULT_N_BOOT)
    ap.add_argument("--seed", type=int, default=ct.DEFAULT_SEED)
    ap.add_argument(
        "--extra-d3",
        type=Path,
        nargs="*",
        default=[],
        metavar="SUMMARY_JSON",
        help="other sessions' catching_trials_summary.json: their d3 block is listed and their "
        "d3.paired summed into the S3.1b count (D-S8-16 ⑤)",
    )
    args = ap.parse_args(argv)
    summary, trial_rows = pool(
        parse_arms(args.arm),
        args.floor,
        args.n_valid_target,
        args.z,
        args.n_boot,
        args.seed,
        args.extra_d3,
    )
    json_path, csv_path = write_outputs(summary, trial_rows, args.out)
    print(report(summary))
    print(f"\n-> {json_path}\n-> {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
