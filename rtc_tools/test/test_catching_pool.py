"""catching_pool (dynamic_catching S8-E) — pooling units and arms, with hand-built inputs.

Every unit here is a hand-written ``catching_trials`` output dir (the CSV and
the summary JSON the pool reads), so each count the pool reports can be
checked against a number written down in the test: the D-S8-16 ①b truncation
(target reached mid-unit, ``beyond_target``, ``INSUFFICIENT_N``), the S8-E pass
line (84/200 PASS vs 83/200 FAIL at floor 0.35), the exact McNemar over
``(seed, idx)`` pairs, and the D-3 S3.1b count with ``--extra-d3``.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import pytest

from rtc_tools.analysis import catching_pool as cp, catching_trials as ct

FIELDS = (
    "idx",
    "kind",
    "supervisor",
    "accepted",
    "seed",
    "invalid_reason",
    "truth_success",
    "delta_commit_ms",
    "delta_tc_ms",
    "delta_max_ms",
    "ref_saturated_max_streak",
)


def _row(idx, seed, success=False, reason="", delta=1.0, **extra):
    row = {
        "idx": idx,
        "kind": "s35b",
        "supervisor": "CAUGHT" if success else "MISSED",
        "accepted": reason != "srv_refused",
        "seed": seed,
        "invalid_reason": reason,
        "truth_success": success,
        "delta_commit_ms": -delta,
        "delta_tc_ms": delta,
        "delta_max_ms": 2 * delta,
        "ref_saturated_max_streak": 0,
    }
    row.update(extra)
    return row


def _unit(path: Path, rows, lane=True) -> Path:
    path.mkdir(parents=True)
    fields = list(FIELDS) + [k for k in rows[0] if k not in FIELDS]
    with (path / cp.TRIALS_CSV).open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "validity": {"lane_rules_evaluated": lane},
        "seeds": sorted({r["seed"] for r in rows}),
    }
    (path / cp.SUMMARY_JSON).write_text(json.dumps(summary))
    return path


def test_parse_cell_round_trips_the_csv_writer():
    assert cp.parse_cell("idx", "7") == 7
    assert cp.parse_cell("seed", "") is None
    assert cp.parse_cell("truth_success", "True") is True
    assert math.isnan(cp.parse_cell("delta_tc_ms", "nan"))
    assert cp.parse_cell("invalid_reason", "lane_drop") == "lane_drop"
    assert cp.parse_cell("supervisor", "1") == "1"  # a string column stays a string


def test_truncation_stops_at_the_target_th_valid_trial_mid_unit(tmp_path):
    # Unit 1: idx 1 invalid → 4 valid (successes idx 0, 2, 3). Unit 2: idx 0
    # invalid, then the 5th and 6th valid are its idx 1 and 2 (idx 2 a
    # success); idx 3 and 4 come after the 6th valid.
    u1 = _unit(
        tmp_path / "u1",
        [
            _row(i, 601, success=i in (0, 2, 3), reason="lane_drop" if i == 1 else "")
            for i in range(5)
        ],
    )
    u2 = _unit(
        tmp_path / "u2",
        [_row(i, 602, success=i == 2, reason="sim_stall" if i == 0 else "") for i in range(5)],
    )
    summary, rows = cp.pool({"a": [u1, u2]}, floor=0.1, n_valid_target=6, n_boot=50)
    arm = summary["arms"]["a"]
    v = arm["validity"]
    assert (v["n_total"], v["n_valid"], v["beyond_target"]) == (8, 6, 2)
    assert v["n_invalid"]["lane_drop"] == 1 and v["n_invalid"]["sim_stall"] == 1
    assert arm["truth"]["successes"] == 4 and arm["truth"]["n"] == 6
    assert arm["truth"]["itt"]["n"] == 8
    assert [u["beyond_target"] for u in arm["units"]] == [0, 2]
    assert [u["seeds"] for u in arm["units"]] == [[601], [602]]
    beyond = [(r["unit_dir"], r["idx"]) for r in rows if r["beyond_target"]]
    assert beyond == [(str(u2), 3), (str(u2), 4)]
    # The given order decides: the other way round, unit 1's idx 3 (a
    # success) is the one left beyond the target.
    summary, rows = cp.pool({"a": [u2, u1]}, floor=0.1, n_valid_target=6, n_boot=50)
    assert summary["arms"]["a"]["truth"]["successes"] == 3
    beyond = [(r["unit_dir"], r["idx"]) for r in rows if r["beyond_target"]]
    assert beyond == [(str(u1), 3), (str(u1), 4)]


def test_short_arm_is_insufficient_n(tmp_path):
    u = _unit(tmp_path / "u", [_row(i, 601, success=True) for i in range(5)])
    summary, _ = cp.pool({"a": [u]}, floor=0.1, n_valid_target=10, n_boot=50)
    assert summary["arms"]["a"]["truth"]["verdict"] == "INSUFFICIENT_N(5 < 10)"
    assert summary["arms"]["a"]["validity"]["beyond_target"] == 0


@pytest.mark.parametrize(("k", "verdict", "lower"), [(84, "PASS", 0.3537), (83, "FAIL", 0.3489)])
def test_the_s8e_pass_line(tmp_path, k, verdict, lower):
    # 203 trials: 3 invalid mid-unit (out of n, in ITT), the first k valid ones
    # successes; then 5 more successes after the 200th valid (beyond_target).
    invalid = {10, 50, 150}
    rows, n_valid = [], 0
    for i in range(208):
        if i in invalid:
            rows.append(_row(i, 601, reason="controller_silent"))
            continue
        rows.append(_row(i, 601, success=n_valid < k or n_valid >= 200))
        n_valid += 1
    u = _unit(tmp_path / "u", rows)
    summary, _ = cp.pool({"a": [u]}, floor=0.35, n_valid_target=200, n_boot=50)
    tr = summary["arms"]["a"]["truth"]
    assert (tr["successes"], tr["n"]) == (k, 200)
    assert tr["lower_975"] == pytest.approx(lower, abs=1e-4)
    assert tr["verdict"] == verdict
    assert tr["itt"]["n"] == 203 and tr["itt"]["successes"] == k
    assert summary["arms"]["a"]["validity"]["beyond_target"] == 5


def test_mcnemar_pairs_on_seed_and_idx_valid_in_both(tmp_path):
    # 10 shared (seed, idx): 7 A-only successes, 1 B-only, 1 both, 1 neither.
    # Plus one pair invalid in B (dropped) and one idx only A has (unpaired).
    a_success = [True] * 7 + [False, True, False]
    b_success = [False] * 7 + [True, True, False]
    a = [_row(i, 601, success=s) for i, s in enumerate(a_success)]
    b = [_row(i, 601, success=s) for i, s in enumerate(b_success)]
    a.append(_row(10, 601, success=True))
    b.append(_row(10, 601, reason="lane_drop"))
    a.append(_row(11, 601, success=True))
    ua, ub = _unit(tmp_path / "a", a), _unit(tmp_path / "b", b)
    summary, _ = cp.pool({"A": [ua], "B": [ub]}, floor=0.1, n_valid_target=12, n_boot=50)
    (m,) = summary["mcnemar"]
    assert (m["a"], m["b"], m["n_pairs"]) == ("A", "B", 10)
    assert (m["a_success_b_fail"], m["a_fail_b_success"]) == (7, 1)
    assert (m["both_success"], m["both_fail"], m["discordant"]) == (1, 1, 8)
    assert m["p_exact_two_sided"] == pytest.approx(18 / 256)


def test_a_seed_idx_twice_in_one_arm_is_refused(tmp_path):
    u1 = _unit(tmp_path / "u1", [_row(0, 601)])
    u2 = _unit(tmp_path / "u2", [_row(0, 601)])
    with pytest.raises(SystemExit, match="re-run unit"):
        cp.pool({"a": [u1, u2]}, floor=0.1, n_boot=50)


def test_a_unit_from_before_the_validity_rules_is_refused(tmp_path):
    u = _unit(tmp_path / "u", [_row(0, 601)])
    text = (u / cp.TRIALS_CSV).read_text().replace("invalid_reason", "something_else")
    (u / cp.TRIALS_CSV).write_text(text)
    with pytest.raises(SystemExit, match="no invalid_reason column"):
        cp.read_unit(u)


def test_d3_s31b_is_judged_on_the_arms_and_lists_the_extra_summaries(tmp_path):
    """D-S8-16 ⑤: S8-E's own trials decide S3.1b. An --extra-d3 summary's
    d3.paired counts another population (every accepted trial with a lane
    launch) — listed with that definition, never added: 100 arm trials + 150
    extra is NOT met."""
    ua = _unit(tmp_path / "a", [_row(i, 601, delta=0.5 + i) for i in range(60)])
    # One trial without a clock covariate does not count.
    rows_b = [_row(i, 601) for i in range(40)] + [_row(40, 601, delta=math.nan)]
    ub = _unit(tmp_path / "b", rows_b)
    extra = tmp_path / "tuning_summary.json"
    extra.write_text(json.dumps({"d3": {"paired": 150, "delta_max_ms_p50_p95_max": [4, 10, 16]}}))
    summary, _ = cp.pool(
        {"A": [ua], "B": [ub]}, floor=0.1, n_valid_target=100, n_boot=50, extra_d3=[extra]
    )
    s = summary["d3_s31b"]
    assert s["per_arm"] == {"A": 60, "B": 40}
    assert (s["total"], s["met"]) == (100, False)
    assert s["extra_sources"][0]["paired"] == 150
    assert "never added" in s["extra_sources"][0]["paired_definition"]
    assert s["extra_sources"][0]["delta_max_ms_p50_p95_max"] == [4, 10, 16]
    assert "cannot be pooled" in s["note"]
    assert summary["arms"]["A"]["d3"]["delta_tc_ms_abs_p50_p95_max"][2] == pytest.approx(59.5)
    uc = _unit(tmp_path / "c", [_row(i, 602) for i in range(100)])
    summary, _ = cp.pool(
        {"A": [ua], "B": [ub], "C": [uc]}, floor=0.1, n_valid_target=100, n_boot=50
    )
    assert (summary["d3_s31b"]["total"], summary["d3_s31b"]["met"]) == (200, True)


def test_gate_map_block_only_when_rows_carry_map_open(tmp_path):
    rows = [_row(i, 701, success=i < 3, map_open=i < 4) for i in range(8)]
    u = _unit(tmp_path / "u", rows)
    summary, _ = cp.pool({"leap": [u]}, floor=0.1, n_valid_target=8, n_boot=50)
    gm = summary["arms"]["leap"]["gate_map"]
    assert (gm["verdicted"], gm["open"]) == (8, 4)
    assert gm["truth_open"]["successes"] == 3 and gm["truth_whole"]["n"] == 8
    u2 = _unit(tmp_path / "u2", [_row(i, 601) for i in range(3)])
    summary, _ = cp.pool({"p1b": [u2]}, floor=0.1, n_valid_target=3, n_boot=50)
    assert summary["arms"]["p1b"]["gate_map"] is None
    assert summary["arms"]["p1b"]["tick_overrun"].startswith("NOT_EVALUATED")


def test_cli_writes_strict_json_and_the_trial_table(tmp_path, capsys):
    ua = _unit(tmp_path / "a", [_row(i, 601, success=i % 2 == 0) for i in range(6)])
    ub = _unit(tmp_path / "b", [_row(i, 601, success=i % 3 == 0) for i in range(6)], lane=False)
    out = tmp_path / "out"
    rc = cp.main(
        ["--arm", "tennis", str(ua), "--arm", "beanbag", str(ub)]
        + ["--floor", "0.35", "--n-valid-target", "6", "--n-boot", "50", "--out", str(out)]
    )
    assert rc == 0

    def refuse(name):
        raise ValueError(name)

    summary = json.loads((out / "pool_summary.json").read_text(), parse_constant=refuse)
    assert set(summary["arms"]) == {"tennis", "beanbag"}
    assert summary["arms"]["beanbag"]["validity"]["lane_rules_evaluated"] is False
    with (out / "pool_trials.csv").open() as handle:
        table = list(csv.DictReader(handle))
    assert len(table) == 12 and table[0]["arm"] == "tennis"
    text = capsys.readouterr().out
    assert "McNemar tennis vs beanbag" in text and "lane rules NOT evaluated" in text


def test_cli_rejects_an_arm_without_dirs(tmp_path):
    with pytest.raises(SystemExit):
        cp.main(["--arm", "tennis", "--floor", "0.35", "--out", str(tmp_path)])


def test_pool_reuses_the_session_statistics():
    """P5: one implementation of the verdict statistics, not a copy."""
    assert cp.ct.truth_block is ct.truth_block
    assert cp.ct.impulse_correlation is ct.impulse_correlation


def test_g8b_and_c2_are_recomputed_from_the_pooled_trial_columns(tmp_path):
    """Two units with different per-trial sample counts: the pooled mean NEES is
    Σ NEES / Σ n over every trial of both (not a mean of unit means), and the
    C2 count spans both units."""

    def nees_row(i, seed, n, mean, a=0.01):
        return _row(
            i,
            seed,
            nees_h100ms_n=n,
            nees_h100ms_nan=0,
            nees_h100ms_mean=mean,
            nees_h100ms_cov95=1.0,
            err_h100ms_x=0.0,
            err_h100ms_y=0.0,
            err_h100ms_z=0.0,
            c2_A_x=a,
            c2_A_y=0.0,
            c2_A_z=0.0,
            c2_B_x=0.0,
            c2_B_y=a,
            c2_B_z=0.0,
            c2_join="exact",
        )

    u1 = _unit(tmp_path / "u1", [nees_row(i, 601, 10, 2.0) for i in range(4)])
    u2 = _unit(tmp_path / "u2", [nees_row(i, 602, 40, 4.0) for i in range(2)])
    summary, rows = cp.pool({"a": [u1, u2]}, floor=0.1, n_valid_target=6, n_boot=100)
    g = summary["arms"]["a"]["g8b"]["horizons"]["100"]
    assert g["n_samples"] == 4 * 10 + 2 * 40
    assert g["mean_nees"] == pytest.approx((4 * 10 * 2.0 + 2 * 40 * 4.0) / 120)
    c2 = summary["arms"]["a"]["c2"]
    assert c2["n"] == 6 and c2["n_exact"] == 6
    assert c2["independence"] == "NOT_EVALUATED(n < 100)"
    assert "nees_h100ms_mean" in rows[0] and "c2_A_x" in rows[0]
