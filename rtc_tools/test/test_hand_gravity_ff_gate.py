"""Hand gravity-feedforward A/B/A gate judge.

Every threshold is pinned with a boundary PAIR — a value just inside and one
just outside — on synthetic ``joint_ff.csv`` files whose verdict is known by
construction, so a green here is evidence about the judge, not the fixture.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import pytest

from rtc_tools.analysis.hand_gravity_ff_gate import (
    DEFAULT_BIG,
    DEFAULT_EXCLUDE,
    DEFAULT_LOOP_DISCRIMINATOR,
    DEFAULT_SMALL,
    FAIL,
    NOT_EVALUATED,
    PASS,
    UNDECIDED,
    Check,
    Thresholds,
    classify,
    find_joint_ff_csv,
    joint_names_of,
    judge,
    load_joint_ff_csv,
    main,
    overall,
    render_markdown,
    split_segments,
    summarise,
    window_rows,
)

# The header, exactly as the producer writes it for the shipped hand.
JOINTS = [
    "cmc_01_joint",
    "cmc_02_joint",
    "thumb_mcp_joint",
    "thumb_pip_link02_joint",
    "index_mcp_rotate_joint",
    "index_mcp_joint",
    "index_pip_joint",
    "middle_mcp_joint",
    "middle_pip_joint",
    "ring_mcp_joint",
]
FIXED = [
    "t_relative_s",
    "tick",
    "estop",
    "readable",
    "seeded",
    "traj_active",
    "ff_enabled",
    "ff_finite",
    "model_fresh",
    "held",
    "held_ticks",
    "singular",
    "closure_error",
    "clamped_mask",
    "num_joints",
]
HEADER = FIXED + [f"{p}{j}" for p in ("q_", "q_ref_", "g_", "tau_ff_") for j in JOINTS]

TH = Thresholds()
DT = 0.01
BIG = "cmc_01_joint"
SMALL = "thumb_mcp_joint"

# Errors that put every judged joint comfortably inside its thresholds.
GOOD_ON = 0.0005
GOOD_OFF = 0.02


# ── fixture builders ─────────────────────────────────────────────────────────


def row(t: float, tick: int, ff: int, err: dict[str, float] | float, **flags) -> dict:
    """One tick. `err` is |q_ref - q| per joint (a float applies to all)."""
    base = {
        "t_relative_s": round(t, 6),
        "tick": tick,
        "estop": 0,
        "readable": 1,
        "seeded": 1,
        "traj_active": 0,
        "ff_enabled": ff,
        "ff_finite": ff,
        "model_fresh": 1,
        "held": 0,
        "held_ticks": 0,
        "singular": 0,
        "closure_error": 0.0,
        "clamped_mask": 0,
        "num_joints": len(JOINTS),
    }
    base.update(flags)
    for j in JOINTS:
        e = err[j] if isinstance(err, dict) else err
        base[f"q_{j}"] = 0.5 - e
        base[f"q_ref_{j}"] = 0.5
        base[f"g_{j}"] = 0.02
        base[f"tau_ff_{j}"] = 0.02 * ff
    return base


def segment(ff: int, span_s: float, err, t0: float = 0.0, tick0: int = 0, per_row=None):
    """Rows from t0 to t0 + span_s inclusive (so the segment's span is exact)."""
    n = round(span_s / DT)
    rows = []
    for k in range(n + 1):
        extra = per_row(k * DT) if per_row else {}
        e = err(k * DT) if callable(err) else err
        rows.append(row(t0 + k * DT, tick0 + k, ff, e, **extra))
    return rows


def aba(on_err=GOOD_ON, off_err=GOOD_OFF, spans=(12.0, 12.0, 12.0), per_row=None):
    """A/B/A: on, off, on. `on_err` / `off_err` may be a float, a per-joint dict,
    or a function of the time since the segment started."""
    rows: list[dict] = []
    t0, tick0 = 0.0, 0
    for ff, span in zip((1, 0, 1), spans, strict=True):
        rows += segment(ff, span, on_err if ff else off_err, t0, tick0, per_row)
        t0, tick0 = rows[-1]["t_relative_s"] + DT, rows[-1]["tick"] + 1
    return rows


def write_csv(path: Path, rows: list[dict], header: list[str] = HEADER) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(rows)
    return path


def run_judge(tmp_path, rows, th=TH, header: list[str] = HEADER, **kw):
    table = load_joint_ff_csv(write_csv(tmp_path / "joint_ff.csv", rows, header))
    classes = classify(table.joints, DEFAULT_BIG, DEFAULT_SMALL, DEFAULT_EXCLUDE)
    return by_key(judge(summarise(table, th), classes, th, **kw))


def by_key(checks: list[Check]) -> dict[str, Check]:
    return {c.key: c for c in checks}


def per_joint(value: float, joint: str, others: float) -> dict[str, float]:
    return {j: (value if j == joint else others) for j in JOINTS}


# ── defaults ─────────────────────────────────────────────────────────────────


def test_default_lists_cover_the_hand_minus_the_excluded_joint():
    assert set(DEFAULT_BIG) | set(DEFAULT_SMALL) == set(JOINTS) - set(DEFAULT_EXCLUDE)
    assert not set(DEFAULT_BIG) & set(DEFAULT_SMALL)
    assert DEFAULT_EXCLUDE == ("index_mcp_rotate_joint",)
    assert DEFAULT_LOOP_DISCRIMINATOR in DEFAULT_BIG


def test_thresholds_are_the_pre_registered_values():
    assert (
        Thresholds(
            min_segment_s=10.0,
            window_s=3.0,
            big_off_min_rad=0.010,
            big_on_max_rad=0.001,
            big_ratio_max=0.05,
            small_off_min_rad=0.003,
            small_on_max_rad=0.001,
        )
        == TH
    )


def test_joint_names_come_from_the_tau_ff_block():
    assert joint_names_of(HEADER) == JOINTS
    # `q_` alone would count the reference block too
    assert len([c for c in HEADER if c.startswith("q_")]) == 2 * len(JOINTS)


# ── segments and windows ─────────────────────────────────────────────────────


def test_rows_split_into_contiguous_segments_by_ff_enabled():
    segs = split_segments(aba(spans=(1.0, 2.0, 0.5)))
    assert [(s.ff_on, round(s.span_s, 6)) for s in segs] == [
        (True, 1.0),
        (False, 2.0),
        (True, 0.5),
    ]
    assert segs[1].t_start == pytest.approx(1.0 + DT)
    assert sum(len(s.rows) for s in segs) == len(aba(spans=(1.0, 2.0, 0.5)))


def test_min_segment_boundary(tmp_path):
    inside = run_judge(tmp_path, aba(spans=(10.0, 10.0, 10.0)))
    assert inside["a.segments"].verdict == PASS
    assert inside[f"{BIG}.on_abs"].verdict == PASS
    short_off = run_judge(tmp_path, aba(spans=(12.0, 9.99, 12.0)))
    assert short_off["a.segments"].verdict == UNDECIDED
    assert "1 shorter than the minimum" in short_off["a.segments"].value
    assert short_off[f"{BIG}.on_abs"].verdict == NOT_EVALUATED
    assert short_off[f"{BIG}.on_ratio"].verdict == NOT_EVALUATED
    assert short_off[f"{SMALL}.on_abs"].verdict == NOT_EVALUATED
    assert overall(short_off.values()) == UNDECIDED


def test_a_run_with_one_state_only_is_undecided(tmp_path):
    only_on = segment(1, 30.0, GOOD_ON)
    checks = run_judge(tmp_path, only_on)
    assert checks["a.segments"].verdict == UNDECIDED
    assert checks["g.ff_finite"].verdict == PASS  # the on window exists and is judged
    assert overall(checks.values()) == UNDECIDED


def test_window_is_the_last_window_s_of_the_segment():
    seg = split_segments(segment(1, 12.0, GOOD_ON))[0]
    win = window_rows(seg, 3.0)
    assert win[0]["t_relative_s"] == pytest.approx(9.0)  # inclusive at t_end - window_s
    assert win[-1]["t_relative_s"] == pytest.approx(12.0)
    assert len(win) == round(3.0 / DT) + 1


def test_window_statistics_ignore_the_transient_before_it(tmp_path):
    """Error 0.1 rad until the last 3 s, then GOOD_ON: judged on the tail only."""

    def on_err(t_in_seg):
        return 0.1 if t_in_seg < 9.0 else GOOD_ON

    checks = run_judge(tmp_path, aba(on_err=on_err))
    assert checks[f"{BIG}.on_abs"].verdict == PASS
    assert f"on {GOOD_ON:.5f} rad" in checks[f"{BIG}.on_abs"].value


def test_window_keeps_readable_non_estop_rows_only():
    def per_row(t_in_seg):
        if t_in_seg >= 11.0:
            return {"estop": 1}
        if 10.0 <= t_in_seg < 10.5:
            return {"readable": 0}
        return {}

    seg = split_segments(segment(1, 12.0, GOOD_ON, per_row=per_row))[0]
    win = window_rows(seg, 3.0)
    assert all(r["estop"] == 0 and r["readable"] == 1 for r in win)
    # 9.0..12.0 is 301 rows; minus 101 E-STOP rows (11.0..12.0) and 50 unreadable
    assert len(win) == 301 - 101 - 50


def test_a_window_without_usable_rows_is_undecided_not_passed(tmp_path):
    rows = aba(per_row=lambda t: {"estop": 1} if t >= 8.0 else {})
    checks = run_judge(tmp_path, rows)
    assert checks[f"{BIG}.off_floor"].verdict == UNDECIDED
    assert "no usable rows in any off window" in checks[f"{BIG}.off_floor"].note
    assert checks[f"{BIG}.on_abs"].verdict == UNDECIDED
    assert checks["g.ff_finite"].verdict == NOT_EVALUATED
    assert overall(checks.values()) == UNDECIDED


def test_several_segments_judge_the_worst_on_and_the_smallest_off(tmp_path):
    # on 0.0005 | off 0.02 | on 0.0012 (bad) | off 0.009 (below the big premise) | on 0.0005
    rows: list[dict] = []
    t0, tick0 = 0.0, 0
    for ff, err in ((1, 0.0005), (0, 0.02), (1, 0.0012), (0, 0.009), (1, 0.0005)):
        rows += segment(ff, 12.0, err, t0, tick0)
        t0, tick0 = rows[-1]["t_relative_s"] + DT, rows[-1]["tick"] + 1
    checks = run_judge(tmp_path, rows)
    assert "on 5, off 5" not in checks["a.segments"].value
    assert checks["a.segments"].value.startswith("on 3, off 2 usable")
    # the smallest off mean (0.009) fails the big premise -> UNDECIDED, not the 0.02
    assert checks[f"{BIG}.off_floor"].verdict == UNDECIDED
    assert "off 0.00900 rad" in checks[f"{BIG}.off_floor"].value
    # the small joints' premise (0.003) holds at 0.009; their worst on is 0.0012
    assert checks[f"{SMALL}.off_floor"].verdict == PASS
    assert checks[f"{SMALL}.on_abs"].verdict == FAIL
    assert "on 0.00120 rad" in checks[f"{SMALL}.on_abs"].value


# ── big joints ───────────────────────────────────────────────────────────────


@pytest.mark.parametrize(("on_err", "verdict"), [(0.00099, PASS), (0.00101, FAIL)])
def test_big_on_abs_boundary(tmp_path, on_err, verdict):
    # off 0.03 keeps the ratio check inside (limit 0.0015 rad) so only on_abs moves
    rows = aba(on_err=per_joint(on_err, BIG, GOOD_ON), off_err=per_joint(0.03, BIG, GOOD_OFF))
    checks = run_judge(tmp_path, rows)
    assert checks[f"{BIG}.on_abs"].verdict == verdict
    assert checks[f"{BIG}.off_floor"].verdict == PASS
    assert checks[f"{BIG}.on_ratio"].verdict == PASS
    # the other joints are untouched
    assert checks["cmc_02_joint.on_abs"].verdict == PASS
    assert overall(checks.values()) == verdict


@pytest.mark.parametrize(("on_err", "verdict"), [(0.00059, PASS), (0.00061, FAIL)])
def test_big_on_ratio_boundary_with_on_abs_inside(tmp_path, on_err, verdict):
    """off 0.012 -> ratio limit 0.0006 rad, below the 0.001 absolute limit."""
    rows = aba(on_err=per_joint(on_err, BIG, GOOD_ON), off_err=per_joint(0.012, BIG, GOOD_OFF))
    checks = run_judge(tmp_path, rows)
    assert checks[f"{BIG}.on_abs"].verdict == PASS
    assert checks[f"{BIG}.on_ratio"].verdict == verdict
    assert "= 0.00060 rad here" in checks[f"{BIG}.on_ratio"].threshold


@pytest.mark.parametrize(("off_err", "verdict"), [(0.0101, PASS), (0.0099, UNDECIDED)])
def test_big_off_premise_boundary(tmp_path, off_err, verdict):
    checks = run_judge(tmp_path, aba(off_err=per_joint(off_err, BIG, GOOD_OFF)))
    assert checks[f"{BIG}.off_floor"].verdict == verdict
    assert checks[f"{BIG}.off_floor"].threshold == ">= 0.01 rad (big)"


def test_big_premise_failing_makes_the_joint_undecided_even_with_a_bad_on(tmp_path):
    """The gate as registered: no discrimination -> UNDECIDED for that joint."""
    rows = aba(on_err=per_joint(0.002, BIG, GOOD_ON), off_err=per_joint(0.005, BIG, GOOD_OFF))
    checks = run_judge(tmp_path, rows)
    assert checks[f"{BIG}.off_floor"].verdict == UNDECIDED
    assert "cannot discriminate" in checks[f"{BIG}.off_floor"].note
    assert checks[f"{BIG}.on_abs"].verdict == UNDECIDED
    assert checks[f"{BIG}.on_ratio"].verdict == UNDECIDED
    assert "premise not met" in checks[f"{BIG}.on_abs"].note
    assert overall(checks.values()) == UNDECIDED


# ── small joints ─────────────────────────────────────────────────────────────


@pytest.mark.parametrize(("on_err", "verdict"), [(0.00099, PASS), (0.00101, FAIL)])
def test_small_on_abs_boundary(tmp_path, on_err, verdict):
    checks = run_judge(tmp_path, aba(on_err=per_joint(on_err, SMALL, GOOD_ON)))
    assert checks[f"{SMALL}.on_abs"].verdict == verdict
    assert checks[f"{SMALL}.on_abs"].threshold == "<= 0.001 rad (small)"
    assert f"{SMALL}.on_ratio" not in checks  # no ratio check for a small joint


@pytest.mark.parametrize(("off_err", "verdict"), [(0.0031, PASS), (0.0029, UNDECIDED)])
def test_small_off_premise_boundary(tmp_path, off_err, verdict):
    checks = run_judge(tmp_path, aba(off_err=per_joint(off_err, SMALL, GOOD_OFF)))
    assert checks[f"{SMALL}.off_floor"].verdict == verdict
    assert checks[f"{SMALL}.on_abs"].verdict == (PASS if verdict == PASS else UNDECIDED)


# ── excluded / unlisted / discriminator ──────────────────────────────────────


def test_excluded_joint_is_reported_never_judged(tmp_path):
    bad = per_joint(0.3, "index_mcp_rotate_joint", GOOD_ON)
    checks = run_judge(
        tmp_path, aba(on_err=bad, off_err=per_joint(0.0, "index_mcp_rotate_joint", GOOD_OFF))
    )
    info = checks["index_mcp_rotate_joint.info"]
    assert info.verdict == NOT_EVALUATED
    assert "excluded by --exclude" in info.note
    assert "on 0.30000 rad" in info.value
    assert not any(k.startswith("index_mcp_rotate_joint.on") for k in checks)
    assert overall(checks.values()) == PASS


def test_loop_discriminator_is_a_note_only(tmp_path):
    checks = run_judge(tmp_path, aba())
    for key in ("off_floor", "on_abs", "on_ratio"):
        assert "loop-consistency discriminator" in checks[f"ring_mcp_joint.{key}"].note
    assert "loop-consistency discriminator" not in checks[f"{BIG}.on_abs"].note
    none = run_judge(tmp_path, aba(), loop_discriminator=None)
    assert "loop-consistency discriminator" not in none["ring_mcp_joint.on_abs"].note
    # the verdicts are the same either way
    assert {k: c.verdict for k, c in checks.items()} == {k: c.verdict for k, c in none.items()}


def test_classify_matches_by_suffix_and_rejects_typos_and_overlaps():
    prefixed = [f"right_{j}" for j in JOINTS]
    classes = classify(prefixed, DEFAULT_BIG, DEFAULT_SMALL, DEFAULT_EXCLUDE)
    assert classes["right_cmc_01_joint"] == "big"
    assert classes["right_thumb_mcp_joint"] == "small"
    assert classes["right_index_mcp_rotate_joint"] == "excluded"
    unlisted = classify(JOINTS, ("cmc_01_joint",), (), ())
    assert unlisted["cmc_02_joint"] == "unlisted"
    with pytest.raises(ValueError, match="matches no joint"):
        classify(JOINTS, ("cmc_99_joint",), (), ())
    with pytest.raises(ValueError, match="both"):
        classify(JOINTS, ("cmc_01_joint",), ("cmc_01_joint",), ())


def test_unlisted_joint_is_info_only(tmp_path):
    table = load_joint_ff_csv(write_csv(tmp_path / "joint_ff.csv", aba()))
    classes = classify(table.joints, ("cmc_01_joint",), (), ())
    checks = by_key(judge(summarise(table, TH), classes, TH))
    assert checks["cmc_02_joint.info"].verdict == NOT_EVALUATED
    assert "in no joint list" in checks["cmc_02_joint.info"].note


# ── global checks ────────────────────────────────────────────────────────────


def test_ff_finite_must_hold_on_every_on_window_row(tmp_path):
    clean = run_judge(tmp_path, aba())
    assert clean["g.ff_finite"].verdict == PASS
    assert clean["g.ff_finite"].value.startswith("0 / 602 on-window rows")

    def one_bad(t_in_seg):
        return {"ff_finite": 0} if abs(t_in_seg - 11.5) < DT / 2 else {}

    one = run_judge(tmp_path, aba(per_row=one_bad))
    assert one["g.ff_finite"].verdict == FAIL
    assert one["g.ff_finite"].value.startswith("2 / 602")  # once per on segment
    # outside the window (t < 9 s) it is not judged
    early = run_judge(tmp_path, aba(per_row=lambda t: {"ff_finite": 0} if t < 1.0 else {}))
    assert early["g.ff_finite"].verdict == PASS


def test_off_rows_do_not_need_ff_finite(tmp_path):
    # the builder already writes ff_finite = 0 on off rows; make it explicit
    checks = run_judge(tmp_path, aba(per_row=lambda t: {}))
    assert all(r["ff_finite"] == 0 for r in aba() if r["ff_enabled"] == 0)
    assert checks["g.ff_finite"].verdict == PASS


def test_held_must_be_zero_on_every_window_row_on_or_off(tmp_path):
    clean = run_judge(tmp_path, aba())
    assert clean["g.held"].verdict == PASS
    assert clean["g.held"].value == "0 / 903 window rows held"

    def held_in_off(t_in_seg):
        return {"held": 1, "held_ticks": 7} if 10.0 <= t_in_seg < 10.1 else {}

    rows = aba()
    for r in rows:
        if r["ff_enabled"] == 0:
            t_in = r["t_relative_s"] - 12.0 - DT
            r.update(held_in_off(t_in))
    held = run_judge(tmp_path, rows)
    assert held["g.held"].verdict == FAIL
    assert held["g.held"].note == "held_ticks max 7"
    assert overall(held.values()) == FAIL


# The producer writes `mode` right after `ff_enabled`; the judge reads by name.
HEADER_WITH_MODE = (
    HEADER[: HEADER.index("ff_enabled") + 1] + ["mode"] + HEADER[HEADER.index("ff_enabled") + 1 :]
)


def test_a_file_without_mode_has_no_mode_check(tmp_path):
    assert "g.mode" not in run_judge(tmp_path, aba())


def test_window_rows_must_be_in_tracking_when_mode_is_logged(tmp_path):
    clean = run_judge(tmp_path, aba(per_row=lambda t: {"mode": 0}), header=HEADER_WITH_MODE)
    assert clean["g.mode"].verdict == PASS
    assert clean["g.mode"].value == "0 / 903 window rows not in tracking (mode != 0)"

    # A damping_feedforward stretch inside an on window: q_ref == q there, so
    # the per-joint error would pass trivially — the gate must say so instead.
    def damped_tail(t_in_seg):
        return {"mode": 2} if t_in_seg >= 11.0 else {"mode": 0}

    rows = aba(on_err=0.0, per_row=damped_tail)
    damped = run_judge(tmp_path, rows, header=HEADER_WITH_MODE)
    assert damped["g.mode"].verdict == FAIL
    assert damped["g.mode"].value.startswith("303 / 903")  # 1.0 s + 1 row x 3 segments
    assert "tracking runs only" in damped["g.mode"].note
    assert overall(damped.values()) == FAIL
    # outside the window it is not judged
    early = run_judge(
        tmp_path,
        aba(per_row=lambda t: {"mode": 1} if t < 1.0 else {"mode": 0}),
        header=HEADER_WITH_MODE,
    )
    assert early["g.mode"].verdict == PASS


def test_overall_precedence():
    assert overall([Check("a", PASS, "", "")]) == PASS
    assert overall([Check("a", PASS, "", ""), Check("b", NOT_EVALUATED, "", "")]) == PASS
    assert overall([Check("a", UNDECIDED, "", ""), Check("b", PASS, "", "")]) == UNDECIDED
    assert overall([Check("a", UNDECIDED, "", ""), Check("b", FAIL, "", "")]) == FAIL


# ── input resolution and CLI ─────────────────────────────────────────────────


def test_find_csv_in_a_session_dir(tmp_path):
    session = tmp_path / "260101_1200"
    one = write_csv(session / "controllers" / "hand_joint_controller" / "joint_ff.csv", aba())
    assert find_joint_ff_csv(session, None) == one
    assert find_joint_ff_csv(one, None) == one
    (tmp_path / "empty_dir").mkdir()
    with pytest.raises(ValueError, match="no joint_ff.csv"):
        find_joint_ff_csv(tmp_path / "empty_dir", None)
    two = write_csv(session / "controllers" / "other" / "joint_ff.csv", aba())
    with pytest.raises(ValueError, match="2 joint_ff.csv files"):
        find_joint_ff_csv(session, None)
    assert find_joint_ff_csv(session, Path("controllers/other/joint_ff.csv")) == two
    assert find_joint_ff_csv(session, two) == two
    with pytest.raises(ValueError, match="no such file"):
        find_joint_ff_csv(session, Path("nope.csv"))


def test_loader_rejects_the_wrong_inputs(tmp_path):
    other = tmp_path / "x.csv"
    other.write_text("t_relative_s,tick,actual_pos_a\n0,0,1\n")
    with pytest.raises(ValueError, match="missing columns"):
        load_joint_ff_csv(other)
    no_joints = tmp_path / "y.csv"
    no_joints.write_text(",".join(FIXED) + "\n" + ",".join("0" for _ in FIXED) + "\n")
    with pytest.raises(ValueError, match="no tau_ff_"):
        load_joint_ff_csv(no_joints)
    header_only = tmp_path / "z.csv"
    header_only.write_text(",".join(HEADER) + "\n")
    with pytest.raises(ValueError, match="no data rows"):
        load_joint_ff_csv(header_only)


def test_a_line_cut_mid_write_is_skipped(tmp_path):
    path = write_csv(tmp_path / "joint_ff.csv", aba())
    lines = path.read_text().splitlines()
    lines[-1] = ",".join(lines[-1].split(",")[:5])
    path.write_text("\n".join(lines) + "\n")
    table = load_joint_ff_csv(path)
    assert len(table.rows) == len(aba()) - 1


def test_main_exit_codes(tmp_path, capsys):
    session = tmp_path / "s"
    good = write_csv(session / "controllers" / "hand_joint_controller" / "joint_ff.csv", aba())
    assert main([str(session)]) == 0
    out = capsys.readouterr().out
    assert "## hand gravity FF verdict: **PASS**" in out
    assert f"csv    : {good}" in out
    assert "| `cmc_01_joint.on_ratio` | PASS |" in out
    assert "| `index_mcp_rotate_joint.info` | NOT_EVALUATED |" in out

    bad = write_csv(tmp_path / "bad" / "joint_ff.csv", aba(on_err=per_joint(0.002, BIG, GOOD_ON)))
    assert main([str(bad)]) == 1
    assert "**FAIL**" in capsys.readouterr().out

    short = write_csv(tmp_path / "short" / "joint_ff.csv", aba(spans=(12.0, 5.0, 12.0)))
    assert main([str(short)]) == 2
    assert "**UNDECIDED**" in capsys.readouterr().out

    assert main([str(tmp_path / "nowhere")]) == 3
    assert "error:" in capsys.readouterr().err
    write_csv(session / "controllers" / "other" / "joint_ff.csv", aba())
    assert main([str(session)]) == 3  # two files, no --csv
    assert main([str(session), "--csv", "controllers/other/joint_ff.csv"]) == 0
    capsys.readouterr()
    assert main([str(good), "--big", "cmc_99_joint"]) == 3
    assert "matches no joint" in capsys.readouterr().err


def test_main_thresholds_and_lists_are_flags(tmp_path):
    path = write_csv(tmp_path / "joint_ff.csv", aba(on_err=per_joint(0.002, BIG, GOOD_ON)))
    assert main([str(path)]) == 1
    assert main([str(path), "--big-on-max-rad", "0.003", "--big-ratio-max", "0.2"]) == 0
    # moving the failing joint to --exclude: info only
    big = ",".join(j for j in DEFAULT_BIG if j != BIG)
    assert main([str(path), "--big", big, "--exclude", f"{BIG},index_mcp_rotate_joint"]) == 0
    short = write_csv(tmp_path / "short.csv", aba(spans=(12.0, 5.0, 12.0)))
    assert main([str(short)]) == 2
    assert main([str(short), "--min-segment-s", "4"]) == 0


def test_json_out(tmp_path):
    path = write_csv(tmp_path / "joint_ff.csv", aba())
    out = tmp_path / "ff.json"
    assert main([str(path), "--json-out", str(out)]) == 0
    payload = json.loads(out.read_text())
    assert payload["verdict"] == PASS
    assert payload["joints"] == JOINTS
    assert payload["classes"]["ring_mcp_joint"] == "big"
    assert payload["thresholds"]["big_on_max_rad"] == 0.001
    assert [s["ff_on"] for s in payload["segments"]] == [True, False, True]
    assert payload["per_joint"][BIG]["on_mean"] == pytest.approx(GOOD_ON)
    assert payload["per_joint"][BIG]["ratio"] == pytest.approx(GOOD_ON / GOOD_OFF)
    keys = [c["key"] for c in payload["checks"]]
    assert keys[0] == "a.segments" and keys[-2:] == ["g.ff_finite", "g.held"]


def test_render_lists_segments_and_joints(tmp_path):
    table = load_joint_ff_csv(write_csv(tmp_path / "joint_ff.csv", aba(spans=(12.0, 5.0, 12.0))))
    classes = classify(table.joints, DEFAULT_BIG, DEFAULT_SMALL, DEFAULT_EXCLUDE)
    summary = summarise(table, TH)
    text = render_markdown(judge(summary, classes, TH), summary, classes, TH)
    assert "| 1 | off | " in text and "| no (< 10 s) |" in text
    assert "| `ring_mcp_joint` | big |" in text
    assert math.isnan(summary.joints[BIG].off_mean)  # the only off segment was ignored
