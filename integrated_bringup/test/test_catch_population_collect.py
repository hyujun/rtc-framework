"""catch_population_collect (#821, stage 2): the finished units as an arm's result files.

Every unit is planted in the test — a status, a ``conditions.txt`` and the
analyser's ``ct/catching_trials.csv`` — over chunk files the test writes. What is
pinned: ``sim_throws.csv`` is the units' rows behind ``arm,seed,chunk``; the catch
set is the thrown throws caught on at least one seed, each with the seeds, and
its meta lets a reader tell a throw that was not thrown from one that was missed;
a unit that threw another file (a copy from another result folder) is refused by
name. The analysis itself (``catching_tc_vector``) needs a recorded session and is
held by its command line and by which interpreter runs it.
"""

import csv
import json
import sys
from pathlib import Path

import pytest

from integrated_bringup import catch_population_collect as cpc, catching_tc_vector as tcv
from rtc_tools.analysis.catching_throw_list import read_throw_list_file, write_throw_list
from rtc_tools.utils.catch_population import ResultLayout

ARM = {"robot": "ur5e_p1b", "arm": "nlp_mpc_docking", "search": "nlp", "segment": "mpc_docking"}
SET_SHA = "5e" * 32  # the sha256 of the set the sample was drawn from
HEADER = ["idx", "kind", "throw_id", "seed", "invalid_reason", "plan_verdict", "truth_success"]


def _throw(throw_id):
    return {
        "throw_id": throw_id,
        "kind": "pass_point",
        "pos": [2.0, 0.0, 1.0 + throw_id / 100],
        "vel": [-3.0, 0.0, 4.0],
        "distance_m": 2.0,
        "elevation_deg": 45.0,
    }


def _folder(tmp_path, chunks=((1, 2, 3), (4, 5)), drawn=5, of=5):
    """A result folder with one arm's sample, in ``chunks`` of throw ids."""
    layout = ResultLayout(tmp_path)
    throws_dir = layout.throws_dir(ARM["arm"])
    throws_dir.mkdir(parents=True)
    meta = {
        "kind": "sample",
        "plan_set_sha256": SET_SHA,
        "plan_set_throws": of,
        "n_throws": drawn,
        "seed": 821,
        "max_throws": None if drawn == of else drawn,
        "strata_axes": ["distance_m", "elevation_deg"],
        "axes": {"discrete": ["distance_m"]},
        "design": {"grid": "…"},
    }
    every = [_throw(i) for ids in chunks for i in ids]
    write_throw_list(throws_dir / "sample.json", every, meta)
    for k, ids in enumerate(chunks):
        write_throw_list(
            throws_dir / f"chunk_{k:03d}.json",
            [_throw(i) for i in ids],
            {**meta, "chunk_index": k},
        )
    return layout


def _unit(layout, seed, chunk, rows, *, status="DONE", header=HEADER, conditions=None):
    """Plant a unit: ``rows`` are ``(throw_id, plan_verdict, truth_success[, invalid])``."""
    unit = layout.unit_dir(ARM["arm"], seed, chunk)
    (unit / "ct").mkdir(parents=True)
    (unit / "status").write_text(status + "\n")
    chunk_file = layout.throws_dir(ARM["arm"]) / f"{chunk}.json"
    cond = {
        "robot": ARM["robot"],
        "n": len(rows or ()),
        "seed": seed,
        "throws_file": chunk_file,
        "throws_file_sha256": cpc.sha256_file(chunk_file) if chunk_file.is_file() else "",
        "expect_mode": ARM["segment"],
        "expect_search": ARM["search"],
    }
    cond.update(conditions or {})
    (unit / "conditions.txt").write_text("".join(f"{k}: {v}\n" for k, v in cond.items()))
    if rows is not None:
        with open(unit / "ct" / "catching_trials.csv", "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for idx, row in enumerate(rows):
                throw_id, verdict, caught, *invalid = row
                full = {
                    "idx": idx,
                    "kind": "pass_point",
                    "throw_id": throw_id,
                    "seed": "",
                    "invalid_reason": invalid[0] if invalid else "",
                    "plan_verdict": verdict,
                    "truth_success": caught,
                }
                writer.writerow([full.get(name, "") for name in header])
        (unit / "ct" / "vec.csv").write_text("idx\n")
    return unit


def _collect(layout, thrown_set="segment"):
    return cpc.collect_arm(layout, thrown_set=thrown_set, **ARM)


def _sim_throws(layout):
    with open(layout.sim_throws(ARM["arm"]), newline="") as handle:
        return list(csv.DictReader(handle))


# ── the two files ────────────────────────────────────────────────────────────
def test_every_row_of_every_done_unit_stands_behind_arm_seed_and_chunk(tmp_path):
    layout = _folder(tmp_path)
    _unit(
        layout,
        8210,
        "chunk_000",
        [(1, "published", True), (2, "withheld", False), (3, "no_plan", False)],
    )
    _unit(layout, 8210, "chunk_001", [(4, "published", False), (5, "published", True)])
    result = _collect(layout)
    rows = _sim_throws(layout)
    assert list(rows[0])[:3] == ["arm", "seed", "chunk"]
    assert [(r["arm"], r["seed"], r["chunk"], r["throw_id"]) for r in rows] == [
        ("nlp_mpc_docking", "8210", "chunk_000", "1"),
        ("nlp_mpc_docking", "8210", "chunk_000", "2"),
        ("nlp_mpc_docking", "8210", "chunk_000", "3"),
        ("nlp_mpc_docking", "8210", "chunk_001", "4"),
        ("nlp_mpc_docking", "8210", "chunk_001", "5"),
    ]
    # The analyser's columns are carried as text, untouched.
    assert [r["plan_verdict"] for r in rows] == [
        "published",
        "withheld",
        "no_plan",
        "published",
        "published",
    ]
    assert [r["truth_success"] for r in rows] == ["True", "False", "False", "False", "True"]
    assert [r["invalid_reason"] for r in rows] == [""] * 5
    assert result.sim_throws == layout.sim_throws(ARM["arm"])
    assert result.units_done == [
        "nlp_mpc_docking/seed_8210/chunk_000",
        "nlp_mpc_docking/seed_8210/chunk_001",
    ]


def test_the_analysers_own_seed_column_does_not_hide_the_estimators(tmp_path):
    """catching_trials.csv has a `seed` column of its own — the throw series' seed, empty
    for a list. Two `seed` columns would make the estimator's unreadable by name."""
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    _collect(layout)
    with open(layout.sim_throws(ARM["arm"]), newline="") as handle:
        header = next(csv.reader(handle))
    assert header.count("seed") == 1 and header[1] == "seed" and "series_seed" in header
    assert _sim_throws(layout)[0]["seed"] == "8210"


def test_the_catch_set_is_the_throws_caught_on_at_least_one_seed(tmp_path):
    layout = _folder(tmp_path)
    _unit(
        layout,
        8210,
        "chunk_000",
        [(1, "published", True), (2, "published", False), (3, "withheld", False)],
    )
    _unit(layout, 8210, "chunk_001", [(4, "published", False), (5, "published", True)])
    # A second seed threw the first chunk only: throw 2 is caught there, throw 1 is not.
    _unit(
        layout,
        8211,
        "chunk_000",
        [(1, "published", False), (2, "published", True), (3, "no_plan", False)],
    )
    result = _collect(layout)
    listed = read_throw_list_file(layout.set_file(ARM["arm"], "catch"))
    assert [t["throw_id"] for t in listed.throws] == [1, 2, 5]
    assert [(t["caught_seeds"], t["thrown_seeds"]) for t in listed.throws] == [
        ([8210], [8210, 8211]),
        ([8211], [8210, 8211]),
        ([8210], [8210]),  # not thrown under 8211: its absence there is not a miss
    ]
    # The entry is the chunk file's, design columns and all.
    assert listed.throws[2]["pos"] == (2.0, 0.0, 1.05) and listed.throws[2]["distance_m"] == 2.0
    assert (result.thrown_ids, result.caught_ids) == (5, 3)
    assert result.catch_set == layout.set_file(ARM["arm"], "catch")


def test_the_meta_says_what_was_thrown_and_what_each_seed_did_with_it(tmp_path):
    layout = _folder(tmp_path, drawn=5, of=40)
    _unit(
        layout,
        8210,
        "chunk_000",
        [(1, "published", True), (2, "published", False), (3, "withheld", False)],
    )
    _unit(layout, 8210, "chunk_001", [(4, "published", False), (5, "published", True)])
    _unit(layout, 8211, "chunk_000", [(1, "no_plan", False), (2, "published", True), (3, "x", "")])
    _collect(layout)
    meta = read_throw_list_file(layout.set_file(ARM["arm"], "catch")).meta
    assert (meta["kind"], meta["set_name"], meta["tier"]) == (
        "catch_set",
        "throw_catch_set",
        "catch",
    )
    assert meta["tool"] == "integrated_bringup.catch_population_trials"
    assert meta["robot"] == "ur5e_p1b"
    assert meta["arm"] == {"name": "nlp_mpc_docking", "search": "nlp", "segment": "mpc_docking"}
    # Which set, and how much of it: the merge key across PCs is (its sha256, throw_id).
    assert meta["thrown_set"] == "segment"
    assert meta["source_set"] == {
        "set_name": "throw_segment_set",
        "sha256": SET_SHA,
        "n_throws": 40,
    }
    assert meta["merge_key"] == ["source_set.sha256", "throw_id"]
    sample = meta["sample"]
    assert (sample["n_throws"], sample["of"], sample["whole_set"]) == (5, 40, False)
    assert sample["sha256"] == cpc.sha256_file(layout.throws_dir(ARM["arm"]) / "sample.json")
    assert (sample["seed"], sample["strata_axes"]) == (821, ["distance_m", "elevation_deg"])
    # Per seed: what it threw, and of that how many invalid / published / caught.
    assert meta["chunks_total"] == 2
    assert meta["seeds"] == [
        {
            "seed": 8210,
            "chunks": ["chunk_000", "chunk_001"],
            "thrown": 5,
            "invalid": 0,
            "pair_published": 4,
            "caught": 2,
        },
        {
            "seed": 8211,
            "chunks": ["chunk_000"],
            "thrown": 3,
            "invalid": 0,
            "pair_published": 1,
            "caught": 1,
        },
    ]
    assert meta["counts"] == {"thrown": 5, "caught": 3}
    assert "caught on that seed" in meta["note"] and "not being here is not a miss" in meta["note"]
    # The design the throws were drawn in travels with the set, as with the other sets.
    assert meta["axes"] == {"discrete": ["distance_m"]} and meta["design"] == {"grid": "…"}


def test_a_chunk_thrown_in_part_is_collected_and_said(tmp_path):
    """A smoke's limit or a driver that ended early: the unit is DONE, its chunk is not
    thrown whole, and a throw left out of it is not a missed one."""
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])  # of 1, 2, 3
    _unit(layout, 8210, "chunk_001", [(4, "published", False), (5, "published", True)])
    result = _collect(layout)
    assert result.seeds[0].partial_chunks == ["chunk_000"]
    assert result.lines()[1].endswith("caught 2 (2 chunks, 1 partial)")
    seed = read_throw_list_file(layout.set_file(ARM["arm"], "catch")).meta["seeds"][0]
    assert seed["chunks"] == ["chunk_000", "chunk_001"]
    assert seed["partial_chunks"] == ["chunk_000"]
    assert "partial_chunks" in cpc.CATCH_SET_NOTE


def test_a_seed_thrown_whole_has_no_partial_chunks(tmp_path):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_001", [(4, "published", False), (5, "published", True)])
    result = _collect(layout)
    assert "partial" not in result.lines()[1]
    assert (
        "partial_chunks"
        not in read_throw_list_file(layout.set_file(ARM["arm"], "catch")).meta["seeds"][0]
    )


def test_the_catch_set_says_which_run_each_unit_was(tmp_path):
    # A catch is one run's outcome: the set carries each collected unit's date, the two
    # revisions and whether the viewer was on, as its conditions.txt recorded them.
    layout = _folder(tmp_path)
    said = {
        "date_start": "2026-10-11T20:34:55+09:00",
        "rtc_framework_rev": "b3d8dfd2",
        "ball_perception_rev": "9005aab",
        "viewer": "on",
    }
    _unit(layout, 8210, "chunk_000", [(1, "published", True)], conditions=said)
    _unit(layout, 8211, "chunk_000", [(1, "published", True)])  # an older record: no such lines
    _collect(layout)
    meta = read_throw_list_file(layout.set_file(ARM["arm"], "catch")).meta
    assert meta["runs"] == [
        {"seed": 8210, "chunk": "chunk_000", **said},
        {"seed": 8211, "chunk": "chunk_000", **dict.fromkeys(said)},
    ]
    assert "ONE run's outcome" in meta["note"]


def test_a_set_thrown_whole_says_so(tmp_path):
    layout = _folder(tmp_path, drawn=5, of=5)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    _collect(layout, thrown_set="search")
    meta = read_throw_list_file(layout.set_file(ARM["arm"], "catch")).meta
    assert meta["sample"]["whole_set"] is True and meta["thrown_set"] == "search"
    assert meta["source_set"]["set_name"] == "throw_search_set"


def test_an_invalid_throw_is_thrown_and_nothing_else(tmp_path):
    """The analyser's invalid_reason (a refused launch, a lost lane): such a throw is
    neither a published pair nor a catch, whatever its other cells say."""
    layout = _folder(tmp_path)
    _unit(
        layout,
        8210,
        "chunk_000",
        [(1, "published", True, "not_launched"), (2, "published", True), (3, "published", False)],
    )
    result = _collect(layout)
    seed = result.seeds[0]
    assert (seed.thrown, seed.invalid, seed.pair_published, seed.caught) == (3, 1, 2, 1)
    listed = read_throw_list_file(layout.set_file(ARM["arm"], "catch"))
    assert [t["throw_id"] for t in listed.throws] == [2]
    assert _sim_throws(layout)[0]["invalid_reason"] == "not_launched"


def test_units_that_are_not_done_are_left_out_and_said(tmp_path):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    _unit(layout, 8210, "chunk_001", [(4, "published", True)], status="FAIL:host_busy")
    # A failed attempt moved aside is evidence, not a unit.
    aside = layout.unit_dir(ARM["arm"], 8210, "chunk_001").with_name("chunk_001.fail1")
    aside.mkdir()
    (aside / "status").write_text("DONE\n")
    result = _collect(layout)
    assert result.units_done == ["nlp_mpc_docking/seed_8210/chunk_000"]
    assert result.units_not_done == ["nlp_mpc_docking/seed_8210/chunk_001 (FAIL:host_busy)"]
    assert [r["throw_id"] for r in _sim_throws(layout)] == ["1"]
    assert "not collected: nlp_mpc_docking/seed_8210/chunk_001 (FAIL:host_busy)" in "\n".join(
        result.lines()
    )


def test_with_no_done_unit_a_stale_result_is_removed(tmp_path):
    layout = _folder(tmp_path)
    for path in (layout.sim_throws(ARM["arm"]), layout.set_file(ARM["arm"], "catch")):
        path.write_text("stale\n")
    _unit(layout, 8210, "chunk_000", [(1, "published", True)], status="FAIL:startup")
    result = _collect(layout)
    assert result.sim_throws is None and result.catch_set is None
    assert not layout.sim_throws(ARM["arm"]).exists()
    assert not layout.set_file(ARM["arm"], "catch").exists()


@pytest.mark.parametrize("refusal", ["another file", "two conditions"])
def test_a_refused_collection_leaves_no_earlier_result(tmp_path, refusal):
    """An earlier collection's files would be read as this one's."""
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    _collect(layout)
    assert layout.sim_throws(ARM["arm"]).exists()
    if refusal == "another file":
        changed = {"throws_file_sha256": "ab" * 32}
    else:
        changed = {"profile_sha256": "other"}
    _unit(layout, 8210, "chunk_001", [(4, "published", True)], conditions=changed)
    with pytest.raises(cpc.CollectError):
        _collect(layout)
    assert not layout.sim_throws(ARM["arm"]).exists()
    assert not layout.set_file(ARM["arm"], "catch").exists()


def test_a_run_that_caught_nothing_writes_its_throws_and_no_set(tmp_path):
    """The throw-list format has no empty list; a stale set must not stand for this run."""
    layout = _folder(tmp_path)
    layout.set_file(ARM["arm"], "catch").write_text("stale\n")
    _unit(layout, 8210, "chunk_000", [(1, "published", False), (2, "withheld", False)])
    result = _collect(layout)
    assert result.catch_set is None and not layout.set_file(ARM["arm"], "catch").exists()
    assert len(_sim_throws(layout)) == 2 and result.seeds[0].caught == 0
    assert "not written (none caught)" in result.lines()[-1]


def test_units_of_two_analyser_versions_are_joined_on_the_union_of_columns(tmp_path):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    wider = [*HEADER, "pred_wake_mm"]
    _unit(layout, 8210, "chunk_001", [(4, "published", True)], header=wider)
    _collect(layout)
    rows = _sim_throws(layout)
    assert list(rows[0])[-1] == "pred_wake_mm" and rows[0]["pred_wake_mm"] == ""


# ── units that are not this folder's ─────────────────────────────────────────
def test_a_unit_that_threw_another_file_is_refused_by_name(tmp_path):
    """The merge of two PCs: a unit copied in must have thrown THIS folder's chunk."""
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    _unit(
        layout,
        8210,
        "chunk_001",
        [(4, "published", True)],
        conditions={"throws_file_sha256": "ab" * 32},
    )
    with pytest.raises(cpc.CollectError) as refused:
        _collect(layout)
    message = str(refused.value)
    assert "nlp_mpc_docking/seed_8210/chunk_001: it threw another file" in message
    assert "chunk_000" not in message.split("\n", 1)[1]
    # Nothing is written from a folder that holds such a unit.
    assert not layout.sim_throws(ARM["arm"]).exists()


@pytest.mark.parametrize(
    ("key", "values"),
    [
        ("profile_sha256", ("aa" * 32, "bb" * 32)),
        ("end_on_ball_low", ("1 margin_m=default grace_s=default", "")),
        ("host_watch", ("abort", "warn")),
    ],
)
def test_the_units_of_a_seed_thrown_under_two_conditions_are_refused(tmp_path, key, values):
    """A resumed run skips its DONE units: chunks thrown before and after a change of the
    estimator's profile or of the rules that end a throw must not become one seed."""
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)], conditions={key: values[0]})
    _unit(layout, 8210, "chunk_001", [(4, "published", True)], conditions={key: values[1]})
    with pytest.raises(cpc.CollectError) as refused:
        _collect(layout)
    message = str(refused.value)
    assert f"seed 8210, {key}" in message
    assert "chunk_000" in message and "chunk_001" in message
    assert not layout.sim_throws(ARM["arm"]).exists()


def test_two_seeds_may_differ_in_their_conditions(tmp_path):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)], conditions={"host_watch": "abort"})
    _unit(layout, 8211, "chunk_000", [(1, "published", True)], conditions={"host_watch": "warn"})
    assert not cpc.mixed_conditions(cpc.units_present(cpc.sim_root(layout), ARM["arm"]))


@pytest.mark.parametrize(
    ("conditions", "why"),
    [
        ({"seed": 9999}, "its conditions.txt says seed 9999, its directory seed_8210"),
        ({"robot": "iiwa7_leap"}, "it ran iiwa7_leap, this folder is ur5e_p1b"),
        ({"expect_search": "grid"}, "it ran the arm grid x mpc_docking"),
        ({"expect_mode": "mpc"}, "it ran the arm nlp x mpc"),
        ({"throws_file_sha256": ""}, "its conditions.txt records no throws_file_sha256"),
    ],
)
def test_a_unit_of_another_seed_robot_or_arm_is_refused(tmp_path, conditions, why):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)], conditions=conditions)
    with pytest.raises(cpc.CollectError, match=why):
        _collect(layout)


def test_a_unit_of_a_chunk_this_folder_does_not_have_is_refused(tmp_path):
    layout = _folder(tmp_path)
    _unit(
        layout,
        8210,
        "chunk_007",
        [(1, "published", True)],
        conditions={"throws_file_sha256": "ab" * 32},
    )
    with pytest.raises(cpc.CollectError, match="this folder has no chunk_007.json"):
        _collect(layout)


def test_a_failed_unit_is_not_asked_whose_it_is(tmp_path):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    _unit(
        layout,
        8210,
        "chunk_001",
        [(4, "published", True)],
        status="FAIL:trials rc=1",
        conditions={"throws_file_sha256": "ab" * 32},
    )
    assert _collect(layout).units_done == ["nlp_mpc_docking/seed_8210/chunk_000"]


def test_a_done_unit_without_its_analysis_is_refused(tmp_path):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", None)
    with pytest.raises(cpc.CollectError, match="DONE but has no ct/catching_trials.csv"):
        _collect(layout)


def test_a_row_of_a_throw_the_chunk_does_not_list_is_refused(tmp_path):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True), (99, "published", True)])
    with pytest.raises(cpc.CollectError, match="throw 99 is not in chunk_000.json"):
        _collect(layout)


def test_the_sample_must_be_drawn_from_the_set_the_configuration_says_is_thrown(tmp_path):
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    segment_set = layout.set_file(ARM["arm"], "segment")
    write_throw_list(segment_set, [_throw(1)], {"kind": "plan_set"})
    with pytest.raises(cpc.CollectError, match="was not drawn from this folder's"):
        _collect(layout)
    # Without that set in the folder (units merged elsewhere) there is nothing to hold.
    segment_set.unlink()
    assert _collect(layout).caught_ids == 1


def test_the_sample_drawn_from_this_folders_set_is_collected(tmp_path):
    layout = ResultLayout(tmp_path)
    layout.arm(ARM["arm"]).mkdir()
    segment_set = layout.set_file(ARM["arm"], "segment")
    write_throw_list(segment_set, [_throw(i) for i in (1, 2)], {"kind": "plan_set"})
    throws_dir = layout.throws_dir(ARM["arm"])
    throws_dir.mkdir()
    meta = {"plan_set_sha256": cpc.sha256_file(segment_set), "plan_set_throws": 2, "n_throws": 2}
    write_throw_list(throws_dir / "sample.json", [_throw(1), _throw(2)], meta)
    write_throw_list(throws_dir / "chunk_000.json", [_throw(1), _throw(2)], meta)
    _unit(layout, 8210, "chunk_000", [(1, "published", True), (2, "published", False)])
    assert _collect(layout).caught_ids == 1


# ── what a unit records ──────────────────────────────────────────────────────
def test_a_units_records_are_read_as_they_are_written(tmp_path):
    unit = tmp_path / "unit"
    unit.mkdir()
    assert cpc.unit_status(unit) is None and cpc.read_conditions(unit) == {}
    (unit / "status").write_text("FAIL:trials rc=1\n")
    (unit / "conditions.txt").write_text(
        "date_start: 2026-10-11T22:38:43+09:00\nseed: 8210\nexpect_kv: \nseed: 8211\n"
    )
    assert cpc.unit_status(unit) == "FAIL:trials rc=1"
    assert cpc.read_conditions(unit) == {
        "date_start": "2026-10-11T22:38:43+09:00",
        "expect_kv": "",
        "seed": "8211",
    }


def test_the_units_of_an_arm_are_found_by_seed_and_chunk(tmp_path):
    layout = ResultLayout(tmp_path)
    for seed, chunk in ((8211, "chunk_000"), (8210, "chunk_001"), (8210, "chunk_000")):
        layout.unit_dir("grid_mpc", seed, chunk).mkdir(parents=True)
    layout.unit_dir("grid_mpc", 8210, "chunk_000.fail1").mkdir()
    layout.unit_dir("nlp_mpc_docking", 8210, "chunk_000").mkdir(parents=True)
    (cpc.sim_root(layout) / "progress.log").write_text("")
    assert cpc.sim_root(layout) == tmp_path / "sim"
    found = cpc.units_present(cpc.sim_root(layout), "grid_mpc")
    assert [(u.seed, u.chunk) for u in found] == [
        (8210, "chunk_000"),
        (8210, "chunk_001"),
        (8211, "chunk_000"),
    ]
    assert found[0].name == "grid_mpc/seed_8210/chunk_000"
    assert cpc.units_present(cpc.sim_root(layout), "no_such_arm") == []


# ── the analysis ─────────────────────────────────────────────────────────────
def test_the_analysis_is_what_analyse_unit_sh_runs(tmp_path):
    unit = tmp_path / "unit"
    command = cpc.analysis_command("/usr/bin/python3", unit, "/share/config/ur5e_p1b")
    assert command == [
        "/usr/bin/python3",
        "-m",
        "integrated_bringup.catching_tc_vector",
        str(unit),
        "/share/config/ur5e_p1b",
        str(unit / "ct" / "vec.csv"),
        str(unit / "ct"),
    ]
    script = Path(__file__).resolve().parents[1] / "tools" / "catching_eval" / "analyse_unit.sh"
    assert '/usr/bin/python3 "$D/tc_vector.py" "$U" "$CFG" "$U/ct/vec.csv" "$U/ct"' in (
        script.read_text()
    )


def test_a_unit_is_analysed_once_and_only_when_done(tmp_path):
    unit = tmp_path / "unit"
    (unit / "ct").mkdir(parents=True)
    assert not cpc.needs_analysis(unit)  # no status: still being collected, or failed
    (unit / "status").write_text("FAIL:host_busy\n")
    assert not cpc.needs_analysis(unit)
    (unit / "status").write_text("DONE\n")
    assert cpc.needs_analysis(unit)
    (unit / "ct" / "vec.csv").write_text("idx\n")
    assert not cpc.needs_analysis(unit)


def test_a_failed_analysis_leaves_its_output_in_the_units_log(tmp_path):
    unit = tmp_path / "unit"
    unit.mkdir()
    assert not cpc.analyse_unit(unit, tmp_path, sys.executable)  # no session to read
    assert (unit / "ct" / "log.txt").read_text() != ""
    assert not (unit / "ct" / "vec.csv").exists()


def test_an_analysis_that_does_not_end_is_killed_and_says_so(tmp_path):
    unit = tmp_path / "unit"
    unit.mkdir()
    stuck = tmp_path / "python"
    stuck.write_text("#!/bin/bash\nsleep 30\n")
    stuck.chmod(0o755)
    assert not cpc.analyse_unit(unit, tmp_path, str(stuck), timeout_s=0.2)
    assert "analysis killed: it ran past 0.2 s" in (unit / "ct" / "log.txt").read_text()


def test_the_analysis_runs_under_the_installed_analysers_interpreter(tmp_path):
    """analyse_unit.sh names /usr/bin/python3: the installed catching_trials is bound to
    the distribution's python, and a venv carries another pandas."""
    assert tcv.shebang_interpreter("#!/usr/bin/python3") == "/usr/bin/python3"
    assert tcv.shebang_interpreter("#!/usr/bin/python3 -u") == "/usr/bin/python3"
    assert tcv.shebang_interpreter("#!/usr/bin/env python3") is None  # whatever PATH finds
    assert tcv.shebang_interpreter("import re") is None
    entry = tmp_path / "catching_trials"
    entry.write_text("#!/opt/python/bin/python3\n# EASY-INSTALL-ENTRY-SCRIPT\n")
    assert tcv.analysis_python(entry) == "/opt/python/bin/python3"
    entry.write_text("#!/usr/bin/env python3\n")
    assert tcv.analysis_python(entry) == sys.executable
    assert tcv.analysis_python(tmp_path / "missing") == sys.executable


def test_the_analyser_is_handed_the_unit_and_its_probe_dump_when_it_has_one(tmp_path):
    unit = tmp_path / "unit"
    assert tcv.trials_argv(unit, "/cfg", "/out") == [
        str(unit / "session"),
        str(unit / "trials"),
        "--config-dir",
        "/cfg",
        "--out",
        "/out",
    ]
    (unit / "probe").mkdir(parents=True)
    (unit / "probe" / "lane_prediction_dump.csv").write_text("t\n")
    assert tcv.trials_argv(unit, "/cfg", "/out")[-2:] == [
        "--probe-dump",
        str(unit / "probe" / "lane_prediction_dump.csv"),
    ]


def test_the_analysis_main_refuses_a_short_command_line(capsys):
    assert tcv.main([]) == 2
    assert "<unit_dir> <config_dir> <out.csv>" in capsys.readouterr().err


def test_a_caught_throw_keeps_every_key_a_throw_list_allows(tmp_path):
    """`caught_seeds` / `thrown_seeds` are not keys the sim driver would overwrite."""
    layout = _folder(tmp_path)
    _unit(layout, 8210, "chunk_000", [(1, "published", True)])
    _collect(layout)
    document = json.loads(layout.set_file(ARM["arm"], "catch").read_text())
    assert document["schema"] == "catching_throw_list/1"
    assert document["throws"][0]["caught_seeds"] == [8210]
