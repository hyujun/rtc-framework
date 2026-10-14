"""#821: the stage 1 command and the report of a throw population.

``catch_population`` strings existing tools together and ``catch_population_report``
counts what they wrote, so what is pinned here is the stringing and the counting:

- the report on a result folder built by hand — every number of the text is one the
  test can count, with and without stage 2's ``sim_throws.csv``; the stratified
  estimate on a hand-computed case; the plots are written;
- the pipeline end to end on a tiny grid with a stand-in judge binary: the layout's
  files, sets that follow the judge, a resume that redoes nothing, a refused folder,
  an empty tier and a refused planner that the run survives, ``--report-only``.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

import pytest
import yaml

from rtc_tools.analysis import (
    catch_population as cp,
    catch_population_report as rep,
    catch_search_map as csm,
    catch_throw_design as ctd,
)
from rtc_tools.analysis.catchability_map import BallParams
from rtc_tools.analysis.catching_throw_list import read_throw_list_file, write_throw_list
from rtc_tools.utils.catch_population import Arm, ResultLayout

_REPO = Path(__file__).resolve().parents[2]
_SHIPPED = _REPO / "integrated_bringup" / "config" / "ur5e_p1b"
G = 9.81


def _config_doc(arms: list[dict], population: dict | None = None, **sample) -> dict:
    return {
        "schema": "catch_population/1",
        "arms": arms,
        "ball": {
            "config": "mujoco_simulator.yaml",
            "drag_coefficient": {"value": 0.55, "source": "test"},
            "air_density": {"value": 1.204, "source": "test"},
        },
        "timing": {
            "detection_delay_s": 0.15,
            "vision_period_s": 0.05,
            "prediction_horizon_s": 1.0,
            "source": "a test profile",
        },
        "judge": {"horizon_s": 2.0, "step_s": 0.002, "min_catch_height_m": 0.0},
        "population": population
        or {
            "launch": {"distance_m": "2.0 3.0", "azimuth_deg": "0", "release_height_m": "1.4"},
            "pass": {"dx_m": "-0.2 0.0", "dy_m": "0.0"},
            "elevation_deg": {"max": 60, "min": 30, "step": 15},
            "min_pass_time_s": 0.5,
            "speed_max_m_s": 12,
        },
        "sample": {
            "throw_set": "segment",
            "max_throws": None,
            "strata": ["distance_m"],
            "seed": 1,
            "chunk": 2,
            **sample,
        },
        "sim": {
            "viewer": False,
            "estimator_seeds": [8210],
            "end_on_ball_low": True,
            "host_watch": "abort",
            "retries": 1,
        },
    }


# ── a result folder built by hand ─────────────────────────────────────────────

DISTANCES = (2.0, 3.0)
ELEVATIONS = (30.0, 45.0, 60.0)
PASS_DX = (-0.2, 0.0)
CATCH = (0.0, 0.0, 1.2)


def _throw(throw_id: int, distance: float, elevation: float, dx: float) -> dict:
    """A drag-free launch from (distance, 0, 1.4) through (dx, 0, 1.2) at ``elevation``."""
    theta = math.radians(elevation)
    reach, drop = distance - dx, CATCH[2] - 1.4
    speed = math.sqrt(
        G * reach**2 / (2.0 * math.cos(theta) ** 2 * (reach * math.tan(theta) - drop))
    )
    return {
        "throw_id": throw_id,
        "kind": "pass_point",
        "pos": (distance, 0.0, 1.4),
        "vel": (-speed * math.cos(theta), 0.0, speed * math.sin(theta)),
        "omega": (0.0, 0.0, 0.0),
        "distance_m": distance,
        "azimuth_deg": 0.0,
        "release_height_m": 1.4,
        "pass_dx_m": dx,
        "pass_dy_m": 0.0,
        "elevation_deg": elevation,
        "speed_m_s": speed,
        "pass_x": dx,
        "pass_y": 0.0,
        "pass_z": CATCH[2],
        "pass_time_s": reach / (speed * math.cos(theta)),
        "apex_height_m": 1.4 + (speed * math.sin(theta)) ** 2 / (2.0 * G),
        "screen": "",
        "limit": "",
    }


# Ids in product order: distance 2.0 holds 0..5, distance 3.0 holds 6..11.
POPULATION = [
    _throw(i, *values)
    for i, values in enumerate((d, e, x) for d in DISTANCES for e in ELEVATIONS for x in PASS_DX)
]
SETS = {
    "grid_mpc": {"search": [0, 1, 2, 6, 7, 10], "segment": [0, 1, 6]},
    "grid_mpc_docking": {"search": [0, 1, 2, 3, 6, 7, 8, 9], "segment": [0, 6]},
}
ARMS = [
    {"search": "grid", "segment": "mpc"},
    {"search": "grid", "segment": "mpc_docking", "throw_set": "search", "max_throws": 4},
    {"search": "nlp", "segment": "mpc"},
    {"search": "nlp", "segment": "mpc_docking"},
]


def _summary(search: str, accepted: int, segment: dict, reasons: dict) -> dict:
    return {
        "searches": {
            search: {"accepted": accepted, "reasons": {"throws": reasons}, "segment": segment}
        },
        "provenance": {
            "judge": {"path": "/somewhere/judge", "sha256": "ab" * 32},
            "catching_tree_sha256": "cd" * 32,
            "repo": None,
        },
    }


def _write_summary(layout: ResultLayout, arm: str, summary: dict) -> None:
    layout.map_dir(arm).mkdir(parents=True)
    (layout.map_dir(arm) / rep.MAP_SUMMARY).write_text(json.dumps(summary))


def _write_set(layout: ResultLayout, arm: str, tier: str, ids: list[int]) -> None:
    write_throw_list(layout.set_file(arm, tier), [POPULATION[i] for i in ids], {"tier": tier})


def _folder(root: Path) -> ResultLayout:
    """A result folder of POPULATION with four arms: one that throws its whole segment
    set, one that throws a sample of its search set, one whose segment planner refused
    its configuration, one not judged."""
    layout = ResultLayout(root)
    layout.design.mkdir(parents=True)
    layout.config.write_text(yaml.safe_dump(_config_doc(ARMS)))
    design = {
        "grid": {
            "axes": {
                "distance_m": list(DISTANCES),
                "azimuth_deg": [0.0],
                "release_height_m": [1.4],
                "pass_dx_m": list(PASS_DX),
                "pass_dy_m": [0.0],
                "elevation_deg": list(ELEVATIONS),
            },
            "throws": 20,
        },
        "elevation": {"max_deg": 60.0, "min_deg": 30.0, "step_deg": 15.0},
        "speed": {"max_m_s": 12.0},
        "limits": {"min_pass_time_s": 0.5, "limited": 5},
        "screens": {"labels": {"none": 17, "screen:shoot_ascending": 2, "screen:shoot_fail": 1}},
        "body": {"vertices_world_m": [[0.0, 0.0, 0.3], [0.0, 0.0, 0.8], [0.1, 0.0, 1.1]]},
        "wait_catch_point_world_m": list(CATCH),
        "approach_axis_world": [1.0, 0.0, 0.0],
        "ball": {
            "radius_m": 0.0335,
            "mass_kg": 0.057,
            "drag_coefficient": 0.0,
            "air_density_kg_m3": 0.0,
            "sources": dict.fromkeys(
                ("radius_m", "mass_kg", "drag_coefficient", "air_density_kg_m3"), "test"
            ),
        },
    }
    (layout.design / "design_meta.json").write_text(json.dumps(design))
    write_throw_list(layout.population, POPULATION, {})
    for arm, tiers in SETS.items():
        search = arm.split("_", 1)[0]
        _write_summary(
            layout,
            arm,
            _summary(
                search,
                len(tiers["search"]),
                {"accepted": len(tiers["segment"]), "reasons": {"throws": {"segment:slack": 3}}},
                {"search:too_far": 12 - len(tiers["search"]), "screen:shoot_ascending": 2},
            ),
        )
        for tier, ids in tiers.items():
            _write_set(layout, arm, tier, ids)
    assert ctd.main(_sample_argv(layout, "grid_mpc", "segment")) == 0
    assert (
        ctd.main([*_sample_argv(layout, "grid_mpc_docking", "search"), "--max-throws", "4"]) == 0
    )
    _write_summary(
        layout,
        "nlp_mpc",
        _summary("nlp", 2, {"refused_configuration": "the planner said no"}, {"search:reach": 10}),
    )
    _write_set(layout, "nlp_mpc", "search", [4, 5])
    return layout


def _sample_argv(layout: ResultLayout, arm: str, tier: str) -> list[str]:
    return [
        "sample", "--plan-set", str(layout.set_file(arm, tier)),
        "--out-dir", str(layout.throws_dir(arm)),
        "--strata", "distance_m", "--seed", "1", "--chunk", "2",
    ]  # fmt: skip


SIM_HEADER = (
    "arm", "seed", "chunk",
    # a unit's table has a `seed` of its own: the name is in the header twice
    "idx", "throw_id", "seed", "invalid_reason", "plan_verdict", "plan_reject", "truth_success",
)  # fmt: skip


def _write_sim(layout: ResultLayout, arm: str, rows: list[tuple]) -> None:
    """``rows``: (estimator seed, throw id, plan_verdict, truth_success[, invalid_reason])."""
    with layout.sim_throws(arm).open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(SIM_HEADER)
        for index, (seed, throw_id, verdict, truth, *invalid) in enumerate(rows):
            reject = "" if verdict == "published" else "segment:slack"
            writer.writerow(
                [
                    arm,
                    seed,
                    "chunk_000",
                    index,
                    throw_id,
                    "",
                    *(invalid or [""]),
                    verdict,
                    reject,
                    truth,
                ]
            )


def _sampled(layout: ResultLayout, arm: str) -> dict[float, list[int]]:
    """The ids ``arm`` throws, by distance."""
    out: dict[float, list[int]] = {}
    for throw in read_throw_list_file(layout.throws_dir(arm) / "sample.json").throws:
        out.setdefault(throw["distance_m"], []).append(throw["throw_id"])
    return out


def _with_sim(root: Path) -> ResultLayout:
    layout = _folder(root)
    _write_sim(
        layout,
        "grid_mpc",
        [
            (8210, 0, "published", "True"),
            (8210, 1, "published", "False"),
            (8210, 1, "published", "True", "host_busy"),  # a rig failure: not a thrown throw
            (8210, 6, "withheld", "False"),
            (8211, 0, "published", "False"),
            (8211, 1, "no_plan", "False"),
            (8211, 6, "published", "True"),
        ],
    )
    near, far = (_sampled(layout, "grid_mpc_docking")[d] for d in DISTANCES)
    assert len(near) == len(far) == 2  # 4 of 8, the same share of each distance
    _write_sim(
        layout,
        "grid_mpc_docking",
        [
            (8210, near[0], "published", "False"),
            (8210, near[1], "withheld", "False"),
            (8210, far[0], "published", "True"),
            (8210, far[1], "published", "False"),
        ],
    )
    return layout


def _line(text: str, start: str) -> str:
    """The first line of ``text`` that starts with ``start``, its whitespace runs single."""
    lines = [" ".join(line.split()) for line in text.splitlines()]
    found = [line for line in lines if line.startswith(start)]
    assert found, f"no line starts with {start!r}"
    return found[0]


def test_the_stage_1_report_counts_the_sets_from_the_folders_files(tmp_path):
    layout = _folder(tmp_path / "result")
    text = rep.report_text(rep.load_result(layout.root))

    assert "design points 20" in " ".join(text.split())
    assert (
        _line(text, "no launch — screen:shoot_ascending") == "no launch — screen:shoot_ascending 2"
    )
    assert _line(text, "removed by the pass-time limit") == "removed by the pass-time limit 5"
    assert _line(text, "POPULATION") == "POPULATION 12"
    assert _line(text, "elevation range") == "elevation range max 60 deg, min 30 deg, step 15 deg"

    arms = text.split("  arm ")
    grid = next(block for block in arms if block.startswith("grid_mpc\n"))
    assert _line(grid, "search set") == "search set 6 50.0 % of the population"
    assert _line(grid, "segment set") == (
        "segment set 3 25.0 % of the population, 50.0 % of the search set"
    )
    assert "3 of the segment set's 3 — the whole set" in _line(grid, "to throw")
    # the label of a throw without a launch is not a reason of the population
    assert _line(grid, "refused by the search") == (
        "refused by the search, of the population: search:too_far 6"
    )
    assert "segment:slack 3" in _line(grid, "refused at the segment tier")
    assert "judge sha256 abababababab, composed tree sha256 cdcdcdcdcdcd" in grid

    sampled = next(block for block in arms if block.startswith("grid_mpc_docking\n"))
    assert "4 of the search set's 8 (strata distance_m, seed 1, 2 chunks of 2)" in _line(
        sampled, "to throw"
    )
    refused = next(block for block in arms if block.startswith("nlp_mpc\n"))
    assert _line(refused, "search set") == "search set 2 16.7 % of the population"
    assert _line(refused, "segment set") == (
        "segment set the segment planner refused its configuration — the planner said no"
    )
    unjudged = next(block for block in arms if block.startswith("nlp_mpc_docking\n"))
    assert _line(unjudged, "search set") == "search set not judged"
    assert _line(unjudged, "segment set") == "segment set not judged"
    assert _line(unjudged, "to throw") == "to throw not judged"

    assert _line(text, "search sets of grid_mpc and grid_mpc_docking") == (
        "search sets of grid_mpc and grid_mpc_docking: both 5, grid_mpc only 1, "
        "grid_mpc_docking only 3, neither 3"
    )
    # per axis value: the population's count there, each set's count and share of it
    table = text[text.index("BY AXIS VALUE AND BIN") :]
    header = _line(table, "population").split()
    assert header == [
        "population", "grid_mpc:search", "grid_mpc:segment",
        "grid_mpc_docking:search", "grid_mpc_docking:segment", "nlp_mpc:search",
    ]  # fmt: skip
    distance = table[table.index(" distance_m") : table.index(" azimuth_deg")]
    assert _line(distance, "2 ") == "2 6 3 50.0 % 2 33.3 % 4 66.7 % 1 16.7 % 2 33.3 %"
    assert _line(distance, "3 ") == "3 6 3 50.0 % 1 16.7 % 4 66.7 % 1 16.7 % 0 0.0 %"
    for axis, width in rep.BINNED_AXES:
        assert f" {axis} (bins of {width:g})" in table
    # without stage 2 results this is all there is
    assert "THE SIM" not in text and "SIM BY AXIS" not in text


def test_with_sim_throws_the_report_adds_the_sim_tiers_per_arm_and_seed(tmp_path):
    layout = _with_sim(tmp_path / "result")
    text = rep.report_text(rep.load_result(layout.root))
    sim = text[text.index("THE SIM (stage 2)") : text.index("BY AXIS VALUE AND BIN")]
    whole, sampled = sim.split("  arm ")[1:]

    first, second = whole.split("estimator seed ")[1:]
    assert first.startswith("8210: 4 rows, 1 invalid (a rig failure), 0 without a verdict")
    assert _line(first, "thrown") == "thrown 3"
    assert _line(first, "pair published").startswith("pair published 2 66.7 % of the thrown [")
    caught = _line(first, "caught (truth_success)")
    assert caught.startswith("caught (truth_success) 1 33.3 % of the thrown [")
    assert caught.endswith("50.0 % of the published")
    assert _line(first, "not published") == "not published, by the sim's reason: segment:slack 1"
    assert "THE WHOLE SEGMENT SET WAS THROWN (3 throws)" in first and "ESTIMATE" not in first
    assert second.startswith("8211: 3 rows, 0 invalid")
    assert _line(second, "pair published").startswith("pair published 2 66.7 %")

    # a sample of the search set: counts within the thrown, and the set-wide ESTIMATE apart
    assert sampled.startswith("grid_mpc_docking — thrown from its search set")
    assert "COUNTS WITHIN THE THROWN" in sampled
    assert _line(sampled, "pair published").startswith("pair published 3 75.0 % of the thrown")
    assert "ESTIMATE FOR THE WHOLE SEARCH SET (8 throws; 4 thrown)" in sampled
    assert "weighted by its set count over its thrown count; not a count" in sampled
    estimate = sampled[sampled.index("ESTIMATE") :]
    # distance 2.0: 4 x 1/2, distance 3.0: 4 x 2/2 published; caught 4 x 0/2 + 4 x 1/2
    assert _line(estimate, "pair published") == (
        "pair published ~ 6.0 75.0 % of the 8 set throws in a stratum with a thrown throw"
    )
    assert _line(estimate, "caught") == (
        "caught ~ 2.0 25.0 % of the 8 set throws in a stratum with a thrown throw"
    )

    tables = text[text.index("SIM BY AXIS VALUE AND BIN — arm grid_mpc, estimator seed 8210") :]
    assert _line(tables, "thrown").split() == ["thrown", "pair", "published", "caught"]
    distance = tables[tables.index(" distance_m") : tables.index(" azimuth_deg")]
    assert _line(distance, "2 ") == "2 2 2 100.0 % 1 50.0 %"
    assert _line(distance, "3 ") == "3 1 0 0.0 % 0 0.0 %"
    assert "arm grid_mpc, estimator seed 8211" in text
    assert "arm grid_mpc_docking, estimator seed 8210" in text


def test_a_partly_thrown_sample_leaves_the_unthrown_strata_out_of_the_estimate(tmp_path):
    layout = _folder(tmp_path / "result")
    near = _sampled(layout, "grid_mpc_docking")[2.0]
    _write_sim(
        layout,
        "grid_mpc_docking",
        [(8210, near[0], "published", "True"), (8210, near[1], "published", "False")],
    )
    text = rep.report_text(rep.load_result(layout.root))
    estimate = text[text.index("ESTIMATE FOR THE WHOLE SEARCH SET (8 throws; 2 thrown)") :]
    assert _line(estimate, "pair published").startswith(
        "pair published ~ 4.0 100.0 % of the 4 set"
    )
    assert _line(estimate, "caught").startswith("caught ~ 2.0 50.0 % of the 4 set")
    assert _line(estimate, "outside the estimate") == (
        "outside the estimate: 4 set throws in 1 strata without a thrown throw"
    )


def test_the_stratified_estimate_weights_each_stratum_by_its_set_count_over_its_thrown_count():
    found = rep.stratified_estimate(
        {("a",): 10, ("b",): 6, ("c",): 4}, {("a",): 5, ("b",): 3}, {("a",): 2, ("b",): 3}
    )
    assert found["estimate"] == pytest.approx(10 * 2 / 5 + 6 * 3 / 3)
    assert (found["set_throws"], found["covered"], found["uncovered"]) == (20, 16, 4)
    assert (found["uncovered_strata"], found["thrown"], found["hits"]) == (1, 8, 5)
    assert not found["exact"]
    whole = rep.stratified_estimate({("a",): 2, ("b",): 1}, {("a",): 2, ("b",): 1}, {("a",): 1})
    assert whole["exact"] and whole["estimate"] == 1.0 and whole["uncovered"] == 0
    with pytest.raises(ValueError, match="3 thrown of a set count of 2"):
        rep.stratified_estimate({("a",): 2}, {("a",): 3}, {})
    with pytest.raises(ValueError, match="strata the set does not have"):
        rep.stratified_estimate({("a",): 2}, {("z",): 1}, {})


def test_bins_stand_on_multiples_of_the_width_and_hold_every_value():
    throws = [{"x": 0.52}, {"x": 0.99}, {"x": 1.0}, {"x": None}]
    assert rep.bin_edges(throws, "x", 0.1) == pytest.approx([0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1])
    assert rep.bin_edges([{"x": None}], "x", 0.1) is None
    groups = rep.shares_by_axis(
        [{"throw_id": i, "x": v} for i, v in enumerate((0.52, 0.99, 1.0))],
        {1, 2},
        "x",
        rep.bin_edges(throws, "x", 0.1),
    )
    assert [(g["n"], g["accepted"]) for g in groups] == [
        (1, 0),
        (0, 0),
        (0, 0),
        (0, 0),
        (1, 1),
        (1, 1),
    ]


def test_a_tier_is_a_set_an_empty_set_or_no_set_with_its_reason(tmp_path):
    layout = ResultLayout(tmp_path)
    arm = Arm("grid", "mpc", "segment", None)
    assert rep.tier_state(layout, arm, "search", None) == rep.TierState(None, "not judged", None)

    def summary(entry):
        return {"searches": {"grid": entry}}

    refused = summary({"refused_configuration": "no grid map"})
    for tier in rep.OFFLINE_TIERS:
        state = rep.tier_state(layout, arm, tier, refused)
        assert state.count is None and not state.pending and "no grid map" in state.why

    judged = summary({"accepted": 3, "segment": {"accepted": 0}})
    pending = rep.tier_state(layout, arm, "search", judged)
    assert pending.count is None and pending.pending and "not written" in pending.why
    # the judge accepted no throw: a set of 0 throws, for which no file is written
    assert rep.tier_state(layout, arm, "segment", judged) == rep.TierState(0, "", None)
    layout.arm(arm.name).mkdir()
    layout.set_file(arm.name, "search").write_text("{}")
    assert rep.tier_state(layout, arm, "search", judged) == rep.TierState(
        3, "", layout.set_file(arm.name, "search")
    )
    stopped = summary({"accepted": 3, "segment": {"refused_configuration": "eta out of range"}})
    state = rep.tier_state(layout, arm, "segment", stopped)
    assert state.count is None and "segment planner refused" in state.why
    # a map that ran no segment pass judged no segment tier
    assert rep.tier_state(layout, arm, "segment", summary({"accepted": 3})).why == "not judged"


def test_files_that_contradict_each_other_are_refused(tmp_path):
    layout = _with_sim(tmp_path / "a")
    _write_set(layout, "grid_mpc", "segment", [0, 1])  # the map accepted 3
    with pytest.raises(SystemExit, match="holds 2 throws but the map of grid_mpc accepted 3"):
        rep.load_result(layout.root)

    layout = _with_sim(tmp_path / "b")
    _write_sim(
        layout, "grid_mpc", [(8210, 0, "published", "True"), (8210, 0, "withheld", "False")]
    )
    with pytest.raises(SystemExit, match="seed 8210 holds 1 throws more than once"):
        rep.load_result(layout.root)

    layout = _with_sim(tmp_path / "c")
    _write_sim(layout, "grid_mpc", [(8210, 11, "published", "True")])  # not in the thrown list
    with pytest.raises(SystemExit, match="1 throws that .*sample.json does not"):
        rep.load_result(layout.root)

    layout = _with_sim(tmp_path / "d")
    _write_sim(layout, "grid_mpc", [(8210, 99, "published", "True")])
    with pytest.raises(SystemExit, match="the population does not"):
        rep.load_result(layout.root)

    layout = _with_sim(tmp_path / "e")
    layout.sim_throws("grid_mpc").write_text("throw_id,seed\n0,1\n")
    with pytest.raises(SystemExit, match="the first columns must be arm, seed, chunk"):
        rep.load_result(layout.root)

    with pytest.raises(SystemExit, match="not a result folder"):
        rep.load_result(tmp_path / "nothing")


def test_the_drawn_throws_are_picked_by_id_and_a_flight_ends_at_its_pass_point():
    picked = rep.pick_throws(POPULATION, 5)
    assert len(picked) == 5
    assert rep.pick_throws(list(reversed(POPULATION)), 5) == picked
    assert rep.pick_throws(POPULATION, 50) == POPULATION

    sources = dict.fromkeys(("radius_m", "mass_kg", "drag_coefficient", "air_density_kg_m3"), "t")
    ball = BallParams(0.0335, 0.057, 0.0, 0.0, sources)
    for throw, path in zip(POPULATION, rep.flight_paths(POPULATION, ball), strict=True):
        assert path[0] == pytest.approx(throw["pos"])
        assert path[-1] == pytest.approx(
            [throw["pass_x"], throw["pass_y"], throw["pass_z"]], abs=1e-3
        )
    assert rep.flight_paths([], ball) == []


def test_the_report_writes_the_text_it_prints_and_a_plot_per_set(tmp_path, capsys):
    layout = _with_sim(tmp_path / "result")
    capsys.readouterr()
    assert rep.main([str(layout.root), "--max-flights", "4"]) == 0
    printed = capsys.readouterr().out
    text = (layout.report / rep.REPORT_TEXT).read_text()
    assert printed.startswith(text)
    assert "drawn flights end within 0." in printed  # the drag-free flights: under a millimetre

    expected = {
        "set_population.png",
        "set_grid_mpc_search.png",
        "set_grid_mpc_segment.png",
        "sets_grid_mpc.png",
        "set_grid_mpc_published_seed_8210.png",
        "set_grid_mpc_caught_seed_8210.png",
        "sets_grid_mpc_seed_8210.png",
        "set_grid_mpc_published_seed_8211.png",
        "set_grid_mpc_caught_seed_8211.png",
        "sets_grid_mpc_seed_8211.png",
        "set_grid_mpc_docking_search.png",
        "set_grid_mpc_docking_segment.png",
        "sets_grid_mpc_docking.png",
        "set_grid_mpc_docking_published_seed_8210.png",
        "set_grid_mpc_docking_caught_seed_8210.png",
        "sets_grid_mpc_docking_seed_8210.png",
        "set_nlp_mpc_search.png",
        "sets_nlp_mpc.png",
    }
    written = {p.name for p in layout.report.glob("*.png")}
    assert written == expected
    assert all((layout.report / name).stat().st_size > 1000 for name in expected)


# ── the pipeline, with a stand-in judge ───────────────────────────────────────

# A stand-in for catch_search_batch that answers by throw id. It reports a reach bound
# (the design asks for one). Search pass: an even throw gets a plan at its second wake.
# Segment pass (the binding has a `segment` map): an even throw whose id is a multiple of 3
# has its pair published at its second wake, the other even throws never get past `slack` —
# unless the segment planner's mode is in BARREN (no pair, ever) or in REFUSE (exit 3).
_FAKE_JUDGE = """#!{python}
import csv, json, sys, yaml
argv = sys.argv[1:]
if "--print-reach-bound" in argv:
    print(json.dumps({{"schema": "catch_reach_bound/1", "centre": [0.0, 0.0, 0.5],
                      "radius": 5.0, "tolerance": 0.002, "joints": [], "unbounded_by": []}}))
    sys.exit(0)
opt = {{argv[i]: argv[i + 1] for i in range(0, len(argv) - 1, 2)}}
binding = yaml.safe_load(open(opt["--binding"]))
segment = "segment" in binding
mode = binding["segment"]["mode"] if segment else ""
if mode in {refuse!r}:
    print("catch_search_batch: the segment planner refused its configuration - fake", file=sys.stderr)
    sys.exit(3)
nv = len(binding["device_of_model"])
wakes = {{}}
for row in csv.DictReader(open(opt["--wakes"])):
    wakes.setdefault(int(row["throw_id"]), {{}})[int(row["wake"])] = int(row["now_ns"])
head = ["throw_id", "wake", "now_ns"] + (["outcome"] if segment else []) + [
    "search_valid", "plan_reason", "nlp_reason", "t_c_ns", "lead_s",
    *[f"p_c_{{a}}" for a in "xyz"], *[f"q_star{{i}}" for i in range(nv)]]
if segment:
    head += ["segment_outcome", "segment_kind", "segment_core_reason", "segment_k"]
head += ["wall_us"]
with open(opt["--out"], "w") as handle:
    handle.write(",".join(head) + "\\n")
    for tid in sorted(wakes):
        for wake in sorted(wakes[tid]):
            valid = int(tid % 2 == 0 and wake >= 1)
            outcome, seg, kind = "published", "off", "none"
            published = valid
            if segment and valid:
                kind = "first"
                published = int(tid % 3 == 0 and mode not in {barren!r})
                outcome = "published" if published else "held"
                seg = "published" if published else "slack"
            plan = ["1900000000", "0.7", *["0.1"] * 3, *["0.2"] * nv] if published else [""] * (5 + nv)
            cells = [str(tid), str(wake), str(wakes[tid][wake])]
            if segment:
                cells.append(outcome)
            cells += [str(valid), "0" if valid else "4", "off"] + plan
            if segment:
                cells += [seg, kind, "none", "0"]
            cells.append("12.5")
            handle.write(",".join(cells) + "\\n")
            if published or (not segment and valid):
                break
"""

TINY = {
    "launch": {"distance_m": "1.5 3.5", "azimuth_deg": "-10 10", "release_height_m": "1.4"},
    "pass": {"dx_m": "-0.2 0.0", "dy_m": "0.0"},
    "elevation_deg": {"max": 60, "min": 15, "step": 15},
    "min_pass_time_s": 0.5,
    "speed_max_m_s": 12,
}
TWO_ARMS = [{"search": "grid", "segment": "mpc"}, {"search": "nlp", "segment": "mpc_docking"}]


def _fake_judge(folder: Path, *, barren: tuple = (), refuse: tuple = ()) -> Path:
    path = folder / "fake_catch_search_batch"
    path.write_text(_FAKE_JUDGE.format(python=sys.executable, barren=barren, refuse=refuse))
    path.chmod(0o755)
    return path


def _robot_folder(folder: Path, arms: list[dict] = TWO_ARMS, **sample) -> Path:
    """A copy of a shipped robot folder with a tiny population file in it; the file."""
    pytest.importorskip("pinocchio")
    if not _SHIPPED.is_dir():
        pytest.skip("integrated_bringup config is not beside this checkout")
    config_dir = folder / "some_robot"
    shutil.copytree(_SHIPPED, config_dir)
    path = config_dir / "catch_population.yaml"
    path.write_text(yaml.safe_dump(_config_doc(arms, TINY, **sample)))
    return path


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _ids(path: Path) -> list[int]:
    return [t["throw_id"] for t in read_throw_list_file(path).throws]


def _run(config: Path, out: Path, judge: Path, *extra: str) -> int:
    return cp.main(
        [
            "--config",
            str(config),
            "--out",
            str(out),
            "--judge",
            str(judge),
            "--max-flights",
            "3",
            *extra,
        ]
    )


def _products(layout: ResultLayout) -> list[Path]:
    """What a finished stage 1 leaves, less the report."""
    files = [layout.design / "design_meta.json", layout.population]
    for arm in ("grid_mpc", "nlp_mpc_docking"):
        files.append(layout.map_dir(arm) / rep.MAP_SUMMARY)
        files += [layout.set_file(arm, tier) for tier in rep.OFFLINE_TIERS]
        files += sorted(layout.throws_dir(arm).iterdir())
    return files


@pytest.fixture(scope="module")
def finished(tmp_path_factory):
    """One whole run of the tiny population: (configuration file, judge, layout)."""
    root = tmp_path_factory.mktemp("population")
    config = _robot_folder(root)
    judge = _fake_judge(root)
    layout = ResultLayout(root / "result")
    assert _run(config, layout.root, judge) == 0
    return config, judge, layout


def test_the_run_writes_the_layout_and_the_sets_follow_the_judge(finished):
    config, judge, layout = finished
    assert layout.config.read_bytes() == config.read_bytes()
    population = _ids(layout.population)
    design = ctd.read_design_csv(layout.design / "design.csv")
    kept = [
        r["throw_id"]
        for r in design
        if not r["limit"] and r["screen"] not in ctd.UNLAUNCHABLE_SCREENS
    ]
    assert population == kept and 0 < len(population) < len(design)
    # the stand-in judge screens nothing by reach, so every throw of the population is judged
    judged = [
        i
        for i, t in zip(population, read_throw_list_file(layout.population).throws, strict=True)
        if not t["screen"]
    ]

    for arm, search, mode in (
        ("grid_mpc", "grid", "mpc"),
        ("nlp_mpc_docking", "nlp", "mpc_docking"),
    ):
        summary = json.loads((layout.map_dir(arm) / rep.MAP_SUMMARY).read_text())
        # one run judged both tiers, with the arm's two modes and nothing else changed
        assert list(summary["searches"]) == [search]
        assert summary["segment_mode"] == mode
        assert set(summary["overrides"]) == {"planner.segment.mode"}
        assert summary["provenance"]["segments"]["mode"] == mode
        searched = _ids(layout.set_file(arm, "search"))
        assert searched == [i for i in judged if i % 2 == 0] and searched
        segment = _ids(layout.set_file(arm, "segment"))
        assert segment == [i for i in judged if i % 6 == 0] and 0 < len(segment) < len(searched)
        assert read_throw_list_file(layout.set_file(arm, "segment")).meta["tier"] == "segment"
        # what stage 2 throws: the whole segment set, shuffled, in chunks of 2
        sample = read_throw_list_file(layout.throws_dir(arm) / "sample.json")
        assert sorted(t["throw_id"] for t in sample.throws) == segment
        assert sample.meta["plan_set_sha256"] == _sha(layout.set_file(arm, "segment"))
        assert (sample.meta["seed"], sample.meta["chunk"], sample.meta["strata_axes"]) == (
            1,
            2,
            ["distance_m"],
        )
        chunks = layout.chunks(arm)
        assert len(chunks) == math.ceil(len(segment) / 2)
        assert [i for chunk in chunks for i in _ids(chunk)] == [
            t["throw_id"] for t in sample.throws
        ]


def test_the_manifest_says_what_was_judged_with_what(finished):
    config, judge, layout = finished
    manifest = json.loads(layout.manifest.read_text())
    assert manifest["schema"] == cp.MANIFEST_SCHEMA
    assert manifest["config"] == {
        "source": str(config),
        "robot": "some_robot",
        "sha256": _sha(config),
        "copy": "config.yaml",
    }
    assert manifest["design"]["population"] == {
        "file": "population.json",
        "throws": len(_ids(layout.population)),
        "sha256": _sha(layout.population),
    }
    for arm in ("grid_mpc", "nlp_mpc_docking"):
        entry = manifest["arms"][arm]
        assert entry["judged"] and entry["throw_set"] == "segment"
        for tier in rep.OFFLINE_TIERS:
            path = layout.set_file(arm, tier)
            assert entry["sets"][tier] == {
                "file": path.name,
                "throws": len(_ids(path)),
                "sha256": _sha(path),
            }
        assert entry["throws"]["set_sha256"] == entry["sets"]["segment"]["sha256"]
        assert entry["throws"]["chunks"] == [p.name for p in layout.chunks(arm)]
        provenance = entry["provenance"]
        assert provenance["judge"] == {"path": str(judge), "sha256": _sha(judge)}
        assert provenance["catching_tree_sha256"] == _sha(
            layout.map_dir(arm) / "catching_tree.yaml"
        )
        assert provenance["overrides"]["planner.segment.mode"]["override"] == entry["segment"]
    # the revision is written only where there is a checkout to read it from
    assert ("repo" in manifest) == (csm.repo_state() is not None)


def test_the_run_ends_with_the_report(finished):
    _, _, layout = finished
    text = (layout.report / rep.REPORT_TEXT).read_text()
    assert text.startswith("catch population report — some_robot\n")
    population = len(_ids(layout.population))
    assert _line(text, "POPULATION") == f"POPULATION {population}"
    for arm in ("grid_mpc", "nlp_mpc_docking"):
        block = next(b for b in text.split("  arm ") if b.startswith(f"{arm}\n"))
        assert _line(block, "search set").startswith(
            f"search set {len(_ids(layout.set_file(arm, 'search')))} "
        )
        assert _line(block, "segment set").startswith(
            f"segment set {len(_ids(layout.set_file(arm, 'segment')))} "
        )
        assert "segment:slack" in _line(block, "refused at the segment tier")
        assert (layout.report / f"sets_{arm}.png").is_file()
    assert (layout.report / "set_population.png").is_file()


def test_a_second_run_redoes_nothing_and_a_removed_product_alone_is_made_again(finished):
    config, judge, layout = finished
    before = {p: (p.stat().st_mtime_ns, p.read_bytes()) for p in _products(layout)}
    assert _run(config, layout.root, judge) == 0
    assert {p: p.stat().st_mtime_ns for p in before} == {p: m for p, (m, _) in before.items()}

    # what an interrupted run leaves: a set not written, a sample cut short
    gone = [layout.set_file("grid_mpc", "segment"), layout.chunks("grid_mpc")[-1]]
    for path in gone:
        path.unlink()
    assert _run(config, layout.root, judge) == 0
    after = {p: (p.stat().st_mtime_ns, p.read_bytes()) for p in _products(layout)}
    assert set(after) == set(before)
    remade = {p for p in after if after[p][0] != before[p][0]}
    assert remade == {
        layout.set_file("grid_mpc", "segment"),
        *layout.throws_dir("grid_mpc").iterdir(),
    }
    assert all(after[p][1] == before[p][1] for p in after)  # the same bytes


def test_a_folder_judged_with_other_files_is_refused_by_their_names(tmp_path):
    # The population file's hash does not cover the controller configuration beside it:
    # a skipped step would hand the old verdicts on as the new files'.
    config = _robot_folder(tmp_path)
    judge = _fake_judge(tmp_path)
    layout = ResultLayout(tmp_path / "result")
    assert _run(config, layout.root, judge) == 0
    recorded = json.loads(layout.inputs.read_text())
    assert recorded["config_dir"]["sim.yaml"] == _sha(config.parent / "sim.yaml")
    assert config.name not in recorded["config_dir"]  # that file is held by config.yaml
    manifest = layout.manifest.read_bytes()

    layer = config.parent / "sim.yaml"
    kept = layer.read_bytes()
    layer.write_bytes(kept + b"# tuned\n")
    with pytest.raises(SystemExit, match="judged with other files.*sim.yaml.*use a new --out"):
        _run(config, layout.root, judge)
    layer.write_bytes(kept)

    added = config.parent / "controllers" / "added.yaml"
    added.parent.mkdir(exist_ok=True)
    added.write_text("{}\n")
    with pytest.raises(SystemExit, match="changed since: controllers/added.yaml"):
        _run(config, layout.root, judge)
    added.unlink()

    assert layout.manifest.read_bytes() == manifest  # a refused run wrote nothing
    assert _run(config, layout.root, judge) == 0  # the files as they were: a resume


def _rebuilt(judge: Path) -> Path:
    """The same judge as another binary (a rebuild): other bytes, the same behaviour."""
    other = judge.parent / "rebuilt"
    other.mkdir(exist_ok=True)
    path = other / judge.name
    path.write_text(judge.read_text() + "# rebuilt\n")
    path.chmod(0o755)
    return path


def test_the_judges_binary_is_recorded_by_its_sha256_alone(finished):
    _, judge, layout = finished
    recorded = json.loads(layout.inputs.read_text())
    assert recorded["judge_sha256"] == _sha(judge)
    assert judge.name not in layout.inputs.read_text()  # its path is the PC's


def test_another_judge_binary_is_warned_of_and_the_record_is_kept(tmp_path, capsys):
    config = _robot_folder(tmp_path)
    judge = _fake_judge(tmp_path)
    layout = ResultLayout(tmp_path / "result")
    assert _run(config, layout.root, judge) == 0
    recorded = layout.inputs.read_bytes()
    capsys.readouterr()
    # One arm is judged again — by the new binary; the other keeps its verdicts.
    shutil.rmtree(layout.map_dir("nlp_mpc_docking"))
    assert _run(config, layout.root, _rebuilt(judge)) == 0
    said = capsys.readouterr()
    assert "WARNING the judge is another binary" in said.err
    assert layout.inputs.read_bytes() == recorded
    assert "judge binaries: 2 different — grid_mpc " in said.out


def test_a_folder_that_recorded_no_judge_is_not_warned_of(tmp_path, capsys):
    config = _robot_folder(tmp_path)
    judge = _fake_judge(tmp_path)
    layout = ResultLayout(tmp_path / "result")
    assert _run(config, layout.root, judge) == 0
    older = json.loads(layout.inputs.read_text())
    del older["judge_sha256"]  # what a folder judged before #822 holds
    layout.inputs.write_text(json.dumps(older, indent=2) + "\n")
    capsys.readouterr()
    assert _run(config, layout.root, _rebuilt(judge)) == 0
    assert "WARNING" not in capsys.readouterr().err


def test_a_sample_of_a_set_that_is_no_longer_there_is_drawn_again(tmp_path):
    config = _robot_folder(tmp_path)
    judge = _fake_judge(tmp_path)
    layout = ResultLayout(tmp_path / "result")
    assert _run(config, layout.root, judge) == 0
    sample = layout.throws_dir("grid_mpc") / "sample.json"
    document = json.loads(sample.read_text())
    assert document["meta"]["plan_set_sha256"] == _sha(layout.set_file("grid_mpc", "segment"))
    # a sample that says it was drawn from other bytes than the set file's
    document["meta"]["plan_set_sha256"] = "00" * 32
    sample.write_text(json.dumps(document))
    assert _run(config, layout.root, judge) == 0
    again = json.loads(sample.read_text())
    assert again["meta"]["plan_set_sha256"] == _sha(layout.set_file("grid_mpc", "segment"))


def test_a_folder_of_another_configuration_is_refused(finished, tmp_path):
    config, judge, layout = finished
    other = tmp_path / "some_robot"
    shutil.copytree(config.parent, other)
    changed = other / config.name
    changed.write_text(changed.read_text() + "# a comment is other bytes\n")
    manifest = layout.manifest.read_bytes()
    with pytest.raises(SystemExit, match="another configuration.*use a new --out"):
        _run(changed, layout.root, judge)
    with pytest.raises(SystemExit, match="another configuration.*use a new --out"):
        _run(changed, layout.root, judge, "--report-only")
    assert (
        layout.manifest.read_bytes() == manifest
        and layout.config.read_bytes() == config.read_bytes()
    )

    stray = tmp_path / "stray"
    stray.mkdir()
    (stray / "notes.txt").write_text("not a result")
    with pytest.raises(SystemExit, match="not a result folder of this command"):
        _run(config, stray, judge)
    assert sorted(p.name for p in stray.iterdir()) == ["notes.txt"]


def test_report_only_writes_the_report_again_and_judges_nothing(finished, capsys):
    config, _, layout = finished
    before = {p: p.stat().st_mtime_ns for p in [*_products(layout), layout.manifest]}
    text = (layout.report / rep.REPORT_TEXT).read_text()
    shutil.rmtree(layout.report)
    capsys.readouterr()
    # no --config, and a judge that does not exist: nothing is judged
    assert cp.main(["--out", str(layout.root), "--report-only", "--max-flights", "3"]) == 0
    assert capsys.readouterr().out.startswith(text)
    assert (layout.report / rep.REPORT_TEXT).read_text() == text
    assert (layout.report / "set_population.png").is_file()
    assert {p: p.stat().st_mtime_ns for p in before} == before
    # the report command is the same function
    assert rep.main([str(layout.root), "--max-flights", "3"]) == 0
    assert (layout.report / rep.REPORT_TEXT).read_text() == text
    with pytest.raises(SystemExit, match="--config is required"):
        cp.main(["--out", str(layout.root)])
    with pytest.raises(SystemExit, match="no arm 'nlp_mpc'"):
        cp.main(["--config", str(config), "--out", str(layout.root), "--arms", "nlp_mpc"])


def test_an_empty_tier_is_a_result_and_the_other_arms_go_on(tmp_path):
    config = _robot_folder(tmp_path)
    layout = ResultLayout(tmp_path / "result")
    # first one arm only: the other is not judged yet
    assert _run(config, layout.root, _fake_judge(tmp_path), "--arms", "grid_mpc") == 0
    text = (layout.report / rep.REPORT_TEXT).read_text()
    block = next(b for b in text.split("  arm ") if b.startswith("nlp_mpc_docking\n"))
    assert _line(block, "search set") == "search set not judged"
    manifest = json.loads(layout.manifest.read_text())
    assert manifest["arms"]["nlp_mpc_docking"]["judged"] is False
    assert not layout.arm("nlp_mpc_docking").exists()
    grid_map = (layout.map_dir("grid_mpc") / rep.MAP_SUMMARY).stat().st_mtime_ns

    # then the rest, with a judge whose docking planner never publishes a pair
    assert _run(config, layout.root, _fake_judge(tmp_path, barren=("mpc_docking",))) == 0
    assert (layout.map_dir("grid_mpc") / rep.MAP_SUMMARY).stat().st_mtime_ns == grid_map
    assert _ids(layout.set_file("nlp_mpc_docking", "search"))
    assert not layout.set_file("nlp_mpc_docking", "segment").exists()
    assert not layout.throws_dir("nlp_mpc_docking").exists()
    assert layout.chunks("grid_mpc")  # the other arm has its throws
    manifest = json.loads(layout.manifest.read_text())
    entry = manifest["arms"]["nlp_mpc_docking"]
    assert entry["sets"]["segment"] == {"throws": 0, "empty": "the judge accepted no throw"}
    assert entry["sets"]["search"]["throws"] > 0 and entry["throws"] is None
    text = (layout.report / rep.REPORT_TEXT).read_text()
    block = next(b for b in text.split("  arm ") if b.startswith("nlp_mpc_docking\n"))
    assert _line(block, "segment set").startswith("segment set 0 0.0 % of the population")
    assert _line(block, "to throw") == "to throw 0 the segment set is empty"
    assert not (layout.report / "set_nlp_mpc_docking_segment.png").exists()
    assert (layout.report / "sets_nlp_mpc_docking.png").is_file()
    # a third run finds nothing to do: an empty tier is finished, not pending
    assert _run(config, layout.root, _fake_judge(tmp_path, barren=("mpc_docking",))) == 0


def test_a_planner_that_refuses_its_configuration_leaves_no_set_and_the_run_goes_on(tmp_path):
    config = _robot_folder(
        tmp_path,
        [
            {"search": "grid", "segment": "mpc_docking"},
            {"search": "grid", "segment": "mpc", "throw_set": "search", "max_throws": 3},
        ],
    )
    layout = ResultLayout(tmp_path / "result")
    assert _run(config, layout.root, _fake_judge(tmp_path, refuse=("mpc_docking",))) == 0
    assert _ids(layout.set_file("grid_mpc_docking", "search"))  # the search pass stands
    assert not layout.set_file("grid_mpc_docking", "segment").exists()
    assert not layout.throws_dir("grid_mpc_docking").exists()
    manifest = json.loads(layout.manifest.read_text())
    refused = manifest["arms"]["grid_mpc_docking"]["sets"]["segment"]
    assert refused["throws"] is None and "refused its configuration" in refused["no_set"]
    text = (layout.report / rep.REPORT_TEXT).read_text()
    block = next(b for b in text.split("  arm ") if b.startswith("grid_mpc_docking\n"))
    assert "the segment planner refused its configuration" in _line(block, "segment set")
    assert _line(block, "to throw") == "to throw nothing — there is no segment set"
    # the arm that throws a sample of its search set: its own throw_set and max_throws
    sample = read_throw_list_file(layout.throws_dir("grid_mpc") / "sample.json")
    assert len(sample.throws) == 3 and sample.meta["max_throws"] == 3
    assert sample.meta["plan_set_sha256"] == _sha(layout.set_file("grid_mpc", "search"))
    assert manifest["arms"]["grid_mpc"]["throws"]["of_set"] == "search"
