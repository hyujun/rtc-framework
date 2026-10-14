"""The shipped throw sets (#821): ``config/<robot>/throw_sets/throw_catch_set_<arm>.json``.

A robot ships, per arm (search x segment planner), the CATCH SET of its throw
population: the throws of the arm's segment set that the sim caught in the
viewer runs of 2026-10-11 / 10-12 (``catch_population_trials --collect-only``
over those units). They replace the #798 sets (D-1 · D-2 · candidate box, in
the git history), and what that test file held is held here for the new ones:

- every file is a ``catching_throw_list/1`` the sim driver and the offline map
  read as is (the shared loader validates it — ids unique, vectors finite), and
  the file on disk is the document the loader read;
- it names its robot, its arm and its size;
- every throw carries its labels — here the seeds it was thrown and caught on,
  and the design axes the report bins by;
- no throw is in a set twice. The #798 sets of a robot shared no throw at all;
  two ARMS' catch sets are drawn from ONE population and may share a throw, so
  what is asked is that a shared id is the same launch.

A catch set is one run's outcome (``meta.runs``): the meta must say what was
thrown, of which set, and what each seed did with it, so that "not in the set"
is not read as "missed".

The sets and the robot's ``catch_population.yaml`` ship side by side: a set must be a
set OF the population that file defines (its grid, its limits, its arms, its sample),
or the file says one thing and the data beside it another.
"""

from __future__ import annotations

import json
import os
import re

import pytest

from rtc_tools.analysis import catching_throw_list as ctl
from rtc_tools.utils.catch_population import axis_values, load_config

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")
# robot → arm → (throws caught, throws thrown, throws of the set that was sampled)
SETS = {
    "ur5e_p1b": {"grid_mpc": (89, 890, 890), "nlp_mpc_docking": (32, 1110, 1110)},
    "iiwa7_leap": {"grid_mpc": (451, 1000, 4212), "nlp_mpc_docking": (30, 87, 87)},
}
ROBOTS = tuple(SETS)
ARMS = {"grid_mpc": ("grid", "mpc"), "nlp_mpc_docking": ("nlp", "mpc_docking")}
DESIGN_AXES = (
    "distance_m",
    "azimuth_deg",
    "release_height_m",
    "pass_dx_m",
    "pass_dy_m",
    "elevation_deg",
)
SEEDS = [8210]


def _path(robot, arm):
    return os.path.join(CONFIG, robot, "throw_sets", f"throw_catch_set_{arm}.json")


def _sets(robot):
    out = {}
    for arm in SETS[robot]:
        assert os.path.isfile(_path(robot, arm)), _path(robot, arm)
        out[arm] = ctl.read_throw_list_file(_path(robot, arm))
    return out


def _launch(throw):
    return tuple(round(float(v), 9) for v in (*throw["pos"], *throw["vel"]))


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_folder_holds_exactly_the_catch_sets_of_the_robots_arms(robot):
    folder = os.path.join(CONFIG, robot, "throw_sets")
    assert sorted(os.listdir(folder)) == sorted(
        f"throw_catch_set_{arm}.json" for arm in SETS[robot]
    )


@pytest.mark.parametrize("robot", ROBOTS)
def test_every_set_is_a_valid_list_of_its_robot_and_arm_with_the_expected_size(robot):
    for arm, listed in _sets(robot).items():
        caught, _, _ = SETS[robot][arm]
        search, segment = ARMS[arm]
        meta = listed.meta
        assert meta["robot"] == robot, arm
        assert (meta["kind"], meta["set_name"], meta["tier"]) == (
            "catch_set",
            "throw_catch_set",
            "catch",
        ), arm
        assert meta["arm"] == {"name": arm, "search": search, "segment": segment}
        assert len(listed.throws) == caught, (arm, len(listed.throws))
        assert len({t["throw_id"] for t in listed.throws}) == caught, arm


@pytest.mark.parametrize("robot", ROBOTS)
def test_every_throw_carries_the_seeds_it_was_caught_on_and_its_design_axes(robot):
    for arm, listed in _sets(robot).items():
        for t in listed.throws:
            assert t["caught_seeds"], (arm, t["throw_id"])
            assert set(t["caught_seeds"]) <= set(t["thrown_seeds"]) <= set(SEEDS)
            for axis in DESIGN_AXES:
                assert isinstance(t[axis], float), (arm, t["throw_id"], axis)
            # a throw of a set is a throw of the population: launched, not limited
            assert (t["screen"], t["limit"]) == ("", ""), (arm, t["throw_id"])


@pytest.mark.parametrize("robot", ROBOTS)
def test_no_throw_is_in_a_set_twice_and_a_throw_of_two_arms_is_one_launch(robot):
    sets = _sets(robot)
    by_id: dict[int, tuple] = {}
    for arm, listed in sets.items():
        launches = [_launch(t) for t in listed.throws]
        assert len(set(launches)) == len(launches), f"{arm} throws a launch twice"
        for t in listed.throws:
            # The arms' sets are narrowed from one population: an id means one launch.
            assert by_id.setdefault(t["throw_id"], _launch(t)) == _launch(t), (arm, t["throw_id"])


@pytest.mark.parametrize("robot", ROBOTS)
def test_the_meta_says_what_was_thrown_of_which_set_and_what_each_seed_caught(robot):
    for arm, listed in _sets(robot).items():
        caught, thrown, of = SETS[robot][arm]
        meta = listed.meta
        # the set that was thrown, and how much of it: the merge key's first half
        assert meta["thrown_set"] == "segment"
        assert meta["source_set"]["set_name"] == "throw_segment_set"
        assert meta["source_set"]["n_throws"] == of
        assert re.fullmatch(r"[0-9a-f]{64}", meta["source_set"]["sha256"]), arm
        assert meta["merge_key"] == ["source_set.sha256", "throw_id"]
        sample = meta["sample"]
        assert (sample["n_throws"], sample["of"], sample["whole_set"]) == (
            thrown,
            of,
            thrown == of,
        )
        # one seed, every chunk thrown, nothing invalid
        (seed,) = meta["seeds"]
        assert (seed["seed"], seed["thrown"], seed["invalid"], seed["caught"]) == (
            SEEDS[0],
            thrown,
            0,
            caught,
        ), arm
        assert caught <= seed["pair_published"] <= thrown
        assert len(seed["chunks"]) == meta["chunks_total"]
        assert meta["counts"] == {"thrown": thrown, "caught": caught}
        # which run each unit was: a catch is one run's outcome
        assert [(run["seed"], run["chunk"]) for run in meta["runs"]] == [
            (SEEDS[0], chunk) for chunk in seed["chunks"]
        ]
        for run in meta["runs"]:
            assert run["rtc_framework_rev"] and run["date_start"] and run["viewer"] == "on", arm
        assert "not being here is not a miss" in meta["note"]


@pytest.mark.parametrize("robot", ROBOTS)
def test_every_set_is_a_set_of_the_population_the_robots_configuration_defines(robot):
    config = load_config(os.path.join(CONFIG, robot, "catch_population.yaml"))
    population = config.population
    # an arm of the configuration has a set, and a set is an arm of the configuration
    assert sorted(arm.name for arm in config.arms) == sorted(SETS[robot])
    axes = {
        "distance_m": axis_values(population.distance_m),
        "azimuth_deg": axis_values(population.azimuth_deg),
        "release_height_m": axis_values(population.release_height_m),
        "pass_dx_m": axis_values(population.pass_dx_m),
        "pass_dy_m": axis_values(population.pass_dy_m),
    }
    for arm, listed in _sets(robot).items():
        meta, design = listed.meta, listed.meta["design"]
        # the grid: the configuration's axes, and its elevation ladder
        grid = design["grid"]["axes"]
        for axis, values in axes.items():
            assert grid[axis] == pytest.approx(values), (arm, axis)
        elevation = design["elevation"]
        assert (elevation["max_deg"], elevation["min_deg"], elevation["step_deg"]) == (
            population.elevation_max_deg,
            population.elevation_min_deg,
            population.elevation_step_deg,
        ), arm
        assert max(grid["elevation_deg"]) == population.elevation_max_deg
        assert min(grid["elevation_deg"]) >= population.elevation_min_deg
        # the limits of the population, the ball and the delay it was judged with
        assert design["speed"]["max_m_s"] == population.speed_max_m_s, arm
        assert meta["limit"]["min_pass_time_s"] == population.min_pass_time_s, arm
        assert design["lead"]["detection_delay_s"] == config.timing.detection_delay_s, arm
        assert design["ball"]["drag_coefficient"] == config.ball.drag_coefficient.value, arm
        assert design["ball"]["air_density_kg_m3"] == config.ball.air_density.value, arm
        # the sample: which set the arm throws, how many at most, drawn how
        configured, sample = config.arm(arm), meta["sample"]
        assert meta["thrown_set"] == configured.throw_set, arm
        assert sample["seed"] == config.sample.seed, arm
        assert sample["strata_axes"] == list(config.sample.strata), arm
        if sample["whole_set"]:
            assert configured.max_throws is None or sample["of"] <= configured.max_throws, arm
        else:
            assert sample["n_throws"] == sample["max_throws"] == configured.max_throws, arm
        # and every throw is a point of that grid
        for t in listed.throws:
            for axis in DESIGN_AXES:
                assert any(abs(t[axis] - v) < 1e-9 for v in grid[axis]), (arm, t["throw_id"], axis)


@pytest.mark.parametrize("robot", ROBOTS)
def test_a_shipped_set_names_no_path_of_the_pc_it_was_judged_on(robot):
    """The ball's sources are relative to the configuration folder (#822): the label
    names a file of this robot's folder, the same from a source tree and an install."""
    for arm, listed in _sets(robot).items():
        with open(_path(robot, arm), encoding="utf-8") as handle:
            assert "/home/" not in handle.read(), arm
        for key in ("radius_m", "mass_kg"):
            label = listed.meta["design"]["ball"]["sources"][key]
            file, _, field = label.rpartition(":")
            assert field == f"projectile_ball.{key}", (arm, label)
            assert file.startswith(f"{robot}/"), (arm, label)
            assert os.path.isfile(os.path.join(CONFIG, file)), (arm, label)


def test_the_shipped_file_is_what_the_loader_reads_bit_for_bit():
    # The pairing key is the file's sha256: the file on disk must be the document the
    # loader validates — no trailing edits, one JSON document, UTF-8.
    for robot in ROBOTS:
        for arm, listed in _sets(robot).items():
            with open(_path(robot, arm), "rb") as f:
                raw = f.read()
            on_disk = json.loads(raw.decode("utf-8"))["throws"]
            assert json.loads(json.dumps(listed.throws)) == on_disk, arm
