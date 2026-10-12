"""#821: the throw population's configuration file (rtc_tools.utils.catch_population).

The file is read fail-closed: what these tests pin is that every key is required,
that an unknown key, a wrong type and a value out of range are each refused by the
key's full path, and that the result folder's layout is the one path table the
three commands share. The two shipped files are read at the end.
"""

from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from rtc_tools.utils import catch_population as cp

_REPO = Path(__file__).resolve().parents[2]
_SHIPPED = {
    robot: _REPO / "integrated_bringup" / "config" / robot / "catch_population.yaml"
    for robot in ("ur5e_p1b", "iiwa7_leap")
}

GOOD = {
    "schema": "catch_population/1",
    "arms": [
        {"search": "grid", "segment": "mpc"},
        {"search": "nlp", "segment": "mpc_docking", "throw_set": "search", "max_throws": 40},
    ],
    "ball": {
        "config": "ball.yaml",
        "drag_coefficient": {"value": 0.55, "source": "a"},
        "air_density": {"value": 1.204, "source": "b"},
    },
    "timing": {
        "detection_delay_s": 0.15,
        "vision_period_s": 0.05,
        "prediction_horizon_s": 1.0,
        "source": "a profile",
    },
    "judge": {"horizon_s": 2.5, "step_s": 0.002, "min_catch_height_m": 0.0},
    "population": {
        "launch": {
            "distance_m": "1.5:4.0:0.5",
            "azimuth_deg": [-20, 0, 20],
            "release_height_m": "1.0 1.4",
        },
        "pass": {"dx_m": 0.0, "dy_m": "-0.2:0.2:0.2"},
        "elevation_deg": {"max": 70, "min": 5, "step": 5},
        "min_pass_time_s": 0.5,
        "speed_max_m_s": 12,
    },
    "sample": {
        "throw_set": "segment",
        "max_throws": None,
        "strata": ["distance_m", "elevation_deg"],
        "seed": 821,
        "chunk": 250,
    },
    "sim": {
        "viewer": True,
        "estimator_seeds": [8210, 8211],
        "end_on_ball_low": True,
        "host_watch": "abort",
        "retries": 3,
    },
}


def _write(tmp_path: Path, document, robot: str = "some_robot") -> Path:
    folder = tmp_path / "config" / robot
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "ball.yaml").write_text("{}\n")
    path = folder / "catch_population.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path


def _edited(path: tuple, value) -> dict:
    """GOOD with the node at ``path`` set to ``value`` (``...`` deletes the key)."""
    document = copy.deepcopy(GOOD)
    node = document
    for key in path[:-1]:
        node = node[key]
    if value is ...:
        del node[path[-1]]
    else:
        node[path[-1]] = value
    return document


def _leaf_paths(node, prefix=()):
    if isinstance(node, dict):
        for key, value in node.items():
            yield from _leaf_paths(value, (*prefix, key))
    elif isinstance(node, list) and node and isinstance(node[0], dict):
        for key in node[0]:
            yield (*prefix, 0, key)
    else:
        yield prefix


def _dotted(path: tuple) -> str:
    out = ""
    for key in path:
        out += f"[{key}]" if isinstance(key, int) else ("." if out else "") + str(key)
    return out


def test_a_good_file_reads_as_its_values_and_the_robot_is_its_folder(tmp_path):
    path = _write(tmp_path, GOOD)
    config = cp.load_config(path)
    assert config.path == path.resolve() and config.robot == "some_robot"
    assert config.config_dir == path.parent.resolve()
    assert len(config.sha256) == 64
    assert [a.name for a in config.arms] == ["grid_mpc", "nlp_mpc_docking"]
    # an arm takes the sample's set and size unless it states its own
    assert (config.arms[0].throw_set, config.arms[0].max_throws) == ("segment", None)
    assert (config.arms[1].throw_set, config.arms[1].max_throws) == ("search", 40)
    assert config.arm("nlp_mpc_docking") is config.arms[1]
    with pytest.raises(cp.ConfigError, match="no arm 'grid_nlp'"):
        config.arm("grid_nlp")
    assert config.ball.config == path.parent.resolve() / "ball.yaml"
    assert config.ball.drag_coefficient == cp.Sourced(0.55, "a")
    assert config.timing.vision_period_s == 0.05 and config.timing.source == "a profile"
    assert config.judge == cp.Judge(2.5, 0.002, 0.0)
    pop = config.population
    assert pop.distance_m == "1.5:4.0:0.5"
    # a YAML list and a bare number become the design tool's argument text
    assert cp.axis_values(pop.azimuth_deg) == [-20.0, 0.0, 20.0]
    assert cp.axis_values(pop.pass_dx_m) == [0.0]
    assert (pop.elevation_max_deg, pop.elevation_min_deg, pop.elevation_step_deg) == (70, 5, 5)
    assert config.sample == cp.Sample("segment", None, ("distance_m", "elevation_deg"), 821, 250)
    assert config.sim == cp.Sim(True, (8210, 8211), True, "abort", 3)


def test_every_key_is_required_and_the_refusal_names_it(tmp_path):
    optional = {("arms", 0, "throw_set"), ("arms", 0, "max_throws")}
    paths = set(_leaf_paths(GOOD)) - optional
    assert len(paths) > 30
    for path in sorted(paths, key=_dotted):
        with pytest.raises(
            cp.ConfigError, match=rf"'{_dotted(path)}' is required".replace("[", r"\[")
        ):
            cp.parse_config(_edited(path, ...), tmp_path / "x.yaml")


def test_an_unknown_key_is_refused_at_every_level(tmp_path):
    for parent in [(), ("arms", 0), ("ball",), ("ball", "air_density"), ("timing",), ("judge",),
                   ("population",), ("population", "launch"), ("population", "pass"),
                   ("population", "elevation_deg"), ("sample",), ("sim",)]:  # fmt: skip
        document = _edited((*parent, "robot"), "ur5e")
        name = _dotted((*parent, "robot")).replace("[", r"\[")
        with pytest.raises(cp.ConfigError, match=f"unknown key '{name}'"):
            cp.parse_config(document, tmp_path / "x.yaml")


@pytest.mark.parametrize(
    ("path", "value", "why"),
    [
        (("schema",), "catch_population/2", "'schema' must be 'catch_population/1'"),
        (("arms",), [], "'arms' must be a non-empty list"),
        (("arms", 0, "search"), "random", r"'arms\[0\].search' must be one of grid, nlp"),
        (("arms", 0, "segment"), "closed_form", r"'arms\[0\].segment' must be one of mpc, mpc_d"),
        (("arms", 1, "throw_set"), "catch", r"'arms\[1\].throw_set' must be one of search, seg"),
        (("arms", 1, "max_throws"), 0, r"'arms\[1\].max_throws' must be >= 1"),
        (
            ("arms", 1),
            {"search": "grid", "segment": "mpc"},
            r"'arms\[1\]' repeats the arm grid_mpc",
        ),
        (("ball", "config"), "../other/ball.yaml", "'ball.config' is a file name"),
        (("ball", "drag_coefficient", "value"), 0, "'ball.drag_coefficient.value' must be > 0"),
        (("ball", "air_density", "source"), " ", "'ball.air_density.source' must be a non-empty"),
        (("timing", "detection_delay_s"), -0.1, "'timing.detection_delay_s' must be >= 0"),
        (
            ("timing", "vision_period_s"),
            "fast",
            "'timing.vision_period_s' must be a finite number",
        ),
        (("timing", "prediction_horizon_s"), True, "'timing.prediction_horizon_s' must be a fin"),
        (("judge", "step_s"), 0.0, "'judge.step_s' must be > 0"),
        (
            ("judge", "min_catch_height_m"),
            float("nan"),
            "'judge.min_catch_height_m' must be a fin",
        ),
        (("population", "launch", "distance_m"), "1.5:4.0", "distance_m': '1.5:4.0': a range is"),
        (("population", "launch", "azimuth_deg"), "0:1:0.3", "azimuth_deg': '0:1:0.3': hi is not"),
        (("population", "pass", "dx_m"), [], "'population.pass.dx_m' must be a non-empty list"),
        (("population", "pass", "dy_m"), "a b", "'population.pass.dy_m'"),
        (
            ("population", "elevation_deg", "step"),
            0,
            "'population.elevation_deg.step' must be > 0",
        ),
        (("population", "elevation_deg", "min"), 80, "needs -90 < min <= max < 90"),
        (("population", "min_pass_time_s"), -1, "'population.min_pass_time_s' must be >= 0"),
        (("population", "speed_max_m_s"), 0, "'population.speed_max_m_s' must be > 0"),
        (("sample", "throw_set"), "catch", "'sample.throw_set' must be one of search, segment"),
        (("sample", "max_throws"), 2.5, "'sample.max_throws' must be an integer"),
        (("sample", "strata"), [], "'sample.strata' must be a non-empty list"),
        (("sample", "strata"), ["distance_m", "speed"], r"'sample.strata\[1\]' must be one of"),
        (("sample", "strata"), ["distance_m", "distance_m"], "names an axis twice"),
        (("sample", "seed"), -1, "'sample.seed' must be >= 0"),
        (("sample", "chunk"), 0, "'sample.chunk' must be >= 1"),
        (("sim", "viewer"), "yes", "'sim.viewer' must be true or false"),
        (("sim", "estimator_seeds"), [], "'sim.estimator_seeds' must be a non-empty list"),
        (("sim", "estimator_seeds"), [1, 1], "'sim.estimator_seeds' repeats a seed"),
        (("sim", "estimator_seeds"), [1.5], r"'sim.estimator_seeds\[0\]' must be an integer"),
        (("sim", "host_watch"), "kill", "'sim.host_watch' must be one of abort, warn, off"),
        (("sim", "retries"), 0, "'sim.retries' must be >= 1"),
    ],
)
def test_a_wrong_value_is_refused_by_its_key(tmp_path, path, value, why):
    with pytest.raises(cp.ConfigError, match=why):
        cp.parse_config(_edited(path, value), tmp_path / "x.yaml")


def test_a_file_that_cannot_be_this_schema_is_refused(tmp_path):
    with pytest.raises(cp.ConfigError, match="cannot read"):
        cp.load_config(tmp_path / "none.yaml")
    bad = tmp_path / "bad.yaml"
    bad.write_text("arms: [\n")
    with pytest.raises(cp.ConfigError, match="is not YAML"):
        cp.load_config(bad)
    bad.write_text("- 1\n")
    with pytest.raises(cp.ConfigError, match="'' must be a map"):
        cp.load_config(bad)
    # the ball's file is looked for beside the configuration
    path = _write(tmp_path, GOOD)
    (path.parent / "ball.yaml").unlink()
    with pytest.raises(cp.ConfigError, match="'ball.config': no file"):
        cp.load_config(path)


def test_a_range_that_yaml_reads_as_another_number_is_refused(tmp_path):
    # PyYAML reads an unquoted -40:40:20 as the base-60 integer -146420: one value, finite,
    # an axis the design would run on. The file must say what it means or be refused.
    path = _write(tmp_path, _edited(("population", "launch", "azimuth_deg"), "AXIS"))
    text = path.read_text()
    assert text.count("azimuth_deg: AXIS") == 1
    for unquoted in ("-40:40:20", "1:4:1", "1:30:0.5"):
        path.write_text(text.replace("azimuth_deg: AXIS", f"azimuth_deg: {unquoted}"))
        with pytest.raises(cp.ConfigError, match=f"{unquoted} is read as a base-60 number"):
            cp.load_config(path)
    # quoted, the same text is the range it says; a plain number is still one value
    path.write_text(text.replace("azimuth_deg: AXIS", "azimuth_deg: '-40:40:20'"))
    assert cp.load_config(path).population.azimuth_deg == "-40:40:20"
    path.write_text(text.replace("azimuth_deg: AXIS", "azimuth_deg: 15"))
    assert cp.axis_values(cp.load_config(path).population.azimuth_deg) == [15.0]


def test_a_key_given_twice_is_refused(tmp_path):
    # yaml keeps the last of two equal keys: a block pasted twice would win without a word.
    path = _write(tmp_path, GOOD)
    twice = path.read_text() + yaml.safe_dump({"sample": GOOD["sample"]}, sort_keys=False)
    path.write_text(twice)
    with pytest.raises(cp.ConfigError, match="is not YAML.*the key 'sample' is given twice"):
        cp.load_config(path)


def test_the_axis_parser_is_the_design_tools():
    from rtc_tools.analysis import catch_throw_design as design

    assert design.axis_values is cp.axis_values and design.AXIS_DECIMALS == cp.AXIS_DECIMALS


def test_the_layout_is_one_table_of_paths(tmp_path):
    layout = cp.ResultLayout(tmp_path)
    arm = "grid_mpc"
    assert layout.config == tmp_path / "config.yaml"
    assert layout.manifest == tmp_path / "manifest.json"
    assert layout.design == tmp_path / "design"
    assert layout.population == tmp_path / "population.json"
    assert layout.report == tmp_path / "report"
    assert layout.map_dir(arm) == tmp_path / arm / "map"
    assert layout.set_file(arm, "search") == tmp_path / arm / "throw_search_set.json"
    assert layout.set_file(arm, "segment") == tmp_path / arm / "throw_segment_set.json"
    assert layout.set_file(arm, "catch") == tmp_path / arm / "throw_catch_set.json"
    assert layout.sim_throws(arm) == tmp_path / arm / "sim_throws.csv"
    assert layout.unit_dir(arm, 8210, "chunk_003") == (
        tmp_path / "sim" / arm / "seed_8210" / "chunk_003"
    )
    assert layout.sim_root == tmp_path / "sim" and layout.stop_file == tmp_path / "STOP"
    assert layout.chunks(arm) == []
    throws = layout.throws_dir(arm)
    throws.mkdir(parents=True)
    names = ("chunk_001.json", "chunk_000.json", "sample.json", "chunk_1.json", "chunk_1000.json")
    for name in names:
        (throws / name).write_text("{}")
    assert [p.name for p in layout.chunks(arm)] == [
        "chunk_000.json",
        "chunk_001.json",
        "chunk_1000.json",
    ]


@pytest.mark.parametrize("robot", sorted(_SHIPPED))
def test_the_shipped_file_reads_and_its_arms_are_the_two_pairs(robot):
    path = _SHIPPED[robot]
    if not path.is_file():
        pytest.skip("the integrated_bringup source tree is not beside this package")
    config = cp.load_config(path)
    assert config.robot == robot
    assert [(a.search, a.segment) for a in config.arms] == [
        ("grid", "mpc"),
        ("nlp", "mpc_docking"),
    ]
    assert {a.throw_set for a in config.arms} == {"segment"}
    assert config.sim.viewer is True and config.sim.end_on_ball_low is True
    # the two robots share the population and differ in what is thrown of it
    assert config.population == cp.load_config(_SHIPPED["ur5e_p1b"]).population
