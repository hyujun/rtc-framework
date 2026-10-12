"""
catch_population.py — #821: the throw population's configuration file and result folder

One YAML file says what is thrown at a catching robot and how it is narrowed::

    population ─search─▶ throw_search_set ─segment─▶ throw_segment_set ─sim─▶ throw_catch_set
       (design)   (offline judge)            (offline judge)            (viewer on)

Three commands read it — stage 1 (``rtc_tools catch_population``: the design and
the two offline tiers), stage 2 (``integrated_bringup catch_population_trials``:
the sim) and the report — so the file's schema and the layout of the folder they
share live here, in the one module all three import. Nothing here imports numpy,
pinocchio or ROS: stage 2 runs on a PC that has only the installed packages.

**The robot is the folder the file is in** (``<...>/config/<robot>/<file>``): the
controller configuration the arms are judged with is that folder, and ``ball.config``
is a file name inside it. No key names a robot.

The file is read fail-closed (:func:`load_config`): a missing key, an unknown key
and a value of the wrong type or outside its range are each refused by the key's
full path — a typo must not silently fall back to a default that then describes
another population. Schema (every key required unless marked optional)::

    schema: catch_population/1
    arms:                       # search x segment planner: the ONLY two shipped keys an arm changes
      - search: grid            # planner.search.mode: grid | nlp
        segment: mpc            # planner.segment.mode: mpc | mpc_docking
        throw_set: segment      # optional: the set stage 2 throws for this arm (default sample.throw_set)
        max_throws: 1000        # optional: this arm's sample size (default sample.max_throws)
    ball:
      config: mujoco_simulator.yaml             # in the file's folder: the ball's radius and mass
      drag_coefficient: {value: 0.55, source: "<where the number is from>"}
      air_density: {value: 1.204, source: "<...>"}
    timing:                     # the estimator profile's values, copied: stage 1 needs no estimator workspace
      detection_delay_s: 0.15
      vision_period_s: 0.0333333333333
      prediction_horizon_s: 1.0
      source: "<profile name and revision>"
    judge:                      # the offline flight integration
      horizon_s: 2.5
      step_s: 0.002
      min_catch_height_m: 0.0
    population:
      launch: {distance_m: "1.5:4.0:0.5", azimuth_deg: "-40:40:20", release_height_m: "1.0 1.4 1.7"}
      pass: {dx_m: "-0.4:0.4:0.2", dy_m: "-0.4:0.4:0.2"}
      elevation_deg: {max: 70, min: 5, step: 5}
      min_pass_time_s: 0.5
      speed_max_m_s: 12
    sample:                     # which throws of a set stage 2 throws
      throw_set: segment        # search | segment
      max_throws: 2500          # per arm; null = the whole set
      strata: [distance_m, elevation_deg]
      seed: 821
      chunk: 250                # throws per sim bringup
    sim:
      viewer: true
      estimator_seeds: [8210]
      end_on_ball_low: true
      host_watch: abort         # abort | warn | off
      retries: 3                # bringups a chunk may take on a rig failure

An axis (``launch.*``, ``pass.*``) is ``lo:hi:step`` (both ends included) or a
list of values, as text or as a YAML list of numbers.

An arm is named ``<search>_<segment>`` (:attr:`Arm.name`); two arms of one name
are refused. :class:`ResultLayout` is where every stage writes and reads.
"""

from __future__ import annotations

import hashlib
import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import yaml

SCHEMA = "catch_population/1"
_CHUNK_NAME = re.compile(r"chunk_\d{3,}\.json")
SEARCH_MODES = ("grid", "nlp")
# closed_form has no segment planner: there is no segment tier to judge.
SEGMENT_MODES = ("mpc", "mpc_docking")
THROW_SETS = ("search", "segment")
HOST_WATCH = ("abort", "warn", "off")
# Stratification axes a sample may name: the design's six axes.
STRATA_AXES = (
    "distance_m",
    "azimuth_deg",
    "release_height_m",
    "pass_dx_m",
    "pass_dy_m",
    "elevation_deg",
)
SET_NAMES = {
    "search": "throw_search_set",
    "segment": "throw_segment_set",
    "catch": "throw_catch_set",
}


class ConfigError(ValueError):
    """The configuration file is not one this schema reads; the message names the key."""


@dataclass(frozen=True)
class Sourced:
    """A number and where it is from (the offline judge records both)."""

    value: float
    source: str


@dataclass(frozen=True)
class Arm:
    search: str
    segment: str
    throw_set: str  # the set stage 2 throws: "search" | "segment"
    max_throws: int | None  # None = the whole set

    @property
    def name(self) -> str:
        return f"{self.search}_{self.segment}"


@dataclass(frozen=True)
class Ball:
    config: Path  # absolute: the file in the configuration folder
    drag_coefficient: Sourced
    air_density: Sourced


@dataclass(frozen=True)
class Timing:
    detection_delay_s: float
    vision_period_s: float
    prediction_horizon_s: float
    source: str


@dataclass(frozen=True)
class Judge:
    horizon_s: float
    step_s: float
    min_catch_height_m: float


@dataclass(frozen=True)
class Population:
    """The design grid. Each axis is the text the design tool's argument takes."""

    distance_m: str
    azimuth_deg: str
    release_height_m: str
    pass_dx_m: str
    pass_dy_m: str
    elevation_max_deg: float
    elevation_min_deg: float
    elevation_step_deg: float
    min_pass_time_s: float
    speed_max_m_s: float


@dataclass(frozen=True)
class Sample:
    throw_set: str
    max_throws: int | None
    strata: tuple[str, ...]
    seed: int
    chunk: int


@dataclass(frozen=True)
class Sim:
    viewer: bool
    estimator_seeds: tuple[int, ...]
    end_on_ball_low: bool
    host_watch: str
    retries: int


@dataclass(frozen=True)
class PopulationConfig:
    path: Path  # the file, absolute
    sha256: str  # of the file's bytes
    arms: tuple[Arm, ...]
    ball: Ball
    timing: Timing
    judge: Judge
    population: Population
    sample: Sample
    sim: Sim

    @property
    def config_dir(self) -> Path:
        """The robot's configuration folder: the one the file is in."""
        return self.path.parent

    @property
    def robot(self) -> str:
        return self.path.parent.name

    def arm(self, name: str) -> Arm:
        for arm in self.arms:
            if arm.name == name:
                return arm
        raise ConfigError(f"no arm '{name}' (arms: {[a.name for a in self.arms]})")


# ── Reading ──────────────────────────────────────────────────────────────────


def _map(node, path: str, required: Sequence[str], optional: Sequence[str] = ()) -> Mapping:
    if not isinstance(node, Mapping):
        raise ConfigError(f"'{path}' must be a map")
    known = (*required, *optional)
    for key in node:
        if key not in known:
            raise ConfigError(
                f"unknown key '{path + '.' if path else ''}{key}' (known: {', '.join(known)})"
            )
    for key in required:
        if key not in node:
            raise ConfigError(f"'{path + '.' if path else ''}{key}' is required")
    return node


def _number(node, path: str, *, low: float | None = None, above: float | None = None) -> float:
    # bool is an int subclass: `true` where a number belongs is a mistake, not 1.
    if isinstance(node, bool) or not isinstance(node, int | float) or not math.isfinite(node):
        raise ConfigError(f"'{path}' must be a finite number, got {node!r}")
    value = float(node)
    if low is not None and value < low:
        raise ConfigError(f"'{path}' must be >= {low:g}, got {value:g}")
    if above is not None and not value > above:
        raise ConfigError(f"'{path}' must be > {above:g}, got {value:g}")
    return value


def _integer(node, path: str, *, low: int) -> int:
    if isinstance(node, bool) or not isinstance(node, int):
        raise ConfigError(f"'{path}' must be an integer, got {node!r}")
    if node < low:
        raise ConfigError(f"'{path}' must be >= {low}, got {node}")
    return int(node)


def _bool(node, path: str) -> bool:
    if not isinstance(node, bool):
        raise ConfigError(f"'{path}' must be true or false, got {node!r}")
    return node


def _text(node, path: str) -> str:
    if not isinstance(node, str) or not node.strip():
        raise ConfigError(f"'{path}' must be a non-empty string, got {node!r}")
    return node.strip()


def _choice(node, path: str, choices: Sequence[str]) -> str:
    if not isinstance(node, str) or node not in choices:
        raise ConfigError(f"'{path}' must be one of {', '.join(choices)}, got {node!r}")
    return node


def _optional_count(node, path: str) -> int | None:
    return None if node is None else _integer(node, path, low=1)


# Axis values are compared as rounded numbers (a range's ``lo + i * step`` is not exact).
AXIS_DECIMALS = 9


def axis_values(text: str) -> list[float]:
    """An axis from its argument: ``lo:hi:step`` (both ends included) or a list of values."""
    if ":" in text:
        parts = text.split(":")
        if len(parts) != 3:
            raise ValueError(f"'{text}': a range is lo:hi:step")
        lo, hi, step = (float(p) for p in parts)
        if not step > 0.0 or hi < lo:
            raise ValueError(f"'{text}': the step must be > 0 and hi >= lo")
        count = round((hi - lo) / step)
        if abs(lo + count * step - hi) > 10.0**-AXIS_DECIMALS:
            raise ValueError(f"'{text}': hi is not a whole number of steps from lo")
        return [round(lo + i * step, AXIS_DECIMALS) for i in range(count + 1)]
    return [float(v) for v in text.replace(",", " ").split()]


def _axis(node, path: str) -> str:
    """An axis as the design tool's argument text; a YAML list of numbers is joined."""
    if isinstance(node, list):
        if not node or any(isinstance(v, bool) or not isinstance(v, int | float) for v in node):
            raise ConfigError(f"'{path}' must be a non-empty list of numbers, got {node!r}")
        text = " ".join(repr(float(v)) for v in node)
    elif isinstance(node, str):
        text = node.strip()
    elif isinstance(node, int | float) and not isinstance(node, bool):
        text = repr(float(node))
    else:
        raise ConfigError(f"'{path}' must be 'lo:hi:step' or a list of values, got {node!r}")
    try:
        values = axis_values(text)
    except ValueError as exc:
        raise ConfigError(f"'{path}': {exc}") from exc
    if not values or not all(math.isfinite(v) for v in values):
        raise ConfigError(f"'{path}': '{text}' is not a list of finite values")
    return text


def _sourced(node, path: str, *, above: float) -> Sourced:
    node = _map(node, path, ("value", "source"))
    return Sourced(
        _number(node["value"], f"{path}.value", above=above),
        _text(node["source"], f"{path}.source"),
    )


def parse_config(document, path: Path) -> PopulationConfig:
    """``document`` (the parsed YAML of the file at ``path``) as a configuration.
    :class:`ConfigError` names the first key that is wrong."""
    path = Path(path).resolve()
    root = _map(
        document,
        "",
        ("schema", "arms", "ball", "timing", "judge", "population", "sample", "sim"),
    )
    if root["schema"] != SCHEMA:
        raise ConfigError(f"'schema' must be '{SCHEMA}', got {root['schema']!r}")

    sample_node = _map(
        root["sample"], "sample", ("throw_set", "max_throws", "strata", "seed", "chunk")
    )
    strata = sample_node["strata"]
    if not isinstance(strata, list) or not strata:
        raise ConfigError("'sample.strata' must be a non-empty list of axes")
    for i, axis in enumerate(strata):
        _choice(axis, f"sample.strata[{i}]", STRATA_AXES)
    if len(set(strata)) != len(strata):
        raise ConfigError(f"'sample.strata' names an axis twice: {strata}")
    sample = Sample(
        throw_set=_choice(sample_node["throw_set"], "sample.throw_set", THROW_SETS),
        max_throws=_optional_count(sample_node["max_throws"], "sample.max_throws"),
        strata=tuple(strata),
        seed=_integer(sample_node["seed"], "sample.seed", low=0),
        chunk=_integer(sample_node["chunk"], "sample.chunk", low=1),
    )

    if not isinstance(root["arms"], list) or not root["arms"]:
        raise ConfigError("'arms' must be a non-empty list")
    arms = []
    for i, node in enumerate(root["arms"]):
        where = f"arms[{i}]"
        node = _map(node, where, ("search", "segment"), ("throw_set", "max_throws"))
        arm = Arm(
            search=_choice(node["search"], f"{where}.search", SEARCH_MODES),
            segment=_choice(node["segment"], f"{where}.segment", SEGMENT_MODES),
            throw_set=_choice(node["throw_set"], f"{where}.throw_set", THROW_SETS)
            if "throw_set" in node
            else sample.throw_set,
            max_throws=_optional_count(node["max_throws"], f"{where}.max_throws")
            if "max_throws" in node
            else sample.max_throws,
        )
        if any(arm.name == other.name for other in arms):
            raise ConfigError(f"'{where}' repeats the arm {arm.name}")
        arms.append(arm)

    ball_node = _map(root["ball"], "ball", ("config", "drag_coefficient", "air_density"))
    ball_name = _text(ball_node["config"], "ball.config")
    if Path(ball_name).name != ball_name:
        raise ConfigError(
            f"'ball.config' is a file name in the configuration folder, got '{ball_name}'"
        )
    ball = Ball(
        config=path.parent / ball_name,
        drag_coefficient=_sourced(ball_node["drag_coefficient"], "ball.drag_coefficient", above=0),
        air_density=_sourced(ball_node["air_density"], "ball.air_density", above=0),
    )

    timing_node = _map(
        root["timing"],
        "timing",
        ("detection_delay_s", "vision_period_s", "prediction_horizon_s", "source"),
    )
    timing = Timing(
        detection_delay_s=_number(
            timing_node["detection_delay_s"], "timing.detection_delay_s", low=0
        ),
        vision_period_s=_number(timing_node["vision_period_s"], "timing.vision_period_s", above=0),
        prediction_horizon_s=_number(
            timing_node["prediction_horizon_s"], "timing.prediction_horizon_s", above=0
        ),
        source=_text(timing_node["source"], "timing.source"),
    )

    judge_node = _map(root["judge"], "judge", ("horizon_s", "step_s", "min_catch_height_m"))
    judge = Judge(
        horizon_s=_number(judge_node["horizon_s"], "judge.horizon_s", above=0),
        step_s=_number(judge_node["step_s"], "judge.step_s", above=0),
        min_catch_height_m=_number(judge_node["min_catch_height_m"], "judge.min_catch_height_m"),
    )

    pop = _map(
        root["population"],
        "population",
        ("launch", "pass", "elevation_deg", "min_pass_time_s", "speed_max_m_s"),
    )
    launch = _map(
        pop["launch"], "population.launch", ("distance_m", "azimuth_deg", "release_height_m")
    )
    passing = _map(pop["pass"], "population.pass", ("dx_m", "dy_m"))
    elevation = _map(pop["elevation_deg"], "population.elevation_deg", ("max", "min", "step"))
    e_max = _number(elevation["max"], "population.elevation_deg.max")
    e_min = _number(elevation["min"], "population.elevation_deg.min")
    if not -90.0 < e_min <= e_max < 90.0:
        raise ConfigError(
            "'population.elevation_deg' needs -90 < min <= max < 90, got "
            f"min {e_min:g}, max {e_max:g}"
        )
    population = Population(
        distance_m=_axis(launch["distance_m"], "population.launch.distance_m"),
        azimuth_deg=_axis(launch["azimuth_deg"], "population.launch.azimuth_deg"),
        release_height_m=_axis(launch["release_height_m"], "population.launch.release_height_m"),
        pass_dx_m=_axis(passing["dx_m"], "population.pass.dx_m"),
        pass_dy_m=_axis(passing["dy_m"], "population.pass.dy_m"),
        elevation_max_deg=e_max,
        elevation_min_deg=e_min,
        elevation_step_deg=_number(elevation["step"], "population.elevation_deg.step", above=0),
        min_pass_time_s=_number(pop["min_pass_time_s"], "population.min_pass_time_s", low=0),
        speed_max_m_s=_number(pop["speed_max_m_s"], "population.speed_max_m_s", above=0),
    )

    sim_node = _map(
        root["sim"],
        "sim",
        ("viewer", "estimator_seeds", "end_on_ball_low", "host_watch", "retries"),
    )
    seeds = sim_node["estimator_seeds"]
    if not isinstance(seeds, list) or not seeds:
        raise ConfigError("'sim.estimator_seeds' must be a non-empty list of integers")
    seeds = [_integer(s, f"sim.estimator_seeds[{i}]", low=0) for i, s in enumerate(seeds)]
    if len(set(seeds)) != len(seeds):
        raise ConfigError(f"'sim.estimator_seeds' repeats a seed: {seeds}")
    sim = Sim(
        viewer=_bool(sim_node["viewer"], "sim.viewer"),
        estimator_seeds=tuple(seeds),
        end_on_ball_low=_bool(sim_node["end_on_ball_low"], "sim.end_on_ball_low"),
        host_watch=_choice(sim_node["host_watch"], "sim.host_watch", HOST_WATCH),
        retries=_integer(sim_node["retries"], "sim.retries", low=1),
    )

    return PopulationConfig(
        path=path,
        sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else "",
        arms=tuple(arms),
        ball=ball,
        timing=timing,
        judge=judge,
        population=population,
        sample=sample,
        sim=sim,
    )


def load_config(path: Path | str) -> PopulationConfig:
    """The configuration file at ``path``. :class:`ConfigError` for a file that cannot be
    read, is not YAML, or is not this schema; the ball's file must exist beside it."""
    path = Path(path).resolve()
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ConfigError(f"cannot read {path}: {exc.strerror or exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path} is not YAML: {exc}") from exc
    config = parse_config(document, path)
    if not config.ball.config.is_file():
        raise ConfigError(f"'ball.config': no file {config.ball.config}")
    return config


# ── The result folder ────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ResultLayout:
    """Where the stages write and read under one result folder::

    <root>/config.yaml                 the configuration file as it was read (a copy)
    <root>/manifest.json               what was judged with what (hashes, revision if known)
    <root>/design/                     the design (catch_throw_design design)
    <root>/population.json             the population: a catching_throw_list/1
    <root>/<arm>/map/                  the offline judge's output (catch_search_map)
    <root>/<arm>/throw_search_set.json
    <root>/<arm>/throw_segment_set.json
    <root>/<arm>/throws/               what stage 2 throws: sample.json + chunk_NNN.json
    <root>/<arm>/throw_catch_set.json  stage 2: the thrown throws the sim caught
    <root>/<arm>/sim_throws.csv        stage 2: one row per thrown throw and seed
    <root>/sim/<arm>/seed_<seed>/chunk_NNN/   one sim bringup (a unit)
    <root>/sim/progress.log            stage 2: one line per unit started and ended
    <root>/STOP                        stage 2: its presence ends the run before the next unit
    <root>/report/                     statistics text and 3D plots
    """

    root: Path

    @property
    def config(self) -> Path:
        return self.root / "config.yaml"

    @property
    def manifest(self) -> Path:
        return self.root / "manifest.json"

    @property
    def design(self) -> Path:
        return self.root / "design"

    @property
    def population(self) -> Path:
        return self.root / "population.json"

    @property
    def report(self) -> Path:
        return self.root / "report"

    def arm(self, arm: str) -> Path:
        return self.root / arm

    def map_dir(self, arm: str) -> Path:
        return self.arm(arm) / "map"

    def set_file(self, arm: str, tier: str) -> Path:
        """``tier``: search | segment | catch."""
        return self.arm(arm) / f"{SET_NAMES[tier]}.json"

    def throws_dir(self, arm: str) -> Path:
        return self.arm(arm) / "throws"

    def chunks(self, arm: str) -> list[Path]:
        """The chunk files stage 2 throws for ``arm``, in order."""
        # The sampler pads the index to three digits, and to more past 999 chunks.
        return sorted(
            (
                p
                for p in self.throws_dir(arm).glob("chunk_*.json")
                if _CHUNK_NAME.fullmatch(p.name)
            ),
            key=lambda p: int(p.stem[len("chunk_") :]),
        )

    def sim_throws(self, arm: str) -> Path:
        return self.arm(arm) / "sim_throws.csv"

    @property
    def sim_root(self) -> Path:
        return self.root / "sim"

    @property
    def stop_file(self) -> Path:
        return self.root / "STOP"

    def unit_dir(self, arm: str, seed: int, chunk: str) -> Path:
        """``chunk``: a chunk file's stem (``chunk_003``)."""
        return self.sim_root / arm / f"seed_{seed}" / chunk
