#!/usr/bin/env python3
"""Stage 1 of a throw population: the design and the two offline tiers, from one file (#821).

One configuration file (``rtc_tools.utils.catch_population``: its schema and the
result folder's layout) says what is thrown at a catching robot and how it is
narrowed. This command runs everything of it that needs no simulator::

    design ─limit─▶ population ─search─▶ search set ─segment─▶ segment set ─sample─▶ throws

    catch_population --config <catch_population.yaml> --out <result folder> [--jobs N]

It judges nothing itself and re-implements no step: each one is the existing
tool, called through its ``main`` with the arguments the file gives.

1. ``catch_throw_design design`` — the grid, the launch speeds, the screens and
   the pass-time limit → ``design/``;
2. ``catch_throw_design select`` — the launchable, unlimited throws: the
   POPULATION → ``population.json``;
3. per arm (a search and a segment planner — the only two values of the
   robot's shipped configuration an arm changes):

   - ``catch_search_map --search <search> --segment-mode <segment>
     --judge-segments`` on the population, ONE run: the search pass, then the
     whole planner wake on the throws it accepted → ``<arm>/map/``;
   - ``catch_throw_design plan-set --tier search`` / ``--tier segment`` →
     ``<arm>/throw_search_set.json`` / ``<arm>/throw_segment_set.json``;
   - ``catch_throw_design sample`` of the set the arm throws, by the file's
     strata, seed and chunk → ``<arm>/throws/`` (what stage 2 throws).

Then ``manifest.json`` says what was judged with what, and
``catch_population_report`` writes ``report/`` and prints the statistics.

**The robot is the folder the file is in**: the controller configuration judged
is that folder's. No robot is named here.

**Resumable.** A step whose product is in the folder is skipped, so a run that
was interrupted is continued by the same command line. The folder belongs to
one configuration: its ``config.yaml`` is a copy of the file, and a file of
other bytes is refused — the sets of two configurations in one folder would read
as one population. Use a new ``--out``.

**An empty tier is a result.** When the judge accepts no throw of a tier there
is no set file and nothing to throw for that arm; a search or a segment planner
that refuses its configuration judged nothing. Both are recorded in the
manifest and said in the report, and the other arms go on.

**The manifest** (``manifest.json``): the configuration file's sha256 and where
it was read from, and per arm the sets' sizes and sha256s, the sample's, and
what the map's own summary records of its provenance (the judge's and the
composed tree's hash, the layers, the override). The git revision is written
only where it can be read — a PC with the installed packages alone has no
checkout. Keys of the manifest this command does not write are kept.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import sys
import time
from collections.abc import Callable, Sequence
from pathlib import Path

from rtc_tools.analysis import catch_search_map as csm, catch_throw_design as ctd
from rtc_tools.analysis.catch_population_report import (
    DEFAULT_MAX_FLIGHTS,
    MAP_SUMMARY,
    OFFLINE_TIERS,
    read_map_summary,
    report,
    tier_state,
)
from rtc_tools.analysis.catching_throw_list import read_throw_list_file
from rtc_tools.utils.catch_population import (
    Arm,
    ConfigError,
    PopulationConfig,
    ResultLayout,
    load_config,
)

MANIFEST_SCHEMA = "catch_population_manifest/1"
# The manifest's keys this command writes; any other key is another stage's and is kept.
_MANIFEST_KEYS = ("schema", "tool", "config", "repo", "design", "arms")
# Of a map's provenance, what the manifest repeats.
_PROVENANCE_KEYS = ("catching_tree_sha256", "judge", "overrides", "segments", "layers", "repo")


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _number(value: float) -> str:
    return repr(float(value))


def _say(text: str) -> None:
    print(f"[catch_population] {text}", flush=True)


def run_tool(step: str, tool: Callable[[list[str]], int], argv: Sequence[str]) -> None:
    """One step: ``tool`` (a tool's ``main``) on ``argv``. A tool that exits says why;
    the step's name is put in front of it."""
    _say(step)
    started = time.monotonic()
    try:
        code = tool([str(a) for a in argv])
    except SystemExit as exc:
        if isinstance(exc.code, str):
            raise SystemExit(f"{step}: {exc.code}") from exc
        raise
    if code:
        raise SystemExit(f"{step}: exited {code}")
    _say(f"{step}: done in {time.monotonic() - started:.1f} s")


# ── The arguments of each tool, from the configuration ────────────────────────


def setup_arguments(config: PopulationConfig) -> list[str]:
    """What ``catch_search_map.add_setup_arguments`` reads: the profile and the ball."""
    ball = config.ball
    return [
        "--config-dir",
        str(config.config_dir),
        "--ball-config",
        str(ball.config),
        "--drag-coefficient",
        _number(ball.drag_coefficient.value),
        "--drag-coefficient-source",
        ball.drag_coefficient.source,
        "--air-density",
        _number(ball.air_density.value),
        "--air-density-source",
        ball.air_density.source,
    ]


def design_arguments(config: PopulationConfig, layout: ResultLayout, jobs: int) -> list[str]:
    population = config.population
    return [
        "design",
        *setup_arguments(config),
        "--out-dir",
        str(layout.design),
        # One token each: an axis may start with a minus sign.
        *[
            f"--{ctd._AXIS_ARGUMENT[axis]}={getattr(population, axis)}"
            for axis in ctd._AXIS_ARGUMENT
        ],
        "--max-elevation-deg",
        _number(population.elevation_max_deg),
        "--min-elevation-deg",
        _number(population.elevation_min_deg),
        "--elevation-step-deg",
        _number(population.elevation_step_deg),
        "--min-pass-time-s",
        _number(population.min_pass_time_s),
        "--speed-max-m-s",
        _number(population.speed_max_m_s),
        "--detection-delay-s",
        _number(config.timing.detection_delay_s),
        "--horizon-s",
        _number(config.judge.horizon_s),
        "--step-s",
        _number(config.judge.step_s),
        "--jobs",
        str(jobs),
    ]


def map_arguments(
    config: PopulationConfig, layout: ResultLayout, arm: Arm, jobs: int
) -> list[str]:
    """One run for both tiers: the search pass on the population, and with
    ``--judge-segments`` the whole planner wake on the throws it accepted."""
    timing, judge = config.timing, config.judge
    return [
        *setup_arguments(config),
        "--search",
        arm.search,
        "--segment-mode",
        arm.segment,
        "--judge-segments",
        "--out-dir",
        str(layout.map_dir(arm.name)),
        "--throws-file",
        str(layout.population),
        "--detection-delay-s",
        _number(timing.detection_delay_s),
        "--vision-period-s",
        _number(timing.vision_period_s),
        "--prediction-horizon-s",
        _number(timing.prediction_horizon_s),
        "--horizon-s",
        _number(judge.horizon_s),
        "--step-s",
        _number(judge.step_s),
        f"--min-catch-height-m={_number(judge.min_catch_height_m)}",
        "--jobs",
        str(jobs),
    ]


def sample_arguments(config: PopulationConfig, layout: ResultLayout, arm: Arm) -> list[str]:
    sample = config.sample
    argv = [
        "sample",
        "--plan-set",
        str(layout.set_file(arm.name, arm.throw_set)),
        "--out-dir",
        str(layout.throws_dir(arm.name)),
        "--strata",
        *sample.strata,
        "--seed",
        str(sample.seed),
        "--chunk",
        str(sample.chunk),
    ]
    if arm.max_throws is not None:
        argv += ["--max-throws", str(arm.max_throws)]
    return argv


# ── The steps ─────────────────────────────────────────────────────────────────


def claim_folder(config: PopulationConfig, layout: ResultLayout) -> None:
    """Make ``layout.root`` this configuration's result folder, or refuse it.

    A new or empty folder gets a copy of the file. A folder that has a copy must have
    the same bytes; a folder with other content and no copy is not a result folder.
    """
    if layout.config.is_file():
        held = _sha256(layout.config)
        if held != config.sha256:
            raise SystemExit(
                f"{layout.root} holds the results of another configuration (its "
                f"{layout.config.name} has sha256 {held[:12]}, {config.path} has "
                f"{config.sha256[:12]}): use a new --out"
            )
        return
    if layout.root.exists() and any(layout.root.iterdir()):
        raise SystemExit(
            f"{layout.root} is not empty and has no {layout.config.name}: it is not a result "
            "folder of this command — use a new --out"
        )
    layout.root.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(config.path, layout.config)


def sample_is_complete(layout: ResultLayout, arm: Arm) -> bool:
    """Whether ``<arm>/throws/`` holds a whole sample: ``sample.json`` and every chunk
    file it was cut into (``sample`` writes the list first, then the chunks)."""
    path = layout.throws_dir(arm.name) / "sample.json"
    if not path.is_file():
        return False
    try:
        meta = json.loads(path.read_text())["meta"]
        expected = math.ceil(int(meta["n_throws"]) / int(meta["chunk"]))
    except (ValueError, KeyError, TypeError):
        return False
    return len(list(path.parent.glob("chunk_*.json"))) == expected


def judge_arm(
    config: PopulationConfig, layout: ResultLayout, arm: Arm, jobs: int, judge: Sequence[str]
) -> None:
    """The offline tiers of one arm and the throws stage 2 throws for it."""
    name = arm.name
    if read_map_summary(layout, arm) is None:
        run_tool(
            f"{name}: search and segment tiers of the population",
            csm.main,
            [*map_arguments(config, layout, arm, jobs), *judge],
        )
    else:
        _say(f"{name}: map exists ({layout.map_dir(name) / MAP_SUMMARY}), skipped")
    summary = read_map_summary(layout, arm)
    for tier in OFFLINE_TIERS:
        state = tier_state(layout, arm, tier, summary)
        if state.path is not None:
            _say(f"{name}: {tier} set exists ({state.count} throws), skipped")
        elif state.count == 0:
            _say(f"{name}: the {tier} set is EMPTY — the judge accepted no throw of it")
        elif not state.pending:
            # A refused configuration: nothing was judged, so there is no set to write.
            _say(f"{name}: no {tier} set — {state.why}")
        else:
            run_tool(
                f"{name}: {tier} set",
                ctd.main,
                [
                    "plan-set",
                    "--tier",
                    tier,
                    "--throws-file",
                    layout.population,
                    "--verdicts",
                    layout.map_dir(name) / f"verdicts_{arm.search}.csv",
                    "--out",
                    layout.set_file(name, tier),
                ],
            )
    chosen = tier_state(layout, arm, arm.throw_set, summary)
    if chosen.path is None:
        _say(f"{name}: nothing to throw — there is no {arm.throw_set} set with a throw")
        return
    if sample_is_complete(layout, arm):
        _say(f"{name}: throws exist ({layout.throws_dir(name)}), skipped")
        return
    # What an interrupted sample left: `sample` writes into an empty directory only.
    throws_dir = layout.throws_dir(name)
    if throws_dir.is_dir():
        for stale in [throws_dir / "sample.json", *throws_dir.glob("chunk_*.json")]:
            stale.unlink(missing_ok=True)
    run_tool(f"{name}: throws to throw", ctd.main, sample_arguments(config, layout, arm))


# ── The manifest ──────────────────────────────────────────────────────────────


def _file_record(path: Path, throws: int) -> dict:
    return {"file": path.name, "throws": throws, "sha256": _sha256(path)}


def arm_manifest(layout: ResultLayout, arm: Arm) -> dict:
    """What the folder holds of one arm: its sets (or why there is none), the throws
    to throw, and the provenance the map's summary records."""
    entry: dict = {
        "search": arm.search,
        "segment": arm.segment,
        "throw_set": arm.throw_set,
        "max_throws": arm.max_throws,
    }
    summary = read_map_summary(layout, arm)
    entry["judged"] = summary is not None
    if summary is None:
        return entry
    provenance = summary.get("provenance") or {}
    entry["provenance"] = {
        key: provenance[key] for key in _PROVENANCE_KEYS if provenance.get(key) is not None
    }
    entry["sets"] = {}
    for tier in OFFLINE_TIERS:
        state = tier_state(layout, arm, tier, summary)
        if state.path is not None:
            entry["sets"][tier] = _file_record(state.path, state.count)
        elif state.count == 0:
            entry["sets"][tier] = {"throws": 0, "empty": "the judge accepted no throw"}
        else:
            entry["sets"][tier] = {"throws": None, "no_set": state.why}
    sample = layout.throws_dir(arm.name) / "sample.json"
    if sample.is_file():
        meta = json.loads(sample.read_text())["meta"]
        entry["throws"] = {
            **_file_record(sample, int(meta["n_throws"])),
            "of_set": arm.throw_set,
            "set_sha256": meta["plan_set_sha256"],
            "chunks": [p.name for p in sorted(sample.parent.glob("chunk_*.json"))],
        }
    else:
        entry["throws"] = None
    return entry


def write_manifest(config: PopulationConfig, layout: ResultLayout) -> dict:
    """Write ``manifest.json`` from what the folder holds now, and return it."""
    kept = {}
    if layout.manifest.is_file():
        previous = json.loads(layout.manifest.read_text())
        kept = {k: v for k, v in previous.items() if k not in _MANIFEST_KEYS}
    design_meta = json.loads((layout.design / "design_meta.json").read_text())
    design_provenance = design_meta.get("provenance") or {}
    manifest: dict = {
        "schema": MANIFEST_SCHEMA,
        "tool": "rtc_tools.analysis.catch_population",
        "config": {
            "source": str(config.path),
            "robot": config.robot,
            "sha256": config.sha256,
            "copy": layout.config.name,
        },
        "design": {
            "design_points": design_meta["grid"]["throws"],
            "population": _file_record(
                layout.population, len(read_throw_list_file(layout.population).throws)
            ),
            "provenance": {
                key: design_provenance[key]
                for key in _PROVENANCE_KEYS
                if design_provenance.get(key) is not None
            },
        },
        "arms": {arm.name: arm_manifest(layout, arm) for arm in config.arms},
        **kept,
    }
    # The checkout this run was made from: only where there is one.
    repo = csm.repo_state()
    if repo is not None:
        manifest["repo"] = repo
    layout.manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


# ── CLI ───────────────────────────────────────────────────────────────────────


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument(
        "--config",
        type=Path,
        help="the population's configuration file, in the robot's configuration folder "
        "(required unless --report-only)",
    )
    ap.add_argument("--out", type=Path, required=True, help="the result folder")
    ap.add_argument("--jobs", type=int, default=1, help="processes of the design and the maps")
    ap.add_argument("--judge", type=Path, help=f"path to {csm.SEARCH_EXECUTABLE} (default: ament)")
    ap.add_argument(
        "--arms",
        nargs="+",
        metavar="ARM",
        help="judge these arms only (<search>_<segment>; default: every arm of the file)",
    )
    ap.add_argument(
        "--report-only",
        action="store_true",
        help="write the report of the folder as it is; nothing is judged",
    )
    ap.add_argument("--show", action="store_true", help="also open the report's plots in windows")
    ap.add_argument(
        "--max-flights", type=int, default=DEFAULT_MAX_FLIGHTS, help="flights drawn per set"
    )
    args = ap.parse_args(argv)
    if args.jobs < 1:
        raise SystemExit("--jobs must be >= 1")
    layout = ResultLayout(Path(args.out).resolve())
    if args.config is None and not args.report_only:
        raise SystemExit("--config is required (only --report-only reads the folder's own copy)")
    config, arms = None, []
    if args.config is not None:
        try:
            config = load_config(args.config)
            arms = [config.arm(name) for name in args.arms] if args.arms else list(config.arms)
        except ConfigError as exc:
            raise SystemExit(f"{args.config}: {exc}") from exc
    if args.report_only:
        if config is not None and layout.config.is_file():
            claim_folder(config, layout)  # the file given must be the folder's
        return report(layout.root, show=args.show, max_flights=args.max_flights)

    claim_folder(config, layout)
    judge = ["--judge", str(args.judge)] if args.judge is not None else []
    started = time.monotonic()
    if (layout.design / "design_meta.json").is_file():
        _say(f"design exists ({layout.design}), skipped")
    else:
        run_tool("design", ctd.main, [*design_arguments(config, layout, args.jobs), *judge])
    if layout.population.is_file():
        _say(f"population exists ({layout.population}), skipped")
    else:
        run_tool(
            "population (the design less the unlaunchable and the limited throws)",
            ctd.main,
            ["select", "--design-dir", layout.design, "--out", layout.population],
        )
    for arm in arms:
        judge_arm(config, layout, arm, args.jobs, judge)
    write_manifest(config, layout)
    _say(f"judged in {time.monotonic() - started:.1f} s; manifest: {layout.manifest}")
    return report(layout.root, show=args.show, max_flights=args.max_flights)


if __name__ == "__main__":
    sys.exit(main())
