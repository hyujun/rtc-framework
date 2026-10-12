#!/usr/bin/env python3
"""Throw population report: the sets' statistics and 3D plots of a result folder (#821).

``catch_population`` (stage 1: the design and the two offline tiers) and the sim
stage (stage 2) write into one result folder
(``rtc_tools.utils.catch_population.ResultLayout``)::

    population ─search─▶ search set ─segment─▶ segment set ─▶ thrown ─▶ pair published ─▶ caught
       (design)      (offline judge)       (offline judge)      (the sim, per estimator seed)

This reads that folder and writes ``report/``:

- ``population_report.txt`` — the text this prints. The configuration (grid,
  elevation range, limits), the size of every set (design points → without a
  launch / removed by the limit → population → per arm the search set, the
  segment set and the throws to throw), the reject reasons, how the arms' sets
  overlap, and per value of the six design axes and per bin of speed / pass time
  / apex height each set's count and its share of the population there.
- ``set_<name>.png`` — one 3D plot per set: its flights from the launch to the
  pass point, coloured by launch elevation, with the arm at its wait pose.
- ``sets_<arm>.png`` — an arm's sets in one plot, one colour per set.

**Nothing here judges a throw.** Every number is counted from the folder's
files: the sets are the throw lists stage 1 wrote, the reject reasons are the
map summary's own (``catch_search_map.reason_distribution``), the per-axis
shares are ``catch_search_map.acceptance_by_axis`` and the overlap is
``catch_search_map.search_disagreement``, each on "is the throw in the set". A
set that was not judged is written as ``not judged``; a set the judge left empty
is a set of 0 throws.

**With stage 2 results** (``<arm>/sim_throws.csv``: one row per thrown throw and
estimator seed — the columns of a unit's ``catching_trials.csv`` behind ``arm``,
``seed``, ``chunk``) the report adds, per arm and seed, the tiers thrown → pair
published in the sim → caught (``truth_success``), again per axis value and
bin, and the plots ``set_<arm>_published_seed_<seed>.png``,
``set_<arm>_caught_seed_<seed>.png`` and ``sets_<arm>_seed_<seed>.png`` (launch
point and flight coloured by outcome). The rows are read by
``catch_search_map.sim_table``: a trial with an ``invalid_reason`` is a rig
failure and is not a thrown throw. Without that file the report is stage 1's.

**A sampled arm.** When the thrown throws are a sample of the arm's set, the
counts of the sim tiers are counts WITHIN THE THROWN. The set-wide numbers are
then ESTIMATES (:func:`stratified_estimate`): every stratum of the sample
(``throws/sample.json``'s ``meta.strata``: the set's count per stratum) is
weighted back by its set count over its thrown count. A stratum none of whose
throws was thrown is outside the estimate and is counted apart. The interval
next to a share is the Wilson 95 % interval of the share within the thrown
(``catching_trials.wilson_interval``); the estimate carries none.

**The arm** is drawn from the design's own record: the joint origins at the
wait pose and the catch point, which ``catch_throw_design design`` computed by
FK of the model the throws were judged with (``design_meta.json``). No model
file is opened here, so the report runs on a PC that holds the folder alone.

Every position is in the SIM WORLD.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from rtc_tools.analysis import catch_search_map as csm, catch_throw_design as ctd
from rtc_tools.analysis.catchability_map import BallParams
from rtc_tools.analysis.catching_throw_list import read_throw_list_file
from rtc_tools.analysis.catching_trials import wilson_interval
from rtc_tools.utils.catch_population import (
    Arm,
    ConfigError,
    PopulationConfig,
    ResultLayout,
    parse_config,
)

REPORT_TEXT = "population_report.txt"
# The file catch_search_map writes last: a map directory that has it is a finished map.
MAP_SUMMARY = "search_map_summary.json"
OFFLINE_TIERS = ("search", "segment")
NOT_JUDGED = "not judged"

# The derived axes the sets are also counted along, in bins of this width.
BINNED_AXES = (("speed_m_s", 1.0), ("pass_time_s", 0.1), ("apex_height_m", 0.5))

DEFAULT_MAX_FLIGHTS = 400
# The spacing of a drawn flight's points [s]: for the eye, not for a verdict.
PLOT_STEP_S = 0.01

# One colour per set of a combined plot: a neutral for the context and three hues
# that stay apart under colour-vision deficiency, in this order.
CONTEXT_COLOUR = "#9a9a96"
SET_COLOURS = ("#2a78d6", "#eb6834", "#1baf7a")
ARM_COLOUR = "#0b0b0b"

# sim_throws.csv: these three columns lead, then every column of a unit's
# catching_trials.csv, of which the report reads the four below.
SIM_LEADING = ("arm", "seed", "chunk")
SIM_COLUMNS = ("throw_id", "plan_verdict", "truth_success", "invalid_reason")


# ── What the folder holds ─────────────────────────────────────────────────────


@dataclass(frozen=True)
class TierState:
    """One offline tier of an arm as the folder holds it: how many throws its set has
    (``None``: there is no set, and ``why``), and the set's file (``None`` for an empty
    set — no file is written for one — and for no set). ``pending``: the map judged
    the tier and accepted throws, and the set's file is not written yet."""

    count: int | None
    why: str
    path: Path | None
    pending: bool = False


def read_map_summary(layout: ResultLayout, arm: Arm) -> dict | None:
    """The summary of the arm's offline map; None when the map is not finished."""
    path = layout.map_dir(arm.name) / MAP_SUMMARY
    return json.loads(path.read_text()) if path.is_file() else None


def tier_state(layout: ResultLayout, arm: Arm, tier: str, summary: Mapping | None) -> TierState:
    """The state of ``tier`` (search | segment) of ``arm``, from the map's summary and
    whether the set's file is there. A judge that refused its configuration judged
    nothing: no set, by the judge's own text. A tier the judge accepted no throw of is
    a set of 0 throws."""
    if summary is None:
        return TierState(None, NOT_JUDGED, None)
    entry = summary["searches"][arm.search]
    if "refused_configuration" in entry:
        return TierState(
            None, f"the search refused its configuration — {entry['refused_configuration']}", None
        )
    if tier == "segment":
        entry = entry.get("segment")
        if entry is None:
            return TierState(None, NOT_JUDGED, None)
        if "refused_configuration" in entry:
            return TierState(
                None,
                "the segment planner refused its configuration — "
                f"{entry['refused_configuration']}",
                None,
            )
    accepted = int(entry["accepted"])
    path = layout.set_file(arm.name, tier)
    if path.is_file():
        return TierState(accepted, "", path)
    if accepted == 0:
        return TierState(0, "", None)
    return TierState(None, f"{path.name} is not written (an unfinished run)", None, pending=True)


@dataclass(frozen=True)
class SimSeed:
    """One arm's sim trials of one estimator seed, reduced: the valid trials' throws in
    thrown order, which of them had their pair published and which were caught, and
    ``catch_search_map.sim_table``'s counts of the rows (``table``)."""

    seed: int
    thrown: list[dict]
    published: set[int]
    caught: set[int]
    table: Mapping


@dataclass(frozen=True)
class ArmResult:
    arm: Arm
    summary: Mapping | None  # the map's summary; None: not judged
    tiers: Mapping[str, TierState]
    sets: Mapping[str, list[dict] | None]  # tier → its throws; None: no set
    sample: Mapping | None  # throws/sample.json's meta; None: nothing to throw
    to_throw: list[dict] | None
    sim: Sequence[SimSeed] = ()


@dataclass(frozen=True)
class PopulationResult:
    layout: ResultLayout
    config: PopulationConfig
    manifest: Mapping
    design: Mapping  # design_meta.json
    population: list[dict]
    arms: Sequence[ArmResult] = field(default_factory=tuple)


def _throws_of(path: Path, by_id: Mapping[int, dict]) -> list[dict]:
    """The population's entries of the throws a list file holds, in file order."""
    listed = read_throw_list_file(path)
    try:
        return [by_id[int(t["throw_id"])] for t in listed.throws]
    except KeyError as exc:
        raise SystemExit(f"{path}: throw {exc} is not in the population") from exc


def read_sim_throws(path: Path, by_id: Mapping[int, dict]) -> list[SimSeed]:
    """``sim_throws.csv`` of one arm as one :class:`SimSeed` per estimator seed.

    The rows of a seed are reduced by ``catch_search_map.sim_table`` (the reader of
    ``catching_trials`` rows): a trial with an ``invalid_reason`` or without a
    ``plan_verdict`` is left out and counted. A throw thrown twice under one seed, and
    a throw the population does not hold, are refused: the file is one row per thrown
    throw and seed.

    The three leading columns are read by position: a unit's table has a ``seed``
    column of its own (the throw's, not the estimator's), so the name is in the header
    twice, and a row read by name would give the later one.
    """
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, [])
        if tuple(header[: len(SIM_LEADING)]) != SIM_LEADING:
            raise SystemExit(f"{path}: the first columns must be {', '.join(SIM_LEADING)}")
        missing = [c for c in SIM_COLUMNS if c not in header]
        if missing:
            raise SystemExit(f"{path}: no column {', '.join(missing)}")
        # Reversed: of two columns of one name the first — the leading one — is kept.
        rows = [dict(reversed(list(zip(header, cells, strict=False)))) for cells in reader]
    by_seed: dict[int, list[dict]] = {}
    for line, row in enumerate(rows, start=2):
        try:
            by_seed.setdefault(int(row["seed"]), []).append(row)
        except ValueError as exc:
            raise SystemExit(f"{path}:{line}: seed '{row['seed']}' is not an integer") from exc
    out = []
    for seed, own in sorted(by_seed.items()):
        joined, table = csm.sim_table(own)
        ids = [entry["throw_id"] for entry in joined if entry["throw_id"] is not None]
        repeated = sorted(i for i, n in Counter(ids).items() if n > 1)
        if repeated:
            raise SystemExit(
                f"{path}: seed {seed} holds {len(repeated)} throws more than once "
                f"(first: {repeated[:5]})"
            )
        unknown = sorted(i for i in ids if i not in by_id)
        if unknown:
            raise SystemExit(
                f"{path}: seed {seed} holds {len(unknown)} throws the population does not "
                f"(first: {unknown[:5]})"
            )
        out.append(
            SimSeed(
                seed=seed,
                thrown=[by_id[i] for i in ids],
                published={
                    e["throw_id"]
                    for e in joined
                    if e["throw_id"] is not None and e["sim_class"] == csm.SIM_ACCEPTED_PUBLISHED
                },
                caught={
                    e["throw_id"]
                    for e in joined
                    if e["throw_id"] is not None and e["truth_success"] is True
                },
                table=table,
            )
        )
    return out


def load_result(root: Path) -> PopulationResult:
    """Everything the report reads of the result folder ``root``. ``SystemExit`` (naming
    the file) for a folder that is not one, and for files that contradict each other."""
    layout = ResultLayout(Path(root).resolve())
    if not layout.config.is_file():
        raise SystemExit(f"{layout.root} is not a result folder: no {layout.config.name}")
    try:
        # parse_config, not load_config: the copy stands apart from the robot's folder.
        config = parse_config(
            yaml.safe_load(layout.config.read_text(encoding="utf-8")), layout.config
        )
    except (ConfigError, yaml.YAMLError) as exc:
        raise SystemExit(f"{layout.config}: {exc}") from exc
    meta_path = layout.design / "design_meta.json"
    if not meta_path.is_file() or not layout.population.is_file():
        raise SystemExit(
            f"{layout.root} holds no population yet (no {meta_path.name} or "
            f"{layout.population.name}): run catch_population first"
        )
    design = json.loads(meta_path.read_text())
    population = read_throw_list_file(layout.population).throws
    by_id = {int(t["throw_id"]): t for t in population}
    manifest = json.loads(layout.manifest.read_text()) if layout.manifest.is_file() else {}

    arms = []
    for arm in config.arms:
        summary = read_map_summary(layout, arm)
        tiers = {tier: tier_state(layout, arm, tier, summary) for tier in OFFLINE_TIERS}
        sets: dict[str, list[dict] | None] = {}
        for tier, state in tiers.items():
            if state.count is None:
                sets[tier] = None
                continue
            sets[tier] = _throws_of(state.path, by_id) if state.path is not None else []
            if len(sets[tier]) != state.count:
                raise SystemExit(
                    f"{state.path} holds {len(sets[tier])} throws but the map of "
                    f"{arm.name} accepted {state.count}: the set is not this map's"
                )
        sample_path = layout.throws_dir(arm.name) / "sample.json"
        sample, to_throw = None, None
        if sample_path.is_file():
            listed = read_throw_list_file(sample_path)
            sample, to_throw = listed.meta, _throws_of(sample_path, by_id)
        sim: list[SimSeed] = []
        sim_path = layout.sim_throws(arm.name)
        if sim_path.is_file():
            sim = read_sim_throws(sim_path, by_id)
            if to_throw is not None:
                listed_ids = {t["throw_id"] for t in to_throw}
                for one in sim:
                    stray = sorted(
                        t["throw_id"] for t in one.thrown if t["throw_id"] not in listed_ids
                    )
                    if stray:
                        raise SystemExit(
                            f"{sim_path}: seed {one.seed} holds {len(stray)} throws that "
                            f"{sample_path} does not (first: {stray[:5]})"
                        )
        arms.append(ArmResult(arm, summary, tiers, sets, sample, to_throw, tuple(sim)))
    return PopulationResult(layout, config, manifest, design, population, tuple(arms))


# ── Statistics ────────────────────────────────────────────────────────────────


def bin_edges(throws: Sequence[Mapping], axis: str, width: float) -> list[float] | None:
    """Edges of the ``width``-wide bins, on multiples of ``width``, that hold every
    finite value ``throws`` have of ``axis``; None when they have none."""
    values = [float(t[axis]) for t in throws if csm._finite(t.get(axis))]
    if not values:
        return None
    first = math.floor(min(values) / width)
    count = math.floor(max(values) / width) - first + 1
    return [round((first + i) * width, ctd.AXIS_DECIMALS) for i in range(count + 1)]


def shares_by_axis(
    base: Sequence[Mapping], ids: set[int], axis: str, edges: Sequence[float] | None = None
) -> list[dict]:
    """``catch_search_map.acceptance_by_axis`` with "accepted" read as "in ``ids``": per
    value (or bin) of ``axis``, how many throws of ``base`` there are and how many of
    them the set holds."""
    rows = [{axis: t.get(axis), "accepted": t["throw_id"] in ids} for t in base]
    return csm.acceptance_by_axis(rows, axis, edges)


def set_overlap(
    population: Sequence[Mapping], first: tuple[str, set[int]], second: tuple[str, set[int]]
) -> dict:
    """How two sets of the population overlap: ``both`` / ``only_first`` / ``only_second``
    / ``neither`` (``catch_search_map.search_disagreement`` on set membership)."""
    (name_a, ids_a), (name_b, ids_b) = first, second

    def verdicts(ids: set[int]) -> list[dict]:
        return [
            {"throw_id": t["throw_id"], "accepted": t["throw_id"] in ids, "reject_reason": ""}
            for t in population
        ]

    found = csm.search_disagreement(verdicts(ids_a), verdicts(ids_b), (name_a, name_b))
    return {
        "both": found["both_accepted"],
        "only_first": len(found[f"only_{name_a}"]),
        "only_second": len(found[f"only_{name_b}"]),
        "neither": found["both_refused"],
    }


def stratified_estimate(
    totals: Mapping[tuple, int], thrown: Mapping[tuple, int], hits: Mapping[tuple, int]
) -> dict:
    """The number of a set's throws that would be a hit, from a sample of it.

    ``totals`` is the set's count per stratum, ``thrown`` and ``hits`` the sample's.
    Every stratum with a thrown throw gives ``total × hits / thrown``; their sum is the
    ``estimate`` over the ``covered`` throws of the set (those in such a stratum). A
    stratum without a thrown throw says nothing: its throws are ``uncovered`` and no
    rate is assumed for them. ``exact`` when every throw of the set was thrown — the
    estimate is then a count.
    """
    estimate = 0.0
    covered = uncovered = uncovered_strata = 0
    for key, total in totals.items():
        n = thrown.get(key, 0)
        if n > total:
            raise ValueError(f"stratum {key}: {n} thrown of a set count of {total}")
        if n == 0:
            uncovered += total
            uncovered_strata += total > 0
            continue
        covered += total
        estimate += total * hits.get(key, 0) / n
    stray = sorted(key for key in thrown if key not in totals)
    if stray:
        raise ValueError(f"thrown throws in strata the set does not have: {stray[:3]}")
    return {
        "estimate": estimate,
        "set_throws": covered + uncovered,
        "covered": covered,
        "uncovered": uncovered,
        "uncovered_strata": uncovered_strata,
        "thrown": sum(thrown.values()),
        "hits": sum(hits.values()),
        "exact": all(thrown.get(key, 0) == total for key, total in totals.items()),
    }


def sample_estimates(sample: Mapping, one: SimSeed) -> dict[str, dict]:
    """:func:`stratified_estimate` of the pair-published and the caught throws of the
    set a sample was drawn from, from the throws ``one`` holds."""
    axes = sample["strata_axes"]
    totals = {
        tuple(round(float(s["values"][axis]), ctd.AXIS_DECIMALS) for axis in axes): int(s["total"])
        for s in sample["strata"]
    }
    thrown = Counter(ctd.stratum_key(t, axes) for t in one.thrown)
    return {
        name: stratified_estimate(
            totals,
            thrown,
            Counter(ctd.stratum_key(t, axes) for t in one.thrown if t["throw_id"] in ids),
        )
        for name, ids in (("published", one.published), ("caught", one.caught))
    }


# ── The text ──────────────────────────────────────────────────────────────────


def _share(count: int, of: int) -> str:
    return f"{100.0 * count / of:5.1f} %" if of else "    -  "


def _interval(count: int, of: int) -> str:
    low, high = wilson_interval(count, of)
    return f"[{100.0 * low:.1f}, {100.0 * high:.1f}] %" if of else "[-]"


def _reasons(counts: Mapping[str, int]) -> str:
    return ", ".join(f"{reason} {n}" for reason, n in counts.items()) or "(none)"


def _ids(throws: Sequence[Mapping]) -> set[int]:
    return {t["throw_id"] for t in throws}


def tier_table(
    title: str,
    base_name: str,
    base: Sequence[Mapping],
    columns: Mapping[str, set[int]],
    axis: str,
    edges: Sequence[float] | None = None,
) -> list[str]:
    """One row per value (or bin) of ``axis``: the count of ``base`` there, and each
    column's count with its share of that base count."""
    groups = {name: shares_by_axis(base, ids, axis, edges) for name, ids in columns.items()}
    first = shares_by_axis(base, set(), axis, edges)
    keys = [f"[{g['lo']:g}, {g['hi']:g})" if "lo" in g else f"{g['value']:g}" for g in first]
    width = max([len(k) for k in keys] + [6])
    widths = {name: max(18, len(name) + 2) for name in columns}
    lines = [
        title,
        f"  {'':{width}}  {base_name:>10}"
        + "".join(f"{name:>{widths[name]}}" for name in columns),
    ]
    for index, key in enumerate(keys):
        cells = f"{first[index]['n']:>10d}"
        for name in columns:
            group = groups[name][index]
            cell = f"{group['accepted']:d} {_share(group['accepted'], group['n'])}"
            cells += f"{cell:>{widths[name]}}"
        lines.append(f"  {key:{width}}  {cells}")
    return lines


def _axis_tables(
    heading: str, base_name: str, base: Sequence[Mapping], columns: Mapping[str, set[int]]
) -> list[str]:
    lines = [heading]
    for axis in ctd.DESIGN_AXES:
        lines += tier_table(f" {axis}", base_name, base, columns, axis)
        lines.append("")
    for axis, width in BINNED_AXES:
        edges = bin_edges(base, axis, width)
        if edges is None:
            continue
        lines += tier_table(f" {axis} (bins of {width:g})", base_name, base, columns, axis, edges)
        lines.append("")
    return lines


def _configuration_lines(result: PopulationResult) -> list[str]:
    design, config = result.design, result.config
    lines = ["CONFIGURATION"]
    for axis in ctd.DESIGN_AXES:
        values = design["grid"]["axes"][axis]
        lines.append(f"  {axis:18} {len(values):3d} values  {values}")
    elevation = design["elevation"]
    lines.append(
        f"  elevation range    max {elevation['max_deg']:g} deg, min {elevation['min_deg']:g} "
        f"deg, step {elevation['step_deg']:g} deg"
    )
    lines.append(f"  speed limit        {design['speed']['max_m_s']:g} m/s")
    lines.append(
        f"  min pass time      {design['limits']['min_pass_time_s']:g} s (launch -> pass point)"
    )
    timing = config.timing
    lines.append(
        f"  wakes              first {timing.detection_delay_s:g} s after release, every "
        f"{timing.vision_period_s:g} s, predicting {timing.prediction_horizon_s:g} s "
        f"({timing.source})"
    )
    sample = config.sample
    lines.append(
        f"  sample             strata {' x '.join(sample.strata)}, seed {sample.seed}, "
        f"chunks of {sample.chunk}"
    )
    for arm in config.arms:
        size = "the whole set" if arm.max_throws is None else f"at most {arm.max_throws}"
        lines.append(
            f"  arm {arm.name:18} search {arm.search}, segment planner {arm.segment}; "
            f"throws its {arm.throw_set} set ({size})"
        )
    return lines


def _arm_lines(result: PopulationResult, arm: ArmResult) -> list[str]:
    n_pop = len(result.population)
    lines = [f"  arm {arm.arm.name}"]
    for tier in OFFLINE_TIERS:
        throws = arm.sets[tier]
        if throws is None:
            lines.append(f"    {tier:8} set                 {arm.tiers[tier].why}")
            continue
        line = (
            f"    {tier:8} set                 {len(throws):7d}   "
            f"{_share(len(throws), n_pop)} of the population"
        )
        if tier == "segment" and arm.sets["search"]:
            share = _share(len(throws), len(arm.sets["search"])).strip()
            line += f", {share} of the search set"
        lines.append(line)
    chosen = arm.sets[arm.arm.throw_set]
    if arm.sample is not None:
        meta = arm.sample
        chunks = len(result.layout.chunks(arm.arm.name))
        whole = " — the whole set" if meta["n_throws"] == meta["plan_set_throws"] else ""
        lines.append(
            f"    to throw                     {meta['n_throws']:7d}   of the "
            f"{arm.arm.throw_set} set's {meta['plan_set_throws']}{whole} (strata "
            f"{' x '.join(meta['strata_axes'])}, seed {meta['seed']}, {chunks} chunks of "
            f"{meta['chunk']})"
        )
    elif chosen is not None and not chosen:
        lines.append(
            f"    to throw                           0   the {arm.arm.throw_set} set is empty"
        )
    elif arm.summary is None:
        lines.append(f"    to throw                     {NOT_JUDGED}")
    else:
        why = (
            f"there is no {arm.arm.throw_set} set"
            if chosen is None
            else "the sample is not written (an unfinished run)"
        )
        lines.append(f"    to throw                     nothing — {why}")
    if arm.summary is None:
        return lines
    entry = arm.summary["searches"][arm.arm.search]
    if "reasons" in entry:
        # A throw without a launch is in no throw list: its label is not a reason here.
        reasons = {
            reason: n
            for reason, n in entry["reasons"]["throws"].items()
            if reason not in ctd.UNLAUNCHABLE_SCREENS
        }
        lines.append(f"    refused by the search, of the population: {_reasons(reasons)}")
    segment = entry.get("segment") or {}
    if "reasons" in segment:
        lines.append(
            "    refused at the segment tier, of the search set: "
            f"{_reasons(segment['reasons']['throws'])}"
        )
    provenance = arm.summary.get("provenance") or {}
    repo = provenance.get("repo")
    lines.append(
        f"    judged with: judge sha256 {(provenance.get('judge') or {}).get('sha256', '?')[:12]}, "
        f"composed tree sha256 {str(provenance.get('catching_tree_sha256', '?'))[:12]}"
        + (f", revision {repo['commit']} (dirty: {repo['dirty']})" if repo else "")
    )
    return lines


def _overlap_lines(result: PopulationResult) -> list[str]:
    lines = []
    arms = result.arms
    for tier in OFFLINE_TIERS:
        for i, first in enumerate(arms):
            for second in arms[i + 1 :]:
                a, b = first.sets[tier], second.sets[tier]
                if a is None or b is None:
                    continue
                names = (first.arm.name, second.arm.name)
                found = set_overlap(result.population, (names[0], _ids(a)), (names[1], _ids(b)))
                lines.append(
                    f"  {tier} sets of {names[0]} and {names[1]}: both {found['both']}, "
                    f"{names[0]} only {found['only_first']}, {names[1]} only "
                    f"{found['only_second']}, neither {found['neither']}"
                )
    return lines


def _sim_lines(arm: ArmResult) -> list[str]:
    """The sim tiers of one arm, per estimator seed: counts within the thrown, and the
    estimate for the whole set when the thrown throws are a sample of it."""
    name = arm.arm.throw_set
    lines = [f"  arm {arm.arm.name} — thrown from its {name} set"]
    for one in arm.sim:
        table = one.table
        thrown = len(one.thrown)
        published, caught = len(one.published), len(one.caught)
        lines.append(
            f"    estimator seed {one.seed}: {table['trials']} rows, {table['invalid']} invalid "
            f"(a rig failure), {table['unjudged']} without a verdict, "
            f"{table['without_throw_id']} without a throw id"
        )
        lines.append("      COUNTS WITHIN THE THROWN (the interval is Wilson 95 %)")
        lines.append(f"        thrown                  {thrown:7d}")
        lines.append(
            f"        pair published          {published:7d}   {_share(published, thrown)} of "
            f"the thrown  {_interval(published, thrown)}"
        )
        lines.append(
            f"        caught (truth_success)  {caught:7d}   {_share(caught, thrown)} of the "
            f"thrown  {_interval(caught, thrown)}"
            + (
                f", {_share(len(one.caught & one.published), published).strip()} of the published"
                if published
                else ""
            )
        )
        stray = len(one.caught - one.published)
        if stray:
            lines.append(f"        caught without a published pair: {stray}")
        held = {**table["not_published_by_reason"]}
        for reason, n in table["refused_by_reason"].items():
            held[reason] = held.get(reason, 0) + n
        lines.append(f"        not published, by the sim's reason: {_reasons(csm._ranked(held))}")
        if arm.sample is None:
            lines.append("      no sample.json beside the throws: no estimate for the set")
            continue
        estimates = sample_estimates(arm.sample, one)
        whole = estimates["published"]
        size = whole["set_throws"]
        if whole["exact"]:
            lines.append(
                f"      THE WHOLE {name.upper()} SET WAS THROWN ({size} throws): the counts above "
                "are the set's"
            )
            continue
        lines.append(
            f"      ESTIMATE FOR THE WHOLE {name.upper()} SET ({size} throws; {thrown} thrown) — "
            f"each stratum of {' x '.join(arm.sample['strata_axes'])} weighted by its set "
            "count over its thrown count; not a count"
        )
        for label, key in (("pair published", "published"), ("caught", "caught")):
            found = estimates[key]
            lines.append(
                f"        {label:22} ~ {found['estimate']:8.1f}   "
                f"{_share(found['estimate'], found['covered']).strip()} of the "
                f"{found['covered']} set throws in a stratum with a thrown throw"
            )
        if whole["uncovered"]:
            lines.append(
                f"        outside the estimate: {whole['uncovered']} set throws in "
                f"{whole['uncovered_strata']} strata without a thrown throw"
            )
    return lines


def report_text(result: PopulationResult) -> str:
    """The statistics text of a result folder."""
    design, manifest = result.design, result.manifest
    source = manifest.get("config") or {}
    repo = manifest.get("repo")
    labels = design["screens"]["labels"]
    limits = design["limits"]
    n_pop = len(result.population)
    lines = [
        f"catch population report — {source.get('robot', result.layout.root.name)}",
        f"result folder : {result.layout.root}",
        f"configuration : {source.get('source', result.layout.config)} "
        f"(sha256 {result.config.sha256[:12]})",
        "revision      : "
        + (f"{repo['commit']} (dirty: {repo['dirty']})" if repo else "not recorded"),
        "",
        *_configuration_lines(result),
        "",
        "THE SETS",
        f"  design points                      {design['grid']['throws']:7d}",
    ]
    for label in ctd.UNLAUNCHABLE_SCREENS:
        lines.append(f"    no launch — {label:22} {labels.get(label, 0):7d}")
    lines.append(f"    removed by the pass-time limit   {limits['limited']:7d}")
    lines.append(f"  POPULATION                         {n_pop:7d}")
    screened = Counter(t[csm.SCREEN_KEY] for t in result.population if t.get(csm.SCREEN_KEY))
    if screened:
        lines.append(
            "    of which screened (no search can accept them; none is run): "
            f"{_reasons(csm._ranked(screened))}"
        )
    for arm in result.arms:
        lines += _arm_lines(result, arm)
    lines += _overlap_lines(result)

    with_sim = [arm for arm in result.arms if arm.sim]
    if with_sim:
        lines += ["", "THE SIM (stage 2) — thrown -> pair published in the sim -> caught"]
        for arm in with_sim:
            lines += _sim_lines(arm)

    columns = {
        f"{arm.arm.name}:{tier}": _ids(arm.sets[tier])
        for arm in result.arms
        for tier in OFFLINE_TIERS
        if arm.sets[tier] is not None
    }
    lines.append("")
    lines += _axis_tables(
        "BY AXIS VALUE AND BIN — each set's count, and its share of the population there",
        "population",
        result.population,
        columns,
    )
    for arm in with_sim:
        for one in arm.sim:
            lines += _axis_tables(
                f"SIM BY AXIS VALUE AND BIN — arm {arm.arm.name}, estimator seed {one.seed}: "
                "counts within the thrown, and the share of the thrown there",
                "thrown",
                one.thrown,
                {"pair published": one.published, "caught": one.caught},
            )
    while lines and not lines[-1]:
        lines.pop()
    return "\n".join(lines)


# ── The 3D plots ──────────────────────────────────────────────────────────────


def pick_throws(throws: Sequence[Mapping], count: int) -> list[Mapping]:
    """At most ``count`` of ``throws``, the same ones on every run: those ranked first
    by a hash of the throw id (``catch_throw_design``'s rank — no RNG stream)."""
    if len(throws) <= count:
        return list(throws)
    ranked = sorted(throws, key=lambda t: (ctd._rank(0, "plot", t["throw_id"]), t["throw_id"]))
    return ranked[: max(count, 0)]


def flight_paths(
    throws: Sequence[Mapping], ball: BallParams, step_s: float = PLOT_STEP_S
) -> list[np.ndarray]:
    """Each throw's positions from its launch to its pass instant, ``(k, 3)``.

    The flights are ``catch_throw_design.integrate_flights`` — the force law the throws
    were designed on — sampled every ``step_s``; the last point is interpolated at the
    throw's ``pass_time_s``.
    """
    if not throws:
        return []
    position = np.array([t["pos"] for t in throws], dtype=float)
    velocity = np.array([t["vel"] for t in throws], dtype=float)
    pass_time = np.array([float(t["pass_time_s"]) for t in throws])
    batch = ctd.integrate_flights(
        position, velocity, ball, horizon_s=float(pass_time.max()) + step_s, step_s=step_s
    )
    paths = []
    for i, instant in enumerate(pass_time):
        last = min(int(instant // step_s), batch.time_s.size - 2)
        head = batch.position_m[: last + 1, i]
        share = (instant - batch.time_s[last]) / step_s
        end = head[-1] + share * (batch.position_m[last + 1, i] - head[-1])
        paths.append(np.vstack([head, end]))
    return paths


def pyplot(show: bool):
    """``matplotlib.pyplot`` with the 3D projection; the Agg backend unless ``show``.

    ``mpl_toolkits`` can resolve to another matplotlib's copy than the ``matplotlib``
    this interpreter imports (an older system package beside a newer one in an
    environment), and ``mplot3d`` of the wrong one does not import. The package is
    pointed at the copy beside its own matplotlib; where the two already agree nothing
    changes.
    """
    import matplotlib  # noqa: PLC0415

    if not show:
        matplotlib.use("Agg")
    import mpl_toolkits  # noqa: PLC0415

    own = Path(matplotlib.__file__).parent.parent / "mpl_toolkits"
    if (own / "mplot3d").is_dir() and list(mpl_toolkits.__path__) != [str(own)]:
        mpl_toolkits.__path__ = [str(own)]
        for name in [n for n in sys.modules if n.startswith("mpl_toolkits.")]:
            del sys.modules[name]
    import matplotlib.pyplot as plt  # noqa: PLC0415
    from matplotlib import projections  # noqa: PLC0415

    if "3d" not in projections.get_projection_names():
        # pyplot was imported before the package pointed at its own copy.
        from mpl_toolkits.mplot3d import Axes3D  # noqa: PLC0415

        projections.register_projection(Axes3D)
    return plt


@dataclass(frozen=True)
class Scene:
    """What every plot of a folder shares: the arm, the catch point and the box."""

    body: np.ndarray  # (n, 3) joint origins at the wait pose
    catch: np.ndarray
    approach: np.ndarray | None  # the hand's approach axis
    low: np.ndarray
    high: np.ndarray
    elevation: tuple[float, float]

    @classmethod
    def of(cls, design: Mapping, population: Sequence[Mapping]) -> Scene:
        body = np.array(design["body"]["vertices_world_m"], dtype=float).reshape(-1, 3)
        catch = np.array(design["wait_catch_point_world_m"], dtype=float)
        axis = design.get("approach_axis_world")
        points = np.vstack(
            [
                body,
                catch,
                *[t["pos"] for t in population],
                *[[t["pass_x"], t["pass_y"], t["pass_z"]] for t in population],
            ]
        )
        low, high = points.min(axis=0), points.max(axis=0)
        apexes = [t["apex_height_m"] for t in population if csm._finite(t.get("apex_height_m"))]
        low[2], high[2] = 0.0, max([high[2], *apexes])
        margin = 0.04 * float((high - low).max())
        low[:2] -= margin
        high += margin
        elevation = design["elevation"]
        return cls(
            body,
            catch,
            None if axis is None else np.array(axis, dtype=float),
            low,
            high,
            (float(elevation["min_deg"]), float(elevation["max_deg"])),
        )

    def draw(self, ax, title: str) -> None:
        """The arm at its wait pose, the catch point and the axes of one plot."""
        stand = np.vstack([[self.body[0, 0], self.body[0, 1], 0.0], self.body, self.catch])
        ax.plot(*stand.T, color=ARM_COLOUR, lw=3.0, zorder=20, label="arm at its wait pose")
        ax.scatter(*self.body.T, color=ARM_COLOUR, s=14, zorder=20, depthshade=False)
        ax.scatter(
            *self.catch,
            color="white",
            edgecolor=ARM_COLOUR,
            linewidth=1.5,
            s=60,
            zorder=21,
            depthshade=False,
            label="catch point (wait pose)",
        )
        if self.approach is not None:
            tip = self.catch + 0.2 * self.approach
            ax.plot(*np.vstack([self.catch, tip]).T, color=ARM_COLOUR, lw=1.2, zorder=21)
        ax.set_xlim(self.low[0], self.high[0])
        ax.set_ylim(self.low[1], self.high[1])
        ax.set_zlim(self.low[2], self.high[2])
        ax.set_box_aspect(tuple(self.high - self.low))
        ax.set_xlabel("world x [m]")
        ax.set_ylabel("world y [m]")
        ax.set_zlabel("world z [m]")
        ax.view_init(elev=18, azim=-62)
        ax.set_title(title, fontsize=10)


@dataclass(frozen=True)
class Layer:
    """One set of a combined plot: its name, all its throws and its colour."""

    name: str
    throws: Sequence[Mapping]
    colour: str
    alpha: float
    launches: bool = False  # mark the launch points in the layer's colour


class Plotter:
    """Draws a folder's plots: the flights are integrated once per throw."""

    def __init__(self, result: PopulationResult, max_flights: int, show: bool):
        self.plt = pyplot(show)
        from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: PLC0415

        self._collection = Line3DCollection
        ball = result.design["ball"]
        self.ball = BallParams(
            ball["radius_m"],
            ball["mass_kg"],
            ball["drag_coefficient"],
            ball["air_density_kg_m3"],
            ball["sources"],
        )
        self.scene = Scene.of(result.design, result.population)
        self.label = (result.manifest.get("config") or {}).get("robot", result.layout.root.name)
        self.out = result.layout.report
        self.max_flights = max_flights
        self.worst_end_m = 0.0
        self.written: list[Path] = []
        self._paths: dict[int, np.ndarray] = {}

    def drawn(self, throws: Sequence[Mapping]) -> tuple[list[Mapping], list[np.ndarray]]:
        """The throws of ``throws`` that are drawn (at most ``max_flights``, picked by
        id) and their flights."""
        chosen = [
            t for t in pick_throws(throws, self.max_flights) if csm._finite(t.get("pass_time_s"))
        ]
        fresh = [t for t in chosen if t["throw_id"] not in self._paths]
        for throw, path in zip(fresh, flight_paths(fresh, self.ball), strict=True):
            self._paths[throw["throw_id"]] = path
            target = np.array([throw["pass_x"], throw["pass_y"], throw["pass_z"]], dtype=float)
            self.worst_end_m = max(self.worst_end_m, float(np.linalg.norm(path[-1] - target)))
        return chosen, [self._paths[t["throw_id"]] for t in chosen]

    def _lines(self, paths: Sequence[np.ndarray], **style):
        """``paths`` as one line collection. A flight ends at its own pass instant, so
        the shorter ones are padded with their last point: one array, whatever the
        matplotlib."""
        longest = max((len(path) for path in paths), default=0)
        padded = [
            np.vstack([path, np.repeat(path[-1:], longest - len(path), 0)]) for path in paths
        ]
        return self._collection(np.array(padded).reshape(-1, max(longest, 2), 3), **style)

    def _figure(self):
        figure = self.plt.figure(figsize=(11, 8))
        ax = figure.add_subplot(projection="3d")
        # Drawn in the order given, not by depth: a set over the set it is part of.
        ax.computed_zorder = False
        return figure, ax

    def _save(self, figure, name: str) -> None:
        figure.tight_layout()
        path = self.out / name
        figure.savefig(path, dpi=140)
        self.written.append(path)

    def one_set(self, name: str, title: str, throws: Sequence[Mapping]) -> None:
        """``set_<name>.png``: the set's flights coloured by launch elevation."""
        chosen, paths = self.drawn(throws)
        figure, ax = self._figure()
        low, high = self.scene.elevation
        norm = self.plt.Normalize(low, max(high, low + 1e-9))
        cmap = self.plt.get_cmap("viridis")
        ax.add_collection3d(
            self._lines(
                paths,
                colors=[cmap(norm(t["elevation_deg"])) for t in chosen],
                linewidths=0.6,
                alpha=0.6,
            )
        )
        launches = np.array([t["pos"] for t in throws], dtype=float)
        ax.scatter(*launches.T, color=CONTEXT_COLOUR, s=4, alpha=0.4, label="launch points (all)")
        self.scene.draw(
            ax, f"{self.label} — {title}: {len(throws)} throws ({len(paths)} flights drawn)"
        )
        mappable = self.plt.cm.ScalarMappable(cmap=cmap, norm=norm)
        figure.colorbar(mappable, ax=ax, shrink=0.55, pad=0.02, label="launch elevation [deg]")
        ax.legend(loc="upper left", fontsize=8)
        self._save(figure, f"set_{name}.png")

    def combined(self, name: str, title: str, layers: Sequence[Layer]) -> None:
        """``<name>.png``: several sets in one plot, one colour each."""
        figure, ax = self._figure()
        drawn = []
        for order, layer in enumerate(layers):
            _, paths = self.drawn(layer.throws)
            drawn.append(len(paths))
            ax.add_collection3d(
                self._lines(
                    paths, colors=layer.colour, linewidths=0.6, alpha=layer.alpha, zorder=order
                )
            )
            # A proxy line: a collection gives the legend no handle of its own.
            ax.plot(
                [], [], [], color=layer.colour, lw=2.5, label=f"{layer.name} ({len(layer.throws)})"
            )
            if layer.launches and layer.throws:
                launches = np.array([t["pos"] for t in layer.throws], dtype=float)
                # Smaller on every later layer: a launch point several outcomes share
                # shows them as rings.
                ax.scatter(
                    *launches.T,
                    color=layer.colour,
                    s=36.0 / (1 + order) ** 1.5,
                    depthshade=False,
                    zorder=len(layers) + order,
                )
        self.scene.draw(
            ax,
            f"{self.label} — {title} ({' / '.join(str(n) for n in drawn)} flights drawn, "
            f"at most {self.max_flights} per set)",
        )
        ax.legend(loc="upper left", fontsize=8)
        self._save(figure, f"{name}.png")


def write_plots(result: PopulationResult, *, max_flights: int, show: bool) -> Plotter:
    """Every plot of the folder into ``report/``; a set of 0 throws gets none."""
    plotter = Plotter(result, max_flights, show)
    plotter.one_set("population", "population", result.population)
    context = Layer("population", result.population, CONTEXT_COLOUR, 0.3)
    for arm in result.arms:
        name = arm.arm.name
        layers = [context]
        for tier, colour in zip(OFFLINE_TIERS, SET_COLOURS, strict=False):
            throws = arm.sets[tier]
            if throws is None:
                continue
            layers.append(Layer(f"{tier} set", throws, colour, 0.7))
            if throws:
                plotter.one_set(f"{name}_{tier}", f"{name}, {tier} set", throws)
        if len(layers) > 1:
            plotter.combined(f"sets_{name}", f"{name}: the sets together", layers)
        for one in arm.sim:
            outcomes = {
                "thrown, no pair published": [
                    t for t in one.thrown if t["throw_id"] not in one.published | one.caught
                ],
                "pair published, not caught": [
                    t
                    for t in one.thrown
                    if t["throw_id"] in one.published and t["throw_id"] not in one.caught
                ],
                "caught": [t for t in one.thrown if t["throw_id"] in one.caught],
            }
            for tier, ids in (("published", one.published), ("caught", one.caught)):
                throws = [t for t in one.thrown if t["throw_id"] in ids]
                if throws:
                    plotter.one_set(
                        f"{name}_{tier}_seed_{one.seed}",
                        f"{name}, {tier} in the sim (estimator seed {one.seed})",
                        throws,
                    )
            plotter.combined(
                f"sets_{name}_seed_{one.seed}",
                f"{name}: the thrown throws by outcome (estimator seed {one.seed})",
                [
                    Layer(label, throws, colour, 0.75, launches=True)
                    for (label, throws), colour in zip(outcomes.items(), SET_COLOURS, strict=True)
                ],
            )
    return plotter


# ── CLI ───────────────────────────────────────────────────────────────────────


def report(root: Path, *, show: bool = False, max_flights: int = DEFAULT_MAX_FLIGHTS) -> int:
    """Write and print the report of the result folder ``root``; with ``show`` the plots
    are also opened in windows (the call then returns when they are closed)."""
    if max_flights < 1:
        raise SystemExit("--max-flights must be >= 1")
    result = load_result(root)
    out = result.layout.report
    out.mkdir(parents=True, exist_ok=True)
    text = report_text(result)
    (out / REPORT_TEXT).write_text(text + "\n", encoding="utf-8")
    print(text)
    plotter = write_plots(result, max_flights=max_flights, show=show)
    print(f"\nreport: {out / REPORT_TEXT}")
    print(f"plots:  {len(plotter.written)} PNG files in {out}")
    print(f"(drawn flights end within {plotter.worst_end_m * 1e3:.1f} mm of their pass points)")
    if show:
        plotter.plt.show()
    plotter.plt.close("all")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("result_dir", type=Path, help="a result folder of catch_population")
    ap.add_argument("--show", action="store_true", help="also open the plots in windows")
    ap.add_argument(
        "--max-flights",
        type=int,
        default=DEFAULT_MAX_FLIGHTS,
        help="flights drawn per set (a larger set: the same throws on every run, by id)",
    )
    args = ap.parse_args(argv)
    return report(args.result_dir, show=args.show, max_flights=args.max_flights)


if __name__ == "__main__":
    sys.exit(main())
