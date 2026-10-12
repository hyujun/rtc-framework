"""Collect what the throw population's sim units caught (#821, stage 2).

``catch_population_trials`` leaves one unit directory per (arm, estimator seed,
chunk) under ``<result>/sim/`` (``rtc_tools.utils.catch_population.ResultLayout``).
This module turns the finished ones into an arm's two result files:

* ``<result>/<arm>/sim_throws.csv`` — every row of every DONE unit's
  ``ct/catching_trials.csv`` behind three leading columns ``arm,seed,chunk``. One
  row per throw and seed: what the sim did with it (``plan_verdict``,
  ``truth_success``, ``invalid_reason``, … — the analyser's columns, untouched).
* ``<result>/<arm>/throw_catch_set.json`` — a ``catching_throw_list/1`` file of
  the thrown throws the sim caught (``truth_success``) on at least one seed.

**A catch set is "caught on that seed".** The same throw is caught under one
estimator seed and missed under another (E1-F20), so every throw of the set
carries ``caught_seeds`` and ``thrown_seeds``, and the meta carries, per seed,
how many throws were thrown / invalid / had their plan–segment pair published /
were caught. **A throw that was not thrown is not a missed throw**: the meta
says which set was thrown (``search`` | ``segment``), whether all of it or a
sample (``sample``: n drawn of N), and which chunks each seed threw, so a reader
can tell "not in the catch set" three ways — not drawn, not thrown yet, thrown
and missed. The offline segment set and "the pair was published in the sim" are
not the same thing either, which is why the published count stands beside the
thrown one.

**Units from another PC merge here.** The key that joins two PCs' results is
(the sha256 of the set that was sampled, ``throw_id``); a unit directory copied
into this folder's ``sim/`` tree is collected like a local one, provided it
threw THIS folder's chunk file: a unit whose recorded ``throws_file_sha256`` is
not the sha256 of ``<arm>/throws/<chunk>.json`` is refused by name
(:class:`CollectError`) — its ``throw_id`` s would name other throws.

The analysis (:func:`analyse_unit`) is ``integrated_bringup.catching_tc_vector``
in a process of its own, under the interpreter the installed ``catching_trials``
uses. It is CPU heavy and is never started beside a running sim: the caller
runs it between two units or at the end.

Nothing here imports numpy, pandas or ROS.
"""

from __future__ import annotations

import csv
import hashlib
import re
import subprocess
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

DONE = "DONE"
TOOL = "integrated_bringup.catch_population_trials"
CATCH_SET_KIND = "catch_set"
LEADING_COLUMNS = ("arm", "seed", "chunk")
# The sample meta a set carries on (catch_throw_design's _CARRIED_META): the axes and the
# design the throws were drawn in, so that a set file describes its own throws.
CARRIED_META = ("axes", "design", "limit")
CATCH_SET_NOTE = (
    "throw_catch_set: the thrown throws the sim caught (truth_success) on at least one "
    "estimator seed. A catch is 'caught on that seed' (caught_seeds); thrown_seeds are the "
    "seeds the throw was thrown on. A throw that is not here was not drawn into the sample, "
    "or not thrown yet (seeds[].chunks of chunks_total), or thrown and missed — not being "
    "here is not a miss. pair_published counts the throws whose plan and first segment were "
    "published in the sim, which the offline segment set does not decide. A catch is ONE "
    "run's outcome (runs: each unit's date, revisions and viewer): the same chunk thrown "
    "again under the same seed does not catch the same throws."
)
_SEED_DIR = re.compile(r"^seed_(\d+)$")
_CHUNK_DIR = re.compile(r"^chunk_\d{3,}$")


class CollectError(RuntimeError):
    """The unit directories do not make one result; the message names the unit."""


@dataclass(frozen=True)
class UnitRef:
    """One unit directory: ``<result>/sim/<arm>/seed_<seed>/<chunk>``."""

    arm: str
    seed: int
    chunk: str  # the chunk file's stem
    path: Path

    @property
    def name(self) -> str:
        return f"{self.arm}/seed_{self.seed}/{self.chunk}"


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def unit_status(unit: Path) -> str | None:
    """The unit's ``status`` (``DONE`` | ``FAIL:<why>``), None when it has none."""
    try:
        return (Path(unit) / "status").read_text(encoding="utf-8").strip() or None
    except OSError:
        return None


def read_conditions(unit: Path) -> dict[str, str]:
    """``conditions.txt`` as a map (``key: value`` lines; a key written twice keeps its
    last value). Empty when the unit has none."""
    out: dict[str, str] = {}
    try:
        text = (Path(unit) / "conditions.txt").read_text(encoding="utf-8")
    except OSError:
        return out
    for line in text.splitlines():
        key, sep, value = line.partition(":")
        if sep:
            out[key.strip()] = value.strip()
    return out


def sim_root(layout) -> Path:
    """``<result>/sim``: the folder every unit of the result folder is under (the
    layout names a unit's directory, three levels below it)."""
    return layout.sim_root


def units_present(units_root: Path, arm: str) -> list[UnitRef]:
    """The unit directories of ``arm`` under ``units_root`` (``<result>/sim``), by seed
    and chunk. A failed attempt moved aside (``chunk_003.fail1``) is not a unit."""
    units = []
    arm_dir = Path(units_root) / arm
    if not arm_dir.is_dir():
        return units
    for seed_dir in arm_dir.iterdir():
        match = _SEED_DIR.match(seed_dir.name)
        if not match or not seed_dir.is_dir():
            continue
        for unit in seed_dir.iterdir():
            if unit.is_dir() and _CHUNK_DIR.match(unit.name):
                units.append(UnitRef(arm, int(match.group(1)), unit.name, unit))
    return sorted(units, key=lambda u: (u.seed, u.chunk))


def unit_refusal(unit: UnitRef, chunk_file: Path, *, robot: str, search: str, segment: str):
    """Why a DONE unit is not one of this result folder's, or None.

    The unit must have thrown ``chunk_file`` (the same bytes), under the seed its
    directory names, at ``robot``, as the arm ``search`` x ``segment``.
    """
    cond = read_conditions(unit.path)
    recorded = cond.get("throws_file_sha256")
    if not recorded:
        return "its conditions.txt records no throws_file_sha256"
    if not chunk_file.is_file():
        return f"this folder has no {chunk_file.name} for arm {unit.arm}"
    here = sha256_file(chunk_file)
    if recorded != here:
        return (
            f"it threw another file (throws_file_sha256 {recorded[:12]}…, this folder's "
            f"{chunk_file.name} is {here[:12]}…)"
        )
    if cond.get("seed") != str(unit.seed):
        return f"its conditions.txt says seed {cond.get('seed')}, its directory seed_{unit.seed}"
    if cond.get("robot") != robot:
        return f"it ran {cond.get('robot')}, this folder is {robot}"
    told = (cond.get("expect_search"), cond.get("expect_mode"))
    if told != (search, segment):
        return f"it ran the arm {told[0]} x {told[1]}, its directory says {search} x {segment}"
    return None


# ── analysis ─────────────────────────────────────────────────────────────────


def analysis_command(python: str, unit: Path, config_dir: Path | str) -> list[str]:
    """The process that writes ``<unit>/ct/`` (what ``analyse_unit.sh`` runs)."""
    ct = Path(unit) / "ct"
    return [
        python,
        "-m",
        "integrated_bringup.catching_tc_vector",
        str(unit),
        str(config_dir),
        str(ct / "vec.csv"),
        str(ct),
    ]


def needs_analysis(unit: Path) -> bool:
    """A DONE unit without ``ct/vec.csv`` (``analyse_unit.sh``'s rule)."""
    return unit_status(unit) == DONE and not (Path(unit) / "ct" / "vec.csv").is_file()


def analyse_unit(unit: Path, config_dir: Path | str, python: str) -> bool:
    """Run the analysis of one unit into ``<unit>/ct/`` (its output in ``ct/log.txt``).
    True when it wrote ``vec.csv``."""
    ct = Path(unit) / "ct"
    ct.mkdir(exist_ok=True)
    with open(ct / "log.txt", "w", encoding="utf-8") as log:
        done = subprocess.run(
            analysis_command(python, unit, config_dir),
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return done.returncode == 0 and (ct / "vec.csv").is_file()


# ── the two files ────────────────────────────────────────────────────────────


def _flag(text: str | None) -> bool:
    return (text or "").strip().lower() == "true"


def _invalid(row: Mapping[str, str]) -> bool:
    return bool((row.get("invalid_reason") or "").strip())


def _throw_id(row: Mapping[str, str], where: str) -> int:
    try:
        return int(float(row["throw_id"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise CollectError(f"{where}: a row has no throw_id ({row.get('throw_id')!r})") from exc


def read_unit_rows(unit: UnitRef) -> tuple[list[str], list[dict[str, str]]]:
    """The header and rows of the unit's ``ct/catching_trials.csv``, as text."""
    path = unit.path / "ct" / "catching_trials.csv"
    try:
        with open(path, newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            rows = list(reader)
            header = list(reader.fieldnames or [])
    except OSError as exc:
        raise CollectError(
            f"unit {unit.name} is DONE but has no ct/catching_trials.csv (its analysis "
            "failed? see ct/log.txt)"
        ) from exc
    if "throw_id" not in header:
        raise CollectError(f"unit {unit.name}: ct/catching_trials.csv has no throw_id column")
    return header, rows


# What a unit's conditions.txt says of the run it was: a catch is one run's outcome,
# so the set says which run each of its units was.
RUN_KEYS = ("date_start", "rtc_framework_rev", "ball_perception_rev", "viewer")


def unit_run_record(unit: UnitRef) -> dict:
    """One unit as the run it was: its seed and chunk, and :data:`RUN_KEYS` of its
    ``conditions.txt`` (None for one it does not record)."""
    cond = read_conditions(unit.path)
    return {"seed": unit.seed, "chunk": unit.chunk, **{key: cond.get(key) for key in RUN_KEYS}}


@dataclass
class SeedCounts:
    """What one estimator seed's units did with the throws they threw."""

    seed: int
    chunks: list[str] = field(default_factory=list)
    thrown: int = 0
    invalid: int = 0
    pair_published: int = 0
    caught: int = 0

    def add(self, row: Mapping[str, str]) -> bool:
        """Count one throw; True when it was caught. An invalid throw (the analyser's
        ``invalid_reason``) is thrown and nothing else."""
        self.thrown += 1
        if _invalid(row):
            self.invalid += 1
            return False
        if (row.get("plan_verdict") or "").strip() == "published":
            self.pair_published += 1
        if _flag(row.get("truth_success")):
            self.caught += 1
            return True
        return False

    def as_record(self) -> dict:
        return {
            "seed": self.seed,
            "chunks": list(self.chunks),
            "thrown": self.thrown,
            "invalid": self.invalid,
            "pair_published": self.pair_published,
            "caught": self.caught,
        }


@dataclass
class ArmCollection:
    """What :func:`collect_arm` found and wrote for one arm."""

    arm: str
    seeds: list[SeedCounts] = field(default_factory=list)
    units_done: list[str] = field(default_factory=list)
    units_not_done: list[str] = field(default_factory=list)  # "name (status)"
    thrown_ids: int = 0  # distinct throws thrown on any seed
    caught_ids: int = 0  # distinct throws caught on any seed
    sim_throws: Path | None = None
    catch_set: Path | None = None

    def lines(self) -> list[str]:
        out = [
            f"{self.arm}: {len(self.units_done)} units collected"
            + (f", {len(self.units_not_done)} not DONE" if self.units_not_done else "")
        ]
        for name in self.units_not_done:
            out.append(f"  not collected: {name}")
        for seed in self.seeds:
            out.append(
                f"  seed {seed.seed}: thrown {seed.thrown}, invalid {seed.invalid}, pair "
                f"published {seed.pair_published}, caught {seed.caught} ({len(seed.chunks)} chunks)"
            )
        if self.seeds:
            out.append(
                f"  throw_catch_set: {self.caught_ids} of {self.thrown_ids} thrown throws "
                f"caught on at least one seed -> {self.catch_set or 'not written (none caught)'}"
            )
        return out


def catch_set_meta(
    *,
    robot: str,
    arm: str,
    search: str,
    segment: str,
    thrown_set: str,
    sample_meta: Mapping,
    sample_sha256: str | None,
    chunks_total: int,
    seeds: Sequence[SeedCounts],
    runs: Sequence[Mapping],
    thrown_ids: int,
    caught_ids: int,
) -> dict:
    """The catch set's ``meta`` (the module docstring says what a reader needs from it).

    ``sample_meta`` is ``throws/sample.json``'s: ``plan_set_sha256`` /
    ``plan_set_throws`` are the set that was sampled (the merge key's first half and
    its size N), ``n_throws`` the n drawn.
    """
    drawn = sample_meta.get("n_throws")
    of = sample_meta.get("plan_set_throws")
    return {
        "tool": TOOL,
        "kind": CATCH_SET_KIND,
        "tier": "catch",
        "set_name": "throw_catch_set",
        "robot": robot,
        "arm": {"name": arm, "search": search, "segment": segment},
        "thrown_set": thrown_set,
        "source_set": {
            "set_name": f"throw_{thrown_set}_set",
            "sha256": sample_meta.get("plan_set_sha256"),
            "n_throws": of,
        },
        "sample": {
            "sha256": sample_sha256,
            "n_throws": drawn,
            "of": of,
            "whole_set": drawn is not None and drawn == of,
            "seed": sample_meta.get("seed"),
            "max_throws": sample_meta.get("max_throws"),
            "strata_axes": sample_meta.get("strata_axes"),
        },
        "merge_key": ["source_set.sha256", "throw_id"],
        "chunks_total": chunks_total,
        "seeds": [seed.as_record() for seed in seeds],
        # One entry per unit collected (:func:`unit_run_record`).
        "runs": list(runs),
        "counts": {"thrown": thrown_ids, "caught": caught_ids},
        "note": CATCH_SET_NOTE,
        **{k: sample_meta[k] for k in CARRIED_META if k in sample_meta},
    }


def collect_arm(
    layout,
    *,
    robot: str,
    arm: str,
    search: str,
    segment: str,
    thrown_set: str,
) -> ArmCollection:
    """Rebuild ``sim_throws.csv`` and ``throw_catch_set.json`` of ``arm`` from the unit
    directories present (``layout``: the result folder's ``ResultLayout``).

    Every DONE unit is checked against this folder's chunk file first
    (:func:`unit_refusal`); one that fails raises :class:`CollectError` naming every
    such unit, and nothing is written. Units that are not DONE are listed in the
    result and left out. With no DONE unit the two files are removed (a stale file
    would be read as this run's). A catch set of no throw is not written either —
    the throw-list format has no empty list — and the result says so.
    """
    from rtc_tools.analysis.catching_throw_list import read_throw_list_file, write_throw_list

    result = ArmCollection(arm)
    units_root = sim_root(layout)
    chunk_files = {path.stem: path for path in layout.chunks(arm)}
    done: list[UnitRef] = []
    refused = []
    for unit in units_present(units_root, arm):
        status = unit_status(unit.path)
        if status != DONE:
            result.units_not_done.append(f"{unit.name} ({status or 'no status'})")
            continue
        chunk_file = chunk_files.get(unit.chunk, layout.throws_dir(arm) / f"{unit.chunk}.json")
        why = unit_refusal(unit, chunk_file, robot=robot, search=search, segment=segment)
        if why:
            refused.append(f"{unit.name}: {why}")
        else:
            done.append(unit)
    if refused:
        raise CollectError(
            f"{len(refused)} unit(s) under {units_root / arm} are not this result folder's "
            "(move them out):\n  " + "\n  ".join(refused)
        )

    sim_throws, catch_set = layout.sim_throws(arm), layout.set_file(arm, "catch")
    if not done:
        sim_throws.unlink(missing_ok=True)
        catch_set.unlink(missing_ok=True)
        return result

    sample_path = layout.throws_dir(arm) / "sample.json"
    try:
        sample = read_throw_list_file(sample_path)
    except (OSError, ValueError) as exc:
        raise CollectError(f"{arm}: cannot read the thrown sample {sample_path}: {exc}") from exc
    thrown_from = layout.set_file(arm, thrown_set)
    if thrown_from.is_file() and sha256_file(thrown_from) != sample.meta.get("plan_set_sha256"):
        raise CollectError(
            f"{arm}: {sample_path} was not drawn from this folder's {thrown_from.name} "
            "(plan_set_sha256 differs) — the configuration says that set is thrown"
        )

    header: list[str] = []
    out_rows: list[tuple[UnitRef, dict[str, str]]] = []
    seeds: dict[int, SeedCounts] = {}
    runs: list[dict] = []
    thrown_seeds: dict[int, list[int]] = {}
    caught_seeds: dict[int, list[int]] = {}
    throws: dict[int, dict] = {}
    for unit in done:
        unit_header, rows = read_unit_rows(unit)
        header += [name for name in unit_header if name not in header]
        listed = {
            int(t["throw_id"]): t for t in read_throw_list_file(chunk_files[unit.chunk]).throws
        }
        counts = seeds.setdefault(unit.seed, SeedCounts(unit.seed))
        counts.chunks.append(unit.chunk)
        runs.append(unit_run_record(unit))
        for row in rows:
            throw_id = _throw_id(row, f"unit {unit.name}")
            if throw_id not in listed:
                raise CollectError(
                    f"unit {unit.name}: throw {throw_id} is not in {chunk_files[unit.chunk].name}"
                )
            if unit.seed in thrown_seeds.get(throw_id, ()):
                raise CollectError(
                    f"unit {unit.name}: throw {throw_id} is thrown twice under seed {unit.seed}"
                )
            throws[throw_id] = listed[throw_id]
            thrown_seeds.setdefault(throw_id, []).append(unit.seed)
            if counts.add(row):
                caught_seeds.setdefault(throw_id, []).append(unit.seed)
            out_rows.append((unit, row))
        result.units_done.append(unit.name)

    clash = [name for name in LEADING_COLUMNS if name != "seed" and name in header]
    if clash:
        raise CollectError(f"{arm}: catching_trials.csv already has the column(s) {clash}")
    # The analyser's own `seed` column is the throw series' seed (empty for a list); the
    # leading one is the estimator's. The analyser's is kept under its tool's name.
    columns = [*LEADING_COLUMNS, *("series_seed" if name == "seed" else name for name in header)]
    sim_throws.parent.mkdir(parents=True, exist_ok=True)
    with open(sim_throws, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        for unit, row in out_rows:
            writer.writerow([arm, unit.seed, unit.chunk, *(row.get(name, "") for name in header)])
    result.sim_throws = sim_throws

    result.seeds = [seeds[seed] for seed in sorted(seeds)]
    result.thrown_ids, result.caught_ids = len(thrown_seeds), len(caught_seeds)
    if not caught_seeds:
        catch_set.unlink(missing_ok=True)
        return result
    caught = [
        {
            **throws[throw_id],
            "caught_seeds": sorted(caught_seeds[throw_id]),
            "thrown_seeds": sorted(thrown_seeds[throw_id]),
        }
        for throw_id in sorted(caught_seeds)
    ]
    meta = catch_set_meta(
        robot=robot,
        arm=arm,
        search=search,
        segment=segment,
        thrown_set=thrown_set,
        sample_meta=sample.meta,
        sample_sha256=sample.sha256,
        chunks_total=len(chunk_files),
        seeds=result.seeds,
        runs=runs,
        thrown_ids=result.thrown_ids,
        caught_ids=result.caught_ids,
    )
    write_throw_list(catch_set, caught, meta)
    result.catch_set = catch_set
    return result
