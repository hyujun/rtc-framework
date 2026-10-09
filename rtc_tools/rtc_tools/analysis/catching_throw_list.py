"""The throw list file (``catching_throw_list/1``): one format for every tool that writes
or throws a list of launches.

An offline tool writes the launches it judged; the sim trial driver
(``integrated_bringup.catching_sim_trials --throws-file``) throws them in file
order. Both read and write through this module, so a file one side writes is a
file the other side accepts.

The document::

    {"schema": "catching_throw_list/1", "frame": "sim_world", "meta": {...},
     "throws": [{"throw_id": 0, "pos": [x, y, z], "vel": [vx, vy, vz],
                 "omega": [wx, wy, wz], "kind": "...", ...}, ...]}

``pos`` / ``vel`` / ``omega`` are SIM WORLD m, m/s, rad/s. ``throws`` is not
empty; every entry has an integer ``throw_id`` unique in the file (the key that
joins an offline verdict to a sim trial), finite 3-vectors ``pos`` and ``vel``,
optional ``omega`` (default zero) and optional string ``kind`` (default
``"list"``). Any other key is carried through untouched into the sim trial's
record, except :data:`THROW_RECORD_KEYS`, which are refused.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

THROW_LIST_SCHEMA = "catching_throw_list/1"
THROW_LIST_FRAME = "sim_world"
# Keys the sim driver's trial record writes itself, so a list entry may not
# carry them: its value would be silently overwritten in ``trial_results.json``.
THROW_RECORD_KEYS = frozenset(
    {
        "idx",
        "accepted",
        "seed",
        "message",
        "truth_csv",
        "final_outcome",
        "outcome",
        "cycle_closed",
        "final_mode",
        "launch_wall_time",
        "tick_min",
        "tick_max",
        "wall_t_relative_offset",
        "mode_log",
        "n_truth_rows",
        "controller_mirror",
        "armed_at_throw",
        "home_start_q",
        "home_total_s",
        "q_at_throw",
        "err_q_at_throw",
        "err_qd_at_throw",
        "aim_error_m",
        "aim_pass_after_first_truth_s",
        "aim_pass_speed_m_s",
        "free_flight_samples",
        "first_contact_after_first_truth_s",
        "model_rms_m",
        "host_rtf_min",
        "host_lag_max_ms",
        "host_busy",
    }
)
# The keys a parsed throw always has, in this order; every other key follows.
THROW_LIST_KNOWN_KEYS = frozenset({"throw_id", "pos", "vel", "omega", "kind"})


def _list_vec3(value, where: str) -> tuple[float, float, float]:
    """``value`` as three finite floats, or ``ValueError`` naming ``where``."""
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{where} must be a list of exactly 3 numbers, got {value!r}")
    out = []
    for v in value:
        if isinstance(v, bool) or not isinstance(v, int | float):
            raise ValueError(f"{where} must hold numbers, got {v!r}")
        if not math.isfinite(v):
            raise ValueError(f"{where} must be finite, got {v!r}")
        out.append(float(v))
    return (out[0], out[1], out[2])


def parse_throw_list(doc, source: str) -> tuple[list[dict], dict]:
    """``(throws, meta)`` of a decoded throw-list document; ``source`` names it in errors.

    Each throw is ``throw_id``, ``kind``, ``pos`` / ``vel`` / ``omega`` as float
    3-tuples, then every other key of the entry untouched. Raises ``ValueError``
    naming ``source``, the entry and the offending value.
    """
    if not isinstance(doc, dict):
        raise ValueError(f"{source}: the top level must be an object, got {type(doc).__name__}")
    if doc.get("schema") != THROW_LIST_SCHEMA:
        raise ValueError(
            f"{source}: schema must be {THROW_LIST_SCHEMA!r}, got {doc.get('schema')!r}"
        )
    if doc.get("frame") != THROW_LIST_FRAME:
        raise ValueError(f"{source}: frame must be {THROW_LIST_FRAME!r}, got {doc.get('frame')!r}")
    meta = doc.get("meta", {})
    if not isinstance(meta, dict):
        raise ValueError(f"{source}: meta must be an object, got {type(meta).__name__}")
    entries = doc.get("throws")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"{source}: throws must be a non-empty list")
    throws: list[dict] = []
    seen: set[int] = set()
    for i, entry in enumerate(entries):
        where = f"{source}: throws[{i}]"
        if not isinstance(entry, dict):
            raise ValueError(f"{where} must be an object, got {type(entry).__name__}")
        tid = entry.get("throw_id")
        if isinstance(tid, bool) or not isinstance(tid, int):
            raise ValueError(f"{where}.throw_id must be an integer, got {tid!r}")
        if tid in seen:
            raise ValueError(f"{where}.throw_id {tid} is duplicated")
        seen.add(tid)
        for key in ("pos", "vel"):
            if key not in entry:
                raise ValueError(f"{where} has no {key!r}")
        kind = entry.get("kind", "list")
        if not isinstance(kind, str):
            raise ValueError(f"{where}.kind must be a string, got {kind!r}")
        clash = sorted(THROW_RECORD_KEYS.intersection(entry))
        if clash:
            raise ValueError(f"{where} carries {clash}, keys the trial record writes itself")
        throw = {
            "throw_id": tid,
            "kind": kind,
            "pos": _list_vec3(entry["pos"], f"{where}.pos"),
            "vel": _list_vec3(entry["vel"], f"{where}.vel"),
            "omega": _list_vec3(entry.get("omega", [0.0, 0.0, 0.0]), f"{where}.omega"),
        }
        throw.update((k, v) for k, v in entry.items() if k not in THROW_LIST_KNOWN_KEYS)
        throws.append(throw)
    return throws, meta


@dataclass(frozen=True)
class ThrowListFile:
    """One read of a throw-list file: what it holds and which bytes it was."""

    path: str  # absolute
    throws: list[dict]
    meta: dict
    sha256: str  # of the file's bytes

    def record(self) -> dict:
        """The file's identity for a run's metadata: path, sha256, throw count, ``meta``."""
        return {
            "path": self.path,
            "sha256": self.sha256,
            "n_throws": len(self.throws),
            "meta": self.meta,
        }


def read_throw_list_file(path: str | os.PathLike) -> ThrowListFile:
    """Read and validate a throw-list file once. ``OSError`` if it cannot be read,
    ``ValueError`` (naming the file) if it is not a valid ``catching_throw_list/1``."""
    raw = Path(path).read_bytes()
    try:
        doc = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path}: not a UTF-8 JSON document ({exc})") from exc
    throws, meta = parse_throw_list(doc, str(path))
    return ThrowListFile(os.path.abspath(path), throws, meta, hashlib.sha256(raw).hexdigest())


def load_throw_list(path: str | os.PathLike) -> tuple[list[dict], dict]:
    """The throws of a throw-list file, in file order, and its ``meta``
    (:func:`parse_throw_list` says what a throw holds)."""
    listed = read_throw_list_file(path)
    return listed.throws, listed.meta


def write_throw_list(
    path: str | os.PathLike, throws: Sequence[Mapping], meta: Mapping | None = None
) -> None:
    """Write ``throws`` as a throw-list file that :func:`load_throw_list` reads back equal.

    Floats are written at ``repr`` precision (``json`` default), so ``pos`` /
    ``vel`` / ``omega`` round-trip bit for bit. The document is validated the
    way the loader validates it, before anything is written.
    """
    doc = {
        "schema": THROW_LIST_SCHEMA,
        "frame": THROW_LIST_FRAME,
        "meta": dict(meta or {}),
        "throws": [
            {k: list(v) if isinstance(v, tuple) else v for k, v in throw.items()}
            for throw in throws
        ],
    }
    parse_throw_list(json.loads(json.dumps(doc)), str(path))
    Path(path).write_text(json.dumps(doc, indent=2), encoding="utf-8")
