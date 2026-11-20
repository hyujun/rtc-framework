"""Cost-weight overlays for a sim run: scale groups of MPC weights by ``10**theta``.

A sim run overrides controller parameters with ONE ROS 2 params file
(``sim_overlay:=``). This tool writes that file for a weight sweep. A *group
definition* names which weight leaves of the shipped controller YAML move
together; a theta per group says by how many decades. Nothing about a robot,
a joint or a weight value is in this module: the group definition names the
leaves and the shipped YAML supplies every number.

Group definition (YAML or JSON)::

    node: <ROS node name the overlay addresses>
    controller: <top-level key of the shipped YAML that holds the controller>
    root: a.b.c            # optional: dotted path under it that every group path is relative to
    groups:
      <group>:
        - state_weights.base_orientation       # a scalar leaf, or a list: every element
        - state_weights.base_orientation[2]    # one element of a list
        - {map: state_weights.joints, keys: [j1, j2]}   # per-joint map: named entries
        - {map: state_weights.joints, keys: all}        # every entry of the map

Output value of every leaf of a group: ``shipped * 10**theta``. A group with
theta = 0 writes NO key (unless ``write_all``), so theta = 0 everywhere is the
shipped configuration by construction, not by multiplying by 1.0. A list is
overridden whole by ROS, so a list with one scaled element is written whole,
the other elements at their shipped value. Values are written as YAML floats
with ``repr`` digits: the ROS parameter type is fixed by the YAML scalar type,
and an int where the controller reads a double changes the type.

One group is the *gauge*: it is held at theta = 0 (refused otherwise), because
scaling every weight by the same factor changes the cost scale and not the
optimum's direction, so that direction is not a free parameter.

Refused (exit 2): a path or joint the shipped YAML lacks, a leaf in two
groups, theta != 0 on the gauge, a non-finite or non-numeric shipped leaf, a
theta for an undefined group, and (merging) one leaf written to two different
values by two components.

``write_all`` writes every leaf of every group at its theta value (theta = 0:
the shipped number as a float). It exists to push the whole weight set through
the controller's override path in a test; a sweep does not use it.

Beside the overlay, ``<overlay stem>.meta.json`` records the thetas, the gauge,
the group definition's sha256, the shipped YAML's path and sha256, and the
components merged.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

PARAMS_KEY = "ros__parameters"


class WeightOverlayError(ValueError):
    """A refusal; the message names what was wrong."""


# ── group definition ─────────────────────────────────────────────────────────

# a leaf: the key path inside the controller section (root included) and, for a
# list element, its index
Leaf = tuple[tuple[str, ...], int | None]
_ELEMENT = re.compile(r"^(?P<key>[^\[\]]+)\[(?P<idx>\d+)\]$")


@dataclass(frozen=True)
class GroupDef:
    node: str
    controller: str
    root: tuple[str, ...]
    groups: dict[str, list[Leaf]]  # in definition order, leaves expanded
    sha256: str


def sha256_of(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_mapping(path: Path) -> dict:
    # YAML 1.2 is a JSON superset for what is used here, so one loader reads both.
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise WeightOverlayError(f"{path}: not a mapping")
    return data


def _walk(tree: Any, keys: tuple[str, ...], where: str) -> Any:
    cur = tree
    for i, k in enumerate(keys):
        if not isinstance(cur, dict) or k not in cur:
            raise WeightOverlayError(
                f"{where}: the shipped YAML has no key '{'.'.join(keys[: i + 1])}'"
            )
        cur = cur[k]
    return cur


def _split(path: str) -> tuple[str, ...]:
    parts = tuple(path.split("."))
    if not path or any(not p for p in parts):
        raise WeightOverlayError(f"malformed key path '{path}'")
    return parts


def _expand_entry(entry: Any, root_tree: Any, root: tuple[str, ...], group: str) -> list[Leaf]:
    """One group-definition entry → its leaves, checked against the shipped tree."""
    where = f"group '{group}'"
    if isinstance(entry, dict):
        if set(entry) != {"map", "keys"}:
            raise WeightOverlayError(f"{where}: an entry mapping needs exactly 'map' and 'keys'")
        base = _split(str(entry["map"]))
        node = _walk(root_tree, base, where)
        if not isinstance(node, dict):
            raise WeightOverlayError(f"{where}: '{entry['map']}' is not a map in the shipped YAML")
        keys = entry["keys"]
        if keys == "all":
            keys = list(node)
        elif not isinstance(keys, list) or not all(isinstance(k, str) for k in keys):
            raise WeightOverlayError(
                f"{where}: 'keys' of '{entry['map']}' is 'all' or a name list"
            )
        out: list[Leaf] = []
        for k in keys:
            if k not in node:
                raise WeightOverlayError(
                    f"{where}: the shipped map '{'.'.join(root + base)}' has no entry '{k}'"
                )
            out.extend(_expand_path(base + (k,), root_tree, where))
        return out
    if not isinstance(entry, str):
        raise WeightOverlayError(f"{where}: an entry is a key path string or a {{map, keys}}")
    m = _ELEMENT.match(entry.rsplit(".", 1)[-1])
    if m:
        head = entry.rsplit(".", 1)[0] + "." if "." in entry else ""
        path = _split(head + m["key"])
        node = _walk(root_tree, path, where)
        idx = int(m["idx"])
        if not isinstance(node, list) or idx >= len(node):
            raise WeightOverlayError(f"{where}: '{entry}' is not an element of a shipped list")
        return [(path, idx)]
    return _expand_path(_split(entry), root_tree, where)


def _expand_path(path: tuple[str, ...], root_tree: Any, where: str) -> list[Leaf]:
    node = _walk(root_tree, path, where)
    if isinstance(node, list):
        return [(path, i) for i in range(len(node))]
    if isinstance(node, dict):
        raise WeightOverlayError(
            f"{where}: '{'.'.join(path)}' is a map; name its entries with {{map, keys}}"
        )
    return [(path, None)]


def load_group_def(path: Path, shipped: dict) -> GroupDef:
    """Read and resolve a group definition against the shipped YAML (refusals: module docstring)."""
    raw = _load_mapping(path)
    for key in ("node", "controller", "groups"):
        if key not in raw:
            raise WeightOverlayError(f"{path}: missing '{key}'")
    controller = str(raw["controller"])
    if controller not in shipped or not isinstance(shipped[controller], dict):
        raise WeightOverlayError(f"the shipped YAML has no controller section '{controller}'")
    root = _split(str(raw["root"])) if raw.get("root") else ()
    tree = _walk(shipped[controller], root, "root")
    if not isinstance(raw["groups"], dict) or not raw["groups"]:
        raise WeightOverlayError(f"{path}: 'groups' must be a non-empty mapping")
    groups: dict[str, list[Leaf]] = {}
    owner: dict[Leaf, str] = {}
    for name, entries in raw["groups"].items():
        if not isinstance(entries, list) or not entries:
            raise WeightOverlayError(f"group '{name}': needs a non-empty list of entries")
        leaves: list[Leaf] = []
        for e in entries:
            for leaf in _expand_entry(e, tree, root, str(name)):
                if leaf in leaves:
                    continue  # the same leaf listed twice in one group
                if leaf in owner:
                    raise WeightOverlayError(
                        f"leaf {_leaf_name(leaf)} is in two groups: '{owner[leaf]}' and '{name}'"
                    )
                owner[leaf] = str(name)
                leaves.append(leaf)
        groups[str(name)] = leaves
    return GroupDef(str(raw["node"]), controller, root, groups, sha256_of(path))


def _leaf_name(leaf: Leaf) -> str:
    path, idx = leaf
    return ".".join(path) + ("" if idx is None else f"[{idx}]")


# ── generation ───────────────────────────────────────────────────────────────


def _shipped_value(tree: Any, leaf: Leaf) -> float:
    path, idx = leaf
    v = _walk(tree, path, "leaf")
    if idx is not None:
        v = v[idx]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise WeightOverlayError(f"shipped leaf {_leaf_name(leaf)} is not a finite number: {v!r}")
    return float(v)


def _set_nested(doc: dict, keys: tuple[str, ...], value: Any) -> None:
    cur = doc
    for k in keys[:-1]:
        cur = cur.setdefault(k, {})
    cur[keys[-1]] = value


def build_overlay(
    shipped: dict,
    gdef: GroupDef,
    theta: dict[str, float],
    gauge: str,
    *,
    write_all: bool = False,
) -> dict:
    """The params-file document for ``theta`` (groups absent from it are at 0)."""
    if gauge not in gdef.groups:
        raise WeightOverlayError(f"the gauge group '{gauge}' is not defined")
    for name, t in theta.items():
        if name not in gdef.groups:
            raise WeightOverlayError(f"theta given for undefined group '{name}'")
        if not isinstance(t, (int, float)) or isinstance(t, bool) or not math.isfinite(t):
            raise WeightOverlayError(f"theta of '{name}' is not a finite number: {t!r}")
    if theta.get(gauge, 0.0) != 0.0:
        raise WeightOverlayError(
            f"theta of the gauge group '{gauge}' must be 0 (got {theta[gauge]!r})"
        )
    tree = _walk(shipped[gdef.controller], gdef.root, "root")
    # target path -> (shipped value(s) as floats, indices that are written)
    out_values: dict[tuple[str, ...], Any] = {}
    for name, leaves in gdef.groups.items():
        t = float(theta.get(name, 0.0))
        for leaf in leaves:
            base = _shipped_value(tree, leaf)  # validates every leaf, written or not
            if t == 0.0 and not write_all:
                continue
            try:
                value = base if t == 0.0 else base * (10.0**t)
            except OverflowError:
                value = math.inf
            if not math.isfinite(value):
                raise WeightOverlayError(f"theta {t!r} makes {_leaf_name(leaf)} non-finite")
            path, idx = leaf
            if idx is None:
                out_values[path] = value
            else:
                if path not in out_values:
                    out_values[path] = [float(x) for x in _walk(tree, path, "leaf")]
                out_values[path][idx] = value
    controller: dict = {}
    for path, value in out_values.items():
        _set_nested(controller, gdef.root + path, value)
    params: dict = {gdef.controller: controller} if controller else {}
    return {gdef.node: {PARAMS_KEY: params}}


# ── merge ────────────────────────────────────────────────────────────────────


def _same(a: Any, b: Any) -> bool:
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b, strict=True))
    return type(a) is type(b) and a == b


def _merge_into(dst: dict, src: dict, trail: tuple[str, ...]) -> None:
    for k, v in src.items():
        here = trail + (str(k),)
        if k not in dst:
            dst[k] = v
        elif isinstance(dst[k], dict) and isinstance(v, dict):
            _merge_into(dst[k], v, here)
        elif isinstance(dst[k], dict) or isinstance(v, dict):
            raise WeightOverlayError(f"overlay conflict at '{'.'.join(here)}': a map and a value")
        elif not _same(dst[k], v):
            raise WeightOverlayError(
                f"overlay conflict at '{'.'.join(here)}': {dst[k]!r} and {v!r}"
            )


def merge_overlays(docs: list[dict]) -> dict:
    """Deep-merge params-file documents by node and parameter leaf.

    The same leaf with different values (or types: 5 is not 5.0 to ROS) in two
    documents is an error naming the leaf; identical values merge.
    """
    out: dict = {}
    for d in docs:
        if not isinstance(d, dict):
            raise WeightOverlayError("an overlay is not a mapping")
        _merge_into(out, json.loads(json.dumps(d)), ())
    return out


def dump_overlay(doc: dict) -> str:
    """YAML text; floats keep ``repr`` digits (PyYAML writes ``1.0e-05``, never ``1e-05``)."""
    return yaml.safe_dump(doc, sort_keys=False, default_flow_style=False)


def merge_overlay_files(paths: list[Path], out: Path) -> dict:
    """Merge the overlay files into ``out`` (+ its sidecar); returns the sidecar's content."""
    docs = [_load_mapping(p) for p in paths]
    merged = merge_overlays(docs)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(dump_overlay(merged))
    meta: dict = {"components": [component_entry(p) for p in paths]}
    lifted = [m for m in (_read_sidecar(p) for p in paths) if m and "theta" in m]
    if len(lifted) == 1:  # one weight component: its record is the merged file's record
        meta.update({k: v for k, v in lifted[0].items() if k != "components"})
        meta["components"] = [component_entry(p) for p in paths]
    write_sidecar(out, meta)
    return meta


def component_entry(path: Path) -> dict:
    return {"path": str(Path(path)), "sha256": sha256_of(Path(path))}


def sidecar_path(overlay: Path) -> Path:
    return Path(overlay).with_suffix(".meta.json")


def _read_sidecar(overlay: Path) -> dict | None:
    p = sidecar_path(overlay)
    return json.loads(p.read_text()) if p.is_file() else None


def write_sidecar(overlay: Path, meta: dict) -> None:
    sidecar_path(overlay).write_text(json.dumps(meta, indent=2) + "\n")


# ── physical scale ───────────────────────────────────────────────────────────

WEIGHT_CLASSES = (
    "position",
    "orientation",
    "linear_velocity",
    "angular_velocity",
    "joint_position",
    "joint_velocity",
    "force",
    "torque",
)


@dataclass(frozen=True)
class RobotScale:
    mass_kg: float
    leg_length_m: float
    foot_length_m: float
    foot_width_m: float


def physical_scale(src: RobotScale, dst: RobotScale) -> dict[str, float]:
    """Multiplier of each weight class when porting weights from ``src`` to ``dst``.

    Rule: an error of the same *relative* size costs the same. A weight w on a
    quantity x enters the cost as w * x**2, so with a reference size x_ref per
    class, w_dst / w_src = (C_dst / C_src) / (x_ref_dst / x_ref_src)**2, where C
    is one cost scale shared by all classes (here an energy, M * g * L; a
    common factor never changes which weights are heavier than which, and the
    gauge group removes it from the search anyway). Reference sizes, with the
    same gravity g on both robots and the time scale T = sqrt(L / g):

        position            L                 orientation, joint position   1
        linear velocity     L / T = sqrt(g L) angular velocity, joint velocity   1 / T
        force               M * g             torque (contact moment)       M * g * F

    where M is the total mass, L the leg length and F = sqrt(foot length * foot
    width), the foot's own size (a contact moment is bounded by the force times
    how far the centre of pressure may move). g cancels in every ratio. All four
    inputs must be positive and finite.
    """
    for r in (src, dst):
        for v in (r.mass_kg, r.leg_length_m, r.foot_length_m, r.foot_width_m):
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0.0):
                raise WeightOverlayError(f"a robot scale must be positive and finite: {r!r}")
    m = dst.mass_kg / src.mass_kg
    length = dst.leg_length_m / src.leg_length_m
    foot = math.sqrt(dst.foot_length_m * dst.foot_width_m) / math.sqrt(
        src.foot_length_m * src.foot_width_m
    )
    cost = m * length  # energy M g L
    time = math.sqrt(length)  # T = sqrt(L / g)
    ref = {
        "position": length,
        "orientation": 1.0,
        "linear_velocity": length / time,
        "angular_velocity": 1.0 / time,
        "joint_position": 1.0,
        "joint_velocity": 1.0 / time,
        "force": m,
        "torque": m * foot,
    }
    return {c: cost / (ref[c] ** 2) for c in WEIGHT_CLASSES}


# ── CLI ──────────────────────────────────────────────────────────────────────


def generate(
    shipped_path: Path,
    groups_path: Path,
    theta: dict[str, float],
    gauge: str,
    out: Path,
    merge: list[Path] = (),
    *,
    write_all: bool = False,
) -> dict:
    """Write the overlay and its sidecar; returns the sidecar's content."""
    shipped = _load_mapping(shipped_path)
    gdef = load_group_def(groups_path, shipped)
    own = build_overlay(shipped, gdef, theta, gauge, write_all=write_all)
    merged = merge_overlays([own] + [_load_mapping(p) for p in merge])
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(dump_overlay(merged))
    meta = {
        "theta": {n: float(theta.get(n, 0.0)) for n in gdef.groups},
        "gauge": gauge,
        "write_all": write_all,
        "groups_path": str(groups_path),
        "groups_sha256": gdef.sha256,
        "shipped_path": str(shipped_path),
        "shipped_sha256": sha256_of(shipped_path),
        "components": [
            {
                "path": "generated weights",
                "sha256": hashlib.sha256(dump_overlay(own).encode()).hexdigest(),
            }
        ]
        + [component_entry(p) for p in merge],
    }
    write_sidecar(out, meta)
    return meta


def _parse_theta(items: list[str]) -> dict[str, float]:
    out: dict[str, float] = {}
    for it in items:
        name, sep, val = it.partition("=")
        if not sep or not name:
            raise WeightOverlayError(f"--theta expects GROUP=VALUE, got '{it}'")
        if name in out:
            raise WeightOverlayError(f"--theta given twice for '{name}'")
        try:
            out[name] = float(val)
        except ValueError:
            raise WeightOverlayError(f"--theta {name}: '{val}' is not a number") from None
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Write a sim overlay scaling groups of shipped weights by 10**theta, "
        "and/or merge params overlays (module docstring)."
    )
    p.add_argument("--shipped", type=Path, help="the shipped controller YAML")
    p.add_argument("--groups", type=Path, help="the group definition")
    p.add_argument("--gauge", help="the group held at theta = 0")
    p.add_argument("--theta", action="append", default=[], metavar="GROUP=VALUE")
    p.add_argument("--write-all", action="store_true", help="write every group leaf (tests)")
    p.add_argument(
        "--merge",
        action="append",
        default=[],
        type=Path,
        metavar="OVERLAY",
        help="merge this params overlay in (repeatable); alone (no --shipped): merge only",
    )
    p.add_argument("--out", type=Path, required=True, help="the overlay file to write")
    args = p.parse_args(argv)
    try:
        if args.shipped is None:
            if args.groups or args.gauge or args.theta or args.write_all or len(args.merge) < 1:
                p.error("without --shipped only --merge FILE [FILE ...] --out is accepted")
            merge_overlay_files(args.merge, args.out)
        else:
            if args.groups is None or args.gauge is None:
                p.error("--shipped needs --groups and --gauge")
            generate(
                args.shipped,
                args.groups,
                _parse_theta(args.theta),
                args.gauge,
                args.out,
                args.merge,
                write_all=args.write_all,
            )
    except (WeightOverlayError, OSError, yaml.YAMLError) as e:
        print(f"weight_overlay: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
