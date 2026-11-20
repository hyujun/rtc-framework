"""Write a copy of an MJCF with the inertia of chosen bodies scaled.

A robustness run wants a plant that differs from the controller's model by a
known amount and in nothing else: the same bodies, joints, geometry and
sensors, one body some percent heavier. This scales the ``<inertial>`` of each
named body — mass and inertia tensor by the same factor, so the body keeps its
centre of mass and its shape of inertia (a denser body, not a different one).

It is a parameter change, never a structural one: no body is added or removed,
and a body without an explicit ``<inertial>`` is refused rather than guessed at.
``model_match`` on the result shows the mass it moved; a run on it is valid only
in an arm whose registration widens the model tolerance for exactly that.

The copy keeps the source's relative paths (meshdir, includes), so write it
next to the source.
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

INERTIA_ATTRIBUTES = ("mass", "diaginertia", "fullinertia")


def perturb(source: Path, out: Path, factors: dict[str, float]) -> dict[str, float]:
    """Scale each body of `factors` ({name: factor}); returns {name: mass added [kg]}."""
    for name, factor in factors.items():
        if not factor > 0.0:
            raise ValueError(f"{name}: the factor must be > 0, got {factor}")
    tree = ET.parse(source)
    bodies = {b.get("name"): b for b in tree.getroot().iter("body")}
    added: dict[str, float] = {}
    for name, factor in factors.items():
        if name not in bodies:
            raise ValueError(f"{source}: no body {name!r}")
        inertial = bodies[name].find("inertial")
        if inertial is None or inertial.get("mass") is None:
            raise ValueError(f"{name}: no explicit <inertial mass=…> to scale")
        mass = float(inertial.get("mass"))
        for attribute in INERTIA_ATTRIBUTES:
            value = inertial.get(attribute)
            if value is not None:
                inertial.set(attribute, " ".join(repr(float(x) * factor) for x in value.split()))
        added[name] = mass * (factor - 1.0)
    note = ", ".join(f"{n} x {f!r}" for n, f in factors.items())
    out.write_text(
        f"<!-- {out.name}: {source.name} with the inertia of {note} "
        "(rtc_tools perturb_mjcf); do not edit -->\n"
        + ET.tostring(tree.getroot(), "unicode")
        + "\n"
    )
    return added


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("source", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument(
        "--scale", action="append", required=True, metavar="BODY=FACTOR", help="repeatable"
    )
    args = p.parse_args(argv)
    try:
        factors = {b: float(f) for b, f in (item.split("=", 1) for item in args.scale)}
        added = perturb(args.source, args.out, factors)
    except ValueError as exc:
        print(f"perturb_mjcf: {exc}", file=sys.stderr)
        return 2
    for name, kg in added.items():
        print(f"{name}: {kg:+.6f} kg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
