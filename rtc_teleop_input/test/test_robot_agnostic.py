"""ARCH-1: the package names no robot, body or controller of a consumer."""

from __future__ import annotations

import re
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
# built from parts so that this file does not match itself
_WORDS = ("g" + "1", "b" + "7", "ur" + "5e", "loco" + "motion", "demo" + "_joint")
# Bounded by letters and digits only, not by \b: "_" is a word character, so
# \b would let a controller name like "<word>_controller" through.
PATTERN = re.compile(r"(?<![A-Za-z0-9])(" + "|".join(_WORDS) + r")(?![A-Za-z0-9])", re.IGNORECASE)


def test_no_robot_names():
    hits = []
    for path in sorted(PACKAGE.rglob("*")):
        if not path.is_file() or "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        for number, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
            if PATTERN.search(line):
                hits.append(f"{path.relative_to(PACKAGE)}:{number}: {line.strip()}")
    assert hits == []
