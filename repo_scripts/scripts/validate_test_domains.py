#!/usr/bin/env python3
"""Test DDS domain allocation gate (issue #401).

Tests that bring up a ROS node open a DDS participant. `colcon test` runs
PACKAGES in parallel, so two packages sharing a domain put their participants
on the same discovery bus and the same Fast DDS shared-memory port objects
(`/dev/shm/fastrtps_port<N>` + its named mutex). #401 is what that costs: a
suite hung inside rmw endpoint create/destroy until ctest killed it at 60 s,
leaving no result file. Within a package the risk does not arise -- ctest runs
that package's tests sequentially -- so the invariant is per package, not per
target:

    a ROS_DOMAIN_ID claimed in CMakeLists is owned by exactly one package.

There is no allocation table to keep in sync: the CMakeLists ARE the table and
this script derives it. That is deliberate. The bug this gate exists to stop
was not a bad number, it was a hand-copied comment -- udp_hand_driver said
"domains 44-52 are taken" and picked 53, which integrated_bringup already had.
Any doc that restates the numbers rots the same way (AP-DOC-1); `--list` prints
the current allocation instead.

Usage:
    validate_test_domains.py            # check, exit 1 on violation
    validate_test_domains.py --list     # print the derived allocation
    validate_test_domains.py --self-test
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# `ENV ROS_DOMAIN_ID=44)` / `ENV ROS_DOMAIN_ID=52` -- the ament_add_test(ENV ...)
# spelling. Trailing context is deliberately unanchored: the value is followed
# by `)`, a newline, or another ENV assignment depending on the call site.
DOMAIN_RE = re.compile(r"\bENV\s+ROS_DOMAIN_ID=(\d+)")

# Same assignment, value unconstrained. A `set()` variable or generator
# expression is invisible to DOMAIN_RE, so the package would look like it claims
# nothing and its id would look free -- the gate would pass while handing the
# next author a collision. Reject the form instead of trying to evaluate CMake.
ANY_VALUE_RE = re.compile(r"\bENV\s+ROS_DOMAIN_ID=(\S+)")

# ROS 2 computes RTPS ports from the domain id; above 101 the default port
# configuration runs past the ephemeral range on Linux. 0 is the default every
# unisolated process already uses, so claiming it isolates nothing.
MIN_DOMAIN = 1
MAX_DOMAIN = 101

SKIP_DIRS = {"build", "install", "log", ".git", "__pycache__", ".venv"}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def owning_package(cmakelists: Path, root: Path) -> str:
    """Nearest ancestor holding a package.xml -- CMakeLists can be nested."""
    for parent in [cmakelists.parent, *cmakelists.parents]:
        if parent < root or not str(parent).startswith(str(root)):
            break
        if (parent / "package.xml").exists():
            return parent.relative_to(root).as_posix() or "<root>"
    return cmakelists.parent.relative_to(root).as_posix() or "<root>"


def strip_comment(line: str) -> str:
    """Drop a CMake `#` comment. Quoted `#` is not special enough here to be
    worth a parser -- no call site in this repo puts one in a test argument."""
    idx = line.find("#")
    return line if idx < 0 else line[:idx]


def collect(root: Path) -> tuple[dict[int, list[tuple[str, str, int]]], list[tuple[str, str]]]:
    """(domain id -> [(package, file:line, lineno)], [(file:line, raw value)])."""
    found: dict[int, list[tuple[str, str, int]]] = {}
    non_literal: list[tuple[str, str]] = []
    for path in sorted(root.rglob("CMakeLists.txt")):
        if any(part in SKIP_DIRS for part in path.relative_to(root).parts):
            continue
        pkg = owning_package(path, root)
        rel = path.relative_to(root).as_posix()
        for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            # Comments are prose, not claims. Without this the gate flags the
            # very comments that explain it (it flagged its own the first time
            # it ran) and counts commented-out assignments as live ones.
            line = strip_comment(raw)
            for m in DOMAIN_RE.finditer(line):
                found.setdefault(int(m.group(1)), []).append((pkg, f"{rel}:{lineno}", lineno))
            for m in ANY_VALUE_RE.finditer(line):
                value = m.group(1).rstrip(")")
                if not value.isdigit():
                    non_literal.append((f"{rel}:{lineno}", value))
    return found, non_literal


def check(
    found: dict[int, list[tuple[str, str, int]]],
    non_literal: list[tuple[str, str]] = (),
) -> list[str]:
    problems: list[str] = []
    for loc, value in non_literal:
        problems.append(
            f"ROS_DOMAIN_ID={value} at {loc} is not a literal -- this gate reads "
            f"CMakeLists as text, so a variable makes the package look like it "
            f"claims nothing and hands the next author a silent collision"
        )
    for domain in sorted(found):
        sites = found[domain]
        packages = sorted({pkg for pkg, _, _ in sites})
        if len(packages) > 1:
            where = "\n      ".join(f"{loc}  ({pkg})" for pkg, loc, _ in sites)
            problems.append(
                f"ROS_DOMAIN_ID={domain} is claimed by {len(packages)} packages "
                f"({', '.join(packages)}) -- colcon test runs packages in parallel, "
                f"so these share a discovery bus and Fast DDS SHM port:\n      {where}"
            )
        if not (MIN_DOMAIN <= domain <= MAX_DOMAIN):
            problems.append(
                f"ROS_DOMAIN_ID={domain} is outside {MIN_DOMAIN}..{MAX_DOMAIN} "
                f"({sites[0][1]}) -- 0 is the shared default and >101 runs past the "
                f"default RTPS port range"
            )
    return problems


def render_table(found: dict[int, list[tuple[str, str, int]]]) -> str:
    lines = ["  domain  package (targets)", "  ------  ------------------"]
    for domain in sorted(found):
        sites = found[domain]
        pkgs = sorted({pkg for pkg, _, _ in sites})
        lines.append(f"  {domain:>6}  {', '.join(pkgs)} ({len(sites)})")
    # The next id ABOVE the claimed band, not the smallest unclaimed one: the
    # allocation here has always grown upwards from 44, and pointing a reader at
    # "1" would invite an id that reads like an accident next to its neighbours.
    claimed = set(found)
    top = max(claimed) if claimed else MIN_DOMAIN - 1
    free = [d for d in range(top + 1, MAX_DOMAIN + 1) if d not in claimed]
    lines.append(f"  next free: {free[0] if free else '<none>'}")
    return "\n".join(lines)


def self_test() -> int:
    """The gate must be shown to fire; a checker nobody ran against a known
    violation cannot be distinguished from one that always says 'clean'."""
    failures: list[str] = []

    def case(name: str, found: dict, expect_hits: int) -> None:
        problems = check(found)
        if len(problems) != expect_hits:
            failures.append(
                f"{name}: expected {expect_hits} problem(s), got {len(problems)}: {problems}"
            )

    # The #401 shape: one id, two packages.
    case(
        "duplicate across packages",
        {53: [("integrated_bringup", "a:1", 1), ("udp_hand_driver", "b:2", 2)]},
        1,
    )
    # Same id repeated inside ONE package is legitimate: ctest is sequential
    # there. This is the case ur5e_bt_coordinator relies on -- if this ever
    # starts failing, every bt test target has to carry its own number.
    case(
        "repeat within one package",
        {55: [("ur5e_bt_coordinator", "a:1", 1), ("ur5e_bt_coordinator", "a:2", 2)]},
        0,
    )
    case("distinct ids", {44: [("p", "a:1", 1)], 45: [("q", "b:1", 1)]}, 0)
    case("domain 0 claimed", {0: [("p", "a:1", 1)]}, 1)
    case("domain above range", {200: [("p", "a:1", 1)]}, 1)

    # A variable value is a hole, not a claim: DOMAIN_RE cannot see it, so
    # without this the tree would look clean while one package claimed nothing.
    if len(check({}, [("a:1", "${BT_TEST_DOMAIN_ID}")])) != 1:
        failures.append("non-literal ROS_DOMAIN_ID was not reported")
    if check({55: [("p", "a:1", 1)]}, []):
        failures.append("a literal claim was reported as non-literal")

    # Comment stripping: prose about the gate must not be read as a claim. The
    # first run of this gate flagged the comment that documents it.
    if DOMAIN_RE.search(strip_comment("  # ENV ROS_DOMAIN_ID=${SOME_VAR} would be invisible")):
        failures.append("a claim inside a comment was counted")
    if not DOMAIN_RE.search(strip_comment("    ENV ROS_DOMAIN_ID=55)  # bt suites")):
        failures.append("stripping a trailing comment dropped a real claim")

    # The regex must match the real CMake spellings, and must not match prose.
    for probe in ("    ENV ROS_DOMAIN_ID=44)", "ENV ROS_DOMAIN_ID=52", "  ENV  ROS_DOMAIN_ID=7 )"):
        if not DOMAIN_RE.search(probe):
            failures.append(f"pattern does not match probe {probe!r}")
    for anti in ("# domains 44-52 are taken", "ROS_DOMAIN_ID is set by the caller"):
        if DOMAIN_RE.search(anti):
            failures.append(f"pattern matches antiprobe {anti!r}")

    if failures:
        print("validate_test_domains --self-test FAILED", file=sys.stderr)
        for f in failures:
            print(f"  {f}", file=sys.stderr)
        return 1
    print("validate_test_domains --self-test: all cases pass")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="print the derived allocation")
    parser.add_argument("--self-test", action="store_true", help="exercise the checker itself")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()

    root = repo_root()
    found, non_literal = collect(root)

    if args.list:
        print(render_table(found))
        return 0

    problems = check(found, non_literal)
    if problems:
        print("Test DDS domain allocation (issue #401) -- violations:", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        print("\ncurrent allocation:\n" + render_table(found), file=sys.stderr)
        return 1

    print(f"Test DDS domain allocation: {len(found)} domain(s) claimed, no collisions.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
