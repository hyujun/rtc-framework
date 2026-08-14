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

    (1) a ROS_DOMAIN_ID claim is owned by exactly one package, and
    (2) a test that opens a participant makes such a claim at all.

(2) is the mirror image of (1) and was the larger hole. Uniqueness only looks
at packages that claimed something; the packages that claim NOTHING all run on
domain 0, together, which is the same shared bus reached by the default instead
of by a bad number. When #401's fix isolated ur5e_bt_coordinator, 15 other
participant-opening targets across five packages were still sitting on 0 -- the
victim had been moved off the bus and the bus left running. Rule (2) is what
`agent_docs/testing-debug.md` already told authors to do; only (1) was enforced.

Claims live in two substrates because packages do. C++/CMake packages spell
theirs as `ament_add_test(ENV ROS_DOMAIN_ID=<n>)`; an `ament_python` package
has no CMakeLists at all and spells it as an `os.environ` assignment in
`test/conftest.py`, which pytest imports before any test module. Both feed one
allocation space, so a python claim can collide with a CMake one and is checked
against it.

Detection for (2) reads the test sources, not CMake: a participant appears when
a test initialises the client library, and in this repo that call is often in a
fixture header that no CMake source list names, so includes are followed. Only
code counts -- prose about `rclcpp::init()` is stripped first. That is not
hypothetical tidiness: on this gate's first run a `//` comment in
`rtc_base/.../thread_config.hpp` and a docstring in a launch test produced five
false positives, the same way the CMake half once flagged the comment that
documents it.

There is no allocation table to keep in sync: the sources ARE the table and
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
# ── Coverage axis (issue #401 follow-up) ──────────────────────────────────
# Uniqueness alone leaves the mirror-image hole open: a package that claims
# NOTHING runs on domain 0, and every other unclaimed package is there too.
# That is the same shared bus the hang needs, reached by the default rather
# than by a bad number, so the gate cannot see it by reading claims. It has
# to ask the opposite question -- "does this test open a participant?" --
# and that answer lives in the test sources, not in CMake.
#
# A DDS participant appears when a test initialises the client library. Both
# spellings are load-bearing: `ament_python` packages have no CMakeLists at
# all, so the CMake substrate above is blind to them by construction.
INIT_RE = re.compile(r"\brclcpp::init\s*\(|\brclpy\.init\s*\(")

# Fixtures do the initialising in this repo (`inject_fixture.hpp`,
# `test_helpers.hpp`), and a header is never named in the CMake source list.
# Without following includes every bt-style suite would look participant-free.
INCLUDE_RE = re.compile(r'^[ \t]*#[ \t]*include[ \t]+"([^"]+)"', re.M)

# Registration vs. source list are different calls for the
# `ament_add_gtest_executable` + `ament_add_gtest_test` pair (one executable,
# several ctest entries -- integrated_bringup's gate_closure_diagnostic), so
# the two are collected separately and joined on the target name.
REGISTER_RE = re.compile(r"\bament_add_(?:gtest|gmock|gtest_test|pytest_test|test)\s*\(")
SOURCED_RE = re.compile(r"\bament_add_(?:gtest|gmock|gtest_executable)\s*\(")
SOURCE_TOKEN_RE = re.compile(r"(?:^|[\s\"])((?:test|src)/[\w./-]+\.(?:cpp|cc|py))")

# `ament_python` has no ENV hook, so the claim has to be made where pytest is
# guaranteed to run it before the test module imports rclpy: conftest.py.
PY_ENV_RE = re.compile(
    r"""os\.environ(?:\.setdefault\(\s*|\[\s*)["']ROS_DOMAIN_ID["']\s*(?:\]\s*=\s*|,\s*)["'](\d+)["']"""
)
PY_ENV_MENTION_RE = re.compile(r"ROS_DOMAIN_ID")

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


# Prose about `rclcpp::init()` is not a call to it. The CMake half of this gate
# learned the same lesson (it flagged the comment documenting itself); the
# source half hit it twice on the first run -- `thread_config.hpp:104` says
# "rclcpp::init() creates before the node exists" in a `//` comment, which
# dragged three rtc_base suites in through the include closure, and
# test_launch_hand_affinity_wiring.py says it inside a docstring.
C_BLOCK_COMMENT_RE = re.compile(r"/\*.*?\*/", re.S)
C_LINE_COMMENT_RE = re.compile(r"//[^\n]*")
PY_DOCSTRING_RE = re.compile(r'"""(?:.|\n)*?"""|\'\'\'(?:.|\n)*?\'\'\'')
PY_LINE_COMMENT_RE = re.compile(r"#[^\n]*")


def strip_code_comments(text: str, suffix: str) -> str:
    """Drop comments and python docstrings before looking for a call.

    Deliberately not a parser: the only cost of over-stripping (a `//` inside a
    string literal) is losing text the patterns above do not look for anyway,
    while under-stripping turns every explanatory comment into a false claim.
    """
    if suffix == ".py":
        return PY_LINE_COMMENT_RE.sub("", PY_DOCSTRING_RE.sub("", text))
    return C_LINE_COMMENT_RE.sub("", C_BLOCK_COMMENT_RE.sub("", text))


def _text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def _call_body(text: str, start: int) -> str:
    """Body of a CMake call whose opening paren sits at `start - 1`."""
    depth, i = 1, start
    while i < len(text) and depth:
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
        i += 1
    return text[start : i - 1]


def _strip_comments(text: str) -> str:
    return "\n".join(strip_comment(line) for line in text.splitlines())


def opens_participant(root: Path, rel: str, _memo: dict | None = None, _seen=()) -> bool:
    """Does this test source -- or a header it pulls in -- start a ROS context?

    Include-following is not optional. This repo puts the `rclcpp::init` in
    fixture headers (`ur5e_bt_coordinator/test/inject_fixture.hpp`), and a
    header never appears in the CMake source list, so a scan that stopped at
    the named .cpp would call the whole suite participant-free.
    """
    memo = _memo if _memo is not None else {}
    if rel in memo:
        return memo[rel]
    if rel in _seen:  # include cycle
        return False
    path = root / rel
    body = strip_code_comments(_text(path), path.suffix)
    if not body:
        memo[rel] = False
        return False
    if INIT_RE.search(body):
        memo[rel] = True
        return True
    here = path.parent
    pkg_root = root / rel.split("/", 1)[0]
    result = False
    for inc in INCLUDE_RE.findall(body):
        for base in (here, pkg_root / "test", pkg_root / "include"):
            cand = (base / inc).resolve()
            if not cand.is_file() or not str(cand).startswith(str(root)):
                continue
            if opens_participant(root, cand.relative_to(root).as_posix(), memo, (*_seen, rel)):
                result = True
            break
        if result:
            break
    memo[rel] = result
    return result


def scan_registrations(root: Path) -> tuple[list[tuple], set[str]]:
    """([(pkg, loc, target, sources, claimed, unresolved)], packages-with-participants).

    `ament_add_gtest_executable` + `ament_add_gtest_test` split one binary
    across several ctest entries, so sources and registrations are collected
    separately and joined on the target name.
    """
    memo: dict[str, bool] = {}
    sources_of: dict[tuple[str, str], tuple[str, ...]] = {}
    calls: list[tuple[str, str, str, tuple[str, ...], bool, bool]] = []
    with_participants: set[str] = set()

    for path in sorted(root.rglob("CMakeLists.txt")):
        if any(part in SKIP_DIRS for part in path.relative_to(root).parts):
            continue
        pkg = owning_package(path, root)
        rel = path.relative_to(root).as_posix()
        raw = _text(path)
        text = _strip_comments(raw)
        for m in SOURCED_RE.finditer(text):
            body = _call_body(text, m.end())
            tokens = body.split()
            if not tokens:
                continue
            sources_of[(pkg, tokens[0])] = tuple(SOURCE_TOKEN_RE.findall(body))
        for m in REGISTER_RE.finditer(text):
            body = _call_body(text, m.end())
            tokens = body.split()
            if not tokens:
                continue
            lineno = text.count("\n", 0, m.start()) + 1
            target = tokens[0]
            srcs = tuple(SOURCE_TOKEN_RE.findall(body)) or sources_of.get((pkg, target), ())
            claimed = bool(ANY_VALUE_RE.search(body))
            unresolved = not srcs and "${" in body
            calls.append((pkg, f"{rel}:{lineno}", target, srcs, claimed, unresolved))

    resolved: list[tuple] = []
    for pkg, loc, target, srcs, claimed, unresolved in calls:
        opening = tuple(s for s in srcs if opens_participant(root, f"{pkg}/{s}", memo))
        if opening:
            with_participants.add(pkg)
        resolved.append((pkg, loc, target, opening, claimed, unresolved))
    return resolved, with_participants


def python_package_claims(root: Path) -> tuple[list[tuple[int, str, str]], list[str]]:
    """Claims and problems for `ament_python` packages, which have no CMakeLists.

    `rtc_digital_twin` runs `rclpy.init()` under pytest with no CMake anywhere
    in the package, so `ament_add_test(ENV ...)` is not available and the text
    substrate above cannot even see the package. pytest imports conftest.py
    before the test module, which is the one place the id can be set early
    enough for rclpy to read it.
    """
    claims: list[tuple[int, str, str]] = []
    problems: list[str] = []
    for manifest in sorted(root.rglob("package.xml")):
        if any(part in SKIP_DIRS for part in manifest.relative_to(root).parts):
            continue
        pkg_dir = manifest.parent
        if "<build_type>ament_python</build_type>" not in _text(manifest):
            continue
        if (pkg_dir / "CMakeLists.txt").exists():
            continue  # covered by the CMake substrate
        pkg = pkg_dir.relative_to(root).as_posix()
        tests = sorted(p for p in pkg_dir.rglob("test/**/*.py") if INIT_RE.search(_text(p)))
        if not tests:
            continue
        conftests = [pkg_dir / "test" / "conftest.py", pkg_dir / "conftest.py"]
        found_here = next((c for c in conftests if c.exists()), None)
        where = tests[0].relative_to(root).as_posix()
        if found_here is None:
            problems.append(
                f"{pkg} is ament_python and its tests call rclpy.init ({where}) but the "
                f"package has no test/conftest.py claiming a ROS_DOMAIN_ID -- there is no "
                f"ament_add_test(ENV ...) here, so conftest.py is where the claim has to go"
            )
            continue
        body = _text(found_here)
        loc = found_here.relative_to(root).as_posix()
        match = PY_ENV_RE.search(body)
        if match:
            claims.append((int(match.group(1)), pkg, loc))
        elif PY_ENV_MENTION_RE.search(body):
            problems.append(
                f"{loc} mentions ROS_DOMAIN_ID but not as a literal "
                f'os.environ["ROS_DOMAIN_ID"] = "<n>" assignment -- this gate reads the '
                f"file as text and cannot evaluate anything else"
            )
        else:
            problems.append(
                f"{pkg} tests call rclpy.init ({where}) but {loc} does not set "
                f'os.environ["ROS_DOMAIN_ID"] -- without it pytest runs on the shared '
                f"default domain 0"
            )
    return claims, problems


def coverage_check(registrations: list[tuple], with_participants: set[str]) -> list[str]:
    """Every test that opens a participant must claim a domain.

    The complement of `check()`: that one asks whether two claims collide, this
    one asks whether a claim exists at all. Domain 0 is what an unclaimed test
    gets, and it is shared with every other unclaimed test running in parallel.
    """
    problems: list[str] = []
    for pkg, loc, target, sources, claimed, unresolved in registrations:
        if claimed:
            continue
        if sources:
            problems.append(
                f"{target} ({loc}) opens a DDS participant via {', '.join(sources)} but "
                f"claims no ROS_DOMAIN_ID -- it runs on the shared default 0 alongside "
                f"every other unclaimed package colcon starts in parallel"
            )
        elif unresolved and pkg in with_participants:
            problems.append(
                f"{target} ({loc}) has a CMake-variable source list this gate cannot "
                f"resolve, and {pkg} does open participants -- spell the sources "
                f"literally or claim a ROS_DOMAIN_ID; the gate must not guess"
            )
    return problems


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

    # ── Coverage axis: the claim that is missing, not the one that collides ──
    def cov(name: str, regs: list, with_participants: set, expect_hits: int) -> None:
        problems = coverage_check(regs, with_participants)
        if len(problems) != expect_hits:
            failures.append(
                f"{name}: expected {expect_hits} problem(s), got {len(problems)}: {problems}"
            )

    # (pkg, loc, target, participant-opening sources, claimed, unresolved)
    cov(
        "participant test with no claim",
        [("p", "a:1", "t", ("test/t.cpp",), False, False)],
        {"p"},
        1,
    )
    cov(
        "participant test with a claim",
        [("p", "a:1", "t", ("test/t.cpp",), True, False)],
        {"p"},
        0,
    )
    # A pure-unit test needs no id; demanding one everywhere would make the gate
    # noise and train the next author to add ENV without reading why.
    cov("non-participant test with no claim", [("p", "a:1", "t", (), False, False)], set(), 0)
    # Unresolvable sources are only a problem where participants exist -- there
    # the gate cannot tell, and "cannot tell" must not read as "clean".
    cov(
        "unresolved sources in a participant package",
        [("p", "a:1", "t", (), False, True)],
        {"p"},
        1,
    )
    cov("unresolved sources elsewhere", [("q", "a:1", "t", (), False, True)], {"p"}, 0)

    # Both client libraries count: ament_python packages have no CMakeLists, so
    # the rclpy spelling is the only signal there.
    for probe in (
        "  rclcpp::init(argc, argv);",
        "rclpy.init(args=args)",
        "rclcpp::init (0, nullptr)",
    ):
        if not INIT_RE.search(probe):
            failures.append(f"INIT_RE does not match probe {probe!r}")
    # Prose, not calls. Both of these were live false positives on the first run:
    # a `//` comment in thread_config.hpp pulled in three rtc_base suites through
    # the include closure, and a docstring did the same to a launch test.
    if INIT_RE.search(
        strip_code_comments("// rclcpp::init() creates before the node exists.", ".hpp")
    ):
        failures.append("a C++ comment mentioning rclcpp::init was read as a call")
    _q = '"' * 3
    _docstring_probe = "\n".join(["x = 1", _q, "the DDS threads rclcpp::init() creates", _q])
    if INIT_RE.search(strip_code_comments(_docstring_probe, ".py")):
        failures.append("a python docstring mentioning rclcpp::init was read as a call")
    if INIT_RE.search(strip_code_comments("# rclpy.init() happens in conftest", ".py")):
        failures.append("a python comment mentioning rclpy.init was read as a call")
    if not INIT_RE.search(
        strip_code_comments("  rclcpp::init(argc, argv);  // start here", ".cpp")
    ):
        failures.append("stripping a trailing comment dropped a real init")
    # A commented-out include must not drag its header into the closure.
    if INCLUDE_RE.search(strip_code_comments('// #include "inject_fixture.hpp"', ".cpp")):
        failures.append("a commented-out include was followed")

    # The two call families must not be confused: gtest_test registers without
    # sources, gtest_executable carries sources without registering.
    if not REGISTER_RE.search("ament_add_gtest_test(t TEST_NAME x)"):
        failures.append("REGISTER_RE misses ament_add_gtest_test")
    if REGISTER_RE.search("ament_add_gtest_executable(t test/t.cpp)"):
        failures.append("REGISTER_RE matched ament_add_gtest_executable")
    if SOURCED_RE.search("ament_add_gtest_test(t TEST_NAME x)"):
        failures.append("SOURCED_RE matched ament_add_gtest_test")
    if not SOURCED_RE.search("ament_add_gtest_executable(t test/t.cpp)"):
        failures.append("SOURCED_RE misses ament_add_gtest_executable")
    if _call_body("f(a b (c) d) tail", 2) != "a b (c) d":
        failures.append("_call_body did not balance nested parens")
    if SOURCE_TOKEN_RE.findall("ament_add_gtest(t test/t.cpp src/impl.cpp ENV X=1)") != [
        "test/t.cpp",
        "src/impl.cpp",
    ]:
        failures.append("SOURCE_TOKEN_RE did not pick up the source list")

    for probe in (
        'os.environ["ROS_DOMAIN_ID"] = "58"',
        "os.environ.setdefault('ROS_DOMAIN_ID', '58')",
    ):
        if not PY_ENV_RE.search(probe) or PY_ENV_RE.search(probe).group(1) != "58":
            failures.append(f"PY_ENV_RE does not read probe {probe!r}")
    if PY_ENV_RE.search('os.environ["ROS_DOMAIN_ID"] = str(DOMAIN)'):
        failures.append("PY_ENV_RE accepted a non-literal python claim")

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
    # ament_python packages claim in conftest.py, and their ids share the one
    # allocation space -- merge before checking uniqueness, or a python claim
    # and a CMake claim could collide unseen.
    py_claims, py_problems = python_package_claims(root)
    for domain, pkg, loc in py_claims:
        found.setdefault(domain, []).append((pkg, loc, 0))
    registrations, with_participants = scan_registrations(root)
    covered = sum(1 for r in registrations if r[3] and r[4])

    if args.list:
        print(render_table(found))
        print(f"  participant-opening test targets under a claim: {covered}")
        return 0

    problems = check(found, non_literal) + py_problems
    problems += coverage_check(registrations, with_participants)
    if problems:
        print("Test DDS domain allocation (issue #401) -- violations:", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        print("\ncurrent allocation:\n" + render_table(found), file=sys.stderr)
        return 1

    print(
        f"Test DDS domain allocation: {len(found)} domain(s) claimed, no collisions; "
        f"{covered} participant-opening test target(s) covered."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
