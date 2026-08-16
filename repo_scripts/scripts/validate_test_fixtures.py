#!/usr/bin/env python3
"""Test fixture package-resolution gate (issue #454).

A test fixture that resolves a robot model from a package the CI runner cannot
acquire fails there and *only* there. `integrated_bringup` runs in the
`test_cpp_besteffort` (continue-on-error) lane, so the throw never turns a PR
red; the only symptom is a codecov patch % that reads as "no tests were
written". #452 is what that cost: `test_momentum_observer_wiring` was 13-of-14
red in CI for its whole life, so the joint-order oracle it exists for -- mixing
device-order residuals with pinocchio-order Jacobians yields a finite, smooth,
WRONG answer that no gate and no NaN check catches -- had never run outside one
developer machine.

`agent_docs/testing-debug.md` already required fixtures to resolve models from
`robot_descriptions/robots/<name>/`. The rule existed; the sensor did not.

    A test may resolve a package CI cannot acquire only if it SKIPS when that
    package is absent.

Not "may not resolve it" -- an unacquirable package can still be the only place a
model lives, and an explicit skip is an honest report of that gap rather than an
invisible red. #457 retired the last such fixture (`hand_description`'s ur5e_p1b
is vendored in `robot_descriptions/robots/ur5e_p1b/` now), so the corpus today
resolves nothing it cannot prove. The rule stays because the next one will not
announce itself, and the gate's own firing proof moved to a synthetic corpus in
--self-test rather than depending on a real violation staying alive.

Detection cannot be a literal scan. Before #457 the five parameterised fixtures
reached the package through a struct field:

    cfg.urdf_path = get_package_share_directory(ec.urdf_pkg) + "/" + ec.urdf_rel;

so `get_package_share_directory("hand_description")` appeared in 2 files while 8
resolved it, and a literal-only gate would have passed six of the eight it exists
to catch. So a NON-LITERAL argument counts as unresolvable too: the gate cannot
prove where it points. That is also why those fixtures now name the package
literally -- an `ok` verdict says where the path goes, a `guarded` one only says
the test survives being wrong.

Fixtures live in headers that no CMake source list names, so includes are
followed: a violation inside a `.hpp` is charged to every test `.cpp` that
reaches it, and each of those must carry the guard.

Only code counts -- comments are stripped first, or this file's own prose and
`optional_package.hpp`'s rationale would both register as violations.

SCOPE, stated rather than implied: C++ test sources only. Python tests resolve
packages too (`get_package_share_directory` in launch tests), and today every
one of those names an in-repo package, so extending the gate would change no
verdict -- but that is a fact about the corpus now, not a property the gate
enforces. A Python fixture that reaches outside the workspace would pass this
gate silently. The same holds for a resolution built by string concatenation
rather than passed as an argument.

Usage:
    validate_test_fixtures.py            # check, exit 1 on violation
    validate_test_fixtures.py --list     # print what each test source resolves
    validate_test_fixtures.py --self-test
"""

from __future__ import annotations

import argparse
import re
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# The guard every unresolvable resolution must be paired with.
GUARD_TOKEN = "RTC_SKIP_IF_PACKAGE_MISSING"

# ament_index calls that turn a package name into a filesystem path. A test that
# calls any of these on a package CI lacks throws PackageNotFoundError.
RESOLVE_CALL = re.compile(
    r"\bget_package_(?:share_directory|prefix|share_path)\s*\(\s*([^)]*?)\s*[,)]"
)
STRING_LITERAL = re.compile(r'^"([A-Za-z0-9_]+)"$')

SOURCE_SUFFIXES = (".cpp", ".hpp", ".h", ".cc")
QUOTED_INCLUDE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.MULTILINE)


def strip_comments(text: str) -> str:
    """Blank out // and /* */ comments, preserving line count and string literals."""
    out = []
    i, n = 0, len(text)
    while i < n:
        ch = text[i]
        if ch == '"':
            out.append(ch)
            i += 1
            while i < n:
                out.append(text[i])
                if text[i] == "\\" and i + 1 < n:
                    out.append(text[i + 1])
                    i += 2
                    continue
                if text[i] == '"':
                    i += 1
                    break
                i += 1
            continue
        if ch == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                i += 1
            continue
        if ch == "/" and i + 1 < n and text[i + 1] == "*":
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                if text[i] == "\n":
                    out.append("\n")
                i += 1
            i += 2
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def in_repo_packages(root: Path) -> set[str]:
    """Package names CI can build because they live in this repo."""
    names = set()
    for pkg_xml in root.glob("*/package.xml"):
        m = re.search(r"<name>\s*([A-Za-z0-9_]+)\s*</name>", pkg_xml.read_text())
        if m:
            names.add(m.group(1))
    return names


def deps_repos_packages(root: Path) -> set[str]:
    """Names CI acquires via `vcs import` from deps.repos."""
    deps = root / "deps.repos"
    if not deps.is_file():
        return set()
    return set(re.findall(r"^\s{2}([A-Za-z0-9_-]+):\s*$", deps.read_text(), re.MULTILINE))


def find_resolutions(code: str) -> list[tuple[int, str, bool]]:
    """(line, argument, is_literal) for every package-resolving call in `code`."""
    found = []
    for m in RESOLVE_CALL.finditer(code):
        arg = m.group(1).strip()
        if not arg:
            continue
        line = code.count("\n", 0, m.start()) + 1
        lit = STRING_LITERAL.match(arg)
        found.append((line, lit.group(1) if lit else arg, bool(lit)))
    return found


def collect_test_sources(root: Path) -> list[Path]:
    return sorted(
        p for p in root.glob("*/test/**/*") if p.suffix in SOURCE_SUFFIXES and p.is_file()
    )


def build_include_graph(sources: list[Path]) -> dict[Path, set[Path]]:
    """path -> set of test sources it includes (quoted includes, same test dir)."""
    by_name: dict[tuple[str, str], Path] = {}
    for p in sources:
        by_name[(p.parent.as_posix(), p.name)] = p
    graph: dict[Path, set[Path]] = {p: set() for p in sources}
    for p in sources:
        for inc in QUOTED_INCLUDE.findall(p.read_text(errors="replace")):
            target = by_name.get((p.parent.as_posix(), Path(inc).name))
            if target is not None and target != p:
                graph[p].add(target)
    return graph


def reaching_entries(target: Path, sources: list[Path], graph: dict[Path, set[Path]]) -> set[Path]:
    """Test .cpp files that reach `target` through quoted includes (target itself if .cpp)."""
    entries = set()
    for src in sources:
        if src.suffix not in (".cpp", ".cc"):
            continue
        seen, stack = {src}, [src]
        while stack:
            cur = stack.pop()
            if cur == target:
                entries.add(src)
                break
            for nxt in graph.get(cur, ()):
                if nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
    return entries


def analyse(root: Path) -> tuple[list[str], list[str]]:
    """Return (violations, listing)."""
    acquirable = in_repo_packages(root) | deps_repos_packages(root)
    sources = collect_test_sources(root)
    graph = build_include_graph(sources)

    code_of = {p: strip_comments(p.read_text(errors="replace")) for p in sources}
    # The file that DEFINES the guard necessarily resolves an arbitrary package.
    definers = {p for p, c in code_of.items() if f"define {GUARD_TOKEN}" in c}

    violations: list[str] = []
    listing: list[str] = []

    for src in sources:
        if src in definers:
            continue
        for line, arg, is_literal in find_resolutions(code_of[src]):
            rel = src.relative_to(root).as_posix()
            if is_literal and arg in acquirable:
                listing.append(f"  ok       {rel}:{line}  {arg}")
                continue
            what = f'"{arg}"' if is_literal else f"{arg} (non-literal)"
            entries = reaching_entries(src, sources, graph) or {src}
            unguarded = sorted(
                e.relative_to(root).as_posix() for e in entries if GUARD_TOKEN not in code_of[e]
            )
            if unguarded:
                violations.append(
                    f"{rel}:{line}: resolves {what}, which CI cannot acquire, "
                    f"and these reaching tests do not {GUARD_TOKEN}: {', '.join(unguarded)}"
                )
                listing.append(f"  VIOLATION {rel}:{line}  {what}")
            else:
                listing.append(f"  guarded  {rel}:{line}  {what}")

    return violations, listing


# ── self-test: the gate's own firing proof ───────────────────────────────────
_SELF_TEST_CASES = [
    # (name, code, expect_resolution_count, expect_literal_flags)
    ("literal", 'x = get_package_share_directory("robot_descriptions");', 1, [True]),
    ("non_literal", "x = get_package_share_directory(ec.urdf_pkg);", 1, [False]),
    ("line_comment", '// get_package_share_directory("hand_description")\n', 0, []),
    ("block_comment", '/* get_package_share_directory("hand_description") */\n', 0, []),
    (
        "comment_then_code",
        '// prose\nx = get_package_share_directory("hand_description");',
        1,
        [True],
    ),
    ("prefix_variant", 'x = get_package_prefix("hand_description");', 1, [True]),
    ("string_kept", 'const char* s = "// not a comment";\n', 0, []),
]


def _write_synthetic_corpus(root: Path) -> None:
    """A four-case repo: acquirable / unacquirable+guarded / unacquirable+bare / via-include."""
    test_dir = root / "mypkg" / "test"
    test_dir.mkdir(parents=True)
    (root / "mypkg" / "package.xml").write_text("<package><name>mypkg</name></package>\n")

    # in-repo package, literal -> ok
    (test_dir / "ok.cpp").write_text('x = get_package_share_directory("mypkg");\n')
    # unacquirable, guarded -> accepted
    (test_dir / "guarded.cpp").write_text(
        f'x = get_package_share_directory("elsewhere");\n{GUARD_TOKEN}("elsewhere");\n'
    )
    # unacquirable, bare -> violation
    (test_dir / "bare.cpp").write_text('x = get_package_share_directory("elsewhere");\n')
    # non-literal inside a header, charged to the bare .cpp that includes it
    (test_dir / "fixture.hpp").write_text("x = get_package_share_directory(cfg.pkg);\n")
    (test_dir / "viainclude.cpp").write_text('#include "fixture.hpp"\n')


def _synthetic_corpus_failures() -> list[str]:
    """Prove end-to-end that the gate fires on what it must and accepts what it must.

    The corpus this gate guards is expected to hold zero violations and (since
    #457) zero guarded resolutions, so it cannot serve as its own positive
    control -- asking for one would mean keeping a violation-shaped fixture alive
    forever. A throwaway tree can hold all four cases at once instead.
    """
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        _write_synthetic_corpus(root)
        violations, listing = analyse(root)

    def verdict(name: str) -> str | None:
        for line in listing:
            if f"/{name}:" in line:
                return line.strip().split()[0]
        return None

    failures = []
    for name, want in (
        ("ok.cpp", "ok"),
        ("guarded.cpp", "guarded"),
        ("bare.cpp", "VIOLATION"),
        ("fixture.hpp", "VIOLATION"),
    ):
        got = verdict(name)
        if got != want:
            failures.append(f"synthetic {name}: expected verdict {want}, got {got}")

    charged = [v for v in violations if "viainclude.cpp" in v]
    if not charged:
        failures.append(
            "synthetic fixture.hpp violation was not charged to the including "
            "viainclude.cpp -- the include graph is not being followed"
        )
    if any("guarded.cpp" in v for v in violations):
        failures.append("synthetic guarded.cpp was reported as a violation")
    return failures


def self_test() -> int:
    failures = []
    for name, code, want_n, want_flags in _SELF_TEST_CASES:
        got = find_resolutions(strip_comments(code))
        if len(got) != want_n:
            failures.append(f"{name}: expected {want_n} resolutions, got {len(got)} ({got})")
            continue
        flags = [is_lit for _, _, is_lit in got]
        if flags != want_flags:
            failures.append(f"{name}: expected literal flags {want_flags}, got {flags}")

    failures.extend(_synthetic_corpus_failures())

    # The real corpus must be clean. (Its *shape* proves nothing either way: after
    # #457 it contains no guarded and no unacquirable resolution at all, which is
    # the goal, not a weakness -- which is why the firing proof above is synthetic.)
    violations, _ = analyse(REPO_ROOT)
    if violations:
        failures.append(f"repo corpus is not clean: {len(violations)} violation(s)")

    if failures:
        for f in failures:
            print(f"  FAIL {f}", file=sys.stderr)
        print("validate_test_fixtures --self-test FAILED", file=sys.stderr)
        return 1
    print("validate_test_fixtures --self-test: all cases pass")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--list", action="store_true", help="print every resolution and its verdict")
    ap.add_argument("--self-test", action="store_true", help="prove the gate still fires")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    violations, listing = analyse(REPO_ROOT)

    if args.list:
        acquirable = sorted(in_repo_packages(REPO_ROOT) | deps_repos_packages(REPO_ROOT))
        print(f"acquirable by CI ({len(acquirable)}): {', '.join(acquirable)}")
        print("resolutions in test sources:")
        for line in listing:
            print(line)
        return 0

    if violations:
        print("Test fixture package-resolution gate FAILED (#454):", file=sys.stderr)
        for v in violations:
            print(f"  {v}", file=sys.stderr)
        print(
            f"\nA test that resolves a package CI cannot acquire must skip when it is\n"
            f"absent. Prefer moving the fixture onto a model in\n"
            f"robot_descriptions/robots/ -- that is what #457 did to the last one, and\n"
            f"it is the only outcome that actually runs the path in CI. If the model\n"
            f"genuinely cannot be vendored, make the test GTEST_SKIP when the package\n"
            f'is absent behind a {GUARD_TOKEN}("<pkg>") macro (ament_index has no\n'
            f"non-throwing query, so catch PackageNotFoundError and nothing wider).",
            file=sys.stderr,
        )
        return 1

    print(f"Test fixture package-resolution gate: OK ({len(listing)} resolutions checked)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
