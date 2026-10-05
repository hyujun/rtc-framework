#!/usr/bin/env python3
"""Test registration gate.

In a package built by CMake a test file runs only if a call in the package's
CMakeLists.txt names it. A file nobody named is still a test to its author and
to a reader: it sits in test/, it is called test_*, it passes when run by hand
-- and `colcon test` reports green without it. Nothing else notices. The build
does not fail, the count of tests does not go down (it never went up), and the
Stop hook's co-update check looks at new `.cpp` under src/ only.

That happened on #711 (2026-10-05): `integrated_bringup/test/
test_catching_keys_tools.py` was written and passed under a direct pytest, and
it was seven commits later that someone saw no `ament_add_pytest_test` named
it. The test run reported in between was green without it.

    Every file test/**/test_*.{cpp,cc,py,sh} of a CMake package is named
    outside a comment by that package's CMakeLists.txt.

"Named" is one of two things:

  - the file name itself (`test/test_foo.py`, `${CMAKE_CURRENT_SOURCE_DIR}/
    test/test_foo.sh`), or
  - the bare stem (`test_foo`), where the file builds its path from a name --
    `test/${name}.cpp`, or `list(TRANSFORM <var> APPEND .cpp)` -- for that
    extension. Four packages register this way (a function or a foreach over
    the stems), and without the second half of the condition the stem of a
    gtest target would also vouch for an unregistered `test_foo.py` beside it.

This reads tokens, it does not evaluate CMake. So it proves that the build file
mentions the test, not that the call it sits in is reached: a registration
inside an `if()` that is false on every machine passes. What it does catch is
the case that happened -- the file was never mentioned at all.

A file that is deliberately run some other way says so in the CMakeLists.txt:

    # test-registration-exempt: test_foo.sh -- <who runs it instead, and why>

The reason is required, and an exemption whose file is gone is itself reported,
so the list cannot outlive what it excuses.

SCOPE, stated rather than implied:
  - `ament_python` packages are not checked: pytest collects every test_*.py
    under them by itself, so there is no registration to forget.
  - Only names of the form test_*. A helper beside the tests (a fixture header,
    `conftest.py`, `lib/assert.sh`) is not a test and is not asked about; a
    helper that is called test_* is, and the fix is its name.
  - Sources that a registered test compiles in with it (EXTRA_SOURCES) are
    named by that call like any other.

Usage:
    validate_test_registration.py            # check, exit 1 on violation
    validate_test_registration.py --list     # print every test file's verdict
    validate_test_registration.py --self-test
"""

from __future__ import annotations

import argparse
import re
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

SKIP_DIRS = {"build", "install", "log", ".git", "__pycache__", ".venv"}

TEST_FILE_RE = re.compile(r"^test_[\w.-]*\.(cpp|cc|py|sh)$")
BUILD_TYPE_RE = re.compile(r"<build_type>\s*([\w-]+)\s*</build_type>")

# `# test-registration-exempt: <file name> -- <reason>`. The separator is
# optional so that a missing reason is reported as that, not as a malformed line.
EXEMPT_RE = re.compile(r"#\s*test-registration-exempt:[ \t]*(\S+)[ \t]*(?:--|—|–)?[ \t]*([^\n]*)")


def strip_cmake_comments(text: str) -> str:
    """Drop `#` comments, keeping quoted strings and the line count."""
    out = []
    for line in text.splitlines():
        in_quote = False
        cut = len(line)
        i = 0
        while i < len(line):
            ch = line[i]
            if ch == "\\" and in_quote:
                i += 2
                continue
            if ch == '"':
                in_quote = not in_quote
            elif ch == "#" and not in_quote:
                cut = i
                break
            i += 1
        out.append(line[:cut])
    return "\n".join(out)


def composes_path_for(code: str, ext: str) -> bool:
    """True when the build file assembles a `<name>.<ext>` path from a variable."""
    return bool(re.search(r"(?:\}|\bAPPEND[ \t]+\"?)\." + re.escape(ext) + r"(?![\w])", code))


def is_named(code: str, file_name: str) -> bool:
    """True when `code` (comments stripped) names the test file `file_name`."""
    stem, ext = file_name.rsplit(".", 1)
    # A whole token on both sides: `hand.cpp` is not named by `left_hand.cpp`,
    # and `test_foo` is not named by `test_foo_extra` or by `test_foo.py`.
    if re.search(r"(?<![\w.-])" + re.escape(file_name) + r"(?![\w.-])", code):
        return True
    if composes_path_for(code, ext):
        return bool(re.search(r"(?<![\w./-])" + re.escape(stem) + r"(?![\w.-])", code))
    return False


def cmake_packages(root: Path) -> list[Path]:
    """Directories holding a package.xml and a CMakeLists.txt, not ament_python."""
    found = []
    for xml in sorted(root.rglob("package.xml")):
        if SKIP_DIRS & set(xml.relative_to(root).parts):
            continue
        pkg = xml.parent
        if not (pkg / "CMakeLists.txt").is_file():
            continue
        m = BUILD_TYPE_RE.search(xml.read_text(encoding="utf-8", errors="replace"))
        if m and m.group(1) == "ament_python":
            continue
        found.append(pkg)
    return found


def test_files(pkg: Path) -> list[Path]:
    test_dir = pkg / "test"
    if not test_dir.is_dir():
        return []
    return sorted(
        p
        for p in test_dir.rglob("*")
        if p.is_file()
        and TEST_FILE_RE.match(p.name)
        and not (SKIP_DIRS & set(p.relative_to(pkg).parts))
    )


def analyse(root: Path) -> tuple[list[str], list[str]]:
    """Return (violations, listing) for every CMake package under `root`."""
    violations: list[str] = []
    listing: list[str] = []
    for pkg in cmake_packages(root):
        cmake = pkg / "CMakeLists.txt"
        cmake_rel = cmake.relative_to(root).as_posix()
        raw = cmake.read_text(encoding="utf-8", errors="replace")
        code = strip_cmake_comments(raw)
        files = test_files(pkg)
        names = {p.name for p in files}

        exempt: dict[str, str] = {}
        for m in EXEMPT_RE.finditer(raw):
            name, reason = m.group(1), m.group(2).strip()
            if not reason:
                violations.append(
                    f"{cmake_rel}: the exemption of {name} gives no reason -- write who runs "
                    f"it instead: `# test-registration-exempt: {name} -- <reason>`"
                )
            if name not in names:
                violations.append(
                    f"{cmake_rel}: exempts {name}, and {pkg.name}/test/ holds no such test "
                    f"file -- drop the exemption, or give it the file's present name"
                )
            exempt[name] = reason

        for path in files:
            rel = path.relative_to(root).as_posix()
            if is_named(code, path.name):
                listing.append(f"  registered  {rel}")
            elif path.name in exempt:
                listing.append(f"  exempt      {rel}  ({exempt[path.name]})")
            else:
                listing.append(f"  UNREGISTERED {rel}")
                violations.append(
                    f"{rel}: nothing in {cmake_rel} names it -- `colcon test` does not run it"
                )
    return violations, listing


# (name, CMake text, test file name, is it named?)
_SELF_TEST_CASES = [
    ("literal path", "ament_add_pytest_test(t test/test_a.py)\n", "test_a.py", True),
    (
        "absolute path",
        "ament_add_test(t COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/test_a.sh)\n",
        "test_a.sh",
        True,
    ),
    ("never mentioned", "ament_add_pytest_test(t test/test_a.py)\n", "test_b.py", False),
    (
        "mentioned in a comment only",
        "# ament_add_pytest_test(t test/test_a.py)\nproject(p)\n",
        "test_a.py",
        False,
    ),
    (
        "a '#' inside a string does not start a comment",
        'ament_add_test(t COMMAND sh -c "echo # ; test/test_a.sh")\n',
        "test_a.sh",
        True,
    ),
    (
        "a longer file name does not vouch for a shorter one",
        "ament_add_gtest(t test/test_left_hand.cpp)\n",
        "hand.cpp",
        False,
    ),
    (
        "a file name does not vouch for one it is the prefix of",
        "ament_add_pytest_test(t test/test_a.py)\n",
        "test_a.py.bak.py",
        False,
    ),
    (
        "stem handed to a function that builds test/${name}.cpp",
        "function(add_t name)\n  ament_add_gtest(${name} test/${name}.cpp)\nendfunction()\n"
        "add_t(test_a)\n",
        "test_a.cpp",
        True,
    ),
    (
        "stem in a list that is TRANSFORMed to .cpp",
        "set(_s test_a test_b)\nlist(TRANSFORM _s APPEND .cpp)\nament_add_gtest(t ${_s})\n",
        "test_b.cpp",
        True,
    ),
    (
        "a stem the build file never lists",
        "function(add_t name)\n  ament_add_gtest(${name} test/${name}.cpp)\nendfunction()\n"
        "add_t(test_a)\n",
        "test_ab.cpp",
        False,
    ),
    (
        "a gtest target's name does not vouch for the .py of the same stem",
        "function(add_t name)\n  ament_add_gtest(${name} test/${name}.cpp)\nendfunction()\n"
        "add_t(test_a)\n",
        "test_a.py",
        False,
    ),
    (
        "a bare stem counts for nothing where no path is built from a name",
        "ament_add_gtest(test_a test/other.cpp)\n",
        "test_a.cpp",
        False,
    ),
]


def _write_synthetic_corpus(root: Path) -> None:
    def package(name: str, cmake: str, build_type: str = "ament_cmake") -> Path:
        pkg = root / name
        (pkg / "test" / "sub").mkdir(parents=True)
        (pkg / "package.xml").write_text(
            f"<package><name>{name}</name><export><build_type>{build_type}</build_type>"
            f"</export></package>\n"
        )
        (pkg / "CMakeLists.txt").write_text(cmake)
        return pkg

    pkg = package(
        "pkg_cmake",
        "ament_add_gtest(t_ok test/test_ok.cpp)\n"
        "ament_add_pytest_test(t_deep test/sub/test_deep.py)\n"
        "# test-registration-exempt: test_elsewhere.sh -- run by the CI job\n"
        "# test-registration-exempt: test_noreason.sh\n"
        "# test-registration-exempt: test_gone.sh -- it was renamed\n",
    )
    for rel in (
        "test/test_ok.cpp",
        "test/sub/test_deep.py",
        "test/test_forgotten.py",
        "test/sub/test_forgotten_deep.cpp",
        "test/test_elsewhere.sh",
        "test/test_noreason.sh",
        # not tests by name: never asked about
        "test/conftest.py",
        "test/fixture.hpp",
        "test/sub/helper.sh",
    ):
        (pkg / rel).write_text("\n")

    # pytest collects these by itself, CMakeLists.txt or not.
    py = package("pkg_python", "project(pkg_python)\n", build_type="ament_python")
    (py / "test" / "test_collected.py").write_text("\n")

    # A package that states no build type is built by its CMakeLists.txt.
    bare = root / "pkg_bare"
    (bare / "test").mkdir(parents=True)
    (bare / "package.xml").write_text("<package><name>pkg_bare</name></package>\n")
    (bare / "CMakeLists.txt").write_text("project(pkg_bare)\n")
    (bare / "test" / "test_bare.cpp").write_text("\n")

    # A copy of a package under build/ is not the package.
    stray = root / "build" / "pkg_cmake"
    (stray / "test").mkdir(parents=True)
    (stray / "package.xml").write_text("<package><name>pkg_cmake</name></package>\n")
    (stray / "CMakeLists.txt").write_text("project(pkg_cmake)\n")
    (stray / "test" / "test_stray.cpp").write_text("\n")


def _synthetic_corpus_failures() -> list[str]:
    """Run the whole gate over a throwaway tree that holds every verdict at once.

    The repository is expected to hold no unregistered test, so it cannot be its
    own positive control.
    """
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        _write_synthetic_corpus(root)
        violations, listing = analyse(root)

    def verdict(rel: str) -> str | None:
        for line in listing:
            parts = line.split()
            if len(parts) >= 2 and parts[1] == rel:
                return parts[0]
        return None

    failures = []
    for rel, want in (
        ("pkg_cmake/test/test_ok.cpp", "registered"),
        ("pkg_cmake/test/sub/test_deep.py", "registered"),
        ("pkg_cmake/test/test_forgotten.py", "UNREGISTERED"),
        ("pkg_cmake/test/sub/test_forgotten_deep.cpp", "UNREGISTERED"),
        ("pkg_cmake/test/test_elsewhere.sh", "exempt"),
        ("pkg_cmake/test/test_noreason.sh", "exempt"),
        ("pkg_cmake/test/conftest.py", None),
        ("pkg_cmake/test/fixture.hpp", None),
        ("pkg_cmake/test/sub/helper.sh", None),
        ("pkg_python/test/test_collected.py", None),
        ("pkg_bare/test/test_bare.cpp", "UNREGISTERED"),
        ("build/pkg_cmake/test/test_stray.cpp", None),
    ):
        got = verdict(rel)
        if got != want:
            failures.append(f"synthetic {rel}: expected verdict {want}, got {got}")

    def reported(*needles: str) -> bool:
        return any(all(n in v for n in needles) for v in violations)

    for needles, what in (
        (("test_forgotten.py", "does not run it"), "the unregistered .py"),
        (
            ("test_forgotten_deep.cpp", "does not run it"),
            "the unregistered .cpp in a subdirectory",
        ),
        (("test_bare.cpp", "does not run it"), "the test of a package with no build type"),
        (("test_noreason.sh", "gives no reason"), "the exemption without a reason"),
        (("test_gone.sh", "holds no such test file"), "the exemption of a file that is gone"),
    ):
        if not reported(*needles):
            failures.append(f"synthetic corpus: {what} was not reported")
    if len(violations) != 5:
        failures.append(
            f"synthetic corpus: expected exactly 5 violations, got {len(violations)}: {violations}"
        )
    return failures


def self_test() -> int:
    failures = []
    for name, cmake, file_name, want in _SELF_TEST_CASES:
        got = is_named(strip_cmake_comments(cmake), file_name)
        if got != want:
            failures.append(f"{name}: expected named={want}, got {got}")

    failures.extend(_synthetic_corpus_failures())

    violations, listing = analyse(REPO_ROOT)
    if violations:
        failures.append(f"repo corpus is not clean: {len(violations)} violation(s)")
    # A gate that found no test file at all would also report a clean corpus.
    if not any(line.split()[0] == "registered" for line in listing):
        failures.append("repo corpus: no registered test file was found at all")

    if failures:
        for f in failures:
            print(f"  FAIL {f}", file=sys.stderr)
        print("validate_test_registration --self-test FAILED", file=sys.stderr)
        return 1
    print("validate_test_registration --self-test: all cases pass")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--list", action="store_true", help="print every test file and its verdict")
    ap.add_argument("--self-test", action="store_true", help="prove the gate still fires")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    violations, listing = analyse(REPO_ROOT)

    if args.list:
        print("test files of the CMake packages:")
        for line in listing:
            print(line)
        return 0

    if violations:
        print("Test registration gate FAILED:", file=sys.stderr)
        for v in violations:
            print(f"  {v}", file=sys.stderr)
        print(
            "\nA test file of a CMake package runs only if the package's CMakeLists.txt\n"
            "names it. Register it (ament_add_gtest / ament_add_pytest_test /\n"
            "ament_add_test). If it is a helper and not a test, give it a name that\n"
            "does not start with test_. If something other than `colcon test` runs it,\n"
            "say so in that CMakeLists.txt:\n"
            "    # test-registration-exempt: <file name> -- <who runs it instead>",
            file=sys.stderr,
        )
        return 1

    n_exempt = sum(1 for line in listing if line.split()[0] == "exempt")
    print(
        f"Test registration gate: OK ({len(listing) - n_exempt} test files registered, "
        f"{n_exempt} exempt)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
