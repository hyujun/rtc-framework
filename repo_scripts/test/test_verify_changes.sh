#!/usr/bin/env bash
# End-to-end assertions for .claude/hooks/verify-changes.sh routing.
#
# Why end-to-end rather than unit: the hook is a stack of filters, and the
# failure mode that matters is one layer quietly cancelling a fix made in
# another. A previous attempt at this repaired the changed-file routing so
# CMake-only edits reached the co-update gate, and the very next stage --
# is_pure_format(), which returns success on an empty file list -- classified
# them as "pure formatting" and skipped that gate anyway. Every layer looked
# right in isolation. The only question worth asking is the one these tests
# ask: given this input, does the warning actually come out?
#
# Each case builds a throwaway git repository, runs the real hook against it,
# and greps the hook's stderr. Phase 2 (build/test) is suppressed via
# RTC_VERIFY_SKIP_BUILD -- colcon is not available here and is not what is
# under test. The *routing decision* still is, so the hook echoes BUILD_PKGS as
# a probe line before discarding it; cases 13-15 read that.
#
# An assertion here has to be able to fail. Case 7 previously checked for the
# absence of a README reminder that the fixture could never have produced, so it
# held whether the code under it worked or not. Where a case guards a fix, it
# was run against the pre-fix hook to confirm it goes red -- 12 of these do --
# and the ones asserting an absence were checked by severing the mechanism they
# depend on.
#
# Usage: repo_scripts/test/test_verify_changes.sh
set -uo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
HOOK="$REPO_ROOT/.claude/hooks/verify-changes.sh"
PASS=0
FAIL=0
SKIP=0

fail() {
  printf '  FAIL: %s\n' "$1" >&2
  FAIL=$((FAIL + 1))
}
pass() {
  printf '  ok: %s\n' "$1"
  PASS=$((PASS + 1))
}
skip() {
  printf '  skip: %s\n' "$1"
  SKIP=$((SKIP + 1))
}

# The pure-format fast path needs a clang-format the hook can resolve (a system
# binary or uvx). Without one the hook fails closed (PURE_FORMAT=0), so the cases
# that exercise the fast path would assert the wrong thing; declare the
# dependency and skip them loudly rather than emitting a spurious failure.
have_formatter() { command -v clang-format >/dev/null 2>&1 || command -v uvx >/dev/null 2>&1; }

# The YAML parse gate fails OPEN when PyYAML is not importable (a missing tool
# must not wedge every turn). That is correct hook behaviour, but it means the
# one case needing a REAL parse failure cannot be exercised without PyYAML --
# so declare the dependency and skip loudly rather than emit a spurious red on
# a box (or a CI job) that never installed it. CI installs PyYAML precisely so
# this gate is actually tested; see .github/workflows/docs-validate.yml.
have_pyyaml() { python3 -c 'import yaml' >/dev/null 2>&1; }

# Build a minimal repo that looks enough like this one for the hook's package
# discovery (package.xml at a directory root) to work.
make_fixture() {
  local dir
  dir=$(mktemp -d)
  git -C "$dir" init -q
  git -C "$dir" config user.email t@example.com
  git -C "$dir" config user.name test
  mkdir -p "$dir/rtc_demo/src" "$dir/rtc_demo/include" "$dir/agent_docs" "$dir/rtc_demo/config"
  cat >"$dir/rtc_demo/package.xml" <<'XML'
<?xml version="1.0"?>
<package format="3">
  <name>rtc_demo</name>
  <version>0.0.1</version>
  <description>fixture</description>
  <maintainer email="t@example.com">t</maintainer>
  <license>MIT</license>
</package>
XML
  cat >"$dir/rtc_demo/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(rtc_demo)
add_library(rtc_demo src/existing.cpp)
CMAKE
  echo 'int existing() { return 0; }' >"$dir/rtc_demo/src/existing.cpp"
  echo '# demo' >"$dir/rtc_demo/README.md"
  echo '# docs' >"$dir/agent_docs/notes.md"
  git -C "$dir" add -A
  git -C "$dir" commit -qm init
  echo "$dir"
}

# Run the hook inside a fixture and echo its stderr.
run_hook() {
  local dir="$1"
  ( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_SKIP_BUILD=1 \
      bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null )
}

# Phase 2's build branch is the one region SKIP_BUILD cannot reach: blanking
# BUILD_PKGS is exactly what lets the suite run without colcon, so the build
# classification had zero coverage (#435). RTC_VERIFY_BUILD_CMD injects a stub
# in build.sh's place, so a bound-kill (124) and a compile error can be replayed
# deterministically and in a second. $1 = exit code the stub returns.
#
# The stub must FAIL: a stub that succeeded would fall through to the real
# `colcon test` call, which this fixture cannot serve.
make_build_stub() {
  local d rc="$1"
  d=$(mktemp -d)
  cat >"$d/build.sh" <<EOF
#!/usr/bin/env bash
echo "stub-build args: \$*"
echo "-- filler line so the tail is not the whole log --"
echo "error: fixture_file.cpp:7:3: expected ';' before '}'"
exit $rc
EOF
  chmod +x "$d/build.sh"
  echo "$d"
}

# Like run_hook but WITHOUT RTC_VERIFY_SKIP_BUILD, so Phase 2 actually runs and
# routes its build through the stub.
run_hook_build() {
  local dir="$1" stub="$2"
  ( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_BUILD_CMD="$stub/build.sh" \
      bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null )
}

# make_fixture ships one package (rtc_demo), which routes to the per-package
# branch. PROC-3 needs a package named rtc_base or rtc_msgs; the two branches
# had the same defect and must be fixed together, so both get cases.
add_rtc_base() {
  local dir="$1"
  mkdir -p "$dir/rtc_base/src"
  sed 's/rtc_demo/rtc_base/' "$dir/rtc_demo/package.xml" >"$dir/rtc_base/package.xml"
  cat >"$dir/rtc_base/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(rtc_base)
add_library(rtc_base src/base.cpp)
CMAKE
  echo 'int base_fn() { return 0; }' >"$dir/rtc_base/src/base.cpp"
  echo '# base' >"$dir/rtc_base/README.md"
  git -C "$dir" add -A
  git -C "$dir" commit -qm "add rtc_base"
}

expect_contains() {
  local name="$1" haystack="$2" needle="$3"
  if grep -qF -- "$needle" <<<"$haystack"; then
    pass "$name"
  else
    fail "$name -- expected output to mention '$needle', got:
$(sed 's/^/      /' <<<"${haystack:-<empty>}")"
  fi
}

expect_not_contains() {
  local name="$1" haystack="$2" needle="$3"
  if grep -qF -- "$needle" <<<"$haystack"; then
    fail "$name -- output should not mention '$needle', got:
$(sed 's/^/      /' <<<"$haystack")"
  else
    pass "$name"
  fi
}

# The stderr-substring assertions above cannot tell a blocked turn (exit 2) from
# a clean one (exit 0): a hook that wrongly exit-2s on an unrelated path while
# not emitting the checked needle would slip past every expect_not_contains. So
# the block/no-block decision -- the hook's actual contract -- is asserted here.
# Capture `rc=$?` immediately after `out=$(run_hook ...)`.
expect_exit() {
  local name="$1" got="$2" want="$3"
  if [ "$got" -eq "$want" ]; then
    pass "$name"
  else
    fail "$name -- expected exit $want, got $got"
  fi
}

echo "verify-changes.sh end-to-end routing"

# 1. A CMakeLists-only change must reach the package.xml co-update gate.
#    This is the case the double early-exit used to drop: the change that
#    triggers the gate was also the change that never got there.
dir=$(make_fixture)
sed -i 's/^project(rtc_demo)/project(rtc_demo)\nfind_package(fmt REQUIRED)/' "$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir"); rc=$?
expect_contains "CMake-only change reaches the find_package co-update gate" "$out" "find_package(fmt)"
expect_exit "missing package.xml dep blocks the turn" "$rc" 2
rm -rf "$dir"

# 2. Formatting churn alongside a CMake change must not be graded "pure
#    format". is_pure_format() only inspects source files, so a commit whose
#    source edits are cosmetic used to skip Phase 1 wholesale, taking the
#    unrelated CMake gate down with it.
if have_formatter; then
  dir=$(make_fixture)
  printf 'int existing()  {  return 0;  }\n' >"$dir/rtc_demo/src/existing.cpp"
  sed -i 's/^project(rtc_demo)/project(rtc_demo)\nfind_package(fmt REQUIRED)/' "$dir/rtc_demo/CMakeLists.txt"
  out=$(run_hook "$dir")
  expect_contains "reformat + CMake change still runs the co-update gate" "$out" "find_package(fmt)"
  rm -rf "$dir"
else
  skip "reformat + CMake change still runs the co-update gate (no clang-format/uvx)"
fi

# 3. An untracked new .cpp must be seen. `git diff HEAD` cannot see it, and an
#    agent that writes a file without staging it is the normal case, so the
#    "new .cpp missing from CMakeLists" gate was unreachable in practice.
dir=$(make_fixture)
echo 'int fresh() { return 1; }' >"$dir/rtc_demo/src/fresh.cpp"
out=$(run_hook "$dir")
expect_contains "untracked new .cpp is checked against CMakeLists" "$out" "fresh.cpp"
rm -rf "$dir"

# 4. A docs-only change must reach the documentation validator.
dir=$(make_fixture)
printf '# docs\n\nSee [missing](./nope.md).\n' >"$dir/agent_docs/notes.md"
out=$(run_hook "$dir")
expect_contains "docs-only change runs validate_docs" "$out" "nope.md"
rm -rf "$dir"

# 5. A malformed YAML config must be caught; config/ has no other gate at all.
if have_pyyaml; then
  dir=$(make_fixture)
  printf 'a: [1, 2\nb: {\n' >"$dir/rtc_demo/config/broken.yaml"
  git -C "$dir" add -A
  out=$(run_hook "$dir")
  expect_contains "broken YAML is reported" "$out" "broken.yaml"
  rm -rf "$dir"
else
  skip "broken YAML is reported (PyYAML not importable)"
fi

# 6. A non-ASCII filename must not vanish. git C-quotes such paths by default,
#    which breaks every extension regex -- and a file that matches no filter is
#    silently exempt from every check rather than loudly rejected.
dir=$(make_fixture)
echo 'int hangul() { return 2; }' >"$dir/rtc_demo/src/한글.cpp"
out=$(run_hook "$dir")
expect_contains "non-ASCII filename is still checked" "$out" "한글.cpp"
rm -rf "$dir"

# 7. A genuinely pure-format change must still take the fast path, or the gate
#    becomes noise that gets switched off.
#
#    The observable has to be something the fast path actually controls. This
#    assertion used to check that no README reminder appeared -- but the README
#    checklist only fires on include/, launch/, config/, a structural add/delete
#    or package.xml, none of which a src/-only edit touches. It therefore held
#    whether or not PURE_FORMAT was set, and pinned nothing. (Confirmed by
#    forcing PURE_FORMAT=0 with clang-format off PATH: identical output.)
#    Phase 0 IS gated on it, so use ARCH-1: reflowing a line that mentions a
#    robot must not be reported as newly introducing it.
if have_formatter; then
  dir=$(make_fixture)
  printf 'int ur5e_thing()  {  return 0;  }\n' >"$dir/rtc_demo/src/existing.cpp"
  git -C "$dir" commit -qam "robot mention at HEAD"
  printf 'int ur5e_thing() {   return 0;   }\n' >"$dir/rtc_demo/src/existing.cpp"
  out=$(run_hook "$dir")
  expect_not_contains "pure formatting skips the ARCH grep" "$out" "ARCH-1"
  rm -rf "$dir"
else
  skip "pure formatting skips the ARCH grep (no clang-format/uvx)"
fi

# 7b. ...and the control: the same file, changed for real, must still be
#     screened. Without this pair, 7 could pass by the grep being broken.
dir=$(make_fixture)
printf 'int existing() { return 0; }\nint ur5e_specific() { return 1; }\n' \
  >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook "$dir")
expect_contains "a real change carrying a robot name raises ARCH-1" "$out" "ARCH-1"
rm -rf "$dir"

# 8. ARCH-7: a new executable in an rtc_* package is reported.
dir=$(make_fixture)
printf 'add_executable(rtc_demo_node src/existing.cpp)\n' >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir"); rc=$?
expect_contains "ARCH-7 catches a new rtc_* executable" "$out" "ARCH-7"
expect_exit "an ARCH-7 violation blocks the turn" "$rc" 2
rm -rf "$dir"

# 9. ...but a target carrying the same-line ARCH-7-exempt marker is not. The
#    name deliberately is NOT example_-prefixed: otherwise the name rule would
#    exempt it first and this would silently stop testing the marker at all
#    (case 18 covers the name form; case 17 the preceding-line form).
dir=$(make_fixture)
printf 'add_executable(agnostic_demo src/existing.cpp)  # ARCH-7-exempt: standalone tool\n' >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "ARCH-7 respects the same-line exempt marker" "$out" "ARCH-7"
expect_exit "same-line exempt marker does not block" "$rc" 0
rm -rf "$dir"

# 10. ARCH-5 over a multi-line ament_target_dependencies(). The single-line
#     regex a previous attempt used missed this form, which is the one this
#     repository actually writes.
dir=$(make_fixture)
cat >>"$dir/rtc_demo/CMakeLists.txt" <<'CMAKE'
ament_target_dependencies(rtc_demo
  rclcpp
  robot_descriptions
)
CMAKE
out=$(run_hook "$dir")
expect_contains "ARCH-5 catches multi-line ament_target_dependencies" "$out" "ARCH-5"
rm -rf "$dir"

# 11. ARCH-5 over package.xml, including <build_export_depend> which the first
#     attempt omitted entirely.
dir=$(make_fixture)
sed -i 's#</package>#  <build_export_depend>robot_descriptions</build_export_depend>\n</package>#' "$dir/rtc_demo/package.xml"
out=$(run_hook "$dir")
expect_contains "ARCH-5 catches build_export_depend" "$out" "ARCH-5"
rm -rf "$dir"

# 12. The legitimate runtime dependency form must stay silent.
dir=$(make_fixture)
sed -i 's#</package>#  <exec_depend>robot_descriptions</exec_depend>\n</package>#' "$dir/rtc_demo/package.xml"
out=$(run_hook "$dir")
expect_not_contains "ARCH-5 allows exec_depend" "$out" "ARCH-5"
rm -rf "$dir"

# --- Build routing -----------------------------------------------------------
#
# RTC_VERIFY_SKIP_BUILD blanks BUILD_PKGS before Phase 2 so this suite can run
# without a colcon workspace -- which also made build routing structurally
# unobservable, so the headline claim of the untracked scoping had no assertion
# behind it at all. The hook echoes its routing decision as a probe line first;
# these read it.

# 13. Tracked source routes to a build.
dir=$(make_fixture)
printf 'int existing() { return 42; }\n' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook "$dir")
expect_contains "tracked source is routed to a build" "$out" "BUILD_PKGS=[rtc_demo]"
rm -rf "$dir"

# 14. An untracked header under include/ must be BUILT. Excluding untracked
#     files wholesale was the wrong axis: it kept scratch from forcing a
#     rebuild (intended) but also meant a brand-new public header was
#     ARCH-grepped, CMake-gated, and then never compiled.
dir=$(make_fixture)
printf '#pragma once\nint fresh();\n' >"$dir/rtc_demo/include/fresh.hpp"
out=$(run_hook "$dir")
expect_contains "untracked header under include/ is routed to a build" "$out" "BUILD_PKGS=[rtc_demo]"
rm -rf "$dir"

# 15. ...while untracked scratch outside a package source dir still is not.
#     This is the case the exclusion exists for: a probe script under the tree
#     must not trigger a full-workspace rebuild every turn.
dir=$(make_fixture)
printf 'x = 1\n' >"$dir/scratch_probe.py"
out=$(run_hook "$dir")
expect_contains "untracked scratch is not routed to a build" "$out" "BUILD_PKGS=[]"
rm -rf "$dir"

# 15a. A brand-new test source must be BUILT (#358). The location axis replaced
#      the tracked-ness axis precisely because a never-compiled new file reads as
#      green, but the replacement listed src/ and include/ and not test/ -- so the
#      same hole stayed open on the path this repo touches most often. A new test
#      is the case where "the hook was green" is most misleading: the whole point
#      of the turn was the assertions nobody ran.
dir=$(make_fixture)
mkdir -p "$dir/rtc_demo/test"
printf '#include <gtest/gtest.h>\nTEST(F, G) { EXPECT_TRUE(true); }\n' >"$dir/rtc_demo/test/test_fresh.cpp"
out=$(run_hook "$dir")
expect_contains "untracked test source is routed to a build" "$out" "BUILD_PKGS=[rtc_demo]"
rm -rf "$dir"

# 15b. Same for a test-only header under <pkg>/test/include/ -- the never-installed
#      layout sibling packages consume by source-tree path (rtc_base/testing/).
#      Deeper nesting than 15a, so it also pins that the filter keys on $2 rather
#      than on the path depth.
dir=$(make_fixture)
mkdir -p "$dir/rtc_demo/test/include/rtc_demo/testing"
printf '#pragma once\nint fresh();\n' >"$dir/rtc_demo/test/include/rtc_demo/testing/fresh.hpp"
out=$(run_hook "$dir")
expect_contains "untracked test-only header is routed to a build" "$out" "BUILD_PKGS=[rtc_demo]"
rm -rf "$dir"

# 15c. A brand-new launch file must be BUILT (#360). launch/ is DIRECTORY-installed
#      (install(DIRECTORY launch/ ...) in five ament_cmake packages, glob("launch/*.py")
#      in rtc_digital_twin's setup.py), so a new file becomes an installed artifact
#      with no CMakeLists edit -- which also means CHANGED_META_TRACKED does not
#      catch it either. Editing that same file once it is tracked DOES build the
#      package, and nothing justifies the two answers differing.
dir=$(make_fixture)
mkdir -p "$dir/rtc_demo/launch"
printf 'def generate_launch_description():\n    return None\n' \
  >"$dir/rtc_demo/launch/fresh.launch.py"
out=$(run_hook "$dir")
expect_contains "untracked launch file is routed to a build" "$out" "BUILD_PKGS=[rtc_demo]"
rm -rf "$dir"

# 15d. Same for scripts/. Whether a new script needs a CMakeLists edit is a
#      per-package accident -- integrated_bringup installs them one by one with
#      install(PROGRAMS ...) (so the edit routes the package anyway), rtc_math
#      installs the directory (so nothing routes it). The axis must not depend on
#      which of those two a package happens to use.
dir=$(make_fixture)
mkdir -p "$dir/rtc_demo/scripts"
printf 'x = 1\n' >"$dir/rtc_demo/scripts/fresh.py"
out=$(run_hook "$dir")
expect_contains "untracked script is routed to a build" "$out" "BUILD_PKGS=[rtc_demo]"
rm -rf "$dir"

# 15e. ...and the widening stays an allowlist. A .py under <pkg>/config/ is not
#      package source: config/ holds YAML that the parse gate already covers, and
#      admitting it would make the filter "any directory under a package", which is
#      the unbounded reading 15 exists to rule out.
dir=$(make_fixture)
printf 'x = 1\n' >"$dir/rtc_demo/config/fresh.py"
out=$(run_hook "$dir")
expect_contains "untracked .py under config/ is not routed to a build" "$out" "BUILD_PKGS=[]"
rm -rf "$dir"

# --- path-scoped rule glob gate ----------------------------------------------

# 15f. A .claude/rules/*.md whose globs match nothing must BLOCK. A rule that
#      cannot fire is guidance that silently never arrives -- the same shape as
#      the constitution copy that stopped at 7 of 9 RT rules (#213). The gate
#      exists because the runtime side of this (did it load?) is invisible from
#      inside the authoring session.
dir=$(make_fixture)
mkdir -p "$dir/.claude/rules"
printf -- '---\npaths:\n  - "nonexistent_dir/**/*.zzz"\n---\n\n# dead rule\n' \
  >"$dir/.claude/rules/dead.md"
out=$(run_hook "$dir"); rc=$?
expect_contains "a rule whose globs match nothing is reported" "$out" "can never load"
expect_exit "a rule whose globs match nothing blocks the turn" "$rc" 2
rm -rf "$dir"

# 15g. ...while a rule that matches real files is silent. Without this the gate
#      could be a constant blocker and 15f would still pass.
dir=$(make_fixture)
mkdir -p "$dir/.claude/rules"
printf -- '---\npaths:\n  - "**/*.cpp"\n---\n\n# live rule\n' \
  >"$dir/.claude/rules/live.md"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "a rule matching real files is not reported" "$out" "can never load"
expect_exit "a rule matching real files does not block" "$rc" 0
rm -rf "$dir"

# 15h. Redundant anchors are a hedge, not a defect: a rule stays green as long as
#      ONE pattern matches. rt-path.md carries 16 patterns of which 8 match
#      nothing (no .h/.cc in this tree), so per-pattern strictness would block on
#      the repo's own working rule.
dir=$(make_fixture)
mkdir -p "$dir/.claude/rules"
printf -- '---\npaths:\n  - "**/*.cpp"\n  - "**/*.cc"\n---\n\n# partly dead\n' \
  >"$dir/.claude/rules/hedged.md"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "a partially matching rule is not reported" "$out" "can never load"
expect_exit "a partially matching rule does not block" "$rc" 0
rm -rf "$dir"

# --- ARCH-7 target-name scope ------------------------------------------------

# 16. Re-touching an existing target must not read as introducing one. CMake
#     lines get rewritten in place, so added-line scope answered a different
#     question than the rule asks -- and every add_executable in this repo's
#     rtc_* packages predates the rule.
dir=$(make_fixture)
printf 'add_executable(rtc_demo_node src/existing.cpp)\n' >>"$dir/rtc_demo/CMakeLists.txt"
git -C "$dir" commit -qam "existing target at HEAD"
sed -i 's/^add_executable(rtc_demo_node/  add_executable(rtc_demo_node/' "$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir")
expect_not_contains "reindenting an existing target does not fire ARCH-7" "$out" "ARCH-7"
rm -rf "$dir"

# 17. The marker must work where CMake conventionally puts a justification --
#     the line above the call. Same-line-only matching rejected the idiomatic
#     form while accepting a bare marker appended to the call itself.
dir=$(make_fixture)
printf '# ARCH-7-exempt: robot-agnostic standalone tool\nadd_executable(agnostic_tool src/existing.cpp)\n' \
  >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir")
expect_not_contains "ARCH-7 marker is honoured on the preceding line" "$out" "ARCH-7"
rm -rf "$dir"

# 18. example_* is out of scope by name (design-principles.md), so examples
#     need no marker and cannot drift out of one.
dir=$(make_fixture)
printf 'add_executable(example_basic_usage examples/basic.cpp)\n' >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir")
expect_not_contains "example_* targets are exempt by name" "$out" "ARCH-7"
rm -rf "$dir"

# 19. The report names the offending target, not just the file.
dir=$(make_fixture)
printf 'add_executable(rt_control_node src/existing.cpp)\n' >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir")
expect_contains "ARCH-7 names the new target" "$out" "rt_control_node"
rm -rf "$dir"

# 20. ARCH-5 must not punish writing the rule down next to the call. The
#     multi-line join matched `[^)]*`, so a comment restating the invariant
#     was itself reported as a violation.
dir=$(make_fixture)
cat >>"$dir/rtc_demo/CMakeLists.txt" <<'CMAKE'
ament_target_dependencies(rtc_demo
  rclcpp
  # NOTE: robot_descriptions stays exec_depend only, never linked here
)
CMAKE
out=$(run_hook "$dir")
expect_not_contains "ARCH-5 ignores a comment restating the rule" "$out" "ARCH-5"
rm -rf "$dir"

# --- YAML gate ---------------------------------------------------------------

# 21. A multi-document file is legal YAML; safe_load alone rejected it.
#     Vacuous when the gate is skipping (no PyYAML), so it needs PyYAML present.
if have_pyyaml; then
  dir=$(make_fixture)
  printf 'a: 1\n---\nb: 2\n' >"$dir/rtc_demo/config/multi.yaml"
  out=$(run_hook "$dir")
  expect_not_contains "multi-document YAML is accepted" "$out" "YAML parse failures"
  rm -rf "$dir"
else
  skip "multi-document YAML is accepted (PyYAML not importable)"
fi

# 22. Interpreter noise on stderr is not a parse failure. The gate used to key
#     on "stderr is non-empty", so any startup warning blocked the turn; it now
#     keys on the interpreter EXIT STATUS. A valid YAML under PYTHONDEVMODE emits
#     nothing, so that fixture pinned nothing (the old buggy gate passed it too).
#     Inject *real* stderr into every python3 the hook spawns via a
#     usercustomize.py on PYTHONPATH, while the YAML stays valid: the old
#     stderr-keyed gate reports a parse failure here, the exit-status gate does not.
#     Needs PyYAML present, or the gate skips before the exit-status path runs.
if have_pyyaml; then
  dir=$(make_fixture)
  noise=$(mktemp -d)
  printf 'import sys; sys.stderr.write("simulated startup ResourceWarning\\n")\n' >"$noise/usercustomize.py"
  printf 'a: 1\n' >"$dir/rtc_demo/config/ok.yaml"
  out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_SKIP_BUILD=1 PYTHONPATH="$noise" \
          bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null ); rc=$?
  expect_not_contains "an interpreter warning is not a parse failure" "$out" "YAML parse failures"
  expect_exit "interpreter stderr noise does not block a valid YAML" "$rc" 0
  rm -rf "$dir" "$noise"
else
  skip "an interpreter warning is not a parse failure (PyYAML not importable)"
fi

# 23. Missing PyYAML must fail OPEN, like clang-format and shellcheck. Failing
#     closed wedged every turn touching a YAML with no in-band recovery.
dir=$(make_fixture)
stub=$(mktemp -d)
printf 'raise ImportError("stub")\n' >"$stub/yaml.py"
printf 'a: 1\n' >"$dir/rtc_demo/config/ok.yaml"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_SKIP_BUILD=1 PYTHONPATH="$stub" \
        bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null )
expect_contains "a missing PyYAML fails open" "$out" "YAML parse gate skipped"
rm -rf "$dir" "$stub"

# --- Docs gate scope ---------------------------------------------------------

# 24. A pre-existing defect in a doc the change merely touched must not block.
#     Whole-file scope meant the agent had to repair damage it did not cause
#     before it could end the turn.
dir=$(make_fixture)
printf '# docs\n\nSee [missing](./nope.md).\n' >"$dir/agent_docs/notes.md"
git -C "$dir" commit -qam "pre-existing broken link"
printf '\nAn unrelated new sentence.\n' >>"$dir/agent_docs/notes.md"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "a pre-existing doc defect on an untouched line does not block" "$out" "nope.md"
expect_exit "a pre-existing untouched-line doc defect does not block the turn" "$rc" 0
rm -rf "$dir"

# 25. ...but a link the change itself adds still does.
dir=$(make_fixture)
printf '# docs\n\nSee [missing](./nope.md).\n' >"$dir/agent_docs/notes.md"
git -C "$dir" commit -qam "pre-existing broken link"
printf '\nSee [also-missing](./gone.md).\n' >>"$dir/agent_docs/notes.md"
out=$(run_hook "$dir"); rc=$?
expect_contains "a newly added broken link still blocks" "$out" "gone.md"
expect_exit "a newly added broken link blocks the turn" "$rc" 2
rm -rf "$dir"

# 26. A deleted doc must not crash the validator or block on its absence. The
#     hook drops it via `[ -f ]` before the validator runs, so that half only
#     proves the hook exits clean. The validator's own skip-a-missing-target
#     path (which the hook relies on) is asserted directly below -- otherwise
#     removing that guard would regress silently, the fixture staying green.
dir=$(make_fixture)
git -C "$dir" rm -q "agent_docs/notes.md"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "a deleted doc is not an error" "$out" "Traceback"
expect_exit "a deleted doc does not block the turn" "$rc" 0
vrc=0
vout=$( cd "$dir" && python3 "$REPO_ROOT/repo_scripts/scripts/validate_docs.py" \
          --files agent_docs/notes.md 2>&1 ) || vrc=$?
expect_exit "validate_docs skips a missing --files target instead of crashing" "$vrc" 0
expect_not_contains "validate_docs does not traceback on a missing target" "$vout" "Traceback"
rm -rf "$dir"

# --- Filenames ---------------------------------------------------------------

# 27. A path containing a space must survive as one path. Unquoted word
#     splitting turned it into fragments that reached the co-update gate as
#     invented filenames no edit could ever satisfy.
dir=$(make_fixture)
echo 'int spaced() { return 3; }' >"$dir/rtc_demo/src/my file.cpp"
out=$(run_hook "$dir")
expect_contains "a spaced filename is reported intact" "$out" "my file.cpp"
rm -rf "$dir"

# --- Fail-closed / routing regressions -------------------------------------

# 28. A formatter that emits no output (uvx cold/offline cache, transient error)
#     must NOT let a change be graded pure-format. A stub clang-format that exits
#     non-zero with empty stdout is put first on PATH; the old code diffed two
#     empty process substitutions, got exit 0, and skipped Phase 0. Empty output
#     is "unverifiable", not "clean" -- so a robot-name reformat must still fire
#     ARCH-1 (fail closed).
dir=$(make_fixture)
stub=$(mktemp -d)
printf '#!/bin/sh\nexit 1\n' >"$stub/clang-format"
chmod +x "$stub/clang-format"
printf 'int ur5e_thing()  {  return 0;  }\n' >"$dir/rtc_demo/src/existing.cpp"
git -C "$dir" commit -qam "robot mention at HEAD"
printf 'int ur5e_thing() {   return 0;   }\n' >"$dir/rtc_demo/src/existing.cpp"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_SKIP_BUILD=1 PATH="$stub:$PATH" \
        bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null )
expect_contains "a failing formatter is not treated as pure-format (fails closed)" "$out" "ARCH-1"
rm -rf "$dir" "$stub"

# 29. An untracked module in the ament_python layout (<pkg>/<pkg>/) must be
#     BUILT. The src|include-only routing missed it, so a brand-new Python
#     module was ARCH-grepped and then never compiled or tested.
dir=$(make_fixture)
mkdir -p "$dir/rtc_demo/rtc_demo"
printf 'def foo():\n    return 1\n' >"$dir/rtc_demo/rtc_demo/foo.py"
out=$(run_hook "$dir")
expect_contains "untracked ament_python module is routed to a build" "$out" "BUILD_PKGS=[rtc_demo]"
rm -rf "$dir"

# 30. The new-file CMakeLists check matches whole filenames, not substrings: an
#     unlisted new .cpp whose basename is a suffix of a listed one (isting.cpp
#     inside existing.cpp) must still be flagged, or the omission slips through.
dir=$(make_fixture)
printf 'int isting() { return 7; }\n' >"$dir/rtc_demo/src/isting.cpp"
out=$(run_hook "$dir")
expect_contains "an unlisted new .cpp is flagged even when its name is a substring of a listed one" "$out" "isting.cpp not found"
rm -rf "$dir"

# 31. Constitution parity: CLAUDE.md and AGENTS.md carry the same rules for two
#     audiences, and nothing else in the harness reads AGENTS.md -- it went 4
#     commits stale that way. Editing one must remind about the other. Both
#     directions, and silent when both move together (or neither does).
dir=$(make_fixture)
printf '# c\n' >"$dir/CLAUDE.md"
printf '# a\n' >"$dir/AGENTS.md"
git -C "$dir" add -A && git -C "$dir" commit -qm parity
printf '# c\n\n새 규칙.\n' >"$dir/CLAUDE.md"
out=$(run_hook "$dir")
expect_contains "CLAUDE.md alone reminds about AGENTS.md" "$out" "CLAUDE.md changed without AGENTS.md"
printf '# a\n\n같은 규칙.\n' >"$dir/AGENTS.md"
out=$(run_hook "$dir")
expect_not_contains "editing both is silent" "$out" "changed without"
rm -rf "$dir"

# 32. ...and the reverse direction, so the reminder is not one-way.
dir=$(make_fixture)
printf '# c\n' >"$dir/CLAUDE.md"
printf '# a\n' >"$dir/AGENTS.md"
git -C "$dir" add -A && git -C "$dir" commit -qm parity
printf '# a\n\n새 규칙.\n' >"$dir/AGENTS.md"
out=$(run_hook "$dir")
expect_contains "AGENTS.md alone reminds about CLAUDE.md" "$out" "AGENTS.md changed without CLAUDE.md"
rm -rf "$dir"

# 33. ARCH-5 allows <test_depend>robot_descriptions: a test that resolves the
#     share dir through ament at runtime needs the dep to order installation,
#     and that runtime lookup is the pattern ARCH-5 asks for in the first place.
#     Pinned as a test so the exception is not just prose -- widening the
#     alternation to catch test_depend would break rtc_controller_manager.
dir=$(make_fixture)
sed -i 's|</package>|  <test_depend>robot_descriptions</test_depend>\n</package>|' \
  "$dir/rtc_demo/package.xml"
out=$(run_hook "$dir")
expect_not_contains "ARCH-5 permits <test_depend>robot_descriptions" "$out" "ARCH-5"
rm -rf "$dir"

# 34. ...while <depend> in the same position still trips it, so 33 is not green
#     because the ARCH-5 package.xml check went missing altogether.
dir=$(make_fixture)
sed -i 's|</package>|  <depend>robot_descriptions</depend>\n</package>|' \
  "$dir/rtc_demo/package.xml"
out=$(run_hook "$dir")
expect_contains "ARCH-5 still catches <depend>robot_descriptions" "$out" "ARCH-5"
rm -rf "$dir"

# --- Phase 2 build classification (#435) --------------------------------------
#
# One machine, one commit, one package: 179.4s (killed at the bound) while a
# long build competed for CPU, 2.5s idle. Both used to print the byte-identical
# "<pkg>: build failed", so the agent went debugging a change that was fine.
# Every other branch in Phase 2 already classified by exit code; these did not.

# 35. A build killed at the bound (124) must say so, must NOT read as a broken
#     build, and must still block.
dir=$(make_fixture)
stub=$(make_build_stub 124)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_build "$dir" "$stub"); rc=$?
expect_contains "a bound-killed build is reported as a timeout" "$out" "build TIMED OUT after 180s"
expect_not_contains "a bound-killed build is not reported as broken code" "$out" "build FAILED"
expect_exit "a bound-killed build still blocks the turn" "$rc" 2
# 36. ...and carries the evidence that separates contention from a slow build.
expect_contains "a build timeout reports loadavg" "$out" "loadavg"
expect_contains "a build timeout reports the concurrent build count" "$out" "build/compiler processes"
rm -rf "$dir" "$stub"

# 37. A genuinely failing build must be distinguishable from 35 -- different
#     string, the exit code, and the build output that says WHICH file broke.
#     The output was discarded (>/dev/null 2>&1), so even a correctly classified
#     failure told the agent nothing.
dir=$(make_fixture)
stub=$(make_build_stub 1)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_build "$dir" "$stub"); rc=$?
expect_contains "a real build failure reports its exit code" "$out" "build FAILED (exit 1)"
expect_not_contains "a real build failure is not reported as a timeout" "$out" "TIMED OUT"
expect_contains "a real build failure carries the build output tail" "$out" "error: fixture_file.cpp:7:3"
expect_contains "the tail shows how the build was invoked" "$out" "stub-build args: -p rtc_demo"
expect_exit "a real build failure blocks the turn" "$rc" 2
rm -rf "$dir" "$stub"

# 38. The PROC-3 path (rtc_base / rtc_msgs -> ./build.sh full) had the same
#     defect. Fixing only the per-package branch leaves it on the highest-impact
#     path, so both are pinned.
dir=$(make_fixture)
add_rtc_base "$dir"
stub=$(make_build_stub 124)
echo 'int base_fn() { return 1; }' >"$dir/rtc_base/src/base.cpp"
out=$(run_hook_build "$dir" "$stub"); rc=$?
expect_contains "a bound-killed PROC-3 build is reported as a timeout" "$out" "PROC-3 broad build"
expect_contains "the PROC-3 timeout names its own bound" "$out" "TIMED OUT after 300s"
expect_contains "the PROC-3 timeout reports loadavg" "$out" "loadavg"
expect_exit "a bound-killed PROC-3 build still blocks the turn" "$rc" 2
rm -rf "$dir" "$stub"

# 39. ...and a real PROC-3 build failure gets the same treatment as 37.
dir=$(make_fixture)
add_rtc_base "$dir"
stub=$(make_build_stub 1)
echo 'int base_fn() { return 1; }' >"$dir/rtc_base/src/base.cpp"
out=$(run_hook_build "$dir" "$stub"); rc=$?
expect_contains "a real PROC-3 build failure reports its exit code" "$out" "PROC-3 broad build (build.sh full) FAILED (exit 1,"
expect_not_contains "a real PROC-3 build failure is not reported as a timeout" "$out" "TIMED OUT"
expect_contains "a real PROC-3 build failure carries the build output tail" "$out" "error: fixture_file.cpp:7:3"
expect_contains "the PROC-3 tail shows how the build was invoked" "$out" "stub-build args: full"
expect_exit "a real PROC-3 build failure blocks the turn" "$rc" 2
rm -rf "$dir" "$stub"

printf '\n%d passed, %d failed, %d skipped\n' "$PASS" "$FAIL" "$SKIP"
[ "$FAIL" -eq 0 ]
