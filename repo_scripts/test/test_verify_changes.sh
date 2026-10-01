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
# was run against the pre-fix hook to confirm it goes red, and the ones
# asserting an absence were checked by severing the mechanism they depend on.
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
# routes its build through the stub. Called as `--run`: since 2026-10-01 that
# is the only call that builds -- the turn-end call checks the verdict and
# builds nothing (cases 62-65), so every case that is about what the build or
# the tests answered goes through here.
run_hook_build() {
  local dir="$1" stub="$2"
  ( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_BUILD_CMD="$stub/build.sh" \
      bash "$HOOK" --run </dev/null 2>&1 >/dev/null )
}

# The turn-end twin: the Stop call with the same build seam in place. It never
# builds, so "stub-build args" in its output would mean that it did. The cases
# about what the TURN END does beside a simulator go through here.
run_stop_build() {
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

# --- test isolation gates (Phase 1d) -----------------------------------------

# The gate scripts judge the tree they live in, so each case copies them into
# the fixture (the hook resolves them in PROJECT_DIR) and gives it a second
# package. $1 and $2 are the two packages' ROS_DOMAIN_ID claims.
make_testgate_fixture() {
  local dir
  dir=$(make_fixture)
  mkdir -p "$dir/repo_scripts/scripts" "$dir/rtc_other"
  cp "$REPO_ROOT/repo_scripts/scripts/validate_test_domains.py" \
    "$REPO_ROOT/repo_scripts/scripts/validate_test_fixtures.py" "$dir/repo_scripts/scripts/"
  sed 's/rtc_demo/rtc_other/' "$dir/rtc_demo/package.xml" >"$dir/rtc_other/package.xml"
  printf 'project(rtc_other)\nament_add_gtest(t_other test/t.cpp\n  ENV ROS_DOMAIN_ID=%s)\n' \
    "$2" >"$dir/rtc_other/CMakeLists.txt"
  git -C "$dir" add -A && git -C "$dir" commit -qm "second package and the gates"
  printf 'ament_add_gtest(t_demo test/t.cpp\n  ENV ROS_DOMAIN_ID=%s)\n' "$1" \
    >>"$dir/rtc_demo/CMakeLists.txt"
  echo "$dir"
}

# 15i. A claim that collides with another package's must BLOCK at turn end, not
#      first on the PR. #513 and #571 both passed every local check and failed
#      CI: the gate ran only in CI, and Phase 2 runs repo_scripts' tests only
#      when repo_scripts changed.
dir=$(make_testgate_fixture 60 60)
out=$(run_hook "$dir"); rc=$?
expect_contains "a colliding ROS_DOMAIN_ID claim is reported" "$out" "ROS_DOMAIN_ID=60 is claimed by 2 packages"
expect_exit "a colliding ROS_DOMAIN_ID claim blocks the turn" "$rc" 2
rm -rf "$dir"

# 15j. ...while distinct claims are silent. Without this the gate could be a
#      constant blocker and 15i would still pass.
dir=$(make_testgate_fixture 60 61)
out=$(run_hook "$dir"); rc=$?
expect_not_contains "distinct ROS_DOMAIN_ID claims are not reported" "$out" "Test isolation gates"
expect_exit "distinct ROS_DOMAIN_ID claims do not block" "$rc" 0
rm -rf "$dir"

# 15k. The trigger is a relevance filter: a turn that touches no build file,
#      conftest or test source does not run the gates. The collision is
#      committed here so that only the trigger stands between it and a block.
dir=$(make_testgate_fixture 60 60)
git -C "$dir" commit -qam "collision at HEAD"
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "a src-only turn does not run the test gates" "$out" "Test isolation gates"
expect_exit "a src-only turn is not blocked by the test gates" "$rc" 0
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

# 17b. ...and where a multi-line justification puts it: at the TOP of the
#      block, with prose between it and the call. One-line-above matching
#      rejected this, and the rejection reads as ARCH-7 misfiring rather than
#      as a format nit, because the exemption *was* declared (observed
#      2026-09-10 on rtc_inference_check).
dir=$(make_fixture)
printf '# ARCH-7-exempt\n# offline inspector: takes a path on argv, owns no RT loop,\n# and appears in no bringup chain.\nadd_executable(inspector_tool src/existing.cpp)\n' \
  >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "ARCH-7 marker is honoured at the top of a comment block" "$out" "ARCH-7"
expect_exit "a block-marked target does not block the turn" "$rc" 0
rm -rf "$dir"

# 17c. The block is bounded. A blank line ends it, so a marker that is not
#      attached to THIS call -- a file header, or the justification for the
#      call above -- must not exempt it. Without this the widening in 17b would
#      have no upper edge and every target under a marker-bearing header would
#      go quiet.
dir=$(make_fixture)
printf '# ARCH-7-exempt: belongs to nothing in particular\n\nadd_executable(detached_marker_node src/existing.cpp)\n' \
  >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir"); rc=$?
expect_contains "a marker separated by a blank line does not exempt" "$out" "detached_marker_node"
expect_exit "a detached marker still blocks" "$rc" 2
rm -rf "$dir"

# 17d. ...and it does not leak from one call to the next. The intervening
#      add_executable is not a comment, so it ends the block.
dir=$(make_fixture)
printf '# ARCH-7-exempt: only this one\nadd_executable(marked_tool src/existing.cpp)\nadd_executable(unmarked_node src/existing.cpp)\n' \
  >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir")
expect_contains "the marker does not carry to the following target" "$out" "unmarked_node"
expect_not_contains "the marked target stays exempt" "$out" "marked_tool"
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

# 31. Constitution split: AGENTS.md is the constitution and CLAUDE.md imports it
#     (`@AGENTS.md`). Dropping the import line loses the whole constitution for
#     Claude Code with no other symptom, so it blocks; a CLAUDE.md edit that
#     keeps it only reminds that rules belong in AGENTS.md. The old parity
#     reminder ("mirror into AGENTS.md") would recreate the duplicated copies.
dir=$(make_fixture)
printf '@AGENTS.md\n\n# c\n' >"$dir/CLAUDE.md"
printf '# a\n' >"$dir/AGENTS.md"
git -C "$dir" add -A && git -C "$dir" commit -qm split
printf '@AGENTS.md\n\n# c\n\n- hook 배선.\n' >"$dir/CLAUDE.md"
out=$(run_hook "$dir"); rc=$?
expect_contains "a CLAUDE.md edit reminds that rules belong in AGENTS.md" "$out" "CLAUDE.md changed: it holds only Claude Code mechanisms"
expect_not_contains "a CLAUDE.md edit no longer asks to mirror it into AGENTS.md" "$out" "mirror it into AGENTS.md"
expect_exit "a CLAUDE.md edit that keeps the import does not block" "$rc" 0
printf '# c\n\n- hook 배선.\n' >"$dir/CLAUDE.md"
out=$(run_hook "$dir"); rc=$?
expect_contains "dropping @AGENTS.md from CLAUDE.md is reported" "$out" "Constitution import missing"
expect_exit "dropping @AGENTS.md from CLAUDE.md blocks the turn" "$rc" 2
rm -rf "$dir"

# 32. ...and an AGENTS.md-only edit is silent: CLAUDE.md imports it, so there is
#     no second copy to keep in step.
dir=$(make_fixture)
printf '@AGENTS.md\n\n# c\n' >"$dir/CLAUDE.md"
printf '# a\n' >"$dir/AGENTS.md"
git -C "$dir" add -A && git -C "$dir" commit -qm split
printf '# a\n\n새 규칙.\n' >"$dir/AGENTS.md"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "an AGENTS.md-only edit asks for no CLAUDE.md follow-up" "$out" "AGENTS.md changed"
expect_exit "an AGENTS.md-only edit does not block" "$rc" 0
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
expect_contains "a bound-killed build is reported as a timeout" "$out" "build TIMED OUT after 900s"
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
# --tests: build.sh builds no tests by default, and a package built without them
# tests as "0 tests, 0 failures" -- the hook would call that a pass.
expect_contains "the tail shows how the build was invoked" "$out" "stub-build args: -p rtc_demo --tests"
expect_exit "a real build failure blocks the turn" "$rc" 2
rm -rf "$dir" "$stub"

# 37b. The bound is an environment override, and the message names the bound
#      that was in force -- not a number written into the message. (This slot
#      held the Stop-budget deadline, past which a test was not started; that
#      mechanism went with the turn-end build on 2026-10-01.)
dir=$(make_fixture)
stub=$(make_build_stub 124)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_BUILD_CMD="$stub/build.sh" \
    RTC_VERIFY_BUILD_BOUND_S=42 bash "$HOOK" --run </dev/null 2>&1 >/dev/null ); rc=$?
expect_contains "the build bound follows RTC_VERIFY_BUILD_BOUND_S" "$out" "build TIMED OUT after 42s"
expect_contains "...and the message names the variable to raise" "$out" "RTC_VERIFY_BUILD_BOUND_S"
expect_exit "a bound-killed build under an overridden bound still blocks" "$rc" 2
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
expect_contains "the PROC-3 timeout names its own bound" "$out" "TIMED OUT after 2400s"
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
expect_contains "the PROC-3 tail shows how the build was invoked" "$out" "stub-build args: full --tests"
expect_exit "a real PROC-3 build failure blocks the turn" "$rc" 2
rm -rf "$dir" "$stub"

# --- A build already running in the workspace ---------------------------------
#
# 2026-09-14: the agent's own `./build.sh` ran as a background shell task, the
# hook built the same packages beside it and was killed at the bound. The hook
# now looks for a colcon / build.sh whose cwd is its colcon workspace before
# building, and blocks without building when it finds one.

# Nested as <ws>/src/repo so the hook's WORKSPACE ($PROJECT_DIR/../..) is a
# directory this test owns rather than "/". Echoes the repo dir.
make_nested_fixture() {
  local ws dir
  ws=$(mktemp -d)
  mkdir -p "$ws/src"
  dir=$(make_fixture)
  mv "$dir" "$ws/src/repo"
  echo "$ws/src/repo"
}

# A stand-in for a running build: a script whose process NAME is its file name.
# Direct shebang on purpose -- through `#!/usr/bin/env bash` env execs bash and
# the process is named "bash", which no name match can see (measured).
write_rival() {
  cat >"$1" <<'EOF'
#!/bin/bash
trap 'kill "$child" 2>/dev/null; exit 0' TERM
sleep 60 &
child=$!
wait "$child"
EOF
  chmod +x "$1"
}

# $1 = pid, $2 = process name. The background subshell execs the rival, so its
# name changes a moment after `&` returns; asserting before that would test
# nothing.
wait_for_name() {
  local _
  for _ in $(seq 1 50); do
    [ "$(cat "/proc/$1/comm" 2>/dev/null)" = "$2" ] && return 0
    sleep 0.1
  done
  return 1
}

# 39b. A rival in the workspace blocks the turn WITHOUT building beside it: the
#      stub build would print "stub-build args" into the report if it ran.
for rival in colcon build.sh; do
  dir=$(make_nested_fixture)
  ws=$(cd "$dir/../.." && pwd -P)
  stub=$(make_build_stub 1)
  bin=$(mktemp -d)
  write_rival "$bin/$rival"
  (cd "$ws" && exec "$bin/$rival") >/dev/null 2>&1 &
  rpid=$!
  if wait_for_name "$rpid" "$rival"; then
    echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
    out=$(run_hook_build "$dir" "$stub"); rc=$?
    expect_contains "a $rival running in the workspace is reported" "$out" "build/test NOT run"
    # cmdline of a shebang script is "<interpreter> <script>".
    expect_contains "the report names the running $rival by pid" "$out" "$rpid: /bin/bash $bin/$rival"
    expect_not_contains "nothing is built beside a running $rival" "$out" "stub-build args"
    expect_exit "a $rival running in the workspace blocks the turn" "$rc" 2
  else
    fail "the $rival stand-in never showed up under its own name"
  fi
  kill "$rpid" 2>/dev/null
  wait "$rpid" 2>/dev/null
  rm -rf "$ws" "$stub" "$bin"
done

# 39c. ...but a colcon running for ANOTHER workspace is not a rival: the build
#      runs as before. Without this the check could block on any colcon at all.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
stub=$(make_build_stub 1)
bin=$(mktemp -d)
elsewhere=$(mktemp -d)
write_rival "$bin/colcon"
(cd "$elsewhere" && exec "$bin/colcon") >/dev/null 2>&1 &
rpid=$!
if wait_for_name "$rpid" colcon; then
  echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
  out=$(run_hook_build "$dir" "$stub"); rc=$?
  expect_not_contains "a colcon in another workspace is not a rival" "$out" "build/test NOT run"
  expect_contains "the build still runs beside a colcon elsewhere" "$out" "build FAILED (exit 1)"
  expect_exit "that build failure still blocks the turn" "$rc" 2
else
  fail "the colcon stand-in never showed up under its own name"
fi
kill "$rpid" 2>/dev/null
wait "$rpid" 2>/dev/null
rm -rf "$ws" "$stub" "$bin" "$elsewhere"

# --- A simulator running from the workspace ------------------------------------
#
# 2026-09-26: the S8-E success-rate sims ran as a background shell task; a hook
# build beside them slows the sim below real time and the catches fail for the
# rig's sake. A sim whose executable or command line lies under the workspace
# DEFERS build/test (a sim, unlike a build, does not end on its own); one from
# another workspace does not.

# $1 = directory to hold the stand-in. The kernel truncates the process name to
# 15 characters, so the stand-in named like the real node shows up as
# "mujoco_simulato" -- the name the hook matches.
start_sim_standin() {
  mkdir -p "$1"
  write_rival "$1/mujoco_simulator_node"
  ("$1/mujoco_simulator_node") >/dev/null 2>&1 &
  echo $!
}
stop_standin() {
  kill "$1" 2>/dev/null
  wait "$1" 2>/dev/null
}

# 39d. A sim started from this workspace's install tree: no build, no block,
#      watermark kept -- and the first stop after it ends grades the change.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
stub=$(make_build_stub 1)
base=$(git -C "$dir" rev-parse HEAD)
echo "$base" >"$dir/.git/rtc-verify-base"
spid=$(start_sim_standin "$ws/install/rtc_mujoco_sim/lib/rtc_mujoco_sim")
if wait_for_name "$spid" mujoco_simulato; then
  # Committed in-turn, as in 51: an uncommitted edit would be graded against
  # HEAD anyway, so only a commit shows whether the watermark was kept.
  echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
  git -C "$dir" commit -qam "edit committed in-turn while the sim runs"
  out=$(run_stop_build "$dir" "$stub"); rc=$?
  expect_contains "a sim from the workspace defers build/test" "$out" "Build/test deferred"
  expect_contains "the deferral names the sim by pid" "$out" "$spid: /bin/bash $ws/install/rtc_mujoco_sim"
  expect_not_contains "nothing is built beside a running sim" "$out" "stub-build args"
  expect_exit "a sim from the workspace does not block the turn" "$rc" 0
  if [ "$(cat "$dir/.git/rtc-verify-base")" = "$base" ]; then
    pass "a sim deferral keeps the watermark"
  else
    fail "a sim deferral advanced the watermark"
  fi
  stop_standin "$spid"
  # The deferral dropped nothing: the turn end now asks for the verdict, and
  # --run builds the change (the stub build fails, so it must report that).
  out=$(run_stop_build "$dir" "$stub"); rc=$?
  expect_contains "the change deferred for a sim is owed once it ends" "$out" "build/test verdict missing for: rtc_demo"
  expect_exit "the turn end blocks on it once the sim ends" "$rc" 2
  out=$(run_hook_build "$dir" "$stub"); rc=$?
  expect_contains "the change deferred for a sim is built once it ends" "$out" "build FAILED (exit 1)"
  expect_exit "that deferred build failure blocks once the sim ends" "$rc" 2
else
  fail "the sim stand-in never showed up as mujoco_simulato"
  stop_standin "$spid"
fi
rm -rf "$ws" "$stub"

# 39e. Deferring build/test does not switch the other gates off: a blocking
#      defect elsewhere still blocks, and says build/test was deferred.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
stub=$(make_build_stub 0)
spid=$(start_sim_standin "$ws/install/rtc_mujoco_sim/lib/rtc_mujoco_sim")
if wait_for_name "$spid" mujoco_simulato; then
  # add_missing_dep is defined further down; the same edit, inline.
  sed -i 's/^project(rtc_demo)/project(rtc_demo)\nfind_package(fmt REQUIRED)/' "$dir/rtc_demo/CMakeLists.txt"
  out=$(run_stop_build "$dir" "$stub"); rc=$?
  expect_contains "another gate still reports beside a running sim" "$out" "find_package(fmt)"
  expect_contains "the block says build/test was deferred" "$out" "Build/test deferred"
  expect_exit "another gate still blocks beside a running sim" "$rc" 2
else
  fail "the sim stand-in never showed up as mujoco_simulato"
fi
stop_standin "$spid"
rm -rf "$ws" "$stub"

# 39f. A workspace reached through a symlink: colcon's setup scripts put the
#      LOGICAL path in AMENT_PREFIX_PATH, so the sim's argv carries it while
#      `pwd -P` gives the physical one. Both must match.
real=$(mktemp -d)
mkdir -p "$real/src"
mv "$(make_fixture)" "$real/src/repo"
link="$(mktemp -d)/ws"
ln -s "$real" "$link"
stub=$(make_build_stub 1)
spid=$(start_sim_standin "$link/install/rtc_mujoco_sim/lib/rtc_mujoco_sim")
if wait_for_name "$spid" mujoco_simulato; then
  echo 'int existing() { return 1; }' >"$link/src/repo/rtc_demo/src/existing.cpp"
  out=$(run_stop_build "$link/src/repo" "$stub"); rc=$?
  expect_contains "a sim started through the workspace symlink defers" "$out" "Build/test deferred"
  expect_not_contains "nothing is built beside a sim on the symlinked path" "$out" "stub-build args"
else
  fail "the symlinked sim stand-in never showed up as mujoco_simulato"
fi
stop_standin "$spid"
rm -rf "$real" "$(dirname "$link")" "$stub"

# 39g. ...a sim from another workspace is not ours: the build runs as before.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
stub=$(make_build_stub 1)
elsewhere=$(mktemp -d)
spid=$(start_sim_standin "$elsewhere/install/rtc_mujoco_sim/lib/rtc_mujoco_sim")
if wait_for_name "$spid" mujoco_simulato; then
  echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
  out=$(run_stop_build "$dir" "$stub"); rc=$?
  expect_not_contains "a sim from another workspace defers nothing" "$out" "Build/test deferred"
  # ...and the absence is not the turn end having had nothing to ask for.
  expect_contains "the turn end asks for the verdict beside a sim elsewhere" "$out" "build/test verdict missing for: rtc_demo"
  out=$(run_hook_build "$dir" "$stub"); rc=$?
  expect_contains "the build still runs beside a sim elsewhere" "$out" "build FAILED (exit 1)"
else
  fail "the other-workspace sim stand-in never showed up as mujoco_simulato"
fi
stop_standin "$spid"
rm -rf "$ws" "$stub" "$elsewhere"

# --- Phase 5 formatter drift ---------------------------------------------------
#
# format-code.sh only sees Edit / Write, so a file written through Bash reached
# commits unformatted and nothing downstream graded it. Phase 5 blocks drift the
# change introduced and must stay quiet on debt it did not.

# The hook resolves ruff venv-first, then <rtc_ws>/.venv relative to itself, then
# PATH; mirror that so the Python cases skip instead of failing on a box without it.
have_ruff() {
  [ -n "${VIRTUAL_ENV:-}" ] && [ -x "${VIRTUAL_ENV}/bin/ruff" ] && return 0
  [ -x "$REPO_ROOT/../../.venv/bin/ruff" ] && return 0
  command -v ruff >/dev/null 2>&1
}

if have_ruff; then
  # 40. A new installed-source .py that is not a ruff fixed point blocks, and
  #     the report names the command that fixes it.
  dir=$(make_fixture)
  mkdir -p "$dir/rtc_demo/rtc_demo"
  printf "x = {  'a':1 }\n" >"$dir/rtc_demo/rtc_demo/written_by_bash.py"
  out=$(run_hook "$dir"); rc=$?
  expect_contains "a new unformatted .py is reported as formatter drift" "$out" "Formatter drift introduced"
  expect_contains "the drift report names the fix" "$out" "ruff format rtc_demo/rtc_demo/written_by_bash.py"
  expect_exit "a new unformatted .py blocks the turn" "$rc" 2
  rm -rf "$dir"

  # 41. A formatted file made unformatted by the change blocks too -- including
  #     in-turn commits, which the watermark base still sees.
  dir=$(make_fixture)
  mkdir -p "$dir/rtc_demo/rtc_demo"
  printf 'x = {"a": 1}\n' >"$dir/rtc_demo/rtc_demo/mod.py"
  git -C "$dir" add -A && git -C "$dir" commit -qm formatted
  git -C "$dir" rev-parse HEAD >"$dir/.git/rtc-verify-base"
  printf "x = {  'a':2 }\n" >"$dir/rtc_demo/rtc_demo/mod.py"
  git -C "$dir" commit -qam "unformatted, committed in-turn"
  out=$(run_hook "$dir"); rc=$?
  expect_contains "drift committed in-turn on a formatted file is reported" "$out" "ruff format rtc_demo/rtc_demo/mod.py"
  expect_exit "drift committed in-turn on a formatted file blocks" "$rc" 2
  rm -rf "$dir"

  # 42. A file already unformatted at the base is debt this diff did not cause:
  #     touching it must not block. Asserting the absence alone would also hold
  #     if Phase 5 never ran, so 41 above is the half that can go red.
  dir=$(make_fixture)
  mkdir -p "$dir/rtc_demo/rtc_demo"
  printf "x = {  'a':1 }\n" >"$dir/rtc_demo/rtc_demo/legacy.py"
  git -C "$dir" add -A && git -C "$dir" commit -qm legacy
  printf "x = {  'a':1 }\ny = {  'b':2 }\n" >"$dir/rtc_demo/rtc_demo/legacy.py"
  out=$(run_hook "$dir"); rc=$?
  expect_not_contains "touching a file unformatted at the base is not drift" "$out" "Formatter drift"
  expect_exit "touching a file unformatted at the base does not block" "$rc" 0
  rm -rf "$dir"

  # 43. Untracked scratch outside the installed-source dirs is not graded, the
  #     same scope build/test uses.
  dir=$(make_fixture)
  printf "x = {  'a':1 }\n" >"$dir/scratch_probe.py"
  out=$(run_hook "$dir")
  expect_not_contains "untracked scratch outside package source dirs is not graded" "$out" "Formatter drift"
  rm -rf "$dir"
else
  skip "Phase 5 .py cases (40-43): no ruff the hook can resolve"
fi

# 44. C++ takes the clang-format branch: an edit that breaks formatting of a
#     formatted file blocks and names clang-format.
if have_formatter; then
  dir=$(make_fixture)
  printf 'int existing( ){return   1;}\n' >"$dir/rtc_demo/src/existing.cpp"
  out=$(run_hook "$dir"); rc=$?
  expect_contains "C++ drift names clang-format" "$out" "clang-format -i rtc_demo/src/existing.cpp"
  expect_exit "C++ drift blocks the turn" "$rc" 2
  printf 'int existing() { return 1; }\n' >"$dir/rtc_demo/src/existing.cpp"
  out=$(run_hook "$dir")
  expect_not_contains "a formatted C++ edit is not drift" "$out" "Formatter drift"
  rm -rf "$dir"
else
  skip "Phase 5 C++ case (44): no clang-format the hook can resolve"
fi

if have_ruff; then
  # 45. Command substitution strips trailing newlines, so comparing "$(fmt)"
  #     with "$(cat f)" read a missing final newline and trailing blank lines as
  #     clean. Both are drift ruff format would rewrite.
  dir=$(make_fixture)
  mkdir -p "$dir/rtc_demo/rtc_demo"
  printf 'x = 1' >"$dir/rtc_demo/rtc_demo/no_final_newline.py"
  printf 'y = 1\n\n\n' >"$dir/rtc_demo/rtc_demo/trailing_blank_lines.py"
  out=$(run_hook "$dir"); rc=$?
  expect_contains "a missing final newline is formatter drift" "$out" "no_final_newline.py: formatter would rewrite"
  expect_contains "trailing blank lines are formatter drift" "$out" "trailing_blank_lines.py: formatter would rewrite"
  expect_exit "newline drift blocks the turn" "$rc" 2

  # 46. Past the Stop-budget deadline Phase 5 stops grading and says which files
  #     it skipped, instead of running into the SIGKILL. 45 above is the half
  #     that proves the same input is drift when graded.
  out=$(export RTC_VERIFY_FORMAT_DEADLINE_S=0; run_hook "$dir"); rc=$?
  expect_contains "files past the deadline are listed as ungraded" "$out" "formatter drift NOT graded"
  expect_exit "ungraded files do not block the turn" "$rc" 0
  rm -rf "$dir"

  # 47. format-code.sh's own output must pass Phase 5. With `ruff format` run
  #     before `ruff check --fix`, UP015 dropped "r" from an over-long open()
  #     that format had already split, and the one-line-fitting call stayed
  #     split -- so a file written through Write was blocked as drift.
  if command -v jq >/dev/null 2>&1; then
    dir=$(make_fixture)
    mkdir -p "$dir/rtc_demo/rtc_demo"
    printf '[tool.ruff]\nline-length = 99\n\n[tool.ruff.lint]\nselect = ["UP015"]\n' >"$dir/pyproject.toml"
    git -C "$dir" add -A && git -C "$dir" commit -qm pyproject
    name=$(printf 'p%.0s' $(seq 1 77))
    printf 'def f(%s):\n    with open(%s, "r") as fh:\n        return fh.read()\n' "$name" "$name" \
      >"$dir/rtc_demo/rtc_demo/written_by_write.py"
    jq -n --arg p "$dir/rtc_demo/rtc_demo/written_by_write.py" '{tool_input: {file_path: $p}}' \
      | bash "$REPO_ROOT/.claude/hooks/format-code.sh"
    out=$(run_hook "$dir"); rc=$?
    expect_not_contains "a file format-code.sh just wrote is not graded as drift" "$out" "written_by_write.py"
    expect_exit "a file format-code.sh just wrote does not block" "$rc" 0
    rm -rf "$dir"

    # 48. An import added by one Edit and used by a later one survives the Edit
    #     in between: format-code.sh runs after every Edit, and autofixing F401
    #     deleted the import before the code that used it was written. The
    #     other safe fixes still apply (UP015 drops "r" in the same file).
    dir=$(make_fixture)
    mkdir -p "$dir/rtc_demo/rtc_demo"
    printf '[tool.ruff]\nline-length = 99\n\n[tool.ruff.lint]\nselect = ["F401", "UP015"]\n' \
      >"$dir/pyproject.toml"
    f="$dir/rtc_demo/rtc_demo/import_first.py"
    printf 'import os\n\n\ndef g(p):\n    with open(p, "r") as fh:\n        return fh.read()\n' >"$f"
    jq -n --arg p "$f" '{tool_input: {file_path: $p}}' | bash "$REPO_ROOT/.claude/hooks/format-code.sh"
    body=$(cat "$f")
    expect_contains "a not-yet-used import survives the format hook" "$body" "import os"
    expect_not_contains "other safe fixes still apply" "$body" '"r"'
    rm -rf "$dir"
  else
    skip "format-code.sh round-trip (47-48): no jq"
  fi
else
  skip "Phase 5 newline / deadline / format-code cases (45-47): no ruff the hook can resolve"
fi

# --- Documentation gate: whole-file and cross-file findings ----------------------

# 48. D12's byte cap is a whole-file budget reported at line 1. The added-line
#     narrowing dropped it, so a constitution grown past its cap by an edit in
#     the middle passed here and only CI said so.
dir=$(make_fixture)
for i in $(seq 1 150); do printf -- '- rule %03d %s\n' "$i" "$(printf 'a%.0s' $(seq 1 100))"; done >"$dir/AGENTS.md"
git -C "$dir" add -A && git -C "$dir" commit -qm constitution
awk 'NR >= 70 && NR <= 90 { $0 = $0 " " sprintf("%0100d", 0) } { print }' "$dir/AGENTS.md" >"$dir/AGENTS.md.new"
mv "$dir/AGENTS.md.new" "$dir/AGENTS.md"
out=$(run_hook "$dir"); rc=$?
expect_contains "a constitution grown past the byte cap mid-file is reported" "$out" "AGENTS.md:1: [D12]"
expect_exit "a constitution over the byte cap blocks the turn" "$rc" 2
rm -rf "$dir"

# 49. Renumbering a constitution heading breaks refs in files the change never
#     touched, and bare refs on unchanged lines of the constitution itself; the
#     per-file, added-line scope saw neither. The heading change now resolves
#     every section ref in the tracked corpus.
dir=$(make_fixture)
printf '# a\n\n## 6. Escalation\n\nsee §6 above.\n' >"$dir/AGENTS.md"
printf '# ref\n\nsee AGENTS.md §6 for escalation.\n' >"$dir/agent_docs/ref.md"
git -C "$dir" add -A && git -C "$dir" commit -qm numbered
printf '# a\n\n## 6. Escalation\n\nsee §6 above.\n\nmore text.\n' >"$dir/AGENTS.md"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "an edit that keeps the headings does not scan the corpus" "$out" "numbered headings changed"
expect_exit "an edit that keeps the headings does not block" "$rc" 0
sed -i 's/^## 6\. Escalation$/## 7. Escalation/' "$dir/AGENTS.md"
out=$(run_hook "$dir"); rc=$?
expect_contains "renumbering reports a ref in an untouched file" "$out" "agent_docs/ref.md:3: [D13]"
expect_contains "renumbering reports a bare ref on an unchanged constitution line" "$out" "AGENTS.md:5: [D13]"
expect_exit "renumbering that strands refs blocks the turn" "$rc" 2
rm -rf "$dir"

# 50. The import gate keyed on CLAUDE.md being in the change set, so removing
#     either constitution file -- the most complete loss of the import -- was
#     never reported.
for gone in AGENTS.md CLAUDE.md; do
  dir=$(make_fixture)
  printf '@AGENTS.md\n\n# c\n' >"$dir/CLAUDE.md"
  printf '# a\n' >"$dir/AGENTS.md"
  git -C "$dir" add -A && git -C "$dir" commit -qm split
  git -C "$dir" rm -q "$gone"
  out=$(run_hook "$dir"); rc=$?
  expect_contains "deleting $gone is reported as a lost import" "$out" "Constitution import missing"
  expect_exit "deleting $gone blocks the turn" "$rc" 2
  rm -rf "$dir"
done

# --- Background agents in flight -----------------------------------------------
#
# Stop fires at the main agent's turn end while background agents may still be
# writing the checkout. The hook defers rather than grade a half-written tree --
# only for labels that edit this checkout, and never by dropping the change.

# Like run_hook, with the Stop input given explicitly.
run_hook_input() {
  local dir="$1" input="$2"
  ( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_SKIP_BUILD=1 \
      bash "$HOOK" <<<"$input" 2>&1 >/dev/null )
}

# Stop input carrying one background task, shaped as Claude Code sends it.
bg_input() {
  printf '{"stop_hook_active": false, "background_tasks": [{"id": "t1", "type": "%s", "status": "%s", "description": "Compact READMEs"}]}' "$1" "$2"
}

add_missing_dep() {
  sed -i 's/^project(rtc_demo)/project(rtc_demo)\nfind_package(fmt REQUIRED)/' "$1/rtc_demo/CMakeLists.txt"
}

# 51. A blocking defect committed in-turn while an editing agent is in flight:
#     no block, no gate output, watermark kept -- and the next stop with nothing
#     in flight grades it. The second half is what makes this a deferral.
for label in subagent workflow teammate; do
  dir=$(make_fixture)
  base=$(git -C "$dir" rev-parse HEAD)
  echo "$base" >"$dir/.git/rtc-verify-base"
  add_missing_dep "$dir"
  git -C "$dir" commit -qam "missing dep, committed in-turn"
  out=$(run_hook_input "$dir" "$(bg_input "$label" running)"); rc=$?
  expect_exit "a $label in flight does not block" "$rc" 0
  expect_contains "a $label in flight is reported as a deferral" "$out" "Verification deferred"
  expect_not_contains "a $label in flight defers the gates" "$out" "find_package(fmt)"
  if [ "$(cat "$dir/.git/rtc-verify-base")" = "$base" ]; then
    pass "a deferral for a $label keeps the watermark"
  else
    fail "a deferral for a $label advanced the watermark"
  fi
  out=$(run_hook "$dir"); rc=$?
  expect_contains "a change deferred for a $label is graded once none are in flight" "$out" "find_package(fmt)"
  expect_exit "a change deferred for a $label blocks once none are in flight" "$rc" 2
  rm -rf "$dir"
done

# 51b. The deferral notice lists at most five tasks; a longer list must be
#      summarised, not kill the hook under `set -euo pipefail`.
dir=$(make_fixture)
add_missing_dep "$dir"
many=$(jq -n '{stop_hook_active: false, background_tasks: [range(7) | {id: "t\(.)", type: "subagent", status: "running", description: "agent \(.)"}]}')
out=$(run_hook_input "$dir" "$many"); rc=$?
expect_exit "seven agents in flight do not block" "$rc" 0
expect_contains "seven agents in flight are summarised" "$out" "(+2 more)"
expect_not_contains "the sixth agent is not listed" "$out" "agent 5"
rm -rf "$dir"

# 52. ...but not for work that cannot be mid-edit here: a shell (a CI watcher
#     would switch the gate off for its whole run), a label the hook does not
#     know, a task no longer in flight, or a malformed field.
for spec in 'shell|running' 'cloud session|running' 'subagent|completed'; do
  IFS='|' read -r label status <<<"$spec"
  dir=$(make_fixture)
  add_missing_dep "$dir"
  out=$(run_hook_input "$dir" "$(bg_input "$label" "$status")"); rc=$?
  expect_contains "a $label ($status) does not defer the gates" "$out" "find_package(fmt)"
  expect_exit "a $label ($status) still blocks" "$rc" 2
  rm -rf "$dir"
done
dir=$(make_fixture)
add_missing_dep "$dir"
out=$(run_hook_input "$dir" '{"stop_hook_active": false, "background_tasks": "bogus"}'); rc=$?
expect_contains "a malformed background_tasks does not defer the gates" "$out" "find_package(fmt)"
expect_exit "a malformed background_tasks still blocks" "$rc" 2
rm -rf "$dir"

# 53. A README kept beside the headers is not public surface: editing it asked
#     whether the package README reflected the edit. The header control keeps
#     the absence from holding just because the checklist never fires here.
dir=$(make_fixture)
mkdir -p "$dir/rtc_demo/include/rtc_demo"
echo '# se3' >"$dir/rtc_demo/include/rtc_demo/README.md"
git -C "$dir" add -A && git -C "$dir" commit -qm "doc beside headers"
printf '# se3\n\nmore.\n' >"$dir/rtc_demo/include/rtc_demo/README.md"
out=$(run_hook "$dir")
expect_not_contains "a README under include/ does not raise the README checklist" "$out" "public surface changed"
printf 'int demo_fn();\n' >"$dir/rtc_demo/include/rtc_demo/demo.hpp"
out=$(run_hook "$dir")
expect_contains "a header under include/ still raises the README checklist" "$out" "public surface changed"
rm -rf "$dir"

# --- Verdict reuse -------------------------------------------------------------
#
# 2026-09-30: sixteen turn ends over a tree that did not change between them
# each rebuilt and re-tested the same two packages, because the watermark is a
# commit and an uncommitted change stays "changed" however often it has passed.
# The hook now remembers the CONTENT it passed at.

# A test command that answers green (or red) and counts its calls, the twin of
# make_build_stub. $1 = file the calls are counted in, $2 = exit code.
make_test_stub() {
  local d
  d=$(mktemp -d)
  cat >"$d/test.sh" <<EOF
#!/usr/bin/env bash
echo "\$1" >>"$1"
if [ "$2" -eq 0 ]; then
  echo "Summary: 3 tests, 0 errors, 0 failures, 0 skipped"
else
  echo "Summary: 3 tests, 0 errors, 1 failure, 0 skipped"
fi
exit $2
EOF
  chmod +x "$d/test.sh"
  echo "$d"
}
calls() { wc -l <"$1" 2>/dev/null | tr -d ' ' || echo 0; }

# Phase 2 with both seams: a build that succeeds and a test command that is
# counted. Extra environment goes in front, e.g. RTC_VERIFY_NO_REUSE=1.
# Called as `--run` (see run_hook_build): the cases below are about when a
# build and a test run are repeated and when their verdict is reused, and
# --run is the call that builds. stdin is closed -- nothing there to read.
run_hook_green() {
  local dir="$1" bstub="$2" tstub="$3"
  shift 3
  ( cd "$dir" && env "$@" CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_BUILD_CMD="$bstub/build.sh" \
      RTC_VERIFY_TEST_CMD="$tstub/test.sh" \
      bash "$HOOK" --run </dev/null 2>&1 >/dev/null )
}
# The turn-end call with BOTH seams in place: were it to build or to test, the
# stubs would count it. Extra environment goes in front.
run_stop() {
  local dir="$1" bstub="$2" tstub="$3"
  shift 3
  ( cd "$dir" && env "$@" CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_BUILD_CMD="$bstub/build.sh" \
      RTC_VERIFY_TEST_CMD="$tstub/test.sh" \
      bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null )
}

# 54. The same working tree is verified once. The second --run over it runs
#     nothing; an edit, however small, runs everything again.
dir=$(make_fixture)
count=$(mktemp)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub "$count" 0)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a green change passes" "$rc" 0
expect_not_contains "the first --run over a change is not called unchanged" "$out" "nothing re-run"
if [ "$(calls "$count")" = 1 ]; then pass "the first --run tests the package"; else fail "the first --run ran the tests $(calls "$count") times"; fi
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_contains "the second --run over the same tree runs nothing" "$out" "nothing re-run"
expect_exit "the second --run over the same tree passes" "$rc" 0
if [ "$(calls "$count")" = 1 ]; then pass "the same tree is not tested twice"; else fail "the same tree was tested $(calls "$count") times"; fi
# 54b. Committing what was verified does not make it new.
git -C "$dir" commit -qam "the verified change"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
if [ "$(calls "$count")" = 1 ]; then pass "committing a verified change does not re-test it"; else fail "the commit of a verified change was tested again"; fi
# 54c. The switch turns the reuse off.
echo 'int existing() { return 2; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
before=$(calls "$count")
out=$(run_hook_green "$dir" "$bstub" "$tstub" RTC_VERIFY_NO_REUSE=1)
expect_not_contains "RTC_VERIFY_NO_REUSE runs the gates over an unchanged tree" "$out" "nothing re-run"
if [ "$(calls "$count")" = $((before + 1)) ]; then pass "RTC_VERIFY_NO_REUSE tests an unchanged tree again"; else fail "RTC_VERIFY_NO_REUSE did not re-test (calls $(calls "$count"), before $before)"; fi
# 54d. An edit after a pass is graded.
echo 'int existing() { return 3; }' >"$dir/rtc_demo/src/existing.cpp"
before=$(calls "$count")
out=$(run_hook_green "$dir" "$bstub" "$tstub")
expect_not_contains "an edit after a pass is not called unchanged" "$out" "nothing re-run"
if [ "$(calls "$count")" = $((before + 1)) ]; then pass "an edit after a pass is tested"; else fail "an edit after a pass was not tested"; fi
rm -rf "$dir" "$bstub" "$tstub" "$count"

# 54e. A pass with Phase 2 switched off is not a verdict on the tree: the next
#      --run over the same tree builds and tests it.
dir=$(make_fixture)
count=$(mktemp)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub "$count" 0)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook "$dir"); rc=$?
expect_exit "a stop with the build switched off passes" "$rc" 0
out=$(run_hook_green "$dir" "$bstub" "$tstub")
expect_not_contains "a tree passed without a build is not called unchanged" "$out" "nothing re-run"
if [ "$(calls "$count")" = 1 ]; then pass "a tree passed without a build is tested"; else fail "a tree passed without a build was never tested"; fi
rm -rf "$dir" "$bstub" "$tstub" "$count"

# 54f. Taking the tree id leaves nothing in the repository's object store.
dir=$(make_fixture)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub /dev/null 0)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
echo 'scratch that nobody added' >"$dir/untracked_note.txt"
before=$(find "$dir/.git/objects" -type f | sort)
out=$(run_hook_green "$dir" "$bstub" "$tstub")
out=$(run_hook_green "$dir" "$bstub" "$tstub")
expect_contains "the tree id still finds the unchanged tree" "$out" "nothing re-run"
if [ "$(find "$dir/.git/objects" -type f | sort)" = "$before" ]; then
  pass "the hook writes no object into the repository"
else
  fail "the hook left objects in .git/objects"
fi
rm -rf "$dir" "$bstub" "$tstub"

# 55. Only a pass is remembered: a blocked tree is graded again, whole.
dir=$(make_fixture)
add_missing_dep "$dir"
out=$(run_hook "$dir"); rc=$?
expect_exit "a blocking defect blocks" "$rc" 2
out=$(run_hook "$dir"); rc=$?
expect_contains "the same blocked tree is graded again" "$out" "find_package(fmt)"
expect_not_contains "a blocked tree is never called unchanged" "$out" "nothing re-run"
expect_exit "the same blocked tree blocks again" "$rc" 2
rm -rf "$dir"

# 55b. A red test is not remembered either.
dir=$(make_fixture)
count=$(mktemp)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub "$count" 1)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a red test blocks" "$rc" 2
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the same red tree blocks again" "$rc" 2
expect_not_contains "a red package is not reused" "$out" "not repeated"
if [ "$(calls "$count")" = 2 ]; then pass "a red package is tested again"; else fail "a red package was tested $(calls "$count") times over two runs"; fi
rm -rf "$dir" "$bstub" "$tstub" "$count"

# 56. A package keeps its verdict while the change set IN PACKAGES is the
#     same: a repository-level document edited after the code passed does not
#     re-test it, and the other gates still read that document.
dir=$(make_fixture)
count=$(mktemp)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub "$count" 0)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
printf '# docs\n\nA plan note written after the code passed.\n' >"$dir/agent_docs/notes.md"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_contains "a repository-level doc edit reuses the package verdict" "$out" "build/test not repeated for [rtc_demo]"
expect_not_contains "...and is not the whole-tree shortcut" "$out" "nothing re-run"
expect_exit "a repository-level doc edit passes" "$rc" 0
if [ "$(calls "$count")" = 1 ]; then pass "the package is not tested again for a repository-level doc"; else fail "a repository-level doc edit re-tested the package"; fi
# 56b. A file inside the package -- its README included -- re-grades it.
printf '# demo\n\nmore.\n' >"$dir/rtc_demo/README.md"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
expect_not_contains "a file inside the package ends the reuse" "$out" "not repeated"
if [ "$(calls "$count")" = 2 ]; then pass "a README inside the package re-tests it"; else fail "a README inside the package did not re-test it (calls $(calls "$count"))"; fi
# 56c. Reverting the package to what it passed at finds the verdict again.
printf '# demo\n' >"$dir/rtc_demo/README.md"
echo 'int existing() { return 9; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
if [ "$(calls "$count")" = 3 ]; then pass "a new package content is tested"; else fail "a new package content was not tested"; fi
rm -rf "$dir" "$bstub" "$tstub" "$count"

# 56e. The verdict is of the packages' content, not of the diff against the
#      watermark. A package graded green beside a blocking gate keeps its key;
#      another file of the package then changes and the watermark moves past
#      it without a build. The first file is again the whole change set -- and
#      the package is not the one that was graded.
if have_pyyaml; then
  dir=$(make_fixture)
  count=$(mktemp)
  bstub=$(make_build_stub 0)
  tstub=$(make_test_stub "$count" 0)
  echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
  mkdir -p "$dir/docs"
  printf 'a: [1, 2\n' >"$dir/docs/broken.yaml"
  out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
  expect_exit "56e setup: the other gate blocks" "$rc" 2
  if [ "$(calls "$count")" = 1 ]; then pass "56e setup: the package is graded beside the blocking gate"; else fail "56e setup: tested $(calls "$count") times"; fi
  rm -rf "$dir/docs"
  git -C "$dir" stash -q
  echo '# demo, changed and never graded' >"$dir/rtc_demo/README.md"
  git -C "$dir" commit -qam "another file of the package"
  git -C "$dir" rev-parse HEAD >"$dir/.git/rtc-verify-base"
  git -C "$dir" stash pop -q
  out=$(run_hook_green "$dir" "$bstub" "$tstub")
  expect_not_contains "a moved watermark does not revive the old verdict" "$out" "not repeated"
  if [ "$(calls "$count")" = 2 ]; then pass "the package is graded with the file the watermark skipped"; else fail "the package was not re-graded (calls $(calls "$count"))"; fi
  rm -rf "$dir" "$bstub" "$tstub" "$count"
else
  skip "56e needs PyYAML"
fi

# 56d. The reuse leaves the other gates on: a blocking defect in a repository
#      document still blocks while the package verdict is reused.
if have_pyyaml; then
  dir=$(make_fixture)
  count=$(mktemp)
  bstub=$(make_build_stub 0)
  tstub=$(make_test_stub "$count" 0)
  echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
  out=$(run_hook_green "$dir" "$bstub" "$tstub")
  mkdir -p "$dir/docs"
  printf 'a: [1, 2\n' >"$dir/docs/broken.yaml"
  out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
  expect_contains "the package verdict is reused beside a failing gate" "$out" "not repeated for [rtc_demo]"
  expect_contains "the failing gate still reports" "$out" "YAML parse failures"
  expect_exit "the failing gate still blocks" "$rc" 2
  rm -rf "$dir" "$bstub" "$tstub" "$count"
else
  skip "56d needs PyYAML"
fi

# --- A measurement holding the host --------------------------------------------
#
# 2026-09-30: an evaluation that launches one simulator per unit has none
# running between two units, and nine turn ends in a row started `colcon test`
# in that gap. The driver now lists itself in <workspace>/.rtc-verify-hold.

start_time_of() { sed 's/^.*) //' "/proc/$1/stat" | cut -d' ' -f20; }

# 57. A live process listed in the hold file defers build/test on the
#     simulator's terms; once it is gone the change is graded.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
count=$(mktemp)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub "$count" 0)
base=$(git -C "$dir" rev-parse HEAD)
echo "$base" >"$dir/.git/rtc-verify-base"
sleep 60 &
hpid=$!
echo "$hpid $(start_time_of "$hpid")" >"$ws/.rtc-verify-hold"
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
git -C "$dir" commit -qam "edit committed in-turn while a measurement runs"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_contains "a held host defers build/test" "$out" "Build/test deferred"
expect_contains "the deferral names the holder by pid" "$out" "$hpid: sleep 60"
expect_exit "a held host does not block the turn" "$rc" 0
if [ "$(calls "$count")" = 0 ]; then pass "nothing is tested on a held host"; else fail "a held host was tested beside"; fi
if [ "$(cat "$dir/.git/rtc-verify-base")" = "$base" ]; then pass "a hold keeps the watermark"; else fail "a hold advanced the watermark"; fi
# 57b. The same pid with another start time is another process: it holds nothing.
echo "$hpid 1" >"$ws/.rtc-verify-hold"
out=$(run_stop "$dir" "$bstub" "$tstub")
expect_not_contains "a reused pid does not hold the host" "$out" "Build/test deferred"
expect_contains "...so the turn end asks for the verdict" "$out" "build/test verdict missing for: rtc_demo"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
if [ "$(calls "$count")" = 1 ]; then pass "the change is tested when the holder is not the one listed"; else fail "a reused pid kept the gate off"; fi
# 57d. A hold line without a newline at its end is still a hold.
printf '%s %s' "$hpid" "$(start_time_of "$hpid")" >"$ws/.rtc-verify-hold"
echo 'int existing() { return 5; }' >"$dir/rtc_demo/src/existing.cpp"
before=$(calls "$count")
out=$(run_stop "$dir" "$bstub" "$tstub")
expect_contains "a hold line without a newline defers build/test" "$out" "Build/test deferred"
if [ "$(calls "$count")" = "$before" ]; then pass "nothing is tested beside an unterminated hold line"; else fail "an unterminated hold line was ignored"; fi
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
kill "$hpid" 2>/dev/null
wait "$hpid" 2>/dev/null
# 57c. A dead holder holds nothing: a driver that died cannot switch the gate off.
echo "$hpid" >"$ws/.rtc-verify-hold"
echo 'int existing() { return 2; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_stop "$dir" "$bstub" "$tstub")
expect_not_contains "a dead holder does not hold the host" "$out" "Build/test deferred"
expect_contains "...so the turn end asks for the verdict" "$out" "build/test verdict missing for: rtc_demo"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
if [ "$(calls "$count")" = 2 ]; then pass "the change is tested once the holder is gone"; else fail "a dead holder kept the gate off"; fi
rm -rf "$ws" "$bstub" "$tstub" "$count"

# 57e. The wrapper writes the line the hook reads: a command run under
#      with_verify_hold.sh holds the host while it runs, and not after.
HOLD_WRAPPER="$REPO_ROOT/repo_scripts/scripts/with_verify_hold.sh"
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
count=$(mktemp)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub "$count" 0)
echo "1 1" >"$ws/.rtc-verify-hold"   # another holder's line, stale
RTC_VERIFY_WORKSPACE="$ws" "$HOLD_WRAPPER" sleep 60 &
wpid=$!
for _ in $(seq 50); do grep -q "^$wpid " "$ws/.rtc-verify-hold" 2>/dev/null && break; sleep 0.1; done
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_contains "a command under the wrapper holds the host" "$out" "Build/test deferred"
expect_contains "the holder named is the wrapper" "$out" "$wpid: "
if [ "$(calls "$count")" = 0 ]; then pass "nothing is tested beside a wrapped command"; else fail "a wrapped command was tested beside"; fi
pkill -P "$wpid" sleep 2>/dev/null
wait "$wpid" 2>/dev/null
if [ "$(cat "$ws/.rtc-verify-hold" 2>/dev/null)" = "1 1" ]; then
  pass "the wrapper takes its own line out and leaves the others"
else
  fail "after the wrapper the hold file reads [$(cat "$ws/.rtc-verify-hold" 2>&1)]"
fi
out=$(run_stop "$dir" "$bstub" "$tstub")
expect_not_contains "the host is free once the wrapped command is over" "$out" "Build/test deferred"
expect_contains "...so the turn end asks for the verdict" "$out" "build/test verdict missing for: rtc_demo"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
if [ "$(calls "$count")" = 1 ]; then pass "the change is tested after the wrapped command"; else fail "the change was not tested after the wrapped command"; fi
# 57f. The wrapper returns the command's exit code and removes a file it emptied.
rm -f "$ws/.rtc-verify-hold"
RTC_VERIFY_WORKSPACE="$ws" "$HOLD_WRAPPER" bash -c 'exit 7'; rc=$?
expect_exit "the wrapper returns the command's exit code" "$rc" 7
if [ ! -e "$ws/.rtc-verify-hold" ]; then pass "the wrapper removes a hold file it emptied"; else fail "the wrapper left an empty hold file"; fi
"$HOLD_WRAPPER" >/dev/null 2>&1; rc=$?
expect_exit "the wrapper without a command is a usage error" "$rc" 2
rm -rf "$ws" "$bstub" "$tstub" "$count"

# 59. A run with the build switched off claims nothing while packages wait for
#     their build: the watermark stays. With nothing to build it advances.
dir=$(make_fixture)
base=$(git -C "$dir" rev-parse HEAD)
echo "$base" >"$dir/.git/rtc-verify-base"
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
git -C "$dir" commit -qam "a source edit committed in-turn"
out=$(run_hook "$dir"); rc=$?
expect_exit "a run with the build switched off passes" "$rc" 0
expect_contains "...and says what it did not do" "$out" "watermark kept"
if [ "$(cat "$dir/.git/rtc-verify-base")" = "$base" ]; then
  pass "a build that was switched off keeps the watermark"
else
  fail "a build that was switched off advanced the watermark"
fi
rm -rf "$dir"
dir=$(make_fixture)
base=$(git -C "$dir" rev-parse HEAD)
echo "$base" >"$dir/.git/rtc-verify-base"
printf '# docs\n\nmore.\n' >"$dir/agent_docs/notes.md"
git -C "$dir" commit -qam "a document, nothing to build"
out=$(run_hook "$dir")
if [ "$(cat "$dir/.git/rtc-verify-base")" = "$(git -C "$dir" rev-parse HEAD)" ]; then
  pass "with nothing to build the switch does not hold the watermark"
else
  fail "a document-only change did not advance the watermark"
fi
rm -rf "$dir"

# 60. A shell script in a package's source directories routes the package to
#     build/test; one anywhere else is linted and no more.
dir=$(make_fixture)
mkdir -p "$dir/rtc_demo/scripts" "$dir/rtc_demo/test"
printf '#!/usr/bin/env bash\necho one\n' >"$dir/rtc_demo/scripts/tool.sh"
printf '#!/usr/bin/env bash\necho one\n' >"$dir/rtc_demo/test/test_tool.sh"
git -C "$dir" add -A
git -C "$dir" commit -qm "two scripts"
printf '#!/usr/bin/env bash\necho two\n' >"$dir/rtc_demo/scripts/tool.sh"
out=$(run_hook "$dir")
expect_contains "an edited script under scripts/ builds its package" "$out" "BUILD_PKGS=[rtc_demo]"
git -C "$dir" checkout -q -- rtc_demo/scripts/tool.sh
printf '#!/usr/bin/env bash\necho new\n' >"$dir/rtc_demo/test/test_new.sh"
out=$(run_hook "$dir")
expect_contains "a new shell test under test/ builds its package" "$out" "BUILD_PKGS=[rtc_demo]"
rm -f "$dir/rtc_demo/test/test_new.sh"
printf '#!/usr/bin/env bash\necho scratch\n' >"$dir/rtc_demo/probe.sh"
printf '#!/usr/bin/env bash\necho root\n' >"$dir/helper.sh"
out=$(run_hook "$dir")
expect_contains "a script outside the source directories builds nothing" "$out" "BUILD_PKGS=[]"
rm -rf "$dir"

# 58. Every run leaves one line in the timing log, with its verdict.
dir=$(make_fixture)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub /dev/null 0)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
run_hook_green "$dir" "$bstub" "$tstub" >/dev/null
run_hook_green "$dir" "$bstub" "$tstub" >/dev/null
add_missing_dep "$dir"
run_hook_green "$dir" "$bstub" "$tstub" >/dev/null
log=$(cut -f2 "$dir/.git/rtc-verify-timing.log" 2>/dev/null | tr '\n' ' ')
if [ "$log" = "pass pass-unchanged blocked " ]; then
  pass "the timing log records pass, pass-unchanged and blocked"
else
  fail "the timing log reads [$log]"
fi
rm -rf "$dir" "$bstub" "$tstub"

# 61. A package that was built WITHOUT tests is UNVERIFIED, whatever the test
#     command answers. build.sh skips tests by default since 2026-10-01, and
#     `colcon test` on such a tree reports "0 tests" or the result files of an
#     earlier build -- a pass either way. run_build passes --tests, but the
#     verdict reads the tree itself (BUILD_TESTING in the CMake cache), so a
#     build path that forgets the flag cannot turn into a green.
#     The fixture sits at <ws>/src/repo so the hook's workspace (two levels up)
#     is a directory this test owns.
ws=$(mktemp -d)
mkdir -p "$ws/src" "$ws/build/rtc_demo"
dir=$(make_fixture)
mv "$dir" "$ws/src/repo"
dir="$ws/src/repo"
count=$(mktemp)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub "$count" 0)
echo 'BUILD_TESTING:BOOL=OFF' >"$ws/build/rtc_demo/CMakeCache.txt"
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a package built without tests blocks the turn" "$rc" 2
expect_contains "built-without-tests is named as the reason" "$out" "rtc_demo: colcon test NOT RUN — the package is built WITHOUT tests"
expect_contains "built-without-tests is UNVERIFIED, not a failure of the code" "$out" "UNVERIFIED"
if [ "$(calls "$count")" = 0 ]; then pass "a package built without tests is not tested"; else fail "the test command ran $(calls "$count") times over a tree with no tests"; fi
# 61b. The same tree with the tests built is graded normally.
echo 'BUILD_TESTING:BOOL=ON' >"$ws/build/rtc_demo/CMakeCache.txt"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the same change passes once the tests are built" "$rc" 0
if [ "$(calls "$count")" = 1 ]; then pass "a package built with tests is tested"; else fail "the test command ran $(calls "$count") times"; fi
# 61c. An untyped cache entry (a project that never declares the option) and
#      the other spellings of "off" count too.
for entry in 'BUILD_TESTING:UNINITIALIZED=OFF' 'BUILD_TESTING:BOOL=0' 'BUILD_TESTING:STRING=FALSE'; do
  echo "$entry" >"$ws/build/rtc_demo/CMakeCache.txt"
  echo "int existing() { return ${#entry}; }" >"$dir/rtc_demo/src/existing.cpp"
  out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
  expect_exit "'$entry' blocks" "$rc" 2
done
# 61d. No cache at all (ament_python, or the build seam of this suite) says
#      nothing and does not block -- every earlier green case relies on that.
rm -f "$ws/build/rtc_demo/CMakeCache.txt"
echo 'int existing() { return 7; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a package with no CMake cache is graded by its tests" "$rc" 0
# 61e. The PROC-3 path asks the same question of every package of the repo
#      before its workspace-wide `colcon test`.
add_rtc_base "$dir"
echo 'BUILD_TESTING:BOOL=OFF' >"$ws/build/rtc_demo/CMakeCache.txt"
echo 'int base_fn() { return 1; }' >"$dir/rtc_base/src/base.cpp"
out=$(run_hook_build "$dir" "$bstub"); rc=$?
expect_exit "PROC-3 over a tree built without tests blocks" "$rc" 2
expect_contains "PROC-3 names the packages built without tests" "$out" "PROC-3 broad test NOT RUN — built WITHOUT tests (BUILD_TESTING=OFF in the CMake cache): rtc_demo"
rm -rf "$ws" "$bstub" "$tstub" "$count"

# --- The turn end checks the verdict; --run produces it ------------------------
#
# 2026-10-01: the turn-end call stopped building and testing. Over two days it
# had spent 78% of its build/test time on tests, about 70% of that re-running a
# full suite the agent had run minutes earlier on the same tree, and in a month
# it had blocked 25 times without one real failure. It now only checks that
# the changed packages carry a green verdict for their present content;
# `verify-changes.sh --run`, called during the turn, is what builds, tests and
# records one.

# A build stand-in that succeeds and counts its calls ($1 = file to count in).
make_counting_build_stub() {
  local d
  d=$(mktemp -d)
  cat >"$d/build.sh" <<EOF
#!/usr/bin/env bash
echo "\$*" >>"$1"
exit 0
EOF
  chmod +x "$d/build.sh"
  echo "$d"
}
# run_stop (the turn-end call) and run_hook_green (--run), both with the two
# seams in place, are defined under "Verdict reuse".

# 62. A changed package with no verdict blocks the turn end, and the turn end
#     neither builds nor tests it.
dir=$(make_fixture)
bcount=$(mktemp)
tcount=$(mktemp)
bstub=$(make_counting_build_stub "$bcount")
tstub=$(make_test_stub "$tcount" 0)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a changed package without a verdict blocks the turn end" "$rc" 2
expect_contains "the block names the package that is owed" "$out" "build/test verdict missing for: rtc_demo"
expect_contains "the block names the command that pays it" "$out" ".claude/hooks/verify-changes.sh --run"
if [ "$(calls "$bcount")" = 0 ]; then pass "the turn end does not build"; else fail "the turn end built $(calls "$bcount") times"; fi
if [ "$(calls "$tcount")" = 0 ]; then pass "the turn end does not test"; else fail "the turn end tested $(calls "$tcount") times"; fi
# 62b. --run builds with --tests, tests, and says it passed.
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "--run over a green change passes" "$rc" 0
expect_contains "--run says what it built and tested" "$out" "verify-changes --run: PASS"
if [ "$(cat "$bcount")" = "-p rtc_demo --tests" ]; then pass "--run builds the package with its tests"; else fail "--run built with [$(cat "$bcount")]"; fi
if [ "$(calls "$tcount")" = 1 ]; then pass "--run tests the package"; else fail "--run tested $(calls "$tcount") times"; fi
# 62c. The turn end over the tree --run passed: nothing owed, nothing run.
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the turn end passes a tree --run passed" "$rc" 0
expect_contains "...as the unchanged tree it is" "$out" "nothing re-run"
# 62d. Committing that tree changes nothing, and the watermark moves to it.
git -C "$dir" commit -qam "what --run passed"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the turn end passes the commit of a tree --run passed" "$rc" 0
if [ "$(cat "$dir/.git/rtc-verify-base")" = "$(git -C "$dir" rev-parse HEAD)" ]; then
  pass "the watermark advances to that commit"
else
  fail "the watermark did not advance to the commit of a verified tree"
fi
# 62e. A repository-level document written afterwards: the verdict stands, the
#      other gates read the document.
git -C "$dir" reset -q --soft HEAD~1
git -C "$dir" rev-parse HEAD >"$dir/.git/rtc-verify-base"
printf '# docs\n\nWritten after the code passed.\n' >"$dir/agent_docs/notes.md"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a repository-level doc after --run passes the turn end" "$rc" 0
expect_contains "...on the package verdict --run left" "$out" "build/test not repeated for [rtc_demo]"
# 62f. An edit inside the package voids the verdict.
echo 'int existing() { return 2; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "an edit inside the package after --run blocks the turn end" "$rc" 2
expect_contains "...as a missing verdict" "$out" "build/test verdict missing for: rtc_demo"
if [ "$(calls "$bcount")" = 1 ] && [ "$(calls "$tcount")" = 1 ]; then
  pass "five turn ends built and tested nothing"
else
  fail "the turn ends ran the build $(($(calls "$bcount") - 1)) and the tests $(($(calls "$tcount") - 1)) times"
fi
# 62g. The timing log says which call each line was.
log=$(cut -f2,6 "$dir/.git/rtc-verify-timing.log" 2>/dev/null | tr '\t\n' '  ')
if [ "$log" = "blocked mode=stop pass mode=run pass-unchanged mode=stop pass-unchanged mode=stop pass mode=stop blocked mode=stop " ]; then
  pass "the timing log records the mode of each call"
else
  fail "the timing log reads [$log]"
fi
rm -rf "$dir" "$bstub" "$tstub" "$bcount" "$tcount"

# 62h. A red --run leaves no verdict: the turn end still owes the package.
dir=$(make_fixture)
bcount=$(mktemp)
tcount=$(mktemp)
bstub=$(make_counting_build_stub "$bcount")
tstub=$(make_test_stub "$tcount" 1)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "--run over a red test fails" "$rc" 2
expect_not_contains "...and does not say it passed" "$out" "verify-changes --run: PASS"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the turn end after a red --run blocks" "$rc" 2
expect_contains "...on the verdict that was never earned" "$out" "build/test verdict missing for: rtc_demo"
rm -rf "$dir" "$bstub" "$tstub" "$bcount" "$tcount"

# 63. PROC-3 (rtc_base / rtc_msgs) is owed as a whole, and not built at the
#     turn end either.
dir=$(make_fixture)
add_rtc_base "$dir"
bcount=$(mktemp)
bstub=$(make_counting_build_stub "$bcount")
tstub=$(make_test_stub /dev/null 0)
echo 'int base_fn() { return 1; }' >"$dir/rtc_base/src/base.cpp"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "an rtc_base change without a verdict blocks the turn end" "$rc" 2
expect_contains "the PROC-3 block says the whole workspace is owed" "$out" "build/test verdict missing for: every package (PROC-3"
if [ "$(calls "$bcount")" = 0 ]; then pass "the turn end does not start the PROC-3 build"; else fail "the turn end started the PROC-3 build"; fi
rm -rf "$dir" "$bstub" "$tstub" "$bcount"

# 64. A held host: --run refuses to build and remembers nothing; the turn end
#     defers while the hold lasts and asks for --run once it is over.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
bcount=$(mktemp)
tcount=$(mktemp)
bstub=$(make_counting_build_stub "$bcount")
tstub=$(make_test_stub "$tcount" 0)
sleep 60 &
hpid=$!
echo "$hpid $(start_time_of "$hpid")" >"$ws/.rtc-verify-hold"
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "--run on a held host does not pass" "$rc" 2
expect_contains "--run on a held host says why nothing ran" "$out" "build/test NOT run — a simulator"
expect_contains "...and names the holder" "$out" "$hpid: sleep 60"
if [ "$(calls "$bcount")" = 0 ] && [ "$(calls "$tcount")" = 0 ]; then pass "--run builds and tests nothing on a held host"; else fail "--run built or tested on a held host"; fi
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the turn end on a held host does not block" "$rc" 0
expect_contains "...it defers the missing verdict" "$out" "Build/test deferred"
kill "$hpid" 2>/dev/null
wait "$hpid" 2>/dev/null
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the turn end blocks once the hold is over" "$rc" 2
expect_contains "the refused --run left no verdict behind" "$out" "build/test verdict missing for: rtc_demo"
rm -rf "$ws" "$bstub" "$tstub" "$bcount" "$tcount"

# 64b. A build running in the workspace while a verdict is missing: the turn
#      end says to wait for it (it is most likely the --run), and builds nothing.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
bcount=$(mktemp)
bstub=$(make_counting_build_stub "$bcount")
tstub=$(make_test_stub /dev/null 0)
bin=$(mktemp -d)
write_rival "$bin/colcon"
(cd "$ws" && exec "$bin/colcon") >/dev/null 2>&1 &
rpid=$!
if wait_for_name "$rpid" colcon; then
  echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
  out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
  expect_exit "a missing verdict beside a running build blocks the turn end" "$rc" 2
  expect_contains "...and says a build is running" "$out" "a build is running in this colcon workspace"
  expect_contains "...naming it by pid" "$out" "$rpid: /bin/bash $bin/colcon"
  if [ "$(calls "$bcount")" = 0 ]; then pass "the turn end builds nothing beside it"; else fail "the turn end built beside a running build"; fi
else
  fail "the colcon stand-in never showed up under its own name"
fi
kill "$rpid" 2>/dev/null
wait "$rpid" 2>/dev/null
rm -rf "$ws" "$bstub" "$tstub" "$bcount" "$bin"

# 65. --run takes no input: with a stdin that never closes it still finishes.
#     An unknown argument is a usage error, not a turn-end call.
dir=$(make_fixture)
fifo="$dir/../$(basename "$dir").fifo"
mkfifo "$fifo"
exec 9<>"$fifo"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" timeout 60 bash "$HOOK" --run <&9 2>&1 >/dev/null ); rc=$?
expect_exit "--run does not wait on stdin" "$rc" 0
exec 9<&-
rm -f "$fifo"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" bash "$HOOK" --build </dev/null 2>&1 >/dev/null ); rc=$?
expect_exit "an unknown argument is a usage error" "$rc" 64
expect_contains "...and prints the usage" "$out" "usage: verify-changes.sh [--run]"
rm -rf "$dir"

# --- The colcon command line itself ----------------------------------------------
#
# 2026-10-01: from its first version the per-package branch read the tests'
# verdict with `colcon test-result --packages-select <pkg>`. That verb has no
# such option: colcon printed a usage error, the error text holds no
# "<n> failures", and `colcon test` itself exits 0 on a failing test -- so a
# package whose tests ran was recorded green whether they passed or not. Every
# case above replaces both colcon calls with RTC_VERIFY_TEST_CMD, which is why
# none of them could see it. The cases below leave that seam alone and put a
# stand-in `colcon` on PATH that refuses what the real one refuses.

# The two verbs as the hook meets them (measured against colcon-core 0.21 in a
# scratch workspace with one failing ctest):
#   test         exits 0 on a failing test UNLESS --return-code-on-test-failure
#                is given; a --packages-select name it does not know is a
#                warning, nothing tested, exit 0; a package whose ctest could not
#                run fails the call whatever the flags; the summary line counts
#                the packages that finished
#   test-result  takes --test-result-base / --verbose / --all and nothing else
#                -- anything else is a usage error, exit 2; lists every result
#                file under the base that holds a failure, however old
# FAKE_COLCON_MODE = green | red | crash | unknown; FAKE_COLCON_CALLS = call log;
# FAKE_COLCON_RED = the packages that fail in mode red (default: all of them).
# A "result file" here is one line: "<summary>|<failing test name>".
make_fake_colcon() {
  local d
  d=$(mktemp -d)
  cat >"$d/colcon" <<'EOF'
#!/usr/bin/env bash
verb="${1:-}"
shift || true
echo "$verb $*" >>"${FAKE_COLCON_CALLS:-/dev/null}"
mode="${FAKE_COLCON_MODE:-green}"
case "$verb" in
  test)
    pkgs=()
    strict=""
    while [ $# -gt 0 ]; do
      case "$1" in
        --packages-select)
          shift
          while [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; do pkgs+=("$1"); shift; done
          ;;
        --return-code-on-test-failure) strict=1; shift ;;
        --event-handlers) shift 2 ;;
        *) echo "colcon: error: unrecognized arguments: $1" >&2; exit 2 ;;
      esac
    done
    if [ ${#pkgs[@]} -eq 0 ]; then
      for manifest in src/*/*/package.xml; do
        [ -f "$manifest" ] && pkgs+=("$(basename "$(dirname "$manifest")")")
      done
    fi
    case "$mode" in
      unknown)
        echo "WARNING:colcon.colcon_core.package_selection:ignoring unknown package '${pkgs[0]}' in --packages-select" >&2
        echo "Summary: 0 packages finished [0.11s]"
        exit 0
        ;;
      crash)
        echo "Summary: 0 packages finished [0.11s]"
        echo "  1 package failed: ${pkgs[0]}"
        exit 1
        ;;
    esac
    failed=()
    for p in "${pkgs[@]}"; do
      mkdir -p "build/$p/Testing/20261001-0900"
      if [ "$mode" = red ] && case " ${FAKE_COLCON_RED:-${pkgs[*]}} " in *" $p "*) true ;; *) false ;; esac; then
        echo "2 tests, 0 errors, 1 failure, 0 skipped|demo.Fails" >"build/$p/Testing/20261001-0900/Test.xml"
        failed+=("$p")
      else
        echo "2 tests, 0 errors, 0 failures, 0 skipped|" >"build/$p/Testing/20261001-0900/Test.xml"
      fi
    done
    if [ ${#pkgs[@]} -eq 1 ]; then
      echo "Summary: 1 package finished [0.52s]"
    else
      echo "Summary: ${#pkgs[@]} packages finished [0.52s]"
    fi
    if [ ${#failed[@]} -gt 0 ]; then
      echo "  ${#failed[@]} package had test failures: ${failed[*]}"
      [ -n "$strict" ] && exit 1
    fi
    exit 0
    ;;
  test-result)
    base=build
    verbose=""
    while [ $# -gt 0 ]; do
      case "$1" in
        --test-result-base) base="$2"; shift 2 ;;
        --verbose) verbose=1; shift ;;
        --all) shift ;;
        *)
          echo "usage: colcon test-result [-h] [--test-result-base TEST_RESULT_BASE] [--all]" >&2
          echo "colcon: error: unrecognized arguments: $*" >&2
          exit 2
          ;;
      esac
    done
    red=0
    while IFS= read -r file; do
      content=$(cat "$file")
      case "${content%%|*}" in *" 0 errors, 0 failures"*) continue ;; esac
      echo "$file: ${content%%|*}"
      [ -n "$verbose" ] && echo "- ${content#*|}"
      red=1
    done < <(find "$base" -name Test.xml 2>/dev/null | sort)
    echo
    echo "Summary: 2 tests, 0 errors, $red failures, 0 skipped"
    exit "$red"
    ;;
  *) echo "colcon: error: argument verb_name: invalid choice: '$verb'" >&2; exit 2 ;;
esac
EOF
  chmod +x "$d/colcon"
  echo "$d"
}

# --run with the build stubbed green and the stand-in colcon first on PATH.
# $1 = repo dir, $2 = build stub dir, $3 = fake colcon dir, rest = environment.
run_hook_colcon() {
  local dir="$1" bstub="$2" fake="$3"
  shift 3
  ( cd "$dir" && env "$@" PATH="$fake:$PATH" CLAUDE_PROJECT_DIR="$dir" \
      RTC_VERIFY_BUILD_CMD="$bstub/build.sh" \
      bash "$HOOK" --run </dev/null 2>&1 >/dev/null )
}

# 66. A package whose tests FAIL is red: --run blocks, names the failing result
#     and the failing test, and remembers no verdict.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
bstub=$(make_build_stub 0)
fake=$(make_fake_colcon)
ccalls=$(mktemp)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
# A failure an earlier run left on disk: ctest keeps one Testing/<stamp>/ per
# run, so `colcon test-result` goes on listing it after the test was fixed.
mkdir -p "$ws/build/rtc_demo/Testing/20260901-0000"
echo "2 tests, 0 errors, 1 failure, 0 skipped|demo.FixedLongAgo" >"$ws/build/rtc_demo/Testing/20260901-0000/Test.xml"
touch -d '2026-09-01 00:00:00' "$ws/build/rtc_demo/Testing/20260901-0000/Test.xml"
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=red FAKE_COLCON_CALLS="$ccalls"); rc=$?
expect_exit "a package whose tests fail blocks --run" "$rc" 2
expect_contains "...as a test failure" "$out" "rtc_demo: colcon test FAILED"
expect_contains "...naming the result file this run wrote" "$out" "build/rtc_demo/Testing/20261001-0900/Test.xml: 2 tests, 0 errors, 1 failure"
expect_contains "...and the failing test" "$out" "- demo.Fails"
expect_not_contains "a failure left on disk by an earlier run is not reported" "$out" "20260901-0000"
if grep -qxF "test --packages-select rtc_demo --return-code-on-test-failure --event-handlers console_direct+" "$ccalls"; then
  pass "colcon test is asked for an exit code that follows the tests"
else
  fail "colcon test was called as: $(cat "$ccalls")"
fi
if grep -q '^test-result .*--packages-select' "$ccalls"; then
  fail "colcon test-result was given --packages-select, which it does not have"
else
  pass "colcon test-result is not given an option it does not have"
fi
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null ); rc=$?
expect_exit "a red --run leaves the turn end owing the package" "$rc" 2
expect_contains "...by name" "$out" "build/test verdict missing for: rtc_demo"

# 66b. The same tree with the tests green passes -- the old failure still on
#      disk does not make it red -- and the verdict is remembered.
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=green FAKE_COLCON_CALLS="$ccalls"); rc=$?
expect_exit "a package whose tests pass passes --run" "$rc" 0
expect_contains "...and says what it tested" "$out" "built and tested [rtc_demo]"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null ); rc=$?
expect_exit "the turn end over a green --run passes" "$rc" 0
rm -rf "$ws" "$bstub" "$fake" "$ccalls"

# 67. Exit 0 is not a pass by itself: colcon that tested NOTHING (a name it does
#     not know is a warning, not an error) leaves the package unverified.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
bstub=$(make_build_stub 0)
fake=$(make_fake_colcon)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=unknown); rc=$?
expect_exit "a colcon test that tested nothing blocks --run" "$rc" 2
expect_contains "...as unverified, with the count colcon gave" "$out" "rtc_demo: colcon test exited 0 but its summary counts 0 finished packages, not 1 — UNVERIFIED"
# 67b. A test run that fails without leaving a failing result (a crashed or
#      killed test binary writes none) is unverified, not green.
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=crash); rc=$?
expect_exit "a colcon test that fails without a result blocks --run" "$rc" 2
expect_contains "...as unverified" "$out" "rtc_demo: colcon test exited 1 with no parseable result summary — UNVERIFIED"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null ); rc=$?
expect_exit "neither leaves a verdict" "$rc" 2
rm -rf "$ws" "$bstub" "$fake"

# 68. The PROC-3 path past its build -- the one branch no case reached, because
#     a build stub that succeeds falls through to `colcon test`. Red, then green.
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
add_rtc_base "$dir"
bstub=$(make_build_stub 0)
fake=$(make_fake_colcon)
ccalls=$(mktemp)
echo 'int base_fn() { return 1; }' >"$dir/rtc_base/src/base.cpp"
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=red FAKE_COLCON_CALLS="$ccalls"); rc=$?
expect_exit "a failing test in the PROC-3 run blocks --run" "$rc" 2
expect_contains "...as a PROC-3 test failure" "$out" "PROC-3 broad test failed:"
expect_contains "...naming the failing test" "$out" "- demo.Fails"
if grep -qxF "test --return-code-on-test-failure --event-handlers console_direct+" "$ccalls"; then
  pass "the PROC-3 colcon test covers the workspace and returns the tests' exit code"
else
  fail "the PROC-3 colcon test was called as: $(cat "$ccalls")"
fi
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=unknown); rc=$?
expect_contains "a PROC-3 run that tested nothing is unverified" "$out" "PROC-3 broad test: colcon test exited 0 but its summary counts 0 finished packages, not one or more — UNVERIFIED"
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=green); rc=$?
expect_exit "a green PROC-3 run passes --run" "$rc" 0
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null ); rc=$?
expect_exit "...and the turn end after it" "$rc" 0
rm -rf "$ws" "$bstub" "$fake" "$ccalls"

# A second package beside rtc_demo, committed. $1 = repo dir.
add_rtc_other() {
  local dir="$1"
  mkdir -p "$dir/rtc_other/src"
  sed 's/rtc_demo/rtc_other/' "$dir/rtc_demo/package.xml" >"$dir/rtc_other/package.xml"
  printf 'cmake_minimum_required(VERSION 3.16)\nproject(rtc_other)\nadd_library(rtc_other src/other.cpp)\n' \
    >"$dir/rtc_other/CMakeLists.txt"
  echo 'int other() { return 0; }' >"$dir/rtc_other/src/other.cpp"
  echo '# other' >"$dir/rtc_other/README.md"
  git -C "$dir" add -A
  git -C "$dir" commit -qm "add rtc_other"
}

# 70. Two changed packages are tested ONE AT A TIME, each by its own colcon
#     test and with its own verdict: the green one is recorded although the
#     other is red. (One colcon test over both was tried and taken out in
#     review: colcon runs packages side by side, which puts a test that asserts
#     a wall-clock budget under its neighbour's load, and one call has one exit
#     code for every package in it.)
dir=$(make_nested_fixture)
ws=$(cd "$dir/../.." && pwd -P)
add_rtc_other "$dir"
bcount=$(mktemp)
bstub=$(make_counting_build_stub "$bcount")
fake=$(make_fake_colcon)
ccalls=$(mktemp)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
echo 'int other() { return 1; }' >"$dir/rtc_other/src/other.cpp"
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=red FAKE_COLCON_RED=rtc_other FAKE_COLCON_CALLS="$ccalls"); rc=$?
expect_exit "a failing package beside a passing one blocks --run" "$rc" 2
if [ "$(calls "$bcount")" = 2 ]; then pass "each package is built by its own call"; else fail "two packages took $(calls "$bcount") build calls"; fi
if [ "$(grep -c '^test ' "$ccalls")" = 2 ] \
   && grep -qxF "test --packages-select rtc_demo --return-code-on-test-failure --event-handlers console_direct+" "$ccalls" \
   && grep -qxF "test --packages-select rtc_other --return-code-on-test-failure --event-handlers console_direct+" "$ccalls"; then
  pass "each package is tested by its own colcon test"
else
  fail "the two packages were tested as: $(grep '^test ' "$ccalls")"
fi
expect_contains "the report names the package that failed" "$out" "rtc_other: colcon test FAILED"
expect_not_contains "...and not the package whose tests passed" "$out" "rtc_demo: colcon test FAILED"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null ); rc=$?
expect_contains "the red package stays owed" "$out" "build/test verdict missing for: rtc_other"
expect_not_contains "...and the green one beside it keeps its verdict" "$out" "verdict missing for: rtc_demo"
# 70b. colcon that tested nothing leaves the package it was asked for unverified.
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=unknown); rc=$?
expect_exit "a package colcon did not test blocks --run" "$rc" 2
expect_contains "...under its own name" "$out" "rtc_other: colcon test exited 0 but its summary counts 0 finished packages, not 1 — UNVERIFIED"
# 70c. Green: both verdicts are recorded and the turn end owes nothing.
echo 'int existing() { return 2; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_colcon "$dir" "$bstub" "$fake" FAKE_COLCON_MODE=green); rc=$?
expect_exit "two green packages pass --run" "$rc" 0
expect_contains "...and both are named" "$out" "built and tested [rtc_demo rtc_other]"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null ); rc=$?
expect_exit "the turn end over two green packages passes" "$rc" 0
rm -rf "$ws" "$bstub" "$fake" "$ccalls" "$bcount"

# 71. colcon.pkg at a package root turns the package's ctest parallel. A turn
#     that adds nothing else is still graded: the package is owed a test run,
#     and the domain gate reads the claimants that would now overlap.
dir=$(make_testgate_fixture 60 61)
git -C "$dir" add -A && git -C "$dir" commit -qm "two claimants, sequential"
printf 'ctest-args: ["-j", "4"]\n' >"$dir/rtc_demo/colcon.pkg"
out=$(run_hook "$dir"); rc=$?
expect_contains "a colcon.pkg-only change routes its package to build/test" "$out" "BUILD_PKGS=[rtc_demo]"
expect_contains "...and runs the test gates" "$out" "t_demo (rtc_demo/CMakeLists.txt:"
expect_contains "...which refuse a claimant without its domain's lock" "$out" "claims ROS_DOMAIN_ID=60, and rtc_demo/colcon.pkg runs this package's ctest with -j 4"
expect_exit "...and block the turn" "$rc" 2
# 71b. With the lock the same change passes.
printf 'set_tests_properties(t_demo PROPERTIES RESOURCE_LOCK ros_domain_60)\n' >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir"); rc=$?
expect_not_contains "a claimant holding its lock is not reported" "$out" "Test isolation gates"
expect_exit "...and does not block" "$rc" 0
rm -rf "$dir"

# 69. A verdict recorded by the reading that could not see a failing test is
#     not honoured: the key carries the reading's tag, and an entry without it
#     (what every checkout holds from before the fix) matches nothing.
dir=$(make_fixture)
count=$(mktemp)
bstub=$(make_build_stub 0)
tstub=$(make_test_stub "$count" 0)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$bstub" "$tstub")
printf '# docs\n\nWritten after the code passed.\n' >"$dir/agent_docs/notes.md"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a tagged verdict is honoured at the turn end" "$rc" 0
if grep -q '^rtc_demo t2:' "$dir/.git/rtc-verify-pass-pkgs"; then
  pass "the verdict is recorded under the reading's tag"
else
  fail "the verdict file holds: $(cat "$dir/.git/rtc-verify-pass-pkgs" 2>&1)"
fi
sed -i 's/^rtc_demo t2:/rtc_demo /' "$dir/.git/rtc-verify-pass-pkgs"
printf '# docs\n\nWritten after the code passed. Edited once more.\n' >"$dir/agent_docs/notes.md"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a verdict without the tag is not honoured" "$rc" 2
expect_contains "...the package is owed again" "$out" "build/test verdict missing for: rtc_demo"
# 69b. The whole-tree pass is the same kind of record and carries the same tag.
#      Without it a tree the old reading passed would leave the turn end at
#      "nothing re-run" before any package key was looked at.
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the tree passes --run again" "$rc" 0
if grep -q '^t2:' "$dir/.git/rtc-verify-pass-tree"; then
  pass "the whole-tree pass is recorded under the reading's tag"
else
  fail "the pass-tree file holds: $(cat "$dir/.git/rtc-verify-pass-tree" 2>&1)"
fi
out=$(run_stop "$dir" "$bstub" "$tstub")
expect_contains "a tagged whole-tree pass is honoured" "$out" "nothing re-run"
sed -i 's/^t2://' "$dir/.git/rtc-verify-pass-tree"
sed -i 's/^rtc_demo t2:/rtc_demo /' "$dir/.git/rtc-verify-pass-pkgs"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_not_contains "a whole-tree pass without the tag is not honoured" "$out" "nothing re-run"
expect_exit "...and the tree is graded again" "$rc" 2
rm -rf "$dir" "$bstub" "$tstub" "$count"

# --- A --run in flight ----------------------------------------------------------
#
# --run is made to be backgrounded: it can take minutes, and the agent goes on
# working -- ending a turn, editing, committing -- while it does.

# "<pid> <start time>" of a live process, the line a running --run keeps in
# its lock. $1 = pid.
lock_line() { printf '%s %s\n' "$1" "$(sed 's/^.*) //' "/proc/$1/stat" | cut -d' ' -f20)"; }

# 72. A --run that is running is something to wait for, whatever it is doing:
#     the turn end says so instead of asking for a second one, and a second one
#     refuses to start.
dir=$(make_fixture)
bcount=$(mktemp)
tcount=$(mktemp)
bstub=$(make_counting_build_stub "$bcount")
tstub=$(make_test_stub "$tcount" 0)
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
sleep 60 &
holder=$!
lock_line "$holder" >"$dir/.git/rtc-verify-run.lock"
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a missing verdict beside a running --run blocks the turn end" "$rc" 2
expect_contains "...naming the --run by pid" "$out" "$holder: verify-changes.sh --run"
expect_contains "...and saying to wait for it" "$out" "wait for it to finish"
expect_not_contains "...not to start another" "$out" "the turn end does not build or test"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "a second --run refuses to start" "$rc" 2
expect_contains "...and says which one is running" "$out" "another --run is already running in this checkout ($holder: verify-changes.sh --run)"
if [ "$(calls "$bcount")" = 0 ]; then pass "the second --run builds nothing"; else fail "the second --run built beside the first"; fi
if [ "$(cat "$dir/.git/rtc-verify-run.lock")" = "$(lock_line "$holder")" ]; then
  pass "the second --run leaves the first one's lock alone"
else
  fail "the lock of the running --run was overwritten or removed"
fi
kill "$holder" 2>/dev/null
wait "$holder" 2>/dev/null
# 72b. A lock a killed --run left behind names a process that is gone: it holds
#      nothing, --run takes its place and removes its own lock when it ends.
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_contains "a stale lock does not make the turn end wait" "$out" "the turn end does not build or test"
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "--run starts over a stale lock" "$rc" 0
if [ "$(calls "$bcount")" = 1 ]; then pass "...and builds"; else fail "--run over a stale lock built $(calls "$bcount") times"; fi
if [ ! -e "$dir/.git/rtc-verify-run.lock" ]; then pass "--run removes its lock when it ends"; else fail "--run left its lock behind"; fi
# 72c. A blocked --run removes its lock too.
echo 'int existing() { return 2; }' >"$dir/rtc_demo/src/existing.cpp"
rstub=$(make_build_stub 1)
out=$(run_hook_build "$dir" "$rstub"); rc=$?
expect_exit "a --run whose build fails blocks" "$rc" 2
if [ ! -e "$dir/.git/rtc-verify-run.lock" ]; then pass "a blocked --run removes its lock"; else fail "a blocked --run left its lock behind"; fi
rm -rf "$dir" "$bstub" "$tstub" "$rstub" "$bcount" "$tcount"

# 73. A package edited while --run was building it gets no verdict: the key the
#     verdict would be filed under names content the compiler never saw.
dir=$(make_fixture)
count=$(mktemp)
tstub=$(make_test_stub "$count" 0)
estub=$(mktemp -d)
printf '#!/usr/bin/env bash\necho "int existing() { return 99; }" >"%s/rtc_demo/src/existing.cpp"\nexit 0\n' "$dir" \
  >"$estub/build.sh"
chmod +x "$estub/build.sh"
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$estub" "$tstub"); rc=$?
expect_exit "a package edited during its build blocks --run" "$rc" 2
expect_contains "...and says the tested content is gone" "$out" "rtc_demo: built and tested green, but its files changed while that ran — no verdict recorded"
out=$(run_stop "$dir" "$estub" "$tstub"); rc=$?
expect_contains "...and the package stays owed" "$out" "build/test verdict missing for: rtc_demo"
# 73b. The same tree, left alone while --run runs, is recorded.
bstub=$(make_build_stub 0)
out=$(run_hook_green "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the tree left alone passes --run" "$rc" 0
rm -rf "$dir" "$bstub" "$estub" "$tstub" "$count"

# 74. A commit made while --run was running is not stepped over: the watermark
#     stays where the change set was taken from, and the next call grades what
#     the commit brought -- reusing the package verdict --run did earn.
dir=$(make_fixture)
count=$(mktemp)
tstub=$(make_test_stub "$count" 0)
out=$(run_hook "$dir")
base=$(git -C "$dir" rev-parse HEAD)
if [ "$(cat "$dir/.git/rtc-verify-base" 2>/dev/null)" = "$base" ]; then
  pass "precondition: the watermark is the commit the turn starts from"
else
  fail "precondition: no watermark at the starting commit"
fi
cstub=$(mktemp -d)
printf '#!/usr/bin/env bash\ncd "%s" || exit 1\necho "# late" >agent_docs/late.md\ngit add agent_docs/late.md && git commit -qm "committed while --run was building"\nexit 0\n' "$dir" \
  >"$cstub/build.sh"
chmod +x "$cstub/build.sh"
echo 'int existing() { return 1; }' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook_green "$dir" "$cstub" "$tstub"); rc=$?
expect_exit "a --run with a commit made beside it still passes what it graded" "$rc" 0
expect_contains "...and says the watermark stayed" "$out" "the working tree changed while the gates ran -- watermark kept"
if [ "$(cat "$dir/.git/rtc-verify-base")" = "$base" ]; then
  pass "the watermark does not step over the commit"
else
  fail "the watermark moved to $(cat "$dir/.git/rtc-verify-base"), past a commit no gate read (started at $base)"
fi
if [ "$(tail -n 1 "$dir/.git/rtc-verify-timing.log" | cut -f2)" = "pass-tree-moved" ]; then
  pass "the timing log says the tree moved"
else
  fail "the timing log says: $(tail -n 1 "$dir/.git/rtc-verify-timing.log")"
fi
bstub=$(make_build_stub 0)
before=$(calls "$count")
out=$(run_stop "$dir" "$bstub" "$tstub"); rc=$?
expect_exit "the next turn end grades the commit and passes" "$rc" 0
expect_contains "...reusing the verdict --run earned" "$out" "build/test not repeated for [rtc_demo]"
if [ "$(cat "$dir/.git/rtc-verify-base")" = "$(git -C "$dir" rev-parse HEAD)" ] && [ "$(calls "$count")" = "$before" ]; then
  pass "...and only then moves the watermark, testing nothing again"
else
  fail "after the turn end the watermark is $(cat "$dir/.git/rtc-verify-base") and the tests ran $(calls "$count") times"
fi
rm -rf "$dir" "$bstub" "$cstub" "$tstub" "$count"

printf '\n%d passed, %d failed, %d skipped\n' "$PASS" "$FAIL" "$SKIP"
[ "$FAIL" -eq 0 ]
