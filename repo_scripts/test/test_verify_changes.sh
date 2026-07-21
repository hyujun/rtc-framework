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

fail() {
  printf '  FAIL: %s\n' "$1" >&2
  FAIL=$((FAIL + 1))
}
pass() {
  printf '  ok: %s\n' "$1"
  PASS=$((PASS + 1))
}

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

echo "verify-changes.sh end-to-end routing"

# 1. A CMakeLists-only change must reach the package.xml co-update gate.
#    This is the case the double early-exit used to drop: the change that
#    triggers the gate was also the change that never got there.
dir=$(make_fixture)
sed -i 's/^project(rtc_demo)/project(rtc_demo)\nfind_package(fmt REQUIRED)/' "$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir")
expect_contains "CMake-only change reaches the find_package co-update gate" "$out" "find_package(fmt)"
rm -rf "$dir"

# 2. Formatting churn alongside a CMake change must not be graded "pure
#    format". is_pure_format() only inspects source files, so a commit whose
#    source edits are cosmetic used to skip Phase 1 wholesale, taking the
#    unrelated CMake gate down with it.
dir=$(make_fixture)
printf 'int existing()  {  return 0;  }\n' >"$dir/rtc_demo/src/existing.cpp"
sed -i 's/^project(rtc_demo)/project(rtc_demo)\nfind_package(fmt REQUIRED)/' "$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir")
expect_contains "reformat + CMake change still runs the co-update gate" "$out" "find_package(fmt)"
rm -rf "$dir"

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
dir=$(make_fixture)
printf 'a: [1, 2\nb: {\n' >"$dir/rtc_demo/config/broken.yaml"
git -C "$dir" add -A
out=$(run_hook "$dir")
expect_contains "broken YAML is reported" "$out" "broken.yaml"
rm -rf "$dir"

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
dir=$(make_fixture)
printf 'int ur5e_thing()  {  return 0;  }\n' >"$dir/rtc_demo/src/existing.cpp"
git -C "$dir" commit -qam "robot mention at HEAD"
printf 'int ur5e_thing() {   return 0;   }\n' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook "$dir")
expect_not_contains "pure formatting skips the ARCH grep" "$out" "ARCH-1"
rm -rf "$dir"

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
out=$(run_hook "$dir")
expect_contains "ARCH-7 catches a new rtc_* executable" "$out" "ARCH-7"
rm -rf "$dir"

# 9. ...but an explicitly exempt target (agnostic node / example) is not, or
#    the documented exceptions would be unusable.
dir=$(make_fixture)
printf 'add_executable(example_demo src/existing.cpp)  # ARCH-7-exempt: example\n' >>"$dir/rtc_demo/CMakeLists.txt"
out=$(run_hook "$dir")
expect_not_contains "ARCH-7 respects the exempt marker" "$out" "ARCH-7"
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
dir=$(make_fixture)
printf 'a: 1\n---\nb: 2\n' >"$dir/rtc_demo/config/multi.yaml"
out=$(run_hook "$dir")
expect_not_contains "multi-document YAML is accepted" "$out" "YAML parse failures"
rm -rf "$dir"

# 22. Interpreter noise on stderr is not a parse failure. The gate used to key
#     on "stderr is non-empty", so a ResourceWarning blocked the turn.
dir=$(make_fixture)
printf 'a: 1\n' >"$dir/rtc_demo/config/ok.yaml"
out=$( cd "$dir" && CLAUDE_PROJECT_DIR="$dir" RTC_VERIFY_SKIP_BUILD=1 PYTHONDEVMODE=1 \
        bash "$HOOK" <<<'{"stop_hook_active": false}' 2>&1 >/dev/null )
expect_not_contains "an interpreter warning is not a parse failure" "$out" "YAML parse failures"
rm -rf "$dir"

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
out=$(run_hook "$dir")
expect_not_contains "a pre-existing doc defect on an untouched line does not block" "$out" "nope.md"
rm -rf "$dir"

# 25. ...but a link the change itself adds still does.
dir=$(make_fixture)
printf '# docs\n\nSee [missing](./nope.md).\n' >"$dir/agent_docs/notes.md"
git -C "$dir" commit -qam "pre-existing broken link"
printf '\nSee [also-missing](./gone.md).\n' >>"$dir/agent_docs/notes.md"
out=$(run_hook "$dir")
expect_contains "a newly added broken link still blocks" "$out" "gone.md"
rm -rf "$dir"

# 26. A deleted doc must not crash the validator or block on its absence.
dir=$(make_fixture)
git -C "$dir" rm -q "agent_docs/notes.md"
out=$(run_hook "$dir")
expect_not_contains "a deleted doc is not an error" "$out" "Traceback"
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

printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ]
