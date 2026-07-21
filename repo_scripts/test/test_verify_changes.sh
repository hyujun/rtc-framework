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
# under test.
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
dir=$(make_fixture)
printf 'int existing()  {  return 0;  }\n' >"$dir/rtc_demo/src/existing.cpp"
out=$(run_hook "$dir")
expect_not_contains "pure formatting alone raises no co-update warning" "$out" "README.md reflects it"
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

printf '\n%d passed, %d failed\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ]
