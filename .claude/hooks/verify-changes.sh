#!/bin/bash
# Stop hook: check doc/metadata co-updates + run tests on changed packages
# Success = silent. Failures block Claude with actionable feedback.
set -euo pipefail

INPUT=$(cat)

# Prevent infinite loop: only fire once per stop cycle
if [ "$(echo "$INPUT" | jq -r '.stop_hook_active')" = "true" ]; then
  exit 0
fi

PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(pwd)}"
cd "$PROJECT_DIR" 2>/dev/null || exit 0

# Get changed files (staged + unstaged vs HEAD)
CHANGED=$(git diff --name-only HEAD 2>/dev/null || true)
[ -z "$CHANGED" ] && exit 0

# Filter to source files only
CHANGED_SRC=$(echo "$CHANGED" | grep -E '\.(cpp|hpp|h|cc|py)$' || true)
[ -z "$CHANGED_SRC" ] && exit 0

WARNINGS=""
CHANGED_PKGS=""

# --- Phase 1: Doc/metadata co-update check ---
for pkg_dir in $(echo "$CHANGED_SRC" | cut -d'/' -f1 | sort -u); do
  # Only check actual ROS2 packages
  [ -f "$pkg_dir/package.xml" ] || continue
  CHANGED_PKGS="${CHANGED_PKGS} ${pkg_dir}"

  # Check README.md co-update for source changes
  PKG_SRC=$(echo "$CHANGED" | grep "^${pkg_dir}/\(src\|include\)/" || true)
  if [ -n "$PKG_SRC" ]; then
    if ! echo "$CHANGED" | grep -q "^${pkg_dir}/README.md$"; then
      WARNINGS="${WARNINGS}  - ${pkg_dir}: source changed but README.md not updated\n"
    fi
  fi

  # Check for new .cpp files possibly missing from CMakeLists.txt
  NEW_SRC=$(git diff --diff-filter=A --name-only HEAD 2>/dev/null | grep "^${pkg_dir}/src/.*\.cpp$" | grep -v test || true)
  for f in $NEW_SRC; do
    bname=$(basename "$f")
    if ! grep -q "$bname" "${pkg_dir}/CMakeLists.txt" 2>/dev/null; then
      WARNINGS="${WARNINGS}  - ${pkg_dir}: new file ${bname} not found in CMakeLists.txt\n"
    fi
  done
done

# --- Phase 2: Run tests on changed packages ---
TEST_FAILURES=""
for pkg in $CHANGED_PKGS; do
  # Build first
  if ! ./build.sh -p "$pkg" >/dev/null 2>&1; then
    TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: build failed\n"
    continue
  fi

  # Run tests (timeout 60s per package)
  TEST_OUTPUT=$(timeout 60 bash -c "colcon test --packages-select $pkg --event-handlers console_direct+ 2>&1" || true)
  RESULT=$(colcon test-result --packages-select "$pkg" 2>&1 || true)

  if echo "$RESULT" | grep -qE "[1-9][0-9]* failures"; then
    FAILED_TESTS=$(echo "$RESULT" | grep -E "FAILED|failures" || true)
    TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: ${FAILED_TESTS}\n"
  fi
done

# --- Report ---
REPORT=""
if [ -n "$WARNINGS" ]; then
  REPORT="Doc/metadata co-update issues:\n${WARNINGS}\n"
fi
if [ -n "$TEST_FAILURES" ]; then
  REPORT="${REPORT}Test failures:\n${TEST_FAILURES}\n"
fi

if [ -n "$REPORT" ]; then
  echo -e "${REPORT}See agent_docs/modification-guide.md for the full checklist." >&2
  exit 2
fi

exit 0
