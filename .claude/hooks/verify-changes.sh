#!/bin/bash
# Stop hook — gate the turn end on doc/metadata co-updates + build/test.
#
# Intent : enforce CLAUDE.md §4 Workflow Loop steps 4·5·6 + PROC-1 (doc/code
#          sync) without trusting Claude's self-check (Anthropic 2026.04:
#          agent self-eval is unreliable).
# Trigger: every turn end. Reads {stop_hook_active} from stdin JSON; bails
#          early on re-entry to avoid infinite loops.
#
# Phases :
#   0. ARCH grep (architecture-fitness sensor)
#        - ARCH-1 : grep ur5e / hand-coded DOF in rtc_*/include|src
#        - ARCH-4 : grep ur5e_*/ including rtc_*/src/ private headers
#   1. Doc / metadata co-update
#        - README.md change required when src/ or include/ changed
#        - new .cpp must appear in CMakeLists.txt
#        - package.xml change required when find_package() added in CMakeLists.txt
#   2. Build + test on changed packages
#        - rtc_base / rtc_msgs change -> ./build.sh full + colcon test all
#          (PROC-3: broad downstream impact)
#        - else                       -> ./build.sh -p <pkg> + colcon test <pkg>
#
# Exit   : 0 on pass (silent). 2 on any failure -> Claude is blocked, stderr
#          message is auto-injected next turn. Pointer to modification-guide.md
#          is appended so the agent has a recovery entry point.
# Limits : 60s timeout per package (silent on overrun -- large packages may
#          partial-pass). YAML config / Doxygen / cross-package docs NOT
#          checked (modification-guide.md "Updating an Existing Package" 6
#          steps cover these manually). Only checks files vs HEAD: staged or
#          unstaged.
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
ARCH_VIOLATIONS=""
CHANGED_PKGS=""

# Identify changed packages (those with package.xml at root)
for pkg_dir in $(echo "$CHANGED_SRC" | cut -d'/' -f1 | sort -u); do
  [ -f "$pkg_dir/package.xml" ] || continue
  CHANGED_PKGS="${CHANGED_PKGS} ${pkg_dir}"
done

# --- Phase 0: Architecture-fitness grep (ARCH-1, ARCH-4) ---
# Only run if a CHANGED rtc_* or ur5e_* file matched, to bound cost.
RTC_TOUCHED=$(echo "$CHANGED_SRC" | grep -E '^rtc_[a-z_]+/' || true)
UR5E_TOUCHED=$(echo "$CHANGED_SRC" | grep -E '^ur5e_[a-z_]+/' || true)

if [ -n "$RTC_TOUCHED" ]; then
  # ARCH-1: rtc_* must not hardcode robot identifier or fixed DOF
  # Restrict grep to changed files to keep noise low and surface NEW violations
  for f in $RTC_TOUCHED; do
    [ -f "$f" ] || continue
    HITS=$(grep -niE '\b(ur5e|6.?dof|10.?dof|num_joints[[:space:]]*=[[:space:]]*[0-9])' "$f" 2>/dev/null || true)
    if [ -n "$HITS" ]; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-1 (robot-specific in rtc_*): ${f}\n${HITS}\n"
    fi
  done
fi

if [ -n "$UR5E_TOUCHED" ]; then
  # ARCH-4: ur5e_* must not include rtc_*/src/ private headers
  for f in $UR5E_TOUCHED; do
    [ -f "$f" ] || continue
    HITS=$(grep -nE '#include[[:space:]]+"rtc_[a-z_]+/src/' "$f" 2>/dev/null || true)
    if [ -n "$HITS" ]; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-4 (ur5e_* includes rtc_*/src/ private header): ${f}\n${HITS}\n"
    fi
  done
fi

# --- Phase 1: Doc/metadata co-update check ---
for pkg_dir in $CHANGED_PKGS; do
  # README.md co-update for source changes
  PKG_SRC=$(echo "$CHANGED" | grep "^${pkg_dir}/\(src\|include\)/" || true)
  if [ -n "$PKG_SRC" ]; then
    if ! echo "$CHANGED" | grep -q "^${pkg_dir}/README.md$"; then
      WARNINGS="${WARNINGS}  - ${pkg_dir}: source changed but README.md not updated\n"
    fi
  fi

  # New .cpp files possibly missing from CMakeLists.txt
  NEW_SRC=$(git diff --diff-filter=A --name-only HEAD 2>/dev/null | grep "^${pkg_dir}/src/.*\.cpp$" | grep -v test || true)
  for f in $NEW_SRC; do
    bname=$(basename "$f")
    if ! grep -q "$bname" "${pkg_dir}/CMakeLists.txt" 2>/dev/null; then
      WARNINGS="${WARNINGS}  - ${pkg_dir}: new file ${bname} not found in CMakeLists.txt\n"
    fi
  done

  # package.xml co-update for new find_package() in CMakeLists.txt
  if echo "$CHANGED" | grep -q "^${pkg_dir}/CMakeLists.txt$"; then
    NEW_FIND=$(git diff HEAD -- "${pkg_dir}/CMakeLists.txt" 2>/dev/null \
                | grep -E '^\+[[:space:]]*find_package\(' \
                | sed -E 's/^\+[[:space:]]*find_package\([[:space:]]*([A-Za-z0-9_]+).*/\1/' \
                | grep -vE '^(ament_cmake|ament_lint_auto|ament_cmake_gtest|ament_cmake_pytest|GTest)$' \
                || true)
    for dep in $NEW_FIND; do
      if [ -f "${pkg_dir}/package.xml" ]; then
        if ! grep -qE "<(build_depend|exec_depend|depend|test_depend)>${dep}<" "${pkg_dir}/package.xml" 2>/dev/null; then
          # Only warn if package.xml itself was NOT changed -- agent may have already added it
          if ! echo "$CHANGED" | grep -q "^${pkg_dir}/package.xml$"; then
            WARNINGS="${WARNINGS}  - ${pkg_dir}: find_package(${dep}) added in CMakeLists.txt but package.xml has no matching <depend>\n"
          fi
        fi
      fi
    done
  fi
done

# --- Phase 2: Build + test, with PROC-3 fallback for rtc_base / rtc_msgs ---
TEST_FAILURES=""
PROC3=$(echo "$CHANGED_PKGS" | tr ' ' '\n' | grep -E '^(rtc_base|rtc_msgs)$' || true)

if [ -n "$PROC3" ]; then
  # PROC-3: broad rebuild + full test (60s * count would still time out, so use
  # a generous bound on the build and a per-package test timeout).
  if ! timeout 300 ./build.sh full >/dev/null 2>&1; then
    TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad build (./build.sh full) failed (rtc_base / rtc_msgs touched)\n"
  else
    TEST_OUTPUT=$(timeout 180 bash -c "colcon test --event-handlers console_direct+ 2>&1" || true)
    RESULT=$(colcon test-result 2>&1 || true)
    if echo "$RESULT" | grep -qE "[1-9][0-9]* failures"; then
      FAILED_TESTS=$(echo "$RESULT" | grep -E "FAILED|failures" | head -20 || true)
      TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad test failed:\n${FAILED_TESTS}\n"
    fi
  fi
else
  for pkg in $CHANGED_PKGS; do
    if ! ./build.sh -p "$pkg" >/dev/null 2>&1; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: build failed\n"
      continue
    fi

    TEST_OUTPUT=$(timeout 60 bash -c "colcon test --packages-select $pkg --event-handlers console_direct+ 2>&1" || true)
    RESULT=$(colcon test-result --packages-select "$pkg" 2>&1 || true)

    if echo "$RESULT" | grep -qE "[1-9][0-9]* failures"; then
      FAILED_TESTS=$(echo "$RESULT" | grep -E "FAILED|failures" || true)
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: ${FAILED_TESTS}\n"
    fi
  done
fi

# --- Report ---
REPORT=""
if [ -n "$ARCH_VIOLATIONS" ]; then
  REPORT="Architecture-fitness violations (agent_docs/invariants.md):\n${ARCH_VIOLATIONS}\n"
fi
if [ -n "$WARNINGS" ]; then
  REPORT="${REPORT}Doc/metadata co-update issues:\n${WARNINGS}\n"
fi
if [ -n "$TEST_FAILURES" ]; then
  REPORT="${REPORT}Test/build failures:\n${TEST_FAILURES}\n"
fi

if [ -n "$REPORT" ]; then
  echo -e "${REPORT}See agent_docs/modification-guide.md for the full checklist." >&2
  exit 2
fi

exit 0
