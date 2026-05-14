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
#        - ARCH-1 : grep robot-name / hand-coded DOF in rtc_*/include|src,
#                   with negation-aware filter (lines containing "must NOT",
#                   "forbidden", "robot-agnostic", "no <X>-specific" are
#                   dropped — they encode the rule, not a violation).
#        - ARCH-4 : grep rtc_*/src/ private header includes inside the
#                   integration package set (auto-derived from package.xml
#                   files that <depend> on any rtc_*).
#   1. Doc / metadata co-update
#        - README.md change required when src/ or include/ changed
#        - new .cpp must appear in CMakeLists.txt
#        - package.xml change required when find_package() added in CMakeLists.txt
#   2. Build + test on changed packages
#        - rtc_base / rtc_msgs change -> ./build.sh full + colcon test all
#          (PROC-3: broad downstream impact)
#        - else                       -> ./build.sh -p <pkg> + colcon test <pkg>
#   3. Stale install/ detection (rename-aware)
#        - any deleted launch/*.py or config/**/*.yaml whose basename still
#          resolves under install/ — warns about stray artefacts that
#          colcon --symlink-install does not prune.
#
# Pure-format fast path:
#   Phases 0 + 1 are SKIPPED when every changed source file is identical to
#   HEAD after running it through the project's formatter (clang-format for
#   C++, ruff for Python). Such commits carry no semantic delta — ARCH-1
#   greps would flag pre-existing references that happen to be on a line
#   clang-format reflowed, and README co-update would be noise (nothing new
#   to document). Phase 2 (build/test) still runs because formatter changes
#   like include reordering can break compilation.
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

# --- Pure-format fast path detection ---
# Returns 0 if every changed source file is identical to HEAD after
# round-tripping through its formatter. We also skip if any file is brand-new
# or deleted (no HEAD blob to compare against), or if the formatter binary is
# missing (cannot prove pure-format -> fail closed and run full checks).
#
# ruff binary lookup mirrors .claude/hooks/format-code.sh: prefer venv,
# fall back to PATH.
find_ruff() {
  if [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/ruff" ]]; then
    echo "${VIRTUAL_ENV}/bin/ruff"; return 0
  fi
  local script_dir
  script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
  local ws_venv="${script_dir}/../../../../.venv/bin/ruff"
  [[ -x "$ws_venv" ]] && { echo "$ws_venv"; return 0; }
  command -v ruff 2>/dev/null && return 0
  return 1
}

is_pure_format() {
  command -v clang-format >/dev/null 2>&1 || return 1
  local RUFF_BIN
  RUFF_BIN=$(find_ruff) || RUFF_BIN=""

  # Reject any file add/delete/rename — only modifications can be pure-format.
  if git diff --diff-filter=ADRC --name-only HEAD 2>/dev/null \
       | grep -qE '\.(cpp|hpp|h|cc|py)$'; then
    return 1
  fi

  local f
  for f in $CHANGED_SRC; do
    [ -f "$f" ] || return 1
    case "$f" in
      *.cpp|*.hpp|*.h|*.cc)
        # Round-trip both versions through clang-format with $f as the
        # filename hint so .clang-format / file-type rules apply.
        diff <(git show "HEAD:$f" 2>/dev/null \
                 | clang-format --assume-filename="$f" 2>/dev/null) \
             <(clang-format --assume-filename="$f" < "$f" 2>/dev/null) \
             >/dev/null 2>&1 || return 1
        ;;
      *.py)
        [ -n "$RUFF_BIN" ] || return 1
        diff <(git show "HEAD:$f" 2>/dev/null \
                 | "$RUFF_BIN" format --stdin-filename="$f" - 2>/dev/null) \
             <("$RUFF_BIN" format --stdin-filename="$f" - < "$f" 2>/dev/null) \
             >/dev/null 2>&1 || return 1
        ;;
      *) return 1 ;;
    esac
  done
  return 0
}

PURE_FORMAT=0
if is_pure_format; then
  PURE_FORMAT=1
elif ! command -v clang-format >/dev/null 2>&1; then
  # Diagnostic: formatter absence forces fail-closed; surface once so
  # debugging stale-cache style fast-path misses isn't blind.
  echo "verify-changes: clang-format not found; pure-format fast path disabled." >&2
fi

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
# Skipped on pure-format commits — no semantic change can introduce a new
# ARCH violation, and the grep would re-flag pre-existing references on
# any line clang-format happened to reflow.
RTC_TOUCHED=""
INTEGRATION_TOUCHED=""
if [ "$PURE_FORMAT" -eq 0 ]; then
  RTC_TOUCHED=$(echo "$CHANGED_SRC" | grep -E '^rtc_[a-z_]+/' || true)
  # ARCH-4 target set is derived dynamically: any non-rtc_* package whose
  # package.xml depends on at least one rtc_* package. Previously this was
  # a hardcoded "^ur5e_*/" prefix which silently lost coverage after the
  # ur5e_bringup → integrated_bringup / ur5e_hand_driver → udp_hand_driver
  # renames (memory project_workspace_repackaging, 2026-05-03).
  INTEGRATION_PKGS=""
  for px in */package.xml; do
    [ -f "$px" ] || continue
    pkg=$(dirname "$px")
    case "$pkg" in
      rtc_*) continue ;;
    esac
    if grep -qE '<(build_depend|exec_depend|depend|test_depend)>rtc_[a-z_]+<' "$px" 2>/dev/null; then
      INTEGRATION_PKGS="${INTEGRATION_PKGS} ${pkg}"
    fi
  done
  for pkg in $INTEGRATION_PKGS; do
    MATCHES=$(echo "$CHANGED_SRC" | grep -E "^${pkg}/" || true)
    if [ -n "$MATCHES" ]; then
      INTEGRATION_TOUCHED="${INTEGRATION_TOUCHED}${MATCHES}
"
    fi
  done
fi

if [ -n "$RTC_TOUCHED" ]; then
  # ARCH-1: rtc_* must not hardcode robot identifier or fixed DOF.
  # Restrict grep to changed files to keep noise low and surface NEW violations.
  # Negation filter: lines that *forbid* the term (header comments like
  # "must NOT test UR5e", "no ur5e-specific code", "robot-agnostic") are
  # the rule itself, not a violation. Without this filter the hook punished
  # well-intentioned prohibitive docstrings — see memory
  # feedback_arch1_grep_false_positive (2026-05-07).
  for f in $RTC_TOUCHED; do
    [ -f "$f" ] || continue
    HITS=$(grep -niE '\b(ur5e|iiwa7|leap|allegro|6.?dof|10.?dof|num_joints[[:space:]]*=[[:space:]]*[0-9])' "$f" 2>/dev/null \
            | grep -viE '(must[[:space:]]*not|forbidden|robot-agnostic|no[[:space:]]+[a-z0-9_.-]+-specific|NOT[[:space:]]+(test|use|hardcode|include|reference))' \
            || true)
    if [ -n "$HITS" ]; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-1 (robot-specific in rtc_*): ${f}\n${HITS}\n"
    fi
  done
fi

if [ -n "$INTEGRATION_TOUCHED" ]; then
  # ARCH-4: integration packages must not include rtc_*/src/ private headers
  for f in $INTEGRATION_TOUCHED; do
    [ -f "$f" ] || continue
    HITS=$(grep -nE '#include[[:space:]]+"rtc_[a-z_]+/src/' "$f" 2>/dev/null || true)
    if [ -n "$HITS" ]; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-4 (integration pkg includes rtc_*/src/ private header): ${f}\n${HITS}\n"
    fi
  done
fi

# --- Phase 1: Doc/metadata co-update check ---
# Skipped on pure-format commits — there is no semantic delta to mirror in
# READMEs, and CMake/package.xml co-update triggers (new .cpp file, new
# find_package) cannot fire because pure-format excludes file adds.
if [ "$PURE_FORMAT" -eq 0 ]; then
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
fi  # PURE_FORMAT guard for Phase 1

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

# --- Phase 3: Stale install/ detection (rename-aware) ---
# colcon build --symlink-install does NOT prune deleted files from install/,
# so a rename can leave the old launch/config file resolvable by ros2 launch
# (memory project_iiwa7_leap_bringup, 2026-05-14). Only warns — full rebuild
# (./build.sh -c) is too destructive to invoke automatically and is forbidden
# when external packages share the tree (memory project_local_deps_state).
# Skipped on pure-format (no deletes possible) and when install/ is absent.
STALE_INSTALL=""
INSTALL_ROOTS=""
if [ "$PURE_FORMAT" -eq 0 ]; then
  # Probe likely install/ locations: workspace root sibling of repo, plus repo-local
  for cand in "../../install" "../../../install" "./install"; do
    [ -d "$cand" ] && INSTALL_ROOTS="${INSTALL_ROOTS} ${cand}"
  done
  if [ -n "$INSTALL_ROOTS" ]; then
    DELETED=$(git diff --diff-filter=D --name-only HEAD 2>/dev/null \
                | grep -E '(^|/)(launch/[^/]+\.(py|xml|yaml)$|config/.*\.(yaml|yml)$)' \
                || true)
    for path in $DELETED; do
      bname=$(basename "$path")
      for root in $INSTALL_ROOTS; do
        FOUND=$(find "$root" -name "$bname" -type f 2>/dev/null | head -3 || true)
        if [ -n "$FOUND" ]; then
          STALE_INSTALL="${STALE_INSTALL}  - ${path} deleted from src but still in install/:\n$(echo "$FOUND" | sed 's/^/      /')\n"
          break
        fi
      done
    done
  fi
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
if [ -n "$STALE_INSTALL" ]; then
  REPORT="${REPORT}Stale install/ artefacts (rename without prune — manual rm required):\n${STALE_INSTALL}\n"
fi

if [ -n "$REPORT" ]; then
  echo -e "${REPORT}See agent_docs/modification-guide.md for the full checklist." >&2
  exit 2
fi

exit 0
