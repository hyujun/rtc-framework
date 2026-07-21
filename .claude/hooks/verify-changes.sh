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
#        - ARCH-1 : grep robot-name / `num_joints=<literal>` in rtc_*/include|src,
#                   with negation-aware filter (lines containing "must NOT",
#                   "forbidden", "robot-agnostic", "no <X>-specific" are
#                   dropped — they encode the rule, not a violation). "N-DOF"
#                   string alternatives were dropped: "6-DOF" is SE(3) math,
#                   not a joint count (see the Phase 0 inline note).
#        - ARCH-4 : grep rtc_*/src/ private header includes inside the
#                   integration package set (auto-derived from package.xml
#                   files that <depend> on any rtc_*).
#   1. Doc / metadata co-update
#        - README.md: NON-BLOCKING checklist reminder, and only when the change
#          touches public surface (include/ header, launch/, config/, a source
#          file add/delete, or package.xml). src/-only internal refactors / bug
#          fixes do NOT trigger it -- they carry no doc-visible delta and the
#          old "any src change requires README" gate over-blocked them.
#        - new .cpp must appear in CMakeLists.txt        (blocking)
#        - package.xml change required when find_package() added (blocking)
#   2. Build + test on changed packages
#        - rtc_base / rtc_msgs change -> ./build.sh full + colcon test all
#          (PROC-3: broad downstream impact)
#        - else                       -> ./build.sh -p <pkg> + colcon test <pkg>
#        A test run that TIMES OUT or fails to launch is reported as UNVERIFIED
#        and blocks (exit 2) -- the colcon test exit code is preserved and
#        handled explicitly rather than inferred from test-result alone, so a
#        killed/partial run can no longer masquerade as "0 failures".
#   3. Stale install/ detection (rename-aware)
#        - any deleted launch/*.py or config/**/*.yaml whose basename still
#          resolves under install/ — warns about stray artefacts that
#          colcon --symlink-install does not prune.
#   4. shellcheck on changed *.sh (repo_scripts/**, build.sh, install.sh)
#        - runs at --severity=warning (notes do not block); repo-root
#          .shellcheckrc supplies external-sources + SC2034 suppression.
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
# Exit   : 0 on pass (a non-blocking doc checklist may still print to stderr).
#          2 on any hard failure -> Claude is blocked, stderr message is
#          auto-injected next turn. Pointer to modification-guide.md is appended
#          so the agent has a recovery entry point.
#          Loop safety: re-entry is gated by {stop_hook_active} (read from stdin
#          below) -- the hook fires once per stop cycle, so it cannot wedge the
#          turn in an infinite block. Official Stop-hook exit-2 semantics:
#          "Prevents Claude from stopping, continues the conversation"
#          (code.claude.com/docs/en/hooks). The agent must act on the injected
#          report; do not rely on any undocumented consecutive-block cap.
# Limits : per-package bounds: 180s build + 60s test. PROC-3 path: 300s build +
#          180s test. A build OR test that hits its timeout (exit 124) or fails
#          to launch (exit >=125) is reported as a failure/unverified and blocks
#          -- overrun is no longer silent. If a package's tests legitimately
#          exceed the bound, raise it here rather than letting the gate pass
#          blind. All within the Stop hook's 540s budget (settings.json).
#          YAML config / Doxygen / cross-package docs NOT checked
#          (modification-guide.md "Updating an Existing Package" 6 steps cover
#          these manually). Only checks files vs HEAD: staged or unstaged.
set -euo pipefail

INPUT=$(cat)

# Prevent infinite loop: only fire once per stop cycle
if [ "$(echo "$INPUT" | jq -r '.stop_hook_active')" = "true" ]; then
  exit 0
fi

PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(pwd)}"
cd "$PROJECT_DIR" 2>/dev/null || exit 0

# Resolve colcon workspace root (sibling-of-src) and source setup_env.sh so
# bare `colcon test` / `colcon test-result` calls below find ROS, deps, venv,
# and COLCON_DEFAULTS_FILE — without depending on the parent shell having
# pre-sourced setup_env.sh. RTC_DEPS_PREFIX guard mirrors bootstrap.sh so a
# re-source is a no-op when env is already set.
WORKSPACE="$(cd "$PROJECT_DIR/../.." 2>/dev/null && pwd)"
SETUP_ENV="$PROJECT_DIR/repo_scripts/scripts/setup_env.sh"
if [ -z "${RTC_DEPS_PREFIX:-}" ] && [ -f "$SETUP_ENV" ]; then
  # ROS /opt/ros/*/setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES);
  # under this script's `set -u` that aborts the hook before any phase runs when
  # the parent shell has not pre-sourced setup_env. Relax -u only around the
  # source (build_deps.sh sources ROS the same way, sans -u). -e stays on —
  # ROS setup.bash is -e-clean.
  set +u
  # shellcheck source=/dev/null
  source "$SETUP_ENV"
  set -u
fi

# Get changed files: tracked (staged + unstaged vs HEAD) UNION untracked.
#
# `git diff --name-only HEAD` cannot see untracked files by definition, and an
# agent that writes a new .cpp without `git add` is the normal case, not the
# exception. That blind spot skipped the ENTIRE gate -- not just the "new .cpp
# missing from CMakeLists" check, but build and test as well.
#
# The union is required; `git ls-files -mo --exclude-standard` alone is NOT a
# drop-in replacement. `-m` means "modified relative to the index", so a file
# that was `git add`-ed and then left alone is absent from its output. Swapping
# to it would trade the untracked blind spot for a staged one. Measured:
#   staged file, worktree == index
#     git diff --name-only HEAD           -> listed
#     git ls-files -mo --exclude-standard -> MISSING
CHANGED=$( { git diff --name-only HEAD 2>/dev/null || true; \
             git ls-files -o --exclude-standard 2>/dev/null || true; } \
           | sort -u | grep -v '^$' || true)
[ -z "$CHANGED" ] && exit 0

# Route by file class. Previously this filtered to source/shell and bailed if
# both were empty, which meant docs-only, YAML-only and CMake/package.xml-only
# changes bypassed every check below. The CMake case was the sharpest: the
# blocking co-update checks (new .cpp absent from CMakeLists, find_package
# without a matching <depend>) live AFTER that early exit, so the very change
# class that triggers them could never reach them.
CHANGED_SRC=$(echo "$CHANGED" | grep -E '\.(cpp|hpp|h|cc|py)$' || true)
CHANGED_SH=$(echo "$CHANGED" | grep -E '\.sh$' || true)
CHANGED_BUILD=$(echo "$CHANGED" | grep -E '(^|/)(CMakeLists\.txt|package\.xml)$' || true)
CHANGED_DOC=$(echo "$CHANGED" | grep -E '\.md$|^\.github/(workflows|actions)/' || true)
CHANGED_YAML=$(echo "$CHANGED" | grep -E '\.ya?ml$' | grep -vE '^\.github/' || true)
if [ -z "$CHANGED_SRC" ] && [ -z "$CHANGED_SH" ] && [ -z "$CHANGED_BUILD" ] \
   && [ -z "$CHANGED_DOC" ] && [ -z "$CHANGED_YAML" ]; then
  exit 0
fi

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

# Resolve a clang-format invocation: prefer a system binary, else the
# repo-pinned version via uvx (mirrors .claude/hooks/format-code.sh so the
# pure-format fast path works on boxes without a system clang-format — e.g.
# fresh dev/CI). Populates CLANG_FORMAT_CMD as an argv array; the pin matches
# format-code.sh so the round-trip and the auto-formatter agree.
CLANG_FORMAT_PIN="18.1.8"
CLANG_FORMAT_CMD=()
resolve_clang_format() {
  if command -v clang-format >/dev/null 2>&1; then
    CLANG_FORMAT_CMD=(clang-format)
    return 0
  fi
  if command -v uvx >/dev/null 2>&1; then
    CLANG_FORMAT_CMD=(uvx --from "clang-format==${CLANG_FORMAT_PIN}" clang-format)
    return 0
  fi
  return 1
}
resolve_clang_format && HAVE_CLANG_FORMAT=1 || HAVE_CLANG_FORMAT=0

is_pure_format() {
  [ "$HAVE_CLANG_FORMAT" -eq 1 ] || return 1
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
                 | "${CLANG_FORMAT_CMD[@]}" --assume-filename="$f" 2>/dev/null) \
             <("${CLANG_FORMAT_CMD[@]}" --assume-filename="$f" < "$f" 2>/dev/null) \
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
elif [ "$HAVE_CLANG_FORMAT" -eq 0 ]; then
  # Diagnostic: formatter absence forces fail-closed; surface once so
  # debugging stale-cache style fast-path misses isn't blind. Both a system
  # clang-format and the uvx fallback are missing here.
  echo "verify-changes: clang-format unavailable (no system binary or uvx);" \
       "pure-format fast path disabled." >&2
fi

WARNINGS=""       # blocking doc/metadata issues (CMake / package.xml)
CHECKLIST=""      # non-blocking reminders (README co-update) -- never exit 2 alone
ARCH_VIOLATIONS=""
CHANGED_PKGS=""

# Identify changed packages (those with package.xml at root).
# Build metadata counts as a package touch: a CMakeLists.txt / package.xml edit
# must reach the Phase 1 co-update checks and get rebuilt, even when no source
# file changed alongside it.
for pkg_dir in $(printf '%s\n%s\n' "$CHANGED_SRC" "$CHANGED_BUILD" \
                   | grep -v '^$' | cut -d'/' -f1 | sort -u); do
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
  # ARCH-1 scope is rtc_* production code (include|src|module dirs). Test files
  # under rtc_*/test|tests/ legitimately instantiate concrete robots — a
  # plotter test needs realistic fixtures like `leap_state.csv` — so excluding
  # them keeps the header's stated "include|src" intent without punishing test
  # fixtures (see memory feedback_arch1_grep_false_positive; test-fixture
  # false-positive observed 2026-07-01 on rtc_tools/test/test_plot_rtc_log.py).
  RTC_TOUCHED=$(echo "$CHANGED_SRC" | grep -E '^rtc_[a-z_]+/' | grep -vE '(^|/)tests?/' || true)
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
  #
  # Scope is the ADDED LINES of each changed file, not the whole file. Grepping
  # whole files reported pre-existing hits in regions the change never touched:
  # any edit to a file that already mentions a robot re-flagged those lines, and
  # renumbered them, so they read as new. Observed 2026-07-16 on
  # rtc_controller_interface — a purely additive change re-surfaced four hits
  # that `git show HEAD:<file>` proved identical at HEAD. A file-scoped grep
  # cannot distinguish "you added this" from "this was already here", which is
  # the only question this gate asks. Added-line scope keeps a brand new rtc_*
  # file screened in full: every one of its lines reads as added.
  #
  # Pattern targets: robot identifiers (ur5e/iiwa7/leap/allegro) and a
  # `num_joints = <literal>` assignment — the forms a real hardcode takes. The
  # old `6.?dof`/`10.?dof` string alternatives were removed: "6-DOF" is
  # SE(3)/task-space dimensionality (6 = 3 translation + 3 rotation), not a
  # robot joint count, so they fired only on legitimate math/prose ("6-DoF
  # task-space", floating-base "first 6 DoF", the `control_6dof` flag, an
  # "all valid" robot-agnostic enumeration) and never on an actual hardcode —
  # real DOF hardcoding is a numeric literal in array/matrix sizing, which
  # carries no "dof" substring. The one true signal that used `10-DoF`
  # (rtc_mpc capacity constants "for UR5e + 10-DoF hand") still matches via its
  # robot name (memory feedback_arch1_grep_whole_file_scope, 2026-07-17).
  #
  # Negation filter: lines that *forbid* the term (header comments like
  # "must NOT test UR5e", "no ur5e-specific code", "robot-agnostic") are
  # the rule itself, not a violation. Without this filter the hook punished
  # well-intentioned prohibitive docstrings (2026-05-07).
  for f in $RTC_TOUCHED; do
    [ -f "$f" ] || continue
    # Real file line numbers of added lines, from the `+c,d` side of each hunk
    # header. Kept as line numbers rather than grepping the raw '+' text so the
    # report still points at a location the agent can open.
    ADDED_LINES=$(git diff -U0 HEAD -- "$f" 2>/dev/null | awk '
      /^@@/ {
        match($0, /\+[0-9]+(,[0-9]+)?/)
        spec = substr($0, RSTART + 1, RLENGTH - 1)
        split(spec, p, ",")
        count = (p[2] == "" ? 1 : p[2])
        for (i = 0; i < count; i++) print p[1] + i
      }' || true)
    [ -z "$ADDED_LINES" ] && continue
    HITS=$(grep -niE '\b(ur5e|iiwa7|leap|allegro|num_joints[[:space:]]*=[[:space:]]*[0-9])' "$f" 2>/dev/null \
            | grep -viE '(must[[:space:]]*not|forbidden|robot-agnostic|no[[:space:]]+[a-z0-9_.-]+-specific|NOT[[:space:]]+(test|use|hardcode|include|reference))' \
            | awk -F: -v added="$ADDED_LINES" '
                BEGIN { n = split(added, a, "\n"); for (i = 1; i <= n; i++) if (a[i] != "") keep[a[i]] = 1 }
                ($1 in keep)' \
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

# ARCH-5: robot_descriptions is a data-only package -- consumers may declare an
# <exec_depend> and look it up through ament_index at runtime, but must never
# take a build-time dependency on it. Runs on changed build metadata only, which
# is why the routing above had to stop discarding CMakeLists/package.xml-only
# changes: this check is unreachable otherwise.
if [ -n "$CHANGED_BUILD" ]; then
  for f in $CHANGED_BUILD; do
    [ -f "$f" ] || continue
    # The data-only package itself is exempt: it cannot build-depend on itself,
    # and its CMakeLists carries a "DO NOT add find_package(robot_descriptions)"
    # comment that would otherwise make the rule flag its own documentation.
    case "$f" in robot_descriptions/*) continue ;; esac
    case "$f" in
      */CMakeLists.txt|CMakeLists.txt)
        # Strip comments before matching -- prose that names the forbidden call
        # (guidance, changelog notes) is not the forbidden call.
        HITS=$(sed 's/#.*//' "$f" 2>/dev/null \
                 | grep -nE 'find_package[[:space:]]*\([[:space:]]*robot_descriptions|ament_target_dependencies[^)]*robot_descriptions' \
                 || true)
        ;;
      */package.xml|package.xml)
        # <exec_depend> is the sanctioned form; <depend> and <build_depend> are not.
        HITS=$(grep -nE '<(depend|build_depend|buildtool_depend)>robot_descriptions<' "$f" 2>/dev/null || true)
        ;;
      *) HITS="" ;;
    esac
    if [ -n "$HITS" ]; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-5 (build-time dep on data-only robot_descriptions; use <exec_depend> + ament_index): ${f}\n${HITS}\n"
    fi
  done
fi

# --- Phase 0b: ARCH-6 topic QoS depth sensor (NON-BLOCKING) ---
# All ROS 2 topics must use KEEP_LAST depth 1 (invariants.md ARCH-6). Flag any
# changed production source (test fixtures exempt) that sets depth != 1:
#   rclcpp::QoS(N) / QoS{N} where N != 1, keep_last(N != 1), Python depth=N != 1,
#   and a bare SensorDataQoS() (default depth 5) not narrowed with keep_last(1).
# Non-blocking (rides the checklist) — the numeric patterns are precise but the
# bare-SensorDataQoS line filter can false-positive across a two-line construct,
# so this warns rather than exit-2s. Skipped on pure-format commits.
QOS_VIOLATIONS=""
if [ "$PURE_FORMAT" -eq 0 ]; then
  QOS_SRC=$(echo "$CHANGED_SRC" | grep -vE '(^|/)tests?/|(^|/)test_[^/]*\.py$|_test\.(cpp|cc|hpp|h)$' || true)
  for f in $QOS_SRC; do
    [ -f "$f" ] || continue
    # A line carrying the `ARCH-6-exempt` marker is a recorded exception
    # (invariants.md ARCH-6 세부 스펙 — e.g. accumulating sensor streams) and
    # is dropped from the sensor so it does not re-flag every time the file changes.
    HITS=$(grep -nE 'rclcpp::QoS[({]([0-9]{2,}|[02-9])[)}]|keep_last\(([0-9]{2,}|[02-9])\)|depth[[:space:]]*=[[:space:]]*([0-9]{2,}|[02-9])' "$f" 2>/dev/null | grep -v 'ARCH-6-exempt' || true)
    BARE=$(grep -nE 'SensorDataQoS\(\)' "$f" 2>/dev/null | grep -v 'keep_last' | grep -v 'ARCH-6-exempt' || true)
    ALL=$(printf '%s\n%s' "$HITS" "$BARE" | grep -vE '^[[:space:]]*$' || true)
    if [ -n "$ALL" ]; then
      QOS_VIOLATIONS="${QOS_VIOLATIONS}  - ARCH-6 (topic QoS depth != 1): ${f}\n${ALL}\n"
    fi
  done
fi

# --- Phase 0b: Agent doc corpus validation ---
# Docs previously had no verification path at all: this hook bailed before
# reaching anything, and ros2-advanced-ci.yml paths-ignores docs/, agent_docs/
# and root *.md. Broken links, banned line anchors and dead detection regexes
# accumulated unnoticed for exactly that reason (issue #213).
#
# Blocking, because the failure mode it guards is silence -- a detection pattern
# that no longer matches reads identically to a clean tree.
DOC_VIOLATIONS=""
if [ -n "$CHANGED_DOC" ] && [ -f repo_scripts/scripts/validate_docs.py ]; then
  if ! DOC_OUT=$(python3 repo_scripts/scripts/validate_docs.py 2>&1); then
    DOC_VIOLATIONS="${DOC_OUT}"
  fi
fi

# --- Phase 1: Doc/metadata co-update check ---
# Skipped on pure-format commits — there is no semantic delta to mirror in
# READMEs, and CMake/package.xml co-update triggers (new .cpp file, new
# find_package) cannot fire because pure-format excludes file adds.
if [ "$PURE_FORMAT" -eq 0 ]; then
for pkg_dir in $CHANGED_PKGS; do
  # README.md co-update -- NON-BLOCKING checklist, and only for public-surface
  # changes. A src/-only edit (internal refactor, bug fix, private-impl change)
  # carries no doc-visible delta, so requiring a README bump there was pure
  # over-blocking. We flag only when the change plausibly alters documented
  # behavior/usage: a public header (include/), launch/ or config/, a source
  # file add/delete (structural), or package.xml (deps/exec surface).
  PKG_PUBLIC=$(echo "$CHANGED" | grep -E "^${pkg_dir}/(include|launch|config)/" || true)
  PKG_STRUCT=$(git diff --diff-filter=AD --name-only HEAD 2>/dev/null \
                 | grep -E "^${pkg_dir}/(src|include)/.*\.(cpp|hpp|h|cc|py)$" || true)
  PKG_PKGXML=$(echo "$CHANGED" | grep -E "^${pkg_dir}/package.xml$" || true)
  if [ -n "$PKG_PUBLIC" ] || [ -n "$PKG_STRUCT" ] || [ -n "$PKG_PKGXML" ]; then
    if ! echo "$CHANGED" | grep -q "^${pkg_dir}/README.md$"; then
      CHECKLIST="${CHECKLIST}  - ${pkg_dir}: public surface changed (header / launch / config / file add-del / dep) — confirm README.md reflects it, or note in your report why no doc change is needed\n"
    fi
  fi

  # New .cpp files possibly missing from CMakeLists.txt.
  # `--diff-filter=A` only reports STAGED adds, so an agent's brand-new,
  # never-added .cpp was invisible here -- the exact case this check exists for.
  # Union in untracked files for the same reason as $CHANGED above.
  NEW_SRC=$( { git diff --diff-filter=A --name-only HEAD 2>/dev/null || true; \
               git ls-files -o --exclude-standard 2>/dev/null || true; } \
             | sort -u | grep "^${pkg_dir}/src/.*\.cpp$" | grep -v test || true)
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
  # All colcon invocations run from $WORKSPACE so build/install/log land in the
  # colcon ws root (CLAUDE.md §9.1), not in this repo's cwd.
  if ! timeout 300 ./build.sh full >/dev/null 2>&1; then
    TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad build (./build.sh full) failed (rtc_base / rtc_msgs touched)\n"
  else
    # Preserve `colcon test`'s exit code: 124 = timed out, >=125 = could not
    # launch (env/build), 0 or 1 = ran (pass/fail read from test-result). Never
    # swallow it with `|| true`, or a killed run reads as "0 failures".
    TEST_RC=0
    timeout 180 bash -c "cd '$WORKSPACE' && colcon test --event-handlers console_direct+ 2>&1" >/dev/null || TEST_RC=$?
    RESULT=$(cd "$WORKSPACE" && colcon test-result 2>&1 || true)
    if [ "$TEST_RC" -eq 124 ]; then
      TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad test TIMED OUT after 180s — UNVERIFIED, treat as failure (raise the bound in verify-changes.sh or run 'colcon test' manually)\n"
    elif [ "$TEST_RC" -ge 125 ]; then
      TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad 'colcon test' could not launch (exit ${TEST_RC}; env/build issue) — UNVERIFIED\n"
    elif echo "$RESULT" | grep -qE "[1-9][0-9]* (error|failure)s?"; then
      FAILED_TESTS=$(echo "$RESULT" | grep -iE "FAILED|error|failure" | head -20 || true)
      TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad test failed:\n${FAILED_TESTS}\n"
    elif [ "$TEST_RC" -ne 0 ]; then
      TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad 'colcon test' exited ${TEST_RC} with no parseable result summary — UNVERIFIED\n"
    fi
  fi
else
  for pkg in $CHANGED_PKGS; do
    # 180s build bound mirrors the PROC-3 path's `timeout 300 ./build.sh full`.
    # Without it a slow/hung single-package build is unbounded, so N changed
    # packages can blow past the Stop hook's 540s budget (settings.json) and get
    # SIGKILLed mid-build -> the turn ends with the gate silently skipped.
    # A timeout-killed build (exit 124) is reported as a build failure -> exit 2.
    if ! timeout 180 ./build.sh -p "$pkg" >/dev/null 2>&1; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: build failed\n"
      continue
    fi

    # Preserve the exit code (see PROC-3 path above): distinguish timeout /
    # launch failure / real test failure instead of inferring from test-result.
    TEST_RC=0
    timeout 60 bash -c "cd '$WORKSPACE' && colcon test --packages-select $pkg --event-handlers console_direct+ 2>&1" >/dev/null || TEST_RC=$?
    RESULT=$(cd "$WORKSPACE" && colcon test-result --packages-select "$pkg" 2>&1 || true)

    if [ "$TEST_RC" -eq 124 ]; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: colcon test TIMED OUT after 60s — UNVERIFIED, treat as failure (raise the bound in verify-changes.sh or run 'colcon test' manually)\n"
    elif [ "$TEST_RC" -ge 125 ]; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: colcon test could not launch (exit ${TEST_RC}; env/build issue) — UNVERIFIED\n"
    elif echo "$RESULT" | grep -qE "[1-9][0-9]* (error|failure)s?"; then
      FAILED_TESTS=$(echo "$RESULT" | grep -iE "FAILED|error|failure" || true)
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: ${FAILED_TESTS}\n"
    elif [ "$TEST_RC" -ne 0 ]; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: colcon test exited ${TEST_RC} with no parseable result summary — UNVERIFIED\n"
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

# --- Phase 4: shellcheck on changed shell scripts ---
# Lint gate for repo_scripts/**/*.sh + build.sh / install.sh. Runs at
# --severity=warning so info-level notes (SC1091 source-following, intentional
# SC2086 word-splitting in RT cpu-list code) do not block. The repo-root
# .shellcheckrc (external-sources=true, disable=SC2034) is auto-loaded from
# PROJECT_DIR cwd. Fail-open when shellcheck is absent — missing tooling must
# not hard-block a turn (mirrors the clang-format fail-open above).
SHELLCHECK_FAILURES=""
if [ -n "$CHANGED_SH" ]; then
  if command -v shellcheck >/dev/null 2>&1; then
    for f in $CHANGED_SH; do
      [ -f "$f" ] || continue
      SC_OUT=$(shellcheck --severity=warning -f gcc "$f" 2>/dev/null || true)
      [ -n "$SC_OUT" ] && SHELLCHECK_FAILURES="${SHELLCHECK_FAILURES}${SC_OUT}\n"
    done
  else
    echo "verify-changes: shellcheck not found; shell-script lint gate skipped." >&2
  fi
fi

# --- Report ---
REPORT=""
if [ -n "$ARCH_VIOLATIONS" ]; then
  REPORT="Architecture-fitness violations (agent_docs/invariants.md):\n${ARCH_VIOLATIONS}\n"
fi
if [ -n "$DOC_VIOLATIONS" ]; then
  REPORT="${REPORT}Agent doc corpus validation failed (repo_scripts/scripts/validate_docs.py):\n${DOC_VIOLATIONS}\n"
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
if [ -n "$SHELLCHECK_FAILURES" ]; then
  REPORT="${REPORT}shellcheck (warning+) on changed shell scripts:\n${SHELLCHECK_FAILURES}\n"
fi

# ARCH-6 QoS depth is a non-blocking sensor: fold it into the checklist stream
# so it surfaces alongside (never as) a hard failure.
if [ -n "$QOS_VIOLATIONS" ]; then
  CHECKLIST="${CHECKLIST}${QOS_VIOLATIONS}"
fi

if [ -n "$REPORT" ]; then
  # A hard failure is blocking. Ride the non-blocking checklist along so the
  # agent sees doc reminders while it is already addressing the real failure.
  if [ -n "$CHECKLIST" ]; then
    REPORT="${REPORT}Doc checklist (reminder — not itself blocking):\n${CHECKLIST}\n"
  fi
  echo -e "${REPORT}See agent_docs/modification-guide.md for the full checklist." >&2
  exit 2
fi

# No hard failure: surface the doc checklist as a non-blocking reminder and let
# the turn end. README co-update is a judgement call the agent/user makes, not a
# gate -- so it prints but never forces exit 2.
if [ -n "$CHECKLIST" ]; then
  echo -e "Doc checklist (non-blocking — turn NOT blocked):\n${CHECKLIST}\nSee agent_docs/modification-guide.md. If a public-surface change genuinely needs no README edit, note that in your report." >&2
fi

exit 0
