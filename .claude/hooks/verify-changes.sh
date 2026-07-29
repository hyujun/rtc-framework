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
#        - ARCH-5 : robot_descriptions must stay an <exec_depend> (added lines of
#                   CMakeLists / package.xml; multi-line ament_target_dependencies
#                   and <build_export_depend> included).
#        - ARCH-7 : rtc_* must not own a control-framework executable. Compares
#                   add_executable target NAMES against HEAD (not added lines --
#                   CMake lines get rewritten in place). `example_*` targets are
#                   out of scope by name; anything else opts out with an
#                   `ARCH-7-exempt` comment on or directly above the call.
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
#   1b. Docs + YAML sensors
#        - changed *.md -> validate_docs.py --files (CHANGED scope only; CI does
#          the full-corpus scan. A whole-repo scan here would let a defect in an
#          untouched -- or gitignored, hence invisible -- file block every turn).
#          Within a tracked doc, findings are further narrowed to ADDED lines:
#          whole-file scope made every pre-existing defect in a file blocking as
#          soon as the agent touched it for an unrelated reason.
#        - changed *.yaml -> parse check (config/** had no gate at all). Verdict
#          is the interpreter exit status, and a missing PyYAML fails OPEN.
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
#          Doxygen / cross-package doc consistency NOT checked
#          (modification-guide.md "Updating an Existing Package" 6 steps cover
#          these manually). Changed set = tracked-vs-HEAD UNION untracked;
#          build/test additionally requires an untracked file to live under
#          <pkg>/src/ or <pkg>/include/, so a new header is compiled while a
#          scratch file under rtc_base/ still cannot trigger a full-workspace
#          rebuild. Routing behaviour is asserted end-to-end by
#          repo_scripts/test/test_verify_changes.sh.
#          A path containing a newline is C-quoted by git regardless of
#          core.quotePath and is NOT handled; paths with spaces are.
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

# Changed files = tracked modifications vs HEAD, PLUS untracked new files.
#
# `git diff --name-only HEAD` alone cannot see a file that was never added,
# which is the normal state of one an agent just wrote -- so the "new .cpp
# missing from CMakeLists" gate could not fire on precisely the files it
# exists for. The tempting one-line swap to `git ls-files -mo` regresses the
# other way: `-m` is relative to the index, so a staged-then-untouched file
# drops out. The union covers both.
#
# core.quotePath=false keeps non-ASCII paths as UTF-8 instead of C-quoted
# escapes ("...\355\225\234.cpp"). A quoted path matches no extension filter,
# and a file matching no filter is silently exempt from every check below
# rather than loudly rejected.
CHANGED_TRACKED=$(git -c core.quotePath=false diff --name-only HEAD 2>/dev/null || true)
CHANGED_UNTRACKED=$(git -c core.quotePath=false ls-files -o --exclude-standard 2>/dev/null || true)
CHANGED=$(printf '%s\n%s\n' "$CHANGED_TRACKED" "$CHANGED_UNTRACKED" | grep -v '^[[:space:]]*$' | sort -u || true)
[ -z "$CHANGED" ] && exit 0

# Classify. Every class below has at least one check, so any of them keeps the
# turn in scope -- previously only source and shell did, which meant docs-only,
# YAML-only, CMake-only and package.xml-only changes skipped the hook whole.
# CMake-only was the sharpest case: the co-update gates for CMakeLists and
# package.xml live *after* the exit, so the change that triggers them was the
# change that never reached them.
CHANGED_SRC=$(echo "$CHANGED" | grep -E '\.(cpp|hpp|h|cc|py)$' || true)
CHANGED_SH=$(echo "$CHANGED" | grep -E '\.sh$' || true)
CHANGED_DOCS=$(echo "$CHANGED" | grep -E '\.md$' || true)
CHANGED_YAML=$(echo "$CHANGED" | grep -E '\.(yaml|yml)$' || true)
CHANGED_META=$(echo "$CHANGED" | grep -E '(^|/)(CMakeLists\.txt|package\.xml)$' || true)
if [ -z "$CHANGED_SRC" ] && [ -z "$CHANGED_SH" ] && [ -z "$CHANGED_DOCS" ] \
   && [ -z "$CHANGED_YAML" ] && [ -z "$CHANGED_META" ]; then
  exit 0
fi

# Build/test scope excludes untracked *scratch*, not untracked source.
#
# The first cut of this split keyed on tracked-ness, which is the wrong axis: it
# kept a scratch .py under rtc_base/ from triggering the PROC-3 full-workspace
# rebuild (the intent, and correct) but it also meant a brand-new
# rtc_base/include/**/foo.hpp was ARCH-grepped and CMake-gated and then never
# compiled. rtc_base is header-heavy and PROC-3's broad rebuild exists for
# exactly that file, so the hole sat on the highest-impact path.
#
# The axis that actually separates the two cases is *location*: real package
# source lives under <pkg>/src/ or <pkg>/include/, scratch does not. An
# untracked file there is part of the tree being verified even though it has no
# HEAD blob; an untracked note, probe script, or log anywhere else is not.
CHANGED_SRC_TRACKED=$(echo "$CHANGED_TRACKED" | grep -E '\.(cpp|hpp|h|cc|py)$' || true)
CHANGED_META_TRACKED=$(echo "$CHANGED_TRACKED" | grep -E '(^|/)(CMakeLists\.txt|package\.xml)$' || true)
# Untracked source that belongs to a package, so it is built rather than treated
# as scratch. Real source lives under <pkg>/src/ or <pkg>/include/ (ament_cmake)
# OR under <pkg>/<pkg>/ (ament_python packages -- rtc_tools, rtc_digital_twin --
# keep their modules in a same-named subdir, which the src|include-only regex
# missed, so a brand-new untracked module was ARCH-grepped but never built).
# A backreference is not portable across grep flavours, so match with awk.
CHANGED_SRC_UNTRACKED=$(echo "$CHANGED_UNTRACKED" | awk -F/ '
  NF >= 3 && $NF ~ /\.(cpp|hpp|h|cc|py)$/ && ($2 == "src" || $2 == "include" || $2 == $1)' \
  || true)
CHANGED_SRC_BUILD=$(printf '%s\n%s\n' "$CHANGED_SRC_TRACKED" "$CHANGED_SRC_UNTRACKED" \
  | grep -v '^[[:space:]]*$' | sort -u || true)

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

  # No source files at all is NOT "pure format". The loop below iterates over
  # $CHANGED_SRC and returns success on an empty list, so a CMake-only or
  # docs-only change used to be classified as cosmetic and skip Phase 1 --
  # silently cancelling the routing fix above. Same trap with formatting churn
  # alongside a CMake edit: the source half really is cosmetic, and the
  # unrelated CMake gate went down with it.
  [ -n "$CHANGED_SRC" ] || return 1
  if [ -n "$CHANGED_DOCS" ] || [ -n "$CHANGED_YAML" ] || [ -n "$CHANGED_META" ] \
     || [ -n "$CHANGED_SH" ]; then
    return 1
  fi
  # An untracked file has no HEAD blob to compare against.
  if echo "$CHANGED_UNTRACKED" | grep -qE '\.(cpp|hpp|h|cc|py)$'; then
    return 1
  fi

  # Reject any file add/delete/rename — only modifications can be pure-format.
  if git diff --diff-filter=ADRC --name-only HEAD 2>/dev/null \
       | grep -qE '\.(cpp|hpp|h|cc|py)$'; then
    return 1
  fi

  # Capture both formatted sides rather than diffing process substitutions.
  # A formatter that fails (uvx cannot provision the pinned clang-format on a
  # cold/offline cache, a transient error) emits NOTHING to stdout under
  # 2>/dev/null; `diff <(empty) <(empty)` then exits 0 and every modified file
  # reads as "cosmetic", silently skipping Phase 0/1 -- the exact fail-OPEN this
  # gate must not have. A real source file never formats to empty, so an empty
  # working-tree result means "cannot prove pure-format" -> fail CLOSED.
  local f head_fmt work_fmt
  for f in $CHANGED_SRC; do
    [ -f "$f" ] || return 1
    case "$f" in
      *.cpp|*.hpp|*.h|*.cc)
        # Round-trip both versions through clang-format with $f as the
        # filename hint so .clang-format / file-type rules apply.
        head_fmt=$(git show "HEAD:$f" 2>/dev/null \
                     | "${CLANG_FORMAT_CMD[@]}" --assume-filename="$f" 2>/dev/null)
        work_fmt=$("${CLANG_FORMAT_CMD[@]}" --assume-filename="$f" < "$f" 2>/dev/null)
        ;;
      *.py)
        [ -n "$RUFF_BIN" ] || return 1
        head_fmt=$(git show "HEAD:$f" 2>/dev/null \
                     | "$RUFF_BIN" format --stdin-filename="$f" - 2>/dev/null)
        work_fmt=$("$RUFF_BIN" format --stdin-filename="$f" - < "$f" 2>/dev/null)
        ;;
      *) return 1 ;;
    esac
    # Empty output == formatter failure == not provably pure-format.
    [ -n "$work_fmt" ] || return 1
    [ "$head_fmt" = "$work_fmt" ] || return 1
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

# Identify changed packages. Derived from ALL changed files, not just source:
# a package whose only edit is CMakeLists.txt, package.xml or config/*.yaml is
# still a changed package, and deriving this from $CHANGED_SRC was the third
# layer (after the early exit and the pure-format check) that kept CMake-only
# edits away from their own gate.
#
# Iterated with `while read` rather than `for x in $(...)`: a path containing a
# space is two words to the shell, and the resulting fragments matched no
# package while still reaching the per-file gates below as invented filenames.
while IFS= read -r pkg_dir; do
  [ -n "$pkg_dir" ] || continue
  [ -f "$pkg_dir/package.xml" ] || continue
  CHANGED_PKGS="${CHANGED_PKGS} ${pkg_dir}"
done <<< "$(echo "$CHANGED" | cut -d'/' -f1 | sort -u)"

# Build/test operates on tracked source plus untracked source that lives in a
# real package source dir (see CHANGED_SRC_BUILD).
BUILD_PKGS=""
while IFS= read -r pkg_dir; do
  [ -n "$pkg_dir" ] || continue
  [ -f "$pkg_dir/package.xml" ] || continue
  BUILD_PKGS="${BUILD_PKGS} ${pkg_dir}"
done <<< "$(printf '%s\n%s\n' "$CHANGED_SRC_BUILD" "$CHANGED_META_TRACKED" \
             | grep -v '^[[:space:]]*$' | cut -d'/' -f1 | sort -u)"

# Emit `name<TAB>exempt<TAB>lineno` for every add_executable() in the CMake
# source arriving on stdin. Used by ARCH-7 to diff target NAMES between HEAD and
# the working tree instead of diffing lines.
#
# `exempt` is 1 when the target opts out, which it may do two ways:
#   - the name matches example_*  — design-principles.md puts examples outside
#     the runtime-identity scope by name, so they need no per-target marker and
#     cannot drift out of one;
#   - an `ARCH-7-exempt` comment sits on the add_executable line OR the line
#     directly above it. Same-line-only matching (the first cut) rejected the
#     idiomatic CMake form, where the justification is a comment above the call,
#     while accepting a bare marker appended to the call itself.
exe_targets() {
  awk '
    { line[NR] = $0 }
    END {
      for (i = 1; i <= NR; i++) {
        if (line[i] !~ /^[[:space:]]*add_executable[[:space:]]*\(/) continue
        name = line[i]
        sub(/^[[:space:]]*add_executable[[:space:]]*\([[:space:]]*/, "", name)
        sub(/[^A-Za-z0-9_${}].*$/, "", name)
        if (name == "") continue
        exempt = 0
        if (name ~ /^example_/) exempt = 1
        if (line[i] ~ /ARCH-7-exempt/) exempt = 1
        if (i > 1 && line[i - 1] ~ /ARCH-7-exempt/) exempt = 1
        printf "%s\t%d\t%d\n", name, exempt, i
      }
    }'
}

# --- Phase 0: Architecture-fitness grep (ARCH-1, ARCH-4) ---
# Only run if a CHANGED rtc_* file or a changed file in the integration package
# set matched, to bound cost. (The scope was once a hardcoded `ur5e_*` prefix;
# INTEGRATION_PKGS below derives it from package.xml instead.)
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
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    [ -f "$f" ] || continue
    # Real file line numbers of added lines, from the `+c,d` side of each hunk
    # header. Kept as line numbers rather than grepping the raw '+' text so the
    # report still points at a location the agent can open.
    if git ls-files --error-unmatch "$f" >/dev/null 2>&1; then
      ADDED_LINES=$(git diff -U0 HEAD -- "$f" 2>/dev/null | awk '
        /^@@/ {
          match($0, /\+[0-9]+(,[0-9]+)?/)
          spec = substr($0, RSTART + 1, RLENGTH - 1)
          split(spec, p, ",")
          count = (p[2] == "" ? 1 : p[2])
          for (i = 0; i < count; i++) print p[1] + i
        }' || true)
    else
      # Untracked: every line is new, so the whole file is in scope.
      ADDED_LINES=$(awk '{ print NR }' "$f" 2>/dev/null || true)
    fi
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
  done <<< "$RTC_TOUCHED"
fi

if [ -n "$INTEGRATION_TOUCHED" ]; then
  # ARCH-4: integration packages must not include rtc_*/src/ private headers
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    [ -f "$f" ] || continue
    HITS=$(grep -nE '#include[[:space:]]+"rtc_[a-z_]+/src/' "$f" 2>/dev/null || true)
    if [ -n "$HITS" ]; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-4 (integration pkg includes rtc_*/src/ private header): ${f}\n${HITS}\n"
    fi
  done <<< "$INTEGRATION_TOUCHED"
fi

# --- Phase 0a: ARCH-5 / ARCH-7 build-metadata sensors ---
# ARCH-5: robot_descriptions is a data-only package -- consumers get it at
#   runtime via ament_index, never as a build dependency.
# ARCH-7: rtc_* packages do not own the control-framework runtime identity.
#   Robot-agnostic standalone nodes and examples are exempt (see
#   design-principles.md).
#
# ARCH-5 is scoped to ADDED lines, matching ARCH-1: whole-file scope re-reports
# pre-existing hits every time an unrelated edit touches the file, which is how
# a gate turns into noise and then gets ignored.
#
# ARCH-7 is NOT line-scoped, because for this question added-line scope is not
# an approximation of "new" -- it is a different question with a different
# answer. CMake target lines get rewritten in place: reindenting, renaming a
# source file, or wrapping a block in `if()` re-presents an existing
# add_executable as an added line. All seven add_executable targets in this
# repo's rtc_* packages predate the rule, so the line-scoped form hard-blocked
# any turn that so much as reformatted rtc_urdf_bridge/CMakeLists.txt. What the
# rule actually asks is whether a target NAME is new, so compare the set of
# target names at HEAD against the set in the working tree.
if [ "$PURE_FORMAT" -eq 0 ] && [ -n "$CHANGED_META" ]; then
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    [ -f "$f" ] || continue
    if git ls-files --error-unmatch "$f" >/dev/null 2>&1; then
      ADDED=$(git diff -U0 HEAD -- "$f" 2>/dev/null | grep '^+' | grep -v '^+++' || true)
    else
      ADDED=$(cat "$f" 2>/dev/null || true)
    fi
    [ -z "$ADDED" ] && continue

    # Comments are not code. Writing the rule down next to the call --
    # `# NOTE: robot_descriptions stays exec_depend only, never linked here` --
    # is the behaviour this gate wants to encourage, and the multi-line join
    # below happily matched it. Strip `#` (CMake) and `<!-- -->` (package.xml)
    # comments before the ARCH-5 patterns run.
    ADDED_CODE=$(echo "$ADDED" | sed -E 's/<!--.*-->//; s/#.*$//')

    # ARCH-5. `ament_target_dependencies(... )` is routinely spread over
    # several lines in this repo, so a line-scoped regex misses the common
    # form; check the added block as a whole for the package name in any
    # build-time position.
    if echo "$ADDED_CODE" | grep -qE 'find_package[[:space:]]*\([[:space:]]*robot_descriptions'; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-5 (robot_descriptions is data-only — find_package is a build-time dep): ${f}\n"
    fi
    if echo "$ADDED_CODE" | grep -qE '<(build_depend|depend|build_export_depend)>robot_descriptions</'; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-5 (robot_descriptions must be <exec_depend> only): ${f}\n"
    fi
    if echo "$ADDED_CODE" | tr '\n' ' ' \
         | grep -qE 'ament_target_dependencies[^)]*robot_descriptions'; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-5 (robot_descriptions linked as a build dep): ${f}\n"
    fi

    # ARCH-7: an executable target NAME that does not exist at HEAD.
    case "$f" in
      rtc_*/CMakeLists.txt)
        HEAD_EXE=$(git show "HEAD:$f" 2>/dev/null | exe_targets | cut -f1 || true)
        NEW_EXE=$(exe_targets < "$f" \
                    | awk -F'\t' -v head="$HEAD_EXE" '
                        BEGIN {
                          n = split(head, h, "\n")
                          for (i = 1; i <= n; i++) if (h[i] != "") seen[h[i]] = 1
                        }
                        !($1 in seen) && $2 == "0" { print "      " $1 " (line " $3 ")" }
                      ' || true)
        if [ -n "$NEW_EXE" ]; then
          ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-7 (rtc_* must not own a control-framework executable; agnostic nodes/examples opt out with an ARCH-7-exempt comment on or above the add_executable line): ${f}\n${NEW_EXE}\n"
        fi
        ;;
    esac
  done <<< "$CHANGED_META"
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
  while IFS= read -r f; do
    [ -n "$f" ] || continue
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
  done <<< "$QOS_SRC"
fi

# --- Phase 1: Doc/metadata co-update check ---
# Skipped on pure-format commits — there is no semantic delta to mirror in
# READMEs, and CMake/package.xml co-update triggers (new .cpp file, new
# find_package) cannot fire because pure-format excludes file adds.
if [ "$PURE_FORMAT" -eq 0 ]; then
# Hoisted out of the per-package loop: these two git queries are repo-wide and
# identical on every iteration, so computing them once and re-filtering by
# package below saves one `git diff` spawn per changed package each turn.
STRUCT_AD=$(git diff --diff-filter=AD --name-only HEAD 2>/dev/null || true)
ADDED_A=$(git diff --diff-filter=A --name-only HEAD 2>/dev/null || true)
for pkg_dir in $CHANGED_PKGS; do
  # README.md co-update -- NON-BLOCKING checklist, and only for public-surface
  # changes. A src/-only edit (internal refactor, bug fix, private-impl change)
  # carries no doc-visible delta, so requiring a README bump there was pure
  # over-blocking. We flag only when the change plausibly alters documented
  # behavior/usage: a public header (include/), launch/ or config/, a source
  # file add/delete (structural), or package.xml (deps/exec surface).
  PKG_PUBLIC=$(echo "$CHANGED" | grep -E "^${pkg_dir}/(include|launch|config)/" || true)
  PKG_STRUCT=$(echo "$STRUCT_AD" \
                 | grep -E "^${pkg_dir}/(src|include)/.*\.(cpp|hpp|h|cc|py)$" || true)
  PKG_PKGXML=$(echo "$CHANGED" | grep -E "^${pkg_dir}/package.xml$" || true)
  if [ -n "$PKG_PUBLIC" ] || [ -n "$PKG_STRUCT" ] || [ -n "$PKG_PKGXML" ]; then
    if ! echo "$CHANGED" | grep -q "^${pkg_dir}/README.md$"; then
      CHECKLIST="${CHECKLIST}  - ${pkg_dir}: public surface changed (header / launch / config / file add-del / dep) — confirm README.md reflects it, or note in your report why no doc change is needed\n"
    fi
  fi

  # New .cpp files possibly missing from CMakeLists.txt. Staged adds AND
  # untracked files: an agent that writes a source file without `git add`
  # is the common case, and that is exactly when this gate needs to fire.
  NEW_SRC=$(printf '%s\n%s\n' "$ADDED_A" "$CHANGED_UNTRACKED" \
              | grep "^${pkg_dir}/src/.*\.cpp$" | grep -v test | sort -u || true)
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    bname=$(basename "$f")
    # Match the basename as a whole path token, not a bare substring: an
    # unanchored `grep -qF hand.cpp` matches inside `left_hand.cpp`, so a
    # genuinely unlisted new file reads as registered whenever its name is a
    # suffix of an existing entry. Require a non-filename char (or line edge) on
    # both sides; escape regex metacharacters in the basename first.
    bname_re=$(printf '%s' "$bname" | sed 's/[^A-Za-z0-9_]/\\&/g')
    if ! grep -qE "(^|[^A-Za-z0-9_])${bname_re}([^A-Za-z0-9_]|$)" "${pkg_dir}/CMakeLists.txt" 2>/dev/null; then
      WARNINGS="${WARNINGS}  - ${pkg_dir}: new file ${bname} not found in CMakeLists.txt\n"
    fi
  done <<< "$NEW_SRC"

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

# --- Phase 1b: Documentation + YAML sensors ---
# Scoped to the CHANGED files, never the whole corpus. CI scans everything;
# a Stop hook that did the same would let a defect in an untouched file --
# or a gitignored scratch note invisible to `git status` -- block every turn
# with no way out.
DOC_FAILURES=""
DOC_FILES=()
while IFS= read -r d; do
  [ -n "$d" ] && [ -f "$d" ] && DOC_FILES+=("$d")
done <<< "$CHANGED_DOCS"
if [ "${#DOC_FILES[@]}" -gt 0 ]; then
  # Resolved relative to this hook, not to PROJECT_DIR: the script ships in the
  # same repository as the hook, so this keeps working when the two are pointed
  # at different trees (as the routing tests do).
  HOOK_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
  VALIDATE_DOCS="$HOOK_DIR/../../repo_scripts/scripts/validate_docs.py"
  [ -f "$VALIDATE_DOCS" ] || VALIDATE_DOCS="$PROJECT_DIR/repo_scripts/scripts/validate_docs.py"
  if [ -f "$VALIDATE_DOCS" ]; then
    # One invocation, and the verdict comes from the exit status. The previous
    # form piped through `xargs -0`, which re-invokes the validator once per
    # ARG_MAX batch, and then decided by grepping the concatenated output for
    # "docs validation clean" -- so if any batch was clean, every other batch's
    # findings were discarded.
    DOC_RC=0
    DOC_OUT=$(python3 "$VALIDATE_DOCS" --files "${DOC_FILES[@]}" 2>&1) || DOC_RC=$?
    if [ "$DOC_RC" -ne 0 ]; then
      # Scope findings in a TRACKED doc to its added lines, mirroring ARCH-1.
      # Whole-file scope meant that touching any .md made every pre-existing
      # defect in it blocking: the agent had to repair damage it did not cause,
      # in a file it edited incidentally, before it could end the turn. An
      # untracked doc is new in its entirety, so all of its lines are in scope.
      DOC_ALLOW=$(mktemp)
      for d in "${DOC_FILES[@]}"; do
        if git ls-files --error-unmatch "$d" >/dev/null 2>&1; then
          git diff -U0 HEAD -- "$d" 2>/dev/null | awk -v F="$d" '
            /^@@/ {
              match($0, /\+[0-9]+(,[0-9]+)?/)
              spec = substr($0, RSTART + 1, RLENGTH - 1)
              split(spec, p, ",")
              count = (p[2] == "" ? 1 : p[2])
              for (i = 0; i < count; i++) print F ":" p[1] + i
            }' >> "$DOC_ALLOW" || true
        else
          printf '%s:*\n' "$d" >> "$DOC_ALLOW"
        fi
      done
      DOC_FAILURES=$(echo "$DOC_OUT" | awk -v allow="$DOC_ALLOW" '
        BEGIN {
          while ((getline l < allow) > 0) {
            if (l ~ /:\*$/) { sub(/:\*$/, "", l); wholefile[l] = 1 } else keep[l] = 1
          }
        }
        # Validator findings are "path:line: [Dn] message"; anything else is a
        # summary or a traceback and must survive so real breakage stays loud.
        {
          if (match($0, /^[^:]+:[0-9]+: \[D[0-9]+\] /)) {
            head = substr($0, 1, RLENGTH)
            sub(/: \[D[0-9]+\] $/, "", head)
            split(head, hp, ":")
            if (head in keep || hp[1] in wholefile) print "  - " $0
          } else if ($0 !~ /^$/ && $0 !~ /finding\(s\) across/) {
            print "  - " $0
          }
        }' || true)
      rm -f "$DOC_ALLOW"
      # Everything filtered out means the touched lines are clean.
      [ -n "$(echo "$DOC_FAILURES" | grep -v '^[[:space:]]*$' || true)" ] || DOC_FAILURES=""
    fi
  fi
fi

YAML_FAILURES=""
if [ -n "$CHANGED_YAML" ]; then
  # config/**/*.yaml is a first-class surface here (device backends, controller
  # gains, robot profiles) and had no gate at all. This one only proves the
  # file parses -- schema is out of scope -- but a YAML that does not load is
  # a launch-time failure that no test would have caught either.
  #
  # Fail OPEN when PyYAML is missing, matching clang-format and shellcheck. The
  # first cut treated the ImportError as a parse failure, so a hook invocation
  # without the venv wedged every turn that touched a YAML, with no in-band
  # recovery.
  if python3 -c 'import yaml' >/dev/null 2>&1; then
    while IFS= read -r yf; do
      [ -n "$yf" ] && [ -f "$yf" ] || continue
      # safe_load_all, not safe_load: a multi-document file is legal YAML and
      # rejecting it is a false block. The verdict is the exit STATUS -- keying
      # on "stderr is non-empty" turned any interpreter warning (a venv
      # DeprecationWarning, PYTHONDEVMODE ResourceWarning) into a hard failure.
      YRC=0
      YERR=$(python3 -c 'import sys,yaml
with open(sys.argv[1], encoding="utf-8") as fh:
    list(yaml.safe_load_all(fh))' "$yf" 2>&1) || YRC=$?
      if [ "$YRC" -ne 0 ]; then
        YAML_FAILURES="${YAML_FAILURES}  - ${yf}: $(echo "$YERR" | tail -1)\n"
      fi
    done <<< "$CHANGED_YAML"
  else
    echo "verify-changes: PyYAML not importable; YAML parse gate skipped." >&2
  fi
fi

# --- Phase 2: Build + test, with PROC-3 fallback for rtc_base / rtc_msgs ---
# RTC_VERIFY_SKIP_BUILD lets repo_scripts/test/test_verify_changes.sh exercise
# the routing without a colcon workspace. It is never set in normal operation.
TEST_FAILURES=""
PROC3=$(echo "$BUILD_PKGS" | tr ' ' '\n' | grep -E '^(rtc_base|rtc_msgs)$' || true)
if [ -n "${RTC_VERIFY_SKIP_BUILD:-}" ]; then
  # Emit the routing decision before discarding it. Blanking BUILD_PKGS is what
  # lets the suite run without a colcon workspace, but it also made the build
  # routing structurally unobservable -- so the central claim of the untracked
  # scoping ("a new header under <pkg>/include/ IS built, a scratch file is
  # not") had no assertion behind it. This probe is the seam the tests read.
  echo "verify-changes[probe]: BUILD_PKGS=[${BUILD_PKGS# }] PROC3=[$(echo "$PROC3" | tr '\n' ' ' | sed 's/ *$//')]" >&2
  PROC3=""
  BUILD_PKGS=""
fi

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
  for pkg in $BUILD_PKGS; do
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
    while IFS= read -r path; do
      [ -n "$path" ] || continue
      bname=$(basename "$path")
      for root in $INSTALL_ROOTS; do
        FOUND=$(find "$root" -name "$bname" -type f 2>/dev/null | head -3 || true)
        if [ -n "$FOUND" ]; then
          STALE_INSTALL="${STALE_INSTALL}  - ${path} deleted from src but still in install/:\n$(echo "$FOUND" | sed 's/^/      /')\n"
          break
        fi
      done
    done <<< "$DELETED"
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
    while IFS= read -r f; do
      [ -n "$f" ] || continue
      [ -f "$f" ] || continue
      SC_OUT=$(shellcheck --severity=warning -f gcc "$f" 2>/dev/null || true)
      [ -n "$SC_OUT" ] && SHELLCHECK_FAILURES="${SHELLCHECK_FAILURES}${SC_OUT}\n"
    done <<< "$CHANGED_SH"
  else
    echo "verify-changes: shellcheck not found; shell-script lint gate skipped." >&2
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
if [ -n "$DOC_FAILURES" ]; then
  REPORT="${REPORT}Documentation validation (repo_scripts/scripts/validate_docs.py):\n${DOC_FAILURES}\n"
fi
if [ -n "$YAML_FAILURES" ]; then
  REPORT="${REPORT}YAML parse failures:\n${YAML_FAILURES}\n"
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

# Constitution parity: CLAUDE.md and AGENTS.md state the same rules for two
# different audiences (Claude Code / every other tool), and nothing else in the
# harness looks at AGENTS.md at all.  It went 4 commits stale that way, losing
# ARCH-7, NUM-5 and the cwd-drift recovery notes for the tools that only read
# it.  Non-blocking on purpose: a Claude-only change (hook wiring, slash command)
# legitimately touches one and not the other -- the agent decides, and says so.
if echo "$CHANGED_TRACKED" | grep -qx 'CLAUDE.md' && \
   ! echo "$CHANGED_TRACKED" | grep -qx 'AGENTS.md'; then
  CHECKLIST="${CHECKLIST}  - CLAUDE.md changed without AGENTS.md: if the edit states a rule (not a Claude-only mechanism), mirror it into AGENTS.md so non-Claude tools get it too — or note in your report why it is Claude-specific\n"
fi
if echo "$CHANGED_TRACKED" | grep -qx 'AGENTS.md' && \
   ! echo "$CHANGED_TRACKED" | grep -qx 'CLAUDE.md'; then
  CHECKLIST="${CHECKLIST}  - AGENTS.md changed without CLAUDE.md: confirm the same rule holds for Claude Code, or note why it does not\n"
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
