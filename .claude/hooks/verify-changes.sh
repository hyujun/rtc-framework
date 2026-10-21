#!/bin/bash
# Stop hook — gate the turn end on doc/metadata co-updates + build/test.
#
# Intent : enforce AGENTS.md §4 Workflow Loop steps 4·5·6 + PROC-1 (doc/code
#          sync) without trusting the agent's self-check.
# Trigger: every turn end. Reads {stop_hook_active} from stdin JSON; bails
#          early on re-entry. Also reads {background_tasks}: defers (exit 0,
#          watermark kept) while a background agent may still be writing the
#          checkout -- see "Background agents still writing the checkout".
# Modes  : (no argument)  the turn-end call. Runs every gate EXCEPT the build
#                         and the tests: for those it only CHECKS that the
#                         changed packages carry a green verdict for their
#                         present content, and blocks when one does not.
#          --run          called by hand, during the turn. Runs every gate AND
#                         builds and tests the changed packages, and records
#                         the verdicts the turn-end call looks for. Reads no
#                         stdin. Long runs: background it, then wait for it in
#                         the FOREGROUND (`tail --pid=<pid> -f /dev/null`, the
#                         pid being the first field of the run lock) before
#                         ending the turn. Ending the turn to await the tool's
#                         completion notice gets a block whose only way out --
#                         the stop_hook_active re-entry -- grades nothing.
#          The turn end stopped building on 2026-10-01: in a month it blocked
#          25 times with no real failure, and most of its test time re-ran
#          suites just run. It keeps: a tree edited after its last green test
#          run does not pass.
#
# Phases :
#   0.  ARCH-5 robot_descriptions stays an <exec_depend> (added lines) · ARCH-7
#       no new rtc_* executable target NAME (`example_*` and an `ARCH-7-exempt`
#       comment opt out) · ARCH-1 robot names / `num_joints=<literal>` on added
#       lines of rtc_* production code, negation lines dropped · ARCH-4 no
#       rtc_*/src/ include from an integration package · 0b ARCH-6 QoS depth
#       (NON-BLOCKING) · 0c PROC-8 test wait helpers stay sleep-only.
#   1.  README reminder on a public-surface change -- header / launch / config,
#       a directory (<= 2 levels below the package) added or removed, package.xml
#       (NON-BLOCKING; a file add/del inside an existing directory is not one,
#       so READMEs stop growing per-file trees, #869 D4); a new .cpp
#       must be in CMakeLists.txt, a new find_package() in package.xml
#       (blocking).
#   1b. changed *.md -> validate_docs.py --files, findings narrowed to added
#       lines of a tracked doc; changed *.yaml -> parse check (no PyYAML:
#       fails OPEN).
#   1c. changed .claude/rules/*.md -> validate_claude_rules.py (globs can fire)
#   1d. changed CMakeLists.txt / conftest.py / colcon.pkg / test sources ->
#       validate_test_{domains,fixtures,registration}.py, WHOLE repo.
#   2.  Build + test on changed packages -- EXECUTED under --run, only CHECKED
#       at the turn end ("Turn end: evidence, not execution"). Routing: a
#       package's source, build metadata, shell scripts in its source dirs and
#       other tracked non-Markdown files (CHANGED_*_BUILD; owning package
#       only); rtc_base / rtc_msgs -> ./build.sh full --tests + colcon test all
#       (PROC-3); a changed header also routes the packages that compile
#       against it (PROC-2, header_dependents), a .cpp edit does not; else
#       ./build.sh -p <pkg> --tests + colcon test <pkg>, one at a time in
#       dependency order ("Build order"). The verdict is the exit code of
#       `colcon test --return-code-on-test-failure` plus its count of packages
#       asked for (run_colcon_test): a timeout, a launch failure, a failure
#       with no failing result, fewer packages finished, or BUILD_TESTING=OFF
#       after the build (pkgs_built_without_tests) is UNVERIFIED and blocks.
#       A result file whose every case was skipped (a dependency only the venv
#       has: colcon runs pytest under /usr/bin/python3, see
#       agent_docs/testing-debug.md) is a NOTE after the PASS line; the agent
#       runs it by hand (skipped_result_files).
#   3.  a deleted launch/config file install/ still holds, as a DANGLING link
#       or a copied file of that name (Phase 3).
#   4.  shellcheck --severity=warning on changed *.sh, whole file (Phase 4).
#   5.  formatter drift (clang-format / ruff format) the change INTRODUCED;
#       pre-existing debt passes. At the turn end, past
#       RTC_VERIFY_FORMAT_DEADLINE_S (480s) the rest is listed as ungraded;
#       --run has no deadline. Runs BEFORE Phase 2: under --run, drift means
#       nothing is built.
#   NOT graded: `ruff check` lint, Doxygen, YAML default/range/unit, README
#   need (modification-guide.md Completion Checklist).
#
# Verdict reuse -- keyed on content, never on time or the watermark; only a
# PASS that went through build/test is remembered. An identical WHOLE tree runs
# nothing ("Nothing changed since the last pass"); a package whose directories
# (Markdown aside) match its last green build is not rebuilt (--run) or owed
# (turn end), while every other gate still runs ("Package verdict reuse").
# --run also answers for the installed BINARIES ("The binaries a verdict was
# recorded with"); the turn end does not look at the install tree. A
# backgrounded --run holds a lock (run_lock_holder), records no verdict for a
# package edited meanwhile (remember_tested_verdict), and moves the watermark
# only if the tree is still the one graded (end of this file). A simulator from
# this workspace, or a host hold (workspace_holds), DEFERS a missing verdict at
# the turn end and makes --run refuse to build.
#   Names, for a document that points here instead of restating this (the
#   comment at each definition owns the detail): RTC_VERIFY_NO_REUSE=1 turns
#   both reuses off; RTC_VERIFY_BASE=<commit> names the baseline of a --run
#   (see "Verification baseline"); the pass files and rtc-verify-timing.log
#   (one line per run, with its mode) are kept in .git/ ("Kept in .git/
#   beside the watermark"); a measurement holds the host through
#   <workspace>/.rtc-verify-hold, which repo_scripts/scripts/with_verify_hold.sh
#   writes for the lifetime of the driver it wraps (workspace_holds).
#
# Pure-format fast path: Phases 0 + 1 are SKIPPED when every changed source
#   file equals the baseline after its formatter; Phase 2 still runs.
#
# Exit   : 0 on pass (a non-blocking checklist may still print to stderr).
#          2 on any hard failure -- at the turn end a changed package without
#          a verdict is one; stderr is injected next turn, with a pointer to
#          modification-guide.md. --run exits the same way (2 also when it
#          refused to build); 64 on an unknown argument.
#          Claude Code overrides the hook after 8 CONSECUTIVE blocks
#          (code.claude.com/docs/en/best-practices) -- an unverified stop, not
#          an exit; do not lean on it.
# Limits : --run bounds (seconds, each an environment override): one package
#          build RTC_VERIFY_BUILD_BOUND_S (900), its tests
#          RTC_VERIFY_TEST_BOUND_S (600); the PROC-3 workspace build
#          RTC_VERIFY_FULL_BUILD_BOUND_S (2400) and its tests
#          RTC_VERIFY_FULL_TEST_BOUND_S (1200). They end a hang, not fit a
#          budget: nothing kills --run from outside. Sized on this 6C/12T box
#          (repo_scripts/README.md "빌드 병렬도와 메모리"): a package
#          recompiled 241-279s, a warm workspace after an rtc_msgs touch 353s,
#          a clean `build.sh full --tests` 18m49s, the slowest suite 105-177s.
#          A bound hit (exit 124) or launch failure (exit >=125) blocks as
#          UNVERIFIED, in a message DISTINCT from a real failure's (#435); a
#          124 also reports loadavg and the live build/compiler count, a real
#          failure its exit code and the tail of the build log.
#          A colcon / build.sh already running in this workspace (name + cwd:
#          workspace_build_rivals) means --run builds nothing and the turn end
#          says to wait. A MuJoCo simulator from this workspace's install tree
#          makes --run refuse too, and DEFERS a missing verdict at the turn
#          end (exit 0, watermark kept; workspace_sim_rivals).
#          Doxygen / cross-package doc consistency NOT checked
#          (modification-guide.md "Updating an Existing Package" and its
#          Completion Checklist cover these manually). Changed set = tracked-vs-$VERIFY_BASE UNION
#          untracked, where VERIFY_BASE is the watermark commit this hook last
#          passed at (see "Verification baseline" below) — NOT HEAD, so work
#          the agent committed during the turn is still verified;
#          build/test additionally requires an untracked file to live in one of
#          the installed-source dirs allowlisted at CHANGED_SRC_UNTRACKED below
#          (the SSoT) or, if it is not source, under config/ (YAML) or test/
#          (CHANGED_DATA_BUILD): scratch elsewhere cannot trigger a build.
#          Routing behaviour is asserted end-to-end by
#          repo_scripts/test/test_verify_changes.sh.
#          A path containing a newline is C-quoted by git regardless of
#          core.quotePath and is NOT handled; paths with spaces are.
set -euo pipefail

# Mode (see "Modes" in the header). --run is typed by hand and must not sit
# waiting on a terminal for stdin.
RUN_MODE=""
case "${1:-}" in
  "") ;;
  --run) RUN_MODE=1 ;;
  *)
    echo "usage: verify-changes.sh [--run]   (no argument: the Stop hook call, input on stdin)" >&2
    exit 64
    ;;
esac

if [ -n "$RUN_MODE" ]; then
  INPUT='{}'
else
  INPUT=$(cat)
fi

# Prevent infinite loop: only fire once per stop cycle
if [ "$(echo "$INPUT" | jq -r '.stop_hook_active')" = "true" ]; then
  exit 0
fi

# By hand the shell's cwd is wherever the last command left it, so --run
# falls back to the checkout this script sits in rather than to $(pwd).
HOOK_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ -n "$RUN_MODE" ]; then
  PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(cd "$HOOK_DIR/../.." 2>/dev/null && pwd)}"
else
  PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(pwd)}"
fi
cd "$PROJECT_DIR" 2>/dev/null || exit 0

# Resolve the colcon workspace root (sibling-of-src) and source setup_env.sh so
# bare `colcon` calls below find ROS, deps, venv and COLCON_DEFAULTS_FILE
# without a pre-sourced parent shell. The RTC_DEPS_PREFIX guard mirrors
# bootstrap.sh, so a re-source is a no-op.
WORKSPACE="$(cd "$PROJECT_DIR/../.." 2>/dev/null && pwd)"
SETUP_ENV="$PROJECT_DIR/repo_scripts/scripts/setup_env.sh"
if [ -z "${RTC_DEPS_PREFIX:-}" ] && [ -f "$SETUP_ENV" ]; then
  # ROS setup.bash reads unbound vars (AMENT_TRACE_SETUP_FILES); setup_env.sh
  # lifts -u for its own duration and restores it, so this `set -u` hook survives.
  # shellcheck source=/dev/null
  source "$SETUP_ENV"
fi

# ── Verification baseline ──
#
# Every gate below diffs against $VERIFY_BASE, not HEAD: with HEAD a turn that
# ends `git commit`-clean (the normal shape, AGENTS.md §11) has an empty change
# set and NOTHING is verified.
#
# The baseline is a WATERMARK — the commit this hook last finished a clean run
# at — kept in .git/ (per clone, never committed): "everything that changed
# since this gate last passed". A turn that commits is verified ONCE, then the
# watermark advances to HEAD; a blocked run (exit 2) does NOT advance it; the
# added-line narrowing (docs, ARCH-1, ARCH-5) sees lines added by those commits.
#
# FALLS BACK TO HEAD whenever the watermark is missing, unreadable, not a
# commit, or no longer an ancestor of HEAD (rebase, branch switch, reset) --
# never a diff against an unrelated history. Then the commits of a branch just
# switched to are not graded, so --run (only):
#   * SAYS so when HEAD carries commits the watermark's line does not (below);
#   * takes the baseline by name, RTC_VERIFY_BASE=<commit>, which must be an
#     ancestor of HEAD or it blocks. A named baseline also overrides "this tree
#     already passed" (that pass may be the empty one); package verdicts,
#     keyed on content, are still reused.
# The turn end ignores RTC_VERIFY_BASE: it builds nothing, and a branch switch
# must not leave every later turn owing verdicts for commits made elsewhere.
VERIFY_BASE_FILE="$(git rev-parse --git-dir 2>/dev/null || echo .git)/rtc-verify-base"
VERIFY_BASE=HEAD
BASE_NAMED=""
if [ -n "$RUN_MODE" ] && [ -n "${RTC_VERIFY_BASE:-}" ]; then
  NAMED_BASE=$(git rev-parse --verify --quiet "${RTC_VERIFY_BASE}^{commit}" 2>/dev/null || true)
  if [ -z "$NAMED_BASE" ] || ! git merge-base --is-ancestor "$NAMED_BASE" HEAD 2>/dev/null; then
    echo "verify-changes --run: RTC_VERIFY_BASE='${RTC_VERIFY_BASE}' is not a commit HEAD descends from -- nothing graded. Name the commit this branch left the verified line at, e.g. RTC_VERIFY_BASE=\$(git merge-base origin/main HEAD)." >&2
    exit 2
  fi
  VERIFY_BASE="$NAMED_BASE"
  BASE_NAMED=1
elif [ -f "$VERIFY_BASE_FILE" ]; then
  WATERMARK=$(cat "$VERIFY_BASE_FILE" 2>/dev/null || true)
  if [ -n "$WATERMARK" ] \
     && git rev-parse --verify --quiet "${WATERMARK}^{commit}" >/dev/null 2>&1; then
    if git merge-base --is-ancestor "$WATERMARK" HEAD 2>/dev/null; then
      VERIFY_BASE="$WATERMARK"
    elif [ -n "$RUN_MODE" ]; then
      # Only when HEAD holds commits the watermark's line does not: back on the
      # commit a graded branch was cut from there is nothing left out.
      FORK=$(git merge-base "$WATERMARK" HEAD 2>/dev/null || true)
      UNGRADED=0
      [ -z "$FORK" ] || UNGRADED=$(git rev-list --count "${FORK}..HEAD" 2>/dev/null || echo 0)
      if [ "${UNGRADED:-0}" -gt 0 ]; then
        echo "verify-changes --run: the watermark (${WATERMARK:0:8}) is not an ancestor of HEAD -- a branch switch, rebase or reset since this gate last passed. The baseline is HEAD: only what is not committed is graded, and the ${UNGRADED} commit(s) since ${FORK:0:8} are NOT. To grade them: RTC_VERIFY_BASE=${FORK:0:8} .claude/hooks/verify-changes.sh --run" >&2
      fi
    fi
  fi
fi

# Called on every NON-BLOCKING exit, never on exit 2 and never on the
# stop_hook_active re-entry (which verified nothing and must not claim to have).
advance_verify_base() {
  git rev-parse HEAD > "$VERIFY_BASE_FILE" 2>/dev/null || true
}

# ── Verdict reuse: state and helpers ──
#
# The watermark is a COMMIT, so an uncommitted change stays "changed" however
# often it passed. A tree's verdict is a function of its content, so content is
# what is remembered.
#
# Kept in .git/ beside the watermark: per clone, never committed.
#   rtc-verify-pass-tree   tree id of the working tree at the last full pass
#   rtc-verify-pass-pkgs   "<pkg> <content key>" per package that built and
#                          tested green
#   rtc-verify-timing.log  one line per run: when, verdict, seconds, packages,
#                          mode (stop = the turn-end call, run = --run), and
#                          reason= the gates that blocked (`arch+build`; `-` on
#                          a pass) -- codes: block_reasons
# RTC_VERIFY_NO_REUSE=1 switches both reuses off (every gate runs, nothing is
# read from the two pass files; a pass is still recorded).
GIT_DIR_PATH="$(git rev-parse --git-dir 2>/dev/null || echo .git)"
PASS_TREE_FILE="$GIT_DIR_PATH/rtc-verify-pass-tree"
PASS_PKGS_FILE="$GIT_DIR_PATH/rtc-verify-pass-pkgs"
TIMING_LOG="$GIT_DIR_PATH/rtc-verify-timing.log"
RUN_LOCK="$GIT_DIR_PATH/rtc-verify-run.lock"

# ── The binaries a verdict was recorded with ──
#
# The pass files say "this source built and tested green", not what is
# INSTALLED: after a build on other content (a temporary patch, a change that
# failed) the source can return to what passed while the binaries do not, and
# the next sim would run code that is in no commit.
# So a verdict is recorded with a stamp of the package's installed compiled
# products, and --run does not reuse a verdict whose stamp no longer matches:
# it builds and tests that package again.
#   rtc-verify-pass-artifacts   "<pkg> <stamp>" per package with a verdict
#   rtc-verify-pass-sources     "<pkg> <id>": the package's source content
#                               (the next section) when the stamp was taken
# The stamp is the name and CONTENT (blob id, not mtime: a relink of unchanged
# source gets a new mtime) of every shared or static library under
# <workspace>/install/<pkg>/lib and every ELF file directly in lib/<pkg>/.
# Scripts and the Python tree are left out (every build rewrites them). About
# 0.4 s, paid only by --run and a recorded verdict.
#
# The TURN END does not read the stamp. A debug, sanitizer or tracing build
# kept on purpose across turns also has "other binaries than the verdict's",
# and blocking on it would have every turn end ask for a --run that replaces
# that build. What the turn end guarantees stays what it was: the source has a
# green verdict. A package with no install directory has no stamp and is never
# stale by it (ament_python, a fixture without a workspace).
#
# The stamp does not say WHOSE source the binaries were built from (a branch
# verified here, then back on main). So a verdict also records the package's
# SOURCE id (pkg_source_ids), and --run treats a package whose id moved while
# its binaries are still the recorded ones as owed a build and a test; the
# message says which case. Its dependents are not rebuilt.
PASS_ARTIFACTS_FILE="$GIT_DIR_PATH/rtc-verify-pass-artifacts"
PASS_SOURCES_FILE="$GIT_DIR_PATH/rtc-verify-pass-sources"
artifact_stamp() {  # $1 = package
  local lib="$WORKSPACE/install/$1/lib" f
  [ -d "$lib" ] || return 0
  {
    find -L "$lib" -type f \( -name '*.so' -o -name '*.so.*' -o -name '*.a' \) \
      -not -path '*/python3*' -print 2>/dev/null || true
    if [ -d "$lib/$1" ]; then
      for f in "$lib/$1"/*; do
        [ -f "$f" ] || continue
        # ELF magic, read as hex: the first four bytes are 7f 45 4c 46.
        [ "$(head -c 4 "$f" 2>/dev/null | od -An -tx1 | tr -d ' \n')" = "7f454c46" ] || continue
        printf '%s\n' "$f"
      done
    fi
  } | LC_ALL=C sort | while IFS= read -r f; do
      printf '%s %s\n' "${f#"$lib"/}" "$(git hash-object "$f" 2>/dev/null || true)"
    done | git hash-object --stdin 2>/dev/null || true
}
# The packages a verdict names: itself, or every package for PROC-3.
verdict_packages() {  # $1 = package or PROC-3
  local d
  if [ "$1" != "PROC-3" ]; then printf '%s\n' "$1"; return 0; fi
  for d in "$PROJECT_DIR"/*/; do
    [ -f "${d}package.xml" ] && basename "$d"
  done
  return 0
}
# One "<mode> <type> <blob>\t<path>" line per file of a package directory of
# tree $1 (a tree id from work_tree_id), Markdown left out: what both a
# package's verdict key (pkg_content_key) and its source id (pkg_source_ids)
# are made of, so the two cannot disagree. quotePath off: a quoted non-ASCII
# path ends in `"`, and its ".md" would not be seen.
pkg_tree_lines() {  # $1 = a tree id
  local pkgs
  pkgs=$(git_scratch ls-tree -r --name-only "$1" 2>/dev/null \
           | sed -n 's|^\([^/]*\)/package\.xml$|\1|p' || true)
  git_scratch -c core.quotePath=false ls-tree -r "$1" 2>/dev/null \
    | awk -v pkgs="$pkgs" '
        BEGIN { n = split(pkgs, a, "\n"); for (i = 1; i <= n; i++) if (a[i] != "") keep[a[i]] = 1 }
        {
          path = $0; sub(/^[^\t]*\t/, "", path)
          top = path; sub(/\/.*/, "", top)
          if ((top in keep) && path !~ /\.md$/) print
        }' || true
}
# The package-route predicates, for every awk below that reads a path as
# fields split on "/" (prepended to the program: "$AWK_ROUTE"'...').
#   installed_src_dir(d, pkg)  the dirs whose files, untracked or not, are
#     built as part of <pkg>: src/ and include/ (ament_cmake source), test/
#     (gtest sources and test-only headers), <pkg>/<pkg>/ (ament_python
#     modules), launch/ and scripts/ (DIRECTORY-installed: a new file reaches
#     install/ with no metadata edit, #360 -- the build validates nothing
#     there, it buys a matching install tree and a rerun of the tests).
#     config/ is deliberately absent (no source; it would make this "any
#     directory"): its YAML is data, below.
#   config_yaml(d, f)  a YAML under config/, loaded by the tests from the
#     install tree.
#   test_data(d, f)    anything under test/ but Markdown: a fixture the tests
#     read (a URDF, an MJCF, a golden table).
AWK_ROUTE='
  function installed_src_dir(d, pkg) {
    return d == "src" || d == "include" || d == "test" || d == "launch" \
           || d == "scripts" || d == pkg
  }
  function config_yaml(d, f) { return d == "config" && f ~ /\.(yaml|yml)$/ }
  function test_data(d, f) { return d == "test" && f !~ /\.md$/ }'
# The untracked files the build route takes as part of their package (stdin to
# stdout, one path per line): the union of what CHANGED_SRC_UNTRACKED,
# CHANGED_SH_BUILD and CHANGED_DATA_BUILD route (case 76l of the suite holds
# one file of each class to both). Every other untracked file is scratch.
untracked_routed() {
  awk -F/ "$AWK_ROUTE"'
    NF >= 3 && ( ($NF ~ /\.(cpp|hpp|h|cc|py|sh)$/ && installed_src_dir($2, $1)) \
                 || config_yaml($2, $NF) || test_data($2, $NF) )' || true
}
# "<pkg> <id> <scratch>" per package directory of tree $1: the hash of its
# files by blob (pkg_tree_lines), untracked scratch left out, and whether the
# package holds such scratch (1) or not (0). One sha1sum per package.
#
# An untracked file the build route takes (untracked_routed) IS in the id (its
# line is the same once committed). Untracked SCRATCH is left out, and a
# package holding some is recorded WITHOUT an id: counted in, a note in
# rtc_base/ would force the PROC-3 build; left out but recorded, the id would
# move once the file is committed. No id: never stale by its source.
#
# The untracked list reaches awk on stdin, ahead of the tree ("?" every
# untracked path, "+" the routed ones; a tree line starts with its mode): as a
# -v argument it overflowed the per-argument limit and every id came back
# empty. An embedded repository is listed as "dir/", a gitlink at "dir".
pkg_source_ids() {  # $1 = a tree id
  local untracked
  [ -n "$1" ] || return 0
  untracked=$(git -c core.quotePath=false ls-files -o --exclude-standard 2>/dev/null || true)
  {
    printf '%s\n' "$untracked" | sed -e 's|/$||' -e 's|^|? |'
    printf '%s\n' "$untracked" | untracked_routed | sed -e 's|^|+ |'
    pkg_tree_lines "$1"
  } | awk '
        BEGIN { cmd = "sha1sum | cut -c1-40 | tr -d \"\\n\"" }
        /^\? / { scratch[substr($0, 3)] = 1; next }
        /^\+ / { delete scratch[substr($0, 3)]; next }
        {
          path = $0; sub(/^[^\t]*\t/, "", path)
          top = path; sub(/\/.*/, "", top)
          pkg[top] = 1
          if (path in scratch) { dirty[top] = 1; next }
          buf[top] = buf[top] $0 "\n"
        }
        END {
          for (t in pkg) {
            printf "%s ", t; fflush()
            printf "%s", buf[t] | cmd
            close(cmd)
            printf " %d\n", (t in dirty) ? 1 : 0
          }
        }' || true
}
# The ids of tree $1 in SOURCE_IDS, computed once per tree in a call.
SOURCE_IDS=""
SOURCE_IDS_TREE=""
load_source_ids() {  # $1 = a tree id
  if [ -z "$1" ]; then
    SOURCE_IDS=""
  elif [ "$1" != "$SOURCE_IDS_TREE" ]; then
    SOURCE_IDS=$(pkg_source_ids "$1")
    SOURCE_IDS_TREE="$1"
  fi
}
remember_artifact_stamps() {  # $1 = package or PROC-3, $2 = the tree it passed on
  local p stamp id
  load_source_ids "${2:-$(work_tree_id)}"
  {
    while IFS= read -r p; do
      [ -n "$p" ] || continue
      stamp=$(artifact_stamp "$p")
      id=$(awk -v p="$p" '$1 == p && $3 == 0 { print $2 }' <<< "$SOURCE_IDS")
      grep -v "^$p " "$PASS_ARTIFACTS_FILE" > "$PASS_ARTIFACTS_FILE.tmp" || true
      [ -z "$stamp" ] || printf '%s %s\n' "$p" "$stamp" >> "$PASS_ARTIFACTS_FILE.tmp"
      mv "$PASS_ARTIFACTS_FILE.tmp" "$PASS_ARTIFACTS_FILE"
      grep -v "^$p " "$PASS_SOURCES_FILE" > "$PASS_SOURCES_FILE.tmp" || true
      [ -z "$stamp" ] || [ -z "$id" ] || printf '%s %s\n' "$p" "$id" >> "$PASS_SOURCES_FILE.tmp"
      mv "$PASS_SOURCES_FILE.tmp" "$PASS_SOURCES_FILE"
    done <<< "$(verdict_packages "$1")"
  } 2>/dev/null || true
}
# A --run that passes also stamps every package with no stamp yet, so a
# temporary patch is watched in any package, not only those --run built. A
# baseline, not a verdict: never refreshed here (only a green build and test
# replaces a stamp). Likewise a stamped package with no source id gets the
# present tree's, unless it holds untracked scratch (pkg_source_ids).
baseline_artifact_stamps() {
  local p stamp id
  load_source_ids "${WORK_TREE:-$(work_tree_id)}"
  {
    while IFS= read -r p; do
      [ -n "$p" ] || continue
      if ! grep -q "^$p " "$PASS_ARTIFACTS_FILE" 2>/dev/null; then
        stamp=$(artifact_stamp "$p")
        [ -z "$stamp" ] || printf '%s %s\n' "$p" "$stamp" >> "$PASS_ARTIFACTS_FILE"
      fi
      grep -q "^$p " "$PASS_ARTIFACTS_FILE" 2>/dev/null || continue
      grep -q "^$p " "$PASS_SOURCES_FILE" 2>/dev/null && continue
      id=$(awk -v p="$p" '$1 == p && $3 == 0 { print $2 }' <<< "$SOURCE_IDS")
      [ -z "$id" ] || printf '%s %s\n' "$p" "$id" >> "$PASS_SOURCES_FILE"
    done <<< "$(verdict_packages PROC-3)"
  } 2>/dev/null || true
}
# Packages (still in the repo) whose installed binaries are not the ones
# recorded with their verdict, space separated.
stale_artifact_pkgs() {
  local p stamp
  [ -f "$PASS_ARTIFACTS_FILE" ] || return 0
  while read -r p stamp; do
    [ -n "$p" ] && [ -f "$PROJECT_DIR/$p/package.xml" ] || continue
    [ "$(artifact_stamp "$p")" = "$stamp" ] || printf '%s ' "$p"
  done < "$PASS_ARTIFACTS_FILE"
  return 0
}
# Packages (still in the repo) whose source id differs from the one recorded
# with their verdict while their installed binaries are still the recorded
# ones: built from another tree's source. Reads SOURCE_IDS (load_source_ids,
# called by the caller: this one runs in a command substitution).
# A package whose binaries differ is stale_artifact_pkgs's, not named here.
stale_source_pkgs() {
  local p id stamp
  [ -f "$PASS_SOURCES_FILE" ] && [ -f "$PASS_ARTIFACTS_FILE" ] || return 0
  [ -n "$SOURCE_IDS" ] || return 0
  while read -r p id; do
    [ -n "$p" ] && [ -f "$PROJECT_DIR/$p/package.xml" ] || continue
    [ "$(awk -v p="$p" '$1 == p { print $2 }' <<< "$SOURCE_IDS")" != "$id" ] || continue
    stamp=$(grep "^$p " "$PASS_ARTIFACTS_FILE" | cut -d' ' -f2 || true)
    [ -n "$stamp" ] && [ "$(artifact_stamp "$p")" = "$stamp" ] && printf '%s ' "$p"
  done < "$PASS_SOURCES_FILE"
  return 0
}
# Both pass files carry this tag in front of what they record: it names the
# test reading the verdict came from. Untagged entries predate the exit-code
# verdict (see run_colcon_test), when a failing package could be recorded
# green, and are not honoured.
VERDICT_TAG="t2:"

# One --run per checkout, and a turn end that can see it.
# --run holds this lock ("<pid> <start time>", start time = field 22 of
# /proc/<pid>/stat, as in the hold file) from its first gate to its exit, so a
# turn end sees it between builds and a second --run cannot build into the
# same trees. Prints "<pid>: verify-changes.sh --run" for a LIVE holder other
# than this process; a line left by a killed --run (pid gone or reused) reads
# as no holder.
# Start time of pid $1 (field 22 of /proc/<pid>/stat, clock ticks since boot):
# with the pid, what tells a live process from a reused number. comm may hold
# spaces; what follows the LAST ')' starts at field 3.
proc_start_time() { sed 's/^.*) //' "/proc/$1/stat" 2>/dev/null | cut -d' ' -f20; }
# Command line of pid $1, argv joined by spaces and cut to a report line.
proc_cmdline() { tr '\0' ' ' <"/proc/$1/cmdline" 2>/dev/null | cut -c1-120; }

run_lock_holder() {
  local pid="" start="" now
  [ -f "$RUN_LOCK" ] || return 0
  read -r pid start _ <"$RUN_LOCK" 2>/dev/null || true
  case "$pid" in '' | *[!0-9]*) return 0 ;; esac
  [ "$pid" != "$$" ] || return 0
  [ -r "/proc/$pid/stat" ] || return 0
  now=$(proc_start_time "$pid")
  [ -n "$start" ] && [ "$now" = "$start" ] || return 0
  printf '%s: verify-changes.sh --run\n' "$pid"
}

# Tree id of the working tree as it is now -- what `git add -A` would stage,
# through a throwaway index (ignored files out; a nested repository counts by
# its HEAD). Prints nothing when git cannot do it: every caller reads that as
# "no reuse". Objects go to a scratch object directory (removed at exit; the
# real store as alternate), so read them through git_scratch.
# The index starts EMPTY, so every file is hashed (~0.3 s). A copy of the real
# index is wrong: git trusts an entry whose size and mtime match unless it is
# as young as the index file, so a same-size edit right after a commit read as
# unchanged.
OBJ_STORE=$(git rev-parse --path-format=absolute --git-path objects 2>/dev/null || true)
OBJ_SCRATCH=$(mktemp -d 2>/dev/null || true)
RUN_LOCK_HELD=""
cleanup() {
  [ -n "${OBJ_SCRATCH:-}" ] && rm -rf "$OBJ_SCRATCH"
  [ -n "$RUN_LOCK_HELD" ] && rm -f "$RUN_LOCK"
  return 0
}
trap cleanup EXIT
git_scratch() {  # git, reading the scratch objects beside the repository's
  GIT_ALTERNATE_OBJECT_DIRECTORIES="$OBJ_SCRATCH" git "$@"
}
if [ -n "$RUN_MODE" ]; then
  RUN_RIVAL=$(run_lock_holder)
  if [ -z "$RUN_RIVAL" ]; then
    # Whatever is there now is stale. noclobber makes the create atomic: of two
    # --run started together, one finds the file already made.
    rm -f "$RUN_LOCK"
    if (set -o noclobber; printf '%s %s\n' "$$" \
          "$(proc_start_time "$$")" >"$RUN_LOCK") 2>/dev/null; then
      RUN_LOCK_HELD=1
    else
      RUN_RIVAL=$(run_lock_holder)
      RUN_RIVAL="${RUN_RIVAL:-another process (it took ${RUN_LOCK} first)}"
    fi
  fi
  if [ -z "$RUN_LOCK_HELD" ]; then
    echo "verify-changes --run: NOT run — another --run is already running in this checkout (${RUN_RIVAL}). Two of them would build into the same build/ and install/ trees. Wait for it to finish (your own background task: \`tail --pid=<its pid> -f /dev/null\` in the foreground); its verdict is the one the turn end reads." >&2
    exit 2
  fi
fi
work_tree_id() {
  local idx tree=""
  [ -d "$OBJ_STORE" ] && [ -d "$OBJ_SCRATCH" ] || return 0
  idx=$(mktemp -u) || return 0
  if GIT_INDEX_FILE="$idx" GIT_OBJECT_DIRECTORY="$OBJ_SCRATCH" \
     GIT_ALTERNATE_OBJECT_DIRECTORIES="$OBJ_STORE" git add -A . >/dev/null 2>&1; then
    tree=$(GIT_INDEX_FILE="$idx" GIT_OBJECT_DIRECTORY="$OBJ_SCRATCH" \
           GIT_ALTERNATE_OBJECT_DIRECTORIES="$OBJ_STORE" git write-tree 2>/dev/null || true)
  fi
  rm -f "$idx" "$idx.lock"
  printf '%s' "$tree"
}

# $1 = verdict, $2 = packages built/tested, $3 = packages reused. Append-only,
# trimmed to the last 500 runs, and never allowed to fail the hook.
log_timing() {
  {
    printf '%s\t%s\t%ss\tbuilt=[%s]\treused=[%s]\tmode=%s\treason=%s\n' \
      "$(date -Is 2>/dev/null || date)" "$1" "$SECONDS" "${2# }" "${3# }" \
      "$([ -n "$RUN_MODE" ] && echo run || echo stop)" "${4:--}" >> "$TIMING_LOG"
    if [ "$(wc -l < "$TIMING_LOG")" -gt 600 ]; then
      tail -n 500 "$TIMING_LOG" > "$TIMING_LOG.tmp" && mv "$TIMING_LOG.tmp" "$TIMING_LOG"
    fi
  } 2>/dev/null || true
}

# Changed files = tracked modifications vs $VERIFY_BASE, PLUS untracked new files.
# `git diff --name-only` alone misses a never-added file (the normal state of
# one the agent just wrote); `git ls-files -mo` misses a staged-then-untouched
# one (`-m` is relative to the index). The union covers both.
# core.quotePath=false: a C-quoted non-ASCII path matches no extension filter
# and would be silently exempt from every check below.
CHANGED_TRACKED=$(git -c core.quotePath=false diff --name-only "$VERIFY_BASE" 2>/dev/null || true)
CHANGED_UNTRACKED=$(git -c core.quotePath=false ls-files -o --exclude-standard 2>/dev/null || true)
CHANGED=$(printf '%s\n%s\n' "$CHANGED_TRACKED" "$CHANGED_UNTRACKED" | grep -v '^[[:space:]]*$' | sort -u || true)
# --run looks at the installed binaries before any "nothing to do" exit (see
# "The binaries a verdict was recorded with"): a package whose binaries were
# rebuilt since its verdict, or are another tree's build, is built and tested
# again -- with nothing changed against the watermark, and with the tree the
# one that last passed. The second case needs the tree id before the exit.
STALE_ARTIFACT_PKGS=""
WORK_TREE=""
if [ -n "$RUN_MODE" ] && [ -z "${RTC_VERIFY_NO_REUSE:-}" ]; then
  STALE_ARTIFACT_PKGS=$(stale_artifact_pkgs)
  if [ -n "$STALE_ARTIFACT_PKGS" ]; then
    echo "verify-changes: the installed binaries of [${STALE_ARTIFACT_PKGS% }] are not the ones their verdict was recorded with (built again since) -- building and testing them again." >&2
  fi
  WORK_TREE=$(work_tree_id)
  load_source_ids "$WORK_TREE"
  STALE_SOURCE_PKGS=$(stale_source_pkgs)
  if [ -n "$STALE_SOURCE_PKGS" ]; then
    # A package edited since its verdict is built by the change set anyway:
    # only the others are news. "Edited" = routed to a build (a tracked
    # non-Markdown file, or an untracked one the route takes). awk reads the
    # whole list: an early-exiting `grep -q` fails the pipe under pipefail.
    ROUTED_BY_CHANGE=$( { printf '%s\n' "$CHANGED_TRACKED" | grep -v '\.md$' || true
                          printf '%s\n' "$CHANGED_UNTRACKED" | untracked_routed; } )
    FOREIGN_PKGS=""
    for p in $STALE_SOURCE_PKGS; do
      if ! awk -F/ -v p="$p" 'NF >= 2 && $1 == p { hit = 1 } END { exit !hit }' <<< "$ROUTED_BY_CHANGE"; then
        FOREIGN_PKGS="${FOREIGN_PKGS}${p} "
      fi
    done
    if [ -n "$FOREIGN_PKGS" ]; then
      echo "verify-changes: the installed binaries of [${FOREIGN_PKGS% }] were built from other source than this tree's (a branch verified in this checkout?) -- building and testing them again." >&2
    fi
    STALE_ARTIFACT_PKGS="${STALE_ARTIFACT_PKGS}${STALE_SOURCE_PKGS}"
  fi
fi
# --run leaving with nothing to do says so ($1 = why), rather than exiting
# silently like a tool that did not start. What it says about the install tree
# is what it looked at: RTC_VERIFY_NO_REUSE switches the look off, and a
# package with no recorded id or stamp is not one it can find.
say_nothing_to_do() {
  [ -n "$RUN_MODE" ] || return 0
  if [ -n "${RTC_VERIFY_NO_REUSE:-}" ]; then
    echo "verify-changes --run: $1 -- nothing was built or tested (RTC_VERIFY_NO_REUSE: the installed binaries were not looked at)." >&2
  else
    echo "verify-changes --run: $1 and no installed binaries were found stale -- nothing was built or tested." >&2
  fi
}
if [ -z "$CHANGED" ] && [ -z "$STALE_ARTIFACT_PKGS" ]; then
  say_nothing_to_do "nothing changed against the baseline (${VERIFY_BASE:0:8})"
  [ -z "$RUN_MODE" ] || baseline_artifact_stamps
  advance_verify_base
  exit 0
fi

# Classify. Every class below has at least one check, so any of them keeps the
# turn in scope past the exit below -- docs-only, YAML-only, CMake-only and
# package.xml-only changes must reach the gates that read them.
CHANGED_SRC=$(echo "$CHANGED" | grep -E '\.(cpp|hpp|h|cc|py)$' || true)
CHANGED_SH=$(echo "$CHANGED" | grep -E '\.sh$' || true)
CHANGED_DOCS=$(echo "$CHANGED" | grep -E '\.md$' || true)
CHANGED_YAML=$(echo "$CHANGED" | grep -E '\.(yaml|yml)$' || true)
CHANGED_META=$(echo "$CHANGED" | grep -E '(^|/)(CMakeLists\.txt|package\.xml)$' || true)
# colcon.pkg at a package root: arguments colcon adds to every build / test of
# that package -- in this repo, the job count of its ctest. It changes how the
# package's tests RUN, so it routes the package to build/test and to the test
# gates the way a CMakeLists edit does.
CHANGED_TESTCFG=$(echo "$CHANGED" | grep -E '^[^/]+/colcon\.pkg$' || true)
# Any other tracked file of a PACKAGE, its Markdown aside: robot data, a
# behaviour tree, a .msg, a test's data file. No class above names them (see
# CHANGED_DATA_BUILD).
CHANGED_PKG_FILES=$(echo "$CHANGED_TRACKED" | awk -F/ 'NF >= 2 && $NF !~ /\.md$/' \
  | while IFS= read -r f; do
      if [ -f "${f%%/*}/package.xml" ]; then printf '%s\n' "$f"; fi
    done || true)
# ...and an UNTRACKED file under a package's test/, its Markdown aside: a new
# fixture the tests read (a URDF, an MJCF, a golden table).
CHANGED_TEST_DATA=$(echo "$CHANGED_UNTRACKED" | awk -F/ "$AWK_ROUTE"'NF >= 3 && test_data($2, $NF)' \
  | while IFS= read -r f; do
      if [ -f "${f%%/*}/package.xml" ]; then printf '%s\n' "$f"; fi
    done || true)
if [ -z "$CHANGED_SRC" ] && [ -z "$CHANGED_SH" ] && [ -z "$CHANGED_DOCS" ] \
   && [ -z "$CHANGED_YAML" ] && [ -z "$CHANGED_META" ] && [ -z "$CHANGED_TESTCFG" ] \
   && [ -z "$CHANGED_PKG_FILES" ] && [ -z "$CHANGED_TEST_DATA" ] \
   && [ -z "$STALE_ARTIFACT_PKGS" ]; then
  say_nothing_to_do "no gate or build reads what changed against the baseline (${VERIFY_BASE:0:8}; untracked scratch?)"
  [ -z "$RUN_MODE" ] || baseline_artifact_stamps
  advance_verify_base
  exit 0
fi

# ── Background agents still writing the checkout ──
#
# Stop fires when the MAIN agent ends its turn; a background subagent does not
# hold that turn open, so the gates would grade a half-written tree.
# Claude Code passes `background_tasks` in the Stop input (checked on 2.1.269:
# running/pending backgrounded tasks, labeled by `type`). While one that edits
# this checkout is in flight, verification is DEFERRED, not skipped: exit 0
# WITHOUT advance_verify_base, so the first turn end with none in flight diffs
# against the same watermark and grades everything they wrote (in a later
# session too: the watermark is in .git/).
#
# Deferring labels are an allowlist: subagent / workflow / teammate. Not shell
# or monitor (a CI watcher would switch the gate off for tens of minutes), and
# not an unknown label: a wrong block costs one turn, a wrong defer an
# unverified tree. A missing field (older Claude Code) yields nothing.
IN_FLIGHT=$(printf '%s' "$INPUT" | jq -r '
  [(.background_tasks // [])[]
   | select(.type == "subagent" or .type == "workflow" or .type == "teammate")
   | select(.status == "running" or .status == "pending")
   | "  - \(.type): \((.description // .id) | tostring | .[0:100])"]
  | .[]' 2>/dev/null || true)
if [ -n "$IN_FLIGHT" ]; then
  {
    echo "Verification deferred (turn NOT blocked) -- background work may still be writing this checkout:"
    sed -n '1,5p' <<<"$IN_FLIGHT"
    n=$(wc -l <<<"$IN_FLIGHT")
    if [ "$n" -gt 5 ]; then echo "  (+$((n - 5)) more)"; fi
    echo "Everything changed since $(git rev-parse --short "$VERIFY_BASE" 2>/dev/null || echo "$VERIFY_BASE") is graded at the first turn end with none in flight."
  } >&2
  exit 0
fi

# ── Nothing changed since the last pass ──
#
# The working tree, byte for byte, is the one this hook last passed at:
# committing a verified change, or a turn that only talked, lands here. The
# tree id does NOT cover the install tree, system packages or other
# repositories (--run checks the install tree: STALE_ARTIFACT_PKGS). Nor does
# it say WHAT the last pass graded (it may be the empty HEAD-fallback pass), so
# a --run that names RTC_VERIFY_BASE is not answered from here.
[ -n "$WORK_TREE" ] || WORK_TREE=$(work_tree_id)
if [ -n "$WORK_TREE" ] && [ -z "${RTC_VERIFY_NO_REUSE:-}" ] && [ -z "$STALE_ARTIFACT_PKGS" ] \
   && [ -z "$BASE_NAMED" ] \
   && [ "$(cat "$PASS_TREE_FILE" 2>/dev/null || true)" = "${VERDICT_TAG}${WORK_TREE}" ]; then
  echo "verify-changes: the working tree is the one this gate last passed at (tree ${WORK_TREE:0:12}) -- nothing re-run." >&2
  [ -z "$RUN_MODE" ] || baseline_artifact_stamps
  advance_verify_base
  log_timing "pass-unchanged" "" ""
  exit 0
fi

# Build/test scope excludes untracked *scratch*, not untracked source. The
# axis is *location*: an untracked file in a package's installed-source dirs
# is part of the tree (a new rtc_base/include/**/foo.hpp must be compiled); a
# note, probe or log anywhere else is not, and must not trigger PROC-3.
CHANGED_SRC_TRACKED=$(echo "$CHANGED_TRACKED" | grep -E '\.(cpp|hpp|h|cc|py)$' || true)
CHANGED_META_TRACKED=$(echo "$CHANGED_TRACKED" | grep -E '(^|/)(CMakeLists\.txt|package\.xml)$' || true)
# Untracked source that belongs to a package (installed_src_dir, AWK_ROUTE),
# so it is built rather than treated as scratch.
CHANGED_SRC_UNTRACKED=$(echo "$CHANGED_UNTRACKED" | awk -F/ "$AWK_ROUTE"'
  NF >= 3 && $NF ~ /\.(cpp|hpp|h|cc|py)$/ && installed_src_dir($2, $1)' || true)
CHANGED_SRC_BUILD=$(printf '%s\n%s\n' "$CHANGED_SRC_TRACKED" "$CHANGED_SRC_UNTRACKED" \
  | grep -v '^[[:space:]]*$' | sort -u || true)
# A shell script in one of those directories routes its package to build/test
# too, tracked or not: a package whose tests ARE shell scripts, or drive one,
# must run them for an edit of the script under test. Kept out of
# CHANGED_SRC_BUILD, which also feeds the pure-format check and knows no
# formatter for a shell script.
CHANGED_SH_BUILD=$(echo "$CHANGED" | awk -F/ "$AWK_ROUTE"'
  NF >= 3 && $NF ~ /\.sh$/ && installed_src_dir($2, $1)' || true)
# Every other TRACKED file of a package routes it to build/test as well, its
# Markdown aside -- the set its verdict is keyed on (pkg_content_key), so an
# edit that voids a verdict also owes a build (a shipped YAML is graded by the
# tests that load it from the install tree). Untracked files keep their
# allowlists: the source dirs above, a new YAML under config/, and every file
# under test/ but Markdown (CHANGED_TEST_DATA; at worst a build for a stray
# note, PROC-3 under rtc_base / rtc_msgs). A new file in any other data
# directory is routed once tracked.
#
# LIMIT, not closed here: the route is the OWNING package. A test in another
# package that reads this file by path (rtc_tools reads integrated_bringup's
# config) is not run, as it is not run for a .cpp change either -- what fans
# out is rtc_base / rtc_msgs (PROC-3) and a changed header (PROC-2, below).
CHANGED_DATA_BUILD=$( {
    echo "$CHANGED_PKG_FILES"
    echo "$CHANGED_UNTRACKED" | awk -F/ "$AWK_ROUTE"'NF >= 3 && config_yaml($2, $NF)'
    echo "$CHANGED_TEST_DATA"
  } || true)

# --- Pure-format fast path detection ---
# Returns 0 if every changed source file is identical to the baseline after
# round-tripping through its formatter. A brand-new or deleted file (no base
# blob) or a missing formatter cannot prove pure-format -> fail closed and run
# full checks.
#
# The formatters, resolved the way format-code.sh resolves them (one pin, one
# search order: lib/formatters.sh), so what that hook wrote is what this one
# accepts.
# shellcheck source=.claude/hooks/lib/formatters.sh
. "$HOOK_DIR/lib/formatters.sh"
[ "${#CLANG_FORMAT_CMD[@]}" -gt 0 ] && HAVE_CLANG_FORMAT=1 || HAVE_CLANG_FORMAT=0
RUFF_BIN=$(find_ruff) || RUFF_BIN=""

# The one formatter dispatch, shared by the pure-format fast path and Phase 5 so
# the two cannot disagree about which files have a formatter or how it is run.
# format_stdin <path>: format stdin the way <path> is formatted, to stdout.
# Non-zero when <path> has no formatter here (type unsupported, tool absent) or
# the run failed; a run is bounded because a cold uvx provisioning can stall.
FORMAT_CALL_TIMEOUT_S=60
format_stdin() {
  case "$1" in
    *.cpp|*.hpp|*.h|*.cc)
      [ "$HAVE_CLANG_FORMAT" -eq 1 ] || return 1
      timeout "$FORMAT_CALL_TIMEOUT_S" "${CLANG_FORMAT_CMD[@]}" --assume-filename="$1" 2>/dev/null
      ;;
    *.py)
      [ -n "$RUFF_BIN" ] || return 1
      timeout "$FORMAT_CALL_TIMEOUT_S" "$RUFF_BIN" format --stdin-filename="$1" - 2>/dev/null
      ;;
    *) return 1 ;;
  esac
}
# format_fix_cmd <path>: the in-place fix, spelled with the binary that graded
# <path> -- a bare `ruff` / `clang-format` may be absent or another version,
# and then the "fixed" file drifts again.
format_fix_cmd() {
  case "$1" in
    *.py) printf '%s format %s' "$RUFF_BIN" "$1" ;;
    *) printf '%s -i %s' "${CLANG_FORMAT_CMD[*]}" "$1" ;;
  esac
}

is_pure_format() {
  [ "$HAVE_CLANG_FORMAT" -eq 1 ] || return 1

  # No source files at all is NOT "pure format" (the loop below succeeds on an
  # empty list), and any non-source change alongside disqualifies it -- else
  # the CMake / docs gates would be skipped with the cosmetic half.
  [ -n "$CHANGED_SRC" ] || return 1
  if [ -n "$CHANGED_DOCS" ] || [ -n "$CHANGED_YAML" ] || [ -n "$CHANGED_META" ] \
     || [ -n "$CHANGED_SH" ] || [ -n "$CHANGED_TESTCFG" ]; then
    return 1
  fi
  # An untracked file has no HEAD blob to compare against.
  if echo "$CHANGED_UNTRACKED" | grep -qE '\.(cpp|hpp|h|cc|py)$'; then
    return 1
  fi

  # Reject any file add/delete/rename — only modifications can be pure-format.
  if git diff --diff-filter=ADRC --name-only "$VERIFY_BASE" 2>/dev/null \
       | grep -qE '\.(cpp|hpp|h|cc|py)$'; then
    return 1
  fi

  # Capture both formatted sides rather than diffing process substitutions: a
  # failing formatter prints NOTHING, `diff <(empty) <(empty)` exits 0, and
  # every file would read as cosmetic. A real source file never formats to
  # empty, so an empty result means "cannot prove pure-format" -> fail CLOSED.
  local f head_fmt work_fmt
  for f in $CHANGED_SRC; do
    [ -f "$f" ] || return 1
    # Round-trip both versions through the formatter with $f as the filename
    # hint so .clang-format / pyproject / file-type rules apply.
    head_fmt=$(git show "$VERIFY_BASE:$f" 2>/dev/null | format_stdin "$f") || return 1
    work_fmt=$(format_stdin "$f" < "$f") || return 1
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
  # Diagnostic: no system clang-format and no uvx forces fail-closed; say so
  # once so fast-path misses are not blind.
  echo "verify-changes: clang-format unavailable (no system binary or uvx);" \
       "pure-format fast path disabled." >&2
fi

WARNINGS=""       # blocking doc/metadata issues (CMake / package.xml)
CHECKLIST=""      # non-blocking reminders (README co-update) -- never exit 2 alone
ARCH_VIOLATIONS=""
CHANGED_PKGS=""

# Identify changed packages from ALL changed files, not just source: a package
# whose only edit is CMakeLists.txt, package.xml or config/*.yaml is changed.
# `while read`, not `for x in $(...)`: a path with a space would split into
# invented filenames.
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
done <<< "$(printf '%s\n%s\n%s\n%s\n%s\n' "$CHANGED_SRC_BUILD" "$CHANGED_META_TRACKED" "$CHANGED_SH_BUILD" \
             "$CHANGED_TESTCFG" "$CHANGED_DATA_BUILD" \
             | grep -v '^[[:space:]]*$' | cut -d'/' -f1 | sort -u)"
# ── PROC-2: what compiles against a changed header ──
#
# A changed header is a change to every package that includes it; routing only
# the OWNING package left dependents compiled against the old layout, untested.
# "Header" is by location: <pkg>/include/ and <pkg>/test/include/ (test
# headers siblings take by source path). A .cpp edit does not fan out: what
# dependents' tests say about a behaviour change is the sensor matrix's
# question (docs/testing.md). "Compiles against" = a build-type dependency in
# package.xml (<depend>, <build_depend>, <build_export_depend>, <test_depend>),
# transitively; <exec_depend> compiles against nothing. No colcon (the suite
# has none). Over-approximates on purpose: a comment-only header edit fans out.
header_dependents() {  # $1 = package -> the packages that compile against it
  local frontier="$1" seen=" $1 " next p f d
  while [ -n "$frontier" ]; do
    next=""
    for p in $frontier; do
      for f in "$PROJECT_DIR"/*/package.xml; do
        [ -f "$f" ] || continue
        grep -qE "<(depend|build_depend|build_export_depend|test_depend)( [^>]*)?>[[:space:]]*${p}[[:space:]]*</" "$f" 2>/dev/null || continue
        d=$(basename "$(dirname "$f")")
        case "$seen" in *" $d "*) continue ;; esac
        seen="${seen}${d} "
        next="${next} ${d}"
      done
    done
    frontier="$next"
  done
  for d in $seen; do
    [ "$d" = "$1" ] || printf '%s\n' "$d"
  done
}
HEADER_PKGS=$(printf '%s\n%s\n%s\n' "$CHANGED_SRC_BUILD" "$CHANGED_PKG_FILES" "$CHANGED_TEST_DATA" \
  | awk -F/ 'NF >= 3 && $NF !~ /\.md$/ && ($2 == "include" || ($2 == "test" && $3 == "include")) { print $1 }' \
  | sort -u || true)
HEADER_FANOUT=""
for pkg_dir in $HEADER_PKGS; do
  [ -f "$pkg_dir/package.xml" ] || continue
  added=""
  for dep in $(header_dependents "$pkg_dir"); do
    case " $BUILD_PKGS " in
      *" $dep "*) ;;
      *) BUILD_PKGS="${BUILD_PKGS} ${dep}"; added="${added} ${dep}" ;;
    esac
  done
  [ -z "$added" ] || HEADER_FANOUT="${HEADER_FANOUT} ${pkg_dir}:[${added# }]"
done
if [ -n "$HEADER_FANOUT" ]; then
  echo "verify-changes: a header changed in a package others compile against (PROC-2) -- routed to build/test with it:${HEADER_FANOUT}" >&2
  # shellcheck disable=SC2046,SC2086  # a space-separated list of names
  BUILD_PKGS=$(printf ' %s' $(printf '%s\n' $BUILD_PKGS | sort -u))
fi
# --run: a package whose installed binaries were rebuilt since its verdict is
# owed a build and a test run whether or not its source changed.
for pkg_dir in $STALE_ARTIFACT_PKGS; do
  case " $BUILD_PKGS " in
    *" $pkg_dir "*) ;;
    *) BUILD_PKGS="${BUILD_PKGS} ${pkg_dir}" ;;
  esac
done
# Back into name order: the appended names came after the sorted ones, and the
# order note below ("built in name order") is read as the order that ran.
if [ -n "$STALE_ARTIFACT_PKGS" ]; then
  # shellcheck disable=SC2046,SC2086  # a space-separated list of names
  BUILD_PKGS=$(printf ' %s' $(printf '%s\n' $BUILD_PKGS | sort -u))
fi

# Emit `name<TAB>exempt<TAB>lineno` for every add_executable() in the CMake
# source on stdin. ARCH-7 diffs target NAMES, not lines.
# `exempt` is 1 when the target opts out: its name matches example_*
# (design-principles.md scopes examples out by name), or an `ARCH-7-exempt`
# comment sits on the add_executable line or ANYWHERE in the contiguous comment
# block directly above it (a marker goes at the TOP of a justification). The
# block ends at the first non-comment line, blank included: a file header
# cannot exempt the first target, nor a marker leak onto the next call.
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
        for (j = i - 1; j >= 1 && exempt == 0; j--) {
          if (line[j] !~ /^[[:space:]]*#/) break   # blank or code ends the block
          if (line[j] ~ /ARCH-7-exempt/) exempt = 1
        }
        printf "%s\t%d\t%d\n", name, exempt, i
      }
    }'
}

# Real file line numbers of the lines a TRACKED file $1 gained since the
# baseline, from the `+c,d` side of each hunk header -- numbers rather than
# the '+' text, so a report can point at a line the agent can open. One per
# line. (An untracked file is new in its entirety; its callers say so.)
diff_added_line_numbers() {
  git diff -U0 "$VERIFY_BASE" -- "$1" 2>/dev/null | awk '
    /^@@/ {
      match($0, /\+[0-9]+(,[0-9]+)?/)
      spec = substr($0, RSTART + 1, RLENGTH - 1)
      split(spec, p, ",")
      count = (p[2] == "" ? 1 : p[2])
      for (i = 0; i < count; i++) print p[1] + i
    }' || true
}

# --- Phase 0: Architecture-fitness grep (ARCH-1, ARCH-4) ---
# Only run if a CHANGED rtc_* file or a changed file in the integration package
# set matched, to bound cost (the set is derived from package.xml below).
# Skipped on pure-format commits — no semantic change can introduce a new
# ARCH violation, and the grep would re-flag pre-existing references on
# any line clang-format happened to reflow.
RTC_TOUCHED=""
INTEGRATION_TOUCHED=""
if [ "$PURE_FORMAT" -eq 0 ]; then
  # ARCH-1 scope is rtc_* production code (include|src|module dirs). Test files
  # under rtc_*/test|tests/ legitimately instantiate concrete robots — a
  # plotter test needs realistic fixtures like `leap_state.csv` — so they are
  # excluded (a test-fixture false positive otherwise).
  RTC_TOUCHED=$(echo "$CHANGED_SRC" | grep -E '^rtc_[a-z_]+/' | grep -vE '(^|/)tests?/' || true)
  # ARCH-4 target set is derived dynamically: any non-rtc_* package whose
  # package.xml depends on at least one rtc_* package. A hardcoded "^ur5e_*/"
  # prefix silently lost coverage after package renames.
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
  # Scope is the ADDED LINES of each changed file: a whole-file grep re-flags
  # pre-existing mentions on any unrelated edit, and cannot tell "you added
  # this" from "this was already here" -- the only question this gate asks. A
  # brand new rtc_* file is screened in full.
  #
  # Patterns: robot identifiers (ur5e/iiwa7/leap/allegro) and a
  # `num_joints = <literal>` assignment — the forms a real hardcode takes. No
  # `6.?dof` patterns: "6-DOF" is SE(3)/task-space dimensionality, not a joint
  # count, and real DOF hardcoding is a numeric literal with no "dof" in it.
  #
  # Negation filter: lines that *forbid* the term ("must NOT test UR5e", "no
  # ur5e-specific code", "robot-agnostic") are the rule itself, not a violation.
  #
  # Known residual false-positive class: robot names in PROSE (docstrings,
  # launch defaults, "e.g. UR5e") that the negation filter does not cover.
  # Triage on a hit, in order: (1) `git show HEAD:<file>` — if the line
  # pre-existed this turn, it is a harness false positive; report it as a
  # harness-pruning signal (CLAUDE.md §Claude Code), do not "fix" working code.
  # (2) If the hit is prose you just wrote in rtc_*, reword robot-neutrally (by
  # scene SHAPE: "16-dof arm-plus-hand") and push the concrete example down to
  # a consumer package's docs/config.
  # Decision (2026-08-30, reconfirmed): keep this ratchet -- no whole-file
  # scope, no prose exemption marker. There is no "robot names in prose are
  # not a violation" exemption: followed, it leaves the line in place, and a
  # blocked run keeps the watermark, so the next turn blocks on it forever.
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    [ -f "$f" ] || continue
    if git ls-files --error-unmatch "$f" >/dev/null 2>&1; then
      ADDED_LINES=$(diff_added_line_numbers "$f")
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
#   runtime via ament_index, never as a build dependency. Added lines only.
# ARCH-7: rtc_* packages do not own the control-framework runtime identity
#   (robot-agnostic nodes and examples exempt: design-principles.md). NOT
#   line-scoped: CMake target lines get rewritten in place (reindent, `if()`
#   wrap), re-presenting old targets as added. The question is whether a
#   target NAME is new, so compare baseline and working-tree name sets.
if [ "$PURE_FORMAT" -eq 0 ] && [ -n "$CHANGED_META" ]; then
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    [ -f "$f" ] || continue
    if git ls-files --error-unmatch "$f" >/dev/null 2>&1; then
      ADDED=$(git diff -U0 "$VERIFY_BASE" -- "$f" 2>/dev/null | grep '^+' | grep -v '^+++' || true)
    else
      ADDED=$(cat "$f" 2>/dev/null || true)
    fi
    [ -z "$ADDED" ] && continue

    # Comments are not code: writing the rule down next to the call is
    # encouraged, and the multi-line join below would match it. Strip `#`
    # (CMake) and `<!-- -->` (package.xml) comments before the ARCH-5 patterns.
    ADDED_CODE=$(echo "$ADDED" | sed -E 's/<!--.*-->//; s/#.*$//')

    # ARCH-5. `ament_target_dependencies(... )` often spans several lines, so
    # check the added block as a whole for the name in any build-time position.
    if echo "$ADDED_CODE" | grep -qE 'find_package[[:space:]]*\([[:space:]]*robot_descriptions'; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - ARCH-5 (robot_descriptions is data-only — find_package is a build-time dep): ${f}\n"
    fi
    # <test_depend> is deliberately absent from this alternation: ARCH-5 allows
    # it for a test that resolves the share dir through ament at runtime, where
    # the dep only orders installation.  See invariants.md §ARCH-5 세부 스펙.
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
        HEAD_EXE=$(git show "$VERIFY_BASE:$f" 2>/dev/null | exe_targets | cut -f1 || true)
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
# Non-blocking (rides the checklist): the bare-SensorDataQoS line filter can
# false-positive across a two-line construct. Skipped on pure-format commits.
QOS_VIOLATIONS=""
if [ "$PURE_FORMAT" -eq 0 ]; then
  QOS_SRC=$(echo "$CHANGED_SRC" | grep -vE '(^|/)tests?/|(^|/)test_[^/]*\.py$|_test\.(cpp|cc|hpp|h)$' || true)
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    [ -f "$f" ] || continue
    # Dropped: a line with the `ARCH-6-exempt` marker (a recorded exception,
    # invariants.md ARCH-6 세부 스펙), and a whole-line comment -- it describes a
    # QoS (often another node's), it does not set one. Whole-line only: a
    # trailing comment sits on a line of code. `#` opens a comment in Python
    # and a directive in C/C++, so the comment form is read by extension.
    case "$f" in
      *.py) QOS_COMMENT='^[0-9]+:[[:space:]]*#' ;;
      *) QOS_COMMENT='^[0-9]+:[[:space:]]*(//|/\*|\*)' ;;
    esac
    HITS=$(grep -nE 'rclcpp::QoS[({]([0-9]{2,}|[02-9])[)}]|keep_last\(([0-9]{2,}|[02-9])\)|depth[[:space:]]*=[[:space:]]*([0-9]{2,}|[02-9])' "$f" 2>/dev/null | grep -v 'ARCH-6-exempt' | grep -vE "$QOS_COMMENT" || true)
    BARE=$(grep -nE 'SensorDataQoS\(\)' "$f" 2>/dev/null | grep -v 'keep_last' | grep -v 'ARCH-6-exempt' | grep -vE "$QOS_COMMENT" || true)
    ALL=$(printf '%s\n%s' "$HITS" "$BARE" | grep -vE '^[[:space:]]*$' || true)
    if [ -n "$ALL" ]; then
      QOS_VIOLATIONS="${QOS_VIOLATIONS}  - ARCH-6 (topic QoS depth != 1): ${f}\n${ALL}\n"
    fi
  done <<< "$QOS_SRC"
fi

# --- Phase 0c: PROC-8 — the shared test wait helper stays sleep-only ---
# rtc_base/test/include/rtc_base/testing/ holds sleep-poll waits four packages
# share. Pumping an executor there conflates two primitives: ur5e_bt_coordinator's
# fixture owns the sole spinner thread (a second driver of one executor is UB
# in rclcpp), and udp_hand_driver's aux-lane test proves a timer sits OUTSIDE
# the default callback group — pumping that executor would service it and the
# assertion would prove nothing (#356).
# Helper headers only: the caller half of PROC-8 stays prose (no greppable
# form that is not mostly legitimate). NOT skipped on PURE_FORMAT: a real spin
# call must not ride in under a format commit.
WAIT_HELPERS=$(echo "$CHANGED" | grep -E '^rtc_base/test/include/rtc_base/testing/.*\.hpp$' || true)
if [ -n "$WAIT_HELPERS" ]; then
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    [ -f "$f" ] || continue
    # Comments are stripped first so the header may keep explaining WHY it must
    # not spin without tripping the gate that enforces it.
    HITS=$(sed 's://.*::' "$f" 2>/dev/null \
            | grep -nE '\b(spin_some|spin_once|spin_until_future_complete|spin_node_some|spin_node_once|add_node)\b' \
            || true)
    if [ -n "$HITS" ]; then
      ARCH_VIOLATIONS="${ARCH_VIOLATIONS}  - PROC-8 (shared test wait helper must stay sleep-only — it must never drive an executor): ${f}\n${HITS}\n"
    fi
  done <<< "$WAIT_HELPERS"
fi

# --- Phase 1: Doc/metadata co-update check ---
# Skipped on pure-format commits: no semantic delta to mirror in READMEs, and
# the CMake/package.xml triggers cannot fire (pure-format excludes file adds).
if [ "$PURE_FORMAT" -eq 0 ]; then
# Hoisted out of the per-package loop: these two git queries are repo-wide.
# --no-renames: a file moved into a new directory is an add there and a delete
# of the old one, which is what the directory trigger asks about.
STRUCT_AD=$(git diff --diff-filter=AD --no-renames --name-status "$VERIFY_BASE" 2>/dev/null || true)
STRUCT_ADDED=$(printf '%s\n%s\n' "$(echo "$STRUCT_AD" | sed -n 's/^A\t//p')" "$CHANGED_UNTRACKED" \
                 | grep -v '^[[:space:]]*$' | sort -u || true)
STRUCT_DELETED=$(echo "$STRUCT_AD" | sed -n 's/^D\t//p')
ADDED_A=$(git diff --diff-filter=A --name-only "$VERIFY_BASE" 2>/dev/null || true)
for pkg_dir in $CHANGED_PKGS; do
  # README.md co-update -- NON-BLOCKING checklist, and only for public-surface
  # changes: a public header (include/), launch/ or config/, a DIRECTORY (at
  # most two levels below the package) appearing or disappearing, or package.xml
  # (deps/exec surface). A src/-only edit carries no doc-visible delta, and a
  # file add/delete inside an existing directory does not either: a file-level
  # trigger made per-file README trees the habit (#869 D4) -- file roles live in
  # header comments. A doc kept beside the headers
  # (rtc_math/include/rtc_math/se3/README.md) is not surface.
  # repo_scripts/config/*_budget.yaml are the docs gate's own data (Phase 1b
  # grades them), not user-facing config: a budget raise is not a README delta.
  PKG_PUBLIC=$(echo "$CHANGED" | grep -E "^${pkg_dir}/(include|launch|config)/" \
                 | grep -vE '\.md$|^repo_scripts/config/[a-z_]+_budget\.yaml$' || true)
  # Directory = the file's dir cut to <pkg>/a/ or <pkg>/a/b/ (package-root files
  # have none). New: nothing under it at the baseline. Removed: nothing tracked
  # or untracked left under it. One git call per candidate directory, not file.
  PKG_STRUCT=""
  for sd in $(echo "$STRUCT_ADDED" | awk -F/ -v p="$pkg_dir" '$1 == p && NF >= 3 && $NF !~ /\.md$/ {
                 d = $1 "/" $2 "/"; if (NF >= 4) d = d $3 "/"; print d }' | sort -u); do
    if [ -z "$(git ls-tree -r --name-only "$VERIFY_BASE" -- "$sd" 2>/dev/null | head -n 1)" ]; then
      PKG_STRUCT="${PKG_STRUCT}${sd}\n"
    fi
  done
  for sd in $(echo "$STRUCT_DELETED" | awk -F/ -v p="$pkg_dir" '$1 == p && NF >= 3 && $NF !~ /\.md$/ {
                 d = $1 "/" $2 "/"; if (NF >= 4) d = d $3 "/"; print d }' | sort -u); do
    if [ -z "$(git ls-files -c -o --exclude-standard -- "$sd" 2>/dev/null | head -n 1)" ]; then
      PKG_STRUCT="${PKG_STRUCT}${sd}\n"
    fi
  done
  PKG_PKGXML=$(echo "$CHANGED" | grep -E "^${pkg_dir}/package.xml$" || true)
  if [ -n "$PKG_PUBLIC" ] || [ -n "$PKG_STRUCT" ] || [ -n "$PKG_PKGXML" ]; then
    if ! echo "$CHANGED" | grep -q "^${pkg_dir}/README.md$"; then
      CHECKLIST="${CHECKLIST}  - ${pkg_dir}: public surface changed (header / launch / config / directory add-del / dep) — confirm README.md's user-facing parts (interface · config · directory) reflect it, or note in your report why no doc change is needed\n"
    fi
  fi

  # New .cpp files possibly missing from CMakeLists.txt. Staged adds AND
  # untracked files: a source file written without `git add` is the common case.
  NEW_SRC=$(printf '%s\n%s\n' "$ADDED_A" "$CHANGED_UNTRACKED" \
              | grep "^${pkg_dir}/src/.*\.cpp$" | grep -v test | sort -u || true)
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    bname=$(basename "$f")
    # Match the basename as a whole path token, not a bare substring: an
    # unanchored `grep -qF hand.cpp` matches inside `left_hand.cpp`. Require a
    # non-filename char (or line edge) on both sides; escape regex
    # metacharacters in the basename first.
    bname_re=$(printf '%s' "$bname" | sed 's/[^A-Za-z0-9_]/\\&/g')
    if ! grep -qE "(^|[^A-Za-z0-9_])${bname_re}([^A-Za-z0-9_]|$)" "${pkg_dir}/CMakeLists.txt" 2>/dev/null; then
      WARNINGS="${WARNINGS}  - ${pkg_dir}: new file ${bname} not found in CMakeLists.txt\n"
    fi
  done <<< "$NEW_SRC"

  # package.xml co-update for new find_package() in CMakeLists.txt
  if echo "$CHANGED" | grep -q "^${pkg_dir}/CMakeLists.txt$"; then
    NEW_FIND=$(git diff "$VERIFY_BASE" -- "${pkg_dir}/CMakeLists.txt" 2>/dev/null \
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
# Scoped to the CHANGED files, never the whole corpus (CI scans everything): a
# defect in an untouched -- or gitignored, invisible -- file would otherwise
# block every turn with no way out.
DOC_FAILURES=""
DOC_FILES=()
while IFS= read -r d; do
  [ -n "$d" ] && [ -f "$d" ] && DOC_FILES+=("$d")
done <<< "$CHANGED_DOCS"
# Resolved relative to this hook, not to PROJECT_DIR: the script ships in the
# same repository as the hook, so this works when the two point at different
# trees (as the routing tests do).
VALIDATE_DOCS="$HOOK_DIR/../../repo_scripts/scripts/validate_docs.py"
[ -f "$VALIDATE_DOCS" ] || VALIDATE_DOCS="$PROJECT_DIR/repo_scripts/scripts/validate_docs.py"
if [ "${#DOC_FILES[@]}" -gt 0 ]; then
  if [ -f "$VALIDATE_DOCS" ]; then
    # One invocation, and the verdict comes from the exit status (an `xargs -0`
    # split plus a grep for "clean" once discarded every other batch's findings).
    DOC_RC=0
    DOC_OUT=$(python3 "$VALIDATE_DOCS" --files "${DOC_FILES[@]}" 2>&1) || DOC_RC=$?
    if [ "$DOC_RC" -ne 0 ]; then
      # Scope findings in a TRACKED doc to its added lines, mirroring ARCH-1:
      # touching a .md must not make its pre-existing defects blocking. An
      # untracked doc is new in its entirety, so all of its lines are in scope.
      DOC_ALLOW=$(mktemp)
      for d in "${DOC_FILES[@]}"; do
        if git ls-files --error-unmatch "$d" >/dev/null 2>&1; then
          diff_added_line_numbers "$d" | awk -v F="$d" '{ print F ":" $0 }' >> "$DOC_ALLOW" || true
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
        # D12 line/byte caps are whole-file budgets reported at a nominal line
        # (1, or cap+1) that the edit which blew the budget almost never adds,
        # so they bypass the narrowing -- filtered, a constitution could grow
        # past its cap here and only CI would say so.
        {
          if ($0 ~ /: \[D12\] [0-9]+ (bytes|lines) > [0-9]+ /) {
            print "  - " $0
          } else if (match($0, /^[^:]+:[0-9]+: \[D[0-9]+\] /)) {
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

# A constitution renumbering breaks section refs in files this change never
# touched -- and bare refs on unchanged lines of the constitution itself --
# which the added-line scope above cannot see. So when a constitution's
# numbered-heading set moved, resolve every section ref in the tracked corpus,
# unnarrowed. D13 only (`--section-refs`): other pre-existing debt in untouched
# files still cannot block the turn.
constitution_sections() {  # stdin: markdown -> its numbered section ids, sorted
  sed -nE 's/^#{2,3}[[:space:]]+([0-9]+(\.[0-9]+)?)\.?[[:space:]].*/\1/p' | sort
}
for c in AGENTS.md CLAUDE.md; do
  echo "$CHANGED_TRACKED" | grep -qx "$c" || continue
  [ -f "$c" ] && [ -f "$VALIDATE_DOCS" ] || continue
  base_secs=$(git show "$VERIFY_BASE:$c" 2>/dev/null | constitution_sections || true)
  [ "$base_secs" = "$(constitution_sections < "$c")" ] && continue
  SECREF_RC=0
  SECREF_OUT=$(python3 "$VALIDATE_DOCS" --section-refs 2>&1) || SECREF_RC=$?
  if [ "$SECREF_RC" -ne 0 ]; then
    SECREF_FAILURES=$(echo "$SECREF_OUT" | grep -vE '^$|finding\(s\) across' | sed 's/^/  - /')
    DOC_FAILURES=$(printf '%s\n  - (%s numbered headings changed: section refs resolved corpus-wide)\n%s\n' \
      "$DOC_FAILURES" "$c" "$SECREF_FAILURES" | awk 'NF && !seen[$0]++')
  fi
  break
done

YAML_FAILURES=""
if [ -n "$CHANGED_YAML" ]; then
  # config/**/*.yaml (device backends, controller gains, robot profiles): this
  # only proves the file parses -- schema is out of scope -- but a YAML that
  # does not load is a launch-time failure no test would catch either.
  # Fail OPEN when PyYAML is missing, matching clang-format and shellcheck: a
  # hook run without the venv must not wedge every turn that touches a YAML.
  if python3 -c 'import yaml' >/dev/null 2>&1; then
    while IFS= read -r yf; do
      [ -n "$yf" ] && [ -f "$yf" ] || continue
      # safe_load_all: a multi-document file is legal YAML. The verdict is the
      # exit STATUS, not "stderr is non-empty" (interpreter warnings).
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

# --- Phase 1c: path-scoped rules must be able to fire ---
# A .claude/rules/*.md whose `paths:` globs match nothing never loads, silently
# (#213), so it blocks. The other half of the pair -- did the rule ACTUALLY
# load -- is the InstructionsLoaded hook (.claude/hooks/log-instructions-loaded.sh),
# whose log is the ground truth (this reimplements the runtime matcher).
# Scoped to the changed rule files, like every other Phase 1 sensor.
RULES_FAILURES=""
CHANGED_RULES=$(echo "$CHANGED_DOCS" | grep -E '(^|/)\.claude/rules/.*\.md$' || true)
if [ -n "$CHANGED_RULES" ]; then
  RULE_FILES=()
  while IFS= read -r rf; do
    [ -n "$rf" ] && [ -f "$rf" ] && RULE_FILES+=("$rf")
  done <<< "$CHANGED_RULES"
  if [ "${#RULE_FILES[@]}" -gt 0 ]; then
    # Resolved relative to this hook first, like validate_docs.py above, so the
    # routing tests can point PROJECT_DIR at a fixture that has no scripts.
    VALIDATE_RULES="$HOOK_DIR/../../repo_scripts/scripts/validate_claude_rules.py"
    [ -f "$VALIDATE_RULES" ] || VALIDATE_RULES="$PROJECT_DIR/repo_scripts/scripts/validate_claude_rules.py"
    if [ -f "$VALIDATE_RULES" ]; then
      RULES_RC=0
      RULES_OUT=$(python3 "$VALIDATE_RULES" --root "$PROJECT_DIR" --files "${RULE_FILES[@]}" 2>&1) \
        || RULES_RC=$?
      if [ "$RULES_RC" -ne 0 ]; then
        RULES_FAILURES=$(echo "$RULES_OUT" | grep -E '\[R[0-9]+\]' || echo "$RULES_OUT")
      fi
    else
      echo "verify-changes: validate_claude_rules.py not found; rule glob gate skipped." >&2
    fi
  fi
fi

# --- Phase 1d: test gates (CI's domain, fixture and registration checks) ---
# validate_test_domains.py (#401: one ROS_DOMAIN_ID per package, every
# participant-opening test claims one), validate_test_fixtures.py (#454: no
# fixture resolves a package outside the repo and deps.repos) and
# validate_test_registration.py (a test file no CMakeLists.txt call names is
# never run, silently) judge OTHER packages' files, so only CI caught them.
#
# Whole-repo verdicts: a collision is between two packages, and the untouched
# one is half of it; CI keeps main clean, so inherited debt cannot block an
# unrelated turn. ~0.5 s each: the trigger is only a relevance filter.
#
# Resolved in PROJECT_DIR only, not next to the hook as Phase 1b/1c do: these
# scripts judge the tree they live in (repo root = their parents[2]). The
# routing tests rely on this: they copy the scripts into a fixture.
TESTGATE_FAILURES=""
CHANGED_TESTGATE=$(echo "$CHANGED" | grep -E '(^|/)(CMakeLists\.txt|conftest\.py|colcon\.pkg)$|(^|/)test(ing)?/' || true)
if [ -n "$CHANGED_TESTGATE" ]; then
  for gate in validate_test_domains validate_test_fixtures validate_test_registration; do
    gate_py="$PROJECT_DIR/repo_scripts/scripts/${gate}.py"
    if [ ! -f "$gate_py" ]; then
      echo "verify-changes: ${gate}.py not found; that test gate skipped." >&2
      continue
    fi
    GATE_RC=0
    GATE_OUT=$(python3 "$gate_py" 2>&1) || GATE_RC=$?
    if [ "$GATE_RC" -ne 0 ]; then
      TESTGATE_FAILURES="${TESTGATE_FAILURES}  - repo_scripts/scripts/${gate}.py (exit ${GATE_RC}):\n$(echo "$GATE_OUT" | sed 's/^/      /')\n"
    fi
  done
fi

# --- Phase 5: formatter drift introduced by the change ---
# format-code.sh (PostToolUse) formats only what Edit / Write touch; a file
# written through Bash had nothing else to catch it (CI runs no formatter).
#
# Blocks only on drift the change INTRODUCED: the working file is not a formatter
# fixed point AND its $VERIFY_BASE blob either does not exist or was one (a
# rename is graded whole). Scope is CHANGED_SRC_BUILD. Fails OPEN when the
# formatter is missing or prints nothing: style, not correctness.
# Compared byte-for-byte through files: command substitution strips trailing
# newlines, hiding a missing final newline or trailing blank lines.
#
# At the turn end, once $SECONDS passes RTC_VERIFY_FORMAT_DEADLINE_S the rest
# are listed as ungraded (non-blocking) rather than risk a Stop-budget SIGKILL
# with no report. --run has no budget, so it grades them all unless the
# variable is set.
#
# Placed before Phase 2, out of number order: under --run, drift means nothing
# is built (see the build decision below). It blocks the run whatever the build
# answers, and the reformat changes the package content every verdict is keyed
# on, so a build beside it was thrown away (#880: 150 s for one file).
FORMAT_FAILURES=""
FORMAT_UNGRADED=""
if [ -n "$RUN_MODE" ]; then
  FORMAT_DEADLINE_S="${RTC_VERIFY_FORMAT_DEADLINE_S:-2147483647}"
else
  FORMAT_DEADLINE_S="${RTC_VERIFY_FORMAT_DEADLINE_S:-480}"
fi
FMT_OUT=$(mktemp)
FMT_BASE=$(mktemp)
while IFS= read -r f; do
  [ -n "$f" ] && [ -f "$f" ] || continue
  if [ "$SECONDS" -ge "$FORMAT_DEADLINE_S" ]; then
    FORMAT_UNGRADED="${FORMAT_UNGRADED} $f"
    continue
  fi
  format_stdin "$f" < "$f" > "$FMT_OUT" || continue
  [ -s "$FMT_OUT" ] || continue
  cmp -s "$FMT_OUT" "$f" && continue
  if git cat-file -e "$VERIFY_BASE:$f" 2>/dev/null; then
    git show "$VERIFY_BASE:$f" > "$FMT_BASE" 2>/dev/null || continue
    format_stdin "$f" < "$FMT_BASE" > "$FMT_OUT" || continue
    cmp -s "$FMT_OUT" "$FMT_BASE" || continue
  fi
  FORMAT_FAILURES="${FORMAT_FAILURES}  - $f: formatter would rewrite it -- run: $(format_fix_cmd "$f")\n"
done <<< "$CHANGED_SRC_BUILD"
rm -f "$FMT_OUT" "$FMT_BASE"
if [ -n "$FORMAT_UNGRADED" ]; then
  CHECKLIST="${CHECKLIST}  - formatter drift NOT graded (${SECONDS}s of the Stop budget spent before Phase 5):${FORMAT_UNGRADED} -- run the formatter on these yourself\n"
fi

# --- Phase 2: Build + test, with PROC-3 fallback for rtc_base / rtc_msgs ---
# Test seams, never set in normal operation: RTC_VERIFY_SKIP_BUILD lets
# repo_scripts/test/test_verify_changes.sh exercise the routing without a
# colcon workspace; RTC_VERIFY_BUILD_CMD replaces the build call (an executable
# taking build.sh's arguments), so the suite reaches the build branches below
# with a stub that exits with a chosen code.
TEST_FAILURES=""
BUILD_CMD="${RTC_VERIFY_BUILD_CMD:-./build.sh}"

# Run the build, keeping BOTH the exit code and the output, so a bound-kill
# (124) and a compile error are told apart (#435).
# Sets BUILD_RC and BUILD_LOG; callers must rm the log.
# --tests is not optional: build.sh skips tests by default, and `colcon test`
# on a package built without them reports "0 tests, 0 failures" -- a pass to
# every branch below.
run_build() {  # $1 = timeout seconds, rest = args for the build command
  local secs="$1"; shift
  BUILD_LOG=$(mktemp)
  BUILD_RC=0
  timeout "$secs" "$BUILD_CMD" "$@" --tests >"$BUILD_LOG" 2>&1 || BUILD_RC=$?
}

# ── Build order ──
#
# Packages are built AND tested one at a time, so in name order (`sort -u`) a
# package is tested before a dependency of it is rebuilt -- against the old
# installed library with the new headers (a symlink install serves headers
# from the source): a false red (#631) or a false GREEN that gets recorded.
# So the order is colcon's own (`colcon list --topological-order` over this
# repository; 0.2 s, no ROS environment needed). A package that list does not
# name keeps its place after the ones it does. Without a usable list the order
# stays the name order and the run SAYS so. One package costs no call.
#
# The query runs from the workspace root with colcon's log off: every colcon
# verb writes log/ into its cwd, here the repository (AGENTS.md §9.1).
colcon_list() {  # args for `colcon list`; prints names, fails when it cannot
  ( cd "$WORKSPACE" 2>/dev/null && COLCON_LOG_PATH=/dev/null timeout 60 colcon list --names-only "$@" 2>/dev/null )
}
order_build_pkgs() {  # reorders BUILD_PKGS in place
  local topo p ordered=""
  # shellcheck disable=SC2086  # BUILD_PKGS is a space-separated list of names
  set -- $BUILD_PKGS
  [ $# -ge 2 ] || return 0
  topo=$(colcon_list --topological-order --base-paths "$PROJECT_DIR") || topo=""
  if [ -z "$topo" ]; then
    echo "verify-changes: could not order [${BUILD_PKGS# }] by dependency ('colcon list' gave nothing) -- built in name order. If one of them depends on another, its tests run against the dependency as it was installed BEFORE this run: run --run again once it passes." >&2
    return 0
  fi
  for p in $topo; do
    case " $BUILD_PKGS " in *" $p "*) ordered="${ordered} ${p}" ;; esac
  done
  for p in $BUILD_PKGS; do
    case " $ordered " in *" $p "*) ;; *) ordered="${ordered} ${p}" ;; esac
  done
  BUILD_PKGS="$ordered"
}

# A package whose build did not finish leaves its OLD binaries installed, and a
# dependent tested against them would record a false verdict. So the packages
# above a failed one are held back: not built, not tested, reported, no
# verdict. BUILD_HELD is "<pkg>:<the package whose build failed>" entries.
# Without a usable `colcon list` nothing is held (the order note says so).
BUILD_HELD=""
hold_dependents_of() {  # $1 = package whose build did not finish
  local above d
  above=$(colcon_list --base-paths "$PROJECT_DIR" --packages-above "$1") || above=""
  for d in $above; do
    [ "$d" = "$1" ] && continue
    case " $BUILD_HELD " in *" $d:"*) ;; *) BUILD_HELD="${BUILD_HELD} ${d}:$1" ;; esac
  done
}
build_held_by() {  # $1 = package -> the failed dependency holding it, or nothing
  local e
  for e in $BUILD_HELD; do
    [ "${e%%:*}" = "$1" ] && { printf '%s' "${e#*:}"; return 0; }
  done
  return 0
}

# Names (of "$@") whose build tree was configured WITHOUT tests. The verdict
# must not rest on every build path asking for tests: with BUILD_TESTING=OFF
# `colcon test-result` answers "0 tests, 0 failures" or from stale result
# files, both read as a pass. Read after the build, before the tests. No
# CMakeCache.txt (ament_python, or not built this run) is not reported.
pkgs_built_without_tests() {
  local p cache
  for p in "$@"; do
    cache="$WORKSPACE/build/$p/CMakeCache.txt"
    [ -f "$cache" ] || continue
    if grep -qE '^BUILD_TESTING:[A-Z]*=(OFF|0|FALSE|NO)$' "$cache"; then
      printf '%s ' "$p"
    fi
  done
}

# The same question for every package of this repo (a top-level directory with
# a package.xml; the directory name is the package name throughout this repo).
repo_pkgs_built_without_tests() {
  local d
  for d in "$PROJECT_DIR"/*/; do
    [ -f "${d}package.xml" ] || continue
    pkgs_built_without_tests "$(basename "$d")"
  done
}

# Run `colcon test` over "$@" (package names; none = every package of the
# workspace). Sets TEST_RC, TEST_LOG (colcon's console output) and TEST_MARKER
# (a file older than every result this run writes); callers rm both.
# --return-code-on-test-failure is the verdict: 0 = every selected package ran
# and none failed, 1 = a test failed. Without it colcon exits 0 either way
# (reading result files instead once recorded every failing package green).
# Suite cases 66-68 run this line against a `colcon` that refuses bad options.
run_colcon_test() {  # $1 = timeout seconds, rest = package names
  local secs="$1" select=""
  shift
  [ $# -gt 0 ] && select="--packages-select $*"
  TEST_MARKER=$(mktemp)
  TEST_LOG=$(mktemp)
  TEST_RC=0
  timeout "$secs" bash -c "cd '$WORKSPACE' && colcon test $select --return-code-on-test-failure --event-handlers console_direct+ 2>&1" >"$TEST_LOG" || TEST_RC=$?
}

# What that run says, in one word, left in TEST_STATUS: green | red | timeout |
# launch | short. TEST_FINISHED is the number of packages colcon counted.
#
# Exit 0 alone is not green: `colcon test --packages-select <unknown name>`
# tests nothing and exits 0 ("Summary: 0 packages finished"), and a package
# that never started is not counted either. So a pass needs colcon's summary
# line counting as many packages as were asked for ($1; empty = any number
# above zero, for the workspace-wide run). No summary line reads as "short",
# which blocks instead of guessing.
classify_colcon_test() {  # $1 = number of packages asked for, or empty
  TEST_FINISHED=$(sed -n 's/^Summary: \([0-9][0-9]*\) packages\{0,1\} finished.*/\1/p' "$TEST_LOG" | tail -n 1)
  TEST_FINISHED="${TEST_FINISHED:-0}"
  if [ "$TEST_RC" -eq 124 ]; then
    TEST_STATUS=timeout
  elif [ "$TEST_RC" -ge 125 ]; then
    TEST_STATUS=launch
  elif [ "$TEST_RC" -ne 0 ]; then
    TEST_STATUS=red
  elif [ -n "${1:-}" ] && [ "$TEST_FINISHED" -ne "$1" ]; then
    TEST_STATUS=short
  elif [ "$TEST_FINISHED" -eq 0 ]; then
    TEST_STATUS=short
  else
    TEST_STATUS=green
  fi
}

# fresh_test_failures, indented for the `echo -e` report and capped: a broken
# fixture can fail hundreds of cases, and the report is read by an agent.
failed_tests_report() {
  fresh_test_failures "$@" | sed -n '1,40p' | sed -e 's/\\/\\\\/g' -e 's/^/      /'
}

# The result files under "$@" (bases relative to the workspace: build/<pkg>, or
# build) that report an error or a failure AND were written by this run, with
# the failing test names under each. For the report only -- the verdict is the
# exit code above. "Written by this run" matters: ctest keeps one
# Testing/<stamp>/ per run and nothing removes them, so a failure fixed weeks
# ago is still listed by `colcon test-result`.
fresh_test_failures() {
  local base line keep=""
  for base in "$@"; do
    [ -d "$WORKSPACE/$base" ] || continue
    # --verbose prints, per file: the file line, "- <test name>" per failing
    # test, and each failure's message. The message bodies are left out.
    while IFS= read -r line; do
      case "$line" in
        build/*": "*)
          keep=""
          # "not older than the marker", not "newer": two files written in
          # the same clock tick carry the same timestamp.
          if [ -f "$WORKSPACE/${line%%: *}" ] && ! [ "$TEST_MARKER" -nt "$WORKSPACE/${line%%: *}" ]; then
            keep=1
            printf '%s\n' "$line"
          fi
          ;;
        "- "*)
          if [ -n "$keep" ]; then printf '  %s\n' "$line"; fi
          ;;
      esac
    done < <(cd "$WORKSPACE" && colcon test-result --test-result-base "$base" --verbose 2>/dev/null || true)
  done
}

# The result files under "$@" (bases as above) that this run wrote and in which
# every case was SKIPPED: "tests" > 0 and "skipped" >= "tests" on the first
# <testsuite(s)> tag (pytest's xunit; gtest carries no "skipped"). Such a file
# passes `colcon test` while grading nothing (its module skipped itself for a
# dependency colcon's interpreter lacks). Report only.
skipped_result_files() {
  local base file tag tests skipped
  for base in "$@"; do
    [ -d "$WORKSPACE/$base" ] || continue
    while IFS= read -r file; do
      [ "$TEST_MARKER" -nt "$file" ] && continue
      tag=$(grep -m1 -oE '<testsuites? [^>]*' "$file" 2>/dev/null) || continue
      tests=$(printf '%s' "$tag" | sed -n 's/.* tests="\([0-9][0-9]*\)".*/\1/p')
      skipped=$(printf '%s' "$tag" | sed -n 's/.* skipped="\([0-9][0-9]*\)".*/\1/p')
      [ -n "$tests" ] && [ -n "$skipped" ] || continue
      [ "$tests" -gt 0 ] && [ "$skipped" -ge "$tests" ] || continue
      printf '      %s (%s cases, every one skipped)\n' "${file#"$WORKSPACE"/}" "$tests"
    done < <(find "$WORKSPACE/$base" -name '*.xml' -type f 2>/dev/null | sort)
  done
}
SKIPPED_WHOLE=""

# Last lines of a failed build, indented for the report. Backslashes doubled:
# `echo -e` would eat a "\n" inside a compiler-quoted string literal.
build_log_tail() {
  tail -n 15 "$BUILD_LOG" | sed -e 's/\\/\\\\/g' -e 's/^/      /'
}

# Whether a 124 was the code's fault or a lost CPU race: loadavg plus a live
# build/compiler count at the moment of the kill.
build_contention_evidence() {
  local load busy
  load=$(cut -d' ' -f1-3 /proc/loadavg 2>/dev/null || echo "unavailable")
  if command -v pgrep >/dev/null 2>&1; then
    # Match process NAMES (-x), not command lines (-f): -f also matches any
    # ancestor shell whose command line holds the pattern (e.g. a hand
    # invocation), inflating under the very conditions it measures. Counted
    # after the kill, so this build's dying children can be in it -- an
    # indicator of contention, not an exact count of rivals.
    busy=$(pgrep -c -x '(colcon|cmake|ninja|make|cc1plus|cc1|ld)' 2>/dev/null || true)
  else
    busy="?"
  fi
  printf 'loadavg %s, %s build/compiler processes running at kill time' "$load" "${busy:-0}"
}

# A build already writing this workspace -- looked for BEFORE starting ours.
#
# A background SHELL task does not defer; its `./build.sh` beside ours races
# for CPU and writes the same build/ and install/ trees. When one is found
# nothing is built: the turn end blocks naming it (a wrong block costs a turn,
# a wrong defer an unverified tree) and --run refuses.
#
# Matched by process NAME (-x, as above), then by cwd: build.sh cds into the
# workspace, and the AGENTS.md §9.1 form runs colcon from its root. A colcon
# aimed here from elsewhere with absolute --build-base/--install-base is NOT
# seen, and `bash build.sh` is named "bash" -- its colcon is seen once it
# starts. Prints "<pid>: <cmdline>".
workspace_build_rivals() {
  command -v pgrep >/dev/null 2>&1 || return 0
  local ws pid cwd cmd
  ws=$(cd "$WORKSPACE" 2>/dev/null && pwd -P) || return 0
  for pid in $(pgrep -x '(colcon|build\.sh)' 2>/dev/null || true); do
    cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null) || continue
    [ "$cwd" = "$ws" ] || continue
    cmd=$(proc_cmdline "$pid") || cmd="?"
    printf '%s: %s\n' "$pid" "$cmd"
  done
}
# A simulator running from this workspace -- looked for with the build rivals.
#
# A host busy with colcon slows the MuJoCo sim below real time (RTF 0.29-0.74
# measured) and its input goes stale -- failures of the rig, not the code.
# DEFER, not block, unlike a rival build: a sim does not end on its own (the
# user may hold one open), and a block would push the agent to stop a sim that
# is not its own. The verdict is not asked for, the other gates still run (and
# still block), and on a pass the watermark is NOT advanced -- the
# background-agent deferral's rule. --run refuses outright.
#
# Matched by the kernel's 15-character process name, then by path: the node's
# executable (/proc/<pid>/exe, physical) under the physical workspace, or any
# argv entry under the workspace's logical (AMENT_PREFIX_PATH, via a symlink)
# or physical path. Prints "<pid>: <cmdline>".
SIM_PROCESS_NAMES='mujoco_simulato'
workspace_sim_rivals() {
  command -v pgrep >/dev/null 2>&1 || return 0
  local ws_phys ws_logic pid cmd exe
  ws_phys=$(cd "$WORKSPACE" 2>/dev/null && pwd -P) || return 0
  ws_logic=${WORKSPACE%/}
  for pid in $(pgrep -x "($SIM_PROCESS_NAMES)" 2>/dev/null || true); do
    cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null) || continue
    exe=$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)
    case "$exe" in
      "$ws_phys"/*) ;;
      *)
        case " $cmd" in
          *" $ws_phys/"* | *" $ws_logic/"*) ;;
          *) continue ;;
        esac
        ;;
    esac
    printf '%s: %s\n' "$pid" "$(cut -c1-120 <<<"$cmd")"
  done
}
# A measurement holding the host -- looked for with the sim rivals.
#
# An evaluation that launches one simulator per unit has none running between
# units, and a turn ending on "unit N done" ends in that gap. So the driver
# writes "<pid> [<start time>]" lines to <workspace>/.rtc-verify-hold (start
# time = field 22 of /proc/<pid>/stat). While a listed process is alive,
# build/test is deferred on the simulator's terms (other gates run, watermark
# kept). A dead pid, or a reused one (start time differs), holds nothing.
# Prints "<pid>: <cmdline>".
HOLD_FILE="$WORKSPACE/.rtc-verify-hold"
workspace_holds() {
  [ -f "$HOLD_FILE" ] || return 0
  local pid start now cmd
  # `read` fails on a last line that has no newline and still fills the
  # variables: a driver that wrote the file with printf holds the host too.
  while read -r pid start _ || [ -n "${pid:-}" ]; do
    case "$pid" in '' | *[!0-9]*) continue ;; esac
    [ -r "/proc/$pid/stat" ] || continue
    if [ -n "${start:-}" ]; then
      now=$(proc_start_time "$pid")
      [ "$now" = "$start" ] || continue
    fi
    cmd=$(proc_cmdline "$pid") || cmd="?"
    printf '%s: %s\n' "$pid" "${cmd% }"
  done < "$HOLD_FILE"
}

# ── Package verdict reuse ──
#
# A package that built and tested green is not built and tested again while
# the packages are what they were graded as. The key is the content of EVERY
# package directory of the working tree (every file, by blob), so an edit in
# any package re-grades all of them. It leaves out the repository-level files
# (docs/, agent_docs/, .github/, .claude/, the root: no build reads them) and
# Markdown inside a package (*.md, any depth: PROC-1 has the README updated
# after the code passes). No CMakeLists installs a .md and no test outside
# repo_scripts opens one (checked 2026-10-01); a test that starts to must end
# this exemption. repo_scripts (its tests read the repository, Markdown
# included) and PROC-3 key on the whole tree.
#
# The key is NOT the diff against the watermark: once the watermark moves past
# an unbuilt commit, two different trees share one change set.
#
# Only a green build AND a green test is remembered, per package, at the moment
# it happens: a turn blocked by another gate keeps the verdicts it did earn.
pkg_content_key() {  # $1 = a tree id from work_tree_id
  # One line per file of a package directory, Markdown left out
  # (pkg_tree_lines, shared with the source id).
  pkg_tree_lines "$1" | git hash-object --stdin 2>/dev/null || true
}
PKG_CONTENT_KEY=""
if [ -n "${WORK_TREE:-}" ]; then
  PKG_CONTENT_KEY=$(pkg_content_key "$WORK_TREE")
fi
# $1 = package the key is for; $2 / $3 = the tree and its package key, the ones
# this call started with unless given (VERDICT_TAG: see the pass files above).
change_set_key() {
  local tree="${2-${WORK_TREE:-}}" pkgs="${3-$PKG_CONTENT_KEY}"
  [ -n "$tree" ] || return 0
  case "$1" in
    repo_scripts | PROC-3) printf '%s%s' "$VERDICT_TAG" "$tree" ;;
    *) [ -z "$pkgs" ] || printf '%s%s' "$VERDICT_TAG" "$pkgs" ;;
  esac
}
pkg_verdict_reusable() {  # $1 = package, $2 = key
  # PROC-3's one verdict is of every package: any stale one ends it.
  case "$1" in
    PROC-3) [ -z "$STALE_ARTIFACT_PKGS" ] || return 1 ;;
    *) case " $STALE_ARTIFACT_PKGS " in *" $1 "*) return 1 ;; esac ;;
  esac
  [ -n "$2" ] && [ -z "${RTC_VERIFY_NO_REUSE:-}" ] \
    && grep -qxF "$1 $2" "$PASS_PKGS_FILE" 2>/dev/null
}
remember_pkg_verdict() {  # $1 = package, $2 = key, $3 = the tree it passed on
  [ -n "$2" ] || return 0
  {
    grep -v "^$1 " "$PASS_PKGS_FILE" > "$PASS_PKGS_FILE.tmp" || true
    printf '%s %s\n' "$1" "$2" >> "$PASS_PKGS_FILE.tmp"
    mv "$PASS_PKGS_FILE.tmp" "$PASS_PKGS_FILE"
  } 2>/dev/null || true
  remember_artifact_stamps "$1" "${3:-}"
}
# A green build and test, recorded -- if what the key names is still there.
# The key ($2) was taken at the start; a backgrounded --run makes an edit
# during the build ordinary, and filed under the old key the verdict would be
# honoured whenever the tree returns to it. So the key is retaken and the
# verdict recorded only if it has not moved; otherwise the package stays owed
# and the report says why. An edit made AND undone inside the run is not seen.
remember_tested_verdict() {  # $1 = package (or PROC-3), $2 = key at the start
  local tree_now key_now
  [ -n "$2" ] || return 0
  tree_now=$(work_tree_id)
  key_now=$(change_set_key "$1" "$tree_now" "$(pkg_content_key "$tree_now")")
  if [ "$key_now" != "$2" ]; then
    TEST_FAILURES="${TEST_FAILURES}  - ${1}: built and tested green, but its files changed while that ran — no verdict recorded: what was tested is not what is there now. Run '${RUN_CMD}' again and leave the packages alone until it finishes.\n"
    return 0
  fi
  remember_pkg_verdict "$1" "$2" "$tree_now"
}
BUILT_PKGS=""
REUSED_PKGS=""
BUILD_SWITCHED_OFF=""
PROC3_KEY=""

PROC3=$(echo "$BUILD_PKGS" | tr ' ' '\n' | grep -E '^(rtc_base|rtc_msgs)$' || true)
if [ -n "${RTC_VERIFY_SKIP_BUILD:-}" ]; then
  # Emit the routing decision before discarding it: blanking BUILD_PKGS lets
  # the suite run without a colcon workspace, and this probe line is the seam
  # it asserts build routing on.
  echo "verify-changes[probe]: BUILD_PKGS=[${BUILD_PKGS# }] PROC3=[$(echo "$PROC3" | tr '\n' ' ' | sed 's/ *$//')]" >&2
  # What was switched off is remembered: a pass that skipped a build it owed
  # verified less than a pass claims (see the end of the hook).
  [ -n "$PROC3$BUILD_PKGS" ] && BUILD_SWITCHED_OFF=1
  PROC3=""
  BUILD_PKGS=""
fi

# Packages whose verdict stands are taken out before anything is looked for:
# a turn with nothing left to build neither blocks on a rival build nor defers
# for a simulator. PROC-3 is all-or-nothing -- one broad build, one verdict,
# keyed on every changed file.
if [ -n "$PROC3" ]; then
  PROC3_KEY=$(change_set_key PROC-3)
  if pkg_verdict_reusable "PROC-3" "$PROC3_KEY"; then
    REUSED_PKGS="$BUILD_PKGS"
    PROC3=""
    BUILD_PKGS=""
  fi
else
  REMAINING=""
  for pkg in $BUILD_PKGS; do
    if pkg_verdict_reusable "$pkg" "$(change_set_key "$pkg")"; then
      REUSED_PKGS="${REUSED_PKGS} ${pkg}"
    else
      REMAINING="${REMAINING} ${pkg}"
    fi
  done
  BUILD_PKGS="$REMAINING"
fi
if [ -n "$REUSED_PKGS" ]; then
  echo "verify-changes: build/test not repeated for [${REUSED_PKGS# }] -- green with these same packages." >&2
fi

# ── Turn end: evidence, not execution ──
#
# What is left in PROC3 / BUILD_PKGS here has no green verdict for its present
# content. --run builds and tests it. The turn-end call does not: it names what
# is owed and blocks, so the build and the tests run DURING the turn -- where
# their output can be read, nothing is cut to fit a Stop budget, and a full
# suite is run once instead of once by the agent and once more here.
#
# At the turn end, in this order: a build already running (most likely that
# --run, backgrounded) -> wait for it; a simulator or a held host -> defer,
# because nothing can be built beside it and it is not the agent's to stop;
# otherwise -> block and name the command. --run meets the same two rivals and
# builds beside neither.
BUILD_BOUND_S="${RTC_VERIFY_BUILD_BOUND_S:-900}"
TEST_BOUND_S="${RTC_VERIFY_TEST_BOUND_S:-600}"
FULL_BUILD_BOUND_S="${RTC_VERIFY_FULL_BUILD_BOUND_S:-2400}"
FULL_TEST_BOUND_S="${RTC_VERIFY_FULL_TEST_BOUND_S:-1200}"
RUN_CMD=".claude/hooks/verify-changes.sh --run"
if [ -n "$PROC3" ]; then
  OWED="every package (PROC-3: rtc_base / rtc_msgs touched)"
else
  OWED="${BUILD_PKGS# }"
fi

RIVALS=""
SIM_RIVALS=""
SIM_DEFERRED=""
if [ -n "$PROC3$BUILD_PKGS" ]; then
  RIVALS=$(workspace_build_rivals)
  if [ -z "$RUN_MODE" ]; then
    # A --run in flight is a build to wait for even while no colcon of its is
    # alive (its gates, the gap between two packages, its last phases).
    RUN_RIVAL=$(run_lock_holder)
    if [ -n "$RUN_RIVAL" ]; then
      RIVALS="${RUN_RIVAL}${RIVALS:+
$RIVALS}"
    fi
  fi
  SIM_RIVALS=$(workspace_sim_rivals)
  HOLDS=$(workspace_holds)
  if [ -n "$HOLDS" ]; then
    SIM_RIVALS="${SIM_RIVALS:+$SIM_RIVALS
}$HOLDS"
  fi
fi
# The first three of a process list ($1), indented ($2, default six spaces) for
# the `echo -e` report. Backslashes doubled, as build_log_tail does.
report_procs() { sed -n '1,3p' <<<"$1" | sed -e 's/\\/\\\\/g' -e "s/^/${2-      }/"; }

# A test run that produced no verdict, reported: every TEST_STATUS but red and
# green, which the call sites word themselves. $1 = what was tested (a package,
# or "PROC-3 broad test"), $2 = how many packages colcon was asked for (empty =
# the whole workspace), $3 = the bound it ran under, $4 = that bound's variable.
report_unverified_tests() {
  case "$TEST_STATUS" in
    timeout)
      TEST_FAILURES="${TEST_FAILURES}  - ${1}: colcon test TIMED OUT after ${3}s — UNVERIFIED, treat as failure (a hung test, or raise ${4} if the suite is legitimately this long)\n"
      ;;
    launch)
      TEST_FAILURES="${TEST_FAILURES}  - ${1}: colcon test could not launch (exit ${TEST_RC}; env/build issue) — UNVERIFIED\n"
      ;;
    noresult)
      TEST_FAILURES="${TEST_FAILURES}  - ${1}: colcon test exited ${TEST_RC} with no parseable result summary — UNVERIFIED (a test that crashed or was killed writes no result file; see build/<pkg>/Testing/Temporary/LastTest.log)\n"
      ;;
    short)
      TEST_FAILURES="${TEST_FAILURES}  - ${1}: colcon test exited 0 but its summary counts ${TEST_FINISHED} finished packages, not ${2:-one or more} — UNVERIFIED (colcon did not test what it was asked to)\n"
      ;;
  esac
}

if [ -z "$PROC3$BUILD_PKGS" ]; then
  : # nothing owed: no package changed, or every verdict stands
elif [ -z "$RUN_MODE" ] && [ -n "$RIVALS" ]; then
  TEST_FAILURES="${TEST_FAILURES}  - build/test verdict missing for: ${OWED} — and a build is running in this colcon workspace (${WORKSPACE}):\n$(report_procs "$RIVALS")\n    If it is your '${RUN_CMD}', wait for it to finish in the foreground -- \`tail --pid=<its pid> -f /dev/null\`, again if the tool times out -- then end the turn again. Ending the turn to await its completion notice is what lands here, and the stop after this block grades nothing. If it is another build, run '${RUN_CMD}' once it is over.\n"
elif [ -z "$RUN_MODE" ] && [ -n "$SIM_RIVALS" ]; then
  # Deferred, not failed: reported at the end (see workspace_sim_rivals).
  SIM_DEFERRED=1
elif [ -z "$RUN_MODE" ]; then
  TEST_FAILURES="${TEST_FAILURES}  - build/test verdict missing for: ${OWED} — the turn end does not build or test. Run '${RUN_CMD}' (it builds with --tests, runs colcon test and records the verdict; if it will take long, background it and wait for it in the foreground with \`tail --pid=<pid> -f /dev/null\`, the pid being the first field of ${RUN_LOCK}), then end the turn again. An edit inside any package after that run voids the verdict (a *.md does not).\n"
elif [ -n "$FORMAT_FAILURES" ]; then
  TEST_FAILURES="${TEST_FAILURES}  - build/test NOT run — formatter drift (listed below) blocks this run whatever the build answers, and the reformat changes what a verdict would be recorded for. Run the formatter, then run this again.\n"
elif [ -n "$RIVALS" ]; then
  TEST_FAILURES="${TEST_FAILURES}  - build/test NOT run — a build is already running in this colcon workspace (${WORKSPACE}):\n$(report_procs "$RIVALS")\n    Building beside it would race it for CPU and write the same build/ and install/ trees, so neither verdict could be trusted. Wait for it to finish (your own background task: \`tail --pid=<its pid> -f /dev/null\` in the foreground), then run this again.\n"
elif [ -n "$SIM_RIVALS" ]; then
  TEST_FAILURES="${TEST_FAILURES}  - build/test NOT run — a simulator from this colcon workspace (${WORKSPACE}) is running, or a measurement holds the host (${HOLD_FILE}):\n$(report_procs "$SIM_RIVALS")\n    Building beside it would slow the sim below real time and corrupt what it measures (ball input goes stale, catches fail). Run this again once it is over; do not stop a sim or a measurement that is not yours to get past this.\n"
elif [ -n "$PROC3" ]; then
  # PROC-3: one broad rebuild, then every test of the workspace. All colcon
  # invocations run from $WORKSPACE (AGENTS.md §9.1), not this repo's cwd.
  run_build "$FULL_BUILD_BOUND_S" full
  if [ "$BUILD_RC" -eq 124 ]; then
    TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad build (build.sh full) TIMED OUT after ${FULL_BUILD_BOUND_S}s — UNVERIFIED, not necessarily broken code ($(build_contention_evidence)). Re-run './build.sh full --tests' on an idle box before debugging the change; if the build is legitimately this long, raise RTC_VERIFY_FULL_BUILD_BOUND_S.\n"
    rm -f "$BUILD_LOG"
  elif [ "$BUILD_RC" -ne 0 ]; then
    TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad build (build.sh full) FAILED (exit ${BUILD_RC}, rtc_base / rtc_msgs touched):\n$(build_log_tail)\n"
    rm -f "$BUILD_LOG"
  elif NO_TESTS_PKGS=$(repo_pkgs_built_without_tests); [ -n "$NO_TESTS_PKGS" ]; then
    rm -f "$BUILD_LOG"
    TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad test NOT RUN — built WITHOUT tests (BUILD_TESTING=OFF in the CMake cache): ${NO_TESTS_PKGS}— UNVERIFIED. 'colcon test' would report 0 tests or stale results for them. Rebuild with './build.sh full --tests'.\n"
  else
    rm -f "$BUILD_LOG"
    # The exit code is the verdict (see run_colcon_test): 124 = timed out,
    # >=125 = could not launch (env/build), 1 = a test failed, 0 = ran green.
    # Never swallow it with `|| true`, or a killed run reads as "0 failures".
    run_colcon_test "$FULL_TEST_BOUND_S"
    classify_colcon_test ""
    if [ "$TEST_STATUS" = red ]; then
      FAILED_TESTS=$(failed_tests_report build)
      if [ -n "$FAILED_TESTS" ]; then
        TEST_FAILURES="${TEST_FAILURES}  - PROC-3 broad test failed:\n${FAILED_TESTS}\n"
      else
        TEST_STATUS=noresult
      fi
    elif [ "$TEST_STATUS" = green ]; then
      remember_tested_verdict "PROC-3" "$PROC3_KEY"
      SKIPPED_NOTE=$(skipped_result_files build)
      [ -z "$SKIPPED_NOTE" ] || SKIPPED_WHOLE="${SKIPPED_WHOLE}${SKIPPED_NOTE}"$'\n'
    fi
    report_unverified_tests "PROC-3 broad test" "" "$FULL_TEST_BOUND_S" RTC_VERIFY_FULL_TEST_BOUND_S
    rm -f "$TEST_LOG" "$TEST_MARKER"
  fi
  BUILT_PKGS=" PROC-3"
else
  order_build_pkgs
  for pkg in $BUILD_PKGS; do
    HELD_BY=$(build_held_by "$pkg")
    if [ -n "$HELD_BY" ]; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: NOT BUILT — it depends on ${HELD_BY}, whose build did not finish in this run. Built now, it would be tested against the ${HELD_BY} installed before — UNVERIFIED. Fix ${HELD_BY}, then run --run again.\n"
      continue
    fi
    # The bound ends a hung build; it is not a budget (see Limits in the
    # header). A bound-kill (124) is UNVERIFIED, a real break FAILED: both
    # exit 2, different messages.
    run_build "$BUILD_BOUND_S" -p "$pkg"
    [ "$BUILD_RC" -eq 0 ] || hold_dependents_of "$pkg"
    if [ "$BUILD_RC" -eq 124 ]; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: build TIMED OUT after ${BUILD_BOUND_S}s — UNVERIFIED, not necessarily broken code ($(build_contention_evidence)). If the load is high this build lost a CPU race; re-run './build.sh -p ${pkg} --tests' before debugging the change, and raise RTC_VERIFY_BUILD_BOUND_S if the build is legitimately this long.\n"
      rm -f "$BUILD_LOG"
      continue
    elif [ "$BUILD_RC" -ne 0 ]; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: build FAILED (exit ${BUILD_RC}):\n$(build_log_tail)\n"
      rm -f "$BUILD_LOG"
      continue
    fi
    rm -f "$BUILD_LOG"

    if [ -n "$(pkgs_built_without_tests "$pkg")" ]; then
      TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: colcon test NOT RUN — the package is built WITHOUT tests (BUILD_TESTING=OFF in build/${pkg}/CMakeCache.txt) — UNVERIFIED. 'colcon test' would report 0 tests or stale results. Rebuild with './build.sh -p ${pkg} --tests'.\n"
      continue
    fi

    # One package at a time, each with the box to itself: wall-clock-budget
    # tests are RUN_SERIAL only within their package, and one `colcon test`
    # over all packages has one exit code for all of them.
    # The exit code is the verdict (see run_colcon_test).
    #
    # RTC_VERIFY_TEST_CMD is the test-side twin of RTC_VERIFY_BUILD_CMD: an
    # executable taking the package name, whose exit code stands for the test
    # run's and whose output for its result summary. It lets the suite reach a
    # GREEN package without a colcon workspace; it replaces the colcon command
    # line, which has cases of its own against a stand-in `colcon` on PATH.
    # Never set in normal operation.
    BUILT_PKGS="${BUILT_PKGS} ${pkg}"
    if [ -n "${RTC_VERIFY_TEST_CMD:-}" ]; then
      TEST_RC=0
      RESULT=$(timeout "$TEST_BOUND_S" "$RTC_VERIFY_TEST_CMD" "$pkg" 2>&1) || TEST_RC=$?
      if [ "$TEST_RC" -eq 124 ]; then
        TEST_STATUS=timeout
      elif [ "$TEST_RC" -ge 125 ]; then
        TEST_STATUS=launch
      elif echo "$RESULT" | grep -qE "[1-9][0-9]* (error|failure)s?"; then
        TEST_STATUS=red
        FAILED_TESTS=$(echo "$RESULT" | grep -iE "FAILED|error|failure" || true)
        TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: ${FAILED_TESTS}\n"
      elif [ "$TEST_RC" -gt 1 ]; then
        TEST_STATUS=noresult
      else
        TEST_STATUS=green
      fi
    else
      run_colcon_test "$TEST_BOUND_S" "$pkg"
      classify_colcon_test 1
      if [ "$TEST_STATUS" = red ]; then
        FAILED_TESTS=$(failed_tests_report "build/${pkg}")
        if [ -n "$FAILED_TESTS" ]; then
          TEST_FAILURES="${TEST_FAILURES}  - ${pkg}: colcon test FAILED:\n${FAILED_TESTS}\n"
        else
          TEST_STATUS=noresult
        fi
      elif [ "$TEST_STATUS" = green ]; then
        SKIPPED_NOTE=$(skipped_result_files "build/${pkg}")
        [ -z "$SKIPPED_NOTE" ] || SKIPPED_WHOLE="${SKIPPED_WHOLE}${SKIPPED_NOTE}"$'\n'
      fi
      rm -f "$TEST_LOG" "$TEST_MARKER"
    fi
    if [ "$TEST_STATUS" = green ]; then
      remember_tested_verdict "$pkg" "$(change_set_key "$pkg")"
    fi
    report_unverified_tests "$pkg" 1 "$TEST_BOUND_S" RTC_VERIFY_TEST_BOUND_S
  done
fi

# --- Phase 3: Stale install/ detection (rename-aware) ---
# colcon build --symlink-install does NOT prune deleted files from install/,
# so a rename can leave the old launch/config file resolvable by ros2 launch.
# Only warns — a full rebuild (./build.sh -c) is too destructive to invoke
# automatically, and is forbidden when external packages share the tree.
# Skipped on pure-format (no deletes possible) and when install/ is absent.
#
# Looked for: the file's NAME as a regular file (a copy; name match, inexact)
# or as a DANGLING link (an installed config under --symlink-install IS a link,
# which `-type f` alone never matches). A live link of that name is another
# package's or robot's file and is not reported. Any file under config/
# counts. build.sh prunes such links from the packages it builds (rt_common.sh
# prune_dangling_install_links), so under --run Phase 2 has removed them.
STALE_INSTALL=""
INSTALL_ROOTS=""
if [ "$PURE_FORMAT" -eq 0 ]; then
  # The colcon workspace's install/ by absolute path (so the report names a
  # path that works from any cwd), plus a repo-local install/ -- an AGENTS.md
  # §9.1 slip, still worth looking into.
  INSTALL_CANDS="./install"
  [ -n "$WORKSPACE" ] && INSTALL_CANDS="$WORKSPACE/install $INSTALL_CANDS"
  for cand in $INSTALL_CANDS; do
    [ -d "$cand" ] && INSTALL_ROOTS="${INSTALL_ROOTS} ${cand}"
  done
  if [ -n "$INSTALL_ROOTS" ]; then
    DELETED=$(git diff --diff-filter=D --name-only "$VERIFY_BASE" 2>/dev/null \
                | grep -E '(^|/)(launch/[^/]+\.(py|xml|yaml)$|config/.+$)' \
                || true)
    while IFS= read -r path; do
      [ -n "$path" ] || continue
      bname=$(basename "$path")
      for root in $INSTALL_ROOTS; do
        FOUND=$(find "$root" -name "$bname" \( -type f -o -xtype l \) 2>/dev/null | head -3 || true)
        if [ -n "$FOUND" ]; then
          STALE_INSTALL="${STALE_INSTALL}  - ${path} deleted from src but still in install/:\n$(echo "$FOUND" | sed 's/^/      /')\n"
          break
        fi
      done
    done <<< "$DELETED"
  fi
fi

# --- Phase 4: shellcheck on changed shell scripts ---
# Every changed *.sh at --severity=warning (info notes such as SC1091 do not
# block); the repo-root .shellcheckrc is found by walking up. Fails open when
# the linter is absent. No comment line here may begin with the linter's name:
# it parses such a line as a directive and aborts the file.
#
# Whole file, not added lines: the linter reports the symptom, not the cause (a
# deleted assignment surfaces as SC2154 at the unchanged use). Fair because
# docs-validate.yml lints the whole tracked *.sh corpus at this severity: a
# finding here was caused by the diff or slipped past that gate. Fix it (or a
# `disable=` directive with a reason) -- do not narrow this.
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
# Which gates blocked, as short codes for the timing log's reason column
# (`arch+build`), so a run of blocks can be read without the stderr text.
block_reasons() {
  local r=""
  [ -n "$ARCH_VIOLATIONS" ] && r="$r+arch"
  [ -n "$WARNINGS" ] && r="$r+meta"
  [ -n "$DOC_FAILURES" ] && r="$r+docs"
  [ -n "$YAML_FAILURES" ] && r="$r+yaml"
  [ -n "$RULES_FAILURES" ] && r="$r+rules"
  [ -n "$TESTGATE_FAILURES" ] && r="$r+testgate"
  [ -n "$TEST_FAILURES" ] && r="$r+build"
  [ -n "$STALE_INSTALL" ] && r="$r+stale"
  [ -n "$SHELLCHECK_FAILURES" ] && r="$r+shellcheck"
  [ -n "$FORMAT_FAILURES" ] && r="$r+format"
  [ -n "$IMPORT_MISSING" ] && r="$r+import"
  printf '%s' "${r#+}"
}
REPORT=""
IMPORT_MISSING=""
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
if [ -n "$RULES_FAILURES" ]; then
  REPORT="${REPORT}Path-scoped rule cannot fire (repo_scripts/scripts/validate_claude_rules.py):\n${RULES_FAILURES}\n"
fi
if [ -n "$TESTGATE_FAILURES" ]; then
  REPORT="${REPORT}Test isolation gates and test registration (the CI checks, run whole-repo):\n${TESTGATE_FAILURES}\n"
fi
if [ -n "$TEST_FAILURES" ]; then
  REPORT="${REPORT}Test/build failures:\n${TEST_FAILURES}\n"
fi
if [ -n "$STALE_INSTALL" ]; then
  REPORT="${REPORT}Stale install/ artefacts (deleted from src, still in install/ — './build.sh -p <pkg>' removes a link whose target is gone; a copied file needs a manual rm):\n${STALE_INSTALL}\n"
fi
if [ -n "$SHELLCHECK_FAILURES" ]; then
  REPORT="${REPORT}shellcheck (warning+) on changed shell scripts:\n${SHELLCHECK_FAILURES}\n"
fi
if [ -n "$FORMAT_FAILURES" ]; then
  REPORT="${REPORT}Formatter drift introduced on changed sources:\n${FORMAT_FAILURES}\n"
fi

# Constitution split: AGENTS.md is the tool-neutral constitution and CLAUDE.md
# imports it (`@AGENTS.md`), adding only Claude Code mechanisms. Checked:
#   * the import is lost (the line dropped, CLAUDE.md or AGENTS.md deleted or
#     moved): Claude Code silently loses the constitution -> blocking. Keyed on
#     the BASE carrying the import, so a deletion or rename is graded too;
#   * a rule lands in CLAUDE.md, where no other tool reads it -> non-blocking
#     (only the agent can tell a rule from a mechanism).
if git show "$VERIFY_BASE:CLAUDE.md" 2>/dev/null | grep -qx '@AGENTS.md'; then
  if [ ! -f CLAUDE.md ] || ! grep -qx '@AGENTS.md' CLAUDE.md; then
    REPORT="${REPORT}Constitution import missing:\n  - CLAUDE.md is gone or no longer has the line '@AGENTS.md' -- Claude Code would load none of AGENTS.md; restore it\n\n"
    IMPORT_MISSING=1
  elif [ ! -f AGENTS.md ]; then
    REPORT="${REPORT}Constitution import missing:\n  - CLAUDE.md imports '@AGENTS.md' but AGENTS.md is gone from the repository root -- Claude Code would load no constitution; restore it\n\n"
    IMPORT_MISSING=1
  fi
fi
if echo "$CHANGED_TRACKED" | grep -qx 'CLAUDE.md' && [ -f CLAUDE.md ]; then
  CHECKLIST="${CHECKLIST}  - CLAUDE.md changed: it holds only Claude Code mechanisms — if the edit states a rule, put it in AGENTS.md (imported by CLAUDE.md) so other tools get it too\n"
fi

# ARCH-6 QoS depth is a non-blocking sensor: fold it into the checklist stream
# so it surfaces alongside (never as) a hard failure.
if [ -n "$QOS_VIOLATIONS" ]; then
  CHECKLIST="${CHECKLIST}${QOS_VIOLATIONS}"
fi

SIM_NOTE=""
if [ -n "$SIM_DEFERRED" ]; then
  SIM_NOTE="Build/test deferred -- a simulator from this colcon workspace (${WORKSPACE}) is running, or a measurement holds the host (${HOLD_FILE}):\n$(report_procs "$SIM_RIVALS" "  ")\nBuilding beside it would slow the sim below real time and corrupt what it measures (ball input goes stale, catches fail). The build/test verdict for ${OWED} stays owed: everything changed since $(git rev-parse --short "$VERIFY_BASE" 2>/dev/null || echo "$VERIFY_BASE") is still graded, and the first turn end with no sim running asks for '${RUN_CMD}'. Do not stop a sim or a measurement that is not yours to get past this.\n"
fi

if [ -n "$REPORT" ]; then
  [ -n "$SIM_NOTE" ] && REPORT="${REPORT}${SIM_NOTE}"
  # A hard failure is blocking. Ride the non-blocking checklist along so the
  # agent sees doc reminders while it is already addressing the real failure.
  if [ -n "$CHECKLIST" ]; then
    REPORT="${REPORT}Doc checklist (reminder — not itself blocking):\n${CHECKLIST}\n"
  fi
  echo -e "${REPORT}See agent_docs/modification-guide.md for the full checklist." >&2
  log_timing "blocked" "$BUILT_PKGS" "$REUSED_PKGS" "$(block_reasons)"
  exit 2
fi

# No hard failure: the doc checklist prints as a non-blocking reminder (README
# co-update is a judgement call the agent/user makes, never exit 2 alone).
if [ -n "$CHECKLIST" ]; then
  echo -e "Doc checklist (non-blocking — turn NOT blocked):\n${CHECKLIST}\nSee agent_docs/modification-guide.md. If a public-surface change genuinely needs no README edit, note that in your report." >&2
fi

if [ -n "$SIM_NOTE" ]; then
  # Deferral: the turn ends, the watermark stays.
  echo -e "${SIM_NOTE}" >&2
  log_timing "deferred" "$BUILT_PKGS" "$REUSED_PKGS"
  exit 0
fi

# A full pass. The tree id taken at the start is remembered only if the tree is
# still that one: something that wrote the checkout while the gates ran was
# not graded.
#
# A run whose build was switched off (RTC_VERIFY_SKIP_BUILD) while packages
# were waiting for one claims neither: the tree is not remembered and the
# watermark stays, so the next stop still owes those packages their build.
if [ -n "$BUILD_SWITCHED_OFF" ]; then
  echo "verify-changes: build/test was switched off (RTC_VERIFY_SKIP_BUILD) with packages waiting -- watermark kept, nothing remembered." >&2
  log_timing "pass-unbuilt" "" ""
  exit 0
fi
#
# The watermark moves under the same condition: it is written from HEAD as it
# is NOW, so a commit made during a backgrounded --run would be stepped over,
# graded by no gate. With the tree changed, the package verdicts still stand
# and everything since the old watermark is graded again at the next call.
TREE_MOVED=""
if [ -n "${WORK_TREE:-}" ] && [ "$(work_tree_id)" != "$WORK_TREE" ]; then
  TREE_MOVED=1
  echo "verify-changes: the working tree changed while the gates ran -- watermark kept: what was written or committed meanwhile is graded at the next call." >&2
else
  if [ -n "${WORK_TREE:-}" ]; then
    printf '%s%s\n' "$VERDICT_TAG" "$WORK_TREE" > "$PASS_TREE_FILE" 2>/dev/null || true
  fi
  [ -z "$RUN_MODE" ] || baseline_artifact_stamps
  advance_verify_base
fi
log_timing "$([ -n "$TREE_MOVED" ] && echo pass-tree-moved || echo pass)" "$BUILT_PKGS" "$REUSED_PKGS"
# By hand there is no caller to read the exit code off a status line.
if [ -n "$RUN_MODE" ]; then
  echo "verify-changes --run: PASS (${SECONDS}s) -- built and tested [${BUILT_PKGS# }], verdict reused for [${REUSED_PKGS# }]." >&2
  if [ -n "$SKIPPED_WHOLE" ]; then
    echo "verify-changes --run: NOTE -- result files in which every case was SKIPPED (green, but they graded nothing: a dependency only the venv has; run them by hand with the venv -- agent_docs/testing-debug.md):" >&2
    printf '%s' "$SKIPPED_WHOLE" >&2
  fi
fi
exit 0
