#!/bin/bash
# Stop hook — gate the turn end on doc/metadata co-updates + build/test.
#
# Intent : enforce AGENTS.md §4 Workflow Loop steps 4·5·6 + PROC-1 (doc/code
#          sync) without trusting Claude's self-check (Anthropic 2026.04:
#          agent self-eval is unreliable).
# Trigger: every turn end. Reads {stop_hook_active} from stdin JSON; bails
#          early on re-entry to avoid infinite loops. Also reads
#          {background_tasks}: defers (exit 0, watermark kept) while a
#          background agent may still be writing the checkout -- see
#          "Background agents still writing the checkout".
# Modes  : (no argument)  the turn-end call. Runs every gate EXCEPT the build
#                         and the tests: for those it only CHECKS that the
#                         changed packages carry a green verdict for their
#                         present content, and blocks when one does not.
#          --run          called by hand, during the turn. Runs every gate AND
#                         builds and tests the changed packages, and records
#                         the verdicts the turn-end call looks for. Reads no
#                         stdin. Long runs: background it and wait for it.
#          The turn end stopped building on 2026-10-01. Measured over the two
#          days before (18 turn ends that built, 1,769 s): 78% of that time was
#          tests, about 70% of the test time re-ran a full suite the agent had
#          run on the same tree minutes earlier, 14 of 15 came right after a
#          commit -- and in a month of transcripts the phase blocked 25 times
#          without one real build or test failure (11 times a build was already
#          running, 14 timeouts or tests not started for the Stop budget). What the
#          turn end keeps is the part that did earn its place: a tree whose
#          last edit came after its last full test run does not pass.
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
#                   `ARCH-7-exempt` comment on the call, or anywhere in the
#                   comment block attached directly above it.
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
#   1c. changed .claude/rules/*.md -> validate_claude_rules.py (globs can fire)
#   1d. changed CMakeLists.txt / conftest.py / colcon.pkg / test sources -> the
#        CI test gates validate_test_domains.py + validate_test_fixtures.py,
#        WHOLE repo (a domain collision spans two packages; CI keeps main clean)
#   2. Build + test on changed packages -- EXECUTED under --run, only CHECKED
#      at the turn end (see Modes and "Turn end: evidence, not execution")
#        - a package is "changed" for this phase by its source (.cpp/.hpp/.h/
#          .cc/.py), its CMakeLists.txt / package.xml / colcon.pkg, or a shell
#          script in its source directories (see CHANGED_SH_BUILD)
#        - rtc_base / rtc_msgs change -> ./build.sh full --tests + colcon test all
#          (PROC-3: broad downstream impact)
#        - else                       -> ./build.sh -p <pkg> --tests + colcon test <pkg>,
#          one package at a time (each suite has the box to itself), in
#          DEPENDENCY order: a package is built and tested after the changed
#          packages it depends on, and not at all when one of those failed to
#          build (see "Build order")
#        (--tests: build.sh skips tests by default, and a package built without
#        them tests as "0 tests, 0 failures" -- see run_build. Independently of
#        that flag, a package whose CMake cache says BUILD_TESTING=OFF after the
#        build is reported UNVERIFIED and not tested -- pkgs_built_without_tests)
#        The tests' verdict is `colcon test --return-code-on-test-failure`'s
#        exit code plus its summary line counting the packages asked for --
#        not a reading of result files (run_colcon_test says what that reading
#        missed). A test run that TIMES OUT, fails to launch, fails without
#        leaving a failing result, or finishes fewer packages than asked is
#        reported as UNVERIFIED and blocks (exit 2), so a killed/partial run
#        cannot masquerade as "0 failures".
#   3. Stale install/ detection (rename-aware)
#        - any deleted launch/*.py or config/**/*.yaml whose basename still
#          resolves under install/ — warns about stray artefacts that
#          colcon --symlink-install does not prune.
#   4. shellcheck on changed *.sh (any path), whole file
#        - runs at --severity=warning (notes do not block); repo-root
#          .shellcheckrc supplies external-sources + SC2034 suppression.
#        - NOT narrowed to added lines like the doc phase: see Phase 4.
#   5. Formatter drift on changed C++ / Python (clang-format / ruff format)
#        - blocks only drift the change INTRODUCED (base blob absent or already
#          a formatter fixed point); pre-existing debt passes. See Phase 5.
#        - `ruff check` lint is NOT graded; Doxygen, YAML default/range/unit and
#          README need stay manual (modification-guide.md Completion Checklist).
#        - at the turn end, past RTC_VERIFY_FORMAT_DEADLINE_S (480s) the rest
#          is listed as ungraded; --run has no Stop budget and no deadline.
#
# Verdict reuse (what the turn end reads, and what keeps an unchanged tree
# from being re-verified):
#   - the WHOLE working tree is identical to the one this hook last passed at
#     -> nothing is run (see "Nothing changed since the last pass");
#   - the package directories are, in content, the ones a package last built
#     and tested green with (their Markdown aside: a README brought up to date
#     after the code passed keeps the verdict) -> its build/test is not
#     repeated (--run) or not owed (turn end); every other gate still runs
#     (see "Package verdict reuse").
#   Both are keyed on content, never on time or on the watermark, and only a
#   PASS that went through build/test is remembered -- so committing a tree
#   --run passed costs the turn end nothing, and editing a package after it
#   voids the verdict.
#   A verdict is of the SOURCE. --run answers for the BINARIES as well: with a
#   verdict it records the package's installed binaries, and it does not reuse
#   a verdict whose binaries were rebuilt since -- it builds and tests that
#   package again (see "The binaries a verdict was recorded with"). The turn
#   end does not look at the install tree.
#   --run is made to be backgrounded, so three things hold while it runs: it
#   takes a lock, which a second --run refuses and the turn end reads as "a
#   build to wait for" (run_lock_holder); a package edited during the build or
#   the tests gets no verdict (remember_tested_verdict); and the watermark and
#   the whole-tree pass move only if the tree is still the one the gates read
#   (end of this file) -- a commit made meanwhile is graded at the next call.
#   A simulator from this workspace, or a measurement that holds the host (see
#   workspace_holds), DEFERS a missing verdict at the turn end and makes --run
#   refuse to build.
#   Names, for a document that points here instead of restating this (the
#   comment at each definition owns the detail): RTC_VERIFY_NO_REUSE=1 turns
#   both reuses off; the pass files and rtc-verify-timing.log (one line per
#   run, with its mode) are kept in .git/ ("Kept in .git/ beside the
#   watermark"); a measurement holds the host through
#   <workspace>/.rtc-verify-hold, which repo_scripts/scripts/with_verify_hold.sh
#   writes for the lifetime of the driver it wraps (workspace_holds).
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
#          2 on any hard failure -- at the turn end a changed package without
#          a verdict is one -> Claude is blocked, stderr message is
#          auto-injected next turn. Pointer to modification-guide.md is appended
#          so the agent has a recovery entry point. --run exits the same way
#          (2 also when it refused to build); 64 on an unknown argument.
#          Loop safety: re-entry is gated by {stop_hook_active} (read from stdin
#          below) -- the hook fires once per stop cycle, so it cannot wedge the
#          turn in an infinite block. Official Stop-hook exit-2 semantics:
#          "Prevents Claude from stopping, continues the conversation"
#          (code.claude.com/docs/en/hooks). The agent must act on the injected
#          report. Claude Code overrides the hook after 8 CONSECUTIVE blocks
#          (documented: code.claude.com/docs/en/best-practices) -- that cap
#          is an unverified stop, not an exit; do not lean on it.
# Limits : --run bounds (seconds, each an environment override): one package
#          build RTC_VERIFY_BUILD_BOUND_S (900), its tests
#          RTC_VERIFY_TEST_BOUND_S (600); the PROC-3 workspace build
#          RTC_VERIFY_FULL_BUILD_BOUND_S (2400) and its tests
#          RTC_VERIFY_FULL_TEST_BOUND_S (1200). They are there to end a hang,
#          not to fit a budget: --run is called by hand and nothing kills it
#          from outside. Sized on this 6C/12T box (2026-10-01, one package at a
#          time x make -j6; repo_scripts/README.md "빌드 병렬도와 메모리"): a
#          whole package recompiled is 241-279s (integrated_bringup), a warm
#          workspace after an rtc_msgs touch 353s, a clean `build.sh full
#          --tests` 18m49s; the slowest suite, rtc_tools, is 105-177s.
#          The bounds these replaced were cut to the Stop hook's 540s budget
#          (180s + 120s per package, 300s + 180s for PROC-3, and a deadline
#          past which a test was not started), and none of the three builds
#          above fits them: PROC-3 was UNVERIFIED for a one-line rtc_base
#          header edit, by decision (user, 2026-08-14), because the alternative
#          was a ~13min block at every such turn end. Taking the build out of
#          the turn end is what removed that trade.
#          A build OR test that hits its bound (exit 124) or fails to launch
#          (exit >=125) blocks as UNVERIFIED, in a message DISTINCT from a real
#          failure's. That distinction is what the build path lacked (#435):
#          `if ! timeout 180 ./build.sh ...` swallowed $? and the output, so a
#          bound-kill and a compile error printed the same "<pkg>: build
#          failed" and machine load could masquerade as broken code (measured:
#          179.4s killed under contention vs 2.5s idle, same commit). A 124
#          also reports loadavg and the live build/compiler count, and a real
#          failure reports its exit code and the tail of the build log.
#          A colcon / build.sh ALREADY running in this workspace (e.g. the
#          agent's own background shell task) is looked for first: --run
#          builds nothing beside it -- that races for CPU and writes the same
#          build/ and install/ trees -- and the turn end, when a verdict is
#          missing, says to wait for it. Matched by name + cwd, so a colcon
#          aimed here from elsewhere with an absolute --build-base is not seen
#          (see workspace_build_rivals). A MuJoCo simulator started from this
#          workspace's install tree makes --run refuse as well (building beside
#          a timing-sensitive sim run slows it below real time), and at the
#          turn end it DEFERS a missing verdict instead of blocking (exit 0
#          with the watermark kept, unless another gate blocks;
#          workspace_sim_rivals): a sim does not end on its own the way a rival
#          build does.
#          Doxygen / cross-package doc consistency NOT checked
#          (modification-guide.md "Updating an Existing Package" and its
#          Completion Checklist cover these manually). Changed set = tracked-vs-$VERIFY_BASE UNION
#          untracked, where VERIFY_BASE is the watermark commit this hook last
#          passed at (see "Verification baseline" below) — NOT HEAD, so work
#          the agent committed during the turn is still verified;
#          build/test additionally requires an untracked file to live in one of
#          the installed-source dirs allowlisted at CHANGED_SRC_UNTRACKED below
#          (that comment is the SSoT -- this summary still said src|include only
#          after four more dirs joined the list), so a new header is compiled
#          while a scratch file under rtc_base/ still cannot trigger a
#          full-workspace rebuild. Routing behaviour is asserted end-to-end by
#          repo_scripts/test/test_verify_changes.sh.
#          A path containing a newline is C-quoted by git regardless of
#          core.quotePath and is NOT handled; paths with spaces are.
set -euo pipefail

# Mode (see "Modes" in the header). The turn-end call takes no argument and
# its input on stdin; --run is typed by hand and must not sit waiting on a
# terminal for input that is not coming.
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
if [ -n "$RUN_MODE" ]; then
  PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." 2>/dev/null && pwd)}"
else
  PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(pwd)}"
fi
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

# ── Verification baseline ───────────────────────────────────────────────────
#
# Every gate below diffs against $VERIFY_BASE, not against HEAD. The two differ
# only when the agent COMMITTED during the turn — and that is the case this
# exists for: with HEAD as the baseline a turn that ends `git commit`-clean
# produces an empty change set, the `[ -z "$CHANGED" ]` exit fires, and NOTHING
# is verified. Not the doc validator, not the ARCH greps, not build or test.
# That is the normal shape of a turn in this repo (AGENTS.md §11 housekeeping
# runs "commit 완료 후"), so the gate was silently absent on most of them.
# Observed 2026-09-09: a bare-section-ref D10 violation committed in-turn passed
# the hook and was caught only by CI's full-corpus scan.
#
# The baseline is a WATERMARK — the commit this hook last finished a clean run
# at — kept in .git/ so it is per-clone, never committed, and gone after a fresh
# clone. Semantics: "everything that changed since this gate last passed",
# which is what the gate is actually promising. Consequences worth stating:
#
#   * a turn that commits is verified ONCE, on that turn; the watermark then
#     advances to HEAD and the next idle turn is empty again, so this costs one
#     verification — precisely the one that was missing — and not a rebuild per
#     turn thereafter;
#   * a blocked run (exit 2) does NOT advance it, so the next turn re-checks;
#   * the added-line narrowing (docs, ARCH-1, ARCH-5) now sees lines added by
#     those commits, which is where it was blindest.
#
# FALLS BACK TO HEAD whenever the watermark is missing, unreadable, not a
# commit, or no longer an ancestor of HEAD (rebase, branch switch, reset). That
# makes the fallback byte-identical to the pre-watermark behaviour rather than
# producing a diff against an unrelated history — degrade toward the old gate,
# never toward a workspace-wide rebuild.
VERIFY_BASE_FILE="$(git rev-parse --git-dir 2>/dev/null || echo .git)/rtc-verify-base"
VERIFY_BASE=HEAD
if [ -f "$VERIFY_BASE_FILE" ]; then
  WATERMARK=$(cat "$VERIFY_BASE_FILE" 2>/dev/null || true)
  if [ -n "$WATERMARK" ] \
     && git rev-parse --verify --quiet "${WATERMARK}^{commit}" >/dev/null 2>&1 \
     && git merge-base --is-ancestor "$WATERMARK" HEAD 2>/dev/null; then
    VERIFY_BASE="$WATERMARK"
  fi
fi

# Called on every NON-BLOCKING exit, never on exit 2 and never on the
# stop_hook_active re-entry (which verified nothing and must not claim to have).
advance_verify_base() {
  git rev-parse HEAD > "$VERIFY_BASE_FILE" 2>/dev/null || true
}

# ── Verdict reuse: state and helpers ────────────────────────────────────────
#
# Measured 2026-09-30 (one session, 16 turn ends over a tree that did not
# change between them): every one of them rebuilt and re-tested the same two
# packages, 3-5 minutes each, because the watermark above is a COMMIT -- an
# uncommitted change stays "changed since the last pass" however often it has
# passed, and committing it afterwards makes it changed once more. The verdict
# of a tree is a function of its content, so the content is what is remembered.
#
# Kept in .git/ beside the watermark: per clone, never committed.
#   rtc-verify-pass-tree   tree id of the working tree at the last full pass
#   rtc-verify-pass-pkgs   "<pkg> <content key>" per package that built and
#                          tested green
#   rtc-verify-timing.log  one line per run: when, verdict, seconds, packages,
#                          mode (stop = the turn-end call, run = --run)
# RTC_VERIFY_NO_REUSE=1 switches both reuses off (every gate runs, nothing is
# read from the two pass files; a pass is still recorded).
GIT_DIR_PATH="$(git rev-parse --git-dir 2>/dev/null || echo .git)"
PASS_TREE_FILE="$GIT_DIR_PATH/rtc-verify-pass-tree"
PASS_PKGS_FILE="$GIT_DIR_PATH/rtc-verify-pass-pkgs"
TIMING_LOG="$GIT_DIR_PATH/rtc-verify-timing.log"
RUN_LOCK="$GIT_DIR_PATH/rtc-verify-run.lock"

# ── The binaries a verdict was recorded with ────────────────────────────────
#
# The two pass files say "this source built and tested green". They say
# nothing about what is INSTALLED now, and the two come apart whenever a build
# runs on other content after the verdict and the source then returns to what
# passed:
#   * a temporary patch applied, built for a measurement and reverted
#     (2026-10-02: --run answered "nothing re-run" over binaries that still
#     carried the patch, and built without tests);
#   * a change that fails its tests under --run and is checked out again.
# The tree is the one that passed, so both reuses honoured it, and the next
# simulator run would have executed code that is in no commit.
#
# So a verdict is recorded together with a stamp of the package's installed
# compiled products, and --run does not reuse a verdict whose stamp no longer
# matches: it builds and tests that package again, which leaves the install
# tree built from the tree being graded.
#   rtc-verify-pass-artifacts   "<pkg> <stamp>" per package with a verdict
# The stamp is the name and CONTENT (blob id) of every shared or static
# library under <workspace>/install/<pkg>/lib and of every ELF file directly
# in lib/<pkg>/. Content, not mtime: a checkout or a pull rewrites source
# files, the next build recompiles and relinks them, and the binaries come out
# byte for byte what they were with a new mtime (measured 2026-10-02 -- after a
# fast-forward the two packages it touched were relinked by a build of
# unchanged content; their blob ids did not move, and did not move either
# across a build without tests and one with). Scripts and the Python tree are
# left out: a build that changes nothing rewrites them. Hashing every
# package's products takes about 0.4 s (21 packages, 50 MB), and only --run
# and a recorded verdict pay it.
#
# The TURN END does not read the stamp. A debug, sanitizer or tracing build
# kept on purpose across turns also has "other binaries than the verdict's",
# and blocking on it would have every turn end ask for a --run that replaces
# that build. What the turn end guarantees stays what it was: the source has a
# green verdict. A package with no install directory has no stamp and is never
# stale by it (ament_python, a fixture without a workspace).
PASS_ARTIFACTS_FILE="$GIT_DIR_PATH/rtc-verify-pass-artifacts"
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
# The packages a verdict names: itself, or every package of the repo for the
# one broad PROC-3 verdict.
verdict_packages() {  # $1 = package or PROC-3
  local d
  if [ "$1" != "PROC-3" ]; then printf '%s\n' "$1"; return 0; fi
  for d in "$PROJECT_DIR"/*/; do
    [ -f "${d}package.xml" ] && basename "$d"
  done
  return 0
}
remember_artifact_stamps() {  # $1 = package or PROC-3
  local p stamp
  {
    while IFS= read -r p; do
      [ -n "$p" ] || continue
      stamp=$(artifact_stamp "$p")
      grep -v "^$p " "$PASS_ARTIFACTS_FILE" > "$PASS_ARTIFACTS_FILE.tmp" || true
      [ -z "$stamp" ] || printf '%s %s\n' "$p" "$stamp" >> "$PASS_ARTIFACTS_FILE.tmp"
      mv "$PASS_ARTIFACTS_FILE.tmp" "$PASS_ARTIFACTS_FILE"
    done <<< "$(verdict_packages "$1")"
  } 2>/dev/null || true
}
# A --run that passes also takes a stamp for every package that has none yet:
# the binaries as they are when the tree passes. Without it only a package
# --run itself built would ever be watched, and the temporary patch of the
# first case above can sit in any package. It is a baseline, not a verdict --
# binaries already stale when it is taken are not found -- and it is never
# refreshed here: only a green build and test replaces a stamp.
baseline_artifact_stamps() {
  local p stamp
  {
    while IFS= read -r p; do
      [ -n "$p" ] || continue
      grep -q "^$p " "$PASS_ARTIFACTS_FILE" 2>/dev/null && continue
      stamp=$(artifact_stamp "$p")
      [ -z "$stamp" ] || printf '%s %s\n' "$p" "$stamp" >> "$PASS_ARTIFACTS_FILE"
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
# Both pass files carry this tag in front of what they record. It names the
# test reading the verdict came from: entries without it are from before
# 2026-10-01, when a package whose tests ran was green whether they passed or
# not (see run_colcon_test). Neither file is honoured without the tag, so a
# tree that reading passed is graded once more by one that can see red.
VERDICT_TAG="t2:"

# One --run per checkout, and a turn end that can see it.
#
# --run holds this lock ("<pid> <start time>", the start time being field 22 of
# /proc/<pid>/stat, as in the hold file) from its first gate to its exit. Two
# things needed it. A turn end knew a --run was in flight only by finding its
# colcon or build.sh, so in the seconds before the first build, between two
# package builds and after the tests it said "verdict missing, run --run" --
# and a second --run started on that advice builds into the same build/ and
# install/ trees as the first. Prints "<pid>: verify-changes.sh --run" for a
# LIVE holder other than this process; a line left by a killed --run names a
# pid that is gone or was reused, and reads as no holder.
run_lock_holder() {
  local pid="" start="" now
  [ -f "$RUN_LOCK" ] || return 0
  read -r pid start _ <"$RUN_LOCK" 2>/dev/null || true
  case "$pid" in '' | *[!0-9]*) return 0 ;; esac
  [ "$pid" != "$$" ] || return 0
  [ -r "/proc/$pid/stat" ] || return 0
  # comm may hold spaces; what follows the LAST ')' starts at field 3.
  now=$(sed 's/^.*) //' "/proc/$pid/stat" 2>/dev/null | cut -d' ' -f20)
  [ -n "$start" ] && [ "$now" = "$start" ] || return 0
  printf '%s: verify-changes.sh --run\n' "$pid"
}

# Tree id of the working tree as it is now: tracked changes, untracked files,
# deletions -- what `git add -A` would stage, written through a throwaway index
# so the real one is not touched. Ignored files are left out, like the change
# set below leaves them out; a nested repository counts by its HEAD, as it does
# for `git diff`. Prints nothing when git cannot do it, which every caller
# reads as "no reuse".
#
# Nothing is left behind in the repository: the objects `git add` writes go to
# a scratch object directory that is removed when the hook exits, with the real
# store as its alternate (so only content the store does not have is written
# at all). Written into the real store, every modified or untracked file would
# become a loose, unreachable blob at every turn end. Whoever reads an object
# of that tree afterwards goes through git_scratch.
#
# The index starts EMPTY, so every file is hashed (about 0.3 s here). Starting
# from a copy of the real index costs 25 ms and is wrong: git trusts a cached
# entry whose size and mtime match unless the entry is as young as the index
# file, and a copy is always younger than the entries in it -- so a same-size
# edit made in the second the index was written (`return 0` -> `return 1`
# right after a commit) read as unchanged. Measured in this hook's own suite.
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
          "$(sed 's/^.*) //' "/proc/$$/stat" 2>/dev/null | cut -d' ' -f20)" >"$RUN_LOCK") 2>/dev/null; then
      RUN_LOCK_HELD=1
    else
      RUN_RIVAL=$(run_lock_holder)
      RUN_RIVAL="${RUN_RIVAL:-another process (it took ${RUN_LOCK} first)}"
    fi
  fi
  if [ -z "$RUN_LOCK_HELD" ]; then
    echo "verify-changes --run: NOT run — another --run is already running in this checkout (${RUN_RIVAL}). Two of them would build into the same build/ and install/ trees. Wait for it to finish (if it is your own background task, wait on that task); its verdict is the one the turn end reads." >&2
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
    printf '%s\t%s\t%ss\tbuilt=[%s]\treused=[%s]\tmode=%s\n' \
      "$(date -Is 2>/dev/null || date)" "$1" "$SECONDS" "${2# }" "${3# }" \
      "$([ -n "$RUN_MODE" ] && echo run || echo stop)" >> "$TIMING_LOG"
    if [ "$(wc -l < "$TIMING_LOG")" -gt 600 ]; then
      tail -n 500 "$TIMING_LOG" > "$TIMING_LOG.tmp" && mv "$TIMING_LOG.tmp" "$TIMING_LOG"
    fi
  } 2>/dev/null || true
}

# Changed files = tracked modifications vs $VERIFY_BASE, PLUS untracked new files.
#
# `git diff --name-only` alone cannot see a file that was never added,
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
CHANGED_TRACKED=$(git -c core.quotePath=false diff --name-only "$VERIFY_BASE" 2>/dev/null || true)
CHANGED_UNTRACKED=$(git -c core.quotePath=false ls-files -o --exclude-standard 2>/dev/null || true)
CHANGED=$(printf '%s\n%s\n' "$CHANGED_TRACKED" "$CHANGED_UNTRACKED" | grep -v '^[[:space:]]*$' | sort -u || true)
# --run looks at the installed binaries before any "nothing to do" exit (see
# "The binaries a verdict was recorded with"): a package whose binaries were
# rebuilt since its verdict is built and tested again -- with nothing changed
# against the watermark, and with the tree the one that last passed.
STALE_ARTIFACT_PKGS=""
if [ -n "$RUN_MODE" ] && [ -z "${RTC_VERIFY_NO_REUSE:-}" ]; then
  STALE_ARTIFACT_PKGS=$(stale_artifact_pkgs)
  if [ -n "$STALE_ARTIFACT_PKGS" ]; then
    echo "verify-changes: the installed binaries of [${STALE_ARTIFACT_PKGS% }] are not the ones their verdict was recorded with (built again since) -- building and testing them again." >&2
  fi
fi
if [ -z "$CHANGED" ] && [ -z "$STALE_ARTIFACT_PKGS" ]; then
  [ -z "$RUN_MODE" ] || baseline_artifact_stamps
  advance_verify_base
  exit 0
fi

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
# colcon.pkg at a package root: arguments colcon adds to every build / test of
# that package -- in this repo, the job count of its ctest. It changes how the
# package's tests RUN (side by side instead of one at a time), so it routes the
# package to build/test and to the test gates the way a CMakeLists edit does;
# on its own it used to leave the hook at the exit below, ungraded.
CHANGED_TESTCFG=$(echo "$CHANGED" | grep -E '^[^/]+/colcon\.pkg$' || true)
if [ -z "$CHANGED_SRC" ] && [ -z "$CHANGED_SH" ] && [ -z "$CHANGED_DOCS" ] \
   && [ -z "$CHANGED_YAML" ] && [ -z "$CHANGED_META" ] && [ -z "$CHANGED_TESTCFG" ] \
   && [ -z "$STALE_ARTIFACT_PKGS" ]; then
  [ -z "$RUN_MODE" ] || baseline_artifact_stamps
  advance_verify_base
  exit 0
fi

# ── Background agents still writing the checkout ────────────────────────────
#
# Stop fires when the MAIN agent ends its turn, and a background subagent does
# not hold that turn open -- so every gate below would grade a tree another
# agent is halfway through writing. Observed 2026-09-13: with nine README
# agents sharing the checkout, one turn end blocked on an anchor to a heading
# the agent merging five docs had not written yet (inviting the main agent to
# "fix" a file it did not own), and three others passed only because nothing
# happened to be half-written at that instant.
#
# Claude Code passes `background_tasks` in the Stop input (checked on 2.1.269:
# running/pending backgrounded tasks, labeled by `type`). While one that edits
# this checkout is in flight, verification is DEFERRED, not skipped: exit 0
# WITHOUT advance_verify_base, so the first turn end with none in flight diffs
# against the same watermark and grades everything they wrote. A session that
# ends first leaves the watermark in .git/ for the next session's first stop.
#
# Deferring labels are an allowlist: subagent / workflow / teammate. Not shell
# or monitor -- a CI watcher runs for tens of minutes and would switch the gate
# off for all of it -- and not an unknown label, since a wrong block costs one
# turn while a wrong defer costs an unverified tree. A missing or malformed
# field (older Claude Code) makes jq yield nothing, i.e. the gate as before.
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

# ── Nothing changed since the last pass ─────────────────────────────────────
#
# The working tree, byte for byte, is the one this hook last passed at: every
# gate below would read the same input and the build would be a no-op followed
# by the same tests. Committing a verified change, or ending a turn that only
# talked, lands here.
#
# What the tree id does NOT cover is the world outside the repository: the
# install tree, system packages, another repository in the workspace. The
# watermark never covered those either -- a change there was not re-verified
# before this existed unless the repository changed too.
#
# The install tree is the one of those --run does look at (STALE_ARTIFACT_PKGS,
# taken above): a package whose binaries were rebuilt since its verdict is
# built and tested again, unchanged tree or not.
WORK_TREE=$(work_tree_id)
if [ -n "$WORK_TREE" ] && [ -z "${RTC_VERIFY_NO_REUSE:-}" ] && [ -z "$STALE_ARTIFACT_PKGS" ] \
   && [ "$(cat "$PASS_TREE_FILE" 2>/dev/null || true)" = "${VERDICT_TAG}${WORK_TREE}" ]; then
  echo "verify-changes: the working tree is the one this gate last passed at (tree ${WORK_TREE:0:12}) -- nothing re-run." >&2
  [ -z "$RUN_MODE" ] || baseline_artifact_stamps
  advance_verify_base
  log_timing "pass-unchanged" "" ""
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
# OR <pkg>/test/ (gtest sources and the never-installed test-only headers under
# <pkg>/test/include/ that sibling packages consume by source-tree path)
# OR under <pkg>/<pkg>/ (ament_python packages -- rtc_tools, rtc_digital_twin --
# keep their modules in a same-named subdir, which the src|include-only regex
# missed, so a brand-new untracked module was ARCH-grepped but never built).
#
# test/ was the last hole of the original tracked-ness axis (#358): a brand-new
# test file was ARCH-grepped, CMake-gated, and then never compiled or run, so the
# turn whose entire purpose was adding assertions is the turn the gate verifies
# least. Both #356 additions -- a test-only header and a 9-case suite -- landed on
# that path. Widening here can only cost a build of a package that owns an
# untracked .py probe under test/; the other direction costs a silent green.
#
# launch/ and scripts/ are installed artifacts too (#360), and a new file lands in
# the install tree with NO metadata edit: launch/ is DIRECTORY-installed in five
# ament_cmake packages and globbed by rtc_digital_twin's setup.py, and rtc_math
# installs scripts/ the same way. No CMakeLists change means CHANGED_META_TRACKED
# does not route them either, so a new launch file was the one artifact that could
# reach the install tree without any part of this hook building its package --
# while *editing* that same file once tracked did build it. That asymmetry is what
# this list closes; it is not a claim that the build validates these files.
# It does not: install(DIRECTORY) copies, so no .py here is parsed or compiled, and
# the launch-wiring suites (test_launch_shield_wiring.py and siblings) hardcode
# their filenames rather than globbing, so a new launch file is not covered until
# someone adds it to those lists. What the build buys is an install tree that
# matches the source tree and a rerun of the owning package's tests.
#
# The list stays an ALLOWLIST of installed-source dirs. config/ is deliberately
# absent: it holds YAML, which the parse gate already covers, and admitting it
# would turn this into "any directory under a package" -- the unbounded reading the
# scratch exclusion exists to prevent.
# A backreference is not portable across grep flavours, so match with awk.
CHANGED_SRC_UNTRACKED=$(echo "$CHANGED_UNTRACKED" | awk -F/ '
  NF >= 3 && $NF ~ /\.(cpp|hpp|h|cc|py)$/ \
    && ($2 == "src" || $2 == "include" || $2 == "test" \
        || $2 == "launch" || $2 == "scripts" || $2 == $1)' \
  || true)
CHANGED_SRC_BUILD=$(printf '%s\n%s\n' "$CHANGED_SRC_TRACKED" "$CHANGED_SRC_UNTRACKED" \
  | grep -v '^[[:space:]]*$' | sort -u || true)
# A shell script in one of those directories routes its package to build/test
# too, tracked or not. It was linted (Phase 4) and nothing else: a package
# whose tests ARE shell scripts, or drive one, had them run for a .py edit and
# not for an edit of the script under test. Seen 2026-09-30 -- a turn that
# changed only *.sh in a package tested every other changed package and not
# that one. Kept out of CHANGED_SRC_BUILD, which also feeds the pure-format
# check and knows no formatter for a shell script.
CHANGED_SH_BUILD=$(echo "$CHANGED" | awk -F/ '
  NF >= 3 && $NF ~ /\.sh$/ \
    && ($2 == "src" || $2 == "include" || $2 == "test" \
        || $2 == "launch" || $2 == "scripts" || $2 == $1)' \
  || true)

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
# <path> -- a bare `ruff` / `clang-format` may be absent here, or a different
# version than the venv ruff / uvx-pinned clang-format, and then the "fixed"
# file drifts again.
format_fix_cmd() {
  case "$1" in
    *.py) printf '%s format %s' "$RUFF_BIN" "$1" ;;
    *) printf '%s -i %s' "${CLANG_FORMAT_CMD[*]}" "$1" ;;
  esac
}

is_pure_format() {
  [ "$HAVE_CLANG_FORMAT" -eq 1 ] || return 1

  # No source files at all is NOT "pure format". The loop below iterates over
  # $CHANGED_SRC and returns success on an empty list, so a CMake-only or
  # docs-only change used to be classified as cosmetic and skip Phase 1 --
  # silently cancelling the routing fix above. Same trap with formatting churn
  # alongside a CMake edit: the source half really is cosmetic, and the
  # unrelated CMake gate went down with it.
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
done <<< "$(printf '%s\n%s\n%s\n%s\n' "$CHANGED_SRC_BUILD" "$CHANGED_META_TRACKED" "$CHANGED_SH_BUILD" \
             "$CHANGED_TESTCFG" | grep -v '^[[:space:]]*$' | cut -d'/' -f1 | sort -u)"
# --run: a package whose installed binaries were rebuilt since its verdict is
# owed a build and a test run whether or not its source changed.
for pkg_dir in $STALE_ARTIFACT_PKGS; do
  case " $BUILD_PKGS " in
    *" $pkg_dir "*) ;;
    *) BUILD_PKGS="${BUILD_PKGS} ${pkg_dir}" ;;
  esac
done

# Emit `name<TAB>exempt<TAB>lineno` for every add_executable() in the CMake
# source arriving on stdin. Used by ARCH-7 to diff target NAMES between HEAD and
# the working tree instead of diffing lines.
#
# `exempt` is 1 when the target opts out, which it may do two ways:
#   - the name matches example_*  — design-principles.md puts examples outside
#     the runtime-identity scope by name, so they need no per-target marker and
#     cannot drift out of one;
#   - an `ARCH-7-exempt` comment sits on the add_executable line, or ANYWHERE in
#     the contiguous comment block directly above it.
#
#     The scope widened twice, each time because the marker was written where
#     CMake convention puts it and the hook did not look there. Same-line-only
#     (the first cut) rejected the idiomatic form where the justification is a
#     comment above the call. Then one-line-above rejected the equally idiomatic
#     form where that justification runs to several lines — a marker naturally
#     goes at the TOP of such a block, and the line directly above the call is
#     the last line of the prose instead (observed 2026-09-10 on
#     rtc_inference_check). Both rejections looked like ARCH-7 violations while
#     the exemption was in fact declared, which is the worst shape for a gate:
#     the author reads it as the rule misfiring rather than as a format nit.
#
#     The block is bounded by the first line that is not a comment — a blank
#     line included. So a file header separated from the code by a blank line
#     cannot exempt the first target in the file, and the marker for one call
#     cannot leak onto the next one (the intervening add_executable ends the
#     block). Prose that merely MENTIONS the marker inside an attached comment
#     block still exempts, exactly as it did before this widening.
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
  # fixtures (test-fixture false-positive observed 2026-07-01 on
  # rtc_tools/test/test_plot_rtc_log.py).
  RTC_TOUCHED=$(echo "$CHANGED_SRC" | grep -E '^rtc_[a-z_]+/' | grep -vE '(^|/)tests?/' || true)
  # ARCH-4 target set is derived dynamically: any non-rtc_* package whose
  # package.xml depends on at least one rtc_* package. Previously this was
  # a hardcoded "^ur5e_*/" prefix which silently lost coverage after the
  # ur5e_bringup → integrated_bringup / ur5e_hand_driver → udp_hand_driver
  # renames (observed 2026-05-03).
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
  # robot name (whole-file-scope false positives observed 2026-07-17).
  #
  # Negation filter: lines that *forbid* the term (header comments like
  # "must NOT test UR5e", "no ur5e-specific code", "robot-agnostic") are
  # the rule itself, not a violation. Without this filter the hook punished
  # well-intentioned prohibitive docstrings (2026-05-07).
  #
  # Known residual false-positive class: robot names in PROSE (docstrings,
  # launch defaults, "e.g. UR5e") that the negation filter does not cover.
  # Triage on a hit, in order: (1) `git show HEAD:<file>` — if the line
  # pre-existed this turn, it is a harness false positive; report it as a
  # harness-pruning signal (CLAUDE.md §Claude Code), do not "fix" working code.
  # (2) If the hit is prose you just wrote in rtc_*, reword robot-neutrally
  # and push the concrete example down to a consumer package's docs/config.
  #
  # Case (2) exercised 2026-08-30 on a NEW rtc_mujoco_sim header (object_pool):
  # a rationale comment naming a scene path plus a "measured on the <robot>
  # scene" note both fired, because an untracked file is screened in full. Both
  # were genuine — an agnostic public header should not carry a robot asset
  # path that rots on rename — and rewording them by scene SHAPE ("16-dof
  # arm-plus-hand") while leaving the named example in that package's README
  # made them more useful, not vaguer. Identical sentences already sitting in
  # that package's TRACKED files did not fire, which is the gate working as
  # designed: it asks only "did YOU add this", never "is this file clean".
  # Decision 2026-08-30: keep this ratchet. Do not widen scope to whole files
  # (that is the 2026-07-16 regression above) and do not add a
  # measurement-prose exemption marker — rewording cost two comment edits and
  # improved both.
  #
  # Confirmed a third time 2026-09-22 on a NEW rtc_controllers field comment
  # (catching_params.hpp) that named the robot whose shipped profile triggered
  # the incident: rewording pushed the incident record down to the plan and
  # left a header that is agnostic, which is what the rule asks for. That turn
  # also found the real defect, and it was NOT here — .claude/rules/arch-source.md
  # carried a judgment line reading "robot names in prose are not a violation
  # (known gate false positive)", contradicting this SSoT. An agent following
  # it leaves the line in place, and because a blocked run does not advance the
  # watermark, the next turn blocks on the same line — forever. The rule file
  # was corrected instead of this scope. Do not re-derive the comment exemption
  # from that sentence; it no longer exists.
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    [ -f "$f" ] || continue
    # Real file line numbers of added lines, from the `+c,d` side of each hunk
    # header. Kept as line numbers rather than grepping the raw '+' text so the
    # report still points at a location the agent can open.
    if git ls-files --error-unmatch "$f" >/dev/null 2>&1; then
      ADDED_LINES=$(git diff -U0 "$VERIFY_BASE" -- "$f" 2>/dev/null | awk '
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
      ADDED=$(git diff -U0 "$VERIFY_BASE" -- "$f" 2>/dev/null | grep '^+' | grep -v '^+++' || true)
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

# --- Phase 0c: PROC-8 — the shared test wait helper stays sleep-only ---
# rtc_base/test/include/rtc_base/testing/ holds sleep-poll waits four packages
# share. Teaching one to pump an executor conflates two different primitives:
# ur5e_bt_coordinator's fixture owns the sole spinner thread (a second driver of
# the same executor is UB in rclcpp), and udp_hand_driver's aux-lane test proves
# a timer sits OUTSIDE the default callback group — pumping the default executor
# there would service it and the assertion would prove nothing. The repo merged
# and reverted that conflation once already (#356). Until this gate existed the
# only thing standing in the way was a header comment, which the next person to
# trim comments would have removed with no sensor firing.
#
# Scoped to the helper headers themselves, not their callers: "caller must not
# wrap this in a spin loop" is the other half of PROC-8, but greppable forms of
# it (a spin_some anywhere in a test that also waits) are overwhelmingly
# legitimate. That half stays prose. Deliberately NOT skipped on PURE_FORMAT —
# clang-format cannot introduce a spin call, so there is no false-positive class
# to suppress, and a real one must never ride in under a format commit.
#
# Labelled 0c and placed last of the Phase 0 sensors so the section labels run in
# file order (#358). It arrived as a second "Phase 0b" next to the ARCH-6 sensor,
# and invariants.md PROC-8 pointed at that ambiguous name. Position is free here:
# every Phase 0 sensor only appends to ARCH_VIOLATIONS, which is not read until
# the report.
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
# Skipped on pure-format commits — there is no semantic delta to mirror in
# READMEs, and CMake/package.xml co-update triggers (new .cpp file, new
# find_package) cannot fire because pure-format excludes file adds.
if [ "$PURE_FORMAT" -eq 0 ]; then
# Hoisted out of the per-package loop: these two git queries are repo-wide and
# identical on every iteration, so computing them once and re-filtering by
# package below saves one `git diff` spawn per changed package each turn.
STRUCT_AD=$(git diff --diff-filter=AD --name-only "$VERIFY_BASE" 2>/dev/null || true)
ADDED_A=$(git diff --diff-filter=A --name-only "$VERIFY_BASE" 2>/dev/null || true)
for pkg_dir in $CHANGED_PKGS; do
  # README.md co-update -- NON-BLOCKING checklist, and only for public-surface
  # changes. A src/-only edit (internal refactor, bug fix, private-impl change)
  # carries no doc-visible delta, so requiring a README bump there was pure
  # over-blocking. We flag only when the change plausibly alters documented
  # behavior/usage: a public header (include/), launch/ or config/, a source
  # file add/delete (structural), or package.xml (deps/exec surface).
  # A doc kept beside the headers (rtc_math/include/rtc_math/se3/README.md) is
  # not surface: editing it used to ask whether the README reflected the edit.
  PKG_PUBLIC=$(echo "$CHANGED" | grep -E "^${pkg_dir}/(include|launch|config)/" \
                 | grep -vE '\.md$' || true)
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
# Scoped to the CHANGED files, never the whole corpus. CI scans everything;
# a Stop hook that did the same would let a defect in an untouched file --
# or a gitignored scratch note invisible to `git status` -- block every turn
# with no way out.
DOC_FAILURES=""
DOC_FILES=()
while IFS= read -r d; do
  [ -n "$d" ] && [ -f "$d" ] && DOC_FILES+=("$d")
done <<< "$CHANGED_DOCS"
# Resolved relative to this hook, not to PROJECT_DIR: the script ships in the
# same repository as the hook, so this keeps working when the two are pointed
# at different trees (as the routing tests do).
HOOK_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
VALIDATE_DOCS="$HOOK_DIR/../../repo_scripts/scripts/validate_docs.py"
[ -f "$VALIDATE_DOCS" ] || VALIDATE_DOCS="$PROJECT_DIR/repo_scripts/scripts/validate_docs.py"
if [ "${#DOC_FILES[@]}" -gt 0 ]; then
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
          git diff -U0 "$VERIFY_BASE" -- "$d" 2>/dev/null | awk -v F="$d" '
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
# touched -- and bare refs on unchanged lines of the constitution itself -- which
# the per-file, added-line scope above cannot see; CI's corpus scan would, one
# push later. So when a constitution's numbered-heading set moved, resolve every
# section ref in the tracked corpus, unnarrowed. D13 only (`--section-refs`): other
# pre-existing debt in untouched files still cannot block the turn.
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

# --- Phase 1c: path-scoped rules must be able to fire ---
# A .claude/rules/*.md whose `paths:` globs match nothing never loads, and the
# failure is silent: the file is present, readable and reviewable, and the only
# symptom is guidance quietly not arriving. That is the same shape as the
# constitution copy that stopped at 7 of 9 RT rules (#213), so it blocks.
#
# This gate is the half of the sensor pair that needs no session. The other
# half -- did the rule ACTUALLY load -- is the InstructionsLoaded hook
# (.claude/hooks/log-instructions-loaded.sh), whose log is the ground truth,
# because the glob semantics here are an independent reimplementation of the
# runtime matcher rather than the matcher itself.
#
# Scoped to the changed rule files, like every other Phase 1 sensor: a rule
# that was already dead in a file this turn did not touch is not this turn's
# to repair.
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
    HOOK_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
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

# --- Phase 1d: test isolation gates (CI's test-domain and fixture checks) ---
# validate_test_domains.py (#401: one ROS_DOMAIN_ID per package, and every
# participant-opening test claims one) and validate_test_fixtures.py (#454: no
# fixture resolves a package outside the repo and deps.repos) ran only in CI.
# Phase 2 runs repo_scripts' own tests only when repo_scripts changed, but
# both gates judge OTHER packages' CMakeLists and test sources. So a claim
# that collided passed every local check and first failed on the PR, twice:
# #513 (two integrated_bringup tests with no claim) and #571 (a new test on
# udp_hand_driver's domain 54).
#
# Whole-repo verdicts, unlike the doc gate's changed-lines scope: a collision
# is between two packages, and the one this turn did not touch is half of it.
# That cannot block an unrelated turn on inherited debt, because CI keeps main
# clean on both. Each run takes about 0.5 s, so the trigger is only a
# relevance filter: a build file, a pytest conftest, or a test source or
# fixture header.
#
# Resolved in PROJECT_DIR only, not next to the hook as Phase 1b/1c do. These
# scripts judge the tree they live in (repo root = their parents[2]), so a copy
# beside the hook would judge the hook's repo, not the checkout under test. The
# routing tests rely on this: they copy the scripts into a fixture.
TESTGATE_FAILURES=""
CHANGED_TESTGATE=$(echo "$CHANGED" | grep -E '(^|/)(CMakeLists\.txt|conftest\.py|colcon\.pkg)$|(^|/)test(ing)?/' || true)
if [ -n "$CHANGED_TESTGATE" ]; then
  for gate in validate_test_domains validate_test_fixtures; do
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

# --- Phase 2: Build + test, with PROC-3 fallback for rtc_base / rtc_msgs ---
# RTC_VERIFY_SKIP_BUILD lets repo_scripts/test/test_verify_changes.sh exercise
# the routing without a colcon workspace. It is never set in normal operation.
#
# RTC_VERIFY_BUILD_CMD is the same kind of seam for the build call itself: an
# executable taking build.sh's arguments. The suite injects a stub that exits
# with a chosen code, which is the only way to reach the branches below without
# a colcon workspace -- they were the one region of this hook with zero
# coverage, because SKIP_BUILD blanks BUILD_PKGS and skips Phase 2 whole. Same
# risk profile as SKIP_BUILD: never set in normal operation.
TEST_FAILURES=""
BUILD_CMD="${RTC_VERIFY_BUILD_CMD:-./build.sh}"

# Run the build, keeping BOTH things the old call threw away: the exit code and
# the output. `if ! timeout 180 ./build.sh -p "$pkg" >/dev/null 2>&1` swallowed
# $? in the `if !` and the log in the redirect, so a bound-kill (124) and a
# compile error emitted the byte-identical "<pkg>: build failed" -- #435. Same
# machine, same commit, same package: 179.4s killed while a long build competed
# for CPU, 2.5s idle. The verdict was set by machine load, not by the code, and
# nothing in the report said so.
# Sets BUILD_RC and BUILD_LOG; callers must rm the log.
#
# --tests is not optional here. build.sh does not build tests by default (they
# are more than half the compile time), and `colcon test` on a package built
# without them reports "0 tests, 0 failures" -- which every branch below reads
# as a pass. A build that precedes a test run must ask for the tests.
run_build() {  # $1 = timeout seconds, rest = args for the build command
  local secs="$1"; shift
  BUILD_LOG=$(mktemp)
  BUILD_RC=0
  timeout "$secs" "$BUILD_CMD" "$@" --tests >"$BUILD_LOG" 2>&1 || BUILD_RC=$?
}

# ── Build order ─────────────────────────────────────────────────────────────
#
# BUILD_PKGS comes out of `sort -u`: name order. The packages are built AND
# tested one at a time, so in name order a package is tested before a package
# it depends on has been rebuilt — against the library installed before, with
# the headers of the tree being graded (a symlink install serves headers from
# the source). 2026-10-02 (#631): an enumerator removed in rtc_controllers
# shifted the values after it; integrated_bringup, first by name, was compiled
# with the new values, loaded the old library, printed "ready" for kPublished
# and went red. A second --run passed. The same order turns into a false GREEN
# when the stale library happens to satisfy the dependent's tests: its verdict
# is recorded, its own binaries are not rebuilt by the dependency's build, and
# nothing asks for it again.
#
# So the order is colcon's own (`colcon list --topological-order`, over this
# repository: package.xml build, run and test dependencies; 0.2 s, no ROS
# environment needed). A package that list does not name keeps its place after
# the ones it does. Without a usable list the order stays the name order and
# the run SAYS so — a silent fallback would be the defect again, unannounced.
# One package has no order and costs no call.
#
# The query runs from the workspace root with colcon's log switched off. Every
# colcon verb writes a log/ tree into the directory it is called from, `list`
# included, and this hook's directory is the repository: called from here it
# left src/<repo>/log behind on every multi-package --run (AGENTS.md §9.1).
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

# A package whose build did not finish leaves its OLD binaries installed. What
# depends on it would be built against those and tested against them, and a
# green from that would be recorded as this tree's verdict — the same hole as
# the name order, reached through a failed build. So the packages above a
# failed one are held back: not built, not tested, reported, no verdict.
# BUILD_HELD is "<pkg>:<the package whose build failed>" entries. Without a
# usable `colcon list` nothing is held (the order note above already said the
# dependencies are unknown).
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

# Names (of "$@") whose build tree was configured WITHOUT tests.
#
# run_build asks for the tests, but the verdict must not rest on every build
# path remembering to. A package configured with BUILD_TESTING=OFF has nothing
# for `colcon test` to run, and `colcon test-result` then answers either
# "0 tests, 0 failures" or from the result files an earlier build left behind
# -- both of which the branches below read as a pass. So read what the tree
# says, after the build and before the tests.
# No CMakeCache.txt (ament_python, or a package this run did not build) says
# nothing either way and is not reported.
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
#
# --return-code-on-test-failure is the verdict. Without it `colcon test` exits 0
# whether the tests passed or not, and the verdict has to be read back out of
# result files -- which is where this hook was blind from its first version
# until 2026-10-01: the per-package branch asked `colcon test-result
# --packages-select <pkg>`, an option that verb does not have. colcon answered
# with a usage error, the error text holds no "<n> failures", and so every
# package whose tests RAN was recorded green, failing or not. The suite could
# not see it: RTC_VERIFY_TEST_CMD stands in for both colcon calls at once, so
# the real command line was never executed by a test (cases 66-68 now run it
# against a `colcon` that refuses arguments the real one refuses). With the
# flag the exit code is the tests' own: 0 = every selected package ran and
# none failed, 1 = a test failed.
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
# Exit 0 alone is not green. `colcon test --packages-select <a name it does not
# know>` warns, tests nothing and exits 0 ("Summary: 0 packages finished"), and
# a package that never started is not in the count either. So a pass needs the
# positive half too: colcon's own summary line, counting as many packages as
# were asked for ($1; empty = any number above zero, for the workspace-wide
# run). No summary line -- a colcon configured without that event handler --
# reads as "short", which blocks instead of guessing.
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
# exit code above.
#
# "Written by this run" is not optional: ctest keeps one Testing/<stamp>/ per
# run and nothing removes them (build/<pkg> holds dozens), so a failure fixed
# weeks ago is still on disk and `colcon test-result` still lists it.
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

# Last lines of a failed build, indented for the report. Backslashes are doubled
# because the report is emitted with `echo -e`, which would otherwise eat the
# "\n" inside a compiler-quoted string literal and mangle the very line the
# agent needs to read.
build_log_tail() {
  tail -n 15 "$BUILD_LOG" | sed -e 's/\\/\\\\/g' -e 's/^/      /'
}

# What separates "this build is broken/too slow" from "it lost a CPU race": the
# classification above only says WHICH kind of failure it was, not whether a
# 124 was the code's fault. loadavg plus a live build/compiler count is the
# cheapest thing that answers it at the moment of the kill.
build_contention_evidence() {
  local load busy
  load=$(cut -d' ' -f1-3 /proc/loadavg 2>/dev/null || echo "unavailable")
  if command -v pgrep >/dev/null 2>&1; then
    # Match process NAMES (-x), not command lines (-f). pgrep -f would also match
    # any ancestor shell whose command line happens to contain this pattern --
    # including the one that invoked the hook by hand, which reported 3 rivals on
    # an idle box where exactly one build was running. An evidence line that
    # inflates under the very conditions it exists to measure is worse than none.
    # Counted after the kill, so this build's own dying children can still be in
    # it -- an indicator of contention, not an exact count of rivals.
    busy=$(pgrep -c -x '(colcon|cmake|ninja|make|cc1plus|cc1|ld)' 2>/dev/null || true)
  else
    busy="?"
  fi
  printf 'loadavg %s, %s build/compiler processes running at kill time' "$load" "${busy:-0}"
}

# A build already writing this workspace -- looked for BEFORE starting ours.
#
# Evidence after a kill (above) explains a 124; it does not prevent the race.
# Observed 2026-09-14: the main agent started `./build.sh -p <3 pkgs>` as a
# background SHELL task and ended its turn. Shell tasks do not defer (see
# "Background agents still writing the checkout"), so this hook built the same
# packages beside it and was killed at the bound (loadavg 15.5). A timeout is
# the mild outcome: two colcon runs also write the same build/ and install/
# trees, so a concurrent ninja or a half-installed package can surface as a
# compile error or a test verdict that belongs to neither run.
#
# So when one is found the hook does not build at all -- it blocks with a
# message naming the rival and saying to wait for it. Block, not defer, for the
# reason shell tasks do not defer: a wrong block costs a turn, a wrong defer an
# unverified tree. Nothing is dropped; the watermark only advances on a pass.
#
# Matched by process NAME (-x, for the reason given above) and then by working
# directory. build.sh cds into the workspace before colcon and stays there for
# the post-build RT check; the AGENTS.md §9.1 form runs colcon from the
# workspace root. A colcon pointed here from another directory with absolute
# --build-base/--install-base is NOT recognised, and `bash build.sh` is named
# "bash" (only a direct-shebang exec keeps the script's name) -- its colcon is
# still seen once it starts. Prints "<pid>: <cmdline>".
workspace_build_rivals() {
  command -v pgrep >/dev/null 2>&1 || return 0
  local ws pid cwd cmd
  ws=$(cd "$WORKSPACE" 2>/dev/null && pwd -P) || return 0
  for pid in $(pgrep -x '(colcon|build\.sh)' 2>/dev/null || true); do
    cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null) || continue
    [ "$cwd" = "$ws" ] || continue
    cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | cut -c1-120) || cmd="?"
    printf '%s: %s\n' "$pid" "$cmd"
  done
}
# A simulator running from this workspace -- looked for with the build rivals.
#
# Observed 2026-09-26 (dynamic_catching S8-E): the agent ran the success-rate
# sims as a background SHELL task, which does not defer this hook, and any turn
# end with a change to grade would have built and tested beside them. A host
# busy with colcon slows the MuJoCo sim below real time (RTF 0.29-0.74
# measured when another session's tests overlapped), the sim's ball stamps
# follow sim time and fall behind the wall clock, and the controller discards
# the input as stale -- catches fail for a reason that is the rig, not the code
# (5/28 loaded trials caught vs 13/20 re-run).
#
# DEFER, not block, unlike a rival build: a build ends on its own, a sim does
# not -- the user may hold one open for a visual check for as long as they
# like, and a block would push the agent to stop a sim that is not its own.
# The missing verdict is not asked for, the other gates still run (and still
# block), and on a pass the watermark is NOT advanced, so the first turn end
# with no sim grades everything changed meanwhile -- the background-agent
# deferral's rule. (The turn end no longer builds; what it defers since
# 2026-10-01 is the block that asks for --run, which could only be answered by
# building beside the sim. --run itself refuses outright.)
#
# Matched by the kernel's 15-character process name, then by path: the node's
# executable (/proc/<pid>/exe, physical) under the physical workspace, or any
# command-line argument under the workspace as either the logical path colcon's
# setup scripts put in AMENT_PREFIX_PATH or the physical one (a workspace
# reached through a symlink shows the logical path in argv). A sim from another
# workspace is ignored. Prints "<pid>: <cmdline>".
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
# The sim check above sees a simulator that is RUNNING. An evaluation that
# launches one simulator per unit has none running between two units, for a
# few seconds each time -- and a turn that ends on "unit N done" ends exactly
# there. Observed 2026-09-30: nine turn ends in a row started `colcon test`
# in that gap, each ran 2-3 minutes beside the next unit, and one unit was
# aborted by its own host-load watch (RTF 0.944, `colcon test --packages-select
# rtc_tools` named as the cause).
#
# So the driver of such a run says so: a line "<pid> [<start time>]" in
# <workspace>/.rtc-verify-hold, where the start time is field 22 of
# /proc/<pid>/stat. While a listed process is alive, build/test is deferred on
# the simulator's terms (other gates run, watermark kept). A dead pid, or a
# live one whose start time differs (the pid was reused), holds nothing, so a
# driver that died without cleaning up cannot switch the gate off.
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
      # comm may hold spaces; what follows the LAST ')' starts at field 3.
      now=$(sed 's/^.*) //' "/proc/$pid/stat" 2>/dev/null | cut -d' ' -f20)
      [ "$now" = "$start" ] || continue
    fi
    cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | cut -c1-120) || cmd="?"
    printf '%s: %s\n' "$pid" "${cmd% }"
  done < "$HOLD_FILE"
}

# ── Package verdict reuse ───────────────────────────────────────────────────
#
# A package that built and tested green is not built and tested again while
# the packages are what they were graded as. The key is the content of EVERY
# package directory of the working tree (every file of each, by blob) -- this
# package or any other, source or not -- so an edit anywhere in any package
# re-grades all of them, as before. What the key leaves out:
#   * the repository-level files (docs/, agent_docs/, .github/, .claude/, the
#     root): no package's build reads them, and the turn that fixes a plan
#     document after the code passed is the common case this serves;
#   * Markdown inside a package (*.md, at any depth). The other common case:
#     the code passes, then the package README is brought up to date with it
#     (PROC-1 asks for exactly that order) -- and with the README in the key
#     that edit voided the verdict the code had just earned, a full build and
#     test for a file neither reads. Checked 2026-10-01: no CMakeLists installs
#     or configures a .md, and no test outside repo_scripts opens one. A test
#     that starts to read a package's Markdown has to take it out of this
#     exemption; the docs gates (Phase 1b) grade the .md itself either way.
# repo_scripts is the exception to both -- its tests run the validators and
# this hook against the repository itself, Markdown included -- so its key is
# the whole tree, as is PROC-3's.
#
# The key is NOT the diff against the watermark. That one (the blobs of the
# changed files) named the same key for different packages once the watermark
# had moved: a.py changed against B1 and graded, then b.py committed and the
# watermark advanced past it without a build -- a.py is again the whole change
# set, and b.py was never graded.
#
# Only a green build AND a green test is remembered, per package, at the moment
# it happens: a turn blocked by another gate keeps the verdicts it did earn.
pkg_content_key() {  # $1 = a tree id from work_tree_id
  local pkgs
  pkgs=$(git_scratch ls-tree -r --name-only "$1" 2>/dev/null \
           | sed -n 's|^\([^/]*\)/package\.xml$|\1|p' || true)
  # One "<mode> <type> <blob>\t<path>" line per file of a package directory,
  # Markdown left out. quotePath off: a quoted non-ASCII path ends in `"`, and
  # its ".md" would not be seen.
  git_scratch -c core.quotePath=false ls-tree -r "$1" 2>/dev/null \
    | awk -v pkgs="$pkgs" '
        BEGIN { n = split(pkgs, a, "\n"); for (i = 1; i <= n; i++) if (a[i] != "") keep[a[i]] = 1 }
        {
          path = $0; sub(/^[^\t]*\t/, "", path)
          top = path; sub(/\/.*/, "", top)
          if ((top in keep) && path !~ /\.md$/) print
        }' \
    | git hash-object --stdin 2>/dev/null || true
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
remember_pkg_verdict() {  # $1 = package, $2 = key
  [ -n "$2" ] || return 0
  {
    grep -v "^$1 " "$PASS_PKGS_FILE" > "$PASS_PKGS_FILE.tmp" || true
    printf '%s %s\n' "$1" "$2" >> "$PASS_PKGS_FILE.tmp"
    mv "$PASS_PKGS_FILE.tmp" "$PASS_PKGS_FILE"
  } 2>/dev/null || true
  remember_artifact_stamps "$1"
}
# A green build and test, recorded -- if what the key names is still there.
#
# The key ($2) was taken when this call started; the build and the tests ran
# minutes later, on whatever the files were by then. --run is made to be
# backgrounded, so an edit inside a package while it runs is ordinary: the
# compiler then built T1 and the verdict would be filed under T0's key, to be
# honoured whenever the tree is T0 again (the edit reverted, a stash, a
# checkout). So the key is taken again here and the verdict is recorded only
# if it has not moved; otherwise the package stays owed and the report says
# why. An edit made AND undone inside the run is not seen -- the two keys
# agree -- which is the price of not hashing the tree around every compile.
remember_tested_verdict() {  # $1 = package (or PROC-3), $2 = key at the start
  local tree_now key_now
  [ -n "$2" ] || return 0
  tree_now=$(work_tree_id)
  key_now=$(change_set_key "$1" "$tree_now" "$(pkg_content_key "$tree_now")")
  if [ "$key_now" != "$2" ]; then
    TEST_FAILURES="${TEST_FAILURES}  - ${1}: built and tested green, but its files changed while that ran — no verdict recorded: what was tested is not what is there now. Run '${RUN_CMD}' again and leave the packages alone until it finishes.\n"
    return 0
  fi
  remember_pkg_verdict "$1" "$2"
}
BUILT_PKGS=""
REUSED_PKGS=""
BUILD_SWITCHED_OFF=""
PROC3_KEY=""

PROC3=$(echo "$BUILD_PKGS" | tr ' ' '\n' | grep -E '^(rtc_base|rtc_msgs)$' || true)
if [ -n "${RTC_VERIFY_SKIP_BUILD:-}" ]; then
  # Emit the routing decision before discarding it. Blanking BUILD_PKGS is what
  # lets the suite run without a colcon workspace, but it also made the build
  # routing structurally unobservable -- so the central claim of the untracked
  # scoping ("a new header under <pkg>/include/ IS built, a scratch file is
  # not") had no assertion behind it. This probe is the seam the tests read.
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

# ── Turn end: evidence, not execution ───────────────────────────────────────
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
  TEST_FAILURES="${TEST_FAILURES}  - build/test verdict missing for: ${OWED} — and a build is running in this colcon workspace (${WORKSPACE}):\n$(report_procs "$RIVALS")\n    If it is your '${RUN_CMD}', wait for it to finish (wait on that task), then end the turn again. If it is another build, run '${RUN_CMD}' once it is over.\n"
elif [ -z "$RUN_MODE" ] && [ -n "$SIM_RIVALS" ]; then
  # Deferred, not failed: reported at the end (see workspace_sim_rivals).
  SIM_DEFERRED=1
elif [ -z "$RUN_MODE" ]; then
  TEST_FAILURES="${TEST_FAILURES}  - build/test verdict missing for: ${OWED} — the turn end does not build or test. Run '${RUN_CMD}' (it builds with --tests, runs colcon test and records the verdict; if it will take long, background it and wait for it), then end the turn again. An edit inside any package after that run voids the verdict (a *.md does not).\n"
elif [ -n "$RIVALS" ]; then
  TEST_FAILURES="${TEST_FAILURES}  - build/test NOT run — a build is already running in this colcon workspace (${WORKSPACE}):\n$(report_procs "$RIVALS")\n    Building beside it would race it for CPU and write the same build/ and install/ trees, so neither verdict could be trusted. Wait for it to finish (if it is your own background task, wait on that task), then run this again.\n"
elif [ -n "$SIM_RIVALS" ]; then
  TEST_FAILURES="${TEST_FAILURES}  - build/test NOT run — a simulator from this colcon workspace (${WORKSPACE}) is running, or a measurement holds the host (${HOLD_FILE}):\n$(report_procs "$SIM_RIVALS")\n    Building beside it would slow the sim below real time and corrupt what it measures (ball input goes stale, catches fail). Run this again once it is over; do not stop a sim or a measurement that is not yours to get past this.\n"
elif [ -n "$PROC3" ]; then
  # PROC-3: one broad rebuild, then every test of the workspace.
  # All colcon invocations run from $WORKSPACE so build/install/log land in the
  # colcon ws root (AGENTS.md §9.1), not in this repo's cwd.
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
    # header). A bound-killed build (exit 124) is reported as UNVERIFIED,
    # separately from a build that really broke -> both exit 2, different
    # messages.
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

    # One package at a time, each with the box to itself.
    #
    # The packages that matter run their own tests side by side (colcon.pkg:
    # ctest -j / pytest -n), and inside each a test that asserts a measured
    # wall-clock budget is RUN_SERIAL. That isolation ends at the package: it
    # says nothing about the tests of ANOTHER package colcon runs beside it.
    # One `colcon test` over every changed package was tried (2026-10-01) and
    # taken out again in review -- it put those budget tests under the load of
    # a neighbour's suite, and one call has one exit code, so a single
    # load-induced miss left every package of the batch without a verdict.
    # Here each package gets its own call, its own verdict, and the cores.
    #
    # The exit code is the verdict (see run_colcon_test and the PROC-3 path
    # above): timeout / launch failure / real test failure are told apart by
    # it, not inferred from result files.
    #
    # RTC_VERIFY_TEST_CMD is the test-side twin of RTC_VERIFY_BUILD_CMD: an
    # executable taking the package name, whose exit code stands for the test
    # run's and whose output for its result summary. It is what lets the suite
    # reach a GREEN package without a colcon workspace, and it replaces the
    # colcon command line -- which is why that line has cases of its own, run
    # against a stand-in `colcon` on PATH. Never set in normal operation.
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
    DELETED=$(git diff --diff-filter=D --name-only "$VERIFY_BASE" 2>/dev/null \
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
# Lint gate for every changed *.sh. Runs at --severity=warning so info-level
# notes (SC1091 source-following, intentional SC2086 word-splitting in RT
# cpu-list code) do not block. The repo-root .shellcheckrc (external-sources=true,
# disable=SC2034) is found by walking up from each script. Fails open when the
# linter is absent — missing tooling must not hard-block a turn (mirrors the
# clang-format fail-open above). No comment line here may begin with the word
# "shellcheck": the linter parses it as a directive and aborts the whole file.
#
# Whole file, unlike the doc phase's added-lines narrowing: shellcheck reports a
# finding where the symptom is, not where the cause is (a `declare -A expected`
# in one test surfaced as SC2178 on an untouched helper; deleting an assignment
# surfaces as SC2154 at the unchanged use). Narrowing to added lines would trade
# a false block for a missed one. The scope is fair only while no file carries a
# warning into a turn, and docs-validate.yml keeps that true by linting the whole
# tracked *.sh corpus with this same severity. So a finding here was either
# caused by the diff (maybe on a line it did not touch) or slipped past that
# gate; either way fix it (rename, or a `disable=` directive with a reason) --
# do not narrow this.
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

# --- Phase 5: formatter drift introduced by the change ---
# format-code.sh (PostToolUse) formats only what the Edit / Write tools touch. A
# file written through Bash -- heredoc, sed -i, a python rewrite script -- used
# to reach a commit unformatted with nothing downstream to notice: the only
# formatter call in this hook was the pure-format fast path, and CI runs none.
# Two such .py files reached main that way.
#
# Blocks only on drift the change INTRODUCED: the working file is not a formatter
# fixed point AND its $VERIFY_BASE blob either does not exist or was one. A file
# already unformatted at the base is debt this diff did not cause; grading it
# whole would block every unrelated touch of a legacy file -- the false-block
# class the doc phase's added-lines narrowing removed. (A rename reads as a new
# file and is graded whole.) Scope is CHANGED_SRC_BUILD, so untracked scratch
# outside the installed-source dirs is not graded. Fails OPEN when the formatter
# is missing or prints nothing (a syntax error, uvx unable to provision): a
# verdict needs output to compare, and this gate is style, not correctness.
#
# Compared byte-for-byte through files. `[ "$(fmt)" = "$(cat f)" ]` read a
# missing final newline or trailing blank lines as clean, because command
# substitution strips trailing newlines from both sides.
#
# At the turn end it gets whatever is left of the Stop budget: once $SECONDS
# passes RTC_VERIFY_FORMAT_DEADLINE_S the remaining files are listed as
# ungraded (non-blocking) rather than risking the SIGKILL that would end the
# turn with no report at all. --run has no budget to protect, and its build
# and tests alone can take longer than that deadline -- which would list every
# file as ungraded -- so it grades them all unless the variable is set.
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
if [ -n "$RULES_FAILURES" ]; then
  REPORT="${REPORT}Path-scoped rule cannot fire (repo_scripts/scripts/validate_claude_rules.py):\n${RULES_FAILURES}\n"
fi
if [ -n "$TESTGATE_FAILURES" ]; then
  REPORT="${REPORT}Test isolation gates (the CI checks, run whole-repo):\n${TESTGATE_FAILURES}\n"
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
if [ -n "$FORMAT_FAILURES" ]; then
  REPORT="${REPORT}Formatter drift introduced on changed sources:\n${FORMAT_FAILURES}\n"
fi

# Constitution split: AGENTS.md is the single tool-neutral constitution and
# CLAUDE.md imports it (`@AGENTS.md`), adding only Claude Code mechanisms. The
# parity reminder this replaces ("mirror a CLAUDE.md edit into AGENTS.md") dates
# from two hand-kept copies -- the copies went 4 commits stale, losing ARCH-7 and
# NUM-5 for non-Claude tools -- and under the import it would recreate exactly
# that duplication. What can still go wrong:
#   * the import is lost -- the line dropped, CLAUDE.md deleted, or AGENTS.md
#     deleted or moved: Claude Code silently loses the whole constitution, and
#     nothing else in the harness would say so -> blocking. Keyed on the BASE
#     carrying the import, not on CLAUDE.md being in the change set, so a
#     deletion or a rename of either file (which leaves CLAUDE.md untouched)
#     is graded too;
#   * a rule lands in CLAUDE.md, where no other tool reads it -> non-blocking,
#     since only the agent can tell a rule from a mechanism.
if git show "$VERIFY_BASE:CLAUDE.md" 2>/dev/null | grep -qx '@AGENTS.md'; then
  if [ ! -f CLAUDE.md ] || ! grep -qx '@AGENTS.md' CLAUDE.md; then
    REPORT="${REPORT}Constitution import missing:\n  - CLAUDE.md is gone or no longer has the line '@AGENTS.md' -- Claude Code would load none of AGENTS.md; restore it\n\n"
  elif [ ! -f AGENTS.md ]; then
    REPORT="${REPORT}Constitution import missing:\n  - CLAUDE.md imports '@AGENTS.md' but AGENTS.md is gone from the repository root -- Claude Code would load no constitution; restore it\n\n"
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
  log_timing "blocked" "$BUILT_PKGS" "$REUSED_PKGS"
  exit 2
fi

# No hard failure: surface the doc checklist as a non-blocking reminder and let
# the turn end. README co-update is a judgement call the agent/user makes, not a
# gate -- so it prints but never forces exit 2.
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
# Both were claimed once -- a run of the hook by hand, build off, in the real
# clone -- and every later stop passed the branch head unbuilt.
if [ -n "$BUILD_SWITCHED_OFF" ]; then
  echo "verify-changes: build/test was switched off (RTC_VERIFY_SKIP_BUILD) with packages waiting -- watermark kept, nothing remembered." >&2
  log_timing "pass-unbuilt" "" ""
  exit 0
fi
#
# The watermark moves under the same condition. It is written from HEAD as HEAD
# is NOW, and the change set was computed when this call started: a commit made
# while a backgrounded --run was building would be stepped over, graded by no
# gate, and the next turn end would diff against it and find nothing. With the
# tree unchanged, HEAD can only hold content this call graded. With it changed
# the pass stands for what was graded -- the package verdicts are recorded --
# and everything since the old watermark is graded again at the next call,
# reusing those verdicts.
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
fi
exit 0
