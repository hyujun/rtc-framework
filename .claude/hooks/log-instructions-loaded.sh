#!/usr/bin/env bash
#
# InstructionsLoaded hook — records which CLAUDE.md / .claude/rules/*.md files
# actually entered context, and why.
#
# WHY THIS EXISTS
#
# Path-scoped rules (.claude/rules/*.md with `paths:` frontmatter) are the
# repo's only channel that fires *automatically* while editing a matching file.
# But the injection is invisible from inside a session in the one case that
# matters: after authoring a NEW rule, an agent cannot tell "my glob is wrong"
# from "the rule loaded before I created it". Observed in the #229 migration —
# rt-path.md was seen firing once, and two rules added in the same session were
# never observed at all, which forced the constitution de-duplication to be
# deferred to a fresh session purely for lack of a sensor.
#
# This hook is that sensor. `InstructionsLoaded` fires at session start AND on
# lazy load during a session, and its matcher distinguishes the load reason
# (session_start / nested_traversal / path_glob_match / include / compact).
#
# CONTRACT (why this script looks the way it does)
#
#   - The exit code is IGNORED for this event and no output is shown to the
#     user or to Claude. It is a side-effect-only event, so this script must
#     never be relied on to block anything, and must never be noisy.
#   - The event-specific stdin fields are NOT documented, so this logs the raw
#     payload verbatim rather than plucking a field name that may not exist.
#     Read the log with grep/jq; the field names are whatever the runtime
#     actually sends. Logging the raw line is also future-proof if they change.
#   - No matcher is registered in settings.json, so every load reason is
#     captured. If a future runtime requires an explicit matcher, this log will
#     simply stay empty -- and an empty log after a session that opened a
#     matching file IS the finding. In that case register the five matcher
#     values explicitly.
#
# HOW TO USE IT
#
#   1. Open a file the rule claims to match (Read is enough -- path-scoped
#      rules trigger on read, not only on edit).
#   2. grep the log for the rule's filename:
#        grep arch-source .claude/instructions-loaded.log
#      A hit means the glob matched and the rule is in context. No hit means
#      the rule did not load, and the glob (not the channel) is the suspect.
#
# The log lives at <project>/.claude/instructions-loaded.log and is covered by
# the repo-wide `*.log` gitignore entry, so it is never committed.

set -uo pipefail

PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
LOG="${PROJECT_DIR}/.claude/instructions-loaded.log"

# Collapse the payload to one line so the log stays greppable. JSON strings
# cannot contain a literal newline (they carry \n escapes instead), so
# stripping newlines is lossless for the payload itself.
PAYLOAD=$(tr -d '\r\n' 2>/dev/null || true)
[ -n "$PAYLOAD" ] || PAYLOAD='{}'

printf '%s %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$PAYLOAD" >>"$LOG" 2>/dev/null || exit 0

# Bound the file. This appends on every load event of every session, so without
# a cap it grows unbounded on a long-lived checkout. Keeping the tail is right:
# the question this log answers is always about the current session.
LINE_COUNT=$(wc -l <"$LOG" 2>/dev/null || echo 0)
if [ "${LINE_COUNT:-0}" -gt 500 ]; then
  tail -n 250 "$LOG" >"${LOG}.tmp" 2>/dev/null && mv -f "${LOG}.tmp" "$LOG" 2>/dev/null || true
fi

exit 0
