#!/bin/bash
# PreToolUse hook — block direct `colcon` invocations that would create
# build/install/log trees outside the colcon workspace root.
#
# Intent : enforce CLAUDE.md §9.1 (colcon CWD hard rule) automatically.
#          Previously this rule lived only in CLAUDE.md prose + a rules/*.md
#          file that had no actual injection wiring — so violations recurred.
# Trigger: PreToolUse on Bash. Reads {tool_input.command} from stdin JSON.
# Logic  :
#   1. Only inspect commands that invoke `colcon` directly. Wrapper scripts
#      (./build.sh / ./install.sh) are allowed unconditionally because they
#      cd into $WORKSPACE internally.
#   2. A direct colcon call is allowed if ANY of:
#      a. command contains `cd <ws-root> &&` (or via $HOME / ~)
#      b. command sets `--build-base <ws-root>/build`
#                  AND `--install-base <ws-root>/install` to absolute paths
#   3. Otherwise: exit 2 with the rule + recovery patterns inlined to stderr
#      so the model sees both in the same turn (this script is the SSoT for
#      the colcon-cwd rule body now that .claude/rules/colcon-cwd.md was
#      removed — see CLAUDE.md §9.1).
#
# Exit   : 0 = allow tool call. 2 = block + show stderr to model.

set -euo pipefail

WS_ROOT_ABS="/home/junho/ros2_ws/rtc_ws"
WS_ROOT_TILDE='~/ros2_ws/rtc_ws'
WS_ROOT_HOME='$HOME/ros2_ws/rtc_ws'

input="$(cat)"
cmd="$(printf '%s' "$input" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("tool_input",{}).get("command",""))' 2>/dev/null || true)"

[[ -z "$cmd" ]] && exit 0

# Step 1: only guard direct colcon invocations. Look for a `colcon` token
# at a command boundary (start of string, after `&&`, `||`, `;`, `|`,
# pipeline separator, or whitespace following one of these).
if ! printf '%s' "$cmd" | grep -qE '(^|[[:space:];&|]+)colcon[[:space:]]+(build|test|test-result)'; then
  exit 0
fi

# Step 2a: cd to ws-root anywhere before colcon? Accept absolute, tilde,
# or $HOME form.
if printf '%s' "$cmd" | grep -qE "cd[[:space:]]+(${WS_ROOT_ABS}|${WS_ROOT_TILDE}|\"${WS_ROOT_HOME}\"|${WS_ROOT_HOME})([[:space:]]|/|\$|&)"; then
  exit 0
fi

# Step 2b: explicit absolute --build-base + --install-base under ws-root.
if printf '%s' "$cmd" | grep -qE -- "--build-base[[:space:]]+${WS_ROOT_ABS}/build" && \
   printf '%s' "$cmd" | grep -qE -- "--install-base[[:space:]]+${WS_ROOT_ABS}/install"; then
  exit 0
fi

# Block. Inline the rule body so the model sees recovery patterns inline.
cat >&2 <<'EOF'
[BLOCKED by guard-colcon-cwd.sh] Direct `colcon` call without ws-root CWD.

CLAUDE.md §9.1 hard rule: `colcon build` / `colcon test` / `colcon test-result`
must run from the colcon workspace root (~/ros2_ws/rtc_ws). Running them
from inside src/rtc-framework/ creates a parallel build/install/log tree
that .clangd's CompilationDatabase picks up while the ws-root incremental
cache drifts — untraceable stale state.

Safe call patterns:

  1. Wrapper (preferred):
       ./build.sh -p <pkg>                 # internally `cd "$WORKSPACE"`

  2. Direct colcon with cd prefix:
       cd ~/ros2_ws/rtc_ws && \
         source ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/setup_env.sh > /dev/null 2>&1 && \
         colcon test --packages-select <pkg> --event-handlers console_direct+

  3. Absolute --build-base / --install-base (only if cwd cannot move):
       colcon build \
         --base-paths ~/ros2_ws/rtc_ws/src \
         --build-base ~/ros2_ws/rtc_ws/build \
         --install-base ~/ros2_ws/rtc_ws/install \
         --packages-select <pkg>

Note: Bash tool resets cwd between turns AND spawns a fresh shell that
loses ROS_DISTRO / AMENT_PREFIX_PATH / PYTHONPATH. For `colcon test`,
always prepend the setup_env.sh source above — otherwise every gtest
reports 0% PASS with `ModuleNotFoundError: No module named 'ament_cmake_test'`.

Post-incident verification (if you suspect a stray tree was created):

  ls /home/junho/ros2_ws/rtc_ws/src/rtc-framework/{build,install,log} 2>/dev/null

Any of those existing means colcon ran from the wrong cwd — delete them.
EOF

exit 2
