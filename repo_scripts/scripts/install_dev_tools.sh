#!/usr/bin/env bash
# install_dev_tools.sh — install developer-only Python tools into the workspace .venv.
#
# Currently: ruff (formatter + linter, pyproject.toml is the SSoT).
# Idempotent. Skips when the requested tool is already present at the right version.
#
# Run after `source repo_scripts/scripts/setup_env.sh` (must have `.venv` active).
set -euo pipefail

if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  echo "[install_dev_tools] No active venv. Source setup_env.sh first." >&2
  exit 1
fi

python3 -m pip install --quiet --upgrade ruff
echo "[install_dev_tools] ruff: $(ruff --version)"
