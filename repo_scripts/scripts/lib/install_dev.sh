#!/bin/bash
# install_dev.sh — Developer / debug tooling for install.sh
#
# 제공 함수:
#   install_vscode_debug_tools  — gdb + ptrace_scope policy (VS Code Attach)
#   install_perf_tools          — linux-tools-* + hotspot + FlameGraph clone
#                                 + perf_event_paranoid sysctl
#
# Caller scope 의존:
#   WORKSPACE, SET_PTRACE_SCOPE, SET_PERF_TOOLS, apt_update_if_stale, 로거

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

[[ -n "${_INSTALL_DEV_LOADED:-}" ]] && return 0
_INSTALL_DEV_LOADED=1

# ── VS Code / GDB debug tools ────────────────────────────────────────────
install_vscode_debug_tools() {
  info "Installing GDB and debug tools for VS Code..."
  sudo apt-get install -y \
      gdb \
      gdb-multiarch \
      > /dev/null
  success "GDB installed"

  # ── ptrace_scope: Attach to running process ────────────────────────────
  # VS Code의 'Attach to Node' 런치 구성은 GDB ptrace를 사용합니다.
  # Ubuntu 기본값 ptrace_scope=1은 자식 프로세스 외 attach 거부.
  local PTRACE_CURRENT
  PTRACE_CURRENT=$(cat /proc/sys/kernel/yama/ptrace_scope 2>/dev/null || echo "unknown")

  if [[ "$SET_PTRACE_SCOPE" -eq 1 ]]; then
    info "Setting ptrace_scope=0 for VS Code Attach debugger..."
    echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope > /dev/null
    # 영구 적용 (sysctl.conf)
    if ! grep -q 'kernel.yama.ptrace_scope' /etc/sysctl.d/99-ptrace.conf 2>/dev/null; then
      echo 'kernel.yama.ptrace_scope = 0' | sudo tee /etc/sysctl.d/99-ptrace.conf > /dev/null
      sudo sysctl -p /etc/sysctl.d/99-ptrace.conf > /dev/null 2>&1 || true
    fi
    success "ptrace_scope set to 0 (VS Code Attach enabled, persists across reboots)"
    warn "Security note: ptrace_scope=0 allows any process to attach to another."
    warn "Only use this on a development machine, not a production/robot system."
  else
    if [[ "$PTRACE_CURRENT" != "0" ]]; then
      echo ""
      warn "ptrace_scope is currently ${PTRACE_CURRENT} (not 0)."
      warn "VS Code 'Attach to Node' debugger may fail with 'Operation not permitted'."
      warn "To enable: re-run with --ptrace-scope, or run manually:"
      warn "  echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope"
      echo ""
    else
      success "ptrace_scope=0 already set (VS Code Attach ready)"
    fi
  fi
}

# ── Linux perf + Hotspot (profiling) ──────────────────────────────────────────
# Mirrors install_vscode_debug_tools(): apt binaries (L1) + sysctl policy (L2).
# Idempotent: second run skips when binaries are present and paranoid is set.
install_perf_tools() {
  info "Installing Linux perf + Hotspot for profiling..."
  apt_update_if_stale

  # ── L1: apt binaries ────────────────────────────────────────────────────
  # linux-tools-$(uname -r) carries the version-matched perf binary; the
  # generic metapackage tracks the running kernel on Ubuntu but can lag.
  local KERNEL_RELEASE
  KERNEL_RELEASE="$(uname -r)"

  sudo apt-get install -y \
      linux-tools-generic \
      linux-tools-"${KERNEL_RELEASE}" \
      hotspot \
      > /dev/null 2>&1 \
    || warn "linux-tools-${KERNEL_RELEASE} not available — generic only (perf may print kernel mismatch)"

  if command -v perf >/dev/null 2>&1; then
    success "perf: $(perf --version 2>&1 | head -1)"
  else
    warn "perf binary not on PATH. Try: sudo apt install linux-tools-${KERNEL_RELEASE}"
  fi

  if command -v hotspot >/dev/null 2>&1; then
    success "hotspot installed (run: hotspot <session>/perf/perf.data)"
  else
    warn "hotspot binary not on PATH after apt install"
  fi

  # ── FlameGraph (browser-based viewer; faster than Hotspot for large traces) ──
  # Clone into <ws>/deps/FlameGraph/ — same isolation tier as fmt/mimalloc/aligator.
  # Idempotent: skip clone if already present, otherwise `git pull` to refresh.
  local FG_DIR="${WORKSPACE}/deps/FlameGraph"
  if [[ -d "$FG_DIR/.git" ]]; then
    info "FlameGraph already cloned at $FG_DIR — pulling latest..."
    (cd "$FG_DIR" && git pull --quiet 2>/dev/null) || warn "git pull failed in $FG_DIR (continuing)"
  else
    info "Cloning brendangregg/FlameGraph → $FG_DIR..."
    mkdir -p "${WORKSPACE}/deps"
    if git clone --depth=1 --quiet \
        https://github.com/brendangregg/FlameGraph "$FG_DIR" 2>&1 | tail -5; then
      success "FlameGraph cloned (run: ./repo_scripts/scripts/flame.sh)"
    else
      warn "FlameGraph clone failed — flame.sh will be unavailable"
    fi
  fi

  # ── L2: sysctl perf_event_paranoid (mirrors ptrace_scope policy) ────────
  local PARANOID_CURRENT
  PARANOID_CURRENT="$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo unknown)"

  if [[ "$SET_PERF_TOOLS" -eq 1 ]]; then
    info "Setting perf_event_paranoid=1 (non-root perf record enabled)..."
    echo 1 | sudo tee /proc/sys/kernel/perf_event_paranoid > /dev/null
    if ! grep -q 'kernel.perf_event_paranoid' /etc/sysctl.d/99-perf.conf 2>/dev/null; then
      echo 'kernel.perf_event_paranoid = 1' | sudo tee /etc/sysctl.d/99-perf.conf > /dev/null
      sudo sysctl -p /etc/sysctl.d/99-perf.conf > /dev/null 2>&1 || true
    fi
    success "perf_event_paranoid=1 (persists across reboots)"
    warn "Security note: lowers kernel sampling restriction. Dev machine only."
  else
    if [[ "$PARANOID_CURRENT" -gt 1 ]] 2>/dev/null; then
      warn "perf_event_paranoid=${PARANOID_CURRENT} — non-root perf record disabled."
      warn "  ros2 launch ... enable_perf:=true will require sudo at launch time."
      warn "  Fix permanently: re-run install.sh --perf"
    else
      success "perf_event_paranoid=${PARANOID_CURRENT} (non-root perf record OK)"
    fi
  fi
}
