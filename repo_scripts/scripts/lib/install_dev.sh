#!/bin/bash
# install_dev.sh — Developer / debug tooling for install.sh
#
# 제공 함수:
#   install_vscode_debug_tools  — gdb + ptrace_scope policy (VS Code Attach)
#   install_tracing_tools       — lttng-modules-dkms + lttng-tools + ros-jazzy-ros2trace
#                                 + tracetools-launch/read + python3-bt2
#                                 + tracing group membership
#
# Caller scope 의존:
#   WORKSPACE, SET_PTRACE_SCOPE, SET_TRACING_TOOLS, apt_update_if_stale, 로거

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

# ── ros2_tracing stack (LTTng + tracetools + bt2) ─────────────────────────────
# Installs the apt packages required by `ros2 launch ... enable_tracing:=true`,
# adds the user to the 'tracing' group (mirrors the ptrace_scope policy pattern
# in install_vscode_debug_tools — group membership replaces sysctl tuning).
#
# Idempotent: apt-get install is no-op when packages are present; usermod
# only fires when the user is not already in the group.
install_tracing_tools() {
  info "Installing ros2_tracing stack (LTTng + tracetools + bt2)..."
  apt_update_if_stale

  # ── L1: kernel headers + dkms toolchain (lttng-modules-dkms builds against
  #        the running kernel; build-essential is *not* pulled in by the dkms
  #        package itself but is required for the out-of-tree build) ─────────
  local KERNEL_RELEASE
  KERNEL_RELEASE="$(uname -r)"

  sudo apt-get install -y \
      build-essential \
      dkms \
      linux-headers-"${KERNEL_RELEASE}" \
      > /dev/null 2>&1 \
    || warn "linux-headers-${KERNEL_RELEASE} unavailable — lttng-modules build may fail"

  # ── L2: LTTng userspace + kernel module (DKMS rebuilds on each kernel) ───
  sudo apt-get install -y \
      lttng-tools \
      lttng-modules-dkms \
      liblttng-ust-dev \
      babeltrace2 \
      python3-bt2 \
      > /dev/null 2>&1 \
    || warn "LTTng stack install reported an error — re-run with verbose apt to inspect"

  if command -v lttng >/dev/null 2>&1; then
    success "lttng-tools: $(lttng --version 2>&1 | head -1)"
  else
    warn "lttng binary not on PATH after apt install"
  fi

  if command -v babeltrace2 >/dev/null 2>&1; then
    success "babeltrace2: $(babeltrace2 --version 2>&1 | head -1)"
  else
    warn "babeltrace2 binary not on PATH after apt install"
  fi

  # python3-bt2 is the binding ctf_to_chrome_trace.py uses for structured
  # CTF parsing. Falls back to babeltrace2 CLI text parsing if missing.
  if python3 -c "import bt2" 2>/dev/null; then
    success "python3-bt2 binding importable"
  else
    warn "python3 -c 'import bt2' failed — converter will fall back to CLI parser"
  fi

  # ── L3: ros2_tracing apt packages (ros-jazzy-tracetools is pulled in
  #        transitively by rclcpp; the extras below add the launch action,
  #        CLI verb, and post-processing utilities) ───────────────────────────
  sudo apt-get install -y \
      ros-jazzy-ros2trace \
      ros-jazzy-tracetools-launch \
      ros-jazzy-tracetools-read \
      ros-jazzy-tracetools-trace \
      > /dev/null 2>&1 \
    || warn "ros-jazzy tracing packages install reported an error"

  if dpkg -s ros-jazzy-tracetools-launch >/dev/null 2>&1; then
    success "tracetools-launch installed (enables tracetools_launch.action.Trace)"
  fi

  # ── L4: tracing group membership (lttng-sessiond accepts non-root clients
  #        whose effective gid is 'tracing'; without this, kernel tracing
  #        always requires sudo). Mirrors the ptrace_scope policy pattern. ──
  if ! getent group tracing >/dev/null 2>&1; then
    warn "'tracing' group not found — lttng-tools install should have created it. Skipping group add."
  elif id -nG "$(id -un)" | tr ' ' '\n' | grep -qx tracing; then
    success "user '$(id -un)' already in 'tracing' group"
  else
    info "Adding user '$(id -un)' to 'tracing' group..."
    sudo usermod -a -G tracing "$(id -un)"
    success "user '$(id -un)' added to 'tracing' group"
    warn "Group membership requires re-login (or 'newgrp tracing' in the current shell)"
    warn "  Without it, 'enable_tracing:=true' with kernel events will fail with EPERM."
  fi

  # ── L5: kernel module load check (DKMS builds on install but does not
  #        autoload; lttng-sessiond loads on first kernel session) ─────────
  if modinfo lttng-tracer >/dev/null 2>&1; then
    success "lttng-tracer kernel module available (version $(modinfo -F version lttng-tracer 2>/dev/null || echo '?'))"
  else
    warn "lttng-tracer kernel module not yet built — DKMS may still be compiling"
    warn "  Check: sudo dkms status lttng-modules"
  fi
}
