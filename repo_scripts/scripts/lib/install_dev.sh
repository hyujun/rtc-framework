#!/bin/bash
# install_dev.sh — Developer / debug tooling for install.sh
#
# 제공 함수:
#   install_vscode_debug_tools         — gdb + ptrace_scope policy (VS Code Attach)
#   install_code_intelligence_tools    — clangd + LLVM OpenMP headers (libomp-dev)
#                                         + resource-dir omp.h verification
#   install_tracing_tools              — lttng-tools + lttng-modules-dkms (DKMS
#                                         build w/ timeout + dpkg retry) +
#                                         ${ROS_PKG_PREFIX}-ros2trace +
#                                         tracetools-launch/read + python3-bt2
#                                         + tracing group + modprobe verify
#                                         (Secure Boot MOK mismatch diagnosis)
#   verify_lttng_tracer_loadable       — install_tracing_tools L5 helper
#
# Caller scope 의존:
#   WORKSPACE, ROS_PKG_PREFIX, SET_PTRACE_SCOPE, SET_TRACING_TOOLS,
#   apt_update_if_stale, 로거

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

# ── clangd code intelligence (clangd + LLVM OpenMP headers) ───────────────
# pinocchio/proxsuite include <omp.h>. clangd's own resource-dir ships the
# Clang intrinsic headers but *not* omp.h, so the preamble fails and Eigen
# typedefs cascade into 'aka int' / 'no member' noise (issue #193).
#
# The fix is to install LLVM's omp.h *into the clangd resource-dir* via
# `libomp-dev` — never to add GCC's compiler-internal include dir to .clangd:
# that also exposes GCC's xmmintrin.h/mmintrin.h ahead of clang's and breaks
# _mm_prefetch. `libomp-dev` is used instead of a version-pinned
# `libomp-N-dev` because Ubuntu's `clangd` and `libomp-dev` meta packages
# resolve to the same LLVM major (Noble: 18), so they stay coupled on upgrade.
#
# Idempotent: apt-get install is a no-op when the packages are present.
install_code_intelligence_tools() {
  info "Installing clangd code intelligence tools (clangd + LLVM OpenMP headers)..."
  apt_update_if_stale
  sudo apt-get install -y \
      clangd \
      libomp-dev \
      > /dev/null
  success "clangd + libomp-dev installed"

  # ── Verify omp.h landed in the resource-dir of the clangd actually on PATH.
  #    A non-apt clangd (e.g. a VS Code-bundled binary) has its own
  #    resource-dir, so the meta package may not cover it. ─────────────────
  local CLANGD_MAJOR OMP_HEADER
  CLANGD_MAJOR=$(clangd --version 2>/dev/null | grep -oP 'version \K[0-9]+' || echo "")
  if [[ -z "$CLANGD_MAJOR" ]]; then
    warn "clangd not found on PATH — skipping omp.h resource-dir verification."
    return 0
  fi

  OMP_HEADER=$(ls /usr/lib/llvm-"${CLANGD_MAJOR}"/lib/clang/*/include/omp.h 2>/dev/null | head -1)
  if [[ -n "$OMP_HEADER" ]]; then
    success "omp.h present in clangd-${CLANGD_MAJOR} resource-dir (${OMP_HEADER})"
  else
    warn "clangd ${CLANGD_MAJOR} is on PATH but no omp.h in its resource-dir"
    warn "  (/usr/lib/llvm-${CLANGD_MAJOR}/lib/clang/*/include/omp.h)."
    warn "IntelliSense will report 'omp.h file not found' plus a cascade of"
    warn "'no member' / 'aka int' errors in pinocchio/proxsuite headers."
    warn "Install the matching package manually:"
    warn "  sudo apt-get install -y libomp-${CLANGD_MAJOR}-dev"
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

  # ── L2a: LTTng userspace (silent — fast, deterministic apt path) ────────
  sudo apt-get install -y \
      lttng-tools \
      liblttng-ust-dev \
      babeltrace2 \
      python3-bt2 \
      > /dev/null 2>&1 \
    || warn "LTTng userspace install reported an error — re-run with verbose apt to inspect"

  # ── L2b: LTTng kernel module (DKMS build — verbose + timeout + retry) ───
  # dkms postinst has been observed to stall silently on fresh Ubuntu 24.04
  # (lttng-modules-dkms 2.14.0 + kernel 6.17 HWE) — `dkms status` shows
  # "added" without kernel/arch row, build/ directory empty, dkms-build
  # process idle with zero syscalls. The cause is in dpkg/debconf chain, not
  # kernel compatibility: `sudo dpkg --configure -a` consistently retries
  # and the DKMS build completes. The timeout below bounds the stall so
  # non-interactive runs do not hang forever.
  local DKMS_TIMEOUT=600
  info "Installing lttng-modules-dkms (DKMS build, up to ${DKMS_TIMEOUT}s; output kept for progress)..."
  if ! sudo timeout --kill-after=30 "$DKMS_TIMEOUT" apt-get install -y lttng-modules-dkms; then
    warn "lttng-modules-dkms install timed out or was killed — retrying via 'sudo dpkg --configure -a'"
    if sudo dpkg --configure -a; then
      success "dpkg --configure -a recovered the lttng-modules postinst (DKMS build completed on retry)"
    else
      warn "dpkg --configure -a also failed — lttng-tracer kernel module unavailable"
      warn "  Manual recovery: sudo apt-get remove --purge lttng-modules-dkms && sudo reboot && re-run --tracing"
    fi
  fi

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

  # ── L3: ros2_tracing apt packages (${ROS_PKG_PREFIX}-tracetools is pulled in
  #        transitively by rclcpp; the extras below add the launch action,
  #        CLI verb, and post-processing utilities). Package names are identical
  #        across humble/jazzy — only the distro prefix differs, so this tracks
  #        ROS_PKG_PREFIX like every other apt install in the installer. ───────
  sudo apt-get install -y \
      "${ROS_PKG_PREFIX}-ros2trace" \
      "${ROS_PKG_PREFIX}-tracetools-launch" \
      "${ROS_PKG_PREFIX}-tracetools-read" \
      "${ROS_PKG_PREFIX}-tracetools-trace" \
      > /dev/null 2>&1 \
    || warn "${ROS_PKG_PREFIX} tracing packages install reported an error"

  if dpkg -s "${ROS_PKG_PREFIX}-tracetools-launch" >/dev/null 2>&1; then
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

  # ── L5: kernel module load verification (modprobe --dry-run) ────────────
  # modinfo only checks file existence on disk. modprobe is the real gate:
  # on Secure Boot systems a DKMS-built module signed by a MOK that is NOT
  # enrolled in the kernel keyring (e.g. CN derived from a hostname that
  # changed since the previous MOK enrollment) loads with "Key was rejected
  # by service" even though `modinfo` and `dkms status` both report success.
  verify_lttng_tracer_loadable
}

# Helper for install_tracing_tools L5.
# Runs `modprobe --dry-run` and, on failure under Secure Boot, compares the
# module's signer CN against the enrolled MOK CNs and prints the exact
# `mokutil --import` + reboot recovery path.
verify_lttng_tracer_loadable() {
  if ! modinfo lttng-tracer >/dev/null 2>&1; then
    warn "lttng-tracer kernel module not found — DKMS may have failed silently"
    warn "  Check: sudo dkms status lttng-modules"
    return
  fi

  local DRY_OUT DRY_RC
  # NOTE: `cmd || true; rc=$?` always yields rc=0 (the `true`), silently masking
  # a modprobe rejection and disabling the Secure Boot diagnosis below. Capture
  # the real status via the if-condition (set -e safe).
  if DRY_OUT=$(sudo modprobe --dry-run -v lttng-tracer 2>&1); then
    DRY_RC=0
  else
    DRY_RC=$?
  fi
  if [[ $DRY_RC -eq 0 ]]; then
    success "lttng-tracer loadable (modprobe dry-run OK, version $(modinfo -F version lttng-tracer 2>/dev/null || echo '?'))"
    return
  fi

  warn "lttng-tracer built but modprobe rejects load:"
  echo "$DRY_OUT" | sed 's/^/    /' >&2

  # Secure Boot MOK signer/enrolled-CN mismatch diagnosis
  if command -v mokutil >/dev/null 2>&1 \
     && mokutil --sb-state 2>/dev/null | grep -q "SecureBoot enabled"; then
    local SIGNER ENROLLED_CNS
    SIGNER=$(modinfo -F signer lttng-tracer 2>/dev/null | head -1)
    ENROLLED_CNS=$(sudo mokutil --list-enrolled 2>/dev/null \
                   | awk -F'CN=' '/Subject:/ && NF>1 {print $NF}' \
                   | sort -u)
    if [[ -n "$SIGNER" ]] && ! echo "$ENROLLED_CNS" | grep -qxF "$SIGNER"; then
      warn "Cause: Secure Boot MOK signer is not enrolled."
      warn "  Module signed by: ${SIGNER}"
      warn "  Enrolled MOKs:"
      echo "$ENROLLED_CNS" | sed 's/^/      /' >&2
      warn "  Recovery:"
      warn "    sudo mokutil --import /var/lib/shim-signed/mok/MOK.der"
      warn "    sudo reboot   # On boot: MOK Manager → Enroll MOK → Continue → password"
      warn "    sudo modprobe lttng-tracer && lsmod | grep lttng   # verify after reboot"
    else
      warn "  Signer is enrolled but modprobe still fails — inspect 'sudo dmesg | tail' for the reject reason."
    fi
  fi
}
