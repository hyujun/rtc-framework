#!/bin/bash
# install_python.sh — Python venv + system Python deps for install.sh
#
# 제공 함수:
#   ensure_venv               — .venv 자동 생성 (uv venv, 24.04 PEP 668 우회)
#   install_python_base_deps  — python3-dev, python3-numpy (eigenpy cmake 의존)
#   install_python_deps       — requirements.lock으로 venv 일괄 동기화 (uv pip sync)
#
# Caller scope 의존:
#   WORKSPACE, INSTALL_SCRIPT_DIR, apt_update_if_stale, 로거,
#   is_venv_active, ensure_uv (install_uv.sh)
#
# 정책 (2026-05-16, uv 도입):
#   - venv 생성·sync 모두 uv (pip 미사용)
#   - lock은 requirements.lock (uv pip compile --generate-hashes 산출)
#   - apt-provided 패키지 (numpy/scipy/matplotlib/pandas/PyQt5)는 lock에서 제외 →
#     system-site-packages로 상속. requirements.in 정책 주석 참조.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

[[ -n "${_INSTALL_PYTHON_LOADED:-}" ]] && return 0
_INSTALL_PYTHON_LOADED=1

# ── Python venv 자동 생성 (uv, 24.04 PEP 668 우회) ─────────────────────────
# 24.04+ 는 libpython3.12-stdlib 가 EXTERNALLY-MANAGED 마커를 깔아
# system Python 에 대한 `pip install` 을 차단한다 (PEP 668).
# 대응: workspace 루트에 .venv 를 uv로 생성하여 후속 install이 venv 안에서 동작.
#   --system-site-packages: ROS rclpy / ament_* / apt numpy/scipy/matplotlib/pandas/PyQt5 상속
#   기존 .venv 가 있으면 재사용 (멱등)
#   이미 다른 venv 가 활성이면 그것을 그대로 사용
ensure_venv() {
  if is_venv_active; then
    info "venv already active: ${VIRTUAL_ENV} — reusing"
    return 0
  fi

  ensure_uv

  local venv_dir="${WORKSPACE}/.venv"
  if [[ ! -f "${venv_dir}/bin/activate" ]]; then
    info "Creating Python venv at ${venv_dir} (uv, --system-site-packages)..."
    # python3-venv는 uv venv가 stdlib venv module을 호출하므로 필요
    apt_update_if_stale
    sudo apt-get install -y python3-venv > /dev/null
    uv venv --system-site-packages "${venv_dir}" \
      || error "uv venv failed at ${venv_dir}"
    success "venv created: ${venv_dir}"
  else
    info "venv already exists at ${venv_dir} — activating"
  fi

  # shellcheck disable=SC1091
  source "${venv_dir}/bin/activate"
  success "venv activated: ${VIRTUAL_ENV}"
}

# ── Python dev headers + NumPy (required by eigenpy cmake detection) ───────────
# eigenpy's python.cmake calls FIND_NUMPY at cmake configure time.
# python3-dev provides Python.h; python3-numpy provides numpy headers.
# Must be installed before colcon build (and ideally before pinocchio apt install).
#
# uv 도입 후 venv 내부 numpy는 system-site-packages로 상속받는다 (requirements.in
# 정책상 lock에서 제외). 따라서 별도 venv-pip 호출 불필요.
install_python_base_deps() {
  info "Installing Python dev headers and NumPy (required by eigenpy cmake)..."
  sudo apt-get install -y \
      python3-dev \
      python3-numpy \
      > /dev/null
  success "python3-dev and python3-numpy installed"
}

# ── Python dependencies (apt + venv lock sync) ──────────────────────────────────
# apt:  numpy/scipy/matplotlib/pandas/PyQt5 — venv가 system-site-packages로 상속
# venv: mujoco + transitive + Cython + ruff + setuptools/wheel — requirements.lock
install_python_deps() {
  info "Installing apt Python packages (inherited by venv via system-site-packages)..."
  apt_update_if_stale
  sudo apt-get install -y \
      python3-matplotlib \
      python3-pandas \
      python3-numpy \
      python3-scipy \
      python3-pyqt5 \
      > /dev/null
  success "apt Python packages installed"

  if ! is_venv_active; then
    warn "No active venv — skipping uv pip sync (requirements.lock not applied)"
    return 0
  fi

  local lock="${INSTALL_SCRIPT_DIR}/requirements.lock"
  if [[ ! -f "${lock}" ]]; then
    error "requirements.lock not found at ${lock}"
  fi

  info "Syncing venv with ${lock} (uv pip sync, hash-verified)..."
  # uv pip sync = lock에 명시된 정확한 set 으로 venv 일치시킴
  #   - lock에 없는 패키지는 venv에서 제거 (단 system-site-packages는 건드리지 않음)
  #   - hash 검증으로 wheel 변조 차단
  uv pip sync "${lock}" \
    || error "uv pip sync failed (lock: ${lock})"
  success "venv synced with requirements.lock"
}
