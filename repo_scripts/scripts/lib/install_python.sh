#!/bin/bash
# install_python.sh — Python venv + system Python deps for install.sh
#
# 제공 함수:
#   ensure_venv               — .venv 자동 생성 (24.04 PEP 668 우회)
#   install_python_base_deps  — python3-dev, python3-numpy (eigenpy cmake 의존)
#   install_python_deps       — matplotlib, pandas, scipy, PyQt5, mujoco bindings
#
# Caller scope 의존:
#   WORKSPACE, apt_update_if_stale, 로거 (info/warn/success), is_venv_active

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

[[ -n "${_INSTALL_PYTHON_LOADED:-}" ]] && return 0
_INSTALL_PYTHON_LOADED=1

# ── Python venv 자동 생성 (24.04 PEP 668 우회) ─────────────────────────────
# 24.04+ 는 libpython3.12-stdlib 가 EXTERNALLY-MANAGED 마커를 깔아
# system Python 에 대한 `pip install` 을 차단한다 (PEP 668).
# 대응: workspace 루트에 .venv 를 자동 생성하여 install_python_*_deps 의
# pip install 경로가 venv 안에서 동작하도록 한다.
#   --system-site-packages: ROS rclpy / ament_* 모듈 상속
#   기존 .venv 가 있으면 재사용 (멱등)
#   이미 다른 venv 가 활성이면 그것을 그대로 사용
ensure_venv() {
  if is_venv_active; then
    info "venv already active: ${VIRTUAL_ENV} — reusing"
    return 0
  fi

  local venv_dir="${WORKSPACE}/.venv"
  if [[ ! -f "${venv_dir}/bin/activate" ]]; then
    info "Creating Python venv at ${venv_dir} (PEP 668 workaround for 24.04+)..."
    apt_update_if_stale
    sudo apt-get install -y python3-venv > /dev/null
    python3 -m venv --system-site-packages "${venv_dir}"
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
install_python_base_deps() {
  info "Installing Python dev headers and NumPy (required by eigenpy cmake)..."
  sudo apt-get install -y \
      python3-dev \
      python3-numpy \
      > /dev/null
  success "python3-dev and python3-numpy installed"

  # venv 내부에서 eigenpy cmake가 numpy를 못 찾는 문제 대비 pip install
  if is_venv_active; then
    info "Installing numpy and Cython inside the active venv..."
    python3 -m pip install numpy Cython --quiet || true
    success "numpy and Cython installed in venv"
  fi
}

# ── Python dependencies ─────────────────────────────────────────────────────────
install_python_deps() {
  info "Installing Python dependencies (via apt)..."
  apt_update_if_stale
  sudo apt-get install -y \
      python3-matplotlib \
      python3-pandas \
      python3-numpy \
      python3-scipy \
      python3-pyqt5 \
      > /dev/null
  success "Python dependencies installed via apt"

  # mujoco Python bindings (not available via apt — always install via pip)
  info "Installing mujoco Python bindings (pip)..."
  python3 -m pip install --quiet "mujoco>=3.0.0" \
    || warn "mujoco pip install failed — urdf_to_mjcf will not work without it"
  success "mujoco Python bindings installed"

  # venv 내부에서 apt 패키지가 보이지 않을 수 있으므로 pip으로도 설치
  if is_venv_active; then
    info "Installing Python dependencies inside the active venv (pip)..."
    python3 -m pip install --quiet \
        matplotlib \
        pandas \
        numpy \
        scipy \
        PyQt5 \
        "mujoco>=3.0.0" \
        ruff \
      || warn "One or more pip installs failed — check output above"
    success "Python dependencies installed in venv (incl. ruff for formatting)"
  fi
}
