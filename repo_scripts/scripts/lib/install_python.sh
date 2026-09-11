#!/bin/bash
# install_python.sh — Python venv + system Python deps for install.sh
#
# 제공 함수:
#   ensure_venv               — .venv 자동 생성 (uv venv, 24.04 PEP 668 우회)
#   install_python_base_deps  — python3-dev (eigenpy Python.h 빌드 헤더)
#   install_python_deps       — requirements.lock으로 venv 일괄 동기화 (uv pip sync)
#
# Caller scope 의존:
#   WORKSPACE, INSTALL_SCRIPT_DIR, apt_update_if_stale, 로거,
#   is_venv_active · venv_uses_system_python · get_system_python (rt_common.sh),
#   ensure_uv (install_uv.sh)
#
# 정책 (2026-05-23, scientific stack venv 이전):
#   - venv 생성·sync 모두 uv (pip 미사용)
#   - lock은 requirements.lock (uv pip compile --generate-hashes 산출)
#   - numpy/scipy/matplotlib/pandas/PyQt5는 venv 안에 핀-버전으로 설치 →
#     다른 workspace 의 시스템 패키지 변경에 영향 안 받음 (cross-workspace 격리).
#     numpy<2 핀은 ros-jazzy-rclpy ABI 호환용 (requirements.in 주석 참조).
#   - rclpy / ament_* / python3-bt2 등은 --system-site-packages 로 상속 유지.

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
#   --python $(get_system_python) (rt_common.sh): 버전이 아니라 **경로**로 고정한다
#     (24.04 에선 /usr/bin/python3.12 — requirements.lock 의 대상).
#     `--python 3.12` 는 runtime PC 의 python3.9/3.10 은 피하지만, uv 기본값
#     (python-preference=managed) 때문에 uv-managed 3.12 가 설치돼 있으면 그걸 base
#     로 고른다. 그 venv 는 --system-site-packages 여도 /usr/lib/python3/dist-packages
#     (apt catkin_pkg · python3-yaml …) 를 못 봐서 fresh PC 에서 rtc_base configure 가
#     catkin_pkg ImportError 로 죽었다. pyproject.toml ruff target-version = "py312".
#   workspace .venv 는 base 가 틀리면 재생성, 맞으면 재사용 (멱등) — setup_env.sh 가
#     이미 활성화했어도 검사한다 (전엔 "already active" 로 빠져 검사에 도달 못 했다)
#   이미 다른 venv 가 활성이면 그것을 그대로 사용 (build.sh 가 base 를 경고한다).
#     단 가리키는 디렉토리가 없는 VIRTUAL_ENV (활성화한 채 지운 venv) 는 활성으로 치지
#     않는다. 옮긴 workspace 의 .venv 는 uv activate 가 옛 경로를 박아 두어 여기서
#     고쳐지지 않는다 — .venv 를 지우고 다시 실행한다.
ensure_venv() {
  local venv_dir="${WORKSPACE:?}/.venv" sys_py
  sys_py=$(get_system_python)

  if is_venv_active && [[ -d "${VIRTUAL_ENV}" ]] && ! [[ "${VIRTUAL_ENV}" -ef "${venv_dir}" ]]; then
    info "venv already active: ${VIRTUAL_ENV} — reusing"
    return 0
  fi

  ensure_uv

  local recreate=0
  if [[ -e "${venv_dir}" ]] && ! venv_uses_system_python "${venv_dir}"; then
    warn "Existing venv at ${venv_dir} is not based on ${sys_py} with system-site-packages — recreating"
    recreate=1
  fi

  if [[ "${recreate}" -eq 1 || ! -f "${venv_dir}/bin/activate" ]]; then
    info "Creating Python venv at ${venv_dir} (uv, ${sys_py}, --system-site-packages)..."
    # python3.12-venv는 uv venv가 stdlib venv module을 호출하므로 필요.
    # 메타패키지 python3-venv 는 시스템 default python3 (runtime PC 에선 3.9 가능)
    # 를 따라가므로 명시적으로 3.12 변종을 깐다.
    apt_update_if_stale
    sudo apt-get install -y python3.12 python3.12-venv > /dev/null
    # apt 가 성공한 뒤에 지운다 — dpkg lock · sudo 거부로 멈추면 기존 venv 가 남는다.
    # 활성 상태여도 지워도 된다: 같은 경로에 다시 만든다.
    rm -rf "${venv_dir}"
    uv venv --python "${sys_py}" --system-site-packages "${venv_dir}" \
      || error "uv venv failed at ${venv_dir} (${sys_py} not found?)"
    success "venv created: ${venv_dir}"
  else
    info "venv already exists at ${venv_dir} (${sys_py}) — activating"
  fi

  # 이미 이 venv 가 활성이면 activate 를 다시 source 하지 않는다. uv 의 activate 는
  # 첫 줄의 `deactivate nondestructive` 가 PATH 를 처음 활성화하던 시점으로 되돌려
  # 그 뒤에 붙은 항목 — 위 ensure_uv 의 ~/.local/bin — 을 지우고, 그러면 뒤이은
  # `uv pip sync` 가 command not found 로 죽는다. 같은 경로에 다시 만들었으므로
  # PATH 의 ${venv_dir}/bin 은 그대로 유효하다.
  if ! [[ "${VIRTUAL_ENV:-}" -ef "${venv_dir}" ]]; then
    # shellcheck disable=SC1091
    source "${venv_dir}/bin/activate"
  fi
  success "venv activated: ${VIRTUAL_ENV}"
}

# ── Python dev headers (required by eigenpy cmake detection) ──────────────────
# eigenpy's python.cmake calls FIND_NUMPY at cmake configure time, and needs
# Python.h to compile its C extension. python3-dev provides Python.h.
# NumPy headers come from the apt python3-numpy, not the venv: build.sh pins
# Python3_EXECUTABLE to get_system_python (rt_common.sh), so eigenpy's
# numpy.get_include() runs outside the venv and never sees requirements.lock.
install_python_base_deps() {
  info "Installing Python dev headers (Python.h for eigenpy cmake)..."
  sudo apt-get install -y \
      python3-dev \
      > /dev/null
  success "python3-dev installed"
}

# ── Python dependencies (venv lock sync only) ──────────────────────────────────
# Full scientific stack + mujoco + Cython + ruff + setuptools/wheel ships via
# requirements.lock. Cross-workspace isolation: another control project on the
# same host can pin a different numpy without affecting this venv.
install_python_deps() {
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
