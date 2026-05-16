#!/bin/bash
# install_uv.sh — uv 부트스트랩 (Astral uv installer)
#
# 제공 함수:
#   ensure_uv  — uv가 PATH에 없으면 curl installer로 ~/.local/bin/uv 설치
#
# Caller scope 의존:
#   apt_update_if_stale, 로거 (info/warn/success/error)
#
# 24.04 fresh desktop 가정: curl / ca-certificates 미설치일 수 있음 → apt로 보장.
# Astral installer는 ~/.local/bin에 single binary 설치 (sudo 불필요).

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

[[ -n "${_INSTALL_UV_LOADED:-}" ]] && return 0
_INSTALL_UV_LOADED=1

# ── uv 설치 (Astral curl installer) ─────────────────────────────────────────
# 정책:
#   - uv가 이미 PATH에 있으면 재사용 (멱등)
#   - 없으면 curl + ca-certificates 보장 → curl installer 실행
#   - ~/.local/bin이 현재 셸 PATH에 없으면 export로 추가
#   - 설치 실패 시 error exit (pip fallback 만들지 않음 — 분기 복잡도 회피)
ensure_uv() {
  # ~/.local/bin이 PATH에 없을 수 있음 (fresh shell)
  if [[ ":${PATH}:" != *":${HOME}/.local/bin:"* ]]; then
    export PATH="${HOME}/.local/bin:${PATH}"
  fi

  if command -v uv > /dev/null 2>&1; then
    info "uv already installed: $(command -v uv) ($(uv --version 2>&1))"
    return 0
  fi

  info "uv not found — installing via Astral curl installer..."

  # 24.04 fresh: curl + ca-certificates 보장
  apt_update_if_stale
  sudo apt-get install -y curl ca-certificates > /dev/null \
    || error "Failed to install curl / ca-certificates (required for uv bootstrap)"

  # Astral 공식 installer (single binary → ~/.local/bin/uv)
  # XDG_BIN_HOME 기본값 = ~/.local/bin
  if ! curl -LsSf https://astral.sh/uv/install.sh | sh; then
    error "uv install failed (curl https://astral.sh/uv/install.sh | sh)"
  fi

  # installer가 ~/.local/bin에 박았는데 PATH 재export 필요할 수 있음
  if [[ ":${PATH}:" != *":${HOME}/.local/bin:"* ]]; then
    export PATH="${HOME}/.local/bin:${PATH}"
  fi

  if ! command -v uv > /dev/null 2>&1; then
    error "uv install reported success but binary not found in PATH (expected ${HOME}/.local/bin/uv)"
  fi

  success "uv installed: $(command -v uv) ($(uv --version 2>&1))"
}
