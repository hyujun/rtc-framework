#!/bin/bash
# bootstrap.sh — 공통 entry-script 부트스트랩
#
# build.sh / install.sh 의 공통 초기화 시퀀스를 캡슐화한다:
#   1) caller 의 script directory 를 _RT_SCRIPT_DIR 로 export
#   2) lib/rt_common.sh 를 source (없으면 install.sh 와 동등한 fallback logger 정의)
#   3) make_logger 로 caller-specific logger 활성화
#   4) setup_env.sh 가 아직 source 되지 않았으면 자동 source
#
# 사용:
#   source "$(dirname "${BASH_SOURCE[0]}")/repo_scripts/scripts/lib/bootstrap.sh" INSTALL
#   # 이후 $_RT_SCRIPT_DIR (절대경로) 사용
#
# 인자:
#   $1 — logger tag (예: "BUILD", "INSTALL"). 누락 시 "RTC".

# 재진입 가드 (caller 가 두 번 source 해도 안전)
if [[ -n "${_RT_BOOTSTRAP_LOADED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi
_RT_BOOTSTRAP_LOADED=1

# caller script 의 절대 경로 — BASH_SOURCE[1] 은 bootstrap.sh 를 source 한 파일
_RT_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)"

_RT_COMMON="${_RT_SCRIPT_DIR}/repo_scripts/scripts/lib/rt_common.sh"
_RT_LOGGER_TAG="${1:-RTC}"

if [[ -f "$_RT_COMMON" ]]; then
  # shellcheck source=repo_scripts/scripts/lib/rt_common.sh
  source "$_RT_COMMON"
  make_logger "$_RT_LOGGER_TAG" emoji
else
  # fallback: rt_common.sh 없을 때 최소 색상/로거 정의 (install.sh 기존 동작 보존)
  RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
  BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
  info()    { echo -e "${BLUE}▶ $*${NC}"; }
  warn()    { echo -e "${YELLOW}⚠ $*${NC}"; }
  error()   { echo -e "${RED}✘ $*${NC}" >&2; exit 1; }
  success() { echo -e "${GREEN}✔ $*${NC}"; }
fi

# 격리 환경 자동 활성화 — RTC_DEPS_PREFIX 가 비었으면 setup_env.sh 를 source
_SETUP_ENV="${_RT_SCRIPT_DIR}/repo_scripts/scripts/setup_env.sh"
if [[ -z "${RTC_DEPS_PREFIX:-}" && -f "$_SETUP_ENV" ]]; then
  # shellcheck source=/dev/null
  source "$_SETUP_ENV"
fi

unset _RT_COMMON _RT_LOGGER_TAG _SETUP_ENV
