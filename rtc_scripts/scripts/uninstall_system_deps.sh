#!/usr/bin/env bash
# uninstall_system_deps.sh — 대화형 시스템 정리 스크립트 (격리로 전환 시 1회 실행).
#
# 순서:
#   A) /usr/local/ stale 잔재 제거 (2024-07 hpp-fcl, pinocchio 3.0, gtsam, teaser)
#   B) /usr/local/ 의 fmt/mimalloc/aligator install_manifest uninstall + ~/libs/ 삭제
#   C) robotpkg 14개 제거 + ros-jazzy-proxsuite 전환 + /opt/openrobots 삭제
#   D) /opt/rti.com 제거
#
# 실행 전 필수:
#   - Step 8 (colcon build) + Step 9 (RPATH 검증) 완료
#   - Step 0 백업본 존재 (../rtc_ws_backup_*/)
#
# 각 단계마다 사용자 [y/N] 확인. sudo 필요.

set -eo pipefail

# 위치: src/rtc-framework/rtc_scripts/scripts/uninstall_system_deps.sh → 4 up = <rtc_ws>
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${_SCRIPT_DIR}/../../../.." && pwd)"
REPO="$(cd "${_SCRIPT_DIR}/../.." && pwd)"
TS="$(date +%Y%m%d_%H%M%S)"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'
info()    { echo -e "${BLUE}▶ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠ $*${NC}"; }
success() { echo -e "${GREEN}✔ $*${NC}"; }
error()   { echo -e "${RED}✘ $*${NC}" >&2; }

confirm() {
  local prompt="$1"
  read -r -p "$(echo -e "${YELLOW}${prompt} [y/N]: ${NC}")" ans
  [[ "$ans" =~ ^[Yy]$ ]]
}

banner() {
  echo ""
  echo -e "${BOLD}${BLUE}═══ $* ═══${NC}"
}

# ── Pre-flight ────────────────────────────────────────────────────────────────
banner "Pre-flight checks"
if [[ ! -f "${WS}/deps/install/lib/libaligator.so.0.19.0" ]]; then
  error "deps/install/ 가 비어있음. Step 6 (scripts/build_deps.sh) 먼저 실행하세요."
  exit 1
fi
if [[ ! -d "${WS}/install" ]] || [[ -z "$(ls -A "${WS}/install" 2>/dev/null)" ]]; then
  error "colcon install/ 가 비어있음. Step 8 (colcon build) 먼저 실행하세요."
  exit 1
fi
success "격리된 환경 확인: deps/install + install/ 정상"

# ── 단계 A: /usr/local stale (2024-07 잔재) 제거 ──────────────────────────────
banner "단계 A — /usr/local stale 2024-07 잔재 제거"
STALE_LIB="/tmp/rtc_isolation_stale_${TS}.txt"
sudo find /usr/local -maxdepth 6 \
  \( -name 'libhpp-fcl*' \
     -o -name 'libpinocchio*3.0.0*' \
     -o -name 'libpinocchio_casadi*' \
     -o -name 'libgtsam*' \
     -o -name 'libteaser*' \
     -o -name 'libCppUnitLite*' \
     -o -name 'libmetis-gtsam*' \
     -o -name 'libproxsuite-nlp*' \
     -o -name 'libmujoco.so*' \) \
  2>/dev/null | tee "$STALE_LIB" | wc -l | xargs -I{} echo "  stale files found: {}"
echo "   → list saved: $STALE_LIB"
if confirm "위 파일들을 삭제할까요?"; then
  # 백업 (선택적)
  sudo tar -czf "/tmp/rtc_stale_backup_${TS}.tar.gz" -T "$STALE_LIB" 2>/dev/null || true
  xargs -a "$STALE_LIB" sudo rm -f
  sudo rm -rf /usr/local/lib/cmake/{hpp-fcl,pinocchio,GTSAM,GTSAM_UNSTABLE,GTSAMCMakeTools,teaserpp,proxsuite-nlp,example-robot-data,jrl-cmakemodules,mujoco} 2>/dev/null || true
  sudo rm -rf /usr/local/lib/python3.12/dist-packages/{hppfcl,pinocchio,proxsuite_nlp,example_robot_data} 2>/dev/null || true
  sudo rm -rf /usr/local/share/example-robot-data 2>/dev/null || true
  sudo ldconfig
  success "단계 A 완료 (백업: /tmp/rtc_stale_backup_${TS}.tar.gz)"
else
  warn "단계 A 건너뜀"
fi

# ── 단계 B: /usr/local 의 fmt/mimalloc/aligator uninstall + ~/libs/ 삭제 ──────
banner "단계 B — /usr/local 의 fmt/mimalloc/aligator uninstall + ~/libs/ 삭제"
for pkg in aligator mimalloc fmt; do
  manifest="$HOME/libs/$pkg/build/install_manifest.txt"
  if [[ -f "$manifest" ]]; then
    n=$(wc -l < "$manifest")
    info "  $pkg manifest: $n files"
  else
    warn "  $pkg: no manifest (skip)"
  fi
done
if confirm "install_manifest 로 uninstall + ~/libs/ 삭제할까요?"; then
  for pkg in aligator mimalloc fmt; do
    manifest="$HOME/libs/$pkg/build/install_manifest.txt"
    if [[ -f "$manifest" ]]; then
      xargs -a "$manifest" sudo rm -f
      success "  $pkg uninstalled"
    fi
  done
  sudo ldconfig
  rm -rf "$HOME/libs/fmt" "$HOME/libs/mimalloc" "$HOME/libs/aligator"
  rmdir "$HOME/libs" 2>/dev/null || warn "~/libs/ 에 다른 항목이 남음: $(ls $HOME/libs 2>/dev/null)"
  success "단계 B 완료"
else
  warn "단계 B 건너뜀"
fi

# ── 단계 C: robotpkg 제거 + ros-jazzy-proxsuite 전환 ──────────────────────────
banner "단계 C — robotpkg 15개 제거 + ros-jazzy-proxsuite 전환"

# C-0: 사용 여부 재확인
info "C-0: workspace 의 find_package 참조 확인 (출력 비어있어야 안전)"
if grep -rE 'find_package\((casadi|blasfeo|fatrop|qpOASES|openscenegraph|gepetto|visit_struct|collada-dom|simde|jrl-cmakemodules|pythonqt|qgv|osgqt)' \
     "${WS}/src" 2>/dev/null; then
  warn "위 패키지가 workspace 에서 참조됨 — 단계 C 중단"
  exit 1
fi
success "C-0: workspace 가 robotpkg 패키지 미참조 확인"

# C-1: ros-jazzy-proxsuite 설치 + rtc_tsid 호환성 검증
if confirm "C-1: ros-jazzy-proxsuite 설치 + robotpkg-proxsuite 제거 + rtc_tsid 재빌드 테스트?"; then
  sudo apt-get install -y ros-jazzy-proxsuite
  sudo apt-get remove --purge -y robotpkg-proxsuite 2>/dev/null || true
  sudo apt-get autoremove --purge -y

  info "  rtc_tsid 재빌드 + 테스트"
  if (cd "$WS" && bash -c "source \"${_SCRIPT_DIR}/setup_env.sh\" && colcon build --packages-select rtc_tsid --cmake-clean-cache && colcon test --packages-select rtc_tsid --event-handlers console_direct+ && colcon test-result --verbose"); then
    success "C-1: rtc_tsid 테스트 통과 — 다음 단계 진행 가능"
  else
    error "C-1: rtc_tsid 테스트 실패 — robotpkg-proxsuite 복원"
    sudo apt-get install -y robotpkg-proxsuite
    warn "단계 C-2 는 robotpkg-proxsuite / robotpkg-jrl-cmakemodules 보존하도록 수동 조정 필요"
    exit 1
  fi
else
  warn "C-1 건너뜀 — 후속 단계 불가. 종료."
  exit 1
fi

# C-2: 나머지 14개 robotpkg 제거
if confirm "C-2: 나머지 robotpkg 14개 제거?"; then
  sudo apt-get remove --purge -y \
    robotpkg-blasfeo \
    robotpkg-casadi \
    robotpkg-collada-dom \
    robotpkg-fatrop \
    robotpkg-jrl-cmakemodules \
    robotpkg-openscenegraph \
    robotpkg-py312-casadi \
    robotpkg-py312-eigenpy \
    robotpkg-py312-pythonqt \
    robotpkg-py312-qt5-gepetto-viewer \
    robotpkg-qpoases+doc \
    robotpkg-qt5-osgqt \
    robotpkg-qt5-qgv \
    robotpkg-simde \
    robotpkg-visit-struct 2>/dev/null || true
  sudo apt-get autoremove --purge -y
  success "C-2: 완료"
else
  warn "C-2 건너뜀"
fi

# C-3: robotpkg apt source + /opt/openrobots 제거
if confirm "C-3: robotpkg apt source + /opt/openrobots 잔재 제거?"; then
  sudo rm -f /etc/apt/sources.list.d/robotpkg.list
  sudo rm -f /usr/share/keyrings/robotpkg-archive-keyring.gpg
  sudo apt-get update
  [[ -d /opt/openrobots ]] && sudo rm -rf /opt/openrobots
  # ~/.bashrc 의 주석 처리된 /opt/openrobots export 완전 삭제
  sed -i '/^# \[ISOLATED .*\]/,/^# export CMAKE_PREFIX_PATH=.*\/opt\/openrobots/d' ~/.bashrc 2>/dev/null || true
  success "C-3: 완료"
else
  warn "C-3 건너뜀"
fi

# ── 단계 D: /opt/rti.com 제거 ─────────────────────────────────────────────────
banner "단계 D — /opt/rti.com 제거"
if [[ -d /opt/rti.com ]]; then
  if confirm "/opt/rti.com (RTI Connext DDS) 제거?"; then
    sudo rm -rf /opt/rti.com
    sed -i '/RTI_LICENSE_FILE/d' ~/.bashrc 2>/dev/null || true
    sed -i '/alias ros2rti=/d' ~/.bashrc 2>/dev/null || true
    success "단계 D 완료"
  else
    warn "단계 D 건너뜀"
  fi
else
  info "/opt/rti.com 이미 없음 — 건너뜀"
fi

# ── 최종 검증 ─────────────────────────────────────────────────────────────────
banner "최종 검증"
echo "robotpkg 잔여: $(dpkg -l | grep -c '^ii  robotpkg-')"
echo "/opt/openrobots: $(ls /opt/openrobots 2>&1 | head -1)"
echo "/opt/rti.com:    $(ls /opt/rti.com    2>&1 | head -1)"
echo "/usr/local/lib/libhpp-fcl.so: $(ls /usr/local/lib/libhpp-fcl.so 2>&1 | head -1)"
echo "~/libs: $(ls ~/libs 2>&1 | head -1)"
success "Cleanup 완료"
