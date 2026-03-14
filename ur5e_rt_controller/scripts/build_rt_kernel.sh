#!/bin/bash
# build_rt_kernel.sh — PREEMPT_RT 커널 빌드 및 설치
#
# Ubuntu 22.04 / 24.04 에서 PREEMPT_RT 패치된 커널을 빌드한다.
# Option B (최대 성능) — docs/RT_OPTIMIZATION.md 참조.
#
# 이전 실행에서 완료된 단계는 자동으로 감지하여 건너뛴다.
# 예: 빌드가 완료된 상태라면 설치 단계부터 재개한다.
#
# Usage:
#   sudo ./build_rt_kernel.sh                # 대화형 (완료 단계 자동 스킵)
#   sudo ./build_rt_kernel.sh --batch        # 비대화형 (menuconfig 건너뜀)
#   sudo ./build_rt_kernel.sh --dry-run      # 다운로드 및 패치까지만 실행
#   sudo ./build_rt_kernel.sh --status       # 진행 상태만 확인 (실행 안 함)
#   sudo ./build_rt_kernel.sh --force-step 5 # 5단계부터 강제 재실행
#   sudo ./build_rt_kernel.sh --clean        # 빌드 소스 정리 후 처음부터
#   sudo ./build_rt_kernel.sh --help
#
# 실행 시점: 최초 1회 (커널 빌드 → 설치 → 재부팅)
# 소요 시간: 30분 ~ 2시간 (CPU 성능에 따라 다름)
#
# 검증:
#   uname -v | grep PREEMPT_RT   # 재부팅 후

set -euo pipefail

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
DIM='\033[2m'
BOLD='\033[1m'
NC='\033[0m'

info()    { echo -e "${BLUE}[RT-KERNEL]${NC} $*"; }
success() { echo -e "${GREEN}[RT-KERNEL]${NC} $*"; }
warn()    { echo -e "${YELLOW}[RT-KERNEL]${NC} $*"; }
error()   { echo -e "${RED}[RT-KERNEL]${NC} $*" >&2; exit 1; }

# ── Helper: physical CPU core count (SMT/HT 제외) ────────────────────────────
# nproc은 논리 코어(HT 포함)를 반환하므로, 물리 코어만 카운트한다.
# 예: i7-8700 (6C/12T) → nproc=12 이지만 이 함수는 6을 반환.
get_physical_cores() {
  if command -v lscpu &>/dev/null; then
    local count
    count=$(lscpu -p=Core,Socket 2>/dev/null | grep -v '^#' | sort -u | wc -l)
    if [[ "$count" -gt 0 ]]; then
      echo "$count"
      return
    fi
  fi
  if [[ -f /sys/devices/system/cpu/cpu0/topology/core_id ]]; then
    local seen=""
    local count=0
    for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*/; do
      local pkg_file="${cpu_dir}topology/physical_package_id"
      local core_file="${cpu_dir}topology/core_id"
      [[ -f "$pkg_file" && -f "$core_file" ]] || continue
      local key
      key="$(cat "$pkg_file"):$(cat "$core_file")"
      if [[ ! " $seen " == *" $key "* ]]; then
        seen="$seen $key"
        ((count++))
      fi
    done
    if [[ "$count" -gt 0 ]]; then
      echo "$count"
      return
    fi
  fi
  nproc
}

# ── Argument parsing ─────────────────────────────────────────────────────────
BATCH_MODE=0
DRY_RUN=0
STATUS_ONLY=0
FORCE_STEP=0
CLEAN_BUILD=0
BUILD_DIR="${HOME}/rt_kernel_build"

show_help() {
  echo ""
  echo -e "${BOLD}build_rt_kernel.sh — PREEMPT_RT 커널 빌드${NC}"
  echo ""
  echo "Usage: sudo $0 [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  --batch          비대화형 모드 (menuconfig 건너뜀)"
  echo "  --dry-run        다운로드 및 패치까지만 실행 (빌드 안 함)"
  echo "  --status         진행 상태만 확인 (실행하지 않음)"
  echo "  --force-step N   N단계부터 강제 재실행 (1-6)"
  echo "  --clean          빌드 소스 정리 후 처음부터 (다운로드 파일은 보존)"
  echo "  --build-dir DIR  빌드 디렉토리 지정 (기본: ~/rt_kernel_build)"
  echo "  --help           이 도움말 표시"
  echo ""
  echo "단계 구성:"
  echo "  [1/6] 필수 빌드 패키지 설치"
  echo "  [2/6] 커널 소스 + RT 패치 다운로드"
  echo "  [3/6] 압축 해제 및 패치 적용"
  echo "  [4/6] 커널 설정 (PREEMPT_RT 활성화)"
  echo "  [5/6] 커널 빌드 (make bindeb-pkg)"
  echo "  [6/6] .deb 패키지 설치 + GRUB 설정"
  echo ""
  echo "이전 실행에서 완료된 단계는 자동으로 건너뜁니다."
  echo ""
  echo "Examples:"
  echo "  sudo ./build_rt_kernel.sh --batch          # 전체 자동 (완료 단계 스킵)"
  echo "  sudo ./build_rt_kernel.sh --status         # 상태만 확인"
  echo "  sudo ./build_rt_kernel.sh --force-step 5   # 빌드부터 강제 재실행"
  echo "  sudo ./build_rt_kernel.sh --clean --batch  # 정리 후 처음부터"
  echo ""
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --batch)     BATCH_MODE=1; shift ;;
    --dry-run)   DRY_RUN=1; shift ;;
    --status)    STATUS_ONLY=1; shift ;;
    --force-step)
      [[ $# -ge 2 ]] || error "--force-step requires a step number (1-6)"
      FORCE_STEP="$2"
      if [[ "$FORCE_STEP" -lt 1 || "$FORCE_STEP" -gt 6 ]]; then
        error "--force-step must be 1-6 (given: ${FORCE_STEP})"
      fi
      shift 2 ;;
    --clean)     CLEAN_BUILD=1; shift ;;
    --build-dir)
      [[ $# -ge 2 ]] || error "--build-dir requires a value"
      BUILD_DIR="$2"; shift 2 ;;
    -h|--help)   show_help ;;
    *)           error "Unknown option: '$1'  (run $0 --help)" ;;
  esac
done

# ── Root check ───────────────────────────────────────────────────────────────
if [[ "$EUID" -ne 0 ]]; then
  error "Root privileges required. Run: sudo $0 $*"
fi

# ── Banner ───────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║     PREEMPT_RT Kernel Build Script                   ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════════════╝${NC}"
echo ""

# ── OS 버전 감지 및 커널/패치 버전 설정 ────────────────────────────────────────
if ! command -v lsb_release &>/dev/null; then
  error "lsb_release not found. Install with: sudo apt-get install -y lsb-release"
fi
UBUNTU_VER=$(lsb_release -rs 2>/dev/null)

if [[ "$UBUNTU_VER" == "24.04" ]]; then
  KERNEL_MAJOR=6
  KERNEL_VERSION="6.8.2"
  PATCH_VERSION="6.8.2-rt11"
  RT_PATCH_DIR="6.8"
elif [[ "$UBUNTU_VER" == "22.04" ]]; then
  KERNEL_MAJOR=6
  KERNEL_VERSION="6.6.127"
  PATCH_VERSION="6.6.127-rt69"
  RT_PATCH_DIR="6.6"
else
  error "지원하지 않는 Ubuntu 버전입니다: ${UBUNTU_VER} (22.04 또는 24.04 필요)"
fi

KERNEL_URL="https://mirrors.edge.kernel.org/pub/linux/kernel/v${KERNEL_MAJOR}.x/linux-${KERNEL_VERSION}.tar.xz"
PATCH_URL="https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/${RT_PATCH_DIR}/patch-${PATCH_VERSION}.patch.xz"

# RT 커널 버전 문자열 (설치 확인용)
RT_TAG=$(echo "$PATCH_VERSION" | grep -oP 'rt\d+')
RT_KERNEL_FULL="${KERNEL_VERSION}-${RT_TAG}-rt-custom"

# NVIDIA GPU 감지
HAS_NVIDIA="no"
if lspci 2>/dev/null | grep -qi nvidia; then
  HAS_NVIDIA="yes"
fi

# CPU 정보
LOGICAL_CORES=$(nproc)
PHYSICAL_CORES=$(get_physical_cores)
BUILD_THREADS="${LOGICAL_CORES}"

KERNEL_SRC_DIR="${BUILD_DIR}/linux-${KERNEL_VERSION}"

# ══════════════════════════════════════════════════════════════════════════════
# Step completion detection functions
# ══════════════════════════════════════════════════════════════════════════════

# [1/6] 패키지 설치 완료 여부
is_step1_done() {
  local required_pkgs=("build-essential" "libncurses-dev" "libssl-dev" "libelf-dev" "flex" "bison")
  for pkg in "${required_pkgs[@]}"; do
    if ! dpkg -s "$pkg" &>/dev/null; then
      return 1
    fi
  done
  return 0
}

# [2/6] 다운로드 완료 여부
is_step2_done() {
  [[ -f "${BUILD_DIR}/linux-${KERNEL_VERSION}.tar.xz" ]] && \
  [[ -f "${BUILD_DIR}/patch-${PATCH_VERSION}.patch.xz" ]]
}

# [3/6] 압축 해제 + 패치 완료 여부
is_step3_done() {
  [[ -f "${KERNEL_SRC_DIR}/.rt_patched" ]]
}

# [4/6] 커널 설정 완료 여부
is_step4_done() {
  [[ -f "${KERNEL_SRC_DIR}/.config" ]] && \
  grep -q "CONFIG_PREEMPT_RT=y" "${KERNEL_SRC_DIR}/.config" 2>/dev/null
}

# [5/6] 빌드 완료 여부
is_step5_done() {
  # linux-image .deb 존재 (dbg 제외)
  local found
  found=$(find "${BUILD_DIR}" -maxdepth 1 -name "linux-image-${KERNEL_VERSION}*.deb" \
    ! -name "*dbg*" 2>/dev/null | head -1)
  [[ -n "$found" ]]
}

# [6/6] 설치 완료 여부
is_step6_done() {
  # dpkg -l에서 RT 커널 이미지가 ii (installed) 상태인지 확인
  dpkg -l "linux-image-${RT_KERNEL_FULL}" 2>/dev/null | grep -q "^ii" 2>/dev/null
}

# ── 진행 상태 표시 ────────────────────────────────────────────────────────────
show_status() {
  local labels=(
    "[1/6] 필수 빌드 패키지 설치"
    "[2/6] 커널 소스 + RT 패치 다운로드"
    "[3/6] 압축 해제 및 패치 적용"
    "[4/6] 커널 설정 (PREEMPT_RT)"
    "[5/6] 커널 빌드 (make bindeb-pkg)"
    "[6/6] .deb 패키지 설치 + GRUB"
  )
  local check_fns=("is_step1_done" "is_step2_done" "is_step3_done"
                    "is_step4_done" "is_step5_done" "is_step6_done")
  local first_todo=0

  echo -e "${BOLD}━━━ 진행 상태 ━━━${NC}"
  echo ""
  info "Ubuntu: ${UBUNTU_VER}  |  커널: ${KERNEL_VERSION}  |  RT: ${PATCH_VERSION}"
  info "빌드 디렉토리: ${BUILD_DIR}"
  info "CPU: 물리 ${PHYSICAL_CORES}코어 / 논리 ${LOGICAL_CORES}코어  |  NVIDIA: ${HAS_NVIDIA}"
  echo ""

  for i in "${!labels[@]}"; do
    local step=$((i + 1))
    local forced=0

    # --force-step 적용: 해당 단계 이후는 미완료로 표시
    if [[ "$FORCE_STEP" -gt 0 && "$step" -ge "$FORCE_STEP" ]]; then
      forced=1
    fi

    if [[ "$forced" -eq 0 ]] && ${check_fns[$i]}; then
      echo -e "  ${GREEN}[DONE]${NC} ${labels[$i]}"
    else
      if [[ "$first_todo" -eq 0 ]]; then
        echo -e "  ${YELLOW}[>>>>]${NC} ${labels[$i]}  ${DIM}← 여기서 재개${NC}"
        first_todo=$step
      else
        echo -e "  ${DIM}[TODO]${NC} ${labels[$i]}"
      fi
    fi
  done

  echo ""

  if [[ "$first_todo" -eq 0 ]]; then
    echo -e "  ${GREEN}${BOLD}모든 단계 완료!${NC} 재부팅하여 RT 커널을 사용하세요: sudo reboot"
  else
    echo -e "  ${BOLD}${first_todo}단계부터 실행합니다${NC}"
  fi
  echo ""
}

# ── --clean 처리 ──────────────────────────────────────────────────────────────
if [[ "$CLEAN_BUILD" -eq 1 ]]; then
  info "빌드 소스 정리 중... (다운로드 파일은 보존)"
  if [[ -d "${KERNEL_SRC_DIR}" ]]; then
    rm -rf "${KERNEL_SRC_DIR}"
    success "소스 디렉토리 삭제: ${KERNEL_SRC_DIR}"
  fi
  # .deb 파일도 정리 (재빌드 유도)
  find "${BUILD_DIR}" -maxdepth 1 -name "linux-*.deb" -delete 2>/dev/null || true
  find "${BUILD_DIR}" -maxdepth 1 -name "linux-*.buildinfo" -delete 2>/dev/null || true
  find "${BUILD_DIR}" -maxdepth 1 -name "linux-*.changes" -delete 2>/dev/null || true
  success "빌드 산출물 정리 완료 (tar.xz, patch.xz는 보존됨)"
  echo ""
fi

# ── --status 처리 ─────────────────────────────────────────────────────────────
show_status

if [[ "$STATUS_ONLY" -eq 1 ]]; then
  exit 0
fi

# ── 단계 스킵 여부 판단 함수 ──────────────────────────────────────────────────
should_skip() {
  local step="$1"
  local check_fn="$2"

  # --force-step이 지정되었고 현재 단계가 그 이후면 강제 실행
  if [[ "$FORCE_STEP" -gt 0 && "$step" -ge "$FORCE_STEP" ]]; then
    return 1  # 스킵하지 않음
  fi

  # 완료 확인 함수 호출
  if $check_fn; then
    return 0  # 스킵
  fi
  return 1    # 스킵하지 않음
}

# ══════════════════════════════════════════════════════════════════════════════
# [1/6] 필수 빌드 패키지 설치
# ══════════════════════════════════════════════════════════════════════════════
echo ""
info "━━━ [1/6] 필수 빌드 패키지 설치 ━━━"

if should_skip 1 is_step1_done; then
  success "필수 패키지가 이미 설치되어 있습니다 — 건너뜀"
else
  # 이전 실행에서 dpkg가 broken 상태일 수 있으므로 먼저 복구
  if dpkg --audit 2>&1 | grep -q .; then
    warn "dpkg broken 상태 감지 — 복구 중..."
    dpkg --configure -a 2>/dev/null || true
    apt-get install -f -y > /dev/null 2>&1 || true
  fi

  apt-get update -qq
  apt-get install -y \
      build-essential bc curl wget \
      libncurses-dev libssl-dev libelf-dev \
      flex bison debhelper \
      python3 dkms cpio \
      > /dev/null

  if [[ "$HAS_NVIDIA" == "yes" ]]; then
    info "NVIDIA DKMS 드라이버 설치 중..."
    if ! apt-get install -y nvidia-dkms-550 > /dev/null 2>&1; then
      warn "nvidia-dkms-550 실패, nvidia-dkms-535 시도..."
      if ! apt-get install -y nvidia-dkms-535 > /dev/null 2>&1; then
        warn "NVIDIA DKMS 자동 설치 실패 — 수동 설치가 필요할 수 있습니다"
      fi
    fi
  fi
  success "필수 패키지 설치 완료"
fi

# ══════════════════════════════════════════════════════════════════════════════
# [2/6] 커널 소스 + RT 패치 다운로드
# ══════════════════════════════════════════════════════════════════════════════
echo ""
info "━━━ [2/6] 커널 소스 다운로드 ━━━"

if should_skip 2 is_step2_done; then
  success "커널 소스 + RT 패치가 이미 다운로드되어 있습니다 — 건너뜀"
else
  mkdir -p "${BUILD_DIR}"
  cd "${BUILD_DIR}"

  if [[ ! -f "linux-${KERNEL_VERSION}.tar.xz" ]]; then
    info "커널 소스 다운로드: ${KERNEL_URL}"
    if ! curl -fLO "${KERNEL_URL}"; then
      error "커널 소스 다운로드 실패: ${KERNEL_URL}"
    fi
  else
    info "커널 소스 이미 존재: linux-${KERNEL_VERSION}.tar.xz"
  fi

  if [[ ! -f "patch-${PATCH_VERSION}.patch.xz" ]]; then
    info "RT 패치 다운로드: ${PATCH_URL}"
    if ! curl -fLO "${PATCH_URL}"; then
      error "RT 패치 다운로드 실패: ${PATCH_URL}"
    fi
  else
    info "RT 패치 이미 존재: patch-${PATCH_VERSION}.patch.xz"
  fi
  success "다운로드 완료"
fi

# ══════════════════════════════════════════════════════════════════════════════
# [3/6] 압축 해제 및 패치 적용
# ══════════════════════════════════════════════════════════════════════════════
echo ""
info "━━━ [3/6] 압축 해제 및 패치 적용 ━━━"

if should_skip 3 is_step3_done; then
  success "RT 패치가 이미 적용되어 있습니다 — 건너뜀"
else
  cd "${BUILD_DIR}"

  if [[ -d "${KERNEL_SRC_DIR}" ]]; then
    warn "기존 소스 디렉토리 제거: ${KERNEL_SRC_DIR}"
    rm -rf "${KERNEL_SRC_DIR}"
  fi

  info "커널 소스 압축 해제 중..."
  tar -xf "linux-${KERNEL_VERSION}.tar.xz"

  cd "${KERNEL_SRC_DIR}"
  info "RT 패치 적용 중..."
  xzcat "../patch-${PATCH_VERSION}.patch.xz" | patch -p1 --quiet

  # 마커 파일 생성 (재실행 시 스킵하기 위함)
  touch ".rt_patched"
  success "패치 적용 완료"
fi

# ══════════════════════════════════════════════════════════════════════════════
# [4/6] 커널 설정 (PREEMPT_RT 활성화)
# ══════════════════════════════════════════════════════════════════════════════
echo ""
info "━━━ [4/6] 커널 설정 (PREEMPT_RT 활성화) ━━━"

if should_skip 4 is_step4_done; then
  success "커널 설정이 이미 완료되어 있습니다 (CONFIG_PREEMPT_RT=y) — 건너뜀"
else
  cd "${KERNEL_SRC_DIR}"

  # 현재 실행 중인 커널 설정을 기반으로 시작
  RUNNING_CONFIG="/boot/config-$(uname -r)"
  if [[ -f "$RUNNING_CONFIG" ]]; then
    cp "$RUNNING_CONFIG" .config
    info "기존 설정 복사: ${RUNNING_CONFIG}"
  else
    warn "/boot/config-$(uname -r) 없음 — make defconfig 사용"
    make defconfig
  fi

  # 서명 키 비활성화 (빌드 오류 방지)
  scripts/config --set-str SYSTEM_TRUSTED_KEYS ""
  scripts/config --set-str SYSTEM_REVOCATION_KEYS ""

  # PREEMPT_RT 활성화
  scripts/config --disable CONFIG_PREEMPT_NONE
  scripts/config --disable CONFIG_PREEMPT_VOLUNTARY
  scripts/config --enable CONFIG_PREEMPT_RT

  # 디버그 정보 제외 (빌드 시간 및 deb 크기 절감)
  scripts/config --disable CONFIG_DEBUG_INFO
  scripts/config --disable CONFIG_DEBUG_INFO_BTF

  # RT 커널 식별을 위한 LOCALVERSION 설정
  scripts/config --set-str LOCALVERSION "-rt-custom"

  # 새 옵션에 대한 기본값 적용
  make olddefconfig

  if [[ "$BATCH_MODE" -eq 0 ]]; then
    echo ""
    info "menuconfig를 실행합니다."
    info "'General setup' → 'Preemption Model'이 'Fully Preemptible Kernel (Real-Time)'인지 확인하세요."
    echo ""
    make menuconfig
  else
    info "배치 모드: menuconfig 건너뜀 (scripts/config으로 설정 완료)"
  fi

  success "커널 설정 완료"
fi

# ── Dry-run 종료점 ───────────────────────────────────────────────────────────
if [[ "$DRY_RUN" -eq 1 ]]; then
  echo ""
  success "Dry-run 완료. 소스 및 패치가 준비되었습니다: ${KERNEL_SRC_DIR}"
  info "빌드하려면: cd ${KERNEL_SRC_DIR} && make -j${BUILD_THREADS} bindeb-pkg"
  exit 0
fi

# ══════════════════════════════════════════════════════════════════════════════
# [5/6] 커널 빌드
# ══════════════════════════════════════════════════════════════════════════════
echo ""
info "━━━ [5/6] 커널 빌드 ━━━"

if should_skip 5 is_step5_done; then
  success "커널이 이미 빌드되어 있습니다 (.deb 존재) — 건너뜀"
  DEB_IMAGE=$(find "${BUILD_DIR}" -maxdepth 1 -name "linux-image-${KERNEL_VERSION}*.deb" \
    ! -name "*dbg*" -printf '%f\n' 2>/dev/null | head -1)
  info "기존 빌드: ${DEB_IMAGE}"
else
  cd "${KERNEL_SRC_DIR}"
  if [[ "$LOGICAL_CORES" -ne "$PHYSICAL_CORES" ]]; then
    info "CPU: 물리 ${PHYSICAL_CORES}코어 / 논리 ${LOGICAL_CORES}코어 (SMT/HT)"
  fi
  info "빌드 스레드: ${BUILD_THREADS} (논리 코어 수)  |  시작: $(date '+%H:%M:%S')"
  make -j"${BUILD_THREADS}" bindeb-pkg
  success "커널 빌드 완료  |  종료: $(date '+%H:%M:%S')"
fi

# ══════════════════════════════════════════════════════════════════════════════
# [6/6] .deb 패키지 설치 + GRUB 설정
# ══════════════════════════════════════════════════════════════════════════════
echo ""
info "━━━ [6/6] .deb 패키지 설치 ━━━"

if should_skip 6 is_step6_done; then
  success "RT 커널이 이미 설치되어 있습니다 (${RT_KERNEL_FULL}) — 건너뜀"
else
  cd "${BUILD_DIR}"

  # 현재 빌드에 해당하는 .deb만 설치 (이전 빌드와 구분)
  DEB_IMAGE=$(find . -maxdepth 1 -name "linux-image-${KERNEL_VERSION}*.deb" ! -name "*dbg*" -printf '%f\n' 2>/dev/null | head -1)
  DEB_HEADERS=$(find . -maxdepth 1 -name "linux-headers-${KERNEL_VERSION}*.deb" -printf '%f\n' 2>/dev/null | head -1)

  if [[ -z "$DEB_IMAGE" ]]; then
    error "linux-image .deb 패키지를 찾을 수 없습니다. 빌드를 먼저 실행하세요: $0 --force-step 5"
  fi

  info "설치할 패키지:"
  info "  Image:   ${DEB_IMAGE}"
  [[ -n "$DEB_HEADERS" ]] && info "  Headers: ${DEB_HEADERS}"

  # NVIDIA DKMS는 커스텀 RT 커널 헤더 이름을 인식하지 못해 빌드 실패할 수 있음.
  # dpkg post-install 트리거에서 DKMS autoinstall이 실패하면 dpkg 전체가 실패하므로,
  # 커널 설치 중 DKMS autoinstall 훅을 일시 비활성화한 뒤 복원한다.
  DKMS_POSTINST="/etc/kernel/postinst.d/dkms"
  DKMS_DISABLED=0
  if [[ -f "$DKMS_POSTINST" ]]; then
    info "DKMS autoinstall 훅 일시 비활성화..."
    chmod -x "$DKMS_POSTINST"
    DKMS_DISABLED=1
  fi

  # headers를 먼저 설치하여 이후 DKMS 빌드 시 헤더가 존재하도록 함
  DEBS_TO_INSTALL=()
  [[ -n "$DEB_HEADERS" ]] && DEBS_TO_INSTALL+=("${DEB_HEADERS}")
  DEBS_TO_INSTALL+=("${DEB_IMAGE}")

  dpkg -i "${DEBS_TO_INSTALL[@]}"

  # DKMS 훅 복원
  if [[ "$DKMS_DISABLED" -eq 1 ]] && [[ -f "$DKMS_POSTINST" ]]; then
    chmod +x "$DKMS_POSTINST"
    info "DKMS autoinstall 훅 복원 완료"
  fi

  # NVIDIA GPU가 있으면 DKMS 수동 빌드 시도 (실패해도 무시)
  if [[ "$HAS_NVIDIA" == "yes" ]]; then
    info "NVIDIA DKMS 모듈 빌드 시도 (커널: ${RT_KERNEL_FULL})..."
    set +e
    dkms autoinstall -k "${RT_KERNEL_FULL}" 2>&1
    DKMS_RC=$?
    set -e
    if [[ $DKMS_RC -eq 0 ]]; then
      success "NVIDIA DKMS 모듈 빌드 성공"
    else
      warn "NVIDIA DKMS 모듈 빌드 실패 — RT 커널 자체는 정상 설치됨"
      warn "NVIDIA 드라이버가 필요한 경우 재부팅 후 수동 설치:"
      warn "  sudo dkms autoinstall -k \$(uname -r)"
    fi
  fi

  success "커널 패키지 설치 완료"

  # ── GRUB 기본 부팅 항목을 RT 커널로 설정 ────────────────────────────────────
  echo ""
  info "━━━ GRUB 기본 부팅 항목을 RT 커널로 설정 ━━━"

  # 설치된 RT 커널의 정확한 버전 문자열을 .deb 파일명에서 추출
  RT_KERNEL_VER=$(echo "$DEB_IMAGE" | sed -n 's/linux-image-\(.*\)_.*\.deb/\1/p')

  if [[ -z "$RT_KERNEL_VER" ]]; then
    warn "RT 커널 버전을 .deb 파일명에서 추출할 수 없습니다 — GRUB 설정을 수동으로 확인하세요"
  else
    info "설치된 RT 커널 버전: ${RT_KERNEL_VER}"

    # grub.cfg 갱신
    update-grub 2>/dev/null

    # /boot/grub/grub.cfg에서 RT 커널의 menuentry 표시 이름을 추출
    GRUB_ENTRY=""
    if [[ -f /boot/grub/grub.cfg ]]; then
      SUBMENU_TITLE=$(grep -oP "submenu\s+'\K[^']+" /boot/grub/grub.cfg | head -1)
      ENTRY_TITLE=$(grep -v recovery /boot/grub/grub.cfg \
        | grep -oP "menuentry\s+'\K[^']*${RT_KERNEL_VER}[^']*(?=')" \
        | head -1)

      if [[ -n "$SUBMENU_TITLE" && -n "$ENTRY_TITLE" ]]; then
        GRUB_ENTRY="${SUBMENU_TITLE}>${ENTRY_TITLE}"
      elif [[ -n "$ENTRY_TITLE" ]]; then
        GRUB_ENTRY="$ENTRY_TITLE"
      fi
    fi

    if [[ -n "$GRUB_ENTRY" ]]; then
      info "GRUB 메뉴 항목: ${GRUB_ENTRY}"

      GRUB_FILE="/etc/default/grub"
      cp "$GRUB_FILE" "${GRUB_FILE}.bak.rt.$(date +%Y%m%d%H%M%S)"

      if grep -q '^GRUB_DEFAULT=' "$GRUB_FILE"; then
        sed -i "s|^GRUB_DEFAULT=.*|GRUB_DEFAULT=\"${GRUB_ENTRY}\"|" "$GRUB_FILE"
      else
        echo "GRUB_DEFAULT=\"${GRUB_ENTRY}\"" >> "$GRUB_FILE"
      fi

      update-grub
      success "GRUB 기본 부팅이 RT 커널(${RT_KERNEL_VER})로 설정되었습니다"

      info "검증 — /etc/default/grub 의 GRUB_DEFAULT:"
      grep '^GRUB_DEFAULT=' "$GRUB_FILE" | sed 's/^/  /'
    else
      warn "GRUB 메뉴에서 RT 커널 항목을 찾을 수 없습니다"
      warn "수동 설정 방법:"
      warn "  1. grep menuentry /boot/grub/grub.cfg | grep '${RT_KERNEL_VER}'"
      warn "  2. sudo grub-set-default 'Advanced options for Ubuntu>Ubuntu, with Linux ${RT_KERNEL_VER}'"
      warn "  3. sudo update-grub"
    fi
  fi

  # ── NVIDIA GRUB 최적화 ──────────────────────────────────────────────────────
  if [[ "$HAS_NVIDIA" == "yes" ]]; then
    echo ""
    info "NVIDIA MSI 최적화 적용 중..."
    GRUB_FILE="/etc/default/grub"

    if grep -q "nvidia.NVreg_EnableMSI=1" "$GRUB_FILE" 2>/dev/null; then
      info "NVIDIA MSI 설정이 이미 존재합니다 — 건너뜀"
    else
      cp "$GRUB_FILE" "${GRUB_FILE}.bak.$(date +%Y%m%d%H%M%S)"
      if grep -q '^GRUB_CMDLINE_LINUX_DEFAULT=' "$GRUB_FILE"; then
        sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="\(.*\)"/GRUB_CMDLINE_LINUX_DEFAULT="\1 nvidia.NVreg_EnableMSI=1"/' "$GRUB_FILE"
      else
        warn "GRUB_CMDLINE_LINUX_DEFAULT not found in ${GRUB_FILE} — adding new entry"
        echo 'GRUB_CMDLINE_LINUX_DEFAULT="nvidia.NVreg_EnableMSI=1"' >> "$GRUB_FILE"
      fi
      update-grub
      success "NVIDIA MSI 설정 추가 완료"
    fi
  fi
fi

# ── 완료 ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${GREEN}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${GREEN}║            PREEMPT_RT 커널 설치 완료!                ║${NC}"
echo -e "${BOLD}${GREEN}╚══════════════════════════════════════════════════════╝${NC}"
echo ""

# 최종 상태 표시
show_status

info "다음 단계:"
info "  1. sudo reboot"
info "  2. uname -v | grep PREEMPT_RT   (PREEMPT_RT 확인)"
info "  3. uname -r                      (커널 버전 확인)"
echo ""
info "문제 해결:"
info "  재부팅 후 RT 커널이 아닌 경우:"
info "    1. 재부팅 시 GRUB 메뉴에서 'Advanced options' → RT 커널 선택"
info "    2. 또는: sudo grep menuentry /boot/grub/grub.cfg  (RT 항목 확인)"
info "    3. sudo grub-set-default '<RT 커널 항목>'  →  sudo update-grub"
echo ""
info "RT 최적화 가이드: docs/RT_OPTIMIZATION.md"
info "IRQ affinity:     sudo ./setup_irq_affinity.sh"
info "UDP 최적화:       sudo ./setup_udp_optimization.sh"
echo ""
