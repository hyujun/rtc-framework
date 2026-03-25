#!/bin/bash
# setup_nvidia_rt.sh — NVIDIA 드라이버 + Linux RT 커널 공존 환경 설정
#
# NVIDIA GPU(디스플레이 전용)와 PREEMPT_RT 커널이 안정적으로 공존하도록
# modprobe, IRQ affinity, persistence mode, nouveau 블랙리스트, DKMS를 설정한다.
# GRUB/디스플레이/CPU governor 설정은 별도 스크립트로 분리됨 (7/7에서 호출).
#
# 대상: Ubuntu 22.04 / 24.04, PREEMPT_RT 커널, NVIDIA GPU (CUDA 미사용)
# 제어 루프: 500Hz (2ms period), 목표 max jitter: < 200μs
#
# Usage:
#   sudo ./setup_nvidia_rt.sh          # 전체 설정 적용
#   sudo ./setup_nvidia_rt.sh --help   # 도움말
#
# 실행 시점: RT 커널 부팅 후, ROS2 실행 전 (1회 설정, 재부팅 필요)
# 검증:
#   cyclictest --mlockall --smp --priority=80 --interval=2000 --duration=60
#   cat /proc/interrupts | grep nvidia   # NVIDIA IRQ 확인
#   nvidia-smi -q -d PERFORMANCE         # persistence mode 확인

set -euo pipefail

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR_NVIDIA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR_NVIDIA}/lib/rt_common.sh"
make_logger "NVIDIA-RT"

# NVIDIA 스크립트 전용 로깅 재정의 (기존 호출 코드와 호환)
success() { echo -e "${GREEN}[SUCCESS]${NC} $*"; }
warn()    { echo -e "${YELLOW}[WARNING]${NC} $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*" >&2; }
info()    { echo -e "${CYAN}[INFO]${NC} $*"; }

# ── 지연 작업 플래그 (멱등성 최적화) ───────────────────────────────────────
_NEED_DAEMON_RELOAD=0
_NEED_INITRAMFS_UPDATE=0

# ── Help ────────────────────────────────────────────────────────────────────
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo ""
  echo -e "${BOLD}setup_nvidia_rt.sh — NVIDIA + RT 커널 공존 환경 설정${NC}"
  echo ""
  echo "Usage: sudo $0"
  echo ""
  echo "수행 작업:"
  echo "  1. Pre-flight checks (Ubuntu 버전, RT 커널, NVIDIA GPU, CPU 레이아웃)"
  echo "  2. NVIDIA modprobe 설정 (/etc/modprobe.d/nvidia-rt.conf)"
  echo "  3. NVIDIA IRQ affinity systemd 서비스"
  echo "  4. nvidia-smi persistence mode 서비스"
  echo "  5. nouveau 블랙리스트 (활성 시)"
  echo "  6. NVIDIA DKMS 모듈 빌드 (RT 커널용)"
  echo "  7. 연관 스크립트 호출 (GRUB, 디스플레이, CPU governor) + 검증 요약"
  echo ""
  exit 0
fi

# ── Banner ──────────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║     NVIDIA + RT Kernel Coexistence Setup                  ║${NC}"
echo -e "${BOLD}${CYAN}║     Target: 500Hz control loop, <200μs jitter             ║${NC}"
echo -e "${BOLD}${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
echo ""

# ── Tracking arrays for summary ─────────────────────────────────────────────
CHANGES_APPLIED=()
BACKUP_FILES=()
WARNINGS=()

# get_physical_cores() — rt_common.sh에서 제공

# ── Helper: backup a file before modifying ──────────────────────────────────
backup_file() {
  local file="$1"
  if [[ -f "$file" ]]; then
    local bak="${file}.bak.$(date +%Y%m%d_%H%M%S)"
    cp "$file" "$bak"
    BACKUP_FILES+=("$bak")
    info "백업: ${bak}"
  fi
}

# ══════════════════════════════════════════════════════════════════════════════
# [1/11] Pre-flight Checks
# ══════════════════════════════════════════════════════════════════════════════
echo -e "${BOLD}━━━ [1/7] Pre-flight Checks ━━━${NC}"
echo ""

# ── Root check ──────────────────────────────────────────────────────────────
if [[ "$EUID" -ne 0 ]]; then
  error "Root 권한이 필요합니다. 실행: sudo $0"
  exit 1
fi
success "Root 권한 확인"

# ── Ubuntu version detection ────────────────────────────────────────────────
if [[ ! -f /etc/os-release ]]; then
  error "/etc/os-release 파일을 찾을 수 없습니다"
  exit 1
fi

# shellcheck source=/dev/null
source /etc/os-release
UBUNTU_VERSION="${VERSION_ID:-unknown}"

if [[ "$UBUNTU_VERSION" != "22.04" && "$UBUNTU_VERSION" != "24.04" ]]; then
  error "지원하지 않는 Ubuntu 버전: ${UBUNTU_VERSION}"
  error "이 스크립트는 Ubuntu 22.04 또는 24.04만 지원합니다."
  exit 1
fi
success "Ubuntu ${UBUNTU_VERSION} 감지"

# ── RT kernel check ─────────────────────────────────────────────────────────
KERNEL_VER=$(uname -r)
if [[ "$KERNEL_VER" =~ rt|realtime ]]; then
  success "RT 커널 확인: ${KERNEL_VER}"
else
  warn "현재 커널(${KERNEL_VER})에 'rt' 또는 'realtime' 접미사가 없습니다"
  warn "PREEMPT_RT 커널로 부팅 후 다시 실행하세요"
  WARNINGS+=("RT 커널이 아닐 수 있음: ${KERNEL_VER}")
fi

# ── NVIDIA GPU detection ────────────────────────────────────────────────────
NVIDIA_GPU_MODEL="unknown"
NVIDIA_DRIVER_VER="unknown"

if command -v nvidia-smi &>/dev/null; then
  NVIDIA_GPU_MODEL=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo "unknown")
  NVIDIA_DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || echo "unknown")
elif command -v lspci &>/dev/null && lspci 2>/dev/null | grep -qi nvidia; then
  NVIDIA_GPU_MODEL=$(lspci 2>/dev/null | grep -i nvidia | head -1 | sed 's/.*: //')
  # Try dpkg for driver version
  NVIDIA_DRIVER_VER=$(dpkg -l 2>/dev/null | grep -oP 'nvidia-driver-\K[0-9]+' | head -1 || echo "unknown")
fi

if [[ "$NVIDIA_GPU_MODEL" == "unknown" ]]; then
  if ! lspci 2>/dev/null | grep -qi nvidia; then
    error "NVIDIA GPU를 찾을 수 없습니다. 이 스크립트는 NVIDIA GPU가 필요합니다."
    exit 1
  fi
fi

success "NVIDIA GPU: ${NVIDIA_GPU_MODEL}"
info "  드라이버 버전: ${NVIDIA_DRIVER_VER}"

# ── CPU layout auto-detection (rt_common.sh 공유 함수 사용) ─────────────────
compute_cpu_layout
RT_CORES=$(compute_expected_isolated)
OS_CORES="$OS_CORES_DESC"

if [[ "$LOGICAL_CORES" -ne "$TOTAL_CORES" ]]; then
  info "SMT/Hyper-Threading 감지: 논리 ${LOGICAL_CORES}코어, 물리 ${TOTAL_CORES}코어"
  info "thread_config.hpp 레이아웃과 일치하도록 물리 코어(${TOTAL_CORES}) 기준으로 설정합니다"
fi

if [[ "$TOTAL_CORES" -lt 4 ]]; then
  error "물리 CPU 코어가 ${TOTAL_CORES}개입니다. 최소 4코어가 필요합니다."
  error "  Core 0-1: OS/NVIDIA IRQ, Core 2+: RT 제어 루프"
  exit 1
fi

if [[ "$TOTAL_CORES" -le 4 ]]; then
  warn "4코어 시스템: OS 코어가 1개(Core 0)뿐입니다"
  warn "X11/Wayland + NVIDIA IRQ + 시스템 프로세스가 모두 Core 0에서 동작하므로"
  warn "디스플레이 렌더링 프레임이 밀려 화면 끊김이 발생할 수 있습니다"
  WARNINGS+=("4코어 시스템 — 디스플레이 성능 저하 가능 (OS 코어 1개)")
fi

success "CPU 레이아웃 (${TOTAL_CORES}코어):"
info "  OS / NVIDIA IRQ:  Core ${OS_CORES}"
info "  RT 격리 대상:     Core ${RT_CORES} (thread_config.hpp 레이아웃과 일치)"
info "  IRQ affinity mask: 0x${IRQ_AFFINITY_MASK}"

# ── Display server detection ──────────────────────────────────────────────
DISPLAY_SERVER="unknown"
if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
  DISPLAY_SERVER="wayland"
elif [[ -n "${DISPLAY:-}" ]]; then
  DISPLAY_SERVER="x11"
else
  # 비-GUI 환경에서도 X11 설정 파일은 설치 (재부팅 후 GUI 시작 시 적용)
  DISPLAY_SERVER="headless"
fi
info "디스플레이 서버: ${DISPLAY_SERVER}"

# ── nouveau check ───────────────────────────────────────────────────────────
NOUVEAU_ACTIVE=0
if lsmod 2>/dev/null | grep -q nouveau; then
  NOUVEAU_ACTIVE=1
  warn "nouveau 모듈이 활성 상태입니다!"
  warn "NVIDIA 독점 드라이버와 충돌합니다. [6/11] 단계에서 블랙리스트를 설정합니다."
  WARNINGS+=("nouveau 모듈 활성 — 블랙리스트 적용 예정")
else
  success "nouveau 비활성 확인"
fi

# ── cyclictest check ────────────────────────────────────────────────────────
if command -v cyclictest &>/dev/null; then
  success "cyclictest 설치 확인"
else
  warn "cyclictest가 설치되어 있지 않습니다"
  warn "설치: sudo apt-get install -y rt-tests"
  WARNINGS+=("cyclictest 미설치 — sudo apt-get install -y rt-tests")
fi

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# [2/7] NVIDIA modprobe Configuration
# ══════════════════════════════════════════════════════════════════════════════
echo -e "${BOLD}━━━ [2/7] NVIDIA modprobe 설정 ━━━${NC}"
echo ""

MODPROBE_CONF="/etc/modprobe.d/nvidia-rt.conf"
MODPROBE_CONTENT="# NVIDIA + RT 커널 공존을 위한 modprobe 설정
# Generated by setup_nvidia_rt.sh
#
# NVreg_EnableGpuFirmware=0 : GPU 펌웨어 로딩 비활성화 (RT 지연 방지)
# NVreg_PreserveVideoMemoryAllocations=1 : suspend/resume 시 VRAM 보존
# modeset=1 : DRM KMS 활성화 (Wayland/X11 호환)
options nvidia NVreg_EnableGpuFirmware=0
options nvidia NVreg_PreserveVideoMemoryAllocations=1
options nvidia-drm modeset=1"

if [[ -f "$MODPROBE_CONF" ]] && diff -q <(echo "$MODPROBE_CONTENT") "$MODPROBE_CONF" &>/dev/null; then
  info "이미 동일한 설정이 존재합니다 — 건너뜀"
else
  backup_file "$MODPROBE_CONF"
  echo "$MODPROBE_CONTENT" > "$MODPROBE_CONF"
  success "${MODPROBE_CONF} 작성 완료"
  CHANGES_APPLIED+=("NVIDIA modprobe 설정: ${MODPROBE_CONF}")
fi

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# [3/7] NVIDIA IRQ Affinity systemd Service
# ══════════════════════════════════════════════════════════════════════════════
echo -e "${BOLD}━━━ [3/7] NVIDIA IRQ Affinity 서비스 ━━━${NC}"
echo ""

IRQ_SERVICE="/etc/systemd/system/nvidia-irq-affinity.service"
IRQ_SCRIPT="/usr/local/bin/nvidia-irq-affinity.sh"

# IRQ affinity 스크립트 생성
# 4코어: 0x1 = Core 0 only (Core 1은 RT Control이 사용)
# 6코어+: 0x3 = Core 0-1
IRQ_SCRIPT_CONTENT="#!/bin/bash
# nvidia-irq-affinity.sh — NVIDIA IRQ를 OS 전용 코어로 제한
# Generated by setup_nvidia_rt.sh
# Affinity mask은 thread_config.hpp의 코어 레이아웃과 일치
set -euo pipefail

AFFINITY_MASK=\"${IRQ_AFFINITY_MASK}\"
PINNED=0

# /proc/interrupts에서 nvidia 관련 IRQ 검색
while IFS= read -r line; do
  irq=\$(echo \"\$line\" | awk -F: '{print \$1}' | tr -d ' ')
  if [[ -n \"\$irq\" && -f \"/proc/irq/\${irq}/smp_affinity\" ]]; then
    echo \"\$AFFINITY_MASK\" > \"/proc/irq/\${irq}/smp_affinity\" 2>/dev/null && ((PINNED++)) || true
  fi
done < <(grep -E 'nvidia|NVrm' /proc/interrupts 2>/dev/null || true)

# Ubuntu 24.04: /sys/kernel/irq/ 폴백 (일부 IRQ가 /proc/interrupts에 없을 수 있음)
if [[ -d /sys/kernel/irq ]]; then
  for irq_dir in /sys/kernel/irq/*/; do
    irq=\$(basename \"\$irq_dir\")
    actions_file=\"\${irq_dir}actions\"
    if [[ -f \"\$actions_file\" ]] && grep -qE 'nvidia|NVrm' \"\$actions_file\" 2>/dev/null; then
      if [[ -f \"/proc/irq/\${irq}/smp_affinity\" ]]; then
        echo \"\$AFFINITY_MASK\" > \"/proc/irq/\${irq}/smp_affinity\" 2>/dev/null && ((PINNED++)) || true
      fi
    fi
  done
fi

echo \"nvidia-irq-affinity: pinned \${PINNED} NVIDIA IRQs (affinity mask: 0x\${AFFINITY_MASK})\""

# IRQ affinity systemd 서비스
# 참고: setup_irq_affinity.sh는 모든 IRQ(NIC 포함)를 일회성으로 pin하는 스크립트.
# 이 서비스는 NVIDIA IRQ만 대상으로 하며, 재부팅 시 자동 적용을 보장한다.
# 두 스크립트는 병행 사용 가능 (setup_irq_affinity.sh가 더 넓은 범위를 커버).
IRQ_SERVICE_CONTENT="[Unit]
Description=Pin NVIDIA IRQs to OS cores for RT kernel
After=nvidia-persistenced.service
After=multi-user.target

[Service]
Type=oneshot
ExecStart=${IRQ_SCRIPT}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target"

# 스크립트 설치 (멱등: 내용 동일하면 skip)
if write_file_if_changed "$IRQ_SCRIPT" "$IRQ_SCRIPT_CONTENT" false; then
  chmod +x "$IRQ_SCRIPT"
  success "IRQ affinity 스크립트: ${IRQ_SCRIPT}"
else
  info "IRQ affinity 스크립트 이미 동일 — 건너뜀"
fi

# 서비스 설치 (멱등)
if write_file_if_changed "$IRQ_SERVICE" "$IRQ_SERVICE_CONTENT"; then
  success "서비스 파일: ${IRQ_SERVICE}"
  CHANGES_APPLIED+=("NVIDIA IRQ affinity 서비스: ${IRQ_SERVICE}")
  _NEED_DAEMON_RELOAD=1
else
  info "서비스가 이미 동일한 설정으로 존재합니다 — 건너뜀"
fi

if [[ "$_NEED_DAEMON_RELOAD" -eq 1 ]]; then
  : # daemon-reload는 스크립트 끝에서 한 번만 실행
fi
systemctl enable nvidia-irq-affinity.service 2>/dev/null || true
# 현재 NVIDIA IRQ가 있으면 즉시 실행 (timeout 30초 — systemd 블로킹 방지)
if grep -qE "nvidia|NVrm" /proc/interrupts 2>/dev/null; then
  timeout 30 systemctl start nvidia-irq-affinity.service 2>/dev/null || true
  success "NVIDIA IRQ affinity 즉시 적용 완료"
else
  info "현재 NVIDIA IRQ 없음 — 재부팅 후 자동 적용됩니다"
fi

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# [4/7] nvidia-smi Persistence Mode
# ══════════════════════════════════════════════════════════════════════════════
echo -e "${BOLD}━━━ [4/7] nvidia-smi Persistence Mode ━━━${NC}"
echo ""

PERSIST_SERVICE="/etc/systemd/system/nvidia-persistence.service"

# 현재 즉시 활성화
if command -v nvidia-smi &>/dev/null; then
  nvidia-smi -pm 1 2>/dev/null && success "Persistence mode 활성화 (즉시)" || warn "nvidia-smi -pm 1 실패 (재부팅 후 서비스로 적용)"
else
  warn "nvidia-smi를 찾을 수 없습니다 — 서비스만 생성합니다"
fi

PERSIST_SERVICE_CONTENT="[Unit]
Description=NVIDIA Persistence Mode for RT kernel
After=nvidia-persistenced.service
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/nvidia-smi -pm 1
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target"

if write_file_if_changed "$PERSIST_SERVICE" "$PERSIST_SERVICE_CONTENT"; then
  _NEED_DAEMON_RELOAD=1
  success "Persistence mode 서비스: ${PERSIST_SERVICE}"
  CHANGES_APPLIED+=("NVIDIA persistence mode 서비스: ${PERSIST_SERVICE}")
else
  info "Persistence 서비스가 이미 동일한 설정으로 존재합니다 — 건너뜀"
fi
systemctl enable nvidia-persistence.service 2>/dev/null || true
timeout 30 systemctl start nvidia-persistence.service 2>/dev/null || true

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# [5/7] nouveau Blacklist (if active)
# ══════════════════════════════════════════════════════════════════════════════
echo -e "${BOLD}━━━ [5/7] nouveau 블랙리스트 ━━━${NC}"
echo ""

NOUVEAU_CONF="/etc/modprobe.d/blacklist-nouveau.conf"
NOUVEAU_CONTENT="# nouveau 모듈 블랙리스트 (NVIDIA 독점 드라이버와 충돌 방지)
# Generated by setup_nvidia_rt.sh
blacklist nouveau
options nouveau modeset=0"

if [[ "$NOUVEAU_ACTIVE" -eq 1 ]]; then
  if [[ -f "$NOUVEAU_CONF" ]] && diff -q <(echo "$NOUVEAU_CONTENT") "$NOUVEAU_CONF" &>/dev/null; then
    info "블랙리스트가 이미 동일한 설정으로 존재합니다 — 건너뜀"
  else
    backup_file "$NOUVEAU_CONF"
    echo "$NOUVEAU_CONTENT" > "$NOUVEAU_CONF"
    success "nouveau 블랙리스트: ${NOUVEAU_CONF}"
    CHANGES_APPLIED+=("nouveau 블랙리스트: ${NOUVEAU_CONF}")
    _NEED_INITRAMFS_UPDATE=1
  fi
else
  # nouveau가 비활성이어도 블랙리스트 파일이 없으면 예방적으로 설치
  if [[ ! -f "$NOUVEAU_CONF" ]]; then
    echo "$NOUVEAU_CONTENT" > "$NOUVEAU_CONF"
    info "nouveau가 비활성이지만, 예방적으로 블랙리스트 파일을 설치했습니다"
    CHANGES_APPLIED+=("nouveau 블랙리스트 (예방): ${NOUVEAU_CONF}")
  else
    info "nouveau 비활성 + 블랙리스트 존재 — 건너뜀"
  fi
fi

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# [6/7] NVIDIA DKMS Module Build for RT Kernel
echo -e "${BOLD}━━━ [6/7] NVIDIA DKMS 모듈 빌드 ━━━${NC}"
echo ""

# RT 커널로 부팅하면 기존 NVIDIA DKMS 모듈이 빌드되지 않아 nvidia-smi가 작동하지 않을 수 있다.
# 커스텀 RT 커널에서 NVIDIA DKMS 빌드가 실패하는 원인 3가지:
#   1. `dkms autoinstall` → apport 검증: "kernel package linux-headers-X is not supported"
#   2. NVIDIA 빌드 시스템 → PREEMPT_RT 감지 후 빌드 차단
#   3. dkms.conf → BUILD_EXCLUSIVE_CONFIG="!CONFIG_PREEMPT_RT" 지시문
# 해결:
#   - `dkms build -m nvidia -v VERSION -k KERNEL`으로 apport 우회
#   - `IGNORE_PREEMPT_RT_PRESENCE=1` 환경변수로 RT 감지 차단 우회
#   - dkms.conf의 BUILD_EXCLUSIVE_CONFIG 라인 주석 처리
DKMS_NEEDED=0

if ! command -v nvidia-smi &>/dev/null; then
  # nvidia-smi 바이너리 자체가 없는 경우 — NVIDIA 드라이버 패키지 미설치
  warn "nvidia-smi를 찾을 수 없습니다"
  # NVIDIA 드라이버 패키지가 설치되어 있는지 확인
  NVIDIA_PKG=$(dpkg -l 2>/dev/null | grep -oP 'nvidia-driver-\K[0-9]+' | sort -rn | head -1 || true)
  if [[ -n "$NVIDIA_PKG" ]]; then
    info "NVIDIA 드라이버 패키지 감지: nvidia-driver-${NVIDIA_PKG}"
    DKMS_NEEDED=1
  else
    warn "NVIDIA 드라이버 패키지가 설치되지 않았습니다"
    warn "설치: sudo apt-get install -y nvidia-driver-535  (또는 최신 버전)"
    WARNINGS+=("NVIDIA 드라이버 패키지 미설치 — sudo apt-get install nvidia-driver-XXX")
  fi
elif ! nvidia-smi &>/dev/null; then
  # nvidia-smi 바이너리는 있지만 실행 실패 — 커널 모듈 미로드
  warn "nvidia-smi 실행 실패 — NVIDIA 커널 모듈이 로드되지 않았습니다"
  DKMS_NEEDED=1
else
  success "nvidia-smi 정상 작동 — DKMS 빌드 불필요"
fi

if [[ "$DKMS_NEEDED" -eq 1 ]]; then
  if ! command -v dkms &>/dev/null; then
    warn "dkms가 설치되어 있지 않습니다"
    warn "설치: sudo apt-get install -y dkms"
    WARNINGS+=("dkms 미설치 — sudo apt-get install -y dkms")
  else
    # ── 커널 헤더 확인 ──────────────────────────────────────────────────────
    # 커스텀 RT 커널은 linux-headers-xxx-rt-custom .deb로 설치되었을 수 있다.
    # /lib/modules/$(uname -r)/build 심볼릭 링크가 존재하면 헤더 사용 가능.
    HEADERS_DIR="/lib/modules/${KERNEL_VER}/build"
    if [[ ! -d "$HEADERS_DIR" ]]; then
      warn "커널 헤더 디렉토리가 없습니다: ${HEADERS_DIR}"
      warn "커스텀 RT 커널 헤더를 먼저 설치하세요:"
      warn "  build_rt_kernel.sh 빌드 디렉토리에서:"
      warn "    sudo dpkg -i linux-headers-${KERNEL_VER}_*.deb"
      WARNINGS+=("커널 헤더 미설치 — /lib/modules/${KERNEL_VER}/build 없음")
    else
      info "커널 헤더 확인: ${HEADERS_DIR}"

      # ── NVIDIA DKMS 모듈 버전 탐색 ─────────────────────────────────────
      # /var/lib/dkms/nvidia/ 에서 설치된 NVIDIA DKMS 소스 버전을 찾는다.
      NVIDIA_DKMS_VER=""
      if [[ -d /var/lib/dkms/nvidia ]]; then
        NVIDIA_DKMS_VER=$(ls -1 /var/lib/dkms/nvidia/ 2>/dev/null \
          | grep -E '^[0-9]+\.' | sort -V | tail -1 || true)
      fi

      if [[ -z "$NVIDIA_DKMS_VER" ]]; then
        warn "NVIDIA DKMS 소스를 찾을 수 없습니다 (/var/lib/dkms/nvidia/)"
        warn "NVIDIA DKMS 패키지 설치:"
        warn "  sudo apt-get install -y nvidia-dkms-535  (또는 설치된 드라이버 버전)"
        WARNINGS+=("NVIDIA DKMS 소스 없음 — nvidia-dkms-XXX 설치 필요")
      else
        info "NVIDIA DKMS 소스 버전: ${NVIDIA_DKMS_VER}"

        # ── 이미 빌드되었는지 확인 ────────────────────────────────────────
        DKMS_STATUS=$(dkms status nvidia/"${NVIDIA_DKMS_VER}" -k "${KERNEL_VER}" 2>/dev/null || true)
        if echo "$DKMS_STATUS" | grep -q "installed"; then
          success "NVIDIA ${NVIDIA_DKMS_VER} 모듈이 이미 설치됨 (커널: ${KERNEL_VER})"
        else
          # ── RT 커널용 NVIDIA DKMS 빌드 준비 ─────────────────────────────
          # NVIDIA 빌드 시스템은 PREEMPT_RT 커널을 명시적으로 차단한다:
          #   1. conftest.sh에서 CONFIG_PREEMPT_RT 감지 → 빌드 거부
          #   2. dkms.conf의 BUILD_EXCLUSIVE_CONFIG="!CONFIG_PREEMPT_RT"
          # 이 두 가지를 우회해야 RT 커널에서 NVIDIA 모듈을 빌드할 수 있다.

          # (a) dkms.conf의 BUILD_EXCLUSIVE_CONFIG 주석 처리
          NVIDIA_DKMS_CONF="/usr/src/nvidia-${NVIDIA_DKMS_VER}/dkms.conf"
          if [[ -f "$NVIDIA_DKMS_CONF" ]]; then
            if grep -q 'BUILD_EXCLUSIVE_CONFIG.*PREEMPT_RT' "$NVIDIA_DKMS_CONF" 2>/dev/null; then
              info "NVIDIA dkms.conf에서 BUILD_EXCLUSIVE_CONFIG (PREEMPT_RT 차단) 비활성화..."
              backup_file "$NVIDIA_DKMS_CONF"
              sed -i 's/^\(BUILD_EXCLUSIVE_CONFIG=.*PREEMPT_RT\)/#\1  # Disabled for RT kernel by setup_nvidia_rt.sh/' \
                "$NVIDIA_DKMS_CONF"
              success "BUILD_EXCLUSIVE_CONFIG 비활성화 완료"
              CHANGES_APPLIED+=("NVIDIA dkms.conf BUILD_EXCLUSIVE_CONFIG 비활성화")
            fi
          fi

          # (b) IGNORE_PREEMPT_RT_PRESENCE=1 환경변수 설정
          # NVIDIA 빌드 시스템의 conftest.sh가 RT 커널 감지 후 빌드를 거부하는 것을 우회
          export IGNORE_PREEMPT_RT_PRESENCE=1
          export IGNORE_CC_MISMATCH=1
          info "NVIDIA RT 빌드 우회 환경변수 설정:"
          info "  IGNORE_PREEMPT_RT_PRESENCE=1"
          info "  IGNORE_CC_MISMATCH=1"

          # ── dkms build + install ────────────────────────────────────────
          info "NVIDIA ${NVIDIA_DKMS_VER} 모듈 빌드 시작 (커널: ${KERNEL_VER})..."
          info "  dkms build -m nvidia -v ${NVIDIA_DKMS_VER} -k ${KERNEL_VER}"

          set +e
          BUILD_OUTPUT=$(dkms build -m nvidia -v "${NVIDIA_DKMS_VER}" -k "${KERNEL_VER}" 2>&1)
          BUILD_RC=$?
          set -e

          if [[ $BUILD_RC -eq 0 ]]; then
            success "NVIDIA DKMS 빌드 성공"

            # install
            info "NVIDIA DKMS 모듈 설치 중..."
            set +e
            INSTALL_OUTPUT=$(dkms install -m nvidia -v "${NVIDIA_DKMS_VER}" -k "${KERNEL_VER}" 2>&1)
            INSTALL_RC=$?
            set -e

            if [[ $INSTALL_RC -eq 0 ]]; then
              success "NVIDIA DKMS 모듈 설치 성공"
              CHANGES_APPLIED+=("NVIDIA DKMS 모듈 빌드/설치: nvidia/${NVIDIA_DKMS_VER} for ${KERNEL_VER}")
            else
              warn "NVIDIA DKMS 모듈 설치 실패:"
              echo "$INSTALL_OUTPUT" | tail -5
              WARNINGS+=("NVIDIA DKMS install 실패 — 재부팅 후 재시도")
            fi
          else
            warn "NVIDIA DKMS 빌드 실패:"
            echo "$BUILD_OUTPUT" | tail -10

            # 빌드 실패 원인 분석
            if echo "$BUILD_OUTPUT" | grep -qi "gcc.*not found\|cc.*not found"; then
              warn "빌드 도구(gcc) 미설치: sudo apt-get install -y build-essential"
              WARNINGS+=("gcc 미설치 — sudo apt-get install build-essential")
            fi
            if echo "$BUILD_OUTPUT" | grep -qi "kernel source\|kernel header"; then
              warn "커널 헤더/소스 문제: /lib/modules/${KERNEL_VER}/build 확인"
              WARNINGS+=("커널 헤더/소스 불완전 — Makefile 등 확인 필요")
            fi

            # make.log 경로 안내
            MAKE_LOG="/var/lib/dkms/nvidia/${NVIDIA_DKMS_VER}/build/make.log"
            if [[ -f "$MAKE_LOG" ]]; then
              warn "상세 빌드 로그: ${MAKE_LOG}"
              warn "마지막 20줄:"
              tail -20 "$MAKE_LOG" 2>/dev/null | while IFS= read -r line; do
                warn "  ${line}"
              done
            fi
            WARNINGS+=("NVIDIA DKMS 빌드 실패 — ${MAKE_LOG} 확인")
          fi

          # ── IGNORE_PREEMPT_RT_PRESENCE 영구 설정 ──────────────────────
          # 향후 커널 업데이트 시 DKMS 자동 빌드에서도 RT 우회가 적용되도록
          ENV_FILE="/etc/environment"
          if ! grep -q "IGNORE_PREEMPT_RT_PRESENCE" "$ENV_FILE" 2>/dev/null; then
            echo "IGNORE_PREEMPT_RT_PRESENCE=1" >> "$ENV_FILE"
            info "IGNORE_PREEMPT_RT_PRESENCE=1 → ${ENV_FILE} (영구 설정)"
            CHANGES_APPLIED+=("IGNORE_PREEMPT_RT_PRESENCE=1 영구 설정: ${ENV_FILE}")
          fi
        fi

        # ── 모듈 로드 시도 ──────────────────────────────────────────────────
        # 빌드/설치 성공 후 또는 이미 설치된 경우 modprobe 시도
        if dkms status nvidia/"${NVIDIA_DKMS_VER}" -k "${KERNEL_VER}" 2>/dev/null | grep -q "installed"; then
          if ! lsmod | grep -q "^nvidia "; then
            info "NVIDIA 커널 모듈 로드 중..."
            if modprobe nvidia 2>/dev/null; then
              success "nvidia 모듈 로드 성공"
              if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
                success "nvidia-smi 정상 작동 확인"
              else
                warn "nvidia-smi 아직 작동하지 않음 — 재부팅 후 확인하세요"
                WARNINGS+=("NVIDIA 모듈 설치됨, 재부팅 필요할 수 있음")
              fi
            else
              warn "nvidia 모듈 로드 실패 — 재부팅 후 자동 로드됩니다"
              WARNINGS+=("nvidia modprobe 실패 — 재부팅 필요")
            fi
          else
            success "nvidia 모듈 이미 로드됨"
          fi
        fi
      fi
    fi
  fi
fi

echo ""

# ══════════════════════════════════════════════════════════════════════════════
# [7/7] Related Scripts + Validation & Summary
# ══════════════════════════════════════════════════════════════════════════════
# ── 지연된 시스템 작업 일괄 실행 ────────────────────────────────────────────
if [[ "$_NEED_DAEMON_RELOAD" -eq 1 ]]; then
  info "systemctl daemon-reload 실행 중..."
  timeout 30 systemctl daemon-reload || true
  success "daemon-reload 완료"
fi
if [[ "$_NEED_INITRAMFS_UPDATE" -eq 1 ]]; then
  info "update-initramfs -u 실행 중..."
  update-initramfs -u 2>/dev/null || true
  success "initramfs 업데이트 완료"
fi

# ── 연관 RT 스크립트 호출 (GRUB, 디스플레이, CPU governor) ──────────────────
echo ""
echo -e "${BOLD}━━━ 연관 RT 스크립트 호출 ━━━${NC}"
echo ""

if [[ -f "${SCRIPT_DIR_NVIDIA}/setup_grub_rt.sh" ]]; then
  info "GRUB RT 파라미터 설정 (setup_grub_rt.sh)..."
  bash "${SCRIPT_DIR_NVIDIA}/setup_grub_rt.sh" || {
    warn "GRUB RT setup failed — run manually: sudo ${SCRIPT_DIR_NVIDIA}/setup_grub_rt.sh"
    WARNINGS+=("setup_grub_rt.sh 실행 실패")
  }
else
  warn "setup_grub_rt.sh not found — skipping GRUB RT parameter setup"
fi

if [[ -f "${SCRIPT_DIR_NVIDIA}/setup_display_rt.sh" ]]; then
  info "디스플레이 최적화 (setup_display_rt.sh)..."
  bash "${SCRIPT_DIR_NVIDIA}/setup_display_rt.sh" || {
    warn "Display RT setup failed — run manually: sudo ${SCRIPT_DIR_NVIDIA}/setup_display_rt.sh"
    WARNINGS+=("setup_display_rt.sh 실행 실패")
  }
else
  warn "setup_display_rt.sh not found — skipping display optimization"
fi

if [[ -f "${SCRIPT_DIR_NVIDIA}/setup_cpu_governor.sh" ]]; then
  info "CPU governor 설정 (setup_cpu_governor.sh)..."
  bash "${SCRIPT_DIR_NVIDIA}/setup_cpu_governor.sh" || {
    warn "CPU governor setup failed — run manually: sudo ${SCRIPT_DIR_NVIDIA}/setup_cpu_governor.sh"
    WARNINGS+=("setup_cpu_governor.sh 실행 실패")
  }
else
  warn "setup_cpu_governor.sh not found — skipping CPU governor setup"
fi

echo ""
echo -e "${BOLD}━━━ [7/7] 설정 요약 ━━━${NC}"
echo ""

# ── Summary table ───────────────────────────────────────────────────────────
echo -e "${BOLD}┌─────────────────────────────────────────────────────────────────┐${NC}"
echo -e "${BOLD}│ 시스템 정보                                                    │${NC}"
echo -e "${BOLD}├─────────────────────────────────────────────────────────────────┤${NC}"
printf  "│ %-20s : %-40s │\n" "Ubuntu" "${UBUNTU_VERSION}"
printf  "│ %-20s : %-40s │\n" "Kernel" "${KERNEL_VER}"
printf  "│ %-20s : %-40s │\n" "NVIDIA GPU" "${NVIDIA_GPU_MODEL:0:40}"
printf  "│ %-20s : %-40s │\n" "Driver Version" "${NVIDIA_DRIVER_VER}"
printf  "│ %-20s : %-40s │\n" "Total CPU Cores" "${TOTAL_CORES}"
printf  "│ %-20s : %-40s │\n" "OS Cores" "${OS_CORES}"
printf  "│ %-20s : %-40s │\n" "RT Isolated Cores" "${RT_CORES}"
echo -e "${BOLD}├─────────────────────────────────────────────────────────────────┤${NC}"
echo -e "${BOLD}│ 적용된 변경사항                                                │${NC}"
echo -e "${BOLD}├─────────────────────────────────────────────────────────────────┤${NC}"

if [[ ${#CHANGES_APPLIED[@]} -eq 0 ]]; then
  echo    "│ (모든 설정이 이미 적용되어 있습니다)                            │"
else
  for change in "${CHANGES_APPLIED[@]}"; do
    printf "│ ${GREEN}✔${NC} %-60s │\n" "${change:0:60}"
  done
fi

echo -e "${BOLD}└─────────────────────────────────────────────────────────────────┘${NC}"

# ── Warnings ────────────────────────────────────────────────────────────────
if [[ ${#WARNINGS[@]} -gt 0 ]]; then
  echo ""
  echo -e "${YELLOW}${BOLD}경고사항:${NC}"
  for w in "${WARNINGS[@]}"; do
    echo -e "  ${YELLOW}⚠${NC} ${w}"
  done
fi

# ── Backup files ────────────────────────────────────────────────────────────
if [[ ${#BACKUP_FILES[@]} -gt 0 ]]; then
  echo ""
  echo -e "${BOLD}백업된 파일 (롤백 시 복원):${NC}"
  for bak in "${BACKUP_FILES[@]}"; do
    echo "  ${bak}"
  done
  echo ""
  echo -e "${BOLD}롤백 방법:${NC}"
  echo "  각 .bak 파일을 원본 경로로 복사한 뒤 재부팅하세요."
  echo "  예시:"
  if [[ ${#BACKUP_FILES[@]} -gt 0 ]]; then
    local_bak="${BACKUP_FILES[0]}"
    local_orig="${local_bak%.bak.*}"
    echo "    sudo cp ${local_bak} ${local_orig}"
    echo "    sudo update-grub && sudo update-initramfs -u && sudo reboot"
  fi
fi

# ── cyclictest command ──────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}재부팅 후 검증 (cyclictest):${NC}"
echo -e "${CYAN}"
cat << EOF
  sudo cyclictest --mlockall --smp --priority=80 \\
    --interval=2000 --distance=0 \\
    --cpu=${RT_CORES} \\
    --duration=60 --histogram=200 --quiet
EOF
echo -e "${NC}"
info "목표: Max latency < 200μs (500Hz 제어 루프 안정성)"

# ── Additional verification commands ────────────────────────────────────────
echo -e "${BOLD}기타 검증 명령:${NC}"
echo "  cat /sys/devices/system/cpu/isolated         # 격리된 CPU 확인"
echo "  cat /proc/interrupts | grep nvidia           # NVIDIA IRQ 확인"
echo "  nvidia-smi -q -d PERFORMANCE                 # persistence mode 확인"
echo "  dkms status                                   # DKMS 모듈 빌드 상태"
echo "  cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor  # CPU governor"
echo "  systemctl status nvidia-irq-affinity          # IRQ affinity 서비스"
echo "  systemctl status nvidia-persistence           # persistence 서비스"
echo ""

# ── Reboot prompt ───────────────────────────────────────────────────────────
if [[ ${#CHANGES_APPLIED[@]} -gt 0 ]]; then
  echo -e "${BOLD}${YELLOW}변경사항을 적용하려면 재부팅이 필요합니다.${NC}"
  echo ""
  read -rp "지금 재부팅하시겠습니까? [y/N]: " REBOOT_CONFIRM
  if [[ "${REBOOT_CONFIRM,,}" == "y" || "${REBOOT_CONFIRM,,}" == "yes" ]]; then
    info "3초 후 재부팅합니다..."
    sleep 3
    reboot
  else
    info "재부팅을 건너뜁니다. 수동으로 재부팅하세요: sudo reboot"
  fi
else
  success "모든 설정이 이미 적용되어 있습니다. 재부팅이 필요하지 않습니다."
fi

echo ""
