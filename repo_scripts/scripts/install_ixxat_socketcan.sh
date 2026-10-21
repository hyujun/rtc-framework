#!/bin/bash
# install_ixxat_socketcan.sh — IXXAT USB-to-CAN FD SocketCAN 드라이버 (ix_usb_can) out-of-tree 빌드 · 설치
#
# hms-networks/ixxat-socketcan-usb (vendor 08db; 메인라인 미머지) 를 실행 중인
# 커널의 헤더로 빌드하고 /lib/modules/<커널>/kernel/drivers/net/can/usb/ixxat_usb/
# 에 설치한다 (DKMS 아님 — 커널을 바꾸면 다시 돌린다). 빌드는 일반 사용자로,
# 설치만 root 로 한다: root 로 make 를 돌리면 kbuild 산출물이 root 소유가 되어
# 다음 사용자 빌드가 깨진다.
#
# 전제: 커널 헤더 (/lib/modules/$(uname -r)/build, 없으면 --kernel-src), git, make,
# gcc. Secure Boot 가 켜져 있으면 서명 없는 모듈은 로드되지 않는다 (MOK — 이
# 저장소는 enroll_lttng_mok.sh 가 같은 문제를 다룬다). 드라이버 2.1.11+ 는 어댑터
# 펌웨어 < 1.6.0 을 거부한다 — dmesg 가 알려 주고, 펌웨어 갱신은 Windows 의
# IXXAT 도구로 한다.
#
# Usage:
#   ./install_ixxat_socketcan.sh build [--ref v2.1.15] [--kernel-src DIR]   # sudo 불필요
#   sudo ./install_ixxat_socketcan.sh install                               # .ko 설치 + depmod + modprobe
#   ./install_ixxat_socketcan.sh status                                     # 모듈 · dmesg · CAN 인터페이스
#
# 소스 위치: $RTC_IXXAT_SRC (기본 <rtc_ws>/deps/src/ixxat-socketcan-usb — deps/ 는
# COLCON_IGNORE 가 있어 colcon 이 보지 않는다).
#
# 다음 단계: 어댑터 연결 → `status` 로 can0 확인 → sudo setup_can_interface.sh
# (bitrate · fd on · systemd 재적용).

set -euo pipefail

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR}/lib/rt_common.sh"
make_logger "IXXAT"

# 위치: src/rtc-framework/repo_scripts/scripts/ → 4 up = <rtc_ws> (build_deps.sh 와 같은 셈)
WS="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
readonly DRV="ix_usb_can"
readonly REPO_URL="https://github.com/hms-networks/ixxat-socketcan-usb.git"
readonly DEFAULT_REF="v2.1.15"
readonly MOD_SUBDIR="kernel/drivers/net/can/usb/ixxat_usb"
SRC="${RTC_IXXAT_SRC:-${WS}/deps/src/ixxat-socketcan-usb}"
KO="${SRC}/${MOD_SUBDIR}/${DRV}.ko"
KVER="$(uname -r)"
INSTALL_DIR="/lib/modules/${KVER}/${MOD_SUBDIR}"

# `lsmod | grep -q` 가 아니다: set -o pipefail 아래서 grep -q 가 먼저 닫으면 lsmod 가
# SIGPIPE 로 죽어 로드된 모듈을 "없다" 고 한다 (PC#1 에서 실제로 났다).
module_loaded() { [[ -d "/sys/module/${DRV}" ]]; }

usage() {
  echo ""
  echo "Usage:"
  echo "  install_ixxat_socketcan.sh build [--ref TAG] [--kernel-src DIR]"
  echo "  sudo install_ixxat_socketcan.sh install"
  echo "  install_ixxat_socketcan.sh status"
  echo ""
  echo "  --ref TAG          git tag / commit (기본 ${DEFAULT_REF})"
  echo "  --kernel-src DIR   커널 빌드 트리 (기본 /lib/modules/\$(uname -r)/build)"
  echo "  RTC_IXXAT_SRC      소스 디렉토리 (기본 ${WS}/deps/src/ixxat-socketcan-usb)"
  echo ""
  exit 0
}

# ── build ───────────────────────────────────────────────────────────────────
do_build() {
  local ref="$DEFAULT_REF"
  local kernel_src="/lib/modules/${KVER}/build"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --ref)        [[ $# -ge 2 ]] || fatal "--ref requires a value"; ref="$2"; shift ;;
      --kernel-src) [[ $# -ge 2 ]] || fatal "--kernel-src requires a value"; kernel_src="$2"; shift ;;
      *)            fatal "Unknown build option: $1" ;;
    esac
    shift
  done
  [[ "$EUID" -ne 0 ]] || warn "building as root — the source tree becomes root-owned"

  for tool in git make gcc; do
    command -v "$tool" &>/dev/null || fatal "${tool} not found (sudo apt-get install -y git build-essential)"
  done
  if [[ ! -f "${kernel_src}/Makefile" ]]; then
    fatal "kernel build tree not found: ${kernel_src} — install linux-headers-${KVER} or pass --kernel-src <dir>"
  fi

  section "Step 1: source (${ref}) → ${SRC}"
  if [[ ! -d "${SRC}/.git" ]]; then
    mkdir -p "$(dirname "$SRC")"
    git clone --quiet "$REPO_URL" "$SRC"
    info "  cloned"
  else
    git -C "$SRC" fetch --quiet --tags origin
    info "  fetched"
  fi
  git -C "$SRC" checkout --quiet "$ref"
  info "  at $(git -C "$SRC" describe --tags --always)"

  section "Step 2: make all (KERNEL_SRC=${kernel_src})"
  # v2.1.15 는 <linux/unaligned.h> 를 쓰는데 그 헤더는 6.12 에서 <asm/unaligned.h> 를
  # 옮긴 것이다 (PC#2 6.1.112 에 없다). 소스를 고치지 않고 shim 디렉토리를 include
  # 경로 앞에 둔다 — 커널 트리에 헤더가 있으면 그쪽이 먼저 잡혀 shim 은 놀고 있다.
  local -a extra=()
  if [[ ! -f "${kernel_src}/include/linux/unaligned.h" ]]; then
    local shim="${SRC}/.rtc-compat/include"
    mkdir -p "${shim}/linux"
    printf '/* shim by install_ixxat_socketcan.sh: kernels before 6.12 keep this header under asm/ */\n#include <asm/unaligned.h>\n' \
      >"${shim}/linux/unaligned.h"
    extra=("KCPPFLAGS=-I${shim}")
    info "  kernel has no include/linux/unaligned.h — using shim ${shim}"
  fi
  # 다른 커널로 빌드한 산출물이 남아 있으면 vermagic 이 어긋난 .ko 를 설치하게 된다 → clean
  make -C "$SRC" clean KERNEL_SRC="$kernel_src" >/dev/null 2>&1 || true
  if ! make -C "$SRC" all KERNEL_SRC="$kernel_src" "${extra[@]}"; then
    fatal "build failed — paste the output into the issue (#896 asks for the result on each PC)"
  fi
  [[ -f "$KO" ]] || fatal "build reported success but ${KO} is missing"

  echo ""
  success "built ${KO}"
  modinfo "$KO" | grep -E '^(version|vermagic|srcversion):' | sed 's/^/  /' || true
  info "next: sudo ${SCRIPT_DIR}/install_ixxat_socketcan.sh install"
}

# ── install ─────────────────────────────────────────────────────────────────
do_install() {
  require_root install
  [[ -f "$KO" ]] || fatal "${KO} not built — run: $0 build"

  local vermagic
  vermagic="$(modinfo -F vermagic "$KO" | awk '{print $1}')"
  if [[ "$vermagic" != "$KVER" ]]; then
    fatal "${KO} was built for ${vermagic}, running kernel is ${KVER} — run: $0 build"
  fi

  section "Step 1: install ${DRV}.ko → ${INSTALL_DIR}"
  if module_loaded; then
    warn "  ${DRV} is loaded — unloading (CAN interfaces go away until modprobe)"
    rmmod "$DRV"
  fi
  install -d "$INSTALL_DIR"
  install -m 644 "$KO" "${INSTALL_DIR}/${DRV}.ko"
  depmod -a
  info "  installed, depmod done"

  # updates/ (DKMS) 가 kernel/ 보다 우선한다 — 옛 DKMS 사본이 있으면 지금 설치한 것은 그늘에 든다
  local resolved
  resolved="$(modinfo -n "$DRV" 2>/dev/null || true)"
  if [[ "$resolved" != "${INSTALL_DIR}/${DRV}.ko" ]]; then
    fatal "modprobe would load '${resolved:-<none>}', not the file just installed — remove the other copy (dkms status ${DRV}; dkms remove ${DRV}/<ver> --all) and re-run"
  fi

  section "Step 2: modprobe ${DRV}"
  modprobe "$DRV"
  info "  loaded"
  echo ""
  do_status
}

# ── status ──────────────────────────────────────────────────────────────────
do_status() {
  section "ix_usb_can status (kernel ${KVER})"
  if module_loaded; then
    success "module loaded: $(modinfo -F version "$DRV" 2>/dev/null || echo '?') ($(modinfo -n "$DRV" 2>/dev/null || echo '?'))"
  else
    warn "module not loaded (sudo modprobe ${DRV}; not installed → $0 install)"
  fi
  if command -v lsusb &>/dev/null; then
    local usb
    usb="$(lsusb -d 08db: 2>/dev/null || true)"
    if [[ -n "$usb" ]]; then
      info "adapter: ${usb}"
    else
      warn "no IXXAT adapter (USB vendor 08db) on the bus — plug it in"
    fi
  fi
  local dm
  dm="$(dmesg 2>/dev/null | grep -i -E 'ixxat|ix_usb_can' | tail -8 || true)"
  if [[ -n "$dm" ]]; then
    info "dmesg (last lines; firmware < 1.6.0 is refused here):"
    echo "$dm" | sed 's/^/  /'
  else
    info "dmesg: nothing from ixxat (unreadable without sudo on this host?) — sudo dmesg | grep -i ixxat"
  fi
  local links
  links="$(ip -details link show type can 2>/dev/null || true)"
  if [[ -n "$links" ]]; then
    info "CAN interfaces:"
    echo "$links" | sed 's/^/  /'
    info "next: sudo ${SCRIPT_DIR}/setup_can_interface.sh   # bitrate · fd on · systemd"
  else
    warn "no CAN interface yet (ip -details link show type can)"
  fi
}

# ── dispatch ────────────────────────────────────────────────────────────────
CMD="${1:-}"
[[ -n "$CMD" ]] || usage
shift
case "$CMD" in
  -h|--help|help) usage ;;
  build)   do_build "$@" ;;
  install) do_install "$@" ;;
  status)  do_status "$@" ;;
  *)       fatal "Unknown command: ${CMD} (build | install | status)" ;;
esac
