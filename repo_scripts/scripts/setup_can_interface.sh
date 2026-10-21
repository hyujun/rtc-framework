#!/bin/bash
# setup_can_interface.sh — 실기 CAN FD 인터페이스 (can0) 설정 + systemd 재적용 + can-utils
#
# IXXAT USB-to-CAN FD (ix_usb_can, install_ixxat_socketcan.sh) 가 만든 can0 를
# proto2 손 (udp_hand_driver `protocol_version: 2`) 의 버스 규격으로 올린다:
#   nominal 1 Mbit/s · data 8 Mbit/s (BRS) · sample point 80 / 80 % · fd on.
# TDC 는 data BRP 가 1 · 2 이면 커널이 auto 로 켠다 (드라이버 2.1.15+) — 명시하지
# 않는다. one-shot 도 명시하지 않는다: ctrlmode 에 넣으면 그 플래그를 모르는
# 컨트롤러에서 EOPNOTSUPP 가 나고, 기본값이 off 다 (검증 출력에서 ONE-SHOT 이
# 없어야 한다).
#
# 기본값은 epic #895 D32 의 제안값이고 실측 (H1-F06) 이 고친다: restart-ms 100,
# txqueuelen 64. 인자로 모두 바꿀 수 있다.
#
# 재적용: USB 어댑터는 재연결 · 재부팅마다 새 can0 로 나타나므로 (bitrate 없음,
# down) setup_udp_optimization.sh Step 5 패턴대로 생성 스크립트
# (/usr/local/bin/rtc-can-<iface>.sh) + oneshot 서비스 (rtc-can-<iface>.service)
# 를 둔다. 서비스는 부팅 시 (multi-user) 와 udev 가 인터페이스를 더할 때마다
# (SYSTEMD_WANTS, RemainAfterExit=no) 돈다.
#
# Usage:
#   sudo ./setup_can_interface.sh                              # can0, D32 기본값
#   sudo ./setup_can_interface.sh can1 --txqueuelen 10 --restart-ms 0
#   sudo ./setup_can_interface.sh --bitrate 500000 --dbitrate 2000000
#   sudo ./setup_can_interface.sh --remove                     # 서비스 · udev rule 제거
#
# 검증:
#   ip -details link show can0        # "can <FD> state ERROR-ACTIVE", bitrate 1000000 / dbitrate 8000000,
#                                     # restart-ms 100, tdc-mode auto; ctrlmode 에 ONE-SHOT 없음
#   candump can0                      # 손 프레임 (게이트: #896)
#   systemctl status rtc-can-can0     # 마지막 적용 결과
#
# 테스트 훅: RTC_ETC_ROOT=<dir> 이면 /usr/local/bin · /etc/systemd/system ·
# /etc/udev/rules.d 를 그 아래에 쓰고 root 를 요구하지 않는다
# (test/test_can_setup.sh — ip · systemctl · udevadm · apt-get 은 stub).
# RTC_SYSFS_ROOT (기본 /sys) 로 인터페이스 존재 · 종류를 읽는다.

set -euo pipefail

# ── 공통 라이브러리 로드 ────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/rt_common.sh
source "${SCRIPT_DIR}/lib/rt_common.sh"
make_logger "CAN-IF"

readonly ARPHRD_CAN=280

usage() {
  echo ""
  echo "Usage: sudo setup_can_interface.sh [IFACE] [OPTIONS]"
  echo ""
  echo "  IFACE                 인터페이스 이름 (기본 can0)"
  echo "  --bitrate N           nominal bitrate [bit/s]   (기본 1000000)"
  echo "  --dbitrate N          data bitrate [bit/s]      (기본 8000000, BRS)"
  echo "  --sample-point X      nominal sample point      (기본 0.8)"
  echo "  --dsample-point X     data sample point         (기본 0.8)"
  echo "  --txqueuelen N        송신 큐 길이              (기본 64, D32)"
  echo "  --restart-ms N        bus-off 자동 복구 [ms], 0 = off (기본 100, D32)"
  echo "  --wait N              생성 스크립트가 인터페이스를 기다리는 시간 [s] (기본 20)"
  echo "  --no-persist          systemd 서비스 · udev rule 을 만들지 않는다"
  echo "  --remove              서비스 · udev rule 을 제거한다 (인터페이스는 그대로)"
  echo "  -h, --help            도움말"
  echo ""
  echo "can-utils (candump · cansend) 가 없으면 apt 로 설치한다."
  echo ""
  exit 0
}

# ── 인자 파싱 ───────────────────────────────────────────────────────────────
IFACE="can0"
BITRATE=1000000
DBITRATE=8000000
SAMPLE_POINT=0.8
DSAMPLE_POINT=0.8
TXQUEUELEN=64
RESTART_MS=100
WAIT_S=20
PERSIST=1
REMOVE=0

need_value() { [[ $# -ge 2 && "$2" != -* ]] || fatal "$1 requires a value (see --help)"; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help|help)  usage ;;
    --bitrate)       need_value "$@"; BITRATE="$2"; shift ;;
    --dbitrate)      need_value "$@"; DBITRATE="$2"; shift ;;
    --sample-point)  need_value "$@"; SAMPLE_POINT="$2"; shift ;;
    --dsample-point) need_value "$@"; DSAMPLE_POINT="$2"; shift ;;
    --txqueuelen)    need_value "$@"; TXQUEUELEN="$2"; shift ;;
    --restart-ms)    need_value "$@"; RESTART_MS="$2"; shift ;;
    --wait)          need_value "$@"; WAIT_S="$2"; shift ;;
    --no-persist)    PERSIST=0 ;;
    --remove)        REMOVE=1 ;;
    -*)              fatal "Unknown option: $1 (see --help)" ;;
    *)               IFACE="$1" ;;
  esac
  shift
done
for v in BITRATE DBITRATE TXQUEUELEN RESTART_MS WAIT_S; do
  [[ "${!v}" =~ ^[0-9]+$ ]] || fatal "${v,,} must be a non-negative integer: ${!v}"
done
for v in SAMPLE_POINT DSAMPLE_POINT; do
  [[ "${!v}" =~ ^0\.[0-9]+$ ]] || fatal "${v,,} must be a fraction like 0.8: ${!v}"
done

ETC_ROOT="${RTC_ETC_ROOT:-}"
[[ -n "$ETC_ROOT" ]] || require_root
SYS_ROOT="${RTC_SYSFS_ROOT:-/sys}"
BIN_DIR="${ETC_ROOT}/usr/local/bin"
SYSTEMD_DIR="${ETC_ROOT}/etc/systemd/system"
UDEV_DIR="${ETC_ROOT}/etc/udev/rules.d"
APPLY_SCRIPT="${BIN_DIR}/rtc-can-${IFACE}.sh"
SERVICE="rtc-can-${IFACE}.service"
SERVICE_PATH="${SYSTEMD_DIR}/${SERVICE}"
UDEV_RULE="${UDEV_DIR}/80-rtc-can-${IFACE}.rules"

iface_present() { [[ -d "${SYS_ROOT}/class/net/${IFACE}" ]]; }
iface_type()    { cat "${SYS_ROOT}/class/net/${IFACE}/type" 2>/dev/null || echo 0; }

# ── --remove ────────────────────────────────────────────────────────────────
if [[ "$REMOVE" -eq 1 ]]; then
  section "Removing ${SERVICE} and udev rule..."
  if [[ -f "$SERVICE_PATH" ]]; then
    systemctl disable "$SERVICE" 2>/dev/null || true
    rm -f "$SERVICE_PATH" "$APPLY_SCRIPT"
    systemctl daemon-reload
    info "  ${SERVICE} removed"
  else
    info "  ${SERVICE} not installed — skipped"
  fi
  if [[ -f "$UDEV_RULE" ]]; then
    rm -f "$UDEV_RULE"
    udevadm control --reload 2>/dev/null || true
    info "  ${UDEV_RULE} removed"
  fi
  exit 0
fi

# ── 1. can-utils ────────────────────────────────────────────────────────────
section "Step 1: can-utils (candump · cansend)..."
if command -v candump &>/dev/null; then
  info "  can-utils already installed — skipped"
elif DEBIAN_FRONTEND=noninteractive apt-get install -y can-utils >/dev/null 2>&1; then
  info "  can-utils installed"
else
  warn "  apt-get install can-utils failed — install by hand: sudo apt-get install -y can-utils"
fi

# ── 2. 생성 스크립트 (설정의 단일 출처 — 지금 적용과 재적용이 같은 파일을 쓴다) ──
section "Step 2: apply script (${APPLY_SCRIPT})..."
install -d "$BIN_DIR"

APPLY_CONTENT="#!/bin/bash
# Bring up ${IFACE} as CAN FD for the proto2 hand (udp_hand_driver protocol_version 2)
# Generated by setup_can_interface.sh — do not edit manually; re-run the script to change values
set -e
IFACE=\"${IFACE}\"
WAIT_S=${WAIT_S}
for ((i = 0; i < WAIT_S * 10; i++)); do
  ip link show \"\$IFACE\" &>/dev/null && break
  sleep 0.1
done
ip link show \"\$IFACE\" &>/dev/null || { echo \"rtc-can: \${IFACE} not present after \${WAIT_S}s (adapter unplugged? ix_usb_can not loaded?)\" >&2; exit 1; }
ip link set \"\$IFACE\" down
ip link set \"\$IFACE\" txqueuelen ${TXQUEUELEN}
ip link set \"\$IFACE\" type can bitrate ${BITRATE} dbitrate ${DBITRATE} fd on sample-point ${SAMPLE_POINT} dsample-point ${DSAMPLE_POINT} restart-ms ${RESTART_MS}
ip link set \"\$IFACE\" up"

if write_file_if_changed "$APPLY_SCRIPT" "$APPLY_CONTENT" false; then
  chmod +x "$APPLY_SCRIPT"
  info "  Script written (bitrate ${BITRATE} / dbitrate ${DBITRATE}, sample-point ${SAMPLE_POINT} / ${DSAMPLE_POINT}, txqueuelen ${TXQUEUELEN}, restart-ms ${RESTART_MS})"
else
  info "  ${APPLY_SCRIPT} already up-to-date — skipped"
fi

# ── 3. systemd oneshot + udev (재부팅 · 재연결 재적용) ───────────────────────
if [[ "$PERSIST" -eq 1 ]]; then
  section "Step 3: systemd persistence (${SERVICE}) + udev hotplug..."
  install -d "$SYSTEMD_DIR" "$UDEV_DIR"

  # RemainAfterExit=no: 어댑터를 다시 꽂으면 udev 가 같은 서비스를 다시 start 할 수 있다.
  SERVICE_CONTENT="[Unit]
Description=Bring up ${IFACE} as CAN FD (RTC proto2 hand)
After=network-pre.target

[Service]
Type=oneshot
ExecStart=${APPLY_SCRIPT}
RemainAfterExit=no

[Install]
WantedBy=multi-user.target"

  if write_file_if_changed "$SERVICE_PATH" "$SERVICE_CONTENT" false; then
    info "  Service written to ${SERVICE_PATH}"
  else
    info "  ${SERVICE_PATH} already up-to-date — skipped"
  fi

  UDEV_CONTENT="# Re-apply the CAN FD settings whenever ${IFACE} appears (USB adapter plugged in)
# Generated by setup_can_interface.sh — do not edit manually
ACTION==\"add\", SUBSYSTEM==\"net\", ENV{INTERFACE}==\"${IFACE}\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}+=\"${SERVICE}\""

  if write_file_if_changed "$UDEV_RULE" "$UDEV_CONTENT" false; then
    info "  udev rule written to ${UDEV_RULE}"
  else
    info "  ${UDEV_RULE} already up-to-date — skipped"
  fi

  systemctl daemon-reload
  systemctl enable "$SERVICE" 2>/dev/null || true
  udevadm control --reload 2>/dev/null || true
  info "  ${SERVICE} enabled (boot + hotplug)"
fi

# ── 4. 지금 적용 ────────────────────────────────────────────────────────────
section "Step 4: apply now..."
if ! iface_present; then
  warn "  ${IFACE} not present — nothing applied now. Plug the adapter in (udev starts ${SERVICE})"
  warn "  or load the driver: sudo modprobe ix_usb_can  (install_ixxat_socketcan.sh)"
  exit 0
fi
if [[ "$(iface_type)" -ne "$ARPHRD_CAN" ]]; then
  fatal "${IFACE} is not a CAN interface (ARPHRD type $(iface_type), expected ${ARPHRD_CAN})"
fi
"$APPLY_SCRIPT"
info "  applied"

echo ""
DETAILS="$(ip -details link show "$IFACE" 2>/dev/null || true)"
[[ -n "$DETAILS" ]] && echo "$DETAILS"
if [[ "$DETAILS" == *ONE-SHOT* ]]; then
  warn "ONE-SHOT is set on ${IFACE} — D32 expects it off (host frames must be retried behind echoes)."
  warn "  sudo ip link set ${IFACE} down && sudo ip link set ${IFACE} type can one-shot off && sudo ip link set ${IFACE} up"
fi
echo ""
info "${IFACE} configured."
info "  candump ${IFACE}                   # hand frames (gate)"
info "  systemctl status ${SERVICE}"
