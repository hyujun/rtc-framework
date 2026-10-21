#!/bin/bash
# test_can_setup.sh — setup_can_interface.sh · setup_vcan.sh 의 계약 (issue #896, D32).
#
# 대상: 생성 스크립트 (/usr/local/bin/rtc-can-<iface>.sh · rtc-vcan-<iface>.sh) 에
# 박히는 `ip link set ... type can ...` 한 줄과 txqueuelen, oneshot 서비스 · udev
# rule 의 배선, 인터페이스 존재 · 종류에 따른 분기, 멱등성.
#
# 계약: 기본값은 W1 (1 M / 8 M, sample point 0.8 / 0.8, fd on) + D32 (restart-ms
# 100, txqueuelen 64) 이고 인자가 그대로 그 줄에 들어간다. one-shot · tdc 는 줄에
# 나타나지 않는다 (미지원 컨트롤러에서 EOPNOTSUPP). 서비스는 RemainAfterExit=no
# (udev 가 재연결마다 다시 start). vcan 은 없으면 만들고 MTU 가 72 가 아니면 고친다.
#
# 격리 방식: RTC_ETC_ROOT 로 /usr/local/bin · /etc/systemd/system · /etc/udev/rules.d
# 를 임시 디렉토리로 돌리고 (root 불필요), RTC_SYSFS_ROOT 로 가짜 /sys/class/net
# 을 준다. ip · modprobe · systemctl · udevadm · apt-get · candump 는 stub 이며
# 판정은 stub 이 받은 인자 로그로 한다. **실제 ip 가 그 인자를 받아들이는지는 이
# 파일이 보지 못한다** — 그것은 실기 (#896 게이트) 가 본다.
#
# 실행: ./test_can_setup.sh   (exit 0 = PASS)
# colcon test가 ament_add_test로 자동 실행한다.
set -eu -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS="${SCRIPT_DIR}/../scripts"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/assert.sh"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
STUB_BIN="$TMP/bin"
CMD_LOG="$TMP/cmd.log"
mkdir -p "$STUB_BIN"

# ── stubs ────────────────────────────────────────────────────────────────────
for tool in ip modprobe systemctl udevadm apt-get; do
  printf '#!/bin/bash\necho "%s $*" >>"%s"\n' "$tool" "$CMD_LOG" >"$STUB_BIN/$tool"
done
# candump present → apt-get must not be reached
printf '#!/bin/bash\nexit 0\n' >"$STUB_BIN/candump"
chmod +x "$STUB_BIN"/*

# ── fake sysfs ───────────────────────────────────────────────────────────────
sys_reset() { rm -rf "$TMP/sys"; mkdir -p "$TMP/sys/class/net"; }
# sys_add_iface NAME TYPE MTU FLAGS
sys_add_iface() {
  local d="$TMP/sys/class/net/$1"
  mkdir -p "$d"
  echo "$2" >"$d/type"
  echo "$3" >"$d/mtu"
  echo "$4" >"$d/flags"
}

run_script() {
  # run_script <script> [args...] → stdout+stderr in $OUT, exit code in $RC
  : >"$CMD_LOG"
  set +e
  OUT="$(env -i PATH="$STUB_BIN:/usr/bin:/bin" HOME="$TMP" RTC_ETC_ROOT="$TMP/etc" \
    RTC_SYSFS_ROOT="$TMP/sys" bash "$@" 2>&1)"
  RC=$?
  set -e
}

grep_count() { grep -c -- "$1" "$2" 2>/dev/null || true; }

# ═════════════════════════════════════════════════════════════════════════════
# setup_can_interface.sh
# ═════════════════════════════════════════════════════════════════════════════
CAN_SH="${SCRIPTS}/setup_can_interface.sh"
APPLY="$TMP/etc/usr/local/bin/rtc-can-can0.sh"
SERVICE="$TMP/etc/etc/systemd/system/rtc-can-can0.service"
RULE="$TMP/etc/etc/udev/rules.d/80-rtc-can-can0.rules"

# 1. defaults, interface present and of type CAN (ARPHRD_CAN = 280)
sys_reset; sys_add_iface can0 280 16 0x1002
run_script "$CAN_SH"
expect_eq "defaults: exit 0" 0 "$RC"
expect_eq "defaults: type can line" 1 "$(grep_count 'ip link set "$IFACE" type can bitrate 1000000 dbitrate 8000000 fd on sample-point 0.8 dsample-point 0.8 restart-ms 100' "$APPLY")"
expect_eq "defaults: txqueuelen 64" 1 "$(grep_count 'ip link set "$IFACE" txqueuelen 64' "$APPLY")"
expect_eq "defaults: no one-shot in the line" 0 "$(grep_count 'one-shot' "$APPLY")"
expect_eq "defaults: no tdc in the line" 0 "$(grep_count 'tdc' "$APPLY")"
expect_eq "defaults: apply script executable" "yes" "$([[ -x "$APPLY" ]] && echo yes || echo no)"
expect_eq "defaults: service ExecStart" 1 "$(grep_count "ExecStart=${APPLY}" "$SERVICE")"
expect_eq "defaults: service re-runs on hotplug" 1 "$(grep_count 'RemainAfterExit=no' "$SERVICE")"
expect_eq "defaults: udev rule wants the service" 1 "$(grep_count 'ENV{INTERFACE}=="can0", TAG+="systemd", ENV{SYSTEMD_WANTS}+="rtc-can-can0.service"' "$RULE")"
expect_eq "defaults: systemctl enable" 1 "$(grep_count 'systemctl enable rtc-can-can0.service' "$CMD_LOG")"
expect_eq "defaults: udevadm reload" 1 "$(grep_count 'udevadm control --reload' "$CMD_LOG")"
expect_eq "defaults: can-utils present → no apt" 0 "$(grep_count 'apt-get' "$CMD_LOG")"
# applied now, in order: down → txqueuelen → type can → up
expect_eq "defaults: applied now" "ip link set can0 down
ip link set can0 txqueuelen 64
ip link set can0 type can bitrate 1000000 dbitrate 8000000 fd on sample-point 0.8 dsample-point 0.8 restart-ms 100
ip link set can0 up" "$(grep '^ip link set can0' "$CMD_LOG")"

# 2. idempotent: same args again → files unchanged (no backup), still applied
run_script "$CAN_SH"
expect_eq "idempotent: exit 0" 0 "$RC"
expect_eq "idempotent: no .bak" 0 "$(find "$TMP/etc" -name '*.bak.*' | wc -l)"
expect_eq "idempotent: skipped messages" 3 "$(echo "$OUT" | grep -c 'already up-to-date')"
expect_eq "idempotent: applied again" 1 "$(grep_count 'ip link set can0 up' "$CMD_LOG")"

# 3. overrides land in the line, other interface name
sys_reset; sys_add_iface can1 280 16 0x1002
run_script "$CAN_SH" can1 --bitrate 500000 --dbitrate 2000000 --sample-point 0.75 --dsample-point 0.7 --txqueuelen 10 --restart-ms 0
expect_eq "override: exit 0" 0 "$RC"
APPLY1="$TMP/etc/usr/local/bin/rtc-can-can1.sh"
expect_eq "override: type can line" 1 "$(grep_count 'ip link set "$IFACE" type can bitrate 500000 dbitrate 2000000 fd on sample-point 0.75 dsample-point 0.7 restart-ms 0' "$APPLY1")"
expect_eq "override: txqueuelen 10" 1 "$(grep_count 'ip link set "$IFACE" txqueuelen 10' "$APPLY1")"
expect_eq "override: own service" "yes" "$([[ -f "$TMP/etc/etc/systemd/system/rtc-can-can1.service" ]] && echo yes || echo no)"

# 4. bad values are refused before anything is written
sys_reset
run_script "$CAN_SH" can2 --restart-ms abc
expect_eq "bad restart-ms: non-zero" 1 "$(( RC != 0 ))"
expect_eq "bad restart-ms: nothing written" "no" "$([[ -f "$TMP/etc/usr/local/bin/rtc-can-can2.sh" ]] && echo yes || echo no)"
run_script "$CAN_SH" can2 --sample-point 80
expect_eq "bad sample-point: non-zero" 1 "$(( RC != 0 ))"

# 5. interface absent: persistence installed, nothing applied, exit 0
sys_reset
run_script "$CAN_SH" can3
expect_eq "absent: exit 0" 0 "$RC"
expect_eq "absent: service written" "yes" "$([[ -f "$TMP/etc/etc/systemd/system/rtc-can-can3.service" ]] && echo yes || echo no)"
expect_eq "absent: no ip link set" 0 "$(grep_count 'ip link set' "$CMD_LOG")"
expect_eq "absent: warns" 1 "$(echo "$OUT" | grep -c 'not present')"

# 6. interface of another type (ethernet = 1) → fatal, not applied
sys_reset; sys_add_iface eth9 1 1500 0x1003
run_script "$CAN_SH" eth9
expect_eq "wrong type: non-zero" 1 "$(( RC != 0 ))"
expect_eq "wrong type: no ip link set" 0 "$(grep_count 'ip link set' "$CMD_LOG")"

# 7. --no-persist writes the apply script only
sys_reset; sys_add_iface can4 280 16 0x1002
run_script "$CAN_SH" can4 --no-persist
expect_eq "no-persist: exit 0" 0 "$RC"
expect_eq "no-persist: no service" "no" "$([[ -f "$TMP/etc/etc/systemd/system/rtc-can-can4.service" ]] && echo yes || echo no)"
expect_eq "no-persist: no udev rule" "no" "$([[ -f "$TMP/etc/etc/udev/rules.d/80-rtc-can-can4.rules" ]] && echo yes || echo no)"
expect_eq "no-persist: applied" 1 "$(grep_count 'ip link set can4 up' "$CMD_LOG")"

# 8. --remove drops service, apply script and udev rule
run_script "$CAN_SH" can0 --remove
expect_eq "remove: exit 0" 0 "$RC"
expect_eq "remove: files gone" "no" "$([[ -f "$SERVICE" || -f "$APPLY" || -f "$RULE" ]] && echo yes || echo no)"
expect_eq "remove: disabled" 1 "$(grep_count 'systemctl disable rtc-can-can0.service' "$CMD_LOG")"

# 9. can-utils absent → apt-get install
rm -f "$STUB_BIN/candump"
sys_reset
run_script "$CAN_SH" can5
expect_eq "no can-utils: apt-get install" 1 "$(grep_count 'apt-get install -y can-utils' "$CMD_LOG")"
printf '#!/bin/bash\nexit 0\n' >"$STUB_BIN/candump"; chmod +x "$STUB_BIN/candump"

# ═════════════════════════════════════════════════════════════════════════════
# setup_vcan.sh
# ═════════════════════════════════════════════════════════════════════════════
VCAN_SH="${SCRIPTS}/setup_vcan.sh"
VAPPLY="$TMP/etc/usr/local/bin/rtc-vcan-vcan0.sh"
VSERVICE="$TMP/etc/etc/systemd/system/rtc-vcan-vcan0.service"

# 10. absent: modprobe, add, mtu 72, up; persistence written
sys_reset
run_script "$VCAN_SH"
expect_eq "vcan absent: exit 0" 0 "$RC"
expect_eq "vcan absent: commands" "modprobe vcan
ip link add dev vcan0 type vcan
ip link set vcan0 down
ip link set vcan0 mtu 72
ip link set vcan0 up" "$(grep -E '^(modprobe|ip link)' "$CMD_LOG")"
expect_eq "vcan absent: apply script has mtu 72" 1 "$(grep_count 'ip link set "$IFACE" mtu 72' "$VAPPLY")"
expect_eq "vcan absent: service ExecStart" 1 "$(grep_count "ExecStart=${VAPPLY}" "$VSERVICE")"
expect_eq "vcan absent: enabled" 1 "$(grep_count 'systemctl enable rtc-vcan-vcan0.service' "$CMD_LOG")"

# 11. present at mtu 72 and up → nothing touched
sys_reset; sys_add_iface vcan0 280 72 0x1001
run_script "$VCAN_SH"
expect_eq "vcan optimal: exit 0" 0 "$RC"
expect_eq "vcan optimal: no ip link" 0 "$(grep_count 'ip link' "$CMD_LOG")"
expect_eq "vcan optimal: no modprobe" 0 "$(grep_count 'modprobe' "$CMD_LOG")"

# 12. present at classic mtu 16 → down, mtu 72, up (no add)
sys_reset; sys_add_iface vcan0 280 16 0x1001
run_script "$VCAN_SH"
expect_eq "vcan mtu16: commands" "ip link set vcan0 down
ip link set vcan0 mtu 72
ip link set vcan0 up" "$(grep -E '^(modprobe|ip link)' "$CMD_LOG")"

# 13. present, mtu 72, down → up only
sys_reset; sys_add_iface vcan0 280 72 0x1000
run_script "$VCAN_SH"
expect_eq "vcan down: commands" "ip link set vcan0 up" "$(grep -E '^(modprobe|ip link)' "$CMD_LOG")"

# 14. --no-persist on another name
sys_reset
run_script "$VCAN_SH" vcan1 --no-persist
expect_eq "vcan1 no-persist: exit 0" 0 "$RC"
expect_eq "vcan1 no-persist: added" 1 "$(grep_count 'ip link add dev vcan1 type vcan' "$CMD_LOG")"
expect_eq "vcan1 no-persist: no service" "no" "$([[ -f "$TMP/etc/etc/systemd/system/rtc-vcan-vcan1.service" ]] && echo yes || echo no)"

# 15. --remove deletes the interface and the service
sys_reset; sys_add_iface vcan0 280 72 0x1001
run_script "$VCAN_SH" --remove
expect_eq "vcan remove: exit 0" 0 "$RC"
expect_eq "vcan remove: link del" 1 "$(grep_count 'ip link del dev vcan0' "$CMD_LOG")"
expect_eq "vcan remove: service gone" "no" "$([[ -f "$VSERVICE" || -f "$VAPPLY" ]] && echo yes || echo no)"

summary_and_exit "test_can_setup"
