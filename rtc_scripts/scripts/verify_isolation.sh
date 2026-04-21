#!/usr/bin/env bash
# verify_isolation.sh — RPATH + ldd 격리 검증.
#
# 워크스페이스 install/ 의 모든 공유 라이브러리와 실행 바이너리가 격리된
# 경로만 (ROS / deps/install / workspace / libc / 승인된 /opt/* drop) 에서
# 의존성을 해결하는지 검사.

# NOTE: setup_env.sh 가 source 하는 ROS / colcon scripts 에 unbound var 가 있어서
# set -u 는 사용하지 않음. set -e 만 사용.
set -e

# 위치: src/rtc-framework/rtc_scripts/scripts/verify_isolation.sh → 4 up = <rtc_ws>
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${_SCRIPT_DIR}/../../../.." && pwd)"
# shellcheck source=/dev/null
source "${_SCRIPT_DIR}/setup_env.sh"

RED='\033[0;31m'; GREEN='\033[0;32m'; NC='\033[0m'

echo "═══ RPATH + ldd 격리 검증 (${WS}) ═══"
echo ""

_is_leak_line() {
  # ldd 한 줄이 격리 외부 경로를 가리키면 0, 아니면 1 반환
  local line="$1"
  case "$line" in
    *" => /opt/ros/jazzy/"*)            return 1 ;;
    *" => ${WS}/deps/install/"*)         return 1 ;;
    *" => ${WS}/install/"*)              return 1 ;;
    *" => /lib/x86_64-linux-gnu/"*)      return 1 ;;
    *" => /lib64/"*)                     return 1 ;;
    *" => /usr/lib/x86_64-linux-gnu/"*)  return 1 ;;
    *" => /opt/onnxruntime"*)            return 1 ;;
    *" => /opt/mujoco-"*)                return 1 ;;
    *"linux-vdso"*|*"ld-linux"*)         return 1 ;;
    *" => "*) return 0 ;;  # 다른 => 매치는 누수
    *)        return 1 ;;  # => 가 없는 줄 (not a dynamic 등)
  esac
}

fail=0
audit_count=0
for target in \
    "${WS}"/install/*/lib/*.so \
    "${WS}"/install/*/lib/*.so.* \
    "${WS}"/install/*/lib/*/* ; do
  [[ -f "$target" ]] || continue
  file -L "$target" 2>/dev/null | grep -q 'ELF' || continue
  audit_count=$((audit_count + 1))

  # ldd 출력을 라인별 평가
  leaks=""
  while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    if _is_leak_line "$line"; then
      leaks="${leaks}${line}
"
    fi
  done < <(ldd "$target" 2>/dev/null)

  if [[ -n "$leaks" ]]; then
    echo -e "${RED}✘ LEAK${NC}: $target"
    echo "$leaks" | sed 's/^/     /'
    fail=1
  fi
done

echo ""
echo "audit count: $audit_count ELF objects"
if [[ "$fail" -eq 0 ]]; then
  echo -e "${GREEN}✓ 모든 ELF 가 격리된 경로로만 resolve${NC}"
fi

echo ""
echo "=== 대표 바이너리 — ur5e_rt_controller ==="
rt_bin="${WS}/install/ur5e_bringup/lib/ur5e_bringup/ur5e_rt_controller"
if [[ -f "$rt_bin" ]]; then
  ldd "$rt_bin" 2>/dev/null | grep -E 'aligator|fmt|mimalloc|pinocchio|hpp-fcl|proxsuite|mujoco' | sed 's/^/  /'
else
  echo "  (not built yet)"
fi

exit "$fail"
