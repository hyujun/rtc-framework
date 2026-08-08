#!/bin/bash
# test_cpu_shield_mask.sh — cpu_shield.sh의 stale-shield 판정 로직 단위 테스트 (issue #349 D14).
#
# 검증 대상은 두 개다:
#   normalise_cpu_set()      "2-9,12-13" 같은 range 표기를 집합으로 정규화
#   shield_matches_desired() desired == actual == mode 일 때만 0
#
# 이 둘이 D14의 전부다 — 기존 do_on은 desired를 계산해 놓고 비교하지 않은 채
# mode 문자열만 보고 early-return 했고, 그 로그가 desired를 출력해서 축소되지
# 않은 shield를 축소됐다고 주장했다.
#
# cset은 이 저장소의 개발기에 설치돼 있지 않다. 그래서 실제 mask를 읽는
# shield_actual_user_cpus()는 여기서 stub으로 대체하고, 테스트는 순수 판정
# 로직만 구동한다. 읽기 계층이 실패할 때 "matches"가 아니라 "unknown"으로
# 떨어지는지도 함께 고정한다 — 읽을 수 없는 mask를 일치로 간주하는 것이 애초에
# stale shield를 만든 실패이기 때문이다.
#
# 실행: ./test_cpu_shield_mask.sh   (exit 0 = PASS)
set -eu -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="${SCRIPT_DIR}/../scripts"

# cpu_shield.sh는 소스되면 마지막의 case 디스패치를 실행한다. RTC_CPU_SHIELD_LIB_ONLY
# 대신 함수 정의부만 뽑아 쓰는 대신, 여기서는 필요한 함수만 재현 없이 얻기 위해
# 스크립트를 서브셸에서 source하고 COMMAND를 no-op으로 둔다.
_RT_LOG_PREFIX="test"
# shellcheck disable=SC1091
source "${SCRIPTS_DIR}/lib/rt_common.sh"

# cpu_shield.sh 본문에서 판정 함수만 추출해 정의한다 (dispatch 실행 회피).
# sed 범위는 함수 시그니처 기준이며, 함수명이 바뀌면 아래 assert가 즉시 깨진다.
#
# normalise_cpu_set / shield_isolation_method 는 여기서 뽑지 않는다 — #386 A 가
# 둘을 (isolcpus_isolated_cpus · shield_actual_user_cpus 와 함께) rt_common.sh
# 로 올렸고, 그건 위에서 이미 source 된다.
eval "$(sed -n '/^shield_matches_desired() {/,/^}/p' "${SCRIPTS_DIR}/cpu_shield.sh")"
# 마커 판독은 #350 에서 "<mode> <profile>" 로 바뀌었고 shield_matches_desired 가
# 그것을 호출한다 — 함께 뽑지 않으면 판정이 아니라 미정의 함수를 테스트하게 된다.
eval "$(sed -n '/^shield_marker_read() {/,/^}/p' "${SCRIPTS_DIR}/cpu_shield.sh")"
eval "$(sed -n '/^shield_marker_write() {/,/^}/p' "${SCRIPTS_DIR}/cpu_shield.sh")"
SHIELD_MODE_FILE="$(mktemp)"
trap 'rm -f "$SHIELD_MODE_FILE"' EXIT

# 가드를 두 목록으로 나눈 이유 — **하나로 두면 이동을 못 본다.** #386 이 위 네
# 함수를 rt_common.sh 로 옮길 때 이 가드가 red 를 낼 것으로 예상했으나, source
# 가 sed 추출보다 **먼저** 실행되므로 추출이 빈 문자열이어도 declare -F 는
# source 된 정의를 보고 통과했다 (실측). 즉 옛 단일 목록은 "cpu_shield.sh 에서
# 뽑았다" 를 주장하면서 실제로는 어느 파일에서 왔는지 구분하지 못한다.
for _fn in shield_matches_desired shield_marker_read shield_marker_write; do
  if ! declare -F "$_fn" >/dev/null; then
    echo "FAIL: cpu_shield.sh에서 ${_fn} 을 추출하지 못했다 (함수명 변경?)" >&2
    exit 1
  fi
done
for _fn in normalise_cpu_set shield_isolation_method shield_actual_user_cpus \
  isolcpus_isolated_cpus; do
  if ! declare -F "$_fn" >/dev/null; then
    echo "FAIL: rt_common.sh 가 ${_fn} 을 정의하지 않는다 (#386 A 의 이동이 되돌려졌나?)" >&2
    exit 1
  fi
done

PASS=0
FAIL=0
FAIL_MSGS=()

fail() { FAIL=$((FAIL+1)); FAIL_MSGS+=("$1"); }
pass() { PASS=$((PASS+1)); }

expect_eq() {
  local label="$1" expected="$2" actual="$3"
  if [[ "$expected" == "$actual" ]]; then pass; else
    fail "[$label] expected='$expected' actual='$actual'"
  fi
}

expect_rc() {
  # expect_rc "label" <expected_rc> <cmd...>
  local label="$1" want="$2"; shift 2
  local got=0
  "$@" >/dev/null 2>&1 || got=$?
  if [[ "$want" -eq "$got" ]]; then pass; else
    fail "[$label] expected rc=$want actual rc=$got"
  fi
}

# ── normalise_cpu_set ───────────────────────────────────────────────────────
expect_eq "range 확장"        "2 3 4 5 6 7 8 9"       "$(normalise_cpu_set '2-9')"
expect_eq "range+단일 혼합"   "2 3 4 5 6 7 8 9 12 13" "$(normalise_cpu_set '2-9,12-13')"
# 순서와 표기가 달라도 같은 집합이면 같아야 한다 — 이게 비교의 전제다.
expect_eq "순서 무관"         "$(normalise_cpu_set '2-9,12-13')" "$(normalise_cpu_set '12,13,2,3,4,5,6,7,8,9')"
expect_eq "중복 제거"         "1 2"                   "$(normalise_cpu_set '1,2,2,1')"
expect_eq "빈 입력"           ""                      "$(normalise_cpu_set '')"
# 멱등성: 출력 형식이 공백 구분이므로 자기 출력을 다시 먹여도 같아야 한다.
# 이 축이 없어서 "2 3 4 ..." 를 되먹이면 "234..." 라는 없는 cpu id 하나로
# 뭉개지는 것을 13케이스가 통과시켰다 (헤더 주석의 " 2 3 " 예시도 거짓이었다).
expect_eq "멱등"              "2 3 4 5 6 7 8 9 12 13" "$(normalise_cpu_set "$(normalise_cpu_set '2-9,12-13')")"
expect_eq "공백 구분 입력"    "2 3"                   "$(normalise_cpu_set ' 2 3 ')"
expect_eq "공백만"            ""                      "$(normalise_cpu_set '   ')"

# ── shield_matches_desired ──────────────────────────────────────────────────
# 읽기 계층을 stub으로 갈아끼워 판정만 구동한다.
STUB_ACTUAL=""
STUB_RC=0
shield_actual_user_cpus() {
  [[ "$STUB_RC" -ne 0 ]] && return "$STUB_RC"
  normalise_cpu_set "$STUB_ACTUAL"
}

echo "robot" >"$SHIELD_MODE_FILE"

STUB_ACTUAL="2-9,12-13"; STUB_RC=0
expect_rc "일치 → 0" 0 shield_matches_desired robot "2-9,12-13"
# 표기만 다른 같은 집합도 일치로 봐야 한다 (아니면 매 런치마다 불필요한 재구성).
expect_rc "표기 달라도 일치 → 0" 0 shield_matches_desired robot "12,13,2-9"

# D14의 본체: 레이아웃이 좁아졌는데 옛 넓은 shield가 살아있는 상태.
STUB_ACTUAL="2-9,12-13"
expect_rc "축소 필요 → 비0" 1 shield_matches_desired robot "2-5"
# 반대 방향(넓어짐)도 재구성 대상이다.
expect_rc "확대 필요 → 비0" 1 shield_matches_desired robot "2-9,12-15"

# 모드만 다르면 rc 2 — "코어는 맞고 마커만 틀렸다" 는 별개 상태다. do_on 은
# 여기서 reset 하지 않고 마커만 다시 쓴다. rc 1 (코어 불일치) 과 합쳐 두면
# 텍스트 파일 하나 때문에 adopt 된 프로세스가 전부 쫓겨나고, 로그는 같은
# 코어 목록을 "differs (X / X)" 로 두 번 출력한다.
STUB_ACTUAL="2-9,12-13"
expect_rc "모드만 불일치 → 2" 2 shield_matches_desired sim "2-9,12-13"
# 마커 파일 자체가 없는 경우 (tmp 정리 등) 도 같은 등급이다 — 코어는 맞다.
mv "$SHIELD_MODE_FILE" "${SHIELD_MODE_FILE}.bak"
expect_rc "마커 파일 부재 → 2" 2 shield_matches_desired robot "2-9,12-13"
mv "${SHIELD_MODE_FILE}.bak" "$SHIELD_MODE_FILE"
# 코어가 틀리면 마커가 맞아도 rc 1 이어야 한다 (등급이 뒤집히지 않는지).
expect_rc "코어 불일치 우선 → 1" 1 shield_matches_desired robot "2-5"

# fail-closed: 읽을 수 없으면 "일치"가 아니라 "모름" → 재구성.
STUB_RC=1
expect_rc "mask 읽기 실패 → 비0" 1 shield_matches_desired robot "2-9,12-13"
STUB_RC=0; STUB_ACTUAL=""
expect_rc "빈 mask → 비0" 1 shield_matches_desired robot "2-9,12-13"

# desired가 비면 판정 불가 — 조용히 통과시키면 shield 없이 RT가 돈다.
STUB_ACTUAL="2-9"
expect_rc "desired 비어있음 → 비0" 1 shield_matches_desired robot ""

# ── launch profile 축 (issue #350) ──────────────────────────────────────────
# 마커는 이제 "<mode> <profile>" 이다. 세 가지를 고정한다: (1) #350 이전에 쓰인
# bare 마커가 default profile 로 읽혀야 하고 (안 그러면 업그레이드 직후 모든
# 박스가 shield 를 한 번씩 재구성한다), (2) profile 만 달라도 rc 2 여야 하며,
# (3) profile 전환은 코어까지 달라지므로 rc 1 (재구성) 로 떨어진다.
STUB_RC=0; STUB_ACTUAL="2-9"
echo "robot" >"$SHIELD_MODE_FILE"   # #350 이전 형식
expect_rc "구형 마커 = default profile → 0" 0 shield_matches_desired robot "2-9" mpc_on
expect_eq "구형 마커 판독" "robot mpc_on" "$(shield_marker_read)"

shield_marker_write robot mpc_off
expect_eq "신형 마커 판독" "robot mpc_off" "$(shield_marker_read)"
# 코어는 우연히 같고 profile 만 다른 상태 — 마커만 갱신하면 되는 등급이다.
expect_rc "profile 만 불일치 → 2" 2 shield_matches_desired robot "2-9" mpc_on
# 실제 전환은 코어가 함께 바뀌므로 재구성 등급이어야 한다.
expect_rc "profile 전환(코어 축소) → 1" 1 shield_matches_desired robot "2-5" mpc_off
STUB_ACTUAL="2-5"
expect_rc "축소된 shield + off 마커 → 0" 0 shield_matches_desired robot "2-5" mpc_off
# profile 인자를 생략하면 default 로 떨어진다 (profile 을 모르는 기존 호출자).
shield_marker_write robot mpc_on
expect_rc "인자 생략 = default → 0" 0 shield_matches_desired robot "2-5"
STUB_ACTUAL="2-9"

# ── shield_isolation_method (실측 기반, 2026-08-07) ─────────────────────────
# dev PC 에 cset 을 설치하고 shield 를 세운 실측에서, do_status 가 살아 있는
# shield 를 "Isolated cores: none" 으로 보고했다. 격리 탐지 전체가
# /sys/devices/system/cpu/isolated 를 게이트로 삼았는데 **그 파일은 isolcpus 만
# 쓴다** — cset shield 는 아무것도 안 쓴다. 그래서 우선순위를 뒤집었고, 그
# 순서를 여기 박는다.
#
# 읽기 두 축을 모두 스텁으로 가린다. 이 박스의 실제 /sys/.../isolated 는 비어
# 있어서, 그걸 가리지 않으면 "둘 다 활성" 케이스를 만들 수 없고 우선순위가
# 검증되지 않은 채 통과한다 (실제로 그렇게 짰다가 순서를 뒤집는 mutation 이
# 통과했다).
STUB_ISOLCPUS=""
isolcpus_isolated_cpus() { echo "$STUB_ISOLCPUS"; }

# 둘 다 활성 — 동적 shield 가 이겨야 한다. **이 케이스가 우선순위의 유일한
# 증인이다**: 한쪽만 켜면 어느 순서로 물어도 같은 답이 나온다.
STUB_RC=0; STUB_ACTUAL="1-3,7-9"; STUB_ISOLCPUS="2-5"
expect_eq "isolation.둘 다면 cset 우선" "cset 1 2 3 7 8 9" "$(shield_isolation_method)"

# cset 단독
STUB_ISOLCPUS=""
expect_eq "isolation.cset 단독" "cset 1 2 3 7 8 9" "$(shield_isolation_method)"

# shield 를 못 읽을 때만 isolcpus 축으로 내려간다.
STUB_ACTUAL=""; STUB_RC=1; STUB_ISOLCPUS="2-5"
expect_eq "isolation.isolcpus fallback" "isolcpus 2-5" "$(shield_isolation_method)"

# 둘 다 없으면 none — 이게 "격리 없음" 의 유일한 정의여야 한다.
STUB_ISOLCPUS=""
expect_eq "isolation.none" "none" "$(shield_isolation_method)"

# 원상복구 (아래 결과 집계에 영향 없도록)
STUB_RC=0; STUB_ACTUAL="2-9"

# ── 결과 ────────────────────────────────────────────────────────────────────
echo "PASS=${PASS} FAIL=${FAIL}"
if [[ "$FAIL" -gt 0 ]]; then
  printf '  %s\n' "${FAIL_MSGS[@]}" >&2
  exit 1
fi
exit 0
