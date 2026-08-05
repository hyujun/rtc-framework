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

# cpu_shield.sh 본문에서 판정 함수 3개만 추출해 정의한다 (dispatch 실행 회피).
# sed 범위는 함수 시그니처 기준이며, 함수명이 바뀌면 아래 assert가 즉시 깨진다.
eval "$(sed -n '/^normalise_cpu_set() {/,/^}/p' "${SCRIPTS_DIR}/cpu_shield.sh")"
eval "$(sed -n '/^shield_matches_desired() {/,/^}/p' "${SCRIPTS_DIR}/cpu_shield.sh")"
SHIELD_MODE_FILE="$(mktemp)"
trap 'rm -f "$SHIELD_MODE_FILE"' EXIT

if ! declare -F normalise_cpu_set >/dev/null || ! declare -F shield_matches_desired >/dev/null; then
  echo "FAIL: cpu_shield.sh에서 판정 함수를 추출하지 못했다 (함수명 변경?)" >&2
  exit 1
fi

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

# ── 결과 ────────────────────────────────────────────────────────────────────
echo "PASS=${PASS} FAIL=${FAIL}"
if [[ "$FAIL" -gt 0 ]]; then
  printf '  %s\n' "${FAIL_MSGS[@]}" >&2
  exit 1
fi
exit 0
