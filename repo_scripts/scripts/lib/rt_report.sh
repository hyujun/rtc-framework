#!/bin/bash
# rt_report.sh — RT 검증 스크립트 공통 리포트 인프라.
#
# check_rt_setup.sh (정적 RT 환경 점검) 와 verify_rt_runtime.sh (런타임 스레드
# 검증) 가 동일하게 사용하는 PASS/WARN/FAIL/SKIP 카운터, 카테고리 트래커,
# 출력 포맷(verbose/summary/json) 의 골격을 단일 위치에 모아둔다. 각 스크립트는
# 카테고리 ID·라벨·여분 metadata(BENCHMARK 분기, threads 블록 등) 만 자체적으로
# 채우고, 본 lib 가 공통 행/요약/JSON 블록 출력을 담당한다.
#
# 호출 측 계약 (전역):
#   OUTPUT_MODE      — "verbose" | "summary" | "json" (기본 verbose)
#   PASS_COUNT / WARN_COUNT / FAIL_COUNT / SKIP_COUNT — 정수 카운터
#   CATEGORY_STATUS — associative array (cat → "PASS"/"WARN"/"FAIL")
#   CATEGORY_DETAIL — associative array (cat → 한 줄 detail 문자열)
#   color globals   — rt_common.sh 가 setup_colors() 로 채워두는 변수
#
# 호출 측은 본 lib 를 source 하기 *전에* rt_common.sh 를 source 해야 한다.

# 이 파일은 직접 실행하지 않는다.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

# ── 중복 source 방지 ───────────────────────────────────────────────────────
[[ -n "${_RT_REPORT_LOADED:-}" ]] && return 0
_RT_REPORT_LOADED=1

# ── Counter / category state 초기화 헬퍼 ────────────────────────────────────
# 호출 측이 PASS_COUNT=0 등을 손수 선언할 필요가 없도록 lib 로 제공.
# 호출 측에서 declare -A CATEGORY_STATUS / CATEGORY_DETAIL 은 이 함수 호출 후
# 이미 declare 되어 있을 수 있으므로 -g 옵션을 써서 caller scope 에 만든다.
init_report_state() {
  PASS_COUNT=0
  WARN_COUNT=0
  FAIL_COUNT=0
  SKIP_COUNT=0
  declare -gA CATEGORY_STATUS
  declare -gA CATEGORY_DETAIL
  CATEGORY_STATUS=()
  CATEGORY_DETAIL=()
}

# ── Output helpers (verbose 모드에서만 한 줄 표기) ─────────────────────────
_pass() {
  ((PASS_COUNT++)) || true
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "  ${GREEN}[PASS]${NC} $*"
  fi
}

_warn() {
  ((WARN_COUNT++)) || true
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "  ${YELLOW}[WARN]${NC} $*"
  fi
}

_fail() {
  ((FAIL_COUNT++)) || true
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "  ${RED}[FAIL]${NC} $*"
  fi
}

_skip() {
  ((SKIP_COUNT++)) || true
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo -e "  ${DIM}[SKIP]${NC} $*"
  fi
}

_section() {
  if [[ "$OUTPUT_MODE" == "verbose" ]]; then
    echo ""
    echo -e "${BOLD}[$1] $2${NC}"
  fi
}

# ── Category tracking (최악 상태 유지) ──────────────────────────────────────
_category_start() {
  CATEGORY_STATUS["$1"]="PASS"
  CATEGORY_DETAIL["$1"]=""
}

_category_update() {
  local cat="$1" status="$2"
  local current="${CATEGORY_STATUS[$cat]}"
  # FAIL > WARN > PASS
  if [[ "$status" == "FAIL" ]]; then
    CATEGORY_STATUS["$cat"]="FAIL"
  elif [[ "$status" == "WARN" && "$current" != "FAIL" ]]; then
    CATEGORY_STATUS["$cat"]="WARN"
  fi
}

_category_set_detail() {
  CATEGORY_DETAIL["$1"]="$2"
}

# ── Summary mode: 카테고리 1줄 출력 ────────────────────────────────────────
# 인자: <category_key> <display_label> [<label_width> (기본 16)]
print_category_row() {
  local cat="$1" label="$2" width="${3:-16}"
  local status="${CATEGORY_STATUS[$cat]:-SKIP}"
  local detail="${CATEGORY_DETAIL[$cat]:-}"

  local color="$GREEN"
  local icon="PASS"
  case "$status" in
    WARN) color="$YELLOW"; icon="WARN" ;;
    FAIL) color="$RED"; icon="FAIL" ;;
    SKIP) color="$DIM"; icon="SKIP" ;;
  esac

  echo -e "  ${color}[${icon}]${NC} $(printf "%-${width}s" "$label") ${DIM}${detail}${NC}"
}

# ── Verbose / summary 공통: Result 마지막 줄 ───────────────────────────────
# OUTPUT_MODE == json 일 때는 호출 측이 본 함수를 부르지 않도록.
print_result_line() {
  echo ""
  echo -ne "  ${BOLD}Result:${NC} "
  [[ "$PASS_COUNT" -gt 0 ]] && echo -ne "${GREEN}${PASS_COUNT} pass${NC}  "
  [[ "$WARN_COUNT" -gt 0 ]] && echo -ne "${YELLOW}${WARN_COUNT} warn${NC}  "
  [[ "$FAIL_COUNT" -gt 0 ]] && echo -ne "${RED}${FAIL_COUNT} fail${NC}  "
  [[ "$SKIP_COUNT" -gt 0 ]] && echo -ne "${DIM}${SKIP_COUNT} skip${NC}"
  echo ""
}

# ── JSON: categories 블록 ────────────────────────────────────────────────────
# 인자: <category_key>...
# 출력: 들여쓰기된 카테고리별 JSON 항목 (마지막 항목 뒤에는 콤마 없음).
# 호출 측은 본 블록 앞뒤로 "{... \"categories\": {" / "}," 을 추가한다.
emit_json_categories() {
  local first=1
  local cat status detail
  for cat in "$@"; do
    status="${CATEGORY_STATUS[$cat]:-SKIP}"
    detail="${CATEGORY_DETAIL[$cat]:-}"
    # JSON 특수문자 이스케이프
    detail=$(echo "$detail" | sed 's/\\/\\\\/g; s/"/\\"/g')

    [[ "$first" -eq 0 ]] && echo ","
    printf "    \"%s\": {\"status\": \"%s\", \"detail\": \"%s\"}" "$cat" "$status" "$detail"
    first=0
  done
  echo ""
}

# ── JSON: summary 한 줄 ─────────────────────────────────────────────────────
# 호출 측이 marker (",") 결정 후 본 함수를 호출.
emit_json_summary() {
  printf "  \"summary\": {\"pass\": %s, \"warn\": %s, \"fail\": %s, \"skip\": %s}" \
    "$PASS_COUNT" "$WARN_COUNT" "$FAIL_COUNT" "$SKIP_COUNT"
  echo ""
}
