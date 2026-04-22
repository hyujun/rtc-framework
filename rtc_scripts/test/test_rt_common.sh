#!/bin/bash
# test_rt_common.sh — rt_common.sh의 hybrid CPU 감지 로직 단위 테스트.
#
# 임시 디렉토리에 가짜 sysfs 트리를 만들고 $RTC_SYSFS_ROOT / $RTC_PROC_CPUINFO
# override로 detect_hybrid_capability가 C++ 측 DetectCpuTopology와 동일한
# 결과를 반환하는지 검증한다.
#
# 실행: ./test_rt_common.sh   (exit 0 = PASS)
# colcon test가 ament_add_test로 자동 실행한다.
set -eu -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/../scripts/lib"

# Source with faux _rt_log setup so rt_common.sh doesn't require a TTY.
_RT_LOG_PREFIX="test"
# shellcheck disable=SC1091
source "${LIB_DIR}/rt_common.sh"

PASS=0
FAIL=0
FAIL_MSGS=()

fail() { FAIL=$((FAIL+1)); FAIL_MSGS+=("$1"); }
pass() { PASS=$((PASS+1)); }

expect_eq() {
  # expect_eq "label" expected actual
  local label="$1" expected="$2" actual="$3"
  if [[ "$expected" == "$actual" ]]; then
    pass
  else
    fail "[$label] expected='$expected' actual='$actual'"
  fi
}

# ── Mock sysfs builder ──────────────────────────────────────────────────────
# Creates $ROOT/devices/system/cpu/cpuN/topology/{physical_package_id,core_id}
# and cpufreq/cpuinfo_max_freq, plus types/{intel_core,intel_atom}/cpus.

mock_reset() {
  local root="$1"
  rm -rf -- "$root"
  mkdir -p -- "$root/devices/system/cpu"
}

mock_add_cpu() {
  # $1=root $2=cpu $3=core_id $4=max_freq_khz
  local root="$1" cpu="$2" core_id="$3" max_freq="${4:-0}"
  local d="$root/devices/system/cpu/cpu${cpu}"
  mkdir -p "$d/topology" "$d/cpufreq"
  echo "0" >"$d/topology/physical_package_id"
  echo "$core_id" >"$d/topology/core_id"
  if (( max_freq > 0 )); then
    echo "$max_freq" >"$d/cpufreq/cpuinfo_max_freq"
  fi
}

mock_set_types() {
  # $1=root $2=p_cpus_csv $3=e_cpus_csv (optional)
  local root="$1" p_csv="$2" e_csv="${3:-}"
  if [[ -n "$p_csv" ]]; then
    mkdir -p "$root/devices/system/cpu/types/intel_core"
    echo -n "$p_csv" >"$root/devices/system/cpu/types/intel_core/cpus"
  fi
  if [[ -n "$e_csv" ]]; then
    mkdir -p "$root/devices/system/cpu/types/intel_atom"
    echo -n "$e_csv" >"$root/devices/system/cpu/types/intel_atom/cpus"
  fi
}

mock_write_cpuinfo() {
  # $1=path $2="true"|"false" for hybrid flag
  local p="$1" has_hybrid="$2"
  local flag=""
  if [[ "$has_hybrid" == "true" ]]; then flag=" hybrid"; fi
  {
    echo "processor	: 0"
    echo "vendor_id	: GenuineIntel"
    echo "flags		: fpu vme de pse tsc msr${flag} pae"
  } >"$p"
}

TMP="$(mktemp -d)"
trap 'rm -rf -- "$TMP"' EXIT

# ── Test 1: NUC 13 Pro i7-1360P (4P + 8E) ──────────────────────────────────
test_nuc13_i7_1360p() {
  local root="$TMP/nuc13"
  mock_reset "$root"
  local i
  # P-cores: core_id 0..3, each with SMT sibling.
  for i in 0 1 2 3; do
    mock_add_cpu "$root" $((i*2))   "$i" 5000000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5000000
  done
  # E-cores: core_id 4..11, cpus 8..15, uniform freq (no LP-E).
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((8+i)) $((4+i)) 3800000
  done
  mock_set_types "$root" "0,1,2,3,4,5,6,7" "8,9,10,11,12,13,14,15"
  mock_write_cpuinfo "$TMP/nuc13_cpuinfo" "true"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/nuc13_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" detect_hybrid_capability

  expect_eq "NUC13.is_hybrid"         "1"            "$IS_HYBRID"
  expect_eq "NUC13.p_core_has_smt"    "1"            "$P_CORE_HAS_SMT"
  expect_eq "NUC13.has_lp_e_cores"    "0"            "$HAS_LP_E_CORES"
  expect_eq "NUC13.num_p_physical"    "4"            "$NUM_P_PHYSICAL"
  expect_eq "NUC13.num_p_logical"     "8"            "$NUM_P_LOGICAL"
  expect_eq "NUC13.num_e_cores"       "8"            "$NUM_E_CORES"
  expect_eq "NUC13.num_lpe_cores"     "0"            "$NUM_LPE_CORES"
  expect_eq "NUC13.generation"        "raptor_lake_p" "$NUC_GENERATION"
  expect_eq "NUC13.p_physical_ids"    "0 2 4 6"      "$P_CORE_PHYSICAL_IDS"
  expect_eq "NUC13.p_sibling_ids"     "1 3 5 7"      "$P_CORE_SIBLING_IDS"
}

# ── Test 2: BIOS HT off on a Raptor-Lake-class chip ────────────────────────
test_bios_ht_off() {
  local root="$TMP/htoff"
  mock_reset "$root"
  local i
  # Only one logical CPU per P-core (sibling disabled).
  for i in 0 1 2 3; do
    mock_add_cpu "$root" "$i" "$i" 5000000
  done
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((4+i)) $((4+i)) 3800000
  done
  mock_set_types "$root" "0,1,2,3" "4,5,6,7,8,9,10,11"
  mock_write_cpuinfo "$TMP/htoff_cpuinfo" "true"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/htoff_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" detect_hybrid_capability

  expect_eq "HTOff.is_hybrid"         "1"                      "$IS_HYBRID"
  expect_eq "HTOff.p_core_has_smt"    "0"                      "$P_CORE_HAS_SMT"
  expect_eq "HTOff.generation"        "raptor_lake_p_ht_off"   "$NUC_GENERATION"
  expect_eq "HTOff.num_p_physical"    "4"                      "$NUM_P_PHYSICAL"
  expect_eq "HTOff.num_p_logical"     "4"                      "$NUM_P_LOGICAL"
}

# ── Test 3: Container — no sysfs types/ dir, no hybrid flag ────────────────
test_container_fallback() {
  local root="$TMP/container"
  mock_reset "$root"
  # Only topology files, no types/intel_core, no hybrid flag.
  local i
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" "$i" "$i" 3000000
  done
  mock_write_cpuinfo "$TMP/container_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/container_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" detect_hybrid_capability

  expect_eq "Container.is_hybrid"     "0"     "$IS_HYBRID"
  expect_eq "Container.generation"    "none"  "$NUC_GENERATION"
  expect_eq "Container.num_p_physical" "0"    "$NUM_P_PHYSICAL"
}

# ── Test 4: AMD Ryzen — no hybrid anywhere ─────────────────────────────────
test_amd_ryzen() {
  local root="$TMP/amd"
  mock_reset "$root"
  # 8C/16T with SMT.
  local i
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((i*2))   "$i" 5400000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5400000
  done
  mock_write_cpuinfo "$TMP/amd_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/amd_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" detect_hybrid_capability

  expect_eq "AMD.is_hybrid"          "0"    "$IS_HYBRID"
  expect_eq "AMD.generation"         "none" "$NUC_GENERATION"
  expect_eq "AMD.p_core_physical"    ""     "$P_CORE_PHYSICAL_IDS"
}

# ── Test 5: Env hint override (generation only, sysfs still says AMD) ──────
test_env_hint_override() {
  local root="$TMP/hint"
  mock_reset "$root"
  local i
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" "$i" "$i" 3000000
  done
  mock_write_cpuinfo "$TMP/hint_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/hint_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="raptor_lake_p" detect_hybrid_capability

  expect_eq "Hint.generation"        "raptor_lake_p"  "$NUC_GENERATION"
  # sysfs-derived fields stay untouched.
  expect_eq "Hint.is_hybrid"         "0"              "$IS_HYBRID"
  expect_eq "Hint.p_core_physical"   ""               "$P_CORE_PHYSICAL_IDS"
}

# ── Test 6: cpulist range parser ────────────────────────────────────────────
test_cpulist_parser() {
  local out
  out=$(_rt_parse_cpulist "0-3,8,10-11")
  expect_eq "Parse.0-3_8_10-11"  "0 1 2 3 8 10 11"  "$out"

  out=$(_rt_parse_cpulist "0")
  expect_eq "Parse.single"       "0"                "$out"

  out=$(_rt_parse_cpulist "")
  expect_eq "Parse.empty"        ""                 "$out"
}

# ── Run all ─────────────────────────────────────────────────────────────────
test_cpulist_parser
test_nuc13_i7_1360p
test_bios_ht_off
test_container_fallback
test_amd_ryzen
test_env_hint_override

echo
echo "── test_rt_common.sh summary ──"
echo "  PASS: $PASS"
echo "  FAIL: $FAIL"
if (( FAIL > 0 )); then
  printf '  %s\n' "${FAIL_MSGS[@]}"
  exit 1
fi
exit 0
