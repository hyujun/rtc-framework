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
  # Optional: $3=cpu_family (int), $4=cpu_model (int)
  # When family/model are omitted, the corresponding lines are skipped —
  # tests that exercise the "unknown silicon" path (e.g. AMD, container,
  # legacy mocks) rely on this absence.
  local p="$1" has_hybrid="$2" family="${3:-}" model="${4:-}"
  local flag=""
  if [[ "$has_hybrid" == "true" ]]; then flag=" hybrid"; fi
  {
    echo "processor	: 0"
    echo "vendor_id	: GenuineIntel"
    if [[ -n "$family" ]]; then echo "cpu family	: ${family}"; fi
    if [[ -n "$model"  ]]; then echo "model		: ${model}"; fi
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

# ── Test 7: freq-clustering fallback — Meteor Lake (NUC 14 Pro, 185H) ──────
# sysfs types/ 미노출 + cpuinfo hybrid flag 없음 (custom RT 커널 시나리오).
# cpuinfo_max_freq 만으로 6P + 8E + 2LP-E 를 재구성해야 한다.
test_freq_fallback_meteor_lake() {
  local root="$TMP/mtl_freq"
  mock_reset "$root"
  local i
  # 6 P-cores, each with SMT sibling. core_id 0..5, cpus 0..11 at 5.1 GHz.
  for i in 0 1 2 3 4 5; do
    mock_add_cpu "$root" $((i*2))   "$i" 5100000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5100000
  done
  # 8 E-cores, no SMT, core_id 6..13, cpus 12..19 at 3.8 GHz.
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((12+i)) $((6+i)) 3800000
  done
  # 2 LP E-cores, core_id 14..15, cpus 20..21 at 2.5 GHz (under 70% of E-max).
  mock_add_cpu "$root" 20 14 2500000
  mock_add_cpu "$root" 21 15 2500000
  # Intentionally skip mock_set_types AND hybrid flag — simulates the NUC14SRK
  # case where the kernel suppresses both signals.
  mock_write_cpuinfo "$TMP/mtl_freq_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/mtl_freq_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "MTLFreq.is_hybrid"        "1"                 "$IS_HYBRID"
  expect_eq "MTLFreq.detect_source"    "cpufreq_cluster"   "$HYBRID_DETECT_SOURCE"
  expect_eq "MTLFreq.p_core_has_smt"   "1"                 "$P_CORE_HAS_SMT"
  expect_eq "MTLFreq.has_lp_e_cores"   "1"                 "$HAS_LP_E_CORES"
  expect_eq "MTLFreq.num_p_physical"   "6"                 "$NUM_P_PHYSICAL"
  expect_eq "MTLFreq.num_p_logical"    "12"                "$NUM_P_LOGICAL"
  expect_eq "MTLFreq.num_e_cores"      "8"                 "$NUM_E_CORES"
  expect_eq "MTLFreq.num_lpe_cores"    "2"                 "$NUM_LPE_CORES"
  expect_eq "MTLFreq.generation"       "meteor_lake"       "$NUC_GENERATION"
  expect_eq "MTLFreq.p_physical_ids"   "0 2 4 6 8 10"      "$P_CORE_PHYSICAL_IDS"
  expect_eq "MTLFreq.p_sibling_ids"    "1 3 5 7 9 11"      "$P_CORE_SIBLING_IDS"
  expect_eq "MTLFreq.lpe_ids"          "20 21"             "$LPE_CORE_IDS"
}

# ── Test 8: freq-clustering regression guard — homogeneous AMD still false ─
# AMD Ryzen has identical max_freq across all cores; freq fallback must NOT
# produce a false-positive hybrid classification.
test_freq_fallback_amd_negative() {
  local root="$TMP/amd_freq"
  mock_reset "$root"
  local i
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((i*2))   "$i" 5400000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5400000
  done
  mock_write_cpuinfo "$TMP/amd_freq_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/amd_freq_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "AMDFreq.is_hybrid"        "0"       "$IS_HYBRID"
  expect_eq "AMDFreq.detect_source"    "none"    "$HYBRID_DETECT_SOURCE"
  expect_eq "AMDFreq.generation"       "none"    "$NUC_GENERATION"
}

# ── Test 9: detect_source tag is "sysfs_types" when primary path wins ──────
# Reuses the NUC13 mock (sysfs + hybrid flag) to pin down the reported source.
test_detect_source_primary() {
  local root="$TMP/src_primary"
  mock_reset "$root"
  local i
  for i in 0 1 2 3; do
    mock_add_cpu "$root" $((i*2))   "$i" 5000000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5000000
  done
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((8+i)) $((4+i)) 3800000
  done
  mock_set_types "$root" "0,1,2,3,4,5,6,7" "8,9,10,11,12,13,14,15"
  mock_write_cpuinfo "$TMP/src_primary_cpuinfo" "true"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/src_primary_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "SrcPrimary.detect_source" "sysfs_types"  "$HYBRID_DETECT_SOURCE"
  expect_eq "SrcPrimary.is_hybrid"     "1"            "$IS_HYBRID"
}

# ── Test 10: container fallback remains conservative when freq unavailable ─
# Container mock adds CPUs WITHOUT cpufreq/cpuinfo_max_freq files. The freq
# fallback must return "cannot form opinion" rather than force a split.
test_container_no_freq_files() {
  local root="$TMP/container_nofreq"
  mock_reset "$root"
  local i
  # mock_add_cpu with max_freq=0 → does not create cpufreq/cpuinfo_max_freq.
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" "$i" "$i" 0
  done
  mock_write_cpuinfo "$TMP/container_nofreq_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/container_nofreq_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "NoFreq.is_hybrid"       "0"     "$IS_HYBRID"
  expect_eq "NoFreq.detect_source"   "none"  "$HYBRID_DETECT_SOURCE"
}

# ── Test 11: CPUID-based platform label — i9-13900K (Raptor Lake-S) ────────
# Raptor Lake-S desktop shares the (P-HT, no LP-E) fingerprint with NUC13
# Pro's Raptor Lake-P mobile. CPUID model 0xBF (191) must distinguish them
# in PLATFORM_LABEL even though NUC_GENERATION stays raptor_lake_p.
test_platform_label_raptor_lake_s_desktop() {
  local root="$TMP/rls_desktop"
  mock_reset "$root"
  local i
  # 8 P-cores with SMT siblings, 16 E-cores — i9-13900K layout.
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((i*2))   "$i" 5800000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5800000
  done
  for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
    mock_add_cpu "$root" $((16+i)) $((8+i)) 4300000
  done
  mock_set_types "$root" "0-15" "16-31"
  mock_write_cpuinfo "$TMP/rls_desktop_cpuinfo" "true" "6" "191"  # 0xBF

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/rls_desktop_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "RLS.is_hybrid"           "1"                          "$IS_HYBRID"
  expect_eq "RLS.generation"          "raptor_lake_p"              "$NUC_GENERATION"
  expect_eq "RLS.cpu_vendor"          "GenuineIntel"               "$CPU_VENDOR"
  expect_eq "RLS.cpu_family"          "6"                          "$CPU_FAMILY"
  expect_eq "RLS.cpu_model"           "191"                        "$CPU_MODEL"
  expect_eq "RLS.platform_label"      "Raptor Lake-S desktop"      "$PLATFORM_LABEL"
  expect_eq "RLS.no_ht_by_design"     "0"                          "$PLATFORM_NO_HT_BY_DESIGN"
  expect_eq "RLS.num_p_physical"      "8"                          "$NUM_P_PHYSICAL"
  expect_eq "RLS.num_e_cores"         "16"                         "$NUM_E_CORES"
}

# ── Test 12: CPUID-based platform label — NUC 13 Pro (Raptor Lake-P) ───────
# Same topology fingerprint as Test 11 but CPUID model 0xBA (186) distinguishes.
test_platform_label_raptor_lake_p_mobile() {
  local root="$TMP/rlp_mobile"
  mock_reset "$root"
  local i
  for i in 0 1 2 3; do
    mock_add_cpu "$root" $((i*2))   "$i" 5000000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5000000
  done
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((8+i)) $((4+i)) 3800000
  done
  mock_set_types "$root" "0,1,2,3,4,5,6,7" "8,9,10,11,12,13,14,15"
  mock_write_cpuinfo "$TMP/rlp_mobile_cpuinfo" "true" "6" "186"  # 0xBA

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/rlp_mobile_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "RLP.generation"          "raptor_lake_p"              "$NUC_GENERATION"
  expect_eq "RLP.cpu_model"           "186"                        "$CPU_MODEL"
  expect_eq "RLP.platform_label"      "Raptor Lake-P mobile"       "$PLATFORM_LABEL"
  expect_eq "RLP.no_ht_by_design"     "0"                          "$PLATFORM_NO_HT_BY_DESIGN"
}

# ── Test 13: Arrow Lake-S desktop (no HT by silicon design) ────────────────
# Core Ultra 9 285K: 8 P-cores (Lion Cove, no HT) + 16 E-cores (Skymont),
# no LP-E. Fingerprint (no P-HT, no LP-E) collides with raptor_lake_p_ht_off
# but CPUID model 0xC6 (198) must flag platform_no_ht_by_design=1 so the
# verifier PASSes instead of suggesting an impossible BIOS toggle.
test_platform_label_arrow_lake_s_desktop() {
  local root="$TMP/als_desktop"
  mock_reset "$root"
  local i
  # 8 P-cores, no SMT siblings (Lion Cove has no HT).
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" "$i" "$i" 5700000
  done
  # 16 E-cores, no SMT, uniform freq (no LP-E on desktop).
  for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
    mock_add_cpu "$root" $((8+i)) $((8+i)) 4600000
  done
  mock_set_types "$root" "0-7" "8-23"
  mock_write_cpuinfo "$TMP/als_desktop_cpuinfo" "true" "6" "198"  # 0xC6

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/als_desktop_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "ALS.is_hybrid"           "1"                          "$IS_HYBRID"
  expect_eq "ALS.p_core_has_smt"      "0"                          "$P_CORE_HAS_SMT"
  expect_eq "ALS.has_lp_e_cores"      "0"                          "$HAS_LP_E_CORES"
  expect_eq "ALS.generation"          "raptor_lake_p_ht_off"       "$NUC_GENERATION"
  expect_eq "ALS.cpu_model"           "198"                        "$CPU_MODEL"
  expect_eq "ALS.platform_label"      "Arrow Lake-S desktop"       "$PLATFORM_LABEL"
  expect_eq "ALS.no_ht_by_design"     "1"                          "$PLATFORM_NO_HT_BY_DESIGN"
  expect_eq "ALS.num_p_physical"      "8"                          "$NUM_P_PHYSICAL"
  expect_eq "ALS.num_e_cores"         "16"                         "$NUM_E_CORES"
}

# ── Test 14: unknown silicon (e.g. future CPU) — empty label, no crash ─────
test_platform_label_unknown_model() {
  local root="$TMP/unknown"
  mock_reset "$root"
  local i
  for i in 0 1 2 3; do
    mock_add_cpu "$root" "$i" "$i" 3000000
  done
  mock_write_cpuinfo "$TMP/unknown_cpuinfo" "false" "6" "255"  # not in table

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/unknown_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "Unknown.cpu_family"      "6"                          "$CPU_FAMILY"
  expect_eq "Unknown.cpu_model"       "255"                        "$CPU_MODEL"
  expect_eq "Unknown.platform_label"  ""                           "$PLATFORM_LABEL"
  expect_eq "Unknown.no_ht_by_design" "0"                          "$PLATFORM_NO_HT_BY_DESIGN"
}

# ── Test 15: AMD vendor — no platform label, family/model still surfaced ───
test_platform_label_amd_vendor() {
  local root="$TMP/amd_id"
  mock_reset "$root"
  local i
  for i in 0 1 2 3; do
    mock_add_cpu "$root" "$i" "$i" 4800000
  done
  # AMD reports family 25 (Zen 3/4) — non-Intel, so lookup returns empty.
  {
    echo "processor	: 0"
    echo "vendor_id	: AuthenticAMD"
    echo "cpu family	: 25"
    echo "model		: 33"
    echo "flags		: fpu vme de pse tsc msr pae"
  } >"$TMP/amd_id_cpuinfo"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/amd_id_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "AMD.cpu_vendor"          "AuthenticAMD"               "$CPU_VENDOR"
  expect_eq "AMD.cpu_family"          "25"                         "$CPU_FAMILY"
  expect_eq "AMD.cpu_model"           "33"                         "$CPU_MODEL"
  expect_eq "AMD.platform_label"      ""                           "$PLATFORM_LABEL"
  expect_eq "AMD.no_ht_by_design"     "0"                          "$PLATFORM_NO_HT_BY_DESIGN"
}

# ── Test 16: missing /proc/cpuinfo fields — graceful defaults ──────────────
# Legacy mocks omit cpu family / model entirely; existing tests rely on this.
# Verify we get safe zero/empty defaults rather than crashing.
test_platform_label_missing_fields() {
  local root="$TMP/no_fm"
  mock_reset "$root"
  local i
  for i in 0 1 2 3; do
    mock_add_cpu "$root" "$i" "$i" 3000000
  done
  # No family / model arguments — mock_write_cpuinfo skips both lines.
  mock_write_cpuinfo "$TMP/no_fm_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/no_fm_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "NoFM.cpu_vendor"         "GenuineIntel"               "$CPU_VENDOR"
  expect_eq "NoFM.cpu_family"         "0"                          "$CPU_FAMILY"
  expect_eq "NoFM.cpu_model"          "0"                          "$CPU_MODEL"
  expect_eq "NoFM.platform_label"     ""                           "$PLATFORM_LABEL"
}

# ── Test 17: PHYSICAL_CORE_SLOTS — NUC13 (4P+8E HT on) ─────────────────────
# P-physical (0,2,4,6) followed by E-cores (8..15). SMT siblings excluded.
test_physical_core_slots_nuc13_hybrid() {
  local root="$TMP/pcs_nuc13"
  mock_reset "$root"
  local i
  for i in 0 1 2 3; do
    mock_add_cpu "$root" $((i*2))   "$i" 5000000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5000000
  done
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((8+i)) $((4+i)) 3800000
  done
  mock_set_types "$root" "0,1,2,3,4,5,6,7" "8,9,10,11,12,13,14,15"
  mock_write_cpuinfo "$TMP/pcs_nuc13_cpuinfo" "true" "6" "186"  # 0xBA

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/pcs_nuc13_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "PCS.NUC13.slots" "0 2 4 6 8 9 10 11 12 13 14 15" "$PHYSICAL_CORE_SLOTS"
}

# ── Test 18: PHYSICAL_CORE_SLOTS — i9-13900K (8P+16E HT on) ────────────────
test_physical_core_slots_13900k_hybrid() {
  local root="$TMP/pcs_13900k"
  mock_reset "$root"
  local i
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((i*2))   "$i" 5800000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5800000
  done
  for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
    mock_add_cpu "$root" $((16+i)) $((8+i)) 4300000
  done
  mock_set_types "$root" "0-15" "16-31"
  mock_write_cpuinfo "$TMP/pcs_13900k_cpuinfo" "true" "6" "191"  # 0xBF

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/pcs_13900k_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  local expected="0 2 4 6 8 10 12 14 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31"
  expect_eq "PCS.13900K.slots" "$expected" "$PHYSICAL_CORE_SLOTS"
}

# ── Test 19: PHYSICAL_CORE_SLOTS — AMD Ryzen 8C/16T (non-hybrid SMT) ───────
# Non-hybrid SMT must collapse to even logicals (sibling excluded).
test_physical_core_slots_amd_ryzen_smt() {
  local root="$TMP/pcs_amd"
  mock_reset "$root"
  local i
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((i*2))   "$i" 5400000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5400000
  done
  mock_write_cpuinfo "$TMP/pcs_amd_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/pcs_amd_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "PCS.AMD.is_hybrid" "0"                          "$IS_HYBRID"
  expect_eq "PCS.AMD.slots"     "0 2 4 6 8 10 12 14"         "$PHYSICAL_CORE_SLOTS"
}

# ── Test 20: PHYSICAL_CORE_SLOTS — SMT-off 4-core (identity) ──────────────
test_physical_core_slots_smt_off_identity() {
  local root="$TMP/pcs_smtoff"
  mock_reset "$root"
  local i
  for i in 0 1 2 3; do
    mock_add_cpu "$root" "$i" "$i" 3000000
  done
  mock_write_cpuinfo "$TMP/pcs_smtoff_cpuinfo" "false"

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/pcs_smtoff_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  expect_eq "PCS.SmtOff.slots" "0 1 2 3" "$PHYSICAL_CORE_SLOTS"
}

# ── Test 21: PHYSICAL_CORE_SLOTS — NUC14 Pro 비표준 enumeration ────────────
# NUC14 Pro Meteor Lake (Core Ultra 7 155H) 일부 BIOS 펌웨어는 cpu 0 의 core_id
# 를 다른 P-core 들의 중간에 배치한다. sysfs 출력:
#   core_id A → cpus {1, 2}   (logical pair)
#   core_id B → cpus {3, 4}
#   core_id C → cpus {0, 5}   ← cpu 0 의 sibling 이 cpu 5
#   core_id D → cpus {6, 7}
#   core_id E → cpus {8, 9}
#   core_id F → cpus {10, 11}
# core_id 정렬 순서로 그대로 iteration 하면 P_CORE_PHYSICAL_IDS = [1, 3, 0, 6, 8, 10]
# 으로 비단조 — slot 0 이 cpu 1 이 되고 slot 2 가 cpu 0 이 되어 layout v4.1 의
# "slot 0 = OS 영역" 가정이 깨진다. (physical, sibling) pair 를 physical
# ascending 으로 sort 해 [0, 1, 3, 6, 8, 10] 으로 normalize 해야 한다.
test_physical_core_slots_nuc14_nonstandard_enum() {
  local root="$TMP/pcs_nuc14_nonstd"
  mock_reset "$root"
  # mock_add_cpu: $1=root $2=cpu $3=core_id $4=max_freq_khz
  # core_id ordering is intentionally non-monotonic across cpu number.
  mock_add_cpu "$root" 1  100 5000000  # core_id 100, cpus {1, 2}
  mock_add_cpu "$root" 2  100 5000000
  mock_add_cpu "$root" 3  101 5000000  # core_id 101, cpus {3, 4}
  mock_add_cpu "$root" 4  101 5000000
  mock_add_cpu "$root" 0  102 5000000  # core_id 102, cpus {0, 5} — cpu 0 in middle
  mock_add_cpu "$root" 5  102 5000000
  mock_add_cpu "$root" 6  103 5000000  # core_id 103, cpus {6, 7}
  mock_add_cpu "$root" 7  103 5000000
  mock_add_cpu "$root" 8  104 5000000  # core_id 104, cpus {8, 9}
  mock_add_cpu "$root" 9  104 5000000
  mock_add_cpu "$root" 10 105 5000000  # core_id 105, cpus {10, 11}
  mock_add_cpu "$root" 11 105 5000000
  # 8 E-cores 12..19
  local i
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((12+i)) $((200+i)) 3800000
  done
  # 2 LP-E cores 20..21, freq < 70% of E-max
  mock_add_cpu "$root" 20 208 2500000
  mock_add_cpu "$root" 21 209 2500000

  mock_set_types "$root" "0,1,2,3,4,5,6,7,8,9,10,11" "12,13,14,15,16,17,18,19,20,21"
  mock_write_cpuinfo "$TMP/pcs_nuc14_nonstd_cpuinfo" "true" "6" "170"  # 0xAA

  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/pcs_nuc14_nonstd_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 \
    detect_hybrid_capability

  # P-physical sorted ascending despite non-monotonic core_id grouping.
  expect_eq "NUC14NS.p_physical"    "0 1 3 6 8 10"                 "$P_CORE_PHYSICAL_IDS"
  expect_eq "NUC14NS.p_sibling"     "5 2 4 7 9 11"                 "$P_CORE_SIBLING_IDS"
  expect_eq "NUC14NS.e_cores"       "12 13 14 15 16 17 18 19"      "$E_CORE_IDS"
  expect_eq "NUC14NS.lpe_cores"     "20 21"                        "$LPE_CORE_IDS"
  # physical_core_slots: P → E → LP-E, all logical-id ascending.
  expect_eq "NUC14NS.slots" \
    "0 1 3 6 8 10 12 13 14 15 16 17 18 19 20 21" \
    "$PHYSICAL_CORE_SLOTS"
  # slot 0 maps to cpu 0 (OS-eligible primary), NOT cpu 1 like the unsorted
  # core_id-iteration order would yield.
  expect_eq "NUC14NS.is_hybrid"     "1"                            "$IS_HYBRID"
  expect_eq "NUC14NS.p_core_has_smt" "1"                           "$P_CORE_HAS_SMT"
  expect_eq "NUC14NS.has_lp_e_cores" "1"                           "$HAS_LP_E_CORES"
  expect_eq "NUC14NS.generation"    "meteor_lake"                  "$NUC_GENERATION"
}

# ── Test 22: sanity-check hook fires a warning on disagreement ─────────────
# Primary sysfs path classifies cpus 0-7 as P (all at a flat 5.0 GHz), but
# freq-clustering sees uniform freq → can't form an opinion → no warning.
# To force a disagreement we give sysfs an "intel_core" override that doesn't
# match the freq split. Sanity hook must print to stderr; we just check that
# the command does not alter detected globals (diagnostic only).
test_sanity_check_disagreement() {
  local root="$TMP/sanity"
  mock_reset "$root"
  local i
  # 4 P-cores @ 5.0 GHz (cpus 0..3) + 4 E-cores @ 3.0 GHz (cpus 4..7).
  for i in 0 1 2 3; do
    mock_add_cpu "$root" "$i" "$i" 5000000
  done
  for i in 4 5 6 7; do
    mock_add_cpu "$root" "$i" "$i" 3000000
  done
  # sysfs says the split is {0,1,2,3,4} | {5,6,7} — intentionally mislabels
  # cpu4 as a P-core. Freq-clustering would disagree (cpu4 is below P
  # threshold). Sanity hook must fire.
  mock_set_types "$root" "0,1,2,3,4" "5,6,7"
  mock_write_cpuinfo "$TMP/sanity_cpuinfo" "true"

  # Route stderr to a file so we can check the hook emitted something.
  local stderr_file="$TMP/sanity_stderr.log"
  RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/sanity_cpuinfo" \
    RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=1 \
    detect_hybrid_capability 2>"$stderr_file"

  # Primary split wins (unchanged behaviour).
  expect_eq "Sanity.detect_source"   "sysfs_types"  "$HYBRID_DETECT_SOURCE"
  expect_eq "Sanity.is_hybrid"       "1"            "$IS_HYBRID"
  # Hook must have printed a disagreement warning to stderr.
  if grep -q "hybrid sanity: P-core sets disagree" "$stderr_file" 2>/dev/null; then
    pass
  else
    fail "[Sanity] sanity-check warning not found in stderr: $(cat "$stderr_file")"
  fi
}

# ── get_rt_cores_with_siblings() / compute_shield_cores() tests ────────────
#
# These exercise the layout v4.1 SSoT helpers consumed by setup_grub_rt.sh
# (nohz_full / rcu_nocbs) and cpu_shield.sh (cset shield range). The helper
# walks $RTC_SYSFS_ROOT/devices/system/cpu/cpu*/topology/core_id; we build a
# fake topology and shadow get_physical_cores() per tier so the helper sees
# the desired core count without touching the host CPU.
#
# Coverage matrix:
#   * non-SMT: 4c, 6c, 10c, 12c, 14c, 16c — each tier verifies the helper
#     output collapses to the expected range (e.g. 12c → "1-5")
#   * SMT (6C/12T mock): RT physical cores 1,2,3 → "1-3,7-9" (sibling cpus 7,8,9)
#   * compute_shield_cores output matches the v4.1 tier table after migration

# Build a fake non-SMT sysfs tree: cpu0..N-1 each with core_id == cpu number.
_mock_sysfs_non_smt() {
  local root="$1" ncpu="$2"
  mock_reset "$root"
  local i
  for ((i=0; i<ncpu; i++)); do
    mock_add_cpu "$root" "$i" "$i" 0
  done
}

# Build a fake SMT sysfs tree: ncpu_phys physical cores with 2 HT siblings.
# cpu 0..N-1 are the physical-core primaries (core_id == i),
# cpu N..2N-1 are the HT siblings sharing core_id with cpu (i-N).
_mock_sysfs_smt() {
  local root="$1" ncpu_phys="$2"
  mock_reset "$root"
  local i
  for ((i=0; i<ncpu_phys; i++)); do
    mock_add_cpu "$root" "$i"               "$i" 0
    mock_add_cpu "$root" "$((i+ncpu_phys))" "$i" 0
  done
}

# Shadow get_physical_cores so layout helpers see the tier we want.
_force_physical_cores() {
  eval "get_physical_cores() { echo $1; }"
}

test_get_rt_cores_with_siblings_non_smt() {
  local root="$TMP/rtcs_nonsmt"
  local n
  # tier → expected output (matches docs/RT_OPTIMIZATION.md table)
  declare -A expected=(
    [4]="1-3" [6]="1-3" [8]="1-3" [9]="1-3"
    [10]="1-4" [11]="1-4"
    [12]="1-5" [13]="1-5" [14]="1-5" [15]="1-5" [16]="1-5"
  )
  for n in 4 6 8 9 10 11 12 13 14 15 16; do
    _mock_sysfs_non_smt "$root" "$n"
    _force_physical_cores "$n"
    local got
    got=$(RTC_SYSFS_ROOT="$root" get_rt_cores_with_siblings)
    expect_eq "rtcs_nonsmt.${n}c" "${expected[$n]}" "$got"
  done
}

test_get_rt_cores_with_siblings_smt_6c12t() {
  # 6 physical cores, SMT enabled. RT physical cores: 1,2,3.
  # Siblings: cpu 7 (core_id 1), cpu 8 (core_id 2), cpu 9 (core_id 3).
  # Expected output: "1-3,7-9" (range-collapsed sorted CSV).
  local root="$TMP/rtcs_smt_6c"
  _mock_sysfs_smt "$root" 6
  _force_physical_cores 6
  local got
  got=$(RTC_SYSFS_ROOT="$root" get_rt_cores_with_siblings)
  expect_eq "rtcs_smt.6c12t" "1-3,7-9" "$got"
}

test_get_rt_cores_with_siblings_smt_12c24t() {
  # 12 physical cores, SMT enabled. RT physical cores: 1,2,3,4,5.
  # Siblings: cpu 13..17 (share core_id with cpu 1..5).
  # Expected output: "1-5,13-17".
  local root="$TMP/rtcs_smt_12c"
  _mock_sysfs_smt "$root" 12
  _force_physical_cores 12
  local got
  got=$(RTC_SYSFS_ROOT="$root" get_rt_cores_with_siblings)
  expect_eq "rtcs_smt.12c24t" "1-5,13-17" "$got"
}

# ── slot→logical translation (canonical rt_common.sh helper) ─────────────────
test_slot_to_logical_cpu() {
  PHYSICAL_CORE_SLOTS="0 2 4 6 8 10"
  expect_eq "slot2log.s0"       "0"  "$(slot_to_logical_cpu 0)"
  expect_eq "slot2log.s1"       "2"  "$(slot_to_logical_cpu 1)"
  expect_eq "slot2log.s3"       "6"  "$(slot_to_logical_cpu 3)"
  expect_eq "slot2log.sentinel" "-1" "$(slot_to_logical_cpu -1)"   # cpu_core=-1 (no pin)
  expect_eq "slot2log.oob"      "20" "$(slot_to_logical_cpu 20)"   # out-of-range → identity
  PHYSICAL_CORE_SLOTS=""
  expect_eq "slot2log.empty"    "3"  "$(slot_to_logical_cpu 3)"    # container → identity
  unset PHYSICAL_CORE_SLOTS
}

# ── mask_to_cpus / rt_classify_external_rt_threads (verify_rt_runtime.sh 소비) ──
# 분류기는 external driver 프로세스의 RT 루프가 계획된 코어에 FIFO 로 앉아
# 있는지 판정한다. 이름으로 못 고르므로 (controller_manager 는 루프 스레드에
# pthread 이름을 안 준다 — 전 TID 가 같은 comm) 정책+우선순위가 선택자다.
# 아래 fixture 는 issue #343 이 실제로 관측한 네 상태를 고정한다.

test_mask_to_cpus() {
  expect_eq "mask.single"  "3"     "$(mask_to_cpus 8)"
  expect_eq "mask.multi"   "0,1"   "$(mask_to_cpus 3)"
  expect_eq "mask.all12"   "0,1,2,3,4,5,6,7,8,9,10,11" "$(mask_to_cpus fff)"
  expect_eq "mask.none"    "none"  "$(mask_to_cpus 0)"
  expect_eq "mask.high"    "10"    "$(mask_to_cpus 400)"
}

test_classify_external_rt_ok() {
  # 수정 후 기대 상태: cpu_affinity 가 루프 스레드 하나만 logical 10 으로 옮긴다.
  # 나머지 TID (executor, DDS) 는 로밍하며 판정에 관여하지 않는다.
  local out
  out=$(printf '%s\n' \
    "100 SCHED_OTHER 0 fff" \
    "101 SCHED_FIFO 50 400" \
    "102 SCHED_OTHER 0 fff" | rt_classify_external_rt_threads 50 10)
  expect_eq "cls.ok.verdict" "OK" "${out%%|*}"
  expect_eq "cls.ok.detail"  "1/1 SCHED_FIFO prio 50 on cpu 10" "${out#*|}"
}

test_classify_external_rt_no_fifo() {
  # issue #343 의 사각지대: FIFO 요청이 EPERM 으로 거부되면 affinity 는 그대로
  # 적용되므로 위치만 보는 검사는 통과한다. 실제로 로컬에서 재현된 상태다.
  local out
  out=$(printf '%s\n' \
    "100 SCHED_OTHER 0 fff" \
    "101 SCHED_OTHER 0 400" | rt_classify_external_rt_threads 50 10)
  expect_eq "cls.nofifo.verdict" "NO_FIFO" "${out%%|*}"
  expect_eq "cls.nofifo.detail"  "2 threads, none SCHED_FIFO" "${out#*|}"
}

test_classify_external_rt_wrong_cpu() {
  # 수정 전 상태: 루프는 FIFO 지만 핀이 안 걸려 전 코어를 로밍한다.
  local out
  out=$(printf '%s\n' \
    "100 SCHED_OTHER 0 400" \
    "101 SCHED_FIFO 50 fff" | rt_classify_external_rt_threads 50 10)
  expect_eq "cls.wrongcpu.verdict" "WRONG_CPU" "${out%%|*}"
  case "${out#*|}" in
    *"101:{0,1,2,3,4,5,6,7,8,9,10,11}"*) pass ;;
    *) fail "[cls.wrongcpu.detail] offending tid/cpus 누락: ${out#*|}" ;;
  esac
}

test_classify_external_rt_wrong_prio() {
  # 주입한 thread_priority 와 실제가 갈린 경우 — 값이 조용히 달라지는 것을 막는다.
  local out
  out=$(printf '%s\n' \
    "101 SCHED_FIFO 20 400" | rt_classify_external_rt_threads 50 10)
  expect_eq "cls.wrongprio.verdict" "WRONG_PRIO" "${out%%|*}"
  case "${out#*|}" in
    *"20"*"expected 50"*) pass ;;
    *) fail "[cls.wrongprio.detail] 실제/기대 우선순위 누락: ${out#*|}" ;;
  esac
}

test_classify_external_rt_ignores_non_rt_threads() {
  # RTDE 워커·DDS 가 아무 코어에 있어도 OK 여야 한다. 이들을 함께 강제하는 것이
  # #163 Phase 4 가 유예한 taskset -a 전면 적용이고, 이 fix 의 전제가 아니다.
  local out
  out=$(printf '%s\n' \
    "100 SCHED_OTHER 0 1" \
    "101 SCHED_FIFO 50 400" \
    "102 SCHED_OTHER 0 2" \
    "103 SCHED_OTHER 0 fff" | rt_classify_external_rt_threads 50 10)
  expect_eq "cls.ignore.verdict" "OK" "${out%%|*}"
}

# ── rt_classify_external_thread_layout (issue #345, udp_hand_node) ────────────
# 위 분류기는 "RT 스레드 하나 + 무명 나머지" 를, all-threads 검사는 "전부 같은
# 코어" 를 가정한다. hand 는 셋 다 아니다 — CommLoop 은 FIFO 65 로 hand 코어,
# hand_aux_io 는 CFS 로 aux 코어, 나머지(executor/DDS)는 CFS 로 hand 코어.
# 여기서 external driver 의 **이름별 policy/priority** 를 처음으로 검사한다.

# hand 코어 = logical 11 (12코어 NUC13 slot 7), aux = logical 0.
_HAND_SPEC="hand_udp_recv:11:SCHED_FIFO:65 hand_aux_io:0:SCHED_OTHER:0"

test_classify_layout_ok() {
  local out
  out=$(printf '%s\n' \
    "100 udp_hand_node SCHED_OTHER 0 800" \
    "101 hand_udp_recv SCHED_FIFO 65 800" \
    "102 hand_aux_io SCHED_OTHER 0 1" \
    "103 recv SCHED_OTHER 0 800" | rt_classify_external_thread_layout 11 "$_HAND_SPEC")
  expect_eq "layout.ok.verdict" "OK" "${out%%|*}"
}

test_classify_layout_aux_dragged_back() {
  # `taskset -a` 스윕이 되살아나면 정확히 이 모양이 된다 — aux 가 hand 코어로.
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 65 800" \
    "102 hand_aux_io SCHED_OTHER 0 800" | rt_classify_external_thread_layout 11 "$_HAND_SPEC")
  expect_eq "layout.aux_dragged.verdict" "WRONG_CPU" "${out%%|*}"
}

test_classify_layout_commloop_lost_fifo() {
  # ApplyThreadConfig 가 affinity 실패로 조기 반환하면 정책이 안 붙는다.
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_OTHER 0 800" \
    "102 hand_aux_io SCHED_OTHER 0 1" | rt_classify_external_thread_layout 11 "$_HAND_SPEC")
  expect_eq "layout.no_fifo.verdict" "WRONG_SCHED" "${out%%|*}"
}

test_classify_layout_wrong_prio() {
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 50 800" \
    "102 hand_aux_io SCHED_OTHER 0 1" | rt_classify_external_thread_layout 11 "$_HAND_SPEC")
  expect_eq "layout.wrong_prio.verdict" "WRONG_SCHED" "${out%%|*}"
}

test_classify_layout_aux_became_realtime() {
  # 정책 검사가 *유일하게* 필요한 방향: OTHER 를 기대한 레인이 FIFO 로 도는 경우.
  # 반대 방향(FIFO 기대인데 OTHER)은 prio 검사가 덤으로 잡으므로, 이 케이스가
  # 없으면 정책 검사를 통째로 지워도 테스트가 전부 통과한다 (실측으로 확인함).
  # 실제 위험도 있다 — aux 가 RT 로 승격되면 aux 코어를 선점할 뿐 아니라
  # pin_dds_threads_to_slot 의 FIFO 가드가 그것을 조용히 건너뛰게 된다.
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 65 800" \
    "102 hand_aux_io SCHED_FIFO 65 1" | rt_classify_external_thread_layout 11 "$_HAND_SPEC")
  expect_eq "layout.aux_rt.verdict" "WRONG_SCHED" "${out%%|*}"
}

test_classify_layout_unpinned_sentinel_is_not_a_failure() {
  # RTC_HAND_AUX_SLOT=-1 는 "aux 를 고정하지 않는다" 는 문서화된 sentinel
  # (thread_config.hpp cpu_core=-1, slot_to_logical_cpu 는 음수를 그대로 통과).
  # 가드가 없으면 $((1 << -1)) 이 조용히 거대한 음수가 되어, 올바르게 설정된
  # 스레드가 어떤 마스크를 갖든 WRONG_CPU 로 오탐된다 — 잡아야 할 오설정이
  # 아니라 verifier 가 만들어내는 가짜 FAIL 이다.
  local out spec="hand_udp_recv:11:SCHED_FIFO:65 hand_aux_io:-1:SCHED_OTHER:0"
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 65 800" \
    "102 hand_aux_io SCHED_OTHER 0 fff" | rt_classify_external_thread_layout 11 "$spec")
  expect_eq "layout.unpinned_sentinel.verdict" "OK" "${out%%|*}"
}

test_classify_layout_missing_thread() {
  # 이름이 안 붙었거나(=ApplyThreadConfig 조기 반환) 레인이 안 떴을 때. 코어가
  # 맞는지보다 먼저 보고돼야 한다 — 없는 스레드는 더 나쁜 발견이다.
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 65 800" | rt_classify_external_thread_layout 11 "$_HAND_SPEC")
  expect_eq "layout.missing.verdict" "MISSING" "${out%%|*}"
}

test_classify_layout_unnamed_thread_must_be_on_driver_core() {
  # spec 에 없는 TID (executor, DDS) 는 driver 코어에 있어야 한다 — 수용 기준의
  # "그 외 steady state TID 는 hand_driver slot" 이 이 줄이다.
  local out
  out=$(printf '%s\n' \
    "100 udp_hand_node SCHED_OTHER 0 1" \
    "101 hand_udp_recv SCHED_FIFO 65 800" \
    "102 hand_aux_io SCHED_OTHER 0 1" | rt_classify_external_thread_layout 11 "$_HAND_SPEC")
  expect_eq "layout.stray.verdict" "WRONG_CPU" "${out%%|*}"
}

test_classify_layout_degraded_tier_collapse() {
  # <=5 코어 tier: hand slot 과 OS slot 이 둘 다 0 이라 두 기대가 같은 논리 CPU 로
  # 풀린다. 예외 처리 없이 통과해야 한다 — 그 tier 는 원래 Core 0 에 전부 모인다.
  local out
  out=$(printf '%s\n' \
    "100 udp_hand_node SCHED_OTHER 0 1" \
    "101 hand_udp_recv SCHED_FIFO 65 1" \
    "102 hand_aux_io SCHED_OTHER 0 1" \
    | rt_classify_external_thread_layout 0 "hand_udp_recv:0:SCHED_FIFO:65 hand_aux_io:0:SCHED_OTHER:0")
  expect_eq "layout.degraded.verdict" "OK" "${out%%|*}"
}

test_classify_layout_no_threads() {
  local out
  out=$(printf '' | rt_classify_external_thread_layout 11 "$_HAND_SPEC")
  expect_eq "layout.empty.verdict" "NO_THREADS" "${out%%|*}"
}

# ── cpu_shield.sh::compute_shield_cores → get_rt_shield_cpus (slot→logical+sibling) ─
# The shield set is the LOGICAL CPUs RT threads pin to (via PHYSICAL_CORE_SLOTS,
# the same map C++ ApplyThreadConfig uses) plus HT siblings — NOT the raw slot
# indices get_rt_cores() returns. These tests lock that behaviour under mock sysfs.

test_get_rt_shield_cpus_non_smt() {
  local root="$TMP/shield_nonsmt"
  declare -A expected=( [6]="1-3" [8]="1-3" [9]="1-3" [10]="1-4" [11]="1-4" [12]="1-5" [16]="1-5" )
  local n
  for n in 6 8 9 10 11 12 16; do
    _mock_sysfs_non_smt "$root" "$n"
    mock_write_cpuinfo "$TMP/shield_nonsmt_cpuinfo" "false"
    _force_physical_cores "$n"
    local got
    got=$(RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/shield_nonsmt_cpuinfo" \
          RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 get_rt_shield_cpus)
    expect_eq "shield.nonsmt.${n}c" "${expected[$n]}" "$got"
  done
}

test_get_rt_shield_cpus_smt_primaries_first() {
  # Standard enum: cpu 0..5 primaries (core_id i), cpu 6..11 siblings.
  # Slots 1,2,3 → logical 1,2,3 (identity) → + siblings 7,8,9.
  local root="$TMP/shield_smt_std"
  _mock_sysfs_smt "$root" 6
  mock_write_cpuinfo "$TMP/shield_smt_std_cpuinfo" "false"
  _force_physical_cores 6
  local got
  got=$(RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/shield_smt_std_cpuinfo" \
        RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 get_rt_shield_cpus)
  expect_eq "shield.smt_primaries.6c" "1-3,7-9" "$got"
}

test_get_rt_shield_cpus_smt_interleaved() {
  # Interleaved enum (AMD SMT style): cpu 2i, 2i+1 share core_id i. So slot 1 →
  # logical 2, slot 2 → logical 4, slot 3 → logical 6 (NOT 1,2,3). Shield =
  # {2,3,4,5,6,7} = "2-7". The pre-fix code returned raw "1-3", which shielded
  # logical 1 (core 0's HT sibling — the OS core's hyperthread) instead of the
  # real RT logical CPUs. This is the regression this whole change fixes.
  local root="$TMP/shield_smt_il"
  mock_reset "$root"
  local i
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((i*2))   "$i" 5400000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5400000
  done
  mock_write_cpuinfo "$TMP/shield_smt_il_cpuinfo" "false"
  _force_physical_cores 8
  local got
  got=$(RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/shield_smt_il_cpuinfo" \
        RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 get_rt_shield_cpus)
  expect_eq "shield.smt_interleaved.8c" "2-7" "$got"
}

test_get_rt_shield_cpus_low_core_no_phantom() {
  # 2-core machine: the degraded layout still references slots 1,2,3 but cores 2
  # and 3 do not exist. Phantom cores must be dropped so cset/taskset never gets
  # an invalid core id (would fail). Only the one existing RT core (1) survives.
  local root="$TMP/shield_2core"
  _mock_sysfs_non_smt "$root" 2
  mock_write_cpuinfo "$TMP/shield_2core_cpuinfo" "false"
  _force_physical_cores 2
  local got
  got=$(RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/shield_2core_cpuinfo" \
        RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 get_rt_shield_cpus)
  expect_eq "shield.2core.no_phantom" "1" "$got"
}

# ── cpu_shield.sh cpuset span → get_cm_shield_cpus (RT ∪ nrt, OS slot dropped) ─
# The cset "user" cpuset must fit the WHOLE CM process (RT + nrt threads), not
# just the RT cores get_rt_shield_cpus returns — cset moves a process, and the
# CM's nrt threads pin outside the RT-only span. Issue #151.

test_get_cm_shield_cpus_nuc13_hybrid() {
  # NUC13 Pro (Raptor Lake-P 4P+8E, HT on): P-cores 0-3 → logical 0-7,
  # E-cores 4-11 → logical 8-15. RT slots 1-5 → logical {2,4,6,8,9}; nrt slots
  # 8,9 → logical {12,13}. CM cpuset = union + P-core HT siblings = "2-9,12-13".
  local root="$TMP/cm_nuc13"
  mock_reset "$root"
  local i
  for i in 0 1 2 3; do
    mock_add_cpu "$root" $((i*2))   "$i" 5000000
    mock_add_cpu "$root" $((i*2+1)) "$i" 5000000
  done
  for i in 0 1 2 3 4 5 6 7; do
    mock_add_cpu "$root" $((8+i)) $((4+i)) 3800000
  done
  mock_set_types "$root" "0,1,2,3,4,5,6,7" "8,9,10,11,12,13,14,15"
  mock_write_cpuinfo "$TMP/cm_nuc13_cpuinfo" "true" "6" "186"  # 0xBA
  _force_physical_cores 12
  local got
  got=$(RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/cm_nuc13_cpuinfo" \
        RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 get_cm_shield_cpus)
  expect_eq "cm_shield.nuc13_hybrid.12c" "2-9,12-13" "$got"
}

test_get_cm_shield_cpus_non_smt() {
  # Non-SMT: RT slots 1-5 (identity) + nrt slots per tier, no siblings.
  #   6-7c:  RT 1-3 + nrt 5              → "1-3,5"
  #   8-9c:  RT 1-3 + nrt 6,7            → "1-3,6-7"
  #   10-11c:RT 1-4 + nrt 7,8            → "1-4,7-8"
  #   12c+:  RT 1-5 + nrt 8,9            → "1-5,8-9"
  local root="$TMP/cm_nonsmt"
  declare -A expected=( [6]="1-3,5" [8]="1-3,6-7" [10]="1-4,7-8" [12]="1-5,8-9" [16]="1-5,8-9" )
  local n
  for n in 6 8 10 12 16; do
    _mock_sysfs_non_smt "$root" "$n"
    mock_write_cpuinfo "$TMP/cm_nonsmt_cpuinfo" "false"
    _force_physical_cores "$n"
    local got
    got=$(RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/cm_nonsmt_cpuinfo" \
          RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 get_cm_shield_cpus)
    expect_eq "cm_shield.nonsmt.${n}c" "${expected[$n]}" "$got"
  done
}

test_get_cm_shield_cpus_degraded_drops_os_slot() {
  # Degraded (<6-core): nrt shares OS Core 0 (slot 0). The OS slot must be
  # dropped so the shield never claims Core 0 — result collapses to RT-only.
  # 4-core: RT slots 1,2,3 → "1-3" (identical to get_rt_shield_cpus).
  local root="$TMP/cm_degraded"
  _mock_sysfs_non_smt "$root" 4
  mock_write_cpuinfo "$TMP/cm_degraded_cpuinfo" "false"
  _force_physical_cores 4
  local got
  got=$(RTC_SYSFS_ROOT="$root" RTC_PROC_CPUINFO="$TMP/cm_degraded_cpuinfo" \
        RTC_FORCE_HYBRID_GENERATION="" RTC_HYBRID_SANITY=0 get_cm_shield_cpus)
  expect_eq "cm_shield.degraded.4c.no_os_slot" "1-3" "$got"
}

# ── Run all ─────────────────────────────────────────────────────────────────
test_cpulist_parser
test_nuc13_i7_1360p
test_bios_ht_off
test_container_fallback
test_amd_ryzen
test_env_hint_override
test_freq_fallback_meteor_lake
test_freq_fallback_amd_negative
test_detect_source_primary
test_container_no_freq_files
test_platform_label_raptor_lake_s_desktop
test_platform_label_raptor_lake_p_mobile
test_platform_label_arrow_lake_s_desktop
test_platform_label_unknown_model
test_platform_label_amd_vendor
test_platform_label_missing_fields
test_physical_core_slots_nuc13_hybrid
test_physical_core_slots_13900k_hybrid
test_physical_core_slots_amd_ryzen_smt
test_physical_core_slots_smt_off_identity
test_physical_core_slots_nuc14_nonstandard_enum
test_sanity_check_disagreement

# v4.1 Layout SSoT helpers (setup_grub_rt.sh / cpu_shield.sh consumers).
test_get_rt_cores_with_siblings_non_smt
test_get_rt_cores_with_siblings_smt_6c12t
test_get_rt_cores_with_siblings_smt_12c24t
test_slot_to_logical_cpu
test_mask_to_cpus
test_classify_external_rt_ok
test_classify_external_rt_no_fifo
test_classify_external_rt_wrong_cpu
test_classify_external_rt_wrong_prio
test_classify_external_rt_ignores_non_rt_threads
test_classify_layout_ok
test_classify_layout_aux_dragged_back
test_classify_layout_commloop_lost_fifo
test_classify_layout_wrong_prio
test_classify_layout_aux_became_realtime
test_classify_layout_unpinned_sentinel_is_not_a_failure
test_classify_layout_missing_thread
test_classify_layout_unnamed_thread_must_be_on_driver_core
test_classify_layout_degraded_tier_collapse
test_classify_layout_no_threads
test_get_rt_shield_cpus_non_smt
test_get_rt_shield_cpus_smt_primaries_first
test_get_rt_shield_cpus_smt_interleaved
test_get_rt_shield_cpus_low_core_no_phantom
test_get_cm_shield_cpus_nuc13_hybrid
test_get_cm_shield_cpus_non_smt
test_get_cm_shield_cpus_degraded_drops_os_slot

# ── Phase 1 safety primitives: write_file_if_changed / with_temporary_disable ──

test_write_file_if_changed_creates_new() {
  local tmp_root
  tmp_root=$(mktemp -d)
  local target="${tmp_root}/new.conf"
  write_file_if_changed "$target" "hello"
  expect_eq "new_created.exists" "true" "$([[ -f "$target" ]] && echo true || echo false)"
  expect_eq "new_created.content" "hello" "$(cat "$target")"
  # Default mode 0644
  expect_eq "new_created.mode" "644" "$(stat -c '%a' "$target")"
  rm -rf -- "$tmp_root"
}

test_write_file_if_changed_skips_identical() {
  local tmp_root
  tmp_root=$(mktemp -d)
  local target="${tmp_root}/same.conf"
  printf '%s\n' "same" > "$target"
  local before
  before=$(stat -c '%Y' "$target")
  sleep 1  # mtime 비교 안정
  if write_file_if_changed "$target" "same"; then
    fail "identical_skip.return: expected non-zero (skipped), got 0 (written)"
  else
    pass
  fi
  local after
  after=$(stat -c '%Y' "$target")
  expect_eq "identical_skip.mtime_unchanged" "$before" "$after"
  rm -rf -- "$tmp_root"
}

test_write_file_if_changed_preserves_mode() {
  local tmp_root
  tmp_root=$(mktemp -d)
  local target="${tmp_root}/exec.sh"
  printf '%s\n' "old" > "$target"
  chmod 0755 "$target"
  write_file_if_changed "$target" "new"
  expect_eq "preserve_mode.content" "new" "$(cat "$target")"
  expect_eq "preserve_mode.mode" "755" "$(stat -c '%a' "$target")"
  rm -rf -- "$tmp_root"
}

test_write_file_if_changed_atomic_no_partial() {
  # write_file_if_changed 의 tmp 파일이 target 디렉토리 안에서 생성되는지 검증
  # (cross-fs EXDEV 방지). 직접적 atomicity 는 crash injection 없이 unit test 가
  # 어려우므로, mktemp pattern + same-dir 위치만 검증.
  local tmp_root
  tmp_root=$(mktemp -d)
  local target="${tmp_root}/atomic.conf"
  write_file_if_changed "$target" "data"
  # write 후 tmp 잔존 없어야 함 (mv 가 atomic 으로 rename)
  local leftover
  leftover=$(find "$tmp_root" -maxdepth 1 -name '.atomic.conf.*' 2>/dev/null | wc -l)
  expect_eq "atomic.no_leftover_tmp" "0" "$leftover"
  expect_eq "atomic.target_exists" "true" "$([[ -f "$target" ]] && echo true || echo false)"
  rm -rf -- "$tmp_root"
}

test_with_temporary_disable_restores_on_success() {
  local tmp_root
  tmp_root=$(mktemp -d)
  local hook="${tmp_root}/fake-hook"
  printf '#!/bin/bash\nexit 0\n' > "$hook"
  chmod +x "$hook"
  with_temporary_disable "$hook" -- true
  # 명령 성공 후 hook 이 다시 실행 가능해야 함
  expect_eq "tmp_disable.success.restored" "true" "$([[ -x "$hook" ]] && echo true || echo false)"
  rm -rf -- "$tmp_root"
}

test_with_temporary_disable_restores_on_failure() {
  # 핵심 buy: 내부 명령이 실패해도 hook 이 +x 로 복원되는가
  local tmp_root
  tmp_root=$(mktemp -d)
  local hook="${tmp_root}/fake-hook"
  printf '#!/bin/bash\nexit 0\n' > "$hook"
  chmod +x "$hook"
  # set -e 환경에서 실패 명령 — 함수 종료 후 hook 이 +x 여야 함
  with_temporary_disable "$hook" -- false || true
  expect_eq "tmp_disable.failure.restored" "true" "$([[ -x "$hook" ]] && echo true || echo false)"
  rm -rf -- "$tmp_root"
}

test_with_temporary_disable_missing_hook() {
  # Hook 자체가 없으면 그냥 명령만 실행되어야 함 (fatal X)
  local tmp_root
  tmp_root=$(mktemp -d)
  local missing="${tmp_root}/no-such-hook"
  local sentinel="${tmp_root}/sentinel"
  with_temporary_disable "$missing" -- touch "$sentinel"
  expect_eq "tmp_disable.missing_hook.cmd_ran" "true" "$([[ -f "$sentinel" ]] && echo true || echo false)"
  rm -rf -- "$tmp_root"
}

test_write_file_if_changed_creates_new
test_write_file_if_changed_skips_identical
test_write_file_if_changed_preserves_mode
test_write_file_if_changed_atomic_no_partial
test_with_temporary_disable_restores_on_success
test_with_temporary_disable_restores_on_failure
test_with_temporary_disable_missing_hook

echo
echo "── test_rt_common.sh summary ──"
echo "  PASS: $PASS"
echo "  FAIL: $FAIL"
if (( FAIL > 0 )); then
  printf '  %s\n' "${FAIL_MSGS[@]}"
  exit 1
fi
exit 0
