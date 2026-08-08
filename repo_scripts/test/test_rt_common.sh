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
  # tier → expected output (matches docs/RT_OPTIMIZATION.md table).
  # #380 이 mpc_worker 슬롯 4·5 를 회수한 뒤로 RT 집합은 tier 와 무관하게
  # slot 1-3 이다. 10+ 가 "1-4"/"1-5" 로 되돌아오면 아무도 실행하지 않는
  # 코어를 다시 격리하고 있다는 뜻이다.
  declare -A expected=(
    [4]="1-3" [6]="1-3" [8]="1-3" [9]="1-3"
    [10]="1-3" [11]="1-3"
    [12]="1-3" [13]="1-3" [14]="1-3" [15]="1-3" [16]="1-3"
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
  # 12 physical cores, SMT enabled. RT physical cores: 1,2,3 (#380 회수 이후).
  # Siblings: cpu 13..15 (share core_id with cpu 1..3).
  # Expected output: "1-3,13-15".
  local root="$TMP/rtcs_smt_12c"
  _mock_sysfs_smt "$root" 12
  _force_physical_cores 12
  local got
  got=$(RTC_SYSFS_ROOT="$root" get_rt_cores_with_siblings)
  expect_eq "rtcs_smt.12c24t" "1-3,13-15" "$got"
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

# hand 코어 = logical 9 (12코어 NUC13 slot 5, v6/#383), aux = logical 0.
# 이 두 숫자는 slot 이 아니라 **논리 CPU** 라서 생성기가 안 닿는다 — 마스크
# 0x200 = bit 9 도 같다. layout 을 고치면 여기를 손으로 따라와야 한다.
_HAND_SPEC="hand_udp_recv:9:SCHED_FIFO:65 hand_aux_io:0:SCHED_OTHER:0"

test_classify_layout_ok() {
  local out
  out=$(printf '%s\n' \
    "100 udp_hand_node SCHED_OTHER 0 200" \
    "101 hand_udp_recv SCHED_FIFO 65 200" \
    "102 hand_aux_io SCHED_OTHER 0 1" \
    "103 recv SCHED_OTHER 0 200" | rt_classify_external_thread_layout 9 "$_HAND_SPEC")
  expect_eq "layout.ok.verdict" "OK" "${out%%|*}"
}

test_classify_layout_aux_dragged_back() {
  # `taskset -a` 스윕이 되살아나면 정확히 이 모양이 된다 — aux 가 hand 코어로.
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 65 200" \
    "102 hand_aux_io SCHED_OTHER 0 200" | rt_classify_external_thread_layout 9 "$_HAND_SPEC")
  expect_eq "layout.aux_dragged.verdict" "WRONG_CPU" "${out%%|*}"
}

test_classify_layout_commloop_lost_fifo() {
  # ApplyThreadConfig 가 affinity 실패로 조기 반환하면 정책이 안 붙는다.
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_OTHER 0 200" \
    "102 hand_aux_io SCHED_OTHER 0 1" | rt_classify_external_thread_layout 9 "$_HAND_SPEC")
  expect_eq "layout.no_fifo.verdict" "WRONG_SCHED" "${out%%|*}"
}

test_classify_layout_wrong_prio() {
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 50 200" \
    "102 hand_aux_io SCHED_OTHER 0 1" | rt_classify_external_thread_layout 9 "$_HAND_SPEC")
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
    "101 hand_udp_recv SCHED_FIFO 65 200" \
    "102 hand_aux_io SCHED_FIFO 65 1" | rt_classify_external_thread_layout 9 "$_HAND_SPEC")
  expect_eq "layout.aux_rt.verdict" "WRONG_SCHED" "${out%%|*}"
}

test_classify_layout_unpinned_sentinel_is_not_a_failure() {
  # RTC_HAND_AUX_SLOT=-1 는 "aux 를 고정하지 않는다" 는 문서화된 sentinel
  # (thread_config.hpp cpu_core=-1, slot_to_logical_cpu 는 음수를 그대로 통과).
  # 가드가 없으면 $((1 << -1)) 이 조용히 거대한 음수가 되어, 올바르게 설정된
  # 스레드가 어떤 마스크를 갖든 WRONG_CPU 로 오탐된다 — 잡아야 할 오설정이
  # 아니라 verifier 가 만들어내는 가짜 FAIL 이다.
  local out spec="hand_udp_recv:9:SCHED_FIFO:65 hand_aux_io:-1:SCHED_OTHER:0"
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 65 200" \
    "102 hand_aux_io SCHED_OTHER 0 fff" | rt_classify_external_thread_layout 9 "$spec")
  expect_eq "layout.unpinned_sentinel.verdict" "OK" "${out%%|*}"
}

test_classify_layout_missing_thread() {
  # 이름이 안 붙었거나(=ApplyThreadConfig 조기 반환) 레인이 안 떴을 때. 코어가
  # 맞는지보다 먼저 보고돼야 한다 — 없는 스레드는 더 나쁜 발견이다.
  local out
  out=$(printf '%s\n' \
    "101 hand_udp_recv SCHED_FIFO 65 200" | rt_classify_external_thread_layout 9 "$_HAND_SPEC")
  expect_eq "layout.missing.verdict" "MISSING" "${out%%|*}"
}

test_classify_layout_unnamed_thread_must_be_on_driver_core() {
  # spec 에 없는 TID (executor, DDS) 는 driver 코어에 있어야 한다 — 수용 기준의
  # "그 외 steady state TID 는 hand_driver slot" 이 이 줄이다.
  local out
  out=$(printf '%s\n' \
    "100 udp_hand_node SCHED_OTHER 0 1" \
    "101 hand_udp_recv SCHED_FIFO 65 200" \
    "102 hand_aux_io SCHED_OTHER 0 1" | rt_classify_external_thread_layout 9 "$_HAND_SPEC")
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
  out=$(printf '' | rt_classify_external_thread_layout 9 "$_HAND_SPEC")
  expect_eq "layout.empty.verdict" "NO_THREADS" "${out%%|*}"
}

# ── cpu_shield.sh::compute_shield_cores → get_rt_shield_cpus (slot→logical+sibling) ─
# The shield set is the LOGICAL CPUs RT threads pin to (via PHYSICAL_CORE_SLOTS,
# the same map C++ ApplyThreadConfig uses) plus HT siblings — NOT the raw slot
# indices get_rt_cores() returns. These tests lock that behaviour under mock sysfs.

test_get_rt_shield_cpus_non_smt() {
  local root="$TMP/shield_nonsmt"
  # 전 tier 동일 — #380 이후 RT 슬롯은 1-3 뿐이다.
  declare -A expected=( [6]="1-3" [8]="1-3" [9]="1-3" [10]="1-3" [11]="1-3" [12]="1-3" [16]="1-3" )
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
# just the RT cores get_rt_shield_cpus returns — cset moves a process, and under
# v4.1 the CM's nrt threads pinned outside the RT-only span. Issue #151.
#
# v5 (#349) dissolved that condition: nrt sits on aux slot 2 (⊂ RT) on tiers ≥ 6
# and on the OS slot (dropped) on tier 4, so the union now equals the RT span on
# EVERY tier. The union is kept rather than collapsed into get_rt_shield_cpus —
# it is what makes a future tier that splits nrt back out widen the cpuset
# automatically instead of silently leaving those threads outside the shield.
# These cases therefore pin "union == RT span" as a *result*, not as an identity.

test_get_cm_shield_cpus_nuc13_hybrid() {
  # NUC13 Pro (Raptor Lake-P 4P+8E, HT on): P-cores 0-3 → logical 0-7,
  # E-cores 4-11 → logical 8-15. RT slots 1-3 → logical {2,4,6}; v5 부터
  # nrt 는 aux slot 2 → logical 4 로 **이미 RT 집합 안**이라 union 이 넓어지지
  # 않는다. CM cpuset = RT + P-core HT siblings = "2-7".
  #
  # 이 한 줄이 primary target tier 의 shield 를 실기 이전에 데스크톱에서
  # 결정적으로 검증하는 유일한 센서다. 값의 역사가 곧 코어 반환의 역사다:
  # v4.1 "2-9,12-13" → #349 가 nrt 를 aux slot 으로 접어 "2-9" → #380 이
  # mpc_worker 슬롯 4·5 (logical 8,9) 를 회수해 "2-7". 앞의 값이 되돌아오면
  # 그만큼의 반환이 취소된 것이다. E 코어 8·9 는 이제 system cpuset 에 남는다.
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
  expect_eq "cm_shield.nuc13_hybrid.12c" "2-7" "$got"
}

test_get_cm_shield_cpus_non_smt() {
  # Non-SMT: RT slots (identity, no siblings). v5 부터 nrt 는 aux slot 2 라
  # RT 집합의 부분집합이고, union 은 RT span 그대로다 — v4.1 이 여기 붙이던
  # 꼬리(",5" / ",6-7" / ",7-8" / ",8-9")가 tier 마다 사라진다.
  # #380 이 worker 슬롯을 회수한 뒤로는 tier 축까지 평평해진다 — 모든 tier 가
  # RT 1-3 + nrt 2 ⊂ RT → "1-3".
  local root="$TMP/cm_nonsmt"
  declare -A expected=( [6]="1-3" [8]="1-3" [10]="1-3" [12]="1-3" [16]="1-3" )
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

# ── External driver slot table (issue #353) ─────────────────────────────────
# get_{arm,hand}_driver_slot 는 thread_config.hpp tier 표의 shell 미러이고,
# verify_rt_runtime.sh 의 기대값과 print_thread_layout 의 다이어그램이 둘 다
# 이것을 소비한다. 아래 fixture 가 **경계마다** 값을 박으므로, 표를 고치면서
# 소비자를 안 고치는 대신 표 자체를 잘못 고치면 여기서 red 가 난다.

test_external_driver_slots_per_tier() {
  # 경계는 SelectThreadConfigs() dispatch 와 같다: <6 degraded, {6,7} 공유,
  # {8,9} 분리 시작, 8+ 고정.
  #
  # v6 (#383) 부터 8 이상은 값이 **평평하다** (arm 4 / hand 5). 그래서 이 표만으로는
  # "ncpu=12 가 12-tier 로 갔는가" 를 더 이상 구별하지 못한다 — 구별할 차이가 layout
  # 에서 사라졌기 때문이지 센서가 약해진 게 아니다. tier dispatch 축은 여전히
  # test_layout_tier_tables_per_tier (아래) 와 test_thread_layout_equivalence.py
  # (ncpu 1..17+24 마다 python/manifest 의 tier id 일치) 가 각각 독립으로 박는다.
  # 여기서 남는 역할은 degraded→공유→전용 전이 두 곳(5→6, 7→8)의 고정이다.
  expect_eq "drvslot.arm.4"   "0" "$(get_arm_driver_slot 4)"
  expect_eq "drvslot.hand.4"  "0" "$(get_hand_driver_slot 4)"
  expect_eq "drvslot.arm.5"   "0" "$(get_arm_driver_slot 5)"
  expect_eq "drvslot.hand.5"  "0" "$(get_hand_driver_slot 5)"
  expect_eq "drvslot.arm.6"   "4" "$(get_arm_driver_slot 6)"
  expect_eq "drvslot.hand.6"  "4" "$(get_hand_driver_slot 6)"
  expect_eq "drvslot.arm.7"   "4" "$(get_arm_driver_slot 7)"
  expect_eq "drvslot.hand.7"  "4" "$(get_hand_driver_slot 7)"
  expect_eq "drvslot.arm.8"   "4" "$(get_arm_driver_slot 8)"
  expect_eq "drvslot.hand.8"  "5" "$(get_hand_driver_slot 8)"
  expect_eq "drvslot.arm.9"   "4" "$(get_arm_driver_slot 9)"
  expect_eq "drvslot.hand.9"  "5" "$(get_hand_driver_slot 9)"
  expect_eq "drvslot.arm.10"  "4" "$(get_arm_driver_slot 10)"
  expect_eq "drvslot.hand.10" "5" "$(get_hand_driver_slot 10)"
  expect_eq "drvslot.arm.11"  "4" "$(get_arm_driver_slot 11)"
  expect_eq "drvslot.hand.11" "5" "$(get_hand_driver_slot 11)"
  expect_eq "drvslot.arm.12"  "4" "$(get_arm_driver_slot 12)"
  expect_eq "drvslot.hand.12" "5" "$(get_hand_driver_slot 12)"
  expect_eq "drvslot.arm.16"  "4" "$(get_arm_driver_slot 16)"
  expect_eq "drvslot.hand.16" "5" "$(get_hand_driver_slot 16)"
  expect_eq "drvslot.arm.24"  "4" "$(get_arm_driver_slot 24)"
  expect_eq "drvslot.hand.24" "5" "$(get_hand_driver_slot 24)"
}

test_external_driver_slots_share_a_core_only_on_the_degraded_tiers() {
  # 6-7 tier 의 print_thread_layout 은 "arm_driver + hand_driver (shared)" 를
  # 한 줄로 그린다. 그 줄이 참이려면 두 슬롯이 같아야 하고, 8+ 부터는 달라야
  # 한다 — 다이어그램이 조용히 거짓말하지 않게 그 전제를 박는다.
  local n
  for n in 4 5 6 7; do
    expect_eq "drvslot.shared.${n}" "$(get_arm_driver_slot "$n")" "$(get_hand_driver_slot "$n")"
  done
  for n in 8 10 12 16; do
    if [[ "$(get_arm_driver_slot "$n")" == "$(get_hand_driver_slot "$n")" ]]; then
      fail "[drvslot.split.${n}] arm 과 hand 가 같은 slot ($(get_arm_driver_slot "$n")) — 전용 코어 tier 인데 공유"
    else
      pass
    fi
  done
}

test_print_thread_layout_uses_the_slot_helpers() {
  # print_thread_layout 은 이 표의 세 번째 미러다. 숫자를 다시 적으면 헬퍼를
  # 고쳐도 다이어그램만 옛 배치를 계속 그린다 — 그래서 배선을 확인한다
  # (값 자체는 위 fixture 가 소유한다).
  local out n arm hand
  for n in 8 10 12 16; do
    arm=$(get_arm_driver_slot "$n")
    hand=$(get_hand_driver_slot "$n")
    out=$(print_thread_layout "$n")
    if grep -qE "Core ${arm}: +arm_driver" <<<"$out"; then
      pass
    else
      fail "[layout.arm.${n}] 'Core ${arm}: arm_driver' 줄이 없다 — 다이어그램이 헬퍼를 안 쓴다"
    fi
    if grep -qE "Core ${hand}: +hand_driver" <<<"$out"; then
      pass
    else
      fail "[layout.hand.${n}] 'Core ${hand}: hand_driver' 줄이 없다 — 다이어그램이 헬퍼를 안 쓴다"
    fi
  done
}

test_verifier_expected_slots_track_the_helpers() {
  # 이 슬라이스의 본체: verify_rt_runtime.sh 가 tier 표를 자체 복제하면
  # **틀린 기대값으로 PASS** 를 낸다 (센서가 거짓말하는 방향). 스크립트를
  # source 해서 build_external_drivers 를 직접 돌리고, 채워진 기대 slot 이
  # 헬퍼와 같은지 본다 — grep 이 아니라 실제 호출 결과다.
  local verifier="${SCRIPT_DIR}/../scripts/verify_rt_runtime.sh"
  local n out
  for n in 4 6 8 10 12 16; do
    # 서브셸: 검증기는 top-level 에서 자기 상태(색상, 리포터, PHYSICAL_CORES)를
    # 세팅하므로 이 테스트 프로세스의 상태와 섞이면 안 된다.
    # `|| true`: 가드가 사라져 source 가 run_checks 를 돌리면 서브셸이 검증기의
    # exit code (2/3) 로 죽는다. set -e 아래서는 그게 이 스크립트 전체를
    # **요약 출력 전에** 끝내 버려서, 진짜 red 가 침묵한 red 로 보인다.
    # 여기서 삼키고 아래 populated 검사가 사람이 읽을 메시지를 내게 한다.
    out=$(
      set +eu
      # shellcheck disable=SC1090
      source "$verifier" >/dev/null 2>&1
      PHYSICAL_CORES="$n"
      build_external_drivers >/dev/null 2>&1
      printf '%s %s %s %s' \
        "${EXTERNAL_DRIVER_SLOTS[arm_driver]}" "$(get_arm_driver_slot "$n")" \
        "${EXTERNAL_DRIVER_SLOTS[hand_driver]}" "$(get_hand_driver_slot "$n")"
    ) || true
    local varm earm vhand ehand
    read -r varm earm vhand ehand <<<"$out"
    # source 나 build_external_drivers 가 죽으면 네 값이 전부 빈 문자열이 되고
    # expect_eq "" "" 는 **통과한다** — 비교 전에 실제로 슬롯을 받았는지 본다.
    if [[ "$varm" =~ ^[0-9]+$ && "$vhand" =~ ^[0-9]+$ && "$earm" =~ ^[0-9]+$ && "$ehand" =~ ^[0-9]+$ ]]; then
      pass
    else
      fail "[verifier.populated.${n}] 기대 slot 을 못 읽었다 (out='${out}') — source/build_external_drivers 실패"
      continue
    fi
    expect_eq "verifier.arm.${n}"  "$earm"  "$varm"
    expect_eq "verifier.hand.${n}" "$ehand" "$vhand"
  done
}

test_verifier_is_sourceable_without_running() {
  # 위 테스트가 성립하려면 source 가 검증을 *실행하지* 않아야 한다. 가드가
  # 사라지면 source 한 순간 run_checks 가 돌고 exit 로 테스트를 끌고 나간다.
  local verifier="${SCRIPT_DIR}/../scripts/verify_rt_runtime.sh"
  local sentinel=""
  sentinel=$(
    set +eu
    # shellcheck disable=SC1090
    source "$verifier" >/dev/null 2>&1
    echo "still-here"
  ) || true
  expect_eq "verifier.sourceable" "still-here" "$sentinel"
}

_verifier_tables() {
  # verify_rt_runtime.sh 를 source 해 (실행은 아래 main 가드가 막는다) 검증기가
  # 실제로 만드는 표 두 개를 꺼낸다. 소스 텍스트를 grep 하지 않는 이유: grep 은
  # 자기 주석을 읽을 뿐 배선이 살아 있는지 못 본다 — profile 을 안 넘기는 회귀는
  # 호출 라인이 그대로 남은 채 발생한다.
  #
  # `set --` 은 필수다. 인자 없는 `source` 는 **호출자의 위치인자를 물려주므로**,
  # 이걸 빼면 이 함수의 $1 이 검증기의 CLI 파서에 알 수 없는 옵션으로 들어가
  # show_help → exit 0 으로 서브셸이 조용히 죽고 표는 빈 채 비교된다.
  local marker_content="$1" ncpu="$2"
  shift 2
  local verifier="${SCRIPT_DIR}/../scripts/verify_rt_runtime.sh"
  local marker
  marker=$(mktemp)
  printf '%s\n' "$marker_content" >"$marker"
  (
    set +eu
    RTC_SHIELD_MARKER_FILE="$marker"
    set -- "$@"
    # shellcheck disable=SC1090
    source "$verifier" >/dev/null 2>&1
    PHYSICAL_CORES="$ncpu"
    build_expected_threads
    echo "profile=${LAYOUT_PROFILE}"
    echo "source=${PROFILE_SOURCE}"
    echo "expected=$(printf '%s,' "${EXPECTED_THREADS[@]}")"
    # 빈 배열에 printf 를 걸면 포맷이 한 번 찍혀 "," 가 남으므로 개수를 따로 낸다.
    echo "forbidden_n=${#FORBIDDEN_THREADS[@]}"
    echo "forbidden=$(printf '%s,' "${FORBIDDEN_THREADS[@]}")"
  )
  rm -f "$marker"
}

test_verifier_follows_the_shield_marker_profile() {
  # #350 — 검증기의 기본 profile 은 활성 shield 의 marker 다. 이 배선이 끊기면
  # MPC-off 로 띄운 박스에서 mpc_main 미발견이 하드 FAIL 로 돌아온다 (착수 전 상태).
  local out
  out=$(_verifier_tables "sim mpc_off" 12)
  expect_eq "verifier.profile.off" "profile=mpc_off" "$(grep '^profile=' <<<"$out")"
  expect_eq "verifier.source.off" "source=shield marker" "$(grep '^source=' <<<"$out")"
  if grep '^expected=' <<<"$out" | grep -q 'mpc_'; then
    fail "[verifier.exp.off] mpc_off 인데 기대 표에 mpc 행이 남아 있다 — 미발견이 FAIL 로 돌아온다"
  else
    pass
  fi
  expect_eq "verifier.forbidden.off" \
    "forbidden=mpc_main," "$(grep '^forbidden=' <<<"$out")"
  expect_eq "verifier.forbidden_n.off" "forbidden_n=1" "$(grep '^forbidden_n=' <<<"$out")"

  # 기본 profile 에서는 mpc 가 기대 표에 있고 금지 목록은 비어야 한다.
  out=$(_verifier_tables "robot mpc_on" 12)
  expect_eq "verifier.profile.on" "profile=mpc_on" "$(grep '^profile=' <<<"$out")"
  if grep '^expected=' <<<"$out" | grep -q 'mpc_main:3:1:60'; then
    pass
  else
    fail "[verifier.exp.on] mpc_on 인데 기대 표에 mpc_main 이 없다"
  fi
  expect_eq "verifier.forbidden.on" "forbidden_n=0" "$(grep '^forbidden_n=' <<<"$out")"

  # #350 이전 marker (bare mode) 는 default profile 로 읽혀야 한다 — 그 marker 가
  # 만들어진 shield 가 실제로 MPC 예약을 포함한 레이아웃이기 때문이다.
  out=$(_verifier_tables "robot" 12)
  expect_eq "verifier.profile.legacy" "profile=mpc_on" "$(grep '^profile=' <<<"$out")"

  # --profile 은 marker 를 이긴다 — shield 가 없는 박스(cset 미설치 dev PC)에서
  # 검증기를 돌리거나 반대 profile 을 일부러 확인할 때의 유일한 통로다.
  out=$(_verifier_tables "robot mpc_on" 12 --profile mpc_off)
  expect_eq "verifier.flag.wins" "profile=mpc_off" "$(grep '^profile=' <<<"$out")"
  expect_eq "verifier.flag.source" "source=--profile" "$(grep '^source=' <<<"$out")"
  out=$(_verifier_tables "robot mpc_on" 12 --profile=mpc_off)
  expect_eq "verifier.flag.eq_form" "profile=mpc_off" "$(grep '^profile=' <<<"$out")"
}

# ── check_rt_setup.sh::check_cpu_isolation (issue #386 A) ───────────────────
# 이동한 shield_isolation_method 의 **새 소비처**. 이동만 검증하면 반쪽이다 —
# 이 검사가 예전에 틀렸던 이유는 함수가 없어서가 아니라 아무도 안 불러서였고,
# 그 배선은 소스 grep 으로는 죽었는지 살았는지 구분되지 않는다 (#353 과 같은 축).
#
# 판정 두 축(method · 코어 일치)을 모두 스텁으로 가린다. 이 박스는 cset 도 없고
# /sys/.../isolated 도 비어 있어서, 안 가리면 네 케이스 중 "none" 하나만 관측된다.
_isolation_category() {
  # $1 = shield_isolation_method 가 낼 문자열, $2 = EXPECTED_ISOLATED
  local method_out="$1" expected="$2"
  local checker="${SCRIPT_DIR}/../scripts/check_rt_setup.sh"
  (
    set +eu
    # 인자 없는 source 는 호출자의 위치인자를 물려준다 — 검사기의 CLI 파서가
    # 그걸 unknown option 으로 받아 show_help → exit 0 으로 조용히 죽는다.
    set --
    # shellcheck disable=SC1090
    source "$checker" >/dev/null 2>&1
    shield_isolation_method() { echo "$method_out"; }
    EXPECTED_ISOLATED="$expected"
    OUTPUT_MODE="verbose"
    # 출력을 $(...) 로 받으면 안 된다 — 명령 치환은 서브셸이라 CATEGORY_* 갱신이
    # 호출자에게 안 돌아오고, 카테고리 단언이 전부 빈 문자열 비교로 공허해진다.
    local tmp
    tmp=$(mktemp)
    check_cpu_isolation >"$tmp" 2>&1
    echo "status=${CATEGORY_STATUS[cpu_isolation]:-}"
    echo "detail=${CATEGORY_DETAIL[cpu_isolation]:-}"
    echo "out=$(tr '\n' ' ' <"$tmp")"
    rm -f "$tmp"
  )
}

test_check_cpu_isolation_sees_a_live_cset_shield() {
  local out

  # 이 이슈의 본체: cset shield 가 기대값대로 서 있는데 "격리 미활성" 이 나오던
  # 자리다. /sys/.../isolated 는 여전히 비어 있다 (cset 은 거기 안 쓴다).
  out=$(_isolation_category "cset 1 2 3 7 8 9" "1-3,7-9")
  expect_eq "isolcheck.cset.status" "status=PASS" "$(grep '^status=' <<<"$out")"
  expect_eq "isolcheck.cset.detail" "detail=1-3,7-9 (cset)" "$(grep '^detail=' <<<"$out")"
  if grep -q 'cset shield' <<<"$out" && ! grep -q '격리 미활성' <<<"$out"; then
    pass
  else
    fail "[isolcheck.cset.msg] 살아 있는 cset shield 를 격리 미활성으로 보고했다: $out"
  fi

  # 표기가 다른 같은 집합 — 기대값은 범위 표기, method 출력은 정규화된 목록.
  # 문자열로 직접 비교하면 여기서 불일치가 난다.
  out=$(_isolation_category "cset 2 3 4 5 8 9 10 11" "2-5,8-11")
  expect_eq "isolcheck.cset.normalised" "status=PASS" "$(grep '^status=' <<<"$out")"

  # isolcpus 축은 PASS 하되 빌드 성능 경고 때문에 카테고리는 WARN 이다.
  out=$(_isolation_category "isolcpus 1-3,7-9" "1-3,7-9")
  expect_eq "isolcheck.isolcpus.status" "status=WARN" "$(grep '^status=' <<<"$out")"
  expect_eq "isolcheck.isolcpus.detail" "detail=1-3,7-9 (isolcpus)" "$(grep '^detail=' <<<"$out")"
  if grep -q 'isolcpus' <<<"$out"; then
    pass
  else
    fail "[isolcheck.isolcpus.msg] isolcpus 로 격리됐는데 그 이름이 안 나온다: $out"
  fi

  # 둘 다 없을 때만 "미활성" 이다.
  out=$(_isolation_category "none" "1-3,7-9")
  expect_eq "isolcheck.none.status" "status=WARN" "$(grep '^status=' <<<"$out")"
  expect_eq "isolcheck.none.detail" "detail=none (auto-activate on launch)" \
    "$(grep '^detail=' <<<"$out")"

  # 코어가 기대와 다르면 method 와 무관하게 불일치다. cset 축의 처방은 GRUB 이
  # 아니라 shield 재구성이어야 한다 — 옛 코드는 cset 상태에도 GRUB 을 안내했다.
  out=$(_isolation_category "cset 1 2 3" "1-3,7-9")
  expect_eq "isolcheck.mismatch.status" "status=WARN" "$(grep '^status=' <<<"$out")"
  if grep -q '불일치' <<<"$out"; then
    pass
  else
    fail "[isolcheck.mismatch.msg] 코어가 다른데 불일치 보고가 없다: $out"
  fi
}

# ── verify_rt_runtime.sh::check_process_discovery (issue #386 D) ────────────
# 이 함수는 pgrep 과 실 /proc/<pid>/task/ 를 직접 걸어서 테스트가 전무했다.
# #380 이 이 안의 PASS 게이트를 TID 축에서 이름 축으로 고쳤을 때 그 변경이 통과한
# 센서는 `bash -n` 과 source 뿐이었다 — 판정 로직 자체는 한 번도 실행되지 않았다.
#
# $RTC_PROC_ROOT 로 /proc 트리를, discover_controller_pid 스텁으로 pgrep 을 가린다
# (pgrep 은 실 커널 테이블을 보므로 fixture 로 대체되지 않는 유일한 입력이다).
#
# fixture 의 스레드 이름을 EXPECTED_THREADS 에서 파생시키는 것은 순환이 아니다 —
# 여기서 고정하는 것은 표의 *내용*이 아니라 **표를 훑는 판정**이고, 표 자체의
# oracle 은 손으로 박은 리터럴을 쓰는 test_rtc_expected_threads_table_per_tier 다.
_discovery_run() {
  # $1 = fixture 트리를 만드는 콜백 이름 (인자로 task_dir 을 받는다)
  # $2 = discover_controller_pid 가 낼 pid ("" 면 미발견)
  local builder="$1" pid="$2"
  local verifier="${SCRIPT_DIR}/../scripts/verify_rt_runtime.sh"
  local root
  root=$(mktemp -d)
  [[ -n "$pid" ]] && "$builder" "${root}/${pid}/task"
  (
    set +eu
    set --
    RTC_PROC_ROOT="$root"
    # external driver 탐색은 실 pgrep 을 쓰므로, 로봇이 떠 있는 박스에서 이 테스트가
    # 다른 답을 내지 않도록 존재할 수 없는 comm 으로 고정한다 (경로는 그대로 돈다).
    RTC_ARM_DRIVER_COMM="rtc_no_such_arm"
    RTC_HAND_DRIVER_COMM="rtc_no_such_hand"
    # shellcheck disable=SC1090
    source "$verifier" >/dev/null 2>&1
    discover_controller_pid() { echo "$pid"; }
    PHYSICAL_CORES=12
    LAYOUT_PROFILE="mpc_on"
    build_expected_threads
    OUTPUT_MODE="verbose"
    # 명령 치환은 서브셸이라 CATEGORY_* / 종료코드를 함께 관측할 수 없다.
    local tmp rc
    tmp=$(mktemp)
    check_process_discovery >"$tmp" 2>&1
    rc=$?
    echo "rc=${rc}"
    echo "status=${CATEGORY_STATUS[process_discovery]:-}"
    echo "detail=${CATEGORY_DETAIL[process_discovery]:-}"
    echo "out=$(tr '\n' ' ' <"$tmp")"
    rm -f "$tmp"
  )
  rm -rf "$root"
}

# 기대 표의 모든 이름에 tid 를 하나씩 준다 (정상 기동).
_fixture_all_threads() {
  local task_dir="$1" tid=1000 name
  while read -r name; do
    mkdir -p "${task_dir}/${tid}"
    printf '%s\n' "$name" >"${task_dir}/${tid}/comm"
    tid=$((tid + 1))
  done < <(rtc_expected_threads 12 mpc_on | cut -d: -f1)
}

# 필수 이름 하나(nrt_publish)를 빼고, 그 자리를 **같은 comm 의 중복**으로 채운다.
# 이러면 매칭 TID 수(known_count)는 표의 행 수에 도달하는데 고유 이름은 모자란다
# — TID 축으로 PASS 를 판정하면 "6/6 전체 감지" 를 내면서 nrt_publish 가 아예
# 안 떠 있는 상태가 통과한다. #380 이 고친 fail-open 이 정확히 이것이다.
_fixture_duplicate_comm() {
  local task_dir="$1" tid=1000 name
  while read -r name; do
    [[ "$name" == "nrt_publish" ]] && continue
    mkdir -p "${task_dir}/${tid}"
    printf '%s\n' "$name" >"${task_dir}/${tid}/comm"
    tid=$((tid + 1))
  done < <(rtc_expected_threads 12 mpc_on | cut -d: -f1)
  mkdir -p "${task_dir}/${tid}"
  printf '%s\n' "rt_control" >"${task_dir}/${tid}/comm"
}

_fixture_no_task_dir() { :; }

test_check_process_discovery_judges_by_name() {
  local out

  # ① 전원 기동 — 이름 축으로 6/6.
  out=$(_discovery_run _fixture_all_threads 4242)
  expect_eq "discovery.ok.rc" "rc=0" "$(grep '^rc=' <<<"$out")"
  expect_eq "discovery.ok.status" "status=PASS" "$(grep '^status=' <<<"$out")"
  if grep -q '6/6' <<<"$out"; then pass; else
    fail "[discovery.ok.msg] 전원 기동인데 6/6 이 아니다: $out"
  fi

  # ② 중복 comm + 필수 1개 누락 → **FAIL 이어야 한다** (fail-open 회귀 고정).
  out=$(_discovery_run _fixture_duplicate_comm 4242)
  expect_eq "discovery.dup.status" "status=FAIL" "$(grep '^status=' <<<"$out")"
  if grep -q 'nrt_publish' <<<"$out"; then pass; else
    fail "[discovery.dup.missing] 누락된 필수 스레드 이름이 보고되지 않았다: $out"
  fi
  if grep -q '같은 이름의 스레드가 여럿' <<<"$out"; then pass; else
    fail "[discovery.dup.warn] 두 축이 갈렸는데 판정 축 로그가 없다: $out"
  fi
  if grep -q '6/6' <<<"$out"; then
    fail "[discovery.dup.failopen] 중복 TID 로 6/6 을 냈다 — TID 축 판정 회귀: $out"
  else
    pass
  fi

  # ③ task dir 부재 (프로세스가 방금 죽음) → FAIL, rc 1.
  out=$(_discovery_run _fixture_no_task_dir 4242)
  expect_eq "discovery.notask.rc" "rc=1" "$(grep '^rc=' <<<"$out")"
  expect_eq "discovery.notask.status" "status=FAIL" "$(grep '^status=' <<<"$out")"
  expect_eq "discovery.notask.detail" "detail=PID 4242, no task dir" \
    "$(grep '^detail=' <<<"$out")"

  # ④ controller 미발견 → FAIL, rc 1. 미검증은 PASS 가 아니다.
  out=$(_discovery_run _fixture_all_threads "")
  expect_eq "discovery.nopid.rc" "rc=1" "$(grep '^rc=' <<<"$out")"
  expect_eq "discovery.nopid.status" "status=FAIL" "$(grep '^status=' <<<"$out")"
  expect_eq "discovery.nopid.detail" "detail=not running" "$(grep '^detail=' <<<"$out")"
}

# ── Layout tier tables (issue #153 M1) ──────────────────────────────────────
# get_mpc_cores / get_rt_cores / get_nrt_cores 는 이제 manifest 에서 생성되지만,
# **이 fixture 의 리터럴이 생성기의 oracle 이다**. 생성물끼리 대조하면 잘못된
# 표가 세 언어에서 사이좋게 일치하므로 mutation 이 없다 — 그래서 기대값을 손으로
# 박는다. M1 전에는 이 세 함수에 직접 단위 테스트가 아예 없었다 (파생 함수인
# get_cm_shield_cpus / get_rt_cores_with_siblings 만 덮여 있었다).

test_layout_tier_tables_per_tier() {
  local n
  # ≤5 (degraded) — nrt 가 OS Core 0 을 공유한다.
  for n in 1 4 5; do
    expect_eq "mpc.${n}" "3"     "$(get_mpc_cores "$n")"
    expect_eq "rt.${n}"  "1,2,3" "$(get_rt_cores "$n")"
    expect_eq "nrt.${n}" "0"     "$(get_nrt_cores "$n")"
  done
  # 6-7 — v5 부터 nrt 3 레인은 aux slot 2 (rt_callback 슬롯) 위다.
  for n in 6 7; do
    expect_eq "mpc.${n}" "3"     "$(get_mpc_cores "$n")"
    expect_eq "rt.${n}"  "1,2,3" "$(get_rt_cores "$n")"
    expect_eq "nrt.${n}" "2"     "$(get_nrt_cores "$n")"
  done
  # 8-9 — arm/hand 전용 슬롯.
  for n in 8 9; do
    expect_eq "mpc.${n}" "3"     "$(get_mpc_cores "$n")"
    expect_eq "rt.${n}"  "1,2,3" "$(get_rt_cores "$n")"
    expect_eq "nrt.${n}" "2"     "$(get_nrt_cores "$n")"
  done
  # 10+ — #380 이 mpc_worker 슬롯을 회수한 뒤로 RT 집합은 tier 와 무관하게
  # rt_control + rt_callback + mpc_main 셋이다. 이 행들이 다시 넓어지면 shield 가
  # 아무도 안 쓰는 코어를 격리한다는 뜻이므로 tier 마다 따로 박는다.
  for n in 10 11 12 13 14 15 16 17 24; do
    expect_eq "mpc.${n}" "3"     "$(get_mpc_cores "$n")"
    expect_eq "rt.${n}"  "1,2,3" "$(get_rt_cores "$n")"
    expect_eq "nrt.${n}" "2"     "$(get_nrt_cores "$n")"
  done
  # v5 는 nrt 를 RT 집합의 부분집합으로 만든다 (aux slot = rt_callback slot).
  # 이 관계가 깨지면 get_cm_shield_cpus 의 union 이 다시 넓어지므로 함께 박는다.
  for n in 6 8 10 12 16 24; do
    expect_eq "nrt_subset_of_rt.${n}" "2" "$(get_role_slot rt_callback "$n")"
  done
  # OS slot 은 모든 tier 공통 (v4.1) — shield 가 절대 덮으면 안 되는 코어.
  for n in 1 4 6 8 10 12 16 24; do
    expect_eq "os.${n}" "0" "$(get_os_cores)"
  done
  # get_mpc_main_core 는 파생이므로 첫 항목과 항상 같아야 한다.
  for n in 4 8 12 16; do
    expect_eq "mpcmain.${n}" "3" "$(get_mpc_main_core "$n")"
  done
}

test_layout_worker_roles_absent_on_every_tier() {
  # #380 이 mpc_worker 를 manifest 에서 지웠다. 이 테스트는 그 전에 "worker 는 10
  # 코어부터" 를 박던 자리이며, 지금은 **어느 tier 에서도 존재하지 않음**을 박는다.
  # get_role_spec 이 다시 성공하면 누군가 manifest 에 슬롯을 되살렸다는 뜻이고,
  # rtc_expected_threads 가 영영 안 나타날 스레드를 검증기에 넘긴다 — worker 가
  # 죽은 채 예약만 남아 있던 것이 #380 의 결함 그 자체다.
  local n
  for n in 4 6 8 9 10 11 12 16 24; do
    if get_role_spec mpc_worker_0 "$n" >/dev/null 2>&1; then
      fail "[worker.absent.${n}] mpc_worker_0 이 ${n}-core tier 에 존재한다고 보고 — #380 이 제거했다"
    else
      pass
    fi
    if get_role_spec mpc_worker_1 "$n" >/dev/null 2>&1; then
      fail "[worker.absent1.${n}] mpc_worker_1 이 ${n}-core tier 에 존재한다고 보고 — #380 이 제거했다"
    else
      pass
    fi
  done
}

test_rtc_expected_threads_table_per_tier() {
  # verify_rt_runtime.sh 의 기대 표. 행 수·policy·priority 를 tier 마다 박는다 —
  # 이 표가 조용히 틀리면 검증기가 **PASS 를 내며** 잘못 배치된 스레드를 통과시킨다.
  expect_eq "exp.4" \
    "rt_control:1:1:90 rt_callback:2:1:70 mpc_main:3:0:0 nrt_logging:0:0:0 nrt_callback:0:0:0 nrt_publish:0:0:0" \
    "$(rtc_expected_threads 4 | tr '\n' ' ' | sed 's/ $//')"
  expect_eq "exp.6" \
    "rt_control:1:1:90 rt_callback:2:1:70 mpc_main:3:1:60 nrt_logging:2:0:0 nrt_callback:2:0:0 nrt_publish:2:0:0" \
    "$(rtc_expected_threads 6 | tr '\n' ' ' | sed 's/ $//')"
  expect_eq "exp.8" \
    "rt_control:1:1:90 rt_callback:2:1:70 mpc_main:3:1:60 nrt_logging:2:0:0 nrt_callback:2:0:0 nrt_publish:2:0:0" \
    "$(rtc_expected_threads 8 | tr '\n' ' ' | sed 's/ $//')"
  expect_eq "exp.10" \
    "rt_control:1:1:90 rt_callback:2:1:70 mpc_main:3:1:60 nrt_logging:2:0:0 nrt_callback:2:0:0 nrt_publish:2:0:0" \
    "$(rtc_expected_threads 10 | tr '\n' ' ' | sed 's/ $//')"
  expect_eq "exp.12" \
    "rt_control:1:1:90 rt_callback:2:1:70 mpc_main:3:1:60 nrt_logging:2:0:0 nrt_callback:2:0:0 nrt_publish:2:0:0" \
    "$(rtc_expected_threads 12 | tr '\n' ' ' | sed 's/ $//')"
  # v5 이후 nrt 3행이 전부 slot 2 로 수렴하므로, 이 표만으로는 "세 행이 각각
  # 존재하는가" 가 슬롯 축에서 무뎌진다. 행 수를 따로 박아 verifier 가 세 TID 를
  # 모두 요구한다는 사실을 지킨다 (#349 D15 — 이름이 갈려야 각 행이 산다).
  local n
  for n in 6 8 10 12 16; do
    expect_eq "exp.nrt_rows.${n}" "3" \
      "$(rtc_expected_threads "$n" | grep -c '^nrt_')"
  done
  # 4-core 만 mpc 가 CFS 로 강등된다 (policy 필드 0). 그 구분이 사라지면 검증기가
  # degraded 박스에서 존재하지 않는 FIFO 60 을 요구한다.
  expect_eq "exp.mpc_policy.4"  "0" "$(rtc_expected_threads 4 | grep '^mpc_main:' | cut -d: -f3)"
  expect_eq "exp.mpc_policy.16" "1" "$(rtc_expected_threads 16 | grep '^mpc_main:' | cut -d: -f3)"
  # arm/hand 는 별도 프로세스라 이 표에 있으면 안 된다 (있으면 항상 false-WARN).
  for n in 4 8 12 16; do
    if rtc_expected_threads "$n" | grep -qE '^(arm|hand)_driver:'; then
      fail "[exp.external.${n}] arm/hand_driver 가 in-process 기대 표에 있다 — 항상 false-WARN 이다"
    else
      pass
    fi
  done
}

test_layout_profile_mpc_off_frees_the_mpc_slots() {
  # #350 — profile 축의 oracle. 손으로 박는 이유는 위 tier 표와 같다: 생성물끼리
  # 대조하면 잘못된 표가 세 언어에서 사이좋게 일치하므로 mutation 이 없다.
  local n
  # MPC 를 끄면 RT 집합은 rt_control + rt_callback 만 남는다 — 전 tier 공통이다
  # (tier 마다 반환하는 슬롯 수는 다르지만 남는 집합은 같다).
  for n in 1 4 5 6 7 8 9 10 11 12 13 14 15 16 17 24; do
    expect_eq "rt.off.${n}"  "1,2" "$(get_rt_cores "$n" mpc_off)"
    expect_eq "mpc.off.${n}" ""    "$(get_mpc_cores "$n" mpc_off)"
  done
  # mpc_on 은 default 이므로 무인자 호출과 반드시 같다. 이게 갈리면 profile 을
  # 모르는 기존 호출자 (setup_grub_rt.sh, check_rt_setup.sh, IRQ affinity) 가
  # 조용히 다른 레이아웃을 받는다 — #350 이 절대 건드리지 않기로 한 축이다.
  for n in 4 6 8 10 12 16 24; do
    expect_eq "rt.on_is_default.${n}"  "$(get_rt_cores "$n")"  "$(get_rt_cores "$n" mpc_on)"
    expect_eq "mpc.on_is_default.${n}" "$(get_mpc_cores "$n")" "$(get_mpc_cores "$n" mpc_on)"
  done
  # profile 은 MPC role 만 떨어뜨린다. nrt / arm / hand 는 profile 축을 아예 안
  # 받으므로 $2 를 줘도 무시해야 한다 — 이들이 함께 움직이면 shield union 이
  # 엉뚱하게 좁아져 그 스레드들의 self-pin 이 EINVAL 로 죽는다 (#151).
  for n in 6 8 10 12 16; do
    expect_eq "nrt.profile_blind.${n}"  "$(get_nrt_cores "$n")"  "$(get_nrt_cores "$n" mpc_off)"
    expect_eq "arm.profile_blind.${n}" \
      "$(get_arm_driver_slot "$n")" "$(get_arm_driver_slot "$n" mpc_off)"
    expect_eq "hand.profile_blind.${n}" \
      "$(get_hand_driver_slot "$n")" "$(get_hand_driver_slot "$n" mpc_off)"
  done
  # 알 수 없는 profile 은 조용히 default 로 떨어지면 안 된다 — 오타 하나가
  # "MPC 껐다고 생각했는데 코어는 그대로 예약" 을 만든다.
  local out
  for out in get_rt_cores get_mpc_cores rtc_expected_threads rtc_forbidden_threads; do
    if "$out" 12 mpc_offf >/dev/null 2>&1; then
      fail "[profile.unknown.${out}] 알 수 없는 profile 을 받아들였다 — 비0 이어야 한다"
    else
      pass
    fi
  done
  if rtc_is_layout_profile mpc_off && rtc_is_layout_profile mpc_on &&
    ! rtc_is_layout_profile mpc_offf; then
    pass
  else
    fail "[profile.predicate] rtc_is_layout_profile 이 선언된 profile 집합과 어긋난다"
  fi
  expect_eq "profile.default" "mpc_on" "$(rtc_default_profile)"

  # D11(a) — GRUB (nohz_full / rcu_nocbs) 는 boot-static 이므로 profile 을 타면
  # 안 된다. 인자를 줘도 무시하고 항상 가장 넓은 집합이어야 재부팅 없이 profile
  # 전환이 가능하다. 이 등식이 깨지는 순간 커널 커맨드라인과 런타임 레이아웃의
  # 계약이 갈라진다.
  expect_eq "grub.profile_blind" \
    "$(get_rt_cores_with_siblings)" "$(get_rt_cores_with_siblings mpc_off)"
}

test_layout_profile_verifier_tables_partition() {
  # off 에서 mpc 행은 기대 표에서 빠지고 forbidden 표로 옮겨간다. 한쪽만 하면
  # "MPC 꺼짐" 과 "MPC 가 방금 반환한 코어에서 돌고 있음" 이 검증기에게 똑같이
  # 보인다 — 둘 다 그냥 행이 없는 상태이기 때문이다.
  local n
  for n in 4 6 8 10 12 16; do
    if rtc_expected_threads "$n" mpc_off | grep -q '^mpc_'; then
      fail "[profile.exp.${n}] mpc_off 기대 표에 mpc 행이 남아 있다"
    else
      pass
    fi
    # RT 두 레인과 nrt 3 레인은 profile 과 무관하게 그대로 기대된다.
    expect_eq "profile.exp.rows.${n}" \
      "$(rtc_expected_threads "$n" | grep -cvE '^mpc_')" \
      "$(rtc_expected_threads "$n" mpc_off | grep -c .)"
    # 반대로 on 에서는 금지 목록이 비어야 한다 — 아무 role 도 안 떨어뜨린다.
    if [[ -z "$(rtc_forbidden_threads "$n" mpc_on)" ]]; then
      pass
    else
      fail "[profile.forbidden.on.${n}] default profile 인데 금지 스레드가 있다"
    fi
  done
  # off 금지 목록은 그 tier 에 실제로 존재하는 mpc role 전부다. #380 이 worker 를
  # 지운 뒤로 그것은 모든 tier 에서 mpc_main 하나이며, 여기에 다시 이름이 붙으면
  # 검증기가 영영 안 나올 스레드를 금지 대상으로 든다.
  local n
  for n in 4 8 10 12 16; do
    expect_eq "profile.forbidden.${n}" "mpc_main" \
      "$(rtc_forbidden_threads "$n" mpc_off | tr '\n' ' ' | sed 's/ $//')"
  done
}

test_rtc_expected_threads_tracks_the_role_helpers() {
  # 표의 각 행이 get_role_* 와 같은 값을 내는지 — 배선 확인. 위 fixture 가 값을
  # 소유하고, 이건 표가 같은 manifest 를 읽는지를 본다.
  local n row name slot policy prio
  for n in 4 6 8 10 12 16; do
    while IFS=: read -r name slot policy prio _; do
      expect_eq "exp.wire.${n}.${name}.slot" "$(get_role_slot "$name" "$n")" "$slot"
      expect_eq "exp.wire.${n}.${name}.prio" "$(get_role_priority "$name" "$n")" "$prio"
      if [[ "$(get_role_policy "$name" "$n")" == "SCHED_FIFO" ]]; then
        expect_eq "exp.wire.${n}.${name}.pol" "1" "$policy"
      else
        expect_eq "exp.wire.${n}.${name}.pol" "0" "$policy"
      fi
    done < <(rtc_expected_threads "$n")
  done
}

test_print_thread_layout_derives_every_core_number() {
  # #353 은 arm/hand 축에서만 이 배선을 박았다. M1 이 나머지 축(rt_control,
  # rt_callback, mpc, nrt)도 헬퍼에서 끌어오게 했으므로 그 전부를 확인한다 —
  # 다이어그램은 운영자가 박스 상태를 판단하는 화면이라 조용히 낡으면 비싸다.
  local out n
  for n in 6 8 10 12 16; do
    out=$(print_thread_layout "$n")
    if grep -qE "Core $(get_role_slot rt_control "$n"): +rt_control" <<<"$out"; then
      pass
    else
      fail "[layout.rt_control.${n}] rt_control 줄이 헬퍼 slot 을 안 쓴다"
    fi
    if grep -qE "Core $(get_role_slot rt_callback "$n"): +rt_callback" <<<"$out"; then
      pass
    else
      fail "[layout.rt_callback.${n}] rt_callback 줄이 헬퍼 slot 을 안 쓴다"
    fi
    if grep -qE "Core $(get_role_slot nrt_logging "$n")[:-]" <<<"$out"; then
      pass
    else
      fail "[layout.nrt_logging.${n}] nrt_logging slot 이 다이어그램에 없다"
    fi
    # nrt_publish 는 자기 comm 이름을 가진 별개 스레드다 (#349 D15). 역할
    # 목록은 손으로 쓰는 부분이라 여기 빠지면 다이어그램은 코어에 스레드
    # 하나를 보여주는데 verify_rt_runtime.sh 는 둘을 기대한다 — 운영자가
    # co-residency 를 진단하는 바로 그 화면에서 갈린다.
    if grep -qE "Core $(get_role_slot nrt_publish "$n")[:-].*nrt_publish" <<<"$out"; then
      pass
    else
      fail "[layout.nrt_publish.${n}] nrt_publish 가 자기 slot 행에 없다"
    fi
    # MPC lane 은 #380 이후 모든 tier 에서 단일 코어라 "Core 3" 으로 그려진다.
    # 범위 분기는 남겨 둔다 — manifest 가 다시 여러 슬롯을 주면 그때 발화한다.
    local first last
    first=$(get_mpc_cores "$n" | cut -d',' -f1)
    last=$(get_mpc_cores "$n" | tr ',' '\n' | tail -1)
    if [[ "$first" == "$last" ]]; then
      grep -qE "Core ${first}: +mpc_main" <<<"$out" && pass ||
        fail "[layout.mpc.${n}] 'Core ${first}: mpc_main' 줄이 없다"
    else
      grep -qE "Core ${first}-${last}: +mpc_main" <<<"$out" && pass ||
        fail "[layout.mpc.${n}] 'Core ${first}-${last}' MPC lane 이 없다"
    fi
    # 존재하지 않는 코어를 spare 로 그리면 안 된다 (ncpu-1 이 마지막 슬롯).
    if grep -qE "Core ${n}[:-]" <<<"$out"; then
      fail "[layout.spare.${n}] ${n}-core 박스인데 Core ${n} 을 그렸다 — 슬롯은 0..$((n - 1))"
    else
      pass
    fi
  done
}

test_external_driver_slots_per_tier
test_external_driver_slots_share_a_core_only_on_the_degraded_tiers
test_print_thread_layout_uses_the_slot_helpers
test_verifier_expected_slots_track_the_helpers
test_verifier_is_sourceable_without_running
test_verifier_follows_the_shield_marker_profile
test_check_cpu_isolation_sees_a_live_cset_shield
test_check_process_discovery_judges_by_name
test_layout_tier_tables_per_tier
test_layout_worker_roles_absent_on_every_tier
test_rtc_expected_threads_table_per_tier
test_layout_profile_mpc_off_frees_the_mpc_slots
test_layout_profile_verifier_tables_partition
test_rtc_expected_threads_tracks_the_role_helpers
test_print_thread_layout_derives_every_core_number

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
