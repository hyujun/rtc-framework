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

# ── Test 17: sanity-check hook fires a warning on disagreement ─────────────
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
test_sanity_check_disagreement

echo
echo "── test_rt_common.sh summary ──"
echo "  PASS: $PASS"
echo "  FAIL: $FAIL"
if (( FAIL > 0 )); then
  printf '  %s\n' "${FAIL_MSGS[@]}"
  exit 1
fi
exit 0
