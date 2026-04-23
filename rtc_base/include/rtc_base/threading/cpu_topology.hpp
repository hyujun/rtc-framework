#ifndef RTC_BASE_CPU_TOPOLOGY_HPP_
#define RTC_BASE_CPU_TOPOLOGY_HPP_

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <unistd.h>
#include <vector>

// CPU topology detection for hybrid-aware thread placement.
//
// Stage A introduces the detection layer but does NOT yet consume it —
// `SelectThreadConfigs()` still dispatches on `GetPhysicalCpuCount()`.
// The types and detector here become load-bearing in Stage B when the
// NUC-specific layouts land.
//
// Detection strategy (in priority order):
//   1. sysfs `/sys/devices/system/cpu/types/{intel_core,intel_atom}/cpus`
//      — kernel >=5.18 exposes Intel hybrid classification here.
//   2. `/proc/cpuinfo` flag `hybrid` — kernel ≥5.14 Alder-Lake+ indicator.
//   3. Fallback: treat as NOT_NUC_HYBRID (AMD, older Intel, containers
//      without sysfs access).
//
// Generation classification uses the (p_core_has_smt, has_lp_e_cores) pair:
//   (true,  false) → RAPTOR_LAKE_P      (NUC 13 Pro)
//   (true,  true ) → METEOR_LAKE        (NUC 14 Pro)
//   (false, true ) → ARROW_LAKE_H       (NUC 15 Pro+)
//   (false, false) with is_hybrid=1 → RAPTOR_LAKE_P_HT_OFF (BIOS anomaly)
//   anything else → NOT_NUC_HYBRID
//
// Header-only to match rtc_base convention. Non-RT — callers must prewarm
// `GetCpuTopology()` from `on_configure()` before any RT thread starts.

namespace rtc {

enum class NucGeneration {
  NOT_NUC_HYBRID = 0,
  RAPTOR_LAKE_P,        // NUC 13 Pro: 4P+8E or 6P+8E, P-core SMT on, no LP-E
  METEOR_LAKE,          // NUC 14 Pro: P-core SMT on, with LP-E tile
  ARROW_LAKE_H,         // NUC 15 Pro+: P-core SMT off, with LP-E tile
  RAPTOR_LAKE_P_HT_OFF, // Intel hybrid detected but P-core SMT missing —
                        // almost always BIOS has disabled Hyper-Threading.
                        // Stage A surfaces this as FAIL in check_rt_setup.sh.
};

// Aligator fallback when P-core worker budget is insufficient. Phase 1
// defines the enum; Stage C wires the Aligator runner switch. Phase 1 tier
// (NUC 13 4P minimum) never produces SERIAL_MPC.
enum class DegradationMode {
  NONE = 0,
  SERIAL_MPC,
};

// Which detection path populated the hybrid fields. Surfaced so callers
// (check_rt_setup.sh, diagnostic logging) can warn when the primary sysfs
// topology is absent — usually a signal that the kernel build is missing
// CONFIG_INTEL_HFI_THERMAL / SCHED_MC_PRIO / SCHED_CLUSTER.
enum class HybridDetectSource {
  NONE = 0,    // Not hybrid (or no CPU at all).
  SYSFS_TYPES, // /sys/.../cpu/types/intel_{core,atom}   (+cpuinfo hybrid flag)
  CPUFREQ_CLUSTER, // cpuinfo_max_freq heterogeneity clustering.
};

struct CpuTopology {
  // Aggregate counts (valid on every CPU).
  int num_physical_cores{0};
  int num_logical_cores{0};
  bool smt_active{false};

  // Hybrid-specific fields. On NOT_NUC_HYBRID, num_p_physical ==
  // num_physical_cores and the id lists are empty.
  bool is_hybrid{false};
  bool p_core_has_smt{false};
  bool has_lp_e_cores{false};
  int num_p_physical{0};
  int num_p_logical{0};
  int num_e_cores{0};
  int num_lpe_cores{0};

  // Logical CPU id lists. Invariant (when p_core_has_smt == true):
  //   For each i: p_core_physical_ids[i] and p_core_sibling_ids[i] belong
  //   to the same physical core (identical core_id in sysfs topology).
  //   Verified by test TEST(CpuTopology, SmtSiblingInvariant).
  std::vector<int> p_core_physical_ids;
  std::vector<int> p_core_sibling_ids;
  std::vector<int> e_core_ids;
  std::vector<int> lpe_core_ids;

  NucGeneration generation{NucGeneration::NOT_NUC_HYBRID};

  // Which detection path populated the hybrid fields (NONE when not hybrid).
  // Used by callers to warn when the primary sysfs path was unavailable and
  // a fallback had to classify the CPU.
  HybridDetectSource detect_source{HybridDetectSource::NONE};

  // Tier-selection metric for future hybrid dispatch. Returns the count of
  // "high-performance" physical cores — P-core count on hybrid, plain
  // physical count otherwise.
  [[nodiscard]] int num_effective_cores() const noexcept {
    return is_hybrid ? num_p_physical : num_physical_cores;
  }
};

// ── Internal helpers (parsing, file IO) ───────────────────────────────────
namespace internal::topology {

inline std::string ReadFileTrim(const std::filesystem::path &p) noexcept {
  std::error_code ec;
  if (!std::filesystem::exists(p, ec))
    return {};
  std::ifstream f(p);
  if (!f)
    return {};
  std::string s;
  std::getline(f, s);
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' ||
                        s.back() == ' ' || s.back() == '\t')) {
    s.pop_back();
  }
  return s;
}

inline int ReadInt(const std::filesystem::path &p, int fallback = -1) noexcept {
  const auto s = ReadFileTrim(p);
  if (s.empty())
    return fallback;
  try {
    return std::stoi(s);
  } catch (...) {
    return fallback;
  }
}

// Parse sysfs cpulist strings: "0-7,12-15" or "0,2,4".
inline std::vector<int> ParseCpuList(std::string_view s) noexcept {
  std::vector<int> out;
  std::size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && (s[i] == ',' || s[i] == ' ' || s[i] == '\t'))
      ++i;
    if (i >= s.size())
      break;
    int a = 0;
    bool has = false;
    while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) {
      a = a * 10 + (s[i] - '0');
      ++i;
      has = true;
    }
    if (!has)
      break;
    if (i < s.size() && s[i] == '-') {
      ++i;
      int b = 0;
      while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) {
        b = b * 10 + (s[i] - '0');
        ++i;
      }
      for (int v = a; v <= b; ++v)
        out.push_back(v);
    } else {
      out.push_back(a);
    }
  }
  return out;
}

// Try `<dir>/cpus` first (older kernels), then `<dir>/cpulist`. Either
// form may be present depending on kernel version.
inline std::vector<int>
ReadCpuListFile(const std::filesystem::path &dir) noexcept {
  auto s = ReadFileTrim(dir / "cpus");
  if (!s.empty())
    return ParseCpuList(s);
  s = ReadFileTrim(dir / "cpulist");
  if (!s.empty())
    return ParseCpuList(s);
  return {};
}

inline bool
CpuinfoHasHybridFlag(const std::filesystem::path &cpuinfo) noexcept {
  std::ifstream f(cpuinfo);
  if (!f)
    return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("flags", 0) != 0)
      continue;
    // Match " hybrid" or "\thybrid" (whole word). Simple substring is
    // sufficient since "hybrid" is not a substring of any other known flag.
    if (line.find(" hybrid") != std::string::npos ||
        line.find("\thybrid") != std::string::npos) {
      return true;
    }
  }
  return false;
}

// Frequency-clustering fallback. Returns (p_cpus, e_cpus) when the CPU
// set splits into two frequency tiers — highest-tier ids go to p_cpus,
// the rest to e_cpus. Returns empty vectors if the heterogeneity is too
// small (< kSpreadPct) or the data is unavailable. Zero-dependency: reads
// cpuinfo_max_freq from each cpuN directory that the caller enumerated.
//
// Parameters tuned for Intel hybrid SoCs:
//   Meteor Lake Core Ultra 9 185H   — P≈5.1 GHz  E≈3.8 GHz  LP-E≈2.5 GHz
//   Raptor Lake-P i7-1360P          — P≈5.0 GHz  E≈3.7 GHz  (no LP-E)
//   Arrow Lake-H Core Ultra 9 285H  — P≈5.4 GHz  E≈4.5 GHz  LP-E≈2.5 GHz
// Homogeneous silicon (AMD Zen 4/5, Alder-Lake-N all-E, SKUs without SMT
// heterogeneity) typically reports < 3 % spread and is rejected by the
// 15 % threshold.
inline std::pair<std::vector<int>, std::vector<int>>
ClusterByMaxFreq(const std::filesystem::path &cpu_root,
                 const std::vector<int> &online_cpus) noexcept {
  constexpr int kSpreadPct = 15;         // min (max-min)/max% to call hybrid
  constexpr int kPCoreThresholdPct = 85; // P-core: f >= max * 85/100
  std::pair<std::vector<int>, std::vector<int>> empty;
  if (online_cpus.empty())
    return empty;

  std::vector<std::pair<int, int>> freqs; // (cpu, max_khz)
  freqs.reserve(online_cpus.size());
  int max_f = 0;
  int min_f = 0;
  for (const int cpu : online_cpus) {
    const int f = ReadInt(cpu_root / ("cpu" + std::to_string(cpu)) /
                          "cpufreq/cpuinfo_max_freq");
    if (f <= 0)
      return empty; // Any missing reading disqualifies the fallback.
    freqs.emplace_back(cpu, f);
    if (f > max_f)
      max_f = f;
    if (min_f == 0 || f < min_f)
      min_f = f;
  }
  if (max_f <= 0)
    return empty;
  const int spread_pct = (max_f - min_f) * 100 / max_f;
  if (spread_pct < kSpreadPct)
    return empty;

  const int p_threshold = max_f * kPCoreThresholdPct / 100;
  std::vector<int> p_cpus, e_cpus;
  for (const auto &[cpu, f] : freqs) {
    if (f >= p_threshold)
      p_cpus.push_back(cpu);
    else
      e_cpus.push_back(cpu);
  }
  // Must have both tiers — otherwise homogeneous (and the spread check
  // above should already have rejected, but belt-and-suspenders).
  if (p_cpus.empty() || e_cpus.empty())
    return empty;
  return {std::move(p_cpus), std::move(e_cpus)};
}

// Derive P-core SMT pairing, E/LP-E split, and all hybrid counters from a
// (p_cpus, e_cpus) split. Shared between the sysfs-primary and freq-fallback
// paths so their outputs are identical shape.
inline void PopulateHybridFromCpus(CpuTopology &t,
                                   const std::filesystem::path &cpu_root,
                                   const std::vector<int> &p_cpus,
                                   const std::vector<int> &e_cpus) noexcept {
  if (!p_cpus.empty()) {
    std::map<int, std::vector<int>> p_core_id_to_cpus;
    for (const int cpu : p_cpus) {
      const int cid = ReadInt(cpu_root / ("cpu" + std::to_string(cpu)) /
                              "topology/core_id");
      if (cid >= 0)
        p_core_id_to_cpus[cid].push_back(cpu);
    }
    for (auto &[cid, cpus] : p_core_id_to_cpus) {
      std::sort(cpus.begin(), cpus.end());
      t.p_core_physical_ids.push_back(cpus.front());
      if (cpus.size() >= 2)
        t.p_core_sibling_ids.push_back(cpus[1]);
    }
    t.num_p_physical = static_cast<int>(p_core_id_to_cpus.size());
    t.num_p_logical = static_cast<int>(p_cpus.size());
    t.p_core_has_smt =
        (t.num_p_logical > t.num_p_physical) &&
        (t.p_core_sibling_ids.size() == t.p_core_physical_ids.size());
  }

  if (!e_cpus.empty()) {
    std::vector<std::pair<int, int>> e_freq;
    e_freq.reserve(e_cpus.size());
    int max_freq = 0;
    for (const int cpu : e_cpus) {
      const int f = ReadInt(cpu_root / ("cpu" + std::to_string(cpu)) /
                            "cpufreq/cpuinfo_max_freq");
      e_freq.emplace_back(cpu, f);
      if (f > max_freq)
        max_freq = f;
    }
    const int lpe_threshold = max_freq > 0 ? (max_freq * 70 / 100) : 0;
    for (const auto &[cpu, f] : e_freq) {
      if (lpe_threshold > 0 && f > 0 && f < lpe_threshold)
        t.lpe_core_ids.push_back(cpu);
      else
        t.e_core_ids.push_back(cpu);
    }
    t.num_e_cores = static_cast<int>(t.e_core_ids.size());
    t.num_lpe_cores = static_cast<int>(t.lpe_core_ids.size());
    t.has_lp_e_cores = (t.num_lpe_cores > 0);
  }
}

inline NucGeneration ClassifyGeneration(bool is_hybrid, bool p_core_has_smt,
                                        bool has_lp_e_cores) noexcept {
  if (!is_hybrid)
    return NucGeneration::NOT_NUC_HYBRID;
  if (!p_core_has_smt && has_lp_e_cores)
    return NucGeneration::ARROW_LAKE_H;
  if (p_core_has_smt && has_lp_e_cores)
    return NucGeneration::METEOR_LAKE;
  if (p_core_has_smt && !has_lp_e_cores)
    return NucGeneration::RAPTOR_LAKE_P;
  // Hybrid flag set but no SMT — nearly always BIOS HT disabled on a
  // Raptor-Lake-P class chip. Surface as a distinct value so the shell
  // verifier can FAIL loudly.
  return NucGeneration::RAPTOR_LAKE_P_HT_OFF;
}

// Env-var hint override (§1.3 decision: generation-only, never fabricates
// id lists). A hint on non-hybrid hardware forces the enum but leaves
// is_hybrid at whatever sysfs reported, preserving Validate-layer safety.
inline NucGeneration ApplyEnvHint(NucGeneration detected,
                                  const char *hint) noexcept {
  if (!hint || !*hint)
    return detected;
  const std::string_view h{hint};
  if (h == "raptor_lake_p")
    return NucGeneration::RAPTOR_LAKE_P;
  if (h == "meteor_lake")
    return NucGeneration::METEOR_LAKE;
  if (h == "arrow_lake_h")
    return NucGeneration::ARROW_LAKE_H;
  if (h == "raptor_lake_p_ht_off")
    return NucGeneration::RAPTOR_LAKE_P_HT_OFF;
  if (h == "none")
    return NucGeneration::NOT_NUC_HYBRID;
  return detected; // unknown hint — ignore
}

} // namespace internal::topology

// ── Detection entry points ────────────────────────────────────────────────

// Testable detector. `sysfs_root` is typically "/sys", `proc_cpuinfo_path`
// "/proc/cpuinfo". Tests pass a temporary directory tree. `env_gen_hint`
// is the value of RTC_FORCE_HYBRID_GENERATION (or nullptr).
inline CpuTopology DetectCpuTopology(std::string_view sysfs_root,
                                     std::string_view proc_cpuinfo_path,
                                     const char *env_gen_hint) noexcept {
  namespace fs = std::filesystem;
  using namespace internal::topology;

  CpuTopology t;
  const fs::path root{std::string(sysfs_root)};
  const fs::path cpu_root = root / "devices/system/cpu";
  const fs::path cpuinfo{std::string(proc_cpuinfo_path)};

  // Step 1: enumerate logical CPUs + group by (pkg, core_id).
  std::map<long, std::vector<int>> core_id_to_logicals;
  for (int cpu = 0; cpu < 1024; ++cpu) {
    const fs::path topo = cpu_root / ("cpu" + std::to_string(cpu)) / "topology";
    const fs::path pkg_p = topo / "physical_package_id";
    const fs::path core_p = topo / "core_id";
    std::error_code ec;
    if (!fs::exists(pkg_p, ec))
      break;
    const int pkg = ReadInt(pkg_p);
    const int core = ReadInt(core_p);
    if (pkg < 0 || core < 0)
      break;
    const long key = static_cast<long>(pkg) * 100000L + core;
    core_id_to_logicals[key].push_back(cpu);
    t.num_logical_cores = cpu + 1;
  }
  t.num_physical_cores = static_cast<int>(core_id_to_logicals.size());
  t.smt_active = t.num_logical_cores > t.num_physical_cores;

  if (t.num_logical_cores == 0) {
    // Sysfs unavailable (container). Fall back to sysconf for a usable
    // number; hybrid detection below will stay false.
    const long n = ::sysconf(_SC_NPROCESSORS_ONLN);
    if (n > 0) {
      t.num_logical_cores = static_cast<int>(n);
      t.num_physical_cores = static_cast<int>(n);
    }
  }

  // Step 2a (primary): hybrid classification via intel_core / intel_atom
  //                   sysfs directories (+ optional cpuinfo `hybrid` flag).
  const auto p_cpus_sysfs = ReadCpuListFile(cpu_root / "types/intel_core");
  const auto e_cpus_sysfs = ReadCpuListFile(cpu_root / "types/intel_atom");
  const bool types_present = !p_cpus_sysfs.empty() || !e_cpus_sysfs.empty();
  const bool cpuinfo_hybrid = CpuinfoHasHybridFlag(cpuinfo);
  const bool sysfs_says_hybrid =
      (!p_cpus_sysfs.empty() && !e_cpus_sysfs.empty()) ||
      (types_present && cpuinfo_hybrid);

  std::vector<int> p_cpus;
  std::vector<int> e_cpus;
  if (sysfs_says_hybrid) {
    t.is_hybrid = true;
    t.detect_source = HybridDetectSource::SYSFS_TYPES;
    p_cpus = p_cpus_sysfs;
    e_cpus = e_cpus_sysfs;
  } else {
    // Step 2b (fallback): cpuinfo_max_freq clustering. Fires on kernels
    // that don't export the hybrid topology — typically custom RT builds
    // where CONFIG_INTEL_HFI_THERMAL / SCHED_MC_PRIO / SCHED_CLUSTER were
    // stripped, or pre-6.10 kernels on Meteor/Arrow Lake.
    std::vector<int> online_cpus;
    for (const auto &[key, logicals] : core_id_to_logicals)
      for (const int cpu : logicals)
        online_cpus.push_back(cpu);
    std::sort(online_cpus.begin(), online_cpus.end());
    auto [p_fb, e_fb] = ClusterByMaxFreq(cpu_root, online_cpus);
    if (!p_fb.empty() && !e_fb.empty()) {
      t.is_hybrid = true;
      t.detect_source = HybridDetectSource::CPUFREQ_CLUSTER;
      p_cpus = std::move(p_fb);
      e_cpus = std::move(e_fb);
    }
  }

  // Step 3: derive SMT sibling pairing + E/LP-E split from the chosen split.
  //        Shared helper keeps the two paths byte-identical in output shape.
  if (t.is_hybrid)
    PopulateHybridFromCpus(t, cpu_root, p_cpus, e_cpus);

  // Step 4: classify generation + apply env hint override.
  t.generation =
      ClassifyGeneration(t.is_hybrid, t.p_core_has_smt, t.has_lp_e_cores);
  t.generation = ApplyEnvHint(t.generation, env_gen_hint);
  return t;
}

// Process-wide cached topology. First call may touch disk (sysfs, cpuinfo)
// so it must be invoked before any RT thread starts. Downstream RT code
// reads the cached copy without IO.
inline const CpuTopology &GetCpuTopology() noexcept {
  static const CpuTopology topo = DetectCpuTopology(
      "/sys", "/proc/cpuinfo", std::getenv("RTC_FORCE_HYBRID_GENERATION"));
  return topo;
}

// Stable string for logs and shell bridge. Must stay in sync with the
// bash helper `get_nuc_generation` in rt_common.sh.
inline std::string_view NucGenerationToString(NucGeneration g) noexcept {
  switch (g) {
  case NucGeneration::RAPTOR_LAKE_P:
    return "raptor_lake_p";
  case NucGeneration::METEOR_LAKE:
    return "meteor_lake";
  case NucGeneration::ARROW_LAKE_H:
    return "arrow_lake_h";
  case NucGeneration::RAPTOR_LAKE_P_HT_OFF:
    return "raptor_lake_p_ht_off";
  case NucGeneration::NOT_NUC_HYBRID:
    return "none";
  }
  return "none";
}

inline std::string_view DegradationModeToString(DegradationMode m) noexcept {
  switch (m) {
  case DegradationMode::NONE:
    return "none";
  case DegradationMode::SERIAL_MPC:
    return "serial_mpc";
  }
  return "none";
}

// Matches shell `HYBRID_DETECT_SOURCE` values from rt_common.sh —
// "sysfs_types" / "cpufreq_cluster" / "none". Used for structured logging
// so the two layers speak the same tag.
inline std::string_view
HybridDetectSourceToString(HybridDetectSource s) noexcept {
  switch (s) {
  case HybridDetectSource::NONE:
    return "none";
  case HybridDetectSource::SYSFS_TYPES:
    return "sysfs_types";
  case HybridDetectSource::CPUFREQ_CLUSTER:
    return "cpufreq_cluster";
  }
  return "none";
}

} // namespace rtc

#endif // RTC_BASE_CPU_TOPOLOGY_HPP_
