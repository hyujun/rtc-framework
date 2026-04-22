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

  // Step 2: hybrid classification via intel_core / intel_atom directories.
  const auto p_cpus_raw = ReadCpuListFile(cpu_root / "types/intel_core");
  const auto e_cpus_raw = ReadCpuListFile(cpu_root / "types/intel_atom");
  const bool types_present = !p_cpus_raw.empty() || !e_cpus_raw.empty();
  const bool cpuinfo_hybrid = CpuinfoHasHybridFlag(cpuinfo);

  t.is_hybrid = (!p_cpus_raw.empty() && !e_cpus_raw.empty()) ||
                (types_present && cpuinfo_hybrid);

  if (t.is_hybrid && !p_cpus_raw.empty()) {
    // Step 3-5: P-core grouping by core_id, SMT sibling pairing.
    std::map<int, std::vector<int>> p_core_id_to_cpus;
    for (const int cpu : p_cpus_raw) {
      const int cid = ReadInt(cpu_root / ("cpu" + std::to_string(cpu)) /
                              "topology/core_id");
      if (cid >= 0)
        p_core_id_to_cpus[cid].push_back(cpu);
    }
    for (auto &[cid, cpus] : p_core_id_to_cpus) {
      std::sort(cpus.begin(), cpus.end());
      t.p_core_physical_ids.push_back(cpus.front());
      if (cpus.size() >= 2) {
        t.p_core_sibling_ids.push_back(cpus[1]);
      }
    }
    t.num_p_physical = static_cast<int>(p_core_id_to_cpus.size());
    t.num_p_logical = static_cast<int>(p_cpus_raw.size());
    t.p_core_has_smt =
        (t.num_p_logical > t.num_p_physical) &&
        (t.p_core_sibling_ids.size() == t.p_core_physical_ids.size());

    // Step 3b: E-core vs LP-E-core split using cpuinfo_max_freq 70% rule.
    if (!e_cpus_raw.empty()) {
      std::vector<std::pair<int, int>> e_freq;
      int max_freq = 0;
      e_freq.reserve(e_cpus_raw.size());
      for (const int cpu : e_cpus_raw) {
        const int f = ReadInt(cpu_root / ("cpu" + std::to_string(cpu)) /
                              "cpufreq/cpuinfo_max_freq");
        e_freq.emplace_back(cpu, f);
        if (f > max_freq)
          max_freq = f;
      }
      const int lpe_threshold = max_freq > 0 ? (max_freq * 70 / 100) : 0;
      for (const auto &[cpu, f] : e_freq) {
        if (lpe_threshold > 0 && f > 0 && f < lpe_threshold) {
          t.lpe_core_ids.push_back(cpu);
        } else {
          t.e_core_ids.push_back(cpu);
        }
      }
      t.num_e_cores = static_cast<int>(t.e_core_ids.size());
      t.num_lpe_cores = static_cast<int>(t.lpe_core_ids.size());
      t.has_lp_e_cores = (t.num_lpe_cores > 0);
    }
  }

  // Step 6: classify + env hint override.
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

} // namespace rtc

#endif // RTC_BASE_CPU_TOPOLOGY_HPP_
